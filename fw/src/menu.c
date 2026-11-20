/* Terminal menu on the config port (VT100/ANSI), plus a line-mode command
 * interface for scripting (':' in the menu).
 *
 * Keys: Up/Down select an entry, Right/Enter opens a submenu, runs an action
 * or starts editing a value (then Up/Down change it, Left/Right/Enter/Esc
 * end editing), Left/Esc go back, +/- change a value without editing mode,
 * PgUp/PgDn change in big steps,
 * Esc goes back, S saves, Ctrl-L redraws.
 *
 * Runs in PendSV (lowest priority). Input arrives from the USB ISR. */
#include <math.h>
#include <string.h>
#include "menu.h"
#include "cdc.h"
#include "settings.h"
#include "audio.h"
#include "ptt.h"
#include "uart.h"
#include "usb.h"
#include "dfu.h"

/* ---- output ------------------------------------------------------------- */
static char obuf[128];
static uint16_t olen, ocol;

static void out_flush(void)
{
    if (olen)
        cfg_write(obuf, olen);
    olen = 0;
}

static void out_ch(char c)
{
    if (olen == sizeof obuf)
        out_flush();
    obuf[olen++] = c;
    if (c == '\n' || c == '\r')
        ocol = 0;
    else if (c != '\x1b')
        ocol++;
}

static void out_str(const char *s) { while (*s) out_ch(*s++); }
/* escape sequences don't take columns */
static void out_esc(const char *s) { uint16_t c = ocol; out_str(s); ocol = c; }
static void out_pad(uint16_t col) { while (ocol < col) out_ch(' '); }
static void out_eol(void) { out_esc("\x1b[K"); out_str("\r\n"); }

static void out_uint(uint32_t v)
{
    char t[11];
    int n = 0;
    do { t[n++] = (char)('0' + v % 10); v /= 10; } while (v);
    while (n) out_ch(t[--n]);
}

static void out_int(int32_t v)
{
    if (v < 0) { out_ch('-'); out_uint((uint32_t)-v); }
    else out_uint((uint32_t)v);
}

/* v / div with one decimal (div > 1), rounded */
static void out_num(int32_t v, uint16_t div)
{
    if (div <= 1) { out_int(v); return; }
    int32_t t = (v * 10 + (v < 0 ? -(int32_t)div / 2 : (int32_t)div / 2)) / div;
    if (t < 0) { out_ch('-'); t = -t; }
    out_uint((uint32_t)t / 10);
    out_ch('.');
    out_uint((uint32_t)t % 10);
}

static void out_hex(uint32_t v, int digits)
{
    while (digits--) out_ch("0123456789ABCDEF"[(v >> (4 * digits)) & 15]);
}

static float peak_db(uint16_t peak)
{
    return peak ? 20.0f * log10f((float)peak / 32767.0f) : -99.0f;
}

/* level meter: -60..0 dBFS */
static void out_meter(uint16_t peak)
{
    float db = peak_db(peak);
    int n = (int)((db + 60.0f) / 2.0f + 0.5f);
    if (n < 0) n = 0;
    if (n > 30) n = 30;
    out_ch('[');
    for (int i = 0; i < 30; i++)
        out_ch(i < n ? (i >= 27 ? '!' : '#') : '.');
    out_str("] ");
    if (peak) { out_num((int32_t)(db * 10.0f), 10); out_str(" dBFS"); }
    else out_str("-inf");
}

/* ---- items -------------------------------------------------------------- */
enum { T_SUB, T_ACT, T_NUM, T_BOOL, T_CHOICE, T_INFO };
enum { K_UP = 0x100, K_DOWN, K_RIGHT, K_LEFT, K_PGUP, K_PGDN, K_ESC };
enum { V_U8, V_I16, V_U16 };

typedef struct page page_t;
typedef struct {
    const char *label;
    const char *key;               /* command-line name, NULL = none */
    uint8_t type, vt;
    void *ptr;
    int32_t (*get)(void);          /* instead of ptr */
    void (*set)(int32_t);
    int32_t min, max, step, big;
    uint16_t div;                  /* display scale, 1 = integer */
    const char *unit;
    const char *const *names;      /* T_CHOICE */
    const page_t *sub;
    void (*act)(void);
    bool confirm;
    void (*info)(void);            /* T_INFO: draws the value column */
    void (*apply)(void);           /* after a change */
} item_t;

struct page {
    const char *title;
    const item_t *items;
    uint8_t n;
    bool live;                     /* redraw ~10 times a second */
};

static int32_t item_get(const item_t *it)
{
    if (it->get) return it->get();
    switch (it->vt) {
    case V_I16: return *(int16_t *)it->ptr;
    case V_U16: return *(uint16_t *)it->ptr;
    default:    return *(uint8_t *)it->ptr;
    }
}

static void item_set(const item_t *it, int32_t v)
{
    int32_t max = it->type == T_BOOL ? 1 : it->type == T_CHOICE ? it->max - 1 : it->max;
    int32_t min = it->type == T_NUM ? it->min : 0;
    if (v < min) v = min;
    if (v > max) v = max;
    if (it->set) it->set(v);
    else switch (it->vt) {
    case V_I16: *(int16_t *)it->ptr = (int16_t)v; break;
    case V_U16: *(uint16_t *)it->ptr = (uint16_t)v; break;
    default:    *(uint8_t *)it->ptr = (uint8_t)v; break;
    }
    if (it->apply) it->apply();
}

static void item_value(const item_t *it)
{
    switch (it->type) {
    case T_SUB:  out_str(">"); break;
    case T_NUM:
        out_num(item_get(it), it->div);
        if (it->unit) { out_ch(' '); out_str(it->unit); }
        break;
    case T_BOOL: out_str(item_get(it) ? "on" : "off"); break;
    case T_CHOICE: out_str(it->names[item_get(it)]); break;
    case T_INFO: if (it->info) it->info(); break;
    }
}

static bool selectable(const item_t *it) { return it->type != T_INFO; }

/* ---- page contents ------------------------------------------------------ */
static audio_stat_t st_out, st_in;   /* fetched once per draw */
static bool settings_from_flash;

static void act_save(void);
static void act_defaults(void);
static void act_dfu(void) { out_str("\x1b[2J\x1b[H\x1b[?25hEntering DFU bootloader...\r\n"); dfu_request(); }
static void act_reset(void) { out_str("\x1b[2J\x1b[H\x1b[?25hResetting...\r\n"); reset_request(); }

static void info_rates(void)
{
    out_str("out "); out_uint(st_out.fs); out_str(" x"); out_num((int32_t)(st_out.osr * 10.0f), 10);
    out_str(st_out.active ? " (on)" : " (off)");
    out_str(", in "); out_uint(st_in.fs); out_str(" x"); out_num((int32_t)(st_in.osr * 10.0f), 10);
    out_str(st_in.active ? " (on)" : " (off)");
}

static void info_ptt(void)
{
    out_str("PTT1 "); out_str(ptt_state(0) ? "ON " : "off");
    out_str("  PTT2 "); out_str(ptt_state(1) ? "ON " : "off");
    if (ptt_tot_tripped()) out_str("  TIME-OUT");
}

static void info_vox(void)
{
    out_meter(vox_level());
    out_str(vox_active() ? "  VOX ON" : "");
}

static void info_coding(void)
{
    const uint8_t *c = radio_line_coding();
    out_uint(c[0] | c[1] << 8 | (uint32_t)c[2] << 16 | (uint32_t)c[3] << 24);
    out_ch(' ');
    out_uint(c[6]);
    out_ch("NOEMS"[c[5] < 5 ? c[5] : 0]);
    out_str(c[4] == 0 ? "1" : c[4] == 1 ? "1.5" : "2");
}

static void info_lines(void)
{
    uint8_t l = ptt_lines();
    out_str("DTR "); out_str(l & 1 ? "1" : "0");
    out_str("  RTS "); out_str(l & 2 ? "1" : "0");
}

static void info_bytes(void)
{
    out_str("to radio "); out_uint(uart_tx_bytes);
    out_str(", from radio "); out_uint(uart_rx_bytes);
}

static void info_version(void) { out_str(FW_VERSION " (" __DATE__ ")"); }
static void info_serial(void) { out_str(usb_serial()); }
static void info_chip(void)
{
    out_str("DEV_ID 0x"); out_hex(DBGMCU->IDCODE & 0xFFF, 3);
    out_str(" REV 0x"); out_hex(DBGMCU->IDCODE >> 16, 4);
    out_str(", flash "); out_uint(*(volatile uint16_t *)FLASHSIZE_BASE); out_str(" KB");
}
static void info_settings(void) { out_str(settings_from_flash ? "loaded from flash" : "defaults (not saved)"); }

static void info_tx_level(void) { out_meter(st_out.peak); }
static void info_rx_level(void) { out_meter(st_in.peak); }

static void info_stream(const audio_stat_t *s)
{
    if (!s->active) { out_str("idle, "); out_uint(s->fs); out_str(" Hz x"); out_num((int32_t)(s->osr * 10.0f), 10); return; }
    out_uint(s->fs); out_str(" Hz x"); out_num((int32_t)(s->osr * 10.0f), 10); out_str(", fill "); out_int(s->fill);
    out_str(", corr "); out_num((int32_t)(s->ppm * 10.0f), 10);
    out_str(" ppm, xruns "); out_uint(s->xruns);
}
static void info_out_stream(void) { info_stream(&st_out); }
static void info_in_stream(void) { info_stream(&st_in); }

static void info_clock(void)
{
    out_str("host frame vs. crystal ");
    out_num((int32_t)(audio_frame_ppm() * 10.0f), 10);
    out_str(" ppm");
}

static void info_dsp(void)
{
    out_num((int32_t)(audio_dsp_load() * 10.0f), 10);
    out_str(" % CPU");
}

static void info_usb(void)
{
    out_str(usb_suspended ? "suspended" : usb_configured ? "configured" : "not configured");
}

static int32_t get_man1(void) { return ptt_manual_get(0); }
static int32_t get_man2(void) { return ptt_manual_get(1); }
static void set_man1(int32_t v) { ptt_manual(0, v); }
static void set_man2(int32_t v) { ptt_manual(1, v); }

static const char *const lp_names[LP_COUNT] = { "off", "3 kHz", "4 kHz", "8 kHz", "16 kHz" };

static void info_lp(void)
{
    out_str("TX "); out_str(audio_filter_active(false) ? "active" : cfg.lp_tx ? "bypassed (above fs/2)" : "off");
    out_str(", RX "); out_str(audio_filter_active(true) ? "active" : cfg.lp_rx ? "bypassed (above fs/2)" : "off");
}

static const char *const ptt_names[PTT_SRC_COUNT] = { "off", "DTR", "RTS", "DTR and not RTS", "RTS and not DTR" };

#define VOL(lbl, k, p) { .label = lbl, .key = k, .type = T_NUM, .vt = V_I16, .ptr = p, \
    .min = VOL_MIN, .max = VOL_MAX, .step = VOL_RES, .big = 5 * 256, .div = 256, .unit = "dB", \
    .apply = audio_gain_update }
#define BOOL(lbl, k, p, ap) { .label = lbl, .key = k, .type = T_BOOL, .vt = V_U8, .ptr = p, .apply = ap }
#define INFO(lbl, fn) { .label = lbl, .type = T_INFO, .info = fn }

static const item_t audio_items[] = {
    VOL("TX level (USB -> radio)", "tx_level", &cfg.vol_out),
    BOOL("TX mute", "tx_mute", &cfg.mute_out, audio_gain_update),
    VOL("RX level (radio -> USB)", "rx_level", &cfg.vol_in),
    BOOL("RX mute", "rx_mute", &cfg.mute_in, audio_gain_update),
    BOOL("RX DC blocker", "dc_block", &cfg.dc_block, NULL),
    BOOL("Oversampling", "oversample", &cfg.oversample, audio_reconfigure),
    { .label = "TX low-pass (USB -> radio)", .key = "lp_tx", .type = T_CHOICE, .vt = V_U8,
      .ptr = &cfg.lp_tx, .max = LP_COUNT, .names = lp_names, .apply = audio_filter_update },
    { .label = "RX low-pass (radio -> USB)", .key = "lp_rx", .type = T_CHOICE, .vt = V_U8,
      .ptr = &cfg.lp_rx, .max = LP_COUNT, .names = lp_names, .apply = audio_filter_update },
    INFO("Low-pass state", info_lp),
    { .label = "Buffer (latency)", .key = "buffer", .type = T_NUM, .vt = V_U8, .ptr = &cfg.buf_ms,
      .min = 1, .max = 8, .step = 1, .big = 2, .div = 1, .unit = "ms", .apply = audio_resync },
    INFO("Sample rates (Hz) x OSR", info_rates),
    INFO("Filter CPU load", info_dsp),
};

static const item_t ptt_items[] = {
    { .label = "PTT1 (sleeve) serial source", .key = "ptt1", .type = T_CHOICE, .vt = V_U8,
      .ptr = &cfg.ptt_src[0], .max = PTT_SRC_COUNT, .names = ptt_names, .apply = ptt_update },
    BOOL("PTT1 by VOX", "ptt1_vox", &cfg.ptt_vox[0], ptt_update),
    { .label = "PTT2 (tip) serial source", .key = "ptt2", .type = T_CHOICE, .vt = V_U8,
      .ptr = &cfg.ptt_src[1], .max = PTT_SRC_COUNT, .names = ptt_names, .apply = ptt_update },
    BOOL("PTT2 by VOX", "ptt2_vox", &cfg.ptt_vox[1], ptt_update),
    { .label = "Time-out (0 = off)", .key = "tot", .type = T_NUM, .vt = V_U16, .ptr = &cfg.tot_s,
      .min = 0, .max = 600, .step = 5, .big = 60, .div = 1, .unit = "s" },
    { .label = "Manual PTT1", .type = T_BOOL, .get = get_man1, .set = set_man1 },
    { .label = "Manual PTT2", .type = T_BOOL, .get = get_man2, .set = set_man2 },
    INFO("State", info_ptt),
};

static const item_t vox_items[] = {
    { .label = "Threshold", .key = "vox_thr", .type = T_NUM, .vt = V_I16, .ptr = &cfg.vox_thr,
      .min = -60, .max = 0, .step = 1, .big = 6, .div = 1, .unit = "dBFS", .apply = vox_thr_update },
    { .label = "Hang time", .key = "vox_hang", .type = T_NUM, .vt = V_U16, .ptr = &cfg.vox_hang,
      .min = 0, .max = 5000, .step = 50, .big = 500, .div = 1, .unit = "ms" },
    INFO("Level", info_vox),
};

static const item_t serial_items[] = {
    INFO("Line coding (set by host)", info_coding),
    INFO("Control lines", info_lines),
    INFO("Bytes", info_bytes),
};

static const item_t system_items[] = {
    BOOL("Status LED", "led", &cfg.led_en, ptt_update),
    INFO("Firmware", info_version),
    INFO("Serial number", info_serial),
    INFO("Chip", info_chip),
    INFO("Settings", info_settings),
    { .label = "Enter DFU bootloader", .type = T_ACT, .act = act_dfu, .confirm = true },
    { .label = "Reset", .type = T_ACT, .act = act_reset, .confirm = true },
};

static const item_t status_items[] = {
    INFO("TX level (USB -> radio)", info_tx_level),
    INFO("RX level (radio -> USB)", info_rx_level),
    INFO("TX stream", info_out_stream),
    INFO("RX stream", info_in_stream),
    INFO("Clock", info_clock),
    INFO("Filters (oversampling)", info_dsp),
    INFO("PTT", info_ptt),
    INFO("VOX", info_vox),
    INFO("Radio port", info_lines),
    INFO("USB", info_usb),
};

#define PAGE(t, items, l) { t, items, ARRAY_LEN(items), l }
static const page_t audio_page  = PAGE("Audio", audio_items, true);
static const page_t ptt_page    = PAGE("PTT", ptt_items, true);
static const page_t vox_page    = PAGE("VOX", vox_items, true);
static const page_t serial_page = PAGE("Serial (radio port)", serial_items, true);
static const page_t system_page = PAGE("System", system_items, false);
static const page_t status_page = PAGE("Live status", status_items, true);

static const item_t main_items[] = {
    { .label = "Audio", .type = T_SUB, .sub = &audio_page },
    { .label = "PTT", .type = T_SUB, .sub = &ptt_page },
    { .label = "VOX", .type = T_SUB, .sub = &vox_page },
    { .label = "Serial (radio port)", .type = T_SUB, .sub = &serial_page },
    { .label = "System", .type = T_SUB, .sub = &system_page },
    { .label = "Live status", .type = T_SUB, .sub = &status_page },
    { .label = "Save settings", .type = T_ACT, .act = act_save },
    { .label = "Load defaults", .type = T_ACT, .act = act_defaults, .confirm = true },
};
static const page_t main_page = PAGE("Main", main_items, false);

static const page_t *const all_pages[] = { &audio_page, &ptt_page, &vox_page, &system_page };

/* ---- menu state --------------------------------------------------------- */
enum { M_CLOSED, M_MENU, M_CMD };
static uint8_t mode;
static const page_t *page = &main_page;
static uint8_t sel;
static const page_t *stack[4];
static uint8_t sel_stack[4];
static uint8_t depth;
static const item_t *armed;          /* action waiting for a second Enter */
static const char *msg;
static bool dirty, clear_req;
static bool editing;                 /* Up/Down change the selected value */

static volatile bool open_req, close_req, refresh_req;
static uint16_t sof_count;

static void first_selectable(void)
{
    sel = 0;
    while (sel < page->n && !selectable(&page->items[sel]))
        sel++;
}

static void act_save(void)
{
    bool ok = settings_save();
    audio_resync();
    settings_from_flash = ok;
    msg = ok ? "Settings saved." : "Saving FAILED.";
}

static void act_defaults(void)
{
    settings_defaults();
    audio_gain_update();
    audio_filter_update();
    vox_thr_update();
    ptt_update();
    audio_resync();
    msg = "Defaults loaded (not saved yet).";
}

static void draw(void)
{
    if (page->live) {
        audio_stat(false, &st_out);
        audio_stat(true, &st_in);
    }
    out_esc(clear_req ? "\x1b[2J\x1b[H\x1b[?25l" : "\x1b[H\x1b[?25l");
    clear_req = false;
    out_esc("\x1b[1m");
    out_str(" aioc-slop " FW_VERSION);
    out_esc("\x1b[0m");
    out_str("   ");
    for (uint8_t i = 0; i < depth; i++) {
        out_str(stack[i]->title);
        out_str(" > ");
    }
    out_str(page->title);
    out_eol();
    out_eol();
    for (uint8_t i = 0; i < page->n; i++) {
        const item_t *it = &page->items[i];
        bool s = i == sel && selectable(it);
        if (s && editing) {
            /* edit mode: the value is highlighted, not the line */
            out_str(" > ");
            out_str(it->label);
            out_pad(33);
            out_esc("\x1b[7m");
            out_ch(' ');
            item_value(it);
            out_str(" ");
            out_esc("\x1b[0m");
        } else {
            if (s) out_esc("\x1b[7m");
            out_str(s ? " > " : "   ");
            out_str(it->label);
            out_pad(34);
            item_value(it);
            if (s) { out_pad(64); out_esc("\x1b[0m"); }
        }
        out_eol();
    }
    out_eol();
    if (armed) out_str(" Press Right/Enter again to confirm, any other key cancels.");
    else if (msg) { out_str(" "); out_str(msg); }
    out_eol();
    out_esc("\x1b[2m");
    if (editing) {
        out_str(" Up/Down or +/- change  PgUp/PgDn big steps  Left/Right/Enter/Esc done");
        out_eol();
        out_str(" S save");
    } else {
        out_str(" Up/Down select  Right/Enter open/edit/run  Left/Esc back");
        out_eol();
        out_str(" +/- quick change  PgUp/PgDn big steps  S save  Ctrl-L redraw  : command line");
    }
    out_esc("\x1b[0m");
    out_eol();
    out_esc("\x1b[J");
}

static void change(const item_t *it, int dir, bool big)
{
    int32_t v = item_get(it);
    switch (it->type) {
    case T_NUM:    item_set(it, v + dir * (big ? it->big : it->step)); break;
    case T_BOOL:   item_set(it, !v); break;
    case T_CHOICE: item_set(it, (v + dir + it->max) % it->max); break;
    }
}

/* ---- command line ------------------------------------------------------- */
static char line[64];
static uint8_t llen;

static const item_t *find_key(const char *k)
{
    for (size_t p = 0; p < ARRAY_LEN(all_pages); p++)
        for (uint8_t i = 0; i < all_pages[p]->n; i++) {
            const item_t *it = &all_pages[p]->items[i];
            if (it->key && !strcmp(it->key, k))
                return it;
        }
    return NULL;
}

static void show_item(const item_t *it)
{
    out_str(it->key);
    out_str(" = ");
    item_value(it);
    out_str("\r\n");
}

static char lower(char c) { return (c >= 'A' && c <= 'Z') ? (char)(c + 32) : c; }

static bool streq_i(const char *a, const char *b)
{
    while (*a && lower(*a) == lower(*b)) { a++; b++; }
    return !*a && !*b;
}

/* decimal with optional one-digit fraction -> tenths */
static bool parse_tenths(const char *s, int32_t *out)
{
    bool neg = false;
    int32_t v = 0, frac = 0;
    if (*s == '-' || *s == '+') neg = *s++ == '-';
    if (!*s) return false;
    while (*s >= '0' && *s <= '9') v = v * 10 + (*s++ - '0');
    if (*s == '.') {
        s++;
        if (*s >= '0' && *s <= '9') frac = *s++ - '0';
        while (*s >= '0' && *s <= '9') s++;
    }
    if (*s) return false;
    v = v * 10 + frac;
    *out = neg ? -v : v;
    return true;
}

static bool set_from_text(const item_t *it, const char *val)
{
    int32_t t;
    switch (it->type) {
    case T_BOOL:
        if (streq_i(val, "on") || streq_i(val, "1") || streq_i(val, "true")) { item_set(it, 1); return true; }
        if (streq_i(val, "off") || streq_i(val, "0") || streq_i(val, "false")) { item_set(it, 0); return true; }
        return false;
    case T_CHOICE:
        for (int32_t i = 0; i < it->max; i++)
            if (streq_i(val, it->names[i])) { item_set(it, i); return true; }
        if (parse_tenths(val, &t) && t % 10 == 0 && t / 10 >= 0 && t / 10 < it->max) {
            item_set(it, t / 10);
            return true;
        }
        return false;
    case T_NUM:
        if (!parse_tenths(val, &t)) return false;
        t = t * it->div / 10;
        if (it->step > 1) t = t / it->step * it->step;
        if (t < it->min || t > it->max) return false;
        item_set(it, t);
        return true;
    }
    return false;
}

static void cmd_prompt(void) { out_str("aioc> "); }

static void cmd_help(void)
{
    out_str("Commands: show | get <key> | set <key> <value> | save | defaults |\r\n"
            "          status | dfu | reset | menu (or Esc)\r\n");
}

static void cmd_run(char *l)
{
    char *argv[3] = { 0 };
    int argc = 0;
    for (char *p = l; *p && argc < 3; ) {
        while (*p == ' ') *p++ = 0;
        if (!*p) break;
        argv[argc++] = p;
        if (argc == 3) break;                    /* the rest is the value */
        while (*p && *p != ' ') p++;
    }
    if (argc == 0) return;
    const char *c = argv[0];

    if (streq_i(c, "help") || streq_i(c, "?")) cmd_help();
    else if (streq_i(c, "show")) {
        for (size_t p = 0; p < ARRAY_LEN(all_pages); p++)
            for (uint8_t i = 0; i < all_pages[p]->n; i++)
                if (all_pages[p]->items[i].key) show_item(&all_pages[p]->items[i]);
    } else if (streq_i(c, "get") && argc == 2) {
        const item_t *it = find_key(argv[1]);
        if (it) show_item(it); else out_str("unknown key\r\n");
    } else if (streq_i(c, "set") && argc == 3) {
        const item_t *it = find_key(argv[1]);
        if (!it) out_str("unknown key\r\n");
        else if (!set_from_text(it, argv[2])) out_str("bad value\r\n");
        else show_item(it);
    } else if (streq_i(c, "save")) {
        act_save(); out_str(msg); out_str("\r\n"); msg = NULL;
    } else if (streq_i(c, "defaults")) {
        act_defaults(); out_str(msg); out_str("\r\n"); msg = NULL;
    } else if (streq_i(c, "status")) {
        audio_stat(false, &st_out);
        audio_stat(true, &st_in);
        for (uint8_t i = 0; i < status_page.n; i++) {
            out_str(status_page.items[i].label);
            out_str(": ");
            status_page.items[i].info();
            out_str("\r\n");
        }
    } else if (streq_i(c, "dfu")) act_dfu();
    else if (streq_i(c, "reset")) act_reset();
    else if (streq_i(c, "menu") || streq_i(c, "exit")) {
        mode = M_MENU; clear_req = dirty = true; return;
    } else out_str("unknown command, try help\r\n");
}

static void cmd_key(int k)
{
    if (k == K_ESC) {
        mode = M_MENU; clear_req = dirty = true;
        return;
    }
    if (k == '\r' || k == '\n') {
        out_str("\r\n");
        line[llen] = 0;
        llen = 0;
        cmd_run(line);
        if (mode == M_CMD) cmd_prompt();
    } else if ((k == 0x7F || k == 0x08) && llen) {
        llen--;
        out_str("\b \b");
    } else if (k >= 0x20 && k < 0x7F && llen < sizeof line - 1) {
        line[llen++] = (char)k;
        out_ch((char)k);
    }
}

/* ---- key handling ------------------------------------------------------- */

static void menu_key(int k)
{
    const item_t *it = sel < page->n ? &page->items[sel] : NULL;
    if (it && !selectable(it)) it = NULL;
    const item_t *was_armed = armed;
    armed = NULL;
    msg = NULL;
    dirty = true;

    if (editing) {
        switch (k) {
        case K_UP:   case '+': change(it, +1, false); return;
        case K_DOWN: case '-': change(it, -1, false); return;
        case K_PGUP: change(it, +1, true); return;
        case K_PGDN: change(it, -1, true); return;
        case K_LEFT: case K_RIGHT: case K_ESC: case '\r': case '\n':
            editing = false;
            return;
        case 'S': case 's':
            act_save();
            return;
        }
        return;
    }

    switch (k) {
    case K_UP:
    case K_DOWN:
        for (uint8_t i = 0; i < page->n; i++) {
            sel = (uint8_t)((sel + (k == K_DOWN ? 1 : page->n - 1)) % page->n);
            if (selectable(&page->items[sel])) break;
        }
        break;
    case '+': if (it) change(it, +1, false); break;
    case '-': if (it) change(it, -1, false); break;
    case K_PGUP: if (it) change(it, +1, true); break;
    case K_PGDN: if (it) change(it, -1, true); break;
    case K_RIGHT: case '\r': case '\n':
        if (!it) break;
        if (it->type == T_SUB && depth < ARRAY_LEN(stack)) {
            stack[depth] = page;
            sel_stack[depth++] = sel;
            page = it->sub;
            first_selectable();
            clear_req = true;
        } else if (it->type == T_ACT) {
            if (it->confirm && was_armed != it) armed = it;
            else it->act();
        } else {
            editing = true;                      /* number, on/off, choice */
        }
        break;
    case K_LEFT: case K_ESC: case 0x7F: case 0x08: case 'q':
        if (depth) {
            page = stack[--depth];
            sel = sel_stack[depth];
            clear_req = true;
        }
        break;
    case 'S': case 's':
        act_save();
        break;
    case 0x0C:                                   /* Ctrl-L */
        clear_req = true;
        break;
    case ':':
        mode = M_CMD;
        dirty = false;
        llen = 0;
        out_esc("\x1b[2J\x1b[H\x1b[?25h");
        cmd_help();
        cmd_prompt();
        break;
    }
}

static uint8_t esc_state;
static uint16_t csi_num;

static void key(int k)
{
    if (mode == M_MENU) menu_key(k);
    else if (mode == M_CMD) cmd_key(k);
}

static void key_in(uint8_t c)
{
    switch (esc_state) {
    case 0:
        if (c == 0x1B) { esc_state = 1; return; }
        if (c) key(c);
        return;
    case 1:
        if (c == '[' || c == 'O') { esc_state = 2; csi_num = 0; return; }
        esc_state = 0;
        key(K_ESC);
        if (c) key_in(c);
        return;
    default:
        if (c >= '0' && c <= '9') { csi_num = (uint16_t)(csi_num * 10 + (c - '0')); return; }
        if (c == ';') return;
        esc_state = 0;
        switch (c) {
        case 'A': key(K_UP); break;
        case 'B': key(K_DOWN); break;
        case 'C': key(K_RIGHT); break;
        case 'D': key(K_LEFT); break;
        case '~':
            if (csi_num == 5) key(K_PGUP);
            else if (csi_num == 6) key(K_PGDN);
            break;
        }
    }
}

/* ---- events ------------------------------------------------------------- */
void menu_dtr(bool on)
{
    if (on) open_req = true;
    else close_req = true;
    menu_pend();
}

void menu_sof(void)
{
    if (++sof_count >= 100) {
        sof_count = 0;
        if (mode == M_MENU && page->live) {
            refresh_req = true;
            menu_pend();
        }
    }
}

void menu_init(void)
{
    NVIC_SetPriority(PendSV_IRQn, PRIO_MENU);
    settings_from_flash = settings_load();
}

void PendSV_Handler(void)
{
    if (close_req) {
        close_req = false;
        mode = M_CLOSED;
        cfg_flush();
    }
    if (open_req) {
        open_req = false;
        mode = M_MENU;
        page = &main_page;
        depth = 0;
        first_selectable();
        armed = NULL;
        msg = NULL;
        editing = false;
        esc_state = 0;
        clear_req = dirty = true;
    }
    uint8_t c;
    while (cfg_getc(&c))
        if (mode != M_CLOSED) key_in(c);
    if (refresh_req) {
        refresh_req = false;
        if (mode == M_MENU && page->live) dirty = true;
    }
    if (dirty && mode == M_MENU) {
        dirty = false;
        draw();
    }
    out_flush();
    dfu_run_pending();
}
