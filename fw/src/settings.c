/* Settings in the last 2 KB flash page */
#include "settings.h"

#define SETTINGS_MAGIC   0x434F4941u   /* "AIOC" */
#define SETTINGS_VERSION 1

extern const uint8_t _settings_page[];
settings_t cfg;

void settings_defaults(void)
{
    cfg = (settings_t){
        .magic = SETTINGS_MAGIC, .version = SETTINGS_VERSION, .size = sizeof(settings_t),
        .vol_out = 0, .vol_in = 0, .mute_out = 0, .mute_in = 0,
        .buf_ms = 2, .dc_block = 1,
        /* CHIRP raises DTR and RTS together, so it must not key the radio */
        .ptt_src = { PTT_SRC_DTR_NRTS, PTT_SRC_OFF },
        .ptt_vox = { 0, 0 },
        .tot_s = 0,
        .vox_thr = -30, .vox_hang = 500,
        .led_en = 1, .sleep_en = 1,
    };
}

static uint32_t crc32(const void *data, size_t n)
{
    const uint8_t *p = data;
    uint32_t c = 0xFFFFFFFFu;
    while (n--) {
        c ^= *p++;
        for (int k = 0; k < 8; k++)
            c = (c >> 1) ^ (0xEDB88320u & -(c & 1));
    }
    return ~c;
}

bool settings_load(void)
{
    const settings_t *f = (const settings_t *)_settings_page;
    if (f->magic == SETTINGS_MAGIC && f->version == SETTINGS_VERSION &&
        f->size == sizeof(settings_t) &&
        f->crc == crc32(f, offsetof(settings_t, crc))) {
        cfg = *f;
        return true;
    }
    settings_defaults();
    return false;
}

/* Instruction fetches stall while the flash is busy, so by the time the
 * next instruction runs the operation is over: check the result once. */
static bool flash_done(void)
{
    __DSB();
    uint32_t sr = FLASH->SR;
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPERR;
    return !(sr & (FLASH_SR_BSY | FLASH_SR_PGERR | FLASH_SR_WRPERR));
}

bool settings_save(void)
{
    cfg.magic = SETTINGS_MAGIC;
    cfg.version = SETTINGS_VERSION;
    cfg.size = sizeof(settings_t);
    cfg.crc = crc32(&cfg, offsetof(settings_t, crc));

    bool ok = true;
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;

    FLASH->CR = FLASH_CR_PER;
    FLASH->AR = (uint32_t)_settings_page;
    FLASH->CR = FLASH_CR_PER | FLASH_CR_STRT;
    ok = flash_done();
    FLASH->CR = 0;

    if (ok) {
        FLASH->CR = FLASH_CR_PG;
        const uint16_t *src = (const uint16_t *)&cfg;
        volatile uint16_t *dst = (volatile uint16_t *)_settings_page;
        for (size_t i = 0; i < sizeof(settings_t) / 2 && ok; i++) {
            dst[i] = src[i];
            ok = flash_done();
        }
        FLASH->CR = 0;
    }
    FLASH->CR = FLASH_CR_LOCK;
    return ok && settings_load();
}
