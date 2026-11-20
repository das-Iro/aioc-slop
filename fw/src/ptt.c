/* PTT outputs, VOX, PTT time-out and the status LED */
#include <math.h>
#include "ptt.h"
#include "settings.h"
#include "usb.h"

/* Which LED lights when PB8 is high and PB9 low. Swap if the colours are
 * the wrong way round on the board. */
#define LED_GREEN_A_HIGH 1

static volatile uint8_t lines;
static volatile bool manual[2];
static volatile bool vox_on, tot_trip;
static volatile bool out_state[2];
static uint32_t vox_env, vox_thr_lin;
static uint16_t vox_hold;
static uint32_t tot_ms;

static bool src_active(uint8_t src)
{
    bool dtr = lines & 1, rts = lines & 2;
    switch (src) {
    case PTT_SRC_DTR:      return dtr;
    case PTT_SRC_RTS:      return rts;
    case PTT_SRC_DTR_NRTS: return dtr && !rts;
    case PTT_SRC_RTS_NDTR: return rts && !dtr;
    }
    return false;
}

static void led(int c)   /* 0 off, 1 green, 2 red */
{
    uint32_t a = 1u << LED_A_PIN, b = 1u << LED_B_PIN;
    if (c == 0 || !cfg.led_en)
        GPIOB->BRR = a | b;
    else if ((c == 1) == LED_GREEN_A_HIGH)
        GPIOB->BSRR = a | (b << 16);
    else
        GPIOB->BSRR = b | (a << 16);
}

void ptt_update(void)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    bool up = usb_configured && !usb_suspended;
    bool want[2];
    for (int i = 0; i < 2; i++)
        want[i] = up && (src_active(cfg.ptt_src[i]) || (cfg.ptt_vox[i] && vox_on) || manual[i]);
    if (!want[0] && !want[1]) {
        tot_trip = false;
        tot_ms = 0;
    }
    if (tot_trip)
        want[0] = want[1] = false;
    GPIOA->BSRR = want[0] ? (1u << PTT1_PIN) : (1u << (PTT1_PIN + 16));
    GPIOA->BSRR = want[1] ? (1u << PTT2_PIN) : (1u << (PTT2_PIN + 16));
    out_state[0] = want[0];
    out_state[1] = want[1];
    led((want[0] || want[1]) ? 2 : up ? 1 : 0);
    __set_PRIMASK(pm);
}

void ptt_init(void)
{
    vox_thr_update();
    ptt_update();
}

void ptt_set_lines(uint8_t l) { lines = l & 3; ptt_update(); }
uint8_t ptt_lines(void) { return lines; }
void ptt_manual(int n, bool on) { manual[n] = on; ptt_update(); }
bool ptt_manual_get(int n) { return manual[n]; }
bool ptt_state(int n) { return out_state[n]; }
bool vox_active(void) { return vox_on; }
bool ptt_tot_tripped(void) { return tot_trip; }
uint16_t vox_level(void) { return (uint16_t)MIN(vox_env, 32767u); }

void vox_thr_update(void)
{
    vox_thr_lin = (uint32_t)(32767.0f * powf(10.0f, (float)cfg.vox_thr / 20.0f));
}

void vox_feed(uint32_t peak)
{
    if (peak > vox_env)
        vox_env = peak;
}

void ptt_sof(void)
{
    bool was = vox_on;
    if (vox_env >= vox_thr_lin && vox_env) {
        vox_on = true;
        vox_hold = cfg.vox_hang;
    } else if (vox_hold) {
        vox_hold--;
    } else {
        vox_on = false;
    }
    vox_env -= vox_env >> 4;            /* ~0.56 dB per ms */
    if (vox_on != was && (cfg.ptt_vox[0] || cfg.ptt_vox[1]))
        ptt_update();

    if ((out_state[0] || out_state[1]) && cfg.tot_s && !tot_trip) {
        if (++tot_ms >= (uint32_t)cfg.tot_s * 1000u) {
            tot_trip = true;
            ptt_update();
        }
    }
}
