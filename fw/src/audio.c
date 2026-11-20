/* USB audio class 1, synchronous endpoints.
 *
 * host -> radio: iso OUT -> ring -> DMA2 Ch3 -> DAC1 CH1 (PA4), paced by TIM6
 * radio -> host: ADC2 IN12 (PB2), paced by TIM3 -> DMA2 Ch1 -> ring -> iso IN
 *
 * Both sample clocks are locked to the host: the USB SOF interrupt
 * timestamps a free-running 72 MHz counter (TIM2) to measure the host's
 * frame period, which sets the timer periods (feed-forward). A slow PI loop
 * on each ring's fill level removes what is left. The fractional part of
 * the period is dithered once per frame. */
#include <math.h>
#include "audio.h"
#include "settings.h"
#include "ptt.h"
#include "clock.h"

#define RING_N   512u            /* samples, power of two */
#define RING_M   (RING_N - 1u)

static uint16_t dac_ring[RING_N];
static uint16_t adc_ring[RING_N];

static const uint32_t rates[] = { 8000, 11025, 16000, 22050, 24000, 32000, 44100, 48000 };

typedef struct {
    TIM_TypeDef *tim;
    DMA_Channel_TypeDef *dma;
    uint16_t *ring;
    uint32_t fs;
    volatile bool active;
    bool     sync;              /* next packet re-establishes the target fill */
    uint32_t hw;                /* samples moved by the DMA (monotonic) */
    uint32_t sw;                /* samples moved by USB (monotonic) */
    uint16_t dma_pos;
    uint32_t frac;              /* period dither accumulator, Q16 */
    uint32_t pkt_acc;           /* fractional samples per frame (IN) */
    float    err_f, integ, ppm; /* fill loop */
    int32_t  fill;
    uint32_t xruns;
    uint16_t idle;              /* frames without an OUT packet */
    uint16_t peak;
    int32_t  gain;              /* Q15, 0 = muted */
    int32_t  dc;                /* DC estimate, Q12 (IN only) */
} stream_t;

static stream_t out_s = { .tim = TIM6, .dma = DMA2_Channel3, .ring = dac_ring, .fs = 48000 };
static stream_t in_s  = { .tim = TIM3, .dma = DMA2_Channel1, .ring = adc_ring, .fs = 48000 };

static float frame_clk = 72000.0f;   /* timer clocks per USB frame */
static uint32_t ts_ref;
static uint16_t fn_ref;
static bool ts_valid;

/* ---- helpers ------------------------------------------------------------ */
static uint32_t target_fill(const stream_t *s)
{
    return (uint32_t)cfg.buf_ms * s->fs / 1000u;
}

/* Follow the DMA position; must run at least once per ring period (>10 ms) */
static void dma_track(stream_t *s)
{
    uint16_t pos = (uint16_t)(RING_N - s->dma->CNDTR) & RING_M;
    s->hw += (uint16_t)(pos - s->dma_pos) & RING_M;
    s->dma_pos = pos;
}

static void set_period(stream_t *s)
{
    float per = frame_clk * 1000.0f / (float)s->fs / (1.0f + s->ppm * 1e-6f);
    uint32_t q = (uint32_t)(per * 65536.0f);
    s->frac += q & 0xFFFF;
    uint32_t arr = (q >> 16) + (s->frac >> 16);
    s->frac &= 0xFFFF;
    s->tim->ARR = arr - 1;
}

/* Fill loop. err > 0: too many samples buffered. dir = +1 if a faster
 * sample clock shrinks the fill (DAC), -1 if it grows it (ADC). */
static void fill_loop(stream_t *s, int32_t err, float dir)
{
    s->err_f += ((float)err - s->err_f) * (1.0f / 32.0f);
    s->integ += s->err_f * 0.01f;                    /* ppm per ms per sample */
    if (s->integ > 300.0f) s->integ = 300.0f;
    if (s->integ < -300.0f) s->integ = -300.0f;
    float p = dir * (20.0f * s->err_f + s->integ);
    if (p > 500.0f) p = 500.0f;
    if (p < -500.0f) p = -500.0f;
    s->ppm = p;
}

static void loop_reset(stream_t *s)
{
    s->err_f = s->integ = s->ppm = 0.0f;
    s->sync = true;
}

static void dac_clear(void)
{
    for (uint32_t i = 0; i < RING_N; i++)
        dac_ring[i] = 2048;
}

static int32_t db_to_q15(int16_t v256)
{
    return (int32_t)(32768.0f * expf((float)v256 * (0.11512925f / 256.0f)));  /* ln10/20 */
}

void audio_gain_update(void)
{
    out_s.gain = cfg.mute_out ? 0 : db_to_q15(cfg.vol_out);
    in_s.gain  = cfg.mute_in  ? 0 : db_to_q15(cfg.vol_in);
}

static void set_rate(stream_t *s, uint32_t fs)
{
    s->fs = fs;
    s->frac = 0;
    s->pkt_acc = 0;
    loop_reset(s);
    set_period(s);
    if (s == &out_s)
        dac_clear();
}

/* ---- init --------------------------------------------------------------- */
void audio_init(void)
{
    RCC->AHBENR  |= RCC_AHBENR_DMA2EN | RCC_AHBENR_ADC12EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM6EN |
                    RCC_APB1ENR_DAC1EN;

    /* TIM2: free-running 72 MHz timestamp counter for SOF */
    TIM2->PSC = 0;
    TIM2->ARR = 0xFFFFFFFFu;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 = TIM_CR1_CEN;

    /* Sample timers: update event -> TRGO */
    for (int i = 0; i < 2; i++) {
        TIM_TypeDef *t = i ? TIM3 : TIM6;
        t->PSC = 0;
        t->ARR = 1499;
        t->CR2 = TIM_CR2_MMS_1;
        t->CR1 = TIM_CR1_ARPE;
    }

    /* DAC1 CH1, triggered by TIM6 TRGO (TSEL=000), DMA2 Ch3 */
    dac_clear();
    DMA2_Channel3->CPAR = (uint32_t)&DAC1->DHR12R1;
    DMA2_Channel3->CMAR = (uint32_t)dac_ring;
    DMA2_Channel3->CNDTR = RING_N;
    DMA2_Channel3->CCR = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_1 | DMA_CCR_MINC |
                         DMA_CCR_CIRC | DMA_CCR_DIR | DMA_CCR_PL_1 | DMA_CCR_EN;
    DAC1->DHR12R1 = 2048;
    DAC1->CR = DAC_CR_EN1 | DAC_CR_TEN1 | DAC_CR_DMAEN1;

    /* ADC2 IN12, triggered by TIM3 TRGO (EXTSEL=0100), DMA2 Ch1 circular */
    ADC12_COMMON->CCR = ADC_CCR_CKMODE_1;            /* HCLK/2 = 36 MHz */
    ADC2->CR = 0;                                    /* ADVREGEN 10 -> 00 */
    ADC2->CR = ADC_CR_ADVREGEN_0;                    /* -> 01: regulator on */
    delay_us(20);
    ADC2->CR |= ADC_CR_ADCAL;
    delay_us(100);                                   /* calibration: ~2 us */
    if (ADC2->CR & ADC_CR_ADCAL)
        fatal();
    delay_us(2);
    ADC2->CR |= ADC_CR_ADEN;
    delay_us(10);
    if (!(ADC2->ISR & ADC_ISR_ADRDY))
        fatal();
    ADC2->SMPR2 = 5u << ADC_SMPR2_SMP12_Pos;         /* 61.5 cycles */
    ADC2->SQR1 = 12u << ADC_SQR1_SQ1_Pos;            /* L = 0: one conversion */
    ADC2->CFGR = ADC_CFGR_EXTEN_0 | ADC_CFGR_EXTSEL_2 | ADC_CFGR_DMAEN | ADC_CFGR_DMACFG;
    DMA2_Channel1->CPAR = (uint32_t)&ADC2->DR;
    DMA2_Channel1->CMAR = (uint32_t)adc_ring;
    DMA2_Channel1->CNDTR = RING_N;
    DMA2_Channel1->CCR = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC |
                         DMA_CCR_CIRC | DMA_CCR_PL_1 | DMA_CCR_EN;
    ADC2->CR |= ADC_CR_ADSTART;

    audio_gain_update();
    set_rate(&out_s, 48000);
    set_rate(&in_s, 48000);
    TIM6->EGR = TIM_EGR_UG;
    TIM3->EGR = TIM_EGR_UG;
    TIM6->CR1 |= TIM_CR1_CEN;
    TIM3->CR1 |= TIM_CR1_CEN;
}

/* ---- USB events --------------------------------------------------------- */
void audio_reset(void)
{
    out_s.active = in_s.active = false;
    dac_clear();
    loop_reset(&out_s);
    loop_reset(&in_s);
}

void audio_suspend(void)
{
    audio_reset();
    ts_valid = false;
}

static volatile bool resync_req;

void audio_resync(void)   /* any context: handled in the next SOF */
{
    resync_req = true;
}

static void in_prepare(void);

void audio_set_alt(uint8_t itf, uint8_t alt)
{
    stream_t *s = itf == IF_AS_OUT ? &out_s : &in_s;
    s->active = alt;
    loop_reset(s);
    if (s == &out_s) {
        dac_clear();
        s->idle = 0;
        usb_iso_enable(EP_AUDIO_OUT, alt);
    } else {
        s->dc = 0;
        usb_iso_enable(EP_AUDIO_IN, false);
        if (alt) {
            in_prepare();                /* first packet ready before enabling */
            usb_iso_enable(EP_AUDIO_IN, true);
        }
    }
    ptt_update();
}

void audio_out_packet(void)
{
    stream_t *s = &out_s;
    uint16_t bytes = usb_iso_rx_count(EP_AUDIO_OUT);
    if (!s->active)
        return;
    s->idle = 0;
    dma_track(s);
    if (s->sync) {
        s->sw = s->hw + target_fill(s);
        s->sync = false;
    }
    volatile uint16_t *p = usb_ep_pma(EP_AUDIO_OUT);
    int32_t g = s->gain;
    uint32_t peak = 0;
    uint32_t w = s->sw;
    for (uint16_t i = 0; i < bytes / 2; i++) {
        int32_t v = ((int32_t)(int16_t)p[2 * i] * g) >> 15;
        if (v > 32767) v = 32767;
        uint32_t a = (uint32_t)(v < 0 ? -v : v);
        if (a > peak) peak = a;
        dac_ring[w++ & RING_M] = (uint16_t)((v >> 4) + 2048);
    }
    s->sw = w;
    if (peak > s->peak) s->peak = (uint16_t)MIN(peak, 32767u);
    vox_feed(peak);

    int32_t fill = (int32_t)(s->sw - s->hw);
    s->fill = fill;
    if (fill < 0 || fill > (int32_t)(RING_N - 2 * AUDIO_PKT / 2)) {
        s->xruns++;
        dac_clear();
        s->sync = true;
        return;
    }
    /* measured right after writing: the setpoint includes this packet */
    fill_loop(s, fill - (int32_t)(target_fill(s) + bytes / 2), 1.0f);
}

static void in_prepare(void)
{
    stream_t *s = &in_s;
    dma_track(s);
    uint32_t n = s->fs / 1000u;
    s->pkt_acc += s->fs % 1000u;
    if (s->pkt_acc >= 1000u) {
        s->pkt_acc -= 1000u;
        n++;
    }
    int32_t fill = (int32_t)(s->hw - s->sw);
    if (s->sync || fill < (int32_t)n || fill > (int32_t)(RING_N - 64)) {
        if (!s->sync)
            s->xruns++;
        s->sync = false;
        s->sw = s->hw - target_fill(s) - n;
    }
    volatile uint16_t *p = usb_ep_pma(EP_AUDIO_IN);
    int32_t g = s->gain;
    uint32_t peak = 0;
    uint32_t r = s->sw;
    bool dcb = cfg.dc_block;
    for (uint32_t i = 0; i < n; i++) {
        int32_t x = ((int32_t)adc_ring[r++ & RING_M] - 2048) << 4;
        if (dcb) {
            s->dc += ((x << 12) - s->dc) >> 12;      /* ~1.9 Hz at 48 kHz */
            x -= s->dc >> 12;
        }
        int32_t v = (x * g) >> 15;
        if (v > 32767) v = 32767;
        if (v < -32768) v = -32768;
        uint32_t a = (uint32_t)(v < 0 ? -v : v);
        if (a > peak) peak = a;
        p[2 * i] = (uint16_t)v;
    }
    s->sw = r;
    usb_iso_tx_count(EP_AUDIO_IN, (uint16_t)(n * 2));
    if (peak > s->peak) s->peak = (uint16_t)MIN(peak, 32767u);

    fill = (int32_t)(s->hw - s->sw);
    s->fill = fill;
    fill_loop(s, fill - (int32_t)target_fill(s), -1.0f);
}

void audio_in_done(void)
{
    if (in_s.active)
        in_prepare();
}

void audio_sof(uint32_t t, uint16_t fn)
{
    /* Host frame period, averaged over >= 1024 frames */
    if (!ts_valid) {
        ts_ref = t;
        fn_ref = fn;
        ts_valid = true;
    } else {
        uint16_t df = (fn - fn_ref) & 0x7FF;
        if (df >= 1024) {
            float m = (float)(t - ts_ref) / (float)df;
            if (fabsf(m - 72000.0f) < 72.0f)         /* within 1000 ppm */
                frame_clk += (m - frame_clk) * 0.25f;
            ts_ref = t;
            fn_ref = fn;
        }
    }

    if (resync_req) {
        /* After a long stall (flash write) the DMA may have wrapped */
        resync_req = false;
        ts_valid = false;
        out_s.dma_pos = (uint16_t)(RING_N - out_s.dma->CNDTR) & RING_M;
        in_s.dma_pos = (uint16_t)(RING_N - in_s.dma->CNDTR) & RING_M;
        dac_clear();
        loop_reset(&out_s);
        loop_reset(&in_s);
    }
    dma_track(&out_s);
    dma_track(&in_s);

    /* OUT stream stopped without SET_INTERFACE: play silence */
    if (out_s.active && ++out_s.idle == 3) {
        dac_clear();
        out_s.sync = true;
        vox_feed(0);
    }

    set_period(&out_s);
    set_period(&in_s);
}

/* ---- class requests ----------------------------------------------------- */
enum { SET_CUR = 0x01, GET_CUR = 0x81, GET_MIN = 0x82, GET_MAX = 0x83, GET_RES = 0x84 };

static int put16(uint8_t *buf, int16_t v)
{
    buf[0] = (uint8_t)v;
    buf[1] = (uint8_t)((uint16_t)v >> 8);
    return 2;
}

int audio_setup(const usb_setup_t *s, uint8_t *buf, const uint8_t **in)
{
    uint8_t rcpt = s->bmRequestType & 0x1F;
    uint8_t cs = s->wValue >> 8;

    if (rcpt == 2) {                                 /* endpoint: sample rate */
        uint8_t ep = s->wIndex & 0xFF;
        stream_t *st = ep == EP_AUDIO_OUT ? &out_s :
                       ep == (0x80 | EP_AUDIO_IN) ? &in_s : NULL;
        if (!st || cs != 0x01)
            return -1;
        if (s->bRequest == SET_CUR) {
            uint32_t fs = buf[0] | (uint32_t)buf[1] << 8 | (uint32_t)buf[2] << 16;
            for (size_t i = 0; i < ARRAY_LEN(rates); i++)
                if (rates[i] == fs) {
                    set_rate(st, fs);
                    return 0;
                }
            return -1;
        }
        if (s->bRequest == GET_CUR) {
            buf[0] = (uint8_t)st->fs;
            buf[1] = (uint8_t)(st->fs >> 8);
            buf[2] = (uint8_t)(st->fs >> 16);
            return 3;
        }
        return -1;
    }

    /* interface: feature unit */
    uint8_t unit = s->wIndex >> 8;
    if (unit != UAC_FU_OUT && unit != UAC_FU_IN)
        return -1;
    bool o = unit == UAC_FU_OUT;
    uint8_t *mute = o ? &cfg.mute_out : &cfg.mute_in;
    int16_t *vol = o ? &cfg.vol_out : &cfg.vol_in;

    if (cs == 0x01) {                                /* mute */
        if (s->bRequest == SET_CUR) {
            *mute = buf[0] ? 1 : 0;
            audio_gain_update();
            return 0;
        }
        if (s->bRequest == GET_CUR) {
            buf[0] = *mute;
            return 1;
        }
        return -1;
    }
    if (cs == 0x02) {                                /* volume */
        switch (s->bRequest) {
        case SET_CUR: {
            int16_t v = (int16_t)(buf[0] | buf[1] << 8);
            if (v == (int16_t)0x8000) v = VOL_MIN;   /* -inf */
            v = (int16_t)MAX(VOL_MIN, MIN(VOL_MAX, v));
            *vol = (int16_t)((v / VOL_RES) * VOL_RES);
            audio_gain_update();
            return 0;
        }
        case GET_CUR: return put16(buf, *vol);
        case GET_MIN: return put16(buf, VOL_MIN);
        case GET_MAX: return put16(buf, VOL_MAX);
        case GET_RES: return put16(buf, VOL_RES);
        }
    }
    return -1;
}

/* ---- status ------------------------------------------------------------- */
void audio_stat(bool in, audio_stat_t *st)
{
    stream_t *s = in ? &in_s : &out_s;
    __disable_irq();
    st->fs = s->fs;
    st->active = s->active;
    st->fill = s->fill;
    st->ppm = s->ppm;
    st->xruns = s->xruns;
    st->peak = s->peak;
    s->peak = 0;
    __enable_irq();
}

float audio_frame_ppm(void)
{
    return (frame_clk / 72000.0f - 1.0f) * 1e6f;
}
