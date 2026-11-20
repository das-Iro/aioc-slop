/* USB audio class 1, synchronous endpoints, with oversampling.
 *
 * host -> radio: iso OUT -> out_fs ring -> interpolator (xOSR) -> dac_hw ring
 *                -> DMA2 Ch3 -> DAC1 CH1 (PA4), paced by TIM6
 * radio -> host: ADC2 IN12 (PB2), paced by TIM3 -> DMA2 Ch1 -> adc_hw ring
 *                -> decimator (/OSR) -> in_fs ring -> iso IN
 *
 * The converters run at fs * OSR (~192 kHz). Rate conversion is a simple
 * moving average: the ADC side averages each group of OSR samples into one,
 * the DAC side repeats every sample OSR times (zero-stuffing followed by an
 * OSR-long moving average, i.e. a sample-and-hold).
 * The filters run in a low-priority interrupt (DSP_IRQ) pended from every
 * USB SOF, so the SOF timestamp in the USB ISR is never delayed by them.
 *
 * Both sample clocks are locked to the host: the SOF interrupt timestamps a
 * free-running 72 MHz counter (TIM2) to measure the host frame period, which
 * sets the timer periods (feed-forward). A slow PI loop keeps the total
 * number of queued samples (fs ring + filter + DMA ring) at its setpoint.
 * The fractional timer period is dithered once per frame. */
#include <math.h>
#include "audio.h"
#include "settings.h"
#include "ptt.h"
#include "clock.h"

#define HW_N     1024u            /* converter-rate DMA rings, power of two */
#define HW_M     (HW_N - 1u)
#define FS_N     512u             /* fs-rate rings */
#define FS_M     (FS_N - 1u)
#define HW_RATE  192000u          /* converter rate target */
#define OSR_MAX  24
#define LEAD_US  1500u            /* DAC ring kept this far ahead of playback */

static uint16_t dac_hw[HW_N];
static uint16_t adc_hw[HW_N];
static int16_t out_fs[FS_N];
static int16_t in_fs[FS_N];
static const uint32_t rates[] = { 8000, 11025, 16000, 22050, 24000, 32000, 44100, 48000 };

typedef struct {
    TIM_TypeDef *tim;
    DMA_Channel_TypeDef *dma;
    volatile uint32_t fs;
    volatile bool active;
    volatile bool reconf;       /* DSP: redo filters and positions */
    volatile uint8_t osr;
    /* fs ring: OUT w = USB, r = DSP; IN w = DSP, r = USB */
    volatile uint32_t w, r;
    volatile uint16_t done;     /* hw ring: next index the DSP handles */
    uint32_t ph;                /* interpolator phase (DSP) */
    /* USB side */
    volatile bool sync;
    float    sp;                /* fill setpoint, captured at sync */
    bool     sp_valid;
    float    err_f, integ, ppm;
    float    fill;
    uint32_t xruns;
    uint16_t idle;
    uint16_t peak;
    int32_t  gain;              /* Q15, 0 = muted */
    int32_t  dc;                /* DC estimate, Q12 (IN only) */
    uint32_t frac;              /* period dither accumulator, Q16 */
    uint32_t pkt_acc;           /* fractional samples per frame (IN) */
} stream_t;

static stream_t out_s = { .tim = TIM6, .dma = DMA2_Channel3, .fs = 48000, .osr = 1 };
static stream_t in_s  = { .tim = TIM3, .dma = DMA2_Channel1, .fs = 48000, .osr = 1 };

static float frame_clk = 72000.0f;   /* timer clocks per USB frame */
static uint32_t ts_ref;
static uint16_t fn_ref;
static bool ts_valid;
static uint32_t dsp_cyc, dsp_frames;
static float dsp_load;

/* ---- helpers ------------------------------------------------------------ */
static uint8_t osr_for(uint32_t fs)
{
    if (!cfg.oversample) return 1;
    uint32_t o = HW_RATE / fs;
    return (uint8_t)MIN(o, (uint32_t)OSR_MAX);
}

static uint16_t dma_pos(const stream_t *s)
{
    return (uint16_t)(HW_N - s->dma->CNDTR) & HW_M;
}

static uint32_t target_fill(const stream_t *s)
{
    return (uint32_t)cfg.buf_ms * s->fs / 1000u;
}

static void set_period(stream_t *s)
{
    float per = frame_clk * 1000.0f / ((float)s->fs * s->osr) / (1.0f + s->ppm * 1e-6f);
    uint32_t q = (uint32_t)(per * 65536.0f);
    s->frac += q & 0xFFFF;
    uint32_t arr = (q >> 16) + (s->frac >> 16);
    s->frac &= 0xFFFF;
    s->tim->ARR = arr - 1;
}

/* Fill loop. err > 0: too many samples queued. dir = +1 if a faster
 * sample clock shrinks the fill (DAC), -1 if it grows it (ADC). */
static void fill_loop(stream_t *s, float err, float dir)
{
    s->err_f += (err - s->err_f) * (1.0f / 32.0f);
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
    s->sp_valid = false;
    s->sync = true;
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

/* ---- DSP (DSP_IRQ, below the USB ISR) ------------------------------------ */

static void reconfigure(stream_t *s, bool out)
{
    uint8_t osr = osr_for(s->fs);
    uint16_t pos = dma_pos(s);
    __disable_irq();
    if (out)
        for (uint32_t i = 0; i < HW_N; i++)
            dac_hw[i] = 2048;
    s->osr = osr;
    s->ph = 0;
    s->done = pos;
    s->reconf = false;
    loop_reset(s);
    __enable_irq();
}

static void dsp_adc(void)
{
    stream_t *s = &in_s;
    uint16_t pos = dma_pos(s);
    uint16_t d = s->done;
    uint32_t avail = (uint16_t)(pos - d) & HW_M;
    const uint32_t osr = s->osr;
    uint32_t w = s->w;
    if (avail > HW_N / 2) {                  /* fell behind (long stall) */
        d = pos;
        avail = 0;
    }
    for (; avail >= osr; avail -= osr) {
        /* moving average over the OSR samples of this group */
        int32_t sum = 0;
        for (uint32_t i = 0; i < osr; i++) {
            sum += adc_hw[d];
            d = (d + 1) & HW_M;
        }
        /* remove the 2048 offset, 12 bit -> 16 bit; the averaging adds
         * fractional bits below the 12-bit LSB */
        int32_t y = (sum - 2048 * (int32_t)osr) * 16 / (int32_t)osr;
        in_fs[w++ & FS_M] = (int16_t)MAX(-32768, MIN(32767, y));
    }
    __disable_irq();
    s->w = w;
    s->done = d;
    __enable_irq();
}

static void dsp_dac(void)
{
    stream_t *s = &out_s;
    uint16_t pos = dma_pos(s);
    uint16_t d = s->done;
    uint32_t queued = (uint16_t)(d - pos) & HW_M;
    const uint32_t osr = s->osr;
    const uint32_t lead = s->fs * osr / 1000u * LEAD_US / 1000u;
    uint32_t r = s->r, ph = s->ph;
    uint32_t w = s->w;
    if (queued > HW_N / 2) {                 /* DMA overtook us */
        d = pos;
        queued = 0;
    }
    while (queued < lead) {
        if (ph == 0) {
            if ((int32_t)(w - r) <= 0) {
                w = s->w;                     /* re-read, USB may have added */
                if ((int32_t)(w - r) <= 0)
                    out_fs[r & FS_M] = 0;     /* no data: play silence */
            }
            r++;
        }
        /* zero-stuffing + OSR-long moving average = repeat the sample */
        int32_t v = (out_fs[(r - 1u) & FS_M] >> 4) + 2048;
        uint16_t dv = (uint16_t)(v < 0 ? 0 : v > 4095 ? 4095 : v);
        for (; ph < osr && queued < lead; ph++, queued++) {
            dac_hw[d] = dv;
            d = (d + 1) & HW_M;
        }
        if (ph == osr) ph = 0;
    }
    s->ph = ph;
    __disable_irq();
    s->r = r;
    s->done = d;
    __enable_irq();
}

void EXTI0_IRQHandler(void)   /* DSP_IRQ */
{
    uint32_t t0 = DWT->CYCCNT;
    if (out_s.reconf) reconfigure(&out_s, true);
    if (in_s.reconf) reconfigure(&in_s, false);
    dsp_adc();
    dsp_dac();
    dsp_cyc += DWT->CYCCNT - t0;
}

/* ---- init --------------------------------------------------------------- */
void audio_init(void)
{
    RCC->AHBENR  |= RCC_AHBENR_DMA2EN | RCC_AHBENR_ADC12EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM6EN |
                    RCC_APB1ENR_DAC1EN;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  /* cycle counter: DSP load */
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

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
    for (uint32_t i = 0; i < HW_N; i++)
        dac_hw[i] = 2048;
    DMA2_Channel3->CPAR = (uint32_t)&DAC1->DHR12R1;
    DMA2_Channel3->CMAR = (uint32_t)dac_hw;
    DMA2_Channel3->CNDTR = HW_N;
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
    ADC2->SMPR2 = 5u << ADC_SMPR2_SMP12_Pos;         /* 61.5 cycles: 2 us/conv */
    ADC2->SQR1 = 12u << ADC_SQR1_SQ1_Pos;            /* L = 0: one conversion */
    ADC2->CFGR = ADC_CFGR_EXTEN_0 | ADC_CFGR_EXTSEL_2 | ADC_CFGR_DMAEN |
                 ADC_CFGR_DMACFG | ADC_CFGR_OVRMOD;
    DMA2_Channel1->CPAR = (uint32_t)&ADC2->DR;
    DMA2_Channel1->CMAR = (uint32_t)adc_hw;
    DMA2_Channel1->CNDTR = HW_N;
    DMA2_Channel1->CCR = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC |
                         DMA_CCR_CIRC | DMA_CCR_PL_1 | DMA_CCR_EN;
    ADC2->CR |= ADC_CR_ADSTART;

    audio_gain_update();
    out_s.reconf = in_s.reconf = true;
    out_s.osr = osr_for(out_s.fs);
    in_s.osr = osr_for(in_s.fs);
    set_period(&out_s);
    set_period(&in_s);
    TIM6->EGR = TIM_EGR_UG;
    TIM3->EGR = TIM_EGR_UG;
    TIM6->CR1 |= TIM_CR1_CEN;
    TIM3->CR1 |= TIM_CR1_CEN;

    NVIC_SetPriority(DSP_IRQ, PRIO_DSP);
    NVIC_EnableIRQ(DSP_IRQ);
    NVIC_SetPendingIRQ(DSP_IRQ);
}

/* ---- USB events --------------------------------------------------------- */
void audio_reset(void)
{
    out_s.active = in_s.active = false;
    loop_reset(&out_s);
    loop_reset(&in_s);
}

void audio_suspend(void)
{
    audio_reset();
    ts_valid = false;
    /* no more SOFs, so the DSP stops: silence the DAC ring */
    for (uint32_t i = 0; i < HW_N; i++)
        dac_hw[i] = 2048;
}

void audio_resync(void)   /* any context */
{
    ts_valid = false;
    out_s.reconf = in_s.reconf = true;
    NVIC_SetPendingIRQ(DSP_IRQ);
}

void audio_reconfigure(void) { audio_resync(); }

static void in_prepare(void);

void audio_set_alt(uint8_t itf, uint8_t alt)
{
    stream_t *s = itf == IF_AS_OUT ? &out_s : &in_s;
    s->active = alt;
    loop_reset(s);
    if (s == &out_s) {
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

/* Samples queued between USB and the converter, in fs samples */
static float out_queued(void)
{
    stream_t *s = &out_s;
    uint32_t q = (uint16_t)(s->done - dma_pos(s)) & HW_M;
    if (q > HW_N / 2) q = 0;
    return (float)(int32_t)(s->w - s->r) + (float)q / s->osr;
}

static float in_queued(void)
{
    stream_t *s = &in_s;
    uint32_t q = (uint16_t)(dma_pos(s) - s->done) & HW_M;
    if (q > HW_N / 2) q = 0;
    return (float)(int32_t)(s->w - s->r) + (float)q / s->osr;
}

void audio_out_packet(void)
{
    stream_t *s = &out_s;
    uint16_t bytes = usb_iso_rx_count(EP_AUDIO_OUT);
    if (!s->active)
        return;
    s->idle = 0;
    uint32_t w = s->w;
    if (s->sync) {
        uint32_t r = s->r, t = target_fill(s);
        for (uint32_t i = 0; i < t; i++)
            out_fs[(r + i) & FS_M] = 0;
        w = r + t;
        s->sync = false;
        s->sp_valid = false;
    }
    volatile uint16_t *p = usb_ep_pma(EP_AUDIO_OUT);
    int32_t g = s->gain;
    uint32_t peak = 0;
    for (uint16_t i = 0; i < bytes / 2; i++) {
        int32_t v = ((int32_t)(int16_t)p[2 * i] * g) >> 15;
        if (v > 32767) v = 32767;
        uint32_t a = (uint32_t)(v < 0 ? -v : v);
        if (a > peak) peak = a;
        out_fs[w++ & FS_M] = (int16_t)v;
    }
    s->w = w;
    if (peak > s->peak) s->peak = (uint16_t)MIN(peak, 32767u);
    vox_feed(peak);

    int32_t ring = (int32_t)(w - s->r);
    if (ring < 0 || ring > (int32_t)(FS_N - 2 * AUDIO_PKT / 2)) {
        s->xruns++;
        s->sync = true;
        return;
    }
    float f = out_queued();
    if (!s->sp_valid) {                  /* first packet after a sync */
        s->sp = f;
        s->sp_valid = true;
    }
    s->fill = f - s->sp;
    fill_loop(s, s->fill, 1.0f);
}

static void in_prepare(void)
{
    stream_t *s = &in_s;
    uint32_t n = s->fs / 1000u;
    s->pkt_acc += s->fs % 1000u;
    if (s->pkt_acc >= 1000u) {
        s->pkt_acc -= 1000u;
        n++;
    }
    uint32_t w = s->w;
    int32_t ring = (int32_t)(w - s->r);
    if (s->sync || ring < (int32_t)n || ring > (int32_t)(FS_N - 64)) {
        if (!s->sync)
            s->xruns++;
        s->sync = false;
        s->sp_valid = false;
        s->r = w - target_fill(s) - n;
    }
    volatile uint16_t *p = usb_ep_pma(EP_AUDIO_IN);
    int32_t g = s->gain;
    uint32_t peak = 0;
    uint32_t r = s->r;
    bool dcb = cfg.dc_block;
    for (uint32_t i = 0; i < n; i++) {
        int32_t x = in_fs[r++ & FS_M];
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
    s->r = r;
    usb_iso_tx_count(EP_AUDIO_IN, (uint16_t)(n * 2));
    if (peak > s->peak) s->peak = (uint16_t)MIN(peak, 32767u);

    float f = in_queued();
    if (!s->sp_valid) {
        s->sp = f;
        s->sp_valid = true;
    }
    s->fill = f - s->sp;
    fill_loop(s, s->fill, -1.0f);
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

    /* OUT stream stopped without SET_INTERFACE: the DSP plays silence */
    if (out_s.active && ++out_s.idle == 3) {
        out_s.sync = true;
        vox_feed(0);
    }

    set_period(&out_s);
    set_period(&in_s);
    NVIC_SetPendingIRQ(DSP_IRQ);

    if (++dsp_frames >= 1000) {                      /* load over ~1 s */
        uint32_t c = dsp_cyc;
        dsp_cyc = 0;
        dsp_load = (float)c * (100.0f / ((float)SYSCLK_HZ * dsp_frames / 1000.0f));
        dsp_frames = 0;
    }
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
                    st->fs = fs;
                    st->frac = 0;
                    st->pkt_acc = 0;
                    st->osr = osr_for(fs);   /* timer period right away */
                    loop_reset(st);
                    st->reconf = true;
                    NVIC_SetPendingIRQ(DSP_IRQ);
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
    st->osr = s->osr;
    st->fill = (int32_t)lrintf(s->fill);
    st->ppm = s->ppm;
    st->xruns = s->xruns;
    st->peak = s->peak;
    s->peak = 0;
    __enable_irq();
}

float audio_frame_ppm(void) { return (frame_clk / 72000.0f - 1.0f) * 1e6f; }
float audio_dsp_load(void) { return dsp_load; }
