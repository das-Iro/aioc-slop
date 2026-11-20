/* USB audio class 1, synchronous endpoints, with oversampling.
 *
 * host -> radio: iso OUT -> out_fs ring -> rational interpolator + 1st-order
 *                sigma-delta -> dac_hw ring -> DMA2 Ch3 -> DAC1 CH1 (PA4),
 *                free-running at 480 kHz (TIM6)
 * radio -> host: ADC2 IN12 (PB2), free-running at 1.125 MS/s -> DMA2 Ch1 ->
 *                adc_hw ring -> rational decimator -> in_fs ring -> iso IN
 *
 * Both converters run at fixed rates from the crystal. Rate conversion is a
 * moving average with fractional boundary weights (after rhgndf's AIOC fork,
 * branch adc_dma): the ADC side averages `ratio` samples per output, the DAC
 * side repeats each sample `ratio` times and blends the boundary sample. The
 * DAC gets 16-bit samples through a first-order sigma-delta modulator that
 * pushes the 4 bits the DAC can't output above the audio band.
 * The DSP runs in a low-priority interrupt (DSP_IRQ) pended from every USB
 * SOF and the ADC DMA half/full events, so the SOF timestamp in the USB ISR
 * is never delayed by it.
 *
 * Both paths are locked to the host: the SOF interrupt timestamps a free-
 * running 72 MHz counter (TIM2) to measure the host frame period, which sets
 * the conversion ratios (feed-forward). A slow PI loop keeps the number of
 * queued samples (fs ring + DMA ring) at its setpoint. */
#include <math.h>
#include "audio.h"
#include "settings.h"
#include "ptt.h"
#include "clock.h"

#define HW_N     2048u            /* DAC DMA ring (4.3 ms at 480 kHz) */
#define HW_M     (HW_N - 1u)
#define DAC_CLKS 150u             /* HCLK cycles per DAC sample: 480 kHz */
#define ADC_N    4096u            /* ADC DMA ring (3.6 ms at 1.125 MS/s) */
#define ADC_M    (ADC_N - 1u)
/* ADC sample time code (SMPR) and the resulting conversion period in HCLK
 * cycles: ADC clock = HCLK/2 = 36 MHz, conversion = sample time + 12.5 */
#ifndef ADC_SMP
#define ADC_SMP  4u               /* 19.5 cycles -> 1.125 MS/s; measured: same
                                     noise floor as 2.4 MS/s, -9 % CPU */
#endif
#define ADC_CLKS ((uint32_t[]){ 28, 30, 34, 40, 64, 148, 388, 1228 }[ADC_SMP])
#define FS_N     512u             /* fs-rate rings */
#define FS_M     (FS_N - 1u)
#define LEAD_US  1500u            /* DAC ring kept this far ahead of playback */

static uint16_t dac_hw[HW_N];
static uint16_t adc_hw[ADC_N];
static int16_t out_fs[FS_N];
static int16_t in_fs[FS_N];
static const uint32_t rates[] = { 8000, 11025, 16000, 22050, 24000, 32000, 44100, 48000 };

#define LP_SECTIONS 2           /* 4th-order Butterworth */
#define LP_RX_RATE  64000u      /* RX low-pass runs at up to this rate (k fs,
                                   k <= 4), before the last decimation step */

typedef struct {
    float b0, b1, b2, a1, a2;   /* normalized, a0 = 1 */
    float z1, z2;               /* transposed direct form II state */
} biquad_t;

typedef struct {
    DMA_Channel_TypeDef *dma;
    volatile uint32_t fs;
    volatile bool active;
    volatile bool reconf;       /* DSP: redo filters and positions */
    uint16_t ring_n;            /* DMA ring length */
    volatile float ratio;       /* ADC: inputs per output, DAC: outputs per input */
    float    need, acc;         /* resampler state (DSP) */
    int32_t  cur, carry;        /* DAC: current sample, sigma-delta error */
    /* fs ring: OUT w = USB, r = DSP; IN w = DSP, r = USB */
    volatile uint32_t w, r;
    volatile uint16_t done;     /* hw ring: next index the DSP handles */
    /* USB side */
    volatile bool sync;
    float    sp;                /* fill setpoint, captured at sync */
    bool     sp_valid;
    float    err_f, integ, ppm;
    float    fill;
    uint32_t xruns;
    uint16_t idle;
    uint16_t peak;
    int32_t  gain;              /* Q12 (4096 = 0 dB, up to +12 dB), 0 = muted */
    biquad_t lp[LP_SECTIONS];   /* low-pass: cascaded biquads */
    float    lp_sum;            /* RX: sum over the last decimation step */
    uint32_t lp_n;
    uint32_t lp_os;             /* RX: low-pass rate / fs (1, 2 or 4) */
    bool     lp_on;
    int32_t  dc;                /* DC estimate, Q12 (IN only) */
    bool     dc_init;           /* seed the DC estimate with the next sample */
    uint32_t pkt_acc;           /* fractional samples per frame (IN) */
} stream_t;

static stream_t out_s = { .dma = DMA2_Channel3, .fs = 48000, .ring_n = HW_N, .ratio = 10.0f };
static stream_t in_s  = { .dma = DMA2_Channel1, .fs = 48000, .ring_n = ADC_N, .ratio = 50.0f };

static float frame_clk = 72000.0f;   /* timer clocks per USB frame */
static uint32_t ts_ref;
static uint16_t fn_ref;
static bool ts_valid;
static uint32_t dsp_cyc, dsp_frames;
static float dsp_load;

/* ---- helpers ------------------------------------------------------------ */
static uint16_t dma_pos(const stream_t *s)
{
    return (uint16_t)(s->ring_n - s->dma->CNDTR) & (s->ring_n - 1u);
}

static uint32_t target_fill(const stream_t *s)
{
    return (uint32_t)cfg.buf_ms * s->fs / 1000u;
}

/* The converters free-run on the crystal; follow the host clock with the
 * conversion ratios. Converter samples per host frame / USB samples per
 * frame, corrected by the fill loop. */
static void set_ratio(stream_t *s)
{
    float conv = frame_clk / (float)(s == &in_s ? ADC_CLKS : DAC_CLKS);
    s->ratio = conv * 1000.0f / (float)s->fs / (1.0f + s->ppm * 1e-6f);
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

static int32_t db_to_q12(int16_t v256)
{
    return (int32_t)(4096.0f * expf((float)v256 * (0.11512925f / 256.0f)));  /* ln10/20 */
}

/* ---- low-pass ------------------------------------------------------------ */
static const float lp_hz[LP_COUNT] = { 0, 3000, 4000, 8000, 16000 };

/* 4th-order Butterworth low-pass: two biquads (bilinear transform, RBJ
 * cookbook) with the Butterworth pole Qs. 24 dB/octave, flat passband.
 * Off when the cutoff is not clearly below Nyquist. */
static void lp_design(stream_t *s, uint8_t sel)
{
    static const float q[LP_SECTIONS] = { 0.54119610f, 1.30656296f };
    float fc = sel < LP_COUNT ? lp_hz[sel] : 0.0f;
    bool on = fc > 0.0f && fc < 0.45f * (float)s->fs;
    /* RX: run at k fs (anti-aliasing). At 44.1/48 kHz k = 1: the input RC
     * (R5/C11, ~7 kHz) already keeps content away from fs/2 there. */
    uint32_t os = 1;
    if (s == &in_s)
        while (os < 4 && s->fs * os * 2 <= LP_RX_RATE)
            os *= 2;
    biquad_t bq[LP_SECTIONS];
    for (int k = 0; k < LP_SECTIONS; k++) {
        biquad_t *f = &bq[k];
        *f = (biquad_t){ .b0 = 1.0f };
        if (!on)
            continue;
        float rate = (float)s->fs * os;
        float w0 = 2.0f * 3.14159265f * fc / rate;
        float cw = cosf(w0), alpha = sinf(w0) / (2.0f * q[k]);
        float a0 = 1.0f + alpha;
        f->b0 = (1.0f - cw) * 0.5f / a0;
        f->b1 = (1.0f - cw) / a0;
        f->b2 = f->b0;
        f->a1 = -2.0f * cw / a0;
        f->a2 = (1.0f - alpha) / a0;
    }
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    for (int k = 0; k < LP_SECTIONS; k++)
        s->lp[k] = bq[k];
    s->lp_os = os;
    s->lp_sum = 0.0f;
    s->lp_n = 0;
    s->lp_on = on;
    __set_PRIMASK(pm);
}

void audio_filter_update(void)
{
    lp_design(&out_s, cfg.lp_tx);
    lp_design(&in_s, cfg.lp_rx);
}

bool audio_filter_active(bool in) { return in ? in_s.lp_on : out_s.lp_on; }

static inline float lp_run(stream_t *s, float x)
{
    for (int k = 0; k < LP_SECTIONS; k++) {
        biquad_t *f = &s->lp[k];
        float y = f->b0 * x + f->z1;
        f->z1 = f->b1 * x - f->a1 * y + f->z2;
        f->z2 = f->b2 * x - f->a2 * y;
        x = y;
    }
    return x;
}

void audio_gain_update(void)
{
    out_s.gain = cfg.mute_out ? 0 : db_to_q12(cfg.vol_out);
    in_s.gain  = cfg.mute_in  ? 0 : db_to_q12(cfg.vol_in);
}

/* ---- DSP (DSP_IRQ, below the USB ISR) ------------------------------------ */

static void reconfigure(stream_t *s, bool out)
{
    uint16_t pos = dma_pos(s);
    __disable_irq();
    if (out)
        for (uint32_t i = 0; i < HW_N; i++)
            dac_hw[i] = 0x8000;
    s->need = out ? 0.0f : s->ratio;
    s->acc = 0.0f;
    s->cur = s->carry = 0;
    s->done = pos;
    s->reconf = false;
    loop_reset(s);
    __enable_irq();
}

/* Rational moving-average decimator (after rhgndf's AIOC fork): every
 * output is the average of `ratio` input samples; the sample on a boundary
 * is split between the two outputs by its fractional weight. With
 * oversampling off it point-samples instead (one input per output). */
static void dsp_adc(void)
{
    stream_t *s = &in_s;
    uint16_t pos = dma_pos(s);
    uint16_t d = s->done;
    uint32_t avail = (uint16_t)(pos - d) & ADC_M;
    uint32_t w = s->w;
    if (avail > ADC_N / 2) {                 /* fell behind (long stall) */
        d = pos;
        avail = 0;
    }
    const float R = s->ratio;
    const bool avg = cfg.oversample;
    float need = s->need, acc = s->acc;
    /* With the RX low-pass on, the moving average produces 4 fs; the
     * low-pass runs there and an average of 4 makes fs. Content above the
     * cutoff is attenuated before it can fold into the audio band. */
    const bool lpf = s->lp_on;
    const uint32_t os = lpf ? s->lp_os : 1u;
    const float Ri = R / (float)os;
    const float scale_i = 16.0f / Ri, inv_os = 1.0f / (float)os;
    float lp_sum = s->lp_sum;
    uint32_t lp_n = s->lp_n;
    biquad_t bq[LP_SECTIONS];                /* local copy: keeps it in registers */
    for (int k = 0; k < LP_SECTIONS; k++)
        bq[k] = s->lp[k];
    while (avail) {
        if (need >= 1.0f) {
            /* whole samples of the current output, up to the ring end */
            uint32_t k = MIN((uint32_t)need, avail);
            k = MIN(k, ADC_N - d);
            if (avg) {
                const uint16_t *p = &adc_hw[d];
                uint32_t sum = 0;
#pragma GCC unroll 4
                for (uint32_t i = 0; i < k; i++)
                    sum += p[i];
                acc += (float)sum;
            }
            need -= (float)k;
            d = (d + k) & ADC_M;
            avail -= k;
        } else {
            /* boundary sample: finish this output, start the next */
            float x = (float)adc_hw[d];
            d = (d + 1) & ADC_M;
            avail--;
            float y = (avg ? (acc + x * need) * scale_i : x * 16.0f) - 32768.0f;
            acc = x * (1.0f - need);
            need += Ri - 1.0f;
            if (lpf) {
                for (int k = 0; k < LP_SECTIONS; k++) {
                    biquad_t *f = &bq[k];
                    float o = f->b0 * y + f->z1;
                    f->z1 = f->b1 * y - f->a1 * o + f->z2;
                    f->z2 = f->b2 * y - f->a2 * o;
                    y = o;
                }
                lp_sum += y;
                if (++lp_n < os)
                    continue;
                y = lp_sum * inv_os;
                lp_sum = 0.0f;
                lp_n = 0;
            }
            int32_t v = (int32_t)lrintf(y);
            in_fs[w++ & FS_M] = (int16_t)MAX(-32768, MIN(32767, v));
        }
    }
    s->need = need;
    s->acc = acc;
    s->lp_sum = lp_sum;
    s->lp_n = lp_n;
    if (lpf) {
        uint32_t pm = __get_PRIMASK();
        __disable_irq();
        if (s->lp_on)                         /* not redesigned meanwhile */
            for (int k = 0; k < LP_SECTIONS; k++) {
                s->lp[k].z1 = bq[k].z1;
                s->lp[k].z2 = bq[k].z2;
            }
        __set_PRIMASK(pm);
    }
    __disable_irq();
    s->w = w;
    s->done = d;
    __enable_irq();
}

/* Rational interpolator: each input sample is repeated `ratio` times; the
 * output sample on a boundary blends the old and new input by its
 * fractional weight. Then a first-order sigma-delta modulator turns the
 * 16-bit value into the DAC's 12 bits, feeding the dropped 4 bits forward
 * into the next sample. With oversampling off: plain sample-and-hold,
 * truncated to 12 bits. */
static void dsp_dac(void)
{
    stream_t *s = &out_s;
    uint16_t pos = dma_pos(s);
    uint16_t d = s->done;
    uint32_t queued = (uint16_t)(d - pos) & HW_M;
    const uint32_t lead = SYSCLK_HZ / DAC_CLKS / 1000u * LEAD_US / 1000u;
    const float R = s->ratio;
    const bool os = cfg.oversample;
    float rem = s->need;
    int32_t cur = s->cur, carry = s->carry;
    uint32_t r = s->r, w = s->w;
    if (queued > HW_N / 2) {                 /* DMA overtook us */
        d = pos;
        queued = 0;
    }
    while (queued < lead) {
        int32_t v;
        uint32_t k = 1;
        if (rem >= 1.0f) {
            /* a run of repeats of the current sample */
            k = MIN((uint32_t)rem, lead - queued);
            k = MIN(k, HW_N - d);             /* up to the ring end */
            v = cur;
            rem -= (float)k;
        } else {
            if ((int32_t)(w - r) <= 0) {
                w = s->w;                     /* re-read, USB may have added */
                if ((int32_t)(w - r) <= 0)
                    out_fs[r & FS_M] = 0;     /* no data: play silence */
            }
            int32_t nx = out_fs[r++ & FS_M];
            v = os ? cur + (int32_t)((float)(nx - cur) * (1.0f - rem)) : nx;
            cur = nx;
            rem += R - 1.0f;
        }
        uint32_t u = (uint32_t)(v + 32768);   /* 0..65535, left-aligned DAC */
        uint16_t *p = &dac_hw[d];
        if (os) {
            /* first-order sigma-delta: carry the dropped 4 bits forward.
             * u + carry <= 0xFFFF + 15; clamp the top. */
            for (uint32_t i = 0; i < k; i++) {
                uint32_t a = u + (uint32_t)carry;
                if (a > 0xFFFF) a = 0xFFFF;
                p[i] = (uint16_t)(a & 0xFFF0);
                carry = (int32_t)(a & 0x000F);
            }
        } else {
            for (uint32_t i = 0; i < k; i++)
                p[i] = (uint16_t)(u & 0xFFF0);
        }
        d = (d + k) & HW_M;
        queued += k;
    }
    s->need = rem;
    s->cur = cur;
    s->carry = carry;
    __disable_irq();
    s->r = r;
    s->done = d;
    __enable_irq();
}

void DMA2_Channel1_IRQHandler(void)   /* ADC ring half/full */
{
    DMA2->IFCR = DMA_IFCR_CGIF1;
    NVIC_SetPendingIRQ(DSP_IRQ);
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
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM6EN |
                    RCC_APB1ENR_DAC1EN;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  /* cycle counter: DSP load */
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* TIM2: free-running 72 MHz timestamp counter for SOF */
    TIM2->PSC = 0;
    TIM2->ARR = 0xFFFFFFFFu;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 = TIM_CR1_CEN;

    /* DAC sample timer: fixed 480 kHz, update event -> TRGO */
    TIM6->PSC = 0;
    TIM6->ARR = DAC_CLKS - 1;
    TIM6->CR2 = TIM_CR2_MMS_1;
    TIM6->CR1 = TIM_CR1_ARPE;

    /* DAC1 CH1 (16-bit left-aligned), triggered by TIM6 TRGO, DMA2 Ch3 */
    for (uint32_t i = 0; i < HW_N; i++)
        dac_hw[i] = 0x8000;
    DMA2_Channel3->CPAR = (uint32_t)&DAC1->DHR12L1;
    DMA2_Channel3->CMAR = (uint32_t)dac_hw;
    DMA2_Channel3->CNDTR = HW_N;
    DMA2_Channel3->CCR = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_1 | DMA_CCR_MINC |
                         DMA_CCR_CIRC | DMA_CCR_DIR | DMA_CCR_PL_1 | DMA_CCR_EN;
    DAC1->DHR12L1 = 0x8000;
    DAC1->CR = DAC_CR_EN1 | DAC_CR_TEN1 | DAC_CR_DMAEN1;

    /* ADC2 IN12, continuous conversion (1.125 MS/s), DMA2 Ch1 circular */
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
    ADC2->SMPR2 = ADC_SMP << ADC_SMPR2_SMP12_Pos;   /* C11 holds the pin */
    ADC2->SQR1 = 12u << ADC_SQR1_SQ1_Pos;            /* L = 0: one conversion */
    ADC2->CFGR = ADC_CFGR_CONT | ADC_CFGR_DMAEN | ADC_CFGR_DMACFG | ADC_CFGR_OVRMOD;
    DMA2_Channel1->CPAR = (uint32_t)&ADC2->DR;
    DMA2_Channel1->CMAR = (uint32_t)adc_hw;
    DMA2_Channel1->CNDTR = ADC_N;
    /* half/full interrupts make sure the DSP keeps up between SOFs */
    DMA2_Channel1->CCR = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC |
                         DMA_CCR_CIRC | DMA_CCR_PL_1 | DMA_CCR_HTIE | DMA_CCR_TCIE |
                         DMA_CCR_EN;
    ADC2->CR |= ADC_CR_ADSTART;

    audio_gain_update();
    audio_filter_update();
    out_s.reconf = in_s.reconf = true;
    set_ratio(&out_s);
    set_ratio(&in_s);
    TIM6->EGR = TIM_EGR_UG;
    TIM6->CR1 |= TIM_CR1_CEN;

    NVIC_SetPriority(DSP_IRQ, PRIO_DSP);
    NVIC_EnableIRQ(DSP_IRQ);
    NVIC_SetPriority(DMA2_Channel1_IRQn, PRIO_DSP);
    NVIC_EnableIRQ(DMA2_Channel1_IRQn);
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
        dac_hw[i] = 0x8000;
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
        s->dc_init = true;
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
    return (float)(int32_t)(s->w - s->r) + (float)q / s->ratio;
}

static float in_queued(void)
{
    stream_t *s = &in_s;
    uint32_t q = (uint16_t)(dma_pos(s) - s->done) & ADC_M;
    if (q > ADC_N / 2) q = 0;
    return (float)(int32_t)(s->w - s->r) + (float)q / s->ratio;
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
    const bool lpf = s->lp_on;
    uint32_t peak = 0;
    for (uint16_t i = 0; i < bytes / 2; i++) {
        int32_t v = ((int32_t)(int16_t)p[2 * i] * g) >> 12;
        if (lpf)
            v = (int32_t)lrintf(lp_run(s, (float)v));
        if (v > 32767) v = 32767;
        if (v < -32768) v = -32768;
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
        if (s->dc_init) {
            s->dc = x << 12;                          /* no settling time */
            s->dc_init = false;
        }
        if (dcb) {
            s->dc += ((x << 12) - s->dc) >> 12;      /* ~1.9 Hz at 48 kHz */
            x -= s->dc >> 12;
        }
        int32_t v = (x * g) >> 12;
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

    set_ratio(&out_s);
    set_ratio(&in_s);
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
                    st->pkt_acc = 0;
                    set_ratio(st);
                    lp_design(st, st == &in_s ? cfg.lp_rx : cfg.lp_tx);
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
    st->osr = cfg.oversample ? s->ratio : 1.0f;
    st->fill = (int32_t)lrintf(s->fill);
    st->ppm = s->ppm;
    st->xruns = s->xruns;
    st->peak = s->peak;
    s->peak = 0;
    __enable_irq();
}

float audio_frame_ppm(void) { return (frame_clk / 72000.0f - 1.0f) * 1e6f; }
float audio_dsp_load(void) { return dsp_load; }
