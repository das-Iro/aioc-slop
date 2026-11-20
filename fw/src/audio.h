#pragma once
#include "board.h"
#include "usb.h"

/* UAC1 entity IDs */
#define UAC_IT_USB_OUT  1
#define UAC_FU_OUT      2
#define UAC_OT_SPK      3
#define UAC_IT_MIC      4
#define UAC_FU_IN       5
#define UAC_OT_USB_IN   6

#define VOL_MIN   (-60 * 256)   /* 1/256 dB */
#define VOL_MAX   0
#define VOL_RES   128           /* 0.5 dB */

typedef struct {
    uint32_t fs;        /* sample rate */
    bool     active;    /* host is streaming (alt 1) */
    uint8_t  osr;       /* oversampling ratio in use (1 = off) */
    int32_t  fill;      /* samples queued (rings + DMA), deviation from setpoint */
    float    ppm;       /* clock correction from the buffer fill loop */
    uint32_t xruns;     /* resyncs (under/overruns) */
    uint16_t peak;      /* peak level since the last read, 0..32767 */
} audio_stat_t;

void audio_init(void);
void audio_reset(void);                 /* bus reset / (de)configuration */
void audio_suspend(void);
void audio_resync(void);                /* after long interrupt stalls */
void audio_gain_update(void);           /* after cfg.vol_* / mute_* changed */
void audio_sof(uint32_t t, uint16_t fn);
int  audio_setup(const usb_setup_t *s, uint8_t *buf, const uint8_t **in);
void audio_set_alt(uint8_t itf, uint8_t alt);
void audio_out_packet(void);            /* EP1 CTR_RX */
void audio_in_done(void);               /* EP2 CTR_TX */

/* For the menu. out = host->radio, in = radio->host */
void  audio_stat(bool in, audio_stat_t *st);   /* clears peak */
float audio_frame_ppm(void);            /* host clock vs. crystal */
float audio_dsp_load(void);             /* filter CPU load, percent */
void  audio_reconfigure(void);          /* after cfg.oversample changed */
