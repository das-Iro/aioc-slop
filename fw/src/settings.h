#pragma once
#include "board.h"

enum { PTT_SRC_OFF, PTT_SRC_DTR, PTT_SRC_RTS, PTT_SRC_DTR_NRTS, PTT_SRC_RTS_NDTR,
       PTT_SRC_COUNT };

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    int16_t  vol_out;      /* host -> radio, 1/256 dB (UAC1 units) */
    int16_t  vol_in;       /* radio -> host */
    uint8_t  mute_out;
    uint8_t  mute_in;
    uint8_t  buf_ms;       /* audio ring target fill, ms */
    uint8_t  dc_block;     /* high-pass on the radio -> host path */
    uint8_t  ptt_src[2];   /* PTT1, PTT2: serial control line source */
    uint8_t  ptt_vox[2];   /* PTT1, PTT2: keyed by VOX too */
    uint16_t tot_s;        /* PTT time-out, 0 = off */
    int16_t  vox_thr;      /* dBFS */
    uint16_t vox_hang;     /* ms */
    uint8_t  led_en;
    uint8_t  sleep_en;     /* WFI in idle; off = busy idle (noise test) */
    uint8_t  pad[4];
    uint32_t crc;
} settings_t;

extern settings_t cfg;

void settings_defaults(void);
bool settings_load(void);   /* false: flash empty/invalid, defaults used */
bool settings_save(void);   /* from PendSV only; stalls the CPU ~40 ms */
