#pragma once
#include "board.h"

void ptt_init(void);
void ptt_update(void);              /* re-evaluate PTT outputs and LEDs */
void ptt_set_lines(uint8_t lines);  /* radio port control lines: bit0 DTR, bit1 RTS */
uint8_t ptt_lines(void);
void ptt_manual(int n, bool on);    /* from the menu */
bool ptt_manual_get(int n);
void ptt_sof(void);                 /* 1 ms tick from the USB SOF */
void vox_feed(uint32_t peak);       /* peak of each host -> radio packet */
void vox_thr_update(void);          /* after cfg.vox_thr changed */
bool ptt_state(int n);
bool vox_active(void);
bool ptt_tot_tripped(void);
uint16_t vox_level(void);           /* envelope, 0..32767 */
