#pragma once
#include "usb.h"

void cdc_reset(void);
int  cdc_setup(const usb_setup_t *s, uint8_t *buf, const uint8_t **in);
void cdc_rx(uint8_t ep);
void cdc_tx_done(uint8_t ep);
void cdc_kick(void);               /* from the end of the USB ISR */

/* Config port, used by the menu (PendSV) */
void cfg_write(const char *s, uint16_t n);
bool cfg_getc(uint8_t *c);
bool cfg_dtr(void);
void cfg_flush(void);
const uint8_t *radio_line_coding(void);   /* 7 bytes, CDC format */
