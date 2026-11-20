#pragma once
#include "board.h"
void dfu_check_boot(void);   /* called first thing in Reset_Handler */
void dfu_request(void);      /* from any context: enter the bootloader soon */
void reset_request(void);    /* from any context: plain reset with re-enumeration */
void dfu_run_pending(void);  /* from PendSV: does the actual detach + reset */
void usb_reconnect_pulse(void); /* hold D+ low so the host sees a disconnect */
