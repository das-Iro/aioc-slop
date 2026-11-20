#pragma once
#include "board.h"
void clock_init(void);          /* HSE 8 MHz -> PLL 72 MHz, USB 48 MHz */
/* Sleep for us microseconds (SysTick one-shot + WFI). Only from thread mode
 * or PendSV: it needs the SysTick interrupt to preempt the caller. */
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void fatal(void);               /* fast red/green blink forever */
