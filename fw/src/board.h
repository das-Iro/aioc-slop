/* AIOC rev 1.0 hardware (STM32F302CBT6, 8 MHz crystal) */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "stm32f302xc.h"

#define FW_VERSION  "0.1"
#define USB_VID     0x1209   /* pid.codes */
#define USB_PID     0x0001   /* pid.codes test PID: development only */

#define SYSCLK_HZ   72000000u

/* GPIO */
#define PTT1_PIN    1   /* PA1 -> Q1B, pulls the sleeve (RX-PTT1) low */
#define PTT2_PIN    0   /* PA0 -> Q1A, tip of the 3.5 mm plug */
#define LED_A_PIN   8   /* PB8 (through R15) */
#define LED_B_PIN   9   /* PB9; green/red LEDs are antiparallel between them */

/* Interrupt priorities, lower = more urgent. Everything that touches USB
 * endpoints or shared ring indices runs at PRIO_IO so none of it preempts
 * the rest. The menu runs in PendSV at the lowest priority. */
#define PRIO_IO     1
#define PRIO_DELAY  2
#define PRIO_DSP    3   /* audio filters, pended from the SOF */
#define PRIO_MENU   15

#define USB_IRQ     USB_LP_CAN_RX0_IRQn
#define DSP_IRQ     EXTI0_IRQn      /* unused EXTI line, pended in software */

/* Ask the USB ISR to run its deferred work (endpoint kicks). */
static inline void usb_pend(void) { NVIC_SetPendingIRQ(USB_IRQ); }
/* Ask the menu (PendSV) to run. */
static inline void menu_pend(void) { SCB->ICSR = SCB_ICSR_PENDSVSET_Msk; }

/* Sleep until cond is true. WFI with PRIMASK set still wakes on a pending
 * interrupt, so a wakeup between the test and the WFI cannot be lost. */
#define WAIT_UNTIL(cond) do { __disable_irq(); \
        while (!(cond)) { __WFI(); __enable_irq(); __ISB(); __disable_irq(); } \
        __enable_irq(); } while (0)

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
