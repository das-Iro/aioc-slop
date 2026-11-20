/* aioc-slop: alternative firmware for the AIOC (All-In-One-Cable) */
#include "board.h"
#include "clock.h"
#include "settings.h"
#include "menu.h"
#include "ptt.h"
#include "uart.h"
#include "audio.h"
#include "usb.h"

static void gpio_init(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;

    /* PA0/PA1 PTT outputs (low = off); PA2..PA8 analog (PA4 = DAC);
     * PA9..PA15 keep AF/reset state (UART and USB set later, SWD) */
    GPIOA->BRR = (1u << PTT1_PIN) | (1u << PTT2_PIN);
    GPIOA->MODER = (GPIOA->MODER & 0xFFFC0000u) | 0x0003FFF5u;

    /* PB2 analog (ADC), PB3/PB4 keep reset state (SWO, NJTRST),
     * PB8/PB9 LED outputs, everything else analog */
    GPIOB->BRR = (1u << LED_A_PIN) | (1u << LED_B_PIN);
    GPIOB->MODER = (GPIOB->MODER & (0xFu << 6)) | 0xFFF5FC3Fu;

    GPIOC->MODER = 0xFFFFFFFFu;
}

int main(void)
{
    clock_init();
    gpio_init();
    menu_init();          /* loads the settings */
    ptt_init();
    uart_init();
    audio_init();
    usb_init();

    /* Everything runs in interrupts. The core never sleeps: WFI made the
     * supply current step with every burst of interrupt work, which showed
     * up as 1 kHz lines and 5..8 dB more noise on the RX audio. */
    for (;;)
        __NOP();
}
