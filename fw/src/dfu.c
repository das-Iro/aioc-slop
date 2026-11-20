/* Jump into the STM32 ROM bootloader (USB DFU, 0483:df11).
 * The board has a fixed 1.5 k pull-up on D+, so the firmware forces a
 * disconnect by driving D+ low for a while before resetting. */
#include "dfu.h"
#include "clock.h"

#define DFU_MAGIC   0xB007D00Du
#define SYSMEM_BASE 0x1FFFD800u  /* F302xB/C system memory (AN2606) */

__attribute__((section(".noinit"))) static volatile uint32_t dfu_magic;
static volatile uint8_t pending;   /* 1 = reset, 2 = bootloader */

void dfu_check_boot(void)
{
    if (dfu_magic != DFU_MAGIC)
        return;
    dfu_magic = 0;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->CFGR1 = (SYSCFG->CFGR1 & ~SYSCFG_CFGR1_MEM_MODE) | SYSCFG_CFGR1_MEM_MODE_0;
    uint32_t sp = *(volatile uint32_t *)SYSMEM_BASE;
    uint32_t pc = *(volatile uint32_t *)(SYSMEM_BASE + 4);
    __set_MSP(sp);
    ((void (*)(void))pc)();
    for (;;) ;
}

void usb_reconnect_pulse(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->BRR = 1u << 12;
    GPIOA->MODER = (GPIOA->MODER & ~(3u << 24)) | (1u << 24);  /* PA12 output low */
    delay_ms(20);
    GPIOA->MODER &= ~(3u << 24);                               /* back to input */
}

void dfu_request(void)
{
    pending = 2;
    menu_pend();
}

void reset_request(void)
{
    pending = 1;
    menu_pend();
}

void dfu_run_pending(void)
{
    if (!pending)
        return;
    /* PTT off before anything else */
    GPIOA->BRR = (1u << PTT1_PIN) | (1u << PTT2_PIN);
    delay_ms(10);                 /* let the status stage of DFU_DETACH finish */
    NVIC_DisableIRQ(USB_LP_CAN_RX0_IRQn);
    NVIC_DisableIRQ(USB_HP_CAN_TX_IRQn);
    USB->CNTR = USB_CNTR_FRES | USB_CNTR_PDWN;
    usb_reconnect_pulse();
    if (pending == 2)
        dfu_magic = DFU_MAGIC;
    NVIC_SystemReset();
}
