/* Vector table and reset handler */
#include "board.h"
#include "dfu.h"

extern uint32_t _estack, _sidata, _sdata, _edata, _sbss, _ebss;
int main(void);

void Reset_Handler(void);
void Default_Handler(void);

/* A fault or unexpected interrupt: halt here (visible with a debugger). */
void Default_Handler(void) { __disable_irq(); for (;;) __WFI(); }

#define WEAK __attribute__((weak, alias("Default_Handler")))
void NMI_Handler(void) WEAK;          void HardFault_Handler(void) WEAK;
void MemManage_Handler(void) WEAK;    void BusFault_Handler(void) WEAK;
void UsageFault_Handler(void) WEAK;   void SVC_Handler(void) WEAK;
void DebugMon_Handler(void) WEAK;     void PendSV_Handler(void) WEAK;
void SysTick_Handler(void) WEAK;
void RCC_IRQHandler(void) WEAK;
void FLASH_IRQHandler(void) WEAK;
void DMA1_Channel4_IRQHandler(void) WEAK;
void DMA1_Channel5_IRQHandler(void) WEAK;
void USB_HP_CAN_TX_IRQHandler(void) WEAK;
void USB_LP_CAN_RX0_IRQHandler(void) WEAK;
void USART1_IRQHandler(void) WEAK;

#define NIRQ 82
typedef void (*vec_t)(void);
#pragma GCC diagnostic ignored "-Woverride-init"
__attribute__((section(".isr_vector"), used))
const vec_t vectors[16 + NIRQ] = {
    [0]  = (vec_t)&_estack,
    [1]  = Reset_Handler,
    [2]  = NMI_Handler,
    [3]  = HardFault_Handler,
    [4]  = MemManage_Handler,
    [5]  = BusFault_Handler,
    [6]  = UsageFault_Handler,
    [11] = SVC_Handler,
    [12] = DebugMon_Handler,
    [14] = PendSV_Handler,
    [15] = SysTick_Handler,
    [16 ... 16 + NIRQ - 1] = Default_Handler,
    [16 + 4]  = FLASH_IRQHandler,
    [16 + 5]  = RCC_IRQHandler,
    [16 + 14] = DMA1_Channel4_IRQHandler,
    [16 + 15] = DMA1_Channel5_IRQHandler,
    [16 + 19] = USB_HP_CAN_TX_IRQHandler,
    [16 + 20] = USB_LP_CAN_RX0_IRQHandler,
    [16 + 37] = USART1_IRQHandler,
};

void Reset_Handler(void)
{
    /* Before touching anything: a requested jump to the ROM DFU bootloader. */
    dfu_check_boot();

    uint32_t *s = &_sidata, *d = &_sdata;
    while (d < &_edata) *d++ = *s++;
    for (d = &_sbss; d < &_ebss; ) *d++ = 0;

    SCB->CPACR |= (0xFu << 20);     /* FPU: full access to CP10/CP11 */
    __DSB(); __ISB();

    main();
    for (;;) __WFI();
}
