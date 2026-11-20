#include "clock.h"

static uint32_t core_mhz = 8;   /* HSI until clock_init() switches */
static volatile bool tick_done;

void SysTick_Handler(void)
{
    SysTick->CTRL = 0;
    tick_done = true;
}

static void tick_start(uint32_t us)
{
    tick_done = false;
    SysTick->LOAD = us * core_mhz - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;
}

void delay_us(uint32_t us)
{
    while (us) {
        uint32_t n = MIN(us, 200000u);   /* 24-bit LOAD at 72 MHz */
        tick_start(n);
        WAIT_UNTIL(tick_done);
        us -= n;
    }
}

void delay_ms(uint32_t ms) { delay_us(ms * 1000u); }

void RCC_IRQHandler(void)
{
    RCC->CIR |= RCC_CIR_HSERDYC | RCC_CIR_PLLRDYC;
}

void fatal(void)
{
    __disable_irq();
    /* No interrupts here, so blink with a cycle-count delay. */
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER = (GPIOB->MODER & ~(0xFu << (LED_A_PIN * 2))) |
                   (0x5u << (LED_A_PIN * 2));
    for (;;) {
        GPIOB->ODR ^= (1u << LED_A_PIN) | (1u << LED_B_PIN);
        for (volatile uint32_t i = 0; i < 400000; i++) ;
    }
}

void clock_init(void)
{
    NVIC_SetPriority(SysTick_IRQn, PRIO_DELAY);
    NVIC_SetPriority(RCC_IRQn, PRIO_DELAY);
    RCC->CIR = RCC_CIR_HSERDYIE | RCC_CIR_PLLRDYIE;
    NVIC_EnableIRQ(RCC_IRQn);

    /* HSE, with a 100 ms timeout */
    RCC->CR |= RCC_CR_HSEON;
    tick_start(100000);
    WAIT_UNTIL((RCC->CR & RCC_CR_HSERDY) || tick_done);
    SysTick->CTRL = 0;
    if (!(RCC->CR & RCC_CR_HSERDY))
        fatal();

    /* 72 MHz needs 2 wait states */
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_1;

    /* PLL = 8 MHz * 9 = 72 MHz; APB1 = 36 MHz (its timers run at 72 MHz);
     * APB2 = 72 MHz; USB = PLL / 1.5 = 48 MHz (USBPRE = 0) */
    RCC->CFGR2 = 0;
    RCC->CFGR = RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_PLLMUL9 | RCC_CFGR_PPRE1_DIV2;
    RCC->CR |= RCC_CR_PLLON;
    WAIT_UNTIL(RCC->CR & RCC_CR_PLLRDY);

    RCC->CFGR |= RCC_CFGR_SW_PLL;
    /* The switch takes a few clock cycles; check once instead of spinning. */
    __DSB(); __NOP(); __NOP(); __NOP(); __NOP();
    if ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        fatal();
    core_mhz = 72;

    RCC->CIR = 0;
    NVIC_DisableIRQ(RCC_IRQn);
}
