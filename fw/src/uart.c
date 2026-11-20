/* USART1 (PA9 TX -> radio, PA10 RX <- radio) with DMA both ways.
 * Runs at PRIO_IO like the USB ISR, so the rings need no locking. */
#include "uart.h"

#define RX_N 512u
#define TX_N 1024u

static uint8_t rx_ring[RX_N];
static uint16_t rx_rd;
static uint8_t tx_ring[TX_N];
static volatile uint16_t tx_head, tx_tail, tx_len;
volatile uint32_t uart_rx_bytes, uart_tx_bytes;

static uint16_t rx_wr(void) { return (uint16_t)(RX_N - DMA1_Channel5->CNDTR) & (RX_N - 1); }

uint16_t uart_rx_avail(void) { return (uint16_t)(rx_wr() - rx_rd) & (RX_N - 1); }

uint16_t uart_rx_read(uint8_t *dst, uint16_t max)
{
    uint16_t n = MIN(uart_rx_avail(), max);
    for (uint16_t i = 0; i < n; i++)
        dst[i] = rx_ring[(rx_rd + i) & (RX_N - 1)];
    rx_rd = (rx_rd + n) & (RX_N - 1);
    uart_rx_bytes += n;
    return n;
}

uint16_t uart_tx_space(void) { return TX_N - 1 - ((tx_head - tx_tail) & (TX_N - 1)); }

static void tx_start(void)
{
    if (tx_len || tx_head == tx_tail)
        return;
    uint16_t n = (uint16_t)(tx_head > tx_tail ? (uint32_t)(tx_head - tx_tail) : TX_N - tx_tail);
    tx_len = n;
    DMA1_Channel4->CCR &= ~DMA_CCR_EN;
    DMA1_Channel4->CMAR = (uint32_t)&tx_ring[tx_tail];
    DMA1_Channel4->CNDTR = n;
    DMA1_Channel4->CCR |= DMA_CCR_EN;
}

void uart_write(const uint8_t *src, uint16_t n)
{
    n = MIN(n, uart_tx_space());
    for (uint16_t i = 0; i < n; i++) {
        tx_ring[tx_head] = src[i];
        tx_head = (tx_head + 1) & (TX_N - 1);
    }
    uart_tx_bytes += n;
    tx_start();
}

void DMA1_Channel4_IRQHandler(void)   /* TX done */
{
    DMA1->IFCR = DMA_IFCR_CGIF4;
    tx_tail = (tx_tail + tx_len) & (TX_N - 1);
    tx_len = 0;
    tx_start();
    usb_pend();                       /* space for a paused OUT endpoint */
}

void DMA1_Channel5_IRQHandler(void)   /* RX half/full */
{
    DMA1->IFCR = DMA_IFCR_CGIF5;
    usb_pend();
}

void USART1_IRQHandler(void)          /* RX idle line: flush what we have */
{
    USART1->ICR = USART_ICR_IDLECF | USART_ICR_ORECF | USART_ICR_FECF |
                  USART_ICR_PECF | USART_ICR_NCF;
    usb_pend();
}

void uart_set_coding(uint32_t baud, uint8_t stop, uint8_t parity, uint8_t bits)
{
    if (baud < 1200) baud = 1200;
    if (baud > 4500000) baud = 4500000;
    uint32_t cr1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE;
    if (parity == 1 || parity == 2) {                /* odd / even */
        cr1 |= USART_CR1_PCE | (parity == 1 ? USART_CR1_PS : 0);
        if (bits == 8)
            cr1 |= USART_CR1_M0;                     /* 8 data + parity */
    }
    uint32_t cr2 = stop == 1 ? USART_CR2_STOP_0 | USART_CR2_STOP_1 :   /* 1.5 */
                   stop == 2 ? USART_CR2_STOP_1 : 0;
    USART1->CR1 = 0;
    USART1->BRR = (SYSCLK_HZ + baud / 2) / baud;
    USART1->CR2 = cr2;
    USART1->CR3 = USART_CR3_DMAR | USART_CR3_DMAT | USART_CR3_OVRDIS;
    USART1->CR1 = cr1 | USART_CR1_UE;
}

void uart_init(void)
{
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /* PA9/PA10 AF7, pull-up on RX */
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xFFu << 4)) | (0x77u << 4);
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(3u << 20)) | (1u << 20);
    GPIOA->MODER = (GPIOA->MODER & ~(0xFu << 18)) | (0xAu << 18);

    DMA1_Channel5->CPAR = (uint32_t)&USART1->RDR;
    DMA1_Channel5->CMAR = (uint32_t)rx_ring;
    DMA1_Channel5->CNDTR = RX_N;
    DMA1_Channel5->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_HTIE | DMA_CCR_TCIE | DMA_CCR_EN;

    DMA1_Channel4->CPAR = (uint32_t)&USART1->TDR;
    DMA1_Channel4->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE;

    uart_set_coding(9600, 0, 0, 8);

    NVIC_SetPriority(DMA1_Channel4_IRQn, PRIO_IO);
    NVIC_SetPriority(DMA1_Channel5_IRQn, PRIO_IO);
    NVIC_SetPriority(USART1_IRQn, PRIO_IO);
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    NVIC_EnableIRQ(USART1_IRQn);
}
