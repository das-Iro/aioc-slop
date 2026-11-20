#pragma once
#include "board.h"
void     uart_init(void);
void     uart_set_coding(uint32_t baud, uint8_t stop, uint8_t parity, uint8_t bits);
uint16_t uart_rx_avail(void);
uint16_t uart_rx_read(uint8_t *dst, uint16_t max);
uint16_t uart_tx_space(void);
void     uart_write(const uint8_t *src, uint16_t n);
extern volatile uint32_t uart_rx_bytes, uart_tx_bytes;
