/* Minimal USB device stack for the STM32F302xB/C USB FS peripheral
 * (512 B packet memory, 1x16 bit/word access scheme) */
#pragma once
#include "board.h"

typedef struct __attribute__((packed)) {
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} usb_setup_t;

/* Interfaces */
enum {
    IF_CFG_COMM, IF_CFG_DATA,       /* CDC-ACM: configuration menu */
    IF_RADIO_COMM, IF_RADIO_DATA,   /* CDC-ACM: UART bridge to the radio */
    IF_AC, IF_AS_OUT, IF_AS_IN,     /* USB audio class 1 */
    IF_DFU,                         /* DFU runtime (detach to ROM bootloader) */
    IF_COUNT
};

/* Endpoints (EPnR index = endpoint number) */
#define EP_AUDIO_OUT    1   /* 0x01 iso OUT: host -> radio mic */
#define EP_AUDIO_IN     2   /* 0x82 iso IN:  radio speaker -> host */
#define EP_CFG          3   /* 0x03/0x83 bulk */
#define EP_CFG_NOTIF    4   /* 0x84 interrupt */
#define EP_RADIO        5   /* 0x05/0x85 bulk */
#define EP_RADIO_NOTIF  6   /* 0x86 interrupt */
#define EP_COUNT        7

#define EP0_SIZE        32
#define CDC_PKT         32
#define AUDIO_PKT       96  /* 48 samples * 16 bit mono */

extern volatile bool usb_configured;
extern volatile bool usb_suspended;

void usb_init(void);

/* Bulk/interrupt endpoints */
void     usb_ep_tx(uint8_t ep, const uint8_t *src, uint16_t n);
uint16_t usb_ep_rx(uint8_t ep, uint8_t *dst);   /* copies the received packet */
void     usb_ep_rx_enable(uint8_t ep);

/* Isochronous endpoints: both double-buffer descriptors point at one PMA
 * buffer; data is copied in/out inside the CTR interrupt. */
volatile uint16_t *usb_ep_pma(uint8_t ep);      /* halfword i is at [2*i] */
uint16_t usb_iso_rx_count(uint8_t ep);
void     usb_iso_tx_count(uint8_t ep, uint16_t n);
void     usb_iso_enable(uint8_t ep, bool on);

/* Descriptors (usb_desc.c) */
extern const uint8_t usb_dev_desc[18];
extern uint8_t usb_cfg_desc[];
extern const uint16_t usb_cfg_desc_len;
int usb_string_desc(uint8_t idx, uint8_t *buf, uint16_t max);
const char *usb_serial(void);
void usb_desc_init(void);
