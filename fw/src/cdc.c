/* Two CDC-ACM ports: config menu and the radio UART bridge */
#include "cdc.h"
#include "uart.h"
#include "ptt.h"
#include "menu.h"

#define CFG_TX_N 4096u
#define CFG_RX_N 256u

static uint8_t cfg_tx[CFG_TX_N];
static volatile uint16_t cfg_tx_head, cfg_tx_tail;
static uint8_t cfg_rx[CFG_RX_N];
static volatile uint16_t cfg_rx_head, cfg_rx_tail;

typedef struct {
    uint8_t coding[7];
    uint8_t lines;
    bool in_busy;
    bool zlp;
    bool rx_paused;
} port_t;

static port_t port[2] = {
    { .coding = { 0x00, 0xC2, 0x01, 0x00, 0, 0, 8 } },   /* 115200 8N1 */
    { .coding = { 0x80, 0x25, 0x00, 0x00, 0, 0, 8 } },   /* 9600 8N1 */
};

void cdc_reset(void)
{
    for (int i = 0; i < 2; i++) {
        port[i].lines = 0;
        port[i].in_busy = port[i].zlp = port[i].rx_paused = false;
    }
    cfg_tx_head = cfg_tx_tail = 0;
    ptt_set_lines(0);
    menu_dtr(false);
}

bool cfg_dtr(void) { return port[0].lines & 1; }
const uint8_t *radio_line_coding(void) { return port[1].coding; }

int cdc_setup(const usb_setup_t *s, uint8_t *buf, const uint8_t **in)
{
    int idx = (s->wIndex & 0xFF) == IF_CFG_COMM ? 0 : 1;
    port_t *p = &port[idx];
    switch (s->bRequest) {
    case 0x20:                                       /* SET_LINE_CODING */
        for (int i = 0; i < 7; i++)
            p->coding[i] = buf[i];
        if (idx == 1)
            uart_set_coding(buf[0] | buf[1] << 8 | (uint32_t)buf[2] << 16 |
                            (uint32_t)buf[3] << 24, buf[4], buf[5], buf[6]);
        return 0;
    case 0x21:                                       /* GET_LINE_CODING */
        *in = p->coding;
        return 7;
    case 0x22: {                                     /* SET_CONTROL_LINE_STATE */
        uint8_t old = p->lines;
        p->lines = s->wValue & 3;
        if (idx == 1)
            ptt_set_lines(p->lines);
        else if ((old ^ p->lines) & 1)
            menu_dtr(p->lines & 1);
        return 0;
    }
    case 0x23:                                       /* SEND_BREAK */
        return 0;
    }
    return -1;
}

static uint16_t cfg_rx_space(void)
{
    return CFG_RX_N - 1 - ((cfg_rx_head - cfg_rx_tail) & (CFG_RX_N - 1));
}

void cdc_rx(uint8_t ep)
{
    uint8_t buf[CDC_PKT];
    uint16_t n = usb_ep_rx(ep, buf);
    if (ep == EP_CFG) {
        for (uint16_t i = 0; i < n; i++) {
            cfg_rx[cfg_rx_head] = buf[i];
            cfg_rx_head = (cfg_rx_head + 1) & (CFG_RX_N - 1);
        }
        /* A lone ESC at the end of a packet is the Esc key, not the start
         * of an escape sequence: mark it with a NUL. */
        if (n && buf[n - 1] == 0x1B) {
            cfg_rx[cfg_rx_head] = 0;
            cfg_rx_head = (cfg_rx_head + 1) & (CFG_RX_N - 1);
        }
        menu_pend();
        if (cfg_rx_space() > CDC_PKT + 1)
            usb_ep_rx_enable(ep);
        else
            port[0].rx_paused = true;
    } else {
        uart_write(buf, n);
        if (uart_tx_space() >= CDC_PKT)
            usb_ep_rx_enable(ep);
        else
            port[1].rx_paused = true;
    }
}

void cdc_tx_done(uint8_t ep)
{
    port[ep == EP_CFG ? 0 : 1].in_busy = false;
}

static void send(uint8_t ep, port_t *p, const uint8_t *buf, uint16_t n)
{
    usb_ep_tx(ep, buf, n);
    p->in_busy = true;
    p->zlp = n == CDC_PKT;
}

void cdc_kick(void)
{
    if (!usb_configured)
        return;
    uint8_t buf[CDC_PKT];

    /* config port */
    port_t *p = &port[0];
    if (p->rx_paused && cfg_rx_space() > CDC_PKT + 1) {
        p->rx_paused = false;
        usb_ep_rx_enable(EP_CFG);
    }
    if (!p->in_busy) {
        uint16_t n = 0;
        while (n < CDC_PKT && cfg_tx_tail != cfg_tx_head) {
            buf[n++] = cfg_tx[cfg_tx_tail];
            cfg_tx_tail = (cfg_tx_tail + 1) & (CFG_TX_N - 1);
        }
        if (n || p->zlp)
            send(EP_CFG, p, buf, n);
    }

    /* radio port */
    p = &port[1];
    if (p->rx_paused && uart_tx_space() >= CDC_PKT) {
        p->rx_paused = false;
        usb_ep_rx_enable(EP_RADIO);
    }
    if (!p->in_busy) {
        uint16_t n = uart_rx_read(buf, CDC_PKT);
        if (n || p->zlp)
            send(EP_RADIO, p, buf, n);
    }
}

/* ---- menu side (PendSV) ------------------------------------------------- */
void cfg_write(const char *s, uint16_t n)
{
    if (!cfg_dtr())
        return;
    for (uint16_t i = 0; i < n; i++) {
        uint16_t next = (cfg_tx_head + 1) & (CFG_TX_N - 1);
        if (next == cfg_tx_tail)
            break;                                   /* full: drop */
        cfg_tx[cfg_tx_head] = (uint8_t)s[i];
        cfg_tx_head = next;
    }
    usb_pend();
}

void cfg_flush(void) { cfg_tx_head = cfg_tx_tail; }

bool cfg_getc(uint8_t *c)
{
    if (cfg_rx_tail == cfg_rx_head)
        return false;
    *c = cfg_rx[cfg_rx_tail];
    cfg_rx_tail = (cfg_rx_tail + 1) & (CFG_RX_N - 1);
    if (port[0].rx_paused)
        usb_pend();
    return true;
}
