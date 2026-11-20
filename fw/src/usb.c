#include "usb.h"
#include "clock.h"
#include "dfu.h"
#include "audio.h"
#include "cdc.h"
#include "ptt.h"
#include "menu.h"

volatile bool usb_configured;
volatile bool usb_suspended;

/* ---- packet memory ---------------------------------------------------- */
#define EPR(n)      (*(volatile uint16_t *)(USB_BASE + 4u * (n)))
/* USB-side byte address -> CPU pointer (1x16 bit/word: 32-bit stride) */
#define PMA16(a)    ((volatile uint16_t *)(USB_PMAADDR + 2u * (a)))
#define BT_ADDR_TX(n)  (*PMA16((n) * 8u + 0))
#define BT_COUNT_TX(n) (*PMA16((n) * 8u + 2))
#define BT_ADDR_RX(n)  (*PMA16((n) * 8u + 4))
#define BT_COUNT_RX(n) (*PMA16((n) * 8u + 6))

/* PMA layout (bytes, USB side). Total 456 of 512. */
#define PMA_EP0_TX      56
#define PMA_EP0_RX      (PMA_EP0_TX + EP0_SIZE)
#define PMA_AUDIO_OUT   (PMA_EP0_RX + EP0_SIZE)
#define PMA_AUDIO_IN    (PMA_AUDIO_OUT + AUDIO_PKT)
#define PMA_CFG_TX      (PMA_AUDIO_IN + AUDIO_PKT)
#define PMA_CFG_RX      (PMA_CFG_TX + CDC_PKT)
#define PMA_CFG_NOTIF   (PMA_CFG_RX + CDC_PKT)
#define PMA_RADIO_TX    (PMA_CFG_NOTIF + 8)
#define PMA_RADIO_RX    (PMA_RADIO_TX + CDC_PKT)
#define PMA_RADIO_NOTIF (PMA_RADIO_RX + CDC_PKT)
#define PMA_END         (PMA_RADIO_NOTIF + 8)
_Static_assert(PMA_END <= 512, "packet memory overflow");

/* COUNT_RX block size field */
#define RXBLK(n) ((n) > 62 ? (0x8000u | (((n) / 32u - 1u) << 10)) : (((n) / 2u) << 10))

static void pma_write(uint16_t addr, const uint8_t *src, uint16_t n)
{
    volatile uint16_t *p = PMA16(addr);
    for (uint16_t i = 0; i < n; i += 2) {
        uint16_t v = src[i];
        if (i + 1 < n) v |= (uint16_t)src[i + 1] << 8;
        p[i] = v;      /* halfword i/2 lives at p[2*(i/2)] = p[i] */
    }
}

static void pma_read(uint16_t addr, uint8_t *dst, uint16_t n)
{
    volatile uint16_t *p = PMA16(addr);
    for (uint16_t i = 0; i < n; i += 2) {
        uint16_t v = p[i];
        dst[i] = (uint8_t)v;
        if (i + 1 < n) dst[i + 1] = (uint8_t)(v >> 8);
    }
}

/* ---- endpoint registers ----------------------------------------------- */
/* Set EP type/address/kind and the STAT bits; optionally reset DTOGs.
 * STAT and DTOG bits are toggle-on-write-1, CTR bits clear-on-write-0. */
static void ep_config(uint8_t n, uint16_t type, uint16_t stat_rx, uint16_t stat_tx)
{
    uint16_t v = EPR(n);
    uint16_t w = type | n | USB_EP_CTR_RX | USB_EP_CTR_TX;
    w |= (v ^ stat_rx) & USB_EPRX_STAT;
    w |= (v ^ stat_tx) & USB_EPTX_STAT;
    w |= v & (USB_EP_DTOG_RX | USB_EP_DTOG_TX);    /* toggle back to 0 */
    EPR(n) = w;
}

static void ep_stat_tx(uint8_t n, uint16_t stat)
{
    uint16_t v = EPR(n);
    EPR(n) = (v & USB_EPREG_MASK) | USB_EP_CTR_RX | USB_EP_CTR_TX |
             ((v ^ stat) & USB_EPTX_STAT);
}

static void ep_stat_rx(uint8_t n, uint16_t stat)
{
    uint16_t v = EPR(n);
    EPR(n) = (v & USB_EPREG_MASK) | USB_EP_CTR_RX | USB_EP_CTR_TX |
             ((v ^ stat) & USB_EPRX_STAT);
}

static void ep_clear_ctr_rx(uint8_t n)
{
    EPR(n) = (EPR(n) & USB_EPREG_MASK & ~USB_EP_CTR_RX) | USB_EP_CTR_TX;
}

static void ep_clear_ctr_tx(uint8_t n)
{
    EPR(n) = (EPR(n) & USB_EPREG_MASK & ~USB_EP_CTR_TX) | USB_EP_CTR_RX;
}

/* ---- public endpoint API ---------------------------------------------- */
void usb_ep_tx(uint8_t ep, const uint8_t *src, uint16_t n)
{
    pma_write(BT_ADDR_TX(ep), src, n);
    BT_COUNT_TX(ep) = n;
    ep_stat_tx(ep, USB_EP_TX_VALID);
}

uint16_t usb_ep_rx(uint8_t ep, uint8_t *dst)
{
    uint16_t n = BT_COUNT_RX(ep) & 0x3FF;
    pma_read(BT_ADDR_RX(ep), dst, n);
    return n;
}

void usb_ep_rx_enable(uint8_t ep) { ep_stat_rx(ep, USB_EP_RX_VALID); }

volatile uint16_t *usb_ep_pma(uint8_t ep) { return PMA16(BT_ADDR_TX(ep)); }

uint16_t usb_iso_rx_count(uint8_t ep)
{
    /* DTOG_RX already toggled: if 0 the peripheral now uses buffer 0, so
     * the packet just received is in buffer 1 (descriptor at +4/+6). */
    if (EPR(ep) & USB_EP_DTOG_RX)
        return BT_COUNT_TX(ep) & 0x3FF;
    return BT_COUNT_RX(ep) & 0x3FF;
}

void usb_iso_tx_count(uint8_t ep, uint16_t n)
{
    BT_COUNT_TX(ep) = n;   /* buffer 0 */
    BT_COUNT_RX(ep) = n;   /* buffer 1 */
}

void usb_iso_enable(uint8_t ep, bool on)
{
    if (ep == EP_AUDIO_OUT)
        ep_config(ep, USB_EP_ISOCHRONOUS, on ? USB_EP_RX_VALID : USB_EP_RX_DIS, USB_EP_TX_DIS);
    else
        ep_config(ep, USB_EP_ISOCHRONOUS, USB_EP_RX_DIS, on ? USB_EP_TX_VALID : USB_EP_TX_DIS);
}

/* ---- buffer table ------------------------------------------------------ */
static void btable_init(void)
{
    USB->BTABLE = 0;
    BT_ADDR_TX(0) = PMA_EP0_TX;   BT_COUNT_TX(0) = 0;
    BT_ADDR_RX(0) = PMA_EP0_RX;   BT_COUNT_RX(0) = RXBLK(EP0_SIZE);

    /* iso OUT: both RX buffers (descriptor slots 0 and 1) alias one buffer */
    BT_ADDR_TX(EP_AUDIO_OUT) = PMA_AUDIO_OUT;  BT_COUNT_TX(EP_AUDIO_OUT) = RXBLK(AUDIO_PKT);
    BT_ADDR_RX(EP_AUDIO_OUT) = PMA_AUDIO_OUT;  BT_COUNT_RX(EP_AUDIO_OUT) = RXBLK(AUDIO_PKT);
    /* iso IN: same trick */
    BT_ADDR_TX(EP_AUDIO_IN) = PMA_AUDIO_IN;    BT_COUNT_TX(EP_AUDIO_IN) = 0;
    BT_ADDR_RX(EP_AUDIO_IN) = PMA_AUDIO_IN;    BT_COUNT_RX(EP_AUDIO_IN) = 0;

    BT_ADDR_TX(EP_CFG) = PMA_CFG_TX;     BT_COUNT_TX(EP_CFG) = 0;
    BT_ADDR_RX(EP_CFG) = PMA_CFG_RX;     BT_COUNT_RX(EP_CFG) = RXBLK(CDC_PKT);
    BT_ADDR_TX(EP_CFG_NOTIF) = PMA_CFG_NOTIF;  BT_COUNT_TX(EP_CFG_NOTIF) = 0;
    BT_ADDR_RX(EP_CFG_NOTIF) = PMA_CFG_NOTIF;  BT_COUNT_RX(EP_CFG_NOTIF) = 0;
    BT_ADDR_TX(EP_RADIO) = PMA_RADIO_TX; BT_COUNT_TX(EP_RADIO) = 0;
    BT_ADDR_RX(EP_RADIO) = PMA_RADIO_RX; BT_COUNT_RX(EP_RADIO) = RXBLK(CDC_PKT);
    BT_ADDR_TX(EP_RADIO_NOTIF) = PMA_RADIO_NOTIF;  BT_COUNT_TX(EP_RADIO_NOTIF) = 0;
    BT_ADDR_RX(EP_RADIO_NOTIF) = PMA_RADIO_NOTIF;  BT_COUNT_RX(EP_RADIO_NOTIF) = 0;
}

/* ---- control endpoint -------------------------------------------------- */
enum { EP0_IDLE, EP0_IN_DATA, EP0_OUT_DATA, EP0_STATUS_IN };
static uint8_t ep0_state;
static usb_setup_t setup;
static uint8_t ep0_buf[128];
static const uint8_t *ep0_ptr;
static uint16_t ep0_rem, ep0_out_got;
static bool ep0_zlp;
static uint8_t pending_addr;
static uint8_t alt_setting[IF_COUNT];

static void ep0_stall(void)
{
    uint16_t v = EPR(0);
    EPR(0) = (v & USB_EPREG_MASK) | USB_EP_CTR_RX | USB_EP_CTR_TX |
             ((v ^ USB_EP_RX_STALL) & USB_EPRX_STAT) | ((v ^ USB_EP_TX_STALL) & USB_EPTX_STAT);
    ep0_state = EP0_IDLE;
}

static void ep0_send_next(void)
{
    uint16_t n = MIN(ep0_rem, EP0_SIZE);
    if (n == 0)
        ep0_zlp = false;
    usb_ep_tx(0, ep0_ptr, n);
    ep0_ptr += n;
    ep0_rem -= n;
}

static void ep0_start_in(const uint8_t *data, uint16_t len)
{
    if (len > setup.wLength)
        len = setup.wLength;
    ep0_ptr = data;
    ep0_rem = len;
    ep0_zlp = len < setup.wLength && (len % EP0_SIZE) == 0;
    ep0_state = EP0_IN_DATA;
    ep0_send_next();
    ep_stat_rx(0, USB_EP_RX_VALID);  /* for the status OUT */
}

static void ep0_status_in(void)
{
    ep0_state = EP0_STATUS_IN;
    usb_ep_tx(0, NULL, 0);
}

static void configure(uint8_t cfgval)
{
    usb_configured = false;
    for (int i = 0; i < IF_COUNT; i++)
        alt_setting[i] = 0;
    audio_reset();
    cdc_reset();
    for (uint8_t n = 1; n < EP_COUNT; n++)
        ep_config(n, 0, USB_EP_RX_DIS, USB_EP_TX_DIS);
    if (cfgval == 1) {
        ep_config(EP_AUDIO_OUT, USB_EP_ISOCHRONOUS, USB_EP_RX_DIS, USB_EP_TX_DIS);
        ep_config(EP_AUDIO_IN, USB_EP_ISOCHRONOUS, USB_EP_RX_DIS, USB_EP_TX_DIS);
        ep_config(EP_CFG, USB_EP_BULK, USB_EP_RX_VALID, USB_EP_TX_NAK);
        ep_config(EP_CFG_NOTIF, USB_EP_INTERRUPT, USB_EP_RX_DIS, USB_EP_TX_NAK);
        ep_config(EP_RADIO, USB_EP_BULK, USB_EP_RX_VALID, USB_EP_TX_NAK);
        ep_config(EP_RADIO_NOTIF, USB_EP_INTERRUPT, USB_EP_RX_DIS, USB_EP_TX_NAK);
        usb_configured = true;
    }
    ptt_update();
}

/* DFU runtime class requests */
static int dfu_setup(const usb_setup_t *s, uint8_t *buf, const uint8_t **in)
{
    switch (s->bRequest) {
    case 0x00:                       /* DFU_DETACH */
        dfu_request();
        return 0;
    case 0x03:                       /* DFU_GETSTATUS: OK, appIDLE */
        buf[0] = 0; buf[1] = 10; buf[2] = 0; buf[3] = 0; buf[4] = 0; buf[5] = 0;
        return 6;
    case 0x05:                       /* DFU_GETSTATE: appIDLE */
        buf[0] = 0;
        return 1;
    }
    return -1;
}

/* Returns the IN length (>= 0) or -1 to stall. For OUT requests with a data
 * stage it is called after the data arrived (in buf). */
static int handle_setup(const usb_setup_t *s, uint8_t *buf, const uint8_t **in)
{
    uint8_t type = (s->bmRequestType >> 5) & 3;
    uint8_t rcpt = s->bmRequestType & 0x1F;
    *in = buf;

    if (type == 1) {                 /* class */
        if (rcpt == 1) {
            uint8_t itf = s->wIndex & 0xFF;
            if (itf == IF_CFG_COMM || itf == IF_RADIO_COMM)
                return cdc_setup(s, buf, in);
            if (itf == IF_AC)
                return audio_setup(s, buf, in);
            if (itf == IF_DFU)
                return dfu_setup(s, buf, in);
        } else if (rcpt == 2) {
            return audio_setup(s, buf, in);
        }
        return -1;
    }
    if (type != 0)
        return -1;

    switch (s->bRequest) {
    case 0x00:                       /* GET_STATUS */
        buf[0] = 0; buf[1] = 0;
        return 2;
    case 0x01:                       /* CLEAR_FEATURE */
    case 0x03:                       /* SET_FEATURE */
        return 0;
    case 0x05:                       /* SET_ADDRESS (applied after status) */
        pending_addr = s->wValue & 0x7F;
        return 0;
    case 0x06:                       /* GET_DESCRIPTOR */
        switch (s->wValue >> 8) {
        case 1: *in = usb_dev_desc; return sizeof usb_dev_desc;
        case 2: *in = usb_cfg_desc; return usb_cfg_desc_len;
        case 3: return usb_string_desc(s->wValue & 0xFF, buf, sizeof ep0_buf);
        }
        return -1;
    case 0x08:                       /* GET_CONFIGURATION */
        buf[0] = usb_configured;
        return 1;
    case 0x09:                       /* SET_CONFIGURATION */
        if (s->wValue > 1)
            return -1;
        configure(s->wValue);
        return 0;
    case 0x0A:                       /* GET_INTERFACE */
        if ((s->wIndex & 0xFF) >= IF_COUNT)
            return -1;
        buf[0] = alt_setting[s->wIndex & 0xFF];
        return 1;
    case 0x0B: {                     /* SET_INTERFACE */
        uint8_t itf = s->wIndex & 0xFF, alt = s->wValue & 0xFF;
        if (itf == IF_AS_OUT || itf == IF_AS_IN) {
            if (alt > 1)
                return -1;
            alt_setting[itf] = alt;
            audio_set_alt(itf, alt);
            return 0;
        }
        return (itf < IF_COUNT && alt == 0) ? 0 : -1;
    }
    }
    return -1;
}

static void ep0_setup(void)
{
    pma_read(PMA_EP0_RX, (uint8_t *)&setup, 8);
    const uint8_t *in;
    if (setup.wLength && !(setup.bmRequestType & 0x80)) {
        /* host-to-device with data: receive it first */
        if (setup.wLength > sizeof ep0_buf) {
            ep0_stall();
            return;
        }
        ep0_out_got = 0;
        ep0_state = EP0_OUT_DATA;
        ep_stat_rx(0, USB_EP_RX_VALID);
        return;
    }
    int r = handle_setup(&setup, ep0_buf, &in);
    if (r < 0)
        ep0_stall();
    else if (setup.bmRequestType & 0x80)
        ep0_start_in(in, (uint16_t)r);
    else
        ep0_status_in();
}

static void ep0_out(void)
{
    if (ep0_state == EP0_OUT_DATA) {
        uint16_t n = BT_COUNT_RX(0) & 0x3FF;
        n = MIN(n, (uint16_t)(sizeof ep0_buf - ep0_out_got));
        pma_read(PMA_EP0_RX, ep0_buf + ep0_out_got, n);
        ep0_out_got += n;
        if (ep0_out_got >= setup.wLength) {
            const uint8_t *in;
            if (handle_setup(&setup, ep0_buf, &in) < 0)
                ep0_stall();
            else
                ep0_status_in();
            return;
        }
    } else {
        ep0_state = EP0_IDLE;        /* status OUT of an IN transfer */
    }
    ep_stat_rx(0, USB_EP_RX_VALID);
}

static void ep0_in(void)
{
    if (ep0_state == EP0_IN_DATA) {
        if (ep0_rem || ep0_zlp)
            ep0_send_next();
    } else if (ep0_state == EP0_STATUS_IN) {
        if (pending_addr) {
            USB->DADDR = USB_DADDR_EF | pending_addr;
            pending_addr = 0;
        }
        ep0_state = EP0_IDLE;
    }
}

/* ---- bus events -------------------------------------------------------- */
static void bus_reset(void)
{
    btable_init();
    ep_config(0, USB_EP_CONTROL, USB_EP_RX_VALID, USB_EP_TX_NAK);
    configure(0);
    ep0_state = EP0_IDLE;
    pending_addr = 0;
    USB->DADDR = USB_DADDR_EF;
    usb_suspended = false;
}

static void usb_isr(void)
{
    uint16_t istr = USB->ISTR;

    if (istr & USB_ISTR_SOF) {
        uint32_t t = TIM2->CNT;      /* timestamp first: clock recovery */
        USB->ISTR = (uint16_t)~USB_ISTR_SOF;
        audio_sof(t, USB->FNR & USB_FNR_FN);
        ptt_sof();
        menu_sof();
    }
    if (istr & USB_ISTR_RESET) {
        USB->ISTR = (uint16_t)~USB_ISTR_RESET;
        bus_reset();
    }
    while ((istr = USB->ISTR) & USB_ISTR_CTR) {
        uint8_t ep = istr & USB_ISTR_EP_ID;
        uint16_t epr = EPR(ep);
        if (epr & USB_EP_CTR_RX) {
            ep_clear_ctr_rx(ep);
            switch (ep) {
            case 0: if (epr & USB_EP_SETUP) ep0_setup(); else ep0_out(); break;
            case EP_AUDIO_OUT: audio_out_packet(); break;
            case EP_CFG:
            case EP_RADIO: cdc_rx(ep); break;
            }
        }
        if (epr & USB_EP_CTR_TX) {
            ep_clear_ctr_tx(ep);
            switch (ep) {
            case 0: ep0_in(); break;
            case EP_AUDIO_IN: audio_in_done(); break;
            case EP_CFG:
            case EP_RADIO: cdc_tx_done(ep); break;
            }
        }
    }
    if (istr & USB_ISTR_SUSP) {
        USB->CNTR |= USB_CNTR_FSUSP;
        USB->ISTR = (uint16_t)~USB_ISTR_SUSP;
        usb_suspended = true;
        audio_suspend();
        ptt_update();
    }
    if (istr & USB_ISTR_WKUP) {
        USB->CNTR &= ~USB_CNTR_FSUSP;
        USB->ISTR = (uint16_t)~USB_ISTR_WKUP;
        usb_suspended = false;
        ptt_update();
    }
    if (istr & (USB_ISTR_ERR | USB_ISTR_ESOF | USB_ISTR_PMAOVR))
        USB->ISTR = (uint16_t)~(USB_ISTR_ERR | USB_ISTR_ESOF | USB_ISTR_PMAOVR);

    cdc_kick();     /* deferred work: CDC IN data, re-enabling paused OUTs */
}

void USB_LP_CAN_RX0_IRQHandler(void) { usb_isr(); }
void USB_HP_CAN_TX_IRQHandler(void)  { usb_isr(); }

void usb_init(void)
{
    /* The D+ pull-up is fixed: force a disconnect so the host re-enumerates
     * after a reset or a firmware update. */
    usb_desc_init();
    usb_reconnect_pulse();

    /* PA11/PA12 -> AF14 (USB) */
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xFFu << 12)) | (0xEEu << 12);
    GPIOA->OSPEEDR |= (3u << 22) | (3u << 24);
    GPIOA->MODER = (GPIOA->MODER & ~(0xFu << 22)) | (0xAu << 22);

    RCC->APB1ENR |= RCC_APB1ENR_USBEN;
    USB->CNTR = USB_CNTR_FRES;       /* power up, keep in reset */
    delay_us(2);                     /* tSTARTUP */
    USB->CNTR = 0;
    USB->ISTR = 0;
    btable_init();
    USB->CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SUSPM |
                USB_CNTR_WKUPM | USB_CNTR_SOFM;

    NVIC_SetPriority(USB_LP_CAN_RX0_IRQn, PRIO_IO);
    NVIC_SetPriority(USB_HP_CAN_TX_IRQn, PRIO_IO);
    NVIC_EnableIRQ(USB_LP_CAN_RX0_IRQn);
    NVIC_EnableIRQ(USB_HP_CAN_TX_IRQn);
}
