/* Descriptors: 2x CDC-ACM, UAC1 speaker + microphone, DFU runtime */
#include "usb.h"
#include "audio.h"

#define LO(x) ((uint8_t)((x) & 0xFF))
#define HI(x) ((uint8_t)(((x) >> 8) & 0xFF))
#define W(x)  LO(x), HI(x)
#define B3(x) LO(x), HI(x), (uint8_t)(((x) >> 16) & 0xFF)

enum { STR_LANG, STR_MANUF, STR_PRODUCT, STR_SERIAL, STR_CFG, STR_RADIO,
       STR_AUDIO, STR_DFU };

const uint8_t usb_dev_desc[18] = {
    18, 1, W(0x0200),
    0xEF, 0x02, 0x01,             /* misc / IAD */
    EP0_SIZE, W(USB_VID), W(USB_PID), W(0x0001),
    STR_MANUF, STR_PRODUCT, STR_SERIAL, 1
};

#define CDC_FUNC(itf_comm, ep_notif, ep_data, str)                              \
    /* IAD */ 8, 0x0B, itf_comm, 2, 0x02, 0x02, 0x00, str,                       \
    /* comm interface (protocol 0: no AT commands, keeps ModemManager away) */  \
    9, 4, itf_comm, 0, 1, 0x02, 0x02, 0x00, str,                                 \
    5, 0x24, 0x00, W(0x0110),               /* header */                         \
    5, 0x24, 0x01, 0x00, (itf_comm) + 1,    /* call management */                \
    4, 0x24, 0x02, 0x02,                    /* ACM: line coding + ctrl lines */  \
    5, 0x24, 0x06, itf_comm, (itf_comm) + 1,/* union */                          \
    7, 5, 0x80 | (ep_notif), 0x03, W(8), 255,                                    \
    /* data interface */                                                         \
    9, 4, (itf_comm) + 1, 0, 2, 0x0A, 0x00, 0x00, 0,                             \
    7, 5, (ep_data), 0x02, W(CDC_PKT), 0,                                        \
    7, 5, 0x80 | (ep_data), 0x02, W(CDC_PKT), 0

/* Rates offered by both audio directions */
#define RATES B3(8000), B3(11025), B3(16000), B3(22050), B3(24000), B3(32000), \
              B3(44100), B3(48000)
#define NRATES 8

#define AC_LEN (10 + 12 + 9 + 9 + 12 + 9 + 9)

#define AS_IFACE(itf, term, ep)                                                 \
    9, 4, itf, 0, 0, 0x01, 0x02, 0x00, 0,       /* alt 0: no bandwidth */       \
    9, 4, itf, 1, 1, 0x01, 0x02, 0x00, 0,       /* alt 1: streaming */          \
    7, 0x24, 0x01, term, 1, W(0x0001),          /* AS general: PCM */           \
    8 + 3 * NRATES, 0x24, 0x02, 0x01, 1, 2, 16, NRATES, RATES,                  \
    9, 5, ep, 0x0D, W(AUDIO_PKT), 1, 0, 0,      /* iso, synchronous */          \
    7, 0x25, 0x01, 0x01, 0, W(0)                /* sampling freq control */

uint8_t usb_cfg_desc[] = {
    9, 2, W(0), IF_COUNT, 1, 0, 0x80, 50,       /* bus powered, 100 mA */

    CDC_FUNC(IF_CFG_COMM, EP_CFG_NOTIF, EP_CFG, STR_CFG),
    CDC_FUNC(IF_RADIO_COMM, EP_RADIO_NOTIF, EP_RADIO, STR_RADIO),

    /* Audio function */
    8, 0x0B, IF_AC, 3, 0x01, 0x00, 0x00, STR_AUDIO,
    9, 4, IF_AC, 0, 0, 0x01, 0x01, 0x00, STR_AUDIO,
    10, 0x24, 0x01, W(0x0100), W(AC_LEN), 2, IF_AS_OUT, IF_AS_IN,
    /* host -> radio: USB streaming IT 1 -> FU 2 -> speaker OT 3 */
    12, 0x24, 0x02, UAC_IT_USB_OUT, W(0x0101), 0, 1, W(0), 0, 0,
    9, 0x24, 0x06, UAC_FU_OUT, UAC_IT_USB_OUT, 1, 0x03, 0x00, 0,
    9, 0x24, 0x03, UAC_OT_SPK, W(0x0301), 0, UAC_FU_OUT, 0,
    /* radio -> host: microphone IT 4 -> FU 5 -> USB streaming OT 6 */
    12, 0x24, 0x02, UAC_IT_MIC, W(0x0201), 0, 1, W(0), 0, 0,
    9, 0x24, 0x06, UAC_FU_IN, UAC_IT_MIC, 1, 0x03, 0x00, 0,
    9, 0x24, 0x03, UAC_OT_USB_IN, W(0x0101), 0, UAC_FU_IN, 0,

    AS_IFACE(IF_AS_OUT, UAC_IT_USB_OUT, EP_AUDIO_OUT),
    AS_IFACE(IF_AS_IN, UAC_OT_USB_IN, 0x80 | EP_AUDIO_IN),

    /* DFU runtime: dfu-util detaches the device into the ROM bootloader */
    9, 4, IF_DFU, 0, 0, 0xFE, 0x01, 0x01, STR_DFU,
    9, 0x21, 0x09, W(255), W(2048), W(0x0110),  /* WillDetach | CanDnload */
};
const uint16_t usb_cfg_desc_len = sizeof usb_cfg_desc;

static char serial[25];

const char *usb_serial(void)
{
    if (!serial[0]) {
        static const char hex[] = "0123456789ABCDEF";
        const uint8_t *uid = (const uint8_t *)UID_BASE;
        for (int i = 0; i < 12; i++) {
            serial[2 * i] = hex[uid[i] >> 4];
            serial[2 * i + 1] = hex[uid[i] & 15];
        }
    }
    return serial;
}

int usb_string_desc(uint8_t idx, uint8_t *buf, uint16_t max)
{
    const char *s;
    switch (idx) {
    case STR_LANG:
        buf[0] = 4; buf[1] = 3; buf[2] = 0x09; buf[3] = 0x04;
        return 4;
    case STR_MANUF:   s = "AIOC-alt"; break;
    case STR_PRODUCT: s = "AIOC-alt"; break;
    case STR_SERIAL:  s = usb_serial(); break;
    case STR_CFG:     s = "AIOC config"; break;
    case STR_RADIO:   s = "AIOC radio"; break;
    case STR_AUDIO:   s = "AIOC audio"; break;
    case STR_DFU:     s = "AIOC DFU"; break;
    default: return -1;
    }
    int n = 2;
    while (*s && n + 2 <= max) {
        buf[n++] = (uint8_t)*s++;
        buf[n++] = 0;
    }
    buf[0] = (uint8_t)n;
    buf[1] = 3;
    return n;
}

/* wTotalLength is filled in at startup (the array can't size itself) */
void usb_desc_init(void)
{
    usb_cfg_desc[2] = LO(sizeof usb_cfg_desc);
    usb_cfg_desc[3] = HI(sizeof usb_cfg_desc);
}
