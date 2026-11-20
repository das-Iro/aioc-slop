/* Settings in the last 2 KB flash page */
#include "settings.h"

#define SETTINGS_MAGIC   0x434F4941u   /* "AIOC" */
#define SETTINGS_VERSION 2

extern const uint8_t _settings_page[];
settings_t cfg;

void settings_defaults(void)
{
    cfg = (settings_t){
        .magic = SETTINGS_MAGIC, .version = SETTINGS_VERSION, .size = sizeof(settings_t),
        .vol_out = 0, .vol_in = 0, .mute_out = 0, .mute_in = 0,
        .buf_ms = 2, .dc_block = 1,
        /* CHIRP raises DTR and RTS together, so it must not key the radio */
        .ptt_src = { PTT_SRC_DTR_NRTS, PTT_SRC_OFF },
        .ptt_vox = { 0, 0 },
        .tot_s = 0,
        .vox_thr = -30, .vox_hang = 500,
        .led_en = 1,
        .oversample = 1,
    };
}

static uint32_t crc32(const void *data, size_t n)
{
    const uint8_t *p = data;
    uint32_t c = 0xFFFFFFFFu;
    while (n--) {
        c ^= *p++;
        for (int k = 0; k < 8; k++)
            c = (c >> 1) ^ (0xEDB88320u & -(c & 1));
    }
    return ~c;
}

bool settings_load(void)
{
    const settings_t *f = (const settings_t *)_settings_page;
    if (f->magic == SETTINGS_MAGIC && f->version == SETTINGS_VERSION &&
        f->size == sizeof(settings_t) &&
        f->crc == crc32(f, offsetof(settings_t, crc))) {
        cfg = *f;
        return true;
    }
    settings_defaults();
    return false;
}

/* Wait for the end of a flash operation: EOP/error interrupt + WFI. */
static volatile uint32_t flash_sr;
static volatile bool flash_irq;

void FLASH_IRQHandler(void)
{
    flash_sr = FLASH->SR;
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPERR;
    flash_irq = true;
}

static bool flash_start_wait(uint32_t cr)
{
    flash_irq = false;
    FLASH->CR = cr | FLASH_CR_EOPIE | FLASH_CR_ERRIE;
    WAIT_UNTIL(flash_irq);
    return (flash_sr & FLASH_SR_EOP) && !(flash_sr & (FLASH_SR_PGERR | FLASH_SR_WRPERR));
}

bool settings_save(void)
{
    cfg.magic = SETTINGS_MAGIC;
    cfg.version = SETTINGS_VERSION;
    cfg.size = sizeof(settings_t);
    cfg.crc = crc32(&cfg, offsetof(settings_t, crc));

    bool ok = true;
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;

    FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPERR;
    NVIC_SetPriority(FLASH_IRQn, PRIO_DELAY);
    NVIC_EnableIRQ(FLASH_IRQn);

    FLASH->CR = FLASH_CR_PER;
    FLASH->AR = (uint32_t)_settings_page;
    ok = flash_start_wait(FLASH_CR_PER | FLASH_CR_STRT);

    const uint16_t *src = (const uint16_t *)&cfg;
    volatile uint16_t *dst = (volatile uint16_t *)_settings_page;
    for (size_t i = 0; i < sizeof(settings_t) / 2 && ok; i++) {
        flash_irq = false;
        FLASH->CR = FLASH_CR_PG | FLASH_CR_EOPIE | FLASH_CR_ERRIE;
        dst[i] = src[i];                 /* starts the programming */
        WAIT_UNTIL(flash_irq);
        ok = (flash_sr & FLASH_SR_EOP) && !(flash_sr & (FLASH_SR_PGERR | FLASH_SR_WRPERR));
    }
    FLASH->CR = 0;
    NVIC_DisableIRQ(FLASH_IRQn);
    FLASH->CR = FLASH_CR_LOCK;
    return ok && settings_load();
}
