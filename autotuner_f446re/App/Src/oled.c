/*
 * oled.c — see oled.h.
 *
 * Each page is one I2C write using the controller's "continuation" control bytes, so the page and
 * column commands and the 128 data bytes go in a single DMA transfer:
 *     0x80 B0|page   0x80 col_low   0x80 col_high   0x40 data[0..127]
 * (0x80 = one command byte follows, 0x40 = data until the stop condition). Page addressing works the
 * same on SSD1306 and SH1106; the SH1106 just has its 128 visible columns starting at column 2.
 */
#include "oled.h"
#include <string.h>

#if OLED_CONTROLLER == OLED_SH1106
#define COLUMN_OFFSET 2
#else
#define COLUMN_OFFSET 0
#endif

#define PAGE_PACKET (6 + 1 + OLED_WIDTH)          /* 3 commands (2 bytes each) + 0x40 + 128 data = 135 */

volatile uint8_t  oled_present = 0;
volatile uint8_t  oled_address = 0;
volatile uint32_t oled_frames = 0;
volatile uint32_t oled_errors = 0;

static I2C_HandleTypeDef *s_hi2c = NULL;
static uint8_t  s_tx[OLED_PAGES][PAGE_PACKET];    /* DMA reads from here: never on the stack */
static int      s_next_page = OLED_PAGES;          /* OLED_PAGES = idle                       */
static int      s_in_flight = 0;                   /* a page transfer was started             */

static const uint8_t INIT_SEQUENCE[] = {
    0x00,                     /* control byte: commands follow                            */
    0xAE,                     /* display off                                             */
    0xD5, 0x80,               /* clock divide / oscillator                              */
    0xA8, 0x3F,               /* multiplex ratio 64                                     */
    0xD3, 0x00,               /* display offset 0                                       */
    0x40,                     /* start line 0                                           */
#if OLED_CONTROLLER == OLED_SH1106
    0xAD, 0x8B,               /* SH1106: DC-DC converter on                             */
#else
    0x8D, 0x14,               /* SSD1306: charge pump on (page addressing is the default) */
#endif
    0xA1,                     /* segment remap: column 127 -> SEG0 (not mirrored)       */
    0xC8,                     /* COM scan from bottom (not upside down)                 */
    0xDA, 0x12,               /* COM pins: alternative configuration (64 rows)          */
    0x81, 0x80,               /* contrast                                               */
#if OLED_CONTROLLER == OLED_SH1106
    0xD9, 0x22,               /* pre-charge                                            */
    0xDB, 0x35,               /* VCOM deselect level                                   */
#else
    0xD9, 0xF1,
    0xDB, 0x40,
#endif
    0xA4,                     /* show RAM content                                       */
    0xA6,                     /* normal (not inverted)                                  */
    0xAF,                     /* display on                                             */
};

static void build_page_header(int page)
{
    uint8_t *p = s_tx[page];
    p[0] = 0x80; p[1] = (uint8_t)(0xB0 | page);
    p[2] = 0x80; p[3] = (uint8_t)(0x00 | (COLUMN_OFFSET & 0x0F));
    p[4] = 0x80; p[5] = (uint8_t)(0x10 | (COLUMN_OFFSET >> 4));
    p[6] = 0x40;
}

int oled_init(I2C_HandleTypeDef *hi2c)
{
    s_hi2c = hi2c;
    oled_present = 0;
    static const uint8_t candidates[2] = {0x3C, 0x3D};
    for (int i = 0; i < 2 && !oled_present; i++) {
        if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(candidates[i] << 1), 2, 5) == HAL_OK) {
            oled_address = candidates[i];
            oled_present = 1;
        }
    }
    if (!oled_present) return -1;

    if (HAL_I2C_Master_Transmit(hi2c, (uint16_t)(oled_address << 1), (uint8_t *)INIT_SEQUENCE,
                                sizeof(INIT_SEQUENCE), 50) != HAL_OK) {
        oled_present = 0;
        oled_errors++;
        return -1;
    }
    /* clear the whole RAM (132 columns on SH1106) so no noise shows at the edges */
    for (int page = 0; page < OLED_PAGES; page++) {
        uint8_t clear[7 + 132];
        memset(clear, 0, sizeof(clear));
        clear[0] = 0x80; clear[1] = (uint8_t)(0xB0 | page);
        clear[2] = 0x80; clear[3] = 0x00;
        clear[4] = 0x80; clear[5] = 0x10;
        clear[6] = 0x40;
        (void)HAL_I2C_Master_Transmit(hi2c, (uint16_t)(oled_address << 1), clear, 7 + 132, 50);
    }
    for (int page = 0; page < OLED_PAGES; page++) build_page_header(page);
    s_next_page = OLED_PAGES;
    s_in_flight = 0;
    return 0;
}

int oled_busy(void)
{
    return s_next_page < OLED_PAGES || s_in_flight;
}

int oled_flush(const oled_fb_t *fb)
{
    if (!oled_present || oled_busy()) return -1;
    for (int page = 0; page < OLED_PAGES; page++)
        memcpy(&s_tx[page][7], &fb->buf[page * OLED_WIDTH], OLED_WIDTH);
    s_next_page = 0;
    oled_poll();                                    /* start page 0 right away */
    return 0;
}

void oled_poll(void)
{
    if (!oled_present || s_hi2c == NULL) return;
    if (HAL_I2C_GetState(s_hi2c) != HAL_I2C_STATE_READY) return;   /* previous page still going */
    if (s_in_flight) {                                              /* previous page finished    */
        s_in_flight = 0;
        if (s_next_page >= OLED_PAGES) oled_frames++;
    }
    if (s_next_page >= OLED_PAGES) return;                          /* idle */
    if (HAL_I2C_Master_Transmit_DMA(s_hi2c, (uint16_t)(oled_address << 1), s_tx[s_next_page],
                                    PAGE_PACKET) == HAL_OK) {
        s_in_flight = 1;
        s_next_page++;
    } else {
        oled_errors++;                              /* bus busy or error: drop this frame */
        s_next_page = OLED_PAGES;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != s_hi2c) return;
    oled_errors++;
    s_next_page = OLED_PAGES;                       /* give up on this frame; the next one retries */
    s_in_flight = 0;
}
