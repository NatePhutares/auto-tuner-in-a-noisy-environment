/*
 * oled_gfx.h — 128 x 64 monochrome framebuffer + 5x7 text. No hardware: oled.c sends the buffer.
 *
 * Memory layout = the controller's own: 8 pages of 8 pixel rows; byte [page*128 + x], bit (y % 8).
 * Characters are 6 x 8 pixels (5x7 glyph + 1 column gap) at scale 1, 12 x 16 at scale 2, ...
 */
#ifndef OLED_GFX_H
#define OLED_GFX_H

#include <stdint.h>

#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_PAGES   (OLED_HEIGHT / 8)
#define OLED_FB_SIZE (OLED_WIDTH * OLED_PAGES)     /* 1024 bytes */

typedef struct {
    uint8_t buf[OLED_FB_SIZE];
} oled_fb_t;

void oled_fb_clear(oled_fb_t *fb);
void oled_fb_pixel(oled_fb_t *fb, int x, int y, int on);

/* Draws text with its top-left corner at (x, y). Printable ASCII 32..126; others show as '?'.
 * scale 1 = 6x8 px per char, 2 = 12x16, 3 = 18x24. Returns the x after the last character. */
int  oled_fb_text(oled_fb_t *fb, int x, int y, const char *text, int scale);

/* Width in pixels of text at that scale (for right-aligning). */
int  oled_fb_text_width(const char *text, int scale);

#endif /* OLED_GFX_H */
