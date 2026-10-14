/*
 * app_screen.h — what the OLED shows (layout only; no I2C, no DSP).
 *
 *   mode:            NEAREST        labels small (6x8 px), values large (12x16 px), right-aligned
 *   target:               C5        '-' in NEAREST mode
 *   sung:                 A4        note being sung (nearest note to the smoothed pitch), '-' when silent
 *
 * The sung note is chosen by app_main.c (it holds the last note briefly so short consonants don't blank it).
 */
#ifndef APP_SCREEN_H
#define APP_SCREEN_H

#include "oled_gfx.h"

/* target_midi < 0 or sung_midi < 0 show '-'. mode: AT_MODE_NEAREST / AT_MODE_FIXED. */
void app_screen_draw(oled_fb_t *fb, int mode, int target_midi, int sung_midi);

#endif /* APP_SCREEN_H */
