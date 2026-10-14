/*
 * app_screen.h — what the OLED shows (layout only; no I2C, no DSP).
 *
 *   mode:            NEAREST        labels small (6x8 px), values large (12x16 px), right-aligned
 *   target:               C5        '-' in NEAREST mode
 *   oor_percent:         12%        '-' in NEAREST mode
 *
 * oor_percent = share of voiced hops that were out of range since the last mode / target change
 * (counted by app_main.c).
 */
#ifndef APP_SCREEN_H
#define APP_SCREEN_H

#include "oled_gfx.h"

/* target_midi < 0 or oor_percent < 0 show '-'. mode: AT_MODE_NEAREST / AT_MODE_FIXED. */
void app_screen_draw(oled_fb_t *fb, int mode, int target_midi, int oor_percent);

#endif /* APP_SCREEN_H */
