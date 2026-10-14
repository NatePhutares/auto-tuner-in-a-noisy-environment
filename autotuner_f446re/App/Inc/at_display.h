/*
 * at_display.h — the two display lines (spec section 4). Python: display_lines().
 * Text only, no OLED driver: an SSD1306 driver can draw these two strings later.
 *
 *   width 21 (SSD1306, 6 px font):   "Target: C4    [FIXED]"   "Sung: A3        +3 st"
 *   width 16 (LCD1602)           :   "T:C4     [FIXED]"        "S:A3       +3 st"
 */
#ifndef AT_DISPLAY_H
#define AT_DISPLAY_H

#include "autotuner.h"

#define AT_DISPLAY_MAX_WIDTH 32

/* line1 / line2 must hold width + 1 chars. width <= AT_DISPLAY_MAX_WIDTH. */
void at_display_lines(const at_info_t *info, int width, char *line1, char *line2);

#endif /* AT_DISPLAY_H */
