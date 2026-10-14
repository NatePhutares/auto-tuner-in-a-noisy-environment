/*
 * app_screen.c — see app_screen.h. Change the layout here; nothing else depends on it.
 */
#include "app_screen.h"
#include "at_config.h"
#include "at_notes.h"

#define VALUE_SCALE 2
#define ROW_Y0      2      /* top of the first row                     */
#define ROW_STEP    21     /* rows at y = 2, 23, 44 (16 px tall values) */
#define LABEL_DY    4      /* small label centred on the large value   */

static void row(oled_fb_t *fb, int index, const char *label, const char *value)
{
    int y = ROW_Y0 + index * ROW_STEP;
    oled_fb_text(fb, 0, y + LABEL_DY, label, 1);
    oled_fb_text(fb, OLED_WIDTH - oled_fb_text_width(value, VALUE_SCALE), y, value, VALUE_SCALE);
}

void app_screen_draw(oled_fb_t *fb, int mode, int target_midi, int oor_percent)
{
    char target[8], percent[6];
    oled_fb_clear(fb);

    row(fb, 0, "mode:", mode == AT_MODE_FIXED ? "FIXED" : "NEAREST");

    if (target_midi >= 0) at_note_name(target_midi, target);
    else { target[0] = '-'; target[1] = '\0'; }
    row(fb, 1, "target:", target);

    if (oor_percent >= 0) {
        int p = oor_percent > 100 ? 100 : oor_percent, i = 0;
        if (p >= 100) percent[i++] = '1';
        if (p >= 10)  percent[i++] = (char)('0' + (p / 10) % 10);
        percent[i++] = (char)('0' + p % 10);
        percent[i++] = '%';
        percent[i] = '\0';
    } else {
        percent[0] = '-'; percent[1] = '\0';
    }
    row(fb, 2, "oor_percent:", percent);
}
