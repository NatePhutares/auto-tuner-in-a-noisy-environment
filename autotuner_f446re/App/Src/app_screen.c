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

/* MIDI note -> "C#4", or "-" for none. buf needs 8 bytes. */
static const char *note_or_dash(int midi, char *buf)
{
    if (midi >= 0) return at_note_name(midi, buf);
    buf[0] = '-';
    buf[1] = '\0';
    return buf;
}

void app_screen_draw(oled_fb_t *fb, int mode, int target_midi, int sung_midi)
{
    char target[8], sung[8];
    oled_fb_clear(fb);

    row(fb, 0, "mode:", mode == AT_MODE_FIXED ? "FIXED" : "NEAREST");
    row(fb, 1, "target:", note_or_dash(target_midi, target));
    row(fb, 2, "sung:", note_or_dash(sung_midi, sung));
}
