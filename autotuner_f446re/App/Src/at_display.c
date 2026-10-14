/*
 * at_display.c — see at_display.h. No printf: works without float support in newlib-nano.
 */
#include "at_display.h"
#include <math.h>
#include <string.h>

/* "+3", "-12", "+0" ... appended to buf */
static void append_signed(char *buf, long v, int negative_zero)
{
    char tmp[12];
    int n = 0;
    unsigned long u = (unsigned long)(v < 0 ? -v : v);
    do { tmp[n++] = (char)('0' + u % 10); u /= 10; } while (u);
    size_t len = strlen(buf);
    buf[len++] = (v < 0 || negative_zero) ? '-' : '+';
    while (n) buf[len++] = tmp[--n];
    buf[len] = '\0';
}

static void copy_trunc(char *out, const char *in, int width)
{
    int i = 0;
    for (; i < width && in[i]; i++) out[i] = in[i];
    out[i] = '\0';
}

/* left + right right-aligned in width; returns 0 if they don't fit with one space between */
static int fit(char *out, int width, const char *left, const char *right)
{
    int l = (int)strlen(left), r = (int)strlen(right);
    if (l + 1 + r > width) return 0;
    memcpy(out, left, (size_t)l);
    for (int i = l; i < width - r; i++) out[i] = ' ';
    memcpy(out + width - r, right, (size_t)r);
    out[width] = '\0';
    return 1;
}

void at_display_lines(const at_info_t *info, int width, char *line1, char *line2)
{
    if (width > AT_DISPLAY_MAX_WIDTH) width = AT_DISPLAY_MAX_WIDTH;
    int fixed = info->mode == AT_MODE_FIXED;
    int has_target = info->voiced && info->target >= 0 && info->has_midi;
    char name[8], left[AT_DISPLAY_MAX_WIDTH + 8], right[24];
    const char *tp = width > 16 ? "Target: " : "T:";
    const char *sp = width > 16 ? "Sung: " : "S:";

    /* line 1: target note + mode */
    strcpy(left, tp);
    strcat(left, info->target >= 0 ? at_note_name(info->target, name) : "--");
    if (!fit(line1, width, left, fixed ? "[FIXED]" : "[NEAREST]") &&
        !fit(line1, width, left, fixed ? "[F]" : "[N]")) {
        copy_trunc(line1, left, width);
    }

    /* line 2: sung note + status */
    int sung = (int)floorf(info->smoothed_midi + 0.5f);
    strcpy(left, sp);
    strcat(left, (info->voiced && info->has_midi) ? at_note_name(sung, name) : "--");
    const char *hints[3] = {"", 0, 0};
    if (fixed && info->out_of_range) {
        hints[0] = "OUT OF RANGE"; hints[1] = "OUT RANGE"; hints[2] = "OOR";
    } else if (has_target && fixed) {
        right[0] = '\0';
        append_signed(right, (long)(info->target - sung), 0);
        strcat(right, " st");
        hints[0] = right;
    } else if (has_target) {
        float cents = 100.0f * (info->smoothed_midi - (float)info->target);
        long c = lrintf(cents);
        right[0] = '\0';
        append_signed(right, c, c == 0 && cents < 0.0f);
        strcat(right, " c");
        hints[0] = right;
    }
    for (int i = 0; i < 3 && hints[i]; i++)
        if (fit(line2, width, left, hints[i])) return;
    copy_trunc(line2, left, width);
}
