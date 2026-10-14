/*
 * at_notes.c — see at_notes.h.
 */
#include "at_notes.h"
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

static const char *const NOTE_NAMES[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};

float at_hz_to_midi(float hz)   { return 69.0f + 12.0f * log2f(hz / 440.0f); }
float at_midi_to_hz(float midi) { return 440.0f * exp2f((midi - 69.0f) / 12.0f); }

static int floor_div(int a, int b) { int q = a / b; if ((a % b != 0) && ((a < 0) != (b < 0))) q--; return q; }
static int pos_mod(int a, int b)   { int r = a % b; return r < 0 ? r + b : r; }

char *at_note_name(int midi, char *buf)
{
    const char *n = NOTE_NAMES[pos_mod(midi, 12)];
    int octave = floor_div(midi, 12) - 1;
    int i = 0;
    while (*n) buf[i++] = *n++;
    if (octave < 0) { buf[i++] = '-'; octave = -octave; }
    if (octave >= 10) buf[i++] = (char)('0' + octave / 10);
    buf[i++] = (char)('0' + octave % 10);
    buf[i] = '\0';
    return buf;
}

int at_parse_note(const char *text)
{
    while (*text == ' ') text++;
    /* plain MIDI number */
    const char *q = text;
    if (*q == '-') q++;
    if (isdigit((unsigned char)*q)) {
        char *end;
        long v = strtol(text, &end, 10);
        while (*end == ' ' || *end == '\r' || *end == '\n') end++;
        return *end == '\0' ? (int)v : -1000;
    }
    static const int letter_pc[7] = {9, 11, 0, 2, 4, 5, 7};   /* A B C D E F G */
    char L = (char)toupper((unsigned char)text[0]);
    if (L < 'A' || L > 'G') return -1000;
    int pc = letter_pc[L - 'A'];
    int i = 1;
    if (text[1] == '#') { pc += 1; i = 2; }
    else if (text[1] == 'b') { pc -= 1; i = 2; }
    /* Python only accepts the sharp names and Db Eb Gb Ab Bb */
    if (i == 2 && text[1] == '#' && (L == 'E' || L == 'B')) return -1000;
    if (i == 2 && text[1] == 'b' && (L == 'C' || L == 'F')) return -1000;
    q = text + i;
    if (*q == '-') q++;
    if (!isdigit((unsigned char)*q)) return -1000;
    char *end;
    long octave = strtol(text + i, &end, 10);
    while (*end == ' ' || *end == '\r' || *end == '\n') end++;
    if (*end != '\0') return -1000;
    return pos_mod(pc, 12) + 12 * ((int)octave + 1);
}

/* ---------------------------------------------------------------- note tracker */
void at_note_tracker_init(at_note_tracker_t *t, const at_config_t *cfg)
{
    memset(t, 0, sizeof(*t));
    t->median_len = cfg->median_len > AT_MEDIAN_MAX ? AT_MEDIAN_MAX : cfg->median_len;
    t->hyst_frames = cfg->hyst_frames;
    t->hyst_semitones2 = 2.0f * cfg->hyst_cents / 100.0f;
    t->scale_mask = cfg->scale_mask;
}

void at_note_tracker_set_scale(at_note_tracker_t *t, uint16_t scale_mask)
{
    t->scale_mask = scale_mask ? scale_mask : AT_SCALE_CHROMATIC;
}

static int nearest_allowed(const at_note_tracker_t *t, float m)
{
    if ((t->scale_mask & AT_SCALE_CHROMATIC) == AT_SCALE_CHROMATIC)
        return (int)floorf(m + 0.5f);
    int base = (int)floorf(m);
    int best = base;
    float best_dist = 1e9f;
    for (int note = base - 3; note < base + 5; note++) {
        if ((t->scale_mask >> pos_mod(note, 12)) & 1u) {
            float dist = fabsf(m - (float)note);
            if (dist < best_dist) { best = note; best_dist = dist; }
        }
    }
    return best;
}

static float median(const float *v, int n)
{
    float s[AT_MEDIAN_MAX];
    memcpy(s, v, (size_t)n * sizeof(float));
    for (int i = 1; i < n; i++) {                 /* insertion sort, n <= 8 */
        float x = s[i];
        int j = i - 1;
        while (j >= 0 && s[j] > x) { s[j + 1] = s[j]; j--; }
        s[j + 1] = x;
    }
    return (n & 1) ? s[n / 2] : 0.5f * (s[n / 2 - 1] + s[n / 2]);
}

void at_note_tracker_update(at_note_tracker_t *t, int voiced, float raw_midi)
{
    if (!voiced) {
        t->unvoiced_count++;
        if (t->unvoiced_count >= 3) t->history_len = 0;   /* don't mix pre- and post-gap pitches */
        t->candidate_count = 0;
        return;                                            /* hold the last valid note */
    }
    t->unvoiced_count = 0;
    if (t->history_len == t->median_len) {
        memmove(t->history, t->history + 1, (size_t)(t->median_len - 1) * sizeof(float));
        t->history_len--;
    }
    t->history[t->history_len++] = raw_midi;
    t->smoothed_midi = median(t->history, t->history_len);
    t->has_smoothed = 1;

    int candidate = nearest_allowed(t, t->smoothed_midi);
    if (!t->has_target) {
        t->target_note = candidate;
        t->has_target = 1;
    } else if (candidate != t->target_note) {
        float margin = fabsf(t->smoothed_midi - (float)t->target_note) - fabsf(t->smoothed_midi - (float)candidate);
        if (margin > t->hyst_semitones2) t->candidate_count++;
        else                             t->candidate_count = 0;
        if (t->candidate_count >= t->hyst_frames) {
            t->target_note = candidate;
            t->candidate_count = 0;
        }
    } else {
        t->candidate_count = 0;
    }
}

/* ---------------------------------------------------------------- out-of-range monitor */
void at_range_monitor_init(at_range_monitor_t *m, const at_config_t *cfg)
{
    m->max_shift = cfg->max_shift;
    m->margin = cfg->oor_margin;
    m->frames = cfg->oor_frames;
    at_range_monitor_reset(m);
}

void at_range_monitor_reset(at_range_monitor_t *m)
{
    m->is_out_of_range = 0;
    m->change_count = 0;
}

int at_range_monitor_update(at_range_monitor_t *m, int has_gap, float gap)
{
    if (!has_gap) return m->is_out_of_range;
    int wanted = m->is_out_of_range ? !(gap < m->max_shift - m->margin)
                                    : (gap > m->max_shift + m->margin);
    if (wanted != m->is_out_of_range) {
        if (++m->change_count >= m->frames) {
            m->is_out_of_range = wanted;
            m->change_count = 0;
        }
    } else {
        m->change_count = 0;
    }
    return m->is_out_of_range;
}
