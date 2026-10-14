/*
 * at_notes.h — everything about notes: Hz <-> MIDI, note names, the note tracker
 * (median + scale snapping + hysteresis) and the fixed-mode out-of-range monitor.
 * Python: hz_to_midi, midi_to_hz, note_name, parse_note, class NoteTracker, class RangeMonitor.
 */
#ifndef AT_NOTES_H
#define AT_NOTES_H

#include <stdint.h>
#include "at_config.h"

#define AT_MEDIAN_MAX 8

float at_hz_to_midi(float hz);
float at_midi_to_hz(float midi);

/* "C4", "F#3" ... into buf (>= 5 bytes). Returns buf. */
char *at_note_name(int midi, char *buf);

/* "C6", "F#3", "Bb2" or a MIDI number "84" -> MIDI note. Returns -1000 if the text is not a note. */
int at_parse_note(const char *text);

/* ---------------- note tracker ---------------- */
typedef struct {
    int      median_len, hyst_frames;
    float    hyst_semitones2;       /* 2 * hyst_cents / 100 */
    uint16_t scale_mask;
    float    history[AT_MEDIAN_MAX];
    int      history_len;
    int      unvoiced_count;
    int      has_target;
    int      target_note;
    int      has_smoothed;
    float    smoothed_midi;
    int      candidate_count;
} at_note_tracker_t;

void at_note_tracker_init(at_note_tracker_t *t, const at_config_t *cfg);
void at_note_tracker_set_scale(at_note_tracker_t *t, uint16_t scale_mask);

/* voiced = 0: raw_midi ignored, last note held. Results are read from t->smoothed_midi / t->target_note
 * (valid when has_smoothed / has_target). */
void at_note_tracker_update(at_note_tracker_t *t, int voiced, float raw_midi);

/* ---------------- out-of-range monitor (fixed mode) ---------------- */
typedef struct {
    float max_shift, margin;
    int   frames;
    int   is_out_of_range;
    int   change_count;
} at_range_monitor_t;

void at_range_monitor_init(at_range_monitor_t *m, const at_config_t *cfg);
void at_range_monitor_reset(at_range_monitor_t *m);
/* has_gap = 0 on unvoiced hops (state kept). Returns the flag. */
int  at_range_monitor_update(at_range_monitor_t *m, int has_gap, float pitch_gap);

#endif /* AT_NOTES_H */
