/*
 * at_post.h — the three output stages that run on every finalised block after the LPC synthesis,
 * in this order (Python: TDPSOLA._emit -> _level -> _gate):
 *
 *   1. dry crossfade : unvoiced stretches (consonants, breath) use the dry, unshifted audio
 *                      (linear 160-sample crossfade)                                  TDPSOLA._emit
 *   2. level match   : slow gain so the output level follows the input level          TDPSOLA._level
 *   3. pause gate    : -25 dB between phrases (hold 150 ms, lookahead 20 ms)          TDPSOLA._gate
 *
 * Each stage owns its state and works in place on a block whose first sample has absolute index
 * `start`. The per-sample flags (use_dry, active) and the dry audio come from the shifter's rings.
 */
#ifndef AT_POST_H
#define AT_POST_H

#include <stdint.h>
#include "at_config.h"

/* ---- 1. dry crossfade ---- */
typedef struct {
    float step;           /* 1 / dry_fade */
    float blend;          /* 0 = shifted, 1 = dry */
} at_xfade_t;

void at_xfade_init(at_xfade_t *x, const at_config_t *cfg);
void at_xfade_run(at_xfade_t *x, float *block, int32_t start, int len,
                  const float *dry_ring, const uint8_t *use_dry_ring);

/* ---- 2. level match ---- */
typedef struct {
    float tau_samples;    /* fs * level_tau_s */
    float gain_limit;     /* 10^(level_max_db / 20) */
    float in_power, out_power, gain;
} at_level_t;

void at_level_init(at_level_t *l, const at_config_t *cfg);
void at_level_run(at_level_t *l, float *block, int32_t start, int len, const float *dry_ring);

/* ---- 3. pause gate ---- */
typedef struct {
    int32_t hold, ahead;  /* samples */
    float   depth;        /* linear gain while closed */
    float   attack, release;
    float   gain;
    int32_t last_active;  /* newest index <= scan position with active = 1 */
    int32_t scan_pos;     /* next index of the active ring to look at     */
} at_gate_t;

void at_gate_init(at_gate_t *g, const at_config_t *cfg);
void at_gate_run(at_gate_t *g, float *block, int32_t start, int len,
                 const uint8_t *active_ring, int32_t samples_received);

#endif /* AT_POST_H */
