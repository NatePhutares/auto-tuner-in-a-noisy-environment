/*
 * autotuner.h — the whole auto-tuner pipeline, one call per 128-sample hop.
 * C port of autotuner.py (class AutoTuner). This is the only header main.c needs.
 *
 *   mic -> noise suppressor (at_denoise)
 *            |-> detection: 75 Hz-1 kHz band-pass (at_biquad) -> FFT autocorrelation (at_pitch)
 *            |      -> MIDI, median, hysteresis (at_notes) -> mode -> shift, clamp, out-of-range
 *            |-> audio: 75 Hz high-pass -> LPC (at_lpc) -> residual TD-PSOLA -> 1/A(z) (at_psola)
 *                       -> dry crossfade, level match, pause gate (at_post)
 *
 * Latency: AT_DENOISE_DELAY + AT_PSOLA_DELAY = 384 + 612 = 996 samples (62 ms).
 * Output sample out[i] of the hop that ends at input sample n corresponds to input sample
 * n - 996 - 128 + i.
 *
 * Usage:
 *     static autotuner_t tuner;              // ~60 kB, keep it static (not on the stack)
 *     at_config_t cfg;  at_config_default(&cfg);
 *     autotuner_init(&tuner, &cfg);
 *     every hop: autotuner_process_hop(&tuner, in, out, &info);   // in/out: AT_H floats in [-1, 1)
 */
#ifndef AUTOTUNER_H
#define AUTOTUNER_H

#include <stdint.h>
#include "at_config.h"
#include "at_biquad.h"
#include "at_denoise.h"
#include "at_pitch.h"
#include "at_notes.h"
#include "at_lpc.h"
#include "at_psola.h"

/* Per-hop results (same fields as the Python info dict / golden-vector CSV). */
typedef struct {
    int32_t n_end;          /* samples seen so far (end of this hop)                  */
    int     voiced;
    float   f0_hz;          /* detected pitch, 0 if unvoiced                           */
    float   confidence;     /* r(tau)/r(0) at the peak                                  */
    float   tau;            /* lag in samples, 0 if none                               */
    float   rms_db;         /* frame level, dBFS                                        */
    int     has_midi;       /* 0 until the first voiced hop                            */
    float   smoothed_midi;  /* median-smoothed sung pitch (MIDI)                       */
    int     target;         /* target MIDI note, -1 = none                             */
    int     mode;           /* AT_MODE_NEAREST / AT_MODE_FIXED                         */
    float   shift_raw;      /* target - sung, semitones                                */
    float   shift;          /* clamped to +-max_shift                                  */
    float   shift_smoothed; /* after the retune filter                                 */
    int     out_of_range;
    uint8_t status;         /* AT_ST_* bits, ready for the UART packet                 */
    float   pitch_ratio;    /* alpha = 2^(shift_smoothed / 12)                         */
    float   period;         /* period used by the shifter, samples                     */
} at_info_t;

typedef struct {
    at_config_t cfg;

    at_denoise_t       denoise;
    at_biquad_t        bandpass, highpass;
    at_pitch_t         pitch;
    at_note_tracker_t  notes;
    at_range_monitor_t range;
    at_lpc_t           lpc;
    at_psola_t         psola;

    float   frame[AT_N];                    /* detection-path frame               */
    float   audio_frame[AT_N];              /* audio-path frame (for LPC)          */
    float   hop_energy[AT_HOPS_PER_FRAME];  /* sum of squares of the last 4 hops  */
    int32_t samples_seen;

    float   retune_gain;
    float   smoothed_shift;
    float   last_period;

    int     gate_has_floor;
    float   gate_floor_power;
    float   gate_smoothed_power;
    float   gate_rise;                      /* 3 dB/s (old gate, SNR gate off)    */
    float   gate_rise_unvoiced;             /* SNR gate: floor rise on unvoiced hops */
    float   gate_rise_voiced;               /* SNR gate: floor rise on voiced hops   */
    float   gate_open_ratio;                /* 10^(gate_open_db / 10)             */
    float   gate_snr_ratio;                 /* 10^(gate_snr_db / 10)              */

    int     mode;                           /* run-time controls, read once per hop */
    int     fixed_target_midi;

    /* scratch for one hop */
    float   clean[AT_H], detect[AT_H], audio[AT_H], refl[AT_LPC_ORDER];
} autotuner_t;

void autotuner_init(autotuner_t *t, const at_config_t *cfg);

/* Run-time controls. They take effect at the next hop. Return 0 if accepted, -1 if rejected. */
int  autotuner_set_mode(autotuner_t *t, int mode);
int  autotuner_set_target(autotuner_t *t, int midi);          /* AT_TARGET_MIN..AT_TARGET_MAX */
void autotuner_set_scale(autotuner_t *t, uint16_t scale_mask); /* nearest mode; AT_SCALE_* rotated to the key */

/* in: AT_H input samples, out: AT_H output samples, info: may be NULL. */
void autotuner_process_hop(autotuner_t *t, const float *in, float *out, at_info_t *info);

#endif /* AUTOTUNER_H */
