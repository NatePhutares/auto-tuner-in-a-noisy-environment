/*
 * autotuner.c — the pipeline glue (Python: AutoTuner.__init__ / process_hop). The DSP itself lives in
 * the at_*.c modules; this file only wires them together and makes the per-hop decisions:
 * voicing, target note, shift + retune smoothing, out-of-range flag, pause-gate activity.
 */
#include "autotuner.h"
#include "at_fft.h"
#include <math.h>
#include <string.h>

void autotuner_init(autotuner_t *t, const at_config_t *cfg)
{
    memset(t, 0, sizeof(*t));
    t->cfg = *cfg;
    at_fft_init();
    at_denoise_init(&t->denoise, cfg);
    at_biquad_init(&t->bandpass, at_sos_bandpass, 2);
    at_biquad_init(&t->highpass, at_sos_highpass, 1);
    at_pitch_init(&t->pitch, cfg);
    at_note_tracker_init(&t->notes, cfg);
    at_range_monitor_init(&t->range, cfg);
    at_lpc_init(&t->lpc, cfg);
    at_psola_init(&t->psola, cfg);

    t->retune_gain = cfg->retune_tau_s <= 0.0f ? 1.0f
                   : 1.0f - expf(-(float)AT_H / (float)AT_FS / cfg->retune_tau_s);
    t->smoothed_shift = 0.0f;
    t->last_period = (float)AT_FS / 160.0f;
    t->gate_rise = powf(10.0f, 3.0f / 10.0f * (float)AT_H / (float)AT_FS);     /* floor may rise 3 dB/s */
    t->gate_rise_unvoiced = powf(10.0f, cfg->gate_floor_rise_db_s / 10.0f * (float)AT_H / (float)AT_FS);
    t->gate_rise_voiced = powf(10.0f, cfg->gate_floor_rise_voiced_db_s / 10.0f * (float)AT_H / (float)AT_FS);
    t->gate_open_ratio = powf(10.0f, cfg->gate_open_db / 10.0f);
    t->gate_snr_ratio = powf(10.0f, cfg->gate_snr_db / 10.0f);

    t->mode = AT_MODE_NEAREST;
    t->fixed_target_midi = 72;
    autotuner_set_mode(t, cfg->mode);
    autotuner_set_target(t, cfg->target_midi);
}

int autotuner_set_mode(autotuner_t *t, int mode)
{
    if (mode != AT_MODE_NEAREST && mode != AT_MODE_FIXED) return -1;
    t->mode = mode;
    return 0;
}

int autotuner_set_target(autotuner_t *t, int midi)
{
    if (midi < AT_TARGET_MIN || midi > AT_TARGET_MAX) return -1;   /* ignore the command */
    t->fixed_target_midi = midi;
    return 0;
}

void autotuner_set_scale(autotuner_t *t, uint16_t scale_mask)
{
    at_note_tracker_set_scale(&t->notes, scale_mask);
}

static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

/* Pause-gate activity of this hop (Python: the block before `reflection = None`).
 * Pitch-only mode with the SNR gate (default): a voiced hop opens the gate only if it is also
 * gate_snr_db above the tracked background floor, so loud steady noise that happens to look pitched
 * no longer opens it. The floor follows the background quickly on unvoiced hops, slowly on voiced. */
static int gate_activity(autotuner_t *t, const float *clean, int voiced)
{
    const at_config_t *c = &t->cfg;
    int snr_gate = c->gate_pitch_only && c->gate_snr_db > 0.0f;
    float p = 0.0f;
    for (int i = 0; i < AT_H; i++) p += clean[i] * clean[i];
    p = p / (float)AT_H + 1e-14f;
    t->gate_smoothed_power += 0.3f * (p - t->gate_smoothed_power);       /* ~3-hop average */
    p = t->gate_smoothed_power;
    if (p > 1e-9f) {                                                     /* ignore digital silence */
        if (!t->gate_has_floor) { t->gate_floor_power = p; t->gate_has_floor = 1; }
        float rise = snr_gate ? (voiced ? t->gate_rise_voiced : t->gate_rise_unvoiced) : t->gate_rise;
        float f = t->gate_floor_power * rise;
        t->gate_floor_power = f < p ? f : p;
    }
    /* a click/snap puts most of the frame's energy into one 8 ms hop; a sung note spreads it evenly */
    float emax = 0.0f, esum = 0.0f;
    for (int h = 0; h < AT_HOPS_PER_FRAME; h++) {
        esum += t->hop_energy[h];
        if (t->hop_energy[h] > emax) emax = t->hop_energy[h];
    }
    int transient = emax > c->gate_transient_share * (esum + 1e-20f);
    int gate_voiced = voiced && !(c->gate_pitch_only && transient);
    if (snr_gate)                                                        /* clearly above the background */
        gate_voiced = gate_voiced && t->gate_has_floor && p > t->gate_floor_power * t->gate_snr_ratio;
    return gate_voiced || (!c->gate_pitch_only && t->gate_has_floor && p > t->gate_floor_power * t->gate_open_ratio);
}

void autotuner_process_hop(autotuner_t *t, const float *in, float *out, at_info_t *info)
{
    const at_config_t *c = &t->cfg;

    /* 1. noise suppressor: everything below sees the cleaned signal */
    if (c->denoise) at_denoise_process(&t->denoise, in, t->clean);
    else            memcpy(t->clean, in, sizeof(t->clean));

    /* 2. split into detection path and audio path */
    if (c->use_filter) at_biquad_run(&t->bandpass, t->clean, t->detect, AT_H);
    else               memcpy(t->detect, t->clean, sizeof(t->detect));
    at_biquad_run(&t->highpass, t->clean, t->audio, AT_H);

    memmove(t->frame, t->frame + AT_H, (AT_N - AT_H) * sizeof(float));
    memcpy(t->frame + AT_N - AT_H, t->detect, AT_H * sizeof(float));
    float e = 0.0f;
    for (int i = 0; i < AT_H; i++) e += t->clean[i] * t->clean[i];
    memmove(t->hop_energy, t->hop_energy + 1, (AT_HOPS_PER_FRAME - 1) * sizeof(float));
    t->hop_energy[AT_HOPS_PER_FRAME - 1] = e;
    t->samples_seen += AT_H;

    /* 3. pitch detection + voicing */
    at_pitch_result_t pr = {0, 0, 0.0f, 0.0f, 0.0f};
    float rms_db = -120.0f;
    if (t->samples_seen >= AT_N) {
        float esum = 0.0f;
        for (int h = 0; h < AT_HOPS_PER_FRAME; h++) esum += t->hop_energy[h];
        rms_db = 10.0f * log10f(esum / (float)AT_N + 1e-24f);
        at_pitch_detect(&t->pitch, t->frame, &pr);
    }
    int voiced = pr.has_peak && pr.confidence > c->voicing_thresh && rms_db > c->rms_floor_dbfs;

    /* 4. notes: controls are read once per hop */
    int mode = t->mode, fixed_target = t->fixed_target_midi;
    at_note_tracker_update(&t->notes, voiced, voiced ? at_hz_to_midi(pr.f0_hz) : 0.0f);
    int has_target_note = (mode == AT_MODE_FIXED) ? 1 : t->notes.has_target;
    int target_note = (mode == AT_MODE_FIXED) ? fixed_target : t->notes.target_note;

    /* 5. shift in semitones, clamped; relaxed to 0 when unvoiced */
    int has_target = voiced && t->notes.has_smoothed && has_target_note;
    float raw_shift = has_target ? (float)target_note - t->notes.smoothed_midi : 0.0f;
    float shift = clampf(raw_shift, -c->max_shift, c->max_shift);
    t->smoothed_shift += t->retune_gain * (shift - t->smoothed_shift);
    float pitch_ratio = exp2f(t->smoothed_shift / 12.0f);

    /* 6. out-of-range indication (fixed mode only; display only, the clamp always applies) */
    int oor;
    if (mode == AT_MODE_FIXED) {
        oor = at_range_monitor_update(&t->range, has_target, fabsf((float)fixed_target - t->notes.smoothed_midi));
    } else {
        at_range_monitor_reset(&t->range);
        oor = 0;
    }
    uint8_t status = (uint8_t)((oor ? AT_ST_OUT_OF_RANGE : 0u) | (mode == AT_MODE_FIXED ? AT_ST_MODE_FIXED : 0u)
                               | (voiced ? AT_ST_VOICED : 0u));

    if (voiced)
        t->last_period = clampf((float)AT_FS / at_midi_to_hz(t->notes.smoothed_midi),
                                (float)AT_TAU_MIN, (float)AT_TAU_MAX);

    /* 7. pause gate activity, LPC of the newest audio frame, shifter */
    int active = gate_activity(t, t->clean, voiced);
    memmove(t->audio_frame, t->audio_frame + AT_H, (AT_N - AT_H) * sizeof(float));
    memcpy(t->audio_frame + AT_N - AT_H, t->audio, AT_H * sizeof(float));
    at_lpc_analyze(&t->lpc, t->audio_frame, t->refl);
    at_psola_push(&t->psola, t->audio, t->last_period, pitch_ratio, voiced, t->refl, active, out);

    if (info) {
        info->n_end = t->samples_seen;
        info->voiced = voiced;
        info->f0_hz = voiced ? pr.f0_hz : 0.0f;
        info->confidence = pr.confidence;
        info->tau = pr.has_peak ? pr.tau : 0.0f;
        info->rms_db = rms_db;
        info->has_midi = t->notes.has_smoothed;
        info->smoothed_midi = t->notes.smoothed_midi;
        info->target = has_target_note ? target_note : -1;
        info->mode = mode;
        info->shift_raw = raw_shift;
        info->shift = shift;
        info->shift_smoothed = t->smoothed_shift;
        info->out_of_range = oor;
        info->status = status;
        info->pitch_ratio = pitch_ratio;
        info->period = t->last_period;
    }
}
