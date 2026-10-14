/*
 * at_config.c — default values, identical to the Config dataclass in autotuner.py.
 */
#include "at_config.h"

void at_config_default(at_config_t *c)
{
    c->use_filter = 1;

    c->denoise = 1;
    c->dn_floor_db = -25.0f;
    c->dn_bias = 2.5f;
    c->dn_rise_db_s = 6.0f;
    c->dn_smooth = 0.8f;
    c->dn_beta = 0.92f;

    c->gate = 1;
    c->gate_depth_db = -25.0f;
    c->gate_open_db = 9.0f;
    c->gate_snr_db = 6.0f;
    c->gate_floor_rise_db_s = 10.0f;
    c->gate_floor_rise_voiced_db_s = 1.0f;
    c->gate_pitch_only = 1;
    c->gate_transient_share = 0.9f;
    c->gate_hold_s = 0.15f;
    c->gate_lookahead_s = 0.02f;
    c->gate_attack_s = 0.005f;
    c->gate_release_s = 0.06f;

    c->voicing_thresh = 0.5f;
    c->rms_floor_dbfs = -40.0f;
    c->peak_rel = 0.9f;

    c->median_len = 5;
    c->hyst_cents = 35.0f;
    c->hyst_frames = 3;
    c->scale_mask = AT_SCALE_CHROMATIC;

    c->lpc_lag_hz = 40.0f;
    c->lpc_block = 32;
    c->residual_stretch = 0.135f;

    c->mark_tol = 0.15f;

    c->retune_tau_s = 0.05f;
    c->dry_unvoiced = 1;
    c->dry_after_hops = 3;
    c->dry_fade = 160;
    c->level_match = 1;
    c->level_tau_s = 0.2f;
    c->level_max_db = 12.0f;

    c->mode = AT_MODE_NEAREST;
    c->target_midi = 72;
    c->max_shift = 12.0f;
    c->oor_margin = 0.5f;
    c->oor_frames = 6;
}
