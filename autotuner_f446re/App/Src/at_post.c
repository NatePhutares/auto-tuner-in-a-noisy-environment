/*
 * at_post.c — see at_post.h. Per-sample cost: a few multiply-adds per stage.
 */
#include "at_post.h"
#include "at_ring.h"
#include <math.h>

/* ---------------------------------------------------------------- 1. dry crossfade */
void at_xfade_init(at_xfade_t *x, const at_config_t *cfg)
{
    x->step = 1.0f / (float)(cfg->dry_fade > 1 ? cfg->dry_fade : 1);
    x->blend = 0.0f;
}

void at_xfade_run(at_xfade_t *x, float *block, int32_t start, int len,
                  const float *dry_ring, const uint8_t *use_dry_ring)
{
    float b = x->blend;
    for (int i = 0; i < len; i++) {
        int32_t n = start + i;
        if (at_ring_getb(use_dry_ring, n)) { b += x->step; if (b > 1.0f) b = 1.0f; }
        else                               { b -= x->step; if (b < 0.0f) b = 0.0f; }
        block[i] = (1.0f - b) * block[i] + b * at_ring_getf(dry_ring, n);
    }
    x->blend = b;
}

/* ---------------------------------------------------------------- 2. level match */
void at_level_init(at_level_t *l, const at_config_t *cfg)
{
    l->tau_samples = (float)AT_FS * cfg->level_tau_s;
    l->gain_limit = powf(10.0f, cfg->level_max_db / 20.0f);
    l->in_power = 0.0f;
    l->out_power = 0.0f;
    l->gain = 1.0f;
}

void at_level_run(at_level_t *l, float *block, int32_t start, int len, const float *dry_ring)
{
    if (len <= 0) return;
    float pin = 0.0f, pout = 0.0f;
    for (int i = 0; i < len; i++) {
        float d = at_ring_getf(dry_ring, start + i);
        pin += d * d;
        pout += block[i] * block[i];
    }
    pin /= (float)len;
    pout /= (float)len;
    if (pin > 1e-7f && pout > 1e-9f) {                 /* about -70 dBFS: silence never updates */
        float a = 1.0f - expf(-(float)len / l->tau_samples);
        l->in_power += a * (pin - l->in_power);
        l->out_power += a * (pout - l->out_power);
    }
    float new_gain = l->gain;
    if (l->out_power > 0.0f) {
        new_gain = sqrtf(l->in_power / l->out_power);
        if (new_gain > l->gain_limit) new_gain = l->gain_limit;
        if (new_gain < 1.0f / l->gain_limit) new_gain = 1.0f / l->gain_limit;
    }
    float g0 = l->gain, dg = (new_gain - g0) / (float)len;   /* linear ramp across the block */
    for (int i = 0; i < len; i++) block[i] *= g0 + dg * (float)(i + 1);
    l->gain = new_gain;
}

/* ---------------------------------------------------------------- 3. pause gate */
void at_gate_init(at_gate_t *g, const at_config_t *cfg)
{
    g->hold = (int32_t)(cfg->gate_hold_s * (float)AT_FS);
    g->ahead = (int32_t)(cfg->gate_lookahead_s * (float)AT_FS);
    g->depth = powf(10.0f, cfg->gate_depth_db / 20.0f);
    g->attack = 1.0f - expf(-1.0f / (cfg->gate_attack_s * (float)AT_FS));
    g->release = 1.0f - expf(-1.0f / (cfg->gate_release_s * (float)AT_FS));
    g->gain = 1.0f;
    g->last_active = INT32_MIN / 2;
    g->scan_pos = 0;
}

/* Sample n is "open" if any active sample lies in [n - hold, n + ahead] (and below samples_received).
 * last_active is the newest active index seen so far, which answers that in O(1) per sample. */
void at_gate_run(at_gate_t *g, float *block, int32_t start, int len,
                 const uint8_t *active_ring, int32_t samples_received)
{
    float gain = g->gain;
    for (int i = 0; i < len; i++) {
        int32_t n = start + i;
        int32_t limit = n + g->ahead;
        if (limit > samples_received - 1) limit = samples_received - 1;
        while (g->scan_pos <= limit) {
            if (at_ring_getb(active_ring, g->scan_pos)) g->last_active = g->scan_pos;
            g->scan_pos++;
        }
        float wanted = (g->last_active >= n - g->hold) ? 1.0f : g->depth;
        gain += ((wanted > gain) ? g->attack : g->release) * (wanted - gain);
        block[i] *= gain;
    }
    g->gain = gain;
}
