/*
 * at_denoise.c — see at_denoise.h. Line-by-line port of NoiseSuppressor.process().
 * Cost per hop: one 512-point real FFT + one inverse + ~257 x 20 flops.
 */
#include "at_denoise.h"
#include "at_fft.h"
#include <math.h>
#include <string.h>

#define TWO_PI_F 6.28318530717958647692f

void at_denoise_init(at_denoise_t *d, const at_config_t *cfg)
{
    memset(d, 0, sizeof(*d));
    d->smooth = cfg->dn_smooth;
    d->bias = cfg->dn_bias;
    d->beta = cfg->dn_beta;
    d->gain_floor = powf(10.0f, cfg->dn_floor_db / 20.0f);
    d->rise = powf(10.0f, cfg->dn_rise_db_s / 10.0f * (float)AT_H / (float)AT_FS);
    for (int n = 0; n < AT_N; n++)
        d->window[n] = sqrtf(0.5f * (1.0f - cosf(TWO_PI_F * (float)n / (float)AT_N)));
    for (int k = 0; k < AT_DN_BINS; k++) {
        d->prev_gain[k] = 1.0f;
        d->prev_gamma[k] = 1.0f;
    }
}

/* power of bin k from the packed spectrum */
static inline float bin_power(const float *X, int k)
{
    if (k == 0)        return X[0] * X[0];
    if (k == AT_N / 2) return X[1] * X[1];
    return X[2 * k] * X[2 * k] + X[2 * k + 1] * X[2 * k + 1];
}

/* Wiener gain per bin, updates the noise tracker and the decision-directed memory. */
static void compute_gain(at_denoise_t *d)
{
    for (int k = 0; k < AT_DN_BINS; k++) {
        float p = bin_power(d->spec, k);
        if (!d->started) {
            d->smoothed_power[k] = p;
            d->noise_power[k] = p;
        } else {
            d->smoothed_power[k] = d->smooth * d->smoothed_power[k] + (1.0f - d->smooth) * p;
        }
        float nz = d->noise_power[k] * d->rise;
        if (d->smoothed_power[k] < nz) nz = d->smoothed_power[k];
        d->noise_power[k] = nz;

        float post = p / (d->bias * nz + 1e-12f);
        float ml = post - 1.0f;
        if (ml < 0.0f) ml = 0.0f;
        float prior = d->beta * d->prev_gain[k] * d->prev_gain[k] * d->prev_gamma[k] + (1.0f - d->beta) * ml;
        float g = prior / (1.0f + prior);
        if (g < d->gain_floor) g = d->gain_floor;
        d->gain[k] = g;
        d->prev_gain[k] = g;
        d->prev_gamma[k] = post;
    }
    d->started = 1;
}

/* 3-tap [0.25 0.5 0.25] smoothing over frequency with edge padding, applied to the packed spectrum */
static void apply_smoothed_gain(at_denoise_t *d)
{
    const int last = AT_DN_BINS - 1;
    for (int k = 0; k < AT_DN_BINS; k++) {
        float gl = d->gain[k > 0 ? k - 1 : 0];
        float gr = d->gain[k < last ? k + 1 : last];
        float g = 0.25f * gl + 0.5f * d->gain[k] + 0.25f * gr;
        if (k == 0)            d->spec[0] *= g;
        else if (k == last)    d->spec[1] *= g;
        else { d->spec[2 * k] *= g; d->spec[2 * k + 1] *= g; }
    }
}

void at_denoise_process(at_denoise_t *d, const float *in, float *out)
{
    memmove(d->input, d->input + AT_H, (AT_N - AT_H) * sizeof(float));
    memcpy(d->input + AT_N - AT_H, in, AT_H * sizeof(float));

    for (int n = 0; n < AT_N; n++) d->frame[n] = d->input[n] * d->window[n];
    at_rfft_forward(d->frame, d->spec, AT_N);

    compute_gain(d);
    apply_smoothed_gain(d);

    at_rfft_inverse(d->spec, d->frame, AT_N);
    for (int n = 0; n < AT_N; n++) d->ola[n] += d->frame[n] * d->window[n] * 0.5f;

    memcpy(out, d->ola, AT_H * sizeof(float));
    memmove(d->ola, d->ola + AT_H, (AT_N - AT_H) * sizeof(float));
    memset(d->ola + AT_N - AT_H, 0, AT_H * sizeof(float));
}
