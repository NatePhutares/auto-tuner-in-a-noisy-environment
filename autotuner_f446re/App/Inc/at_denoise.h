/*
 * at_denoise.h — streaming STFT noise suppressor (Python: class NoiseSuppressor).
 *
 * sqrt-Hann analysis/synthesis, N = 512, hop 128. Per bin:
 *   P = |X|^2,  Ps = a Ps + (1-a) P,  Nz = min(Nz * rise, Ps)        (minimum statistics)
 *   gamma = P / (bias Nz),  xi = beta G_prev^2 gamma_prev + (1-beta) max(gamma-1, 0)
 *   G = max(xi / (1 + xi), floor), then 3-tap smoothing over frequency
 * Output delay: N - H = 384 samples.
 */
#ifndef AT_DENOISE_H
#define AT_DENOISE_H

#include "at_config.h"

#define AT_DN_BINS (AT_N / 2 + 1)

typedef struct {
    float smooth, bias, beta, gain_floor, rise;
    float window[AT_N];             /* sqrt-Hann (periodic)                  */
    float input[AT_N];              /* last N input samples                 */
    float ola[AT_N];                /* overlap-add accumulator              */
    float spec[AT_N];               /* packed spectrum (scratch)            */
    float frame[AT_N];              /* scratch                              */
    float smoothed_power[AT_DN_BINS];
    float noise_power[AT_DN_BINS];
    float prev_gain[AT_DN_BINS];
    float prev_gamma[AT_DN_BINS];
    float gain[AT_DN_BINS];         /* scratch                              */
    int   started;
} at_denoise_t;

void at_denoise_init(at_denoise_t *d, const at_config_t *cfg);

/* in: AT_H samples, out: AT_H cleaned samples (delayed by AT_DENOISE_DELAY). in == out allowed. */
void at_denoise_process(at_denoise_t *d, const float *in, float *out);

#endif /* AT_DENOISE_H */
