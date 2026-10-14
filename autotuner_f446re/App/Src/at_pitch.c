/*
 * at_pitch.c — see at_pitch.h. Port of PitchDetector.__call__().
 * Cost per hop: two 1024-point real FFTs + ~200 compares.
 */
#include "at_pitch.h"
#include "at_fft.h"
#include <math.h>
#include <string.h>

#define TWO_PI_F 6.28318530717958647692f
#define NLAG (AT_TAU_MAX + 2)

/* autocorrelation lags 0..NLAG-1 of p->buf[0..AT_N-1], zero-padded to AT_FFT_SIZE */
static void autocorr_fft(at_pitch_t *p, float *r)
{
    memset(p->buf + AT_N, 0, (AT_FFT_SIZE - AT_N) * sizeof(float));
    at_rfft_forward(p->buf, p->spec, AT_FFT_SIZE);
    p->spec[0] = p->spec[0] * p->spec[0];
    p->spec[1] = p->spec[1] * p->spec[1];
    for (int k = 1; k < AT_FFT_SIZE / 2; k++) {
        float re = p->spec[2 * k], im = p->spec[2 * k + 1];
        p->spec[2 * k] = re * re + im * im;
        p->spec[2 * k + 1] = 0.0f;
    }
    at_rfft_inverse(p->spec, p->buf, AT_FFT_SIZE);
    memcpy(r, p->buf, NLAG * sizeof(float));
}

void at_pitch_init(at_pitch_t *p, const at_config_t *cfg)
{
    p->peak_rel = cfg->peak_rel;
    for (int n = 0; n < AT_N; n++)                                  /* numpy.hanning: symmetric */
        p->window[n] = 0.5f - 0.5f * cosf(TWO_PI_F * (float)n / (float)(AT_N - 1));
    memcpy(p->buf, p->window, AT_N * sizeof(float));
    autocorr_fft(p, p->window_ac);
}

void at_pitch_detect(at_pitch_t *p, const float *frame, at_pitch_result_t *res)
{
    float *r = p->r;                                                  /* kept in the struct: small stack */
    for (int n = 0; n < AT_N; n++) p->buf[n] = frame[n] * p->window[n];
    autocorr_fft(p, r);
    for (int t = 0; t < NLAG; t++) r[t] /= p->window_ac[t];

    res->valid = 0;
    res->has_peak = 0;
    res->f0_hz = 0.0f;
    res->confidence = 0.0f;
    res->tau = 0.0f;
    if (r[0] <= 1e-12f) return;
    res->valid = 1;

    float inv0 = 1.0f / r[0];
    for (int t = 0; t < NLAG; t++) r[t] *= inv0;                     /* r(tau)/r(0) */

    /* tallest local maximum in the lag range */
    float best_peak = -1e30f;
    for (int t = AT_TAU_MIN; t <= AT_TAU_MAX; t++) {
        float c = r[t];
        if (c > r[t - 1] && c >= r[t + 1] && c > 0.0f && c > best_peak) best_peak = c;
    }
    if (best_peak < -1e29f) {                                         /* no peak at all */
        float best = r[AT_TAU_MIN];
        for (int t = AT_TAU_MIN + 1; t <= AT_TAU_MAX; t++) if (r[t] > best) best = r[t];
        res->confidence = best > 0.0f ? best : 0.0f;
        return;
    }
    /* octave-error guard: shortest-lag peak nearly as tall as the best */
    int tau0 = AT_TAU_MIN;
    for (int t = AT_TAU_MIN; t <= AT_TAU_MAX; t++) {
        float c = r[t];
        if (c > r[t - 1] && c >= r[t + 1] && c > 0.0f && c >= p->peak_rel * best_peak) { tau0 = t; break; }
    }
    float l = r[tau0 - 1], c = r[tau0], rr = r[tau0 + 1];
    float den = l - 2.0f * c + rr;
    float delta = (den < -1e-12f) ? 0.5f * (l - rr) / den : 0.0f;    /* parabolic interpolation */
    if (delta > 1.0f) delta = 1.0f;
    if (delta < -1.0f) delta = -1.0f;
    float tau = (float)tau0 + delta;
    res->has_peak = 1;
    res->tau = tau;
    res->f0_hz = (float)AT_FS / tau;
    res->confidence = c - 0.25f * (l - rr) * delta;
}
