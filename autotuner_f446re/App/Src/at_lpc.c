/*
 * at_lpc.c — see at_lpc.h.
 * Cost per hop: autocorrelation 17 x 512 MACs + Levinson (~150 MACs).
 */
#include "at_lpc.h"
#include <math.h>
#include <string.h>

#define TWO_PI_F 6.28318530717958647692f

void at_lpc_init(at_lpc_t *l, const at_config_t *cfg)
{
    memset(l, 0, sizeof(*l));
    for (int n = 0; n < AT_N; n++)
        l->window[n] = 0.5f - 0.5f * cosf(TWO_PI_F * (float)n / (float)(AT_N - 1));
    for (int k = 0; k <= AT_LPC_ORDER; k++) {
        float x = TWO_PI_F * cfg->lpc_lag_hz * (float)k / (float)AT_FS;
        l->lag_window[k] = expf(-0.5f * x * x);
    }
}

void at_levinson(const float *r, int order, float *a, float *refl)
{
    float prev[AT_LPC_ORDER + 1];
    for (int i = 0; i <= order; i++) a[i] = 0.0f;
    for (int i = 0; i < order; i++) refl[i] = 0.0f;
    a[0] = 1.0f;
    float err = r[0];
    for (int m = 1; m <= order; m++) {
        float acc = r[m];
        for (int i = 1; i < m; i++) acc += a[i] * r[m - i];
        float k = -acc / err;
        refl[m - 1] = k;
        memcpy(prev, a, (size_t)(m + 1) * sizeof(float));
        for (int i = 1; i < m; i++) a[i] = prev[i] + k * prev[m - i];
        a[m] = k;
        err *= (1.0f - k * k);
        if (err <= 0.0f) break;
    }
}

void at_step_up(const float *refl, int order, float *a)
{
    float prev[AT_LPC_ORDER + 1];
    for (int i = 0; i <= order; i++) a[i] = 0.0f;
    a[0] = 1.0f;
    for (int m = 1; m <= order; m++) {
        memcpy(prev, a, (size_t)(m + 1) * sizeof(float));
        for (int i = 1; i < m; i++) a[i] = prev[i] + refl[m - 1] * prev[m - i];
        a[m] = refl[m - 1];
    }
}

void at_lpc_analyze(at_lpc_t *l, const float *frame, float *refl)
{
    float r[AT_LPC_ORDER + 1];
    float a[AT_LPC_ORDER + 1];
    for (int n = 0; n < AT_N; n++) l->windowed[n] = frame[n] * l->window[n];
    for (int k = 0; k <= AT_LPC_ORDER; k++) {
        float acc = 0.0f;
        for (int n = 0; n + k < AT_N; n++) acc += l->windowed[n] * l->windowed[n + k];
        r[k] = acc;
    }
    if (r[0] < 1e-9f) {                                       /* silent: keep the previous envelope */
        memcpy(refl, l->last_reflection, sizeof(l->last_reflection));
        return;
    }
    for (int k = 0; k <= AT_LPC_ORDER; k++) r[k] *= l->lag_window[k];
    r[0] *= 1.0f + 1e-4f;                                     /* white-noise correction (-40 dB) */
    at_levinson(r, AT_LPC_ORDER, a, l->last_reflection);
    memcpy(refl, l->last_reflection, sizeof(l->last_reflection));
}

void at_allpole_init(at_allpole_t *f)
{
    memset(f, 0, sizeof(*f));
}

void at_allpole_run(at_allpole_t *f, const float *a, const float *x, float *y, int len)
{
    /* h[0..15] = past outputs oldest..newest, then the new outputs: no shifting per sample */
    float h[AT_LPC_ORDER + 64];
    for (int i = 0; i < AT_LPC_ORDER; i++) h[i] = f->past[AT_LPC_ORDER - 1 - i];
    int done = 0;
    while (done < len) {
        int chunk = len - done < 64 ? len - done : 64;
        for (int n = 0; n < chunk; n++) {
            float *hn = &h[AT_LPC_ORDER + n];               /* hn[-i] = y[n - i] */
            float acc = x[done + n];
            for (int i = 1; i <= AT_LPC_ORDER; i++) acc -= a[i] * hn[-i];
            *hn = acc;
            y[done + n] = acc;
        }
        for (int i = 0; i < AT_LPC_ORDER; i++) h[i] = h[chunk + i];   /* keep the newest 16 */
        done += chunk;
    }
    for (int i = 0; i < AT_LPC_ORDER; i++) f->past[i] = h[AT_LPC_ORDER - 1 - i];
}
