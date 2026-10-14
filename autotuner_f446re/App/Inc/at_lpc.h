/*
 * at_lpc.h — LPC (vocal-tract envelope) helpers. Python: levinson, step_up, AutoTuner._lpc,
 * and the all-pole synthesis filter used in TDPSOLA._synthesize.
 *
 * A(z) = 1 + sum_{i=1..p} a[i] z^-i  (p = AT_LPC_ORDER).  Reflection coefficients k[0..p-1].
 */
#ifndef AT_LPC_H
#define AT_LPC_H

#include "at_config.h"

/* ---- analysis: reflection coefficients of the newest N-sample audio frame ---- */
typedef struct {
    float window[AT_N];                  /* symmetric Hann                                  */
    float lag_window[AT_LPC_ORDER + 1];  /* Gaussian lag window exp(-0.5 (2 pi f_lag k / fs)^2) */
    float windowed[AT_N];                /* scratch                                         */
    float last_reflection[AT_LPC_ORDER]; /* kept when a frame is silent                     */
} at_lpc_t;

void at_lpc_init(at_lpc_t *l, const at_config_t *cfg);

/* frame: AT_N samples (oldest first). Writes AT_LPC_ORDER reflection coefficients to refl. */
void at_lpc_analyze(at_lpc_t *l, const float *frame, float *refl);

/* Levinson-Durbin: autocorr[0..order] -> a[0..order] (a[0] = 1), refl[0..order-1]. */
void at_levinson(const float *autocorr, int order, float *a, float *refl);

/* Reflection -> A(z) coefficients a[0..order] (step-up recursion). */
void at_step_up(const float *refl, int order, float *a);

/* ---- synthesis 1/A(z), keeps its own past outputs between calls ---- */
typedef struct {
    float past[AT_LPC_ORDER];            /* past outputs, newest first */
} at_allpole_t;

void at_allpole_init(at_allpole_t *f);
/* y[n] = x[n] - sum a[i] y[n-i];  x and y may be the same buffer */
void at_allpole_run(at_allpole_t *f, const float *a, const float *x, float *y, int len);

#endif /* AT_LPC_H */
