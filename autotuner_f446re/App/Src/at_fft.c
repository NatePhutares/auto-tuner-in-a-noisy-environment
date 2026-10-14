/*
 * at_fft.c — real FFT via an n/2-point complex FFT plus a split step.
 *
 * Two back-ends, same packed output (see at_fft.h):
 *   default            : portable radix-2 code below (no library needed)
 *   AT_USE_CMSIS_DSP   : CMSIS-DSP arm_rfft_fast_f32 (define the symbol and add CMSIS-DSP to the
 *                        project, see C_PORT.md). Roughly 2-3x faster on the Cortex-M4.
 *
 * Portable back-end: one table cos/sin(2 pi k / 1024), k < 512, serves every size n <= 1024
 * (stride 1024/n); bit-reversal permutations are precomputed for n/2 = 256 and 512.
 * The inverse uses IFFT(Z) = conj(FFT(conj(Z))) so one butterfly loop serves both directions.
 */
#include "at_fft.h"
#include <math.h>
#include <stdint.h>

#ifdef AT_USE_CMSIS_DSP
/* ------------------------------------------------------------------ CMSIS-DSP back-end */
#include "arm_math.h"

static arm_rfft_fast_instance_f32 s_rfft512, s_rfft1024;
static float s_tmp[AT_FFT_MAX];
static int   s_ready = 0;

void at_fft_init(void)
{
    arm_rfft_fast_init_f32(&s_rfft512, 512);
    arm_rfft_fast_init_f32(&s_rfft1024, 1024);
    s_ready = 1;
}

static arm_rfft_fast_instance_f32 *instance(int n)
{
    if (!s_ready) at_fft_init();
    return n == 1024 ? &s_rfft1024 : &s_rfft512;    /* the pipeline only uses 512 and 1024 */
}

void at_rfft_forward(const float *x, float *X, int n)
{
    for (int i = 0; i < n; i++) s_tmp[i] = x[i];    /* CMSIS modifies its input */
    arm_rfft_fast_f32(instance(n), s_tmp, X, 0);
}

void at_rfft_inverse(const float *X, float *x, int n)
{
    for (int i = 0; i < n; i++) s_tmp[i] = X[i];
    arm_rfft_fast_f32(instance(n), s_tmp, x, 1);    /* includes the 1/n scaling */
}

#else
/* ------------------------------------------------------------------ portable back-end */
#define TWO_PI_F 6.28318530717958647692f

static float    s_cos[AT_FFT_MAX / 2];
static float    s_sin[AT_FFT_MAX / 2];
static uint16_t s_rev256[256];
static uint16_t s_rev512[512];
static float    s_work[AT_FFT_MAX];          /* n/2 complex values, interleaved re/im */
static int      s_ready = 0;

static void make_rev(uint16_t *rev, int m)
{
    int bits = 0;
    while ((1 << bits) < m) bits++;
    for (int i = 0; i < m; i++) {
        int r = 0;
        for (int b = 0; b < bits; b++) if (i & (1 << b)) r |= 1 << (bits - 1 - b);
        rev[i] = (uint16_t)r;
    }
}

void at_fft_init(void)
{
    for (int k = 0; k < AT_FFT_MAX / 2; k++) {
        float ph = TWO_PI_F * (float)k / (float)AT_FFT_MAX;
        s_cos[k] = cosf(ph);
        s_sin[k] = sinf(ph);
    }
    make_rev(s_rev256, 256);
    make_rev(s_rev512, 512);
    s_ready = 1;
}

static void bit_reverse(float *d, int m)
{
    const uint16_t *rev = (m == 512) ? s_rev512 : (m == 256) ? s_rev256 : 0;
    for (int i = 0, j = 0; i < m; i++) {
        if (rev) j = rev[i];
        else {                                        /* other sizes: incremental reversal */
            if (i > 0) { int bit = m >> 1; for (; j & bit; bit >>= 1) j ^= bit; j ^= bit; }
        }
        if (i < j) {
            float tr = d[2 * i], ti = d[2 * i + 1];
            d[2 * i] = d[2 * j];  d[2 * i + 1] = d[2 * j + 1];
            d[2 * j] = tr;        d[2 * j + 1] = ti;
        }
    }
}

/* In-place forward complex FFT of m points, data interleaved, no scaling. */
static void cfft_forward(float *d, int m)
{
    bit_reverse(d, m);
    /* stage len = 2: twiddle 1 */
    for (int s = 0; s < 2 * m; s += 4) {
        float ar = d[s], ai = d[s + 1], br = d[s + 2], bi = d[s + 3];
        d[s] = ar + br;     d[s + 1] = ai + bi;
        d[s + 2] = ar - br; d[s + 3] = ai - bi;
    }
    /* stage len = 4: twiddles 1 and -j */
    for (int s = 0; s < 2 * m; s += 8) {
        float *a0 = &d[s], *a1 = &d[s + 2], *b0 = &d[s + 4], *b1 = &d[s + 6];
        float br = b0[0], bi = b0[1];
        b0[0] = a0[0] - br; b0[1] = a0[1] - bi;
        a0[0] += br;        a0[1] += bi;
        br = b1[1]; bi = -b1[0];                      /* b1 * (-j) */
        b1[0] = a1[0] - br; b1[1] = a1[1] - bi;
        a1[0] += br;        a1[1] += bi;
    }
    for (int len = 8; len <= m; len <<= 1) {
        int half = len >> 1;
        int stride = AT_FFT_MAX / len;
        for (int t = 0; t < half; t++) {
            float wr = s_cos[t * stride], wi = -s_sin[t * stride];   /* e^{-j 2 pi t / len} */
            for (int s = 2 * t; s < 2 * m; s += 2 * len) {
                float *a = &d[s], *b = &d[s + len];
                float br = b[0] * wr - b[1] * wi;
                float bi = b[0] * wi + b[1] * wr;
                b[0] = a[0] - br;  b[1] = a[1] - bi;
                a[0] += br;        a[1] += bi;
            }
        }
    }
}

void at_rfft_forward(const float *x, float *X, int n)
{
    if (!s_ready) at_fft_init();
    int m = n / 2;
    int stride = AT_FFT_MAX / n;
    for (int i = 0; i < n; i++) s_work[i] = x[i];        /* z[k] = x[2k] + j x[2k+1] */
    cfft_forward(s_work, m);

    X[0] = s_work[0] + s_work[1];                        /* DC      */
    X[1] = s_work[0] - s_work[1];                        /* Nyquist */
    for (int k = 1; k < m; k++) {
        float zr = s_work[2 * k],       zi = s_work[2 * k + 1];
        float cr = s_work[2 * (m - k)], ci = -s_work[2 * (m - k) + 1];   /* conj(Z[m-k]) */
        float er = 0.5f * (zr + cr), ei = 0.5f * (zi + ci);              /* even part        */
        float o_r = 0.5f * (zi - ci), o_i = -0.5f * (zr - cr);           /* -j (Z - conj)/2  */
        float wr = s_cos[k * stride], wi = -s_sin[k * stride];           /* e^{-j 2 pi k/n}  */
        X[2 * k]     = er + (wr * o_r - wi * o_i);
        X[2 * k + 1] = ei + (wr * o_i + wi * o_r);
    }
}

void at_rfft_inverse(const float *X, float *x, int n)
{
    if (!s_ready) at_fft_init();
    int m = n / 2;
    int stride = AT_FFT_MAX / n;
    /* rebuild Z[k] = E[k] + j O[k] (stored conjugated for the conj-FFT-conj inverse) */
    s_work[0] = 0.5f * (X[0] + X[1]);
    s_work[1] = -0.5f * (X[0] - X[1]);
    for (int k = 1; k < m; k++) {
        float ar = X[2 * k],       ai = X[2 * k + 1];
        float br = X[2 * (m - k)], bi = -X[2 * (m - k) + 1];   /* conj X[m-k] */
        float er = 0.5f * (ar + br), ei = 0.5f * (ai + bi);
        float dr = 0.5f * (ar - br), di = 0.5f * (ai - bi);
        float wr = s_cos[k * stride], wi = s_sin[k * stride]; /* e^{+j 2 pi k / n} */
        float o_r = dr * wr - di * wi, o_i = dr * wi + di * wr;
        s_work[2 * k]     = er - o_i;
        s_work[2 * k + 1] = -(ei + o_r);
    }
    cfft_forward(s_work, m);
    float scale = 1.0f / (float)m;
    for (int i = 0; i < m; i++) {
        x[2 * i]     = s_work[2 * i] * scale;
        x[2 * i + 1] = -s_work[2 * i + 1] * scale;
    }
}
#endif /* AT_USE_CMSIS_DSP */
