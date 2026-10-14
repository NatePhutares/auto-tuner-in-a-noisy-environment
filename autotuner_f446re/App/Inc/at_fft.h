/*
 * at_fft.h — real FFT for sizes 64..1024 (power of two), single precision.
 *
 * Packed spectrum format is the same as CMSIS-DSP arm_rfft_fast_f32, so this module can be swapped
 * for CMSIS later without touching the callers:
 *     X[0] = Re X(0)      X[1] = Re X(n/2)      X[2k] = Re X(k), X[2k+1] = Im X(k)   for 1 <= k < n/2
 *
 * Forward:  X(k) = sum x[t] e^{-j 2 pi k t / n}           (numpy.fft.rfft)
 * Inverse:  x[t] = 1/n sum ...                              (numpy.fft.irfft, includes the 1/n)
 */
#ifndef AT_FFT_H
#define AT_FFT_H

#define AT_FFT_MAX 1024

/* Builds the twiddle table. Call once before any transform (autotuner_init does it). */
void at_fft_init(void);

/* x: n real samples -> X: n floats, packed (see above). x and X must not overlap. */
void at_rfft_forward(const float *x, float *X, int n);

/* X: n floats, packed -> x: n real samples. X and x must not overlap. X is left unchanged. */
void at_rfft_inverse(const float *X, float *x, int n);

#endif /* AT_FFT_H */
