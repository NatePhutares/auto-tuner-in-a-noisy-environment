/*
 * at_pitch.h — pitch detection by FFT autocorrelation (Wiener-Khinchin). Python: class PitchDetector.
 *
 *   r = IFFT(|FFT(hann * frame, 1024)|^2)[0..tau_max+1] / r_window      (window taper divided out)
 *   peaks of r(tau)/r(0) in tau = 16..200; take the shortest-lag peak >= 0.9 * tallest (octave guard)
 *   parabolic interpolation -> tau, confidence = interpolated peak height
 */
#ifndef AT_PITCH_H
#define AT_PITCH_H

#include "at_config.h"

typedef struct {
    float peak_rel;
    float window[AT_N];                       /* symmetric Hann, numpy.hanning(N)         */
    float window_ac[AT_TAU_MAX + 2];          /* window's own autocorrelation             */
    float buf[AT_FFT_SIZE];                   /* scratch                                  */
    float spec[AT_FFT_SIZE];                  /* scratch                                  */
    float r[AT_TAU_MAX + 2];                  /* scratch: normalised autocorrelation      */
} at_pitch_t;

typedef struct {
    int   valid;        /* 0 = silent frame (Python returns None)                                     */
    int   has_peak;     /* 0 = no peak in the lag range (Python returns f0 = nan)                     */
    float f0_hz;
    float confidence;
    float tau;          /* lag in samples (fractional)                                                */
} at_pitch_result_t;

void at_pitch_init(at_pitch_t *p, const at_config_t *cfg);

/* frame: last AT_N samples of the detection path, oldest first. */
void at_pitch_detect(at_pitch_t *p, const float *frame, at_pitch_result_t *res);

#endif /* AT_PITCH_H */
