/*
 * at_biquad.h — IIR biquad cascade, transposed direct form II (same structure as scipy.signal.sosfilt).
 *
 * Coefficient sets used by the pipeline (fs = 16 kHz):
 *   at_sos_bandpass : 4th-order Butterworth band-pass 75 Hz - 1 kHz, 2 sections (detection path)
 *   at_sos_highpass : 2nd-order Butterworth high-pass 75 Hz, 1 section (audio path)
 */
#ifndef AT_BIQUAD_H
#define AT_BIQUAD_H

#define AT_BIQUAD_MAX_SECTIONS 2

typedef struct {
    float b0, b1, b2, a1, a2;   /* a0 = 1 */
} at_sos_t;

typedef struct {
    const at_sos_t *sos;
    int   sections;
    float z[AT_BIQUAD_MAX_SECTIONS][2];   /* state per section */
} at_biquad_t;

extern const at_sos_t at_sos_bandpass[2];
extern const at_sos_t at_sos_highpass[1];

void at_biquad_init(at_biquad_t *f, const at_sos_t *sos, int sections);

/* Filters len samples; in and out may be the same buffer. */
void at_biquad_run(at_biquad_t *f, const float *in, float *out, int len);

#endif /* AT_BIQUAD_H */
