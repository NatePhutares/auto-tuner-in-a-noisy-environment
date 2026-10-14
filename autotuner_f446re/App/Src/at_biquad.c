/*
 * at_biquad.c — biquad cascade + the two coefficient sets of autotuner.py.
 *
 * Coefficients were printed from the Python prototype (fs = 16000):
 *     signal.butter(2, [75, 1000], btype="bandpass", fs=16000, output="sos")
 *     signal.butter(2, 75, "highpass", fs=16000, output="sos")
 * If fs or the band edges change, regenerate them with those two lines and paste them here.
 */
#include "at_biquad.h"

const at_sos_t at_sos_bandpass[2] = {
    { 0.026074803746257019f, 0.052149607492514037f, 0.026074803746257019f,
      -1.5144522504469842f, 0.62320308870522911f },
    { 1.0f, -2.0f, 1.0f,
      -1.9592476244026351f, 0.96021806322027492f },
};

const at_sos_t at_sos_highpass[1] = {
    { 0.97938932735214013f, -1.9587786547042803f, 0.97938932735214013f,
      -1.9583538097539748f, 0.95920349965458518f },
};

void at_biquad_init(at_biquad_t *f, const at_sos_t *sos, int sections)
{
    f->sos = sos;
    f->sections = sections;
    for (int s = 0; s < AT_BIQUAD_MAX_SECTIONS; s++) {
        f->z[s][0] = 0.0f;
        f->z[s][1] = 0.0f;
    }
}

void at_biquad_run(at_biquad_t *f, const float *in, float *out, int len)
{
    for (int i = 0; i < len; i++) {
        float x = in[i];
        for (int s = 0; s < f->sections; s++) {
            const at_sos_t *c = &f->sos[s];
            float y = c->b0 * x + f->z[s][0];
            f->z[s][0] = c->b1 * x - c->a1 * y + f->z[s][1];
            f->z[s][1] = c->b2 * x - c->a2 * y;
            x = y;
        }
        out[i] = x;
    }
}
