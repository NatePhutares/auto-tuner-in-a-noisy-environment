/*
 * at_pc_main.c — runs the C auto-tuner on a PC, for checking it against autotuner_v2.py.
 * Not part of the STM32 build (pc_test is not a CubeIDE source folder).
 *
 *   gcc -O2 -std=c11 -Wall -Wextra -Wdouble-promotion -I../App/Inc ../App/Src/at_*.c ../App/Src/autotuner.c \
 *       at_pc_main.c -lm -o at_pc
 *   ./at_pc input_q15.raw out_c.raw hops_c.csv [nearest|fixed] [target_midi]
 *
 * Input/output: raw little-endian int16 at 16 kHz (the format export_golden() writes).
 * Output is aligned to the input like Python run(): out[t] ~ in[t] (the 996-sample delay is removed).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "autotuner.h"

static autotuner_t tuner;

int main(int argc, char **argv)
{
    if (argc < 4) { fprintf(stderr, "usage: %s in.raw out.raw hops.csv [nearest|fixed] [target]\n", argv[0]); return 1; }
    FILE *f = fopen(argv[1], "rb");
    if (!f) { perror(argv[1]); return 1; }
    fseek(f, 0, SEEK_END);
    long n = ftell(f) / 2;
    fseek(f, 0, SEEK_SET);
    int16_t *x = malloc((size_t)n * 2);
    if (fread(x, 2, (size_t)n, f) != (size_t)n) { fprintf(stderr, "read error\n"); return 1; }
    fclose(f);

    at_config_t cfg;
    at_config_default(&cfg);
    if (argc > 4 && strcmp(argv[4], "fixed") == 0) cfg.mode = AT_MODE_FIXED;
    if (argc > 5) cfg.target_midi = at_parse_note(argv[5]);
    autotuner_init(&tuner, &cfg);

    const long delay = AT_DENOISE_DELAY + AT_PSOLA_DELAY + AT_H;   /* hop-aligned output */
    long total = ((n + delay + 2 * AT_H) / AT_H + 1) * AT_H;
    float *y = calloc((size_t)total, sizeof(float));
    FILE *csv = fopen(argv[3], "w");
    fprintf(csv, "hop,n_end,f0,conf,tau,rms_db,voiced,m_s,target,mode,d_raw,d,d_s,oor,status,alpha,T\n");
    float in[AT_H];
    at_info_t info;
    clock_t t0 = clock();
    for (long h = 0; h * AT_H < total; h++) {
        for (int i = 0; i < AT_H; i++) {
            long k = h * AT_H + i;
            in[i] = k < n ? (float)x[k] / 32768.0f : 0.0f;
        }
        autotuner_process_hop(&tuner, in, &y[h * AT_H], &info);
        fprintf(csv, "%ld,%d,%.9g,%.9g,%.9g,%.9g,%d,%.9g,%d,%d,%.9g,%.9g,%.9g,%d,%d,%.9g,%.9g\n",
                h, (int)info.n_end, info.voiced ? (double)info.f0_hz : (double)NAN, (double)info.confidence,
                info.tau > 0 ? (double)info.tau : (double)NAN, (double)info.rms_db, info.voiced,
                info.has_midi ? (double)info.smoothed_midi : (double)NAN, info.target, info.mode,
                (double)info.shift_raw, (double)info.shift, (double)info.shift_smoothed, info.out_of_range,
                info.status, (double)info.pitch_ratio, (double)info.period);
    }
    double secs = (double)(clock() - t0) / CLOCKS_PER_SEC;
    fclose(csv);

    /* remove the 996-sample pipeline delay (the extra hop of alignment is part of `delay` above) */
    const long lag = AT_DENOISE_DELAY + AT_PSOLA_DELAY;
    FILE *o = fopen(argv[2], "wb");
    for (long t = 0; t < n; t++) {
        float v = y[t + lag] * 32768.0f;
        long q = lrintf(v);
        if (q > 32767) q = 32767;
        if (q < -32768) q = -32768;
        int16_t s = (int16_t)q;
        fwrite(&s, 2, 1, o);
    }
    fclose(o);
    fprintf(stderr, "%ld samples (%.1f s audio) in %.2f s CPU\n", n, (double)n / 16000.0, secs);
    return 0;
}
