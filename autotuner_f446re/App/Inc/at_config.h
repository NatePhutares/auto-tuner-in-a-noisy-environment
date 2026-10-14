/*
 * at_config.h — every constant and tunable of the auto-tuner (C port of autotuner_v2.py, class Config).
 *
 * Two kinds of values:
 *   1. #define sizes  : fixed at compile time because they size arrays (fs, N, H, FFT, rings).
 *                       Changing fs / f_lo / f_hi also needs new biquad coefficients (at_biquad.c).
 *   2. at_config_t    : thresholds and switches, filled with the Python defaults by at_config_default().
 *                       Change a field after at_config_default() and before autotuner_init().
 *
 * Names follow the Python Config fields so the two files can be read side by side.
 */
#ifndef AT_CONFIG_H
#define AT_CONFIG_H

#include <stdint.h>

/* ---------------- framing (design doc: everything derives from fs and N) ---------------- */
#define AT_FS            16000      /* sample rate, Hz                                   */
#define AT_N             512        /* frame length (32 ms)                              */
#define AT_H             128        /* hop (8 ms) = real-time deadline                   */
#define AT_FFT_SIZE      1024       /* pitch FFT: zero-pad N -> 2N (linear autocorr.)    */
#define AT_HOPS_PER_FRAME (AT_N / AT_H)

/* singing range: tau = fs / f0 */
#define AT_TAU_MIN       16         /* round(16000 / 1000 Hz) */
#define AT_TAU_MAX       200        /* round(16000 /   80 Hz) */

/* LPC */
#define AT_LPC_ORDER     16

/* TD-PSOLA timing (Config.delay, grain_reach, psola_delay, denoise_delay) */
#define AT_PSOLA_BASE_DELAY 400                                   /* Config.delay                  */
#define AT_GRAIN_REACH   220                                      /* round(tau_max * max_stretch)  */
#define AT_PSOLA_DELAY   (AT_PSOLA_BASE_DELAY + AT_GRAIN_REACH - AT_TAU_MAX + (AT_N - AT_H) / 2)  /* 612 */
#define AT_DENOISE_DELAY (AT_N - AT_H)                             /* 384 = 24 ms                   */

/* ring buffer sizes (powers of two). The Python version keeps whole-signal arrays; every access
 * there lies within the last ~1000 samples, so 2048 leaves plenty of margin. */
#define AT_RING          2048
#define AT_MARK_RING     256        /* analysis pitch marks kept                                */
#define AT_REFL_RING     16         /* LPC reflection sets kept (one per hop)                   */

/* tuning modes */
#define AT_MODE_NEAREST  0
#define AT_MODE_FIXED    1
#define AT_TARGET_MIN    24         /* C1 */
#define AT_TARGET_MAX    108        /* C8 */

/* status byte (same bits as uart_packet.h) */
#define AT_ST_OUT_OF_RANGE  0x01u
#define AT_ST_MODE_FIXED    0x02u
#define AT_ST_VOICED        0x04u

/* scale masks for nearest mode: bit p set = pitch class p (0 = C) allowed */
#define AT_SCALE_CHROMATIC  0x0FFFu
#define AT_SCALE_MAJOR_C    0x0AB5u   /* 0 2 4 5 7 9 11 */
#define AT_SCALE_MINOR_C    0x05ADu   /* 0 2 3 5 7 8 10 */

typedef struct {
    /* detection path filter */
    uint8_t use_filter;          /* 1 = 75 Hz-1 kHz band-pass in the detection path                 */
    /* noise suppression */
    uint8_t denoise;
    float   dn_floor_db;         /* -25 dB deepest attenuation per bin                              */
    float   dn_bias;             /* 2.5 minimum-statistics bias                                     */
    float   dn_rise_db_s;        /* 6 dB/s noise estimate rise rate                                 */
    float   dn_smooth;           /* 0.8 time smoothing of the power spectrum                       */
    float   dn_beta;             /* 0.92 decision-directed a-priori SNR smoothing                  */
    /* pause gate */
    uint8_t gate;
    float   gate_depth_db;       /* -25 dB                                                           */
    float   gate_open_db;        /* 9 dB above the tracked floor (level mode only)                  */
    float   gate_snr_db;         /* 6 dB: pitch-only mode, a voiced hop must also be this far above
                                    the tracked floor (0 = off: old gate, loud steady noise opens it) */
    float   gate_floor_rise_db_s;        /* 10 dB/s floor rise on unvoiced hops (SNR gate on)        */
    float   gate_floor_rise_voiced_db_s; /* 1 dB/s on voiced hops: a held note doesn't become floor  */
    uint8_t gate_pitch_only;     /* 1 = open only on voiced hops (default)                          */
    float   gate_transient_share;/* 0.9                                                              */
    float   gate_hold_s;         /* 0.15                                                             */
    float   gate_lookahead_s;    /* 0.02                                                             */
    float   gate_attack_s;       /* 0.005                                                            */
    float   gate_release_s;      /* 0.06                                                             */
    /* pitch / voicing */
    float   voicing_thresh;      /* 0.5 peak r(tau)/r(0)                                            */
    float   rms_floor_dbfs;      /* -40                                                              */
    float   peak_rel;            /* 0.9 octave-error guard                                          */
    /* notes */
    int     median_len;          /* 5 (max 8)                                                       */
    float   hyst_cents;          /* 35                                                               */
    int     hyst_frames;         /* 3                                                                */
    uint16_t scale_mask;         /* AT_SCALE_CHROMATIC                                              */
    /* source-filter split */
    float   lpc_lag_hz;          /* 40 Hz Gaussian lag window                                       */
    int     lpc_block;           /* 32 samples per synthesis-filter update                          */
    float   residual_stretch;    /* 0.135                                                            */
    /* pitch marks */
    float   mark_tol;            /* 0.15 of the period                                              */
    /* correction */
    float   retune_tau_s;        /* 0.05 (0 = instant snap)                                         */
    uint8_t dry_unvoiced;        /* 1 = pass consonants/breath through unshifted                    */
    int     dry_after_hops;      /* 3                                                                */
    int     dry_fade;            /* 160 samples                                                     */
    uint8_t level_match;
    float   level_tau_s;         /* 0.2                                                              */
    float   level_max_db;        /* 12                                                               */
    /* tuning mode */
    int     mode;                /* AT_MODE_NEAREST                                                 */
    int     target_midi;         /* 72 = C5                                                         */
    float   max_shift;           /* 12 semitones                                                    */
    float   oor_margin;          /* 0.5                                                              */
    int     oor_frames;          /* 6                                                                */
} at_config_t;

/* Fill cfg with the autotuner_v2.py defaults. */
void at_config_default(at_config_t *cfg);

#endif /* AT_CONFIG_H */
