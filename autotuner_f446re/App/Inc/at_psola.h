/*
 * at_psola.h — the pitch shifter: LPC residual + TD-PSOLA + 1/A(z) synthesis, streaming.
 * Python: class TDPSOLA (source_filter = True, mark_mode = "xcorr", ola_norm = "alpha").
 *
 * Timeline when `samples_received` samples have arrived (end of the hop):
 *   residual known up to          samples_received - 192   (LPC frame centre + H/2)
 *   output finalised up to        samples_received - AT_PSOLA_DELAY  (612)
 *   synthesis marks placed up to  finalised + AT_GRAIN_REACH
 *
 * Internal steps (static functions in at_psola.c, one per Python method):
 *   update_residual  e = A(z) x around the newest frame centre          _update_residual
 *   extend_marks     analysis pitch marks by normalised cross-corr.     _extend_marks(_xcorr)
 *   place            synthesis marks every period/alpha, nearest grain   _place
 *   ola              one Hann grain (stretched when lowering)           _ola
 *   synthesize       1/A(z) with interpolated reflection coefficients    _synthesize
 *   emit             finalise a block, then at_post stages              _emit
 */
#ifndef AT_PSOLA_H
#define AT_PSOLA_H

#include <stdint.h>
#include "at_config.h"
#include "at_lpc.h"
#include "at_post.h"

typedef struct {
    /* config copies */
    float   residual_stretch, mark_tol;
    int     lpc_block, dry_unvoiced, dry_after_hops, use_gate, use_level;

    /* sample rings (absolute index n -> [n & AT_RING_MASK]) */
    float   audio[AT_RING];          /* audio path (high-passed, cleaned)              */
    float   residual[AT_RING];       /* LPC residual of the audio path                 */
    float   ola[AT_RING];            /* overlap-add accumulator                        */
    float   out[AT_RING];            /* finished output                                */
    uint8_t use_dry[AT_RING];        /* 1 = use the dry input here (unvoiced)          */
    uint8_t active[AT_RING];         /* 1 = someone is singing here (pause gate)       */

    int32_t samples_received;
    int32_t residual_end;            /* residual known for n < residual_end            */
    int32_t samples_emitted;

    /* LPC reflection sets, one per hop (ring indexed by hop number) */
    float   refl[AT_REFL_RING][AT_LPC_ORDER];
    int32_t refl_count;

    /* analysis marks (ring indexed by mark number) */
    int32_t marks[AT_MARK_RING];
    int32_t mark_count;
    int32_t nearest_mark;            /* absolute mark number */
    int32_t first_mark;

    /* next synthesis mark = syn_int + syn_frac (split so it stays exact after hours) */
    int     has_next;
    int32_t syn_int;
    float   syn_frac;

    int     unvoiced_hops;

    /* output stages */
    at_allpole_t synth;
    at_xfade_t   xfade;
    at_level_t   level;
    at_gate_t    gate;

    /* scratch */
    float   grain[2 * AT_TAU_MAX + 1];
    float   block[2 * AT_H];
    float   scratch[2 * AT_TAU_MAX + 2 * AT_H];   /* linear copies for residual / mark search */
} at_psola_t;

void at_psola_init(at_psola_t *s, const at_config_t *cfg);

/* One hop of AT_H samples.
 *   audio_hop   : high-passed cleaned audio
 *   period      : pitch period in samples (last valid one when unvoiced)
 *   pitch_ratio : alpha = 2^(shift / 12)
 *   voiced      : voicing decision of this hop's frame
 *   refl        : LPC reflection coefficients of the newest audio frame
 *   active      : pause-gate activity of this hop
 *   out         : AT_H output samples = finished samples [received - 612 - 128, received - 612)
 */
void at_psola_push(at_psola_t *s, const float *audio_hop, float period, float pitch_ratio,
                   int voiced, const float *refl, int active, float *out);

#endif /* AT_PSOLA_H */
