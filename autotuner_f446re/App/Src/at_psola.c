/*
 * at_psola.c — see at_psola.h. Each static function below is one method of the Python TDPSOLA class.
 *
 * Differences from Python that do not change the result:
 *   - whole-signal arrays are rings of AT_RING samples (at_ring.h)
 *   - grains that would land on already-emitted output are skipped (Python writes them into a part
 *     of the array that is never read again)
 *   - the next synthesis mark is kept as integer + fraction so float32 stays exact for long runs
 */
#include "at_psola.h"
#include "at_ring.h"
#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979323846f

static inline int32_t round_i(float x) { return (int32_t)lrintf(x); }   /* half-to-even, like Python */

static inline int32_t mark_at(const at_psola_t *s, int32_t idx) { return s->marks[idx & (AT_MARK_RING - 1)]; }

static void push_mark(at_psola_t *s, int32_t pos)
{

    s->marks[s->mark_count & (AT_MARK_RING - 1)] = pos;
    s->mark_count++;
}

void at_psola_init(at_psola_t *s, const at_config_t *cfg)
{
    memset(s, 0, sizeof(*s));
    s->residual_stretch = cfg->residual_stretch;
    s->mark_tol = cfg->mark_tol;
    s->lpc_block = cfg->lpc_block > 0 ? cfg->lpc_block : 32;
    s->dry_unvoiced = cfg->dry_unvoiced;
    s->dry_after_hops = cfg->dry_after_hops;
    s->use_gate = cfg->gate;
    s->use_level = cfg->level_match;
    at_allpole_init(&s->synth);
    at_xfade_init(&s->xfade, cfg);
    at_level_init(&s->level, cfg);
    at_gate_init(&s->gate, cfg);
}

/* ---------------------------------------------------------------- residual e = A(z) x */
static void update_residual(at_psola_t *s, const float *refl)
{
    memcpy(s->refl[s->refl_count & (AT_REFL_RING - 1)], refl, AT_LPC_ORDER * sizeof(float));
    s->refl_count++;

    int32_t centre = s->samples_received - AT_N / 2;
    int32_t start = centre - AT_H / 2, stop = centre + AT_H / 2;
    if (start < 0) start = 0;
    if (stop <= start) return;

    float a[AT_LPC_ORDER + 1];
    at_step_up(refl, AT_LPC_ORDER, a);
    float *x = s->scratch;                               /* audio[start - 16 .. stop), linear */
    int count = (int)(stop - start) + AT_LPC_ORDER;
    for (int j = 0; j < count; j++) x[j] = at_ring_getf(s->audio, start - AT_LPC_ORDER + j);
    for (int32_t n = start; n < stop; n++) {
        const float *xn = &x[n - start + AT_LPC_ORDER];  /* xn[-i] = audio[n - i] */
        float acc = 0.0f;
        for (int i = 0; i <= AT_LPC_ORDER; i++) acc += a[i] * xn[-i];
        s->residual[n & AT_RING_MASK] = acc;
    }
    s->residual_end = stop;
}

/* ---------------------------------------------------------------- analysis pitch marks */
/* Best candidate c in [first, first + count) for the next mark: normalised cross-correlation of the
 * residual window around c (2*half+1 samples, demeaned) with the demeaned reference window.
 * buf holds the residual from first - half to first + count - 1 + half (linear copy). */
static int32_t best_candidate(const float *buf, int32_t first, int count, int half, const float *ref, float ref_norm)
{
    int len = 2 * half + 1;
    float inv_len = 1.0f / (float)len;
    int32_t best = first;
    float best_score = -1e30f;
    for (int ci = 0; ci < count; ci++) {
        const float *w = &buf[ci];
        float sum = 0.0f;
        for (int j = 0; j < len; j++) sum += w[j];
        float mean = sum * inv_len;
        float dot = 0.0f, energy = 0.0f;
        for (int j = 0; j < len; j++) {
            float v = w[j] - mean;
            dot += v * ref[j];
            energy += v * v;
        }
        float score = dot / (sqrtf(energy) * ref_norm + 1e-12f);
        if (score > best_score) { best_score = score; best = first + ci; }
    }
    return best;
}

static void extend_marks(at_psola_t *s, float period)
{
    int32_t available = s->residual_end;
    if (s->mark_count == 0) {                          /* first mark: largest |e| in the first period */
        int32_t first_period = round_i(period);
        if (available < first_period + 1) return;
        int32_t best = 0;
        float best_v = -1.0f;
        for (int32_t n = 0; n < first_period; n++) {
            float v = fabsf(at_ring_getf(s->residual, n));
            if (v > best_v) { best_v = v; best = n; }
        }
        push_mark(s, best);
        s->first_mark = best;
    }

    int half = round_i(0.5f * period);
    if (half < 4) half = 4;
    int tol = round_i(s->mark_tol * period);
    if (tol < 2) tol = 2;
    int32_t step = round_i(period);
    if (step < 1) step = 1;
    float *ref = s->grain;                              /* scratch, 2*half+1 <= 2*tau_max+1 */

    for (;;) {
        int32_t last = mark_at(s, s->mark_count - 1);
        int32_t predicted = last + step;
        if (predicted + tol + half >= available) break;
        if (last - half < 0) { push_mark(s, predicted); continue; }

        int len = 2 * half + 1;
        float mean = 0.0f;
        for (int j = 0; j < len; j++) mean += at_ring_getf(s->residual, last - half + j);
        mean /= (float)len;
        float norm = 0.0f;
        for (int j = 0; j < len; j++) {
            ref[j] = at_ring_getf(s->residual, last - half + j) - mean;
            norm += ref[j] * ref[j];
        }
        norm = sqrtf(norm) + 1e-12f;

        int32_t first = predicted - tol;
        int count = 2 * tol + 1;
        float *buf = s->scratch;                        /* residual[first - half .. first + count - 1 + half] */
        for (int j = 0; j < count + 2 * half; j++) buf[j] = at_ring_getf(s->residual, first - half + j);
        int32_t best = best_candidate(buf, first, count, half, ref, norm);
        push_mark(s, best);
    }
}

/* ---------------------------------------------------------------- one grain */
static void ola(at_psola_t *s, int32_t analysis_mark, int L, int32_t centre, float pitch_ratio,
                float synthesis_period)
{
    /* Hann-windowed grain, half-width L, from the residual. w[j] = 0.5 (1 - cos(pi j / L)) */
    float c = 1.0f, sn = 0.0f;
    const float cd = cosf(PI_F / (float)L), sd = sinf(PI_F / (float)L);
    for (int j = 0; j <= 2 * L; j++) {
        s->grain[j] = at_ring_getf(s->residual, analysis_mark - L + j) * (0.5f - 0.5f * c);
        float c2 = c * cd - sn * sd;                    /* rotate: cos/sin(pi (j+1) / L) */
        sn = sn * cd + c * sd;
        c = c2;
    }

    /* lowering: stretch the excitation grain by (1/alpha)^residual_stretch (linear interpolation) */
    float stretch = 1.0f;
    if (pitch_ratio < 1.0f && s->residual_stretch > 0.0f) {
        stretch = powf(1.0f / pitch_ratio, s->residual_stretch);
        float cap = (float)AT_GRAIN_REACH / (float)L;   /* keep the grain inside the lookahead */
        if (stretch > cap) stretch = cap;
    }
    int Ls = round_i(stretch * (float)L);
    float gain = synthesis_period / (float)Ls;

    int32_t out_start = centre - Ls;
    int32_t lowest = s->samples_emitted > 0 ? s->samples_emitted : 0;
    for (int j = 0; j <= 2 * Ls; j++) {
        int32_t n = out_start + j;
        if (n < lowest) continue;
        float v;
        if (Ls == L) {
            v = s->grain[j];
        } else {
            float pos = (float)(j - Ls) * ((float)L / (float)Ls) + (float)L;
            if (pos <= 0.0f) v = s->grain[0];
            else {
                int i0 = (int)pos;
                if (i0 >= 2 * L) v = s->grain[2 * L];
                else {
                    float f = pos - (float)i0;
                    v = s->grain[i0] + f * (s->grain[i0 + 1] - s->grain[i0]);
                }
            }
        }
        s->ola[n & AT_RING_MASK] += gain * v;
    }
}

/* ---------------------------------------------------------------- synthesis marks */
static void place(at_psola_t *s, float period, float pitch_ratio)
{
    int32_t available = s->residual_end;
    int32_t finalized = s->samples_received - AT_PSOLA_DELAY;
    if (finalized <= 0 || s->mark_count == 0) return;
    float synthesis_period = period / pitch_ratio;
    int L = round_i(period);
    if (!s->has_next) {
        s->has_next = 1;
        s->syn_int = s->first_mark;
        s->syn_frac = 0.0f;
    }
    int32_t oldest = s->mark_count - AT_MARK_RING;
    if (oldest < 0) oldest = 0;

    while (s->syn_int < finalized + AT_GRAIN_REACH) {
        int32_t mi = s->nearest_mark;
        if (mi < oldest) mi = oldest;
        while (mi > oldest && mark_at(s, mi) + L >= available) mi--;
        while (mi + 1 < s->mark_count && mark_at(s, mi + 1) + L < available) {
            float d_next = (float)(mark_at(s, mi + 1) - s->syn_int) - s->syn_frac;
            float d_cur = (float)(mark_at(s, mi) - s->syn_int) - s->syn_frac;
            if (fabsf(d_next) <= fabsf(d_cur)) mi++;
            else break;
        }
        s->nearest_mark = mi;
        int32_t am = mark_at(s, mi);
        if (am + L >= available) break;                 /* grain not fully available yet */

        /* round(syn_int + syn_frac), half to even */
        int32_t centre = s->syn_int;
        if (s->syn_frac > 0.5f || (s->syn_frac == 0.5f && (s->syn_int & 1))) centre++;
        ola(s, am, L, centre, pitch_ratio, synthesis_period);

        s->syn_frac += synthesis_period;
        float whole = floorf(s->syn_frac);
        s->syn_int += (int32_t)whole;
        s->syn_frac -= whole;
    }
}

/* ---------------------------------------------------------------- 1/A(z) synthesis */
/* Blocks of lpc_block samples; each block uses reflection coefficients interpolated between the two
 * nearest analysis frame centres (hop i's frame centre is at (i+1) H - N/2). */
static void synthesize(at_psola_t *s, float *x, int32_t start, int len)
{
    float k[AT_LPC_ORDER], a[AT_LPC_ORDER + 1];
    int32_t count = s->refl_count;
    for (int pos = 0; pos < len; pos += s->lpc_block) {
        int length = len - pos < s->lpc_block ? len - pos : s->lpc_block;
        /* hop_index = (middle + N/2) / H - 1 with middle = start + pos + length/2, done in integers */
        int32_t twice = 2 * (start + pos) + length + AT_N - 2 * AT_H;   /* 2 H hop_index */
        int32_t lower = twice / (2 * AT_H);
        float frac = (float)(twice % (2 * AT_H)) / (float)(2 * AT_H);
        if (lower > count - 1) { lower = count - 1; frac = 1.0f; }
        if (lower < 0) { lower = 0; frac = 0.0f; }
        int32_t upper = lower + 1 < count ? lower + 1 : count - 1;
        const float *kl = s->refl[lower & (AT_REFL_RING - 1)];
        const float *ku = s->refl[upper & (AT_REFL_RING - 1)];
        for (int i = 0; i < AT_LPC_ORDER; i++) k[i] = (1.0f - frac) * kl[i] + frac * ku[i];
        at_step_up(k, AT_LPC_ORDER, a);
        at_allpole_run(&s->synth, a, x + pos, x + pos, length);
    }
}

/* ---------------------------------------------------------------- finalise a block */
static void emit(at_psola_t *s)
{
    int32_t finalized = s->samples_received - AT_PSOLA_DELAY;
    if (finalized <= s->samples_emitted) return;
    int32_t start = s->samples_emitted;
    int len = (int)(finalized - start);
    if (len > 2 * AT_H) len = 2 * AT_H;                 /* never happens: one hop per call */
    s->samples_emitted = start + len;

    float *blk = s->block;
    for (int i = 0; i < len; i++) {
        float *slot = &s->ola[(start + i) & AT_RING_MASK];
        blk[i] = *slot;
        *slot = 0.0f;                                   /* free the slot for reuse */
    }
    synthesize(s, blk, start, len);
    if (s->dry_unvoiced) at_xfade_run(&s->xfade, blk, start, len, s->audio, s->use_dry);
    if (s->use_level)    at_level_run(&s->level, blk, start, len, s->audio);
    if (s->use_gate)     at_gate_run(&s->gate, blk, start, len, s->active, s->samples_received);
    for (int i = 0; i < len; i++) s->out[(start + i) & AT_RING_MASK] = blk[i];
}

/* ---------------------------------------------------------------- one hop */
void at_psola_push(at_psola_t *s, const float *audio_hop, float period, float pitch_ratio,
                   int voiced, const float *refl, int active, float *out)
{
    for (int i = 0; i < AT_H; i++) {
        int32_t n = s->samples_received + i;
        s->audio[n & AT_RING_MASK] = audio_hop[i];
        s->active[n & AT_RING_MASK] = (uint8_t)(active ? 1 : 0);
    }
    s->samples_received += AT_H;
    update_residual(s, refl);

    /* the voicing decision is about the frame centre: mark that hop-wide region */
    s->unvoiced_hops = voiced ? 0 : s->unvoiced_hops + 1;
    int32_t centre = s->samples_received - AT_N / 2;
    int32_t rs = centre - AT_H / 2, re = centre + AT_H / 2;
    if (rs < 0) rs = 0;
    if (re > 0) {
        uint8_t dry = (uint8_t)(s->dry_unvoiced && s->unvoiced_hops >= s->dry_after_hops);
        for (int32_t n = rs; n < re; n++) s->use_dry[n & AT_RING_MASK] = dry;
    }

    extend_marks(s, period);
    place(s, period, pitch_ratio);
    emit(s);

    int32_t first = s->samples_received - AT_PSOLA_DELAY - AT_H;
    for (int i = 0; i < AT_H; i++) out[i] = at_ring_getf(s->out, first + i);
}
