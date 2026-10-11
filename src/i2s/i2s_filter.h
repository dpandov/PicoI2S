// Interpolation filters for the dual-line output (i2s_filter_type_t in
// i2s_tx.h), 8x up to 48 kHz and 4x above, plus the sin(x)/x compensation
// at the sample rate. Coefficients and their design checks:
// tools/i2s_filter_design.py, i2s_filter_coef.h.
//
//   sharp      linear phase, flat to 0.4535 Fs (ripple < 0.0002 dB), images
//              at least 100 dB down (stopband from 0.5465 Fs at 8x, from
//              0.65 Fs at 4x, where it is flat to 0.35 Fs)
//   slow       linear phase, flat to 0.25 Fs, -2.8 dB at 0.4535 Fs, -3 dB
//              near 0.46 Fs, 100 dB from 0.75 Fs (leaks above Fs / 2 by
//              design)
//   min-phase  the slow magnitude with a minimum-phase first stage: no
//              pre-ringing, about 8.6 input samples of delay at 8x instead
//              of 12.9 (sharp: 36)
//   sinc       at the sample rate, flat at -4 dB to 0.4535 Fs together
//              with the DAC's zero-order hold
//
// Portable integer code without SDK calls (the self-test recomputes the
// expected output with it). Samples are interleaved stereo, 32-bit
// left-justified, scaled by 1/16 on entry: the cascades' worst-case gain
// stays below 8, so no stage output and no pair sum overflows 32 bits, and
// the 8 zero low bits of a 24-bit sample make the scaling lossless. Q30
// coefficients, 64-bit accumulation, arithmetic shift. Each stage keeps
// the last span - 1 input frames, so the output does not depend on how the
// input is chunked.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "i2s/i2s_filter_coef.h"
#include "i2s/i2s_tx.h"

#define I2S_FILTER_SCALE_SHIFT 4
#define I2S_FILTER_MAX_IN      96u     // input frames per call
#define I2S_FILTER_MAX_STAGES  3u
#define I2S_FILTER_MAX_OSR     8u

typedef enum {
    I2S_STAGE_HB,       // half-band x2: c = 2h at the odd offsets 1, 3, ... from the centre
    I2S_STAGE_POLY,     // FIR x2: c = branch 0, c1 = branch 1 (2h, oldest sample first)
    I2S_STAGE_SYM,      // linear-phase FIR x1, length 2m + 1: c = m outer pairs, then the centre
} i2s_stage_kind_t;

typedef struct {
    uint8_t kind;       // i2s_stage_kind_t
    uint8_t n;          // HB: coefficients; POLY: branch 0 taps; SYM: pairs
    uint8_t n1;         // POLY: branch 1 taps
    const int32_t *c;
    const int32_t *c1;
} i2s_stage_t;

static const i2s_stage_t i2s_stage_a = {I2S_STAGE_HB, I2S_COEF_A_N, 0, i2s_coef_a, 0};
static const i2s_stage_t i2s_stage_b = {I2S_STAGE_HB, I2S_COEF_B_N, 0, i2s_coef_b, 0};
static const i2s_stage_t i2s_stage_c = {I2S_STAGE_HB, I2S_COEF_C_N, 0, i2s_coef_c, 0};
static const i2s_stage_t i2s_stage_h = {I2S_STAGE_HB, I2S_COEF_H_N, 0, i2s_coef_h, 0};
static const i2s_stage_t i2s_stage_s = {I2S_STAGE_HB, I2S_COEF_S_N, 0, i2s_coef_s, 0};
static const i2s_stage_t i2s_stage_bs = {I2S_STAGE_HB, I2S_COEF_BS_N, 0, i2s_coef_bs, 0};
static const i2s_stage_t i2s_stage_cs = {I2S_STAGE_HB, I2S_COEF_CS_N, 0, i2s_coef_cs, 0};
static const i2s_stage_t i2s_stage_m = {I2S_STAGE_POLY, I2S_COEF_M0_N, I2S_COEF_M1_N, i2s_coef_m0,
                                        i2s_coef_m1};
static const i2s_stage_t i2s_stage_k = {I2S_STAGE_SYM, I2S_COEF_K_N - 1u, 0, i2s_coef_k, 0};

_Static_assert(I2S_COEF_M0_N >= I2S_COEF_M1_N, "POLY: branch 0 is the longer one");

// Input frames one output depends on.
static inline unsigned i2s_stage_span(const i2s_stage_t *s) {
    switch (s->kind) {
    case I2S_STAGE_HB:
        return 2u * s->n;
    case I2S_STAGE_POLY:
        return s->n;
    default:
        return 2u * s->n + 1u;
    }
}

static inline unsigned i2s_stage_factor(const i2s_stage_t *s) {
    return s->kind == I2S_STAGE_SYM ? 1u : 2u;
}

// History frames of the longest stage (A).
#define I2S_FILTER_MAX_HIST (2u * I2S_COEF_A_N)

typedef struct {
    const i2s_stage_t *stage;
    unsigned hist;              // span - 1 frames kept
    int32_t *buf;               // stereo: hist frames of history, then the input
} i2s_filter_stage_state_t;

typedef struct {
    unsigned osr;               // output frames per input frame
    unsigned n_stages;
    i2s_filter_stage_state_t st[I2S_FILTER_MAX_STAGES];
    int32_t mem[2u * (3u * I2S_FILTER_MAX_HIST + (1u + 2u + 4u) * I2S_FILTER_MAX_IN)];
} i2s_filter_t;

// Sets up the cascade for the filter at the rate, history zeroed. False
// for NOS (nothing to compute).
static inline bool i2s_filter_init(i2s_filter_t *f, i2s_filter_type_t type, uint32_t rate) {
    const i2s_stage_t *s[I2S_FILTER_MAX_STAGES] = {0};
    bool high = i2s_filter_osr(type, rate) == 4u;
    unsigned n = 0;
    switch (type) {
    case I2S_FILTER_SHARP:
        if (high)
            s[0] = &i2s_stage_h, s[1] = &i2s_stage_b, n = 2;
        else
            s[0] = &i2s_stage_a, s[1] = &i2s_stage_b, s[2] = &i2s_stage_c, n = 3;
        break;
    case I2S_FILTER_SLOW:
    case I2S_FILTER_MINPHASE:
        s[0] = type == I2S_FILTER_SLOW ? &i2s_stage_s : &i2s_stage_m;
        if (high)
            s[1] = &i2s_stage_b, n = 2;
        else
            s[1] = &i2s_stage_bs, s[2] = &i2s_stage_cs, n = 3;
        break;
    case I2S_FILTER_NOS_SINC:
        s[0] = &i2s_stage_k, n = 1;
        break;
    default:
        return false;
    }
    f->n_stages = n;
    f->osr = 1;
    int32_t *p = f->mem;
    for (unsigned i = 0; i < n; i++) {
        unsigned in = I2S_FILTER_MAX_IN * f->osr;
        unsigned hist = i2s_stage_span(s[i]) - 1u;
        f->st[i].stage = s[i];
        f->st[i].hist = hist;
        f->st[i].buf = p;
        for (unsigned k = 0; k < 2u * (hist + in); k++)
            p[k] = 0;
        p += 2u * (hist + in);
        f->osr *= i2s_stage_factor(s[i]);
    }
    return true;
}

#define I2S_FILTER_INLINE static inline __attribute__((always_inline))

typedef struct {
    int32_t l, r;
} i2s_frame_t;

// The two kernels below run as hand-scheduled loops on the Cortex-M33
// (GCC spills registers in them): paired L / R loads, two taps per
// iteration, about 11 cycles per stereo tap for fold and 7 for dot, bound
// by the loads. The C loops are the reference (same arithmetic: 32-bit
// pair sum, 64-bit multiply-accumulate) and run on other targets.
#if defined(__ARM_ARCH_8M_MAIN__) && defined(__ARM_FEATURE_DSP)
#define I2S_FILTER_ASM 1
#else
#define I2S_FILTER_ASM 0
#endif

static inline int64_t i2s_join64(uint32_t lo, uint32_t hi) {
    return (int64_t)(((uint64_t)hi << 32) | lo);
}

// Folded pairs: acc += sum c[k] (a[-k] + b[k]), both channels.
I2S_FILTER_INLINE void i2s_fold(const int32_t *c, unsigned n, const i2s_frame_t *a,
                                const i2s_frame_t *b, int64_t *l, int64_t *r) {
#if I2S_FILTER_ASM
    uint32_t llo = (uint32_t)*l, lhi = (uint32_t)((uint64_t)*l >> 32);
    uint32_t rlo = (uint32_t)*r, rhi = (uint32_t)((uint64_t)*r >> 32);
    unsigned pairs = n >> 1;
    int32_t t0, t1, t2, t3, t4;
    if (pairs) {
        __asm__ volatile(
            "1:\n\t"
            "ldrd  %[t0], %[t1], [%[a]], #-8\n\t"
            "ldrd  %[t2], %[t3], [%[b]], #8\n\t"
            "ldr   %[t4], [%[c]], #4\n\t"
            "add   %[t0], %[t0], %[t2]\n\t"
            "add   %[t1], %[t1], %[t3]\n\t"
            "smlal %[llo], %[lhi], %[t4], %[t0]\n\t"
            "smlal %[rlo], %[rhi], %[t4], %[t1]\n\t"
            "ldrd  %[t0], %[t1], [%[a]], #-8\n\t"
            "ldrd  %[t2], %[t3], [%[b]], #8\n\t"
            "ldr   %[t4], [%[c]], #4\n\t"
            "add   %[t0], %[t0], %[t2]\n\t"
            "add   %[t1], %[t1], %[t3]\n\t"
            "smlal %[llo], %[lhi], %[t4], %[t0]\n\t"
            "smlal %[rlo], %[rhi], %[t4], %[t1]\n\t"
            "subs  %[k], %[k], #1\n\t"
            "bne   1b\n\t"
            : [a] "+r"(a), [b] "+r"(b), [c] "+r"(c), [k] "+r"(pairs), [llo] "+r"(llo),
              [lhi] "+r"(lhi), [rlo] "+r"(rlo), [rhi] "+r"(rhi), [t0] "=&r"(t0), [t1] "=&r"(t1),
              [t2] "=&r"(t2), [t3] "=&r"(t3), [t4] "=&r"(t4)
            :
            : "cc", "memory");
    }
    int64_t al = i2s_join64(llo, lhi), ar = i2s_join64(rlo, rhi);
    if (n & 1u) {
        int64_t c0 = c[0];
        al += c0 * (a[0].l + b[0].l);
        ar += c0 * (a[0].r + b[0].r);
    }
    *l = al;
    *r = ar;
#else
    int64_t al = *l, ar = *r;
    for (unsigned k = 0; k < n; k++) {
        int64_t ck = c[k];
        al += ck * (a[-(int)k].l + b[k].l);
        ar += ck * (a[-(int)k].r + b[k].r);
    }
    *l = al;
    *r = ar;
#endif
}

// Dot product: acc += sum c[k] w[k], both channels.
I2S_FILTER_INLINE void i2s_dot(const int32_t *c, unsigned n, const i2s_frame_t *w, int64_t *l,
                               int64_t *r) {
#if I2S_FILTER_ASM
    uint32_t llo = (uint32_t)*l, lhi = (uint32_t)((uint64_t)*l >> 32);
    uint32_t rlo = (uint32_t)*r, rhi = (uint32_t)((uint64_t)*r >> 32);
    unsigned pairs = n >> 1;
    int32_t t0, t1, t2, t3, t4;
    if (pairs) {
        __asm__ volatile(
            "1:\n\t"
            "ldrd  %[t0], %[t1], [%[w]], #8\n\t"
            "ldr   %[t4], [%[c]], #4\n\t"
            "ldrd  %[t2], %[t3], [%[w]], #8\n\t"
            "smlal %[llo], %[lhi], %[t4], %[t0]\n\t"
            "smlal %[rlo], %[rhi], %[t4], %[t1]\n\t"
            "ldr   %[t4], [%[c]], #4\n\t"
            "smlal %[llo], %[lhi], %[t4], %[t2]\n\t"
            "smlal %[rlo], %[rhi], %[t4], %[t3]\n\t"
            "subs  %[k], %[k], #1\n\t"
            "bne   1b\n\t"
            : [w] "+r"(w), [c] "+r"(c), [k] "+r"(pairs), [llo] "+r"(llo), [lhi] "+r"(lhi),
              [rlo] "+r"(rlo), [rhi] "+r"(rhi), [t0] "=&r"(t0), [t1] "=&r"(t1), [t2] "=&r"(t2),
              [t3] "=&r"(t3), [t4] "=&r"(t4)
            :
            : "cc", "memory");
    }
    int64_t al = i2s_join64(llo, lhi), ar = i2s_join64(rlo, rhi);
    if (n & 1u) {
        int64_t c0 = c[0];
        al += c0 * w[0].l;
        ar += c0 * w[0].r;
    }
    *l = al;
    *r = ar;
#else
    int64_t al = *l, ar = *r;
    for (unsigned k = 0; k < n; k++) {
        al += (int64_t)c[k] * w[k].l;
        ar += (int64_t)c[k] * w[k].r;
    }
    *l = al;
    *r = ar;
#endif
}

static inline i2s_frame_t i2s_frame_q30(int64_t l, int64_t r) {
    return (i2s_frame_t){(int32_t)(l >> I2S_COEF_SHIFT), (int32_t)(r >> I2S_COEF_SHIFT)};
}

// One stage over n_in stereo frames already in place after the history
// (st->buf + 2 x hist); writes factor x n_in stereo frames to out, then
// keeps the last hist input frames as the next history. w[t] is the window
// of span input frames ending at the newest one.
I2S_FILTER_INLINE void i2s_stage_run(i2s_filter_stage_state_t *st, unsigned n_in, int32_t *out) {
    const i2s_stage_t *s = st->stage;
    const int32_t *c = s->c;
    unsigned n = s->n;
    const i2s_frame_t *x = (const i2s_frame_t *)st->buf;
    i2s_frame_t *o = (i2s_frame_t *)out;
    if (s->kind == I2S_STAGE_HB) {
        // Even output: w[n - 1]; odd: sum c[k] (w[n-1-k] + w[n+k]).
        for (unsigned i = 0; i < n_in; i++) {
            const i2s_frame_t *w = &x[i];
            int64_t l = 0, r = 0;
            i2s_fold(c, n, &w[n - 1u], &w[n], &l, &r);
            o[2u * i] = w[n - 1u];
            o[2u * i + 1u] = i2s_frame_q30(l, r);
        }
    } else if (s->kind == I2S_STAGE_POLY) {
        // Branch 0 over w[0 .. n-1], branch 1 over w[n-n1 .. n-1].
        const int32_t *c1 = s->c1;
        unsigned n1 = s->n1;
        for (unsigned i = 0; i < n_in; i++) {
            const i2s_frame_t *w = &x[i];
            int64_t l0 = 0, r0 = 0, l1 = 0, r1 = 0;
            i2s_dot(c, n, w, &l0, &r0);
            i2s_dot(c1, n1, &w[n - n1], &l1, &r1);
            o[2u * i] = i2s_frame_q30(l0, r0);
            o[2u * i + 1u] = i2s_frame_q30(l1, r1);
        }
    } else {
        // sum c[j] (w[2n-j] + w[j]) + c[n] w[n].
        for (unsigned i = 0; i < n_in; i++) {
            const i2s_frame_t *w = &x[i];
            int64_t l = (int64_t)c[n] * w[n].l, r = (int64_t)c[n] * w[n].r;
            i2s_fold(c, n, &w[2u * n], &w[0], &l, &r);
            o[i] = i2s_frame_q30(l, r);
        }
    }
    int32_t *h = st->buf;
    for (unsigned k = 0; k < 2u * st->hist; k++)
        h[k] = h[2u * n_in + k];
}

// Filters n_in <= I2S_FILTER_MAX_IN interleaved stereo frames (32-bit
// left-justified) into n_in x osr stereo frames at 1/16 scale (see
// i2s_filter_output()).
I2S_FILTER_INLINE void i2s_filter_run(i2s_filter_t *f, const int32_t *pcm, unsigned n_in,
                                      int32_t *out) {
    int32_t *in = f->st[0].buf + 2u * f->st[0].hist;
    for (unsigned k = 0; k < 2u * n_in; k++)
        in[k] = pcm[k] >> I2S_FILTER_SCALE_SHIFT;
    unsigned n = n_in;
    for (unsigned i = 0; i < f->n_stages; i++) {
        i2s_filter_stage_state_t *st = &f->st[i];
        int32_t *dst = i + 1u < f->n_stages ? f->st[i + 1u].buf + 2u * f->st[i + 1u].hist : out;
        i2s_stage_run(st, n, dst);
        n *= i2s_stage_factor(st->stage);
    }
}

// Back to full scale with saturation: a 32-bit left-justified sample.
I2S_FILTER_INLINE int32_t i2s_filter_output(int32_t y) {
    const int32_t lim = (int32_t)1 << (31 - I2S_FILTER_SCALE_SHIFT);
    if (y >= lim)
        return INT32_MAX;
    if (y < -lim)
        return INT32_MIN;
    return (int32_t)((uint32_t)y << I2S_FILTER_SCALE_SHIFT);
}
