// De-emphasis for sources flagged with 50/15 us pre-emphasis (IEC 60908):
// one biquad per rate fitted to the analogue curve (1 + s 15 us) /
// (1 + s 50 us), within 0.004 dB to 20 kHz at 44.1 / 48 kHz.
// Coefficients and design checks: tools/i2s_deemph_design.py,
// i2s_deemph_coef.h.
//
// Direct form I, Q30 coefficients, 64-bit accumulation, output rounded
// toward zero to 32 bits. The state is the 32-bit input and output, so the
// added error stays below 0.01 LSB of a 24-bit sample (rms). Samples are
// interleaved stereo, 32-bit left-justified in and out; in == out is
// allowed. The impulse response is non-negative and the DC gain one
// coefficient unit below 1, which leaves room for the rounding error:
// 24-bit input never reaches the clamp (the design tool checks it), which
// only guards wider input.
//
// The filter always runs, so its state is warm when the output switches;
// a block whose on / off differs from the previous one crossfades
// linearly across the block, ending on the new output.
//
// Portable integer code without SDK calls.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "i2s/i2s_deemph_coef.h"

typedef struct {
    int32_t x1, x2, y1, y2;
} i2s_deemph_state_t;

typedef struct {
    int32_t b0, b1, b2, na1, na2;   // Q30; na = -a
} i2s_deemph_coef_t;

typedef struct {
    i2s_deemph_coef_t c;
    bool valid;                 // rate supported
    bool on;                    // output of the last block
    i2s_deemph_state_t ch[2];
} i2s_deemph_t;

// Coefficients for the rate, state zeroed, starting with the output on or
// off. False for an unsupported rate: then the output is always the input.
static inline bool i2s_deemph_init(i2s_deemph_t *d, uint32_t rate, bool on) {
    *d = (i2s_deemph_t){.on = on};
    for (unsigned i = 0; i < I2S_DEEMPH_RATES; i++) {
        if (i2s_deemph_rate[i] == rate) {
            const int32_t *k = i2s_deemph_coef[i];
            d->c = (i2s_deemph_coef_t){k[0], k[1], k[2], k[3], k[4]};
            d->valid = true;
        }
    }
    return d->valid;
}

#define I2S_DEEMPH_INLINE static inline __attribute__((always_inline))

// acc / 2^I2S_DEEMPH_SHIFT rounded toward zero, clamped to 32 bits.
// Toward zero, the state decays to exactly 0 after the input does (the
// feedback coefficients sum to less than 1); rounding to nearest would
// leave a DC remainder of 1 or 2 LSBs. Branch-free on the M33.
I2S_DEEMPH_INLINE int32_t i2s_deemph_out(int64_t acc) {
    acc += (uint32_t)((int32_t)(acc >> 32) >> 31) >> (32 - I2S_DEEMPH_SHIFT);
    int32_t hi = (int32_t)(acc >> 32);
    if ((uint32_t)hi + (1u << (I2S_DEEMPH_SHIFT - 1)) >= (2u << (I2S_DEEMPH_SHIFT - 1)))
        return hi < 0 ? INT32_MIN : INT32_MAX;
    return (int32_t)(acc >> I2S_DEEMPH_SHIFT);
}

// One filter output.
I2S_DEEMPH_INLINE int32_t i2s_deemph_step(const i2s_deemph_coef_t *k, int32_t x, int32_t x1,
                                          int32_t x2, int32_t y1, int32_t y2) {
    int64_t acc = (int64_t)k->b0 * x;
    acc += (int64_t)k->b1 * x1;
    acc += (int64_t)k->b2 * x2;
    acc += (int64_t)k->na1 * y1;
    acc += (int64_t)k->na2 * y2;
    return i2s_deemph_out(acc);
}

// One channel (stride 2), the filtered output when on, else the input.
// Two frames per iteration, so the state rotates without register moves.
I2S_DEEMPH_INLINE void i2s_deemph_channel(const i2s_deemph_coef_t *c, i2s_deemph_state_t *s,
                                          const int32_t *in, int32_t *out, unsigned n, bool on) {
    const i2s_deemph_coef_t k = *c;
    int32_t x1 = s->x1, x2 = s->x2, y1 = s->y1, y2 = s->y2;
    const int32_t *end = in + 2u * (n & ~1u);
    while (in != end) {
        int32_t xa = in[0], xb = in[2];
        int32_t ya = i2s_deemph_step(&k, xa, x1, x2, y1, y2);
        int32_t yb = i2s_deemph_step(&k, xb, xa, x1, ya, y1);
        out[0] = on ? ya : xa;
        out[2] = on ? yb : xb;
        x2 = xa;
        x1 = xb;
        y2 = ya;
        y1 = yb;
        in += 4;
        out += 4;
    }
    if (n & 1u) {
        int32_t x = in[0];
        int32_t y = i2s_deemph_step(&k, x, x1, x2, y1, y2);
        out[0] = on ? y : x;
        x2 = x1;
        x1 = x;
        y2 = y1;
        y1 = y;
    }
    *s = (i2s_deemph_state_t){x1, x2, y1, y2};
}

// One channel, crossfading from the input to the filtered output (to_on)
// or back: weight (i + 1) / n at frame i, so the last frame is the new
// output.
I2S_DEEMPH_INLINE void i2s_deemph_fade(const i2s_deemph_coef_t *k, i2s_deemph_state_t *s,
                                       const int32_t *in, int32_t *out, unsigned n, bool to_on) {
    for (unsigned i = 0; i < n; i++) {
        int32_t x = in[2u * i];
        int32_t y = i2s_deemph_step(k, x, s->x1, s->x2, s->y1, s->y2);
        *s = (i2s_deemph_state_t){x, s->x1, y, s->y1};
        int32_t from = to_on ? x : y, to = to_on ? y : x;
        int64_t w = (int64_t)(((i + 1u) << 16) / n);
        out[2u * i] = (int32_t)(from + ((((int64_t)to - from) * w) >> 16));
    }
}

// Processes n_frames interleaved stereo frames: filtered when on, the input
// when off, a crossfade when on differs from the previous call.
I2S_DEEMPH_INLINE void i2s_deemph_run(i2s_deemph_t *d, const int32_t *in, int32_t *out,
                                      unsigned n_frames, bool on) {
    if (n_frames == 0)
        return;
    if (!d->valid) {
        if (in != out)
            for (unsigned k = 0; k < 2u * n_frames; k++)
                out[k] = in[k];
        return;
    }
    for (unsigned ch = 0; ch < 2u; ch++) {
        if (on != d->on)
            i2s_deemph_fade(&d->c, &d->ch[ch], in + ch, out + ch, n_frames, on);
        else
            i2s_deemph_channel(&d->c, &d->ch[ch], in + ch, out + ch, n_frames, on);
    }
    d->on = on;
}
