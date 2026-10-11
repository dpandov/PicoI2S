// Reference stream for the filtered dual-line formats (every filter but
// NOS): the words the I2S output must carry, computed from the
// S/PDIF-decoded frames of the same window with the transmitter's own
// filter (i2s/i2s_filter.h), the same saturation and the format's
// truncation, so the comparison is bit-exact by construction. Portable C
// (host-testable); the filter runs in bounded chunks so the self-test task
// can interleave with the real filter on core 1.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "i2s/i2s_filter.h"
#include "selftest/selftest_i2s_decode.h"

// Output frames per selftest_i2s_expect_step() (8 input frames at 8x, 64
// at the sample rate): under 80 us for the sharp 8x cascade on the slowest
// clock.
#define SELFTEST_EXPECT_STEP_OUT 64u

typedef struct {
    i2s_filter_t      filter;
    i2s_filter_type_t type;
    uint32_t          rate;
    uint32_t          transient;    // output frames, see selftest_i2s_expect_transient()
    const int32_t    *pcm;          // interleaved L/R input frames
    uint32_t          n_in, pos;
    uint32_t         *l, *r;        // output words per channel
    uint32_t          max_out, n_out;
    uint32_t          mask;         // the format's top `width` bits
    // RAM copies of the cascade's stages and coefficients (core 1 keeps
    // its hot loops off flash).
    i2s_stage_t       stage[I2S_FILTER_MAX_STAGES];
    int32_t           coef[I2S_FILTER_MAX_STAGES][I2S_COEF_A_N + 1u];
    int32_t           coef1[I2S_FILTER_MAX_STAGES][I2S_COEF_M0_N];
} selftest_i2s_expect_t;

// Selects the filter of the running output (type, sample rate) and works
// out its transient. False for NOS (no reference: the pattern check
// applies).
bool selftest_i2s_expect_setup(selftest_i2s_expect_t *x, i2s_filter_type_t type, uint32_t rate);

// Output frames per input frame of the filter set up (0 after a failed
// setup).
static inline unsigned selftest_i2s_expect_osr(const selftest_i2s_expect_t *x) {
    return x->filter.osr;
}

// Leading reference frames that depend on samples before the window (the
// filter starts from zero history): every stage's reach back, carried
// through the later stages, plus one input frame of margin. Output frames
// of the filter set up; the reference is exact from this index on.
static inline uint32_t selftest_i2s_expect_transient(const selftest_i2s_expect_t *x) {
    return x->transient;
}

// Starts the computation over n_in input frames (32-bit left-justified, as
// decoded from the line) with the filter set up, history zeroed, into l /
// r (max_out words each); words are truncated to `width` bits.
void selftest_i2s_expect_begin(selftest_i2s_expect_t *x, const int32_t *pcm, uint32_t n_in,
                               unsigned width, uint32_t *l, uint32_t *r, uint32_t max_out);

// Filters up to out_frames output frames' worth of input (at least one
// input frame). True when complete.
bool selftest_i2s_expect_step(selftest_i2s_expect_t *x, uint32_t out_frames);

// The reference view for the decoder, once complete.
void selftest_i2s_expect_reference(const selftest_i2s_expect_t *x, selftest_i2s_reference_t *ref);
