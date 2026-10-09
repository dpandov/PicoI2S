// Fine rate trim of the S/PDIF PIO clock divider. Pure integer code, no SDK
// dependencies, so host simulations use the same arithmetic.
//
// The SM divider is INT + FRAC/256: one FRAC step moves the rate by
// 1/(256 x INT), 163 ppm at INT = 24. The target divider carries 16 more
// fraction bits (Q8.24, so INT < 256), and a first-order sigma-delta in time
// switches the register between the two FRAC values around it so that the
// time-weighted mean divider equals the target. The phase error it leaves
// is a sawtooth of about two FRAC steps times the gap between calls:
// 2 x gap x clk_sys / (256 x INT) clk_sys cycles peak to peak. From a
// 25 us timer interrupt (spdif_tx.c) that is about 5 cycles (31..38 ns)
// at INT = 6, under one cycle at INT = 24; from the main loop (260 us) it
// was ten times that.
#pragma once

#include <stdbool.h>
#include <stdint.h>

// Positive: faster output.
#define SPDIF_TRIM_MAX_PPB 2000000
// Longer gaps between calls are counted as this long.
#define SPDIF_TRIM_MAX_GAP_US 10000u

#define SPDIF_TRIM_STEP (1u << 16)   // one FRAC step in Q8.24

typedef struct {
    uint32_t target;    // divider, Q8.24
    uint32_t applied;   // divider in the SM, Q8.24, a whole FRAC step
    int64_t  err;       // integral of (applied - target), Q8.24 x us
    uint32_t last_us;
} spdif_trim_t;

static inline int32_t spdif_trim_clamp(int32_t ppb) {
    if (ppb > SPDIF_TRIM_MAX_PPB)
        return SPDIF_TRIM_MAX_PPB;
    if (ppb < -SPDIF_TRIM_MAX_PPB)
        return -SPDIF_TRIM_MAX_PPB;
    return ppb;
}

// div_int / (1 + ppb / 1e9) in Q8.24, rounded. div_int must be 1..255.
static inline uint32_t spdif_trim_divider(uint32_t div_int, int32_t ppb) {
    uint64_t num = ((uint64_t)div_int << 24) * 1000000000u;
    uint64_t den = (uint64_t)(1000000000 + (int64_t)spdif_trim_clamp(ppb));
    return (uint32_t)((num + den / 2u) / den);
}

static inline void spdif_trim_reset(spdif_trim_t *t, uint32_t div_int, uint32_t now_us) {
    t->target = div_int << 24;
    t->applied = t->target;
    t->err = 0;
    t->last_us = now_us;
}

static inline void spdif_trim_account(spdif_trim_t *t, uint32_t now_us) {
    uint32_t dt = now_us - t->last_us;
    t->last_us = now_us;
    if (dt > SPDIF_TRIM_MAX_GAP_US)
        dt = SPDIF_TRIM_MAX_GAP_US;
    t->err += (int64_t)(int32_t)(t->applied - t->target) * dt;
}

// The time until now still counts against the old target.
static inline void spdif_trim_set_target(spdif_trim_t *t, uint32_t target, uint32_t now_us) {
    spdif_trim_account(t, now_us);
    t->target = target;
}

// Accounts for the time since the last call and returns the divider (Q8.24)
// to apply from now on.
static inline uint32_t spdif_trim_dither(spdif_trim_t *t, uint32_t now_us) {
    spdif_trim_account(t, now_us);
    uint32_t lo = t->target & ~(SPDIF_TRIM_STEP - 1u);
    // Above the target so far (running slow): take the lower divider.
    bool take_lo = (t->target & (SPDIF_TRIM_STEP - 1u)) == 0 || t->err > 0;
    t->applied = take_lo ? lo : lo + SPDIF_TRIM_STEP;
    return t->applied;
}

// CLKDIV register value (INT in 31:16, FRAC in 15:8) for a Q8.24 divider.
static inline uint32_t spdif_trim_clkdiv_reg(uint32_t div_q24) {
    return (div_q24 >> 8) & 0xFFFFFF00u;
}
