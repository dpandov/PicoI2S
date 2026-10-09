// Fine rate trim of the PIO clock dividers (S/PDIF and its companion).
// Pure integer code, no SDK dependencies, so host simulations use the same
// arithmetic.
//
// The SM divider is INT + FRAC/256: one FRAC step moves the rate by
// 1/(256 x INT), 651 ppm at INT = 6. The target divider carries 16 more
// fraction bits (Q8.24, so INT < 256), between the register values lo and
// hi = lo + STEP. A pacer state machine (spdif_pacer.pio, spdif_tx.c)
// writes the CLKDIV register twice per period of T clk_sys cycles, hi for
// t_hi cycles at the end of the period and lo for the rest, from a ring of
// M periods that DMA feeds it endlessly; spdif_trim_pattern() fills the
// ring. The share of hi over the ring is set for the mean rate 1/target (a
// time-weighted mean divider of target would run slow by f (1 - f) /
// (256 x INT)^2, 0.1 ppm at INT = 6), rounded to a whole cycle over the
// ring and spread across the periods by a first-order sigma-delta: the
// residual is half a cycle in T x M at most (0.0014 ppm at INT = 6, T =
// 3686), 2.5 cycles within 2.5 cycles of a whole step, where the pattern
// rounds to the step or to one minimal pulse.
//
// Phase error: a triangle at 1/T of f (1 - f) x T / (256 x INT) clk_sys
// cycles peak to peak, f the hi share, at most T / (1024 x INT): 0.6 cycles
// (4 ns) at INT = 6 with T = 25 us, whatever clk_sys. The sigma-delta's
// extra cycles and, close to a whole step, the sparser pulses add lines
// below 1/T, under 0.02 ns for INT >= 3.
#pragma once

#include <stdbool.h>
#include <stdint.h>

// Positive: faster output.
#define SPDIF_TRIM_MAX_PPB 2000000

#define SPDIF_TRIM_STEP (1u << 16)   // one FRAC step in Q8.24

// Pattern ring: M periods of two halves, each half a (delay, value) pair.
#define SPDIF_TRIM_RING_PERIODS 64u
#define SPDIF_TRIM_RING_WORDS   (4u * SPDIF_TRIM_RING_PERIODS)
// Pacer cycles of one half beyond its delay word d (spdif_pacer.pio: out,
// d + 1 passes of jmp x--, out, in), and the smallest d.
#define SPDIF_TRIM_HALF_EXTRA   4u
#define SPDIF_TRIM_MIN_DELAY    1u
#define SPDIF_TRIM_MIN_HALF     (SPDIF_TRIM_HALF_EXTRA + SPDIF_TRIM_MIN_DELAY)

static inline int32_t spdif_trim_clamp(int32_t ppb) {
    if (ppb > SPDIF_TRIM_MAX_PPB)
        return SPDIF_TRIM_MAX_PPB;
    if (ppb < -SPDIF_TRIM_MAX_PPB)
        return -SPDIF_TRIM_MAX_PPB;
    return ppb;
}

// div_int / (1 + ppb / 1e9) in Q8.24, rounded, at least 1.0 (the hardware
// divider cannot go below 1: at INT = 1 no faster than nominal). div_int
// must be 1..255.
static inline uint32_t spdif_trim_divider(uint32_t div_int, int32_t ppb) {
    uint64_t num = ((uint64_t)div_int << 24) * 1000000000u;
    uint64_t den = (uint64_t)(1000000000 + (int64_t)spdif_trim_clamp(ppb));
    uint32_t q = (uint32_t)((num + den / 2u) / den);
    return q < 1u << 24 ? 1u << 24 : q;
}

// CLKDIV register value (INT in 31:16, FRAC in 15:8) for a Q8.24 divider.
static inline uint32_t spdif_trim_clkdiv_reg(uint32_t div_q24) {
    return (div_q24 >> 8) & 0xFFFFFF00u;
}

// Share of the time on hi (Q0.32) for the mean rate 1/target: d x hi /
// (target x STEP), d = target - lo.
static inline uint32_t spdif_trim_hi_share(uint32_t target) {
    uint32_t d = target & (SPDIF_TRIM_STEP - 1u);
    uint64_t bias = ((uint64_t)d * (SPDIF_TRIM_STEP - d) << 16) / target;
    return (d << 16) + (uint32_t)bias;
}

// One half of `cycles` that ends with reg written: the value goes in first,
// so a reader racing the write sees the new value with the old delay at
// worst, never the reverse.
static inline void spdif_trim_put(volatile uint32_t *w, uint32_t cycles, uint32_t reg) {
    w[1] = reg;
    w[0] = cycles - SPDIF_TRIM_HALF_EXTRA;
}

// Width of the pulse in period k of n, 0 for none: `pulses` pulses of x
// cycles in all, both spread as evenly as whole cycles and periods allow
// (first-order sigma-delta).
static inline uint32_t spdif_trim_pulse(uint32_t k, uint32_t n, uint32_t pulses, uint32_t x) {
    uint32_t j = k * pulses / n;                // pulses before period k
    if ((k + 1u) * pulses / n == j)
        return 0;
    return (j + 1u) * x / pulses - j * x / pulses;
}

// Fills ring[4 x n] for the Q8.24 target (at least 1.0) with periods of
// `period` pacer cycles (3 x SPDIF_TRIM_MIN_HALF - 1 or more; n <= 64 and
// n x period below 2^26). Each period is lo for period - t_hi cycles, then
// hi for t_hi. The hi time over the ring goes out as pulses of hi on lo, or
// of lo on hi, whichever is shorter: one per period, or if that leaves
// under SPDIF_TRIM_MIN_HALF per period, fewer pulses of at least that
// width; a total below half that minimum rounds to none. A period's first
// write starts its hi part (at mid-period if t_hi is 0 or `period`), the
// second ends it with the value the next period starts with.
static inline void spdif_trim_pattern(volatile uint32_t *ring, uint32_t n, uint32_t period,
                                      uint32_t target) {
    uint32_t lo = target & ~(SPDIF_TRIM_STEP - 1u);
    uint32_t reg_lo = spdif_trim_clkdiv_reg(lo);
    uint32_t reg_hi = spdif_trim_clkdiv_reg(lo + SPDIF_TRIM_STEP);
    uint32_t total = n * period;
    uint32_t r = (uint32_t)(((uint64_t)spdif_trim_hi_share(target) * total + (1u << 31)) >> 32);
    bool on_hi = r <= total - r;
    uint32_t x = on_hi ? r : total - r;
    if (x < SPDIF_TRIM_MIN_HALF)
        x = 2u * x >= SPDIF_TRIM_MIN_HALF ? SPDIF_TRIM_MIN_HALF : 0u;
    uint32_t pulses = x / SPDIF_TRIM_MIN_HALF;
    if (pulses > n)
        pulses = n;
    uint32_t w = spdif_trim_pulse(0, n, pulses, x);
    uint32_t t_hi = on_hi ? w : period - w;
    for (uint32_t k = 0; k < n; k++) {
        w = spdif_trim_pulse(k + 1u < n ? k + 1u : 0u, n, pulses, x);
        uint32_t t_next = on_hi ? w : period - w;
        uint32_t lo_part = t_hi == 0 || t_hi == period ? period / 2u : period - t_hi;
        volatile uint32_t *p = &ring[4u * k];
        spdif_trim_put(p, lo_part, t_hi ? reg_hi : reg_lo);
        spdif_trim_put(p + 2, period - lo_part, t_next == period ? reg_hi : reg_lo);
        t_hi = t_next;
    }
}
