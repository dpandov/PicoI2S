// I2S / dual-line output: format table, timing selection, PIO program
// generators and sample converters. Pure integer code without SDK calls
// (the instruction encoders are hardware/pio_instructions.h), so a host
// simulation runs the same programs and converters as the firmware.
//
// Timing: one input frame is 128 x div clk_sys cycles (div: the S/PDIF
// divider), one output frame C_out = 128 x div / osr cycles, made of F
// BCLK slots of N state-machine cycles at divider D (F x N x D = C_out).
// A slot starts with BCLK low for floor(N / 2) cycles (data changes
// there), then high for the rest (the DAC samples on the rising edge).
#pragma once

#include <stdbool.h>
#include <stdint.h>

#if defined(__ARM_FEATURE_SAT)
#include <arm_acle.h>
#endif

#include "hardware/pio_instructions.h"

#include "i2s/i2s_filter.h"
#include "i2s/i2s_tx.h"

// Side-set bits, base I2S_BCLK_PIN.
#define I2S_SIDE_BCLK 1u
#define I2S_SIDE_LINE 2u        // LRCK or LATCH
#define I2S_SIDE_BITS 2u

#define I2S_DIRECT_SLOTS      64u   // I2S / LJ / RJ: two 32-bit slots per frame
#define I2S_DIRECT_LOOP_SLOTS 31u   // loop slots per half; the 32nd is the special one
// Dual-line frames: at most 32 slots, so a frame is at most two words and
// the loop reload (at most F - 2) fits set's 5-bit immediate.
#define I2S_DUAL_SLOTS_MAX    32u
// Dual16 .. dual24: LATCH rises with the slot I2S_LATCH_SLOTS before the
// end of the frame and falls I2S_LATCH_TAIL state-machine cycles into the
// BCLK-low half of the next frame's slot 0. Only the falling edge matters
// to the DACs: three periods are more than any of them asks for, and the
// tail moves the edge away from both BCLK edges around it (see the
// datasheet notes below).
#define I2S_LATCH_SLOTS       3u
#define I2S_LATCH_TAIL        1u
_Static_assert(I2S_LATCH_SLOTS >= 1u && I2S_LATCH_SLOTS <= 3u, "program groups");
// Each half slot is ceil(half / 8) instructions (3-bit delay beside two
// side-set bits); N <= 16 gives 2 instructions per slot, 3 for the dual16
// .. dual24 slot 0 (its LATCH tail): 8 for the four direct groups, 11 for
// dual16 .. dual24 (slot 0, the loop, three LATCH slots), 4 for the
// TDA1541A. 16 holds N <= 32 for the direct formats, N <= 20 or so for
// dual16 .. dual24 and N <= 64 for the TDA1541A.
#define I2S_PROGRAM_MAX       16u
#define I2S_CLOCK_PROGRAM_MAX 10u

typedef enum {
    I2S_CONV_COPY,      // 32-bit words as they are
    I2S_CONV_SHIFT,     // arithmetic right shift (right-justified)
    I2S_CONV_DUAL,      // DL / DR pairs, 2F bits per frame (i2s_pack_dual)
} i2s_conv_t;

typedef struct {
    uint8_t out_pins;       // data pins driven by out: 1, or 2 (DL, DR)
    // LRCK / LATCH level per half slot (bit 0 = BCLK-low half, bit 1 =
    // BCLK-high half) for each instruction group, in program order:
    // direct formats L loop, L last, R loop, R last; dual16 .. dual24
    // loop, LATCH slots (slot 0 with its tail is built in); TDA1541A LATCH
    // slot, loop.
    uint8_t line[4];
    uint8_t conv;           // i2s_conv_t
    uint8_t shift;          // SHIFT: bits
    uint8_t bits;           // DUAL: DAC word length
    bool    latch_first;    // DUAL: LATCH pulse in the BCLK-low half of slot 0 (TDA1541A);
                            // else LATCH high from slot F-3 into slot 0 of the next frame
                            // (I2S_LATCH_SLOTS, I2S_LATCH_TAIL; the LSB is in slot F-1
                            // either way)
    bool    offset_binary;  // DUAL: MSB inverted
} i2s_format_spec_t;

// LRCK formats: I2S has the level of the other half already on each
// half's last slot (the MSB follows the edge by one BCLK); left- and
// right-justified switch on the half.
//
// Dual-line LATCH placement, from the datasheets:
// - TDA1541A, simultaneous mode (OB/TWC at -5 V): 16 bits MSB first in
//   OFFSET BINARY, BCK <= 6.4 MHz, data clocked on the rising BCK edge,
//   output updated on the positive-going LE transition, tFBRL (BCK fall
//   to LE rise) >= 0 and tRBFL (BCK rise to LE fall) >= 0, no BCK edge
//   while LE is high. Here the LSB sits in the last slot and LE fills the
//   BCK-low half of slot 0 of the next frame: it rises with the falling
//   edge that ends the LSB slot and falls with the rising edge that clocks
//   the next MSB (floor(N / 2) x D clk_sys cycles: 65 ns at the shortest,
//   48 kHz on the 153.6 MHz clock).
// - PCM56: 16 bits BTC MSB first, rising CLK edge (<= 10 MHz); LE high
//   for >= 1 clock cycle before going low and low for >= 1 cycle after.
// - AD1865: 18 bits two's complement MSB first, clocked on the rising CLK
//   edge (<= 13.5 MHz); the falling LL / LR edge loads "the 18 most recent
//   bits" into the DAC; figure 9 wants >= 30-40 ns between the edges. LL
//   and LR may be tied together. AD1861 (18 bits) and AD1862 (20 bits,
//   <= 17 MHz): the same, a low-going LE updates the DAC.
// - PCM63 / PCM1702: 20 bits BTC MSB first, rising CLK edge, "any number of
//   bits can precede the 20"; transfer on the falling LE edge, LE high for
//   >= 1 clock cycle before it and low for >= 1 cycle after.
// - PCM1704: 24 bits BTC MSB first, rising BCLK edge (<= 25 MHz), "any
//   number of bits can precede the 24", transfer on the falling WCLK edge;
//   WCLK high and low for more than one BCLK period, 10 ns from a rising
//   BCLK edge to the falling WCLK edge and back.
// - AD1860: LE may be high while the word shifts in; high >= 40 ns, low
//   >= 40 ns, >= 60 ns from the LSB's rising CLK edge to the falling LE
//   edge, >= 15 ns from there to the next rising edge. AD1865: the same
//   with >= 40 ns from the LSB edge (its application drives LL / LR from
//   a 50 % word clock).
// - Dual16 .. dual24: LSB in slot F-1. LATCH rises at the start of slot
//   F-3 (I2S_LATCH_SLOTS BCLK periods before the frame end) and falls one
//   state-machine cycle (D clk_sys, I2S_LATCH_TAIL) after the BCLK falling
//   edge that starts the next frame's slot 0, so the last bit clocked in
//   before it is the LSB; the next MSB is on the data line by then but is
//   clocked only at the rising edge after it. From the LSB's rising edge
//   to the falling LATCH edge: half a period plus D clk_sys, at least
//   78 ns over both clock tables, oversampled or not (the AD1860 asks for
//   60 ns; at the BCLK falling edge it would be 52 ns at the shortest);
//   from there to the next rising edge: the rest of the low half, at
//   least 26 ns (15 ns asked). Worst case dual24 at 48 kHz, F25 N4 D4.
static const i2s_format_spec_t i2s_format_table[I2S_FORMAT_COUNT] = {
    [I2S_FORMAT_I2S]             = {1, {0x0, 0x3, 0x3, 0x0}, I2S_CONV_COPY,  0,                 0,  false, false},
    [I2S_FORMAT_LEFT_JUSTIFIED]  = {1, {0x3, 0x3, 0x0, 0x0}, I2S_CONV_COPY,  0,                 0,  false, false},
    [I2S_FORMAT_RIGHT_JUSTIFIED] = {1, {0x3, 0x3, 0x0, 0x0}, I2S_CONV_SHIFT, 32u - I2S_RJ_BITS, 0,  false, false},
    [I2S_FORMAT_TDA1541A]        = {2, {0x1, 0x0, 0x0, 0x0}, I2S_CONV_DUAL,  0,                 16, true,  true},
    [I2S_FORMAT_DUAL_16]         = {2, {0x0, 0x3, 0x0, 0x0}, I2S_CONV_DUAL,  0,                 16, false, false},
    [I2S_FORMAT_DUAL_18]         = {2, {0x0, 0x3, 0x0, 0x0}, I2S_CONV_DUAL,  0,                 18, false, false},
    [I2S_FORMAT_DUAL_20]         = {2, {0x0, 0x3, 0x0, 0x0}, I2S_CONV_DUAL,  0,                 20, false, false},
    [I2S_FORMAT_DUAL_24]         = {2, {0x0, 0x3, 0x0, 0x0}, I2S_CONV_DUAL,  0,                 24, false, false},
};

typedef struct {
    uint8_t osr;        // output frames per input frame: 1, 4 or 8
    uint8_t slots;      // F: BCLK slots per output frame
    uint8_t cycles;     // N: state-machine cycles per slot
    uint8_t divider;    // D: integer clock divider of the state machine(s)
    uint8_t lsb_slot;   // dual-line: slot carrying the LSB; 0 otherwise
    uint8_t latch_slot; // dual-line: first slot of the LATCH pulse (0 or F - latch_slots); 0 otherwise
    uint8_t latch_slots;// dual-line: LATCH slots before the frame end (1 for the TDA1541A: slot 0)
    uint8_t latch_tail; // dual-line: LATCH cycles into the next frame's slot 0 (0 for the TDA1541A)
    uint8_t out_pins;   // 1 or 2
    uint8_t x_preload;  // set x to this before enabling: the first loop's slot count - 1
} i2s_timing_t;

//--------------------------------------------------------------------+
// Timing selection
//--------------------------------------------------------------------+

// Output frames per input frame: i2s_filter_osr() for the dual-line
// formats, 1 for I2S / LJ / RJ (never filtered).
static inline unsigned i2s_oversampling(i2s_format_t f, uint32_t rate, i2s_filter_type_t filter) {
    return i2s_format_is_dual(f) ? i2s_filter_osr(filter, rate) : 1u;
}

// Smallest frame: 64 slots for the direct formats (fixed), the word alone
// for dual-line (the LATCH pulse shares a slot with the LSB, or in the
// TDA1541A format with the MSB).
static inline unsigned i2s_min_slots(const i2s_format_spec_t *s) {
    return s->conv == I2S_CONV_DUAL ? s->bits : I2S_DIRECT_SLOTS;
}

// Instructions of one slot of n cycles: ceil(half / 8) per half.
static inline unsigned i2s_slot_instructions(unsigned n) {
    unsigned lo = n / 2u, hi = n - lo;
    return (lo + 7u) / 8u + (hi + 7u) / 8u;
}

// LATCH pulse length in slots of a dual-line format, 0 for the others.
static inline unsigned i2s_latch_slots(const i2s_format_spec_t *s) {
    if (s->conv != I2S_CONV_DUAL)
        return 0u;
    return s->latch_first ? 1u : I2S_LATCH_SLOTS;
}

// Program length for n cycles per slot: four slot groups for the direct
// formats, two for the TDA1541A; for dual16 .. dual24 slot 0 (its low half
// split by the LATCH tail), the loop and one per LATCH slot.
static inline unsigned i2s_program_length(const i2s_format_spec_t *s, unsigned n) {
    unsigned slot = i2s_slot_instructions(n);
    if (s->conv != I2S_CONV_DUAL)
        return 4u * slot;
    if (s->latch_first)
        return 2u * slot;
    unsigned lo = n / 2u, hi = n - lo;
    unsigned first = (I2S_LATCH_TAIL + 7u) / 8u + (lo - I2S_LATCH_TAIL + 7u) / 8u + (hi + 7u) / 8u;
    return first + (1u + I2S_LATCH_SLOTS) * slot;
}

// Timing for the S/PDIF divider (128 x spdif_div clk_sys cycles per input
// frame), format and oversampling: the smallest F >= the format's minimum
// dividing C_out = 128 x spdif_div / osr such that C_out / F = N x D with
// N >= 4 and D >= 1 (trim_room: D >= 2, so the divider fraction can carry
// the UAC1 trim); the smallest N (largest D) for that F. The direct
// formats need F = 64 exactly, N a multiple of 8 and D >= 2, so that the
// clock state machine can make the 256 Fs MCLK (i2s_mclk_program_build():
// 2 D cycles per period, even for 50 %), which needs a spdif_div that is a
// multiple of 4 (clock/audio_clock.h). False if no such split exists
// (C_out not a multiple of osr, no F up to the maximum, D over 255 or a
// program over I2S_PROGRAM_MAX).
static inline bool i2s_select_timing(uint32_t spdif_div, i2s_format_t f, unsigned osr,
                                     bool trim_room, i2s_timing_t *t) {
    if (f >= I2S_FORMAT_COUNT || spdif_div == 0 || osr == 0 || (128u * spdif_div) % osr != 0)
        return false;
    const i2s_format_spec_t *s = &i2s_format_table[f];
    bool dual = s->conv == I2S_CONV_DUAL;
    uint32_t c_out = 128u * spdif_div / osr;
    // MCLK (direct): D >= 2 for its 2 D period, 3 with trim room so that the
    // wait has a cycle of slack for the BCLK halves of 4 D - 1.
    unsigned d_min = (trim_room ? 2u : 1u) + (dual ? 0u : 1u);
    unsigned f_max = dual ? I2S_DUAL_SLOTS_MAX : I2S_DIRECT_SLOTS;
    for (unsigned slots = i2s_min_slots(s); slots <= f_max; slots++) {
        if (c_out % slots != 0)
            continue;
        uint32_t per_slot = c_out / slots;
        for (unsigned n = dual ? 4u : 8u; n * d_min <= per_slot; n++) {
            if (per_slot % n != 0 || (!dual && n % 8u != 0))
                continue;
            uint32_t d = per_slot / n;
            if (d > 255u)
                continue;                               // a larger N brings D down
            if (i2s_program_length(s, n) > I2S_PROGRAM_MAX)
                break;                                  // only grows with N
            t->osr = (uint8_t)osr;
            t->slots = (uint8_t)slots;
            t->cycles = (uint8_t)n;
            t->divider = (uint8_t)d;
            t->out_pins = s->out_pins;
            if (dual) {
                unsigned w = i2s_latch_slots(s);
                t->lsb_slot = (uint8_t)(slots - 1u);
                t->latch_slot = (uint8_t)(s->latch_first ? 0u : slots - w);
                t->latch_slots = (uint8_t)w;
                t->latch_tail = (uint8_t)(s->latch_first ? 0u : I2S_LATCH_TAIL);
                // The loop: slots 1 .. F-1 (TDA1541A) or 1 .. F-w-1.
                t->x_preload = (uint8_t)(slots - (s->latch_first ? 0u : w) - 2u);
            } else {
                t->lsb_slot = 0;
                t->latch_slot = 0;
                t->latch_slots = 0;
                t->latch_tail = 0;
                t->x_preload = I2S_DIRECT_LOOP_SLOTS - 1u;
            }
            return true;
        }
    }
    return false;
}

// State-machine cycles per output frame.
static inline unsigned i2s_frame_cycles(const i2s_timing_t *t) {
    return (unsigned)t->slots * t->cycles;
}

// TDA1541A format: BCLK periods per half of the DEM clock. DEM always
// runs at the oversampled word rate (8x Fs below 50 kHz, 4x above), also
// with NOS and the sinc compensation, where it is BCLK / 2 or BCLK / 4 (16
// slots): F x osr / dem_osr BCLK periods per DEM period, a whole number of
// DEM periods per output frame, so it stays phase-locked to LATCH. 0 for
// the other formats, or if a half is not a whole number of BCLK periods
// from 1 to 32.
static inline unsigned i2s_dem_half_bclks(const i2s_timing_t *t, i2s_format_t f, uint32_t rate) {
    unsigned dem_osr = i2s_filter_osr(I2S_FILTER_SHARP, rate);
    unsigned bclks = (unsigned)t->slots * t->osr;
    if (f != I2S_FORMAT_TDA1541A || t->osr == 0 || dem_osr % t->osr != 0 ||
        bclks % (2u * dem_osr) != 0)
        return 0u;
    unsigned k = bclks / (2u * dem_osr);
    return k <= 32u ? k : 0u;
}

// Words a block of n_frames output frames occupies: n_frames x F x
// out_pins / 32, whole when n_frames x F is a multiple of 32 (direct) or
// 16 (dual-line); the blocks used, 96 x osr frames, always are.
static inline uint32_t i2s_words_per_frames(const i2s_timing_t *t, uint32_t n_frames) {
    return n_frames * t->slots * t->out_pins / 32u;
}

//--------------------------------------------------------------------+
// Programs
//--------------------------------------------------------------------+

// Builds the program for the timing into prog[I2S_PROGRAM_MAX]. A slot is
// `out pins, n` (data changes at the slot start) followed by nops, BCLK
// low then high, the LRCK / LATCH level per half from the format table;
// the slot's last instruction is `jmp x--` to the group's start (loop
// group), `set x` to the loop's count - 1 (special slot) or a nop (LATCH
// slot). Groups: direct formats L loop (31 slots), L last, R loop, R last;
// dual-line the special slot 0, then the loop of the following slots;
// dual16 .. 24 end the frame with I2S_LATCH_SLOTS LATCH slots and carry
// LATCH into slot 0 for I2S_LATCH_TAIL cycles of its low half; the
// TDA1541A's slot 0 carries its LE pulse in the low half. Wrap target 0,
// wrap at length - 1. Before enabling, x must be t->x_preload (dual-line:
// the program starts at slot 0, which sets x itself). Returns the length.
static inline unsigned i2s_program_build(uint16_t *prog, const i2s_timing_t *t, i2s_format_t f) {
    enum { LOOP, PLAIN, SPECIAL };
    const i2s_format_spec_t *s = &i2s_format_table[f];
    bool dual = s->conv == I2S_CONV_DUAL;
    unsigned loop_slots = dual ? (unsigned)t->x_preload + 1u : I2S_DIRECT_LOOP_SLOTS;
    uint8_t kind[2u + I2S_LATCH_SLOTS], line[2u + I2S_LATCH_SLOTS];
    unsigned groups = 0;
    if (!dual) {
        for (unsigned g = 0; g < 4u; g++) {
            kind[groups] = (g & 1u) ? SPECIAL : LOOP;
            line[groups++] = s->line[g];
        }
    } else if (s->latch_first) {
        kind[groups] = SPECIAL;
        line[groups++] = s->line[0];
        kind[groups] = LOOP;
        line[groups++] = s->line[1];
    } else {
        kind[groups] = SPECIAL;
        line[groups++] = 0x0;                   // plus the tail
        kind[groups] = LOOP;
        line[groups++] = s->line[0];
        for (unsigned k = 0; k < t->latch_slots; k++) {
            kind[groups] = PLAIN;
            line[groups++] = s->line[1];
        }
    }
    unsigned n = t->cycles, lo = n / 2u, hi = n - lo;
    unsigned len = 0;
    for (unsigned g = 0; g < groups; g++) {
        unsigned start = len;
        // Segments of the slot: cycles, BCLK half, LRCK / LATCH level.
        unsigned seg_n[3], seg_half[3], seg_line[3], segs = 0;
        unsigned tail = kind[g] == SPECIAL && dual ? t->latch_tail : 0u;
        if (tail) {
            seg_n[segs] = tail;
            seg_half[segs] = 0;
            seg_line[segs++] = 1;
        }
        seg_n[segs] = lo - tail;
        seg_half[segs] = 0;
        seg_line[segs++] = line[g] & 1u;
        seg_n[segs] = hi;
        seg_half[segs] = 1;
        seg_line[segs++] = (line[g] >> 1) & 1u;
        bool first = true;
        for (unsigned k = 0; k < segs; k++) {
            unsigned side = (seg_half[k] ? I2S_SIDE_BCLK : 0u) | (seg_line[k] ? I2S_SIDE_LINE : 0u);
            unsigned left = seg_n[k];
            while (left) {
                unsigned d = left > 8u ? 8u : left;
                left -= d;
                bool last = k == segs - 1u && left == 0;
                unsigned op;
                if (first)
                    op = pio_encode_out(pio_pins, s->out_pins);
                else if (last && kind[g] == SPECIAL)
                    op = pio_encode_set(pio_x, loop_slots - 1u);
                else if (last && kind[g] == LOOP)
                    op = pio_encode_jmp_x_dec(start);
                else
                    op = pio_encode_nop();
                prog[len++] = (uint16_t)(op | pio_encode_sideset(I2S_SIDE_BITS, side) |
                                         pio_encode_delay(d - 1u));
                first = false;
            }
        }
    }
    return len;
}

// Builds the direct formats' MCLK program into prog[I2S_CLOCK_PROGRAM_MAX]
// for the clock state machine running at clk_sys (divider 1): two MCLK
// periods of 2 D cycles per BCLK half (4 D cycles, N = 8), restarted on
// every BCLK edge read back from the pad (`wait gpio`), so it follows the
// I2S state machine's divider, dithered or not, without a divider of its
// own: the wait ends MCLK's last low half, so a BCLK half that the
// fractional divider makes longer or shorter than 4 D cycles (a positive
// trim runs INT D - 1 with FRAC near 256: halves of 4 D - 1) moves that
// low half by the same amount. MCLK rises 3 cycles after each BCLK edge
// (2 of pad synchroniser, 1 of wait), a fixed offset; the wait is reached
// D - 2 cycles before a nominal edge. Wrap target 0, wrap at length - 1.
// Returns the length, 0 for D < 2.
static inline unsigned i2s_mclk_program_build(uint16_t *prog, unsigned d, unsigned bclk_gpio) {
    if (d < 2u || d > 32u)
        return 0;
    unsigned len = 0;
    for (unsigned level = 2; level-- > 0;) {        // the rising, then the falling edge
        prog[len++] = (uint16_t)pio_encode_wait_gpio(level != 0, bclk_gpio);
        prog[len++] = (uint16_t)(pio_encode_set(pio_pins, 1) | pio_encode_delay(d - 1u));
        prog[len++] = (uint16_t)(pio_encode_set(pio_pins, 0) | pio_encode_delay(d - 1u));
        prog[len++] = (uint16_t)(pio_encode_set(pio_pins, 1) | pio_encode_delay(d - 1u));
        prog[len++] = (uint16_t)pio_encode_set(pio_pins, 0);   // the wait ends this half
    }
    return len;
}

// Builds the TDA1541A DEM program into prog[I2S_CLOCK_PROGRAM_MAX] for the
// clock state machine running at clk_sys (divider 1): a 50 % square wave of
// 2 k BCLK periods, each half counting k BCLK periods on the pad (`wait
// gpio`), so it follows the I2S state machine's divider, trimmed or not.
// Enabled together with the I2S state machine, DEM rises with the frame
// start, then every 2 k BCLK periods: every later DEM edge comes a fixed 4
// clk_sys after a falling BCLK edge (2 of pad synchroniser, the wait, the
// jmp), so the first high half is 4 cycles longer than the others. Wrap
// target 0, wrap at length - 1. Returns the length, 0 for k outside 1..32.
static inline unsigned i2s_dem_program_build(uint16_t *prog, unsigned k, unsigned bclk_gpio) {
    if (k < 1u || k > 32u)
        return 0;
    unsigned len = 0;
    for (unsigned level = 2; level-- > 0;) {        // 1 then 0
        prog[len++] = (uint16_t)pio_encode_set(pio_pins, level);
        prog[len++] = (uint16_t)pio_encode_set(pio_x, k - 1u);
        unsigned loop = len;
        prog[len++] = (uint16_t)pio_encode_wait_gpio(true, bclk_gpio);
        prog[len++] = (uint16_t)pio_encode_wait_gpio(false, bclk_gpio);
        prog[len++] = (uint16_t)pio_encode_jmp_x_dec(loop);
    }
    return len;
}

//--------------------------------------------------------------------+
// Converters (interrupt or core-1 context: the callers are in RAM)
//--------------------------------------------------------------------+

#define I2S_CONV_INLINE static inline __attribute__((always_inline))

// Direct formats: two 32-bit left-justified words per frame.
I2S_CONV_INLINE void i2s_convert_copy(uint32_t *dst, const int32_t *pcm, uint32_t n_frames) {
    for (uint32_t i = 0; i < 2u * n_frames; i++)
        dst[i] = (uint32_t)pcm[i];
}

I2S_CONV_INLINE void i2s_convert_shift(uint32_t *dst, const int32_t *pcm, uint32_t n_frames,
                                       unsigned shift) {
    for (uint32_t i = 0; i < 2u * n_frames; i++)
        dst[i] = (uint32_t)(pcm[i] >> shift);
}

// Dual-line packer tables, built at run time so the caller decides where
// they live (RAM for the interrupt and core 1): spread maps bit k of the
// index to bit 2k; lo is the spread of the DAC word's bits 16..23 (index:
// sample bits 16..23, those beyond the word length cleared) in bits 31..16.
#define I2S_PACK_TABLE_ENTRIES 256u

typedef struct {
    uint16_t spread[I2S_PACK_TABLE_ENTRIES];
    uint32_t lo[I2S_PACK_TABLE_ENTRIES];
} i2s_pack_tables_t;

static inline void i2s_pack_tables_init(i2s_pack_tables_t *tab, unsigned bits) {
    unsigned keep = bits >= 24u ? 0xFFu : bits <= 16u ? 0u : (0xFFu << (24u - bits)) & 0xFFu;
    for (unsigned b = 0; b < I2S_PACK_TABLE_ENTRIES; b++) {
        uint32_t v = 0;
        for (unsigned k = 0; k < 8u; k++)
            v |= ((b >> k) & 1u) << (2u * k);
        tab->spread[b] = (uint16_t)v;
    }
    for (unsigned b = 0; b < I2S_PACK_TABLE_ENTRIES; b++)
        tab->lo[b] = (uint32_t)tab->spread[b & keep] << 16;
}

_Static_assert(I2S_FILTER_SCALE_SHIFT == 4, "the packer saturates to 28 bits");

// A packer input sample reduced to its top 28 bits, in bits 27..0 (the
// bits above are ignored): filter output (scaled: 1/16 scale) saturated
// as i2s_filter_output() does, or a full-scale 32-bit left-justified
// sample shifted down. The top 24 bits equal those of i2s_filter_output()
// or of the sample.
I2S_CONV_INLINE int32_t i2s_pack_input(int32_t y, bool scaled) {
    if (!scaled)
        return y >> 4;
#if defined(__ARM_FEATURE_SAT)
    return __ssat(y, 28);
#else
    const int32_t lim = (int32_t)1 << 27;
    return y >= lim ? lim - 1 : y < -lim ? -lim : y;
#endif
}

// The top 16 bits of both packer inputs as 32 stream bits: per bit DR
// then DL, MSB first.
I2S_CONV_INLINE uint32_t i2s_pack_hi(const uint16_t *sp, int32_t l, int32_t r) {
    uint32_t l1 = sp[((uint32_t)l >> 20) & 0xFFu], l0 = sp[((uint32_t)l >> 12) & 0xFFu];
    uint32_t r1 = sp[((uint32_t)r >> 20) & 0xFFu], r0 = sp[((uint32_t)r >> 12) & 0xFFu];
    return ((l1 << 16) | l0) | (((r1 << 16) | r0) << 1);
}

// A frame's stream bits from interleaved input: the top 16 bits of the
// DAC words in hi (the MSB pair inverted by ob), the rest left-aligned in
// lo (zeros beyond the word length). sp, lt: the tables' spread and lo.
I2S_CONV_INLINE void i2s_pack_data(const uint16_t *sp, const uint32_t *lt, const int32_t *pcm,
                                   bool scaled, uint32_t ob, uint32_t *hi, uint32_t *lo) {
    int32_t l = i2s_pack_input(pcm[0], scaled), r = i2s_pack_input(pcm[1], scaled);
    *hi = i2s_pack_hi(sp, l, r) ^ ob;
    *lo = lt[((uint32_t)l >> 4) & 0xFFu] | (lt[((uint32_t)r >> 4) & 0xFFu] << 1);
}

// Hides where p points, so that GCC keeps the lo table's address in a
// register of its own instead of adding its offset into the tables to
// every index.
#if defined(__GNUC__)
#define I2S_PACK_OPAQUE(p) __asm__("" : "+r"(p))
#else
#define I2S_PACK_OPAQUE(p) ((void)0)
#endif

// General packer loop (see i2s_pack_dual()), e = 32 - p. A frame writes
// the pending word completed by the top of hi and the word after it,
// complete or not (the next frame rewrites it if not); adv = 2F - 32 bits
// remain past the first.
I2S_CONV_INLINE void i2s_pack_any(uint32_t *dst, const int32_t *pcm, const int32_t *end,
                                  const i2s_pack_tables_t *tab, bool scaled, uint32_t ob,
                                  unsigned lead, int32_t adv) {
    uint32_t pend = 0, hi, lo;
    int32_t e;
    const uint16_t *sp = tab->spread;
    const uint32_t *lt = tab->lo;
    I2S_PACK_OPAQUE(lt);
    if (lead == 0) {
        // Nothing pending: the first frame's hi is a whole word.
        i2s_pack_data(sp, lt, pcm, scaled, ob, &hi, &lo);
        pcm += 2;
        *dst++ = hi;
        if (pcm == end)
            return;             // not a whole-word block
        pend = lo;
        e = 32 - adv;
    } else {
        e = 32 - (int32_t)lead;
    }
    for (end -= 2; pcm != end; pcm += 2) {
        i2s_pack_data(sp, lt, pcm, scaled, ob, &hi, &lo);
        uint32_t m = 1u << e;
        uint64_t a = ((uint64_t)pend << 32) + (uint64_t)hi * m;
        uint64_t b = (uint64_t)lo * m;
        uint32_t w1 = (uint32_t)a | (uint32_t)(b >> 32);
        dst[0] = (uint32_t)(a >> 32);
        dst[1] = w1;
        e -= adv;
        if (e < 0) {
            e += 32;
            dst += 2;
            pend = (uint32_t)b;
        } else {
            dst += 1;
            pend = w1;
        }
    }
    // The last frame ends on the block's word boundary (p + 2 x bits -
    // 32 = 32): both words are complete and nothing remains.
    i2s_pack_data(sp, lt, pcm, scaled, ob, &hi, &lo);
    uint32_t m = 1u << e;
    uint64_t a = ((uint64_t)pend << 32) + (uint64_t)hi * m;
    dst[0] = (uint32_t)(a >> 32);
    dst[1] = (uint32_t)a | (uint32_t)(((uint64_t)lo * m) >> 32);
}

// Dual-line packer: n_frames frames from interleaved stereo samples (L, R)
// into the 2F-bit frame layout, a continuous MSB-first stream of pairs
// (DR, DL) per slot (bit 0 of each `out pins, 2` pair is the lower pin,
// DL) packed into 32-bit words. scaled: filter output at 1/16 scale,
// saturated here (i2s_pack_input()); else full-scale samples. Each sample
// is truncated to the format's word length, the MSB inverted for offset
// binary; the word ends in t->lsb_slot (slot F-1), the leading slots
// carry zeros. n_frames x F must be a multiple of 16 so the block ends on
// a word boundary. tab: i2s_pack_tables_init() with the format's word
// length.
//
// Four loops: F 16 (dual16 and the TDA1541A, one word per frame), F 32
// (two words, the data at a fixed offset), F 24 (three words per two
// frames, fixed offsets), both without offset binary (the TDA1541A is
// F 16 in practice), and
// any other F, where a frame starts at any even bit of a word: each frame
// is taken as its data followed by the next frame's leading zeros, the
// block starts with the first frame's and ends with the last data. The general loop keeps p (1..32) stream bits
// pending at the top of a word and places the data with a 32 x 32 -> 64
// multiply by 2^(32 - p), which splits it across two words in one
// instruction (no shift by 32 in C).
I2S_CONV_INLINE void i2s_pack_dual(uint32_t *dst, const int32_t *pcm, uint32_t n_frames,
                                   const i2s_timing_t *t, const i2s_format_spec_t *spec,
                                   const i2s_pack_tables_t *tab, bool scaled) {
    if (n_frames == 0)
        return;
    unsigned bits = spec->bits;
    unsigned lead = 2u * (t->lsb_slot + 1u - bits);
    uint32_t ob = spec->offset_binary ? 0xC0000000u : 0u;     // the MSB pair in hi
    const int32_t *end = pcm + 2u * n_frames;

    if (t->slots == 16u) {
        do {
            int32_t l = i2s_pack_input(pcm[0], scaled), r = i2s_pack_input(pcm[1], scaled);
            pcm += 2;
            *dst++ = i2s_pack_hi(tab->spread, l, r) ^ ob;
        } while (pcm != end);
        return;
    }

    if (t->slots == 32u && lead > 0 && lead < 32u && !ob) {
        unsigned rl = 32u - lead;
        const uint16_t *sp = tab->spread;
        const uint32_t *lt = tab->lo;
        I2S_PACK_OPAQUE(lt);
        do {
            uint32_t hi, lo;
            i2s_pack_data(sp, lt, pcm, scaled, 0u, &hi, &lo);
            pcm += 2;
            dst[0] = hi >> lead;
            dst[1] = (hi << rl) | (lo >> lead);
            dst += 2;
        } while (pcm != end);
        return;
    }

    if (t->slots == 24u && lead > 0 && lead < 16u && n_frames % 2u == 0 && !ob) {
        // Frame A at bit 0 of the first word, B at bit 16 of the second.
        unsigned ra = 32u - lead, sb = 16u + lead, rb = 16u - lead;
        const uint16_t *sp = tab->spread;
        const uint32_t *lt = tab->lo;
        I2S_PACK_OPAQUE(lt);
        do {
            uint32_t ha, la, hb, lb;
            i2s_pack_data(sp, lt, pcm, scaled, 0u, &ha, &la);
            i2s_pack_data(sp, lt, pcm + 2, scaled, 0u, &hb, &lb);
            pcm += 4;
            dst[0] = ha >> lead;
            dst[1] = (ha << ra) | (la >> lead) | (hb >> sb);
            dst[2] = (hb << rb) | (lb >> sb);
            dst += 3;
        } while (pcm != end);
        return;
    }

    // The general loop with ob a constant: only the TDA1541A inverts the MSB.
    int32_t adv = 2 * (int32_t)t->slots - 32;
    if (ob)
        i2s_pack_any(dst, pcm, end, tab, scaled, 0xC0000000u, lead, adv);
    else
        i2s_pack_any(dst, pcm, end, tab, scaled, 0u, lead, adv);
}
