// I2S / dual-line decoder for the self-test. Portable C without Pico SDK
// dependencies, so it is host-testable.
//
// Input: a capture taken right after every BCLK falling edge (the slot's
// data and the LRCK / LATCH level of its BCLK-low half), W bits per sample
// (selftest_i2s_layout_t: the pins from I2S_DATA_PIN upwards), 32 / W
// samples per word, earliest sample in the top bits. Within a sample bit 0
// is DATA (DL in the dual-line formats), bit 1 DR, bit clk_bit BCLK and
// bit clk_bit + 1 LRCK (LATCH). Output: counters describing how well the
// stream follows the format's frame layout (from the generator's timing,
// i2s/i2s_format.h) and either the test pattern (selftest_pattern.h) or a
// reference stream (the filtered formats: selftest_i2s_expect.h), with
// the samples rebuilt as 32-bit left-justified words.
//
// The decoder runs in bounded steps (selftest_i2s_decode_begin() /
// selftest_i2s_decode_step()); selftest_i2s_decode() runs them all.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "i2s/i2s_tx.h"

enum {
    SELFTEST_I2S_OK = 0,    // decoded (check the counters)
    SELFTEST_I2S_NOSYNC,    // LRCK never toggles / no LATCH pulse
};

// Capture and frame layout of one window.
typedef struct {
    uint8_t  sample_bits;   // W: 8 or 16
    uint8_t  clk_bit;       // BCLK bit in a sample (I2S_BCLK_PIN - I2S_DATA_PIN)
    uint8_t  format;        // i2s_format_t
    uint8_t  width;         // bits of a sample the format carries: 32 (i2s, lj),
                            // 24 / 16 (rj), or the dual-line word length
    uint8_t  slots;         // F: BCLK slots per frame (64 for the LRCK formats)
    uint8_t  lsb_slot;      // dual-line: slot of the LSB
    uint8_t  latch_slot;    // dual-line: first slot where LATCH is sampled high
    uint8_t  latch_slots;   // dual-line: LATCH pulse length in slots
    bool     offset_binary; // dual-line: MSB inverted on the line
} selftest_i2s_layout_t;

// Reference stream for the filtered formats: the words the output must
// carry, per output frame, as 32-bit left-justified two's complement with
// the bits below the width zero (what the decoder rebuilds). The captured
// frames are located in it by their first run of frames that are neither
// silent nor clipped.
typedef struct {
    const uint32_t *l, *r;
    uint32_t n;             // frames usable for the compare
    uint32_t search_from;   // first index the captured window may start at
} selftest_i2s_reference_t;

// A decoded frame: 32-bit left-justified words, bits below the width zero,
// two's complement. resync: not consecutive to the previous frame.
typedef struct {
    uint32_t l, r;
    uint32_t resync;
} selftest_i2s_frame_t;

typedef struct {
    int      status;        // SELFTEST_I2S_*
    uint32_t frames;        // complete frames decoded in sync
    uint32_t silent;        // frames with L = R = 0 (included in frames)
    uint32_t err_frame;     // LRCK half frames not F/2 slots, LATCH pulses not
                            // 1 slot wide or not F slots apart, samples with
                            // BCLK high
    uint32_t err_sample;    // pattern violations, or reference mismatches
                            // (non-silent frames)
    uint32_t gaps;          // silence between two pattern frames (dropouts)
    // First violation: frame index in the window, previous L (pattern; 0 if
    // none) or the expected L (reference), L and R of the offending frame,
    // as 32-bit words.
    uint32_t viol_frame, viol_prev, viol_l, viol_r;
    bool     located;       // reference: the window was found in the stream
    int32_t  offset;        // reference: index of captured frame 0 in it
} selftest_i2s_decode_t;

// Decoder state across steps. Private to the decoder.
typedef struct {
    const uint32_t *cap;
    uint32_t n;                     // samples
    selftest_i2s_layout_t lay;
    unsigned pattern_bits;
    const selftest_i2s_reference_t *ref;
    selftest_i2s_frame_t *frames;
    uint32_t max_frames;
    selftest_i2s_decode_t *r;
    uint8_t  phase;
    uint8_t  sample_shift;          // log2(samples per word)
    uint32_t sample_mask;
    uint32_t i;                     // scan / check position
    uint32_t ones;                  // LATCH samples high (scan)
    // Framing.
    uint32_t e, l;                  // LRCK: current edge, pending left word
    bool     have_l, resync;
    uint32_t idle, pulse;           // LATCH: idle level, current pulse
    bool     synced;
    // Pattern check.
    struct {
        unsigned e;
        uint32_t mask_e, mask_low, step, prev_l;
        bool     step_carry, have_prev, seen_audio, in_gap;
    } p;
    // Reference compare.
    uint32_t first, key, pos;
} selftest_i2s_decoder_t;

// Starts decoding n_samples of capture in the given layout. pattern_bits:
// 24 or 16 (the stream's word length) for the pattern check, 0 for none
// (silent frames are still counted, gaps are not). ref (may be NULL): the
// reference stream to compare against instead of the pattern. frames: a
// buffer for max_frames decoded frames (at least n_samples / F).
void selftest_i2s_decode_begin(selftest_i2s_decoder_t *d, const uint32_t *cap, uint32_t n_samples,
                               const selftest_i2s_layout_t *lay, unsigned pattern_bits,
                               const selftest_i2s_reference_t *ref, selftest_i2s_frame_t *frames,
                               uint32_t max_frames, selftest_i2s_decode_t *r);

// One bounded step of roughly `budget` samples of work (the check passes
// charge 16 per frame or reference position). True when complete.
bool selftest_i2s_decode_step(selftest_i2s_decoder_t *d, uint32_t budget);

// The whole decode in one call.
void selftest_i2s_decode(const uint32_t *cap, uint32_t n_samples, const selftest_i2s_layout_t *lay,
                         unsigned pattern_bits, const selftest_i2s_reference_t *ref,
                         selftest_i2s_frame_t *frames, uint32_t max_frames,
                         selftest_i2s_decode_t *r);

// Sum of the error counters. 0 for a clean stream. Gaps are not errors.
uint32_t selftest_i2s_decode_errors(const selftest_i2s_decode_t *r);

// Bits of a sample the format carries: 32 (i2s, lj), 24 (rj24), 16
// (rj16), or the dual-line word length.
unsigned selftest_i2s_format_width(i2s_format_t f);
