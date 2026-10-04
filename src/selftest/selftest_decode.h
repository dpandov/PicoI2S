// S/PDIF line decoder for the self-test. Portable C without Pico SDK
// dependencies, so it is host-testable.
//
// Input: a 1-bit oversampled capture of the line, 32 samples per word,
// earliest sample in bit 31. The capture rate need not be known: the cell
// period is estimated from the run lengths (at least ~3 samples per cell).
// Output: counters describing how well the stream follows IEC 60958 and
// the test pattern (selftest_pattern.h).
#pragma once

#include <stdint.h>

#define SELFTEST_CS_BYTES 24   // 192-bit channel status block

enum {
    SELFTEST_DEC_OK = 0,    // decoded (check the counters)
    SELFTEST_DEC_NOSIG,     // no transitions: static line
    SELFTEST_DEC_NOSYNC,    // transitions, but no valid preamble
};

typedef struct {
    int      status;        // SELFTEST_DEC_*
    uint32_t level;         // NOSIG: the static line level
    uint32_t runs;          // complete runs of equal samples
    uint32_t ovs_q8;        // samples per cell, Q24.8
    uint32_t frames;        // frames decoded in sync
    uint32_t blocks;        // B preambles seen in sync
    uint32_t err_cell;      // runs not 1..3 cells, missing slot-boundary
                            // transitions, bad preambles, sync losses
    uint32_t err_parity;    // subframes with odd parity over slots 4..31
    uint32_t err_order;     // M/W alternation, B spacing (192 frames)
    uint32_t err_vu;        // validity or user bit set
    uint32_t err_cs_ab;     // C bit differs between the two subframes
    int      cs_match;      // 1 every complete block matched, 0 mismatch,
                            // -1 no complete block
    uint32_t err_sample;    // pattern violations (non-silent frames)
    uint32_t silent;        // frames with L = R = 0 (included in frames)
    uint32_t gaps;          // silence between two pattern frames (dropouts)
    // First pattern violation: frame index in the window, previous L (0 if
    // none), L and R of the offending frame.
    uint32_t viol_frame, viol_prev, viol_l, viol_r;
    uint64_t span_samples;  // capture samples covered by complete runs
    uint32_t span_cells;    // cells in those runs (rate measurement)
    uint8_t  cs[SELFTEST_CS_BYTES];   // last complete channel status block
} selftest_decode_t;

// Decodes n_words of capture. expect_cs: the channel status the
// transmitter should send. pattern_bits: 24 or 16 for the pattern check, 0
// for none (silent frames are still counted, gaps are not). Not reentrant
// (static work buffers).
void selftest_decode(const uint32_t *cap, uint32_t n_words,
                     const uint8_t expect_cs[SELFTEST_CS_BYTES], unsigned pattern_bits,
                     selftest_decode_t *r);

// Sum of the error counters, plus 1 for a channel status mismatch or no
// complete block. 0 for a clean stream. Gaps are not errors.
uint32_t selftest_decode_errors(const selftest_decode_t *r);
