// S/PDIF line decoder for the self-test. Portable C without Pico SDK
// dependencies, so it is host-testable.
//
// Input: a 1-bit oversampled capture of the line, 32 samples per word,
// earliest sample in bit 31. The capture rate need not be known: the cell
// period is estimated from the run lengths (at least ~3 samples per cell).
// Output: counters describing how well the stream follows IEC 60958 and
// the test pattern (selftest_pattern.h), and optionally the decoded
// frames themselves (the reference for the I2S oversampling check).
//
// The decoder runs in bounded steps (selftest_decode_begin() /
// selftest_decode_step()), so a cooperative task can share its core with
// time-critical work; selftest_decode() runs all steps at once.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define SELFTEST_CS_BYTES 24   // 192-bit channel status block

// Cells a capture may hold (bounds the cell buffer to 20 KB): six blocks
// plus two frames at most.
#define SELFTEST_DECODE_MAX_CELLS 163840u

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

// Decoder state across steps. Private to the decoder.
typedef struct {
    const uint32_t *cap;
    uint32_t n_words;
    const uint8_t *expect_cs;
    unsigned pattern_bits;
    int32_t *frames_out;
    uint32_t frames_max, frames_stored;
    selftest_decode_t *r;
    uint8_t phase;
    // Run iterator (two passes over the words).
    uint32_t word, level, run, count;
    bool first;
    // Cells.
    uint32_t ncells, cell_err;
    uint64_t span_samples;
    uint32_t span_cells;
    // Subframes.
    uint32_t pos;
    bool searching, synced_once;
    struct {
        bool     in_sync, last_was_a, have_b, have_a, have_prev, seen_audio, in_gap;
        uint32_t fc, a_sample, a_cbit, prev_l;
        uint8_t  cs[SELFTEST_CS_BYTES];
    } s;
} selftest_decoder_t;

// Starts decoding n_words of capture. expect_cs: the channel status the
// transmitter should send. pattern_bits: 24 or 16 for the pattern check, 0
// for none (silent frames are still counted, gaps are not). frames_out
// (may be NULL): receives up to frames_max decoded frames as interleaved
// L/R 32-bit left-justified samples, in line order (consecutive unless a
// sync loss is counted). Not reentrant (static work buffers).
void selftest_decode_begin(selftest_decoder_t *d, const uint32_t *cap, uint32_t n_words,
                           const uint8_t expect_cs[SELFTEST_CS_BYTES], unsigned pattern_bits,
                           int32_t *frames_out, uint32_t frames_max, selftest_decode_t *r);

// One bounded step: up to `words` capture words in the run passes, up to
// `subframes` subframes (or 64 x subframes cells of preamble search) in
// the subframe pass. Returns true when the decode is complete.
bool selftest_decode_step(selftest_decoder_t *d, uint32_t words, uint32_t subframes);

// Frames stored in frames_out so far (min(frames, frames_max)).
uint32_t selftest_decode_frames_stored(const selftest_decoder_t *d);

// The whole decode in one call.
void selftest_decode(const uint32_t *cap, uint32_t n_words,
                     const uint8_t expect_cs[SELFTEST_CS_BYTES], unsigned pattern_bits,
                     selftest_decode_t *r);

// Sum of the error counters, plus 1 for a channel status mismatch or no
// complete block. 0 for a clean stream. Gaps are not errors.
uint32_t selftest_decode_errors(const selftest_decode_t *r);

// True if the decoded frame sequence is complete and in order (no sync
// loss, bad cell or parity error): usable as a reference for the samples
// on the line. Pattern violations and the channel status do not matter.
bool selftest_decode_frames_clean(const selftest_decode_t *r);
