// S/PDIF receiver, portable part (no SDK dependencies, host-testable):
// the decoder's subframe words (spdif_rx.pio) to framed stereo samples,
// channel status and per-block flags, plus the rate arithmetic.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "spdif/spdif_rx.h"

#define SPDIF_RX_BLOCK_FRAMES 192u
#define SPDIF_RX_CS_BYTES     (SPDIF_RX_BLOCK_FRAMES / 8u)

// Word layout, bit 0 first: preamble cells 4, 5, 6 as read; L4..L31, the
// line level right after the edge that starts slot k; L32, the level after
// the next subframe's first edge. Slot k carries NOT (Lk XOR Lk+1), so
// even parity over slots 4..31 means L4 == L32, and L4 equals the level
// of preamble cell 0 (cell 7 is its inverse, the slot-4 edge inverts it
// back).

// Preamble cells 4..6 relative to cell 0, bit 0 = cell 4.
#define SPDIF_RX_PRE_B 6u
#define SPDIF_RX_PRE_M 3u
#define SPDIF_RX_PRE_W 5u

static inline uint32_t spdif_rx_word_code(uint32_t w) {
    return (w ^ (0u - ((w >> 3) & 1u))) & 7u;
}

static inline bool spdif_rx_word_parity_ok(uint32_t w) {
    return (((w >> 3) ^ (w >> 31)) & 1u) == 0;
}

// Slots 4..31 in bits 3..30.
static inline uint32_t spdif_rx_word_slots(uint32_t w) {
    return ~(w ^ (w >> 1));
}

// 24-bit sample, 32-bit left-justified.
static inline int32_t spdif_rx_word_sample(uint32_t w) {
    return (int32_t)((spdif_rx_word_slots(w) << 5) & 0xFFFFFF00u);
}

static inline uint32_t spdif_rx_word_cbit(uint32_t w) {
    return (spdif_rx_word_slots(w) >> 29) & 1u;
}

// Receives frames of one block at a time, all with the same flags
// (SPDIF_RX_FLAG_*); non-PCM frames arrive as zeros.
typedef void (*spdif_rx_emit_fn)(void *ctx, const int32_t *pcm, uint32_t frames, uint32_t flags);

// Frames gathered before a call of the emit function.
#define SPDIF_RX_OUT_FRAMES 64u

typedef struct {
    // Framing.
    bool     aligned;           // the next word is a subframe A
    bool     emitting;          // the current block goes to the output
    uint8_t  n_carry;
    int16_t  frame;             // block position of the next frame, -1 unknown
    uint32_t carry[3];          // words of a pair still waiting for the next one
    // Health. A subframe is good with the expected preamble and even parity.
    uint32_t good_run;          // consecutive good subframes
    uint32_t bad_run;           // consecutive bad subframes
    uint32_t err_parity;        // since init
    uint32_t err_frame;
    uint32_t slips;             // framing discontinuities (word lost or added)
    // Channel status (subframe A, B where A is bad).
    uint8_t  cs[SPDIF_RX_CS_BYTES];         // block being received

    uint8_t  cs_last[SPDIF_RX_CS_BYTES];    // last complete block
    bool     cs_complete;
    uint8_t  block_flags;       // of the current block, known from frame 6
    // Output.
    spdif_rx_emit_fn emit_fn;
    void    *emit_ctx;
    uint32_t n_out;
    int32_t  out[2u * SPDIF_RX_OUT_FRAMES];
} spdif_rx_parser_t;

// Everything cleared, channel status included; no output.
void spdif_rx_parser_init(spdif_rx_parser_t *p);

// Framing lost (decoder restarted): pending frames are dropped; channel
// status, counters and the output setting are kept.
void spdif_rx_parser_resync(spdif_rx_parser_t *p);

// Output from the next block start on (fn NULL: none, at once).
void spdif_rx_parser_set_output(spdif_rx_parser_t *p, spdif_rx_emit_fn fn, void *ctx);

// Consumes n words. The first six frames of a block are held until their
// channel-status bits 0..5 give the block's flags, and a frame until the
// next one has arrived; the rest is emitted by the end of the call. A bad
// subframe, its frame, the frame before and the eight subframes after it
// come out as zeros.

void spdif_rx_parse(spdif_rx_parser_t *p, const uint32_t *words, uint32_t n);

// Flags of the current block, or of the last complete one before frame 6.
bool spdif_rx_parser_pcm(const spdif_rx_parser_t *p);
bool spdif_rx_parser_emphasis(const spdif_rx_parser_t *p);

// Word length from a channel-status block (consumer bits 32..35), 24 when
// not given or professional.
uint8_t spdif_rx_cs_word_bits(const uint8_t cs[SPDIF_RX_CS_BYTES]);

// Measured offset of `words` subframes in `us` microseconds from the
// nominal rate, in ppm (saturated to +-1e6). us > 0.
int32_t spdif_rx_rate_ppm(uint64_t words, uint64_t us, uint32_t nominal);

// The supported rate within tol_ppm of the measurement, 0 if none.
uint32_t spdif_rx_rate_classify(uint64_t words, uint64_t us, uint32_t tol_ppm);

extern const uint32_t spdif_rx_rates[5];
