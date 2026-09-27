// S/PDIF (IEC 60958) frame encoder: PCM -> biphase-mark cells for the PIO.
//
// Output: 4 words per stereo frame, in order A0 A1 B0 B1 (subframe A = left,
// B = right, two words each). Each word carries 32 cells, sent MSB first.
// A cell is half a time slot; 64 cells per subframe, 128 per frame.
//
// Subframe time slots: 0-3 preamble, 4-27 audio (24 bits, LSB first),
// 28 validity (0), 29 user (0), 30 channel status, 31 even parity over 4-31.
// Preambles for a preceding cell level of 0 (inverted when it is 1):
//   B (block start, subframe A of frame 0)  11101000
//   M (subframe A, other frames)            11100010
//   W (subframe B)                          11100100
#pragma once

#include <stdint.h>

#define SPDIF_BLOCK_FRAMES   192
#define SPDIF_WORDS_PER_FRAME 4
#define SPDIF_CS_BYTES       (SPDIF_BLOCK_FRAMES / 8)

typedef struct {
    uint8_t  cs[SPDIF_CS_BYTES];  // channel status block, bit k = cs[k/8] bit k%8
    uint32_t frame;               // position in the block, 0..191
    uint32_t level;               // line level after the last cell, 0 or 1
} spdif_encoder_t;

// Starts a new block with the given channel status; line level starts at 0.
// Call at least once before spdif_encode_frames() (builds the cell table).
void spdif_encoder_init(spdif_encoder_t *enc, const uint8_t cs[SPDIF_CS_BYTES]);

// Encodes n_frames interleaved L/R samples. Samples are 32-bit two's
// complement, left-justified: bits 31..8 are the 24-bit sample, bits 7..0
// are ignored. out receives n_frames * SPDIF_WORDS_PER_FRAME words.
// Channel-status bit k goes into both subframes of block frame k.
void spdif_encode_frames(spdif_encoder_t *enc, const int32_t *pcm,
                         uint32_t n_frames, uint32_t *out);

// Consumer channel status (IEC 60958-3): PCM, copy permitted, category
// general, no pre-emphasis, sample-rate code and word length (16 or 24).
// Returns 0 on success, -1 for an unsupported rate or word length.
int spdif_channel_status_build(uint8_t cs[SPDIF_CS_BYTES], uint32_t sample_rate,
                               uint8_t word_bits);
