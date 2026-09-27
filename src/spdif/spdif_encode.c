// S/PDIF (IEC 60958) frame encoder, see spdif_encode.h for the cell format.
//
// Every subframe starts and ends at the same line level: a preamble has four
// transitions, and slots 4-31 have one transition each plus one per 1 bit,
// of which there is an even number (even parity). The level therefore only
// changes inside a subframe, and enc->level stays at its initial value.

#include "spdif/spdif_encode.h"

#include <string.h>

#if PICO_ON_DEVICE
#include "pico.h"
// Runs from the DMA interrupt: keep it out of XIP so a cache miss cannot
// stall the refill.
#define ENCODE_FUNC(name) __not_in_flash_func(name)
// Forced so that no helper lands in flash, even in a Debug build.
#define ENCODE_INLINE static __force_inline
#else
#define ENCODE_FUNC(name) name
#define ENCODE_INLINE static inline
#endif

// Preamble cells for a preceding level of 0, in bits 31..24 of word 0.
#define PRE_B 0xE8000000u
#define PRE_M 0xE2000000u
#define PRE_W 0xE4000000u

// byte -> biphase-mark cells of its 8 bits, LSB first, for a starting level
// of 0: bits 15..0 hold the 16 cells (first cell in bit 15), bits 31..16 are
// all ones if the level after the byte is 1 (odd parity). XOR with 0 or ~0
// gives the cells for either starting level, and the new upper half is the
// mask for the next byte. Built at init into RAM (1 KB) so the interrupt
// never waits on flash.
static uint32_t s_cells[256];
static volatile uint32_t s_cells_ready;

static void build_cell_table(void) {
    for (uint32_t b = 0; b < 256; b++) {
        uint32_t level = 0, cells = 0;
        for (uint32_t i = 0; i < 8; i++) {
            uint32_t first = level ^ 1u;                 // transition at slot start
            uint32_t second = first ^ ((b >> i) & 1u);   // mid-slot transition for a 1
            cells = (cells << 2) | (first << 1) | second;
            level = second;
        }
        s_cells[b] = cells | (level ? 0xFFFF0000u : 0u);
    }
    s_cells_ready = 1;
}

void spdif_encoder_init(spdif_encoder_t *enc, const uint8_t cs[SPDIF_CS_BYTES]) {
    if (!s_cells_ready)
        build_cell_table();
    memcpy(enc->cs, cs, SPDIF_CS_BYTES);
    enc->frame = 0;
    enc->level = 0;
}

// Mask for the next byte from a table entry already XORed with the current
// mask: 0 or ~0.
ENCODE_INLINE uint32_t next_mask(uint32_t e) {
    return 0u - (e >> 31);
}

// Encodes one subframe into out[0..1]. m = 0 or ~0 for line level 0 or 1,
// pre = preamble cells already XORed with m, c26 = channel-status bit << 26.
ENCODE_INLINE void encode_subframe(int32_t sample, uint32_t c26, uint32_t pre,
                                   uint32_t m, uint32_t *out) {
    // Slots 4..31 as bits 0..27: audio 0-23, V 24, U 25, C 26, P 27 (0 here).
    uint32_t x = ((uint32_t)sample >> 8) | c26;

    uint32_t e0 = s_cells[x & 0xFFu] ^ m;                         // slots 4-11
    uint32_t e1 = s_cells[(x >> 8) & 0x0Fu] ^ next_mask(e0);      // slots 12-15
    uint32_t e2 = s_cells[(x >> 12) & 0xFFu] ^ next_mask(e1);     // slots 16-23
    uint32_t e3 = s_cells[x >> 20] ^ next_mask(e2);               // slots 24-31

    // e1 is a 4-bit index: its cells are in bits 15..8.
    out[0] = pre | ((e0 & 0xFFFFu) << 8) | ((e1 >> 8) & 0xFFu);

    // Parity: slot 31 was encoded as 0, so the last cell ends at the level
    // before it. Forcing that cell to the subframe start level (see top)
    // adds the mid-slot transition exactly when the parity bit must be 1.
    out[1] = (e2 << 16) | (e3 & 0xFFFEu) | (m & 1u);
}

void ENCODE_FUNC(spdif_encode_frames)(spdif_encoder_t *enc, const int32_t *pcm,
                                      uint32_t n_frames, uint32_t *out) {
    uint32_t m = 0u - (enc->level & 1u);
    uint32_t pre_m = PRE_M ^ (m & 0xFF000000u);
    uint32_t pre_w = PRE_W ^ (m & 0xFF000000u);
    uint32_t frame = enc->frame;
    const uint8_t *cs = enc->cs;
    const int32_t *end = pcm + 2 * n_frames;

    while (pcm != end) {
        uint32_t c26 = ((uint32_t)(cs[frame >> 3] >> (frame & 7u)) & 1u) << 26;
        // The level inversion cancels in PRE_B ^ PRE_M.
        uint32_t pre_a = frame == 0 ? pre_m ^ (PRE_B ^ PRE_M) : pre_m;
        encode_subframe(pcm[0], c26, pre_a, m, out);
        encode_subframe(pcm[1], c26, pre_w, m, out + 2);
        pcm += 2;
        out += SPDIF_WORDS_PER_FRAME;
        if (++frame == SPDIF_BLOCK_FRAMES)
            frame = 0;
    }
    enc->frame = frame;
}

int spdif_channel_status_build(uint8_t cs[SPDIF_CS_BYTES], uint32_t sample_rate,
                               uint8_t word_bits) {
    uint8_t rate_code, len_code;

    // Byte 3 bits 0-3, sampling frequency (IEC 60958-3 consumer).
    switch (sample_rate) {
    case 32000: rate_code = 0x3; break;
    case 44100: rate_code = 0x0; break;
    case 48000: rate_code = 0x2; break;
    case 88200: rate_code = 0x8; break;
    case 96000: rate_code = 0xA; break;
    default: return -1;
    }

    // Byte 4 bit 0 selects the 20/24-bit maximum, bits 1-3 the length below it.
    switch (word_bits) {
    case 16: len_code = 0x02; break;   // max 20 bits, 16 bits used
    case 24: len_code = 0x0B; break;   // max 24 bits, 24 bits used
    default: return -1;
    }

    memset(cs, 0, SPDIF_CS_BYTES);
    // Byte 0: bit 0 = 0 consumer, bit 1 = 0 linear PCM, bit 2 = 1 copy
    // permitted (no copyright assertion), bits 3-5 = 0 no pre-emphasis,
    // bits 6-7 = 0 mode 0.
    cs[0] = 0x04;
    // Byte 1 = 0: category general.
    // Byte 2 = 0: source and channel number unspecified.
    // Byte 3 bits 4-5 = 0: clock accuracy level II (+-1000 ppm), which covers
    // the PLL error of every clock setting (see clock/audio_clock.c).
    cs[3] = rate_code;
    cs[4] = len_code;
    return 0;
}
