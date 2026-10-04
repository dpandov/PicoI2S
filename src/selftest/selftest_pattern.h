// Self-test pattern, as played by tools/wasapi_play.c and checked on the
// S/PDIF line by the decoder.
//
// Frame n, as sent over USB:
//   24-bit: l = (n * SELFTEST_STEP24) mod 2^24, r = ~l (24 bits), in 32-bit
//           containers with the value in bits 31..8, low byte zero.
//   16-bit: l = (n * SELFTEST_STEP16) mod 2^16, r = ~l (16 bits), int16.
// On the line (24-bit S/PDIF sample values) the 16-bit pattern lands in
// bits 23..8 with a zero low byte.
//
// The steps are odd, so every bit toggles over time. The pattern never
// produces l = r = 0, so such a frame is silence: counted, never an error.
// The check follows the output word length: per frame r against l (and a
// zero low byte at 16 bits), between consecutive non-silent frames the step
// of l. It needs no absolute phase, and starts and stops against silence
// are not errors.
#pragma once

#include <stdint.h>

#define SELFTEST_STEP24 0x0B5A37u
#define SELFTEST_STEP16 0x5A37u
#define SELFTEST_MASK24 0x00FFFFFFu

// Pattern frame n as 24-bit S/PDIF sample values; bits is 16 or 24.
static inline void selftest_pattern_frame(uint32_t n, unsigned bits, uint32_t *l, uint32_t *r) {
    if (bits == 16) {
        *l = ((n * SELFTEST_STEP16) & 0xFFFFu) << 8;
        *r = ~*l & 0x00FFFF00u;
    } else {
        *l = (n * SELFTEST_STEP24) & SELFTEST_MASK24;
        *r = ~*l & SELFTEST_MASK24;
    }
}
