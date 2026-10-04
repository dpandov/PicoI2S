// Self-test window classification and report string. Portable C, shared
// by the firmware and the host tests.
//
// Report (USB string STRID_SELFTEST, read by tools/selftest.py), one
// decoded capture window at a time:
//
//   #<seq> <state> tx<rate>/<bits> f<F> z<Z> e<E> p<P> o<O> v<V> s<S>
//          c<C><cs3><cs4> fs<Hz> B<bad> G<gaps> K<skip> [@<n>:<prev>><L>/<R>]
//
//   seq    windows checked since boot, mod 10^6
//   state  ok    clean, every frame carries the pattern
//          sil   clean, some frames silent (L = R = 0); with the pattern
//                check off: all frames silent
//          ERR   errors below, channel status mismatch / no complete block,
//                or fewer than SELFTEST_MIN_FRAMES frames
//   tx     transmitter format the window was checked against
//   F      frames decoded in sync (~577 per window)
//   Z      silent frames
//   E      cell errors: runs not 1..3 cells, missing slot transitions, bad
//          preambles, sync losses
//   P      parity errors      O  order errors (M/W alternation, B spacing)
//   V      validity or user bit set
//   S      pattern violations; "-" when the pattern check is off (test tone
//          build: the output is not the pattern)
//          (E, P, O, V, S capped at 99; F, Z at 9999)
//   C      1 every complete channel status block equals the expected one
//          for the tx format, 0 differs, - no complete block, x the C bit
//          differs between subframes
//   cs3    decoded channel status byte 3, hex: rate code (00 44.1k, 02 48k,
//          03 32k, 08 88.2k, 0A 96k)
//   cs4    byte 4: word length (02 16-bit, 0B 24-bit)
//   Hz     measured frame rate: clk_sys by the frequency counter (against
//          the crystal) / capture divider x cells / samples / 128
//   bad    cumulative bad windows: ERR, NOSYNC, NOCARR, CARRIER, CAPTMO
//   gaps   cumulative windows with silence between pattern frames (dropouts)
//   skip   cumulative windows discarded because the transmitter format or
//          clk_sys changed during the capture
//          (bad, gaps, skip saturate at 99999)
//   @      first pattern violation, only if S > 0: frame index in the
//          window, previous L, L and R (hex)
//
// Other states:
//   #<seq> idle tx0 lvl<L> B G K          output stopped, line static at L
//   #<seq> CARRIER tx0 r<runs> B G K      output stopped, but the line toggles
//   #<seq> NOCARR tx<r>/<b> lvl<L> B G K  output running, line static
//   #<seq> NOSYNC tx<r>/<b> r<runs> q<samples per cell x100> B G K
//   #<seq> CAPTMO tx.. B G K              capture DMA did not complete
//   #0 NOPIO                              no free PIO SM / DMA channel
//   #0 wait B0 G0 K0                      no window yet
// Lower-case states are healthy, upper-case states are failures.
//
// Example, playing 48 kHz / 24-bit:
//   #812 ok tx48000/24 f577 z0 e0 p0 o0 v0 s0 c1020B fs47991 B2 G0 K1
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "selftest/selftest_decode.h"

// Longest report (USB string descriptor limit).
#define SELFTEST_STRING_MAX 126u

// Frames a window must decode to pass (a window holds ~577).
#define SELFTEST_MIN_FRAMES 500u

typedef enum {
    SELFTEST_WAIT = 0,
    SELFTEST_OK,
    SELFTEST_SIL,
    SELFTEST_ERR,
    SELFTEST_NOSYNC,
    SELFTEST_NOCARR,
    SELFTEST_IDLE,
    SELFTEST_CARRIER,
    SELFTEST_CAPTMO,
    SELFTEST_NOPIO,
} selftest_state_t;

typedef struct {
    uint32_t seq;
    uint8_t  state;         // selftest_state_t
    uint8_t  level;         // static line level (idle, NOCARR)
    bool     pattern;       // pattern check on
    char     cs_flag;
    uint8_t  cs3, cs4;
    uint32_t tx_rate;       // 0: output stopped
    uint8_t  tx_bits;
    uint32_t frames, silent;
    uint32_t err_cell, err_parity, err_order, err_vu, err_sample;
    uint32_t runs, ovs_x100;
    uint32_t fs_hz;
    uint32_t viol_frame, viol_prev, viol_l, viol_r;
    uint32_t bad, gaps, skipped;    // cumulative
} selftest_report_t;

// Records a decoded window. tx_rate / tx_bits: the transmitter format for
// the whole window (tx_rate 0 if stopped). pattern: the pattern check was
// on. The frame rate is clk_hz / cap_div x cells / samples / 128.
void selftest_report_window(selftest_report_t *rep, const selftest_decode_t *r,
                            uint32_t tx_rate, uint8_t tx_bits, bool pattern,
                            uint32_t clk_hz, uint32_t cap_div);

// Records a window whose capture did not complete.
void selftest_report_timeout(selftest_report_t *rep, uint32_t tx_rate, uint8_t tx_bits);

// Records a window discarded because of a format or clock change.
void selftest_report_skipped(selftest_report_t *rep);

bool selftest_state_is_bad(selftest_state_t s);

// Formats the report (at most SELFTEST_STRING_MAX characters, truncated to
// size - 1). Returns the length.
size_t selftest_report_format(char *buf, size_t size, const selftest_report_t *rep);
