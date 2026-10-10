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
//   F      frames decoded in sync: 577 per window (three channel status
//          blocks), up to 1154 when the window is stretched to cover a
//          filtered I2S capture (see the I2S report)
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
//
// I2S report (USB string STRID_SELFTEST_I2S, its own index since the
// S/PDIF report can fill the 126-character USB string on its own). Its
// window is captured in the same core-1 iteration as the S/PDIF one and
// carries the same seq:
//
//   #<seq> <state> <fmt> <flt> tx<rate>/<bits> f<F> z<Z> s<S> e<E> L<late>
//          B<bad> G<gaps> K<skip> [@<n>:<exp>><L>/<R>]
//
//   fmt    output format from the format jumpers, i2s_format_name(): i2s,
//          lj, rj16 (rj24 with PICOI2S_RJ24), tda1541a, dual16, dual18,
//          dual20, dual24 (in every state)
//   flt    filter of the output (every state): for the dual-line formats
//          the filter jumpers' i2s_filter_name() with the oversampling of
//          the running output appended for the oversampling filters
//          (sharp8, sharp4, slow8, slow4, minphase8, minphase4: 8 below
//          50 kHz, 4 above; the bare name while the output is stopped),
//          sinc (sin(x)/x compensation at the sample rate) or nos; - for
//          the direct formats
//   state  ok / sil / ERR as above (ERR: framing or sample errors, or fewer
//          frames than the window holds minus a sixteenth)
//   tx     S/PDIF format the I2S output ran with during the window
//   F      frames decoded in sync. The window holds 20 KB of samples, one
//          per BCLK slot: 20480 (Pico 2, 8 pins) or 10240 (RP2350-One, 16
//          pins) slots, i.e. 320 / 160 frames of 64 slots for the direct
//          formats, and 20480 / F (10240 / F) output frames for the
//          dual-line formats, F = 16 .. 32 slots per frame from the
//          generator (e.g. 1280 / 640 for dual16 and tda1541a)
//   Z      silent frames
//   S      sample errors, "-" when no sample check ran. Direct formats and
//          nos: pattern violations (a format narrower than the stream,
//          rj16, tda1541a or dual16 .. dual20 at 24 bits, is checked at its own
//          width); test tone build: no check. Filtered formats (every
//          filter but nos, sinc included): frames that differ from the
//          reference stream, which is the window's S/PDIF samples run
//          through the transmitter's filter, in every build; if the window
//          is not found in the reference at all, every non-silent frame
//          counts
//   E      framing errors: LRCK half frames not F/2 slots, LATCH pulses
//          not 1 slot wide or not F slots apart, samples with BCLK high
//          (S, E capped at 99; F, Z at 9999)
//   late   cumulative blocks that missed the output (i2s_tx_late())
//   bad    cumulative bad windows: ERR, NOSYNC, NOCLK, NOTIM, OFF, PINHI
//   gaps   cumulative windows with silence between pattern frames
//   skip   cumulative windows discarded: as the S/PDIF report, or (the
//          filtered formats) no clean S/PDIF decode to serve as the
//          reference
//          (late, bad, gaps, skip saturate at 99999)
//   @      first sample error, only if S > 0: frame index in the window,
//          previous L (pattern) or expected L (reference; 0 if the window
//          was not found), L and R as 32-bit left-justified words (hex)
//
// Other states:
//   #<seq> idle <fmt> <flt> tx0 lvl<L> B G K    output stopped, all pins low
//   #<seq> PINHI <fmt> <flt> tx0 lvl<L> B G K   output stopped, a pin is high;
//                                               L: DATA, DR, BCLK, LRCK in bits
//                                               0..3 (hex)
//   #<seq> OFF <fmt> <flt> tx<r>/<b> ..         S/PDIF runs, I2S output does not
//   #<seq> NOCLK <fmt> <flt> tx<r>/<b> ..       output running, no BCLK edges
//   #<seq> NOTIM <fmt> <flt> tx<r>/<b> ..       the generator has no timing for
//                                               the format at this rate
//   #<seq> NOSYNC <fmt> <flt> tx<r>/<b> ..      BCLK runs, LRCK never toggles or
//                                               no LATCH pulse
//   #0 NOPIO <fmt> <flt>                        no free PIO SM / DMA channel
//   #0 wait <fmt> <flt> L0 B0 G0 K0             no window yet
//
// Examples, playing 48 kHz / 24-bit:
//   #812 ok i2s - tx48000/24 f320 z0 s0 e0 L0 B0 G0 K1
//   #812 ok dual16 sharp8 tx48000/24 f1280 z0 s0 e0 L0 B0 G0 K1
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "selftest/selftest_decode.h"
#include "selftest/selftest_i2s_decode.h"

// Longest report (USB string descriptor limit).
#define SELFTEST_STRING_MAX 126u

// Frames a window must decode to pass (a window holds at least 577).
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
    SELFTEST_OFF,       // I2S only: S/PDIF runs, the I2S output does not
    SELFTEST_NOCLK,     // I2S only: output running, no BCLK edges
    SELFTEST_PINHI,     // I2S only: output stopped, a pin is high
    SELFTEST_NOTIM,     // I2S only: no timing for the format and rate
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

//--------------------------------------------------------------------+
// I2S report
//--------------------------------------------------------------------+

// Filter token of the I2S report for the direct formats ("-").
#define SELFTEST_FILTER_DIRECT 0xFFu

// Which sample check a window ran.
typedef enum {
    SELFTEST_CHECK_NONE = 0,    // framing only ("s-")
    SELFTEST_CHECK_PATTERN,     // the test pattern
    SELFTEST_CHECK_REFERENCE,   // the filtered S/PDIF samples
} selftest_check_t;

// A window passes with at least this many of the frames it holds.
static inline uint32_t selftest_i2s_min_frames(uint32_t window_frames) {
    return window_frames - window_frames / 16u;
}

typedef struct {
    uint32_t seq;
    uint8_t  state;         // selftest_state_t
    uint8_t  level;         // pin levels while stopped (idle, PINHI)
    uint8_t  check;         // selftest_check_t
    uint8_t  filter;        // i2s_filter_type_t, or SELFTEST_FILTER_DIRECT
    uint8_t  osr;           // output frames per input frame, 0 while stopped
    const char *fmt_name;   // i2s_format_name(), NULL before the first window
    uint32_t tx_rate;       // 0: I2S output stopped
    uint8_t  tx_bits;
    uint32_t frames, silent, err_frame, err_sample;
    uint32_t viol_frame, viol_prev, viol_l, viol_r;
    uint32_t late;                  // i2s_tx_late()
    uint32_t bad, gaps, skipped;    // cumulative
} selftest_i2s_report_t;

// Sets what every report carries: the filter token and the late block
// count. Before each window. filter: the dual-line formats' filter
// (i2s_filter_type_t) or SELFTEST_FILTER_DIRECT; osr: output frames per
// input frame of the running output, 0 while it is stopped.
void selftest_i2s_report_output(selftest_i2s_report_t *rep, unsigned filter, unsigned osr,
                                uint32_t late);

// Records a decoded window of the running output (tx_rate / tx_bits: its
// S/PDIF format). check: the sample check that ran. window_frames: the
// frames the window holds (samples / F).
void selftest_i2s_report_window(selftest_i2s_report_t *rep, const selftest_i2s_decode_t *r,
                                uint32_t tx_rate, uint8_t tx_bits, selftest_check_t check,
                                uint32_t window_frames);

// Records a window without a decode: IDLE or PINHI (output stopped, level =
// the four pin levels), OFF, NOCLK or NOTIM (running at tx_rate / tx_bits).
void selftest_i2s_report_state(selftest_i2s_report_t *rep, selftest_state_t state,
                               uint32_t tx_rate, uint8_t tx_bits, uint8_t level);

void selftest_i2s_report_skipped(selftest_i2s_report_t *rep);

// As selftest_report_format(), for the I2S report.
size_t selftest_i2s_report_format(char *buf, size_t size, const selftest_i2s_report_t *rep);
