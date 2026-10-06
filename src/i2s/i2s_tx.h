// I2S / dual-line output, run in lockstep with the S/PDIF transmitter.
//
// One PIO state machine on the S/PDIF transmitter's PIO block generates
// BCLK, LRCK or LATCH and the data line(s) from a ring of four DMA
// buffers: the S/PDIF DMA interrupt hands every block of frames it fetched
// to i2s_tx through the spdif_tx companion interface (spdif_tx.h), the
// state machines are enabled in one write and consume one input frame per
// the same number of clk_sys cycles, so the two outputs carry the same
// samples and stop together. The trim (UAC1 mode) is applied to both.
// There is no MCLK.
//
// I2S, left- and right-justified go out at the sample rate, converted in
// the interrupt. The dual-line formats (multibit DACs) take one of five
// filters, selected by the filter jumpers (i2s_filter.h): sharp, slow and
// minimum phase oversample 8x up to 48 kHz and 4x above (output word rate
// 256..384 kHz); the sinc compensation stays at the sample rate (a short
// FIR that flattens the DAC's sin(x)/x hold droop, at -4 dB); NOS passes
// the samples through at the sample rate. All but NOS run on core 1, so
// that output lags S/PDIF by one block plus the filter's group delay; NOS
// is converted in the interrupt like the single-line formats. In dual16
// (TDA1541A) a third state machine drives the DEM clock on I2S_DEM_PIN: a
// 50 % square wave at the oversampled word rate (8x Fs below 50 kHz, 4x
// above) with every filter, NOS included, phase-locked to LATCH; the pin
// is low in every other format.
//
// Timing (i2s_format.h): an output frame is F BCLK slots of N state
// machine cycles at divider D, with F x N x D = 128 x div / osr clk_sys
// cycles (div: the S/PDIF divider), selected per rate and format at start.
#pragma once

#include <stdbool.h>
#include <stdint.h>

// Selected by the three format jumpers (board_config.h): the code is the
// enum value, grounded pin = 1.
typedef enum {
    I2S_FORMAT_I2S = 0,             // 000: Philips I2S, data one BCLK after the LRCK edge,
                                    //      LRCK low = left, 32-bit slots
    I2S_FORMAT_LEFT_JUSTIFIED,      // 001: MSB at the LRCK edge, LRCK high = left
    I2S_FORMAT_RIGHT_JUSTIFIED_24,  // 010: LSB at the end of the slot, 24 bits
    I2S_FORMAT_RIGHT_JUSTIFIED_16,  // 011: 16 bits
    I2S_FORMAT_DUAL_16,             // 100: DL + DR, common BCLK and LATCH, 16 bits (TDA1541 simultaneous)
    I2S_FORMAT_DUAL_18,             // 101: 18 bits (AD1865)
    I2S_FORMAT_DUAL_20,             // 110: 20 bits (PCM63, PCM1702)
    I2S_FORMAT_DUAL_24,             // 111: 24 bits
} i2s_format_t;

#define I2S_FORMAT_COUNT 8u

static inline bool i2s_format_is_dual(i2s_format_t f) {
    return f >= I2S_FORMAT_DUAL_16;
}

// Filter for the dual-line formats, selected by the three filter jumpers
// (board_config.h): the code is the enum value, grounded pin = 1; all
// open is NOS (no filter). Codes 101..111 are reserved and select the
// sharp filter. Responses in i2s/i2s_filter.h.
typedef enum {
    I2S_FILTER_NOS = 0,         // 000: none, the samples at the sample rate
    I2S_FILTER_SHARP,           // 001: linear phase, flat to 0.4535 Fs (20 kHz at 44.1)
    I2S_FILTER_SLOW,            // 010: linear phase, slow roll-off
    I2S_FILTER_NOS_SINC,        // 011: sample rate, sin(x)/x compensation of the DAC's hold, -4 dB
    I2S_FILTER_MINPHASE,        // 100: the slow magnitude, minimum phase (no pre-ringing)
} i2s_filter_type_t;

#define I2S_FILTER_COUNT 5u

// Output frames per input frame for a filter: 8 below 50 kHz and 4 above
// for the oversampling filters, 1 for NOS and the sinc compensation.
static inline unsigned i2s_filter_osr(i2s_filter_type_t type, uint32_t rate) {
    if (type == I2S_FILTER_SHARP || type == I2S_FILTER_SLOW || type == I2S_FILTER_MINPHASE)
        return rate < 50000u ? 8u : 4u;
    return 1u;
}

// Jumper code to filter: reserved codes select the sharp filter.
static inline i2s_filter_type_t i2s_filter_from_code(unsigned code) {
    return code < I2S_FILTER_COUNT ? (i2s_filter_type_t)code : I2S_FILTER_SHARP;
}

// Short name of a filter for reports: "nos", "sharp", "slow", "sinc",
// "minphase".
static inline const char *i2s_filter_name(i2s_filter_type_t type) {
    static const char *const names[I2S_FILTER_COUNT] = {"nos", "sharp", "slow", "sinc", "minphase"};
    return (unsigned)type < I2S_FILTER_COUNT ? names[type] : "?";
}

// Claims the I2S and DEM state machines on spdif_tx_pio() and two DMA
// channels, drives all output pins low, and registers itself as the
// S/PDIF transmitter's companion. filter: from the filter jumpers, used by
// the dual-line formats only. Call once, after spdif_tx_init() and before
// the first spdif_tx_start() and before core 1 is launched. From then on
// the output follows the S/PDIF one.
void i2s_tx_init(i2s_format_t format, i2s_filter_type_t filter);

i2s_format_t i2s_tx_format(void);

// The filter the dual-line formats use (the jumper setting; NOS for the
// I2S / LJ / RJ formats, which are never filtered).
i2s_filter_type_t i2s_tx_filter(void);

// True between the S/PDIF start and stop that carried this output along.
bool i2s_tx_running(void);

// Output frames per input frame of the running output: 8 or 4 for the
// oversampling filters, 1 otherwise (NOS, sinc) and when stopped.
unsigned i2s_tx_oversampling(void);

// Blocks that missed the output since boot: the DMA had reached their
// buffer before they were written (the buffer then went out again with
// its previous content), or the mailbox to core 1 was full. Never reset.
uint32_t i2s_tx_late(void);

// Core 1: filters and packs at most one pending block of the filtered
// path per call, returns at once when there is none. Call from the core-1 loop; cooperative with other core-1
// tasks.
void i2s_tx_core1_task(void);

// Short name of a format for reports: "i2s", "lj", "rj24", "rj16",
// "dual16", "dual18", "dual20", "dual24".
const char *i2s_format_name(i2s_format_t f);
