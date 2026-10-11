// Output self-test (build option PICOI2S_SELFTEST).
//
// A PIO state machine and a DMA channel sample SPDIF_PIN, and a second
// pair captures the I2S / dual-line pins once per BCLK slot. The S/PDIF
// line is decoded bit-exactly and checked against the transmitter's
// format and the test pattern (selftest_pattern.h); the I2S stream is
// framed per the jumper format with the generator's timing
// (i2s/i2s_format.h) and checked against the pattern (direct formats, NOS)
// or, for the filtered dual-line formats (the oversampling filters and the
// sinc compensation), against the window's S/PDIF samples run through the
// transmitter's filter (selftest_i2s_expect.h).
// Capture windows that overlap a format or clk_sys change, a volume ramp or
// a source-switch fade are discarded.
// The results of the last windows are served as USB strings STRID_SELFTEST
// (S/PDIF) and STRID_SELFTEST_I2S, formats in selftest_report.h, read by
// tools/selftest.py.
//
// The work runs on core 1 as a cooperative task beside the dual-line
// filter: selftest_task() starts captures, polls them and decodes in
// bounded steps, returning within about 100 us. It uses no interrupts and
// does not touch the watchdog, the transmitters' resources or the pin
// configuration beyond enabling the inputs.
#pragma once

#include <stdint.h>

// Hidden string indices, not referenced by any descriptor (next to
// STRID_DIAG).
#define STRID_SELFTEST     0x21
#define STRID_SELFTEST_I2S 0x22

// Claims two PIO state machines and two DMA channels. Call once on core 0,
// after spdif_tx_init() and i2s_tx_init(), before core 1 starts. If the
// resources are not available, the reports say NOPIO and selftest_task()
// does nothing.
void selftest_init(void);

// One step of the self-test; call repeatedly from the core-1 loop.
void selftest_task(void);

// Longest selftest_task() call so far, in microseconds.
uint32_t selftest_task_max_us(void);

// Reports of the last window, at most SELFTEST_STRING_MAX characters. Core 0.
const char *selftest_string(void);
const char *selftest_i2s_string(void);
