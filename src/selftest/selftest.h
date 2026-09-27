// S/PDIF output self-test (build option USB2SPDIF_SELFTEST).
//
// Core 1 samples SPDIF_PIN with its own PIO state machine and DMA channel,
// decodes the line bit-exactly and checks it against the transmitter's
// format and the test pattern (selftest_pattern.h). Capture windows that
// overlap a format or clk_sys change are discarded. The result of the last
// window is served as USB string STRID_SELFTEST, format in
// selftest_report.h, read by tools/selftest.py.
//
// Core 1 uses no interrupts and does not touch the watchdog, the
// transmitter's resources or the pin configuration beyond enabling its
// input.
#pragma once

// Hidden string index, not referenced by any descriptor (next to STRID_DIAG).
#define STRID_SELFTEST 0x21

// Claims a PIO state machine and a DMA channel, then starts core 1. Call
// once, after spdif_tx_init(). If the resources are not available, the
// report says NOPIO and core 1 is not started.
void selftest_init(void);

// Report of the last window, at most SELFTEST_STRING_MAX characters. Core 0.
const char *selftest_string(void);
