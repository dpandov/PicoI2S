// Crash and hang diagnostics without a debug probe or UART.
//
// A watchdog reboots on a hang; a highest-priority SysTick records the PC/LR
// where the main loop stalled; panic() and HardFault record their location.
// Records survive the reboot in watchdog scratch 0..3 (the SDK uses 4..7).
// diag_string() is served as USB string STRID_DIAG, read by tools/diag.py.
#pragma once

#include <stdint.h>

// Hidden string index, not referenced by any descriptor.
#define STRID_DIAG  0x20

// Breadcrumbs: where the main loop / callbacks were last seen.
enum {
    DIAG_MARK_BOOT = 1,
    DIAG_MARK_TUD_TASK,
    DIAG_MARK_SINK,
    DIAG_MARK_LED,
    DIAG_MARK_LOG,
    DIAG_MARK_SET_ITF,
    DIAG_MARK_SET_ITF_DONE,
    DIAG_MARK_CLOSE_EP,
    DIAG_MARK_FB_PARAMS,
    DIAG_MARK_CLOCK_REQ,
};

// Live counters, incremented by the code they describe.
typedef struct {
    uint32_t loops;         // main loop iterations
    uint32_t set_itf;       // SET_INTERFACE callbacks
    uint32_t clock_req;     // clock entity requests
} diag_counters_t;

extern volatile diag_counters_t diag_cnt;

// Call first thing in main(). Captures the previous-boot record, then arms
// the watchdog and the SysTick hang detector.
void diag_init(void);

// Call once per main-loop iteration: feeds the watchdog and the heartbeat.
void diag_loop(void);

// Record a breadcrumb (a single register write, safe anywhere).
void diag_mark(uint32_t mark);

// Application status appended to the report (copied, truncated to 48 chars).
// Call from the main loop only.
void diag_set_app_status(const char *status);

// Report text, at most 126 characters (USB string limit).
const char *diag_string(void);
