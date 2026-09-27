// Crash and hang diagnostics. See diag.h.

#include "diag/diag.h"

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "hardware/structs/watchdog.h"
#include "hardware/structs/systick.h"
#include "hardware/structs/scb.h"

// Watchdog scratch usage (scratch 4..7 belong to the SDK / bootrom).
#define SCR_MAGIC   0   // DIAG_MAGIC in the top half, boot count in the bottom
#define SCR_MARK    1   // last breadcrumb
#define SCR_INFO    2   // panic fmt pointer, fault PC or hang PC
#define SCR_KIND    3   // DIAG_KIND_* in bits 0..7, hang LR in bits 8..31 (>> 8)

#define DIAG_MAGIC      0xD1A60000u
#define DIAG_MAGIC_MASK 0xFFFF0000u

enum {
    DIAG_KIND_NONE = 0,
    DIAG_KIND_PANIC,        // SCR_INFO = panic format string pointer
    DIAG_KIND_HARDFAULT,    // SCR_INFO = faulting PC
    DIAG_KIND_HANG,         // SCR_INFO = interrupted PC, LR in SCR_KIND
    DIAG_KIND_WATCHDOG,     // watchdog reset with no other record
};

// Must exceed the longest legitimate main-loop iteration.
#define DIAG_HANG_MS        300u
#define DIAG_WATCHDOG_MS    1000u
#define SYSTICK_PERIOD_MS   10u

volatile diag_counters_t diag_cnt;

static struct {
    uint32_t boots;
    uint32_t kind;
    uint32_t info;
    uint32_t lr;
    uint32_t mark;
} prev;

static volatile uint32_t heartbeat;
static char report[128];
static char app_status[96];
static uint32_t loop_last_us;
static uint32_t loop_max_us;

void diag_set_app_status(const char *status) {
    strncpy(app_status, status, sizeof(app_status) - 1);
    app_status[sizeof(app_status) - 1] = '\0';
}

static inline io_rw_32 *scr(unsigned i) {
    return &watchdog_hw->scratch[i];
}

void diag_mark(uint32_t mark) {
    *scr(SCR_MARK) = mark;
}

void diag_loop(void) {
    uint32_t now = time_us_32();
    if (loop_last_us != 0 && now - loop_last_us > loop_max_us) {
        loop_max_us = now - loop_last_us;
    }
    loop_last_us = now;
    heartbeat++;
    diag_cnt.loops++;
    watchdog_update();
}

uint32_t diag_loop_max_us(void) {
    return loop_max_us;
}

void diag_loop_max_reset(void) {
    loop_max_us = 0;
}

//--------------------------------------------------------------------+
// Fault capture
//--------------------------------------------------------------------+

// Spin until the watchdog reboots the chip.
static void __attribute__((noreturn)) die(void) {
    while (true) {
        __asm volatile("nop");
    }
}

// Replaces the SDK panic (PICO_PANIC_FUNCTION in CMake). The format string
// is in flash, so its address stays valid across the reboot.
void __attribute__((noreturn)) diag_panic(const char *fmt, ...) {
    *scr(SCR_INFO) = (uint32_t)(uintptr_t)fmt;
    *scr(SCR_KIND) = DIAG_KIND_PANIC;
    die();
}

void __attribute__((used, noreturn)) diag_hardfault_c(uint32_t *frame) {
    *scr(SCR_INFO) = frame[6];      // stacked PC
    *scr(SCR_KIND) = DIAG_KIND_HARDFAULT | (frame[5] & 0xFFFFFF00u);
    die();
}

// Passes the exception frame (MSP or PSP, per EXC_RETURN bit 2) to C.
void __attribute__((naked)) isr_hardfault(void) {
    __asm volatile(
        "movs r0, #4          \n"
        "mov  r1, lr          \n"
        "tst  r0, r1          \n"
        "beq  1f              \n"
        "mrs  r0, psp         \n"
        "b    diag_hardfault_c\n"
        "1:                   \n"
        "mrs  r0, msp         \n"
        "b    diag_hardfault_c\n");
}

//--------------------------------------------------------------------+
// Hang detector (SysTick at highest priority)
//--------------------------------------------------------------------+

void __attribute__((used)) diag_systick_c(uint32_t *frame) {
    static uint32_t last_beat;
    static uint32_t stale_ticks;
    uint32_t beat = heartbeat;
    if (beat != last_beat) {
        last_beat = beat;
        stale_ticks = 0;
        return;
    }
    if (++stale_ticks == DIAG_HANG_MS / SYSTICK_PERIOD_MS &&
        (*scr(SCR_KIND) & 0xFFu) == DIAG_KIND_NONE) {
        // Record where the main loop is stuck; the watchdog then reboots.
        *scr(SCR_INFO) = frame[6];
        *scr(SCR_KIND) = DIAG_KIND_HANG | (frame[5] & 0xFFFFFF00u);
    }
}

void __attribute__((naked)) isr_systick(void) {
    __asm volatile(
        "movs r0, #4          \n"
        "mov  r1, lr          \n"
        "tst  r0, r1          \n"
        "beq  1f              \n"
        "mrs  r0, psp         \n"
        "b    diag_systick_c  \n"
        "1:                   \n"
        "mrs  r0, msp         \n"
        "b    diag_systick_c  \n");
}

// The reload depends on clk_sys: call again after changing clk_sys.
static void systick_start(void) {
    uint32_t reload = clock_get_hz(clk_sys) / (1000u / SYSTICK_PERIOD_MS) - 1u;
    systick_hw->csr = 0;
    systick_hw->rvr = reload & 0x00FFFFFFu;
    systick_hw->cvr = 0;
    // Highest priority (SHPR3[31:24] = 0) so a hang inside an IRQ is caught.
    scb_hw->shpr[2] &= 0x00FFFFFFu;
    systick_hw->csr = M33_SYST_CSR_CLKSOURCE_BITS | M33_SYST_CSR_TICKINT_BITS |
                      M33_SYST_CSR_ENABLE_BITS;
}

//--------------------------------------------------------------------+
// Init and report
//--------------------------------------------------------------------+

void diag_init(void) {
    uint32_t magic = *scr(SCR_MAGIC);
    bool valid = (magic & DIAG_MAGIC_MASK) == DIAG_MAGIC;

    prev.boots = valid ? (magic & 0xFFFFu) : 0;
    if (valid) {
        uint32_t k = *scr(SCR_KIND);
        prev.kind = k & 0xFFu;
        prev.lr   = k & 0xFFFFFF00u;
        prev.info = *scr(SCR_INFO);
        prev.mark = *scr(SCR_MARK);
        if (prev.kind == DIAG_KIND_NONE && watchdog_caused_reboot()) {
            prev.kind = DIAG_KIND_WATCHDOG;
        }
    }

    *scr(SCR_MAGIC) = DIAG_MAGIC | ((prev.boots + 1u) & 0xFFFFu);
    *scr(SCR_KIND)  = DIAG_KIND_NONE;
    *scr(SCR_INFO)  = 0;
    diag_mark(DIAG_MARK_BOOT);

    systick_start();
    watchdog_enable(DIAG_WATCHDOG_MS, true);
}

void diag_clock_changed(void) {
    systick_start();
}

static bool in_flash(uint32_t p) {
    return p >= 0x10000000u && p < 0x10400000u;
}

const char *diag_string(void) {
    static const char *const kinds[] = {"none", "panic", "fault", "hang", "wdog"};
    const char *k = prev.kind < 5 ? kinds[prev.kind] : "?";
    int n;
    if (prev.kind == DIAG_KIND_NONE) {
        n = snprintf(report, sizeof(report), "b%lu %s | %s",
                     (unsigned long)prev.boots, k, app_status);
    } else {
        n = snprintf(report, sizeof(report), "b%lu %s m%lu pc%08lx lr%08lx | %s",
                     (unsigned long)prev.boots, k, (unsigned long)prev.mark,
                     (unsigned long)prev.info, (unsigned long)prev.lr, app_status);
    }
    if (prev.kind == DIAG_KIND_PANIC && in_flash(prev.info) && n > 0 &&
        (size_t)n < sizeof(report) - 4) {
        snprintf(report + n, sizeof(report) - (size_t)n, " '%s'", (const char *)prev.info);
    }
    return report;
}
