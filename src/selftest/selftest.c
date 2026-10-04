// S/PDIF output self-test, see selftest.h.

#include "selftest/selftest.h"

#include <string.h>

#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/structs/pads_bank0.h"

#include "board_config.h"
#include "clock/audio_clock.h"
#include "spdif/spdif_encode.h"
#include "spdif/spdif_tx.h"
#include "selftest/selftest_decode.h"
#include "selftest/selftest_report.h"
#include "selftest_capture.pio.h"

_Static_assert(SPDIF_CS_BYTES == SELFTEST_CS_BYTES, "channel status size");

// The test tone is not the pattern: check framing and channel status only.
#if USB2SPDIF_TEST_TONE
#define PATTERN_CHECK false
#else
#define PATTERN_CHECK true
#endif

// Capture divider: the smallest integer that keeps the oversampling at or
// below MAX_OVS samples per cell (tx divider 5..36 -> 5..9).
#define MAX_OVS         9u
// Three blocks plus two frames: at least two complete blocks at any phase.
#define WINDOW_CELLS    (3u * 192u * 128u + 256u)
#define MAX_WORDS       ((WINDOW_CELLS * MAX_OVS + 31u) / 32u)     // 81 KB
// Output stopped: 131072 samples at clk_sys / 16 (~14 ms), every ~25 ms.
#define IDLE_DIV        16u
#define IDLE_WORDS      4096u
#define IDLE_PAUSE_US   10000u
// A window is discarded if the measured clk_sys is off the configured one
// by more than this (per mille): a clock change outside audio_clock.
#define CLK_TOLERANCE_PERMILLE 5u

static uint32_t cap_buf[MAX_WORDS];
static PIO  cap_pio;
static uint cap_sm;
static uint cap_offset;
static uint cap_dma;

//--------------------------------------------------------------------+
// Result publication: core 1 writes, core 0 reads (sequence lock)
//--------------------------------------------------------------------+

static volatile uint32_t pub_lock;      // odd while core 1 writes
static selftest_report_t pub;

static void publish(const selftest_report_t *rep) {
    pub_lock++;
    __dmb();
    memcpy(&pub, rep, sizeof(pub));
    __dmb();
    pub_lock++;
}

// A consistent copy of the last report; retries while core 1 is writing.
static void read_published(selftest_report_t *out) {
    for (int tries = 0; tries < 1000; tries++) {
        uint32_t s = pub_lock;
        __dmb();
        if (s & 1u) {
            continue;
        }
        memcpy(out, &pub, sizeof(*out));
        __dmb();
        if (pub_lock == s) {
            return;
        }
    }
    memcpy(out, &pub, sizeof(*out));
}

//--------------------------------------------------------------------+
// Capture
//--------------------------------------------------------------------+

static bool claim_on(PIO p, uint pin) {
    uint base = pio_get_gpio_base(p);
    if (pin < base || pin >= base + 32u || !pio_can_add_program(p, &selftest_capture_program)) {
        return false;
    }
    int sm = pio_claim_unused_sm(p, false);
    if (sm < 0) {
        return false;
    }
    cap_pio = p;
    cap_sm = (uint)sm;
    cap_offset = (uint)pio_add_program(p, &selftest_capture_program);
    return true;
}

// Any PIO block can read any pad. Prefer a block other than the one
// driving the pin, so the transmitter's block is left alone.
static bool claim_pio(uint pin) {
    static const PIO pios[] = {
#if NUM_PIOS > 2
        pio2,
#endif
        pio1, pio0,
    };
    uint tx_func = (uint)gpio_get_function(pin);
    for (int pass = 0; pass < 2; pass++) {
        for (unsigned i = 0; i < count_of(pios); i++) {
            bool tx_block = pio_get_funcsel(pios[i]) == tx_func;
            if (tx_block == (pass == 0)) {
                continue;
            }
            if (claim_on(pios[i], pin)) {
                return true;
            }
        }
    }
    return false;
}

static bool capture_init(uint pin) {
    if (!claim_pio(pin)) {
        return false;
    }
    int ch = dma_claim_unused_channel(false);
    if (ch < 0) {
        pio_remove_program(cap_pio, &selftest_capture_program, cap_offset);
        pio_sm_unclaim(cap_pio, cap_sm);
        return false;
    }
    cap_dma = (uint)ch;

    // Input enabled, isolation off. The transmitter's pio_gpio_init() has
    // done this already; FUNCSEL and the output settings stay its own.
    hw_set_bits(&pads_bank0_hw->io[pin], PADS_BANK0_GPIO0_IE_BITS);
    hw_clear_bits(&pads_bank0_hw->io[pin], PADS_BANK0_GPIO0_ISO_BITS);

    // Read only: no pio_gpio_init(), no pindirs.
    pio_sm_config c = selftest_capture_program_get_default_config(cap_offset);
    sm_config_set_in_pins(&c, pin);
    sm_config_set_in_pin_count(&c, 1);
    sm_config_set_in_shift(&c, false, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv_int_frac8(&c, IDLE_DIV, 0);
    pio_sm_init(cap_pio, cap_sm, cap_offset, &c);   // left disabled

    dma_channel_config dc = dma_channel_get_default_config(cap_dma);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, false);
    channel_config_set_write_increment(&dc, true);
    channel_config_set_dreq(&dc, pio_get_dreq(cap_pio, cap_sm, false));
    channel_config_set_irq_quiet(&dc, true);
    dma_channel_configure(cap_dma, &dc, cap_buf, &cap_pio->rxf[cap_sm], 0, false);
    return true;
}

// Captures n_words at clk_sys / div, polled. Returns false on timeout.
static __noinline bool __not_in_flash_func(capture)(uint32_t div, uint32_t n_words,
                                                    uint32_t timeout_us) {
    pio_sm_set_enabled(cap_pio, cap_sm, false);
    pio_sm_set_clkdiv_int_frac8(cap_pio, cap_sm, (uint16_t)div, 0);
    pio_sm_clear_fifos(cap_pio, cap_sm);
    pio_sm_restart(cap_pio, cap_sm);
    pio_sm_clkdiv_restart(cap_pio, cap_sm);
    pio_sm_exec(cap_pio, cap_sm, pio_encode_jmp(cap_offset));

    dma_channel_set_write_addr(cap_dma, cap_buf, false);
    dma_channel_set_trans_count(cap_dma, n_words, true);
    pio_sm_set_enabled(cap_pio, cap_sm, true);

    uint32_t t0 = time_us_32();
    bool done = true;
    while (dma_channel_is_busy(cap_dma)) {
        if (time_us_32() - t0 > timeout_us) {
            done = false;
            break;
        }
    }
    pio_sm_set_enabled(cap_pio, cap_sm, false);
    if (!done) {
        dma_channel_abort(cap_dma);
    }
    return done;
}

//--------------------------------------------------------------------+
// Core 1
//--------------------------------------------------------------------+

static uint32_t tx_rate_now(void) {
    return spdif_tx_running() ? spdif_tx_sample_rate() : 0u;
}

static void core1_main(void) {
    static selftest_report_t rep;
    static selftest_decode_t dec;
    uint8_t cs[SELFTEST_CS_BYTES];

    for (;;) {
        // Generations are odd while a change is in progress.
        uint32_t g_tx = spdif_tx_generation();
        uint32_t g_clk = audio_clock_generation();
        if ((g_tx | g_clk) & 1u) {
            busy_wait_us(100);
            continue;
        }
        __dmb();
        uint32_t rate = tx_rate_now();
        uint8_t bits = spdif_tx_word_bits();
        uint32_t clk = clock_get_hz(clk_sys);

        uint32_t div = IDLE_DIV, words = IDLE_WORDS;
        if (rate) {
            uint32_t dtx = (clk + 64u * rate) / (128u * rate);
            div = (dtx + MAX_OVS - 1u) / MAX_OVS;
            if (div < 1u) {
                div = 1u;
            }
            words = (uint32_t)(((uint64_t)WINDOW_CELLS * dtx / div + 31u) / 32u);
            if (words > MAX_WORDS) {
                words = MAX_WORDS;
            }
        }
        uint32_t expect_us = (uint32_t)((uint64_t)words * 32u * div * 1000000u / clk);
        bool done = capture(div, words, 3u * expect_us + 20000u);
        uint32_t fc_hz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) * 1000u;

        __dmb();
        bool changed = spdif_tx_generation() != g_tx || audio_clock_generation() != g_clk ||
                       tx_rate_now() != rate || spdif_tx_word_bits() != bits ||
                       clock_get_hz(clk_sys) != clk;
        uint64_t fc_k = (uint64_t)fc_hz * 1000u;
        if (fc_k < (uint64_t)clk * (1000u - CLK_TOLERANCE_PERMILLE) ||
            fc_k > (uint64_t)clk * (1000u + CLK_TOLERANCE_PERMILLE)) {
            changed = true;
        }
        if (changed) {
            selftest_report_skipped(&rep);
            publish(&rep);
            continue;
        }
        if (!done) {
            selftest_report_timeout(&rep, rate, bits);
            publish(&rep);
            continue;
        }

        if (!rate || spdif_channel_status_build(cs, rate, bits) != 0) {
            memset(cs, 0, sizeof(cs));
        }
        bool pattern = PATTERN_CHECK && rate != 0;
        selftest_decode(cap_buf, words, cs, pattern ? bits : 0u, &dec);
        selftest_report_window(&rep, &dec, rate, bits, pattern, fc_hz, div);
        publish(&rep);
        if (!rate) {
            busy_wait_us(IDLE_PAUSE_US);
        }
    }
}

//--------------------------------------------------------------------+
// Core 0
//--------------------------------------------------------------------+

void selftest_init(void) {
    if (!capture_init(SPDIF_PIN)) {
        selftest_report_t rep = {.state = SELFTEST_NOPIO};
        publish(&rep);
        return;
    }
    multicore_launch_core1(core1_main);
}

const char *selftest_string(void) {
    static char buf[SELFTEST_STRING_MAX + 1u];
    selftest_report_t rep;
    read_published(&rep);
    selftest_report_format(buf, sizeof(buf), &rep);
    return buf;
}
