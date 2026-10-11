// Output self-test, see selftest.h.

#include "selftest/selftest.h"

#include <string.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/structs/clocks.h"
#include "hardware/structs/pads_bank0.h"

#include "board_config.h"
#include "app/audio_pipeline.h"
#include "clock/audio_clock.h"
#include "i2s/i2s_format.h"
#include "i2s/i2s_tx.h"
#include "spdif/spdif_encode.h"
#include "spdif/spdif_tx.h"
#include "selftest/selftest_decode.h"
#include "selftest/selftest_i2s_decode.h"
#include "selftest/selftest_i2s_expect.h"
#include "selftest/selftest_report.h"
#include "selftest_capture.pio.h"

_Static_assert(SPDIF_CS_BYTES == SELFTEST_CS_BYTES, "channel status size");
_Static_assert(I2S_DATA_R_PIN == I2S_DATA_PIN + 1 && I2S_LRCK_PIN == I2S_BCLK_PIN + 1,
               "I2S capture pin pairs must be consecutive");
_Static_assert(I2S_BCLK_PIN > I2S_DATA_PIN + 1 && I2S_LRCK_PIN - I2S_DATA_PIN < 16,
               "I2S pins must lie within 16 pins above I2S_DATA_PIN");

// The test tone is not the pattern: check framing and channel status only.
#if PICOI2S_TEST_TONE
#define PATTERN_CHECK false
#else
#define PATTERN_CHECK true
#endif

//--------------------------------------------------------------------+
// S/PDIF capture
//--------------------------------------------------------------------+

// Capture divider: the smallest integer that keeps the oversampling at or
// below MAX_OVS samples per cell (tx divider 5..36 -> 5..9), or that fits
// a stretched window into the buffer (then 4 .. 4.5 samples per cell; the
// decoder works from about 3).
#define MAX_OVS         9u
#define BLOCK_FRAMES    192u
// Three blocks plus two frames: at least two complete blocks at any phase.
// Stretched (filtered I2S check, below) to at most six blocks.
#define MIN_BLOCKS      3u
#define MAX_BLOCKS      6u
#define EXTRA_FRAMES    2u
#define MIN_WINDOW_CELLS ((MIN_BLOCKS * BLOCK_FRAMES + EXTRA_FRAMES) * 128u)
#define MAX_FRAMES      (MAX_BLOCKS * BLOCK_FRAMES + EXTRA_FRAMES)         // 1154
#define MAX_WORDS       ((MIN_WINDOW_CELLS * MAX_OVS + 31u) / 32u)        // 81 KB
_Static_assert(MAX_FRAMES * 128u <= SELFTEST_DECODE_MAX_CELLS, "window exceeds the cell buffer");
// Output stopped: 131072 samples at clk_sys / 16 (~14 ms), every ~25 ms.
#define IDLE_DIV        16u
#define IDLE_WORDS      4096u
#define IDLE_PAUSE_US   10000u
// A window is discarded if the measured clk_sys is off the configured one
// by more than this (per mille): a clock change outside audio_clock.
#define CLK_TOLERANCE_PERMILLE 5u
// A window is discarded if the attenuation changed this recently: a ramp
// or fade is not bit-exact. Covers the window, the ring and the filters.
#define RAMP_HOLD_MS    100u

//--------------------------------------------------------------------+
// I2S capture
//--------------------------------------------------------------------+

// One sample per BCLK slot of W = 8 (Pico 2: GP16..23) or 16 (RP2350-One:
// GP14..29) pins from I2S_DATA_PIN, 32 / W per word; 20 KB of buffer hold
// 20480 or 10240 slots: 320 / 160 frames of the direct formats, 20480 / F
// (10240 / F) output frames of the dual-line formats (1280 / 640 at
// F = 16).
#define I2S_SAMPLE_BITS ((I2S_LRCK_PIN - I2S_DATA_PIN) < 8u ? 8u : 16u)
#define I2S_CLK_BIT     (I2S_BCLK_PIN - I2S_DATA_PIN)
#define I2S_WORDS       5120u                                            // 20 KB
#define I2S_SAMPLES     (I2S_WORDS * 32u / I2S_SAMPLE_BITS)
#define I2S_MAX_FRAMES  (I2S_SAMPLES / 16u)                              // frames at F = 16

// The capture program: wait 1 pin b; wait 0 pin b; in pins, W (b = the
// BCLK index above the IN base). It samples right after every BCLK
// falling edge, where the transmitter changes the data and the LRCK /
// LATCH line: the sample holds the slot's data (stable until the DAC
// clocks it on the rising edge) and the line level of the slot's BCLK-low
// half, which is where the TDA1541A LATCH pulse lives; the other
// dual-line formats hold LATCH through both halves of their LATCH slots
// and into slot 0's low half (one more sample), so one sample per slot
// frames both styles. The wait sees the edge two cycles late (input
// synchroniser) and the in executes one cycle after (so it reads the pins
// as they were one cycle after the edge), so the loop needs N x D >= 4
// clk_sys cycles per slot and a BCLK-low half of at least 2; the transmitter's smallest timing is N x D = 9 with a
// 4-cycle low half (i2s/i2s_format.h). The pins are only read, never
// claimed.
#define I2S_CAPTURE_LEN     3u
#define I2S_CAPTURE_IE_MASK ((3u << I2S_DATA_PIN) | (3u << I2S_BCLK_PIN))

static uint16_t i2s_capture_instr[I2S_CAPTURE_LEN];
static const pio_program_t i2s_capture_program = {
    .instructions = i2s_capture_instr,
    .length = I2S_CAPTURE_LEN,
    .origin = -1,
    .pio_version = 0,
#if PICO_PIO_VERSION > 0
    .used_gpio_ranges = (1u << (I2S_DATA_PIN / 16)) | (1u << ((I2S_DATA_PIN + I2S_SAMPLE_BITS - 1u) / 16)),
#endif
};

static void i2s_capture_build(void) {
    uint16_t *i = i2s_capture_instr;
    i[0] = pio_encode_wait_pin(true, I2S_CLK_BIT);
    i[1] = pio_encode_wait_pin(false, I2S_CLK_BIT);
    i[2] = pio_encode_in(pio_pins, I2S_SAMPLE_BITS);
}

static pio_sm_config i2s_capture_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset, offset + I2S_CAPTURE_LEN - 1u);
    return c;
}

//--------------------------------------------------------------------+
// Filtered I2S check
//--------------------------------------------------------------------+

// The filtered output (every dual-line filter but NOS) plays a block
// after the S/PDIF line (the filter runs a block ahead of the DMA). Both
// carry the same input frame index for the same sample, the filter's
// group delay being in the reference too, but the reference computed from
// the window alone is exact only after the filter's transient
// (selftest_i2s_expect_transient()). So the I2S capture starts the block,
// the transient and OS_DELAY_MARGIN_FRAMES after the S/PDIF one, and the
// S/PDIF window is stretched so that it also covers the I2S capture plus
// the start jitter of the task (OS_MARGIN_FRAMES; the filter is causal,
// so the reference needs no input beyond the captured frames).
#define OS_DELAY_MARGIN_FRAMES 96u
#define OS_MARGIN_FRAMES       192u
// Reference words per channel: the longest window an 8x filter gets (four
// blocks) times 8, or six blocks times 4 or 1.
#define EXPECT_MAX_OUT      ((4u * BLOCK_FRAMES + EXTRA_FRAMES) * 8u)
_Static_assert(EXPECT_MAX_OUT >= MAX_FRAMES * 4u, "reference buffer");

// Work per selftest_task() call, about 100 us on the slowest clock.
#define SPDIF_STEP_WORDS     32u
#define SPDIF_STEP_SUBFRAMES 8u
#define I2S_STEP_BUDGET      256u

//--------------------------------------------------------------------+
// Buffers
//--------------------------------------------------------------------+

typedef struct {
    PIO      pio;
    uint     sm, offset, dma;
    uint32_t *buf;
    uint32_t t0;            // start of the running capture
    uint32_t timeout_us;
} capture_t;

static uint32_t  spdif_buf[MAX_WORDS];
static uint32_t  i2s_buf[I2S_WORDS];
static int32_t   spdif_frames[2u * MAX_FRAMES];
static uint32_t  expect_l[EXPECT_MAX_OUT], expect_r[EXPECT_MAX_OUT];
static selftest_i2s_frame_t i2s_frames[I2S_MAX_FRAMES];
static capture_t spdif_cap, i2s_cap;

//--------------------------------------------------------------------+
// Result publication: core 1 writes, core 0 reads (sequence lock)
//--------------------------------------------------------------------+

typedef struct {
    selftest_report_t     spdif;
    selftest_i2s_report_t i2s;
} reports_t;

static volatile uint32_t pub_lock;      // odd while core 1 writes
static reports_t pub;

static void publish(const reports_t *rep) {
    pub_lock++;
    __dmb();
    memcpy(&pub, rep, sizeof(pub));
    __dmb();
    pub_lock++;
}

// A consistent copy of the last reports; retries while core 1 is writing.
static void read_published(reports_t *out) {
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

static bool claim_on(capture_t *c, PIO p, const pio_program_t *prog, uint pin, uint count) {
    uint base = pio_get_gpio_base(p);
    if (pin < base || pin + count > base + 32u || !pio_can_add_program(p, prog)) {
        return false;
    }
    int sm = pio_claim_unused_sm(p, false);
    if (sm < 0) {
        return false;
    }
    c->pio = p;
    c->sm = (uint)sm;
    c->offset = (uint)pio_add_program(p, prog);
    return true;
}

// Any PIO block can read any pad. Prefer a block other than the
// transmitters', so theirs is left alone.
static bool claim_pio(capture_t *c, const pio_program_t *prog, uint pin, uint count) {
    static const PIO pios[] = {
#if NUM_PIOS > 2
        pio2,
#endif
        pio1, pio0,
    };
    for (int pass = 0; pass < 2; pass++) {
        for (unsigned i = 0; i < count_of(pios); i++) {
            bool tx_block = pios[i] == spdif_tx_pio();
            if (tx_block == (pass == 0)) {
                continue;
            }
            if (claim_on(c, pios[i], prog, pin, count)) {
                return true;
            }
        }
    }
    return false;
}

// Claims a state machine and a DMA channel that capture count pins from
// pin upwards, shifting left, into buf.
typedef pio_sm_config (*default_config_fn)(uint offset);

static bool capture_init(capture_t *c, const pio_program_t *prog, default_config_fn config,
                         uint pin, uint count, uint32_t ie_mask, uint32_t *buf, uint32_t div) {
    if (!claim_pio(c, prog, pin, count)) {
        return false;
    }
    int ch = dma_claim_unused_channel(false);
    if (ch < 0) {
        pio_remove_program(c->pio, prog, c->offset);
        pio_sm_unclaim(c->pio, c->sm);
        return false;
    }
    c->dma = (uint)ch;
    c->buf = buf;

    // Input enabled, isolation off. The transmitters' pio_gpio_init() does
    // this too; FUNCSEL and the output settings stay their own.
    for (uint p = 0; p < 32u; p++) {
        if (ie_mask & (1u << p)) {
            hw_set_bits(&pads_bank0_hw->io[p], PADS_BANK0_GPIO0_IE_BITS);
            hw_clear_bits(&pads_bank0_hw->io[p], PADS_BANK0_GPIO0_ISO_BITS);
        }
    }

    // Read only: no pio_gpio_init(), no pindirs.
    pio_sm_config sc = config(c->offset);
    sm_config_set_in_pins(&sc, pin);
    sm_config_set_in_pin_count(&sc, count);
    sm_config_set_in_shift(&sc, false, true, 32);
    sm_config_set_fifo_join(&sc, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv_int_frac8(&sc, (uint16_t)div, 0);
    pio_sm_init(c->pio, c->sm, c->offset, &sc);     // left disabled

    dma_channel_config dc = dma_channel_get_default_config(c->dma);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, false);
    channel_config_set_write_increment(&dc, true);
    channel_config_set_dreq(&dc, pio_get_dreq(c->pio, c->sm, false));
    channel_config_set_irq_quiet(&dc, true);
    dma_channel_configure(c->dma, &dc, buf, &c->pio->rxf[c->sm], 0, false);
    return true;
}

// Starts capturing n_words at clk_sys / div; the capture is given up after
// timeout_us. The enable bit is set and cleared atomically: the receiver
// (spdif/spdif_rx.c) may share the PIO block from core 0.
static void __not_in_flash_func(capture_start)(capture_t *c, uint32_t div, uint32_t n_words,
                                               uint32_t timeout_us) {
    hw_clear_bits(&c->pio->ctrl, 1u << c->sm);
    pio_sm_set_clkdiv_int_frac8(c->pio, c->sm, (uint16_t)div, 0);
    pio_sm_clear_fifos(c->pio, c->sm);
    pio_sm_restart(c->pio, c->sm);
    pio_sm_clkdiv_restart(c->pio, c->sm);
    pio_sm_exec(c->pio, c->sm, pio_encode_jmp(c->offset));

    dma_channel_set_write_addr(c->dma, c->buf, false);
    dma_channel_set_trans_count(c->dma, n_words, true);
    c->t0 = time_us_32();
    c->timeout_us = timeout_us;
    hw_set_bits(&c->pio->ctrl, 1u << c->sm);
}

// 0 while the capture runs, 1 when it completed, -1 on timeout (measured
// from the start); the state machine is stopped either way.
static int __not_in_flash_func(capture_poll)(capture_t *c) {
    if (dma_channel_is_busy(c->dma)) {
        if (time_us_32() - c->t0 <= c->timeout_us) {
            return 0;
        }
        hw_clear_bits(&c->pio->ctrl, 1u << c->sm);
        dma_channel_abort(c->dma);
        return -1;
    }
    hw_clear_bits(&c->pio->ctrl, 1u << c->sm);
    return 1;
}

static uint32_t window_timeout_us(uint32_t expect_us) {
    return 3u * expect_us + 20000u;
}

// clk_sys against the crystal, without waiting: the counter runs for about
// 85 us after fc_start().
static void fc_start(void) {
    fc_hw_t *fc = &clocks_hw->fc0;
    fc->ref_khz = clock_get_hz(clk_ref) / 1000u;
    fc->interval = 10;
    fc->min_khz = 0;
    fc->max_khz = 0xffffffffu;
    fc->src = CLOCKS_FC0_SRC_VALUE_CLK_SYS;
}

static bool fc_result(uint32_t *hz) {
    fc_hw_t *fc = &clocks_hw->fc0;
    if (!(fc->status & CLOCKS_FC0_STATUS_DONE_BITS)) {
        return false;
    }
    *hz = (fc->result >> CLOCKS_FC0_RESULT_KHZ_LSB) * 1000u;
    return true;
}

//--------------------------------------------------------------------+
// Window plan
//--------------------------------------------------------------------+

typedef struct {
    uint32_t dtx;           // S/PDIF PIO divider
    uint32_t frames;        // S/PDIF window in frames
    uint32_t div, words;    // S/PDIF capture divider and length
    uint32_t i2s_words;     // I2S capture length (0: none)
    uint32_t i2s_delay_us;  // I2S capture start after the S/PDIF one (0: together)
    uint32_t i2s_frames;    // output frames the I2S capture holds
    bool     compare;       // reference check of the filtered output
} plan_t;

static uint32_t ceil_div(uint32_t a, uint32_t b) {
    return (a + b - 1u) / b;
}

// Frame layout of the running I2S output from the generator's timing.
static bool i2s_layout(selftest_i2s_layout_t *lay, i2s_format_t f, uint32_t dtx, unsigned osr) {
    i2s_timing_t t;
    if (!i2s_select_timing(dtx, f, osr, i2s_tx_trimmed(), &t)) {
        return false;
    }
    lay->sample_bits = I2S_SAMPLE_BITS;
    lay->clk_bit = I2S_CLK_BIT;
    lay->format = (uint8_t)f;
    lay->width = (uint8_t)selftest_i2s_format_width(f);
    lay->slots = t.slots;
    lay->lsb_slot = t.lsb_slot;
    lay->latch_slot = t.latch_slot;
    // The sample after slot 0's falling BCLK edge shows the pin one clk_sys
    // after it, so it holds a LATCH tail of 2 clk_sys or more as well.
    lay->latch_slots = (uint8_t)(t.latch_slots + (t.latch_tail * t.divider >= 2u ? 1u : 0u));
    lay->offset_binary = i2s_format_is_dual(f) && i2s_format_table[f].offset_binary;
    return true;
}

// compare: the reference check of a filtered output with osr output
// frames per input frame and a transient of `transient` output frames.
static void plan_window(plan_t *p, uint32_t rate, uint32_t clk, bool i2s_run,
                        const selftest_i2s_layout_t *lay, unsigned osr, bool compare,
                        uint32_t transient) {
    memset(p, 0, sizeof(*p));
    p->dtx = (clk + 64u * rate) / (128u * rate);
    uint32_t blocks = MIN_BLOCKS;
    if (i2s_run) {
        p->i2s_words = I2S_WORDS;
        p->i2s_frames = I2S_SAMPLES / lay->slots;
    }
    if (i2s_run && compare) {
        p->compare = true;
        // Window frames before the first captured one; the capture starts
        // the block of lag later still.
        uint32_t lead = ceil_div(transient, osr) + OS_DELAY_MARGIN_FRAMES;
        p->i2s_delay_us = (uint32_t)(((uint64_t)(lead + SPDIF_TX_BUFFER_FRAMES) * 1000000u +
                                      rate - 1u) / rate);
        uint32_t in_frames = ceil_div(p->i2s_frames, osr);
        blocks = ceil_div(lead + in_frames + OS_MARGIN_FRAMES, BLOCK_FRAMES);
        if (blocks < MIN_BLOCKS) {
            blocks = MIN_BLOCKS;
        }
        uint32_t max_blocks = EXPECT_MAX_OUT / osr / BLOCK_FRAMES;
        if (max_blocks > MAX_BLOCKS) {
            max_blocks = MAX_BLOCKS;
        }
        if (blocks > max_blocks) {
            // Shorten the I2S capture to what the longest window covers.
            blocks = max_blocks;
            uint32_t room = blocks * BLOCK_FRAMES - lead - OS_MARGIN_FRAMES;
            p->i2s_frames = room * osr;
            p->i2s_words = p->i2s_frames * lay->slots * I2S_SAMPLE_BITS / 32u;
            p->i2s_frames = p->i2s_words * 32u / I2S_SAMPLE_BITS / lay->slots;
        }
    }
    p->frames = blocks * BLOCK_FRAMES + EXTRA_FRAMES;
    uint32_t cells = p->frames * 128u;
    uint64_t total = (uint64_t)cells * p->dtx;         // capture samples at divider 1
    uint32_t div = ceil_div(p->dtx, MAX_OVS);
    uint32_t fit = (uint32_t)((total + 32ull * MAX_WORDS - 1u) / (32ull * MAX_WORDS));
    if (div < fit) {
        div = fit;
    }
    if (div < 1u) {
        div = 1u;
    }
    p->div = div;
    p->words = (uint32_t)((total / div + 31u) / 32u);
    if (p->words > MAX_WORDS) {
        p->words = MAX_WORDS;
    }
}

//--------------------------------------------------------------------+
// Core 1 task
//--------------------------------------------------------------------+

enum {
    PH_START,
    PH_CAPTURE,
    PH_VERIFY,
    PH_SPDIF_DECODE,
    PH_SPDIF_DONE,
    PH_EXPECT,
    PH_I2S_DECODE,
    PH_REPORT,
    PH_PAUSE,
};

typedef struct {
    uint8_t  phase;
    uint32_t pause_until;
    // Snapshot at the window start.
    uint32_t g_tx, g_clk, rate, clk;
    uint8_t  bits;
    // The S/PDIF line is on. Off while the output runs (PICOI2S_KEEP_CLOCKS
    // idle), S/PDIF is reported stopped and the I2S output checked alone.
    bool     line;
    bool     i2s_run, layout_ok, pattern, filtered;
    i2s_format_t fmt;
    i2s_filter_type_t filter;
    unsigned osr;
    selftest_i2s_layout_t lay;
    plan_t   plan;
    // Capture progress.
    bool     i2s_pending, spdif_checked, i2s_checked;
    int      spdif_result, i2s_result;
    uint32_t fc_hz;
    // Decode.
    selftest_check_t i2s_check;
    selftest_i2s_reference_t ref;
} window_t;

static bool ready;                      // resources claimed
static window_t w;
static reports_t rep;
static selftest_decoder_t spdif_dec;
static selftest_decode_t spdif_res;
static selftest_i2s_decoder_t i2s_dec;
static selftest_i2s_decode_t i2s_res;
static selftest_i2s_expect_t expect;
static uint8_t cs[SELFTEST_CS_BYTES];
static uint32_t task_max_us;

static uint32_t tx_rate_now(void) {
    return spdif_tx_running() ? spdif_tx_sample_rate() : 0u;
}

static uint8_t i2s_pin_levels(void) {
    uint32_t g = gpio_get_all();
    return (uint8_t)(((g >> I2S_DATA_PIN) & 3u) | (((g >> I2S_BCLK_PIN) & 3u) << 2));
}

// The report's filter token for the jumper format and filter.
static unsigned filter_token(i2s_format_t fmt, i2s_filter_type_t filter) {
    return i2s_format_is_dual(fmt) ? (unsigned)filter : SELFTEST_FILTER_DIRECT;
}

static void finish_window(uint32_t pause_us) {
    publish(&rep);
    w.pause_until = time_us_32() + pause_us;
    w.phase = PH_PAUSE;
}

// Snapshots the output state and starts the captures. False while a
// change is in progress.
static bool start_window(void) {
    // Generations are odd while a change is in progress.
    uint32_t g_tx = spdif_tx_generation();
    uint32_t g_clk = audio_clock_generation();
    if ((g_tx | g_clk) & 1u) {
        return false;
    }
    __dmb();
    w.g_tx = g_tx;
    w.g_clk = g_clk;
    w.rate = tx_rate_now();
    w.line = spdif_tx_line_on();
    w.bits = spdif_tx_word_bits();
    w.clk = clock_get_hz(clk_sys);
    w.i2s_run = w.rate != 0 && i2s_tx_running();
    w.fmt = i2s_tx_format();
    w.filter = i2s_tx_filter();
    w.osr = i2s_tx_oversampling();
    w.pattern = PATTERN_CHECK && w.line;
    selftest_i2s_report_output(&rep.i2s, filter_token(w.fmt, w.filter), w.i2s_run ? w.osr : 0u,
                               i2s_tx_late());

    uint32_t div = IDLE_DIV, words = IDLE_WORDS;
    w.layout_ok = false;
    w.filtered = false;
    w.i2s_pending = false;
    memset(&w.plan, 0, sizeof(w.plan));
    if (w.rate) {
        plan_t *p = &w.plan;
        uint32_t dtx = (w.clk + 64u * w.rate) / (128u * w.rate);
        w.layout_ok = w.i2s_run && i2s_layout(&w.lay, w.fmt, dtx, w.osr);
        // A filtered dual-line output gets the reference check, with the
        // filter's oversampling matching the running output's.
        w.filtered = w.layout_ok && w.line && i2s_format_is_dual(w.fmt) &&
                     selftest_i2s_expect_setup(&expect, w.filter, w.rate) &&
                     selftest_i2s_expect_osr(&expect) == w.osr;
        plan_window(p, w.rate, w.clk, w.layout_ok, &w.lay, w.osr, w.filtered,
                    w.filtered ? selftest_i2s_expect_transient(&expect) : 0u);
        div = p->div;
        words = p->words;
        if (w.layout_ok) {
            uint32_t slots_per_s = w.rate * w.osr * w.lay.slots;
            uint32_t expect_us = (uint32_t)((uint64_t)p->i2s_words * (32u / I2S_SAMPLE_BITS) *
                                            1000000u / slots_per_s);
            i2s_cap.timeout_us = window_timeout_us(expect_us);
            if (p->i2s_delay_us) {
                w.i2s_pending = true;       // started from the capture phase
            } else {
                capture_start(&i2s_cap, 1u, p->i2s_words, i2s_cap.timeout_us);
            }
        }
    }
    uint32_t expect_us = (uint32_t)((uint64_t)words * 32u * div * 1000000u / w.clk);
    capture_start(&spdif_cap, div, words, window_timeout_us(expect_us));
    fc_start();
    w.spdif_checked = w.i2s_checked = false;
    w.spdif_result = w.i2s_result = 0;
    w.phase = PH_CAPTURE;
    return true;
}

// True when every capture of the window has completed or timed out.
static bool poll_captures(void) {
    uint32_t now = time_us_32();
    if (w.i2s_pending && now - spdif_cap.t0 >= w.plan.i2s_delay_us) {
        capture_start(&i2s_cap, 1u, w.plan.i2s_words, i2s_cap.timeout_us);
        w.i2s_pending = false;
    }
    if (!w.spdif_checked) {
        w.spdif_result = capture_poll(&spdif_cap);
        w.spdif_checked = w.spdif_result != 0;
    }
    if (!w.i2s_checked && w.layout_ok && !w.i2s_pending) {
        w.i2s_result = capture_poll(&i2s_cap);
        w.i2s_checked = w.i2s_result != 0;
    }
    return w.spdif_checked && (!w.layout_ok || w.i2s_checked);
}

// Discards the window if the output changed under it, else starts the
// S/PDIF decode.
static bool verify_window(void) {
    if (!fc_result(&w.fc_hz)) {
        return false;
    }
    __dmb();
    bool changed = spdif_tx_generation() != w.g_tx || audio_clock_generation() != w.g_clk ||
                   tx_rate_now() != w.rate || spdif_tx_line_on() != w.line ||
                   spdif_tx_word_bits() != w.bits ||
                   clock_get_hz(clk_sys) != w.clk ||
                   (w.rate != 0 && i2s_tx_running()) != w.i2s_run ||
                   i2s_tx_oversampling() != w.osr ||
                   audio_pipeline_ramp_age_ms() < RAMP_HOLD_MS;
    uint64_t fc_k = (uint64_t)w.fc_hz * 1000u;
    if (fc_k < (uint64_t)w.clk * (1000u - CLK_TOLERANCE_PERMILLE) ||
        fc_k > (uint64_t)w.clk * (1000u + CLK_TOLERANCE_PERMILLE)) {
        changed = true;
    }
    if (changed) {
        selftest_report_skipped(&rep.spdif);
        selftest_i2s_report_skipped(&rep.i2s);
        finish_window(0);
        return true;
    }
    if (w.spdif_result < 0) {
        selftest_report_timeout(&rep.spdif, w.line ? w.rate : 0u, w.bits);
        w.phase = PH_SPDIF_DONE;
        return true;
    }
    if (!w.line || spdif_channel_status_build(cs, w.rate, w.bits) != 0) {
        memset(cs, 0, sizeof(cs));
    }
    bool keep_frames = w.plan.compare && w.i2s_result > 0;
    selftest_decode_begin(&spdif_dec, spdif_buf, w.plan.words ? w.plan.words : IDLE_WORDS, cs,
                          w.pattern ? w.bits : 0u, keep_frames ? spdif_frames : NULL, MAX_FRAMES,
                          &spdif_res);
    w.phase = PH_SPDIF_DECODE;
    return true;
}

// The S/PDIF report is in; decides the I2S path.
static void spdif_done(void) {
    if (w.spdif_result >= 0) {
        selftest_report_window(&rep.spdif, &spdif_res, w.line ? w.rate : 0u, w.bits, w.pattern,
                               w.fc_hz, w.plan.div ? w.plan.div : IDLE_DIV);
    }
    if (!w.rate) {
        uint8_t lvl = i2s_pin_levels();
        // The output may have started during the decode.
        __dmb();
        if (spdif_tx_generation() != w.g_tx || tx_rate_now() != 0u) {
            selftest_i2s_report_skipped(&rep.i2s);
            finish_window(0);
            return;
        }
        selftest_i2s_report_state(&rep.i2s, lvl ? SELFTEST_PINHI : SELFTEST_IDLE, 0, 0, lvl);
        finish_window(IDLE_PAUSE_US);
        return;
    }
    if (!w.i2s_run) {
        selftest_i2s_report_state(&rep.i2s, SELFTEST_OFF, w.rate, w.bits, 0);
        finish_window(0);
        return;
    }
    if (!w.layout_ok) {
        selftest_i2s_report_state(&rep.i2s, SELFTEST_NOTIM, w.rate, w.bits, 0);
        finish_window(0);
        return;
    }
    if (w.i2s_result < 0) {
        selftest_i2s_report_state(&rep.i2s, SELFTEST_NOCLK, w.rate, w.bits, 0);
        finish_window(0);
        return;
    }
    if (w.plan.compare) {
        // The reference needs the window's S/PDIF samples complete and in
        // order.
        uint32_t stored = selftest_decode_frames_stored(&spdif_dec);
        if (w.spdif_result < 0 || !selftest_decode_frames_clean(&spdif_res) ||
            stored != spdif_res.frames) {
            selftest_i2s_report_skipped(&rep.i2s);
            finish_window(0);
            return;
        }
        if (stored > EXPECT_MAX_OUT / w.osr) {
            stored = EXPECT_MAX_OUT / w.osr;
        }
        selftest_i2s_expect_begin(&expect, spdif_frames, stored, w.lay.width, expect_l, expect_r,
                                  EXPECT_MAX_OUT);
        w.phase = PH_EXPECT;
        return;
    }
    w.i2s_check = w.pattern ? SELFTEST_CHECK_PATTERN : SELFTEST_CHECK_NONE;
    selftest_i2s_decode_begin(&i2s_dec, i2s_buf, w.plan.i2s_words * 32u / I2S_SAMPLE_BITS, &w.lay,
                              w.pattern ? w.bits : 0u, NULL, i2s_frames, I2S_MAX_FRAMES, &i2s_res);
    w.phase = PH_I2S_DECODE;
}

static void expect_done(void) {
    selftest_i2s_expect_reference(&expect, &w.ref);
    w.i2s_check = SELFTEST_CHECK_REFERENCE;
    selftest_i2s_decode_begin(&i2s_dec, i2s_buf, w.plan.i2s_words * 32u / I2S_SAMPLE_BITS, &w.lay,
                              0, &w.ref, i2s_frames, I2S_MAX_FRAMES, &i2s_res);
    w.phase = PH_I2S_DECODE;
}

static void __not_in_flash_func(task_step)(void) {
    switch (w.phase) {
    case PH_PAUSE:
        if ((int32_t)(time_us_32() - w.pause_until) < 0) {
            return;
        }
        w.phase = PH_START;
        /* fall through */
    case PH_START:
        start_window();
        return;
    case PH_CAPTURE:
        if (poll_captures()) {
            w.phase = PH_VERIFY;
        }
        return;
    case PH_VERIFY:
        verify_window();
        return;
    case PH_SPDIF_DECODE:
        if (selftest_decode_step(&spdif_dec, SPDIF_STEP_WORDS, SPDIF_STEP_SUBFRAMES)) {
            w.phase = PH_SPDIF_DONE;
        }
        return;
    case PH_SPDIF_DONE:
        spdif_done();
        return;
    case PH_EXPECT:
        if (selftest_i2s_expect_step(&expect, SELFTEST_EXPECT_STEP_OUT)) {
            expect_done();
        }
        return;
    case PH_I2S_DECODE:
        if (selftest_i2s_decode_step(&i2s_dec, I2S_STEP_BUDGET)) {
            w.phase = PH_REPORT;
        }
        return;
    case PH_REPORT:
        selftest_i2s_report_window(&rep.i2s, &i2s_res, w.rate, w.bits, w.i2s_check,
                                   w.plan.i2s_frames);
        finish_window(0);
        return;
    default:
        w.phase = PH_START;
        return;
    }
}

void __not_in_flash_func(selftest_task)(void) {
    if (!ready) {
        return;
    }
    uint32_t t0 = time_us_32();
    task_step();
    uint32_t dt = time_us_32() - t0;
    if (dt > task_max_us) {
        task_max_us = dt;
    }
}

uint32_t selftest_task_max_us(void) {
    return task_max_us;
}

//--------------------------------------------------------------------+
// Core 0
//--------------------------------------------------------------------+

void selftest_init(void) {
    i2s_capture_build();
    rep.i2s.fmt_name = i2s_format_name(i2s_tx_format());
    selftest_i2s_report_output(&rep.i2s, filter_token(i2s_tx_format(), i2s_tx_filter()), 0, 0);
    if (!capture_init(&spdif_cap, &selftest_capture_program, selftest_capture_program_get_default_config,
                      SPDIF_PIN, 1, 1u << SPDIF_PIN, spdif_buf, IDLE_DIV) ||
        !capture_init(&i2s_cap, &i2s_capture_program, i2s_capture_config, I2S_DATA_PIN,
                      I2S_SAMPLE_BITS, I2S_CAPTURE_IE_MASK, i2s_buf, 1)) {
        rep.spdif.state = SELFTEST_NOPIO;
        rep.i2s.state = SELFTEST_NOPIO;
        publish(&rep);
        return;
    }
    publish(&rep);
    w.phase = PH_START;
    ready = true;
}

const char *selftest_string(void) {
    static char buf[SELFTEST_STRING_MAX + 1u];
    reports_t r;
    read_published(&r);
    selftest_report_format(buf, sizeof(buf), &r.spdif);
    return buf;
}

const char *selftest_i2s_string(void) {
    static char buf[SELFTEST_STRING_MAX + 1u];
    reports_t r;
    read_published(&r);
    selftest_i2s_report_format(buf, sizeof(buf), &r.i2s);
    return buf;
}
