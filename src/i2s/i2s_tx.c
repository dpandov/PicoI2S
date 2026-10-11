// I2S / dual-line output: a PIO state machine fed by a four-buffer DMA
// ring, run as the S/PDIF transmitter's companion (spdif_tx.h). The DMA
// chain is the S/PDIF one with a longer ring: a data channel plays a
// buffer into the PIO and chains to a control channel that reloads the
// next buffer's address from a four-entry ring. The S/PDIF interrupt hands
// every fetched block to fill(): I2S / LJ / RJ and dual-line NOS are
// converted there into the buffer the DMA plays next; the other dual-line
// filters copy the block to a mailbox for core 1, which filters and packs
// it into the buffer after that (one block of lag; buffer 0 starts as
// silence). A source that may flag pre-emphasis (the S/PDIF input) sends
// every format through core 1, de-emphasis first. Format table, timing
// selection, program generators and converters are in i2s_format.h, the
// filters in i2s_filter.h.
//
// Under a trim the two state machines follow it through separate pacers
// whose rings round the rate separately (up to about 20 ppb apart), so the
// DMA here would drift against the S/PDIF fill cadence by up to 6 frames
// per hour. A slow proportional loop holds it: every live fill adds the
// distance from the DMA to its target buffer, the main loop turns the
// window's mean against the first window's into a rate offset for this
// state machine's pacer (trim_offset_ppb, from spdif_tx_set_trim_ppb()).

#include "i2s/i2s_tx.h"

#include <string.h>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "pico.h"

#include "board_config.h"
#include "i2s/i2s_deemph.h"
#include "i2s/i2s_filter.h"
#include "i2s/i2s_format.h"
#include "spdif/spdif_tx.h"

_Static_assert(I2S_DATA_R_PIN == I2S_DATA_PIN + 1, "out pins must be consecutive");
_Static_assert(I2S_LRCK_PIN == I2S_BCLK_PIN + 1, "side-set pins must be consecutive");
_Static_assert(I2S_FILTER_MAX_IN == SPDIF_TX_BUFFER_FRAMES, "the filter works in S/PDIF blocks");

#define DATA_PINS  2u
#define CLOCK_PINS 2u
#define PIN_MASK   ((((1ull << DATA_PINS) - 1u) << I2S_DATA_PIN) | \
                    (((1ull << CLOCK_PINS) - 1u) << I2S_BCLK_PIN))
#define CLOCK_MASK   (1ull << I2S_CLOCK_PIN)

#define BUFFERS          4u
#define IN_FRAMES        SPDIF_TX_BUFFER_FRAMES
#define OUT_FRAMES_MAX   (IN_FRAMES * I2S_FILTER_MAX_OSR)
// Largest buffer: 768 output frames of 32 slots on two pins.
#define BUFFER_WORDS_MAX (OUT_FRAMES_MAX * I2S_DUAL_SLOTS_MAX * 2u / 32u)
#define MAILBOX_SLOTS    4u
// Minimum distance from the DMA read pointer to a buffer about to be
// written, in input frames. The S/PDIF side has already required 16
// frames for its own buffer, and this DMA runs within 5 frames of that
// one (the difference in what the two TX FIFOs hold).
#define GUARD_FRAMES 6u

// Phase lock: offset in ppb per buffer of drift, and its limit. One ppb
// moves the phase by 1e-9 buffers per buffer played, so at 333..1000
// buffers per second (32..96 kHz) the time constant is 1e9 / (K x buffers
// per second): 600..200 s. A constant mismatch m leaves m / K buffers.
#define PHASE_K_PPB        5000
#define PHASE_OFFSET_MAX   2000
// Fills summed between two reads at most (4 s at 96 kHz): the sum of
// distances below ring_words stays within 32 bits.
#define PHASE_FILLS_MAX    4096u

static i2s_format_t fmt;
static i2s_filter_type_t filter_type;   // NOS for the direct formats
static i2s_format_spec_t spec;      // RAM copy of the format table entry
static PIO pio;
static uint sm;
static uint clk_sm;
static bool clk_used;               // the clock state machine runs: MCLK (direct) or DEM (TDA1541A),
                                    // at clk_sys, following BCLK
static uint dma_data;
static uint dma_ctrl;
static uint32_t dma_mask;

static uint32_t buffers[BUFFERS * BUFFER_WORDS_MAX];
// Read by the control channel through a 16-byte read ring.
static uint32_t buffer_addr[BUFFERS] __attribute__((aligned(16)));

static uint16_t program_instr[I2S_PROGRAM_MAX];
static pio_program_t program = {
    .instructions = program_instr,
    .length = 0,
    .origin = -1,
    .pio_version = 0,
#if PICO_PIO_VERSION > 0
    .used_gpio_ranges = (1u << (I2S_DATA_PIN / 16)) | (1u << (I2S_BCLK_PIN / 16)) |
                        (1u << (I2S_LRCK_PIN / 16)),
#endif
};
static uint16_t clk_instr[I2S_CLOCK_PROGRAM_MAX];
static pio_program_t clk_program = {
    .instructions = clk_instr,
    .length = 0,
    .origin = -1,
    .pio_version = 0,
#if PICO_PIO_VERSION > 0
    .used_gpio_ranges = 1u << (I2S_CLOCK_PIN / 16),
#endif
};
static bool loaded;
static bool clk_loaded;
static uint offset;
static uint clk_offset;

static volatile bool running;       // between arm() and stop()
static volatile bool active;        // prepared: blocks are accepted (fill, core 1)

// Current layout, read by the interrupt and core 1.
static i2s_timing_t timing;
static bool trim_room;              // prepared for a trimmed rate (D >= 2)
static bool via_core1;              // blocks go through core 1
static bool filtered;               // dual-line filter on core 1
static bool deemph_used;            // de-emphasis on core 1 (flag per block)
static unsigned lag;                // buffers between a block's fill and its play
static uint32_t words_per_buffer;
static uint32_t ring_words;
static uint32_t guard_words;
static uint32_t next_buffer;        // pre-fill before the DMA runs

// Dual-line packer tables, read by the interrupt and core 1.
static i2s_pack_tables_t pack_tables;

// Mailbox from the interrupt (producer, core 0) to core 1 (consumer):
// the block's frames and its target buffer, slot head % MAILBOX_SLOTS.
static int32_t mailbox[MAILBOX_SLOTS][2u * IN_FRAMES];
static uint8_t mailbox_target[MAILBOX_SLOTS];
static bool mailbox_emphasis[MAILBOX_SLOTS];
static volatile uint32_t mb_head;
static volatile uint32_t mb_tail;
static volatile bool core1_busy;
static volatile uint32_t late_irq;      // interrupt side: direct path skipped, mailbox full
static volatile uint32_t late_core1;    // core 1: the DMA had reached the target

// Phase lock: the interrupt sums the live fills' distances, the main loop
// reads them (trim_room starts only).
static volatile uint32_t phase_sum;
static volatile uint32_t phase_fills;
static bool phase_have_ref;             // main loop from here on
static uint32_t phase_ref;              // first window's mean distance, words x 256
static int32_t phase_drift;             // DMA lead over the reference, words x 256
static int32_t phase_offset;            // ppb

// Core 1 only.
static i2s_deemph_t deemph;
static i2s_filter_t filter;
static int32_t filter_out[2u * OUT_FRAMES_MAX];

// Next word the data channel reads, as an index into buffers[].
static __force_inline uint32_t dma_position(void) {
    uint32_t w = (dma_hw->ch[dma_data].read_addr - (uintptr_t)buffers) / 4u;
    // Past the end: the last buffer finished, the control channel restarts buffer 0.
    return w < ring_words ? w : 0u;
}

// True if buffer `target` can be written: the DMA is one or two buffers
// before it (neither inside nor past it), at least guard_words away when
// it is the next one.
static __force_inline bool buffer_clear(unsigned target) {
    uint32_t p = dma_position();
    unsigned dist = (target - p / words_per_buffer) & (BUFFERS - 1u);
    if (dist == 0 || dist == BUFFERS - 1u)
        return false;
    uint32_t start = target * words_per_buffer;
    uint32_t ahead = start >= p ? start - p : start + ring_words - p;
    return dist > 1u || ahead >= guard_words;
}

// Buffer for the block the S/PDIF side is filling into its buffer b, the
// DMA at p: the one `lag` + 1 after the buffer the DMA reads, with the
// parity of b + lag (S/PDIF block k is in its buffer k % 2, here in
// (k + lag) % 4). The parity settles the race at a buffer boundary, where
// this DMA may be a few words behind the S/PDIF one and still at the end
// of the previous buffer.
static __force_inline unsigned live_target(unsigned b, uint32_t p) {
    unsigned target = (p / words_per_buffer + lag + 1u) & (BUFFERS - 1u);
    if (((target ^ b ^ lag) & 1u) != 0)
        target = (target + 1u) & (BUFFERS - 1u);
    return target;
}

// Unfiltered block at the sample rate into an output buffer.
static __force_inline void convert(uint32_t *dst, const int32_t *pcm, uint32_t n_frames) {
    switch (spec.conv) {
    case I2S_CONV_COPY:
        i2s_convert_copy(dst, pcm, n_frames);
        break;
    case I2S_CONV_SHIFT:
        i2s_convert_shift(dst, pcm, n_frames, spec.shift);
        break;
    default:
        i2s_pack_dual(dst, pcm, n_frames, &timing, &spec, &pack_tables, false);
        break;
    }
}

// Register access and inlined converters only: runs from RAM in the
// S/PDIF DMA interrupt. The copy loop stays a loop (no memcpy from flash).
static void __attribute__((optimize("no-tree-loop-distribute-patterns")))
__not_in_flash_func(companion_fill)(unsigned b, const int32_t *pcm, uint32_t n_frames, bool live,
                                    uint32_t flags) {
    if (!active)
        return;
    if (n_frames > IN_FRAMES)
        n_frames = IN_FRAMES;
    unsigned target;
    if (live) {
        uint32_t p = dma_position();
        target = live_target(b, p);
        // Target start ahead of the DMA: grows while this output runs slow.
        if (trim_room && phase_fills < PHASE_FILLS_MAX) {
            uint32_t start = target * words_per_buffer;
            phase_sum += start >= p ? start - p : start + ring_words - p;
            phase_fills++;
        }
    } else {
        target = next_buffer++ & (BUFFERS - 1u);
    }

    if (via_core1) {
        if (mb_head - mb_tail >= MAILBOX_SLOTS) {
            late_irq++;
            return;
        }
        unsigned slot = mb_head & (MAILBOX_SLOTS - 1u);
        int32_t *dst = mailbox[slot];
        for (uint32_t i = 0; i < 2u * n_frames; i++)
            dst[i] = pcm[i];
        mailbox_target[slot] = (uint8_t)target;
        mailbox_emphasis[slot] = (flags & SPDIF_TX_FLAG_EMPHASIS) != 0;
        __dmb();
        mb_head++;
        return;
    }

    if (live && !buffer_clear(target)) {
        late_irq++;
        return;
    }
    convert(&buffers[target * words_per_buffer], pcm, n_frames);
}

// Core 1. The busy flag and the re-check of `active` after raising it
// pair with stop(), which clears `active` and then waits for busy to drop:
// one side always sees the other, so stop() never tears down state under
// a block in progress, and no block starts after it.
void __not_in_flash_func(i2s_tx_core1_task)(void) {
    if (!active || mb_tail == mb_head)
        return;
    core1_busy = true;
    __dmb();
    if (!active) {
        core1_busy = false;
        return;
    }
    unsigned slot = mb_tail & (MAILBOX_SLOTS - 1u);
    unsigned target = mailbox_target[slot];
    int32_t *in = mailbox[slot];
    if (deemph_used)
        i2s_deemph_run(&deemph, in, in, IN_FRAMES, mailbox_emphasis[slot]);
    if (filtered)
        i2s_filter_run(&filter, in, IN_FRAMES, filter_out);

    // Checked after the filter, by when the DMA has settled past the
    // boundary the block was fetched at (see live_target()). The packer
    // saturates the filters' 1/16-scale output.
    if (!buffer_clear(target))
        late_core1++;
    else if (filtered)
        i2s_pack_dual(&buffers[target * words_per_buffer], filter_out, IN_FRAMES * timing.osr,
                      &timing, &spec, &pack_tables, true);
    else
        convert(&buffers[target * words_per_buffer], in, IN_FRAMES);
    __dmb();
    mb_tail++;
    __dmb();
    core1_busy = false;
}

static void unload_programs(void) {
    if (loaded) {
        pio_remove_program(pio, &program, offset);
        loaded = false;
    }
    if (clk_loaded) {
        pio_remove_program(pio, &clk_program, clk_offset);
        clk_loaded = false;
    }
}

static bool load_programs(void) {
    int off = pio_add_program(pio, &program);
    if (off < 0)
        return false;
    offset = (uint)off;
    loaded = true;
    if (!clk_used)
        return true;
    off = pio_add_program(pio, &clk_program);
    if (off < 0) {
        unload_programs();
        return false;
    }
    clk_offset = (uint)off;
    clk_loaded = true;
    return true;
}

static void configure_dma(void) {
    dma_channel_config c = dma_channel_get_default_config(dma_data);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));
    channel_config_set_chain_to(&c, dma_ctrl);
    dma_channel_configure(dma_data, &c, &pio->txf[sm], buffers,
                          dma_encode_transfer_count(words_per_buffer), false);

    // One word per trigger: the next buffer address, which also restarts
    // the data channel.
    c = dma_channel_get_default_config(dma_ctrl);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_ring(&c, false, 4);
    channel_config_set_irq_quiet(&c, true);
    dma_channel_configure(dma_ctrl, &c, &dma_hw->ch[dma_data].al3_read_addr_trig,
                          buffer_addr, dma_encode_transfer_count(1), false);
}

// The S/PDIF divider for the rate (the rounding of spdif_tx.c; the S/PDIF
// side has already checked it is within tolerance).
static uint32_t spdif_divider(uint32_t sample_rate) {
    uint32_t cells = 128u * sample_rate;
    return (clock_get_hz(clk_sys) + cells / 2u) / cells;
}

static bool companion_prepare(uint32_t sample_rate, uint32_t options) {
    unload_programs();
    unsigned osr = i2s_oversampling(fmt, sample_rate, filter_type);
    // A trimmed output's divider fraction carries the trim, so D >= 2.
    bool trimmed = (options & SPDIF_TX_START_TRIMMED) != 0;
    if (!i2s_select_timing(spdif_divider(sample_rate), fmt, osr, trimmed, &timing))
        return false;
    trim_room = trimmed;
    program.length = (uint8_t)i2s_program_build(program_instr, &timing, fmt);
    if (clk_used) {
        clk_program.length = (uint8_t)(
            fmt == I2S_FORMAT_TDA1541A
                ? i2s_dem_program_build(clk_instr, i2s_dem_half_bclks(&timing, fmt, sample_rate),
                                        I2S_BCLK_PIN)
                : i2s_mclk_program_build(clk_instr, timing.divider, I2S_BCLK_PIN));
        if (clk_program.length == 0)
            return false;
    }
    if (!load_programs())
        return false;

    filtered = i2s_format_is_dual(fmt) && filter_type != I2S_FILTER_NOS;
    deemph_used = (options & SPDIF_TX_START_EMPHASIS) != 0;
    via_core1 = filtered || deemph_used;
    lag = via_core1 ? 1u : 0u;
    words_per_buffer = i2s_words_per_frames(&timing, IN_FRAMES * osr);
    ring_words = BUFFERS * words_per_buffer;
    guard_words = GUARD_FRAMES * words_per_buffer / IN_FRAMES;
    for (unsigned i = 0; i < BUFFERS; i++)
        buffer_addr[i] = (uintptr_t)&buffers[i * words_per_buffer];
    // Silence in every buffer: the filtered path starts with buffer 0
    // unwritten, and the dual-line frames rely on zeros around the words.
    memset(buffers, 0, ring_words * sizeof(buffers[0]));
    next_buffer = lag;
    phase_sum = 0;
    phase_fills = 0;
    phase_have_ref = false;
    phase_drift = 0;
    phase_offset = 0;
    mb_head = 0;
    mb_tail = 0;
    if (filtered)
        i2s_filter_init(&filter, filter_type, sample_rate);
    if (deemph_used)
        i2s_deemph_init(&deemph, sample_rate, false);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset, offset + program.length - 1u);
    sm_config_set_sideset(&c, I2S_SIDE_BITS, false, false);
    sm_config_set_sideset_pins(&c, I2S_BCLK_PIN);
    sm_config_set_out_pins(&c, I2S_DATA_PIN, timing.out_pins);
    sm_config_set_out_shift(&c, false, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv_int_frac8(&c, timing.divider, 0);
    pio_sm_init(pio, sm, offset, &c);
    // The loop count of the first group; the pins stay low until enabled.
    pio_sm_exec(pio, sm, pio_encode_set(pio_x, timing.x_preload));
    if (clk_used) {
        c = pio_get_default_sm_config();
        sm_config_set_wrap(&c, clk_offset, clk_offset + clk_program.length - 1u);
        sm_config_set_set_pins(&c, I2S_CLOCK_PIN, 1);
        sm_config_set_clkdiv_int_frac8(&c, 1u, 0);
        pio_sm_init(pio, clk_sm, clk_offset, &c);
    }
    configure_dma();

    __dmb();
    active = true;
    return true;
}

static uint32_t sm_mask(void) {
    return (1u << sm) | (clk_used ? 1u << clk_sm : 0u);
}

// The control channel loads buffer 0 and starts the data channel, which
// fills the TX FIFO before the state machines are enabled.
static uint32_t companion_arm(void) {
    running = true;
    dma_channel_start(dma_ctrl);
    return sm_mask();
}

static void companion_stop(void) {
    running = false;
    active = false;
    __dmb();
    while (core1_busy)
        tight_loop_contents();
    // RP2350-E5: an aborted channel can be re-triggered through CHAIN_TO, so
    // clear EN on both channels (non-triggering CTRL alias) before the
    // abort. With EN clear, the control channel's write no longer starts
    // the data channel either.
    hw_clear_bits(&dma_hw->ch[dma_data].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    hw_clear_bits(&dma_hw->ch[dma_ctrl].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    dma_hw->abort = dma_mask;
    while (dma_hw->abort & dma_mask)
        tight_loop_contents();
    while ((dma_hw->ch[dma_data].ctrl_trig | dma_hw->ch[dma_ctrl].ctrl_trig) &
           DMA_CH0_CTRL_TRIG_BUSY_BITS)
        tight_loop_contents();

    pio_set_sm_mask_enabled(pio, sm_mask(), false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    pio_sm_set_pins_with_mask64(pio, sm, 0, PIN_MASK);
    if (clk_used) {
        pio_sm_clear_fifos(pio, clk_sm);
        pio_sm_restart(pio, clk_sm);
        pio_sm_set_pins_with_mask64(pio, clk_sm, 0, CLOCK_MASK);
    }
    unload_programs();
}

// Only the I2S state machine's divider carries the trim: MCLK and DEM
// follow BCLK themselves.
static uint32_t companion_trim_divider(io_rw_32 **reg) {
    *reg = &pio->sm[sm].clkdiv;
    return timing.divider;
}

// The window since the last call against the first one: a lead (the
// distance shrank, this output runs fast) gives a negative offset, a
// larger divider.
static int32_t companion_trim_offset_ppb(void) {
    uint32_t irq = save_and_disable_interrupts();
    uint32_t sum = phase_sum;
    uint32_t n = phase_fills;
    phase_sum = 0;
    phase_fills = 0;
    restore_interrupts(irq);
    if (!running || !trim_room || n == 0)
        return phase_offset;
    uint32_t mean = (uint32_t)(((uint64_t)sum << 8) / n);
    if (!phase_have_ref) {
        phase_ref = mean;
        phase_have_ref = true;
        return phase_offset;
    }
    // Modulo two buffers: the target's parity is pinned (live_target()),
    // so a slip moves the distance by two.
    int32_t span = (int32_t)(2u * words_per_buffer << 8);
    int32_t lead = (int32_t)(phase_ref - mean);
    if (lead > span / 2)
        lead -= span;
    else if (lead < -span / 2)
        lead += span;
    phase_drift = lead;
    int64_t off = -(int64_t)PHASE_K_PPB * lead / ((int64_t)words_per_buffer << 8);
    if (off > PHASE_OFFSET_MAX)
        off = PHASE_OFFSET_MAX;
    else if (off < -PHASE_OFFSET_MAX)
        off = -PHASE_OFFSET_MAX;
    phase_offset = (int32_t)off;
    return phase_offset;
}

static const spdif_tx_companion_t companion = {
    .prepare = companion_prepare,
    .fill = companion_fill,
    .arm = companion_arm,
    .stop = companion_stop,
    .trim_divider = companion_trim_divider,
    .trim_offset_ppb = companion_trim_offset_ppb,
};

void i2s_tx_init(i2s_format_t format, i2s_filter_type_t filter_sel) {
    fmt = format < I2S_FORMAT_COUNT ? format : I2S_FORMAT_I2S;
    filter_type = i2s_format_is_dual(fmt) ? i2s_filter_from_code(filter_sel) : I2S_FILTER_NOS;
    spec = i2s_format_table[fmt];
    i2s_pack_tables_init(&pack_tables, spec.bits);
    clk_used = fmt == I2S_FORMAT_TDA1541A || !i2s_format_is_dual(fmt);
    pio = spdif_tx_pio();
    sm = (uint)pio_claim_unused_sm(pio, true);
    clk_sm = (uint)pio_claim_unused_sm(pio, true);
    dma_data = (uint)dma_claim_unused_channel(true);
    dma_ctrl = (uint)dma_claim_unused_channel(true);
    dma_mask = (1u << dma_data) | (1u << dma_ctrl);

    // Pins low before handing them to the PIO so they never float or
    // glitch high.
    pio_sm_set_pins_with_mask64(pio, sm, 0, PIN_MASK);
    pio_sm_set_consecutive_pindirs(pio, sm, I2S_DATA_PIN, DATA_PINS, true);
    pio_sm_set_consecutive_pindirs(pio, sm, I2S_BCLK_PIN, CLOCK_PINS, true);
    for (uint p = 0; p < 32u; p++) {
        if (PIN_MASK & (1ull << p))
            pio_gpio_init(pio, p);
    }
    // Sharp edges on BCLK (up to 9.6 MHz) and MCLK (up to 24.6 MHz); LATCH /
    // LRCK and DEM are slow.
    gpio_set_slew_rate(I2S_BCLK_PIN, GPIO_SLEW_RATE_FAST);
    if (clk_used) {
        pio_sm_set_pins_with_mask64(pio, clk_sm, 0, CLOCK_MASK);
        pio_sm_set_consecutive_pindirs(pio, clk_sm, I2S_CLOCK_PIN, 1, true);
        pio_gpio_init(pio, I2S_CLOCK_PIN);
        if (!i2s_format_is_dual(fmt))
            gpio_set_slew_rate(I2S_CLOCK_PIN, GPIO_SLEW_RATE_FAST);
    } else {
        gpio_init(I2S_CLOCK_PIN);
        gpio_put(I2S_CLOCK_PIN, false);
        gpio_set_dir(I2S_CLOCK_PIN, GPIO_OUT);
    }

    spdif_tx_set_companion(&companion);
}

i2s_format_t i2s_tx_format(void) {
    return fmt;
}

i2s_filter_type_t i2s_tx_filter(void) {
    return filter_type;
}

bool i2s_tx_running(void) {
    return running;
}

bool i2s_tx_trimmed(void) {
    return trim_room;
}

unsigned i2s_tx_oversampling(void) {
    return running ? timing.osr : 1u;
}

uint32_t i2s_tx_late(void) {
    return late_irq + late_core1;
}

int32_t i2s_tx_trim_offset_ppb(void) {
    return phase_offset;
}

int32_t i2s_tx_phase_drift_words(void) {
    return phase_drift / 256;
}

const char *i2s_format_name(i2s_format_t f) {
    static const char *const names[I2S_FORMAT_COUNT] = {
        "i2s", "lj", I2S_RJ_BITS == 24u ? "rj24" : "rj16", "tda1541a",
        "dual16", "dual18", "dual20", "dual24",
    };
    return f < I2S_FORMAT_COUNT ? names[f] : "?";
}
