// S/PDIF transmitter: PIO state machine fed by ping-pong DMA.
//
// A data channel plays one buffer into the PIO, then chains to a control
// channel that writes the next buffer's address from a two-entry ring into
// the data channel's READ_ADDR trigger alias. The hardware therefore
// alternates between the two buffers forever with no CPU involvement, and
// never reads outside them. The two buffers are one channel-status block,
// and each always holds the same block positions, so the line stays valid
// S/PDIF (preambles, parity, B every 192 frames) even when the refill is
// late: the previous block is then repeated.
//
// The completion interrupt locates the data channel's read pointer and
// re-encodes the other buffer from the source, one chunk at a time, only
// while the DMA is clear of the chunk being written.
//
// Rate trim (SPDIF_TX_START_TRIMMED): a pacer state machine per trimmed
// clock (the S/PDIF one, the companion's) writes that state machine's
// CLKDIV register twice per SPDIF_TX_DITHER_US period at cycle-exact times
// from a ring of (delay, value) pairs (spdif_trim.h, spdif_pacer.pio). A
// feed channel streams the ring into the pacer's TX FIFO endlessly, a
// write channel moves every value the pacer pushes into the CLKDIV
// register. Both writes of a period take the same path, so the DMA latency
// is common to them and cancels. A CLKDIV write does not restart the
// divider (only CTRL.CLKDIV_RESTART does), so the output phase stays
// continuous. No CPU work while running.

#include "spdif/spdif_tx.h"

#include <string.h>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/structs/dma_debug.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "pico.h"

#include "spdif/spdif_encode.h"
#include "spdif/spdif_trim.h"
#include "spdif_pacer.pio.h"
#include "spdif_tx.pio.h"

#define CELLS_PER_FRAME 128u
#define BUFFER_WORDS    (SPDIF_TX_BUFFER_FRAMES * SPDIF_WORDS_PER_FRAME)
#define RING_WORDS      (2u * BUFFER_WORDS)
#define MAX_CLOCK_PPM   1000u

_Static_assert(2 * SPDIF_TX_BUFFER_FRAMES == SPDIF_BLOCK_FRAMES,
               "the two buffers must hold exactly one channel-status block");

// Frames encoded between two checks of the DMA position.
#define CHUNK_FRAMES 16u
// Minimum distance from the DMA read pointer to a chunk about to be written:
// 16 frames (167 us at 96 kHz) against under 30 us to encode a chunk.
#define GUARD_WORDS  (16u * SPDIF_WORDS_PER_FRAME)

_Static_assert(SPDIF_TX_BUFFER_FRAMES % CHUNK_FRAMES == 0, "whole chunks per buffer");

#define DMA_IRQ_INDEX 1
// Above USB (default 0x80) so encoding is never delayed by USB handling,
// but below the diagnostic SysTick (0) so a hang in here is still caught.
#define DMA_IRQ_PRIORITY 0x40

// Beside the S/PDIF receiver (spdif_rx.c: 2 state machines, 26
// instructions); the transmitter's block is full on the RP2350-One and
// pio1 is the self-test's. All of them are driven from core 0.
#define PACER_PIO       pio0
#define PACER_RING_BITS 10      // log2 of the ring's size in bytes

_Static_assert(SPDIF_TRIM_RING_WORDS * 4u == 1u << PACER_RING_BITS, "ring size");

static PIO tx_pio;
static uint tx_sm;
static uint tx_offset;
static uint tx_pin;
static uint dma_data;
static uint dma_ctrl;
static uint32_t dma_mask;
static uint32_t data_mask;

static uint32_t buffers[RING_WORDS];
// Read by the control channel through an 8-byte read ring.
static uint32_t buffer_addr[2] __attribute__((aligned(8)));
static int32_t pcm[SPDIF_TX_BUFFER_FRAMES * 2];
static spdif_encoder_t encoder;
static spdif_source_fn source_fn;
static const spdif_tx_companion_t *companion;
static volatile bool running;
static uint32_t tx_rate;
static uint32_t tx_div;          // integer PIO divider at zero trim
static int32_t tx_error_ppb;
static uint8_t tx_bits;
static volatile bool cs_emphasis;   // the flag in encoder.cs
static volatile uint32_t generation;
static volatile uint32_t late_irqs;
static bool late_prev;          // the last interrupt was late

// Trim pacers, [0] S/PDIF, [1] the companion's. Main loop only.
typedef struct {
    uint sm;
    uint dma_feed;              // ring -> TX FIFO
    uint dma_write;             // RX FIFO -> CLKDIV
    io_rw_32 *reg;              // CLKDIV register it drives; NULL: stopped
    uint32_t div;               // integer divider at zero trim
    uint32_t target;            // Q8.24 divider in the ring
    uint32_t *ring;
} pacer_t;

static uint32_t pacer_rings[2][SPDIF_TRIM_RING_WORDS] __attribute__((aligned(1u << PACER_RING_BITS)));
static pacer_t pacers[2];
static uint pacer_offset;
static uint32_t pacer_period;   // SPDIF_TX_DITHER_US in clk_sys cycles
static bool trimmed;
static int32_t trim_ppb;

// State of the last serviced interrupt.
static uint irq_buf;            // buffer the DMA was reading
static uint32_t irq_time;       // timer, us
static uint32_t buf_us;         // one buffer period, us

static void tx_stop(void);

static void pacer_init(void) {
    if (!pio_can_add_program(PACER_PIO, &spdif_pacer_program))
        panic("spdif_tx: no room for the trim pacer program on pio0");
    pacer_offset = (uint)pio_add_program(PACER_PIO, &spdif_pacer_program);
    for (uint k = 0; k < 2u; k++) {
        pacer_t *p = &pacers[k];
        int sm = pio_claim_unused_sm(PACER_PIO, false);
        if (sm < 0)
            panic("spdif_tx: no free state machine on pio0 for the trim pacer");
        p->sm = (uint)sm;
        p->dma_feed = (uint)dma_claim_unused_channel(true);
        p->dma_write = (uint)dma_claim_unused_channel(true);
        p->ring = pacer_rings[k];
    }
}

// Starts writing reg (the CLKDIV register of a running state machine at the
// integer divider div) with the zero-trim pattern: the value it already
// holds.
static void pacer_start(pacer_t *p, io_rw_32 *reg, uint32_t div) {
    p->reg = reg;
    p->div = div;
    p->target = div << 24;
    spdif_trim_pattern(p->ring, SPDIF_TRIM_RING_PERIODS, pacer_period, p->target);

    // The atomic CTRL aliases: the block may be shared with the other core.
    uint32_t sm_bit = 1u << p->sm;
    hw_clear_bits(&PACER_PIO->ctrl, sm_bit);
    pio_sm_config c = spdif_pacer_program_get_default_config(pacer_offset);
    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_in_shift(&c, true, true, 32);
    sm_config_set_clkdiv_int_frac8(&c, 1, 0);
    pio_sm_set_config(PACER_PIO, p->sm, &c);
    pio_sm_clear_fifos(PACER_PIO, p->sm);
    pio_sm_restart(PACER_PIO, p->sm);
    pio_sm_clkdiv_restart(PACER_PIO, p->sm);
    // OSR empty whatever the restart left in it, so the first `out` pulls
    // the first delay from the FIFO.
    pio_sm_exec(PACER_PIO, p->sm, pio_encode_mov(pio_osr, pio_null));
    pio_sm_exec(PACER_PIO, p->sm, pio_encode_out(pio_null, 32));
    pio_sm_exec(PACER_PIO, p->sm, pio_encode_jmp(pacer_offset));
    // Fresh DREQ handshakes for the emptied FIFOs.
    dma_debug_hw->ch[p->dma_write].dbg_ctdreq = 0;
    dma_debug_hw->ch[p->dma_feed].dbg_ctdreq = 0;

    // Write first, so the RX FIFO never fills.
    dma_channel_config d = dma_channel_get_default_config(p->dma_write);
    channel_config_set_transfer_data_size(&d, DMA_SIZE_32);
    channel_config_set_read_increment(&d, false);
    channel_config_set_write_increment(&d, false);
    channel_config_set_dreq(&d, pio_get_dreq(PACER_PIO, p->sm, false));
    channel_config_set_high_priority(&d, true);
    dma_channel_configure(p->dma_write, &d, reg, &PACER_PIO->rxf[p->sm],
                          dma_encode_endless_transfer_count(), true);

    d = dma_channel_get_default_config(p->dma_feed);
    channel_config_set_transfer_data_size(&d, DMA_SIZE_32);
    channel_config_set_read_increment(&d, true);
    channel_config_set_write_increment(&d, false);
    channel_config_set_ring(&d, false, PACER_RING_BITS);
    channel_config_set_dreq(&d, pio_get_dreq(PACER_PIO, p->sm, true));
    dma_channel_configure(p->dma_feed, &d, &PACER_PIO->txf[p->sm], p->ring,
                          dma_encode_endless_transfer_count(), true);

    hw_set_bits(&PACER_PIO->ctrl, sm_bit);
}

// The register keeps the last value written; the next start rewrites it.
static void pacer_stop(pacer_t *p) {
    if (!p->reg)
        return;
    p->reg = NULL;
    hw_clear_bits(&PACER_PIO->ctrl, 1u << p->sm);
    // RP2350-E5: EN cleared before the abort.
    uint32_t mask = (1u << p->dma_feed) | (1u << p->dma_write);
    hw_clear_bits(&dma_hw->ch[p->dma_feed].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    hw_clear_bits(&dma_hw->ch[p->dma_write].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    dma_hw->abort = mask;
    while (dma_hw->abort & mask)
        tight_loop_contents();
    while ((dma_hw->ch[p->dma_feed].ctrl_trig | dma_hw->ch[p->dma_write].ctrl_trig) &
           DMA_CH0_CTRL_TRIG_BUSY_BITS)
        tight_loop_contents();
    pio_sm_clear_fifos(PACER_PIO, p->sm);
}

// Rewritten in place while the feed channel reads it: every word is valid
// in either version, so a read racing the rewrite only mixes the two
// patterns. A pair read half old, half new (new value, old delay)
// mis-times one half, and where the reading crosses between the patterns
// one period mixes them: a phase step of at most one period at one FRAC
// step per crossing. An interrupt holding the rewrite up only lets the old
// rate run on where the channel overtakes it.
static void pacer_set(pacer_t *p, int32_t ppb) {
    if (!p->reg)
        return;
    uint32_t target = spdif_trim_divider(p->div, ppb);
    if (target == p->target)
        return;
    p->target = target;
    spdif_trim_pattern(p->ring, SPDIF_TRIM_RING_PERIODS, pacer_period, target);
}

static void change_begin(void) {
    generation++;
    __dmb();
}

static void change_end(void) {
    __dmb();
    generation++;
}

// Next word the data channel reads, as an index into buffers[].
static __force_inline uint32_t dma_position(void) {
    uint32_t w = (dma_hw->ch[dma_data].read_addr - (uintptr_t)buffers) / 4u;
    // Past the end: buffer 1 finished, the control channel restarts buffer 0.
    return w < RING_WORDS ? w : 0u;
}

// True if a chunk of buffer b starting at buffers[w] can be written: the
// DMA is still in the other buffer, at least GUARD_WORDS before w.
static __force_inline bool chunk_clear(uint b, uint32_t w) {
    uint32_t p = dma_position();
    if (p / BUFFER_WORDS == b)
        return false;
    uint32_t ahead = w >= p ? w - p : w + RING_WORDS - p;
    return ahead >= GUARD_WORDS;
}

// Encodes buffer b from the source. live: the DMA is running, so stop at
// the first chunk it could reach (the rest keeps its previous, valid
// frames). Returns false if stopped early.
static bool __not_in_flash_func(fill_buffer)(uint b, bool live) {
    uint32_t base = b * BUFFER_WORDS;
    if (live && !chunk_clear(b, base))
        return false;
    uint32_t flags = source_fn(pcm, SPDIF_TX_BUFFER_FRAMES);
    bool emphasis = (flags & SPDIF_TX_FLAG_EMPHASIS) != 0;
    if (emphasis != cs_emphasis) {
        spdif_channel_status_set_emphasis(encoder.cs, emphasis);
        cs_emphasis = emphasis;
    }
    if (companion)
        companion->fill(b, pcm, SPDIF_TX_BUFFER_FRAMES, live, flags);
    // Pins the block position to the buffer, whatever was skipped before.
    encoder.frame = b * SPDIF_TX_BUFFER_FRAMES;
    for (uint32_t f = 0; f < SPDIF_TX_BUFFER_FRAMES; f += CHUNK_FRAMES) {
        uint32_t w = base + f * SPDIF_WORDS_PER_FRAME;
        if (live && !chunk_clear(b, w))
            return false;
        spdif_encode_frames(&encoder, &pcm[2u * f], CHUNK_FRAMES, &buffers[w]);
    }
    return true;
}

// Register access only (no SDK calls) so the whole handler runs from RAM.
// Late means at least one buffer went out, or will go out, with its
// previous content; consecutive late interrupts count once.
static void __not_in_flash_func(dma_irq_handler)(void) {
    io_rw_32 *ints = &dma_hw->irq_ctrl[DMA_IRQ_INDEX].ints;
    if (!(*ints & data_mask))
        return;
    *ints = data_mask;

    uint32_t now = timer_hw->timerawl;
    uint32_t dt = now - irq_time;
    uint cur = dma_position() / BUFFER_WORDS;
    // Same buffer within one period: this completion was already seen by
    // the previous run (it happened after the flag was cleared).
    if (cur == irq_buf && dt < buf_us)
        return;
    // Same buffer: an even number of completions (>= 2); other buffer
    // after two periods: an odd number (>= 3).
    bool late = cur == irq_buf || dt >= 2u * buf_us;
    irq_buf = cur;
    irq_time = now;
    if (!fill_buffer(cur ^ 1u, true))
        late = true;
    if (late && !late_prev)
        late_irqs++;
    late_prev = late;
}

// Integer divider only: a fractional divider dithers the cell period, which
// shows up as jitter on the S/PDIF line (the trim adds a fraction only on
// request). Returns 0 if out of tolerance.
static uint32_t pio_divider(uint32_t sample_rate) {
    uint32_t clk = clock_get_hz(clk_sys);
    uint32_t target = CELLS_PER_FRAME * sample_rate;
    uint32_t div = (clk + target / 2u) / target;
    if (div < 1u || div > 0xFFFFu)
        return 0;
    uint64_t actual = (uint64_t)div * target;
    uint64_t err = clk > actual ? clk - actual : actual - clk;
    if (err * 1000000u > (uint64_t)MAX_CLOCK_PPM * actual)
        return 0;
    return div;
}

static int32_t rate_error_ppb(uint32_t div, uint32_t sample_rate) {
    uint64_t clk = clock_get_hz(clk_sys);
    uint64_t cells = (uint64_t)CELLS_PER_FRAME * div * sample_rate;
    return (int32_t)((int64_t)((clk * 1000000000u + cells / 2u) / cells) - 1000000000);
}

static bool rate_supported(uint32_t sample_rate) {
    switch (sample_rate) {
    case 32000: case 44100: case 48000: case 88200: case 96000:
        return true;
    default:
        return false;
    }
}

void spdif_tx_init(unsigned pin) {
    tx_pin = pin;
    if (!pio_claim_free_sm_and_add_program_for_gpio_range(&spdif_tx_program, &tx_pio, &tx_sm,
                                                          &tx_offset, pin, 1, true))
        panic("spdif_tx: no free PIO SM");
    dma_data = (uint)dma_claim_unused_channel(true);
    dma_ctrl = (uint)dma_claim_unused_channel(true);
    data_mask = 1u << dma_data;
    dma_mask = data_mask | (1u << dma_ctrl);
    buffer_addr[0] = (uintptr_t)&buffers[0];
    buffer_addr[1] = (uintptr_t)&buffers[BUFFER_WORDS];

    // Pin low before handing it to the PIO so it never floats or glitches high.
    pio_sm_set_pins_with_mask64(tx_pio, tx_sm, 0, 1ull << pin);
    pio_sm_set_consecutive_pindirs(tx_pio, tx_sm, pin, 1, true);
    pio_gpio_init(tx_pio, pin);
    // Sharp edges: the cell rate is up to 12.288 MHz at 96 kHz.
    gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);

    uint irq = (uint)dma_get_irq_num(DMA_IRQ_INDEX);
    irq_add_shared_handler(irq, dma_irq_handler, PICO_SHARED_IRQ_HANDLER_HIGHEST_ORDER_PRIORITY);
    irq_set_priority(irq, DMA_IRQ_PRIORITY);
    irq_set_enabled(irq, true);

    pacer_init();
}

static void configure_dma(void) {
    dma_channel_config c = dma_channel_get_default_config(dma_data);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(tx_pio, tx_sm, true));
    channel_config_set_chain_to(&c, dma_ctrl);
    dma_channel_configure(dma_data, &c, &tx_pio->txf[tx_sm], buffers,
                          dma_encode_transfer_count(BUFFER_WORDS), false);

    // One word per trigger: the next buffer address, which also restarts
    // the data channel.
    c = dma_channel_get_default_config(dma_ctrl);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_ring(&c, false, 3);
    channel_config_set_irq_quiet(&c, true);
    dma_channel_configure(dma_ctrl, &c, &dma_hw->ch[dma_data].al3_read_addr_trig,
                          buffer_addr, dma_encode_transfer_count(1), false);
}

static bool tx_start(uint32_t sample_rate, uint8_t word_bits, spdif_source_fn source, uint32_t options) {
    if (running)
        tx_stop();

    uint8_t cs[SPDIF_CS_BYTES];
    uint32_t div = rate_supported(sample_rate) ? pio_divider(sample_rate) : 0;
    if (div == 0 || source == NULL || spdif_channel_status_build(cs, sample_rate, word_bits) != 0)
        return false;

    source_fn = source;
    tx_rate = sample_rate;
    tx_div = div;
    tx_error_ppb = rate_error_ppb(div, sample_rate);
    trim_ppb = 0;
    trimmed = false;
    tx_bits = word_bits;
    buf_us = (SPDIF_TX_BUFFER_FRAMES * 1000000u + sample_rate / 2u) / sample_rate;
    spdif_encoder_init(&encoder, cs);
    cs_emphasis = false;

    pio_sm_config c = spdif_tx_program_get_default_config(tx_offset);
    sm_config_set_out_pins(&c, tx_pin, 1);
    sm_config_set_out_shift(&c, false, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv_int_frac8(&c, div, 0);
    pio_sm_init(tx_pio, tx_sm, tx_offset, &c);
    configure_dma();

    if (companion && !companion->prepare(sample_rate, options)) {
        tx_rate = 0;
        tx_error_ppb = 0;
        tx_bits = 0;
        return false;
    }
    trimmed = (options & SPDIF_TX_START_TRIMMED) && div <= 255u;
    fill_buffer(0, false);
    fill_buffer(1, false);

    dma_hw->irq_ctrl[DMA_IRQ_INDEX].ints = dma_mask;
    irq_buf = 0;
    late_prev = false;
    irq_time = timer_hw->timerawl;
    dma_irqn_set_channel_mask_enabled(DMA_IRQ_INDEX, data_mask, true);

    running = true;
    // The control channel loads buffer 0 and starts the data channel. DMA
    // first so the FIFO is full before the SM pulls its first word. The
    // companion's state machine starts in the same write.
    dma_channel_start(dma_ctrl);
    uint32_t sm_mask = 1u << tx_sm;
    if (companion)
        sm_mask |= companion->arm();
    pio_enable_sm_mask_in_sync(tx_pio, sm_mask);
    if (trimmed) {
        // clk_sys only changes while the output is stopped.
        uint64_t hz = clock_get_hz(clk_sys);
        pacer_period = (uint32_t)((hz * SPDIF_TX_DITHER_US + 500000u) / 1000000u);
        pacer_start(&pacers[0], &tx_pio->sm[tx_sm].clkdiv, div);
        io_rw_32 *reg = NULL;
        uint32_t cdiv = companion ? companion->trim_divider(&reg) : 0u;
        if (cdiv >= 1u && cdiv <= 255u && reg)
            pacer_start(&pacers[1], reg, cdiv);
    }
    return true;
}

static void tx_stop(void) {
    if (!running)
        return;
    running = false;
    pacer_stop(&pacers[0]);
    pacer_stop(&pacers[1]);
    trimmed = false;
    tx_rate = 0;
    tx_error_ppb = 0;
    trim_ppb = 0;
    tx_bits = 0;

    dma_irqn_set_channel_mask_enabled(DMA_IRQ_INDEX, data_mask, false);
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
    // The abort can raise a spurious completion flag.
    dma_hw->irq_ctrl[DMA_IRQ_INDEX].ints = dma_mask;

    pio_sm_set_enabled(tx_pio, tx_sm, false);
    pio_sm_clear_fifos(tx_pio, tx_sm);
    pio_sm_restart(tx_pio, tx_sm);
    // Idle line: pin driven low, Toslink LED off.
    pio_sm_set_pins_with_mask64(tx_pio, tx_sm, 0, 1ull << tx_pin);
    if (companion)
        companion->stop();
}

PIO spdif_tx_pio(void) {
    return tx_pio;
}

void spdif_tx_set_companion(const spdif_tx_companion_t *c) {
    companion = c;
}

bool spdif_tx_running(void) {
    return running;
}

uint32_t spdif_tx_sample_rate(void) {
    return tx_rate;
}

uint8_t spdif_tx_word_bits(void) {
    return tx_bits;
}

static bool tx_set_word_bits(uint8_t word_bits) {
    uint8_t cs[SPDIF_CS_BYTES];
    if (!running || spdif_channel_status_build(cs, tx_rate, word_bits) != 0)
        return false;
    // The encoder reads cs from the DMA interrupt. The block in flight goes
    // out with mixed old and new bits, which receivers tolerate.
    uint32_t irq = save_and_disable_interrupts();
    spdif_channel_status_set_emphasis(cs, cs_emphasis);
    memcpy(encoder.cs, cs, sizeof(cs));
    tx_bits = word_bits;
    restore_interrupts(irq);
    return true;
}

bool spdif_tx_start(uint32_t sample_rate, uint8_t word_bits, spdif_source_fn source, uint32_t options) {
    change_begin();
    bool ok = tx_start(sample_rate, word_bits, source, options);
    change_end();
    return ok;
}

void spdif_tx_stop(void) {
    change_begin();
    tx_stop();
    change_end();
}

bool spdif_tx_set_word_bits(uint8_t word_bits) {
    change_begin();
    bool ok = tx_set_word_bits(word_bits);
    change_end();
    return ok;
}

bool spdif_tx_set_trim_ppb(int32_t ppb) {
    if (!running || !trimmed)
        return false;
    trim_ppb = spdif_trim_clamp(ppb);
    pacer_set(&pacers[0], trim_ppb);
    int32_t offset = companion && companion->trim_offset_ppb ? companion->trim_offset_ppb() : 0;
    pacer_set(&pacers[1], spdif_trim_clamp(trim_ppb + offset));
    return true;
}

int32_t spdif_tx_trim_ppb(void) {
    return trim_ppb;
}

int32_t spdif_tx_rate_error_ppb(void) {
    return tx_error_ppb;
}

uint32_t spdif_tx_generation(void) {
    return generation;
}

uint32_t spdif_tx_late_irqs(void) {
    return late_irqs;
}
