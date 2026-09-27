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

#include "spdif/spdif_tx.h"

#include <string.h>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "pico.h"

#include "spdif/spdif_encode.h"
#include "spdif/spdif_trim.h"
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
// 16 frames (167 us at 96 kHz) against about 10 us to encode a chunk.
#define GUARD_WORDS  (16u * SPDIF_WORDS_PER_FRAME)

_Static_assert(SPDIF_TX_BUFFER_FRAMES % CHUNK_FRAMES == 0, "whole chunks per buffer");

#define DMA_IRQ_INDEX 1
// Above USB (default 0x80) so encoding is never delayed by USB handling,
// but below the diagnostic SysTick (0) so a hang in here is still caught.
#define DMA_IRQ_PRIORITY 0x40

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
static volatile bool running;
static uint32_t tx_rate;
static uint32_t tx_div;          // integer PIO divider at zero trim
static int32_t tx_error_ppb;
static uint8_t tx_bits;
static volatile uint32_t generation;
static volatile uint32_t late_irqs;
static bool late_prev;          // the last interrupt was late

// Main loop only.
static spdif_trim_t trim;
static int32_t trim_ppb;
static uint32_t trim_reg;       // CLKDIV value last written

// State of the last serviced interrupt.
static uint irq_buf;            // buffer the DMA was reading
static uint32_t irq_time;       // timer, us
static uint32_t buf_us;         // one buffer period, us

static void tx_stop(void);

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
    source_fn(pcm, SPDIF_TX_BUFFER_FRAMES);
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

static bool tx_start(uint32_t sample_rate, uint8_t word_bits, spdif_source_fn source) {
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
    spdif_trim_reset(&trim, div < 256u ? div : 0u, timer_hw->timerawl);
    trim_reg = spdif_trim_clkdiv_reg(trim.applied);
    tx_bits = word_bits;
    buf_us = (SPDIF_TX_BUFFER_FRAMES * 1000000u + sample_rate / 2u) / sample_rate;
    spdif_encoder_init(&encoder, cs);
    fill_buffer(0, false);
    fill_buffer(1, false);

    pio_sm_config c = spdif_tx_program_get_default_config(tx_offset);
    sm_config_set_out_pins(&c, tx_pin, 1);
    sm_config_set_out_shift(&c, false, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv_int_frac8(&c, div, 0);
    pio_sm_init(tx_pio, tx_sm, tx_offset, &c);

    configure_dma();
    dma_hw->irq_ctrl[DMA_IRQ_INDEX].ints = dma_mask;
    irq_buf = 0;
    late_prev = false;
    irq_time = timer_hw->timerawl;
    dma_irqn_set_channel_mask_enabled(DMA_IRQ_INDEX, data_mask, true);

    running = true;
    // The control channel loads buffer 0 and starts the data channel. DMA
    // first so the FIFO is full before the SM pulls its first word.
    dma_channel_start(dma_ctrl);
    pio_sm_set_enabled(tx_pio, tx_sm, true);
    return true;
}

static void tx_stop(void) {
    if (!running)
        return;
    running = false;
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
    memcpy(encoder.cs, cs, sizeof(cs));
    tx_bits = word_bits;
    restore_interrupts(irq);
    return true;
}

bool spdif_tx_start(uint32_t sample_rate, uint8_t word_bits, spdif_source_fn source) {
    change_begin();
    bool ok = tx_start(sample_rate, word_bits, source);
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

// Writing CLKDIV while running does not restart the divider (only
// CTRL.CLKDIV_RESTART does), so the cell phase stays continuous.
bool spdif_tx_set_trim_ppb(int32_t ppb) {
    if (!running || tx_div > 255u)
        return false;
    trim_ppb = spdif_trim_clamp(ppb);
    spdif_trim_set_target(&trim, spdif_trim_divider(tx_div, trim_ppb), timer_hw->timerawl);
    return true;
}

int32_t spdif_tx_trim_ppb(void) {
    return trim_ppb;
}

void spdif_tx_trim_task(void) {
    // On a whole FRAC step there is nothing to dither.
    if (!running || trim.applied == trim.target)
        return;
    uint32_t reg = spdif_trim_clkdiv_reg(spdif_trim_dither(&trim, timer_hw->timerawl));
    if (reg != trim_reg) {
        trim_reg = reg;
        tx_pio->sm[tx_sm].clkdiv = reg;
    }
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
