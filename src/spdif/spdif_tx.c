// S/PDIF transmitter: PIO state machine fed by ping-pong DMA.
//
// Two DMA channels chain to each other, each playing one buffer. When a
// channel finishes, the other is already playing; the completion interrupt
// rewinds the finished channel and re-encodes its buffer from the source.

#include "spdif/spdif_tx.h"

#include <string.h>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "pico.h"

#include "spdif/spdif_encode.h"
#include "spdif_tx.pio.h"

#define CELLS_PER_FRAME 128u
#define BUFFER_WORDS    (SPDIF_TX_BUFFER_FRAMES * SPDIF_WORDS_PER_FRAME)
#define MAX_CLOCK_PPM   1000u

#define DMA_IRQ_INDEX 1
// Above USB (default 0x80) so encoding is never delayed by USB handling,
// but below the diagnostic SysTick (0) so a hang in here is still caught.
#define DMA_IRQ_PRIORITY 0x40

static PIO tx_pio;
static uint tx_sm;
static uint tx_offset;
static uint tx_pin;
static uint dma_ch[2];
static uint32_t dma_mask;

static uint32_t buffers[2][BUFFER_WORDS];
static int32_t pcm[SPDIF_TX_BUFFER_FRAMES * 2];
static spdif_encoder_t encoder;
static spdif_source_fn source_fn;
static volatile bool running;
static uint32_t tx_rate;
static uint8_t tx_bits;
// Buffer whose channel completes next; buffers finish strictly alternately.
static uint next_buf;

static void __not_in_flash_func(fill_buffer)(uint b) {
    source_fn(pcm, SPDIF_TX_BUFFER_FRAMES);
    spdif_encode_frames(&encoder, pcm, SPDIF_TX_BUFFER_FRAMES, buffers[b]);
}

// Register access only (no SDK calls) so the whole handler runs from RAM.
static void __not_in_flash_func(dma_irq_handler)(void) {
    io_rw_32 *ints = &dma_hw->irq_ctrl[DMA_IRQ_INDEX].ints;
    for (;;) {
        uint ch = dma_ch[next_buf];
        if (!(*ints & (1u << ch)))
            break;
        *ints = 1u << ch;
        // Non-triggering alias; the chain from the other channel restarts it.
        dma_hw->ch[ch].read_addr = (uintptr_t)buffers[next_buf];
        fill_buffer(next_buf);
        next_buf ^= 1u;
    }
}

static void configure_dma(uint b) {
    dma_channel_config c = dma_channel_get_default_config(dma_ch[b]);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(tx_pio, tx_sm, true));
    channel_config_set_chain_to(&c, dma_ch[b ^ 1u]);
    dma_channel_configure(dma_ch[b], &c, &tx_pio->txf[tx_sm], buffers[b],
                          BUFFER_WORDS, false);
}

// Integer divider only: a fractional divider dithers the cell period, which
// shows up as jitter on the S/PDIF line. Returns 0 if out of tolerance.
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
    dma_ch[0] = (uint)dma_claim_unused_channel(true);
    dma_ch[1] = (uint)dma_claim_unused_channel(true);
    dma_mask = (1u << dma_ch[0]) | (1u << dma_ch[1]);

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

bool spdif_tx_start(uint32_t sample_rate, uint8_t word_bits, spdif_source_fn source) {
    if (running)
        spdif_tx_stop();

    uint8_t cs[SPDIF_CS_BYTES];
    uint32_t div = rate_supported(sample_rate) ? pio_divider(sample_rate) : 0;
    if (div == 0 || source == NULL || spdif_channel_status_build(cs, sample_rate, word_bits) != 0)
        return false;

    source_fn = source;
    tx_rate = sample_rate;
    tx_bits = word_bits;
    spdif_encoder_init(&encoder, cs);
    fill_buffer(0);
    fill_buffer(1);
    next_buf = 0;

    pio_sm_config c = spdif_tx_program_get_default_config(tx_offset);
    sm_config_set_out_pins(&c, tx_pin, 1);
    sm_config_set_out_shift(&c, false, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv_int_frac8(&c, div, 0);
    pio_sm_init(tx_pio, tx_sm, tx_offset, &c);

    configure_dma(0);
    configure_dma(1);
    dma_hw->irq_ctrl[DMA_IRQ_INDEX].ints = dma_mask;
    dma_irqn_set_channel_mask_enabled(DMA_IRQ_INDEX, dma_mask, true);

    running = true;
    // DMA first so the FIFO is full before the SM pulls its first word.
    dma_channel_start(dma_ch[0]);
    pio_sm_set_enabled(tx_pio, tx_sm, true);
    return true;
}

void spdif_tx_stop(void) {
    if (!running)
        return;
    running = false;
    tx_rate = 0;
    tx_bits = 0;

    dma_irqn_set_channel_mask_enabled(DMA_IRQ_INDEX, dma_mask, false);
    // RP2350-E5: an aborted channel can be re-triggered through CHAIN_TO, so
    // clear EN on both chained channels (non-triggering CTRL alias) before
    // the abort.
    hw_clear_bits(&dma_hw->ch[dma_ch[0]].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    hw_clear_bits(&dma_hw->ch[dma_ch[1]].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    dma_hw->abort = dma_mask;
    while (dma_hw->abort & dma_mask)
        tight_loop_contents();
    while ((dma_hw->ch[dma_ch[0]].ctrl_trig | dma_hw->ch[dma_ch[1]].ctrl_trig) &
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

bool spdif_tx_set_word_bits(uint8_t word_bits) {
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
