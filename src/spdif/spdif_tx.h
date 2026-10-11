// S/PDIF transmitter: PIO state machine fed by ping-pong DMA.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "hardware/pio.h"

// Frames per DMA buffer. The two buffers together are one channel-status
// block (SPDIF_BLOCK_FRAMES).
#define SPDIF_TX_BUFFER_FRAMES 96

// Flags a source returns for the frames it filled.
#define SPDIF_TX_FLAG_EMPHASIS (1u << 0)   // 50/15 us pre-emphasis: flagged in
                                           // the channel status, passed to the
                                           // companion

// Fills n_frames interleaved L/R samples, 32-bit left-justified (see
// spdif_encode.h), and returns their SPDIF_TX_FLAG_* bits. Must always
// fill all frames; use zeros for silence.
// Called from the DMA interrupt (and twice from spdif_tx_start() to
// pre-fill): keep it short and non-blocking. Normally once per buffer
// played; a late interrupt (see spdif_tx_late_irqs()) skips calls, and in
// rare cases uses only part of the frames.
typedef uint32_t (*spdif_source_fn)(int32_t *pcm, uint32_t n_frames);

// Claims a PIO state machine and two DMA channels for the output, and for
// the rate trim two pacer state machines on pio0 and four DMA channels.
// Pin is driven low.
void spdif_tx_init(unsigned pin);

// The transmitter's PIO block, for a companion's state machine.
PIO spdif_tx_pio(void);

// A second output run in lockstep from the same frames (i2s/i2s_tx.h):
// its state machine sits on spdif_tx_pio() and is enabled in the same
// write as the S/PDIF one, and it gets every block of frames the S/PDIF
// interrupt fetches. Under a trim its pacer rounds the rate apart from the
// S/PDIF one; trim_offset_ppb() lets it hold its phase. All callbacks are
// called with interrupts as the caller has them; fill() runs in the DMA
// interrupt.
typedef struct {
    // From spdif_tx_start(), after the S/PDIF state machine and DMA are
    // configured, before the buffers are pre-filled. Set up for
    // sample_rate (program, divider, DMA chain) with the start's
    // SPDIF_TX_START_* options. false fails the start.
    bool (*prepare)(uint32_t sample_rate, uint32_t options);
    // The n_frames interleaved L/R samples, 32-bit left-justified, just
    // fetched for buffer b (0 or 1), with the source's SPDIF_TX_FLAG_*
    // bits. live: the DMA runs, so write only what it is clear of (the
    // S/PDIF side has already checked its own buffer b is clear at the
    // start).
    void (*fill)(unsigned b, const int32_t *pcm, uint32_t n_frames, bool live, uint32_t flags);
    // Just before the state machines are enabled: start the DMA chain so
    // the TX FIFO is full. Returns the state-machine mask to enable in
    // sync with the S/PDIF one.
    uint32_t (*arm)(void);
    // From spdif_tx_stop(), after the S/PDIF DMA is aborted and its state
    // machine disabled: stop and park the pins.
    void (*stop)(void);
    // After the state machines of an SPDIF_TX_START_TRIMMED start are
    // enabled: the companion's state machine follows the trim through its
    // own divider, switched by a pacer of its own. Returns that integer
    // divider at zero trim (1..255; 0: no trim) and sets *reg to the one
    // CLKDIV register that takes it.
    uint32_t (*trim_divider)(io_rw_32 **reg);
    // From spdif_tx_set_trim_ppb() on a trimmed output (main loop): the
    // companion's rate offset in ppb (positive = faster) added to the trim
    // for its pacer only; 0 for none. May be NULL.
    int32_t (*trim_offset_ppb)(void);
} spdif_tx_companion_t;

// Registers (or, with NULL, removes) the companion. Before the first start.
void spdif_tx_set_companion(const spdif_tx_companion_t *c);

// spdif_tx_start() options.
#define SPDIF_TX_START_TRIMMED  (1u << 0)   // the rate will follow an external
                                            // clock (spdif_tx_set_trim_ppb())
#define SPDIF_TX_START_EMPHASIS (1u << 1)   // the source may flag pre-emphasis
                                            // (SPDIF_TX_FLAG_EMPHASIS)

// Starts output at sample_rate with word_bits (16 or 24) in the channel
// status. The PIO clock divider must come out as an integer within
// 1000 ppm of 128 x sample_rate for the current clk_sys; otherwise, or for
// an unsupported rate, returns false and stays stopped. Restarts if
// running. options: SPDIF_TX_START_* bits.
bool spdif_tx_start(uint32_t sample_rate, uint8_t word_bits, spdif_source_fn source, uint32_t options);

// Stops output and drives the pin low (no carrier).
void spdif_tx_stop(void);

bool spdif_tx_running(void);

// Turns the line off (pin held low, no carrier) or back on while the
// output runs: the state machine, the source and the companion carry on.
// spdif_tx_start() turns it on.
void spdif_tx_set_line(bool on);
// Running with the line on.
bool spdif_tx_line_on(void);

// Current output format; 0 when stopped.
uint32_t spdif_tx_sample_rate(void);
uint8_t spdif_tx_word_bits(void);

// Changes the word length in the channel status without interrupting the
// output. Returns false if stopped or word_bits is unsupported.
bool spdif_tx_set_word_bits(uint8_t word_bits);

// Rate trim in parts per billion (positive = faster) relative to the
// integer-divider rate, for following an external clock. Clamped to
// +/-SPDIF_TRIM_MAX_PPB (spdif_trim.h). Applied by switching the PIO
// divider fraction between its two adjacent values in fixed periods
// (SPDIF_TX_DITHER_US), timed by a pacer state machine and DMA, without
// re-phasing the output. The companion's pacer gets the trim plus its
// trim_offset_ppb(). Reset to 0 by spdif_tx_start(). Returns false if
// stopped or started without SPDIF_TX_START_TRIMMED. Main loop only.
bool spdif_tx_set_trim_ppb(int32_t ppb);
int32_t spdif_tx_trim_ppb(void);

// Trim period: each trimmed divider holds its upper fraction value for a
// part of every period, so the output clocks carry a phase triangle at
// its frequency (40 kHz) of at most a quarter of one divider-fraction
// step over this time (spdif_trim.h).
#define SPDIF_TX_DITHER_US 25u

// Rate error of the running output at zero trim against its nominal sample
// rate, in ppb, from the configured clk_sys (see clock/audio_clock.c). 0
// when stopped.
int32_t spdif_tx_rate_error_ppb(void);

// Incremented before and after every start, stop and word-length change:
// odd while the output or its format is changing. For observers on the
// other core.
uint32_t spdif_tx_generation(void);

// Late refills since boot: each time the interrupt came too late (or was
// held off inside) for at least one buffer, which then went out again with
// its previous content (still valid S/PDIF). Not reset by start or stop.
uint32_t spdif_tx_late_irqs(void);
