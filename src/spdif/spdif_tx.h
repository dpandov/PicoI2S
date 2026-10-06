// S/PDIF transmitter: PIO state machine fed by ping-pong DMA.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "hardware/pio.h"

// Frames per DMA buffer. The two buffers together are one channel-status
// block (SPDIF_BLOCK_FRAMES).
#define SPDIF_TX_BUFFER_FRAMES 96

// Fills n_frames interleaved L/R samples, 32-bit left-justified (see
// spdif_encode.h). Must always fill all frames; use zeros for silence.
// Called from the DMA interrupt (and twice from spdif_tx_start() to
// pre-fill): keep it short and non-blocking. Normally once per buffer
// played; a late interrupt (see spdif_tx_late_irqs()) skips calls, and in
// rare cases uses only part of the frames.
typedef void (*spdif_source_fn)(int32_t *pcm, uint32_t n_frames);

// Claims a PIO state machine and two DMA channels. Pin is driven low.
void spdif_tx_init(unsigned pin);

// The transmitter's PIO block, for a companion's state machine.
PIO spdif_tx_pio(void);

// A second output run in lockstep from the same frames (i2s/i2s_tx.h):
// its state machine sits on spdif_tx_pio() and is enabled in the same
// write as the S/PDIF one, and it gets every block of frames the S/PDIF
// interrupt fetches. All callbacks are called with interrupts as the
// caller has them; fill() runs in the DMA interrupt.
typedef struct {
    // From spdif_tx_start(), after the S/PDIF state machine and DMA are
    // configured, before the buffers are pre-filled. Set up for
    // sample_rate (program, divider, DMA chain). false fails the start.
    bool (*prepare)(uint32_t sample_rate);
    // The n_frames interleaved L/R samples, 32-bit left-justified, just
    // fetched for buffer b (0 or 1). live: the DMA runs, so write only
    // what it is clear of (the S/PDIF side has already checked its own
    // buffer b is clear at the start).
    void (*fill)(unsigned b, const int32_t *pcm, uint32_t n_frames, bool live);
    // Just before the state machines are enabled: start the DMA chain so
    // the TX FIFO is full. Returns the state-machine mask to enable in
    // sync with the S/PDIF one.
    uint32_t (*arm)(void);
    // From spdif_tx_stop(), after the S/PDIF DMA is aborted and its state
    // machine disabled: stop and park the pins.
    void (*stop)(void);
    // Mirror of spdif_tx_set_trim_ppb() / spdif_tx_trim_task().
    void (*set_trim_ppb)(int32_t ppb);
    void (*trim_task)(void);
} spdif_tx_companion_t;

// Registers (or, with NULL, removes) the companion. Before the first start.
void spdif_tx_set_companion(const spdif_tx_companion_t *c);

// Starts output at sample_rate with word_bits (16 or 24) in the channel
// status. The PIO clock divider must come out as an integer within
// 1000 ppm of 128 x sample_rate for the current clk_sys; otherwise, or for
// an unsupported rate, returns false and stays stopped. Restarts if running.
bool spdif_tx_start(uint32_t sample_rate, uint8_t word_bits, spdif_source_fn source);

// Stops output and drives the pin low (no carrier).
void spdif_tx_stop(void);

bool spdif_tx_running(void);

// Current output format; 0 when stopped.
uint32_t spdif_tx_sample_rate(void);
uint8_t spdif_tx_word_bits(void);

// Changes the word length in the channel status without interrupting the
// output. Returns false if stopped or word_bits is unsupported.
bool spdif_tx_set_word_bits(uint8_t word_bits);

// Rate trim in parts per billion (positive = faster) relative to the
// integer-divider rate, for following an external clock. Clamped to
// +/-SPDIF_TRIM_MAX_PPB (spdif_trim.h). Applied by spdif_tx_trim_task(),
// without re-phasing the output. Reset to 0 by spdif_tx_start(). Returns
// false if stopped. Main loop only.
bool spdif_tx_set_trim_ppb(int32_t ppb);
int32_t spdif_tx_trim_ppb(void);

// Applies the trim by dithering the PIO divider fraction. Call every
// main-loop iteration; returns at once while the trim is 0.
void spdif_tx_trim_task(void);

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
