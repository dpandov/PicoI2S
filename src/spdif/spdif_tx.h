// S/PDIF transmitter: PIO state machine fed by ping-pong DMA.
#pragma once

#include <stdbool.h>
#include <stdint.h>

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

// Incremented before and after every start, stop and word-length change:
// odd while the output or its format is changing. For observers on the
// other core.
uint32_t spdif_tx_generation(void);

// Late refills since boot: each time the interrupt came too late (or was
// held off inside) for at least one buffer, which then went out again with
// its previous content (still valid S/PDIF). Not reset by start or stop.
uint32_t spdif_tx_late_irqs(void);
