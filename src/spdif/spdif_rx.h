// S/PDIF receiver: two inputs (coax, Toslink), each decoded by a PIO state
// machine on a block other than the transmitter's, subframes moved by DMA
// into a ring; the main loop checks preambles and parity, assembles the
// channel status, measures the frame rate against the system timer and
// moves the samples into a FIFO of 32-bit left-justified stereo frames.
//
// Lock: a run of valid frames at one of the supported rates (32, 44.1,
// 48 kHz; 88.2 and 96 kHz as available), measured within the IEC 60958
// level II tolerance. The selected input is the locked one, coax first
// when both are locked (a coax lock takes over from Toslink). Brief errors
// (a few corrupted subframes) do not drop the lock; the bad samples and a
// few around them are replaced by zeros. clk_sys changes
// (clock/audio_clock.h) are followed automatically.
//
// The FIFO fills while an input is selected, read or not (unread, it
// starts over when full); every discontinuity (lock, unlock, input or rate
// change, overflow while read, decoder restart) flushes it and bumps
// spdif_rx_generation().
//
// Main loop only, except where noted.
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SPDIF_RX_NONE = 0,
    SPDIF_RX_COAX,
    SPDIF_RX_TOSLINK,
} spdif_rx_input_t;

// Per-chunk flags returned by spdif_rx_read().
#define SPDIF_RX_FLAG_EMPHASIS (1u << 0)    // 50/15 us pre-emphasis flagged

typedef struct {
    spdif_rx_input_t input;     // selected input; NONE when neither is locked
    uint32_t sample_rate;       // nominal rate of the selected input, 0 if none
    int32_t  rate_ppm;          // measured offset from the nominal rate
    bool     cs_valid;          // a whole channel-status block received since lock
    bool     pcm;               // linear PCM (channel status bit 1 clear)
    bool     emphasis;          // 50/15 us pre-emphasis (current block)
    uint8_t  word_bits;         // from the channel status: 16..24, 24 if not given
    bool     locked_coax;       // each input's own lock, whether selected or not
    bool     locked_toslink;
} spdif_rx_status_t;

typedef struct {
    // Since boot.
    uint32_t locks;             // lock acquisitions (either input)
    uint32_t unlocks;           // lock losses
    uint32_t err_parity;        // subframes with a parity error (selected input)
    uint32_t err_frame;         // preamble / framing errors (selected input)
    uint32_t overflows;         // FIFO (while read) or DMA ring overflows
} spdif_rx_stats_t;

// Claims the PIO state machines and DMA channels, enables the pads as
// inputs with pull-downs (an unconnected input reads a static low) and
// starts listening. Call once after spdif_tx_init().
void spdif_rx_init(unsigned coax_pin, unsigned toslink_pin);

// Every main-loop iteration: decode what arrived, track lock and channel
// status, select the input.
void spdif_rx_task(void);

void spdif_rx_get_status(spdif_rx_status_t *status);
void spdif_rx_get_stats(spdif_rx_stats_t *stats);

// Incremented on every discontinuity of the FIFO's stream (see above).
uint32_t spdif_rx_generation(void);

// Empties the FIFO without a generation change, e.g. before a pre-roll.
void spdif_rx_flush(void);

// Frames waiting in the FIFO.
uint32_t spdif_rx_level(void);

// FIFO capacity in frames.
uint32_t spdif_rx_depth(void);

// Reads up to max_frames interleaved L/R frames, 32-bit left-justified
// (24 significant bits). Stops early where the channel-status flags
// change, so a chunk carries one set of flags (*flags, SPDIF_RX_FLAG_*).
// Non-PCM blocks are delivered as zeros. Returns the frames read.
uint32_t spdif_rx_read(int32_t *pcm, uint32_t max_frames, uint32_t *flags);
