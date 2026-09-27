// USB audio stream -> S/PDIF output.
//
// The main loop moves PCM from the USB receive FIFO into a ring of 32-bit
// left-justified stereo frames; the S/PDIF DMA interrupt drains the ring at
// the transmitter's rate. The USB feedback keeps the receive FIFO half full.
//
// The output starts when a stream has pre-filled, and stops (pin idle) after
// 3 s without audio samples: stream closed, or open with nothing arriving.
// Digital silence inside an open stream counts as audio.
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    // Since the current or last stream started (opened or changed format).
    uint32_t underruns;       // ring ran dry while playing
    uint32_t overruns;        // USB packets that overwrote unread FIFO data
    uint32_t late_irqs;       // S/PDIF interrupts too late to refill in time
    uint32_t ring_min;        // lowest ring level while playing, in frames
    // Since the stream first started playing (excludes the start-up).
    uint32_t fifo_max;        // highest receive FIFO level, in bytes
    uint32_t loop_max_us;     // longest main-loop iteration
    // Since boot.
    uint32_t underruns_total;
    uint32_t overruns_total;
} audio_pipeline_stats_t;

// Call after spdif_tx_init().
void audio_pipeline_init(void);

// Call every main-loop iteration, after tud_task().
void audio_pipeline_task(void);

// Stream samples are going out (pre-roll done, no underrun since).
bool audio_pipeline_playing(void);

void audio_pipeline_get_stats(audio_pipeline_stats_t *stats);
