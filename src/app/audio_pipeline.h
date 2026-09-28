// USB audio stream -> S/PDIF output.
//
// The main loop moves PCM from the USB receive FIFO into a ring of 32-bit
// left-justified stereo frames; the S/PDIF DMA interrupt drains the ring at
// the transmitter's rate. The receive FIFO is held half full: in UAC2 mode
// by the USB feedback, in UAC1 mode (adaptive) by trimming the transmitter
// rate to the host's (app/clock_servo.h).
//
// The output starts when a stream has pre-filled, and stops (pin idle) after
// 3 s without audio samples: stream closed, or open with nothing arriving.
// Digital silence inside an open stream counts as audio.
//
// The host's volume (usb_audio_volume()) is applied on the way into the
// ring, ramped 1 dB per chunk read: at 0 dB the samples pass unchanged;
// below it they are scaled, rounded to 24 bits and sent as 24-bit words
// whatever the stream depth. Muted streams send zeros and still count as
// audio.
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
    // Now.
    int32_t  trim_ppb;        // transmitter rate trim (0 in UAC2 mode)
    bool     clock_locked;    // UAC1: playing and the clock servo locked
} audio_pipeline_stats_t;

// Call after spdif_tx_init().
void audio_pipeline_init(void);

// Call every main-loop iteration, after tud_task(). Also applies the
// transmitter rate trim (spdif_tx_trim_task()).
void audio_pipeline_task(void);

// Stream samples are going out (pre-roll done, no underrun since).
bool audio_pipeline_playing(void);

void audio_pipeline_get_stats(audio_pipeline_stats_t *stats);
