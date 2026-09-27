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

// Call after spdif_tx_init().
void audio_pipeline_init(void);

// Call every main-loop iteration, after tud_task().
void audio_pipeline_task(void);

// Stream samples are going out (pre-roll done, no underrun since).
bool audio_pipeline_playing(void);

// Times the ring ran dry while playing.
uint32_t audio_pipeline_underruns(void);
