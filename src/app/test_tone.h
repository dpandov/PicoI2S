// Test signal for checking the wiring and receiver lock without a host:
// 1 kHz sine at -18 dBFS on both channels, for a 48 kHz S/PDIF stream.
#pragma once

#include <stdint.h>

#define TEST_TONE_SAMPLE_RATE 48000u

// S/PDIF source callback (see spdif_tx.h).
void test_tone_source(int32_t *pcm, uint32_t n_frames);
