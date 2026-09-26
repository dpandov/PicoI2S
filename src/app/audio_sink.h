// Consumer of the USB audio stream.
//
// Drains the receive FIFO at exactly the nominal sample rate, timed by the
// crystal-derived microsecond timer, after pre-filling it to half (pre-roll).
#pragma once

#include <stdbool.h>
#include <stdint.h>

void audio_sink_task(void);

// Pre-roll done and audio flowing.
bool audio_sink_running(void);

uint32_t audio_sink_underruns(void);
