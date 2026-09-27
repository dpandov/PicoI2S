// clk_sys per sample-rate family, chosen so that the S/PDIF PIO divider
// (clk_sys / (128 x Fs)) is an integer for every rate in the family.
#pragma once

#include <stdint.h>

typedef enum {
    AUDIO_FAMILY_48K,     // 32, 48, 96 kHz
    AUDIO_FAMILY_44K1,    // 44.1, 88.2 kHz
} audio_family_t;

audio_family_t audio_clock_family(uint32_t sample_rate);

// Switches clk_sys; clk_peri stays on the 48 MHz USB PLL. Stop the S/PDIF
// output first. Anything derived from clk_sys (SysTick) must be restarted.
void audio_clock_set_family(audio_family_t family);

audio_family_t audio_clock_current_family(void);

// Incremented before and after every switch: odd while clk_sys is changing.
// For observers on the other core.
uint32_t audio_clock_generation(void);
