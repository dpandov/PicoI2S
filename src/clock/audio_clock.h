// clk_sys per sample rate, chosen so that the S/PDIF PIO divider
// (clk_sys / (128 x Fs)) is an integer, with the smallest rate error the
// PLL allows.
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    AUDIO_CLOCK_ACCURATE,   // lowest PLL error: the Pico is the clock master (UAC2)
    AUDIO_CLOCK_FAST,       // highest clk_sys: the clock servo cancels the error (UAC1)
} audio_clock_profile_t;

// Selects the PLL table. Call once, before the first audio_clock_set_rate().
void audio_clock_init(audio_clock_profile_t profile);

// True if audio_clock_set_rate(sample_rate) would change clk_sys.
bool audio_clock_rate_changes(uint32_t sample_rate);

// Switches clk_sys to the setting for sample_rate (nothing happens if it is
// already there); clk_peri stays on the 48 MHz USB PLL. Stop the S/PDIF
// output first. Anything derived from clk_sys (SysTick) must be restarted.
// Returns the new clk_sys in Hz.
uint32_t audio_clock_set_rate(uint32_t sample_rate);

// Incremented before and after every switch: odd while clk_sys is changing.
// For observers on the other core.
uint32_t audio_clock_generation(void);
