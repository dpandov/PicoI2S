#include "clock/audio_clock.h"

#include "hardware/clocks.h"
#include "hardware/sync.h"

// 12 MHz crystal, refdiv 1, VCO 750..1600 MHz, postdiv 1..7 each. clk_sys
// must be an integer multiple of 128 x 96 kHz (48k family) or 128 x 88.2 kHz
// (44.1k family). Accurate: the best in-spec match from an exhaustive search;
// the 48k family is exact, the 44.1k family cannot be (it would need a VCO
// above 1600 MHz). Fast: the highest clk_sys below 150 MHz, for UAC1, where
// the clock servo cancels the error and a fine trim step matters more.
// Receivers accept +/-1000 ppm; UAC2 hosts follow the feedback.
typedef struct {
    uint32_t vco_hz;
    uint8_t  postdiv1;
    uint8_t  postdiv2;
} pll_setting_t;

static const pll_setting_t pll_table[2][2] = {
    [AUDIO_CLOCK_ACCURATE] = {
        [AUDIO_FAMILY_48K]  = {1536000000u, 5, 5},   //  61.44 MHz, 0 ppm (div 15 / 10 / 5)
        [AUDIO_FAMILY_44K1] = {1524000000u, 5, 3},   // 101.6 MHz, -63 ppm (div 18 / 9)
    },
    [AUDIO_CLOCK_FAST] = {
        [AUDIO_FAMILY_48K]  = {1032000000u, 7, 1},   // 147.428571 MHz, -186 ppm (36 / 24 / 12)
        [AUDIO_FAMILY_44K1] = { 948000000u, 7, 1},   // 135.428571 MHz, -344 ppm (24 / 12)
    },
};

static const pll_setting_t *table = pll_table[AUDIO_CLOCK_ACCURATE];
static audio_family_t current = AUDIO_FAMILY_48K;
static volatile uint32_t generation;

void audio_clock_init(audio_clock_profile_t profile) {
    table = pll_table[profile == AUDIO_CLOCK_FAST ? AUDIO_CLOCK_FAST : AUDIO_CLOCK_ACCURATE];
}

audio_family_t audio_clock_family(uint32_t sample_rate) {
    return (sample_rate % 11025u == 0) ? AUDIO_FAMILY_44K1 : AUDIO_FAMILY_48K;
}

void audio_clock_set_family(audio_family_t family) {
    const pll_setting_t *p = &table[family == AUDIO_FAMILY_44K1 ? AUDIO_FAMILY_44K1 : AUDIO_FAMILY_48K];
    generation++;
    __dmb();
    set_sys_clock_pll(p->vco_hz, p->postdiv1, p->postdiv2);

    // Explicit, in case the SDK is configured to move clk_peri with clk_sys.
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ, 48 * MHZ);
    current = family;
    __dmb();
    generation++;
}

uint32_t audio_clock_generation(void) {
    return generation;
}

audio_family_t audio_clock_current_family(void) {
    return current;
}
