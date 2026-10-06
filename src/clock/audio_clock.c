#include "clock/audio_clock.h"

#include "hardware/clocks.h"
#include "hardware/sync.h"

// 12 MHz crystal, refdiv 1, VCO 750..1600 MHz, postdiv 1..7 each. clk_sys
// must be an integer multiple of 128 x Fs. Accurate: per rate, the exact
// multiple where one exists in range (32 and 48 kHz), otherwise the best
// in-spec match from an exhaustive search; the clocks are also fast enough
// for the oversampling filter on core 1 (i2s/i2s_filter.h), which rules
// out the exact 61.44 MHz at 96 kHz. 153.6 and 152.4 MHz are 2.4 and 1.6 %
// above the RP2350's 150 MHz rating. Fast: the highest clk_sys below
// 150 MHz, for UAC1, where the clock servo cancels the error and a fine
// trim step matters more.
// Receivers accept +/-1000 ppm; UAC2 hosts follow the feedback.
typedef struct {
    uint32_t rate;
    uint32_t vco_hz;
    uint8_t  postdiv1;
    uint8_t  postdiv2;
} pll_setting_t;

#define SETTINGS 5u

static const pll_setting_t pll_table[2][SETTINGS] = {
    [AUDIO_CLOCK_ACCURATE] = {
        {32000, 1536000000u, 5, 3},   // 102.4 MHz, 0 ppm (div 25)
        {44100, 1524000000u, 5, 2},   // 152.4 MHz, -63 ppm (div 27)
        {48000, 1536000000u, 5, 2},   // 153.6 MHz, 0 ppm (div 25)
        {88200, 1524000000u, 5, 3},   // 101.6 MHz, -63 ppm (div 9)
        {96000, 1032000000u, 7, 1},   // 147.428571 MHz, -186 ppm (div 12)
    },
    [AUDIO_CLOCK_FAST] = {
        {32000, 1032000000u, 7, 1},   // 147.428571 MHz, -186 ppm (div 36)
        {44100,  948000000u, 7, 1},   // 135.428571 MHz, -344 ppm (div 24)
        {48000, 1032000000u, 7, 1},   // (div 24)
        {88200,  948000000u, 7, 1},   // (div 12)
        {96000, 1032000000u, 7, 1},   // (div 12)
    },
};

static const pll_setting_t *table = pll_table[AUDIO_CLOCK_ACCURATE];
static const pll_setting_t *current;
static volatile uint32_t generation;

void audio_clock_init(audio_clock_profile_t profile) {
    table = pll_table[profile == AUDIO_CLOCK_FAST ? AUDIO_CLOCK_FAST : AUDIO_CLOCK_ACCURATE];
    current = NULL;
}

// Unsupported rates get the 48 kHz setting.
static const pll_setting_t *setting_for(uint32_t sample_rate) {
    for (unsigned i = 0; i < SETTINGS; i++) {
        if (table[i].rate == sample_rate) {
            return &table[i];
        }
    }
    return setting_for(48000);
}

static bool same_clock(const pll_setting_t *a, const pll_setting_t *b) {
    return a && b && a->vco_hz == b->vco_hz && a->postdiv1 == b->postdiv1 &&
           a->postdiv2 == b->postdiv2;
}

bool audio_clock_rate_changes(uint32_t sample_rate) {
    return !same_clock(current, setting_for(sample_rate));
}

uint32_t audio_clock_set_rate(uint32_t sample_rate) {
    const pll_setting_t *p = setting_for(sample_rate);
    if (!same_clock(current, p)) {
        generation++;
        __dmb();
        set_sys_clock_pll(p->vco_hz, p->postdiv1, p->postdiv2);

        // Explicit, in case the SDK is configured to move clk_peri with clk_sys.
        clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                        48 * MHZ, 48 * MHZ);
        current = p;
        __dmb();
        generation++;
    }
    return clock_get_hz(clk_sys);
}

uint32_t audio_clock_generation(void) {
    return generation;
}
