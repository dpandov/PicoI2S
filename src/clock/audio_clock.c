#include "clock/audio_clock.h"

#include "hardware/clocks.h"

// 12 MHz crystal, refdiv 1, VCO 750..1600 MHz, postdiv 1..7. Each setting is
// the closest integer-PLL match to 12 x 128 x the family's 96/88.2 kHz rate,
// kept below 150 MHz so no core voltage change is needed. The residual error
// is absorbed by the USB feedback; receivers accept +/-1000 ppm.
typedef struct {
    uint32_t vco_hz;
    uint8_t  postdiv1;
    uint8_t  postdiv2;
} pll_setting_t;

static const pll_setting_t pll_48k  = {1032000000u, 7, 1};  // 147.428571 MHz, -186 ppm
static const pll_setting_t pll_44k1 = { 948000000u, 7, 1};  // 135.428571 MHz, -344 ppm

static audio_family_t current = AUDIO_FAMILY_48K;

audio_family_t audio_clock_family(uint32_t sample_rate) {
    return (sample_rate % 11025u == 0) ? AUDIO_FAMILY_44K1 : AUDIO_FAMILY_48K;
}

void audio_clock_set_family(audio_family_t family) {
    const pll_setting_t *p = (family == AUDIO_FAMILY_44K1) ? &pll_44k1 : &pll_48k;
    set_sys_clock_pll(p->vco_hz, p->postdiv1, p->postdiv2);

    // Explicit, in case the SDK is configured to move clk_peri with clk_sys.
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ, 48 * MHZ);
    current = family;
}

audio_family_t audio_clock_current_family(void) {
    return current;
}
