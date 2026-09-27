#include "app/test_tone.h"

#include "pico.h"

// One period of 1 kHz at 48 kHz, 24-bit: round(8388607 x 10^(-18/20) x
// sin(2 pi k / 48)). Peak 1056063 = -18.00 dBFS.
static const int32_t __not_in_flash("test_tone") tone_24bit[48] = {
           0,   137844,   273329,   404138,   528032,   642890,
      746749,   837831,   914577,   975675,  1020079,  1047028,
     1056063,  1047028,  1020079,   975675,   914577,   837831,
      746749,   642890,   528032,   404138,   273329,   137844,
           0,  -137844,  -273329,  -404138,  -528032,  -642890,
     -746749,  -837831,  -914577,  -975675, -1020079, -1047028,
    -1056063, -1047028, -1020079,  -975675,  -914577,  -837831,
     -746749,  -642890,  -528032,  -404138,  -273329,  -137844,
};

// Runs from the DMA interrupt; the table is in RAM for the same reason.
void __not_in_flash_func(test_tone_source)(int32_t *pcm, uint32_t n_frames) {
    static uint32_t phase;
    for (uint32_t i = 0; i < n_frames; i++) {
        int32_t s = (int32_t)((uint32_t)tone_24bit[phase] << 8);   // left-justify
        pcm[2 * i]     = s;
        pcm[2 * i + 1] = s;
        if (++phase == 48u) {
            phase = 0;
        }
    }
}
