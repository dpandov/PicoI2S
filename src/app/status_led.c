#include "app/status_led.h"

#include "pico/stdlib.h"
#include "board_config.h"
#include "usb/usb_audio.h"

#if LOCK_LED_WS2812
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"
#endif

// The hold time bridges the pre-roll after an underrun or a brief input
// stall to avoid flicker.
#define LED_AUDIO_HOLD_US     500000u
#define LED_FLASH_COUNT       2u
#define LED_FLASH_ON_US       80000u
#define LED_FLASH_PERIOD_US   200000u

typedef enum {
    LED_OFF = 0,
    LED_USB,
    LED_INPUT,
    LED_INPUT_EMPHASIS,
} led_colour_t;

static uint64_t       last_audio_us;
static audio_source_t last_source;     // source last heard
static led_colour_t   last_colour;
static uint64_t       flash_start_us;  // 0 = no flash sequence running
static bool           led_on;

#if LOCK_LED_WS2812

// The WS2812 latches a frame after 50 us of low line; a second frame sent
// sooner would be taken for the next LED in a chain. Frames are 30 us.
#define WS2812_FRAME_GAP_US   100u
#define WS2812_BIT_HZ         8000000u    // 10 state-machine cycles per bit

static PIO          led_pio;
static uint         led_sm;
static uint         led_offset;
static led_colour_t led_want;
static bool         led_dirty;
static uint64_t     led_write_us;

static void led_hw_init(void) {
    hard_assert(pio_claim_free_sm_and_add_program_for_gpio_range(&ws2812_program, &led_pio, &led_sm,
                                                                 &led_offset, LOCK_LED_PIN, 1, true));
    pio_gpio_init(led_pio, LOCK_LED_PIN);
    pio_sm_set_consecutive_pindirs(led_pio, led_sm, LOCK_LED_PIN, 1, true);
    pio_sm_config c = ws2812_program_get_default_config(led_offset);
    sm_config_set_sideset_pins(&c, LOCK_LED_PIN);
    sm_config_set_out_shift(&c, false, true, 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    pio_sm_init(led_pio, led_sm, led_offset, &c);
    pio_sm_set_enabled(led_pio, led_sm, true);
    led_dirty = true;                 // first frame turns the LED off
}

static uint32_t led_frame(uint8_t r, uint8_t g, uint8_t b) {
#if LOCK_LED_ORDER_RGB
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
#else
    return ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
#endif
}

static uint32_t led_colour_frame(led_colour_t c) {
    switch (c) {
    case LED_USB:            return led_frame(LOCK_LED_R, LOCK_LED_G, LOCK_LED_B);
    case LED_INPUT:          return led_frame(LOCK_LED_INPUT_R, LOCK_LED_INPUT_G, LOCK_LED_INPUT_B);
    case LED_INPUT_EMPHASIS: return led_frame(LOCK_LED_EMPH_R, LOCK_LED_EMPH_G, LOCK_LED_EMPH_B);
    default:                 return 0u;
    }
}

// Sends the wanted state once the previous frame has latched. clk_sys
// changes with the sample-rate family, so the divider is set per frame.
static void led_hw_flush(uint64_t now) {
    if (!led_dirty || now - led_write_us < WS2812_FRAME_GAP_US) {
        return;
    }
    uint32_t sys = clock_get_hz(clk_sys);
    uint32_t div_int = sys / WS2812_BIT_HZ;
    uint32_t div_frac = ((sys % WS2812_BIT_HZ) * 256u) / WS2812_BIT_HZ;
    pio_sm_set_clkdiv_int_frac8(led_pio, led_sm, (uint16_t)div_int, (uint8_t)div_frac);
    pio_sm_put(led_pio, led_sm, led_colour_frame(led_want) << 8);
    led_on = led_want != LED_OFF;
    led_dirty = false;
    led_write_us = now;
}

static void led_hw_set(led_colour_t c, uint64_t now) {
    if (c != led_want) {
        led_want = c;
        led_dirty = true;
    }
    led_hw_flush(now);
}

#else

static void led_hw_init(void) {
    gpio_init(LOCK_LED_PIN);
    gpio_set_dir(LOCK_LED_PIN, GPIO_OUT);
    gpio_init(INPUT_EMPH_LED_PIN);
    gpio_set_dir(INPUT_EMPH_LED_PIN, GPIO_OUT);
}

// The EMPH LED shows what the RP2350-One shows as yellow.
static void led_hw_set(led_colour_t c, uint64_t now) {
    (void)now;
    led_on = c != LED_OFF;
    gpio_put(LOCK_LED_PIN, led_on);
    gpio_put(INPUT_EMPH_LED_PIN, c == LED_INPUT_EMPHASIS);
}

#endif

void status_led_init(void) {
    led_hw_init();
#ifdef INPUT_LOCK_LED_PIN
    gpio_init(INPUT_LOCK_LED_PIN);
    gpio_set_dir(INPUT_LOCK_LED_PIN, GPIO_OUT);
#endif
}

static led_colour_t source_colour(const audio_pipeline_source_t *s) {
    switch (s->heard) {
    case AUDIO_SOURCE_USB:     return LED_USB;
    case AUDIO_SOURCE_COAX:
    case AUDIO_SOURCE_TOSLINK: return s->emphasis ? LED_INPUT_EMPHASIS : LED_INPUT;
    default:                   return LED_OFF;
    }
}

// The hold keeps the last colour while the source it showed still owns the
// output (USB stream open, input locked).
static led_colour_t audio_colour(uint64_t now, const audio_pipeline_source_t *s) {
    if (s->heard != AUDIO_SOURCE_NONE) {
        last_audio_us = now;
        last_source = s->heard;
        last_colour = source_colour(s);
        return last_colour;
    }
    bool held = s->owner != AUDIO_SOURCE_NONE && s->owner == last_source &&
                last_audio_us != 0 && (now - last_audio_us) < LED_AUDIO_HOLD_US;
    return held ? last_colour : LED_OFF;
}

void status_led_task(uint32_t events, const audio_pipeline_source_t *source) {
    uint64_t now = time_us_64();

#ifdef INPUT_LOCK_LED_PIN
    gpio_put(INPUT_LOCK_LED_PIN, source->input.locked_coax || source->input.locked_toslink);
#endif

    if (events & USB_AUDIO_EVT_MOUNTED) {
        flash_start_us = now;
    }

    if (flash_start_us != 0) {
        uint64_t t = now - flash_start_us;
        if (t < (uint64_t)LED_FLASH_COUNT * LED_FLASH_PERIOD_US) {
            led_hw_set((t % LED_FLASH_PERIOD_US) < LED_FLASH_ON_US ? LED_USB : LED_OFF, now);
            return;
        }
        flash_start_us = 0;
    }

    led_hw_set(audio_colour(now, source), now);
}

bool status_led_is_on(void) {
    return led_on;
}
