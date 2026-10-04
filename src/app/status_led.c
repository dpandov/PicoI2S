#include "app/status_led.h"

#include "pico/stdlib.h"
#include "board_config.h"
#include "usb/usb_audio.h"

#if LOCK_LED_WS2812
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"
#endif

// The hold time bridges the pre-roll after an underrun to avoid flicker.
#define LED_AUDIO_HOLD_US     500000u
#define LED_FLASH_COUNT       2u
#define LED_FLASH_ON_US       80000u
#define LED_FLASH_PERIOD_US   200000u

static uint64_t last_audio_us;
static uint64_t flash_start_us;      // 0 = no flash sequence running
static bool     led_on;

#if LOCK_LED_WS2812

// The WS2812 latches a frame after 50 us of low line; a second frame sent
// sooner would be taken for the next LED in a chain. Frames are 30 us.
#define WS2812_FRAME_GAP_US   100u
#define WS2812_BIT_HZ         8000000u    // 10 state-machine cycles per bit

static PIO      led_pio;
static uint     led_sm;
static uint     led_offset;
static bool     led_want;
static bool     led_dirty;
static uint64_t led_write_us;

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
#if LOCK_LED_ORDER_RGB
    uint32_t frame = ((uint32_t)LOCK_LED_R << 16) | ((uint32_t)LOCK_LED_G << 8) | LOCK_LED_B;
#else
    uint32_t frame = ((uint32_t)LOCK_LED_G << 16) | ((uint32_t)LOCK_LED_R << 8) | LOCK_LED_B;
#endif
    pio_sm_put(led_pio, led_sm, led_want ? frame << 8 : 0u);
    led_on = led_want;
    led_dirty = false;
    led_write_us = now;
}

static void led_hw_set(bool on, uint64_t now) {
    if (on != led_want) {
        led_want = on;
        led_dirty = true;
    }
    led_hw_flush(now);
}

#else

static void led_hw_init(void) {
    gpio_init(LOCK_LED_PIN);
    gpio_set_dir(LOCK_LED_PIN, GPIO_OUT);
}

static void led_hw_set(bool on, uint64_t now) {
    (void)now;
    gpio_put(LOCK_LED_PIN, on);
    led_on = on;
}

#endif

void status_led_init(void) {
    led_hw_init();
}

static bool audio_active(uint64_t now, bool stream_open, bool audio_flowing) {
    if (stream_open && audio_flowing) {
        last_audio_us = now;
        return true;
    }
    return stream_open && last_audio_us != 0 && (now - last_audio_us) < LED_AUDIO_HOLD_US;
}

void status_led_task(uint32_t events, bool stream_open, bool audio_flowing) {
    uint64_t now = time_us_64();

    if (events & USB_AUDIO_EVT_MOUNTED) {
        flash_start_us = now;
    }

    if (flash_start_us != 0) {
        uint64_t t = now - flash_start_us;
        if (t < (uint64_t)LED_FLASH_COUNT * LED_FLASH_PERIOD_US) {
            led_hw_set((t % LED_FLASH_PERIOD_US) < LED_FLASH_ON_US, now);
            return;
        }
        flash_start_us = 0;
    }

    led_hw_set(audio_active(now, stream_open, audio_flowing), now);
}

bool status_led_is_on(void) {
    return led_on;
}
