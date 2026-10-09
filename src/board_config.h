// Board pin assignments. Wiring is described in README.md. The board is a
// build option (USB2SPDIF_BOARD in CMakeLists.txt): the Waveshare
// RP2350-One by default, or the Raspberry Pi Pico 2. Same chip, flash and
// crystal, different pins.
#pragma once

#if !USB2SPDIF_BOARD_RP2350_ONE && !USB2SPDIF_BOARD_PICO2
#error "USB2SPDIF_BOARD not set (CMakeLists.txt)"
#endif

// S/PDIF data to the Toslink transmitter input.
// I2S / dual-line output (i2s/i2s_tx.h): DATA (DL in the dual-line modes)
// and DR are consecutive PIO out pins; BCLK and LRCK (LATCH in the
// dual-line modes) are consecutive side-set pins; CLOCK carries the
// 256 Fs MCLK in the I2S / LJ / RJ formats and the TDA1541 DEM clock in
// dual-line 16 (low in the other dual-line formats).
#if USB2SPDIF_BOARD_RP2350_ONE
// S/PDIF at the end of the GP0..GP13 row (the short side of the board);
// I2S in the row next to 3V3: GP28, 27, 26, 15, 14 in order.
#define SPDIF_PIN      12
#define I2S_DATA_PIN   14
#define I2S_DATA_R_PIN 15
#define I2S_BCLK_PIN   26
#define I2S_LRCK_PIN   27
#define I2S_CLOCK_PIN  28
#else
// Pico 2: S/PDIF on physical pin 34 (GND on 33, 3V3 on 36); I2S on 21,
// 22, 24, 25, 26 (GND on 23 and 28).
#define SPDIF_PIN      28
#define I2S_DATA_PIN   16
#define I2S_DATA_R_PIN 17
#define I2S_BCLK_PIN   18
#define I2S_LRCK_PIN   19
#define I2S_CLOCK_PIN  20
#endif

// S/PDIF inputs (spdif/spdif_rx.h), played when USB is not available: a
// coax receiver (75 ohm termination, AC coupling, self-biased inverter or
// comparator) and a 3.3 V Toslink receiver module. Coax has priority.
#if USB2SPDIF_BOARD_RP2350_ONE
#define SPDIF_IN_COAX_PIN    10
#define SPDIF_IN_TOSLINK_PIN 11
#else
// Pico 2: physical pins 31 and 32, beside GND (33) and the S/PDIF output.
#define SPDIF_IN_COAX_PIN    26
#define SPDIF_IN_TOSLINK_PIN 27
#endif

// USB audio class, read once at boot (internal pull-up): open = UAC2,
// grounded = UAC1. Physical pin 7 on both boards (Pico 2: GND on pin 8).
#define UAC_MODE_PIN 5

// Output format jumpers, read once at boot (internal pull-ups): grounded =
// 1, bit 0 first; the 3-bit code is an i2s_format_t. Pico 2: physical pins
// 9, 10, 11 (GND on 8 and 13). On the RP2350-One all seven jumpers (GP2..GP8)
// are on the side opposite the I2S pins.
#define I2S_FORMAT_PIN0 6
#define I2S_FORMAT_PIN1 7
#define I2S_FORMAT_PIN2 8
// Dual-line filter jumpers, read once at boot (internal pull-ups): grounded
// = 1, bit 0 first; the 3-bit code is an i2s_filter_type_t (NOS when all
// open, GP4 alone = sharp). Pico 2: physical pins 6, 5, 4 (GND on 3 and 8).
#define I2S_FILTER_PIN0 4
#define I2S_FILTER_PIN1 3
#define I2S_FILTER_PIN2 2

// Onboard LED: audio/LOCK indicator only. The RP2350-One has no plain LED;
// its WS2812 shows the source heard, dim: red for USB, green for the S/PDIF
// input, yellow for the input with pre-emphasis. Values set by eye: the
// green die is far brighter than the red one, so green runs at a fraction
// of red's value (yellow needs the smallest step), and the plain green is
// a cold tint, so a step of red warms it. Its colour order is RGB, not the
// usual GRB (sent green first it lights green).
#if USB2SPDIF_BOARD_RP2350_ONE
#define LOCK_LED_PIN       16
#define LOCK_LED_WS2812    1
#define LOCK_LED_ORDER_RGB 1
#define LOCK_LED_R         0x0A
#define LOCK_LED_G         0x00
#define LOCK_LED_B         0x00
#define LOCK_LED_INPUT_R   0x01
#define LOCK_LED_INPUT_G   0x02
#define LOCK_LED_INPUT_B   0x00
#define LOCK_LED_EMPH_R    0x0A
#define LOCK_LED_EMPH_G    0x01
#define LOCK_LED_EMPH_B    0x00
#else
#define LOCK_LED_PIN    PICO_DEFAULT_LED_PIN
#define LOCK_LED_WS2812 0
// External S/PDIF input LEDs (active high, series resistor to GND): LOCK
// on physical pin 29, EMPH (the input plays with pre-emphasis, which the
// I2S output corrects) on pin 27, GND between them on 28.
#define INPUT_LOCK_LED_PIN 22
#define INPUT_EMPH_LED_PIN 21
#endif

#define CONSOLE_UART      uart0
#define CONSOLE_UART_TX   0
#define CONSOLE_UART_RX   1
// High rate so the non-blocking log ring (app/log.c) drains quickly.
#define CONSOLE_BAUD      921600
