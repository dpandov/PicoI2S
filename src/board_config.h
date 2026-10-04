// Board pin assignments. Wiring is described in README.md. The board is a
// build option (USB2SPDIF_BOARD in CMakeLists.txt): the Waveshare
// RP2350-One by default, or the Raspberry Pi Pico 2. Same chip, flash and
// crystal, different pins.
#pragma once

#if !USB2SPDIF_BOARD_RP2350_ONE && !USB2SPDIF_BOARD_PICO2
#error "USB2SPDIF_BOARD not set (CMakeLists.txt)"
#endif

// S/PDIF data to the Toslink transmitter input.
#if USB2SPDIF_BOARD_RP2350_ONE
// At the end of the GP0..GP13 row, on the short side of the board.
#define SPDIF_PIN 12
#else
// Physical pin 34, next to GND (33) and 3V3 (36).
#define SPDIF_PIN 28
#endif

// USB audio class, read once at boot (internal pull-up): open = UAC2,
// grounded = UAC1. Physical pin 7 on both boards (Pico 2: GND on pin 8).
#define UAC_MODE_PIN 5

// Onboard LED: audio/LOCK indicator only. The RP2350-One has no plain LED;
// its WS2812 is driven dim red. Its colour order is RGB, not the usual GRB
// (sent green first it lights green).
#if USB2SPDIF_BOARD_RP2350_ONE
#define LOCK_LED_PIN       16
#define LOCK_LED_WS2812    1
#define LOCK_LED_ORDER_RGB 1
#define LOCK_LED_R         0x0A
#define LOCK_LED_G         0x00
#define LOCK_LED_B         0x00
#else
#define LOCK_LED_PIN    PICO_DEFAULT_LED_PIN
#define LOCK_LED_WS2812 0
#endif

#define CONSOLE_UART      uart0
#define CONSOLE_UART_TX   0
#define CONSOLE_UART_RX   1
// High rate so the non-blocking log ring (app/log.c) drains quickly.
#define CONSOLE_BAUD      921600
