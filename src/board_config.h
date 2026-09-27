// Board pin assignments. Wiring is described in README.md.
#pragma once

// S/PDIF data to the Toslink transmitter input.
#define SPDIF_PIN 28

// USB audio class, read once at boot (internal pull-up): open = UAC2,
// grounded = UAC1. Physical pin 7, next to GND on pin 8.
#define UAC_MODE_PIN 5

// Onboard LED: audio/LOCK indicator only.
#define LOCK_LED_PIN PICO_DEFAULT_LED_PIN

#define CONSOLE_UART      uart0
#define CONSOLE_UART_TX   0
#define CONSOLE_UART_RX   1
// High rate so the non-blocking log ring (app/log.c) drains quickly.
#define CONSOLE_BAUD      921600
