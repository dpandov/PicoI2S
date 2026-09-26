// Board pin assignments. Wiring is described in README.md.
#pragma once

// S/PDIF data to the Toslink transmitter input.
#define SPDIF_PIN 28

// Onboard LED: audio/LOCK indicator only.
#define LOCK_LED_PIN PICO_DEFAULT_LED_PIN

#define CONSOLE_UART      uart0
#define CONSOLE_UART_TX   0
#define CONSOLE_UART_RX   1
// High rate because stdio blocks while the UART FIFO is full; at 115200 one
// log line would stall the USB task for several milliseconds.
#define CONSOLE_BAUD      921600
