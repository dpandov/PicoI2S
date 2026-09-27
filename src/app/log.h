// Non-blocking console log on the UART.
//
// Messages go into a RAM ring and log_task() moves them to the UART FIFO
// while it has room, so logging never stalls the main loop. A message that
// does not fit is dropped whole and counted. Main loop only.
#pragma once

#include <stdint.h>

// Sets up the console UART (board_config.h).
void log_init(void);

// printf-style; longer messages are truncated to LOG_LINE_MAX - 1 chars.
#define LOG_LINE_MAX 128
void log_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Call every main-loop iteration.
void log_task(void);

// Messages dropped because the ring was full.
uint32_t log_dropped(void);
