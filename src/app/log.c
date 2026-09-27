#include "app/log.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "board_config.h"

// 22 ms of output at 921600 baud.
#define RING_BYTES  2048u
#define RING_MASK   (RING_BYTES - 1u)

_Static_assert((RING_BYTES & RING_MASK) == 0, "ring size must be a power of two");

static char ring[RING_BYTES];
static uint32_t head;               // free-running byte counts
static uint32_t tail;
static bool cr_sent;                // '\r' of a "\n" already in the UART FIFO
static uint32_t dropped;
static uint32_t dropped_reported;

static uint32_t room(void) {
    return RING_BYTES - (head - tail);
}

static bool put(const char *s, uint32_t n) {
    if (n > room()) {
        return false;
    }
    for (uint32_t i = 0; i < n; i++) {
        ring[(head + i) & RING_MASK] = s[i];
    }
    head += n;
    return true;
}

void log_init(void) {
    uart_init(CONSOLE_UART, CONSOLE_BAUD);
    gpio_set_function(CONSOLE_UART_TX, UART_FUNCSEL_NUM(CONSOLE_UART, CONSOLE_UART_TX));
    gpio_set_function(CONSOLE_UART_RX, UART_FUNCSEL_NUM(CONSOLE_UART, CONSOLE_UART_RX));
}

void log_printf(const char *fmt, ...) {
    char line[LOG_LINE_MAX];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n <= 0) {
        return;
    }
    if ((unsigned)n >= sizeof(line)) {
        n = (int)sizeof(line) - 1;
        line[n - 1] = '\n';
    }
    if (!put(line, (uint32_t)n)) {
        dropped++;
    }
}

void log_task(void) {
    if (dropped != dropped_reported && room() >= 32u) {
        char line[32];
        uint32_t d = dropped;
        int n = snprintf(line, sizeof(line), "log: %lu dropped\n",
                         (unsigned long)(d - dropped_reported));
        if (n > 0 && put(line, (uint32_t)n)) {
            dropped_reported = d;
        }
    }
    while (tail != head && uart_is_writable(CONSOLE_UART)) {
        char c = ring[tail & RING_MASK];
        if (c == '\n' && !cr_sent) {
            uart_get_hw(CONSOLE_UART)->dr = '\r';
            cr_sent = true;
            continue;
        }
        uart_get_hw(CONSOLE_UART)->dr = (uint8_t)c;
        cr_sent = false;
        tail++;
    }
}

uint32_t log_dropped(void) {
    return dropped;
}
