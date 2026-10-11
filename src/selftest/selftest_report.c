// Self-test window classification and report string, see selftest_report.h.

#include "selftest/selftest_report.h"

#include <stdarg.h>
#include <stdio.h>

#define CUMULATIVE_MAX 99999u

static void bump(uint32_t *counter) {
    if (*counter < CUMULATIVE_MAX) {
        (*counter)++;
    }
}

static selftest_state_t classify(const selftest_decode_t *r, bool running, bool pattern) {
    if (!running) {
        return r->status == SELFTEST_DEC_NOSIG ? SELFTEST_IDLE : SELFTEST_CARRIER;
    }
    if (r->status == SELFTEST_DEC_NOSIG) {
        return SELFTEST_NOCARR;
    }
    if (r->status == SELFTEST_DEC_NOSYNC) {
        return SELFTEST_NOSYNC;
    }
    if (selftest_decode_errors(r) != 0 || r->frames < SELFTEST_MIN_FRAMES || r->blocks < 2u) {
        return SELFTEST_ERR;
    }
    // Without the pattern, isolated zero frames are ordinary audio.
    bool silent = pattern ? r->silent != 0 : r->silent == r->frames;
    return silent ? SELFTEST_SIL : SELFTEST_OK;
}

void selftest_report_window(selftest_report_t *rep, const selftest_decode_t *r,
                            uint32_t tx_rate, uint8_t tx_bits, bool pattern,
                            uint32_t clk_hz, uint32_t cap_div) {
    rep->seq++;
    rep->tx_rate = tx_rate;
    rep->tx_bits = tx_rate ? tx_bits : 0u;
    rep->pattern = pattern;
    rep->level = (uint8_t)r->level;
    rep->cs_flag = r->err_cs_ab ? 'x' : r->cs_match < 0 ? '-' : r->cs_match ? '1' : '0';
    rep->cs3 = r->cs[3];
    rep->cs4 = r->cs[4];
    rep->frames = r->frames;
    rep->silent = r->silent;
    rep->err_cell = r->err_cell;
    rep->err_parity = r->err_parity;
    rep->err_order = r->err_order;
    rep->err_vu = r->err_vu;
    rep->err_sample = r->err_sample;
    rep->viol_frame = r->viol_frame;
    rep->viol_prev = r->viol_prev;
    rep->viol_l = r->viol_l;
    rep->viol_r = r->viol_r;
    rep->runs = r->runs;
    rep->ovs_x100 = (uint32_t)(((uint64_t)r->ovs_q8 * 100u + 128u) / 256u);

    uint64_t den = r->span_samples * (uint64_t)cap_div * 128u;
    rep->fs_hz = den ? (uint32_t)(((uint64_t)clk_hz * r->span_cells + den / 2u) / den) : 0u;

    rep->state = (uint8_t)classify(r, tx_rate != 0, pattern);
    if (selftest_state_is_bad((selftest_state_t)rep->state)) {
        bump(&rep->bad);
    }
    if (r->gaps) {
        bump(&rep->gaps);
    }
}

void selftest_report_timeout(selftest_report_t *rep, uint32_t tx_rate, uint8_t tx_bits) {
    rep->seq++;
    rep->tx_rate = tx_rate;
    rep->tx_bits = tx_rate ? tx_bits : 0u;
    rep->state = SELFTEST_CAPTMO;
    bump(&rep->bad);
}

void selftest_report_skipped(selftest_report_t *rep) {
    bump(&rep->skipped);
}

bool selftest_state_is_bad(selftest_state_t s) {
    return s == SELFTEST_ERR || s == SELFTEST_NOSYNC || s == SELFTEST_NOCARR ||
           s == SELFTEST_CARRIER || s == SELFTEST_CAPTMO || s == SELFTEST_OFF ||
           s == SELFTEST_NOCLK || s == SELFTEST_PINHI || s == SELFTEST_NOTIM;
}

static const char *const state_names[] = {
    [SELFTEST_WAIT] = "wait",     [SELFTEST_OK] = "ok",           [SELFTEST_SIL] = "sil",
    [SELFTEST_ERR] = "ERR",       [SELFTEST_NOSYNC] = "NOSYNC",   [SELFTEST_NOCARR] = "NOCARR",
    [SELFTEST_IDLE] = "idle",     [SELFTEST_CARRIER] = "CARRIER", [SELFTEST_CAPTMO] = "CAPTMO",
    [SELFTEST_NOPIO] = "NOPIO",   [SELFTEST_OFF] = "OFF",         [SELFTEST_NOCLK] = "NOCLK",
    [SELFTEST_PINHI] = "PINHI",   [SELFTEST_NOTIM] = "NOTIM",
};

static const char *state_name(selftest_state_t state) {
    return state <= SELFTEST_NOTIM ? state_names[state] : state_names[SELFTEST_WAIT];
}

static unsigned long cap(uint32_t v, uint32_t max) {
    return (unsigned long)(v > max ? max : v);
}

static void append(char *buf, size_t size, size_t *len, const char *fmt, ...)
    __attribute__((format(printf, 4, 5)));

// Appends at *len, never past size - 1.
static void append(char *buf, size_t size, size_t *len, const char *fmt, ...) {
    if (*len + 1u >= size) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf + *len, size - *len, fmt, ap);
    va_end(ap);
    if (n > 0) {
        *len += (size_t)n < size - *len ? (size_t)n : size - *len - 1u;
    }
}

size_t selftest_report_format(char *buf, size_t size, const selftest_report_t *p) {
    if (size == 0) {
        return 0;
    }
    if (size > SELFTEST_STRING_MAX + 1u) {
        size = SELFTEST_STRING_MAX + 1u;
    }
    buf[0] = '\0';
    size_t len = 0;
    selftest_state_t st = p->state <= SELFTEST_NOPIO ? (selftest_state_t)p->state : SELFTEST_WAIT;

    append(buf, size, &len, "#%lu %s", (unsigned long)(p->seq % 1000000u), state_name(st));
    if (st == SELFTEST_NOPIO) {
        return len;
    }
    if (st != SELFTEST_WAIT) {
        if (p->tx_rate) {
            append(buf, size, &len, " tx%lu/%u", (unsigned long)p->tx_rate, (unsigned)p->tx_bits);
        } else {
            append(buf, size, &len, " tx0");
        }
    }

    switch (st) {
    case SELFTEST_OK:
    case SELFTEST_SIL:
    case SELFTEST_ERR:
        append(buf, size, &len, " f%lu z%lu e%lu p%lu o%lu v%lu", cap(p->frames, 9999u),
               cap(p->silent, 9999u), cap(p->err_cell, 99u), cap(p->err_parity, 99u),
               cap(p->err_order, 99u), cap(p->err_vu, 99u));
        if (p->pattern) {
            append(buf, size, &len, " s%lu", cap(p->err_sample, 99u));
        } else {
            append(buf, size, &len, " s-");
        }
        append(buf, size, &len, " c%c%02X%02X fs%lu", p->cs_flag, (unsigned)p->cs3,
               (unsigned)p->cs4, cap(p->fs_hz, 999999u));
        break;
    case SELFTEST_IDLE:
    case SELFTEST_NOCARR:
        append(buf, size, &len, " lvl%u", (unsigned)p->level);
        break;
    case SELFTEST_CARRIER:
        append(buf, size, &len, " r%lu", cap(p->runs, 999999u));
        break;
    case SELFTEST_NOSYNC:
        append(buf, size, &len, " r%lu q%lu", cap(p->runs, 999999u), cap(p->ovs_x100, 99999u));
        break;
    default:
        break;
    }

    append(buf, size, &len, " B%lu G%lu K%lu", cap(p->bad, CUMULATIVE_MAX),
           cap(p->gaps, CUMULATIVE_MAX), cap(p->skipped, CUMULATIVE_MAX));

    if ((st == SELFTEST_OK || st == SELFTEST_SIL || st == SELFTEST_ERR) && p->pattern &&
        p->err_sample) {
        append(buf, size, &len, " @%lu:%06lX>%06lX/%06lX", cap(p->viol_frame, 9999u),
               (unsigned long)(p->viol_prev & 0xFFFFFFu), (unsigned long)(p->viol_l & 0xFFFFFFu),
               (unsigned long)(p->viol_r & 0xFFFFFFu));
    }
    return len;
}

//--------------------------------------------------------------------+
// I2S report
//--------------------------------------------------------------------+

void selftest_i2s_report_output(selftest_i2s_report_t *rep, unsigned filter, unsigned osr,
                                uint32_t late) {
    rep->filter = (uint8_t)filter;
    rep->osr = (uint8_t)osr;
    rep->late = late;
}

void selftest_i2s_report_window(selftest_i2s_report_t *rep, const selftest_i2s_decode_t *r,
                                uint32_t tx_rate, uint8_t tx_bits, selftest_check_t check,
                                uint32_t window_frames) {
    rep->seq++;
    rep->tx_rate = tx_rate;
    rep->tx_bits = tx_bits;
    rep->check = (uint8_t)check;
    rep->level = 0;
    rep->frames = r->frames;
    rep->silent = r->silent;
    rep->err_frame = r->err_frame;
    rep->err_sample = r->err_sample;
    rep->viol_frame = r->viol_frame;
    rep->viol_prev = r->viol_prev;
    rep->viol_l = r->viol_l;
    rep->viol_r = r->viol_r;

    selftest_state_t st;
    if (r->status == SELFTEST_I2S_NOSYNC) {
        st = SELFTEST_NOSYNC;
    } else if (selftest_i2s_decode_errors(r) != 0 ||
               r->frames < selftest_i2s_min_frames(window_frames)) {
        st = SELFTEST_ERR;
    } else {
        // Only the pattern never produces a zero frame: elsewhere isolated
        // zero frames are ordinary audio.
        bool silent = check == SELFTEST_CHECK_PATTERN ? r->silent != 0 : r->silent == r->frames;
        st = silent ? SELFTEST_SIL : SELFTEST_OK;
    }
    rep->state = (uint8_t)st;
    if (selftest_state_is_bad(st)) {
        bump(&rep->bad);
    }
    if (r->gaps) {
        bump(&rep->gaps);
    }
}

void selftest_i2s_report_state(selftest_i2s_report_t *rep, selftest_state_t state,
                               uint32_t tx_rate, uint8_t tx_bits, uint8_t level) {
    rep->seq++;
    rep->state = (uint8_t)state;
    rep->tx_rate = tx_rate;
    rep->tx_bits = tx_rate ? tx_bits : 0u;
    rep->level = level;
    if (selftest_state_is_bad(state)) {
        bump(&rep->bad);
    }
}

void selftest_i2s_report_skipped(selftest_i2s_report_t *rep) {
    bump(&rep->skipped);
}

// "-", "nos", "sinc", or an oversampling filter's name with the running
// output's oversampling ("sharp8"; the bare name while stopped).
static void filter_token(char *tok, size_t size, const selftest_i2s_report_t *p) {
    if (p->filter >= I2S_FILTER_COUNT) {
        snprintf(tok, size, "-");
        return;
    }
    i2s_filter_type_t f = (i2s_filter_type_t)p->filter;
    if (p->osr > 1u) {
        snprintf(tok, size, "%s%u", i2s_filter_name(f), (unsigned)p->osr);
    } else {
        snprintf(tok, size, "%s", i2s_filter_name(f));
    }
}

size_t selftest_i2s_report_format(char *buf, size_t size, const selftest_i2s_report_t *p) {
    if (size == 0) {
        return 0;
    }
    if (size > SELFTEST_STRING_MAX + 1u) {
        size = SELFTEST_STRING_MAX + 1u;
    }
    buf[0] = '\0';
    size_t len = 0;
    selftest_state_t st = p->state <= SELFTEST_NOTIM ? (selftest_state_t)p->state : SELFTEST_WAIT;

    char tok[16];
    filter_token(tok, sizeof(tok), p);
    append(buf, size, &len, "#%lu %s %s %s", (unsigned long)(p->seq % 1000000u), state_name(st),
           p->fmt_name ? p->fmt_name : "?", tok);
    if (st == SELFTEST_NOPIO) {
        return len;
    }
    if (st != SELFTEST_WAIT) {
        if (p->tx_rate) {
            append(buf, size, &len, " tx%lu/%u", (unsigned long)p->tx_rate, (unsigned)p->tx_bits);
        } else {
            append(buf, size, &len, " tx0");
        }
    }

    switch (st) {
    case SELFTEST_OK:
    case SELFTEST_SIL:
    case SELFTEST_ERR:
        append(buf, size, &len, " f%lu z%lu", cap(p->frames, 9999u), cap(p->silent, 9999u));
        if (p->check != SELFTEST_CHECK_NONE) {
            append(buf, size, &len, " s%lu", cap(p->err_sample, 99u));
        } else {
            append(buf, size, &len, " s-");
        }
        append(buf, size, &len, " e%lu", cap(p->err_frame, 99u));
        break;
    case SELFTEST_IDLE:
    case SELFTEST_PINHI:
        append(buf, size, &len, " lvl%X", (unsigned)(p->level & 0xFu));
        break;
    default:
        break;
    }

    append(buf, size, &len, " L%lu B%lu G%lu K%lu", cap(p->late, CUMULATIVE_MAX),
           cap(p->bad, CUMULATIVE_MAX), cap(p->gaps, CUMULATIVE_MAX),
           cap(p->skipped, CUMULATIVE_MAX));

    if ((st == SELFTEST_OK || st == SELFTEST_SIL || st == SELFTEST_ERR) &&
        p->check != SELFTEST_CHECK_NONE && p->err_sample) {
        append(buf, size, &len, " @%lu:%08lX>%08lX/%08lX", cap(p->viol_frame, 9999u),
               (unsigned long)p->viol_prev, (unsigned long)p->viol_l, (unsigned long)p->viol_r);
    }
    return len;
}
