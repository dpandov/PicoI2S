// S/PDIF receiver, see spdif_rx.h.
//
// Per input: a decoder state machine (spdif_rx.pio) pushes one word per
// subframe; an endless DMA channel writes them into a ring that wraps in
// hardware, so the CPU only follows the write pointer. The state machine
// clock is set for one nominal rate at a time. Unlocked inputs cycle
// through the rates; once words parse, the frame rate measured against the
// timer picks the right one, and a clean run of LOCK_US within
// LOCK_TOL_PPM makes the lock.

#include "spdif/spdif_rx.h"

#include <string.h>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/structs/pads_bank0.h"
#include "hardware/timer.h"

#include "clock/audio_clock.h"
#include "spdif/spdif_rx_parse.h"
#include "spdif/spdif_tx.h"
#include "spdif_rx.pio.h"

// Words per input ring: 10.7 ms at 96 kHz.
#define RING_WORDS 2048u
#define RING_MASK  (RING_WORDS - 1u)
#define RING_BITS  13u                        // log2 of the ring size in bytes
_Static_assert(4u * RING_WORDS == 1u << RING_BITS, "ring size");
// Worst-case word rate of a decoder set for the wrong rate, for the ring
// lap check while the rate is unknown.
#define MAX_WORDS_PER_S 300000u

// 21 ms at 96 kHz, 46 ms at 44.1 kHz.
#define FIFO_FRAMES 2048u
// Flag changes the FIFO can hold (they happen at block starts).
#define CHANGES     8u

#define RATES (sizeof(spdif_rx_rates) / sizeof(spdif_rx_rates[0]))
// State-machine cycles per cell (spdif_rx.pio).
#define CYCLES_PER_CELL 8u

#define ACQUIRE_GOOD     64u      // consecutive good subframes: start measuring
#define RESYNC_BAD       8u       // consecutive bad subframes: restart the decoder
#define DWELL_US         5000u    // per candidate rate without a run of good subframes
#define CLASSIFY_US      2000u    // first rate estimate
#define CLASSIFY_TOL_PPM 40000u   // the supported rates are 8.8 % or more apart
#define LOCK_US          16000u   // lock measurement window
#define LOCK_TOL_PPM     1200u    // level II (1000 ppm) plus the measurement error
// Rate re-check while locked. A decoder set for 44.1 kHz also decodes
// 48 kHz (and the other way round), so a rate change without a signal gap
// shows up only here.
#define CHECK_US         16000u
#define CHECK_TOL_PPM    1500u

#define NOSIG_US         2000u    // no words: restart the decoder, hold the lock
#define HOLD_US          4000u    // lock held while the decoder resynchronises
#define CLOCK_HOLD_US    50000u   // the same after a clk_sys change
#define STAMP_SPIN_US    100u     // longest wait for a word when time-stamping
// The task does nothing sooner after its last run.
#define TASK_PERIOD_US   1000u
// Good subframes in a row that confirm the hunt's polarity.
#define SYNC_GOOD        16u

typedef enum { ST_HUNT, ST_ACQUIRE, ST_LOCKED } rx_state_t;

typedef struct {
    uint      pin, sm, dma;
    uint32_t *ring;
    uint32_t  widx;             // DMA write index at the last poll
    uint32_t  tail;             // next word to parse
    uint64_t  words;            // words written up to widx
    uint64_t  poll_us;
    spdif_rx_parser_t parser;
    rx_state_t state;
    uint8_t   rate_idx;         // rate the decoder is set for
    bool      locked;
    bool      holding;          // locked, decoder resynchronising
    bool      classified;       // ACQUIRE: rate estimate matched rate_idx
    bool      high;             // the preambles start with a high run
    bool      synced;           // good words since the last decoder restart
    uint64_t  t_state;          // HUNT: dwell start; ACQUIRE: window start
    uint64_t  hold_until;
    uint64_t  t_words;          // last time words arrived in sync
    uint64_t  m_us, m_words;    // measurement window start
    uint64_t  l_us, l_words;    // long window, since lock or resync
    uint32_t  err_mark;         // parser errors at ACQUIRE start
    uint32_t  seen_parity, seen_frame, seen_slips;
    int32_t   ppm;
} rx_input_t;

static uint32_t rings[2][RING_WORDS] __attribute__((aligned(4u * RING_WORDS)));
static rx_input_t inputs[2];
static PIO rx_pio;
static uint rx_offset;
static uint32_t clk_hz;
static uint32_t clk_gen;
static uint32_t task_us;
static spdif_rx_input_t selected;
static spdif_rx_stats_t stats;

// FIFO of frames for the selected input, with the positions where the
// flags change.
typedef struct {
    uint32_t pos;
    uint8_t  flags;
} change_t;

static int32_t  fifo[2u * FIFO_FRAMES];
static uint32_t fifo_wr, fifo_rd;       // frame counts
static change_t changes[CHANGES];
static uint32_t ch_head, ch_tail;
static uint8_t  wr_flags;               // of the last frame written, 0xFF: none
static uint8_t  rd_flags;
static bool     read_since_flush;
static uint32_t generation;

static void fifo_empty(void) {
    fifo_rd = fifo_wr;
    ch_tail = ch_head;
    wr_flags = 0xFFu;
    read_since_flush = false;
}

static void fifo_flush(void) {
    fifo_empty();
    generation++;
}

// An overflow is a discontinuity only for a reader; unread, the FIFO
// just starts over.
static void emit(void *ctx, const int32_t *pcm, uint32_t n, uint32_t flags) {
    (void)ctx;
    if (n > FIFO_FRAMES - (fifo_wr - fifo_rd) ||
        (flags != wr_flags && ch_head - ch_tail == CHANGES)) {
        if (read_since_flush) {
            stats.overflows++;
            fifo_flush();
        } else {
            fifo_empty();
        }
    }
    if (flags != wr_flags) {
        changes[ch_head % CHANGES] = (change_t){fifo_wr, (uint8_t)flags};
        ch_head++;
        wr_flags = (uint8_t)flags;
    }
    uint32_t i = fifo_wr % FIFO_FRAMES;
    uint32_t first = FIFO_FRAMES - i < n ? FIFO_FRAMES - i : n;
    memcpy(&fifo[2u * i], pcm, first * 8u);
    memcpy(fifo, &pcm[2u * first], (n - first) * 8u);
    fifo_wr += n;
}

//--------------------------------------------------------------------+
// Decoder hardware
//--------------------------------------------------------------------+

static uint32_t dma_index(const rx_input_t *in) {
    return ((dma_hw->ch[in->dma].write_addr - (uintptr_t)in->ring) / 4u) & RING_MASK;
}

static void set_divider(rx_input_t *in) {
    // clk_sys / (CYCLES_PER_CELL x 128 x rate) in 1/256 steps.
    uint32_t den = CYCLES_PER_CELL * 128u / 256u * spdif_rx_rates[in->rate_idx];
    uint32_t d256 = (clk_hz + den / 2u) / den;
    pio_sm_set_clkdiv_int_frac8(rx_pio, in->sm, d256 >> 8, d256 & 0xFFu);
}

static void poll(rx_input_t *in, uint64_t now) {
    uint32_t w = dma_index(in);
    in->words += (w - in->widx) & RING_MASK;
    in->widx = w;
    in->poll_us = now;
}

// Back to the hunt for a preamble, with the words so far dropped. The hunt
// keeps the preamble level of the last good words; after a restart that
// found none it tries the other level.
static void restart_decoder(rx_input_t *in, uint64_t now) {
    PIO p = rx_pio;
    if (!in->synced)
        in->high = !in->high;
    in->synced = false;
    // Atomic: the self-test may share the block from core 1.
    hw_clear_bits(&p->ctrl, 1u << in->sm);
    pio_sm_clear_fifos(p, in->sm);
    pio_sm_restart(p, in->sm);
    pio_sm_clkdiv_restart(p, in->sm);
    pio_sm_exec(p, in->sm, pio_encode_mov(pio_isr, pio_null));
    pio_sm_exec(p, in->sm, pio_encode_jmp(rx_offset + (in->high ? spdif_rx_offset_hunt_high
                                                                 : spdif_rx_offset_hunt_low)));
    poll(in, now);
    in->tail = in->widx;
    spdif_rx_parser_resync(&in->parser);
    hw_set_bits(&p->ctrl, 1u << in->sm);
}

// The timer at the arrival of a word, and the words received by then.
// Spins until the next word (one subframe); false without one, or when
// interrupts kept it from a clean reading (the caller tries again later).
// Each index read lies between two timer reads, so a change is known to
// within the time from before the previous read to after this one.
static bool stamp(rx_input_t *in, uint64_t *us, uint64_t *words) {
    uint64_t t_start = time_us_64();
    uint64_t t_before = t_start;
    uint32_t w_prev = dma_index(in);
    for (;;) {
        uint64_t t0 = time_us_64();
        uint32_t w = dma_index(in);
        uint64_t t1 = time_us_64();
        if (w != w_prev && t1 - t_before <= 1u) {
            in->words += (w - in->widx) & RING_MASK;
            in->widx = w;
            *us = t1;
            *words = in->words;
            return true;
        }
        if (t1 - t_start > STAMP_SPIN_US)
            return false;
        w_prev = w;
        t_before = t0;
    }
}

static void input_init(rx_input_t *in, uint pin, uint sm, uint32_t *ring) {
    in->pin = pin;
    in->sm = sm;
    in->ring = ring;
    in->rate_idx = 2;           // 48 kHz first
    spdif_rx_parser_init(&in->parser);

    // Input only: FUNCSEL stays as it is (normally NULL, so never driven).
    gpio_set_pulls(pin, false, true);
    hw_set_bits(&pads_bank0_hw->io[pin], PADS_BANK0_GPIO0_IE_BITS);
    hw_clear_bits(&pads_bank0_hw->io[pin], PADS_BANK0_GPIO0_ISO_BITS);

    pio_sm_config c = spdif_rx_program_get_default_config(rx_offset);
    sm_config_set_in_pins(&c, pin);
    sm_config_set_jmp_pin(&c, pin);
    sm_config_set_in_shift(&c, true, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_sm_init(rx_pio, sm, rx_offset + spdif_rx_offset_hunt_high, &c);
    set_divider(in);

    in->dma = (uint)dma_claim_unused_channel(true);
    dma_channel_config d = dma_channel_get_default_config(in->dma);
    channel_config_set_transfer_data_size(&d, DMA_SIZE_32);
    channel_config_set_read_increment(&d, false);
    channel_config_set_write_increment(&d, true);
    channel_config_set_ring(&d, true, RING_BITS);
    channel_config_set_dreq(&d, pio_get_dreq(rx_pio, sm, false));
    dma_channel_configure(in->dma, &d, ring, &rx_pio->rxf[sm], dma_encode_endless_transfer_count(), true);

    uint64_t now = time_us_64();
    restart_decoder(in, now);
    in->state = ST_HUNT;
    in->t_state = now;
}

// Two state machines and room for the program on a block other than the
// transmitter's. pio0 first: the SDK's free-SM search (the transmitter)
// starts at the last block and the self-test's at pio2, so the three
// normally get a block each; the transmitter's trim pacers take the other
// two state machines and 4 instructions of pio0 (spdif_tx.c).
static bool claim_block(uint sms[2]) {
    static const PIO order[] = {
        pio0,
        pio1,
#if NUM_PIOS > 2
        pio2,
#endif
    };
    for (unsigned i = 0; i < sizeof(order) / sizeof(order[0]); i++) {
        PIO p = order[i];
        if (p == spdif_tx_pio() || !pio_can_add_program(p, &spdif_rx_program))
            continue;
        int a = pio_claim_unused_sm(p, false);
        if (a < 0)
            continue;
        int b = pio_claim_unused_sm(p, false);
        if (b < 0) {
            pio_sm_unclaim(p, (uint)a);
            continue;
        }
        rx_pio = p;
        rx_offset = (uint)pio_add_program(p, &spdif_rx_program);
        sms[0] = (uint)a;
        sms[1] = (uint)b;
        return true;
    }
    return false;
}

void spdif_rx_init(unsigned coax_pin, unsigned toslink_pin) {
    uint sms[2];
    if (coax_pin >= 32u || toslink_pin >= 32u || !claim_block(sms))
        panic("spdif_rx: no PIO block");
    clk_hz = clock_get_hz(clk_sys);
    clk_gen = audio_clock_generation();
    wr_flags = 0xFFu;
    input_init(&inputs[0], coax_pin, sms[0], rings[0]);
    input_init(&inputs[1], toslink_pin, sms[1], rings[1]);
}

//--------------------------------------------------------------------+
// Lock
//--------------------------------------------------------------------+

static bool is_selected(const rx_input_t *in) {
    return selected != SPDIF_RX_NONE && &inputs[selected - 1] == in;
}

static void enter_hunt(rx_input_t *in, uint64_t now) {
    in->state = ST_HUNT;
    in->t_state = now;
}

static void next_rate(rx_input_t *in, uint64_t now) {
    in->rate_idx = (uint8_t)((in->rate_idx + 1u) % RATES);
    set_divider(in);
    restart_decoder(in, now);
    enter_hunt(in, now);
}

static void set_rate(rx_input_t *in, uint32_t rate, uint64_t now) {
    for (uint8_t i = 0; i < RATES; i++)
        if (spdif_rx_rates[i] == rate)
            in->rate_idx = i;
    set_divider(in);
    restart_decoder(in, now);
    enter_hunt(in, now);
}

static void unlock(rx_input_t *in, uint64_t now) {
    in->locked = false;
    in->holding = false;
    in->ppm = 0;
    stats.unlocks++;
    enter_hunt(in, now);
}

// Sync lost while locked: the lock is held for hold_us.
static void lose_sync(rx_input_t *in, uint64_t now, uint32_t hold_us) {
    restart_decoder(in, now);
    if (!in->holding || now + hold_us > in->hold_until)
        in->hold_until = now + hold_us;
    in->holding = true;
    if (is_selected(in))
        fifo_flush();
}

static uint32_t parser_errors(const spdif_rx_parser_t *p) {
    return p->err_parity + p->err_frame + p->slips;
}

static void step_hunt(rx_input_t *in, uint64_t now) {
    spdif_rx_parser_t *p = &in->parser;
    if (p->good_run >= ACQUIRE_GOOD) {
        if (stamp(in, &in->m_us, &in->m_words)) {
            in->state = ST_ACQUIRE;
            in->classified = false;
            in->err_mark = parser_errors(p);
            in->t_state = now;
            in->t_words = now;
            return;
        }
    } else if (p->bad_run >= RESYNC_BAD) {
        restart_decoder(in, now);
    }
    if (now - in->t_state > DWELL_US)
        next_rate(in, now);
}

static void step_acquire(rx_input_t *in, uint64_t now) {
    spdif_rx_parser_t *p = &in->parser;
    if (parser_errors(p) != in->err_mark || p->bad_run > 0 || now - in->t_words > NOSIG_US) {
        if (p->bad_run >= RESYNC_BAD)
            restart_decoder(in, now);
        in->state = ST_HUNT;    // the dwell continues
        return;
    }
    uint64_t dt = now - in->m_us;
    if (!in->classified && dt >= CLASSIFY_US) {
        uint64_t us, words;
        if (!stamp(in, &us, &words))
            return;
        uint32_t rate = spdif_rx_rate_classify(words - in->m_words, us - in->m_us, CLASSIFY_TOL_PPM);
        if (rate == 0) {
            next_rate(in, now);
        } else if (rate != spdif_rx_rates[in->rate_idx]) {
            set_rate(in, rate, now);
        } else {
            in->classified = true;
        }
        return;
    }
    if (in->classified && dt >= LOCK_US) {
        uint64_t us, words;
        if (!stamp(in, &us, &words))
            return;
        int32_t ppm = spdif_rx_rate_ppm(words - in->m_words, us - in->m_us, spdif_rx_rates[in->rate_idx]);
        if (ppm > (int32_t)LOCK_TOL_PPM || ppm < -(int32_t)LOCK_TOL_PPM) {
            enter_hunt(in, now);
            return;
        }
        in->state = ST_LOCKED;
        in->locked = true;
        in->holding = false;
        in->ppm = ppm;
        in->l_us = in->m_us;
        in->l_words = in->m_words;
        in->m_us = us;
        in->m_words = words;
        in->t_words = now;
        p->cs_complete = false;
        stats.locks++;
    }
}

static void step_locked(rx_input_t *in, uint64_t now) {
    spdif_rx_parser_t *p = &in->parser;
    if (in->holding) {
        if (p->good_run >= ACQUIRE_GOOD) {
            uint64_t us, words;
            if (stamp(in, &us, &words)) {
                in->holding = false;
                in->t_words = now;
                in->l_us = in->m_us = us;
                in->l_words = in->m_words = words;
            }
        } else if ((int64_t)(now - in->hold_until) > 0) {
            unlock(in, now);
        } else if (p->bad_run >= RESYNC_BAD) {
            restart_decoder(in, now);
        }
        return;
    }
    if (p->bad_run >= RESYNC_BAD || now - in->t_words > NOSIG_US) {
        lose_sync(in, now, HOLD_US);
        return;
    }
    if (now - in->m_us < CHECK_US)
        return;
    uint64_t us, words;
    if (!stamp(in, &us, &words))
        return;
    uint32_t rate = spdif_rx_rates[in->rate_idx];
    int32_t ppm = spdif_rx_rate_ppm(words - in->m_words, us - in->m_us, rate);
    if (ppm > (int32_t)CHECK_TOL_PPM || ppm < -(int32_t)CHECK_TOL_PPM) {
        unlock(in, now);        // rate change: measured again from the hunt
        restart_decoder(in, now);
        return;
    }
    in->m_us = us;
    in->m_words = words;
    in->ppm = spdif_rx_rate_ppm(words - in->l_words, us - in->l_us, rate);
}

static void input_task(rx_input_t *in, uint64_t now) {
    spdif_rx_parser_t *p = &in->parser;
    uint64_t dt = now - in->poll_us;
    uint32_t prev = in->widx;
    poll(in, now);

    // The DMA may have lapped the ring since the last poll.
    uint32_t per_s = in->state == ST_HUNT ? MAX_WORDS_PER_S : 2u * spdif_rx_rates[in->rate_idx] * 11u / 10u;
    if (dt * per_s >= (uint64_t)(RING_WORDS - 64u) * 1000000u) {
        if (is_selected(in) && read_since_flush)
            stats.overflows++;
        if (in->state == ST_LOCKED) {
            // The rate windows missed the lapped words: resync under the
            // hold, which measures them afresh.
            lose_sync(in, now, HOLD_US);
        } else {
            in->tail = in->widx;
            spdif_rx_parser_resync(p);
            if (is_selected(in))
                fifo_flush();
        }
    }

    uint32_t n = (in->widx - in->tail) & RING_MASK;
    if (n) {
        uint32_t first = RING_WORDS - in->tail;
        if (first > n)
            first = n;
        spdif_rx_parse(p, &in->ring[in->tail], first);
        if (n > first)
            spdif_rx_parse(p, in->ring, n - first);
        in->tail = in->widx;
    }
    if (in->widx != prev && p->bad_run < RESYNC_BAD)
        in->t_words = now;
    // Bit 3 of a good word: the level of preamble cell 0. The last words
    // may still wait in the parser, unchecked.
    if (p->good_run >= SYNC_GOOD) {
        in->synced = true;
        in->high = (in->ring[(in->widx - 1u - p->n_carry) & RING_MASK] >> 3) & 1u;
    }

    bool sel = is_selected(in);

    if (sel) {
        stats.err_parity += p->err_parity - in->seen_parity;
        stats.err_frame += p->err_frame - in->seen_frame;
        // A word lost or added inside a block: the stream is discontinuous.
        if (p->slips != in->seen_slips)
            fifo_flush();
    }
    in->seen_parity = p->err_parity;
    in->seen_frame = p->err_frame;
    in->seen_slips = p->slips;

    switch (in->state) {
    case ST_HUNT:
        step_hunt(in, now);
        break;
    case ST_ACQUIRE:
        step_acquire(in, now);
        break;
    case ST_LOCKED:
        step_locked(in, now);
        break;
    }
}

static void follow_clock(uint64_t now) {
    uint32_t gen = audio_clock_generation();
    uint32_t hz = clock_get_hz(clk_sys);
    if (gen == clk_gen && hz == clk_hz)
        return;
    clk_gen = gen;
    clk_hz = hz;
    for (unsigned i = 0; i < 2; i++) {
        rx_input_t *in = &inputs[i];
        set_divider(in);
        if (in->locked) {
            lose_sync(in, now, CLOCK_HOLD_US);
        } else {
            restart_decoder(in, now);
            if (in->state == ST_ACQUIRE)
                in->state = ST_HUNT;
        }
    }
}

static void select_input(void) {
    spdif_rx_input_t want = inputs[0].locked ? SPDIF_RX_COAX
                          : inputs[1].locked ? SPDIF_RX_TOSLINK
                          : SPDIF_RX_NONE;
    if (want == selected)
        return;
    if (selected != SPDIF_RX_NONE)
        spdif_rx_parser_set_output(&inputs[selected - 1].parser, NULL, NULL);
    selected = want;
    if (selected != SPDIF_RX_NONE)
        spdif_rx_parser_set_output(&inputs[selected - 1].parser, emit, NULL);
    fifo_flush();
}

void spdif_rx_task(void) {
    uint32_t t = timer_hw->timerawl;
    if (t - task_us < TASK_PERIOD_US || (audio_clock_generation() & 1u))
        return;
    task_us = t;
    uint64_t now = time_us_64();

    follow_clock(now);
    input_task(&inputs[0], now);
    input_task(&inputs[1], now);
    select_input();
}

//--------------------------------------------------------------------+
// API
//--------------------------------------------------------------------+

void spdif_rx_get_status(spdif_rx_status_t *s) {
    memset(s, 0, sizeof(*s));
    s->input = selected;
    s->locked_coax = inputs[0].locked;
    s->locked_toslink = inputs[1].locked;
    s->word_bits = 24;
    if (selected == SPDIF_RX_NONE)
        return;
    const rx_input_t *in = &inputs[selected - 1];
    const spdif_rx_parser_t *p = &in->parser;
    s->sample_rate = spdif_rx_rates[in->rate_idx];
    s->rate_ppm = in->ppm;
    s->cs_valid = p->cs_complete;
    s->pcm = spdif_rx_parser_pcm(p);
    s->emphasis = spdif_rx_parser_emphasis(p);
    if (p->cs_complete)
        s->word_bits = spdif_rx_cs_word_bits(p->cs_last);
}

void spdif_rx_get_stats(spdif_rx_stats_t *s) {
    *s = stats;
}

uint32_t spdif_rx_generation(void) {
    return generation;
}

void spdif_rx_flush(void) {
    fifo_empty();
}

uint32_t spdif_rx_level(void) {
    return fifo_wr - fifo_rd;
}

uint32_t spdif_rx_depth(void) {
    return FIFO_FRAMES;
}

uint32_t spdif_rx_read(int32_t *pcm, uint32_t max_frames, uint32_t *flags) {
    if (ch_tail != ch_head && changes[ch_tail % CHANGES].pos == fifo_rd) {
        rd_flags = changes[ch_tail % CHANGES].flags;
        ch_tail++;
    }
    uint32_t n = fifo_wr - fifo_rd;
    if (n > max_frames)
        n = max_frames;
    if (ch_tail != ch_head && changes[ch_tail % CHANGES].pos - fifo_rd < n)
        n = changes[ch_tail % CHANGES].pos - fifo_rd;
    uint32_t i = fifo_rd % FIFO_FRAMES;
    uint32_t first = FIFO_FRAMES - i < n ? FIFO_FRAMES - i : n;
    memcpy(pcm, &fifo[2u * i], first * 8u);
    memcpy(&pcm[2u * first], fifo, (n - first) * 8u);
    fifo_rd += n;
    if (n)
        read_since_flush = true;
    *flags = rd_flags;
    return n;
}
