// S/PDIF line decoder, see selftest_decode.h.
//
//  1. Runs: the capture is split into runs of equal samples (clz per word).
//     The first and last runs are partial and dropped.
//  2. Cell period: histogram of run lengths. The shortest common length
//     (> 1/64 of all runs) marks the 1-cell cluster; its mean, refined
//     over all runs of 1..3 cells, gives samples per cell.
//  3. Cells: each run becomes round(len / period) cells of its level. Valid
//     runs are 1, 2 or 3 cells (3 only in preambles).
//  4. Subframes: find a preamble (B/M/W, either polarity, preceded by a
//     transition), then step 64 cells per subframe. Every slot 4..31 must
//     start with a transition; bit = transition in the middle of the slot.
//     A bad preamble drops sync and the search restarts.
// Passes 1, 3 and 4 are resumable: a step processes a bounded number of
// words or subframes and keeps its position in the decoder state.

#include "selftest/selftest_decode.h"

#include <stdbool.h>
#include <string.h>

#include "selftest/selftest_pattern.h"

#if PICO_ON_DEVICE
#include "pico.h"
// Runs on core 1 beside the oversampling filter: out of XIP so it does
// not compete with core 0 for the flash cache.
#define DECODE_FUNC(name) __not_in_flash_func(name)
#else
#define DECODE_FUNC(name) name
#endif

#define MAX_CELLS   SELFTEST_DECODE_MAX_CELLS
#define HIST_BINS   256u
#define BLOCK       192u

enum { PH_HIST, PH_CELLS, PH_SUBFRAMES, PH_DONE };

static uint32_t cellbits[MAX_CELLS / 32u];
static uint32_t hist[HIST_BINS];

//--------------------------------------------------------------------+
// Runs and cells
//--------------------------------------------------------------------+

typedef void (*run_fn)(selftest_decoder_t *d, uint32_t level, uint32_t len);

static void run_iter_reset(selftest_decoder_t *d) {
    d->word = 0;
    d->level = d->cap[0] >> 31;
    d->run = 0;
    d->count = 0;
    d->first = true;
}

// Calls fn for every complete run in the next `words` words; true when the
// capture is exhausted.
static bool DECODE_FUNC(run_iter)(selftest_decoder_t *d, uint32_t words, run_fn fn) {
    uint32_t level = d->level, run = d->run;
    bool first = d->first;
    uint32_t end = d->word + words < d->n_words ? d->word + words : d->n_words;
    for (uint32_t i = d->word; i < end; i++) {
        uint32_t w = d->cap[i];
        uint32_t rem = 32;
        while (rem) {
            // Bits differing from the current level among the rem
            // unconsumed ones (w is shifted left as bits are consumed).
            uint32_t x = level ? ~w : w;
            if (rem < 32) {
                x &= ~0u << (32 - rem);
            }
            if (x == 0) {
                run += rem;
                break;
            }
            uint32_t z = (uint32_t)__builtin_clz(x);
            run += z;
            if (!first) {
                fn(d, level, run);
                d->count++;
            }
            first = false;
            level ^= 1u;
            run = 0;
            w <<= z;
            rem -= z;
        }
    }
    d->word = end;
    d->level = level;
    d->run = run;
    d->first = first;
    return end >= d->n_words;
}

static void DECODE_FUNC(hist_add)(selftest_decoder_t *d, uint32_t level, uint32_t len) {
    (void)d;
    (void)level;
    hist[len < HIST_BINS ? len : HIST_BINS - 1u]++;
}

// Samples per cell (Q24.8) from the run-length histogram, 0 if none.
static uint32_t DECODE_FUNC(estimate_period)(uint32_t total) {
    uint32_t lo = 1;
    while (lo < HIST_BINS - 1u && (uint64_t)hist[lo] * 64u <= total) {
        lo++;
    }
    if (lo >= HIST_BINS - 1u) {
        return 0;
    }
    // Mean over [lo, 1.5 lo + 1]: the 1-cell cluster, not the 2-cell one.
    uint32_t hi = lo + lo / 2u + 1u;
    uint64_t sum = 0, n = 0;
    for (uint32_t i = lo; i <= hi && i < HIST_BINS - 1u; i++) {
        sum += (uint64_t)hist[i] * i;
        n += hist[i];
    }
    uint32_t ovs = (uint32_t)((sum * 256u + n / 2u) / n);
    // Refine: total samples / total cells over runs of 1..3 cells.
    // Converges from within about +-25 %.
    for (int iter = 0; iter < 4; iter++) {
        uint64_t samples = 0, cells = 0;
        for (uint32_t i = 1; i < HIST_BINS - 1u; i++) {
            uint32_t c = (i * 256u + ovs / 2u) / ovs;
            if (c >= 1u && c <= 3u) {
                samples += (uint64_t)hist[i] * i;
                cells += (uint64_t)hist[i] * c;
            }
        }
        if (cells == 0) {
            break;
        }
        ovs = (uint32_t)((samples * 256u + cells / 2u) / cells);
    }
    return ovs;
}

static void DECODE_FUNC(add_cells)(selftest_decoder_t *d, uint32_t level, uint32_t len) {
    // 32-bit division: a run is shorter than the capture (< 2^20 samples).
    uint32_t ovs = d->r->ovs_q8;
    uint32_t n = (len * 256u + ovs / 2u) / ovs;
    if (n == 0 || n > 3u) {
        d->cell_err++;
    }
    if (n > 8u) {
        n = 8u;
    }
    d->span_samples += len;
    d->span_cells += n;
    for (uint32_t k = 0; k < n && d->ncells < MAX_CELLS; k++, d->ncells++) {
        if (level) {
            cellbits[d->ncells >> 5] |= 0x80000000u >> (d->ncells & 31u);
        }
    }
}

static inline uint32_t cell(uint32_t i) {
    return (cellbits[i >> 5] >> (31u - (i & 31u))) & 1u;
}

//--------------------------------------------------------------------+
// Subframes
//--------------------------------------------------------------------+

enum { PRE_NONE = -1, PRE_B = 0, PRE_M, PRE_W };

// Preamble at cell i, either polarity. Cells for a preceding level of 0:
// B 11101000, M 11100010, W 11100100.
static int DECODE_FUNC(preamble_at)(uint32_t i) {
    uint32_t v = 0;
    for (uint32_t k = 0; k < 8u; k++) {
        v = (v << 1) | cell(i + k);
    }
    if (!(v & 0x80u)) {
        v ^= 0xFFu;
    }
    switch (v) {
    case 0xE8: return PRE_B;
    case 0xE2: return PRE_M;
    case 0xE4: return PRE_W;
    default:   return PRE_NONE;
    }
}

// Next preamble at or after from that follows a transition, searching at
// most `limit` cells; the position reached if none.
static uint32_t DECODE_FUNC(find_preamble)(uint32_t from, uint32_t limit, uint32_t ncells) {
    uint32_t i = from < 1u ? 1u : from;
    uint32_t end = i + limit;
    for (; i < end && i + 64u <= ncells; i++) {
        if (cell(i - 1u) != cell(i) && preamble_at(i) != PRE_NONE) {
            return i;
        }
    }
    return i;
}

static inline bool pattern_pair_ok(unsigned bits, uint32_t l, uint32_t r) {
    if (bits == 16) {
        return (l & 0xFFu) == 0 && r == (~l & 0x00FFFF00u);
    }
    return r == (~l & SELFTEST_MASK24);
}

static inline bool pattern_step_ok(unsigned bits, uint32_t prev_l, uint32_t l) {
    uint32_t step = bits == 16 ? SELFTEST_STEP16 << 8 : SELFTEST_STEP24;
    return ((l - prev_l) & SELFTEST_MASK24) == step;
}

static void DECODE_FUNC(frame_done)(selftest_decoder_t *d, uint32_t l, uint32_t r_sample) {
    selftest_decode_t *r = d->r;
    if (d->frames_out && d->frames_stored < d->frames_max) {
        d->frames_out[2u * d->frames_stored] = (int32_t)(l << 8);
        d->frames_out[2u * d->frames_stored + 1u] = (int32_t)(r_sample << 8);
        d->frames_stored++;
    }
    if (l == 0 && r_sample == 0) {
        r->silent++;
        d->s.have_prev = false;
        d->s.in_gap = d->s.seen_audio;
        return;
    }
    if (d->pattern_bits) {
        if (d->s.in_gap) {
            r->gaps++;
        }
        bool ok = pattern_pair_ok(d->pattern_bits, l, r_sample) &&
                  (!d->s.have_prev || pattern_step_ok(d->pattern_bits, d->s.prev_l, l));
        if (!ok) {
            if (r->err_sample == 0) {
                r->viol_frame = r->frames;
                r->viol_prev = d->s.have_prev ? d->s.prev_l : 0u;
                r->viol_l = l;
                r->viol_r = r_sample;
            }
            r->err_sample++;
        }
    }
    d->s.seen_audio = true;
    d->s.in_gap = false;
    d->s.prev_l = l;
    d->s.have_prev = true;
}

// One subframe at d->pos (a preamble position). Resynchronises on a bad
// preamble.
static void DECODE_FUNC(decode_subframe)(selftest_decoder_t *d) {
    selftest_decode_t *r = d->r;
    uint32_t pos = d->pos;
    int t = preamble_at(pos);
    if (t == PRE_NONE) {
        r->err_cell++;
        memset(&d->s, 0, sizeof(d->s));
        d->pos = pos + 1u;
        d->searching = true;
        return;
    }

    bool bad = pos > 0 && cell(pos - 1u) == cell(pos);
    uint32_t data = 0;          // slot k in bit k (4..31)
    for (uint32_t k = 4; k < 32u; k++) {
        uint32_t p = pos + 2u * k;
        uint32_t c0 = cell(p), c1 = cell(p + 1u);
        if (c0 == cell(p - 1u)) {
            bad = true;         // no transition at the slot boundary
        }
        data |= (c0 ^ c1) << k;
    }
    if (bad) {
        r->err_cell++;
    }
    if ((uint32_t)__builtin_popcount(data) & 1u) {
        r->err_parity++;
    }
    if ((data >> 28) & 3u) {
        r->err_vu++;
    }
    uint32_t sample = (data >> 4) & SELFTEST_MASK24;
    uint32_t cbit = (data >> 30) & 1u;
    bool is_a = t != PRE_W;

    if (d->s.in_sync && is_a == d->s.last_was_a) {
        r->err_order++;
    }
    if (is_a) {
        if (t == PRE_B) {
            if (d->s.have_b) {
                if (d->s.fc + 1u != BLOCK) {
                    r->err_order++;
                } else {
                    bool eq = memcmp(d->s.cs, d->expect_cs, SELFTEST_CS_BYTES) == 0;
                    r->cs_match = (eq && r->cs_match != 0) ? 1 : 0;
                    memcpy(r->cs, d->s.cs, SELFTEST_CS_BYTES);
                }
            }
            r->blocks++;
            d->s.have_b = true;
            d->s.fc = 0;
            memset(d->s.cs, 0, sizeof(d->s.cs));
        } else if (d->s.have_b && ++d->s.fc >= BLOCK) {
            r->err_order++;     // B missing
            d->s.have_b = false;
        }
        if (d->s.have_b && cbit) {
            d->s.cs[d->s.fc >> 3] |= (uint8_t)(1u << (d->s.fc & 7u));
        }
        d->s.a_sample = sample;
        d->s.a_cbit = cbit;
        d->s.have_a = true;
    } else {
        if (d->s.have_a) {
            frame_done(d, d->s.a_sample, sample);
            if (cbit != d->s.a_cbit) {
                r->err_cs_ab++;
            }
            r->frames++;
        }
        d->s.have_a = false;
    }
    d->s.last_was_a = is_a;
    d->s.in_sync = true;
    d->pos = pos + 64u;
}

// Up to `subframes` subframes; a preamble search costs 64 cells per
// subframe of budget. True when the cells are exhausted.
static bool DECODE_FUNC(subframe_pass)(selftest_decoder_t *d, uint32_t subframes) {
    uint32_t ncells = d->ncells;
    while (subframes) {
        if (d->pos + 64u > ncells) {
            return true;
        }
        if (d->searching) {
            uint32_t found = find_preamble(d->pos, 64u * subframes, ncells);
            if (found + 64u > ncells) {
                d->pos = found;
                return true;
            }
            if (preamble_at(found) == PRE_NONE || cell(found - 1u) == cell(found)) {
                d->pos = found;     // budget exhausted inside the search
                return false;
            }
            d->pos = found;
            d->searching = false;
            d->synced_once = true;
            continue;
        }
        decode_subframe(d);
        subframes--;
    }
    return false;
}

//--------------------------------------------------------------------+
// Entry points
//--------------------------------------------------------------------+

void selftest_decode_begin(selftest_decoder_t *d, const uint32_t *cap, uint32_t n_words,
                           const uint8_t expect_cs[SELFTEST_CS_BYTES], unsigned pattern_bits,
                           int32_t *frames_out, uint32_t frames_max, selftest_decode_t *r) {
    memset(d, 0, sizeof(*d));
    memset(r, 0, sizeof(*r));
    d->cap = cap;
    d->n_words = n_words;
    d->expect_cs = expect_cs;
    d->pattern_bits = pattern_bits;
    d->frames_out = frames_out;
    d->frames_max = frames_out ? frames_max : 0u;
    d->r = r;
    r->cs_match = -1;
    if (n_words == 0) {
        r->status = SELFTEST_DEC_NOSIG;
        d->phase = PH_DONE;
        return;
    }
    memset(hist, 0, sizeof(hist));
    run_iter_reset(d);
    d->phase = PH_HIST;
}

bool DECODE_FUNC(selftest_decode_step)(selftest_decoder_t *d, uint32_t words, uint32_t subframes) {
    selftest_decode_t *r = d->r;
    switch (d->phase) {
    case PH_HIST:
        if (!run_iter(d, words, hist_add)) {
            return false;
        }
        r->runs = d->count;
        if (r->runs == 0) {
            r->status = SELFTEST_DEC_NOSIG;
            r->level = d->cap[0] >> 31;
            d->phase = PH_DONE;
            return true;
        }
        r->ovs_q8 = estimate_period(r->runs);
        if (r->ovs_q8 < 256u) {         // under 1 sample per cell
            r->status = SELFTEST_DEC_NOSYNC;
            d->phase = PH_DONE;
            return true;
        }
        memset(cellbits, 0, sizeof(cellbits));
        run_iter_reset(d);
        d->phase = PH_CELLS;
        return false;
    case PH_CELLS:
        if (!run_iter(d, words, add_cells)) {
            return false;
        }
        r->err_cell = d->cell_err;
        r->span_samples = d->span_samples;
        r->span_cells = d->span_cells;
        r->status = SELFTEST_DEC_OK;
        d->pos = 0;
        d->searching = true;
        d->phase = PH_SUBFRAMES;
        return false;
    case PH_SUBFRAMES:
        if (!subframe_pass(d, subframes)) {
            return false;
        }
        if (!d->synced_once) {
            r->status = SELFTEST_DEC_NOSYNC;
        }
        d->phase = PH_DONE;
        return true;
    default:
        return true;
    }
}

uint32_t selftest_decode_frames_stored(const selftest_decoder_t *d) {
    return d->frames_stored;
}

void selftest_decode(const uint32_t *cap, uint32_t n_words,
                     const uint8_t expect_cs[SELFTEST_CS_BYTES], unsigned pattern_bits,
                     selftest_decode_t *r) {
    static selftest_decoder_t d;
    selftest_decode_begin(&d, cap, n_words, expect_cs, pattern_bits, NULL, 0, r);
    while (!selftest_decode_step(&d, 0xFFFFFFFFu, 0xFFFFFFFFu / 64u)) {
    }
}

uint32_t selftest_decode_errors(const selftest_decode_t *r) {
    return r->err_cell + r->err_parity + r->err_order + r->err_vu + r->err_cs_ab +
           r->err_sample + (r->cs_match != 1 ? 1u : 0u);
}

bool selftest_decode_frames_clean(const selftest_decode_t *r) {
    return r->status == SELFTEST_DEC_OK && r->err_cell == 0 && r->err_parity == 0 &&
           r->err_order == 0 && r->frames != 0;
}
