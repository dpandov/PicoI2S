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

#include "selftest/selftest_decode.h"

#include <stdbool.h>
#include <string.h>

#include "selftest/selftest_pattern.h"

#if PICO_ON_DEVICE
#include "pico.h"
// Runs continuously on core 1: out of XIP so it does not compete with
// core 0 for the flash cache.
#define DECODE_FUNC(name) __not_in_flash_func(name)
#else
#define DECODE_FUNC(name) name
#endif

// A window of 3 blocks at up to ~11 samples per cell fits; bounds the cell
// buffer to 16 KB.
#define MAX_CELLS   131072u
#define HIST_BINS   256u
#define BLOCK       192u

static uint32_t cellbits[MAX_CELLS / 32u];
static uint32_t hist[HIST_BINS];

//--------------------------------------------------------------------+
// Runs and cells
//--------------------------------------------------------------------+

typedef void (*run_fn)(void *ctx, uint32_t level, uint32_t len);

// Calls fn for every complete run; returns the number of runs.
static uint32_t DECODE_FUNC(for_each_run)(const uint32_t *cap, uint32_t n_words, run_fn fn,
                                          void *ctx) {
    uint32_t level = cap[0] >> 31;
    uint32_t run = 0, count = 0;
    bool first = true;
    for (uint32_t i = 0; i < n_words; i++) {
        uint32_t w = cap[i];
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
                fn(ctx, level, run);
                count++;
            }
            first = false;
            level ^= 1u;
            run = 0;
            w <<= z;
            rem -= z;
        }
    }
    return count;
}

static void DECODE_FUNC(hist_add)(void *ctx, uint32_t level, uint32_t len) {
    (void)ctx;
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

typedef struct {
    uint32_t ovs_q8;
    uint32_t ncells;
    uint32_t err_cell;
    uint64_t span_samples;
    uint32_t span_cells;
} cell_ctx_t;

static void DECODE_FUNC(add_cells)(void *vctx, uint32_t level, uint32_t len) {
    cell_ctx_t *c = vctx;
    // 32-bit division: a run is shorter than the capture (< 2^20 samples).
    uint32_t n = (len * 256u + c->ovs_q8 / 2u) / c->ovs_q8;
    if (n == 0 || n > 3u) {
        c->err_cell++;
    }
    if (n > 8u) {
        n = 8u;
    }
    c->span_samples += len;
    c->span_cells += n;
    for (uint32_t k = 0; k < n && c->ncells < MAX_CELLS; k++, c->ncells++) {
        if (level) {
            cellbits[c->ncells >> 5] |= 0x80000000u >> (c->ncells & 31u);
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

// Next preamble at or after from that follows a transition; ncells if none.
static uint32_t DECODE_FUNC(find_preamble)(uint32_t from, uint32_t ncells) {
    for (uint32_t i = from < 1u ? 1u : from; i + 64u <= ncells; i++) {
        if (cell(i - 1u) != cell(i) && preamble_at(i) != PRE_NONE) {
            return i;
        }
    }
    return ncells;
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

typedef struct {
    bool     in_sync;
    bool     last_was_a;
    bool     have_b;        // a B was seen since sync: fc is valid
    uint32_t fc;            // frame index in the block
    bool     have_a;        // subframe A of the current frame decoded
    uint32_t a_sample, a_cbit;
    bool     have_prev;     // prev_l holds the last non-silent L
    uint32_t prev_l;
    bool     seen_audio;    // a non-silent frame was seen since sync
    bool     in_gap;        // silence after a non-silent frame
    uint8_t  cs[SELFTEST_CS_BYTES];
} frame_state_t;

static void DECODE_FUNC(frame_done)(frame_state_t *s, uint32_t l, uint32_t r_sample,
                                    unsigned pattern_bits, selftest_decode_t *r) {
    if (l == 0 && r_sample == 0) {
        r->silent++;
        s->have_prev = false;
        s->in_gap = s->seen_audio;
        return;
    }
    if (pattern_bits) {
        if (s->in_gap) {
            r->gaps++;
        }
        bool ok = pattern_pair_ok(pattern_bits, l, r_sample) &&
                  (!s->have_prev || pattern_step_ok(pattern_bits, s->prev_l, l));
        if (!ok) {
            if (r->err_sample == 0) {
                r->viol_frame = r->frames;
                r->viol_prev = s->have_prev ? s->prev_l : 0u;
                r->viol_l = l;
                r->viol_r = r_sample;
            }
            r->err_sample++;
        }
    }
    s->seen_audio = true;
    s->in_gap = false;
    s->prev_l = l;
    s->have_prev = true;
}

static void DECODE_FUNC(decode_subframes)(uint32_t ncells,
                                          const uint8_t expect_cs[SELFTEST_CS_BYTES],
                                          unsigned pattern_bits, selftest_decode_t *r) {
    frame_state_t s;
    memset(&s, 0, sizeof(s));

    uint32_t pos = find_preamble(0, ncells);
    if (pos >= ncells) {
        r->status = SELFTEST_DEC_NOSYNC;
        return;
    }
    while (pos + 64u <= ncells) {
        int t = preamble_at(pos);
        if (t == PRE_NONE) {
            r->err_cell++;
            memset(&s, 0, sizeof(s));
            pos = find_preamble(pos + 1u, ncells);
            continue;
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

        if (s.in_sync && is_a == s.last_was_a) {
            r->err_order++;
        }
        if (is_a) {
            if (t == PRE_B) {
                if (s.have_b) {
                    if (s.fc + 1u != BLOCK) {
                        r->err_order++;
                    } else {
                        bool eq = memcmp(s.cs, expect_cs, SELFTEST_CS_BYTES) == 0;
                        r->cs_match = (eq && r->cs_match != 0) ? 1 : 0;
                        memcpy(r->cs, s.cs, SELFTEST_CS_BYTES);
                    }
                }
                r->blocks++;
                s.have_b = true;
                s.fc = 0;
                memset(s.cs, 0, sizeof(s.cs));
            } else if (s.have_b && ++s.fc >= BLOCK) {
                r->err_order++;     // B missing
                s.have_b = false;
            }
            if (s.have_b && cbit) {
                s.cs[s.fc >> 3] |= (uint8_t)(1u << (s.fc & 7u));
            }
            s.a_sample = sample;
            s.a_cbit = cbit;
            s.have_a = true;
        } else {
            if (s.have_a) {
                frame_done(&s, s.a_sample, sample, pattern_bits, r);
                if (cbit != s.a_cbit) {
                    r->err_cs_ab++;
                }
                r->frames++;
            }
            s.have_a = false;
        }
        s.last_was_a = is_a;
        s.in_sync = true;
        pos += 64u;
    }
}

//--------------------------------------------------------------------+
// Entry points
//--------------------------------------------------------------------+

void DECODE_FUNC(selftest_decode)(const uint32_t *cap, uint32_t n_words,
                                  const uint8_t expect_cs[SELFTEST_CS_BYTES],
                                  unsigned pattern_bits, selftest_decode_t *r) {
    memset(r, 0, sizeof(*r));
    r->cs_match = -1;
    if (n_words == 0) {
        r->status = SELFTEST_DEC_NOSIG;
        return;
    }

    memset(hist, 0, sizeof(hist));
    r->runs = for_each_run(cap, n_words, hist_add, NULL);
    if (r->runs == 0) {
        r->status = SELFTEST_DEC_NOSIG;
        r->level = cap[0] >> 31;
        return;
    }
    r->ovs_q8 = estimate_period(r->runs);
    if (r->ovs_q8 < 256u) {         // under 1 sample per cell
        r->status = SELFTEST_DEC_NOSYNC;
        return;
    }

    memset(cellbits, 0, sizeof(cellbits));
    cell_ctx_t c = {.ovs_q8 = r->ovs_q8};
    for_each_run(cap, n_words, add_cells, &c);
    r->err_cell = c.err_cell;
    r->span_samples = c.span_samples;
    r->span_cells = c.span_cells;

    r->status = SELFTEST_DEC_OK;
    decode_subframes(c.ncells, expect_cs, pattern_bits, r);
}

uint32_t selftest_decode_errors(const selftest_decode_t *r) {
    return r->err_cell + r->err_parity + r->err_order + r->err_vu + r->err_cs_ab +
           r->err_sample + (r->cs_match != 1 ? 1u : 0u);
}
