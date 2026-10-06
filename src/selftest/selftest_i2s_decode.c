// I2S / dual-line decoder, see selftest_i2s_decode.h.
//
// LRCK formats: from the first LRCK edge, every half frame must be F/2
// slots with the next edge right after it; a wrong half drops sync and the
// search restarts at the next edge. The word is the F/2 samples from the
// edge (left-justified), the F/2 after it (I2S), or the last 24 / 16 of
// the half (right-justified). Dual-line: a LATCH pulse (the level the line
// holds least, one sample wide) marks slot latch_slot of a frame; the word
// is the `width` samples ending at the frame's LSB slot, and the next
// pulse must come F samples later.
//
// Passes: a scan (BCLK must be low in every sample, LATCH polarity), the
// framing into the frame buffer, then the check of the frames against the
// pattern or the reference stream. Each pass is resumable.

#include "selftest/selftest_i2s_decode.h"

#include <stdbool.h>
#include <string.h>

#include "selftest/selftest_pattern.h"

#if PICO_ON_DEVICE
#include "pico.h"
#define DECODE_FUNC(name) __not_in_flash_func(name)
#else
#define DECODE_FUNC(name) name
#endif

// Consecutive frames (not silent, not clipped) that locate the captured
// window in the reference.
#define KEY_FRAMES      8u
// Budget charged per frame or reference position in the check passes.
#define CHECK_COST      16u

enum {
    PH_SCAN,
    PH_LRCK_EDGE,       // searching the next LRCK edge
    PH_LRCK,            // checking half frames
    PH_LATCH_SEARCH,    // searching the next LATCH pulse
    PH_LATCH_RUN,       // measuring a pulse, decoding its frame
    PH_PATTERN,
    PH_LOCATE,
    PH_SEARCH,
    PH_COMPARE,
    PH_DONE,
};

unsigned selftest_i2s_format_width(i2s_format_t f) {
    switch (f) {
    case I2S_FORMAT_RIGHT_JUSTIFIED_24: return 24u;
    case I2S_FORMAT_RIGHT_JUSTIFIED_16: return 16u;
    case I2S_FORMAT_DUAL_16:            return 16u;
    case I2S_FORMAT_DUAL_18:            return 18u;
    case I2S_FORMAT_DUAL_20:            return 20u;
    case I2S_FORMAT_DUAL_24:            return 24u;
    default:                            return 32u;
    }
}

//--------------------------------------------------------------------+
// Samples
//--------------------------------------------------------------------+

static inline uint32_t sample(const selftest_i2s_decoder_t *d, uint32_t i) {
    uint32_t per_word = 1u << d->sample_shift;
    uint32_t shift = 32u - d->lay.sample_bits * ((i & (per_word - 1u)) + 1u);
    return (d->cap[i >> d->sample_shift] >> shift) & d->sample_mask;
}

static inline uint32_t line(const selftest_i2s_decoder_t *d, uint32_t i) {
    return (sample(d, i) >> (d->lay.clk_bit + 1u)) & 1u;
}

static inline uint32_t bclk(const selftest_i2s_decoder_t *d, uint32_t i) {
    return (sample(d, i) >> d->lay.clk_bit) & 1u;
}

// Bits of samples first .. first + count - 1 (channel bit 0 or 1), MSB first,
// left-justified in 32 bits.
static uint32_t DECODE_FUNC(word)(const selftest_i2s_decoder_t *d, uint32_t first, uint32_t count,
                                  unsigned chan) {
    uint32_t w = 0;
    for (uint32_t k = 0; k < count; k++) {
        w = (w << 1) | ((sample(d, first + k) >> chan) & 1u);
    }
    return w << (32u - count);
}

static void DECODE_FUNC(store_frame)(selftest_i2s_decoder_t *d, uint32_t l, uint32_t r) {
    if (d->r->frames < d->max_frames) {
        selftest_i2s_frame_t *f = &d->frames[d->r->frames];
        f->l = l;
        f->r = r;
        f->resync = d->resync;
    }
    d->resync = false;
    d->r->frames++;
    if (l == 0 && r == 0) {
        d->r->silent++;
    }
}

//--------------------------------------------------------------------+
// Framing
//--------------------------------------------------------------------+

// LRCK: half frames of `half` slots. True when the capture is exhausted.
static bool DECODE_FUNC(lrck_pass)(selftest_i2s_decoder_t *d, uint32_t *budget) {
    uint32_t half = d->lay.slots / 2u;
    unsigned width = d->lay.width;
    uint32_t n = d->n;
    while (*budget) {
        if (d->phase == PH_LRCK_EDGE) {
            uint32_t i = d->i;
            while (i < n && *budget && line(d, i) == line(d, i - 1u)) {
                i++;
                (*budget)--;
            }
            if (i >= n) {
                return true;
            }
            if (line(d, i) == line(d, i - 1u)) {
                d->i = i;
                return false;
            }
            d->e = i;
            d->synced = true;
            d->have_l = false;
            d->resync = true;
            d->phase = PH_LRCK;
            continue;
        }
        // Half frame e .. e + half - 1, next edge at e + half (for I2S also
        // the last data slot).
        uint32_t e = d->e;
        if (e + half >= n) {
            return true;
        }
        uint32_t lvl = line(d, e);
        bool ok = line(d, e + half) != lvl;
        for (uint32_t k = 1; ok && k < half; k++) {
            ok = line(d, e + k) == lvl;
        }
        *budget -= *budget > half ? half : *budget;
        if (!ok) {
            d->r->err_frame++;
            d->i = e + 1u;
            d->phase = PH_LRCK_EDGE;
            continue;
        }
        uint32_t w;
        bool left;
        if (d->lay.format == I2S_FORMAT_I2S) {
            w = word(d, e + 1u, half, 0);
            left = lvl == 0;
        } else {
            w = word(d, e + half - width, width, 0);
            left = lvl == 1;
        }
        if (left) {
            d->l = w;
            d->have_l = true;
        } else if (d->have_l) {
            store_frame(d, d->l, w);
            d->have_l = false;
        }
        d->e = e + half;
    }
    return false;
}

// LATCH: pulses F slots apart. True when the capture is exhausted.
static bool DECODE_FUNC(latch_pass)(selftest_i2s_decoder_t *d, uint32_t *budget) {
    uint32_t n = d->n, slots = d->lay.slots;
    unsigned width = d->lay.width;
    uint32_t msb_xor = d->lay.offset_binary ? 0x80000000u : 0u;
    while (*budget) {
        uint32_t i = d->i;
        if (d->phase == PH_LATCH_SEARCH) {
            while (i < n && *budget && line(d, i) == d->idle) {
                i++;
                (*budget)--;
            }
            d->i = i;
            if (i >= n) {
                return true;
            }
            if (line(d, i) == d->idle) {
                return false;
            }
            if (d->synced && i != d->pulse + slots) {
                d->r->err_frame++;
                d->resync = true;
            }
            d->pulse = i;
            d->synced = true;
            d->phase = PH_LATCH_RUN;
            continue;
        }
        while (i < n && *budget && line(d, i) != d->idle) {
            i++;
            (*budget)--;
        }
        d->i = i;
        if (i >= n) {
            return true;            // pulse still active at the window end
        }
        if (line(d, i) != d->idle) {
            return false;
        }
        if (i - d->pulse != 1u) {
            d->r->err_frame++;
        }
        // The frame starts latch_slot slots before the pulse; its word
        // ends at the LSB slot.
        uint32_t start = d->pulse + slots - d->lay.latch_slot;       // + slots: stays unsigned
        uint32_t first = start + d->lay.lsb_slot + 1u - width;
        if (first >= slots && first + width - slots <= n) {
            first -= slots;
            store_frame(d, word(d, first, width, 0) ^ msb_xor, word(d, first, width, 1) ^ msb_xor);
            *budget -= *budget > 2u * width ? 2u * width : *budget;
        }
        d->phase = PH_LATCH_SEARCH;
    }
    return false;
}

//--------------------------------------------------------------------+
// Checks on the decoded frames
//--------------------------------------------------------------------+

static void pattern_init(selftest_i2s_decoder_t *d) {
    unsigned bits = d->pattern_bits, width = d->lay.width;
    d->p.e = bits < width ? bits : width;
    d->p.mask_e = 0xFFFFFFFFu >> (32u - d->p.e);
    d->p.mask_low = ~(0xFFFFFFFFu << (32u - d->p.e));
    if (bits == 16) {
        d->p.step = SELFTEST_STEP16;
    } else {
        d->p.step = SELFTEST_STEP24 >> (24u - d->p.e);
        d->p.step_carry = d->p.e < 24u;
    }
}

static void DECODE_FUNC(pattern_frame)(selftest_i2s_decoder_t *d, uint32_t index) {
    const selftest_i2s_frame_t *f = &d->frames[index];
    selftest_i2s_decode_t *r = d->r;
    if (f->resync) {
        d->p.have_prev = false;
    }
    if (f->l == 0 && f->r == 0) {
        d->p.have_prev = false;
        d->p.in_gap = d->p.seen_audio;
        return;
    }
    if (d->p.in_gap) {
        r->gaps++;
    }
    uint32_t le = f->l >> (32u - d->p.e), re = f->r >> (32u - d->p.e);
    bool ok = ((f->l | f->r) & d->p.mask_low) == 0 && re == (~le & d->p.mask_e);
    if (ok && d->p.have_prev) {
        uint32_t delta = (le - (d->p.prev_l >> (32u - d->p.e))) & d->p.mask_e;
        ok = delta == d->p.step || (d->p.step_carry && delta == ((d->p.step + 1u) & d->p.mask_e));
    }
    if (!ok) {
        if (r->err_sample == 0) {
            r->viol_frame = index;
            r->viol_prev = d->p.have_prev ? d->p.prev_l : 0u;
            r->viol_l = f->l;
            r->viol_r = f->r;
        }
        r->err_sample++;
    }
    d->p.seen_audio = true;
    d->p.in_gap = false;
    d->p.prev_l = f->l;
    d->p.have_prev = true;
}

static void DECODE_FUNC(mismatch)(selftest_i2s_decoder_t *d, uint32_t index, uint32_t expect_l) {
    selftest_i2s_decode_t *r = d->r;
    if (r->err_sample == 0) {
        r->viol_frame = index;
        r->viol_prev = expect_l;
        r->viol_l = d->frames[index].l;
        r->viol_r = d->frames[index].r;
    }
    r->err_sample++;
}

// A frame that can be part of the locate key: not silent, and neither
// channel at a clip value (full scale truncated to the width).
static inline bool key_frame(const selftest_i2s_decoder_t *d, const selftest_i2s_frame_t *f) {
    uint32_t max = 0x7FFFFFFFu & (0xFFFFFFFFu << (32u - d->lay.width));
    if ((f->l | f->r) == 0) {
        return false;
    }
    return f->l != 0x80000000u && f->l != max && f->r != 0x80000000u && f->r != max;
}

static bool DECODE_FUNC(key_matches)(const selftest_i2s_decoder_t *d, uint32_t pos) {
    for (uint32_t k = 0; k < d->key; k++) {
        const selftest_i2s_frame_t *f = &d->frames[d->first + k];
        if (f->l != d->ref->l[pos + k] || f->r != d->ref->r[pos + k]) {
            return false;
        }
    }
    return true;
}

// The frames left to check: those stored.
static inline uint32_t stored_frames(const selftest_i2s_decoder_t *d) {
    return d->r->frames < d->max_frames ? d->r->frames : d->max_frames;
}

static bool DECODE_FUNC(check_pass)(selftest_i2s_decoder_t *d, uint32_t *budget) {
    uint32_t n = stored_frames(d);
    while (*budget) {
        *budget -= *budget > CHECK_COST ? CHECK_COST : *budget;
        switch (d->phase) {
        case PH_PATTERN:
            if (d->i >= n) {
                return true;
            }
            pattern_frame(d, d->i++);
            break;
        case PH_LOCATE: {
            // The key: the first KEY_FRAMES consecutive frames that are
            // neither silent nor clipped. A clipped run (the filter's
            // overshoot at full scale) looks the same wherever it occurs,
            // so a key inside one can match the wrong place.
            uint32_t run = 0;
            while (d->i < n && run < KEY_FRAMES) {
                run = key_frame(d, &d->frames[d->i]) ? run + 1u : 0u;
                d->i++;
            }
            if (run < 2u) {
                return true;        // nothing usable to locate
            }
            d->key = run;
            d->first = d->i - run;
            d->pos = d->ref->search_from;
            d->phase = PH_SEARCH;
            break;
        }
        case PH_SEARCH:
            if (d->pos + d->key > d->ref->n) {
                // Not in the reference: every non-silent frame is wrong.
                for (uint32_t k = 0; k < n; k++) {
                    if (d->frames[k].l | d->frames[k].r) {
                        mismatch(d, k, 0);
                    }
                }
                return true;
            }
            if (key_matches(d, d->pos)) {
                d->r->located = true;
                d->r->offset = (int32_t)d->pos - (int32_t)d->first;
                d->i = 0;
                d->phase = PH_COMPARE;
                break;
            }
            d->pos++;
            break;
        case PH_COMPARE: {
            if (d->i >= n) {
                return true;
            }
            uint32_t c = d->i++;
            int32_t idx = d->r->offset + (int32_t)c;
            if (idx < 0) {
                break;              // before the reference starts
            }
            if ((uint32_t)idx >= d->ref->n) {
                mismatch(d, c, 0);  // beyond the reference: uncovered
                break;
            }
            if (d->frames[c].l != d->ref->l[idx] || d->frames[c].r != d->ref->r[idx]) {
                mismatch(d, c, d->ref->l[idx]);
            }
            break;
        }
        default:
            return true;
        }
    }
    return false;
}

//--------------------------------------------------------------------+
// Entry points
//--------------------------------------------------------------------+

void selftest_i2s_decode_begin(selftest_i2s_decoder_t *d, const uint32_t *cap, uint32_t n_samples,
                               const selftest_i2s_layout_t *lay, unsigned pattern_bits,
                               const selftest_i2s_reference_t *ref, selftest_i2s_frame_t *frames,
                               uint32_t max_frames, selftest_i2s_decode_t *r) {
    memset(d, 0, sizeof(*d));
    memset(r, 0, sizeof(*r));
    d->cap = cap;
    d->n = n_samples;
    d->lay = *lay;
    d->pattern_bits = pattern_bits;
    d->ref = ref;
    d->frames = frames;
    d->max_frames = max_frames;
    d->r = r;
    d->sample_shift = lay->sample_bits == 8u ? 2u : 1u;
    d->sample_mask = 0xFFFFFFFFu >> (32u - lay->sample_bits);
    if (n_samples == 0 || lay->slots < 2u) {
        r->status = SELFTEST_I2S_NOSYNC;
        d->phase = PH_DONE;
        return;
    }
    r->status = SELFTEST_I2S_OK;
    if (pattern_bits) {
        pattern_init(d);
    }
    d->phase = PH_SCAN;
}

bool DECODE_FUNC(selftest_i2s_decode_step)(selftest_i2s_decoder_t *d, uint32_t budget) {
    selftest_i2s_decode_t *r = d->r;
    bool dual = i2s_format_is_dual((i2s_format_t)d->lay.format);
    switch (d->phase) {
    case PH_SCAN: {
        // Every sample was taken after a BCLK falling edge.
        uint32_t end = d->i + budget < d->n ? d->i + budget : d->n;
        for (uint32_t i = d->i; i < end; i++) {
            r->err_frame += bclk(d, i);
            d->ones += line(d, i);
        }
        d->i = end;
        if (end < d->n) {
            return false;
        }
        d->resync = true;
        if (dual) {
            d->idle = d->ones > d->n / 2u ? 1u : 0u;
            d->i = 0;
            d->phase = PH_LATCH_SEARCH;
        } else {
            d->i = 1;
            d->phase = PH_LRCK_EDGE;
        }
        return false;
    }
    case PH_LRCK_EDGE:
    case PH_LRCK:
    case PH_LATCH_SEARCH:
    case PH_LATCH_RUN: {
        bool done = dual ? latch_pass(d, &budget) : lrck_pass(d, &budget);
        if (!done) {
            return false;
        }
        if (!d->synced) {
            r->status = SELFTEST_I2S_NOSYNC;
            d->phase = PH_DONE;
            return true;
        }
        d->i = 0;
        d->phase = d->ref ? PH_LOCATE : d->pattern_bits ? PH_PATTERN : PH_DONE;
        return d->phase == PH_DONE;
    }
    case PH_PATTERN:
    case PH_LOCATE:
    case PH_SEARCH:
    case PH_COMPARE:
        if (!check_pass(d, &budget)) {
            return false;
        }
        d->phase = PH_DONE;
        return true;
    default:
        return true;
    }
}

void selftest_i2s_decode(const uint32_t *cap, uint32_t n_samples, const selftest_i2s_layout_t *lay,
                         unsigned pattern_bits, const selftest_i2s_reference_t *ref,
                         selftest_i2s_frame_t *frames, uint32_t max_frames,
                         selftest_i2s_decode_t *r) {
    static selftest_i2s_decoder_t d;
    selftest_i2s_decode_begin(&d, cap, n_samples, lay, pattern_bits, ref, frames, max_frames, r);
    while (!selftest_i2s_decode_step(&d, 0x7FFFFFFFu)) {
    }
}

uint32_t selftest_i2s_decode_errors(const selftest_i2s_decode_t *r) {
    return r->err_frame + r->err_sample;
}
