// S/PDIF receiver word parser, see spdif_rx_parse.h.

#include "spdif/spdif_rx_parse.h"

#include <string.h>

#if PICO_ON_DEVICE
#include "pico.h"
#define PARSE_FUNC(name) __not_in_flash_func(name)
#else
#define PARSE_FUNC(name) name
#endif

// block_flags
#define BF_EMPHASIS SPDIF_RX_FLAG_EMPHASIS
#define BF_NONPCM   0x40u
#define BF_KNOWN    0x80u

// Frames whose channel-status bits decide the block's flags (bits 0..5).
#define FLAG_FRAMES 6
// About one garbage word in 16 passes the preamble and parity checks, so a
// good subframe is used only after CONFIRM good ones in a row and when the
// next frame is good as well: noise, a decoder that lost sync or one at the
// wrong rate would otherwise leak full-scale samples.
#define CONFIRM 8u
// Words a pair needs: its own two and the next frame's.
#define PAIR_WORDS 4u

_Static_assert(sizeof(((spdif_rx_parser_t *)0)->carry) / sizeof(uint32_t) == PAIR_WORDS - 1u,
               "carry holds the words of an undecided pair");

const uint32_t spdif_rx_rates[5] = {32000, 44100, 48000, 88200, 96000};

// Byte 0 of the channel status. Consumer (IEC 60958-3): bit 1 non-PCM,
// bits 3..5 = 1, 0, 0 for 50/15 us. Professional (bit 0, AES3): bits
// 2..4 = 1, 1, 0 for 50/15 us.
static uint8_t flags_from_cs0(uint8_t b0) {
    uint8_t f = BF_KNOWN;
    if (b0 & 0x02u)
        f |= BF_NONPCM;
    bool emph = (b0 & 0x01u) ? (b0 & 0x1Cu) == 0x0Cu : (b0 & 0x38u) == 0x08u;
    if (emph)
        f |= BF_EMPHASIS;
    return f;
}

void spdif_rx_parser_init(spdif_rx_parser_t *p) {
    memset(p, 0, sizeof(*p));
    p->frame = -1;
}

void spdif_rx_parser_resync(spdif_rx_parser_t *p) {
    p->aligned = false;
    p->emitting = false;
    p->n_carry = 0;
    p->frame = -1;
    p->good_run = 0;
    p->bad_run = 0;
    p->block_flags = 0;
    p->n_out = 0;
}

void spdif_rx_parser_set_output(spdif_rx_parser_t *p, spdif_rx_emit_fn fn, void *ctx) {
    p->emit_fn = fn;
    p->emit_ctx = ctx;
    p->emitting = false;
    p->n_out = 0;
}

static void PARSE_FUNC(flush)(spdif_rx_parser_t *p) {
    if (p->n_out == 0 || !(p->block_flags & BF_KNOWN))
        return;
    if (p->emitting) {
        if (p->block_flags & BF_NONPCM)
            memset(p->out, 0, p->n_out * 2u * sizeof(p->out[0]));
        // Bits 3..5 mean something else in a data stream.
        uint32_t emphasis = p->block_flags & BF_NONPCM ? 0u : p->block_flags & BF_EMPHASIS;
        p->emit_fn(p->emit_ctx, p->out, p->n_out, emphasis);
    }
    p->n_out = 0;
}

// The block in progress is abandoned: framing lost.
static void lose_frame(spdif_rx_parser_t *p) {
    flush(p);
    p->n_out = 0;
    p->frame = -1;
    p->block_flags = 0;
    p->slips++;
}

static inline void PARSE_FUNC(count)(spdif_rx_parser_t *p, bool good) {
    if (good) {
        p->good_run++;
        p->bad_run = 0;
    } else {
        p->bad_run++;
        p->good_run = 0;
    }
}

// Word with the preamble code in bits 0..2 and the parity error in bit 31.
static inline uint32_t check_bits(uint32_t w) {
    return (w ^ (uint32_t)((int32_t)(w << 28) >> 31)) & 0x80000007u;
}

static inline bool good_a(uint32_t w) {
    uint32_t c = check_bits(w);
    return c == SPDIF_RX_PRE_M || c == SPDIF_RX_PRE_B;
}

// One subframe pair in the A, B slots, w[0..1], with the next pair in
// w[2..3]. Returns the words consumed: 1 when the pair turns out to be
// misaligned (w[1] is then looked at again as A).
static uint32_t PARSE_FUNC(pair)(spdif_rx_parser_t *p, const uint32_t *w) {
    uint32_t a = w[0], b = w[1];
    uint32_t ca = spdif_rx_word_code(a);
    uint32_t cb = spdif_rx_word_code(b);
    bool pa = spdif_rx_word_parity_ok(a);
    bool pb = spdif_rx_word_parity_ok(b);
    int f = p->frame;

    if (ca == SPDIF_RX_PRE_W && (cb == SPDIF_RX_PRE_B || cb == SPDIF_RX_PRE_M)) {
        // a was a subframe B: one word lost or added.
        if (f >= 0)
            lose_frame(p);
        p->err_frame++;
        count(p, false);
        p->aligned = false;
        return 1;
    }

    bool a_type = ca == SPDIF_RX_PRE_B || ca == SPDIF_RX_PRE_M;
    bool oka = a_type && pa;
    bool okb = cb == SPDIF_RX_PRE_W && pb;
    if (!a_type)
        p->err_frame++;
    else if (!pa)
        p->err_parity++;
    if (cb != SPDIF_RX_PRE_W)
        p->err_frame++;
    else if (!pb)
        p->err_parity++;
    bool next = good_a(w[2]) && check_bits(w[3]) == SPDIF_RX_PRE_W;
    bool use_a = oka && okb && next && p->good_run >= CONFIRM;

    count(p, oka);
    bool use_b = okb && next && p->good_run >= CONFIRM;
    count(p, okb);

    if (oka && ca == SPDIF_RX_PRE_B) {
        if (f > 0) {
            // B away from position 0.
            lose_frame(p);
            p->err_frame++;
        }
        f = 0;
    } else if (f < 0) {
        return 2;
    } else if (f == 0 && oka) {
        p->err_frame++;     // M where B was due
    }

    if (f == 0) {
        flush(p);
        p->n_out = 0;
        memset(p->cs, 0, sizeof(p->cs));
        p->block_flags = 0;
        p->emitting = p->emit_fn != NULL;
    }

    uint32_t c;
    if (oka)
        c = spdif_rx_word_cbit(a);
    else if (okb)
        c = spdif_rx_word_cbit(b);
    else
        c = p->cs_complete ? (p->cs_last[f >> 3] >> (f & 7)) & 1u : 0u;
    p->cs[f >> 3] |= (uint8_t)(c << (f & 7));

    if (p->emitting) {
        // Full only past the flag frames, where flush() empties it.
        if (p->n_out == SPDIF_RX_OUT_FRAMES)
            flush(p);
        int32_t *o = &p->out[2u * p->n_out];
        o[0] = use_a ? spdif_rx_word_sample(a) : 0;
        o[1] = use_b ? spdif_rx_word_sample(b) : 0;
        p->n_out++;
    }

    if (f == FLAG_FRAMES - 1)
        p->block_flags = flags_from_cs0(p->cs[0]);

    if (++f == (int)SPDIF_RX_BLOCK_FRAMES) {
        flush(p);
        memcpy(p->cs_last, p->cs, sizeof(p->cs));
        p->cs_complete = true;
        f = 0;
    }
    p->frame = (int16_t)f;
    return 2;
}

// Frames inside a block whose flags are known (positions 6..190), after
// CONFIRM good subframes, while both subframes of the frame and of the next
// one are good: up to max frames, w holding one frame more. Returns the
// frames taken. Channel-status bits are gathered 32 at a time
// (little-endian bytes).
static uint32_t PARSE_FUNC(inner_frames)(spdif_rx_parser_t *p, const uint32_t *w, uint32_t max) {
    uint32_t a = w[0], b = w[1];
    if (check_bits(a) != SPDIF_RX_PRE_M || check_bits(b) != SPDIF_RX_PRE_W)
        return 0;
    uint32_t f = (uint32_t)p->frame;
    int32_t *o = NULL;
    if (p->emitting) {
        o = &p->out[2u * p->n_out];
        if (max > SPDIF_RX_OUT_FRAMES - p->n_out)
            max = SPDIF_RX_OUT_FRAMES - p->n_out;
    }
    uint32_t k = 0;
    while (k < max) {
        uint8_t *cw = &p->cs[4u * (f >> 5)];
        uint32_t cs;
        memcpy(&cs, cw, 4);
        uint32_t lim = 32u - (f & 31u);
        if (lim > max - k)
            lim = max - k;
        uint32_t j = 0;
        for (; j < lim; j++, w += 2) {
            // The next frame is at most 191: never a B.
            uint32_t na = w[2], nb = w[3];
            if (check_bits(na) != SPDIF_RX_PRE_M || check_bits(nb) != SPDIF_RX_PRE_W)
                break;
            // NOT slots = Lk XOR Lk+1.
            uint32_t sa = a ^ (a >> 1);
            cs |= (~sa >> 29 & 1u) << ((f + j) & 31u);
            if (o) {
                o[0] = (int32_t)(~(sa << 5) & 0xFFFFFF00u);
                o[1] = (int32_t)(~((b ^ (b >> 1)) << 5) & 0xFFFFFF00u);
                o += 2;
            }
            a = na;
            b = nb;
        }

        memcpy(cw, &cs, 4);
        k += j;
        f += j;
        if (j < lim)
            break;
    }
    if (o)
        p->n_out += k;
    if (k) {
        p->frame = (int16_t)f;
        p->good_run += 2u * k;
        p->bad_run = 0;
    }
    return k;
}

// Consumes words until fewer than a pair needs are left (only while
// aligned). Returns the words consumed.
static uint32_t PARSE_FUNC(parse_words)(spdif_rx_parser_t *p, const uint32_t *w, uint32_t n) {
    uint32_t i = 0;
    while (i < n) {
        if (!p->aligned) {
            if (!good_a(w[i])) {
                count(p, false);
                i++;
                continue;
            }
            p->aligned = true;
        }
        if (n - i < PAIR_WORDS)
            break;
        int f = p->frame;
        if (f >= FLAG_FRAMES && f < (int)SPDIF_RX_BLOCK_FRAMES - 1 && p->good_run >= CONFIRM) {
            if (p->n_out == SPDIF_RX_OUT_FRAMES)
                flush(p);
            uint32_t max = (n - i) / 2u - 1u;
            if (max > SPDIF_RX_BLOCK_FRAMES - 1u - (uint32_t)f)
                max = SPDIF_RX_BLOCK_FRAMES - 1u - (uint32_t)f;
            uint32_t k = inner_frames(p, &w[i], max);
            i += 2u * k;
            if (k)
                continue;
        }
        i += pair(p, &w[i]);
    }
    return i;
}

void PARSE_FUNC(spdif_rx_parse)(spdif_rx_parser_t *p, const uint32_t *w, uint32_t n) {
    if (p->n_carry) {
        // The carried words with the first new ones; what is left of them
        // afterwards lies within the new words.
        uint32_t tmp[PAIR_WORDS - 1u + 2u * PAIR_WORDS];
        uint32_t k = n < 2u * PAIR_WORDS ? n : 2u * PAIR_WORDS;
        memcpy(tmp, p->carry, p->n_carry * 4u);
        memcpy(&tmp[p->n_carry], w, k * 4u);
        uint32_t len = p->n_carry + k;
        uint32_t left = len - parse_words(p, tmp, len);
        if (k == n) {
            memcpy(p->carry, &tmp[len - left], left * 4u);
            p->n_carry = (uint8_t)left;
            flush(p);
            return;
        }
        w += k - left;
        n -= k - left;
    }
    uint32_t left = n - parse_words(p, w, n);
    memcpy(p->carry, &w[n - left], left * 4u);
    p->n_carry = (uint8_t)left;
    flush(p);
}

static bool current_flags(const spdif_rx_parser_t *p, uint8_t *f) {
    if (p->block_flags & BF_KNOWN) {
        *f = p->block_flags;
        return true;
    }
    if (p->cs_complete) {
        *f = flags_from_cs0(p->cs_last[0]);
        return true;
    }
    return false;
}

bool spdif_rx_parser_pcm(const spdif_rx_parser_t *p) {
    uint8_t f;
    return current_flags(p, &f) && !(f & BF_NONPCM);
}

bool spdif_rx_parser_emphasis(const spdif_rx_parser_t *p) {
    uint8_t f;
    return current_flags(p, &f) && (f & BF_EMPHASIS) && !(f & BF_NONPCM);
}

uint8_t spdif_rx_cs_word_bits(const uint8_t cs[SPDIF_RX_CS_BYTES]) {
    // Bits 33..35 as a number (bit 33 = 1): 20- / 24-bit maximum (bit 32).
    static const uint8_t max20[8] = {0, 16, 18, 0, 19, 20, 17, 0};
    static const uint8_t max24[8] = {0, 20, 22, 0, 23, 24, 21, 0};
    if (cs[0] & 0x01u)
        return 24;
    uint32_t v = (cs[4] >> 1) & 7u;
    uint8_t bits = (cs[4] & 0x01u) ? max24[v] : max20[v];
    return bits ? bits : 24;
}

int32_t spdif_rx_rate_ppm(uint64_t words, uint64_t us, uint32_t nominal) {
    // Expected subframes x 1e6 = 2 x us x nominal.
    int64_t expect = (int64_t)(2u * us * nominal);
    int64_t diff = (int64_t)(words * 1000000u) - expect;
    if (expect <= 0 || diff >= expect || diff <= -expect)
        return diff < 0 ? -1000000 : 1000000;
    // diff x 1e6 fits below 2^63 while |diff| < expect < 2^43.
    if (expect < (1ll << 43))
        return (int32_t)(diff * 1000000 / expect);
    return (int32_t)(diff / (expect / 1000000));
}

uint32_t spdif_rx_rate_classify(uint64_t words, uint64_t us, uint32_t tol_ppm) {
    for (unsigned i = 0; i < 5; i++) {
        int32_t ppm = spdif_rx_rate_ppm(words, us, spdif_rx_rates[i]);
        if (ppm <= (int32_t)tol_ppm && ppm >= -(int32_t)tol_ppm)
            return spdif_rx_rates[i];
    }
    return 0;
}
