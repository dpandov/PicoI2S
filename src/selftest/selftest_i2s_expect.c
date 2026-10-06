// Reference stream for the filtered formats, see selftest_i2s_expect.h.
//
// i2s_filter_run() gives the same output for any chunking of its input, so
// running it a few input frames at a time from zero history yields the
// transmitter's words wherever the window's history has filled every
// stage.

#include "selftest/selftest_i2s_expect.h"

#if PICO_ON_DEVICE
#include "pico.h"
#define EXPECT_FUNC(name) __not_in_flash_func(name)
#else
#define EXPECT_FUNC(name) name
#endif

// Interleaved stereo output of one step.
static int32_t step_out[2u * SELFTEST_EXPECT_STEP_OUT];

static unsigned coef_count(const i2s_stage_t *s) {
    return s->kind == I2S_STAGE_SYM ? s->n + 1u : s->n;
}

// Points stage i at a RAM copy of its coefficients (left in flash if a
// table were larger than the copy).
static void stage_to_ram(selftest_i2s_expect_t *x, unsigned i) {
    const i2s_stage_t *s = x->filter.st[i].stage;
    i2s_stage_t *d = &x->stage[i];
    unsigned n = coef_count(s);
    *d = *s;
    if (n <= sizeof(x->coef[i]) / sizeof(x->coef[i][0])) {
        for (unsigned k = 0; k < n; k++) {
            x->coef[i][k] = s->c[k];
        }
        d->c = x->coef[i];
    }
    if (s->kind == I2S_STAGE_POLY && s->n1 <= sizeof(x->coef1[i]) / sizeof(x->coef1[i][0])) {
        for (unsigned k = 0; k < s->n1; k++) {
            x->coef1[i][k] = s->c1[k];
        }
        d->c1 = x->coef1[i];
    }
    x->filter.st[i].stage = d;
}

// History zeroed, stages on their RAM copies.
static bool filter_init(selftest_i2s_expect_t *x) {
    if (!i2s_filter_init(&x->filter, x->type, x->rate)) {
        x->filter.osr = 0;
        x->filter.n_stages = 0;
        return false;
    }
    for (unsigned i = 0; i < x->filter.n_stages; i++) {
        stage_to_ram(x, i);
    }
    return true;
}

bool selftest_i2s_expect_setup(selftest_i2s_expect_t *x, i2s_filter_type_t type, uint32_t rate) {
    x->type = type;
    x->rate = rate;
    x->transient = 0;
    if (!filter_init(x)) {
        return false;
    }
    // A stage output is exact once the span - 1 inputs before its newest
    // one are: from factor x (exact input index + span - 1) on.
    uint32_t e = 0;
    for (unsigned i = 0; i < x->filter.n_stages; i++) {
        const i2s_stage_t *s = x->filter.st[i].stage;
        e = i2s_stage_factor(s) * (e + i2s_stage_span(s) - 1u);
    }
    x->transient = e + x->filter.osr;
    return true;
}

void selftest_i2s_expect_begin(selftest_i2s_expect_t *x, const int32_t *pcm, uint32_t n_in,
                               unsigned width, uint32_t *l, uint32_t *r, uint32_t max_out) {
    filter_init(x);
    x->pcm = pcm;
    x->n_in = n_in;
    x->pos = 0;
    x->l = l;
    x->r = r;
    x->max_out = max_out;
    x->n_out = 0;
    x->mask = 0xFFFFFFFFu << (32u - width);
}

bool EXPECT_FUNC(selftest_i2s_expect_step)(selftest_i2s_expect_t *x, uint32_t out_frames) {
    unsigned osr = x->filter.osr;
    if (osr == 0) {
        x->pos = x->n_in;               // not set up (NOS): nothing to compute
        return true;
    }
    if (out_frames > SELFTEST_EXPECT_STEP_OUT) {
        out_frames = SELFTEST_EXPECT_STEP_OUT;
    }
    uint32_t n = out_frames / osr;
    if (n == 0) {
        n = 1;
    }
    if (n > x->n_in - x->pos) {
        n = x->n_in - x->pos;
    }
    if (x->n_out + n * osr > x->max_out) {
        n = (x->max_out - x->n_out) / osr;
        if (n == 0) {
            x->pos = x->n_in;           // output full: stop
            return true;
        }
    }
    if (n) {
        i2s_filter_run(&x->filter, &x->pcm[2u * x->pos], n, step_out);
        // Saturated to full scale and truncated to the DAC word, as the
        // packer sees it.
        for (uint32_t i = 0; i < n * osr; i++) {
            x->l[x->n_out + i] = (uint32_t)i2s_filter_output(step_out[2u * i]) & x->mask;
            x->r[x->n_out + i] = (uint32_t)i2s_filter_output(step_out[2u * i + 1u]) & x->mask;
        }
        x->n_out += n * osr;
        x->pos += n;
    }
    return x->pos >= x->n_in;
}

// The filter is causal: the last outputs are as exact as any.
void selftest_i2s_expect_reference(const selftest_i2s_expect_t *x, selftest_i2s_reference_t *ref) {
    ref->l = x->l;
    ref->r = x->r;
    ref->n = x->n_out;
    ref->search_from = x->transient < x->n_out ? x->transient : x->n_out;
}
