#include "app/clock_servo.h"

// Loop in audio time: e = level error in seconds of audio, trim = u, with
// de/dt = host rate / our rate - 1 - u. PI: u = KP e + KI integral(e), so
// wn^2 = KI and 2 zeta wn = KP. Stage k divides wn by 2^k: KP >> k, KI >> 2k.
// Stage 0: wn = 2 rad/s, zeta = 1. The error is low-passed with a time
// constant of 2^k windows first (about 0.1 / wn at every stage), against the
// window-to-window noise from main-loop stalls.
#define KP0              4          // 1/s
#define KI0              4          // 1/s^2
#define STAGE_LAST       5          // wn = 0.0625 rad/s
// Time in a stage before the next, slower one: three time constants.
#define HOLD0_US         1500000u
// A window is this many consumer chunks.
#define WINDOW_CHUNKS    24u
// Longer gaps between updates (debugger, stall) restart the averaging.
#define MAX_LATE_WINDOWS 4u
#define LOCK_CALM_US     1000000u
// Starting offset held rather than corrected.
#define REF_MAX_CHUNKS   2u
// Largest trim change per window (about 1000 ppm/s at 48 kHz): large level
// errors (lost packets, a host rate step) are corrected with a ramp.
#define MAX_STEP_PPB     50000
// A resume continues from the last lock if it was lost no longer ago than
// this (servo time: a drain before an underrun takes a window or two).
#define WARM_MAX_UNLOCKED_US 2000000u
// The host may have changed meanwhile: an error outside the calm band
// (setpoint / 32) this soon after a resume restarts the acquisition.
#define WARM_VERIFY_US   10000000u
// A resume holds its trim while it averages the starting level over this
// many windows: at the slow stages a main-loop stall in a single reference
// window would offset the trim by several ppm.
#define WARM_REF_WINDOWS 8u
// The lock, calm and stage bands are fractions of this (or of the set point
// if that is larger): independent of how little the FIFO holds.
#define BAND_MIN_NS      24000000

static int64_t shr_sym(int64_t x, unsigned n) {
    return x < 0 ? -((-x) >> n) : x >> n;
}

static uint32_t add_sat(uint32_t a, uint32_t b) {
    return a + b < a ? UINT32_MAX : a + b;
}

static int64_t clamp64(int64_t x, int64_t lim) {
    return x > lim ? lim : (x < -lim ? -lim : x);
}

void clock_servo_start(clock_servo_t *s, const clock_servo_params_t *p) {
    *s = (clock_servo_t){0};
    s->sample_rate = p->sample_rate;
    s->frame_bytes = p->frame_bytes;
    s->setpoint_bytes = p->setpoint_bytes;
    uint64_t win_frames = (uint64_t)WINDOW_CHUNKS * p->chunk_frames;
    s->window_us = (uint32_t)((win_frames * 1000000u + p->sample_rate / 2u) / p->sample_rate);
    s->ki_scale = ((int64_t)KI0 * s->window_us * 65536 + 500000) / 1000000;
    s->setpoint_ns = (int64_t)p->setpoint_bytes * 1000000000 /
                     ((int64_t)p->frame_bytes * p->sample_rate);
    s->band_ns = s->setpoint_ns > BAND_MIN_NS ? s->setpoint_ns : BAND_MIN_NS;
    // A raw error this large drops a stage at once: at the slow stages the
    // filtered error lags by seconds, and the FIFO holds only the set point.
    s->drop_ns = s->setpoint_ns / 4 < s->band_ns / 8 ? s->setpoint_ns / 4 : s->band_ns / 8;
    s->ref_max_ns = (int64_t)REF_MAX_CHUNKS * p->chunk_frames * 1000000000 / p->sample_rate;
    s->trim = (int32_t)clamp64(p->trim_ppb, CLOCK_SERVO_MAX_PPB);
    s->integ = (int64_t)s->trim * 65536;
    s->ref_len = 1;
    s->unlocked_us = UINT32_MAX;
}

void clock_servo_resume(clock_servo_t *s, const clock_servo_params_t *p) {
    clock_servo_t prev = *s;
    clock_servo_start(s, p);
    if (prev.sample_rate != p->sample_rate || prev.unlocked_us > WARM_MAX_UNLOCKED_US)
        return;
    // The integral is the rate estimate. The trim holds it while the new
    // reference is averaged, then the loop continues from zero error.
    s->integ = prev.lock_integ;
    s->trim = (int32_t)shr_sym(prev.lock_integ, 16);
    s->stage = prev.lock_stage;
    s->lock_integ = prev.lock_integ;
    s->lock_stage = prev.lock_stage;
    s->unlocked_us = prev.unlocked_us;
    s->ref_len = WARM_REF_WINDOWS;
    s->verify_us = WARM_VERIFY_US;
}

static int64_t prop_term(int64_t e_ns, unsigned stage) {
    return shr_sym(KP0 * e_ns, stage);
}

// e: filtered error against the reference; raw: this window's unfiltered one.
static void run_stage_logic(clock_servo_t *s, int64_t e, int64_t raw) {
    int64_t mag = e < 0 ? -e : e;
    int64_t raw_mag = raw < 0 ? -raw : raw;
    s->stage_us = add_sat(s->stage_us, s->window_us);
    if (s->stage < STAGE_LAST && s->stage_us >= (HOLD0_US << s->stage) &&
        mag < s->band_ns / 32) {
        // Bumpless: the proportional part given up moves into the integral.
        s->integ += (prop_term(e, s->stage) - prop_term(e, s->stage + 1u)) * 65536;
        s->stage++;
        s->stage_us = 0;
    } else if (s->stage > 0 && raw_mag > s->drop_ns && s->stage_us >= 2u * s->window_us) {
        s->stage--;
        s->stage_us = 0;
    }

    if (mag < s->band_ns / 32) {
        s->calm_us = add_sat(s->calm_us, s->window_us);
    } else {
        s->calm_us = 0;
    }
    if (mag > s->band_ns / 16) {
        s->locked = false;
    } else if (s->stage > 0 && s->calm_us >= LOCK_CALM_US) {
        s->locked = true;
    }
}

// One window: average level -> error -> PI.
static bool close_window(clock_servo_t *s) {
    int64_t avg_q8 = (int64_t)((s->acc * 256u + s->window_us / 2u) / s->window_us);
    s->acc = 0;
    int64_t e_q8 = avg_q8 - (int64_t)s->setpoint_bytes * 256;
    int64_t e = e_q8 * 1000000000 / ((int64_t)256 * s->frame_bytes * s->sample_rate);
    e = clamp64(e, 100000000);
    s->err_ns = (int32_t)e;

    // The pre-roll ends on an instantaneous level, which sits up to about a
    // chunk and a packet off the average. Hold the starting average instead
    // of pulling it in with the acquisition gains.
    if (s->ref_n < s->ref_len) {
        s->ref_sum += e;
        s->ref_n++;
        int64_t mean = s->ref_sum / s->ref_n;
        s->ref_ns = clamp64(mean, s->ref_max_ns);
        s->filt_ns = mean - s->ref_ns;
        return false;
    }
    int64_t raw = e - s->ref_ns;
    s->filt_ns += shr_sym(raw - s->filt_ns, s->stage);
    e = s->filt_ns;

    // Unfiltered: the slow stages' low-pass would delay this by seconds.
    if (s->verify_us) {
        if ((raw < 0 ? -raw : raw) > s->band_ns / 32) {
            // As a fresh start from the current trim, holding the level
            // reached (the next window is the new reference).
            s->verify_us = 0;
            s->unlocked_us = UINT32_MAX;
            s->ref_n = 0;
            s->ref_len = 1;
            s->ref_sum = 0;
            s->integ = (int64_t)s->trim * 65536;
            s->stage = 0;
            s->stage_us = 0;
            s->calm_us = 0;
            s->locked = false;
            return false;
        }
        s->verify_us = s->verify_us > s->window_us ? s->verify_us - s->window_us : 0;
    }

    run_stage_logic(s, e, raw);

    const int64_t lim = CLOCK_SERVO_MAX_PPB;
    // The stage change may have moved the integral out of range.
    int64_t i_prev = clamp64(s->integ, lim * 65536);
    int64_t p = prop_term(e, s->stage);
    int64_t di = shr_sym(e * s->ki_scale, 2u * s->stage);
    int64_t integ = clamp64(i_prev + di, lim * 65536);
    int64_t u = p + shr_sym(integ, 16);
    int64_t hi = s->trim + MAX_STEP_PPB;
    int64_t lo = s->trim - MAX_STEP_PPB;
    hi = hi < lim ? hi : lim;
    lo = lo > -lim ? lo : -lim;
    // Anti-windup: no integration further into a limit.
    if ((u > hi && di > 0) || (u < lo && di < 0)) {
        integ = i_prev;
        u = p + shr_sym(integ, 16);
    }
    s->integ = integ;
    if (s->locked) {
        s->lock_integ = integ;
        s->lock_stage = s->stage;
        s->unlocked_us = 0;
    } else {
        s->unlocked_us = add_sat(s->unlocked_us, s->window_us);
    }
    int32_t trim = (int32_t)(u > hi ? hi : (u < lo ? lo : u));
    if (trim == s->trim)
        return false;
    s->trim = trim;
    return true;
}

bool clock_servo_update(clock_servo_t *s, uint32_t fifo_bytes, uint64_t now_us) {
    if (!s->primed || now_us - s->t_prev > (uint64_t)MAX_LATE_WINDOWS * s->window_us) {
        s->primed = true;
        s->acc = 0;
        s->t_prev = now_us;
        s->win_end = now_us + s->window_us;
        s->level = fifo_bytes;
        return false;
    }
    // The level held since the last call; windows end exactly on time.
    bool changed = false;
    uint64_t t = s->t_prev;
    while (now_us >= s->win_end) {
        s->acc += (uint64_t)s->level * (s->win_end - t);
        t = s->win_end;
        s->win_end += s->window_us;
        changed |= close_window(s);
    }
    s->acc += (uint64_t)s->level * (now_us - t);
    s->t_prev = now_us;
    s->level = fifo_bytes;
    return changed;
}

int32_t clock_servo_trim_ppb(const clock_servo_t *s) {
    return s->trim;
}

bool clock_servo_locked(const clock_servo_t *s) {
    return s->locked;
}

int32_t clock_servo_error_ns(const clock_servo_t *s) {
    return s->err_ns;
}
