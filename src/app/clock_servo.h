// Clock servo for adaptive USB audio: steers the S/PDIF rate trim so the
// USB receive FIFO stays at its set point while the host sends at its own
// rate. Pure integer code, no SDK dependencies (host-testable).
//
// The FIFO level is time-averaged over windows of whole consumer chunks, so
// the sawtooth of 1 ms packets in and chunk-sized reads out cancels. Each
// window runs a PI loop on the level error, expressed as audio time, holding
// the level the stream started at (within two chunks of the set point). It
// starts fast (natural frequency 2 rad/s, critically damped) to catch the
// host rate within a second or two, then halves its bandwidth stage by stage
// down to 0.0625 rad/s (0.01 Hz), so that after settling the trim only
// wanders slowly. A large error drops back to a faster stage. The trim moves
// by at most 50 ppm per window. A resume on a still running output continues
// from the last locked rate estimate and stage; a level error beyond 1/32 of
// the set point in its first 10 s (the host rate changed) restarts the
// acquisition.
#pragma once

#include <stdbool.h>
#include <stdint.h>

// Output limit, ppb. Covers +/-1000 ppm host offset plus our PLL error.
#define CLOCK_SERVO_MAX_PPB 1500000

typedef struct {
    uint32_t sample_rate;
    uint32_t frame_bytes;       // bytes per stereo frame in the FIFO
    uint32_t setpoint_bytes;    // FIFO level to hold
    uint32_t chunk_frames;      // frames the consumer takes at a time
    int32_t  trim_ppb;          // initial trim, e.g. minus the known clock error
} clock_servo_params_t;

typedef struct {
    // Configuration.
    uint32_t sample_rate;
    uint32_t frame_bytes;
    uint32_t setpoint_bytes;
    uint32_t window_us;
    int64_t  ki_scale;          // stage-0 integral gain x window, Q16
    int64_t  setpoint_ns;
    int64_t  ref_max_ns;
    // Level averaging.
    bool     primed;
    uint32_t level;             // last FIFO level, bytes
    uint64_t t_prev;
    uint64_t win_end;
    uint64_t acc;               // bytes x us in the current window
    // Loop.
    int64_t  integ;             // integral term, ppb Q16
    int32_t  trim;              // output, ppb
    int32_t  err_ns;            // last window's level error, audio ns
    uint8_t  ref_n;             // windows averaged into ref_ns so far
    uint8_t  ref_len;           // windows to average
    int64_t  ref_sum;
    int64_t  ref_ns;            // error held as zero: the starting level
    int64_t  filt_ns;           // low-passed error against ref_ns
    uint8_t  stage;
    uint32_t stage_us;          // time in the current stage
    uint32_t calm_us;           // time the error has stayed in the lock band
    bool     locked;
    // Last locked window, for a resume.
    int64_t  lock_integ;
    uint8_t  lock_stage;
    uint32_t unlocked_us;       // time since, UINT32_MAX: never locked
    uint32_t verify_us;         // after a resume: time left to confirm the rate
} clock_servo_t;

// Resets the servo for a new stream; the first update primes it.
void clock_servo_start(clock_servo_t *s, const clock_servo_params_t *p);

// As clock_servo_start(), for a new stream on an output that kept running
// since the last one. If this servo was locked shortly before at the same
// rate, it continues from that rate estimate and stage, ignoring
// p->trim_ppb.
void clock_servo_resume(clock_servo_t *s, const clock_servo_params_t *p);

// Feeds the FIFO level, every main-loop iteration (it is integrated over
// time, so irregular calls are fine). Returns true when the trim changed.
bool clock_servo_update(clock_servo_t *s, uint32_t fifo_bytes, uint64_t now_us);

// Trim to apply, ppb, positive = faster output.
int32_t clock_servo_trim_ppb(const clock_servo_t *s);

// Rate acquired and the FIFO level near its set point.
bool clock_servo_locked(const clock_servo_t *s);

// Last window's FIFO level error in audio ns (positive = above set point).
int32_t clock_servo_error_ns(const clock_servo_t *s);
