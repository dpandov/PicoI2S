"""Designs the 50/15 us de-emphasis filters of the I2S / dual-line output and
writes src/i2s/i2s_deemph_coef.h (the header is committed; rerun after a
change).

    python -m pip install numpy scipy
    python tools/i2s_deemph_design.py

Target: the analogue curve H(s) = (1 + s 15 us) / (1 + s 50 us) (IEC 60908).
One biquad per sample rate, its magnitude fitted minimax in dB over 20 Hz ..
20 kHz (0.45 Fs at 32 kHz), with the DC gain fixed and the gain held at or
below 0 dB up to Fs/2. Coefficients b0, b1, b2, -a1, -a2 in Q30 for the
direct form I in src/i2s/i2s_deemph.h; b2 is set from the others so the
quantised DC gain just below 1 (see the output bound). The output is
rounded toward zero.

Checks on the quantised coefficients (the tool stops without writing the
header when one fails):
  - magnitude error at most 0.01 dB in the band, gain <= 0 dB to Fs/2
  - poles real and inside the unit circle, impulse response non-negative
  - |a1| + |a2| < 1: rounded toward zero, the state decays to exactly 0
    after the input does (no zero-input limit cycle)
  - accumulator: sum |c| x 2^31 below 2^63
  - output bound: with a non-negative impulse response the peak gain is the
    DC gain, and the margin it leaves below 2^31 covers the worst-case
    rounding error (under 1 LSB, times the L1 norm of 1/A), so 24-bit
    left-justified input never reaches the output clamp
Also printed: the phase deviation from the analogue curve, raw and after
removing the best-fitting constant delay (the digital fit leads it by a
few us, which is a pure time shift), and the rounding-noise gain.
"""
import os
import sys

import numpy as np
from scipy import optimize, signal

Q = 30
SCALE = 2 ** Q
T1, T2 = 50e-6, 15e-6
RATES = (32000, 44100, 48000, 88200, 96000)
MAG_TOL = 0.01
OUT = os.path.join(os.path.dirname(__file__), "..", "src", "i2s", "i2s_deemph_coef.h")


def target(f):
    s = 2j * np.pi * f
    return (1 + s * T2) / (1 + s * T1)


def band(fs, n):
    return np.geomspace(20.0, top(fs), n)


def top(fs):
    return 20000.0 if fs > 40000 else 0.45 * fs


def resp(b, a, f, fs):
    z = np.exp(-2j * np.pi * np.asarray(f) / fs)
    return (b[0] + b[1] * z + b[2] * z * z) / (a[0] + a[1] * z + a[2] * z * z)


def from_params(p):
    """(b0, b1, a1, a2) -> b, a with unity DC gain."""
    b0, b1, a1, a2 = p
    return [b0, b1, 1 + a1 + a2 - b0 - b1], [1.0, a1, a2]


def fit(fs):
    f = band(fs, 800)
    tdb = 20 * np.log10(np.abs(target(f)))
    fd = np.linspace(0, fs / 2, 2000)[1:]

    def err(p):
        return 20 * np.log10(np.abs(resp(*from_params(p), f, fs))) - tdb

    def gain(p):
        return 20 * np.log10(np.abs(resp(*from_params(p), fd, fs)))

    b, a = signal.bilinear([T2, 1], [T1, 1], fs)
    p = optimize.least_squares(err, [b[0], b[1], a[1], 0.0]).x
    cons = [{"type": "ineq", "fun": lambda x: x[4] - err(x[:4])},
            {"type": "ineq", "fun": lambda x: x[4] + err(x[:4])},
            {"type": "ineq", "fun": lambda x: -gain(x[:4])}]
    r = optimize.minimize(lambda x: x[4], np.r_[p, np.abs(err(p)).max()], constraints=cons,
                          method="SLSQP", options={"maxiter": 500, "ftol": 1e-12})
    return from_params(r.x[:4])


def quantise(b, a):
    """Q30 b0, b1, b2, -a1, -a2; b2 leaves the DC gain k units below 1, with
    the smallest k whose margin covers the rounding error."""
    na1, na2 = (int(round(-v * SCALE)) for v in a[1:])
    b0, b1 = (int(round(v * SCALE)) for v in b[:2])
    a_q = [1.0, -na1 / SCALE, -na2 / SCALE]
    g = signal.lfilter([1.0], a_q, np.r_[1.0, np.zeros(8191)])
    l1g = np.abs(g).sum()
    asum = SCALE - na1 - na2
    for k in range(0, 64):
        b2 = asum - b0 - b1 - k
        if k / asum * 2.0 ** 31 >= l1g + 0.5:
            return [b0, b1, b2, na1, na2], k, l1g, g
    sys.exit("no DC margin")


def check(fs, c, k, l1g, g):
    b = [v / SCALE for v in c[:3]]
    a = [1.0, -c[3] / SCALE, -c[4] / SCALE]
    f = band(fs, 20000)
    h = resp(b, a, f, fs)
    t = target(f)
    mag = 20 * np.log10(np.abs(h) / np.abs(t))
    ph = np.unwrap(np.angle(h / t))
    tau = -np.dot(ph, 2 * np.pi * f) / np.dot(2 * np.pi * f, 2 * np.pi * f)
    res = np.degrees(ph + 2 * np.pi * f * tau)
    fd = np.linspace(0, fs / 2, 50001)
    peak = 20 * np.log10(np.abs(resp(b, a, fd, fs)).max())
    poles = np.roots(a)
    imp = signal.lfilter(b, a, np.r_[1.0, np.zeros(8191)])
    pr, pp, _ = signal.residuez(b, a)
    dom = np.argmax(np.abs(pr))
    acc = sum(abs(v) for v in c) * 2 ** 31
    asum = SCALE - c[3] - c[4]
    dc = sum(c[:3]) / asum
    margin = (1 - dc) * 2.0 ** 31
    bound = l1g
    ok = (np.abs(mag).max() <= MAG_TOL and peak <= 0.0 and np.abs(poles).max() < 1
          and np.isreal(poles).all() and pr[dom].real > 0 and pp[dom].real > 0
          and imp.min() >= 0 and abs(c[3]) + abs(c[4]) < SCALE and acc < 2 ** 63 and margin > bound)
    print(f"  {fs:5d} Hz: {np.abs(mag).max():.4f} dB to {f[-1] / 1000:.1f} kHz, "
          f"phase {np.abs(np.degrees(ph)).max():.2f} deg ({-tau * 1e6:+.2f} us lead, "
          f"then {np.abs(res).max():.2f} deg), peak {peak:+.1e} dB, DC {20 * np.log10(dc):+.1e} dB (k {k}), "
          f"poles {poles.real.round(4).tolist()}, h min {imp.min():.1e}, "
          f"margin {margin:.1f} > {bound:.1f} LSB, rounding-noise gain {np.sum(g * g):.2f}")
    if not ok:
        sys.exit(f"{fs} Hz: spec not met")
    return np.abs(mag).max()


print("de-emphasis 50/15 us, one biquad per rate (errors vs the analogue curve, quantised):")
table = []
for fs in RATES:
    b, a = fit(fs)
    c, k, l1g, g = quantise(b, a)
    err = check(fs, c, k, l1g, g)
    table.append((fs, c, err))

with open(OUT, "w", newline="\n") as f:
    f.write("// De-emphasis (50/15 us) coefficients, generated by tools/i2s_deemph_design.py:\n"
            "// one biquad per rate, b0, b1, b2, -a1, -a2 in Q30 (i2s_deemph.h).\n"
            "#pragma once\n\n#include <stdint.h>\n\n#define I2S_DEEMPH_SHIFT 30\n"
            f"#define I2S_DEEMPH_RATES {len(RATES)}u\n\n"
            "static const uint32_t i2s_deemph_rate[I2S_DEEMPH_RATES] = {"
            + ", ".join(f"{fs}u" for fs in RATES) + "};\n\n"
            "static const int32_t i2s_deemph_coef[I2S_DEEMPH_RATES][5] = {\n")
    for fs, c, err in table:
        hi = top(fs) / 1000
        f.write("    {" + ", ".join(f"{v:11d}" for v in c) + f"}},  // {fs} Hz, {err:.4f} dB to {hi:.1f} kHz\n")
    f.write("};\n")
print("wrote", os.path.normpath(OUT))
