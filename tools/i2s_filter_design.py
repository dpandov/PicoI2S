"""Designs the interpolation filters of the dual-line I2S output and writes
src/i2s/i2s_filter_coef.h (the header is committed; rerun after a change).

    python -m pip install numpy scipy
    python tools/i2s_filter_design.py

Filters (i2s_filter_type_t in src/i2s/i2s_tx.h), 8x up to 48 kHz, 4x above:
  sharp      linear phase: half-bands A, B, C (8x) or H, B (4x)
  slow       linear phase: half-band S, then BS, CS (8x) or B (4x)
  min-phase  slow magnitude, minimum phase: M (spectral factor of S^2),
             then BS, CS (8x) or B (4x)
  sinc       at the sample rate: K, zero-order-hold compensation, -4 dB

Stage kinds (src/i2s/i2s_filter.h), coefficients Q30:
  HB    half-band x2: 2h at the odd offsets 1, 3, 5, ... from the centre;
        the even outputs are the input samples
  POLY  any FIR x2: 2h of branch 0 (taps 0, 2, 4, ...) and branch 1 (taps 1,
        3, 5, ...), each stored oldest-sample first (reversed)
  SYM   linear-phase FIR x1, length 2m + 1: m outer pairs, then the centre

Checks: passband ripple, images of the passband at least 100 dB down (the
slow filters' own first-stage leakage excepted), and the gain bound the
1/16 input scaling in i2s_filter.h relies on (below 8).
"""
import os
import sys

import numpy as np
from scipy import signal

Q = 30
SCALE = 2 ** Q
ATT = 100.0
RIPPLE = 0.002
OUT = os.path.join(os.path.dirname(__file__), "..", "src", "i2s", "i2s_filter_coef.h")


def quant(x):
    return np.clip(np.round(np.asarray(x, dtype=float) * SCALE), -2**31, 2**31 - 1).astype(np.int64)


def db(h, f, fs):
    _, H = signal.freqz(h, worN=np.atleast_1d(f), fs=fs)
    return 20 * np.log10(np.maximum(np.abs(H), 1e-15))


def hb_taps(q):
    """One-sided 2h (ints) -> full-length 2h (float), centre 1."""
    m = len(q)
    n = 4 * m - 1
    c = n // 2
    h = np.zeros(n)
    h[c] = 1.0
    for k, v in enumerate(q):
        h[c + 2 * k + 1] = h[c - 2 * k - 1] = v / SCALE
    return h


def design_hb(fp, n_min):
    """Shortest equiripple half-band (pass 0..fp, stop 1-fp of the input
    rate) meeting ATT and RIPPLE after quantisation."""
    for n in range(n_min, 400, 4):
        h = signal.remez(n, [0, fp, 1 - fp, 1], [1, 0], fs=2.0)
        c = n // 2
        q = quant(2 * h[c + 1::2])
        h2 = hb_taps(q)
        stop = db(h2, np.linspace(1 - fp, 1, 4000), 2.0).max() - 6.0206
        ripple = np.ptp(db(h2, np.linspace(0, fp, 2000), 2.0))
        if stop <= -ATT and ripple <= RIPPLE:
            return q, h2
    sys.exit(f"no half-band for fp {fp}")


def design_minphase(h2_lin):
    """Minimum-phase factor of |H|^2 for a linear-phase 2x stage (full 2h)."""
    proto = np.convolve(h2_lin, h2_lin)
    m = signal.minimum_phase(proto, method="homomorphic", n_fft=1 << 18)
    m = m / m.sum() * 2.0
    q = quant(m)
    b0, b1 = q[0::2], q[1::2]
    return b0[::-1], b1[::-1], q / SCALE


def design_sinc(n, gain_db, f_edge):
    """Linear-phase FIR at the sample rate whose response times the
    zero-order hold sin(pi f) / (pi f) is flat at gain_db up to f_edge."""
    m = n // 2
    f = np.linspace(0, f_edge, 400)
    target = 10 ** (gain_db / 20) / np.sinc(f)
    a = np.column_stack([np.ones_like(f)] + [2 * np.cos(2 * np.pi * f * k) for k in range(1, m + 1)])
    coef, *_ = np.linalg.lstsq(a, target, rcond=None)
    h = np.concatenate([coef[:0:-1], coef])
    q = quant(h)
    return np.concatenate([q[:m], q[m:m + 1]]), q / SCALE


A, A2 = design_hb(0.4535, 131)
B, B2 = design_hb(0.4535 / 2, 23)
C, C2 = design_hb(0.4535 / 4, 15)
H, H2 = design_hb(0.35, 43)
S, S2 = design_hb(0.25, 23)
BS, BS2 = design_hb(0.375, 23)
CS, CS2 = design_hb(0.1875, 11)
M0, M1, M2 = design_minphase(S2)
K, K2 = design_sinc(15, -4.0, 0.4535)


def branch_bound(h2, kind):
    """Largest sum |coefficient| over the outputs of one stage."""
    if kind == "hb":
        return max(1.0, np.abs(h2[0::2]).sum())     # the odd outputs (taps at even indices)
    if kind == "poly":
        return max(np.abs(h2[0::2]).sum(), np.abs(h2[1::2]).sum())
    return np.abs(h2).sum()


def check(name, stages, osr, fprot, leak_ok):
    f = np.linspace(0, osr / 2, 200000)
    r = np.zeros_like(f)
    for h2, rate, kind in stages:
        r += db(h2, f, 2.0 * rate) - 6.0206
    img = np.zeros_like(f, bool)
    for k in range(1, osr + 1):
        if not (leak_ok and k == 1):
            img |= np.abs(f - k) <= fprot
    worst = r[img & (f > fprot)].max()
    ripple = np.ptp(r[f <= min(fprot, 0.4535)])
    at_edge = r[np.argmin(np.abs(f - 0.4535))]
    bound = np.prod([branch_bound(h2, kind) for h2, _, kind in stages])
    print(f"  {name}: ripple {ripple:.5f} dB to {min(fprot, 0.4535)} Fs, {at_edge:+.2f} dB at 0.4535 Fs, "
          f"images {worst:.1f} dB, gain bound {bound:.2f}")
    if worst > -ATT or bound >= 8.0:
        sys.exit(f"{name}: spec not met")


print("cascades:")
check("sharp 8x", [(A2, 1, "hb"), (B2, 2, "hb"), (C2, 4, "hb")], 8, 0.4535, False)
check("sharp 4x", [(H2, 1, "hb"), (B2, 2, "hb")], 4, 0.35, False)
check("slow 8x", [(S2, 1, "hb"), (BS2, 2, "hb"), (CS2, 4, "hb")], 8, 0.4535, True)
check("slow 4x", [(S2, 1, "hb"), (B2, 2, "hb")], 4, 0.4535, True)
check("min-phase 8x", [(M2, 1, "poly"), (BS2, 2, "hb"), (CS2, 4, "hb")], 8, 0.4535, True)
check("min-phase 4x", [(M2, 1, "poly"), (B2, 2, "hb")], 4, 0.4535, True)
ff = np.linspace(0, 0.4535, 2000)
_, hk = signal.freqz(K2, worN=ff, fs=1.0)
net = 20 * np.log10(np.abs(hk) * np.sinc(ff))
print(f"  sinc: {net.min():+.3f}..{net.max():+.3f} dB with the hold to 0.4535 Fs, gain bound {np.abs(K2).sum():.2f}")
_, gd = signal.group_delay((M2, [1]), w=np.array([0.02, 0.4]), fs=2.0)
print(f"  min-phase stage: {len(M2)} taps, group delay {gd[0] / 2:.2f} / {gd[1] / 2:.2f} input samples at 0.01 / 0.2 Fs")


def emit(f, name, arr, comment):
    f.write(f"// {comment}\n#define I2S_COEF_{name}_N {len(arr)}u\n")
    f.write(f"static const int32_t i2s_coef_{name.lower()}[I2S_COEF_{name}_N] = {{\n")
    for i in range(0, len(arr), 4):
        f.write("    " + ", ".join(f"{int(v):11d}" for v in arr[i:i + 4]) + ",\n")
    f.write("};\n\n")


with open(OUT, "w", newline="\n") as f:
    f.write("// Interpolation filter coefficients, generated by tools/i2s_filter_design.py\n"
            "// (stage kinds and layouts there and in i2s_filter.h). Q30.\n"
            "#pragma once\n\n#include <stdint.h>\n\n#define I2S_COEF_SHIFT 30\n\n")
    emit(f, "A", A, "Sharp, 2x at Fs: flat to 0.4535 Fs, 100 dB from 0.5465 Fs.")
    emit(f, "B", B, "2x at 2 Fs: flat to 0.2268 of its input rate.")
    emit(f, "C", C, "2x at 4 Fs: flat to 0.1134 of its input rate.")
    emit(f, "H", H, "Sharp, 2x at Fs for 88.2 / 96 kHz: flat to 0.35 Fs, 100 dB from 0.65 Fs.")
    emit(f, "S", S, "Slow, 2x at Fs: flat to 0.25 Fs, -2.8 dB at 0.4535 Fs, 100 dB from 0.75 Fs.")
    emit(f, "BS", BS, "2x at 2 Fs after S or M (8x): flat to 0.375 of its input rate.")
    emit(f, "CS", CS, "2x at 4 Fs after S or M (8x): flat to 0.1875 of its input rate.")
    emit(f, "M0", M0, "Minimum phase, 2x at Fs: the magnitude of S. Branch 0, oldest sample first.")
    emit(f, "M1", M1, "Branch 1, oldest sample first.")
    emit(f, "K", K, "Sinc compensation at Fs: flat at -4 dB with the DAC's hold to 0.4535 Fs.")
print("wrote", os.path.normpath(OUT))
