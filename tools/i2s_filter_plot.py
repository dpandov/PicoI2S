"""Plots the dual-line I2S filters from the coefficients the firmware runs
(src/i2s/i2s_filter_coef.h, parsed, so the plots follow the committed
header): frequency response over the full range and zoomed around Fs/2,
impulse response and step response.

    python -m pip install numpy matplotlib
    python tools/i2s_filter_plot.py                     all filters at 44.1 kHz
    python tools/i2s_filter_plot.py --rate 96000 --filter sharp,minphase
    python tools/i2s_filter_plot.py --save filters.png  write a PNG instead of a window

The rate selects the oversampling as the firmware does (8x below 50 kHz,
4x above; NOS and sinc stay at the sample rate). By default the DAC's
zero-order hold at the output word rate is included, so the curves are
what leaves the DAC before its analogue filter; that is what makes NOS
and the sinc compensation comparable with the oversampling filters.
--no-hold shows the digital response alone. Above Fs/2 the frequency
plots show what an input tone at the alias frequency leaves as images.
The time axes are in input
samples from the input impulse or step, so the latency (group delay)
shows; responses are scaled to unity gain at DC (sinc: -4 dB by design).
"""
import argparse
import os
import re
import sys

import numpy as np

COEF = os.path.join(os.path.dirname(__file__), "..", "src", "i2s", "i2s_filter_coef.h")
FILTERS = ["sharp", "slow", "minphase", "sinc", "nos"]
COLORS = {"sharp": "tab:blue", "slow": "tab:green", "minphase": "tab:red", "sinc": "tab:purple",
          "nos": "tab:gray"}
# Slow and minimum phase have the same magnitude: dashed keeps both visible.
STYLES = {"minphase": (0, (4, 3))}


def read_coefficients(path):
    """Coefficient arrays of the header by name (A, B, ..., M0, M1, K), as floats."""
    text = open(path, encoding="utf-8").read()
    shift = int(re.search(r"#define I2S_COEF_SHIFT (\d+)", text).group(1))
    arrays = {}
    for name, body in re.findall(r"static const int32_t i2s_coef_(\w+)\[[^\]]*\] = \{([^}]*)\}", text):
        arrays[name.upper()] = np.array([int(v) for v in body.replace("\n", " ").split(",") if v.strip()],
                                        dtype=float) / 2.0 ** shift
    return arrays


def hb_taps(c):
    """Half-band: 2h at the odd offsets from the centre (1, 3, ...) -> full 2h, centre 1."""
    n = 4 * len(c) - 1
    mid = n // 2
    h = np.zeros(n)
    h[mid] = 1.0
    for k, v in enumerate(c):
        h[mid + 2 * k + 1] = h[mid - 2 * k - 1] = v
    return h


def poly_taps(b0, b1):
    """Two branches of 2h, oldest sample first -> full 2h."""
    h = np.zeros(len(b0) + len(b1))
    h[0::2] = b0[::-1]
    h[1::2] = b1[::-1]
    return h


def sym_taps(c):
    """m outer pairs, then the centre -> full linear-phase taps."""
    m = len(c) - 1
    return np.concatenate([c[:m], c[m:], c[:m][::-1]])


def cascade(name, rate, coef):
    """The firmware's stages for the filter at the rate: a list of
    (taps, interpolation factor 1 or 2); see i2s_filter_init()."""
    if name == "nos":
        return []
    if name == "sinc":
        return [(sym_taps(coef["K"]), 1)]
    high = rate > 50000
    if name == "sharp":
        names = ["H", "B"] if high else ["A", "B", "C"]
        return [(hb_taps(coef[n]), 2) for n in names]
    first = hb_taps(coef["S"]) if name == "slow" else poly_taps(coef["M0"], coef["M1"])
    rest = ["B"] if high else ["BS", "CS"]
    return [(first, 2)] + [(hb_taps(coef[n]), 2) for n in rest]


def oversampling(stages):
    return int(np.prod([factor for _, factor in stages])) if stages else 1


def run(stages, x):
    """The cascade on input x at the sample rate: zero-stuff and filter per
    x2 stage (2h taps keep unity gain), filter at the rate for x1."""
    for h, factor in stages:
        if factor == 2:
            up = np.zeros(2 * len(x))
            up[0::2] = x
            x = up
        x = np.convolve(x, h)
    return x


def response(stages, f, rate, hold):
    """Complex response at frequencies f (Hz): the stages' DTFTs at their own
    rates (x 1/2 for a x2 stage, which makes the zero-stuffing's image), the
    hold at the output rate when asked."""
    h_total = np.ones(len(f), complex)
    r = rate
    for h, factor in stages:
        r *= factor
        n = np.arange(len(h))
        h_total *= np.exp(-2j * np.pi * np.outer(f / r, n)) @ h / factor
    if hold:
        h_total *= np.sinc(f / r)
    return h_total


def group_delay(stages, rate):
    """Group delay in input samples at 1 kHz (digital part; the hold adds half
    an output sample)."""
    f = np.array([900.0, 1100.0])
    ph = np.unwrap(np.angle(response(stages, f, rate, False)))
    return -(ph[1] - ph[0]) / (2 * np.pi * (f[1] - f[0])) * rate


def db(h):
    return 20 * np.log10(np.maximum(np.abs(h), 1e-12))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--rate", type=int, default=44100, help="sample rate in Hz (default 44100)")
    ap.add_argument("--filter", default=",".join(FILTERS),
                    help="comma-separated: " + ", ".join(FILTERS) + " (default all)")
    ap.add_argument("--no-hold", action="store_true", help="leave out the DAC's zero-order hold")
    ap.add_argument("--fmax", type=float, default=0, help="full plot range in Hz (default 4 x rate)")
    ap.add_argument("--save", metavar="PNG", help="write the figure to a file instead of showing it")
    args = ap.parse_args()
    names = [n.strip() for n in args.filter.split(",") if n.strip()]
    bad = [n for n in names if n not in FILTERS]
    if bad or not names:
        sys.exit(f"unknown filter {', '.join(bad)}; choose from {', '.join(FILTERS)}")

    if args.save:
        import matplotlib
        matplotlib.use("Agg")
    else:
        # A TCL_LIBRARY left by another program (e.g. CSR BlueSuite) without
        # init.tcl stops Tk; without it Python finds its own Tcl.
        tcl = os.environ.get("TCL_LIBRARY")
        if tcl and not os.path.isfile(os.path.join(tcl, "init.tcl")):
            del os.environ["TCL_LIBRARY"]
    import matplotlib.pyplot as plt

    coef = read_coefficients(COEF)
    rate, hold = args.rate, not args.no_hold
    fmax = args.fmax or 4 * rate
    f_full = np.linspace(0, fmax, 8001)
    f_zoom = np.linspace(0.3 * rate, 0.7 * rate, 4001)
    f_pass = np.linspace(0, 0.5 * rate, 2001)

    fig, ax = plt.subplots(2, 2, figsize=(14, 9))
    a_full, a_zoom, a_imp, a_step = ax[0, 0], ax[0, 1], ax[1, 0], ax[1, 1]
    a_pass = a_zoom.inset_axes([0.07, 0.08, 0.42, 0.38])
    cascades = {name: cascade(name, rate, coef) for name in names}
    delays = {name: max(0.0, round(group_delay(st, rate), 2)) for name, st in cascades.items()}
    pre = 8
    n_in = pre + int(np.ceil(max(delays.values()))) + 40     # input samples shown
    print(f"{rate} Hz, {'with' if hold else 'without'} the DAC hold")
    for name in names:
        stages, gd = cascades[name], delays[name]
        osr = oversampling(stages)
        taps = sum(len(h) for h, _ in stages)
        label = f"{name} {osr}x ({gd:.1f} samples)" if osr > 1 else f"{name} ({gd:.1f} samples)"
        line = dict(color=COLORS[name], ls=STYLES.get(name, "-"))

        a_full.plot(f_full / 1e3, db(response(stages, f_full, rate, hold)), lw=1, label=label, **line)
        a_zoom.plot(f_zoom / 1e3, db(response(stages, f_zoom, rate, hold)), lw=1.2, **line)
        a_pass.plot(f_pass / 1e3, db(response(stages, f_pass, rate, hold)), lw=1, **line)

        # Impulse and step at the output rate, from `pre` input samples before.
        imp = np.zeros(n_in)
        imp[pre] = 1.0
        step = np.zeros(n_in)
        step[pre:] = 1.0
        t = np.arange(n_in * osr) / osr - pre
        style = dict(lw=1.1, drawstyle="steps-post" if hold else "default", **line)
        a_imp.plot(t, run(stages, imp)[:len(t)], **style)
        a_step.plot(t, run(stages, step)[:len(t)], **style)

        at = lambda fr: db(response(stages, np.array([fr]), rate, hold))[0]
        print(f"  {label:32s} {taps:3d} taps   20 kHz {at(20000):+7.2f} dB   Fs/2 {at(rate / 2):+8.2f} dB"
              f"   0.55 Fs {at(0.55 * rate):+8.1f} dB")

    nyq = rate / 2e3
    a_full.set(title=f"Frequency response, {rate / 1e3:g} kHz", xlabel="kHz", ylabel="dB",
               xlim=(0, fmax / 1e3), ylim=(-160, 10))
    for k in range(1, int(fmax / rate) + 1):
        a_full.axvline(k * rate / 1e3, color="k", lw=0.5, ls=":")
    a_zoom.set(title="Around Fs/2", xlabel="kHz", ylabel="dB", xlim=(0.3 * rate / 1e3, 0.7 * rate / 1e3),
               ylim=(-140, 5))
    a_pass.set(xlim=(0, nyq), ylim=(-6, 1))
    a_pass.set_title("passband", fontsize=8)
    a_pass.tick_params(labelsize=7)
    for a in (a_full, a_zoom, a_pass):
        a.axvline(nyq, color="k", lw=0.7, ls="--")
        a.axvline(20, color="k", lw=0.5, ls=":")
        a.grid(alpha=0.3)
    a_imp.set(title="Impulse response", xlabel="input samples", ylabel="amplitude",
              xlim=(-pre, n_in - pre))
    a_step.set(title="Step response", xlabel="input samples", ylabel="amplitude",
               xlim=(-pre, n_in - pre))
    for a in (a_imp, a_step):
        a.axvline(0, color="k", lw=0.5, ls=":")
        a.grid(alpha=0.3)
    fig.suptitle("Dual-line I2S filters" + (" with the DAC's zero-order hold" if hold else ", digital only")
                 + "; group delay in input samples")
    fig.legend(*a_full.get_legend_handles_labels(), loc="upper center", ncol=len(names), fontsize=9,
               bbox_to_anchor=(0.5, 0.965), frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    if args.save:
        fig.savefig(args.save, dpi=120)
        print("wrote", args.save)
    else:
        plt.show()


if __name__ == "__main__":
    main()
