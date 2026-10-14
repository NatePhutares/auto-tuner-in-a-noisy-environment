"""Noise-robustness evaluation (design doc section 9, proposal section 4).

    python evaluate.py [--seeds 3] [--vowel a]

Mixes a synthetic sung phrase (exact F0 truth) with white / pink / fan+hum /
babble noise at fixed SNRs and compares detection with and without the
75 Hz-1 kHz band-pass.  Writes results/ (CSV, three PNG figures, markdown table).

Frame bookkeeping: hop i ends at n_end=(i+1)*H and covers [n_end-N, n_end).
  voiced frame  : truth F0 > 0 for the whole frame
  silent frame  : truth F0 == 0 for the whole frame (noise only)
  steady frame  : whole frame is >0.1 s into a note (intended note unambiguous)
Frames that straddle a boundary are excluded.
"""
import argparse
import csv
import os
from dataclasses import replace

import matplotlib
import numpy as np
from scipy import signal

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import synth
from autotuner import Config, design_bandpass, hz_to_midi, run

OUT = "results"
NOISES = ["white", "pink", "fan_hum", "babble"]
SNRS = [None, 20, 10, 0, -5]          # None = clean
BLUE, ORANGE = "#2a78d6", "#eb6834"   # categorical slots 1-2 (validated)
INK, MUTED, GRID, SURFACE = "#0b0b0b", "#898781", "#e1e0d9", "#fcfcfb"


def frame_masks(sig, tr, config):
    N = config.N
    L = len(sig["f0"])
    ne = tr["n_end"]
    inside = (ne <= L) & (ne - N >= 0)
    ne_c, st_c = np.minimum(ne, L), np.clip(ne - N, 0, L)
    cv = np.concatenate([[0], np.cumsum(sig["f0"] > 0)])
    cs = np.concatenate([[0], np.cumsum(sig["steady"])])
    nv, ns = cv[ne_c] - cv[st_c], cs[ne_c] - cs[st_c]
    centre = np.minimum(ne - N // 2, L - 1)
    return dict(voiced=inside & (nv == N), silent=inside & (nv == 0),
                steady=inside & (ns == N), centre=centre)


def metrics(sig, tr, config):
    m = frame_masks(sig, tr, config)
    det = tr["voiced"]
    f_true = sig["f0"][m["centre"]]
    with np.errstate(all="ignore"):
        err = 1200 * np.log2(tr["f0"] / f_true)
    vf = m["voiced"]
    ok = vf & det & (np.abs(err) <= 50)
    octv = vf & det & (np.abs(np.abs(err) - 1200) <= 50)
    gross = vf & det & (np.abs(err) > 50)
    note_ok = m["steady"] & det & (tr["target"] == sig["intended"][m["centre"]])
    return dict(
        rpa=ok.sum() / vf.sum(),
        mae_cents=float(np.abs(err[ok]).mean()) if ok.any() else np.nan,
        gross=gross.sum() / vf.sum(),
        octave=octv.sum() / vf.sum(),
        miss=(vf & ~det).sum() / vf.sum(),
        false_alarm=(m["silent"] & det).sum() / max(m["silent"].sum(), 1),
        note_acc=note_ok.sum() / max(m["steady"].sum(), 1),
    )


def snr_db(clean, noise, active, sos=None, skip=1600):
    if sos is not None:
        clean, noise = signal.sosfilt(sos, clean), signal.sosfilt(sos, noise)
    a = active.copy()
    a[:skip] = False
    return 10 * np.log10(np.mean(clean[a] ** 2) / np.mean(noise[skip:] ** 2))


def sweep(config, vowel, seeds):
    sig = synth.make_test_signal(config.fs, vowel=vowel)
    active = sig["f0"] > 0
    sos = design_bandpass(config)
    rows = []
    for kind in NOISES:
        for snr in SNRS:
            for seed in range(1 if snr is None else seeds):
                if snr is None:
                    x, cl, nz = sig["x"], sig["x"], np.zeros_like(sig["x"])
                else:
                    x, cl, nz = synth.add_noise(sig["x"], kind, snr, active, np.random.default_rng(100 + seed))
                for filt in (False, True):
                    _, tr = run(x, replace(config, use_filter=filt), synthesize=False)
                    r = dict(noise=kind, snr_in=snr if snr is not None else 99, seed=seed,
                             filter=int(filt), **metrics(sig, tr, config))
                    if snr is not None:
                        r["snr_in_meas"] = snr_db(cl, nz, active)
                        r["snr_out_meas"] = snr_db(cl, nz, active, sos)
                    rows.append(r)
        print(f"  {kind} done")
    return sig, rows


def aggregate(rows, key):
    out = {}
    for r in rows:
        out.setdefault((r["noise"], r["snr_in"], r["filter"]), []).append(r[key])
    return {k: float(np.nanmean(v)) for k, v in out.items()}


def style(ax):
    ax.set_facecolor(SURFACE)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=8)
    ax.grid(axis="y", color=GRID, lw=0.6)
    ax.set_axisbelow(True)


def fig_sweep(rows):
    levels = [s for s in SNRS if s is not None]
    xs = list(range(len(levels) + 1))
    labels = ["clean"] + [f"{s} dB" for s in levels]
    metrics_to_plot = [("rpa", "Raw pitch accuracy (±50 cents)", (0, 1.02)),
                       ("false_alarm", "Noise read as pitch (silent frames)", (0, 1.02)),
                       ("note_acc", "Correct target note (steady frames)", (0, 1.02))]
    fig, axes = plt.subplots(3, 4, figsize=(13, 8), sharex=True, facecolor=SURFACE)
    for ri, (key, title, ylim) in enumerate(metrics_to_plot):
        agg = aggregate(rows, key)
        for ci, kind in enumerate(NOISES):
            ax = axes[ri, ci]
            style(ax)
            for filt, col, ls, name in [(0, ORANGE, "--", "no filter"), (1, BLUE, "-", "band-pass")]:
                ys = [agg[(kind, 99, filt)]] + [agg[(kind, s, filt)] for s in levels]
                ax.plot(xs, ys, color=col, ls=ls, lw=2, marker="o", ms=5, mfc=SURFACE, mew=2,
                        label=name if (ri == 0 and ci == 0) else None)
            ax.set_ylim(*ylim)
            ax.set_xticks(xs, labels)
            if ri == 0:
                ax.set_title(kind.replace("_", " + "), color=INK, fontsize=11, loc="left")
            if ci == 0:
                ax.set_ylabel(title, color=INK, fontsize=9)
            if ri == 2:
                ax.set_xlabel("input SNR", color=MUTED, fontsize=8)
    axes[0, 0].legend(frameon=False, fontsize=8, loc="lower left")
    fig.suptitle("Pitch detection vs noise level: band-pass on the detection path vs none",
                 color=INK, fontsize=13, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(f"{OUT}/fig_sweep.png", dpi=150, facecolor=SURFACE)
    plt.close(fig)


def fig_snr_gain(rows, config):
    gains = {}
    for r in rows:
        if r["snr_in"] != 99:
            gains.setdefault(r["noise"], []).append(r["snr_out_meas"] - r["snr_in_meas"])
    sos = design_bandpass(config)
    wf, hf = signal.sosfreqz(sos, worN=8192, fs=config.fs)
    pred = 10 * np.log10(8000 / np.trapezoid(np.abs(hf) ** 2, wf))
    fig, ax = plt.subplots(figsize=(6.5, 3.6), facecolor=SURFACE)
    style(ax)
    vals = [np.mean(gains[k]) for k in NOISES]
    ax.bar(range(4), vals, color=BLUE, width=0.5)
    for i, v in enumerate(vals):
        ax.text(i, v + 0.25 if v >= 0 else 0.25, f"{v:+.1f} dB", ha="center", color=INK, fontsize=9)
    ax.axhline(pred, color=ORANGE, ls="--", lw=1.5)
    ax.text(3.45, pred + 0.2, f"white-noise prediction {pred:.1f} dB", color=INK, fontsize=8, ha="right")
    ax.axhline(0, color=GRID, lw=1)
    ax.set_xticks(range(4), [k.replace("_", " + ") for k in NOISES])
    ax.set_ylabel("SNR gain from filtering (dB)", color=INK, fontsize=9)
    ax.set_title("Filtering helps most where noise is out of band", color=INK, fontsize=11, loc="left")
    fig.tight_layout()
    fig.savefig(f"{OUT}/fig_snr_gain.png", dpi=150, facecolor=SURFACE)
    plt.close(fig)
    return {k: float(np.mean(gains[k])) for k in NOISES}, float(pred)


def fig_tracks(sig, config):
    active = sig["f0"] > 0
    cases = [("clean", sig["x"]), ("white noise, 0 dB", None), ("fan + hum, 0 dB", None)]
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True, facecolor=SURFACE)
    centre_t = None
    for ax, (name, x) in zip(axes, cases):
        style(ax)
        if x is None:
            kind = "white" if name.startswith("white") else "fan_hum"
            x, _, _ = synth.add_noise(sig["x"], kind, 0, active, np.random.default_rng(100))
        truth = np.where(sig["f0"] > 0, hz_to_midi(np.maximum(sig["f0"], 1)), np.nan)
        t = np.arange(len(truth)) / config.fs
        ax.plot(t, truth, color=MUTED, lw=1.2, label="truth")
        for filt, col, mk, lab in [(False, ORANGE, "x", "no filter"), (True, BLUE, "o", "band-pass")]:
            _, tr = run(x, replace(config, use_filter=filt), synthesize=False)
            tc = (tr["n_end"] - config.N // 2) / config.fs
            ax.plot(tc, np.where(tr["voiced"], hz_to_midi(tr["f0"]), np.nan), ls="none",
                    marker=mk, ms=3, color=col, label=lab, alpha=0.9)
        ax.set_ylabel("MIDI note", color=INK, fontsize=9)
        ax.set_title(name, color=INK, fontsize=10, loc="left")
        ax.set_ylim(40, 90)
    axes[0].legend(frameon=False, fontsize=8, ncol=3, loc="upper left")
    axes[-1].set_xlabel("time (s)", color=MUTED, fontsize=8)
    fig.tight_layout()
    fig.savefig(f"{OUT}/fig_tracks.png", dpi=150, facecolor=SURFACE)
    plt.close(fig)


def fig_spectra(sig, config):
    """Proposal 2.3: where voice and each noise live relative to the pass-band."""
    active = sig["f0"] > 0
    sos = design_bandpass(config)
    fig, ax = plt.subplots(figsize=(8, 4), facecolor=SURFACE)
    style(ax)
    f, p = signal.welch(sig["x"][active], config.fs, nperseg=2048)
    ax.semilogx(f[1:], 10 * np.log10(p[1:]), color=INK, lw=2, label="voice")
    cols = {"white": BLUE, "pink": ORANGE, "fan_hum": "#1baf7a", "babble": "#6250d6"}
    for kind in NOISES:
        _, _, nz = synth.add_noise(sig["x"], kind, 0, active, np.random.default_rng(100))
        f, p = signal.welch(nz, config.fs, nperseg=2048)
        ax.semilogx(f[1:], 10 * np.log10(p[1:]), color=cols[kind], lw=1.2, label=kind.replace("_", " + "))
    ax.set_ylim(-120, -30)
    ax.axvspan(config.f_lo, config.f_hi, color=GRID, alpha=0.5, lw=0)
    ax.text(config.f_lo * 1.1, -117, "detection pass-band", color=MUTED, fontsize=8)
    ax.set_xlabel("frequency (Hz)", color=MUTED, fontsize=8)
    ax.set_ylabel("PSD (dB/Hz), noise at 0 dB SNR", color=INK, fontsize=9)
    ax.legend(frameon=False, fontsize=8, ncol=5, loc="upper center", bbox_to_anchor=(0.5, 1.12))
    fig.tight_layout()
    fig.savefig(f"{OUT}/fig_spectra.png", dpi=150, facecolor=SURFACE)
    plt.close(fig)


def write_outputs(rows, gains, pred):
    with open(f"{OUT}/sweep.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[-1].keys()))
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in w.fieldnames})
    rpa, fa, na = aggregate(rows, "rpa"), aggregate(rows, "false_alarm"), aggregate(rows, "note_acc")
    mae, octv = aggregate(rows, "mae_cents"), aggregate(rows, "octave")
    lines = ["| noise | SNR | RPA no filter | RPA band-pass | false alarm nf / bp | note acc nf / bp | MAE cents bp | octave err nf / bp |",
             "|---|---|---|---|---|---|---|---|"]
    for kind in NOISES:
        for s in SNRS:
            if s is None and kind != "white":
                continue
            s_ = 99 if s is None else s
            k0, k1 = (kind, s_, 0), (kind, s_, 1)
            lines.append(f"| {kind if s is not None else 'clean'} | {'-' if s is None else s} | {rpa[k0]:.2f} | {rpa[k1]:.2f} | "
                         f"{fa[k0]:.2f} / {fa[k1]:.2f} | {na[k0]:.2f} / {na[k1]:.2f} | {mae[k1]:.1f} | {octv[k0]:.2f} / {octv[k1]:.2f} |")
    lines += ["", "SNR gain from the filter (measured, mean over levels): " +
              ", ".join(f"{k} {v:+.1f} dB" for k, v in gains.items()) + f"; white-noise prediction {pred:.1f} dB"]
    open(f"{OUT}/summary.md", "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--vowel", default="a")
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    config = Config()
    print("sweeping...")
    sig, rows = sweep(config, a.vowel, a.seeds)
    gains, pred = fig_snr_gain(rows, config)
    fig_sweep(rows)
    fig_tracks(sig, config)
    fig_spectra(sig, config)
    write_outputs(rows, gains, pred)


if __name__ == "__main__":
    main()
