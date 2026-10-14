"""Block-level checks of the prototype against numbers in the design document.

    python selftest.py

These are also the checks to repeat on the C port (same inputs, same thresholds).
"""
from dataclasses import replace

import numpy as np
from scipy import signal

import synth
from autotuner import (MODE_FIXED, MODE_NEAREST, ST_MODE_FIXED, ST_OUT_OF_RANGE, ST_VOICED, AutoTuner,
                       Config, NoteTracker, PitchDetector, RangeMonitor, design_bandpass, display_lines,
                       hz_to_midi, parse_note, run)

config = Config()
failures = []


def check(name, ok, detail=""):
    print(f"[{'PASS' if ok else 'FAIL'}] {name}  {detail}")
    if not ok:
        failures.append(name)


# 1. derived constants (design doc: everything follows from fs and N)
check("tau range 16..200", (config.tau_min, config.tau_max) == (16, 200), f"{config.tau_min}..{config.tau_max}")
check("hop deadline 8 ms", config.H / config.fs == 0.008)
check("N >= 2 periods of 80 Hz", config.N >= 2 * config.fs / config.f0_min, f"{2 * config.fs / config.f0_min:.0f} <= {config.N}")

# 2. band-pass: two biquads, -3 dB at the edges, hum rejection
sos = design_bandpass(config)
w, h = signal.sosfreqz(sos, worN=[50, 75, 300, 1000, 3000], fs=config.fs)
db = 20 * np.log10(np.abs(h))
check("band-pass is 2 biquads", sos.shape == (2, 6))
check("-3 dB at 75 Hz and 1 kHz", abs(db[1] + 3) < 0.5 and abs(db[3] + 3) < 0.5, f"{db[1]:.2f}, {db[3]:.2f} dB")
check("50 Hz mains attenuated > 8 dB", db[0] < -8, f"{db[0]:.1f} dB")
wf, hf = signal.sosfreqz(sos, worN=8192, fs=config.fs)
enbw = np.trapezoid(np.abs(hf) ** 2, wf)
print(f"       noise bandwidth of the filter: {enbw:.0f} Hz -> white-noise gain "
      f"{10 * np.log10(8000 / enbw):.1f} dB (design doc: 925 Hz, 9.4 dB)")

# 3. pitch detector accuracy over the full singing range, harmonic tones through the filter
det = PitchDetector(config)
worst = 0.0
for f0 in [82, 110, 165, 262, 440, 660, 880, 990]:
    t = np.arange(config.N + 2000) / config.fs
    x = sum(np.sin(2 * np.pi * k * f0 * t) / k for k in range(1, 8))
    xd = signal.sosfilt(sos, x)[-config.N:]
    r = det(xd)
    err = 1200 * np.log2(r[0] / f0)
    worst = max(worst, abs(err))
    if abs(err) > 5 or r[1] < 0.9:
        check(f"detector {f0} Hz", False, f"err {err:.1f} cents conf {r[1]:.2f}")
check("detector within 5 cents, 82..990 Hz", worst < 5, f"worst {worst:.2f} cents")

# 4. silence and noise are not pitch
rng = np.random.default_rng(0)
check("digital silence -> no pitch", det(np.zeros(config.N)) is None)
fa = 0
for _ in range(200):
    xd = signal.sosfilt(sos, rng.standard_normal(config.N + 1000))[-config.N:]
    r = det(xd)
    fa += int(r is not None and np.isfinite(r[0]) and r[1] > config.voicing_thresh)
check("white noise rarely voiced (<5%)", fa / 200 < 0.05, f"{fa / 2:.1f}% of 200 frames")

# 5. note tracker: hysteresis, scale snap
nt = NoteTracker(replace(config, median_len=1))
seq = [60.0] * 4 + [60.55] * 6 + [60.9] * 6 + [61.0] * 6      # creeping past the boundary
tg = [nt.update(m)[1] for m in seq]
check("hysteresis holds target at 0.55 st", set(tg[4:10]) == {60}, f"targets {sorted(set(tg[4:10]))}")
check("switches after > 35 cents past boundary", tg[-1] == 61, f"final {tg[-1]}")
nt = NoteTracker(replace(config, median_len=1, scale_pcs=(0, 2, 4, 5, 7, 9, 11)))
check("C major snaps C# -> C or D", nt.update(61.2)[1] in (60, 62))
nt = NoteTracker(config)
for m in [60.0, 60.0, 72.0, 60.0, 60.0]:                      # isolated octave jump
    out = nt.update(m)
check("median removes a one-frame octave jump", abs(out[0] - 60) < 0.01, f"m_s {out[0]:.2f}")

# 6. pipeline on a detuned sung phrase (clean): pitch correction and level
for tau, limit in [(0.01, 6.0), (0.05, 12.0)]:
    s = synth.make_test_signal(vibrato_cents=0)
    y, tr_in = run(s["x"], replace(config, retune_tau_s=tau))
    _, tr = run(s["x"], synthesize=False)
    _, tr_out = run(y, synthesize=False)
    L, N = len(s["f0"]), config.N
    ne = tr["n_end"]
    cs = np.concatenate([[0], np.cumsum(s["steady"])])
    inside = (ne <= L) & (ne - N >= 0)
    st = np.zeros(len(ne), bool)
    st[inside] = (cs[ne[inside]] - cs[ne[inside] - N]) == N
    inte = s["intended"][np.minimum(ne - N // 2, L - 1)]

    def mean_err(t):
        ok = st & t["voiced"]
        return np.abs(100 * (hz_to_midi(t["f0"][ok]) - inte[ok])).mean()

    e_in, e_out = mean_err(tr), mean_err(tr_out)
    check(f"retune {tau * 1000:.0f} ms: mean cents error to intended note drops",
          e_out < limit and e_out < e_in, f"{e_in:.1f} -> {e_out:.1f} cents")
lvl = y.std() / s["x"].std()
check("output level within 1 dB of input", abs(20 * np.log10(lvl)) < 1, f"{20 * np.log10(lvl):+.2f} dB")
check("no NaN/inf in output", bool(np.isfinite(y).all()))

# 7. streaming equals offline (process_hop in a loop is what the MCU does) and PSOLA history is bounded
tuner = AutoTuner(config, max_samples=len(s["x"]) + 4096)
xp = np.zeros(int(np.ceil((len(s["x"]) + 1000) / config.H)) * config.H)
xp[:len(s["x"])] = s["x"]
chunks = [tuner.process_hop(xp[i:i + config.H])[1] for i in range(0, len(xp), config.H)]
ys = np.concatenate(chunks)[:len(s["x"])]
check("streaming output == offline output", np.allclose(ys, run(s["x"], config)[0][: len(ys)], atol=1e-12))
check("PSOLA only touches the last 1024 samples", True, "(asserted inside TDPSOLA._ola on every grain)")

# 8. fixed-note mode (spec autotuner-target-mode-changes.md)
check("note parsing C6=84, F#3=54, Bb2=46", [parse_note(t) for t in ("C6", "F#3", "Bb2", "84")] == [84, 54, 46, 84])


def sustained(note, dur=1.6):
    mel = synth.make_melody([note], [dur], [0.0], vibrato_cents=0, jitter_cents=0)
    x = synth.synth_voice(mel["f0"], mel["env"], "a")
    return 0.1 * x / x.std()


def out_pitch(y, lo=0.5, hi=1.4):
    """Median detected MIDI pitch of y over [lo, hi) s (shifter check: detection on the output)."""
    _, t = run(y, synthesize=False)
    tc = (t["n_end"] - config.N // 2) / config.fs
    win = (tc >= lo) & (tc < hi)
    sel = win & t["voiced"]
    return (float(np.median(hz_to_midi(t["f0"][sel]))) if sel.any() else float("nan")), sel.sum() / win.sum()


def info(name, detail):
    print(f"[INFO] {name}  {detail}")


xa3 = sustained(57)                                             # A3 sung
# (target, expected output, out_of_range expected, pitch is checked)
for tgt, want, oor_want, strict in [(60, 60, False, True), (50, 50, False, True), (46, 46, False, True),
                                     (69, 69, False, True), (84, 69, True, True), (30, 45, True, True)]:
    y, tr = run(xa3, replace(config, mode=MODE_FIXED, target_midi=tgt))
    p, vfrac = out_pitch(y)
    v = tr["voiced"].astype(bool)
    late = v & ((tr["n_end"] - config.N // 2) / config.fs > 0.5)
    oor = bool(tr["oor"][late].all()) if oor_want else bool(tr["oor"].any())
    d_ok = np.allclose(tr["d"][late], np.clip(tgt - tr["m_s"][late], -12, 12), atol=1e-9)
    lvl = 20 * np.log10(y[8000:22000].std() / xa3[8000:22000].std())
    detail = f"d {want - 57:+d} st, output {p:.2f}, voiced {100 * vfrac:.0f}%, level {lvl:+.1f} dB"
    check(f"fixed A3 -> {tgt}: shift {want - 57:+d} st, out_of_range {'set' if oor_want else 'clear'}",
          d_ok and oor == oor_want and bool(np.isfinite(y).all()), f"flag {'set' if oor else 'clear'}")
    if strict:
        check(f"  output pitch reaches {want} within 15 cents", abs(p - want) * 100 < 15 and vfrac > 0.9, detail)
    else:
        info(f"  output pitch {p:.2f} (not checked)", detail)

for st in (0.0, 1.0):                                           # grain stretch must not move the pitch
    p, vfrac = out_pitch(run(xa3, replace(config, mode=MODE_FIXED, target_midi=50, grain_stretch=st))[0])
    check(f"grain_stretch {st}: A3 -> D3 still lands on 50", abs(p - 50) * 100 < 15 and vfrac > 0.9, f"output {p:.2f}")

y, tr = run(xa3, replace(config, mode=MODE_FIXED, target_midi=84))
v = tr["voiced"].astype(bool)
st = set(tr["status"][tr["oor"].astype(bool) & v])
check("status byte on voiced out-of-range hops = 0x07", st == {ST_OUT_OF_RANGE | ST_MODE_FIXED | ST_VOICED},
      f"{sorted(hex(x) for x in st)}")
check("status byte keeps out_of_range on unvoiced hops (0x03)",
      set(tr["status"][tr["oor"].astype(bool) & ~v]) <= {ST_OUT_OF_RANGE | ST_MODE_FIXED})

# out-of-range hysteresis and debounce
rm = RangeMonitor(config)
s1 = [rm.update(13.0) for _ in range(6)]
s2 = [rm.update(g) for g in [12.0] * 10 + [None] * 5]
s3 = [rm.update(11.0) for _ in range(6)]
check("out-of-range enters only after 6 hops above 12.5", s1 == [False] * 5 + [True])
check("stays set between 11.5 and 12.5 and on unvoiced hops", all(s2))
check("clears after 6 hops below 11.5", s3 == [True] * 5 + [False])
rm.reset()
check("12.2 st never sets the flag", not any(rm.update(12.2) for _ in range(20)))
bounce = [rm.update(g) for g in [13.0, 13.0, 13.0, 12.0] * 5]
check("debounce: interrupted runs do not set the flag", not any(bounce))

# commands from the laptop take effect at a hop boundary; nearest mode never flags
y, tr = run(xa3, config, commands=[(0.6, {"mode": MODE_FIXED, "target": "C6"}), (1.2, {"mode": MODE_NEAREST})])
t0 = tr["n_end"] - config.H
fixed = (t0 >= 0.6 * config.fs) & (t0 < 1.2 * config.fs)
check("mode command applied from the next hop", bool((tr["mode"][fixed] == MODE_FIXED).all()
                                                       and (tr["mode"][~fixed] == MODE_NEAREST).all()))
check("nearest mode clears out_of_range", not tr["oor"][t0 >= 1.2 * config.fs].any())
check("nearest mode |d| <= 0.5 st on a clean note", np.abs(tr["d"][~fixed & tr["voiced"].astype(bool)]).max() <= 0.5 + 1e-9)

# display text
info = dict(mode=MODE_FIXED, voiced=True, target=84, m_s=57.1, oor=True)
l1, l2 = display_lines(info)
check("OLED: 'Target: C6 [FIXED]' / 'Sung: A3 OUT OF RANGE'", "C6" in l1 and "[FIXED]" in l1 and "A3" in l2
      and "OUT OF RANGE" in l2 and len(l1) <= 21 and len(l2) <= 21, f"{l1!r} {l2!r}")
l1, l2 = display_lines(dict(info, target=60, oor=False))
check("OLED: '+3 st'", "+3 st" in l2, repr(l2))
l1, l2 = display_lines(dict(mode=MODE_NEAREST, voiced=True, target=50, m_s=49.68, oor=False))
check("OLED: nearest '-32 c'", "-32 c" in l2 and "[NEAREST]" in l1, f"{l1!r} {l2!r}")
l1, l2 = display_lines(info, width=16)
l3, l4 = display_lines(dict(mode=MODE_NEAREST, voiced=True, target=61, m_s=60.8, oor=False), width=16)
check("LCD1602 lines fit 16 chars", all(len(z) <= 16 for z in (l1, l2, l3, l4)), f"{l1!r} {l2!r} {l3!r} {l4!r}")

print()
print("ALL PASS" if not failures else f"{len(failures)} FAILED: {failures}")
raise SystemExit(1 if failures else 0)
