"""Pause-gate check on a real recording (report section 4.7).

    python gate_eval.py [wav_files/introduce.wav]

Runs the full pipeline with the old gate (gate_snr_db = 0) and the SNR gate (6 dB) and reports
  * noise only (white noise, no voice): share of time the gate is open
  * the recording as it is, and with white noise added at -30 / -20 dBFS:
      voice kept  = share of reference-voiced samples (pYIN) where the gate is open
      noise open  = share of the recording's background-only samples where the gate is open
"Open" = gate gain above -6 dB (the gate is either ~0 dB or -25 dB, so the threshold is not critical).

The SNR gate's floor tracker starts low and rises only 1 dB/s while the noise reads as voiced, so on a
7.5 s clip it is still settling. The "settled" column therefore also runs each case after 10 s of the
same background (the recording's own pre-speech room noise, tiled, plus the added white noise) and
scores only the recording part.
"""
import sys
from dataclasses import replace

import numpy as np

import autotuner
import wav_reference
from autotuner import Config, run

_trace = []
_orig_gate = autotuner.TDPSOLA._gate


def _gate(self, output, start, stop):
    gains = _orig_gate(self, np.ones(len(output)), start, stop)
    if self.config.gate and len(output):
        _trace.append((start, gains.copy()))
    return output * gains


autotuner.TDPSOLA._gate = _gate


def gate_open(x, config):
    """Per input sample: True where the gate gain is above -6 dB."""
    _trace.clear()
    run(x, config)
    g = np.zeros(len(x) + config.denoise_delay + 1)
    for start, gains in _trace:
        stop = min(start + len(gains), len(g))
        if start < len(g):
            g[start:stop] = gains[:stop - start]
    g = g[config.denoise_delay:config.denoise_delay + len(x)]   # shifter time = input time + denoiser delay
    return g > 10 ** (-6 / 20)


def white_at(n, dbfs, rng):
    return rng.standard_normal(n) * 10 ** (dbfs / 20)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "wav_files/introduce.wav"
    base = Config()
    ref = wav_reference.make_wav_signal(path, base.fs)
    x, voiced, bg = ref["x"], ref["f0"] > 0, ref["background"]
    rng = np.random.default_rng(7)
    configs = [("old gate", replace(base, gate_snr_db=0.0)), ("SNR gate 6 dB", base)]
    pre = int(10 * base.fs)
    first_speech = int(np.flatnonzero(ref["speech"])[0])
    room = x[:max(first_speech - base.fs // 10, base.fs // 2)]            # room noise before the first word
    room_pre = np.tile(room, pre // len(room) + 1)[:pre]
    rows = []
    nz = white_at(len(x) + pre, -20, rng)
    rows.append(("white noise -20 dBFS, no voice: time open",
                 [gate_open(nz[pre:], c).mean() for _, c in configs] + [gate_open(nz, base)[pre:].mean()]))
    for name, extra in [("recording as is", None), ("recording + white -30 dBFS", -30), ("recording + white -20 dBFS", -20)]:
        add = np.zeros(len(x) + pre) if extra is None else white_at(len(x) + pre, extra, np.random.default_rng(11))
        xx = x + add[pre:]
        opens = [gate_open(xx, c) for _, c in configs]
        settled = gate_open(np.concatenate([room_pre, x]) + add, base)[pre:]
        opens.append(settled)
        rows.append((f"{name}: voice kept", [o[voiced].mean() for o in opens]))
        rows.append((f"{name}: background open", [o[bg].mean() for o in opens]))
    print(f"| case | {configs[0][0]} | {configs[1][0]} | {configs[1][0]}, settled |\n|---|---|---|---|")
    for name, vals in rows:
        print(f"| {name} | " + " | ".join(f"{100 * v:.0f}%" for v in vals) + " |")


if __name__ == "__main__":
    main()
