"""Reference pitch for a real recording (pYIN), in the same per-sample format as synth.make_test_signal().

    sig = wav_reference.make_wav_signal("wav_files/introduce.wav")

A real recording has no exact F0 truth, so the reference is pYIN (librosa) run on the
recording as it is. pYIN is a different algorithm from the prototype's correlogram detector
(probabilistic YIN + HMM voicing), so agreement with it is a fair accuracy measure, but it is
an estimate, not truth.

Per-sample arrays (same keys as synth.make_test_signal):
  x          the recording, float in [-1, 1)
  f0         reference F0 in Hz, 0 where unvoiced
  intended   nearest MIDI note of the reference (what NEAREST mode should pick), -1 if unvoiced
  steady     inside a run of the same nearest note, >= 30 ms after the run starts and the run
             >= 60 ms long (speech glides, so the 0.1 s rule of the synthetic phrase is too strict)
  background no speech at all: short-term level within 6 dB of the recording's floor
             (used for false alarms instead of "f0 == 0", which would also count consonants)
"""
from __future__ import annotations

import numpy as np

from autotuner import hz_to_midi, load_wav

HOP = 128


def _short_term_db(x, win=512):
    p = np.convolve(x ** 2, np.ones(win) / win, mode="same")
    return 10 * np.log10(p + 1e-12)


def make_wav_signal(path, fs=16000, fmin=65.0, fmax=1000.0, f_min_valid=70.0,
                    speech_margin_db=6.0, steady_skip_s=0.03, steady_min_s=0.06):
    import librosa

    x = np.asarray(load_wav(path, fs), dtype=float)
    n = len(x)
    f0_fr, vflag, vprob = librosa.pyin(x, fmin=fmin, fmax=fmax, sr=fs, frame_length=1024,
                                       hop_length=HOP, center=True)
    centres = np.arange(len(f0_fr)) * HOP                      # center=True: frame t is centred at t*HOP

    # speech region: short-term level clearly above the recording's own background floor
    lvl = _short_term_db(x)
    floor = np.percentile(lvl, 20)
    speech = lvl > floor + speech_margin_db
    background = lvl < floor + speech_margin_db / 2            # leave a guard band between the two

    # per-sample voicing (nearest pYIN frame) and F0 (interpolated in log-frequency)
    idx = np.clip(np.round(np.arange(n) / HOP).astype(int), 0, len(f0_fr) - 1)
    v_fr = vflag & np.isfinite(f0_fr) & (np.nan_to_num(f0_fr) >= f_min_valid)
    voiced = v_fr[idx] & speech
    good = np.flatnonzero(v_fr)
    logf = np.interp(np.arange(n), centres[good], np.log2(f0_fr[good])) if len(good) else np.zeros(n)
    f0 = np.where(voiced, 2.0 ** logf, 0.0)

    intended = np.where(voiced, np.round(hz_to_midi(np.maximum(f0, 1.0))), -1).astype(int)
    steady = np.zeros(n, bool)
    edges = np.flatnonzero(np.diff(intended) != 0) + 1
    skip, min_len = int(steady_skip_s * fs), int(steady_min_s * fs)
    for a, b in zip(np.r_[0, edges], np.r_[edges, n]):
        if intended[a] >= 0 and b - a >= min_len:
            steady[a + skip:b] = True

    return dict(x=x, f0=f0, intended=intended, steady=steady, background=background & ~voiced,
                speech=speech, ref_frames=dict(f0=f0_fr, voiced=vflag, prob=vprob, centres=centres),
                floor_db=float(floor))


if __name__ == "__main__":
    import sys
    s = make_wav_signal(sys.argv[1] if len(sys.argv) > 1 else "wav_files/introduce.wav")
    fs = 16000
    v = s["f0"] > 0
    m = hz_to_midi(s["f0"][v])
    print(f"length {len(s['x']) / fs:.2f} s, voiced {v.sum() / fs:.2f} s, steady {s['steady'].sum() / fs:.2f} s, "
          f"background {s['background'].sum() / fs:.2f} s, floor {s['floor_db']:.1f} dBFS")
    print("reference MIDI percentiles 5/50/95:", np.round(np.percentile(m, [5, 50, 95]), 1))
