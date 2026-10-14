"""Synthetic sung voice with exact F0 ground truth, plus noise generators.

The voice is additive synthesis: harmonics of a known F0 contour shaped by
vowel formants (Lorentzian resonances) and a -6 dB/oct tilt, with vibrato,
portamento between notes, small jitter and a touch of breath noise.
Because the F0 contour is generated, the truth is exact (no pYIN needed).
"""
from __future__ import annotations

import numpy as np
from scipy import signal

FS = 16000

# (centre Hz, bandwidth Hz) for F1..F3
VOWELS = {
    "a": [(700, 110), (1220, 120), (2600, 160)],
    "e": [(530, 100), (1840, 130), (2480, 160)],
    "i": [(270, 80), (2290, 140), (3010, 180)],
    "o": [(570, 100), (840, 100), (2410, 160)],
    "u": [(300, 80), (870, 100), (2240, 160)],
}


def _formant_gain(f, formants):
    """Magnitude of cascaded 2nd-order resonators (unity gain at DC)."""
    g = np.ones_like(f)
    for F, B in formants:
        Q = F / B
        g /= np.sqrt((1.0 - (f / F) ** 2) ** 2 + (f / (F * Q)) ** 2)
    return g


def _lowpass_noise(n, fc, rng, fs=FS):
    sos = signal.butter(2, fc, "low", fs=fs, output="sos")
    y = signal.sosfilt(sos, rng.standard_normal(n))
    return y / (y.std() + 1e-12)


def synth_voice(f0, env, vowel="a", rng=None, breath_db=-35.0, fs=FS):
    """f0: Hz per sample (0 = rest); env: 0..1 amplitude per sample."""
    rng = rng or np.random.default_rng(0)
    f0 = np.asarray(f0, float)
    voiced = f0 > 0
    f0s = np.where(voiced, f0, 100.0)               # keep the phase running through rests
    phase = 2 * np.pi * np.cumsum(f0s) / fs
    formants = VOWELS[vowel]
    kmax = int(7000 / max(f0s[voiced].min() if voiced.any() else 100.0, 1.0))
    out = np.zeros(len(f0))
    for k in range(1, kmax + 1):
        fk = k * f0s
        g = np.where(fk < 7500, _formant_gain(fk, formants) / k, 0.0)
        out += g * np.sin(k * phase)
    out *= env * voiced
    act = out[env > 0.1]
    if len(act):
        breath = _lowpass_noise(len(f0), 3000, rng) * env * voiced
        out += breath * act.std() * 10 ** (breath_db / 20)
    return out


def make_melody(notes, dur, detune_cents, vibrato_hz=5.5, vibrato_cents=20.0,
                glide_s=0.06, jitter_cents=4.0, fs=FS, rng=None):
    """notes: MIDI ints or None (rest). Returns dict with per-sample truth.

    f0        true F0 in Hz (0 in rests)
    env       amplitude envelope
    intended  the note the singer meant (MIDI int, -1 in rests)
    steady    True away from note onsets (>0.1 s in), where 'intended' is unambiguous
    """
    rng = rng or np.random.default_rng(0)
    lens = [int(round(d * fs)) for d in dur]
    n = sum(lens)
    midi = np.zeros(n)
    intended = np.full(n, -1, int)
    steady = np.zeros(n, bool)
    env = np.zeros(n)
    pos, prev = 0, None
    for note, L, det in zip(notes, lens, detune_cents):
        sl = slice(pos, pos + L)
        if note is None:
            midi[sl] = np.nan
            prev = None
        else:
            seg = np.full(L, note + det / 100.0)
            if prev is not None:                        # portamento from the previous note
                g = min(int(glide_s * fs), L)
                ramp = 0.5 * (1 - np.cos(np.pi * np.arange(g) / g))
                seg[:g] = prev + (seg[:g] - prev) * ramp
            t = np.arange(L) / fs
            vib = vibrato_cents / 100.0 * np.sin(2 * np.pi * vibrato_hz * t) * np.clip((t - 0.1) / 0.2, 0, 1)
            midi[sl] = seg + vib
            intended[sl] = note
            steady[pos + int(0.1 * fs):pos + L] = True
            e = np.ones(L)
            a = min(int(0.02 * fs), L // 2)
            e[:a] = 0.5 * (1 - np.cos(np.pi * np.arange(a) / a))   # attack
            e[-a:] = e[-a:] * 0.5 * (1 + np.cos(np.pi * np.arange(a) / a))  # release
            env[sl] = e
            prev = note + det / 100.0
        pos += L
    jit = _lowpass_noise(n, 20, rng) * jitter_cents / 100.0
    f0 = np.where(np.isnan(midi), 0.0, 440.0 * 2 ** ((np.nan_to_num(midi) + jit - 69) / 12))
    # notes joined by portamento have no rest between them, so env stays 1 there
    return dict(f0=f0, env=env, intended=intended, steady=steady)


def make_test_signal(fs=FS, vowel="a", seed=0, vibrato_cents=20.0, detunes=None):
    """A sung phrase spanning ~A2 (110 Hz) to A5 (880 Hz), each note a bit off pitch."""
    rng = np.random.default_rng(seed)
    notes = [45, 52, 57, 64, 69, 76, 81, 76, 69, None, 57, 45, 52, None, 64, 69]
    dur = [0.45] * len(notes)
    for i, nt in enumerate(notes):
        if nt is None:
            dur[i] = 0.35
    det = detunes if detunes is not None else rng.uniform(-30, 30, len(notes))
    mel = make_melody(notes, dur, det, vibrato_cents=vibrato_cents, fs=fs, rng=rng)
    x = synth_voice(mel["f0"], mel["env"], vowel, rng, fs=fs)
    # equal loudness per note (a singer compensates for formant/pitch effects), -20 dBFS RMS
    ints = mel["intended"]
    edges = np.flatnonzero(np.diff(ints) != 0) + 1
    for a, b in zip(np.r_[0, edges], np.r_[edges, len(ints)]):
        if ints[a] >= 0:
            x[a:b] *= 0.1 / x[a:b].std()
    mel["x"] = x
    mel["detune"] = np.asarray(det)
    return mel


# ---------------------------------------------------------------- noise
def white(n, rng):
    return rng.standard_normal(n)


def pink(n, rng):
    X = np.fft.rfft(rng.standard_normal(n))
    f = np.fft.rfftfreq(n)
    f[0] = f[1]
    return np.fft.irfft(X / np.sqrt(f / f[1]), n)


def fan_hum(n, rng, fs=FS):
    """Low-frequency air-con rumble plus 50 Hz mains hum and harmonics (Thailand grid)."""
    t = np.arange(n) / fs
    rumble = _lowpass_noise(n, 400, rng, fs)
    hum = sum(a * np.sin(2 * np.pi * 50 * k * t + rng.uniform(0, 2 * np.pi))
              for k, a in [(1, .6), (2, .5), (3, .3), (5, .15)])
    return rumble + 0.7 * hum / hum.std()


def babble(n, rng, voices=4, fs=FS):
    """Several other talkers: in-band noise with its own pitch structure."""
    mix = np.zeros(n)
    for _ in range(voices):
        walk = _lowpass_noise(n, 3, rng, fs)
        f0 = rng.uniform(100, 250) * 2 ** (150 * walk / 1200)
        env = np.clip(_lowpass_noise(n, 4, rng, fs) * 0.8 + 0.5, 0, 1)
        v = synth_voice(f0, env, rng.choice(list(VOWELS)), rng, fs=fs)
        mix += v / (v[env > 0.1].std() + 1e-12)
    return mix


NOISES = {"white": white, "pink": pink, "fan_hum": fan_hum, "babble": babble}


def add_noise(clean, kind, snr_db, active, rng):
    """Mix at a fixed SNR (signal power measured over 'active' = singing samples).

    Returns (noisy, clean_scaled, noise_scaled); the last two have the common gain
    applied (only if clipping would occur), so SNR can be recomputed exactly.
    """
    noise = NOISES[kind](len(clean), rng)
    ps = np.mean(clean[active] ** 2)
    pn = np.mean(noise ** 2)
    noise = noise * np.sqrt(ps / (pn * 10 ** (snr_db / 10)))
    peak = np.abs(clean + noise).max()
    g = 0.98 / peak if peak > 0.98 else 1.0
    clean_s, noise_s = clean * g, noise * g
    return clean_s + noise_s, clean_s, noise_s
