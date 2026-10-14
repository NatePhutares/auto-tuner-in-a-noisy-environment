"""Auto-Tuner in a Noisy Environment -- Python reference prototype (v2).

v2 changes (13 Oct 2026, after the first real-mic test):
  * noise suppressor (STFT Wiener gain, minimum-statistics noise tracking) in front of everything.
    v1 only band-passed the *detection* path, so room noise went straight to the output, and the
    low-frequency noise made the voiced/unvoiced flag flicker (noise got pitch-shifted too).
  * pitch marks follow the previous grain by cross-correlation instead of peak-picking
    (on the real recording peak-picking jumped >10 % between consecutive periods 54 % of the time).
  * source-filter split: LPC residual is shifted, the vocal-tract filter 1/A(z) is applied afterwards,
    and when lowering the excitation grains are stretched (longer glottal pulses, formants unchanged).
    Plain TD-PSOLA left short pulses with gaps -> buzzy/robotic when lowering a lot.
  * pause gate after the shifter: a further -25 dB between phrases (hold 150 ms, 20 ms lookahead).
    Default: opens only on pitched (voiced) hops, and a hop holding > 90 % of its frame's energy counts as
    a click, so finger snaps, taps and handling noise in pauses are removed. --gate-level also opens it
    on loud unpitched sounds (keeps word-initial s/t/k, lets snaps through).
  * 14 Oct 2026, SNR gate: in the default pitch-only mode the gate also needs the hop to stand
    gate_snr_db (6 dB) above the tracked background floor. Steady noise (fan, white/pink/brown noise)
    scores ~0.55 pitch confidence after the band-pass + denoiser, so before this only the absolute
    -40 dBFS level floor kept it out, and loud noise opened the gate. The floor now rises fast on
    unvoiced hops (it catches up within ~1-2 s when the noise gets louder) and slowly on voiced
    hops (a long held note does not pull the floor up to itself). --gate-snr 0 gives the old gate.
    Measured (white/bright noise at -20 dBFS): gate open ~99 % -> ~30 % of the time; voice in a quiet
    room fully kept. Limit: when the noise is nearly as loud as the voice, the gate also closes on
    the voice (lower gate_snr_db keeps more voice, higher rejects more noise). Pink/brown noise is
    only slightly improved: its slow level swings look like voice to a single-mic gate.
  * cost: algorithmic latency 25 ms -> about 62 ms (24 ms denoiser + 38 ms shifter lookahead).
  * --stretch sets how much the excitation is stretched when lowering (default 0.135; 1.0 sounded
    like slow motion).
  --classic gives the v1 behaviour of the shifter (denoiser stays on unless --no-denoise).

Streaming, hop-by-hop implementation of the pipeline design document.
Everything is organised the way the C port on the STM32 will be: one call
to AutoTuner.process_hop() per 128-sample hop, all state held in the object.

    mic -> framing (N=512, H=128)
              |-> detection path: 75 Hz-1 kHz band-pass (2 biquads)
              |       -> FFT autocorrelation (1024-pt, zero padded)
              |       -> pitch + confidence (lag 16..200)
              |       -> note mapping (MIDI, median, hysteresis)
              |       -> tuning mode -> shift (semitones), clamp, out-of-range flag
              |-> audio path: wideband voice (75 Hz high-pass only)
                      -> LPC split: residual e = A(z) x  -> TD-PSOLA on e (marks by cross-correlation)
                      -> 1/A(z) synthesis (pitch_ratio = 2^(shift/12) from the detection path)
    (a noise suppressor sits in front of both paths)

Tuning modes (spec: autotuner-target-mode-changes.md):
    MODE_NEAREST  shift = nearest (allowed) note - sung MIDI pitch   about +-0.5 st, default / evaluation baseline
    MODE_FIXED    shift = fixed target - sung MIDI pitch, clamped to +-12   out_of_range flag with hysteresis + debounce

Output alignment: run() returns output with output[t] ~ input[t]; the real-time system
delivers it `Config.delay` samples (25 ms) late, because PSOLA needs lookahead.

Usage:
    python autotuner.py in.wav -o out.wav [--scale major --key C] [--retune 0.05]
    python autotuner.py in.wav -o out.wav --mode fixed --target C4
    python autotuner.py in.wav -o out.wav --plot pitch.png --play airpods
    python autotuner.py --list-devices      # audio output devices for --play
    python autotuner.py --demo               # synthetic voice, writes demo_*.wav
    python autotuner.py --demo --golden golden   # also dump vectors for the C check
"""
from __future__ import annotations

import argparse
import os
from dataclasses import dataclass, replace

import numpy as np
from scipy import signal

NOTE_NAMES = ["C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"]
SCALES = {"chromatic": None, "major": (0, 2, 4, 5, 7, 9, 11), "minor": (0, 2, 3, 5, 7, 8, 10)}

MODE_NEAREST, MODE_FIXED = 0, 1
MODE_NAMES = {MODE_NEAREST: "NEAREST", MODE_FIXED: "FIXED"}
TARGET_MIN, TARGET_MAX = 24, 108       # accepted range for a fixed target (C1..C8), as on the MCU

# status byte in the MCU -> laptop packet
ST_OUT_OF_RANGE, ST_MODE_FIXED, ST_VOICED = 0x01, 0x02, 0x04


@dataclass
class Config:
    # --- framing (design doc: everything derives from fs and N) ---
    fs: int = 16000
    N: int = 512          # frame length (32 ms)
    H: int = 128          # hop (8 ms) = real-time deadline
    fft_size: int = 1024      # zero-pad N -> 2N, avoids circular autocorrelation
    # --- singing range anchor ---
    f0_min: float = 80.0
    f0_max: float = 1000.0
    # --- detection path filter ---
    use_filter: bool = True       # False = baseline "no filtering" comparison
    f_lo: float = 75.0
    f_hi: float = 1000.0
    # --- noise suppression (in front of everything: detection and audio both see the cleaned signal) ---
    denoise: bool = True
    dn_floor_db: float = -25.0    # deepest attenuation per bin (a floor avoids "underwater" artefacts)
    dn_bias: float = 2.5          # minimum-statistics bias: the tracked minimum under-estimates the mean noise power
    dn_rise_db_s: float = 6.0     # how fast the noise estimate may rise (noise that gets louder, in dB per second)
    dn_smooth: float = 0.8        # time smoothing of the power spectrum used for the noise tracker
    dn_beta: float = 0.92         # decision-directed a-priori SNR smoothing (higher = less musical noise)
    # --- pause gate (after the shifter): turns the remaining noise down between phrases ---
    gate: bool = True
    gate_depth_db: float = -25.0   # extra attenuation while nobody is singing
    gate_open_db: float = 9.0      # level mode: open when the cleaned hop level is this far above the tracked noise floor
    gate_snr_db: float = 6.0       # pitch-only mode: a voiced hop opens the gate only if it is also this far above
                                   # the tracked floor (0 = off, the old gate: steady loud noise could open it)
    gate_floor_rise_db_s: float = 10.0        # floor tracker rise rate on unvoiced hops (SNR gate on)
    gate_floor_rise_voiced_db_s: float = 1.0  # ... on voiced hops, so a held note does not become the "floor"
    gate_pitch_only: bool = True   # open only on voiced (pitched) hops: removes snaps/claps/knocks in pauses,
                                   # but can clip unvoiced consonants at word starts (s, t, k, sh)
    gate_transient_share: float = 0.9  # pitch-only gate: a hop holding > 90 % of the frame energy is a click
    gate_hold_s: float = 0.15      # stay open this long after the last active hop (word endings, breaths)
    gate_lookahead_s: float = 0.02 # open this much before an onset (uses the shifter's existing delay)
    gate_attack_s: float = 0.005
    gate_release_s: float = 0.06
    # --- audio path ---
    audio_hpf_hz: float | None = 75.0
    # --- pitch / voicing ---
    voicing_thresh: float = 0.5   # peak r(tau)/r(0)
    rms_floor_dbfs: float = -40.0
    peak_rel: float = 0.9         # take the shortest-lag peak >= 0.9 * best peak
    # --- notes ---
    median_len: int = 5
    hyst_cents: float = 35.0      # past the note boundary by this much ...
    hyst_frames: int = 3          # ... for this many consecutive frames
    scale_pcs: tuple | None = None  # allowed pitch classes; None = chromatic
    # --- source-filter split (LPC): shift the excitation, keep the vocal-tract filter ---
    source_filter: bool = True    # False = plain TD-PSOLA on the waveform (old)
    lpc_order: int = 16
    lpc_lag_hz: float = 40.0      # Gaussian lag window (smooths the spectral envelope, keeps the filter stable)
    lpc_block: int = 32           # synthesis filter coefficients are refreshed every this many samples
    residual_stretch: float = 0.135 # lowering, LPC mode: stretch excitation grains by (1/alpha)^this.
                                  # 0 = short clicks with gaps (buzzy/robotic when lowering a lot),
                                  # 1 = pulses stretched like slowed-down tape (dull, "slow-motion" sound).
                                  # Formants are not affected either way. 0.135 -> x1.10 at -12 st.
    max_stretch: float = 1.1      # longest grain = max_stretch * tau_max (sets the extra lookahead / delay)
    # --- pitch marks (TD-PSOLA grain centres) ---
    mark_mode: str = "xcorr"      # "xcorr": follow the previous grain's waveform (steady marks);
                                  # "peak": tallest sample in a +-30 % window (old; jumps between formant peaks)
    mark_tol: float = 0.15        # xcorr: search +-this fraction of the period around the predicted mark
    # --- correction ---
    retune_tau_s: float = 0.05    # 0 = instant snap ("T-Pain" effect)
    delay: int = 400              # PSOLA lookahead / output delay (25 ms)
    ola_norm: str = "alpha"       # "alpha": grains scaled by 1/alpha (spec, MCU, smooth when lowering)
                                  # "w": divide by the window sum (old; buzzy when lowering, kept for A/B)
    grain_stretch: float = 0.2    # lowering only: stretch grains by (1/alpha)^this. 0 = plain TD-PSOLA
                                  # (gaps between pulses, robotic at big shifts), 1 = no gaps but formants
                                  # drop with the pitch (deep voice). See TDPSOLA._ola.
    dry_unvoiced: bool = True     # pass consonants/breath through unshifted (they have no pitch)
    dry_after_hops: int = 3       # ... after this many unvoiced hops in a row (24 ms)
    dry_fade: int = 160           # crossfade length in samples (10 ms)
    level_match: bool = True      # slow output gain so the output level follows the input level
    level_tau_s: float = 0.2      # averaging time of the level estimate (no pumping on syllables)
    level_max_db: float = 12.0    # gain limit, +-dB
    # --- tuning mode ---
    mode: int = MODE_NEAREST
    target_midi: int = 72         # fixed-mode target (C5); changed at run time with set_target()
    max_shift: float = 12.0       # |d| clamp in semitones (TD-PSOLA quality limit)
    oor_margin: float = 0.5       # out-of-range hysteresis: enter > 12.5 st, leave < 11.5 st
    oor_frames: int = 6           # out-of-range debounce, consecutive voiced hops (~48 ms)

    @property
    def denoise_delay(self) -> int:
        """Latency of the noise suppressor in samples (N - H = 24 ms), 0 if it is off."""
        return self.N - self.H if self.denoise else 0

    @property
    def psola_delay(self) -> int:
        """Output delay of the shifter in samples. With the LPC split the residual of a hop is only known
        (N - H) / 2 = 192 samples behind the newest input, so the lookahead grows by that much."""
        return (self.delay + self.grain_reach - self.tau_max
                + ((self.N - self.H) // 2 if self.source_filter else 0))

    @property
    def grain_reach(self) -> int:
        """Largest synthesis grain half-width = how far ahead of the finalised output marks are placed."""
        return int(round(self.tau_max * (self.max_stretch if self.source_filter else 1.0)))

    @property
    def tau_min(self) -> int:
        return int(round(self.fs / self.f0_max))   # 16

    @property
    def tau_max(self) -> int:
        return int(round(self.fs / self.f0_min))   # 200


def hz_to_midi(frequency_hz):
    return 69.0 + 12.0 * np.log2(frequency_hz / 440.0)


def midi_to_hz(midi_pitch):
    return 440.0 * 2.0 ** ((midi_pitch - 69.0) / 12.0)


def note_name(midi_note: int) -> str:
    return f"{NOTE_NAMES[midi_note % 12]}{midi_note // 12 - 1}"


def parse_note(text) -> int:
    """'C6', 'F#3', 'Bb2' or a MIDI number ('84') -> MIDI int."""
    stripped_text = str(text).strip()
    if stripped_text.lstrip("-").isdigit():
        return int(stripped_text)
    flat_to_sharp = {"Db": "C#", "Eb": "D#", "Gb": "F#", "Ab": "G#", "Bb": "A#"}
    accidental = stripped_text[1] if len(stripped_text) > 1 and stripped_text[1] in "#b" else ""
    name, octave = stripped_text[:1].upper() + accidental, stripped_text[1 + len(accidental):]
    name = flat_to_sharp.get(name, name)
    if name not in NOTE_NAMES or not octave.lstrip("-").isdigit():
        raise ValueError(f"not a note: {text!r} (use e.g. C6, F#3 or a MIDI number)")
    return NOTE_NAMES.index(name) + 12 * (int(octave) + 1)


def design_bandpass(config: Config) -> np.ndarray:
    """4th-order Butterworth band-pass as 2 cascaded biquads (shape (2, 6))."""
    return signal.butter(2, [config.f_lo, config.f_hi], btype="bandpass", fs=config.fs, output="sos")


# --------------------------------------------------------------------------
# LPC helpers (vocal-tract envelope)
# --------------------------------------------------------------------------
def levinson(autocorr, order: int):
    """Levinson-Durbin. Returns (a, k): A(z) = 1 + sum a[i] z^-i and the reflection coefficients k[0..order-1]."""
    a = np.zeros(order + 1)
    a[0] = 1.0
    reflection = np.zeros(order)
    error = autocorr[0]
    for m in range(1, order + 1):
        acc = autocorr[m] + np.dot(a[1:m], autocorr[m - 1:0:-1])
        k_m = -acc / error
        reflection[m - 1] = k_m
        previous = a.copy()
        a[1:m] = previous[1:m] + k_m * previous[m - 1:0:-1]
        a[m] = k_m
        error *= (1.0 - k_m * k_m)
        if error <= 0:
            break
    return a, reflection


def step_up(reflection):
    """Reflection coefficients -> A(z) coefficients (stable for |k| < 1, also after interpolation)."""
    order = len(reflection)
    a = np.zeros(order + 1)
    a[0] = 1.0
    for m in range(1, order + 1):
        previous = a.copy()
        a[1:m] = previous[1:m] + reflection[m - 1] * previous[m - 1:0:-1]
        a[m] = reflection[m - 1]
    return a


# --------------------------------------------------------------------------
# Pitch detection: FFT autocorrelation (Wiener-Khinchin)
# --------------------------------------------------------------------------
class PitchDetector:
    def __init__(self, config: Config):
        self.config = config
        self.window = np.hanning(config.N)
        window_spectrum = np.fft.rfft(self.window, config.fft_size)
        full_window_autocorrelation = np.fft.irfft(np.abs(window_spectrum) ** 2, config.fft_size)
        # window's own autocorrelation; +2 keeps lag tau_max + 1 for the parabolic interpolation
        self.window_autocorrelation = full_window_autocorrelation[: config.tau_max + 2]

    def __call__(self, frame):
        """Return (f0_hz, confidence, tau_samples) or None if the frame is silent."""
        config = self.config
        frame_spectrum = np.fft.rfft(frame * self.window, config.fft_size)
        autocorrelation = np.fft.irfft(np.abs(frame_spectrum) ** 2, config.fft_size)[: config.tau_max + 2]
        autocorrelation = autocorrelation / self.window_autocorrelation   # divide out the window's taper
        if autocorrelation[0] <= 1e-12:
            return None
        normalized_autocorrelation = autocorrelation / autocorrelation[0]   # r(tau)/r(0)
        lags = np.arange(config.tau_min, config.tau_max + 1)
        left, center, right = (normalized_autocorrelation[lags - 1], normalized_autocorrelation[lags],
                               normalized_autocorrelation[lags + 1])
        is_peak = (center > left) & (center >= right) & (center > 0)
        if not is_peak.any():
            best = normalized_autocorrelation[config.tau_min:config.tau_max + 1].max()
            return float("nan"), float(max(best, 0.0)), float("nan")
        candidate_lags = lags[is_peak]
        peak_values = normalized_autocorrelation[candidate_lags]
        # Octave-error guard: shortest-lag peak that is nearly as tall as the best.
        tau0 = int(candidate_lags[np.argmax(peak_values >= config.peak_rel * peak_values.max())])
        left, center, right = (normalized_autocorrelation[tau0 - 1], normalized_autocorrelation[tau0],
                               normalized_autocorrelation[tau0 + 1])
        denominator = left - 2.0 * center + right
        delta = 0.5 * (left - right) / denominator if denominator < -1e-12 else 0.0   # parabolic interpolation
        delta = float(np.clip(delta, -1.0, 1.0))
        tau = tau0 + delta
        confidence = float(center - 0.25 * (left - right) * delta)
        return config.fs / tau, confidence, tau


# --------------------------------------------------------------------------
# Note tracking: median smoothing, scale snapping, hysteresis
# --------------------------------------------------------------------------
class NoteTracker:
    def __init__(self, config: Config):
        self.config = config
        self.pitch_history: list[float] = []
        self.unvoiced_count = 0
        self.target_note: int | None = None
        self.smoothed_midi: float | None = None
        self.candidate_count = 0

    def nearest_allowed(self, midi_pitch: float) -> int:
        allowed_pitch_classes = self.config.scale_pcs
        if allowed_pitch_classes is None:
            return int(np.floor(midi_pitch + 0.5))
        base_note = int(np.floor(midi_pitch))
        best_note, best_distance = base_note, 1e9
        for note in range(base_note - 3, base_note + 5):
            if note % 12 in allowed_pitch_classes and abs(midi_pitch - note) < best_distance:
                best_note, best_distance = note, abs(midi_pitch - note)
        return best_note

    def update(self, raw_midi: float | None):
        """raw_midi: MIDI pitch of a voiced frame, or None. Returns (smoothed_midi, target_note)."""
        config = self.config
        if raw_midi is None:
            self.unvoiced_count += 1
            if self.unvoiced_count >= 3:
                self.pitch_history.clear()          # don't mix pre- and post-gap pitches
            self.candidate_count = 0
            return self.smoothed_midi, self.target_note   # hold last valid note
        self.unvoiced_count = 0
        self.pitch_history.append(raw_midi)
        self.pitch_history = self.pitch_history[-config.median_len:]
        self.smoothed_midi = float(np.median(self.pitch_history))
        candidate_note = self.nearest_allowed(self.smoothed_midi)
        if self.target_note is None:
            self.target_note = candidate_note
        elif candidate_note != self.target_note:
            margin_semitones = (abs(self.smoothed_midi - self.target_note)
                                - abs(self.smoothed_midi - candidate_note))
            if margin_semitones > 2.0 * config.hyst_cents / 100.0:
                self.candidate_count += 1
            else:
                self.candidate_count = 0
            if self.candidate_count >= config.hyst_frames:
                self.target_note, self.candidate_count = candidate_note, 0
        else:
            self.candidate_count = 0
        return self.smoothed_midi, self.target_note


class RangeMonitor:
    """Fixed-mode out-of-range flag: hysteresis + debounce, updated on voiced hops only.

    pitch_gap = |fixed target - smoothed MIDI pitch|. Enter when pitch_gap > max_shift + oor_margin,
    leave when pitch_gap < max_shift - oor_margin; either change needs `oor_frames` consecutive hops.
    Unvoiced hops (pitch_gap None) keep the state and the debounce count.
    """

    def __init__(self, config: Config):
        self.config = config
        self.is_out_of_range = False
        self.change_count = 0

    def reset(self):
        self.is_out_of_range, self.change_count = False, 0

    def update(self, pitch_gap: float | None) -> bool:
        if pitch_gap is None:
            return self.is_out_of_range
        config = self.config
        if self.is_out_of_range:
            wanted_state = not (pitch_gap < config.max_shift - config.oor_margin)
        else:
            wanted_state = pitch_gap > config.max_shift + config.oor_margin
        if wanted_state != self.is_out_of_range:
            self.change_count += 1
            if self.change_count >= config.oor_frames:
                self.is_out_of_range, self.change_count = wanted_state, 0
        else:
            self.change_count = 0
        return self.is_out_of_range


# --------------------------------------------------------------------------
# Noise suppression: STFT Wiener gain with minimum-statistics noise tracking
# --------------------------------------------------------------------------
class NoiseSuppressor:
    """Streaming spectral noise suppressor, one call per 128-sample hop.

    sqrt-Hann analysis and synthesis windows, N = 512, hop H = 128 (75 % overlap, window power sums to 2).
    Per bin k:
        P[k]   = |X[k]|^2                                        frame power
        Ps[k]  = a Ps[k] + (1 - a) P[k]                          smoothed over time
        Nz[k]  = min(Nz[k] * rise, Ps[k])                        noise tracker: follows dips, creeps upward
        gamma  = P / (bias * Nz)                                 a-posteriori SNR
        xi     = beta G_prev^2 gamma_prev + (1 - beta) max(gamma - 1, 0)    a-priori SNR (decision-directed)
        G      = max(xi / (1 + xi), g_floor)                     Wiener gain, then 3-tap smoothing over k
    The noise estimate needs no voice-activity detector, so it also follows noise that changes slowly.
    MCU: one 512-point real FFT + one inverse per hop (CMSIS arm_rfft_fast_f32), 257 bins of
    float arithmetic; the only divisions are xi/(1+xi) and P/(bias*Nz). Output delay = N - H samples.
    """

    def __init__(self, config: Config):
        self.config, self.N, self.H = config, config.N, config.H
        self.window = np.sqrt(0.5 * (1.0 - np.cos(2.0 * np.pi * np.arange(self.N) / self.N)))   # sqrt-Hann
        self.input_buffer = np.zeros(self.N)
        self.overlap_add = np.zeros(self.N)
        bins = self.N // 2 + 1
        self.smoothed_power: np.ndarray | None = None
        self.noise_power: np.ndarray | None = None
        self.previous_gain = np.ones(bins)
        self.previous_gamma = np.ones(bins)
        self.gain_floor = 10.0 ** (config.dn_floor_db / 20.0)
        self.rise = 10.0 ** (config.dn_rise_db_s / 10.0 * config.H / config.fs)

    def process(self, hop):
        config = self.config
        self.input_buffer = np.concatenate([self.input_buffer[self.H:], hop])
        spectrum = np.fft.rfft(self.input_buffer * self.window)
        power = spectrum.real ** 2 + spectrum.imag ** 2
        if self.smoothed_power is None:
            self.smoothed_power, self.noise_power = power.copy(), power.copy()
        else:
            self.smoothed_power = config.dn_smooth * self.smoothed_power + (1.0 - config.dn_smooth) * power
        self.noise_power = np.minimum(self.noise_power * self.rise, self.smoothed_power)
        posterior_snr = power / (config.dn_bias * self.noise_power + 1e-12)
        prior_snr = (config.dn_beta * self.previous_gain ** 2 * self.previous_gamma
                     + (1.0 - config.dn_beta) * np.maximum(posterior_snr - 1.0, 0.0))
        gain = np.maximum(prior_snr / (1.0 + prior_snr), self.gain_floor)
        self.previous_gain, self.previous_gamma = gain, posterior_snr
        padded = np.pad(gain, 1, mode="edge")
        gain = 0.25 * padded[:-2] + 0.5 * padded[1:-1] + 0.25 * padded[2:]
        self.overlap_add += np.fft.irfft(spectrum * gain, self.N) * self.window * 0.5
        output = self.overlap_add[:self.H].copy()
        self.overlap_add = np.concatenate([self.overlap_add[self.H:], np.zeros(self.H)])
        return output


# --------------------------------------------------------------------------
# TD-PSOLA, streaming
# --------------------------------------------------------------------------
class TDPSOLA:
    """Pitch-synchronous overlap-add with a fixed output delay of config.delay samples.

    Timeline at the moment `samples_received` samples have arrived (the end of the hop):
        output finalised up to   samples_received - delay      (delay = 400)
        synthesis marks placed up to   samples_received - delay + tau_max  = samples_received - 200
        pitch estimate refers to frame centre   samples_received - N/2 = samples_received - 256
    so the pitch information is never older than the marks it controls.

    This version keeps whole-signal arrays for clarity; the assertion in _ola()
    proves every access lies within the last HIST samples, so the C port can
    use ring buffers of 1024 samples.
    """
    HIST = 1024

    def __init__(self, config: Config, max_samples: int):
        self.config, self.delay = config, config.psola_delay
        size = max_samples + 2 * config.grain_reach + 16
        self.audio = np.zeros(size)           # audio path
        self.residual = np.zeros(size)        # LPC residual of the audio path (source_filter mode)
        self.residual_end = 0                 # residual is known for samples < residual_end
        self.lpc_reflection: list[np.ndarray] = []   # one reflection set per hop, centred N/2 behind its hop end
        self.synth_past = np.zeros(config.lpc_order)  # last outputs of the synthesis filter, newest first
        self.detection = np.zeros(size)       # detection path (used for mark picking)
        self.overlap_add = np.zeros(size)     # overlap-add accumulator
        self.window_sum = np.zeros(size)      # sum of window weights (for normalisation)
        self.samples_received = 0
        self.analysis_marks: list[int] = []   # analysis pitch marks
        self.nearest_mark_index = 0           # pointer to nearest analysis mark
        self.next_synthesis_mark: float | None = None   # next synthesis mark
        self.samples_emitted = 0
        self._hann_cache: dict[int, np.ndarray] = {}
        self.use_dry = np.zeros(size, dtype=bool)   # per-sample: use the dry (unshifted) input here
        self.active = np.zeros(size, dtype=bool)    # per-sample: someone is singing / speaking here (gate)
        self.gate_gain = 1.0
        self.dry_blend = 0.0                        # crossfade state, 0 = shifted, 1 = dry
        self.unvoiced_hops = 0
        self.smoothed_input_power = 0.0             # smoothed input / output power (level match)
        self.smoothed_output_power = 0.0
        self.level_gain = 1.0

    def _hann(self, half_width: int):
        window = self._hann_cache.get(half_width)
        if window is None:
            index = np.arange(2 * half_width + 1)
            window = 0.5 * (1.0 - np.cos(2.0 * np.pi * index / (2 * half_width)))   # COLA at hop half_width
            self._hann_cache[half_width] = window
        return window

    def push(self, audio_hop, detection_hop, period: float, pitch_ratio: float, voiced: bool = True,
             reflection=None, active: bool = True):
        hop_length = len(audio_hop)
        self.active[self.samples_received:self.samples_received + hop_length] = active
        self.audio[self.samples_received:self.samples_received + hop_length] = audio_hop
        self.detection[self.samples_received:self.samples_received + hop_length] = detection_hop
        self.samples_received += hop_length
        if self.config.source_filter:
            self._update_residual(reflection)
        # the voicing decision is about the frame centre (samples_received - N/2): mark that hop-wide region
        self.unvoiced_hops = 0 if voiced else self.unvoiced_hops + 1
        frame_center = self.samples_received - self.config.N // 2
        region_start, region_end = max(frame_center - hop_length // 2, 0), frame_center + hop_length // 2
        if region_end > 0:
            self.use_dry[region_start:region_end] = (
                self.config.dry_unvoiced and self.unvoiced_hops >= self.config.dry_after_hops)
        self._extend_marks(period)
        self._place(period, pitch_ratio)
        return self._emit()

    def _update_residual(self, reflection):
        """Inverse-filter the hop-wide region around the newest frame centre with that frame's LPC."""
        config = self.config
        order = config.lpc_order
        reflection = (np.zeros(order) if reflection is None else np.asarray(reflection))
        self.lpc_reflection.append(reflection)
        centre = self.samples_received - config.N // 2
        start, stop = max(centre - config.H // 2, 0), centre + config.H // 2
        if stop <= start:
            return
        history = max(start - order, 0)
        segment = signal.lfilter(step_up(reflection), [1.0], self.audio[history:stop])
        self.residual[start:stop] = segment[start - history:]
        self.residual_end = stop

    def _mark_signal(self):
        """(signal, number of valid samples) the pitch marks are placed on."""
        if self.config.source_filter:
            return self.residual, self.residual_end
        return self.detection, self.samples_received

    def _extend_marks(self, period: float):
        mark_signal, available = self._mark_signal()
        samples_received = available
        if not self.analysis_marks:
            first_period = int(round(period))
            if samples_received < first_period + 1:
                return
            self.analysis_marks.append(int(np.argmax(np.abs(mark_signal[:first_period]))))
        if self.config.mark_mode == "xcorr":
            self._extend_marks_xcorr(period)
            return
        while True:
            last_mark = self.analysis_marks[-1]
            search_start = last_mark + max(1, int(round(0.7 * period)))
            search_end = last_mark + int(round(1.3 * period))
            if search_end >= samples_received:
                break
            self.analysis_marks.append(search_start + int(np.argmax(mark_signal[search_start:search_end + 1])))

    def _extend_marks_xcorr(self, period: float):
        """Next mark = the position near (last mark + period) whose surrounding waveform best matches
        the waveform around the last mark (normalised cross-correlation over one period).

        Peak-picking jumps between the several peaks inside one period (formant ringing), so the grains
        are cut at inconsistent phases and the lowered voice sounds rough and robotic. Following the
        previous grain keeps all marks on the same feature of the waveform. MCU: about 2 x 0.3 period
        candidates x one period of multiply-adds per mark, i.e. ~10k MACs per 10 ms, no FFT.
        """
        mark_signal, samples_received = self._mark_signal()
        correlation_half = max(4, int(round(0.5 * period)))
        tolerance = max(2, int(round(self.config.mark_tol * period)))
        while True:
            last_mark = self.analysis_marks[-1]
            predicted = last_mark + max(1, int(round(period)))
            if predicted + tolerance + correlation_half >= samples_received:
                break
            if last_mark - correlation_half < 0:
                self.analysis_marks.append(predicted)
                continue
            reference = mark_signal[last_mark - correlation_half:last_mark + correlation_half + 1]
            reference = reference - reference.mean()
            reference_norm = np.sqrt(np.dot(reference, reference)) + 1e-12
            candidates = np.arange(predicted - tolerance, predicted + tolerance + 1)
            windows = np.lib.stride_tricks.sliding_window_view(
                mark_signal[candidates[0] - correlation_half:candidates[-1] + correlation_half + 1],
                2 * correlation_half + 1)
            windows = windows - windows.mean(axis=1, keepdims=True)
            scores = (windows @ reference) / (np.sqrt(np.einsum("ij,ij->i", windows, windows)) * reference_norm + 1e-12)
            self.analysis_marks.append(int(candidates[int(np.argmax(scores))]))

    def _place(self, period: float, pitch_ratio: float):
        config = self.config
        samples_received = self._mark_signal()[1]        # grains may only use samples that are known
        finalized_until = self.samples_received - self.delay
        if finalized_until <= 0 or not self.analysis_marks:
            return
        synthesis_period = period / pitch_ratio
        grain_half_width = int(round(period))          # grain half-width = one analysis period
        if self.next_synthesis_mark is None:
            self.next_synthesis_mark = float(self.analysis_marks[0])
        while self.next_synthesis_mark < finalized_until + config.grain_reach:
            mark_index = self.nearest_mark_index
            while mark_index > 0 and self.analysis_marks[mark_index] + grain_half_width >= samples_received:
                mark_index -= 1
            while (mark_index + 1 < len(self.analysis_marks)
                   and self.analysis_marks[mark_index + 1] + grain_half_width < samples_received
                   and abs(self.analysis_marks[mark_index + 1] - self.next_synthesis_mark)
                   <= abs(self.analysis_marks[mark_index] - self.next_synthesis_mark)):
                mark_index += 1
            self.nearest_mark_index = mark_index
            analysis_mark = self.analysis_marks[mark_index]
            if analysis_mark + grain_half_width >= samples_received:
                break                           # grain not fully available yet
            self._ola(analysis_mark, grain_half_width, int(round(self.next_synthesis_mark)),
                      pitch_ratio, synthesis_period)
            self.next_synthesis_mark += synthesis_period

    def _ola(self, analysis_mark: int, grain_half_width: int, center: int, pitch_ratio: float,
             synthesis_period: float):
        """Add one Hann-windowed grain (half-width grain_half_width, from analysis_mark) centred at `center`.

        Level: Hann grains of half-width L at spacing synthesis_period sum to L/synthesis_period on
        average, so the grain is scaled by synthesis_period/L (= 1/pitch_ratio for plain TD-PSOLA).
        This keeps each grain's tapered edges small; those edges hold the neighbouring glottal pulses.

        Lowering (pitch_ratio < 1): plain TD-PSOLA leaves a near-silent gap of about (1 - pitch_ratio)
        of every output period, which sounds like a buzzy pulse train. The grain is
        therefore stretched in time by stretch = (1/pitch_ratio)^grain_stretch (linear interpolation),
        which fills part of the gap and lowers the formants by the same factor.
        The neighbouring pulses stay at the window edges, so no extra pulses appear.
        """
        assert analysis_mark - grain_half_width >= self.samples_received - self.HIST, \
            "access outside the 1024-sample history"
        start, stop = analysis_mark - grain_half_width, analysis_mark + grain_half_width + 1
        grain = np.zeros(2 * grain_half_width + 1)
        clipped_start = max(start, 0)
        source = self.residual if self.config.source_filter else self.audio
        grain[clipped_start - start:] = source[clipped_start:stop]
        window = self._hann(grain_half_width)
        windowed_grain = grain * window
        stretch = 1.0
        exponent = self.config.residual_stretch if self.config.source_filter else self.config.grain_stretch
        if pitch_ratio < 1.0 and exponent > 0:
            stretch = (1.0 / pitch_ratio) ** exponent
            stretch = min(stretch, self.config.grain_reach / grain_half_width)   # keep the grain inside the output lookahead
        stretched_half_width = int(round(stretch * grain_half_width))
        if stretched_half_width != grain_half_width:
            read_positions = (np.arange(-stretched_half_width, stretched_half_width + 1)
                              * (grain_half_width / stretched_half_width) + grain_half_width)   # positions in the original grain
            windowed_grain = np.interp(read_positions, np.arange(2 * grain_half_width + 1), windowed_grain)
            window = np.interp(read_positions, np.arange(2 * grain_half_width + 1), window)
        grain_gain = synthesis_period / stretched_half_width     # MCU: one multiply per sample
        output_start = center - stretched_half_width
        clipped_output_start = max(output_start, 0)
        output_stop = output_start + 2 * stretched_half_width + 1
        self.overlap_add[clipped_output_start:output_stop] += grain_gain * windowed_grain[clipped_output_start - output_start:]
        self.window_sum[clipped_output_start:output_stop] += window[clipped_output_start - output_start:]

    def _emit(self):
        finalized_until = self.samples_received - self.delay
        if finalized_until <= self.samples_emitted:
            return np.zeros(0)
        start, self.samples_emitted = self.samples_emitted, finalized_until
        if self.config.ola_norm == "w":
            output = self.overlap_add[start:finalized_until] / np.maximum(self.window_sum[start:finalized_until], 1e-3)
        else:
            output = self.overlap_add[start:finalized_until].copy()
        if self.config.source_filter:
            output = self._synthesize(output, start)       # excitation -> speech through the vocal-tract filter
        if not self.config.dry_unvoiced:
            return self._gate(self._level(output, start, finalized_until), start, finalized_until)
        # crossfade between the shifted output and the dry input (linear ramp, dry_fade samples)
        step = 1.0 / max(self.config.dry_fade, 1)
        blend = np.empty(len(output))
        blend_state = self.dry_blend
        for i, wants_dry in enumerate(self.use_dry[start:finalized_until]):   # MCU: same per-sample slew
            blend_state = min(blend_state + step, 1.0) if wants_dry else max(blend_state - step, 0.0)
            blend[i] = blend_state
        self.dry_blend = blend_state
        return self._gate(self._level((1.0 - blend) * output + blend * self.audio[start:finalized_until],
                                      start, finalized_until), start, finalized_until)

    def _synthesize(self, excitation, start: int):
        """All-pole synthesis 1/A(z) over the finalised excitation, in blocks of lpc_block samples.

        Each block uses reflection coefficients interpolated linearly between the two nearest analysis
        frame centres (stable for |k| < 1). The filter keeps its own past outputs between blocks, so the
        formants keep ringing through the gaps that a lowered pitch leaves between excitation pulses.
        MCU: 16 multiply-adds per sample + one step-up (~140 MACs) per block.
        """
        config = self.config
        order, block = config.lpc_order, config.lpc_block
        count = len(self.lpc_reflection)
        output = np.empty(len(excitation))
        position = 0
        while position < len(excitation):
            length = min(block, len(excitation) - position)
            middle = start + position + length / 2.0
            hop_index = (middle + config.N // 2) / config.H - 1.0
            lower = int(np.clip(np.floor(hop_index), 0, count - 1))
            upper = min(lower + 1, count - 1)
            fraction = float(np.clip(hop_index - lower, 0.0, 1.0))
            reflection = (1.0 - fraction) * self.lpc_reflection[lower] + fraction * self.lpc_reflection[upper]
            denominator = step_up(reflection)
            initial = signal.lfiltic([1.0], denominator, self.synth_past)
            filtered, _ = signal.lfilter([1.0], denominator, excitation[position:position + length], zi=initial)
            output[position:position + length] = filtered
            self.synth_past = np.concatenate([filtered[::-1], self.synth_past])[:order]
            position += length
        return output

    def _gate(self, output, start: int, stop: int):
        """Pause gate: full gain near active hops (hold after, lookahead before), gate_depth_db elsewhere.

        The decision for sample n looks at active[n - hold : n + lookahead]; the lookahead is free because
        output sample n is only emitted psola_delay samples after it arrived. Gain moves with a one-pole
        attack/release. MCU: a counter per hop + one multiply per sample.
        """
        config = self.config
        if not config.gate or len(output) == 0:
            return output
        fs = config.fs
        hold, ahead = int(config.gate_hold_s * fs), int(config.gate_lookahead_s * fs)
        lo, hi = max(start - hold, 0), min(stop + ahead, self.samples_received)
        window = self.active[lo:hi].astype(np.int32)
        cumulative = np.concatenate([[0], np.cumsum(window)])
        index = np.arange(start, stop)
        a = np.clip(index - hold - lo, 0, len(window))
        b = np.clip(index + ahead + 1 - lo, 0, len(window))
        target = np.where(cumulative[b] - cumulative[a] > 0, 1.0, 10.0 ** (config.gate_depth_db / 20.0))
        attack = 1.0 - np.exp(-1.0 / (config.gate_attack_s * fs))
        release = 1.0 - np.exp(-1.0 / (config.gate_release_s * fs))
        gains = np.empty(len(output))
        gain = self.gate_gain
        for i, wanted in enumerate(target):                    # MCU: same per-sample slew
            gain += (attack if wanted > gain else release) * (wanted - gain)
            gains[i] = gain
        self.gate_gain = gain
        return output * gains

    def _level(self, output, start: int, stop: int):
        """Match the output level to the (time-aligned) input level.

        TD-PSOLA changes the level with the shift (about -7 dB at +12 st, +2 dB at -12 st).
        Input and output power are tracked with a slow one-pole (level_tau_s), and the gain
        sqrt(input power / output power) is ramped linearly across the block. Silence does not update
        the estimate, so pauses are never boosted. MCU: two multiply-adds per sample + one sqrt per hop.
        """
        config = self.config
        if not config.level_match or len(output) == 0:
            return output
        input_block = self.audio[start:stop]
        input_power, output_power = float(np.mean(input_block * input_block)), float(np.mean(output * output))
        if input_power > 1e-7 and output_power > 1e-9:                     # about -70 dBFS: skip silence
            smoothing = 1.0 - np.exp(-len(output) / (config.fs * config.level_tau_s))
            self.smoothed_input_power += smoothing * (input_power - self.smoothed_input_power)
            self.smoothed_output_power += smoothing * (output_power - self.smoothed_output_power)
        new_gain = self.level_gain
        if self.smoothed_output_power > 0:
            gain_limit = 10.0 ** (config.level_max_db / 20.0)
            new_gain = float(np.clip(np.sqrt(self.smoothed_input_power / self.smoothed_output_power),
                                     1.0 / gain_limit, gain_limit))
        gain_ramp = np.linspace(self.level_gain, new_gain, len(output) + 1)[1:]
        self.level_gain = new_gain
        return output * gain_ramp


# --------------------------------------------------------------------------
# The whole pipeline
# --------------------------------------------------------------------------
class AutoTuner:
    def __init__(self, config: Config | None = None, synthesize: bool = True, max_samples: int = 1 << 20):
        self.config = config = config or Config()
        self.synthesize = synthesize
        self.bandpass_sos = design_bandpass(config)
        self.bandpass_state = np.zeros((self.bandpass_sos.shape[0], 2))
        self.highpass_sos = (signal.butter(2, config.audio_hpf_hz, "highpass", fs=config.fs, output="sos")
                             if config.audio_hpf_hz else None)
        self.highpass_state = (np.zeros((self.highpass_sos.shape[0], 2))
                               if self.highpass_sos is not None else None)
        self.denoiser = NoiseSuppressor(config) if config.denoise else None
        self.audio_frame = np.zeros(config.N)
        self.lpc_window = np.hanning(config.N)
        lag_index = np.arange(config.lpc_order + 1)
        self.lpc_lag_window = np.exp(-0.5 * (2.0 * np.pi * config.lpc_lag_hz * lag_index / config.fs) ** 2)
        self.last_reflection = np.zeros(config.lpc_order)
        self.gate_floor_power: float | None = None             # tracked noise floor of the cleaned signal
        self.gate_smoothed_power = 0.0
        self.gate_rise = 10.0 ** (3.0 / 10.0 * config.H / config.fs)   # floor may rise 3 dB/s (old gate)
        self.gate_rise_unvoiced = 10.0 ** (config.gate_floor_rise_db_s / 10.0 * config.H / config.fs)
        self.gate_rise_voiced = 10.0 ** (config.gate_floor_rise_voiced_db_s / 10.0 * config.H / config.fs)
        self.frame = np.zeros(config.N)
        self.hop_energy = np.zeros(config.N // config.H)   # sum of squares of the raw input, last N samples
        self.samples_seen = 0
        self.pitch_detector = PitchDetector(config)
        self.note_tracker = NoteTracker(config)
        self.retune_gain = (1.0 if config.retune_tau_s <= 0
                            else 1.0 - np.exp(-config.H / config.fs / config.retune_tau_s))
        self.smoothed_shift = 0.0                  # smoothed shift in semitones
        self.last_period = config.fs / 160.0       # last valid pitch period in samples
        self.psola = TDPSOLA(config, max_samples) if synthesize else None
        # run-time controls (on the MCU: volatile, written by the UART command parser)
        self.mode = MODE_NEAREST
        self.fixed_target_midi = config.target_midi
        self.set_mode(config.mode)
        self.set_target(config.target_midi)
        self.range_monitor = RangeMonitor(config)

    def _lpc(self, frame):
        """Reflection coefficients of the newest N-sample audio frame (kept from before if the frame is silent)."""
        config = self.config
        windowed = frame * self.lpc_window
        autocorr = np.correlate(windowed, windowed, "full")[config.N - 1:config.N + config.lpc_order]
        if autocorr[0] < 1e-9:
            return self.last_reflection
        autocorr = autocorr * self.lpc_lag_window
        autocorr[0] *= 1.0 + 1e-4                              # white-noise correction (-40 dB)
        self.last_reflection = levinson(autocorr, config.lpc_order)[1]
        return self.last_reflection

    def set_mode(self, mode: int):
        if mode not in MODE_NAMES:
            raise ValueError(f"unknown mode {mode}")
        self.mode = mode

    def set_target(self, target):
        midi_note = parse_note(target) if isinstance(target, str) else int(target)
        if not TARGET_MIN <= midi_note <= TARGET_MAX:
            raise ValueError(f"target {midi_note} outside {TARGET_MIN}..{TARGET_MAX}")   # MCU: ignore command
        self.fixed_target_midi = midi_note

    def process_hop(self, hop_samples):
        config = self.config
        hop_samples = np.asarray(hop_samples, dtype=float)
        if self.denoiser is not None:
            hop_samples = self.denoiser.process(hop_samples)   # everything below sees the cleaned signal
        if config.use_filter:
            detection_hop, self.bandpass_state = signal.sosfilt(self.bandpass_sos, hop_samples,
                                                                zi=self.bandpass_state)
        else:
            detection_hop = hop_samples
        if self.highpass_sos is not None:
            audio_hop, self.highpass_state = signal.sosfilt(self.highpass_sos, hop_samples,
                                                            zi=self.highpass_state)
        else:
            audio_hop = hop_samples

        self.frame = np.concatenate([self.frame[config.H:], detection_hop])
        self.hop_energy = np.concatenate([self.hop_energy[1:], [np.sum(hop_samples ** 2)]])
        self.samples_seen += config.H

        f0_hz, confidence, lag_samples, rms_db = float("nan"), 0.0, float("nan"), -120.0
        if self.samples_seen >= config.N:
            # level gate on the *raw* input: band-passing would hide energy above 1 kHz
            rms_db = 10.0 * np.log10(self.hop_energy.sum() / config.N + 1e-24)
            detection = self.pitch_detector(self.frame)
            if detection is not None:
                f0_hz, confidence, lag_samples = detection
        voiced = bool(np.isfinite(f0_hz) and confidence > config.voicing_thresh
                      and rms_db > config.rms_floor_dbfs)

        # controls are read once per hop, so a command takes effect at the next hop boundary
        mode, fixed_target_midi = self.mode, self.fixed_target_midi

        smoothed_midi, tracked_note = self.note_tracker.update(float(hz_to_midi(f0_hz)) if voiced else None)
        target_note = fixed_target_midi if mode == MODE_FIXED else tracked_note

        # shift in semitones, clamped; relaxed to 0 when unvoiced
        has_target = voiced and smoothed_midi is not None and target_note is not None
        raw_shift = (target_note - smoothed_midi) if has_target else 0.0
        shift = float(np.clip(raw_shift, -config.max_shift, config.max_shift))
        self.smoothed_shift += self.retune_gain * (shift - self.smoothed_shift)
        pitch_ratio = 2.0 ** (self.smoothed_shift / 12.0)

        # out-of-range indication (fixed mode only; display only, the clamp above always applies)
        if mode == MODE_FIXED:
            out_of_range = self.range_monitor.update(
                abs(fixed_target_midi - smoothed_midi) if has_target else None)
        else:
            self.range_monitor.reset()
            out_of_range = False
        status = (ST_OUT_OF_RANGE if out_of_range else 0) | (ST_MODE_FIXED if mode == MODE_FIXED else 0) \
            | (ST_VOICED if voiced else 0)

        if voiced:
            self.last_period = float(np.clip(config.fs / midi_to_hz(smoothed_midi),
                                             config.tau_min, config.tau_max))
        period = self.last_period

        # The dict keys are the column names of the golden-vector CSV for the C port, so they stay short and fixed.
        info = dict(n_end=self.samples_seen, f0=f0_hz if voiced else float("nan"), conf=confidence,
                    tau=lag_samples, rms_db=rms_db, voiced=voiced,
                    m_s=smoothed_midi if smoothed_midi is not None else float("nan"),
                    target=target_note if target_note is not None else -1,
                    mode=mode, d_raw=raw_shift, d=shift, d_s=self.smoothed_shift, oor=out_of_range,
                    status=status, alpha=pitch_ratio, T=period)
        hop_power = float(np.mean(hop_samples ** 2)) + 1e-14       # cleaned signal, before the filters
        self.gate_smoothed_power += 0.3 * (hop_power - self.gate_smoothed_power)   # ~3-hop average
        hop_power = self.gate_smoothed_power
        snr_gate = config.gate_pitch_only and config.gate_snr_db > 0
        if hop_power > 1e-9:                                         # ignore digital silence (start-up, padding)
            if self.gate_floor_power is None:
                self.gate_floor_power = hop_power
            if snr_gate:
                rise = self.gate_rise_voiced if voiced else self.gate_rise_unvoiced
            else:
                rise = self.gate_rise
            self.gate_floor_power = min(self.gate_floor_power * rise, hop_power)
        # a click/snap puts most of the frame's energy into one 8 ms hop; a sung note spreads it evenly
        transient = self.hop_energy.max() > config.gate_transient_share * (self.hop_energy.sum() + 1e-20)
        gate_voiced = voiced and not (config.gate_pitch_only and transient)
        if snr_gate:                                                 # and clearly above the background
            gate_voiced = gate_voiced and (self.gate_floor_power is not None and
                                           hop_power > self.gate_floor_power * 10.0 ** (config.gate_snr_db / 10.0))
        active = gate_voiced or (not config.gate_pitch_only and self.gate_floor_power is not None and
                            hop_power > self.gate_floor_power * 10.0 ** (config.gate_open_db / 10.0))
        reflection = None
        if config.source_filter:
            self.audio_frame = np.concatenate([self.audio_frame[config.H:], audio_hop])
            reflection = self._lpc(self.audio_frame)
        output = (self.psola.push(audio_hop, detection_hop, period, pitch_ratio, voiced, reflection, active)
                  if self.synthesize else None)
        return info, output


def run(samples, config: Config | None = None, synthesize: bool = True, commands=None):
    """Process a whole signal. Returns (output, track); output[t] ~ samples[t], track = per-hop arrays.

    commands: optional list of (time_s, {"mode": ..., "target": ...}) simulating CMD
    messages from the laptop; each is applied before the first hop starting at or after time_s.
    """
    config = config or Config()
    samples = np.asarray(samples, dtype=float)
    signal_length, hop_length = len(samples), config.H
    pre_delay = config.denoise_delay
    total_length = int(np.ceil((signal_length + pre_delay + config.psola_delay + 2 * hop_length) / hop_length)) * hop_length
    padded_samples = np.zeros(total_length)
    padded_samples[:signal_length] = samples
    tuner = AutoTuner(config, synthesize=synthesize, max_samples=total_length)
    pending_commands = sorted(commands or [], key=lambda command: command[0])
    hop_infos, output_chunks = [], []
    for hop_start in range(0, total_length, hop_length):
        while pending_commands and pending_commands[0][0] * config.fs <= hop_start:
            command = pending_commands.pop(0)[1]
            if "mode" in command:
                tuner.set_mode(command["mode"])
            if "target" in command:
                tuner.set_target(command["target"])
        info, output_chunk = tuner.process_hop(padded_samples[hop_start:hop_start + hop_length])
        hop_infos.append(info)
        if synthesize:
            output_chunks.append(output_chunk)
    track = {key: np.array([info[key] for info in hop_infos]) for key in hop_infos[0]}
    output = np.concatenate(output_chunks)[pre_delay:pre_delay + signal_length] if synthesize else None
    return output, track


# --------------------------------------------------------------------------
# I/O helpers, golden vectors, CLI
# --------------------------------------------------------------------------
def load_wav(path: str, sample_rate: int):
    from scipy.io import wavfile
    file_sample_rate, samples = wavfile.read(path)
    if samples.dtype.kind == "i":
        samples = samples.astype(float) / np.iinfo(samples.dtype).max
    elif samples.dtype.kind == "u":
        samples = (samples.astype(float) - 128) / 128
    else:
        samples = samples.astype(float)
    if samples.ndim > 1:
        samples = samples.mean(axis=1)
    if file_sample_rate != sample_rate:
        from math import gcd
        divisor = gcd(file_sample_rate, sample_rate)
        samples = signal.resample_poly(samples, sample_rate // divisor, file_sample_rate // divisor)
    return samples


def make_louder(samples, sample_rate: int, gain_db: float | None = None, target_dbfs: float = -18.0,
                ceiling_dbfs: float = -1.0, max_gain_db: float = 30.0):
    """Output gain + soft limiter.

    gain_db None = automatic: the loud parts (95th percentile of the 100 ms energy) are brought to
    target_dbfs, at most +max_gain_db. Then a soft limiter keeps peaks under ceiling_dbfs:
    below half the ceiling the signal is untouched, above it tanh bends it smoothly towards the ceiling,
    so single clicks do not limit how loud the voice can be. Returns (output, applied gain in dB).
    MCU/laptop: one multiply + one tanhf (or a small table) per sample.
    """
    samples = np.asarray(samples, dtype=float)
    if gain_db is None:
        window = max(1, int(0.1 * sample_rate))
        energy = np.convolve(samples ** 2, np.ones(window) / window, mode="same")
        loud_dbfs = 10.0 * np.log10(np.percentile(energy, 95) + 1e-12)
        gain_db = float(np.clip(target_dbfs - loud_dbfs, 0.0, max_gain_db))
    louder = samples * 10.0 ** (gain_db / 20.0)
    ceiling = 10.0 ** (ceiling_dbfs / 20.0)
    knee = 0.5 * ceiling
    magnitude = np.abs(louder)
    over = magnitude > knee
    limited = louder.copy()
    limited[over] = np.sign(louder[over]) * (knee + (ceiling - knee)
                                             * np.tanh((magnitude[over] - knee) / (ceiling - knee)))
    return limited, gain_db


def save_wav(path: str, samples, sample_rate: int):
    from scipy.io import wavfile
    wavfile.write(path, sample_rate,
                  np.clip(np.round(np.asarray(samples) * 32767), -32768, 32767).astype(np.int16))


def export_golden(output_dir: str, samples, config: Config, commands=None):
    """Dump int16 input/output and per-hop values for checking the C port.

    The Python run uses the int16-quantised input, so C and Python see identical samples.
    """
    os.makedirs(output_dir, exist_ok=True)
    quantized_input = np.clip(np.round(np.asarray(samples) * 32768), -32768, 32767).astype(np.int16)
    output, track = run(quantized_input / 32768.0, config, commands=commands)
    quantized_input.tofile(os.path.join(output_dir, "input_q15.raw"))
    np.clip(np.round(output * 32768), -32768, 32767).astype(np.int16).tofile(
        os.path.join(output_dir, "output_q15.raw"))
    column_names = ["n_end", "f0", "conf", "tau", "rms_db", "voiced", "m_s", "target", "mode",
                    "d_raw", "d", "d_s", "oor", "status", "alpha", "T"]
    with open(os.path.join(output_dir, "hops.csv"), "w") as csv_file:
        csv_file.write("hop," + ",".join(column_names) + "\n")
        for hop_index in range(len(track["n_end"])):
            csv_file.write(f"{hop_index}," + ",".join(f"{float(track[name][hop_index]):.9g}"
                                                      for name in column_names) + "\n")
    tuner = AutoTuner(config, synthesize=False)
    np.savetxt(os.path.join(output_dir, "sos_bp.csv"), tuner.bandpass_sos, delimiter=",", fmt="%.17g")
    if tuner.highpass_sos is not None:
        np.savetxt(os.path.join(output_dir, "sos_hp.csv"), tuner.highpass_sos, delimiter=",", fmt="%.17g")
    return output, track


def display_lines(info, width: int = 21) -> tuple[str, str]:
    """Two display lines for one hop's info (spec section 4).

    width=21: SSD1306 128 px with the 6 px font; width=16: LCD1602.
    Line 1: target note and mode, line 2: sung note and status (st / cents / OUT OF RANGE).
    """
    mode, voiced = int(info["mode"]), bool(info["voiced"])
    target_note = int(info["target"])
    smoothed_midi = float(info["m_s"])
    has_target = voiced and target_note >= 0 and np.isfinite(smoothed_midi)
    target_name = note_name(target_note) if target_note >= 0 else "--"
    sung_name = note_name(int(np.floor(smoothed_midi + 0.5))) if voiced and np.isfinite(smoothed_midi) else "--"
    target_prefix, sung_prefix = ("Target: ", "Sung: ") if width > 16 else ("T:", "S:")
    if mode == MODE_FIXED and info["oor"]:
        hints = ["OUT OF RANGE", "OUT RANGE", "OOR"]
    elif not has_target:
        hints = [""]
    elif mode == MODE_FIXED:
        hints = [f"{target_note - int(np.floor(smoothed_midi + 0.5)):+d} st"]
    else:
        hints = [f"{100 * (smoothed_midi - target_note):+.0f} c"]

    def fit(left_text, right_text):
        return (left_text + right_text.rjust(width - len(left_text))
                if len(left_text) + 1 + len(right_text) <= width else None)

    line1 = (fit(target_prefix + target_name, f"[{MODE_NAMES[mode]}]")
             or fit(target_prefix + target_name, f"[{MODE_NAMES[mode][0]}]"))
    line2 = next((text for text in (fit(sung_prefix + sung_name, hint) for hint in hints) if text),
                 (sung_prefix + sung_name)[:width])
    return line1, line2


def summarize(track) -> dict:
    """Per-recording log values (spec section 7)."""
    voiced = track["voiced"].astype(bool)
    fixed_voiced = voiced & (track["mode"] == MODE_FIXED)
    return dict(voiced_pct=100 * voiced.mean(),
                oor_pct=100 * (track["oor"].astype(bool) & fixed_voiced).sum() / max(fixed_voiced.sum(), 1),
                fixed_voiced_hops=int(fixed_voiced.sum()))


def plot_track(path: str, track, config: Config, title: str = ""):
    """Pitch plot: sung pitch, target, output shift, out-of-range stretches shaded."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    time_s = (track["n_end"] - config.N // 2 - config.denoise_delay) / config.fs
    voiced = track["voiced"].astype(bool)
    sung_pitch = np.where(voiced, track["m_s"], np.nan)
    target_pitch = np.where(voiced & (track["target"] >= 0), track["target"], np.nan)
    output_pitch = np.where(voiced, track["m_s"] + track["d_s"], np.nan)
    fig, ax = plt.subplots(figsize=(11, 4))
    out_of_range = track["oor"].astype(bool)
    edges = np.flatnonzero(np.diff(np.r_[0, out_of_range.astype(int), 0]))
    for span_index, (span_start, span_end) in enumerate(zip(edges[::2], edges[1::2])):
        ax.axvspan(time_s[span_start], time_s[span_end - 1] + config.H / config.fs, color="#eb6834",
                   alpha=0.15, lw=0, label="out of range" if span_index == 0 else None)
    ax.plot(time_s, sung_pitch, ".", ms=3, color="#898781", label="sung pitch")
    ax.plot(time_s, target_pitch, "-", lw=1.5, color="#2a78d6", label="target note")
    ax.plot(time_s, output_pitch, "-", lw=1, color="#0b0b0b", label="output pitch (sung + shift)")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("MIDI note")
    ax.set_title(title, loc="left")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", fontsize=8, ncol=4, frameon=False)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", nargs="?", help="input wav (any rate, mono or stereo)")
    parser.add_argument("-o", "--output", default="out.wav")
    parser.add_argument("--demo", action="store_true", help="use a synthetic detuned voice")
    parser.add_argument("--mode", choices=["nearest", "fixed"], default="nearest")
    parser.add_argument("--target", help="fixed-mode target note, e.g. C6, F#3 or MIDI 84")
    parser.add_argument("--scale", choices=list(SCALES), default="chromatic", help="nearest mode only")
    parser.add_argument("--key", default="C", choices=NOTE_NAMES)
    parser.add_argument("--retune", type=float, default=0.05, help="retune time constant in s (0 = snap)")
    parser.add_argument("--stretch", type=float, default=None,
                        help="lowering only, 0..1: how much the grains are stretched. 0 = buzzy/robotic at big "
                             "shifts; 1 = slowed-down-tape sound (and, with --classic, deeper formants). "
                             f"Default {Config.residual_stretch} (LPC) / {Config.grain_stretch} (--classic)")
    parser.add_argument("--no-filter", action="store_true", help="disable the detection band-pass")
    parser.add_argument("--no-denoise", action="store_true", help="disable the noise suppressor")
    parser.add_argument("--gate-db", type=float, default=Config.gate_depth_db,
                        help="extra attenuation between phrases (default %(default)s; 0 = gate off)")
    parser.add_argument("--gate-pitch-only", action="store_true",
                        help="(default) gate opens only when a pitch is detected: cuts snaps, taps, clicks in pauses")
    parser.add_argument("--gate-snr", type=float, default=Config.gate_snr_db,
                        help="pitch-only gate: also require this many dB above the background floor "
                             "(default %(default)s; 0 = old gate, which loud steady noise can open)")
    parser.add_argument("--gate-level", action="store_true",
                        help="gate also opens on loud sounds (keeps word-initial s/t/k, but lets snaps through)")
    parser.add_argument("--classic", action="store_true",
                        help="v1 shifter: TD-PSOLA on the waveform with peak-picked marks (for A/B)")
    parser.add_argument("--denoise-db", type=float, default=Config.dn_floor_db,
                        help="noise suppressor floor in dB (default %(default)s; -35 = stronger, -15 = gentler)")
    parser.add_argument("--loud", nargs="?", const="auto", metavar="DB",
                        help="make the output louder (soft limiter keeps peaks under -1 dBFS). "
                             "No value = automatic (loud parts to -18 dBFS), or give the gain in dB, e.g. --loud 12")
    parser.add_argument("--plot", help="save a pitch plot (out-of-range shaded) to this PNG")
    parser.add_argument("--display", choices=["oled", "lcd"],
                        help="print the display text about 10 times a second")
    parser.add_argument("--play", nargs="?", const="", metavar="DEVICE",
                        help="play the output: speaker, airpods, a device index or a name substring "
                             "(no value = ask)")
    parser.add_argument("--list-devices", action="store_true", help="list audio output devices and exit")
    parser.add_argument("--golden", help="directory to write golden vectors for the C port")
    args = parser.parse_args()

    if args.list_devices:
        import playback
        playback.print_devices()
        return

    pitch_classes = SCALES[args.scale]
    if pitch_classes is not None:
        key_offset = NOTE_NAMES.index(args.key)
        pitch_classes = tuple((pitch_class + key_offset) % 12 for pitch_class in pitch_classes)
    mode = MODE_FIXED if args.mode == "fixed" else MODE_NEAREST
    if mode == MODE_FIXED and not args.target:
        parser.error("--mode fixed needs --target (e.g. --target C4)")
    try:
        fixed_target = parse_note(args.target) if args.target else Config.target_midi
    except ValueError as error:
        parser.error(str(error))
    if not TARGET_MIN <= fixed_target <= TARGET_MAX:
        parser.error(f"--target must be between {note_name(TARGET_MIN)} and {note_name(TARGET_MAX)}")
    if args.stretch is not None and not 0.0 <= args.stretch <= 1.0:
        parser.error("--stretch must be between 0 and 1")
    stretch_options = {}
    if args.stretch is not None:
        stretch_options = ({"grain_stretch": args.stretch} if args.classic
                           else {"residual_stretch": args.stretch,
                                 "max_stretch": max(1.0, min(2.0, 2.0 ** args.stretch))})
    config = Config(scale_pcs=pitch_classes, retune_tau_s=args.retune, use_filter=not args.no_filter,
                    denoise=not args.no_denoise, dn_floor_db=args.denoise_db,
                    gate=args.gate_db < 0, gate_depth_db=min(args.gate_db, 0.0),
                    gate_pitch_only=not args.gate_level, gate_snr_db=args.gate_snr,
                    source_filter=not args.classic, mark_mode="peak" if args.classic else "xcorr",
                    mode=mode, target_midi=fixed_target, **stretch_options)
    mode_text = (f"mode {MODE_NAMES[mode]}"
                 + (f", target {note_name(fixed_target)}" if mode == MODE_FIXED else ""))

    if args.demo:
        import synth
        test_signal = synth.make_test_signal(config.fs)
        noisy_samples, _, _ = synth.add_noise(test_signal["x"], "white", 10, test_signal["f0"] > 0,
                                              np.random.default_rng(1))
        demo_inputs = {"demo_clean": test_signal["x"], "demo_noisy10dB": noisy_samples}
        for demo_name, samples in demo_inputs.items():
            output, track = run(samples, config)
            save_wav(f"{demo_name}.wav", samples, config.fs)
            save_wav(f"{demo_name}_tuned.wav", output, config.fs)
            summary = summarize(track)
            print(f"{demo_name}: wrote {demo_name}.wav and {demo_name}_tuned.wav ({mode_text}; "
                  f"{summary['voiced_pct']:.0f}% of hops voiced"
                  + (f", {summary['oor_pct']:.0f}% of voiced hops out of range"
                     if mode == MODE_FIXED else "") + ")")
        if args.plot:
            plot_track(args.plot, track, config, f"demo_noisy10dB, {mode_text}")
        if args.golden:
            export_golden(args.golden, noisy_samples, config)
            print("golden vectors ->", args.golden)
        if args.play is not None:
            import playback
            playback.play(output, config.fs, args.play)
        return

    if not args.input:
        parser.error("give an input wav or use --demo")
    samples = load_wav(args.input, config.fs)
    output, track = run(samples, config)
    if args.loud is not None:
        try:
            requested_gain = None if args.loud == "auto" else float(args.loud)
        except ValueError:
            parser.error("--loud takes a number of dB, e.g. --loud 12")
        output, applied_gain = make_louder(output, config.fs, requested_gain)
        print(f"output gain {applied_gain:+.1f} dB (peaks soft-limited at -1 dBFS)")
    save_wav(args.output, output, config.fs)
    if args.golden:
        export_golden(args.golden, samples, config)
    summary = summarize(track)
    used_targets = track["target"][track["voiced"].astype(bool) & (track["target"] >= 0)]
    print(f"wrote {args.output}: {mode_text}; {summary['voiced_pct']:.0f}% voiced hops, "
          f"notes used: {sorted({note_name(int(target)) for target in used_targets})}")
    if mode == MODE_FIXED:
        print(f"out of range: {summary['oor_pct']:.1f}% of voiced hops "
              f"(the reachable window is the sung note +-{config.max_shift:.0f} st)")
    if args.display:
        width = 22 if args.display == "oled" else 16
        hops_per_row = max(1, int(round(0.1 * config.fs / config.H)))
        for hop_index in range(0, len(track["n_end"]), hops_per_row):
            info = {key: track[key][hop_index] for key in track}
            line1, line2 = display_lines(info, width)
            print(f"{(track['n_end'][hop_index] - config.denoise_delay) / config.fs:6.2f}s | {line1:<{width}} | {line2:<{width}} |")
    if args.plot:
        plot_track(args.plot, track, config, f"{os.path.basename(args.input)}, {mode_text}")
        print("plot ->", args.plot)
    if args.play is not None:
        import playback
        playback.play(output, config.fs, args.play)


if __name__ == "__main__":
    main()
