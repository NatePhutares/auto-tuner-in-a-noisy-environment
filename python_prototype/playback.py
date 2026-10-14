"""Play the OUT audio on a chosen output device (spec section 10).

The MCU only streams audio; where it is heard is decided here on the laptop.

    python autotuner.py --list-devices
    python autotuner.py in.wav --play speaker     # laptop speaker / system default
    python autotuner.py in.wav --play airpods     # name substring match
    python autotuner.py in.wav --play 7           # device index
    python autotuner.py in.wav --play             # numbered menu

Devices are matched by name, not a fixed index, because indices change when
Bluetooth devices connect or disconnect. If the chosen device is missing the
default output (normally the laptop speaker) is used instead, with a warning.
Needs:  pip install sounddevice
"""
from __future__ import annotations

import numpy as np

ALIASES = {"airpods": ["airpods"], "speaker": None}    # None = system default output
BT_HINTS = ("airpods", "bluetooth", "buds", "headset", "hands-free", "wh-", "wf-")


def _sd():
    try:
        import sounddevice as sd
    except ImportError as e:
        raise SystemExit("playback needs sounddevice:  pip install sounddevice") from e
    return sd


def output_devices():
    sd = _sd()
    return [(i, d) for i, d in enumerate(sd.query_devices()) if d["max_output_channels"] > 0]


def print_devices():
    sd = _sd()
    default = sd.default.device[1]
    for i, d in output_devices():
        mark = "*" if i == default else " "
        print(f"{mark}{i:3d}  {d['name']}  ({int(d['default_samplerate'])} Hz)")
    print("* = system default output")


def is_bluetooth(name: str) -> bool:
    n = name.lower()
    return any(h in n for h in BT_HINTS)


def pick_device(spec: str | None):
    """Return (index or None for default, display name)."""
    sd = _sd()
    devs = output_devices()
    if spec == "":                                     # --play with no value: ask
        print_devices()
        spec = input("output device (index or name, Enter = default): ").strip() or "speaker"
    key = (spec or "speaker").strip().lower()
    if key.isdigit() and any(i == int(key) for i, _ in devs):
        i = int(key)
        return i, sd.query_devices(i)["name"]
    subs = ALIASES.get(key, [key])
    if subs is not None:
        for i, d in devs:
            if any(s in d["name"].lower() for s in subs):
                return i, d["name"]
        print(f"warning: no output device matching {spec!r}; using the default output")
    default = sd.default.device[1]
    name = sd.query_devices(default)["name"] if default is not None and default >= 0 else "default"
    return None, name


def play(y, fs: int, spec: str | None = "speaker"):
    """Play y (float, -1..1) at fs on the chosen device; blocks until done."""
    sd = _sd()
    dev, name = pick_device(spec)
    y = np.clip(np.asarray(y, dtype=np.float32), -1.0, 1.0)
    rate = fs
    try:
        sd.check_output_settings(device=dev, samplerate=fs, channels=1)
    except Exception:
        # many devices (Bluetooth especially) reject 16 kHz: upsample on the laptop only
        from scipy.signal import resample_poly
        rate = 48000
        y = resample_poly(y, rate // fs, 1).astype(np.float32) if rate % fs == 0 else \
            resample_poly(y, rate, fs).astype(np.float32)
    bt = is_bluetooth(name)
    lat = "about +150-250 ms extra (Bluetooth), not for live singing" if bt else "no extra latency"
    print(f"playing on: {name} @ {rate} Hz  [{lat}]")
    sd.play(y, rate, device=dev)
    sd.wait()
