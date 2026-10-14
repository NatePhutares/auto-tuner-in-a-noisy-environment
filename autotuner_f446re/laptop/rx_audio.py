"""
rx_audio.py - receive the auto-tuner audio stream from the Nucleo and play it.

Packet (266 bytes, little-endian), one per 8 ms hop:
    0xA5 0x5A | seq u16 | pitch_hz f32 | target u8 | status u8 | 128 x int16 @ 16 kHz

Usage:
    python rx_audio.py                    # auto-detect ST-LINK port, default speaker
    python rx_audio.py --list             # list audio output devices and serial ports
    python rx_audio.py --output airpods   # play on a device whose name contains "airpods"
    python rx_audio.py --output 5         # play on device index 5
    python rx_audio.py --port COM5 --wav take1.wav --seconds 20
    python rx_audio.py --mode fixed --target C4   # set the tuning mode / target at start

While running, type a command and press Enter. What you type appears on the "> " line under the
status line (the once-a-second refresh no longer erases it). The command is checked here, sent to
the MCU, and confirmed from the next packets ([ok] ...):
    set mode nearest
    set mode fixed
    target <note>          e.g. target C5, target F#3, target Bb2   (C1..C8)
Ctrl+C to stop (the WAV is written on exit).

Requires: pip install pyserial sounddevice numpy scipy
"""
import argparse
import collections
import os
import queue
import shutil
import struct
import sys
import threading
import time
import wave

import numpy as np
import serial
import serial.tools.list_ports
import sounddevice as sd
from scipy.signal import firwin, lfilter

FS = 16000
HOP = 128
BAUD = 921600
SYNC = b"\xA5\x5A"
HDR = struct.Struct("<2sHfBB")            # sync, seq, pitch, target, status
PKT_SIZE = HDR.size + 2 * HOP             # 266

NOTE_NAMES = ["C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"]
TARGET_MIN, TARGET_MAX = 24, 108          # C1..C8, same range as the MCU
CONFIRM_TIMEOUT_S = 1.0                   # how long to wait for the packets to show a command took effect
COMMAND_HELP = "set mode nearest | set mode fixed | target <note> (e.g. target C5)"
ST_OUT_OF_RANGE, ST_MODE_FIXED, ST_VOICED = 0x01, 0x02, 0x04

JITTER_TARGET_MS = 80                     # playback starts once this much audio is buffered
JITTER_MAX_MS = 250                       # above this, drop the oldest audio (clock drift)


# ---------------------------------------------------------------- serial port
def find_stlink_port():
    for p in serial.tools.list_ports.comports():
        text = f"{p.description} {p.manufacturer or ''}".lower()
        if "stlink" in text or "st-link" in text or "stmicro" in text:
            return p.device
    return None


# ---------------------------------------------------------------- audio device
def pick_output_device(choice):
    """choice: None (system default), an index, or a name substring like 'airpods'."""
    if choice is None:
        return None
    devices = sd.query_devices()
    if choice.isdigit():
        idx = int(choice)
        if 0 <= idx < len(devices) and devices[idx]["max_output_channels"] > 0:
            return idx
        print(f"[warn] output device {idx} not found, using default speaker")
        return None
    for idx, d in enumerate(devices):
        if d["max_output_channels"] > 0 and choice.lower() in d["name"].lower():
            return idx
    print(f"[warn] no output device matching '{choice}' (not connected?), using default speaker")
    return None


class Upsample3:
    """16 kHz -> 48 kHz for devices that refuse 16 kHz. Streaming FIR with kept state."""
    def __init__(self):
        self.h = firwin(96, 1.0 / 3.0) * 3.0          # cutoff at the old Nyquist, gain 3 for zero-stuffing
        self.zi = np.zeros(len(self.h) - 1)

    def __call__(self, x):
        up = np.zeros(3 * len(x))
        up[::3] = x
        y, self.zi = lfilter(self.h, 1.0, up, zi=self.zi)
        return y


# ---------------------------------------------------------------- jitter buffer
class JitterBuffer:
    def __init__(self):
        self.q = collections.deque()
        self.n = 0
        self.lock = threading.Lock()
        self.started = False
        self.underruns = 0
        self.dropped = 0

    def push(self, samples):
        with self.lock:
            self.q.append(samples)
            self.n += len(samples)
            while self.n > FS * JITTER_MAX_MS // 1000:          # too far behind: drop oldest
                old = self.q.popleft()
                self.n -= len(old)
                self.dropped += len(old)
            if not self.started and self.n >= FS * JITTER_TARGET_MS // 1000:
                self.started = True

    def pull(self, count):
        out = np.zeros(count, dtype=np.float32)
        with self.lock:
            if not self.started:
                return out
            i = 0
            while i < count and self.q:
                chunk = self.q[0]
                take = min(count - i, len(chunk))
                out[i:i + take] = chunk[:take]
                if take == len(chunk):
                    self.q.popleft()
                else:
                    self.q[0] = chunk[take:]
                self.n -= take
                i += take
            if i < count:                                        # ran dry: refill before playing again
                self.underruns += 1
                self.started = False
        return out

    def ms(self):
        with self.lock:
            return 1000.0 * self.n / FS


# ---------------------------------------------------------------- commands to the MCU
def note_name(midi):
    return f"{NOTE_NAMES[midi % 12]}{midi // 12 - 1}" if midi > 0 else "--"


def parse_note(text):
    """'C5', 'f#3', 'Bb2' or a MIDI number -> MIDI int, or None (same rules as at_parse_note on the MCU)."""
    text = text.strip()
    if text.lstrip("-").isdigit():
        return int(text)
    if not text:
        return None
    text = text[0].upper() + text[1:]
    accidental = text[1] if len(text) > 1 and text[1] in "#b" else ""
    name, octave = text[0] + accidental, text[1 + len(accidental):]
    name = {"Db": "C#", "Eb": "D#", "Gb": "F#", "Ab": "G#", "Bb": "A#"}.get(name, name)
    if name not in NOTE_NAMES or not octave.lstrip("-").isdigit():
        return None
    return NOTE_NAMES.index(name) + 12 * (int(octave) + 1)


def parse_command(line):
    """Returns (text to send, what to expect) or raises ValueError with a message for the user.
    what to expect: ("mode", fixed?) or ("target", midi)."""
    words = line.split()
    low = [w.lower() for w in words]
    if len(words) == 3 and low[0] == "set" and low[1] == "mode" and low[2] in ("nearest", "fixed"):
        return f"set mode {low[2]}", ("mode", low[2] == "fixed")
    if len(words) == 2 and low[0] == "target":
        midi = parse_note(words[1])
        if midi is None:
            raise ValueError(f"not a note: {words[1]} (use e.g. C5, F#3, Bb2)")
        if not TARGET_MIN <= midi <= TARGET_MAX:
            raise ValueError(f"{words[1]} is outside {note_name(TARGET_MIN)}..{note_name(TARGET_MAX)}")
        return f"target {note_name(midi)}", ("target", midi)
    raise ValueError(f"unknown command: {line}   (commands: {COMMAND_HELP})")


def confirm_pending(pending, target, status, say):
    """Drops the expectations this packet confirms (prints [ok]) or that timed out (prints a warning)."""
    fixed = bool(status & ST_MODE_FIXED)
    still = []
    for (kind, value), deadline in pending:
        if kind == "mode" and fixed == value:
            say(f"[ok] MCU mode is now {'FIXED' if fixed else 'NEAREST'}")
        elif kind == "target" and fixed and target == value:
            say(f"[ok] MCU target is now {note_name(value)}")
        elif kind == "target" and not fixed and time.time() > deadline:
            say(f"[ok?] target {note_name(value)} sent; it is used once you 'set mode fixed'")
        elif time.time() > deadline:
            say(f"[warn] MCU did not confirm '{kind} {value}' within {CONFIRM_TIMEOUT_S:.0f} s "
                  "(is the new firmware flashed?)")
        else:
            still.append(((kind, value), deadline))
    return still


class Console:
    """A status line and a "> " prompt line under it, both redrawn in place.

    Windows: keys are read one by one (msvcrt), so the refresh redraws what you have typed instead of
    erasing it. Other systems: the terminal echoes the typing itself and lines come from stdin.
    Messages ([sent], [ok], ...) are printed above the status line.
    """

    def __init__(self):
        self.status = ""
        self.typed = ""
        self.drawn = False                                   # status + prompt lines are on screen
        self.lines = queue.Queue()
        self.windows = os.name == "nt"
        if self.windows:
            import msvcrt
            self.msvcrt = msvcrt
            os.system("")                                  # lets the Windows console handle \x1b codes
        else:
            def reader():
                for line in sys.stdin:
                    if line.strip():
                        self.lines.put(line.strip())
            threading.Thread(target=reader, daemon=True).start()

    def poll(self):
        """Returns the command lines completed since the last call."""
        if self.windows:
            changed = False
            while self.msvcrt.kbhit():
                ch = self.msvcrt.getwch()
                if ch in ("\x00", "\xe0"):                 # arrow / function key: ignore it
                    self.msvcrt.getwch()
                elif ch in ("\r", "\n"):
                    if self.typed.strip():
                        self.lines.put(self.typed.strip())
                    self.typed = ""
                elif ch == "\x08":                          # Backspace
                    self.typed = self.typed[:-1]
                elif ch == "\x1b":                          # Esc clears the line
                    self.typed = ""
                elif ch == "\x03":                          # Ctrl+C
                    raise KeyboardInterrupt
                elif ch.isprintable():
                    self.typed += ch
                changed = True
            if changed:
                self.redraw()
        out = []
        while not self.lines.empty():
            out.append(self.lines.get())
        return out

    def set_status(self, text):
        self.status = text
        self.redraw()

    def say(self, text):
        """Prints a message above the status line."""
        if self.windows and self.drawn:
            sys.stdout.write("\r\x1b[2K\x1b[1A\r\x1b[2K")   # clear the prompt line and the status line
            self.drawn = False
        else:
            sys.stdout.write("\r\x1b[2K")
        sys.stdout.write(text + "\n")
        self.redraw()

    def redraw(self):
        width = max(20, shutil.get_terminal_size((100, 20)).columns - 1)   # never wrap: lines are redrawn in place
        status = self.status[:width]
        if not self.windows:
            sys.stdout.write("\r\x1b[2K" + status)
            sys.stdout.flush()
            return
        prompt = "> " + self.typed
        if len(prompt) > width:                          # very long input: show its end
            prompt = "> ..." + self.typed[-(width - 5):]
        if self.drawn:                                   # cursor is on the prompt line: go back up one
            sys.stdout.write("\r\x1b[2K\x1b[1A")
        sys.stdout.write("\r\x1b[2K" + status + "\n\x1b[2K" + prompt)
        sys.stdout.flush()
        self.drawn = True

    def close(self):
        sys.stdout.write("\n")


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port, e.g. COM5 (default: auto-detect ST-LINK)")
    ap.add_argument("--output", help="playback device: index or name substring (speaker, airpods, ...)")
    ap.add_argument("--wav", default="capture.wav", help="WAV file to save (default capture.wav)")
    ap.add_argument("--seconds", type=float, default=0, help="stop after N seconds (0 = until Ctrl+C)")
    ap.add_argument("--no-play", action="store_true", help="only record, no playback")
    ap.add_argument("--list", action="store_true", help="list audio devices and serial ports, then exit")
    ap.add_argument("--mode", choices=["nearest", "fixed"], help="tuning mode sent to the MCU at start")
    ap.add_argument("--target", help="fixed-mode target note sent at start, e.g. C4 or F#3")
    args = ap.parse_args()

    if args.list:
        print(sd.query_devices())
        print("\nSerial ports:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device:8s} {p.description}")
        return

    port = args.port or find_stlink_port()
    if not port:
        sys.exit("No ST-LINK serial port found. Plug in the Nucleo or pass --port COMx.")

    jb = JitterBuffer()
    stream = None
    if not args.no_play:
        dev = pick_output_device(args.output)
        rate, up = FS, None
        try:
            sd.check_output_settings(device=dev, samplerate=FS, channels=1, dtype="float32")
        except Exception:
            rate, up = 48000, Upsample3()                     # device can't do 16 kHz: upsample x3

        def callback(outdata, frames, t, status):
            n_src = frames // 3 if up else frames
            x = jb.pull(n_src)
            outdata[:, 0] = up(x)[:frames] if up else x

        stream = sd.OutputStream(device=dev, samplerate=rate, channels=1, dtype="float32",
                                 blocksize=3 * HOP if up else HOP, callback=callback)
        name = sd.query_devices(stream.device)["name"]
        print(f"Playback : {name} @ {rate} Hz  (output latency ~{1000 * stream.latency:.0f} ms)")

    ser = serial.Serial(port, BAUD, timeout=0.1)
    ser.reset_input_buffer()
    print(f"Serial   : {port} @ {BAUD}")
    console = None
    startup = []
    if args.target:
        startup.append(f"target {args.target}")
    if args.mode:
        startup.append(f"set mode {args.mode}")
    console = Console()
    print(f"Commands : {COMMAND_HELP}  (type + Enter, Esc clears)")
    print(f"Recording: {args.wav}   (Ctrl+C to stop)\n")

    recorded = []
    buf = bytearray()
    last_seq = None
    n_pkt = lost = resync = 0
    sec_pkt = 0
    peak = 0.0
    last_pitch, last_target, last_status = 0.0, 0, 0
    voiced_fixed = oor_fixed = 0
    pending = []                                          # [(expectation, deadline)] waiting for packets
    t_start = t_stat = time.time()
    if stream:
        stream.start()

    try:
        while True:
            for line in startup + console.poll():
                console.say(f"> {line}")
                try:
                    text, expect = parse_command(line)
                except ValueError as error:
                    console.say(f"[error] {error}")
                    continue
                ser.write((text + "\r\n").encode("ascii"))
                console.say(f"[sent] {text}")
                pending.append((expect, time.time() + CONFIRM_TIMEOUT_S))
            startup = []
            buf += ser.read(ser.in_waiting or 1)
            while True:
                i = buf.find(SYNC)
                if i < 0:
                    del buf[:-1]                                  # keep a possible half sync byte
                    break
                if i > 0:
                    del buf[:i]
                    resync += 1
                if len(buf) < PKT_SIZE:
                    break
                _, seq, pitch, target, status = HDR.unpack_from(buf, 0)
                pcm = np.frombuffer(bytes(buf[HDR.size:PKT_SIZE]), dtype="<i2")
                # a valid packet is followed by the next sync word (or the buffer ends)
                if len(buf) >= PKT_SIZE + 2 and buf[PKT_SIZE:PKT_SIZE + 2] != SYNC:
                    del buf[:2]                                   # false sync inside audio data
                    resync += 1
                    continue
                del buf[:PKT_SIZE]

                if last_seq is not None:
                    lost += (seq - last_seq - 1) & 0xFFFF
                last_seq = seq
                n_pkt += 1
                sec_pkt += 1

                last_pitch, last_target, last_status = pitch, target, status
                pending = confirm_pending(pending, target, status, console.say)
                if status & ST_MODE_FIXED and status & ST_VOICED:
                    voiced_fixed += 1
                    oor_fixed += bool(status & ST_OUT_OF_RANGE)

                x = pcm.astype(np.float32) / 32768.0
                peak = max(peak, float(np.max(np.abs(x))))
                recorded.append(pcm.copy())
                jb.push(x)

            now = time.time()
            if now - t_stat >= 1.0:
                db = 20 * np.log10(peak) if peak > 0 else -120.0
                mode = "FIXED" if last_status & ST_MODE_FIXED else "NEAREST"
                sung = (note_name(int(round(69 + 12 * np.log2(last_pitch / 440.0))))
                        if last_status & ST_VOICED and last_pitch > 0 else "--")
                flag = " OUT OF RANGE" if last_status & ST_OUT_OF_RANGE else ""
                # most useful first: the line is cut on the right to leave room for the "> " prompt
                target_text = note_name(last_target) if last_status & ST_MODE_FIXED else "-"
                console.set_status(f"{mode} target {target_text} sung {sung}{flag} | {sec_pkt} pkt/s lost {lost} "
                                   f"| peak {db:4.0f} dB | buf {jb.ms():3.0f} ms | underruns {jb.underruns}")
                sec_pkt, peak, t_stat = 0, 0.0, now
            if args.seconds and now - t_start >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if console is not None:
            console.close()
        if stream:
            stream.stop()
            stream.close()
        ser.close()
        if recorded:
            data = np.concatenate(recorded)
            with wave.open(args.wav, "wb") as w:
                w.setnchannels(1)
                w.setsampwidth(2)
                w.setframerate(FS)
                w.writeframes(data.astype("<i2").tobytes())
            print(f"\nSaved {len(data) / FS:.1f} s to {args.wav}  "
                  f"({n_pkt} packets, {lost} lost, {resync} resyncs)")
            if voiced_fixed:
                print(f"Fixed mode: {100.0 * oor_fixed / voiced_fixed:.1f} % of voiced hops were out of range")
        else:
            print("\nNo packets received. Check that the board is running and the port is right.")


if __name__ == "__main__":
    main()
