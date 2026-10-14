# C port of `autotuner.py`: what was added and why

Written 14 Oct 2026. The C code on the STM32F446RE now runs the same pipeline as `python_prototype/autotuner.py` (including the SNR gate added to both on 14 Oct, §4) (default settings: noise suppressor, LPC source-filter TD-PSOLA with cross-correlation pitch marks, dry crossfade, level match, pause gate, NEAREST and FIXED modes). The mic input path you already had working (`slot_to_s24`, `hop_to_pcm`, `MIC_SHIFT`, `MIC_GAIN_Q8`) is untouched.

---

## 1. Folder layout

```
autotuner_f446re/
├── C_PORT.md                 ← this file
├── Core/Src/main.c           ← CHANGED: 3 lines (include, app_init, app_process_hop)
├── App/
│   ├── Inc/                  ← one header per module (public API + state struct)
│   └── Src/                  ← one .c per module
├── laptop/rx_audio.py        ← CHANGED: sends commands, shows mode/target/sung note
└── pc_test/                  ← NEW: runs the C code on the PC and compares it with Python
                                (not compiled by CubeIDE: it is not a source folder)
```

`App` was already a source folder with `../App/Inc` on the include path, so CubeIDE picks up the new files automatically: **Refresh (F5) the project, then Build.** No CubeMX change and no extra library is needed.

---

## 2. How one 8 ms hop flows through the files

```
main.c  (I2S DMA callback → hop_ready)            unchanged mic code: 24-bit slot → int16 pcm[128]
  └─ app_process_hop(pcm)                          app_main.c
       ├─ uart_cmd_get_line → at_command_parse     commands typed on the laptop (applied at hop boundary)
       ├─ autotuner_process_hop(in, out, &info)    autotuner.c   (Python: AutoTuner.process_hop)
       │    ├─ at_denoise_process                  at_denoise.c  noise suppressor (STFT Wiener)
       │    ├─ at_biquad_run ×2                    at_biquad.c   75 Hz–1 kHz band-pass / 75 Hz high-pass
       │    ├─ at_pitch_detect                     at_pitch.c    FFT autocorrelation → f0, confidence
       │    ├─ at_note_tracker_update              at_notes.c    MIDI, median, hysteresis
       │    ├─ at_range_monitor_update             at_notes.c    fixed-mode out-of-range flag
       │    ├─ at_lpc_analyze                      at_lpc.c      reflection coefficients of the frame
       │    └─ at_psola_push                       at_psola.c    residual → marks → grains → 1/A(z)
       │         └─ at_xfade_run / at_level_run / at_gate_run   at_post.c
       ├─ uart_packet_send(out, f0, target, status)  uart_packet.c (unchanged)
       ├─ update_sung_note / app_screen_draw       app_main.c / app_screen.c   OLED content (5 Hz + after commands)
       └─ oled_flush / oled_poll                   oled.c        one OLED page per hop by I2C DMA
```

---

## 3. Every new file and its purpose

All DSP files start with `at_` (auto-tuner). Each module keeps **all of its state in its own struct** and is only called from one place, so you can change one without touching the others.

| File (.h + .c) | Python counterpart | Purpose | Edit this when you want to… |
|---|---|---|---|
| `at_config` | `class Config` | Every constant and tunable in one place. `#define`s for sizes (fs, N, H, FFT, ring sizes, delays); `at_config_t` + `at_config_default()` for thresholds and switches, with the same names and defaults as the Python `Config`. | change a threshold (`voicing_thresh`, `dn_floor_db`, `gate_depth_db`, `retune_tau_s`, …). Override fields in `app_init()` after `at_config_default()`. |
| `at_fft` | `np.fft.rfft / irfft` | Real FFT 512 / 1024 points, output packed like CMSIS-DSP. Portable code by default; `#define AT_USE_CMSIS_DSP` switches to `arm_rfft_fast_f32` (§7). | speed up the FFT or switch to CMSIS. Nothing else changes, because the packed format is the same. |
| `at_biquad` | `signal.sosfilt` + `design_bandpass` | Biquad cascade (transposed direct form II, same as SciPy) and the two coefficient tables, printed from SciPy. | change fs or the band edges (regenerate the coefficients with the two SciPy lines in the file comment). |
| `at_denoise` | `class NoiseSuppressor` | Spectral noise suppressor: sqrt-Hann STFT, minimum-statistics noise tracking, decision-directed Wiener gain, 3-tap gain smoothing. 24 ms delay. | change how noise is removed. |
| `at_pitch` | `class PitchDetector` | Wiener–Khinchin autocorrelation (1024-pt, zero-padded), window-taper correction, octave guard (shortest peak ≥ 0.9 × best), parabolic interpolation. | change pitch detection. |
| `at_notes` | `hz_to_midi`, `note_name`, `parse_note`, `NoteTracker`, `RangeMonitor` | Note maths and decisions: Hz↔MIDI, note names/parsing, 5-point median, scale snapping (12-bit mask), note hysteresis, out-of-range hysteresis + debounce. | change note snapping, hysteresis, or the OOR rule. |
| `at_lpc` | `levinson`, `step_up`, `AutoTuner._lpc`, synthesis part of `_synthesize` | LPC analysis (Hann window, Gaussian lag window, white-noise correction, Levinson–Durbin), reflection→A(z) step-up, and the all-pole synthesis filter 1/A(z) with its own memory. | change the vocal-tract model (order, lag window). |
| `at_psola` | `class TDPSOLA` (push, `_update_residual`, `_extend_marks_xcorr`, `_place`, `_ola`, `_synthesize`, `_emit`) | The pitch shifter. Each Python method is one `static` function with the same name. Whole-signal arrays became ring buffers (`at_ring.h`). | change pitch marks, grain placement, grain stretch or the overlap-add. |
| `at_post` | `TDPSOLA._emit` crossfade, `_level`, `_gate` | The three output stages, each with its own state: dry/unvoiced crossfade, slow level match, pause gate (hold 150 ms, lookahead 20 ms). | tune what happens between phrases or on consonants. |
| `at_ring.h` | (none) | Two inline helpers: read sample *n* of a ring (negative *n* reads 0, like the zero padding of the Python arrays). | — |
| `autotuner` | `class AutoTuner` | The glue. Owns one instance of every module and makes the per-hop decisions: voicing, target note (mode), shift + clamp + retune smoothing, status bits, pause-gate activity (`gate_activity()`, SNR gate). Public API: `autotuner_init`, `autotuner_process_hop`, `autotuner_set_mode/target/scale`, `at_info_t`. | change the decision logic between modules. |
| `at_display` | `display_lines()` | Port of the Python two-line display text (`Target: C4    [FIXED]` / `Sung: A3  +3 st`). **Not used any more**: the OLED shows the layout in `app_screen` instead. Kept for reference; safe to delete. | — |
| `oled_gfx` | (none: MCU only) | 128×64 framebuffer in the controller's page layout, 5×7 font, text at any scale. No hardware, so it can be tested on the PC. | change the font or add drawing functions. |
| `oled` | (none: MCU only) | I2C driver for SH1106 (default, 1.3") or SSD1306 (0.96"): finds the display at 0x3C/0x3D, init sequence, and non-blocking refresh: one 135-byte page per hop by I2C DMA (a screen takes 64 ms and never blocks the audio). Defines `HAL_I2C_ErrorCallback`. If no display answers, it does nothing and everything else runs. | the screen stays black or is shifted (§4, "OLED"). |
| `app_screen` | (none: MCU only) | What the OLED shows: `mode:`, `target:` (`-` in NEAREST), `sung:` (note being sung, `-` when silent). Labels small, values large and right-aligned. | change the screen layout. |
| `at_command` | `set_mode`, `set_target` | Parses the three text commands `set mode nearest`, `set mode fixed`, `target <note>` (case-insensitive). Pure text, no hardware, so it can be tested on the PC. | add a new command (also add it to `parse_command()` in `rx_audio.py`). |
| `uart_cmd` | (none: MCU only) | USART2 RX: the interrupt stores bytes in a ring, and the main loop takes complete lines. Defines `HAL_UART_RxCpltCallback` and `HAL_UART_ErrorCallback`. | change how commands arrive (e.g. add the B1 button). |
| `app_main` | `run()` (real-time version) | Connects hardware and DSP: int16 ↔ float (÷32768, same scaling as your recorded WAVs), commands, DWT cycle measurement, UART packet, out-of-range statistics (Live Expressions only), the sung note for the OLED (`update_sung_note`, holds the last note 200 ms), OLED refresh. Holds the `autotuner_t` instance (static, ~80 kB). | change what is sent to the laptop, what is measured, or how the out-of-range share is counted (`update_oor_stats`). |

### Changed files

| File | Change |
|---|---|
| `Core/Src/main.c` | `#include "app_main.h"`; `uart_packet_init(&huart2)` → `app_init(&huart2, &hi2c1)`; `uart_packet_send(pcm, 0.0f, 0, 0)` → `app_process_hop(pcm)`. All three are inside `USER CODE` blocks, so CubeMX regeneration keeps them. |
| `laptop/rx_audio.py` | Works as the command line (§4): checks what you type, sends it, and prints `[ok]` when the next packets show the MCU applied it. `--mode / --target` send commands at start. The status line shows mode, target, sung note and "OUT OF RANGE"; on exit it prints the % of voiced fixed-mode hops that were out of range (spec §7). |

`uart_packet.c/.h` are unchanged. The packet now carries the real pitch, target and status byte.

### PC test kit (`pc_test/`)

| File | Purpose |
|---|---|
| `at_pc_main.c` | Runs the C auto-tuner on raw int16 audio on the PC; writes the output (delay removed, aligned like Python `run()`) and a `hops.csv` with the same columns as the Python golden CSV. |
| `make_golden.py` | Runs `autotuner.py` on a WAV and saves `input_q15.raw`, `output_q15.raw`, `hops.csv` (uses Python's `export_golden`). |
| `compare.py` | Compares C and Python: voicing/target/status agreement, pitch difference in cents, output level. |
| `run_test.sh` | Does all three: `sh run_test.sh ../../python_prototype/wav_files/introduce.wav C4`. Needs gcc (MSYS2 or WSL on Windows). Writes `out_c.wav` to listen to. |

**Rule of thumb:** after editing any `at_*.c`, run `run_test.sh` on one recording before flashing. If voicing/target/status agreement drops below ~99 %, the edit changed behaviour.

---

## 4. Using it

**Laptop: `rx_audio.py` is the command line.** There is one USB serial port and it carries the audio, so YAT can't open it at the same time. Instead, `rx_audio.py` plays the audio and takes your commands in the same window:
```
python rx_audio.py --mode fixed --target C4      # or just: python rx_audio.py   (starts in NEAREST)
```
Then type a command and press Enter:

| Command | Effect |
|---|---|
| `set mode nearest` | snap to the closest note (±0.5 st) |
| `set mode fixed` | shift to the fixed target (±12 st, clamped, out-of-range flag) |
| `target <note>` | fixed-mode target: `target C5`, `target F#3`, `target Bb2` (C1..C8; a MIDI number such as `target 60` also works) |

Commands are case-insensitive. The script checks them first (`[error] not a note: H4`), sends them, and prints `[ok] MCU mode is now FIXED` / `[ok] MCU target is now C5` once the packets show the change (the MCU itself never sends text, because the TX line carries the audio packets). A target set while in NEAREST mode is stored and used after `set mode fixed`.

**CubeIDE Live Expressions** (all `volatile`):

| Variable | Meaning |
|---|---|
| `app_cycles_last`, `app_cycles_max` | DWT cycles of one `autotuner_process_hop()`; budget = 1 440 000 (8 ms at 180 MHz) |
| `app_deadline_miss`, `hop_overruns` | hops that took too long / arrived before the previous one was done; both should stay 0 |
| `app_info` | pitch, confidence, sung MIDI, target, shift, alpha, status of the last hop |
| `app_oor_voiced`, `app_oor_count` | fixed mode: voiced hops / out-of-range hops since the last mode or target change (out-of-range share = count / voiced; not shown on the OLED) |
| `oled_present`, `oled_address`, `oled_frames`, `oled_errors` | 1 if the display answered at 0x3C/0x3D; screens sent (should grow by ~5 per second); I2C errors |
| `app_cmd_ok`, `app_cmd_rejected` | command lines accepted / rejected by the MCU |
| `pkt_dropped` | UART packets dropped (should stay 0) |

**OLED (1.3" SH1106 or 0.96" SSD1306, I2C):** wire VCC → 3V3, GND → GND, SCL → **PB8** (D15), SDA → **PB9** (D14). It shows the MCU's own state, so it confirms every command independently of the laptop:

```
mode:        FIXED          mode:      NEAREST
target:         C5          target:          -
sung:           A4          sung:          C#4
```
`sung` = the note being sung: the median-smoothed pitch rounded to the nearest note. Room noise also passes the tuner's voicing test (confidence ≈ 0.6 > 0.5) and its "pitch" jumps around, so the screen only shows a note with confidence ≥ 0.6 that stays the same for 40 ms (`SUNG_CONF_MIN`, `SUNG_STABLE_HOPS` in `app_main.c`). On the recordings in `wav_files/` that shows a note for 92–98 % of hummed or sung hops, while noise alone shows one 0–6 % of the time; other people talking still can, since that really is a pitched voice. It shows `-` when nobody is singing; the last note is held for 200 ms so short consonants and breaths don't blank it. The screen redraws 5 times a second and right after a command.

The controller is chosen in `App/Inc/oled.h` (`OLED_CONTROLLER`, default `OLED_SH1106` for 1.3" modules):
- **screen stays black** but `oled_present` = 1 → it's an SSD1306: set `OLED_SSD1306`;
- **picture shifted 2 px with noise at the right edge** → it's an SH1106: set `OLED_SH1106`;
- `oled_present` = 0 → no answer on I2C: check wiring / power (the module's own pull-ups are normally enough).

**Pause gate: when it opens.** Between phrases the gate turns the output down a further 25 dB. In the default pitch-only mode it opens on a hop that is:

1. **voiced:** pitch confidence > 0.5 and frame level > −40 dBFS (`voicing_thresh`, `rms_floor_dbfs`; shared with the tuner);
2. **not a click:** one 8 ms hop holding > 90 % of the frame energy counts as a snap (`gate_transient_share`);
3. **at least 6 dB above the tracked background floor** (`gate_snr_db`, added 14 Oct 2026 in both Python and C).

Rule 3 was added because steady noise scores about 0.55 pitch confidence after the band-pass and the noise suppressor, so it already passes rule 1. Only the fixed −40 dBFS kept it out, and loud noise (Apple "Bright Noise" at volume 5) opened the gate. The floor tracker rises 10 dB/s on unvoiced hops (it catches up within 1–2 s when the noise gets louder) and 1 dB/s on voiced hops (a long held note does not pull the floor up to itself).

Measured with the Python prototype (C is bit-identical):

| Case | Old gate | SNR gate 6 dB |
|---|---|---|
| white / treble-heavy noise at −20 dBFS, no voice: gate open | ~99 % of the time | ~30 % (short bursts) |
| voice in a quiet room: voice kept | 100 % | 100 % |
| voice + noise at −30 dBFS | 87 % | 86 % |
| voice + noise at −20 dBFS (noise nearly as loud as the voice) | 100 % (open anyway) | 34 % |

Pink/brown (bass-heavy) noise improves only slightly: its slow level swings look like voice to a one-mic gate. That is the limit of a single microphone. `gate_snr_db` sets the trade-off: lower (e.g. 4) keeps more voice in loud noise but opens more on noise alone, higher (e.g. 8) the reverse; 0 restores the old gate. In Python: `--gate-snr 6` (default), `--gate-snr 0` for the old behaviour.

**Latency:** 384 (denoiser) + 612 (shifter lookahead) = 996 samples ≈ **62 ms** on the MCU, the same as Python, plus the laptop's jitter buffer.

---

## 5. How it was verified

1. **Golden vectors:** `autotuner.py` was run on `introduce.wav` and `the_first_capture.wav` (53.6 s), in NEAREST and in FIXED (target C4).
2. **Detection path:** voicing, target note and status byte agree on **100 %** of hops (6 705 hops on the long capture); sung pitch differs by at most **0.01 cent**.
3. **Shifter logic:** a test build of the same C code in double precision, given the same first pitch mark as Python, reproduces the Python output **bit for bit** (all 2 573 pitch marks and 2 423 grains identical, output SNR 217–219 dB on both files and both modes). So every step of the port is algorithmically identical to the prototype.
4. **Real float32 build:** the waveform is not sample-identical to Python, and this is expected. The very first pitch mark is chosen as "largest |residual| in the first period", which falls in digital near-silence (~10⁻¹³). Float32 vs float64 rounding picks a different sample there, and every later mark follows from it, so the grains are cut at a different (equally valid) phase. Measured by what you hear, the float32 C output differs from Python **as much as Python differs from itself** when only its first mark is moved by 5 samples (median output-pitch difference 3–6 cents in both cases, same output level within 0.1 dB).
5. **Cortex-M4 build:** compiles with `-Wall -Wextra -Wdouble-promotion` without warnings in the new code; no double-precision library calls in the image; full firmware (with the OLED driver) links with **107 KB of 128 KB RAM** used (including the 4 KB stack and 1 KB heap).
6. **Speed (emulated Cortex-M4, real singing, fixed mode C4):** about 505 k instructions per hop on average, 570 k worst case. On the real chip that should be roughly 0.65–0.85 M cycles, i.e. **45–60 % of the 8 ms budget**. Confirm on the board with `app_cycles_max`. Breakdown: pitch FFTs 37 %, denoiser 22 %, pitch-mark search 16 %, LPC 10 %, synthesis + output stages 6 %, rest 9 %.

---

## 6. What was not ported (on purpose)

| Python feature | Why not on the MCU |
|---|---|
| `--classic` (plain TD-PSOLA, peak-picked marks, `ola_norm="w"`) | A/B comparison mode for the report; the MCU runs the chosen v2 design. |
| `make_louder` / `--loud` | Offline post-gain with a soft limiter; belongs on the laptop if needed. |
| `--plot`, `--demo`, `--play`, WAV I/O | Laptop-side tools. |
| B1 button | The EXTI15_10 interrupt is not enabled in CubeMX yet. Once it is, call `autotuner_set_mode()` from the main loop on a flag. |

---

## 7. Optional: CMSIS-DSP FFT (about 18 % less CPU)

Tested in the emulator: the same results (pitch identical to 4 decimals), about 415 k instead of 505 k instructions per hop, and about 80 KB more flash.

1. Get CMSIS-DSP (e.g. GitHub `ARM-software/CMSIS-DSP`, v1.16.2) and copy into the project: `Include/`, `PrivateInclude/`, and these sources: `TransformFunctions/arm_rfft_fast_f32.c`, `arm_rfft_fast_init_f32.c`, `arm_cfft_f32.c`, `arm_cfft_init_f32.c`, `arm_cfft_radix8_f32.c`, `arm_bitreversal2.c`, `CommonTables/arm_common_tables.c`, `CommonTables/arm_const_structs.c`.
2. Add both include folders to the include paths, and define `AT_USE_CMSIS_DSP` (and `ARM_MATH_CM4`) in *Preprocessor → Define symbols*.
3. Rebuild. Only `at_fft.c` changes behaviour; nothing else is affected.

---

## 8. Things to keep in mind

- **One instance, static.** `autotuner_t` is ~80 kB; never put it on the stack (4 kB). RAM in use: ~107 KB of 128 KB.
- **Sizes are compile-time.** Changing `AT_FS`, `AT_N` or `AT_H` needs new biquad coefficients, and `AT_H` must stay equal to `HOP` in `main.c`.
- **`HAL_UART_RxCpltCallback` lives in `uart_cmd.c`, `HAL_I2C_ErrorCallback` in `oled.c`.** If another module needs one of them later, forward from there; don't define it twice.
- **Commands take effect at the next hop**, never in the middle of one: the interrupt only buffers bytes.
- **The command words live in two places:** `at_command.c` (MCU) and `parse_command()` in `rx_audio.py` (checks before sending). Change both together.
- **Python is still the reference.** If you change the algorithm, change `autotuner.py` first, then the matching `at_*.c` (the table in §3 tells you which), then run `pc_test/run_test.sh`.
