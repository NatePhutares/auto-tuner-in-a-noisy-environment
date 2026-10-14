# CubeMX / CubeIDE Configuration — autotuner_f446re

Hardware configuration for **Auto-Tuner in a Noisy Environment** on the **NUCLEO-F446RE**.
Every value below follows from the pipeline design: **fs = 16 kHz, frame N = 512, hop H = 128 samples (8 ms), SYSCLK = 180 MHz**.

The project was created from the Board Selector (NUCLEO-F446RE) with *"Initialize all peripherals with their default Mode" = No*. The "Default" column shows what that baseline `.ioc` contains.

---

## 1. Summary

| # | Block | Setting | Default (baseline) | Required | Reason |
|---|---|---|---|---|---|
| 1 | RCC | HSE | Disabled | **BYPASS Clock Source** (8 MHz from ST-LINK MCO) | Gives an accurate sample clock (see §2) |
| 2 | Clock | SYSCLK / HCLK | 84 MHz (HSI) | **180 MHz** | Latency budget: 8 ms × 180 MHz = 1.44 M cycles per hop |
| 3 | PLLI2S | I2S clock | /16 ×192 /2 | **/4 ×128 /2 → 128 MHz** | Gives **exactly 16 000 Hz** (0.0 % error) |
| 4 | I2S2 | INMP441 mic | Off | **Half-Duplex Master, Receive, Philips, 24-bit data / 32-bit frame, 16 kHz** | Digital mic input |
| 5 | DMA1 Stream 3 | SPI2_RX (I2S2) → RAM | Off | **Circular, half-word, High** | Ping-pong buffer, one hop per half |
| 6 | USART2 | ST-LINK VCP | Pins only | **Asynchronous, 921600 8N1** | Stream audio to laptop |
| 7 | DMA1 Stream 6 | USART2_TX | Off | **Normal, byte** | Packet sent in background |
| 8 | I2C1 | SSD1306 OLED | Off | **PB8 SCL / PB9 SDA, 400 kHz** | Display target/sung note |
| 9 | DMA1 Stream 7 | I2C1_TX | Off | **Normal, byte** | Non-blocking OLED refresh |
| 10 | NVIC | Priority group | Group 0 (no pre-emption) | **4 bits pre-emption** | Lets the audio DMA IRQ pre-empt others |
| 11 | Project | Stack / Heap | 0x400 / 0x200 | **0x1000 / 0x400** | DSP locals, `snprintf` |
| 12 | CubeIDE | Optimization (Debug) | -O0 | **-O2** | Realistic cycle counts |
| 13 | SYS | Debug | **Disable** | **Trace Asynchronous Sw** | Keep SWD flashing/debug + SWO |

> **Status (13 Oct 2026):** all items applied in CubeMX / CubeIDE and code generated; generated `main.c` / `stm32f4xx_hal_msp.c` checked against this table. Clock tree verified on hardware (LD2 blink).
>
> **Input changed from analog ADC to I2S** because the microphone is an INMP441 (digital I2S). The earlier ADC1 + TIM3 version is kept as `autotuner_f446re.ioc.adc_version` / `main.c.adc_version`; the very first baseline is `autotuner_f446re.ioc.orig`.

---

## 2. Clock (System Core → RCC, then Clock Configuration tab)

**RCC → High Speed Clock (HSE): `BYPASS Clock Source`**

On the Nucleo-64 the 8 MHz HSE comes from the ST-LINK's MCO output by default, not from a crystal, so it must be *Bypass*, not *Crystal/Ceramic Resonator*.

Why not keep HSI? HSI is trimmed to about ±1 % at 25 °C and drifts further with temperature. The sample rate comes from this clock, so a 1 % error moves every detected pitch by 1200·log2(1.01) ≈ **17 cents**. That is a large share of the ±50-cent accuracy window used in evaluation.

| Clock Configuration field | Value |
|---|---|
| PLL Source Mux | HSE |
| Input frequency | 8 MHz |
| /M | 4 → VCO input 2 MHz |
| ×N | 180 → VCO output 360 MHz |
| /P | 2 → **PLLCLK 180 MHz** |
| /Q, /R | leave default (unused) |
| System Clock Mux | PLLCLK |
| **SYSCLK / HCLK** | **180 MHz** |
| AHB prescaler | /1 |
| APB1 prescaler | /4 → PCLK1 45 MHz, **APB1 timer clock 90 MHz** |
| APB2 prescaler | /2 → PCLK2 90 MHz, APB2 timer clock 180 MHz |

Shortcut: type `180` in the HCLK box and press Enter; CubeMX solves the PLL. Check the result matches the table.

At 180 MHz, CubeMX automatically sets **Power Regulator Voltage Scale 1 + Over-Drive** and **Flash latency 5 WS**. Check these in RCC → Parameter Settings.

> If the board hangs at start-up after this change (stuck in `HAL_RCC_OscConfig` → `Error_Handler`), the MCO clock isn't reaching PH0. Fall back to HSI (PLL source HSI, /M 16, ×N 360, /P 2 = 180 MHz) and state the HSI accuracy as a limitation.

---

## 3. Audio input — INMP441 on I2S2 (Multimedia → I2S2)

The INMP441 is a digital MEMS microphone: it outputs 24-bit two's-complement samples over I2S, so it does **not** use the ADC, needs no analog anti-alias filter (the mic's internal ΣΔ decimator does that) and has no AGC.

Mode: **Half-Duplex Master** (Master Clock Output off).

| Parameter | Value |
|---|---|
| Transmission Mode | **Mode Master Receive** |
| Communication Standard | I2S Philips |
| Data and Frame Format | **24 Bits Data on 32 Bits Frame** |
| Selected Audio Frequency | **16 kHz** |
| Real Audio Frequency | **16.0 kHz, error 0.0 %** (needs the PLLI2S setting below) |
| Clock Polarity | Low |

### Pins and wiring

| INMP441 pin | Signal | Nucleo pin | Header |
|---|---|---|---|
| VDD | 3.3 V | 3V3 | CN6 |
| GND | GND | GND | CN6 |
| L/R | channel select | **GND** → mic sends in the **left** slot | — |
| WS | I2S2_WS | **PB12** | CN10 (morpho) |
| SCK | I2S2_CK | **PB10** | D6 (CN9) |
| SD (labelled SO on some boards) | I2S2_SD | **PC1** | A4 (CN8) |

### I2S clock (Clock Configuration tab)

fs = I2SCLK / (64 × (2·I2SDIV + ODD)) for a 32-bit stereo frame. With **I2SCLK = 128 MHz**, 2·I2SDIV + ODD = 125 (DIV 62, ODD 1) gives **16 000 Hz exactly**.

| Field | Value |
|---|---|
| PLLI2SM | **/4** (8 MHz HSE → 2 MHz) |
| PLLI2SN | **×128** → VCO 256 MHz |
| PLLI2SR | **/2** → **128 MHz** |
| I2S_APB1 Clock Mux | **PLLI2SR** |

With the default (/16 ×192 /2) CubeMX gives 15.957 kHz (−0.26 % ≈ −4.5 cents on every detected pitch).
Tip: right-click HCLK → **Lock** before typing `128` in "To I2S_APB1 CLK", otherwise the solver also changes the main PLL.

### DMA (I2S2 → DMA Settings → Add)

| Request | Stream | Direction | Mode | Priority | Width | Memory increment |
|---|---|---|---|---|---|---|
| SPI2_RX | **DMA1 Stream 3** | Peripheral To Memory | **Circular** | **High** | Half Word / Half Word | on |

### Buffer layout in code

Each stereo frame = 2 slots (L, R) × 32 bits = 4 half-words, MSB half first. One hop of 128 samples = **512 half-words**; the circular buffer holds two hops (`uint16_t i2s_buf[1024]`).

```c
HAL_I2S_Receive_DMA(&hi2s2, i2s_buf, 512);   // Size counts 32-bit slots for 24/32-bit formats
int32_t s = (int32_t)(((uint32_t)p[0] << 16) | p[1]) >> 8;   // left slot -> signed 24-bit
```

`HAL_I2S_RxHalfCpltCallback` delivers the first hop and `HAL_I2S_RxCpltCallback` the second, so each callback is exactly one 8 ms hop.

---

## 4. ADC1 + TIM3 (removed)

The first version sampled an analog mic on PA0 with ADC1 triggered by TIM3 TRGO at 16 kHz (PSC 0, ARR 5624, DMA2 Stream 0). Both peripherals are now **disabled**; the settings are preserved in `autotuner_f446re.ioc.adc_version` if an analog mic is used later.

---

## 5. USART2 — stream to laptop (Connectivity → USART2)

Mode: **Asynchronous**. Pins **PA2 (TX) / PA3 (RX)** are already reserved by the board config and wired to the ST-LINK VCP.

| Parameter | Value |
|---|---|
| Baud Rate | **921600** |
| Word Length | 8 bits |
| Parity | None |
| Stop Bits | 1 |
| Data Direction | Receive and Transmit |
| Over Sampling | 16 samples |
| USART2 global interrupt | **Enabled** (TX-complete callback, RX commands) |

Real baud from PCLK1 = 45 MHz: 918 367 baud, **−0.35 %** error (fine; UART tolerates about ±2 %).
Bandwidth check: packet ≈ 266 bytes (2 sync + 2 seq + 4 pitch + 1 target + 1 status + 128 × 2 samples) every 8 ms → 332.5 kbaud on the wire = **36 % of the link**.

### DMA (USART2 → DMA Settings → Add)

| Request | Stream | Direction | Mode | Priority | Width | Memory increment |
|---|---|---|---|---|---|---|
| USART2_TX | **DMA1 Stream 6** | Memory To Peripheral | **Normal** | Medium | Byte / Byte | on |

The ping-pong behaviour is done in code: two packet buffers, `HAL_UART_Transmit_DMA` on one while the other is filled.
RX (mode/target commands from the laptop) uses interrupts: `HAL_UART_Receive_IT` with a 1-byte buffer.

---

## 6. I2C1 — SSD1306 OLED (Connectivity → I2C1)

Mode: **I2C**. Move the pins to **PB8 = SCL (D15)** and **PB9 = SDA (D14)** (Ctrl-click the pin in the pinout view). CubeMX may default to PB6/PB7, which also works, but PB8/PB9 are the Arduino I2C header.

| Parameter | Value |
|---|---|
| I2C Speed Mode | **Fast Mode** |
| I2C Clock Speed | **400000 Hz** |
| Fast Mode Duty Cycle | Tlow/Thigh = 2 |
| Addressing | 7-bit (SSD1306 = 0x3C, i.e. `0x78` as HAL 8-bit address) |
| I2C1 event interrupt | **Enabled** |
| I2C1 error interrupt | **Enabled** |

### DMA (I2C1 → DMA Settings → Add)

| Request | Stream | Direction | Mode | Priority | Width |
|---|---|---|---|---|---|
| I2C1_TX | **DMA1 Stream 7** | Memory To Peripheral | Normal | Low | Byte / Byte |

Why DMA: a full 128 × 64 frame (1024 bytes) takes ≈ 26 ms at 400 kHz, i.e. longer than three hops. A blocking write from the main loop would miss the 8 ms deadline. With DMA the transfer runs in the background, refreshed at 10–20 Hz.

> I2C1_TX can also map to DMA1 Stream 6, which USART2_TX already uses. If CubeMX picks Stream 6 for I2C1_TX, change it to **Stream 7**. Only the streams offered in the dropdown are valid; if a stream above isn't offered, take the free one that is.

Most SSD1306 modules have their own 4.7 kΩ pull-ups. If yours doesn't, add them to 3.3 V.

---

## 7. NVIC (System Core → NVIC)

| Setting | Value |
|---|---|
| **Priority Group** | **4 bits for pre-emption priority, 0 bits for subpriority** |

The baseline uses Group 0 (`HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0)` in `stm32f4xx_hal_msp.c`), which means no interrupt can pre-empt another. The audio DMA interrupt is the real-time one, so it must be able to pre-empt the others.

| Interrupt | Enabled | Pre-emption priority |
|---|---|---|
| Time base: System tick timer | yes | 0 (keep default) |
| **DMA1 stream3 global interrupt** (I2S2 RX) | yes | **1** (highest of ours) |
| DMA1 stream6 global interrupt (USART2_TX) | yes | 5 |
| USART2 global interrupt | yes | 5 |
| DMA1 stream7 global interrupt (I2C1_TX) | yes | 10 |
| I2C1 event / error interrupt | yes | 10 |

Keep the I2S callbacks short (copy/flag only). The DSP for each hop runs in the main loop, which is why the OLED must never block it.

---

## 8. SYS (System Core → SYS)

| Parameter | Value |
|---|---|
| Debug | **Trace Asynchronous Sw** (PA13 SWDIO, PA14 SWCLK, PB3 SWO). The baseline had this set to *Disable* even though the pins were labelled. |
| Timebase Source | SysTick |

Keep Debug enabled, or the board can't be re-flashed without holding reset.

---

## 9. GPIO (already configured, used as-is)

| Pin | Label | Use |
|---|---|---|
| PA5 | LD2 [Green Led] | Debug: toggle per hop / set on deadline overrun (visible on a scope) |
| PC13 | B1 [Blue PushButton] | Optional: switch NEAREST/FIXED mode. Requires enabling **EXTI line[15:10] interrupts** in NVIC |

---

## 10. Project Manager

| Tab | Parameter | Value |
|---|---|---|
| Project | Project Name | `autotuner_f446re` |
| Project | Toolchain / IDE | STM32CubeIDE (Generate Under Root ticked) |
| Project | Minimum Heap Size | **0x400** |
| Project | Minimum Stack Size | **0x1000** |
| Code Generator | Copy only the necessary library files | ticked (default) |
| Code Generator | Keep User Code when re-generating | ticked (default) |
| Code Generator | Generate peripheral initialization as a pair of .c/.h files | optional (keeps `main.c` short) |

---

## 11. CubeIDE project settings (not stored in the .ioc)

`Project → Properties → C/C++ Build → Settings` (apply to both Debug and Release):

| Location | Setting | Value |
|---|---|---|
| MCU Settings | Floating-point unit / ABI | FPv4-SP-D16 / Hard (already set) |
| MCU Settings | Use float with printf from newlib-nano | ticked if you `snprintf("%f")` for the OLED |
| MCU GCC Compiler → Optimization | Optimization level | **-O2** (Debug default is -O0, which inflates DWT cycle counts several times) |
| MCU GCC Compiler → Preprocessor | Define symbols | add **`ARM_MATH_CM4`** (needed by CMSIS-DSP) |
| MCU GCC Compiler → Include paths | add | `../App/Inc`, CMSIS-DSP `Include` folder |
| MCU GCC Compiler → Miscellaneous | Other flags | **`-Wdouble-promotion`** (catches accidental `double` math) |
| MCU GCC Linker → Libraries | Libraries / search path | `arm_cortexM4lf_math` + its folder (if using the prebuilt CMSIS-DSP library) |

Source folder: `.cproject` only lists `Core` and `Drivers` as source locations, so add `App` too: right-click `App` → *New → Source Folder*, or `Properties → C/C++ General → Paths and Symbols → Source Location → Add Folder`.

If you use SWO/ITM `printf`: `Debug Configurations → Debugger → Serial Wire Viewer → Core Clock = 180 MHz` (it was 84 MHz).

---

## 12. Constants in code that must match this configuration

| Constant | Value | Depends on |
|---|---|---|
| `FS` | 16000 | PLLI2S 128 MHz / (64 × 125) |
| `HOP` | 128 | samples per half of the I2S DMA buffer |
| `I2S_HALF` | 512 | half-words per hop (128 × 2 slots × 2) |
| I2S buffer | 1024 half-words | 2 × I2S_HALF (circular ping-pong) |
| `FRAME_N` | 512 | design |
| `FFT_N` | 1024 | 2 × FRAME_N (zero-pad, linear autocorrelation) |
| `TAU_MIN`, `TAU_MAX` | 16, 200 | fs / 1000 Hz, fs / 80 Hz |
| `CPU_HZ` | 180 000 000 | clock tree (DWT cycles → µs) |
| `HOP_CYCLES` | 1 440 000 | 8 ms × 180 MHz (real-time deadline) |
| `UART_BAUD` | 921600 | USART2; laptop `pyserial` uses the same |
| `OLED_ADDR` | 0x78 | SSD1306 0x3C << 1 |

---

## 13. Check after editing (before Generate Code)

1. Pinout view: no red or orange pins; PC1/PB10/PB12 = I2S2 SD/CK/WS, PA2/PA3 = USART2, PB8/PB9 = I2C1.
2. Clock Configuration: no red fields; HCLK 180 MHz, APB1 45 MHz, APB2 90 MHz, I2S_APB1 128 MHz.
3. DMA tab: three distinct streams — DMA1 S3 (SPI2_RX / I2S2), DMA1 S6 (USART2_TX), DMA1 S7 (I2C1_TX).
4. NVIC: priority group 4 bits, I2S DMA (DMA1 stream3) at priority 1.
5. Generate Code, build, then confirm on hardware:
   - `hops_per_sec` reads **125** in Live Expressions (125 × 128 = 16 000 samples/s);
   - DWT cycle count per hop stays well under 1 440 000.
