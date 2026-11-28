# AMT132S-V Encoder Driver

Reads a Same Sky AMT132S-V incremental encoder on an STM32 and reports position,
turns, joint angle, speed and acceleration, with Z-index checking that catches
missed or extra counts. C, STM32 HAL, STM32CubeIDE.

Target: **horizontal fixture, NUCLEO-G474RE** (bench-tested earlier on a NUCLEO-F446RE).

| File | What it is |
|---|---|
| `encoder.h` | Settings, encoder state, documented API |
| `encoder.c` | Driver |
| `main_snippets.c` | Code to paste into `main.c` (readable status output) |
| `main_snippets_csv.c` | Blocks 2 and 3 for CSV output (graphing/logging) |
| `plot_encoder.py` | Live graphs + CSV logger (Python) |
| `tests/` | PC unit tests for the driver (no hardware needed) |

---

## 1. Outputs

| Quantity | Function | Units |
|---|---|---|
| Position | `Encoder_GetPosition` | counts, 64-bit (8192 per motor turn) |
| Turns | `Encoder_GetTurns` | whole motor turns, rounded down (−1 = between −1 and 0) |
| Motor angle in turn | `Encoder_GetAngleDeg` | deg, 0 to <360 |
| Joint angle | `Encoder_GetJointAngleDeg` | deg since start/zero, not wrapped |
| Speed | `Encoder_GetRPM`, `Encoder_GetJointSpeedDegS` | RPM, deg/s |
| Acceleration | `Encoder_GetJointAccelDegS2` | deg/s² |
| Index check | `Encoder_GetIndexCount`, `Encoder_GetIndexErrors`, `Encoder_GetIndexErrorSum` | pulses, errors, net counts gained/lost |

Joint values use `ENC_GEAR_RATIO` (currently 1, so they equal motor values).

**Resolution:** 2048 pulses/rev × 4 edges (A and B, rising and falling) = **8192 counts/rev**,
0.044° per count.

---

## 2. Encoder connector (AMT132S, JST ZPDR-18V-S mating connector)

| Pin | Signal | Use |
|---|---|---|
| 4 | GND | Board GND |
| 6 | +5 V | Board 5 V (4.5 to 5.5 V) |
| 8 | A | Encoder timer CH1 |
| 10 | B | Encoder timer CH2 |
| 12 | Z (index) | PD2: one pulse per motor turn at a fixed spot |
| 1 | RX_ENC (UART in) | Configuration only, 115200 8N1. ASCII `0` sets the zero/index position (non-volatile), `Q` restarts the encoder |
| 2 | TX_ENC (UART out) | Configuration only |
| 14 | MCLR | Reprogramming reset. **Never connect** |
| 3, 5, 7, 9, 11, 13, 15–17 | — | Not used on the S version (A−/B−/Z− exist only on the differential Q version) |
| 18 | Noise GND | Not connected internally |

Outputs are **5 V**: every MCU pin used must be 5 V tolerant (FT in the datasheet pin table).
Label the cable wires and tape off unused ones.

---

## 3. Fixture setup (NUCLEO-G474RE, CubeMX)

**Encoder timer: TIM3** (16-bit)
- Combined Channels: Encoder Mode, **TI1 and TI2**
- Prescaler 0, Counter Period **65535**
- Input Filter: ~8 on both channels
- `encoder.h`: `ENC_TIMER_IS_32BIT 0`
- `Encoder_Init` checks the Counter Period matches this setting and calls
  `Error_Handler()` if not, so a mismatch stops the board at startup instead of
  giving wrong positions at rollover.

**Z index: PD2**
- GPIO_EXTI2, **rising edge**, **pull-down** (so an unplugged Z wire can't trigger false pulses)
- User label `ENC_Z` (gives `ENC_Z_Pin` used in `main_snippets.c`)
- NVIC: enable EXTI line 2 interrupt

**Sample timer: TIM6**
- Prescaler = APB1 timer clock (MHz) − 1, e.g. 169 at 170 MHz → 1 MHz tick
- Counter Period 9999 → 100 Hz (999 → 1 kHz, then set `ENC_SAMPLE_HZ 1000u`)
- NVIC: enable TIM6 interrupt

**Serial: LPUART1** (ST-LINK virtual COM port), 115200 8N1

**Remove the BSP LED2 init:** LED2 is on PA5, the knee encoder SPI clock on the fixture.
Disable it in CubeMX's BSP settings and regenerate. Deleting `BSP_LED_Init(LED2)` by
hand doesn't stick: it's outside the USER CODE markers, so regeneration puts it back.

**Startup:** keep the motor stopped until `Encoder_Init` returns (the encoder needs a still
shaft for 200 ms). Starting while moving also gives the acceleration filter a large
start-up transient for its first ~0.2 s.

Then: copy `encoder.h` → `Core/Inc`, `encoder.c` → `Core/Src`, paste `main_snippets.c`
into `main.c`, build, run, and open the COM port at 115200 baud.

---

## 4. Z-index checking

Every Z pulse must land a whole number of turns (a multiple of 8192 counts) from the first
pulse in the same direction (the edge lands on a different count in each direction, so
each has its own reference). If not, counts were missed or noise added extra ones:
`index_errors` increments, and `index_last_err` / `index_err_sum` record how many
counts and which way. The references then re-align, so one glitch is reported once.

- A single-count error is caught up to ~2400 RPM. Above that, the interrupt delay
  (`ENC_INDEX_LATENCY_US`) means the check allows a few counts of slack.
- If occasional false ±1 errors appear at low speed, set `ENC_INDEX_TOL_COUNTS 1`.
- **Reading errors:** noise on the Z wire (Z has no input filter) shows up as two errors
  that cancel: `index_err_sum` returns to its previous value. A real slip (lost or
  extra A/B counts) shows up as one error and a lasting change in `index_err_sum`.
  Known limitation: the driver doesn't yet tell these apart itself. If Z noise appears
  on the fixture, the planned fix is to confirm each mismatch with the next pulse
  before re-aligning.
- Possible upgrade: the G474 timers have a hardware encoder-index feature, and PD2 is
  TIM3_ETR, so the check could be done exactly in hardware at any speed.

---

## 5. Zeroing

| Method | Effect |
|---|---|
| `Encoder_Zero()` | Current position becomes 0 (applied at the next update) |
| `Encoder_ZeroAtNextIndex()` | The next Z pulse becomes 0: a repeatable zero within one motor turn |
| Encoder serial command `0` (pin 1) | Moves the Z mark to the current shaft angle, saved in the encoder |

Limits: an incremental encoder can't know motion while powered off, so saving the last
position to flash is unreliable (sudden power loss, or the leg moved while off). With a
gearbox, Z repeats several times per joint turn, so it can't identify the joint angle
alone. A full absolute zero needs homing to a hard stop/limit switch then
`Encoder_ZeroAtNextIndex()`, or an absolute encoder on the joint.

---

## 6. Speed and acceleration

**Speed:** window method (counts per sample) by default, with 0.73 RPM steps at 100 Hz.
Set `ENC_USE_CAPTURE 1` for M/T speed (needs a capture timer and a jumper from A),
which is precise at all speeds.

**Acceleration:** an alpha-beta-gamma tracking filter. Each sample it predicts the position
from its own position/speed/acceleration estimates using constant-acceleration
kinematics, then corrects all three in proportion to the prediction error.
Differentiating speed directly would turn one count of jitter into ~440 deg/s² of noise at 100 Hz.

Tune with `ENC_ACCEL_TAU_MS` (90% response ≈ 5.3 × τ). Simulated, motor shaft, gear ratio 1:

| τ | Response time | Noise at rest, 100 Hz | Noise at rest, 1 kHz |
|---|---|---|---|
| 25 ms (default) | ~130 ms | ~27 deg/s² | ~8 deg/s² |
| 50 ms | ~270 ms | ~5 deg/s² | ~2 deg/s² |
| 100 ms | ~530 ms | ~1 deg/s² | — |

---

## 7. Graphing

1. Use `main_snippets_csv.c` for blocks 2 and 3 of `main.c` (100 lines/s).
2. `pip install pyserial matplotlib`
3. Close PuTTY, run `python plot_encoder.py COM4` (your port).

Live angle, speed and acceleration graphs; turns and Z errors in the title (red if any
errors); everything saved to `encoder_<date>_<time>.csv`.

---

## 8. Test checklist

| Check | Expected |
|---|---|
| `samples` | +100 per second |
| One full turn by hand | `pos` ±8192, `turns` ±1, `joint` ±360 |
| Reverse | Signs flip |
| Pass the Z mark several times, both directions | `idx` increases, `idx_err` stays 0 |
| Long run | `pos` keeps counting past 2³¹ correctly |
| Stop | Speed and acceleration settle to ~0 |

## 9. Troubleshooting

| Symptom | Likely cause |
|---|---|
| `pos` drifts on its own, ignores the shaft | A/B not connected (floating inputs): check contacts |
| `idx` never increases | Z wire, PD2 EXTI not enabled, or callback missing |
| `idx_err` increasing | Noise on A/B (cable routing near motor wires), or loose contact |
| ~2048 or ~4096 counts per turn | Encoder Mode not set to TI1 and TI2 |
| Wrong direction | Swap A and B, or make `ENC_GEAR_RATIO` negative |
| Board freezes right after start | `Error_Handler()`: encoder timer Counter Period doesn't match `ENC_TIMER_IS_32BIT`, or the timer failed to start |
| Link error `undefined reference to expf` | Add the `m` library in the linker settings |

---

## 10. Tests

PC unit tests simulate the shaft count by count, including Z pulses and injected
count errors: 16-bit rollover, negative turns, Z error detection, 64-bit long runs,
zeroing, acceleration accuracy, startup safety and CSV format.

```
cd tests
gcc -std=c11 -Wall -Wextra -Ihal_stub -I.. test_encoder.c ../encoder.c -lm -o test_encoder
./test_encoder
```

Run them after any change to `encoder.c`. `hal_stub/main.h` stands in for the
STM32 HAL; it is only for tests, never add it to the CubeIDE project.

---

## Changelog

**v2 (fixture review)**
- 16-bit encoder timer support by default (TIM3), `htim3` / `hlpuart1` in snippets
- 64-bit position printed exactly (no 32-bit overflow on long runs; newlib-nano has no `%lld`)
- Joint angle computed in double precision (float loses counts after ~2048 turns)
- Z-index checking on PD2, zero-at-index
- Turn count (floor division, correct for negatives)
- Acceleration via alpha-beta-gamma tracking filter
- Named constants, Doxygen function docs
- CSV at a true 100 Hz (scheduled send instead of `HAL_Delay`)
- Interrupts before `Encoder_Init` finishes are ignored (the EXTI is enabled
  earlier by `MX_GPIO_Init`; previously a Z pulse then would HardFault)
- Startup check: Counter Period must match `ENC_TIMER_IS_32BIT`; HAL start errors checked
- PC unit tests (`tests/`)
