# IMU test runbook

Four test artifacts cover the IMU stack. They need **different firmware on the
boards**, so the order below matters: phases 1 and 2 flash test sketches, and
phase 3 puts production firmware back before the end-to-end run.

| Phase | What it tests | Board(s) | Needs |
|---|---|---|---|
| 1 | Driver, calibration file, fault injection | MKR Zero | test sketch, sensor, SD card, two taps |
| 2 | Bridge frame coalescing | ESP32-C3 | test sketch only — no MKR, no wiring |
| 3 | — | both | restore production firmware |
| 4 | MKR → C3 → PC, both IMU modes | both | production firmware, `pyserial` |

Phases 1 and 2 are independent and can run in either order. Phase 4 requires
phase 3.

---

## 0. Prerequisites

- `arduino-cli` on PATH, with `arduino:samd@1.8.14` (the exact version is
  enforced — the vendored Wire patch depends on that core's private SERCOM API)
  and the `esp32` core.
- `pip install pyserial` — phase 4 only.
- Both boards' COM ports. Find them with:

```bash
arduino-cli board list
```

The MKR Zero and the ESP32-C3 enumerate separately. Everything below writes
`COM5` for the MKR and `COM7` for the C3 — substitute your own.

**Serial monitor discipline, for every phase:** open the monitor at **115200**
*first*, then press the board's reset button. All three sketches print their
whole report within a few seconds of boot and then idle, so attaching afterwards
loses the beginning. And **close the monitor before phase 4** — the script opens
the same port and will fail if something else holds it.

---

## 1. MKR Zero: driver and fault injection

Flash and watch:

```bash
cd peripherals/mkr_zero/helper_scripts/IMUFixVerify && ./BuildAndUpload.cmd COM5
```

Then open the monitor at 115200 and reset.

### What it does

- **A** — peak-ring geometry and telemetry flag mapping. No hardware needed.
- **B** — calibration file parsing. Needs the SD card.
- **C** — page-1 recovery, gap accounting, armed reporting, peak retention.
  Needs the sensor.
- **D** — fault injection: frozen accelerometer, stuck High-G latch, AMG
  saturation rail. Needs the sensor.

### You will be prompted three times

- `tap the board within 5 s` (C4) — an ordinary tap.
- `knock the board HARD within 8 s (needs > 2 g)` (D2) — must cross the High-G
  threshold so a real interrupt is latched.
- `tap the board FIRMLY within 8 s (needs > 4 g)` (D3) — harder still; it must
  exceed 4 g to land in the band that test is about.

Missing any prompt yields a SKIP, not a failure — but a SKIP means that check
did not run, and the summary reports `INCOMPLETE` rather than `OK`. Re-run if
you want the coverage.

### Two things it changes

- **It rewrites `bno055.cal`.** The original is copied into RAM and written back
  at the end. Watch for `bno055.cal RESTORED`. If it prints the
  `could NOT be restored` line instead, the stored calibration is gone and the
  sensor needs recalibrating — nothing else breaks.
- **Group D reconfigures the sensor** (accelerometer power mode, High-G
  threshold, operating mode) and restores production configuration as it
  finishes. If you interrupt or reset the board mid-group-D, **power-cycle it**
  before trusting the sensor — a reset alone does not clear the BNO055's
  registers.

### Pass criterion

Three outcomes, and only the first is a pass:

- `RESULT: OK` — everything ran and everything passed.
- `RESULT: INCOMPLETE` — nothing failed, but some checks did not run. A missed
  tap prompt, a wedged bus, or an injection the part declined. **Not a pass**:
  read which checks skipped and decide whether you need them.
- `RESULT: FAILURES PRESENT` — a check failed.

An injection that does not take reports SKIP rather than FAIL on purpose: a
sensor that ignored the fault has tested nothing, and calling that a driver
failure would be a lie. The reverse matters too, which is why a suite that
skipped its whole hardware half no longer reports OK.

### If group C skips with "I2C bus is not usable"

The sketch prints the stuck-line bitmask and what it means. In short:

- **SDA still clamped** — a slave is holding the line, usually after a reset
  landed mid-transaction. **Power-cycle the board.** An MCU reset will not clear
  it, because the MCU is not the thing holding the line, and neither will the
  nine-clock recovery that has already been tried. On a USB-powered rig a power
  cycle means unplugging USB (and the battery, if fitted) — the reset button is
  not enough.
- **SDA never moved** — no pull-ups, no power to the sensor, or a dead part.
  Check wiring before anything else.
- **SCL low / SCL never rose** — clock held or shorted; recovery could not even
  run.

Reflashing while a previous run was mid-poll is the usual way to get here, which
is also why the group D warning above says to power-cycle after an interrupted
run.

---

## 2. ESP32-C3: bridge coalescing

```bash
cd peripherals/esp32-c3/tests/BridgeCoalesceVerify && ./BuildAndUpload.cmd COM7
```

Open the monitor at 115200 and reset.

**No wiring, no MKR, no second board.** The sketch puts Serial1 into internal
loopback and transmits master frames to itself, so the real decoder runs on real
frames. If the chip refuses loopback it says so and asks for a jumper between
GPIO21 and GPIO20, which gives the identical path.

### Pass criterion

`RESULT: OK`, `failed 0`, and `crc errors on the loopback: 0`.

---

## 3. Restore production firmware

Both phases above replaced the production sketch. Put it back:

```bash
cd peripherals/mkr_zero && ./BuildAndUpload.cmd COM5
```

```bash
cd peripherals/esp32-c3 && ./BuildAndUpload.cmd COM7
```

Confirm on the MKR's monitor that it prints `IMU: bring-up started (IMUPLUS)`.

---

## 4. End to end: MKR → C3 → PC

### Wiring

- MKR ↔ C3 cross-wired as usual (MKR TX → C3 GPIO20, MKR RX → C3 GPIO21, common
  ground).
- C3's USB into this PC.
- MKR powered.

**Close every serial monitor** before running the script.

### Run both modes in one pass

```bash
python peripherals/esp32-c3/tests/host/imu_e2e_check.py --port COM7 --both --set
```

This commands IMUPLUS, checks it, commands AMG, checks it, and restores IMUPLUS
at the end. `--set` sends `CMD_SET_IMU_MODE` over the wire, so **no reflash is
needed** to test the raw mode.

### Other useful forms

Observe whatever the master booted into, without commanding anything:

```bash
python peripherals/esp32-c3/tests/host/imu_e2e_check.py --port COM7
```

Print every frame as it arrives, for eyeballing live values:

```bash
python peripherals/esp32-c3/tests/host/imu_e2e_check.py --port COM7 --raw
```

List candidate ports:

```bash
python peripherals/esp32-c3/tests/host/imu_e2e_check.py --list
```

### What it checks

- The link: frames arrive, no CRC errors, the bridge's reported protocol version
  and telemetry size match the decoder, and `masterMillis` advances — a stuck
  snapshot being republished would otherwise pass every other check.
- The mode: read from `TLM_FLAG_IMU_FUSION_MODE` and **checked against what was
  asked for**, not merely followed. Without that, a master stuck in the other
  mode would be judged against the contract it happens to satisfy.
- The per-mode contract, including each mode's *silences*: IMUPLUS must publish
  linear acceleration and relative yaw with the magnetometer NAN; AMG must
  publish the magnetometer with linear acceleration and yaw NAN. A fabricated
  zero where there is no measurement is the failure this whole driver was
  rebuilt around, so a mode inventing the other's fields fails.
- `IMU_PRESENT` and `HIGHG_ARMED` on every frame.

### Pass criterion

`RESULT: OK` with `failed 0`. The script exits non-zero on failure, so it can be
chained in a script.

### If no telemetry arrives

The script says so and stops. In order of likelihood: a serial monitor still
holding the port; the MKR unpowered or not cross-wired; the two boards on
mismatched protocol versions (the script reports the bridge's version against its
own). The bridge's own log lines are relayed and printed, so a master-link
failure usually names itself.

---

## Quick reference

| Command | Phase |
|---|---|
| `arduino-cli board list` | 0 |
| `cd peripherals/mkr_zero/helper_scripts/IMUFixVerify && ./BuildAndUpload.cmd COM5` | 1 |
| `cd peripherals/esp32-c3/tests/BridgeCoalesceVerify && ./BuildAndUpload.cmd COM7` | 2 |
| `cd peripherals/mkr_zero && ./BuildAndUpload.cmd COM5` | 3 |
| `cd peripherals/esp32-c3 && ./BuildAndUpload.cmd COM7` | 3 |
| `python peripherals/esp32-c3/tests/host/imu_e2e_check.py --port COM7 --both --set` | 4 |

`./BuildAndUpload.cmd` with no port argument compiles without flashing, which is
the quick way to check a change builds.

`cd peripherals/mkr_zero && ./BuildAndUpload.cmd COM5 amg` flashes the master
with AMG as its **boot** default. Phase 4's `--set` makes this unnecessary for
testing; it remains the only way to change what the board comes up in.
