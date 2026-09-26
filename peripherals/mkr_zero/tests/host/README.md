# Host tests — MKR Zero firmware logic

```bash
peripherals/mkr_zero/tests/host/RunTests.cmd    # Windows, MSVC
make check                                      # Linux/Jetson, g++ or clang++
make check-ports                                # Python 3 + bash, fake sysfs only
make mutations                                  # Python 3 + make + hosted C++11
```

Each suite exits nonzero on failure, and `make check` runs every
suite and fails if any did, so either form gates a script. No hardware, no
board, no serial port.

| suite | module under test | checks |
|---|---|---|
| `switch_tests` | `lib/SwitchFunctions.cpp` | 61 |
| `imu_tests` | `lib/IMUFunctions.cpp`: lifecycle, integrity, confirmed High-G (incl. INT-line candidate → release probe → stuck verdict, flicker, re-latch, retirement, mid-sample edge), channel-level faults | 502 |
| `can_probe_tests` | `lib/CANSniffFunctions.cpp`: the map probe and filter ownership | 89 |
| `bno_init_tests` | **real** `lib/BNO055Init.cpp`: self-test checks, masked read-back, exact interrupt setup | 451 |
| `port_tests.py` | shared MKR USB selector: product identity and ambiguity | 5 cases |

All currently passing.

## What is actually under test

The modules compile **unmodified**. They include `<Arduino.h>`, `<Wire.h>`,
`<SPI.h>`, `<CAN.h>` and `<SdFat.h>` with angle brackets, and this directory goes
first on the include path, so the stand-ins here shadow the real ones. There is
no `#ifdef TEST`, no test-only build of the firmware, and no second copy of the
logic that could drift.

Each suite fakes at a seam the firmware already has, and models hardware
wherever the defect lived in how the hardware behaves:

- **switches** — `switchIngest(sw, raw, nowMs)` is the pure core, and
  `arduino_stub.cpp` models an actual **16-stage 74HC165 chain** (asynchronous
  load while PL is low, shift on the rising edge of CP, Q7 valid before any
  clock). So `switchReadRaw()` is under test too, including **bit order**, which
  a reader cannot verify by inspection and a bench cannot verify without a known
  pattern.
- **IMU** — the staged bring-up (`bno055Init*`) and the counted transport
  (`bno055BusRead/Write`) are faked in `imu_tests.cpp`: a bring-up that reaches
  Configured a few ticks after it is begun and then *stays* there, and a burst
  the test writes register by register. Every poll runs `loop()`'s own order —
  `imuInitTick()` every pass, then `getIMUData()` — because the lifecycle defect
  only shows with that order. The INT line is a plain pin the test drives; an
  RST_INT write drops it, as the part does. `hostOnNextRead()` runs a one-shot
  callback just before or after the next `digitalRead()` of a pin, so a latch
  can land between the driver's edge snapshot and its line sample.
- **BNO bring-up** — a separate two-page register model runs the actual
  `BNO055Init.cpp`, not the IMU suite's staged fake. It varies all reserved
  self-test bits, fails ACC/GYR/MCU tests or the status read, and injects dirty
  interrupt enables and sticky defined read-back bits. Both supported modes
  must reach Configured only after the required self-tests pass. A MAG-only
  failure is not a whole-IMU failure; the raw magnetic channel is invalidated.
- **CAN probe** — `mcp2515_model.cpp` decodes the controller's SPI instruction
  set against a register file, and frames enter from the **bus side**, accepted
  or rejected by the masks and filters the driver actually programmed (RXB0 on
  mask 0 / filters 0-1, RXB1 on mask 1 / filters 2-5, RXM = 11, BUKT rollover,
  nothing received in Configuration). A stub that handed the driver frames
  directly would have hidden the probe defect: the frames it never saw were the
  ones the controller's own filters threw away.

## The tests have been shown to fail

A suite that has only ever passed proves nothing. Each of the shipped defects
was reintroduced into a copy of the source and the suite re-run.

`switch_tests`:

| mutation | failures | which cases died |
|---|---|---|
| `agree[]` survives a rejected read | 2 | outage/debounce only |
| chatter counted on commits, not raw edges | 3 | all three chatter cases |
| data pin left floating instead of pulled up | 2 | absent-chain only |
| one fixed bucket instead of a sliding window | 1 | straddling burst only |
| **none — unmutated control** | **0** | — |

`imu_tests` and `can_probe_tests` (review findings, 2026-09-25):

| mutation | which cases died |
|---|---|
| `imuInitTick()` takes readiness from the Configured *stage* again | failed reads, frozen data and `imuMarkAbsent()` cases |
| fault run cleared before the accel-limit gate again | impossible acceleration only |
| probe keeps the map's filters instead of opening them | wrong map on a busy bus; refused open; skipped probe |
| verdict overwrites a host filter set | host filters survive the verdict only |
| skipping a running probe leaves its accept-all filters | skipped probe only |
| a refused host filter write is not parked | refused host filters only |
| a refused probe open marks OFF without parking the chip | refused open only |
| **none — unmutated control** | **none** |

IMU hardening follow-up (2026-09-26, 34 mutants): `make mutations` reproduces these
regressions in temporary source copies, leaving the checkout untouched. A
mutant must compile successfully and fail assertions; compilation errors do
not count as caught bugs. Unmodified controls must pass first.

| mutation | which cases died |
|---|---|
| ST_RESULT core gate off | missing ACC/GYR/MCU pass and reset-burst cases |
| reserved ST_RESULT bits rejected | all upper-nibble combinations must remain usable |
| legacy reserved INT_STA bits rejected | bits 4/1/0 must not diagnose corruption |
| INT_STA integrity gate off | unexpected defined motion-interrupt bits |
| all-zero accel+gyro gate off | reset signature after POST |
| zero GYRO alone rejected | a stationary part must remain usable |
| naked pin edge accepted | glitch after a rejected burst / across a NACK |
| held INT believed without a probe (the old rule) | stuck-high line: one fake event, flag held forever |
| held INT never considered | real latch whose INT_STA a corrupt read consumed is lost |
| probed on first sight (no candidate poll) | a line high one poll and low the next became an event |
| self-dropped candidate probed anyway | flicker: RST_INT writes and events from a line nothing held |
| self-dropped candidate not counted | rejection is counted for a line that let go by itself |
| probe written on a NACK poll | no write, and no disarmed backstop, on a bus that NACKs |
| no stuck verdict after a failed release | stuck-high line keeps being probed |
| stuck verdict from a corrupt read | the verdict must wait for a sound read |
| trust never restored once the line reads low | a cleared fault must not disarm the pin for good |
| re-latch edge ignored by the probe | impact re-latching after RST_INT, seen through a corrupt read, was lost as INTSTUCK |
| released probe clears again | exactly one RST_INT per probe |
| re-latched probe not cleared | the new latch after a release must be cleared |
| NACK cannot decide a released probe | the impact that knocks the connector loose is lost at retirement |
| RST_INT omitted | latch is counted and physically released |
| NACK drops a glitch edge uncounted | rejection is counted even with no later good read |
| NACK counts a held latch as rejected | one real event must not read as both hg and hgrej |
| retirement drops an open episode silently | an undecided candidate/probe is counted as rejected |
| retirement ignores a released probe | a probe that let go on the retiring poll is the impact |
| line sampled before the edge snapshot | a latch landing mid-sample: counted as rejected and as an event, timed late |
| mid-sample edge not merged | a latch landing just before the sample keeps its edge time |
| bring-up retains a pin edge | configuration edges are not impacts or runtime rejects |
| recovery clears hgrej | counter survives re-init/mode changes and saturates |
| MAG-only failure retires AMG | accel/gyro stay valid; MAG becomes NaN, status PARTIAL |
| failed MAG still published | magnetic validity and all three values are cleared |
| bring-up skips self-test validation | failed/missing self-test is named and latched |
| interrupt read-back verifies only High-G | unexpected defined enable bits disable the backstop |
| interrupt setup preserves other enables | INT_EN and INT_MSK must be written exactly 0x20 |
| **none — unmutated control** | **none** |

The model refuses a filter write the way the silicon does when its request for
Configuration never completes — OPMOD stays where it was — so a test that then
finds Configuration knows the driver's own park ran. (An earlier version of the
model entered Configuration itself before refusing, which made those checks
pass whether or not the driver parked anything.)

The earlier hardening tests incorrectly REQUIRED reserved bits to be zero and
treated an unconfirmed pin edge as an impact. Those expectations were corrected:
mutation testing measures a suite's sensitivity, not the truth of its contract.
The current script covers 17 mutations; the older switch/CAN tables above record
separate historical checks, not additional mutations run by this script.

Cases carrying a `REGRESSION` comment are the ones anchored to a real defect
found in review. Treat a change that makes one of them pass trivially as a
change that has removed the check.

## Before flashing / bench acceptance

Build both IMUPLUS and AMG with the pinned production toolchain. Flash only the
intended mode to a parked rig, with no competing serial reader. Confirm the map
identity is unchanged, IMU comes up, and GPS/C3 links recover. A controlled bump
must increment High-G and release INT; a separately timed wiring disturbance
must not manufacture an impact. Keep the timed console log and distinguish
simulated fault coverage from physical testing. `hgrej` counts rejected evidence
for the entire boot, not distinct impacts; re-flashing/rebooting starts a new
boot. Software gates do not repair intermittent breadboard wiring.

### Follow-up deployment, 2026-09-26

The fusion firmware was flashed to the parked MKR and flash verification passed;
both fusion and AMG production builds passed before upload. All 1,029 MKR C++
checks also passed with AddressSanitizer/UndefinedBehaviorSanitizer, the five USB
selector cases passed, and all 17 mutation checks were caught. The separate C3
bridge suite passed its 79 checks; the C3 was not reflashed.

A 40-second post-flash capture and a separate 120-second bump-test capture kept
the IMU, GPS communication and C3 link up. GPS had no satellite fix and CAN was
in sniff mode with no received traffic, so neither navigation nor active CAN
traffic was validated. The bump capture held `ioerr=0/0`, `hg=0`, `hgrej=0`;
`gaps=1` was already present at its start and did not increase. The user reported
two bumps, but neither registered as High-G. The largest console-reported peak
was 11.45 m/s² (about 1.17 g); these intermittent peak reports cannot exclude a
shorter unsampled event. **Physical impact/re-arm acceptance remains incomplete**,
as does the follow-up wiring-disturbance test. No threshold was lowered to force
a pass. No SD map write was performed during this deployment, and the post-flash
capture did not include the boot-time map identity.

Local bench logs and the pre-update rollback image are in
`/tmp/dashcam-imu-fix.ACN32m/` (temporary, not version-controlled).

## Adding to the stubs

Keep them minimal. This is not an Arduino emulator and should not become one —
the stand-ins provide exactly what the modules under test call, and the next
module to get tests should add the smallest thing that compiles rather than the
whole API.

`HIGH`/`LOW` are enumerators, not macros, as in the real core (ArduinoCore-API
`api/Common.h`, which arduino:samd 1.8.14 uses). A macro would rewrite every
scoped enumerator of the same name, and `VehGear::LOW` is one.

The Wire, CAN and SdFat stand-ins define the markers that `I2CBus.h`,
`CANSniffFunctions.h` and `SDFunctions.h` check for the vendored libraries.
Those guards exist to stop a *firmware* build against the stock libraries; a
stand-in is neither stock nor vendored, and it defines the marker only so the
real headers compile unmodified.

One deliberate gap: with the switch data pin configured as a bare `INPUT` and no
chain fitted, the stub returns `LOW`. Real floating silicon returns *noise*,
which is the entire reason the production code uses `INPUT_PULLUP` — a floating
pin satisfies a two-bit sentinel pattern about one read in four. The stub cannot
model that honestly, so the test asserts the pin mode directly instead of trying
to simulate the failure it prevents.
