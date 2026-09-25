# Host tests — MKR Zero firmware logic

```bash
peripherals/mkr_zero/tests/host/RunTests.cmd    # Windows, MSVC
make check                                      # Linux/Jetson, g++ or clang++
```

Each suite's exit status is its failing-check count, and `make check` runs every
suite and fails if any did, so either form gates a script. No hardware, no
board, no serial port.

| suite | module under test | checks |
|---|---|---|
| `switch_tests` | `lib/SwitchFunctions.cpp` | 61 |
| `imu_tests` | `lib/IMUFunctions.cpp`: bring-up lifecycle, fault accounting | 59 |
| `can_probe_tests` | `lib/CANSniffFunctions.cpp`: the map probe and filter ownership | 89 |

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
  only shows with that order.
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

The model refuses a filter write the way the silicon does when its request for
Configuration never completes — OPMOD stays where it was — so a test that then
finds Configuration knows the driver's own park ran. (An earlier version of the
model entered Configuration itself before refusing, which made those checks
pass whether or not the driver parked anything.)

Every mutant killed the cases that should die and the control stayed green.
Cases carrying a `REGRESSION` comment are the ones anchored to a real defect
found in review. Treat a change that makes one of them pass trivially as a
change that has removed the check.

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
