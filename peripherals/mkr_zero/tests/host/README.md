# Host tests — MKR Zero firmware logic

```bash
peripherals/mkr_zero/tests/host/RunTests.cmd    # Windows, MSVC
make check                                      # Linux/Jetson, g++ or clang++
make check-ports                                # Python 3 + bash, fake sysfs only
make check-sketch                               # Python 3: mkr_zero.ino policy (also in make check)
make mutations                                  # Python 3 + make + hosted C++11
```

Each suite exits nonzero on failure, and `make check` runs every
suite and fails if any did, so either form gates a script. No hardware, no
board, no serial port.

| suite | module under test | checks |
|---|---|---|
| `switch_tests` | `lib/SwitchFunctions.cpp` | 61 |
| `imu_tests` | `lib/IMUFunctions.cpp`: lifecycle, integrity, confirmed High-G (incl. INT-line candidate → release probe → stuck verdict, flicker, re-latch, retirement, mid-sample edge, High-G record surviving recovery and the mode-switch snapshot reset, episodes settled on every re-init), channel-level faults | 540 |
| `can_probe_tests` | `lib/CANSniffFunctions.cpp`: the map probe, filter ownership, receive-overrun counting (frames now arrive through the drain ISR and the ring) | 98 |
| `bno_init_tests` | **real** `lib/BNO055Init.cpp`: self-test checks, masked read-back, exact interrupt setup, High-G threshold per accelerometer range, calibration restored in fusion modes only | 602 |
| `can_ring_tests` | `lib/CANFrameRing.h`: the ISR → `loop()` SPSC ring — 16-byte packing (DLC 0-15, extended, remote), FIFO order, full ring refusing and counting the newest, free-running indices wrapping past 65535, peeking ahead of the tail, the producer filling the ring between a peek and its release | 2673 |
| `can_drain_tests` | `lib/CANSniffFunctions.cpp`: the drain ISR against the MCP2515 model (both buffers and rollover, order, `micros()` stamps, extended and remote frames, DLC clamp, overrun events, floating MISO, per-tick bound, a TXREQ left set, full ring), modes arming and disarming it (incl. refused writes, OBD2 left alone), **every main-loop SPI transaction masked** (preemption injected at every byte), a tick due while masked running at the unmask, the drained count as liveness, SNIFF decoding from the ring (golden values, arrival-time stamps, remote frames skipped, DISCOVER decoding nothing, the stream never passing the decoder) and **ring decode == `canDecodeFrame()`** over 3000 random passes | 178 |
| `can_telemetry_tests` | the DISCOVER telemetry through the **real** `buildTelemetry()`: every CAN-derived field at its sentinel after a SNIFF session and with live map traffic, the acceleration input NaN, `canMapFlags` saying accept-all. g++/clang only (see below) | 146 |
| `can_stream_tests` | `lib/CANRawStream.cpp`: F and FS lines byte for byte (DLC 0 with no trailing space, extended ids, remote frames, the 67-byte FS worst case), lines never partial, DTR low → `nohost`, writes only into a free bank and within `availableForWrite()`, whole lines only, a stalled host (no blocking, ring absorbs, `ringdrop`, FS first on resume), not enumerated, a refused write, FS once a second and only with a listener, **each FS column its own counter**, an FS line too long for the room written alone, frame conservation, SNIFF streaming behind the decoder and **discarding for want of a host only behind it**; the **bounded wait**: packet after packet to a reading host in one call, at most `CAN_STREAM_MAX_PACKETS_PER_CALL`, a stalled host costing one `CAN_STREAM_WAIT_US` wait and then none, the call budget, an idle stream (or one just emptied) never polling or waiting; **console text** in whole ≤ 63-byte packets into a free bank, kept whole when the host stalls; a **line break owed** before the first stream line after the port was closed or a console write was refused, so no F line continues a console fragment | 1252 |
| `can_selftest_tests`, `can_selftest_max_tests` | the **bench self-test generator** (`-DDASHCAM_CAN_STREAM_SELFTEST`, below), built at 2400 frames/s and at the 10 000 cap: exactly RATE frames in every second of armed ticks and never more than one off in between, at most one per tick, sequence/fill/check-word payload and `micros()` stamps, nothing in OFF or OBD2 and an unbroken sequence across modes, SNIFF decoding none of it, every frame reaching a reading host as an F line, ring loss appearing as a sequence gap of exactly `ringdrop`, a real bus frame drained in the same tick | 264 / 258 |
| `sketch_policy_tests.py` | **`mkr_zero.ino`**, read as text (no hosted compiler can build it): boot and both recoveries into DISCOVER only, SNIFF/OBD2 only from the host's command, no probe, the drain timer before the first bring-up, every `initializeOBD2()`/`tickOBD2()` masked, liveness from `canDrainedCount()`, ≥ 4 stream seams each after a whole console line, none in the status line, which prints only through `statusOut` and ends with a flush, no console literal that could open an F/FS line | 231 |
| `port_tests.py` | shared MKR USB selector: product identity and ambiguity | 5 cases |

All currently passing.

## The bench self-test build

`../../build_and_upload.sh [PORT|auto] selftest[=N]` defines
`DASHCAM_CAN_STREAM_SELFTEST=N` (default 2400, 1-10000). While the drain is armed
its ISR also synthesizes N frames a second into the ring, with no SPI and no
bus: id `0x7F0`, DLC 8, data = big-endian sequence number (4 bytes), ring fill
(2) and the low 16 bits of the complemented sequence (2). So the USB stream's
throughput is measured on the bench: the Orin counts sequence gaps exactly and
the FS lines say where any loss happened. Synthetic frames count as drained
(and therefore as a live vehicle, keeping the IMU at production load). The
status line gains `st=` and `loop=N/Mus` (passes since the last line / longest
pass). Without the option none of it is compiled: a production build is
byte-identical to the same tree with every `#if defined(DASHCAM_CAN_STREAM_SELFTEST)`
block deleted (checked on 2026-10-05). Never flash a selftest build to the car.

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
- **CAN drain and raw stream** — the model also speaks READ STATUS, stores
  extended and remote frames as the silicon lays them out (a remote frame keeps
  the PREVIOUS payload in the data registers, the trap the drain must not fall
  into; a standard filter never matches an extended frame), and can float MISO.
  `lib/CANStreamHw.cpp` is registers only — TC3, the NVIC mask, the USB bulk IN
  bank — so it is the one firmware file NOT compiled here: `can_stream_hw_stub.cpp`
  implements its four functions over a model. The test is the timer
  (`hostDrainTick()`); the mask nests and leaves a due tick pending until the
  outermost unmask, as the NVIC does; and every SPI transaction is reported to
  it, so a main-loop transaction made while the drain is armed and unmasked is
  counted, and with preemption on a tick actually fires inside it and corrupts
  it, as on the board. The stub `Serial` captures what the stream writes and
  models the bank: armed after a write until the test collects it — a write
  into an armed bank is where the core's `send()` would spin for 70 ms, and is
  counted. Whether `CANStreamHw.cpp` itself programs TC3 correctly is a bench
  question; these suites prove what is done with it.
- **Equivalence with the previous decoder** — besides the in-suite check that
  decoding from the ring equals calling `canDecodeFrame()` directly, the
  2026-10-04 change was verified once against the code it replaced: the same
  20 000 random passes run through the pre-change `tickCANSniff()` (built from
  `git archive HEAD`) and through the new drain → ring → `tickCANSniff()` path
  gave byte-identical decoded state after every pass. That harness lived in a
  scratch directory and is not part of the suite.

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

IMU hardening follow-up (2026-09-26, 41 mutants; 2026-09-27 adds the High-G
range and CAN overrun rows below, 45, then the two AMG calibration rows, 47 in
all): `make mutations` reproduces these
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
| open episode dropped silently (retire / re-init) | an undecided candidate/probe is counted as rejected |
| re-init drops an open episode | a host mode switch or calibration re-init settles it first |
| released probe ignored (retire / re-init) | a probe that let go before the part stopped being polled is the impact |
| line sampled before the edge snapshot | a latch landing mid-sample: counted as rejected and as an event, timed late |
| mid-sample edge not merged | a latch landing just before the sample keeps its edge time |
| bring-up retains a pin edge | configuration edges are not impacts or runtime rejects |
| recovery clears hgrej | counter survives re-init/mode changes and saturates |
| recovery zeroes the High-G count | imuHighGCount must climb for the whole boot |
| recovery cancels the High-G hold | an impact that retires the part stays published 500 ms through same-pass recovery |
| recovery erases the last latch time | imuHighGMs keeps pointing at the last real latch |
| mode-switch snapshot reset drops the High-G record | the published count must not read N, 0, N across a host mode switch |
| snapshot reset copies the hold without its deadline | resetIMUData must not revive an expired hold |
| High-G hold never expires | the hold ends on its own deadline wherever it is published |
| MAG-only failure retires AMG | accel/gyro stay valid; MAG becomes NaN, status PARTIAL |
| failed MAG still published | magnetic validity and all three values are cleared |
| bring-up skips self-test validation | failed/missing self-test is named and latched |
| interrupt read-back verifies only High-G | unexpected defined enable bits disable the backstop |
| interrupt setup preserves other enables | INT_EN and INT_MSK must be written exactly 0x20 |
| AMG restores the calibration profile | the part raises SYS_ERR 0x09 and AMG never comes up |
| SYS_ERR failure latches a stale OPR_MODE | the failure record must show the mode actually read |
| AMG High-G threshold counted at ±4 g | AMG must write 32 (2 g at ±16 g), not 128 (8 g) |
| fixed High-G byte in every mode | the threshold must follow the range in both modes |
| CAN overruns cleared uncounted | each latched overrun is one counted event |
| CAN overrun count reset by the probe | the count is boot-cumulative, like hgrej |
| **none — unmutated control** | **none** |

The model refuses a filter write the way the silicon does when its request for
Configuration never completes — OPMOD stays where it was — so a test that then
finds Configuration knows the driver's own park ran. (An earlier version of the
model entered Configuration itself before refusing, which made those checks
pass whether or not the driver parked anything.)

Raw CAN stream (2026-10-04, 24 more, 71 in all):

| mutation | which suite caught it |
|---|---|
| ring full one slot early / a refused frame not counted / DLC 8 not flagged | `can_ring_tests` |
| drain ignores RXB1 / believes a floating MISO / is silenced by a pending TXREQ / is unbounded | `can_drain_tests` |
| refused frames not counted as drained (liveness) / extended bit lost in the drain | `can_drain_tests` |
| drain armed in OBD2 / host filter write made with the drain unmasked | `can_drain_tests` |
| DISCOVER decodes / frames stamped at decode time / remote frames decoded / stream passes the decoder | `can_drain_tests` |
| DISCOVER keeps SNIFF's filter bookkeeping (`canMapFlags` lies) | `can_telemetry_tests` |
| stream writes into an armed bank / ignores DTR / ignores `availableForWrite()` | `can_stream_tests` |
| DLC 0 trailing space / extended id in 3 digits / lowercase hex / FS starved behind frames / failed write counted as streamed | `can_stream_tests` |

Review 2026-10-05 (21 more library mutants and 8 sketch mutants, 100 in all).
The first five were found surviving by an adversarial pass and are why the
FS-column and oversized-FS tests exist:

| mutation | which suite caught it |
|---|---|
| FS line's ringdrop / ovf / nohost column fed 0, drained column fed the streamed count | `can_stream_tests` |
| an FS line longer than the room packed into the buffer with frames behind it | `can_stream_tests` |
| stream never waits for its own packet / waits before its first / waits without a time bound / ignores the call budget / waits after the packet that emptied the ring | `can_stream_tests` |
| console text written into an armed bank / in 64-byte packets | `can_stream_tests` |
| no line break owed after the port was closed / after a refused console write; a break on every packet | `can_stream_tests` |
| self-test rate off by a tick / sequence stuck / runs disarmed / not counted as drained / fill not the ring's / plain check word | `can_selftest_tests` |
| sketch: boots into SNIFF / OBD2 give-up parks OFF / OFF retry arms the poller / `tickOBD2()` unmasked / liveness not stamped from the drain / stream seam inside the status line / status line not flushed / a console line opening with `F ` | `sketch_policy_tests.py` |

Not mutated, because no single-threaded test can tell: the compiler barriers
in `CANFrameRing.h`, and the lock in `canSetMode()` — that function disarms the
drain before its first bus access, so on one core it is safe even unmasked; the
lock is kept for the rule's sake, and its siblings that stay armed
(`canSniffSetFilters()` above) are the ones the suite holds to it. Nor the clamp
inside `canRawDiscard()`: its one caller already passes `canRawStreamable()`,
so the mutant is equivalent; the SNIFF no-host test pins the behaviour instead.

The earlier hardening tests incorrectly REQUIRED reserved bits to be zero and
treated an unconfirmed pin edge as an impact. Those expectations were corrected:
mutation testing measures a suite's sensitivity, not the truth of its contract.
The current script covers 71 mutations; the older switch/CAN tables above record
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

Added for the raw stream, and no more than it calls: `micros()` (set by the
test, like `millis()`), `Serial.dtr()` / `availableForWrite()` / `write()` with
the bank model above, `Stream` and a silent `Serial1` plus a one-line
`SparkFun_u-blox_GNSS_Arduino_Library.h` so the telemetry builder links, and
`can_stream_hw_stub.cpp` for `CANStreamHw.h`. The CAN stand-in's
`setFilterRegisters()` reports itself to the mask check as one SPI
transaction, since the real library's version is about thirty.

`can_telemetry_tests` is in the Makefile only: `CommProtocol.h` declares the
payload with `__attribute__((packed))`, which MSVC rejects, so RunTests.cmd
leaves it out.

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
