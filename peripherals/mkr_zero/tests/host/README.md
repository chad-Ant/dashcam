# Host tests — `lib/SwitchFunctions.cpp`

```bash
peripherals/mkr_zero/tests/host/RunTests.cmd    # Windows, MSVC
make check                                      # Linux/Jetson, g++ or clang++
```

Exit status is the failing-check count, so either form gates a script.

61 checks, currently all passing. No hardware, no board, no serial port.

## What is actually under test

The module compiles **unmodified**. `SwitchFunctions.h` includes `<Arduino.h>`
with angle brackets, and this directory goes first on the include path, so
`./Arduino.h` shadows the real one. There is no `#ifdef TEST`, no test-only
build of the firmware, and no second copy of the logic that could drift.

Two seams make it work, and both are improvements independent of testing:

- `switchIngest(sw, raw, nowMs)` — the pure core. Every decision the module
  makes is a function of a sample sequence and a clock, and both are now
  arguments rather than things read from the world. `pollSwitches()` is one line
  over the top of it.
- `arduino_stub.cpp` models an actual **16-stage 74HC165 chain** — asynchronous
  load while PL is low, shift on the rising edge of CP, Q7 valid before any
  clock. So `switchReadRaw()` is under test too, not just the state machine.

That second point matters more than it looks. The obvious stub returns canned
bits, which covers the state machine and leaves the shift loop uncovered — and
the shift loop is where **bit order** lives, which is the one thing a reader
cannot verify by inspection and a bench cannot verify without a known pattern.
A driver that clocked before reading, shifted the wrong way, or mislocated the
sentinels fails against this model.

## The tests have been shown to fail

A suite that has only ever passed proves nothing. Each of the shipped defects
was reintroduced into a copy of the source and the suite re-run:

| mutation | failures | which cases died |
|---|---|---|
| `agree[]` survives a rejected read | 2 | outage/debounce only |
| chatter counted on commits, not raw edges | 3 | all three chatter cases |
| data pin left floating instead of pulled up | 2 | absent-chain only |
| one fixed bucket instead of a sliding window | 1 | straddling burst only |
| **none — unmutated control** | **0** | — |

Every mutant killed exactly the cases that should die and no others, and the
control stayed green. The fourth is the most informative: it isolates the
sliding-window property from everything else, which is the whole reason that
test was written separately.

Cases carrying a `REGRESSION` comment are the ones anchored to a real defect
found in review. Treat a change that makes one of them pass trivially as a
change that has removed the check.

## Adding to the stub

Keep it minimal. This is not an Arduino emulator and should not become one — it
provides exactly what the module under test calls, and the next module to get
tests should add the smallest thing that compiles rather than the whole API.

One deliberate gap: with the data pin configured as a bare `INPUT` and no chain
fitted, the stub returns `LOW`. Real floating silicon returns *noise*, which is
the entire reason the production code uses `INPUT_PULLUP` — a floating pin
satisfies a two-bit sentinel pattern about one read in four. The stub cannot
model that honestly, so the test asserts the pin mode directly instead of trying
to simulate the failure it prevents.
