# Mission-critical coding & review (MKR Zero / SAMD21 / Arduino)

An MKR Zero is often deployed where **no operator can intervene** — on a LiPo,
logging to its SD card, inside an enclosure the user won't open. A hang, a heap
fragmentation, an unchecked SD error, or a corrupted log isn't a stack trace on
someone's screen; it's a dead node or lost data until someone physically resets
it. So treat both **code you generate** and **code you review** as safety-critical
embedded software, not hobby sketches.

The standard below adapts Gerard Holzmann's *Power of Ten* (NASA/JPL) to the MKR
Zero Arduino (C++) reality. The Power of Ten targets C on spacecraft; most of it
transfers directly, but a few rules need judgment in an event-driven Arduino
world. Adapt with reasoning — don't cargo-cult. Where a rule is relaxed below,
the relaxation is stated explicitly so it's a decision, not an accident.

## Posture

- **Default to strict.** When a stricter form costs a few lines but removes a
  field-failure mode, take it. Availability of an unreachable device beats
  brevity.
- **Generation:** every sketch must satisfy the pre-flight checklist at the
  bottom before you emit it.
- **Review:** be **clear and critical**. Do not rubber-stamp. See the review
  rubric and tone section.

## The ten rules, adapted

**1 — Simple control flow. No recursion.**
Keep control flow flat and analyzable. No `goto`, no `setjmp`/`longjmp`, no
recursion (with only **32 KB SRAM** and a fixed stack, recursion risks a stack
overflow that manifests as a mysterious hard fault / reset). Early `return` on
error is fine and usually clearer than nesting.

**2 — Every loop has a fixed, provable upper bound. (Highest priority.)**
Any wait on the outside world must time out and have a defined action on timeout.
The MKR Zero's signature field-fatal bug lives here:

```cpp
// ❌ Hangs forever on battery — no USB host ever opens the port
while (!Serial);
// ❌ Hangs forever if the card is missing/unformatted
while (!SD.begin(SDCARD_SS_PIN));
// ❌ Hangs forever if the sensor is unplugged
while (!sensor.ready());
```

Always bound it and define the fallback:

```cpp
const uint32_t SERIAL_WAIT_MS = 2000;
uint32_t t0 = millis();
while (!Serial && millis() - t0 < SERIAL_WAIT_MS) { }   // proceed regardless after 2 s

if (!SD.begin(SDCARD_SS_PIN)) { /* defined fallback: signal error, run without logging,
                                  retry next cycle — never spin */ }
```

The two intentional exceptions are `loop()` and any RTOS/event task, which are
*meant* to run forever — for those, the inverse rule applies: make sure they
can't accidentally fall through or exit, and keep each `loop()` pass short and
non-blocking.

**3 — No dynamic allocation after initialization. (Sharper here than on a big MCU.)**
With only **32 KB SRAM**, memory discipline matters more than on an ESP32
(~400 KB) or even a Mega. Allocate in `setup()`; never churn the heap in
`loop()`, ISRs, or callbacks.
- Prefer fixed `char` buffers + `snprintf` over `String` concatenation. A
  `String line = a + "," + b;` per log record fragments a tiny heap fast — and
  **fragmentation, not exhaustion, is what kills long-running loggers**.
- Large arrays/buffers go `static`/global, not on the stack — the stack is small
  and a big local array silently overflows it into corruption.
- Watch your SRAM budget at compile time (the IDE reports it). SD, audio, and
  string-heavy libraries eat it quickly; if you're near the limit, that's a
  design constraint to address, not ignore.

**4 — Keep functions short and single-purpose (~60 lines).**
Factor `setup()`/`loop()` into named helpers — `initSD()`, `readSensor()`,
`logRecord()`, `sleepUntilNext()`. A function you can see whole is one you can
verify whole, and it makes the review tractable.

**5 — Defend with checks; recover explicitly.**
Holzmann wants ≥2 assertions/function. On a field device a hard `assert()` aborts
→ reset, trading availability for fail-stop — sometimes right, but usually you
want **graceful recovery**, not a crash. So the adapted rule: validate every
parameter and every piece of external data, and take an *explicit* recovery
action.
- Check sensor outputs for sentinels: `if (isnan(t)) { ... }`.
- Range-check before trusting: a value outside the datasheet range is a fault,
  not a reading.
- **Validate that an input can't exceed 3.3 V** at the design level — an input
  expected to be 5 V is a hardware bug to flag, not a value to read.
- On failure, do something defined: bounded retry, safe state, skip-and-log, or
  sleep-and-retry-next-cycle — never silently continue with bad data.

**6 — Smallest possible scope.**
Declare where used; `const`/`constexpr` by default; never reuse one variable for
two meanings. Narrow scope = fewer places a bad value can come from.

**7 — Check every return value; validate every parameter. (The other top priority.)**
The most-violated and most-valuable rule. On the MKR Zero that means: `SD.begin()`
and every `SD.open()` (card absent/full/corrupt is normal), `file` truthiness,
`Wire.endTransmission()` (0 = ok), `sensor.begin()`, library init calls, and the
return of `Watchdog.enable()`. If you deliberately ignore one, cast to `(void)`
and say why in a comment. Validate inputs too — including **pin numbers against
the MKR Zero map and the EXTINT table** (`hardware.md`): an interrupt requested on
a pin the stock variant leaves `EXTERNAL_INT_NONE` silently never fires — a bug
caught at review, not in the field.

**8 — Restrained preprocessor.**
`#define` for simple constants and include guards only. Prefer `constexpr`/
`const` and `inline` functions to function-like macros (no hidden side effects,
type checking intact). Minimize `#ifdef`.

**9 — Restricted pointers — adapted for event-driven Arduino.**
One level of dereference; no pointer arithmetic gymnastics; don't hide
dereferences in macros/typedefs. **Honest deviation:** Holzmann bans function
pointers, but Arduino is built on callbacks — `attachInterrupt`, RTC alarm
callbacks, and `LowPower.attachInterruptWakeup` *are* function pointers/handlers
and can't be avoided. Don't contort code to dodge them. Instead contain the risk:
keep each callback tiny and single-purpose; in an **ISR** (including SERCOM and
RTC handlers), touch only `volatile` state, set a flag, and defer the real work
to `loop()` — no `Serial`, no `delay`, no allocation, and **no `float` math**
(see Rule on no-FPU below) inside an ISR.

**10 — Compile clean at maximum warnings; run a static analyzer.**
Set Arduino IDE → Preferences → **Compiler warnings = All** (PlatformIO:
`build_flags = -Wall -Wextra`). Treat every warning as a defect; if a warning is
"wrong," rewrite the code until it's trivially valid rather than ignoring it — the
warning is often right for a reason you haven't seen yet. Run **cppcheck** or
**clang-tidy** and aim for zero findings. State this expectation when handing over
code.

## MKR Zero field-reliability additions

Beyond the ten rules, these SAMD21/MKR-Zero specifics separate code that survives
in the field from code that merely compiles:

- **The `while(!Serial)` trap is the #1 field bug.** It is invisible on USB and
  fatal on battery. Bound it (Rule 2). Likewise bound `SD.begin()` and any sensor
  ready-wait.
- **No FPU — keep `float` out of hot paths.** The Cortex-M0+ emulates floating
  point in software, so it's slow. Use integer/fixed-point in ISRs, tight loops,
  and high-rate sampling; convert to `float` only for occasional display math. A
  `float` divide in a per-sample ISR can blow your timing budget.
- **Respect 3.3 V at the design level.** No code can save a pin that gets 5 V —
  but you *can* refuse to design around a 5 V input and flag the level-shifting
  need instead. Treat "this input might be 5 V" as a Critical wiring fault.
- **Flash writes are rare events, not loop work.** FlashStorage / EEPROM-emulation
  wears out flash and stalls the CPU; never write per-iteration (`storage.md`).
  For high-rate persistence, log to SD, not flash.
- **Close/flush the SD card before sleeping or on low battery.** An unclosed file
  can lose its last records; an SD write during a brownout can corrupt the card.
- **The status LED lies on battery.** `LED_BUILTIN` and the power LED are
  USB/VIN-powered and stay dark on battery — never use the LED as your only
  health indicator for a battery-powered node. Prefer logging the boot/reset
  cause.
- **Log the reset cause every boot.** Read the SAMD21 reset-cause register
  (`PM->RCAUSE.reg`) at startup and record it (to SD or Serial). In the field this
  is your only forensic trail for why a node rebooted (brownout? watchdog?
  external reset?).
- **Arm the watchdog for unattended runs.** A hang should reboot, not stay dead
  (`peripherals.md`). Don't feed it from inside an unbounded wait — and don't let
  a plain watchdog run across a long `deepSleep`, or it resets the board mid-sleep
  (an 8 s WDT turns a 5-min sleep into a reboot loop). Use a sleep-aware
  `Watchdog.sleep(ms)`, or disarm the watchdog before sleeping (`power.md`).
- **Only the variant-enabled pins can interrupt.** `attachInterrupt` works on
  D0/D1/D4–D9/A1/A2 (+ D11/SDA NMI) and silently no-ops elsewhere; validate
  against the table (`hardware.md`) before promising an ISR or a wake source on a
  given pin.

## Pre-flight checklist (run before emitting any sketch)

1. No unbounded `while(!Serial)`; every external-wait loop (SD, sensor) has a
   timeout and a defined timeout action.
2. Every meaningful return value is checked (`SD.begin`, `SD.open`, file truth,
   `Wire.endTransmission`, library `begin()`) — or `(void)`-cast with a reason.
3. No heap churn in `loop()`/ISRs/callbacks; large buffers are `static`;
   `String` growth replaced by fixed buffers + `snprintf`. SRAM budget fits 32 KB.
4. Inputs and sensor data are range/NaN-checked with explicit recovery; no input
   is assumed to exceed 3.3 V.
5. Pins are legal for their function on the MKR Zero, and any interrupt/wake pin
   is one the variant actually enables (cross-check `hardware.md`). LED uses
   `LED_BUILTIN`.
6. `loop()` passes are short and non-blocking; battery gaps use sleep, not
   `delay`; no `float` math in ISRs/high-rate loops.
7. ISRs/SERCOM/RTC handlers are flag-only — no `Serial`/`delay`/allocation/`float`.
8. Flash/FlashStorage writes are rare (not per-loop); SD files are closed/flushed
   before sleep.
9. Code is structured to compile clean at `-Wall -Wextra`.

If a sketch can't meet one of these, say so explicitly and explain the trade-off
— don't ship a hidden landmine.

## Reviewing code: rubric and tone

When the user asks you to review or debug their code, the tone is **clear,
direct, and critical**. You are a reviewer for a device that has to work
unattended — flattery helps no one.

- **Don't rubber-stamp.** If the code has problems, say so plainly. "Looks good!"
  on code with a `while(!Serial)` is a disservice — it works on the bench and
  dies on battery.
- **Lead with the worst.** Order findings by severity, most dangerous first.
- **Be specific and concrete.** For each finding: *what* (the construct), *where*
  (line/function), *the failure mode on real hardware*, and *the fix*. "This
  `while(!Serial)` hangs the node forever on battery because no USB host opens the
  port" beats "consider adding a timeout."
- **Always give the corrected form**, not just the complaint. Critical ≠ unkind;
  the goal is working code, so show what right looks like.
- **Rank with labels** so the user can triage:
  - **Critical** — will fail or hang in the field: unbounded waits
    (`while(!Serial)`, `while(!SD.begin())`), unchecked fatal returns, a 5 V
    assumption on an input, heap churn/`String` growth that fragments 32 KB,
    blocking the device so the watchdog can't help.
  - **Major** — wrong under realistic conditions: missing NaN/range checks, no
    retry bound, ADC left at 10-bit when 12 was intended, `float` math in an ISR,
    writing flash in `loop()`, an ISR on a pin the variant disables, SD file never
    closed/flushed.
  - **Minor** — correctness-adjacent: scope too wide, `String` where a buffer
    fits, missing `(void)` cast, LED on pin 13 instead of `LED_BUILTIN`.
  - **Nit** — style/clarity.
- **Acknowledge what's genuinely right, briefly** — but don't pad the review to
  soften it. One honest line, then the substance.

Map your review pass over the ten rules + the MKR-Zero field-reliability list
above; those are the checklist. End with the single highest-impact change if the
user only does one thing.
