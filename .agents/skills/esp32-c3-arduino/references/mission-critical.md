# Mission-critical coding & review (ESP32-C3 / Arduino)

A C3 is usually deployed where **no operator can intervene** — on a battery, in
a wall, on a pole. A hang, a heap exhaustion, or an unchecked error isn't a
stack trace on someone's screen; it's a dead node until someone physically
power-cycles it. So treat both **code you generate** and **code you review** as
safety-critical embedded software, not hobby sketches.

The standard below adapts Gerard Holzmann's *Power of Ten* (NASA/JPL) to the
ESP32-C3 Arduino (C++) reality. The Power of Ten targets C on spacecraft; most
of it transfers directly, but a few rules need judgment in an event-driven
Arduino world. Adapt with reasoning — don't cargo-cult. Where a rule is relaxed
below, the relaxation is stated explicitly so it's a decision, not an accident.

## Posture

- **Default to strict.** When a stricter form costs a few lines but removes a
  field-failure mode, take it. Availability of an unreachable device is worth
  more than brevity.
- **Generation:** every sketch must satisfy the pre-flight checklist at the
  bottom before you emit it.
- **Review:** be **clear and critical**. Do not rubber-stamp. See the review
  rubric and tone section.

## The ten rules, adapted

**1 — Simple control flow. No recursion.**
Keep control flow flat and analyzable. No `goto`, no `setjmp`/`longjmp`, no
recursion (the C3's per-task stack is small and fixed — recursion risks a stack
overflow that manifests as a mysterious reboot). Early `return` on error is
fine and usually clearer than nesting.

**2 — Every loop has a fixed, provable upper bound. (Highest priority on a C3.)**
Any wait on the outside world must time out and have a defined action on
timeout. `while (WiFi.status() != WL_CONNECTED) {}` and
`while (!sensor.ready()) {}` are field-fatal: if the AP is down or the sensor is
unplugged, the node hangs forever. Always bound it:

```cpp
const uint32_t WIFI_TIMEOUT_MS = 15000;
uint32_t t0 = millis();
while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_TIMEOUT_MS) {
  delay(200);
}
if (WiFi.status() != WL_CONNECTED) { /* defined fallback: log + sleep + retry */ }
```

The two intentional exceptions are `loop()` and FreeRTOS task functions, which
are *meant* to run forever — for those, the inverse rule applies: make sure they
can't accidentally fall through or exit. Keep each `loop()` pass short and
non-blocking (Rule on the single core, below).

**3 — No dynamic allocation after initialization.**
Allocate in `setup()`; never churn the heap in `loop()`, ISRs, or callbacks.
The practical C3 forms:
- Prefer fixed buffers and `snprintf` over `String` concatenation. `String s =
  a + b + c;` in `loop()` fragments a heap that already shares ~400 KB with the
  WiFi/BLE stacks — fragmentation, not exhaustion, is what kills long-running
  nodes.
- Large arrays go `static`/global, not on the stack — task stacks are small and
  fixed; a big local array silently overflows it.
- Know that `WiFi`, `WiFiClientSecure` (TLS), and the Bluedroid BLE stack
  allocate substantial heap internally; budget for it and watch
  `ESP.getFreeHeap()`. On tight builds, prefer NimBLE (see `ble.md`).

**4 — Keep functions short and single-purpose (~60 lines).**
Factor `setup()`/`loop()` into named helpers — `connectWiFi()`, `readSensor()`,
`publish()`, `goToSleep()`. A function you can see whole is one you can verify
whole, and it makes the review tractable.

**5 — Defend with checks; recover explicitly.**
Holzmann wants ≥2 assertions/function. On a field device a hard `assert()`
aborts → reboot, which trades availability for fail-stop — sometimes right, but
usually you want **graceful recovery**, not a crash. So the adapted rule:
validate every parameter and every piece of external data, and take an
*explicit* recovery action.
- Check sensor outputs for sentinels: `if (isnan(t)) { ... }`.
- Range-check before trusting: a value outside the datasheet range is a fault,
  not a reading.
- On failure, do something defined: bounded retry, safe state, or sleep-and-
  retry-next-cycle — never silently continue with bad data.

**6 — Smallest possible scope.**
Declare where used; `const`/`constexpr` by default; never reuse one variable for
two meanings. Narrow scope = fewer places a bad value can come from.

**7 — Check every return value; validate every parameter. (The other top priority.)**
This is the most-violated and most-valuable rule. On the C3 that means:
`WiFi.begin`/`status`, `http.GET`/`POST` codes, `sensor.begin()`,
`Wire.endTransmission()`, `esp_now_init/add_peer/send` (they return
`esp_err_t`), `ledcAttach` (returns `bool`), file ops. If you deliberately
ignore one, cast to `(void)` and say why in a comment. Validate inputs too —
including **pin numbers against the C3 constraint table** (`hardware.md`): a pin
in GPIO12–17 or ADC2/GPIO5 is a bug caught at review, not in the field.

**8 — Restrained preprocessor.**
`#define` for simple constants and `#include` guards only. Prefer
`constexpr`/`const` and `inline` functions to function-like macros (no
hidden side effects, type checking intact). Minimize `#ifdef`.

**9 — Restricted pointers — adapted for event-driven Arduino.**
One level of dereference; no pointer arithmetic gymnastics; don't hide
dereferences in macros/typedefs. **Honest deviation:** Holzmann bans function
pointers, but Arduino is built on callbacks — `attachInterrupt`, `WiFi.onEvent`,
and BLE callbacks *are* function pointers/handlers and can't be avoided. Don't
contort code to dodge them. Instead contain the risk: keep each callback tiny
and single-purpose; in an **ISR**, mark it `IRAM_ATTR`, touch only `volatile`
state, set a flag and defer the real work to `loop()` — no `Serial`, no
`delay`, no allocation inside an ISR.

**10 — Compile clean at maximum warnings; run a static analyzer.**
Set Arduino IDE → Preferences → **Compiler warnings = All** (PlatformIO:
`build_flags = -Wall -Wextra`). Treat every warning as a defect; if a warning is
"wrong," rewrite the code until it's trivially valid rather than ignoring it —
the warning is often right for a reason you haven't seen yet. Run **cppcheck**
or **clang-tidy** and aim for zero findings. State this expectation when handing
over code.

## ESP32-C3 field-reliability additions

Beyond the ten rules, these C3 specifics separate code that survives in the
field from code that merely compiles:

- **Don't block the single core.** One RISC-V core runs your code *and* services
  WiFi/BLE. The precise failure mode: a **busy-loop or long synchronous work
  that never yields** starves the idle task and trips the **task watchdog** →
  reset. Note `delay()` is *not* that — Arduino's `delay()` calls `vTaskDelay`,
  which yields, so it feeds the watchdog and lets the WiFi/BLE tasks run. But a
  long `delay()` still blocks *your* task's own logic, so it's poor design for
  responsiveness. Keep `loop()` passes short; for long work, chunk it or yield
  with `vTaskDelay`, and never spin in a tight `while` without a yield.
- **Log the reset/wake reason every boot** (`esp_reset_reason()`, wake cause).
  In the field this is your only forensic trail for why a node rebooted.
- **Bounded retries with backoff, then a defined fallback.** No infinite
  reconnect spins. Try N times, then sleep and retry next cycle.
- **Budget power and heap.** Weak USB/battery → brownout reset during WiFi TX
  spikes; WiFi+BLE+TLS together can exhaust heap. Both are design constraints,
  not afterthoughts.
- **Validate pins against the constraints** in `hardware.md` (flash pins
  GPIO12–17, ADC2/GPIO5, deep-sleep wake only GPIO0–5, strapping GPIO2/8/9).

## Pre-flight checklist (run before emitting any sketch)

1. Every external-wait loop has a timeout and a defined timeout action.
2. Every meaningful return value is checked (or `(void)`-cast with a reason).
3. No heap churn in `loop()`/ISRs/callbacks; large buffers are `static`.
4. Inputs and sensor data are range/NaN-checked with explicit recovery.
5. Pins are legal for their function on the C3 (cross-check `hardware.md`).
6. `loop()` passes are short and non-blocking; no long `delay()` blocking the core.
7. ISRs are `IRAM_ATTR`, flag-only, no allocation/Serial/delay.
8. Code is structured to compile clean at `-Wall -Wextra`.

If a sketch can't meet one of these, say so explicitly and explain the
trade-off — don't ship a hidden landmine.

## Reviewing code: rubric and tone

When the user asks you to review or debug their code, the tone is **clear,
direct, and critical**. You are a reviewer for a device that has to work
unattended — flattery helps no one.

- **Don't rubber-stamp.** If the code has problems, say so plainly. "Looks good!"
  on code with an unbounded reconnect loop is a disservice.
- **Lead with the worst.** Order findings by severity, most dangerous first.
- **Be specific and concrete.** For each finding: *what* (the construct),
  *where* (line/function), *the failure mode on real hardware*, and *the fix*.
  "This `while (WiFi.status()...)` hangs the node forever if the AP is down, and
  `delay()` won't let the watchdog recover it" beats "consider adding a timeout."
- **Always give the corrected form**, not just the complaint. Critical ≠ unkind;
  the goal is working code, so show what right looks like.
- **Rank with labels** so the user can triage:
  - **Critical** — will fail or hang in the field (unbounded loops, unchecked
    fatal returns, illegal pins, heap churn that fragments, blocking the core).
  - **Major** — wrong under realistic conditions (missing NaN/range checks, no
    retry bound, ADC2 use, 16-bit PWM).
  - **Minor** — correctness-adjacent (scope too wide, `String` where a buffer
    fits, missing `(void)` cast).
  - **Nit** — style/clarity.
- **Acknowledge what's genuinely right, briefly** — but don't pad the review to
  soften it. One honest line, then the substance.

Map your review pass over the ten rules + the C3 field-reliability list above;
those are the checklist. End with the single highest-impact change if the user
only does one thing.
