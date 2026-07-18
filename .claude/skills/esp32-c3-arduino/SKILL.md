---
name: esp32-c3-arduino
description: >-
  Write, debug, and explain Arduino (C++) code for the Espressif ESP32-C3
  microcontroller. Use this skill whenever the user is working with an ESP32-C3
  in the Arduino framework — including any mention of "ESP32-C3", "ESP32C3",
  "C3", or specific C3 boards (XIAO ESP32C3, ESP32-C3-DevKitM-1, Lolin C3,
  Super Mini C3, etc.), and any task that touches GPIO/peripherals (I2C, SPI,
  ADC, PWM), WiFi, Bluetooth LE, deep sleep / power saving, or Arduino IDE /
  PlatformIO project setup for the chip. Also use it when the user just says
  "ESP32" but the board, pin numbers, or constraints point to a C3, or when
  ESP32 code is failing to compile and the cause may be a C3-specific quirk or
  an Arduino-core 3.x API change. Prefer this skill over generic Arduino help
  any time the target is a C3 — its single RISC-V core, missing DAC, BLE-only
  radio, and distinct deep-sleep wake sources trip up code written for the
  classic ESP32.
---

# ESP32-C3 (Arduino framework)

The ESP32-C3 is a low-cost single-core 32-bit **RISC-V** SoC (≤160 MHz, ~400 KB
SRAM) with **2.4 GHz WiFi 4** and **Bluetooth 5 LE**. It is cheap and capable,
but it is *not* a drop-in for the classic dual-core Xtensa ESP32, and a large
fraction of ESP32 example code on the internet fails on a C3 for predictable
reasons. The job of this skill is to produce code that actually compiles and
runs on a C3 the first time, and to explain *why* when something the user found
elsewhere doesn't.

Treat both code you write and code you review as **mission-critical embedded
software**, not throwaway sketches — a C3 is usually deployed unattended, where
a hang or an unchecked error means a dead node until someone power-cycles it by
hand. Generation follows a pre-flight checklist; review is **clear and
critical**, never a rubber stamp. The full standard (an ESP32-C3 adaptation of
Holzmann's *Power of Ten* safety-critical rules) is in
`references/mission-critical.md` — consult it for any non-trivial sketch and for
every code review.

Two facts cause most of the trouble, so internalize them before writing code:

1. **The C3 is not a classic ESP32.** It is single-core, has **no DAC**, has
   **no Bluetooth Classic** (BLE only), **no touch sensor**, and a different set
   of deep-sleep wake sources. Code using `dacWrite`, `BluetoothSerial`,
   `touchRead`, `esp_sleep_enable_ext0_wakeup`, or core-1 task pinning will not
   work. See `references/hardware.md`.
2. **Arduino-ESP32 core 3.x changed many APIs.** The current core (3.3.x, built
   on ESP-IDF 5.5) merged and renamed a lot of functions versus the old 2.x
   examples that still dominate search results — most notably PWM/LEDC, ADC,
   I2S, and BLE. Writing 2.x-style code produces confusing compile errors. The
   correct 3.x forms are in the reference files; the most common one is below.

## Quick decision guide — where to look

Read the reference file for the task at hand rather than guessing. Each is
self-contained with copy-pasteable, 3.x-correct examples.

| The user is working on…                                   | Read this                     |
|-----------------------------------------------------------|-------------------------------|
| Pinout, which pins are safe, ADC channels, chip limits    | `references/hardware.md`      |
| GPIO, I2C, SPI, analogRead, PWM/LEDC, UART, interrupts     | `references/peripherals.md`   |
| WiFi STA/AP, HTTP(S) client, web server, OTA, ESP-NOW      | `references/wifi.md`          |
| Bluetooth LE server/client, advertising, NimBLE            | `references/ble.md`           |
| Deep/light sleep, wake sources, low-power patterns         | `references/power.md`         |
| Installing the core, board settings, flashing, PlatformIO  | `references/project-setup.md` |
| Writing robust code, or **reviewing/debugging** user code  | `references/mission-critical.md` |

For a multi-part request (e.g. "read a sensor over I2C and post it to a server,
then deep sleep"), read each relevant file and compose the pieces.

## User-supplied knowledge & precedence

The `knowledge/` folder holds **guideline documents the user adds over time** —
house style, project conventions, client requirements, internal best practices.
Its contents are not fixed: before a non-trivial generation or review task,
check whether `knowledge/` contains anything relevant (list it, read what
applies) and fold that guidance in. New documents can be dropped in at any time
without editing this skill; see `knowledge/README.md`.

**Precedence is strict and must be honored in this order:**

1. **`references/mission-critical.md` — the NASA *Power of Ten* adaptation.**
   Highest authority. If a `knowledge/` document contradicts it, the
   NASA-derived rule **wins**. Don't silently drop the user's guideline — apply
   the safety rule, then tell the user a conflict occurred, name both sides, and
   say briefly why the safety rule prevails.
2. **Hardware reality** (`references/`, verified against the C3 TRM). No
   guideline can override physics — ADC2, flash pins, or 16-bit LEDC stay
   forbidden no matter what a document says.
3. **User `knowledge/` documents.** Authoritative for everything the tiers above
   don't constrain: style, structure, library choices, naming, logging format,
   project requirements. Apply these directly and without fanfare when they
   don't conflict (the normal case).

## The single most common mistake: PWM (LEDC)

Almost every pre-2024 example uses the old API. On core 3.x it does not compile.

```cpp
// ❌ OLD (core 2.x) — DO NOT USE on a C3 today
ledcSetup(0, 5000, 8);        // channel, freq, resolution
ledcAttachPin(7, 0);          // pin, channel
ledcWrite(0, 128);            // channel, duty

// ✅ CURRENT (core 3.x) — channel is auto-assigned, you address the PIN
ledcAttach(7, 5000, 8);       // pin, freq(Hz), resolution(bits)
ledcWrite(7, 128);            // pin, duty (0..2^bits-1)
// or simplest of all, plain Arduino:
analogWrite(7, 128);          // now works directly on ESP32 in 3.x
```

If a user pastes code that calls `ledcSetup`/`ledcAttachPin` and reports a
compile error, this is almost certainly why — convert it and explain the change.
The C3 has only **6 LEDC channels** (the classic ESP32 has 16), which matters
when many PWM outputs are needed simultaneously.

## How to approach a coding task

- **Confirm the board only if pin numbers depend on it.** The chip's
  constraints are fixed, but broken-out pins and onboard LEDs differ between a
  XIAO ESP32C3, a DevKitM-1, and a "Super Mini." If the user names the board,
  use its mapping; if not and the answer needs a specific pin, pick a known-safe
  GPIO and say which one and why (see the safe-pin list in `hardware.md`).
- **Avoid the dangerous pins by default.** Never hand out GPIO12–17 (SPI flash)
  or assume GPIO18/19 are free (native USB D-/D+). GPIO2, GPIO8, GPIO9 are
  strapping pins — usable, but explain the boot-time caveat if you choose them.
- **Prefer explicit peripheral pins.** Write `Wire.begin(SDA_PIN, SCL_PIN)` and
  `SPI.begin(sck, miso, mosi, ss)` with concrete numbers rather than relying on
  board defaults, which vary. It makes the sketch portable and self-documenting.
- **Analog input: use GPIO0–GPIO4 only.** Those are ADC1. The C3's ADC2
  (GPIO5) is documented in the SoC errata as not working properly — avoid it
  entirely, not just when WiFi is on.
- **Single core means core 0.** Use `xTaskCreate(...)` or, if pinning is truly
  needed, core 0 / `tskNO_AFFINITY`. There is no core 1.
- **Bound every wait on the outside world.** No `while (WiFi.status() != ...)`
  or `while (!ready())` without a timeout and a defined fallback — on an
  unattended C3 an unbounded wait is a permanent hang. (Rule 2 in
  `mission-critical.md`.)
- **Check what functions return.** `begin()`, connection status, HTTP codes,
  `Wire.endTransmission()`, `esp_now_*`, `ledcAttach` — validate them and recover
  explicitly rather than charging ahead on a failure. (Rule 7.)

## Output format

For anything beyond a one-line tweak, deliver a **complete, compilable sketch**
that the user can paste into the Arduino IDE and upload — full `#include`s,
`setup()`, and `loop()`, with concrete pin numbers and brief inline comments at
the points that are C3-specific or non-obvious. Before emitting it, run the
**pre-flight checklist** in `mission-critical.md` (bounded waits, checked
returns, no heap churn in `loop()`, NaN/range checks, legal pins, non-blocking
`loop()`, clean at `-Wall`). After the code, add a short plain-language note
covering: which board/pins it assumes, the relevant Arduino IDE board settings
if they matter (e.g. "USB CDC On Boot"), and any wiring or caveats. Keep prose
tight — the working sketch is the deliverable. When the user explicitly wants
just a snippet or an explanation, match that instead; don't force a full sketch
onto a conceptual question.

Always write **core 3.x** code. If you genuinely need a 2.x form (e.g. the user
is pinned to an old core), say so explicitly and explain the difference.

## Reviewing or debugging the user's code

When asked to review or fix existing code, the tone is **clear, direct, and
critical** — you're reviewing software for a device that must run unattended, so
don't rubber-stamp it. Lead with the most dangerous issues, label findings by
severity (Critical / Major / Minor / Nit), and for each give the construct, the
failure mode *on real hardware*, and the corrected form — criticism always comes
with the fix. Run your pass over the ten rules plus the C3 field-reliability
list in `mission-critical.md`, and watch hardest for the field-fatal patterns:
unbounded wait loops, unchecked fatal return values, illegal pins (flash pins,
ADC2/GPIO5, deep-sleep wake outside GPIO0–5), heap churn in `loop()`, and tight
busy-loops that never yield and starve the single core (tripping the watchdog). End with the
single highest-impact change. Acknowledge what's genuinely correct in a line —
then get to the substance.

## Going deeper than these notes

The hardware facts in these reference files are checked against Espressif's
**ESP32-C3 Technical Reference Manual (v1.4)** and reflect its documented
limits and errata (e.g. the broken ADC2 controller, the 14-bit LEDC ceiling,
the GPIO0–5 deep-sleep wake domain). For register-level detail beyond what's
here — exact register bitfields, peripheral timing, the full errata list —
consult the official **ESP32-C3 Technical Reference Manual** and **ESP32-C3
Datasheet** directly. When a user's question turns on a register or a hardware
edge case not covered here, say so and point them there rather than guessing.
