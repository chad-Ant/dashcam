---
name: mkr-zero-arduino
description: >-
  Write, debug, and explain Arduino (C++) code for the Arduino MKR Zero
  (Microchip SAMD21G18A, ARM Cortex-M0+). Use this skill whenever the user works
  with an MKR Zero — any mention of "MKR Zero", "MKRZero", "ABX00012", "SAMD21",
  "SAMD21G18A", or "Cortex-M0+", and any task touching its GPIO/peripherals
  (I2C, SPI, UART, ADC, DAC, PWM), the onboard microSD card, the SERCOM system
  (extra serial/SPI/I2C ports), RTC and low-power standby, LiPo battery
  operation, or Arduino IDE / arduino-cli / PlatformIO setup. Also use it when
  the user just says "MKR" or "Zero" but the pins or 3.3 V logic point to an MKR
  Zero, or when SAMD21 code misbehaves for a board-specific reason (5 V on a
  3.3 V pin, ADC stuck at 10-bit, a while(!Serial) hang on battery, SERCOM
  pin-mux, or no-FPU float math). Prefer this skill over generic Arduino help
  whenever the target is an MKR Zero — its 3.3 V-only I/O, absent radio, 32 KB
  SRAM, single-channel DAC, and SERCOM-multiplexed buses trip up code written
  for a 5 V AVR or a radio board.
---

# Arduino MKR Zero (SAMD21 / Arduino framework)

The MKR Zero (Arduino code **ABX00012**) is built on the Microchip
**SAMD21G18A**: a 32-bit **ARM Cortex-M0+** at **48 MHz**, with **256 KB flash**
and only **32 KB SRAM**. It has a USB native port, an **onboard microSD slot**, a
LiPo battery charger, a 32.768 kHz crystal for the RTC, and an ATECC508A crypto
chip — but **no radio** (no WiFi, no Bluetooth). It is an excellent small 32-bit
board, but it is *not* a 5 V AVR and *not* a wireless board, and a large fraction
of Arduino example code on the internet misbehaves on it for predictable
reasons. The job of this skill is to produce code that compiles and runs on an
MKR Zero the first time, and to explain *why* when something the user found
elsewhere doesn't.

Treat both code you write and code you review as **mission-critical embedded
software**, not throwaway sketches — an MKR Zero is often deployed unattended (on
a LiPo, logging to its SD card, in something the user won't power-cycle by hand),
where a hang or an unchecked error means a dead node or a corrupt log file. The
full standard (a SAMD21 adaptation of Holzmann's *Power of Ten* safety-critical
rules) is in `references/mission-critical.md` — consult it for any non-trivial
sketch and for **every** code review.

## Two facts cause most of the trouble — internalize them first

1. **The MKR Zero runs at 3.3 V and its pins are NOT 5 V tolerant.** Arduino's
   own warning is blunt: applying more than 3.3 V to *any* I/O pin can **damage
   the board**. This is the #1 way MKR Zeros die — someone wires a 5 V sensor,
   relay module, or another 5 V Arduino's TX line straight to a pin. Coming from
   an Uno/Nano/Mega (5 V), this is the habit to break. 5 V devices need a level
   shifter or a divider; `VIN` accepts 5 V (to the regulator) but **no I/O pin
   does**. This is a hardware fact no code can fix — flag it whenever the wiring
   implies 5 V. See `references/hardware.md`.

2. **It's a no-radio SAMD21, not a classic AVR.** Versus an Uno/Nano: the ADC is
   **12-bit but defaults to 10-bit** (Uno compatibility) until you call
   `analogReadResolution(12)`; there is a **real 10-bit DAC** on **A0** (true
   analog out, not PWM); there is **no EEPROM** (use the FlashStorage library —
   and flash contents are erased on every re-upload); serial/SPI/I2C are provided
   by the **SERCOM** system (6 configurable modules, 4 already used, 2 free —
   adding ports needs `pinPeripheral()`, not just `begin()`); and the Cortex-M0+
   has **no FPU**, so `float`/`double` math is software-emulated and slow. Versus
   a radio board (MKR WiFi 1010, MKR 1000): there is **no `WiFi`/`BLE`** — if the
   user needs wireless, they have the wrong board.

## Quick decision guide — where to look

Read the reference file for the task at hand rather than guessing. Each is
self-contained with copy-pasteable, MKR-Zero-correct examples.

| The user is working on…                                          | Read this                        |
|------------------------------------------------------------------|----------------------------------|
| Pinout, safe/taken pins, ADC/DAC, current limits, chip specs     | `references/hardware.md`         |
| GPIO, analogRead/Write, DAC, I2C, SPI, UART, interrupts, timers  | `references/peripherals.md`      |
| Adding extra UART/SPI/I2C ports, SERCOM pin-mux, `pinPeripheral` | `references/sercom.md`           |
| Onboard microSD (`SD.h`), flash-based persistence (no EEPROM)    | `references/storage.md`          |
| RTC, alarms, sleep/standby low-power, LiPo battery operation     | `references/power.md`            |
| Installing the core, board settings, the bootloader, flashing    | `references/project-setup.md`    |
| Writing robust code, or **reviewing/debugging** user code        | `references/mission-critical.md` |

For a multi-part request (e.g. "read a sensor over I2C, log it to the SD card,
then sleep until the next RTC alarm"), read each relevant file and compose the
pieces.

## User-supplied knowledge & precedence

The `knowledge/` folder holds **guideline documents the user adds over time** —
house style, project conventions, client requirements, datasheets, lessons
learned. Its contents are not fixed: before a non-trivial generation or review
task, check whether `knowledge/` contains anything relevant (list it, read what
applies) and fold that guidance in. New documents can be dropped in at any time
without editing this skill; see `knowledge/README.md`.

**Precedence is strict and must be honored in this order:**

1. **`references/mission-critical.md` — the NASA *Power of Ten* adaptation.**
   Highest authority. If a `knowledge/` document contradicts it, the
   NASA-derived rule **wins**. Don't silently drop the user's guideline — apply
   the safety rule, then tell the user a conflict occurred, name both sides, and
   say briefly why the safety rule prevails.
2. **Hardware reality** (`references/`, verified against the SAMD21 datasheet, the
   MKR Zero pinout, and the stock `variant.cpp`). No guideline can override
   physics — 5 V on an I/O pin, a non-existent second DAC, or an `attachInterrupt`
   on a pin the variant leaves disabled stays impossible no matter what a document
   says.
3. **User `knowledge/` documents.** Authoritative for everything the tiers above
   don't constrain: style, structure, library choices, naming, logging format,
   project requirements. Apply these directly and without fanfare when they
   don't conflict (the normal case).

## The most common code mistake: an unbounded `while (!Serial)`

Almost every SAMD21 tutorial opens with this, and it is a field landmine:

```cpp
// ❌ Hangs FOREVER when the board runs on battery (no USB host ever attaches)
void setup() {
  Serial.begin(115200);
  while (!Serial);            // waits for the USB CDC port to be opened
  Serial.println("up");
}
```

On USB it's fine. On a LiPo with no computer attached, `Serial` never becomes
true, so the board hangs in `setup()` and nothing runs — the classic "works on
my desk, dead in the field." Bound it:

```cpp
// ✅ Wait briefly for USB, then carry on regardless
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) { /* wait up to 2 s for a host */ }
  Serial.println("up");      // harmless if no host is listening
}
```

If a user reports "my MKR Zero works over USB but does nothing on battery," this
unbounded wait is the first thing to suspect. (It's Rule 2 in
`mission-critical.md`.)

## How to approach a coding task

- **Pins are fixed by the board; confirm only genuinely ambiguous ones.** The
  MKR Zero's pin map is fixed (unlike multi-vendor chip boards), so use it
  directly: `Wire` on D11/D12, `SPI` on D8/D9/D10, `Serial1` on D13/D14, DAC on
  A0, `LED_BUILTIN` for the onboard LED. Full map in `hardware.md`.
- **Use `LED_BUILTIN`, never pin 13.** On the MKR Zero the onboard LED is on a
  dedicated pin (32), **not** 13 as on a Uno. `digitalWrite(13, HIGH)` does
  nothing useful here. Also note the LED is powered from USB/VIN, **not** the
  battery — it stays dark on battery even when the board is running fine, so
  don't rely on it for battery-powered status.
- **Respect 3.3 V on every input.** Treat any signal that *might* be 5 V as a
  hardware bug to flag, not something to read directly. Mention level shifting
  when the wiring implies it.
- **Unlock the 12-bit ADC explicitly.** Call `analogReadResolution(12)` if the
  user wants the full range; otherwise say you're leaving the 10-bit Uno-compat
  default in place. Be explicit either way.
- **The DAC is real and lives on A0.** For true analog output use
  `analogWriteResolution(10); analogWrite(A0, value);` — don't reach for an RC
  filter on a PWM pin unless A0 is already taken.
- **Mind 32 KB of SRAM.** This is ~12× smaller than an ESP32's. Big buffers go
  `static`/global, prefer fixed buffers + `snprintf` over `String`
  concatenation, and never churn the heap in `loop()` — fragmentation in 32 KB
  bites fast. (`mission-critical.md`, Rule 3.)
- **The onboard SD card is a separate SPI bus.** `SD.begin(SDCARD_SS_PIN)` (or
  just `SD.begin()` — it auto-detects on the MKR Zero). It does **not** share the
  D8/D9/D10 header SPI; both can run at once. See `storage.md`.
- **`attachInterrupt` works on only a fixed set of pins.** The stock variant
  enables it on D0, D1, D4, D5, D6, D7, D8, D9, A1, A2 (plus D11/SDA as NMI) and
  marks every other pin `EXTERNAL_INT_NONE`, so an ISR requested elsewhere
  silently never fires. Check the EXTINT table in `hardware.md` before promising
  an ISR on a pin.
- **Avoid `float` in hot paths.** No FPU → software-emulated floating point.
  Prefer integer/fixed-point in ISRs, tight loops, and high-rate sampling.
- **Check what functions return.** `SD.begin()`, `file.open()`,
  `Wire.endTransmission()`, `sensor.begin()` — validate and recover explicitly
  rather than charging ahead on a failure. (Rule 7.)

## Output format

For anything beyond a one-line tweak, deliver a **complete, compilable sketch**
the user can paste into the Arduino IDE and upload — full `#include`s,
`setup()`, and `loop()`, with concrete MKR Zero pins and brief inline comments at
the points that are SAMD21-specific or non-obvious. Before emitting it, run the
**pre-flight checklist** in `mission-critical.md` (bounded waits — including the
`while(!Serial)` one, checked returns, no heap churn in `loop()`, NaN/range
checks, legal pins/EXTINT lines, non-blocking `loop()`, clean at `-Wall`). After
the code, add a short plain-language note covering: which pins/wiring it assumes,
any libraries to install (e.g. RTCZero, FlashStorage), and caveats (3.3 V levels,
battery behavior). Keep prose tight — the working sketch is the deliverable. When
the user explicitly wants just a snippet or an explanation, match that instead;
don't force a full sketch onto a conceptual question.

## Reviewing or debugging the user's code

When asked to review or fix existing code, the tone is **clear, direct, and
critical** — you're reviewing software for a device that must run unattended, so
don't rubber-stamp it. Lead with the most dangerous issues, label findings by
severity (Critical / Major / Minor / Nit), and for each give the construct, the
failure mode *on real hardware*, and the corrected form — criticism always comes
with the fix. Run your pass over the ten rules plus the MKR-Zero field-
reliability list in `mission-critical.md`, and watch hardest for the field-fatal
patterns: unbounded waits (`while(!Serial)`, `while(!SD.begin())`), unchecked
fatal returns, 5 V assumptions on inputs, heap churn/`String` growth in 32 KB,
an ISR attached to a pin the variant disables, and `float` math in an ISR or
high-rate loop. End
with the single highest-impact change. Acknowledge what's genuinely correct in a
line — then get to the substance.

## Going deeper than these notes

The hardware facts in these reference files are checked against the **Microchip
SAMD21 family datasheet** and the **Arduino MKR Zero pinout (ABX00012)**, and
reflect their documented limits (3.3 V I/O, the 12-bit ADC / 10-bit DAC, the six
SERCOM modules and their default MKR Zero assignment, the EXTINT mapping). For
register-level detail beyond what's here — exact register bitfields, the GCLK
clock tree, peripheral timing, NVM/flash specifics — consult the **SAMD21
datasheet** and the **MKR Zero pinout** directly. When a user's question turns on
a register or a hardware edge case not covered here, say so and point them there
rather than guessing.
