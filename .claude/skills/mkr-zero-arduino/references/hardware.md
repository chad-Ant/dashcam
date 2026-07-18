# MKR Zero / SAMD21 hardware reference

Constraints here are fixed by the SAMD21G18A silicon and the MKR Zero board
wiring. They hold for every MKR Zero. Checked against the **SAMD21 family
datasheet** and the **Arduino MKR Zero pinout (ABX00012)**.

## Core specs

- **MCU:** Microchip **SAMD21G18A**, 32-bit **ARM Cortex-M0+**, **48 MHz**.
  **No FPU** — `float`/`double` are software-emulated (slow; keep them out of
  ISRs and high-rate loops).
- **Flash:** 256 KB. **SRAM:** **32 KB** (small — budget memory carefully).
- **No EEPROM.** Persistence is via flash emulation (FlashStorage library); see
  `storage.md`. Flash is **erased on every sketch upload**.
- **Radio:** **none.** No WiFi, no Bluetooth. (That's the MKR WiFi 1010 / MKR
  1000 / MKR NB 1500, not this board.) Don't write `WiFi`/`BLE` code.
- **USB:** native full-speed device (PA24 D-, PA25 D+). `Serial` is USB CDC.
- **Onboard:** microSD slot (dedicated SPI, SERCOM4), LiPo charger, 32.768 kHz
  crystal (RTC/low-power), ATECC508A crypto chip (on the I2C bus).
- **Digital I/O:** 22 pins. **PWM:** 12 pins. **Analog in:** 7 (A0–A6).
  **Analog out:** 1 (DAC, 10-bit, on A0). **External interrupts:** EIC with 16
  EXTINT lines (0–15) + NMI, shared across pins (see the EXTINT table below).

## The 3.3 V rule (most important hardware fact)

**The MKR Zero runs at 3.3 V and NO I/O pin is 5 V tolerant.** Arduino's
datasheet warning: applying more than 3.3 V to any I/O pin can damage the board.
The SAMD21 datasheet puts hard numbers on it (Table 38-1, Absolute Maximum
Ratings): a pin's voltage must stay within **GND−0.3 V to VDD+0.3 V** — about
**3.6 V** with a 3.3 V rail — and absolute-max VDD itself is **3.8 V**. The chip
operates at 1.62–3.63 V (3.3 V nominal). 5 V on a pin is VDD+1.7 V, far past
absolute max → permanent damage. Practical consequences:

- A 5 V sensor/module output, or a 5 V Arduino's TX line, wired straight to a pin
  can **kill the chip**. Use a logic-level shifter (for bidirectional I2C/SPI) or
  a resistor divider (for a single analog/digital input).
- **Power pins are different from I/O.** `VIN` accepts a **regulated 5 V** (max
  6 V) to the onboard regulator. `5V` is an *output* (USB/VIN passthrough; ~3.7 V
  on battery) — never feed it as an input. `VCC`/`3V3` is the regulated 3.3 V
  rail (≤600 mA). **Never** put more than 5 V on VIN, and never put 5 V on any
  I/O pin.
- The JST connector is **LiPo-only** (single cell, 3.7 V, ≥700 mAh — the charger
  pushes ~350 mA; smaller cells can overheat). Don't connect anything else there.

## Per-pin and per-rail current limits

**Lower than a 5 V AVR**, so don't drive loads directly. The per-group figures
are the datasheet's per-cluster limits (Table 38-1, where a "cluster" is a group
of GPIOs sharing a VDD/GND pair); the per-pin figure is Arduino's design
guideline:

- **Max 7 mA per individual pin** (Arduino guideline).
- **Max 46 mA total source / 65 mA total sink per pin group** (datasheet
  per-cluster limit).
- 3.3 V rail and 5 V pin: up to **600 mA** each (board-level, from the regulator
  / USB).

Drive anything beyond an LED-with-resistor through a transistor/MOSFET or a
driver, and respect the per-group budget when several pins switch at once.

## Default pin map (use these directly)

The MKR Zero's mapping is fixed. The Arduino library objects below are wired to
these pins out of the box:

| Function          | Arduino pin(s)         | SAMD21 port      | Backing SERCOM   |
|-------------------|------------------------|------------------|------------------|
| **I2C** (`Wire`)  | D11 = SDA, D12 = SCL   | PA08, PA09       | **SERCOM2**      |
| **SPI** (`SPI`)   | D8 = COPI/MOSI, D9 = SCK, D10 = CIPO/MISO | PA16, PA17, PA19 | **SERCOM1** |
| **UART** (`Serial1`) | D14 = TX, D13 = RX  | PB22, PB23       | **SERCOM5**      |
| **microSD SPI**   | internal (PA12/13/14/15), CD on PA27 | —     | **SERCOM4** (dedicated) |
| **USB CDC** (`Serial`) | native USB        | PA24/PA25        | USB              |
| **DAC** (10-bit)  | A0 = DAC0              | PA02             | DAC              |
| **AREF**          | AREF                   | PA03             | —                |
| **Onboard LED**   | `LED_BUILTIN` (pin 32) | PB08             | — (USB/VIN powered) |

> `CIPO`/`COPI` are the current names for what older docs call `MISO`/`MOSI`.

**Analog inputs:** A0–A6 = D15–D21 (ports PA02, PB02, PB03, PA04, PA05, PA06,
PA07). A0 doubles as the DAC output. A3 (D18) and A4 (D19) are also PWM-capable.

**SERCOM headroom:** of the six SERCOM modules, **SERCOM1/2/4/5 are already in
use** (SPI, I2C, SD, UART respectively). **SERCOM0 and SERCOM3 are free** for
extra ports — see `sercom.md`. (The Arduino core reaches I2C and the SD card via
the pins' *alternate* SERCOM mux, which is why `Wire` lands on SERCOM2 and the SD
card on SERCOM4 rather than the SERCOM0/SERCOM2 you might guess from the silicon's
primary column.)

## What the MKR Zero does NOT have (the AVR/ESP trap)

- **No 5 V tolerance** (covered above) — the single biggest difference from a
  Uno/Nano/Mega.
- **No EEPROM** — `EEPROM.h` is flash-emulated and lost on re-upload; use
  FlashStorage (`storage.md`).
- **No radio** — no `WiFi`/`BLE` libraries apply.
- **No FPU** — float math is emulated; prefer integers in hot paths.
- **Pin 13 is not the LED** — use `LED_BUILTIN` (pin 32).
- **Not every pin can interrupt** — see the EXTINT section below.

## ADC (analog input)

- **12-bit hardware**, but the Arduino core **defaults to 10-bit** (0–1023) for
  Uno compatibility. Call `analogReadResolution(12)` for full 0–4095. The board
  also supports 8-bit. Be explicit about which you're using.
- **7 inputs: A0–A6.** Reference defaults to the 3.3 V rail; `analogReference()`
  can select internal references or the external `AREF` pin (PA03) — changing the
  reference rescales every reading, so set it once before sampling.
- A0 is shared with the DAC output — you can't read analog on A0 while driving
  the DAC there.
- The SAR ADC is fast but not laboratory-accurate. The datasheet rates it at
  **~10.5 effective bits (ENOB)** despite the 12-bit nominal resolution, with
  total unadjusted error up to ~13 LSB and mV-level offset/gain error
  (Table 38-11), at roughly **350 ksps** (up to ~1 Msps under other settings).
  For real voltage accuracy, average several samples and calibrate against a
  known reference rather than trusting a raw count.
- **Battery monitoring:** the board ties the LiPo voltage to the ADC through a
  divider, so battery voltage is readable in software. Confirm the exact
  pin/divider against the MKR Zero schematic before relying on the number.

## DAC (analog output) — a real one, on A0

Unlike an ESP32-C3 (no DAC) or a classic AVR (PWM only), the SAMD21 has a **true
10-bit DAC** on **A0 (DAC0)**:

```cpp
analogWriteResolution(10);     // DAC is 10-bit (0..1023)
analogWrite(A0, 512);          // ~1.65 V out (half of 3.3 V)
```

The datasheet (§35, Table 38-15) specifies a **single channel, 10-bit, up to
350 ksps**, with a **linear output range of 0.05 V to VDDANA−0.05 V** (so ~0.05 V
to ~3.25 V on a 3.3 V rail — not quite rail-to-rail) and a **minimum resistive
load of 5 kΩ** (max ~100 pF). It's a low-current source: buffer it with an op-amp
to drive any real load. Use this for smooth analog (audio, control voltages,
reference levels) instead of PWM+RC.

## PWM (TCC/TC timers)

- **12 PWM-capable pins:** D0–D8, D10, A3 (D18), A4 (D19). Use
  `analogWrite(pin, duty)`; default duty resolution is 8-bit (0–255), adjustable
  with `analogWriteResolution()`.
- PWM is generated by the SAMD21's **3 TCC** units (with up to 4 outputs each,
  16/24-bit periods) plus the **TC** timers. For a handful of PWM outputs the
  Arduino API is plenty; for precise frequencies or synchronized channels you
  configure TCC at register level via the GCLK clock tree (advanced — see the
  datasheet). Pins sharing a timer share its frequency.

## External interrupts (EXTINT) — a fixed set of pins the core enables

The SAMD21's EIC has 16 EXTINT channels plus an NMI, and the silicon shares those
channels across pins. The **stock MKR Zero variant resolves every collision for
you**: it assigns each EXTINT line to exactly one Arduino pin and marks all the
others `EXTERNAL_INT_NONE`. The practical upshot is simpler than the raw silicon
suggests — `attachInterrupt()` works on **only these pins**, each on a distinct
line (so there are no collisions left for you to manage):

| Pin | Port | EXTINT line | | Pin | Port | EXTINT line |
|-----|------|-------------|-|-----|------|-------------|
| D0  | PA22 | EXTINT6 | | D7  | PA21 | EXTINT5 |
| D1  | PA23 | EXTINT7 | | D8  | PA16 | EXTINT0 |
| D4  | PB10 | EXTINT10 | | D9  | PA17 | EXTINT1 |
| D5  | PB11 | EXTINT11 | | A1  | PB02 | EXTINT2 |
| D6  | PA20 | EXTINT4 | | A2  | PB03 | EXTINT3 |

- **Every other exposed pin is `EXTERNAL_INT_NONE` in the variant** — D2, D3, D10,
  D12, D13, D14, A0, and A3–A6. On those, `digitalPinToInterrupt()` returns
  `NOT_AN_INTERRUPT` and `attachInterrupt()` **silently does nothing**. The
  silicon has EXTINT lines on several of them, but the core disabled them to break
  ties (e.g. PA10/D2 and PB10/D4 both sit on EXTINT10 — the core gave the line to
  D4). Don't promise an ISR on a pin not in the table above.
- **D11/SDA (PA08) maps to the special NMI line**, not an ordinary EXTINT — treat
  it as a special case, and note it's also the I2C SDA pin.
- **D8 and D9 double as SPI** (MOSI/SCK). They're interrupt-capable only when
  you're not using the default `SPI` on them.
- Use `attachInterrupt(digitalPinToInterrupt(pin), isr, mode)`. Keep ISRs tiny:
  set a `volatile` flag and do the work in `loop()` — no `Serial`, no `delay`, no
  allocation, and no `float` math inside an ISR.
- Going beyond these pins means reconfiguring the EIC by hand (advanced) — at
  which point the silicon's shared-line constraints (datasheet Table 7-1) become
  yours to manage.

## Clocks & low-power knobs (details in `power.md`)

- Core runs at 48 MHz from the DFLL. The 32.768 kHz crystal feeds the RTC and
  enables accurate timekeeping and low-power wake (RTCZero / ArduinoLowPower).
- Standby/sleep current drops to the low-µA range with peripherals off; the
  onboard LED (USB-powered) won't be lit on battery regardless.

## Serial / "I see no output" checklist

- `Serial` = **native USB CDC**. After `Serial.begin()`, the port takes a moment
  to enumerate — a small `delay(100)` or a *bounded* `while(!Serial && millis()-
  t0<2000)` avoids losing the first prints.
- **Never** use an unbounded `while(!Serial)` — it hangs forever on battery (no
  host). See SKILL.md and `mission-critical.md`.
- `Serial1` = hardware UART on **D14 (TX) / D13 (RX)** for talking to other
  devices — independent of the USB `Serial`.
- No port in the IDE / can't upload? **Double-tap RESET** to enter the
  bootloader, and check the USB cable isn't power-only. See `project-setup.md`.
