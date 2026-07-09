# ESP32-C3 hardware reference

The constraints here are fixed by the silicon — they hold for every C3 board.
What changes between boards is only which pins are broken out and labeled.

## Core specs

- **CPU:** single-core 32-bit RISC-V, up to 160 MHz (lower it to save power).
- **RAM:** ~400 KB SRAM. No PSRAM on the bare chip (a few modules add some).
- **Radio:** 2.4 GHz WiFi 4 (802.11 b/g/n) + Bluetooth 5 (LE). **No 5 GHz, no
  Bluetooth Classic.**
- **GPIO:** 22 pins, GPIO0–GPIO21.

## What the C3 does NOT have (the classic-ESP32 trap)

These exist on the original ESP32 and appear constantly in tutorials, but are
absent on the C3. Using them is the #1 source of "works in the example, not on
my board":

- **No DAC.** `dacWrite()` / `DAC1` / `DAC2` do not exist. For analog-ish output
  use PWM (`analogWrite` / LEDC) and an RC filter if you need smooth voltage.
- **No Bluetooth Classic.** `BluetoothSerial` (SPP) is unavailable. Bluetooth on
  the C3 is **BLE only** — see `ble.md`.
- **No touch sensor.** `touchRead()` and touch wakeup do not work.
- **No second core.** No `xTaskCreatePinnedToCore(..., 1)`; everything runs on
  core 0.
- **No ULP coprocessor** and **no ext0/ext1 deep-sleep wakeup** — wake sources
  differ; see `power.md`.

## Pins: safe, risky, and forbidden

Think in three tiers.

**Forbidden — don't use for I/O:**

- **GPIO12, 13, 14, 15, 16, 17** — connected to the SPI flash. Using them
  crashes or bricks the boot. They are usually not even broken out.
- **GPIO11** — `VDD_SPI` flash power. Leave alone.

**Use with care (strapping pins)** — fine as outputs/inputs once running, but
their level is sampled at boot, so don't hold them at the wrong level during
reset, and prefer not to use them for things that fight the boot state:

- **GPIO2** — must be high or floating at boot.
- **GPIO8** — must be high at boot. Onboard LED on several boards (often
  active-LOW, e.g. the C3 Super Mini).
- **GPIO9** — the BOOT button on most boards; has an internal pull-up. Hold it
  low during reset to enter download mode.

**Native USB:**

- **GPIO18 (D-), GPIO19 (D+)** — the built-in USB Serial/JTAG. If you rely on
  native USB for programming/Serial, don't repurpose these.

**Generally safe for general I/O:** GPIO0–GPIO10 and GPIO20/21 (mind the
strapping notes on 2/8/9, and that 20/21 are UART0 by default — see below).
GPIO18/19 are also usable if you are not using native USB.

> Board-specific note: always trust the user's board silkscreen/pinout over a
> generic mapping. If you don't know the board, state which GPIO you chose and
> that it's a known-safe pin.

## ADC (analog input)

- The C3 has two SAR ADCs, but only **ADC1** is usable. ADC1 covers **GPIO0–
  GPIO4** (channels 0–4). **GPIO5** is the lone ADC2 pin.
- **Do not use GPIO5 / ADC2.** The TRM states plainly that "the DIG ADC
  controller of SAR ADC2 for ESP32-C3 does not work properly" and to use ADC1
  instead (it's a documented SoC erratum). So treat the C3 as having **five
  analog inputs: GPIO0–GPIO4.** (This supersedes the classic-ESP32 advice that
  ADC2 merely conflicts with WiFi — on the C3 it's broken regardless.)
- **GPIO0 and GPIO1 double as the 32 kHz crystal pins** (XTAL_32K_P/N); if a
  board fits a watch crystal there, those two ADC channels are gone.
  **GPIO4 doubles as JTAG MTMS.**
- Resolution is 12-bit (0–4095) against an internal ~1.1 V reference scaled by
  per-pin attenuation (`analogSetPinAttenuation(pin, ADC_11db)` for ~0–3.3 V).
  The ADC is not precise near the rails; for accurate volts use eFuse-based
  calibration rather than a raw `analogRead`.

## RTC-domain pins (matters for deep sleep)

GPIO0–GPIO5 sit in the VDD3P3_RTC power domain ("RTC pins"); GPIO6–GPIO21 are
plain digital pins. This grouping is why **only GPIO0–GPIO5 can wake the chip
from deep sleep**, and why pin-state *hold* through deep sleep is configured
differently for the two groups (see `power.md`).

## Clocks / power knobs (quick reference; details in `power.md`)

- `setCpuFrequencyMhz(80)` (or 40/20/10) lowers active current. **WiFi needs
  ≥80 MHz** — don't drop below 80 while the radio is on.
- Deep sleep current is in the few-µA range when pins are configured well.

## LEDC / PWM capacity

- **6 LEDC channels**, **4 timers**, **14-bit maximum resolution** (per the
  TRM). For a handful of PWM outputs that's plenty; if a user wants more than 6
  independent PWM signals at once, that's the hard limit and they'll need to
  multiplex or rethink. Don't write `ledcAttach(..., 16)` — 16-bit exceeds the
  hardware and won't behave. (API details and the freq/resolution trade-off in
  `peripherals.md`.)

## Serial / "I see no output" checklist

The C3 has two ways `Serial` can reach your computer, controlled by the Arduino
IDE **Tools → USB CDC On Boot** setting:

- **Enabled:** `Serial` is the native USB CDC (over GPIO18/19). Best for boards
  with native USB (e.g. XIAO ESP32C3, DevKitM-1).
- **Disabled:** `Serial` is hardware UART0 on **GPIO21 (TX) / GPIO20 (RX)** —
  correct for boards with a separate USB-UART bridge chip (e.g. many CH340
  "Super Mini" clones).

If a user gets no serial output, the wrong CDC setting for their board is the
usual cause. `HWCDC`/native USB can also drop off after deep sleep — see
`power.md`.
