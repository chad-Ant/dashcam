# Peripherals (GPIO, ADC, DAC, I2C, SPI, UART, interrupts, timers, WDT)

All examples target the **Arduino MKR Zero** with the **Arduino SAMD core**. Pins
are the board's fixed defaults (see `hardware.md`); adding *extra* serial/SPI/I2C
buses is in `sercom.md`. Remember: **3.3 V logic, not 5 V tolerant.**

## Digital GPIO

Standard Arduino API works as expected — but use `LED_BUILTIN`, not pin 13:

```cpp
pinMode(LED_BUILTIN, OUTPUT);
digitalWrite(LED_BUILTIN, HIGH);   // onboard LED (pin 32). NOTE: USB/VIN-powered,
                                   // so it stays dark on battery even when running.

pinMode(6, INPUT_PULLUP);          // internal pull-up (handy for buttons)
int pressed = (digitalRead(6) == LOW);
```

Internal pull-ups/pull-downs are available. Respect the **7 mA/pin** and
**46 mA source / 65 mA sink per group** limits (`hardware.md`) — drive real loads
through a transistor.

## ADC — analog input (12-bit, but defaults to 10-bit)

```cpp
analogReadResolution(12);                 // unlock full 12-bit (0..4095)
int raw = analogRead(A1);                  // A0..A6 are the analog inputs
float volts = raw * 3.3f / 4095.0f;        // approximate; ADC isn't lab-accurate
```

- Without `analogReadResolution(12)` you get the **10-bit Uno-compat default**
  (0..1023) — a frequent "why are my readings only 0–1023?" surprise.
- **A0 is shared with the DAC**; don't read analog on A0 while driving the DAC.
- Changing `analogReference()` (e.g. to `AR_INTERNAL1V0` or external `AREF`)
  rescales every reading — set it once before sampling, and don't apply more than
  the rail to the AREF pin.
- Average several samples for a stable value; the SAR ADC has offset/gain error.

## DAC — true analog output on A0

The SAMD21 has a real 10-bit DAC (no RC filter needed):

```cpp
analogWriteResolution(10);     // DAC resolution (0..1023)
analogWrite(A0, 512);          // ~1.65 V on A0/DAC0
```

Swings ~0 → ~3.3 V, low current — buffer with an op-amp to drive a load. Ideal
for waveform/audio/control-voltage output. (Only one DAC channel exists.)

## PWM — `analogWrite` on the ~ pins

```cpp
analogWrite(3, 128);                   // ~50% duty (8-bit default) on D3
analogWriteResolution(10);             // optional: 10-bit duty for PWM pins
```

PWM pins: **D0–D8, D10, A3, A4**. Pins sharing a TCC/TC timer share its
frequency. For precise/custom PWM frequency or synchronized channels, configure
TCC at register level via GCLK (advanced — datasheet). Note
`analogWriteResolution()` affects both the DAC and PWM duty range; set it
deliberately.

## I2C (Wire) — D11 = SDA, D12 = SCL (SERCOM2)

```cpp
#include <Wire.h>
void setup() {
  Wire.begin();                        // MKR Zero I2C is fixed on D11/D12
  // Wire.setClock(400000);            // optional: 400 kHz fast mode
}
```

The onboard **ATECC508A** crypto chip and the ESLOV connector share this bus. A
scanner is the fastest way to debug a silent device:

```cpp
for (uint8_t a = 1; a < 127; a++) {
  Wire.beginTransmission(a);
  if (Wire.endTransmission() == 0) { Serial.print("Found 0x"); Serial.println(a, HEX); }
}
```

Always **check `Wire.endTransmission()`** (0 = success) rather than assuming the
write landed. External I2C devices need pull-ups to **3.3 V** (not 5 V); many
breakout boards include them. For a *second* I2C bus, see `sercom.md`.

## SPI — D8 = COPI/MOSI, D9 = SCK, D10 = CIPO/MISO (SERCOM1)

```cpp
#include <SPI.h>
#define CS_PIN 7
void setup() {
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin();                         // header SPI is fixed on D8/D9/D10
}
void writeReg(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg); SPI.transfer(val);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
}
```

This header SPI is **independent of the onboard microSD card's SPI** (which is on
SERCOM4) — both run simultaneously. Pick any free GPIO for chip-select; don't
reuse D8/D9/D10. For a *second* user SPI bus, see `sercom.md`.

## UART — `Serial` (USB) vs `Serial1` (pins)

- `Serial` is the **USB CDC** console (see `hardware.md` / `mission-critical.md`
  for the bounded-wait rule).
- `Serial1` is a hardware UART on **D14 (TX) / D13 (RX)** for device-to-device
  comms:

```cpp
void setup() {
  Serial1.begin(9600);                 // talks to an external device on D13/D14
}
```

Need a *third* serial port? Instantiate one on the free SERCOM0 or SERCOM3 — see
`sercom.md`.

## External interrupts — only the variant-enabled pins

`attachInterrupt()` works on a fixed set of pins that the stock variant enables:
**D0, D1, D4, D5, D6, D7, D8, D9, A1, A2** (plus D11/SDA as the special NMI). On
every other exposed pin the variant sets `EXTERNAL_INT_NONE`, so
`digitalPinToInterrupt()` returns `NOT_AN_INTERRUPT` and the attach silently
no-ops — a common "my ISR never fires" cause. The core already resolved the
SAMD21's shared EXTINT lines, so among the enabled pins there are no collisions to
manage. Full table (with lines, and which pins are excluded) in `hardware.md`.

```cpp
volatile uint32_t pulses = 0;
void onEdge() { pulses++; }            // tiny ISR: flag/counter only

void setup() {
  pinMode(7, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(7), onEdge, FALLING);  // D7 = EXTINT5 (enabled)
}
```

Inside an ISR: no `Serial`, no `delay`, no heap allocation, no `float` math — set
a `volatile` flag/counter and handle it in `loop()`.

## Timers (TC/TCC)

For periodic work, the simplest portable option is a non-blocking `millis()`
scheduler:

```cpp
uint32_t last = 0;
void loop() {
  if (millis() - last >= 1000) {       // every 1 s, without blocking
    last += 1000;
    // periodic task
  }
}
```

For hardware-timed interrupts at a precise rate, drive a **TC** timer via its
GCLK and `TCx_Handler()` ISR, or use a helper library (e.g. one of the SAMD
"TimerInterrupt" libraries) rather than hand-rolling register code unless the
user wants register-level control. The SAMD21 timer setup is more involved than
AVR's `Timer1` — reach for the datasheet's TC/TCC and GCLK sections.

## Watchdog (WDT) — for unattended reliability

An MKR Zero left running unattended should arm the watchdog so a hang reboots it
instead of staying dead:

```cpp
#include <Adafruit_SleepyDog.h>
void setup() {
  int wdMs = Watchdog.enable(8000);    // reset if not "fed" within ~8 s
}
void loop() {
  // ... real work, all paths must complete within the window ...
  Watchdog.reset();                    // feed it once per healthy pass
}
```

Don't feed the watchdog from inside an unbounded wait — that defeats it. The WDT
is the safety net behind Rule 2 (bounded loops) in `mission-critical.md`.
