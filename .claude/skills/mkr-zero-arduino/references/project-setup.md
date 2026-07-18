# Project setup, board settings & flashing

## Installing the core (Arduino IDE 2.x)

1. **Tools → Board → Boards Manager**, search **"Arduino SAMD Boards
   (32-bits ARM Cortex-M0+)"** (by Arduino), and install it. This one package
   covers the whole MKR family plus the Zero.
2. **Tools → Board → Arduino SAMD (32-bits…) → Arduino MKR Zero.**
3. Select the serial port under **Tools → Port** (a native-USB port appears when
   the board is running a sketch or in the bootloader).

Unlike the ESP32 core, the SAMD core's API is stable across versions — the MKR
Zero gotchas are hardware/board facts (3.3 V, SERCOM, no EEPROM), not core-
version churn, so you don't need to pin a specific core release.

`arduino-cli` equivalent:

```bash
arduino-cli core update-index
arduino-cli core install arduino:samd
arduino-cli compile -b arduino:samd:mkrzero your_sketch
arduino-cli upload  -b arduino:samd:mkrzero -p /dev/ttyACM0 your_sketch
```

The fully-qualified board name is **`arduino:samd:mkrzero`**.

## The bootloader & the double-tap reset (essential recovery)

The MKR Zero runs a SAM-BA bootloader. The one trick to know:

- **Can't see the port / upload fails / a sketch broke USB or deep-sleeps:**
  **press RESET twice quickly** (a pencil tip helps). The board enters the
  bootloader, the onboard LED **pulses/fades**, and a new serial port appears —
  upload to that. This recovers almost any "bricked" or non-enumerating board.
- A sketch that sleeps immediately, reconfigures USB, or hangs in `setup()` can
  make the normal auto-reset upload fail; the double-tap forces the bootloader
  regardless of what the sketch does.
- **No port at all and double-tap doesn't help?** Try a different USB cable —
  many micro-USB cables are **charge-only** with no data lines. Also remove the
  black conductive foam if the board still has it on the pins.

## Flashing / can't-upload checklist

1. Right board selected (**Arduino MKR Zero**) and right port.
2. **Double-tap RESET** to force the bootloader, then upload.
3. Swap the USB cable (rule out a power-only cable).
4. Make sure no Serial Monitor on another program is holding the port.

## PlatformIO

`platformio.ini` for the MKR Zero:

```ini
[env:mkrzero]
platform = atmelsam
board = mkrzero
framework = arduino
monitor_speed = 115200

; Common MKR Zero libraries (uncomment as needed):
; lib_deps =
;   arduino-libraries/RTCZero
;   arduino-libraries/ArduinoLowPower
;   cmaglie/FlashStorage
;   adafruit/Adafruit SleepyDog Library
```

The board id is **`mkrzero`** on the `atmelsam` platform.

## Sketch skeleton (already MKR-Zero-aware)

A clean starting point with the bounded Serial wait baked in:

```cpp
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) { }   // wait ≤2 s for USB; never hang on battery

  analogReadResolution(12);                      // unlock the 12-bit ADC (optional but explicit)

  pinMode(LED_BUILTIN, OUTPUT);                  // pin 32, NOT 13 (and USB-powered)
  Serial.println("MKR Zero up");
}

void loop() {
  // non-blocking work; sleep the gaps if on battery (see power.md)
}
```

The bounded `while(!Serial…)` is deliberate: an unbounded `while(!Serial)` hangs
forever on battery (no host) — the single most common MKR Zero field bug.

## Quick board orientation

| Item              | MKR Zero value                                            |
|-------------------|-----------------------------------------------------------|
| MCU               | SAMD21G18A, Cortex-M0+ @ 48 MHz, 256 KB flash, 32 KB SRAM |
| Logic level       | **3.3 V (NOT 5 V tolerant)**                              |
| USB               | Native (CDC `Serial`)                                      |
| Onboard LED       | `LED_BUILTIN` = pin 32 (USB/VIN-powered, off on battery)  |
| Default I2C/SPI/UART | `Wire` D11/D12 · `SPI` D8/D9/D10 · `Serial1` D13/D14    |
| Special           | onboard microSD (own SPI), LiPo charger, 32.768 kHz RTC, ATECC508A |
| Radio             | none                                                       |
| FQBN              | `arduino:samd:mkrzero`                                     |

This skill targets the **MKR Zero specifically**. Other MKR boards share the form
factor and much of the pin map, but differ in radio, exact peripherals, and
sometimes pin functions — don't assume MKR WiFi 1010 / MKR 1000 / MKR WAN details
carry over unchanged.
