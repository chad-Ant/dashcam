# Project setup, board settings & flashing

## Installing the core (Arduino IDE 2.x)

1. **File → Preferences → Additional boards manager URLs**, add:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. **Tools → Board → Boards Manager**, search **esp32** (by Espressif Systems),
   install. The current series is **3.x** (3.3.x as of 2026, ESP-IDF 5.5).
   Installing 3.x matters — the APIs in this skill assume it.
3. **Tools → Board → ESP32 Arduino** → pick the board. If the exact board isn't
   listed, **"ESP32C3 Dev Module"** is the safe generic choice.

`arduino-cli` equivalent:

```bash
arduino-cli config add board_manager.additional_urls \
  https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli compile -b esp32:esp32:esp32c3 your_sketch
arduino-cli upload  -b esp32:esp32:esp32c3 -p /dev/ttyACM0 your_sketch
```

## Board settings that actually matter

Most defaults are fine. The ones that cause real confusion:

- **USB CDC On Boot** — controls where `Serial` goes. *Enabled* = native USB
  (boards with built-in USB like XIAO C3, DevKitM-1). *Disabled* = UART0 on
  GPIO21/20 (boards with a CH340/CP210x bridge). Wrong choice = no serial
  output. (See `hardware.md`.)
- **Flash Size / Partition Scheme** — match the module (commonly 4 MB). Choose a
  partition scheme with OTA space if doing OTA, or "Huge APP" if a single sketch
  is too big to fit the default app partition.
- **Upload Speed** — drop to 115200 if uploads fail intermittently.

## Flashing / can't-upload checklist

Native-USB boards normally just upload. If it fails or the port is missing,
force download mode: **hold BOOT, tap RESET, release BOOT**, then upload. This
is also needed after a sketch that deep-sleeps or reconfigures the USB pins.
Bridge-chip boards: make sure the right serial driver (CH340/CP210x) is
installed and the correct port is selected.

## PlatformIO

`platformio.ini` for a generic C3 dev board:

```ini
[env:esp32-c3-devkitm-1]
platform = espressif32
board = esp32-c3-devkitm-1
framework = arduino
monitor_speed = 115200

; If your board uses native USB for Serial, mirror the IDE's "USB CDC On Boot":
build_flags =
    -DARDUINO_USB_MODE=1
    -DARDUINO_USB_CDC_ON_BOOT=1

; Common alternative boards:
; board = seeed_xiao_esp32c3
; board = lolin_c3_mini
```

Pin the platform version for reproducible builds, e.g.
`platform = espressif32@^6.5.0` (the Arduino 3.x core). Bump deliberately rather
than letting it float.

## Sketch skeleton

A clean starting point that's already C3-aware:

```cpp
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(100);                       // give native USB CDC a moment to attach
  Serial.println("ESP32-C3 up");
}

void loop() {
  // ...
}
```

The `delay(100)` after `Serial.begin` avoids losing the first prints on
native-USB boards, where the CDC link takes a moment to enumerate.

## Quick board cheat-sheet

| Board                  | USB        | Onboard LED        | Notes                          |
|------------------------|------------|--------------------|--------------------------------|
| XIAO ESP32C3           | Native     | none (user adds)   | external antenna connector     |
| ESP32-C3-DevKitM-1     | Native     | GPIO8 (RGB WS2812) | Espressif reference board      |
| Lolin C3 Mini          | Native     | GPIO7              | small, castellated             |
| C3 Super Mini (clones) | CH340 UART | GPIO8 (active-LOW) | set USB CDC On Boot = Disabled |

When unsure, ask the user which board, or proceed with the generic "ESP32C3 Dev
Module" assumptions and state them.
