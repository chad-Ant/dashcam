# RTC, low-power sleep & LiPo battery operation

The MKR Zero is built for battery work: a LiPo charger, a 32.768 kHz crystal for
an accurate RTC, and SAMD21 sleep modes that drop to the low-µA range. The two
libraries that matter are **RTCZero** (timekeeping + alarms) and
**ArduinoLowPower** (sleep/standby + wake sources). Install both via the Library
Manager.

## RTC — timekeeping and alarms (RTCZero)

```cpp
#include <RTCZero.h>
RTCZero rtc;

void setup() {
  rtc.begin();                                  // uses the 32.768 kHz crystal
  rtc.setTime(8, 30, 0);                        // HH, MM, SS
  rtc.setDate(28, 6, 26);                       // DD, MM, YY
  rtc.setAlarmTime(8, 30, 10);                  // fire 10 s later
  rtc.enableAlarm(rtc.MATCH_HHMMSS);
  rtc.attachInterrupt(onAlarm);                 // callback on match
}

void onAlarm() { /* tiny: set a flag, handle in loop() */ }
```

The RTC keeps running through standby, which is what makes "sleep until the next
sample time" possible (below). It's accurate enough for wall-clock logging thanks
to the crystal; for absolute accuracy over months, sync it periodically from an
external time source.

## Low-power sleep (ArduinoLowPower)

```cpp
#include <ArduinoLowPower.h>

void loop() {
  doSample();
  LowPower.sleep(60000);        // light sleep ~60 s (peripherals/RAM retained, fast wake)
  // or:
  LowPower.deepSleep(60000);    // deeper sleep, lower current, slower wake
}
```

Wake on a pin or an RTC alarm instead of a fixed duration:

```cpp
// Pin wake — only on an interrupt-capable pin (see EXTINT map in hardware.md)
LowPower.attachInterruptWakeup(7, onWake, FALLING);   // D7 = INT5
LowPower.deepSleep();                                  // sleep until that edge

// RTC-alarm wake: set an RTCZero alarm (above), then LowPower.deepSleep();
// the alarm interrupt wakes the board at the scheduled time.
```

Patterns that actually save power:

- **Sleep the gaps, don't `delay()` them.** A `delay(60000)` keeps the core fully
  awake burning current; `LowPower.sleep/deepSleep` is the battery win.
- **Quiet the peripherals before sleeping.** Close/flush the SD card, stop
  high-current sensors, and avoid leaving the DAC or PWM driving. SD and radio-
  style spikes dominate the budget far more than the sleeping MCU.
- **USB drops on deep sleep.** After `deepSleep`, the native USB `Serial` link
  disconnects and may need re-enumeration; don't assume the Serial Monitor
  survives a deep-sleep cycle. (If you're debugging power code, expect the port
  to come and go — see `project-setup.md` for the double-tap reset recovery.)
- **Wake sources must be interrupt-capable pins.** Same EXTINT constraints as
  normal interrupts — a pin whose EXTINT line is taken can't be a wake source
  (`hardware.md`).
- **A plain watchdog and a long `deepSleep` fight each other.** An ordinary
  `Watchdog.enable(8000)` keeps counting while the board sleeps, so a 5-minute
  `deepSleep(300000)` trips the watchdog and resets the board long before it
  wakes — turning your sleep interval into an 8-second reboot loop. Either don't
  arm a watchdog across the sleep, or use a **sleep-aware** call that combines
  the two — `Watchdog.sleep(ms)` (Adafruit SleepyDog) sleeps *and* services the
  watchdog as one operation. Match the watchdog window to the longest awake path,
  not the sleep duration.

## LiPo battery & charging

- JST connector is **single-cell LiPo only** (3.7 V nominal). The onboard charger
  runs at ~**350 mA**, so use a battery of at least **~700 mAh** — smaller cells
  can overheat under that charge current.
- On battery, the `5V` pin supplies ~3.7 V (battery passthrough); the 3.3 V
  regulator holds VCC steady. `VIN` is for a regulated 5 V source (max 6 V) and
  disconnects USB power when used.
- **The "ON" LED is wired to USB/VIN, not the battery** — it stays **off on
  battery** even when the board is running perfectly. This is by design (to save
  battery) and is a common "is it even on?" confusion. Don't use the power LED,
  or `LED_BUILTIN`'s availability, as a battery-state indicator.

## Battery voltage monitoring

The board ties LiPo voltage to an ADC input through a divider, and the stock
variant exposes it as the macro **`ADC_BATTERY`** (Arduino pin 33 → PB09,
ADC channel 3), so you can read the battery level in software and, e.g., stop
logging before a brownout:

```cpp
analogReadResolution(12);
int raw = analogRead(ADC_BATTERY);       // variant macro for the battery sense input
float vbat = raw * (3.3f / 4095.0f) * DIVIDER_RATIO;
if (vbat < 3.4f) { /* low battery: flush SD, sleep, stop drawing current */ }
```

`ADC_BATTERY` gives you the right pin, but the on-board divider still scales the
reading — confirm `DIVIDER_RATIO` against the MKR Zero schematic before trusting
the voltage; don't ship a guessed ratio.

## Brownout

The SAMD21's brown-out detector (BOD33) resets the chip if the supply sags too
low — most likely on a tired battery during an SD-write or sensor current spike.
For unattended logging, monitor battery voltage (above), keep writes short, and
arm the watchdog (`peripherals.md`) so a brownout-induced glitch recovers into a
clean reboot rather than a hang.
