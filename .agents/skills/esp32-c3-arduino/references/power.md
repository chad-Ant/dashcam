# Power & sleep

This is where C3 code most often differs from classic-ESP32 tutorials, because
the C3 has **no ULP coprocessor** and **no ext0/ext1 wakeup**. Get the wake
source right and the rest is straightforward.

## Sleep modes at a glance

- **Modem sleep** — radio idles between activity; automatic, mA-range. Just
  don't peg the CPU.
- **Light sleep** — CPU paused, RAM and peripherals retained, fast wake
  (microseconds), sub-mA. Good when you need to resume mid-function.
- **Deep sleep** — almost everything off, ~5 µA, RAM lost except `RTC_DATA_ATTR`
  variables. Wake = a **reboot** that re-runs `setup()`. Best for battery
  sensors that act briefly then sleep for seconds/minutes.

## Deep sleep: timer wakeup (works exactly like classic ESP32)

```cpp
#define uS_PER_S 1000000ULL
RTC_DATA_ATTR int bootCount = 0;     // survives deep sleep

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.printf("Boot #%d, cause %d\n", ++bootCount, esp_sleep_get_wakeup_cause());

  // ... do the work (read sensor, send, etc.) ...

  esp_sleep_enable_timer_wakeup(30 * uS_PER_S);   // sleep 30 s
  Serial.println("Sleeping...");
  Serial.flush();
  esp_deep_sleep_start();             // never returns; reboots into setup()
}
void loop() {}                        // unused in this pattern
```

## Deep sleep: GPIO wakeup — the C3 way (NOT ext0/ext1)

On the classic ESP32 you'd call `esp_sleep_enable_ext0_wakeup()`. **That does
not exist on the C3.** Use `esp_deep_sleep_enable_gpio_wakeup()` with a bitmask,
and only RTC-capable pins **GPIO0–GPIO5** can wake from deep sleep.

```cpp
#include "esp_sleep.h"
#define WAKE_PIN 3                              // must be one of GPIO0..GPIO5

void setup() {
  Serial.begin(115200);
  delay(50);
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO)
    Serial.println("Woke from GPIO");

  // ... do the work ...

  // Wake when WAKE_PIN goes HIGH. Use an external pulldown so it idles LOW.
  esp_deep_sleep_enable_gpio_wakeup(1ULL << WAKE_PIN, ESP_GPIO_WAKEUP_GPIO_HIGH);
  esp_deep_sleep_start();
}
void loop() {}
```

Key points to convey when a user's GPIO wake "does nothing":

- Wake cause comes back as **`ESP_SLEEP_WAKEUP_GPIO`** (= 7), *not*
  `ESP_SLEEP_WAKEUP_EXT0`. Old example code that checks for EXT0 will look like
  it never woke.
- The pin must be **GPIO0–GPIO5** — these are the only pins in the C3's
  VDD3P3_RTC power domain, and the TRM is explicit: every pin can wake the chip
  from *light* sleep, but **only GPIO0–GPIO5 can wake it from deep sleep.**
  GPIO9 (the BOOT button) is *not* in that range and won't work as a deep-sleep
  wake pin even though it's a button.
- Give the pin a defined idle level with an external resistor (pulldown for
  wake-on-HIGH, pull-up for wake-on-LOW). Floating pins cause spurious or
  missed wakes.
- For wake-on-LOW use `ESP_GPIO_WAKEUP_GPIO_LOW`. You can enable multiple pins
  by OR-ing their bits into the mask.

## Which wake sources work in which mode

Per the TRM, the C3's wake sources split by mode — getting this wrong is a
common "why won't it wake" bug:

- **Deep sleep:** GPIO (GPIO0–5 only), RTC timer, and the 32 kHz-crystal
  watchdog. That's it.
- **Light sleep:** all of the above *plus* any GPIO, UART (wake on RX
  activity), WiFi, and Bluetooth. So "wake on incoming UART bytes" or staying
  WiFi-associated across sleeps is a **light-sleep** technique, not a
  deep-sleep one.

## Holding pin states through deep sleep

In deep sleep the digital core powers down, so an output you set (e.g. a line
enabling a sensor or holding a MOSFET/load-switch off) will not, by default,
keep its level. Latch it with the hold feature before sleeping:

```cpp
#include "driver/gpio.h"
#define LOAD_EN 6
digitalWrite(LOAD_EN, LOW);             // desired level during sleep
gpio_hold_en((gpio_num_t)LOAD_EN);      // freeze this pin's state
gpio_deep_sleep_hold_en();              // keep holds active through deep sleep
esp_deep_sleep_start();
// after wake (reboot), release if you want to drive the pin again:
// gpio_hold_dis((gpio_num_t)LOAD_EN);
```

RTC pins (GPIO0–5) and digital pins (GPIO6–21) both support hold; this matters
whenever something external must stay in a known state while the chip sleeps.

## Light sleep with GPIO wake

For light sleep, the per-pin API applies and resumes execution in place:

```cpp
#include "esp_sleep.h"
gpio_wakeup_enable((gpio_num_t)3, GPIO_INTR_HIGH_LEVEL);
esp_sleep_enable_gpio_wakeup();
esp_light_sleep_start();              // returns here when the pin goes high
```

## Cutting active current

- `setCpuFrequencyMhz(80);` — big active-current drop. **Stay ≥ 80 MHz while
  WiFi runs;** you can go lower (40/20/10) for non-radio work.
- Turn the radio off when idle: `WiFi.disconnect(true); WiFi.mode(WIFI_OFF);`
  (and `btStop();` for BLE) before sleeping.
- Drive unused outputs to a defined level; don't leave inputs floating.

## Battery-sensor pattern (the high-value one)

For "read a sensor every N minutes on a battery," deep sleep dominates the
energy budget, so minimize awake time:

```
setup():
  read sensor  →  (WiFi.begin / send, or ESP-NOW burst)  →  WiFi off
  esp_sleep_enable_timer_wakeup(N minutes)
  esp_deep_sleep_start()
loop(): empty
```

ESP-NOW (see `wifi.md`) wakes-sends-sleeps faster than associating with an AP,
which is why it's popular for C3 battery nodes.

## Native-USB gotcha after sleep

On boards using the built-in USB Serial/JTAG, the COM port disappears during
deep sleep and the IDE may not auto-reconnect. To reflash, hold **BOOT**, tap
**RESET**, release BOOT to force download mode. This is normal, not a fault.
