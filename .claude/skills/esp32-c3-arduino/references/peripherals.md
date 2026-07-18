# Peripherals (GPIO, I2C, SPI, ADC, PWM, UART, interrupts)

All examples are **Arduino-ESP32 core 3.x** correct. Pin numbers are examples —
adjust to the user's board, avoiding the forbidden pins in `hardware.md`.

## Digital GPIO

Standard Arduino API works as expected:

```cpp
pinMode(5, OUTPUT);
digitalWrite(5, HIGH);

pinMode(9, INPUT_PULLUP);   // internal pull-up (handy for buttons)
int pressed = (digitalRead(9) == LOW);
```

Internal pull-ups/pull-downs are available on all GPIOs. Onboard LEDs are often
**active-LOW** (LOW = on) — check the board.

## PWM — use `analogWrite` or the LEDC API (3.x)

See SKILL.md for the old-vs-new contrast. In 3.x you address the **pin**, not a
channel; channels are assigned automatically.

```cpp
// Simplest: Arduino-style. 8-bit duty (0..255) by default.
analogWrite(7, 128);                 // ~50% duty on GPIO7
analogWriteFrequency(7, 5000);       // optional: set frequency (Hz)
analogWriteResolution(7, 10);        // optional: 10-bit duty (0..1023)

// Full LEDC control (e.g. servo at 50 Hz). The C3's LEDC hardware maxes out at
// 14-bit resolution, so use 14 — NOT 16 (16 silently fails / is rejected).
ledcAttach(3, 50, 14);               // pin, freq, resolution (≤14 bits)
ledcWrite(3, 1229);                  // 14-bit: full period=16384; ~1.5 ms ≈ 1229
// Smooth hardware fade across the full 14-bit range:
ledcFade(3, 0, 16383, 1000);         // pin, from, to, milliseconds
```

Two hardware limits to respect (both from the TRM):

- **6 channels max** on the C3 (the classic ESP32 has 16).
- **14-bit maximum resolution.** Also, resolution and frequency trade off —
  higher frequencies cap the usable bits (roughly `bits ≤ log2(80 MHz / freq)`),
  so e.g. you can't get 14 bits at 50 kHz. If `ledcAttach` returns a frequency
  that isn't what you asked for, you've hit this ceiling — lower the resolution.

## ADC — analog input

```cpp
analogReadResolution(12);                  // 0..4095 (12-bit is the hardware max)
int raw = analogRead(0);                    // GPIO0 = ADC1_CH0
float volts = raw * 3.3f / 4095.0f;         // rough; ADC isn't precise

analogSetPinAttenuation(0, ADC_11db);       // widen range toward ~3.3 V
```

Use **GPIO0–GPIO4** (these are SAR ADC1, channels 0–4). **Avoid GPIO5** — it is
the only ADC2 pin, and the TRM states the C3's ADC2 controller "does not work
properly" (it's in the SoC errata). This is a stronger reason than the old
"WiFi conflict" advice you'll see for the classic ESP32: on the C3, just don't
use ADC2 at all.

Two pin nuances that bite people: **GPIO0 and GPIO1** are also the optional
32 kHz crystal pins — if the board wires a watch crystal there, those ADC
channels are unavailable. **GPIO4** is also JTAG MTMS, fine for ADC unless
you're doing hardware JTAG debugging.

The ADC is 12-bit against an internal ~1.1 V reference scaled by the chosen
attenuation, and it's not laboratory-accurate (nonlinearity near the rails). If
the user needs real voltage accuracy, point them at the ESP-IDF eFuse
calibration / `esp_adc_cal` path rather than promising precision from a raw
`analogRead`.

## I2C (Wire)

Pass pins explicitly; the C3 can route I2C to almost any GPIO.

```cpp
#include <Wire.h>
#define SDA_PIN 8
#define SCL_PIN 9
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);     // optionally: Wire.begin(SDA, SCL, 400000)
}
```

A scanner is the fastest way to debug a silent device:

```cpp
for (uint8_t a = 1; a < 127; a++) {
  Wire.beginTransmission(a);
  if (Wire.endTransmission() == 0) { Serial.printf("Found 0x%02X\n", a); }
}
```

If GPIO8/9 are awkward on the user's board (strapping pins), any other safe
GPIO pair works — that's the advantage of being explicit.

## SPI

```cpp
#include <SPI.h>
#define SCK_PIN  4
#define MISO_PIN 5
#define MOSI_PIN 6
#define CS_PIN   7
void setup() {
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);   // explicit pins
}
void writeReg(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg); SPI.transfer(val);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
}
```

GPIO4/5 double as ADC1 pins — fine for SPI if you're not also reading analog on
them.

## UART (extra serial port)

`Serial` is the USB/console link (see `hardware.md`). For a *second* device-
facing port, use `Serial1` with explicit pins:

```cpp
#define RX1 20
#define TX1 21
void setup() {
  Serial1.begin(9600, SERIAL_8N1, RX1, TX1);
}
```

(If `Serial` is configured as UART0 on GPIO20/21, pick different pins for
`Serial1` to avoid the clash.)

## External interrupts

```cpp
volatile uint32_t pulses = 0;
void IRAM_ATTR onEdge() { pulses++; }     // keep ISRs tiny; IRAM_ATTR matters

void setup() {
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), onEdge, FALLING);
}
```

Any GPIO can be an interrupt source. Don't call `Serial`, `delay`, or heap
allocation inside the ISR — set a flag and handle it in `loop()`.

## Timers (3.x API)

The timer API was simplified in 3.x — no prescaler argument; you pass the
desired frequency directly.

```cpp
hw_timer_t *timer = nullptr;
volatile bool tick = false;
void IRAM_ATTR onTimer() { tick = true; }

void setup() {
  timer = timerBegin(1000000);              // 1 MHz timer clock
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 1000000, true, 0);      // fire every 1,000,000 ticks = 1 s
}
```

## On-chip temperature sensor

The C3 has a built-in temperature sensor (it reads the *die* temperature, not
ambient — useful for thermal monitoring, not as a room thermometer):

```cpp
float dieTempC = temperatureRead();         // degrees Celsius
```

Expect it to read above ambient when the radio is active and the chip is warm.
