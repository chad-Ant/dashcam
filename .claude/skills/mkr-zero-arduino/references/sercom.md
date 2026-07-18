# The SERCOM system — adding extra UART / SPI / I2C ports

The SAMD21's standout feature (and its standout source of confusion) is
**SERCOM**: six identical serial modules, each configurable as a **UART, SPI
master/slave, or I2C master/slave**. The MKR Zero exposes one of each by default,
but you can press a spare module into service to get a 2nd I2C bus, a 3rd serial
port, etc. — something a fixed-peripheral chip can't do.

This is genuinely more involved than calling `begin()`: you instantiate the port
object against a specific SERCOM, re-mux the pins with `pinPeripheral()`, and
wire up the interrupt handler. Get any of the three wrong and the port silently
does nothing. Read this whole file before adding a port.

## What's already used, what's free

The Arduino core reaches `Wire`, the SD card, and `Serial1` through the pins'
*alternate* SERCOM mux, so the default assignment isn't the one you'd guess from
the silicon's primary column — confirmed from the stock `variant.cpp`:

| SERCOM   | MKR Zero default use                        |
|----------|---------------------------------------------|
| **SERCOM0** | **free**                                 |
| SERCOM1  | `SPI` — D8/D9/D10 (peripheral C)             |
| SERCOM2  | `Wire` (I2C) — D11/D12 (via the ALT mux)     |
| **SERCOM3** | **free**                                 |
| SERCOM4  | onboard microSD card (via the ALT mux)      |
| SERCOM5  | `Serial1` (UART) — D13/D14 (via the ALT mux) |

So you have **two spare modules: SERCOM0 and SERCOM3.** Each pin can reach a
SERCOM only on specific *pads*, via one of two peripheral muxes:

- **`PIO_SERCOM`** = the pin's "peripheral C" SERCOM (its primary SERCOM column).
- **`PIO_SERCOM_ALT`** = the pin's "peripheral D" SERCOM (its alternate column).

You must pick a pin/pad combination that physically routes to the SERCOM you
chose. The authoritative map is the MKR Zero `variant.cpp` (and the SAMD21
datasheet **Table 7-1**). The subset that reaches the two *free* modules
(SERCOM0, SERCOM3) on exposed pins:

```
Pin  Arduino   SERCOM (PIO_SERCOM)   SERCOM_ALT (PIO_SERCOM_ALT)
D0   PA22      SERCOM3.0             SERCOM5.0
D1   PA23      SERCOM3.1             SERCOM5.1
D2   PA10      SERCOM0.2             SERCOM2.2
D3   PA11      SERCOM0.3             SERCOM2.3
D6   PA20      SERCOM5.2             SERCOM3.2
D7   PA21      SERCOM5.3             SERCOM3.3
D8   PA16      SERCOM1.0             SERCOM3.0   (D8/9/10 are the default SPI)
D9   PA17      SERCOM1.1             SERCOM3.1
D10  PA19      SERCOM1.3             SERCOM3.3
D11  PA08      SERCOM0.0             SERCOM2.0   (D11/12 are the default Wire)
D12  PA09      SERCOM0.1             SERCOM2.1
A3   PA04      —                     SERCOM0.0
A4   PA05      —                     SERCOM0.1
A5   PA06      —                     SERCOM0.2
A6   PA07      —                     SERCOM0.3
```

So **SERCOM3** is reachable for a fresh bus on D0/D1 (pads 0/1, peripheral C) and
D6/D7 (pads 2/3, ALT); **SERCOM0** on A3–A6 (pads 0–3, ALT) and D2/D3 (pads 2/3,
peripheral C). (The "x.y" is SERCOMx pad y. Confirm any pin not listed against
`variant.cpp` / Table 7-1 before wiring.)

## The three things every new port needs

1. **Instantiate** the object against a SERCOM and the chosen pins.
2. **`pinPeripheral(pin, PIO_SERCOM or PIO_SERCOM_ALT)`** in `setup()` for each
   pin — this overrides the default digital-I/O mux and lets the SERCOM drive the
   pin. Requires `#include "wiring_private.h"`.
3. **Define `SERCOMx_Handler()`** that forwards to the object's `IrqHandler()`
   (UART and I2C need this; the SPI master path typically doesn't). Forgetting
   the handler is the classic "RX never arrives" bug.

## New UART (extra serial port) on SERCOM3

Put TX on D0 (PA22, SERCOM3.0 = pad 0) and RX on D1 (PA23, SERCOM3.1 = pad 1):

```cpp
#include <Arduino.h>
#include "wiring_private.h"            // pinPeripheral()

// Uart(sercom, RX pin, TX pin, RX pad, TX pad)
Uart Serial2(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);

void SERCOM3_Handler() { Serial2.IrqHandler(); }   // REQUIRED for RX

void setup() {
  Serial2.begin(9600);
  pinPeripheral(0, PIO_SERCOM);        // D0 -> SERCOM3 (peripheral C)
  pinPeripheral(1, PIO_SERCOM);        // D1 -> SERCOM3 (peripheral C)
}

void loop() {
  if (Serial2.available()) { /* read */ }
}
```

If you only transmit, you may omit the RX `pinPeripheral()` and keep that pin for
other use. TX-on-pad-0/2, RX-on-any-pad are the rules; check pad numbers against
the table.

## New I2C (second Wire) on the free SERCOM3

Put a second `TwoWire` on a genuinely free module — **SERCOM0 or SERCOM3** (not
SERCOM1/2/4/5, which are SPI/Wire/SD/Serial1). The clean choice on the MKR Zero is
**SERCOM3 on D0/D1**, because for I2C the SDA signal must sit on the SERCOM's
**pad 0** and SCL on **pad 1**, and D0 (PA22 = SERCOM3 pad 0, peripheral C) and
D1 (PA23 = SERCOM3 pad 1, peripheral C) give you exactly that on exposed pins
(SERCOM0's pad 0/1 are reachable too, on A3/A4 via the ALT mux):

```cpp
#include <Wire.h>
#include "wiring_private.h"

TwoWire myWire(&sercom3, 0, 1);        // (sercom, SDA pin=D0, SCL pin=D1)
void SERCOM3_Handler() { myWire.IrqHandler(); }

void setup() {
  myWire.begin();
  pinPeripheral(0, PIO_SERCOM);        // D0 -> SERCOM3 pad 0 (SDA), peripheral C
  pinPeripheral(1, PIO_SERCOM);        // D1 -> SERCOM3 pad 1 (SCL), peripheral C
}
```

(These are the same pins the single-extra-UART example above uses. If you need an
extra UART **and** an extra I2C at the same time, see the next section — the I2C
stays on SERCOM3 and the UART moves to the other free module, SERCOM0.)

**External pull-ups (2–10 kΩ) to 3.3 V are required on SDA and SCL** for any
fresh I2C bus — and to 3.3 V, never 5 V. SDA must land on the SERCOM's pad 0 and
SCL on pad 1.

## Need an extra UART *and* an extra I2C at once

The two free modules are **SERCOM3 and SERCOM0**, so put one bus on each. **Second
I2C on SERCOM3 (D0/D1)** and **second UART on SERCOM0 (D2/D3).** The I2C needs
pads 0/1 → D0/D1 (peripheral C). For the UART, TX must be on pad 0 or 2; D2 (PA10
= SERCOM0 pad 2) is TX-legal and D3 (PA11 = SERCOM0 pad 3) takes RX, both
peripheral C → `PIO_SERCOM`. The two `SERCOMx_Handler`s are distinct, so nothing
collides with each other or with the default SPI/Wire/SD/Serial1:

```cpp
#include <Wire.h>
#include "wiring_private.h"

// 2nd I2C on SERCOM3 — D0 = SDA (pad0), D1 = SCL (pad1)
TwoWire myWire(&sercom3, 0, 1);
void SERCOM3_Handler() { myWire.IrqHandler(); }

// 2nd UART on SERCOM0 — D2 = TX (pad2), D3 = RX (pad3)
Uart Serial2(&sercom0, 3, 2, SERCOM_RX_PAD_3, UART_TX_PAD_2);   // (sercom, RX, TX, RX pad, TX pad)
void SERCOM0_Handler() { Serial2.IrqHandler(); }

void setup() {
  myWire.begin();
  pinPeripheral(0, PIO_SERCOM);        // D0 -> SERCOM3 pad0 (SDA)
  pinPeripheral(1, PIO_SERCOM);        // D1 -> SERCOM3 pad1 (SCL)

  Serial2.begin(9600);
  pinPeripheral(2, PIO_SERCOM);        // D2 -> SERCOM0 pad2 (TX)
  pinPeripheral(3, PIO_SERCOM);        // D3 -> SERCOM0 pad3 (RX)
}
```

Still required: external 3.3 V pull-ups on D0/D1 for the I2C bus. With this split
there's no SERCOM or pin contention — `Wire`/`SPI`/`Serial1`/SD keep their
defaults, and your two new buses sit on the only free modules. (One overlap to
know: D2/D3 are also the **I2S** SCK/FS pins, so if you need I2S audio at the same
time, route this UART to SERCOM0's other pads instead — A3 = TX, A4 = RX, via
`PIO_SERCOM_ALT`.)

## New SPI on a spare SERCOM

```cpp
#include <SPI.h>
#include "wiring_private.h"

// SPIClass(sercom, MISO pin, SCK pin, MOSI pin, TX pad combo, RX pad)
SPIClass mySPI(&sercom3, 10, 9, 8, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);

void setup() {
  mySPI.begin();
  pinPeripheral(8,  PIO_SERCOM_ALT);   // MOSI -> SERCOM3
  pinPeripheral(9,  PIO_SERCOM_ALT);   // SCK  -> SERCOM3
  pinPeripheral(10, PIO_SERCOM_ALT);   // MISO -> SERCOM3
}
```

The `SPI_PAD_*_SCK_*` constant encodes which pad carries MOSI and which carries
SCK; MISO can be on any pad. Choose the combo that matches your pin/pad routing
(here MOSI on pad 0, SCK on pad 1, MISO on pad 3).

> **Caveat:** D8/D9/D10 are the default `SPI` bus's pins (SERCOM1). The snippet
> above re-muxes them to SERCOM3 to show the ALT-path mechanics, so it **replaces**
> the primary SPI rather than adding a truly independent one — you can't have both
> SERCOM1 and SERCOM3 driving the same pins. For a genuinely separate second SPI,
> pick free pins whose pads cover SCK/MOSI/MISO on a spare SERCOM (consult
> Table 7-1) — on the MKR Zero the available pad-0/1 pins for SERCOM3 are D0/D1,
> with SCK/MISO needing additional ALT-mux pins, so plan the pad map before wiring.

## Debugging a dead new port — check these in order

1. **Wrong SERCOM for the pins.** The pin/pad must physically route to the SERCOM
   you instantiated. Re-check Table 7-1 / `variant.cpp`.
2. **`PIO_SERCOM` vs `PIO_SERCOM_ALT`.** Using the wrong mux silently leaves the
   pin as plain GPIO. Match it to whether the SERCOM is the pin's peripheral C
   or D.
3. **Missing `SERCOMx_Handler()`.** UART/I2C RX won't work without it.
4. **Pad mismatch.** UART TX must be pad 0 or 2; I2C SDA = pad 0, SCL = pad 1;
   SPI per the `SPI_PAD_*` rule.
5. **Clashing with a default port.** Don't re-mux a pin that a default bus (or the
   SD card) is using unless you've freed it.

When unsure, point the user at the SAMD21 datasheet (PORT Function Multiplexing)
and the `ArduinoCore-samd` `variant.cpp` for the exact pads rather than guessing
a pin/pad combination.
