/**
 * @file libgpio_dashcam.h
 * @brief Jetson Orin Nano 40-pin header GPIO line-name constants.
 *
 * These constants map the printed header pin numbers to the gpiod line names
 * visible in `gpioinfo` and usable with GpioPin::openByName() / GpioWatcher::openByName().
 *
 * The Orin Nano exposes GPIO on gpiochip0 (164 lines, named "PA.00" … "PEE.02").
 * Line names are stable across reboots; numeric offsets are not.
 *
 * Verify any new pin with:
 *   gpioinfo | grep -i "<name>"
 *
 * 40-pin header power / GND rails (not in this header):
 *   Pins 1, 17 = 3.3 V     Pins 2, 4 = 5 V     Pins 6,9,14,20,25,30,34,39 = GND
 *
 * I2C, SPI, UART and PWM signals on the same header are listed as comments
 * for reference but are managed by libi2c / libspi / libuart, not by libgpio.
 */

#ifndef LIBGPIO_DASHCAM_H
#define LIBGPIO_DASHCAM_H

namespace dashcam::gpio::pins {

// ─── GPIO-capable pins (gpiochip0 line names) ─────────────────────────────────
// Header  Signal name   gpiod name   Notes
// ─────── ────────────  ───────────  ─────────────────────────────────────────

constexpr const char* PIN7_GPIO09  = "PBB.00";  ///< Header pin  7  (GPIO09)
constexpr const char* PIN11_GPIO17 = "PCC.04";  ///< Header pin 11  (GPIO17 / UART1_RTS)
constexpr const char* PIN12_GPIO18 = "PBB.03";  ///< Header pin 12  (GPIO18 / I2S_CLK)
constexpr const char* PIN13_GPIO27 = "PBB.01";  ///< Header pin 13  (GPIO27)
constexpr const char* PIN15_GPIO22 = "PBB.02";  ///< Header pin 15  (GPIO22)
constexpr const char* PIN16_GPIO23 = "PAA.07";  ///< Header pin 16  (GPIO23)
constexpr const char* PIN18_GPIO24 = "PAA.04";  ///< Header pin 18  (GPIO24)
constexpr const char* PIN22_GPIO25 = "PAA.05";  ///< Header pin 22  (GPIO25)
constexpr const char* PIN29_GPIO05 = "PAA.00";  ///< Header pin 29  (GPIO05)
constexpr const char* PIN31_GPIO06 = "PAA.03";  ///< Header pin 31  (GPIO06)
constexpr const char* PIN32_GPIO12 = "PAA.01";  ///< Header pin 32  (GPIO12 / PWM0)
constexpr const char* PIN33_GPIO13 = "PAA.06";  ///< Header pin 33  (GPIO13 / PWM2)
constexpr const char* PIN35_GPIO19 = "PQ.06";   ///< Header pin 35  (GPIO19 / I2S_FS)
constexpr const char* PIN36_GPIO16 = "PCC.01";  ///< Header pin 36  (GPIO16 / UART1_CTS)
constexpr const char* PIN37_GPIO26 = "PAA.02";  ///< Header pin 37  (GPIO26)
constexpr const char* PIN38_GPIO20 = "PQ.07";   ///< Header pin 38  (GPIO20 / I2S_DIN)
constexpr const char* PIN40_GPIO21 = "PQ.05";   ///< Header pin 40  (GPIO21 / I2S_DOUT)

// ─── gpiochip paths ───────────────────────────────────────────────────────────

constexpr const char* GPIOCHIP0 = "/dev/gpiochip0";  ///< Main GPIO bank — 164 lines
constexpr const char* GPIOCHIP1 = "/dev/gpiochip1";  ///< AON GPIO bank — used for suspend-safe pins

// ─── status LED panel: 5 RGB LEDs behind two 74HCT595 shift registers ─────────
//
// All six lines are in the PAA bank and physically clustered on the header, so a
// miswire tends to be visibly wrong rather than subtly wrong. Grounds at pins 30
// and 34 sit beside the cluster for the shift-register return.
//
// Pins 11/36 (UART1 RTS/CTS) and 12/35/38/40 (I2S) are deliberately avoided —
// libuart and libmidi exist in this repo and may want them.

constexpr const char* LED595_SER      = PIN29_GPIO05;  ///< Header 29 — serial data into 595 #1.
constexpr const char* LED595_SRCLK    = PIN31_GPIO06;  ///< Header 31 — shift clock (data on rising edge).
constexpr const char* LED595_RCLK     = PIN33_GPIO13;  ///< Header 33 — storage/latch clock.
constexpr const char* LED595_SRCLR    = PIN37_GPIO26;  ///< Header 37 — async clear, ACTIVE LOW.
/**
 * Header 16 — Q7S of 595 #2 wired back as an input, THROUGH A DIVIDER.
 *
 * Optional in the sense that the panel lights without it, and load-bearing in
 * the sense that nothing else can tell you the chain is intact. Shifting a
 * known pattern through and reading it out the far end verifies every wire, the
 * daisy-chain link and the logic levels in one test — the same read-back
 * discipline the MKR firmware applies to every configuration register, for the
 * same reason: a write that was acknowledged and did not land is the failure
 * that costs days.
 *
 * *** 10 k FROM Q7S TO THIS PIN, 20 k FROM THIS PIN TO GND. ***
 *
 * Q7S drives to VCC, which on this panel is 5 V, and the Orin Nano's 40-pin
 * GPIO is 3.3 V and NOT 5 V tolerant — a direct connection over-volts the SoC
 * input. The divider lands 5 V at 3.33 V and 0 V at 0 V.
 *
 * A divider is enough because the load is a high-impedance input: the 595 is
 * driving microamps, not the 4 mA at which Q7S is characterised. (Q7S is a
 * weaker pin than the Qn outputs in any case — ±25 mA absolute maximum against
 * their ±35 mA, per the datasheet's limiting values.)
 */
constexpr const char* LED595_QH_LOOP  = PIN16_GPIO23;
/**
 * Header 32 — OE, ACTIVE LOW, driven by PWM0 for global brightness.
 *
 * REQUIRES PINMUX. Header pins default to plain GPIO; PWM0 has to be enabled
 * with `sudo /opt/nvidia/jetson-io/jetson-io.py` followed by a reboot before
 * /sys/class/pwm/ appears at all.
 *
 * Needs an external pull-up to 5 V. At power-on, before the first latch, the
 * 595 outputs are undefined and OE floating means that garbage is displayed —
 * a pull-up holds the outputs disabled until software takes over.
 */
constexpr const char* LED595_OE_PWM   = PIN32_GPIO12;

/**
 * Panel electrical design, recorded here because the SOFTWARE DEPENDS ON IT.
 *
 * Hardware: 2x 74HCT595D, VCC = 5 V external (ground common with the Jetson),
 * common-anode RGB LEDs, anodes to 5 V, the 595s SINKING through per-element
 * resistors. Output LOW = lit.
 *
 * HCT AND NOT HC. Verified against the Nexperia 74HC595/74HCT595 datasheet
 * (docs/), which carries two separate static-characteristics tables:
 *
 *   74HC595    VIH  VCC = 4.5 V         3.15 V min   ( = 0.7 x VCC )
 *              VIH  VCC = 6.0 V         4.20 V min
 *   74HCT595   VIH  VCC = 4.5 to 5.5 V  2.00 V min
 *
 * So an HC part at 5 V wants 3.5 V to read HIGH and the Orin Nano drives 3.3 V
 * — 200 mV short. It works on a bench and drifts with temperature, and the
 * symptom (an occasional wrong colour, or a latched garbage frame) looks
 * exactly like a software fault. HCT leaves 1.3 V of margin instead.
 *
 * Resistors are sized for PERCEIVED balance, not equal current. The eye peaks
 * near 555 nm, so green needs roughly half the current of red or blue to look
 * as bright; blue gets the most because blue LEDs are the least luminous per mA
 * AND the eye is least sensitive there. Equal resistors make every mix read as
 * green — yellow, cyan and white all become "green-ish", which costs a
 * five-LED panel most of its vocabulary.
 *
 *   element  Vf     R       current
 *   red      1.9 V  560 R   5.2 mA
 *   green    2.4 V  1 k     2.5 mA
 *   blue     2.6 V  470 R   4.8 mA
 *
 * Worst case is all five white: ~33 mA on 595 #1 and ~30 mA on #2, against the
 * datasheet's ±70 mA absolute-maximum ICC/IGND per package. Half the limit,
 * which is why LED595_PWM_MAX_DUTY below can be 100 %.
 *
 * The per-pin figure is 5.3 mA worst case against a ±35 mA absolute maximum for
 * the Qn outputs. It is kept at or below 6 mA for a subtler reason: 6 mA is the
 * current at which the datasheet SPECIFIES VOL (0.16 V typ, 0.33 V max at
 * VCC = 4.5 V). Past it the part is not out of spec, its output voltage is
 * simply uncharacterised — and an uncharacterised VOL feeds straight back into
 * the LED current these values were computed from.
 */
constexpr float LED595_I_RED_MA   = 5.2f;
constexpr float LED595_I_GREEN_MA = 2.5f;
constexpr float LED595_I_BLUE_MA  = 4.8f;
/// Absolute-maximum continuous current through one package's GND pin (mA).
constexpr float LED595_I_PACKAGE_MAX_MA = 70.0f;

/**
 * Ceiling on the OE duty cycle, as a fraction.
 *
 * 1.0 — the whole range is available for brightness, because the resistors
 * above already keep the packages at ~50 % of their limit.
 *
 * This is worth stating as a constant rather than an absence, because it was
 * very nearly not 1.0: with the originally specified 330/220/220 resistors the
 * busier package reached ~75 mA at full white, PAST the absolute maximum, and
 * the only lever left would have been capping this at ~0.55 — spending the
 * dimming control on staying inside the current budget and leaving nothing for
 * night use. If the resistors are ever changed, recompute the package total
 * before touching this number.
 */
constexpr float LED595_PWM_MAX_DUTY = 1.0f;

/// PWM carrier for OE (Hz). Well above flicker fusion, and far below anything
/// the 595's output enable path cares about.
constexpr unsigned LED595_PWM_HZ = 2000u;

/**
 * Output order, 595 #1 first in the chain, MSB-first.
 *
 * Plain sequential, which is also what the walk test in the bring-up tool reads
 * against. An interleaved map was considered to balance current across the two
 * packages and turned out to be unnecessary once the resistors evened the
 * per-colour currents out — obvious wiring is debuggable wiring.
 *
 *   595 #1  Q0..Q7 -> LED1 R,G,B   LED2 R,G,B   LED3 R,G
 *   595 #2  Q0..Q6 -> LED3 B       LED4 R,G,B   LED5 R,G,B     (Q7 spare)
 */
constexpr unsigned LED595_COUNT      = 5u;   ///< RGB LEDs on the panel.
constexpr unsigned LED595_CHANNELS   = 15u;  ///< Elements driven (3 per LED).
constexpr unsigned LED595_CHAIN_BITS = 16u;  ///< Two 8-bit registers; bit 15 spare.

// ─── I2C buses on the 40-pin header (informational — use libi2c) ──────────────
// Pin 3  SDA1  →  /dev/i2c-1   (I2C_GP1_DAT)
// Pin 5  SCL1  →  /dev/i2c-1   (I2C_GP1_CLK)
// Pin 27 SDA0  →  /dev/i2c-0   (I2C_GP0_DAT)
// Pin 28 SCL0  →  /dev/i2c-0   (I2C_GP0_CLK)

// ─── SPI buses on the 40-pin header (informational — use libspi) ──────────────
// Pin 19 MOSI  /  Pin 21 MISO  /  Pin 23 CLK  /  Pin 24 CE0  →  /dev/spidev0.0
// Pin 26 CE1   →  /dev/spidev0.1

// ─── UART on the 40-pin header (informational — use libuart) ──────────────────
// Pin  8 TXD   /  Pin 10 RXD   →  /dev/ttyTHS1   (UART1, 3.3 V levels)

} // namespace dashcam::gpio::pins

#endif // LIBGPIO_DASHCAM_H
