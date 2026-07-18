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
