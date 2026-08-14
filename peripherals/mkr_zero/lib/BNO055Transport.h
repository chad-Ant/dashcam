#ifndef BNO055_TRANSPORT_H
#define BNO055_TRANSPORT_H 1

/**
 * @file BNO055Transport.h
 * @brief The BNO055 driver's I2C hooks, routed through this project's bus layer.
 *
 * The Bosch driver takes @c bus_read, @c bus_write and @c delay_msec as function
 * pointers rather than calling Wire itself. That is the whole reason it was
 * chosen over Adafruit's, and this file is where the choice pays: every register
 * access the driver makes passes through @c i2cBusBegin(), is counted when it
 * fails, and is refused outright when the bus is known clamped.
 *
 * The alternative — the shipped @c BNO055_support.cpp — issued raw Wire calls
 * and discarded every return value. On a bus that has already produced
 * successful-but-corrupt transactions on this rig, a transport that cannot
 * report failure is not a transport worth having.
 *
 * @see peripherals/mkr_zero/vendor/BNO055/PATCHES.md
 */

#include <Arduino.h>
#include <BNO055.h>

#include "DataDictionary.h"   // IMU_I2C_CLOCK_HZ

#ifndef DASHCAM_BNO055_VENDORED
#error "Stock BNO055 detected. Build with --library peripherals/mkr_zero/vendor/BNO055 (see vendor/BNO055/PATCHES.md); the global copy resolves an unpinned driver whose bno055_init() overwrites the device address with 0x28, which is the WRONG one for a GY breakout."
#endif

/// I2C clock the BNO055 actually runs at — an ALIAS, not an independent setting.
///
/// This was 100000UL, described as a conservative rate to be raised later as a
/// separate measured change. **It never took effect.** @c i2cBusBegin() runs
/// @c i2cBusRecover() on the first call from any client, and that function ends
/// with @c Wire.begin() followed by @c Wire.setClock(IMU_I2C_CLOCK_HZ) — so the
/// 100 kHz a sketch set in @c setup() was overwritten by the first BNO055
/// transaction, and every bench run reported a clock it was not using.
///
/// The measurement it was waiting for therefore already exists, taken at
/// 400 kHz without anyone realising: 20 820 CHIP_ID reads with zero faults,
/// a bring-up that configures in 705 ms, and 1682 data polls with zero I/O
/// errors and a gravity magnitude holding 9.79-9.81. The clock stretching the
/// datasheet warns about (rev 1.4, section 4) is real and the SAMD21's SERCOM
/// handles it.
///
/// Aliased rather than set to 400000 so the two can never disagree again.
/// @c I2CBus owns the bus clock; this name exists for readers looking for it
/// under the part that uses it.
#define BNO055_I2C_CLOCK_HZ   IMU_I2C_CLOCK_HZ

/// Value @c BNO055_CHIP_ID_ADDR must return. Anything else means the address is
/// wrong or the read is corrupt — the driver's own init return cannot tell you.
#define BNO055_EXPECTED_CHIP_ID  0xA0u

/**
 * @brief Installs the transport hooks and the address into @p dev.
 *
 * Does NOT talk to the part; it only wires the struct up. Bring-up is staged
 * elsewhere because the BNO055 needs 400 ms from power-on and 650 ms from a
 * reset before it answers, and blocking that long is the mistake
 * @c gpsInitTick() exists to avoid.
 *
 * @param[out] dev      Driver context to populate.
 * @param[in]  address  7-bit address, normally from a scan. NOT defaulted:
 *                      the datasheet default is 0x29 and the driver's built-in
 *                      constant is 0x28, so guessing has a 50 % failure rate on
 *                      a board whose strapping you have not checked.
 */
void bno055TransportBind(struct bno055_t &dev, uint8_t address);

/** @brief Scans for a BNO055 at either strapping. 0 when neither answers. */
uint8_t bno055FindAddress();

// ─── the hooks themselves ─────────────────────────────────────────────────────
//
// Exposed as well as installed, because register-level access is exactly what a
// validation sketch needs: reading CHIP_ID ten thousand times through the SAME
// path production uses is the only way to prove that path, and going around it
// with bare Wire calls would prove something else.
//
// extern "C" and these exact signatures are fixed by BNO055_RD_FUNC_PTR /
// BNO055_WR_FUNC_PTR in the driver. Zero is success, non-zero failure.

extern "C" int  bno055BusRead (unsigned char dev_addr, unsigned char reg_addr,
                               unsigned char *reg_data, unsigned char cnt);
extern "C" int  bno055BusWrite(unsigned char dev_addr, unsigned char reg_addr,
                               unsigned char *reg_data, unsigned char cnt);
extern "C" void bno055Delay   (BNO055_MDELAY_DATA_TYPE ms);

/**
 * @brief Consecutive failed transfers since the last success.
 *
 * Mirrors the accel/mag fault counters this project already uses: one NACK is a
 * glitch worth riding out, a run of them is a device to retire.
 */
uint16_t bno055TransportFaults();

/** @brief Total failed transfers since boot, for the console status line. */
uint32_t bno055TransportErrorCount();

/** @brief Clears the consecutive-fault counter after a successful re-init. */
void bno055TransportResetFaults();

#endif // BNO055_TRANSPORT_H
