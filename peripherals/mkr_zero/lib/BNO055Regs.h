#ifndef BNO055_REGS_H
#define BNO055_REGS_H 1

/**
 * @file BNO055Regs.h
 * @brief BNO055 register map, mode constants and device identity.
 *
 * WHY THIS EXISTS RATHER THAN A VENDORED DRIVER
 * ---------------------------------------------
 * This project carried Bosch's reference driver — about 16 000 lines — in
 * @c vendor/BNO055/, and called exactly ONE function from it: @c bno055_init(),
 * which performs seven single-byte register reads. Everything else went through
 * this project's own transport, because the driver's mode handling could not be
 * used (it never waits for the switches it commands) and its data path was never
 * needed (the poll reads one 48-byte burst).
 *
 * Two things came with that driver and neither was worth it:
 *
 *   LICENCE. It declares GPLv3-or-later. This repository is MIT. Distributing
 *   the two together is a question nobody should have to answer, and the answer
 *   was being deferred rather than resolved.
 *
 *   A HAZARD SURFACE. Because the driver was resolved by name, a build without
 *   `--library vendor/BNO055` would silently link the copy in the user's global
 *   Arduino folder — whose @c bno055_init() overwrites the device address with
 *   0x28, the wrong one for this board. That needed a vendoring marker, an
 *   `#error` guard, a patch to the init function, C linkage guards, and two
 *   `#pragma GCC diagnostic` lines to silence 307 warnings the file emitted.
 *   Five patches to make one function safe to call.
 *
 * So the register map lives here, in about ninety lines, and the identity read
 * lives in @c bno055Identify(). Nothing resolves by name, nothing needs a
 * marker, and there is no second licence in the tree.
 *
 * Addresses are from the datasheet (BST-BNO055-DS000-14 rev 1.4, tables 4-2 and
 * 4-3) and were cross-checked against the driver they replace.
 */

#include <Arduino.h>

// ─── page 0: data, status and configuration ───────────────────────────────────

#define BNO055_CHIP_ID_ADDR         0x00u  ///< Always 0xA0.
#define BNO055_ACC_REV_ID_ADDR      0x01u
#define BNO055_MAG_REV_ID_ADDR      0x02u
#define BNO055_GYR_REV_ID_ADDR      0x03u
#define BNO055_SW_REV_ID_LSB_ADDR   0x04u  ///< 16-bit, LSB first.
#define BNO055_BL_REV_ID_ADDR       0x06u
#define BNO055_PAGE_ID_ADDR         0x07u  ///< Reachable from BOTH pages.

/// Start of the contiguous data block: accelerometer, magnetometer, gyroscope,
/// Euler angles, quaternion, linear acceleration, gravity, temperature,
/// calibration, self-test and interrupt status, in that order, through 0x37.
#define BNO055_ACC_DATA_X_LSB_ADDR  0x08u

#define BNO055_TEMP_ADDR            0x34u
#define BNO055_CALIB_STAT_ADDR      0x35u
#define BNO055_ST_RESULT_ADDR       0x36u
#define BNO055_INT_STA_ADDR         0x37u
#define BNO055_SYS_STATUS_ADDR      0x39u
#define BNO055_SYS_ERR_ADDR         0x3Au
#define BNO055_UNIT_SEL_ADDR        0x3Bu
#define BNO055_OPR_MODE_ADDR        0x3Du
#define BNO055_PWR_MODE_ADDR        0x3Eu
#define BNO055_SYS_TRIGGER_ADDR     0x3Fu
#define BNO055_AXIS_MAP_CONFIG_ADDR 0x41u
#define BNO055_AXIS_MAP_SIGN_ADDR   0x42u

/// Calibration offsets and radii, 22 bytes. Readable and writable only in CONFIG.
#define BNO055_ACC_OFFSET_X_LSB_ADDR 0x55u

// ─── page 1: sensor configuration and interrupts ──────────────────────────────

#define BNO055_P1_ACC_CONFIG_ADDR   0x08u  ///< Range, bandwidth, power mode.
#define BNO055_P1_INT_MSK_ADDR      0x0Fu  ///< Which interrupts drive the INT pin.
#define BNO055_P1_INT_EN_ADDR       0x10u  ///< Which interrupts are evaluated.
#define BNO055_P1_ACC_INT_SET_ADDR  0x12u  ///< Per-axis enables and sample count.
#define BNO055_P1_ACC_HG_DUR_ADDR   0x13u  ///< High-G duration.
#define BNO055_P1_ACC_HG_THRES_ADDR 0x14u  ///< High-G threshold.

// ─── operating modes (OPR_MODE, low nibble) ───────────────────────────────────

#define OPERATION_MODE_CONFIG       0x00u
#define OPERATION_MODE_ACCONLY      0x01u
#define OPERATION_MODE_MAGONLY      0x02u
#define OPERATION_MODE_GYRONLY      0x03u
#define OPERATION_MODE_ACCMAG       0x04u
#define OPERATION_MODE_ACCGYRO      0x05u
#define OPERATION_MODE_MAGGYRO      0x06u
#define OPERATION_MODE_AMG          0x07u  ///< Raw, no fusion. Range selectable.
#define OPERATION_MODE_IMUPLUS      0x08u  ///< Fusion without the magnetometer.
#define OPERATION_MODE_COMPASS      0x09u
#define OPERATION_MODE_M4G          0x0Au
#define OPERATION_MODE_NDOF_FMC_OFF 0x0Bu
#define OPERATION_MODE_NDOF         0x0Cu

// ─── power modes (PWR_MODE, low two bits) ─────────────────────────────────────

#define POWER_MODE_NORMAL           0x00u
#define POWER_MODE_LOW_POWER        0x01u
#define POWER_MODE_SUSPEND          0x02u

// ─── accelerometer range (ACC_CONFIG bits 1:0, non-fusion modes only) ─────────

#define ACCEL_RANGE_2G              0x00u
#define ACCEL_RANGE_4G              0x01u
#define ACCEL_RANGE_8G              0x02u
#define ACCEL_RANGE_16G             0x03u

/**
 * @brief What the part says it is.
 *
 * Replaces @c bno055_t. Carries only the fields anything reads, and drops the
 * three function pointers the vendored driver needed to reach a bus — this
 * project calls its transport directly, so indirection through a struct bought
 * nothing but a way for the hooks to be unset.
 */
struct BNO055Device{
    uint8_t  address;          ///< 7-bit address it answered on. 0 = not found.
    uint8_t  chipId;           ///< Must be 0xA0.
    uint8_t  accelRevId;
    uint8_t  magRevId;
    uint8_t  gyroRevId;
    uint8_t  bootloaderRevId;
    /**
     * Firmware revision, LSB and MSB combined.
     *
     * FULL 16 BITS. The vendored driver declared this field as @c unsigned char
     * and then assigned a 16-bit composite to it, so the major version was
     * discarded and a part running 3.08 reported as 0x08 — which on the bench
     * looked exactly like the second byte of a two-byte read having failed, and
     * cost time being investigated as a bus fault. Fixed by not reproducing it.
     */
    uint16_t swRevId;
    uint8_t  pageId;
};

/**
 * @brief Reads the identity registers. Replaces @c bno055_init().
 *
 * Returns a HONEST verdict, which the function it replaces did not: that one
 * assigned its status from each of its several reads in turn, so the value it
 * returned described only the last one and a success return said nothing about
 * whether the part had answered. Callers had to check @c chip_id themselves and
 * the vendored copy carried a note saying so. Here a false return means at least
 * one read failed OR the chip ID is wrong, and there is nothing left to check.
 *
 * @param[out] dev      Populated on success; @c address is set either way.
 * @param[in]  address  7-bit address, normally from @c bno055FindAddress().
 */
bool bno055Identify(BNO055Device &dev, uint8_t address);

#endif // BNO055_REGS_H
