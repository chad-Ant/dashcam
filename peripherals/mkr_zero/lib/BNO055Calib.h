#ifndef BNO055_CALIB_H
#define BNO055_CALIB_H 1

/**
 * @file BNO055Calib.h
 * @brief Saving and restoring the BNO055's calibration offsets via the SD card.
 *
 * WHY THIS IS REQUIRED RATHER THAN NICE TO HAVE. The BNO055 keeps its
 * calibration in volatile registers and loses them on every power-on. Nothing
 * on the part remembers them, and the fusion algorithm cannot be told them by
 * any other route — so without this, every drive begins with an uncalibrated
 * inertial sensor and stays that way until the vehicle happens to move through
 * enough of the right motions. The datasheet says to store and reload them; this
 * is that.
 *
 * The bench measurement that made it concrete: accelerometer calibration fell to
 * 0 after 1400 polls and stayed there for 9000 more. A board bolted into a
 * bracket does not see the orientations Bosch's algorithm wants, so on a vehicle
 * that figure may never recover on its own within a drive. Restoring a profile
 * captured once, by hand, on a bench, is the only practical way it is ever
 * right at key-on.
 *
 * WHAT IS AND IS NOT PORTABLE. These offsets are properties of the SILICON —
 * zero-rate bias, accelerometer offset, the fusion's radii — not of how the
 * board is mounted, so a profile survives the sensor being unbolted and moved.
 * It does NOT survive the sensor being replaced.
 *
 * THAT LAST CASE CANNOT BE DETECTED AUTOMATICALLY, and an earlier version of
 * this file claimed it could. It stored CHIP_ID and refused a profile whose chip
 * did not match — but every BNO055 reports 0xA0, so the check passed for any
 * BNO055 including a brand new one, and a replacement sensor silently inherited
 * the old unit's biases. The part exposes no serial number to do better with.
 *
 * So the guard is honest instead of automatic: the file carries an INSTALL ID
 * that the firmware supplies, and a profile whose ID does not match the running
 * firmware's is refused. Replacing the sensor means bumping
 * @c BNO055_CALIB_INSTALL_ID or deleting the file — which the file says on its
 * own first line, because the person holding a screwdriver is the only one who
 * knows the sensor changed.
 *
 * @see peripherals/mkr_zero/lib/BNO055Init.h — the bring-up this hooks into
 */

#include <Arduino.h>

#include "BNO055Init.h"

/**
 * Bytes in a calibration profile: registers 0x55..0x6A.
 *
 * Accelerometer offset (6), magnetometer offset (6), gyroscope offset (6),
 * accelerometer radius (2), magnetometer radius (2).
 *
 * The magnetometer's share is stored even though fusion mode keeps that sensor
 * switched off. It costs eight bytes, it is what the register block contains,
 * and writing back a partial block would leave whatever the previous power cycle
 * happened to put in the gaps.
 */
#define BNO055_CALIB_BYTES  22u

/// Where the profile lives. One file, not one per vehicle: it describes the
/// SENSOR, and the same sensor in a different car has the same biases.
#define BNO055_CALIB_PATH   "bno055.cal"

/// Largest file @c bno055CalibLoad() will parse, in bytes.
///
/// A REFUSAL BOUND, not a format limit. The load runs with the watchdog armed
/// and reads to EOF, so the parse time is a function of the file's size — which
/// is an attribute of the CARD, not of this firmware. A large file at this path
/// takes the watchdog with it, and the next boot reads the same file: a reboot
/// loop that no amount of retry logic escapes.
///
/// @c bno055CalibStore() emits under 224 bytes. 512 leaves room for comments and
/// hand-editing while staying far below anything that could outlast a watchdog
/// period.
#define BNO055_CALIB_MAX_FILE_BYTES  512u

/**
 * @brief Reads the offset registers. REQUIRES CONFIG MODE.
 *
 * The registers are only readable in CONFIG, so a caller in an operating mode
 * must switch, read, and switch back — @c bno055CalibCapture() does that and is
 * what callers should normally use.
 *
 * @param[in]  state  Configured bring-up state.
 * @param[out] out    @c BNO055_CALIB_BYTES bytes.
 * @return false when the read failed.
 */
bool bno055CalibRead(const BNO055InitState &state, uint8_t *out);

/**
 * @brief Writes the offset registers. REQUIRES CONFIG MODE.
 *
 * Called by the bring-up machine while it is already parked in CONFIG, which is
 * the only point in the sequence where this is both legal and free.
 *
 * @return false when any byte failed to write or read back.
 */
bool bno055CalibWrite(const BNO055InitState &state, const uint8_t *in);

/**
 * @brief Captures the live profile from a RUNNING sensor.
 *
 * Drops the part to CONFIG, reads the offsets, and returns it to its operating
 * mode. That costs roughly 50 ms during which no samples are produced, so it is
 * for a deliberate save and not for a poll loop.
 *
 * @return false when a mode switch or the read failed. The part is returned to
 *         its operating mode either way.
 */
bool bno055CalibCapture(BNO055InitState &state, uint8_t *out);

/**
 * @brief Loads the stored profile from the card.
 *
 * A MISSING FILE IS NOT AN ERROR and must not be treated as one — it is the
 * ordinary state of a rig that has never been calibrated, and of one that lost
 * power inside the write window. Distinguished from a corrupt file, which is.
 *
 * Rejects a profile that is malformed, fails CRC, or contains offsets outside
 * the ranges the datasheet allows — a file can be perfectly intact and still
 * hold values captured from a sensor that was already misbehaving.
 *
 * @param[out] out        @c BNO055_CALIB_BYTES bytes, untouched unless @c true.
 * @param[out] installId  Install ID the profile was written under, for the
 *                        caller's own check; may be nullptr.
 * @return true only when a well-formed, CRC-valid, plausible profile was read.
 */
bool bno055CalibLoad(uint8_t *out, uint16_t *installId);

/**
 * @brief Stores a profile on the card, replacing any previous one.
 *
 * @return false when the card is absent or the write failed. The previous
 *         profile is left intact on failure.
 */
bool bno055CalibStore(const uint8_t *profile, uint16_t installId);

/** @brief True when the two profiles are byte-identical. */
bool bno055CalibEqual(const uint8_t *a, const uint8_t *b);

/**
 * @brief Human-readable one-line summary, for the console.
 * @param buf  At least 80 bytes.
 */
void bno055CalibDescribe(const uint8_t *profile, char *buf, size_t bufLen);

#endif // BNO055_CALIB_H
