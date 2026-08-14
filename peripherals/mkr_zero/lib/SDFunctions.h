#ifndef SDFUNCTIONS_H
#define SDFUNCTIONS_H 1

/**
 * @file SDFunctions.h
 * @brief The MKR Zero's onboard microSD, and the primitives built on it.
 *
 * This header previously declared six functions with no implementation anywhere
 * in the repository — `SDFunctions.cpp` did not exist.  Nothing included it, so
 * the gap never surfaced; the first caller would have got a link error for
 * something that read as supported.  It is now real, and shaped around the
 * consumer that actually needs it: the vehicle CAN signal map.
 *
 * ── WHICH CARD, WHICH BUS ────────────────────────────────────────────────────
 * The onboard slot is on **SPI1** (`SDCARD_SPI` / `SDCARD_SS_PIN` in the MKR
 * Zero variant), a different SERCOM from the MCP2515's SPI.  That separation is
 * why card access cannot contend with CAN traffic, and it is why
 * @c initializeSD() takes no chip-select argument: there is one card on this
 * board, and a caller free to name a pin is a caller free to put the filesystem
 * on the CAN bus.
 *
 * @warning `DataDictionary.h`'s `SD_CS_PIN` (4) describes an EXTERNAL module on
 *          the main SPI bus.  It is not this card.  See vendor/SdFat/PATCHES.md
 *          for why SdFat's own `SDCARD_SPI` define does not help here either.
 */

#include <Arduino.h>
#include <SdFat.h>

#include "DataDictionary.h"

#ifndef DASHCAM_SDFAT_VENDORED
#error "Stock SdFat detected. Build with --library peripherals/mkr_zero/vendor/SdFat (see vendor/SdFat/PATCHES.md); the global copy is unpinned and nothing version-checks it."
#endif

/**
 * @brief Opens the card root so a caller can walk it with @c File32::openNext().
 *
 * Exists because the vehicle map is found by PATTERN, not by a fixed name, and
 * only this translation unit knows whether the card is mounted.
 *
 * @param[out] dir Directory handle; the caller closes it.
 * @return false if no card is mounted or the root could not be opened.
 */
bool sdOpenRoot(File32 &dir);

/// Longest line any parser built on @c sdReadLine() will accept.
#define SD_MAX_LINE         96u

/** Return codes for all SD-card operations. */
enum class SDReturnStatus {
    OK               =  0, ///< Operation succeeded.
    NOK              = -1, ///< Unspecified failure.
    NOK_INIT_FAILED  = -2, ///< Card absent, unformatted, or wiring fault.
    NOK_NOT_FOUND    = -3, ///< Target file does not exist.
    NOK_WRITE_FAILED = -4, ///< Open, write or sync failed.
    NOK_PARSE_ERROR  = -5, ///< File opened but contained nothing usable.
};

/**
 * @brief Mounts the onboard card.  Idempotent; safe to call more than once.
 *
 * Bounded by SdFat's own card-init timeout, so an empty slot fails fast rather
 * than hanging — which is why this is safe to call even on a boot that has
 * quarantined the I2C bus.
 *
 * @return @c OK, or @c NOK_INIT_FAILED if the card could not be reached.
 */
SDReturnStatus initializeSD();

/** @brief True once @c initializeSD() has succeeded. */
bool sdReady();

/**
 * @brief Opens a file for reading.
 * @return false if the card is not mounted or the file does not exist.
 */
bool sdOpenRead(const char *path, File32 &f);

/**
 * @brief Reads one line into @p buf, NUL-terminated, with CR/LF stripped.
 *
 * The bounded primitive every parser here is built on.  A line longer than
 * @p bufLen is TRUNCATED and its remainder discarded up to the next newline, so
 * a corrupt or accidentally-binary file can neither overrun the caller's buffer
 * nor wedge the reader in a loop that never finds a terminator.
 *
 * @return false at end of file with nothing read.
 */
bool sdReadLine(File32 &f, char *buf, size_t bufLen);

/**
 * @brief Replaces a small text file, atomically. For CONFIG, not for logging.
 *
 * WHY THIS MODULE NOW WRITES AT ALL. It was read-only on purpose — see the note
 * below on OBD2 logging, which still stands. This is not logging: it is an
 * occasional, bounded write of a few dozen bytes that the BNO055's datasheet
 * requires, because the sensor loses its calibration offsets on every power-on
 * and cannot be told them again from anywhere else. Without it every drive
 * begins with an uncalibrated inertial sensor.
 *
 * ATOMIC BECAUSE THE POWER IS NOT. This runs in a vehicle whose supply
 * disappears with the ignition, and the file it writes is read back at the next
 * boot and pushed into the sensor's registers. A half-written profile is worse
 * than a missing one: a missing profile costs a warm-up, while a truncated one
 * loads plausible-looking garbage into the offsets and biases every reading
 * afterwards. So the write goes to a temporary file, is flushed, and only then
 * replaces the target — a cut at any point leaves either the old profile or no
 * new one, never a partial.
 *
 * @param path  Destination. A sibling temporary is created alongside it.
 * @param text  NUL-terminated content, written verbatim.
 * @return @c OK, @c NOK_INIT_FAILED when no card is mounted, or
 *         @c NOK_WRITE_FAILED if any step failed — in which case the previous
 *         contents of @p path are left untouched.
 */
SDReturnStatus sdWriteTextAtomic(const char *path, const char *text);

/*
 * Deliberately NOT declared here: OBD2 CSV logging.
 *
 * This header used to declare openOBD2Log() / writeOBD2LogEntry() /
 * closeOBD2Log() and an OBD2LogEntry struct, none of which were ever
 * implemented.  Logging belongs to the Orin Nano, which has the storage, the
 * clock and the budget for it.  A declaration without an implementation is worse
 * than an absence: it advertises a capability that fails at link.
 *
 * @c sdWriteTextAtomic() above is not a step back toward that: it writes a fixed
 * handful of bytes when a calibration converges, not a stream that grows with
 * time. If a second caller ever wants it for something periodic, that is the
 * moment to say no again.
 */

#endif // SDFUNCTIONS_H
