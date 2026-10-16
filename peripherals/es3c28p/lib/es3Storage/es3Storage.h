/**
 * @file es3Storage.h
 * @brief SD-card storage service for the ES3C28P.
 *
 * Owns the ES3C28P's SDMMC bus and all dashboard persistence. Other components
 * receive this class by reference rather than including SD_MMC.h or knowing card
 * pins/filesystem details.
 */

#pragma once

#include <Arduino.h>
#include <FS.h>

#include "es3Calibration.h"

class Es3Storage {
public:
    Es3Storage();

    /** Initialises the ES3C28P SD bus and checks for a readable card. */
    bool begin();

    /** Reinitialises SD after card replacement. Close all File handles first. */
    bool remount();

    bool available() const;
    uint64_t capacityBytes() const;
    /** Actual SDMMC clock selected during the last successful mount. */
    uint32_t busFrequencyHz() const;
    bool exists(const char* path) const;

    /** Reads /es3c28p_touch.cal. Returns false for absent, malformed, or bad data. */
    bool loadTouchCalibration(Es3TouchCalibrationRecord& record) const;

    /** Verifies a temporary record, retains a backup, then promotes it. */
    bool saveTouchCalibration(const Es3TouchCalibrationRecord& record);

    /** Removes the calibration file; no error if it is already absent. */
    bool clearTouchCalibration();

    /** Opens a readable file; missing files never unmount active readers. */
    bool openRead(const char* path, File& file);

private:
    bool _available;
    uint64_t _capacityBytes;
    uint32_t _busFrequencyHz;

    void logRootDirectory() const;
};
