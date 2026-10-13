/**
 * @file cydStorage.h
 * @brief SD-card storage service for the ESP32-2432S028 CYD.
 *
 * Owns the CYD's SD SPI bus and all dashboard persistence. Other components
 * receive this class by reference rather than including SD.h or knowing card
 * pins/filesystem details.
 */

#pragma once

#include <Arduino.h>
#include <FS.h>
#include <SPI.h>

struct CydTouchCalibrationRecord {
    bool xAxisIsRawX;
    int xAtLeft;
    int xAtLeftMiddle;
    int xAtRightMiddle;
    int xAtRight;
    int yAtTop;
    int yAtCenter;
    int yAtBottom;
};

class CydStorage {
public:
    CydStorage();

    /** Initialises the CYD SD bus and checks for a readable card. */
    bool begin();

    /** Reinitialises the SD filesystem after a card insertion or replacement. */
    bool remount();

    bool available() const;
    uint64_t capacityBytes() const;
    /** Actual SPI clock selected during the last successful mount. */
    uint32_t spiFrequencyHz() const;
    bool exists(const char* path) const;

    /** Reads /cyd_touch.cal. Returns false for absent, malformed, or bad data. */
    bool loadTouchCalibration(CydTouchCalibrationRecord& record) const;

    /** Replaces /cyd_touch.cal with a validated calibration record. */
    bool saveTouchCalibration(const CydTouchCalibrationRecord& record);

    /** Removes the calibration file; no error if it is already absent. */
    bool clearTouchCalibration();

    /** Opens a readable SD-card file without exposing SD bus configuration. */
    bool openRead(const char* path, File& file);

private:
    SPIClass _spi;
    bool _available;
    uint64_t _capacityBytes;
    uint32_t _spiFrequencyHz;

    bool calibrationIsSane(const CydTouchCalibrationRecord& record) const;
    void logRootDirectory() const;
};
