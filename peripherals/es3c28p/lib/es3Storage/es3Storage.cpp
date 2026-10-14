/** @file es3Storage.cpp */

#include "es3Storage.h"

#include <SD_MMC.h>
#include "es3Board.h"
#include <cstdio>
#include <cstdlib>

namespace {
constexpr char kTouchCalibrationFile[] = "/es3c28p_touch.cal";
}  // namespace

Es3Storage::Es3Storage()
    : _available(false)
    , _capacityBytes(0)
    , _busFrequencyHz(0)
{
}

bool Es3Storage::begin()
{
    return remount();
}

bool Es3Storage::remount()
{
    _available = false;
    _capacityBytes = 0;
    _busFrequencyHz = 0;
    SD_MMC.end();
    if (!SD_MMC.setPins(Es3Board::sdClock, Es3Board::sdCommand,
                        Es3Board::sdD0, Es3Board::sdD1, Es3Board::sdD2, Es3Board::sdD3)) {
        return false;
    }
    // SD_MMC takes kHz, unlike SPI SD.begin(). Never format on failure.
    const uint32_t frequencies[] = {20000, 10000};
    for (const uint32_t frequency : frequencies) {
        if (SD_MMC.begin("/sdcard", false, false, frequency) &&
            SD_MMC.cardType() != CARD_NONE) {
            _busFrequencyHz = frequency * 1000U;
            break;
        }
        SD_MMC.end();
    }
    if (_busFrequencyHz == 0) {
        return false;
    }
    _capacityBytes = SD_MMC.cardSize();
    _available = _capacityBytes > 0;
    if (_available) {
        Serial0.printf("[es3] SD mounted at %lu MHz\n",
                      static_cast<unsigned long>(_busFrequencyHz / 1000000U));
    }
    return _available;
}

bool Es3Storage::available() const { return _available; }
uint64_t Es3Storage::capacityBytes() const { return _capacityBytes; }
uint32_t Es3Storage::busFrequencyHz() const { return _busFrequencyHz; }
bool Es3Storage::exists(const char* path) const
{
    return _available && path != nullptr && path[0] == '/' && SD_MMC.exists(path);
}

bool Es3Storage::loadTouchCalibration(Es3TouchCalibrationRecord& record) const
{
    if (!_available || !SD_MMC.exists(kTouchCalibrationFile)) {
        return false;
    }
    File file = SD_MMC.open(kTouchCalibrationFile, FILE_READ);
    if (!file) {
        return false;
    }
    char line[96]{};
    const size_t bytes = file.readBytesUntil('\n', line, sizeof(line) - 1);
    file.close();
    if (bytes == 0) {
        return false;
    }

    int rawXAxis = 0;
    Es3TouchCalibrationRecord parsed{};
    const int fields = std::sscanf(line, "ES3C28P_TOUCH_CAL_V3 %d %d %d %d %d %d %d %d", &rawXAxis,
                                   &parsed.xAtLeft, &parsed.xAtLeftMiddle,
                                   &parsed.xAtRightMiddle, &parsed.xAtRight,
                                   &parsed.yAtTop, &parsed.yAtCenter, &parsed.yAtBottom);
    if (fields != 8 || (rawXAxis != 0 && rawXAxis != 1)) {
        return false;
    }
    parsed.xAxisIsRawX = rawXAxis == 1;
    if (!es3CalibrationIsSane(parsed)) {
        return false;
    }
    record = parsed;
    return true;
}

bool Es3Storage::saveTouchCalibration(const Es3TouchCalibrationRecord& record)
{
    if (!_available || !es3CalibrationIsSane(record)) {
        return false;
    }
    if (SD_MMC.exists(kTouchCalibrationFile) && !SD_MMC.remove(kTouchCalibrationFile)) {
        return false;
    }
    File file = SD_MMC.open(kTouchCalibrationFile, FILE_WRITE);
    if (!file) {
        return false;
    }
    const int written = file.printf("ES3C28P_TOUCH_CAL_V3 %d %d %d %d %d %d %d %d\n",
                                   record.xAxisIsRawX ? 1 : 0,
                                   record.xAtLeft, record.xAtLeftMiddle,
                                   record.xAtRightMiddle, record.xAtRight,
                                   record.yAtTop, record.yAtCenter, record.yAtBottom);
    file.close();
    return written > 0;
}

bool Es3Storage::clearTouchCalibration()
{
    if (!_available || !SD_MMC.exists(kTouchCalibrationFile)) {
        return _available;
    }
    return SD_MMC.remove(kTouchCalibrationFile);
}

bool Es3Storage::openRead(const char* path, File& file)
{
    if (!_available || path == nullptr || path[0] != '/') {
        return false;
    }
    file = SD_MMC.open(path, FILE_READ);
    // ESP32 SD implementations normally accept both names, but accepting both
    // avoids a filesystem-wrapper difference between core versions.
    if (!file && path[1] != '\0') {
        file = SD_MMC.open(path + 1, FILE_READ);
    }
    if (!file) {
        // The card may have been removed and reinserted after startup. Remount
        // once before reporting a missing file so FAT directory state is fresh.
        Serial0.printf("[es3] SD retrying after remount: %s\n", path);
        if (remount()) {
            file = SD_MMC.open(path, FILE_READ);
            if (!file && path[1] != '\0') {
                file = SD_MMC.open(path + 1, FILE_READ);
            }
        }
    }
    if (!file) {
        Serial0.printf("[es3] SD cannot open %s; refreshed root contains:\n", path);
        logRootDirectory();
        return false;
    }
    return true;
}

void Es3Storage::logRootDirectory() const
{
    if (!_available) {
        return;
    }
    File root = SD_MMC.open("/");
    if (!root) {
        Serial0.println("[es3]   <root unavailable>");
        return;
    }
    for (File entry = root.openNextFile(); entry; entry = root.openNextFile()) {
        Serial0.printf("[es3]   %s%s (%lu bytes)\n", entry.name(),
                      entry.isDirectory() ? "/" : "",
                      static_cast<unsigned long>(entry.size()));
        entry.close();
    }
    root.close();
}
