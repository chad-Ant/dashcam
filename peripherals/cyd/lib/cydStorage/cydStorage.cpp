/** @file cydStorage.cpp */

#include "cydStorage.h"

#include <SD.h>
#include <cstdio>
#include <cstdlib>

namespace {
constexpr int kSdMiso = 19;
constexpr int kSdMosi = 23;
constexpr int kSdSclk = 18;
constexpr int kSdCs = 5;
constexpr uint32_t kPreferredSdFrequencyHz = 20000000;
constexpr uint32_t kFallbackSdFrequencyHz = 10000000;
constexpr char kTouchCalibrationFile[] = "/cyd_touch.cal";
}  // namespace

CydStorage::CydStorage()
    : _spi(VSPI)
    , _available(false)
    , _capacityBytes(0)
    , _spiFrequencyHz(0)
{
}

bool CydStorage::begin()
{
    return remount();
}

bool CydStorage::remount()
{
    _available = false;
    _capacityBytes = 0;
    _spiFrequencyHz = 0;
    SD.end();
    _spi.end();
    _spi.begin(kSdSclk, kSdMiso, kSdMosi, kSdCs);

    // Most CYD card sockets are reliable at 20 MHz. Fall back to the original
    // conservative rate when a particular card, socket, or wiring cannot
    // initialise there.
    const uint32_t frequencies[] = {kPreferredSdFrequencyHz, kFallbackSdFrequencyHz};
    for (const uint32_t frequency : frequencies) {
        if (SD.begin(kSdCs, _spi, frequency) && SD.cardType() != CARD_NONE) {
            _spiFrequencyHz = frequency;
            break;
        }
        SD.end();
    }
    if (_spiFrequencyHz == 0) {
        return false;
    }
    _capacityBytes = SD.cardSize();
    _available = _capacityBytes > 0;
    if (_available) {
        Serial.printf("[cyd] SD mounted at %lu MHz\n",
                      static_cast<unsigned long>(_spiFrequencyHz / 1000000U));
    }
    return _available;
}

bool CydStorage::available() const { return _available; }
uint64_t CydStorage::capacityBytes() const { return _capacityBytes; }
uint32_t CydStorage::spiFrequencyHz() const { return _spiFrequencyHz; }
bool CydStorage::exists(const char* path) const
{
    return _available && path != nullptr && path[0] == '/' && SD.exists(path);
}

bool CydStorage::calibrationIsSane(const CydTouchCalibrationRecord& record) const
{
    const auto isBetween = [](int value, int endA, int endB) {
        return value >= min(endA, endB) && value <= max(endA, endB);
    };
    return std::abs(record.xAtRight - record.xAtLeft) > 600 &&
           std::abs(record.yAtBottom - record.yAtTop) > 600 &&
           isBetween(record.xAtLeftMiddle, record.xAtLeft, record.xAtRight) &&
           isBetween(record.xAtRightMiddle, record.xAtLeft, record.xAtRight) &&
           isBetween(record.yAtCenter, record.yAtTop, record.yAtBottom);
}

bool CydStorage::loadTouchCalibration(CydTouchCalibrationRecord& record) const
{
    if (!_available || !SD.exists(kTouchCalibrationFile)) {
        return false;
    }
    File file = SD.open(kTouchCalibrationFile, FILE_READ);
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
    CydTouchCalibrationRecord parsed{};
    const int fields = std::sscanf(line, "CYD_TOUCH_CAL_V3 %d %d %d %d %d %d %d %d", &rawXAxis,
                                   &parsed.xAtLeft, &parsed.xAtLeftMiddle,
                                   &parsed.xAtRightMiddle, &parsed.xAtRight,
                                   &parsed.yAtTop, &parsed.yAtCenter, &parsed.yAtBottom);
    if (fields != 8 || (rawXAxis != 0 && rawXAxis != 1)) {
        return false;
    }
    parsed.xAxisIsRawX = rawXAxis == 1;
    if (!calibrationIsSane(parsed)) {
        return false;
    }
    record = parsed;
    return true;
}

bool CydStorage::saveTouchCalibration(const CydTouchCalibrationRecord& record)
{
    if (!_available || !calibrationIsSane(record)) {
        return false;
    }
    if (SD.exists(kTouchCalibrationFile) && !SD.remove(kTouchCalibrationFile)) {
        return false;
    }
    File file = SD.open(kTouchCalibrationFile, FILE_WRITE);
    if (!file) {
        return false;
    }
    const int written = file.printf("CYD_TOUCH_CAL_V3 %d %d %d %d %d %d %d %d\n",
                                   record.xAxisIsRawX ? 1 : 0,
                                   record.xAtLeft, record.xAtLeftMiddle,
                                   record.xAtRightMiddle, record.xAtRight,
                                   record.yAtTop, record.yAtCenter, record.yAtBottom);
    file.close();
    return written > 0;
}

bool CydStorage::clearTouchCalibration()
{
    if (!_available || !SD.exists(kTouchCalibrationFile)) {
        return _available;
    }
    return SD.remove(kTouchCalibrationFile);
}

bool CydStorage::openRead(const char* path, File& file)
{
    if (!_available || path == nullptr || path[0] != '/') {
        return false;
    }
    file = SD.open(path, FILE_READ);
    // ESP32 SD implementations normally accept both names, but accepting both
    // avoids a filesystem-wrapper difference between core versions.
    if (!file && path[1] != '\0') {
        file = SD.open(path + 1, FILE_READ);
    }
    if (!file) {
        // The card may have been removed and reinserted after startup. Remount
        // once before reporting a missing file so FAT directory state is fresh.
        Serial.printf("[cyd] SD retrying after remount: %s\n", path);
        if (remount()) {
            file = SD.open(path, FILE_READ);
            if (!file && path[1] != '\0') {
                file = SD.open(path + 1, FILE_READ);
            }
        }
    }
    if (!file) {
        Serial.printf("[cyd] SD cannot open %s; refreshed root contains:\n", path);
        logRootDirectory();
        return false;
    }
    return true;
}

void CydStorage::logRootDirectory() const
{
    if (!_available) {
        return;
    }
    File root = SD.open("/");
    if (!root) {
        Serial.println("[cyd]   <root unavailable>");
        return;
    }
    for (File entry = root.openNextFile(); entry; entry = root.openNextFile()) {
        Serial.printf("[cyd]   %s%s (%lu bytes)\n", entry.name(),
                      entry.isDirectory() ? "/" : "",
                      static_cast<unsigned long>(entry.size()));
        entry.close();
    }
    root.close();
}
