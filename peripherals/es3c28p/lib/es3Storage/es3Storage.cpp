/** @file es3Storage.cpp */

#include "es3Storage.h"

#include <SD_MMC.h>
#include "es3Board.h"
#include "CalibrationStore.h"

namespace {
// The policy is portable/testable; this adapter alone touches SD_MMC.
struct CalibrationFiles {
    bool exists(const char* path) { return SD_MMC.exists(path); }
    bool remove(const char* path) { return SD_MMC.remove(path); }
    bool rename(const char* from, const char* to) { return SD_MMC.rename(from, to); }
    bool read(const char* path, Es3TouchCalibrationRecord& record) {
        File file = SD_MMC.open(path, FILE_READ);
        if (!file || file.isDirectory()) return false;
        const size_t size = file.size();
        if (!size || size >= es3storage::kCalibrationBytes) return false;
        char text[es3storage::kCalibrationBytes];
        return file.read(reinterpret_cast<uint8_t*>(text), size) == size &&
            es3storage::parseCalibration(text, size, record);
    }
    bool write(const char* path, const Es3TouchCalibrationRecord& record) {
        char text[es3storage::kCalibrationBytes];
        const size_t size = es3storage::formatCalibration(record, text);
        if (!size) return false;
        File file = SD_MMC.open(path, FILE_WRITE);
        if (!file) return false;
        const bool ok = file.write(reinterpret_cast<const uint8_t*>(text), size) == size;
        file.flush(); file.close();
        return ok;
    }
};
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
    if (!_available) return false;
    CalibrationFiles files;
    return es3storage::loadCalibration(files, record);
}

bool Es3Storage::saveTouchCalibration(const Es3TouchCalibrationRecord& record)
{
    if (!_available) return false;
    CalibrationFiles files;
    return es3storage::saveCalibration(files, record);
}

bool Es3Storage::clearTouchCalibration()
{
    if (!_available) return false;
    CalibrationFiles files;
    return es3storage::clearCalibration(files);
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
        Serial0.printf("[es3] SD cannot open %s; root contains:\n", path);
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
    unsigned listed = 0;
    for (File entry = root.openNextFile(); entry && listed < 32; entry = root.openNextFile(), ++listed) {
        Serial0.printf("[es3]   %s%s (%lu bytes)\n", entry.name(),
                      entry.isDirectory() ? "/" : "",
                      static_cast<unsigned long>(entry.size()));
        entry.close();
    }
    root.close();
}
