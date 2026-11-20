#include "../lib/es3Video/VideoBuffers.h"
#include "../lib/es3Storage/CalibrationStore.h"
#include "../lib/es3Storage/SettingsStore.h"
#include <cassert>
#include <cstdlib>
#include <map>
#include <string>
#include <cstdio>

using namespace es3storage;
struct Files {
    std::map<std::string, std::string> files;
    unsigned operation = 0, failAt = 0;
    bool corruptWrite = false, unreadablePrimary = false;
    bool fail() { return ++operation == failAt; }
    bool exists(const char* p) { return files.count(p); }
    bool remove(const char* p) { if (fail()) return false; return files.erase(p) != 0; }
    bool rename(const char* from, const char* to) {
        if (fail() || !exists(from) || exists(to)) return false;
        files[to] = files[from]; files.erase(from); return true;
    }
    bool write(const char* p, const Es3TouchCalibrationRecord& r) {
        char text[kCalibrationBytes]; const auto n = formatCalibration(r, text);
        if (!n || fail()) { files[p] = "short"; return false; }
        files[p] = corruptWrite ? "corrupted" : std::string(text, n); return true;
    }
    bool write(const char* p, const Es3Settings& r) {
        char text[kSettingsBytes]; const auto n = formatSettings(r, text);
        if (!n || fail()) { files[p] = "short"; return false; }
        files[p] = corruptWrite ? "corrupted" : std::string(text, n); return true;
    }
    bool read(const char* p, Es3Settings& r) {
        if (unreadablePrimary && std::string(p) == kSettingsFile) return false;
        const auto it = files.find(p);
        return it != files.end() && parseSettings(it->second.data(), it->second.size(), r);
    }
    bool read(const char* p, Es3TouchCalibrationRecord& r) {
        if (unreadablePrimary && std::string(p) == kCalibrationFile) return false;
        const auto it = files.find(p);
        return it != files.end() && parseCalibration(it->second.data(), it->second.size(), r);
    }
};
static void calibration() {
    const Es3TouchCalibrationRecord old{false, 24, 114, 205, 295, 215, 119, 24};
    auto newer = old; newer.xAtLeft = 25;
    char text[kCalibrationBytes]; const auto n = formatCalibration(old, text);
    Es3TouchCalibrationRecord out{};
    assert(parseCalibration(text, n, out) && sameCalibration(out, old));
    const char* bad[] = {
        "ES3C28P_TOUCH_CAL_V3 0 24 114 205 295 215 119 24 junk",
        "ES3C28P_TOUCH_CAL_V3 0 999999999999999999 114 205 295 215 119 24",
        "ES3C28P_TOUCH_CAL_V3 0 -24 114 205 295 215 119 24",
        "ES3C28P_TOUCH_CAL_V3 0 24 24 205 295 215 119 24",
        "ES3C28P_TOUCH_CAL_V3 0 24 114 205 295 215 119",
    };
    for (auto* value : bad) assert(!parseCalibration(value, strlen(value), out));
    Files original; assert(saveCalibration(original, old));
    // Also exercise removal of an existing backup/temp at the next save.
    assert(saveCalibration(original, old));
    original.files[kCalibrationTemporary] = "leftover";
    Files success = original; success.operation = 0;
    assert(saveCalibration(success, newer));
    const unsigned operations = success.operation;
    for (unsigned i = 1; i <= operations; ++i) {
        Files interrupted = original; interrupted.operation = 0; interrupted.failAt = i;
        assert(!saveCalibration(interrupted, newer));
        assert(loadCalibration(interrupted, out));
        assert(sameCalibration(out, old));
        // Retry recovers from each failure, including the missing-primary window.
        interrupted.failAt = 0;
        assert(saveCalibration(interrupted, newer));
        assert(loadCalibration(interrupted, out) && sameCalibration(out, newer));
    }
    Files corrupt = original; corrupt.corruptWrite = true;
    assert(!saveCalibration(corrupt, newer));
    assert(loadCalibration(corrupt, out) && sameCalibration(out, old));
    // Simulate power interruption after primary -> backup. Ignore temp on boot.
    Files interrupted = original; interrupted.files.erase(kCalibrationFile);
    assert(loadCalibration(interrupted, out) && sameCalibration(out, old));
    Files transient; assert(saveCalibration(transient, old));
    transient.operation = 0; transient.failAt = 3; // write, rotate, failed promote
    transient.unreadablePrimary = true;
    assert(!saveCalibration(transient, newer));
    transient.unreadablePrimary = false;
    assert(loadCalibration(transient, out) && sameCalibration(out, old));
    assert(clearCalibration(success));
    assert(success.files.empty() && !loadCalibration(success, out));
}
static void settings() {
    const Es3Settings defaults{10, 100, 50}, chosen{35, 60, 72};
    // Round trip of the firmware's own format, and the exact file text.
    char text[kSettingsBytes]; size_t n = formatSettings(chosen, text);
    assert(n > 0 && n < kSettingsBytes);
    assert(std::string(text, n) ==
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<settings version=\"1\">\n"
        "  <masterVolume>35</masterVolume>\n  <brightness>60</brightness>\n"
        "  <micThreshold>72</micThreshold>\n</settings>\n");
    Es3Settings out = defaults;
    assert(parseSettings(text, n, out) && sameSettings(out, chosen));
    // Hand edits: whitespace, other order, unknown elements.
    const char* edited = "<settings>\r\n<extra>7</extra><micThreshold> 0 </micThreshold>"
                         "<brightness>100</brightness><masterVolume>\n5\n</masterVolume></settings>";
    out = defaults; assert(parseSettings(edited, strlen(edited), out));
    assert(out.masterVolume == 5 && out.brightness == 100 && out.micThreshold == 0);
    // A bad or missing element keeps only that default.
    const char* partial = "<settings version=\"1\"><masterVolume>101</masterVolume>"
                          "<brightness>4x</brightness><micThreshold>20</micThreshold></settings>";
    out = defaults; assert(parseSettings(partial, strlen(partial), out));
    assert(out.masterVolume == 10 && out.brightness == 100 && out.micThreshold == 20);
    const char* tooManyDigits = "<settings><masterVolume>0010</masterVolume></settings>";
    out = defaults; assert(parseSettings(tooManyDigits, strlen(tooManyDigits), out) && sameSettings(out, defaults));
    const char* unclosed = "<settings><masterVolume>30</settings>";
    out = defaults; assert(parseSettings(unclosed, strlen(unclosed), out) && sameSettings(out, defaults));
    // Not a settings document: rejected, defaults untouched.
    const char* bad[] = {"", "garbage", "<setting><masterVolume>30</masterVolume></setting>",
                         "<settings><masterVolume>30</masterVolume>", "<settingsX></settings>"};
    for (const char* value : bad) {
        out = defaults; assert(!parseSettings(value, strlen(value), out) && sameSettings(out, defaults));
    }
    std::string big = "<settings>" + std::string(kSettingsBytes, ' ') + "</settings>";
    out = defaults; assert(!parseSettings(big.data(), big.size(), out));
    // An unterminated buffer is read only within its size.
    const char cut[] = {'<','s','e','t','t','i','n','g','s','>','<','/','s','e','t','t','i','n','g','s','>','X'};
    out = defaults; assert(parseSettings(cut, sizeof(cut) - 1, out));
    Files rejected; assert(!saveSettings(rejected, Es3Settings{101, 0, 0}) && rejected.files.empty());

    // Persistence: absent file keeps defaults; each interrupted save keeps a
    // readable previous generation, and a retry succeeds.
    Files none; out = defaults; assert(!loadSettings(none, out) && sameSettings(out, defaults));
    Files original; assert(saveSettings(original, defaults) && saveSettings(original, defaults));
    out = Es3Settings{}; assert(loadSettings(original, out) && sameSettings(out, defaults));
    Files success = original; success.operation = 0;
    assert(saveSettings(success, chosen));
    const unsigned operations = success.operation;
    for (unsigned i = 1; i <= operations; ++i) {
        Files interrupted = original; interrupted.operation = 0; interrupted.failAt = i;
        assert(!saveSettings(interrupted, chosen));
        out = Es3Settings{}; assert(loadSettings(interrupted, out) && sameSettings(out, defaults));
        interrupted.failAt = 0;
        assert(saveSettings(interrupted, chosen));
        out = Es3Settings{}; assert(loadSettings(interrupted, out) && sameSettings(out, chosen));
    }
    Files corrupt = original; corrupt.corruptWrite = true;
    assert(!saveSettings(corrupt, chosen));
    out = Es3Settings{}; assert(loadSettings(corrupt, out) && sameSettings(out, defaults));
    Files lostPrimary = success; lostPrimary.files.erase(kSettingsFile);
    out = Es3Settings{}; assert(loadSettings(lostPrimary, out) && sameSettings(out, defaults));
    // Settings and calibration files never collide.
    assert(std::string(kSettingsFile) != kCalibrationFile && success.files.count(kSettingsBackup));
}
static void buffers() {
    using namespace es3video;
    for (unsigned failure = 0; failure <= 8; ++failure) {
        VideoState state; uint32_t capacity[4]{}; bool pan = false;
        std::map<void*, size_t> live;
        unsigned calls = 0;
        auto allocate = [&](size_t n) -> void* {
            if (++calls == failure) return nullptr;
            void* p = malloc(n); assert(p); live[p] = n; return p;
        };
        auto release = [&](void* p) { if (p) { assert(live.erase(p) == 1); free(p); } };
        const bool ok = allocateBuffers(state, capacity, pan, allocate, release);
        assert(ok == (failure == 0 || failure > 4));
        assert(pan == (failure == 0));
        for (unsigned i = 0; i < 4; ++i) {
            assert(capacity[i] * sizeof(uint16_t) == (state.pixels[i] ? live.at(state.pixels[i]) : 0));
        }
        freeBuffers(state, capacity, release); // also simulates worker creation failure
        assert(live.empty());
        for (auto value : capacity) assert(value == 0);
        // Retry a previous successful upgrade with a failed upgrade this time.
        unsigned retryCalls = 0;
        auto retryAllocate = [&](size_t n) -> void* {
            if (++retryCalls == 5) return nullptr;
            void* p = malloc(n); assert(p); live[p] = n; return p;
        };
        assert(allocateBuffers(state, capacity, pan, retryAllocate, release));
        assert(!pan && capacity[0] == kFramePixels && capacity[1] == kFramePixels);
        assert(capacity[2] == 0 && capacity[3] == 0);
        freeBuffers(state, capacity, release); assert(live.empty());
    }
}
int main() {
    calibration(); settings(); buffers();
    puts("calibration and settings persistence and allocation failure checks passed");
}
