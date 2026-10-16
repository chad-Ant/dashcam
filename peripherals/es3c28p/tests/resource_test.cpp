#include "../lib/es3Video/VideoBuffers.h"
#include "../lib/es3Storage/CalibrationStore.h"
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
int main() { calibration(); buffers(); puts("calibration persistence and allocation failure checks passed"); }
