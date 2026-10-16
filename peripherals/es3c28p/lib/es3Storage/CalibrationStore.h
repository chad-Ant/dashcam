#pragma once
#include "es3Calibration.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

namespace es3storage {
constexpr char kCalibrationFile[] = "/es3c28p_touch.cal";
constexpr char kCalibrationBackup[] = "/es3c28p_touch.cal.bak";
constexpr char kCalibrationTemporary[] = "/es3c28p_touch.cal.tmp";
constexpr size_t kCalibrationBytes = 96;

inline bool whitespace(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }
inline bool parseCalibration(const char* text, size_t size, Es3TouchCalibrationRecord& record) {
    const char prefix[] = "ES3C28P_TOUCH_CAL_V3";
    const size_t prefixSize = sizeof(prefix) - 1;
    if (!text || size <= prefixSize || size >= kCalibrationBytes ||
        memcmp(text, prefix, prefixSize)) return false;
    size_t pos = prefixSize;
    int values[8]{};
    for (unsigned i = 0; i < 8; ++i) {
        if (pos == size || !whitespace(text[pos])) return false;
        while (pos < size && whitespace(text[pos])) ++pos;
        if (pos == size || text[pos] < '0' || text[pos] > '9') return false;
        while (pos < size && text[pos] >= '0' && text[pos] <= '9') {
            values[i] = values[i] * 10 + (text[pos++] - '0');
            if (values[i] > 319) return false; // also bounds arithmetic
        }
    }
    while (pos < size && whitespace(text[pos])) ++pos;
    if (pos != size || values[0] > 1) return false;
    const Es3TouchCalibrationRecord parsed{values[0] == 1, values[1], values[2], values[3],
        values[4], values[5], values[6], values[7]};
    if (!es3CalibrationIsSane(parsed)) return false;
    record = parsed; return true;
}
inline size_t formatCalibration(const Es3TouchCalibrationRecord& r, char (&out)[kCalibrationBytes]) {
    if (!es3CalibrationIsSane(r)) return 0;
    const int size = snprintf(out, sizeof(out), "ES3C28P_TOUCH_CAL_V3 %d %d %d %d %d %d %d %d\n",
        r.xAxisIsRawX ? 1 : 0, r.xAtLeft, r.xAtLeftMiddle, r.xAtRightMiddle, r.xAtRight,
        r.yAtTop, r.yAtCenter, r.yAtBottom);
    return size > 0 && size_t(size) < sizeof(out) ? size_t(size) : 0;
}
inline bool sameCalibration(const Es3TouchCalibrationRecord& a, const Es3TouchCalibrationRecord& b) {
    return a.xAxisIsRawX == b.xAxisIsRawX && a.xAtLeft == b.xAtLeft &&
        a.xAtLeftMiddle == b.xAtLeftMiddle && a.xAtRightMiddle == b.xAtRightMiddle &&
        a.xAtRight == b.xAtRight && a.yAtTop == b.yAtTop && a.yAtCenter == b.yAtCenter &&
        a.yAtBottom == b.yAtBottom;
}

// Store supplies read/write/exists/remove/rename. Preserve a readable generation
// across short writes and failed renames; FAT power-loss atomicity is not assumed.
template<class Store>
bool loadCalibration(Store& store, Es3TouchCalibrationRecord& record) {
    return store.read(kCalibrationFile, record) || store.read(kCalibrationBackup, record);
}
template<class Store>
bool saveCalibration(Store& store, const Es3TouchCalibrationRecord& record) {
    if (!es3CalibrationIsSane(record)) return false;
    if (store.exists(kCalibrationTemporary) && !store.remove(kCalibrationTemporary)) return false;
    Es3TouchCalibrationRecord check{};
    if (!store.write(kCalibrationTemporary, record) || !store.read(kCalibrationTemporary, check) ||
        !sameCalibration(check, record)) return false;
    if (store.exists(kCalibrationFile)) {
        if (!store.exists(kCalibrationBackup) || store.read(kCalibrationFile, check)) {
            if (store.exists(kCalibrationBackup) && !store.remove(kCalibrationBackup)) return false;
            if (!store.rename(kCalibrationFile, kCalibrationBackup)) return false;
        } else {
            // A read failure may be transient. Never remove the primary unless
            // another readable generation is known to survive a failed promote.
            if (!store.read(kCalibrationBackup, check) || !store.remove(kCalibrationFile)) return false;
        }
    }
    return store.rename(kCalibrationTemporary, kCalibrationFile) &&
        store.read(kCalibrationFile, check) && sameCalibration(check, record);
}
template<class Store>
bool clearCalibration(Store& store) {
    // Remove backup first: a failed primary removal must leave the active file.
    const char* paths[] = {kCalibrationBackup, kCalibrationTemporary, kCalibrationFile};
    for (const char* path : paths) {
        if (store.exists(path) && !store.remove(path)) return false;
    }
    return true;
}
} // namespace es3storage
