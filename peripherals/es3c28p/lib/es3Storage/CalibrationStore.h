#pragma once
#include "es3Calibration.h"
#include "GenerationFiles.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

namespace es3storage {
// Calibrations for the previous mirrored orientation use a different path.
constexpr char kCalibrationFile[] = "/es3c28p_touch_ips.cal";
constexpr char kCalibrationBackup[] = "/es3c28p_touch_ips.cal.bak";
constexpr char kCalibrationTemporary[] = "/es3c28p_touch_ips.cal.tmp";
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

constexpr GenerationPaths kCalibrationPaths{kCalibrationFile, kCalibrationBackup, kCalibrationTemporary};

template<class Store>
bool loadCalibration(Store& store, Es3TouchCalibrationRecord& record) {
    return loadGeneration(store, kCalibrationPaths, record);
}
template<class Store>
bool saveCalibration(Store& store, const Es3TouchCalibrationRecord& record) {
    if (!es3CalibrationIsSane(record)) return false;
    return saveGeneration(store, kCalibrationPaths, record, sameCalibration);
}
template<class Store>
bool clearCalibration(Store& store) {
    return clearGeneration(store, kCalibrationPaths);
}
} // namespace es3storage
