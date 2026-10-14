#include "../lib/es3Storage/es3Calibration.h"
#include <cassert>
#include <climits>

int main()
{
    // Landscape panel: native Y drives screen X, native X drives screen Y.
    const Es3TouchCalibrationRecord normal{false, 24, 114, 205, 295, 215, 119, 24};
    assert(es3CalibrationIsSane(normal));
    auto r = normal;
    r.xAtLeft = 295; r.xAtLeftMiddle = 205; r.xAtRightMiddle = 114; r.xAtRight = 24;
    assert(es3CalibrationIsSane(r));
    r = normal; r.xAtLeftMiddle = r.xAtLeft;
    assert(!es3CalibrationIsSane(r));  // repeated target / zero divisor
    r = normal; r.xAtLeftMiddle = 205; r.xAtRightMiddle = 114;
    assert(!es3CalibrationIsSane(r));  // targets tapped out of order
    r = normal; r.yAtCenter = r.yAtBottom;
    assert(!es3CalibrationIsSane(r));
    r = normal; r.xAtRight = INT_MAX; r.xAtLeft = INT_MIN;
    assert(!es3CalibrationIsSane(r));  // malformed persisted integers
    r = normal; r.yAtTop = 240;
    assert(!es3CalibrationIsSane(r));  // native axis bounds
    r = normal; r.xAxisIsRawX = true;
    assert(!es3CalibrationIsSane(r));  // mismatched orientation / dimensions
    r = {true, 18, 85, 153, 221, 32, 160, 287};
    assert(es3CalibrationIsSane(r));
    assert(!es3CalibrationIsSane({}));
}
