/**
 * @file lane_geometry.h
 * @brief Pure crop / preprocess / implied-frame geometry of the lane detector.
 *
 * computeLaneGeometry() is what LaneDetector's constructor uses: the same
 * validation (same exceptions) and the same arithmetic, without loading an
 * engine — so tests and tools can ask where the model looks for any source
 * size and config (LaneGeometry is documented in liblanedetector.h).
 */

#ifndef LANE_GEOMETRY_H
#define LANE_GEOMETRY_H

#include <cstdint>
#include "liblanedetector.h"

namespace dashcam::lane {

/**
 * @param srcW, srcH  FULL tee frame size (before the branch crop).
 * @param cfg         Detector config; the engine path is not looked at.
 * @throws std::runtime_error for the same config errors as LaneDetector
 *         (zero size, numLanes != 4, head geometry < 2, cropRatio outside
 *         (0, 1], crops outside [0, 0.9], fewer than 64 rows left).
 */
LaneGeometry computeLaneGeometry(uint32_t srcW, uint32_t srcH,
                                 const LaneDetectorConfig& cfg);

} // namespace dashcam::lane

#endif // LANE_GEOMETRY_H
