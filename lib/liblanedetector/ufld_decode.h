/**
 * @file ufld_decode.h
 * @brief Pure UFLD v2 decode: output tensors → lane slots → ego lane.
 *
 * This is the decode LaneDetector runs after TensorRT, as free functions so
 * tests can feed hand-built tensors and tools can re-decode dumped ones.
 * No CUDA, TensorRT or GStreamer here.
 *
 * Domains (the library's native ones): x in source pixels across the full
 * frame width (0..srcW); y as a fraction of the model's implied frame (see
 * LaneGeometry::impliedToSourceRow for source rows).  A slot's boundary is
 * the least-squares line x = slope·y + intercept over its decoded points.
 */

#ifndef UFLD_DECODE_H
#define UFLD_DECODE_H

#include <array>
#include <cstdint>
#include <vector>

namespace dashcam::lane {

/// UFLD v2 head geometry (culane_res18 defaults).
struct UfldDims {
    int numCellRow    = 200;   ///< row-anchor head location cells
    int numRowAnchors = 72;    ///< row-anchor sample positions
    int numCellCol    = 100;   ///< column-anchor head location cells
    int numColAnchors = 81;    ///< column-anchor sample positions
    int numLanes      = 4;     ///< lane slots (decode requires 4)
};

struct UfldDecodeParams {
    float srcW           = 0;     ///< full source width, px
    float rowAnchorStart = 0.42f; ///< first row anchor, implied fraction
    float laneReferenceY = 0.90f; ///< where xRef is sampled, implied fraction
};

/// Which head decoded a slot: 1,2 = row head (ego-adjacent), 0,3 = column head.
enum class LaneHead : uint8_t { None = 0, Row, Col };

struct LanePoint { float x, y; };

/// One lane boundary slot, in the native domains above.
struct LaneSlot {
    bool     detected  = false;
    LaneHead head      = LaneHead::None;
    int16_t  nPts      = 0;     ///< points that passed the existence test
    float    slope     = 0;     ///< px per implied fraction
    float    intercept = 0;     ///< px
    float    xRef      = 0;     ///< x at laneReferenceY (0 when not detected)
    float    yMin      = 0;     ///< smallest point y
    float    yMax      = 0;     ///< largest point y
    float    rmsPx     = 0;     ///< RMS x residual of the points about the line
};

struct UfldDecodeOut {
    std::array<LaneSlot, 4> slots{};
    int nonFinite = 0;                              ///< NaN/Inf values in the 4 tensors
    std::array<std::vector<LanePoint>, 4> points;   ///< only with wantPoints
};

/**
 * @brief Decode the four UFLD v2 outputs (batch dropped, row-major float32):
 *   locRow   [numCellRow][numRowAnchors][numLanes]
 *   locCol   [numCellCol][numColAnchors][numLanes]
 *   existRow [2][numRowAnchors][numLanes]   (0 = absent, 1 = present)
 *   existCol [2][numColAnchors][numLanes]
 * Slots 1,2 decode from the row head (present on > K/2 anchors), 0,3 from
 * the column head (> K/4), as the UFLD v2 reference demo does.
 */
UfldDecodeOut decodeUfldV2(const float* locRow, const float* locCol,
                           const float* existRow, const float* existCol,
                           const UfldDims& dims, const UfldDecodeParams& params,
                           bool wantPoints = false);

/// How the ego lane is chosen from the slots.
enum class EgoLogic : uint8_t {
    /// Sort the detected slots' xRef and take the gap holding the vehicle
    /// centre (slot identity is discarded).  What LaneDetector ships.
    kSortedV1 = 0,

    /// Candidate fix for plan bug B5 (not selected anywhere yet).  Keeps
    /// slot identity: UFLD v2 trains slots 1 and 2 as the ego lane's left and
    /// right boundaries, so the ego lane is [slot1, slot2] or nothing.
    ///  - lateralValid only when slots 1 and 2 are both detected, in order,
    ///    and the vehicle centre lies between them; a missing ego boundary
    ///    gives currentLaneIndex −1 instead of a merged double-width "lane".
    ///  - numLanes counts adjacent slot pairs (0-1, 1-2, 2-3) that are both
    ///    detected with increasing xRef; a crossed pair is not a lane.
    ///  - currentLaneIndex = 1 when lane 0-1 exists, else 0.
    kSlotIdentityV2 = 1,
};

struct LaneEgo {
    int8_t numLanes         = 0;    ///< detected boundaries − 1 (0 if < 2)
    int8_t currentLaneIndex = -1;   ///< −1 = vehicle centre outside all lanes
    float  lateralOffset    = 0;    ///< −1 left boundary, 0 centre, +1 right
    bool   lateralValid     = false;
};

/// Lane count, ego lane and lateral offset at laneReferenceY.
LaneEgo egoFromSlots(const std::array<LaneSlot, 4>& slots, float vehicleCenterX,
                     EgoLogic logic = EgoLogic::kSortedV1);

} // namespace dashcam::lane

#endif // UFLD_DECODE_H
