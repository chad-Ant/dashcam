#include "ufld_decode.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace dashcam::lane {

namespace {

// Output layouts (batch dropped, row-major):
//   loc_row  [G=numCellRow][K=numRowAnchors][L]  location logits
//   exist_row[2]           [K]               [L]  0=absent, 1=present
//   loc_col  [G=numCellCol][K=numColAnchors][L]
//   exist_col[2]           [K]               [L]
// Lane slots: 1,2 decode via the row head, 0,3 via the column head
// (fixed by the UFLD v2 architecture / reference demo).
//
// The arithmetic below is LaneDetector's original decode, moved verbatim:
// same float types and operation order, so results are bit-identical — on
// finite outputs; non-finite ones are now excluded (plan bug B9).

float rowAnchorY(const UfldDims& d, const UfldDecodeParams& p, int k) {
    const float a = p.rowAnchorStart;
    return a + (1.0f - a) * static_cast<float>(k)
             / static_cast<float>(d.numRowAnchors - 1);
}

// Soft local argmax over cells [g*-1, g*+1] (demo local_width = 1):
// returns the sub-cell location in grid units, offset by +0.5.
float softLocalArgmax(const float* loc, int G, int stride, int off, int gStar) {
    const int lo = std::max(0, gStar - 1);
    const int hi = std::min(G - 1, gStar + 1);
    float m = loc[lo * stride + off];
    for (int g = lo + 1; g <= hi; ++g)
        m = std::max(m, loc[g * stride + off]);
    float se = 0.0f, acc = 0.0f;
    for (int g = lo; g <= hi; ++g) {
        const float e = std::exp(loc[g * stride + off] - m);
        se  += e;
        acc += e * static_cast<float>(g);
    }
    return acc / se + 0.5f;
}

// Least-squares line x = slope·y + intercept from accumulated moments.
bool lineFromMoments(int n, float sumX, float sumY, float sumYX, float sumYY,
                     float& slope, float& intercept) {
    if (n < 2) return false;
    const float invN = 1.0f / static_cast<float>(n);
    const float covYX = sumYX - sumY * sumX * invN;
    const float varY  = sumYY - sumY * sumY * invN;
    if (varY < 1e-6f) return false;
    slope = covYX / varY;
    intercept = (sumX - slope * sumY) * invN;
    return true;
}

// The point (x, y) of anchor k of a slot, or false when the anchor's
// existence logits say absent.  Row head: y is the anchor, x the location;
// column head: x is the anchor, y the location.
struct HeadView {
    const float* loc;
    const float* exist;
    int G, K, L;
    bool rowHead;
};

// Present: exist[1] > exist[0] — false for NaN logits, so a NaN anchor is
// absent both in the existence count and in the fit (plan bug B9: the fit
// used to skip only on exist[1] <= exist[0], which a NaN never satisfies).
bool anchorPresent(const HeadView& h, int k, int lane) {
    return h.exist[1 * h.K * h.L + k * h.L + lane] > h.exist[0 * h.K * h.L + k * h.L + lane];
}

LanePoint anchorPoint(const HeadView& h, const UfldDims& d, const UfldDecodeParams& p,
                      int k, int lane) {
    const int off = k * h.L + lane;
    int   gStar = 0;
    float best  = h.loc[off];
    for (int g = 1; g < h.G; ++g) {
        const float v = h.loc[g * h.K * h.L + off];
        if (v > best) { best = v; gStar = g; }
    }
    const float cell = softLocalArgmax(h.loc, h.G, h.K * h.L, off, gStar);
    if (h.rowHead) {
        const float x = cell / (h.G - 1) * p.srcW;
        const float y = rowAnchorY(d, p, k);
        return { x, y };
    }
    const float x = static_cast<float>(k)
                  / static_cast<float>(h.K - 1) * p.srcW;
    const float y = cell / (h.G - 1);
    return { x, y };
}

LaneSlot decodeSlot(const HeadView& h, const UfldDims& d, const UfldDecodeParams& p,
                    int lane, std::vector<LanePoint>* points) {
    LaneSlot s;
    s.head = h.rowHead ? LaneHead::Row : LaneHead::Col;

    int existCount = 0;
    for (int k = 0; k < h.K; ++k)
        if (anchorPresent(h, k, lane)) ++existCount;

    // demo: row head needs sum > K/2, column head sum > K/4.
    if (h.rowHead ? existCount * 2 <= h.K : existCount * 4 <= h.K) return s;

    float sumX = 0.0f, sumY = 0.0f, sumYX = 0.0f, sumYY = 0.0f;
    float yMin = 0.0f, yMax = 0.0f;
    int n = 0;
    for (int k = 0; k < h.K; ++k) {
        if (!anchorPresent(h, k, lane)) continue;
        const LanePoint pt = anchorPoint(h, d, p, k, lane);
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;   // NaN/Inf location logits (B9)
        const float x = pt.x, y = pt.y;
        sumX += x; sumY += y; sumYX += y * x; sumYY += y * y;
        yMin = n == 0 ? y : std::min(yMin, y);
        yMax = n == 0 ? y : std::max(yMax, y);
        ++n;
        if (points) points->push_back(pt);
    }
    if (!lineFromMoments(n, sumX, sumY, sumYX, sumYY, s.slope, s.intercept)
        || !std::isfinite(s.slope) || !std::isfinite(s.intercept)) {
        s.slope = s.intercept = 0.0f;
        return s;
    }
    s.detected = true;
    s.nPts = static_cast<int16_t>(n);
    s.yMin = yMin;
    s.yMax = yMax;
    s.xRef = s.slope * p.laneReferenceY + s.intercept;

    // Fit quality: a second pass over the same points (recomputed, not
    // stored, so the common path allocates nothing).
    double se = 0.0;
    for (int k = 0; k < h.K; ++k) {
        if (!anchorPresent(h, k, lane)) continue;
        const LanePoint pt = anchorPoint(h, d, p, k, lane);
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;
        const double r = static_cast<double>(pt.x)
                       - (static_cast<double>(s.slope) * pt.y + s.intercept);
        se += r * r;
    }
    s.rmsPx = static_cast<float>(std::sqrt(se / n));
    return s;
}

int countNonFinite(const float* v, size_t n) {
    int c = 0;
    for (size_t i = 0; i < n; ++i)
        if (!std::isfinite(v[i])) ++c;
    return c;
}

} // namespace

UfldDecodeOut decodeUfldV2(const float* locRow, const float* locCol,
                           const float* existRow, const float* existCol,
                           const UfldDims& dims, const UfldDecodeParams& params,
                           bool wantPoints) {
    UfldDecodeOut out;
    const int L = dims.numLanes;
    const HeadView row{ locRow, existRow, dims.numCellRow, dims.numRowAnchors, L, true };
    const HeadView col{ locCol, existCol, dims.numCellCol, dims.numColAnchors, L, false };

    const int slots = std::min(L, static_cast<int>(out.slots.size()));
    for (int i = 0; i < slots; ++i) {
        const bool rowSlot = (i == 1 || i == 2);
        auto* pts = wantPoints ? &out.points[static_cast<size_t>(i)] : nullptr;
        out.slots[static_cast<size_t>(i)] =
            decodeSlot(rowSlot ? row : col, dims, params, i, pts);
    }

    const size_t nRow = static_cast<size_t>(dims.numRowAnchors) * L;
    const size_t nCol = static_cast<size_t>(dims.numColAnchors) * L;
    out.nonFinite = countNonFinite(locRow,   nRow * dims.numCellRow)
                  + countNonFinite(locCol,   nCol * dims.numCellCol)
                  + countNonFinite(existRow, nRow * 2)
                  + countNonFinite(existCol, nCol * 2);
    return out;
}

static LaneEgo egoSlotIdentity(const std::array<LaneSlot, 4>& s, float vehicleX) {
    auto lane = [&](int a, int b) {
        return s[static_cast<size_t>(a)].detected && s[static_cast<size_t>(b)].detected
            && s[static_cast<size_t>(a)].xRef < s[static_cast<size_t>(b)].xRef;
    };
    LaneEgo e;
    e.numLanes = static_cast<int8_t>(lane(0, 1) + lane(1, 2) + lane(2, 3));
    if (!lane(1, 2)) return e;
    const float left = s[1].xRef, right = s[2].xRef;
    if (vehicleX < left || vehicleX > right) return e;
    e.currentLaneIndex = static_cast<int8_t>(lane(0, 1) ? 1 : 0);
    const float centre    = 0.5f * (left + right);
    const float halfWidth = 0.5f * (right - left);
    if (halfWidth > 1.0f) {
        e.lateralOffset = (vehicleX - centre) / halfWidth;
        e.lateralValid  = true;
    }
    return e;
}

LaneEgo egoFromSlots(const std::array<LaneSlot, 4>& slots, float vehicleCenterX,
                     EgoLogic logic) {
    if (logic == EgoLogic::kSlotIdentityV2) return egoSlotIdentity(slots, vehicleCenterX);

    std::array<float, 4> xs{};
    size_t xsCount = 0;
    for (const auto& s : slots)
        if (s.detected) xs[xsCount++] = s.xRef;
    std::sort(xs.begin(), xs.begin() + static_cast<std::ptrdiff_t>(xsCount));

    LaneEgo e;
    if (xsCount < 2) return e;
    e.numLanes = static_cast<int8_t>(xsCount - 1);

    const float vehicleX = vehicleCenterX;
    e.currentLaneIndex = -1;
    for (size_t i = 0; i + 1 < xsCount; ++i) {
        if (vehicleX >= xs[i] && vehicleX <= xs[i + 1]) {
            e.currentLaneIndex = static_cast<int8_t>(i);
            // Lateral projection inside the ego lane: -1 on the left
            // boundary, 0 centred, +1 on the right boundary.
            const float centre    = 0.5f * (xs[i] + xs[i + 1]);
            const float halfWidth = 0.5f * (xs[i + 1] - xs[i]);
            if (halfWidth > 1.0f) {   // degenerate/crossed boundaries guard
                e.lateralOffset = (vehicleX - centre) / halfWidth;
                e.lateralValid  = true;
            }
            break;
        }
    }
    return e;
}

} // namespace dashcam::lane
