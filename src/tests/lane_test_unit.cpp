// lane_test_unit.cpp — lane_test --unit: CPU-only tests of liblanedetector's
// pure parts (no CUDA, no TensorRT, no camera; < 5 s).
//
//   G1–G8  computeLaneGeometry: crops, preprocess ROI, implied-frame mapping
//   D1–D12 decodeUfldV2 / egoFromSlots on hand-built UFLD v2 tensors
//   U6     the output-path guard (data must never land in the public repo)
//   A1–A5  lane_drive's camera-free parts (lane_drive_aux.h): clip ring,
//          clip budget, clip planner, mount monitor, tensor ring
//
// Hand-built tensors: an anchor is made "present" with exist logits (0, 1)
// and its location logits are −10 everywhere except +10 at the chosen cell,
// so the soft local argmax returns that cell exactly and the decoded point
// is known in closed form:
//   row head (slots 1,2): x = (g + 0.5) / (numCellRow − 1) · W,  y = anchor k
//   col head (slots 0,3): x = k / (numColAnchors − 1) · W,  y = (g + 0.5) / (numCellCol − 1)

#include "lane_drive_aux.h"
#include "lane_test_common.h"
#include "lane_geometry.h"
#include "liblanedetector.h"
#include "ufld_decode.h"

#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <type_traits>
#include <utility>

using namespace dashcam::lane;
using lanetest::TestCase;
using lanetest::TestEnv;
using lanetest::bugCheck;
using lanetest::check;
using lanetest::note;

namespace {

std::string f(double v, int prec = 3) {
    char b[48];
    std::snprintf(b, sizeof(b), "%.*f", prec, v);
    return b;
}

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// ─── geometry ────────────────────────────────────────────────────────────────

LaneDetectorConfig cropCfg(float top, float bottom, float refY = 0.90f) {
    LaneDetectorConfig c;
    c.inputCropTop    = top;
    c.inputCropBottom = bottom;
    c.laneReferenceY  = refY;
    return c;
}

void testG1Imx296Defaults(const TestEnv&) {
    const LaneGeometry g = computeLaneGeometry(1456, 1088, cropCfg(0.5f, 0.2059f));
    check(g.cropTopPx == 544 && g.cropBottomPx == 224,
          "G1 crops 544 / 224 (got " + std::to_string(g.cropTopPx) + " / " + std::to_string(g.cropBottomPx) + ")");
    check(g.branchW == 1456 && g.branchH == 320, "G1 band 1456x320");
    check(g.effRatio == 1.0f && g.preYScale == 1.0f && g.preYOff == 0.0f,
          "G1 effRatio 1, preYScale 1, preYOff 0 (no resampling)");
    check(near(g.impliedH, 1600.0 / 3, 0.01) && near(g.impliedTopSrc, 330.667, 0.01),
          "G1 implied frame 533.33 rows from source row 330.67 (got " + f(g.impliedH) + ", " + f(g.impliedTopSrc) + ")");
    check(near(g.refRowSrc, 810.667, 0.01), "G1 refY 0.90 -> source row 810.67 (got " + f(g.refRowSrc) + ")");
    check(near(g.anchorRowSrcFirst, 554.667, 0.01) && near(g.anchorRowSrcLast, 864.0, 0.01),
          "G1 row anchors span source rows 554.67..864.0");
}

void testG2NoCrop(const TestEnv&) {
    // No branch crop: the preprocess reproduces the training transform
    // (resize to H/0.6, keep the bottom 320 rows).
    const LaneGeometry g = computeLaneGeometry(1280, 720, cropCfg(0.0f, 0.0f));
    check(g.branchH == 720 && near(g.effRatio, 0.6, 1e-6), "G2 full band, effRatio 0.6");
    check(g.resizedH == 533 && g.cropTopRows == 213, "G2 resizedH 533, cropTopRows 213");
    check(near(g.impliedH, 720.0, 0.5) && near(g.impliedTopSrc, 0.0, 0.5),
          "G2 implied frame ~= the source frame (" + f(g.impliedTopSrc) + " + " + f(g.impliedH) + ")");
}

void testG3BottomOnly(const TestEnv&) {
    // A bottom crop alone does NOT clamp effRatio (the old comment said it did).
    const LaneGeometry g = computeLaneGeometry(1456, 1088, cropCfg(0.0f, 0.2f));
    check(g.cropBottomPx == 218 && g.branchH == 870, "G3 bottom crop 218, band 870 rows");
    check(near(g.effRatio, 0.75, 0.002), "G3 effRatio 0.75 (got " + f(g.effRatio, 4) + ")");
}

void testG4Usb720(const TestEnv&) {
    const LaneGeometry g = computeLaneGeometry(1280, 720, cropCfg(0.5f, 0.2059f));
    check(g.cropTopPx == 360 && g.cropBottomPx == 148 && g.branchH == 212, "G4 crops 360 / 148, band 212");
    check(near(g.preYScale, 0.6625, 1e-6), "G4 vertical upsample, preYScale 0.6625");
    check(near(g.refRowSrc, 536.667, 0.01), "G4 refRow 536.67 (got " + f(g.refRowSrc) + ")");
    // The appsink caps (gstConversion override) need this exact height: the
    // header's "height - (top+bottom)*height" gives 211.75, which is wrong.
    check(g.branchH == 720 - (static_cast<uint32_t>(std::lround(720 * 0.5f)) & ~1u)
                           - (static_cast<uint32_t>(std::lround(720 * 0.2059f)) & ~1u),
          "G4 band = srcH - even(round(srcH*top)) - even(round(srcH*bottom))");
}

void testG5Ugreen(const TestEnv&) {
    const LaneGeometry u = computeLaneGeometry(1920, 1080, cropCfg(0.5f, 0.2059f));
    check(u.cropTopPx == 540 && u.cropBottomPx == 222 && u.branchH == 318, "G5 UGREEN crops 540 / 222, band 318");
    check(near(u.preYScale, 0.99375, 1e-6), "G5 UGREEN preYScale 0.99375");
    const LaneGeometry t = computeLaneGeometry(1456, 1088, cropCfg(0.5f, 0.0f));
    check(t.branchH == 544 && near(t.preYScale, 1.7, 1e-6), "G5 IMX296 top-only: band 544, preYScale 1.7");
    check(near(t.refRowSrc, t.impliedTopSrc + static_cast<double>(0.90f) * t.impliedH, 1e-9)
              && near(t.impliedH, 1600.0 / 3 * 1.7, 0.01),
          "G5 IMX296 top-only refRow " + f(t.refRowSrc) + " from the formula");
}

template <class Fn>
bool throws(Fn fn) {
    try { fn(); } catch (const std::runtime_error&) { return true; }
    return false;
}

void testG6Rejects(const TestEnv&) {
    check(throws([] { computeLaneGeometry(1456, 100, cropCfg(0.5f, 0.4f)); }), "G6 < 64 rows left throws");
    check(throws([] { computeLaneGeometry(1456, 1088, cropCfg(0.95f, 0.0f)); }), "G6 inputCropTop > 0.9 throws");
    check(throws([] { computeLaneGeometry(1456, 1088, cropCfg(0.0f, 0.91f)); }), "G6 inputCropBottom > 0.9 throws");
    check(throws([] { computeLaneGeometry(1456, 1088, cropCfg(-0.1f, 0.0f)); }), "G6 negative crop throws");
    check(throws([] { LaneDetectorConfig c; c.numLanes = 3; computeLaneGeometry(1456, 1088, c); }),
          "G6 numLanes != 4 throws");
    check(throws([] { LaneDetectorConfig c; c.cropRatio = 0.0f; computeLaneGeometry(1456, 1088, c); }),
          "G6 cropRatio 0 throws");
    check(throws([] { computeLaneGeometry(0, 1088, LaneDetectorConfig{}); }), "G6 zero width throws");
}

void testG7EvenRows(const TestEnv&) {
    bool allEven = true;
    for (uint32_t h : { 1081u, 1088u, 721u, 1079u, 999u })
        for (float t : { 0.3f, 0.5f, 0.37f })
            for (float b : { 0.0f, 0.2059f, 0.13f }) {
                const LaneGeometry g = computeLaneGeometry(1456, h, cropCfg(t, b));
                allEven = allEven && g.cropTopPx % 2 == 0 && g.cropBottomPx % 2 == 0;
            }
    check(allEven, "G7 crops always round to even rows (NV12 chroma)");
    check(computeLaneGeometry(1456, 1081, cropCfg(0.5f, 0.0f)).cropTopPx == 540, "G7 1081*0.5=540.5 -> 541 -> 540");
}

void testG8RoundTrip(const TestEnv&) {
    double worst = 0;
    for (const auto& c : { cropCfg(0.5f, 0.2059f), cropCfg(0.0f, 0.0f), cropCfg(0.0f, 0.2f), cropCfg(0.5f, 0.0f) }) {
        const LaneGeometry g = computeLaneGeometry(1456, 1088, c);
        for (double fr : { 0.0, 0.42, 0.5, 0.9, 1.0 })
            worst = std::max(worst, std::fabs(g.sourceRowToImplied(g.impliedToSourceRow(fr)) - fr));
    }
    check(worst <= 1e-6, "G8 impliedToSourceRow o sourceRowToImplied round trip <= 1e-6 (worst " + std::to_string(worst) + ")");
}

// ─── hand-built UFLD v2 tensors ──────────────────────────────────────────────

constexpr float kW = 1600.0f;

struct Tensors {
    UfldDims d;
    std::vector<float> locRow, locCol, existRow, existCol;

    Tensors()
        : locRow(static_cast<size_t>(d.numCellRow) * d.numRowAnchors * d.numLanes, 0.0f),
          locCol(static_cast<size_t>(d.numCellCol) * d.numColAnchors * d.numLanes, 0.0f),
          existRow(2u * d.numRowAnchors * d.numLanes, 0.0f),
          existCol(2u * d.numColAnchors * d.numLanes, 0.0f) {
        // Everything absent: exist[0] = 1 > exist[1] = 0.
        for (int k = 0; k < d.numRowAnchors; ++k)
            for (int l = 0; l < d.numLanes; ++l) er(0, k, l) = 1.0f;
        for (int k = 0; k < d.numColAnchors; ++k)
            for (int l = 0; l < d.numLanes; ++l) ec(0, k, l) = 1.0f;
    }

    float& lr(int g, int k, int l) { return locRow[(static_cast<size_t>(g) * d.numRowAnchors + k) * d.numLanes + l]; }
    float& lc(int g, int k, int l) { return locCol[(static_cast<size_t>(g) * d.numColAnchors + k) * d.numLanes + l]; }
    float& er(int c, int k, int l) { return existRow[(static_cast<size_t>(c) * d.numRowAnchors + k) * d.numLanes + l]; }
    float& ec(int c, int k, int l) { return existCol[(static_cast<size_t>(c) * d.numColAnchors + k) * d.numLanes + l]; }

    void rowPoint(int lane, int k, int g) {
        er(0, k, lane) = 0.0f; er(1, k, lane) = 1.0f;
        for (int c = 0; c < d.numCellRow; ++c) lr(c, k, lane) = -10.0f;
        lr(g, k, lane) = 10.0f;
    }
    void colPoint(int lane, int k, int g) {
        ec(0, k, lane) = 0.0f; ec(1, k, lane) = 1.0f;
        for (int c = 0; c < d.numCellCol; ++c) lc(c, k, lane) = -10.0f;
        lc(g, k, lane) = 10.0f;
    }

    // Row-head lane at a constant cell (a vertical boundary) on anchors [k0, k1).
    void rowLine(int lane, int g, int k0 = 0, int k1 = -1) {
        if (k1 < 0) k1 = d.numRowAnchors;
        for (int k = k0; k < k1; ++k) rowPoint(lane, k, g);
    }
    // Column-head lane through (k, y(k)) for k in [k0, k1).
    template <class Y>
    void colLine(int lane, int k0, int k1, Y y) {
        for (int k = k0; k < k1; ++k) {
            const int g = static_cast<int>(std::lround(y(k) * (d.numCellCol - 1) - 0.5));
            colPoint(lane, k, std::max(0, std::min(d.numCellCol - 1, g)));
        }
    }

    UfldDecodeOut decode(bool points = false) const {
        UfldDecodeParams p;
        p.srcW = kW;
        return decodeUfldV2(locRow.data(), locCol.data(), existRow.data(), existCol.data(), d, p, points);
    }
};

float rowX(int g) { return (static_cast<float>(g) + 0.5f) / 199.0f * kW; }

// A clean road: ego boundaries (row head) symmetric about x = 800, outer
// boundaries (column head) further out, slanting like a perspective view.
Tensors fourLanes(bool slot0 = true, bool slot1 = true, bool slot2 = true, bool slot3 = true) {
    Tensors t;
    if (slot1) t.rowLine(1, 69);    // x 558.79
    if (slot2) t.rowLine(2, 129);   // x 1041.21; 558.79 + 1041.21 = 1600
    if (slot0) t.colLine(0, 0, 30, [](int k) { return 0.4 + 0.6 * (1.0 - k / 29.0); });
    if (slot3) t.colLine(3, 51, 81, [](int k) { return 0.4 + 0.6 * ((k - 51) / 29.0); });
    return t;
}

void testD1Symmetric(const TestEnv&) {
    const UfldDecodeOut d = fourLanes().decode();
    bool all = true;
    for (const auto& s : d.slots) all = all && s.detected;
    check(all, "D1 all four slots detected");
    check(d.slots[1].head == LaneHead::Row && d.slots[0].head == LaneHead::Col, "D1 slots 1,2 row head; 0,3 column head");
    check(near(d.slots[1].xRef, rowX(69), 0.05) && near(d.slots[2].xRef, rowX(129), 0.05),
          "D1 ego boundaries at 558.79 / 1041.21 (got " + f(d.slots[1].xRef) + " / " + f(d.slots[2].xRef) + ")");
    check(d.slots[0].xRef < d.slots[1].xRef && d.slots[3].xRef > d.slots[2].xRef, "D1 outer boundaries outside the ego lane");
    const LaneEgo e = egoFromSlots(d.slots, kW * 0.5f);
    check(e.numLanes == 3 && e.currentLaneIndex == 1, "D1 3 lanes, ego lane 1");
    check(e.lateralValid && near(e.lateralOffset, 0.0, 0.02), "D1 centred: offset 0 +- 0.02 (got " + f(e.lateralOffset, 4) + ")");
    const LaneEgo r = egoFromSlots(d.slots, kW * 0.5f + 100.0f);
    check(r.lateralValid && r.lateralOffset > 0.3f, "D1 vehicle 100 px right of centre -> offset > 0 (got " + f(r.lateralOffset, 3) + ")");
    const LaneEgo v2 = egoFromSlots(d.slots, kW * 0.5f, EgoLogic::kSlotIdentityV2);
    check(v2.numLanes == 3 && v2.currentLaneIndex == 1 && near(v2.lateralOffset, 0.0, 0.02),
          "D1 v2 agrees on a clean road");
}

// B5: the shipped logic sorts xRef and discards slot identity.
void testD2EgoLeftMissing(const TestEnv&) {
    const UfldDecodeOut d = fourLanes(true, false, true, true).decode();
    const LaneEgo v1 = egoFromSlots(d.slots, kW * 0.5f);
    bugCheck("B5", !v1.lateralValid,
             "D2 left ego boundary missed -> no lateral offset (v1 merges two lanes: ego " +
             std::to_string(v1.currentLaneIndex) + ", offset " + f(v1.lateralOffset) + ")");
    const LaneEgo v2 = egoFromSlots(d.slots, kW * 0.5f, EgoLogic::kSlotIdentityV2);
    check(!v2.lateralValid && v2.currentLaneIndex == -1, "D2 v2: ego unknown, no offset");
}

void testD3EgoRightMissing(const TestEnv&) {
    const UfldDecodeOut d = fourLanes(true, true, false, true).decode();
    const LaneEgo v1 = egoFromSlots(d.slots, kW * 0.5f);
    bugCheck("B5", !v1.lateralValid,
             "D3 right ego boundary missed -> no lateral offset (v1: ego " +
             std::to_string(v1.currentLaneIndex) + ", offset " + f(v1.lateralOffset) + ")");
    const LaneEgo v2 = egoFromSlots(d.slots, kW * 0.5f, EgoLogic::kSlotIdentityV2);
    check(!v2.lateralValid && v2.currentLaneIndex == -1, "D3 v2: ego unknown, no offset");
}

void testD4OuterCrossesEgo(const TestEnv&) {
    // Slot 0 (an outer boundary) comes out between the left ego boundary and
    // the vehicle centre at refY: the ego lane is still slots 1..2.
    Tensors t = fourLanes(false, true, true, true);
    t.colLine(0, 22, 45, [](int k) {
        const double x = k / 80.0 * kW;
        return std::min(0.994, 0.9 + (650.0 - x) * 0.0006);
    });
    const UfldDecodeOut d = t.decode();
    const bool fixture = d.slots[0].detected && d.slots[0].xRef > d.slots[1].xRef + 10 && d.slots[0].xRef < 790;
    check(fixture, "D4 fixture: slot 0 crosses inside the ego lane at refY (xRef " + f(d.slots[0].xRef) + ")");
    const LaneEgo v1 = egoFromSlots(d.slots, kW * 0.5f);
    bugCheck("B5", v1.lateralValid && near(v1.lateralOffset, 0.0, 0.02),
             "D4 offset still measured against slots 1..2 (v1 offset " + f(v1.lateralOffset) + ")");
    const LaneEgo v2 = egoFromSlots(d.slots, kW * 0.5f, EgoLogic::kSlotIdentityV2);
    check(v2.lateralValid && near(v2.lateralOffset, 0.0, 0.02) && v2.numLanes == 2,
          "D4 v2: offset 0 against slots 1..2; crossed pair 0-1 not a lane");
}

void testD5Thresholds(const TestEnv&) {
    Tensors a; a.rowLine(1, 69, 0, 36);
    Tensors b; b.rowLine(1, 69, 0, 37);
    check(!a.decode().slots[1].detected && b.decode().slots[1].detected,
          "D5 row slot: 36 of 72 anchors absent, 37 present (> K/2)");
    Tensors c; c.colLine(0, 0, 20, [](int k) { return 0.4 + 0.02 * k; });
    Tensors e; e.colLine(0, 0, 21, [](int k) { return 0.4 + 0.02 * k; });
    check(!c.decode().slots[0].detected && e.decode().slots[0].detected,
          "D5 column slot: 20 of 81 anchors absent, 21 present (> K/4)");
    Tensors tie;
    tie.rowLine(1, 69);
    for (int k = 0; k < tie.d.numRowAnchors; ++k) tie.er(0, k, 1) = tie.er(1, k, 1) = 0.5f;
    check(!tie.decode().slots[1].detected, "D5 an existence tie counts as absent");
}

void testD6SoftArgmaxEdges(const TestEnv&) {
    Tensors t;
    t.rowLine(1, 0);
    t.rowLine(2, 198);
    for (int k = 0; k < t.d.numRowAnchors; ++k) {
        t.lr(1, k, 1) = 10.0f;     // cells 0 and 1 tie: window clamped to [0, 1]
        t.lr(199, k, 2) = 10.0f;   // cells 198 and 199 tie: window [197, 199]
    }
    const UfldDecodeOut d = t.decode(true);
    check(!d.points[1].empty() && near(d.points[1][0].x, 1.0 / 199.0 * kW, 0.01),
          "D6 low edge: (0+1)/2 + 0.5 = cell 1.0 -> x " + f(d.points[1].empty() ? -1 : d.points[1][0].x));
    check(!d.points[2].empty() && near(d.points[2][0].x, kW, 0.01),
          "D6 high edge: (198+199)/2 + 0.5 = cell 199 -> x = W (" + f(d.points[2].empty() ? -1 : d.points[2][0].x) + ")");
}

// B9: NaN / Inf in the outputs.
void testD7NanLocation(const TestEnv&) {
    Tensors t = fourLanes();
    for (int k = 0; k < 10; ++k)
        for (int g = 0; g < t.d.numCellRow; ++g) t.lr(g, k, 1) = std::numeric_limits<float>::quiet_NaN();
    const UfldDecodeOut d = t.decode();
    check(d.nonFinite == 10 * t.d.numCellRow, "D7 nonFinite counts the NaN location logits (" + std::to_string(d.nonFinite) + ")");
    bugCheck("B9", std::isfinite(d.slots[1].xRef) && near(d.slots[1].xRef, rowX(69), 1.0),
             "D7 NaN location logits are skipped: slot 1 xRef finite and at 558.8 (got " + f(d.slots[1].xRef) + ")");
    const LaneEgo e = egoFromSlots(d.slots, kW * 0.5f);
    // Holds today in this fixture (the NaN xRef lands outside the ego gap);
    // a plain regression check, not a B9 symptom.
    check(!e.lateralValid || std::isfinite(e.lateralOffset), "D7 lateral offset never NaN");
}

void testD8NanExistence(const TestEnv&) {
    // 37 anchors present (just enough), the other 35 have NaN existence
    // logits and location pointing at the left edge.
    Tensors t;
    t.rowLine(1, 69, 0, 37);
    for (int k = 37; k < t.d.numRowAnchors; ++k) {
        for (int g = 0; g < t.d.numCellRow; ++g) t.lr(g, k, 1) = -10.0f;
        t.lr(0, k, 1) = 10.0f;
        t.er(0, k, 1) = t.er(1, k, 1) = std::numeric_limits<float>::quiet_NaN();
    }
    const UfldDecodeOut d = t.decode();
    bugCheck("B9", d.slots[1].detected && near(d.slots[1].xRef, rowX(69), 1.0),
             "D8 NaN existence = absent in the fit too: xRef 558.8 (got " + f(d.slots[1].xRef) +
             " from " + std::to_string(d.slots[1].nPts) + " points)");
}

// B8: the float least-squares fit.
void testD9ConstantY(const TestEnv&) {
    Tensors t;
    t.colLine(0, 0, 30, [](int) { return 0.505; });     // one cell: varY = 0
    const UfldDecodeOut d = t.decode();
    check(!d.slots[0].detected || (std::isfinite(d.slots[0].xRef) && std::fabs(d.slots[0].xRef) < 3 * kW),
          "D9 horizontal column-head line: rejected or finite, |x| < 3W");
}

void testD9bTinyYSpread(const TestEnv&) {
    Tensors t;
    t.colLine(0, 0, 30, [](int k) { return k < 15 ? 0.505 : 0.515; });   // y spread one cell (~0.01)
    const UfldDecodeOut d = t.decode();
    bugCheck("B8", !d.slots[0].detected || (std::isfinite(d.slots[0].xRef) && std::fabs(d.slots[0].xRef) < 3 * kW),
             "D9b near-horizontal column-head line: rejected or |xRef| < 3W (got " + f(d.slots[0].xRef, 0) + ")");
}

void testD10Curve(const TestEnv&) {
    // Report only: what a straight fit costs on a curving boundary.
    Tensors t;
    auto xTrue = [](double y) { return 500.0 + 900.0 * (y - 0.42) * (y - 0.42); };
    for (int k = 0; k < t.d.numRowAnchors; ++k) {
        const double y = 0.42 + 0.58 * k / 71.0;
        const int g = static_cast<int>(std::lround(xTrue(y) / kW * 199.0 - 0.5));
        t.rowPoint(1, k, g);
    }
    const UfldDecodeOut d = t.decode();
    note("D10 curved boundary: straight fit at refY " + f(d.slots[1].xRef, 1) + " px vs true " +
         f(xTrue(0.9), 1) + " px (error " + f(d.slots[1].xRef - xTrue(0.9), 1) + " px, rms " + f(d.slots[1].rmsPx, 2) + ")");
    check(d.slots[1].detected, "D10 curved boundary still decodes");
}

// B6: vehicle centre.
template <class C, class = void>
struct HasVehicleCenter : std::false_type {};
template <class C>
struct HasVehicleCenter<C, std::void_t<decltype(std::declval<C>().vehicleCenterX)>> : std::true_type {};

void testD11VehicleCentre(const TestEnv&) {
    const UfldDecodeOut d = fourLanes().decode();
    const LaneEgo e = egoFromSlots(d.slots, kW * 0.5f - 120.0f);
    check(e.lateralValid && e.lateralOffset < -0.3f, "D11 egoFromSlots honours a vehicle centre left of W/2");
    bugCheck("B6", HasVehicleCenter<LaneDetectorConfig>::value,
             "D11 LaneDetectorConfig exposes vehicleCenterX (the detector hard-codes srcW/2)");
}

void testD12Rms(const TestEnv&) {
    Tensors a; a.rowLine(1, 69);
    // Float moments leave a ~0.002 px residual on a perfect line (immaterial).
    check(a.decode().slots[1].rmsPx < 0.01f, "D12 perfect line: rms < 0.01 px (" + f(a.decode().slots[1].rmsPx, 5) + ")");
    Tensors b;
    for (int k = 0; k < b.d.numRowAnchors; ++k) b.rowPoint(1, k, k % 2 ? 71 : 69);
    const float cell = kW / 199.0f;          // points alternate +-1 cell about the line
    const float rms = b.decode().slots[1].rmsPx;
    check(near(rms, cell, 0.1 * cell), "D12 +-1 cell scatter: rms ~ one cell " + f(cell, 2) + " px (got " + f(rms, 2) + ")");
}

// ─── harness ─────────────────────────────────────────────────────────────────

// B17: lane_test's live smoke must not pass on frozen results.
void testS1LiveSmokeCriterion(const TestEnv&) {
    using lanetest::LivePoll;
    std::vector<LivePoll> healthy, frozen, warmup;
    for (int t = 1; t <= 10; ++t) {
        healthy.push_back({ t > 1, static_cast<uint64_t>(t > 1 ? 13 * t : 0), t > 1 });   // ~13 results/s
        frozen.push_back({ t > 1, static_cast<uint64_t>(t > 1 ? 7 : 0), false });         // stuck after one frame
        warmup.push_back({ t > 5, static_cast<uint64_t>(t > 5 ? 13 * t : 0), t > 5 });    // 5 s warm-up
    }
    check(lanetest::liveSmokeVerdict(healthy, true, 130), "S1 a live, advancing sequence passes");
    check(!lanetest::liveSmokeVerdict(healthy, false, 130), "S1 a dirty teardown fails");
    check(!lanetest::liveSmokeVerdict(warmup, true, 65), "S1 only 5 of 10 polls valid fails");
    bugCheck("B17", !lanetest::liveSmokeVerdict(frozen, true, 7),
             "S1 results frozen after the first frame (sequence never advances) fail the smoke test");
    std::vector<LivePoll> staleButCounting = healthy;
    for (auto& p : staleButCounting) p.fresh = false;
    check(!lanetest::liveSmokeVerdict(staleButCounting, true, 130),
          "S1 advancing but never fresh (results older than 500 ms at every poll) fails");
}

void testU6OutputGuard(const TestEnv&) {
    namespace fs = std::filesystem;
    using lanetest::insideRepo;
    const std::string repo = "/home/jetson/Documents/github_repos/dashcam";
    check(insideRepo("/user/dashcam/out") && insideRepo(repo + "/zzz/new/deeper"),
          "U6 paths in the repo (existing or not) are refused");
    check(!insideRepo("/tmp/lane_out") && !insideRepo("/media/jetson/backup/lane_tests/run_001"),
          "U6 paths outside are allowed");
    check(!insideRepo(repo + "_other/x") && !insideRepo("/user/dashcam2/x"),
          "U6 prefix match is by path component, not by string");
    // Relative paths resolve against the cwd, even when nothing exists yet.
    std::error_code ec;
    const fs::path cwd = fs::current_path(ec);
    for (const std::string& root : { std::string("/user/dashcam"), repo }) {
        if (!fs::exists(root, ec)) continue;
        fs::current_path(root, ec);
        check(insideRepo("not_there_yet/x") && insideRepo("./y"), "U6 relative path from " + root + " is refused");
        fs::current_path("/tmp", ec);
        check(!insideRepo("not_there_yet/x"), "U6 relative path from /tmp is allowed");
    }
    fs::current_path(cwd, ec);
}

// ─── lane_drive aux (clip ring, budget, planner, mount monitor) ──────────────

using namespace lanedrive;
constexpr int64_t kS = 1'000'000'000LL;

void testA1JpegRing(const TestEnv&) {
    JpegRing ring(1000);
    std::vector<uint8_t> b(300, 7);
    for (int i = 0; i < 5; ++i) ring.push(100 + i * 10, i * kS, b.data(), b.size());
    check(ring.bytes() == 900 && ring.pushed() == 5, "A1 cap 1000 B: 3 of 5 frames of 300 B kept (" +
          std::to_string(ring.bytes()) + " B)");
    JFrame f;
    int64_t d = 0;
    check(ring.find(140, f, d, false) && d == 0 && f.mono == 4 * kS, "A1 exact PTS found");
    check(!ring.find(110, f, d, false), "A1 an evicted PTS is not found exactly");
    check(ring.find(133, f, d, true) && f.pts == 130 && d == -3, "A1 nearest PTS with its dpts");
    check(ring.rangeMono(2 * kS, 4 * kS).size() == 2, "A1 range (2 s, 4 s] holds 2 frames");
    check(ring.newestMono() == 4 * kS && std::fabs(ring.spanS() - 2.0) < 1e-9, "A1 newest mono and span");
    ring.setEnabled(false);
    ring.push(200, 5 * kS, b.data(), b.size());
    check(ring.bytes() == 0 && ring.pushed() == 5, "A1 disabled: cleared and nothing pushed");
}

void testA2ClipBudget(const TestEnv&) {
    ClipBudget b(3.6, 100, 1.0);          // 1 MB/s, capacity 100 MB, full
    check(b.admit(kRest, 20e6, 0), "A2 rest admitted while the bucket is full");
    check(!b.admit(kRest, 20e6, 0), "A2 rest refused below its 75 % reserve");
    check(b.admit(kStopGo, 20e6, 0), "A2 stop->go still admitted (50 % reserve)");
    check(b.admit(kLaneChange, 20e6, 0) && !b.admit(kLaneChange, 20e6, 0), "A2 LC admitted down to its 25 % reserve");
    check(b.admit(kFault, 20e6, 0) && b.admit(kFault, 20e6, 0) && !b.admit(kFault, 20e6, 0),
          "A2 faults take the last tokens (no reserve), then are refused");
    check(std::fabs(b.tokens(10) - 10e6) < 1, "A2 refill 1 MB/s");
    b.adjust(-5e6);
    check(std::fabs(b.tokens(10) - 15e6) < 1, "A2 settling a smaller-than-estimated clip returns tokens");
}

void testA3ClipPlanner(const TestEnv&) {
    ClipPlanner::Params p;
    p.pendingCap = 1000;
    ClipPlanner pl(p);
    ClipBudget big(1000, 600, 1.0);
    JpegRing ring(1u << 20);
    std::vector<uint8_t> b(100, 1);
    const int id = pl.trigger("stall", kFault, 100 * kS, 1e3, big);
    check(id == 1 && pl.openJobs() == 1, "A3 first trigger opens job 1");
    check(pl.trigger("mark", kRest, 105 * kS, 1e3, big) == 1, "A3 a trigger inside the window joins it");
    check(pl.trigger("late", kRest, 140 * kS, 1e3, big) == 2, "A3 a trigger after the window opens job 2");
    check(pl.trigger("x", kRest, 0, 1e3, big, false) == -1, "A3 disabled: -1");
    for (int t = 85; t <= 125; ++t) ring.push(t, t * kS, b.data(), b.size());
    pl.collect(ring);
    auto done = pl.finished(ring.newestMono(), 125 * kS);
    check(done.size() == 1 && done[0].id == 1, "A3 job 1 finished once a frame past 115 s arrived");
    if (!done.empty()) {
        check(done[0].frames.size() == 10 && done[0].overflow, "A3 job 1 window 90..115 s: the 1000 B pending cap keeps "
              "the first 10 frames and flags the overflow (" + std::to_string(done[0].frames.size()) + " collected)");
        check(done[0].triggers == "stall;mark" && done[0].priority == kFault, "A3 merged triggers and priority kept");
    }
    check(pl.overflowFrames() > 0, "A3 frames beyond the pending cap are counted (" +
          std::to_string(pl.overflowFrames()) + ")");
    auto late = pl.finished(ring.newestMono(), 156 * kS);
    check(late.size() == 1 && late[0].id == 2, "A3 job 2 times out 5 s past its window with no frames");
    ClipBudget tiny(0.0036, 1, 1.0);       // 1 kB/s, capacity 1 kB
    check(pl.trigger("rest", kRest, 300 * kS, 1e6, tiny) == 0 && pl.refused() == 1, "A3 the budget refuses");
}

// A smooth textured scene (Gaussian blobs + two hard edges), rendered with a
// sub-pixel shift and a rotation about the centre.
Gray scene(int w, int h, double dx, double dy, double rotDeg, unsigned seed, double noise) {
    std::mt19937 rng(1234);
    std::uniform_real_distribution<double> U(0, 1);
    struct Blob { double x, y, s, a; };
    std::vector<Blob> blobs;
    for (int i = 0; i < 160; ++i) blobs.push_back({ U(rng) * w, U(rng) * h, 1.5 + 4 * U(rng), (U(rng) - 0.5) * 140 });
    std::mt19937 nrng(seed);
    std::normal_distribution<double> N(0, noise > 0 ? noise : 1);
    const double c = std::cos(-rotDeg * M_PI / 180), s = std::sin(-rotDeg * M_PI / 180);
    const double cx = w / 2.0, cy = h / 2.0;
    Gray g;
    g.w = w; g.h = h; g.px.resize(static_cast<size_t>(w) * h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            const double ux = x - dx - cx, uy = y - dy - cy;
            const double X = c * ux - s * uy + cx, Y = s * ux + c * uy + cy;
            double v = 110 + (Y > 0.8 * h ? 40 : 0) + (X > 0.3 * w && Y > 0.9 * h ? -30 : 0);
            for (const auto& b : blobs) {
                const double ex = X - b.x, ey = Y - b.y;
                if (std::fabs(ex) < 4 * b.s && std::fabs(ey) < 4 * b.s) v += b.a * std::exp(-(ex * ex + ey * ey) / (2 * b.s * b.s));
            }
            if (noise > 0) v += N(nrng);
            g.px[static_cast<size_t>(y) * w + x] = static_cast<uint8_t>(std::max(0.0, std::min(255.0, v)));
        }
    return g;
}

void testA4MountMonitor(const TestEnv&) {
    const int W = 364, H = 272;
    const float scale = 4.0f;            // CSI 1456 / 364
    MountRoi roi;
    roi.y0 = 0.55f;
    MountMonitor m;
    check(m.setReference(scene(W, H, 0, 0, 0, 1, 2), roi, scale), "A4 reference accepted");
    const MountSample same = m.measure(scene(W, H, 0, 0, 0, 2, 2));
    check(same.ok && std::fabs(same.dx) < 0.5 && std::fabs(same.dy) < 0.5 && std::fabs(same.rollDeg) < 0.15 &&
          same.conf > 0.8, "A4 same view (new noise): dx " + f(same.dx, 2) + " dy " + f(same.dy, 2) + " src px, roll " +
          f(same.rollDeg, 2) + " deg, conf " + f(same.conf, 2));
    const MountSample sh = m.measure(scene(W, H, 1.5, -2.25, 0, 3, 2));
    check(sh.ok && std::fabs(sh.dx - 6.0) < 1.2 && std::fabs(sh.dy + 9.0) < 1.2,
          "A4 shift (+1.5, -2.25) tap px = (+6, -9) src px measured (" + f(sh.dx, 2) + ", " + f(sh.dy, 2) + ")");
    const MountSample ro = m.measure(scene(W, H, 0, 0, 1.0, 4, 2));
    check(ro.ok && std::fabs(ro.rollDeg - 1.0) < 0.3, "A4 roll 1.0 deg measured " + f(ro.rollDeg, 2) + " deg");
    const MountSample far = m.measure(scene(W, H, 40, 30, 0, 6, 2));
    check(!far.ok && far.lost && far.featureRatio > 0.7f, "A4 view moved beyond the search (40, 30) tap px: lost "
          "(conf " + f(far.conf, 2) + ", features " + f(far.featureRatio, 2) + " of the reference)");
    Gray flat;
    flat.w = W; flat.h = H; flat.px.assign(static_cast<size_t>(W) * H, 30);
    MountMonitor dark;
    dark.setReference(flat, roi, scale);
    check(!dark.measure(flat).ok, "A4 featureless (dark) reference: no measurement");
    Gray darkFrame = flat;
    const MountSample dk = m.measure(darkFrame);
    check(!dk.ok && !dk.lost, "A4 a dark frame against a good reference: neither measured nor lost");

    // Verdict: 5 s median over the threshold, sustained 5 s → shifted (latched).
    MountMonitor v;
    MountSample s8;
    s8.ok = true; s8.dy = 8; s8.conf = 0.9f;
    MountSample s0 = s8;
    s0.dy = 0.5f;
    int onAt = -1, offAt = -1;
    for (int t = 0; t < 30; ++t) {
        const auto r = v.feed(t * kS, t < 15 ? s8 : s0);
        if (r.onEdge) onAt = t;
        if (r.offEdge) offAt = t;
    }
    // The median exceeds from the first sample (t = 0); sustained 5 s → t = 5.
    // Back: the median drops at t = 17 (3 of 5 small), clear 5 s → t = 22.
    check(onAt == 5, "A4 FAIL edge 5 s after the 5 s median first exceeds 6 px (t = " + std::to_string(onAt) + ")");
    check(offAt == 22 && v.feed(31 * kS, s0).shifted, "A4 view back: off edge at t = " + std::to_string(offAt) +
          ", shifted stays latched");
    // Lost view: FAIL 5 s after the first lost sample; dark samples hold the state.
    MountMonitor lv;
    MountSample good = s0, lost, darkS;
    lost.lost = true;
    int lostOn = -1;
    for (int t = 0; t < 20; ++t) {
        const auto r = lv.feed(t * kS, t < 5 ? good : (t == 7 || t == 8) ? darkS : lost);
        if (r.onEdge) lostOn = t;
    }
    check(lostOn == 10, "A4 lost view from t = 5 (dark samples at 7, 8 neither reset nor count): FAIL at t = " +
          std::to_string(lostOn));
    MountMonitor q;
    std::mt19937 rng(7);
    std::normal_distribution<double> N(0, 2.0);
    bool any = false;
    for (int t = 0; t < 600; ++t) {
        MountSample n = s0;
        n.dx = static_cast<float>(N(rng)); n.dy = static_cast<float>(N(rng)); n.rollDeg = static_cast<float>(N(rng) * 0.08);
        any = any || q.feed(t * kS, n).shifted;
    }
    check(!any, "A4 10 min of 2 px / 0.16 deg sigma jitter: no FAIL");
}

void testA5TensorRing(const TestEnv&) {
    TensorRing r(3);
    float a[2] = { 1, 2 }, b[1] = { 3 }, c[1] = { 4 }, d[1] = { 5 };
    const float* outs[4] = { a, b, c, d };
    const size_t n[4] = { 2, 1, 1, 1 };
    for (uint64_t s = 1; s <= 5; ++s) { a[0] = static_cast<float>(s); r.put(s, static_cast<int64_t>(s * 10), -1, outs, n); }
    const auto snap = r.snapshot();
    check(snap.size() == 3 && snap[0].seq == 3 && snap[2].seq == 5 && snap[2].data.size() == 5 &&
          snap[2].data[0] == 5.0f && snap[2].data[4] == 5.0f && snap[0].counts[0] == 2,
          "A5 ring of 3 keeps seq 3..5 in order, outputs back to back");
}

} // namespace

namespace lanetest {

std::vector<TestCase> unitTests() {
    return {
        { "G1 imx296 default crops",   5, testG1Imx296Defaults },
        { "G2 no crop (training)",     5, testG2NoCrop },
        { "G3 bottom crop only",       5, testG3BottomOnly },
        { "G4 1280x720 defaults",      5, testG4Usb720 },
        { "G5 ugreen + top-only",      5, testG5Ugreen },
        { "G6 invalid configs",        5, testG6Rejects },
        { "G7 even crop rows",         5, testG7EvenRows },
        { "G8 implied round trip",     5, testG8RoundTrip },
        { "D1 symmetric four lanes",   5, testD1Symmetric },
        { "D2 left ego missing",       5, testD2EgoLeftMissing },
        { "D3 right ego missing",      5, testD3EgoRightMissing },
        { "D4 outer crosses ego",      5, testD4OuterCrossesEgo },
        { "D5 existence thresholds",   5, testD5Thresholds },
        { "D6 soft argmax edges",      5, testD6SoftArgmaxEdges },
        { "D7 NaN location",           5, testD7NanLocation },
        { "D8 NaN existence",          5, testD8NanExistence },
        { "D9 constant-y line",        5, testD9ConstantY },
        { "D9b tiny y spread",         5, testD9bTinyYSpread },
        { "D10 curved boundary",       5, testD10Curve },
        { "D11 vehicle centre",        5, testD11VehicleCentre },
        { "D12 fit rms",               5, testD12Rms },
        { "U6 output-path guard",      5, testU6OutputGuard },
        { "S1 live smoke criterion",   5, testS1LiveSmokeCriterion },
        { "A1 clip ring",              5, testA1JpegRing },
        { "A2 clip budget",            5, testA2ClipBudget },
        { "A3 clip planner",           5, testA3ClipPlanner },
        { "A4 mount monitor",         20, testA4MountMonitor },
        { "A5 tensor ring",            5, testA5TensorRing },
    };
}

} // namespace lanetest
