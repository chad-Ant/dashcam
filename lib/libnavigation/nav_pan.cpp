// nav_pan.cpp — board-side panning helpers of libnavigation (pure): the pose
// interpolator between GPS fixes and the background planner.  See libnavigation.h.

#include "libnavigation.h"

#include <algorithm>
#include <cmath>

namespace dashcam::navigation {

namespace {
constexpr double   kMinIntervalUs  = 100000.0;    ///< Fix interval clamp (10 Hz) ...
constexpr double   kMaxIntervalUs  = 1000000.0;   ///< ... to 1 Hz.
constexpr uint64_t kJitterMarginUs = 50000;       ///< Extra lag so the next fix is usually in.
constexpr uint64_t kRestartGapUs   = 2000000;     ///< No fix for this long: start afresh.
constexpr double   kRestartJumpPx  = 2000.0;      ///< A jump this large: start afresh.
constexpr double   kSameFixPx      = 1e-3;        ///< The bridge repeating the same fix.

/// Shortest-path angle interpolation in degrees, result in [0, 360).
float lerpCourse(float a, float b, double f) {
    double d = std::fmod(double(b) - double(a), 360.0);
    if (d > 180.0) d -= 360.0;
    if (d < -180.0) d += 360.0;
    double c = std::fmod(double(a) + d * f, 360.0);
    if (c < 0) c += 360.0;
    return float(c);
}
} // namespace

// ─── PoseInterpolator ─────────────────────────────────────────────────────────

void PoseInterpolator::reset() {
    n_ = head_ = 0;
    intervalUs_    = 250000.0;
    lastArrivalUs_ = 0;
}

void PoseInterpolator::offer(const WorldPoint& p, float courseDeg, uint64_t arrivalUs) {
    if (n_ > 0) {
        const Fix&     last = fromNewest(0);
        const uint64_t gap  = arrivalUs > lastArrivalUs_ ? arrivalUs - lastArrivalUs_ : 0;
        const double   jump = std::hypot(p.x - last.p.x, p.y - last.p.y);
        // The same fix again (10 Hz telemetry, 4 Hz GNSS): not a new point,
        // unless it has stayed put for longer than a fix interval (stationary:
        // kept as a fix so velocity drops to 0 and no 2 s restart happens).
        const bool repeat = jump < kSameFixPx;
        if (repeat && double(gap) < 1.5 * intervalUs_) return;
        if (arrivalUs < lastArrivalUs_ || gap > kRestartGapUs || jump > kRestartJumpPx) {
            reset();
        } else if (!repeat) {   // a repeat's gap says nothing about the receiver's fix rate
            intervalUs_ += 0.2 * (std::clamp(double(gap), kMinIntervalUs / 2, 1.5 * kMaxIntervalUs) - intervalUs_);
            intervalUs_ = std::clamp(intervalUs_, kMinIntervalUs, kMaxIntervalUs);
        }
    }
    Fix f;
    f.p      = p;
    f.course = courseDeg;
    if (n_ == 0) {
        f.t = arrivalUs;
    } else {
        // Arrival times come in 100 ms steps: pull each fix time toward the
        // predicted one, but never past its arrival or back onto the previous fix.
        const Fix&   last      = fromNewest(0);
        const double predicted = double(last.t) + intervalUs_;
        const double err       = double(arrivalUs) - predicted;
        double       t         = std::fabs(err) > intervalUs_ ? double(arrivalUs) : predicted + 0.3 * err;
        t   = std::clamp(t, double(last.t) + 0.25 * intervalUs_, double(arrivalUs));
        f.t = t > double(last.t) ? uint64_t(t) : last.t + 1;
    }
    ring_[head_]   = f;
    head_          = (head_ + 1) % kRing;
    n_             = std::min(n_ + 1, kRing);
    lastArrivalUs_ = arrivalUs;
}

uint32_t PoseInterpolator::delayUs() const { return uint32_t(intervalUs_) + uint32_t(kJitterMarginUs); }

WorldPoint PoseInterpolator::newest() const { return n_ ? fromNewest(0).p : WorldPoint{}; }

WorldPoint PoseInterpolator::velocity() const {
    if (n_ < 2) return WorldPoint{};
    const Fix& b  = fromNewest(0);
    const Fix& a  = fromNewest(1);
    const double dt = double(b.t - a.t) / 1e6;
    if (dt <= 0) return WorldPoint{};
    return WorldPoint{(b.p.x - a.p.x) / dt, (b.p.y - a.p.y) / dt};
}

WorldPoint PoseInterpolator::at(uint64_t nowUs, float* courseDeg) const {
    if (courseDeg) *courseDeg = NAN;
    if (n_ == 0) return WorldPoint{};
    const uint64_t delay = delayUs();
    const uint64_t t     = nowUs > delay ? nowUs - delay : 0;
    const Fix&     newest = fromNewest(0);
    if (t >= newest.t || n_ == 1) {   // past the newest fix: hold it (no extrapolation)
        if (courseDeg) *courseDeg = newest.course;
        return newest.p;
    }
    for (size_t k = 1; k < n_; ++k) {
        const Fix& a = fromNewest(k);
        const Fix& b = fromNewest(k - 1);
        if (t < a.t) continue;
        const double f = double(t - a.t) / double(b.t - a.t);
        if (courseDeg) {
            const bool ca = std::isfinite(a.course), cb = std::isfinite(b.course);
            *courseDeg = ca && cb ? lerpCourse(a.course, b.course, f) : b.course;
        }
        return WorldPoint{a.p.x + (b.p.x - a.p.x) * f, a.p.y + (b.p.y - a.p.y) * f};
    }
    const Fix& oldest = fromNewest(n_ - 1);   // before the oldest kept fix
    if (courseDeg) *courseDeg = oldest.course;
    return oldest.p;
}

// ─── background planner ───────────────────────────────────────────────────────

bool panBackgroundStale(double ox, double oy, int bgW, int bgH, int mapW, int mapH, const WorldPoint& pos,
                        const WorldPoint& vel, double lookS, int edgePx) {
    auto outside = [&](double x, double y) {
        return x - mapW / 2.0 < ox + edgePx || x + mapW / 2.0 > ox + bgW - edgePx ||
               y - mapH / 2.0 < oy + edgePx || y + mapH / 2.0 > oy + bgH - edgePx;
    };
    return outside(pos.x, pos.y) || outside(pos.x + vel.x * lookS, pos.y + vel.y * lookS);
}

void panBackgroundOrigin(const WorldPoint& pos, const WorldPoint& vel, double leadS, int bgW, int bgH, int mapW,
                         int mapH, int64_t& ox, int64_t& oy, int edgePx) {
    // Per axis: margin m on each side; the lead may use 60% of it but must leave
    // more than edgePx behind the vehicle, or the background is stale at once.
    auto leadCap = [edgePx](int bg, int map) {
        const double m = std::max(0, bg - map) / 2.0;
        return std::max(0.0, std::min(0.6 * m, m - edgePx - 1));
    };
    const double mx = leadCap(bgW, mapW);
    const double my = leadCap(bgH, mapH);
    const double cx = pos.x + std::clamp(vel.x * leadS, -mx, mx);
    const double cy = pos.y + std::clamp(vel.y * leadS, -my, my);
    ox = int64_t(std::llround(cx - bgW / 2.0));
    oy = int64_t(std::llround(cy - bgH / 2.0));
}

} // namespace dashcam::navigation
