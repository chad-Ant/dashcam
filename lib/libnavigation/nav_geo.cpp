// nav_geo.cpp — Web Mercator, tile selection and the GPS freshness state
// machine of libnavigation (all pure; see libnavigation.h).

#include "libnavigation.h"

#include <algorithm>
#include <cmath>

namespace dashcam::navigation {

namespace {
constexpr double kPi           = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6378137.0;   // WGS-84 equatorial radius (Web Mercator sphere)
} // namespace

double worldSize(int zoom) {
    return double(kTileSize) * std::ldexp(1.0, std::clamp(zoom, 0, 30));
}

WorldPoint project(double latDeg, double lonDeg, int zoom) {
    const double size = worldSize(zoom);
    double lon = std::fmod(lonDeg + 180.0, 360.0);
    if (lon < 0) lon += 360.0;                         // [0, 360)
    const double lat = std::clamp(latDeg, -kMaxLatitude, kMaxLatitude);
    const double phi = lat * kPi / 180.0;
    WorldPoint p;
    p.x = lon / 360.0 * size;
    p.y = (1.0 - std::asinh(std::tan(phi)) / kPi) / 2.0 * size;
    p.y = std::clamp(p.y, 0.0, size);
    if (p.x >= size) p.x -= size;
    return p;
}

double wrapDx(double a, double b, int zoom) {
    const double size = worldSize(zoom);
    double d = std::fmod(b - a, size);
    if (d > size / 2) d -= size;
    if (d < -size / 2) d += size;
    return d;
}

double metersPerPixel(double latDeg, int zoom) {
    const double lat = std::clamp(latDeg, -kMaxLatitude, kMaxLatitude);
    return 2.0 * kPi * kEarthRadiusM * std::cos(lat * kPi / 180.0) / worldSize(zoom);
}

std::vector<TilePlacement> tilesForView(const WorldPoint& centre, int zoom, int w, int h) {
    std::vector<TilePlacement> out;
    if (w <= 0 || h <= 0) return out;
    const double size  = worldSize(zoom);
    const int64_t tiles = int64_t(size) / kTileSize;
    // View origin in world pixels (may be negative or past the world width in x).
    const double ox = centre.x - w / 2.0;
    const double oy = centre.y - h / 2.0;
    const int64_t tx0 = int64_t(std::floor(ox / kTileSize));
    const int64_t tx1 = int64_t(std::floor((ox + w - 1) / kTileSize));
    const int64_t ty0 = int64_t(std::floor(oy / kTileSize));
    const int64_t ty1 = int64_t(std::floor((oy + h - 1) / kTileSize));
    for (int64_t ty = ty0; ty <= ty1; ++ty) {
        if (ty < 0 || ty >= tiles) continue;           // above/below the world: no tile
        for (int64_t tx = tx0; tx <= tx1; ++tx) {
            TilePlacement p;
            p.key.z = zoom;
            p.key.x = uint32_t(((tx % tiles) + tiles) % tiles);
            p.key.y = uint32_t(ty);
            p.dstX  = int(std::lround(double(tx * kTileSize) - ox));
            p.dstY  = int(std::lround(double(ty * kTileSize) - oy));
            out.push_back(p);
        }
    }
    return out;
}

// ─── GPS ──────────────────────────────────────────────────────────────────────

const char* positionStateName(PositionState s) {
    switch (s) {
    case PositionState::NoData:     return "no data";
    case PositionState::NoReceiver: return "no receiver";
    case PositionState::Acquiring:  return "acquiring";
    case PositionState::Live:       return "live";
    case PositionState::LastKnown:  return "last known";
    }
    return "?";
}

bool usablePosition(const GpsSample& s) {
    const bool typeOk = s.fixType == 2 || s.fixType == 3 || s.fixType == 4;
    return s.gpsPresent && s.gpsFix && s.fixValid && typeOk && std::isfinite(s.latitude) &&
           std::isfinite(s.longitude) && s.latitude >= -90.0 && s.latitude <= 90.0 && s.longitude >= -180.0 &&
           s.longitude <= 180.0;
}

bool usableCourse(const GpsSample& s, float minSpeedKmh) {
    return usablePosition(s) && std::isfinite(s.courseDeg) && s.courseDeg >= 0.0f && s.courseDeg <= 360.0f &&
           std::isfinite(s.speedKmh) && s.speedKmh >= minSpeedKmh;
}

void NavState::offer(const GpsSample& s) {
    last_       = s;
    haveSample_ = true;
    linkLost_   = false;
    if (usablePosition(s)) {
        lastFix_ = s;
        haveFix_ = true;
    }
}

void NavState::linkLost() { linkLost_ = true; }

NavView NavState::view(uint64_t nowUs) const {
    NavView v;
    if (!haveSample_) return v;   // NoData
    const bool fresh = !linkLost_ && nowUs >= last_.receivedUs &&
                       nowUs - last_.receivedUs < uint64_t(cfg_.staleMs) * 1000;
    const bool liveFix = fresh && usablePosition(last_);
    // Speed and fix details only from a fresh sample: an old value is not shown.
    v.speedKmh   = fresh ? last_.speedKmh : NAN;
    v.fixType    = fresh ? last_.fixType : 0;
    v.satellites = fresh ? last_.satellites : 0;
    if (liveFix) {
        v.state     = PositionState::Live;
        v.latitude  = last_.latitude;
        v.longitude = last_.longitude;
        if (usableCourse(last_, cfg_.minHeadingSpeedKmh)) {
            v.marker    = Marker::Arrow;
            v.courseDeg = last_.courseDeg >= 360.0f ? 0.0f : last_.courseDeg;
        } else {
            v.marker = Marker::Dot;
        }
        return v;
    }
    if (haveFix_) {   // an older fix exists: show it, greyed, never as live
        v.state     = PositionState::LastKnown;
        v.marker    = Marker::LastKnown;
        v.latitude  = lastFix_.latitude;
        v.longitude = lastFix_.longitude;
        return v;
    }
    if (!fresh) return v;   // NoData: the link is down or silent and never had a fix
    v.state = last_.gpsPresent ? PositionState::Acquiring : PositionState::NoReceiver;
    return v;
}

} // namespace dashcam::navigation
