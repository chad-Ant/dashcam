#ifndef LIBNAVIGATION_H
#define LIBNAVIGATION_H

/**
 * @file libnavigation.h
 * @brief Position map for the ES3C28P MAP tab: GPS state, Web Mercator, an
 *        HTTP tile cache, a 320x224 map compositor and the service that feeds
 *        composed JPEGs to the display link.
 *
 * Design: peripherals/es3c28p/NAVIGATION_PLAN.md.  Decisions in force: north-up,
 * a dedicated MAP tab, online raster tiles (OpenStreetMap by default) through a
 * phone hotspot, cached on disk; no routing, road snapping or dead reckoning.
 *
 * @verbatim
 *   C3 bridge telemetry --offerGps()--> mailbox --+
 *                                                 |  render thread (4 fps shown,
 *   tile workers (disk cache, then HTTP while      |  WarmFps hidden, compose only)
 *   the map is shown) --> decoded-tile RAM LRU ---+--> MapCompositor --> JPEG
 *                                                         --> IPreviewSink (Es3DisplayLink::mapSink())
 * @endverbatim
 *
 * Tile policy (OSMF): an application User-Agent, visible attribution, HTTP
 * caching honoured (validators, max-age, stale-if-error), requests only for
 * the visible map — never while the map is hidden, never prefetching.  A 403
 * disables the provider for the session; 429/5xx/network errors back off with
 * jitter up to 60 s (longer if Retry-After says so).
 *
 * Freshness is reported, never invented: a GPS sample older than gpsStaleMs is
 * shown as "Last known"; an invalid fix clears the live marker at once; tiles
 * and position are independent (a fresh position draws on a blank grid when
 * tiles are missing).
 *
 * Thread safety: NavigationService's public methods are callable from any
 * thread; offerGps() only copies a small struct into a mailbox.  The pure
 * helpers have no state.  libnavigation does not depend on libcommlink or
 * libconfig: the app converts telemetry and config into the structs below.
 */

#include "liblog.h"
#include "libvideopreview.h"

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace dashcam::navigation {

constexpr int      kTileSize     = 256;              ///< Raster tile edge (px).
constexpr size_t   kTileRgbBytes = size_t(kTileSize) * kTileSize * 3;
constexpr int      kMinZoom      = 1;
constexpr int      kMaxZoom      = 19;
constexpr double   kMaxLatitude  = 85.05112878;      ///< Web Mercator projection clamp.

// ─── Web Mercator (pure) ──────────────────────────────────────────────────────

/// A point in world pixels at one zoom: x in [0, 256·2^z), y in [0, 256·2^z].
struct WorldPoint {
    double x = 0.0, y = 0.0;
};

/// WGS-84 → world pixels.  Latitude is clamped to ±kMaxLatitude for the
/// projection only; longitude is wrapped into [-180, 180).
WorldPoint project(double latDeg, double lonDeg, int zoom);

/// World width at @p zoom (= 256 · 2^zoom).
double worldSize(int zoom);

/// Shortest signed x difference b − a across the antimeridian.
double wrapDx(double a, double b, int zoom);

/// Ground metres per pixel at @p latDeg (WGS-84 equatorial radius).
double metersPerPixel(double latDeg, int zoom);

/// One raster tile address.  x wraps, y is clamped to the world.
struct TileKey {
    int      z = 0;
    uint32_t x = 0, y = 0;
    bool operator==(const TileKey& o) const { return z == o.z && x == o.x && y == o.y; }
    bool operator!=(const TileKey& o) const { return !(*this == o); }
};

/// A tile and where its top-left corner lands in a view (pixels, may be negative).
struct TilePlacement {
    TileKey key;
    int     dstX = 0, dstY = 0;
};

/// Tiles covering a @p w × @p h view whose centre is world point @p centre.
std::vector<TilePlacement> tilesForView(const WorldPoint& centre, int zoom, int w, int h);

// ─── GPS state (pure) ─────────────────────────────────────────────────────────

/// One bridge telemetry sample, as the app copies it (no libcommlink types).
struct GpsSample {
    double   latitude    = 0.0;   ///< Promoted from the float telemetry (sub-pixel at zoom 16).
    double   longitude   = 0.0;
    float    speedKmh    = 0.0f;  ///< GPS ground speed; NaN if unknown.
    float    courseDeg   = 0.0f;  ///< Filtered course over ground (MKR); NaN while unreliable.
    uint8_t  fixType     = 0;     ///< u-blox: 0 none, 2 2D, 3 3D, 4 GNSS+DR.
    uint8_t  satellites  = 0;
    bool     fixValid    = false; ///< Telemetry fixValid.
    bool     gpsFix      = false; ///< TLM_FLAG_GPS_FIX.
    bool     gpsPresent  = false; ///< TLM_FLAG_GPS_PRESENT (receiver answering).
    uint64_t receivedUs  = 0;     ///< Steady-clock µs when the bridge frame arrived.
};

/// What the map can say about the vehicle's position right now.
enum class PositionState : uint8_t {
    NoData,       ///< No telemetry at all yet (or the link is down and never had a fix).
    NoReceiver,   ///< Telemetry flows, the GNSS receiver is not answering.
    Acquiring,    ///< Receiver present, no usable fix (and none earlier).
    Live,         ///< A usable fix in a sample younger than staleMs.
    LastKnown,    ///< The fix is old, invalid now, or the link dropped: last position, greyed.
};
const char* positionStateName(PositionState s);

enum class Marker : uint8_t { None, Dot, Arrow, LastKnown };

/// The renderer's input, derived by NavState::view().
struct NavView {
    PositionState state      = PositionState::NoData;
    Marker        marker     = Marker::None;
    double        latitude   = 0.0, longitude = 0.0;   ///< Valid unless marker == None.
    float         courseDeg  = 0.0f;                    ///< [0, 360), valid for Arrow.
    float         speedKmh   = 0.0f;                    ///< NaN when unknown.
    uint8_t       fixType    = 0;
    uint8_t       satellites = 0;
};

struct NavStateConfig {
    uint32_t staleMs            = 1500;  ///< Sample age at which Live becomes LastKnown.
    float    minHeadingSpeedKmh = 5.0f;  ///< The MKR's HEADING_MIN_SPEED_KMH gate.
};

/// True when @p s carries a position the map may show (fix flags agree, the
/// fix type is 2D/3D/GNSS+DR, the coordinates are finite and in range; (0,0)
/// is a legitimate position).
bool usablePosition(const GpsSample& s);

/// True when @p s also carries a course the arrow may show: usable position,
/// finite course in [0, 360], speed at or above @p minSpeedKmh.
bool usableCourse(const GpsSample& s, float minSpeedKmh);

/**
 * @brief Freshness and marker state machine.  Not thread-safe (the service
 *        owns one on its render thread).
 */
class NavState {
public:
    explicit NavState(NavStateConfig cfg = {}) : cfg_(cfg) {}
    void    offer(const GpsSample& s);   ///< A new bridge sample.
    void    linkLost();                  ///< The bridge disconnected (keeps the last fix as LastKnown).
    NavView view(uint64_t nowUs) const;

private:
    NavStateConfig cfg_;
    bool      haveSample_ = false, haveFix_ = false, linkLost_ = false;
    GpsSample last_{};      ///< Newest sample.
    GpsSample lastFix_{};   ///< Newest sample with a usable position.
};

// ─── tiles: PNG and HTTP caching rules (pure) ─────────────────────────────────

/**
 * Decodes a 256×256 PNG into packed RGB (@p rgb resized to kTileRgbBytes).
 * Rejects other sizes before decoding pixels, and anything over @p maxBytes.
 */
bool decodeTilePng(const uint8_t* data, size_t size, std::vector<uint8_t>& rgb, std::string* why = nullptr,
                   size_t maxBytes = 512 * 1024);

/// The Cache-Control directives the cache acts on (seconds; -1 = absent).
struct CacheControl {
    int64_t maxAge               = -1;
    int64_t staleWhileRevalidate = 0;
    int64_t staleIfError         = 0;
    bool    noStore              = false;
    bool    noCache              = false;
};
CacheControl parseCacheControl(const std::string& value);

/// RFC 7231 IMF-fixdate (and the two obsolete forms) → Unix seconds; -1 if unparsable.
int64_t parseHttpDate(const std::string& value);

/// Retry-After (delta seconds or HTTP date) → seconds from @p nowS; -1 if absent/unparsable.
int64_t parseRetryAfter(const std::string& value, int64_t nowS);

/// Freshness metadata of one cached tile (Unix seconds).
struct TileFreshness {
    int64_t fetchedAt            = 0;
    int64_t freshUntil           = 0;   ///< Fresh while now < freshUntil.
    int64_t staleWhileRevalidate = 0;   ///< Seconds past freshUntil usable while revalidating.
    int64_t staleIfError         = 0;   ///< Seconds past freshUntil usable when the server is unreachable.
};

/// Freshness from response headers received at @p nowS (Date/Age honoured;
/// heuristic 10% of the Last-Modified age, at most 1 day, when no max-age/Expires).
TileFreshness freshnessFromHeaders(const CacheControl& cc, int64_t dateS, int64_t expiresS, int64_t lastModifiedS,
                                   int64_t ageS, int64_t nowS);

/// The system clock is plausible for HTTP freshness (after 2025-01-01); the
/// Jetson has no RTC battery and boots with an old clock until NTP/GPS set it.
bool clockPlausible(int64_t nowS);

// ─── persistent tile cache (SQLite) ───────────────────────────────────────────

/// One cached tile: the PNG exactly as received, its validators and freshness.
struct CachedTile {
    std::vector<uint8_t> png;
    std::string          etag, lastModified;
    TileFreshness        fresh;
};

/**
 * @brief SQLite tile store with an LRU byte budget.  Thread-safe (one mutex
 *        around one connection).  Disk errors are reported, never fatal.
 */
class TileCache {
public:
    TileCache();
    ~TileCache();
    TileCache(const TileCache&)            = delete;
    TileCache& operator=(const TileCache&) = delete;

    /// Opens/creates @p path (parent directory created).  False with @p why on failure.
    bool open(const std::string& path, uint64_t budgetBytes, std::string* why = nullptr);
    void close();
    bool isOpen() const;

    bool get(const std::string& provider, const TileKey& k, CachedTile& out, int64_t nowS);
    bool put(const std::string& provider, const TileKey& k, const CachedTile& t, int64_t nowS);
    /// 304 Not Modified: new freshness (and validators if sent), same PNG.
    bool refresh(const std::string& provider, const TileKey& k, const TileFreshness& f, const std::string& etag,
                 const std::string& lastModified, int64_t nowS);
    /// Evicts least-recently-used tiles beyond the budget, never one in @p keep.
    void evict(const std::string& provider, const std::vector<TileKey>& keep);

    uint64_t totalBytes() const;
    uint64_t tileCount() const;
    uint64_t evictions() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// ─── compositor (pure apart from its buffers) ─────────────────────────────────

/// Status the frame's top strip shows besides the position.
enum class NetState : uint8_t { Unknown, Online, Offline, Denied };
const char* netStateName(NetState s);

struct MapLayout {
    int width = 320, height = 224;   ///< The board's map viewport.
    int statusH = 16;                ///< Top strip: GPS / speed / direction / network.
    int attributionH = 16;           ///< Bottom strip: the licence credit.
};

/// Returns the decoded RGB of a tile or nullptr when it is not in RAM.
using TileLookup = std::function<const uint8_t*(const TileKey&)>;

struct ComposeResult {
    int  tilesNeeded = 0, tilesDrawn = 0;
    bool mapComplete = false;        ///< Every visible tile drawn.
};

/**
 * Draws one frame into @p rgb (width·height·3): tiles (north-up, the vehicle
 * at the centre of the map area), the marker, a north indicator, a scale bar,
 * the status strip and the attribution strip.  With no position the map area
 * shows a neutral grid and the reason.
 */
ComposeResult composeMap(const NavView& v, int zoom, const MapLayout& layout, const TileLookup& lookup,
                         NetState net, const std::string& attribution, std::vector<uint8_t>& rgb);

/// Baseline 4:4:4 JPEG of packed RGB written straight into @p out; false if it
/// does not fit @p capacity (the caller retries at a lower quality) or on an encoder error.
bool encodeMapJpeg(const uint8_t* rgb, int w, int h, int quality, uint8_t* out, size_t capacity, size_t& outSize);

// ─── the service ──────────────────────────────────────────────────────────────

struct NavigationServiceConfig {
    std::string providerUrl  = "https://tile.openstreetmap.org/{z}/{x}/{y}.png";
    std::string providerId   = "osm-standard";
    std::string userAgent    = "DashcamES3Map/1.0 (+https://github.com/chad-Ant/dashcam)";
    std::string attribution  = "\xC2\xA9 OpenStreetMap contributors";
    std::string cachePath;                       ///< Empty = RAM only (tests).
    uint64_t    cacheBudgetBytes = 256ull << 20;
    int         zoom         = 16;
    int         maxFps       = 4;                ///< While the map is shown.
    int         warmFps      = 1;                ///< While hidden (compose only, disk only); 0 = off.
    int         jpegQuality  = 80;
    uint32_t    gpsStaleMs   = 1500;
    int         fetchWorkers = 2;                ///< 1-2 concurrent HTTP requests.
    // A phone hotspot can be slow but steady: a transfer is aborted when it
    // stalls (below lowSpeedBytesPerS for lowSpeedTimeS), not after a short
    // fixed time that would cut off and re-download a slowly arriving tile.
    uint32_t    connectTimeoutMs   = 4000;
    uint32_t    requestTimeoutMs   = 20000;   ///< Hard cap per request.
    uint32_t    lowSpeedBytesPerS  = 1024;
    uint32_t    lowSpeedTimeS      = 6;
    size_t      maxTileBytes = 512 * 1024;
    uint32_t    maxBackoffMs = 60000;
};

/// Snapshot of the service (counters cumulative since start()).
struct NavigationStats {
    PositionState position = PositionState::NoData;
    NetState      net      = NetState::Unknown;
    bool          visible  = false;              ///< The map sink's demand is active.
    uint64_t gpsSamples = 0;
    uint64_t framesComposed = 0, framesEncoded = 0, framesOffered = 0, framesRefused = 0, encodeFailures = 0;
    uint64_t warmComposes = 0;
    uint64_t tileRamHits = 0, tileRamMisses = 0, tileDiskHits = 0;
    uint64_t httpRequests = 0, http200 = 0, http304 = 0, http404 = 0, http403 = 0, http429 = 0, http5xx = 0,
             httpErrors = 0, httpBytes = 0, httpRejected = 0;   ///< Rejected = not a valid 256×256 PNG / too big.
    uint64_t queueDropped = 0;                   ///< Obsolete queued requests discarded after a view change.
    uint64_t cacheBytes = 0, cacheTiles = 0, cacheEvictions = 0;
    double   lastComposeMs = 0.0, lastEncodeMs = 0.0;
    uint32_t lastJpegBytes = 0;
    int      lastQuality = 0;
    int64_t  firstFrameAfterShowMs = -1;         ///< Demand became active → first frame offered (last time).
};

/**
 * @brief Long-running map producer.  start() once with the display link's map
 *        sink; it keeps running whether or not the MAP tab is shown.
 */
class NavigationService {
public:
    NavigationService();
    ~NavigationService();   ///< stop()s.
    NavigationService(const NavigationService&)            = delete;
    NavigationService& operator=(const NavigationService&) = delete;

    /// False (service off, nothing running) only for invalid settings or a
    /// thread start failure; an unusable cache path degrades to RAM only.
    bool start(const NavigationServiceConfig& cfg, dashcam::preview::IPreviewSink& mapSink,
               dashcam::log::LogCallback log = {});

    void offerGps(const GpsSample& s);   ///< Any thread; copies into a mailbox, never blocks on I/O.
    void bridgeLost();                   ///< The telemetry link dropped.

    NavigationStats stats() const;
    bool running() const;
    void stop();                         ///< Idempotent; joins every thread, releases every lease.

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace dashcam::navigation

#endif // LIBNAVIGATION_H
