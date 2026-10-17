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
 *                                                 |  render thread (nav_render.cpp):
 *   tile workers (nav_fetch.cpp: disk cache, then |   whole frames (MaxFps shown, WarmFps hidden)
 *   HTTP while the map is shown) --> RAM LRU -----+   --> IPreviewSink (Es3DisplayLink::mapSink())
 *                                                     or panning: background / chrome / poses
 *                                                     --> IMapPanSink (Es3DisplayLink::panSink())
 * @endverbatim
 *
 * Tile policy (OSMF): an application User-Agent, visible attribution, HTTP
 * caching honoured (validators, max-age, stale-while-revalidate,
 * stale-if-error), requests only around the map on screen — never while it is
 * hidden (queued work is downgraded to disk-only the moment it is hidden),
 * never along a route.  A 403 or a redirect disables the provider for the
 * session; 429/5xx back off with jitter up to maxBackoffMs (longer if
 * Retry-After says so); a transfer that never reached the server (no
 * connection, DNS) backs off only up to 10 s, so tiles resume soon after the
 * hotspot returns.
 *
 * Board-side panning (map extension v2, when the board has it): instead of whole
 * frames the service sends a background larger than the screen rarely, the
 * fixed strips on change and a pose at poseHz, interpolated between GPS fixes
 * (PoseInterpolator); the board scrolls and draws the marker itself
 * (IMapPanSink).  The tiles of the next background are asked for a few seconds
 * before it is due.
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
constexpr float    kMinHeadingSpeedKmh = 5.0f;       ///< The MKR's HEADING_MIN_SPEED_KMH gate (arrow vs dot).
constexpr uint32_t kPoseSubpixels      = 16;         ///< PanPose coordinates are 1/16 world pixel (= es3map's).

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

/// One bridge telemetry sample, as the app copies it (no libcommlink types;
/// src/bridge_overlay.h toNavSample() makes one from the bridge telemetry).
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
    float    minHeadingSpeedKmh = kMinHeadingSpeedKmh;
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

/// Freshness after a 304: the 304's own Cache-Control/Expires when it has
/// them, otherwise the stored lifetime and stale windows (RFC 9111 4.3.4).
TileFreshness freshnessAfter304(const TileFreshness& stored, const CacheControl& cc, bool hasCacheControl,
                                int64_t dateS, int64_t expiresS, int64_t lastModifiedS, int64_t ageS, int64_t nowS);

/// The system clock is plausible for HTTP freshness (after 2025-01-01); the
/// Jetson has no RTC battery and boots with an old clock until NTP/GPS set it.
bool clockPlausible(int64_t nowS);

/// What a cached tile may be used for now.
enum class TileUse : uint8_t {
    Fresh,             ///< Show; no request (also whenever the clock is implausible).
    StaleRevalidate,   ///< Within stale-while-revalidate: show now and revalidate.
    StaleIfError,      ///< Within stale-if-error only: show when the server cannot be asked or fails.
    Expired,           ///< Show only after a successful revalidation.
};
TileUse classifyTile(const TileFreshness& f, int64_t nowS, bool clockSane);

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

    /// Opens/creates @p path (parent directory created).  False with @p why on
    /// failure.  A corrupt file is set aside as *.corrupt and replaced by an
    /// empty cache (takeNotice() says so).
    bool open(const std::string& path, uint64_t budgetBytes, std::string* why = nullptr);
    void close();
    /// One pending message for the owner's log (a rebuilt file, corruption seen
    /// at run time after which the cache is no longer used), or empty.
    std::string takeNotice();

    bool get(const std::string& provider, const TileKey& k, CachedTile& out, int64_t nowS);
    bool put(const std::string& provider, const TileKey& k, const CachedTile& t, int64_t nowS);
    /// 304 Not Modified: new freshness (and validators if sent), same PNG.
    bool refresh(const std::string& provider, const TileKey& k, const TileFreshness& f, const std::string& etag,
                 const std::string& lastModified, int64_t nowS);
    /// Evicts least-recently-used tiles beyond the budget, never one in @p keep.
    void evict(const std::string& provider, const std::vector<TileKey>& keep);

    /// Counters kept up to date by put()/evict(); never wait for the database.
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

/// Baseline JPEG of packed RGB written straight into @p out (4:4:4, or 4:2:0
/// with @p chroma420: about half the size, slightly softer coloured detail);
/// false if it does not fit @p capacity (the caller retries at a lower quality)
/// or on an encoder error.
bool encodeMapJpeg(const uint8_t* rgb, int w, int h, int quality, uint8_t* out, size_t capacity, size_t& outSize,
                   bool chroma420 = false);

// ─── panning surface (map extension v2, board-side panning) ───────────────────

/**
 * @brief Smooth position between GPS fixes (pure, not thread-safe).
 *
 * The MKR's GNSS gives about 4 fixes a second and the bridge repeats the
 * newest one in every 10 Hz telemetry frame.  offer() keeps only new fixes,
 * smooths their arrival times (the 100 ms bridge cadence quantises them),
 * and at() returns the position one fix interval (+50 ms) in the past,
 * linearly between the two real fixes around that moment: the marker lags
 * about a quarter of a second but never shows a place the vehicle was not
 * at (no extrapolation).  Past the newest fix it holds still.  A gap over
 * 2 s or a jump over 2000 px starts afresh (no sliding across a teleport).
 */
class PoseInterpolator {
public:
    void reset();
    /// A usable fix: world position at the service zoom, course (NaN = none), arrival time.
    void offer(const WorldPoint& p, float courseDeg, uint64_t arrivalUs);
    bool empty() const { return n_ == 0; }
    /// The position at @p nowUs (see the class comment); @p courseDeg gets the
    /// interpolated course (NaN when neither fix has one).
    WorldPoint at(uint64_t nowUs, float* courseDeg = nullptr) const;
    WorldPoint newest() const;
    WorldPoint velocity() const;              ///< px/s over the newest segment (0 with fewer than 2 fixes).
    uint32_t   intervalUs() const { return uint32_t(intervalUs_); }   ///< Smoothed fix interval.
    uint32_t   delayUs() const;               ///< How far at() lags: interval + 50 ms.
    size_t     fixes() const { return n_; }

private:
    struct Fix {
        WorldPoint p;
        float      course = 0.0f;
        uint64_t   t = 0;                     ///< Smoothed fix time.
    };
    static constexpr size_t kRing = 8;
    std::array<Fix, kRing> ring_{};
    size_t   n_ = 0, head_ = 0;               ///< head_ = next write slot.
    double   intervalUs_ = 250000.0;
    uint64_t lastArrivalUs_ = 0;
    const Fix& fromNewest(size_t k) const { return ring_[(head_ + kRing - 1 - k) % kRing]; }
};

/**
 * True when a new background is due: the map area centred on @p pos, and
 * again @p lookS seconds further along @p vel, must stay at least @p edgePx
 * inside the background whose top-left is (ox, oy) (world pixels).
 */
bool panBackgroundStale(double ox, double oy, int bgW, int bgH, int mapW, int mapH, const WorldPoint& pos,
                        const WorldPoint& vel, double lookS, int edgePx);

/**
 * Top-left of a new background around @p pos: centred @p leadS seconds ahead
 * along @p vel, rounded to whole pixels.  The lead is limited per axis to 60%
 * of the margin and to what leaves @p edgePx + 1 behind the vehicle, so a
 * fresh background is never stale already (the vehicle can also turn back).
 */
void panBackgroundOrigin(const WorldPoint& pos, const WorldPoint& vel, double leadS, int bgW, int bgH, int mapW,
                         int mapH, int64_t& ox, int64_t& oy, int edgePx = 8);

/**
 * Draws a panning background into @p rgb (w·h·3): tiles only, top-left at
 * world pixel (ox, oy) (x wraps), missing tiles hatched.  With @p havePosition
 * false: the neutral grid with the reason in the middle (the board centres an
 * unmarked pose there).
 */
ComposeResult composePanBackground(const NavView& v, bool havePosition, int zoom, int64_t ox, int64_t oy, int w,
                                   int h, const TileLookup& lookup, std::vector<uint8_t>& rgb);

/**
 * Draws the chrome of a panning map into @p rgb (w × (topH + bottomH) × 3): the
 * status strip (as composeMap()'s, plus LOADING while the background is
 * incomplete), then the bottom strip: scale bar and north mark at the left,
 * attribution at the right.
 */
void composePanChrome(const NavView& v, int zoom, int w, int topH, int bottomH, NetState net, bool mapComplete,
                      const std::string& attribution, std::vector<uint8_t>& rgb);

/**
 * What a panning map surface wants now.  In a panning session the board keeps
 * a background larger than its map area and scrolls it itself; the host sends
 * the background rarely, the two fixed strips (chrome) on change, and a pose
 * many times a second (Es3MapProtocol.h, extension v2).
 */
struct PanDemand {
    bool     active      = false;   ///< A panning session exists and the MAP tab is shown.
    uint32_t session     = 0;       ///< The board session (0 = none).
    /// Changes whenever the board needs the background and chrome again
    /// although the session stayed: the map was shown again, or an image was
    /// lost after it was offered.  A new session needs them too.
    uint32_t resend      = 0;
    uint16_t mapW        = 0;       ///< Map area between the strips (320x192 on the ES3C28P).
    uint16_t mapH        = 0;
    uint16_t backgroundW = 0;       ///< Every background image is exactly this size.
    uint16_t backgroundH = 0;
    uint16_t chromeTopH  = 0;       ///< Chrome image = mapW x (chromeTopH + chromeBottomH).
    uint16_t chromeBottomH = 0;
    size_t   maxJpegBytes = 0;
    uint8_t  maxPoseHz    = 0;      ///< The board's pose rate ceiling.
};

/// Where the board centres the map area and draws the marker: world pixels x
/// kPoseSubpixels at the service's zoom, in one continuous frame per session
/// (x may exceed the world width near the antimeridian).
struct PanPose {
    uint32_t xQ4        = 0, yQ4 = 0;
    uint16_t courseCdeg = 0;        ///< Clockwise from north, for Marker::Arrow.
    Marker   marker     = Marker::None;
};

/**
 * @brief Receiver of a panning map (libes3display's Es3DisplayLink::panSink(),
 *        or a test).  Same contract as IPreviewSink: cheap, thread-safe and
 *        non-blocking; an offer takes the lease only when it returns true.
 */
class IMapPanSink {
public:
    virtual ~IMapPanSink() = default;
    virtual PanDemand panDemand() const = 0;
    /// A backgroundW x backgroundH JPEG whose top-left corner is world pixel (originX, originY).
    virtual bool offerPanBackground(dashcam::preview::EncodedFrameLease&& jpeg, uint32_t originX,
                                    uint32_t originY) = 0;
    /// A mapW x (chromeTopH + chromeBottomH) JPEG: the top strip's rows, then the bottom strip's.
    virtual bool offerPanChrome(dashcam::preview::EncodedFrameLease&& jpeg) = 0;
    /// The newest pose; a newer offer replaces one not yet sent.
    virtual bool offerPanPose(const PanPose& pose) = 0;
};

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
    int         jpegQuality  = 80;               ///< Whole frames and chrome; a too-large frame retries 10 lower, twice.
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
    // Board-side panning (used when start() got a panning sink and the board has it).
    int         poseHz          = 20;            ///< 1-60; the board's maximum caps it.
    int         panJpegQuality  = 75;            ///< Background quality (4:2:0); retries go lower.
    double      panLeadS        = 2.0;           ///< A new background is centred this far ahead.
};

/// Snapshot of the service (counters cumulative since start()).
struct NavigationStats {
    PositionState position = PositionState::NoData;
    NetState      net      = NetState::Unknown;
    bool          visible  = false;              ///< The map is on screen (whole frames or panning).
    uint64_t gpsSamples = 0;
    uint64_t framesComposed = 0, framesEncoded = 0, framesOffered = 0, framesRefused = 0, encodeFailures = 0;
    uint64_t poolExhausted = 0;                  ///< No free JPEG slot (the sink still holds them all).
    uint64_t warmComposes = 0;
    uint64_t tileRamHits = 0, tileRamMisses = 0, tileDiskHits = 0;
    uint64_t httpRequests = 0, http200 = 0, http304 = 0, http404 = 0, http403 = 0, http429 = 0, http5xx = 0,
             httpErrors = 0, httpBytes = 0, httpRejected = 0;   ///< Rejected = not a valid 256×256 PNG / too big.
    uint64_t httpOther = 0;                      ///< Other statuses (redirects, other 4xx): tile not asked again for a while.
    uint64_t queueDropped = 0;                   ///< Obsolete queued requests discarded after a view change.
    uint64_t cacheBytes = 0, cacheTiles = 0, cacheEvictions = 0;
    double   lastComposeMs = 0.0, lastEncodeMs = 0.0;
    uint32_t lastJpegBytes = 0;
    int      lastQuality = 0;
    int64_t  firstFrameAfterShowMs = -1;         ///< Demand became active → first frame offered (last time).
    // Board-side panning.
    bool     panning = false;                    ///< The panning sink's demand is active.
    uint64_t panBackgrounds = 0, panBackgroundsRefused = 0, panChrome = 0, panChromeRefused = 0;
    uint64_t posesOffered = 0, posesRefused = 0;
    uint32_t lastBackgroundBytes = 0;
    double   lastBackgroundMs = 0.0;             ///< Compose + encode.
    uint32_t interpolationDelayMs = 0;           ///< How far the marker lags the newest fix.
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
    /// As above, with a panning sink too: while @p panSink's demand is active the
    /// service feeds it (background, chrome, poses at poseHz) instead of mapSink.
    bool start(const NavigationServiceConfig& cfg, dashcam::preview::IPreviewSink& mapSink, IMapPanSink* panSink,
               dashcam::log::LogCallback log = {});

    void offerGps(const GpsSample& s);   ///< Any thread; copies into a mailbox, never blocks on I/O.
    void bridgeLost();                   ///< The telemetry link dropped.

    NavigationStats stats() const;       ///< Never waits for the network or the disk.
    bool running() const;
    /// Idempotent and bounded: the render thread is always joined (every sink
    /// call has returned); a tile worker still stuck in a DNS lookup or disk
    /// I/O after kStopWaitMs is left behind, silenced, and finishes on its own.
    void stop();

    static constexpr int kStopWaitMs = 3000;

    struct Impl;   ///< nav_service.h

private:
    std::mutex            lifeMu_;       ///< Serialises start()/stop().
    std::shared_ptr<Impl> impl_;         ///< Shared with the threads: a left-behind worker keeps it alive.
    std::weak_ptr<Impl>   abandoned_;    ///< The state a left-behind worker still uses (empty when none).
};

} // namespace dashcam::navigation

#endif // LIBNAVIGATION_H
