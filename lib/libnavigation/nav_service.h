// nav_service.h — NavigationService::Impl, internal to libnavigation and shared
// by libnavigation.cpp (public API, start/stop), nav_fetch.cpp (tile workers:
// disk cache, HTTP, provider health) and nav_render.cpp (the render thread:
// whole frames and board-side panning).
//
// Threads and what they own:
//   render thread  NavState, PoseInterpolator, WholeFrame, Pan; the only caller
//                  of the sinks; never touches the disk or the network.
//   tile workers   one CURL handle each; TileCache I/O; ramInsert().
//   any thread     offerGps()/bridgeLost() (mailbox), stats().
// Locks are never nested except qMu+mailMu, taken together (scoped_lock) only
// to clear `run` before notifying, so no waiter can miss the stop.

#ifndef NAV_SERVICE_H
#define NAV_SERVICE_H

#include "libnavigation.h"

#include <array>
#include <cmath>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <random>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace dashcam::navigation {

namespace detail {

constexpr size_t   kRamTiles       = 20;     ///< Decoded tiles kept: a panning background (9) + the next one's new tiles.
constexpr size_t   kQueueDepth     = 8;      ///< Distinct tile keys waiting for a worker.
constexpr size_t   kFrameSlots     = 4;      ///< Panning: background + chrome pending, one in flight.
constexpr size_t   kFrameSlotBytes = 65536;  ///< = the board's JPEG slot.

int64_t steadyUs();
int64_t wallS();

/// @p url without its query string or user:password (API keys stay out of logs).
std::string redactUrl(const std::string& url);

struct KeyLess {
    bool operator()(const TileKey& a, const TileKey& b) const {
        if (a.z != b.z) return a.z < b.z;
        if (a.x != b.x) return a.x < b.x;
        return a.y < b.y;
    }
};

/**
 * Tile-server health shared by the workers (thread-safe).  Every setback only
 * pushes the next-allowed time later; a success clears it only if no setback
 * happened while that request was in flight (a 200 that crossed a 429 must not
 * cancel its Retry-After), and never lifts a disable.
 */
class ProviderHealth {
public:
    static constexpr uint32_t kFirstWaitMs     = 2000;
    static constexpr uint32_t kMaxUnreachableMs = 10000;   ///< No HTTP response at all: retry soon.

    void     reset(uint32_t maxServerWaitMs);
    bool     allows(int64_t nowUs) const;
    uint64_t generation() const;                 ///< Take before a request, give to succeeded().
    /// allows() and generation() in one step: call right before a request, so a
    /// setback that lands after admission makes its success not count.
    bool     admit(int64_t nowUs, uint64_t& generation) const;
    void     succeeded(uint64_t generationAtStart);
    /// 429, 5xx, 408, an unexpected status: exponential up to maxServerWaitMs, at least Retry-After.
    void     serverTrouble(int64_t nowUs, int64_t retryAfterS);
    /// The request never reached the server (DNS, connect, TLS, stalled): up to kMaxUnreachableMs.
    void     unreachable(int64_t nowUs);
    /// 403 or a redirect: no more requests this run.  True the first time.
    bool     disable();
    NetState state() const;
    uint32_t lastWaitMs() const;

private:
    void holdOff(int64_t nowUs, uint32_t& stepMs, uint32_t capMs, int64_t minWaitMs);

    mutable std::mutex m_;
    bool             disabled_ = false;
    uint32_t         maxServerMs_ = 60000, serverStepMs_ = 0, unreachableStepMs_ = 0, lastWaitMs_ = 0;
    int64_t          notBeforeUs_ = 0;
    uint64_t         generation_ = 0;
    NetState         net_ = NetState::Unknown;
    std::minstd_rand rng_{uint32_t(steadyUs())};
};

} // namespace detail

struct NavigationService::Impl {
    using EncodedFrameLease = dashcam::preview::EncodedFrameLease;

    NavigationServiceConfig                        cfg;
    dashcam::preview::IPreviewSink*                sink    = nullptr;
    IMapPanSink*                                   panSink = nullptr;
    std::shared_ptr<dashcam::preview::EncodedFramePool> pool;
    TileCache                                      disk;
    std::atomic<bool>                              diskOk{false};

    std::atomic<bool>        run{false};
    std::thread              render;
    std::vector<std::thread> workers;
    std::mutex               exitMu;     ///< Bounded stop: workers report their exit.
    std::condition_variable  exitCv;
    size_t                   workersExited = 0;

    // Log callback; cleared when stop() leaves a worker behind, so nothing logs
    // into an application that has moved on.
    std::mutex                logMu;
    dashcam::log::LogCallback log;
    void logf(dashcam::log::LogLevel lvl, const char* fmt, ...) __attribute__((format(printf, 3, 4)));

    // GPS mailbox (any thread -> render thread).
    std::mutex              mailMu;
    std::condition_variable renderCv;
    bool                    mailNew = false, mailLost = false, tileArrived = false;
    GpsSample               mail{};

    // Decoded tiles.  The render thread holds lruMu for a whole composition so a
    // tile it draws cannot be evicted under it.
    struct RamTile {
        TileKey              key;
        std::vector<uint8_t> rgb;
        uint64_t             lastUse = 0;
        bool                 used    = false;
        bool                 stale   = false;   ///< Shown past its freshness: ask the server while shown.
    };
    std::mutex                             lruMu;
    std::array<RamTile, detail::kRamTiles> ram;
    uint64_t                               useClock = 0;
    const uint8_t* ramLookup(const TileKey& k);           ///< lruMu held; marks it recently used.
    const RamTile* ramFind(const TileKey& k) const;       ///< lruMu held; no LRU touch.
    void           ramInsert(const TileKey& k, std::vector<uint8_t>&& rgb, bool stale);
    void           ramMarkFresh(const TileKey& k);

    // Work for the tile workers.
    struct Request {
        TileKey key;
        bool    network = false;
    };
    std::mutex                                  qMu;
    std::condition_variable                     qCv;
    std::deque<Request>                         queue;
    std::set<TileKey, detail::KeyLess>          busy;       ///< Queued or being fetched.
    std::map<TileKey, int64_t, detail::KeyLess> negative;   ///< Not asked again until (steady us): 404, bad PNG...
    std::map<TileKey, int64_t, detail::KeyLess> diskMiss;   ///< Unusable on disk, server not askable: until (steady us).
    std::vector<TileKey>                        keep;       ///< The newest working set: eviction spares it.
    std::set<long>                              warnedStatus;
    std::atomic<bool>                           putWarned{false};   ///< "cache write failed" logged once per run.
    /// The map is on screen: the only time a worker may use the network
    /// (checked again at dequeue, so work queued while shown never fetches
    /// once hidden).
    std::atomic<bool>                           fetchAllowed{false};
    /// Queue the @p missing tiles, skipping negatives and duplicates; drop queued
    /// work outside @p view; queued work takes @p network (both ways).
    void request(const std::vector<TileKey>& view, const std::vector<TileKey>& missing, bool network);
    void downgradeQueue();   ///< The map was hidden: queued work becomes disk-only.

    detail::ProviderHealth health;

    mutable std::mutex statsMu;
    NavigationStats    st;
    template <typename F> void stat(F f) {
        std::lock_guard<std::mutex> lk(statsMu);
        f(st);
    }

    // ── tile workers (nav_fetch.cpp) ──
    void workerLoop();
    void processTile(void* curl, const Request& rq);
    void reportCacheNotice();

    // ── render thread (nav_render.cpp) ──
    struct Mail {
        bool      newSample = false, lost = false, arrived = false;
        GpsSample sample;
    };
    /// What a whole frame shows, at the precision it is drawn: a repeated 10 Hz
    /// bridge sample that changes none of it is not a change.
    struct RenderKey {
        PositionState state = PositionState::NoData;
        Marker        marker = Marker::None;
        int64_t       px = 0, py = 0;
        int           courseDeg = 0, speedKmh = -1, fixType = 0;
        NetState      net = NetState::Unknown;
        bool operator==(const RenderKey& o) const {
            return state == o.state && marker == o.marker && px == o.px && py == o.py && courseDeg == o.courseDeg &&
                   speedKmh == o.speedKmh && fixType == o.fixType && net == o.net;
        }
        bool operator!=(const RenderKey& o) const { return !(*this == o); }
    };
    struct WholeFrame {
        bool                 wasVisible = false, firstSent = true, pending = true;
        int64_t              lastRenderUs = 0, showUs = 0;
        uint64_t             seq = 0;
        RenderKey            lastKey;
        std::vector<uint8_t> rgb;
    };
    struct Pan {
        uint32_t session = 0, resend = 0;
        bool     firstSent = false;
        bool     reshow    = false;   ///< Not panning since the last pass: the board needs its images again.
        int64_t  showUs = 0, lastPoseUs = 0, lastRequestUs = 0, lastAheadUs = 0, aheadUntilUs = 0;
        std::vector<TileKey> aheadKeys;           ///< The next background's tiles (look-ahead).
        double   frameX = 0.0;   ///< The vehicle's x in this session's continuous frame.
        bool     haveFrame = false;
        struct Background {
            bool                 sent = false, hasPos = false, complete = true;
            int64_t              ox = 0, oy = 0, lastUs = 0;
            std::vector<TileKey> keys, missing;   ///< Its tiles; those not in RAM when it was drawn.
        } bg;
        struct Chrome {
            int64_t              lastUs = 0;
            Marker               marker = Marker::None;
            PositionState        state  = PositionState::NoData;
            std::vector<uint8_t> rgb, last;       ///< last = what the board has (empty: nothing yet).
        } chrome;
        std::vector<uint8_t> rgb;                 ///< Background scratch.
    };
    struct PanPosition {
        bool       have = false;
        WorldPoint pos, vel;      ///< pos.x in the session's continuous frame.
        float      course = NAN;
    };
    enum class BgAction { None, Recentre, Retile };
    enum class Encoded { Ok, NoSlot, TooLarge };

    void      renderLoop();
    Mail      takeMail(NavState& nav);
    RenderKey renderKey(const NavView& v, NetState n) const;
    int64_t   wholeFramePeriodUs(bool visible) const;
    /// One whole-frame pass; returns when the next frame can be due (steady us).
    int64_t   wholeFrameStep(WholeFrame& wf, const dashcam::preview::PreviewDemand& d, bool visible, const NavView& v,
                             NetState n, const Mail& mail, int64_t now);
    void      panStep(Pan& pan, const PanDemand& d, const NavView& v, const PoseInterpolator& interp, NetState n,
                      int64_t now);
    PanPosition panPosition(Pan& pan, const PanDemand& d, const NavView& v, const PoseInterpolator& interp,
                            int64_t now);
    BgAction  decideBackground(const Pan& pan, const PanDemand& d, const PanPosition& p, int64_t now);
    void      sendBackground(Pan& pan, const PanDemand& d, const NavView& v, const PanPosition& p, BgAction a,
                             int64_t now);
    bool      prefetchAhead(Pan& pan, const PanDemand& d, const PanPosition& p, int64_t now);
    void      rerequestMissing(Pan& pan, int64_t now, bool force = false);
    void      sendChrome(Pan& pan, const PanDemand& d, const NavView& v, NetState n, int64_t now);
    void      sendPose(Pan& pan, const PanDemand& d, const NavView& v, const PanPosition& p, int64_t now);
    int       poseHzFor(const PanDemand& d) const;
    std::vector<TileKey> keysForRect(int64_t ox, int64_t oy, int w, int h) const;
    /// lruMu held: @p absent gets the keys not in RAM; @p stale (when given)
    /// those in RAM but shown past their freshness, to be revalidated.
    void      collectMissing(const std::vector<TileKey>& keys, std::vector<TileKey>& absent,
                             std::vector<TileKey>* stale, uint64_t* hits);
    Encoded   encodeLease(const std::vector<uint8_t>& rgb, int w, int h, int q0, int retries, bool chroma420,
                          size_t maxBytes, uint64_t seq, EncodedFrameLease& lease, size_t& outSize, int& quality);
    void      publishState(const NavView& v, NetState n, bool shown, bool panning, const PoseInterpolator& interp);
};

} // namespace dashcam::navigation

#endif // NAV_SERVICE_H
