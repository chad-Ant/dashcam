// libnavigation.cpp — NavigationService's public API: start/stop (bounded),
// the GPS mailbox and stats.  The threads live in nav_render.cpp (render) and
// nav_fetch.cpp (tile workers); their shared state in nav_service.h.

#include "nav_service.h"

#include <curl/curl.h>

#include <chrono>

namespace dashcam::navigation {

using dashcam::log::LogLevel;
using dashcam::preview::EncodedFramePool;

namespace {
std::once_flag g_curlInit;

bool providerUrlOk(const std::string& url, std::string& why) {
    for (const char* key : {"{z}", "{x}", "{y}"})
        if (url.find(key) == std::string::npos) {
            why = std::string("ProviderUrl needs ") + key;
            return false;
        }
    // Only {z} {x} {y} are substituted: anything else in braces would reach the server.
    for (size_t p = url.find('{'); p != std::string::npos; p = url.find('{', p + 1)) {
        const std::string t = url.substr(p, 3);
        if (t != "{z}" && t != "{x}" && t != "{y}") {
            why = "ProviderUrl has an unsupported placeholder at '" + url.substr(p, 8) + "' (only {z} {x} {y})";
            return false;
        }
    }
    if (url.find('}') != std::string::npos && url.find('{') == std::string::npos) {
        why = "ProviderUrl has a stray '}'";
        return false;
    }
    return true;
}
} // namespace

NavigationService::NavigationService() : impl_(std::make_shared<Impl>()) {}
NavigationService::~NavigationService() { stop(); }

bool NavigationService::start(const NavigationServiceConfig& cfg, dashcam::preview::IPreviewSink& mapSink,
                              dashcam::log::LogCallback log) {
    return start(cfg, mapSink, nullptr, std::move(log));
}

bool NavigationService::start(const NavigationServiceConfig& cfg, dashcam::preview::IPreviewSink& mapSink,
                              IMapPanSink* panSink, dashcam::log::LogCallback log) {
    std::lock_guard<std::mutex> life(lifeMu_);
    const std::shared_ptr<Impl> mp = std::atomic_load(&impl_);
    Impl&                       m  = *mp;
    if (m.run.load()) return true;
    auto bad = [&](const std::string& why) {
        if (log) log(LogLevel::WARN, "navigation: not started - " + why);
        return false;
    };
    std::string why;
    if (!providerUrlOk(cfg.providerUrl, why)) return bad(why);
    if (cfg.userAgent.empty() || cfg.attribution.empty() || cfg.providerId.empty())
        return bad("UserAgent, Attribution and ProviderId must be set");
    if (cfg.zoom < kMinZoom || cfg.zoom > kMaxZoom || cfg.maxFps < 1 || cfg.warmFps < 0 || cfg.fetchWorkers < 1 ||
        cfg.fetchWorkers > 2 || cfg.jpegQuality < 1 || cfg.jpegQuality > 100)
        return bad("zoom/fps/workers/quality out of range");
    if (cfg.poseHz < 1 || cfg.poseHz > 60 || cfg.panJpegQuality < 10 || cfg.panJpegQuality > 100 ||
        !(cfg.panLeadS >= 0.0 && cfg.panLeadS <= 10.0))
        return bad("PoseHz/PanJpegQuality/PanLeadS out of range");
    std::call_once(g_curlInit, [] { curl_global_init(CURL_GLOBAL_DEFAULT); });

    m.cfg     = cfg;
    m.sink    = &mapSink;
    m.panSink = panSink;
    m.pool    = EncodedFramePool::create(detail::kFrameSlots, detail::kFrameSlotBytes);
    {
        std::lock_guard<std::mutex> lk(m.logMu);
        m.log = std::move(log);
    }
    m.stat([](NavigationStats& s) { s = NavigationStats{}; });
    m.health.reset(cfg.maxBackoffMs);
    m.fetchAllowed = false;
    m.putWarned    = false;
    {
        std::lock_guard<std::mutex> lk(m.qMu);
        m.queue.clear();
        m.busy.clear();
        m.negative.clear();
        m.diskMiss.clear();
        m.keep.clear();
        m.warnedStatus.clear();
    }
    {
        std::lock_guard<std::mutex> lk(m.mailMu);
        m.mailNew = m.mailLost = m.tileArrived = false;
    }
    {
        std::lock_guard<std::mutex> lk(m.exitMu);
        m.workersExited = 0;
    }
    m.diskOk = false;
    if (!cfg.cachePath.empty() && !abandoned_.expired()) {
        // A worker left behind by the last stop() may still hold the cache open:
        // a second connection would keep its own counters (and a rebuild would
        // move files under it).  RAM only until it has let go.
        m.logf(LogLevel::WARN, "navigation: the previous run's tile worker still holds the cache - RAM only this run");
    } else if (!cfg.cachePath.empty()) {
        std::string err;
        m.diskOk = m.disk.open(cfg.cachePath, cfg.cacheBudgetBytes, &err);
        if (!m.diskOk) m.logf(LogLevel::WARN, "navigation: tile cache unavailable (%s) - RAM only", err.c_str());
        m.reportCacheNotice();
    }
    m.run = true;
    try {
        m.render = std::thread([mp] { mp->renderLoop(); });
        for (int i = 0; i < cfg.fetchWorkers; ++i) m.workers.emplace_back([mp] { mp->workerLoop(); });
    } catch (const std::system_error& e) {
        m.logf(LogLevel::WARN, "navigation: not started - thread start failed (%s)", e.what());
        {
            std::scoped_lock lk(m.qMu, m.mailMu);
            m.run = false;
        }
        m.qCv.notify_all();
        m.renderCv.notify_all();
        if (m.render.joinable()) m.render.join();
        for (auto& t : m.workers) t.join();
        m.workers.clear();
        m.disk.close();
        m.diskOk = false;
        std::lock_guard<std::mutex> lk(m.logMu);
        m.log = {};
        return false;
    }
    m.logf(LogLevel::INFO, "navigation: map service on (zoom %d, %d fps shown, warm %d fps, cache %s%s)", cfg.zoom,
           cfg.maxFps, cfg.warmFps, m.diskOk ? cfg.cachePath.c_str() : "RAM only",
           panSink ? (", board-side panning at " + std::to_string(cfg.poseHz) + " poses/s if the board has it").c_str()
                   : "");
    return true;
}

void NavigationService::offerGps(const GpsSample& s) {
    const std::shared_ptr<Impl> m = std::atomic_load(&impl_);
    {
        std::lock_guard<std::mutex> lk(m->mailMu);
        m->mail    = s;
        m->mailNew = true;
    }
    m->renderCv.notify_one();
    m->stat([](NavigationStats& st) { ++st.gpsSamples; });
}

void NavigationService::bridgeLost() {
    const std::shared_ptr<Impl> m = std::atomic_load(&impl_);
    {
        std::lock_guard<std::mutex> lk(m->mailMu);
        m->mailLost = true;
    }
    m->renderCv.notify_one();
}

NavigationStats NavigationService::stats() const {
    const std::shared_ptr<Impl> m = std::atomic_load(&impl_);
    NavigationStats             s;
    {
        std::lock_guard<std::mutex> lk(m->statsMu);
        s = m->st;
    }
    if (m->diskOk.load()) {   // lock-free counters: never waits behind cache I/O
        s.cacheBytes     = m->disk.totalBytes();
        s.cacheTiles     = m->disk.tileCount();
        s.cacheEvictions = m->disk.evictions();
    }
    return s;
}

bool NavigationService::running() const { return std::atomic_load(&impl_)->run.load(); }

void NavigationService::stop() {
    std::lock_guard<std::mutex> life(lifeMu_);
    const std::shared_ptr<Impl> mp = std::atomic_load(&impl_);
    Impl&                       m  = *mp;
    if (!m.run.load()) return;
    {   // under the waiters' mutexes: no worker or render wait can miss it
        std::scoped_lock lk(m.qMu, m.mailMu);
        m.run = false;
    }
    m.qCv.notify_all();
    m.renderCv.notify_all();
    // The render thread does no blocking I/O (its waits are 50 ms at most), so
    // it is always joined: once stop() returns, no sink is called again.
    if (m.render.joinable()) m.render.join();
    // A worker may be stuck in a DNS lookup (libcurl waits for its resolver
    // thread) or in disk I/O: wait a bounded time, then leave it behind.
    bool allExited;
    {
        std::unique_lock<std::mutex> lk(m.exitMu);
        allExited = m.exitCv.wait_for(lk, std::chrono::milliseconds(kStopWaitMs),
                                      [&] { return m.workersExited >= m.workers.size(); });
    }
    const NavigationStats s = stats();
    if (allExited) {
        for (auto& t : m.workers)
            if (t.joinable()) t.join();
        m.workers.clear();
        m.disk.close();
        m.diskOk = false;
        m.logf(LogLevel::DEBUG,
               "navigation: stopped (%llu frames offered, %llu backgrounds, %llu HTTP requests, %llu disk hits, "
               "cache %llu tiles; position %s)",
               (unsigned long long)s.framesOffered, (unsigned long long)s.panBackgrounds,
               (unsigned long long)s.httpRequests, (unsigned long long)s.tileDiskHits,
               (unsigned long long)s.cacheTiles, positionStateName(s.position));
        return;
    }
    m.logf(LogLevel::WARN,
           "navigation: a tile worker is still busy (DNS lookup or disk) after %d ms - left to finish on its own",
           kStopWaitMs);
    for (auto& t : m.workers)
        if (t.joinable()) t.detach();   // each holds a reference to the Impl: it stays valid
    m.workers.clear();
    {
        std::lock_guard<std::mutex> lk(m.logMu);
        m.log = {};   // nothing more into the application's log
    }
    // A later start() gets a fresh state; the old one is freed (cache closed)
    // when the last worker lets go of it.
    abandoned_ = mp;   // start() keeps off the cache file while it is alive
    std::atomic_store(&impl_, std::make_shared<Impl>());
}

} // namespace dashcam::navigation
