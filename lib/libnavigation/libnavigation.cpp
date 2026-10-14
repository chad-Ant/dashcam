// libnavigation.cpp — NavigationService: the long-running map producer (render
// thread, tile workers with disk cache + HTTP, RAM tile LRU).  See libnavigation.h.

#include "libnavigation.h"

#include <curl/curl.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <map>
#include <random>
#include <set>

namespace dashcam::navigation {

using dashcam::log::LogLevel;
using dashcam::preview::EncodedFrameLease;
using dashcam::preview::EncodedFramePool;

namespace {

constexpr size_t   kRamTiles        = 15;     ///< Decoded tiles kept (6 visible + margin).
constexpr size_t   kQueueDepth      = 8;      ///< Distinct tile keys waiting for a worker.
constexpr size_t   kFrameSlots      = 2;
constexpr size_t   kFrameSlotBytes  = 65536;  ///< = the board's JPEG slot.
constexpr int64_t  kNegativeTtlS    = 300;    ///< A 404 is not asked again for 5 min.
constexpr int64_t  kRejectTtlS      = 60;     ///< Neither is a tile that was not a valid PNG.
constexpr int      kStatusRefreshMs = 1000;   ///< Redraw at least this often while shown.
constexpr int      kQualities[]     = {0, 70, 60};   ///< First = cfg.jpegQuality.

int64_t steadyUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}
int64_t wallS() { return int64_t(std::time(nullptr)); }

struct KeyLess {
    bool operator()(const TileKey& a, const TileKey& b) const {
        if (a.z != b.z) return a.z < b.z;
        if (a.x != b.x) return a.x < b.x;
        return a.y < b.y;
    }
};

std::once_flag g_curlInit;

} // namespace

struct NavigationService::Impl {
    NavigationServiceConfig        cfg;
    dashcam::log::LogCallback      log;
    dashcam::preview::IPreviewSink* sink = nullptr;
    std::shared_ptr<EncodedFramePool> pool;
    TileCache                      disk;
    bool                           diskOk = false;

    std::mutex              lifeMu;
    std::atomic<bool>       run{false};
    std::thread             render;
    std::vector<std::thread> workers;

    // GPS mailbox (any thread → render thread).
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
    };
    std::mutex                     lruMu;
    std::array<RamTile, kRamTiles> ram;
    uint64_t                       useClock = 0;

    // Work for the tile workers.
    struct Request {
        TileKey key;
        bool    network = false;
    };
    std::mutex                       qMu;
    std::condition_variable          qCv;
    std::deque<Request>              queue;
    std::set<TileKey, KeyLess>       busy;          ///< Queued or being fetched.
    std::map<TileKey, int64_t, KeyLess> negative;   ///< key -> wall time until which it is not asked.

    // Provider health (workers write, render reads).
    std::atomic<bool>     denied{false};
    std::atomic<int64_t>  backoffUntilUs{0};
    std::atomic<uint32_t> backoffMs{0};
    std::atomic<uint8_t>  net{static_cast<uint8_t>(NetState::Unknown)};
    std::mutex            rngMu;
    std::minstd_rand      rng{uint32_t(steadyUs())};

    mutable std::mutex statsMu;
    NavigationStats    st;

    // ── helpers ──
    void logf(LogLevel lvl, const char* fmt, ...) __attribute__((format(printf, 3, 4))) {
        if (!log) return;
        char    b[512];
        va_list ap;
        va_start(ap, fmt);
        std::vsnprintf(b, sizeof(b), fmt, ap);
        va_end(ap);
        log(lvl, b);
    }
    template <typename F> void stat(F f) {
        std::lock_guard<std::mutex> lk(statsMu);
        f(st);
    }

    const uint8_t* ramLookup(const TileKey& k) {   // lruMu held
        for (RamTile& t : ram)
            if (t.used && t.key == k) {
                t.lastUse = ++useClock;
                return t.rgb.data();
            }
        return nullptr;
    }
    void ramInsert(const TileKey& k, std::vector<uint8_t>&& rgb) {
        {
            std::lock_guard<std::mutex> lk(lruMu);
            RamTile* slot = nullptr;
            for (RamTile& t : ram)
                if (t.used && t.key == k) slot = &t;
            if (!slot)
                for (RamTile& t : ram)
                    if (!t.used) { slot = &t; break; }
            if (!slot) {
                slot = &ram[0];
                for (RamTile& t : ram)
                    if (t.lastUse < slot->lastUse) slot = &t;
            }
            slot->key     = k;
            slot->rgb     = std::move(rgb);
            slot->used    = true;
            slot->lastUse = ++useClock;
        }
        std::lock_guard<std::mutex> lk(mailMu);
        tileArrived = true;
        renderCv.notify_one();
    }

    void renderLoop();
    void workerLoop();
    void request(const std::vector<TileKey>& view, const std::vector<TileKey>& missing, bool network);
    bool providerAllows(int64_t nowUs) const {
        return !denied.load() && nowUs >= backoffUntilUs.load();
    }
    void succeeded() {
        backoffMs = 0;
        backoffUntilUs = 0;
        net = static_cast<uint8_t>(NetState::Online);
    }
    void backOff(int64_t retryAfterS, bool offline) {
        uint32_t next = backoffMs.load() ? std::min<uint32_t>(cfg.maxBackoffMs, backoffMs.load() * 2) : 2000;
        backoffMs     = next;
        uint32_t jitter;
        {
            std::lock_guard<std::mutex> lk(rngMu);
            jitter = next / 4 ? uint32_t(rng() % (next / 4)) : 0;
        }
        int64_t waitMs = int64_t(next) + jitter;
        if (retryAfterS > 0) waitMs = std::max<int64_t>(waitMs, retryAfterS * 1000);
        backoffUntilUs = steadyUs() + waitMs * 1000;
        if (offline) net = static_cast<uint8_t>(NetState::Offline);
    }
};

// ─── HTTP ─────────────────────────────────────────────────────────────────────

namespace {

struct Response {
    long                 status = 0;
    std::vector<uint8_t> body;
    bool                 tooBig = false;
    std::string          etag, lastModified, cacheControl, expires, date, age, retryAfter;
};

size_t onBody(char* data, size_t size, size_t n, void* user) {
    auto* r = static_cast<std::pair<Response*, size_t>*>(user);
    const size_t len = size * n;
    if (r->first->body.size() + len > r->second) {
        r->first->tooBig = true;
        return 0;   // abort the transfer
    }
    r->first->body.insert(r->first->body.end(), data, data + len);
    return len;
}

size_t onHeader(char* data, size_t size, size_t n, void* user) {
    auto* r = static_cast<Response*>(user);
    const size_t len = size * n;
    std::string line(data, len);
    const size_t colon = line.find(':');
    if (colon == std::string::npos) return len;
    std::string name = line.substr(0, colon);
    for (char& c : name) c = char(std::tolower(static_cast<unsigned char>(c)));
    std::string value = line.substr(colon + 1);
    while (!value.empty() && (value.back() == '\r' || value.back() == '\n' || value.back() == ' ')) value.pop_back();
    while (!value.empty() && value.front() == ' ') value.erase(0, 1);
    if (name == "etag") r->etag = value;
    else if (name == "last-modified") r->lastModified = value;
    else if (name == "cache-control") r->cacheControl = value;
    else if (name == "expires") r->expires = value;
    else if (name == "date") r->date = value;
    else if (name == "age") r->age = value;
    else if (name == "retry-after") r->retryAfter = value;
    return len;
}

int onProgress(void* user, curl_off_t, curl_off_t, curl_off_t, curl_off_t) {
    return static_cast<std::atomic<bool>*>(user)->load() ? 0 : 1;   // abort on stop()
}

std::string tileUrl(const std::string& tmpl, const TileKey& k) {
    std::string u = tmpl;
    auto sub = [&](const char* key, const std::string& v) {
        for (size_t p = u.find(key); p != std::string::npos; p = u.find(key, p + v.size())) u.replace(p, 3, v);
    };
    sub("{z}", std::to_string(k.z));
    sub("{x}", std::to_string(k.x));
    sub("{y}", std::to_string(k.y));
    return u;
}

} // namespace

// ─── tile worker ──────────────────────────────────────────────────────────────

void NavigationService::Impl::workerLoop() {
    CURL* curl = curl_easy_init();
    if (!curl) {
        logf(LogLevel::ERROR, "navigation: libcurl init failed - no map downloads");
    }
    while (true) {
        Request rq;
        {
            std::unique_lock<std::mutex> lk(qMu);
            qCv.wait(lk, [&] { return !run.load() || !queue.empty(); });
            if (!run.load()) break;
            rq = queue.front();
            queue.pop_front();
        }
        const TileKey k   = rq.key;
        const int64_t now = wallS();
        const bool    sane = clockPlausible(now);

        CachedTile cached;
        bool       onDisk = diskOk && disk.get(cfg.providerId, k, cached, now);
        std::vector<uint8_t> rgb;
        bool decoded = false;
        if (onDisk) {
            std::string why;
            decoded = decodeTilePng(cached.png.data(), cached.png.size(), rgb, &why, cfg.maxTileBytes);
            if (!decoded) {
                onDisk = false;   // a corrupt cache entry: refetch it
                logf(LogLevel::DEBUG, "navigation: cached tile %d/%u/%u unusable (%s)", k.z, k.x, k.y, why.c_str());
            }
        }
        // An implausible clock cannot judge freshness: show what is cached.
        const bool fresh = decoded && (!sane || now < cached.fresh.freshUntil);
        if (decoded) stat([](NavigationStats& s) { ++s.tileDiskHits; });

        const bool mayFetch = rq.network && curl && providerAllows(steadyUs());
        if (fresh || (decoded && !mayFetch)) {
            // Expired and no request allowed now: only within the server's stale windows.
            const int64_t staleLimit =
                cached.fresh.freshUntil + std::max(cached.fresh.staleIfError, cached.fresh.staleWhileRevalidate);
            if (fresh || !sane || now < staleLimit) ramInsert(k, std::move(rgb));
        } else if (mayFetch) {
            Response                     r;
            std::pair<Response*, size_t> sinkArg{&r, cfg.maxTileBytes};
            const std::string            url = tileUrl(cfg.providerUrl, k);
            curl_easy_reset(curl);
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_USERAGENT, cfg.userAgent.c_str());
            curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
            curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, long(cfg.connectTimeoutMs));
            curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, long(cfg.requestTimeoutMs));
            curl_easy_setopt(curl, CURLOPT_LOW_SPEED_LIMIT, long(cfg.lowSpeedBytesPerS));
            curl_easy_setopt(curl, CURLOPT_LOW_SPEED_TIME, long(cfg.lowSpeedTimeS));
            curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 0L);
            curl_easy_setopt(curl, CURLOPT_PROTOCOLS, long(CURLPROTO_HTTPS | CURLPROTO_HTTP));
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, onBody);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &sinkArg);
            curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, onHeader);
            curl_easy_setopt(curl, CURLOPT_HEADERDATA, &r);
            curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0L);
            curl_easy_setopt(curl, CURLOPT_XFERINFOFUNCTION, onProgress);
            curl_easy_setopt(curl, CURLOPT_XFERINFODATA, &run);
            struct curl_slist* hdrs = nullptr;
            const bool revalidate = decoded && (!cached.etag.empty() || !cached.lastModified.empty());
            if (revalidate) {   // conditional request: never a blanket no-cache
                if (!cached.etag.empty()) hdrs = curl_slist_append(hdrs, ("If-None-Match: " + cached.etag).c_str());
                if (!cached.lastModified.empty())
                    hdrs = curl_slist_append(hdrs, ("If-Modified-Since: " + cached.lastModified).c_str());
                curl_easy_setopt(curl, CURLOPT_HTTPHEADER, hdrs);
            }
            const CURLcode rc = curl_easy_perform(curl);
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &r.status);
            curl_slist_free_all(hdrs);
            stat([&](NavigationStats& s) {
                ++s.httpRequests;
                s.httpBytes += r.body.size();
            });
            if (!run.load()) break;
            const int64_t t = wallS();
            auto staleUsable = [&] {   // the server is unreachable: stale-if-error applies
                return decoded && (!sane || t < cached.fresh.freshUntil + cached.fresh.staleIfError);
            };
            if (rc != CURLE_OK && !(rc == CURLE_WRITE_ERROR && r.tooBig)) {
                stat([](NavigationStats& s) { ++s.httpErrors; });
                logf(LogLevel::DEBUG, "navigation: %s: %s", url.c_str(), curl_easy_strerror(rc));
                backOff(-1, true);
                if (staleUsable()) ramInsert(k, std::move(rgb));
            } else if (r.tooBig) {
                stat([](NavigationStats& s) { ++s.httpRejected; });
                std::lock_guard<std::mutex> lk(qMu);
                negative[k] = t + kRejectTtlS;
            } else if (r.status == 304 && decoded) {
                stat([](NavigationStats& s) { ++s.http304; });
                succeeded();
                const CacheControl cc = parseCacheControl(r.cacheControl);
                const TileFreshness f = freshnessFromHeaders(cc, parseHttpDate(r.date), parseHttpDate(r.expires),
                                                             parseHttpDate(r.lastModified),
                                                             r.age.empty() ? 0 : std::atoll(r.age.c_str()), t);
                if (diskOk) disk.refresh(cfg.providerId, k, f, r.etag, r.lastModified, t);
                ramInsert(k, std::move(rgb));
            } else if (r.status == 200) {
                std::vector<uint8_t> fresh200;
                std::string          why;
                if (!decodeTilePng(r.body.data(), r.body.size(), fresh200, &why, cfg.maxTileBytes)) {
                    stat([](NavigationStats& s) { ++s.httpRejected; });
                    logf(LogLevel::WARN, "navigation: tile %d/%u/%u rejected (%s)", k.z, k.x, k.y, why.c_str());
                    std::lock_guard<std::mutex> lk(qMu);
                    negative[k] = t + kRejectTtlS;
                } else {
                    stat([](NavigationStats& s) { ++s.http200; });
                    succeeded();
                    const CacheControl cc = parseCacheControl(r.cacheControl);
                    CachedTile         ct;
                    ct.png          = std::move(r.body);
                    ct.etag         = r.etag;
                    ct.lastModified = r.lastModified;
                    ct.fresh = freshnessFromHeaders(cc, parseHttpDate(r.date), parseHttpDate(r.expires),
                                                    parseHttpDate(r.lastModified),
                                                    r.age.empty() ? 0 : std::atoll(r.age.c_str()), t);
                    if (diskOk && !cc.noStore && !disk.put(cfg.providerId, k, ct, t))
                        logf(LogLevel::WARN, "navigation: tile cache write failed (disk full?) - RAM only");
                    ramInsert(k, std::move(fresh200));
                }
            } else if (r.status == 404) {
                stat([](NavigationStats& s) { ++s.http404; });
                succeeded();
                std::lock_guard<std::mutex> lk(qMu);
                negative[k] = t + kNegativeTtlS;
            } else if (r.status == 403) {
                stat([](NavigationStats& s) { ++s.http403; });
                if (!denied.exchange(true))
                    logf(LogLevel::ERROR,
                         "navigation: the tile server refused access (HTTP 403) - map downloads off for this run; "
                         "check <Navigation> UserAgent/ProviderUrl and the provider's policy");
                net = static_cast<uint8_t>(NetState::Denied);
                if (staleUsable()) ramInsert(k, std::move(rgb));
            } else if (r.status == 429 || r.status >= 500) {
                stat([&](NavigationStats& s) { r.status == 429 ? ++s.http429 : ++s.http5xx; });
                backOff(parseRetryAfter(r.retryAfter, t), false);
                logf(LogLevel::DEBUG, "navigation: HTTP %ld from the tile server - backing off %u ms", r.status,
                     backoffMs.load());
                if (staleUsable()) ramInsert(k, std::move(rgb));
            } else {
                stat([](NavigationStats& s) { ++s.httpErrors; });
                logf(LogLevel::DEBUG, "navigation: unexpected HTTP %ld for %s", r.status, url.c_str());
                backOff(-1, false);
                if (staleUsable()) ramInsert(k, std::move(rgb));
            }
        }
        std::lock_guard<std::mutex> lk(qMu);
        busy.erase(k);
    }
    if (curl) curl_easy_cleanup(curl);
}

/// Queue the @p missing tiles (not in RAM), skipping negatives and duplicates;
/// drop queued work for tiles no longer in @p view.
void NavigationService::Impl::request(const std::vector<TileKey>& view, const std::vector<TileKey>& missing,
                                      bool network) {
    const int64_t now = wallS();
    std::lock_guard<std::mutex> lk(qMu);
    // Work for tiles no longer in view is obsolete.
    std::set<TileKey, KeyLess> want(view.begin(), view.end());
    const size_t before = queue.size();
    for (auto it = queue.begin(); it != queue.end();) {
        if (!want.count(it->key)) {
            busy.erase(it->key);
            it = queue.erase(it);
        } else {
            if (network) it->network = true;   // shown now: a queued disk-only request may fetch
            ++it;
        }
    }
    if (queue.size() != before) {
        const uint64_t dropped = before - queue.size();
        std::lock_guard<std::mutex> sl(statsMu);
        st.queueDropped += dropped;
    }
    for (auto it = negative.begin(); it != negative.end();)
        it = it->second <= now ? negative.erase(it) : std::next(it);
    for (const TileKey& k : missing) {
        if (busy.count(k) || negative.count(k)) continue;
        if (queue.size() >= kQueueDepth) break;
        queue.push_back(Request{k, network});
        busy.insert(k);
    }
    qCv.notify_all();
}

// ─── render thread ────────────────────────────────────────────────────────────

void NavigationService::Impl::renderLoop() {
    NavState nav(NavStateConfig{cfg.gpsStaleMs, 5.0f});
    std::vector<uint8_t> rgb;
    bool    wasVisible   = false;
    int64_t lastRenderUs = 0, showUs = 0;
    bool    firstSent    = true;
    uint64_t seq = 0;
    PositionState lastState = PositionState::NoData;
    NetState      lastNet   = NetState::Unknown;
    while (run.load()) {
        const dashcam::preview::PreviewDemand d = sink->demand();
        const bool visible = d.active && d.canvasW > 0 && d.canvasH > 0;
        bool newSample = false, lost = false, arrived = false;
        {
            std::lock_guard<std::mutex> lk(mailMu);
            if (mailNew) {
                nav.offer(mail);
                mailNew   = false;
                newSample = true;
            }
            lost        = mailLost;
            mailLost    = false;
            arrived     = tileArrived;
            tileArrived = false;
        }
        if (lost) nav.linkLost();
        const int64_t now = steadyUs();
        const NavView v   = nav.view(uint64_t(now));
        const NetState n  = static_cast<NetState>(net.load());
        if (visible && !wasVisible) {
            showUs    = now;
            firstSent = false;
        }
        const bool shown = visible != wasVisible;
        wasVisible = visible;

        MapLayout L;
        if (d.canvasW && d.canvasH) {
            L.width  = d.canvasW;
            L.height = d.canvasH;
        }
        const int64_t period = visible ? 1000000 / std::max(1, cfg.maxFps)
                                       : (cfg.warmFps > 0 ? 1000000 / cfg.warmFps : 1000000);
        const bool changed = newSample || arrived || shown || lost || v.state != lastState || n != lastNet;
        const bool due     = now - lastRenderUs >= period &&
                         (visible ? (changed || now - lastRenderUs >= kStatusRefreshMs * 1000) : cfg.warmFps > 0);
        if (due) {
            lastRenderUs = now;
            lastState    = v.state;
            lastNet      = n;
            // Working set: the visible tiles.  While hidden: disk only, never the network.
            std::vector<TileKey> keys;
            if (v.marker != Marker::None) {
                const WorldPoint c = project(v.latitude, v.longitude, cfg.zoom);
                for (const TilePlacement& t : tilesForView(c, cfg.zoom, L.width, L.height - L.statusH - L.attributionH))
                    keys.push_back(t.key);
            }
            std::vector<TileKey> missing;
            ComposeResult res;
            const auto t0 = steadyUs();
            {
                std::lock_guard<std::mutex> lk(lruMu);
                uint64_t hits = 0, misses = 0;
                for (const TileKey& k : keys) {
                    if (ramLookup(k)) ++hits;
                    else {
                        ++misses;
                        missing.push_back(k);
                    }
                }
                res = composeMap(v, cfg.zoom, L, [&](const TileKey& k) { return ramLookup(k); }, n, cfg.attribution,
                                 rgb);
                std::lock_guard<std::mutex> sl(statsMu);
                st.tileRamHits += hits;
                st.tileRamMisses += misses;
            }
            const double composeMs = double(steadyUs() - t0) / 1000.0;
            request(keys, missing, visible);
            if (diskOk && !keys.empty()) disk.evict(cfg.providerId, keys);
            if (!visible) {
                stat([&](NavigationStats& s) {
                    ++s.warmComposes;
                    ++s.framesComposed;
                    s.lastComposeMs = composeMs;
                });
            } else {
                EncodedFrameLease lease = pool->tryAcquire();
                if (lease) {
                    const size_t cap = std::min<size_t>(lease.capacity(), d.maxJpegBytes ? d.maxJpegBytes
                                                                                         : lease.capacity());
                    size_t outSize = 0;
                    int    quality = 0;
                    const auto e0  = steadyUs();
                    for (int q : kQualities) {
                        quality = q ? q : cfg.jpegQuality;
                        if (q && q >= cfg.jpegQuality) continue;   // retries only lower the quality
                        if (encodeMapJpeg(rgb.data(), L.width, L.height, quality, lease.writableData(), cap, outSize))
                            break;
                        outSize = 0;
                    }
                    const double encodeMs = double(steadyUs() - e0) / 1000.0;
                    if (outSize == 0) {
                        stat([&](NavigationStats& s) {
                            ++s.encodeFailures;
                            ++s.framesComposed;
                        });
                    } else {
                        lease.setFrame(outSize, uint16_t(L.width), uint16_t(L.height), uint64_t(now), ++seq, quality);
                        const bool ok = sink->offerFrame(std::move(lease));
                        stat([&](NavigationStats& s) {
                            ++s.framesComposed;
                            ++s.framesEncoded;
                            ok ? ++s.framesOffered : ++s.framesRefused;
                            s.lastComposeMs = composeMs;
                            s.lastEncodeMs  = encodeMs;
                            s.lastJpegBytes = uint32_t(outSize);
                            s.lastQuality   = quality;
                            if (ok && !firstSent) s.firstFrameAfterShowMs = (steadyUs() - showUs) / 1000;
                        });
                        if (ok) firstSent = true;
                    }
                }
            }
        }
        stat([&](NavigationStats& s) {
            s.position = v.state;
            s.net      = n;
            s.visible  = visible;
        });
        // Sleep until the next frame is due, a sample/tile arrives, or (shown)
        // the demand may have changed: poll the sink at least every 50 ms.
        std::unique_lock<std::mutex> lk(mailMu);
        const int64_t waitUs = std::clamp<int64_t>(lastRenderUs + period - steadyUs(), 1000, 50000);
        renderCv.wait_for(lk, std::chrono::microseconds(waitUs),
                          [&] { return !run.load() || mailNew || mailLost || (visible && tileArrived); });
    }
}

// ─── public API ───────────────────────────────────────────────────────────────

NavigationService::NavigationService() : impl_(std::make_unique<Impl>()) {}
NavigationService::~NavigationService() { stop(); }

bool NavigationService::start(const NavigationServiceConfig& cfg, dashcam::preview::IPreviewSink& mapSink,
                              dashcam::log::LogCallback log) {
    Impl&                       m = *impl_;
    std::lock_guard<std::mutex> life(m.lifeMu);
    if (m.run.load()) return true;
    auto bad = [&](const char* why) {
        if (log) log(LogLevel::WARN, std::string("navigation: not started - ") + why);
        return false;
    };
    if (cfg.providerUrl.find("{z}") == std::string::npos || cfg.providerUrl.find("{x}") == std::string::npos ||
        cfg.providerUrl.find("{y}") == std::string::npos)
        return bad("ProviderUrl needs {z} {x} {y}");
    if (cfg.userAgent.empty() || cfg.attribution.empty() || cfg.providerId.empty())
        return bad("UserAgent, Attribution and ProviderId must be set");
    if (cfg.zoom < kMinZoom || cfg.zoom > kMaxZoom || cfg.maxFps < 1 || cfg.warmFps < 0 || cfg.fetchWorkers < 1 ||
        cfg.fetchWorkers > 2 || cfg.jpegQuality < 1 || cfg.jpegQuality > 100)
        return bad("zoom/fps/workers/quality out of range");
    std::call_once(g_curlInit, [] { curl_global_init(CURL_GLOBAL_DEFAULT); });
    m.cfg  = cfg;
    m.log  = std::move(log);
    m.sink = &mapSink;
    m.pool = EncodedFramePool::create(kFrameSlots, kFrameSlotBytes);
    m.st   = NavigationStats{};
    m.denied = false;
    m.backoffMs = 0;
    m.backoffUntilUs = 0;
    m.net = static_cast<uint8_t>(NetState::Unknown);
    {
        std::lock_guard<std::mutex> lk(m.qMu);
        m.queue.clear();
        m.busy.clear();
        m.negative.clear();
    }
    m.diskOk = false;
    if (!cfg.cachePath.empty()) {
        std::string why;
        m.diskOk = m.disk.open(cfg.cachePath, cfg.cacheBudgetBytes, &why);
        if (!m.diskOk) m.logf(LogLevel::WARN, "navigation: tile cache unavailable (%s) - RAM only", why.c_str());
    }
    m.run = true;
    try {
        m.render = std::thread([&m] { m.renderLoop(); });
        for (int i = 0; i < cfg.fetchWorkers; ++i) m.workers.emplace_back([&m] { m.workerLoop(); });
    } catch (const std::system_error&) {
        m.run = false;
        m.qCv.notify_all();
        m.renderCv.notify_all();
        if (m.render.joinable()) m.render.join();
        for (auto& t : m.workers) t.join();
        m.workers.clear();
        m.disk.close();
        return bad("thread start failed");
    }
    m.logf(LogLevel::INFO, "navigation: map service on (zoom %d, %d fps shown, warm %d fps, cache %s)", cfg.zoom,
           cfg.maxFps, cfg.warmFps, m.diskOk ? cfg.cachePath.c_str() : "RAM only");
    return true;
}

void NavigationService::offerGps(const GpsSample& s) {
    Impl& m = *impl_;
    {
        std::lock_guard<std::mutex> lk(m.mailMu);
        m.mail    = s;
        m.mailNew = true;
    }
    m.renderCv.notify_one();
    std::lock_guard<std::mutex> lk(m.statsMu);
    ++m.st.gpsSamples;
}

void NavigationService::bridgeLost() {
    Impl& m = *impl_;
    {
        std::lock_guard<std::mutex> lk(m.mailMu);
        m.mailLost = true;
    }
    m.renderCv.notify_one();
}

NavigationStats NavigationService::stats() const {
    const Impl&     m = *impl_;
    NavigationStats s;
    {
        std::lock_guard<std::mutex> lk(m.statsMu);
        s = m.st;
    }
    if (m.diskOk) {
        s.cacheBytes     = m.disk.totalBytes();
        s.cacheTiles     = m.disk.tileCount();
        s.cacheEvictions = m.disk.evictions();
    }
    return s;
}

bool NavigationService::running() const { return impl_->run.load(); }

void NavigationService::stop() {
    Impl&                       m = *impl_;
    std::lock_guard<std::mutex> life(m.lifeMu);
    if (!m.run.exchange(false)) return;
    m.qCv.notify_all();
    m.renderCv.notify_all();
    if (m.render.joinable()) m.render.join();
    for (auto& t : m.workers)
        if (t.joinable()) t.join();   // a transfer in progress aborts through its progress callback
    m.workers.clear();
    m.disk.close();
    m.diskOk = false;
    const NavigationStats s = stats();
    m.logf(LogLevel::DEBUG,
           "navigation: stopped (%llu frames offered, %llu HTTP requests, %llu disk hits, cache %llu tiles)",
           (unsigned long long)s.framesOffered, (unsigned long long)s.httpRequests,
           (unsigned long long)s.tileDiskHits, (unsigned long long)s.cacheTiles);
}

} // namespace dashcam::navigation
