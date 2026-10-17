// nav_fetch.cpp — NavigationService tile workers: the disk cache, HTTP with
// caching rules, provider health and the request queue.  See nav_service.h.

#include "nav_service.h"

#include <curl/curl.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <ctime>

namespace dashcam::navigation {

using dashcam::log::LogLevel;

namespace {

constexpr int64_t kNegativeTtlS   = 300;       ///< A 404/410 is not asked again for 5 min.
constexpr int64_t kRejectTtlS     = 60;        ///< Nor a tile that was not a valid PNG, or another 4xx.
constexpr int64_t kDiskMissTtlUs  = 2000000;   ///< An unusable disk tile is not re-read for 2 s.
constexpr int64_t kMaxRetryAfterS = 3600;      ///< A larger Retry-After is taken as 1 h.

} // namespace

// ─── helpers ──────────────────────────────────────────────────────────────────

namespace detail {

int64_t steadyUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}
int64_t wallS() { return int64_t(std::time(nullptr)); }

std::string redactUrl(const std::string& url) {
    std::string u = url.substr(0, url.find_first_of("?#"));
    const size_t scheme = u.find("://");
    const size_t start  = scheme == std::string::npos ? 0 : scheme + 3;
    const size_t slash  = u.find('/', start);
    const size_t at     = u.rfind('@', slash == std::string::npos ? u.size() : slash);
    if (at != std::string::npos && at >= start) u.erase(start, at + 1 - start);
    return u;
}

// ─── provider health ──────────────────────────────────────────────────────────

void ProviderHealth::reset(uint32_t maxServerWaitMs) {
    std::lock_guard<std::mutex> lk(m_);
    disabled_    = false;
    maxServerMs_ = std::max<uint32_t>(kFirstWaitMs, maxServerWaitMs);
    serverStepMs_ = unreachableStepMs_ = lastWaitMs_ = 0;
    notBeforeUs_  = 0;
    ++generation_;
    net_ = NetState::Unknown;
}

bool ProviderHealth::allows(int64_t nowUs) const {
    std::lock_guard<std::mutex> lk(m_);
    return !disabled_ && nowUs >= notBeforeUs_;
}

uint64_t ProviderHealth::generation() const {
    std::lock_guard<std::mutex> lk(m_);
    return generation_;
}

bool ProviderHealth::admit(int64_t nowUs, uint64_t& generation) const {
    std::lock_guard<std::mutex> lk(m_);
    generation = generation_;
    return !disabled_ && nowUs >= notBeforeUs_;
}

void ProviderHealth::succeeded(uint64_t generationAtStart) {
    std::lock_guard<std::mutex> lk(m_);
    if (generationAtStart != generation_) return;   // a setback came in meanwhile: it stands
    serverStepMs_ = unreachableStepMs_ = 0;
    notBeforeUs_  = 0;
    if (!disabled_) net_ = NetState::Online;
}

void ProviderHealth::holdOff(int64_t nowUs, uint32_t& stepMs, uint32_t capMs, int64_t minWaitMs) {
    stepMs = stepMs ? std::min<uint32_t>(capMs, stepMs * 2) : std::min<uint32_t>(capMs, kFirstWaitMs);
    const uint32_t jitter = stepMs / 4 ? uint32_t(rng_() % (stepMs / 4)) : 0;
    const int64_t  waitMs = std::max<int64_t>(int64_t(stepMs) + jitter, minWaitMs);
    notBeforeUs_ = std::max(notBeforeUs_, nowUs + waitMs * 1000);   // only ever later
    lastWaitMs_  = uint32_t(std::min<int64_t>(waitMs, UINT32_MAX));
    ++generation_;
}

void ProviderHealth::serverTrouble(int64_t nowUs, int64_t retryAfterS) {
    std::lock_guard<std::mutex> lk(m_);
    holdOff(nowUs, serverStepMs_, maxServerMs_, retryAfterS > 0 ? std::min(retryAfterS, kMaxRetryAfterS) * 1000 : 0);
}

void ProviderHealth::unreachable(int64_t nowUs) {
    std::lock_guard<std::mutex> lk(m_);
    holdOff(nowUs, unreachableStepMs_, kMaxUnreachableMs, 0);
    if (!disabled_) net_ = NetState::Offline;
}

bool ProviderHealth::disable() {
    std::lock_guard<std::mutex> lk(m_);
    const bool first = !disabled_;
    disabled_ = true;
    net_      = NetState::Denied;
    ++generation_;
    return first;
}

NetState ProviderHealth::state() const {
    std::lock_guard<std::mutex> lk(m_);
    return net_;
}

uint32_t ProviderHealth::lastWaitMs() const {
    std::lock_guard<std::mutex> lk(m_);
    return lastWaitMs_;
}

} // namespace detail

using detail::steadyUs;
using detail::wallS;

void NavigationService::Impl::logf(LogLevel lvl, const char* fmt, ...) {
    std::lock_guard<std::mutex> lk(logMu);   // held across the call: stop() can silence it safely
    if (!log) return;
    va_list ap;
    va_start(ap, fmt);
    va_list cp;
    va_copy(cp, ap);
    const int n = std::vsnprintf(nullptr, 0, fmt, cp);
    va_end(cp);
    std::string msg(n > 0 ? size_t(n) : 0, '\0');
    if (n > 0) std::vsnprintf(msg.data(), msg.size() + 1, fmt, ap);
    va_end(ap);
    log(lvl, msg);
}

// ─── RAM tiles ────────────────────────────────────────────────────────────────

const uint8_t* NavigationService::Impl::ramLookup(const TileKey& k) {
    for (RamTile& t : ram)
        if (t.used && t.key == k) {
            t.lastUse = ++useClock;
            return t.rgb.data();
        }
    return nullptr;
}

const NavigationService::Impl::RamTile* NavigationService::Impl::ramFind(const TileKey& k) const {
    for (const RamTile& t : ram)
        if (t.used && t.key == k) return &t;
    return nullptr;
}

void NavigationService::Impl::ramInsert(const TileKey& k, std::vector<uint8_t>&& rgb, bool stale) {
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
        slot->stale   = stale;
        slot->lastUse = ++useClock;
    }
    std::lock_guard<std::mutex> lk(mailMu);
    tileArrived = true;
    renderCv.notify_one();
}

void NavigationService::Impl::ramMarkFresh(const TileKey& k) {
    std::lock_guard<std::mutex> lk(lruMu);
    for (RamTile& t : ram)
        if (t.used && t.key == k) t.stale = false;
}

// ─── request queue ────────────────────────────────────────────────────────────

void NavigationService::Impl::request(const std::vector<TileKey>& view, const std::vector<TileKey>& missing,
                                      bool network) {
    const int64_t nowUs = steadyUs();
    // A disk-only miss is remembered while the server cannot be asked (hidden,
    // backing off, offline or disabled), so unusable rows are not re-read per frame.
    const bool serverAskable = network && fetchAllowed.load() && health.allows(nowUs);
    std::lock_guard<std::mutex> lk(qMu);
    keep = view;
    // Work for tiles no longer in view is obsolete.
    std::set<TileKey, detail::KeyLess> want(view.begin(), view.end());
    const size_t before = queue.size();
    for (auto it = queue.begin(); it != queue.end();) {
        if (!want.count(it->key)) {
            busy.erase(it->key);
            it = queue.erase(it);
        } else {
            it->network = network;   // shown now: may fetch; hidden now: disk only
            ++it;
        }
    }
    if (queue.size() != before) {
        const uint64_t dropped = before - queue.size();
        std::lock_guard<std::mutex> sl(statsMu);
        st.queueDropped += dropped;
    }
    for (auto it = negative.begin(); it != negative.end();) it = it->second <= nowUs ? negative.erase(it) : std::next(it);
    for (auto it = diskMiss.begin(); it != diskMiss.end();) it = it->second <= nowUs ? diskMiss.erase(it) : std::next(it);
    bool queued = false;
    for (const TileKey& k : missing) {
        if (busy.count(k) || negative.count(k) || (!serverAskable && diskMiss.count(k))) continue;
        if (queue.size() >= detail::kQueueDepth) break;
        queue.push_back(Request{k, network});
        busy.insert(k);
        queued = true;
    }
    if (queued) qCv.notify_all();
}

void NavigationService::Impl::downgradeQueue() {
    std::lock_guard<std::mutex> lk(qMu);
    for (Request& r : queue) r.network = false;
}

// ─── HTTP ─────────────────────────────────────────────────────────────────────

namespace {

struct Response {
    CURLcode             rc = CURLE_OK;
    long                 status = 0;
    std::vector<uint8_t> body;
    size_t               cap = 0;
    bool                 tooBig = false;
    uint64_t             generation = 0;   ///< Provider health when the request started.
    std::string          etag, lastModified, cacheControl, expires, date, age, retryAfter, location;
    bool reachedServer() const { return rc == CURLE_OK || (rc == CURLE_WRITE_ERROR && tooBig); }
};

size_t onBody(char* data, size_t size, size_t n, void* user) {
    auto* r = static_cast<Response*>(user);
    const size_t len = size * n;
    if (r->body.size() + len > r->cap) {
        r->tooBig = true;
        return 0;   // abort the transfer
    }
    r->body.insert(r->body.end(), data, data + len);
    return len;
}

std::string trimmed(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

size_t onHeader(char* data, size_t size, size_t n, void* user) {
    auto* r = static_cast<Response*>(user);
    const size_t len = size * n;
    const std::string line(data, len);
    const size_t colon = line.find(':');
    if (colon == std::string::npos) return len;
    std::string name = trimmed(line.substr(0, colon));
    for (char& c : name) c = char(std::tolower(static_cast<unsigned char>(c)));
    const std::string value = trimmed(line.substr(colon + 1));
    if (name == "etag") r->etag = value;
    else if (name == "last-modified") r->lastModified = value;
    else if (name == "cache-control") r->cacheControl = value;
    else if (name == "expires") r->expires = value;
    else if (name == "date") r->date = value;
    else if (name == "age") r->age = value;
    else if (name == "retry-after") r->retryAfter = value;
    else if (name == "location") r->location = value;
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

int64_t ageSeconds(const Response& r) {
    if (r.age.empty() || !std::all_of(r.age.begin(), r.age.end(), [](char c) { return std::isdigit(static_cast<unsigned char>(c)); }))
        return 0;
    return r.age.size() > 12 ? int64_t(1) << 40 : std::atoll(r.age.c_str());
}

TileFreshness freshnessOf(const Response& r, int64_t nowS) {
    return freshnessFromHeaders(parseCacheControl(r.cacheControl), parseHttpDate(r.date), parseHttpDate(r.expires),
                                parseHttpDate(r.lastModified), ageSeconds(r), nowS);
}

/// One GET; conditional when @p validators has an ETag or Last-Modified.
Response fetchTile(CURL* curl, const NavigationServiceConfig& cfg, const std::string& url,
                   const CachedTile* validators, std::atomic<bool>& run) {
    Response r;
    r.cap = cfg.maxTileBytes;
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
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &r);
    curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, onHeader);
    curl_easy_setopt(curl, CURLOPT_HEADERDATA, &r);
    curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0L);
    curl_easy_setopt(curl, CURLOPT_XFERINFOFUNCTION, onProgress);
    curl_easy_setopt(curl, CURLOPT_XFERINFODATA, &run);
    struct curl_slist* hdrs = nullptr;
    if (validators) {   // conditional request: never a blanket no-cache
        if (!validators->etag.empty())
            hdrs = curl_slist_append(hdrs, ("If-None-Match: " + validators->etag).c_str());
        if (!validators->lastModified.empty())
            hdrs = curl_slist_append(hdrs, ("If-Modified-Since: " + validators->lastModified).c_str());
        if (hdrs) curl_easy_setopt(curl, CURLOPT_HTTPHEADER, hdrs);
    }
    r.rc = curl_easy_perform(curl);
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &r.status);
    curl_slist_free_all(hdrs);
    return r;
}

} // namespace

// ─── tile worker ──────────────────────────────────────────────────────────────

void NavigationService::Impl::workerLoop() {
    CURL* curl = curl_easy_init();
    if (!curl) logf(LogLevel::ERROR, "navigation: libcurl init failed - no map downloads");
    while (true) {
        Request rq;
        {
            std::unique_lock<std::mutex> lk(qMu);
            qCv.wait(lk, [&] { return !run.load() || !queue.empty(); });
            if (!run.load()) break;
            rq = queue.front();
            queue.pop_front();
        }
        processTile(curl, rq);
        {
            std::lock_guard<std::mutex> lk(qMu);
            busy.erase(rq.key);
        }
        reportCacheNotice();
        if (!run.load()) break;
    }
    if (curl) curl_easy_cleanup(curl);
    {
        std::lock_guard<std::mutex> lk(exitMu);
        ++workersExited;
    }
    exitCv.notify_all();
}

void NavigationService::Impl::reportCacheNotice() {
    if (!diskOk.load()) return;
    const std::string n = disk.takeNotice();
    if (!n.empty()) logf(LogLevel::WARN, "navigation: %s", n.c_str());
}

/**
 * One tile: show what the disk copy may show now, then ask the server when the
 * map is on screen and the copy is not fresh, and apply the answer.
 *
 *   disk copy        shown at once            asked (when allowed)   on failure
 *   fresh            yes                      no                     -
 *   stale-revalidate yes                      conditional            keep showing
 *   stale-if-error   only if not askable      conditional            show it
 *   expired / none   no                       (conditional) GET      nothing
 */
void NavigationService::Impl::processTile(void* curlHandle, const Request& rq) {
    CURL* const   curl = static_cast<CURL*>(curlHandle);
    const TileKey k    = rq.key;
    const int64_t now  = wallS();
    const bool    sane = clockPlausible(now);

    CachedTile cached;
    bool       onDisk = diskOk.load() && disk.get(cfg.providerId, k, cached, now);
    const TileUse use = onDisk ? classifyTile(cached.fresh, now, sane) : TileUse::Expired;

    std::vector<uint8_t> rgb;
    bool decoded = false, shown = false;
    auto decodeDisk = [&] {
        if (!decoded && onDisk) {
            std::string why;
            decoded = decodeTilePng(cached.png.data(), cached.png.size(), rgb, &why, cfg.maxTileBytes);
            if (!decoded) {
                onDisk = false;   // a corrupt cache entry: refetch it
                logf(LogLevel::DEBUG, "navigation: cached tile %d/%u/%u unusable (%s)", k.z, k.x, k.y, why.c_str());
            }
        }
        return decoded;
    };
    auto showDisk = [&](bool stale) {
        if (shown || !decodeDisk()) return;
        ramInsert(k, std::vector<uint8_t>(rgb), stale);
        shown = true;
        stat([](NavigationStats& s) { ++s.tileDiskHits; });
    };

    const bool mayFetch = rq.network && fetchAllowed.load() && curl && health.allows(steadyUs());
    if (use == TileUse::Fresh || use == TileUse::StaleRevalidate) showDisk(use != TileUse::Fresh);
    else if (use == TileUse::StaleIfError && !mayFetch) showDisk(true);
    if (!mayFetch) {
        if (!shown) {   // nothing usable now: do not re-read it on every render
            std::lock_guard<std::mutex> lk(qMu);
            diskMiss[k] = steadyUs() + kDiskMissTtlUs;
        }
        return;
    }
    if (onDisk && use == TileUse::Fresh) return;

    // Revalidate only a copy that decodes: a 304 must not refresh a corrupt row.
    const bool conditional = onDisk && (!cached.etag.empty() || !cached.lastModified.empty()) && decodeDisk();
    uint64_t generation = 0;
    if (!health.admit(steadyUs(), generation)) {   // a setback came in since mayFetch (the other worker)
        if (onDisk && use == TileUse::StaleIfError) showDisk(true);
        return;
    }
    const std::string url = tileUrl(cfg.providerUrl, k);
    Response r = fetchTile(curl, cfg, url, conditional ? &cached : nullptr, run);
    r.generation = generation;
    stat([&](NavigationStats& s) {
        ++s.httpRequests;
        s.httpBytes += r.body.size();
    });
    if (!run.load()) return;

    const int64_t t      = wallS();
    const int64_t nowUs  = steadyUs();
    auto staleIfError = [&] {   // the server failed us: what the stale windows still allow
        if (onDisk && (use == TileUse::StaleIfError || use == TileUse::StaleRevalidate)) showDisk(true);
    };
    auto markNegative = [&](int64_t ttlS) {   // steady clock: a clock step must not stretch it
        std::lock_guard<std::mutex> lk(qMu);
        negative[k] = nowUs + ttlS * 1000000;
    };
    auto warnOnce = [&](long status, const char* fmt) {
        bool first;
        {
            std::lock_guard<std::mutex> lk(qMu);
            first = warnedStatus.insert(status).second;
        }
        if (first) logf(LogLevel::WARN, fmt, status, detail::redactUrl(url).c_str());
    };
    const long status = r.status;

    if (!r.reachedServer()) {
        stat([](NavigationStats& s) { ++s.httpErrors; });
        logf(LogLevel::DEBUG, "navigation: %s: %s", detail::redactUrl(url).c_str(), curl_easy_strerror(r.rc));
        health.unreachable(nowUs);
        staleIfError();
    } else if (r.tooBig && status == 200) {   // (an oversized error page is judged by its status below)
        stat([](NavigationStats& s) { ++s.httpRejected; });
        markNegative(kRejectTtlS);
        staleIfError();
    } else if (status == 304 && onDisk) {
        stat([](NavigationStats& s) { ++s.http304; });
        health.succeeded(r.generation);
        const TileFreshness f = freshnessAfter304(
            cached.fresh, parseCacheControl(r.cacheControl), !r.cacheControl.empty(), parseHttpDate(r.date),
            parseHttpDate(r.expires), parseHttpDate(r.lastModified.empty() ? cached.lastModified : r.lastModified),
            ageSeconds(r), t);
        if (diskOk.load()) disk.refresh(cfg.providerId, k, f, r.etag, r.lastModified, t);
        if (shown) ramMarkFresh(k);
        else showDisk(false);
    } else if (status == 200) {
        std::vector<uint8_t> fresh;
        std::string          why;
        if (!decodeTilePng(r.body.data(), r.body.size(), fresh, &why, cfg.maxTileBytes)) {
            stat([](NavigationStats& s) { ++s.httpRejected; });
            logf(LogLevel::WARN, "navigation: tile %d/%u/%u rejected (%s)", k.z, k.x, k.y, why.c_str());
            markNegative(kRejectTtlS);
            staleIfError();
        } else {
            stat([](NavigationStats& s) { ++s.http200; });
            health.succeeded(r.generation);
            const CacheControl cc = parseCacheControl(r.cacheControl);
            CachedTile ct;
            ct.png          = std::move(r.body);
            ct.etag         = r.etag;
            ct.lastModified = r.lastModified;
            ct.fresh        = freshnessOf(r, t);
            // Not stored while the clock is implausible: 1970 freshness would make
            // it count as long expired (and evicted first) once the clock is set.
            if (diskOk.load() && !cc.noStore && clockPlausible(t)) {
                if (!disk.put(cfg.providerId, k, ct, t)) {
                    reportCacheNotice();   // a corruption notice says why, once
                    if (!putWarned.exchange(true))
                        logf(LogLevel::WARN, "navigation: tile cache write failed (disk full or cache unusable) - "
                                             "new tiles kept in RAM only");
                } else {
                    std::vector<TileKey> spare;
                    {
                        std::lock_guard<std::mutex> lk(qMu);
                        spare = keep;
                    }
                    disk.evict(cfg.providerId, spare);   // here, not on the render thread: only puts grow it
                }
            }
            ramInsert(k, std::move(fresh), false);
        }
    } else if (status == 404 || status == 410) {
        stat([](NavigationStats& s) { ++s.http404; });
        health.succeeded(r.generation);
        markNegative(kNegativeTtlS);
    } else if (status == 403) {
        stat([](NavigationStats& s) { ++s.http403; });
        if (health.disable())
            logf(LogLevel::ERROR,
                 "navigation: the tile server refused access (HTTP 403) - map downloads off for this run; "
                 "check <Navigation> UserAgent/ProviderUrl and the provider's policy");
        staleIfError();
    } else if (status == 304) {
        // Not modified, but we sent no validators (the disk copy vanished): nothing to apply.
    } else if (status >= 300 && status < 400) {
        stat([](NavigationStats& s) { ++s.httpOther; });
        if (health.disable())
            logf(LogLevel::ERROR,
                 "navigation: the tile server redirects (HTTP %ld to %s) - map downloads off for this run; "
                 "set <Navigation> ProviderUrl to the final URL",
                 status, detail::redactUrl(r.location).c_str());
        staleIfError();
    } else if (status == 429 || status == 408 || status >= 500) {
        stat([&](NavigationStats& s) { status == 429 ? ++s.http429 : ++s.http5xx; });
        health.serverTrouble(nowUs, parseRetryAfter(r.retryAfter, t));
        logf(LogLevel::DEBUG, "navigation: HTTP %ld from the tile server - backing off %u ms", status,
             health.lastWaitMs());
        staleIfError();
    } else if (status == 204 || (status >= 400 && status < 500)) {
        // About this tile, not the server: do not ask for it again for a while.
        stat([](NavigationStats& s) { ++s.httpOther; });
        warnOnce(status, "navigation: HTTP %ld for a tile (%s) - such tiles are not asked again for a minute");
        markNegative(kRejectTtlS);
        staleIfError();
    } else {
        stat([](NavigationStats& s) { ++s.httpErrors; });
        logf(LogLevel::DEBUG, "navigation: unexpected HTTP %ld for %s", status, detail::redactUrl(url).c_str());
        health.serverTrouble(nowUs, -1);
        staleIfError();
    }
}

} // namespace dashcam::navigation
