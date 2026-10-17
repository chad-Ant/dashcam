// nav_render.cpp — the NavigationService render thread: whole frames for a
// board without panning, or board-side panning (background, chrome, poses).
// The only caller of the sinks; never touches the disk or the network.
// See nav_service.h.

#include "nav_service.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace dashcam::navigation {

using dashcam::log::LogLevel;
using dashcam::preview::EncodedFrameLease;
using detail::steadyUs;

namespace {

constexpr int     kStatusRefreshMs   = 1000;     ///< Redraw a shown whole-frame map at least this often.
constexpr int     kWholeFrameRetries = 2;        ///< Each retry encodes 10 quality lower.
constexpr int     kChromeRetries     = 2;
constexpr int     kBackgroundRetries = 3;
constexpr int64_t kPanLookUs         = 1000000;  ///< A background must last this long ahead (make + send + decode).
constexpr int64_t kPanFetchAheadUs   = 3000000;  ///< Ask for the next background's tiles this long before it is due.
constexpr int     kPanEdgePx         = 8;        ///< The map area keeps this much room to the background's border.
constexpr int64_t kPanRetileUs       = 1000000;  ///< Resend an incomplete background at most this often as its tiles arrive.
constexpr int64_t kPanAheadEveryUs   = 1000000;  ///< Look-ahead tile requests at most this often...
constexpr int64_t kPanAheadKeepUs    = 2000000;  ///< ... and they stay wanted this long.
constexpr int64_t kPanRerequestUs    = 1000000;  ///< Ask again for missing or stale background tiles this often.
constexpr int64_t kPanChromeUs       = 500000;   ///< Recompose the chrome this often (sent only when it changed).
constexpr int64_t kPanRetryUs        = 200000;   ///< Between backgrounds (a refused offer, a recentre).

/// A background with almost no margin cannot scroll: recentre at most once a
/// second instead of on every pass.
bool tightBackground(const PanDemand& d) {
    return (d.backgroundW - d.mapW) / 2 < 2 * kPanEdgePx || (d.backgroundH - d.mapH) / 2 < 2 * kPanEdgePx;
}

uint32_t toQ4(double v) {
    return uint32_t(std::clamp<double>(std::llround(v * double(kPoseSubpixels)), 0.0, 4294967295.0));
}

} // namespace

// ─── shared helpers ───────────────────────────────────────────────────────────

std::vector<TileKey> NavigationService::Impl::keysForRect(int64_t ox, int64_t oy, int w, int h) const {
    std::vector<TileKey> keys;
    for (const TilePlacement& t : tilesForView(WorldPoint{double(ox) + w / 2.0, double(oy) + h / 2.0}, cfg.zoom, w, h))
        keys.push_back(t.key);
    return keys;
}

void NavigationService::Impl::collectMissing(const std::vector<TileKey>& keys, std::vector<TileKey>& absent,
                                             std::vector<TileKey>* stale, uint64_t* hits) {
    for (const TileKey& k : keys) {
        const RamTile* t = ramFind(k);
        if (!t) absent.push_back(k);
        else {
            if (hits) ++*hits;
            if (stale && t->stale) stale->push_back(k);
        }
    }
}

NavigationService::Impl::Encoded NavigationService::Impl::encodeLease(const std::vector<uint8_t>& rgb, int w, int h,
                                                                     int q0, int retries, bool chroma420,
                                                                     size_t maxBytes, uint64_t seq,
                                                                     EncodedFrameLease& lease, size_t& outSize,
                                                                     int& quality) {
    outSize = 0;
    quality = 0;
    lease   = pool->tryAcquire();
    if (!lease) return Encoded::NoSlot;
    const size_t cap = std::min<size_t>(lease.capacity(), maxBytes ? maxBytes : lease.capacity());
    for (int k = 0; k <= retries; ++k) {
        const int q = std::max(10, q0 - 10 * k);
        if (encodeMapJpeg(rgb.data(), w, h, q, lease.writableData(), cap, outSize, chroma420)) {
            quality = q;
            lease.setFrame(outSize, uint16_t(w), uint16_t(h), uint64_t(steadyUs()), seq, q);
            return Encoded::Ok;
        }
        quality = q;   // the last quality tried
    }
    outSize = 0;
    lease.release();
    return Encoded::TooLarge;
}

void NavigationService::Impl::publishState(const NavView& v, NetState n, bool shown, bool panning,
                                           const PoseInterpolator& interp) {
    stat([&](NavigationStats& s) {
        s.position             = v.state;
        s.net                  = n;
        s.visible              = shown;
        s.panning              = panning;
        s.interpolationDelayMs = interp.delayUs() / 1000;
    });
}

// ─── the loop ─────────────────────────────────────────────────────────────────

NavigationService::Impl::Mail NavigationService::Impl::takeMail(NavState& nav) {
    Mail m;
    std::lock_guard<std::mutex> lk(mailMu);
    if (mailNew) {
        nav.offer(mail);
        m.sample    = mail;
        m.newSample = true;
        mailNew     = false;
    }
    m.lost      = mailLost;
    m.arrived   = tileArrived;
    mailLost    = false;
    tileArrived = false;
    return m;
}

void NavigationService::Impl::renderLoop() {
    NavState         nav(NavStateConfig{cfg.gpsStaleMs});
    PoseInterpolator interp;
    WholeFrame       wf;
    Pan              pan;
    bool             wasShown = false;
    while (run.load()) {
        const dashcam::preview::PreviewDemand d = sink->demand();
        const PanDemand pd      = panSink ? panSink->panDemand() : PanDemand{};
        const bool      panning = pd.active && pd.backgroundW && pd.backgroundH && pd.mapW && pd.mapH;
        const bool      visible = !panning && d.active && d.canvasW > 0 && d.canvasH > 0;
        const bool      shown   = visible || panning;
        // The network only while the map is on screen; work queued while it was
        // shown becomes disk-only the moment it is hidden.
        fetchAllowed.store(shown);
        if (wasShown && !shown) downgradeQueue();
        wasShown = shown;

        const Mail mail = takeMail(nav);
        if (mail.lost) {
            nav.linkLost();
            interp.reset();
        }
        const int64_t now = steadyUs();
        // The interpolator follows every fix, shown or not, so a panning map
        // starts smooth.  Course only where the arrow may show it.
        if (mail.newSample && usablePosition(mail.sample))
            interp.offer(project(mail.sample.latitude, mail.sample.longitude, cfg.zoom),
                         usableCourse(mail.sample, kMinHeadingSpeedKmh) ? mail.sample.courseDeg : NAN,
                         mail.sample.receivedUs ? mail.sample.receivedUs : uint64_t(now));
        const NavView  v = nav.view(uint64_t(now));
        const NetState n = health.state();

        int64_t waitUs = 50000;
        if (panning) {
            panStep(pan, pd, v, interp, n, now);
            wf.wasVisible = false;   // the whole-frame path starts afresh after panning
            waitUs = pan.lastPoseUs + 1000000 / poseHzFor(pd) - steadyUs();
        } else {
            pan.reshow = true;   // panning again later: background and chrome are sent afresh
            waitUs = wholeFrameStep(wf, d, visible, v, n, mail, now) - steadyUs();
        }
        publishState(v, n, shown, panning, interp);
        // Sleep until the next frame/pose is due, a sample/tile arrives, or the
        // demand may have changed: the sinks are polled at least every 50 ms.
        std::unique_lock<std::mutex> lk(mailMu);
        renderCv.wait_for(lk, std::chrono::microseconds(std::clamp<int64_t>(waitUs, 1000, 50000)), [&] {
            return !run.load() || mailLost || (!panning && (mailNew || (visible && tileArrived)));
        });
    }
    fetchAllowed.store(false);
}

// ─── whole frames ─────────────────────────────────────────────────────────────

int64_t NavigationService::Impl::wholeFramePeriodUs(bool visible) const {
    return visible ? 1000000 / std::max(1, cfg.maxFps) : (cfg.warmFps > 0 ? 1000000 / cfg.warmFps : 1000000);
}

NavigationService::Impl::RenderKey NavigationService::Impl::renderKey(const NavView& v, NetState n) const {
    RenderKey k;
    k.state   = v.state;
    k.marker  = v.marker;
    k.net     = n;
    k.fixType = v.fixType;
    if (v.marker != Marker::None) {
        const WorldPoint p = project(v.latitude, v.longitude, cfg.zoom);
        k.px = std::llround(p.x);
        k.py = std::llround(p.y);
    }
    if (v.marker == Marker::Arrow) k.courseDeg = int(std::lround(v.courseDeg)) % 360;
    k.speedKmh = std::isfinite(v.speedKmh) ? int(std::lround(std::max(0.0f, v.speedKmh))) : -1;
    return k;
}

int64_t NavigationService::Impl::wholeFrameStep(WholeFrame& wf, const dashcam::preview::PreviewDemand& d, bool visible,
                                                const NavView& v, NetState n, const Mail& mail, int64_t now) {
    if (visible && !wf.wasVisible) {
        wf.showUs    = now;
        wf.firstSent = false;
    }
    const bool edge = visible != wf.wasVisible;
    wf.wasVisible   = visible;

    MapLayout L;
    if (d.canvasW && d.canvasH) {
        L.width  = d.canvasW;
        L.height = d.canvasH;
    }
    // A change is kept until a frame shows it (a sample in a pass that is not
    // due is not lost); a repeated bridge sample that changes nothing drawn is
    // not a change.
    const RenderKey key = renderKey(v, n);
    wf.pending = wf.pending || key != wf.lastKey || mail.arrived || edge || mail.lost;
    // Shown just now: draw at once (the last render was a warm one); then at most every period.
    const int64_t period = wholeFramePeriodUs(visible);
    const bool due = (visible && edge) ||
                     (now - wf.lastRenderUs >= period &&
                      (visible ? (wf.pending || now - wf.lastRenderUs >= int64_t(kStatusRefreshMs) * 1000)
                               : cfg.warmFps > 0));
    // When the next frame can be due (the loop also wakes on a sample or a
    // tile, and polls the sinks every 50 ms): never a 1 ms spin while idle.
    auto nextDue = [&] {
        if (!visible) return cfg.warmFps > 0 ? wf.lastRenderUs + period : now + 50000;
        return wf.lastRenderUs + (wf.pending ? period : std::max<int64_t>(period, int64_t(kStatusRefreshMs) * 1000));
    };
    if (!due) return nextDue();
    wf.lastRenderUs = now;
    wf.lastKey      = key;
    wf.pending      = false;

    // Working set: the visible tiles.  While hidden: disk only, never the network.
    std::vector<TileKey> keys;
    if (v.marker != Marker::None)
        for (const TilePlacement& t : tilesForView(project(v.latitude, v.longitude, cfg.zoom), cfg.zoom, L.width,
                                                   L.height - L.statusH - L.attributionH))
            keys.push_back(t.key);
    std::vector<TileKey> ask, stale;
    uint64_t hits = 0;
    const bool askStale = visible && health.allows(now);
    const auto t0 = steadyUs();
    {
        std::lock_guard<std::mutex> lk(lruMu);
        collectMissing(keys, ask, askStale ? &stale : nullptr, &hits);
        composeMap(v, cfg.zoom, L, [&](const TileKey& k) { return ramLookup(k); }, n, cfg.attribution, wf.rgb);
    }
    const uint64_t misses    = ask.size();
    const double   composeMs = double(steadyUs() - t0) / 1000.0;
    ask.insert(ask.end(), stale.begin(), stale.end());
    request(keys, ask, visible);
    if (!visible) {
        stat([&](NavigationStats& s) {
            s.tileRamHits += hits;
            s.tileRamMisses += misses;
            ++s.warmComposes;
            ++s.framesComposed;
            s.lastComposeMs = composeMs;
        });
        return nextDue();
    }
    EncodedFrameLease lease;
    size_t            outSize = 0;
    int               quality = 0;
    const auto        e0      = steadyUs();
    const Encoded     enc = encodeLease(wf.rgb, L.width, L.height, cfg.jpegQuality, kWholeFrameRetries, false,
                                        d.maxJpegBytes, ++wf.seq, lease, outSize, quality);
    const double      encodeMs = double(steadyUs() - e0) / 1000.0;
    const bool        ok       = enc == Encoded::Ok && sink->offerFrame(std::move(lease));
    const bool        first    = ok && !wf.firstSent;
    if (first) wf.firstSent = true;
    stat([&](NavigationStats& s) {
        s.tileRamHits += hits;
        s.tileRamMisses += misses;
        ++s.framesComposed;
        s.lastComposeMs = composeMs;
        if (enc == Encoded::NoSlot) {
            ++s.poolExhausted;
            return;
        }
        if (enc == Encoded::TooLarge) {
            ++s.encodeFailures;
            return;
        }
        ++s.framesEncoded;
        ok ? ++s.framesOffered : ++s.framesRefused;
        s.lastEncodeMs  = encodeMs;
        s.lastJpegBytes = uint32_t(outSize);
        s.lastQuality   = quality;
        if (first) s.firstFrameAfterShowMs = (steadyUs() - wf.showUs) / 1000;
    });
    return nextDue();
}

// ─── board-side panning ───────────────────────────────────────────────────────

int NavigationService::Impl::poseHzFor(const PanDemand& d) const {
    return std::clamp(std::min<int>(cfg.poseHz, d.maxPoseHz ? d.maxPoseHz : 60), 1, 60);
}

void NavigationService::Impl::panStep(Pan& pan, const PanDemand& d, const NavView& v, const PoseInterpolator& interp,
                                      NetState n, int64_t now) {
    if (pan.reshow || d.session != pan.session || d.resend != pan.resend) {
        // A new session, shown again, or an image lost on the way: the board
        // needs a background and a chrome.  Within one session the coordinate
        // frame stays, so poses match any image still on its way.
        const bool   sameSession = d.session == pan.session && pan.session != 0;
        const bool   keepFrame   = sameSession && pan.haveFrame;
        const double frameX      = pan.frameX;
        std::vector<uint8_t> scratch = std::move(pan.rgb);
        pan           = Pan{};
        pan.rgb       = std::move(scratch);
        pan.session   = d.session;
        pan.resend    = d.resend;
        pan.showUs    = now;
        pan.haveFrame = keepFrame;
        pan.frameX    = frameX;
        logf(LogLevel::DEBUG, "navigation: panning map %s (background %ux%u, map area %ux%u)",
             sameSession ? "needs its images again" : "shown", d.backgroundW, d.backgroundH, d.mapW, d.mapH);
    }
    const PanPosition p = panPosition(pan, d, v, interp, now);
    sendPose(pan, d, v, p, now);   // first: its timing must not wait for an encode
    const BgAction a      = decideBackground(pan, d, p, now);
    const int64_t  minGap = tightBackground(d) ? kPanRetileUs : kPanRetryUs;
    if (a != BgAction::None && now - pan.bg.lastUs >= minGap) sendBackground(pan, d, v, p, a, now);
    else if (!prefetchAhead(pan, d, p, now)) rerequestMissing(pan, now);
    sendChrome(pan, d, v, n, now);
}

NavigationService::Impl::PanPosition NavigationService::Impl::panPosition(Pan& pan, const PanDemand& d,
                                                                          const NavView& v,
                                                                          const PoseInterpolator& interp,
                                                                          int64_t now) {
    (void)d;
    PanPosition p;
    p.have   = v.marker != Marker::None;
    p.course = v.marker == Marker::Arrow ? v.courseDeg : NAN;
    if (!p.have) return p;
    // Between the last two fixes while live, the last fix as is otherwise.
    WorldPoint w;
    if (v.state == PositionState::Live && !interp.empty()) {
        float c = NAN;
        w       = interp.at(uint64_t(now), &c);
        p.vel   = interp.velocity();
        if (std::isfinite(c) && v.marker == Marker::Arrow) p.course = c;
    } else {
        w = project(v.latitude, v.longitude, cfg.zoom);
    }
    // One continuous x per session: starting in the half of the world away from
    // the antimeridian seam (x + world when west of the middle), every later
    // position follows by its shortest step, so poses and origins never jump
    // by a world width and stay well inside u32 at 1/16 px.
    const double world = worldSize(cfg.zoom);
    if (!pan.haveFrame) {
        pan.frameX    = w.x + (w.x < world / 2 ? world : 0.0);
        pan.haveFrame = true;
    } else {
        pan.frameX += wrapDx(pan.frameX, w.x, cfg.zoom);
    }
    p.pos = WorldPoint{pan.frameX, w.y};
    return p;
}

NavigationService::Impl::BgAction NavigationService::Impl::decideBackground(const Pan& pan, const PanDemand& d,
                                                                            const PanPosition& p, int64_t now) {
    if (!pan.bg.sent || p.have != pan.bg.hasPos) return BgAction::Recentre;
    if (!p.have) return BgAction::None;
    const double ox = double(pan.bg.ox), oy = double(pan.bg.oy);
    const WorldPoint still{};
    // Out of room now, or within kPanLookUs at this speed (only once the
    // current background has had that long: a fresh one is never replaced for
    // the look-ahead alone, which would churn when the margin is small).
    if (panBackgroundStale(ox, oy, d.backgroundW, d.backgroundH, d.mapW, d.mapH, p.pos, still, 0.0, kPanEdgePx) ||
        (now - pan.bg.lastUs >= kPanLookUs &&
         panBackgroundStale(ox, oy, d.backgroundW, d.backgroundH, d.mapW, d.mapH, p.pos, p.vel,
                            double(kPanLookUs) / 1e6, kPanEdgePx)))
        return BgAction::Recentre;
    // Incomplete: redraw it once a tile it lacked is in RAM (a level, not the
    // one-pass arrival flag, so tiles arriving right after a send still count).
    if (!pan.bg.complete && now - pan.bg.lastUs >= kPanRetileUs) {
        std::lock_guard<std::mutex> lk(lruMu);
        for (const TileKey& k : pan.bg.missing)
            if (ramFind(k)) return BgAction::Retile;
    }
    return BgAction::None;
}

void NavigationService::Impl::sendBackground(Pan& pan, const PanDemand& d, const NavView& v, const PanPosition& p,
                                             BgAction a, int64_t now) {
    int64_t ox = pan.bg.ox, oy = pan.bg.oy;
    if (a == BgAction::Recentre) {
        if (p.have) {
            panBackgroundOrigin(p.pos, p.vel, cfg.panLeadS, d.backgroundW, d.backgroundH, d.mapW, d.mapH, ox, oy,
                                kPanEdgePx);
            const int64_t world = int64_t(worldSize(cfg.zoom));
            ox = std::max<int64_t>(0, ox);
            oy = std::clamp<int64_t>(oy, 0, std::max<int64_t>(0, world - d.backgroundH));
        } else {
            ox = oy = 0;   // the no-position grid; unmarked poses go to its centre
        }
    }
    const auto           t0   = steadyUs();
    std::vector<TileKey> keys = p.have ? keysForRect(ox, oy, d.backgroundW, d.backgroundH) : std::vector<TileKey>{};
    std::vector<TileKey> absent, stale;
    uint64_t             hits = 0;
    ComposeResult        res;
    {
        std::lock_guard<std::mutex> lk(lruMu);
        collectMissing(keys, absent, health.allows(now) ? &stale : nullptr, &hits);
        res = composePanBackground(v, p.have, cfg.zoom, ox, oy, d.backgroundW, d.backgroundH,
                                   [&](const TileKey& k) { return ramLookup(k); }, pan.rgb);
    }
    const uint64_t       misses = absent.size();
    std::vector<TileKey> ask    = absent;
    ask.insert(ask.end(), stale.begin(), stale.end());
    std::vector<TileKey> wanted = keys;   // a recent look-ahead stays queued and spared from eviction
    if (now < pan.aheadUntilUs) wanted.insert(wanted.end(), pan.aheadKeys.begin(), pan.aheadKeys.end());
    request(wanted, ask, true);
    pan.lastRequestUs = now;

    EncodedFrameLease lease;
    size_t            outSize = 0;
    int               quality = 0;
    const Encoded     enc = encodeLease(pan.rgb, d.backgroundW, d.backgroundH, cfg.panJpegQuality,
                                        kBackgroundRetries, true, d.maxJpegBytes, 0, lease, outSize, quality);
    const double      ms  = double(steadyUs() - t0) / 1000.0;
    const bool ok = enc == Encoded::Ok && panSink->offerPanBackground(std::move(lease), uint32_t(ox), uint32_t(oy));
    pan.bg.lastUs = now;
    if (ok) {
        pan.bg.sent     = true;
        pan.bg.hasPos   = p.have;
        pan.bg.complete = !p.have || res.mapComplete;
        pan.bg.ox       = ox;
        pan.bg.oy       = oy;
        pan.bg.keys     = std::move(keys);
        pan.bg.missing  = std::move(absent);
    }
    const bool first = ok && !pan.firstSent;
    if (first) pan.firstSent = true;
    stat([&](NavigationStats& s) {
        s.tileRamHits += hits;
        s.tileRamMisses += misses;
        if (enc == Encoded::NoSlot) ++s.poolExhausted;
        if (enc == Encoded::TooLarge) ++s.encodeFailures;
        ok ? ++s.panBackgrounds : ++s.panBackgroundsRefused;
        s.lastBackgroundBytes = uint32_t(outSize);
        s.lastBackgroundMs    = ms;
        s.lastQuality         = quality;
        if (first) s.firstFrameAfterShowMs = (steadyUs() - pan.showUs) / 1000;
    });
}

bool NavigationService::Impl::prefetchAhead(Pan& pan, const PanDemand& d, const PanPosition& p, int64_t now) {
    if (!p.have || !pan.bg.sent || !pan.bg.hasPos || now - pan.lastAheadUs < kPanAheadEveryUs) return false;
    if (!panBackgroundStale(double(pan.bg.ox), double(pan.bg.oy), d.backgroundW, d.backgroundH, d.mapW, d.mapH, p.pos,
                            p.vel, double(kPanFetchAheadUs) / 1e6, kPanEdgePx))
        return false;
    // The next background is a few seconds away: get its tiles into RAM now
    // (where it will most likely be centred then), so it is complete when sent.
    const double aheadS = double(kPanFetchAheadUs - kPanLookUs) / 1e6;
    int64_t      nx = 0, ny = 0;
    panBackgroundOrigin(WorldPoint{p.pos.x + p.vel.x * aheadS, p.pos.y + p.vel.y * aheadS}, p.vel, cfg.panLeadS,
                        d.backgroundW, d.backgroundH, d.mapW, d.mapH, nx, ny, kPanEdgePx);
    pan.aheadKeys    = keysForRect(std::max<int64_t>(0, nx), std::max<int64_t>(0, ny), d.backgroundW, d.backgroundH);
    pan.aheadUntilUs = now + kPanAheadKeepUs;
    pan.lastAheadUs  = now;
    rerequestMissing(pan, now, true);
    return true;
}

void NavigationService::Impl::rerequestMissing(Pan& pan, int64_t now, bool force) {
    if (!pan.bg.sent || (!force && now - pan.lastRequestUs < kPanRerequestUs)) return;
    // The background's tiles plus a recent look-ahead stay wanted (request()
    // drops queued work outside what it is given).
    std::vector<TileKey> wanted = pan.bg.keys;
    if (now < pan.aheadUntilUs) wanted.insert(wanted.end(), pan.aheadKeys.begin(), pan.aheadKeys.end());
    std::vector<TileKey> ask;
    {
        std::lock_guard<std::mutex> lk(lruMu);
        collectMissing(wanted, ask, health.allows(now) ? &ask : nullptr, nullptr);
    }
    request(wanted, ask, true);
    pan.lastRequestUs = now;
}

void NavigationService::Impl::sendChrome(Pan& pan, const PanDemand& d, const NavView& v, NetState n, int64_t now) {
    const int h = d.chromeTopH + d.chromeBottomH;
    if (h <= 0) return;   // a board without strips
    Pan::Chrome& c     = pan.chrome;
    const bool   never = c.last.empty();
    // Recomposed twice a second, and at once when the marker or GPS state
    // changes so the strip agrees with the marker; sent only when it changed.
    const bool due = (never && now - c.lastUs >= kPanRetryUs) || now - c.lastUs >= kPanChromeUs ||
                     v.marker != c.marker || v.state != c.state;
    if (!due) return;
    c.marker = v.marker;
    c.state  = v.state;
    c.lastUs = now;
    composePanChrome(v, cfg.zoom, d.mapW, d.chromeTopH, d.chromeBottomH, n, pan.bg.complete, cfg.attribution, c.rgb);
    if (!never && c.rgb == c.last) return;
    EncodedFrameLease lease;
    size_t            outSize = 0;
    int               quality = 0;
    const Encoded enc = encodeLease(c.rgb, d.mapW, h, cfg.jpegQuality, kChromeRetries, false, d.maxJpegBytes, 0, lease,
                                    outSize, quality);
    const bool    ok  = enc == Encoded::Ok && panSink->offerPanChrome(std::move(lease));
    if (ok) c.last = c.rgb;
    stat([&](NavigationStats& s) {
        if (enc == Encoded::NoSlot) ++s.poolExhausted;
        if (enc == Encoded::TooLarge) ++s.encodeFailures;
        ok ? ++s.panChrome : ++s.panChromeRefused;
    });
}

void NavigationService::Impl::sendPose(Pan& pan, const PanDemand& d, const NavView& v, const PanPosition& p,
                                       int64_t now) {
    if (now - pan.lastPoseUs < 1000000 / poseHzFor(d)) return;
    pan.lastPoseUs = now;
    // Without a position the board centres an unmarked pose on the grid background.
    const WorldPoint at = p.have ? p.pos
                                 : WorldPoint{double(pan.bg.ox) + d.backgroundW / 2.0,
                                              double(pan.bg.oy) + d.backgroundH / 2.0};
    PanPose pose;
    pose.xQ4    = toQ4(at.x);
    pose.yQ4    = toQ4(at.y);
    pose.marker = v.marker;
    if (std::isfinite(p.course)) {
        const long cd   = std::lround(double(p.course) * 100.0) % 36000;
        pose.courseCdeg = uint16_t(cd < 0 ? cd + 36000 : cd);
    }
    const bool ok = panSink->offerPanPose(pose);
    stat([&](NavigationStats& s) { ok ? ++s.posesOffered : ++s.posesRefused; });
}

} // namespace dashcam::navigation
