// navigation_test — libnavigation without network or hardware.
//
// Pure parts: Web Mercator (against the independent slippy-map formula),
// tile selection incl. the antimeridian, the GPS freshness/marker state
// machine, PNG validation, HTTP caching rules (Cache-Control, dates,
// Retry-After, freshness), the SQLite tile cache (round trip, 304 refresh,
// LRU eviction that spares the view, persistence), the compositor (strips,
// tiles, marker direction, missing tiles) and the 4:4:4 JPEG encoder.
//
// The service runs against a fake HTTP tile server on 127.0.0.1 and a fake
// display sink: no requests while the map is hidden (OSM policy), warm
// rendering while hidden, User-Agent, conditional revalidation (304), the
// disk cache across restarts, a warm map shown at once, 403 / 429 +
// Retry-After / server down / invalid PNG handling, and a bounded stop()
// while a request hangs.
//
// Board-side panning (map extension v2): the pose interpolator between GPS
// fixes, the background planner, background/chrome composition, and the
// service feeding a fake panning sink (background first, chrome, poses at
// poseHz that move smoothly, a new background before the old one runs out,
// everything resent for a new session, the whole-frame sink idle meanwhile).
//
// Also: NavState boundaries and link loss, tile-use classes and 304 merging,
// provider health (a success never cancels a later setback; unreachable waits
// stay short; a disable sticks), URL redaction, incremental cache accounting,
// LRU on read, rebuild of a corrupt cache, no fetch once the map is hidden,
// stale-while-revalidate shown at once, redirects/4xx, start/stop races, and
// panning edge cases (a background completed while parked, resend requests,
// the antimeridian, a link loss, a background with no room to scroll).
//
// Usage: navigation_test     (NAV_TEST_ARTIFACTS=dir writes sample frames as .jpg)
//        navigation_test --live LAT LON [ZOOM] | --drive OUTDIR | --pan-drive OUTDIR

#include "Es3MapProtocol.h"
#include "libnavigation.h"
#include "nav_service.h"   // internal: ProviderHealth, redactUrl

#include <jpeglib.h>
#include <netinet/in.h>
#include <png.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csetjmp>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using namespace dashcam::navigation;
using dashcam::preview::EncodedFrameLease;
using dashcam::preview::IPreviewSink;
using dashcam::preview::PreviewDemand;

static int g_fails = 0;
static void check(bool ok, const std::string& what) {
    std::printf("  %s  %s\n", ok ? "ok  " : "FAIL", what.c_str());
    if (!ok) ++g_fails;
}
static int64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}
static uint64_t nowUs() {
    return uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count());
}
static void sleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
static bool waitFor(const std::function<bool()>& pred, int ms) {
    const int64_t end = nowMs() + ms;
    while (nowMs() < end) {
        if (pred()) return true;
        sleepMs(5);
    }
    return pred();
}
static bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// ─── fixtures ────────────────────────────────────────────────────────────────

/// A 256x256 PNG of one colour with a darker 1-px border (format: RGB or grey).
static std::vector<uint8_t> makePng(uint8_t r, uint8_t g, uint8_t b, int w = 256, int h = 256, bool grey = false) {
    png_image img;
    std::memset(&img, 0, sizeof(img));
    img.version = PNG_IMAGE_VERSION;
    img.width   = uint32_t(w);
    img.height  = uint32_t(h);
    img.format  = grey ? PNG_FORMAT_GRAY : PNG_FORMAT_RGB;
    const int ch = grey ? 1 : 3;
    std::vector<uint8_t> px(size_t(w) * h * ch);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            uint8_t* p     = &px[(size_t(y) * w + x) * ch];
            const bool edge = x == 0 || y == 0 || x == w - 1 || y == h - 1;
            p[0] = edge ? uint8_t(r / 2) : r;
            if (!grey) {
                p[1] = edge ? uint8_t(g / 2) : g;
                p[2] = edge ? uint8_t(b / 2) : b;
            }
        }
    png_alloc_size_t size = 0;
    png_image_write_to_memory(&img, nullptr, &size, 0, px.data(), 0, nullptr);
    std::vector<uint8_t> out(size);
    png_image_write_to_memory(&img, out.data(), &size, 0, px.data(), 0, nullptr);
    out.resize(size);
    return out;
}

/// Tile colour derived from its key, so a composed frame shows which tile went where.
static void keyColour(const TileKey& k, uint8_t& r, uint8_t& g, uint8_t& b) {
    r = uint8_t(40 + (k.x * 37) % 180);
    g = uint8_t(40 + (k.y * 53) % 180);
    b = uint8_t(60 + (k.z * 11) % 120);
}

struct JpegInfo {
    bool ok = false;
    int  w = 0, h = 0, comps = 0;
    bool progressive = false, subsampled = false;
    std::vector<uint8_t> rgb;
};
struct JErr {
    jpeg_error_mgr pub;
    jmp_buf        jb;
};
static void jErrExit(j_common_ptr c) { std::longjmp(reinterpret_cast<JErr*>(c->err)->jb, 1); }
static void jQuiet(j_common_ptr) {}
static JpegInfo decodeJpeg(const uint8_t* d, size_t n) {
    JpegInfo               info;
    jpeg_decompress_struct c;
    JErr                   e;
    c.err                  = jpeg_std_error(&e.pub);
    e.pub.error_exit       = jErrExit;
    e.pub.output_message   = jQuiet;
    if (setjmp(e.jb)) {
        jpeg_destroy_decompress(&c);
        return JpegInfo{};
    }
    jpeg_create_decompress(&c);
    jpeg_mem_src(&c, d, static_cast<unsigned long>(n));
    jpeg_read_header(&c, TRUE);
    info.progressive = c.progressive_mode;
    info.subsampled  = c.comp_info[0].h_samp_factor != 1 || c.comp_info[0].v_samp_factor != 1;
    c.out_color_space = JCS_RGB;
    jpeg_start_decompress(&c);
    info.w     = int(c.output_width);
    info.h     = int(c.output_height);
    info.comps = c.output_components;
    info.rgb.resize(size_t(info.w) * info.h * 3);
    while (c.output_scanline < c.output_height) {
        JSAMPROW row = &info.rgb[size_t(c.output_scanline) * info.w * 3];
        jpeg_read_scanlines(&c, &row, 1);
    }
    jpeg_finish_decompress(&c);
    jpeg_destroy_decompress(&c);
    info.ok = true;
    return info;
}

static GpsSample fix(double lat, double lon, float speed, float course, uint64_t at = 0) {
    GpsSample s;
    s.latitude   = lat;
    s.longitude  = lon;
    s.speedKmh   = speed;
    s.courseDeg  = course;
    s.fixType    = 3;
    s.satellites = 9;
    s.fixValid   = true;
    s.gpsFix     = true;
    s.gpsPresent = true;
    s.receivedUs = at ? at : nowUs();
    return s;
}

// ─── fake HTTP tile server ───────────────────────────────────────────────────

class TileServer {
public:
    struct Seen {
        std::string path, userAgent, ifNoneMatch;
    };
    std::atomic<int>  status{200};        ///< Forced status for tile requests (200 = normal).
    std::atomic<int>  maxAge{3600};
    std::atomic<int>  retryAfter{0};
    std::atomic<int>  delayMs{0};
    std::atomic<bool> html{false};        ///< Serve an HTML error page with 200.
    std::atomic<int>  swr{0};             ///< stale-while-revalidate sent with 200 (0 = none).
    std::atomic<int>  sie{600};           ///< stale-if-error sent with 200.
    std::atomic<bool> wrongSize{false};   ///< Serve a 255x256 PNG.

    bool start() {
        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd_ < 0) return false;
        int one = 1;
        ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in a{};
        a.sin_family      = AF_INET;
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        a.sin_port        = 0;
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0 || ::listen(fd_, 16) != 0) return false;
        socklen_t len = sizeof(a);
        ::getsockname(fd_, reinterpret_cast<sockaddr*>(&a), &len);
        port_ = ntohs(a.sin_port);
        run_  = true;
        thr_  = std::thread([this] { loop(); });
        return true;
    }
    void stop() {
        run_ = false;
        if (fd_ >= 0) ::shutdown(fd_, SHUT_RDWR);
        if (thr_.joinable()) thr_.join();
        for (auto& t : conns_)
            if (t.joinable()) t.join();
        conns_.clear();
        if (fd_ >= 0) ::close(fd_);
        fd_ = -1;
    }
    ~TileServer() { stop(); }
    std::string url() const { return "http://127.0.0.1:" + std::to_string(port_) + "/t/{z}/{x}/{y}.png"; }
    int requests() {
        std::lock_guard<std::mutex> lk(m_);
        return int(seen_.size());
    }
    std::vector<Seen> seen() {
        std::lock_guard<std::mutex> lk(m_);
        return seen_;
    }
    void clear() {
        std::lock_guard<std::mutex> lk(m_);
        seen_.clear();
    }

private:
    void loop() {
        while (run_) {
            const int c = ::accept(fd_, nullptr, nullptr);
            if (c < 0) break;
            conns_.emplace_back([this, c] { serve(c); });
        }
    }
    void serve(int c) {
        std::string req;
        char        buf[4096];
        while (req.find("\r\n\r\n") == std::string::npos) {
            const ssize_t n = ::recv(c, buf, sizeof(buf), 0);
            if (n <= 0) {
                ::close(c);
                return;
            }
            req.append(buf, size_t(n));
        }
        auto header = [&](const char* name) {
            std::string lower = req;
            for (char& ch : lower) ch = char(std::tolower(static_cast<unsigned char>(ch)));
            std::string key = std::string("\r\n") + name + ":";
            const size_t p  = lower.find(key);
            if (p == std::string::npos) return std::string();
            const size_t s = p + key.size(), e = req.find("\r\n", s);
            std::string v  = req.substr(s, e - s);
            while (!v.empty() && v.front() == ' ') v.erase(0, 1);
            return v;
        };
        Seen s;
        const size_t sp1 = req.find(' '), sp2 = req.find(' ', sp1 + 1);
        s.path        = req.substr(sp1 + 1, sp2 - sp1 - 1);
        s.userAgent   = header("user-agent");
        s.ifNoneMatch = header("if-none-match");
        {
            std::lock_guard<std::mutex> lk(m_);
            seen_.push_back(s);
        }
        for (int i = 0; i < delayMs.load() / 10 && run_; ++i) sleepMs(10);
        int z = 0;
        unsigned x = 0, y = 0;
        std::sscanf(s.path.c_str(), "/t/%d/%u/%u.png", &z, &x, &y);
        const std::string etag = "\"t" + std::to_string(z) + "-" + std::to_string(x) + "-" + std::to_string(y) + "\"";
        std::string head, body;
        const int forced = status.load();
        if (forced != 200) {
            head = "HTTP/1.1 " + std::to_string(forced) + " X\r\n";
            if (retryAfter.load()) head += "Retry-After: " + std::to_string(retryAfter.load()) + "\r\n";
            if (forced >= 300 && forced < 400) head += "Location: https://elsewhere.example/t/1/2/3.png?key=SECRET\r\n";
        } else if (!s.ifNoneMatch.empty() && s.ifNoneMatch == etag) {
            head = "HTTP/1.1 304 Not Modified\r\nETag: " + etag + "\r\nCache-Control: max-age=" +
                   std::to_string(maxAge.load()) + "\r\n";
        } else if (html.load()) {
            head = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n";
            body = "<html><body>Access blocked</body></html>";
        } else {
            TileKey k{z, x, y};
            uint8_t r, g, b;
            keyColour(k, r, g, b);
            const auto png = makePng(r, g, b, wrongSize.load() ? 255 : 256, 256);
            body.assign(png.begin(), png.end());
            head = "HTTP/1.1 200 OK\r\nContent-Type: image/png\r\nETag: " + etag + "\r\nCache-Control: max-age=" +
                   std::to_string(maxAge.load()) + ", stale-if-error=" + std::to_string(sie.load()) +
                   (swr.load() ? ", stale-while-revalidate=" + std::to_string(swr.load()) : std::string()) + "\r\n";
        }
        head += "Content-Length: " + std::to_string(body.size()) + "\r\nConnection: close\r\n\r\n";
        const std::string all = head + body;
        size_t off = 0;
        while (off < all.size()) {
            const ssize_t n = ::send(c, all.data() + off, all.size() - off, MSG_NOSIGNAL);
            if (n <= 0) break;
            off += size_t(n);
        }
        ::close(c);
    }

    int                      fd_ = -1;
    uint16_t                 port_ = 0;
    std::atomic<bool>        run_{false};
    std::thread              thr_;
    std::vector<std::thread> conns_;
    std::mutex               m_;
    std::vector<Seen>        seen_;
};

// ─── fake display sink ───────────────────────────────────────────────────────

class FakeSink : public IPreviewSink {
public:
    std::atomic<bool> active{false};
    std::atomic<bool> refuse{false};
    PreviewDemand demand() const override {
        PreviewDemand d;
        d.active       = active.load();
        d.canvasW      = 320;
        d.canvasH      = 224;
        d.maxJpegBytes = 65536;
        return d;
    }
    bool offerFrame(EncodedFrameLease&& f) override {
        if (refuse.load()) return false;
        std::lock_guard<std::mutex> lk(m_);
        last_.assign(f.data(), f.data() + f.size());
        lastW_ = f.width();
        lastH_ = f.height();
        ++frames_;
        f.release();
        return true;
    }
    int frames() {
        std::lock_guard<std::mutex> lk(m_);
        return frames_;
    }
    std::vector<uint8_t> last(int* w = nullptr, int* h = nullptr) {
        std::lock_guard<std::mutex> lk(m_);
        if (w) *w = lastW_;
        if (h) *h = lastH_;
        return last_;
    }

private:
    std::mutex           m_;
    std::vector<uint8_t> last_;
    int                  frames_ = 0, lastW_ = 0, lastH_ = 0;
};

static void saveArtifact(const char* name, const std::vector<uint8_t>& jpeg) {
    const char* dir = std::getenv("NAV_TEST_ARTIFACTS");
    if (!dir || jpeg.empty()) return;
    std::ofstream(std::string(dir) + "/" + name, std::ios::binary)
        .write(reinterpret_cast<const char*>(jpeg.data()), std::streamsize(jpeg.size()));
}

// ─── tests ───────────────────────────────────────────────────────────────────

static void testMercator() {
    std::printf("\n[1] Web Mercator and tile selection\n");
    const WorldPoint o = project(0, 0, 0);
    check(near(o.x, 128, 1e-9) && near(o.y, 128, 1e-9), "(0,0) at zoom 0 is the world centre (128,128)");
    check(near(project(kMaxLatitude, 0, 0).y, 0, 1e-6) && near(project(-89, 0, 0).y, 256, 1e-6),
          "latitude clamps to the projection's top/bottom");
    check(near(project(0, 180, 3).x, 0, 1e-9) && near(project(0, -180, 3).x, 0, 1e-9), "lon +-180 wrap to x = 0");
    // Independent slippy-map formula (OSM wiki): ln(tan + sec), not asinh(tan).
    const struct { double lat, lon; int z; } pts[] = {{51.5074, -0.1278, 16}, {10.7769, 106.7009, 16},
                                                       {-33.8688, 151.2093, 14}, {64.1466, -21.9426, 18}};
    bool allOk = true;
    for (const auto& p : pts) {
        const double n  = std::ldexp(1.0, p.z);
        const double lr = p.lat * M_PI / 180.0;
        const int    tx = int(std::floor((p.lon + 180.0) / 360.0 * n));
        const int    ty = int(std::floor((1.0 - std::log(std::tan(lr) + 1.0 / std::cos(lr)) / M_PI) / 2.0 * n));
        const WorldPoint w = project(p.lat, p.lon, p.z);
        allOk = allOk && int(w.x / 256) == tx && int(w.y / 256) == ty;
    }
    check(allOk, "tile indexes match the slippy-map formula (London, HCMC, Sydney, Reykjavik)");
    check(near(metersPerPixel(0, 16), 2.3887, 1e-3) && near(metersPerPixel(60, 16), 1.1943, 1e-3),
          "metres per pixel: 2.389 at the equator, half at 60 deg (zoom 16)");
    check(near(wrapDx(worldSize(4) - 10, 10, 4), 20, 1e-9) && near(wrapDx(10, worldSize(4) - 10, 4), -20, 1e-9),
          "shortest x difference across the antimeridian");

    const WorldPoint mid = project(10.7769, 106.7009, 16);
    const auto view = tilesForView(mid, 16, 320, 192);
    check(view.size() >= 2 && view.size() <= 6, std::to_string(view.size()) + " tiles cover a 320x192 view (<= 6)");
    bool covers = true;
    for (int y = 0; y < 192; y += 191)
        for (int x = 0; x < 320; x += 319) {
            bool hit = false;
            for (const auto& t : view)
                hit = hit || (x >= t.dstX && x < t.dstX + 256 && y >= t.dstY && y < t.dstY + 256);
            covers = covers && hit;
        }
    check(covers, "the four view corners are covered");
    const auto dl = tilesForView(project(0.001, 179.9995, 12), 12, 320, 192);
    bool wrapZero = false, wrapMax = false;
    for (const auto& t : dl) {
        wrapZero = wrapZero || t.key.x == 0;
        wrapMax  = wrapMax || t.key.x == 4095;
    }
    check(wrapZero && wrapMax, "a view on the antimeridian uses tiles x = 4095 and x = 0");
    check(tilesForView(project(85.05, 0, 3), 3, 320, 192).size() <= 2, "no tiles above the world's top edge");
}

static void testNavState() {
    std::printf("\n[2] GPS freshness and marker\n");
    NavState n;
    const uint64_t t = nowUs();
    check(n.view(t).state == PositionState::NoData && n.view(t).marker == Marker::None, "nothing yet: no data");
    GpsSample s;
    s.receivedUs = t;
    n.offer(s);
    check(n.view(t).state == PositionState::NoReceiver, "telemetry without GPS_PRESENT: no receiver");
    s.gpsPresent = true;
    n.offer(s);
    check(n.view(t).state == PositionState::Acquiring, "receiver, no fix: acquiring");
    n.offer(fix(10.7, 106.7, 3.0f, 45.0f, t));
    NavView v = n.view(t);
    check(v.state == PositionState::Live && v.marker == Marker::Dot, "fix at 3 km/h: live dot (below 5 km/h)");
    n.offer(fix(10.7, 106.7, 30.0f, 90.0f, t));
    v = n.view(t);
    check(v.marker == Marker::Arrow && v.courseDeg == 90.0f, "fix at 30 km/h with a course: arrow at 90");
    n.offer(fix(10.7, 106.7, 30.0f, NAN, t));
    check(n.view(t).marker == Marker::Dot, "NaN course: dot, never the previous arrow");
    n.offer(fix(10.7, 106.7, NAN, 90.0f, t));
    check(n.view(t).marker == Marker::Dot, "unknown speed: dot");
    n.offer(fix(10.7, 106.7, 30.0f, 360.0f, t));
    check(n.view(t).marker == Marker::Arrow && n.view(t).courseDeg == 0.0f, "course 360 is drawn as 0");
    n.offer(fix(10.7, 106.7, 30.0f, 400.0f, t));
    check(n.view(t).marker == Marker::Dot, "course out of range: dot");
    check(n.view(t + 1600000).state == PositionState::LastKnown &&
              n.view(t + 1600000).marker == Marker::LastKnown && std::isnan(n.view(t + 1600000).speedKmh),
          "1.6 s without a sample: last known, grey, no speed shown");
    GpsSample bad = fix(10.8, 106.8, 30.0f, 90.0f, t);
    bad.fixValid  = false;
    n.offer(bad);
    v = n.view(t);
    check(v.state == PositionState::LastKnown && near(v.latitude, 10.7, 1e-9),
          "an invalid fix right after a good one: last known at once, at the last good position");
    n.offer(fix(10.7, 106.7, 30.0f, 90.0f, t));
    n.linkLost();
    check(n.view(t).state == PositionState::LastKnown, "bridge lost: last known");
    n.offer(fix(10.7, 106.7, 30.0f, 90.0f, t));
    check(n.view(t).state == PositionState::Live, "a sample after the link loss: live again");
    {
        NavState nl;
        GpsSample tel;
        tel.gpsPresent = true;
        tel.receivedUs = t;
        nl.offer(tel);
        nl.linkLost();
        check(nl.view(t).state == PositionState::NoData, "link lost before any fix: no data (no invented position)");
        NavState nr;
        GpsSample q;
        q.receivedUs = t;
        nr.offer(q);
        check(nr.view(t + 1600000).state == PositionState::NoData, "a stale 'no receiver' sample: no data");
        NavState b;
        b.offer(fix(10.7, 106.7, 30.0f, 90.0f, t));
        check(b.view(t + 1500000 - 1).state == PositionState::Live && b.view(t + 1500000).state == PositionState::LastKnown,
              "live until exactly gpsStaleMs, last known from then on");
        b.offer(fix(10.7, 106.7, 5.0f, 90.0f, t));
        const bool at5 = b.view(t).marker == Marker::Arrow;
        b.offer(fix(10.7, 106.7, 4.99f, 90.0f, t));
        check(at5 && b.view(t).marker == Marker::Dot, "heading gate: arrow at 5.0 km/h, dot at 4.99");
    }
    NavState z;
    z.offer(fix(0.0, 0.0, 0.0f, NAN, t));
    check(z.view(t).state == PositionState::Live, "(0, 0) is a legitimate position");
    GpsSample out = fix(91.0, 0.0, 0.0f, NAN, t);
    check(!usablePosition(out), "latitude 91 rejected");
    GpsSample noType = fix(10, 10, 0, NAN, t);
    noType.fixType = 0;
    check(!usablePosition(noType), "fix type 0 with fix flags: rejected");
    GpsSample dr = fix(10, 10, 0, NAN, t);
    dr.fixType = 4;
    check(usablePosition(dr), "GNSS+DR (type 4) accepted");
}

static void testPngAndHttpRules() {
    std::printf("\n[3] tile PNG validation and HTTP caching rules\n");
    std::vector<uint8_t> rgb;
    std::string          why;
    check(decodeTilePng(makePng(10, 20, 30).data(), makePng(10, 20, 30).size(), rgb, &why) &&
              rgb.size() == kTileRgbBytes && rgb[(128 * 256 + 128) * 3] == 10 && rgb[(128 * 256 + 128) * 3 + 2] == 30,
          "256x256 RGB PNG decodes");
    const auto grey = makePng(77, 0, 0, 256, 256, true);
    check(decodeTilePng(grey.data(), grey.size(), rgb, &why) && rgb[(5 * 256 + 5) * 3 + 1] == 77,
          "a grey PNG is converted to RGB");
    const auto small = makePng(1, 2, 3, 255, 256);
    check(!decodeTilePng(small.data(), small.size(), rgb, &why) && why.find("255x256") != std::string::npos,
          "255x256 rejected before decoding (" + why + ")");
    const std::string html = "<html><body>blocked</body></html>                   ";
    check(!decodeTilePng(reinterpret_cast<const uint8_t*>(html.data()), html.size(), rgb, &why),
          "an HTML error page is rejected");
    auto png = makePng(1, 2, 3);
    check(!decodeTilePng(png.data(), png.size(), rgb, &why, 100), "over the byte cap: rejected");
    png.resize(png.size() / 2);
    check(!decodeTilePng(png.data(), png.size(), rgb, &why), "a truncated PNG is rejected");

    const CacheControl cc = parseCacheControl("max-age=13784, stale-while-revalidate=604800, stale-if-error=604800");
    check(cc.maxAge == 13784 && cc.staleWhileRevalidate == 604800 && cc.staleIfError == 604800,
          "OSM-style Cache-Control parsed");
    const CacheControl nc = parseCacheControl("no-cache, NO-STORE, max-age=\"60\"");
    check(nc.noCache && nc.noStore && nc.maxAge == 60, "case, quotes and flags");
    check(parseHttpDate("Sun, 06 Nov 1994 08:49:37 GMT") == 784111777 &&
              parseHttpDate("Sunday, 06-Nov-94 08:49:37 GMT") == 784111777 &&
              parseHttpDate("Sun Nov  6 08:49:37 1994") == 784111777 && parseHttpDate("yesterday") == -1,
          "HTTP dates: IMF-fixdate, RFC 850, asctime, junk");
    check(parseRetryAfter("120", 1000) == 120 && parseRetryAfter("Sun, 06 Nov 1994 08:51:37 GMT", 784111777) == 120 &&
              parseRetryAfter("", 0) == -1,
          "Retry-After as seconds and as a date");
    const int64_t now = 1760000000;
    TileFreshness f = freshnessFromHeaders(cc, now, -1, -1, 100, now);
    check(f.freshUntil == now + 13684 && f.staleIfError == 604800, "max-age minus Age");
    f = freshnessFromHeaders(CacheControl{}, now, now + 600, -1, 0, now);
    check(f.freshUntil == now + 600, "Expires relative to Date");
    f = freshnessFromHeaders(CacheControl{}, now, -1, now - 100000, 0, now);
    check(f.freshUntil == now + 10000, "heuristic: 10% of the Last-Modified age");
    f = freshnessFromHeaders(CacheControl{}, now, -1, now - 100000000, 0, now);
    check(f.freshUntil == now + 86400, "heuristic capped at one day");
    f = freshnessFromHeaders(nc, now, -1, -1, 0, now);
    check(f.freshUntil == now, "no-cache: must revalidate");
    check(!clockPlausible(86400 * 365) && clockPlausible(now), "a 1971 clock is not plausible, 2025+ is");
}

static void testRulesAndHealth() {
    std::printf("\n[3b] tile use, 304 merge, provider health, URL redaction\n");
    TileFreshness f;
    f.fetchedAt = 1000, f.freshUntil = 2000, f.staleWhileRevalidate = 100, f.staleIfError = 500;
    check(classifyTile(f, 1999, true) == TileUse::Fresh && classifyTile(f, 2050, true) == TileUse::StaleRevalidate &&
              classifyTile(f, 2300, true) == TileUse::StaleIfError && classifyTile(f, 2600, true) == TileUse::Expired,
          "fresh / stale-while-revalidate / stale-if-error / expired");
    check(classifyTile(f, 999999, false) == TileUse::Fresh, "an implausible clock cannot age a tile: shown as is");
    TileFreshness stored;
    stored.fetchedAt = 1000, stored.freshUntil = 4600, stored.staleWhileRevalidate = 50, stored.staleIfError = 600;
    TileFreshness a = freshnessAfter304(stored, CacheControl{}, false, -1, -1, -1, 0, 9000);
    check(a.freshUntil == 12600 && a.staleWhileRevalidate == 50 && a.staleIfError == 600 && a.fetchedAt == 9000,
          "a 304 without Cache-Control/Expires keeps the stored lifetime and stale windows");
    a = freshnessAfter304(stored, parseCacheControl("max-age=60"), true, -1, -1, -1, 0, 9000);
    check(a.freshUntil == 9060 && a.staleIfError == 0, "a 304 with Cache-Control uses its own");
    check(detail::redactUrl("https://user:pw@tiles.example.com/{z}/{x}/{y}.png?key=SECRET#f") ==
              "https://tiles.example.com/{z}/{x}/{y}.png",
          "URLs are logged without credentials or query");
    check(detail::redactUrl("http://127.0.0.1:8080/t/1/2/3.png") == "http://127.0.0.1:8080/t/1/2/3.png",
          "a plain URL is unchanged");

    detail::ProviderHealth h;
    h.reset(60000);
    check(h.allows(0) && h.state() == NetState::Unknown, "fresh health allows requests");
    const int64_t t0 = 1000000;
    const uint64_t before = h.generation();   // a 200 that started before the 429...
    h.serverTrouble(t0, 30);
    check(!h.allows(t0 + 29900000) && h.allows(t0 + 30000000), "429 Retry-After: 30 honoured");
    h.succeeded(before);
    check(!h.allows(t0 + 29900000), "... and its success does not cancel the Retry-After");
    h.succeeded(h.generation());
    check(h.allows(t0) && h.state() == NetState::Online, "a success that started after it clears the wait");
    h.reset(60000);
    for (int i = 0; i < 12; ++i) h.unreachable(t0);
    check(h.allows(t0 + 12500000) && h.state() == NetState::Offline,
          "no HTTP response 12 times: still retried within 12.5 s (cap 10 s + jitter)");
    h.reset(60000);
    h.serverTrouble(t0, 30);
    const uint64_t g = h.generation();
    h.unreachable(t0);   // a later, shorter setback never shortens a wait
    check(!h.allows(t0 + 29900000), "waits only ever get later (a 2 s unreachable wait keeps the 30 s Retry-After)");
    check(h.disable() && !h.disable(), "disable reports the first time only");
    h.succeeded(h.generation());
    check(h.state() == NetState::Denied && !h.allows(INT64_MAX / 2), "a success never lifts a disable");
    (void)g;
}

static void testTileCache() {
    std::printf("\n[4] SQLite tile cache\n");
    char              tmpl[] = "/tmp/navigation_test_cache_XXXXXX";
    const char*       made   = ::mkdtemp(tmpl);
    const std::string dir    = made ? made : "/tmp/navigation_test_cache";
    std::error_code   ec;
    const std::string path = dir + "/sub/tiles.sqlite";
    TileCache c;
    std::string why;
    check(c.open(path, 6 * 1024, &why), "opens and creates the directory (" + why + ")");
    CachedTile t;
    t.png          = makePng(5, 6, 7);
    t.etag         = "\"abc\"";
    t.lastModified = "Sun, 06 Nov 1994 08:49:37 GMT";
    t.fresh.fetchedAt  = 1760000000;
    t.fresh.freshUntil = 1760003600;
    t.fresh.staleIfError = 600;
    const TileKey k{16, 52000, 30000};
    check(c.put("osm", k, t, 1760000000), "put");
    CachedTile g;
    check(c.get("osm", k, g, 1760000001) && g.png == t.png && g.etag == t.etag && g.fresh.freshUntil == 1760003600 &&
              g.fresh.staleIfError == 600,
          "get returns the PNG, validators and freshness");
    check(!c.get("other-style", k, g, 1760000001), "another provider id does not see it");
    TileFreshness nf = t.fresh;
    nf.freshUntil    = 1760099999;
    check(c.refresh("osm", k, nf, "", "", 1760000100) && c.get("osm", k, g, 1760000101) &&
              g.fresh.freshUntil == 1760099999 && g.etag == "\"abc\"",
          "304 refresh keeps the PNG and the old ETag when none is sent");
    // Budget 6 KiB: each tile is ~1 KiB, so the oldest go, never the kept ones.
    for (uint32_t i = 0; i < 12; ++i) c.put("osm", TileKey{16, i, 1}, t, 1760000200 + i);
    const uint64_t before = c.tileCount();
    c.evict("osm", {TileKey{16, 0, 1}, TileKey{16, 1, 1}});
    check(c.totalBytes() <= 6 * 1024 && c.evictions() > 0, std::to_string(before) + " tiles -> " +
                                                               std::to_string(c.tileCount()) + " under the budget");
    check(c.get("osm", TileKey{16, 0, 1}, g, 1) && c.get("osm", TileKey{16, 1, 1}, g, 1),
          "the oldest tiles survive when they are in the view");
    check(c.get("osm", TileKey{16, 11, 1}, g, 1), "the newest survives");
    check(!c.get("osm", k, g, 1) && !c.get("osm", TileKey{16, 2, 1}, g, 1),
          "the oldest unprotected tiles were evicted");
    check(c.totalBytes() <= 6 * 1024 - 6 * 1024 / 10, "evicted down to 90% of the budget");
    const uint64_t bytes = c.totalBytes(), count = c.tileCount();
    c.close();
    TileCache again;
    check(again.open(path, 1 << 20, &why) && again.get("osm", TileKey{16, 11, 1}, g, 1), "persists across reopen");
    check(again.totalBytes() == bytes && again.tileCount() == count,
          "incremental counters match a full recount after reopen (" + std::to_string(bytes) + " B, " +
              std::to_string(count) + " tiles)");
    CachedTile bigger = t;
    bigger.png        = makePng(9, 200, 30, 256, 256, false);
    for (int i = 0; i < 3; ++i) bigger.png.push_back(0);   // a different size than the stored one
    check(again.put("osm", TileKey{16, 11, 1}, bigger, 1760000300) && again.tileCount() == count &&
              again.totalBytes() == bytes - t.png.size() + bigger.png.size(),
          "replacing a tile counts the new size once (old size subtracted)");
    const uint64_t bytes2 = again.totalBytes();
    again.close();
    check(again.open(path, 1 << 20, &why) && again.totalBytes() == bytes2, "and matches the recount after reopen");
    again.close();
    {   // LRU on read: a tile read an hour later outlives a newer one that was not.
        const std::string lruPath = dir + "/lru.sqlite";
        TileCache lru;
        const uint64_t size = t.png.size();
        check(lru.open(lruPath, 2 * size - 1, &why), "LRU cache opens");
        const int64_t T = 1760000000;
        lru.put("osm", TileKey{16, 100, 1}, t, T);        // A
        lru.put("osm", TileKey{16, 101, 1}, t, T + 10);   // B, newer
        lru.get("osm", TileKey{16, 100, 1}, g, T + 7200); // A read 2 h later: touched
        lru.evict("osm", {});
        check(lru.get("osm", TileKey{16, 100, 1}, g, 1) && !lru.get("osm", TileKey{16, 101, 1}, g, 1),
              "the tile read recently stays; the newer unread one is evicted");
    }
    {   // A damaged file is set aside and replaced by an empty cache.
        const std::string badPath = dir + "/bad.sqlite";
        std::ofstream(badPath, std::ios::binary) << std::string(8192, 'x');
        TileCache bad;
        check(bad.open(badPath, 1 << 20, &why), "a corrupt file still opens a cache (" + why + ")");
        const std::string notice = bad.takeNotice();
        check(!notice.empty() && fs::exists(badPath + ".corrupt") && bad.tileCount() == 0,
              "it was set aside as .corrupt and reported: " + notice.substr(0, 60));
        check(bad.put("osm", k, t, 1760000000) && bad.get("osm", k, g, 1760000001), "and the new cache works");
    }
    TileCache badPath;
    check(!badPath.open("/proc/navigation_test/tiles.sqlite", 1 << 20, &why), "an unwritable path fails cleanly");
    fs::remove_all(dir, ec);
}

static void testCompose() {
    std::printf("\n[5] compositor and JPEG\n");
    std::map<std::pair<uint32_t, uint32_t>, std::vector<uint8_t>> tiles;
    auto lookup = [&](const TileKey& k) -> const uint8_t* {
        auto it = tiles.find({k.x, k.y});
        if (it == tiles.end()) {
            uint8_t r, g, b;
            keyColour(k, r, g, b);
            std::vector<uint8_t> rgb;
            const auto png = makePng(r, g, b);
            decodeTilePng(png.data(), png.size(), rgb);
            it = tiles.emplace(std::make_pair(k.x, k.y), std::move(rgb)).first;
        }
        return it->second.data();
    };
    MapLayout L;
    NavView   v;
    std::vector<uint8_t> rgb;
    ComposeResult r = composeMap(v, 16, L, lookup, NetState::Unknown, "\xC2\xA9 OpenStreetMap contributors", rgb);
    auto px = [&](int x, int y) { return &rgb[(size_t(y) * L.width + x) * 3]; };
    check(rgb.size() == 320u * 224 * 3 && r.tilesNeeded == 0, "no position: no tiles requested");
    check(px(1, 1)[0] < 60 && px(319, 222)[0] > 200, "dark status strip, light attribution strip");

    v.state      = PositionState::Live;
    v.marker     = Marker::Arrow;
    v.latitude   = 10.7769;
    v.longitude  = 106.7009;
    v.courseDeg  = 90.0f;
    v.speedKmh   = 42.0f;
    v.fixType    = 3;
    r = composeMap(v, 16, L, lookup, NetState::Online, "\xC2\xA9 OpenStreetMap contributors", rgb);
    check(r.mapComplete && r.tilesDrawn == r.tilesNeeded && r.tilesNeeded >= 2, "all visible tiles drawn");
    // The centre tile's colour away from the marker.
    const WorldPoint c = project(v.latitude, v.longitude, 16);
    TileKey ck{16, uint32_t(c.x / 256), uint32_t(c.y / 256)};
    uint8_t cr, cg, cb;
    keyColour(ck, cr, cg, cb);
    const uint8_t* p = px(160 - 40, 112);
    const bool leftInSame = int(c.x) % 256 >= 40;
    check(!leftInSame || (p[0] == cr && p[1] == cg && p[2] == cb), "the tile under the vehicle is centred");
    // Course 90: the arrow tip points right (east), nothing marker-blue above.
    auto blue = [&](int x, int y) { const uint8_t* q = px(x, y); return q[2] > 200 && q[0] < 60; };
    check(blue(160 + 8, 112) && !blue(160, 112 - 9) && !blue(160 - 9, 112), "arrow points east at course 90");
    v.courseDeg = 0.0f;
    composeMap(v, 16, L, lookup, NetState::Online, "x", rgb);
    check(blue(160, 112 - 8) && !blue(160 + 9, 112), "arrow points north at course 0");
    v.courseDeg = 180.0f;
    composeMap(v, 16, L, lookup, NetState::Online, "x", rgb);
    check(blue(160, 112 + 8) && !blue(160, 112 - 9), "arrow points south at course 180");

    int calls = 0;
    auto none = [&](const TileKey&) -> const uint8_t* { ++calls; return nullptr; };
    r = composeMap(v, 16, L, none, NetState::Offline, "x", rgb);
    check(!r.mapComplete && r.tilesDrawn == 0 && calls == r.tilesNeeded, "missing tiles: hatched");
    check(blue(160, 112 + 8) && !blue(160, 112 - 9), "the position is still drawn over missing tiles");

    // JPEG: baseline, 4:4:4, 320x224, decodes back; an over-small buffer fails cleanly.
    v.courseDeg = 45.0f;
    composeMap(v, 16, L, lookup, NetState::Online, "\xC2\xA9 OpenStreetMap contributors", rgb);
    std::vector<uint8_t> out(65536);
    size_t n = 0;
    const int64_t t0 = nowMs();
    for (int i = 0; i < 20; ++i) encodeMapJpeg(rgb.data(), 320, 224, 80, out.data(), out.size(), n);
    const double ms = double(nowMs() - t0) / 20.0;
    const JpegInfo ji = decodeJpeg(out.data(), n);
    char line[160];
    std::snprintf(line, sizeof(line), "320x224 q80 4:4:4 baseline: %zu bytes, %.2f ms per encode", n, ms);
    check(ji.ok && ji.w == 320 && ji.h == 224 && !ji.progressive && !ji.subsampled && n > 0 && n <= 65536, line);
    out.resize(n);
    saveArtifact("compose_arrow_45.jpg", out);
    std::vector<uint8_t> tiny(512);
    check(!encodeMapJpeg(rgb.data(), 320, 224, 80, tiny.data(), tiny.size(), n) && n == 0,
          "an over-small output buffer fails without writing past it");
}

struct ServiceRig {
    TileServer        server;
    FakeSink          sink;
    NavigationService nav;
    std::string       dir;
    ServiceRig() {
        char        tmpl[] = "/tmp/navigation_test_XXXXXX";
        const char* d      = ::mkdtemp(tmpl);
        dir                = d ? d : "/tmp/navigation_test_service";
    }
    ~ServiceRig() {
        nav.stop();
        server.stop();
        std::error_code ec;
        fs::remove_all(dir, ec);
    }
    NavigationServiceConfig cfg() {
        NavigationServiceConfig c;
        c.providerUrl      = server.url();
        c.providerId       = "test";
        c.cachePath        = dir + "/tiles.sqlite";
        c.connectTimeoutMs = 500;
        c.requestTimeoutMs = 2000;
        return c;
    }
};

/// Feeds a live GPS fix every 250 ms (the MKR's 4 Hz) on a thread.
class GpsFeed {
public:
    GpsFeed(NavigationService& n, double lat, double lon) : n_(n), lat_(lat), lon_(lon) {
        t_ = std::thread([this] {
            while (run_) {
                n_.offerGps(fix(lat_, lon_, 30.0f, 60.0f));
                sleepMs(250);
            }
        });
    }
    ~GpsFeed() {
        run_ = false;
        t_.join();
    }

private:
    NavigationService& n_;
    double             lat_, lon_;
    std::atomic<bool>  run_{true};
    std::thread        t_;
};

/// 4 Hz fixes delivered 10 times a second (the bridge repeats the newest), moving east at @p kmh.
class MovingFeed {
public:
    MovingFeed(NavigationService& n, double lat, double lon, double kmh) : n_(n) {
        t_ = std::thread([=] {
            const int64_t t0 = nowMs();
            double fixLon = lon;
            int64_t fixMs = t0;
            while (run_) {
                const int64_t now = nowMs();
                if (now - fixMs >= 250) {   // a new GNSS fix
                    fixMs = now;
                    const double metres = kmh / 3.6 * double(now - t0) / 1000.0;
                    fixLon = lon + metres / (111320.0 * std::cos(lat * M_PI / 180.0));
                    if (fixLon >= 180.0) fixLon -= 360.0;   // across the antimeridian
                }
                n_.offerGps(fix(lat, fixLon, float(kmh), 90.0f));
                sleepMs(100);
            }
        });
    }
    ~MovingFeed() {
        run_ = false;
        t_.join();
    }

private:
    NavigationService& n_;
    std::atomic<bool>  run_{true};
    std::thread        t_;
};

/// True when @p frame (a whole 320x224 map frame JPEG) shows the colour of the
/// tile under (lat, lon) a little beside the marker: tiles drawn, not hatched.
static bool centreTileDrawn(const std::vector<uint8_t>& frame, double lat, double lon) {
    const JpegInfo   fj = decodeJpeg(frame.data(), frame.size());
    const WorldPoint c  = project(lat, lon, 16);
    uint8_t er, eg, eb;
    keyColour(TileKey{16, uint32_t(c.x / 256), uint32_t(c.y / 256)}, er, eg, eb);
    const int dx = (int(c.x) % 256 >= 40) ? -30 : 30;   // inside the tile, clear of the marker and border
    if (!fj.ok || fj.w != 320 || fj.h != 224) return false;
    const uint8_t* px = &fj.rgb[(size_t(16 + 96) * 320 + 160 + dx) * 3];
    return std::abs(px[0] - er) <= 12 && std::abs(px[1] - eg) <= 12 && std::abs(px[2] - eb) <= 12;
}

static void testServicePolicy() {
    std::printf("\n[6] service: hidden = no requests, shown = fetch + cache, warm map shown at once\n");
    const double lat = 10.7769, lon = 106.7009;
    {
        ServiceRig r;
        check(r.server.start(), "fake tile server on 127.0.0.1");
        check(r.nav.start(r.cfg(), r.sink), "service starts");
        GpsFeed feed(r.nav, lat, lon);
        sleepMs(2200);
        NavigationStats s = r.nav.stats();
        check(r.server.requests() == 0 && s.httpRequests == 0, "map hidden for 2 s: zero HTTP requests (OSM policy)");
        check(s.warmComposes >= 1 && r.sink.frames() == 0 && s.position == PositionState::Live,
              std::to_string(s.warmComposes) + " warm composes while hidden, nothing sent");
        r.sink.active = true;
        check(waitFor([&] { return r.sink.frames() >= 1; }, 2000), "shown: frames offered");
        check(waitFor([&] { return r.nav.stats().http200 >= 2; }, 3000),
              std::to_string(r.nav.stats().http200) + " tiles fetched for the visible map");
        const auto seen = r.server.seen();
        bool ua = !seen.empty();
        for (const auto& x : seen) ua = ua && x.userAgent == "DashcamES3Map/1.0 (+https://github.com/chad-Ant/dashcam)";
        check(ua, "every request carries the application User-Agent");
        check(r.server.requests() <= 6, std::to_string(r.server.requests()) + " requests (visible tiles only, <= 6)");
        sleepMs(1500);   // the view's remaining tiles arrive (each arrival is drawn)
        const int64_t t0 = nowMs();
        const int f0 = r.sink.frames();
        sleepMs(2000);
        const double fps = (r.sink.frames() - f0) / (double(nowMs() - t0) / 1000.0);
        char line[120];
        std::snprintf(line, sizeof(line), "%.1f fps parked (repeated position): only the 1 s refresh, no re-sends", fps);
        check(fps >= 0.4 && fps <= 1.6, line);
        int w = 0, h = 0;
        const auto last = r.sink.last(&w, &h);
        const JpegInfo ji = decodeJpeg(last.data(), last.size());
        check(ji.ok && ji.w == 320 && ji.h == 224 && w == 320 && h == 224, "frames are 320x224 JPEGs");
        saveArtifact("service_live.jpg", last);
        s = r.nav.stats();
        check(s.cacheTiles == uint64_t(s.http200) && s.cacheTiles >= 2, std::to_string(s.cacheTiles) + " tiles on disk");
        std::printf("        first frame after show: %lld ms; encode %.2f ms; compose %.2f ms; %u bytes q%d\n",
                    (long long)s.firstFrameAfterShowMs, s.lastEncodeMs, s.lastComposeMs, s.lastJpegBytes,
                    s.lastQuality);
        r.nav.stop();

        // Restart on the same cache: fresh tiles, so no request at all, and the
        // warm working set makes the first frame after showing immediate.
        r.server.clear();
        r.sink.active = false;
        NavigationService again;
        check(again.start(r.cfg(), r.sink), "restart on the same disk cache");
        GpsFeed feed2(again, lat, lon);
        sleepMs(1500);   // warm: disk tiles into RAM
        const int before = r.sink.frames();
        r.sink.active = true;
        check(waitFor([&] { return r.sink.frames() > before; }, 1000), "shown again");
        sleepMs(300);
        const NavigationStats s2 = again.stats();
        check(r.server.requests() == 0, "fresh cached tiles: no request after the restart");
        check(s2.tileDiskHits >= 2 && s2.firstFrameAfterShowMs >= 0 && s2.firstFrameAfterShowMs < 150,
              "warm map: first frame " + std::to_string(s2.firstFrameAfterShowMs) + " ms after showing (< 150)");
        const auto lastFrame = r.sink.last();
        const JpegInfo j2 = decodeJpeg(lastFrame.data(), lastFrame.size());
        check(j2.ok, "and it decodes");
        // Whatever the phase of the warm render, the first frame follows the show at once.
        for (int offset : {50, 400, 800}) {
            r.sink.active = false;
            sleepMs(150);
            const uint64_t warm = again.stats().warmComposes;
            check(waitFor([&] { return again.stats().warmComposes > warm; }, 1500), "a warm render happened");
            sleepMs(offset);
            const int n = r.sink.frames();
            const int64_t shown = nowMs();
            r.sink.active = true;
            check(waitFor([&] { return r.sink.frames() > n; }, 1000), "a frame followed the show");
            const int64_t ms = again.stats().firstFrameAfterShowMs;
            check(nowMs() - shown < 1000, "(timed for this show)");
            check(ms >= 0 && ms < 150, "shown " + std::to_string(offset) + " ms after a warm render: first frame in " +
                                           std::to_string(ms) + " ms");
        }
        again.stop();
    }
    {   // Moving: the position changes with every fix, so frames follow it (cap 4 fps).
        ServiceRig r;
        check(r.server.start(), "fake tile server");
        r.sink.active = true;
        check(r.nav.start(r.cfg(), r.sink), "service starts");
        MovingFeed feed(r.nav, lat, lon, 100.0);
        sleepMs(800);
        const int64_t t0 = nowMs();
        const int     f0 = r.sink.frames();
        sleepMs(2000);
        const double fps = (r.sink.frames() - f0) / (double(nowMs() - t0) / 1000.0);
        char line[96];
        std::snprintf(line, sizeof(line), "%.1f fps driving with 4 Hz GPS (cap 4)", fps);
        check(fps >= 2.5 && fps <= 4.6, line);
    }
    for (int warm : {1, 0}) {   // Hidden while tiles are still queued: nothing more is fetched.
        ServiceRig r;
        r.server.delayMs = 400;
        check(r.server.start(), "slow fake tile server");
        NavigationServiceConfig c = r.cfg();
        c.fetchWorkers = 1;
        c.warmFps      = warm;
        r.sink.active  = true;
        check(r.nav.start(c, r.sink), "service starts (one worker, WarmFps " + std::to_string(warm) + ")");
        GpsFeed feed(r.nav, lat, lon);
        check(waitFor([&] { return r.server.requests() >= 1; }, 2000), "shown: the first tile is being fetched");
        r.sink.active = false;
        sleepMs(700);   // the request in flight may finish
        const int n = r.server.requests();
        sleepMs(2000);
        check(r.server.requests() == n, "hidden: the queued tiles are not fetched (" + std::to_string(n) +
                                            " requests, still " + std::to_string(r.server.requests()) + ")");
    }
}

static void testServiceFailures() {
    std::printf("\n[7] service: revalidation, 403, 429 + Retry-After, server down, invalid PNG, bounded stop\n");
    const double lat = 48.8584, lon = 2.2945;
    {   // max-age=0: the next session revalidates with If-None-Match and gets 304.
        ServiceRig r;
        r.server.maxAge = 0;
        check(r.server.start(), "fake tile server");
        r.sink.active = true;
        r.nav.start(r.cfg(), r.sink);
        {
            GpsFeed feed(r.nav, lat, lon);
            waitFor([&] { return r.nav.stats().http200 >= 2; }, 3000);
        }
        r.nav.stop();
        r.server.clear();
        NavigationService again;
        again.start(r.cfg(), r.sink);
        GpsFeed feed(again, lat, lon);
        check(waitFor([&] { return again.stats().http304 >= 2; }, 3000),
              std::to_string(again.stats().http304) + " expired tiles revalidated with 304");
        bool conditional = false;
        for (const auto& s : r.server.seen()) conditional = conditional || !s.ifNoneMatch.empty();
        check(conditional && again.stats().http200 == 0, "conditional requests (If-None-Match), no re-download");
        again.stop();
    }
    {   // 403: provider off for the session.
        ServiceRig r;
        r.server.status = 403;
        check(r.server.start(), "fake tile server");
        r.sink.active = true;
        r.nav.start(r.cfg(), r.sink);
        GpsFeed feed(r.nav, lat, lon);
        check(waitFor([&] { return r.nav.stats().net == NetState::Denied; }, 3000), "403 -> denied");
        sleepMs(300);   // the other worker's request may still be on its way
        const int n = r.server.requests();
        sleepMs(1500);
        check(r.server.requests() == n && n <= 2, "no further requests after the 403 (" + std::to_string(n) + ")");
        r.nav.stop();
    }
    {   // 429 with Retry-After: 3 s of silence.
        ServiceRig r;
        r.server.status     = 429;
        r.server.retryAfter = 3;
        check(r.server.start(), "fake tile server");
        r.sink.active = true;
        r.nav.start(r.cfg(), r.sink);
        GpsFeed feed(r.nav, lat, lon);
        check(waitFor([&] { return r.nav.stats().http429 >= 1; }, 3000), "429 received");
        const int n = r.server.requests();
        sleepMs(2000);
        check(r.server.requests() <= n + 1, "Retry-After honoured: at most one in-flight request more in 2 s (" +
                                                std::to_string(r.server.requests() - n) + ")");
        r.server.status = 200;
        check(waitFor([&] { return r.nav.stats().http200 >= 1; }, 6000), "fetching resumes after the backoff");
        r.nav.stop();
    }
    {   // Server gone: offline, cached tiles still shown within stale-if-error.
        ServiceRig r;
        r.server.maxAge = 1;
        check(r.server.start(), "fake tile server");
        r.sink.active = true;
        r.nav.start(r.cfg(), r.sink);
        {
            GpsFeed feed(r.nav, lat, lon);
            waitFor([&] { return r.nav.stats().cacheTiles >= 2; }, 3000);
        }
        r.nav.stop();
        r.server.stop();
        sleepMs(1100);   // max-age=1 has expired; stale-if-error=600 still holds
        NavigationService again;
        const int before = r.sink.frames();
        again.start(r.cfg(), r.sink);
        GpsFeed feed(again, lat, lon);
        check(waitFor([&] { return again.stats().net == NetState::Offline; }, 3000), "server down -> offline");
        check(waitFor([&] { return r.sink.frames() > before + 2; }, 3000), "frames keep coming");
        check(waitFor([&] { return again.stats().tileDiskHits >= 2; }, 2000),
              "expired tiles within stale-if-error are put on screen from the cache");
        check(waitFor([&] { return centreTileDrawn(r.sink.last(), lat, lon); }, 2500),
              "and drawn: the centre tile's colour is in the frame");
        again.stop();
    }
    {   // An HTML error page with 200: rejected, not cached, not asked again at once.
        ServiceRig r;
        r.server.html = true;
        check(r.server.start(), "fake tile server");
        r.sink.active = true;
        r.nav.start(r.cfg(), r.sink);
        GpsFeed feed(r.nav, lat, lon);
        check(waitFor([&] { return r.nav.stats().httpRejected >= 2; }, 3000), "HTML instead of PNG: rejected");
        waitFor([&] { return r.nav.stats().httpRequests == uint64_t(r.server.requests()); }, 1000);
        sleepMs(300);   // requests still on their way are counted before the snapshot
        const int n = r.server.requests();
        sleepMs(1200);
        std::set<std::string> paths;
        bool repeated = false;
        for (const auto& x : r.server.seen()) repeated = repeated || !paths.insert(x.path).second;
        check(r.server.requests() == n && !repeated && r.nav.stats().cacheTiles == 0,
              "negative-cached: no tile asked twice, nothing stored");
        r.nav.stop();
    }
    {   // A redirect: the provider is misconfigured, off for the run.
        ServiceRig r;
        r.server.status = 301;
        check(r.server.start(), "fake tile server");
        r.sink.active = true;
        r.nav.start(r.cfg(), r.sink);
        GpsFeed feed(r.nav, lat, lon);
        check(waitFor([&] { return r.nav.stats().net == NetState::Denied; }, 3000) && r.nav.stats().httpOther >= 1,
              "301 -> provider off (set the final URL)");
        sleepMs(300);
        const int n = r.server.requests();
        sleepMs(1200);
        check(r.server.requests() == n, "no further requests after the redirect");
    }
    {   // A 400 is about the tile, not the server: no provider backoff, the tile is not asked again soon.
        ServiceRig r;
        r.server.status = 400;
        check(r.server.start(), "fake tile server");
        r.sink.active = true;
        r.nav.start(r.cfg(), r.sink);
        GpsFeed feed(r.nav, lat, lon);
        check(waitFor([&] { return r.nav.stats().httpOther >= 2; }, 3000), "400 counted as 'other'");
        const NavigationStats s = r.nav.stats();
        check(s.net != NetState::Denied && s.net != NetState::Offline && s.httpErrors == 0,
              "not a provider failure (no denial, no offline)");
        sleepMs(300);
        const int n = r.server.requests();
        sleepMs(1200);
        check(r.server.requests() == n, "the rejected tiles are not asked again at once");
    }
    {   // stale-while-revalidate: an expired cached tile is shown at once while it is revalidated.
        ServiceRig r;
        r.server.maxAge = 0;
        r.server.swr    = 600;
        check(r.server.start(), "fake tile server");
        r.sink.active = true;
        r.nav.start(r.cfg(), r.sink);
        {
            GpsFeed feed(r.nav, lat, lon);
            waitFor([&] { return r.nav.stats().cacheTiles >= 2; }, 3000);
        }
        r.nav.stop();
        r.server.delayMs = 1500;   // the revalidation answers slowly
        NavigationService again;
        again.start(r.cfg(), r.sink);
        GpsFeed feed(again, lat, lon);
        check(waitFor([&] { return centreTileDrawn(r.sink.last(), lat, lon); }, 1200) && again.stats().http304 == 0,
              "stale tiles drawn in a frame before the 304 arrives");
        check(waitFor([&] { return again.stats().http304 >= 2; }, 4000), "and revalidated (304)");
        again.stop();
    }
    {   // start()/stop() in quick succession never hangs (no lost wake-up).
        ServiceRig r;
        check(r.server.start(), "fake tile server");
        int64_t worst = 0;
        for (int i = 0; i < 40; ++i) {
            NavigationService n;
            n.start(r.cfg(), r.sink);
            if (i % 2) n.offerGps(fix(lat, lon, 0.0f, NAN));
            sleepMs(i % 5);
            const int64_t t0 = nowMs();
            n.stop();
            worst = std::max(worst, nowMs() - t0);
        }
        check(worst < 500, "40 start/stop cycles, the slowest stop " + std::to_string(worst) + " ms");
    }
    {   // A hung server: stop() aborts the transfer.
        ServiceRig r;
        r.server.delayMs = 5000;
        check(r.server.start(), "fake tile server");
        r.sink.active = true;
        NavigationServiceConfig c = r.cfg();
        c.requestTimeoutMs = 10000;
        r.nav.start(c, r.sink);
        GpsFeed feed(r.nav, lat, lon);
        waitFor([&] { return r.server.requests() >= 1; }, 2000);
        const int64_t t0 = nowMs();
        r.nav.stop();
        const int64_t took = nowMs() - t0;
        check(took < 1000, "stop() with a hanging request took " + std::to_string(took) + " ms (< 1000)");
    }
    {   // No position: no tiles requested, frames say so.
        ServiceRig r;
        check(r.server.start(), "fake tile server");
        r.sink.active = true;
        r.nav.start(r.cfg(), r.sink);
        GpsSample acquiring;
        acquiring.gpsPresent = true;
        acquiring.receivedUs = nowUs();
        r.nav.offerGps(acquiring);
        check(waitFor([&] { return r.sink.frames() >= 1; }, 2000) && r.server.requests() == 0,
              "acquiring: a frame without a map, no tile request");
        check(r.nav.stats().position == PositionState::Acquiring, "position state acquiring");
        saveArtifact("service_acquiring.jpg", r.sink.last());
        r.nav.stop();
    }
}

// ─── board-side panning ──────────────────────────────────────────────────────

static void testPanHelpers() {
    std::printf("\n[8] panning helpers: pose interpolation, background planner, background/chrome images\n");
    {   // 4 Hz fixes along x at 40 px/s, delivered by 10 Hz telemetry (each fix repeated, 100 ms arrival steps).
        PoseInterpolator ip;
        const uint64_t t0 = 1000000000;
        for (int k = 0; k < 40; ++k) {   // 4 s of telemetry
            const uint64_t at  = t0 + uint64_t(k) * 100000;
            const int      fixN = int((at - t0) / 250000);   // newest fix at this moment
            ip.offer(WorldPoint{1000.0 + 10.0 * fixN, 500.0}, 90.0f, at);
        }
        check(ip.fixes() == 8 && near(ip.intervalUs(), 250000, 40000),
              "repeated telemetry samples are not new fixes; interval " + std::to_string(ip.intervalUs() / 1000) + " ms");
        const uint64_t end = t0 + 3900000;
        double prev = -1, maxStep = 0, minStep = 1e9;
        bool monotonic = true;
        for (uint64_t t = end - 1000000; t <= end; t += 50000) {   // 20 Hz poses over the last second
            const WorldPoint p = ip.at(t);
            if (prev >= 0) {
                monotonic = monotonic && p.x >= prev - 1e-9;
                maxStep = std::max(maxStep, p.x - prev);
                minStep = std::min(minStep, p.x - prev);
            }
            prev = p.x;
        }
        char line[160];
        std::snprintf(line, sizeof(line), "20 Hz poses move steadily: %.2f-%.2f px per 50 ms (truth 2.0)", minStep, maxStep);
        check(monotonic && minStep > 1.0 && maxStep < 3.2, line);
        const WorldPoint newest = ip.newest();
        check(near(ip.at(end + 5000000).x, newest.x, 1e-9), "past the newest fix it holds still (no extrapolation)");
        const WorldPoint v = ip.velocity();
        check(near(v.x, 40.0, 12.0) && near(v.y, 0.0, 1e-9), "velocity " + std::to_string(v.x) + " px/s");
        check(ip.delayUs() >= 250000 && ip.delayUs() <= 350000, "lags one interval + 50 ms");
        // Every interpolated point lies on the path between real fixes.
        bool onPath = true;
        for (uint64_t t = t0 + 600000; t <= end; t += 13000) {
            const WorldPoint p = ip.at(t);
            onPath = onPath && p.x >= 1000.0 && p.x <= newest.x && near(p.y, 500.0, 1e-9);
        }
        check(onPath, "interpolated points stay between real fixes");
        // A jump or a long gap: start afresh, no slide.
        ip.offer(WorldPoint{9000.0, 500.0}, NAN, end + 100000);
        check(ip.fixes() == 1 && near(ip.at(end + 200000).x, 9000.0, 1e-9), "a 2000+ px jump restarts at the new fix");
        ip.offer(WorldPoint{9010.0, 500.0}, NAN, end + 2500000);
        check(ip.fixes() == 1, "a gap over 2 s restarts too");
    }
    {   // Course across north takes the short way; NaN courses fall back to the newer fix's.
        PoseInterpolator ip;
        ip.offer(WorldPoint{0, 0}, 350.0f, 1000000);
        ip.offer(WorldPoint{10, 0}, 10.0f, 1250000);
        float c = 0;
        ip.at(1125000 + ip.delayUs(), &c);
        check(near(c, 0.0, 1.0) || near(c, 360.0, 1.0), "350 -> 10 degrees passes through north (" + std::to_string(c) + ")");
        ip.offer(WorldPoint{20, 0}, NAN, 1500000);
        ip.at(1400000 + ip.delayUs(), &c);
        check(std::isnan(c), "towards a fix without course: no course");
    }
    {   // A stationary receiver repeating one position still advances time (no stale hold).
        PoseInterpolator ip;
        for (int k = 0; k < 30; ++k) ip.offer(WorldPoint{5, 5}, NAN, 1000000 + uint64_t(k) * 100000);
        check(ip.fixes() >= 4 && near(ip.at(5000000).x, 5.0, 1e-9), "stationary: same position, fixes keep coming");
        for (int k = 30; k < 330; ++k) ip.offer(WorldPoint{5, 5}, NAN, 1000000 + uint64_t(k) * 100000);
        check(ip.intervalUs() <= 300000, "30 s of a repeated position leaves the fix interval at " +
                                            std::to_string(ip.intervalUs() / 1000) + " ms (not stretched to 1 s)");
    }
    {   // Planner.
        const WorldPoint still{0, 0}, east{30, 0};
        // Background 512x384 at origin (0,0), map 320x192: centred position fine, near the edge not.
        check(!panBackgroundStale(0, 0, 512, 384, 320, 192, WorldPoint{256, 192}, still, 1.0, 8), "centred: keep");
        check(panBackgroundStale(0, 0, 512, 384, 320, 192, WorldPoint{350, 192}, still, 1.0, 8),
              "the map area would cross the right edge margin: new background");
        check(panBackgroundStale(0, 0, 512, 384, 320, 192, WorldPoint{320, 192}, east, 1.0, 8) &&
                  !panBackgroundStale(0, 0, 512, 384, 320, 192, WorldPoint{320, 192}, still, 1.0, 8),
              "the look-ahead along the velocity triggers it earlier");
        int64_t ox = 0, oy = 0;
        panBackgroundOrigin(WorldPoint{1000, 1000}, still, 2.0, 512, 384, 320, 192, ox, oy);
        check(ox == 1000 - 256 && oy == 1000 - 192, "standing: centred on the position");
        panBackgroundOrigin(WorldPoint{1000, 1000}, east, 2.0, 512, 384, 320, 192, ox, oy);
        check(ox == 1000 + 58 - 256 && oy == 1000 - 192, "moving east: centred ahead, lead capped at 60% of the margin");
        panBackgroundOrigin(WorldPoint{1000, 1000}, WorldPoint{10, -5}, 2.0, 512, 384, 320, 192, ox, oy);
        check(ox == 1000 + 20 - 256 && oy == 1000 - 10 - 192, "small velocities: lead = v x 2 s");
        check(!panBackgroundStale(double(ox), double(oy), 512, 384, 320, 192, WorldPoint{1000, 1000},
                                  WorldPoint{10, -5}, 1.0, 8),
              "a fresh background is not stale");
        // A small margin (352x224 around 320x192: 16 px a side): the lead must
        // leave more than the 8 px edge behind, or every background is stale at once.
        panBackgroundOrigin(WorldPoint{1000, 1000}, WorldPoint{200, 0}, 2.0, 352, 224, 320, 192, ox, oy, 8);
        check(!panBackgroundStale(double(ox), double(oy), 352, 224, 320, 192, WorldPoint{1000, 1000}, WorldPoint{},
                                  0.0, 8),
              "small margin: a fresh background leaves room behind the vehicle");
    }
    {   // Background and chrome images.
        std::map<std::pair<uint32_t, uint32_t>, std::vector<uint8_t>> tiles;
        auto lookup = [&](const TileKey& k) -> const uint8_t* {
            auto it = tiles.find({k.x, k.y});
            if (it == tiles.end()) {
                uint8_t r, g, b;
                keyColour(k, r, g, b);
                std::vector<uint8_t> rgb;
                const auto png = makePng(r, g, b);
                decodeTilePng(png.data(), png.size(), rgb);
                it = tiles.emplace(std::make_pair(k.x, k.y), std::move(rgb)).first;
            }
            return it->second.data();
        };
        NavView v;
        v.state = PositionState::Live, v.marker = Marker::Arrow, v.latitude = 48.8584, v.longitude = 2.2945;
        v.courseDeg = 90.0f, v.speedKmh = 50.0f, v.fixType = 3;
        std::vector<uint8_t> rgb;
        const int64_t ox = 33600 * 256 + 100, oy = 22800 * 256 + 30;   // inside tile (33600, 22800) at zoom 16
        ComposeResult r = composePanBackground(v, true, 16, ox, oy, 512, 384, lookup, rgb);
        check(rgb.size() == 512u * 384 * 3 && r.mapComplete && r.tilesNeeded == 6,
              "512x384 background at +100,+30 in a tile: 3x2 tiles, complete (" + std::to_string(r.tilesNeeded) + ")");
        uint8_t cr, cg, cb;
        keyColour(TileKey{16, 33600, 22800}, cr, cg, cb);
        const uint8_t* p0 = &rgb[(size_t(1) * 512 + 1) * 3];   // (1,1): inside the tile, off its darker border
        keyColour(TileKey{16, 33601, 22800}, cr, cg, cb);
        const uint8_t* p1 = &rgb[(size_t(1) * 512 + 157) * 3];   // x = 157 -> world 33601*256+1: the next tile
        uint8_t ar, ag, ab;
        keyColour(TileKey{16, 33600, 22800}, ar, ag, ab);
        char px[160];
        std::snprintf(px, sizeof(px), "the background's top-left pixel is world pixel (ox, oy): %d,%d,%d vs %d,%d,%d; "
                      "x=157: %d,%d,%d vs %d,%d,%d", p0[0], p0[1], p0[2], ar, ag, ab, p1[0], p1[1], p1[2], cr, cg, cb);
        check(p0[0] == ar && p0[1] == ag && p0[2] == ab && p1[0] == cr && p1[1] == cg && p1[2] == cb, px);
        bool blue = false;
        for (size_t i = 0; i + 2 < rgb.size(); i += 3) blue = blue || (rgb[i + 2] > 200 && rgb[i] < 60 && rgb[i + 1] < 140);
        check(!blue, "no marker in the background (the board draws it)");
        NavView none;
        none.state = PositionState::Acquiring;
        r = composePanBackground(none, false, 16, 0, 0, 512, 384, lookup, rgb);
        check(r.tilesNeeded == 0 && rgb.size() == 512u * 384 * 3, "no position: grid and message, no tiles");

        std::vector<uint8_t> chrome;
        composePanChrome(v, 16, 320, 16, 16, NetState::Online, true, "\xC2\xA9 OpenStreetMap contributors", chrome);
        auto cpx = [&](int x, int y) { return &chrome[(size_t(y) * 320 + x) * 3]; };
        check(chrome.size() == 320u * 32 * 3 && cpx(1, 1)[0] < 60 && cpx(318, 30)[0] > 200,
              "chrome 320x32: dark status strip, light attribution strip");
        int dark = 0;   // the scale bar and its label at the bottom-left
        for (int y = 16; y < 32; ++y)
            for (int x = 0; x < 120; ++x) dark += cpx(x, y)[0] < 90;
        check(dark > 40, "scale bar and north mark in the bottom strip");
        std::vector<uint8_t> loading;
        composePanChrome(v, 16, 320, 16, 16, NetState::Online, false, "\xC2\xA9 OpenStreetMap contributors", loading);
        check(loading != chrome, "an incomplete background shows LOADING");

        // 4:2:0 backgrounds: subsampled and smaller than 4:4:4.
        composePanBackground(v, true, 16, ox, oy, 512, 384, lookup, rgb);
        std::vector<uint8_t> out(65536);
        size_t n420 = 0, n444 = 0;
        const bool e420 = encodeMapJpeg(rgb.data(), 512, 384, 75, out.data(), out.size(), n420, true);
        const JpegInfo j420 = decodeJpeg(out.data(), n420);
        encodeMapJpeg(rgb.data(), 512, 384, 75, out.data(), out.size(), n444, false);
        check(e420 && j420.ok && j420.subsampled && j420.w == 512 && j420.h == 384 && n420 < n444,
              "4:2:0 background " + std::to_string(n420) + " B vs 4:4:4 " + std::to_string(n444) + " B");
    }
}

/// A fake panning board: records every background, chrome and pose.
class FakePanSink : public IMapPanSink {
public:
    std::atomic<bool>     active{false};
    std::atomic<uint32_t> session{1};
    std::atomic<uint32_t> resend{0};
    std::atomic<bool>     refuse{false};
    std::atomic<int>      bgW{512}, bgH{384};
    PanDemand panDemand() const override {
        PanDemand d;
        d.active = active.load();
        d.session = session.load();
        d.resend  = resend.load();
        d.mapW = 320, d.mapH = 192, d.backgroundW = uint16_t(bgW.load()), d.backgroundH = uint16_t(bgH.load());
        d.chromeTopH = 16, d.chromeBottomH = 16, d.maxJpegBytes = 65536, d.maxPoseHz = 30;
        return d;
    }
    struct Bg {
        uint32_t ox = 0, oy = 0;
        int64_t  ms = 0;
        size_t   bytes = 0;
        std::vector<uint8_t> jpeg;
    };
    struct Pose {
        PanPose p;
        int64_t us = 0;
    };
    bool offerPanBackground(EncodedFrameLease&& j, uint32_t ox, uint32_t oy) override {
        if (refuse.load() || !active.load() || j.width() != bgW.load() || j.height() != bgH.load()) return false;
        std::lock_guard<std::mutex> lk(m_);
        Bg b;
        b.ox = ox, b.oy = oy, b.ms = nowMs(), b.bytes = j.size();
        b.jpeg.assign(j.data(), j.data() + j.size());
        bgs_.push_back(std::move(b));
        j.release();
        return true;
    }
    bool offerPanChrome(EncodedFrameLease&& j) override {
        if (refuse.load() || !active.load() || j.width() != 320 || j.height() != 32) return false;
        std::lock_guard<std::mutex> lk(m_);
        chromes_.emplace_back(j.data(), j.data() + j.size());
        j.release();
        return true;
    }
    bool offerPanPose(const PanPose& p) override {
        if (!active.load()) return false;
        std::lock_guard<std::mutex> lk(m_);
        poses_.push_back(Pose{p, int64_t(nowUs())});
        return true;
    }
    std::vector<Bg> bgs() {
        std::lock_guard<std::mutex> lk(m_);
        return bgs_;
    }
    std::vector<std::vector<uint8_t>> chromes() {
        std::lock_guard<std::mutex> lk(m_);
        return chromes_;
    }
    std::vector<Pose> poses() {
        std::lock_guard<std::mutex> lk(m_);
        return poses_;
    }
    void clear() {
        std::lock_guard<std::mutex> lk(m_);
        bgs_.clear(), chromes_.clear(), poses_.clear();
    }

private:
    std::mutex                        m_;
    std::vector<Bg>                   bgs_;
    std::vector<std::vector<uint8_t>> chromes_;
    std::vector<Pose>                 poses_;
};

static void testServicePan() {
    std::printf("\n[9] service with a panning board: background, chrome, 20 Hz smooth poses, new backgrounds ahead\n");
    const double lat = 48.8584, lon = 2.2945;
    FakePanSink pan;   // declared first: it outlives the service that calls it
    ServiceRig  r;
    check(r.server.start(), "fake tile server on 127.0.0.1");
    NavigationServiceConfig c = r.cfg();
    c.poseHz = 20;
    check(r.nav.start(c, r.sink, &pan), "service starts with a panning sink");
    r.sink.active = true;   // a whole-frame demand too: panning takes precedence while active
    // 200 km/h at zoom 16 (~1.6 m/px): ~35 px/s, so the 96 px margin runs out within seconds.
    MovingFeed feed(r.nav, lat, lon, 200.0);
    sleepMs(800);
    const int wholeBefore = r.sink.frames();
    pan.active = true;
    check(waitFor([&] { return !pan.bgs().empty() && !pan.chromes().empty(); }, 1500), "background and chrome offered");
    const auto first = pan.bgs().front();
    const WorldPoint at = project(lat, lon, 16);
    check(std::fabs(double(first.ox) + 256 - at.x) < 200 && std::fabs(double(first.oy) + 192 - at.y) < 40,
          "the first background surrounds the position");
    const JpegInfo bj = decodeJpeg(first.jpeg.data(), first.jpeg.size());
    check(bj.ok && bj.w == 512 && bj.h == 384 && bj.subsampled, "512x384 4:2:0 JPEG, " + std::to_string(first.bytes) + " B");
    saveArtifact("pan_background.jpg", first.jpeg);
    saveArtifact("pan_chrome.jpg", pan.chromes().front());
    const NavigationStats s0 = r.nav.stats();
    check(s0.panning && s0.firstFrameAfterShowMs >= 0 && s0.firstFrameAfterShowMs < 200,
          "first background " + std::to_string(s0.firstFrameAfterShowMs) + " ms after showing");

    sleepMs(5000);
    const auto poses = pan.poses();
    const auto bgs   = pan.bgs();
    const NavigationStats s = r.nav.stats();
    // Pose cadence and smoothness over the last 3 s.
    std::vector<FakePanSink::Pose> tail;
    for (const auto& p : poses)
        if (p.us >= poses.back().us - 3000000) tail.push_back(p);
    const double rate = double(tail.size() - 1) / (double(tail.back().us - tail.front().us) / 1e6);
    double maxStep = 0, maxDev = 0;
    bool   east = true, arrows = true;
    std::vector<double> steps;
    for (size_t i = 1; i < tail.size(); ++i) {
        const double dx = (double(tail[i].p.xQ4) - double(tail[i - 1].p.xQ4)) / 16.0;
        const double dt = double(tail[i].us - tail[i - 1].us) / 1e6;
        steps.push_back(dx / dt);
        maxStep = std::max(maxStep, std::fabs(dx));
        east    = east && dx >= -0.01 && tail[i].p.yQ4 == tail[i - 1].p.yQ4;
        arrows  = arrows && tail[i].p.marker == Marker::Arrow && tail[i].p.courseCdeg == 9000;
    }
    double mean = 0;
    for (double v : steps) mean += v;
    mean /= double(steps.size());
    for (double v : steps) maxDev = std::max(maxDev, std::fabs(v - mean));
    const double truth = 200.0 / 3.6 / metersPerPixel(lat, 16);
    char line[200];
    std::snprintf(line, sizeof(line), "%.1f poses/s (target 20)", rate);
    check(rate >= 17.0 && rate <= 21.5, line);
    std::snprintf(line, sizeof(line), "poses move east at %.1f px/s (truth %.1f), largest step %.2f px", mean, truth, maxStep);
    check(east && std::fabs(mean - truth) < truth * 0.15 && maxStep < truth / 20.0 * 2.5, line);
    std::snprintf(line, sizeof(line), "speed between poses varies by at most %.1f px/s (smooth, no 4 Hz jumps)", maxDev);
    check(maxDev < truth * 0.6, line);
    check(arrows, "arrow marker, course 90.00 deg");
    // Every pose shows inside the background the board has by then (the board
    // never runs out): a background is there only after crossing USB at about
    // 400 KB/s and ~80 ms of decoding, which the 1 s look-ahead must cover.
    bool covered = true;
    for (const auto& p : tail) {
        const FakePanSink::Bg* cur = nullptr;
        for (const auto& b : bgs)
            if (b.ms * 1000 + int64_t(b.bytes) * 1000000 / 400000 + 80000 <= p.us) cur = &b;
        if (!cur) continue;
        const es3map::MapPose mp{1, p.p.xQ4, p.p.yQ4, 0, es3map::MarkerArrow, 0};
        const es3map::PanView pv = es3map::computePanView(es3map::packOrigin(cur->ox, cur->oy), 512, 384, 320, 192, mp);
        covered = covered && pv.markerVisible && std::abs(pv.markerXQ4 / 16 - 160) <= 2;
    }
    check(covered, "every pose: the marker stays centred in the background the board has by then");
    check(bgs.size() >= 2 && bgs.size() <= 6, std::to_string(bgs.size()) + " backgrounds in ~6 s at 35 px/s");
    check(r.sink.frames() == wholeBefore || r.sink.frames() <= wholeBefore + 1, "no whole frames while panning");
    std::printf("        background %u B in %.1f ms; interpolation delay %u ms; chrome %llu; poses %llu\n",
                s.lastBackgroundBytes, s.lastBackgroundMs, s.interpolationDelayMs, (unsigned long long)s.panChrome,
                (unsigned long long)s.posesOffered);
    check(s.panChrome >= 1 && s.panChrome <= 3,
          std::to_string(s.panChrome) + " chrome image(s): sent only when it changes (steady speed here)");

    // A new board session: background and chrome again at once.
    const size_t nb = pan.bgs().size(), nc = pan.chromes().size();
    pan.session = 2;
    check(waitFor([&] { return pan.bgs().size() > nb && pan.chromes().size() > nc; }, 500),
          "new session: background and chrome resent");
    // Hidden: no poses; the whole-frame path resumes (warm or shown).
    pan.active = false;
    sleepMs(200);
    const size_t np = pan.poses().size();
    sleepMs(500);
    check(pan.poses().size() == np && !r.nav.stats().panning, "hidden: no poses");
    check(waitFor([&] { return r.sink.frames() > wholeBefore + 1; }, 1500), "the whole-frame sink is fed again");
}

/// Share of pixels of a decoded background that are the missing-tile fill.
static double hatchedShare(const std::vector<uint8_t>& jpeg) {
    const JpegInfo j = decodeJpeg(jpeg.data(), jpeg.size());
    if (!j.ok) return 1.0;
    size_t n = 0;
    for (size_t i = 0; i + 2 < j.rgb.size(); i += 3)
        n += std::abs(j.rgb[i] - 236) <= 4 && std::abs(j.rgb[i + 1] - 236) <= 4 && std::abs(j.rgb[i + 2] - 232) <= 4;
    return double(n) / double(j.rgb.size() / 3);
}

static void testServicePanEdges() {
    std::printf("\n[10] panning edge cases: parked completion, resend, link loss, antimeridian, no room to scroll\n");
    const double lat = 48.8584, lon = 2.2945;
    {   // Parked with a cold cache: the first background is incomplete; it is
        // completed as its tiles arrive, though the car does not move.
        FakePanSink pan;
        ServiceRig  r;
        r.server.delayMs = 300;
        check(r.server.start(), "slow fake tile server");
        check(r.nav.start(r.cfg(), r.sink, &pan), "service starts with a panning sink");
        auto feed = std::make_unique<GpsFeed>(r.nav, lat, lon);
        sleepMs(300);
        pan.active = true;
        check(waitFor([&] {
                  const auto b = pan.bgs();
                  return !b.empty() && hatchedShare(b.back().jpeg) < 0.01;
              }, 6000),
              "parked: a complete background follows once the tiles arrive (" + std::to_string(pan.bgs().size()) +
                  " backgrounds)");
        const auto first = pan.bgs().front();
        check(hatchedShare(first.jpeg) > 0.05, "(the first one was incomplete)");
        const size_t nb = pan.bgs().size();
        sleepMs(1500);
        check(pan.bgs().size() == nb, "and then no more while parked");
        // The board lost its images (shown again, or an image rejected): resend both.
        pan.clear();
        ++pan.resend;
        check(waitFor([&] { return !pan.bgs().empty() && !pan.chromes().empty(); }, 600),
              "resend requested: background and chrome sent again within the same session");
        // The bridge drops (and no more samples come): the marker turns grey at
        // once, well before the 1.5 s staleness, and the strip with it.
        feed.reset();
        sleepMs(50);
        const size_t  nc = pan.chromes().size(), np = pan.poses().size();
        const int64_t lostAt = int64_t(nowUs());
        r.nav.bridgeLost();
        sleepMs(400);
        const auto ps = pan.poses();
        bool afterOk = ps.size() > np + 3, grey = true;
        for (size_t i = np; i < ps.size(); ++i)
            if (ps[i].us > lostAt + 100000) grey = grey && ps[i].p.marker == Marker::LastKnown;
        check(afterOk && grey, "bridge lost: every pose from 100 ms on carries the last-known marker");
        check(pan.chromes().size() > nc, "and a new strip says so");
    }
    {   // Across the antimeridian: poses stay continuous (no jump by a world width).
        FakePanSink pan;
        ServiceRig  r;
        check(r.server.start(), "fake tile server");
        check(r.nav.start(r.cfg(), r.sink, &pan), "service starts");
        pan.active = true;
        MovingFeed feed(r.nav, 10.0, 179.9985, 150.0);   // reaches 180 E after about 4 s
        sleepMs(7000);
        const auto ps  = pan.poses();
        const auto bgs = pan.bgs();
        double maxStep = 0;
        for (size_t i = 1; i < ps.size(); ++i)
            if (ps[i].p.marker != Marker::None && ps[i - 1].p.marker != Marker::None)
                maxStep = std::max(maxStep, std::fabs(double(ps[i].p.xQ4) - double(ps[i - 1].p.xQ4)) / 16.0);
        // Each pose, against the background the board has by then: the marker in view.
        bool contained = !bgs.empty();
        for (const auto& p : ps) {
            const FakePanSink::Bg* cur = nullptr;
            for (const auto& b : bgs)
                if (b.ms * 1000 + int64_t(b.bytes) * 1000000 / 400000 + 80000 <= p.us) cur = &b;
            if (!cur || p.p.marker == Marker::None) continue;
            const es3map::MapPose mp{1, p.p.xQ4, p.p.yQ4, 0, es3map::MarkerArrow, 0};
            contained = contained &&
                        es3map::computePanView(es3map::packOrigin(cur->ox, cur->oy), 512, 384, 320, 192, mp).markerVisible;
        }
        check(ps.size() > 50 && maxStep < 20.0,
              std::to_string(ps.size()) + " poses across 180 E, largest step " + std::to_string(maxStep) + " px");
        check(contained && bgs.size() >= 2,
              std::to_string(bgs.size()) + " backgrounds; every pose's marker is inside the background on the board");
    }
    {   // A background with no room to scroll (as large as the map area): at most
        // one new background a second, not one per pass.
        FakePanSink pan;
        pan.bgW = 320;
        pan.bgH = 192;
        ServiceRig r;
        check(r.server.start(), "fake tile server");
        check(r.nav.start(r.cfg(), r.sink, &pan), "service starts");
        pan.active = true;
        GpsFeed feed(r.nav, lat, lon);
        sleepMs(2500);
        const auto b = pan.bgs();
        bool spaced = true;
        for (size_t i = 2; i < b.size(); ++i) spaced = spaced && b[i].ms - b[i - 1].ms >= 900;   // [0] is the no-position grid
        check(b.size() >= 1 && b.size() <= 4 && spaced,
              std::to_string(b.size()) + " backgrounds in 2.5 s with no margin, at least 0.9 s apart");
    }
}

/// navigation_test --live LAT LON [ZOOM]: the plan's real-endpoint smoke test.
/// The service with its default provider (tile.openstreetmap.org, the app's
/// User-Agent) shows one stationary view for 25 s: only that view's tiles are
/// requested (<= 6), cached under /tmp/navigation_live, and the frame sizes
/// printed (NAV_TEST_ARTIFACTS saves the last frame).  Needs internet.
static int runLive(double lat, double lon, int zoom) {
    FakeSink sink;
    sink.active = true;
    NavigationService nav;
    NavigationServiceConfig c;
    c.cachePath = "/tmp/navigation_live/tiles.sqlite";
    c.zoom      = zoom;
    if (!nav.start(c, sink, [](dashcam::log::LogLevel, const std::string& m) { std::printf("  log: %s\n", m.c_str()); }))
        return 2;
    const int64_t end = nowMs() + 25000;
    while (nowMs() < end) {
        nav.offerGps(fix(lat, lon, 0.0f, NAN));
        sleepMs(250);
    }
    const NavigationStats s = nav.stats();
    nav.stop();
    std::printf("live: %llu requests (%llu x 200, %llu x 304, %llu errors), %llu KiB of tiles; %llu disk hits; "
                "net %s; last frame %u bytes q%d, compose %.2f ms, encode %.2f ms; %d frames\n",
                (unsigned long long)s.httpRequests, (unsigned long long)s.http200, (unsigned long long)s.http304,
                (unsigned long long)s.httpErrors, (unsigned long long)(s.httpBytes / 1024),
                (unsigned long long)s.tileDiskHits, netStateName(s.net), s.lastJpegBytes, s.lastQuality,
                s.lastComposeMs, s.lastEncodeMs, sink.frames());
    saveArtifact("live.jpg", sink.last());
    return s.http200 + s.tileDiskHits > 0 ? 0 : 1;
}

/// navigation_test --drive OUTDIR: a demo drive for evaluating the render.
/// Spoofed 4 Hz GNSS along the Champs-Elysees (a straight avenue, so the line
/// between its two ends follows the road), with the service's default OSM
/// provider: standstill (dot), accelerate to 60 km/h (arrow), a 3 s GNSS outage
/// mid-way (last known), stop at Place de la Concorde (dot).  Every frame the
/// service offers is written to OUTDIR/frame_NNNNN.jpg plus OUTDIR/frames.csv.
/// Needs internet (only the visible tiles along the route are requested).
static int runDrive(const std::string& outDir) {
    struct RecordingSink : IPreviewSink {
        std::string dir;
        std::FILE*  csv = nullptr;
        int         n   = 0;
        int64_t     t0  = nowMs();
        PreviewDemand demand() const override {
            PreviewDemand d;
            d.active = true, d.canvasW = 320, d.canvasH = 224, d.maxJpegBytes = 65536;
            return d;
        }
        bool offerFrame(EncodedFrameLease&& f) override {
            char name[64];
            std::snprintf(name, sizeof(name), "frame_%05d.jpg", n++);
            std::ofstream(dir + "/" + name, std::ios::binary)
                .write(reinterpret_cast<const char*>(f.data()), std::streamsize(f.size()));
            if (csv) std::fprintf(csv, "%s,%lld,%zu,%d\n", name, (long long)(nowMs() - t0), f.size(), f.quality());
            f.release();
            return true;
        }
    } sink;
    sink.dir = outDir;
    sink.csv = std::fopen((outDir + "/frames.csv").c_str(), "w");
    if (!sink.csv) return 2;
    std::fprintf(sink.csv, "file,ms,bytes,quality\n");
    NavigationService nav;
    NavigationServiceConfig c;
    c.cachePath = outDir + "/tiles.sqlite";
    if (!nav.start(c, sink, [](dashcam::log::LogLevel, const std::string& m) { std::printf("  log: %s\n", m.c_str()); }))
        return 2;

    // Avenue des Champs-Elysees: Arc de Triomphe end -> Place de la Concorde end.
    const double lat0 = 48.87340, lon0 = 2.29660, lat1 = 48.86610, lon1 = 2.31960;
    const double kDeg = M_PI / 180.0;
    const double dy = (lat1 - lat0) * 111320.0, dx = (lon1 - lon0) * 111320.0 * std::cos(lat0 * kDeg);
    const double length = std::hypot(dx, dy);
    const float  course = float(std::fmod(std::atan2(dx, dy) / kDeg + 360.0, 360.0));
    std::printf("drive: %.0f m at course %.1f deg\n", length, double(course));
    double dist = 0.0, speed = 0.0;   // m, km/h
    int    phase = 0;                 // 0 standstill, 1 drive, 2 stopped at the end
    int64_t phaseStart = nowMs(), outageStart = -1;
    const int64_t start = nowMs();
    while (true) {
        const double t = (nowMs() - phaseStart) / 1000.0;
        if (phase == 0 && t >= 4.0) { phase = 1; phaseStart = nowMs(); }
        if (phase == 1) {
            const double remaining = length - dist;
            const double target = remaining < 60.0 ? std::max(0.0, remaining * 1.0) : 60.0;   // brake at the end
            speed += std::clamp(target - speed, -6.0, 3.0);   // per 250 ms step
            dist += speed / 3.6 * 0.25;
            if (remaining <= 0.5 || (remaining < 5.0 && speed < 1.0)) { phase = 2; phaseStart = nowMs(); speed = 0; }
        }
        if (phase == 2 && t >= 5.0) break;
        const double f = std::min(1.0, dist / length);
        const bool outage = phase == 1 && dist > length * 0.45 && dist < length * 0.45 + 50.0;   // ~3 s at 60 km/h
        if (outage && outageStart < 0) outageStart = nowMs();
        if (!outage)
            nav.offerGps(fix(lat0 + f * (lat1 - lat0), lon0 + f * (lon1 - lon0), float(speed), speed >= 5.0 ? course : NAN));
        sleepMs(250);
    }
    const NavigationStats s = nav.stats();
    nav.stop();
    std::fclose(sink.csv);
    std::printf("drive: %.0f s, %d frames, %llu tile downloads (%llu KiB), %llu cached, last frame %u B, "
                "max encode %.1f ms; outage began at %.1f s\n",
                (nowMs() - start) / 1000.0, sink.n, (unsigned long long)s.http200,
                (unsigned long long)(s.httpBytes / 1024), (unsigned long long)s.cacheTiles, s.lastJpegBytes,
                s.lastEncodeMs, outageStart < 0 ? -1.0 : (outageStart - start) / 1000.0);
    return sink.n > 0 ? 0 : 1;
}

/// Encodes packed RGB to a baseline JPEG file (simulator output).
static void writeJpegFile(const std::string& path, const std::vector<uint8_t>& rgb, int w, int h) {
    std::vector<uint8_t> out(1 << 20);
    size_t n = 0;
    if (encodeMapJpeg(rgb.data(), w, h, 90, out.data(), out.size(), n))
        std::ofstream(path, std::ios::binary).write(reinterpret_cast<const char*>(out.data()), std::streamsize(n));
}

/**
 * A simulated ES3C28P for panning (extension v2): it "receives" backgrounds
 * after a modelled USB transfer + decode delay, applies the newest pose with
 * es3map::computePanView() exactly as the firmware will, draws the marker,
 * and renders the 320x240 panel at 30 fps into OUTDIR/sim_NNNNN.jpg.
 */
class SimPanBoard : public IMapPanSink {
public:
    explicit SimPanBoard(std::string dir) : dir_(std::move(dir)) {
        csv_ = std::fopen((dir_ + "/backgrounds.csv").c_str(), "w");
        if (csv_) std::fprintf(csv_, "ms,origin_x,origin_y,bytes,quality\n");
        thr_ = std::thread([this] { loop(); });
    }
    ~SimPanBoard() {
        run_ = false;
        thr_.join();
        if (csv_) std::fclose(csv_);
    }
    PanDemand panDemand() const override {
        PanDemand d;
        d.active = true, d.session = 1, d.mapW = 320, d.mapH = 192, d.backgroundW = 512, d.backgroundH = 384;
        d.chromeTopH = 16, d.chromeBottomH = 16, d.maxJpegBytes = 65536, d.maxPoseHz = 30;
        return d;
    }
    bool offerPanBackground(EncodedFrameLease&& j, uint32_t ox, uint32_t oy) override {
        JpegInfo ji = decodeJpeg(j.data(), j.size());
        if (!ji.ok) return false;
        std::lock_guard<std::mutex> lk(m_);
        // USB at an assumed 400 KB/s plus ~80 ms decode/convert on the S3.
        pendingBg_      = std::move(ji.rgb);
        pendingOrigin_  = es3map::packOrigin(ox, oy);
        pendingAtUs_    = int64_t(nowUs()) + int64_t(j.size()) * 1000000 / 400000 + 80000;
        havePendingBg_  = true;
        ++backgrounds;
        maxBytes = std::max(maxBytes, j.size());
        if (csv_) std::fprintf(csv_, "%lld,%u,%u,%zu,%d\n", (long long)(nowMs() - t0_), ox, oy, j.size(), j.quality());
        return true;
    }
    bool offerPanChrome(EncodedFrameLease&& j) override {
        JpegInfo ji = decodeJpeg(j.data(), j.size());
        if (!ji.ok) return false;
        std::lock_guard<std::mutex> lk(m_);
        chrome_ = std::move(ji.rgb);
        return true;
    }
    bool offerPanPose(const PanPose& p) override {
        std::lock_guard<std::mutex> lk(m_);
        pose_ = p;
        havePose_ = true;
        ++poses;
        return true;
    }
    std::atomic<int> backgrounds{0}, poses{0}, frames{0};
    size_t           maxBytes = 0;

private:
    void put(std::vector<uint8_t>& f, int x, int y, uint8_t r, uint8_t g, uint8_t b) {
        if (x < 0 || x >= 320 || y < 16 || y >= 208) return;   // map area only
        uint8_t* p = &f[(size_t(y) * 320 + x) * 3];
        p[0] = r, p[1] = g, p[2] = b;
    }
    void disc(std::vector<uint8_t>& f, double cx, double cy, double rad, double inner, uint8_t r, uint8_t g, uint8_t b) {
        for (int y = int(cy - rad - 1); y <= int(cy + rad + 1); ++y)
            for (int x = int(cx - rad - 1); x <= int(cx + rad + 1); ++x) {
                const double d = std::hypot(x + 0.5 - cx, y + 0.5 - cy);
                if (d <= rad && d >= inner) put(f, x, y, r, g, b);
            }
    }
    void tri(std::vector<uint8_t>& f, const double (&p)[3][2], uint8_t r, uint8_t g, uint8_t b) {
        auto e = [](const double* a, const double* c, double x, double y) {
            return (c[0] - a[0]) * (y - a[1]) - (c[1] - a[1]) * (x - a[0]);
        };
        for (int y = int(std::min({p[0][1], p[1][1], p[2][1]})) - 1; y <= int(std::max({p[0][1], p[1][1], p[2][1]})) + 1; ++y)
            for (int x = int(std::min({p[0][0], p[1][0], p[2][0]})) - 1; x <= int(std::max({p[0][0], p[1][0], p[2][0]})) + 1; ++x) {
                const double px = x + 0.5, py = y + 0.5;
                const double a0 = e(p[0], p[1], px, py), a1 = e(p[1], p[2], px, py), a2 = e(p[2], p[0], px, py);
                if ((a0 >= 0 && a1 >= 0 && a2 >= 0) || (a0 <= 0 && a1 <= 0 && a2 <= 0)) put(f, x, y, r, g, b);
            }
    }
    void loop() {
        std::vector<uint8_t> frame(320 * 240 * 3);
        int64_t next = int64_t(nowUs());
        while (run_) {
            next += 33333;
            const int64_t wait = next - int64_t(nowUs());
            if (wait > 0) std::this_thread::sleep_for(std::chrono::microseconds(wait));
            std::lock_guard<std::mutex> lk(m_);
            if (havePendingBg_ && int64_t(nowUs()) >= pendingAtUs_) {
                bg_ = std::move(pendingBg_);
                origin_ = pendingOrigin_;
                havePendingBg_ = false;
            }
            std::fill(frame.begin(), frame.end(), 0);
            for (int y = 224; y < 240; ++y)   // the board's own tab strip
                for (int x = 0; x < 320; ++x) frame[(size_t(y) * 320 + x) * 3 + 1] = 60;
            if (chrome_.size() == 320u * 32 * 3) {
                std::memcpy(frame.data(), chrome_.data(), 320 * 16 * 3);
                std::memcpy(&frame[size_t(208) * 320 * 3], &chrome_[size_t(16) * 320 * 3], 320 * 16 * 3);
            }
            if (bg_.size() == 512u * 384 * 3 && havePose_) {
                es3map::MapPose mp;
                mp.seq = 1, mp.xQ4 = pose_.xQ4, mp.yQ4 = pose_.yQ4, mp.courseCdeg = pose_.courseCdeg;
                mp.marker = uint8_t(pose_.marker);
                const es3map::PanView v = es3map::computePanView(origin_, 512, 384, 320, 192, mp);
                for (int y = 0; y < 192; ++y)
                    std::memcpy(&frame[(size_t(16 + y) * 320) * 3], &bg_[(size_t(v.cropY + y) * 512 + v.cropX) * 3],
                                320 * 3);
                const double cx = v.markerXQ4 / 16.0, cy = 16 + v.markerYQ4 / 16.0;
                if (v.markerVisible) {
                    if (mp.marker == es3map::MarkerArrow) {
                        const double a = mp.courseCdeg / 100.0 * M_PI / 180.0;
                        auto rot = [&](double x, double y, double* o) {
                            o[0] = cx + x * std::cos(a) + y * std::sin(a);
                            o[1] = cy - (-x * std::sin(a) + y * std::cos(a));
                        };
                        double halo[3][2], body[3][2];
                        rot(0, 14, halo[0]), rot(-10, -10, halo[1]), rot(10, -10, halo[2]);
                        rot(0, 11, body[0]), rot(-7.5, -8, body[1]), rot(7.5, -8, body[2]);
                        tri(frame, halo, 255, 255, 255);
                        tri(frame, body, 20, 100, 255);
                    } else if (mp.marker == es3map::MarkerDot) {
                        disc(frame, cx, cy, 8.5, 0, 255, 255, 255);
                        disc(frame, cx, cy, 6.0, 0, 20, 100, 255);
                    } else {
                        disc(frame, cx, cy, 8.5, 0, 255, 255, 255);
                        disc(frame, cx, cy, 7.0, 4.5, 120, 120, 120);
                    }
                }
            }
            char name[64];
            std::snprintf(name, sizeof(name), "/sim_%05d.jpg", frames.load());
            writeJpegFile(dir_ + name, frame, 320, 240);
            ++frames;
        }
    }

    std::string          dir_;
    std::FILE*           csv_ = nullptr;
    int64_t              t0_  = nowMs();
    std::atomic<bool>    run_{true};
    std::thread          thr_;
    std::mutex           m_;
    std::vector<uint8_t> bg_, pendingBg_, chrome_;
    uint64_t             origin_ = 0, pendingOrigin_ = 0;
    int64_t              pendingAtUs_ = 0;
    bool                 havePendingBg_ = false, havePose_ = false;
    PanPose              pose_;
};

/// navigation_test --pan-drive OUTDIR: the --drive route with board-side
/// panning, rendered by a simulated board at 30 fps (OUTDIR/sim_NNNNN.jpg,
/// backgrounds.csv).  Needs internet (default OSM provider).
static int runPanDrive(const std::string& outDir) {
    FakeSink   whole;   // never active: panning only
    SimPanBoard board(outDir);
    NavigationService nav;
    NavigationServiceConfig c;
    c.cachePath = outDir + "/tiles.sqlite";
    if (!nav.start(c, whole, &board, [](dashcam::log::LogLevel, const std::string& m) { std::printf("  log: %s\n", m.c_str()); }))
        return 2;
    const double lat0 = 48.87340, lon0 = 2.29660, lat1 = 48.86610, lon1 = 2.31960;
    const double kDeg = M_PI / 180.0;
    const double dy = (lat1 - lat0) * 111320.0, dx = (lon1 - lon0) * 111320.0 * std::cos(lat0 * kDeg);
    const double length = std::hypot(dx, dy);
    const float  course = float(std::fmod(std::atan2(dx, dy) / kDeg + 360.0, 360.0));
    double dist = 0.0, speed = 0.0;
    int    phase = 0;
    int64_t phaseStart = nowMs(), lastFix = 0;
    double fixLat = lat0, fixLon = lon0;
    const int64_t start = nowMs();
    while (true) {   // 10 Hz telemetry carrying 4 Hz fixes, as on the car
        const int64_t now = nowMs();
        const double t = (now - phaseStart) / 1000.0;
        if (phase == 0 && t >= 3.0) { phase = 1; phaseStart = now; }
        if (now - lastFix >= 250) {
            lastFix = now;
            if (phase == 1) {
                const double remaining = length - dist;
                const double target = remaining < 80.0 ? std::max(3.0, remaining * 0.8) : 60.0;
                speed += std::clamp(target - speed, -6.0, 3.0);
                dist += speed / 3.6 * 0.25;
                if (remaining <= 1.0) { phase = 2; phaseStart = now; speed = 0; }
            }
            const double f = std::min(1.0, dist / length);
            fixLat = lat0 + f * (lat1 - lat0);
            fixLon = lon0 + f * (lon1 - lon0);
        }
        if (phase == 2 && t >= 3.0) break;
        nav.offerGps(fix(fixLat, fixLon, float(speed), speed >= 5.0 ? course : NAN));
        sleepMs(100);
    }
    const NavigationStats s = nav.stats();
    nav.stop();
    std::printf("pan-drive: %.0f s, %d simulated panel frames, %d backgrounds (largest %zu B), %d poses, "
                "%llu tile downloads; interpolation delay %u ms\n",
                (nowMs() - start) / 1000.0, board.frames.load(), board.backgrounds.load(), board.maxBytes,
                board.poses.load(), (unsigned long long)s.http200, s.interpolationDelayMs);
    return board.backgrounds.load() > 0 ? 0 : 1;
}

int main(int argc, char** argv) {
    if (argc == 3 && std::string(argv[1]) == "--drive") return runDrive(argv[2]);
    if (argc == 3 && std::string(argv[1]) == "--pan-drive") return runPanDrive(argv[2]);
    if (argc >= 4 && std::string(argv[1]) == "--live")
        return runLive(std::atof(argv[2]), std::atof(argv[3]), argc >= 5 ? std::atoi(argv[4]) : 16);
    std::printf("navigation_test: libnavigation without network or hardware\n");
    const int64_t t0 = nowMs();
    testMercator();
    testNavState();
    testPngAndHttpRules();
    testRulesAndHealth();
    testTileCache();
    testCompose();
    testServicePolicy();
    testServiceFailures();
    testPanHelpers();
    testServicePan();
    testServicePanEdges();
    std::printf("\n%s (%d failure%s, %.1f s)\n", g_fails ? "FAILED" : "ALL PASSED", g_fails, g_fails == 1 ? "" : "s",
                double(nowMs() - t0) / 1000.0);
    return g_fails ? 1 : 0;
}
