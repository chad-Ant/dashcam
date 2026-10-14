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
// Usage: navigation_test     (NAV_TEST_ARTIFACTS=dir writes sample frames as .jpg)

#include "libnavigation.h"

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
#include <mutex>
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
                   std::to_string(maxAge.load()) + ", stale-if-error=600\r\n";
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

static void testTileCache() {
    std::printf("\n[4] SQLite tile cache\n");
    const std::string dir = "/tmp/navigation_test_cache";
    std::error_code   ec;
    fs::remove_all(dir, ec);
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
    c.close();
    TileCache again;
    check(again.open(path, 1 << 20, &why) && again.get("osm", TileKey{16, 11, 1}, g, 1), "persists across reopen");
    again.close();
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
    check(!r.mapComplete && r.tilesDrawn == 0 && calls == r.tilesNeeded, "missing tiles: hatched, position still drawn");

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
        dir = "/tmp/navigation_test_service";
        std::error_code ec;
        fs::remove_all(dir, ec);
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
        const int64_t t0 = nowMs();
        const int f0 = r.sink.frames();
        sleepMs(2000);
        const double fps = (r.sink.frames() - f0) / (double(nowMs() - t0) / 1000.0);
        char line[96];
        std::snprintf(line, sizeof(line), "%.1f fps while shown with 4 Hz GPS (cap 4)", fps);
        check(fps >= 2.5 && fps <= 4.6, line);
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
        again.stop();
    }
}

static void testServiceFailures() {
    std::printf("\n[7] service: revalidation, 403, 429 + Retry-After, server down, invalid PNG, bounded stop\n");
    const double lat = 48.8584, lon = 2.2945;
    {   // max-age=0: the next session revalidates with If-None-Match and gets 304.
        ServiceRig r;
        r.server.maxAge = 0;
        r.server.start();
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
        r.server.start();
        r.sink.active = true;
        r.nav.start(r.cfg(), r.sink);
        GpsFeed feed(r.nav, lat, lon);
        check(waitFor([&] { return r.nav.stats().net == NetState::Denied; }, 3000), "403 -> denied");
        const int n = r.server.requests();
        sleepMs(1500);
        check(r.server.requests() == n && n <= 2, "no further requests after the 403 (" + std::to_string(n) + ")");
        r.nav.stop();
    }
    {   // 429 with Retry-After: 3 s of silence.
        ServiceRig r;
        r.server.status     = 429;
        r.server.retryAfter = 3;
        r.server.start();
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
        r.server.start();
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
              "expired tiles within stale-if-error are still drawn from the cache");
        again.stop();
    }
    {   // An HTML error page with 200: rejected, not cached, not asked again at once.
        ServiceRig r;
        r.server.html = true;
        r.server.start();
        r.sink.active = true;
        r.nav.start(r.cfg(), r.sink);
        GpsFeed feed(r.nav, lat, lon);
        check(waitFor([&] { return r.nav.stats().httpRejected >= 2; }, 3000), "HTML instead of PNG: rejected");
        const int n = r.server.requests();
        sleepMs(1200);
        check(r.server.requests() == n && r.nav.stats().cacheTiles == 0, "negative-cached, nothing stored");
        r.nav.stop();
    }
    {   // A hung server: stop() aborts the transfer.
        ServiceRig r;
        r.server.delayMs = 5000;
        r.server.start();
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
        r.server.start();
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

int main(int argc, char** argv) {
    if (argc >= 4 && std::string(argv[1]) == "--live")
        return runLive(std::atof(argv[2]), std::atof(argv[3]), argc >= 5 ? std::atoi(argv[4]) : 16);
    std::printf("navigation_test: libnavigation without network or hardware\n");
    const int64_t t0 = nowMs();
    testMercator();
    testNavState();
    testPngAndHttpRules();
    testTileCache();
    testCompose();
    testServicePolicy();
    testServiceFailures();
    std::printf("\n%s (%d failure%s, %.1f s)\n", g_fails ? "FAILED" : "ALL PASSED", g_fails, g_fails == 1 ? "" : "s",
                double(nowMs() - t0) / 1000.0);
    return g_fails ? 1 : 0;
}
