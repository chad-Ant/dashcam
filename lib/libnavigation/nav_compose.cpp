// nav_compose.cpp — the 320x224 map frame (tiles, marker, north indicator,
// scale bar, status and attribution strips) and its JPEG encoder.

#include "libnavigation.h"
#include "nav_font.h"

#include <jpeglib.h>

#include <algorithm>
#include <cmath>
#include <csetjmp>
#include <cstdio>
#include <cstring>

namespace dashcam::navigation {

const char* netStateName(NetState s) {
    switch (s) {
    case NetState::Unknown: return "unknown";
    case NetState::Online:  return "online";
    case NetState::Offline: return "offline";
    case NetState::Denied:  return "denied";
    }
    return "?";
}

namespace {

constexpr double kPi = 3.14159265358979323846;

struct Rgb {
    uint8_t r, g, b;
};
constexpr Rgb kStatusBg{28, 28, 30};
constexpr Rgb kStatusFg{255, 255, 255};
constexpr Rgb kAttrBg{246, 246, 246};
constexpr Rgb kAttrFg{40, 40, 40};
constexpr Rgb kGridBg{226, 226, 222};
constexpr Rgb kGridLine{205, 205, 200};
constexpr Rgb kMissingBg{236, 236, 232};
constexpr Rgb kMarker{20, 100, 255};
constexpr Rgb kWhite{255, 255, 255};
constexpr Rgb kBlack{0, 0, 0};
constexpr Rgb kGrey{120, 120, 120};
constexpr Rgb kAmber{255, 190, 40};

/// Clipped drawing into one packed-RGB canvas.
class Canvas {
public:
    Canvas(std::vector<uint8_t>& px, int w, int h) : px_(px), w_(w), h_(h), cx1_(w), cy1_(h) {}
    int  w() const { return w_; }
    int  h() const { return h_; }
    void clip(int x0, int y0, int x1, int y1) { cx0_ = x0, cy0_ = y0, cx1_ = x1, cy1_ = y1; }
    void unclip() { cx0_ = 0, cy0_ = 0, cx1_ = w_, cy1_ = h_; }

    void put(int x, int y, Rgb c) {
        if (x < cx0_ || y < cy0_ || x >= cx1_ || y >= cy1_) return;
        uint8_t* p = &px_[(size_t(y) * w_ + x) * 3];
        p[0] = c.r, p[1] = c.g, p[2] = c.b;
    }
    void fill(int x, int y, int w, int h, Rgb c) {
        for (int yy = std::max(y, cy0_); yy < std::min(y + h, cy1_); ++yy)
            for (int xx = std::max(x, cx0_); xx < std::min(x + w, cx1_); ++xx) put(xx, yy, c);
    }
    /// A 256x256 RGB tile with its top-left at (x, y).
    void blitTile(const uint8_t* tile, int x, int y) {
        const int ya = std::max(y, cy0_), yb = std::min(y + kTileSize, cy1_);
        const int xa = std::max(x, cx0_), xb = std::min(x + kTileSize, cx1_);
        if (xa >= xb) return;
        for (int yy = ya; yy < yb; ++yy)
            std::memcpy(&px_[(size_t(yy) * w_ + xa) * 3], tile + (size_t(yy - y) * kTileSize + (xa - x)) * 3,
                        size_t(xb - xa) * 3);
    }
    void disc(double cx, double cy, double r, Rgb c) {
        for (int y = int(std::floor(cy - r)); y <= int(std::ceil(cy + r)); ++y)
            for (int x = int(std::floor(cx - r)); x <= int(std::ceil(cx + r)); ++x)
                if ((x + 0.5 - cx) * (x + 0.5 - cx) + (y + 0.5 - cy) * (y + 0.5 - cy) <= r * r) put(x, y, c);
    }
    void ring(double cx, double cy, double r, double t, Rgb c) {
        for (int y = int(std::floor(cy - r)); y <= int(std::ceil(cy + r)); ++y)
            for (int x = int(std::floor(cx - r)); x <= int(std::ceil(cx + r)); ++x) {
                const double d = std::hypot(x + 0.5 - cx, y + 0.5 - cy);
                if (d <= r && d >= r - t) put(x, y, c);
            }
    }
    void triangle(const double (&p)[3][2], Rgb c) {
        const double minX = std::min({p[0][0], p[1][0], p[2][0]}), maxX = std::max({p[0][0], p[1][0], p[2][0]});
        const double minY = std::min({p[0][1], p[1][1], p[2][1]}), maxY = std::max({p[0][1], p[1][1], p[2][1]});
        auto edge = [](const double* a, const double* b, double x, double y) {
            return (b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0]);
        };
        for (int y = int(std::floor(minY)); y <= int(std::ceil(maxY)); ++y)
            for (int x = int(std::floor(minX)); x <= int(std::ceil(maxX)); ++x) {
                const double px = x + 0.5, py = y + 0.5;
                const double e0 = edge(p[0], p[1], px, py), e1 = edge(p[1], p[2], px, py), e2 = edge(p[2], p[0], px, py);
                if ((e0 >= 0 && e1 >= 0 && e2 >= 0) || (e0 <= 0 && e1 <= 0 && e2 <= 0)) put(x, y, c);
            }
    }

    /// UTF-8 text (ASCII, ° and ©) with the 5x7 font; returns the width drawn.
    int text(int x, int y, const std::string& s, int scale, Rgb c) {
        int pen = x;
        for (char32_t cp : decode(s)) {
            const uint8_t* g = font::glyph(cp);
            for (int col = 0; col < font::kGlyphW; ++col)
                for (int row = 0; row < font::kGlyphH; ++row)
                    if (g[col] & (1u << row)) fill(pen + col * scale, y + row * scale, scale, scale, c);
            pen += (font::kGlyphW + 1) * scale;
        }
        return pen - x;
    }
    /// Text with a one-pixel halo, readable over map imagery.
    void haloText(int x, int y, const std::string& s, int scale, Rgb fg, Rgb halo) {
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx)
                if (dx || dy) text(x + dx, y + dy, s, scale, halo);
        text(x, y, s, scale, fg);
    }
    static int textWidth(const std::string& s, int scale) {
        return int(decode(s).size()) * (font::kGlyphW + 1) * scale - scale;
    }

private:
    static std::vector<char32_t> decode(const std::string& s) {
        std::vector<char32_t> out;
        for (size_t i = 0; i < s.size();) {
            const unsigned char c = static_cast<unsigned char>(s[i]);
            if (c < 0x80) {
                out.push_back(c);
                ++i;
            } else if ((c & 0xE0) == 0xC0 && i + 1 < s.size()) {
                out.push_back(char32_t(((c & 0x1F) << 6) | (static_cast<unsigned char>(s[i + 1]) & 0x3F)));
                i += 2;
            } else {   // longer sequences: one '?' per code point
                size_t n = (c & 0xF0) == 0xE0 ? 3 : (c & 0xF8) == 0xF0 ? 4 : 1;
                out.push_back(U'?');
                i += n;
            }
        }
        return out;
    }

    std::vector<uint8_t>& px_;
    int w_, h_;
    int cx0_ = 0, cy0_ = 0;
    int cx1_, cy1_;
};

void drawGrid(Canvas& c, int y0, int h) {
    c.fill(0, y0, c.w(), h, kGridBg);
    for (int x = 0; x < c.w(); x += 32) c.fill(x, y0, 1, h, kGridLine);
    for (int y = y0; y < y0 + h; y += 32) c.fill(0, y, c.w(), 1, kGridLine);
}

void drawMarker(Canvas& c, const NavView& v, double cx, double cy) {
    switch (v.marker) {
    case Marker::None:
        return;
    case Marker::Dot:
        c.disc(cx, cy, 8.5, kWhite);
        c.disc(cx, cy, 6.0, kMarker);
        return;
    case Marker::LastKnown:
        c.disc(cx, cy, 8.5, kWhite);
        c.ring(cx, cy, 7.0, 2.5, kGrey);
        return;
    case Marker::Arrow: {
        // North-up: the course rotates the arrow clockwise from screen-up.
        const double a = double(v.courseDeg) * kPi / 180.0;
        auto rot = [&](double x, double y, double out[2]) {   // (x right, y up) → screen
            out[0] = cx + x * std::cos(a) + y * std::sin(a);
            out[1] = cy - (-x * std::sin(a) + y * std::cos(a));
        };
        double halo[3][2], body[3][2];
        rot(0, 14, halo[0]), rot(-10, -10, halo[1]), rot(10, -10, halo[2]);
        rot(0, 11, body[0]), rot(-7.5, -8, body[1]), rot(7.5, -8, body[2]);
        c.triangle(halo, kWhite);
        c.triangle(body, kMarker);
        return;
    }
    }
}

void drawNorth(Canvas& c, int mapY0) {
    const double x = c.w() - 14, y = mapY0 + 14;
    const double halo[3][2] = {{x, y - 10}, {x - 7, y + 5}, {x + 7, y + 5}};
    const double body[3][2] = {{x, y - 8}, {x - 5, y + 4}, {x + 5, y + 4}};
    c.triangle(halo, kWhite);
    c.triangle(body, kBlack);
    c.haloText(int(x) - 2, int(y) + 8, "N", 1, kBlack, kWhite);
}

void drawScale(Canvas& c, double lat, int zoom, int mapBottom) {
    static const int kNice[] = {5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000};
    const double mpp = metersPerPixel(lat, zoom);
    int meters = kNice[0];
    for (int m : kNice)
        if (m / mpp <= 90.0) meters = m;
    const int px = std::max(1, int(std::lround(meters / mpp)));
    const int x = 6, y = mapBottom - 8;
    c.fill(x - 1, y - 1, px + 2, 5, kWhite);
    c.fill(x, y, px, 3, kBlack);
    c.fill(x - 1, y - 5, 3, 9, kWhite);
    c.fill(x, y - 4, 1, 7, kBlack);
    c.fill(x + px - 2, y - 5, 3, 9, kWhite);
    c.fill(x + px - 1, y - 4, 1, 7, kBlack);
    char label[24];
    if (meters >= 1000) std::snprintf(label, sizeof(label), "%d km", meters / 1000);
    else std::snprintf(label, sizeof(label), "%d m", meters);
    c.haloText(x + 3, y - 13, label, 1, kBlack, kWhite);
}

std::string statusText(const NavView& v) {
    char b[64];
    switch (v.state) {
    case PositionState::NoData:     return "GPS: no data";
    case PositionState::NoReceiver: return "GPS: no receiver";
    case PositionState::Acquiring:  return "GPS: searching";
    case PositionState::LastKnown:  return "GPS: last known";
    case PositionState::Live: break;
    }
    const char* fix = v.fixType >= 3 ? "3D" : "2D";
    char speed[16];
    if (std::isfinite(v.speedKmh)) std::snprintf(speed, sizeof(speed), "%.0fkm/h", double(std::max(0.0f, v.speedKmh)));
    else std::snprintf(speed, sizeof(speed), "--km/h");
    if (v.marker == Marker::Arrow)
        std::snprintf(b, sizeof(b), "%s %s %03.0f\xC2\xB0", fix, speed, double(v.courseDeg));
    else
        std::snprintf(b, sizeof(b), "%s %s Dir --", fix, speed);
    return b;
}

} // namespace

ComposeResult composeMap(const NavView& v, int zoom, const MapLayout& L, const TileLookup& lookup, NetState net,
                         const std::string& attribution, std::vector<uint8_t>& rgb) {
    ComposeResult res;
    rgb.assign(size_t(L.width) * L.height * 3, 0);
    Canvas c(rgb, L.width, L.height);
    const int mapY0 = L.statusH;
    const int mapH  = L.height - L.statusH - L.attributionH;
    const int mapY1 = mapY0 + mapH;

    // ── map area ──
    c.clip(0, mapY0, L.width, mapY1);
    const bool havePos = v.marker != Marker::None;
    if (!havePos) {
        drawGrid(c, mapY0, mapH);
        const char* why = v.state == PositionState::NoReceiver ? "GPS receiver not found"
                          : v.state == PositionState::Acquiring ? "Waiting for GPS fix"
                                                                : "Waiting for GPS";
        const int tw = Canvas::textWidth(why, 2);
        c.haloText((L.width - tw) / 2, mapY0 + mapH / 2 - 7, why, 2, kBlack, kWhite);
    } else {
        const WorldPoint centre = project(v.latitude, v.longitude, zoom);
        drawGrid(c, mapY0, mapH);
        const auto tiles = tilesForView(centre, zoom, L.width, mapH);
        res.tilesNeeded  = int(tiles.size());
        for (const TilePlacement& t : tiles) {
            const uint8_t* px = lookup ? lookup(t.key) : nullptr;
            if (px) {
                c.blitTile(px, t.dstX, mapY0 + t.dstY);
                ++res.tilesDrawn;
            } else {
                c.fill(t.dstX, mapY0 + t.dstY, kTileSize, kTileSize, kMissingBg);
                for (int d = 0; d < kTileSize * 2; d += 16)   // hatch: missing, not blank land
                    for (int i = 0; i < kTileSize; ++i)
                        if (d - i >= 0 && d - i < kTileSize) c.put(t.dstX + i, mapY0 + t.dstY + d - i, kGridLine);
            }
        }
        res.mapComplete = res.tilesDrawn == res.tilesNeeded;
        drawMarker(c, v, L.width / 2.0, mapY0 + mapH / 2.0);
        drawNorth(c, mapY0);
        drawScale(c, v.latitude, zoom, mapY1);
        if (v.state == PositionState::LastKnown) {
            const int tw = Canvas::textWidth("Last known position", 1);
            c.haloText((L.width - tw) / 2, mapY0 + 6, "Last known position", 1, kBlack, kAmber);
        }
        if (!res.mapComplete) {
            const char* why = net == NetState::Denied    ? "Map access denied"
                              : net == NetState::Offline ? "Map offline - cached area only"
                                                         : "Loading map";
            const int tw = Canvas::textWidth(why, 1);
            c.haloText(L.width - tw - 6, mapY1 - 12, why, 1, kBlack, kWhite);
        }
    }
    c.unclip();

    // ── status strip ──
    c.fill(0, 0, L.width, L.statusH, kStatusBg);
    c.text(3, (L.statusH - 14) / 2, statusText(v), 2, v.state == PositionState::Live ? kStatusFg : kAmber);
    const char* netLabel = net == NetState::Online ? "NET" : net == NetState::Offline ? "OFFLINE"
                           : net == NetState::Denied ? "DENIED" : "";
    if (*netLabel) {
        const int tw = Canvas::textWidth(netLabel, 1);
        c.text(L.width - tw - 3, (L.statusH - 7) / 2, netLabel, 1, net == NetState::Online ? kStatusFg : kAmber);
    }

    // ── attribution strip: whenever imagery may be visible ──
    c.fill(0, mapY1, L.width, L.attributionH, kAttrBg);
    const int aw = Canvas::textWidth(attribution, 1);
    c.text(std::max(2, L.width - aw - 3), mapY1 + (L.attributionH - 7) / 2, attribution, 1, kAttrFg);
    return res;
}

// ─── JPEG ─────────────────────────────────────────────────────────────────────

namespace {
struct FixedDest {
    jpeg_destination_mgr pub;
    uint8_t*             buf;
    size_t               cap;
    bool                 overflow;
    uint8_t              sink[4096];   ///< Absorbs output past the cap, so the encoder finishes normally.
};
void destInit(j_compress_ptr c) {
    auto* d                  = reinterpret_cast<FixedDest*>(c->dest);
    d->pub.next_output_byte  = d->buf;
    d->pub.free_in_buffer    = d->cap;
}
boolean destEmpty(j_compress_ptr c) {
    auto* d                  = reinterpret_cast<FixedDest*>(c->dest);
    d->overflow              = true;
    d->pub.next_output_byte  = d->sink;
    d->pub.free_in_buffer    = sizeof(d->sink);
    return TRUE;
}
void destTerm(j_compress_ptr) {}

struct JpegError {
    jpeg_error_mgr pub;
    jmp_buf        jb;
};
void jpegErrorExit(j_common_ptr c) { std::longjmp(reinterpret_cast<JpegError*>(c->err)->jb, 1); }
void jpegSilent(j_common_ptr) {}
} // namespace

bool encodeMapJpeg(const uint8_t* rgb, int w, int h, int quality, uint8_t* out, size_t capacity, size_t& outSize) {
    outSize = 0;
    if (!rgb || !out || w <= 0 || h <= 0 || capacity == 0) return false;
    jpeg_compress_struct c;
    JpegError            err;
    FixedDest            dest;
    c.err                  = jpeg_std_error(&err.pub);
    err.pub.error_exit     = jpegErrorExit;
    err.pub.output_message = jpegSilent;
    if (setjmp(err.jb)) {   // no non-trivial C++ object lives across this
        jpeg_destroy_compress(&c);
        return false;
    }
    jpeg_create_compress(&c);
    dest.pub.init_destination    = destInit;
    dest.pub.empty_output_buffer = destEmpty;
    dest.pub.term_destination    = destTerm;
    dest.buf                     = out;
    dest.cap                     = capacity;
    dest.overflow                = false;
    c.dest                       = &dest.pub;
    c.image_width                = JDIMENSION(w);
    c.image_height               = JDIMENSION(h);
    c.input_components           = 3;
    c.in_color_space             = JCS_RGB;
    jpeg_set_defaults(&c);
    jpeg_set_quality(&c, std::clamp(quality, 1, 100), TRUE);
    for (int i = 0; i < 3; ++i) {   // 4:4:4: small map text stays legible
        c.comp_info[i].h_samp_factor = 1;
        c.comp_info[i].v_samp_factor = 1;
    }
    c.optimize_coding = TRUE;   // smaller, still baseline sequential Huffman
    jpeg_start_compress(&c, TRUE);
    while (c.next_scanline < c.image_height) {
        JSAMPROW row = const_cast<JSAMPROW>(rgb + size_t(c.next_scanline) * size_t(w) * 3);
        jpeg_write_scanlines(&c, &row, 1);
    }
    jpeg_finish_compress(&c);
    const bool overflow = dest.overflow;
    const size_t used   = capacity - dest.pub.free_in_buffer;
    jpeg_destroy_compress(&c);
    if (overflow) return false;
    outSize = used;
    return true;
}

} // namespace dashcam::navigation
