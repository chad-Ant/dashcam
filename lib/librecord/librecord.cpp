#include "librecord.h"
#include <cairo/cairo.h>
#include <gst/video/video.h>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>

namespace dashcam::record {

// ─── file-local log helper ────────────────────────────────────────────────────

static void doLog(const dashcam::log::LogCallback& cb, dashcam::log::LogLevel lvl,
                  const char* fmt, ...) {
    if (!cb) return;
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    cb(lvl, buf);
}

// ─── Cairo helpers (file-local) ───────────────────────────────────────────────

static const char* headingToCardinal(float deg) {
    // 8-point compass; each sector is 45°, offset by 22.5° so N spans [−22.5, 22.5].
    int sector = static_cast<int>((deg + 22.5f) / 45.0f) % 8;
    static const char* const names[8] = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" };
    return names[sector < 0 ? sector + 8 : sector];
}

// Parse a Pango-style font description ("Family Bold Italic") into Cairo primitives.
static void selectCairoFont(cairo_t* cr, const std::string& fontFace) {
    cairo_font_weight_t weight = CAIRO_FONT_WEIGHT_NORMAL;
    cairo_font_slant_t  slant  = CAIRO_FONT_SLANT_NORMAL;
    std::string family = fontFace;

    struct Suffix { const char* str; cairo_font_weight_t w; cairo_font_slant_t s; };
    static const Suffix kSuffixes[] = {
        { "Bold Italic",  CAIRO_FONT_WEIGHT_BOLD,   CAIRO_FONT_SLANT_ITALIC  },
        { "Bold Oblique", CAIRO_FONT_WEIGHT_BOLD,   CAIRO_FONT_SLANT_OBLIQUE },
        { "Italic",       CAIRO_FONT_WEIGHT_NORMAL, CAIRO_FONT_SLANT_ITALIC  },
        { "Oblique",      CAIRO_FONT_WEIGHT_NORMAL, CAIRO_FONT_SLANT_OBLIQUE },
        { "Bold",         CAIRO_FONT_WEIGHT_BOLD,   CAIRO_FONT_SLANT_NORMAL  },
    };
    for (const auto& sx : kSuffixes) {
        std::size_t len = std::strlen(sx.str);
        if (family.size() > len &&
            family.compare(family.size() - len, len, sx.str) == 0) {
            weight = sx.w;
            slant  = sx.s;
            family.resize(family.size() - len);
            while (!family.empty() && family.back() == ' ') family.pop_back();
            break;
        }
    }
    cairo_select_font_face(cr, family.c_str(), slant, weight);
}

// Render multi-line text with a semi-transparent background box at one corner.
// rightAligned/bottomAligned select which corner; frameW/H are the video dimensions.
static void drawCornerLabel(cairo_t* cr, const char* text,
                             bool rightAligned, bool bottomAligned,
                             double frameW, double frameH,
                             double bgOpacity) {
    const double xpad = 12.0, ypad = 8.0;

    cairo_font_extents_t fe;
    cairo_font_extents(cr, &fe);

    std::vector<std::string> lines;
    for (const char* p = text; *p; ) {
        const char* nl = std::strchr(p, '\n');
        lines.emplace_back(p, nl ? static_cast<std::size_t>(nl - p) : std::strlen(p));
        p = nl ? nl + 1 : p + std::strlen(p);
    }

    double maxW = 0.0;
    for (const auto& ln : lines) {
        cairo_text_extents_t te;
        cairo_text_extents(cr, ln.c_str(), &te);
        maxW = std::max(maxW, te.x_advance);
    }

    double boxW = maxW + 2.0 * xpad;
    double boxH = fe.height * static_cast<double>(lines.size()) + 2.0 * ypad;
    double boxX = rightAligned  ? frameW - boxW : 0.0;
    double boxY = bottomAligned ? frameH - boxH : 0.0;

    cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, bgOpacity);
    cairo_rectangle(cr, boxX, boxY, boxW, boxH);
    cairo_fill(cr);

    cairo_set_source_rgb(cr, 1.0, 1.0, 1.0);
    for (std::size_t i = 0; i < lines.size(); ++i) {
        cairo_move_to(cr, boxX + xpad,
                      boxY + ypad + fe.ascent + static_cast<double>(i) * fe.height);
        cairo_show_text(cr, lines[i].c_str());
    }
}

// ─── Recorder ─────────────────────────────────────────────────────────────────

Recorder::Recorder() = default;

Recorder::~Recorder() {
    disconnect();
}

void Recorder::setLogCallback(dashcam::log::LogCallback cb) {
    log_ = std::move(cb);
}

// ─── overlay data ─────────────────────────────────────────────────────────────

void Recorder::setOverlayData(const OverlayData& data) {
    std::lock_guard<std::mutex> lock(overlayMutex_);
    overlayData_ = data;
}

OverlayData Recorder::getOverlayData() const {
    std::lock_guard<std::mutex> lock(overlayMutex_);
    return overlayData_;
}

void Recorder::setOverlayConfig(const dashcam::config::OverlayConfig& cfg) {
    std::lock_guard<std::mutex> lock(overlayMutex_);
    overlayConfig_ = cfg;
}

// ─── Cairo signal callbacks ───────────────────────────────────────────────────

void Recorder::onCairoDraw(GstElement* /*overlay*/, cairo_t* cr,
                            GstClockTime /*ts*/, GstClockTime /*dur*/,
                            gpointer user_data) {
    static_cast<Recorder*>(user_data)->renderOverlay(cr);
}

void Recorder::onCairoCapsChanged(GstElement* /*overlay*/, GstCaps* caps,
                                   gpointer user_data) {
    GstVideoInfo info;
    if (!gst_video_info_from_caps(&info, caps)) return;
    auto* self = static_cast<Recorder*>(user_data);
    std::lock_guard<std::mutex> lock(self->overlayMutex_);
    self->videoWidth_  = GST_VIDEO_INFO_WIDTH(&info);
    self->videoHeight_ = GST_VIDEO_INFO_HEIGHT(&info);
}

void Recorder::renderOverlay(cairo_t* cr) {
    OverlayData od;
    dashcam::config::OverlayConfig ocfg;
    double w, h;
    {
        std::lock_guard<std::mutex> lock(overlayMutex_);
        if (!overlayConfig_.enabled) return;
        od   = overlayData_;
        ocfg = overlayConfig_;
        w    = static_cast<double>(videoWidth_);
        h    = static_cast<double>(videoHeight_);
    }
    if (w == 0.0 || h == 0.0) return;

    time_t epochSec = static_cast<time_t>(od.timestampMs / 1000LL);
    struct tm tmBuf;
    gmtime_r(&epochSec, &tmBuf);
    char dateBuf[16], timeBuf[16];
    std::strftime(dateBuf, sizeof(dateBuf), "%Y-%m-%d", &tmBuf);
    std::strftime(timeBuf, sizeof(timeBuf), "%H:%M:%S UTC", &tmBuf);

    char buf[128];

    selectCairoFont(cr, ocfg.fontFace);
    cairo_set_font_size(cr, static_cast<double>(ocfg.fontSize));

    const double opacity = static_cast<double>(ocfg.backgroundOpacity);

    std::snprintf(buf, sizeof(buf), "SPD %.1f km/h\nACC %+.1f m/s2",
                  static_cast<double>(od.speedKmh),
                  static_cast<double>(od.accelerationMs2));
    drawCornerLabel(cr, buf, false, false, w, h, opacity);

    std::snprintf(buf, sizeof(buf), "HDG %03.0f %s",
                  static_cast<double>(od.headingDeg), headingToCardinal(od.headingDeg));
    drawCornerLabel(cr, buf, true, false, w, h, opacity);

    std::snprintf(buf, sizeof(buf), "LAT %.6f %c\nLON %.6f %c\nALT %.1f m",
                  std::abs(od.latitude),  od.latitude  >= 0.0 ? 'N' : 'S',
                  std::abs(od.longitude), od.longitude >= 0.0 ? 'E' : 'W',
                  od.altitudeM);
    drawCornerLabel(cr, buf, false, true, w, h, opacity);

    std::snprintf(buf, sizeof(buf), "%s\n%s", dateBuf, timeBuf);
    drawCornerLabel(cr, buf, true, true, w, h, opacity);
}

// ─── lifecycle ────────────────────────────────────────────────────────────────

void Recorder::disconnect() {
    std::lock_guard<std::mutex> lock(overlayMutex_);
    if (cairoOverlay_) {
        g_signal_handler_disconnect(cairoOverlay_, cairoDrawId_);
        g_signal_handler_disconnect(cairoOverlay_, cairoCapsId_);
    }
    cairoOverlay_ = nullptr;
    cairoDrawId_  = 0;
    cairoCapsId_  = 0;
    videoWidth_   = 0;
    videoHeight_  = 0;
}

// ─── recording bin ────────────────────────────────────────────────────────────

GstElement* Recorder::createRecordingBin(const std::string& filename,
                                          uint32_t frNum, uint32_t frDen,
                                          const dashcam::config::EncoderConfig& enc) {
    disconnect();

    std::string x264Opts =
        " speed-preset=" + enc.speedPreset +
        " bitrate=" + std::to_string(enc.bitrate) +
        " key-int-max=" + std::to_string(enc.keyIntMax);
    if (!enc.tune.empty())
        x264Opts += " tune=" + enc.tune;

    const std::string binDesc =
        "nvvidconv name=conv ! video/x-raw,format=(string)BGRx "
        "! cairooverlay name=cairoov "
        "! videoconvert ! video/x-raw,format=(string)I420 "
        "! queue max-size-buffers=3 leaky=0 "
        "! videorate ! video/x-raw,framerate=" +
        std::to_string(frNum) + "/" + std::to_string(frDen) + " "
        "! x264enc" + x264Opts + " insert-vui=true aud=true "
        "! h264parse ! matroskamux ! filesink name=fsink sync=false async=false";

    doLog(log_, dashcam::log::LogLevel::INFO,
          "createRecordingBin pipeline: %s", binDesc.c_str());

    GError*     err = nullptr;
    GstElement* bin = gst_parse_bin_from_description(binDesc.c_str(), TRUE, &err);
    if (!bin || err) {
        if (err) {
            doLog(log_, dashcam::log::LogLevel::ERROR,
                  "gst_parse_bin_from_description failed: %s", err->message);
            g_error_free(err);
        }
        if (bin) gst_object_unref(bin);
        return nullptr;
    }

    GstElement* fsink = gst_bin_get_by_name(GST_BIN(bin), "fsink");
    if (fsink) {
        g_object_set(G_OBJECT(fsink), "location", filename.c_str(), NULL);
        gst_object_unref(fsink);
    }

    // gst_bin_get_by_name returns an owned ref (+1).  Release it immediately so
    // the bin remains the sole owner; the raw pointer stays valid for the bin's
    // lifetime, matching the non-owning semantics documented for cairoOverlay_.
    GstElement* cairoOv = gst_bin_get_by_name(GST_BIN(bin), "cairoov");
    if (!cairoOv) {
        doLog(log_, dashcam::log::LogLevel::ERROR,
              "could not find cairooverlay element in recording bin");
        gst_object_unref(bin);
        return nullptr;
    }

    gulong drawId = g_signal_connect(cairoOv, "draw",
                                     G_CALLBACK(Recorder::onCairoDraw), this);
    gulong capsId = g_signal_connect(cairoOv, "caps-changed",
                                     G_CALLBACK(Recorder::onCairoCapsChanged), this);
    gst_object_unref(cairoOv);

    {
        std::lock_guard<std::mutex> lock(overlayMutex_);
        cairoOverlay_ = cairoOv;
        cairoDrawId_  = drawId;
        cairoCapsId_  = capsId;
    }

    return bin;
}

} // namespace dashcam::record
