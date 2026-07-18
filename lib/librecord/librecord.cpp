#include "librecord.h"
#include <cairo/cairo.h>
#include <gst/video/video.h>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>

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
// Called once in setOverlayConfig(); results are cached to avoid per-frame parsing.
static void parseFontFace(const std::string& fontFace,
                          std::string& family, int& weight, int& slant) {
    weight = CAIRO_FONT_WEIGHT_NORMAL;
    slant  = CAIRO_FONT_SLANT_NORMAL;
    family = fontFace;

    struct Suffix { const char* str; int w; int s; };
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
}

// Render multi-line text with a semi-transparent background box at one corner.
// rightAligned/bottomAligned select which corner; frameW/H are the video dimensions.
static void drawCornerLabel(cairo_t* cr, const char* text,
                             bool rightAligned, bool bottomAligned,
                             double frameW, double frameH,
                             double bgOpacity,
                             double xpad, double ypad) {

    cairo_font_extents_t fe;
    cairo_font_extents(cr, &fe);

    char lineBufs[4][128];
    int  nLines = 0;
    for (const char* p = text; *p && nLines < 4; ) {
        const char* nl  = std::strchr(p, '\n');
        std::size_t len = nl ? static_cast<std::size_t>(nl - p) : std::strlen(p);
        if (len >= sizeof(lineBufs[0])) len = sizeof(lineBufs[0]) - 1;
        std::memcpy(lineBufs[nLines], p, len);
        lineBufs[nLines][len] = '\0';
        ++nLines;
        p = nl ? nl + 1 : p + std::strlen(p);
    }

    double maxW = 0.0;
    for (int i = 0; i < nLines; ++i) {
        cairo_text_extents_t te;
        cairo_text_extents(cr, lineBufs[i], &te);
        maxW = std::max(maxW, te.x_advance);
    }

    double boxW = maxW + 2.0 * xpad;
    double boxH = fe.height * static_cast<double>(nLines) + 2.0 * ypad;
    double boxX = rightAligned  ? frameW - boxW : 0.0;
    double boxY = bottomAligned ? frameH - boxH : 0.0;

    cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, bgOpacity);
    cairo_rectangle(cr, boxX, boxY, boxW, boxH);
    cairo_fill(cr);

    cairo_set_source_rgb(cr, 1.0, 1.0, 1.0);
    for (int i = 0; i < nLines; ++i) {
        cairo_move_to(cr, boxX + xpad,
                      boxY + ypad + fe.ascent + static_cast<double>(i) * fe.height);
        cairo_show_text(cr, lineBufs[i]);
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
    parseFontFace(cfg.fontFace, cachedFontFamily_, cachedFontWeight_, cachedFontSlant_);
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
    std::string fontFamily;
    int         fontWeight, fontSlant;
    double w, h;
    {
        std::lock_guard<std::mutex> lock(overlayMutex_);
        if (!overlayConfig_.enabled) return;
        od         = overlayData_;
        ocfg       = overlayConfig_;
        fontFamily = cachedFontFamily_;
        fontWeight = cachedFontWeight_;
        fontSlant  = cachedFontSlant_;
        w          = static_cast<double>(videoWidth_);
        h          = static_cast<double>(videoHeight_);
    }
    if (w == 0.0 || h == 0.0) return;

    time_t epochSec = static_cast<time_t>(od.timestampMs / 1000LL);
    struct tm tmBuf;
    localtime_r(&epochSec, &tmBuf);
    char dateBuf[16], timeBuf[16];
    std::strftime(dateBuf, sizeof(dateBuf), "%Y-%m-%d", &tmBuf);
    std::strftime(timeBuf, sizeof(timeBuf), "%H:%M:%S %Z", &tmBuf);

    char buf[128];

    cairo_select_font_face(cr, fontFamily.c_str(),
                           static_cast<cairo_font_slant_t>(fontSlant),
                           static_cast<cairo_font_weight_t>(fontWeight));
    cairo_set_font_size(cr, static_cast<double>(ocfg.fontSize));

    const double opacity = static_cast<double>(ocfg.backgroundOpacity);
    const double padX    = static_cast<double>(ocfg.labelPadX);
    const double padY    = static_cast<double>(ocfg.labelPadY);

    std::snprintf(buf, sizeof(buf), "SPD %.1f km/h\nACC %+.1f m/s2",
                  static_cast<double>(od.speedKmh),
                  static_cast<double>(od.accelerationMs2));
    drawCornerLabel(cr, buf, false, false, w, h, opacity, padX, padY);

    std::snprintf(buf, sizeof(buf), "HDG %03.0f %s",
                  static_cast<double>(od.headingDeg), headingToCardinal(od.headingDeg));
    drawCornerLabel(cr, buf, true, false, w, h, opacity, padX, padY);

    std::snprintf(buf, sizeof(buf), "LAT %.6f %c\nLON %.6f %c\nALT %.1f m",
                  std::abs(od.latitude),  od.latitude  >= 0.0 ? 'N' : 'S',
                  std::abs(od.longitude), od.longitude >= 0.0 ? 'E' : 'W',
                  od.altitudeM);
    drawCornerLabel(cr, buf, false, true, w, h, opacity, padX, padY);

    std::snprintf(buf, sizeof(buf), "%s\n%s", dateBuf, timeBuf);
    drawCornerLabel(cr, buf, true, true, w, h, opacity, padX, padY);
}

// ─── lifecycle ────────────────────────────────────────────────────────────────

void Recorder::disconnect() {
    GstElement* cairoOv = nullptr;
    gulong drawId = 0, capsId = 0;
    {
        std::lock_guard<std::mutex> lock(overlayMutex_);
        cairoOv       = cairoOverlay_;
        drawId        = cairoDrawId_;
        capsId        = cairoCapsId_;
        cairoOverlay_ = nullptr;
        cairoDrawId_  = 0;
        cairoCapsId_  = 0;
        videoWidth_   = 0;
        videoHeight_  = 0;
    }
    if (cairoOv) {
        // We won the claim, so the element is still alive: drop the weak ref
        // (so onCairoOverlayDestroyed can't fire later on this Recorder) before
        // disconnecting the signal handlers.  Both calls are outside the lock —
        // g_signal_handler_disconnect blocks until any in-flight draw callback
        // returns, and that callback takes overlayMutex_, so holding it here
        // would deadlock.
        g_object_weak_unref(G_OBJECT(cairoOv),
                            &Recorder::onCairoOverlayDestroyed, this);
        g_signal_handler_disconnect(cairoOv, drawId);
        g_signal_handler_disconnect(cairoOv, capsId);
    }
}

void Recorder::onCairoOverlayDestroyed(gpointer user_data, GObject* /*where*/) {
    auto* self = static_cast<Recorder*>(user_data);
    std::lock_guard<std::mutex> lock(self->overlayMutex_);
    // The element is being finalised: drop our handle so disconnect() no-ops.
    // Do NOT g_signal_handler_disconnect or g_object_weak_unref here — the
    // element's handlers and weak refs are already being torn down.
    self->cairoOverlay_ = nullptr;
    self->cairoDrawId_  = 0;
    self->cairoCapsId_  = 0;
    self->videoWidth_   = 0;
    self->videoHeight_  = 0;
}

// ─── recording bin ────────────────────────────────────────────────────────────

GstElement* Recorder::createRecordingBin(const std::string& filename,
                                          uint32_t frNum, uint32_t frDen,
                                          const dashcam::config::EncoderConfig& enc,
                                          uint32_t queueDepth,
                                          SourceMemory srcMem,
                                          uint32_t outWidth, uint32_t outHeight,
                                          bool overlay) {
    disconnect();

    // enc.speedPreset / enc.tune are ConfigVar<std::string>: read them into
    // std::string locals first.  ConfigVar's implicit operator const T& is not
    // considered in `"literal" + configvar` (operator+ template deduction) nor
    // for member access like enc.tune.empty(), so use plain strings from here on.
    const std::string speedPreset = enc.speedPreset;
    const std::string tune        = enc.tune;
    std::string x264Opts =
        " speed-preset=" + speedPreset +
        " bitrate=" + std::to_string(enc.bitrate) +
        " key-int-max=" + std::to_string(enc.keyIntMax);
    if (!tune.empty())
        x264Opts += " tune=" + tune;

    // Optional inlet downscale: only when BOTH dimensions are given.  Scaling here
    // — before Cairo and the CPU encoder — sheds the most work, and (for NVMM) it
    // rides the VIC inside nvvidconv for free.  Keep the source aspect ratio to
    // avoid distortion.
    const bool scale = (outWidth > 0 && outHeight > 0);
    const std::string wh = scale
        ? ",width=(int)" + std::to_string(outWidth) + ",height=(int)" + std::to_string(outHeight)
        : std::string();

    // Inlet output format: BGRx when overlaying (Cairo draws on BGRx), else I420
    // straight into the encoder — skipping Cairo and a BGRx round-trip.
    const std::string inFmt = overlay ? "BGRx" : "I420";
    const std::string inletCaps = "video/x-raw,format=(string)" + inFmt + wh;

    // Inlet: nvvidconv for NVMM (CSI) — the only element that can pull buffers off
    // NVMM, and it scales on the VIC — or videoconvert (+videoscale when scaling)
    // for system memory (USB), which accepts any raw UVC format and skips a needless
    // VIC round-trip.
    std::string inletChain;
    if (srcMem == SourceMemory::System) {
        inletChain = scale
            ? "videoconvert name=conv ! videoscale ! " + inletCaps
            : "videoconvert name=conv ! " + inletCaps;
    } else {
        inletChain = "nvvidconv name=conv ! " + inletCaps;
    }

    // Cairo overlay stage (BGRx → draw → back to I420); omitted entirely when
    // overlay=false (the inlet already produced I420).
    const std::string overlayStage = overlay
        ? " ! cairooverlay name=cairoov ! videoconvert ! video/x-raw,format=(string)I420"
        : std::string();

    // videorate sits at the INLET, not in front of the encoder: everything
    // downstream of it (Cairo draw, BGRx→I420 videoconvert, x264) then runs at
    // the target rate instead of the sensor rate.  On a 60 fps sensor recorded
    // at 30 that halves the whole software chain's cost — significant on a box
    // with no NVENC.
    //
    // skip-to-first: the recording valve typically opens a couple of seconds
    // after the pipeline starts (camera warmup), so the first buffer arrives
    // with a non-zero running time.  Default videorate would backfill segment
    // start → first PTS with duplicates of that frame — a freeze-frame intro.
    // offset-to-zero then shifts the muxed timestamps back so the file still
    // starts at 0 instead of at the warmup offset.
    const std::string binDesc =
        inletChain +
        " ! videorate skip-to-first=true ! video/x-raw,framerate=" +
        std::to_string(frNum) + "/" + std::to_string(frDen) +
        overlayStage +
        " ! queue max-size-buffers=" + std::to_string(queueDepth) + " leaky=0 "
        "! x264enc" + x264Opts + " insert-vui=true aud=true "
        "! h264parse ! matroskamux offset-to-zero=true "
        "! filesink name=fsink sync=false async=false";

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
    if (!fsink) {
        // Should be impossible after a successful parse, but a bin without its
        // filesink location would only fail later at PLAYING with a confusing
        // "no location" error — fail loudly here instead.
        doLog(log_, dashcam::log::LogLevel::ERROR,
              "could not find filesink element in recording bin");
        gst_object_unref(bin);
        return nullptr;
    }
    g_object_set(G_OBJECT(fsink), "location", filename.c_str(), NULL);
    gst_object_unref(fsink);

    // No-overlay bin: no cairooverlay element, so no signals to wire up.
    // cairoOverlay_ stays null, so disconnect()/~Recorder are no-ops.
    if (!overlay)
        return bin;

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

    // Auto-null cairoOverlay_ if the element is finalised (pipeline torn down)
    // before disconnect() is called, so disconnect()/~Recorder never dereference
    // a freed element.  The element stays alive via the bin; the weak ref only
    // fires on its destruction.
    g_object_weak_ref(G_OBJECT(cairoOv),
                      &Recorder::onCairoOverlayDestroyed, this);
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
