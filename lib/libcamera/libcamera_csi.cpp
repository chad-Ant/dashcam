#include "libcamera_csi.h"
#include <algorithm>
#include <cairo/cairo.h>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <gst/video/video.h>
#include <string>
#include <vector>

// ─── Camera_GST hooks ────────────────────────────────────────────────────────

ERROR_CODE Camera_CSI::pipelineError() const {
    return ERROR_CODE::CSI_PIPELINE_ERROR;
}

std::string Camera_CSI::buildPipelineString(const cameraVideoFormat& fmt,
                                             uint32_t frNum, uint32_t frDen) const {
    return "nvarguscamerasrc name=camerasrc sensor-id=" + std::to_string(info_.deviceId) +
           " ! video/x-raw(memory:NVMM), width=" + std::to_string(fmt.width) +
           ", height=" + std::to_string(fmt.height) +
           ", format=(string)NV12, framerate=" + std::to_string(frNum) + "/" + std::to_string(frDen) +
           " ! tee name=srctee"
           " srctee. ! queue max-size-buffers=2 leaky=2 ! nvvidconv ! video/x-raw, format=(string)BGRx"
           " ! appsink name=mysink drop=true max-buffers=1 emit-signals=false sync=false";
}

/// Supported attribute names (case-insensitive) and their nvarguscamerasrc mappings:
///   "exposure time, absolute" | "exposure"  → exposuretimerange  (nanoseconds)
///   "gain"                                  → gainrange
///   "auto exposure"                         → aelock             (0 = AE unlocked, non-0 = locked)
///   "white balance, automatic"              → awblock            (0 = AWB unlocked, non-0 = locked)
void Camera_CSI::applyAttributeGStreamer(const std::string& name, const std::string& value) {
    if (!camera_src_) {
        status_.currentError = pipelineError();
        return;
    }

    std::string lowerName = name;
    std::transform(lowerName.begin(), lowerName.end(), lowerName.begin(),
                   [](unsigned char c){ return std::tolower(c); });

    if (lowerName == "exposure time, absolute" || lowerName == "exposure") {
        std::string rangeStr = value + " " + value;
        g_object_set(G_OBJECT(camera_src_), "exposuretimerange", rangeStr.c_str(), NULL);
    } else if (lowerName == "gain") {
        std::string rangeStr = value + " " + value;
        g_object_set(G_OBJECT(camera_src_), "gainrange", rangeStr.c_str(), NULL);
    } else if (lowerName == "auto exposure") {
        int ae_mode = 0;
        if (!safeStoi(value, ae_mode)) {
            status_.currentError = ERROR_CODE::INVALID_ATTRIBUTE;
            return;
        }
        g_object_set(G_OBJECT(camera_src_), "aelock", ae_mode == 0 ? TRUE : FALSE, NULL);
    } else if (lowerName == "white balance, automatic") {
        int awb_mode = 0;
        if (!safeStoi(value, awb_mode)) {
            status_.currentError = ERROR_CODE::INVALID_ATTRIBUTE;
            return;
        }
        g_object_set(G_OBJECT(camera_src_), "awblock", awb_mode == 0 ? TRUE : FALSE, NULL);
    } else {
        status_.currentError = ERROR_CODE::INVALID_ATTRIBUTE;
        return;
    }

    status_.currentError = ERROR_CODE::NONE;
}

// ─── lifecycle ───────────────────────────────────────────────────────────────

Camera_CSI::Camera_CSI(const cameraInfo& camera)
    : Camera_GST(camera) {
}

void Camera_CSI::disconnectOverlay() {
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

void Camera_CSI::stop() {
    disconnectOverlay();
    Camera_GST::stop();
}

void Camera_CSI::close() {
    disconnectOverlay();
    Camera_GST::close();
}

// ─── overlay API ─────────────────────────────────────────────────────────────

void Camera_CSI::setOverlayData(const OverlayData& data) {
    std::lock_guard<std::mutex> lock(overlayMutex_);
    overlayData_ = data;
}

OverlayData Camera_CSI::getOverlayData() const {
    std::lock_guard<std::mutex> lock(overlayMutex_);
    return overlayData_;
}

// ─── Cairo overlay helpers ────────────────────────────────────────────────────

static const char* headingToCardinal(float deg) {
    // 8-point compass; each sector is 45°, offset by 22.5° so N spans [-22.5, 22.5].
    int sector = static_cast<int>((deg + 22.5f) / 45.0f) % 8;
    static const char* const names[8] = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" };
    return names[sector < 0 ? sector + 8 : sector];
}

// Render multi-line text with a semi-transparent background box at one corner.
// rightAligned/bottomAligned select which corner; frameW/H are the video dimensions.
static void drawCornerLabel(cairo_t* cr, const char* text,
                             bool rightAligned, bool bottomAligned,
                             double frameW, double frameH) {
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

    cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 0.5);
    cairo_rectangle(cr, boxX, boxY, boxW, boxH);
    cairo_fill(cr);

    cairo_set_source_rgb(cr, 1.0, 1.0, 1.0);
    for (std::size_t i = 0; i < lines.size(); ++i) {
        cairo_move_to(cr, boxX + xpad,
                      boxY + ypad + fe.ascent + static_cast<double>(i) * fe.height);
        cairo_show_text(cr, lines[i].c_str());
    }
}

void Camera_CSI::onCairoDraw(GstElement* /*overlay*/, cairo_t* cr,
                              GstClockTime /*ts*/, GstClockTime /*dur*/,
                              gpointer user_data) {
    static_cast<Camera_CSI*>(user_data)->renderOverlay(cr);
}

void Camera_CSI::onCairoCapsChanged(GstElement* /*overlay*/, GstCaps* caps,
                                     gpointer user_data) {
    GstVideoInfo info;
    if (!gst_video_info_from_caps(&info, caps)) return;
    auto* self = static_cast<Camera_CSI*>(user_data);
    std::lock_guard<std::mutex> lock(self->overlayMutex_);
    self->videoWidth_  = GST_VIDEO_INFO_WIDTH(&info);
    self->videoHeight_ = GST_VIDEO_INFO_HEIGHT(&info);
}

void Camera_CSI::renderOverlay(cairo_t* cr) {
    OverlayData od;
    double w, h;
    {
        std::lock_guard<std::mutex> lock(overlayMutex_);
        od = overlayData_;
        w  = static_cast<double>(videoWidth_);
        h  = static_cast<double>(videoHeight_);
    }
    if (w == 0.0 || h == 0.0) return;

    time_t epochSec = static_cast<time_t>(od.timestampMs / 1000LL);
    struct tm tmBuf;
    gmtime_r(&epochSec, &tmBuf);
    char dateBuf[16], timeBuf[16];
    std::strftime(dateBuf, sizeof(dateBuf), "%Y-%m-%d", &tmBuf);
    std::strftime(timeBuf, sizeof(timeBuf), "%H:%M:%S UTC", &tmBuf);

    char buf[128];

    cairo_select_font_face(cr, "Monospace", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size(cr, 14.0);

    std::snprintf(buf, sizeof(buf), "SPD %.1f km/h\nACC %+.1f m/s2",
                  static_cast<double>(od.speedKmh),
                  static_cast<double>(od.accelerationMs2));
    drawCornerLabel(cr, buf, false, false, w, h);

    std::snprintf(buf, sizeof(buf), "HDG %03.0f %s",
                  static_cast<double>(od.headingDeg), headingToCardinal(od.headingDeg));
    drawCornerLabel(cr, buf, true, false, w, h);

    std::snprintf(buf, sizeof(buf), "LAT %.6f %c\nLON %.6f %c\nALT %.1f m",
                  std::abs(od.latitude),  od.latitude  >= 0.0 ? 'N' : 'S',
                  std::abs(od.longitude), od.longitude >= 0.0 ? 'E' : 'W',
                  od.altitudeM);
    drawCornerLabel(cr, buf, false, true, w, h);

    std::snprintf(buf, sizeof(buf), "%s\n%s", dateBuf, timeBuf);
    drawCornerLabel(cr, buf, true, true, w, h);
}

// ─── recording bin ────────────────────────────────────────────────────────────

GstElement* Camera_CSI::createRecordingBin(const std::string& filename,
                                            uint32_t frNum, uint32_t frDen) {
    // Must be called before start(): the returned bin is meant to be passed to
    // addBranch(), which itself rejects RUNNING.  Calling here while RUNNING
    // would also overwrite the live cairoOverlay_ pointer with a handle to an
    // element that isn't in any pipeline.
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (status_.status == CAMERA_STATUS::RUNNING) {
            status_.currentError = ERROR_CODE::INVALID_ATTRIBUTE;
            return nullptr;
        }
    }

    const std::string binDesc =
        "nvvidconv name=conv ! video/x-raw,format=(string)BGRx "
        "! cairooverlay name=cairoov "
        "! videoconvert ! video/x-raw,format=(string)I420 "
        "! queue max-size-buffers=3 leaky=0 "
        "! videorate ! video/x-raw,framerate=" + std::to_string(frNum) + "/" + std::to_string(frDen) + " "
        "! x264enc speed-preset=ultrafast bitrate=4000 key-int-max=60 insert-vui=true aud=true "
        "! h264parse ! matroskamux ! filesink name=fsink sync=false async=false";

    g_print("createRecordingBin pipeline:\n  %s\n", binDesc.c_str());

    GError*     err = nullptr;
    GstElement* bin = gst_parse_bin_from_description(binDesc.c_str(), TRUE, &err);
    if (!bin || err) {
        if (err) {
            g_printerr("createRecordingBin: gst_parse_bin_from_description failed: %s\n",
                       err->message);
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
    // lifetime, matching the non-owning semantics in stop()/close().
    GstElement* cairoOv = gst_bin_get_by_name(GST_BIN(bin), "cairoov");
    if (!cairoOv) {
        g_printerr("createRecordingBin: could not find cairooverlay element\n");
        gst_object_unref(bin);
        return nullptr;
    }

    gulong drawId = g_signal_connect(cairoOv, "draw",
                                     G_CALLBACK(Camera_CSI::onCairoDraw), this);
    gulong capsId = g_signal_connect(cairoOv, "caps-changed",
                                     G_CALLBACK(Camera_CSI::onCairoCapsChanged), this);
    gst_object_unref(cairoOv);

    {
        std::lock_guard<std::mutex> lock(overlayMutex_);
        cairoOverlay_ = cairoOv;
        cairoDrawId_  = drawId;
        cairoCapsId_  = capsId;
    }

    return bin;
}
