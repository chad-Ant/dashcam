#include "libcamera_csi.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <ctime>

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
           " srctee. ! queue ! nvvidconv ! video/x-raw, format=(string)BGRx"
           " ! videoconvert ! video/x-raw, format=(string)BGR"
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

void Camera_CSI::stop() {
    {
        std::lock_guard<std::mutex> lock(overlayMutex_);
        ovTopLeft_ = ovTopRight_ = ovBottomLeft_ = ovBottomRight_ = nullptr;
    }
    Camera_GST::stop();
}

// ─── overlay API ─────────────────────────────────────────────────────────────

void Camera_CSI::setOverlayData(const OverlayData& data) {
    std::lock_guard<std::mutex> lock(overlayMutex_);
    overlayData_ = data;
    if (ovTopLeft_) {
        updateTextOverlays(data);
    }
}

OverlayData Camera_CSI::getOverlayData() const {
    std::lock_guard<std::mutex> lock(overlayMutex_);
    return overlayData_;
}

// ─── textoverlay helpers ──────────────────────────────────────────────────────

static const char* headingToCardinal(float deg) {
    // 8-point compass; each sector is 45°, offset by 22.5° so N spans [-22.5, 22.5].
    int sector = static_cast<int>((deg + 22.5f) / 45.0f) % 8;
    static const char* const names[8] = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" };
    return names[sector < 0 ? sector + 8 : sector];
}

// configureTextOverlay — replaced by inline pipeline-string properties in createRecordingBin().
// static void configureTextOverlay(GstElement* elem, const char* halign, const char* valign) {
//     gst_util_set_object_arg(G_OBJECT(elem), "halignment", halign);
//     gst_util_set_object_arg(G_OBJECT(elem), "valignment",  valign);
//     g_object_set(G_OBJECT(elem),
//                  "font-desc",         "Monospace Bold 14",
//                  "color",             (guint)0xFFFFFFFF,
//                  "shaded-background", (gboolean)TRUE,
//                  "xpad",              (gint)12,
//                  "ypad",              (gint)8,
//                  NULL);
// }

void Camera_CSI::updateTextOverlays(const OverlayData& od) {
    // UTC timestamp
    time_t epochSec = static_cast<time_t>(od.timestampMs / 1000LL);
    struct tm tmBuf;
    gmtime_r(&epochSec, &tmBuf);
    char dateBuf[16], timeBuf[16];
    std::strftime(dateBuf, sizeof(dateBuf), "%Y-%m-%d", &tmBuf);
    std::strftime(timeBuf, sizeof(timeBuf), "%H:%M:%S UTC", &tmBuf);

    char buf[128];

    // Top-left: speed / acceleration
    std::snprintf(buf, sizeof(buf), "SPD %.1f km/h\nACC %+.1f m/s2",
                  static_cast<double>(od.speedKmh),
                  static_cast<double>(od.accelerationMs2));
    g_object_set(G_OBJECT(ovTopLeft_), "text", buf, NULL);

    // Top-right: heading + cardinal
    std::snprintf(buf, sizeof(buf), "HDG %03.0f %s",
                  static_cast<double>(od.headingDeg),
                  headingToCardinal(od.headingDeg));
    g_object_set(G_OBJECT(ovTopRight_), "text", buf, NULL);

    // Bottom-left: lat / lon / alt
    std::snprintf(buf, sizeof(buf), "LAT %.6f %c\nLON %.6f %c\nALT %.1f m",
                  std::abs(od.latitude),  od.latitude  >= 0.0 ? 'N' : 'S',
                  std::abs(od.longitude), od.longitude >= 0.0 ? 'E' : 'W',
                  od.altitudeM);
    g_object_set(G_OBJECT(ovBottomLeft_), "text", buf, NULL);

    // Bottom-right: date / time
    std::snprintf(buf, sizeof(buf), "%s\n%s", dateBuf, timeBuf);
    g_object_set(G_OBJECT(ovBottomRight_), "text", buf, NULL);
}

// ─── recording bin ────────────────────────────────────────────────────────────

GstElement* Camera_CSI::createRecordingBin(const std::string& filename) {
    // ghost_unlinked_pads=TRUE: GStreamer auto-wraps nvvidconv's unlinked "sink"
    // pad as a ghost pad named "sink" on the bin — no manual ghost-pad code needed.
    // filename is set via g_object_set below (not in the string) to handle paths
    // with spaces or other characters that would break the description parser.
    static const char* binDesc =
        "nvvidconv name=conv ! video/x-raw,format=(string)I420 "
        "! textoverlay name=ov_tl halignment=left  valignment=top    "
          "font-desc=\"Monospace Bold 14\" color=4294967295 shaded-background=true xpad=12 ypad=8 "
        "! textoverlay name=ov_tr halignment=right valignment=top    "
          "font-desc=\"Monospace Bold 14\" color=4294967295 shaded-background=true xpad=12 ypad=8 "
        "! textoverlay name=ov_bl halignment=left  valignment=bottom "
          "font-desc=\"Monospace Bold 14\" color=4294967295 shaded-background=true xpad=12 ypad=8 "
        "! textoverlay name=ov_br halignment=right valignment=bottom "
          "font-desc=\"Monospace Bold 14\" color=4294967295 shaded-background=true xpad=12 ypad=8 "
        "! x264enc tune=zerolatency speed-preset=ultrafast bitrate=4000 key-int-max=60 "
        "! h264parse ! mp4mux ! filesink name=fsink";

    GError*     err = nullptr;
    GstElement* bin = gst_parse_bin_from_description(binDesc, TRUE, &err);
    if (!bin || err) {
        if (err) g_error_free(err);
        if (bin) gst_object_unref(bin);
        return nullptr;
    }

    // Set the output path via g_object_set — safe for paths containing spaces.
    GstElement* fsink = gst_bin_get_by_name(GST_BIN(bin), "fsink");
    if (fsink) {
        g_object_set(G_OBJECT(fsink), "location", filename.c_str(), NULL);
        gst_object_unref(fsink);
    }

    // gst_bin_get_by_name returns an owned ref (+1).  Immediately release it so
    // the bin remains the sole owner; the raw pointers stay valid for the bin's
    // lifetime, matching the non-owning semantics used by setOverlayData/stop().
    GstElement* ovTL = gst_bin_get_by_name(GST_BIN(bin), "ov_tl");
    GstElement* ovTR = gst_bin_get_by_name(GST_BIN(bin), "ov_tr");
    GstElement* ovBL = gst_bin_get_by_name(GST_BIN(bin), "ov_bl");
    GstElement* ovBR = gst_bin_get_by_name(GST_BIN(bin), "ov_br");

    if (!ovTL || !ovTR || !ovBL || !ovBR) {
        if (ovTL) gst_object_unref(ovTL);
        if (ovTR) gst_object_unref(ovTR);
        if (ovBL) gst_object_unref(ovBL);
        if (ovBR) gst_object_unref(ovBR);
        gst_object_unref(bin);
        return nullptr;
    }

    gst_object_unref(ovTL);
    gst_object_unref(ovTR);
    gst_object_unref(ovBL);
    gst_object_unref(ovBR);

    {
        std::lock_guard<std::mutex> lock(overlayMutex_);
        ovTopLeft_     = ovTL;
        ovTopRight_    = ovTR;
        ovBottomLeft_  = ovBL;
        ovBottomRight_ = ovBR;
        updateTextOverlays(overlayData_);
    }

    return bin;
}
