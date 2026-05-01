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

/// Apply corner position and common visual style to a textoverlay element.
static void configureTextOverlay(GstElement* elem,
                                  const char* halign, const char* valign) {
    gst_util_set_object_arg(G_OBJECT(elem), "halignment", halign);
    gst_util_set_object_arg(G_OBJECT(elem), "valignment",  valign);
    g_object_set(G_OBJECT(elem),
                 "font-desc",         "Monospace Bold 14",
                 "color",             (guint)0xFFFFFFFF,   // white
                 "shaded-background", (gboolean)TRUE,       // semi-transparent backing
                 "xpad",              (gint)12,
                 "ypad",              (gint)8,
                 NULL);
}

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

// Keyframe interval for the recording encoder.
// At 60 fps (IMX296 1080p60) this produces one keyframe per second,
// keeping GOP short enough for fast seek on event clips without
// inflating bitrate the way a sub-second interval would.
static constexpr guint MAX_FRAMES_PER_KEYFRAME = 60;

GstElement* Camera_CSI::createRecordingBin(const std::string& filename) {
    GstElement* bin        = gst_bin_new(nullptr);
    GstElement* nvvidconv  = gst_element_factory_make("nvvidconv",   nullptr);
    GstElement* capsfilter = gst_element_factory_make("capsfilter",  nullptr);
    GstElement* ovTL       = gst_element_factory_make("textoverlay", nullptr);
    GstElement* ovTR       = gst_element_factory_make("textoverlay", nullptr);
    GstElement* ovBL       = gst_element_factory_make("textoverlay", nullptr);
    GstElement* ovBR       = gst_element_factory_make("textoverlay", nullptr);
    GstElement* encoder    = gst_element_factory_make("x264enc",     nullptr);
    GstElement* parser     = gst_element_factory_make("h264parse",   nullptr);
    GstElement* muxer      = gst_element_factory_make("mp4mux",      nullptr);
    GstElement* sink       = gst_element_factory_make("filesink",    nullptr);

    if (!bin || !nvvidconv || !capsfilter || !ovTL || !ovTR || !ovBL || !ovBR ||
        !encoder || !parser || !muxer || !sink) {
        GstElement* elems[] = { bin, nvvidconv, capsfilter, ovTL, ovTR, ovBL, ovBR,
                                 encoder, parser, muxer, sink };
        for (GstElement* e : elems) {
            if (e) { gst_object_ref_sink(e); gst_object_unref(e); }
        }
        return nullptr;
    }

    // nvvidconv converts NVMM NV12 → I420 system memory for textoverlay
    GstCaps* i420Caps = gst_caps_from_string("video/x-raw,format=(string)I420");
    g_object_set(G_OBJECT(capsfilter), "caps", i420Caps, NULL);
    gst_caps_unref(i420Caps);

    configureTextOverlay(ovTL, "left",  "top");
    configureTextOverlay(ovTR, "right", "top");
    configureTextOverlay(ovBL, "left",  "bottom");
    configureTextOverlay(ovBR, "right", "bottom");

    gst_util_set_object_arg(G_OBJECT(encoder), "tune",         "zerolatency");
    gst_util_set_object_arg(G_OBJECT(encoder), "speed-preset", "ultrafast");
    g_object_set(G_OBJECT(encoder), "bitrate", (guint)4000, NULL);
    g_object_set(G_OBJECT(encoder), "key-int-max", MAX_FRAMES_PER_KEYFRAME, NULL);

    g_object_set(G_OBJECT(sink), "location", filename.c_str(), NULL);

    gst_bin_add_many(GST_BIN(bin),
                     nvvidconv, capsfilter, ovTL, ovTR, ovBL, ovBR,
                     encoder, parser, muxer, sink, nullptr);

    if (!gst_element_link_many(nvvidconv, capsfilter, ovTL, ovTR, ovBL, ovBR,
                               encoder, parser, muxer, sink, nullptr)) {
        // Null the overlay pointers before freeing the bin — elements are owned
        // by the bin and will be freed by the unref, so any live setOverlayData()
        // caller must not reach them after this point.
        std::lock_guard<std::mutex> lock(overlayMutex_);
        ovTopLeft_ = ovTopRight_ = ovBottomLeft_ = ovBottomRight_ = nullptr;
        gst_object_unref(bin);
        return nullptr;
    }

    // Ghost sink pad on nvvidconv so the bin accepts input from the tee
    GstPad* sinkPad  = gst_element_get_static_pad(nvvidconv, "sink");
    GstPad* ghostPad = gst_ghost_pad_new("sink", sinkPad);
    gst_object_unref(sinkPad);
    gst_element_add_pad(bin, ghostPad);

    // Store non-owning pointers and prime the overlay text with current data
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
