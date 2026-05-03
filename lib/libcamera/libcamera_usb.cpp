#include "libcamera_usb.h"
#include <algorithm>
#include <cctype>
#include <linux/videodev2.h>

// ─── private helper ──────────────────────────────────────────────────────────

/// Map a V4L2 pixel-format four-CC to the GStreamer format string used in a
/// video/x-raw caps filter.  Returns an empty string for unknown codes; the
/// caller omits the format= field in that case and lets GStreamer negotiate.
static const char* v4l2FormatToGst(uint32_t pixelFormat) {
    switch (pixelFormat) {
        case V4L2_PIX_FMT_YUYV:   return "YUY2";
        case V4L2_PIX_FMT_UYVY:   return "UYVY";
        case V4L2_PIX_FMT_NV12:   return "NV12";
        case V4L2_PIX_FMT_NV21:   return "NV21";
        case V4L2_PIX_FMT_YUV420: return "I420";
        case V4L2_PIX_FMT_RGB24:  return "RGB";
        case V4L2_PIX_FMT_BGR24:  return "BGR";
        default:                   return "";
    }
}

// ─── Camera_GST hooks ────────────────────────────────────────────────────────

ERROR_CODE Camera_USB::pipelineError() const {
    return ERROR_CODE::USB_PIPELINE_ERROR;
}

std::string Camera_USB::buildPipelineString(const cameraVideoFormat& fmt,
                                             uint32_t frNum, uint32_t frDen) const {
    std::string formatCaps;
    if (fmt.pixelFormat == V4L2_PIX_FMT_MJPEG) {
        formatCaps = "image/jpeg, width=" + std::to_string(fmt.width) +
                     ", height=" + std::to_string(fmt.height) +
                     ", framerate=" + std::to_string(frNum) + "/" + std::to_string(frDen) +
                     " ! jpegdec ! video/x-raw";
    } else {
        const char* gstFmt = v4l2FormatToGst(fmt.pixelFormat);
        formatCaps = "video/x-raw";
        if (gstFmt[0] != '\0') {
            formatCaps += std::string(", format=(string)") + gstFmt;
        }
        formatCaps += ", width=" + std::to_string(fmt.width) +
                      ", height=" + std::to_string(fmt.height) +
                      ", framerate=" + std::to_string(frNum) + "/" + std::to_string(frDen);
    }

    return "v4l2src name=camerasrc device=" + info_.address +
           " ! " + formatCaps +
           " ! tee name=srctee"
           " srctee. ! queue ! videoconvert ! video/x-raw, format=(string)BGR"
           " ! appsink name=mysink drop=true max-buffers=1 emit-signals=false sync=false";
}

/// Supported attribute names (case-insensitive) and their v4l2src GObject property mappings:
///   "brightness"  → brightness  (integer; range depends on device)
///   "contrast"    → contrast    (integer; range depends on device)
///   "saturation"  → saturation  (integer; range depends on device)
void Camera_USB::applyAttributeGStreamer(const std::string& name, const std::string& value) {
    if (!camera_src_) {
        status_.currentError = pipelineError();
        return;
    }

    std::string lowerName = name;
    std::transform(lowerName.begin(), lowerName.end(), lowerName.begin(),
                   [](unsigned char c){ return std::tolower(c); });

    int intVal = 0;
    if (!safeStoi(value, intVal)) {
        status_.currentError = ERROR_CODE::INVALID_ATTRIBUTE;
        return;
    }

    if (lowerName == "brightness") {
        g_object_set(G_OBJECT(camera_src_), "brightness", intVal, NULL);
    } else if (lowerName == "contrast") {
        g_object_set(G_OBJECT(camera_src_), "contrast", intVal, NULL);
    } else if (lowerName == "saturation") {
        g_object_set(G_OBJECT(camera_src_), "saturation", intVal, NULL);
    } else {
        status_.currentError = ERROR_CODE::INVALID_ATTRIBUTE;
        return;
    }

    status_.currentError = ERROR_CODE::NONE;
}

// ─── lifecycle ───────────────────────────────────────────────────────────────

Camera_USB::Camera_USB(const cameraInfo& camera)
    : Camera_GST(camera) {
}
