#include "libcamera_csi.h"

namespace dashcam::camera {

ERROR_CODE Camera_CSI::pipelineError() const {
    return ERROR_CODE::CSI_PIPELINE_ERROR;
}

std::string Camera_CSI::buildPipelineString(const cameraVideoFormat& fmt,
                                             uint32_t frNum, uint32_t frDen) const {
    // Sensor runs at its native mode (nvarguscamerasrc will not ISP-scale to an
    // arbitrary output, nor accept an arbitrary framerate cap on this rig).
    std::string pipe =
        "nvarguscamerasrc name=camerasrc sensor-id=" + std::to_string(info_.deviceId) +
        " ! video/x-raw(memory:NVMM), width=" + std::to_string(fmt.width) +
        ", height=" + std::to_string(fmt.height) +
        ", format=(string)NV12, framerate=" + std::to_string(frNum) + "/" + std::to_string(frDen);

    // Optional whole-output downscale + framerate cap BEFORE the tee, so the
    // appsink feed and every branch share the scaled stream.  nvvidconv scales on
    // the VIC (~free) and videorate drops frames — both stay on NVMM.
    const bool scale = (outWidth_ > 0 && outHeight_ > 0);
    if (scale) {
        pipe += " ! nvvidconv ! video/x-raw(memory:NVMM), width=" + std::to_string(outWidth_) +
                ", height=" + std::to_string(outHeight_) + ", format=(string)NV12";
    }
    if (outFps_ > 0.0f) {
        uint32_t on, od;
        computeFpsRational(outFps_, on, od);
        pipe += " ! videorate ! video/x-raw(memory:NVMM)";
        if (scale)
            pipe += ", width=" + std::to_string(outWidth_) + ", height=" + std::to_string(outHeight_);
        pipe += ", format=(string)NV12, framerate=" + std::to_string(on) + "/" + std::to_string(od);
    }

    pipe += " ! tee name=srctee"
            " srctee. ! queue max-size-buffers=" + std::to_string(params_.captureQueueDepth) +
            " leaky=2 ! nvvidconv ! video/x-raw, format=(string)BGRx"
            " ! videoconvert ! video/x-raw, format=(string)BGR"
            " ! appsink name=mysink drop=true max-buffers=" + std::to_string(params_.appsinkMaxBuffers) +
            " emit-signals=false sync=false";
    return pipe;
}

const char* Camera_CSI::cameraTypeTag() const { return "CSI"; }

Camera_CSI::Camera_CSI(const cameraInfo& camera)
    : Camera_GST(camera) {
}

} // namespace dashcam::camera
