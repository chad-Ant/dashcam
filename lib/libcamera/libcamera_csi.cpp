#include "libcamera_csi.h"

namespace dashcam::camera {

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
           " ! videoconvert ! video/x-raw, format=(string)BGR"
           " ! appsink name=mysink drop=true max-buffers=1 emit-signals=false sync=false";
}

void Camera_CSI::applyAttributeGStreamer(const std::string& name, const std::string& value) {
    if (!camera_src_) {
        status_.currentError = pipelineError();
        return;
    }
    const auto* entry = dict_.resolve(name, "CSI");
    if (!entry || !applyGstProperty(camera_src_, *entry, value)) {
        status_.currentError = ERROR_CODE::INVALID_ATTRIBUTE;
        return;
    }
    status_.currentError = ERROR_CODE::NONE;
}

Camera_CSI::Camera_CSI(const cameraInfo& camera)
    : Camera_GST(camera) {
}

} // namespace dashcam::camera
