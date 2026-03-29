#include "libcamera_csi.h"

Camera_CSI::Camera_CSI(const cameraInfo& camera) {
    info_ = camera;
}

Camera_CSI::~Camera_CSI() {
    if (isOpen()) {
        close();
    }
}
