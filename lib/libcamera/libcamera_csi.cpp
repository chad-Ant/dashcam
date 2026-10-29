#include "libcamera_csi.h"

namespace dashcam::camera {

// Analogue gain floor for the rig's IMX296 (InnoMaker module), in Argus units
// (1.0 = the driver's minimum, GAIN register 16 = 1.6 dB; 12.5 = register 200,
// the DT maximum).  Below about register 64 the sensor resets itself mid-stream:
// it drops off I2C for ~170 ms, comes back with its power-on defaults, and Argus
// times out (waitCsiFrameStart).  The hazard rises steeply as the gain falls and
// does not follow exposure, register traffic, GPU load or the driver build; bright
// scenes hit it because AE drives the gain to the floor.  Heat is the main
// factor: a module cooled to a ~34 C die ran gain register 16 clean (2026-10-29).
// 4.0 (register 64) passed a 30-min daylight soak where the default range faulted
// within 1-3 min, but a midday two-camera lane_drive soak in the sun still reset
// 6 times at 64 (and 3 times at stream start): a mitigation, not a fix.
// A "gainrange" attribute set before start() replaces this range.
static constexpr const char* kGainRangeFloor = "4 12.5";

ERROR_CODE Camera_CSI::pipelineError() const {
    return ERROR_CODE::CSI_PIPELINE_ERROR;
}

std::string Camera_CSI::buildPipelineString(const cameraVideoFormat& fmt,
                                             uint32_t frNum, uint32_t frDen) const {
    // Sensor runs at its native mode (nvarguscamerasrc will not ISP-scale to an
    // arbitrary output, nor accept an arbitrary framerate cap on this rig).
    std::string pipe =
        "nvarguscamerasrc name=camerasrc sensor-id=" + std::to_string(info_.deviceId) +
        " gainrange=\"" + kGainRangeFloor + "\"" +
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

    // NVMM NV12 → BGRx on the VIC, then the CPU drops the padding byte.
    return pipe + captureBranch("nvvidconv ! video/x-raw, format=(string)BGRx"
                                " ! videoconvert ! video/x-raw, format=(string)BGR");
}

const char* Camera_CSI::cameraTypeTag() const { return "CSI"; }

Camera_CSI::Camera_CSI(const cameraInfo& camera)
    : Camera_GST(camera) {
}

} // namespace dashcam::camera
