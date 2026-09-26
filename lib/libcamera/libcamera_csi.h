/**
 * @file libcamera_csi.h
 * @brief MIPI CSI-2 camera implementation for NVIDIA Jetson via GStreamer / Argus.
 *
 * Camera_CSI extends Camera_GST with an nvarguscamerasrc-based pipeline.  It
 * feeds inference branches (e.g. liblanedetector) and captureFrame(); it is not
 * a recording source — librecord records a UVC camera's own compressed stream
 * and refuses Argus's raw frames (recording them would need a software encoder).
 *
 * Typical usage (as dashcam_v0_2 runs its lane camera):
 * @code
 *   std::vector<cameraInfo> list;
 *   getCameraList(list);
 *   Camera_CSI cam(list[0]);
 *   cam.setAttributeDictionary(dict);
 *   cam.setOutputResolution(1280, 720, 20.0f);         // optional VIC downscale / rate cap
 *
 *   // Optional branch, e.g. an inference bin; leaky = true never stalls the tee:
 *   cam.addBranch("lanes", laneDetector.createBin(), true);
 *
 *   cam.open();
 *   cam.setCameraVideoFormat(0);
 *   cam.start();
 *
 *   while (running) cam.captureFrame(buf, size, written);
 *
 *   cam.stop();
 *   cam.close();
 * @endcode
 *
 * @note Requires GStreamer ≥ 1.20 (JetPack 6.2 / Ubuntu 22.04 ships 1.20.x).
 * @note Orin Nano has no NVENC; all encoding is software (x264enc).
 */

#ifndef LIBCAMERA_CSI_H
#define LIBCAMERA_CSI_H

#include "libcamera_gst.h"

namespace dashcam::camera {

/**
 * @brief iCamera implementation for MIPI CSI-2 sensors on NVIDIA Jetson.
 *
 * Supplies the Camera_GST hooks to build an nvarguscamerasrc pipeline:
 * @verbatim
 *   nvarguscamerasrc name=camerasrc sensor-id=N
 *     ! video/x-raw(memory:NVMM), NV12, WxH, fps
 *     ! tee name=srctee
 *   srctee. ! queue ! nvvidconv ! BGRx ! appsink name=mysink
 *   srctee. ! queue ! valve ! <branch0>
 *   ...
 * @endverbatim
 *
 * Sensor attributes (exposure, gain, AE/AWB lock) can be set before or during
 * capture.  Pre-start attributes are queued and applied in start() before the
 * pipeline transitions to PLAYING, so the first captured frame already uses
 * the requested settings.
 */
class Camera_CSI : public Camera_GST {
protected:
    /** @brief Returns ERROR_CODE::CSI_PIPELINE_ERROR. */
    ERROR_CODE pipelineError() const override;

    /**
     * @brief Build the nvarguscamerasrc pipeline string.
     *
     * Uses @c info_.deviceId as the Argus sensor-id and @c fmt for resolution
     * and framerate.  The default appsink branch converts NV12 (NVMM) to BGRx
     * in system memory.
     */
    std::string buildPipelineString(const cameraVideoFormat& fmt,
                                    uint32_t frNum, uint32_t frDen) const override;

    /**
     * @brief Returns "CSI"; scopes AttributeDictionary resolution to CSI entries.
     *
     * Attributes are applied by the base Camera_GST::applyAttributeGStreamer()
     * against nvarguscamerasrc.  Supported properties include exposuretimerange,
     * gainrange, aelock, awblock, wbmode, saturation, and the TNR/EE families.
     */
    const char* cameraTypeTag() const override;

public:
    /**
     * @brief Construct a Camera_CSI bound to the given device descriptor.
     * @param[in] camera  cameraInfo from getCameraList(); must describe a CSI camera.
     */
    explicit Camera_CSI(const cameraInfo& camera);

    ~Camera_CSI() override = default;
};

} // namespace dashcam::camera

#endif // LIBCAMERA_CSI_H
