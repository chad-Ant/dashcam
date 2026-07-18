/**
 * @file libcamera_usb.h
 * @brief USB UVC camera implementation via GStreamer / V4L2.
 *
 * Camera_USB extends Camera_GST with a v4l2src-based pipeline that handles
 * both MJPEG and raw (YUYV/NV12/…) UVC capture modes.
 *
 * Typical usage:
 * @code
 *   std::vector<cameraInfo> list;
 *   getCameraList(list);
 *   Camera_USB cam(list[0]);
 *
 *   cam.addBranch("inference", myInferenceBin);
 *   cam.open();
 *   cam.setCameraVideoFormat(0);
 *   cam.start();
 *
 *   cam.setBranchEnabled("inference", true);
 *   while (running) cam.captureFrame(buf, size, written);
 *   cam.stop();
 *   cam.close();
 * @endcode
 *
 * @note captureFrame() always returns BGR frames regardless of the sensor
 *       pixel format; videoconvert handles the colour-space conversion.
 */

#ifndef LIBCAMERA_USB_H
#define LIBCAMERA_USB_H

#include "libcamera_gst.h"
#include <string>

namespace dashcam::camera {

/**
 * @brief iCamera implementation for USB UVC cameras via GStreamer v4l2src.
 *
 * Supplies the Camera_GST hooks to build a v4l2src pipeline.  MJPEG frames
 * are decoded by jpegdec before the tee; raw formats are passed through with
 * the GStreamer format string derived from the V4L2 four-CC:
 * @verbatim
 *   v4l2src name=camerasrc device=/dev/videoN
 *     ! [image/jpeg … ! jpegdec ! video/x-raw]   ← MJPEG path
 *       OR
 *     ! [video/x-raw, format=YUY2, WxH, fps]     ← raw path
 *     ! tee name=srctee
 *   srctee. ! queue ! videoconvert ! BGR
 *          ! appsink name=mysink
 *   srctee. ! queue ! valve ! <branch0>
 *   ...
 * @endverbatim
 */
class Camera_USB : public Camera_GST {
protected:
    /** @brief Returns ERROR_CODE::USB_PIPELINE_ERROR. */
    ERROR_CODE pipelineError() const override;

    /**
     * @brief Build the v4l2src pipeline string.
     *
     * Selects the MJPEG or raw path based on @c fmt.pixelFormat.  The raw path
     * maps the V4L2 four-CC to a GStreamer format string; unknown codes omit
     * the @c format= field and let GStreamer negotiate.
     */
    std::string buildPipelineString(const cameraVideoFormat& fmt,
                                    uint32_t frNum, uint32_t frDen) const override;

    /**
     * @brief Returns "USB"; scopes AttributeDictionary resolution to USB entries.
     *
     * Attributes are applied by the base Camera_GST::applyAttributeGStreamer()
     * against v4l2src.  Supported names (case-insensitive) include brightness,
     * contrast, and saturation; unrecognised names set INVALID_ATTRIBUTE.
     */
    const char* cameraTypeTag() const override;

public:
    /**
     * @brief Construct a Camera_USB bound to the given device descriptor.
     * @param[in] camera  cameraInfo from getCameraList(); must describe a USB camera.
     */
    explicit Camera_USB(const cameraInfo& camera);

    ~Camera_USB() override = default;
};

} // namespace dashcam::camera

#endif // LIBCAMERA_USB_H
