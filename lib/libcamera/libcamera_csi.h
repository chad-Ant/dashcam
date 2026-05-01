/**
 * @file libcamera_csi.h
 * @brief MIPI CSI-2 camera implementation for NVIDIA Jetson via GStreamer / Argus.
 *
 * Camera_CSI extends Camera_GST with an nvarguscamerasrc-based pipeline and
 * a thread-safe telemetry overlay data store for recording branches.
 *
 * Typical usage:
 * @code
 *   std::vector<cameraInfo> list;
 *   getCameraList(list);
 *   Camera_CSI cam(list[0]);
 *
 *   GstElement* recBin = cam.createRecordingBin("/data/clip.mp4");
 *   cam.addBranch("recording", recBin);
 *   cam.open();
 *   cam.setCameraVideoFormat(0);
 *   cam.start();
 *
 *   cam.setBranchEnabled("recording", true);
 *   cam.setOverlayData({lat, lon, altM, speedKmh, accelMs2, headingDeg, tsMs});
 *
 *   while (running) cam.captureFrame(buf, size, written);
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
#include <mutex>
#include <string>

/**
 * @brief iCamera implementation for MIPI CSI-2 sensors on NVIDIA Jetson.
 *
 * Supplies the Camera_GST hooks to build an nvarguscamerasrc pipeline:
 * @verbatim
 *   nvarguscamerasrc name=camerasrc sensor-id=N
 *     ! video/x-raw(memory:NVMM), NV12, WxH, fps
 *     ! tee name=srctee
 *   srctee. ! queue ! nvvidconv ! BGRx ! videoconvert ! BGR
 *          ! appsink name=mysink
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
     * and framerate.  The default branch converts NV12 (NVMM) to BGR in system
     * memory before the appsink.
     */
    std::string buildPipelineString(const cameraVideoFormat& fmt,
                                    uint32_t frNum, uint32_t frDen) const override;

    /**
     * @brief Apply a CSI sensor attribute via g_object_set on nvarguscamerasrc.
     *
     * Supported names (case-insensitive):
     *   - "exposure time, absolute" | "exposure" → exposuretimerange (nanoseconds)
     *   - "gain"                                 → gainrange
     *   - "auto exposure"                        → aelock  (0 = unlocked, non-0 = locked)
     *   - "white balance, automatic"             → awblock (0 = unlocked, non-0 = locked)
     */
    void applyAttributeGStreamer(const std::string& name, const std::string& value) override;

private:
    OverlayData        overlayData_;   ///< Latest telemetry; written by setOverlayData(), read by getOverlayData().
    mutable std::mutex overlayMutex_;  ///< Guards overlayData_ and ov*_ pointers for cross-thread access.

    /// Non-owning pointers to the four textoverlay elements inside the recording bin.
    /// Valid only while the pipeline is RUNNING; nulled under overlayMutex_ in stop().
    GstElement* ovTopLeft_     = nullptr;
    GstElement* ovTopRight_    = nullptr;
    GstElement* ovBottomLeft_  = nullptr;
    GstElement* ovBottomRight_ = nullptr;

    /**
     * @brief Format telemetry into strings and push them to the four overlay elements.
     *
     * @pre  Called under overlayMutex_ with all four ov*_ pointers non-null.
     * @param[in] od  Telemetry snapshot to render.
     */
    void updateTextOverlays(const OverlayData& od);

public:
    /**
     * @brief Construct a Camera_CSI bound to the given device descriptor.
     * @param[in] camera  cameraInfo from getCameraList(); must describe a CSI camera.
     */
    explicit Camera_CSI(const cameraInfo& camera);

    ~Camera_CSI() override = default;

    /**
     * @brief Stop the capture pipeline and null out textoverlay element pointers.
     *
     * Nulls the four ov*_ pointers under overlayMutex_ before delegating to
     * Camera_GST::stop(), ensuring setOverlayData() cannot call g_object_set
     * on elements that are being destroyed.
     */
    void stop() override;

    /**
     * @brief Update the telemetry overlay data from any thread.
     *
     * Thread-safe.  If the recording bin is active (pipeline RUNNING), the four
     * textoverlay elements are updated immediately via g_object_set.
     *
     * @param[in] data  New telemetry values to store.
     */
    void setOverlayData(const OverlayData& data);

    /**
     * @brief Return a snapshot of the current telemetry overlay data.
     *
     * Thread-safe.
     *
     * @return Copy of the most recently written OverlayData.
     */
    OverlayData getOverlayData() const;

    /**
     * @brief Create a self-contained recording GstBin that writes an MP4 file.
     *
     * The returned bin accepts NV12 (NVMM) video on its ghost sink pad and
     * internally converts to I420 system memory before rendering the overlay.
     * The internal chain is:
     * @verbatim
     *   nvvidconv ! video/x-raw,format=I420
     *     ! textoverlay(top-left)
     *     ! textoverlay(top-right)
     *     ! textoverlay(bottom-left)
     *     ! textoverlay(bottom-right)
     *     ! x264enc ! h264parse ! mp4mux ! filesink
     * @endverbatim
     *
     * Four GStreamer @c textoverlay elements burn telemetry directly into each
     * passing frame.  Their @c text properties are updated in-place on every
     * setOverlayData() call — no per-frame callbacks, no extra library
     * dependencies beyond @c gst-plugins-base.
     *
     * Corner layout:
     *   - Top-left:     speed, acceleration
     *   - Top-right:    heading + cardinal
     *   - Bottom-left:  latitude, longitude, altitude
     *   - Bottom-right: UTC date, UTC time
     *
     * The bin can be handed directly to addBranch() and started/stopped
     * together with the rest of the pipeline.  The file is finalised (mp4mux
     * index written) when the pipeline transitions to NULL state.
     *
     * @param[in] filename  Absolute or relative path for the output MP4 file.
     * @return Newly created GstBin (floating reference); nullptr on failure.
     *         Ownership transfers to the pipeline via addBranch() / gst_bin_add().
     * @note   Must be called before start().
     */
    GstElement* createRecordingBin(const std::string& filename);
};

#endif // LIBCAMERA_CSI_H
