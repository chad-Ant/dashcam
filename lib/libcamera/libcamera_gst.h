/**
 * @file libcamera_gst.h
 * @brief GStreamer-based camera base class shared by all concrete drivers.
 *
 * Camera_GST implements the complete iCamera lifecycle (open/start/stop/close,
 * captureFrame, attribute queuing, multi-sink branching) using a template-method
 * pattern.  Concrete subclasses supply three hooks:
 *
 *  - buildPipelineString() — returns the GStreamer pipeline description string.
 *  - applyAttributeGStreamer() — translates attribute name/value to g_object_set calls.
 *  - pipelineError() — returns the driver-specific ERROR_CODE for pipeline failures.
 *
 * All GStreamer state (element handles, tee pads, valve map) is owned and
 * managed here; subclasses do not touch it directly.
 */

#ifndef LIBCAMERA_GST_H
#define LIBCAMERA_GST_H

#include "libcamera.h"
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

/**
 * @brief Telemetry payload written into a recording branch overlay.
 *
 * Passed to Camera_CSI::setOverlayData() from any thread.  Fields are all
 * plain data; the struct is copied under a mutex on each write and read.
 */
struct OverlayData {
    double  latitude       = 10.7725;      ///< WGS-84 latitude in decimal degrees.
    double  longitude      = 106.6581;     ///< WGS-84 longitude in decimal degrees.
    double  altitudeM      = 52.3;         ///< Altitude above mean sea level in metres.
    float   speedKmh       = 1.5f;         ///< Ground speed in kilometres per hour.
    float   accelerationMs2 = 0.0f;        ///< Longitudinal acceleration in m/s² (positive = forward).
    float   headingDeg     = 90.0f;        ///< True heading in degrees (0 = North, clockwise).
    int64_t timestampMs    = 1777633580;   ///< UNIX epoch timestamp in milliseconds.
};

/**
 * @brief Abstract GStreamer base class implementing the iCamera interface.
 *
 * Builds a GStreamer pipeline of the form:
 * @verbatim
 *   <subclass-defined source + caps + tee name=srctee>
 *   srctee. ! queue ! <optional format conversion> ! BGR
 *          ! appsink name=mysink              ← captureFrame() source
 *   srctee. ! queue ! valve ! <branch0>       ← addBranch() branches
 *   srctee. ! queue ! valve ! <branch1>
 *   ...
 * @endverbatim
 *
 * The pipeline string (up to and including the appsink) is produced by
 * buildPipelineString().  All other pipeline management (tee pad allocation,
 * valve insertion, state transitions, teardown) is handled here.
 *
 * Attributes set before start() are queued in pendingAttributes_ and applied
 * to camera_src_ inside start(), before the pipeline transitions to PLAYING.
 */
class Camera_GST : public iCamera {
protected:
    cameraInfo   info_;    ///< Static device descriptor supplied at construction.
    cameraStatus status_;  ///< Mutable runtime state; updated by lifecycle methods.

    GstElement* pipeline_;   ///< Top-level GstPipeline; nullptr when not running.
    GstElement* camera_src_; ///< Source element retrieved by name "camerasrc"; used for attribute writes.
    GstElement* appsink_;    ///< Default BGR output sink; pulled by captureFrame().
    GstElement* tee_;        ///< Fan-out tee element; nullptr when not running.

    /// Branches registered via addBranch() before start().  The pipeline takes
    /// ownership of each GstElement* via gst_bin_add() during start().
    std::vector<std::pair<std::string, GstElement*>> branches_;

    /// Request pads obtained from tee_ for each external branch.
    /// Released via gst_element_release_request_pad() in teardownPipeline().
    std::vector<GstPad*> teePads_;

    std::map<std::string, std::string> pendingAttributes_; ///< Attributes queued before start().
    std::map<std::string, GstElement*> branchValves_;      ///< Non-owning valve pointers keyed by branch name.
    mutable std::mutex stateMutex_;                        ///< Protects status_ and pendingAttributes_ for concurrent access.
    
    /**
     * @brief Return the ERROR_CODE used when the GStreamer pipeline fails.
     *
     * Called from start() and open() on pipeline construction or state-change
     * failure.  Each driver returns its own code (CSI_PIPELINE_ERROR,
     * USB_PIPELINE_ERROR, …) so callers can distinguish the origin.
     *
     * @return Driver-specific pipeline error code.
     */
    virtual ERROR_CODE pipelineError() const = 0;

    /**
     * @brief Build the complete GStreamer pipeline description string.
     *
     * The returned string must:
     *  - Name the source element @c camerasrc  (retrieved later via gst_bin_get_by_name).
     *  - Name the tee element    @c srctee.
     *  - Name the appsink        @c mysink, configured with
     *    @c drop=true max-buffers=1 emit-signals=false sync=false.
     *  - Output @c video/x-raw,format=BGR on the mysink branch.
     *
     * @param[in] fmt    Active capture format (width, height, pixelFormat, …).
     * @param[in] frNum  Framerate numerator (already GCD-reduced).
     * @param[in] frDen  Framerate denominator (already GCD-reduced).
     * @return GStreamer pipeline description string suitable for gst_parse_launch().
     */
    virtual std::string buildPipelineString(const cameraVideoFormat& fmt,
                                            uint32_t frNum, uint32_t frDen) const = 0;

    /**
     * @brief Translate a name/value attribute pair into a GStreamer property write.
     *
     * Called both from setCameraAttribute() (when RUNNING) and from start()
     * to flush pendingAttributes_.  Implementations should set
     * @c status_.currentError to INVALID_ATTRIBUTE for unrecognised names or
     * unparseable values, and to NONE on success.
     *
     * @pre camera_src_ is non-null.
     * @param[in] name   Attribute name (case-insensitive match recommended).
     * @param[in] value  New value as a decimal string.
     */
    virtual void applyAttributeGStreamer(const std::string& name,
                                         const std::string& value) = 0;

    /**
     * @brief Parse a decimal integer from @p str without throwing.
     *
     * @param[in]  str     Input string; must be non-empty and contain only
     *                     an optional leading sign followed by decimal digits.
     * @param[out] outVal  Receives the parsed value on success.
     * @return @c true on success; @c false if the string is empty, contains
     *         non-digit characters, or has trailing garbage.
     */
    static bool safeStoi(const std::string& str, int& outVal);

    /**
     * @brief Convert a floating-point frame rate to a reduced integer fraction.
     *
     * Multiplies @p fps by 1000, rounds to the nearest integer, then reduces
     * the resulting fraction by the GCD.  This preserves common drop-frame
     * rates exactly (e.g. 29.97 → 30000/1001).
     *
     * @param[in]  fps    Frame rate in frames per second.
     * @param[out] frNum  Reduced numerator.
     * @param[out] frDen  Reduced denominator.
     */
    static void computeFpsRational(float fps, uint32_t& frNum, uint32_t& frDen);

    /**
     * @brief Construct base state; must be called by every concrete subclass constructor.
     * @param[in] camera  cameraInfo obtained from getCameraList().
     */
    explicit Camera_GST(const cameraInfo& camera);

private:
    /**
     * @brief Release all GStreamer resources acquired since the last open().
     *
     * Sets the pipeline to NULL state first (joins the streaming thread), then
     * releases tee request pads, clears tracking vectors, unrefs element handles,
     * and unrefs the pipeline.  Safe to call when any subset of those pointers
     * is null.  Called from stop(), close(), and every error path in start().
     *
     * @note Not thread-safe; must be called only from lifecycle methods.
     */
    void teardownPipeline();

    /**
     * @brief Convenience helper: tear down and record a pipeline error.
     *
     * Calls teardownPipeline(), sets status to ERROR, and records the
     * driver-specific pipelineError() code.  Used by start() error paths.
     *
     * @note Not thread-safe; must be called only from start().
     */
    void setPipelineError();

public:
    /**
     * @brief Destructor; calls close() if the camera is not already in CLOSED state.
     */
    ~Camera_GST() override;

    /** @copydoc iCamera::open() */
    void open() override;

    /** @copydoc iCamera::close() */
    void close() override;

    /** @copydoc iCamera::isOpen() */
    bool isOpen() const override;

    /** @copydoc iCamera::getCameraInfo() */
    void getCameraInfo(cameraInfo& info) const override;

    /** @copydoc iCamera::setCameraAttribute() */
    void setCameraAttribute(const std::string& name, const std::string& value) override;

    /** @copydoc iCamera::setCameraVideoFormat() */
    void setCameraVideoFormat(uint16_t formatIndex) override;

    /** @copydoc iCamera::getCameraStatus() */
    void getCameraStatus(cameraStatus& status) const override;

    /**
     * @brief Build and start the capture pipeline.
     *
     * Computes the framerate fraction, calls buildPipelineString(), launches
     * the pipeline, links all registered branches to the tee, flushes pending
     * attributes, then transitions the pipeline to PLAYING.
     *
     * @pre  Status == OPEN and cameraInfo::videoFormats is not empty.
     * @post Status transitions: OPEN → RUNNING on success, OPEN → ERROR on failure.
     */
    void start() override;

    /** @copydoc iCamera::stop() */
    void stop() override;

    /** @copydoc iCamera::captureFrame() */
    void captureFrame(uint8_t* buffer, uint32_t bufferSize, uint32_t& bytesWritten) override;

    /**
     * @brief Register a GstElement to be linked to the tee on the next start().
     *
     * start() inserts a @c queue and a @c valve between the tee and the element
     * automatically.  Ownership of @p sinkBin transfers to the pipeline via
     * gst_bin_add().  Re-register branches before each start() following a stop().
     *
     * @param[in] name     Unique branch identifier used by setBranchEnabled().
     * @param[in] sinkBin  GstElement (typically a GstBin) to attach to the tee.
     * @pre Must be called before start().
     */
    void addBranch(const std::string& name, GstElement* sinkBin);

    /**
     * @brief Return the tee element for direct pipeline manipulation.
     * @return Pointer to the @c tee element while RUNNING; nullptr otherwise.
     * @note The returned pointer is non-owning; do not unref it.
     */
    GstElement* getTee() const;

    /**
     * @brief Enable or disable a named branch at runtime without rebuilding the pipeline.
     *
     * Sets the @c drop property on the branch's internal valve element.
     *
     * @param[in] name     Branch name as supplied to addBranch().
     * @param[in] enabled  @c true to pass buffers; @c false to drop them.
     * @pre  Status == RUNNING and @p name was registered via addBranch().
     *       Sets INVALID_ATTRIBUTE if the name is not found.
     */
    void setBranchEnabled(const std::string& name, bool enabled);
};

#endif // LIBCAMERA_GST_H
