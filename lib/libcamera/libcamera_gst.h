/**
 * @file libcamera_gst.h
 * @brief GStreamer-based camera base class shared by all concrete drivers.
 *
 * Camera_GST implements the complete iCamera lifecycle (open/start/stop/close,
 * captureFrame, attribute queuing, multi-sink branching) using a template-method
 * pattern.  Concrete subclasses supply three hooks:
 *
 *  - buildPipelineString() — returns the GStreamer pipeline description string.
 *  - cameraTypeTag() — returns "CSI"/"USB" to scope attribute-dictionary lookups.
 *  - pipelineError() — returns the driver-specific ERROR_CODE for pipeline failures.
 *
 * All GStreamer state (element handles, tee pads, valve map) is owned and
 * managed here; subclasses do not touch it directly.
 */

#ifndef LIBCAMERA_GST_H
#define LIBCAMERA_GST_H

#include "libcamera.h"
#include "liblog.h"
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <condition_variable>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace dashcam::camera {

/**
 * @brief Abstract GStreamer base class implementing the iCamera interface.
 *
 * Builds a multi-sink GStreamer pipeline of the form:
 * @verbatim
 *   <subclass-defined source + caps + tee name=srctee>
 *   srctee. ! queue ! <format conversion> ! BGR
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
 * **Multi-branching architecture:**
 * - Branches (recording bins, inference chains, etc.) are registered via addBranch()
 *   before start(). Each gets a queue (for backpressure isolation) and a valve
 *   (for runtime enable/disable). Recording branches use blocking queues; inference
 *   branches use leaky queues to drop old frames on backpressure.
 * - Branches are enabled/disabled at runtime via setBranchEnabled() without
 *   rebuilding the pipeline.
 * - The valve silently drops buffers if enabled=false (no timestamp discontinuity;
 *   re-enabling works seamlessly).
 *
 * **Attribute lifecycle:**
 * - Attributes set before start() are queued in pendingAttributes_ and applied
 *   to camera_src_ inside start(), before the pipeline transitions to PLAYING.
 * - Attributes set during RUNNING are applied immediately to camera_src_ hardware.
 *
 * **Concurrent safety:**
 * - Concurrent start() calls are serialized by the state machine (optimistic RUNNING claim).
 * - captureFrame() can run on another thread during start() and returns zero bytes
 *   until the pipeline is fully constructed.
 * - Attribute writes during RUNNING may temporarily block; see applyAttributeGStreamer().
 */
/**
 * @brief Tunable GStreamer pipeline timing and queue depths.
 *
 * Defaults preserve the historical hardcoded values.  The application copies
 * these from dashcam::config::PipelineConfig (libcamera cannot depend on
 * libconfig — that would be circular) and installs them via
 * Camera_GST::setPipelineParams() before start().
 */
struct PipelineParams {
    uint32_t captureTimeoutMs     = 1000;  ///< captureFrame() max wait for a frame.
    uint32_t stateChangeTimeoutMs = 5000;  ///< Async PLAYING / NULL state-change wait.
    uint32_t eosTimeoutMs         = 5000;  ///< Teardown EOS flush wait.
    uint32_t captureQueueDepth    = 2;     ///< appsink-branch leaky queue max-size-buffers.
    uint32_t appsinkMaxBuffers    = 1;     ///< appsink max-buffers.
    uint32_t branchQueueDepth     = 2;     ///< Leaky (inference) branch queue max-size-buffers;
                                           ///< blocking (recording) branches keep GStreamer defaults.
};

class Camera_GST : public iCamera {
protected:
    cameraInfo   info_;    ///< Static device descriptor supplied at construction.
    cameraStatus status_;  ///< Mutable runtime state; updated by lifecycle methods.

    /// Pipeline timing / queue tuning; read by buildPipelineString() and the
    /// lifecycle methods.  Set via setPipelineParams() before start().
    PipelineParams params_;

    /// Optional scaled-output override read by buildPipelineString().  0 = disabled
    /// (use the selected format's native dimensions / rate).  Honoured by the CSI
    /// driver, which inserts a VIC nvvidconv downscale + videorate before the tee so
    /// the *whole* camera output (appsink feed and every branch) is scaled; see
    /// setOutputResolution().  Set before start().
    uint32_t outWidth_  = 0;  ///< Scaled output width in pixels; 0 = native.
    uint32_t outHeight_ = 0;  ///< Scaled output height in pixels; 0 = native.
    float    outFps_    = 0.0f;  ///< Scaled output frame rate; 0 = native sensor rate.

    GstElement* pipeline_;   ///< Top-level GstPipeline; nullptr when not running.
    GstElement* camera_src_; ///< Source element retrieved by name "camerasrc"; used for attribute writes.
    GstElement* appsink_;    ///< Default BGR output sink; pulled by captureFrame().
    GstElement* tee_;        ///< Fan-out tee element; nullptr when not running.

    /**
     * @struct BranchEntry
     * @brief A recording or inference sink registered with the camera.
     *
     * Each branch is a GStreamer bin (containing encoding, file I/O, inference,
     * or analysis logic) that receives frames from the main tee element.
     * Branches are linked during start() with an intervening queue and valve
     * for flow control and enable/disable logic.
     */
    struct BranchEntry {
        std::string  name;           ///< User-supplied identifier for setBranchEnabled().
        GstElement*  bin;            ///< GStreamer bin providing the branch logic (owned by pipeline after start()).
        bool         leaky;          ///< true = leaky downstream queue (inference); false = blocking (recording).
        bool         initialEnabled; ///< If false, valve starts with drop=TRUE so recording is paused until setBranchEnabled(true).
    };

    /// Branches registered via addBranch() before start().  The pipeline takes
    /// ownership of each GstElement* via gst_bin_add() during start().
    std::vector<BranchEntry> branches_;

    /// Request pads obtained from tee_ for each external branch.
    /// Released via gst_element_release_request_pad() in teardownPipeline().
    std::vector<GstPad*> teePads_;

    /// Attributes queued via setCameraAttribute() before start(); flushed and
    /// applied in start() before the pipeline transitions to PLAYING, so the
    /// first captured frame already uses requested settings (e.g. exposure).
    std::map<std::string, std::string> pendingAttributes_;

    /// Non-owning pointers to the valve element in each branch, keyed by branch name.
    /// Used by setBranchEnabled() to drop/pass buffers at runtime without rebuilding.
    /// Valid only while RUNNING; owned by the pipeline.
    std::map<std::string, GstElement*> branchValves_;

    /// Protects access to status_, currentFormatIndex, frameCount, freeBufferCount,
    /// currentError, and pendingAttributes_.  Also protects pipeline_ pointer
    /// from race during concurrent start() calls (set RUNNING optimistically).
    /// Lock is acquired briefly; held longer only during gst_init_check() (now
    /// released) and during attribute application (unavoidable, locks GStreamer
    /// property writes).
    mutable std::mutex stateMutex_;

    /// True from the optimistic RUNNING claim in start() until construction
    /// finishes (success path or setPipelineError()).  Guarded by stateMutex_.
    /// stop() waits for this to clear (via startCv_) so a concurrent
    /// stop()/close() can never tear down a pipeline that start() is still
    /// assembling outside the lock (use-after-free on the half-built pipeline).
    bool starting_ = false;

    /// Signalled when starting_ clears.  The wait is bounded in practice:
    /// start() always terminates via its state-change timeouts.
    mutable std::condition_variable startCv_;

    /// Loaded attribute dictionary used by applyAttributeGStreamer() to resolve
    /// capability names to GStreamer property names.  Set via setAttributeDictionary()
    /// before open() or between stop() and start().
    AttributeDictionary dict_;

    /// Optional diagnostic callback injected by the application.  Called from
    /// lifecycle methods; never called while stateMutex_ is held.
    dashcam::log::LogCallback log_{};

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
     *    @c drop=true emit-signals=false sync=false and
     *    @c max-buffers=params_.appsinkMaxBuffers.
     *  - Size the appsink-branch @c queue with
     *    @c max-size-buffers=params_.captureQueueDepth (keep @c leaky=2).
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
     * @brief Camera-type tag ("CSI" or "USB") used to scope attribute-dictionary
     *        resolution in applyAttributeGStreamer().
     *
     * @return A stable string literal identifying the concrete driver.
     */
    virtual const char* cameraTypeTag() const = 0;

    /**
     * @brief Translate a name/value attribute pair into a GStreamer property write.
     *
     * Called both from setCameraAttribute() (when RUNNING) and from start()
     * to flush pendingAttributes_.  Resolves @p name against dict_ scoped to
     * cameraTypeTag(), then applies the property via applyGstProperty().  Sets
     * @c status_.currentError to INVALID_ATTRIBUTE for unrecognised names or
     * unparseable values, and to NONE on success.
     *
     * @pre camera_src_ is non-null (checked; sets pipelineError() otherwise).
     * @param[in] name   Attribute name (matched case-insensitively).
     * @param[in] value  New value as a decimal string.
     */
    virtual void applyAttributeGStreamer(const std::string& name,
                                         const std::string& value);

    /**
     * @brief Apply a single resolved attribute entry to a GStreamer source element.
     *
     * Converts @p value to the type encoded in @p entry and calls g_object_set().
     * Used by concrete applyAttributeGStreamer() implementations.
     *
     * @param[in] src    GStreamer element to modify (must be non-null).
     * @param[in] entry  Resolved dictionary entry (gstProperty + valueType).
     * @param[in] value  Value string from CameraConfig::capabilities.
     * @return @c true on success; @c false if @p value cannot be parsed.
     */
    static bool applyGstProperty(GstElement* src,
                                  const AttributeEntry& entry,
                                  const std::string& value);

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

    /**
     * @brief Drain any pending GST_MESSAGE_ERROR from the pipeline bus.
     *
     * There is no GLib main loop driving a bus watch, so runtime pipeline
     * failures (sensor disconnect, Argus/encoder errors) would otherwise go
     * unnoticed and captureFrame() would silently time out forever.  This polls
     * the bus non-blocking; on the first error it logs the detail and
     * transitions RUNNING → ERROR (recording pipelineError()).  Called from
     * captureFrame() and getCameraStatus(), so even a recording-only camera
     * (no captureFrame() consumer) surfaces pipeline death via status polling.
     * Acquires stateMutex_ internally; must not be called with it held.
     */
    void checkBusErrors();

public:
    /**
     * @brief Install an attribute dictionary used by setCameraAttribute() resolution.
     *
     * The dictionary is copied into the camera object.  Call this before open()
     * or between stop() and start(); the dictionary must remain consistent while
     * the camera is RUNNING.
     *
     * @param[in] dict  Loaded AttributeDictionary (from AttributeDictionary::load()).
     */
    void setAttributeDictionary(const AttributeDictionary& dict);

    /**
     * @brief Inject a log callback.  The callback is invoked on lifecycle events
     *        and pipeline errors; it is never called while stateMutex_ is held.
     *        Defaults to a no-op (silent) if not set.
     */
    void setLogCallback(dashcam::log::LogCallback cb);

    /**
     * @brief Install pipeline timing / queue tuning.
     *
     * Must be called before start() (buildPipelineString() and the state-change
     * waits read these values).  Defaults preserve the historical behaviour.
     *
     * @param p  Tuning values, typically mapped from dashcam::config::PipelineConfig.
     */
    void setPipelineParams(const PipelineParams& p);

    /**
     * @brief Request a scaled camera output instead of the format's native size/rate.
     *
     * Applies to the ENTIRE output — the appsink/inference feed and every branch off
     * the tee — because the scale happens before the tee.  On the CSI/Argus driver
     * the sensor still runs at its native mode; an nvvidconv (VIC) downscale and a
     * videorate cap are inserted, so this is cheap (VIC scaling is ~free) and keeps
     * buffers on NVMM.  Must be called before start().
     *
     * @param width   Output width in pixels; 0 keeps the format's native width.
     * @param height  Output height in pixels; 0 keeps the format's native height.
     * @param fps     Output frame rate; 0 keeps the native sensor rate. Capped by
     *                videorate, so it may only *reduce* the rate, never raise it.
     *
     * @note Honoured by Camera_CSI (Argus ISP/VIC).  Camera_USB ignores it — a
     *       v4l2 source cannot rescale; use a librecord/videoscale downscale per
     *       branch for USB instead.
     */
    void setOutputResolution(uint32_t width, uint32_t height, float fps = 0.0f);

    /**
     * @brief Convert a floating-point frame rate to a reduced integer fraction.
     *
     * Multiplies @p fps by 1000, rounds to the nearest integer, then reduces
     * the resulting fraction by the GCD.
     *
     * @param[in]  fps    Frame rate in frames per second.
     * @param[out] frNum  Reduced numerator.
     * @param[out] frDen  Reduced denominator.
     */
    static void computeFpsRational(float fps, uint32_t& frNum, uint32_t& frDen);

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
     * **Thread-safety:** Concurrent start() calls are serialized by claiming
     * RUNNING status optimistically under stateMutex_ before releasing it for
     * the long GStreamer construction phase. A second concurrent caller sees
     * CAMERA_ALREADY_RUNNING. captureFrame() can run concurrently; it checks
     * both status==RUNNING and appsink_ (null until fully built), so returns
     * zero bytes until the pipeline is ready.
     *
     * @pre  Status == OPEN and cameraInfo::videoFormats is not empty.
     * @post Status transitions: OPEN → RUNNING on success, OPEN → ERROR on failure.
     */
    void start() override;

    /**
     * @copydoc iCamera::stop()
     * @note If a concurrent start() is still constructing the pipeline, stop()
     *       blocks until that construction finishes (bounded by the start()
     *       state-change timeouts), then tears the pipeline down normally.
     */
    void stop() override;

    /** @copydoc iCamera::captureFrame() */
    void captureFrame(uint8_t* buffer, uint32_t bufferSize, uint32_t& bytesWritten) override;

    /**
     * @brief Register a GstElement to be linked to the tee on the next start().
     *
     * During start(), a @c queue and @c valve are automatically inserted between
     * the tee and @p sinkBin:
     * @verbatim
     *   srctee. ! queue [! valve] ! sinkBin
     * @endverbatim
     *
     * Ownership of @p sinkBin transfers to the pipeline via gst_bin_add().
     * Branches must be re-registered before each start() following a stop().
     *
     * @param[in] name     Unique branch identifier used by setBranchEnabled().
     * @param[in] sinkBin  GstElement (typically a GstBin) to attach to the tee.
     *                     Must be a valid GStreamer element; ownership transfers
     *                     to the pipeline.
     * @param[in] leaky    Queue behavior:
     *                     - @c true:  2-buffer leaky downstream queue. Drops old frames
     *                       on backpressure; never blocks the tee. Use for inference
     *                       branches where missing a frame is acceptable but the tee
     *                       must not block.
     *                     - @c false (default): Blocking queue. Buffers accumulate on
     *                       backpressure, blocking the tee if the queue fills. Use for
     *                       recording branches where frame loss is unacceptable.
     * @pre Must be called before start().  Calling while RUNNING sets INVALID_ATTRIBUTE
     *      and returns without modification.
     *
     * @post Branches are linked during start().  On a failed start(), all branches
     *       are torn down by teardownPipeline().  Bins registered but never
     *       started are freed by close() or the destructor.
     */
    void addBranch(const std::string& name, GstElement* sinkBin,
                   bool leaky = false, bool initialEnabled = true);

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

} // namespace dashcam::camera

#endif // LIBCAMERA_GST_H
