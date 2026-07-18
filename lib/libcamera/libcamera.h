/**
 * @file libcamera.h
 * @brief Camera abstraction layer: enumeration types, data structures,
 *        the pure-virtual iCamera interface, and the getCameraList() discovery function.
 *
 * All camera implementations (Camera_CSI, Camera_USB, …) derive from iCamera.
 * Discovery is hardware-agnostic: call getCameraList() to obtain a vector of
 * cameraInfo descriptors, then instantiate the appropriate concrete class.
 */

#ifndef LIBCAMERA_H
#define LIBCAMERA_H

#include "liblog.h"
#include <cstdint>
#include <string>
#include <vector>

namespace dashcam::camera {

// ─── limits ──────────────────────────────────────────────────────────────────

constexpr uint32_t MAX_ATTRIBUTES   = 64;  ///< Maximum V4L2 controls stored per camera.
constexpr uint32_t MAX_VIDEO_FORMATS = 256; ///< Maximum format entries enumerated per V4L2 query.
constexpr uint32_t MAX_MENU_OPTIONS  = 16;  ///< Declared limit for menu-type control options.
constexpr uint8_t  MAX_CAMERAS       = 16;  ///< Legacy constant; getCameraList() uses filesystem enumeration and is not bound by this value.

// ─── enumerations ────────────────────────────────────────────────────────────

/**
 * @brief Return codes shared across all camera operations.
 *
 * Positive values indicate non-fatal conditions (camera already in a given
 * state).  Zero means success.  Negative values indicate errors.
 * Check @c cameraStatus::currentError after any void-returning iCamera call.
 */
enum class ERROR_CODE {
    CAMERA_ALREADY_CLOSED  =  3, ///< close() called on an already-closed camera.
    CAMERA_ALREADY_RUNNING =  2, ///< start() called on an already-running camera.
    CAMERA_ALREADY_OPEN    =  1, ///< open() called on an already-open camera.
    NONE                   =  0, ///< No error; operation succeeded.
    INVALID_FILE_DESCRIPTOR = -1, ///< The underlying device file descriptor is invalid.
    NO_CAMERAS_FOUND       = -2, ///< getCameraList() found no capture-capable V4L2 devices.
    INVALID_ATTRIBUTE      = -3, ///< Attribute name not recognised or value could not be parsed.
    UNSUPPORTED_FORMAT     = -4, ///< Requested format index is out of range.
    CAMERA_BUSY            = -5, ///< Device is claimed by another process.
    CSI_PIPELINE_ERROR     = -6, ///< GStreamer pipeline construction or state-change failed (CSI/Argus path).
    CSI_ID_PARSE_ERROR     = -7, ///< Argus sensor-id could not be determined.
    USB_PIPELINE_ERROR     = -8, ///< GStreamer pipeline construction or state-change failed (USB/V4L2 path).
    UNKNOWN_ERROR          = -9, ///< Unclassified error.
    CAMERA_NOT_OPEN        = -10 ///< start() called while the camera is not in OPEN state.
};

/**
 * @brief Physical interface type reported for a discovered camera.
 */
enum class CAMERA_TYPE {
    CSI,     ///< MIPI CSI-2 sensor accessed via the NVIDIA Argus / tegra-video driver.
    USB,     ///< USB UVC device (driver: uvcvideo).
//    GIGE,    ///< GigE Vision camera (V4L2 attribute enumeration is skipped). GIGE detection not supported yet.
    UNKNOWN  ///< Driver name not recognised; basic capture may still work.
};

/**
 * @brief V4L2 control type as mapped to the library's type system.
 *
 * Used in cameraAttribute::type to describe how a control value should be
 * interpreted and how setCameraAttribute() expects the value string.
 */
enum class CAMERA_ATTRIBUTE_TYPE {
    INTEGER, ///< Signed 32-bit integer or bitmask (V4L2_CTRL_TYPE_INTEGER / BITMASK).
    I64,     ///< Signed 64-bit integer (V4L2_CTRL_TYPE_INTEGER64).
    U32,     ///< Unsigned 32-bit integer (V4L2_CTRL_TYPE_U32).
    U16,     ///< Unsigned 16-bit integer (V4L2_CTRL_TYPE_U16).
    U8,      ///< Unsigned 8-bit integer (V4L2_CTRL_TYPE_U8).
    FLOAT,   ///< Floating-point value (reserved; no direct V4L2 mapping).
    STRING,  ///< Null-terminated string (V4L2_CTRL_TYPE_STRING).
    BOOLEAN, ///< Boolean flag (V4L2_CTRL_TYPE_BOOLEAN); value is "0" or "1".
    MENU,    ///< Enumerated string menu or integer menu; valid options in cameraAttribute::menuOptions.
    BUTTON,  ///< Trigger-only control (V4L2_CTRL_TYPE_BUTTON); value is ignored.
    UNKNOWN  ///< Control type not handled by this library.
};

/**
 * @brief Lifecycle state of a camera instance.
 */
enum class CAMERA_STATUS {
    CLOSED,  ///< Default state; no resources held.
    OPEN,    ///< Device opened and GStreamer initialised; pipeline not yet running.
    RUNNING, ///< Capture pipeline active; captureFrame() will return data.
    ERROR    ///< A non-recoverable error has occurred; close() then re-open to recover.
};

// ─── data structures ─────────────────────────────────────────────────────────

/**
 * @brief Describes a single V4L2 control exposed by a camera.
 *
 * Populated by getCameraList() via VIDIOC_QUERYCTRL / VIDIOC_QUERYMENU.
 * The name matches the kernel-reported control name (e.g. "brightness").
 */
struct cameraAttribute {
    std::string           name;        ///< Human-readable control name from the driver.
    CAMERA_ATTRIBUTE_TYPE type;        ///< Data type of the control value.
    bool                  isWritable;  ///< True if the control can be set (not read-only).
    bool                  isReadable;  ///< True if the control can be read (not write-only).
    float                 minValue;    ///< Minimum value (cast from int32; not meaningful for STRING/BUTTON).
    float                 maxValue;    ///< Maximum value (cast from int32; not meaningful for STRING/BUTTON).
    float                 step;        ///< Step size between valid values; 0 if not applicable.
    std::vector<std::string> menuOptions; ///< Valid option strings for MENU controls; integer menus store the value as a decimal string.
};

/**
 * @brief A fully-qualified capture mode: pixel format × resolution × frame rate.
 *
 * getCameraList() builds one entry per discrete (format, width, height, fps)
 * combination reported by the driver.  Stepwise and continuous size/interval
 * ranges are not enumerated.
 */
struct cameraVideoFormat {
    uint32_t    width;        ///< Frame width in pixels.
    uint32_t    height;       ///< Frame height in pixels.
    float       frameRate;    ///< Frames per second (exact rational converted to float).
    uint32_t    pixelFormat;  ///< V4L2 four-character pixel format code (e.g. V4L2_PIX_FMT_YUYV).
    std::string description;  ///< Human-readable format name from the driver.
};

/**
 * @brief Snapshot of a camera instance's runtime state.
 *
 * Returned by getCameraStatus().  All fields reflect the state at the moment
 * of the call; there is no automatic notification of state changes.
 */
struct cameraStatus {
    CAMERA_STATUS status;             ///< Current lifecycle state.
    uint16_t      currentFormatIndex; ///< Index into cameraInfo::videoFormats for the active format.
    uint64_t      frameCount;         ///< Total frames successfully written by captureFrame() since start().
    uint64_t      freeBufferCount;    ///< Internal pipeline buffers currently available (implementation-defined).
    ERROR_CODE    currentError;       ///< Most recent error code; NONE if the last operation succeeded.
};

/**
 * @brief Static description of a camera device discovered by getCameraList().
 *
 * The contents are populated once during enumeration and do not update to
 * reflect runtime state changes.
 */
struct cameraInfo {
    CAMERA_TYPE  type;         ///< Physical interface type.
    std::string  address;      ///< Device node path (e.g. "/dev/video0").
    uint32_t     deviceId;     ///< For CSI: Argus sensor-id (0-based index among CSI cameras). For USB/UNKNOWN: the numeric suffix of the device node.
    std::vector<cameraAttribute>   attributes;   ///< V4L2 controls exposed by this device.
    std::vector<cameraVideoFormat> videoFormats; ///< All discrete capture modes supported by this device.
};

// ─── discovery ───────────────────────────────────────────────────────────────

/**
 * @brief Enumerate all V4L2 capture-capable devices present on the system.
 *
 * Scans @c /dev/videoN nodes in ascending numeric order.  For each node that
 * reports @c V4L2_CAP_VIDEO_CAPTURE, the function probes supported pixel
 * formats, discrete resolutions, discrete frame rates, and V4L2 controls.
 *
 * CSI cameras (driver: @c tegra-video or @c vi) receive their Argus sensor-id
 * in @c cameraInfo::deviceId, resolved via the device tree's
 * tegra-camera-platform module list (video-node order follows i2c probe order,
 * which can differ from Argus order with mixed sensors).  If the device tree
 * chain cannot be resolved, a sequential id in /dev/videoN order is assumed
 * and a WARN is logged.  USB cameras (driver: @c uvcvideo) receive the raw
 * numeric device index.
 *
 * @param[out] cameraList  Cleared and populated with one entry per discovered
 *                         capture device.  The vector may be empty on return
 *                         if no devices are found.
 * @param[in]  log         Optional diagnostic callback; warns when a device's
 *                         format cross-product exceeds MAX_VIDEO_FORMATS and
 *                         the list is truncated.
 * @return ERROR_CODE::NONE            if at least one camera was found.
 * @return ERROR_CODE::NO_CAMERAS_FOUND if no capture devices were discovered
 *                                      or @c /dev could not be iterated.
 */
ERROR_CODE getCameraList(std::vector<cameraInfo>& cameraList,
                         const dashcam::log::LogCallback& log = {});

// ─── interface ───────────────────────────────────────────────────────────────

/**
 * @brief Pure-virtual camera interface implemented by all concrete drivers.
 *
 * The expected lifecycle is:
 * @code
 *   camera.open();
 *   camera.setCameraVideoFormat(idx);
 *   camera.setCameraAttribute("exposure", "5000");
 *   camera.start();
 *   while (running) camera.captureFrame(buf, size, written);
 *   camera.stop();
 *   camera.close();
 * @endcode
 *
 * All mutating methods are void-returning.  Inspect @c cameraStatus::currentError
 * via getCameraStatus() after any call to detect failures.
 */
class iCamera {
public:
    virtual ~iCamera() = default;

    /**
     * @brief Acquire the device and initialise the underlying capture subsystem.
     *
     * Must be called before any other method.  Safe to call repeatedly only if
     * the camera is in CLOSED state; otherwise sets CAMERA_ALREADY_OPEN.
     *
     * @post Status transitions: CLOSED → OPEN on success, CLOSED → ERROR on failure.
     */
    virtual void open() = 0;

    /**
     * @brief Stop capture if running, release all resources, and return to CLOSED.
     *
     * Calling close() on an already-closed camera sets CAMERA_ALREADY_CLOSED
     * and returns without modifying any other state.
     *
     * @post Status transitions to CLOSED on success.
     */
    virtual void close() = 0;

    /**
     * @brief Returns true if the camera is in any state other than CLOSED.
     * @return @c true for OPEN, RUNNING, or ERROR states.
     */
    virtual bool isOpen() const = 0;

    /**
     * @brief Copy the static device descriptor into @p info.
     * @param[out] info  Receives a copy of the camera's cameraInfo struct.
     */
    virtual void getCameraInfo(cameraInfo& info) const = 0;

    /**
     * @brief Set a named camera control to the given string-encoded value.
     *
     * If the pipeline is not yet running, the attribute is queued and applied
     * automatically when start() is called.  If already running, the change is
     * applied immediately to the hardware.
     *
     * Attribute names are matched case-insensitively against a driver-specific
     * mapping table.  Unknown names set INVALID_ATTRIBUTE.
     *
     * @param[in] name   Control name (e.g. "exposure", "gain").
     * @param[in] value  New value encoded as a decimal string.
     */
    virtual void setCameraAttribute(const std::string& name, const std::string& value) = 0;

    /**
     * @brief Select the active capture format by index into cameraInfo::videoFormats.
     *
     * Must be called before start().  The selected format is used when
     * building the GStreamer pipeline.
     *
     * @param[in] formatIndex  Zero-based index into cameraInfo::videoFormats.
     *                         Sets UNSUPPORTED_FORMAT if out of range.
     */
    virtual void setCameraVideoFormat(uint16_t formatIndex) = 0;

    /**
     * @brief Copy the current runtime status into @p status.
     * @param[out] status  Receives a snapshot of the camera's cameraStatus.
     */
    virtual void getCameraStatus(cameraStatus& status) const = 0;

    /**
     * @brief Build and start the capture pipeline.
     *
     * Requires OPEN state and a valid format selection.  Pending attributes
     * are applied to the source element before the pipeline transitions to
     * PLAYING.
     *
     * @pre  Status == OPEN and cameraInfo::videoFormats is not empty.
     * @post Status transitions: OPEN → RUNNING on success, OPEN → ERROR on failure.
     *       Sets CAMERA_ALREADY_RUNNING and returns immediately if already RUNNING.
     *       Sets CAMERA_NOT_OPEN and returns immediately if the camera is in any
     *       other non-OPEN state (CLOSED, ERROR).
     */
    virtual void start() = 0;

    /**
     * @brief Tear down the capture pipeline and return to OPEN state.
     *
     * All GStreamer resources are released.  The camera can be reconfigured
     * and started again without calling close()/open().
     *
     * @pre  Status == RUNNING.
     * @post Status transitions: RUNNING → OPEN.
     */
    virtual void stop() = 0;

    /**
     * @brief Pull the latest frame into a caller-supplied buffer.
     *
     * Blocks for up to one second waiting for a frame.  If no frame arrives
     * within that window, @p bytesWritten is set to zero and the call returns.
     * If the frame data exceeds @p bufferSize the frame is discarded and
     * @p bytesWritten is set to zero.
     *
     * @param[in]  buffer       Destination buffer; must be at least @p bufferSize bytes.
     * @param[in]  bufferSize   Capacity of @p buffer in bytes.
     * @param[out] bytesWritten Number of bytes written; 0 on timeout or overflow.
     *
     * @pre  Status == RUNNING.
     * @note The default BGR appsink branch produces @c video/x-raw,format=BGR frames.
     *       Frames are written tightly packed (row stride = width × 3), so the
     *       required buffer size is exactly width × height × 3 bytes even when the
     *       source pads rows to a wider alignment.  A frame whose packed size
     *       exceeds @p bufferSize is discarded (bytesWritten = 0).
     */
    virtual void captureFrame(uint8_t* buffer, uint32_t bufferSize, uint32_t& bytesWritten) = 0;
};

// ─── attribute dictionary ─────────────────────────────────────────────────────

/**
 * @brief How a capability value string is converted before being handed to g_object_set().
 */
enum class AttributeValueType {
    String,       ///< Pass value string directly as const char*.
    Int,          ///< Parse as gint.
    Float,        ///< Parse as gfloat.
    Bool,         ///< Parse "true"/"false" or "1"/"0" as gboolean.
    BoolFromZero, ///< Parse as int; 0 → TRUE, non-zero → FALSE (AE/AWB lock inversion).
    RangeString,  ///< Mirror single value to "val val" as const char* (nvargus range props).
};

/**
 * @brief One entry in the GStreamer attribute dictionary.
 *
 * Maps a set of alias names (as written in CameraConfig::capabilities) to the
 * GStreamer element property name used by the camera source element, together
 * with the required value conversion type.
 */
struct AttributeEntry {
    std::string        gstProperty;                            ///< Property name passed to g_object_set().
    std::string        type;                                   ///< Camera type filter: "CSI", "USB", or "any".
    AttributeValueType valueType = AttributeValueType::String; ///< Value conversion type.
    std::vector<std::string> aliases;                          ///< Recognized names, matched case-insensitively.
};

/**
 * @brief Loaded attribute dictionary mapping capability names to GStreamer properties.
 *
 * Loaded once from camera_attributes.xml; passed into every Camera_GST instance via
 * setAttributeDictionary().  The dictionary is camera-type-scoped: resolve() filters
 * entries by "CSI", "USB", or "any" to prevent cross-driver mismatches.
 *
 * Typical usage:
 * @code
 *   dashcam::camera::AttributeDictionary dict;
 *   dashcam::camera::AttributeDictionary::load("config/camera_attributes.xml", dict);
 *   camera.setAttributeDictionary(dict);
 *   camera.setCameraAttribute("exposuretimerange", "13000");
 * @endcode
 */
class AttributeDictionary {
public:
    std::vector<AttributeEntry> entries;

    /**
     * @brief Find the entry matching @p alias for the given camera type.
     *
     * Alias comparison is case-insensitive.  An entry whose type is "any"
     * matches every camera type.
     *
     * @param[in] alias       Capability name (e.g. from a CameraConfig::capabilities map).
     * @param[in] cameraType  "CSI" or "USB".
     * @return Pointer to the matching entry, or nullptr if not found.
     */
    const AttributeEntry* resolve(const std::string& alias,
                                  const std::string& cameraType) const;

    /**
     * @brief Parse @p filePath (a camera_attributes.xml) into @p dict.
     *
     * @p dict is cleared before loading.  On error, @p dict is left empty.
     *
     * @param[in]  filePath  Path to the XML dictionary file.
     * @param[out] dict      Receives parsed entries.
     * @return @c true on success; @c false if the file cannot be read or has no root node.
     */
    static bool load(const std::string& filePath, AttributeDictionary& dict,
                     const dashcam::log::LogCallback& log = {});
};

} // namespace dashcam::camera

#endif // LIBCAMERA_H
