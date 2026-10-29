/**
 * @file libconfig.h
 * @brief Self-describing, strongly-typed XML configuration for the dashcam system.
 *
 * Every tunable parameter is stored as a ConfigVar<T>, which carries its value
 * alongside the runtime-accessible metadata that describes it:
 *
 *   - name         XML element tag and display identifier.
 *   - datatype     ConfigDataType enum (Int / Float / Bool / String).
 *   - value        Current runtime value, always in [min, max] for numeric types.
 *   - defaultValue Built-in default; the XML file is self-documenting because
 *                  the default is written as an attribute alongside the value.
 *   - minValue     Lower bound for numeric types (ignored for Bool / String).
 *   - maxValue     Upper bound for numeric types.
 *   - step         Suggested increment for UI sliders / human editing.
 *   - description  Human-readable one-line explanation of the parameter.
 *
 * Existing code that reads a config field (e.g. @c cfg.recording.segmentSec) does
 * not need to change — @c ConfigVar<T> provides an implicit @c operator const T&
 * conversion.  Assignment via @c = silently clamps numeric values to [min, max].
 *
 * Typical usage:
 * @code
 *   AppConfig cfg;
 *   ConfigReader::load("config/dashcam.xml", cfg, log);
 *
 *   // Existing field access still compiles:
 *   int seg = cfg.recording.segmentSec;    // implicit ConfigVar<int> → int
 *   bool on = cfg.overlay.enabled;         // implicit ConfigVar<bool> → bool
 *
 *   // Metadata available at runtime:
 *   cfg.recording.segmentSec.minValue()    // 10
 *   cfg.recording.segmentSec.description() // "Segment length (s); ..."
 *   cfg.recording.segmentSec.set(99999);   // returns false; value clamped to 3600
 * @endcode
 */

#ifndef LIBCONFIG_H
#define LIBCONFIG_H

#include "liblog.h"

#include <algorithm>
#include <map>
#include <ostream>
#include <string>
#include <type_traits>
#include <vector>

namespace dashcam::config {

// ─── ConfigDataType ───────────────────────────────────────────────────────────

enum class ConfigDataType { Int, Float, Bool, String };

// ─── ConfigVar<T> ─────────────────────────────────────────────────────────────

/**
 * @brief A strongly-typed, self-describing configuration variable.
 *
 * @tparam T  Must be one of: @c int, @c float, @c bool, @c std::string.
 *
 * The variable's @c name() doubles as the XML element tag written by
 * ConfigReader::save() and read back by ConfigReader::load().  Keep the name
 * PascalCase to match the existing XML convention.
 *
 * For numeric types (int / float), use the six-argument constructor to supply
 * min / max / step.  For Bool and String, use the three-argument constructor.
 */
template<typename T>
class ConfigVar {
    static_assert(
        std::is_same_v<T, int>  || std::is_same_v<T, float> ||
        std::is_same_v<T, bool> || std::is_same_v<T, std::string>,
        "ConfigVar<T>: T must be int, float, bool, or std::string");

public:
    // ── Constructors ──────────────────────────────────────────────────────────

    /**
     * @brief Numeric constructor (int / float).
     *
     * @param name        XML element tag name and display identifier (PascalCase).
     * @param defaultVal  Initial and default value; must be in [minVal, maxVal].
     * @param minVal      Minimum allowed value (inclusive).
     * @param maxVal      Maximum allowed value (inclusive).
     * @param step        Suggested editing increment (UI / validation hint only).
     * @param desc        One-line human-readable description.
     */
    ConfigVar(std::string name, T defaultVal,
              T minVal, T maxVal, T step,
              std::string desc)
        : name_(std::move(name))
        , datatype_(deduceType())
        , value_(defaultVal)
        , defaultValue_(defaultVal)
        , minValue_(minVal)
        , maxValue_(maxVal)
        , step_(step)
        , description_(std::move(desc))
    {}

    /**
     * @brief Non-numeric constructor (bool / string).
     *
     * @param name        XML element tag name (PascalCase).
     * @param defaultVal  Initial and default value.
     * @param desc        One-line human-readable description.
     */
    ConfigVar(std::string name, T defaultVal, std::string desc)
        : name_(std::move(name))
        , datatype_(deduceType())
        , value_(defaultVal)
        , defaultValue_(defaultVal)
        , minValue_{}
        , maxValue_{}
        , step_{}
        , description_(std::move(desc))
    {}

    // ── Metadata accessors ────────────────────────────────────────────────────

    const std::string& name()         const { return name_; }
    ConfigDataType     datatype()      const { return datatype_; }
    const T&           defaultValue()  const { return defaultValue_; }
    const T&           minValue()      const { return minValue_; }
    const T&           maxValue()      const { return maxValue_; }
    const T&           step()          const { return step_; }
    const std::string& description()   const { return description_; }

    // ── Value access ──────────────────────────────────────────────────────────

    /** @brief Implicit read conversion — existing code compiles unchanged. */
    operator const T&() const { return value_; }

    /**
     * @brief Set the value with validation.
     *
     * For numeric types the value is clamped to [minValue, maxValue].
     *
     * @return @c true if @p v was already in range (no clamping occurred);
     *         @c false if it was clamped (the stored value is the boundary).
     */
    bool set(const T& v) {
        if constexpr (std::is_arithmetic_v<T> && !std::is_same_v<T, bool>) {
            value_ = std::clamp(v, minValue_, maxValue_);
            return v == value_;
        } else {
            value_ = v;
            return true;
        }
    }

    /** @brief Assignment — delegates to set(); clamping is silent. */
    ConfigVar& operator=(const T& v) { set(v); return *this; }

    // ── Comparison / streaming ─────────────────────────────────────────────────
    // The implicit `operator const T&` alone does NOT let a ConfigVar be compared
    // or streamed like the value it wraps: the standard operators for std::string
    // are function templates, so argument deduction fails on a ConfigVar and never
    // considers the conversion (this is why `cfg.speedPreset == "x"` failed to
    // compile).  These non-template friends restore that ergonomics — for a string
    // ConfigVar the right-hand `const char*`/`std::string` literal converts into T.
    friend bool operator==(const ConfigVar& a, const T& b) { return a.value_ == b; }
    friend bool operator==(const T& a, const ConfigVar& b) { return a == b.value_; }
    friend bool operator!=(const ConfigVar& a, const T& b) { return !(a.value_ == b); }
    friend bool operator!=(const T& a, const ConfigVar& b) { return !(a == b.value_); }
    friend bool operator==(const ConfigVar& a, const ConfigVar& b) { return a.value_ == b.value_; }
    friend bool operator!=(const ConfigVar& a, const ConfigVar& b) { return !(a.value_ == b.value_); }

    friend std::ostream& operator<<(std::ostream& os, const ConfigVar& v) { return os << v.value_; }

private:
    static constexpr ConfigDataType deduceType() {
        if constexpr (std::is_same_v<T, int>)   return ConfigDataType::Int;
        else if constexpr (std::is_same_v<T, float>) return ConfigDataType::Float;
        else if constexpr (std::is_same_v<T, bool>)  return ConfigDataType::Bool;
        else                                          return ConfigDataType::String;
    }

    std::string    name_;
    ConfigDataType datatype_;
    T              value_;
    T              defaultValue_;
    T              minValue_;   ///< Unused for Bool / String.
    T              maxValue_;   ///< Unused for Bool / String.
    T              step_;       ///< Unused for Bool / String.
    std::string    description_;
};

// ─── per-domain config structs ────────────────────────────────────────────────

/**
 * @brief Telemetry sidecar (ASS) rendering parameters.
 * XML section: @c \<Overlay\>
 */
struct OverlayConfig {
    ConfigVar<bool>        enabled           {"Enabled",           true,                           "Write the ASS telemetry sidecar next to each recording"};
    ConfigVar<float>       backgroundOpacity {"BackgroundOpacity", 0.85f, 0.0f,  1.0f,  0.05f,  "Alpha of the label backing box [0=transparent, 1=opaque]"};
    ConfigVar<float>       fontSize          {"FontSize",          14.0f, 4.0f,  72.0f, 0.5f,   "Label font size at 720p; scales with the video resolution"};
    ConfigVar<std::string> fontFace          {"FontFace",          "Monospace Bold",               "Font name; a trailing ' Bold' sets the bold flag"};
    ConfigVar<float>       labelPadX         {"LabelPadX",         12.0f, 0.0f,  64.0f, 1.0f,   "Horizontal label margin from the frame edge (px at 720p)"};
    ConfigVar<float>       labelPadY         {"LabelPadY",         8.0f,  0.0f,  64.0f, 1.0f,   "Vertical label margin / box padding (px at 720p)"};
    ConfigVar<float>       subtitleRateHz    {"SubtitleRateHz",    5.0f,  0.5f,  30.0f, 0.5f,   "Telemetry samples per second written to the ASS sidecar"};
    ConfigVar<int>         staleTimeoutMs    {"StaleTimeoutMs",    2000,  0,     60000, 100,    "Telemetry age (ms) past which motion/position fields render as a dash; 0 = never stale (the clock always stays live)"};
    ConfigVar<bool>        vehicleDetail     {"VehicleDetail",     true,                           "Add a middle-left block to the ASS sidecar with every bridge telemetry field (powertrain, brake, indicators, wheels, GNSS, IMU, OBD-II, CAN map, switches); each field dashed when its source is not live"};
};

/**
 * @brief Metadata for a single V4L2 control discovered on a camera.
 *
 * Populated from cameraInfo::attributes at discovery time and written to the
 * config so the operator can see valid ranges before editing capabilities.
 * Not used at runtime — runtime uses capabilities only.
 *
 * XML element: @c \<Camera\>\<Attributes\>\<Attribute name="…" …\/\>
 */
struct CameraAttributeInfo {
    std::string name;
    bool        writable    = false;
    bool        readable    = false;
    double      minValue    = 0.0;   ///< double: V4L2 ranges are int32, which float holds exactly only to 2^24
    double      maxValue    = 0.0;
    double      step        = 0.0;
    std::string menuOptions; ///< Semicolon-separated option strings for MENU controls.
};

/**
 * @brief Configuration for a single recognized camera.
 *
 * XML element: @c \<Cameras\>\<Camera name="…" type="…"\>
 *
 * @c name and @c type are XML attributes on the @c \<Camera\> element (not
 * child elements) so they remain plain @c std::string.  Tunable scalar fields
 * use ConfigVar to carry their constraints and descriptions.
 *
 * @c capabilities feeds directly into iCamera::setCameraAttribute(); keys must
 * match the driver's attribute names (e.g. "brightness", "exposure_absolute").
 */
struct CameraConfig {
    std::string name;       ///< User label, e.g. "front" or "cabin".  XML attribute.
    std::string type;       ///< Interface type: "CSI" or "USB".  XML attribute.

    ConfigVar<bool>        enabled     {"Enabled",     true,  "Include this camera at startup"};
    ConfigVar<std::string> device      {"Device",      "",    "Device node (e.g. /dev/video2) or, better, its stable udev link (/dev/v4l/by-id/... follows the camera, /dev/v4l/by-path/... the USB port); empty = auto-detect"};
    ConfigVar<int>         sensorId    {"SensorId",    0,     0, 7,   1, "Argus sensor-id for CSI cameras; ignored for USB"};
    ConfigVar<int>         formatIndex {"FormatIndex", 0,     0, 255, 1, "Index into cameraInfo::videoFormats to activate"};

    // Whole-output downscale/rate-cap, applied via Camera_GST::setOutputResolution()
    // before start().  Affects the appsink feed AND every branch (scale sits before
    // the tee).  Honoured by the CSI/Argus driver (VIC scaling, ~free); Camera_USB
    // ignores it.  0 = keep the selected format's native value.
    ConfigVar<int>   outWidth  {"OutWidth",  0,    0,    4096,   2,    "Scaled output width in pixels (0 = native; CSI only)"};
    ConfigVar<int>   outHeight {"OutHeight", 0,    0,    4096,   2,    "Scaled output height in pixels (0 = native; CSI only)"};
    ConfigVar<float> outFps    {"OutFps",    0.0f, 0.0f, 120.0f, 1.0f, "Output framerate cap in fps (0 = native; reduce-only; CSI only)"};

    std::vector<CameraAttributeInfo>   attributeInfo;  ///< Discovered control metadata (informational).
    std::map<std::string, std::string> capabilities;   ///< Attribute name → value applied at cam.start().
};

// ─── deployment directory defaults ──────────────────────────────────────────────
// Default on-device locations for the three output streams, all under the single
// /user/output mount (see docker_dev/launchcode_dev.sh, each a bind mount to a
// subdir of the host backup drive):
//   /user/output/footage  recorded dashcam video (USB primary feed, with overlay)
//   /user/output/logs     run log files
//   /user/output/configs  the app's config (read at startup, seeded on first run)
// None of these locations is itself config-driven — the config directory obviously
// cannot be, and the log directory is resolved before the config is read.  Footage
// is overridable via the <System> config (FootagePath) since it is used after load.
inline constexpr const char* kDefaultFootageDir = "/user/output/footage";
inline constexpr const char* kDefaultLogDir     = "/user/output/logs";
inline constexpr const char* kDefaultConfigsDir = "/user/output/configs";

// Build-local fallback directory names (resolved next to the executable by
// resolveStorageDir()) used when the configured mount above is unavailable — e.g.
// the SD card is removed.  The build creates <exe_dir>/logs and seeds <exe_dir>/config
// (default dashcam.xml + camera_attributes.xml), so the configs fallback reuses that
// seeded dir ("config") and a removed SD card keeps working entirely build-local.
inline constexpr const char* kFallbackFootageName = "footage";
inline constexpr const char* kFallbackLogName     = "logs";
inline constexpr const char* kFallbackConfigsName = "config";

/**
 * @brief System-level runtime parameters.
 * XML section: @c \<System\>
 *
 * The config's own directory is not stored here — it is bootstrap-located from
 * kDefaultConfigsDir (see the dashcam_v0_* apps), so a field pointing at it would be unusable.
 */
struct SystemConfig {
    ConfigVar<std::string> footagePath  {"FootagePath",  kDefaultFootageDir, "Directory for recorded dashcam video (USB primary feed, with overlay)"};
};

/**
 * @brief Camera GStreamer pipeline timing and queue tuning.
 *
 * libcamera and libconfig are independent of each other (neither includes the
 * other), so the app (dashcam_v0_2 / v0_3) copies these values into a
 * dashcam::camera::PipelineParams and calls Camera_GST::setPipelineParams().
 * XML section: @c \<Pipeline\>
 */
struct PipelineConfig {
    ConfigVar<int> captureTimeoutMs     {"CaptureTimeoutMs",     1000, 10,  10000, 10,  "captureFrame() maximum wait for a frame (ms)"};
    ConfigVar<int> stateChangeTimeoutMs {"StateChangeTimeoutMs", 5000, 500, 30000, 100, "Async PLAYING/NULL state-change wait (ms)"};
    ConfigVar<int> eosTimeoutMs         {"EosTimeoutMs",         5000, 500, 30000, 100, "Teardown EOS flush wait (ms)"};
    ConfigVar<int> captureQueueDepth    {"CaptureQueueDepth",    2,    1,   32,    1,   "appsink-branch leaky queue depth (buffers)"};
    ConfigVar<int> appsinkMaxBuffers    {"AppsinkMaxBuffers",    1,    1,   8,     1,   "appsink max-buffers (latest-frame depth)"};
    ConfigVar<int> branchQueueDepth     {"BranchQueueDepth",     2,    1,   32,    1,   "Leaky (inference) tee-branch queue depth (buffers); recording branches use GStreamer defaults"};
};

/**
 * @brief Async logger tuning.
 *
 * liblog cannot depend on libconfig (libconfig already depends on liblog), so
 * each app copies these values into a dashcam::log::LogParams and passes them
 * to dashcam::log::init().  The DASHCAM_LOG_LEVEL environment variable
 * overrides @c level at runtime.  XML section: @c \<Log\>
 */
struct LogConfig {
    ConfigVar<int>         queueSize     {"QueueSize",     8192, 256, 65536, 256, "Async log queue depth (messages); oldest dropped under pressure"};
    ConfigVar<int>         rotateSizeKb  {"RotateSizeKb",  3072, 64,  65536, 64,  "Max log file size before rotation (KB)"};
    ConfigVar<int>         rotateFiles   {"RotateFiles",   3,    1,   16,    1,   "Rotated log files kept"};
    ConfigVar<int>         flushEverySec {"FlushEverySec", 1,    0,   60,    1,   "Periodic flush-to-disk interval (s); 0 = only FlushOn-level flushes"};
    ConfigVar<std::string> level         {"Level",         "debug",              "Minimum level written (debug|info|warn|error|off); DASHCAM_LOG_LEVEL env overrides"};
    ConfigVar<std::string> flushOn       {"FlushOn",       "warn",               "Level that forces an immediate flush to disk (debug|info|warn|error|off)"};
    ConfigVar<bool>        telemetryCsv  {"TelemetryCsv",  false,                "Log every bridge telemetry frame (~10 Hz, decoded vehicle parameters) and status frame (1 Hz) to telemetry_<start>.csv and bridge_status_<start>.csv in the log directory (~20 MB per hour of driving; not rotated)"};
};

/**
 * @brief Recording tuning (compressed passthrough: there is no encoder to tune).
 * XML section: @c \<Recording\>
 */
struct RecordingConfig {
    ConfigVar<int> recordFps    {"RecordFps",    30, 1, 120,  1, "Recording framerate after videorate downsample (fps); capped at the camera rate"};
    ConfigVar<int> queueDepth   {"QueueDepth",   3,  1, 32,   1, "Recording-branch queue depth (buffers)"};

    // ── segmented recording + loop overwrite (dashcam_v0_4) ───────────────────
    ConfigVar<int>   segmentSec          {"SegmentSec",          180,    10,   3600,    1,    "Segment length (s); a new file starts every SegmentSec with no gap between files"};
    ConfigVar<float> maxFootageGB        {"MaxFootageGB",        400.0f, 0.0f, 100000.0f, 1.0f, "Loop-overwrite quota (GB = 1e9 bytes) for this program's own segments; oldest are deleted beyond it; 0 = no quota"};
    ConfigVar<float> minFreeGB           {"MinFreeGB",           20.0f,  0.0f, 10000.0f, 1.0f, "Free-space floor (GB) on the footage filesystem; oldest segments are deleted to keep it"};
    ConfigVar<int>   stallTimeoutMs      {"StallTimeoutMs",      5000,   0,    60000,   100,  "Restart recording when no frame arrives for this long (ms); 0 = off"};
    ConfigVar<int>   firstFrameTimeoutMs {"FirstFrameTimeoutMs", 15000,  1000, 120000,  100,  "Restart recording when the first frame has not arrived this long after start (ms)"};
    ConfigVar<int>   retryIntervalSec    {"RetryIntervalSec",    3,      1,    60,      1,    "Delay between recording restart attempts (s) after the first immediate retry"};
    ConfigVar<int>   syncIntervalMs      {"SyncIntervalMs",      1000,   0,    60000,   100,  "Force the open segment and sidecar to disk (fdatasync) this often, so a power cut loses about this much footage plus the newest frame, which the muxer still holds (a whole second at RecordFps 1; one camera GOP for H.264); 0 = leave it to Linux writeback (5-30 s at risk)"};
    ConfigVar<std::string> exposureMode  {"ExposureMode",        "auto",                          "auto = 30 fps frame-rate priority, but at night (pinned at its brightest and still darker than NightLuma for 10 s) the camera's own auto-exposure takes over - far brighter, ~20 fps - until the camera runs at full frame rate again (and at least NightLuma bright); framerate = always 30 fps (near-black on unlit roads at night); camera = always the camera's auto-exposure (frame rate drops at dusk too). MJPEG cameras only"};
    ConfigVar<int>   targetLuma          {"TargetLuma",          120,    40,   220,     1,    "ExposureMode=auto|framerate: target mean brightness (0-255) of the lower two-thirds of the frame"};
    ConfigVar<int>   nightLuma           {"NightLuma",           35,     5,    120,     1,    "ExposureMode=auto: at its brightest 30 fps setting and still darker than this (0-255) for 10 s, hand exposure to the camera for night"};
    ConfigVar<std::string> focusMode     {"FocusMode",           "camera",                        "camera = leave focus to the camera (its own autofocus, if it has one; the controls are not touched); fixed = continuous autofocus off and the lens held at FocusAbsolute while recording, handed back as found when recording stops - for a windshield mount where autofocus hunts on rain, wipers, dirt on the glass or night lights. USB cameras with focus controls only"};
    ConfigVar<int>   focusAbsolute       {"FocusAbsolute",       0,      0,    65535,   1,    "FocusMode=fixed: V4L2 focus_absolute value to hold, clamped to the camera's range. The scale is the camera's own - list it with v4l2-ctl --list-ctrls and pick by recording (on many UVC cameras the low end is infinity, but check)"};
};

/**
 * @brief Inference rate caps and thresholds — the operator-tunable subset only.
 *
 * Model-locked parameters (input dimensions, class counts, ImageNet mean/std)
 * deliberately stay in each detector's own config struct so they cannot be
 * desynced from the TRT engine via the XML file.  XML section: @c \<Detection\>
 */
struct DetectionConfig {
    ConfigVar<std::string> laneEnginePath {"LaneEnginePath", "models/culane_res18_fp16.engine", "UFLD v2 TensorRT engine path — must be built with trtexec INSIDE the runtime container (engines are TRT-version-locked)"};
    ConfigVar<int>   laneTargetHz      {"LaneTargetHz",      20,    0,    60,   1,     "Lane inference rate cap (Hz); 0 = unthrottled"};
    ConfigVar<int>   laneBranchMaxFps  {"LaneBranchMaxFps",  20,    0,    120,  1,     "Lane branch inlet frame-rate cap (fps, videorate drop-only) so conversion runs below the sensor rate; 0 = uncapped"};
    ConfigVar<float> laneInputCropTop  {"LaneInputCropTop",  0.50f,   0.0f, 0.9f, 0.01f,   "Fraction of frame height cropped off the top inside the lane branch (0.5 = keep lower half)"};
    ConfigVar<float> laneInputCropBottom {"LaneInputCropBottom", 0.2059f, 0.0f, 0.9f, 0.0001f, "Fraction of frame height additionally cropped off the bottom of the lane branch (0.2059 = 224 rows on the 1088-row IMX296, leaving a 320-row mid band)"};
    ConfigVar<float> laneReferenceY    {"LaneReferenceY",    0.90f, 0.5f, 1.0f, 0.01f, "Row where lane x-positions are sampled, as a fraction of the model's implied frame (0.90 = IMX296 source row ~811 at the default crops)"};
    ConfigVar<int>   signTargetHz      {"SignTargetHz",      5,     0,    30,   1,     "Sign inference rate cap (Hz)"};
    ConfigVar<float> signConfThreshold {"SignConfThreshold", 0.50f, 0.0f, 1.0f, 0.01f, "Sign detection confidence threshold"};
    ConfigVar<float> signNmsThreshold  {"SignNmsThreshold",  0.45f, 0.0f, 1.0f, 0.01f, "Sign NMS IoU threshold"};
    ConfigVar<std::string> driverEnginePath {"DriverEnginePath", "models/drowsiness_resnet18_fp16.engine", "Drowsiness classifier TensorRT engine path — must be built with trtexec INSIDE the runtime container (engines are TRT-version-locked)"};
    ConfigVar<int>   driverTargetHz    {"DriverTargetHz",    2,     0,    30,   1,     "Driver-state inference rate cap (Hz)"};
    ConfigVar<int>   driverBranchMaxFps {"DriverBranchMaxFps", 2,   0,    120,  1,     "Driver-state branch inlet frame-rate cap (fps, videorate drop-only); UVC cameras cannot deliver 2 fps natively so the branch drops to it; 0 = uncapped"};
    ConfigVar<float> driverDrowsyThreshold {"DriverDrowsyThreshold", 0.50f, 0.0f, 1.0f, 0.01f, "P(drowsy) at or above which the driver state reports DROWSY"};
    ConfigVar<bool>  driverFaceDetection {"DriverFaceDetection", true, "YuNet DNN face detect + crop before drowsiness classification (matches the model's face-crop training data); no-face frames skip classification"};
    ConfigVar<std::string> driverFaceModelPath {"DriverFaceModelPath", "models/face_detection_yunet_2023mar.onnx", "YuNet DNN face-detection model (ONNX) for driver face detection (vendored OpenCV Zoo face_detection_yunet_2023mar); robust to tilted/off-axis faces from a low dashboard mount"};
    ConfigVar<float> driverFaceScore {"DriverFaceScore", 0.60f, 0.0f, 1.0f, 0.01f, "YuNet detection confidence threshold; lower accepts more off-axis faces (fewer no-face dropouts) at the cost of occasional false boxes"};
    ConfigVar<float> driverFaceDetectScale {"DriverFaceDetectScale", 0.5f, 0.25f, 1.0f, 0.05f, "Run YuNet on the frame downscaled by this factor (detection cost ~quadratic, so 0.5 ~= a quarter of the CPU); the classifier still crops from full resolution. Default 0.5 validated to keep 100% recall at a low dashboard mount; 1.0 = no downscale"};
};

/**
 * @brief Long-horizon driver-fatigue scoring (libdriverstate FatigueScorer).
 * XML section: @c \<DriverScore\>.  Score runs 100 (fresh) down past 0
 * (fatigued); drowsiness deducts per completed chunk, wakefulness heals more
 * slowly, drowsiness-correlated lane drift deducts extra, and the score cap
 * decays with time-on-task.  The acute micro-sleep alert is separate and
 * NEVER replaced by the score.
 */
struct DriverScoreConfig {
    ConfigVar<float> scoreInitial {"ScoreInitial", 100.0f, 0.0f, 1000.0f, 1.0f, "Starting / reset fatigue score"};
    ConfigVar<float> scoreUpper   {"ScoreUpper",   100.0f, 0.0f, 1000.0f, 1.0f, "Score cap before time-on-task decay"};
    ConfigVar<float> scoreLower   {"ScoreLower",   -10.0f, -1000.0f, 0.0f, 1.0f, "Hard score floor"};
    ConfigVar<float> drowsyChunkSec     {"DrowsyChunkSec",     10.0f, 1.0f, 600.0f, 1.0f, "Seconds of CONTINUOUS drowsiness per deduction; recovering mid-chunk costs nothing"};
    ConfigVar<float> drowsyChunkPenalty {"DrowsyChunkPenalty", 10.0f, 0.0f, 100.0f, 1.0f, "Points deducted per completed drowsy chunk"};
    ConfigVar<float> awakeChunkSec      {"AwakeChunkSec",      10.0f, 1.0f, 600.0f, 1.0f, "Seconds of continuous wakefulness per reward"};
    ConfigVar<float> awakeChunkReward   {"AwakeChunkReward",    5.0f, 0.0f, 100.0f, 1.0f, "Points restored per completed awake chunk (asymmetric on purpose: fatigue builds faster than it heals)"};
    ConfigVar<float> laneDepartThresh {"LaneDepartThresh", 0.80f, 0.10f, 2.0f, 0.05f, "|lateral offset| counting as drifting onto/across a lane line (0 centred, 1 on the line)"};
    ConfigVar<float> laneReturnSec    {"LaneReturnSec",    10.0f, 1.0f, 120.0f, 1.0f, "A departure during a drowsy episode that returns within this window deducts LaneDriftPenalty (longer = deliberate lane change, no deduction)"};
    ConfigVar<float> laneDriftPenalty {"LaneDriftPenalty", 10.0f, 0.0f, 100.0f, 1.0f, "Points deducted per drowsiness-correlated lane drift (once per drowsy episode)"};
    ConfigVar<float> capDecayPerHour {"CapDecayPerHour", 10.0f, 0.0f, 100.0f, 1.0f, "Score-cap reduction per full driving hour (time-on-task fatigue)"};
    ConfigVar<float> capDecayFloor   {"CapDecayFloor",   50.0f, 0.0f, 1000.0f, 1.0f, "Cap never decays below this"};
    ConfigVar<float> cautionScore {"CautionScore", 60.0f, -1000.0f, 1000.0f, 1.0f, "Below this: CAUTION (subtle cue)"};
    ConfigVar<float> warningScore {"WarningScore", 30.0f, -1000.0f, 1000.0f, 1.0f, "Below this: WARNING (repeated alert)"};
    ConfigVar<float> fatigueScore {"FatigueScore",  0.0f, -1000.0f, 1000.0f, 1.0f, "At/below this: fatigue zone"};
    ConfigVar<float> fatigueSustainSec {"FatigueSustainSec", 300.0f, 0.0f, 3600.0f, 10.0f, "Continuous seconds in the fatigue zone before the high-confidence FATIGUE alarm"};
    ConfigVar<float> acuteAlertSec {"AcuteAlertSec", 2.0f, 0.5f, 60.0f, 0.5f, "Sustained seconds of instantaneous DROWSY before the immediate micro-sleep alert (independent of the score)"};
    ConfigVar<bool>  noFaceFreezes {"NoFaceFreezes", true, "No-face frames freeze the score (a blocked/averted camera is not drowsiness); false accrues awake instead"};
};

/**
 * @brief Internet connectivity (libnetwork) parameters.
 * XML section: @c \<Network\>
 *
 * The SNTP fields feed dashcam::network::queryTime() for startup clock-health
 * telemetry only.  Plain SNTP is not authenticated, so the app never applies a
 * reply to the privileged system clock; clock discipline belongs to the host
 * time service.  libnetwork itself does not depend on libconfig.
 */
struct NetworkConfig {
    ConfigVar<bool> monitoringEnabled {"MonitoringEnabled", false, "Publish read-only v0.4 snapshots to the local Android connector agent (no LAN listener in the recorder)"};
    ConfigVar<std::string> monitoringSocket {"MonitoringSocket", "/run/dashcam-monitor/telemetry.sock", "Unix datagram socket owned by the host monitoring agent; share its parent into the recording container"};
    ConfigVar<bool> monitoringVideoEnabled {"MonitoringVideoEnabled", false, "Also publish a 640x360 preview of the recording camera to the monitoring agent while a viewer watches (needs MonitoringEnabled)"};
    ConfigVar<std::string> monitoringVideoSocket {"MonitoringVideoSocket", "/run/dashcam-monitor/video.sock", "Unix datagram socket the monitoring agent receives preview frames on"};
    ConfigVar<std::string> monitoringVideoDemand {"MonitoringVideoDemand", "/run/dashcam-monitor/video-demand", "File the monitoring agent touches while a viewer watches; preview frames are converted only while it is fresh (< 3 s old)"};
    ConfigVar<int> monitoringVideoFps {"MonitoringVideoFps", 10, 1, 15, 1, "Monitoring preview frame-rate ceiling (fps); out of range = 10"};
    // WiFi bring-up at startup: ask NetworkManager to connect to the SSID; if it
    // cannot be fulfilled the app runs in offline mode (no time-sync, no streaming).
    ConfigVar<bool>        wifiConnectEnabled  {"WifiConnectEnabled",  true, "Ask the OS (NetworkManager) to connect to the WiFi SSID at startup; on failure, run offline"};
    ConfigVar<std::string> wifiSsid            {"WifiSsid",            "",   "Target WiFi SSID; empty = reconnect to the current/last-used SSID"};
    ConfigVar<int>         wifiTimeoutSec      {"WifiTimeoutSec",      20, 5, 120, 1, "Seconds to wait for the WiFi association before declaring offline"};
    ConfigVar<bool>        wifiRequireInternet {"WifiRequireInternet", false, "Require full internet connectivity (not just WiFi association) to count as online"};

    ConfigVar<bool>        timeSyncEnabled {"TimeSyncEnabled", true, "Query an internet time server (SNTP): v0.3 logs the clock offset at startup; v0.4 also corrects the clock (see ClockSetEnabled) and keeps retrying while offline"};
    ConfigVar<std::string> ntpServer       {"NtpServer",       "pool.ntp.org", "SNTP/NTP time server hostname or IP"};
    ConfigVar<int>         ntpPort         {"NtpPort",         123,   1,   65535, 1,   "SNTP/NTP server UDP port (123 = standard NTP)"};
    ConfigVar<int>         ntpTimeoutMs    {"NtpTimeoutMs",    3000,  100, 30000, 100, "Per-attempt wait for the SNTP reply (ms)"};
    ConfigVar<int>         ntpRetries      {"NtpRetries",      2,     0,   10,    1,   "Extra SNTP attempts after the first when no reply arrives (total tries = 1 + this)"};
    ConfigVar<bool>        clockSetEnabled {"ClockSetEnabled", true, "dashcam_v0_4: set the system clock when NTP (or GPS) says it is more than 2 s off — the Jetson has no RTC battery, so it boots with the last shutdown time. v0.3 never sets the clock"};
    ConfigVar<bool>        gpsTimeFallback {"GpsTimeFallback", true, "dashcam_v0_4: while NTP has not succeeded within the last hour, set the clock from GPS UTC relayed by the ESP32-C3 bridge"};

    // Live video streaming of the recording camera's precompressed feed (the
    // direct libnetwork MediaStreamServer path).  The wire format is chosen
    // automatically from the camera's pixel format: MJPEG → browser-viewable
    // HTTP multipart at http://<ip>:<port>/ ; H.264 → raw TCP (ffplay tcp://).
    ConfigVar<bool>        streamEnabled     {"StreamEnabled",     false, "Serve the recording camera's compressed feed over the network for live viewing (MJPEG: open http://<device-ip>:<StreamPort>/ in a browser)"};
    ConfigVar<int>         streamPort        {"StreamPort",        8090,  1, 65535, 1, "TCP port for the live video stream"};
    ConfigVar<int>         streamMaxClients  {"StreamMaxClients",  4,     1, 32,    1, "Maximum simultaneous stream viewers"};
    ConfigVar<std::string> streamBindAddress {"StreamBindAddress", "",    "Bind the live video stream to this local IPv4 (e.g. 127.0.0.1 = localhost only); empty = all interfaces"};

    // H.264-over-RTP streaming of the INFERENCE camera feeds (raw → x264enc →
    // rtph264pay → udpsink; software-encoded, extra CPU).  The lane/road camera
    // streams to RtpPort and the driver camera to RtpPort+2 (see viewer hint in
    // the log).  Receive with, e.g., gst-launch udpsrc / ffplay on an SDP.
    ConfigVar<bool>        rtpEnabled     {"RtpEnabled",     false,        "Stream the inference camera feeds as H.264 over RTP/UDP (software-encoded — extra CPU)"};
    ConfigVar<std::string> rtpHost        {"RtpHost",        "127.0.0.1",  "RTP/UDP destination: a viewer's unicast IP or a multicast group"};
    ConfigVar<int>         rtpPort        {"RtpPort",        5600,  1, 65531, 1, "Base RTP/UDP destination port (lane cam = RtpPort, driver cam = RtpPort+2)"};
    ConfigVar<int>         rtpBitrateKbps {"RtpBitrateKbps", 4000,  200, 50000, 100, "x264 target bitrate for RTP streaming (kbps)"};

    // Remote control + telemetry channel (libnetwork ControlServer): one TCP
    // connection lets a remote operator re-point the RTP streams at runtime
    // (e.g. "RTP lane here 5600" sends the lane cam to the operator's own IP) and
    // continuously receives ADAS telemetry (lane offset, fatigue score) as JSON
    // lines.  This channel can redirect the video feeds, so it is access-controlled:
    // bind it narrowly (ControlBindAddress), scope it to known IPs (ControlAllowlist),
    // and require a pre-shared key (ControlAuthToken).  Connect with the bundled
    // src/tools/dashcam_ctl.py client (it computes the HMAC challenge response).
    ConfigVar<bool>        controlEnabled     {"ControlEnabled",     false, "Open a TCP control+telemetry channel: remote operators can re-point the RTP streams and stream live ADAS telemetry"};
    ConfigVar<int>         controlPort        {"ControlPort",        8091,  1, 65535, 1, "TCP port for the remote control + telemetry channel"};
    ConfigVar<int>         controlMaxClients  {"ControlMaxClients",  2,     1, 16,    1, "Maximum simultaneous control/telemetry clients"};
    ConfigVar<std::string> controlBindAddress {"ControlBindAddress", "",    "Bind the control channel to this local IPv4 (e.g. 127.0.0.1 = localhost only, or a LAN IP); empty = all interfaces"};
    ConfigVar<std::string> controlAllowlist   {"ControlAllowlist",   "",    "Comma-separated IPv4 allowlist for control clients; empty = accept any source IP"};
    ConfigVar<std::string> controlAuthToken   {"ControlAuthToken",   "",    "Pre-shared key for nonce+HMAC-SHA256 control-channel auth; empty = authentication DISABLED (unauthenticated!)"};
};

/**
 * @brief Live preview of the recording camera on the ES3C28P display over USB.
 * XML section: @c \<DisplayPreview\>
 *
 * The recorder's compressed frames are sampled, DCT-scaled, letterboxed to the
 * 320x240 panel (16:9 -> 320x180) and re-encoded as baseline JPEG on the Jetson
 * (libvideopreview), then sent as credited E3 29 video records on the display's
 * existing USB CDC control link (libes3display).  See
 * peripherals/es3c28p/VIDEO_STREAMING_PLAN.md; its defaults are provisional until
 * the hardware feasibility bench passes.
 *
 * Opt-in: a config file without this section (every file on the car predates it)
 * keeps the preview disabled.  No preview setting or failure may affect recording.
 * Per-field ranges are clamped by ConfigReader::load(); the cross-field rules are
 * checked by validateDisplayPreview().  libconfig does not depend on the preview
 * libraries: the app copies these values into their own config structs.
 *
 * The timeout ranges stay within the display's START_VIDEO ceiling of 10000 ms
 * (es3video::kMaxTimeoutMs in peripherals/es3c28p/lib/es3UsbLink/Es3VideoProtocol.h;
 * config_test checks this and the other protocol caps), with StaleAfterMs capped one
 * step lower so PlaceholderAfterMs > StaleAfterMs can always be met.
 */
struct DisplayPreviewConfig {
    ConfigVar<bool>        enabled            {"Enabled",            false,                    "Stream a letterboxed live preview of the recording camera to the ES3C28P display over USB. Turn on only after the board runs video-capable firmware and its udev alias (see Device) exists; recording never depends on it"};
    ConfigVar<std::string> device             {"Device",             "/dev/dashcam-display",   "The display's tty: its udev alias (peripherals/es3c28p/host/99-dashcam-display.rules) or a /dev/serial/by-id/... path. Only this exact path is opened and no port is probed - the ESP32-C3 bridge has the same USB VID:PID, so never use a /dev/ttyACMn number"};
    ConfigVar<int>         targetFps          {"TargetFps",          10,      1,      15,      1,      "Preview frame-rate ceiling (fps); never more than the camera delivers. 10 until the hardware soak shows 15 holds"};
    ConfigVar<std::string> fit                {"Fit",                "letterbox",              "How the camera image fits the 320x240 panel. letterbox = the whole image at its own aspect ratio with black bars (16:9 -> 320x180), never stretched or cropped. The only supported value: anything else is rejected at startup (WARN) and the preview stays off"};
    ConfigVar<int>         jpegQuality        {"JpegQuality",        65,      20,     95,      1,      "Preview JPEG encoder quality (libjpeg 1-100 scale; baseline 4:2:0). A size/quality trade only: the measured encode cost barely changes with it"};
    ConfigVar<int>         minimumJpegQuality {"MinimumJpegQuality", 45,      10,     95,      1,      "Lowest quality the single re-encode of an oversized frame (over MaxJpegBytes) may use; must not exceed JpegQuality. A frame still oversized after that one retry is dropped"};
    ConfigVar<int>         maxJpegBytes       {"MaxJpegBytes",       65536,   4096,   65536,   1024,   "Hard cap on one encoded preview frame (bytes); the smaller of this and the display's advertised limit applies. 65536 = the display firmware's JPEG slot size"};
    ConfigVar<int>         maxSourceJpegBytes {"MaxSourceJpegBytes", 1048576, 262144, 8388608, 65536,  "Size of each preallocated input slot for a compressed camera frame (bytes; a 1080p MJPEG frame is ~160 KiB). A larger camera frame is dropped with a 'source frame too large' status; recording is unaffected"};
    ConfigVar<int>         chunkDataBytes     {"ChunkDataBytes",     1024,    256,    4096,    256,    "JPEG bytes per USB video CHUNK record (each record adds 20 bytes: 16 of header and CRC, 4 of offset); the display's advertised chunk limit can lower it"};
    ConfigVar<int>         maxFramesInFlight  {"MaxFramesInFlight",  1,       1,      2,       1,      "Credit window cap: frames sent before the display frees a receive buffer. Start with 1; raise to 2 only if measured stages need overlapping. The display's advertised credits can lower it"};
    ConfigVar<int>         assemblyTimeoutMs  {"AssemblyTimeoutMs",  500,     100,    5000,    50,     "Deadline (ms) from a frame's START record to its END; an incomplete frame is aborted and dropped, never retransmitted"};
    ConfigVar<int>         staleAfterMs       {"StaleAfterMs",       500,     100,    9900,    50,     "Time without a newly presented frame (ms) after which the image is visibly marked stale; must be less than PlaceholderAfterMs. Capped at 9900 so a later placeholder still fits the display's 10000 ms protocol limit"};
    ConfigVar<int>         placeholderAfterMs {"PlaceholderAfterMs", 2000,    200,    10000,   100,    "Time without a newly presented frame (ms) after which the stale image is replaced by the 'Video unavailable' placeholder; must exceed StaleAfterMs. Max 10000 = the display protocol's timeout limit"};
    ConfigVar<int>         heartbeatMs        {"HeartbeatMs",        1000,    200,    4000,    100,    "Heartbeat interval to the display (ms), sent even while video is paused; the display ends the session after 5 s without one, hence the 4000 ms cap"};
    ConfigVar<int>         reconnectMs        {"ReconnectMs",        1000,    200,    60000,   100,    "Interval (ms) between attempts to reopen Device after USB loss or a failed handshake; recording continues meanwhile"};
};

/**
 * @brief Check the DisplayPreviewConfig rules that per-field clamping cannot express.
 *
 * Pure: reads @p cfg only, logs nothing, changes nothing.  The rules:
 *   - @c Fit is @c "letterbox" (the only supported mode; compared exactly);
 *   - @c MinimumJpegQuality <= @c JpegQuality (the oversized-frame retry may only
 *     lower quality);
 *   - @c PlaceholderAfterMs > @c StaleAfterMs (the stale mark comes first);
 *   - @c Device is non-empty when @c Enabled (an empty path must not fall back to
 *     probing ports).
 *
 * Intended use, once at startup after ConfigReader::load(): when
 * @c cfg.displayPreview.enabled is set, call this; on @c false log every entry of
 * @p problems as a WARN and leave the preview off for this run.  Recording starts
 * regardless.  Values are reported, never silently repaired, so a typo cannot turn
 * into a different preview behaviour than the one configured.
 *
 * The per-field ranges already keep every timeout within the display's 10000 ms
 * START_VIDEO ceiling, so the timeouts of a configuration that passes here are also
 * acceptable to the display (which requires placeholder >= stale).
 *
 * @param cfg       Preview settings as loaded (each field already within its range).
 * @param problems  Cleared, then given one human-readable line per violated rule.
 * @return @c true if every rule holds (@p problems is then empty).
 */
bool validateDisplayPreview(const DisplayPreviewConfig& cfg,
                            std::vector<std::string>& problems);

/**
 * @brief Position map on the ES3C28P's MAP tab (lib/libnavigation).
 * XML section: @c \<Navigation\>
 *
 * See peripherals/es3c28p/NAVIGATION_PLAN.md.  Online raster tiles (OpenStreetMap
 * by default) are fetched only while the map is on screen, cached on disk, and
 * composed on the Jetson; GPS comes from the existing ESP32-C3 bridge telemetry.
 * Opt-in (absent section = off).  The display link itself (device path, heartbeat,
 * reconnect, chunk size, frames in flight) is configured in \<DisplayPreview\>,
 * whose settings apply to the map too even while the camera preview is disabled.
 * Navigation failures never affect recording or startup.
 */
struct NavigationConfig {
    ConfigVar<bool>        enabled           {"Enabled",           false, "Show the position map on the ES3C28P's MAP tab. Needs map-capable display firmware; the link settings come from <DisplayPreview> (its Device etc.) even when the camera preview there is disabled"};
    ConfigVar<std::string> providerUrl       {"ProviderUrl",       "https://tile.openstreetmap.org/{z}/{x}/{y}.png", "XYZ raster tile URL template with {z} {x} {y}; 256x256 PNG tiles. https only (http is accepted for 127.0.0.1/localhost test servers). The OSM default is subject to the OSMF tile usage policy: visible map only, no prefetch or offline download"};
    ConfigVar<std::string> providerId        {"ProviderId",        "osm-standard", "Cache namespace for ProviderUrl: change it when the provider or its style changes so old tiles are not shown as the new style"};
    ConfigVar<std::string> userAgent         {"UserAgent",         "DashcamES3Map/1.0 (+https://github.com/chad-Ant/dashcam)", "HTTP User-Agent identifying this application to the tile server (required by the OSM policy); never a generic library or browser string"};
    ConfigVar<std::string> attribution       {"Attribution",       "\xC2\xA9 OpenStreetMap contributors", "Credit drawn in the map's bottom strip whenever map imagery is visible (required by the tile/data licence)"};
    ConfigVar<std::string> cachePath         {"CachePath",         "/user/output/navcache/tiles.sqlite", "SQLite tile cache file on persistent storage (the directory is created)"};
    ConfigVar<int>         cacheBudgetMB     {"CacheBudgetMB",     256,  16,  4096, 16, "Disk budget for cached tiles (MiB); least-recently-used tiles outside the current view are evicted beyond it"};
    ConfigVar<int>         zoom              {"Zoom",              16,   12,  18,   1,  "Map zoom level (16: about 2.4 m per pixel at the equator)"};
    ConfigVar<int>         maxFps            {"MaxFps",            4,    1,   10,   1,  "Map frame-rate ceiling while the MAP tab is shown; also the START_MAP target fps"};
    ConfigVar<int>         warmFps           {"WarmFps",           1,    0,   4,    1,  "Re-render rate while the MAP tab is hidden (from the disk cache only, nothing sent), so switching to the map shows a fresh frame at once; 0 = no warm rendering"};
    ConfigVar<int>         jpegQuality       {"JpegQuality",       80,   40,  95,   1,  "Map JPEG quality (4:4:4 for legible text), whole frames and the panning strips; a frame over the display's JPEG limit is retried 10 lower, twice"};
    ConfigVar<int>         gpsStaleMs        {"GpsStaleMs",        1500, 500, 10000, 100, "A bridge telemetry sample older than this (ms) shows the position as 'Last known' (grey marker)"};
    ConfigVar<int>         staleAfterMs      {"StaleAfterMs",      2000, 1500, 9900, 100, "Display side: time without a new map frame or position (ms) before the board marks the map stale; at least 1500 (a shown whole-frame map refreshes every second)"};
    ConfigVar<int>         placeholderAfterMs{"PlaceholderAfterMs",5000, 200, 10000, 100, "Display side: time without a new map frame (ms) before the board shows 'Map unavailable'; must exceed StaleAfterMs"};
    ConfigVar<bool>        panning           {"Panning",           true, "Board-side panning (map extension v2) when the display firmware has it: the board scrolls a larger map itself and draws the marker from 20 Hz positions interpolated between GPS fixes (smooth movement, a few KB/s over USB). Firmware without it gets whole map frames at MaxFps as before"};
    ConfigVar<int>         poseHz            {"PoseHz",            20,   5,   30,   1,  "Panning: position updates sent to the board per second (the board's own maximum caps it)"};
    ConfigVar<int>         panBackgroundWidth{"PanBackgroundWidth",512,  384, 640,  32, "Panning: width of the map image the board scrolls (px); larger = rarer, bigger images. At least the map area + 64 so there is room to scroll; clamped to the board's maximum (whole frames if that leaves under 32 px a side)"};
    ConfigVar<int>         panBackgroundHeight{"PanBackgroundHeight",384, 256, 480, 32, "Panning: height of the map image the board scrolls (px); at least the map area + 64. Clamped to the board's maximum"};
    ConfigVar<int>         panJpegQuality    {"PanJpegQuality",    75,   30,  95,   1,  "Panning: quality of the scrolled map image (4:2:0 to fit the display's 64 KB JPEG limit); retried 10 lower up to three times if too large"};
};

/**
 * @brief Check the NavigationConfig rules clamping cannot express: ProviderUrl has
 *        {z} {x} {y} and no other placeholder, and is https (plain http only to
 *        exactly localhost / 127.0.0.1 / [::1]); UserAgent, Attribution,
 *        ProviderId and CachePath are non-empty; Attribution fits the strip
 *        (52 characters); StaleAfterMs >= 1500 and PlaceholderAfterMs >
 *        StaleAfterMs.  Messages never repeat a URL's query (keys).  Pure; same
 *        use as validateDisplayPreview().
 */
bool validateNavigation(const NavigationConfig& cfg, std::vector<std::string>& problems);

/**
 * @brief The part of \<DisplayPreview\> the map needs even while the camera
 *        preview is disabled (it owns the display link): Device non-empty and
 *        PlaceholderAfterMs > StaleAfterMs.  Pure.
 */
bool validateDisplayLink(const DisplayPreviewConfig& cfg, std::vector<std::string>& problems);

/**
 * @brief Aggregated application configuration.
 * Root XML element: @c \<DashcamConfig\>
 */
struct AppConfig {
    OverlayConfig             overlay;
    std::vector<CameraConfig> cameras;
    SystemConfig              system;
    PipelineConfig            pipeline;
    RecordingConfig           recording;
    DetectionConfig           detection;
    DriverScoreConfig         driverScore;
    LogConfig                 log;
    NetworkConfig             network;
    DisplayPreviewConfig      displayPreview;
    NavigationConfig          navigation;
};

// ─── reader / writer ─────────────────────────────────────────────────────────

/**
 * @brief Loads and saves the dashcam XML configuration.
 *
 * On load, every ConfigVar field is read from the element whose tag matches
 * @c ConfigVar::name().  Numeric values outside [min, max] are clamped and a
 * @c WARN is logged.  Fields absent from the file keep their built-in defaults.
 *
 * On save, every element is written with @c type, @c default, @c description,
 * and (for numeric types) @c min, @c max, @c step as XML attributes so the
 * resulting file is self-documenting.
 */
class ConfigReader {
public:
    static bool load(const std::string& filePath, AppConfig& config,
                     const dashcam::log::LogCallback& log = {});

    static bool save(const std::string& filePath, const AppConfig& config,
                     const dashcam::log::LogCallback& log = {});

    /**
     * @brief Load @p filePath, or create it (and its parent directory) with a
     *        default-valued config if it does not exist.
     *
     * On a missing file the parent directory is created, a default-constructed
     * AppConfig is written to @p filePath, and @p config is set to those defaults.
     * On an existing file this behaves like load().  Intended for the build-local
     * config (configDir() + "/dashcam.xml") so first runs self-seed.
     *
     * @return @c true if the config was loaded or freshly created; @c false only
     *         if the file was missing and could not be created (e.g. unwritable
     *         directory) or an existing file failed to parse.
     */
    static bool loadOrCreate(const std::string& filePath, AppConfig& config,
                             const dashcam::log::LogCallback& log = {});
};

/**
 * @brief Directory for config files next to the running executable: <exe_dir>/config.
 *
 * For a build tree binary this is that build's own config dir (e.g.
 * bin/build_<ts>/config), which the build seeds with a default dashcam.xml and
 * which ConfigReader::loadOrCreate() recreates on demand.  Resolution uses the
 * executable path, so it is independent of the current working directory; falls
 * back to a cwd-relative "config" if the executable path cannot be read.
 */
std::string configDir();

/**
 * @brief Directory named @p name next to the running executable: <exe_dir>/<name>.
 *
 * Resolution uses the executable path (cwd-independent); falls back to a
 * cwd-relative "<name>" if the executable path cannot be read.  Used as the
 * build-local storage fallback (e.g. "footage", "logs", "configs") when the
 * configured mount is unavailable.
 */
std::string exeRelativeDir(const std::string& name);

/**
 * @brief Resolve a storage directory, falling back to build-local on failure.
 *
 * Returns @p preferred (e.g. the configured /user/output/{footage,logs,configs}
 * mount) if it can be created and written to; otherwise returns
 * exeRelativeDir(@p fallbackName) — a directory inside the build tree — after
 * creating it.  This lets a removed SD card / unavailable mount transparently
 * degrade to build-local storage instead of losing recordings or logs.  A WARN is
 * logged (via @p log) when the fallback is used.
 *
 * The writability test creates the directory if missing and writes+removes a probe
 * file, so it detects a stale/read-only mount that a plain existence check would not.
 *
 * @param preferred     Configured target directory (footagePath / kDefaultConfigsDir / kDefaultLogDir).
 * @param fallbackName  Build-local directory name, e.g. "footage", "logs", "configs".
 * @param log           Optional callback; a WARN is emitted when falling back.
 * @return The directory that should actually be used for writing.
 */
std::string resolveStorageDir(const std::string& preferred,
                              const std::string& fallbackName,
                              const dashcam::log::LogCallback& log = {});

} // namespace dashcam::config

#endif // LIBCONFIG_H
