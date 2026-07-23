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
 * Existing code that reads a config field (e.g. @c cfg.encoder.bitrate) does
 * not need to change — @c ConfigVar<T> provides an implicit @c operator const T&
 * conversion.  Assignment via @c = silently clamps numeric values to [min, max].
 *
 * Typical usage:
 * @code
 *   AppConfig cfg;
 *   ConfigReader::load("config/dashcam.xml", cfg, log);
 *
 *   // Existing field access still compiles:
 *   int br = cfg.encoder.bitrate;          // implicit ConfigVar<int> → int
 *   bool on = cfg.overlay.enabled;         // implicit ConfigVar<bool> → bool
 *
 *   // Metadata available at runtime:
 *   cfg.encoder.bitrate.minValue()         // 500
 *   cfg.encoder.bitrate.description()      // "Target bitrate in kbps"
 *   cfg.encoder.bitrate.set(99999);        // returns false; value clamped to 50000
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
 * @brief H.264 software encoder parameters (x264enc; Orin Nano has no NVENC).
 * XML section: @c \<Encoder\>
 */
struct EncoderConfig {
    ConfigVar<int>         bitrate    {"Bitrate",     8000,        500,  50000, 100,   "Target bitrate in kbps (x264enc, Orin Nano has no NVENC)"};
    ConfigVar<std::string> speedPreset{"SpeedPreset", "ultrafast",                     "x264 speed preset (ultrafast/superfast/veryfast/faster/fast/medium/slow)"};
    ConfigVar<int>         keyIntMax  {"KeyIntMax",   60,          1,    300,   1,     "Maximum frames between keyframes"};
    ConfigVar<std::string> tune       {"Tune",        "",                              "x264 tune string (e.g. zerolatency); empty = omit"};
};

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
    float       minValue    = 0.0f;
    float       maxValue    = 0.0f;
    float       step        = 0.0f;
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
    ConfigVar<std::string> device      {"Device",      "",    "Device node (e.g. /dev/video2); empty = auto-detect"};
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
 * kDefaultConfigsDir (see main.cpp), so a field pointing at it would be unusable.
 */
struct SystemConfig {
    ConfigVar<std::string> footagePath  {"FootagePath",  kDefaultFootageDir, "Directory for recorded dashcam video (USB primary feed, with overlay)"};
    ConfigVar<int>         warmupFrames {"WarmupFrames", 9,           0, 120, 1, "Frames to discard after pipeline start before enabling recording"};
};

/**
 * @brief Camera GStreamer pipeline timing and queue tuning.
 *
 * libcamera cannot depend on libconfig (that would be circular — libconfig.cpp
 * includes libcamera.h), so main.cpp copies these values into a
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
 * main.cpp copies these values into a dashcam::log::LogParams and passes them
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
};

/**
 * @brief Recording-branch tuning.  Codec parameters live in EncoderConfig.
 * XML section: @c \<Recording\>
 */
struct RecordingConfig {
    ConfigVar<int> recordFps    {"RecordFps",    30, 1, 120,  1, "Recording framerate after videorate downsample (fps); capped at the camera rate"};
    ConfigVar<int> queueDepth   {"QueueDepth",   3,  1, 32,   1, "Recording-branch queue depth (buffers)"};
    ConfigVar<int> recordWidth  {"RecordWidth",  0,  0, 4096, 2, "Downscale the recording to this width before overlay/encoder (0 = source width; set BOTH dims; keep aspect)"};
    ConfigVar<int> recordHeight {"RecordHeight", 0,  0, 4096, 2, "Downscale the recording to this height before overlay/encoder (0 = source height)"};
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
    ConfigVar<float> laneReferenceY    {"LaneReferenceY",    0.90f, 0.5f, 1.0f, 0.01f, "Row (fraction of height) where lane x-positions are sampled"};
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
    // WiFi bring-up at startup: ask NetworkManager to connect to the SSID; if it
    // cannot be fulfilled the app runs in offline mode (no time-sync, no streaming).
    ConfigVar<bool>        wifiConnectEnabled  {"WifiConnectEnabled",  true, "Ask the OS (NetworkManager) to connect to the WiFi SSID at startup; on failure, run offline"};
    ConfigVar<std::string> wifiSsid            {"WifiSsid",            "",   "Target WiFi SSID; empty = reconnect to the current/last-used SSID"};
    ConfigVar<int>         wifiTimeoutSec      {"WifiTimeoutSec",      20, 5, 120, 1, "Seconds to wait for the WiFi association before declaring offline"};
    ConfigVar<bool>        wifiRequireInternet {"WifiRequireInternet", false, "Require full internet connectivity (not just WiFi association) to count as online"};

    ConfigVar<bool>        timeSyncEnabled {"TimeSyncEnabled", true, "Query an internet time server (SNTP) at startup and log the clock offset; does not change the system clock"};
    ConfigVar<std::string> ntpServer       {"NtpServer",       "pool.ntp.org", "SNTP/NTP time server hostname or IP"};
    ConfigVar<int>         ntpPort         {"NtpPort",         123,   1,   65535, 1,   "SNTP/NTP server UDP port (123 = standard NTP)"};
    ConfigVar<int>         ntpTimeoutMs    {"NtpTimeoutMs",    3000,  100, 30000, 100, "Per-attempt wait for the SNTP reply (ms)"};
    ConfigVar<int>         ntpRetries      {"NtpRetries",      2,     0,   10,    1,   "Extra SNTP attempts after the first when no reply arrives (total tries = 1 + this)"};

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
 * @brief Aggregated application configuration.
 * Root XML element: @c \<DashcamConfig\>
 */
struct AppConfig {
    EncoderConfig             encoder;
    OverlayConfig             overlay;
    std::vector<CameraConfig> cameras;
    SystemConfig              system;
    PipelineConfig            pipeline;
    RecordingConfig           recording;
    DetectionConfig           detection;
    DriverScoreConfig         driverScore;
    LogConfig                 log;
    NetworkConfig             network;
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
