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
 * @brief Cairo overlay rendering parameters.
 * XML section: @c \<Overlay\>
 */
struct OverlayConfig {
    ConfigVar<bool>        enabled           {"Enabled",           true,                           "Render telemetry overlay on recorded video"};
    ConfigVar<float>       backgroundOpacity {"BackgroundOpacity", 0.85f, 0.0f,  1.0f,  0.05f,  "Alpha of the semi-transparent label backing [0=transparent, 1=opaque]"};
    ConfigVar<float>       fontSize          {"FontSize",          14.0f, 4.0f,  72.0f, 0.5f,   "Label font size in points"};
    ConfigVar<std::string> fontFace          {"FontFace",          "Monospace Bold",               "Pango font description string"};
    ConfigVar<float>       labelPadX         {"LabelPadX",         12.0f, 0.0f,  64.0f, 1.0f,   "Horizontal padding inside overlay label boxes (px)"};
    ConfigVar<float>       labelPadY         {"LabelPadY",         8.0f,  0.0f,  64.0f, 1.0f,   "Vertical padding inside overlay label boxes (px)"};
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
    ConfigVar<int>   laneTargetHz      {"LaneTargetHz",      20,    0,    60,   1,     "Lane inference rate cap (Hz); 0 = unthrottled"};
    ConfigVar<float> laneReferenceY    {"LaneReferenceY",    0.90f, 0.5f, 1.0f, 0.01f, "Row (fraction of height) where lane x-positions are sampled"};
    ConfigVar<int>   signTargetHz      {"SignTargetHz",      5,     0,    30,   1,     "Sign inference rate cap (Hz)"};
    ConfigVar<float> signConfThreshold {"SignConfThreshold", 0.50f, 0.0f, 1.0f, 0.01f, "Sign detection confidence threshold"};
    ConfigVar<float> signNmsThreshold  {"SignNmsThreshold",  0.45f, 0.0f, 1.0f, 0.01f, "Sign NMS IoU threshold"};
    ConfigVar<int>   driverTargetHz    {"DriverTargetHz",    1,     0,    30,   1,     "Driver-state inference rate cap (Hz)"};
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
    LogConfig                 log;
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
