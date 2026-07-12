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

    std::vector<CameraAttributeInfo>   attributeInfo;  ///< Discovered control metadata (informational).
    std::map<std::string, std::string> capabilities;   ///< Attribute name → value applied at cam.start().
};

/**
 * @brief System-level runtime parameters.
 * XML section: @c \<System\>
 */
struct SystemConfig {
    ConfigVar<std::string> archivePath  {"ArchivePath",  "./archive", "Directory where recorded MKV files are written"};
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
};

/**
 * @brief Recording-branch tuning.  Codec parameters live in EncoderConfig.
 * XML section: @c \<Recording\>
 */
struct RecordingConfig {
    ConfigVar<int> recordFps  {"RecordFps",  30, 1, 120, 1, "Recording framerate after videorate downsample (fps); capped at the camera rate"};
    ConfigVar<int> queueDepth {"QueueDepth", 3,  1, 32,  1, "Recording-branch queue depth (buffers)"};
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
};

} // namespace dashcam::config

#endif // LIBCONFIG_H
