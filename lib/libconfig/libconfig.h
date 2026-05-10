/**
 * @file libconfig.h
 * @brief XML-based application configuration loader for the dashcam system.
 *
 * Parses a single <DashcamConfig> XML document (via pugixml, which is kept as
 * an implementation detail) and populates a hierarchy of plain-data structs.
 * Every field has a built-in default so the application runs unchanged when a
 * field is absent from the file; only a missing root node is a hard failure.
 *
 * Typical usage:
 * @code
 *   AppConfig cfg;
 *   ConfigReader::load("config/dashcam.xml", cfg);   // falls back to defaults on error
 *
 *   // Wire encoder settings into the recording bin:
 *   //   cfg.encoder.bitrate, cfg.encoder.speedPreset, …
 *
 *   // Apply per-camera capabilities from config:
 *   for (const auto& cam : cfg.cameras)
 *       if (cam.enabled)
 *           for (const auto& [cap, val] : cam.capabilities)
 *               camera.setCameraAttribute(cap, val);
 * @endcode
 */

#ifndef LIBCONFIG_H
#define LIBCONFIG_H

#include "liblog.h"
#include <map>
#include <string>
#include <vector>

namespace dashcam::config {

// ─── per-domain config structs ────────────────────────────────────────────────

/**
 * @brief H.264 software encoder parameters (x264enc; Orin Nano has no NVENC).
 *
 * XML section: <Encoder>
 */
struct EncoderConfig {
    int         bitrate     = 8000;          ///< Target bitrate in kbps.
    std::string speedPreset = "ultrafast";   ///< x264 speed preset name.
    int         keyIntMax   = 60;            ///< Maximum frames between keyframes.
    std::string tune        = "";            ///< x264 tune string (e.g. "zerolatency"); empty = omit.
};

/**
 * @brief Cairo overlay rendering parameters.
 *
 * XML section: <Overlay>
 */
struct OverlayConfig {
    bool        enabled             = true;           ///< Render telemetry on recorded video.
    float       backgroundOpacity   = 0.85f;           ///< Alpha of the semi-transparent label backing.
    float       fontSize            = 14.0f;          ///< Label font size in points.
    std::string fontFace            = "Monospace Bold"; ///< Pango font description string.
};

/**
 * @brief Configuration for a single recognized camera.
 *
 * XML element: <Cameras><Camera name="..." type="...">
 *
 * @c type must be "CSI" or "USB".  @c capabilities feeds directly into
 * iCamera::setCameraAttribute(); keys must match the driver's documented
 * attribute names exactly (e.g. "exposure time, absolute", "brightness").
 */
struct CameraConfig {
    std::string name;                    ///< User-defined label, e.g. "front" or "cabin-left".
    std::string type;                    ///< Interface type: "CSI" or "USB".
    bool        enabled     = true;      ///< Skip this camera if false.
    std::string device;                  ///< Device node (e.g. "/dev/video2"); empty = auto-detect.
    int         sensorId    = 0;         ///< Argus sensor-id for CSI cameras; ignored for USB.
    int         formatIndex = 0;         ///< Index into getCameraList() videoFormats to activate.
    std::map<std::string, std::string> capabilities; ///< Driver capability name → string value.
};

/**
 * @brief System-level runtime parameters.
 *
 * XML section: <System>
 */
struct SystemConfig {
    std::string archivePath  = "./archive"; ///< Directory for recorded MKV files.
    int         warmupFrames = 9;           ///< Frames to discard before enabling recording.
};

/**
 * @brief Aggregated application configuration.
 *
 * Root XML element: <DashcamConfig>
 *
 * @c cameras is ordered; index 0 corresponds to the first <Camera> element, etc.
 */
struct AppConfig {
    EncoderConfig             encoder;
    OverlayConfig             overlay;
    std::vector<CameraConfig> cameras; ///< All recognized cameras in document order.
    SystemConfig              system;
};

// ─── reader / writer ─────────────────────────────────────────────────────────

/**
 * @brief Loads and saves XML config files for the dashcam system.
 *
 * Typical bootstrap flow — generate a default config from discovered hardware:
 * @code
 *   // 1. Discover cameras via libcamera.
 *   std::vector<dashcam::camera::cameraInfo> found;
 *   dashcam::camera::getCameraList(found);
 *
 *   // 2. Build an AppConfig with discovered cameras and all-default settings.
 *   dashcam::config::AppConfig cfg;
 *   for (const auto& info : found) {
 *       dashcam::config::CameraConfig cc;
 *       cc.name   = info.address;          // e.g. "/dev/video0"
 *       cc.type   = (info.type == dashcam::camera::CAMERA_TYPE::CSI) ? "CSI" : "USB";
 *       cc.device = info.address;
 *       cc.sensorId = static_cast<int>(info.deviceId);
 *       cfg.cameras.push_back(cc);
 *   }
 *
 *   // 3. Persist as a starting-point config file.
 *   ConfigReader::save("config/dashcam.xml", cfg);
 * @endcode
 */
class ConfigReader {
public:
    /**
     * @brief Parse @p filePath and populate @p config.
     *
     * On parse error or missing root node, @p config is left at its default
     * values and a diagnostic is printed to stderr.
     *
     * @param[in]  filePath  Path to the XML config file.
     * @param[out] config    Receives parsed values; defaults are preserved for
     *                       any field not present in the file.
     * @return @c true on success; @c false if the file cannot be opened or
     *         contains no <DashcamConfig> root node.
     */
    static bool load(const std::string& filePath, AppConfig& config,
                     const dashcam::log::LogCallback& log = {});

    /**
     * @brief Serialise @p config to an XML file at @p filePath.
     *
     * All sections and all camera entries are written unconditionally,
     * including those that carry default values.  The intent is to produce a
     * fully annotated starting-point file that the operator can edit.
     *
     * The output file is always UTF-8 with a standard XML declaration.
     * Parent directories must already exist; the file is truncated if it exists.
     *
     * @param[in] filePath  Destination path for the XML file.
     * @param[in] config    Configuration to serialise.
     * @param[in] log       Optional callback for error messages; silent if empty.
     * @return @c true on success; @c false if the file cannot be written.
     */
    static bool save(const std::string& filePath, const AppConfig& config,
                     const dashcam::log::LogCallback& log = {});
};

} // namespace dashcam::config

#endif // LIBCONFIG_H
