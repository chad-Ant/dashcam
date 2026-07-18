/**
 * @file libstereocam.h
 * @brief Stereo depth estimation using a pair of iCamera sources.
 *
 * StereoRangefinder wraps two iCamera instances (typically Camera_USB), loads
 * stereo calibration from an OpenCV-format YAML file, and produces per-pixel
 * disparity maps via either the VPI CUDA SGM backend (preferred) or OpenCV
 * StereoSGBM (CPU fallback).
 *
 * Calibration YAML must contain: M1, D1, M2, D2, R, T, imageWidth, imageHeight.
 * R1/R2/P1/P2/Q and remap maps are derived at load time via cv::stereoRectify
 * and cv::initUndistortRectifyMap and are not required in the file.
 *
 * Typical usage:
 * @code
 *   Camera_USB left(leftInfo), right(rightInfo);
 *   dashcam::stereo::StereoRangefinder sf(&left, &right);
 *   sf.setFormatIndex(leftFmtIdx, rightFmtIdx);
 *   sf.loadCalibration("config/stereo_calib.yml");
 *   sf.open();
 *   sf.start();
 *
 *   dashcam::stereo::DepthResult result;
 *   while (running) {
 *       if (sf.computeDepth(result))
 *           float d = sf.getDistanceAt(320, 180, result);
 *   }
 *   sf.stop();
 *   sf.close();
 * @endcode
 *
 * @note Requires OpenCV for calibration loading and the CPU backend.
 * @note Requires NVIDIA VPI 3.x (JetPack 6.2) for the CUDA SGM backend.
 * @note Both iCamera pointers must remain valid for the lifetime of this object.
 * @note captureFrame() output is assumed BGR, 3 bytes/pixel (standard iCamera contract).
 */

#ifndef LIBSTEREOCAM_H
#define LIBSTEREOCAM_H

#include "libcamera.h"
#include "liblog.h"
#include <cstdint>
#include <string>
#include <vector>

namespace dashcam::stereo {

// ─── error codes ─────────────────────────────────────────────────────────────

/**
 * @brief Return codes for StereoRangefinder operations.
 */
enum class StereoError {
    NONE            =  0,  ///< Operation succeeded.
    NOT_CALIBRATED  = -1,  ///< start() or computeDepth() called before loadCalibration().
    CAMERA_FAILED   = -2,  ///< One or both cameras failed to open/start.
    VPI_INIT_FAILED = -3,  ///< VPI initialisation failed; fell back to OpenCV_CPU.
    INVALID_STATE   = -4,  ///< Method called in wrong lifecycle state.
};

// ─── data structures ─────────────────────────────────────────────────────────

/**
 * @brief Calibration data derived from a stereo calibration YAML file.
 *
 * Loaded once by StereoRangefinder::loadCalibration().  Stores the minimum
 * information needed for rectification and depth conversion without exposing
 * OpenCV types in this header.
 *
 * leftMapX/Y and rightMapX/Y are CV_32FC1 remap arrays in row-major order,
 * each of size imageWidth × imageHeight floats.
 */
struct CalibrationData {
    int    imageWidth  = 0;   ///< Width used during calibration (pixels).
    int    imageHeight = 0;   ///< Height used during calibration (pixels).
    double focalPx     = 0.0; ///< Rectified focal length in pixels (P1[0][0] = P2[0][0]).
    double baselineM   = 0.0; ///< Stereo baseline in metres (–P2[0][3] / P2[0][0]).
    double Q[16]       = {};  ///< 4×4 disparity-to-depth matrix, row-major, from stereoRectify.

    std::vector<float> leftMapX;  ///< Left rectification remap X coords, row-major.
    std::vector<float> leftMapY;  ///< Left rectification remap Y coords, row-major.
    std::vector<float> rightMapX; ///< Right rectification remap X coords, row-major.
    std::vector<float> rightMapY; ///< Right rectification remap Y coords, row-major.
};

/**
 * @brief Output of a single stereo disparity computation.
 *
 * Pixel coordinates in disparityF32 are in rectified image space.
 * Use StereoRangefinder::getDistanceAt() to convert disparity to depth.
 */
struct DepthResult {
    std::vector<float> disparityF32;  ///< Disparity in pixels per pixel, row-major. ≤0 = invalid.
    int     width       = 0;     ///< Image width in pixels.
    int     height      = 0;     ///< Image height in pixels.
    int64_t leftTsMs    = 0;     ///< System timestamp when left captureFrame() returned (epoch ms).
    int64_t rightTsMs   = 0;     ///< System timestamp when right captureFrame() returned (epoch ms).
    int64_t timeDeltaMs = 0;     ///< |leftTsMs − rightTsMs|; > 50 ms suggests frame desync.
    bool    valid       = false; ///< true if disparityF32 is populated.
};

// ─── StereoRangefinder ────────────────────────────────────────────────────────

/**
 * @brief Stereo depth estimator that wraps two iCamera sources.
 *
 * Manages the full lifecycle of both cameras together.  The VPI_CUDA backend
 * is attempted on start(); if VPI initialisation fails, the object automatically
 * falls back to OpenCV_CPU.  activeBackend() reflects the backend in use.
 *
 * Frame synchronisation:
 * - Left camera is captured first, then right immediately after.
 * - At 10 fps with max-buffers=1 appsink, the inter-capture delta is typically
 *   < 2 ms since the frame is already queued in the appsink buffer.
 * - timeDeltaMs in DepthResult reports the measured delta; values exceeding
 *   syncToleranceMs are logged via the injected callback but do not abort the computation.
 *
 * Disparity scale:
 * - VPI CUDA SGM:       S16 × 1/32 → float pixels.
 * - OpenCV StereoSGBM:  S16 × 1/16 → float pixels.
 * Both backends write the same unit (pixels) into DepthResult::disparityF32.
 *
 * Depth formula: Z = focalPx × baselineM / disparity_px  (pinhole, rectified plane).
 * Full 3D reprojection: use CalibrationData::Q directly.
 */
class StereoRangefinder {
public:
    enum class Backend { VPI_CUDA, OpenCV_CPU };

    /**
     * @brief Construct bound to two camera sources.
     * @param left     Left camera; not owned; must outlive this object.
     * @param right    Right camera; not owned; must outlive this object.
     * @param backend  Preferred backend; falls back to OpenCV_CPU on VPI failure.
     */
    StereoRangefinder(dashcam::camera::iCamera* left,
                      dashcam::camera::iCamera* right,
                      Backend backend = Backend::VPI_CUDA);

    ~StereoRangefinder();

    StereoRangefinder(const StereoRangefinder&)            = delete;
    StereoRangefinder& operator=(const StereoRangefinder&) = delete;

    /**
     * @brief Load stereo calibration from an OpenCV YAML file.
     *
     * Required keys: M1 (3×3), D1 (1×5 or 5×1), M2 (3×3), D2 (1×5 or 5×1),
     * R (3×3), T (3×1 or 1×3), imageWidth, imageHeight.
     *
     * Derives R1/R2/P1/P2/Q via cv::stereoRectify and computes float remap
     * arrays via cv::initUndistortRectifyMap.  Must be called before start().
     *
     * @param yamlPath  Path to the calibration YAML file.
     * @return true on success; false if the file is missing a required key.
     */
    bool loadCalibration(const std::string& yamlPath);

    /** @brief Read-only access to loaded calibration (valid after loadCalibration()). */
    const CalibrationData& calibration() const { return calib_; }

    /**
     * @brief Select the capture format for each camera before open().
     * @param leftIdx   Index into left cameraInfo::videoFormats.
     * @param rightIdx  Index into right cameraInfo::videoFormats.
     */
    void setFormatIndex(uint16_t leftIdx, uint16_t rightIdx);

    /**
     * @brief Inject a log callback for VPI/calibration diagnostics.
     *        Defaults to a no-op (silent) if not set.
     */
    void setLogCallback(dashcam::log::LogCallback cb);

    /**
     * @brief Open and configure both cameras.
     *
     * Calls open() then setCameraVideoFormat() on each camera.
     *
     * @return CAMERA_FAILED if either camera enters ERROR state; NONE otherwise.
     */
    StereoError open();

    /**
     * @brief Allocate buffers, initialise the chosen backend, and start both cameras.
     *
     * On VPI initialisation failure the backend silently downgrades to OpenCV_CPU.
     *
     * @pre loadCalibration() must have succeeded.
     * @return NOT_CALIBRATED if no calibration is loaded.
     * @return CAMERA_FAILED if either camera does not reach RUNNING state.
     * @return NONE on success.
     */
    StereoError start();

    /**
     * @brief Capture a synchronised frame pair and compute the disparity map.
     *
     * Captures left then right, applies BGR→grey, rectification, and disparity
     * estimation with the active backend.  Fills @p result on success.
     *
     * @param[out] result          Receives disparity map and timing metadata.
     * @param[in]  syncToleranceMs Warn to stderr if |leftTs − rightTs| exceeds this.
     * @return true if @p result is populated; false on capture failure.
     */
    bool computeDepth(DepthResult& result, int syncToleranceMs = 50);

    /**
     * @brief Convert disparity at a pixel to depth in metres.
     *
     * Formula: Z = focalPx × baselineM / disparity_px.
     *
     * @param x       Column in the rectified image (0-based).
     * @param y       Row    in the rectified image (0-based).
     * @param result  A valid DepthResult from computeDepth().
     * @return Depth in metres; −1.0f if out-of-bounds, invalid, or disparity ≤ 0.
     */
    float getDistanceAt(int x, int y, const DepthResult& result) const;

    /**
     * @brief Stop both cameras and release backend resources.
     * @return NONE (always; per-camera errors are discarded).
     */
    StereoError stop();

    /**
     * @brief Close both cameras.
     * @return NONE (always).
     */
    StereoError close();

    /** @brief Backend currently in use; may differ from the constructor argument after fallback. */
    Backend activeBackend() const { return activeBackend_; }

private:
    dashcam::log::LogCallback  log_{};
    dashcam::camera::iCamera* left_;
    dashcam::camera::iCamera* right_;
    Backend         requestedBackend_;
    Backend         activeBackend_;
    uint16_t        leftFmtIdx_  = 0;
    uint16_t        rightFmtIdx_ = 0;
    CalibrationData calib_;
    bool            calibLoaded_ = false;

    // Per-frame working buffers (allocated in start(), never reallocated after)
    std::vector<uint8_t> leftBuf_;    ///< BGR capture output for left camera.
    std::vector<uint8_t> rightBuf_;   ///< BGR capture output for right camera.
    std::vector<uint8_t> leftGray_;   ///< Greyscale converted from leftBuf_.
    std::vector<uint8_t> rightGray_;  ///< Greyscale converted from rightBuf_.
    std::vector<uint8_t> leftRect_;   ///< Rectified greyscale for left.
    std::vector<uint8_t> rightRect_;  ///< Rectified greyscale for right.

    // Opaque VPI handles — typed as void* to keep VPI headers out of this header.
    void* vpiStream_    = nullptr;  ///< VPIStream
    void* vpiPayload_   = nullptr;  ///< VPIPayload (SGM estimator)
    void* vpiLeft_      = nullptr;  ///< VPIImage U8 — left rectified input
    void* vpiRight_     = nullptr;  ///< VPIImage U8 — right rectified input
    void* vpiDisparity_ = nullptr;  ///< VPIImage S16 — disparity output

    /// cv::Ptr<cv::StereoSGBM>* — allocated in start() when activeBackend_ == OpenCV_CPU.
    void* sgbm_ = nullptr;

    /** @brief Create VPI stream, images, and SGM payload. @return false on any failure. */
    bool initVpi();
    /** @brief Release all VPI objects; safe to call when any pointer is null. */
    void destroyVpi();
    /** @brief Delete the cached StereoSGBM object; safe to call when sgbm_ is null. */
    void destroySgbm();

    /** @brief Rec.601 BGR→grey conversion (3 bytes/px → 1 byte/px). */
    static void bgrToGray(const uint8_t* bgr, uint8_t* gray, int width, int height);

    /**
     * @brief Bilinear remap of a greyscale image (border pixels clamped to 0).
     * @param src   Input grey pixels (width × height).
     * @param mapX  Float X map, row-major, size width × height.
     * @param mapY  Float Y map, row-major, same size.
     * @param dst   Output grey pixels (same dimensions).
     */
    static void applyRectification(const uint8_t* src,
                                   const std::vector<float>& mapX,
                                   const std::vector<float>& mapY,
                                   uint8_t* dst, int width, int height);

    bool computeDepthVpi(DepthResult& result);
    bool computeDepthCpu(DepthResult& result);
};

} // namespace dashcam::stereo

#endif // LIBSTEREOCAM_H
