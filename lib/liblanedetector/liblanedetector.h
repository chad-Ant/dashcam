/**
 * @file liblanedetector.h
 * @brief GStreamer-integrated TensorRT lane detector (UFLD v2).
 *
 * Plugs into a Camera_GST pipeline as a leaky branch via addBranch().  An
 * internal inference thread pulls BGR frames from the branch's appsink,
 * preprocesses them on the GPU (CUDA kernel: bottom-crop + resize + normalise),
 * runs TRT inference (Ultra-Fast-Lane-Detection v2), decodes the row/column
 * anchor heads into lane boundaries, and stores the result for poll().
 *
 * Engine build (once, INSIDE the l4t-ml container that runs this app — TRT
 * engines are locked to the exact TensorRT version that built them, and the
 * container's TRT differs from the host's):
 * @code
 *   /usr/src/tensorrt/bin/trtexec --onnx=ufldv2_culane_res18_320x1600.onnx \
 *       --saveEngine=models/culane_res18_fp16.engine --fp16 \
 *       --memPoolSize=workspace:512M --builderOptimizationLevel=2
 * @endcode
 * The engine's IO signature is validated against LaneDetectorConfig at load:
 * input (1,3,H,W) float32 and the four UFLD v2 outputs loc_row / loc_col /
 * exist_row / exist_col.
 *
 * Typical lifecycle:
 * @code
 *   LaneDetectorConfig cfg;
 *   cfg.enginePath = "models/ufld.engine";
 *   LaneDetector detector(1920, 1080, cfg);
 *
 *   cam.addBranch("lanes", detector.createBin(), true);  // leaky queue
 *   cam.open();
 *   cam.setCameraVideoFormat(idx);
 *   cam.start();
 *   detector.start();
 *
 *   while (running) {
 *       LaneResult r = detector.poll();
 *       // r.numLanes, r.currentLaneIndex, r.laneAllowedDirections
 *   }
 *
 *   cam.stop();       // forces appsink flush → inference thread drains
 *   detector.stop();  // joins inference thread
 *   cam.close();
 * @endcode
 *
 * @note A Camera_GST stop()/start() cycle clears all branches.  Call
 *       createBin() again and re-register before each subsequent cam.start().
 * @note poll() is thread-safe; all other methods are not.
 * @note The leaky branch queue limits the lane detector to the rate it can
 *       sustain (~15–30 fps on Orin Nano).  No explicit throttle is needed.
 */

#ifndef LIBLANEDETECTOR_H
#define LIBLANEDETECTOR_H

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <gst/gst.h>
#include "liblog.h"

namespace dashcam::lane {

// ─── output types ─────────────────────────────────────────────────────────────

/**
 * @brief Permitted travel direction for a single lane.
 *
 * @note Defaults to Straight.  Augment via the sign-detection pipeline once
 *       road-marking classification is integrated.
 */
enum class LaneDirection : uint8_t {
    Straight = 0,
    Left,
    Right,
    UTurn,
};

struct LaneResult {
    /// True only after a frame completed CUDA + TensorRT + decode successfully.
    /// A valid result may still contain zero lanes (e.g. a blank/indoor scene).
    bool valid = false;

    /// Monotonic sequence number of successfully processed frames.
    uint64_t sequence = 0;

    /// Total lanes visible between detected boundaries (n boundaries → n-1 lanes).
    /// 0 if fewer than 2 boundaries were found.
    int8_t numLanes = 0;

    /// 0-based index of the lane the ego vehicle is currently in (0 = leftmost).
    /// -1 = sentinel: vehicle centre is outside all detected lane boundaries
    ///      (drifted off-road).  Treat as an error — do not use as an index.
    int8_t currentLaneIndex = -1;

    /// Lateral projection of the vehicle centre inside its current lane,
    /// sampled at laneReferenceY: 0 = centred, -1 = on the left boundary,
    /// +1 = on the right boundary.  |value| near 1 means the vehicle is
    /// drifting onto a lane line — the driver-fatigue scorer consumes this
    /// (see libdriverstate).  Only meaningful when lateralValid is true.
    float lateralOffset = 0.0f;

    /// True when lateralOffset was computed this frame: >= 2 boundaries
    /// detected AND the vehicle centre lies inside a lane
    /// (currentLaneIndex >= 0).
    bool lateralValid = false;

    /// Fixed-capacity directions for the at-most-three lanes produced by the
    /// four-boundary UFLD v2 head.  Only entries [0, numLanes) are meaningful;
    /// fixed storage avoids a heap allocation on every inference result.
    std::array<LaneDirection, 3> laneAllowedDirections{
        LaneDirection::Straight,
        LaneDirection::Straight,
        LaneDirection::Straight};
};

// ─── configuration ────────────────────────────────────────────────────────────

struct LaneDetectorConfig {
    /// Path to the serialised TRT engine.  Must be built with trtexec inside
    /// the SAME container (TRT version) that runs this library.
    std::string enginePath;

    /// Model input dimensions — must match the engine's input tensor.
    /// UFLD v2 CULane res18: 1600×320.
    uint32_t modelInputW = 1600;
    uint32_t modelInputH = 320;

    // ── UFLD v2 head geometry (culane_res18 defaults) ─────────────────────────

    /// Location grid cells of the row-anchor head (num_cell_row).
    int numCellRow    = 200;

    /// Row-anchor sample positions (num_row).
    int numRowAnchors = 72;

    /// Location grid cells of the column-anchor head (num_cell_col).
    int numCellCol    = 100;

    /// Column-anchor sample positions (num_col).
    int numColAnchors = 81;

    /// Lane slots predicted by the model.  UFLD v2 fixes the semantics:
    /// slots 1,2 = ego-adjacent boundaries (row head), 0,3 = outer (col head).
    int numLanes      = 4;

    /// Fraction of the ORIGINAL frame height the model should see (bottom),
    /// reproducing the training Resize(H/crop_ratio)+crop.  When inputCropTop
    /// already removed part of the frame, the preprocess ROI compensates so
    /// the model's effective view stays as close to this as the branch crop
    /// allows.
    float cropRatio   = 0.6f;

    /// Fraction of the frame height cropped off the TOP inside the lane
    /// branch, before conversion/preprocess (0 = full frame, 0.5 = keep the
    /// lower half).  Cuts VIC/CPU conversion and H2D cost; lanes live in the
    /// lower half anyway.
    float inputCropTop = 0.5f;

    /// Fraction of the frame height additionally cropped off the BOTTOM of
    /// the branch (0 = none).  With inputCropTop this selects a mid-frame
    /// band — e.g. 0.5 top + 0.2059 bottom on the 1088-row IMX296 keeps rows
    /// [544, 864): a 320-row band that matches the model input height, so the
    /// preprocess does no vertical resampling at all.
    float inputCropBottom = 0.0f;

    /// Frame-rate cap applied at the branch inlet (videorate drop-only), so
    /// conversion elements run at most this often instead of the sensor rate.
    /// 0 = uncapped.  Keep >= targetHz.
    uint32_t branchMaxFps = 20;

    /// First row anchor as a fraction of source height; anchors are
    /// linspace(rowAnchorStart, 1.0, numRowAnchors) like the UFLD v2 demo.
    float rowAnchorStart = 0.42f;

    /// Source-image y-position (fraction of height) where boundary x-coords
    /// are sampled to determine lane widths and vehicle position.
    float laneReferenceY = 0.90f;

    /// Inference rate cap in Hz.  0 = run as fast as the pipeline allows.
    /// Recommended: 20 for urban driving, 10 for highway-only.
    uint32_t targetHz = 20;

    // ── normalisation (must match training) ───────────────────────────────────

    float meanR = 0.485f, meanG = 0.456f, meanB = 0.406f;
    float stdR  = 0.229f, stdG  = 0.224f, stdB  = 0.225f;

    /// True when the camera tee emits NVMM (CSI/nvarguscamerasrc) buffers,
    /// false for system memory (USB v4l2src / file playback).  Selects the
    /// auto-built conversion chain: nvvidconv (VIC crop+convert in one pass)
    /// vs videocrop+videoconvert.
    bool sourceIsNVMM = true;

    /// Optional override of the auto-built conversion chain (advanced).  When
    /// non-empty it is used verbatim and MUST deliver BGR frames of exactly
    /// width × (height − (inputCropTop+inputCropBottom)·height) — the appsink
    /// caps pin that size, and rate limiting/cropping become the override's
    /// responsibility.
    std::string gstConversion;

    /// Log sink, wired like the other dashcam libraries.  Defaults to the
    /// process-wide liblog callback so construction-time errors are visible;
    /// replace via LaneDetector::setLogCallback().
    dashcam::log::LogCallback log = dashcam::log::getCallback();
};

// ─── detector ─────────────────────────────────────────────────────────────────

class LaneDetectorImpl;

/**
 * @brief TensorRT lane detector driven by the GStreamer pipeline.
 *
 * Frame path: camera tee → leaky queue → lane bin appsink →
 *   inference thread: cudaMemcpyAsync → CUDA kernel → TRT enqueueV3 →
 *   UFLD v2 decode (row + column anchor heads) → latestResult_ (polled).
 */
class LaneDetector {
public:
    /**
     * @param srcWidth   FULL source frame width (tee output resolution).
     * @param srcHeight  FULL source frame height.  The branch applies
     *                   config.inputCropTop internally — pass the uncropped
     *                   tee resolution here.
     * @param config     Engine path and model parameters.
     * @throws std::runtime_error on engine load, engine/config mismatch, or
     *         CUDA allocation failure.
     */
    LaneDetector(uint32_t srcWidth, uint32_t srcHeight,
                 const LaneDetectorConfig& config);
    ~LaneDetector();

    LaneDetector(const LaneDetector&)            = delete;
    LaneDetector& operator=(const LaneDetector&) = delete;

    /**
     * @brief Create the GstBin to pass to Camera_GST::addBranch().
     *
     * The returned bin contains the conversion chain and an appsink.
     * Ownership transfers to the camera pipeline via addBranch().
     * Call before cam.start().
     *
     * @return Floating GstElement* (GstBin with a sink ghost pad),
     *         or nullptr if GStreamer element construction fails.
     */
    GstElement* createBin();

    /** @brief Start the inference thread.  Call after cam.start(). */
    void start();

    /**
     * @brief Stop the inference thread.
     *
     * Recommended order: cam.stop() first (puts appsink into flushing so the
     * thread drains immediately), then detector.stop() (joins the thread).
     */
    void stop();

    /** @brief Return the most recent lane result (thread-safe). */
    LaneResult poll() const;

    /** @brief Number of frames successfully processed by CUDA/TRT. */
    uint64_t processedFrameCount() const;

    /**
     * @brief True when at least one successful result arrived within maxAgeMs.
     *
     * Intended for startup readiness and runtime watchdogs; unlike "zero lanes",
     * a stale result means the inference path is not receiving/processing frames.
     */
    bool hasFreshResult(uint32_t maxAgeMs) const;

    /** @brief Replace the log sink (same pattern as the other libraries). */
    void setLogCallback(dashcam::log::LogCallback cb);

private:
    std::unique_ptr<LaneDetectorImpl> impl_;
};

} // namespace dashcam::lane

#endif // LIBLANEDETECTOR_H
