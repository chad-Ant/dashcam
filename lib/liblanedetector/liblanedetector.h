/**
 * @file liblanedetector.h
 * @brief GStreamer-integrated TensorRT lane detector.
 *
 * Plugs into a Camera_GST pipeline as a leaky branch via addBranch().  An
 * internal inference thread pulls BGR frames from the branch's appsink,
 * preprocesses them on the GPU (CUDA kernel), runs TRT inference (UFLD v1),
 * decodes lane boundaries, and stores the result for poll().
 *
 * Engine build (once, on the target Jetson):
 * @code
 *   trtexec --onnx=ufld.onnx --saveEngine=models/ufld.engine --fp16
 * @endcode
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

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <gst/gst.h>

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
    /// Total lanes visible between detected boundaries (n boundaries → n-1 lanes).
    /// 0 if fewer than 2 boundaries were found.
    int8_t numLanes = 0;

    /// 0-based index of the lane the ego vehicle is currently in (0 = leftmost).
    /// -1 = sentinel: vehicle centre is outside all detected lane boundaries
    ///      (drifted off-road).  Treat as an error — do not use as an index.
    int8_t currentLaneIndex = -1;

    /// One entry per lane.  Size == numLanes.
    std::vector<LaneDirection> laneAllowedDirections;
};

// ─── configuration ────────────────────────────────────────────────────────────

struct LaneDetectorConfig {
    /// Path to the serialised TRT engine (built with trtexec on this Jetson).
    std::string enginePath;

    /// Model input dimensions — must match the engine's input tensor.
    uint32_t modelInputW = 800;
    uint32_t modelInputH = 288;

    // ── UFLD v1 postprocessing ────────────────────────────────────────────────

    /// Number of column grid cells (griding_num).
    /// Class index == gridingNum means "no lane at this row".
    int gridingNum    = 100;

    /// Number of row-anchor sample positions.  Must be ≤ 56.
    int numRowAnchors = 56;

    /// Number of lane boundary lines predicted by the model.
    int numLanes      = 4;

    /// Source-image y-position (fraction of height) where boundary x-coords
    /// are sampled to determine lane widths and vehicle position.
    float laneReferenceY = 0.90f;

    /// Inference rate cap in Hz.  0 = run as fast as the pipeline allows.
    /// Recommended: 20 for urban driving, 10 for highway-only.
    uint32_t targetHz = 20;

    // ── normalisation (must match training) ───────────────────────────────────

    float meanR = 0.485f, meanG = 0.456f, meanB = 0.406f;
    float stdR  = 0.229f, stdG  = 0.224f, stdB  = 0.225f;

    /// GStreamer element chain inserted inside the branch bin before the BGR
    /// appsink.  Must accept whatever format the camera tee emits and convert
    /// it to video/x-raw in system memory ready for videoconvert.
    ///
    /// CSI cameras (NV12/NVMM from nvarguscamerasrc tee):
    ///   "nvvidconv ! video/x-raw,format=BGRx ! videoconvert"
    /// USB cameras (YUYV/MJPEG-decoded frames in system memory):
    ///   "videoconvert"
    std::string gstConversion =
        "nvvidconv ! video/x-raw,format=BGRx ! videoconvert";
};

// ─── detector ─────────────────────────────────────────────────────────────────

class LaneDetectorImpl;

/**
 * @brief TensorRT lane detector driven by the GStreamer pipeline.
 *
 * Frame path: camera tee → leaky queue → sign bin appsink →
 *   inference thread: cudaMemcpyAsync → CUDA kernel → TRT enqueueV3 →
 *   UFLD decode → latestResult_ (polled by caller).
 */
class LaneDetector {
public:
    /**
     * @param srcWidth   Source frame width (tee output resolution).
     * @param srcHeight  Source frame height.
     * @param config     Engine path and model parameters.
     * @throws std::runtime_error on engine load or CUDA allocation failure.
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

private:
    std::unique_ptr<LaneDetectorImpl> impl_;
};

} // namespace dashcam::lane

#endif // LIBLANEDETECTOR_H
