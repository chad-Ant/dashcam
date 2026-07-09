/**
 * @file libsigndetector.h
 * @brief GStreamer-integrated TensorRT road sign detector.
 *
 * Plugs into a Camera_GST pipeline as a leaky branch via addBranch().  An
 * internal inference thread pulls BGR frames from the branch's appsink,
 * preprocesses them on the GPU, runs TRT inference, decodes YOLOv8-style
 * output with per-class NMS, and stores the result for poll().
 *
 * Detected classes: SPD_LIM_START, SPD_LIM_STOP, LANE_DIR.
 *
 * Engine build (once, on the target Jetson):
 * @code
 *   trtexec --onnx=models/model_roadsign.onnx \
 *            --saveEngine=models/model_roadsign.engine \
 *            --fp16
 * @endcode
 *
 * Typical lifecycle:
 * @code
 *   SignDetectorConfig cfg;
 *   cfg.enginePath = "models/model_roadsign.engine";
 *   SignDetector detector(1920, 1080, cfg);
 *
 *   cam.addBranch("signs", detector.createBin(), true);   // leaky queue
 *   cam.open();
 *   cam.setCameraVideoFormat(idx);
 *   cam.start();
 *   detector.start();
 *
 *   while (running) {
 *       SignResult r = detector.poll();
 *       for (const auto& d : r.detections) { ... }
 *   }
 *
 *   cam.stop();       // forces appsink flush → inference thread drains
 *   detector.stop();  // joins inference thread
 *   cam.close();
 * @endcode
 *
 * @note A Camera_GST stop()/start() cycle clears all branches.  Create a
 *       new SignDetector (or call createBin() again and re-register) before
 *       each subsequent cam.start().
 * @note Not thread-safe except for poll(), which is safe from any thread.
 * @note For the 1080p60 CSI front stream, the leaky branch queue limits the
 *       sign detector to the rate it can sustain (~5 fps).  No explicit
 *       throttle is needed.
 */

#ifndef LIBSIGNDETECTOR_H
#define LIBSIGNDETECTOR_H

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <gst/gst.h>

namespace dashcam::sign {

// ─── result types ─────────────────────────────────────────────────────────────

enum class SignClass : uint8_t {
    SPD_LIM_START = 0,
    SPD_LIM_STOP  = 1,
    LANE_DIR      = 2,
};

/**
 * @brief A single road sign detection.
 *
 * Bounding box coordinates are in source-image pixel space
 * (the frame dimensions passed to the SignDetector constructor).
 */
struct Detection {
    SignClass classId;
    float     confidence;
    float     x1, y1;  ///< Top-left corner in source-image pixels.
    float     x2, y2;  ///< Bottom-right corner in source-image pixels.
};

struct SignResult {
    std::vector<Detection> detections; ///< Empty until the first inference frame completes.
};

// ─── configuration ────────────────────────────────────────────────────────────

struct SignDetectorConfig {
    /// Path to the serialised TRT engine (build with trtexec on this Jetson).
    std::string enginePath;

    /// Model input dimensions.  Must match the engine's input tensor.
    uint32_t modelInputW = 640;
    uint32_t modelInputH = 640;

    /// Number of sign classes the model was trained on.  Must equal 3 for
    /// the SPD_LIM_START / SPD_LIM_STOP / LANE_DIR model.
    int numClasses = 3;

    /// Detection confidence threshold; boxes below this score are discarded.
    float confThreshold = 0.50f;

    /// IoU threshold for per-class greedy NMS.
    float nmsThreshold  = 0.45f;

    /// Inference rate cap in Hz.  0 = run as fast as the pipeline allows.
    /// Recommended: 5 — signs are visible for multiple seconds; higher rates
    /// waste GPU without improving detection coverage.
    uint32_t targetHz = 5;

    /// ImageNet normalisation applied after scaling pixels to [0, 1].
    float meanR = 0.485f, meanG = 0.456f, meanB = 0.406f;
    float stdR  = 0.229f, stdG  = 0.224f, stdB  = 0.225f;

    /// GStreamer element chain inserted inside the branch bin before the BGR
    /// appsink.  Must accept whatever format the camera tee outputs and
    /// produce video/x-raw in system memory so videoconvert can finish the job.
    ///
    /// For CSI cameras (NV12 in NVMM memory from nvarguscamerasrc tee):
    ///   "nvvidconv ! video/x-raw,format=BGRx ! videoconvert"
    /// For USB cameras (YUYV / MJPEG-decoded frames in system memory):
    ///   "videoconvert"
    std::string gstConversion =
        "nvvidconv ! video/x-raw,format=BGRx ! videoconvert";
};

// ─── detector ─────────────────────────────────────────────────────────────────

class SignDetectorImpl;

class SignDetector {
public:
    /**
     * @param srcWidth   Source frame width  (must match cam.captureFrame layout).
     * @param srcHeight  Source frame height.
     * @param config     Engine path and model parameters.
     * @throws std::runtime_error on engine load failure or CUDA allocation failure.
     */
    SignDetector(uint32_t srcWidth, uint32_t srcHeight,
                 const SignDetectorConfig& config);
    ~SignDetector();

    SignDetector(const SignDetector&)            = delete;
    SignDetector& operator=(const SignDetector&) = delete;

    /**
     * @brief Create the GstBin to pass to Camera_GST::addBranch().
     *
     * The returned element is a GstBin containing the format-conversion chain
     * and an appsink.  Ownership transfers to the camera pipeline via
     * addBranch() / gst_bin_add().  Call this before cam.start().
     *
     * @return A floating GstElement* (GstBin with a sink ghost pad).
     *         Returns nullptr if GStreamer element construction fails.
     */
    GstElement* createBin();

    /**
     * @brief Start the inference thread.  Call after cam.start().
     */
    void start();

    /**
     * @brief Stop the inference thread.  Call after cam.stop() to guarantee
     *        the appsink is in flushing state before the thread is joined.
     */
    void stop();

    /**
     * @brief Return the most recent detection result (thread-safe).
     *
     * Returns an empty SignResult until the first inference frame completes.
     */
    SignResult poll() const;

private:
    std::unique_ptr<SignDetectorImpl> impl_;
};

} // namespace dashcam::sign

#endif // LIBSIGNDETECTOR_H
