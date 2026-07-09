/**
 * @file libdriverstate.h
 * @brief GStreamer-integrated TensorRT driver state classifier.
 *
 * Plugs into a Camera_GST pipeline as a leaky branch via addBranch().  An
 * internal inference thread pulls BGR frames from the branch's appsink,
 * preprocesses on the GPU, runs TRT inference, applies softmax over the
 * three output logits, and stores the winning class and its confidence for
 * poll().
 *
 * Detected states: NEUTRAL, SLEEPY, DISTRACTED.
 *
 * Engine build (once, on the target Jetson):
 * @code
 *   trtexec --onnx=models/model_driverstate.onnx \
 *            --saveEngine=models/model_driverstate.engine \
 *            --fp16
 * @endcode
 *
 * Typical lifecycle — attach to USB cam C (driver-facing camera):
 * @code
 *   DriverStateConfig cfg;
 *   cfg.enginePath = "models/model_driverstate.engine";
 *   DriverStateDetector detector(640, 360, cfg);
 *
 *   cam.addBranch("driverstate", detector.createBin(), true);
 *   cam.open();
 *   cam.setCameraVideoFormat(idx);
 *   cam.start();
 *   detector.start();
 *
 *   while (running) {
 *       DriverStateResult r = detector.poll();
 *       if (r.valid) { // r.state, r.confidence }
 *   }
 *
 *   cam.stop();
 *   detector.stop();
 *   cam.close();
 * @endcode
 *
 * @note A Camera_GST stop()/start() cycle clears all branches.  Call
 *       createBin() again and re-register before each subsequent cam.start().
 * @note poll() is thread-safe; all other methods are not.
 * @note At 1 Hz the GPU duty cycle for this model is < 0.3% — negligible.
 */

#ifndef LIBDRIVERSTATE_H
#define LIBDRIVERSTATE_H

#include <cstdint>
#include <memory>
#include <string>
#include <gst/gst.h>

namespace dashcam::driver {

// ─── result types ─────────────────────────────────────────────────────────────

enum class DriverState : uint8_t {
    NEUTRAL    = 0,
    SLEEPY     = 1,
    DISTRACTED = 2,
};

struct DriverStateResult {
    /// Predicted driver state.
    DriverState state = DriverState::NEUTRAL;

    /// Softmax probability of the predicted state in [0, 1].
    float confidence = 0.0f;

    /// false until the first inference frame completes.
    /// Always check this before acting on state/confidence.
    bool valid = false;
};

// ─── configuration ────────────────────────────────────────────────────────────

struct DriverStateConfig {
    /// Path to the serialised TRT engine (built with trtexec on this Jetson).
    std::string enginePath;

    /// Model input dimensions — must match the engine's input tensor.
    /// Classification models are commonly 224×224; adjust to match training.
    uint32_t modelInputW = 224;
    uint32_t modelInputH = 224;

    /// Inference rate cap in Hz.  0 = run as fast as the pipeline allows.
    /// 1 Hz is sufficient for driver alertness monitoring and costs < 0.3%
    /// of the Orin Nano's GPU budget.
    uint32_t targetHz = 1;

    /// Apply softmax to the model's raw output logits before extracting the
    /// winning class.  Set false if the model already includes a softmax layer.
    bool applySoftmax = true;

    /// ImageNet normalisation (must match training).
    float meanR = 0.485f, meanG = 0.456f, meanB = 0.406f;
    float stdR  = 0.229f, stdG  = 0.224f, stdB  = 0.225f;

    /// GStreamer conversion chain inserted before the BGR appsink.
    /// USB cameras (YUYV / MJPEG-decoded) deliver system-memory frames so
    /// a plain videoconvert is sufficient.  For a CSI source use
    /// "nvvidconv ! video/x-raw,format=BGRx ! videoconvert" instead.
    std::string gstConversion = "videoconvert";
};

// ─── detector ─────────────────────────────────────────────────────────────────

class DriverStateImpl;

/**
 * @brief TensorRT driver state classifier driven by the GStreamer pipeline.
 *
 * Frame path: camera tee → leaky queue → branch bin appsink →
 *   inference thread: cudaMemcpyAsync → CUDA kernel → TRT enqueueV3 →
 *   softmax + argmax → DriverStateResult (polled by caller).
 */
class DriverStateDetector {
public:
    /**
     * @param srcWidth   Source frame width (tee output resolution).
     * @param srcHeight  Source frame height.
     * @param config     Engine path and model parameters.
     * @throws std::runtime_error on engine load or CUDA allocation failure.
     */
    DriverStateDetector(uint32_t srcWidth, uint32_t srcHeight,
                        const DriverStateConfig& config);
    ~DriverStateDetector();

    DriverStateDetector(const DriverStateDetector&)            = delete;
    DriverStateDetector& operator=(const DriverStateDetector&) = delete;

    /**
     * @brief Create the GstBin to pass to Camera_GST::addBranch().
     *
     * Ownership of the returned element transfers to the camera pipeline.
     * Call before cam.start().  May be called again after cam.stop() to
     * re-register the branch for the next start() cycle.
     *
     * @return Floating GstElement* (GstBin with a sink ghost pad),
     *         or nullptr if element construction fails.
     */
    GstElement* createBin();

    /** @brief Start the inference thread.  Call after cam.start(). */
    void start();

    /**
     * @brief Stop the inference thread.
     *
     * Call after cam.stop() so the appsink is in flushing state before
     * the thread is joined.
     */
    void stop();

    /** @brief Return the most recent driver state result (thread-safe). */
    DriverStateResult poll() const;

private:
    std::unique_ptr<DriverStateImpl> impl_;
};

} // namespace dashcam::driver

#endif // LIBDRIVERSTATE_H
