/**
 * @file libdriverstate.h
 * @brief GStreamer-integrated TensorRT drowsiness classifier (binary ResNet18).
 *
 * Plugs into a Camera_GST pipeline as a leaky branch via addBranch().  An
 * internal inference thread pulls BGR frames from the branch's appsink, runs
 * YuNet DNN face detection (OpenCV FaceDetectorYN, CPU) and crops to the
 * largest detected face — matching the face-crop training data — then
 * preprocesses on the GPU (resize to 224×224 + ImageNet normalisation), runs
 * TRT inference, applies a sigmoid to the model's single output logit, and
 * stores the resulting drowsiness probability for poll().  Frames with no
 * detectable face skip classification and report faceDetected = false.
 *
 * Model: fine-tuned ResNet18, binary Drowsy-vs-Natural classification
 * (huggingface.co/Teen-Different/Driver-Drowsiness-Detection, Dataset 1:
 * 224×224 RGB face crops, ImageNet mean/std, BCEWithLogitsLoss → the engine
 * outputs ONE raw logit; sigmoid is applied here, not in the graph).
 *
 * Engine build (once, INSIDE the l4t-ml container that runs this app — TRT
 * engines are locked to the exact TensorRT version that built them):
 * @code
 *   /usr/src/tensorrt/bin/trtexec --onnx=drowsiness_resnet18.onnx \
 *       --saveEngine=models/drowsiness_resnet18_fp16.engine --fp16 \
 *       --memPoolSize=workspace:512M --builderOptimizationLevel=2
 * @endcode
 *
 * Typical lifecycle — attach to the driver-facing USB camera:
 * @code
 *   DriverStateConfig cfg;
 *   cfg.enginePath = "models/drowsiness_resnet18_fp16.engine";
 *   DriverStateDetector detector(640, 480, cfg);
 *
 *   cam.addBranch("driverstate", detector.createBin(), true);  // leaky queue
 *   cam.open();
 *   cam.setCameraVideoFormat(idx);
 *   cam.start();
 *   detector.start();
 *
 *   while (running) {
 *       DriverStateResult r = detector.poll();
 *       if (r.valid && r.state == DriverState::DROWSY) { // alert }
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
 * @note Sigmoid polarity: sigmoid(logit) = P(natural), NOT P(drowsy) as the
 *       model card loosely suggests — established empirically against
 *       labelled dataset samples (see DriverStateConfig::positiveIsDrowsy).
 *       A live bench spot-check (eyes open vs closed) remains worthwhile.
 * @note At 2 Hz the GPU duty cycle for this model is well under 1% —
 *       negligible next to the lane detector.
 */

#ifndef LIBDRIVERSTATE_H
#define LIBDRIVERSTATE_H

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <gst/gst.h>
#include "liblog.h"

namespace dashcam::driver {

// ─── result types ─────────────────────────────────────────────────────────────

enum class DriverState : uint8_t {
    NATURAL = 0,   ///< Alert / normal driving posture.
    DROWSY  = 1,   ///< Drowsiness detected (probability ≥ drowsyThreshold).
};

/// Long-horizon fatigue assessment derived from the running score (see
/// FatigueScorer).  Complements — never replaces — the caller's acute
/// micro-sleep alert on the instantaneous DriverState.
enum class FatigueLevel : uint8_t {
    OK      = 0,   ///< score >= cautionScore.
    CAUTION = 1,   ///< score in [warningScore, cautionScore): subtle cue.
    WARNING = 2,   ///< score in (fatigueScore, warningScore): repeated alert.
    FATIGUE = 3,   ///< score <= fatigueScore SUSTAINED fatigueSustainSec:
                   ///< high-confidence fatigue — strong "pull over" alarm.
};

struct DriverStateResult {
    /// Monotonic sequence number of successfully processed frames.
    uint64_t sequence = 0;

    /// Thresholded classification of the most recent frame.
    DriverState state = DriverState::NATURAL;

    /// P(drowsy) in [0, 1] — sigmoid of the model's logit.  Prefer this over
    /// state for downstream smoothing/hysteresis (e.g. alert only after N
    /// consecutive polls above threshold).
    float drowsyProbability = 0.0f;

    /// True when the classifier ran on a face region: a face was detected in
    /// this frame, or within the last faceHoldSec (last-known box reused —
    /// a hard head-droop can still momentarily defeat the detector, so brief
    /// dropouts keep classifying).  Always true when face detection is
    /// disabled.  When
    /// false the classifier did NOT run: state/drowsyProbability are reset
    /// to NATURAL/0.  A sustained false means the camera cannot see a face.
    bool faceDetected = false;

    /// false until the first frame completes (classified or no-face).
    /// Always check this before acting on the other fields.
    bool valid = false;

    // ── long-horizon fatigue score (see FatigueScorer) ────────────────────────

    /// Running fatigue score: scoreInitial (100) = fresh, <= fatigueScore (0)
    /// = fatigued.  Updated every inference tick; clamped to
    /// [scoreLower, fatigueCap].
    float fatigueScore = 100.0f;

    /// Current upper clamp on the score: starts at scoreUpper and decays
    /// capDecayPerHour per driving hour (time-on-task fatigue), floored at
    /// capDecayFloor.  Reset together with the score.
    float fatigueCap = 100.0f;

    /// Tiered assessment of fatigueScore (FATIGUE requires the score to hold
    /// in the fatigue zone for fatigueSustainSec — no single-dip alarms).
    FatigueLevel fatigueLevel = FatigueLevel::OK;
};

// ─── fatigue scoring ──────────────────────────────────────────────────────────

/// Tuning for the FatigueScorer.  All time windows in seconds, all scores in
/// points.  Defaults implement the agreed design: -10 per completed 10 s of
/// drowsiness, +5 per completed 10 s awake, -10 per drowsiness-correlated
/// lane drift, cap decaying 10/driving-hour, FATIGUE after 5 min in the zone.
struct FatigueScoreConfig {
    float scoreInitial = 100.0f;  ///< Starting / reset score.
    float scoreUpper   = 100.0f;  ///< Cap before time-on-task decay.
    float scoreLower   = -10.0f;  ///< Hard floor.

    /// A drowsy episode must COMPLETE each full chunk of this many seconds to
    /// deduct drowsyChunkPenalty; recovering mid-chunk discards the partial
    /// (recover within the window → no deduction).
    float drowsyChunkSec     = 10.0f;
    float drowsyChunkPenalty = 10.0f;

    /// Awake accrual, symmetric to the above: each completed chunk of awake
    /// time earns awakeChunkReward (asymmetric on purpose — fatigue builds
    /// faster than it heals).
    float awakeChunkSec    = 10.0f;
    float awakeChunkReward = 5.0f;

    // ── drowsiness-correlated lane drift (fed via setLaneOffset) ─────────────

    /// |lateralOffset| at or above which the vehicle counts as drifting onto /
    /// across a lane line (liblanedetector units: 0 centred, ±1 on the line).
    float laneDepartThresh = 0.8f;

    /// The drift-and-jerk-back signature: a departure that BEGINS during a
    /// drowsy episode and returns under laneDepartThresh within this many
    /// seconds deducts laneDriftPenalty (once per episode).  Longer
    /// excursions are treated as deliberate lane changes — no deduction.
    float laneReturnSec    = 10.0f;
    float laneDriftPenalty = 10.0f;

    // ── time-on-task decay ───────────────────────────────────────────────────

    /// Every full driving hour lowers the score cap by this much...
    float capDecayPerHour = 10.0f;
    /// ...but never below this floor.
    float capDecayFloor   = 50.0f;

    // ── level thresholds ─────────────────────────────────────────────────────

    float cautionScore = 60.0f;   ///< Below this: CAUTION.
    float warningScore = 30.0f;   ///< Below this: WARNING.
    float fatigueScore = 0.0f;    ///< At/below this: fatigue zone.

    /// The score must stay in the fatigue zone this long, uninterrupted,
    /// before FATIGUE is reported (implementation A: the high-confidence
    /// alarm needs 5 min of persistence; it must never cry wolf).
    float fatigueSustainSec = 300.0f;

    /// No-face frames freeze the score (neither accrual runs): a blocked or
    /// averted camera is not evidence of drowsiness.  False resumes awake
    /// accrual during no-face instead.
    bool noFaceFreezes = true;
};

/**
 * @brief Long-horizon driver-fatigue score (100 awake .. <= 0 fatigued).
 *
 * Pure, deterministic state machine — every entry point takes an explicit
 * timestamp so tests can drive synthetic timelines (hours in microseconds).
 * DriverStateDetector embeds one and ticks it from the inference thread;
 * it is exposed here for direct construction in tests.
 *
 * Thread-safe: all methods lock an internal mutex.
 */
class FatigueScorer {
public:
    using TimePoint = std::chrono::steady_clock::time_point;

    /// Consistent snapshot of the three published values under ONE lock —
    /// prefer this to three separate score()/cap()/level() calls on the hot
    /// path (and so a reader never straddles an update).
    struct Snapshot { float score; float cap; FatigueLevel level; };

    /// Inconsistent config (mis-ordered thresholds, non-positive windows, ...)
    /// is CLAMPED to a safe, consistent set rather than rejected — a config
    /// typo must never disable driver monitoring.  A non-empty log receives one
    /// WARN describing the clamp.  Never throws.
    explicit FatigueScorer(const FatigueScoreConfig& cfg, TimePoint now,
                           dashcam::log::LogCallback log = {});

    /// Feed one classifier tick.  valid=false ticks are ignored;
    /// faceDetected=false ticks freeze (or accrue awake — see
    /// FatigueScoreConfig::noFaceFreezes).
    void update(bool valid, bool faceDetected, bool drowsy, TimePoint now);

    /// Feed the latest lane lateral offset (liblanedetector's
    /// LaneResult::lateralOffset).  Invalid samples HOLD any drift in progress
    /// (a lane crossing itself destabilises boundary detection — the strongest
    /// signal must survive a brief validity gap); the return is confirmed by a
    /// later valid in-lane sample and the laneReturnSec timeout still bounds it.
    void laneOffset(float offset, bool offsetValid, TimePoint now);

    /// Reset to a fresh session: score/cap restored, session clock zeroed
    /// (deliberate: a reset after a real break IS a fresh session).
    void reset(TimePoint now);

    Snapshot     snapshot() const;
    float        score() const;
    float        cap()   const;
    FatigueLevel level() const;

private:
    static void sanitize(FatigueScoreConfig& c,
                         const dashcam::log::LogCallback& log);
    void   applyCapDecay(TimePoint now);   // callers hold mutex_
    void   clampScore();
    void   updateLevel(TimePoint now);

    FatigueScoreConfig cfg_;
    mutable std::mutex mutex_;

    float     score_;
    float     cap_;
    TimePoint sessionStart_;

    // Episode accrual (chunk-quantised; partial chunks discard on transition).
    bool      drowsyRun_   = false;
    bool      awakeRun_    = false;
    TimePoint drowsyStart_{};
    TimePoint awakeStart_{};
    int       drowsyChunksPaid_ = 0;
    int       awakeChunksPaid_  = 0;

    // Drowsiness-correlated lane drift (once per drowsy episode).
    bool      driftArmed_   = false;   // departure seen, awaiting return
    bool      driftPaid_    = false;   // this episode already deducted
    TimePoint departTime_{};

    // FATIGUE sustain gate.
    bool         inFatigueZone_ = false;
    TimePoint    fatigueZoneSince_{};
    FatigueLevel level_ = FatigueLevel::OK;
};

// ─── configuration ────────────────────────────────────────────────────────────

struct DriverStateConfig {
    /// Path to the serialised TRT engine.  Must be built with trtexec inside
    /// the SAME container (TRT version) that runs this library.
    std::string enginePath;

    /// Model input dimensions — must match the engine's input tensor.
    /// The drowsiness ResNet18 was trained on 224×224 face crops.
    uint32_t modelInputW = 224;
    uint32_t modelInputH = 224;

    /// Inference rate cap in Hz.  0 = run as fast as the pipeline allows.
    /// 2 Hz is the design rate for driver monitoring on this rig.
    uint32_t targetHz = 2;

    /// Frame-rate cap applied at the branch inlet (videorate drop-only), so
    /// conversion elements run at most this often instead of the camera rate.
    /// 0 = uncapped.  Keep >= targetHz.  UVC cameras cannot deliver 2 fps
    /// natively (C270 minimum is 5), so the branch does the dropping.
    uint32_t branchMaxFps = 2;

    /// P(drowsy) at or above which state == DROWSY.
    float drowsyThreshold = 0.5f;

    /// Sigmoid polarity.  FALSE (default): a positive logit indicates
    /// Natural, i.e. sigmoid(logit) = P(natural) and this library reports
    /// 1 − sigmoid as P(drowsy).  Determined empirically (2026-07-21) against
    /// labelled Driver Drowsiness Dataset samples — alert faces saturate the
    /// sigmoid toward 1 — and consistent with the dataset's alphabetical
    /// class order (Drowsy=0, Non Drowsy=1) under BCEWithLogitsLoss.  Note
    /// this CONTRADICTS the model card's loose "logit predicting drowsiness"
    /// wording; set true only if a bench check shows the opposite.
    bool positiveIsDrowsy = false;

    // ── YuNet DNN face crop (matches the face-crop training data) ─────────────

    /// Detect the driver's face (OpenCV YuNet DNN, CPU) and crop the classifier
    /// input to it.  YuNet is far more robust to tilted / off-axis / partially
    /// closed faces than a Haar cascade — important for a camera mounted low on
    /// the dashboard or steering column, which sees the face from below.
    /// Frames with no face skip classification and report faceDetected = false.
    /// A few ms per frame on one CPU core at 640×480 — negligible at 2 fps.
    bool faceDetection = true;

    /// YuNet face-detection model (ONNX).  A copy of OpenCV Zoo's
    /// face_detection_yunet_2023mar.onnx is vendored under models/ next to the
    /// TRT engines.
    std::string faceModelPath = "models/face_detection_yunet_2023mar.onnx";

    /// YuNet detection confidence in [0, 1]; boxes below this are discarded.
    /// Lower accepts more off-axis / partially-occluded faces (fewer NO-FACE
    /// dropouts) at the cost of occasional false boxes.
    float faceScoreThreshold = 0.6f;

    /// Run YuNet on the frame downscaled by this factor (0.25–1.0), then map
    /// the detected box back to full resolution for the crop.  Detection cost
    /// scales ~quadratically, so 0.5 ≈ a quarter of the CPU; the classifier
    /// still crops from the full-resolution frame, so its input is unchanged.
    /// Default 0.5: validated on-device (2026-07-21) to hold 100% face-detect
    /// recall even at a low dashboard mount / off-axis face, at ~11 ms vs
    /// ~51 ms/frame full-res.  Raise toward 1.0 only if detection misses a
    /// small / distant face; 1.0 disables the downscale (and its resize).
    float faceDetectScale = 0.5f;

    /// Margin added on each side of the detected face box before cropping,
    /// as a fraction of the box side.  The training crops are loose face
    /// crops, not tight detector boxes.
    float faceMarginFrac = 0.2f;

    /// Smallest face accepted, as a fraction of the frame height.  Rejects
    /// spurious small detections in cabin clutter.
    float faceMinSizeFrac = 0.15f;

    /// When detection fails, keep classifying the LAST-KNOWN face region for
    /// this many seconds before reporting faceDetected = false.  Even YuNet can
    /// momentarily drop a hard head-droop / heavy occlusion — exactly the
    /// frames that matter — while a belted driver's face barely moves, so the
    /// last box stays valid.  0 disables the hold.
    float faceHoldSec = 3.0f;

    /// Fallback framing when faceDetection is false: centre-crop the source
    /// frame to a square before the 224×224 resize, avoiding anamorphic
    /// distortion of a 4:3/16:9 frame.
    bool centerCropSquare = true;

    // ── ImageNet normalisation (must match training) ──────────────────────────

    float meanR = 0.485f, meanG = 0.456f, meanB = 0.406f;
    float stdR  = 0.229f, stdG  = 0.224f, stdB  = 0.225f;

    /// True when the camera tee emits NVMM (CSI/nvarguscamerasrc) buffers,
    /// false (default) for system memory — the driver camera is USB/UVC.
    bool sourceIsNVMM = false;

    /// Optional override of the auto-built conversion chain (advanced).  When
    /// non-empty it is used verbatim and MUST deliver BGR frames of exactly
    /// srcWidth × srcHeight; rate limiting becomes the override's
    /// responsibility.
    std::string gstConversion;

    /// Long-horizon fatigue-score tuning (see FatigueScorer).
    FatigueScoreConfig score;

    /// Log sink, wired like the other dashcam libraries.  Defaults to the
    /// process-wide liblog callback so construction-time errors are visible;
    /// replace via DriverStateDetector::setLogCallback().
    dashcam::log::LogCallback log = dashcam::log::getCallback();
};

// ─── detector ─────────────────────────────────────────────────────────────────

class DriverStateImpl;

/**
 * @brief TensorRT drowsiness classifier driven by the GStreamer pipeline.
 *
 * Frame path: camera tee → leaky queue → branch bin appsink →
 *   inference thread: cudaMemcpyAsync → CUDA kernel (crop+resize+normalise) →
 *   TRT enqueueV3 → sigmoid + threshold → DriverStateResult (polled).
 */
class DriverStateDetector {
public:
    /**
     * @param srcWidth   Source frame width (tee output resolution).
     * @param srcHeight  Source frame height.
     * @param config     Engine path and model parameters.
     * @throws std::runtime_error on engine load, engine/config mismatch, or
     *         CUDA allocation failure.
     */
    DriverStateDetector(uint32_t srcWidth, uint32_t srcHeight,
                        const DriverStateConfig& config);
    ~DriverStateDetector();

    DriverStateDetector(const DriverStateDetector&)            = delete;
    DriverStateDetector& operator=(const DriverStateDetector&) = delete;

    /**
     * @brief Create the GstBin to pass to Camera_GST::addBranch().
     *
     * The returned bin contains the conversion chain and an appsink.
     * Ownership transfers to the camera pipeline via addBranch().
     * Call before cam.start().  May be called again after cam.stop() to
     * re-register the branch for the next start() cycle.
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

    /** @brief Return the most recent driver state result (thread-safe). */
    DriverStateResult poll() const;

    /** @brief Number of frames successfully processed (including valid no-face frames). */
    uint64_t processedFrameCount() const;

    /** @brief True when a successful result arrived within maxAgeMs. */
    bool hasFreshResult(uint32_t maxAgeMs) const;

    /**
     * @brief Feed the latest lane lateral offset into the fatigue scorer
     *        (bridge from liblanedetector's LaneResult::lateralOffset /
     *        lateralValid — this library deliberately has no compile-time
     *        dependency on the lane detector).  Thread-safe.
     */
    void setLaneOffset(float offset, bool offsetValid);

    /**
     * @brief Reset the fatigue score to a fresh session (score and cap back
     *        to full, session clock zeroed).  Wire to a GPIO button in a
     *        later version; software-triggered for now.  Thread-safe.
     */
    void resetScore();

    /** @brief Replace the log sink (same pattern as the other libraries). */
    void setLogCallback(dashcam::log::LogCallback cb);

private:
    std::unique_ptr<DriverStateImpl> impl_;
};

} // namespace dashcam::driver

#endif // LIBDRIVERSTATE_H
