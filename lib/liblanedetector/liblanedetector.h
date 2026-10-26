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
 *       --memPoolSize=workspace:512M --builderOptimizationLevel=2 \
 *       --profilingVerbosity=detailed
 * @endcode
 * Build it on the Jetson that runs it: a plan built elsewhere loads with a
 * "different models of devices" warning.  Expect ~413 MB with the FC head in
 * FP16.  The 803 MB engine used until 2026-10-25 had an FP32 head and took
 * ~14 ms/frame against ~9.4 ms.  --profilingVerbosity=detailed only lets
 * tools/lane_eval/engine_check.sh show per-layer precision.
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
 *       createBin() again — with the detector stopped (it is refused while
 *       the inference thread runs) — and re-register before each subsequent
 *       cam.start().  Each start() begins a clean run: poll(), hasFreshResult()
 *       and processedFrameCount() forget the previous run.
 * @note poll(), stats() and processedFrameCount() are thread-safe; the other
 *       methods are not.
 * @note Two limits set the inference rate: branchMaxFps (a videorate at the
 *       branch inlet) and targetHz (a gate in the inference thread), with the
 *       leaky queue / appsink dropping whatever inference cannot keep up
 *       with.  With rateGate = kLegacy (the default) the wall-clock gate
 *       aliases against a branch running at targetHz and delivers ~13 Hz for
 *       20; RateGate::kPts does not (see LaneDetectorConfig::rateGate).
 */

#ifndef LIBLANEDETECTOR_H
#define LIBLANEDETECTOR_H

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <gst/gst.h>
#include "liblog.h"
#include "ufld_decode.h"

namespace dashcam::lane {

// ─── output types ─────────────────────────────────────────────────────────────

/**
 * @brief When a result's frame was taken and how long each stage took.
 *
 * Mono times are CLOCK_MONOTONIC ns (std::chrono::steady_clock).  Stage
 * times are filled only with LaneDetectorConfig::collectTimings (−1 otherwise).
 */
struct LaneTiming {
    int64_t ptsNs         = -1;  ///< buffer PTS as delivered (stream time, not converted to
                                 ///< running time) / processFrame's ptsNs
    int64_t captureMonoNs = -1;  ///< PTS on CLOCK_MONOTONIC via the pipeline's system clock; −1 unknown
    int64_t pullMonoNs    = -1;  ///< appsink pull returned (processFrame: call entry)
    int64_t doneMonoNs    = -1;  ///< decode finished
    float   h2dUs = -1;          ///< host→device copy of the frame
    float   preUs = -1;          ///< preprocess kernel
    float   trtUs = -1;          ///< TensorRT enqueueV3
    float   d2hUs = -1;          ///< output copies
    float   decUs = -1;          ///< CPU decode (decodeUfldV2 + egoFromSlots)
};

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

    /// The four boundary slots as decoded (native domains, ufld_decode.h):
    /// what numLanes / currentLaneIndex / lateralOffset were derived from.
    std::array<LaneSlot, 4> slots{};

    /// NaN/Inf values found in this frame's output tensors.
    int32_t nonFinite = 0;

    LaneTiming timing;
};

/// Monotonic counters since construction (all paths, thread and processFrame).
struct LaneStats {
    uint64_t pulled       = 0;   ///< samples pulled from the appsink
    uint64_t gated        = 0;   ///< pulled frames dropped by the targetHz gate
    uint64_t mapFail      = 0;   ///< gst_buffer_map failures
    uint64_t sizeMismatch = 0;   ///< frames whose size != branch size (skipped)
    uint64_t h2dFail      = 0;
    uint64_t preFail      = 0;
    uint64_t trtFail      = 0;
    uint64_t d2hFail      = 0;
    uint64_t syncFail     = 0;
    uint64_t nonFinite    = 0;   ///< frames with any NaN/Inf output value
    uint64_t processed    = 0;   ///< valid results since construction
    int64_t  lastErrorMonoNs = -1;
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
    /// NOTE: this default (0) differs from libconfig's <Detection>
    /// LaneInputCropBottom (0.2059); set both crops explicitly.
    float inputCropBottom = 0.0f;

    /// Frame-rate cap applied at the branch inlet (videorate drop-only), so
    /// conversion elements run at most this often instead of the sensor rate.
    /// 0 = uncapped.  Keep >= targetHz.
    uint32_t branchMaxFps = 20;

    /// First row anchor as a fraction of source height; anchors are
    /// linspace(rowAnchorStart, 1.0, numRowAnchors) like the UFLD v2 demo.
    float rowAnchorStart = 0.42f;

    /// Where boundary x-coords are sampled to determine lane widths and
    /// vehicle position, as a fraction of the model's IMPLIED frame — not of
    /// the source frame: with the IMX296 defaults 0.90 is source row 810.7 of
    /// 1088 (LaneGeometry::refRowSrc has the source row for any config).
    float laneReferenceY = 0.90f;

    /// Inference rate cap in Hz.  0 = run as fast as the pipeline allows.
    /// Recommended: 20 for urban driving, 10 for highway-only.
    uint32_t targetHz = 20;

    /// How targetHz is enforced.
    ///  kLegacy  wall-clock time since the last inference (the original
    ///           gate).  When the branch already runs at ~targetHz it drops
    ///           every frame that arrives a hair early: ~13 Hz for 20 on the
    ///           IMX296 (plan bug B3).  Kept as the default until the higher
    ///           GPU load of a true 20 Hz is signed off for v0.2/v0.3.
    ///  kPts     decimates on buffer timestamps with half a frame interval of
    ///           slack: exactly targetHz from any source rate >= targetHz.
    ///           Frames without a PTS fall back to the wall-clock gate.
    enum class RateGate : uint8_t { kLegacy = 0, kPts = 1 };
    RateGate rateGate = RateGate::kLegacy;

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
    /// width × LaneGeometry::branchH, i.e. height − cropTopPx − cropBottomPx
    /// with each crop rounded to an EVEN row count (not height·(1 − top −
    /// bottom)) — the appsink caps pin that size, and size pinning, rate
    /// limiting and cropping become the override's responsibility.
    std::string gstConversion;

    /// Measure per-stage times into LaneResult::timing (CUDA events around
    /// each GPU stage, read after the existing stream sync).  Off in
    /// production; the frame timestamps and LaneStats are always on.
    bool collectTimings = false;

    /// Name of the inference thread (pthread_setname_np, ≤ 15 chars kept).
    std::string threadName = "lane_infer";

    /// Log sink, wired like the other dashcam libraries.  Defaults to the
    /// process-wide liblog callback so construction-time errors are visible;
    /// replace via LaneDetector::setLogCallback().
    dashcam::log::LogCallback log = dashcam::log::getCallback();
};

// ─── geometry ─────────────────────────────────────────────────────────────────

/**
 * @brief Where the model looks, for one source size and config.
 *
 * Rows are continuous edge coordinates in the FULL source frame (row r spans
 * [r, r+1)).  The decode works in the model's "implied frame": UFLD v2 was
 * trained on the bottom cropRatio of a frame resized to
 * modelInputH / cropRatio rows, so fraction f of that implied frame (row
 * anchors, laneReferenceY) lands on source row impliedTopSrc + f·impliedH.
 */
struct LaneGeometry {
    uint32_t srcW = 0, srcH = 0;          ///< full tee frame
    uint32_t cropTopPx = 0;               ///< rows removed by the branch, top
    uint32_t cropBottomPx = 0;            ///< rows removed by the branch, bottom
    uint32_t branchW = 0, branchH = 0;    ///< appsink frame (bytes = W·H·3)
    float    keptFrac = 0;                ///< branchH / srcH
    float    effRatio = 0;                ///< min(1, cropRatio / keptFrac)
    int      resizedH = 0;                ///< preprocess resize target rows
    int      cropTopRows = 0;             ///< resizedH − modelInputH
    float    preYScale = 0;               ///< branch rows per model row
    float    preYOff = 0;                 ///< branch row of model row 0's top edge

    double impliedH = 0;                  ///< implied frame height, source rows
    double impliedTopSrc = 0;             ///< implied frame top, source row
    double refRowSrc = 0;                 ///< laneReferenceY in source rows
    double anchorRowSrcFirst = 0;         ///< first row anchor, source row
    double anchorRowSrcLast = 0;          ///< last row anchor, source row

    double impliedToSourceRow(double f) const { return impliedTopSrc + f * impliedH; }
    double sourceRowToImplied(double r) const { return (r - impliedTopSrc) / impliedH; }
};

/// Copies of the tensors one processFrame() call used (parity / debugging).
struct LaneDebugTensors {
    std::vector<float> input;                  ///< 1×3×H×W, CHW RGB, normalised
    std::array<std::vector<float>, 4> outputs; ///< loc_row, loc_col, exist_row, exist_col
};

// ─── detector ─────────────────────────────────────────────────────────────────

class LaneDetectorImpl;
struct LaneDetectorTestHook;

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
     * Call before cam.start(), with the detector stopped: while the
     * inference thread runs it is refused (nullptr + ERROR log).
     * The bin pins the full srcWidth × srcHeight first, so a tee of any other
     * size fails caps negotiation instead of being cropped silently.
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

    /** @brief Frames successfully processed by CUDA/TRT since the last start()
     *         (lifetime total: stats().processed). */
    uint64_t processedFrameCount() const;

    /**
     * @brief True when at least one successful result arrived within maxAgeMs.
     *
     * Intended for startup readiness and runtime watchdogs; unlike "zero lanes",
     * a stale result means the inference path is not receiving/processing frames.
     */
    bool hasFreshResult(uint32_t maxAgeMs) const;

    /**
     * @brief Replace the log sink (same pattern as the other libraries).
     * @return false (and the old sink logs an ERROR) while the inference
     *         thread runs — the thread reads the sink.
     */
    bool setLogCallback(dashcam::log::LogCallback cb);

    /**
     * @brief Run one branch-sized BGR frame synchronously (replay / tests).
     *
     * Same CUDA → TRT → decode path as the inference thread, and the same
     * bookkeeping (sequence, poll(), hasFreshResult()).  Refused (returns
     * false, logs an ERROR) while the inference thread runs.
     *
     * @param bgr    geometry().branchW × branchH × 3 bytes, the frame the
     *               appsink would deliver (already branch-cropped).
     * @param bytes  size of bgr; must equal branchW·branchH·3.
     * @param ptsNs  frame timestamp, carried for the caller (−1 if unknown).
     * @param out    the result (valid=false on failure).
     * @param dbg    optional: receives copies of the input/output tensors.
     * @return true when a valid result was produced.
     */
    bool processFrame(const uint8_t* bgr, size_t bytes, int64_t ptsNs,
                      LaneResult& out, LaneDebugTensors* dbg = nullptr);

    /** @brief Crop / preprocess / implied-frame geometry in use. */
    const LaneGeometry& geometry() const;

    /** @brief Counters since construction (thread-safe). */
    LaneStats stats() const;

    /**
     * @brief Receive every valid result as it is produced.
     *
     * Called on the inference thread (and from processFrame()) right after
     * the result becomes poll()'s — keep the handler short (enqueue only).
     * Only allowed before start(); returns false (and logs) otherwise.
     */
    bool setResultSink(std::function<void(const LaneResult&)> sink);

private:
    friend struct LaneDetectorTestHook;
    std::unique_ptr<LaneDetectorImpl> impl_;
};

/**
 * @brief Test-only access to LaneDetector internals (lane_test).  Inert
 * unless called: production code never touches it.
 */
struct LaneDetectorTestHook {
    enum class Fault : uint8_t {
        H2D,          ///< host→device copy fails
        Preprocess,   ///< preprocess launch fails
        Enqueue,      ///< enqueueV3 fails
        D2H,          ///< an output copy fails
        NaNOutputs,   ///< loc_row / loc_col come back all-NaN (decode still runs)
    };

    /// Make the next `count` frames hit `fault` (thread or processFrame path).
    static void injectFault(LaneDetector& det, Fault fault, int count);

    /// Run a frame that is ALREADY model-sized (modelInputW × modelInputH
    /// BGR, e.g. the training-geometry /var/tmp/ufld_calib frames) with an
    /// identity preprocess.  The detector must have been built with a
    /// modelInputW × modelInputH branch; returns false otherwise.
    static bool runModelSized(LaneDetector& det, const uint8_t* bgr, size_t bytes,
                              LaneResult& out, LaneDebugTensors* dbg = nullptr);

    /// Receive the raw mapped bytes of every frame the inference thread maps:
    /// what the appsink delivered, after the targetHz gate and before the size
    /// check.  Called on the inference thread.  Set before start() only;
    /// returns false while the thread runs.  nullptr clears it.
    static bool onFrame(LaneDetector& det, std::function<void(const uint8_t*, size_t)> cb);

    /// What one valid inference used and produced, for onInference().  Every
    /// pointer is valid only during the callback (the frame is still mapped,
    /// the outputs are the detector's pinned host buffers).
    struct InferenceView {
        const uint8_t* frame = nullptr;      ///< branch-sized BGR frame (branchW × branchH × 3)
        size_t         frameBytes = 0;
        const LaneResult* result = nullptr;  ///< as published (sequence and timing filled)
        const float*   outputs[4] = {};      ///< loc_row, loc_col, exist_row, exist_col
        size_t         outputFloats[4] = {};
    };
    /// Receive every VALID inference right after its result was published
    /// (thread and processFrame paths), on the calling/inference thread —
    /// copy what you need and return.  Set before start() only; returns
    /// false while the thread runs.  nullptr clears it.
    static bool onInference(LaneDetector& det, std::function<void(const InferenceView&)> cb);
};

} // namespace dashcam::lane

#endif // LIBLANEDETECTOR_H
