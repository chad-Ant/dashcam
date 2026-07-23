#include "libdriverstate.h"
#include "libdriverstate_preprocess.h"

#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <cuda_runtime.h>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace dashcam::driver {

// ─── file-local log helper (same pattern as the other detector libs) ─────────

static void doLog(const dashcam::log::LogCallback& cb, dashcam::log::LogLevel lvl,
                  const char* fmt, ...) {
    if (!cb) return;
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    cb(lvl, buf);
}

static void cudaCheck(cudaError_t err, const char* what) {
    if (err != cudaSuccess)
        throw std::runtime_error(
            std::string("libdriverstate: ") + what
            + ": " + cudaGetErrorString(err));
}

// TRT build/runtime diagnostics routed into the library's log callback.
class TrtLogger : public nvinfer1::ILogger {
public:
    const dashcam::log::LogCallback* cb = nullptr;

    void log(Severity severity, const char* msg) noexcept override {
        if (severity > Severity::kWARNING) return;
        const auto lvl = severity == Severity::kWARNING
            ? dashcam::log::LogLevel::WARN
            : dashcam::log::LogLevel::ERROR;
        if (cb && *cb) (*cb)(lvl, std::string("[TRT/driver] ") + msg);
        else std::fprintf(stderr, "[TRT/driver] %s\n", msg);
    }
};

// ─── FatigueScorer ───────────────────────────────────────────────────────────

// Clamp an inconsistent config into a safe, consistent one instead of
// rejecting it: a mistyped <DriverScore> value must never disable driver
// monitoring.  Emits one WARN if anything was changed.
void FatigueScorer::sanitize(FatigueScoreConfig& c,
                             const dashcam::log::LogCallback& log) {
    bool changed = false;
    auto fix = [&](float& field, float value) { field = value; changed = true; };

    if (c.drowsyChunkSec   <= 0.0f) fix(c.drowsyChunkSec,   10.0f);
    if (c.awakeChunkSec    <= 0.0f) fix(c.awakeChunkSec,    10.0f);
    if (c.laneReturnSec    <= 0.0f) fix(c.laneReturnSec,    10.0f);
    if (c.laneDepartThresh <= 0.0f) fix(c.laneDepartThresh, 0.8f);
    if (c.fatigueSustainSec < 0.0f) fix(c.fatigueSustainSec, 0.0f);
    if (c.capDecayPerHour   < 0.0f) fix(c.capDecayPerHour,   0.0f);

    // Score range: keep scoreLower <= scoreUpper, then clamp initial into it.
    if (c.scoreUpper < c.scoreLower)   fix(c.scoreUpper, c.scoreLower);
    const float clampedInit = std::max(c.scoreLower,
                                       std::min(c.scoreInitial, c.scoreUpper));
    if (clampedInit != c.scoreInitial) fix(c.scoreInitial, clampedInit);
    if (c.capDecayFloor > c.scoreUpper) fix(c.capDecayFloor, c.scoreUpper);

    // Level thresholds must be strictly ordered fatigue < warning < caution.
    if (!(c.warningScore > c.fatigueScore)) fix(c.warningScore, c.fatigueScore + 1.0f);
    if (!(c.cautionScore > c.warningScore)) fix(c.cautionScore, c.warningScore + 1.0f);

    if (changed && log)
        log(dashcam::log::LogLevel::WARN,
            "driver score config was inconsistent — clamped to safe values");
}

FatigueScorer::FatigueScorer(const FatigueScoreConfig& cfg, TimePoint now,
                             dashcam::log::LogCallback log)
    : cfg_(cfg) {
    sanitize(cfg_, log);   // never throws; a config typo must not disable us
    reset(now);
}

void FatigueScorer::reset(TimePoint now) {
    std::lock_guard<std::mutex> lk(mutex_);
    score_        = cfg_.scoreInitial;
    cap_          = cfg_.scoreUpper;
    sessionStart_ = now;
    drowsyRun_ = awakeRun_ = false;
    drowsyChunksPaid_ = awakeChunksPaid_ = 0;
    driftArmed_ = driftPaid_ = false;
    inFatigueZone_ = false;
    level_ = FatigueLevel::OK;
    updateLevel(now);   // scoreInitial may sit below a threshold by config
}

void FatigueScorer::applyCapDecay(TimePoint now) {
    const float hours = std::floor(
        std::chrono::duration<float>(now - sessionStart_).count() / 3600.0f);
    cap_ = std::max(cfg_.capDecayFloor,
                    cfg_.scoreUpper - cfg_.capDecayPerHour * hours);
}

void FatigueScorer::clampScore() {
    score_ = std::max(cfg_.scoreLower, std::min(score_, cap_));
}

void FatigueScorer::updateLevel(TimePoint now) {
    if (score_ <= cfg_.fatigueScore) {
        if (!inFatigueZone_) { inFatigueZone_ = true; fatigueZoneSince_ = now; }
        // The high-confidence alarm needs sustained presence in the zone; a
        // brief dip reports WARNING below.
        if (std::chrono::duration<float>(now - fatigueZoneSince_).count()
                >= cfg_.fatigueSustainSec) {
            level_ = FatigueLevel::FATIGUE;
            return;
        }
    } else {
        inFatigueZone_ = false;
    }
    if      (score_ < cfg_.warningScore) level_ = FatigueLevel::WARNING;
    else if (score_ < cfg_.cautionScore) level_ = FatigueLevel::CAUTION;
    else                                 level_ = FatigueLevel::OK;
}

void FatigueScorer::update(bool valid, bool faceDetected, bool drowsy,
                           TimePoint now) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!valid) return;
    applyCapDecay(now);

    const bool treatDrowsy = faceDetected && drowsy;
    const bool treatAwake  = faceDetected ? !drowsy : !cfg_.noFaceFreezes;

    if (treatDrowsy) {
        if (!drowsyRun_) {
            drowsyRun_        = true;
            drowsyStart_      = now;
            drowsyChunksPaid_ = 0;
            driftPaid_        = false;   // fresh episode: drift may deduct once
        }
        awakeRun_ = false;               // partial awake chunk discarded
        const int chunks = static_cast<int>(
            std::chrono::duration<float>(now - drowsyStart_).count()
            / cfg_.drowsyChunkSec);
        if (chunks > drowsyChunksPaid_) {
            score_ -= cfg_.drowsyChunkPenalty * (chunks - drowsyChunksPaid_);
            drowsyChunksPaid_ = chunks;
        }
    } else if (treatAwake) {
        drowsyRun_ = false;   // partial drowsy chunk discarded — recovering
                              // within the window costs nothing
        if (!awakeRun_ || score_ >= cap_) {
            // Start (or, while pinned at the cap, keep restarting) the awake
            // episode: no reward accrues at the cap, so holding the baseline at
            // `now` means a later deduction heals promptly instead of waiting
            // out phantom chunks that "elapsed" with nothing to add.
            awakeRun_        = true;
            awakeStart_      = now;
            awakeChunksPaid_ = 0;
        }
        const int chunks = static_cast<int>(
            std::chrono::duration<float>(now - awakeStart_).count()
            / cfg_.awakeChunkSec);
        if (chunks > awakeChunksPaid_) {
            score_ += cfg_.awakeChunkReward * (chunks - awakeChunksPaid_);
            awakeChunksPaid_ = chunks;
        }
    } else {
        // No-face freeze: a blocked/averted camera is not drowsiness — end
        // both episodes (partials discarded) and hold the score.
        drowsyRun_ = awakeRun_ = false;
    }

    clampScore();
    updateLevel(now);
}

void FatigueScorer::laneOffset(float offset, bool offsetValid, TimePoint now) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!offsetValid) {
        // Boundaries momentarily lost.  Do NOT disarm: a lane CROSSING — the
        // strongest drift signal — is exactly what destabilises detection, so
        // dropping the arm here would forfeit the very events we want.  Hold
        // the armed state; the laneReturnSec timeout (checked on the next valid
        // sample) still bounds it, and a later in-lane sample confirms return.
        return;
    }
    const bool outside = std::fabs(offset) >= cfg_.laneDepartThresh;
    if (!driftArmed_) {
        // A departure arms only while a drowsy episode is active: the
        // drowsiness onset is the trigger for the correlation window.  The
        // armed state deliberately survives the episode's end — the driver
        // waking mid-excursion and jerking back IS the signature.
        if (outside && drowsyRun_) {
            driftArmed_ = true;
            departTime_ = now;
        }
        return;
    }
    if (std::chrono::duration<float>(now - departTime_).count()
            > cfg_.laneReturnSec) {
        driftArmed_ = false;   // long excursion: deliberate lane change
        return;
    }
    if (!outside) {            // jerked back within the window
        driftArmed_ = false;
        if (!driftPaid_) {
            driftPaid_ = true;
            score_ -= cfg_.laneDriftPenalty;
            clampScore();
            updateLevel(now);
        }
    }
}

FatigueScorer::Snapshot FatigueScorer::snapshot() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return {score_, cap_, level_};
}
float FatigueScorer::score() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return score_;
}
float FatigueScorer::cap() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return cap_;
}
FatigueLevel FatigueScorer::level() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return level_;
}

// ─── PIMPL ───────────────────────────────────────────────────────────────────

class DriverStateImpl {
public:
    DriverStateConfig config_;
    dashcam::log::LogCallback log_;   // before logger_: TrtLogger points at it
    uint32_t          srcW_;
    uint32_t          srcH_;

    // Preprocess crop rectangle: the detected face box (square + margin), or
    // the constructor-set fallback framing.  Only the inference thread writes
    // it after construction.
    int roiX_ = 0, roiY_ = 0, roiW_ = 0, roiH_ = 0;

    // ── YuNet DNN face detection (inference-thread only) ─────────────────────
    cv::Ptr<cv::FaceDetectorYN> faceDet_;
    cv::Mat                     facesMat_;    // reused detection output, no alloc
    cv::Mat                     detectMat_;   // reused downscaled input (scale<1)
    int                         faceMinSidePx_ = 0;
    int                         detW_ = 0, detH_ = 0;   // YuNet input size

    // Last successful detection, for the faceHoldSec grace window.
    std::chrono::steady_clock::time_point lastFaceTime_{};
    bool                                  faceEverSeen_ = false;

    // Long-horizon fatigue score, ticked once per processed frame; also fed
    // by the app thread (setLaneOffset / resetScore) — internally locked.
    FatigueScorer scorer_;

    // Per-stage timing, logged at DEBUG every kTimingLogEvery frames so the
    // face-detect cost is visible when deciding the sustainable rate.
    static constexpr int kTimingLogEvery = 20;
    double detectMsAcc_ = 0.0;
    double inferMsAcc_  = 0.0;
    int    inferCount_  = 0;
    int    frameCount_  = 0;

    // ── TRT ──────────────────────────────────────────────────────────────────
    TrtLogger                    logger_;
    nvinfer1::IRuntime*          runtime_ = nullptr;
    nvinfer1::ICudaEngine*       engine_  = nullptr;
    nvinfer1::IExecutionContext* ctx_     = nullptr;
    std::string                  inputName_;
    std::string                  outputName_;

    // ── CUDA ─────────────────────────────────────────────────────────────────
    cudaStream_t stream_   = nullptr;
    void*        dSrcBGR_  = nullptr;
    void*        dInput_   = nullptr;
    void*        dOutput_  = nullptr;
    size_t       srcBytes_ = 0;
    size_t       inBytes_  = 0;
    float*       hOutput_  = nullptr;  // pinned, 1 logit

    // ── GStreamer ─────────────────────────────────────────────────────────────
    GstElement* appsink_ = nullptr;  // ref held via gst_bin_get_by_name

    // ── inference thread ──────────────────────────────────────────────────────
    std::thread       inferThread_;
    std::atomic<bool> stopFlag_{false};

    mutable std::mutex resultMutex_;
    DriverStateResult  latestResult_;
    std::atomic<uint64_t> processedFrames_{0};
    std::atomic<int64_t>  lastResultSteadyMs_{0};

    static std::atomic<int> sCounter;

    // ── construction / destruction ────────────────────────────────────────────

    DriverStateImpl(uint32_t srcW, uint32_t srcH, const DriverStateConfig& cfg)
        : config_(cfg), log_(cfg.log), srcW_(srcW), srcH_(srcH),
          scorer_(cfg.score, std::chrono::steady_clock::now(), cfg.log)
    {
        logger_.cb = &log_;

        if (config_.enginePath.empty())
            throw std::runtime_error("libdriverstate: enginePath is empty");
        if (srcW_ == 0 || srcH_ == 0)
            throw std::runtime_error("libdriverstate: source dimensions are 0");
        if (config_.modelInputW == 0 || config_.modelInputH == 0)
            throw std::runtime_error("libdriverstate: model dimensions are 0");
        if (config_.drowsyThreshold < 0.0f || config_.drowsyThreshold > 1.0f)
            throw std::runtime_error("libdriverstate: drowsyThreshold must be "
                                     "in [0, 1]");
        if (config_.faceMarginFrac < 0.0f || config_.faceMarginFrac > 1.0f)
            throw std::runtime_error("libdriverstate: faceMarginFrac must be "
                                     "in [0, 1]");
        if (config_.faceMinSizeFrac <= 0.0f || config_.faceMinSizeFrac > 1.0f)
            throw std::runtime_error("libdriverstate: faceMinSizeFrac must be "
                                     "in (0, 1]");
        if (config_.faceScoreThreshold < 0.0f || config_.faceScoreThreshold > 1.0f)
            throw std::runtime_error("libdriverstate: faceScoreThreshold must be "
                                     "in [0, 1]");
        if (config_.faceDetectScale < 0.25f || config_.faceDetectScale > 1.0f)
            throw std::runtime_error("libdriverstate: faceDetectScale must be "
                                     "in [0.25, 1]");
        if (config_.faceHoldSec < 0.0f)
            throw std::runtime_error("libdriverstate: faceHoldSec must be >= 0");

        if (config_.faceDetection) {
            // Run YuNet on a (possibly) downscaled frame; the box is mapped back
            // to full resolution for the crop, so classification is unaffected.
            detW_ = std::max(64, static_cast<int>(
                std::lround(srcW_ * config_.faceDetectScale)));
            detH_ = std::max(64, static_cast<int>(
                std::lround(srcH_ * config_.faceDetectScale)));
            try {
                faceDet_ = cv::FaceDetectorYN::create(
                    config_.faceModelPath, "",
                    cv::Size(detW_, detH_),
                    config_.faceScoreThreshold, /*nmsThreshold=*/0.3f,
                    /*topK=*/5000);
            } catch (const cv::Exception& e) {
                throw std::runtime_error("libdriverstate: cannot load YuNet face "
                    "model '" + config_.faceModelPath + "': " + e.what());
            }
            if (!faceDet_)
                throw std::runtime_error("libdriverstate: cannot load YuNet face "
                    "model: " + config_.faceModelPath);
            faceMinSidePx_ = std::max(
                24, static_cast<int>(srcH_ * config_.faceMinSizeFrac));
        }

        // Fallback framing for faceDetection == false; also the initial ROI.
        if (config_.centerCropSquare && srcW_ != srcH_) {
            const uint32_t side = std::min(srcW_, srcH_);
            roiX_ = static_cast<int>((srcW_ - side) / 2);
            roiY_ = static_cast<int>((srcH_ - side) / 2);
            roiW_ = roiH_ = static_cast<int>(side);
        } else {
            roiW_ = static_cast<int>(srcW_);
            roiH_ = static_cast<int>(srcH_);
        }

        try {
            loadEngine();
            allocBuffers();
            cudaCheck(cudaStreamCreate(&stream_), "cudaStreamCreate");
            bindTensors();
        } catch (...) {
            cleanup();
            throw;
        }

        doLog(log_, dashcam::log::LogLevel::INFO,
              "driver state detector ready: %ux%u tee -> %s -> model %ux%u, "
              "branch cap %u fps, infer cap %u Hz, threshold %.2f",
              srcW_, srcH_,
              config_.faceDetection ? "Viola-Jones face crop"
                                    : "fixed centre crop",
              config_.modelInputW, config_.modelInputH,
              config_.branchMaxFps, config_.targetHz, config_.drowsyThreshold);
    }

    ~DriverStateImpl() {
        stop();
        cleanup();
    }

    void cleanup() noexcept {
        cudaFree(dSrcBGR_);       dSrcBGR_ = nullptr;
        cudaFree(dInput_);        dInput_  = nullptr;
        cudaFree(dOutput_);       dOutput_ = nullptr;
        cudaFreeHost(hOutput_);   hOutput_ = nullptr;
        if (stream_)  { cudaStreamDestroy(stream_); stream_  = nullptr; }
        if (appsink_) { gst_object_unref(appsink_); appsink_ = nullptr; }
        delete ctx_;     ctx_     = nullptr;
        delete engine_;  engine_  = nullptr;
        delete runtime_; runtime_ = nullptr;
    }

    // ── engine loading ────────────────────────────────────────────────────────

    static std::string dimsStr(const nvinfer1::Dims& d) {
        std::string s = "(";
        for (int i = 0; i < d.nbDims; ++i)
            s += (i ? "," : "") + std::to_string(d.d[i]);
        return s + ")";
    }

    // Streaming file reader for deserializeCudaEngine — same pattern as
    // liblanedetector, avoids double-buffering the serialized engine in host
    // RAM during deserialization.
    class FileStreamReader : public nvinfer1::IStreamReader {
    public:
        explicit FileStreamReader(const std::string& path)
            : f_(path, std::ios::binary) {}
        bool ok() const { return f_.is_open(); }
        int64_t read(void* dst, int64_t nbBytes) noexcept override {
            f_.read(static_cast<char*>(dst), nbBytes);
            return f_.gcount();
        }
    private:
        std::ifstream f_;
    };

    void loadEngine() {
        FileStreamReader reader(config_.enginePath);
        if (!reader.ok())
            throw std::runtime_error("libdriverstate: cannot open engine: "
                                     + config_.enginePath);

        runtime_ = nvinfer1::createInferRuntime(logger_);
        if (!runtime_)
            throw std::runtime_error("libdriverstate: createInferRuntime failed");

        engine_ = runtime_->deserializeCudaEngine(reader);
        if (!engine_)
            throw std::runtime_error("libdriverstate: deserializeCudaEngine "
                                     "failed — common causes: TRT-version "
                                     "mismatch (rebuild with trtexec inside the "
                                     "runtime container) or CUDA out-of-memory "
                                     "(see [TRT/driver] log lines)");

        ctx_ = engine_->createExecutionContext();
        if (!ctx_)
            throw std::runtime_error("libdriverstate: createExecutionContext failed");

        const int32_t n = engine_->getNbIOTensors();
        for (int32_t i = 0; i < n; ++i) {
            const char* name = engine_->getIOTensorName(i);
            const auto  mode = engine_->getTensorIOMode(name);
            if (mode == nvinfer1::TensorIOMode::kINPUT) {
                if (!inputName_.empty())
                    throw std::runtime_error("libdriverstate: engine has more "
                                             "than one input tensor");
                inputName_ = name;
            } else if (mode == nvinfer1::TensorIOMode::kOUTPUT) {
                if (!outputName_.empty())
                    throw std::runtime_error("libdriverstate: engine has more "
                                             "than one output tensor");
                outputName_ = name;
            }
        }
        if (inputName_.empty() || outputName_.empty())
            throw std::runtime_error("libdriverstate: engine must have exactly "
                                     "one input and one output tensor");

        // Input: (N,3,H,W) float32, N either fixed 1 or dynamic (-1) — the
        // ONNX export used dynamic_axes on the batch.
        const auto in = engine_->getTensorShape(inputName_.c_str());
        const bool inOk = in.nbDims == 4
            && (in.d[0] == 1 || in.d[0] == -1)
            && in.d[1] == 3
            && in.d[2] == static_cast<int64_t>(config_.modelInputH)
            && in.d[3] == static_cast<int64_t>(config_.modelInputW);
        if (!inOk)
            throw std::runtime_error("libdriverstate: input tensor '"
                + inputName_ + "' has dims " + dimsStr(in)
                + ", expected (1|-1,3," + std::to_string(config_.modelInputH)
                + "," + std::to_string(config_.modelInputW)
                + ") — engine/config mismatch");
        if (engine_->getTensorDataType(inputName_.c_str())
                != nvinfer1::DataType::kFLOAT)
            throw std::runtime_error("libdriverstate: input tensor is not float32");

        // Dynamic batch: pin the runtime shape to batch 1 once, up front.
        if (in.d[0] == -1) {
            nvinfer1::Dims4 shape(1, 3, config_.modelInputH, config_.modelInputW);
            if (!ctx_->setInputShape(inputName_.c_str(), shape))
                throw std::runtime_error("libdriverstate: setInputShape(1,3,"
                    + std::to_string(config_.modelInputH) + ","
                    + std::to_string(config_.modelInputW) + ") failed");
        }

        // Output: ONE logit — (1,1), (-1,1) or (1,) depending on export.
        const auto out = engine_->getTensorShape(outputName_.c_str());
        int64_t volume = 1;
        for (int i = 0; i < out.nbDims; ++i)
            if (out.d[i] > 0) volume *= out.d[i];
        if (volume != 1)
            throw std::runtime_error("libdriverstate: output tensor '"
                + outputName_ + "' has dims " + dimsStr(out)
                + ", expected a single logit — this library is for the binary "
                  "Drowsy/Natural BCEWithLogits model");
        if (engine_->getTensorDataType(outputName_.c_str())
                != nvinfer1::DataType::kFLOAT)
            throw std::runtime_error("libdriverstate: output tensor is not float32");
    }

    // ── buffer allocation / binding ───────────────────────────────────────────

    void allocBuffers() {
        srcBytes_ = static_cast<size_t>(srcW_) * srcH_ * 3;
        inBytes_  = static_cast<size_t>(config_.modelInputW)
                  * config_.modelInputH * 3 * sizeof(float);

        cudaCheck(cudaMalloc(&dSrcBGR_, srcBytes_),      "cudaMalloc dSrcBGR");
        cudaCheck(cudaMalloc(&dInput_,  inBytes_),       "cudaMalloc dInput");
        cudaCheck(cudaMalloc(&dOutput_, sizeof(float)),  "cudaMalloc dOutput");
        cudaCheck(cudaMallocHost(reinterpret_cast<void**>(&hOutput_),
                                 sizeof(float)),         "cudaMallocHost hOutput");
    }

    // Tensor addresses never change — bind once at construction.
    void bindTensors() {
        if (!ctx_->setTensorAddress(inputName_.c_str(), dInput_))
            throw std::runtime_error("libdriverstate: setTensorAddress(input) failed");
        if (!ctx_->setTensorAddress(outputName_.c_str(), dOutput_))
            throw std::runtime_error("libdriverstate: setTensorAddress(output) failed");
    }

    // ── GStreamer bin ─────────────────────────────────────────────────────────

    std::string conversionChain() const {
        if (!config_.gstConversion.empty()) return config_.gstConversion;

        std::string chain;
        if (config_.branchMaxFps > 0)
            chain += "videorate drop-only=true max-rate="
                   + std::to_string(config_.branchMaxFps) + " ! ";
        if (config_.sourceIsNVMM)
            chain += "nvvidconv ! video/x-raw,format=BGRx ! videoconvert";
        else
            chain += "videoconvert";
        return chain;
    }

    GstElement* createBin() {
        if (appsink_) { gst_object_unref(appsink_); appsink_ = nullptr; }

        const std::string sinkName =
            "dstatesink_" + std::to_string(sCounter.fetch_add(1));

        // Width/height are pinned to the expected source dimensions: a
        // mismatched tee resolution must fail caps negotiation loudly rather
        // than let the fixed-size H2D copy read out of bounds.
        const std::string desc = conversionChain()
            + " ! video/x-raw,format=BGR,width=" + std::to_string(srcW_)
            + ",height=" + std::to_string(srcH_)
            + " ! appsink name=" + sinkName
            + " drop=true max-buffers=1 emit-signals=false sync=false";

        GError*     err = nullptr;
        GstElement* bin = gst_parse_bin_from_description(
            desc.c_str(), /*ghost_unlinked=*/TRUE, &err);
        if (err || !bin) {
            doLog(log_, dashcam::log::LogLevel::ERROR,
                  "driver state bin parse failed: %s", err ? err->message : "?");
            if (err) g_error_free(err);
            if (bin) gst_object_unref(bin);
            return nullptr;
        }

        appsink_ = gst_bin_get_by_name(GST_BIN(bin), sinkName.c_str());
        if (!appsink_) {
            doLog(log_, dashcam::log::LogLevel::ERROR,
                  "driver state appsink '%s' not found", sinkName.c_str());
            gst_object_unref(bin);
            return nullptr;
        }

        doLog(log_, dashcam::log::LogLevel::DEBUG,
              "driver state bin: %s", desc.c_str());
        return bin;
    }

    // ── YuNet DNN face crop ───────────────────────────────────────────────────

    // Detect the largest face in the mapped BGR frame and set the preprocess
    // ROI to a margin-expanded SQUARE around it (the classifier was trained
    // on square face crops).  Returns false when no acceptable face is found.
    bool detectFaceRoi(const uint8_t* bgr) {
        const cv::Mat frame(static_cast<int>(srcH_), static_cast<int>(srcW_),
                            CV_8UC3,
                            const_cast<uint8_t*>(bgr));   // wraps, no copy

        // Detect on the (possibly downscaled) frame; YuNet cost scales with the
        // input area, so faceDetectScale<1 trades detection resolution for CPU.
        const cv::Mat* det = &frame;
        if (detW_ != static_cast<int>(srcW_) || detH_ != static_cast<int>(srcH_)) {
            cv::resize(frame, detectMat_, cv::Size(detW_, detH_), 0, 0,
                       cv::INTER_AREA);
            det = &detectMat_;
        }
        faceDet_->detect(*det, facesMat_);
        if (facesMat_.empty() || facesMat_.rows == 0) return false;

        // Map detector-space boxes back to full resolution.
        const float sx = static_cast<float>(srcW_) / static_cast<float>(detW_);
        const float sy = static_cast<float>(srcH_) / static_cast<float>(detH_);

        // Each YuNet row is [x, y, w, h, 5×(landmark x,y), score] (CV_32F).
        // Pick the largest box that clears the min-size filter (the driver is
        // the nearest, largest face; small boxes are cabin clutter / passengers).
        int   bestRow  = -1;
        float bestArea = -1.0f;
        for (int i = 0; i < facesMat_.rows; ++i) {
            const float* r = facesMat_.ptr<float>(i);
            const float w = r[2] * sx, h = r[3] * sy;   // full-res dimensions
            if (std::max(w, h) < static_cast<float>(faceMinSidePx_)) continue;
            const float area = w * h;
            if (area > bestArea) { bestArea = area; bestRow = i; }
        }
        if (bestRow < 0) return false;

        const float* r = facesMat_.ptr<float>(bestRow);
        const int bx = static_cast<int>(r[0] * sx), by = static_cast<int>(r[1] * sy);
        const int bw = static_cast<int>(r[2] * sx), bh = static_cast<int>(r[3] * sy);

        // Square side with margin, capped at the frame's short side.
        const int frameMin = static_cast<int>(std::min(srcW_, srcH_));
        int side = static_cast<int>(
            std::max(bw, bh) * (1.0f + 2.0f * config_.faceMarginFrac));
        side = std::min(side, frameMin);

        // Centre on the face box, clamp fully inside the frame (YuNet boxes can
        // extend past the border when the face fills the frame).
        const int cx = bx + bw / 2;
        const int cy = by + bh / 2;
        roiX_ = std::max(0, std::min(cx - side / 2,
                                     static_cast<int>(srcW_) - side));
        roiY_ = std::max(0, std::min(cy - side / 2,
                                     static_cast<int>(srcH_) - side));
        roiW_ = roiH_ = side;
        return true;
    }

    // ── inference thread ──────────────────────────────────────────────────────

    void inferenceLoop() {
        using Clock = std::chrono::steady_clock;

        // Allow the first frame to run immediately.  The 5% tolerance keeps
        // this gate from beating against the branch's videorate cap when both
        // are set to the same rate — without it, timestamp jitter makes the
        // gate reject every other frame and the effective rate halves.
        auto lastInfer = Clock::now() - std::chrono::hours(1);
        const double minInterval = config_.targetHz > 0
            ? 0.95 / config_.targetHz
            : 0.0;
        bool sizeWarned = false;
        bool procWarned = false;

        while (!stopFlag_.load(std::memory_order_relaxed)) {
            if (!appsink_) break;

            GstSample* sample = gst_app_sink_try_pull_sample(
                GST_APP_SINK(appsink_), 100 * GST_MSECOND);
            if (!sample) continue;

            // Time-gate: drop the frame if the next inference window hasn't opened.
            const auto now = Clock::now();
            if (minInterval > 0.0 &&
                std::chrono::duration<double>(now - lastInfer).count() < minInterval) {
                gst_sample_unref(sample);
                continue;
            }
            lastInfer = now;

            GstBuffer* buf = gst_sample_get_buffer(sample);
            if (!buf) { gst_sample_unref(sample); continue; }

            GstMapInfo map;
            if (!gst_buffer_map(buf, &map, GST_MAP_READ)) {
                gst_sample_unref(sample);
                continue;
            }

            DriverStateResult result;
            const bool sizeOk = map.size == srcBytes_;
            // OpenCV (YuNet detect) can throw on a malformed frame; a throw out
            // of this thread would std::terminate the whole process (recording
            // and lanes too), so contain it here and drop just this frame.
            try {
            if (sizeOk) {
                bool haveFace = true;
                if (config_.faceDetection) {
                    const auto t0 = Clock::now();
                    haveFace = detectFaceRoi(map.data);
                    detectMsAcc_ += std::chrono::duration<double, std::milli>(
                        Clock::now() - t0).count();
                    if (haveFace) {
                        lastFaceTime_ = now;
                        faceEverSeen_ = true;
                    } else if (faceEverSeen_ &&
                               std::chrono::duration<double>(
                                   now - lastFaceTime_).count()
                                   <= config_.faceHoldSec) {
                        // Detector dropout (a hard head-droop / heavy
                        // occlusion can still momentarily defeat YuNet): keep
                        // classifying the last-known face box for the grace
                        // window.
                        haveFace = true;
                    }
                }
                if (haveFace) {
                    const auto t0 = Clock::now();
                    result = runInference(map.data);
                    inferMsAcc_ += std::chrono::duration<double, std::milli>(
                        Clock::now() - t0).count();
                    ++inferCount_;
                } else {
                    // No face: classification is meaningless — report that
                    // explicitly instead of classifying cabin background.
                    result.valid        = true;
                    result.faceDetected = false;
                }
                // Fatigue score: one tick per processed frame (invalid
                // results — e.g. CUDA failures — are ignored by the scorer).
                scorer_.update(result.valid, result.faceDetected,
                               result.state == DriverState::DROWSY, now);
                const FatigueScorer::Snapshot fs = scorer_.snapshot();
                result.fatigueScore = fs.score;
                result.fatigueCap   = fs.cap;
                result.fatigueLevel = fs.level;
                if (++frameCount_ % kTimingLogEvery == 0) {
                    doLog(log_, dashcam::log::LogLevel::DEBUG,
                          "driver state timing over %d frames: face detect "
                          "avg %.1f ms, classify avg %.1f ms (%d classified)",
                          kTimingLogEvery, detectMsAcc_ / kTimingLogEvery,
                          inferCount_ ? inferMsAcc_ / inferCount_ : 0.0,
                          inferCount_);
                    detectMsAcc_ = inferMsAcc_ = 0.0;
                    inferCount_  = 0;
                }
            } else if (!sizeWarned) {
                sizeWarned = true;
                doLog(log_, dashcam::log::LogLevel::ERROR,
                      "driver state frame size %zu != expected %zu (%ux%ux3) — "
                      "frames skipped", map.size, srcBytes_, srcW_, srcH_);
            }
            } catch (const std::exception& e) {
                if (!procWarned) {
                    procWarned = true;
                    doLog(log_, dashcam::log::LogLevel::ERROR,
                          "driver state frame processing threw (%s) — frame "
                          "dropped; further such errors suppressed", e.what());
                }
                result.valid = false;   // do not publish a half-built result
            }

            gst_buffer_unmap(buf, &map);
            gst_sample_unref(sample);

            if (sizeOk && result.valid) {
                result.sequence =
                    processedFrames_.fetch_add(1, std::memory_order_relaxed) + 1;
                const int64_t nowMs =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        Clock::now().time_since_epoch()).count();
                std::lock_guard<std::mutex> lk(resultMutex_);
                latestResult_ = result;
                lastResultSteadyMs_.store(nowMs, std::memory_order_release);
            }
        }
    }

    // ── inference pipeline ────────────────────────────────────────────────────

    DriverStateResult runInference(const uint8_t* bgr) {
        if (cudaMemcpyAsync(dSrcBGR_, bgr, srcBytes_,
                            cudaMemcpyHostToDevice, stream_) != cudaSuccess) {
            doLog(log_, dashcam::log::LogLevel::ERROR,
                  "driver state H2D memcpy failed");
            return {};
        }

        // From here on the stream may reference the caller's mapped frame
        // (H2D in flight): every exit must synchronize first so the caller
        // can safely unmap.
        bool ok = true;

        const cudaError_t launchErr = launchPreprocessKernel(
            static_cast<const uint8_t*>(dSrcBGR_),
            static_cast<float*>(dInput_),
            static_cast<int>(srcW_), static_cast<int>(srcH_),
            roiX_, roiY_, roiW_, roiH_,
            static_cast<int>(config_.modelInputW),
            static_cast<int>(config_.modelInputH),
            config_.meanR, config_.meanG, config_.meanB,
            config_.stdR,  config_.stdG,  config_.stdB,
            stream_);
        if (launchErr != cudaSuccess) {
            doLog(log_, dashcam::log::LogLevel::ERROR,
                  "driver state preprocess launch failed: %s",
                  cudaGetErrorString(launchErr));
            ok = false;
        }

        if (ok && !ctx_->enqueueV3(stream_)) {
            doLog(log_, dashcam::log::LogLevel::ERROR,
                  "driver state enqueueV3 failed");
            ok = false;
        }

        if (ok && cudaMemcpyAsync(hOutput_, dOutput_, sizeof(float),
                                  cudaMemcpyDeviceToHost, stream_) != cudaSuccess) {
            doLog(log_, dashcam::log::LogLevel::ERROR,
                  "driver state D2H memcpy failed");
            ok = false;
        }

        if (cudaStreamSynchronize(stream_) != cudaSuccess) {
            doLog(log_, dashcam::log::LogLevel::ERROR,
                  "driver state stream sync failed");
            return {};
        }
        if (!ok) return {};

        return decode(*hOutput_);
    }

    // ── postprocessing ────────────────────────────────────────────────────────
    // The engine outputs one raw logit (BCEWithLogitsLoss training — no
    // sigmoid in the graph).  sigmoid(logit) = P(positive class); the
    // positiveIsDrowsy flag maps that onto P(drowsy).

    DriverStateResult decode(float logit) const {
        float p = 1.0f / (1.0f + std::exp(-logit));
        if (!config_.positiveIsDrowsy) p = 1.0f - p;

        DriverStateResult r;
        r.drowsyProbability = p;
        r.state = p >= config_.drowsyThreshold ? DriverState::DROWSY
                                               : DriverState::NATURAL;
        r.faceDetected = true;   // classification only runs on face frames
        r.valid        = true;
        return r;
    }

    // ── thread control ────────────────────────────────────────────────────────

    void start() {
        if (inferThread_.joinable()) return;   // already running
        stopFlag_.store(false, std::memory_order_relaxed);
        inferThread_ = std::thread(&DriverStateImpl::inferenceLoop, this);
    }

    void stop() {
        stopFlag_.store(true, std::memory_order_release);
        if (inferThread_.joinable()) inferThread_.join();
    }

    DriverStateResult poll() const {
        DriverStateResult r;
        {
            std::lock_guard<std::mutex> lk(resultMutex_);
            r = latestResult_;
        }
        // Overlay the LIVE score: resetScore()/setLaneOffset() mutate the
        // scorer from the caller's thread between inference ticks, so the
        // snapshot embedded in latestResult_ can be up to a branch period
        // stale.  The scorer is independently locked.
        if (r.valid) {
            const FatigueScorer::Snapshot fs = scorer_.snapshot();
            r.fatigueScore = fs.score;
            r.fatigueCap   = fs.cap;
            r.fatigueLevel = fs.level;
        }
        return r;
    }

    uint64_t processedFrameCount() const {
        return processedFrames_.load(std::memory_order_acquire);
    }

    bool hasFreshResult(uint32_t maxAgeMs) const {
        const int64_t last =
            lastResultSteadyMs_.load(std::memory_order_acquire);
        if (last <= 0) return false;
        const int64_t now =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
        return now >= last && static_cast<uint64_t>(now - last) <= maxAgeMs;
    }
};

std::atomic<int> DriverStateImpl::sCounter{0};

// ─── DriverStateDetector public API ──────────────────────────────────────────

DriverStateDetector::DriverStateDetector(uint32_t srcWidth, uint32_t srcHeight,
                                         const DriverStateConfig& config)
    : impl_(std::make_unique<DriverStateImpl>(srcWidth, srcHeight, config))
{}

DriverStateDetector::~DriverStateDetector() = default;

GstElement*       DriverStateDetector::createBin()  { return impl_->createBin(); }
void              DriverStateDetector::start()      { impl_->start(); }
void              DriverStateDetector::stop()       { impl_->stop(); }
DriverStateResult DriverStateDetector::poll() const { return impl_->poll(); }
uint64_t DriverStateDetector::processedFrameCount() const {
    return impl_->processedFrameCount();
}
bool DriverStateDetector::hasFreshResult(uint32_t maxAgeMs) const {
    return impl_->hasFreshResult(maxAgeMs);
}

void DriverStateDetector::setLaneOffset(float offset, bool offsetValid) {
    impl_->scorer_.laneOffset(offset, offsetValid,
                              std::chrono::steady_clock::now());
}

void DriverStateDetector::resetScore() {
    impl_->scorer_.reset(std::chrono::steady_clock::now());
}

void DriverStateDetector::setLogCallback(dashcam::log::LogCallback cb) {
    impl_->log_ = std::move(cb);
}

} // namespace dashcam::driver
