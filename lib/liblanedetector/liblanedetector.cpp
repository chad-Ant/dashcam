#include "liblanedetector.h"
#include "lane_geometry.h"
#include "ufld_decode.h"
#include "liblanedetector_preprocess.h"

#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <cuda_runtime.h>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <mutex>
#include <pthread.h>
#include <stdexcept>
#include <thread>

namespace dashcam::lane {

// ─── file-local log helper (same pattern as librecord) ───────────────────────

static void doLog(const dashcam::log::LogCallback& cb, dashcam::log::LogLevel lvl,
                  const char* fmt, ...) {
    if (!cb) return;
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    // A throwing log sink must not take the inference thread (and with it
    // the process, via std::terminate) down — plan bug B19.
    try {
        cb(lvl, buf);
    } catch (...) {
        std::fprintf(stderr, "liblanedetector: log callback threw; message was: %s\n", buf);
    }
}

// CLOCK_MONOTONIC ns (libstdc++ steady_clock), the clock LaneTiming uses.
static int64_t monoNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

static void cudaCheck(cudaError_t err, const char* what) {
    if (err != cudaSuccess)
        throw std::runtime_error(
            std::string("liblanedetector: ") + what
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
        try {
            if (cb && *cb) { (*cb)(lvl, std::string("[TRT/lane] ") + msg); return; }
        } catch (...) {}   // noexcept: a throwing sink falls back to stderr
        std::fprintf(stderr, "[TRT/lane] %s\n", msg);
    }
};

// ─── PIMPL ───────────────────────────────────────────────────────────────────

class LaneDetectorImpl {
public:
    LaneDetectorConfig config_;
    dashcam::log::LogCallback log_;   // before logger_: TrtLogger points at it
    uint32_t           srcW_;
    uint32_t           srcH_;         // branch-cropped height (buffer/caps size)
    uint32_t           cropTopPx_    = 0;
    uint32_t           cropBottomPx_ = 0;

    // ── TRT ──────────────────────────────────────────────────────────────────
    TrtLogger                    logger_;
    nvinfer1::IRuntime*          runtime_ = nullptr;
    nvinfer1::ICudaEngine*       engine_  = nullptr;
    nvinfer1::IExecutionContext* ctx_     = nullptr;
    std::string                  inputName_;

    // UFLD v2 output tensors, fixed order.
    enum OutIdx { kLocRow = 0, kLocCol, kExistRow, kExistCol, kNumOuts };
    static constexpr const char* kOutNames[kNumOuts] = {
        "loc_row", "loc_col", "exist_row", "exist_col" };

    // ── CUDA ─────────────────────────────────────────────────────────────────
    cudaStream_t stream_          = nullptr;
    void*        dSrcBGR_         = nullptr;
    void*        dInput_          = nullptr;
    void*        dOut_[kNumOuts]  = {};
    float*       hOut_[kNumOuts]  = {};  // pinned
    size_t       srcBytes_        = 0;
    size_t       inputBytes_      = 0;
    size_t       outBytes_[kNumOuts] = {};

    // Preprocess vertical ROI (training transform: resize H/cropRatio, keep
    // bottom modelInputH rows) expressed in cropped-frame source pixels.
    float preYOff_   = 0.0f;
    float preYScale_ = 1.0f;

    // The above plus the implied-frame mapping, for callers (geometry()).
    LaneGeometry geom_;

    // ── GStreamer ─────────────────────────────────────────────────────────────
    GstElement* appsink_ = nullptr;  // ref held via gst_bin_get_by_name

    // ── inference thread ──────────────────────────────────────────────────────
    std::thread       inferThread_;
    std::atomic<bool> stopFlag_{false};

    mutable std::mutex resultMutex_;
    LaneResult         latestResult_;
    std::atomic<uint64_t> processedFrames_{0};
    std::atomic<int64_t>  lastResultSteadyMs_{0};

    static std::atomic<int> sCounter;

    // ── instrumentation ───────────────────────────────────────────────────────
    struct Counters {
        std::atomic<uint64_t> pulled{0}, gated{0}, mapFail{0}, sizeMismatch{0},
                              h2dFail{0}, preFail{0}, trtFail{0}, d2hFail{0},
                              syncFail{0}, nonFinite{0};
        std::atomic<uint64_t> processed{0};   // lifetime (processedFrames_ restarts at start())
        std::atomic<int64_t>  lastErrorMonoNs{-1};
    } stats_;

    // Per-site ERROR log limiter (plan bug B14): kErrBurst lines per
    // kErrWindowNs, then one summary of what was not logged.  Used only by
    // the inference thread / processFrame (never concurrently).
    struct ErrLimiter {
        const char* what;
        int64_t     windowNs   = 0;
        int         shown      = 0;
        uint64_t    suppressed = 0;
    };
    static constexpr int     kErrBurst    = 3;
    static constexpr int64_t kErrWindowNs = 10'000'000'000;
    ErrLimiter limH2d_{"lane H2D memcpy failed"};
    ErrLimiter limPre_{"lane preprocess launch failed"};
    ErrLimiter limTrt_{"lane enqueueV3 failed"};
    ErrLimiter limD2h_{"lane D2H memcpy failed"};
    ErrLimiter limSync_{"lane stream sync failed"};

    void flushLimiter(ErrLimiter& l) {
        if (l.suppressed == 0) return;
        doLog(log_, dashcam::log::LogLevel::ERROR, "%s: %llu more in the last %d s (not logged)",
              l.what, static_cast<unsigned long long>(l.suppressed),
              static_cast<int>(kErrWindowNs / 1'000'000'000));
        l.suppressed = 0;
    }

    void logLimited(ErrLimiter& l, const char* detail = nullptr) {
        const int64_t now = monoNs();
        if (now - l.windowNs >= kErrWindowNs) {
            flushLimiter(l);
            l.windowNs = now;
            l.shown    = 0;
        }
        if (l.shown >= kErrBurst) { ++l.suppressed; return; }
        ++l.shown;
        if (detail) doLog(log_, dashcam::log::LogLevel::ERROR, "%s: %s", l.what, detail);
        else        doLog(log_, dashcam::log::LogLevel::ERROR, "%s", l.what);
    }

    // Plan bug B15: a BGR frame whose rows are padded (width·3 not a multiple
    // of 4) is larger than W·H·3.  Packs its rows (stride from the buffer's
    // GstVideoMeta, else from the caps) into packed_; nullptr if the frame is
    // not a W×H BGR frame at all.
    std::vector<uint8_t> packed_;

    const uint8_t* packRows(GstSample* sample, GstBuffer* buf, const GstMapInfo& map) {
        size_t offset = 0;
        int    stride = 0;
        if (const GstVideoMeta* vm = gst_buffer_get_video_meta(buf)) {
            if (vm->format != GST_VIDEO_FORMAT_BGR || vm->width != srcW_ || vm->height != srcH_)
                return nullptr;
            offset = vm->offset[0];
            stride = vm->stride[0];
        } else {
            GstVideoInfo info;
            GstCaps* caps = gst_sample_get_caps(sample);
            if (!caps || !gst_video_info_from_caps(&info, caps)) return nullptr;
            if (GST_VIDEO_INFO_FORMAT(&info) != GST_VIDEO_FORMAT_BGR
                || GST_VIDEO_INFO_WIDTH(&info)  != static_cast<int>(srcW_)
                || GST_VIDEO_INFO_HEIGHT(&info) != static_cast<int>(srcH_))
                return nullptr;
            offset = GST_VIDEO_INFO_PLANE_OFFSET(&info, 0);
            stride = GST_VIDEO_INFO_PLANE_STRIDE(&info, 0);
        }
        const size_t row = static_cast<size_t>(srcW_) * 3;
        if (stride < static_cast<int>(row)
            || offset + static_cast<size_t>(stride) * (srcH_ - 1) + row > map.size)
            return nullptr;
        packed_.resize(srcBytes_);
        for (uint32_t y = 0; y < srcH_; ++y)
            std::memcpy(&packed_[y * row], map.data + offset + static_cast<size_t>(y) * stride, row);
        return packed_.data();
    }

    std::function<void(const LaneResult&)> sink_;   // set before start() only

    // collectTimings: events around H2D | preprocess | enqueue | D2H.
    enum { kEvStart = 0, kEvH2D, kEvPre, kEvTrt, kEvD2H, kNumEvents };
    cudaEvent_t events_[kNumEvents] = {};

    // LaneDetectorTestHook::onFrame: raw mapped frames (tests only; set before start()).
    std::function<void(const uint8_t*, size_t)> testFrameCb_;

    // LaneDetectorTestHook fault injection: frames left to fail, per point.
    std::atomic<int> faultLeft_[5] = {};

    bool consumeFault(LaneDetectorTestHook::Fault f) {
        std::atomic<int>& left = faultLeft_[static_cast<int>(f)];
        int v = left.load(std::memory_order_relaxed);
        while (v > 0 && !left.compare_exchange_weak(v, v - 1)) {}
        return v > 0;
    }

    void noteError(std::atomic<uint64_t>& counter) {
        counter.fetch_add(1, std::memory_order_relaxed);
        stats_.lastErrorMonoNs.store(monoNs(), std::memory_order_relaxed);
    }

    // ── construction / destruction ────────────────────────────────────────────

    LaneDetectorImpl(uint32_t srcW, uint32_t srcH, const LaneDetectorConfig& cfg)
        : config_(cfg), log_(cfg.log), srcW_(srcW), srcH_(srcH)
    {
        logger_.cb = &log_;

        if (config_.enginePath.empty())
            throw std::runtime_error("liblanedetector: enginePath is empty");
        // Validation and crop / preprocess arithmetic (lane_geometry.cpp).
        geom_         = computeLaneGeometry(srcW, srcH, config_);
        cropTopPx_    = geom_.cropTopPx;
        cropBottomPx_ = geom_.cropBottomPx;
        srcH_         = geom_.branchH;
        preYScale_    = geom_.preYScale;
        preYOff_      = geom_.preYOff;

        try {
            loadEngine();
            allocBuffers();
            cudaCheck(cudaStreamCreate(&stream_), "cudaStreamCreate");
            bindTensors();
            if (config_.collectTimings)
                for (auto& ev : events_)
                    cudaCheck(cudaEventCreate(&ev), "cudaEventCreate");
        } catch (...) {
            cleanup();
            throw;
        }

        doLog(log_, dashcam::log::LogLevel::INFO,
              "lane detector ready: %ux%u tee -> crop top %u + bottom %u -> "
              "%ux%u branch, model %ux%u, branch cap %u fps, infer cap %u Hz",
              srcW_, srcH, cropTopPx_, cropBottomPx_, srcW_, srcH_,
              config_.modelInputW, config_.modelInputH,
              config_.branchMaxFps, config_.targetHz);
    }

    ~LaneDetectorImpl() {
        stop();
        cleanup();
    }

    void cleanup() noexcept {
        cudaFree(dSrcBGR_);       dSrcBGR_  = nullptr;
        cudaFree(dInput_);        dInput_   = nullptr;
        for (int i = 0; i < kNumOuts; ++i) {
            cudaFree(dOut_[i]);      dOut_[i] = nullptr;
            cudaFreeHost(hOut_[i]);  hOut_[i] = nullptr;
        }
        for (auto& ev : events_)
            if (ev) { cudaEventDestroy(ev); ev = nullptr; }
        if (stream_)  { cudaStreamDestroy(stream_);       stream_  = nullptr; }
        if (appsink_) { gst_object_unref(appsink_);       appsink_ = nullptr; }
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

    void expectDims(const char* name, const nvinfer1::Dims& got,
                    std::initializer_list<int64_t> want) const {
        bool ok = got.nbDims == static_cast<int32_t>(want.size());
        int  i  = 0;
        for (int64_t w : want) ok = ok && got.d[i++] == w;
        if (!ok) {
            std::string wanted = "(";
            i = 0;
            for (int64_t w : want) wanted += (i++ ? "," : "") + std::to_string(w);
            wanted += ")";
            throw std::runtime_error(std::string("liblanedetector: tensor '")
                + name + "' has dims " + dimsStr(got) + ", config expects "
                + wanted + " — engine/config mismatch");
        }
    }

    // Streaming file reader for deserializeCudaEngine: avoids holding the
    // whole serialized engine (~800 MB for this model) in host memory while
    // the device weights are allocated — on 8 GB unified RAM the buffered
    // path can double the peak and OOM under desktop memory pressure.
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
            throw std::runtime_error("liblanedetector: cannot open engine: "
                                     + config_.enginePath);

        runtime_ = nvinfer1::createInferRuntime(logger_);
        if (!runtime_)
            throw std::runtime_error("liblanedetector: createInferRuntime failed");

        engine_ = runtime_->deserializeCudaEngine(reader);
        if (!engine_)
            throw std::runtime_error("liblanedetector: deserializeCudaEngine failed "
                                     "— common causes: TRT-version mismatch (engines "
                                     "are locked to the builder version; rebuild with "
                                     "trtexec inside the runtime container) or CUDA "
                                     "out-of-memory (see [TRT/lane] log lines)");

        ctx_ = engine_->createExecutionContext();
        if (!ctx_)
            throw std::runtime_error("liblanedetector: createExecutionContext failed");

        // One float32 input (1,3,H,W).
        const int32_t n = engine_->getNbIOTensors();
        for (int32_t i = 0; i < n; ++i) {
            const char* name = engine_->getIOTensorName(i);
            if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
                if (!inputName_.empty())
                    throw std::runtime_error("liblanedetector: engine has more "
                                             "than one input tensor");
                inputName_ = name;
            }
        }
        if (inputName_.empty())
            throw std::runtime_error("liblanedetector: engine has no input tensor");
        expectDims(inputName_.c_str(), engine_->getTensorShape(inputName_.c_str()),
                   { 1, 3, config_.modelInputH, config_.modelInputW });
        // The preprocess writes float32 CHW: an fp16 or vectorised input
        // binding would read that buffer as something else (plan bug B10).
        if (engine_->getTensorDataType(inputName_.c_str()) != nvinfer1::DataType::kFLOAT)
            throw std::runtime_error("liblanedetector: input '" + inputName_ + "' is not float32");
        if (engine_->getTensorFormat(inputName_.c_str()) != nvinfer1::TensorFormat::kLINEAR)
            throw std::runtime_error("liblanedetector: input '" + inputName_ + "' is not linear CHW");

        // The four UFLD v2 heads, matched by name and validated by shape.
        const int64_t L  = config_.numLanes;
        const int64_t R  = config_.numRowAnchors;
        const int64_t C  = config_.numColAnchors;
        const std::initializer_list<int64_t> want[kNumOuts] = {
            { 1, config_.numCellRow, R, L },   // loc_row
            { 1, config_.numCellCol, C, L },   // loc_col
            { 1, 2,                  R, L },   // exist_row
            { 1, 2,                  C, L },   // exist_col
        };
        for (int o = 0; o < kNumOuts; ++o) {
            const char* name = kOutNames[o];
            bool found = false;
            for (int32_t i = 0; i < n && !found; ++i)
                found = std::string(engine_->getIOTensorName(i)) == name;
            if (!found ||
                engine_->getTensorIOMode(name) != nvinfer1::TensorIOMode::kOUTPUT)
                throw std::runtime_error(std::string("liblanedetector: engine has "
                    "no output tensor '") + name + "' — not a UFLD v2 engine?");
            if (engine_->getTensorDataType(name) != nvinfer1::DataType::kFLOAT)
                throw std::runtime_error(std::string("liblanedetector: output '")
                    + name + "' is not float32");
            if (engine_->getTensorFormat(name) != nvinfer1::TensorFormat::kLINEAR)
                throw std::runtime_error(std::string("liblanedetector: output '")
                    + name + "' is not linear");
            expectDims(name, engine_->getTensorShape(name), want[o]);
        }
    }

    // ── buffer allocation / binding ───────────────────────────────────────────

    void allocBuffers() {
        srcBytes_   = static_cast<size_t>(srcW_) * srcH_ * 3;
        inputBytes_ = static_cast<size_t>(config_.modelInputW)
                    * config_.modelInputH * 3 * sizeof(float);

        const size_t L = static_cast<size_t>(config_.numLanes);
        outBytes_[kLocRow]   = static_cast<size_t>(config_.numCellRow)
                             * config_.numRowAnchors * L * sizeof(float);
        outBytes_[kLocCol]   = static_cast<size_t>(config_.numCellCol)
                             * config_.numColAnchors * L * sizeof(float);
        outBytes_[kExistRow] = 2u * config_.numRowAnchors * L * sizeof(float);
        outBytes_[kExistCol] = 2u * config_.numColAnchors * L * sizeof(float);

        cudaCheck(cudaMalloc(&dSrcBGR_, srcBytes_),   "cudaMalloc dSrcBGR");
        cudaCheck(cudaMalloc(&dInput_,  inputBytes_), "cudaMalloc dInput");
        for (int i = 0; i < kNumOuts; ++i) {
            cudaCheck(cudaMalloc(&dOut_[i], outBytes_[i]), "cudaMalloc dOut");
            cudaCheck(cudaMallocHost(reinterpret_cast<void**>(&hOut_[i]),
                                     outBytes_[i]),        "cudaMallocHost hOut");
        }
    }

    // Tensor addresses never change — bind once at construction.
    void bindTensors() {
        if (!ctx_->setTensorAddress(inputName_.c_str(), dInput_))
            throw std::runtime_error("liblanedetector: setTensorAddress(input) failed");
        for (int i = 0; i < kNumOuts; ++i)
            if (!ctx_->setTensorAddress(kOutNames[i], dOut_[i]))
                throw std::runtime_error(std::string("liblanedetector: "
                    "setTensorAddress(") + kOutNames[i] + ") failed");
    }

    // ── GStreamer bin ─────────────────────────────────────────────────────────

    std::string conversionChain() const {
        if (!config_.gstConversion.empty()) return config_.gstConversion;

        // Pin the FULL tee size first: the crop below is a fixed rectangle,
        // so a different-size stream would otherwise be cropped to the right
        // band size from the wrong pixels and pass the appsink caps
        // silently (plan bug B7).  A mismatch now fails negotiation.
        // (An explicit capsfilter: a bare caps string cannot start a bin
        // description.)
        std::string chain = std::string("capsfilter caps=\"video/x-raw")
            + (config_.sourceIsNVMM ? "(memory:NVMM)" : "")
            + ",width=" + std::to_string(srcW_)
            + ",height=" + std::to_string(geom_.srcH) + "\" ! ";
        if (config_.branchMaxFps > 0)
            chain += "videorate drop-only=true max-rate="
                   + std::to_string(config_.branchMaxFps) + " ! ";
        if (config_.sourceIsNVMM) {
            // VIC does crop + NV12→BGRx in one pass.  nvvidconv top/bottom are
            // rectangle COORDINATES: keep rows [top, top + branchH).
            chain += "nvvidconv";
            if (cropTopPx_ > 0 || cropBottomPx_ > 0)
                chain += " top="    + std::to_string(cropTopPx_)
                       + " bottom=" + std::to_string(cropTopPx_ + srcH_)
                       + " left=0 right=" + std::to_string(srcW_);
            chain += " ! video/x-raw,format=BGRx ! videoconvert";
        } else {
            // videocrop top/bottom are AMOUNTS removed from each edge.
            if (cropTopPx_ > 0 || cropBottomPx_ > 0)
                chain += "videocrop top=" + std::to_string(cropTopPx_)
                       + " bottom=" + std::to_string(cropBottomPx_) + " ! ";
            chain += "videoconvert";
        }
        return chain;
    }

    GstElement* createBin() {
        // The inference thread pulls from appsink_: swapping it under the
        // thread is a race / use-after-free (plan bug B1).
        if (inferThread_.joinable()) {
            doLog(log_, dashcam::log::LogLevel::ERROR,
                  "lane createBin refused: inference thread is running — stop() it first");
            return nullptr;
        }
        if (appsink_) { gst_object_unref(appsink_); appsink_ = nullptr; }

        const std::string sinkName =
            "lanesink_" + std::to_string(sCounter.fetch_add(1));

        // Width/height are pinned to the cropped branch dimensions: a
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
                  "lane bin parse failed: %s", err ? err->message : "?");
            if (err) g_error_free(err);
            if (bin) gst_object_unref(bin);
            return nullptr;
        }

        appsink_ = gst_bin_get_by_name(GST_BIN(bin), sinkName.c_str());
        if (!appsink_) {
            doLog(log_, dashcam::log::LogLevel::ERROR,
                  "lane appsink '%s' not found", sinkName.c_str());
            gst_object_unref(bin);
            return nullptr;
        }

        doLog(log_, dashcam::log::LogLevel::DEBUG,
              "lane bin: %s", desc.c_str());
        return bin;
    }

    // ── inference thread ──────────────────────────────────────────────────────

    // Capture time of a sample on CLOCK_MONOTONIC: base time + running time
    // of its PTS, valid only when the pipeline clock IS the monotonic system
    // clock (otherwise −1).
    int64_t captureMonoNs(GstSample* sample, GstBuffer* buf) const {
        const GstClockTime pts = GST_BUFFER_PTS(buf);
        const GstSegment* seg = gst_sample_get_segment(sample);
        if (!GST_CLOCK_TIME_IS_VALID(pts) || !seg || seg->format != GST_FORMAT_TIME)
            return -1;
        const GstClockTime rt = gst_segment_to_running_time(seg, GST_FORMAT_TIME, pts);
        if (!GST_CLOCK_TIME_IS_VALID(rt)) return -1;
        GstClock* clk = gst_element_get_clock(appsink_);
        if (!clk) return -1;
        bool mono = false;
        if (GST_IS_SYSTEM_CLOCK(clk)) {
            GstClockType type = GST_CLOCK_TYPE_REALTIME;
            g_object_get(clk, "clock-type", &type, nullptr);
            mono = type == GST_CLOCK_TYPE_MONOTONIC;
        }
        gst_object_unref(clk);
        if (!mono) return -1;
        return static_cast<int64_t>(gst_element_get_base_time(appsink_) + rt);
    }

    void inferenceLoop() {
        using Clock = std::chrono::steady_clock;

        if (!config_.threadName.empty())
            pthread_setname_np(pthread_self(), config_.threadName.substr(0, 15).c_str());

        // Allow the first frame to run immediately.
        auto lastInfer = Clock::now() - std::chrono::seconds(1);
        const double minInterval = config_.targetHz > 0
            ? 1.0 / config_.targetHz
            : 0.0;
        // RateGate::kPts state: the next PTS that may run, and the last
        // pulled PTS (frame-interval fallback when buffers carry no duration).
        const GstClockTime gateT = config_.targetHz > 0
            ? GST_SECOND / config_.targetHz : 0;
        GstClockTime nextPts = GST_CLOCK_TIME_NONE;
        GstClockTime lastPulledPts = GST_CLOCK_TIME_NONE;
        bool sizeWarned = false;

        while (!stopFlag_.load(std::memory_order_relaxed)) {
            if (!appsink_) break;

            GstSample* sample = gst_app_sink_try_pull_sample(
                GST_APP_SINK(appsink_), 100 * GST_MSECOND);
            if (!sample) {
                // Not started / flushing / EOS: try_pull returns at once, so
                // back off instead of spinning a core (plan bug B2).  A
                // plain timeout already waited its 100 ms.
                if (gst_app_sink_is_eos(GST_APP_SINK(appsink_)))
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }
            const int64_t pullNs = monoNs();
            stats_.pulled.fetch_add(1, std::memory_order_relaxed);

            GstBuffer* buf = gst_sample_get_buffer(sample);
            if (!buf) { gst_sample_unref(sample); continue; }
            const GstClockTime pts = GST_BUFFER_PTS(buf);

            // Rate gate: drop the frame if it is not yet time for the next
            // inference.
            bool gated = false;
            if (minInterval > 0.0) {
                if (config_.rateGate == LaneDetectorConfig::RateGate::kPts && GST_CLOCK_TIME_IS_VALID(pts)) {
                    // Decimate in PTS with half a frame interval of slack, so
                    // a 20 fps branch feeding a 20 Hz gate keeps every frame
                    // (the wall-clock gate drops whichever frame arrives a
                    // hair early — plan bug B3).
                    GstClockTime d = GST_BUFFER_DURATION(buf);
                    if (!GST_CLOCK_TIME_IS_VALID(d))
                        d = GST_CLOCK_TIME_IS_VALID(lastPulledPts) && pts > lastPulledPts
                          ? pts - lastPulledPts : 0;
                    lastPulledPts = pts;
                    if (!GST_CLOCK_TIME_IS_VALID(nextPts) || pts + gateT < nextPts
                        || pts > nextPts + gateT) {
                        nextPts = pts + gateT;          // first frame / jump: run now
                    } else if (pts + d / 2 > nextPts) {
                        nextPts += gateT;
                    } else {
                        gated = true;
                    }
                } else {
                    const auto now = Clock::now();
                    if (std::chrono::duration<double>(now - lastInfer).count() < minInterval)
                        gated = true;
                    else
                        lastInfer = now;
                }
            }
            if (gated) {
                stats_.gated.fetch_add(1, std::memory_order_relaxed);
                gst_sample_unref(sample);
                continue;
            }

            GstMapInfo map;
            if (!gst_buffer_map(buf, &map, GST_MAP_READ)) {
                noteError(stats_.mapFail);
                gst_sample_unref(sample);
                continue;
            }

            if (testFrameCb_) testFrameCb_(map.data, map.size);

            LaneResult result;
            const uint8_t* frame = map.size == srcBytes_ ? map.data : packRows(sample, buf, map);
            const bool sizeOk = frame != nullptr;
            if (sizeOk) {
                result = runInference(frame);
                result.timing.ptsNs = GST_CLOCK_TIME_IS_VALID(pts)
                                    ? static_cast<int64_t>(pts) : -1;
                result.timing.captureMonoNs = captureMonoNs(sample, buf);
                result.timing.pullMonoNs    = pullNs;
            } else {
                noteError(stats_.sizeMismatch);
                if (!sizeWarned) {
                    sizeWarned = true;
                    doLog(log_, dashcam::log::LogLevel::ERROR,
                          "lane frame size %zu != expected %zu (%ux%ux3) — "
                          "frames skipped", map.size, srcBytes_, srcW_, srcH_);
                }
            }

            gst_buffer_unmap(buf, &map);
            gst_sample_unref(sample);

            if (sizeOk && result.valid) publish(result);
        }
    }

    // Number a valid result, make it the one poll() returns, hand it to the sink.
    void publish(LaneResult& result) {
        const uint64_t sequence =
            processedFrames_.fetch_add(1, std::memory_order_relaxed) + 1;
        stats_.processed.fetch_add(1, std::memory_order_relaxed);
        result.sequence = sequence;
        const int64_t nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        {
            std::lock_guard<std::mutex> lk(resultMutex_);
            latestResult_ = result;
            lastResultSteadyMs_.store(nowMs, std::memory_order_release);
        }
        if (sink_) sink_(result);
    }

    // ── synchronous path (replay / tests) ─────────────────────────────────────

    bool processFrame(const uint8_t* bgr, size_t bytes, int64_t ptsNs,
                      LaneResult& out, LaneDebugTensors* dbg) {
        const int64_t entryNs = monoNs();
        out = LaneResult{};
        if (dbg) {
            dbg->input.clear();
            for (auto& o : dbg->outputs) o.clear();
        }
        if (inferThread_.joinable()) {
            doLog(log_, dashcam::log::LogLevel::ERROR,
                  "lane processFrame refused: inference thread is running");
            return false;
        }
        if (!bgr || bytes != srcBytes_) {
            noteError(stats_.sizeMismatch);
            doLog(log_, dashcam::log::LogLevel::ERROR,
                  "lane processFrame: %zu bytes != expected %zu (%ux%ux3)",
                  bytes, srcBytes_, srcW_, srcH_);
            return false;
        }

        LaneResult result = runInference(bgr);
        result.timing.ptsNs      = ptsNs;
        result.timing.pullMonoNs = entryNs;

        if (dbg && result.valid) {
            // runInference() completed and synchronised the stream; dInput_
            // and hOut_ hold this frame's tensors (not so on a failed frame).
            dbg->input.resize(inputBytes_ / sizeof(float));
            if (cudaMemcpy(dbg->input.data(), dInput_, inputBytes_,
                           cudaMemcpyDeviceToHost) != cudaSuccess) {
                doLog(log_, dashcam::log::LogLevel::ERROR,
                      "lane processFrame: input tensor copy failed");
                dbg->input.clear();
            }
            for (int i = 0; i < kNumOuts; ++i)
                dbg->outputs[static_cast<size_t>(i)].assign(
                    hOut_[i], hOut_[i] + outBytes_[i] / sizeof(float));
        }

        if (!result.valid) return false;
        publish(result);
        out = result;
        return true;
    }

    // ── inference pipeline ────────────────────────────────────────────────────

    LaneResult runInference(const uint8_t* bgr) {
        using Fault = LaneDetectorTestHook::Fault;
        const bool timed = config_.collectTimings && events_[kEvStart] != nullptr;
        if (timed) cudaEventRecord(events_[kEvStart], stream_);

        if (consumeFault(Fault::H2D) ||
            cudaMemcpyAsync(dSrcBGR_, bgr, srcBytes_,
                            cudaMemcpyHostToDevice, stream_) != cudaSuccess) {
            noteError(stats_.h2dFail);
            logLimited(limH2d_);
            return {};
        }
        if (timed) cudaEventRecord(events_[kEvH2D], stream_);

        // From here on the stream may reference the caller's mapped frame
        // (H2D in flight): every exit must synchronize first so the caller
        // can safely unmap.
        bool ok = true;

        const cudaError_t launchErr = consumeFault(Fault::Preprocess)
            ? cudaErrorLaunchFailure
            : launchPreprocessKernel(
                static_cast<const uint8_t*>(dSrcBGR_),
                static_cast<float*>(dInput_),
                static_cast<int>(srcW_), static_cast<int>(srcH_),
                static_cast<int>(config_.modelInputW),
                static_cast<int>(config_.modelInputH),
                preYOff_, preYScale_,
                config_.meanR, config_.meanG, config_.meanB,
                config_.stdR,  config_.stdG,  config_.stdB,
                stream_);
        if (launchErr != cudaSuccess) {
            noteError(stats_.preFail);
            logLimited(limPre_, cudaGetErrorString(launchErr));
            ok = false;
        }
        if (timed && ok) cudaEventRecord(events_[kEvPre], stream_);

        if (ok && (consumeFault(Fault::Enqueue) || !ctx_->enqueueV3(stream_))) {
            noteError(stats_.trtFail);
            logLimited(limTrt_);
            ok = false;
        }
        if (timed && ok) cudaEventRecord(events_[kEvTrt], stream_);

        if (ok) {
            const bool d2hFault = consumeFault(Fault::D2H);
            for (int i = 0; i < kNumOuts; ++i) {
                if (d2hFault ||
                    cudaMemcpyAsync(hOut_[i], dOut_[i], outBytes_[i],
                                    cudaMemcpyDeviceToHost, stream_) != cudaSuccess) {
                    noteError(stats_.d2hFail);
                    logLimited(limD2h_);
                    ok = false;
                    break;
                }
            }
        }
        if (timed && ok) cudaEventRecord(events_[kEvD2H], stream_);

        if (cudaStreamSynchronize(stream_) != cudaSuccess) {
            noteError(stats_.syncFail);
            logLimited(limSync_);
            return {};
        }
        if (!ok) return {};

        if (consumeFault(Fault::NaNOutputs)) {
            const float nan = std::numeric_limits<float>::quiet_NaN();
            std::fill(hOut_[kLocRow], hOut_[kLocRow] + outBytes_[kLocRow] / sizeof(float), nan);
            std::fill(hOut_[kLocCol], hOut_[kLocCol] + outBytes_[kLocCol] / sizeof(float), nan);
        }

        const int64_t decStart = monoNs();
        LaneResult result = decode();
        result.timing.doneMonoNs = monoNs();
        if (result.nonFinite > 0)
            stats_.nonFinite.fetch_add(1, std::memory_order_relaxed);

        if (timed) {
            float ms = 0.0f;
            auto span = [&](int a, int b) {
                return cudaEventElapsedTime(&ms, events_[a], events_[b]) == cudaSuccess
                     ? ms * 1000.0f : -1.0f;
            };
            result.timing.h2dUs = span(kEvStart, kEvH2D);
            result.timing.preUs = span(kEvH2D, kEvPre);
            result.timing.trtUs = span(kEvPre, kEvTrt);
            result.timing.d2hUs = span(kEvTrt, kEvD2H);
            result.timing.decUs = static_cast<float>(result.timing.doneMonoNs - decStart) / 1000.0f;
        }
        return result;
    }

    // ── UFLD v2 postprocessing (ufld_decode.cpp) ──────────────────────────────

    LaneResult decode() const {
        UfldDims dims;
        dims.numCellRow    = config_.numCellRow;
        dims.numRowAnchors = config_.numRowAnchors;
        dims.numCellCol    = config_.numCellCol;
        dims.numColAnchors = config_.numColAnchors;
        dims.numLanes      = config_.numLanes;
        UfldDecodeParams params;
        params.srcW           = static_cast<float>(srcW_);
        params.rowAnchorStart = config_.rowAnchorStart;
        params.laneReferenceY = config_.laneReferenceY;

        const UfldDecodeOut d = decodeUfldV2(hOut_[kLocRow], hOut_[kLocCol],
                                             hOut_[kExistRow], hOut_[kExistCol],
                                             dims, params);
        const LaneEgo e = egoFromSlots(d.slots, static_cast<float>(srcW_) * 0.5f);

        LaneResult result;
        result.valid            = true;
        result.slots            = d.slots;
        result.nonFinite        = d.nonFinite;
        result.numLanes         = e.numLanes;
        result.currentLaneIndex = e.currentLaneIndex;
        result.lateralOffset    = e.lateralOffset;
        result.lateralValid     = e.lateralValid;
        return result;
    }

    // ── thread control ────────────────────────────────────────────────────────

    void start() {
        if (inferThread_.joinable()) return;   // already running
        // A new run starts clean: the previous run's result must not read as
        // valid / fresh, and readiness (processedFrameCount() > 0) must mean
        // this run produced a frame (plan bug B12).
        {
            std::lock_guard<std::mutex> lk(resultMutex_);
            latestResult_ = LaneResult{};
        }
        lastResultSteadyMs_.store(0, std::memory_order_release);
        processedFrames_.store(0, std::memory_order_release);
        stopFlag_.store(false, std::memory_order_relaxed);
        inferThread_ = std::thread(&LaneDetectorImpl::inferenceLoop, this);
    }

    void stop() {
        stopFlag_.store(true, std::memory_order_release);
        if (inferThread_.joinable()) inferThread_.join();
        for (ErrLimiter* l : { &limH2d_, &limPre_, &limTrt_, &limD2h_, &limSync_ })
            flushLimiter(*l);
    }

    LaneResult poll() const {
        std::lock_guard<std::mutex> lk(resultMutex_);
        return latestResult_;
    }

    uint64_t processedFrameCount() const {
        return processedFrames_.load(std::memory_order_acquire);
    }

    bool hasFreshResult(uint32_t maxAgeMs) const {
        const int64_t last = lastResultSteadyMs_.load(std::memory_order_acquire);
        if (last <= 0) return false;
        const int64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        return now >= last && static_cast<uint64_t>(now - last) <= maxAgeMs;
    }
};

std::atomic<int> LaneDetectorImpl::sCounter{0};
constexpr const char* LaneDetectorImpl::kOutNames[];

// ─── LaneDetector public API ──────────────────────────────────────────────────

LaneDetector::LaneDetector(uint32_t srcWidth, uint32_t srcHeight,
                            const LaneDetectorConfig& config)
    : impl_(std::make_unique<LaneDetectorImpl>(srcWidth, srcHeight, config))
{}

LaneDetector::~LaneDetector() = default;

GstElement* LaneDetector::createBin()       { return impl_->createBin(); }
void        LaneDetector::start()           { impl_->start(); }
void        LaneDetector::stop()            { impl_->stop(); }
LaneResult  LaneDetector::poll() const      { return impl_->poll(); }
uint64_t    LaneDetector::processedFrameCount() const {
    return impl_->processedFrameCount();
}
bool LaneDetector::hasFreshResult(uint32_t maxAgeMs) const {
    return impl_->hasFreshResult(maxAgeMs);
}

bool LaneDetector::setLogCallback(dashcam::log::LogCallback cb) {
    // The inference thread (and TensorRT) read log_: replacing it under the
    // running thread is a data race (plan bug B13).
    if (impl_->inferThread_.joinable()) {
        doLog(impl_->log_, dashcam::log::LogLevel::ERROR,
              "lane setLogCallback refused: inference thread is running");
        return false;
    }
    impl_->log_ = std::move(cb);
    return true;
}

bool LaneDetector::processFrame(const uint8_t* bgr, size_t bytes, int64_t ptsNs,
                                LaneResult& out, LaneDebugTensors* dbg) {
    return impl_->processFrame(bgr, bytes, ptsNs, out, dbg);
}

const LaneGeometry& LaneDetector::geometry() const { return impl_->geom_; }

LaneStats LaneDetector::stats() const {
    const auto& c = impl_->stats_;
    LaneStats s;
    s.pulled          = c.pulled.load(std::memory_order_relaxed);
    s.gated           = c.gated.load(std::memory_order_relaxed);
    s.mapFail         = c.mapFail.load(std::memory_order_relaxed);
    s.sizeMismatch    = c.sizeMismatch.load(std::memory_order_relaxed);
    s.h2dFail         = c.h2dFail.load(std::memory_order_relaxed);
    s.preFail         = c.preFail.load(std::memory_order_relaxed);
    s.trtFail         = c.trtFail.load(std::memory_order_relaxed);
    s.d2hFail         = c.d2hFail.load(std::memory_order_relaxed);
    s.syncFail        = c.syncFail.load(std::memory_order_relaxed);
    s.nonFinite       = c.nonFinite.load(std::memory_order_relaxed);
    s.processed       = c.processed.load(std::memory_order_relaxed);
    s.lastErrorMonoNs = c.lastErrorMonoNs.load(std::memory_order_relaxed);
    return s;
}

bool LaneDetector::setResultSink(std::function<void(const LaneResult&)> sink) {
    if (impl_->inferThread_.joinable()) {
        doLog(impl_->log_, dashcam::log::LogLevel::ERROR,
              "lane setResultSink refused: inference thread is running");
        return false;
    }
    impl_->sink_ = std::move(sink);
    return true;
}

// ─── test hook ────────────────────────────────────────────────────────────────

void LaneDetectorTestHook::injectFault(LaneDetector& det, Fault fault, int count) {
    det.impl_->faultLeft_[static_cast<int>(fault)].store(count, std::memory_order_relaxed);
}

bool LaneDetectorTestHook::onFrame(LaneDetector& det,
                                   std::function<void(const uint8_t*, size_t)> cb) {
    if (det.impl_->inferThread_.joinable()) return false;
    det.impl_->testFrameCb_ = std::move(cb);
    return true;
}

bool LaneDetectorTestHook::runModelSized(LaneDetector& det, const uint8_t* bgr,
                                         size_t bytes, LaneResult& out,
                                         LaneDebugTensors* dbg) {
    LaneDetectorImpl& im = *det.impl_;
    // Refuse before touching the ROI: the running thread reads preYOff_/preYScale_.
    if (im.inferThread_.joinable()) return false;
    if (im.srcW_ != im.config_.modelInputW || im.srcH_ != im.config_.modelInputH)
        return false;
    const float off = im.preYOff_, scale = im.preYScale_;
    im.preYOff_   = 0.0f;   // identity: model row m = branch row m
    im.preYScale_ = 1.0f;
    const bool ok = im.processFrame(bgr, bytes, -1, out, dbg);
    im.preYOff_   = off;
    im.preYScale_ = scale;
    return ok;
}

} // namespace dashcam::lane
