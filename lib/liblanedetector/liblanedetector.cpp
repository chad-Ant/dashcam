#include "liblanedetector.h"
#include "liblanedetector_preprocess.h"

#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <cuda_runtime.h>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <fstream>
#include <mutex>
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
    cb(lvl, buf);
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
        if (cb && *cb) (*cb)(lvl, std::string("[TRT/lane] ") + msg);
        else std::fprintf(stderr, "[TRT/lane] %s\n", msg);
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

    // ── internal types ────────────────────────────────────────────────────────

    // Least-squares line x = slope·y + intercept over all decoded points.
    struct Boundary {
        float slope = 0, intercept = 0;
        bool  detected = false;

        float xAtY(float y) const {
            return detected ? slope * y + intercept : 0.0f;
        }
    };

    // ── construction / destruction ────────────────────────────────────────────

    LaneDetectorImpl(uint32_t srcW, uint32_t srcH, const LaneDetectorConfig& cfg)
        : config_(cfg), log_(cfg.log), srcW_(srcW), srcH_(srcH)
    {
        logger_.cb = &log_;

        if (config_.enginePath.empty())
            throw std::runtime_error("liblanedetector: enginePath is empty");
        if (srcW_ == 0 || srcH_ == 0)
            throw std::runtime_error("liblanedetector: source dimensions are 0");
        if (config_.numLanes != 4)
            throw std::runtime_error("liblanedetector: UFLD v2 decode requires "
                                     "numLanes == 4 (slots 1,2 row / 0,3 col)");
        if (config_.numRowAnchors < 2 || config_.numColAnchors < 2 ||
            config_.numCellRow < 2 || config_.numCellCol < 2)
            throw std::runtime_error("liblanedetector: head geometry must be >= 2");
        if (!(config_.cropRatio > 0.0f) || config_.cropRatio > 1.0f)
            throw std::runtime_error("liblanedetector: cropRatio must be in (0, 1]");
        if (config_.inputCropTop < 0.0f || config_.inputCropTop > 0.9f)
            throw std::runtime_error("liblanedetector: inputCropTop must be in [0, 0.9]");
        if (config_.inputCropBottom < 0.0f || config_.inputCropBottom > 0.9f)
            throw std::runtime_error("liblanedetector: inputCropBottom must be in [0, 0.9]");

        // Branch crop: even row counts for NVMM/NV12 chroma alignment.
        cropTopPx_    = static_cast<uint32_t>(
            std::lround(srcH * config_.inputCropTop)) & ~1u;
        cropBottomPx_ = static_cast<uint32_t>(
            std::lround(srcH * config_.inputCropBottom)) & ~1u;
        if (cropTopPx_ + cropBottomPx_ + 64 > srcH)
            throw std::runtime_error("liblanedetector: crop leaves fewer than "
                                     "64 rows of frame");
        srcH_ = srcH - cropTopPx_ - cropBottomPx_;

        // The model should see the bottom cropRatio of the ORIGINAL frame.
        // Inside the branch-cropped band that region is cropRatio/keptFrac of
        // the height — clamped to 1 when the branch already cropped more than
        // the training transform would have (then the whole band is used; a
        // bottom crop always lands here since it removes training-visible
        // rows deliberately, e.g. the bonnet).
        const float keptFrac  = static_cast<float>(srcH_) / srcH;
        const float effRatio  = std::min(1.0f, config_.cropRatio / keptFrac);
        const int resizedH =
            static_cast<int>(std::lround(config_.modelInputH / effRatio));
        const int cropTopRows = resizedH - static_cast<int>(config_.modelInputH);
        preYScale_ = static_cast<float>(srcH_) / static_cast<float>(resizedH);
        preYOff_   = static_cast<float>(cropTopRows) * preYScale_;

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

        std::string chain;
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

    void inferenceLoop() {
        using Clock = std::chrono::steady_clock;

        // Allow the first frame to run immediately.
        auto lastInfer = Clock::now() - std::chrono::seconds(1);
        const double minInterval = config_.targetHz > 0
            ? 1.0 / config_.targetHz
            : 0.0;
        bool sizeWarned = false;

        while (!stopFlag_.load(std::memory_order_relaxed)) {
            if (!appsink_) break;

            GstSample* sample = gst_app_sink_try_pull_sample(
                GST_APP_SINK(appsink_), 100 * GST_MSECOND);
            if (!sample) continue;

            // Time-gate: drop the frame if we haven't reached the next inference window.
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

            LaneResult result;
            const bool sizeOk = map.size == srcBytes_;
            if (sizeOk) {
                result = runInference(map.data);
            } else if (!sizeWarned) {
                sizeWarned = true;
                doLog(log_, dashcam::log::LogLevel::ERROR,
                      "lane frame size %zu != expected %zu (%ux%ux3) — "
                      "frames skipped", map.size, srcBytes_, srcW_, srcH_);
            }

            gst_buffer_unmap(buf, &map);
            gst_sample_unref(sample);

            if (sizeOk && result.valid) {
                const uint64_t sequence =
                    processedFrames_.fetch_add(1, std::memory_order_relaxed) + 1;
                result.sequence = sequence;
                const int64_t nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                    Clock::now().time_since_epoch()).count();
                std::lock_guard<std::mutex> lk(resultMutex_);
                latestResult_ = std::move(result);
                lastResultSteadyMs_.store(nowMs, std::memory_order_release);
            }
        }
    }

    // ── inference pipeline ────────────────────────────────────────────────────

    LaneResult runInference(const uint8_t* bgr) {
        if (cudaMemcpyAsync(dSrcBGR_, bgr, srcBytes_,
                            cudaMemcpyHostToDevice, stream_) != cudaSuccess) {
            doLog(log_, dashcam::log::LogLevel::ERROR, "lane H2D memcpy failed");
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
            static_cast<int>(config_.modelInputW),
            static_cast<int>(config_.modelInputH),
            preYOff_, preYScale_,
            config_.meanR, config_.meanG, config_.meanB,
            config_.stdR,  config_.stdG,  config_.stdB,
            stream_);
        if (launchErr != cudaSuccess) {
            doLog(log_, dashcam::log::LogLevel::ERROR,
                  "lane preprocess launch failed: %s",
                  cudaGetErrorString(launchErr));
            ok = false;
        }

        if (ok && !ctx_->enqueueV3(stream_)) {
            doLog(log_, dashcam::log::LogLevel::ERROR, "lane enqueueV3 failed");
            ok = false;
        }

        if (ok) {
            for (int i = 0; i < kNumOuts; ++i) {
                if (cudaMemcpyAsync(hOut_[i], dOut_[i], outBytes_[i],
                                    cudaMemcpyDeviceToHost, stream_) != cudaSuccess) {
                    doLog(log_, dashcam::log::LogLevel::ERROR,
                          "lane D2H memcpy failed");
                    ok = false;
                    break;
                }
            }
        }

        if (cudaStreamSynchronize(stream_) != cudaSuccess) {
            doLog(log_, dashcam::log::LogLevel::ERROR, "lane stream sync failed");
            return {};
        }
        if (!ok) return {};

        return decode();
    }

    // ── UFLD v2 postprocessing ────────────────────────────────────────────────
    //
    // Output layouts (batch dropped, row-major):
    //   loc_row  [G=numCellRow][K=numRowAnchors][L]  location logits
    //   exist_row[2]           [K]               [L]  0=absent, 1=present
    //   loc_col  [G=numCellCol][K=numColAnchors][L]
    //   exist_col[2]           [K]               [L]
    // Lane slots: 1,2 decode via the row head, 0,3 via the column head
    // (fixed by the UFLD v2 architecture / reference demo).
    //
    // All y values are fractions of the model's implied full frame; the lane
    // count / ego-lane geometry is scale-invariant, so no pixel mapping of y
    // is ever needed (and the branch crop drops out entirely).

    float rowAnchorY(int k) const {
        const float a = config_.rowAnchorStart;
        return a + (1.0f - a) * static_cast<float>(k)
                 / static_cast<float>(config_.numRowAnchors - 1);
    }

    // Soft local argmax over cells [g*-1, g*+1] (demo local_width = 1):
    // returns the sub-cell location in grid units, offset by +0.5.
    static float softLocalArgmax(const float* loc, int G, int stride, int off,
                                 int gStar) {
        const int lo = std::max(0, gStar - 1);
        const int hi = std::min(G - 1, gStar + 1);
        float m = loc[lo * stride + off];
        for (int g = lo + 1; g <= hi; ++g)
            m = std::max(m, loc[g * stride + off]);
        float se = 0.0f, acc = 0.0f;
        for (int g = lo; g <= hi; ++g) {
            const float e = std::exp(loc[g * stride + off] - m);
            se  += e;
            acc += e * static_cast<float>(g);
        }
        return acc / se + 0.5f;
    }

    static Boundary boundaryFromMoments(int n, float sumX, float sumY,
                                        float sumYX, float sumYY) {
        if (n < 2) return {};
        const float invN = 1.0f / static_cast<float>(n);
        const float covYX = sumYX - sumY * sumX * invN;
        const float varY  = sumYY - sumY * sumY * invN;
        if (varY < 1e-6f) return {};
        Boundary b;
        b.slope = covYX / varY;
        b.intercept = (sumX - b.slope * sumY) * invN;
        b.detected = true;
        return b;
    }

    // Decode and fit one lane slot directly from the row head.  Accumulating
    // least-squares moments avoids allocating point vectors on every frame.
    Boundary boundaryRow(int lane) const {
        const int G = config_.numCellRow;
        const int K = config_.numRowAnchors;
        const int L = config_.numLanes;
        const float* loc   = hOut_[kLocRow];
        const float* exist = hOut_[kExistRow];

        int existCount = 0;
        for (int k = 0; k < K; ++k)
            if (exist[1 * K * L + k * L + lane] > exist[0 * K * L + k * L + lane])
                ++existCount;

        if (existCount * 2 <= K) return {};    // demo: sum > K/2

        float sumX = 0.0f, sumY = 0.0f, sumYX = 0.0f, sumYY = 0.0f;
        int n = 0;
        for (int k = 0; k < K; ++k) {
            if (exist[1 * K * L + k * L + lane] <= exist[0 * K * L + k * L + lane])
                continue;
            const int off = k * L + lane;
            int   gStar = 0;
            float best  = loc[off];
            for (int g = 1; g < G; ++g) {
                const float v = loc[g * K * L + off];
                if (v > best) { best = v; gStar = g; }
            }
            const float cell = softLocalArgmax(loc, G, K * L, off, gStar);
            const float x = cell / (G - 1) * srcW_;
            const float y = rowAnchorY(k);
            sumX += x; sumY += y; sumYX += y * x; sumYY += y * y; ++n;
        }
        return boundaryFromMoments(n, sumX, sumY, sumYX, sumYY);
    }

    // Decode and fit one lane slot directly from the column head.
    Boundary boundaryCol(int lane) const {
        const int G = config_.numCellCol;
        const int K = config_.numColAnchors;
        const int L = config_.numLanes;
        const float* loc   = hOut_[kLocCol];
        const float* exist = hOut_[kExistCol];

        int existCount = 0;
        for (int k = 0; k < K; ++k)
            if (exist[1 * K * L + k * L + lane] > exist[0 * K * L + k * L + lane])
                ++existCount;

        if (existCount * 4 <= K) return {};    // demo: sum > K/4

        float sumX = 0.0f, sumY = 0.0f, sumYX = 0.0f, sumYY = 0.0f;
        int n = 0;
        for (int k = 0; k < K; ++k) {
            if (exist[1 * K * L + k * L + lane] <= exist[0 * K * L + k * L + lane])
                continue;
            const int off = k * L + lane;
            int   gStar = 0;
            float best  = loc[off];
            for (int g = 1; g < G; ++g) {
                const float v = loc[g * K * L + off];
                if (v > best) { best = v; gStar = g; }
            }
            const float cell = softLocalArgmax(loc, G, K * L, off, gStar);
            const float x = static_cast<float>(k)
                          / static_cast<float>(K - 1) * srcW_;
            const float y = cell / (G - 1);
            sumX += x; sumY += y; sumYX += y * x; sumYY += y * y; ++n;
        }
        return boundaryFromMoments(n, sumX, sumY, sumYX, sumYY);
    }

    LaneResult decode() const {
        std::array<Boundary, 4> bounds{};
        for (int i = 0; i < config_.numLanes; ++i) {
            const bool rowSlot = (i == 1 || i == 2);
            bounds[static_cast<size_t>(i)] =
                rowSlot ? boundaryRow(i) : boundaryCol(i);
        }

        const float refY = config_.laneReferenceY;   // normalised
        std::array<float, 4> xs{};
        size_t xsCount = 0;
        for (const auto& b : bounds)
            if (b.detected) xs[xsCount++] = b.xAtY(refY);
        std::sort(xs.begin(), xs.begin() + static_cast<std::ptrdiff_t>(xsCount));

        LaneResult result;
        result.valid = true;
        if (xsCount < 2) return result;
        result.numLanes = static_cast<int8_t>(xsCount - 1);

        const float vehicleX = srcW_ * 0.5f;
        result.currentLaneIndex = -1;
        for (size_t i = 0; i + 1 < xsCount; ++i) {
            if (vehicleX >= xs[i] && vehicleX <= xs[i + 1]) {
                result.currentLaneIndex = static_cast<int8_t>(i);
                // Lateral projection inside the ego lane: -1 on the left
                // boundary, 0 centred, +1 on the right boundary.
                const float centre    = 0.5f * (xs[i] + xs[i + 1]);
                const float halfWidth = 0.5f * (xs[i + 1] - xs[i]);
                if (halfWidth > 1.0f) {   // degenerate/crossed boundaries guard
                    result.lateralOffset = (vehicleX - centre) / halfWidth;
                    result.lateralValid  = true;
                }
                break;
            }
        }

        return result;
    }

    // ── thread control ────────────────────────────────────────────────────────

    void start() {
        if (inferThread_.joinable()) return;   // already running
        stopFlag_.store(false, std::memory_order_relaxed);
        inferThread_ = std::thread(&LaneDetectorImpl::inferenceLoop, this);
    }

    void stop() {
        stopFlag_.store(true, std::memory_order_release);
        if (inferThread_.joinable()) inferThread_.join();
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

void LaneDetector::setLogCallback(dashcam::log::LogCallback cb) {
    impl_->log_ = std::move(cb);
}

} // namespace dashcam::lane
