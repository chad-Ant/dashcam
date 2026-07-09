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
#include <cstdio>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

namespace dashcam::lane {

// ─── UFLD v1 row anchors ─────────────────────────────────────────────────────
static constexpr int kMaxRowAnchors = 56;
static constexpr float kRowAnchors[kMaxRowAnchors] = {
    0.400000f, 0.410909f, 0.421818f, 0.432727f, 0.443636f, 0.454545f,
    0.465455f, 0.476364f, 0.487273f, 0.498182f, 0.509091f, 0.520000f,
    0.530909f, 0.541818f, 0.552727f, 0.563636f, 0.574545f, 0.585455f,
    0.596364f, 0.607273f, 0.618182f, 0.629091f, 0.640000f, 0.650909f,
    0.661818f, 0.672727f, 0.683636f, 0.694545f, 0.705455f, 0.716364f,
    0.727273f, 0.738182f, 0.749091f, 0.760000f, 0.770909f, 0.781818f,
    0.792727f, 0.803636f, 0.814545f, 0.825455f, 0.836364f, 0.847273f,
    0.858182f, 0.869091f, 0.880000f, 0.890909f, 0.901818f, 0.912727f,
    0.923636f, 0.934545f, 0.945455f, 0.956364f, 0.967273f, 0.978182f,
    0.989091f, 1.000000f,
};

// ─── helpers ─────────────────────────────────────────────────────────────────

static void cudaCheck(cudaError_t err, const char* what) {
    if (err != cudaSuccess)
        throw std::runtime_error(
            std::string("liblanedetector: ") + what
            + ": " + cudaGetErrorString(err));
}

class TrtLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            std::fprintf(stderr, "[TRT/lane] %s\n", msg);
    }
};

// ─── PIMPL ───────────────────────────────────────────────────────────────────

class LaneDetectorImpl {
public:
    LaneDetectorConfig config_;
    uint32_t           srcW_;
    uint32_t           srcH_;

    // ── TRT ──────────────────────────────────────────────────────────────────
    TrtLogger                    logger_;
    nvinfer1::IRuntime*          runtime_ = nullptr;
    nvinfer1::ICudaEngine*       engine_  = nullptr;
    nvinfer1::IExecutionContext* ctx_     = nullptr;
    std::string                  inputName_;
    std::string                  outputName_;

    // ── CUDA ─────────────────────────────────────────────────────────────────
    cudaStream_t stream_     = nullptr;
    void*        dSrcBGR_    = nullptr;
    void*        dInput_     = nullptr;
    void*        dOutput_    = nullptr;
    size_t       srcBytes_   = 0;
    size_t       inputBytes_ = 0;
    size_t       outputBytes_= 0;
    float*       hOutput_    = nullptr;  // pinned

    // ── GStreamer ─────────────────────────────────────────────────────────────
    GstElement* appsink_ = nullptr;  // ref held via gst_bin_get_by_name

    // ── inference thread ──────────────────────────────────────────────────────
    std::thread       inferThread_;
    std::atomic<bool> stopFlag_{false};

    mutable std::mutex resultMutex_;
    LaneResult         latestResult_;

    static std::atomic<int> sCounter;

    // ── internal types ────────────────────────────────────────────────────────

    struct Boundary {
        float x1 = 0, y1 = 0;
        float x2 = 0, y2 = 0;
        bool  detected = false;

        float xAtY(float y) const {
            if (!detected || std::abs(y1 - y2) < 1.0f) return 0.0f;
            return x1 + (x2 - x1) * (y1 - y) / (y1 - y2);
        }
    };

    // ── construction / destruction ────────────────────────────────────────────

    LaneDetectorImpl(uint32_t srcW, uint32_t srcH, const LaneDetectorConfig& cfg)
        : config_(cfg), srcW_(srcW), srcH_(srcH)
    {
        if (config_.enginePath.empty())
            throw std::runtime_error("liblanedetector: enginePath is empty");
        if (config_.numRowAnchors > kMaxRowAnchors)
            throw std::runtime_error("liblanedetector: numRowAnchors > 56");

        try {
            loadEngine();
            allocBuffers();
            cudaCheck(cudaStreamCreate(&stream_), "cudaStreamCreate");
        } catch (...) {
            cleanup();
            throw;
        }
    }

    ~LaneDetectorImpl() {
        stop();
        cleanup();
    }

    void cleanup() noexcept {
        cudaFree(dSrcBGR_);       dSrcBGR_  = nullptr;
        cudaFree(dInput_);        dInput_   = nullptr;
        cudaFree(dOutput_);       dOutput_  = nullptr;
        cudaFreeHost(hOutput_);   hOutput_  = nullptr;
        if (stream_)  { cudaStreamDestroy(stream_);       stream_  = nullptr; }
        if (appsink_) { gst_object_unref(appsink_);       appsink_ = nullptr; }
        delete ctx_;     ctx_     = nullptr;
        delete engine_;  engine_  = nullptr;
        delete runtime_; runtime_ = nullptr;
    }

    // ── engine loading ────────────────────────────────────────────────────────

    void loadEngine() {
        std::ifstream file(config_.enginePath, std::ios::binary | std::ios::ate);
        if (!file.is_open())
            throw std::runtime_error("liblanedetector: cannot open engine: "
                                     + config_.enginePath);
        const auto size = file.tellg();
        if (size <= 0)
            throw std::runtime_error("liblanedetector: engine file empty or not "
                                     "seekable: " + config_.enginePath);
        file.seekg(0);
        std::vector<char> data(static_cast<size_t>(size));
        if (!file.read(data.data(), size))
            throw std::runtime_error("liblanedetector: engine read failed: "
                                     + config_.enginePath);

        runtime_ = nvinfer1::createInferRuntime(logger_);
        if (!runtime_)
            throw std::runtime_error("liblanedetector: createInferRuntime failed");

        engine_ = runtime_->deserializeCudaEngine(data.data(), data.size());
        if (!engine_)
            throw std::runtime_error("liblanedetector: deserializeCudaEngine failed — "
                                     "build with trtexec on this Jetson module");

        ctx_ = engine_->createExecutionContext();
        if (!ctx_)
            throw std::runtime_error("liblanedetector: createExecutionContext failed");

        const int32_t n = engine_->getNbIOTensors();
        for (int32_t i = 0; i < n; ++i) {
            const char* name = engine_->getTensorName(i);
            if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT
                && inputName_.empty())
                inputName_ = name;
            else if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kOUTPUT
                     && outputName_.empty())
                outputName_ = name;
        }
        if (inputName_.empty() || outputName_.empty())
            throw std::runtime_error("liblanedetector: engine must have one input "
                                     "and one output tensor");
    }

    // ── buffer allocation ─────────────────────────────────────────────────────

    void allocBuffers() {
        srcBytes_    = static_cast<size_t>(srcW_) * srcH_ * 3;
        inputBytes_  = static_cast<size_t>(config_.modelInputW)
                     * config_.modelInputH * 3 * sizeof(float);
        outputBytes_ = static_cast<size_t>(config_.numLanes)
                     * (config_.gridingNum + 1)
                     * config_.numRowAnchors * sizeof(float);

        cudaCheck(cudaMalloc(&dSrcBGR_,  srcBytes_),   "cudaMalloc dSrcBGR");
        cudaCheck(cudaMalloc(&dInput_,   inputBytes_),  "cudaMalloc dInput");
        cudaCheck(cudaMalloc(&dOutput_,  outputBytes_), "cudaMalloc dOutput");
        cudaCheck(cudaMallocHost(reinterpret_cast<void**>(&hOutput_), outputBytes_),
                  "cudaMallocHost hOutput");
    }

    // ── GStreamer bin ─────────────────────────────────────────────────────────

    GstElement* createBin() {
        if (appsink_) { gst_object_unref(appsink_); appsink_ = nullptr; }

        const std::string sinkName =
            "lanesink_" + std::to_string(sCounter.fetch_add(1));

        const std::string desc = config_.gstConversion
            + " ! video/x-raw,format=BGR"
              " ! appsink name=" + sinkName
            + " drop=true max-buffers=1 emit-signals=false sync=false";

        GError*     err = nullptr;
        GstElement* bin = gst_parse_bin_from_description(
            desc.c_str(), /*ghost_unlinked=*/TRUE, &err);
        if (err || !bin) {
            if (err) {
                std::fprintf(stderr, "[liblanedetector] bin parse failed: %s\n",
                             err->message);
                g_error_free(err);
            }
            if (bin) gst_object_unref(bin);
            return nullptr;
        }

        appsink_ = gst_bin_get_by_name(GST_BIN(bin), sinkName.c_str());
        if (!appsink_) {
            std::fprintf(stderr, "[liblanedetector] appsink '%s' not found\n",
                         sinkName.c_str());
            gst_object_unref(bin);
            return nullptr;
        }

        return bin;
    }

    // ── inference thread ──────────────────────────────────────────────────────

    void inferenceLoop() {
        using Clock    = std::chrono::steady_clock;
        using Duration = std::chrono::duration<double>;

        // Allow the first frame to run immediately.
        auto lastInfer = Clock::now() - std::chrono::seconds(1);
        const double minInterval = config_.targetHz > 0
            ? 1.0 / config_.targetHz
            : 0.0;

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
            GstMapInfo map;
            if (!gst_buffer_map(buf, &map, GST_MAP_READ)) {
                gst_sample_unref(sample);
                continue;
            }

            LaneResult result = runInference(map.data);

            gst_buffer_unmap(buf, &map);
            gst_sample_unref(sample);

            std::lock_guard<std::mutex> lk(resultMutex_);
            latestResult_ = std::move(result);
        }
    }

    // ── inference pipeline ────────────────────────────────────────────────────

    LaneResult runInference(const uint8_t* bgr) {
        if (cudaMemcpyAsync(dSrcBGR_, bgr, srcBytes_,
                            cudaMemcpyHostToDevice, stream_) != cudaSuccess) {
            std::fprintf(stderr, "[liblanedetector] H2D memcpy failed\n");
            return {};
        }

        launchPreprocessKernel(
            static_cast<const uint8_t*>(dSrcBGR_),
            static_cast<float*>(dInput_),
            static_cast<int>(srcW_), static_cast<int>(srcH_),
            static_cast<int>(config_.modelInputW),
            static_cast<int>(config_.modelInputH),
            config_.meanR, config_.meanG, config_.meanB,
            config_.stdR,  config_.stdG,  config_.stdB,
            stream_);

        if (!ctx_->setTensorAddress(inputName_.c_str(),  dInput_) ||
            !ctx_->setTensorAddress(outputName_.c_str(), dOutput_)) {
            std::fprintf(stderr, "[liblanedetector] setTensorAddress failed\n");
            return {};
        }
        if (!ctx_->enqueueV3(stream_)) {
            std::fprintf(stderr, "[liblanedetector] enqueueV3 failed\n");
            return {};
        }

        if (cudaMemcpyAsync(hOutput_, dOutput_, outputBytes_,
                            cudaMemcpyDeviceToHost, stream_) != cudaSuccess) {
            std::fprintf(stderr, "[liblanedetector] D2H memcpy failed\n");
            return {};
        }
        if (cudaStreamSynchronize(stream_) != cudaSuccess) {
            std::fprintf(stderr, "[liblanedetector] sync failed\n");
            return {};
        }

        return decode();
    }

    // ── UFLD postprocessing ───────────────────────────────────────────────────

    Boundary decodeBoundary(int laneIdx) const {
        const int G = config_.gridingNum;
        const int R = config_.numRowAnchors;
        const float* base = hOutput_ + laneIdx * (G + 1) * R;

        struct Pt { float x, y; };
        std::vector<Pt> pts;
        pts.reserve(static_cast<size_t>(R));

        for (int r = 0; r < R; ++r) {
            int   bestCol = 0;
            float bestVal = base[0 * R + r];
            for (int g = 1; g <= G; ++g) {
                const float v = base[g * R + r];
                if (v > bestVal) { bestVal = v; bestCol = g; }
            }
            if (bestCol == G) continue;

            const float normX = (static_cast<float>(bestCol) + 0.5f) / G;
            pts.push_back({ normX * srcW_, kRowAnchors[r] * srcH_ });
        }

        if (pts.size() < 2) return {};

        Boundary b;
        b.x1 = pts.back().x;  b.y1 = pts.back().y;
        b.x2 = pts.front().x; b.y2 = pts.front().y;
        b.detected = true;
        return b;
    }

    LaneResult decode() const {
        std::vector<Boundary> bounds;
        bounds.reserve(static_cast<size_t>(config_.numLanes));
        for (int i = 0; i < config_.numLanes; ++i)
            bounds.push_back(decodeBoundary(i));

        const float refY = srcH_ * config_.laneReferenceY;
        std::vector<float> xs;
        for (const auto& b : bounds)
            if (b.detected) xs.push_back(b.xAtY(refY));
        std::sort(xs.begin(), xs.end());

        if (xs.size() < 2) return {};

        LaneResult result;
        result.numLanes = static_cast<int8_t>(xs.size() - 1);

        const float vehicleX = srcW_ * 0.5f;
        result.currentLaneIndex = -1;
        for (uint8_t i = 0; i + 1 < static_cast<uint8_t>(xs.size()); ++i) {
            if (vehicleX >= xs[i] && vehicleX <= xs[i + 1]) {
                result.currentLaneIndex = static_cast<int8_t>(i);
                break;
            }
        }

        result.laneAllowedDirections.assign(
            static_cast<size_t>(result.numLanes), LaneDirection::Straight);
        return result;
    }

    // ── thread control ────────────────────────────────────────────────────────

    void start() {
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
};

std::atomic<int> LaneDetectorImpl::sCounter{0};

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

} // namespace dashcam::lane
