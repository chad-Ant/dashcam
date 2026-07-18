#include "libdriverstate.h"
#include "libdriverstate_preprocess.h"

#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <cuda_runtime.h>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

namespace dashcam::driver {

// ─── helpers ─────────────────────────────────────────────────────────────────

static void cudaCheck(cudaError_t err, const char* what) {
    if (err != cudaSuccess)
        throw std::runtime_error(
            std::string("libdriverstate: ") + what
            + ": " + cudaGetErrorString(err));
}

class TrtLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            std::fprintf(stderr, "[TRT/driver] %s\n", msg);
    }
};

static constexpr int kNumClasses = 3;  // NEUTRAL, SLEEPY, DISTRACTED

// ─── PIMPL ───────────────────────────────────────────────────────────────────

class DriverStateImpl {
public:
    DriverStateConfig config_;
    uint32_t          srcW_;
    uint32_t          srcH_;

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
    size_t       inBytes_    = 0;
    size_t       outBytes_   = 0;
    float*       hOutput_    = nullptr;  // pinned, kNumClasses floats

    // ── GStreamer ─────────────────────────────────────────────────────────────
    GstElement* appsink_ = nullptr;

    // ── inference thread ──────────────────────────────────────────────────────
    std::thread       inferThread_;
    std::atomic<bool> stopFlag_{false};

    mutable std::mutex  resultMutex_;
    DriverStateResult   latestResult_;

    static std::atomic<int> sCounter;

    // ── construction / destruction ────────────────────────────────────────────

    DriverStateImpl(uint32_t srcW, uint32_t srcH, const DriverStateConfig& cfg)
        : config_(cfg), srcW_(srcW), srcH_(srcH)
    {
        if (config_.enginePath.empty())
            throw std::runtime_error("libdriverstate: enginePath is empty");

        try {
            loadEngine();
            allocBuffers();
            cudaCheck(cudaStreamCreate(&stream_), "cudaStreamCreate");
        } catch (...) {
            cleanup();
            throw;
        }
    }

    ~DriverStateImpl() {
        stop();
        cleanup();
    }

    void cleanup() noexcept {
        cudaFree(dSrcBGR_);       dSrcBGR_  = nullptr;
        cudaFree(dInput_);        dInput_   = nullptr;
        cudaFree(dOutput_);       dOutput_  = nullptr;
        cudaFreeHost(hOutput_);   hOutput_  = nullptr;
        if (stream_)  { cudaStreamDestroy(stream_);  stream_  = nullptr; }
        if (appsink_) { gst_object_unref(appsink_);  appsink_ = nullptr; }
        delete ctx_;     ctx_     = nullptr;
        delete engine_;  engine_  = nullptr;
        delete runtime_; runtime_ = nullptr;
    }

    // ── engine loading ────────────────────────────────────────────────────────

    void loadEngine() {
        std::ifstream file(config_.enginePath, std::ios::binary | std::ios::ate);
        if (!file.is_open())
            throw std::runtime_error("libdriverstate: cannot open engine: "
                                     + config_.enginePath);
        const auto size = file.tellg();
        if (size <= 0)
            throw std::runtime_error("libdriverstate: engine file empty or not "
                                     "seekable: " + config_.enginePath);
        file.seekg(0);
        std::vector<char> data(static_cast<size_t>(size));
        if (!file.read(data.data(), size))
            throw std::runtime_error("libdriverstate: engine read failed: "
                                     + config_.enginePath);

        runtime_ = nvinfer1::createInferRuntime(logger_);
        if (!runtime_)
            throw std::runtime_error("libdriverstate: createInferRuntime failed");

        engine_ = runtime_->deserializeCudaEngine(data.data(), data.size());
        if (!engine_)
            throw std::runtime_error("libdriverstate: deserializeCudaEngine failed — "
                                     "build with trtexec on this Jetson module");

        ctx_ = engine_->createExecutionContext();
        if (!ctx_)
            throw std::runtime_error("libdriverstate: createExecutionContext failed");

        const int32_t n = engine_->getNbIOTensors();
        for (int32_t i = 0; i < n; ++i) {
            const char* name = engine_->getIOTensorName(i);
            if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT
                && inputName_.empty())
                inputName_ = name;
            else if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kOUTPUT
                     && outputName_.empty())
                outputName_ = name;
        }
        if (inputName_.empty() || outputName_.empty())
            throw std::runtime_error("libdriverstate: engine must have one input "
                                     "and one output tensor");

        // Validate output shape: expected [1, 3] (batch × num_classes).
        const auto shape = engine_->getTensorShape(outputName_.c_str());
        const int  lastDim = shape.d[shape.nbDims - 1];
        if (lastDim != kNumClasses)
            throw std::runtime_error(
                "libdriverstate: output last dim=" + std::to_string(lastDim)
                + " but expected " + std::to_string(kNumClasses)
                + " (NEUTRAL/SLEEPY/DISTRACTED)");
    }

    // ── buffer allocation ─────────────────────────────────────────────────────

    void allocBuffers() {
        srcBytes_ = static_cast<size_t>(srcW_) * srcH_ * 3;
        inBytes_  = static_cast<size_t>(config_.modelInputW)
                  * config_.modelInputH * 3 * sizeof(float);
        outBytes_ = static_cast<size_t>(kNumClasses) * sizeof(float);

        cudaCheck(cudaMalloc(&dSrcBGR_, srcBytes_),  "cudaMalloc dSrcBGR");
        cudaCheck(cudaMalloc(&dInput_,  inBytes_),   "cudaMalloc dInput");
        cudaCheck(cudaMalloc(&dOutput_, outBytes_),  "cudaMalloc dOutput");
        cudaCheck(cudaMallocHost(reinterpret_cast<void**>(&hOutput_), outBytes_),
                  "cudaMallocHost hOutput");
    }

    // ── GStreamer bin ─────────────────────────────────────────────────────────

    GstElement* createBin() {
        if (appsink_) { gst_object_unref(appsink_); appsink_ = nullptr; }

        const std::string sinkName =
            "dstatesink_" + std::to_string(sCounter.fetch_add(1));

        const std::string desc = config_.gstConversion
            + " ! video/x-raw,format=BGR"
              " ! appsink name=" + sinkName
            + " drop=true max-buffers=1 emit-signals=false sync=false";

        GError*     err = nullptr;
        GstElement* bin = gst_parse_bin_from_description(
            desc.c_str(), /*ghost_unlinked=*/TRUE, &err);
        if (err || !bin) {
            if (err) {
                std::fprintf(stderr, "[libdriverstate] bin parse failed: %s\n",
                             err->message);
                g_error_free(err);
            }
            if (bin) gst_object_unref(bin);
            return nullptr;
        }

        appsink_ = gst_bin_get_by_name(GST_BIN(bin), sinkName.c_str());
        if (!appsink_) {
            std::fprintf(stderr, "[libdriverstate] appsink '%s' not found\n",
                         sinkName.c_str());
            gst_object_unref(bin);
            return nullptr;
        }

        return bin;
    }

    // ── inference thread ──────────────────────────────────────────────────────

    void inferenceLoop() {
        using Clock = std::chrono::steady_clock;
        auto lastInfer = Clock::now() - std::chrono::seconds(1);
        const double minInterval = config_.targetHz > 0
            ? 1.0 / config_.targetHz
            : 0.0;

        while (!stopFlag_.load(std::memory_order_relaxed)) {
            if (!appsink_) break;

            GstSample* sample = gst_app_sink_try_pull_sample(
                GST_APP_SINK(appsink_), 100 * GST_MSECOND);
            if (!sample) continue;

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

            DriverStateResult result = runInference(map.data);

            gst_buffer_unmap(buf, &map);
            gst_sample_unref(sample);

            std::lock_guard<std::mutex> lk(resultMutex_);
            latestResult_ = result;
        }
    }

    // ── inference pipeline ────────────────────────────────────────────────────

    DriverStateResult runInference(const uint8_t* bgr) {
        if (cudaMemcpyAsync(dSrcBGR_, bgr, srcBytes_,
                            cudaMemcpyHostToDevice, stream_) != cudaSuccess) {
            std::fprintf(stderr, "[libdriverstate] H2D memcpy failed\n");
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
            std::fprintf(stderr, "[libdriverstate] setTensorAddress failed\n");
            return {};
        }
        if (!ctx_->enqueueV3(stream_)) {
            std::fprintf(stderr, "[libdriverstate] enqueueV3 failed\n");
            return {};
        }
        if (cudaMemcpyAsync(hOutput_, dOutput_, outBytes_,
                            cudaMemcpyDeviceToHost, stream_) != cudaSuccess) {
            std::fprintf(stderr, "[libdriverstate] D2H memcpy failed\n");
            return {};
        }
        if (cudaStreamSynchronize(stream_) != cudaSuccess) {
            std::fprintf(stderr, "[libdriverstate] sync failed\n");
            return {};
        }

        return decode();
    }

    // ── postprocessing ────────────────────────────────────────────────────────
    // hOutput_ holds kNumClasses raw logits (or probabilities if the model
    // already applies softmax).  We optionally apply softmax then take argmax.

    DriverStateResult decode() const {
        float probs[kNumClasses];

        if (config_.applySoftmax) {
            // Numerically stable softmax: subtract max before exp.
            const float maxLogit = *std::max_element(hOutput_, hOutput_ + kNumClasses);
            float sum = 0.0f;
            for (int i = 0; i < kNumClasses; ++i) {
                probs[i] = std::exp(hOutput_[i] - maxLogit);
                sum += probs[i];
            }
            for (int i = 0; i < kNumClasses; ++i) probs[i] /= sum;
        } else {
            for (int i = 0; i < kNumClasses; ++i) probs[i] = hOutput_[i];
        }

        // Argmax.
        int   bestIdx  = 0;
        float bestProb = probs[0];
        for (int i = 1; i < kNumClasses; ++i) {
            if (probs[i] > bestProb) { bestProb = probs[i]; bestIdx = i; }
        }

        DriverStateResult r;
        r.state      = static_cast<DriverState>(bestIdx);
        r.confidence = bestProb;
        r.valid      = true;
        return r;
    }

    // ── thread control ────────────────────────────────────────────────────────

    void start() {
        stopFlag_.store(false, std::memory_order_relaxed);
        inferThread_ = std::thread(&DriverStateImpl::inferenceLoop, this);
    }

    void stop() {
        stopFlag_.store(true, std::memory_order_release);
        if (inferThread_.joinable()) inferThread_.join();
    }

    DriverStateResult poll() const {
        std::lock_guard<std::mutex> lk(resultMutex_);
        return latestResult_;
    }
};

std::atomic<int> DriverStateImpl::sCounter{0};

// ─── DriverStateDetector public API ──────────────────────────────────────────

DriverStateDetector::DriverStateDetector(uint32_t srcWidth, uint32_t srcHeight,
                                         const DriverStateConfig& config)
    : impl_(std::make_unique<DriverStateImpl>(srcWidth, srcHeight, config))
{}

DriverStateDetector::~DriverStateDetector() = default;

GstElement*       DriverStateDetector::createBin()       { return impl_->createBin(); }
void              DriverStateDetector::start()            { impl_->start(); }
void              DriverStateDetector::stop()             { impl_->stop(); }
DriverStateResult DriverStateDetector::poll() const       { return impl_->poll(); }

} // namespace dashcam::driver
