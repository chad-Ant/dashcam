#include "libsigndetector.h"
#include "libsigndetector_preprocess.h"

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
#include <string>
#include <thread>
#include <vector>

namespace dashcam::sign {

// ─── CUDA / TRT helpers ───────────────────────────────────────────────────────

static void cudaCheck(cudaError_t err, const char* what) {
    if (err != cudaSuccess)
        throw std::runtime_error(
            std::string("libsigndetector: ") + what
            + ": " + cudaGetErrorString(err));
}

class TrtLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            std::fprintf(stderr, "[TRT/sign] %s\n", msg);
    }
};

// ─── IoU for NMS ─────────────────────────────────────────────────────────────

static float iou(const Detection& a, const Detection& b) {
    const float ix1 = std::max(a.x1, b.x1);
    const float iy1 = std::max(a.y1, b.y1);
    const float ix2 = std::min(a.x2, b.x2);
    const float iy2 = std::min(a.y2, b.y2);
    const float inter = std::max(0.0f, ix2 - ix1) * std::max(0.0f, iy2 - iy1);
    const float aArea = (a.x2 - a.x1) * (a.y2 - a.y1);
    const float bArea = (b.x2 - b.x1) * (b.y2 - b.y1);
    return inter / (aArea + bArea - inter + 1e-6f);
}

// ─── PIMPL ────────────────────────────────────────────────────────────────────

class SignDetectorImpl {
public:
    SignDetectorConfig config_;
    uint32_t           srcW_;
    uint32_t           srcH_;

    // ── TRT ──────────────────────────────────────────────────────────────────
    TrtLogger                    logger_;
    nvinfer1::IRuntime*          runtime_ = nullptr;
    nvinfer1::ICudaEngine*       engine_  = nullptr;
    nvinfer1::IExecutionContext* ctx_     = nullptr;

    std::string inputName_;
    std::string outputName_;
    int         numAnchors_ = 0;   // read from engine output shape

    // ── CUDA ─────────────────────────────────────────────────────────────────
    cudaStream_t stream_    = nullptr;
    void*        dSrcBGR_   = nullptr;
    void*        dInput_    = nullptr;
    void*        dOutput_   = nullptr;
    size_t       srcBytes_  = 0;
    size_t       inBytes_   = 0;
    size_t       outBytes_  = 0;
    float*       hOutput_   = nullptr;  // pinned

    // ── GStreamer ─────────────────────────────────────────────────────────────
    GstElement*  appsink_   = nullptr;  // non-owning ref (ref held via get_by_name)

    // ── inference thread ──────────────────────────────────────────────────────
    std::thread       inferThread_;
    std::atomic<bool> stopFlag_{false};

    mutable std::mutex resultMutex_;
    SignResult         latestResult_;

    // ── instance counter for unique appsink names ─────────────────────────────
    static std::atomic<int> sCounter;

    // ── construction / destruction ────────────────────────────────────────────

    SignDetectorImpl(uint32_t srcW, uint32_t srcH, const SignDetectorConfig& cfg)
        : config_(cfg), srcW_(srcW), srcH_(srcH)
    {
        if (config_.enginePath.empty())
            throw std::runtime_error("libsigndetector: enginePath is empty");

        try {
            loadEngine();
            allocBuffers();
            cudaCheck(cudaStreamCreate(&stream_), "cudaStreamCreate");
        } catch (...) {
            cleanup();
            throw;
        }
    }

    ~SignDetectorImpl() {
        stop();
        cleanup();
    }

    void cleanup() noexcept {
        cudaFree(dSrcBGR_);    dSrcBGR_  = nullptr;
        cudaFree(dInput_);     dInput_   = nullptr;
        cudaFree(dOutput_);    dOutput_  = nullptr;
        cudaFreeHost(hOutput_);hOutput_  = nullptr;
        if (stream_) { cudaStreamDestroy(stream_); stream_ = nullptr; }
        if (appsink_) { gst_object_unref(appsink_); appsink_ = nullptr; }
        delete ctx_;     ctx_     = nullptr;
        delete engine_;  engine_  = nullptr;
        delete runtime_; runtime_ = nullptr;
    }

    // ── engine loading ────────────────────────────────────────────────────────

    void loadEngine() {
        std::ifstream file(config_.enginePath, std::ios::binary | std::ios::ate);
        if (!file.is_open())
            throw std::runtime_error("libsigndetector: cannot open engine: "
                                     + config_.enginePath);
        const auto size = file.tellg();
        if (size <= 0)
            throw std::runtime_error("libsigndetector: engine file empty or not seekable: "
                                     + config_.enginePath);
        file.seekg(0);
        std::vector<char> data(static_cast<size_t>(size));
        if (!file.read(data.data(), size))
            throw std::runtime_error("libsigndetector: engine read failed: "
                                     + config_.enginePath);

        runtime_ = nvinfer1::createInferRuntime(logger_);
        if (!runtime_)
            throw std::runtime_error("libsigndetector: createInferRuntime failed");

        engine_ = runtime_->deserializeCudaEngine(data.data(), data.size());
        if (!engine_)
            throw std::runtime_error("libsigndetector: deserializeCudaEngine failed — "
                                     "build with trtexec on this Jetson module");

        ctx_ = engine_->createExecutionContext();
        if (!ctx_)
            throw std::runtime_error("libsigndetector: createExecutionContext failed");

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
            throw std::runtime_error("libsigndetector: engine must have one input "
                                     "and one output tensor");

        // Read output shape: expected [1, 4+numClasses, numAnchors].
        const auto outShape = engine_->getTensorShape(outputName_.c_str());
        if (outShape.nbDims != 3)
            throw std::runtime_error("libsigndetector: unexpected output tensor rank "
                                     "(expected 3: [batch, 4+classes, anchors])");
        const int expectedOutputs = 4 + config_.numClasses;
        if (outShape.d[1] != expectedOutputs)
            throw std::runtime_error("libsigndetector: output dim[1]="
                                     + std::to_string(outShape.d[1])
                                     + " does not match 4+numClasses="
                                     + std::to_string(expectedOutputs));
        numAnchors_ = outShape.d[2];
    }

    // ── buffer allocation ─────────────────────────────────────────────────────

    void allocBuffers() {
        srcBytes_ = static_cast<size_t>(srcW_) * srcH_ * 3;
        inBytes_  = static_cast<size_t>(config_.modelInputW)
                  * config_.modelInputH * 3 * sizeof(float);
        // Output: [1][4+numClasses][numAnchors]
        outBytes_ = static_cast<size_t>(4 + config_.numClasses)
                  * numAnchors_ * sizeof(float);

        cudaCheck(cudaMalloc(&dSrcBGR_, srcBytes_),  "cudaMalloc dSrcBGR");
        cudaCheck(cudaMalloc(&dInput_,  inBytes_),   "cudaMalloc dInput");
        cudaCheck(cudaMalloc(&dOutput_, outBytes_),  "cudaMalloc dOutput");
        cudaCheck(cudaMallocHost(reinterpret_cast<void**>(&hOutput_), outBytes_),
                  "cudaMallocHost hOutput");
    }

    // ── GStreamer bin creation ─────────────────────────────────────────────────

    GstElement* createBin() {
        // Release any previous appsink reference (from a prior createBin() call).
        if (appsink_) {
            gst_object_unref(appsink_);
            appsink_ = nullptr;
        }

        const int   id       = sCounter.fetch_add(1);
        const std::string sinkName = "signsink_" + std::to_string(id);

        // Build pipeline fragment:
        //   [gstConversion] ! video/x-raw,format=BGR ! appsink name=<sinkName>
        // gst_parse_bin_from_description wraps this into a bin with a ghost
        // sink pad at the unlinked start — exactly what addBranch() links to.
        const std::string desc = config_.gstConversion
            + " ! video/x-raw,format=BGR"
              " ! appsink name=" + sinkName
            + " drop=true max-buffers=1 emit-signals=false sync=false";

        GError*     err = nullptr;
        GstElement* bin = gst_parse_bin_from_description(desc.c_str(),
                                                         /*ghost_unlinked=*/TRUE,
                                                         &err);
        if (err || !bin) {
            if (err) {
                std::fprintf(stderr, "[libsigndetector] bin parse failed: %s\n",
                             err->message);
                g_error_free(err);
            }
            if (bin) gst_object_unref(bin);
            return nullptr;
        }

        // Grab a ref to the appsink so the inference thread can pull from it.
        appsink_ = gst_bin_get_by_name(GST_BIN(bin), sinkName.c_str());
        if (!appsink_) {
            std::fprintf(stderr, "[libsigndetector] appsink '%s' not found in bin\n",
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

            SignResult result = runInference(map.data);

            gst_buffer_unmap(buf, &map);
            gst_sample_unref(sample);

            {
                std::lock_guard<std::mutex> lk(resultMutex_);
                latestResult_ = std::move(result);
            }
        }
    }

    // ── inference pipeline ────────────────────────────────────────────────────

    SignResult runInference(const uint8_t* bgr) {
        // 1. Upload BGR to device.
        if (cudaMemcpyAsync(dSrcBGR_, bgr, srcBytes_,
                            cudaMemcpyHostToDevice, stream_) != cudaSuccess) {
            std::fprintf(stderr, "[libsigndetector] H2D memcpy failed\n");
            return {};
        }

        // 2. GPU resize + normalise.
        launchPreprocessKernel(
            static_cast<const uint8_t*>(dSrcBGR_),
            static_cast<float*>(dInput_),
            static_cast<int>(srcW_), static_cast<int>(srcH_),
            static_cast<int>(config_.modelInputW),
            static_cast<int>(config_.modelInputH),
            config_.meanR, config_.meanG, config_.meanB,
            config_.stdR,  config_.stdG,  config_.stdB,
            stream_);

        // 3. TRT inference.
        if (!ctx_->setTensorAddress(inputName_.c_str(),  dInput_) ||
            !ctx_->setTensorAddress(outputName_.c_str(), dOutput_)) {
            std::fprintf(stderr, "[libsigndetector] setTensorAddress failed\n");
            return {};
        }
        if (!ctx_->enqueueV3(stream_)) {
            std::fprintf(stderr, "[libsigndetector] enqueueV3 failed\n");
            return {};
        }

        // 4. Download output.
        if (cudaMemcpyAsync(hOutput_, dOutput_, outBytes_,
                            cudaMemcpyDeviceToHost, stream_) != cudaSuccess) {
            std::fprintf(stderr, "[libsigndetector] D2H memcpy failed\n");
            return {};
        }
        if (cudaStreamSynchronize(stream_) != cudaSuccess) {
            std::fprintf(stderr, "[libsigndetector] sync failed\n");
            return {};
        }

        return decode();
    }

    // ── postprocessing ────────────────────────────────────────────────────────
    // Output layout: [4+numClasses][numAnchors] (batch dim stripped).
    // YOLOv8: rows 0-3 = cx,cy,w,h in model-input pixel coords.
    //         rows 4..4+C-1 = class scores (sigmoid-activated in the export).

    SignResult decode() const {
        const int C = config_.numClasses;
        const int A = numAnchors_;
        const float scaleX = static_cast<float>(srcW_) / config_.modelInputW;
        const float scaleY = static_cast<float>(srcH_) / config_.modelInputH;

        // Collect candidates above confThreshold.
        std::vector<Detection> candidates;
        for (int i = 0; i < A; ++i) {
            const float cx = hOutput_[0 * A + i];
            const float cy = hOutput_[1 * A + i];
            const float w  = hOutput_[2 * A + i];
            const float h  = hOutput_[3 * A + i];

            float bestScore = 0.0f;
            int   bestClass = -1;
            for (int c = 0; c < C; ++c) {
                const float score = hOutput_[(4 + c) * A + i];
                if (score > bestScore) { bestScore = score; bestClass = c; }
            }
            if (bestScore < config_.confThreshold || bestClass < 0) continue;

            Detection d;
            d.classId    = static_cast<SignClass>(bestClass);
            d.confidence = bestScore;
            d.x1 = std::max(0.0f, (cx - w * 0.5f) * scaleX);
            d.y1 = std::max(0.0f, (cy - h * 0.5f) * scaleY);
            d.x2 = std::min(static_cast<float>(srcW_), (cx + w * 0.5f) * scaleX);
            d.y2 = std::min(static_cast<float>(srcH_), (cy + h * 0.5f) * scaleY);
            candidates.push_back(d);
        }

        // Per-class greedy NMS.
        SignResult result;
        for (int c = 0; c < C; ++c) {
            std::vector<Detection> cls;
            for (const auto& d : candidates)
                if (static_cast<int>(d.classId) == c) cls.push_back(d);

            std::sort(cls.begin(), cls.end(),
                      [](const Detection& a, const Detection& b) {
                          return a.confidence > b.confidence;
                      });

            std::vector<bool> suppressed(cls.size(), false);
            for (size_t i = 0; i < cls.size(); ++i) {
                if (suppressed[i]) continue;
                result.detections.push_back(cls[i]);
                for (size_t j = i + 1; j < cls.size(); ++j) {
                    if (!suppressed[j] && iou(cls[i], cls[j]) >= config_.nmsThreshold)
                        suppressed[j] = true;
                }
            }
        }

        return result;
    }

    // ── thread control ────────────────────────────────────────────────────────

    void start() {
        stopFlag_.store(false, std::memory_order_relaxed);
        inferThread_ = std::thread(&SignDetectorImpl::inferenceLoop, this);
    }

    void stop() {
        stopFlag_.store(true, std::memory_order_release);
        if (inferThread_.joinable()) inferThread_.join();
    }

    SignResult poll() const {
        std::lock_guard<std::mutex> lk(resultMutex_);
        return latestResult_;
    }
};

std::atomic<int> SignDetectorImpl::sCounter{0};

// ─── SignDetector public API ──────────────────────────────────────────────────

SignDetector::SignDetector(uint32_t srcWidth, uint32_t srcHeight,
                           const SignDetectorConfig& config)
    : impl_(std::make_unique<SignDetectorImpl>(srcWidth, srcHeight, config))
{}

SignDetector::~SignDetector() = default;

GstElement* SignDetector::createBin() { return impl_->createBin(); }
void        SignDetector::start()     { impl_->start(); }
void        SignDetector::stop()      { impl_->stop(); }
SignResult  SignDetector::poll() const { return impl_->poll(); }

} // namespace dashcam::sign
