#include "libcamera_gst.h"
#include <climits>
#include <cstdlib>

// ─── private helpers ────────────────────────────────────────────────────────

/// Destruction order matters for GStreamer reference counting:
///   1. Atomically claim pipeline_ via swap under stateMutex_ (re-entrancy guard).
///   2. Send EOS and wait — forces mp4mux to write the moov atom.
///   3. Set pipeline to NULL and wait for confirmation — joins the streaming thread.
///   4. Release tee request pads (gst_element_release_request_pad + unref).
///   5. Clear tracking vectors (branches_, teePads_, branchValves_).
///   6. Unref non-owning element handles (tee_, camera_src_, appsink_).
///   7. Unref the pipeline — releases all bin members.
///
/// The pipeline MUST reach NULL before pads are released.  Releasing request
/// pads while the streaming thread is still running (i.e. before NULL state)
/// is a GStreamer API violation and can cause the streaming thread to crash.
void Camera_GST::teardownPipeline() {
    // Atomically claim the pipeline pointer so a concurrent re-entrant call
    // (e.g. a future bus-watch callback racing a lifecycle stop) gets nullptr
    // and exits immediately rather than double-freeing.
    GstElement* pipe = nullptr;
    {
        std::lock_guard<std::mutex> lk(stateMutex_);
        std::swap(pipe, pipeline_);
    }

    if (pipe) {
        // timeout=0 returns the last *achieved* state, which is stale during an
        // async transition.  Check pending too: if the pipeline is mid-transition
        // toward PLAYING we must still send EOS so mp4mux writes its moov atom.
        GstState state, pending;
        gst_element_get_state(pipe, &state, &pending, 0);
        if (state == GST_STATE_PLAYING  || state == GST_STATE_PAUSED ||
            pending == GST_STATE_PLAYING || pending == GST_STATE_PAUSED) {

            gst_element_send_event(pipe, gst_event_new_eos());

            // Wait up to 5 s for EOS to propagate.  With a ±30 s recording
            // pre-buffer active, mp4mux must flush all buffered frames before
            // emitting EOS downstream — this can exceed 1.5 s on a CPU-encoder
            // path, so a generous window is required.
            GstBus* bus = gst_element_get_bus(pipe);
            if (bus) {
                GstMessage* msg = gst_bus_timed_pop_filtered(bus, 5 * GST_SECOND,
                    static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
                if (msg) gst_message_unref(msg);
                gst_object_unref(bus);
            }
        }

        // Transition to NULL joins the streaming thread.  nvarguscamerasrc
        // hardware teardown can be briefly async, so wait for confirmation
        // before releasing pads or unreffing elements.
        GstStateChangeReturn sc = gst_element_set_state(pipe, GST_STATE_NULL);
        if (sc == GST_STATE_CHANGE_ASYNC)
            gst_element_get_state(pipe, nullptr, nullptr, 5 * GST_SECOND);
    }

    GstElement* appsinkToUnref = nullptr;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        appsinkToUnref = appsink_;
        appsink_       = nullptr;
    }

    for (GstPad* pad : teePads_) {
        if (tee_) gst_element_release_request_pad(tee_, pad);
        gst_object_unref(pad);
    }
    teePads_.clear();
    branches_.clear();
    branchValves_.clear();

    if (tee_)          { gst_object_unref(tee_);          tee_        = nullptr; }
    if (camera_src_)   { gst_object_unref(camera_src_);   camera_src_ = nullptr; }
    if (appsinkToUnref){ gst_object_unref(appsinkToUnref);               }
    if (pipe)            gst_object_unref(pipe);
}

void Camera_GST::setPipelineError() {
    teardownPipeline();
    std::lock_guard<std::mutex> lock(stateMutex_);
    status_.status       = CAMERA_STATUS::ERROR;
    status_.currentError = pipelineError();
}

// ─── static utilities ────────────────────────────────────────────────────────

bool Camera_GST::safeStoi(const std::string& str, int& outVal) {
    if (str.empty()) return false;
    char* end = nullptr;
    long val = std::strtol(str.c_str(), &end, 10);
    if (end == str.c_str() || *end != '\0') return false;
    if (val < static_cast<long>(INT_MIN) || val > static_cast<long>(INT_MAX)) return false;
    outVal = static_cast<int>(val);
    return true;
}

void Camera_GST::computeFpsRational(float fps, uint32_t& frNum, uint32_t& frDen) {
    frNum = static_cast<uint32_t>(fps * 1000.0f + 0.5f);
    frDen = 1000;
    if (frNum == 0) { frNum = 1; frDen = 1; return; }
    uint32_t a = frNum, b = frDen;
    while (b) { uint32_t t = a % b; a = b; b = t; }
    frNum /= a;
    frDen /= a;
}

// ─── lifecycle ───────────────────────────────────────────────────────────────

Camera_GST::Camera_GST(const cameraInfo& camera)
    : info_(camera),
      status_({CAMERA_STATUS::CLOSED, 0, 0, 0, ERROR_CODE::NONE}),
      pipeline_(nullptr),
      camera_src_(nullptr),
      appsink_(nullptr),
      tee_(nullptr) {
}

Camera_GST::~Camera_GST() {
    if (isOpen()) close();
}

void Camera_GST::open() {
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (status_.status != CAMERA_STATUS::CLOSED) {
            status_.currentError = status_.currentError == ERROR_CODE::NONE
                ? ERROR_CODE::CAMERA_ALREADY_OPEN : status_.currentError;
            return;
        }
    }

    // gst_init_check() scans the GStreamer plugin registry on first call,
    // which can take hundreds of milliseconds.  It is internally thread-safe
    // and idempotent across processes, so we drop stateMutex_ to keep
    // getCameraStatus()/captureFrame() responsive on other threads.
    GError* err = nullptr;
    bool gstOk = gst_init_check(nullptr, nullptr, &err);
    if (err) g_error_free(err);

    std::lock_guard<std::mutex> lock(stateMutex_);
    if (!gstOk) {
        status_.status = CAMERA_STATUS::ERROR;
        status_.currentError = pipelineError();
        return;
    }
    status_.status = CAMERA_STATUS::OPEN;
    status_.currentError = ERROR_CODE::NONE;
}

void Camera_GST::close() {
    bool requiresStop = false;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (status_.status == CAMERA_STATUS::CLOSED) {
            status_.currentError = ERROR_CODE::CAMERA_ALREADY_CLOSED;
            return;
        }
        requiresStop = (status_.status == CAMERA_STATUS::RUNNING);
    }

    // Drop lock before hitting GStreamer teardown logic to prevent deadlocks
    if (requiresStop) {
        stop();  
    } else {
        teardownPipeline();  
    }

    // Re-acquire to finalize state
    std::lock_guard<std::mutex> lock(stateMutex_);
    status_.status = CAMERA_STATUS::CLOSED;
    status_.currentError = ERROR_CODE::NONE;
}

bool Camera_GST::isOpen() const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    return status_.status != CAMERA_STATUS::CLOSED;
}

// ─── iCamera interface ───────────────────────────────────────────────────────

void Camera_GST::getCameraInfo(cameraInfo& info) const { info = info_; }

void Camera_GST::setCameraAttribute(const std::string& name, const std::string& value) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (status_.status != CAMERA_STATUS::RUNNING) {
        pendingAttributes_[name] = value;
        status_.currentError = ERROR_CODE::NONE;
    } else {
        applyAttributeGStreamer(name, value);
    }
}

void Camera_GST::setCameraVideoFormat(uint16_t formatIndex) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (formatIndex >= info_.videoFormats.size()) {
        status_.currentError = ERROR_CODE::UNSUPPORTED_FORMAT;
        return;
    }
    status_.currentFormatIndex = formatIndex;
}

void Camera_GST::getCameraStatus(cameraStatus& status) const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    status = status_;
}

void Camera_GST::start() {
    uint16_t formatIndex = 0;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (status_.status == CAMERA_STATUS::RUNNING) {
            status_.currentError = ERROR_CODE::CAMERA_ALREADY_RUNNING;
            return;
        }
        if (status_.status != CAMERA_STATUS::OPEN) return;
        if (info_.videoFormats.empty() || status_.currentFormatIndex >= info_.videoFormats.size()) {
            status_.status = CAMERA_STATUS::ERROR;
            status_.currentError = ERROR_CODE::UNSUPPORTED_FORMAT;
            return;
        }
        formatIndex = status_.currentFormatIndex;
        status_.status = CAMERA_STATUS::RUNNING;  // Optimistic claim; Phase 2 proceeds, or setPipelineError() reverts.
    }

    const auto& fmt = info_.videoFormats[formatIndex];

    uint32_t frNum, frDen;
    computeFpsRational(fmt.frameRate, frNum, frDen);

    std::string pipelineStr = buildPipelineString(fmt, frNum, frDen);

    GError* error = nullptr;
    pipeline_ = gst_parse_launch(pipelineStr.c_str(), &error);
    if (error != nullptr || pipeline_ == nullptr) {
        if (error) g_error_free(error);
        if (pipeline_) { gst_object_unref(pipeline_); pipeline_ = nullptr; }
        setPipelineError();
        return;
    }

    appsink_    = gst_bin_get_by_name(GST_BIN(pipeline_), "mysink");
    camera_src_ = gst_bin_get_by_name(GST_BIN(pipeline_), "camerasrc");
    tee_        = gst_bin_get_by_name(GST_BIN(pipeline_), "srctee");

    if (!appsink_ || !camera_src_ || !tee_) {
        setPipelineError();
        return;
    }

    // Link each registered branch to the tee.
    // Layout per branch:  tee ! queue ! valve ! <branchBin>
    // The queue isolates backpressure; the valve enables runtime enable/disable.
    bool branchError = false;
    for (size_t i = 0; i < branches_.size(); ++i) {
        auto& [branchName, branchBin, branchLeaky, branchInitEnabled] = branches_[i];

        auto releaseRemaining = [&](size_t from) {
            for (size_t j = from; j < branches_.size(); ++j) {
                gst_object_ref_sink(branches_[j].bin);
                gst_object_unref(branches_[j].bin);
            }
        };

        GstElement* queue = gst_element_factory_make("queue", nullptr);
        if (!queue) {
            gst_object_ref_sink(branchBin); gst_object_unref(branchBin);
            releaseRemaining(i + 1);
            branchError = true;
            break;
        }

        if (branchLeaky) {
            g_object_set(G_OBJECT(queue),
                "max-size-buffers", (guint)2,
                "max-size-bytes",   (guint)0,
                "max-size-time",    (guint64)0,
                "leaky",            (gint)2,   // GST_QUEUE_LEAK_DOWNSTREAM
                NULL);
        }

        GstElement* valve = gst_element_factory_make("valve", nullptr);
        if (!valve) {
            gst_object_ref_sink(queue);     gst_object_unref(queue);
            gst_object_ref_sink(branchBin); gst_object_unref(branchBin);
            releaseRemaining(i + 1);
            branchError = true;
            break;
        }
        if (!branchInitEnabled)
            g_object_set(G_OBJECT(valve), "drop", TRUE, NULL);

        gst_bin_add_many(GST_BIN(pipeline_), queue, valve, branchBin, nullptr);

        GstPad* teeSrcPad = gst_element_request_pad_simple(tee_, "src_%u");
        if (!teeSrcPad) {
            releaseRemaining(i + 1);
            branchError = true;
            break;
        }

        GstPad* queueSinkPad = gst_element_get_static_pad(queue, "sink");
        GstPadLinkReturn ret = gst_pad_link(teeSrcPad, queueSinkPad);
        gst_object_unref(queueSinkPad);

        if (ret != GST_PAD_LINK_OK ||
            !gst_element_link(queue, valve) ||
            !gst_element_link(valve, branchBin)) {
            gst_element_release_request_pad(tee_, teeSrcPad);
            gst_object_unref(teeSrcPad);
            releaseRemaining(i + 1);
            branchError = true;
            break;
        }
        teePads_.push_back(teeSrcPad);
        branchValves_[branchName] = valve;
    }

    if (branchError) {
        setPipelineError();
        return;
    }

    // Pre-start flush: apply pending attributes while the pipeline is in NULL state.
    // nvarguscamerasrc reads GObject properties during the PAUSED→PLAYING ISP
    // bringup, so frame 0 uses the requested exposure/gain/lock with no AE flash.
    // Holding stateMutex_ here is safe: no streaming thread exists yet, so
    // g_object_set is just writing struct fields — no blocking, no callbacks.
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        for (const auto& [attrName, attrValue] : pendingAttributes_) {
            applyAttributeGStreamer(attrName, attrValue);
        }
        pendingAttributes_.clear();
    }

    // Hardware boots here with the correct ISP settings already applied.
    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        setPipelineError();
        return;
    }
    if (ret == GST_STATE_CHANGE_ASYNC) {
        // Block until Argus/V4L2 hardware fully initialises (up to 5 s).
        // Any setCameraAttribute() calls during this window are safely queued
        // into pendingAttributes_ because status is still OPEN.
        GstState state, pending;
        ret = gst_element_get_state(pipeline_, &state, &pending, 5 * GST_SECOND);
        if (ret == GST_STATE_CHANGE_FAILURE || ret == GST_STATE_CHANGE_ASYNC) {
            setPipelineError();
            return;
        }
    }

    // Drain anything queued during the startup window.  status_ is already
    // RUNNING (claimed at the top of start() to serialise concurrent callers).
    std::map<std::string, std::string> lateAttribs;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        lateAttribs = std::move(pendingAttributes_);
        for (const auto& [attrName, attrValue] : lateAttribs) {
            applyAttributeGStreamer(attrName, attrValue);
        }
    }
}

void Camera_GST::stop() {
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (status_.status != CAMERA_STATUS::RUNNING) return;
        status_.status = CAMERA_STATUS::OPEN;
    }
    teardownPipeline();
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        status_.freeBufferCount = 0;
    }
}

void Camera_GST::captureFrame(uint8_t* buffer, uint32_t bufferSize, uint32_t& bytesWritten) {
    bytesWritten = 0;

    // Briefly lock to check state and take a GStreamer ref on the appsink.
    // This prevents a use-after-free if stop() tears down the pipeline while
    // we're blocking inside try_pull_sample.
    GstElement* sinkRef = nullptr;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (status_.status != CAMERA_STATUS::RUNNING || !appsink_) return;
        sinkRef = static_cast<GstElement*>(gst_object_ref(appsink_));
    }

    // Block for up to 1 s without holding the lock.  If stop() is called
    // concurrently, the pipeline transitions to NULL which forces appsink into
    // flushing state and try_pull_sample returns nullptr immediately.
    GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sinkRef), GST_SECOND);
    gst_object_unref(sinkRef);

    if (!sample) return;

    GstBuffer* gstBuffer = gst_sample_get_buffer(sample);
    if (gstBuffer) {
        gsize dataSize = gst_buffer_get_size(gstBuffer);
        if (dataSize <= bufferSize) {
            gst_buffer_extract(gstBuffer, 0, buffer, dataSize);
            bytesWritten = static_cast<uint32_t>(dataSize);
            std::lock_guard<std::mutex> lock(stateMutex_);
            status_.frameCount++;
        }
    }
    gst_sample_unref(sample);
}

// ─── multi-sink extensions ───────────────────────────────────────────────────

void Camera_GST::addBranch(const std::string& name, GstElement* sinkBin,
                           bool leaky, bool initialEnabled) {  // defaults in header
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (status_.status == CAMERA_STATUS::RUNNING) {
        status_.currentError = ERROR_CODE::INVALID_ATTRIBUTE;
        return;
    }
    branches_.push_back({name, sinkBin, leaky, initialEnabled});
}

GstElement* Camera_GST::getTee() const { return tee_; }

// Valve drop=true discards buffers without flushing — timestamps remain
// continuous in the recording pipeline, so re-enabling mid-stream works cleanly.
void Camera_GST::setBranchEnabled(const std::string& name, bool enabled) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    auto it = branchValves_.find(name);
    if (it == branchValves_.end()) {
        status_.currentError = ERROR_CODE::INVALID_ATTRIBUTE;
        return;
    }
    g_object_set(G_OBJECT(it->second), "drop", enabled ? FALSE : TRUE, NULL);
    status_.currentError = ERROR_CODE::NONE;
}
