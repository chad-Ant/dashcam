#include "libcamera_gst.h"
#include <gst/video/video.h>
#include <algorithm>
#include <cctype>
#include <climits>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// ─── file-local log helper ────────────────────────────────────────────────────

namespace {

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

} // namespace

namespace dashcam::camera {

// ─── private helpers ────────────────────────────────────────────────────────

/// Destruction order matters for GStreamer reference counting:
///   1. Atomically claim pipeline_ via swap under stateMutex_ (re-entrancy guard).
///   2. Send EOS, then drain the appsink while waiting for the pipeline EOS
///      (or replacing error) on the bus — forces the muxer to finalise the
///      container.  See the combined loop below for why draining is mandatory.
///   3. Set pipeline to NULL and wait for confirmation — joins the streaming thread.
///      NOTE: on CSI, nvarguscamerasrc's PAUSED→READY can itself block ~5 s when
///      the Argus session ends in the CANCELLED path (daemon-state dependent);
///      that wait is internal to the NVIDIA element and bounded by its own
///      timeout — not something this code can shorten.
///   4. Release tee request pads (gst_element_release_request_pad + unref).
///   5. Free orphaned branch bins (registered but never added to the pipeline);
///      clear tracking vectors.
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
    GstElement* sinkRef = nullptr;
    {
        std::lock_guard<std::mutex> lk(stateMutex_);
        std::swap(pipe, pipeline_);
        if (pipe && appsink_)
            sinkRef = static_cast<GstElement*>(gst_object_ref(appsink_));
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

            // Wait for the pipeline EOS (or the error that replaces it) while
            // simultaneously draining the appsink, all under one eosTimeoutMs
            // deadline.  Two platform behaviours force the combined loop:
            //  - GstAppSink defers its EOS (and therefore the pipeline EOS
            //    message) until the application has pulled every queued sample.
            //    The captureFrame() consumer has stopped by now, so teardown
            //    must pull the trailing frames itself, and must keep pulling
            //    until the appsink reports EOS — frames still in flight behind
            //    an empty queue would otherwise re-block the EOS handler.
            //  - nvarguscamerasrc sometimes posts an ERROR (Argus CANCELLED)
            //    instead of forwarding EOS at all; a drain-then-wait sequence
            //    would burn the full drain budget before seeing that error.
            // The EOS window stays generous on purpose: with a ±30 s recording
            // pre-buffer, matroskamux can need >1.5 s to flush on a CPU-encoder
            // path.  Bounded: every iteration waits <= 50 ms or pulls a sample
            // (finite after EOS), and the deadline caps the whole loop.
            GstBus* bus = gst_element_get_bus(pipe);
            if (bus) {
                const gint64 deadline = g_get_monotonic_time()
                    + static_cast<gint64>(params_.eosTimeoutMs) * G_TIME_SPAN_MILLISECOND;
                const GstMessageType eosMask =
                    static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR);
                GstMessage* msg = nullptr;
                while (!msg && g_get_monotonic_time() < deadline) {
                    const bool sinkDone = !sinkRef ||
                        gst_app_sink_is_eos(GST_APP_SINK(sinkRef));
                    if (!sinkDone) {
                        GstSample* s = gst_app_sink_try_pull_sample(
                            GST_APP_SINK(sinkRef), 25 * GST_MSECOND);
                        if (s) gst_sample_unref(s);
                    }
                    // Block on the bus only once the appsink is fully drained;
                    // until then just poll so the drain keeps making progress.
                    msg = gst_bus_timed_pop_filtered(
                        bus, sinkDone ? 50 * GST_MSECOND : 0, eosMask);
                }
                if (msg) gst_message_unref(msg);
                gst_object_unref(bus);
            }
        }

        // Transition to NULL joins the streaming thread.  nvarguscamerasrc
        // hardware teardown can be briefly async, so wait for confirmation
        // before releasing pads or unreffing elements.
        GstStateChangeReturn sc = gst_element_set_state(pipe, GST_STATE_NULL);
        if (sc == GST_STATE_CHANGE_ASYNC)
            gst_element_get_state(pipe, nullptr, nullptr,
                                  params_.stateChangeTimeoutMs * GST_MSECOND);
    }
    if (sinkRef) gst_object_unref(sinkRef);

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

    // Free branch bins that were registered but never added to a pipeline
    // (e.g., addBranch() then close() without start(), or a failed early start()).
    // Bins that were gst_bin_add_many()'d have the pipeline as their parent and
    // will be freed by gst_object_unref(pipe) below; check parent to distinguish.
    for (auto& br : branches_) {
        if (br.bin) {
            GstObject* parent = gst_object_get_parent(GST_OBJECT(br.bin));
            if (parent) {
                gst_object_unref(parent);
            } else {
                gst_object_ref_sink(br.bin);
                gst_object_unref(br.bin);
            }
        }
    }
    branches_.clear();
    branchValves_.clear();

    // Null the element handles under the lock (consistent with appsink_ above),
    // then unref outside it.  Other threads read these pointers only under
    // stateMutex_, so the write must be locked too.
    GstElement* teeToUnref = nullptr;
    GstElement* srcToUnref = nullptr;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        teeToUnref = tee_;        tee_        = nullptr;
        srcToUnref = camera_src_; camera_src_ = nullptr;
    }
    if (teeToUnref)     gst_object_unref(teeToUnref);
    if (srcToUnref)     gst_object_unref(srcToUnref);
    if (appsinkToUnref) gst_object_unref(appsinkToUnref);
    if (pipe)           gst_object_unref(pipe);
}

void Camera_GST::setPipelineError() {
    doLog(log_, dashcam::log::LogLevel::ERROR,
          "pipeline error on %s", info_.address.c_str());
    teardownPipeline();
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        status_.status       = CAMERA_STATUS::ERROR;
        status_.currentError = pipelineError();
        starting_ = false;  // construction phase over (failed); unblock stop()
    }
    startCv_.notify_all();
}

void Camera_GST::checkBusErrors() {
    // Take a ref on the pipeline under the lock so a concurrent teardown can't
    // free it while we poll its bus.
    GstElement* pipeRef = nullptr;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (status_.status != CAMERA_STATUS::RUNNING || !pipeline_) return;
        pipeRef = static_cast<GstElement*>(gst_object_ref(pipeline_));
    }

    GstBus* bus = gst_element_get_bus(pipeRef);
    bool sawError = false;
    if (bus) {
        // Drain every message except EOS (which teardownPipeline() waits for) so
        // the bus queue can't grow unbounded over a long capture.  Report the
        // first error encountered.
        const GstMessageType drainMask =
            static_cast<GstMessageType>(GST_MESSAGE_ANY & ~GST_MESSAGE_EOS);
        GstMessage* msg;
        while ((msg = gst_bus_pop_filtered(bus, drainMask)) != nullptr) {
            if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR && !sawError) {
                GError* err = nullptr;
                gchar*  dbg = nullptr;
                gst_message_parse_error(msg, &err, &dbg);
                doLog(log_, dashcam::log::LogLevel::ERROR,
                      "pipeline bus error on %s: %s", info_.address.c_str(),
                      err ? err->message : "unknown");
                if (err) g_error_free(err);
                g_free(dbg);
                sawError = true;
            }
            gst_message_unref(msg);
        }
        gst_object_unref(bus);
    }
    gst_object_unref(pipeRef);

    if (sawError) {
        // Flip to ERROR; the app observes this via getCameraStatus() and calls
        // close() to release the pipeline.  Leave teardown to that path so we
        // don't tear down a pipeline captureFrame() may still be reffing.
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (status_.status == CAMERA_STATUS::RUNNING) {
            status_.status       = CAMERA_STATUS::ERROR;
            status_.currentError = pipelineError();
        }
    }
}

void Camera_GST::applyAttributeGStreamer(const std::string& name,
                                         const std::string& value) {
    if (!camera_src_) {
        status_.currentError = pipelineError();
        return;
    }
    const AttributeEntry* entry = dict_.resolve(name, cameraTypeTag());
    if (!entry || !applyGstProperty(camera_src_, *entry, value)) {
        status_.currentError = ERROR_CODE::INVALID_ATTRIBUTE;
        return;
    }
    status_.currentError = ERROR_CODE::NONE;
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

void Camera_GST::setAttributeDictionary(const dashcam::camera::AttributeDictionary& dict) {
    dict_ = dict;
}

void Camera_GST::setLogCallback(dashcam::log::LogCallback cb) {
    log_ = std::move(cb);
}

void Camera_GST::setPipelineParams(const PipelineParams& p) {
    // Write-once before start(); read by buildPipelineString() and the
    // state-change waits.  No lock needed as long as the documented ordering
    // (set before start) is honoured.
    params_ = p;
}

void Camera_GST::setOutputResolution(uint32_t width, uint32_t height, float fps) {
    // Write-once before start(); read by buildPipelineString().  Same ordering
    // contract as setPipelineParams(), so no lock is required.
    outWidth_  = width;
    outHeight_ = height;
    outFps_    = fps;
}

bool Camera_GST::applyGstProperty(GstElement* src,
                                   const AttributeEntry& entry,
                                   const std::string& value) {
    const char* prop = entry.gstProperty.c_str();

    switch (entry.valueType) {
        case AttributeValueType::String:
            g_object_set(G_OBJECT(src), prop, value.c_str(), NULL);
            return true;

        case AttributeValueType::Int: {
            int iv = 0;
            if (!safeStoi(value, iv)) return false;
            g_object_set(G_OBJECT(src), prop, static_cast<gint>(iv), NULL);
            return true;
        }

        case AttributeValueType::Float: {
            char* end = nullptr;
            float fv = std::strtof(value.c_str(), &end);
            // Reject empty input and trailing garbage (e.g. "1.5abc"), matching
            // the strictness of the Int path via safeStoi().
            if (end == value.c_str() || *end != '\0') return false;
            g_object_set(G_OBJECT(src), prop, static_cast<gfloat>(fv), NULL);
            return true;
        }

        case AttributeValueType::Bool: {
            std::string lower = value;
            std::transform(lower.begin(), lower.end(), lower.begin(),
                           [](unsigned char c){ return std::tolower(c); });
            gboolean bv;
            if      (lower == "true"  || lower == "1") bv = TRUE;
            else if (lower == "false" || lower == "0") bv = FALSE;
            else return false;
            g_object_set(G_OBJECT(src), prop, bv, NULL);
            return true;
        }

        case AttributeValueType::BoolFromZero: {
            int iv = 0;
            if (!safeStoi(value, iv)) return false;
            g_object_set(G_OBJECT(src), prop, iv == 0 ? TRUE : FALSE, NULL);
            return true;
        }

        case AttributeValueType::RangeString: {
            std::string range = value + " " + value;
            g_object_set(G_OBJECT(src), prop, range.c_str(), NULL);
            return true;
        }
    }
    return false;
}

void Camera_GST::computeFpsRational(float fps, uint32_t& frNum, uint32_t& frDen) {
    // Casting a negative (or NaN) float to uint32_t is undefined behaviour;
    // the !(x > 0) form also catches NaN.  Fall back to 1/1 like the zero case.
    if (!(fps > 0.0f)) { frNum = 1; frDen = 1; return; }
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
    if (isOpen()) {
        close();
    } else {
        // Branch bins registered while CLOSED (addBranch() before open(), or
        // after close()) were never adopted by a pipeline and would leak;
        // teardownPipeline() frees exactly those orphans and is a no-op for
        // everything else in this state.
        teardownPipeline();
    }
}

void Camera_GST::open() {
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (status_.status != CAMERA_STATUS::CLOSED) {
            status_.currentError = ERROR_CODE::CAMERA_ALREADY_OPEN;
            return;
        }
    }

    // gst_init_check() scans the GStreamer plugin registry on first call,
    // which can take hundreds of milliseconds.  It is internally thread-safe
    // and idempotent across processes, so we drop stateMutex_ to keep
    // getCameraStatus()/captureFrame() responsive on other threads.
    GError* err = nullptr;
    bool gstOk = gst_init_check(nullptr, nullptr, &err);
    if (!gstOk) {
        doLog(log_, dashcam::log::LogLevel::ERROR,
              "gst_init failed for %s: %s", info_.address.c_str(),
              err ? err->message : "unknown");
    }
    if (err) g_error_free(err);

    std::lock_guard<std::mutex> lock(stateMutex_);
    if (!gstOk) {
        status_.status = CAMERA_STATUS::ERROR;
        status_.currentError = pipelineError();
        return;
    }
    status_.status = CAMERA_STATUS::OPEN;
    status_.currentError = ERROR_CODE::NONE;
    doLog(log_, dashcam::log::LogLevel::INFO,
          "camera opened: %s", info_.address.c_str());
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
    doLog(log_, dashcam::log::LogLevel::INFO,
          "camera closing: %s", info_.address.c_str());

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
    if (status_.status != CAMERA_STATUS::RUNNING || !camera_src_) {
        // Queue in three cases: not yet started; start() is mid-construction
        // (RUNNING claimed before camera_src_ is set); or ERROR state recovery.
        // The late-attributes drain at the end of start() flushes these once
        // camera_src_ is valid.
        pendingAttributes_[name] = value;
        // Preserve the causal error code in ERROR state: overwriting it with
        // NONE would leave status polling showing status=ERROR, error=NONE,
        // masking why the camera failed.
        if (status_.status != CAMERA_STATUS::ERROR)
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
    status_.currentError = ERROR_CODE::NONE;  // success clears any prior error
}

void Camera_GST::getCameraStatus(cameraStatus& status) const {
    // Surface runtime pipeline failures here too, not only in captureFrame():
    // a camera used purely for branch recording has no captureFrame() consumer,
    // so without this poll a dead pipeline would keep reporting RUNNING forever.
    // Logically const (drains an internal message queue); checkBusErrors()
    // acquires stateMutex_ internally, so it must run before the lock below.
    const_cast<Camera_GST*>(this)->checkBusErrors();
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
        if (status_.status != CAMERA_STATUS::OPEN) {
            // CLOSED (never opened) or ERROR: refuse loudly — a silent return
            // would leave a stale currentError (possibly NONE) and the caller,
            // following the "check currentError after every call" contract,
            // would believe start() succeeded.
            status_.currentError = ERROR_CODE::CAMERA_NOT_OPEN;
            return;
        }
        if (info_.videoFormats.empty() || status_.currentFormatIndex >= info_.videoFormats.size()) {
            status_.status = CAMERA_STATUS::ERROR;
            status_.currentError = ERROR_CODE::UNSUPPORTED_FORMAT;
            return;
        }
        formatIndex = status_.currentFormatIndex;
        status_.status = CAMERA_STATUS::RUNNING;  // Optimistic claim; Phase 2 proceeds, or setPipelineError() reverts.
        starting_ = true;  // Cleared (with startCv_ notify) on every exit path:
                           // setPipelineError() for failures, end of start() on success.
        // Clear any stale error (e.g. a prior out-of-range setCameraVideoFormat)
        // so a successful start() reports NONE.  Genuine failures below override
        // this: setPipelineError() sets ERROR, and the attribute flushes set
        // INVALID_ATTRIBUTE — both run after this point.
        status_.currentError = ERROR_CODE::NONE;
    }

    const auto& fmt = info_.videoFormats[formatIndex];

    uint32_t frNum, frDen;
    computeFpsRational(fmt.frameRate, frNum, frDen);

    std::string pipelineStr = buildPipelineString(fmt, frNum, frDen);
    doLog(log_, dashcam::log::LogLevel::INFO,
          "starting pipeline on %s", info_.address.c_str());
    doLog(log_, dashcam::log::LogLevel::DEBUG,
          "pipeline: %s", pipelineStr.c_str());

    // Publish the shared handles under stateMutex_ so status (already RUNNING) and
    // the pointers become visible together to the concurrent captureFrame() reader,
    // instead of being assigned bare while another thread reads them under the lock.
    // pipeline_ is published first so setPipelineError() → teardownPipeline() can
    // release it (and any orphaned branch bins) on every failure path below.
    GError* error = nullptr;
    GstElement* pipeline = gst_parse_launch(pipelineStr.c_str(), &error);
    if (error != nullptr || pipeline == nullptr) {
        if (error) g_error_free(error);
        if (pipeline) gst_object_unref(pipeline);
        setPipelineError();   // pipeline_ still null; teardown frees orphan branches, sets ERROR
        return;
    }
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        pipeline_ = pipeline;
    }

    GstElement* appsink   = gst_bin_get_by_name(GST_BIN(pipeline), "mysink");
    GstElement* cameraSrc = gst_bin_get_by_name(GST_BIN(pipeline), "camerasrc");
    GstElement* tee       = gst_bin_get_by_name(GST_BIN(pipeline), "srctee");

    if (!appsink || !cameraSrc || !tee) {
        doLog(log_, dashcam::log::LogLevel::ERROR,
              "required pipeline elements not found on %s", info_.address.c_str());
        if (appsink)   gst_object_unref(appsink);
        if (cameraSrc) gst_object_unref(cameraSrc);
        if (tee)       gst_object_unref(tee);
        setPipelineError();   // tears down the published pipeline_ and orphan branches
        return;
    }

    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        appsink_    = appsink;
        camera_src_ = cameraSrc;
        tee_        = tee;
    }

    // Link each registered branch to the tee.
    // Layout per branch:  tee ! queue ! valve ! <branchBin>
    // The queue isolates backpressure; the valve enables runtime enable/disable.
    // Valves are collected locally and published to branchValves_ under the lock
    // after the loop, so a concurrent setBranchEnabled() never reads the map
    // mid-insertion.
    std::map<std::string, GstElement*> localValves;
    bool branchError = false;
    for (size_t i = 0; i < branches_.size(); ++i) {
        auto& [branchName, branchBin, branchLeaky, branchInitEnabled] = branches_[i];

        auto releaseRemaining = [&](size_t from) {
            for (size_t j = from; j < branches_.size(); ++j) {
                gst_object_ref_sink(branches_[j].bin);
                gst_object_unref(branches_[j].bin);
                branches_[j].bin = nullptr;  // prevent double-free in teardownPipeline
            }
        };

        GstElement* queue = gst_element_factory_make("queue", nullptr);
        if (!queue) {
            doLog(log_, dashcam::log::LogLevel::ERROR,
                  "failed to create queue element for branch '%s'", branchName.c_str());
            gst_object_ref_sink(branchBin); gst_object_unref(branchBin);
            branchBin = nullptr;
            releaseRemaining(i + 1);
            branchError = true;
            break;
        }

        if (branchLeaky) {
            g_object_set(G_OBJECT(queue),
                "max-size-buffers", (guint)params_.branchQueueDepth,
                "max-size-bytes",   (guint)0,
                "max-size-time",    (guint64)0,
                "leaky",            (gint)2,   // GST_QUEUE_LEAK_DOWNSTREAM
                NULL);
        }

        GstElement* valve = gst_element_factory_make("valve", nullptr);
        if (!valve) {
            doLog(log_, dashcam::log::LogLevel::ERROR,
                  "failed to create valve element for branch '%s'", branchName.c_str());
            gst_object_ref_sink(queue);     gst_object_unref(queue);
            gst_object_ref_sink(branchBin); gst_object_unref(branchBin);
            branchBin = nullptr;
            releaseRemaining(i + 1);
            branchError = true;
            break;
        }
        // forward-sticky-events: a dropping valve must still pass EOS (sticky),
        // otherwise a branch that is disabled at stop() time never delivers EOS
        // to its sink — the muxer never finalises its file and teardownPipeline()
        // waits out the full EOS timeout.  drop-mode is only changeable in
        // NULL/READY, so it must be set here rather than at enable/disable time.
        g_object_set(G_OBJECT(valve), "drop-mode", 1 /* forward-sticky-events */, NULL);
        if (!branchInitEnabled)
            g_object_set(G_OBJECT(valve), "drop", TRUE, NULL);

        gst_bin_add_many(GST_BIN(pipeline_), queue, valve, branchBin, nullptr);

        GstPad* teeSrcPad = gst_element_request_pad_simple(tee_, "src_%u");
        if (!teeSrcPad) {
            doLog(log_, dashcam::log::LogLevel::ERROR,
                  "failed to get tee src pad for branch '%s'", branchName.c_str());
            releaseRemaining(i + 1);
            branchError = true;
            break;
        }

        GstPad* queueSinkPad = gst_element_get_static_pad(queue, "sink");
        if (!queueSinkPad) {
            doLog(log_, dashcam::log::LogLevel::ERROR,
                  "failed to get queue sink pad for branch '%s'", branchName.c_str());
            gst_element_release_request_pad(tee_, teeSrcPad);
            gst_object_unref(teeSrcPad);
            releaseRemaining(i + 1);
            branchError = true;
            break;
        }
        GstPadLinkReturn ret = gst_pad_link(teeSrcPad, queueSinkPad);
        gst_object_unref(queueSinkPad);

        // Link the branch in dataflow order: tee→queue (above), then queue→valve,
        // then valve→branch.  queue→valve MUST be linked before valve→branch: once
        // valve→branch is established the valve becomes caps-transparent to the
        // branch's (system-only) sink, which would then make a strict queue→valve
        // link fail against the NVMM tee source.
        bool queueValveLinked = gst_element_link(queue, valve);

        // valve→branch uses a relaxed pad link that skips the premature query-caps
        // intersection.  A pre-built branch bin whose head is nvvidconv feeding a
        // system-memory consumer (e.g. the cairo recorder in librecord) reports a
        // *system-only* sink in NULL state — the fixed system-memory output
        // back-propagates through the bin — so a strict gst_element_link() refuses
        // an NVMM (CSI) tee source with NOFORMAT even though nvvidconv negotiates
        // NVMM→system fine once PLAYING.  Hierarchy and template caps are still
        // enforced; any real negotiation failure surfaces on the bus via
        // checkBusErrors().  Fall back to a strict link if the branch does not
        // expose a conventionally named "sink" ghost pad.
        bool valveLinked = false;
        if (queueValveLinked) {
            GstPad* branchSink = gst_element_get_static_pad(branchBin, "sink");
            if (branchSink) {
                GstPad* valveSrc = gst_element_get_static_pad(valve, "src");
                valveLinked = valveSrc &&
                    gst_pad_link_full(valveSrc, branchSink,
                        static_cast<GstPadLinkCheck>(GST_PAD_LINK_CHECK_HIERARCHY |
                                                     GST_PAD_LINK_CHECK_TEMPLATE_CAPS))
                        == GST_PAD_LINK_OK;
                if (valveSrc) gst_object_unref(valveSrc);
                gst_object_unref(branchSink);
            } else {
                valveLinked = gst_element_link(valve, branchBin);
            }
        }

        if (ret != GST_PAD_LINK_OK || !queueValveLinked || !valveLinked) {
            doLog(log_, dashcam::log::LogLevel::ERROR,
                  "pad/element link failed for branch '%s'", branchName.c_str());
            gst_element_release_request_pad(tee_, teeSrcPad);
            gst_object_unref(teeSrcPad);
            releaseRemaining(i + 1);
            branchError = true;
            break;
        }
        doLog(log_, dashcam::log::LogLevel::INFO,
              "branch linked: %s", branchName.c_str());
        teePads_.push_back(teeSrcPad);
        localValves[branchName] = valve;
    }

    if (branchError) {
        setPipelineError();
        return;
    }

    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        branchValves_ = std::move(localValves);
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
        // status_ is RUNNING at this point (claimed optimistically above).
        // With the camera_src_ null-guard in setCameraAttribute(), any concurrent
        // attribute writes during this window are safely queued into
        // pendingAttributes_ and drained by the late-attributes flush below.
        GstState state, pending;
        ret = gst_element_get_state(pipeline_, &state, &pending,
                                    params_.stateChangeTimeoutMs * GST_MSECOND);
        if (ret == GST_STATE_CHANGE_FAILURE || ret == GST_STATE_CHANGE_ASYNC) {
            setPipelineError();
            return;
        }
    }

    doLog(log_, dashcam::log::LogLevel::INFO,
          "pipeline running: %s", info_.address.c_str());

    // Drain anything queued during the startup window.  status_ is already
    // RUNNING (claimed at the top of start() to serialise concurrent callers).
    std::map<std::string, std::string> lateAttribs;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        lateAttribs = std::move(pendingAttributes_);
        for (const auto& [attrName, attrValue] : lateAttribs) {
            applyAttributeGStreamer(attrName, attrValue);
        }
        starting_ = false;  // construction phase over (success); unblock stop()
    }
    startCv_.notify_all();
}

void Camera_GST::stop() {
    {
        std::unique_lock<std::mutex> lock(stateMutex_);
        // A concurrent start() may still be constructing the pipeline outside
        // the lock (optimistic RUNNING claim).  Tearing down now would free a
        // half-built pipeline start() is still using.  start() always finishes
        // (bounded by its state-change timeouts), so this wait terminates.
        startCv_.wait(lock, [this] { return !starting_; });
        if (status_.status != CAMERA_STATUS::RUNNING) return;
        status_.status = CAMERA_STATUS::OPEN;
    }
    doLog(log_, dashcam::log::LogLevel::INFO,
          "stopping pipeline: %s", info_.address.c_str());
    teardownPipeline();
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        status_.freeBufferCount = 0;
    }
}

void Camera_GST::captureFrame(uint8_t* buffer, uint32_t bufferSize, uint32_t& bytesWritten) {
    bytesWritten = 0;

    // Surface any runtime pipeline failure; may transition RUNNING → ERROR, in
    // which case the state check below returns without pulling a sample.
    checkBusErrors();

    // Briefly lock to check state and take a GStreamer ref on the appsink.
    // This prevents a use-after-free if stop() tears down the pipeline while
    // we're blocking inside try_pull_sample.
    GstElement* sinkRef = nullptr;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (status_.status != CAMERA_STATUS::RUNNING || !appsink_) return;
        sinkRef = static_cast<GstElement*>(gst_object_ref(appsink_));
    }

    // Block for up to params_.captureTimeoutMs without holding the lock.  If
    // stop() is called concurrently, the pipeline transitions to NULL which
    // forces appsink into flushing state and try_pull_sample returns nullptr
    // immediately.
    GstSample* sample = gst_app_sink_try_pull_sample(
        GST_APP_SINK(sinkRef), params_.captureTimeoutMs * GST_MSECOND);
    gst_object_unref(sinkRef);

    if (!sample) return;

    // Copy the frame row-by-row honouring the GStreamer stride.  videoconvert /
    // nvvidconv pad each row up to a 4-byte-aligned stride, so for the BGR
    // appsink stride == width*3 only when width % 4 == 0.  A raw
    // gst_buffer_extract() of gst_buffer_get_size() bytes would shear the image
    // (or, since the padded size exceeds width*height*3, silently drop the frame)
    // for any width that isn't 4-aligned — e.g. odd USB modes.
    GstBuffer* gstBuffer = gst_sample_get_buffer(sample);
    GstCaps*   caps      = gst_sample_get_caps(sample);
    GstVideoInfo  vinfo;
    GstVideoFrame vframe;
    if (gstBuffer && caps &&
        gst_video_info_from_caps(&vinfo, caps) &&
        gst_video_frame_map(&vframe, &vinfo, gstBuffer, GST_MAP_READ)) {

        const guint   width  = GST_VIDEO_FRAME_WIDTH(&vframe);
        const guint   height = GST_VIDEO_FRAME_HEIGHT(&vframe);
        const gint    stride = GST_VIDEO_FRAME_PLANE_STRIDE(&vframe, 0);
        const guint8* src    = static_cast<const guint8*>(GST_VIDEO_FRAME_PLANE_DATA(&vframe, 0));
        const gsize   rowBytes = static_cast<gsize>(width) * 3;  // appsink caps are BGR

        // Only the default single-plane BGR sink is supported here; bail on
        // anything unexpected (planar, sub-row stride) rather than emit garbage.
        if (GST_VIDEO_FRAME_N_PLANES(&vframe) == 1 && src &&
            stride >= 0 && static_cast<gsize>(stride) >= rowBytes &&
            rowBytes * height <= bufferSize) {
            for (guint row = 0; row < height; ++row) {
                std::memcpy(buffer + row * rowBytes,
                            src + static_cast<gsize>(row) * static_cast<gsize>(stride),
                            rowBytes);
            }
            bytesWritten = static_cast<uint32_t>(rowBytes * height);
            std::lock_guard<std::mutex> lock(stateMutex_);
            status_.frameCount++;
        }
        gst_video_frame_unmap(&vframe);
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

GstElement* Camera_GST::getTee() const {
    // tee_ is published/cleared under stateMutex_ by start()/teardownPipeline();
    // an unguarded read here would race those writers (torn/stale pointer on
    // weakly-ordered ARM).
    std::lock_guard<std::mutex> lock(stateMutex_);
    return tee_;
}

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

} // namespace dashcam::camera
