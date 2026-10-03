#include "libcamera_gst.h"
#include <gst/video/video.h>
#include <algorithm>
#include <cctype>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <system_error>
#include <thread>

// ─── file-local log helper ────────────────────────────────────────────────────

namespace {

using dashcam::log::logPrintf;

// "<element>: <message> (<debug>)" for a bus ERROR or WARNING: which element
// failed, and GStreamer's own detail (e.g. "reason not-negotiated").
std::string describeMessage(GstMessage* msg) {
    GError* err = nullptr;
    gchar*  dbg = nullptr;
    if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) gst_message_parse_error(msg, &err, &dbg);
    else                                            gst_message_parse_warning(msg, &err, &dbg);
    std::string s = GST_MESSAGE_SRC_NAME(msg);
    s += ": ";
    s += err ? err->message : "unknown";
    if (dbg && *dbg) {
        s += " (";
        s += dbg;
        s += ")";
    }
    if (err) g_error_free(err);
    g_free(dbg);
    return s;
}

/// Bus warnings logged per pipeline run before the rest are only counted.
constexpr unsigned kMaxWarningsLogged = 10;

/// V4L2 auto controls, written ahead of every other control in one
/// extra-controls write: uvcvideo refuses a manual value (EACCES) while its
/// auto control is on, and v4l2src writes the fields in order.  That orders
/// start()'s batch only; a write while running carries one control, and its
/// caller sets the auto control first.  Kernel 5.15 names first, then the
/// older ones.
constexpr const char* kAutoControls[] = {
    "auto_exposure", "white_balance_automatic", "focus_automatic_continuous", "hue_automatic",
    "exposure_auto", "white_balance_temperature_auto", "focus_auto",
};

/// Does @p src take V4L2 controls through a GstStructure "extra-controls" (v4l2src)?
bool hasExtraControls(GstElement* src) {
    GParamSpec* pspec = g_object_class_find_property(G_OBJECT_GET_CLASS(src), "extra-controls");
    return pspec && pspec->value_type == GST_TYPE_STRUCTURE;
}

/// async=false on every sink in @p el (itself, or anywhere inside it when it is
/// a bin).  Such a sink does not hold the pipeline's state change waiting for a
/// preroll buffer.
void unsetSinksAsync(GstElement* el) {
    auto apply = [](GstElement* e) {
        if (GST_IS_BIN(e) || !GST_OBJECT_FLAG_IS_SET(e, GST_ELEMENT_FLAG_SINK)) return;
        GParamSpec* ps = g_object_class_find_property(G_OBJECT_GET_CLASS(e), "async");
        if (ps && ps->value_type == G_TYPE_BOOLEAN && (ps->flags & G_PARAM_WRITABLE))
            g_object_set(e, "async", FALSE, NULL);
    };
    apply(el);
    if (!GST_IS_BIN(el)) return;
    GstIterator* it = gst_bin_iterate_recurse(GST_BIN(el));
    GValue item = G_VALUE_INIT;
    for (bool done = false; !done;) {
        switch (gst_iterator_next(it, &item)) {
            case GST_ITERATOR_OK:
                apply(GST_ELEMENT(g_value_get_object(&item)));
                g_value_reset(&item);
                break;
            case GST_ITERATOR_RESYNC:
                gst_iterator_resync(it);
                break;
            default:
                done = true;
                break;
        }
    }
    g_value_unset(&item);
    gst_iterator_free(it);
}

/// The EOS send of one teardown, and its branch flush, shared with the threads
/// that make them: they outlive the teardown when the pipeline is left behind
/// (see Camera_GST::finishStream()).
struct EosSend {
    std::mutex              m;
    std::condition_variable cv;
    bool done     = false;        ///< gst_element_send_event() returned.
    bool flushing = false;        ///< The flush thread is still sending FLUSH_START.
    GstElement* left = nullptr;   ///< The teardown gave up: a ref on the pipeline, which
                                  ///< the last of those threads to return sets to NULL and drops.

    /// With m held, as a thread returns: the pipeline to release, if the
    /// teardown left it behind and no other thread is still inside it.
    GstElement* takeLeft() {
        if (!left || !done || flushing) return nullptr;
        GstElement* p = left;
        left = nullptr;
        return p;
    }
};

/// Sets a pipeline left behind to NULL and drops its ref (no-op for nullptr).
void releaseLeft(GstElement* pipe) {
    if (!pipe) return;
    gst_element_set_state(pipe, GST_STATE_NULL);
    gst_object_unref(pipe);
}

} // namespace

namespace dashcam::camera {

// ─── private helpers ────────────────────────────────────────────────────────

/// Destruction order matters for GStreamer reference counting:
///   1. Take pipeline_, and the branch lists, under stateMutex_ (the readers
///      that hold only the lock, e.g. setBranchEnabled(), then find none).
///   2. If the pipeline reached PLAYING and its stream has not already ended:
///      finishStream() sends EOS and waits, bounded, for the muxers to finalise
///      their containers.  It may leave a stuck pipeline behind (see there).
///   3. Set pipeline to NULL and wait for confirmation — joins the streaming thread.
///      NOTE: on CSI, nvarguscamerasrc's PAUSED→READY can itself block ~5 s when
///      the Argus session ends in the CANCELLED path (daemon-state dependent);
///      that wait is internal to the NVIDIA element and bounded by its own
///      timeout — not something this code can shorten.
///   4. Release tee request pads (gst_element_release_request_pad + unref).
///   5. Free orphaned branch bins (registered but never added to the pipeline).
///   6. Unref non-owning element handles (tee_, camera_src_, appsink_).
///   7. Unref the pipeline — releases all bin members.
///
/// The pipeline MUST reach NULL before pads are released.  Releasing request
/// pads while the streaming thread is still running (i.e. before NULL state)
/// is a GStreamer API violation and can cause the streaming thread to crash.
/// A pipeline left behind in step 2 is therefore not touched past step 1: its
/// EOS or flush thread, whichever returns last, sets it to NULL and drops the
/// last ref.
void Camera_GST::teardownPipeline() {
    // Never two at once: stop() and close() claim tearingDown_, and start()
    // (whose failures land here) starting_, and each waits for the other two.
    // A second call would not be a no-op: past this swap it still releases
    // teePads_ (unlocked) and nulls the element handles, a new start()'s too.
    // The swap takes the pipeline and the branch lists away from the readers
    // that hold only the lock (captureFrame(), setBranchEnabled(), addBranch()).
    GstElement* pipe = nullptr;
    GstElement* sinkRef = nullptr;
    std::vector<BranchEntry> branches;
    std::map<std::string, GstElement*> valves;
    {
        std::lock_guard<std::mutex> lk(stateMutex_);
        std::swap(pipe, pipeline_);
        if (pipe && appsink_)
            sinkRef = static_cast<GstElement*>(gst_object_ref(appsink_));
        branches.swap(branches_);
        valves.swap(branchValves_);
    }

    bool released = true;   // false: finishStream() left the pipeline to its EOS / flush threads
    if (pipe) {
        // EOS only to a pipeline that reached PLAYING.  One still short of it
        // (a start() that timed out: a sink that never prerolled, Argus slow to
        // come up) cannot finalise a container anyway: a sink in PAUSED holds
        // every buffer after its first until PLAYING, so a muxer's closing
        // writes would wait there.  Straight to NULL: a branch that did preroll
        // may have written the start of a file, and it stays unfinalised (the
        // accepted cost; start() reported ERROR).  A stream that already ended
        // (checkBusErrors() saw its EOS) has nothing left to flush.
        GstState state, pending;
        gst_element_get_state(pipe, &state, &pending, 0);
        if (state == GST_STATE_PLAYING && !eosSeen_)
            released = finishStream(pipe, sinkRef, valves);

        if (released) {
            // Transition to NULL joins the streaming thread.  nvarguscamerasrc
            // hardware teardown can be briefly async, so wait for confirmation
            // before releasing pads or unreffing elements.
            GstStateChangeReturn sc = gst_element_set_state(pipe, GST_STATE_NULL);
            if (sc == GST_STATE_CHANGE_ASYNC)
                gst_element_get_state(pipe, nullptr, nullptr,
                                      params_.stateChangeTimeoutMs * GST_MSECOND);
        }
    }
    if (sinkRef) gst_object_unref(sinkRef);

    GstElement* appsinkToUnref = nullptr;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        appsinkToUnref = appsink_;
        appsink_       = nullptr;
    }

    // A pipeline left behind still streams into these pads: only drop our refs;
    // the tee releases them when that pipeline is finalised.
    for (GstPad* pad : teePads_) {
        if (tee_ && released) gst_element_release_request_pad(tee_, pad);
        gst_object_unref(pad);
    }
    teePads_.clear();

    // Free branch bins that were registered but never added to a pipeline
    // (e.g., addBranch() then close() without start(), or a failed early start()).
    // Bins that were gst_bin_add_many()'d have the pipeline as their parent and
    // will be freed by gst_object_unref(pipe) below; check parent to distinguish.
    for (auto& br : branches) {
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

    // Null the element handles under the lock (consistent with appsink_ above),
    // then unref outside it.  Other threads read these pointers only under
    // stateMutex_, so the write must be locked too.
    GstElement* teeToUnref   = nullptr;
    GstElement* srcToUnref   = nullptr;
    GstElement* valveToUnref = nullptr;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        teeToUnref   = tee_;        tee_        = nullptr;
        srcToUnref   = camera_src_; camera_src_ = nullptr;
        valveToUnref = capValve_;   capValve_   = nullptr;
    }
    if (teeToUnref)     gst_object_unref(teeToUnref);
    if (srcToUnref)     gst_object_unref(srcToUnref);
    if (valveToUnref)   gst_object_unref(valveToUnref);
    if (appsinkToUnref) gst_object_unref(appsinkToUnref);
    if (pipe)           gst_object_unref(pipe);
}

/// Sends EOS to a PLAYING pipeline and waits, up to params_.eosTimeoutMs, for
/// the pipeline's EOS (or the error that replaces it), so muxers finalise their
/// containers.  Bounded in every case:
///
/// - The EOS goes out on its own thread.  gst_element_send_event() does not
///   return until each source's streaming thread lets go of its pad, and that
///   thread may be stuck in a push: a branch that stopped taking buffers fills
///   its blocking queue, and the tee then waits on it for good.  The send also
///   holds the pipeline's state lock, so the NULL transition cannot run either.
/// - If the pipeline's EOS has not come by the deadline, each branch is flushed
///   from its head (FLUSH_START on the pad after its valve, which a dropping
///   valve would not pass on).  That unblocks a GStreamer wait anywhere
///   downstream: a sink, a queue, a clock.  A stuck send then completes: the
///   tee's push fails and the source's thread lets go.  Healthy branches get
///   the EOS that late, with no time left to finalise: their frames had
///   stopped with the tee (a WARN says so).  A send that did return means a
///   branch still holds the EOS: an appsink nobody pulls, a slow muxer, or a
///   sink stuck in a system call (a filesink in a write() to a stalled SD
///   card) behind a queue that has not filled.  Such a sink holds its preroll
///   lock, which the NULL transition needs as much as the flush does.  So the
///   flush goes out on a thread of its own, and only the call returning frees
///   it; when the send returned, it goes out without a WARN.
/// - If the send and the flush have not both returned within
///   params_.stateChangeTimeoutMs, something is stuck where no flush reaches:
///   the source's thread in a driver call, or a sink in a system call.  The
///   pipeline is left to those threads (EosSend::left): the last to return
///   sets it to NULL and drops its ref.  Returns false, and the caller must not
///   touch the pipeline again.
///
/// Meanwhile the capture appsink is drained, as teardown always did: the
/// captureFrame() consumer has stopped, and an appsink left at wait-on-eos=true
/// holds its EOS until its queue is pulled.  The wait also ends on an ERROR:
/// nvarguscamerasrc sometimes posts one (Argus CANCELLED) instead of
/// forwarding EOS, and draining first would burn the budget before seeing it.
/// The send is then still given the rest of the EOS window: it returns once
/// the source's thread lets go, which takes a moment, and nothing has stalled.
/// Only a send still stuck at the deadline is warned about.
bool Camera_GST::finishStream(GstElement* pipe, GstElement* sinkRef,
                              const std::map<std::string, GstElement*>& valves) {
    auto send = std::make_shared<EosSend>();
    std::thread sender;
    try {
        GstElement* senderPipe = static_cast<GstElement*>(gst_object_ref(pipe));
        sender = std::thread([send, senderPipe] {
            gst_element_send_event(senderPipe, gst_event_new_eos());
            GstElement* left;
            {
                std::lock_guard<std::mutex> lk(send->m);
                send->done = true;
                left = send->takeLeft();
            }
            send->cv.notify_all();
            releaseLeft(left);
            gst_object_unref(senderPipe);
        });
    } catch (const std::system_error& e) {
        gst_object_unref(pipe);   // the ref the thread would have dropped
        logPrintf(log_, dashcam::log::LogLevel::ERROR,
              "cannot start the EOS thread for %s (%s): stopping without EOS",
              info_.address.c_str(), e.what());
        return true;
    }

    // The EOS window stays generous on purpose: with a ±30 s recording
    // pre-buffer, matroskamux can need >1.5 s to flush on a CPU-encoder path.
    // Every iteration waits <= 50 ms or pulls a sample (finite after EOS).
    GstBus* bus = gst_element_get_bus(pipe);
    const gint64 begin = g_get_monotonic_time();
    const gint64 deadline = begin
        + static_cast<gint64>(params_.eosTimeoutMs) * G_TIME_SPAN_MILLISECOND;
    const GstMessageType eosMask = static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR);
    bool ended = false, sawEos = false;
    while (bus && !ended && !eosSeen_ && g_get_monotonic_time() < deadline) {
        const bool sinkDone = !sinkRef || gst_app_sink_is_eos(GST_APP_SINK(sinkRef));
        if (!sinkDone) {
            GstSample* s = gst_app_sink_try_pull_sample(GST_APP_SINK(sinkRef), 25 * GST_MSECOND);
            if (s) gst_sample_unref(s);
        }
        // Block on the bus only once the appsink is fully drained; until then
        // just poll so the drain keeps making progress.
        GstMessage* msg = gst_bus_timed_pop_filtered(bus, sinkDone ? 50 * GST_MSECOND : 0, eosMask);
        if (msg) {
            ended  = true;
            sawEos = GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS;
            gst_message_unref(msg);
        }
    }
    if (bus) gst_object_unref(bus);

    auto sent = [&] { return send->done; };
    auto settled = [&] { return send->done && !send->flushing; };
    const bool eosDone = sawEos || eosSeen_;
    std::unique_lock<std::mutex> lk(send->m);
    if (eosDone) {
        // The EOS reached every sink, so the send is returning: its message can
        // beat that by a moment.
        send->cv.wait_for(lk, std::chrono::milliseconds(params_.stateChangeTimeoutMs), sent);
    } else if (ended) {
        // An ERROR replaced the EOS: the send returns once the source's thread
        // lets go.  Only a send still stuck at the deadline means a stall.
        const gint64 rest = deadline - g_get_monotonic_time();
        if (rest > 0) send->cv.wait_for(lk, std::chrono::microseconds(rest), sent);
    }
    const bool sendStuck = !sent();
    lk.unlock();
    if (eosDone && !sendStuck) {
        sender.join();
        return true;
    }

    // Short of the EOS, a branch's sink may still be inside a call, holding the
    // preroll lock that both the flush and the NULL transition need.  A send
    // that returned is no stall: the stream ended on an ERROR, or a branch
    // still holds the EOS (an appsink nobody pulls, a slow muxer), and the
    // flush is quiet.
    if (sendStuck)
        logPrintf(log_, dashcam::log::LogLevel::WARN,
              "EOS not delivered on %s within %u ms (waited %lld ms): a branch has stopped "
              "taking frames; flushing the branches (their files are not finalised)",
              info_.address.c_str(), params_.eosTimeoutMs,
              static_cast<long long>((g_get_monotonic_time() - begin) / G_TIME_SPAN_MILLISECOND));
    std::vector<GstPad*> heads;
    for (const auto& [name, valve] : valves) {
        GstPad* valveSrc = gst_element_get_static_pad(valve, "src");
        if (GstPad* head = valveSrc ? gst_pad_get_peer(valveSrc) : nullptr) heads.push_back(head);
        if (valveSrc) gst_object_unref(valveSrc);
    }
    std::thread flusher;
    if (!heads.empty()) {
        lk.lock();
        send->flushing = true;   // before the thread: the EOS thread reads it too
        lk.unlock();
        try {
            flusher = std::thread([send, heads] {
                for (GstPad* head : heads) {
                    gst_pad_send_event(head, gst_event_new_flush_start());
                    gst_object_unref(head);
                }
                GstElement* left;
                {
                    std::lock_guard<std::mutex> flk(send->m);
                    send->flushing = false;
                    left = send->takeLeft();
                }
                send->cv.notify_all();
                releaseLeft(left);
            });
        } catch (const std::system_error& e) {
            lk.lock();
            send->flushing = false;
            lk.unlock();
            for (GstPad* head : heads) gst_object_unref(head);
            logPrintf(log_, dashcam::log::LogLevel::ERROR,
                  "cannot start the flush thread for %s (%s): not flushing",
                  info_.address.c_str(), e.what());
        }
    }
    lk.lock();
    send->cv.wait_for(lk, std::chrono::milliseconds(params_.stateChangeTimeoutMs), settled);
    // A thread that has returned is past its takeLeft(): joining it is quick.
    const bool freed = settled(), sendReturned = send->done, flushReturned = !send->flushing;
    if (!freed) send->left = static_cast<GstElement*>(gst_object_ref(pipe));
    lk.unlock();

    if (sendReturned) sender.join();
    else              sender.detach();
    if (flusher.joinable()) {
        if (flushReturned) flusher.join();
        else               flusher.detach();
    }
    if (freed) return true;
    const char* why = !flushReturned && !sendReturned
                        ? "EOS could not be sent, and a branch's sink did not take the flush"
                    : !flushReturned ? "a branch's sink did not take the flush"
                                     : "EOS could not be sent";
    logPrintf(log_, dashcam::log::LogLevel::ERROR,
          "pipeline on %s is stuck: %s within %u ms of flushing%s; "
          "left behind, released when its streaming thread frees",
          info_.address.c_str(), why, params_.stateChangeTimeoutMs,
          flushReturned ? "" : " (a sink stuck in a system call, e.g. a write to a stalled disk)");
    return false;
}

void Camera_GST::setPipelineError() {
    logPrintf(log_, dashcam::log::LogLevel::ERROR,
          "pipeline error on %s", info_.address.c_str());
    teardownPipeline();
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        status_.status       = CAMERA_STATUS::ERROR;
        status_.currentError = pipelineError();
        starting_ = false;  // construction phase over (failed); unblock stop() / close()
    }
    lifecycleCv_.notify_all();
}

void Camera_GST::logFirstBusError(GstElement* pipe, const char* what) {
    // A bin wakes the state-change wait with FAILURE first and forwards the
    // ERROR to the bus after: wait a moment for it rather than lose the reason.
    GstBus* bus = pipe ? gst_element_get_bus(pipe) : nullptr;
    GstMessage* msg = bus ? gst_bus_timed_pop_filtered(bus, 250 * GST_MSECOND, GST_MESSAGE_ERROR)
                          : nullptr;
    if (msg) {
        logPrintf(log_, dashcam::log::LogLevel::ERROR, "%s on %s: %s", what,
              info_.address.c_str(), describeMessage(msg).c_str());
        gst_message_unref(msg);
    } else {
        logPrintf(log_, dashcam::log::LogLevel::ERROR, "%s on %s (no error on the bus)", what,
              info_.address.c_str());
    }
    if (bus) gst_object_unref(bus);
}

void Camera_GST::checkBusErrors() {
    // Take a ref on the pipeline under the lock so a concurrent teardown can't
    // free it while we poll its bus.  Not while start() is still building: its
    // own state-change waits report a failure, and find the error on the bus.
    GstElement* pipeRef = nullptr;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (status_.status != CAMERA_STATUS::RUNNING || !pipeline_ || starting_) return;
        pipeRef = static_cast<GstElement*>(gst_object_ref(pipeline_));
    }

    GstBus* bus = gst_element_get_bus(pipeRef);
    bool sawError = false;
    bool sawEos   = false;
    if (bus) {
        // Drain every message so the bus queue can't grow unbounded over a long
        // capture.  Report the first error, and warnings (a branch complaining
        // before it fails) up to a cap per run, so a chatty element cannot
        // flood the log.  EOS is kept as eosSeen_: a live camera's stream only
        // ends when its source gives up, and teardown must not wait for a
        // second one.
        GstMessage* msg;
        while ((msg = gst_bus_pop(bus)) != nullptr) {
            if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR && !sawError) {
                logPrintf(log_, dashcam::log::LogLevel::ERROR, "pipeline bus error on %s: %s",
                      info_.address.c_str(), describeMessage(msg).c_str());
                sawError = true;
            } else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_WARNING) {
                const unsigned seen = ++warningsSeen_;
                if (seen <= kMaxWarningsLogged)
                    logPrintf(log_, dashcam::log::LogLevel::WARN, "pipeline warning on %s: %s%s",
                          info_.address.c_str(), describeMessage(msg).c_str(),
                          seen == kMaxWarningsLogged ? " (further warnings not logged)" : "");
            } else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS) {
                sawEos   = true;
                eosSeen_ = true;
            }
            gst_message_unref(msg);
        }
        gst_object_unref(bus);
    }
    gst_object_unref(pipeRef);

    if (sawError || sawEos) {
        // Flip to ERROR; the app observes this via getCameraStatus() and calls
        // close() to release the pipeline.  Leave teardown to that path so we
        // don't tear down a pipeline captureFrame() may still be reffing.
        bool ended = false;
        {
            std::lock_guard<std::mutex> lock(stateMutex_);
            if (status_.status == CAMERA_STATUS::RUNNING) {
                status_.status       = CAMERA_STATUS::ERROR;
                status_.currentError = pipelineError();
                ended = !sawError;
            }
        }
        if (ended)
            logPrintf(log_, dashcam::log::LogLevel::ERROR,
                  "pipeline on %s reached end of stream: the source stopped delivering frames",
                  info_.address.c_str());
    }
}

void Camera_GST::applyAttributeGStreamer(const std::string& name,
                                         const std::string& value) {
    if (!camera_src_) {
        status_.currentError = pipelineError();
        return;
    }
    const AttributeEntry* entry = dict_.resolve(name, cameraTypeTag());
    bool ok = false;
    if (entry && entry->valueType == AttributeValueType::V4l2Control) {
        int iv = 0;
        ok = !entry->gstProperty.empty() && safeStoi(value, iv) && hasExtraControls(camera_src_);
        if (ok && controlBatch_) {
            // A flush: every control goes to the element in one write, below.
            gst_structure_set(controlBatch_, entry->gstProperty.c_str(), G_TYPE_INT, iv, NULL);
        } else if (ok) {
            // Running: this control alone.  v4l2src applies the whole structure
            // it is given, so re-sending earlier controls would undo whatever
            // changed them since (another process, the exposure loop).
            GstStructure* one = gst_structure_new("controls", entry->gstProperty.c_str(),
                                                  G_TYPE_INT, iv, NULL);
            ok = writeExtraControls(camera_src_, one);
            gst_structure_free(one);
        }
    } else if (entry) {
        ok = applyGstProperty(camera_src_, *entry, value);
    }
    status_.currentError = ok ? ERROR_CODE::NONE : ERROR_CODE::INVALID_ATTRIBUTE;
}

// V4L2 controls collect in controlBatch_ and go to v4l2src in one write, auto
// controls first (writeExtraControls()): pendingAttributes_ is sorted by alias,
// so "exposure" would otherwise come before "exposure_auto".
void Camera_GST::flushPendingAttributes(std::vector<std::string>& rejected) {
    controlBatch_ = gst_structure_new_empty("controls");
    for (const auto& [attrName, attrValue] : pendingAttributes_) {
        applyAttributeGStreamer(attrName, attrValue);
        if (status_.currentError == ERROR_CODE::INVALID_ATTRIBUTE)
            rejected.push_back(attrName + "=" + attrValue);
    }
    if (gst_structure_n_fields(controlBatch_) > 0)
        writeExtraControls(camera_src_, controlBatch_);
    gst_structure_free(controlBatch_);
    controlBatch_ = nullptr;
    pendingAttributes_.clear();
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
    GParamSpec* pspec = g_object_class_find_property(G_OBJECT_GET_CLASS(src), prop);
    if (!pspec || !(pspec->flags & G_PARAM_WRITABLE) || (pspec->flags & G_PARAM_CONSTRUCT_ONLY))
        return false;

    // Parse into a GValue of the dictionary's type...
    GValue in = G_VALUE_INIT;
    switch (entry.valueType) {
        case AttributeValueType::String:
            g_value_init(&in, G_TYPE_STRING);
            g_value_set_string(&in, value.c_str());
            break;

        case AttributeValueType::RangeString:
            g_value_init(&in, G_TYPE_STRING);
            g_value_set_string(&in, (value + " " + value).c_str());
            break;

        case AttributeValueType::Int:
        case AttributeValueType::BoolFromZero: {
            int iv = 0;
            if (!safeStoi(value, iv)) return false;
            if (entry.valueType == AttributeValueType::Int) {
                g_value_init(&in, G_TYPE_INT);
                g_value_set_int(&in, iv);
            } else {
                g_value_init(&in, G_TYPE_BOOLEAN);
                g_value_set_boolean(&in, iv == 0 ? TRUE : FALSE);
            }
            break;
        }

        case AttributeValueType::Float: {
            char* end = nullptr;
            const float fv = std::strtof(value.c_str(), &end);
            // Reject empty input and trailing garbage (e.g. "1.5abc"), matching
            // the strictness of the Int path via safeStoi().
            if (end == value.c_str() || *end != '\0') return false;
            g_value_init(&in, G_TYPE_FLOAT);
            g_value_set_float(&in, fv);
            break;
        }

        case AttributeValueType::Bool: {
            std::string lower = value;
            std::transform(lower.begin(), lower.end(), lower.begin(),
                           [](unsigned char c){ return std::tolower(c); });
            gboolean bv;
            if      (lower == "true"  || lower == "1") bv = TRUE;
            else if (lower == "false" || lower == "0") bv = FALSE;
            else return false;
            g_value_init(&in, G_TYPE_BOOLEAN);
            g_value_set_boolean(&in, bv);
            break;
        }

        case AttributeValueType::V4l2Control:   // not a property: writeExtraControls()
            return false;
    }

    // ...then convert it to the property's own type (int → enum, uint, int64;
    // float → double; ...) and range-check it before writing anything.
    GValue out = G_VALUE_INIT;
    g_value_init(&out, pspec->value_type);
    const bool ok = g_value_type_transformable(G_VALUE_TYPE(&in), pspec->value_type) &&
                    g_value_transform(&in, &out) &&
                    !g_param_value_validate(pspec, &out);   // true = had to change it
    if (ok) g_object_set_property(G_OBJECT(src), prop, &out);
    g_value_unset(&in);
    g_value_unset(&out);
    return ok;
}

bool Camera_GST::writeExtraControls(GstElement* src, const GstStructure* controls) {
    if (!src || !controls || !hasExtraControls(src)) return false;
    GstStructure* ordered = gst_structure_new_empty(gst_structure_get_name(controls));
    for (const char* name : kAutoControls)
        if (const GValue* v = gst_structure_get_value(controls, name))
            gst_structure_set_value(ordered, name, v);
    for (gint i = 0; i < gst_structure_n_fields(controls); ++i) {
        const char* name = gst_structure_nth_field_name(controls, static_cast<guint>(i));
        if (!gst_structure_has_field(ordered, name))
            gst_structure_set_value(ordered, name, gst_structure_get_value(controls, name));
    }
    g_object_set(G_OBJECT(src), "extra-controls", ordered, NULL);
    gst_structure_free(ordered);
    return true;
}

std::string Camera_GST::captureBranch(const std::string& convert) const {
    return " ! tee name=srctee"
           " srctee. ! queue max-size-buffers=" + std::to_string(params_.captureQueueDepth) +
           " leaky=2 ! valve name=capvalve drop-mode=1"
           " ! " + convert +
           " ! appsink name=mysink drop=true max-buffers=" + std::to_string(params_.appsinkMaxBuffers) +
           " emit-signals=false sync=false";
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
      tee_(nullptr),
      capValve_(nullptr) {
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
        logPrintf(log_, dashcam::log::LogLevel::ERROR,
              "gst_init failed for %s: %s", info_.address.c_str(),
              err ? err->message : "unknown");
    }
    if (err) g_error_free(err);

    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (!gstOk) {
            status_.status = CAMERA_STATUS::ERROR;
            status_.currentError = pipelineError();
            return;
        }
        status_.status = CAMERA_STATUS::OPEN;
        status_.currentError = ERROR_CODE::NONE;
    }
    logPrintf(log_, dashcam::log::LogLevel::INFO,   // outside the lock: the callback contract
          "camera opened: %s", info_.address.c_str());
}

void Camera_GST::close() {
    bool running = false;
    {
        std::unique_lock<std::mutex> lock(stateMutex_);
        // As in stop(): never tear down a pipeline start() is still building
        // (the status alone does not say so: it can read ERROR by then), nor
        // one a stop() is still tearing down (the status reads OPEN by then).
        lifecycleCv_.wait(lock, [this] { return !starting_ && !tearingDown_; });
        if (status_.status == CAMERA_STATUS::CLOSED) {
            status_.currentError = ERROR_CODE::CAMERA_ALREADY_CLOSED;
            return;
        }
        // The stop() and the close are one teardown under one claim, so no
        // start() can slip in between them.
        running = (status_.status == CAMERA_STATUS::RUNNING);
        if (running) status_.status = CAMERA_STATUS::OPEN;
        tearingDown_ = true;
    }
    logPrintf(log_, dashcam::log::LogLevel::INFO,
          "camera closing: %s", info_.address.c_str());
    if (running)
        logPrintf(log_, dashcam::log::LogLevel::INFO,
              "stopping pipeline: %s", info_.address.c_str());

    // Outside the lock: teardown waits on GStreamer threads that may log.
    teardownPipeline();

    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        status_.status          = CAMERA_STATUS::CLOSED;
        status_.currentError    = ERROR_CODE::NONE;
        status_.freeBufferCount = 0;
        tearingDown_ = false;
    }
    lifecycleCv_.notify_all();
}

bool Camera_GST::isOpen() const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    return status_.status != CAMERA_STATUS::CLOSED;
}

// ─── iCamera interface ───────────────────────────────────────────────────────

void Camera_GST::getCameraInfo(cameraInfo& info) const { info = info_; }

void Camera_GST::setCameraAttribute(const std::string& name, const std::string& value) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (status_.status != CAMERA_STATUS::RUNNING || !camera_src_ || starting_) {
        // Queue in three cases: not yet started; start() still building (the
        // source element exists before PLAYING, but v4l2src may be walking its
        // extra-controls as it opens the device, and freeing them under it);
        // or ERROR state recovery.  The late-attributes drain at the end of
        // start() flushes what start() has not.
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
        std::unique_lock<std::mutex> lock(stateMutex_);
        // A stop() or close() still tearing down has already said OPEN, but its
        // teardown would take this start()'s handles and tee pads: wait for it.
        // It ends within its own bounds (see stop()).
        lifecycleCv_.wait(lock, [this] { return !tearingDown_; });
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
        status_.frameCount = 0;                   // "since start()"
        starting_ = true;  // Cleared (with lifecycleCv_ notify) on every exit path:
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
    logPrintf(log_, dashcam::log::LogLevel::INFO,
          "starting pipeline on %s", info_.address.c_str());
    logPrintf(log_, dashcam::log::LogLevel::DEBUG,
          "pipeline: %s", pipelineStr.c_str());

    // Publish the shared handles under stateMutex_ so status (already RUNNING) and
    // the pointers become visible together to the concurrent captureFrame() reader,
    // instead of being assigned bare while another thread reads them under the lock.
    // pipeline_ is published first so setPipelineError() → teardownPipeline() can
    // release it (and any orphaned branch bins) on every failure path below.
    warningsSeen_ = 0;   // only this thread touches these until the pipeline runs
    eosSeen_      = false;
    GError* error = nullptr;
    GstElement* pipeline = gst_parse_launch(pipelineStr.c_str(), &error);
    if (error != nullptr || pipeline == nullptr) {
        // e.g. "no element \"nvarguscamerasrc\"" or "could not link ...": the
        // reason the camera will not start, so it is worth the line.
        logPrintf(log_, dashcam::log::LogLevel::ERROR, "pipeline description rejected on %s: %s",
              info_.address.c_str(), error ? error->message : "unknown");
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
    GstElement* capValve  = gst_bin_get_by_name(GST_BIN(pipeline), "capvalve");

    if (!appsink || !cameraSrc || !tee || !capValve) {
        logPrintf(log_, dashcam::log::LogLevel::ERROR,
              "required pipeline elements not found on %s", info_.address.c_str());
        if (appsink)   gst_object_unref(appsink);
        if (cameraSrc) gst_object_unref(cameraSrc);
        if (tee)       gst_object_unref(tee);
        if (capValve)  gst_object_unref(capValve);
        setPipelineError();   // tears down the published pipeline_ and orphan branches
        return;
    }

    // The capture appsink's EOS must not wait for the application to pull its
    // last sample (the default): with nobody pulling, a stream that ends would
    // never post EOS, and the camera would keep reporting RUNNING.
    if (g_object_class_find_property(G_OBJECT_GET_CLASS(appsink), "wait-on-eos"))
        g_object_set(appsink, "wait-on-eos", FALSE, NULL);

    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        appsink_    = appsink;
        camera_src_ = cameraSrc;
        tee_        = tee;
        capValve_   = capValve;
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
            logPrintf(log_, dashcam::log::LogLevel::ERROR,
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
            logPrintf(log_, dashcam::log::LogLevel::ERROR,
                  "failed to create valve element for branch '%s'", branchName.c_str());
            gst_object_ref_sink(queue);     gst_object_unref(queue);
            gst_object_ref_sink(branchBin); gst_object_unref(branchBin);
            branchBin = nullptr;
            releaseRemaining(i + 1);
            branchError = true;
            break;
        }
        // A dropping valve must still pass EOS (sticky), otherwise a branch that
        // is disabled at stop() time never delivers EOS to its sink — the muxer
        // never finalises its file and teardownPipeline() waits out the full EOS
        // timeout: forward-sticky-events (1).  drop-mode is only changeable in
        // NULL/READY, so it is set here.
        //
        // A branch that starts DISABLED never gets a buffer before PLAYING, so
        // its sinks could not preroll and start() would time out (measured
        // 2026-09-28).  Its sinks are made async=false: they then do not take
        // part in the preroll at all, and render from their first buffer once
        // the branch is enabled.  (Turning the dropped buffers into GAP events
        // instead, drop-mode 2, prerolls a bare sink only: videorate swallows
        // GAPs, so the inference bins still timed out, and x264enc queues one
        // per frame until its next buffer, without bound.)
        g_object_set(G_OBJECT(valve), "drop-mode", 1 /* forward-sticky-events */, NULL);
        if (!branchInitEnabled) {
            g_object_set(G_OBJECT(valve), "drop", TRUE, NULL);
            unsetSinksAsync(branchBin);
        }

        gst_bin_add_many(GST_BIN(pipeline_), queue, valve, branchBin, nullptr);

        GstPad* teeSrcPad = gst_element_request_pad_simple(tee_, "src_%u");
        if (!teeSrcPad) {
            logPrintf(log_, dashcam::log::LogLevel::ERROR,
                  "failed to get tee src pad for branch '%s'", branchName.c_str());
            releaseRemaining(i + 1);
            branchError = true;
            break;
        }

        GstPad* queueSinkPad = gst_element_get_static_pad(queue, "sink");
        if (!queueSinkPad) {
            logPrintf(log_, dashcam::log::LogLevel::ERROR,
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
            logPrintf(log_, dashcam::log::LogLevel::ERROR,
                  "pad/element link failed for branch '%s'", branchName.c_str());
            gst_element_release_request_pad(tee_, teeSrcPad);
            gst_object_unref(teeSrcPad);
            releaseRemaining(i + 1);
            branchError = true;
            break;
        }
        logPrintf(log_, dashcam::log::LogLevel::INFO,
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
    // A rejected one (a config typo, a value the element does not take) is
    // logged once the lock is released.
    std::vector<std::string> rejected;
    auto logRejected = [&]() {
        for (const auto& r : rejected)
            logPrintf(log_, dashcam::log::LogLevel::WARN,
                  "attribute %s not applied on %s: unknown to the dictionary, or a value "
                  "the element does not take", r.c_str(), info_.address.c_str());
        rejected.clear();
    };
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        flushPendingAttributes(rejected);
    }
    logRejected();

    // Hardware boots here with the correct ISP settings already applied.
    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        // The element that refused posted why (device busy, not-negotiated, ...).
        logFirstBusError(pipeline_, "pipeline refused to start");
        setPipelineError();
        return;
    }
    if (ret == GST_STATE_CHANGE_ASYNC) {
        // Block until Argus/V4L2 hardware fully initialises (up to 5 s).
        // status_ is RUNNING at this point (claimed optimistically above).
        // setCameraAttribute() queues while starting_, so attribute writes
        // during this window wait for the late-attributes flush below.
        GstState state, pending;
        ret = gst_element_get_state(pipeline_, &state, &pending,
                                    params_.stateChangeTimeoutMs * GST_MSECOND);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            logFirstBusError(pipeline_, "pipeline failed while starting");
            setPipelineError();
            return;
        }
        if (ret == GST_STATE_CHANGE_ASYNC) {
            logPrintf(log_, dashcam::log::LogLevel::ERROR,
                  "pipeline on %s did not reach PLAYING within %u ms", info_.address.c_str(),
                  params_.stateChangeTimeoutMs);
            setPipelineError();
            return;
        }
    }

    logPrintf(log_, dashcam::log::LogLevel::INFO,
          "pipeline running: %s", info_.address.c_str());

    // Drain anything queued during the startup window.  status_ is already
    // RUNNING (claimed at the top of start() to serialise concurrent callers).
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        flushPendingAttributes(rejected);
        starting_ = false;  // construction phase over (success); unblock stop() / close()
    }
    lifecycleCv_.notify_all();
    logRejected();
}

void Camera_GST::stop() {
    {
        std::unique_lock<std::mutex> lock(stateMutex_);
        // A concurrent start() may still be constructing the pipeline outside
        // the lock (optimistic RUNNING claim).  Tearing down now would free a
        // half-built pipeline start() is still using.  start() always finishes
        // (bounded by its state-change timeouts), so this wait terminates.
        // Another stop() or close() still tearing down is waited for too: this
        // one then returns with the pipeline gone, not while it is going.
        lifecycleCv_.wait(lock, [this] { return !starting_ && !tearingDown_; });
        if (status_.status != CAMERA_STATUS::RUNNING) return;
        status_.status = CAMERA_STATUS::OPEN;
        tearingDown_ = true;   // close() and start() wait for it
    }
    logPrintf(log_, dashcam::log::LogLevel::INFO,
          "stopping pipeline: %s", info_.address.c_str());
    teardownPipeline();
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        status_.freeBufferCount = 0;
        tearingDown_ = false;
    }
    lifecycleCv_.notify_all();
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
    const char* refused = !sinkBin ? "null bin" : name.empty() ? "empty name" : nullptr;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (!refused && status_.status == CAMERA_STATUS::RUNNING) refused = "camera already running";
        if (!refused) {
            branches_.push_back({name, sinkBin, leaky, initialEnabled});
            return;
        }
        status_.currentError = ERROR_CODE::INVALID_ATTRIBUTE;
    }
    // Logged outside the lock (the callback contract), and the bin released:
    // ownership passes on every call, since no caller frees a refused bin.
    logPrintf(log_, dashcam::log::LogLevel::ERROR, "cannot add camera branch '%s': %s",
          name.c_str(), refused);
    if (sinkBin) {
        gst_object_ref_sink(sinkBin);
        gst_object_unref(sinkBin);
    }
}

GstElement* Camera_GST::getTee() const {
    // tee_ is published/cleared under stateMutex_ by start()/teardownPipeline();
    // an unguarded read here would race those writers (torn/stale pointer on
    // weakly-ordered ARM).
    std::lock_guard<std::mutex> lock(stateMutex_);
    return tee_;
}

// The capture chain (queue ! [nvvidconv !] videoconvert ! appsink) converts
// every frame on the CPU even when nobody calls captureFrame().  Disabling it
// drops buffers at the valve so the converters idle; captureFrame() times out
// while disabled.  drop-mode=1 keeps EOS flowing for teardown.
void Camera_GST::setCaptureEnabled(bool enabled) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (!capValve_ || status_.status != CAMERA_STATUS::RUNNING) {
        status_.currentError = ERROR_CODE::CAMERA_NOT_OPEN;
        return;
    }
    g_object_set(G_OBJECT(capValve_), "drop", enabled ? FALSE : TRUE, NULL);
    status_.currentError = ERROR_CODE::NONE;
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
