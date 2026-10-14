#include "librecord.h"

#include <linux/videodev2.h>
#include <dirent.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <climits>
#include <condition_variable>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <exception>

namespace dashcam::record {

using dashcam::log::LogLevel;

// ─── file-local helpers ───────────────────────────────────────────────────────

static void doLog(const dashcam::log::LogCallback& cb, LogLevel lvl, const char* fmt, ...) {
    if (!cb) return;
    char buf[1536];   // the full pipeline string, with every tap, fits
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    cb(lvl, buf);
}

static int64_t steadyNowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// CLOCK_MONOTONIC in µs: the preview tap's capture-time base (explicitly the
// kernel clock, which GstSystemClock's MONOTONIC mode also reads).
static uint64_t monotonicNowUs() {
    struct timespec ts;
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL + static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}

static int64_t wallNowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

static std::string baseName(const std::string& path) {
    const auto sep = path.find_last_of('/');
    return sep == std::string::npos ? path : path.substr(sep + 1);
}

// Directory holding @p path (its entry must be synced for a new file's NAME to
// survive a power cut; fdatasync on the file alone does not cover it).
static std::string dirOf(const std::string& path) {
    const auto sep = path.find_last_of('/');
    return sep == std::string::npos ? "." : (sep == 0 ? "/" : path.substr(0, sep));
}

// Text of the first ERROR waiting on @p pipeline's bus ("" when none).
static std::string popBusError(GstElement* pipeline) {
    std::string text;
    GstBus* bus = gst_element_get_bus(pipeline);
    if (!bus) return text;
    if (GstMessage* msg = gst_bus_pop_filtered(bus, GST_MESSAGE_ERROR)) {
        GError* err = nullptr;
        gchar*  dbg = nullptr;
        gst_message_parse_error(msg, &err, &dbg);
        text = err ? err->message : "?";
        if (dbg) { text += " ("; text += dbg; text += ")"; }
        if (err) g_error_free(err);
        g_free(dbg);
        gst_message_unref(msg);
    }
    gst_object_unref(bus);
    return text;
}

// ─── segment naming ───────────────────────────────────────────────────────────

static bool allDigits(std::string_view s) {
    return !s.empty() && std::all_of(s.begin(), s.end(),
                                     [](char c) { return c >= '0' && c <= '9'; });
}

std::optional<uint64_t> parseSegmentName(std::string_view name, std::string_view prefix,
                                         bool* isAss) {
    // <prefix>_<SEQ 6..12>_<YYYYMMDD 8>_<HHMMSS 6>.<mkv|ass>
    if (prefix.empty() || name.size() <= prefix.size() + 1) return std::nullopt;
    if (name.substr(0, prefix.size()) != prefix || name[prefix.size()] != '_')
        return std::nullopt;
    std::string_view rest = name.substr(prefix.size() + 1);

    if (rest.size() < 4) return std::nullopt;
    const std::string_view ext = rest.substr(rest.size() - 4);
    if (ext != ".mkv" && ext != ".ass") return std::nullopt;
    rest.remove_suffix(4);

    const auto u1 = rest.find('_');
    if (u1 == std::string_view::npos) return std::nullopt;
    const std::string_view seqPart = rest.substr(0, u1);
    const std::string_view stamp   = rest.substr(u1 + 1);   // YYYYMMDD_HHMMSS
    if (seqPart.size() < 6 || seqPart.size() > 12 || !allDigits(seqPart)) return std::nullopt;
    if (stamp.size() != 15 || stamp[8] != '_' ||
        !allDigits(stamp.substr(0, 8)) || !allDigits(stamp.substr(9)))
        return std::nullopt;

    uint64_t seq = 0;
    const auto r = std::from_chars(seqPart.data(), seqPart.data() + seqPart.size(), seq);
    if (r.ec != std::errc() || r.ptr != seqPart.data() + seqPart.size()) return std::nullopt;
    if (isAss) *isAss = (ext == ".ass");
    return seq;
}

std::string segmentFileName(const std::string& prefix, uint64_t seq, time_t wallSec) {
    struct tm tmBuf;
    localtime_r(&wallSec, &tmBuf);
    char stamp[24];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tmBuf);
    char seqBuf[24];
    std::snprintf(seqBuf, sizeof(seqBuf), "%06llu", static_cast<unsigned long long>(seq));
    return prefix + "_" + seqBuf + "_" + stamp + ".mkv";
}

uint64_t nextSegmentSeq(const std::string& dir, const std::string& prefix) {
    uint64_t maxSeq = 0;
    DIR* d = ::opendir(dir.c_str());
    if (!d) return 1;
    while (const struct dirent* e = ::readdir(d)) {
        if (auto seq = parseSegmentName(e->d_name, prefix)) maxSeq = std::max(maxSeq, *seq);
    }
    ::closedir(d);
    return maxSeq + 1;
}

// ─── construction / accessors ─────────────────────────────────────────────────

SegmentedRecorder::SegmentedRecorder() = default;

SegmentedRecorder::~SegmentedRecorder() { stop(); }

void SegmentedRecorder::setOverlayData(const OverlayData& data) {
    std::lock_guard<std::mutex> lock(overlayMutex_);
    overlayData_ = data;
}

OverlayData SegmentedRecorder::getOverlayData() const {
    std::lock_guard<std::mutex> lock(overlayMutex_);
    return overlayData_;
}

void SegmentedRecorder::setOverlayConfig(const dashcam::config::OverlayConfig& cfg) {
    std::lock_guard<std::mutex> lock(overlayMutex_);
    overlayConfig_ = cfg;
}

void SegmentedRecorder::setLogCallback(dashcam::log::LogCallback cb) { log_ = std::move(cb); }

void SegmentedRecorder::setPreviewTap(PreviewFrameCallback cb, uint32_t maxFps) {
    {
        std::lock_guard<std::mutex> lock(previewCfgMu_);
        previewCfgCb_  = std::move(cb);
        previewCfgFps_ = maxFps;
    }
    // The running session keeps its own copy (read on a streaming thread):
    // swapping it here would race that thread, so the change waits for start().
    bool active;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        active = pipeline_ != nullptr;
    }
    if (active)
        doLog(log_, LogLevel::INFO, "preview tap change stored — takes effect at the next start()");
}

bool SegmentedRecorder::isRecording() const {
    return pipeline_ != nullptr && healthy_.load(std::memory_order_relaxed);
}

std::string SegmentedRecorder::lastError() const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    return lastError_;
}

std::string SegmentedRecorder::currentFile() const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    return currentFile_;
}

uint64_t SegmentedRecorder::deletableBelowSeq() const {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (!pipeline_) return UINT64_MAX;
    // Everything this session assigned and saw closed is deletable; the open
    // (and still-finalising) segments and anything not yet assigned are not.
    return openSeqs_.empty() ? nextSeq_ : *openSeqs_.begin();
}

bool SegmentedRecorder::consumeFragmentClosed() {
    return fragmentClosed_.exchange(false);
}

void SegmentedRecorder::setError(const std::string& why) {
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (lastError_.empty()) lastError_ = why;
    }
    if (healthy_.exchange(false))
        doLog(log_, LogLevel::ERROR, "recording unhealthy: %s", why.c_str());
}

// ─── streaming-thread callbacks ───────────────────────────────────────────────

gchar* SegmentedRecorder::onFormatLocation(GstElement*, guint, GstSample*, gpointer user) {
    auto* self = static_cast<SegmentedRecorder*>(user);
    std::lock_guard<std::mutex> lock(self->stateMutex_);
    // SEQ is protected from retention BEFORE the file exists.
    const uint64_t seq = self->nextSeq_++;
    self->openSeqs_.insert(seq);
    const std::string path =
        self->opts_.dir + "/" + segmentFileName(self->opts_.prefix, seq, std::time(nullptr));
    self->currentFile_ = path;
    return g_strdup(path.c_str());
}

GstPadProbeReturn SegmentedRecorder::onRecqProbe(GstPad*, GstPadProbeInfo* info, gpointer user) {
    auto* self = static_cast<SegmentedRecorder*>(user);
    if (info->type & (GST_PAD_PROBE_TYPE_BUFFER | GST_PAD_PROBE_TYPE_BUFFER_LIST)) {
        self->lastBufferNs_.store(steadyNowNs(), std::memory_order_relaxed);
        self->bufferCount_.fetch_add(1, std::memory_order_relaxed);
        return GST_PAD_PROBE_OK;
    }
    if (info->type & GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM) {
        GstEvent* ev = GST_PAD_PROBE_INFO_EVENT(info);
        // splitmuxsink 1.20 aborts the process (g_assert in check_completed_gop)
        // on an EOS that precedes every buffer.  Nothing was recorded, so there
        // is nothing to finalise: swallow it.
        if (ev && GST_EVENT_TYPE(ev) == GST_EVENT_EOS &&
            self->bufferCount_.load(std::memory_order_relaxed) == 0)
            return GST_PAD_PROBE_DROP;
    }
    return GST_PAD_PROBE_OK;
}

GstFlowReturn SegmentedRecorder::onLumaSample(GstAppSink* sink, gpointer user) {
    auto* self = static_cast<SegmentedRecorder*>(user);
    GstSample* sample = gst_app_sink_pull_sample(sink);
    if (!sample) return GST_FLOW_OK;
    GstBuffer* buf = gst_sample_get_buffer(sample);
    GstMapInfo map;
    constexpr int kW = 64, kH = 36;                      // GRAY8: stride 64 (4-aligned)
    if (buf && gst_buffer_map(buf, &map, GST_MAP_READ)) {
        if (map.size >= size_t(kW) * kH) {
            // Lower two-thirds only: at dusk the sky would otherwise hold the
            // exposure down while the road goes black.
            uint64_t sum = 0;
            for (int y = kH / 3; y < kH; ++y)
                for (int x = 0; x < kW; ++x) sum += map.data[y * kW + x];
            self->luma_.store(static_cast<float>(sum) / float(kW * (kH - kH / 3)));
            self->lumaSeq_.fetch_add(1);
        }
        gst_buffer_unmap(buf, &map);
    }
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

bool SegmentedRecorder::splitNow() {
    if (!smx_ || bufferCount_.load() == 0) return false;
    // splitmuxsink's action signal: close the current fragment and open the next
    // at the upcoming keyframe (every MJPEG frame is one).  Gapless, like a
    // timed rollover; the new file is named by format-location with the
    // current clock.
    g_signal_emit_by_name(smx_, "split-now");
    doLog(log_, LogLevel::INFO, "segment split requested");
    return true;
}

GstPadProbeReturn SegmentedRecorder::onLumaGate(GstPad*, GstPadProbeInfo*, gpointer user) {
    // lumaq's sink pad, i.e. rectee's push into the luma branch, on the
    // recording's streaming thread: pass ~2 frames per second and drop the
    // rest before they are queued.  A probe takes no part in caps negotiation —
    // a videorate max-rate=2 here negotiated a RecordFps-capped recording down
    // to 2 fps through the tee.  GCRA on the arrival time (it cannot run
    // backwards): half a period of tolerance keeps 2/s down to RecordFps 3, and
    // a gap builds no credit.
    auto* self = static_cast<SegmentedRecorder*>(user);
    constexpr int64_t kPeriodNs = 500 * 1000000LL;
    const int64_t now = steadyNowNs();
    if (self->lumaDueNs_ != 0 && now + kPeriodNs / 2 < self->lumaDueNs_) return GST_PAD_PROBE_DROP;
    self->lumaDueNs_ = std::max(now, self->lumaDueNs_) + kPeriodNs;
    return GST_PAD_PROBE_OK;
}

GstPadProbeReturn SegmentedRecorder::onSourceProbe(GstPad*, GstPadProbeInfo*, gpointer user) {
    static_cast<SegmentedRecorder*>(user)->sourceCount_.fetch_add(1, std::memory_order_relaxed);
    return GST_PAD_PROBE_OK;
}

bool SegmentedRecorder::latestLuma(float& luma, uint64_t& seq) const {
    seq = lumaSeq_.load();
    if (seq == 0) return false;
    luma = luma_.load();
    return true;
}

// ─── preview tap (previewsink / previewq streaming threads) ──────────────────

void SegmentedRecorder::failPreview(const char* who, const char* what) {
    // The first failure of a session reports; onPreviewGuard() then cuts the
    // branch at the tee, so nothing of it reaches the camera source.
    if (previewFailed_.exchange(true)) return;
    doLog(log_, LogLevel::WARN, "preview tap error (%s): %s — preview stopped, recording continues",
          who, what);
}

GstPadProbeReturn SegmentedRecorder::onPreviewGuard(GstPad*, GstPadProbeInfo* info, gpointer user) {
    // previewq's sink pad, i.e. the tee's push into the preview branch.  A
    // leaky queue does not isolate errors: once its downstream returned a fatal
    // flow, its next chain call hands that flow back to the tee, the tee to the
    // camera source, and the recording dies.  So check the queue's last result
    // before every buffer and stop feeding a dead branch.  (A failure in the
    // microseconds between this check and the queue's chain can still slip
    // through; the appsink never returns one in practice.)
    auto* self = static_cast<SegmentedRecorder*>(user);
    const bool isEvent = (info->type & GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM) != 0;
    GstEvent*  ev      = isEvent ? GST_PAD_PROBE_INFO_EVENT(info) : nullptr;
    const bool isEos   = ev && GST_EVENT_TYPE(ev) == GST_EVENT_EOS;
    if (!self->previewFailed_.load(std::memory_order_relaxed)) {
        // EOS is checked like a buffer: once stop() closed the gate no buffer
        // checks any more, and a failure that landed just before would make
        // the queue refuse the EOS — and the pipeline-wide EOS never come.
        if (isEvent && !isEos) return GST_PAD_PROBE_OK;
        if (!isEvent && !self->previewGate_.load(std::memory_order_relaxed))
            return GST_PAD_PROBE_DROP;                   // stop() under way: nobody consumes it
        const GstFlowReturn last = gst_pad_get_last_flow_return(self->previewQSrc_);
        if (last == GST_FLOW_OK || last == GST_FLOW_FLUSHING) return GST_PAD_PROBE_OK;
        self->failPreview("previewq", gst_flow_get_name(last));
    }
    // Failed: nothing enters the branch any more — not even EOS, which a queue
    // holding an error turns into a bus ERROR.  The sink still has to reach EOS
    // for the pipeline-wide EOS that stop() waits on, so send it one directly.
    if (isEos && self->previewSinkPad_)
        gst_pad_send_event(self->previewSinkPad_, gst_event_new_eos());
    return GST_PAD_PROBE_DROP;
}

void SegmentedRecorder::updatePreviewCaps(GstCaps* caps) {
    // Re-read only when the caps object changes; the held ref keeps the
    // pointer comparison safe from address reuse.
    if (!caps || caps == previewCaps_) return;
    gst_caps_replace(&previewCaps_, caps);
    const GstStructure* st = gst_caps_get_size(caps) > 0 ? gst_caps_get_structure(caps, 0) : nullptr;
    gint w = 0, h = 0;
    if (st && gst_structure_get_int(st, "width", &w) && gst_structure_get_int(st, "height", &h) &&
        w > 0 && h > 0) {
        previewW_ = static_cast<uint32_t>(w);
        previewH_ = static_cast<uint32_t>(h);
    } else {                                             // no size in the caps: the requested one
        previewW_ = videoW_;
        previewH_ = videoH_;
    }
}

uint64_t SegmentedRecorder::previewCaptureUs(GstAppSink* sink, GstSample* sample, GstBuffer* buf,
                                             uint64_t arrivalUs) {
    if (previewClock_ == 0) {
        // Decided once per session; the clock is fixed once frames flow.  The
        // exact type matters: GstNetClientClock & co. derive from GstSystemClock
        // but do not tick CLOCK_MONOTONIC.
        if (GstClock* clock = gst_element_get_clock(GST_ELEMENT(sink))) {
            GstClockType type = GST_CLOCK_TYPE_REALTIME;
            const bool system = G_OBJECT_TYPE(clock) == GST_TYPE_SYSTEM_CLOCK;
            if (system) g_object_get(clock, "clock-type", &type, nullptr);
            gst_object_unref(clock);
            previewClock_ = (system && type == GST_CLOCK_TYPE_MONOTONIC) ? 1 : 2;
            doLog(log_, LogLevel::INFO, previewClock_ == 1
                      ? "preview tap: capture times from buffer PTS (pipeline clock is CLOCK_MONOTONIC)"
                      : "preview tap: pipeline clock is not CLOCK_MONOTONIC — capture times are "
                        "appsink arrival times");
        }
    }
    uint64_t us = arrivalUs;
    if (previewClock_ == 1) {
        // Running time + base time = the clock reading at capture (v4l2src
        // derives the PTS from the kernel's capture timestamp).
        const GstSegment*  seg  = gst_sample_get_segment(sample);
        const GstClockTime pts  = GST_BUFFER_PTS(buf);
        const GstClockTime base = gst_element_get_base_time(GST_ELEMENT(sink));
        if (seg && seg->format == GST_FORMAT_TIME && GST_CLOCK_TIME_IS_VALID(pts) &&
            GST_CLOCK_TIME_IS_VALID(base)) {
            const GstClockTime rt = gst_segment_to_running_time(seg, GST_FORMAT_TIME, pts);
            if (GST_CLOCK_TIME_IS_VALID(rt)) us = std::min<uint64_t>(arrivalUs, (base + rt) / 1000);
        }
    }
    return std::max(us, previewLastUs_);                 // non-decreasing within the session
}

void SegmentedRecorder::deliverPreview(GstAppSink* sink, GstSample* sample, GstBuffer* buf) {
    const uint64_t captureUs = previewCaptureUs(sink, sample, buf, monotonicNowUs());
    // The branch's only rate limit (every camera frame gets here).  GCRA:
    // conforming when no more than a quarter period early, so deliveries are
    // never closer than 3/4 of a period, and no credit builds up behind a slow
    // camera or callback — no catch-up burst after one.
    if (previewDueUs_ != 0 && captureUs + previewPeriodUs_ / 4 < previewDueUs_) return;
    GstMapInfo map;
    if (!gst_buffer_map(buf, &map, GST_MAP_READ)) return;
    if (map.size > 0) {
        previewDueUs_  = std::max(captureUs, previewDueUs_) + previewPeriodUs_;
        previewLastUs_ = captureUs;
        updatePreviewCaps(gst_sample_get_caps(sample));
        PreviewFrame f;
        f.data               = map.data;
        f.size               = map.size;
        f.width              = previewW_;
        f.height             = previewH_;
        f.v4l2PixFmt         = V4L2_PIX_FMT_MJPEG;
        f.captureMonotonicUs = captureUs;
        f.sequence           = previewCount_.fetch_add(1) + 1;
        // An exception must not unwind through GStreamer's C frames.
        try {
            previewCb_(f);
        } catch (const std::exception& e) {
            failPreview("callback", e.what());
        } catch (...) {
            failPreview("callback", "unknown exception");
        }
    }
    gst_buffer_unmap(buf, &map);
}

GstFlowReturn SegmentedRecorder::onPreviewSample(GstAppSink* sink, gpointer user) {
    auto* self = static_cast<SegmentedRecorder*>(user);
    GstSample* sample = gst_app_sink_pull_sample(sink);
    if (!sample) return GST_FLOW_OK;
    GstBuffer* buf = gst_sample_get_buffer(sample);
    if (buf && self->previewGate_.load() && !self->previewFailed_.load())
        self->deliverPreview(sink, sample, buf);
    gst_sample_unref(sample);
    return GST_FLOW_OK;              // never an error: it would travel up the tee to the camera
}

void SegmentedRecorder::releasePreview() {
    // Pipeline at NULL: every streaming thread has stopped, so no callback runs.
    if (previewQSrc_)    gst_object_unref(previewQSrc_);
    if (previewSinkPad_) gst_object_unref(previewSinkPad_);
    previewQSrc_    = nullptr;
    previewSinkPad_ = nullptr;
    gst_caps_replace(&previewCaps_, nullptr);
    previewCb_ = nullptr;
    previewBranch_.store(false);
    previewGate_.store(false);
}

// ─── bus handling (worker thread; stop() thread after the worker joined) ─────

void SegmentedRecorder::handleMessage(GstMessage* msg) {
    switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_ERROR: {
        GError* err = nullptr;
        gchar*  dbg = nullptr;
        gst_message_parse_error(msg, &err, &dbg);
        // A failure inside the luma tap costs exposure control, not footage.
        const char* src = GST_MESSAGE_SRC_NAME(msg);
        if (src && g_str_has_prefix(src, "luma")) {
            doLog(log_, LogLevel::WARN, "luma tap error (%s): %s — exposure control starved",
                  src, err ? err->message : "?");
            if (err) g_error_free(err);
            g_free(dbg);
            break;
        }
        // Likewise the preview tap costs the display, not footage: stop
        // feeding it (onPreviewGuard) and keep recording.
        if (src && g_str_has_prefix(src, "preview")) {
            doLog(log_, LogLevel::WARN, "preview tap error (%s): %s — preview stopped",
                  src, err ? err->message : "?");
            previewFailed_.store(true);
            if (err) g_error_free(err);
            g_free(dbg);
            break;
        }
        const std::string text = std::string("pipeline error: ") + (err ? err->message : "?");
        doLog(log_, LogLevel::DEBUG, "recording pipeline error detail: %s", dbg ? dbg : "-");
        if (err) g_error_free(err);
        g_free(dbg);
        errorSeen_.store(true);
        setError(text);
        break;
    }
    case GST_MESSAGE_EOS:
        eosSeen_.store(true);
        if (!stopFlag_.load()) setError("unexpected end of stream from the camera");
        break;
    case GST_MESSAGE_ELEMENT: {
        const GstStructure* st = gst_message_get_structure(msg);
        if (!st) break;
        const char* name = gst_structure_get_name(st);
        const char* loc  = gst_structure_get_string(st, "location");
        if (!name || !loc) break;
        if (g_str_equal(name, "splitmuxsink-fragment-opened")) {
            guint64 rt = 0;
            gst_structure_get_uint64(st, "running-time", &rt);
            onFragmentOpened(loc, static_cast<int64_t>(rt));
        } else if (g_str_equal(name, "splitmuxsink-fragment-closed")) {
            onFragmentClosed(loc);
        }
        break;
    }
    default:
        break;
    }
}

void SegmentedRecorder::onFragmentOpened(const std::string& location, int64_t rtNs) {
    doLog(log_, LogLevel::INFO, "segment opened: %s", location.c_str());
    const bool durable = opts_.syncIntervalMs > 0;
    if (!assEnabled_) {
        if (durable) queueSync(dirOf(location), true);
        return;
    }

    closeSidecar();
    assPath_ = location;
    const auto dot = assPath_.find_last_of('.');
    if (dot != std::string::npos && dot > assPath_.find_last_of('/')) assPath_.erase(dot);
    assPath_ += ".ass";
    assFile_.open(assPath_, std::ios::trunc);
    if (!assFile_.is_open()) {
        doLog(log_, LogLevel::ERROR, "cannot open telemetry sidecar %s — segment without it",
              assPath_.c_str());
        fragStartRtNs_ = -1;
        return;
    }
    dashcam::config::OverlayConfig cfg;
    {
        std::lock_guard<std::mutex> lock(overlayMutex_);
        cfg = overlayConfig_;
    }
    detail::writeAssHeader(assFile_, cfg, videoW_, videoH_);
    fragStartRtNs_ = rtNs;
    if (durable) {
        setSyncSidecar(assPath_);
        queueSync(dirOf(location), true);        // both new names: segment + sidecar
    }

    // The fragment-opened message arrives up to one GOP after the fragment's
    // first frame; replay what was sampled since then into the new sidecar.
    const int64_t durNs = static_cast<int64_t>(1e9 / std::max(0.5f, (float)cfg.subtitleRateHz));
    for (const Sample& s : ring_)
        if (s.rtNs >= rtNs)
            detail::writeAssSample(assFile_, s.rtNs - rtNs, durNs, s.od, s.wallMs, s.stale.speed,
                                   s.stale.accel, s.stale.position, s.stale.heading, s.stale.detail);
}

void SegmentedRecorder::onFragmentClosed(const std::string& location) {
    const auto seq = parseSegmentName(baseName(location), opts_.prefix);
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (seq) openSeqs_.erase(*seq);
    }
    // The muxer has written the index: make the finished file durable.
    queueSync(location);
    fragmentClosed_.store(true);
    doLog(log_, LogLevel::INFO, "segment closed: %s", location.c_str());
}

void SegmentedRecorder::closeSidecar() {
    if (!assFile_.is_open()) return;
    assFile_.flush();
    assFile_.close();
    if (opts_.syncIntervalMs > 0) {
        setSyncSidecar("");
        queueSync(assPath_);                     // its final contents
    }
}

// ─── durability: the sync thread ──────────────────────────────────────────────

void SegmentedRecorder::queueSync(const std::string& path, bool directory) {
    if (opts_.syncIntervalMs == 0 || path.empty()) return;
    {
        std::lock_guard<std::mutex> lock(syncMu_);
        syncJobs_.emplace_back(path, directory);
    }
    syncCv_.notify_one();
}

void SegmentedRecorder::setSyncSidecar(const std::string& path) {
    std::lock_guard<std::mutex> lock(syncMu_);
    syncAssPath_ = path;
}

void SegmentedRecorder::stopSyncThread() {
    if (!syncThread_.joinable()) return;
    const std::string current = currentFile();   // after a forced teardown there is
    {                                            // no close message: sync it anyway
        std::lock_guard<std::mutex> lock(syncMu_);
        if (!current.empty()) syncJobs_.emplace_back(current, false);
        syncStop_ = true;
    }
    syncCv_.notify_all();
    syncThread_.join();                          // inside stop()'s hard deadline
}

void SegmentedRecorder::syncLoop() {
    using clock = std::chrono::steady_clock;
    const auto every  = std::chrono::milliseconds(opts_.syncIntervalMs);
    const int64_t slowMs = std::max<int64_t>(250, opts_.syncIntervalMs / 2);
    auto next = clock::now() + every;

    // Descriptors live only on this thread.  fdatasync through our own
    // O_RDONLY descriptor flushes the file (fsync is per inode), whatever
    // descriptor filesink / the sidecar stream wrote through.
    int mkvFd = -1, assFd = -1;
    std::string mkvPath, assPath;

    // Rate-limited reporting: a disk that errors or cannot keep up makes the
    // loss window longer than configured — say so, at most once a minute.
    int slowCount = 0, failCount = 0;
    int64_t slowMax = 0;
    std::string lastErr;
    auto lastReport = clock::now() - std::chrono::minutes(1);

    auto syncOne = [&](int fd, bool directory) {
        if (syncTestHook_) syncTestHook_();
        const int64_t t0 = steadyNowNs();
        if ((directory ? ::fsync(fd) : ::fdatasync(fd)) != 0) {
            ++failCount;
            lastErr = std::strerror(errno);
        } else if (!directory) {
            syncedFiles_.fetch_add(1, std::memory_order_relaxed);
        }
        const int64_t ms = (steadyNowNs() - t0) / 1000000;
        if (ms > slowMs) { ++slowCount; slowMax = std::max(slowMax, ms); }
    };
    // A file that does not exist (yet, or any more) is not a failure; any other
    // open error means that file is not being made durable — report it.
    auto openFailed = [&](const std::string& path) {
        if (errno == ENOENT) return;
        ++failCount;
        lastErr = "open " + baseName(path) + ": " + std::strerror(errno);
    };
    auto track = [&](int& fd, std::string& open, const std::string& want) {
        if (want == open) return;
        if (fd >= 0) ::close(fd);
        fd   = want.empty() ? -1 : ::open(want.c_str(), O_RDONLY | O_CLOEXEC);
        if (fd < 0 && !want.empty()) openFailed(want);
        open = fd >= 0 ? want : std::string();   // not created yet: retry next round
    };

    std::unique_lock<std::mutex> lk(syncMu_);
    for (;;) {
        syncCv_.wait_until(lk, next, [&] { return syncStop_ || !syncJobs_.empty(); });
        auto jobs = std::move(syncJobs_);
        syncJobs_.clear();
        const std::string wantAss = syncAssPath_;
        const bool stopping = syncStop_;
        lk.unlock();
        syncBusySinceNs_.store(steadyNowNs());          // the watchdog times each round

        for (const auto& [path, directory] : jobs) {
            // A closed file we were tracking: sync it through our descriptor.
            if (!directory && path == mkvPath && mkvFd >= 0) {
                syncOne(mkvFd, false); ::close(mkvFd); mkvFd = -1; mkvPath.clear(); continue;
            }
            if (!directory && path == assPath && assFd >= 0) {
                syncOne(assFd, false); ::close(assFd); assFd = -1; assPath.clear(); continue;
            }
            const int fd = ::open(path.c_str(), (directory ? O_DIRECTORY : 0) | O_RDONLY | O_CLOEXEC);
            if (fd >= 0) { syncOne(fd, directory); ::close(fd); }
            else         openFailed(path);
        }

        if (!stopping && clock::now() >= next) {
            // The segment being written is whatever format-location named last.
            track(mkvFd, mkvPath, currentFile());
            track(assFd, assPath, wantAss);
            if (mkvFd >= 0) syncOne(mkvFd, false);
            if (assFd >= 0) syncOne(assFd, false);
            syncCount_.fetch_add(1);
            next += every;                                    // keep the cadence...
            if (next <= clock::now()) next = clock::now() + every;   // ...but never burst
        }

        if ((slowCount || failCount) && clock::now() - lastReport >= std::chrono::minutes(1)) {
            if (failCount)
                doLog(log_, LogLevel::ERROR, "disk sync failed %d time(s) (%s) — footage may not "
                      "survive a power cut", failCount, lastErr.c_str());
            if (slowCount)
                doLog(log_, LogLevel::WARN, "disk sync slow %d time(s), up to %lld ms (interval %u ms) "
                      "— a power cut can lose more than the interval", slowCount,
                      static_cast<long long>(slowMax), opts_.syncIntervalMs);
            slowCount = failCount = 0;
            slowMax = 0;
            lastReport = clock::now();
        }

        syncBusySinceNs_.store(0);
        if (stopping) break;
        lk.lock();
    }
    syncBusySinceNs_.store(steadyNowNs());
    for (int* fd : {&mkvFd, &assFd})
        if (*fd >= 0) { syncOne(*fd, false); ::close(*fd); *fd = -1; }
    syncBusySinceNs_.store(0);
}

// ─── watchdog + sidecar sampling (worker thread) ──────────────────────────────

// ─── watchdog thread ──────────────────────────────────────────────────────────

// No filesystem I/O here or in checkWatchdog(): only atomics, setError()
// (stateMutex_, never held across I/O) and log calls, which only enqueue
// (liblog: async, overrun_oldest).  So a disk that blocks the worker and the
// sync thread cannot silence the checks that let the caller start recovery.
void SegmentedRecorder::monitorLoop() {
    for (;;) {
        {
            std::unique_lock<std::mutex> lk(monMu_);
            if (monCv_.wait_for(lk, std::chrono::milliseconds(200), [&] { return monStop_; })) return;
        }
        checkWatchdog();
    }
}

void SegmentedRecorder::checkWatchdog() {
    if (!healthy_.load()) return;
    const int64_t now = steadyNowNs();

    // Disk sync progress is watched from here, the monitor thread: a stuck
    // fdatasync never returns to report itself, and recording would carry on
    // "healthy" — and the worker may be stuck on the same disk.
    const int64_t busy = syncBusySinceNs_.load();
    if (syncWarnedFor_ != 0 && busy != syncWarnedFor_) {        // that round finished
        doLog(log_, LogLevel::INFO, "disk sync responding again after ~%lld ms",
              static_cast<long long>((now - syncWarnedFor_) / 1000000));
        syncWarnedFor_ = 0;
    }
    if (busy != 0 && opts_.syncIntervalMs > 0) {
        const int64_t stuckMs = (now - busy) / 1000000;
        const int64_t warnMs  = std::max<int64_t>(3000, 2 * int64_t(opts_.syncIntervalMs));
        // A long interval means long healthy rounds (more to flush): the limit
        // scales with it and always leaves room for the warning first.
        const int64_t stallMs = opts_.syncStallTimeoutMs == 0
                                    ? 0 : std::max<int64_t>(opts_.syncStallTimeoutMs, 2 * warnMs);
        if (stuckMs >= warnMs && syncWarnedFor_ != busy) {
            syncWarnedFor_ = busy;
            doLog(log_, LogLevel::WARN, "disk sync not responding for %lld ms — footage written "
                  "since then may not survive a power cut", static_cast<long long>(stuckMs));
        }
        if (stallMs > 0 && stuckMs >= stallMs) {
            setError("disk sync stuck for " + std::to_string(stuckMs) + " ms (disk not responding)");
            return;
        }
    }

    if (bufferCount_.load() == 0) {
        if (now - playingAtNs_ > int64_t(opts_.firstFrameTimeoutMs) * 1000000)
            setError("no first frame within " + std::to_string(opts_.firstFrameTimeoutMs) + " ms");
        return;
    }
    if (opts_.stallTimeoutMs > 0 &&
        now - lastBufferNs_.load() > int64_t(opts_.stallTimeoutMs) * 1000000)
        setError("no frames for " + std::to_string(opts_.stallTimeoutMs) + " ms (camera stalled)");
}

int64_t SegmentedRecorder::runningTimeNs() const {
    GstClock* clock = gst_element_get_clock(pipeline_);
    if (!clock) return -1;
    const GstClockTime now  = gst_clock_get_time(clock);
    const GstClockTime base = gst_element_get_base_time(pipeline_);
    gst_object_unref(clock);
    if (now == GST_CLOCK_TIME_NONE || base == GST_CLOCK_TIME_NONE || now < base) return -1;
    return static_cast<int64_t>(now - base);
}

void SegmentedRecorder::takeSample(int64_t durNs, int64_t staleMs) {
    if (bufferCount_.load() == 0) return;          // nothing is being recorded yet
    const int64_t rt = runningTimeNs();
    if (rt < 0) return;

    Sample s;
    s.rtNs = rt;
    {
        std::lock_guard<std::mutex> lock(overlayMutex_);
        s.od = overlayData_;
    }
    s.wallMs = wallNowMs();
    s.stale  = detail::assStaleness(s.od, s.wallMs, staleMs);

    ring_.push_back(s);
    // A GOP is at most a few seconds; 15 s of ring covers any fragment-opened lag.
    while (!ring_.empty() && ring_.front().rtNs < rt - 15'000'000'000LL) ring_.pop_front();

    if (assFile_.is_open() && fragStartRtNs_ >= 0 && rt >= fragStartRtNs_)
        detail::writeAssSample(assFile_, rt - fragStartRtNs_, durNs, s.od, s.wallMs, s.stale.speed,
                               s.stale.accel, s.stale.position, s.stale.heading, s.stale.detail);
}

void SegmentedRecorder::workerLoop() {
    float   rateHz;
    int64_t staleMs;
    {
        std::lock_guard<std::mutex> lock(overlayMutex_);
        rateHz  = (float)overlayConfig_.subtitleRateHz;
        staleMs = (int64_t)(int)overlayConfig_.staleTimeoutMs;
    }
    if (rateHz <= 0.0f) rateHz = 5.0f;
    const auto    interval = std::chrono::milliseconds(static_cast<int64_t>(1000.0f / rateHz));
    const int64_t durNs    = static_cast<int64_t>(1e9 / rateHz);

    GstBus* bus = gst_element_get_bus(pipeline_);
    // APPLICATION: stop()'s wake-up, so the worker exits without waiting out
    // its poll interval (up to 2 s at the lowest subtitle rate).
    const auto types = static_cast<GstMessageType>(
        GST_MESSAGE_ERROR | GST_MESSAGE_EOS | GST_MESSAGE_ELEMENT | GST_MESSAGE_APPLICATION);
    // With durability on, each sample goes to the page cache at once (a cheap
    // write) and the sync thread pushes it to disk; otherwise flush every 5 s.
    const bool durable = opts_.syncIntervalMs > 0;
    auto nextFlush = std::chrono::steady_clock::now() + std::chrono::seconds(5);

    while (!stopFlag_.load()) {
        // Wake early for bus traffic so segment rotations are handled promptly.
        if (bus) {
            if (GstMessage* msg = gst_bus_timed_pop_filtered(
                    bus, std::chrono::duration_cast<std::chrono::nanoseconds>(interval).count(),
                    types)) {
                handleMessage(msg);
                gst_message_unref(msg);
                while (GstMessage* more = gst_bus_pop_filtered(bus, types)) {
                    handleMessage(more);
                    gst_message_unref(more);
                }
            }
        } else {
            std::this_thread::sleep_for(interval);
        }

        if (assEnabled_) takeSample(durNs, staleMs);

        if (assFile_.is_open()) {
            if (workerTestHook_) workerTestHook_();

            if (durable) {
                assFile_.flush();
            } else if (std::chrono::steady_clock::now() >= nextFlush) {
                assFile_.flush();
                nextFlush = std::chrono::steady_clock::now() + std::chrono::seconds(5);
            }
        }
    }
    if (bus) gst_object_unref(bus);
}

// ─── lifecycle ────────────────────────────────────────────────────────────────

bool SegmentedRecorder::start(const std::string& devicePath, const RecordingFormat& fmt,
                              const SegmentOptions& opts) {
    if (pipeline_) {
        doLog(log_, LogLevel::ERROR, "segmented recording: session already active");
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        lastError_.clear();
        currentFile_.clear();
        openSeqs_.clear();
    }
    auto fail = [this](const std::string& why) {
        {
            std::lock_guard<std::mutex> lock(stateMutex_);
            lastError_ = why;
        }
        doLog(log_, LogLevel::ERROR, "segmented recording failed to start: %s", why.c_str());
        return false;
    };

    if (fmt.width == 0 || fmt.height == 0 || !(fmt.fps > 0.0f))
        return fail("invalid format");
    const bool mjpeg = fmt.v4l2PixFmt == V4L2_PIX_FMT_MJPEG;
    const bool h264  = fmt.v4l2PixFmt == V4L2_PIX_FMT_H264;
    if (!mjpeg && !h264)
        return fail("pixel format is raw — only MJPG/H264 passthrough is recordable (no NVENC)");
    if (opts.dir.empty() || opts.prefix.empty() || opts.segmentSec == 0)
        return fail("invalid segment options");

    opts_ = opts;

    // This session's copy of the preview tap (see setPreviewTap()).
    PreviewFrameCallback previewCb;
    uint32_t             previewFps = 0;
    {
        std::lock_guard<std::mutex> lock(previewCfgMu_);
        previewCb  = previewCfgCb_;
        previewFps = std::min<uint32_t>(previewCfgFps_, 1000);
    }
    const bool previewWanted = previewCb && previewFps > 0;
    const bool previewTap    = previewWanted && mjpeg;
    if (previewWanted && !mjpeg)
        doLog(log_, LogLevel::INFO, "preview tap needs an MJPEG source — this H.264 session records "
              "without a preview");

    gint frNum = 30, frDen = 1;
    gst_util_double_to_fraction(static_cast<double>(fmt.fps), &frNum, &frDen);
    const std::string rate = std::to_string(frNum) + "/" + std::to_string(frDen);

    std::string desc;
    if (testSourceDesc_.empty()) {
        // Keep the framerate pinned here: the recrate cap's videorate takes
        // part in caps negotiation, so with a free rate it would slow the
        // camera itself to RecordFps — and the preview tap and
        // sourceFramesReceived() with it.
        desc = "v4l2src name=camerasrc device=" + detail::gstQuoted(devicePath) +
               (mjpeg ? " ! image/jpeg" : " ! video/x-h264") +
               ", width=" + std::to_string(fmt.width) +
               ", height=" + std::to_string(fmt.height) + ", framerate=" + rate;
    } else {
        desc = testSourceDesc_;
    }
    // Preview tap: split the camera's own frames off BEFORE the RecordFps cap,
    // so the cap does not cap the preview.  The tee carries the recording, so
    // its failure is the recording's: it is deliberately not named preview*.
    if (previewTap) desc += " ! tee name=srctee srctee.";
    // MJPEG frames are intra-only, so dropping them to cap the rate is safe.
    if (mjpeg && opts.maxFps > 0 && static_cast<float>(opts.maxFps) < fmt.fps)
        desc += " ! videorate name=recrate drop-only=true max-rate=" + std::to_string(opts.maxFps);
    if (h264) desc += " ! h264parse";
    const bool lumaTap = lumaTap_ && mjpeg;
    if (lumaTap) desc += " ! tee name=rectee rectee.";
    desc += " ! queue name=recq max-size-buffers=8 leaky=0"
            " ! splitmuxsink name=smx async-finalize=true muxer-factory=matroskamux"
            " muxer-properties=\"properties,offset-to-zero=true\""
            " sink-properties=\"properties,sync=false,async=false,buffer-mode=(int)2\""
            " max-size-time=" + std::to_string(uint64_t(opts.segmentSec) * GST_SECOND);
    // Luma tap: a leaky branch that decodes ~2 frames/s at thumbnail size
    // (onLumaGate() drops the rest ahead of lumaq; no videorate, which would
    // negotiate the recording's rate through rectee).  Its elements are named
    // luma* so a failure inside it is not mistaken for a recording failure (see
    // handleMessage); max-errors=-1 keeps a corrupt JPEG from ever turning into
    // a pipeline error.
    if (lumaTap)
        desc += " rectee. ! queue name=lumaq max-size-buffers=1 leaky=downstream"
                " ! jpegdec name=lumadec max-errors=-1 ! videoscale name=lumascale"
                " ! videoconvert name=lumaconv ! video/x-raw,format=GRAY8,width=64,height=36"
                " ! appsink name=lumasink emit-signals=false sync=false async=false"
                " max-buffers=1 drop=true";
    // Preview tap: leaky like the luma branch, but compressed (the consumer
    // does a DCT-scaled decode off the streaming thread) and rate-limited in
    // deliverPreview() — no videorate, so nothing here can negotiate the
    // camera's rate.  Every element is named preview* (see handleMessage),
    // onPreviewGuard() cuts the branch at the tee if it fails, and
    // enable-last-sample=false keeps the sink from pinning a camera buffer
    // between frames.
    if (previewTap)
        desc += " srctee. ! queue name=previewq max-size-buffers=1 leaky=downstream"
                " ! appsink name=previewsink emit-signals=false sync=false async=false"
                " max-buffers=1 drop=true enable-last-sample=false";

    doLog(log_, LogLevel::INFO, "segmented recording pipeline: %s", desc.c_str());

    GError* err = nullptr;
    GstElement* pipe = gst_parse_launch(desc.c_str(), &err);
    if (!pipe || err) {
        const std::string why = std::string("pipeline parse failed: ") + (err ? err->message : "?");
        if (err) g_error_free(err);
        if (pipe) gst_object_unref(pipe);
        return fail(why);
    }
    GstElement* smx  = gst_bin_get_by_name(GST_BIN(pipe), "smx");
    GstElement* recq = gst_bin_get_by_name(GST_BIN(pipe), "recq");
    if (!smx || !recq) {
        if (smx) gst_object_unref(smx);
        if (recq) gst_object_unref(recq);
        gst_object_unref(pipe);
        return fail("pipeline is missing smx/recq");
    }

    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        sessionStartSeq_ = nextSegmentSeq(opts.dir, opts.prefix);
        nextSeq_         = sessionStartSeq_;
    }
    g_signal_connect(smx, "format-location-full", G_CALLBACK(&SegmentedRecorder::onFormatLocation),
                     this);
    lumaDueNs_ = 0;
    if (lumaTap) {
        if (GstElement* q = gst_bin_get_by_name(GST_BIN(pipe), "lumaq")) {
            GstPad* in = gst_element_get_static_pad(q, "sink");
            gst_pad_add_probe(in,
                              static_cast<GstPadProbeType>(GST_PAD_PROBE_TYPE_BUFFER |
                                                           GST_PAD_PROBE_TYPE_BUFFER_LIST),
                              &SegmentedRecorder::onLumaGate, this, nullptr);
            gst_object_unref(in);
            gst_object_unref(q);
        }
        if (GstElement* sink = gst_bin_get_by_name(GST_BIN(pipe), "lumasink")) {
            GstAppSinkCallbacks cbs;
            std::memset(&cbs, 0, sizeof(cbs));
            cbs.new_sample = &SegmentedRecorder::onLumaSample;
            gst_app_sink_set_callbacks(GST_APP_SINK(sink), &cbs, this, nullptr);
            gst_object_unref(sink);
        }
    }
    previewCount_.store(0);
    previewFailed_.store(false);
    if (previewTap) {
        GstElement* q    = gst_bin_get_by_name(GST_BIN(pipe), "previewq");
        GstElement* sink = gst_bin_get_by_name(GST_BIN(pipe), "previewsink");
        if (q && sink) {
            previewQSrc_    = gst_element_get_static_pad(q, "src");
            previewSinkPad_ = gst_element_get_static_pad(sink, "sink");
            GstPad* in = gst_element_get_static_pad(q, "sink");
            gst_pad_add_probe(in,
                              static_cast<GstPadProbeType>(GST_PAD_PROBE_TYPE_BUFFER |
                                                           GST_PAD_PROBE_TYPE_BUFFER_LIST |
                                                           GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM),
                              &SegmentedRecorder::onPreviewGuard, this, nullptr);
            gst_object_unref(in);
            GstAppSinkCallbacks cbs;
            std::memset(&cbs, 0, sizeof(cbs));
            cbs.new_sample = &SegmentedRecorder::onPreviewSample;
            gst_app_sink_set_callbacks(GST_APP_SINK(sink), &cbs, this, nullptr);
            previewCb_       = std::move(previewCb);
            previewPeriodUs_ = 1000000ULL / previewFps;
            previewW_        = fmt.width;
            previewH_        = fmt.height;
            previewClock_    = 0;
            previewLastUs_   = 0;
            previewDueUs_    = 0;
            previewBranch_.store(true);
            previewGate_.store(true);
        } else {                                 // never expected: costs the preview only
            doLog(log_, LogLevel::WARN, "preview tap: branch elements missing — recording without "
                  "a preview");
        }
        if (q) gst_object_unref(q);
        if (sink) gst_object_unref(sink);
    }
    luma_.store(0.0f);
    lumaSeq_.store(0);
    sourceCount_.store(0);
    hasRateCap_ = false;
    if (GstElement* rate = gst_bin_get_by_name(GST_BIN(pipe), "recrate")) {
        GstPad* in = gst_element_get_static_pad(rate, "sink");
        gst_pad_add_probe(in, GST_PAD_PROBE_TYPE_BUFFER, &SegmentedRecorder::onSourceProbe, this, nullptr);
        gst_object_unref(in);
        gst_object_unref(rate);
        hasRateCap_ = true;
    }
    smx_ = smx;                                  // kept for splitNow()
    GstPad* sinkPad = gst_element_get_static_pad(recq, "sink");
    gst_pad_add_probe(sinkPad,
                      static_cast<GstPadProbeType>(GST_PAD_PROBE_TYPE_BUFFER |
                                                   GST_PAD_PROBE_TYPE_BUFFER_LIST |
                                                   GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM),
                      &SegmentedRecorder::onRecqProbe, this, nullptr);
    gst_object_unref(sinkPad);

    {
        std::lock_guard<std::mutex> lock(overlayMutex_);
        assEnabled_ = (bool)overlayConfig_.enabled;
    }
    videoW_ = fmt.width;
    videoH_ = fmt.height;
    bufferCount_.store(0);
    lastBufferNs_.store(0);
    eosSeen_.store(false);
    errorSeen_.store(false);
    fragmentClosed_.store(false);
    stopFlag_.store(false);
    fragStartRtNs_ = -1;
    ring_.clear();
    syncCount_.store(0);
    syncedFiles_.store(0);
    syncBusySinceNs_.store(0);
    syncWarnedFor_ = 0;
    {
        std::lock_guard<std::mutex> lock(syncMu_);
        syncStop_ = false;
        syncAssPath_.clear();
        syncJobs_.clear();
    }

    // pipeline_ is published under stateMutex_ so deletableBelowSeq() sees a
    // consistent session.
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        pipeline_ = pipe;
    }
    recq_ = recq;
    playingAtNs_ = steadyNowNs();
    if (gst_element_set_state(pipe, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        const std::string busErr = popBusError(pipe);
        gst_element_set_state(pipe, GST_STATE_NULL);
        releasePreview();
        gst_object_unref(recq_);
        recq_ = nullptr;
        gst_object_unref(smx_);
        smx_ = nullptr;
        {
            std::lock_guard<std::mutex> lock(stateMutex_);
            pipeline_ = nullptr;
            openSeqs_.clear();
        }
        gst_object_unref(pipe);
        return fail("pipeline refused to start on " + devicePath +
                    (busErr.empty() ? "" : ": " + busErr));
    }

    healthy_.store(true);
    // The sync thread first: the worker may hand it work as soon as it runs.
    if (opts.syncIntervalMs > 0) syncThread_ = std::thread(&SegmentedRecorder::syncLoop, this);
    worker_ = std::thread(&SegmentedRecorder::workerLoop, this);
    {
        std::lock_guard<std::mutex> lk(monMu_);
        monStop_ = false;
    }
    monitor_ = std::thread(&SegmentedRecorder::monitorLoop, this);
    doLog(log_, LogLevel::INFO, "segmented recording started on %s (%ux%u %s @%.1f, %u s segments)",
          devicePath.c_str(), fmt.width, fmt.height, mjpeg ? "MJPEG" : "H264",
          static_cast<double>(fmt.fps), opts.segmentSec);
    return true;
}

void SegmentedRecorder::teardown() {
    GstBus* bus = gst_element_get_bus(pipeline_);
    const auto types = static_cast<GstMessageType>(
        GST_MESSAGE_ERROR | GST_MESSAGE_EOS | GST_MESSAGE_ELEMENT);

    // Drain what the worker left, so a late ERROR/EOS decides the path below.
    if (bus) {
        while (GstMessage* msg = gst_bus_pop_filtered(bus, types)) {
            handleMessage(msg);
            gst_message_unref(msg);
        }
    }

    const bool haveFrames = bufferCount_.load() > 0;
    bool finalised = !haveFrames || eosSeen_.load();
    if (!finalised) {
        if (errorSeen_.load()) {
            // The source's streaming thread is gone (unplug / device error), so a
            // pipeline EOS would never travel.  Inject it at the record queue:
            // queue forwards it on its own thread and splitmuxsink finalises.
            GstPad* sinkPad = gst_element_get_static_pad(recq_, "sink");
            gst_pad_send_event(sinkPad, gst_event_new_eos());
            gst_object_unref(sinkPad);
        } else {
            gst_element_send_event(pipeline_, gst_event_new_eos());
        }
        const int64_t deadline = steadyNowNs() + int64_t(opts_.eosTimeoutMs) * 1000000;
        while (bus && !finalised) {
            const int64_t left = deadline - steadyNowNs();
            if (left <= 0) break;
            GstMessage* msg = gst_bus_timed_pop_filtered(bus, left, types);
            if (!msg) break;
            // After an error the pipeline never posts EOS (its source is not EOS),
            // so the segment's close message is the completion signal there.
            const bool closed =
                GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ELEMENT && gst_message_get_structure(msg) &&
                gst_structure_has_name(gst_message_get_structure(msg),
                                       "splitmuxsink-fragment-closed");
            handleMessage(msg);
            gst_message_unref(msg);
            if (eosSeen_.load()) finalised = true;
            else if (closed && errorSeen_.load()) {
                std::lock_guard<std::mutex> lock(stateMutex_);
                finalised = openSeqs_.empty();
            }
        }
        if (!finalised)
            doLog(log_, LogLevel::WARN, "segment finalise timed out after %u ms; forcing teardown",
                  opts_.eosTimeoutMs);
    }

    gst_element_set_state(pipeline_, GST_STATE_NULL);
    // Late close messages (and anything else) posted during the NULL transition.
    if (bus) {
        while (GstMessage* msg = gst_bus_pop_filtered(bus, types)) {
            handleMessage(msg);
            gst_message_unref(msg);
        }
        gst_object_unref(bus);
    }
    closeSidecar();
    stopSyncThread();
}

bool SegmentedRecorder::stop() {
    if (!pipeline_) return true;
    previewGate_.store(false);       // the preview tap closes first: no delivery starts after this

    // Hard deadline over the WHOLE shutdown — joining the worker included: it
    // can be stuck in a sidecar write/open on a wedged filesystem just as the
    // teardown can be stuck in the kernel, and neither is recoverable
    // in-process.  The guard must therefore start before the join.
    std::mutex              m;
    std::condition_variable cv;
    bool                    done = false;
    const uint32_t          limitMs = opts_.eosTimeoutMs + 3000;
    std::thread guard([&] {
        std::unique_lock<std::mutex> lk(m);
        if (cv.wait_for(lk, std::chrono::milliseconds(limitMs), [&] { return done; })) return;
        const std::string why = "recording teardown hung for " + std::to_string(limitMs) + " ms";
        // Nothing on the exit path may block — not a log callback (possibly
        // synchronous, e.g. stuck on a full stderr pipe), not the wedged disk.
        if (opts_.exitOnTeardownHang)
            dashcam::log::emergencyExit(log_, why + " — exiting for the restart loop", 3);
        doLog(log_, LogLevel::ERROR, "%s", why.c_str());
        cv.wait(lk, [&] { return done; });
    });

    {                                                          // watchdog off (never blocks)
        std::lock_guard<std::mutex> lk(monMu_);
        monStop_ = true;
    }
    monCv_.notify_all();
    if (monitor_.joinable()) monitor_.join();

    stopFlag_.store(true);
    if (GstBus* bus = gst_element_get_bus(pipeline_)) {        // wake the worker now
        gst_bus_post(bus, gst_message_new_application(GST_OBJECT(pipeline_),
                                                      gst_structure_new_empty("dashcam-stop")));
        gst_object_unref(bus);
    }
    if (worker_.joinable()) worker_.join();

    const std::string file = currentFile();
    teardown();
    releasePreview();                // NULL joined the streaming threads: no callback is running
    const bool clean = [&] {
        std::lock_guard<std::mutex> lock(stateMutex_);
        return openSeqs_.empty();
    }();

    {
        std::lock_guard<std::mutex> lk(m);
        done = true;
    }
    cv.notify_all();
    guard.join();

    gst_object_unref(recq_);
    recq_ = nullptr;
    gst_object_unref(smx_);
    smx_ = nullptr;
    GstElement* pipe = pipeline_;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        pipeline_ = nullptr;
        openSeqs_.clear();
    }
    gst_object_unref(pipe);
    healthy_.store(false);
    ring_.clear();
    doLog(log_, LogLevel::INFO, "segmented recording stopped (%llu frames%s%s)",
          static_cast<unsigned long long>(bufferCount_.load()),
          file.empty() ? "" : ", last ", file.c_str());
    return clean;
}

} // namespace dashcam::record
