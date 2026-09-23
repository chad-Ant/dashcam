#include "librecord.h"

#include <linux/videodev2.h>
#include <dirent.h>
#include <unistd.h>

#include <algorithm>
#include <charconv>
#include <chrono>
#include <climits>
#include <condition_variable>
#include <cstdarg>
#include <cstdio>
#include <ctime>

namespace dashcam::record {

using dashcam::log::LogLevel;

// ─── file-local helpers ───────────────────────────────────────────────────────

static void doLog(const dashcam::log::LogCallback& cb, LogLevel lvl, const char* fmt, ...) {
    if (!cb) return;
    char buf[640];
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

static int64_t wallNowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

static std::string baseName(const std::string& path) {
    const auto sep = path.find_last_of('/');
    return sep == std::string::npos ? path : path.substr(sep + 1);
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

// ─── bus handling (worker thread; stop() thread after the worker joined) ─────

void SegmentedRecorder::handleMessage(GstMessage* msg) {
    switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_ERROR: {
        GError* err = nullptr;
        gchar*  dbg = nullptr;
        gst_message_parse_error(msg, &err, &dbg);
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
    if (!assEnabled_) return;

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

    // The fragment-opened message arrives up to one GOP after the fragment's
    // first frame; replay what was sampled since then into the new sidecar.
    const int64_t durNs = static_cast<int64_t>(1e9 / std::max(0.5f, (float)cfg.subtitleRateHz));
    for (const Sample& s : ring_)
        if (s.rtNs >= rtNs)
            detail::writeAssSample(assFile_, s.rtNs - rtNs, durNs, s.od, s.wallMs, s.stale.speed,
                                   s.stale.accel, s.stale.position, s.stale.heading);
}

void SegmentedRecorder::onFragmentClosed(const std::string& location) {
    const auto seq = parseSegmentName(baseName(location), opts_.prefix);
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (seq) openSeqs_.erase(*seq);
    }
    fragmentClosed_.store(true);
    doLog(log_, LogLevel::INFO, "segment closed: %s", location.c_str());
}

void SegmentedRecorder::closeSidecar() {
    if (!assFile_.is_open()) return;
    assFile_.flush();
    assFile_.close();
}

// ─── watchdog + sidecar sampling (worker thread) ──────────────────────────────

void SegmentedRecorder::checkWatchdog() {
    if (!healthy_.load()) return;
    const int64_t now = steadyNowNs();
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
                               s.stale.accel, s.stale.position, s.stale.heading);
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
    const auto types = static_cast<GstMessageType>(
        GST_MESSAGE_ERROR | GST_MESSAGE_EOS | GST_MESSAGE_ELEMENT);
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

        checkWatchdog();
        if (assEnabled_) takeSample(durNs, staleMs);

        if (std::chrono::steady_clock::now() >= nextFlush) {
            if (assFile_.is_open()) assFile_.flush();
            nextFlush = std::chrono::steady_clock::now() + std::chrono::seconds(5);
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
    gint frNum = 30, frDen = 1;
    gst_util_double_to_fraction(static_cast<double>(fmt.fps), &frNum, &frDen);
    const std::string rate = std::to_string(frNum) + "/" + std::to_string(frDen);

    std::string desc;
    if (testSourceDesc_.empty()) {
        desc = "v4l2src name=camerasrc device=" + detail::gstQuoted(devicePath) +
               (mjpeg ? " ! image/jpeg" : " ! video/x-h264") +
               ", width=" + std::to_string(fmt.width) +
               ", height=" + std::to_string(fmt.height) + ", framerate=" + rate;
    } else {
        desc = testSourceDesc_;
    }
    // MJPEG frames are intra-only, so dropping them to cap the rate is safe.
    if (mjpeg && opts.maxFps > 0 && static_cast<float>(opts.maxFps) < fmt.fps)
        desc += " ! videorate drop-only=true max-rate=" + std::to_string(opts.maxFps);
    if (h264) desc += " ! h264parse";
    desc += " ! queue name=recq max-size-buffers=8 leaky=0"
            " ! splitmuxsink name=smx async-finalize=true muxer-factory=matroskamux"
            " muxer-properties=\"properties,offset-to-zero=true\""
            " sink-properties=\"properties,sync=false,async=false\""
            " max-size-time=" + std::to_string(uint64_t(opts.segmentSec) * GST_SECOND);

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
    gst_object_unref(smx);
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
        gst_object_unref(recq_);
        recq_ = nullptr;
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
    worker_ = std::thread(&SegmentedRecorder::workerLoop, this);
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
}

bool SegmentedRecorder::stop() {
    if (!pipeline_) return true;

    stopFlag_.store(true);
    if (worker_.joinable()) worker_.join();

    // Hard deadline: a teardown stuck in the kernel cannot be recovered in-process.
    std::mutex              m;
    std::condition_variable cv;
    bool                    done = false;
    const uint32_t          limitMs = opts_.eosTimeoutMs + 3000;
    std::thread guard([&] {
        std::unique_lock<std::mutex> lk(m);
        if (cv.wait_for(lk, std::chrono::milliseconds(limitMs), [&] { return done; })) return;
        doLog(log_, LogLevel::ERROR, "recording teardown hung for %u ms%s", limitMs,
              opts_.exitOnTeardownHang ? " — exiting for the restart loop" : "");
        if (opts_.exitOnTeardownHang) _exit(3);
        cv.wait(lk, [&] { return done; });
    });

    const std::string file = currentFile();
    teardown();
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
