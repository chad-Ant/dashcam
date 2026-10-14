/**
 * @file libes3display.cpp
 * @brief Es3DisplayLink: the ES3C28P tty owner (see libes3display.h).
 *
 * Layout: Impl holds what other threads touch (the published demand word, the
 * pending frame and the control queue under handoffMu, the stats snapshot under
 * statsMu); Io holds what only the worker touches (the tty, the parser, the TX
 * record, the control slot, the video session and the frame in transmission).
 * Locks are never held across I/O, a log line or a callback.
 */

#include "libes3display.h"

#include "libuart.h"

#include <fcntl.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <system_error>
#include <thread>
#include <utility>

namespace dashcam::es3display {

namespace {

using dashcam::log::LogLevel;
using dashcam::preview::EncodedFrameLease;
using dashcam::preview::PreviewDemand;
using dashcam::preview::Viewport;

constexpr int      kPollSliceMs     = 100;   ///< Longest sleep: bounds stop() and timer drift.
constexpr int      kOutqSliceMs     = 1;     ///< Re-check of TIOCOUTQ where POLLOUT is no drain signal.
constexpr int      kOutqSafetyMs    = 20;    ///< ... and where it is (a missed wakeup costs this much).
constexpr int      kStopBudgetMs    = 300;   ///< STOP_VIDEO exchange at stop(), including a reply in flight.
constexpr int      kCloseDrainMs    = 100;   ///< Uart::close() grace for a peer that stopped reading.
constexpr size_t   kReadChunk       = 4096;
constexpr int      kMaxReadsPerWake = 16;    ///< Bounded so TX and timers still run under a flood.
constexpr int      kMaxTxSteps      = 64;    ///< Records built/written per pass.
constexpr size_t   kFrameRing       = 16;    ///< Started frames awaiting RESULT / PRESENTED.
constexpr uint32_t kFrameIdLimit    = 0x7fffffffu;  ///< Renegotiate long before frame_id wraps.
constexpr uint32_t kMinNegotiateBackoffMs = 1000;
constexpr int64_t  kMaxCaptureLatencyUs   = 60LL * 1000 * 1000;  ///< Older "capture" times are not latencies.

int64_t nowUs() {
    using namespace std::chrono;
    return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

/// Packed PreviewDemand: one atomic word, so demand() is a single load.
uint64_t packDemand(bool active, uint16_t w, uint16_t h, uint32_t maxJpeg) {
    return (uint64_t(active) << 63) | (uint64_t(w) << 40) | (uint64_t(h) << 24) |
           (uint64_t(maxJpeg) & 0xffffffu);
}

std::string vformat(const char* fmt, va_list ap) {
    char small[256];
    va_list copy;
    va_copy(copy, ap);
    const int n = std::vsnprintf(small, sizeof(small), fmt, copy);
    va_end(copy);
    if (n < 0) return {};
    if (size_t(n) < sizeof(small)) return std::string(small, size_t(n));
    std::string out(size_t(n) + 1, '\0');
    std::vsnprintf(&out[0], out.size(), fmt, ap);
    out.resize(size_t(n));
    return out;
}

const char* commandName(uint8_t type) {
    switch (type) {
    case es3proto::Hello:        return "HELLO";
    case es3proto::Ping:         return "PING";
    case es3proto::Status:       return "STATUS";
    case es3proto::Page:         return "PAGE";
    case es3proto::Alert:        return "ALERT";
    case es3proto::StopAudio:    return "STOP_AUDIO";
    case es3proto::Capabilities: return "CAPABILITIES";
    case es3proto::StartVideo:   return "START_VIDEO";
    case es3proto::StopVideo:    return "STOP_VIDEO";
    default:                     return "?";
    }
}

const char* errorName(uint8_t code) {
    switch (code) {
    case es3proto::BadCommand: return "BadCommand";
    case es3proto::BadPayload: return "BadPayload";
    case es3proto::NotReady:   return "NotReady";
    case es3proto::NoSession:  return "NoSession";
    default:                   return "malformed error";
    }
}

/// Commands the firmware answers from its Arduino loop, not its USB task.
bool isLoopCommand(uint8_t type) {
    return type == es3proto::Status || type == es3proto::Page || type == es3proto::Alert ||
           type == es3proto::StopAudio;
}

bool isApplicationCommand(uint8_t type) {
    return type == es3proto::Ping || type == es3proto::Status || type == es3proto::Page ||
           type == es3proto::Alert || type == es3proto::StopAudio;
}

bool printableField(const std::string& s) {
    if (s.size() >= kPageTextBytes) return false;
    return std::all_of(s.begin(), s.end(), [](char c) { return c >= 32 && c <= 126; });
}

/// Fixed-size window of float samples; summary() sorts a copy (stats() only).
struct LatencyRing {
    std::array<float, kLatencyWindow> v{};
    uint32_t count = 0;
    uint32_t head  = 0;
    double   last  = 0.0;

    void push(double ms) {
        v[head] = static_cast<float>(ms);
        head    = (head + 1) % kLatencyWindow;
        if (count < kLatencyWindow) ++count;
        last = ms;
    }
    LatencySummary summary() const {
        LatencySummary s;
        s.samples = count;
        s.lastMs  = last;
        if (count == 0) return s;
        std::array<float, kLatencyWindow> q{};
        std::copy(v.begin(), v.begin() + count, q.begin());
        std::sort(q.begin(), q.begin() + count);
        auto rank = [&](uint32_t pct) { return q[(count * pct + 99) / 100 - 1]; };  // nearest rank
        s.p50Ms = rank(50);
        s.p95Ms = rank(95);
        return s;
    }
};

/// Quiet-log slots: a message identical to the slot's previous one goes to DEBUG.
enum QuietSlot : uint8_t { QOpen, QHello, QTimeout, QSession, QCaps, QNegotiate, QStall, QWrite, QCount };

} // namespace

// ─── free functions ───────────────────────────────────────────────────────────

bool validateConfig(const DisplayLinkConfig& c, std::string* why) {
    auto fail = [why](const std::string& m) {
        if (why) *why = m;
        return false;
    };
    if (c.device.empty()) return fail("device is empty (an explicit tty path is required; no port is probed)");
    if (c.heartbeatMs < 100 || c.heartbeatMs > 4000) return fail("heartbeatMs must be 100-4000 (device session: 5 s)");
    if (c.reconnectMs < 100 || c.reconnectMs > 60000) return fail("reconnectMs must be 100-60000");
    if (c.replyTimeoutMs < 50 || c.replyTimeoutMs > 4000) return fail("replyTimeoutMs must be 50-4000");
    if (c.loopReplyTimeoutMs < 50 || c.loopReplyTimeoutMs > 4000) return fail("loopReplyTimeoutMs must be 50-4000");
    if (c.chunkDataBytes < 64 || c.chunkDataBytes > es3video::kMaxChunkDataBytes)
        return fail("chunkDataBytes must be 64-4096");
    if (c.maxFramesInFlight < 1 || c.maxFramesInFlight > es3video::kMaxCredits)
        return fail("maxFramesInFlight must be 1-2");
    if (c.targetFps < 1 || c.targetFps > es3video::kMaxTargetFps) return fail("targetFps must be 1-30");
    if (c.maxJpegBytes < 1024 || c.maxJpegBytes > es3video::kMaxJpegBytes)
        return fail("maxJpegBytes must be 1024-65536");
    if (c.assemblyTimeoutMs < 1 || c.assemblyTimeoutMs > es3video::kMaxTimeoutMs)
        return fail("assemblyTimeoutMs must be 1-10000");
    if (c.staleAfterMs < 1 || c.staleAfterMs > es3video::kMaxTimeoutMs) return fail("staleAfterMs must be 1-10000");
    if (c.placeholderAfterMs < c.staleAfterMs || c.placeholderAfterMs > es3video::kMaxTimeoutMs)
        return fail("placeholderAfterMs must be staleAfterMs-10000");
    if (c.creditStallMs != 0 && (c.creditStallMs < 100 || c.creditStallMs > 60000))
        return fail("creditStallMs must be 0 (auto) or 100-60000");
    if (why) why->clear();
    return true;
}

const char* linkStateName(LinkState s) {
    switch (s) {
    case LinkState::Disconnected: return "disconnected";
    case LinkState::Hello:        return "hello";
    case LinkState::Capabilities: return "capabilities";
    case LinkState::Negotiated:   return "negotiated";
    case LinkState::Active:       return "active";
    case LinkState::Paused:       return "paused";
    case LinkState::ControlOnly:  return "control-only";
    }
    return "?";
}

const char* controlStatusName(ControlStatus s) {
    switch (s) {
    case ControlStatus::Ok:           return "ok";
    case ControlStatus::DeviceError:  return "device error";
    case ControlStatus::Timeout:      return "timeout";
    case ControlStatus::Disconnected: return "disconnected";
    case ControlStatus::Cancelled:    return "cancelled";
    }
    return "?";
}

ControlRequest ControlRequest::ping() {
    ControlRequest r;
    r.command = es3proto::Ping;
    return r;
}

ControlRequest ControlRequest::status() {
    ControlRequest r;
    r.command = es3proto::Status;
    return r;
}

ControlRequest ControlRequest::stopAudio() {
    ControlRequest r;
    r.command = es3proto::StopAudio;
    return r;
}

ControlRequest ControlRequest::alert(uint8_t volume) {
    ControlRequest r;
    if (volume > 100) return r;
    r.command    = es3proto::Alert;
    r.length     = 1;
    r.payload[0] = volume;
    return r;
}

ControlRequest ControlRequest::page(uint8_t index, const std::string& title,
                                    const std::array<std::string, 4>& lines) {
    ControlRequest r;
    if (index >= kPageCount || !printableField(title) ||
        !std::all_of(lines.begin(), lines.end(), printableField)) {
        return r;
    }
    static_assert(1 + 5 * kPageTextBytes <= es3proto::kMaxPayload, "PAGE payload must fit an E3 28 frame");
    r.command    = es3proto::Page;
    r.length     = static_cast<uint8_t>(1 + 5 * kPageTextBytes);
    r.payload[0] = index;
    const std::string* fields[5] = {&title, &lines[0], &lines[1], &lines[2], &lines[3]};
    for (size_t i = 0; i < 5; ++i)   // NUL-padded fields (payload is zero-initialised)
        std::memcpy(r.payload + 1 + i * kPageTextBytes, fields[i]->data(), fields[i]->size());
    return r;
}

bool parseDeviceStatus(const ControlResult& r, DeviceStatus& out) {
    if (r.status != ControlStatus::Ok || r.command != es3proto::Status || r.length != 9) return false;
    out.uptimeMs     = es3video::detail::get32(r.payload);
    out.page         = r.payload[4];
    out.brightness   = r.payload[5];
    out.sdCard       = r.payload[6] != 0;
    out.calibrated   = r.payload[7] != 0;
    out.audioPlaying = r.payload[8] != 0;
    return true;
}

// ─── Impl ─────────────────────────────────────────────────────────────────────

struct Es3DisplayLink::Impl {
    /// An application request waiting in the queue or occupying the slot.
    struct Queued {
        ControlRequest req;
        int64_t        enqueuedUs = 0;
    };

    /// Worker-only state; created by start(), destroyed after the join.
    struct Io {
        dashcam::uart::Uart uart;
        int                 fd = -1;
        es3video::MuxParser parser{es3video::kDefaultChunkDataBytes};
        uint8_t             rx[kReadChunk];

        // The record being written: control frames and video records never
        // interleave, so a partial write is finished before anything else.
        enum class TxKind : uint8_t { None, Control, Video };
        uint8_t  tx[es3video::kMaxRecordBytes];
        size_t   txLen = 0, txOff = 0;
        TxKind   txKind = TxKind::None;
        uint8_t  txVideoType = 0;
        int64_t  txProgressUs = 0;
        bool     txBlocked = false;        ///< Last write said EAGAIN: wait for POLLOUT.
        bool     waitingOutq = false;      ///< A video record is held for TIOCOUTQ.
        bool     outqCounted = false;      ///< That hold is already in kernelQueueWaits.
        bool     pollOutDrains = true;     ///< POLLOUT implies an empty queue (cdc-acm); false on a pty.
        uint32_t outqLimit = 0;

        // The control slot (one request in flight).
        struct Control {
            bool     busy = false;         ///< Built; being written or awaiting its reply.
            bool     written = false;
            uint8_t  seq = 0, type = 0;
            int64_t  deadlineUs = 0;
            bool     user = false;
            Queued   app;                  ///< user == true.
            es3video::StartVideoRequest video;  ///< type == StartVideo.
        } ctl;
        uint8_t  nextSeq = 0;
        int64_t  lastRequestUs = 0;        ///< Last request fully written (heartbeat base).

        // Handshake and device limits (per open).
        bool     helloOk = false, connectedOnce = false;
        bool     wantHello = false, wantCaps = false;
        bool     capsKnown = false, videoSupported = false;
        es3video::Capabilities caps;
        uint16_t chunk = 0;
        uint32_t maxJpeg = 0;
        uint8_t  window = 0;

        // Video session.
        struct Session {
            uint32_t token = 0;
            es3video::StartVideoRequest req;
            uint32_t grant = 0, started = 0, results = 0, nextFrameId = 1;
            uint8_t  visibility = es3video::DemandActive;
            int64_t  progressUs = 0;
            bool     creditBlocked = false;
        } s;
        bool     resyncWanted = false;
        int64_t  negotiateNotBeforeUs = 0;

        // The frame being transmitted.
        enum class Phase : uint8_t { Chunk, End, Abort };
        struct Current {
            bool              active = false;
            EncodedFrameLease lease;
            uint32_t          frameId = 0, size = 0, offset = 0, crc = 0;
            Phase             phase = Phase::Chunk;
            uint8_t           abortReason = 0;
            int64_t           startWrittenUs = 0;
        } cur;

        // Started frames still owed a RESULT / PRESENTED.
        struct Outstanding {
            bool     used = false, resulted = false;
            uint32_t frameId = 0;
            int64_t  startUs = 0;           ///< START fully written (0 = not yet).
            uint64_t captureUs = 0;
        };
        std::array<Outstanding, kFrameRing> ring{};

        int64_t  nextOpenUs = 0;
        bool     stopping = false, stopSent = false, stopDone = false;
        int64_t  stopDeadlineUs = 0;

        DisplayLinkStats st;               ///< Worker-side counters, published each pass.
        std::array<std::string, QCount> quiet;
    };

    // Lifecycle (lifeMu serialises start/stop).
    std::mutex                lifeMu;
    DisplayLinkConfig         cfg;
    dashcam::log::LogCallback log;
    std::thread               worker;
    int                       wakeFd = -1;
    std::atomic<bool>         running{false};
    std::atomic<bool>         stopRequested{false};
    std::unique_ptr<Io>       io;

    // Published by the worker.
    std::atomic<uint64_t> demandWord{0};
    std::atomic<uint8_t>  stateValue{static_cast<uint8_t>(LinkState::Disconnected)};

    // Handoff between API callers and the worker (handoffMu).
    std::mutex        handoffMu;
    bool              acceptVideo = false;
    bool              acceptControl = false;
    uint16_t          acceptMaxW = 0, acceptMaxH = 0;
    uint32_t          acceptMaxJpeg = 0;
    EncodedFrameLease pending;
    std::array<Queued, kControlQueueDepth> queue{};
    size_t            qHead = 0, qCount = 0;

    // Caller-side counters.
    std::atomic<uint64_t> framesOffered{0}, framesAccepted{0}, framesRefused{0}, framesReplaced{0},
        controlEnqueued{0}, controlRejected{0}, controlQueueFull{0};

    // Snapshot for stats() (statsMu).
    mutable std::mutex statsMu;
    DisplayLinkStats   published;
    LatencyRing        presentRing, captureRing, controlRing;

    // ── helpers used from any thread ─────────────────────────────────────────
    void wake() {
        if (wakeFd < 0) return;
        const uint64_t one = 1;
        const ssize_t  n   = ::write(wakeFd, &one, sizeof(one));
        (void)n;   // a saturated counter still means "wake up"
    }

    // ── worker ───────────────────────────────────────────────────────────────
    void workerLoop();
    void beginStopping(int64_t now);
    void finishWorker();

    bool openDevice(int64_t now);
    void closeDevice(const std::string& why, bool unexpected);
    void dropQueued(ControlStatus status);
    void failSlot(ControlStatus status);

    void checkTimers(int64_t now);
    int  pollTimeoutMs(int64_t now) const;
    void pumpTx();
    void onRecordWritten(int64_t now);
    bool buildControl(int64_t now);
    void buildControlFrame(uint8_t type, const uint8_t* payload, size_t length);
    bool buildVideo(int64_t now);
    bool kernelQueueAllows();
    bool needNegotiation(uint16_t& w, uint16_t& h);
    bool buildStartVideo(int64_t now, uint16_t w, uint16_t h);

    void readInput();
    void feedParser(const uint8_t* data, size_t size);
    void onControl(const es3proto::Frame& f);
    void onReply(Io::Control& done, const es3proto::Frame& f);
    void onError(Io::Control& done, uint8_t code);
    void onTimeout(Io::Control& done);
    void onVideo(const es3video::Record& r);
    void completeApp(Queued& app, ControlStatus status, uint8_t errorCode, const es3proto::Frame* reply);

    void rehello(const char* why);
    void endSession();
    void abandonCurrent(bool sendAbort, uint8_t reason, bool count = true);
    void negotiationFailed(const std::string& why);
    void resync(int64_t now);
    Io::Outstanding* findFrame(uint32_t frameId);
    void addFrame(uint32_t frameId, uint64_t captureUs);
    bool outstandingWork();

    void setState(LinkState s);
    void publishVideo();
    void publishStats();
    void logLine(LogLevel lvl, const char* fmt, ...) __attribute__((format(printf, 3, 4)));
    void logQuiet(QuietSlot slot, LogLevel lvl, const char* fmt, ...) __attribute__((format(printf, 4, 5)));

    uint32_t stallBoundMs() const {
        return cfg.creditStallMs ? cfg.creditStallMs
                                 : std::max<uint32_t>(2000, 4u * uint32_t(cfg.assemblyTimeoutMs));
    }
    uint32_t replyTimeoutFor(uint8_t type) const {
        return isLoopCommand(type) ? cfg.loopReplyTimeoutMs : cfg.replyTimeoutMs;
    }
    uint32_t txStallMs() const { return std::max<uint32_t>(1000, 2 * cfg.replyTimeoutMs); }
};

// ─── logging ──────────────────────────────────────────────────────────────────

void Es3DisplayLink::Impl::logLine(LogLevel lvl, const char* fmt, ...) {
    if (!log) return;
    va_list ap;
    va_start(ap, fmt);
    const std::string msg = vformat(fmt, ap);
    va_end(ap);
    log(lvl, msg);
}

void Es3DisplayLink::Impl::logQuiet(QuietSlot slot, LogLevel lvl, const char* fmt, ...) {
    if (!log) return;
    va_list ap;
    va_start(ap, fmt);
    std::string msg = vformat(fmt, ap);
    va_end(ap);
    std::string& last = io->quiet[slot];
    const bool repeat = (msg == last);
    if (!repeat) last = msg;
    log(repeat ? LogLevel::DEBUG : lvl, msg);
}

// ─── publication ──────────────────────────────────────────────────────────────

void Es3DisplayLink::Impl::setState(LinkState s) {
    stateValue.store(static_cast<uint8_t>(s), std::memory_order_release);
    io->st.state = s;
}

/// Recompute what offerFrame() accepts and what demand() reports; a frame
/// still pending when video goes inactive is released here.
void Es3DisplayLink::Impl::publishVideo() {
    Io&        w      = *io;
    const bool active = w.fd >= 0 && w.helloOk && w.videoSupported && !w.stopping &&
                        (w.s.token == 0 || w.s.visibility == es3video::DemandActive);
    EncodedFrameLease dropped;
    {
        std::lock_guard<std::mutex> lk(handoffMu);
        acceptVideo   = active;
        acceptMaxW    = w.videoSupported ? w.caps.maxWidth : 0;
        acceptMaxH    = w.videoSupported ? w.caps.maxHeight : 0;
        acceptMaxJpeg = w.videoSupported ? w.maxJpeg : 0;
        if (!active && pending) dropped = std::move(pending);
        demandWord.store(w.videoSupported ? packDemand(active, w.caps.canvasWidth, w.caps.canvasHeight, w.maxJpeg)
                                          : 0,
                         std::memory_order_release);
    }
    if (dropped) {
        ++w.st.framesReleased;
        dropped.release();
    }
}

void Es3DisplayLink::Impl::publishStats() {
    Io& w = *io;
    w.st.connected        = w.fd >= 0 && w.helloOk;
    w.st.videoSupported   = w.videoSupported;
    w.st.sessionToken     = w.s.token;
    w.st.viewport         = w.s.token ? Viewport{w.s.req.viewportX, w.s.req.viewportY, w.s.req.viewportW,
                                                 w.s.req.viewportH}
                                      : Viewport{};
    w.st.chunkBytes       = w.s.token ? w.s.req.chunkBytes : 0;
    w.st.maxJpegBytes     = w.videoSupported ? w.maxJpeg : 0;
    w.st.creditWindow     = w.videoSupported ? w.window : 0;
    w.st.grant            = w.s.grant;
    w.st.sessionStarted   = w.s.started;
    w.st.sessionResults   = w.s.results;
    w.st.visibility       = w.s.visibility;
    w.st.kernelQueueLimit = w.fd >= 0 ? w.outqLimit : 0;
    const es3video::MuxParser::Stats& ps = w.parser.stats();
    w.st.parserGarbageBytes = ps.garbageBytes;
    w.st.parserCrcErrors    = ps.crcErrors;
    w.st.parserHeaderErrors = ps.headerErrors;
    w.st.parserTimeouts     = ps.timeouts;
    std::lock_guard<std::mutex> lk(statsMu);
    published = w.st;
}

// ─── public API ───────────────────────────────────────────────────────────────

Es3DisplayLink::Es3DisplayLink() : impl_(std::make_unique<Impl>()) {}

Es3DisplayLink::~Es3DisplayLink() { stop(); }

bool Es3DisplayLink::start(const DisplayLinkConfig& cfg, dashcam::log::LogCallback log) {
    Impl&                       m = *impl_;
    std::lock_guard<std::mutex> life(m.lifeMu);
    if (m.worker.joinable()) {
        dashcam::log::logPrintf(log, LogLevel::ERROR, "ES3 display: start() while already running");
        return false;
    }
    std::string why;
    if (!validateConfig(cfg, &why)) {
        dashcam::log::logPrintf(log, LogLevel::ERROR, "ES3 display: invalid config: %s - display link off",
                                why.c_str());
        return false;
    }
    const int efd = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (efd < 0) {
        dashcam::log::logPrintf(log, LogLevel::ERROR, "ES3 display: eventfd failed: %s - display link off",
                                std::strerror(errno));
        return false;
    }

    m.cfg    = cfg;
    m.log    = std::move(log);
    m.wakeFd = efd;
    m.io     = std::make_unique<Impl::Io>();
    m.io->st.device = cfg.device;
    m.io->nextOpenUs = nowUs();
    for (auto* c : {&m.framesOffered, &m.framesAccepted, &m.framesRefused, &m.framesReplaced,
                    &m.controlEnqueued, &m.controlRejected, &m.controlQueueFull})
        c->store(0, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lk(m.handoffMu);
        m.acceptVideo = m.acceptControl = false;
        m.qHead = m.qCount = 0;
    }
    {
        std::lock_guard<std::mutex> lk(m.statsMu);
        m.published   = DisplayLinkStats{};
        m.published.device = cfg.device;
        m.presentRing = m.captureRing = m.controlRing = LatencyRing{};
    }
    m.demandWord.store(0);
    m.stateValue.store(static_cast<uint8_t>(LinkState::Disconnected));
    m.stopRequested.store(false);
    m.running.store(true, std::memory_order_release);
    try {
        m.worker = std::thread(&Impl::workerLoop, &m);
    } catch (const std::system_error& e) {
        m.running.store(false);
        ::close(m.wakeFd);
        m.wakeFd = -1;
        m.io.reset();
        dashcam::log::logPrintf(m.log, LogLevel::ERROR, "ES3 display: worker thread failed: %s - display link off",
                                e.what());
        return false;
    }
    m.logLine(LogLevel::INFO,
              "ES3 display: link on %s (heartbeat %u ms, reconnect %u ms, reply timeout %u ms, video %s)",
              cfg.device.c_str(), cfg.heartbeatMs, cfg.reconnectMs, cfg.replyTimeoutMs,
              cfg.video ? "on" : "off");
    return true;
}

void Es3DisplayLink::stop() {
    Impl&                       m = *impl_;
    std::lock_guard<std::mutex> life(m.lifeMu);
    if (!m.worker.joinable()) return;
    if (std::this_thread::get_id() == m.worker.get_id()) {
        // From a callback on the worker itself: it cannot join itself.
        m.stopRequested.store(true);
        dashcam::log::logPrintf(m.log, LogLevel::ERROR,
                                "ES3 display: stop() called from a link callback - ignored (call it elsewhere)");
        return;
    }
    m.running.store(false, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lk(m.handoffMu);
        m.acceptVideo = m.acceptControl = false;
    }
    m.stopRequested.store(true);
    m.wake();
    m.worker.join();
    ::close(m.wakeFd);
    m.wakeFd = -1;
    m.io.reset();
    m.demandWord.store(0);
    m.stateValue.store(static_cast<uint8_t>(LinkState::Disconnected));
    {
        std::lock_guard<std::mutex> lk(m.statsMu);
        m.published.state     = LinkState::Disconnected;
        m.published.connected = false;
    }
    m.logLine(LogLevel::INFO, "ES3 display: link stopped");
}

bool Es3DisplayLink::running() const { return impl_->running.load(std::memory_order_acquire); }

LinkState Es3DisplayLink::state() const {
    return static_cast<LinkState>(impl_->stateValue.load(std::memory_order_acquire));
}

PreviewDemand Es3DisplayLink::demand() const {
    const uint64_t w = impl_->demandWord.load(std::memory_order_acquire);
    PreviewDemand  d;
    d.active       = (w >> 63) != 0;
    d.canvasW      = static_cast<uint16_t>(w >> 40);
    d.canvasH      = static_cast<uint16_t>(w >> 24);
    d.maxJpegBytes = static_cast<size_t>(w & 0xffffffu);
    return d;
}

bool Es3DisplayLink::offerFrame(EncodedFrameLease&& frame) {
    Impl& m = *impl_;
    m.framesOffered.fetch_add(1, std::memory_order_relaxed);
    if (!frame || frame.data() == nullptr || frame.size() == 0 || frame.width() == 0 || frame.height() == 0) {
        m.framesRefused.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    EncodedFrameLease replaced;
    {
        std::lock_guard<std::mutex> lk(m.handoffMu);
        if (!m.running.load(std::memory_order_acquire) || !m.acceptVideo || frame.width() > m.acceptMaxW ||
            frame.height() > m.acceptMaxH || frame.size() > m.acceptMaxJpeg) {
            m.framesRefused.fetch_add(1, std::memory_order_relaxed);
            return false;   // the caller keeps the lease
        }
        replaced  = std::move(m.pending);
        m.pending = std::move(frame);
    }
    m.framesAccepted.fetch_add(1, std::memory_order_relaxed);
    if (replaced) {
        m.framesReplaced.fetch_add(1, std::memory_order_relaxed);
        replaced.release();
    }
    m.wake();
    return true;
}

bool Es3DisplayLink::enqueueControl(const ControlRequest& request) {
    Impl& m = *impl_;
    if (!request.valid() || !isApplicationCommand(request.command) || request.length > es3proto::kMaxPayload) {
        m.controlRejected.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    {
        std::lock_guard<std::mutex> lk(m.handoffMu);
        if (!m.running.load(std::memory_order_acquire) || !m.acceptControl) {
            m.controlRejected.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        if (m.qCount == kControlQueueDepth) {
            m.controlQueueFull.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        Impl::Queued& q = m.queue[(m.qHead + m.qCount) % kControlQueueDepth];
        q.req        = request;
        q.enqueuedUs = nowUs();
        ++m.qCount;
    }
    m.controlEnqueued.fetch_add(1, std::memory_order_relaxed);
    m.wake();
    return true;
}

DisplayLinkStats Es3DisplayLink::stats() const {
    const Impl&      m = *impl_;
    DisplayLinkStats s;
    {
        std::lock_guard<std::mutex> lk(m.statsMu);
        s                = m.published;
        s.presentLatency = m.presentRing.summary();
        s.captureLatency = m.captureRing.summary();
        s.controlLatency = m.controlRing.summary();
    }
    s.state            = state();
    s.framesOffered    = m.framesOffered.load(std::memory_order_relaxed);
    s.framesAccepted   = m.framesAccepted.load(std::memory_order_relaxed);
    s.framesRefused    = m.framesRefused.load(std::memory_order_relaxed);
    s.framesReplaced   = m.framesReplaced.load(std::memory_order_relaxed);
    s.controlEnqueued  = m.controlEnqueued.load(std::memory_order_relaxed);
    s.controlRejected  = m.controlRejected.load(std::memory_order_relaxed);
    s.controlQueueFull = m.controlQueueFull.load(std::memory_order_relaxed);
    return s;
}

// ─── worker: main loop ────────────────────────────────────────────────────────

void Es3DisplayLink::Impl::workerLoop() {
    Io& w = *io;
    for (;;) {
        int64_t now = nowUs();
        if (stopRequested.load() && !w.stopping) beginStopping(now);
        if (w.stopping && (w.fd < 0 || w.stopDone || now >= w.stopDeadlineUs)) break;

        if (w.fd < 0) {
            if (now >= w.nextOpenUs) {
                w.nextOpenUs = now + int64_t(cfg.reconnectMs) * 1000;
                openDevice(now);
            }
            if (w.fd < 0) {
                publishStats();
                struct pollfd p = {wakeFd, POLLIN, 0};
                const int64_t waitMs = std::clamp<int64_t>((w.nextOpenUs - nowUs() + 999) / 1000, 1, kPollSliceMs);
                if (::poll(&p, 1, static_cast<int>(waitMs)) > 0 && (p.revents & POLLIN)) {
                    uint64_t v;
                    while (::read(wakeFd, &v, sizeof(v)) > 0) {}
                }
                continue;
            }
        }

        checkTimers(nowUs());
        if (w.fd >= 0) pumpTx();
        publishStats();
        if (w.fd < 0) continue;

        const bool wantOut = w.txBlocked || (w.waitingOutq && w.pollOutDrains);
        struct pollfd p[2];
        p[0] = {w.fd, static_cast<short>(POLLIN | (wantOut ? POLLOUT : 0)), 0};
        p[1] = {wakeFd, POLLIN, 0};
        const int r = ::poll(p, 2, pollTimeoutMs(nowUs()));
        if (r < 0) {
            if (errno == EINTR) continue;
            closeDevice(std::string("poll failed: ") + std::strerror(errno), true);
            continue;
        }
        if (p[1].revents & POLLIN) {
            uint64_t v;
            while (::read(wakeFd, &v, sizeof(v)) > 0) {}
        }
        // Drain before acting on a hangup: the last bytes may hold a reply.
        if (p[0].revents & POLLIN) readInput();
        if (w.fd >= 0 && (p[0].revents & (POLLHUP | POLLERR | POLLNVAL))) {
            closeDevice("device hangup", true);
            continue;
        }
        if (w.fd >= 0 && (p[0].revents & POLLOUT) && w.waitingOutq && !w.txBlocked) {
            // POLLOUT while the queue is still above the limit: on this driver
            // (a pty) POLLOUT only means "room", so poll TIOCOUTQ on a timer.
            int q = 0;
            if (::ioctl(w.fd, TIOCOUTQ, &q) == 0 && uint32_t(q) >= w.outqLimit) w.pollOutDrains = false;
        }
    }
    finishWorker();
}

void Es3DisplayLink::Impl::beginStopping(int64_t now) {
    Io& w = *io;
    w.stopping       = true;
    w.stopDeadlineUs = now + int64_t(kStopBudgetMs) * 1000;
    abandonCurrent(false, 0);
    {
        std::lock_guard<std::mutex> lk(handoffMu);
        acceptControl = false;
    }
    dropQueued(ControlStatus::Cancelled);
    publishVideo();   // releases a pending frame
    // Nothing to stop: no tty, no handshake, or no session.
    if (w.fd < 0 || !w.helloOk || w.s.token == 0) w.stopDone = true;
}

void Es3DisplayLink::Impl::finishWorker() {
    Io& w = *io;
    if (w.fd >= 0) closeDevice("link stopped", false);
    failSlot(ControlStatus::Cancelled);
    dropQueued(ControlStatus::Cancelled);
    abandonCurrent(false, 0);
    publishVideo();
    setState(LinkState::Disconnected);
    publishStats();
}

// ─── worker: open / close ─────────────────────────────────────────────────────

bool Es3DisplayLink::Impl::openDevice(int64_t now) {
    Io&                       w = *io;
    dashcam::uart::UartConfig u;
    u.baudRate      = 115200;          // CDC ignores it; never 1200 (TinyUSB's bootloader "touch")
    u.exclusive     = cfg.exclusive;
    u.hangupOnClose = false;           // DTR/RTS are the S3's reset lines
    u.closeDrainMs  = kCloseDrainMs;
    // No log callback: a missing device is the normal waiting state; this
    // function logs the first failure of each kind itself.
    errno = 0;
    if (!w.uart.open(cfg.device, u, {})) {
        const int err = errno;
        ++w.st.openFailures;
        logQuiet(QOpen, LogLevel::WARN, "ES3 display: cannot open %s (%s) - retrying every %u ms",
                 cfg.device.c_str(), err ? std::strerror(err) : "configuration failed", cfg.reconnectMs);
        return false;
    }
    w.fd = w.uart.fd();
    const int fl = ::fcntl(w.fd, F_GETFL, 0);
    if (fl < 0 || ::fcntl(w.fd, F_SETFL, fl | O_NONBLOCK) != 0) {
        logQuiet(QOpen, LogLevel::WARN, "ES3 display: cannot make %s non-blocking (%s) - retrying",
                 cfg.device.c_str(), std::strerror(errno));
        w.uart.close();
        w.fd = -1;
        ++w.st.openFailures;
        return false;
    }
    ++w.st.opens;
    w.quiet[QOpen].clear();
    w.parser.reset();
    w.txLen = w.txOff = 0;
    w.txKind        = Io::TxKind::None;
    w.txBlocked     = false;
    w.waitingOutq   = false;
    w.pollOutDrains = true;
    w.ctl           = Io::Control{};
    w.helloOk = w.connectedOnce = false;
    w.capsKnown = w.videoSupported = false;
    w.caps = es3video::Capabilities{};
    w.s    = Io::Session{};
    w.resyncWanted         = false;
    w.negotiateNotBeforeUs = 0;
    w.wantHello            = true;
    w.wantCaps             = false;
    w.lastRequestUs        = now;
    w.outqLimit = cfg.txQueueLimitBytes ? cfg.txQueueLimitBytes
                                        : uint32_t(es3video::kOverheadBytes + es3video::kChunkOffsetBytes +
                                                   cfg.chunkDataBytes);
    {
        std::lock_guard<std::mutex> lk(handoffMu);
        acceptControl = !w.stopping;
    }
    setState(LinkState::Hello);
    logLine(LogLevel::DEBUG, "ES3 display: opened %s, sending HELLO", cfg.device.c_str());
    return true;
}

void Es3DisplayLink::Impl::closeDevice(const std::string& why, bool unexpected) {
    Io& w = *io;
    if (w.fd < 0) return;
    w.uart.close();   // bounded: TIOCNXCL, kCloseDrainMs for a stalled peer, then discard
    w.fd = -1;
    ++w.st.disconnects;
    const ControlStatus status = w.stopping ? ControlStatus::Cancelled : ControlStatus::Disconnected;
    failSlot(status);
    {
        std::lock_guard<std::mutex> lk(handoffMu);
        acceptControl = false;
    }
    dropQueued(status);
    endSession();
    w.txLen = w.txOff = 0;
    w.txKind      = Io::TxKind::None;
    w.txBlocked   = false;
    w.waitingOutq = false;
    w.parser.reset();
    const bool wasConnected = w.helloOk;
    w.helloOk = w.wantHello = w.wantCaps = false;
    w.capsKnown = w.videoSupported = false;
    publishVideo();
    setState(LinkState::Disconnected);
    w.nextOpenUs = nowUs() + int64_t(cfg.reconnectMs) * 1000;
    if (unexpected) {
        logQuiet(QOpen, wasConnected ? LogLevel::WARN : LogLevel::INFO,
                 "ES3 display: %s closed (%s) - reconnecting every %u ms", cfg.device.c_str(), why.c_str(),
                 cfg.reconnectMs);
    } else {
        logLine(LogLevel::DEBUG, "ES3 display: %s closed (%s)", cfg.device.c_str(), why.c_str());
    }
}

/// Complete every queued application request with @p status.
void Es3DisplayLink::Impl::dropQueued(ControlStatus status) {
    std::array<Queued, kControlQueueDepth> taken{};
    size_t                                 n = 0;
    {
        std::lock_guard<std::mutex> lk(handoffMu);
        while (qCount > 0) {
            taken[n++] = std::move(queue[qHead]);
            queue[qHead] = Queued{};
            qHead = (qHead + 1) % kControlQueueDepth;
            --qCount;
        }
        qHead = 0;
    }
    for (size_t i = 0; i < n; ++i) completeApp(taken[i], status, 0, nullptr);
}

/// Free the control slot, completing an application request in it.
void Es3DisplayLink::Impl::failSlot(ControlStatus status) {
    Io& w = *io;
    if (!w.ctl.busy) return;
    Io::Control done = std::move(w.ctl);
    w.ctl            = Io::Control{};
    if (done.user) completeApp(done.app, status, 0, nullptr);
}

void Es3DisplayLink::Impl::completeApp(Queued& app, ControlStatus status, uint8_t errorCode,
                                       const es3proto::Frame* reply) {
    ControlResult r;
    r.status    = status;
    r.command   = app.req.command;
    r.errorCode = errorCode;
    r.latencyMs = double(nowUs() - app.enqueuedUs) / 1000.0;
    if (reply && status == ControlStatus::Ok) {
        r.length = reply->length;
        std::memcpy(r.payload, reply->payload, reply->length);
        std::lock_guard<std::mutex> lk(statsMu);
        controlRing.push(r.latencyMs);
    }
    if (app.req.onDone) app.req.onDone(r);
    app.req.onDone = nullptr;
}

// ─── worker: timers ───────────────────────────────────────────────────────────

void Es3DisplayLink::Impl::checkTimers(int64_t now) {
    Io& w = *io;
    if (w.fd < 0) return;

    // A stalled candidate can hide a complete reply already in the parser.
    // Recover and dispatch it before declaring the control request timed out,
    // even if no further POLLIN event arrives.
    if (!w.stopping) feedParser(nullptr, 0);
    if (w.fd < 0) return;

    // A record that has stopped moving into the kernel: the device is gone
    // without a hangup (or wedged); reopening is the only recovery.
    if (w.txOff < w.txLen && now - w.txProgressUs > int64_t(txStallMs()) * 1000) {
        logQuiet(QWrite, LogLevel::WARN, "ES3 display: USB writes stalled for %u ms", txStallMs());
        closeDevice("write stalled", true);
        return;
    }

    if (w.ctl.busy && w.ctl.written && now >= w.ctl.deadlineUs) {
        Io::Control done = std::move(w.ctl);
        w.ctl            = Io::Control{};
        onTimeout(done);
        if (w.fd < 0) return;
    }

    // Frame still being sent after the assembly deadline: the device has
    // dropped it (or will); stop spending bytes on it.
    if (w.cur.active && w.cur.startWrittenUs != 0 && w.cur.phase != Io::Phase::Abort &&
        now - w.cur.startWrittenUs > int64_t(cfg.assemblyTimeoutMs) * 1000) {
        abandonCurrent(true, es3video::ReasonAssemblyTimeout);
    }

    // Credit stall: frames outstanding, no grant/RESULT progress.
    const bool negotiating = w.ctl.busy && w.ctl.type == es3proto::StartVideo;
    if (w.s.token != 0 && !negotiating && !w.resyncWanted && !w.stopping) {
        if (!outstandingWork()) {
            w.s.progressUs = now;
        } else if (now - w.s.progressUs > int64_t(stallBoundMs()) * 1000) {
            resync(now);
        }
    }

    if (w.s.token != 0 && w.s.nextFrameId >= kFrameIdLimit && !w.resyncWanted) w.resyncWanted = true;
}

/// Frames the device owes us progress on: started without a RESULT, or a
/// ready frame held back by the grant.
bool Es3DisplayLink::Impl::outstandingWork() {
    Io& w = *io;
    if (w.s.started != w.s.results) return true;
    if (w.s.started < w.s.grant) return false;
    std::lock_guard<std::mutex> lk(handoffMu);
    return bool(pending);
}

int Es3DisplayLink::Impl::pollTimeoutMs(int64_t now) const {
    const Io& w = *io;
    int64_t   t = int64_t(kPollSliceMs) * 1000;
    auto      until = [&](int64_t at) { t = std::min(t, at - now); };
    if (w.ctl.busy && w.ctl.written) until(w.ctl.deadlineUs);
    if (!w.ctl.busy && w.helloOk && !w.stopping) until(w.lastRequestUs + int64_t(cfg.heartbeatMs) * 1000);
    if (w.waitingOutq) t = std::min<int64_t>(t, int64_t(w.pollOutDrains ? kOutqSafetyMs : kOutqSliceMs) * 1000);
    if (w.cur.active && w.cur.startWrittenUs) until(w.cur.startWrittenUs + int64_t(cfg.assemblyTimeoutMs) * 1000);
    if (w.s.token) until(w.s.progressUs + int64_t(stallBoundMs()) * 1000);
    if (w.negotiateNotBeforeUs > now) until(w.negotiateNotBeforeUs);
    if (w.txOff < w.txLen) until(w.txProgressUs + int64_t(txStallMs()) * 1000);
    if (w.stopping) until(w.stopDeadlineUs);
    if (t <= 0) return 0;
    return static_cast<int>(std::min<int64_t>((t + 999) / 1000, kPollSliceMs));
}

// ─── worker: transmit ─────────────────────────────────────────────────────────

void Es3DisplayLink::Impl::pumpTx() {
    Io& w = *io;
    for (int step = 0; step < kMaxTxSteps && w.fd >= 0; ++step) {
        if (w.txOff < w.txLen) {
            const ssize_t n = ::write(w.fd, w.tx + w.txOff, w.txLen - w.txOff);
            if (n > 0) {
                w.txOff += size_t(n);
                w.st.bytesSent += uint64_t(n);
                w.txProgressUs = nowUs();
                w.txBlocked    = false;
                if (w.txOff == w.txLen) onRecordWritten(w.txProgressUs);
                continue;
            }
            if (n < 0 && errno == EINTR) continue;
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                w.txBlocked = true;   // keep the rest; POLLOUT resumes it
                return;
            }
            closeDevice(std::string("write failed: ") + (n < 0 ? std::strerror(errno) : "no progress"), true);
            return;
        }
        const int64_t now = nowUs();
        if (buildControl(now)) continue;
        if (buildVideo(now)) continue;
        return;
    }
}

void Es3DisplayLink::Impl::onRecordWritten(int64_t now) {
    Io& w = *io;
    if (w.txKind == Io::TxKind::Control) {
        w.ctl.written    = true;
        w.ctl.deadlineUs = now + int64_t(replyTimeoutFor(w.ctl.type)) * 1000;
        w.lastRequestUs  = now;
        ++w.st.controlSent;
    } else if (w.txKind == Io::TxKind::Video) {
        ++w.st.videoRecordsSent;
        w.st.videoBytesSent += w.txLen;
        if (w.txVideoType == es3video::RecordStart && w.cur.active) {
            w.cur.startWrittenUs = now;
            if (Io::Outstanding* e = findFrame(w.cur.frameId)) e->startUs = now;
        } else if (w.txVideoType == es3video::RecordEnd) {
            ++w.st.framesSent;
        }
    }
    w.txLen = w.txOff = 0;
    w.txKind = Io::TxKind::None;
}

void Es3DisplayLink::Impl::buildControlFrame(uint8_t type, const uint8_t* payload, size_t length) {
    Io& w     = *io;
    w.nextSeq = static_cast<uint8_t>(w.nextSeq % 255 + 1);   // 1..255, as the Python client
    uint8_t frame[es3proto::kMaxFrame];
    const size_t n = es3proto::encode(w.nextSeq, type, payload, length, frame);
    std::memcpy(w.tx, frame, n);
    w.txLen          = n;
    w.txOff          = 0;
    w.txKind         = Io::TxKind::Control;
    w.txProgressUs   = nowUs();
    w.ctl.busy       = true;
    w.ctl.written    = false;
    w.ctl.seq        = w.nextSeq;
    w.ctl.type       = type;
    if (type == es3proto::Ping) ++w.st.pingsSent;
}

/// Next control request, if the slot is free: handshake first, then the
/// application queue, then video negotiation, then the heartbeat.
bool Es3DisplayLink::Impl::buildControl(int64_t now) {
    Io& w = *io;
    if (w.ctl.busy) return false;
    if (w.stopping) {
        if (!w.stopSent && !w.stopDone && w.helloOk && w.s.token) {
            uint8_t p[es3video::kStopVideoRequestBytes];
            es3video::encodeStopVideoRequest(w.s.token, p, sizeof(p));
            buildControlFrame(es3proto::StopVideo, p, sizeof(p));
            w.stopSent = true;
            ++w.st.stopVideoSent;
            return true;
        }
        return false;
    }
    if (w.wantHello) {
        w.wantHello = false;
        buildControlFrame(es3proto::Hello, nullptr, 0);
        return true;
    }
    if (!w.helloOk) return false;
    if (w.wantCaps) {
        w.wantCaps = false;
        buildControlFrame(es3proto::Capabilities, nullptr, 0);
        return true;
    }

    Queued app;
    bool   haveApp = false;
    {
        std::lock_guard<std::mutex> lk(handoffMu);
        if (qCount > 0) {
            app          = std::move(queue[qHead]);
            queue[qHead] = Queued{};
            qHead        = (qHead + 1) % kControlQueueDepth;
            --qCount;
            haveApp = true;
        }
    }
    if (haveApp) {
        buildControlFrame(app.req.command, app.req.payload, app.req.length);
        w.ctl.user = true;
        w.ctl.app  = std::move(app);
        return true;
    }

    uint16_t vw = 0, vh = 0;
    if (w.videoSupported && now >= w.negotiateNotBeforeUs && needNegotiation(vw, vh) &&
        buildStartVideo(now, vw, vh)) {
        return true;
    }

    if (now - w.lastRequestUs >= int64_t(cfg.heartbeatMs) * 1000) {
        buildControlFrame(es3proto::Ping, nullptr, 0);
        return true;
    }
    return false;
}

/// A START_VIDEO is due: a resync was requested, or the pending frame has no
/// session or another size than the session's viewport.  @p w/@p h = viewport.
bool Es3DisplayLink::Impl::needNegotiation(uint16_t& vw, uint16_t& vh) {
    Io& w = *io;
    {
        std::lock_guard<std::mutex> lk(handoffMu);
        if (pending) {
            vw = pending.width();
            vh = pending.height();
            if (w.s.token == 0 || w.resyncWanted || vw != w.s.req.viewportW || vh != w.s.req.viewportH) return true;
            return false;
        }
    }
    if (w.resyncWanted && w.s.req.viewportW && w.s.req.viewportH) {
        vw = w.s.req.viewportW;
        vh = w.s.req.viewportH;
        return true;
    }
    return false;
}

bool Es3DisplayLink::Impl::buildStartVideo(int64_t now, uint16_t vw, uint16_t vh) {
    Io&                         w = *io;
    es3video::StartVideoRequest r;
    r.videoProtocolVersion = es3video::kVersion;
    r.codec                = es3video::CodecBaselineJpeg;
    // Centred, exactly as preview::computeLetterbox() positions the image.
    r.viewportX = vw <= w.caps.canvasWidth ? static_cast<uint16_t>((w.caps.canvasWidth - vw) / 2) : 0;
    r.viewportY = vh <= w.caps.canvasHeight ? static_cast<uint16_t>((w.caps.canvasHeight - vh) / 2) : 0;
    r.viewportW = vw;
    r.viewportH = vh;
    r.maxJpegBytes       = w.maxJpeg;
    r.chunkBytes         = w.chunk;
    r.targetFps          = cfg.targetFps;
    r.assemblyTimeoutMs  = cfg.assemblyTimeoutMs;
    r.staleAfterMs       = cfg.staleAfterMs;
    r.placeholderAfterMs = cfg.placeholderAfterMs;
    uint8_t p[es3video::kStartVideoRequestBytes];
    if (es3video::encodeStartVideoRequest(r, w.caps, p, sizeof(p)) == 0) {
        negotiationFailed("the request does not fit the device's capabilities (viewport " + std::to_string(vw) +
                          "x" + std::to_string(vh) + ")");
        return false;
    }
    (void)now;
    // A frame of the old session still being sent would be dropped as stale
    // once the new token exists: stop it here (the record in flight is done).
    abandonCurrent(false, 0);
    buildControlFrame(es3proto::StartVideo, p, sizeof(p));
    w.ctl.video = r;
    setState(LinkState::Negotiated);
    return true;
}

/// TIOCOUTQ gate for video records (control frames skip it).
bool Es3DisplayLink::Impl::kernelQueueAllows() {
    Io& w = *io;
    int q = 0;
    if (::ioctl(w.fd, TIOCOUTQ, &q) != 0 || q < 0) q = 0;
    w.st.kernelQueueMax = std::max(w.st.kernelQueueMax, uint32_t(q));
    if (uint32_t(q) < w.outqLimit) {
        w.waitingOutq = false;
        w.outqCounted = false;
        return true;
    }
    if (!w.outqCounted) {   // one wait per held record, however many re-checks
        w.outqCounted = true;
        ++w.st.kernelQueueWaits;
    }
    w.waitingOutq = true;
    return false;
}

bool Es3DisplayLink::Impl::buildVideo(int64_t now) {
    Io& w = *io;
    w.waitingOutq = false;
    if (w.stopping || !w.videoSupported || w.s.token == 0) return false;
    if (w.ctl.busy && w.ctl.type == es3proto::StartVideo) return false;   // renegotiating

    if (!w.cur.active) {
        if (w.s.visibility != es3video::DemandActive || w.resyncWanted) return false;
        bool ready = false;
        {
            std::lock_guard<std::mutex> lk(handoffMu);
            ready = pending && pending.width() == w.s.req.viewportW && pending.height() == w.s.req.viewportH;
        }
        if (!ready) return false;
        const bool credit = w.s.started < w.s.grant && w.s.started - w.s.results < w.window;
        if (!credit) {
            if (!w.s.creditBlocked) {
                w.s.creditBlocked = true;
                ++w.st.creditStalls;
            }
            return false;
        }
        if (!kernelQueueAllows()) return false;
        {
            std::lock_guard<std::mutex> lk(handoffMu);
            // Re-check: offerFrame() may have replaced it meanwhile.
            if (!pending || pending.width() != w.s.req.viewportW || pending.height() != w.s.req.viewportH)
                return false;
            w.cur.lease = std::move(pending);
        }
        w.s.creditBlocked = false;
        es3video::StartInfo info;
        info.width             = w.cur.lease.width();
        info.height            = w.cur.lease.height();
        info.jpegBytes         = static_cast<uint32_t>(w.cur.lease.size());
        info.sourceTimestampUs = w.cur.lease.captureMonotonicUs();
        const uint32_t frameId = w.s.nextFrameId;
        const size_t   n = es3video::checkStartForSession(info, w.s.req) == es3video::ReasonOk
                               ? es3video::encodeStart(w.s.token, frameId, info, w.tx, sizeof(w.tx))
                               : 0;
        if (n == 0) {   // offerFrame() checked the limits; only a protocol mismatch gets here
            ++w.st.framesReleased;
            w.cur.lease.release();
            return false;
        }
        ++w.s.nextFrameId;
        ++w.s.started;
        ++w.st.framesStarted;
        w.cur.active         = true;
        w.cur.frameId        = frameId;
        w.cur.size           = info.jpegBytes;
        w.cur.offset         = 0;
        w.cur.crc            = 0;
        w.cur.phase          = Io::Phase::Chunk;
        w.cur.startWrittenUs = 0;
        addFrame(frameId, info.sourceTimestampUs);
        w.txLen        = n;
        w.txOff        = 0;
        w.txKind       = Io::TxKind::Video;
        w.txVideoType  = es3video::RecordStart;
        w.txProgressUs = now;
        return true;
    }

    if (!kernelQueueAllows()) return false;
    size_t n = 0;
    switch (w.cur.phase) {
    case Io::Phase::Chunk: {
        const uint32_t len = std::min<uint32_t>(w.s.req.chunkBytes, w.cur.size - w.cur.offset);
        const uint8_t* src = w.cur.lease.data() + w.cur.offset;
        n = es3video::encodeChunk(w.s.token, w.cur.frameId, w.cur.offset, src, len, w.tx, sizeof(w.tx));
        if (n == 0) {
            abandonCurrent(true, es3video::ReasonInternal);
            return false;
        }
        w.cur.crc = es3video::crc32Update(w.cur.crc, src, len);
        w.cur.offset += len;
        w.txVideoType = es3video::RecordChunk;
        if (w.cur.offset == w.cur.size) {
            w.cur.lease.release();   // the bytes are in tx now: the converter gets its slot back
            w.cur.phase = Io::Phase::End;
        }
        break;
    }
    case Io::Phase::End:
        n = es3video::encodeEnd(w.s.token, w.cur.frameId, w.cur.crc, w.tx, sizeof(w.tx));
        w.txVideoType = es3video::RecordEnd;
        w.cur.active  = false;
        break;
    case Io::Phase::Abort:
        n = es3video::encodeAbort(w.s.token, w.cur.frameId, static_cast<es3video::Reason>(w.cur.abortReason),
                                  w.tx, sizeof(w.tx));
        w.txVideoType = es3video::RecordAbort;
        w.cur.active  = false;
        break;
    }
    if (n == 0) return false;
    w.txLen        = n;
    w.txOff        = 0;
    w.txKind       = Io::TxKind::Video;
    w.txProgressUs = now;
    return true;
}

// ─── worker: receive ──────────────────────────────────────────────────────────

void Es3DisplayLink::Impl::readInput() {
    Io& w = *io;
    for (int i = 0; i < kMaxReadsPerWake && w.fd >= 0; ++i) {
        const ssize_t n = ::read(w.fd, w.rx, sizeof(w.rx));
        if (n > 0) {
            w.st.bytesReceived += uint64_t(n);
            feedParser(w.rx, size_t(n));
            if (size_t(n) < sizeof(w.rx)) break;
            continue;
        }
        if (n < 0 && errno == EINTR) continue;
        if (n == 0 || errno == EAGAIN || errno == EWOULDBLOCK) break;   // POLLHUP decides on EOF
        closeDevice(std::string("read failed: ") + std::strerror(errno), true);
        return;
    }
}

void Es3DisplayLink::Impl::feedParser(const uint8_t* data, size_t size) {
    Io&            w     = *io;
    const uint32_t nowMs = static_cast<uint32_t>(nowUs() / 1000);
    size_t         used  = 0;
    // Include buffered records recovered during an empty timer poll.
    const size_t budget = size + w.parser.buffered();
    for (size_t guard = 0; guard <= budget && w.fd >= 0; ++guard) {
        es3video::MuxParser::Item item;
        used += w.parser.feed(data ? data + used : nullptr, size - used, nowMs, item);
        if (item.kind == es3video::MuxParser::ItemNone) break;
        if (item.kind == es3video::MuxParser::ItemControl) onControl(item.control);
        else onVideo(item.video);
    }
}

void Es3DisplayLink::Impl::onControl(const es3proto::Frame& f) {
    Io& w = *io;
    const bool isError = f.type == es3proto::Error;
    if (!w.ctl.busy || f.sequence != w.ctl.seq ||
        (!isError && f.type != static_cast<uint8_t>(w.ctl.type | es3proto::Reply))) {
        ++w.st.unexpectedReplies;   // late reply of a timed-out request, or noise that framed
        return;
    }
    ++w.st.controlReplies;
    Io::Control done = std::move(w.ctl);
    w.ctl            = Io::Control{};
    if (isError) {
        ++w.st.controlErrors;
        onError(done, f.length == 1 ? f.payload[0] : 0);
    } else {
        onReply(done, f);
    }
}

void Es3DisplayLink::Impl::onReply(Io::Control& done, const es3proto::Frame& f) {
    Io& w = *io;
    switch (done.type) {
    case es3proto::Hello: {
        const size_t idLen = sizeof(kDeviceIdentity) - 1;
        if (f.length != idLen || std::memcmp(f.payload, kDeviceIdentity, idLen) != 0) {
            ++w.st.helloRejected;
            std::string got;
            for (size_t i = 0; i < f.length; ++i)
                got += (f.payload[i] >= 32 && f.payload[i] <= 126) ? char(f.payload[i]) : '.';
            logQuiet(QHello, LogLevel::ERROR,
                     "ES3 display: %s answered HELLO with \"%s\", not \"%s\" - wrong device? closing",
                     cfg.device.c_str(), got.c_str(), kDeviceIdentity);
            closeDevice("not an ES3C28P", false);
            return;
        }
        w.helloOk = true;
        w.st.identity.assign(reinterpret_cast<const char*>(f.payload), idLen);
        w.quiet[QHello].clear();
        if (!w.connectedOnce) {
            w.connectedOnce = true;
            ++w.st.connects;
            logQuiet(QSession, LogLevel::INFO, "ES3 display: connected on %s", cfg.device.c_str());
        }
        if (!cfg.video) {
            setState(LinkState::ControlOnly);
        } else if (!w.capsKnown) {
            w.wantCaps = true;
            setState(LinkState::Capabilities);
        } else {
            setState(w.videoSupported ? LinkState::Negotiated : LinkState::ControlOnly);
        }
        publishVideo();
        return;
    }
    case es3proto::Capabilities: {
        es3video::Capabilities c;
        w.capsKnown = true;
        if (!es3video::decodeCapabilitiesReply(f.payload, f.length, c)) {
            w.videoSupported = false;
            logQuiet(QCaps, LogLevel::ERROR,
                     "ES3 display: unusable CAPABILITIES reply (%u bytes, video protocol %u) - controls only",
                     unsigned(f.length), f.length ? unsigned(f.payload[0]) : 0u);
            setState(LinkState::ControlOnly);
            publishVideo();
            return;
        }
        w.videoSupported = true;
        w.caps           = c;
        w.chunk          = std::min<uint16_t>(cfg.chunkDataBytes, c.maxChunkBytes);
        w.maxJpeg        = std::min<uint32_t>(cfg.maxJpegBytes, c.maxJpegBytes);
        w.window         = std::min<uint8_t>(cfg.maxFramesInFlight, c.maxCredits);
        if (!cfg.txQueueLimitBytes)
            w.outqLimit = uint32_t(es3video::kOverheadBytes + es3video::kChunkOffsetBytes + w.chunk);
        w.st.haveCapabilities = true;
        w.st.capabilities     = c;
        logQuiet(QCaps, LogLevel::INFO,
                 "ES3 display: video v%u, canvas %ux%u, max %ux%u, JPEG %u B (using %u), chunk %u B (using %u), "
                 "credits %u (window %u)",
                 unsigned(c.videoProtocolVersion), c.canvasWidth, c.canvasHeight, c.maxWidth, c.maxHeight,
                 c.maxJpegBytes, w.maxJpeg, unsigned(c.maxChunkBytes), unsigned(w.chunk), unsigned(c.maxCredits),
                 unsigned(w.window));
        setState(LinkState::Negotiated);
        publishVideo();
        return;
    }
    case es3proto::StartVideo: {
        es3video::StartVideoReply r;
        if (!es3video::decodeStartVideoReply(f.payload, f.length, r)) {
            negotiationFailed("malformed START_VIDEO reply");
            return;
        }
        endSession();   // the old token's frames can no longer complete
        w.s.token       = r.sessionToken;
        w.s.req         = done.video;
        w.s.grant       = r.initialGrant;
        w.s.nextFrameId = 1;
        w.s.visibility  = es3video::DemandActive;   // the device re-sends VISIBILITY now
        w.s.progressUs  = nowUs();
        w.resyncWanted  = false;
        ++w.st.sessions;
        w.quiet[QNegotiate].clear();
        logLine(LogLevel::DEBUG, "ES3 display: video session %08x, viewport %u,%u %ux%u, chunk %u, grant %u",
                r.sessionToken, w.s.req.viewportX, w.s.req.viewportY, w.s.req.viewportW, w.s.req.viewportH,
                unsigned(w.s.req.chunkBytes), r.initialGrant);
        setState(LinkState::Active);
        publishVideo();
        return;
    }
    case es3proto::StopVideo:
        if (w.stopping) w.stopDone = true;
        return;
    case es3proto::Status:
        if (f.length == 9) {
            ControlResult tmp;
            tmp.status  = ControlStatus::Ok;
            tmp.command = es3proto::Status;
            tmp.length  = f.length;
            std::memcpy(tmp.payload, f.payload, f.length);
            w.st.haveStatus = parseDeviceStatus(tmp, w.st.lastStatus) || w.st.haveStatus;
        }
        break;
    default:
        break;
    }
    if (done.user) completeApp(done.app, ControlStatus::Ok, 0, &f);
}

void Es3DisplayLink::Impl::onError(Io::Control& done, uint8_t code) {
    Io& w = *io;
    if (code == es3proto::NoSession && done.type != es3proto::Hello) {
        ++w.st.noSessionErrors;
        if (done.user) completeApp(done.app, ControlStatus::DeviceError, code, nullptr);
        if (w.stopping) {
            w.stopDone = true;
            return;
        }
        logQuiet(QTimeout, LogLevel::WARN, "ES3 display: the device has no session for us (NoSession) - HELLO again");
        rehello("device session expired");
        return;
    }
    switch (done.type) {
    case es3proto::Hello:
        logQuiet(QHello, LogLevel::ERROR, "ES3 display: HELLO refused (%s) - closing", errorName(code));
        closeDevice("HELLO refused", false);
        return;
    case es3proto::Capabilities:
        w.capsKnown      = true;
        w.videoSupported = false;
        logQuiet(QCaps, (code == es3proto::BadCommand || code == es3proto::NotReady) ? LogLevel::INFO : LogLevel::WARN,
                 "ES3 display: no video on this firmware (CAPABILITIES: %s) - controls only", errorName(code));
        setState(LinkState::ControlOnly);
        publishVideo();
        return;
    case es3proto::StartVideo:
        negotiationFailed(std::string("START_VIDEO refused: ") + errorName(code));
        return;
    case es3proto::StopVideo:
        if (w.stopping) w.stopDone = true;
        return;
    default:
        break;
    }
    if (done.user) completeApp(done.app, ControlStatus::DeviceError, code, nullptr);
}

void Es3DisplayLink::Impl::onTimeout(Io::Control& done) {
    Io& w = *io;
    ++w.st.controlTimeouts;
    if (done.user) completeApp(done.app, ControlStatus::Timeout, 0, nullptr);   // never replayed
    if (w.stopping) {
        w.stopDone = true;
        return;
    }
    if (done.type == es3proto::Hello) {
        ++w.st.helloTimeouts;
        logQuiet(QHello, LogLevel::WARN, "ES3 display: no HELLO reply from %s in %u ms - reopening in %u ms",
                 cfg.device.c_str(), cfg.replyTimeoutMs, cfg.reconnectMs);
        closeDevice("no HELLO reply", false);
        return;
    }
    logQuiet(QTimeout, LogLevel::WARN, "ES3 display: no reply to %s in %u ms - HELLO again", commandName(done.type),
             replyTimeoutFor(done.type));
    rehello("reply timeout");
}

void Es3DisplayLink::Impl::onVideo(const es3video::Record& r) {
    Io& w = *io;
    if (!es3video::isDeviceToHost(r.type)) {
        ++w.st.unexpectedRecords;
        return;
    }
    if (r.sessionToken != w.s.token || w.s.token == 0) {
        ++w.st.staleRecords;   // another session's record, even if its frame id matches
        return;
    }
    const int64_t now = nowUs();
    switch (r.type) {
    case es3video::RecordCredit: {
        uint32_t g = 0;
        if (es3video::decodeCredit(r, g) != es3video::ReasonOk) {
            ++w.st.malformedRecords;
            return;
        }
        if (g > w.s.grant) {
            w.s.grant         = g;
            w.s.progressUs    = now;
            w.s.creditBlocked = false;
        } else if (g < w.s.grant) {
            ++w.st.lowerGrantIgnored;
        }
        return;
    }
    case es3video::RecordVisibility: {
        es3video::Demand d = es3video::DemandActive;
        if (es3video::decodeVisibility(r, d) != es3video::ReasonOk) {
            ++w.st.malformedRecords;
            return;
        }
        if (d == w.s.visibility) return;
        w.s.visibility = d;
        w.s.progressUs = now;
        logLine(LogLevel::DEBUG, "ES3 display: video demand %s",
                d == es3video::DemandActive ? "active" : d == es3video::DemandHidden ? "hidden" : "calibrating");
        setState(d == es3video::DemandActive ? LinkState::Active : LinkState::Paused);
        publishVideo();
        return;
    }
    case es3video::RecordResult: {
        es3video::Reason reason = es3video::ReasonOk;
        if (es3video::decodeResult(r, reason) != es3video::ReasonOk) {
            ++w.st.malformedRecords;
            return;
        }
        ++w.st.resultReasons[reason];
        // A rejected frame still being sent: stop sending it (the device dropped it).
        if (reason != es3video::ReasonOk && w.cur.active && w.cur.frameId == r.frameId)
            abandonCurrent(false, 0, false);
        Io::Outstanding* e = findFrame(r.frameId);
        if (!e) {
            ++w.st.unknownFrameRecords;
            return;
        }
        w.s.progressUs = now;
        if (!e->resulted) {
            e->resulted = true;
            ++w.s.results;
            if (reason == es3video::ReasonOk) {
                ++w.st.framesCompleted;
                return;
            }
        }
        ++w.st.framesDropped;   // a rejection, or the second RESULT after Ok
        e->used = false;
        return;
    }
    case es3video::RecordPresented: {
        es3video::PresentedInfo pi;
        if (es3video::decodePresented(r, pi) != es3video::ReasonOk) {
            ++w.st.malformedRecords;
            return;
        }
        Io::Outstanding* e = findFrame(r.frameId);
        if (!e) {
            ++w.st.unknownFrameRecords;
            return;
        }
        w.s.progressUs = now;
        if (!e->resulted) {   // PRESENTED implies the assembly was fine
            e->resulted = true;
            ++w.s.results;
            ++w.st.framesCompleted;
        }
        ++w.st.framesPresented;
        w.st.lastDecodeUs = pi.decodeUs;
        w.st.lastBlitUs   = pi.blitUs;
        {
            std::lock_guard<std::mutex> lk(statsMu);
            if (e->startUs) presentRing.push(double(now - e->startUs) / 1000.0);
            const int64_t cap = int64_t(e->captureUs);
            if (cap > 0 && cap <= now && now - cap < kMaxCaptureLatencyUs) captureRing.push(double(now - cap) / 1000.0);
        }
        e->used = false;
        return;
    }
    default:
        ++w.st.unexpectedRecords;
        return;
    }
}

// ─── worker: session bookkeeping ──────────────────────────────────────────────

Es3DisplayLink::Impl::Io::Outstanding* Es3DisplayLink::Impl::findFrame(uint32_t frameId) {
    for (Io::Outstanding& e : io->ring)
        if (e.used && e.frameId == frameId) return &e;
    return nullptr;
}

void Es3DisplayLink::Impl::addFrame(uint32_t frameId, uint64_t captureUs) {
    Io&              w    = *io;
    Io::Outstanding* slot = nullptr;
    for (Io::Outstanding& e : w.ring) {
        if (!e.used) { slot = &e; break; }
        if (!slot || e.frameId < slot->frameId) slot = &e;   // else the oldest
    }
    if (slot->used) ++w.st.framesLost;   // the device never finished it; forget it
    *slot           = Io::Outstanding{};
    slot->used      = true;
    slot->frameId   = frameId;
    slot->captureUs = captureUs;
}

/// Forget the video session: unresolved frames are lost, credits gone.
void Es3DisplayLink::Impl::endSession() {
    Io& w = *io;
    abandonCurrent(false, 0);
    for (Io::Outstanding& e : w.ring) {
        if (e.used) ++w.st.framesLost;
        e = Io::Outstanding{};
    }
    w.s            = Io::Session{};
    w.resyncWanted = false;
}

/// Stop sending the current frame.  The record already in tx still finishes;
/// with @p sendAbort an ABORT for the frame follows it.  @p count = false when
/// the device itself rejected the frame (that is a drop, not a host abandon).
void Es3DisplayLink::Impl::abandonCurrent(bool sendAbort, uint8_t reason, bool count) {
    Io& w = *io;
    if (!w.cur.active) return;
    if (w.cur.phase != Io::Phase::Abort) {   // an ABORT already pending was counted then
        if (count) ++w.st.framesAbandoned;
        w.cur.lease.release();
    }
    if (sendAbort && w.s.token != 0 && w.fd >= 0) {
        w.cur.phase       = Io::Phase::Abort;
        w.cur.abortReason = reason;
        return;   // buildVideo() writes the ABORT and clears active
    }
    w.cur.active = false;
}

void Es3DisplayLink::Impl::negotiationFailed(const std::string& why) {
    Io& w = *io;
    ++w.st.negotiationFailures;
    const uint32_t backoff = std::max<uint32_t>(cfg.reconnectMs, kMinNegotiateBackoffMs);
    w.negotiateNotBeforeUs = nowUs() + int64_t(backoff) * 1000;
    logQuiet(QNegotiate, LogLevel::WARN, "ES3 display: video negotiation failed (%s) - retrying in %u ms",
             why.c_str(), backoff);
    endSession();
    setState(LinkState::Negotiated);
    // The pending frame would trigger the retry at once; newer frames will do.
    EncodedFrameLease dropped;
    {
        std::lock_guard<std::mutex> lk(handoffMu);
        if (pending) dropped = std::move(pending);
    }
    if (dropped) {
        ++w.st.framesReleased;
        dropped.release();
    }
    publishVideo();
}

void Es3DisplayLink::Impl::resync(int64_t now) {
    Io& w = *io;
    ++w.st.resyncs;
    logQuiet(QStall, LogLevel::WARN,
             "ES3 display: no credit or RESULT for %u ms with frames outstanding - renegotiating the video session",
             stallBoundMs());
    abandonCurrent(false, 0);
    w.resyncWanted = true;
    w.s.progressUs = now;
}

void Es3DisplayLink::Impl::rehello(const char* why) {
    Io& w = *io;
    ++w.st.rehellos;
    logLine(LogLevel::DEBUG, "ES3 display: restarting the handshake (%s)", why);
    endSession();   // a new HELLO resets the device's session and its video session
    w.helloOk   = false;
    w.wantHello = true;
    w.wantCaps  = false;
    setState(LinkState::Hello);
    publishVideo();
}

} // namespace dashcam::es3display
