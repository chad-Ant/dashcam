/**
 * @file libes3display.cpp
 * @brief Es3DisplayLink: the ES3C28P tty owner (see libes3display.h).
 *
 * Layout: Impl holds what other threads touch (the published demand word, the
 * pending frame and the control queue under handoffMu, the stats snapshot under
 * statsMu); Io holds what only the worker touches (the tty, the parser, the TX
 * record, the control slot, the video session and the frame in transmission).
 * Handoff/stats locks are never held across tty I/O, a log line or a callback.
 * The separate wake mutex protects the nonblocking eventfd write against close.
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
constexpr uint8_t  kCapsTriesPerOpen      = 2;     ///< MAP/PAN_CAPABILITIES attempts per open before doing without.
constexpr uint16_t kMinPanMarginPx        = 32;    ///< A panning background must leave this much around the map area.
constexpr uint32_t kPanResendGapMs        = 2000;  ///< At most one resend request per this after lost pan images.

// navigation::PanPose / Marker are handed to the wire unchanged: they must match the contract.
static_assert(uint8_t(dashcam::navigation::Marker::None) == es3map::MarkerNone &&
                  uint8_t(dashcam::navigation::Marker::Dot) == es3map::MarkerDot &&
                  uint8_t(dashcam::navigation::Marker::Arrow) == es3map::MarkerArrow &&
                  uint8_t(dashcam::navigation::Marker::LastKnown) == es3map::MarkerLastKnown,
              "navigation::Marker must keep es3map::PoseMarker's values");
static_assert(dashcam::navigation::kPoseSubpixels == es3map::kPoseSubpixels,
              "PanPose coordinates must use the wire's subpixel scale");

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
    case es3map::CmdMapCapabilities: return "MAP_CAPABILITIES";
    case es3map::CmdStartMap:        return "START_MAP";
    case es3map::CmdStopMap:         return "STOP_MAP";
    case es3map::CmdPanCapabilities: return "PAN_CAPABILITIES";
    case es3map::CmdStartPan:        return "START_PAN";
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
    if (c.map) {
        if (c.mapTargetFps < 1 || c.mapTargetFps > es3video::kMaxTargetFps) return fail("mapTargetFps must be 1-30");
        if (c.mapMaxJpegBytes < 1024 || c.mapMaxJpegBytes > es3video::kMaxJpegBytes)
            return fail("mapMaxJpegBytes must be 1024-65536");
        if (c.mapStaleAfterMs < 1 || c.mapStaleAfterMs > es3video::kMaxTimeoutMs)
            return fail("mapStaleAfterMs must be 1-10000");
        if (c.mapPlaceholderAfterMs < c.mapStaleAfterMs || c.mapPlaceholderAfterMs > es3video::kMaxTimeoutMs)
            return fail("mapPlaceholderAfterMs must be mapStaleAfterMs-10000");
        if (c.mapPan && (c.mapPanBackgroundW < 320 || c.mapPanBackgroundW > es3map::kMaxPanBackground))
            return fail("mapPanBackgroundW must be 320-1024");
        if (c.mapPan && (c.mapPanBackgroundH < 64 || c.mapPanBackgroundH > es3map::kMaxPanBackground))
            return fail("mapPanBackgroundH must be 64-1024");
    }
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
    if (index >= kMaxPageCount || !printableField(title) ||
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
        bool     txYielded = false;        ///< Fairness budget used; resume after checking RX/timers.
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
            es3map::StartMapRequest     map;    ///< type == CmdStartMap.
            es3map::StartPanRequest     pan;    ///< type == CmdStartPan.
            uint32_t                    stopToken = 0;  ///< type == StopVideo / CmdStopMap (a source switch).
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

        // Map surface (per open).
        bool     wantMapCaps = false, mapKnown = false, mapSupported = false;
        es3map::MapCapabilities mapCaps;
        uint32_t mapMaxJpeg = 0, mapGeneration = 0;
        bool     wantPanCaps = false, panKnown = false, panSupported = false;
        uint8_t  mapCapsTries = 0, panCapsTries = 0;   ///< Asked this open (a timeout re-HELLOs).
        es3map::PanCapabilities panCaps;
        uint8_t  page = 0xff;              ///< Board's active page from STATUS; 0xff unknown.
        int64_t  nextPollUs = 0;

        // Video session.
        enum class Kind : uint8_t { None = 0, Video = 1, Map = 2 };
        struct Session {
            Kind     kind = Kind::None;
            uint32_t token = 0;
            es3video::StartVideoRequest req;
            uint32_t grant = 0, started = 0, results = 0, nextFrameId = 1;
            uint8_t  visibility = es3video::DemandActive;
            int64_t  progressUs = 0;
            bool     creditBlocked = false;
            bool     pan = false;           ///< A panning map session (extension v2).
            es3map::PanSession panGeo;     ///< pan == true.
            uint32_t poseSeq = 0;           ///< Last MAP_POSE seq sent.
            int64_t  lastPoseUs = 0;
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
            uint8_t           panKind = 0;   ///< es3map::PanImage in a panning session.
        } cur;

        // Started frames still owed a RESULT / PRESENTED.
        struct Outstanding {
            bool     used = false, resulted = false;
            uint32_t frameId = 0;
            int64_t  startUs = 0;           ///< START fully written (0 = not yet).
            uint64_t captureUs = 0;
            uint8_t  panKind = 0;           ///< es3map::PanImage of a panning session's frame.
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
    std::mutex                wakeMu;     ///< Serialises notification with descriptor replacement/close.
    std::atomic<bool>         running{false};
    std::atomic<bool>         stopRequested{false};
    std::unique_ptr<Io>       io;

    // Published by the worker.
    std::atomic<uint64_t> demandWord{0};
    std::atomic<uint64_t> mapDemandWord{0};
    std::atomic<uint8_t>  stateValue{static_cast<uint8_t>(LinkState::Disconnected)};

    // Handoff between API callers and the worker (handoffMu).
    std::mutex        handoffMu;
    bool              acceptVideo = false;
    bool              acceptControl = false;
    uint16_t          acceptMaxW = 0, acceptMaxH = 0;
    uint32_t          acceptMaxJpeg = 0;
    EncodedFrameLease pending;
    bool              acceptMap = false;
    uint16_t          acceptMapW = 0, acceptMapH = 0;
    uint32_t          acceptMapJpeg = 0;
    uint8_t           acceptPages = kPageCount;   ///< PAGE indexes the board has.
    EncodedFrameLease mapPending;
    // Panning map (extension v2): one pending background, chrome and pose.
    dashcam::navigation::PanDemand panDemandNow;   ///< What panDemand() reports.
    uint32_t          panResend = 0;               ///< PanDemand::resend: bumped when the board needs images again.
    int64_t           lastLossResendUs = 0;        ///< Rate limit for resends after a lost image...
    bool              lossResendDue = false;       ///< ... and one deferred by it (bumped by checkTimers()).
    EncodedFrameLease panBgPending, panChromePending;
    uint64_t          panBgOrigin = 0;            ///< es3map::packOrigin() of panBgPending.
    std::atomic<bool> posePending{false};          ///< Written under handoffMu; read lock-free for timers.
    dashcam::navigation::PanPose pose;
    std::array<Queued, kControlQueueDepth> queue{};
    size_t            qHead = 0, qCount = 0;

    // Caller-side counters.
    std::atomic<uint64_t> framesOffered{0}, framesAccepted{0}, framesRefused{0}, framesReplaced{0},
        controlEnqueued{0}, controlRejected{0}, controlQueueFull{0}, mapOffered{0}, mapAccepted{0},
        panBgOffered{0}, panBgAccepted{0}, panChromeOffered{0}, panChromeAccepted{0}, posesOffered{0},
        posesAccepted{0}, posesReplaced{0};

    /// The map surface's IPreviewSink (Es3DisplayLink::mapSink()).
    class MapSink : public dashcam::preview::IPreviewSink {
    public:
        explicit MapSink(Impl& m) : m_(m) {}
        dashcam::preview::PreviewDemand demand() const override;
        bool offerFrame(EncodedFrameLease&& frame) override;

    private:
        Impl& m_;
    };
    MapSink mapSinkObj{*this};

    /// The panning map surface (Es3DisplayLink::panSink()).
    class PanSink : public dashcam::navigation::IMapPanSink {
    public:
        explicit PanSink(Impl& m) : m_(m) {}
        dashcam::navigation::PanDemand panDemand() const override;
        bool offerPanBackground(EncodedFrameLease&& jpeg, uint32_t originX, uint32_t originY) override;
        bool offerPanChrome(EncodedFrameLease&& jpeg) override;
        bool offerPanPose(const dashcam::navigation::PanPose& pose) override;

    private:
        Impl& m_;
    };
    PanSink panSinkObj{*this};

    // Snapshot for stats() (statsMu).
    mutable std::mutex statsMu;
    DisplayLinkStats   published;
    LatencyRing        presentRing, captureRing, controlRing;

    // ── helpers used from any thread ─────────────────────────────────────────
    void wake() {
        std::lock_guard<std::mutex> lk(wakeMu);
        if (wakeFd < 0) return;
        const uint64_t one = 1;
        ssize_t n;
        do { n = ::write(wakeFd, &one, sizeof(one)); } while (n < 0 && errno == EINTR);
        // EAGAIN means the counter already contains a wakeup.
    }
    void closeWake() {
        // Called only before worker launch or after join: worker reads are done.
        std::lock_guard<std::mutex> lk(wakeMu);
        if (wakeFd >= 0) ::close(wakeFd);
        wakeFd = -1;
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
    bool startImage(int64_t now);
    bool continueImage(int64_t now);
    EncodedFrameLease* nextImage();   ///< Requires handoffMu; worker only.
    bool kernelQueueAllows();
    bool needNegotiation(uint16_t& w, uint16_t& h);
    bool buildStartVideo(int64_t now, uint16_t w, uint16_t h);
    bool buildStartMap();
    bool buildPose(int64_t now);
    void dropPanPending();
    Io::Kind desiredKind() const;
    bool sourceSwitch();   ///< Builds the STOP for a session the active page no longer wants.
    EncodedFrameLease& pendingFor(Io::Kind k) { return k == Io::Kind::Map ? mapPending : pending; }

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
    void panImageLost(int64_t now);
    void beginSession(Io::Kind kind, uint32_t token, const es3video::StartVideoRequest& limits, uint32_t grant,
                      const es3map::PanSession* pan);
    void onStopDone(const Io::Control& done);
    void negotiationFailed(const std::string& why);
    void resync(int64_t now);
    Io::Outstanding* findFrame(uint32_t frameId);
    void addFrame(uint32_t frameId, uint64_t captureUs);
    bool outstandingWork();
    bool startingSession() const {
        const Io::Control& c = io->ctl;
        return c.busy && (c.type == es3proto::StartVideo || c.type == es3map::CmdStartMap ||
                          c.type == es3map::CmdStartPan);
    }

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

/// Recompute what both sinks accept and what their demand() reports; a frame
/// still pending when its surface goes inactive is released here.
void Es3DisplayLink::Impl::publishVideo() {
    Io&        w       = *io;
    using Kind         = Io::Kind;
    const bool base    = w.fd >= 0 && w.helloOk && !w.stopping;
    const bool visible = w.s.visibility == es3video::DemandActive;
    // With a map-capable board the active page decides the camera's turn: LIVE only.
    const bool videoTurn = !w.mapSupported || w.page == 0;
    const bool videoActive = base && w.videoSupported && videoTurn &&
                             (w.s.token == 0 ? true : (w.s.kind == Kind::Video && visible));
    const bool mapShown = base && w.mapSupported && w.page == w.mapCaps.mapPage && w.s.token != 0 &&
                          w.s.kind == Kind::Map && visible;
    const bool mapActive = mapShown && !w.s.pan;
    const bool panActive = mapShown && w.s.pan;
    EncodedFrameLease dropped, droppedMap, droppedBg, droppedChrome;
    {
        std::lock_guard<std::mutex> lk(handoffMu);
        acceptVideo   = videoActive;
        acceptMaxW    = w.videoSupported ? w.caps.maxWidth : 0;
        acceptMaxH    = w.videoSupported ? w.caps.maxHeight : 0;
        acceptMaxJpeg = w.videoSupported ? w.maxJpeg : 0;
        if (!videoActive && pending) dropped = std::move(pending);
        demandWord.store(w.videoSupported ? packDemand(videoActive, w.caps.canvasWidth, w.caps.canvasHeight, w.maxJpeg)
                                          : 0,
                         std::memory_order_release);
        acceptMap     = mapActive;
        acceptMapW    = w.mapSupported ? w.mapCaps.viewportW : 0;
        acceptMapH    = w.mapSupported ? w.mapCaps.viewportH : 0;
        acceptMapJpeg = w.mapSupported ? w.mapMaxJpeg : 0;
        acceptPages   = w.mapSupported ? w.mapCaps.pageCount : kPageCount;
        if (!mapActive && mapPending) droppedMap = std::move(mapPending);
        mapDemandWord.store(w.mapSupported ? packDemand(mapActive, w.mapCaps.viewportW, w.mapCaps.viewportH,
                                                        w.mapMaxJpeg)
                                           : 0,
                            std::memory_order_release);
        dashcam::navigation::PanDemand pd;
        if (w.s.token != 0 && w.s.pan) {
            const es3map::PanSession& g = w.s.panGeo;
            pd.active        = panActive;
            pd.session       = w.s.token;
            pd.mapW          = g.mapW;
            pd.mapH          = g.mapH;
            pd.backgroundW   = g.request.backgroundW;
            pd.backgroundH   = g.request.backgroundH;
            pd.chromeTopH    = g.chromeTopH;
            pd.chromeBottomH = g.chromeBottomH;
            pd.maxJpegBytes  = g.request.map.maxJpegBytes;
            pd.maxPoseHz     = w.panCaps.maxPoseHz;
        }
        const bool sameSession = pd.session == panDemandNow.session;
        // Shown again within the same session: the board dropped its images when
        // it hid the map, so the producer must send them again (a new session
        // says so by itself).
        if (pd.active && !panDemandNow.active && sameSession) ++panResend;
        pd.resend    = panResend;
        panDemandNow = pd;
        if (!panActive || !sameSession) {   // images and poses belong to the session they were made for
            if (panBgPending) droppedBg = std::move(panBgPending);
            if (panChromePending) droppedChrome = std::move(panChromePending);
            posePending = false;
        }
    }
    if (dropped) {
        ++w.st.framesReleased;
        dropped.release();
    }
    for (EncodedFrameLease* l : {&droppedMap, &droppedBg, &droppedChrome})
        if (*l) {
            ++w.st.framesReleased;
            l->release();
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
    w.st.mapSupported     = w.mapSupported;
    w.st.mapCapabilities  = w.mapCaps;
    w.st.activePage       = w.page;
    w.st.sessionKind      = static_cast<uint8_t>(w.s.token ? w.s.kind : Io::Kind::None);
    w.st.panSupported     = w.panSupported;
    w.st.panCapabilities  = w.panCaps;
    w.st.panSession       = w.s.token != 0 && w.s.pan;
    w.st.panBackgroundW   = w.st.panSession ? w.s.panGeo.request.backgroundW : 0;
    w.st.panBackgroundH   = w.st.panSession ? w.s.panGeo.request.backgroundH : 0;
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
    m.cfg    = cfg;
    m.log    = std::move(log);
    m.io     = std::make_unique<Impl::Io>();
    m.io->st.device = cfg.device;
    m.io->nextOpenUs = nowUs();
    for (auto* c : {&m.framesOffered, &m.framesAccepted, &m.framesRefused, &m.framesReplaced,
                    &m.controlEnqueued, &m.controlRejected, &m.controlQueueFull, &m.mapOffered, &m.mapAccepted,
                    &m.panBgOffered, &m.panBgAccepted, &m.panChromeOffered, &m.panChromeAccepted, &m.posesOffered,
                    &m.posesAccepted, &m.posesReplaced})
        c->store(0, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lk(m.handoffMu);
        m.acceptVideo = m.acceptControl = false;
        m.qHead = m.qCount = 0;
        m.panDemandNow = dashcam::navigation::PanDemand{};
        m.posePending  = false;
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
    // Allocate the descriptor after potentially throwing setup, so an
    // allocation failure cannot strand it before the worker exists.
    const int efd = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (efd < 0) {
        m.io.reset();
        dashcam::log::logPrintf(m.log, LogLevel::ERROR, "ES3 display: eventfd failed: %s - display link off",
                                std::strerror(errno));
        return false;
    }
    {
        std::lock_guard<std::mutex> lk(m.wakeMu);
        m.wakeFd = efd;
    }
    m.running.store(true, std::memory_order_release);
    try {
        m.worker = std::thread(&Impl::workerLoop, &m);
    } catch (const std::system_error& e) {
        m.running.store(false);
        m.closeWake();
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
    m.closeWake();
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

dashcam::preview::IPreviewSink& Es3DisplayLink::mapSink() { return impl_->mapSinkObj; }

PreviewDemand Es3DisplayLink::Impl::MapSink::demand() const {
    const uint64_t w = m_.mapDemandWord.load(std::memory_order_acquire);
    PreviewDemand  d;
    d.active       = (w >> 63) != 0;
    d.canvasW      = static_cast<uint16_t>(w >> 40);
    d.canvasH      = static_cast<uint16_t>(w >> 24);
    d.maxJpegBytes = static_cast<size_t>(w & 0xffffffu);
    return d;
}

bool Es3DisplayLink::Impl::MapSink::offerFrame(EncodedFrameLease&& frame) {
    Impl& m = m_;
    m.mapOffered.fetch_add(1, std::memory_order_relaxed);
    if (!frame || frame.data() == nullptr || frame.size() == 0) return false;
    EncodedFrameLease replaced;
    {
        std::lock_guard<std::mutex> lk(m.handoffMu);
        if (!m.running.load(std::memory_order_acquire) || !m.acceptMap || frame.width() != m.acceptMapW ||
            frame.height() != m.acceptMapH || frame.size() > m.acceptMapJpeg)
            return false;   // the caller keeps the lease
        replaced     = std::move(m.mapPending);
        m.mapPending = std::move(frame);
    }
    m.mapAccepted.fetch_add(1, std::memory_order_relaxed);
    if (replaced) replaced.release();
    m.wake();
    return true;
}

dashcam::navigation::IMapPanSink& Es3DisplayLink::panSink() { return impl_->panSinkObj; }

dashcam::navigation::PanDemand Es3DisplayLink::Impl::PanSink::panDemand() const {
    std::lock_guard<std::mutex> lk(m_.handoffMu);
    return m_.panDemandNow;
}

bool Es3DisplayLink::Impl::PanSink::offerPanBackground(EncodedFrameLease&& jpeg, uint32_t originX, uint32_t originY) {
    Impl& m = m_;
    m.panBgOffered.fetch_add(1, std::memory_order_relaxed);
    if (!jpeg || jpeg.data() == nullptr || jpeg.size() == 0) return false;
    EncodedFrameLease replaced;
    {
        std::lock_guard<std::mutex> lk(m.handoffMu);
        const dashcam::navigation::PanDemand& d = m.panDemandNow;
        if (!m.running.load(std::memory_order_acquire) || !d.active || jpeg.width() != d.backgroundW ||
            jpeg.height() != d.backgroundH || jpeg.size() > d.maxJpegBytes)
            return false;   // the caller keeps the lease
        replaced        = std::move(m.panBgPending);
        m.panBgPending  = std::move(jpeg);
        m.panBgOrigin   = es3map::packOrigin(originX, originY);
    }
    m.panBgAccepted.fetch_add(1, std::memory_order_relaxed);
    if (replaced) replaced.release();
    m.wake();
    return true;
}

bool Es3DisplayLink::Impl::PanSink::offerPanChrome(EncodedFrameLease&& jpeg) {
    Impl& m = m_;
    m.panChromeOffered.fetch_add(1, std::memory_order_relaxed);
    if (!jpeg || jpeg.data() == nullptr || jpeg.size() == 0) return false;
    EncodedFrameLease replaced;
    {
        std::lock_guard<std::mutex> lk(m.handoffMu);
        const dashcam::navigation::PanDemand& d = m.panDemandNow;
        if (!m.running.load(std::memory_order_acquire) || !d.active || jpeg.width() != d.mapW ||
            jpeg.height() != d.chromeTopH + d.chromeBottomH || jpeg.size() > d.maxJpegBytes)
            return false;
        replaced           = std::move(m.panChromePending);
        m.panChromePending = std::move(jpeg);
    }
    m.panChromeAccepted.fetch_add(1, std::memory_order_relaxed);
    if (replaced) replaced.release();
    m.wake();
    return true;
}

bool Es3DisplayLink::Impl::PanSink::offerPanPose(const dashcam::navigation::PanPose& pose) {
    Impl& m = m_;
    m.posesOffered.fetch_add(1, std::memory_order_relaxed);
    if (pose.courseCdeg >= 36000 || static_cast<uint8_t>(pose.marker) > es3map::MarkerLastKnown) return false;
    bool replaced = false;
    {
        std::lock_guard<std::mutex> lk(m.handoffMu);
        if (!m.running.load(std::memory_order_acquire) || !m.panDemandNow.active) return false;
        replaced      = m.posePending;
        m.pose        = pose;
        m.posePending = true;
    }
    m.posesAccepted.fetch_add(1, std::memory_order_relaxed);
    if (replaced) m.posesReplaced.fetch_add(1, std::memory_order_relaxed);
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
        if (!m.running.load(std::memory_order_acquire) || !m.acceptControl ||
            (request.command == es3proto::Page && request.payload[0] >= m.acceptPages)) {
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
    s.mapFramesOffered  = m.mapOffered.load(std::memory_order_relaxed);
    s.mapFramesAccepted = m.mapAccepted.load(std::memory_order_relaxed);
    s.panBackgroundsOffered  = m.panBgOffered.load(std::memory_order_relaxed);
    s.panBackgroundsAccepted = m.panBgAccepted.load(std::memory_order_relaxed);
    s.panChromeOffered       = m.panChromeOffered.load(std::memory_order_relaxed);
    s.panChromeAccepted      = m.panChromeAccepted.load(std::memory_order_relaxed);
    s.posesOffered           = m.posesOffered.load(std::memory_order_relaxed);
    s.posesAccepted          = m.posesAccepted.load(std::memory_order_relaxed);
    s.posesReplaced          = m.posesReplaced.load(std::memory_order_relaxed);
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
    w.wantMapCaps = w.mapKnown = w.mapSupported = false;
    w.mapCaps     = es3map::MapCapabilities{};
    w.wantPanCaps = w.panKnown = w.panSupported = false;
    w.panCaps     = es3map::PanCapabilities{};
    w.mapCapsTries = w.panCapsTries = 0;
    w.page        = 0xff;
    w.nextPollUs  = 0;
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
    w.wantMapCaps = w.mapKnown = w.mapSupported = false;
    w.wantPanCaps = w.panKnown = w.panSupported = false;
    w.page        = 0xff;
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
        const bool panImage = w.s.pan && w.cur.panKind != es3map::PanImageNone;
        abandonCurrent(true, es3video::ReasonAssemblyTimeout);
        if (panImage) panImageLost(now);
    }

    // A lost pan image deferred by the resend rate limit: ask for it now.
    {
        std::lock_guard<std::mutex> lk(handoffMu);
        if (lossResendDue && now - lastLossResendUs >= int64_t(kPanResendGapMs) * 1000) {
            lossResendDue       = false;
            lastLossResendUs    = now;
            panDemandNow.resend = ++panResend;
        }
    }

    // Credit stall: frames outstanding, no grant/RESULT progress.
    if (w.s.token != 0 && !startingSession() && !w.resyncWanted && !w.stopping) {
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
    return w.s.pan ? bool(panBgPending) || bool(panChromePending) : bool(pendingFor(w.s.kind));
}

int Es3DisplayLink::Impl::pollTimeoutMs(int64_t now) const {
    const Io& w = *io;
    if (w.txYielded) return 0;   // yielding for fairness is not waiting for I/O
    int64_t   t = int64_t(kPollSliceMs) * 1000;
    auto      until = [&](int64_t at) { t = std::min(t, at - now); };
    if (w.ctl.busy && w.ctl.written) until(w.ctl.deadlineUs);
    if (!w.ctl.busy && w.helloOk && !w.stopping) until(w.lastRequestUs + int64_t(cfg.heartbeatMs) * 1000);
    if (!w.ctl.busy && w.helloOk && !w.stopping && w.mapSupported) until(w.nextPollUs);
    if (w.s.pan && w.s.lastPoseUs && w.panCaps.maxPoseHz && posePending.load(std::memory_order_relaxed))
        until(w.s.lastPoseUs + 1000000 / int64_t(w.panCaps.maxPoseHz));
    if (w.waitingOutq) t = std::min<int64_t>(t, int64_t(w.pollOutDrains ? kOutqSafetyMs : kOutqSliceMs) * 1000);
    if (w.cur.active && w.cur.startWrittenUs && w.cur.phase != Io::Phase::Abort)
        until(w.cur.startWrittenUs + int64_t(cfg.assemblyTimeoutMs) * 1000);
    if (w.s.token && !startingSession() && !w.resyncWanted && !w.stopping)
        until(w.s.progressUs + int64_t(stallBoundMs()) * 1000);
    if (w.negotiateNotBeforeUs > now) until(w.negotiateNotBeforeUs);
    if (w.txOff < w.txLen) until(w.txProgressUs + int64_t(txStallMs()) * 1000);
    if (w.stopping) until(w.stopDeadlineUs);
    if (t <= 0) return 0;
    return static_cast<int>(std::min<int64_t>((t + 999) / 1000, kPollSliceMs));
}

// ─── worker: transmit ─────────────────────────────────────────────────────────

void Es3DisplayLink::Impl::pumpTx() {
    Io& w = *io;
    w.txYielded = false;
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
    w.txYielded = w.fd >= 0;
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
        } else if (w.txVideoType == es3video::RecordMapPose) {
            ++w.st.posesSent;
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
    w.ctl.user       = false;   // per request: the callers set what applies after this
    w.ctl.stopToken  = 0;
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
            buildControlFrame(w.s.kind == Io::Kind::Map ? uint8_t(es3map::CmdStopMap)
                                                                 : uint8_t(es3proto::StopVideo),
                              p, sizeof(p));
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
    if (w.wantMapCaps) {
        w.wantMapCaps = false;
        ++w.mapCapsTries;
        buildControlFrame(es3map::CmdMapCapabilities, nullptr, 0);
        return true;
    }
    if (w.wantPanCaps) {
        w.wantPanCaps = false;
        ++w.panCapsTries;
        buildControlFrame(es3map::CmdPanCapabilities, nullptr, 0);
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

    using Kind = Io::Kind;
    if (sourceSwitch()) return true;
    const Kind want = desiredKind();
    // A panning board is asked first: START_PAN waits for its PAN_CAPABILITIES answer.
    const bool panAnswered = !cfg.mapPan || w.panKnown;
    if (want == Kind::Map && panAnswered && now >= w.negotiateNotBeforeUs &&
        (w.s.token == 0 || (w.s.kind == Kind::Map && w.resyncWanted)) && buildStartMap())
        return true;
    uint16_t vw = 0, vh = 0;
    if (w.videoSupported && want == Kind::Video && (w.s.token == 0 || w.s.kind == Kind::Video) &&
        now >= w.negotiateNotBeforeUs && needNegotiation(vw, vh) && buildStartVideo(now, vw, vh)) {
        return true;
    }

    // The board's active page decides the image source (map-capable boards only).
    if (w.mapSupported && now >= w.nextPollUs) {
        const bool shown = w.s.token != 0 && w.s.visibility == es3video::DemandActive;
        w.nextPollUs = now + int64_t(shown ? kPagePollActiveMs : kPagePollIdleMs) * 1000;
        ++w.st.pagePolls;
        buildControlFrame(es3proto::Status, nullptr, 0);
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

/// Which surface the board's receiver should serve: the camera on LIVE, the map
/// on the map page, None (keep the current session, paused) elsewhere.  Without
/// the map extension it is always the camera, exactly as before.
Es3DisplayLink::Impl::Io::Kind Es3DisplayLink::Impl::desiredKind() const {
    const Io& w = *io;
    using Kind  = Io::Kind;
    if (!w.mapSupported) return w.videoSupported ? Kind::Video : Kind::None;
    if (w.page == w.mapCaps.mapPage) return Kind::Map;
    if (w.page == 0) return w.videoSupported ? Kind::Video : Kind::None;
    return Kind::None;
}

bool Es3DisplayLink::Impl::sourceSwitch() {
    Io& w = *io;
    using Kind = Io::Kind;
    if (!w.mapSupported || w.s.token == 0) return false;
    const Kind want = desiredKind();
    if (want == Kind::None || want == w.s.kind) return false;
    // The receiver is single: end this session before the other may start.
    abandonCurrent(false, 0);
    uint8_t p[es3video::kStopVideoRequestBytes];
    es3video::encodeStopVideoRequest(w.s.token, p, sizeof(p));
    buildControlFrame(w.s.kind == Kind::Map ? uint8_t(es3map::CmdStopMap) : uint8_t(es3proto::StopVideo), p,
                      sizeof(p));
    w.ctl.stopToken = w.s.token;
    ++w.st.sourceSwitches;
    logLine(LogLevel::DEBUG, "ES3 display: page %u - switching the image receiver from %s to %s", unsigned(w.page),
            w.s.kind == Kind::Map ? "map" : "camera", want == Kind::Map ? "map" : "camera");
    return true;
}

bool Es3DisplayLink::Impl::buildStartMap() {
    Io&                     w = *io;
    es3map::StartMapRequest r;
    r.page               = w.mapCaps.mapPage;
    r.viewportX          = w.mapCaps.viewportX;
    r.viewportY          = w.mapCaps.viewportY;
    r.viewportW          = w.mapCaps.viewportW;
    r.viewportH          = w.mapCaps.viewportH;
    r.maxJpegBytes       = w.mapMaxJpeg;
    r.chunkBytes         = w.chunk;
    r.targetFps          = cfg.mapTargetFps;
    r.assemblyTimeoutMs  = cfg.assemblyTimeoutMs;
    r.staleAfterMs       = cfg.mapStaleAfterMs;
    r.placeholderAfterMs = cfg.mapPlaceholderAfterMs;
    r.generation         = ++w.mapGeneration;
    if (w.panSupported) {
        es3map::StartPanRequest pr;
        pr.map = r;
        pr.map.targetFps = std::min<uint8_t>(cfg.mapTargetFps, 2);   // backgrounds are rare
        const uint16_t mapH = es3map::panMapHeight(w.mapCaps, w.panCaps);
        pr.backgroundW = std::clamp<uint16_t>(cfg.mapPanBackgroundW, w.mapCaps.viewportW, w.panCaps.maxBackgroundW);
        pr.backgroundH = std::clamp<uint16_t>(cfg.mapPanBackgroundH, mapH, w.panCaps.maxBackgroundH);
        uint8_t pp[es3map::kStartPanRequestBytes];
        if (es3map::encodeStartPanRequest(pr, w.panCaps, w.mapCaps, w.caps, pp, sizeof(pp)) == 0) {
            negotiationFailed("the START_PAN request does not fit the board's pan capabilities");
            return false;
        }
        abandonCurrent(false, 0);
        buildControlFrame(es3map::CmdStartPan, pp, sizeof(pp));
        w.ctl.pan = pr;
        setState(LinkState::Negotiated);
        return true;
    }
    uint8_t p[es3map::kStartMapRequestBytes];
    if (es3map::encodeStartMapRequest(r, w.mapCaps, w.caps, p, sizeof(p)) == 0) {
        negotiationFailed("the START_MAP request does not fit the board's map capabilities");
        return false;
    }
    abandonCurrent(false, 0);
    buildControlFrame(es3map::CmdStartMap, p, sizeof(p));
    w.ctl.map = r;
    setState(LinkState::Negotiated);
    return true;
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

/// The pending pose as a MAP_POSE record, at most the board's maxPoseHz.
bool Es3DisplayLink::Impl::buildPose(int64_t now) {
    Io& w = *io;
    if (w.s.visibility != es3video::DemandActive || w.resyncWanted || !w.panCaps.maxPoseHz) return false;
    if (w.s.lastPoseUs && now - w.s.lastPoseUs < 1000000 / int64_t(w.panCaps.maxPoseHz)) return false;
    {
        std::lock_guard<std::mutex> lk(handoffMu);
        if (!posePending) return false;
    }
    if (!kernelQueueAllows()) return false;
    es3map::MapPose p;
    {
        std::lock_guard<std::mutex> lk(handoffMu);
        if (!posePending) return false;
        p.xQ4        = pose.xQ4;
        p.yQ4        = pose.yQ4;
        p.courseCdeg = pose.courseCdeg;
        p.marker     = static_cast<uint8_t>(pose.marker);
        posePending  = false;
    }
    p.seq = w.s.poseSeq + 1;
    const size_t n = es3map::encodeMapPose(w.s.token, p, w.tx, sizeof(w.tx));
    if (n == 0) return false;   // the sink validated the fields; nothing else can fail
    w.s.poseSeq    = p.seq;
    w.s.lastPoseUs = now;
    w.txLen        = n;
    w.txOff        = 0;
    w.txKind       = Io::TxKind::Video;
    w.txVideoType  = es3video::RecordMapPose;
    w.txProgressUs = now;
    return true;
}

/// Release the panning session's pending images and pose.
void Es3DisplayLink::Impl::dropPanPending() {
    Io&               w = *io;
    EncodedFrameLease bg, chrome;
    {
        std::lock_guard<std::mutex> lk(handoffMu);
        if (panBgPending) bg = std::move(panBgPending);
        if (panChromePending) chrome = std::move(panChromePending);
        posePending = false;
    }
    for (EncodedFrameLease* l : {&bg, &chrome})
        if (*l) {
            ++w.st.framesReleased;
            l->release();
        }
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
    if (w.ctl.busy && (w.ctl.type == es3proto::StartVideo || w.ctl.type == es3map::CmdStartMap ||
                       w.ctl.type == es3map::CmdStartPan || w.ctl.type == es3proto::StopVideo ||
                       w.ctl.type == es3map::CmdStopMap))
        return false;   // (re)negotiating or switching the source
    if (w.mapSupported) {   // a switch is due: nothing more for the outgoing surface
        const Io::Kind want = desiredKind();
        if (want != Io::Kind::None && want != w.s.kind) return false;
    }
    // A pose goes between any two records: it is tiny, uncredited and the
    // board moves the map with it (a background in flight takes ~100 ms).
    if (w.s.pan && buildPose(now)) return true;

    return w.cur.active ? continueImage(now) : startImage(now);
}

/// Select under handoffMu; chrome precedes a background in panning mode.
EncodedFrameLease* Es3DisplayLink::Impl::nextImage() {
    const Io& w = *io;
    if (!w.s.pan) {
        EncodedFrameLease& src = pendingFor(w.s.kind);
        return src && src.width() == w.s.req.viewportW && src.height() == w.s.req.viewportH ? &src : nullptr;
    }
    if (panChromePending) return &panChromePending;
    return panBgPending ? &panBgPending : nullptr;
}

/// Spend one credit only after a compatible lease and a valid START are ready.
bool Es3DisplayLink::Impl::startImage(int64_t now) {
    Io& w = *io;
    if (w.s.visibility != es3video::DemandActive || w.resyncWanted) return false;
    bool ready = false;
    {
        std::lock_guard<std::mutex> lk(handoffMu);
        ready = nextImage() != nullptr;
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
    uint64_t origin = 0;
    {
        std::lock_guard<std::mutex> lk(handoffMu);
        // Re-check: an offer may have replaced it meanwhile.
        EncodedFrameLease* src = nextImage();
        if (!src) return false;
        if (src == &panBgPending) origin = panBgOrigin;
        w.cur.lease = std::move(*src);
    }
    w.s.creditBlocked = false;
    es3video::StartInfo info;
    info.width             = w.cur.lease.width();
    info.height            = w.cur.lease.height();
    info.jpegBytes         = static_cast<uint32_t>(w.cur.lease.size());
    // A panning session's START carries the background origin instead of a time.
    info.sourceTimestampUs = w.s.pan ? origin : w.cur.lease.captureMonotonicUs();
    const uint32_t frameId = w.s.nextFrameId;
    es3map::PanImage panKind = es3map::PanImageNone;
    const es3video::Reason fit = w.s.pan ? es3map::checkPanStart(info, w.s.panGeo, panKind)
                                         : es3video::checkStartForSession(info, w.s.req);
    const size_t n = fit == es3video::ReasonOk ? es3video::encodeStart(w.s.token, frameId, info, w.tx, sizeof(w.tx))
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
    w.cur.panKind        = uint8_t(panKind);
    addFrame(frameId, w.s.pan ? 0 : info.sourceTimestampUs);
    if (w.s.pan) {
        if (Io::Outstanding* e = findFrame(frameId)) e->panKind = panKind;
        ++(panKind == es3map::PanImageBackground ? w.st.panBackgroundsStarted : w.st.panChromeStarted);
    }
    w.txLen        = n;
    w.txOff        = 0;
    w.txKind       = Io::TxKind::Video;
    w.txVideoType  = es3video::RecordStart;
    w.txProgressUs = now;
    return true;
}

/// Continue exactly one record; the caller never interleaves partial records.
bool Es3DisplayLink::Impl::continueImage(int64_t now) {
    Io& w = *io;

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
        const bool es3 = f.length == idLen && std::memcmp(f.payload, kDeviceIdentity, idLen) == 0;
        const bool cyd = f.length == 3 && std::memcmp(f.payload, "CYD", 3) == 0;
        if (!es3 && !cyd) {
            ++w.st.helloRejected;
            std::string got;
            for (size_t i = 0; i < f.length; ++i)
                got += (f.payload[i] >= 32 && f.payload[i] <= 126) ? char(f.payload[i]) : '.';
            logQuiet(QHello, LogLevel::ERROR,
                     "ES3 display: %s answered HELLO with \"%s\", not \"%s\" or CYD - wrong device? closing",
                     cfg.device.c_str(), got.c_str(), kDeviceIdentity);
            closeDevice("not an ES3C28P or CYD", false);
            return;
        }
        w.helloOk = true;
        w.st.identity.assign(reinterpret_cast<const char*>(f.payload), f.length);
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
            // A re-HELLO after a lost MAP/PAN_CAPABILITIES reply: ask again (twice
            // per open at most, then do without, so a broken reply cannot loop).
            if (w.videoSupported && cfg.map && !w.mapKnown) {
                if (w.mapCapsTries < kCapsTriesPerOpen) w.wantMapCaps = true;
                else w.mapKnown = true;   // mapSupported stays false
            } else if (w.mapSupported && cfg.mapPan && !w.panKnown) {
                if (w.panCapsTries < kCapsTriesPerOpen) w.wantPanCaps = true;
                else w.panKnown = true;   // whole map frames
            }
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
        if (cfg.map && !w.mapKnown) w.wantMapCaps = true;   // the map shares these limits
        publishVideo();
        return;
    }
    case es3map::CmdMapCapabilities: {
        es3map::MapCapabilities m;
        w.mapKnown = true;
        if (!es3map::decodeMapCapabilitiesReply(f.payload, f.length, w.caps, m)) {
            w.mapSupported = false;
            logQuiet(QCaps, LogLevel::WARN,
                     "ES3 display: unusable MAP_CAPABILITIES reply (%u bytes, map protocol %u) - no map",
                     unsigned(f.length), f.length ? unsigned(f.payload[0]) : 0u);
            publishVideo();
            return;
        }
        w.mapSupported = true;
        w.mapCaps      = m;
        w.mapMaxJpeg   = std::min<uint32_t>(cfg.mapMaxJpegBytes, m.maxJpegBytes);
        w.nextPollUs   = 0;   // learn the active page at once
        if (cfg.mapPan && !w.panKnown) w.wantPanCaps = true;
        logQuiet(QCaps, LogLevel::INFO, "ES3 display: map v%u on page %u of %u, viewport %u,%u %ux%u, JPEG %u B",
                 unsigned(m.mapProtocolVersion), unsigned(m.mapPage), unsigned(m.pageCount), m.viewportX,
                 m.viewportY, m.viewportW, m.viewportH, w.mapMaxJpeg);
        publishVideo();
        return;
    }
    case es3map::CmdPanCapabilities: {
        es3map::PanCapabilities p;
        w.panKnown = true;
        if (!es3map::decodePanCapabilitiesReply(f.payload, f.length, w.mapCaps, w.caps, p)) {
            w.panSupported = false;
            logQuiet(QCaps, LogLevel::WARN,
                     "ES3 display: unusable PAN_CAPABILITIES reply (%u bytes, pan version %u) - whole map frames",
                     unsigned(f.length), f.length ? unsigned(f.payload[0]) : 0u);
            return;
        }
        // The background the link will ask for must leave the board room to
        // scroll, or panning would recentre constantly: whole frames instead.
        const uint16_t mapH = es3map::panMapHeight(w.mapCaps, p);
        const uint16_t bgW  = std::clamp<uint16_t>(cfg.mapPanBackgroundW, w.mapCaps.viewportW, p.maxBackgroundW);
        const uint16_t bgH  = std::clamp<uint16_t>(cfg.mapPanBackgroundH, mapH, p.maxBackgroundH);
        if ((bgW - w.mapCaps.viewportW) / 2 < kMinPanMarginPx || (bgH - mapH) / 2 < kMinPanMarginPx) {
            w.panSupported = false;
            logQuiet(QCaps, LogLevel::WARN,
                     "ES3 display: a %ux%u panning background leaves under %u px to scroll around the %ux%u map "
                     "area - whole map frames",
                     bgW, bgH, unsigned(kMinPanMarginPx), w.mapCaps.viewportW, mapH);
            return;
        }
        w.panSupported = true;
        w.panCaps      = p;
        logQuiet(QCaps, LogLevel::INFO,
                 "ES3 display: board-side panning v%u, background up to %ux%u, strips %u+%u, %u poses/s",
                 unsigned(p.panVersion), p.maxBackgroundW, p.maxBackgroundH, unsigned(p.chromeTopH),
                 unsigned(p.chromeBottomH), unsigned(p.maxPoseHz));
        return;
    }
    case es3map::CmdStartPan: {
        es3map::StartPanReply r;
        if (!es3map::decodeStartPanReply(f.payload, f.length, r)) {
            negotiationFailed("malformed START_PAN reply");
            return;
        }
        logLine(LogLevel::DEBUG, "ES3 display: panning map session %08x (generation %u), background %ux%u, grant %u",
                r.sessionToken, done.pan.map.generation, done.pan.backgroundW, done.pan.backgroundH,
                r.initialGrant);
        const es3map::PanSession geo = es3map::panSession(done.pan, w.panCaps);
        beginSession(Io::Kind::Map, r.sessionToken, es3map::sessionLimits(done.pan.map), r.initialGrant, &geo);
        return;
    }
    case es3map::CmdStartMap: {
        es3map::StartMapReply r;
        if (!es3map::decodeStartMapReply(f.payload, f.length, r)) {
            negotiationFailed("malformed START_MAP reply");
            return;
        }
        logLine(LogLevel::DEBUG, "ES3 display: map session %08x (generation %u), grant %u", r.sessionToken,
                done.map.generation, r.initialGrant);
        beginSession(Io::Kind::Map, r.sessionToken, es3map::sessionLimits(done.map), r.initialGrant, nullptr);
        return;
    }
    case es3proto::StartVideo: {
        es3video::StartVideoReply r;
        if (!es3video::decodeStartVideoReply(f.payload, f.length, r)) {
            negotiationFailed("malformed START_VIDEO reply");
            return;
        }
        logLine(LogLevel::DEBUG, "ES3 display: video session %08x, viewport %u,%u %ux%u, chunk %u, grant %u",
                r.sessionToken, done.video.viewportX, done.video.viewportY, done.video.viewportW,
                done.video.viewportH, unsigned(done.video.chunkBytes), r.initialGrant);
        beginSession(Io::Kind::Video, r.sessionToken, done.video, r.initialGrant, nullptr);
        return;
    }
    case es3proto::StopVideo:
    case es3map::CmdStopMap:
        onStopDone(done);
        return;
    case es3proto::Status:
        if (f.length == 9) {
            ControlResult tmp;
            tmp.status  = ControlStatus::Ok;
            tmp.command = es3proto::Status;
            tmp.length  = f.length;
            std::memcpy(tmp.payload, f.payload, f.length);
            DeviceStatus ds;
            if (parseDeviceStatus(tmp, ds)) {
                w.st.lastStatus = ds;
                w.st.haveStatus = true;
                if (ds.page != w.page) {
                    w.page = ds.page;
                    publishVideo();
                }
            }
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
    case es3map::CmdMapCapabilities:
        w.mapKnown     = true;
        w.mapSupported = false;
        logQuiet(QCaps, (code == es3proto::BadCommand || code == es3proto::NotReady) ? LogLevel::INFO : LogLevel::WARN,
                 "ES3 display: no map on this firmware (MAP_CAPABILITIES: %s) - camera preview only", errorName(code));
        publishVideo();
        return;
    case es3map::CmdStartMap:
        negotiationFailed(std::string("START_MAP refused: ") + errorName(code));
        return;
    case es3map::CmdPanCapabilities:
        w.panKnown     = true;
        w.panSupported = false;
        logQuiet(QCaps, (code == es3proto::BadCommand || code == es3proto::NotReady) ? LogLevel::INFO : LogLevel::WARN,
                 "ES3 display: no board-side panning on this firmware (PAN_CAPABILITIES: %s) - whole map frames",
                 errorName(code));
        return;
    case es3map::CmdStartPan:
        if (code == es3proto::NotReady) {   // a camera session still exists: retry as for START_MAP
            negotiationFailed(std::string("START_PAN refused: ") + errorName(code));
            return;
        }
        // Advertised but refused: use whole frames on this connection rather than retrying forever.
        w.panSupported = false;
        ++w.st.panFallbacks;
        logQuiet(QNegotiate, LogLevel::WARN, "ES3 display: START_PAN refused (%s) - whole map frames until reconnect",
                 errorName(code));
        setState(LinkState::Negotiated);
        publishVideo();
        return;
    case es3proto::StopVideo:
    case es3map::CmdStopMap:
        onStopDone(done);   // the board no longer has it either
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
        // Hidden right after being shown: the user just left that tab, so learn the
        // new page now rather than at the next 1 s poll (a faster camera/map switch).
        if (w.mapSupported && d != es3video::DemandActive) w.nextPollUs = 0;
        // Active proves which page is shown (the board makes a session active
        // only on its own tab): no need to wait for the next STATUS poll.
        if (w.mapSupported && d == es3video::DemandActive)
            w.page = w.s.kind == Io::Kind::Map ? w.mapCaps.mapPage : uint8_t(0);
        logLine(LogLevel::DEBUG, "ES3 display: %s demand %s", w.s.kind == Io::Kind::Map ? "map" : "video",
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
        // Ok acknowledges assembly once. A replay must not turn a frame
        // awaiting PRESENTED into a drop or keep the stall timer alive.
        if (e->resulted && reason == es3video::ReasonOk) return;
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
        // A background or chrome the board will not show: the producer must
        // send it again.  Superseded means a newer one of the same kind is
        // coming; Hidden/Cancelled are covered by the shown-again resend.
        if (w.s.pan && e->panKind != es3map::PanImageNone && reason != es3video::ReasonSuperseded &&
            reason != es3video::ReasonHidden && reason != es3video::ReasonCancelled)
            panImageLost(now);
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

/// A background or chrome was lost after the producer handed it over: ask it
/// for both again (at most every kPanResendGapMs, so a board that keeps
/// rejecting an image is not flooded).
void Es3DisplayLink::Impl::panImageLost(int64_t now) {
    std::lock_guard<std::mutex> lk(handoffMu);
    if (lastLossResendUs && now - lastLossResendUs < int64_t(kPanResendGapMs) * 1000) {
        lossResendDue = true;   // deferred, never dropped: checkTimers() bumps it when the gap is over
        return;
    }
    lastLossResendUs    = now;
    lossResendDue       = false;
    panDemandNow.resend = ++panResend;
}

/// The common steps after a START_VIDEO / START_MAP / START_PAN reply.
void Es3DisplayLink::Impl::beginSession(Io::Kind kind, uint32_t token, const es3video::StartVideoRequest& limits,
                                        uint32_t grant, const es3map::PanSession* pan) {
    Io& w = *io;
    endSession();   // the old token's frames can no longer complete
    {
        std::lock_guard<std::mutex> lk(handoffMu);
        lossResendDue    = false;   // a new session needs its images anyway
        lastLossResendUs = 0;
    }
    w.s.kind        = kind;
    w.s.token       = token;
    w.s.req         = limits;
    w.s.pan         = pan != nullptr;
    if (pan) w.s.panGeo = *pan;
    w.s.grant       = grant;
    w.s.nextFrameId = 1;
    w.s.visibility  = es3video::DemandActive;   // the device re-sends VISIBILITY now
    w.s.progressUs  = nowUs();
    w.resyncWanted  = false;
    ++w.st.sessions;
    if (kind == Io::Kind::Map) ++w.st.mapSessions;
    if (pan) ++w.st.panSessions;
    w.quiet[QNegotiate].clear();
    setState(LinkState::Active);
    publishVideo();
}

/// STOP_VIDEO / STOP_MAP answered (Ok or Error: either way the board has no such session).
void Es3DisplayLink::Impl::onStopDone(const Io::Control& done) {
    Io& w = *io;
    if (w.stopping) {
        w.stopDone = true;
    } else if (done.stopToken && done.stopToken == w.s.token) {   // a source switch
        endSession();
        setState(LinkState::Negotiated);
        publishVideo();
    }
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
    EncodedFrameLease dropped, droppedMap;
    {
        std::lock_guard<std::mutex> lk(handoffMu);
        if (pending) dropped = std::move(pending);
        if (mapPending) droppedMap = std::move(mapPending);
    }
    for (EncodedFrameLease* l : {&dropped, &droppedMap})
        if (*l) {
            ++w.st.framesReleased;
            l->release();
        }
    dropPanPending();
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
