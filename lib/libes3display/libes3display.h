/**
 * @file libes3display.h
 * @brief Jetson side of the ES3C28P display's USB CDC link: the single owner
 *        of its tty, for control commands (page, alert, status, heartbeat)
 *        AND the credited E3 29 live video preview, on one byte stream.
 *
 * peripherals/es3c28p/VIDEO_STREAMING_PLAN.md ("USB framing and scheduling",
 * "Frozen protocol v1", review additions 3, 4 and 6) is the design; the wire
 * contract is Es3Protocol.h + Es3VideoProtocol.h in this directory (byte-
 * identical with the firmware's copies — never edit them here).
 *
 * ── One I/O thread ────────────────────────────────────────────────────────────
 * start() launches one worker that owns the tty: it opens the configured path
 * (O_NONBLOCK, raw, TIOCEXCL, HUPCL off so a close never bounces the S3),
 * polls it together with an eventfd that offerFrame(), enqueueControl() and
 * stop() use to wake it, and never blocks in write(): partial writes and
 * EAGAIN keep the rest of the record for the next POLLOUT.  A missing or
 * vanished device is not an error — the worker retries every reconnectMs while
 * the application (and recording) carries on.  USB loss (hangup, EIO/ENODEV)
 * closes the tty, releases every queued frame and fails queued controls.
 *
 * ── Link states ───────────────────────────────────────────────────────────────
 *   Disconnected → Hello → Capabilities → Negotiated → Active ⇄ Paused
 *                                      ↘ ControlOnly
 * HELLO must be answered "ES3C28P".  CAPABILITIES answered with BadCommand or
 * NotReady (older firmware, no video memory) means "video unsupported": the
 * link stays ControlOnly until the next reconnect, never re-asking in a loop.
 * Negotiated = video supported, no session yet: the first offered frame picks
 * the viewport (centred on the canvas, as preview::computeLetterbox() places
 * it) and START_VIDEO opens a session.  Active/Paused follow the device's
 * VISIBILITY record (LIVE tab vs another tab or calibration).
 *
 * ── Control channel ───────────────────────────────────────────────────────────
 * One E3 28 request in flight at a time (the firmware answers one at a time),
 * sequence-numbered, replyTimeoutMs per reply (loopReplyTimeoutMs for the
 * commands the board's display loop answers).  A timeout or a NoSession error
 * restarts the handshake (HELLO again); the timed-out request is reported as
 * such and NEVER replayed (an alert must not play twice).  PING keeps the
 * device's 5 s session alive whenever no other request went out for
 * heartbeatMs.  Application requests go through a small bounded queue
 * (kControlQueueDepth; full = rejected, never waited for).
 *
 * ── Video scheduling ──────────────────────────────────────────────────────────
 * offerFrame() keeps at most ONE pending frame (a newer offer replaces it and
 * releases the older lease) plus the one being transmitted, and never blocks.
 * A frame goes out as START (one credit), CHUNKs of the negotiated size at
 * exact offsets, then END with the CRC-32 of the whole JPEG; its lease is
 * released as soon as the last CHUNK is encoded, so the converter gets its
 * slot back while the frame is still in flight.  Credits follow the device's
 * cumulative grant (latest value wins, never added, a lower one ignored);
 * frames started never exceed the grant, and frames started but not yet
 * answered by a RESULT never exceed min(maxFramesInFlight, device credits).
 * No credit or RESULT progress within creditStallMs while frames are
 * outstanding renegotiates the session (a "resync": new token, credits reset).
 *
 * Control has priority: between two records the worker sends a waiting
 * control request first, and before every video record it reads the kernel's
 * output queue (TIOCOUTQ) and holds the record while more than one CHUNK's
 * worth is queued — bytes already in the tty/cdc-acm buffers cannot be
 * overtaken, so a full kernel queue would otherwise delay an alert by a whole
 * frame (plan review addition 4).
 *
 * Typical usage (the app copies its <DisplayPreview> settings):
 * @code
 *   dashcam::es3display::DisplayLinkConfig dc;
 *   dc.device = "/dev/dashcam-display";
 *   dashcam::es3display::Es3DisplayLink display;
 *   display.start(dc, log);                 // false only for a bad config
 *   preview.start(pc, display, log);        // VideoPreview feeds offerFrame()
 *   display.enqueueControl(dashcam::es3display::ControlRequest::alert(80));
 *   ...
 *   preview.stop();                         // stop the producer first
 *   display.stop();
 * @endcode
 *
 * Thread safety: every public method may be called from any thread.
 * demand() is lock-free. offerFrame() takes short handoff/wake mutexes and
 * never waits for tty I/O; its eventfd write is nonblocking and serialised
 * with close. Control completion callbacks and log lines run on the I/O
 * thread: keep them short, and never call stop() from one.  start()/stop() are
 * serialised against each other; the destructor stop()s.
 */

#ifndef LIBES3DISPLAY_H
#define LIBES3DISPLAY_H

#include "Es3MapProtocol.h"
#include "Es3VideoProtocol.h"
#include "liblog.h"
#include "libnavigation.h"
#include "libvideopreview.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace dashcam::es3display {

// ─── limits ───────────────────────────────────────────────────────────────────

constexpr size_t   kControlQueueDepth = 8;     ///< Application requests waiting for the slot.
constexpr size_t   kLatencyWindow     = 128;   ///< Samples behind each p50/p95.
constexpr size_t   kPageTextBytes     = es3proto::kTextBytes;  ///< 24: 23 chars + NUL.
constexpr uint8_t  kPageCount         = 5;     ///< Live, Trip, Camera, System, Settings (firmware without the map).
constexpr uint8_t  kMaxPageCount      = es3map::kMaxPages;  ///< PAGE index ceiling; the board's page_count gates it.
constexpr uint32_t kPagePollActiveMs  = 1000;  ///< STATUS poll while a session's surface is visible.
constexpr uint32_t kPagePollIdleMs    = 250;   ///< ... while none is (the host must notice LIVE / MAP).
constexpr char     kDeviceIdentity[]  = "ES3C28P";  ///< The HELLO reply that identifies the board.

// ─── configuration ────────────────────────────────────────────────────────────

/**
 * @brief Link settings.  start() rejects values outside the documented ranges
 *        (validateConfig() says why).  The app fills the video fields from
 *        <DisplayPreview> (libconfig DisplayPreviewConfig; same names).
 */
struct DisplayLinkConfig {
    /// Exact tty to open: the udev alias or a /dev/serial/by-id path.  Never
    /// probed or discovered — the ESP32-C3 bridge has the same VID:PID.
    std::string device;
    uint32_t heartbeatMs        = 1000;  ///< PING after this long without a request, 100–4000 (device session: 5 s).
    uint32_t reconnectMs        = 1000;  ///< Reopen interval after USB loss / a failed handshake, 100–60000.
    uint32_t replyTimeoutMs     = 500;   ///< Control reply deadline, 50–4000; a miss restarts the handshake.
    /// Reply deadline for the commands the firmware hands to its Arduino loop
    /// (STATUS, PAGE, ALERT, STOP_AUDIO), 50–4000.  The board's USB task reads
    /// nothing until the loop answers, and the loop may be mid-redraw or opening
    /// a WAV; a miss here also costs the video session (HELLO again), so it is
    /// longer than replyTimeoutMs.  Kept under the device's 5 s session.
    uint32_t loopReplyTimeoutMs = 2000;
    uint16_t chunkDataBytes     = 1024;  ///< JPEG bytes per CHUNK, 64–4096; the device limit can lower it.
    uint8_t  maxFramesInFlight  = 1;     ///< Frames started without a RESULT yet, 1–2; device credits can lower it.
    uint8_t  targetFps          = 10;    ///< START_VIDEO target_fps (the device's pacing hint), 1–30.
    uint32_t maxJpegBytes       = 65536; ///< Frame cap, 1024–65536; the smaller of this and the device's applies.
    uint16_t assemblyTimeoutMs  = 500;   ///< START→END deadline, both ends, 1–10000.
    uint16_t staleAfterMs       = 500;   ///< Device marks the image stale, 1–10000.
    uint16_t placeholderAfterMs = 2000;  ///< Device shows "Video unavailable", staleAfterMs–10000.
    /// No credit/RESULT progress for this long with frames outstanding →
    /// renegotiate.  0 = max(2000, 4 × assemblyTimeoutMs); else 100–60000.
    uint32_t creditStallMs      = 0;
    /// Map surface (lib/libnavigation): ask MAP_CAPABILITIES and coordinate
    /// camera/map by the board's active page.  The fields below are the map
    /// session's START_MAP settings (<Navigation> MaxFps/StaleAfterMs/PlaceholderAfterMs).
    bool     map                   = false;
    uint8_t  mapTargetFps          = 4;      ///< 1-30.
    uint32_t mapMaxJpegBytes       = 65536;  ///< 1024-65536; the board's limit can lower it.
    uint16_t mapStaleAfterMs       = 2000;   ///< 1-10000.
    uint16_t mapPlaceholderAfterMs = 5000;   ///< mapStaleAfterMs-10000.
    /// Board-side panning (map extension v2): ask PAN_CAPABILITIES and, when the
    /// board has it, open the map as a panning session (START_PAN) fed through
    /// panSink(); otherwise (or false) whole frames through mapSink() as before.
    bool     mapPan                = true;
    uint16_t mapPanBackgroundW     = 512;    ///< 320-1024; clamped to the board's map area and maximum.
    uint16_t mapPanBackgroundH     = 384;    ///< 64-1024; likewise.
    /// Hold video records while TIOCOUTQ reports at least this many bytes.
    /// 0 = one full CHUNK record (chunk + 20 B): about one record queued at a time.
    uint32_t txQueueLimitBytes  = 0;
    bool     video              = true;  ///< false: never ask CAPABILITIES; controls only.
    bool     exclusive          = true;  ///< TIOCEXCL, so a stray terminal cannot steal bytes.
};

/// Explains the first rule @p cfg breaks in @p why (when given).  Pure.
bool validateConfig(const DisplayLinkConfig& cfg, std::string* why = nullptr);

// ─── state ────────────────────────────────────────────────────────────────────

enum class LinkState : uint8_t {
    Disconnected,  ///< No tty open (absent, unplugged, or between retries).
    Hello,         ///< tty open, HELLO in flight.
    Capabilities,  ///< HELLO answered, CAPABILITIES in flight.
    Negotiated,    ///< Video supported; no session yet (or START_VIDEO in flight).
    Active,        ///< Video session, device demand active (LIVE tab).
    Paused,        ///< Video session, device hidden or calibrating: no new frames.
    ControlOnly,   ///< Video unsupported (old firmware / no video memory) or disabled.
};
const char* linkStateName(LinkState s);

// ─── control requests ─────────────────────────────────────────────────────────

enum class ControlStatus : uint8_t {
    Ok,            ///< Reply received; payload holds its bytes.
    DeviceError,   ///< Error reply; errorCode = es3proto::ErrorCode.
    Timeout,       ///< No reply within replyTimeoutMs (the handshake restarts; not replayed).
    Disconnected,  ///< USB lost before a reply.
    Cancelled,     ///< stop() before a reply.
};
const char* controlStatusName(ControlStatus s);

/// Outcome of one application request, delivered to its callback.
struct ControlResult {
    ControlStatus status    = ControlStatus::Cancelled;
    uint8_t       command   = 0;     ///< es3proto::Type of the request.
    uint8_t       errorCode = 0;     ///< es3proto::ErrorCode when status == DeviceError.
    uint8_t       length    = 0;     ///< Reply payload bytes (status == Ok).
    uint8_t       payload[es3proto::kMaxPayload]{};
    double        latencyMs = 0.0;   ///< enqueueControl() → outcome.
};

/// Runs on the I/O thread: keep it short, never call stop() from it.
using ControlCallback = std::function<void(const ControlResult&)>;

/**
 * @brief One application command for the display.  Build it with a factory
 *        (they validate the arguments; an invalid request has command 0 and
 *        enqueueControl() rejects it).
 */
struct ControlRequest {
    uint8_t         command = 0;     ///< es3proto::Type; 0 = invalid.
    uint8_t         length  = 0;
    uint8_t         payload[es3proto::kMaxPayload]{};
    ControlCallback onDone;          ///< Optional; called exactly once for an accepted request.

    bool valid() const { return command != 0; }

    static ControlRequest ping();
    static ControlRequest status();                       ///< Reply: see parseDeviceStatus().
    static ControlRequest stopAudio();
    static ControlRequest alert(uint8_t volume);          ///< volume 0–100; plays /alert.wav.
    /// @p index < kMaxPageCount (the link refuses an index the board does not
    /// have: 0-4, or 0-5 with the MAP tab); title and lines printable ASCII, at most 23 chars.
    static ControlRequest page(uint8_t index, const std::string& title,
                               const std::array<std::string, 4>& lines);
};

/// STATUS reply contents.
struct DeviceStatus {
    uint32_t uptimeMs     = 0;
    uint8_t  page         = 0;      ///< Selected tab, 0 = Live.
    uint8_t  brightness   = 0;
    bool     sdCard       = false;
    bool     calibrated   = false;
    bool     audioPlaying = false;
};
/// Decodes a STATUS ControlResult (status Ok, 9-byte payload).
bool parseDeviceStatus(const ControlResult& r, DeviceStatus& out);

// ─── statistics ───────────────────────────────────────────────────────────────

/// Last value and p50/p95 over the newest kLatencyWindow samples (0 = none yet).
struct LatencySummary {
    double   lastMs  = 0.0;
    double   p50Ms   = 0.0;
    double   p95Ms   = 0.0;
    uint32_t samples = 0;
};

/// Snapshot of the link (counters cumulative since start()).
struct DisplayLinkStats {
    LinkState   state            = LinkState::Disconnected;
    bool        connected        = false;  ///< HELLO answered on the open tty.
    bool        videoSupported   = false;
    std::string device;                    ///< Configured path.
    std::string identity;                  ///< HELLO reply of the last handshake.
    bool        haveCapabilities = false;
    es3video::Capabilities capabilities;   ///< As advertised (valid when haveCapabilities).

    // Current session (zero without one).
    uint32_t                   sessionToken    = 0;
    dashcam::preview::Viewport viewport;
    uint16_t                   chunkBytes      = 0;  ///< Negotiated CHUNK data size.
    uint32_t                   maxJpegBytes    = 0;  ///< min(config, device).
    uint8_t                    creditWindow    = 0;  ///< min(maxFramesInFlight, device credits).
    uint32_t                   grant           = 0;  ///< Latest cumulative grant.
    uint32_t                   sessionStarted  = 0;  ///< STARTs sent in this session.
    uint32_t                   sessionResults  = 0;  ///< ... of which have their first RESULT.
    uint8_t                    visibility      = es3video::DemandActive;

    // Connection.
    uint64_t opens = 0, openFailures = 0;
    uint64_t connects        = 0;  ///< First successful HELLO after an open.
    uint64_t disconnects     = 0;  ///< tty closures after an open.
    uint64_t rehellos        = 0;  ///< HELLO again on the same tty (timeout / NoSession).
    uint64_t helloTimeouts   = 0;
    uint64_t helloRejected   = 0;  ///< Wrong identity: neither ES3C28P nor CYD.
    uint64_t noSessionErrors = 0;

    // Control.
    uint64_t controlEnqueued = 0, controlRejected = 0, controlQueueFull = 0;
    uint64_t controlSent       = 0;  ///< All requests written, including internal ones.
    uint64_t controlReplies    = 0;  ///< Replies matched to the request in flight (incl. errors).
    uint64_t controlErrors     = 0;  ///< ... that were Error replies.
    uint64_t controlTimeouts   = 0;
    uint64_t pingsSent         = 0;
    uint64_t unexpectedReplies = 0;  ///< Wrong sequence / type, or nothing in flight.

    // Video sessions.
    uint64_t sessions = 0, negotiationFailures = 0, resyncs = 0, stopVideoSent = 0;

    // Frames.  offered = accepted + refused; accepted ones end as replaced
    // (by a newer offer), released (hidden / USB loss / stop before START) or
    // started.  A started frame ends as presented, dropped (a rejecting
    // RESULT, or the second RESULT after Ok) or lost (the session ended first),
    // or is still outstanding.
    uint64_t framesOffered = 0, framesAccepted = 0, framesRefused = 0, framesReplaced = 0;
    uint64_t framesReleased  = 0;
    uint64_t framesStarted   = 0;
    uint64_t framesSent      = 0;  ///< END written.
    uint64_t framesCompleted = 0;  ///< First RESULT Ok (assembled by the device).
    uint64_t framesPresented = 0;
    uint64_t framesDropped   = 0;
    uint64_t framesLost      = 0;
    uint64_t framesAbandoned = 0;  ///< Cut short by the host (deadline, renegotiation, stop); also dropped/lost.
    std::array<uint64_t, es3video::kReasonCount> resultReasons{};  ///< Every RESULT, by reason.

    // Transport.
    uint64_t bytesSent = 0, bytesReceived = 0, videoBytesSent = 0, videoRecordsSent = 0;
    uint64_t creditStalls = 0;        ///< Frames that had to wait for a credit / window slot.
    uint64_t kernelQueueWaits = 0;    ///< Video records held for TIOCOUTQ.
    uint32_t kernelQueueLimit = 0;    ///< Bytes (0 = not connected).
    uint32_t kernelQueueMax   = 0;    ///< Largest TIOCOUTQ seen before a video record.
    uint64_t staleRecords = 0;        ///< Device records with another session's token (ignored).
    uint64_t lowerGrantIgnored = 0, unknownFrameRecords = 0, malformedRecords = 0, unexpectedRecords = 0;
    uint64_t parserGarbageBytes = 0, parserCrcErrors = 0, parserHeaderErrors = 0, parserTimeouts = 0;

    // Timing (host steady clock).
    LatencySummary presentLatency;   ///< START written → PRESENTED received.
    LatencySummary captureLatency;   ///< Lease capture time → PRESENTED (an upper bound incl. the ACK).
    LatencySummary controlLatency;   ///< enqueueControl() → Ok reply.
    uint32_t       lastDecodeUs = 0, lastBlitUs = 0;  ///< From the last PRESENTED.

    // Map surface (Es3MapProtocol.h).
    bool                    mapSupported = false;   ///< MAP_CAPABILITIES answered with a valid set.
    es3map::MapCapabilities mapCapabilities;
    uint8_t                 activePage   = 0xff;    ///< From the last STATUS; 0xff = not known yet.
    uint8_t                 sessionKind  = 0;       ///< 0 none, 1 video, 2 map.
    uint64_t mapSessions = 0, sourceSwitches = 0, pagePolls = 0;
    uint64_t mapFramesOffered = 0, mapFramesAccepted = 0;

    // Board-side panning (map extension v2).
    bool                    panSupported = false;   ///< PAN_CAPABILITIES answered with a valid set.
    es3map::PanCapabilities panCapabilities;
    bool                    panSession   = false;   ///< The current map session is a panning one.
    uint16_t                panBackgroundW = 0, panBackgroundH = 0;   ///< Negotiated (0 = none).
    uint64_t panSessions = 0, panFallbacks = 0;     ///< Fallbacks: START_PAN refused -> whole frames.
    uint64_t panBackgroundsOffered = 0, panBackgroundsAccepted = 0;
    uint64_t panChromeOffered = 0, panChromeAccepted = 0;
    uint64_t panBackgroundsStarted = 0, panChromeStarted = 0;
    uint64_t posesOffered = 0, posesAccepted = 0, posesSent = 0;
    uint64_t posesReplaced = 0;                     ///< Overwritten by a newer pose before sending.
    bool           haveStatus = false;
    DeviceStatus   lastStatus;       ///< Last STATUS reply.
};

// ─── the link ─────────────────────────────────────────────────────────────────

/**
 * @brief Owner of the ES3C28P tty; also the preview converter's sink.
 *
 * All methods are safe on a never-started or stopped instance.
 */
class Es3DisplayLink : public dashcam::preview::IPreviewSink {
public:
    Es3DisplayLink();
    ~Es3DisplayLink() override;
    Es3DisplayLink(const Es3DisplayLink&)            = delete;
    Es3DisplayLink& operator=(const Es3DisplayLink&) = delete;

    /**
     * @brief Validate @p cfg and start the I/O worker.
     * @return false only for an invalid config, a second start(), or an
     *         eventfd/thread failure (logged).  A missing device is NOT a
     *         failure: the worker keeps retrying it every reconnectMs.
     */
    bool start(const DisplayLinkConfig& cfg, dashcam::log::LogCallback log = {});

    /**
     * @brief Stop accepting work, end the video session (best-effort
     *        STOP_VIDEO, bounded to ~300 ms), close the tty, join the worker,
     *        release every lease and fail the queued controls (Cancelled).
     *        Idempotent; bounded (well under a second); safe after a failed
     *        start().
     */
    void stop();

    bool running() const;

    /// What the preview converter should produce now (atomic snapshot).
    dashcam::preview::PreviewDemand demand() const override;

    /**
     * @brief Hand over one encoded frame.  Accepted (true, lease taken) only
     *        when video is supported, the link is connected, the device is
     *        visible, and the frame fits the device (size, dimensions).  A
     *        newer frame replaces a still-pending one (that lease is released
     *        at once).  Never blocks; on false the caller keeps the lease.
     */
    bool offerFrame(dashcam::preview::EncodedFrameLease&& frame) override;

    /**
     * @brief The MAP surface's sink, for libnavigation's NavigationService.
     *
     * demand(): canvas = the board's map viewport (320x224 on the ES3C28P),
     * maxJpegBytes = min(config, board); active only while a map session
     * exists and the board shows the MAP tab.  offerFrame(): exactly the map
     * viewport size, else refused.  With cfg.map false or firmware without
     * the extension, demand() stays inactive with a zero canvas.
     *
     * Source coordination (inside the link): with a map-capable board the link
     * polls STATUS for the active page (kPagePollIdleMs while nothing is shown,
     * kPagePollActiveMs otherwise) and binds the board's single image receiver
     * to the camera on LIVE (page 0) and to the map on the map page: STOP_VIDEO
     * then START_MAP, or STOP_MAP then START_VIDEO.  Other pages keep the
     * current session, paused by the board's VISIBILITY.  The camera preview's
     * demand() is inactive whenever the map owns the receiver, so the converter
     * does no work (the recorder tap's frames are refused before any copy).
     */
    dashcam::preview::IPreviewSink& mapSink();

    /**
     * @brief The panning map surface (map extension v2), for libnavigation.
     *
     * panDemand() is active only while a panning session exists and the board
     * shows the MAP tab; its session value changes with every new session (the
     * board then has no background or chrome yet).  The link prefers a panning
     * session whenever the board answers PAN_CAPABILITIES (cfg.mapPan); while
     * one exists, mapSink()'s demand stays inactive, and vice versa.
     * Backgrounds and chrome each keep one pending image (newest wins), sent
     * as credited E3 29 frames, chrome first; poses keep one pending value and
     * go out as MAP_POSE records between any two records, at most maxPoseHz.
     */
    dashcam::navigation::IMapPanSink& panSink();

    /**
     * @brief Queue an application request (page / alert / stop audio / status
     *        / ping).  False — and the callback is never called — when the
     *        request is invalid, the link is stopped or has no open tty, or the
     *        queue is full.  Accepted requests are sent in order, between video
     *        records, and complete exactly once.
     */
    bool enqueueControl(const ControlRequest& request);

    DisplayLinkStats stats() const;
    LinkState        state() const;

    struct Impl;   ///< Shared state and the I/O worker (libes3display.cpp).

private:
    std::unique_ptr<Impl> impl_;
};

} // namespace dashcam::es3display

#endif // LIBES3DISPLAY_H
