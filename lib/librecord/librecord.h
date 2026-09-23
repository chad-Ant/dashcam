/**
 * @file librecord.h
 * @brief UVC compressed-passthrough recorder with an ASS telemetry sidecar.
 *
 * Records the ALREADY-COMPRESSED stream of a UVC camera (MJPEG or H.264)
 * straight into a Matroska file — no decode, no re-encode.  On Orin Nano
 * (which has no NVENC) this removes the software x264 stage entirely; the CPU
 * cost of recording drops to container muxing.
 *
 * Telemetry is no longer burned into the video.  Instead the Recorder writes
 * an Advanced SubStation Alpha (.ass) sidecar with the SAME name as the
 * recording (clip.mkv → clip.ass) containing the same four-corner layout the
 * old Cairo overlay drew:
 *   - Top-left:     speed, acceleration
 *   - Top-right:    heading + cardinal direction
 *   - Bottom-left:  latitude, longitude, altitude
 *   - Bottom-right: local date and time (host timezone)
 * Corner positions, box opacity and padding come from OverlayConfig; the font
 * size scales with the video resolution (OverlayConfig::fontSize is the size
 * at 720p, PlayResY tracks the actual height).  Players (mpv, VLC, ffmpeg)
 * auto-load the sidecar; `ffmpeg -i clip.mkv -vf ass=clip.ass` burns it in
 * for export.
 *
 * The Recorder owns its own small GStreamer pipeline:
 * @verbatim
 *   v4l2src device=/dev/videoN
 *     ! image/jpeg,width,height,framerate     (or video/x-h264 ! h264parse)
 *     [! videorate drop-only=true max-rate=N]  (MJPEG only: frame-drop cap)
 *     ! queue ! matroskamux ! filesink
 * @endverbatim
 * so recording no longer attaches to a Camera_GST tee — do NOT open the same
 * device with Camera_USB while recording (V4L2 devices are exclusive).
 * CSI/Argus sources are deliberately unsupported: they only produce raw
 * frames, and raw recording would require the software encoder this design
 * removes.  startRecording() rejects any non-compressed pixel format.
 *
 * Typical usage:
 * @code
 *   dashcam::record::Recorder rec;
 *   rec.setLogCallback(log);
 *   rec.setOverlayConfig(cfg.overlay);        // subtitle style + cadence
 *
 *   dashcam::record::RecordingFormat fmt;     // from cameraInfo::videoFormats
 *   fmt.v4l2PixFmt = V4L2_PIX_FMT_MJPEG;
 *   fmt.width = 1280; fmt.height = 720; fmt.fps = 30.0f;
 *
 *   rec.startRecording("/dev/video2", fmt, "clip.mkv");
 *   while (running) {
 *       // Set the per-source validity flags and stamps EXPLICITLY.  They
 *       // default false so an application that never sets them renders dashes
 *       // rather than the struct's placeholder coordinates — which is the
 *       // right default, but it does mean a brace initialiser of the seven
 *       // numeric fields produces an overlay with no telemetry in it.
 *       dashcam::record::OverlayData od;
 *       od.latitude = lat; od.longitude = lon; od.altitudeM = alt;
 *       od.speedKmh = spd; od.accelerationMs2 = acc; od.headingDeg = hdg;
 *       od.timestampMs = epochMs();
 *       od.positionValid = fixIsGood;   od.positionTimestampMs = fixMs;
 *       od.headingValid  = courseIsGood; od.headingTimestampMs = fixMs;
 *       od.speedValid    = speedIsLive;  od.speedTimestampMs    = speedMs;
 *       od.accelValid    = ecuIsLive;    od.accelTimestampMs    = ecuMs;
 *       rec.setOverlayData(od);
 *   }
 *   rec.stopRecording();                      // EOS-finalises clip.mkv + clip.ass
 * @endcode
 *
 * @note Requires GStreamer ≥ 1.20.  No Cairo, no NVIDIA elements.
 */

#ifndef LIBRECORD_H
#define LIBRECORD_H

#include "libconfig.h"
#include "liblog.h"
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <atomic>
#include <cstdint>
#include <fstream>
#include <functional>
#include <deque>
#include <mutex>
#include <optional>
#include <ostream>
#include <set>
#include <string>
#include <string_view>
#include <thread>

namespace dashcam::record {

// ─── telemetry payload ────────────────────────────────────────────────────────

/**
 * @brief Telemetry snapshot logged to the ASS sidecar at SubtitleRateHz.
 *
 * Written by the application thread via Recorder::setOverlayData(); sampled by
 * the Recorder's subtitle thread.  The struct is copied under a mutex on each
 * write/read, so setOverlayData() is safe from any thread at any time.
 */
struct OverlayData {
    double  latitude        = 10.7725;    ///< WGS-84 latitude in decimal degrees.
    double  longitude       = 106.6581;   ///< WGS-84 longitude in decimal degrees.
    double  altitudeM       = 52.3;       ///< Altitude above mean sea level in metres.
    float   speedKmh        = 1.5f;       ///< Ground speed in kilometres per hour.
    float   accelerationMs2 = 0.0f;       ///< Longitudinal acceleration in m/s² (+ve = forward).
    float   headingDeg      = 90.0f;      ///< True heading in degrees (0 = North, clockwise).
    int64_t timestampMs     = 1777633580000LL; ///< UNIX epoch timestamp in milliseconds.

    // ── per-source validity ───────────────────────────────────────────────────
    // THREE domains, not one, because three sources fail independently:
    //   speed        — GNSS ground speed, else the ECU wheel speed
    //   acceleration — derived on the master from ECU speed only
    //   position     — GNSS fix (lat/lon/alt/heading together)
    //
    // Grouping speed and acceleration under one "motion" flag was not enough:
    // a live GNSS fix refreshes speed while a dead ECU leaves acceleration at
    // whatever the previous frame held, and one shared flag then certified both.
    // The stale half is the dangerous half — an inherited acceleration rendered
    // as current is fabricated evidence, and it is wrong by an amount too small
    // to look wrong.
    //
    // Position stays a single domain because its four fields genuinely do come
    // from one source and fail together.
    //
    // Same reasoning, and the same shape, as laneValid / driverValid below.
    // An invalid domain renders as dashes, never as a stale reading.
    bool    speedValid    = false;  ///< speedKmh is backed by a live source.
    bool    accelValid    = false;  ///< accelerationMs2 is backed by a live ECU.
    bool    positionValid = false;  ///< lat / lon / alt are backed by a valid fix.
    /**
     * headingDeg is backed by a course the master judged trustworthy.
     *
     * Separate from @c positionValid even though both come from the GNSS
     * receiver, because they do NOT fail together.  Below roughly 5 km/h a
     * receiver reports a course that wanders the full circle, so the master
     * sends NaN — while the fix itself stays perfectly good.  Folding heading
     * into positionValid therefore certified whatever heading was last held (or
     * the struct's 90.0f placeholder) as live every time the vehicle stopped.
     */
    bool    headingValid  = false;
    int64_t speedTimestampMs    = 0; ///< Epoch ms speedKmh last refreshed.
    int64_t accelTimestampMs    = 0; ///< Epoch ms accelerationMs2 last refreshed.
    int64_t positionTimestampMs = 0; ///< Epoch ms lat/lon/alt last refreshed.
    int64_t headingTimestampMs  = 0; ///< Epoch ms headingDeg last refreshed.

    // ── ADAS telemetry (computed onboard; no GPS/IMU needed) ──────────────────
    // When adasValid is false the ADAS overlay banner and telemetry are omitted.
    // These are deliberately plain scalars so librecord stays decoupled from
    // liblanedetector / libdriverstate — the application copies the results in.
    //
    // adasValid gates the banner as a whole; laneValid / driverValid gate the two
    // halves INDEPENDENTLY.  The lane and driver detectors warm up and fail
    // separately, so their validity must not be conflated: an invalid source
    // renders as a dash rather than a stale or fabricated reading.  The
    // application rebuilds these fields fresh each tick (never inherited).
    bool  adasValid       = false;   ///< false → ADAS overlay banner suppressed entirely.
    bool  laneValid       = false;   ///< A fresh lane result backs the lane fields this sample.
    bool  driverValid     = false;   ///< A fresh driver result backs the fatigue/face fields this sample.
    int   laneCount       = 0;       ///< Lanes detected (LaneResult::numLanes).
    int   egoLaneIndex    = -1;      ///< 0-based ego lane; -1 = off-road / unknown.
    float laneOffset      = 0.0f;    ///< Lateral offset within the lane, -1 (left line) .. +1 (right line).
    bool  laneOffsetValid = false;   ///< laneOffset is meaningful this sample.
    float fatigueScore    = 100.0f;  ///< Fatigue score: 100 fresh → ≤0 fatigued.
    int   fatigueLevel    = 0;       ///< 0 OK, 1 CAUTION, 2 WARNING, 3 FATIGUE.
    bool  driverDrowsy    = false;   ///< Most recent classification is drowsy.
    bool  faceDetected    = true;    ///< Driver's face currently visible to the cabin cam.
};

// ─── recording format ─────────────────────────────────────────────────────────

/**
 * @brief The UVC stream to record — copy the fields from the chosen
 *        cameraInfo::videoFormats entry.
 *
 * Only compressed formats are accepted: V4L2_PIX_FMT_MJPEG or
 * V4L2_PIX_FMT_H264.  Raw formats (YUYV & friends) are rejected by
 * startRecording() — recording them would need a software encoder.
 */
struct RecordingFormat {
    uint32_t v4l2PixFmt = 0;   ///< V4L2 fourcc (V4L2_PIX_FMT_MJPEG / _H264).
    uint32_t width      = 0;   ///< Frame width in pixels.
    uint32_t height     = 0;   ///< Frame height in pixels.
    float    fps        = 0.0f;///< Camera frame rate for this format.
};

// ─── live-stream tap ───────────────────────────────────────────────────────────

/**
 * @brief Delivers each ALREADY-COMPRESSED frame as it is captured, for live
 *        network streaming (see MediaStreamServer in libnetwork).
 *
 * @param data      Frame bytes: one JPEG (MJPEG) or one H.264 access unit
 *                  (Annex-B byte-stream) — valid ONLY for the duration of the
 *                  call, so copy anything you retain.
 * @param len       Byte count.
 * @param keyframe  true for an intra frame (always true for MJPEG; the IDR flag
 *                  for H.264) — a new viewer should ideally start here.
 *
 * Invoked on a GStreamer streaming thread.  Keep it fast and non-blocking (hand
 * the bytes to a stream server / queue); the tap branch is leaky, so a slow
 * consumer drops its own frames and never backpressures the recording.  Set the
 * callback BEFORE startRecording() — it selects whether the tee/appsink branch
 * is built into the pipeline at all.
 */
using CompressedFrameCallback =
    std::function<void(const uint8_t* data, size_t len, bool keyframe)>;

// ─── Recorder ─────────────────────────────────────────────────────────────────

/**
 * @brief One recording session at a time: compressed UVC video to MKV plus an
 *        ASS telemetry sidecar.
 *
 * Thread safety:
 *   - setOverlayData()/getOverlayData() are safe from any thread.
 *   - setOverlayConfig()/setLogCallback() must be called before
 *     startRecording() (they configure the session).
 *   - startRecording()/stopRecording() are not re-entrant; call them from the
 *     application's control thread.
 */
class Recorder {
public:
    Recorder();

    /** @brief Destructor; stops any active recording (EOS-finalised). */
    ~Recorder();

    Recorder(const Recorder&)            = delete;
    Recorder& operator=(const Recorder&) = delete;

    // ─── telemetry ─────────────────────────────────────────────────────────────

    /** @brief Replace the current telemetry snapshot.  Thread-safe. */
    void setOverlayData(const OverlayData& data);

    /** @brief Return a copy of the current telemetry snapshot.  Thread-safe. */
    OverlayData getOverlayData() const;

    /**
     * @brief Set the subtitle style + cadence configuration.
     *
     * Call before startRecording(); the ASS header is written at start.
     * OverlayConfig::enabled=false suppresses the sidecar entirely.
     */
    void setOverlayConfig(const dashcam::config::OverlayConfig& cfg);

    /** @brief Inject a log callback.  Defaults to silent. */
    void setLogCallback(dashcam::log::LogCallback cb);

    /**
     * @brief Tap the compressed stream for live network streaming.
     *
     * Call BEFORE startRecording().  When set, the recording pipeline gains a
     * tee with a leaky appsink branch that hands every compressed frame to @p cb
     * (see CompressedFrameCallback); the recording branch is unaffected.  Pass an
     * empty callback (the default) to record with no streaming tap.
     */
    void setCompressedFrameCallback(CompressedFrameCallback cb);

    // ─── recording lifecycle ───────────────────────────────────────────────────

    /**
     * @brief Start recording a UVC camera's compressed stream.
     *
     * @param devicePath  V4L2 device node (e.g. "/dev/video2").
     * @param fmt         Compressed format to capture (MJPEG or H264 only —
     *                    anything else is rejected with an ERROR log).
     * @param filename    Output MKV path; the ASS sidecar is written next to
     *                    it with the extension replaced (clip.mkv → clip.ass).
     * @param maxFps      MJPEG only: cap the recorded rate by dropping frames
     *                    (videorate drop-only) — dropping intra-only JPEG
     *                    frames is lossless-safe.  0 = record at camera rate.
     *                    Ignored for H264 (dropping would corrupt GOPs).
     * @param eosTimeoutMs  How long stopRecording() waits for the EOS to
     *                    flush before forcing teardown.
     * @return true when the pipeline reached PLAYING; false on any failure
     *         (bad format, busy device, negotiation error) — details logged.
     */
    bool startRecording(const std::string& devicePath,
                        const RecordingFormat& fmt,
                        const std::string& filename,
                        uint32_t maxFps = 0,
                        uint32_t eosTimeoutMs = 4000);

    /**
     * @brief Stop the session: EOS-finalise the MKV, close the ASS sidecar.
     *
     * Safe to call when not recording (no-op).  After it returns the files
     * are complete and playable.
     */
    void stopRecording();

    /**
     * @brief True while a started session is healthy (pipeline running, no
     *        bus error observed).  A bus error is logged and latches false.
     */
    bool isRecording() const;

private:
    // ── configuration / telemetry state ──────────────────────────────────────
    dashcam::log::LogCallback      log_{};
    CompressedFrameCallback        frameCb_{};   ///< Live-stream tap; empty = no tap.
    OverlayData                    overlayData_;
    dashcam::config::OverlayConfig overlayConfig_;
    mutable std::mutex             overlayMutex_;

    /// appsink new-sample handler: maps the buffer and forwards it to frameCb_.
    static GstFlowReturn onNewSample(GstAppSink* sink, gpointer user);

    // ── session state ────────────────────────────────────────────────────────
    GstElement*       pipeline_ = nullptr;
    std::ofstream     assFile_;
    std::string       assPath_;
    uint32_t          videoW_ = 0, videoH_ = 0;
    uint32_t          eosTimeoutMs_ = 4000;
    std::thread       subThread_;
    std::atomic<bool> stopFlag_{false};
    std::atomic<bool> healthy_{false};

    /// Subtitle/bus thread: samples telemetry at SubtitleRateHz, appends ASS
    /// Dialogue events timed by pipeline position, and polls the bus for errors.
    void subtitleLoop();

    /// Write the ASS header ([Script Info] + four corner styles) for videoW/H.
    void writeAssHeader();

    /// Append one telemetry sample (4 Dialogue events) at video time @p posNs.
    /// The bottom-right clock is drawn from @p wallNowMs (the device clock, so
    /// it stays live regardless of telemetry age).
    ///
    /// Staleness is per SOURCE, and there are FOUR because four things fail
    /// independently: @p speedStale dashes SPD, @p accelStale dashes ACC,
    /// @p positionStale dashes LAT/LON/ALT, @p headingStale dashes HDG.
    ///
    /// Speed and acceleration split because a GNSS fix can supply speed while a
    /// dead ECU leaves acceleration stale.  Heading splits from position because
    /// a stationary vehicle has a good fix and no trustworthy course.  Any flag
    /// covering two of these necessarily lies about one of them, and a frozen
    /// reading rendered as current is fabricated evidence.
    void writeAssSample(int64_t posNs, int64_t durNs, const OverlayData& od,
                        int64_t wallNowMs, bool speedStale, bool accelStale,
                        bool positionStale, bool headingStale);
};

// ─── shared internals (librecord.cpp) ─────────────────────────────────────────

namespace detail {
/// Quote a GStreamer string property so external paths cannot inject elements.
std::string gstQuoted(const std::string& value);
/// ASS header ([Script Info] + corner/ADAS styles) for a videoW×videoH stream.
void writeAssHeader(std::ostream& out, const dashcam::config::OverlayConfig& cfg,
                    uint32_t videoW, uint32_t videoH);
/// One telemetry sample (corner Dialogue events) at video time @p posNs; each
/// stale flag dashes its own field (see Recorder::writeAssSample).
void writeAssSample(std::ostream& out, int64_t posNs, int64_t durNs,
                    const OverlayData& od, int64_t wallNowMs, bool speedStale,
                    bool accelStale, bool positionStale, bool headingStale);
/// Per-source staleness of one telemetry snapshot.
struct AssStaleness {
    bool speed = true, accel = true, position = true, heading = true;
};
/// Judge each source: invalid flag, non-finite / out-of-range value, or older
/// than @p staleMs (0 = no age limit) → stale.
AssStaleness assStaleness(const OverlayData& od, int64_t nowMs, int64_t staleMs);
} // namespace detail

// ─── segment naming (libsegment.cpp) ──────────────────────────────────────────
//
// Segments are named <prefix>_<SEQ>_<YYYYMMDD_HHMMSS>.mkv (+ .ass sidecar).
// SEQ is a per-directory sequence (zero-padded to 6 digits, grows beyond) and
// is the ONLY ordering used for loop overwrite: the wall-clock part is for
// humans, and this Jetson can boot with an unset clock.

/**
 * @brief Parse an owned segment file name.
 * @return SEQ when @p name is exactly <prefix>_<6..12 digits>_<8 digits>_<6
 *         digits>.mkv|.ass; std::nullopt for anything else (a foreign file).
 *         @p isAss (optional) reports the extension.  Never throws.
 */
std::optional<uint64_t> parseSegmentName(std::string_view name, std::string_view prefix,
                                         bool* isAss = nullptr);

/// Build <prefix>_<SEQ>_<YYYYMMDD_HHMMSS>.mkv from @p wallSec (local time).
std::string segmentFileName(const std::string& prefix, uint64_t seq, time_t wallSec);

/// Next free SEQ in @p dir: 1 + the highest owned SEQ (1 when none / unreadable).
uint64_t nextSegmentSeq(const std::string& dir, const std::string& prefix);

// ─── SegmentedRecorder (libsegment.cpp) ───────────────────────────────────────

/// Session parameters for SegmentedRecorder::start().
struct SegmentOptions {
    std::string dir;                     ///< Output directory (must exist).
    std::string prefix = "dashcam";      ///< File-name prefix (see parseSegmentName).
    uint32_t segmentSec          = 180;  ///< Segment length; split lands on a keyframe.
    uint32_t maxFps              = 0;    ///< MJPEG drop-only rate cap; 0 = camera rate.
    uint32_t eosTimeoutMs        = 4000; ///< stop(): wait this long for the EOS to finalise.
    uint32_t stallTimeoutMs      = 5000; ///< Unhealthy after this long without a frame; 0 = off.
    uint32_t firstFrameTimeoutMs = 15000;///< Unhealthy when no first frame arrives in time.
    /// stop() teardown still stuck eosTimeoutMs + 3 s after it began (a D-state
    /// write or a wedged V4L2 ioctl): log FATAL and _exit(3) so the outer restart
    /// loop recovers.  Tests turn it off.
    bool exitOnTeardownHang = true;
};

/**
 * @brief Records a UVC camera's compressed stream into gapless, individually
 *        playable MKV segments, each with its own ASS telemetry sidecar.
 *
 * @verbatim
 *   v4l2src ! caps [! videorate drop-only (MJPEG)] [! h264parse]
 *     ! queue name=recq ! splitmuxsink (matroskamux, async-finalize)
 * @endverbatim
 * Every segment starts at t=0 and at a keyframe.  A pad probe on the queue's
 * sink pad (upstream of splitmuxsink's GOP gate) feeds the stall watchdog and
 * drops an EOS that arrives before the first buffer (splitmuxsink 1.20
 * g_assert-aborts the whole process on that).  Sidecar events are timed from
 * the pipeline clock and replayed into each new .ass from a short ring, so the
 * first GOP of every segment is covered even though splitmuxsink announces a
 * new fragment only after that GOP completes.
 *
 * Thread safety: start()/stop() from one control thread; everything else
 * (overlay data, health, watermark, fragment flag) is safe from any thread.
 */
class SegmentedRecorder {
public:
    SegmentedRecorder();
    ~SegmentedRecorder();   ///< stop()s any active session.

    SegmentedRecorder(const SegmentedRecorder&)            = delete;
    SegmentedRecorder& operator=(const SegmentedRecorder&) = delete;

    void        setOverlayData(const OverlayData& data);
    OverlayData getOverlayData() const;
    void        setOverlayConfig(const dashcam::config::OverlayConfig& cfg); ///< Before start().
    void        setLogCallback(dashcam::log::LogCallback cb);                ///< Before start().

    /**
     * @brief Start a session.  Segment SEQs continue after the highest owned
     *        SEQ already in opts.dir.
     * @return true once the pipeline is PLAYING; false on any failure, with
     *         the reason (including the GStreamer bus error) in lastError().
     */
    bool start(const std::string& devicePath, const RecordingFormat& fmt,
               const SegmentOptions& opts);

    /**
     * @brief End the session: EOS-finalise the open segment (also after a
     *        mid-stream source error) and close its sidecar.  No-op when idle.
     * @return false when the finalise wait timed out (the file may lack its
     *         index; it is still recoverable).
     */
    bool stop();

    /// A session exists (start() succeeded and stop() has not run).
    bool isActive() const { return pipeline_ != nullptr; }

    /// Active and healthy: no bus error, no unsolicited EOS, frames flowing
    /// (first frame within firstFrameTimeoutMs, then no gap > stallTimeoutMs).
    bool isRecording() const;

    /// Why the session went unhealthy / failed to start ("" when fine).
    std::string lastError() const;

    /// Retention watermark: owned segments with SEQ below this are closed and
    /// safe to delete.  UINT64_MAX when no session is active.
    uint64_t deletableBelowSeq() const;

    /// True once per segment close since the last call (triggers retention).
    bool consumeFragmentClosed();

    std::string currentFile() const;   ///< Segment being written ("" before the first).
    uint64_t    framesReceived() const { return bufferCount_.load(); }

private:
    friend struct RecorderTestHook;

    struct Sample {                     ///< One ring entry for sidecar replay.
        int64_t              rtNs;
        OverlayData          od;
        int64_t              wallMs;
        detail::AssStaleness stale;
    };

    // configuration / telemetry
    dashcam::log::LogCallback      log_{};
    OverlayData                    overlayData_;
    dashcam::config::OverlayConfig overlayConfig_;
    mutable std::mutex             overlayMutex_;
    std::string                    testSourceDesc_;   ///< Test hook: replaces v4l2src+caps.

    // session
    GstElement*          pipeline_ = nullptr;
    GstElement*          recq_     = nullptr;   ///< Owned ref to the "recq" queue.
    SegmentOptions       opts_;
    uint32_t             videoW_ = 0, videoH_ = 0;
    std::thread          worker_;
    std::atomic<bool>    stopFlag_{false};
    std::atomic<bool>    healthy_{false};
    std::atomic<bool>    eosSeen_{false};
    std::atomic<bool>    errorSeen_{false};
    std::atomic<uint64_t> bufferCount_{0};
    std::atomic<int64_t> lastBufferNs_{0};      ///< steady_clock ns of the newest frame.
    int64_t              playingAtNs_ = 0;      ///< steady_clock ns when PLAYING was requested.
    std::atomic<bool>    fragmentClosed_{false};

    mutable std::mutex   stateMutex_;           ///< Guards the fields below.
    std::string          lastError_;
    std::string          currentFile_;
    std::set<uint64_t>   openSeqs_;             ///< Assigned, not yet confirmed closed.
    uint64_t             sessionStartSeq_ = 0;
    uint64_t             nextSeq_         = 0;  ///< Next SEQ format-location will assign.

    // sidecar (worker thread, then the stop() thread after the worker joined)
    bool                 assEnabled_ = false;
    std::ofstream        assFile_;
    std::string          assPath_;
    int64_t              fragStartRtNs_ = -1;   ///< Running time where the open .ass starts.
    std::deque<Sample>   ring_;

    void workerLoop();
    void handleMessage(GstMessage* msg);
    void onFragmentOpened(const std::string& location, int64_t rtNs);
    void onFragmentClosed(const std::string& location);
    void checkWatchdog();
    void takeSample(int64_t durNs, int64_t staleMs);
    int64_t runningTimeNs() const;               ///< Pipeline clock − base time; -1 if unknown.
    void setError(const std::string& why);
    void closeSidecar();
    void teardown();                             ///< EOS/finalise + NULL; stop()'s body.

    static gchar*            onFormatLocation(GstElement* smx, guint fragmentId,
                                              GstSample* first, gpointer self);
    static GstPadProbeReturn onRecqProbe(GstPad* pad, GstPadProbeInfo* info, gpointer self);
};

// ─── loop overwrite (libretention.cpp) ────────────────────────────────────────

/// What enforceRetention() may delete and when.
struct RetentionPolicy {
    std::string dir;                  ///< Footage directory (scanned non-recursively).
    std::string prefix = "dashcam";   ///< Only names parseSegmentName() accepts are touched.
    uint64_t    maxBytes     = 0;     ///< Quota on owned segment bytes; 0 = no quota.
    uint64_t    minFreeBytes = 0;     ///< Free-space floor (statvfs f_bavail × f_frsize).
};

/// Outcome of one enforceRetention() pass.
struct RetentionResult {
    int      deleted     = 0;         ///< Files unlinked.
    uint64_t freedBytes  = 0;         ///< Allocated bytes of the unlinked files.
    uint64_t ownedBytes  = 0;         ///< Owned bytes remaining after the pass.
    int64_t  freeBytes   = -1;        ///< Free bytes after the pass; -1 = statvfs failed.
    int      errors      = 0;         ///< unlink failures (logged, skipped).
    bool     stillOver   = false;     ///< Quota or floor still violated (nothing deletable left).
};

/**
 * @brief Delete the oldest owned segments until owned bytes ≤ maxBytes and
 *        free space ≥ minFreeBytes.
 *
 * Oldest = lowest SEQ (ties: file name), never the wall-clock part.  A .mkv and
 * its .ass are one unit; an orphan .ass is its own unit.  Only regular files
 * (lstat — no symlinks, no directories) whose names parse as owned segments are
 * considered, and only units with SEQ < @p deletableBelowSeq (the recorder's
 * watermark, protecting the open and still-finalising segments).  Uses
 * ::unlink; ENOENT counts as done; other failures are logged and skipped, so a
 * pass always terminates.  @p keepGoing is checked between deletions (pass the
 * shutdown flag).
 */
RetentionResult enforceRetention(const RetentionPolicy& policy, uint64_t deletableBelowSeq,
                                 const std::function<bool()>& keepGoing,
                                 const dashcam::log::LogCallback& log);

} // namespace dashcam::record

#endif // LIBRECORD_H
