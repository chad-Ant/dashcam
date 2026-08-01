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
#include <mutex>
#include <string>
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

} // namespace dashcam::record

#endif // LIBRECORD_H
