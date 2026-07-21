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
 *   while (running)
 *       rec.setOverlayData({lat, lon, alt, spd, acc, hdg, epochMs()});
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
#include <atomic>
#include <cstdint>
#include <fstream>
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
    OverlayData                    overlayData_;
    dashcam::config::OverlayConfig overlayConfig_;
    mutable std::mutex             overlayMutex_;

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
    /// it stays live regardless of telemetry age); when @p stale is true the
    /// motion/position fields render as a dash instead of a frozen reading.
    void writeAssSample(int64_t posNs, int64_t durNs, const OverlayData& od,
                        int64_t wallNowMs, bool stale);
};

} // namespace dashcam::record

#endif // LIBRECORD_H
