/**
 * @file librecord.h
 * @brief Cairo telemetry overlay and GStreamer recording branch for any Camera_GST camera.
 *
 * Recorder is camera-type-agnostic.  It produces a self-contained GstBin that
 * can be handed to any Camera_GST::addBranch() call — CSI, USB, or any future
 * subclass.  The bin accepts whatever format the tee emits (NV12/NVMM from CSI,
 * BGRx/YUYV from USB) because nvvidconv at the bin's inlet handles format and
 * memory-type conversion before the cairo overlay stage.
 *
 * Typical usage:
 * @code
 *   Camera_CSI cam(info);
 *   cam.setAttributeDictionary(attrDict);
 *
 *   dashcam::record::Recorder recorder;
 *   recorder.setOverlayConfig(cfg.overlay);
 *
 *   GstElement* bin = recorder.createRecordingBin("clip.mkv", frNum, frDen, cfg.encoder);
 *   cam.addBranch("recording", bin, false, false);   // valve starts closed
 *   cam.open();
 *   cam.start();
 *
 *   cam.setBranchEnabled("recording", true);
 *   while (running) {
 *       cam.captureFrame(buf, size, written);
 *       recorder.setOverlayData({lat, lon, alt, spd, acc, hdg, tsMs});
 *   }
 *
 *   recorder.disconnect();   // MUST be called before cam.stop() / cam.close()
 *   cam.stop();
 *   cam.close();
 * @endcode
 *
 * @warning disconnect() MUST be called before the camera pipeline is torn down
 *          (before stop() or close()).  The Cairo "draw" signal fires on the
 *          GStreamer streaming thread; if the pipeline transitions to NULL while
 *          a draw callback is in flight, the cairo_t* and GstElement* become
 *          dangling pointers and the process will crash.  The Recorder destructor
 *          also calls disconnect() as a safety net, but relying on it is a bug.
 *
 * @note Requires GStreamer ≥ 1.20, Cairo, and the NVIDIA Jetson GStreamer plugins
 *       (nvvidconv) on Orin Nano / JetPack 6.2.
 * @note Orin Nano has no NVENC; x264enc (software) is used for encoding.
 */

#ifndef LIBRECORD_H
#define LIBRECORD_H

#include "libconfig.h"
#include "liblog.h"
#include <gst/gst.h>
#include <cstdint>
#include <mutex>
#include <string>

// Forward-declare cairo_t so the private callbacks can reference it without
// pulling the full Cairo headers into every translation unit that includes this header.
typedef struct _cairo cairo_t;

namespace dashcam::record {

// ─── telemetry payload ────────────────────────────────────────────────────────

/**
 * @brief Telemetry snapshot rendered as a four-corner overlay on recorded video.
 *
 * Written by the application thread via Recorder::setOverlayData(); read by the
 * GStreamer streaming thread inside the Cairo "draw" callback.  The Recorder
 * copies the struct under a mutex on each write/read, so it is safe to call
 * setOverlayData() from any thread at any time while the pipeline is running.
 *
 * Corner layout:
 *   - Top-left:     speed, acceleration
 *   - Top-right:    heading + cardinal direction
 *   - Bottom-left:  latitude, longitude, altitude
 *   - Bottom-right: UTC date and time
 */
struct OverlayData {
    double  latitude        = 10.7725;    ///< WGS-84 latitude in decimal degrees.
    double  longitude       = 106.6581;   ///< WGS-84 longitude in decimal degrees.
    double  altitudeM       = 52.3;       ///< Altitude above mean sea level in metres.
    float   speedKmh        = 1.5f;       ///< Ground speed in kilometres per hour.
    float   accelerationMs2 = 0.0f;       ///< Longitudinal acceleration in m/s² (+ve = forward).
    float   headingDeg      = 90.0f;      ///< True heading in degrees (0 = North, clockwise).
    int64_t timestampMs     = 1777633580; ///< UNIX epoch timestamp in milliseconds.
};

// ─── Recorder ─────────────────────────────────────────────────────────────────

/**
 * @brief Manages one Cairo-overlaid recording branch for a Camera_GST camera.
 *
 * Each Recorder instance handles exactly one recording bin at a time.  Create a
 * new Recorder (or call disconnect() and createRecordingBin() again) to start a
 * new recording session.
 *
 * Thread safety:
 *   - setOverlayData() and getOverlayData() are safe to call from any thread.
 *   - setOverlayConfig() is safe before start() and between stop()/start() cycles.
 *     Calling it while the pipeline is RUNNING is also safe (protected by the
 *     same mutex) but the change takes effect on the next rendered frame.
 *   - createRecordingBin() and disconnect() must be called from the same thread
 *     that calls Camera_GST lifecycle methods (they are not re-entrant).
 */
class Recorder {
public:
    Recorder();

    /**
     * @brief Destructor; calls disconnect() if signal handlers are still connected.
     *
     * @warning If the camera pipeline has already been destroyed at this point
     *          the behaviour is undefined.  Always call disconnect() explicitly
     *          before camera.stop() / camera.close().
     */
    ~Recorder();

    Recorder(const Recorder&)            = delete;
    Recorder& operator=(const Recorder&) = delete;

    // ─── overlay data ──────────────────────────────────────────────────────────

    /**
     * @brief Replace the current telemetry snapshot.  Thread-safe.
     * @param data  New telemetry values; copied under the internal mutex.
     */
    void setOverlayData(const OverlayData& data);

    /**
     * @brief Return a copy of the current telemetry snapshot.  Thread-safe.
     */
    OverlayData getOverlayData() const;

    /**
     * @brief Replace the overlay rendering configuration.
     *
     * Thread-safe.  Changes take effect on the next rendered frame.
     * May be called before or during recording.
     *
     * @param cfg  Font, size, opacity, and enabled flag.
     */
    void setOverlayConfig(const dashcam::config::OverlayConfig& cfg);

    /**
     * @brief Inject a log callback for pipeline construction diagnostics.
     *        Defaults to a no-op (silent) if not set.
     */
    void setLogCallback(dashcam::log::LogCallback cb);

    // ─── recording bin ─────────────────────────────────────────────────────────

    /**
     * @brief Create the GstBin that writes an MKV file with a telemetry overlay.
     *
     * The returned bin accepts the camera tee's native format on its ghost sink
     * pad.  nvvidconv at the inlet converts from NV12/NVMM (CSI) or BGRx (USB)
     * to BGRx system memory before the Cairo overlay stage.  The internal chain:
     * @verbatim
     *   nvvidconv ! video/x-raw,format=BGRx
     *     ! cairooverlay
     *     ! videoconvert ! video/x-raw,format=I420
     *     ! queue ! videorate ! video/x-raw,framerate=N/D
     *     ! x264enc ! h264parse ! matroskamux ! filesink
     * @endverbatim
     *
     * Pass the returned pointer to Camera_GST::addBranch() and then call
     * Camera_GST::start().  The Recorder connects the Cairo signal handlers
     * internally during this call, so createRecordingBin() must be called
     * before addBranch() and start().
     *
     * @param filename    Output MKV path (absolute or relative to cwd).
     * @param frNum       Target frame rate numerator (use Camera_GST::computeFpsRational).
     * @param frDen       Target frame rate denominator.
     * @param enc         Encoder parameters (bitrate, speed-preset, keyIntMax, tune).
     * @param queueDepth  Depth (buffers) of the pre-encoder queue; from RecordingConfig::queueDepth.
     * @return Newly created GstBin (floating ref) on success; nullptr on failure.
     *         Ownership transfers to the pipeline via addBranch() / gst_bin_add().
     */
    GstElement* createRecordingBin(const std::string& filename,
                                   uint32_t frNum = 60, uint32_t frDen = 1,
                                   const dashcam::config::EncoderConfig& enc = {},
                                   uint32_t queueDepth = 3);

    // ─── lifecycle ─────────────────────────────────────────────────────────────

    /**
     * @brief Disconnect the Cairo "draw" and "caps-changed" signal handlers.
     *
     * Safe to call multiple times (no-op if already disconnected).  Must be
     * called before the camera pipeline is torn down (before stop() or close())
     * to prevent the streaming thread from calling into a freed cairo_t*.
     */
    void disconnect();

private:
    dashcam::log::LogCallback      log_{};
    OverlayData                    overlayData_;
    dashcam::config::OverlayConfig overlayConfig_;
    mutable std::mutex             overlayMutex_;

    /// Non-owning pointer to the cairooverlay element inside the recording bin.
    /// Valid from createRecordingBin() until disconnect().
    GstElement* cairoOverlay_ = nullptr;
    gulong      cairoDrawId_  = 0;  ///< "draw" signal handler ID; 0 when disconnected.
    gulong      cairoCapsId_  = 0;  ///< "caps-changed" signal handler ID; 0 when disconnected.
    int         videoWidth_   = 0;  ///< Frame width set by the caps-changed callback.
    int         videoHeight_  = 0;  ///< Frame height set by the caps-changed callback.

    /// Cached font parse result (derived from overlayConfig_.fontFace in setOverlayConfig).
    /// Protected by overlayMutex_; snapshotted in renderOverlay() alongside overlayConfig_.
    std::string cachedFontFamily_;
    int         cachedFontWeight_ = 0;  ///< cairo_font_weight_t
    int         cachedFontSlant_  = 0;  ///< cairo_font_slant_t

    /** @brief Render all four corner labels onto @p cr for the current frame. */
    void renderOverlay(cairo_t* cr);

    /** @brief GStreamer "draw" signal callback; delegates to renderOverlay(). */
    static void onCairoDraw(GstElement*, cairo_t*, GstClockTime, GstClockTime, gpointer);

    /** @brief GStreamer "caps-changed" callback; stores frame dimensions. */
    static void onCairoCapsChanged(GstElement*, GstCaps*, gpointer);
};

} // namespace dashcam::record

#endif // LIBRECORD_H
