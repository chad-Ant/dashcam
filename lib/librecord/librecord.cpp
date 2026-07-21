#include "librecord.h"

#include <linux/videodev2.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <ctime>

namespace dashcam::record {

// ─── file-local log helper ────────────────────────────────────────────────────

static void doLog(const dashcam::log::LogCallback& cb, dashcam::log::LogLevel lvl,
                  const char* fmt, ...) {
    if (!cb) return;
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    cb(lvl, buf);
}

// ─── telemetry formatting (same content the Cairo overlay drew) ───────────────

static const char* headingToCardinal(float deg) {
    static const char* kCardinals[] = {
        "N", "NE", "E", "SE", "S", "SW", "W", "NW" };
    const int idx = static_cast<int>(std::lround(deg / 45.0f)) & 7;
    return kCardinals[idx];
}

// ASS timestamp: H:MM:SS.CC (centiseconds).
static std::string assTime(int64_t ns) {
    if (ns < 0) ns = 0;
    const int64_t cs = ns / 10000000;   // centiseconds
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%d:%02d:%02d.%02d",
                  static_cast<int>(cs / 360000),
                  static_cast<int>((cs / 6000) % 60),
                  static_cast<int>((cs / 100) % 60),
                  static_cast<int>(cs % 100));
    return buf;
}

// ─── construction / destruction ───────────────────────────────────────────────

Recorder::Recorder()  = default;

Recorder::~Recorder() { stopRecording(); }

// ─── telemetry accessors ──────────────────────────────────────────────────────

void Recorder::setOverlayData(const OverlayData& data) {
    std::lock_guard<std::mutex> lock(overlayMutex_);
    overlayData_ = data;
}

OverlayData Recorder::getOverlayData() const {
    std::lock_guard<std::mutex> lock(overlayMutex_);
    return overlayData_;
}

void Recorder::setOverlayConfig(const dashcam::config::OverlayConfig& cfg) {
    std::lock_guard<std::mutex> lock(overlayMutex_);
    overlayConfig_ = cfg;
}

void Recorder::setLogCallback(dashcam::log::LogCallback cb) {
    log_ = std::move(cb);
}

// ─── ASS sidecar ──────────────────────────────────────────────────────────────

void Recorder::writeAssHeader() {
    dashcam::config::OverlayConfig cfg;
    {
        std::lock_guard<std::mutex> lock(overlayMutex_);
        cfg = overlayConfig_;
    }

    // Font size scales with the video resolution: OverlayConfig::fontSize is
    // the size at 720p and PlayResY tracks the real height, so the rendered
    // text keeps the same proportion of the frame at any resolution.
    const float scale   = static_cast<float>(videoH_) / 720.0f;
    const int   fontPx  = std::max(4, static_cast<int>(std::lround(
                              static_cast<float>(cfg.fontSize) * scale)));
    const int   marginX = static_cast<int>(std::lround((float)cfg.labelPadX * scale));
    const int   marginY = static_cast<int>(std::lround((float)cfg.labelPadY * scale));
    const int   boxPad  = std::max(1, static_cast<int>(std::lround(
                              (float)cfg.labelPadY * scale * 0.5f)));

    // ASS alpha: 00 = opaque, FF = transparent.
    const float opacity = std::min(1.0f, std::max(0.0f, (float)cfg.backgroundOpacity));
    const int   alpha   = static_cast<int>(std::lround((1.0f - opacity) * 255.0f));

    // "Monospace Bold" (Pango-style) → font name + bold flag.
    std::string face = cfg.fontFace;
    int bold = 0;
    const auto b = face.find(" Bold");
    if (b != std::string::npos) { bold = -1; face.erase(b, 5); }
    if (face.empty()) face = "Monospace";

    assFile_ << "[Script Info]\n"
                "Title: dashcam telemetry\n"
                "ScriptType: v4.00+\n"
                "PlayResX: " << videoW_ << "\n"
                "PlayResY: " << videoH_ << "\n"
                "WrapStyle: 2\n"
                "ScaledBorderAndShadow: yes\n\n";

    // BorderStyle=3 draws an opaque box (BackColour) behind the text — the
    // overlay rectangle; Outline doubles as the box padding.
    // Alignment (numpad): 7=TL, 9=TR, 1=BL, 3=BR — the four corners.
    assFile_ << "[V4+ Styles]\n"
                "Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, "
                "OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, "
                "ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, "
                "Alignment, MarginL, MarginR, MarginV, Encoding\n";
    char alphaHex[8];
    std::snprintf(alphaHex, sizeof(alphaHex), "%02X", alpha);
    const int aligns[4] = { 7, 9, 1, 3 };
    static const char* names[4] = { "TL", "TR", "BL", "BR" };
    for (int i = 0; i < 4; ++i) {
        assFile_ << "Style: " << names[i] << "," << face << "," << fontPx
                 << ",&H00FFFFFF,&H00FFFFFF,&H" << alphaHex << "000000,&H"
                 << alphaHex << "000000," << bold
                 << ",0,0,0,100,100,0,0,3," << boxPad << ",0,"
                 << aligns[i] << "," << marginX << "," << marginX << ","
                 << marginY << ",1\n";
    }

    assFile_ << "\n[Events]\n"
                "Format: Layer, Start, End, Style, Name, MarginL, MarginR, "
                "MarginV, Effect, Text\n";
}

void Recorder::writeAssSample(int64_t posNs, int64_t durNs, const OverlayData& od,
                              int64_t wallNowMs, bool stale) {
    const std::string t0 = assTime(posNs);
    const std::string t1 = assTime(posNs + durNs);
    char buf[192];

    // Stale telemetry is no longer trustworthy: draw a dash in place of the
    // motion/position fields rather than a frozen (and now misleading) reading.
    if (stale) {
        assFile_ << "Dialogue: 0," << t0 << "," << t1
                 << ",TL,,0,0,0,,SPD -- km/h\\NACC -- m/s2\n";
        assFile_ << "Dialogue: 0," << t0 << "," << t1 << ",TR,,0,0,0,,HDG --\n";
        assFile_ << "Dialogue: 0," << t0 << "," << t1
                 << ",BL,,0,0,0,,LAT --\\NLON --\\NALT --\n";
    } else {
        std::snprintf(buf, sizeof(buf), "SPD %.1f km/h\\NACC %+.1f m/s2",
                      static_cast<double>(od.speedKmh),
                      static_cast<double>(od.accelerationMs2));
        assFile_ << "Dialogue: 0," << t0 << "," << t1 << ",TL,,0,0,0,," << buf << "\n";

        std::snprintf(buf, sizeof(buf), "HDG %03.0f %s",
                      static_cast<double>(od.headingDeg), headingToCardinal(od.headingDeg));
        assFile_ << "Dialogue: 0," << t0 << "," << t1 << ",TR,,0,0,0,," << buf << "\n";

        std::snprintf(buf, sizeof(buf), "LAT %.6f %c\\NLON %.6f %c\\NALT %.1f m",
                      std::abs(od.latitude),  od.latitude  >= 0.0 ? 'N' : 'S',
                      std::abs(od.longitude), od.longitude >= 0.0 ? 'E' : 'W',
                      od.altitudeM);
        assFile_ << "Dialogue: 0," << t0 << "," << t1 << ",BL,,0,0,0,," << buf << "\n";
    }

    // The bottom-right clock is the device wall clock, not GPS telemetry, so it
    // stays live even when the telemetry snapshot has gone stale.
    time_t epochSec = static_cast<time_t>(wallNowMs / 1000LL);
    struct tm tmBuf;
    localtime_r(&epochSec, &tmBuf);
    char dateBuf[16], timeBuf[16];
    std::strftime(dateBuf, sizeof(dateBuf), "%Y-%m-%d", &tmBuf);
    std::strftime(timeBuf, sizeof(timeBuf), "%H:%M:%S %Z", &tmBuf);
    assFile_ << "Dialogue: 0," << t0 << "," << t1 << ",BR,,0,0,0,,"
             << dateBuf << "\\N" << timeBuf << "\n";

    assFile_.flush();
}

// ─── subtitle / bus thread ────────────────────────────────────────────────────

void Recorder::subtitleLoop() {
    float   rateHz;
    bool    writeAss;
    int64_t staleMs;
    {
        std::lock_guard<std::mutex> lock(overlayMutex_);
        rateHz   = (float)overlayConfig_.subtitleRateHz;
        writeAss = (bool)overlayConfig_.enabled && assFile_.is_open();
        staleMs  = (int64_t)(int)overlayConfig_.staleTimeoutMs;
    }
    if (rateHz <= 0.0f) rateHz = 5.0f;
    const auto interval =
        std::chrono::milliseconds(static_cast<int64_t>(1000.0f / rateHz));
    const int64_t durNs = static_cast<int64_t>(1e9 / rateHz);

    GstBus* bus = gst_element_get_bus(pipeline_);
    int64_t lastPos = -1;

    while (!stopFlag_.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(interval);

        // Bus errors (device unplugged, negotiation failure mid-stream) latch
        // the session unhealthy; the application polls isRecording().
        if (bus) {
            while (GstMessage* msg = gst_bus_pop_filtered(bus, GST_MESSAGE_ERROR)) {
                GError* err = nullptr;
                gst_message_parse_error(msg, &err, nullptr);
                doLog(log_, dashcam::log::LogLevel::ERROR,
                      "recording pipeline error: %s", err ? err->message : "?");
                if (err) g_error_free(err);
                gst_message_unref(msg);
                healthy_.store(false);
            }
        }

        if (!writeAss) continue;

        gint64 pos = 0;
        if (!gst_element_query_position(pipeline_, GST_FORMAT_TIME, &pos))
            continue;                       // not producing yet
        if (pos <= lastPos) continue;       // keep event times monotonic
        lastPos = pos;

        OverlayData od;
        {
            std::lock_guard<std::mutex> lock(overlayMutex_);
            od = overlayData_;
        }

        const int64_t nowMs =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        // Stale when the snapshot's own capture time is older than the window
        // (a timestamp in the future is treated as fresh).  staleMs==0 disables.
        const bool stale = staleMs > 0 && (nowMs - od.timestampMs) > staleMs;

        writeAssSample(pos, durNs, od, nowMs, stale);
    }

    if (bus) gst_object_unref(bus);
}

// ─── recording lifecycle ──────────────────────────────────────────────────────

bool Recorder::startRecording(const std::string& devicePath,
                              const RecordingFormat& fmt,
                              const std::string& filename,
                              uint32_t maxFps,
                              uint32_t eosTimeoutMs) {
    if (pipeline_) {
        doLog(log_, dashcam::log::LogLevel::ERROR,
              "startRecording: session already active");
        return false;
    }
    if (fmt.width == 0 || fmt.height == 0 || !(fmt.fps > 0.0f)) {
        doLog(log_, dashcam::log::LogLevel::ERROR,
              "startRecording: invalid format %ux%u@%.1f",
              fmt.width, fmt.height, static_cast<double>(fmt.fps));
        return false;
    }

    // Strict precompressed gate: this recorder exists to avoid software
    // encoding on a box with no NVENC.  Raw formats need an encoder → refuse.
    const bool mjpeg = fmt.v4l2PixFmt == V4L2_PIX_FMT_MJPEG;
    const bool h264  = fmt.v4l2PixFmt == V4L2_PIX_FMT_H264;
    if (!mjpeg && !h264) {
        doLog(log_, dashcam::log::LogLevel::ERROR,
              "startRecording: pixel format %c%c%c%c is raw — only precompressed "
              "UVC streams (MJPG/H264) are recordable on Orin Nano (no NVENC)",
              fmt.v4l2PixFmt & 0xff, (fmt.v4l2PixFmt >> 8) & 0xff,
              (fmt.v4l2PixFmt >> 16) & 0xff, (fmt.v4l2PixFmt >> 24) & 0xff);
        return false;
    }

    gint frNum = 30, frDen = 1;
    gst_util_double_to_fraction(static_cast<double>(fmt.fps), &frNum, &frDen);

    std::string desc = "v4l2src name=camerasrc device=" + devicePath;
    if (mjpeg) {
        desc += " ! image/jpeg, width=" + std::to_string(fmt.width) +
                ", height=" + std::to_string(fmt.height) +
                ", framerate=" + std::to_string(frNum) + "/" + std::to_string(frDen);
        // MJPEG frames are intra-only: dropping them to cap the rate is safe.
        if (maxFps > 0 && static_cast<float>(maxFps) < fmt.fps)
            desc += " ! videorate drop-only=true max-rate=" + std::to_string(maxFps);
    } else {
        desc += " ! video/x-h264, width=" + std::to_string(fmt.width) +
                ", height=" + std::to_string(fmt.height) +
                ", framerate=" + std::to_string(frNum) + "/" + std::to_string(frDen) +
                " ! h264parse";
    }
    desc += " ! queue max-size-buffers=8 leaky=0"
            " ! matroskamux offset-to-zero=true"
            " ! filesink name=fsink sync=false async=false location=\"" +
            filename + "\"";

    doLog(log_, dashcam::log::LogLevel::INFO, "recording pipeline: %s", desc.c_str());

    GError* err = nullptr;
    pipeline_ = gst_parse_launch(desc.c_str(), &err);
    if (!pipeline_ || err) {
        doLog(log_, dashcam::log::LogLevel::ERROR,
              "recording pipeline parse failed: %s", err ? err->message : "?");
        if (err) g_error_free(err);
        if (pipeline_) { gst_object_unref(pipeline_); pipeline_ = nullptr; }
        return false;
    }

    videoW_       = fmt.width;
    videoH_       = fmt.height;
    eosTimeoutMs_ = eosTimeoutMs;

    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING)
            == GST_STATE_CHANGE_FAILURE) {
        doLog(log_, dashcam::log::LogLevel::ERROR,
              "recording pipeline refused to start on %s", devicePath.c_str());
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        return false;
    }

    // ASS sidecar next to the recording: clip.mkv → clip.ass.
    bool assEnabled;
    {
        std::lock_guard<std::mutex> lock(overlayMutex_);
        assEnabled = (bool)overlayConfig_.enabled;
    }
    if (assEnabled) {
        assPath_ = filename;
        const auto dot = assPath_.find_last_of('.');
        const auto sep = assPath_.find_last_of('/');
        if (dot != std::string::npos && (sep == std::string::npos || dot > sep))
            assPath_.erase(dot);
        assPath_ += ".ass";
        assFile_.open(assPath_, std::ios::trunc);
        if (assFile_.is_open()) {
            writeAssHeader();
            doLog(log_, dashcam::log::LogLevel::INFO,
                  "telemetry sidecar: %s", assPath_.c_str());
        } else {
            doLog(log_, dashcam::log::LogLevel::ERROR,
                  "cannot open telemetry sidecar %s — recording without it",
                  assPath_.c_str());
        }
    }

    stopFlag_.store(false);
    healthy_.store(true);
    subThread_ = std::thread(&Recorder::subtitleLoop, this);
    return true;
}

void Recorder::stopRecording() {
    if (!pipeline_) return;

    stopFlag_.store(true, std::memory_order_release);
    if (subThread_.joinable()) subThread_.join();

    // EOS → matroskamux writes duration/cues → wait for the EOS to reach the
    // sink (bounded), then tear down.
    gst_element_send_event(pipeline_, gst_event_new_eos());
    GstBus* bus = gst_element_get_bus(pipeline_);
    if (bus) {
        GstMessage* msg = gst_bus_timed_pop_filtered(
            bus, eosTimeoutMs_ * GST_MSECOND,
            static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
        if (!msg) {
            doLog(log_, dashcam::log::LogLevel::WARN,
                  "EOS flush timed out after %u ms; forcing teardown", eosTimeoutMs_);
        } else {
            if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
                GError* err = nullptr;
                gst_message_parse_error(msg, &err, nullptr);
                doLog(log_, dashcam::log::LogLevel::ERROR,
                      "error during EOS flush: %s", err ? err->message : "?");
                if (err) g_error_free(err);
            }
            gst_message_unref(msg);
        }
        gst_object_unref(bus);
    }

    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    healthy_.store(false);

    if (assFile_.is_open()) {
        assFile_.flush();
        assFile_.close();
        doLog(log_, dashcam::log::LogLevel::INFO,
              "telemetry sidecar closed: %s", assPath_.c_str());
    }
}

bool Recorder::isRecording() const {
    return pipeline_ != nullptr && healthy_.load(std::memory_order_relaxed);
}

} // namespace dashcam::record
