#include "librecord.h"

#include <linux/videodev2.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
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

// Quote a GStreamer string property.  Device paths and filenames are external
// configuration, so they must not be able to terminate the property and append
// arbitrary pipeline elements.
static std::string gstQuoted(const std::string& value) {
    std::string out;
    out.reserve(value.size() + 2);
    out.push_back('"');
    for (char ch : value) {
        if (ch == '\\' || ch == '"') out.push_back('\\');
        out.push_back(ch);
    }
    out.push_back('"');
    return out;
}

// ─── telemetry formatting (same content the Cairo overlay drew) ───────────────

/// True when @p deg is a heading this file can format without invoking UB.
///
/// std::lround() below is undefined — not merely wrong — for an argument that
/// does not fit a long, and std::isfinite() does NOT bound that: 1e30f is
/// perfectly finite and 1e30/45 overflows every integer type.  So the range is
/// checked, not just the finiteness.  The bound is the semantic one rather than
/// the representable one: a heading outside [0, 360] is not a heading, and a
/// caller sending one has a defect that should surface as a dash rather than as
/// a plausible-looking direction computed modulo 8.
static bool headingRenderable(float deg) {
    return std::isfinite(deg) && (deg >= 0.0f) && (deg <= 360.0f);
}

static const char* headingToCardinal(float deg) {
    static const char* kCardinals[] = {
        "N", "NE", "E", "SE", "S", "SW", "W", "NW" };
    // Total by construction.  Callers already gate on headingRenderable(), but
    // this is a file-static helper next to a formatter, and relying on every
    // future call site remembering the precondition is how the original defect
    // got in.  The clamp costs nothing and removes the UB at the source.
    if (!headingRenderable(deg)) return kCardinals[0];
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

void Recorder::setCompressedFrameCallback(CompressedFrameCallback cb) {
    frameCb_ = std::move(cb);
}

// ─── live-stream tap: appsink new-sample handler ──────────────────────────────

GstFlowReturn Recorder::onNewSample(GstAppSink* sink, gpointer user) {
    auto* self = static_cast<Recorder*>(user);
    GstSample* sample = gst_app_sink_pull_sample(sink);
    if (!sample) return GST_FLOW_OK;

    GstBuffer* buf = gst_sample_get_buffer(sample);
    if (buf && self->frameCb_) {
        GstMapInfo map;
        if (gst_buffer_map(buf, &map, GST_MAP_READ)) {
            const bool keyframe = !GST_BUFFER_FLAG_IS_SET(buf, GST_BUFFER_FLAG_DELTA_UNIT);
            self->frameCb_(map.data, map.size, keyframe);
            gst_buffer_unmap(buf, &map);
        }
    }
    gst_sample_unref(sample);
    return GST_FLOW_OK;
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

    // ADAS status banner (top-centre, alignment 8): the lane + driver-fatigue
    // telemetry the dashcam computes onboard.  Same box style as the corners.
    assFile_ << "Style: ADAS," << face << "," << fontPx
             << ",&H00FFFFFF,&H00FFFFFF,&H" << alphaHex << "000000,&H"
             << alphaHex << "000000," << bold
             << ",0,0,0,100,100,0,0,3," << boxPad << ",0,8,"
             << marginX << "," << marginX << "," << marginY << ",1\n";

    assFile_ << "\n[Events]\n"
                "Format: Layer, Start, End, Style, Name, MarginL, MarginR, "
                "MarginV, Effect, Text\n";
}

void Recorder::writeAssSample(int64_t posNs, int64_t durNs, const OverlayData& od,
                              int64_t wallNowMs, bool speedStale, bool accelStale,
                              bool positionStale, bool headingStale) {
    const std::string t0 = assTime(posNs);
    const std::string t1 = assTime(posNs + durNs);
    char buf[192];

    // Motion and position age SEPARATELY, because they come from separate
    // hardware that fails separately: speed and acceleration from the ECU over
    // OBD-II, position and heading from the GNSS receiver.  A single staleness
    // decision could only ever be right about one of them — so a vehicle with a
    // dead ECU and a good fix either lost its position needlessly, or kept
    // displaying a speed the ECU stopped reporting minutes ago.  The second is
    // the dangerous one: a frozen-but-plausible speed burned into a recording is
    // false evidence, and it is wrong by an amount too small to look wrong.
    //
    // Rendered exactly like the ADAS halves below, for the same reason: an
    // invalid source becomes a dash, never a stale or fabricated reading.
    // Speed and acceleration are dashed INDEPENDENTLY within the same corner.
    // They share a source only when the ECU is the one supplying speed; with a
    // GNSS fix and a dead ECU, speed is live and acceleration is not, and a
    // single decision for the pair necessarily lies about one of them.
    char spdBuf[48];
    char accBuf[48];
    if (speedStale) std::snprintf(spdBuf, sizeof(spdBuf), "SPD -- km/h");
    else            std::snprintf(spdBuf, sizeof(spdBuf), "SPD %.1f km/h",
                                  static_cast<double>(od.speedKmh));
    if (accelStale) std::snprintf(accBuf, sizeof(accBuf), "ACC -- m/s2");
    else            std::snprintf(accBuf, sizeof(accBuf), "ACC %+.1f m/s2",
                                  static_cast<double>(od.accelerationMs2));
    std::snprintf(buf, sizeof(buf), "%s\\N%s", spdBuf, accBuf);
    assFile_ << "Dialogue: 0," << t0 << "," << t1 << ",TL,,0,0,0,," << buf << "\n";

    // Heading is dashed INDEPENDENTLY of the coordinates.  A stationary vehicle
    // has a perfectly good fix and no trustworthy course — below roughly walking
    // pace a receiver reports a direction that wanders the whole circle, so the
    // master sends NaN for it while the position stays valid.  Sharing one flag
    // with the coordinates meant the last travelled heading, or the struct's
    // 90.0f placeholder before any fix at all, was rendered as live at every
    // stop.
    if (headingStale) {
        assFile_ << "Dialogue: 0," << t0 << "," << t1 << ",TR,,0,0,0,,HDG --\n";
    } else {
        std::snprintf(buf, sizeof(buf), "HDG %03.0f %s",
                      static_cast<double>(od.headingDeg), headingToCardinal(od.headingDeg));
        assFile_ << "Dialogue: 0," << t0 << "," << t1 << ",TR,,0,0,0,," << buf << "\n";
    }

    if (positionStale) {
        assFile_ << "Dialogue: 0," << t0 << "," << t1
                 << ",BL,,0,0,0,,LAT --\\NLON --\\NALT --\n";
    } else {
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

    // ADAS banner (top-centre): lane position + fatigue state.  Drawn only when
    // the application has fed a valid ADAS snapshot (adasValid); otherwise the
    // recording carries just the GPS corners.  The lane and driver halves are
    // rendered INDEPENDENTLY from laneValid / driverValid: an invalid source
    // becomes a dash rather than a stale or fabricated reading (the two detectors
    // warm up and fail on their own).  No commas in the text — the ASS Dialogue
    // Text field is positional.
    if (od.adasValid) {
        char lane[64];
        if (od.laneValid && od.laneOffsetValid && od.egoLaneIndex >= 0)
            std::snprintf(lane, sizeof(lane), "LANE %d/%d %+.2f",
                          od.egoLaneIndex + 1, od.laneCount,
                          static_cast<double>(od.laneOffset));
        else
            std::snprintf(lane, sizeof(lane), "LANE --");

        char fat[64];
        if (od.driverValid) {
            static const char* lvlName[4] = { "OK", "CAUTION", "WARN", "FATIGUE" };
            const char* lvl = (od.fatigueLevel >= 0 && od.fatigueLevel < 4)
                                  ? lvlName[od.fatigueLevel] : "?";
            std::snprintf(fat, sizeof(fat), "FAT %.0f %s%s%s",
                          static_cast<double>(od.fatigueScore), lvl,
                          od.driverDrowsy ? "  DROWSY" : "",
                          od.faceDetected ? ""         : "  NOFACE");
        } else {
            std::snprintf(fat, sizeof(fat), "FAT --");
        }

        char adas[256];
        std::snprintf(adas, sizeof(adas), "%s   %s", lane, fat);
        assFile_ << "Dialogue: 0," << t0 << "," << t1 << ",ADAS,,0,0,0,," << adas << "\n";
    }
}

// ─── sidecar path helpers ─────────────────────────────────────────────────────

// "/footage/clip.mkv" → "/footage/clip.ass".  Replaces the extension when the
// basename has one, otherwise appends.
static std::string sidecarPathFor(const std::string& videoPath) {
    std::string p = videoPath;
    const auto dot = p.find_last_of('.');
    const auto sep = p.find_last_of('/');
    if (dot != std::string::npos && (sep == std::string::npos || dot > sep))
        p.erase(dot);
    return p + ".ass";
}

std::string Recorder::segmentPattern(const std::string& filename) {
    // splitmuxsink runs g_strdup_printf() on `location`, so any '%' already in
    // the caller's path would be read as a conversion and corrupt the output
    // name (or worse, read a non-existent argument).  Escape them.
    auto escapePercent = [](const std::string& s) {
        std::string out;
        out.reserve(s.size() + 8);
        for (char c : s) { out += c; if (c == '%') out += '%'; }
        return out;
    };
    const auto dot = filename.find_last_of('.');
    const auto sep = filename.find_last_of('/');
    const bool hasExt = (dot != std::string::npos) &&
                        (sep == std::string::npos || dot > sep + 1);
    const std::string base = hasExt ? filename.substr(0, dot) : filename;
    const std::string ext  = hasExt ? filename.substr(dot)    : std::string(".mkv");
    return escapePercent(base) + "_%05d" + escapePercent(ext);
}

void Recorder::openSidecarLocked(const std::string& path) {
    assPath_ = path;
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

void Recorder::closeSidecarLocked() {
    if (!assFile_.is_open()) return;
    assFile_.flush();
    assFile_.close();
    doLog(log_, dashcam::log::LogLevel::INFO,
          "telemetry sidecar closed: %s", assPath_.c_str());
}

// ─── segment rotation ─────────────────────────────────────────────────────────

gchar* Recorder::onFormatLocation(GstElement* splitmux, guint fragmentId,
                                  GstSample* firstSample, gpointer user) {
    (void)splitmux;
    auto* self = static_cast<Recorder*>(user);

    // Build this fragment's name from the pattern splitmuxsink already holds, so
    // the naming rule lives in exactly one place (segmentPattern()).
    gchar*      cLoc = nullptr;
    g_object_get(G_OBJECT(splitmux), "location", &cLoc, NULL);
    const std::string pattern = cLoc ? cLoc : "";
    if (cLoc) g_free(cLoc);
    if (pattern.empty()) return nullptr;     // fall back to splitmuxsink's own naming

    gchar* namedC = g_strdup_printf(pattern.c_str(), fragmentId);
    const std::string named = namedC ? namedC : "";
    if (!namedC) return nullptr;

    // Where this segment starts on the pipeline's running-time axis.  The
    // subtitle thread samples gst_element_query_position(), which keeps counting
    // across fragments, so without this rebase every sidecar after the first
    // would carry timestamps far beyond its own segment's duration and render
    // nothing.
    int64_t baseNs = 0;
    if (firstSample) {
        if (GstBuffer* buf = gst_sample_get_buffer(firstSample)) {
            const GstClockTime pts = GST_BUFFER_PTS(buf);
            if (GST_CLOCK_TIME_IS_VALID(pts)) baseNs = static_cast<int64_t>(pts);
        }
    }

    {
        std::lock_guard<std::mutex> lock(self->assMutex_);
        // Rotation is atomic under the lock: the outgoing sidecar is flushed and
        // closed before the new base time is published, so a Dialogue line can
        // never straddle two segments.
        self->closeSidecarLocked();
        self->segmentBaseNs_ = baseNs;

        bool assEnabled;
        {
            std::lock_guard<std::mutex> cfgLock(self->overlayMutex_);
            assEnabled = (bool)self->overlayConfig_.enabled;
        }
        if (assEnabled) self->openSidecarLocked(sidecarPathFor(named));
    }

    doLog(self->log_, dashcam::log::LogLevel::INFO,
          "recording segment %u: %s", fragmentId, named.c_str());
    return namedC;   // transfer full — splitmuxsink g_free()s it
}

// ─── subtitle / bus thread ────────────────────────────────────────────────────

void Recorder::subtitleLoop() {
    float   rateHz;
    bool    assConfigured;
    int64_t staleMs;
    {
        std::lock_guard<std::mutex> lock(overlayMutex_);
        rateHz        = (float)overlayConfig_.subtitleRateHz;
        assConfigured = (bool)overlayConfig_.enabled;
        staleMs       = (int64_t)(int)overlayConfig_.staleTimeoutMs;
    }
    if (rateHz <= 0.0f) rateHz = 5.0f;
    const auto interval =
        std::chrono::milliseconds(static_cast<int64_t>(1000.0f / rateHz));
    const int64_t durNs = static_cast<int64_t>(1e9 / rateHz);

    GstBus* bus = gst_element_get_bus(pipeline_);
    int64_t lastPos = -1;
    auto nextAssFlush = std::chrono::steady_clock::now() + std::chrono::seconds(5);

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

        if (!assConfigured) continue;

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
        //
        // Judged per domain.  A domain is also stale whenever its validity flag
        // is clear, so a source the application knows to be dead is dashed
        // immediately rather than after the timeout: the flag says "this is not
        // backed by a live source", which is a stronger statement than "this has
        // not been refreshed lately" and should not wait for a clock.
        // A non-finite value is treated as stale REGARDLESS of its validity
        // flag.  The flag is set by the application, which is outside this
        // library's control, and a NaN or Inf reaching the formatters below
        // becomes "nan" burned into the recording.  Trusting a caller-supplied
        // bool over the number it describes would put a garbage reading in the
        // render path of a device whose output is evidence.
        //
        // Heading is checked for RANGE, not just finiteness, because finiteness
        // is not enough for it: the cardinal-point conversion divides by 45 and
        // calls std::lround(), which is undefined for an argument that does not
        // fit a long — and 1e30f is finite.  An out-of-range heading is also
        // simply not a heading, so dashing it is the correct rendering as well
        // as the safe one.
        const bool speedStale =
            !od.speedValid || !std::isfinite(od.speedKmh) ||
            (staleMs > 0 && (nowMs - od.speedTimestampMs) > staleMs);
        const bool accelStale =
            !od.accelValid || !std::isfinite(od.accelerationMs2) ||
            (staleMs > 0 && (nowMs - od.accelTimestampMs) > staleMs);
        const bool positionStale =
            !od.positionValid ||
            !std::isfinite(od.latitude) || !std::isfinite(od.longitude) ||
            !std::isfinite(od.altitudeM) ||
            (staleMs > 0 && (nowMs - od.positionTimestampMs) > staleMs);
        const bool headingStale =
            !od.headingValid || !headingRenderable(od.headingDeg) ||
            (staleMs > 0 && (nowMs - od.headingTimestampMs) > staleMs);

        // The sidecar file and the segment time base are both republished by
        // onFormatLocation() on a streaming thread, so re-check them under the
        // lock on every sample rather than caching either at thread start.
        {
            std::lock_guard<std::mutex> lock(assMutex_);
            if (!assFile_.is_open()) continue;
            // Timestamps are relative to the segment this sample belongs to, so
            // each sidecar starts near 00:00:00 and lines up with its own MKV.
            // Clamped: a sample can be queried a hair before the fragment's
            // first buffer PTS, which would otherwise go negative.
            const int64_t rel = std::max<int64_t>(0, pos - segmentBaseNs_);
            writeAssSample(rel, durNs, od, nowMs, speedStale, accelStale,
                           positionStale, headingStale);
            if (std::chrono::steady_clock::now() >= nextAssFlush) {
                assFile_.flush();
                nextAssFlush = std::chrono::steady_clock::now()
                             + std::chrono::seconds(5);
            }
        }
    }

    if (bus) gst_object_unref(bus);
}

// ─── recording lifecycle ──────────────────────────────────────────────────────

bool Recorder::startRecording(const std::string& devicePath,
                              const RecordingFormat& fmt,
                              const std::string& filename,
                              uint32_t maxFps,
                              uint32_t eosTimeoutMs,
                              uint32_t segmentSeconds) {
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

    const bool streaming = static_cast<bool>(frameCb_);

    // Common source + caps (+ MJPEG rate cap); everything downstream of this is
    // where the record and (optional) stream branches diverge.
    std::string trunk = "v4l2src name=camerasrc device=" + gstQuoted(devicePath);
    if (mjpeg) {
        trunk += " ! image/jpeg, width=" + std::to_string(fmt.width) +
                 ", height=" + std::to_string(fmt.height) +
                 ", framerate=" + std::to_string(frNum) + "/" + std::to_string(frDen);
        // MJPEG frames are intra-only: dropping them to cap the rate is safe.
        if (maxFps > 0 && static_cast<float>(maxFps) < fmt.fps)
            trunk += " ! videorate drop-only=true max-rate=" + std::to_string(maxFps);
    } else {
        trunk += " ! video/x-h264, width=" + std::to_string(fmt.width) +
                 ", height=" + std::to_string(fmt.height) +
                 ", framerate=" + std::to_string(frNum) + "/" + std::to_string(frDen);
    }

    // Record branch: (H264 needs h264parse for matroskamux) → queue → mux → file.
    //
    // segmentSeconds == 0 keeps the original single-file tail byte-identical.
    // Non-zero swaps the mux+filesink pair for a splitmuxsink, which finalises
    // each segment as it closes so an abrupt power cut can only lose the one in
    // progress.
    //
    // muxer= and sink= MUST be spelled into this string rather than assigned
    // afterwards.  They are GstElement-valued properties, and parse-launch
    // builds the element from the description — which matters because
    // splitmuxsink requests its sink pad FROM THE CURRENT MUXER at link time.
    // Set them after the parse and the queue is linked against the default
    // mp4mux instead, which rejects the caps, releases the request pad and fails
    // the pipeline with "Internal data stream error" before a frame moves.
    //
    // (The string-valued muxer-factory / muxer-properties pair is the wrong tool
    // here regardless: it only takes effect with async-finalize=true, which this
    // design avoids — see the stopRecording() note.)
    //
    // offset-to-zero on the muxer matters MORE with splitmuxsink than without:
    // reset-muxer defaults true, so matroskamux is driven to NULL between
    // fragments and recomputes its zero per segment.  Each segment therefore
    // starts at t=0 instead of at its absolute running time, which is exactly
    // what the per-segment sidecars assume.
    const std::string recTail =
        segmentSeconds > 0
            ? " ! splitmuxsink name=fsink"
              " muxer=\"matroskamux offset-to-zero=true\""
              " sink=\"filesink sync=false async=false\""
              " max-size-bytes=0"
              " max-size-time=" +
                  std::to_string(static_cast<guint64>(segmentSeconds) * GST_SECOND) +
              " location=" + gstQuoted(segmentPattern(filename))
            : " ! matroskamux offset-to-zero=true"
              " ! filesink name=fsink sync=false async=false location=" +
                  gstQuoted(filename);
    const std::string recBranch =
        std::string(h264 ? " ! h264parse" : "") +
        " ! queue max-size-buffers=8 leaky=0" + recTail;

    std::string desc;
    if (!streaming) {
        desc = trunk + recBranch;
    } else {
        // tee fans identical buffers to both branches.  The stream branch has its
        // OWN queue (leaky=downstream → drop old frames, never backpressure the
        // recording) and, for H.264, its own h264parse producing an Annex-B
        // byte-stream with in-band SPS/PPS (config-interval=-1) so a viewer that
        // joins mid-stream can decode.  drop=true on the appsink is a second guard.
        desc  = trunk + " ! tee name=rectee";
        desc += " rectee." + recBranch;
        desc += " rectee. ! queue max-size-buffers=4 leaky=downstream";
        if (h264)
            desc += " ! h264parse config-interval=-1"
                    " ! video/x-h264, stream-format=byte-stream, alignment=au";
        desc += " ! appsink name=streamsink emit-signals=false sync=false max-buffers=4 drop=true";
    }

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

    videoW_         = fmt.width;
    videoH_         = fmt.height;
    eosTimeoutMs_   = eosTimeoutMs;
    segmentSeconds_ = segmentSeconds;
    segmentBaseNs_  = 0;

    // Everything about the splitmuxsink except the rotation callback is already
    // configured by the parse string above, so all that is left is to hook the
    // per-fragment signal that names each segment and rotates its sidecar.
    if (segmentSeconds > 0) {
        GstElement* smux = gst_bin_get_by_name(GST_BIN(pipeline_), "fsink");
        if (!smux) {
            doLog(log_, dashcam::log::LogLevel::ERROR,
                  "splitmuxsink 'fsink' not found in the recording pipeline");
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            return false;
        }
        g_signal_connect(smux, "format-location-full",
                         G_CALLBACK(&Recorder::onFormatLocation), this);
        gst_object_unref(smux);

        doLog(log_, dashcam::log::LogLevel::INFO,
              "segmented recording: %us per segment, pattern %s%s",
              segmentSeconds, segmentPattern(filename).c_str(),
              mjpeg ? " (MJPEG is intra-only — segments cut exactly on time)"
                    : " (H264: cuts land on the camera's own IDRs, so a segment "
                      "can overrun the target)");
    }

    // Attach the live-stream tap: pull each compressed frame off the appsink and
    // forward it to frameCb_.  set_callbacks() does not take ownership of the
    // element, so release the ref gst_bin_get_by_name() added.
    if (streaming) {
        GstElement* appsink = gst_bin_get_by_name(GST_BIN(pipeline_), "streamsink");
        if (appsink) {
            GstAppSinkCallbacks cbs;
            std::memset(&cbs, 0, sizeof(cbs));
            cbs.new_sample = &Recorder::onNewSample;
            gst_app_sink_set_callbacks(GST_APP_SINK(appsink), &cbs, this, nullptr);
            gst_object_unref(appsink);
        } else {
            doLog(log_, dashcam::log::LogLevel::WARN,
                  "stream tap: appsink 'streamsink' not found — recording without streaming");
        }
    }

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
    // In segmented mode the sidecar is opened (and rotated) by
    // onFormatLocation() as each fragment forms, so there is nothing to open
    // here — and nothing to clobber, since that handler may already have run.
    if (assEnabled && segmentSeconds == 0) {
        std::lock_guard<std::mutex> lock(assMutex_);
        openSidecarLocked(sidecarPathFor(filename));
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

    // Closed only after the pipeline is fully down, so the rotation handler can
    // no longer fire and reopen it behind us.
    {
        std::lock_guard<std::mutex> lock(assMutex_);
        closeSidecarLocked();
    }
    segmentSeconds_ = 0;
    segmentBaseNs_  = 0;
}

bool Recorder::isRecording() const {
    return pipeline_ != nullptr && healthy_.load(std::memory_order_relaxed);
}

} // namespace dashcam::record
