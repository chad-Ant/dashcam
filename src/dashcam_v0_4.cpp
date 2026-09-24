// dashcam_v0_4.cpp — dashcam software v0.4
//
// v0.4 = dashcam recording ONLY.  Recording starts immediately at program
// start: one USB UVC camera's own compressed stream (MJPEG/H.264) is copied
// into gapless MKV segments (no decode, no encode) with an .ass sidecar
// showing the live clock (speed/heading/position as "--": no GPS yet).
// No lane detection, no driver monitoring, no network.
//
// Loop overwrite: when this program's own segments exceed <MaxFootageGB>, or
// free space drops below <MinFreeGB>, the oldest segments (lowest SEQ — never
// the clock, which can be unset at boot) are deleted.  The open and
// still-finalising segments, other programs' files and v0.3's footage are
// never touched.
//
// Recovery: a missing camera, an unplug, a pipeline error or a silent stall
// restarts recording into a new segment (retry forever).  A process crash or a
// teardown wedged in the kernel is left to the launcher's restart loop.
//
// Low light (ExposureMode=framerate, the default): the camera's own
// auto-exposure would stretch exposure past the frame time and drop to ~15 fps
// in the dark.  Instead a software loop (libcamera_exposure) keeps exposure
// under the frame time and adds gain, fed ~2 Hz by the recorder's luma tap —
// 30 fps held, darker/noisier night footage.  MJPEG cameras only.
//
// Clock (no RTC battery: the Jetson boots with the last shutdown time): a
// background thread asks an NTP server and, while NTP has not succeeded in
// the last hour, GPS UTC from the ESP32-C3 bridge is the fallback
// (libtimesync).  A wrong clock is corrected by setting the system clock; any
// clock step — ours or the host's own NTP service — starts a new segment so
// file names and the sidecar clock are right from then on.  Recording never
// waits for the time: it starts immediately and is split when the clock moves.
//
// Camera choice (resolveRecordCamera): enabled explicit <Camera type="USB">
// pins first (a pin that is not a capture device right now is logged and
// ignored), then the first compressed-capable USB camera.  A <Camera
// name="cabin" type="USB"> device is the last resort: it is recorded (with a
// warning) only when no other USB camera can record.
#include "libcamera.h"
#include "libcamera_exposure.h"
#include "libcommlink.h"
#include "libconfig.h"
#include "liblog.h"
#include "libnetwork.h"
#include "librecord.h"
#include "libtimesync.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <gst/gst.h>
#include <memory>
#include <mutex>
#include <linux/videodev2.h>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/file.h>
#include <sys/statvfs.h>
#include <unistd.h>

using namespace dashcam::camera;
using dashcam::log::LogLevel;
namespace fs  = std::filesystem;
namespace rec = dashcam::record;

// ─── shutdown flag ────────────────────────────────────────────────────────────

static volatile std::sig_atomic_t g_run = 1;
static void onSignal(int) { g_run = 0; }

// ─── tuning ───────────────────────────────────────────────────────────────────

static constexpr const char* kPrefix            = "dashcam";
static constexpr const char* kLockName          = ".dashcam_v04.lock";
static constexpr auto        kTick              = std::chrono::milliseconds(200);
static constexpr auto        kRetentionEvery    = std::chrono::seconds(30);
static constexpr auto        kPreferredProbe    = std::chrono::seconds(60);
static constexpr auto        kFailureSummary    = std::chrono::seconds(60);
static constexpr uint64_t    kHardFloorBytes    = 1000000000ULL;  // retain BEFORE starting below this
static constexpr uint32_t    kEosTimeoutMs      = 4000;

// ─── helpers ──────────────────────────────────────────────────────────────────

static int64_t epochMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

static dashcam::log::LogParams makeLogParams(const dashcam::config::LogConfig& l) {
    dashcam::log::LogParams lp;
    lp.queueSize     = static_cast<uint32_t>(l.queueSize);
    lp.rotateSizeKb  = static_cast<uint32_t>(l.rotateSizeKb);
    lp.rotateFiles   = static_cast<uint32_t>(l.rotateFiles);
    lp.flushEverySec = static_cast<uint32_t>(l.flushEverySec);
    lp.level         = l.level;
    lp.flushOn       = l.flushOn;
    return lp;
}

static uint64_t gbToBytes(float gb) {
    return gb > 0.0f ? static_cast<uint64_t>(static_cast<double>(gb) * 1e9) : 0;
}

// Free bytes on the filesystem holding @p dir; -1 when unknown.
static int64_t freeBytes(const std::string& dir) {
    struct statvfs vfs;
    if (::statvfs(dir.c_str(), &vfs) != 0) return -1;
    return static_cast<int64_t>(vfs.f_bavail) * static_cast<int64_t>(vfs.f_frsize);
}

// Create @p dir and prove it takes a write.  Returns 0 or the failing errno
// (ENOSPC is distinguished by the caller: the storage is present but full).
static int probeDir(const std::string& dir) {
    std::error_code ec;
    fs::create_directories(dir, ec);
    const std::string probe = dir + "/.dashcam_write_probe";
    const int fd = ::open(probe.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    if (fd < 0) return errno ? errno : EIO;
    int err = 0;
    if (::write(fd, "x", 1) != 1) err = errno ? errno : EIO;
    if (::fsync(fd) != 0 && !err) err = errno ? errno : EIO;
    if (::close(fd) != 0 && !err) err = errno ? errno : EIO;
    ::unlink(probe.c_str());
    return err;
}

// <repo>/footage_fallback: the binary lives in <repo>/bin/build_<ts>/, so this is
// stable across builds (v0.3's per-build fallback was never cleaned up).
static std::string fallbackFootageDir() {
    std::error_code ec;
    const fs::path exe = fs::read_symlink("/proc/self/exe", ec);
    if (ec) return "footage_fallback";
    return (exe.parent_path().parent_path().parent_path() / "footage_fallback").string();
}

// ─── camera choice (pure; covered by --self-test) ─────────────────────────────

// Largest compressed mode <=1080p at 24-31 fps; H.264 preferred over MJPEG
// (smaller files at the same zero encode cost).  Same rule as v0.3.
static int pickUsbRecordFormat(const cameraInfo& ci) {
    long bestH264Area = -1, bestMjpgArea = -1;
    int  bestH264 = -1, bestMjpg = -1;
    for (size_t i = 0; i < ci.videoFormats.size(); ++i) {
        const auto& f = ci.videoFormats[i];
        if (f.frameRate < 24.0f || f.frameRate > 31.0f) continue;
        if (f.width > 1920 || f.height > 1080) continue;
        const long area = static_cast<long>(f.width) * f.height;
        if (f.pixelFormat == V4L2_PIX_FMT_H264) {
            if (area > bestH264Area) { bestH264Area = area; bestH264 = (int)i; }
        } else if (f.pixelFormat == V4L2_PIX_FMT_MJPEG) {
            if (area > bestMjpgArea) { bestMjpgArea = area; bestMjpg = (int)i; }
        }
    }
    return bestH264 >= 0 ? bestH264 : bestMjpg;
}

struct RecordChoice {
    const cameraInfo*        cam = nullptr;
    int                      fmt = -1;
    bool                     cabinFallback = false;  ///< Only the cabin camera could record.
    std::vector<std::string> notes;                  ///< WARN-level explanations.
};

static RecordChoice resolveRecordCamera(const std::vector<cameraInfo>& cams,
                                        const std::vector<dashcam::config::CameraConfig>& configs) {
    RecordChoice rc;
    std::string cabinDev;
    for (const auto& cfg : configs)
        if (cfg.type == "USB" && cfg.name == "cabin") { cabinDev = (std::string)cfg.device; break; }

    auto exactConfig = [&](const cameraInfo& c) -> const dashcam::config::CameraConfig* {
        for (const auto& cfg : configs)
            if (cfg.type == "USB" && !std::string(cfg.device).empty() &&
                (std::string)cfg.device == c.address)
                return &cfg;
        return nullptr;
    };
    auto formatFor = [&](const dashcam::config::CameraConfig* cfg, const cameraInfo& c) -> int {
        if (!cfg) return pickUsbRecordFormat(c);
        const int idx = (int)cfg->formatIndex;
        if (idx >= 0 && static_cast<size_t>(idx) < c.videoFormats.size()) {
            const auto pf = c.videoFormats[static_cast<size_t>(idx)].pixelFormat;
            if (pf == V4L2_PIX_FMT_H264 || pf == V4L2_PIX_FMT_MJPEG) return idx;
            rc.notes.push_back(c.address + ": configured FormatIndex " + std::to_string(idx) +
                               " is not compressed; selecting a format automatically");
        } else {
            rc.notes.push_back(c.address + ": configured FormatIndex " + std::to_string(idx) +
                               " is out of range; selecting a format automatically");
        }
        return pickUsbRecordFormat(c);
    };
    auto findUsb = [&](const std::string& dev) -> const cameraInfo* {
        for (const auto& c : cams)
            if (c.type == CAMERA_TYPE::USB && c.address == dev) return &c;
        return nullptr;
    };

    // 1. Explicit, enabled, non-cabin USB pins.
    for (const auto& cfg : configs) {
        if (cfg.type != "USB" || cfg.name == "cabin" || !(bool)cfg.enabled) continue;
        const std::string dev = cfg.device;
        if (dev.empty()) continue;
        const cameraInfo* c = findUsb(dev);
        if (!c) {
            rc.notes.push_back("config pin '" + cfg.name + "' -> " + dev +
                               " is not a USB capture device right now (stale pin?) — ignored");
            continue;
        }
        if (!cabinDev.empty() && c->address == cabinDev) continue;
        const int idx = formatFor(&cfg, *c);
        if (idx >= 0) { rc.cam = c; rc.fmt = idx; return rc; }
    }

    // 2. First compressed-capable USB camera that is not disabled or the cabin.
    auto disabled = [&](const cameraInfo& c) {
        const auto* cfg = exactConfig(c);
        return cfg && !(bool)cfg->enabled;
    };
    for (const auto& c : cams) {
        if (c.type != CAMERA_TYPE::USB || disabled(c)) continue;
        if (!cabinDev.empty() && c.address == cabinDev) continue;
        const int idx = formatFor(exactConfig(c), c);
        if (idx >= 0) { rc.cam = &c; rc.fmt = idx; return rc; }
    }

    // 3. Last resort: the cabin camera — footage beats no footage.
    if (!cabinDev.empty()) {
        if (const cameraInfo* c = findUsb(cabinDev)) {
            const int idx = pickUsbRecordFormat(*c);
            if (idx >= 0) {
                rc.cam = c;
                rc.fmt = idx;
                rc.cabinFallback = true;
                rc.notes.push_back("only the cabin camera " + cabinDev +
                                   " can record — recording it as dashcam footage");
            }
        }
    }
    return rc;
}

// ─── --self-test: resolver invariants, no hardware ────────────────────────────

static int runSelfTest() {
    int failures = 0;
    auto check = [&](bool ok, const char* what) {
        std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
        if (!ok) ++failures;
    };
    auto usb = [](const char* dev, std::vector<cameraVideoFormat> fmts) {
        cameraInfo c;
        c.type = CAMERA_TYPE::USB;
        c.address = dev;
        c.deviceId = 0;
        c.videoFormats = std::move(fmts);
        return c;
    };
    const cameraVideoFormat mj4k  {3840, 2160, 30.0f, V4L2_PIX_FMT_MJPEG, "MJPG"};
    const cameraVideoFormat mj1080{1920, 1080, 30.0f, V4L2_PIX_FMT_MJPEG, "MJPG"};
    const cameraVideoFormat mj60  {1920, 1080, 60.0f, V4L2_PIX_FMT_MJPEG, "MJPG"};
    const cameraVideoFormat h720  {1280,  720, 30.0f, V4L2_PIX_FMT_H264,  "H264"};
    const cameraVideoFormat yuyv  { 640,  480, 30.0f, V4L2_PIX_FMT_YUYV,  "YUYV"};
    auto pin = [](const char* name, const char* dev, bool enabled = true, int fmt = 0) {
        dashcam::config::CameraConfig c;
        c.name = name;
        c.type = "USB";
        c.device = std::string(dev);
        c.enabled = enabled;
        c.formatIndex = fmt;
        return c;
    };

    std::printf("dashcam_v0_4 self-test (camera choice)\n");
    {
        // The UGREEN 4K: MJPEG 4K@30, 1080p@60/30, YUYV — no H.264.
        const std::vector<cameraInfo> cams{usb("/dev/video0", {mj4k, mj60, mj1080, yuyv})};
        const auto rc = resolveRecordCamera(cams, {});
        check(rc.cam == &cams[0] && rc.fmt == 2, "auto: 1080p30 MJPEG (not 4K, not 60 fps)");
    }
    {
        const std::vector<cameraInfo> cams{usb("/dev/video0", {mj1080, h720})};
        const auto rc = resolveRecordCamera(cams, {});
        check(rc.fmt == 1, "auto: H.264 preferred over MJPEG");
    }
    {
        const std::vector<cameraInfo> cams{usb("/dev/video0", {yuyv})};
        check(resolveRecordCamera(cams, {}).cam == nullptr, "raw-only camera is not recordable");
        check(resolveRecordCamera({}, {}).cam == nullptr, "no cameras -> none");
    }
    {
        // Today's stale config: usb0 -> /dev/video1 (metadata node, not discovered).
        const std::vector<cameraInfo> cams{usb("/dev/video0", {mj4k, mj1080})};
        const auto rc = resolveRecordCamera(cams, {pin("usb0", "/dev/video1")});
        check(rc.cam == &cams[0] && rc.fmt == 1, "stale pin ignored, auto-pick records");
        check(!rc.notes.empty() && rc.notes[0].find("stale pin") != std::string::npos,
              "stale pin is reported");
    }
    {
        const std::vector<cameraInfo> cams{usb("/dev/video0", {mj1080}), usb("/dev/video2", {mj4k, mj1080})};
        const auto rc = resolveRecordCamera(cams, {pin("road", "/dev/video2", true, 0)});
        check(rc.cam == &cams[1] && rc.fmt == 0, "explicit pin wins with its FormatIndex");
        const auto bad = resolveRecordCamera(cams, {pin("road", "/dev/video2", true, 9)});
        check(bad.cam == &cams[1] && bad.fmt == 1, "out-of-range FormatIndex falls back");
    }
    {
        const std::vector<cameraInfo> cams{usb("/dev/video0", {mj1080}), usb("/dev/video2", {mj1080})};
        const auto rc = resolveRecordCamera(cams, {pin("x", "/dev/video0", false)});
        check(rc.cam == &cams[1], "disabled device skipped");
    }
    {
        const std::vector<cameraInfo> cams{usb("/dev/video0", {mj1080}), usb("/dev/video2", {mj1080})};
        const auto rc = resolveRecordCamera(cams, {pin("cabin", "/dev/video0")});
        check(rc.cam == &cams[1] && !rc.cabinFallback, "cabin reserved when another camera records");
    }
    {
        const std::vector<cameraInfo> cams{usb("/dev/video0", {mj1080})};
        const auto rc = resolveRecordCamera(cams, {pin("cabin", "/dev/video0")});
        check(rc.cam == &cams[0] && rc.cabinFallback && !rc.notes.empty(),
              "lone cabin camera recorded with a warning");
    }
    {
        const std::vector<cameraInfo> cams{usb("/dev/video0", {mj1080}), usb("/dev/video2", {yuyv})};
        const auto rc = resolveRecordCamera(cams, {pin("cabin", "/dev/video0")});
        check(rc.cam == &cams[0] && rc.cabinFallback, "cabin recorded when the other camera is raw-only");
    }
    std::printf("RESULT: %s (%d failure%s)\n", failures ? "FAIL" : "PASS", failures,
                failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}

// ─── storage: preferred footage dir or the fixed fallback, single-instance lock ─

class Storage {
public:
    Storage(std::string preferred, rec::RetentionPolicy policy, dashcam::log::LogCallback log)
        : preferred_(std::move(preferred)), fallback_(fallbackFootageDir()),
          policy_(std::move(policy)), log_(std::move(log)) {}
    ~Storage() { release(); }

    /// Pick (and lock) the directory to record into; "" when nothing is usable.
    std::string acquire(std::string& why) {
        std::string dir;
        const int err = usable(preferred_);
        if (err == 0) {
            dir = preferred_;
        } else {
            const int ferr = usable(fallback_);
            if (ferr != 0) {
                why = "no writable footage directory (" + preferred_ + ": " + std::strerror(err) +
                      "; " + fallback_ + ": " + std::strerror(ferr) + ")";
                return "";
            }
            if (!onFallback_ || dir_ != fallback_)
                log_(LogLevel::WARN, "footage directory " + preferred_ + " unusable (" +
                                     std::strerror(err) + ") — recording to " + fallback_);
            dir = fallback_;
        }
        if (dir != dir_) {
            release();
            if (!lock(dir, why)) return "";
            dir_ = dir;
        }
        onFallback_ = (dir_ != preferred_);
        return dir_;
    }

    /// True when running on the fallback and the preferred directory works again.
    bool preferredBack() const { return onFallback_ && usable(preferred_) == 0; }

    const std::string& dir() const { return dir_; }

    rec::RetentionResult retain(uint64_t watermark) {
        rec::RetentionPolicy p = policy_;
        p.dir = dir_;
        return rec::enforceRetention(p, watermark, [] { return g_run != 0; }, log_);
    }

    void release() {
        if (lockFd_ >= 0) { ::close(lockFd_); lockFd_ = -1; }   // closing drops the flock
        dir_.clear();
    }

private:
    std::string               preferred_, fallback_, dir_;
    rec::RetentionPolicy      policy_;
    dashcam::log::LogCallback log_;
    int                       lockFd_ = -1;
    bool                      onFallback_ = false;

    // 0 when @p dir takes a write.  A full disk (ENOSPC) is still OUR storage:
    // make room with retention (nothing is recording, so every owned segment is
    // closed) and probe again.
    int usable(const std::string& dir) const {
        int err = probeDir(dir);
        if (err == ENOSPC) {
            rec::RetentionPolicy p = policy_;
            p.dir = dir;
            rec::enforceRetention(p, UINT64_MAX, [] { return g_run != 0; }, log_);
            err = probeDir(dir);
        }
        return err;
    }

    bool lock(const std::string& dir, std::string& why) {
        const std::string path = dir + "/" + kLockName;
        const int fd = ::open(path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0644);
        if (fd < 0) { why = "cannot open " + path + ": " + std::strerror(errno); return false; }
        if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
            why = errno == EWOULDBLOCK ? "another dashcam_v0_4 is recording into " + dir
                                       : "cannot lock " + path + ": " + std::strerror(errno);
            ::close(fd);
            return false;
        }
        lockFd_ = fd;
        return true;
    }
};

// ─── repeated-message damper ──────────────────────────────────────────────────

// Wraps a log callback: each distinct message is logged once at its own level,
// repeats at DEBUG.  For chatty retry loops (a missing bridge, an offline NTP
// server) that would otherwise write the same line every few seconds forever.
static dashcam::log::LogCallback quietRepeats(dashcam::log::LogCallback log) {
    auto seen = std::make_shared<std::pair<std::mutex, std::set<std::string>>>();
    return [log, seen](LogLevel lvl, const std::string& msg) {
        bool first;
        {
            std::lock_guard<std::mutex> lock(seen->first);
            if (seen->second.size() > 256) seen->second.clear();
            first = seen->second.insert(msg).second;
        }
        log(first || lvl == LogLevel::DEBUG ? lvl : LogLevel::DEBUG, msg);
    };
}

// ─── failure reporting: first ERROR, repeats DEBUG, a summary every minute ────

class FailureLog {
public:
    explicit FailureLog(dashcam::log::LogCallback log) : log_(std::move(log)) {}

    void failed(const std::string& why) {
        const auto now = std::chrono::steady_clock::now();
        ++streak_;
        if (why != last_) {
            log_(LogLevel::ERROR, "recording not running: " + why + " (retrying)");
            last_ = why;
            repeats_ = 0;
            nextSummary_ = now + kFailureSummary;
            return;
        }
        ++repeats_;
        log_(LogLevel::DEBUG, "recording retry failed: " + why);
        if (now >= nextSummary_) {
            log_(LogLevel::ERROR, "recording still not running after " + std::to_string(streak_) +
                                  " attempts: " + why);
            nextSummary_ = now + kFailureSummary;
        }
    }

    void recovered() {
        if (streak_ > 0)
            log_(LogLevel::INFO, "recording recovered after " + std::to_string(streak_) +
                                 " failed attempt" + (streak_ == 1 ? "" : "s"));
        streak_ = 0;
        repeats_ = 0;
        last_.clear();
    }

    int streak() const { return streak_; }

private:
    dashcam::log::LogCallback             log_;
    std::string                           last_;
    int                                   streak_ = 0, repeats_ = 0;
    std::chrono::steady_clock::time_point nextSummary_{};
};

// ─── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    if (argc == 2 && std::string(argv[1]) == "--self-test") return runSelfTest();

    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);
    gst_init(&argc, &argv);

    // Config first (so <Log> can shape the logger); stderr until init.
    auto log = dashcam::log::getCallback();
    dashcam::config::AppConfig cfg;
    const std::string configsDir = dashcam::config::resolveStorageDir(
        dashcam::config::kDefaultConfigsDir, dashcam::config::kFallbackConfigsName, log);
    if (!dashcam::config::ConfigReader::loadOrCreate(configsDir + "/dashcam.xml", cfg, log))
        log(LogLevel::WARN, "config load/create failed; using defaults");

    const std::string logDir = dashcam::config::resolveStorageDir(
        dashcam::config::kDefaultLogDir, dashcam::config::kFallbackLogName, log);
    dashcam::log::init(logDir, makeLogParams(cfg.log));

    const auto& r = cfg.recording;
    const uint64_t quota = gbToBytes(r.maxFootageGB);
    const uint64_t floor = gbToBytes(r.minFreeGB);
    log(LogLevel::INFO, "dashcam v0.4 starting (recording only)");
    const std::string exposureMode = r.exposureMode;
    const bool frameRateExposure = exposureMode != "camera";
    if (exposureMode != "camera" && exposureMode != "framerate")
        log(LogLevel::WARN, "ExposureMode '" + exposureMode + "' unknown (framerate|camera) — using framerate");
    {
        char quotaBuf[32];
        std::snprintf(quotaBuf, sizeof(quotaBuf), "%.1f GB", (double)(float)r.maxFootageGB);
        const std::string quotaText = quota ? quotaBuf : "off";
        char buf[512];
        std::snprintf(buf, sizeof(buf),
                      "settings: footage=%s segment=%d s quota=%s floor=%.1f GB stall=%d ms "
                      "firstFrame=%d ms retry=%d s recordFps=%d exposure=%s config=%s/dashcam.xml",
                      std::string(cfg.system.footagePath).c_str(), (int)r.segmentSec,
                      quotaText.c_str(),
                      (double)(float)r.minFreeGB, (int)r.stallTimeoutMs, (int)r.firstFrameTimeoutMs,
                      (int)r.retryIntervalSec, (int)r.recordFps,
                      frameRateExposure ? ("framerate (target luma " +
                                           std::to_string((int)r.targetLuma) + ")").c_str()
                                        : "camera",
                      configsDir.c_str());
        log(LogLevel::INFO, buf);
    }
    if (quota > 0 && quota < 3000000000ULL)
        log(LogLevel::WARN, "MaxFootageGB holds only a few 1080p segments — footage will cycle fast");

    // librecord / discovery chatter is demoted to DEBUG while retries repeat, so a
    // camera that is gone for an hour does not flood the log.
    std::atomic<bool> quiet{false};
    auto recLog = [&](LogLevel lvl, const std::string& msg) {
        log(quiet.load() && lvl != LogLevel::DEBUG ? LogLevel::DEBUG : lvl, msg);
    };

    rec::RetentionPolicy policy;
    policy.prefix       = kPrefix;
    policy.maxBytes     = quota;
    policy.minFreeBytes = floor;
    Storage   storage(cfg.system.footagePath, policy, log);
    FailureLog failures(log);

    rec::SegmentedRecorder recorder;
    recorder.setLogCallback(recLog);
    recorder.setOverlayConfig(cfg.overlay);

    UvcExposureControl exposure;
    uint64_t           lastLumaSeq = 0;
    // Every session end hands exposure back to the camera (a no-op after an
    // unplug), so a stopped dashcam never leaves the camera in manual mode.
    auto stopRecording = [&]() {
        recorder.stop();
        exposure.close();
    };

    // ── clock: NTP first, GPS fallback ───────────────────────────────────────
    const auto& net = cfg.network;
    const bool clockSet = (bool)net.clockSetEnabled;
    dashcam::timesync::TimeKeeper        timeKeeper({}, log);
    dashcam::timesync::ClockJumpDetector clockJumps;
    std::atomic<bool>                    timeRun{true};
    std::thread                          ntpThread;
    if ((bool)net.timeSyncEnabled) {
        ntpThread = std::thread([&, ntpLog = quietRepeats(log)] {
            const std::string server = net.ntpServer;
            const int  tries   = 1 + std::max(0, (int)net.ntpRetries);
            bool       failing = false;
            while (timeRun.load()) {
                dashcam::network::TimeResult t;
                for (int i = 0; i < tries && !t.valid && timeRun.load(); ++i)
                    t = dashcam::network::queryTime(server, static_cast<uint16_t>((int)net.ntpPort),
                                                    (int)net.ntpTimeoutMs, ntpLog);
                if (t.valid) {
                    if (clockSet) {
                        timeKeeper.offerNtp(t.offsetSeconds, server);
                    } else if (failing || !timeKeeper.synced()) {
                        char off[48];
                        std::snprintf(off, sizeof(off), "%+.3f s", t.offsetSeconds);
                        log(LogLevel::INFO, std::string("time: NTP offset ") + off +
                                            " (ClockSetEnabled=false: clock left alone)");
                    }
                    failing = false;
                } else if (!failing) {
                    ntpLog(LogLevel::INFO, "time: no NTP reply from " + server + " (offline?) — retrying "
                                           "every 30 s" + ((bool)net.gpsTimeFallback && clockSet
                                                           ? "; GPS time is the fallback" : ""));
                    failing = true;
                }
                // Until NTP answers, retry every 30 s; afterwards re-check every
                // 30 min (the host's own NTP service keeps the clock fine-tuned).
                const auto until = std::chrono::steady_clock::now() +
                                   std::chrono::seconds(t.valid ? 1800 : 30);
                while (timeRun.load() && std::chrono::steady_clock::now() < until)
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        });
    } else {
        log(LogLevel::INFO, "time: NTP disabled (<Network><TimeSyncEnabled>)");
    }

    // GPS time comes from the ESP32-C3 bridge (MKR Zero GPS).  Optional: without
    // the bridge the link retries quietly in its own thread.
    dashcam::commlink::CommLink bridge;
    if (clockSet && (bool)net.gpsTimeFallback) {
        bridge.setTelemetryCallback([&timeKeeper](const dashcam::commlink::Telemetry& t) {
            dashcam::timesync::UtcFields u;
            u.year = t.year; u.month = t.month; u.day = t.day;
            u.hour = t.hour; u.minute = t.minute; u.second = t.second;
            timeKeeper.offerGps(u, (t.flags & hostproto::TLM_FLAG_TIME_VALID) != 0);
        });
        dashcam::commlink::CommLinkConfig bcfg;          // by-id discovery, auto-stream
        auto bridgeLog = quietRepeats(recLog);
        if (!bridge.open(bcfg, bridgeLog))
            bridgeLog(LogLevel::INFO, "time: GPS bridge (ESP32-C3) not present — retrying in the background");
        bridge.start();
    }

    std::set<std::string> notesSeen;   // resolver warnings are logged once each
    uint64_t sessions = 0;
    auto nextAttempt   = std::chrono::steady_clock::now();
    auto nextRetention = nextAttempt;
    auto nextProbe     = nextAttempt + kPreferredProbe;
    bool warnedStillOver = false;

    auto runRetention = [&]() {
        const auto res = storage.retain(recorder.deletableBelowSeq());
        if (res.deleted > 0)
            log(LogLevel::INFO, "retention: deleted " + std::to_string(res.deleted) + " file(s), " +
                                std::to_string(res.freedBytes / 1000000) + " MB; footage now " +
                                std::to_string(res.ownedBytes / 1000000) + " MB");
        if (res.stillOver && !warnedStillOver) {
            log(LogLevel::WARN, "retention: quota/free-space floor still violated with nothing "
                                "left to delete (free " +
                                (res.freeBytes < 0 ? std::string("unknown")
                                                   : std::to_string(res.freeBytes / 1000000) + " MB") +
                                ")");
        }
        warnedStillOver = res.stillOver;
        nextRetention = std::chrono::steady_clock::now() + kRetentionEvery;
    };

    auto scheduleRetry = [&](const std::string& why) {
        failures.failed(why);
        quiet.store(true);
        // The first retry after a working session is immediate; then back off.
        nextAttempt = std::chrono::steady_clock::now() +
                      (failures.streak() <= 1 ? std::chrono::seconds(0)
                                              : std::chrono::seconds((int)r.retryIntervalSec));
    };

    auto tryStart = [&]() -> bool {
        std::string why;
        const std::string dir = storage.acquire(why);
        if (dir.empty()) { scheduleRetry(why); return false; }
        const int64_t freeNow = freeBytes(dir);
        if (freeNow >= 0 && static_cast<uint64_t>(freeNow) < kHardFloorBytes) runRetention();

        std::vector<cameraInfo> cams;
        getCameraList(cams, recLog);
        const RecordChoice rc = resolveRecordCamera(cams, cfg.cameras);
        for (const auto& n : rc.notes)
            if (notesSeen.insert(n).second) log(LogLevel::WARN, n);
        if (!rc.cam) { scheduleRetry("no recordable USB camera found"); return false; }

        const auto& f = rc.cam->videoFormats[static_cast<size_t>(rc.fmt)];
        rec::RecordingFormat fmt;
        fmt.v4l2PixFmt = f.pixelFormat;
        fmt.width      = f.width;
        fmt.height     = f.height;
        fmt.fps        = f.frameRate;

        rec::SegmentOptions opts;
        opts.dir                 = dir;
        opts.prefix              = kPrefix;
        opts.segmentSec          = static_cast<uint32_t>((int)r.segmentSec);
        opts.maxFps              = static_cast<uint32_t>((int)r.recordFps);
        opts.eosTimeoutMs        = kEosTimeoutMs;
        opts.stallTimeoutMs      = static_cast<uint32_t>((int)r.stallTimeoutMs);
        opts.firstFrameTimeoutMs = static_cast<uint32_t>((int)r.firstFrameTimeoutMs);

        const bool mjpeg = f.pixelFormat == V4L2_PIX_FMT_MJPEG;
        recorder.setLumaTap(frameRateExposure && mjpeg);
        if (!recorder.start(rc.cam->address, fmt, opts)) {
            scheduleRetry(rc.cam->address + ": " + recorder.lastError());
            return false;
        }
        quiet.store(false);
        failures.recovered();
        ++sessions;
        char buf[256];
        std::snprintf(buf, sizeof(buf), "recording %s %ux%u %s @%.0f fps into %s%s",
                      rc.cam->address.c_str(), f.width, f.height,
                      f.pixelFormat == V4L2_PIX_FMT_H264 ? "H264" : "MJPEG",
                      (double)f.frameRate, dir.c_str(), rc.cabinFallback ? " (cabin camera)" : "");
        log(LogLevel::INFO, buf);
        if (frameRateExposure) {
            std::string whyNot;
            lastLumaSeq = 0;
            if (!mjpeg)
                log(LogLevel::INFO, "exposure: camera auto-exposure kept (frame-rate priority needs MJPEG)");
            else if (!exposure.open(rc.cam->address, f.frameRate, (int)r.targetLuma, recLog, whyNot))
                log(LogLevel::WARN, "exposure: camera auto-exposure kept — " + whyNot);
        }
        runRetention();
        return true;
    };

    // ── supervisor loop ──────────────────────────────────────────────────────
    while (g_run) {
        const auto now = std::chrono::steady_clock::now();
        double jump = 0.0;
        if (clockJumps.poll(jump)) {
            char b[96];
            std::snprintf(b, sizeof(b), "time: system clock stepped by %+.1f s", jump);
            if (recorder.isRecording() && recorder.splitNow())
                log(LogLevel::INFO, std::string(b) + " — new segment so names and the sidecar clock are right");
            else
                log(LogLevel::INFO, b);
        }
        if (!recorder.isActive()) {
            if (now >= nextAttempt) tryStart();
        } else if (!recorder.isRecording()) {
            const std::string why = recorder.lastError();
            stopRecording();
            scheduleRetry(why.empty() ? "recording stopped" : why);
        } else {
            rec::OverlayData od;
            od.timestampMs   = epochMs();
            // No GPS/IMU/ECU yet: every per-source validity flag stays false, so
            // speed, acceleration, heading and position render as dashes and
            // only the clock is live.
            od.adasValid     = false;
            recorder.setOverlayData(od);

            float    luma = 0.0f;
            uint64_t seq  = 0;
            if (exposure.isOpen() && recorder.latestLuma(luma, seq) && seq != lastLumaSeq) {
                lastLumaSeq = seq;
                exposure.onLuma(luma);
            }

            if (recorder.consumeFragmentClosed() || now >= nextRetention) runRetention();
            if (now >= nextProbe) {
                nextProbe = now + kPreferredProbe;
                if (storage.preferredBack()) {
                    log(LogLevel::INFO, "preferred footage directory is back — switching to it");
                    stopRecording();
                    storage.release();
                    nextAttempt = now;
                }
            }
        }
        std::this_thread::sleep_for(kTick);
    }

    log(LogLevel::INFO, "shutdown requested");
    stopRecording();
    timeRun.store(false);
    bridge.close();
    if (ntpThread.joinable()) ntpThread.join();
    storage.release();
    log(LogLevel::INFO, "dashcam v0.4 stopped (" + std::to_string(sessions) + " recording session" +
                        (sessions == 1 ? "" : "s") + ")");
    dashcam::log::shutdown();
    return 0;
}
