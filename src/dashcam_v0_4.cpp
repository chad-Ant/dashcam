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
// Camera: <Camera name="dashcam" type="USB"> pins the recording camera, best by
// its stable udev link (<Device>/dev/v4l/by-id/...-video-index0</Device>), which
// follows the camera when /dev/videoN changes; <FormatIndex> picks the mode (4 =
// 1080p30 MJPEG on the UGREEN 4K — 0 would be 4K).  With a pin, v0.4 records
// that camera or nothing: while it is missing v0.4 waits, and a camera another
// system owns (name="cabin" driver camera, a rangefinder) is never taken.
// Without a pin: the first compressed-capable USB camera (cabin last resort) —
// any camera, so the pin is what protects other systems'.  A dashcam.xml that
// exists but cannot be read: only a lone recordable USB camera is recorded.
//
// Recovery: a missing camera, an unplug, a pipeline error or a silent stall
// restarts recording into a new segment (retry forever).  A process crash or a
// teardown wedged in the kernel is left to the launcher's restart loop.
//
// Exposure (ExposureMode=auto, the default): a software loop
// (libcamera_exposure), fed ~2 Hz by the recorder's luma tap, keeps exposure
// under the frame time and adds gain — 30 fps held in dim light.  At night
// that ceiling is near-black (the camera's own auto-exposure is ~8x brighter:
// it uses internal gain the UVC gain control does not expose), so when the
// loop is pinned at its ceiling and still darker than NightLuma for 10 s the
// camera's auto-exposure takes over (~20 fps) until the camera runs at full
// frame rate again with a picture at least as bright as NightLuma.
// framerate = never hand over; camera = always the camera.
// MJPEG cameras only.
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
// warning) only when no other USB camera can record, and never when that entry
// is disabled.
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
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <gst/gst.h>
#include <memory>
#include <mutex>
#include <linux/videodev2.h>
#include <map>
#include <set>
#include <string>
#include <system_error>
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
// A camera that has not taken the exposure hand-back (two UVC control writes,
// normally a few ms) this long after the footage is final is wedged: a stuck
// control write only returns at the USB timeout (~5 s each).
static constexpr int         kCameraRestoreMs   = 2000;
// A directory probe (test write + fsync) not back after this long means that
// filesystem stopped answering: the directory counts as unusable.
static constexpr int         kProbeTimeoutMs    = 5000;
// The supervisor loop silent this long is stuck (everything it waits on is
// bounded well below this): exit for the launcher's restart loop.
static constexpr int         kSupervisorStuckMs = 30000;
// Before starting a session on a nearly full disk, wait this long at most for
// the retention pass (it runs on its own thread).
static constexpr int         kPreStartRetentionMs = 3000;

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

// probeDir() with a deadline: the supervisor must never wait on a filesystem
// that stopped answering.  The probe runs on its own thread; past the deadline
// it counts as ETIMEDOUT and the thread is left to finish (detached).  While a
// probe of a directory is still stuck, later probes of it fail at once rather
// than piling up threads.
class BoundedProbe {
public:
    using Fn = std::function<int(const std::string&)>;
    explicit BoundedProbe(Fn probe = probeDir, int timeoutMs = kProbeTimeoutMs)
        : probe_(std::move(probe)), timeoutMs_(timeoutMs) {}

    int operator()(const std::string& dir) const {
        auto sh = shared_;                               // outlives the detached probe
        {
            std::lock_guard<std::mutex> lk(sh->m);
            if (!sh->busy.insert(dir).second) return ETIMEDOUT;   // previous one still stuck
        }
        auto result = std::make_shared<std::promise<int>>();
        std::future<int> done = result->get_future();
        try {
            std::thread([sh, dir, result, probe = probe_] {
                const int err = probe(dir);
                {
                    std::lock_guard<std::mutex> lk(sh->m);
                    sh->busy.erase(dir);
                }
                result->set_value(err);
            }).detach();
        } catch (const std::system_error&) {             // no thread: probe inline
            {
                std::lock_guard<std::mutex> lk(sh->m);
                sh->busy.erase(dir);
            }
            return probe_(dir);
        }
        if (done.wait_for(std::chrono::milliseconds(timeoutMs_)) != std::future_status::ready)
            return ETIMEDOUT;
        return done.get();
    }

private:
    struct Shared {
        std::mutex            m;
        std::set<std::string> busy;
    };
    std::shared_ptr<Shared> shared_ = std::make_shared<Shared>();
    Fn                      probe_;
    int                     timeoutMs_;
};

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
    bool                     pinned = false;         ///< A <Camera name="dashcam"> entry decided.
    std::string              why;                    ///< No camera: the reason (retry message).
    std::vector<std::string> notes;                  ///< WARN-level explanations.
};

// The config entry that pins v0.4's recording camera (name="cabin" is the
// driver camera; other systems, e.g. a rangefinder, use their own names).
static constexpr const char* kDashcamEntry = "dashcam";
static constexpr const char* kByIdPrefix   = "/dev/v4l/by-id/";

static std::string trimmed(const std::string& s) {
    const auto a = s.find_first_not_of(" \t\r\n");
    return a == std::string::npos ? std::string() : s.substr(a, s.find_last_not_of(" \t\r\n") - a + 1);
}

// "idVendor:idProduct:serial" of the USB device behind a V4L2 node, from
// sysfs; "" when unknown.  Two cameras of one model with the same (often
// generic) serial share one /dev/v4l/by-id name.
using UsbIdentityFn = std::function<std::string(const std::string& node)>;
static std::string usbIdentity(const std::string& node) {
    std::error_code ec;
    const fs::path iface = fs::canonical(
        "/sys/class/video4linux/" + fs::path(node).filename().string() + "/device", ec);
    if (ec) return {};
    auto read = [](const fs::path& p) {
        std::ifstream in(p);
        std::string s;
        std::getline(in, s);
        return s;
    };
    for (fs::path d = iface; d.has_relative_path(); d = d.parent_path())   // interface → device
        if (fs::exists(d / "idVendor", ec))
            return read(d / "idVendor") + ":" + read(d / "idProduct") + ":" + read(d / "serial");
    return {};
}

// configKnown = false: dashcam.xml exists but could not be read, so any pin in
// it is unknown.  Then v0.4 records only a lone recordable USB camera — with
// several it cannot tell which one is the dashcam, and must not take a camera
// another system owns.
static RecordChoice resolveRecordCamera(const std::vector<cameraInfo>& cams,
                                        const std::vector<dashcam::config::CameraConfig>& configs,
                                        bool configKnown = true,
                                        const UsbIdentityFn& identity = usbIdentity) {
    RecordChoice rc;
    // A configured device is compared as the node it names NOW: a stable
    // /dev/v4l/by-id|by-path link follows the camera when /dev/videoN changes.
    auto node = [](const dashcam::config::CameraConfig& cfg) {
        return resolveDeviceNode((std::string)cfg.device);
    };
    std::string cabinDev;
    bool        cabinEnabled = true;
    for (const auto& cfg : configs)
        if (cfg.type == "USB" && cfg.name == "cabin") {
            cabinDev     = node(cfg);
            cabinEnabled = (bool)cfg.enabled;
            break;
        }

    auto exactConfig = [&](const cameraInfo& c) -> const dashcam::config::CameraConfig* {
        for (const auto& cfg : configs)
            if (cfg.type == "USB" && !std::string(cfg.device).empty() && node(cfg) == c.address)
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
    auto disabled = [&](const cameraInfo& c) {
        const auto* cfg = exactConfig(c);
        return cfg && !(bool)cfg->enabled;
    };
    auto findUsb = [&](const std::string& dev) -> const cameraInfo* {
        for (const auto& c : cams)
            if (c.type == CAMERA_TYPE::USB && c.address == dev) return &c;
        return nullptr;
    };

    // 0. The dashcam camera, pinned by name: record that camera or nothing, so
    //    a camera another system owns (the cabin driver camera, a rangefinder)
    //    is never taken — not even while the dashcam camera is missing.  The
    //    pin is the first dashcam entry with a Device (or a disabled one: off).
    const dashcam::config::CameraConfig* pin = nullptr;
    int dashcamEntries = 0;
    for (const auto& cfg : configs) {
        if (cfg.type != "USB" || cfg.name != kDashcamEntry) continue;
        ++dashcamEntries;
        if (!pin && (!(bool)cfg.enabled || !node(cfg).empty())) pin = &cfg;
    }
    if (dashcamEntries > 1)
        rc.notes.push_back(std::string("more than one <Camera name=\"") + kDashcamEntry +
                           "\"> entry — using the first one with a <Device>");
    if (!pin && dashcamEntries > 0)
        rc.notes.push_back(std::string("<Camera name=\"") + kDashcamEntry + "\"> has no <Device> — "
                           "choosing a camera automatically (set it to the camera's "
                           "/dev/v4l/by-id/...-video-index0 link to pin it)");
    if (pin) {
        rc.pinned = true;
        const std::string want = trimmed(pin->device);   // as configured, for messages
        const std::string dev  = node(*pin);
        if (!(bool)pin->enabled) {
            rc.why = "dashcam camera disabled in the config (<Enabled>false</Enabled>) — not recording";
            return rc;
        }
        const cameraInfo* c = findUsb(dev);
        if (!c) {
            std::error_code ec;
            rc.why = fs::exists(dev, ec)
                ? "pinned dashcam camera " + want + " (" + dev + ") is not a USB video capture "
                  "device — for a UVC camera use its ...-video-index0 link; not recording"
                : "pinned dashcam camera " + want + " not present — waiting for it (other cameras "
                  "are left alone)";
            return rc;
        }
        // Owned by another entry (the cabin, a rangefinder, or disabled)?
        for (const auto& cfg : configs) {
            if (cfg.type != "USB" || cfg.name == kDashcamEntry || node(cfg) != c->address) continue;
            rc.why = "pinned dashcam camera " + want + " is also configured as '" + cfg.name +
                     "' — not recording";
            return rc;
        }
        // A by-id name is shared by cameras of one model with the same serial.
        if (want.rfind(kByIdPrefix, 0) == 0) {
            const std::string id = identity(c->address);
            for (const auto& o : cams)
                if (!id.empty() && &o != c && o.type == CAMERA_TYPE::USB && identity(o.address) == id) {
                    rc.why = "pinned dashcam camera " + want + " is ambiguous: " + o.address +
                             " is the same model with the same serial — pin it by its "
                             "/dev/v4l/by-path/... link instead; not recording";
                    return rc;
                }
        }
        const int idx = formatFor(pin, *c);
        if (idx < 0) {
            rc.why = "pinned dashcam camera " + want + " offers no MJPEG/H.264 mode — not recording";
            return rc;
        }
        rc.cam = c;
        rc.fmt = idx;
        return rc;
    }

    // No pin: choose automatically (single-camera setups, older configs) —
    // this may take ANY camera; the pin is what protects other systems'.
    if (!configKnown) {
        int recordable = 0;
        for (const auto& c : cams)
            if (c.type == CAMERA_TYPE::USB && pickUsbRecordFormat(c) >= 0) ++recordable;
        if (recordable > 1) {
            rc.why = "dashcam.xml could not be read and " + std::to_string(recordable) +
                     " recordable USB cameras are present — no pin to tell the dashcam camera "
                     "apart; not recording until the config is fixed";
            return rc;
        }
    }
    // 1. Explicit, enabled USB entries of no other role (dashcam entries were
    //    handled above; cabin is the driver camera).
    for (const auto& cfg : configs) {
        if (cfg.type != "USB" || cfg.name == "cabin" || cfg.name == kDashcamEntry ||
            !(bool)cfg.enabled)
            continue;
        const std::string want = trimmed(cfg.device);
        if (want.empty()) continue;
        const std::string dev = node(cfg);
        const cameraInfo* c = findUsb(dev);
        if (!c) {
            rc.notes.push_back("config pin '" + cfg.name + "' -> " + want +
                               " is not a USB capture device right now (stale pin?) — ignored");
            continue;
        }
        if (!cabinDev.empty() && c->address == cabinDev) continue;
        const int idx = formatFor(&cfg, *c);
        if (idx >= 0) { rc.cam = c; rc.fmt = idx; return rc; }
    }

    // 2. First compressed-capable USB camera that is not disabled or the cabin.
    for (const auto& c : cams) {
        if (c.type != CAMERA_TYPE::USB || disabled(c)) continue;
        if (!cabinDev.empty() && c.address == cabinDev) continue;
        const int idx = formatFor(exactConfig(c), c);
        if (idx >= 0) { rc.cam = &c; rc.fmt = idx; return rc; }
    }

    // 3. Last resort: the cabin camera — footage beats no footage.  Never one
    //    the operator disabled (<Enabled>false</Enabled> on the cabin entry, or
    //    a disabled entry for that device).
    auto anyEntryDisables = [&](const std::string& dev) {
        for (const auto& cfg : configs)
            if (cfg.type == "USB" && node(cfg) == dev && !(bool)cfg.enabled) return true;
        return false;
    };
    if (!cabinDev.empty() && cabinEnabled && !anyEntryDisables(cabinDev)) {
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

// ─── session stop: footage and camera, neither waiting on the other ───────────

// Runs restoreCamera on its own thread beside stopRecorder (which has its own
// hard deadline), then waits at most budgetMs more for it.  False = the camera
// part is still stuck; its thread is left running and the caller must exit.
// The footage never waits on a camera control write, and the camera is
// restored even when stopRecorder ends in _exit(3) on a wedged disk.
static bool stopSession(const std::function<void()>& restoreCamera,
                        const std::function<void()>& stopRecorder, int budgetMs) {
    auto done = std::make_shared<std::promise<void>>();
    std::future<void> restored = done->get_future();
    try {
        std::thread([restoreCamera, done] { restoreCamera(); done->set_value(); }).detach();
    } catch (const std::system_error&) {                 // no thread: footage first, then the camera
        stopRecorder();
        restoreCamera();
        return true;
    }
    stopRecorder();
    return restored.wait_for(std::chrono::milliseconds(budgetMs)) == std::future_status::ready;
}

// ─── --self-test: session stop ordering and bounds ────────────────────────────

static int selfTestStopSession() {
    int failures = 0;
    auto check = [&](bool ok, const std::string& what) {
        std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());
        if (!ok) ++failures;
    };
    using clk = std::chrono::steady_clock;
    auto ms = [](clk::time_point a, clk::time_point b) {
        return (long)std::chrono::duration_cast<std::chrono::milliseconds>(b - a).count();
    };
    auto sleep = [](int m) { std::this_thread::sleep_for(std::chrono::milliseconds(m)); };
    std::printf("dashcam_v0_4 self-test (session stop)\n");
    {
        const auto t0 = clk::now();
        const bool ok = stopSession([&] { sleep(300); }, [&] { sleep(300); }, 1000);
        const long el = ms(t0, clk::now());
        check(ok && el < 550, "camera restore and recorder stop run side by side (" + std::to_string(el) + " ms, not 600)");
    }
    {
        // A wedged camera: its control write would block for seconds.
        const auto t0 = clk::now();
        clk::time_point recStart{};
        const bool ok = stopSession([] { std::this_thread::sleep_for(std::chrono::seconds(3)); },
                                    [&] { recStart = clk::now(); sleep(100); }, 200);
        const long el = ms(t0, clk::now());
        check(ms(t0, recStart) < 50, "footage finalising starts at once, not after the camera (" +
              std::to_string(ms(t0, recStart)) + " ms)");
        check(!ok && el < 600, "stuck camera reported within recorder stop + budget (" + std::to_string(el) + " ms)");
    }
    {
        // A slow finalise (EOS) gives the camera time to answer.
        const bool ok = stopSession([&] { sleep(300); }, [&] { sleep(500); }, 50);
        check(ok, "camera restored during a slow finalise: no extra wait");
    }
    return failures;
}

// ─── --self-test: resolver invariants, no hardware ────────────────────────────

static int selfTestCameras() {
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
    {
        const std::vector<cameraInfo> cams{usb("/dev/video0", {mj1080})};
        const auto rc = resolveRecordCamera(cams, {pin("cabin", "/dev/video0", false)});
        check(rc.cam == nullptr, "disabled lone cabin camera is NOT recorded");
    }
    {
        const std::vector<cameraInfo> cams{usb("/dev/video0", {mj1080}), usb("/dev/video2", {yuyv})};
        const auto rc = resolveRecordCamera(cams, {pin("cabin", "/dev/video0", false)});
        check(rc.cam == nullptr, "disabled cabin not recorded even when the other camera is raw-only");
    }
    {
        // Cabin entry enabled, but a separate entry disables the same device.
        const std::vector<cameraInfo> cams{usb("/dev/video0", {mj1080})};
        const auto rc = resolveRecordCamera(cams, {pin("off", "/dev/video0", false), pin("cabin", "/dev/video0")});
        check(rc.cam == nullptr, "cabin device disabled by another entry is NOT recorded");
        const auto rc2 = resolveRecordCamera(cams, {pin("cabin", "/dev/video0"), pin("off", "/dev/video0", false)});
        check(rc2.cam == nullptr, "... whichever order the entries are in");
    }

    // ── the dashcam pin: <Camera name="dashcam"> = that camera or nothing ──
    auto has = [](const RecordChoice& rc, const char* what) {
        return rc.why.find(what) != std::string::npos;
    };
    {
        const std::vector<cameraInfo> cams{usb("/dev/video0", {mj1080}), usb("/dev/video2", {mj1080})};
        const auto rc = resolveRecordCamera(cams, {pin("dashcam", "/dev/video2")});
        check(rc.cam == &cams[1] && rc.pinned, "dashcam pin: records exactly that camera");
    }
    const char* absent = "/dev/v04-selftest-absent-video0";   // exists on no host
    {
        // Missing, while a recordable spare and a pinned cabin camera are present.
        const std::vector<cameraInfo> cams{usb("/dev/video0", {mj1080}), usb("/dev/video4", {mj1080})};
        const auto rc = resolveRecordCamera(cams, {pin("cabin", "/dev/video4"), pin("dashcam", absent)});
        check(rc.cam == nullptr && rc.pinned && has(rc, "not present"),
              "dashcam pin missing: waits — never records a spare camera");
        const std::vector<cameraInfo> cabinOnly{usb("/dev/video4", {mj1080})};
        const auto rc2 = resolveRecordCamera(cabinOnly, {pin("cabin", "/dev/video4"), pin("dashcam", absent)});
        check(rc2.cam == nullptr && !rc2.cabinFallback && has(rc2, "not present"),
              "dashcam pin missing, only the cabin camera present: still waits (no cabin fallback)");
    }
    {
        // Another system's entry — or a disabled one — on the pinned device.
        const std::vector<cameraInfo> cams{usb("/dev/video0", {mj1080}), usb("/dev/video2", {mj1080})};
        const auto rc = resolveRecordCamera(cams, {pin("rangefinder", "/dev/video2"), pin("dashcam", "/dev/video2")});
        check(rc.cam == nullptr && has(rc, "'rangefinder'"), "dashcam pin on a rangefinder's device: refused");
        const auto rc2 = resolveRecordCamera(cams, {pin("dashcam", "/dev/video2"), pin("spare", "/dev/video2", false)});
        check(rc2.cam == nullptr && has(rc2, "'spare'"), "dashcam pin on a device another entry disables: refused");
    }
    {
        // An empty leftover dashcam entry above the real one.
        const std::vector<cameraInfo> cams{usb("/dev/video0", {mj1080}), usb("/dev/video2", {mj1080})};
        const auto rc = resolveRecordCamera(cams, {pin("dashcam", ""), pin("dashcam", "/dev/video2")});
        check(rc.cam == &cams[1] && rc.pinned, "empty dashcam entry first: the one with a Device is the pin");
        const auto rc2 = resolveRecordCamera(cams, {pin("dashcam", ""), pin("dashcam", absent)});
        check(rc2.cam == nullptr && has(rc2, "not present"), "... and it still waits when missing (no fallback)");
        const auto rc3 = resolveRecordCamera(cams, {pin("dashcam", "", false)});
        check(rc3.cam == nullptr && has(rc3, "disabled"), "disabled dashcam entry without a Device: recording off");
        const auto rc4 = resolveRecordCamera(cams, {pin("dashcam", "\n        /dev/video2\n      ")});
        check(rc4.cam == &cams[1], "Device text wrapped onto its own line: whitespace ignored");
    }
    {
        // Two cameras of one model with the same serial share a by-id name.
        const std::string byId = std::string(kByIdPrefix) + "usb-Same_Model-0000000001-video-index0";
        const std::vector<cameraInfo> cams{usb(byId.c_str(), {mj1080}), usb("/dev/video2", {mj1080})};
        auto same = [](const std::string&) { return std::string("eba4:6579:0000000001"); };
        auto diff = [](const std::string& n) { return std::string("eba4:6579:") + n; };
        const auto rc = resolveRecordCamera(cams, {pin("dashcam", byId.c_str())}, true, same);
        check(rc.cam == nullptr && has(rc, "ambiguous") && has(rc, "by-path"),
              "by-id pin shared by two identical cameras: refused, pointing to by-path");
        const auto rc2 = resolveRecordCamera(cams, {pin("dashcam", byId.c_str())}, true, diff);
        check(rc2.cam == &cams[0], "by-id pin with a unique camera identity: recorded");
    }
    {
        // dashcam.xml exists but cannot be read: no pin is known.
        const std::vector<cameraInfo> two{usb("/dev/video0", {mj1080}), usb("/dev/video2", {mj1080})};
        const auto rc = resolveRecordCamera(two, {}, false);
        check(rc.cam == nullptr && has(rc, "could not be read"),
              "unreadable config, two recordable cameras: waits (cannot tell the dashcam apart)");
        const std::vector<cameraInfo> one{usb("/dev/video0", {mj1080}), usb("/dev/video2", {yuyv})};
        check(resolveRecordCamera(one, {}, false).cam == &one[0],
              "unreadable config, one recordable camera: recorded");
    }
    {
        const std::vector<cameraInfo> cams{usb("/dev/video0", {mj1080})};
        const auto rc = resolveRecordCamera(cams, {pin("dashcam", "/dev/video0", false)});
        check(rc.cam == nullptr && has(rc, "disabled"), "dashcam pin disabled: recording off, with the reason");
        const auto rc2 = resolveRecordCamera(cams, {pin("cabin", "/dev/video0"), pin("dashcam", "/dev/video0")});
        check(rc2.cam == nullptr && has(rc2, "cabin"), "dashcam pin on the cabin device: refused");
    }
    {
        const std::vector<cameraInfo> cams{usb("/dev/video0", {mj4k, mj1080})};
        const auto rc = resolveRecordCamera(cams, {pin("dashcam", "/dev/video0", true, 1)});
        check(rc.cam == &cams[0] && rc.fmt == 1, "dashcam pin: its FormatIndex picks the mode");
        const std::vector<cameraInfo> raw{usb("/dev/video0", {yuyv})};
        check(has(resolveRecordCamera(raw, {pin("dashcam", "/dev/video0")}), "no MJPEG/H.264"),
              "dashcam pin on a raw-only camera: refused, with the reason");
    }
    {
        const std::vector<cameraInfo> cams{usb("/dev/video0", {mj1080})};
        const auto rc = resolveRecordCamera(cams, {pin("dashcam", "")});
        check(rc.cam == &cams[0] && !rc.pinned && !rc.notes.empty(),
              "dashcam entry without a Device: automatic choice, with a note");
        const auto rc2 = resolveRecordCamera(cams, {pin("dashcam", "/dev/video0"), pin("dashcam", "/dev/video7")});
        check(rc2.cam == &cams[0] && !rc2.notes.empty(), "two dashcam entries: the first wins, with a note");
    }
    {
        // Stable links: the pin names a by-id style link, not /dev/videoN.
        char td[] = "/tmp/v04_selftest_pin_XXXXXX";
        if (!::mkdtemp(td)) { check(false, "pin temp dir"); return failures; }
        const std::string dir = td, link = dir + "/usb-Cam-video-index0";
        for (const char* f : {"/video7", "/video9", "/meta"}) std::ofstream(dir + f).put('x');
        const std::vector<cameraInfo> cams{usb((dir + "/video7").c_str(), {mj1080}),
                                           usb((dir + "/video9").c_str(), {mj1080})};
        std::error_code ec;
        fs::create_symlink("video7", link, ec);
        const auto a = resolveRecordCamera(cams, {pin("dashcam", link.c_str())});
        fs::remove(link, ec);
        fs::create_symlink("video9", link, ec);                 // replugged: now /dev/video9
        const auto b = resolveRecordCamera(cams, {pin("dashcam", link.c_str())});
        fs::remove(link, ec);
        fs::create_symlink("meta", link, ec);                   // e.g. the ...-index1 metadata node
        const auto c = resolveRecordCamera(cams, {pin("dashcam", link.c_str())});
        fs::remove(link, ec);                                   // unplugged: no link at all
        const auto d = resolveRecordCamera(cams, {pin("dashcam", link.c_str())});
        check(a.cam == &cams[0], "stable-link pin resolves to the camera's current node");
        check(b.cam == &cams[1], "... and follows it when the node number changes");
        check(c.cam == nullptr && has(c, "not a USB video capture"), "link to a non-capture node: refused, with the reason");
        check(d.cam == nullptr && has(d, "not present"), "link gone (camera unplugged): waits");
        fs::remove_all(dir, ec);
    }
    return failures;
}

// ─── storage: preferred footage dir or the fixed fallback, single-instance lock ─

class Storage {
public:
    using ProbeFn = std::function<int(const std::string&)>;

    Storage(std::string preferred, rec::RetentionPolicy policy, dashcam::log::LogCallback log,
            std::string fallback = fallbackFootageDir(), ProbeFn probe = probeDir)
        : preferred_(std::move(preferred)), fallback_(std::move(fallback)),
          policy_(std::move(policy)), log_(std::move(log)), probe_(std::move(probe)) {}
    ~Storage() { release(); }

    /**
     * Pick and lock the directory to record into: the preferred one, else the
     * fallback.  "" (with @p why) when neither is usable.
     *
     * Must be called while this process is NOT recording.  A full directory
     * (ENOSPC) is cleaned up only AFTER its lock is held: the lock is what
     * guarantees no other dashcam_v0_4 is writing its open segment there, and
     * our own recorder is stopped — so every owned segment in it is closed.
     */
    std::string acquire(std::string& why) {
        std::string reasons, preferredWhy;
        for (const std::string* dir : {&preferred_, &fallback_}) {
            std::string whyNot;
            if (claim(*dir, whyNot)) {
                const bool fallback = (*dir != preferred_);
                if (fallback && !onFallback_)
                    log_(LogLevel::WARN, "footage directory " + preferred_ + " unusable (" + preferredWhy +
                                         ") — recording to " + fallback_);
                onFallback_ = fallback;
                return dir_;
            }
            if (*dir == preferred_) preferredWhy = whyNot;
            reasons += (reasons.empty() ? "" : "; ") + *dir + ": " + whyNot;
        }
        why = "no usable footage directory (" + reasons + ")";
        return "";
    }

    /**
     * On the fallback: true when the preferred directory takes writes again and
     * no other dashcam_v0_4 holds it.  A full preferred directory is cleaned up
     * here only under a temporary hold of its lock (we record on the fallback,
     * so all our segments there are closed); a directory locked by another
     * instance is never touched and does not count as "back" — otherwise every
     * probe would restart our recording for nothing.
     */
    bool preferredBack() {
        if (!onFallback_) return false;
        int err = probe_(preferred_);
        if (err != 0 && err != ENOSPC) return false;
        std::string whyNot;
        const int fd = lockDir(preferred_, whyNot);
        if (fd < 0) return false;
        if (err == ENOSPC) {
            retainIn(preferred_, UINT64_MAX);
            err = probe_(preferred_);
        }
        ::close(fd);                              // acquire() re-takes it
        return err == 0;
    }

    const std::string& dir() const { return dir_; }

    /// Retention in the held directory; @p watermark protects the open segments.
    rec::RetentionResult retain(uint64_t watermark) { return retainIn(dir_, watermark); }

    /// The same pass as a self-contained job (copies only — no reference to
    /// this object), for the retention thread: a directory scan and deletes
    /// can block on a failing disk and must not hold up the supervisor.  The
    /// job keeps the directory LOCKED until it ends — a dup of the lock fd
    /// (flock belongs to the open file description), so even after release()
    /// no other dashcam_v0_4, and no later claim of ours, can start recording
    /// under a pass that may delete anything it scans.  Empty when no
    /// directory is held.
    std::function<rec::RetentionResult()> retentionPass(uint64_t watermark) const {
        if (lockFd_ < 0) return {};
        const int held = ::fcntl(lockFd_, F_DUPFD_CLOEXEC, 0);
        if (held < 0) return {};
        auto hold = std::shared_ptr<int>(new int(held), [](int* fd) { ::close(*fd); delete fd; });
        rec::RetentionPolicy p = policy_;
        p.dir = dir_;
        return [p, watermark, log = log_, hold] {
            return rec::enforceRetention(p, watermark, [] { return g_run != 0; }, log);
        };
    }

    void release() {
        if (lockFd_ >= 0) { ::close(lockFd_); lockFd_ = -1; }   // closing drops the flock
        dir_.clear();
    }

private:
    std::string               preferred_, fallback_, dir_;
    rec::RetentionPolicy      policy_;
    dashcam::log::LogCallback log_;
    ProbeFn                   probe_;
    int                       lockFd_ = -1;
    bool                      onFallback_ = false;

    rec::RetentionResult retainIn(const std::string& dir, uint64_t watermark) {
        rec::RetentionPolicy p = policy_;
        p.dir = dir;
        return rec::enforceRetention(p, watermark, [] { return g_run != 0; }, log_);
    }

    // Lock @p dir (unless already held) and make sure it takes a write.  Never
    // deletes anything in a directory whose lock it does not hold.
    bool claim(const std::string& dir, std::string& whyNot) {
        int err = probe_(dir);
        if (err != 0 && err != ENOSPC) {
            whyNot = err == ETIMEDOUT ? "not answering (write probe still pending)" : std::strerror(err);
            return false;
        }
        int fd = -1;
        if (dir != dir_) {
            fd = lockDir(dir, whyNot);             // keep the current lock until this one works
            if (fd < 0) return false;
        }
        if (err == ENOSPC) {
            retainIn(dir, UINT64_MAX);            // lock held, our recorder stopped: all closed
            err = probe_(dir);
            if (err != 0) {
                whyNot = std::string("still unwritable after cleanup: ") + std::strerror(err);
                if (fd >= 0) ::close(fd);
                return false;
            }
        }
        if (fd >= 0) {
            release();
            lockFd_ = fd;
            dir_    = dir;
        }
        return true;
    }

    // A new descriptor holding @p dir's flock; -1 (with @p whyNot) if another
    // dashcam_v0_4 holds it or it cannot be opened.  flock conflicts between
    // separate opens even within one process, so a second lock on the directory
    // we already hold would fail — callers check dir_ first.
    static int lockDir(const std::string& dir, std::string& whyNot) {
        const std::string path = dir + "/" + kLockName;
        const int fd = ::open(path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0644);
        if (fd < 0) { whyNot = "cannot open " + path + ": " + std::strerror(errno); return -1; }
        if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
            whyNot = errno == EWOULDBLOCK
                         ? "locked: another dashcam_v0_4 records into " + dir +
                               ", or a cleanup pass of ours still runs there"
                         : "cannot lock " + path + ": " + std::strerror(errno);
            ::close(fd);
            return -1;
        }
        return fd;
    }
};

// ─── --self-test: storage never deletes in a directory it does not own ────────

static int selfTestStorage() {
    int failures = 0;
    auto check = [&](bool ok, const char* what) {
        std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
        if (!ok) ++failures;
    };
    std::printf("dashcam_v0_4 self-test (storage)\n");
    char ta[] = "/tmp/v04_selftest_pref_XXXXXX";
    char tb[] = "/tmp/v04_selftest_fb_XXXXXX";
    if (!::mkdtemp(ta) || !::mkdtemp(tb)) { check(false, "temp dirs"); return failures; }
    const std::string A = ta, B = tb;
    const std::string seg = A + "/dashcam_000001_20260101_000000.mkv";   // "another recorder's open segment"
    auto writeSeg = [&] { std::FILE* f = std::fopen(seg.c_str(), "w"); if (f) { std::fputs("frames", f); std::fclose(f); } };
    // A is "full" (ENOSPC) while that segment exists; deleting it frees space.
    auto probe = [&](const std::string& d) { return d == A && fs::exists(seg) ? ENOSPC : probeDir(d); };
    rec::RetentionPolicy pol;
    pol.prefix   = kPrefix;
    pol.maxBytes = 1;                                   // any owned byte is over quota
    auto quiet = [](LogLevel, const std::string&) {};

    writeSeg();
    const int other = ::open((A + "/" + kLockName).c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0644);
    const bool held = other >= 0 && ::flock(other, LOCK_EX | LOCK_NB) == 0;
    check(held, "a second instance holds the preferred directory");
    {
        Storage st(A, pol, quiet, B, probe);
        std::string why;
        check(st.acquire(why) == B, "full preferred held by another instance: falls back");
        check(fs::exists(seg), "... and deletes nothing there (the other recorder's segment survives)");
        check(!st.preferredBack() && fs::exists(seg),
              "preferredBack() while it is held: not back, nothing deleted");
        if (other >= 0) ::close(other);                 // the other instance exits
        check(st.preferredBack(), "preferred back once free (cleaned under its own lock)");
        check(!fs::exists(seg), "... owned segment deleted only then");
        st.release();
        check(st.acquire(why) == A, "acquire then takes the preferred directory");
    }
    writeSeg();
    {
        Storage st(A, pol, quiet, B, probe);
        std::string why;
        check(st.acquire(why) == A && !fs::exists(seg), "full preferred with a free lock: cleaned under the lock, used");
    }
    {
        Storage s1(A, pol, quiet, B), s2(A, pol, quiet, B);
        std::string why;
        check(s1.acquire(why) == A && s2.acquire(why) == B, "two instances never share a directory");
    }
    {
        Storage st("/proc/v04_selftest_no", pol, quiet, "/proc/v04_selftest_no2");
        std::string why;
        check(st.acquire(why).empty() && !why.empty(), "nothing usable: empty result with a reason");
    }
    std::error_code ec;
    fs::remove_all(A, ec);
    fs::remove_all(B, ec);
    return failures;
}

// ─── retention thread: deletes never block the supervisor ─────────────────────

// Runs retention passes one at a time on its own thread; the supervisor only
// posts a pass and collects its result.  A pass still running when the next
// is due is not queued again.  At shutdown a pass stuck on a dead disk is left
// behind (detached) instead of blocking the exit — passes hold only copies.
//
// NEVER start a session in a directory while a pass there runs (idleFor): a
// pass posted while no session ran has an unlimited watermark and may delete
// ANY segment it scans — including the one a new session just opened.  And a
// bounded watermark is no substitute: a pass that deletes every segment resets
// the numbering (next SEQ = highest existing + 1) below its own watermark.
class RetentionRunner {
public:
    using Pass = std::function<rec::RetentionResult()>;
    RetentionRunner() : thread_([this] { loop(); }) {}
    ~RetentionRunner() { stop(); }
    RetentionRunner(const RetentionRunner&)            = delete;
    RetentionRunner& operator=(const RetentionRunner&) = delete;

    /// Start @p pass over @p dir; false (nothing queued) while a previous pass
    /// still runs.
    bool post(Pass pass, const std::string& dir) {
        std::lock_guard<std::mutex> lk(sh_->m);
        if (sh_->busy || sh_->stop) return false;
        sh_->busy      = true;
        sh_->since     = std::chrono::steady_clock::now();
        sh_->dir       = dir;
        sh_->job       = std::move(pass);
        sh_->cv.notify_one();
        return true;
    }
    /// Wait at most @p ms until no pass over @p dir is running; true then.
    bool idleFor(const std::string& dir, int ms) {
        std::unique_lock<std::mutex> lk(sh_->m);
        return sh_->cv.wait_for(lk, std::chrono::milliseconds(ms),
                                [&] { return !sh_->busy || sh_->dir != dir; });
    }
    /// A finished pass's result, once.
    bool poll(rec::RetentionResult& out) {
        std::lock_guard<std::mutex> lk(sh_->m);
        if (!sh_->hasResult) return false;
        out            = sh_->result;
        sh_->hasResult = false;
        return true;
    }
    /// How long the current pass has run (0 when idle).
    int64_t busyMs() const {
        std::lock_guard<std::mutex> lk(sh_->m);
        return sh_->busy ? std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now() - sh_->since).count()
                         : 0;
    }
    /// Wait at most @p ms for the current pass to finish; true when idle.
    bool waitIdle(int ms) {
        std::unique_lock<std::mutex> lk(sh_->m);
        return sh_->cv.wait_for(lk, std::chrono::milliseconds(ms), [&] { return !sh_->busy; });
    }
    void stop() {
        if (!thread_.joinable()) return;
        bool busy;
        {
            std::lock_guard<std::mutex> lk(sh_->m);
            sh_->stop = true;
            busy      = sh_->busy;
            sh_->cv.notify_all();
        }
        if (busy && !waitIdle(1000)) thread_.detach();    // stuck on a dead disk: leave it
        else thread_.join();
    }

private:
    struct Shared {                                        // shared with a detached thread
        mutable std::mutex                    m;
        std::condition_variable               cv;
        bool                                  stop = false, busy = false, hasResult = false;
        std::chrono::steady_clock::time_point since{};
        std::string                           dir;         ///< Directory of the running pass.
        Pass                                  job;
        rec::RetentionResult                  result{};
    };
    std::shared_ptr<Shared> sh_ = std::make_shared<Shared>();
    std::thread             thread_;

    void loop() {
        auto sh = sh_;                                     // keep it alive even if detached
        std::unique_lock<std::mutex> lk(sh->m);
        for (;;) {
            sh->cv.wait(lk, [&] { return sh->stop || (sh->busy && sh->job); });
            if (sh->stop && !sh->job) return;
            Pass job = std::move(sh->job);
            sh->job  = nullptr;
            lk.unlock();
            const rec::RetentionResult r = job();
            job = nullptr;                                 // releases the directory lock it held
            lk.lock();
            sh->result    = r;
            sh->hasResult = true;
            sh->busy      = false;
            sh->cv.notify_all();
            if (sh->stop) return;
        }
    }
};

// One RetentionRunner per directory, so a pass stuck on one disk (say the
// preferred, gone dead) never starves retention on the other (the fallback).
// Supervisor thread only.
class Retention {
public:
    bool post(const std::string& dir, RetentionRunner::Pass pass) {
        if (!pass) return false;
        auto& r = runners_[dir];
        if (!r) r = std::make_unique<RetentionRunner>();
        return r->post(std::move(pass), dir);
    }
    /// Wait at most @p ms until no pass over @p dir runs; true then.
    bool idleFor(const std::string& dir, int ms) {
        const auto it = runners_.find(dir);
        return it == runners_.end() || it->second->idleFor(dir, ms);
    }
    bool poll(rec::RetentionResult& out) {
        for (auto& [dir, r] : runners_)
            if (r->poll(out)) return true;
        return false;
    }
    int64_t busyMs() const {
        int64_t ms = 0;
        for (const auto& [dir, r] : runners_) ms = std::max(ms, r->busyMs());
        return ms;
    }
    void stop() {
        for (auto& [dir, r] : runners_) r->stop();
    }

private:
    std::map<std::string, std::unique_ptr<RetentionRunner>> runners_;
};

// Where a new session may record — never under a running cleanup pass (see
// RetentionRunner): the directory acquire() picks, once no pass runs there
// (beforeStart may post one, e.g. when space is low); else, after waitMs, the
// other directory — the pass keeps its directory locked, so acquire() moves
// on.  Recording elsewhere beats recording nowhere; the preferred directory
// is taken back once it is free.  "" (with why) = nowhere right now.
static std::string pickStartDir(Storage& storage, Retention& retention, int waitMs,
                                const std::function<void(const std::string&)>& beforeStart,
                                std::string& why) {
    std::string dir = storage.acquire(why);
    if (dir.empty()) return dir;
    if (beforeStart) beforeStart(dir);
    if (retention.idleFor(dir, waitMs)) return dir;
    const std::string busy = dir;
    storage.release();
    std::string whyNot;
    dir = storage.acquire(whyNot);
    if (!dir.empty() && dir != busy && retention.idleFor(dir, 0)) return dir;
    storage.release();
    why = "cleanup pass still running in " + busy + " and no other footage directory is usable" +
          (whyNot.empty() ? std::string() : " (" + whyNot + ")");
    return "";
}

// ─── supervisor heartbeat: the last line of recovery ─────────────────────────

// Everything the supervisor waits on is bounded (recorder.stop() by its own
// deadline, the camera hand-back by kCameraRestoreMs, directory probes by
// kProbeTimeoutMs; retention runs on its own thread) — but anything left that
// blocks it (a V4L2 ioctl on a wedged device, a filesystem call not yet
// bounded, a blocking log fallback) would leave a dead recording unrecovered.
// This thread does no I/O: if the loop stops beating for limitMs it calls
// onStuck (in v0.4: emergencyExit, so the launcher's restart loop takes over).
class SupervisorWatchdog {
public:
    using StuckFn = std::function<void(int64_t silentMs)>;
    SupervisorWatchdog(int limitMs, StuckFn onStuck)
        : limitMs_(limitMs), onStuck_(std::move(onStuck)) {
        beat();
        thread_ = std::thread([this] { loop(); });
    }
    ~SupervisorWatchdog() { stop(); }
    SupervisorWatchdog(const SupervisorWatchdog&)            = delete;
    SupervisorWatchdog& operator=(const SupervisorWatchdog&) = delete;

    void beat() {
        beatNs_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    }
    void stop() {
        {
            std::lock_guard<std::mutex> lk(m_);
            stop_ = true;
        }
        cv_.notify_all();
        if (thread_.joinable()) thread_.join();
    }

private:
    const int               limitMs_;
    StuckFn                 onStuck_;
    std::atomic<int64_t>    beatNs_{0};
    std::mutex              m_;
    std::condition_variable cv_;
    bool                    stop_ = false;
    std::thread             thread_;

    void loop() {
        const auto tick = std::chrono::milliseconds(std::max(50, std::min(500, limitMs_ / 8)));
        std::unique_lock<std::mutex> lk(m_);
        while (!cv_.wait_for(lk, tick, [&] { return stop_; })) {
            const int64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            const int64_t silentMs = (now - beatNs_.load()) / 1000000;
            if (silentMs < limitMs_) continue;
            lk.unlock();
            onStuck_(silentMs);                            // v0.4: does not return
            return;
        }
    }
};

// ─── --self-test: nothing the supervisor does may block it ───────────────────

static int selfTestSupervision() {
    int failures = 0;
    auto check = [&](bool ok, const std::string& what) {
        std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());
        if (!ok) ++failures;
    };
    using clk = std::chrono::steady_clock;
    auto msSince = [](clk::time_point t0) {
        return (long)std::chrono::duration_cast<std::chrono::milliseconds>(clk::now() - t0).count();
    };
    std::printf("dashcam_v0_4 self-test (supervision)\n");
    {
        // A probe that blocks until released: a filesystem that stopped answering.
        auto gate = std::make_shared<std::atomic<bool>>(false);
        BoundedProbe probe([gate](const std::string&) {
            while (!gate->load()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
            return 0;
        }, 200);
        auto t0 = clk::now();
        const int e1 = probe("/stuck");
        const long ms1 = msSince(t0);
        check(e1 == ETIMEDOUT && ms1 >= 190 && ms1 < 600, "stuck probe times out (" + std::to_string(ms1) + " ms)");
        t0 = clk::now();
        const int e2 = probe("/stuck");
        check(e2 == ETIMEDOUT && msSince(t0) < 50, "while it is still stuck, the next probe fails at once");
        gate->store(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        check(probe("/stuck") == 0, "once the filesystem answers, probes work again");
    }
    {
        // Storage with a preferred directory that stopped answering: falls back.
        char ta[] = "/tmp/v04_selftest_stuckpref_XXXXXX";
        char tb[] = "/tmp/v04_selftest_stuckfb_XXXXXX";
        if (::mkdtemp(ta) && ::mkdtemp(tb)) {
            const std::string A = ta, B = tb;
            auto gate = std::make_shared<std::atomic<bool>>(false);
            BoundedProbe probe([gate, A](const std::string& d) {
                if (d == A) while (!gate->load()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
                return probeDir(d);
            }, 300);
            rec::RetentionPolicy pol;
            pol.prefix = kPrefix;
            Storage st(A, pol, [](LogLevel, const std::string&) {}, B, probe);
            std::string why;
            const auto t0 = clk::now();
            const std::string got = st.acquire(why);
            check(got == B && msSince(t0) < 1500, "preferred directory not answering: fallback within the probe deadline");
            check(!st.preferredBack(), "... and it is not 'back' while its probe is stuck");
            gate->store(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            check(st.preferredBack(), "... until it answers again");
            st.release();
            std::error_code ec;
            fs::remove_all(A, ec);
            fs::remove_all(B, ec);
        } else {
            check(false, "temp dirs");
        }
    }
    {
        auto gate = std::make_shared<std::atomic<bool>>(false);
        RetentionRunner rr;
        auto t0 = clk::now();
        const bool posted = rr.post([gate] {
            while (!gate->load()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
            rec::RetentionResult r{};
            r.deleted = 3;
            return r;
        }, "/footage");
        check(posted && msSince(t0) < 50, "retention pass posted without waiting for it");
        check(!rr.post([] { return rec::RetentionResult{}; }, "/footage"), "a second pass is not queued while one runs");
        rec::RetentionResult res{};
        check(!rr.poll(res) && !rr.waitIdle(100), "no result while the pass is stuck");
        t0 = clk::now();
        check(!rr.idleFor("/footage", 150) && msSince(t0) >= 140,
              "no session may start in the pass's directory while it runs (waited, then refused)");
        check(rr.idleFor("/fallback", 150), "... another directory (e.g. the fallback) is free at once");
        gate->store(true);
        check(rr.waitIdle(1000) && rr.poll(res) && res.deleted == 3 && !rr.poll(res),
              "its result is collected once when it finishes");
        auto gate2 = std::make_shared<std::atomic<bool>>(false);
        rr.post([gate2] {
            while (!gate2->load()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
            return rec::RetentionResult{};
        }, "/footage");
        t0 = clk::now();
        rr.stop();                                           // pass stuck on a dead disk
        check(msSince(t0) < 1500, "shutdown does not wait for a stuck pass");
        gate2->store(true);
    }
    {
        // The hazard idleFor() guards against: a pass posted while no session
        // runs (unlimited watermark, as deletableBelowSeq() of a stopped
        // recorder) that scans after a new segment appeared deletes it.
        char td[] = "/tmp/v04_selftest_race_XXXXXX";
        if (::mkdtemp(td)) {
            const std::string dir = td;
            auto seg = [&](uint64_t seq) {
                return dir + "/" + rec::segmentFileName(kPrefix, seq, 1790000000 + (time_t)seq);
            };
            for (uint64_t q = 1; q <= 3; ++q) std::ofstream(seg(q)) << std::string(4096, 'x');
            rec::RetentionPolicy pol;
            pol.dir      = dir;
            pol.prefix   = kPrefix;
            pol.maxBytes = 1;                                // over quota: delete what it may
            auto gate = std::make_shared<std::atomic<bool>>(false);
            RetentionRunner rr;
            rr.post([gate, pol] {
                while (!gate->load()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
                return rec::enforceRetention(pol, UINT64_MAX, [] { return true; }, nullptr);
            }, dir);
            check(!rr.idleFor(dir, 100), "cleanup running where a session would start: start deferred");
            std::ofstream(seg(4)) << "open segment";         // had the session started anyway...
            gate->store(true);
            rr.waitIdle(2000);
            check(!fs::exists(seg(4)), "... its new segment would have been deleted (why the start must wait)");
            check(rr.idleFor(dir, 100), "cleanup finished: the start may go ahead");
            std::error_code ec;
            fs::remove_all(dir, ec);
        } else {
            check(false, "race temp dir");
        }
    }
    {
        // tryStart's decision (pickStartDir) with a cleanup pass that outlives
        // its deadline (waitMs stands in for the 3 s): no session starts under
        // it — the recording goes to the other directory — and the busy
        // directory stays locked against everyone until the pass ends.
        char ta[] = "/tmp/v04_selftest_gate_pref_XXXXXX";
        char tb[] = "/tmp/v04_selftest_gate_fb_XXXXXX";
        if (::mkdtemp(ta) && ::mkdtemp(tb)) {
            const std::string A = ta, B = tb;
            for (uint64_t q = 1; q <= 3; ++q)
                std::ofstream(A + "/" + rec::segmentFileName(kPrefix, q, 1790000000 + (time_t)q)) << "old";
            rec::RetentionPolicy pol;
            pol.prefix   = kPrefix;
            pol.maxBytes = 1;                                  // wipe what it may
            auto noLog = [](LogLevel, const std::string&) {};
            Storage   st(A, pol, noLog, B, probeDir);
            Retention ret;
            auto gate = std::make_shared<std::atomic<bool>>(false);
            std::string why;
            const auto t0 = clk::now();
            const std::string d1 = pickStartDir(st, ret, 200, [&](const std::string& d) {
                // the pre-start pass of a stopped recorder: unlimited watermark
                auto pass = st.retentionPass(UINT64_MAX);
                ret.post(d, [gate, pass] {
                    while (!gate->load()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    return pass();
                });
            }, why);
            check(d1 == B && msSince(t0) < 1500,
                  "cleanup outlives its deadline: the session starts in the other directory, not under it");
            check(!st.preferredBack(), "... the busy directory is not 'back' while its pass runs");
            {
                std::string whyNot;
                Storage other(A, pol, noLog, "/proc/v04_selftest_unwritable", probeDir);
                check(other.acquire(whyNot).empty(), "... and no other instance can take it (the pass holds its lock)");
            }
            gate->store(true);
            check(ret.idleFor(A, 2000) && st.preferredBack(), "pass done: the preferred directory is back");
            st.release();
            // Nowhere else to go: no start at all (then retry), never under the pass.
            auto gate2 = std::make_shared<std::atomic<bool>>(false);
            Storage lone(A, pol, noLog, "/proc/v04_selftest_unwritable", probeDir);
            std::string why2;
            const std::string d2 = pickStartDir(lone, ret, 200, [&](const std::string& d) {
                auto pass = lone.retentionPass(UINT64_MAX);
                ret.post(d, [gate2, pass] {
                    while (!gate2->load()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    return pass();
                });
            }, why2);
            check(d2.empty() && why2.find("cleanup pass still running") != std::string::npos,
                  "no other directory usable: no start, retry later");
            gate2->store(true);
            ret.idleFor(A, 2000);
            const std::string d3 = pickStartDir(lone, ret, 200, nullptr, why2);
            check(d3 == A, "once the pass is done the start goes ahead there");
            lone.release();
            ret.stop();
            std::error_code ec;
            fs::remove_all(A, ec);
            fs::remove_all(B, ec);
        } else {
            check(false, "gate temp dirs");
        }
    }
    {
        std::atomic<int> fired{0};
        std::atomic<int64_t> silent{0};
        SupervisorWatchdog wd(300, [&](int64_t ms) { silent = ms; ++fired; });
        for (int i = 0; i < 12; ++i) { wd.beat(); std::this_thread::sleep_for(std::chrono::milliseconds(50)); }
        check(fired == 0, "a beating supervisor is left alone");
        std::this_thread::sleep_for(std::chrono::milliseconds(600));
        check(fired == 1 && silent >= 300, "a silent supervisor is caught once (" + std::to_string(silent) + " ms)");
        wd.stop();
    }
    return failures;
}

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
    if (argc == 2 && std::string(argv[1]) == "--self-test") {
        const int failures = selfTestCameras() + selfTestStorage() + selfTestStopSession() +
                             selfTestSupervision();
        std::printf("RESULT: %s (%d failure%s)\n", failures ? "FAIL" : "PASS", failures,
                    failures == 1 ? "" : "s");
        return failures ? 1 : 0;
    }

    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);
    gst_init(&argc, &argv);

    // Config first (so <Log> can shape the logger); stderr until init.
    auto log = dashcam::log::getCallback();
    dashcam::config::AppConfig cfg;
    const std::string configsDir = dashcam::config::resolveStorageDir(
        dashcam::config::kDefaultConfigsDir, dashcam::config::kFallbackConfigsName, log);
    const bool configOk =
        dashcam::config::ConfigReader::loadOrCreate(configsDir + "/dashcam.xml", cfg, log);
    if (!configOk) log(LogLevel::WARN, "config load/create failed; using defaults");

    const std::string logDir = dashcam::config::resolveStorageDir(
        dashcam::config::kDefaultLogDir, dashcam::config::kFallbackLogName, log);
    dashcam::log::init(logDir, makeLogParams(cfg.log));
    if (!configOk)          // said again now that the log file exists
        log(LogLevel::ERROR, "config " + configsDir + "/dashcam.xml could not be read — defaults, "
                             "no camera pin: recording only while exactly one recordable USB "
                             "camera is present");

    const auto& r = cfg.recording;
    const uint64_t quota = gbToBytes(r.maxFootageGB);
    const uint64_t floor = gbToBytes(r.minFreeGB);
    log(LogLevel::INFO, "dashcam v0.4 starting (recording only)");
    std::string exposureMode = r.exposureMode;
    if (exposureMode != "auto" && exposureMode != "camera" && exposureMode != "framerate") {
        log(LogLevel::WARN, "ExposureMode '" + exposureMode + "' unknown (auto|framerate|camera) — using auto");
        exposureMode = "auto";
    }
    const bool frameRateExposure = exposureMode != "camera";   // v0.4 runs the exposure (auto|framerate)
    const bool nightModeEnabled  = exposureMode == "auto";
    std::string exposureText = exposureMode;
    if (frameRateExposure) exposureText += " (target luma " + std::to_string((int)r.targetLuma) +
                                           (nightModeEnabled ? ", night below " + std::to_string((int)r.nightLuma)
                                                             : std::string()) + ")";
    {
        char quotaBuf[32];
        std::snprintf(quotaBuf, sizeof(quotaBuf), "%.1f GB", (double)(float)r.maxFootageGB);
        const std::string quotaText = quota ? quotaBuf : "off";
        char buf[512];
        std::snprintf(buf, sizeof(buf),
                      "settings: footage=%s segment=%d s quota=%s floor=%.1f GB stall=%d ms "
                      "firstFrame=%d ms retry=%d s sync=%d ms recordFps=%d exposure=%s config=%s/dashcam.xml",
                      std::string(cfg.system.footagePath).c_str(), (int)r.segmentSec,
                      quotaText.c_str(),
                      (double)(float)r.minFreeGB, (int)r.stallTimeoutMs, (int)r.firstFrameTimeoutMs,
                      (int)r.retryIntervalSec, (int)r.syncIntervalMs, (int)r.recordFps,
                      exposureText.c_str(),
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
    // Directory probes are bounded: a footage filesystem that stopped
    // answering counts as unusable instead of stalling the supervisor.
    Storage   storage(cfg.system.footagePath, policy, log, fallbackFootageDir(), BoundedProbe{});
    Retention retention;                   // deletes run off the supervisor, one runner per dir
    FailureLog failures(log);

    rec::SegmentedRecorder recorder;
    recorder.setLogCallback(recLog);
    recorder.setOverlayConfig(cfg.overlay);

    UvcExposureControl exposure;
    uint64_t           lastLumaSeq = 0;
    // (monotonic s, camera frames) samples: the camera's real frame rate over
    // the last ~2 s — night mode's "light is back" test.
    std::deque<std::pair<double, uint64_t>> fpsWindow;
    // Every session end finalises the footage and hands exposure back to the
    // camera (a no-op after an unplug), so a stopped dashcam never leaves the
    // camera in manual mode.  The two run side by side (stopSession): the
    // footage never waits on a camera control write, and the camera is still
    // restored when recorder.stop() ends in _exit(3) on a wedged disk.  A camera
    // still not answering kCameraRestoreMs after the footage is final is wedged
    // as well, and the stuck write cannot be cancelled: exit.  Mid-run with 3 for
    // the launcher's restart loop, like a wedged teardown; on SIGINT/SIGTERM with
    // 0 — the footage is final, which is all the graceful exit promises.
    auto stopRecording = [&]() {
        if (!stopSession([&exposure] { exposure.close(); }, [&recorder] { recorder.stop(); },
                         kCameraRestoreMs)) {
            const bool shuttingDown = !g_run;
            const std::string why = "camera not answering the exposure hand-back " +
                                    std::to_string(kCameraRestoreMs) + " ms after the footage was "
                                    "finalised — " + (shuttingDown ? "exiting (shutdown)"
                                                                   : "exiting for the restart loop");
            // Never blocking on the log callback (it may be synchronous).
            dashcam::log::emergencyExit(log, why, shuttingDown ? 0 : 3);
        }
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
                using dashcam::timesync::TimeKeeper;
                dashcam::network::TimeResult t;
                double monoRx = 0.0;
                for (int i = 0; i < tries && !t.valid && timeRun.load(); ++i) {
                    const double base0 = TimeKeeper::systemNow() - TimeKeeper::monotonicNow();
                    t = dashcam::network::queryTime(server, static_cast<uint16_t>((int)net.ntpPort),
                                                    (int)net.ntpTimeoutMs, ntpLog);
                    monoRx = TimeKeeper::monotonicNow();
                    // SNTP times the round trip on the wall clock: if GPS or the host
                    // stepped it mid-query, the estimate is off by half the step.
                    if (t.valid && std::fabs((TimeKeeper::systemNow() - monoRx) - base0) > 0.05) {
                        ntpLog(LogLevel::DEBUG, "time: clock stepped during the NTP query — sample dropped");
                        t = dashcam::network::TimeResult{};
                    }
                }
                if (t.valid) {
                    if (clockSet) {
                        // Absolute estimate (server transmit time + half the round
                        // trip) pinned to when the reply arrived — never the
                        // offset, which would be applied a second time if GPS or
                        // the host's NTP service stepped the clock meanwhile.
                        const double utcRx = static_cast<double>(t.unixSeconds) + t.unixNanos / 1e9 +
                                             std::max(0.0, t.roundTripSeconds) / 2.0;
                        timeKeeper.offerNtp(utcRx, monoRx, server);
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
    uint64_t    sessions = 0;          // sessions that delivered frames
    std::string pendingStart;          // "recording ..." line, logged once frames flow
    bool        startConfirmed = false;
    auto nextAttempt   = std::chrono::steady_clock::now();
    auto nextRetention = nextAttempt;
    auto nextProbe     = nextAttempt + kPreferredProbe;
    bool warnedStillOver = false;

    // Retention runs on its own thread (a scan + deletes can block on a failing
    // disk): runRetention() only posts a pass; collectRetention() logs results.
    auto runRetention = [&]() {
        nextRetention = std::chrono::steady_clock::now() + kRetentionEvery;
        if (!storage.dir().empty())
            retention.post(storage.dir(), storage.retentionPass(recorder.deletableBelowSeq()));
    };
    bool retentionSlowWarned = false;
    auto collectRetention = [&]() {
        const int64_t busyMs = retention.busyMs();
        if (busyMs > 60000 && !retentionSlowWarned) {
            log(LogLevel::WARN, "retention pass running for " + std::to_string(busyMs / 1000) +
                                " s — footage disk slow or not answering");
            retentionSlowWarned = true;
        }
        rec::RetentionResult res;
        if (!retention.poll(res)) return;
        retentionSlowWarned = false;
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
        // Never under a running cleanup pass (see pickStartDir).
        const std::string dir = pickStartDir(storage, retention, kPreStartRetentionMs,
            [&](const std::string& d) {
                const int64_t freeNow = freeBytes(d);
                if (freeNow >= 0 && static_cast<uint64_t>(freeNow) < kHardFloorBytes)
                    runRetention();                    // make room first
            }, why);
        collectRetention();
        if (dir.empty()) { scheduleRetry(why); return false; }

        std::vector<cameraInfo> cams;
        getCameraList(cams, recLog);
        const RecordChoice rc = resolveRecordCamera(cams, cfg.cameras, configOk);
        for (const auto& n : rc.notes)
            if (notesSeen.insert(n).second) log(LogLevel::WARN, n);
        if (!rc.cam) { scheduleRetry(rc.why.empty() ? "no recordable USB camera found" : rc.why); return false; }

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
        opts.syncIntervalMs      = static_cast<uint32_t>((int)r.syncIntervalMs);

        const bool mjpeg = f.pixelFormat == V4L2_PIX_FMT_MJPEG;
        recorder.setLumaTap(frameRateExposure && mjpeg);
        if (!recorder.start(rc.cam->address, fmt, opts)) {
            scheduleRetry(rc.cam->address + ": " + recorder.lastError());
            return false;
        }
        // start() only means the pipeline was asked to play: a busy or failing
        // camera errors a moment later.  Recovery (and the first-retry-is-
        // immediate reset that goes with it) is declared by the supervisor once
        // frames actually arrive.
        char buf[256];
        std::snprintf(buf, sizeof(buf), "recording %s %ux%u %s @%.0f fps into %s%s",
                      rc.cam->address.c_str(), f.width, f.height,
                      f.pixelFormat == V4L2_PIX_FMT_H264 ? "H264" : "MJPEG",
                      (double)f.frameRate, dir.c_str(), rc.cabinFallback ? " (cabin camera)" : "");
        pendingStart   = buf;
        startConfirmed = false;
        if (frameRateExposure) {
            std::string whyNot;
            lastLumaSeq = 0;
            if (!mjpeg)
                log(LogLevel::INFO, "exposure: camera auto-exposure kept (frame-rate priority needs MJPEG)");
            else if (!exposure.open(rc.cam->address, f.frameRate, (int)r.targetLuma, recLog, whyNot))
                log(LogLevel::WARN, "exposure: camera auto-exposure kept — " + whyNot);
            else if (nightModeEnabled) {
                dashcam::camera::NightModeSwitch::Policy night;
                night.nightLuma = static_cast<float>((int)r.nightLuma);
                exposure.enableNightMode(night);
            }
            fpsWindow.clear();
        }
        runRetention();
        return true;
    };

    // ── supervisor heartbeat ─────────────────────────────────────────────────
    // An independent, I/O-free thread: if this loop stops beating (a blocked
    // device or filesystem call), exit so the launcher's restart loop recovers.
    // Exit 0 only on SIGINT/SIGTERM once the footage is final.
    std::atomic<bool> footageFinal{false};
    SupervisorWatchdog heartbeat(kSupervisorStuckMs, [&](int64_t silentMs) {
        const bool clean = !g_run && footageFinal.load();
        dashcam::log::emergencyExit(
            log, "supervisor stuck for " + std::to_string(silentMs / 1000) + " s (a blocked device "
                 "or filesystem call) — " + (clean ? "exiting (shutdown, footage final)"
                                                   : "exiting for the restart loop"),
            clean ? 0 : 3);
    });

    // ── supervisor loop ──────────────────────────────────────────────────────
    while (g_run) {
        heartbeat.beat();
        const auto now = std::chrono::steady_clock::now();
        collectRetention();
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
            // Deliberately NOT confirmed here even if a few frames arrived: a
            // session that dies before the next tick (<= 200 ms) counts as a
            // failed start and backs off, so a camera that fails right after its
            // first frame cannot drive a tight restart loop.
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

            if (!startConfirmed && recorder.framesReceived() > 0) {
                startConfirmed = true;
                quiet.store(false);
                failures.recovered();
                ++sessions;
                log(LogLevel::INFO, pendingStart);
            }

            float    luma = 0.0f;
            uint64_t seq  = 0;
            if (exposure.isOpen() && recorder.latestLuma(luma, seq) && seq != lastLumaSeq) {
                lastLumaSeq = seq;
                const double t = dashcam::timesync::TimeKeeper::monotonicNow();
                fpsWindow.emplace_back(t, recorder.sourceFramesReceived());
                while (fpsWindow.size() > 2 && t - fpsWindow.front().first > 2.5) fpsWindow.pop_front();
                const double span = t - fpsWindow.front().first;
                const float fps = span >= 1.0 ? static_cast<float>((fpsWindow.back().second -
                                                                    fpsWindow.front().second) / span)
                                              : NAN;
                exposure.onLuma(luma, fps, t);
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

    heartbeat.beat();
    log(LogLevel::INFO, "shutdown requested");
    stopRecording();
    footageFinal.store(true);
    heartbeat.beat();
    timeRun.store(false);
    bridge.close();
    heartbeat.beat();
    if (ntpThread.joinable()) ntpThread.join();
    retention.stop();                          // a pass stuck on a dead disk is left behind
    storage.release();
    heartbeat.stop();
    log(LogLevel::INFO, "dashcam v0.4 stopped (" + std::to_string(sessions) + " recording session" +
                        (sessions == 1 ? "" : "s") + ")");
    // Bounded log teardown: spdlog's shutdown joins its worker, which a
    // stalled console pipe can hold forever.  The footage is final: exit 0.
    auto logDone = std::make_shared<std::promise<void>>();
    std::future<void> logFinished = logDone->get_future();
    try {
        std::thread([logDone] { dashcam::log::shutdown(); logDone->set_value(); }).detach();
    } catch (const std::system_error&) {
        dashcam::log::shutdown();
        return 0;
    }
    if (logFinished.wait_for(std::chrono::seconds(2)) != std::future_status::ready) ::_exit(0);
    return 0;
}
