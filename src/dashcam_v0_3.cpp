// dashcam_v0_3.cpp — dashcam software v0.3
//
// v0.3 = v0.2 (UVC passthrough recording + IMX296 lane detection) plus driver
// drowsiness monitoring (libdriverstate: YuNet DNN face crop + binary
// ResNet18 TensorRT classifier) on a driver-facing UVC camera at 2 Hz.
//
// Camera roles:
//   USB UVC (road-facing)  -> PRIMARY footage.  The camera's own compressed
//       MJPEG/H264 stream is muxed straight into MKV (librecord passthrough);
//       telemetry goes to an .ass sidecar.
//   IMX296 CSI (Argus)     -> lane detection inference ONLY (liblanedetector).
//   USB UVC (driver-facing)-> drowsiness classification ONLY (libdriverstate):
//       raw YUYV branch rate-limited to 2 fps, face crop, DROWSY/NATURAL.
//
// Startup configuration setter (resolveCameraConfiguration, in the init phase):
// inspects which cameras are present, assigns each discovered camera a role,
// then logs the recognised profile.  The two anchor cases:
//   * one USB camera and nothing else   -> RECORD-ONLY   (dashcam footage only).
//   * one USB camera + one IMX296 CSI   -> RECORD + LANES (footage on the USB,
//                                          lane detection on the IMX296).
// Driver monitoring layers a third role on top when a cabin camera is available
// (see below).  Any role that cannot be filled simply stays OFF.
//
// Driver-camera selection (librecord opens its camera exclusively, so the
// recorder and the driver monitor can never share one UVC device):
//   - Pinned: a <Camera name="cabin" type="USB"> config entry with a non-empty
//     <Device> (e.g. /dev/video2) reserves that device for driver monitoring;
//     recording takes the first OTHER precompressed-capable USB camera.  If
//     that leaves no recordable camera the app logs an ERROR and runs
//     WITHOUT recording (the operator explicitly assigned the camera away).
//     <Enabled>false</Enabled> on the cabin entry disables driver monitoring.
//   - Auto (no cabin entry / empty Device): first USB camera records (v0.2
//     behaviour) and the first REMAINING USB camera with a raw YUYV format
//     becomes the driver camera; with a single USB camera plugged in, driver
//     monitoring stays off.
//
// Every subsystem degrades gracefully: a missing engine, face model, or camera
// logs an ERROR and the rest keeps running.
//
// Driver alerts, two independent layers (tuned via <DriverScore>):
//   ACUTE  — sustained instantaneous DROWSY (AcuteAlertSec) -> immediate WARN
//            micro-sleep alert; wire a GPIO/MIDI alarm there later.
//   SCORE  — long-horizon fatigue score (libdriverstate FatigueScorer):
//            100 down past 0; -10 per completed 10 s drowsy, +5 per 10 s
//            awake, -10 per drowsiness-correlated lane drift (lateral offset
//            bridged from liblanedetector), cap decays 10/driving-hour.
//            Graded OK/CAUTION/WARNING tiers; FATIGUE (score <= 0 sustained
//            5 min) logs at ERROR — the "pull over" alarm.  SIGUSR1 resets
//            the score to a fresh session (GPIO button in a later version).
//
// Build: `make` (target dashcam_v0_3; needs TRT + OpenCV, no VPI dependency).
// Run:   docker_dev/launchcode_v0_3.sh   (starts this binary immediately)

#include "libcamera_csi.h"
#include "libcamera_usb.h"
#include "libconfig.h"
#include "libdriverstate.h"
#include "liblanedetector.h"
#include "liblog.h"
#include "libnetwork.h"
#include "librecord.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <gst/gst.h>
#include <linux/videodev2.h>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace dashcam::camera;
namespace fs = std::filesystem;

// ─── shutdown / reset flags ───────────────────────────────────────────────────

static std::atomic<bool> g_run{true};
static void onSignal(int) { g_run.store(false); }

// SIGUSR1 = software fatigue-score reset ("I took a break").  A GPIO button
// will trigger the same path in a later version.
static std::atomic<bool> g_resetScore{false};
static void onResetScore(int) { g_resetScore.store(true); }

// ─── driver-alert tuning ──────────────────────────────────────────────────────

// The ACUTE micro-sleep alert (sustained instantaneous DROWSY) is separate
// from — and never replaced by — the long-horizon fatigue score.  Its fire
// window comes from <DriverScore> AcuteAlertSec; the clear window stays fixed.
static constexpr float kAlertClearSec  = 2.0f;
// Sustained no-face before the "driver not visible" notice (the detector
// already bridges short dropouts with its faceHoldSec grace window).
static constexpr float kNoFaceSec      = 5.0f;
// While the high-confidence FATIGUE level is latched, re-raise the "pull over"
// alarm every this-many seconds — a one-shot alert a fatigued driver misses is
// no alert.  (A GPIO/audio alarm would repeat on the same cadence.)
static constexpr float kFatigueRealertSec = 30.0f;

// ─── helpers ─────────────────────────────────────────────────────────────────

static std::string localTimestamp() {
    auto tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    struct tm tm{};
    localtime_r(&tt, &tm);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d",
        tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
        tm.tm_hour, tm.tm_min, tm.tm_sec);
    return buf;
}

static int64_t epochMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

static dashcam::camera::PipelineParams
makePipelineParams(const dashcam::config::PipelineConfig& p) {
    dashcam::camera::PipelineParams pp;
    pp.captureTimeoutMs     = static_cast<uint32_t>(p.captureTimeoutMs);
    pp.stateChangeTimeoutMs = static_cast<uint32_t>(p.stateChangeTimeoutMs);
    pp.eosTimeoutMs         = static_cast<uint32_t>(p.eosTimeoutMs);
    pp.captureQueueDepth    = static_cast<uint32_t>(p.captureQueueDepth);
    pp.appsinkMaxBuffers    = static_cast<uint32_t>(p.appsinkMaxBuffers);
    pp.branchQueueDepth     = static_cast<uint32_t>(p.branchQueueDepth);
    return pp;
}

static dashcam::log::LogParams
makeLogParams(const dashcam::config::LogConfig& l) {
    dashcam::log::LogParams lp;
    lp.queueSize     = static_cast<uint32_t>(l.queueSize);
    lp.rotateSizeKb  = static_cast<uint32_t>(l.rotateSizeKb);
    lp.rotateFiles   = static_cast<uint32_t>(l.rotateFiles);
    lp.flushEverySec = static_cast<uint32_t>(l.flushEverySec);
    lp.level         = l.level;
    lp.flushOn       = l.flushOn;
    return lp;
}

// Query internet time (SNTP) and optionally step the system clock, per the
// <Network> config.  Called BEFORE log::init() so the corrected clock is in
// effect when the log file is named and before any footage timestamps are
// stamped; its own progress is logged through the pre-init callback (stderr,
// like the config-load messages).  Returns a one-line outcome the caller logs
// to the file after init(), so the persistent log still records what happened.
static std::string syncSystemTime(const dashcam::config::NetworkConfig& net,
                                  const dashcam::log::LogCallback& log) {
    using LvL = dashcam::log::LogLevel;
    namespace nw = dashcam::network;

    if (!net.timeSyncEnabled) {
        log(LvL::INFO, "time sync: disabled (<Network><TimeSyncEnabled> = false)");
        return "time sync: disabled";
    }

    const std::string server = net.ntpServer;
    const int tries = 1 + std::max(0, (int)net.ntpRetries);
    nw::TimeResult t;
    for (int i = 0; i < tries && !t.valid; ++i) {
        if (i > 0)
            log(LvL::INFO, "time sync: retry " + std::to_string(i) + "/" +
                           std::to_string(tries - 1) + " to " + server);
        t = nw::queryTime(server, static_cast<uint16_t>((int)net.ntpPort),
                          (int)net.ntpTimeoutMs, log);
    }

    if (!t.valid) {
        log(LvL::WARN, "time sync: no NTP reply from " + server + " after " +
                       std::to_string(tries) + " attempt(s); keeping current clock");
        return "time sync: FAILED (" + server + ", " + std::to_string(tries) + " tries)";
    }

    char off[48];
    std::snprintf(off, sizeof(off), "%+.3f", t.offsetSeconds);
    const std::string base = "time sync: " + server + " offset " + off + " s";

    if (!net.ntpStepClock) {
        log(LvL::INFO, base + " (NtpStepClock=false; clock unchanged)");
        return base + " (not stepped)";
    }
    if (std::fabs(t.offsetSeconds) <= (double)(float)net.ntpStepThresholdSec) {
        char thr[32];
        std::snprintf(thr, sizeof(thr), "%.3f", (double)(float)net.ntpStepThresholdSec);
        log(LvL::INFO, base + " within threshold " + thr + " s; clock unchanged");
        return base + " (within threshold)";
    }
    // stepSystemClock() logs its own success / EPERM(need-root) detail.
    if (nw::stepSystemClock(t, log))
        return base + " -> clock stepped";
    return base + " -> STEP FAILED (need root/CAP_SYS_TIME?)";
}

// Build an H.264-over-RTP output branch bin from an RtpSession description, ready
// for Camera_GST::addBranch().  gst_parse_bin_from_description ghosts the head
// element's sink pad so the tee's queue/valve can link to it.  Returns nullptr on
// a parse failure (the caller then simply runs without the RTP stream).
static GstElement* makeRtpBranchBin(const dashcam::network::RtpSession& rtp, bool nvmm,
                                    float fps, const dashcam::log::LogCallback& log) {
    GError* err = nullptr;
    const std::string desc = rtp.branchDescription(nvmm, fps);
    GstElement* bin = gst_parse_bin_from_description(desc.c_str(), TRUE, &err);
    if (!bin || err) {
        log(dashcam::log::LogLevel::ERROR,
            std::string("RTP branch parse failed (streaming off): ") + (err ? err->message : "?"));
        if (err) g_error_free(err);
        if (bin) gst_object_unref(bin);
        return nullptr;
    }
    return bin;
}

static dashcam::lane::LaneDetectorConfig
makeLaneConfig(const dashcam::config::DetectionConfig& d,
               const dashcam::log::LogCallback& log) {
    dashcam::lane::LaneDetectorConfig c;
    c.enginePath      = (std::string)d.laneEnginePath;
    c.targetHz        = static_cast<uint32_t>((int)d.laneTargetHz);
    c.branchMaxFps    = static_cast<uint32_t>((int)d.laneBranchMaxFps);
    c.inputCropTop    = (float)d.laneInputCropTop;
    c.inputCropBottom = (float)d.laneInputCropBottom;
    c.laneReferenceY  = (float)d.laneReferenceY;
    c.sourceIsNVMM    = true;                   // IMX296 tee emits NVMM
    c.log             = log;
    return c;
}

static dashcam::driver::FatigueScoreConfig
makeScoreConfig(const dashcam::config::DriverScoreConfig& s) {
    dashcam::driver::FatigueScoreConfig c;
    c.scoreInitial       = (float)s.scoreInitial;
    c.scoreUpper         = (float)s.scoreUpper;
    c.scoreLower         = (float)s.scoreLower;
    c.drowsyChunkSec     = (float)s.drowsyChunkSec;
    c.drowsyChunkPenalty = (float)s.drowsyChunkPenalty;
    c.awakeChunkSec      = (float)s.awakeChunkSec;
    c.awakeChunkReward   = (float)s.awakeChunkReward;
    c.laneDepartThresh   = (float)s.laneDepartThresh;
    c.laneReturnSec      = (float)s.laneReturnSec;
    c.laneDriftPenalty   = (float)s.laneDriftPenalty;
    c.capDecayPerHour    = (float)s.capDecayPerHour;
    c.capDecayFloor      = (float)s.capDecayFloor;
    c.cautionScore       = (float)s.cautionScore;
    c.warningScore       = (float)s.warningScore;
    c.fatigueScore       = (float)s.fatigueScore;
    c.fatigueSustainSec  = (float)s.fatigueSustainSec;
    c.noFaceFreezes      = (bool)s.noFaceFreezes;
    return c;
}

static dashcam::driver::DriverStateConfig
makeDriverConfig(const dashcam::config::DetectionConfig& d,
                 const dashcam::config::DriverScoreConfig& s,
                 const dashcam::log::LogCallback& log) {
    dashcam::driver::DriverStateConfig c;
    c.enginePath      = (std::string)d.driverEnginePath;
    c.targetHz        = static_cast<uint32_t>((int)d.driverTargetHz);
    c.branchMaxFps    = static_cast<uint32_t>((int)d.driverBranchMaxFps);
    c.drowsyThreshold = (float)d.driverDrowsyThreshold;
    c.faceDetection      = (bool)d.driverFaceDetection;
    c.faceModelPath      = (std::string)d.driverFaceModelPath;
    c.faceScoreThreshold = (float)d.driverFaceScore;
    c.faceDetectScale    = (float)d.driverFaceDetectScale;
    c.sourceIsNVMM       = false;               // driver camera is USB/UVC
    c.score           = makeScoreConfig(s);
    c.log             = log;
    return c;
}

static const char* levelStr(dashcam::driver::FatigueLevel l) {
    using dashcam::driver::FatigueLevel;
    switch (l) {
        case FatigueLevel::OK:      return "OK";
        case FatigueLevel::CAUTION: return "CAUTION";
        case FatigueLevel::WARNING: return "WARNING";
        case FatigueLevel::FATIGUE: return "FATIGUE";
    }
    return "?";
}

// True if this video node's sensor sysfs card name contains `needle`.
static bool sensorNameContains(const cameraInfo& info, const char* needle) {
    const std::string::size_type slash = info.address.find_last_of('/');
    const std::string node = (slash == std::string::npos)
                           ? info.address : info.address.substr(slash + 1);
    std::ifstream f("/sys/class/video4linux/" + node + "/name");
    if (!f) return false;
    std::string name((std::istreambuf_iterator<char>(f)),
                     std::istreambuf_iterator<char>());
    return name.find(needle) != std::string::npos;
}

// Recording format: PRECOMPRESSED only (MJPEG/H264 — librecord passthrough).
// Largest area <=1080p at ~24-31 fps; H264 preferred over MJPEG when both
// exist (smaller files at the same zero encode cost).
static int pickUsbRecordFormat(const cameraInfo& ci) {
    long bestH264Area = -1, bestMjpgArea = -1;
    int bestH264 = -1, bestMjpg = -1;
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

// Driver-monitor format: raw YUYV so the branch can convert to BGR without a
// JPEG decode.  Prefer 640x480 (square-crops well for the 224x224 model); any
// other YUYV mode >=240 rows works as fallback, smallest area first.  Lowest
// native frame rate wins within a size — the branch drops to 2 fps anyway, so
// a slow mode just saves USB bandwidth (the C270 floor is 5 fps).
static int pickDriverFormat(const cameraInfo& ci) {
    int best = -1;
    long bestArea = -1;
    float bestRate = 1e9f;
    for (size_t i = 0; i < ci.videoFormats.size(); ++i) {
        const auto& f = ci.videoFormats[i];
        if (f.pixelFormat != V4L2_PIX_FMT_YUYV) continue;
        if (f.height < 240 || f.frameRate < 2.0f) continue;
        const bool ideal     = (f.width == 640 && f.height == 480);
        const bool bestIdeal = (best >= 0)
            && ci.videoFormats[(size_t)best].width  == 640
            && ci.videoFormats[(size_t)best].height == 480;
        const long area = static_cast<long>(f.width) * f.height;
        const bool better =
            (ideal && !bestIdeal) ||
            (ideal == bestIdeal &&
             (best < 0 || area < bestArea ||
              (area == bestArea && f.frameRate < bestRate)));
        if (better) { best = (int)i; bestArea = area; bestRate = f.frameRate; }
    }
    return best;
}

// ─── camera configuration setter ───────────────────────────────────────────────
// The initialisation-phase step that inspects which cameras are present and
// assigns each a functionality (role).  Two operator-facing scenarios anchor it
// (file header): a lone USB camera records; a USB + IMX296 pair records on the
// USB and runs lane detection on the IMX296.  The v0.3 driver-monitoring camera
// (pinned <Camera name="cabin"> or an auto-picked spare USB) layers on top.
// Every role degrades independently — an unfilled role just stays OFF.

// Roles assigned to the discovered cameras.  Each pointer aliases an element of
// the caller's `cams` vector (nullptr = role unfilled) with its chosen format
// index into that camera's videoFormats.
struct CameraConfiguration {
    const cameraInfo* record    = nullptr;  ///< UVC MJPEG/H264 passthrough recording.
    int               recordFmt = -1;       ///< Format index for `record`.
    const cameraInfo* lane      = nullptr;  ///< IMX296 CSI lane detection (single native mode).
    const cameraInfo* driver    = nullptr;  ///< UVC driver-monitoring (drowsiness).
    int               driverFmt = -1;       ///< Format index for `driver`.
    bool              driverPinned = false; ///< Driver camera came from a config pin (vs auto).

    // Discovery census, for the recognised-configuration summary line.
    int usbCount = 0, imx296Count = 0, otherCsiCount = 0;
};

// One-line human label for the recognised configuration (the "profile"), keyed
// on which roles ended up filled.  The first two lines are the operator's named
// scenarios; the rest cover the driver-monitoring and degraded permutations.
static std::string describeConfiguration(const CameraConfiguration& c) {
    const bool r = c.record, l = c.lane, d = c.driver;
    if ( r && !l && !d) return "RECORD-ONLY (single USB dashcam recording)";
    if ( r &&  l && !d) return "RECORD + LANES (USB recording + IMX296 lane detection)";
    if ( r && !l &&  d) return "RECORD + DRIVER-MONITOR (USB recording + drowsiness; no IMX296)";
    if ( r &&  l &&  d) return "RECORD + LANES + DRIVER-MONITOR (full v0.3)";
    if (!r &&  l && !d) return "LANES-ONLY (IMX296 lane detection; no recordable USB)";
    if (!r &&  l &&  d) return "LANES + DRIVER-MONITOR (no recordable USB)";
    if (!r && !l &&  d) return "DRIVER-MONITOR-ONLY (no recordable USB, no IMX296)";
    return "NONE (no camera role could be assigned)";
}

// The configuration setter proper: assign a role to each discovered camera and
// announce the recognised profile.  Assignment order is load-bearing — a pinned
// cabin (driver) camera is reserved BEFORE recording picks, so the recorder
// never grabs the device the operator set aside for driver monitoring.
static CameraConfiguration
resolveCameraConfiguration(const std::vector<cameraInfo>& cams,
                           const std::string& cabinDev, bool cabinEnabled,
                           const dashcam::log::LogCallback& log) {
    using dashcam::log::LogLevel;
    CameraConfiguration cc;
    cc.driverPinned = !cabinDev.empty();

    // Census + lane camera: first IMX296 CSI becomes the lane-detection source.
    for (const auto& c : cams) {
        if (c.type == CAMERA_TYPE::USB) { ++cc.usbCount; continue; }
        if (c.type != CAMERA_TYPE::CSI) continue;
        if (sensorNameContains(c, "imx296")) {
            ++cc.imx296Count;
            if (!cc.lane) cc.lane = &c;
        } else {
            ++cc.otherCsiCount;
        }
    }

    // Driver camera first (see the ordering note above).
    if (!cabinEnabled) {
        log(LogLevel::INFO,
            "cabin camera disabled in config — driver monitoring OFF");
    } else if (!cabinDev.empty()) {
        for (const auto& c : cams) {
            if (c.type != CAMERA_TYPE::USB || c.address != cabinDev) continue;
            const int idx = pickDriverFormat(c);
            if (idx < 0)
                log(LogLevel::ERROR,
                    "pinned cabin camera " + cabinDev + " has no usable YUYV "
                    "format — driver monitoring OFF");
            else { cc.driver = &c; cc.driverFmt = idx; }
            break;
        }
        // Reached here inside the pinned branch, so cabinEnabled is true and a
        // null driver means the pin was unusable.  Distinguish "device absent"
        // (logged here) from "device present but no YUYV format" (logged above).
        if (!cc.driver &&
            std::none_of(cams.begin(), cams.end(), [&](const cameraInfo& c) {
                return c.type == CAMERA_TYPE::USB && c.address == cabinDev; }))
            log(LogLevel::ERROR,
                "pinned cabin camera " + cabinDev + " not found — "
                "driver monitoring OFF");
    }

    // Recording camera: first precompressed-capable USB the driver monitor has
    // not claimed.
    for (const auto& c : cams) {
        if (c.type != CAMERA_TYPE::USB) continue;
        if (cc.driver && c.address == cc.driver->address) continue;
        const int idx = pickUsbRecordFormat(c);
        if (idx >= 0) { cc.record = &c; cc.recordFmt = idx; break; }
    }

    // Auto mode: driver camera = first remaining YUYV-capable USB.
    if (!cc.driver && cabinEnabled && cabinDev.empty()) {
        for (const auto& c : cams) {
            if (c.type != CAMERA_TYPE::USB) continue;
            if (cc.record && c.address == cc.record->address) continue;
            const int idx = pickDriverFormat(c);
            if (idx >= 0) { cc.driver = &c; cc.driverFmt = idx; break; }
        }
        // A lone recording camera (the RECORD-ONLY scenario) legitimately has no
        // spare for driver monitoring — that is the expected single-camera
        // profile, not an error.  Only flag it when a spare USB existed but was
        // unusable (e.g. a webcam with no raw YUYV mode).
        if (!cc.driver && cc.usbCount > 1)
            log(LogLevel::ERROR,
                "no free USB camera for driver monitoring (add a cabin camera "
                "or pin one via <Camera name=\"cabin\" type=\"USB\">) — "
                "driver monitoring OFF");
    }

    // Per-role absence notices.  Recording is the primary function, so its
    // absence is an ERROR; lane detection is a degradable secondary subsystem
    // and its absence in a USB-only rig is by design (the RECORD-ONLY profile),
    // so it is reported at INFO — the recognised-profile line names it anyway.
    if (!cc.record)
        log(LogLevel::ERROR,
            cc.driver ? "no USB camera left for recording (cabin pin claimed "
                        + cc.driver->address + ") — continuing WITHOUT recording"
                      : "no USB camera found — continuing WITHOUT recording");
    if (!cc.lane)
        log(LogLevel::INFO,
            "no IMX296 CSI camera found — continuing WITHOUT lane detection");

    // Announce the recognised configuration and the assigned roles.
    log(LogLevel::INFO,
        "camera configuration: " + describeConfiguration(cc)
        + "  [discovered " + std::to_string(cams.size()) + ": "
        + std::to_string(cc.usbCount) + " USB, "
        + std::to_string(cc.imx296Count) + " IMX296"
        + (cc.otherCsiCount ? ", " + std::to_string(cc.otherCsiCount) + " other CSI"
                            : std::string())
        + "]");
    if (cc.record) {
        const auto& f = cc.record->videoFormats[(size_t)cc.recordFmt];
        log(LogLevel::INFO,
            "  role recording  -> " + cc.record->address + " "
            + std::to_string(f.width) + "x" + std::to_string(f.height) + "@"
            + std::to_string((int)f.frameRate)
            + (f.pixelFormat == V4L2_PIX_FMT_H264 ? " H264" : " MJPG")
            + " passthrough");
    }
    if (cc.lane) {
        const auto& f = cc.lane->videoFormats.at(0);   // IMX296: single mode
        log(LogLevel::INFO,
            "  role lanes      -> " + cc.lane->address + " (IMX296) "
            + std::to_string(f.width) + "x" + std::to_string(f.height));
    }
    if (cc.driver) {
        const auto& f = cc.driver->videoFormats[(size_t)cc.driverFmt];
        log(LogLevel::INFO,
            "  role driver     -> " + cc.driver->address + " "
            + std::to_string(f.width) + "x" + std::to_string(f.height) + "@"
            + std::to_string((int)f.frameRate) + " YUYV"
            + (cc.driverPinned ? " (pinned)" : " (auto)"));
    }
    return cc;
}

// ─── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);
    std::signal(SIGUSR1, onResetScore);   // software fatigue-score reset
    gst_init(&argc, &argv);

    // ── config first (so <Log> can shape the logger), stderr until init ──────
    auto log = dashcam::log::getCallback();
    dashcam::config::AppConfig cfg;
    const std::string configsDir = dashcam::config::resolveStorageDir(
        dashcam::config::kDefaultConfigsDir, dashcam::config::kFallbackConfigsName, log);
    if (!dashcam::config::ConfigReader::loadOrCreate(configsDir + "/dashcam.xml", cfg, log))
        log(dashcam::log::LogLevel::WARN, "config load/create failed; using defaults");

    // WiFi bring-up BEFORE anything network-dependent: ask the OS (NetworkManager)
    // to connect to the SSID.  If it cannot be fulfilled we run in OFFLINE MODE —
    // no internet time-sync, no network streaming.  Detail goes to stderr like the
    // other pre-init startup steps; the summary is logged after init().
    bool offline = false;
    std::string wifiSummary;
    {
        dashcam::network::WifiConnectConfig wc;
        wc.enabled         = cfg.network.wifiConnectEnabled;
        wc.ssid            = (std::string)cfg.network.wifiSsid;
        wc.timeoutSec      = (int)cfg.network.wifiTimeoutSec;
        wc.requireInternet = cfg.network.wifiRequireInternet;
        if (wc.enabled) {
            dashcam::network::WifiStatus ws = dashcam::network::connectWifi(wc, log);
            offline = (ws.state == dashcam::network::ConnectivityState::Offline);
            wifiSummary = offline ? ("network: OFFLINE MODE — " + ws.detail)
                                  : ("network: ONLINE (WiFi '" + ws.ssid + "')");
        } else {
            wifiSummary = "network: WiFi auto-connect disabled (assuming network present)";
        }
    }

    // Internet time sync BEFORE log::init(), so a Jetson that booted with a
    // wrong/unset RTC names its log file — and stamps all footage — with the
    // corrected clock (detail goes to stderr; the summary is logged below).
    // Skipped in offline mode (there is no internet to reach).
    const std::string timeSyncSummary =
        offline ? std::string("time sync: skipped (offline mode)")
                : syncSystemTime(cfg.network, log);

    const std::string logDir = dashcam::config::resolveStorageDir(
        dashcam::config::kDefaultLogDir, dashcam::config::kFallbackLogName, log);
    dashcam::log::init(logDir, makeLogParams(cfg.log));
    log(dashcam::log::LogLevel::INFO,
        "dashcam v0.3 starting (UVC record + IMX296 lanes + driver monitoring)");
    log(dashcam::log::LogLevel::INFO, wifiSummary);
    log(dashcam::log::LogLevel::INFO, timeSyncSummary);

    dashcam::camera::AttributeDictionary dict;
    std::string attrPath = configsDir + "/camera_attributes.xml";
    if (!fs::exists(attrPath))
        attrPath = dashcam::config::configDir() + "/camera_attributes.xml";
    if (!dashcam::camera::AttributeDictionary::load(attrPath, dict, log))
        log(dashcam::log::LogLevel::WARN, "attribute dictionary not loaded");

    const std::string footageDir = dashcam::config::resolveStorageDir(
        cfg.system.footagePath, dashcam::config::kFallbackFootageName, log);

    // ── cabin-camera pin from config (see header) ────────────────────────────
    std::string cabinDev;
    bool cabinEnabled = true;
    for (const auto& cc : cfg.cameras) {
        if (cc.type != "USB" || cc.name != "cabin") continue;
        cabinEnabled = (bool)cc.enabled;
        cabinDev     = (std::string)cc.device;
        break;
    }

    // ── camera discovery: USB record + USB driver + IMX296 lanes ─────────────
    std::vector<cameraInfo> cams;
    if (getCameraList(cams, log) != ERROR_CODE::NONE || cams.empty()) {
        log(dashcam::log::LogLevel::ERROR, "no cameras discovered; aborting");
        dashcam::log::shutdown();
        return 1;
    }

    // ── configuration setter: assign a functionality to each camera ──────────
    // Inspect which cameras are present and map each to a role (recording, lane
    // detection, driver monitoring), then announce the recognised profile.
    const CameraConfiguration camCfg =
        resolveCameraConfiguration(cams, cabinDev, cabinEnabled, log);
    const cameraInfo* laneInfo  = camCfg.lane;
    const cameraInfo* usbInfo   = camCfg.record;
    const int         usbFmtIdx = camCfg.recordFmt;
    const cameraInfo* drvInfo   = camCfg.driver;
    const int         drvFmtIdx = camCfg.driverFmt;

    if (!usbInfo && !laneInfo && !drvInfo) {
        log(dashcam::log::LogLevel::ERROR, "no usable cameras at all; aborting");
        dashcam::log::shutdown();
        return 1;
    }

    float laneFps = 20.0f;
    for (const auto& cc : cfg.cameras) {
        if (cc.type != "CSI") continue;
        const std::string dev = cc.device;
        if (!dev.empty() && laneInfo && dev != laneInfo->address) continue;
        if ((float)cc.outFps > 0.0f) laneFps = (float)cc.outFps;
        break;
    }

    // ── lane detector: load the TRT engine before pipeline setup ─────────────
    std::unique_ptr<dashcam::lane::LaneDetector> laneDet;
    if (laneInfo) {
        const auto& lFmt = laneInfo->videoFormats.at(0);  // IMX296: single mode
        const auto t0 = std::chrono::steady_clock::now();
        try {
            laneDet = std::make_unique<dashcam::lane::LaneDetector>(
                lFmt.width, lFmt.height, makeLaneConfig(cfg.detection, log));
            const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0).count();
            log(dashcam::log::LogLevel::INFO,
                "lane engine loaded in " + std::to_string(ms) + " ms");
        } catch (const std::exception& e) {
            log(dashcam::log::LogLevel::ERROR,
                std::string(e.what()) + " — continuing WITHOUT lane detection");
        }
    }

    // ── driver-state detector: load the TRT engine + YuNet face model ────────
    std::unique_ptr<dashcam::driver::DriverStateDetector> drvDet;
    if (drvInfo) {
        const auto& dFmt = drvInfo->videoFormats[(size_t)drvFmtIdx];
        const auto t0 = std::chrono::steady_clock::now();
        try {
            drvDet = std::make_unique<dashcam::driver::DriverStateDetector>(
                dFmt.width, dFmt.height,
                makeDriverConfig(cfg.detection, cfg.driverScore, log));
            const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0).count();
            log(dashcam::log::LogLevel::INFO,
                "driver engine loaded in " + std::to_string(ms) + " ms");
        } catch (const std::exception& e) {
            log(dashcam::log::LogLevel::ERROR,
                std::string(e.what()) + " — continuing WITHOUT driver monitoring");
        }
    }

    // ── inference-camera RTP streaming sessions (control layer) ──────────────
    // Lane/road camera streams to RtpPort, driver camera to RtpPort+2 (distinct
    // ports so the two H.264 streams never collide on one UDP endpoint).
    dashcam::network::RtpSession laneRtp, drvRtp;
    {
        dashcam::network::RtpStreamConfig rc;
        rc.enabled     = cfg.network.rtpEnabled;
        rc.host        = (std::string)cfg.network.rtpHost;
        rc.bitrateKbps = (int)cfg.network.rtpBitrateKbps;
        rc.port = static_cast<uint16_t>((int)cfg.network.rtpPort);       laneRtp.configure(rc);
        rc.port = static_cast<uint16_t>((int)cfg.network.rtpPort + 2);   drvRtp.configure(rc);
    }

    // ── lane camera ──────────────────────────────────────────────────────────
    Camera_CSI laneCam(laneInfo ? *laneInfo : cameraInfo{});
    if (laneDet) {
        const auto& lFmt = laneInfo->videoFormats.at(0);
        laneCam.setLogCallback(log);
        laneCam.setAttributeDictionary(dict);
        laneCam.setPipelineParams(makePipelineParams(cfg.pipeline));
        laneCam.setOutputResolution(lFmt.width, lFmt.height, laneFps);

        GstElement* laneBin = laneDet->createBin();
        if (laneBin) {
            laneCam.addBranch("lanes", laneBin, /*leaky=*/true);
        } else {
            log(dashcam::log::LogLevel::ERROR,
                "lane bin creation failed — continuing WITHOUT lane detection");
            laneDet.reset();
        }
    }
    if (laneDet && cfg.network.rtpEnabled && !offline) {
        // CSI/Argus tee is NVMM → nvvidconv head.  Leaky so encoding never blocks
        // the lane inference feed.
        GstElement* rtpBin = makeRtpBranchBin(laneRtp, /*nvmm=*/true, laneFps, log);
        if (rtpBin) {
            laneCam.addBranch("lane-rtp", rtpBin, /*leaky=*/true);
            log(dashcam::log::LogLevel::INFO,
                "lane RTP stream -> " + (std::string)cfg.network.rtpHost + ":" +
                std::to_string((int)cfg.network.rtpPort) + "   view: " + laneRtp.viewerHint());
        }
    }
    if (laneDet) {
        laneCam.open();
        laneCam.setCameraVideoFormat(0);
        laneCam.start();

        cameraStatus ls;
        laneCam.getCameraStatus(ls);
        if (ls.status != CAMERA_STATUS::RUNNING) {
            log(dashcam::log::LogLevel::ERROR,
                "IMX296 start failed — continuing WITHOUT lane detection");
            laneCam.stop(); laneDet.reset();
            laneCam.close();
        } else {
            laneCam.setCaptureEnabled(false);   // branch-only consumer
            laneDet->start();
        }
    }

    // ── driver camera ────────────────────────────────────────────────────────
    Camera_USB drvCam(drvInfo ? *drvInfo : cameraInfo{});
    if (drvDet) {
        drvCam.setLogCallback(log);
        drvCam.setPipelineParams(makePipelineParams(cfg.pipeline));

        GstElement* drvBin = drvDet->createBin();
        if (drvBin) {
            drvCam.addBranch("driverstate", drvBin, /*leaky=*/true);
        } else {
            log(dashcam::log::LogLevel::ERROR,
                "driver bin creation failed — continuing WITHOUT driver monitoring");
            drvDet.reset();
        }
    }
    if (drvDet && cfg.network.rtpEnabled && !offline) {
        // USB driver cam tee is system-memory raw → videoconvert head.
        const float drvFps = drvInfo->videoFormats[(size_t)drvFmtIdx].frameRate;
        GstElement* rtpBin = makeRtpBranchBin(drvRtp, /*nvmm=*/false, drvFps, log);
        if (rtpBin) {
            drvCam.addBranch("driver-rtp", rtpBin, /*leaky=*/true);
            log(dashcam::log::LogLevel::INFO,
                "driver RTP stream -> " + (std::string)cfg.network.rtpHost + ":" +
                std::to_string((int)cfg.network.rtpPort + 2) + "   view: " + drvRtp.viewerHint());
        }
    }
    if (drvDet) {
        drvCam.open();
        drvCam.setCameraVideoFormat(drvFmtIdx);
        drvCam.start();

        cameraStatus ds;
        drvCam.getCameraStatus(ds);
        if (ds.status != CAMERA_STATUS::RUNNING) {
            log(dashcam::log::LogLevel::ERROR,
                "driver camera start failed — continuing WITHOUT driver monitoring");
            drvCam.stop(); drvDet.reset();
            drvCam.close();
        } else {
            drvCam.setCaptureEnabled(false);    // branch-only consumer
            drvDet->start();
        }
    }

    // ── recording: compressed UVC passthrough + ASS telemetry sidecar ────────
    // Declared before rec so it outlives the recording pipeline: rec.stopRecording()
    // (below, on shutdown) tears the pipeline down first, so no stream callback can
    // fire into a destroyed server.
    dashcam::network::MediaStreamServer streamSrv;
    dashcam::record::Recorder rec;
    std::string recFile;
    bool recActive = false;
    if (usbInfo) {
        const auto& uFmt = usbInfo->videoFormats[(size_t)usbFmtIdx];
        recFile = footageDir + "/primary_" + localTimestamp() + ".mkv";

        rec.setLogCallback(log);
        rec.setOverlayConfig(cfg.overlay);

        dashcam::record::RecordingFormat rfmt;
        rfmt.v4l2PixFmt = uFmt.pixelFormat;
        rfmt.width      = uFmt.width;
        rfmt.height     = uFmt.height;
        rfmt.fps        = uFmt.frameRate;

        // Seed the telemetry clock before the first sample.
        dashcam::record::OverlayData od0 = rec.getOverlayData();
        od0.timestampMs = epochMs();
        rec.setOverlayData(od0);

        // Live-stream tap (direct precompressed path): fan the recording camera's
        // own compressed frames out to network viewers, no re-encode.  The wire
        // format follows the camera's pixel format.  Must be set BEFORE
        // startRecording() so the tee/appsink branch is built into the pipeline.
        if (cfg.network.streamEnabled && !offline) {
            const bool recMjpeg = uFmt.pixelFormat == V4L2_PIX_FMT_MJPEG;
            dashcam::network::StreamServerConfig sc;
            sc.port       = static_cast<uint16_t>((int)cfg.network.streamPort);
            sc.maxClients = (int)cfg.network.streamMaxClients;
            sc.wire       = recMjpeg ? dashcam::network::StreamWire::MjpegHttp
                                     : dashcam::network::StreamWire::RawTcp;
            if (streamSrv.start(sc, log)) {
                rec.setCompressedFrameCallback(
                    [&streamSrv](const uint8_t* d, size_t n, bool) { streamSrv.pushFrame(d, n); });
                log(dashcam::log::LogLevel::INFO,
                    std::string("live stream: ") +
                    (recMjpeg ? "open http://<device-ip>:" : "view: ffplay tcp://<device-ip>:") +
                    std::to_string(streamSrv.port()) + (recMjpeg ? "/ in a browser" : ""));
            } else {
                log(dashcam::log::LogLevel::ERROR,
                    "live stream: failed to start — recording without streaming");
            }
        }

        recActive = rec.startRecording(usbInfo->address, rfmt, recFile,
                                       (uint32_t)(int)cfg.recording.recordFps);
        if (!recActive)
            log(dashcam::log::LogLevel::ERROR,
                "recording failed to start — continuing WITHOUT recording");
    }
    if (!recActive && !laneDet && !drvDet) {
        log(dashcam::log::LogLevel::ERROR, "no active subsystems; aborting");
        dashcam::log::shutdown();
        return 1;
    }

    log(dashcam::log::LogLevel::INFO,
        std::string(recActive ? "recording: " + recFile : "recording OFF")
        + (laneDet ? "  | lanes ACTIVE" : "  | lanes OFF")
        + (drvDet  ? "  | driver monitoring ACTIVE" : "  | driver monitoring OFF")
        + "  (SIGINT to stop)");

    // ── main loop: telemetry clock, health, lane results, driver alerts ──────
    using clock = std::chrono::steady_clock;
    auto secondsSince = [](clock::time_point t) {
        return std::chrono::duration<float>(clock::now() - t).count();
    };

    auto lastStatus = clock::now();
    dashcam::lane::LaneResult lastLane;
    dashcam::driver::DriverStateResult lastDrv;

    // Acute micro-sleep alert hysteresis (fire window from config; see the
    // constants block).  Independent of the long-horizon fatigue score.
    const float acuteAlertSec = (float)cfg.driverScore.acuteAlertSec;
    bool  alertActive = false, faceLost = false;
    auto  drowsySince = clock::now(), naturalSince = clock::now();
    auto  faceLostSince = clock::now();
    bool  drowsyRun = false, naturalRun = false;

    // Fatigue-level transition tracking (score itself lives in the detector).
    auto lastLevel = dashcam::driver::FatigueLevel::OK;
    auto lastFatigueAlarm = clock::now();

    while (g_run.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Software fatigue-score reset (SIGUSR1; GPIO button later).
        if (g_resetScore.exchange(false) && drvDet) {
            drvDet->resetScore();
            log(dashcam::log::LogLevel::INFO,
                "fatigue score reset to a fresh session (SIGUSR1)");
        }

        // No GPS/IMU source yet: motion fields stay at defaults, but the
        // sidecar clock tracks wall time.
        if (recActive) {
            dashcam::record::OverlayData od = rec.getOverlayData();
            od.timestampMs = epochMs();
            rec.setOverlayData(od);
        }

        if (laneDet) {
            const dashcam::lane::LaneResult lr = laneDet->poll();
            if (lr.numLanes != lastLane.numLanes ||
                lr.currentLaneIndex != lastLane.currentLaneIndex) {
                log(dashcam::log::LogLevel::INFO,
                    "lane update: lanes=" + std::to_string((int)lr.numLanes)
                    + " ego=" + std::to_string((int)lr.currentLaneIndex));
                lastLane = lr;
            }
            // Bridge lane position into the fatigue scorer (the libraries are
            // deliberately decoupled — the app is the only place both exist).
            if (drvDet)
                drvDet->setLaneOffset(lr.lateralOffset, lr.lateralValid);
        }

        if (drvDet) {
            const dashcam::driver::DriverStateResult dr = drvDet->poll();
            lastDrv = dr;
            if (dr.valid && dr.faceDetected) {
                if (faceLost) {
                    log(dashcam::log::LogLevel::INFO, "driver face reacquired");
                    faceLost = false;
                }
                if (dr.state == dashcam::driver::DriverState::DROWSY) {
                    if (!drowsyRun) { drowsyRun = true; drowsySince = clock::now(); }
                    naturalRun = false;
                    if (!alertActive && secondsSince(drowsySince) >= acuteAlertSec) {
                        alertActive = true;
                        // Alarm hook: GPIO buzzer / MIDI chime goes here.
                        log(dashcam::log::LogLevel::WARN,
                            "DRIVER DROWSY ALERT (p="
                            + std::to_string(dr.drowsyProbability) + ", sustained "
                            + std::to_string((int)acuteAlertSec) + "s)");
                    }
                } else {
                    if (!naturalRun) { naturalRun = true; naturalSince = clock::now(); }
                    drowsyRun = false;
                    if (alertActive && secondsSince(naturalSince) >= kAlertClearSec) {
                        alertActive = false;
                        log(dashcam::log::LogLevel::INFO, "driver drowsy alert cleared");
                    }
                }
                faceLostSince = clock::now();
            } else if (dr.valid) {
                // No face past the detector's own hold window.
                drowsyRun = naturalRun = false;
                if (!faceLost && secondsSince(faceLostSince) >= kNoFaceSec) {
                    faceLost = true;
                    log(dashcam::log::LogLevel::WARN,
                        "driver face not visible for "
                        + std::to_string((int)kNoFaceSec) + "s"
                        + (alertActive ? " (drowsy alert still latched)" : ""));
                }
            }

            // Long-horizon fatigue tier transitions (score-driven, graded).
            using dashcam::driver::FatigueLevel;
            if (dr.valid && dr.fatigueLevel != lastLevel) {
                const std::string msg =
                    std::string("fatigue level ") + levelStr(lastLevel) + " -> "
                    + levelStr(dr.fatigueLevel)
                    + " (score=" + std::to_string((int)dr.fatigueScore)
                    + "/" + std::to_string((int)dr.fatigueCap) + ")";
                if (dr.fatigueLevel == FatigueLevel::FATIGUE) {
                    // Strong alarm hook (repeating chime / voice prompt) here.
                    log(dashcam::log::LogLevel::ERROR,
                        "DRIVER FATIGUE — high-confidence, sustained: pull "
                        "over when safe. " + msg);
                    lastFatigueAlarm = clock::now();
                } else if (dr.fatigueLevel > lastLevel)
                    log(dashcam::log::LogLevel::WARN, msg);
                else
                    log(dashcam::log::LogLevel::INFO, msg + " (recovering)");
                lastLevel = dr.fatigueLevel;
            }
            // Re-raise the FATIGUE alarm while it stays latched (a missed
            // one-shot is no alert).
            if (dr.valid && dr.fatigueLevel == FatigueLevel::FATIGUE &&
                secondsSince(lastFatigueAlarm) >= kFatigueRealertSec) {
                lastFatigueAlarm = clock::now();
                log(dashcam::log::LogLevel::ERROR,
                    "DRIVER FATIGUE — still fatigued (score="
                    + std::to_string((int)dr.fatigueScore)
                    + "): pull over when safe.");
            }
        }

        auto now = clock::now();
        if (now - lastStatus >= std::chrono::seconds(5)) {
            lastStatus = now;
            if (recActive && !rec.isRecording()) {
                log(dashcam::log::LogLevel::ERROR,
                    "recording no longer healthy; stopping");
                break;
            }
            if (laneDet) {
                cameraStatus ls;
                laneCam.getCameraStatus(ls);
                if (ls.status != CAMERA_STATUS::RUNNING) {
                    log(dashcam::log::LogLevel::ERROR,
                        "IMX296 no longer RUNNING — lane detection lost");
                    laneCam.stop(); laneDet->stop(); laneDet.reset();
                    laneCam.close();
                } else {
                    log(dashcam::log::LogLevel::DEBUG,
                        "lane: lanes=" + std::to_string((int)lastLane.numLanes)
                        + " ego=" + std::to_string((int)lastLane.currentLaneIndex));
                }
            }
            if (drvDet) {
                cameraStatus ds;
                drvCam.getCameraStatus(ds);
                if (ds.status != CAMERA_STATUS::RUNNING) {
                    log(dashcam::log::LogLevel::ERROR,
                        "driver camera no longer RUNNING — driver monitoring lost");
                    drvCam.stop(); drvDet->stop(); drvDet.reset();
                    drvCam.close();
                } else {
                    log(dashcam::log::LogLevel::DEBUG,
                        std::string("driver: ")
                        + (!lastDrv.valid ? "warming up"
                           : !lastDrv.faceDetected ? "no face"
                           : "p(drowsy)=" + std::to_string(lastDrv.drowsyProbability))
                        + (lastDrv.valid
                           ? "  score=" + std::to_string((int)lastDrv.fatigueScore)
                             + "/" + std::to_string((int)lastDrv.fatigueCap)
                             + " " + levelStr(lastDrv.fatigueLevel)
                           : "")
                        + (alertActive ? " [ALERT]" : ""));
                }
            }
            if (!recActive && !laneDet && !drvDet) {
                log(dashcam::log::LogLevel::ERROR,
                    "all subsystems lost; stopping");
                break;
            }
        }
    }

    // ── graceful shutdown: EOS-finalise the MKV, close the sidecar ───────────
    log(dashcam::log::LogLevel::INFO, "shutting down (signal or subsystem loss)");
    if (recActive) rec.stopRecording();   // stops the pipeline → no more stream callbacks
    streamSrv.stop();                     // then disconnect viewers (safe: no callbacks in flight)
    if (laneDet) {
        laneCam.stop();
        laneDet->stop();
        laneCam.close();
    }
    if (drvDet) {
        drvCam.stop();      // flushing appsink → inference thread drains
        drvDet->stop();     // joins inference thread
        drvCam.close();
    }

    if (recActive) {
        auto sizeKb = [](const std::string& f) -> long {
            return fs::exists(f) ? (long)(fs::file_size(f) / 1024) : 0;
        };
        const std::string assFile = recFile.substr(0, recFile.size() - 4) + ".ass";
        log(dashcam::log::LogLevel::INFO,
            "stopped. primary " + std::to_string(sizeKb(recFile)) + " KB, telemetry "
            + std::to_string(sizeKb(assFile)) + " KB");
    } else {
        log(dashcam::log::LogLevel::INFO, "stopped.");
    }
    dashcam::log::shutdown();
    return 0;
}
