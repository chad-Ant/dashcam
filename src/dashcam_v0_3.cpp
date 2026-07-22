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
#include "librecord.h"

#include <algorithm>
#include <atomic>
#include <chrono>
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

    const std::string logDir = dashcam::config::resolveStorageDir(
        dashcam::config::kDefaultLogDir, dashcam::config::kFallbackLogName, log);
    dashcam::log::init(logDir, makeLogParams(cfg.log));
    log(dashcam::log::LogLevel::INFO,
        "dashcam v0.3 starting (UVC record + IMX296 lanes + driver monitoring)");

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

    const cameraInfo* laneInfo = nullptr;
    for (const auto& c : cams)
        if (c.type == CAMERA_TYPE::CSI && !laneInfo && sensorNameContains(c, "imx296"))
            laneInfo = &c;

    // Driver camera first: a pinned device is reserved before recording picks.
    const cameraInfo* drvInfo = nullptr;
    int drvFmtIdx = -1;
    if (!cabinEnabled) {
        log(dashcam::log::LogLevel::INFO,
            "cabin camera disabled in config — driver monitoring OFF");
    } else if (!cabinDev.empty()) {
        for (const auto& c : cams) {
            if (c.type != CAMERA_TYPE::USB || c.address != cabinDev) continue;
            const int idx = pickDriverFormat(c);
            if (idx < 0) {
                log(dashcam::log::LogLevel::ERROR,
                    "pinned cabin camera " + cabinDev + " has no usable YUYV "
                    "format — driver monitoring OFF");
            } else {
                drvInfo = &c; drvFmtIdx = idx;
            }
            break;
        }
        // Reached here inside the pinned branch, so cabinEnabled is true and a
        // null drvInfo means the pin was unusable.  Distinguish "device absent"
        // (logged here) from "device present but no YUYV format" (logged above).
        if (!drvInfo &&
            std::none_of(cams.begin(), cams.end(), [&](const cameraInfo& c) {
                return c.type == CAMERA_TYPE::USB && c.address == cabinDev; }))
            log(dashcam::log::LogLevel::ERROR,
                "pinned cabin camera " + cabinDev + " not found — "
                "driver monitoring OFF");
    }

    // Recording camera: first precompressed-capable USB that the driver
    // monitor has not claimed.
    const cameraInfo* usbInfo = nullptr;
    int usbFmtIdx = -1;
    for (const auto& c : cams) {
        if (c.type != CAMERA_TYPE::USB) continue;
        if (drvInfo && c.address == drvInfo->address) continue;
        const int idx = pickUsbRecordFormat(c);
        if (idx >= 0) { usbInfo = &c; usbFmtIdx = idx; break; }
    }

    // Auto mode: driver camera = first remaining YUYV-capable USB.
    if (!drvInfo && cabinEnabled && cabinDev.empty()) {
        for (const auto& c : cams) {
            if (c.type != CAMERA_TYPE::USB) continue;
            if (usbInfo && c.address == usbInfo->address) continue;
            const int idx = pickDriverFormat(c);
            if (idx >= 0) { drvInfo = &c; drvFmtIdx = idx; break; }
        }
        if (!drvInfo)
            log(dashcam::log::LogLevel::ERROR,
                "no free USB camera for driver monitoring (add a cabin camera "
                "or pin one via <Camera name=\"cabin\" type=\"USB\">) — "
                "driver monitoring OFF");
    }

    if (!usbInfo)
        log(dashcam::log::LogLevel::ERROR,
            drvInfo ? "no USB camera left for recording (cabin pin claimed "
                      + drvInfo->address + ") — continuing WITHOUT recording"
                    : "no USB camera found — continuing WITHOUT recording");
    if (!laneInfo)
        log(dashcam::log::LogLevel::ERROR,
            "no IMX296 CSI camera found — continuing WITHOUT lane detection");
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

    if (usbInfo) {
        const auto& uFmt = usbInfo->videoFormats[(size_t)usbFmtIdx];
        log(dashcam::log::LogLevel::INFO,
            "record USB " + usbInfo->address + " "
            + std::to_string(uFmt.width) + "x" + std::to_string(uFmt.height) + "@"
            + std::to_string((int)uFmt.frameRate)
            + (uFmt.pixelFormat == V4L2_PIX_FMT_H264 ? " H264" : " MJPG")
            + " passthrough");
    }
    if (drvInfo) {
        const auto& dFmt = drvInfo->videoFormats[(size_t)drvFmtIdx];
        log(dashcam::log::LogLevel::INFO,
            "driver USB " + drvInfo->address + " "
            + std::to_string(dFmt.width) + "x" + std::to_string(dFmt.height)
            + "@" + std::to_string((int)dFmt.frameRate) + " YUYV"
            + (cabinDev.empty() ? " (auto)" : " (pinned)"));
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
    if (recActive) rec.stopRecording();
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
