// dashcam_v0_2.cpp — dashcam software v0.2
//
// Camera roles:
//   USB UVC webcam        -> PRIMARY footage.  The camera's own compressed
//       MJPEG stream is muxed straight into MKV (librecord passthrough —
//       no x264, no NVENC needed).  Telemetry (speed/accel, heading,
//       lat/lon/alt, local clock) is written to an .ass sidecar with the
//       same name as the recording instead of being burned into the video.
//   IMX296 CSI slot 1 (Argus sensor-id 1) -> lane detection inference ONLY
//       (liblanedetector, UFLD v2 TensorRT): native-resolution tee rate-
//       limited to the configured fps, lane branch with 20 fps inlet cap +
//       mid-band crop (lower half minus the bottom 224 rows).
// Recording is strictly UVC/precompressed — CSI sensors only produce raw
// frames and librecord refuses them by design.
//
// Kept from earlier versions:
//   - Records immediately on start; graceful SIGINT/SIGTERM shutdown
//     (EOS-finalised MKV + closed .ass sidecar, cameras closed).
//   - Logs to the terminal (liblog console sink) AND the log file.
//   - Storage on the designated media mounts with build-local fallback.
//   - Lane detection degrades gracefully: a missing/incompatible engine or a
//     dead IMX296 logs an ERROR and the recording continues without lanes.
//
// Build: `make` (target dashcam_v0_2; needs TRT, no VPI dependency).
// Run:   docker_dev/launchcode_v0_2.sh   (starts this binary immediately)

#include "libcamera_csi.h"
#include "libcamera_usb.h"
#include "libconfig.h"
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

// ─── shutdown flag ────────────────────────────────────────────────────────────

static std::atomic<bool> g_run{true};
static void onSignal(int) { g_run.store(false); }

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

// ─── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);
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
        "dashcam v0.2 starting (UVC passthrough record + IMX296 lanes)");

    dashcam::camera::AttributeDictionary dict;
    std::string attrPath = configsDir + "/camera_attributes.xml";
    if (!fs::exists(attrPath))
        attrPath = dashcam::config::configDir() + "/camera_attributes.xml";
    if (!dashcam::camera::AttributeDictionary::load(attrPath, dict, log))
        log(dashcam::log::LogLevel::WARN, "attribute dictionary not loaded");

    const std::string footageDir = dashcam::config::resolveStorageDir(
        cfg.system.footagePath, dashcam::config::kFallbackFootageName, log);

    // ── camera discovery: first USB (record) + IMX296 (lanes) ────────────────
    std::vector<cameraInfo> cams;
    if (getCameraList(cams, log) != ERROR_CODE::NONE || cams.empty()) {
        log(dashcam::log::LogLevel::ERROR, "no cameras discovered; aborting");
        dashcam::log::shutdown();
        return 1;
    }
    const cameraInfo *usbInfo = nullptr, *laneInfo = nullptr;
    for (const auto& c : cams) {
        if (c.type == CAMERA_TYPE::USB && !usbInfo) usbInfo = &c;
        if (c.type == CAMERA_TYPE::CSI && !laneInfo && sensorNameContains(c, "imx296"))
            laneInfo = &c;
    }
    if (!usbInfo)  log(dashcam::log::LogLevel::ERROR, "no USB camera found");
    if (!laneInfo) log(dashcam::log::LogLevel::ERROR, "no IMX296 CSI camera found");
    if (!usbInfo || !laneInfo) { dashcam::log::shutdown(); return 1; }

    const int usbFmtIdx = pickUsbRecordFormat(*usbInfo);
    if (usbFmtIdx < 0) {
        log(dashcam::log::LogLevel::ERROR,
            "USB camera has no precompressed (MJPG/H264) format; aborting");
        dashcam::log::shutdown();
        return 1;
    }
    const auto& uFmt = usbInfo->videoFormats[(size_t)usbFmtIdx];
    const auto& lFmt = laneInfo->videoFormats.at(0);  // IMX296: single sensor mode

    float laneFps = 20.0f;
    for (const auto& cc : cfg.cameras) {
        if (cc.type != "CSI") continue;
        const std::string dev = cc.device;
        if (!dev.empty() && dev != laneInfo->address) continue;
        if ((float)cc.outFps > 0.0f) laneFps = (float)cc.outFps;
        break;
    }

    log(dashcam::log::LogLevel::INFO,
        "primary USB " + usbInfo->address + " "
        + std::to_string(uFmt.width) + "x" + std::to_string(uFmt.height) + "@"
        + std::to_string((int)uFmt.frameRate)
        + (uFmt.pixelFormat == V4L2_PIX_FMT_H264 ? " H264" : " MJPG")
        + " passthrough | lanes IMX296 " + laneInfo->address + " (sensor-id "
        + std::to_string(laneInfo->deviceId) + ") native "
        + std::to_string(lFmt.width) + "x" + std::to_string(lFmt.height)
        + "@" + std::to_string((int)laneFps));

    // ── lane detector: load the TRT engine before pipeline setup ─────────────
    std::unique_ptr<dashcam::lane::LaneDetector> laneDet;
    {
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

    // ── lane camera ──────────────────────────────────────────────────────────
    Camera_CSI laneCam(*laneInfo);
    laneCam.setLogCallback(log);
    laneCam.setAttributeDictionary(dict);
    laneCam.setPipelineParams(makePipelineParams(cfg.pipeline));
    laneCam.setOutputResolution(lFmt.width, lFmt.height, laneFps);  // native, rate-limited

    if (laneDet) {
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

    // ── recording: compressed UVC passthrough + ASS telemetry sidecar ────────
    const std::string recFile =
        footageDir + "/primary_" + localTimestamp() + ".mkv";

    dashcam::record::Recorder rec;
    rec.setLogCallback(log);
    rec.setOverlayConfig(cfg.overlay);

    dashcam::record::RecordingFormat rfmt;
    rfmt.v4l2PixFmt = uFmt.pixelFormat;
    rfmt.width      = uFmt.width;
    rfmt.height     = uFmt.height;
    rfmt.fps        = uFmt.frameRate;

    // Seed the telemetry clock before the first sample.
    dashcam::record::OverlayData od0 = rec.getOverlayData();
    // These builds carry no telemetry source: OverlayData's built-in
    // placeholder coordinates are the demo content.  The validity flags
    // default to false (fail-closed, so a real source that dies renders as
    // dashes), which would blank this demo's overlay entirely — so say
    // explicitly that the placeholders are intended to be drawn.
    od0.timestampMs = epochMs();
    od0.speedValid    = true;
    od0.accelValid    = true;
    od0.positionValid = true;
    od0.headingValid  = true;
    od0.speedTimestampMs    = od0.timestampMs;
    od0.accelTimestampMs    = od0.timestampMs;
    od0.positionTimestampMs = od0.timestampMs;
    od0.headingTimestampMs  = od0.timestampMs;
    rec.setOverlayData(od0);

    if (!rec.startRecording(usbInfo->address, rfmt, recFile,
                            (uint32_t)(int)cfg.recording.recordFps)) {
        log(dashcam::log::LogLevel::ERROR, "recording failed to start; aborting");
        if (laneDet) { laneCam.stop(); laneDet->stop(); laneCam.close(); }
        dashcam::log::shutdown();
        return 1;
    }

    log(dashcam::log::LogLevel::INFO,
        "recording: " + recFile
        + (laneDet ? "  | lane detection ACTIVE" : "  | lane detection OFF")
        + "  (SIGINT to stop)");

    // ── main loop: telemetry clock, recorder health, lane results ────────────
    auto lastStatus = std::chrono::steady_clock::now();
    dashcam::lane::LaneResult lastLane;
    while (g_run.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // No GPS/IMU source yet: motion fields stay at defaults, but the
        // sidecar clock tracks wall time.
        dashcam::record::OverlayData od = rec.getOverlayData();
        od.timestampMs = epochMs();
        od.speedValid    = true;
        od.accelValid    = true;
        od.positionValid = true;
        od.headingValid  = true;
        od.speedTimestampMs    = od.timestampMs;
        od.accelTimestampMs    = od.timestampMs;
        od.positionTimestampMs = od.timestampMs;
        od.headingTimestampMs  = od.timestampMs;
        rec.setOverlayData(od);

        if (laneDet) {
            const dashcam::lane::LaneResult lr = laneDet->poll();
            if (lr.numLanes != lastLane.numLanes ||
                lr.currentLaneIndex != lastLane.currentLaneIndex) {
                log(dashcam::log::LogLevel::INFO,
                    "lane update: lanes=" + std::to_string((int)lr.numLanes)
                    + " ego=" + std::to_string((int)lr.currentLaneIndex));
                lastLane = lr;
            }
        }

        auto now = std::chrono::steady_clock::now();
        if (now - lastStatus >= std::chrono::seconds(5)) {
            lastStatus = now;
            if (!rec.isRecording()) {
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
        }
    }

    // ── graceful shutdown: EOS-finalise the MKV, close the sidecar ───────────
    log(dashcam::log::LogLevel::INFO, "shutting down (signal or camera loss)");
    rec.stopRecording();
    if (laneDet) {
        laneCam.stop();
        laneDet->stop();
        laneCam.close();
    }

    auto sizeKb = [](const std::string& f) -> long {
        return fs::exists(f) ? (long)(fs::file_size(f) / 1024) : 0;
    };
    const std::string assFile = recFile.substr(0, recFile.size() - 4) + ".ass";
    log(dashcam::log::LogLevel::INFO,
        "stopped. primary " + std::to_string(sizeKb(recFile)) + " KB, telemetry "
        + std::to_string(sizeKb(assFile)) + " KB");
    dashcam::log::shutdown();
    return 0;
}
