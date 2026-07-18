// dashcam_v0_1.cpp — dashcam software v0.1
//
// Capabilities:
//   1. Records dashcam footage from BOTH cameras immediately on start:
//        USB webcam  -> primary footage (full res, telemetry overlay per config)
//        IMX296 CSI  -> secondary/debug feed (whole-output VIC downscale)
//      The IMX296 is selected by its sysfs sensor name, not by device-node or
//      probe order, so a fitted IMX219 is never picked by accident.
//   2. Graceful shutdown on SIGINT/SIGTERM: valves closed, EOS flushed, MKVs
//      finalised (librecord/libcamera teardown path), cameras closed.
//   3. Logs go to the terminal (liblog console sink) AND the log file.
//   4. Storage on the designated media mounts (/user/output/{configs,footage,
//      logs}) with automatic build-local fallback when the media is absent
//      (dashcam::config::resolveStorageDir probes writability).
//
// Orin Nano: no NVENC — both encodes are software x264 (ultrafast, config).
//
// Build: `make` (target dashcam_v0_1; no VPI dependency).
// Run:   docker_dev/launchcode_v0_1.sh   (starts this binary immediately)

#include "libcamera_csi.h"
#include "libcamera_usb.h"
#include "libconfig.h"
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
#include <string>
#include <thread>
#include <vector>

using namespace dashcam::camera;
namespace fs = std::filesystem;

// ─── shutdown flag ────────────────────────────────────────────────────────────

static std::atomic<bool> g_run{true};
static void onSignal(int) { g_run.store(false); }

// ─── helpers (same shapes as the validated dual-recording test) ───────────────

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

// True if this video node's sensor is the IMX296 (sysfs card name, e.g.
// "vi-output, imx296 9-001a") — robust against i2c probe-order changes.
static bool isImx296(const cameraInfo& info) {
    const std::string::size_type slash = info.address.find_last_of('/');
    const std::string node = (slash == std::string::npos)
                           ? info.address : info.address.substr(slash + 1);
    std::ifstream f("/sys/class/video4linux/" + node + "/name");
    if (!f) return false;
    std::string name((std::istreambuf_iterator<char>(f)),
                     std::istreambuf_iterator<char>());
    return name.find("imx296") != std::string::npos;
}

// Prefer MJPG at the largest size <=1080p @~30, else largest raw @~30, else 0.
static int pickUsbRecordFormat(const cameraInfo& ci) {
    int best = ci.videoFormats.empty() ? -1 : 0;
    long bestMjpgArea = -1, bestRawArea = -1;
    int bestMjpg = -1, bestRaw = -1;
    for (size_t i = 0; i < ci.videoFormats.size(); ++i) {
        const auto& f = ci.videoFormats[i];
        if (f.frameRate < 24.0f || f.frameRate > 31.0f) continue;
        if (f.width > 1920 || f.height > 1080) continue;
        long area = static_cast<long>(f.width) * f.height;
        if (f.pixelFormat == V4L2_PIX_FMT_MJPEG) {
            if (area > bestMjpgArea) { bestMjpgArea = area; bestMjpg = (int)i; }
        } else if (area > bestRawArea) { bestRawArea = area; bestRaw = (int)i; }
    }
    if (bestMjpg >= 0) return bestMjpg;
    if (bestRaw  >= 0) return bestRaw;
    return best;
}

// Warmup: pull frames from the appsink so AE/AWB settle before the recording
// valve opens (same gate the production tests use).
static void warmup(Camera_GST& cam, uint32_t bufBytes, int frames) {
    std::vector<uint8_t> buf(bufBytes);
    uint32_t written = 0;
    for (int i = 0; i < frames && g_run.load(); ++i)
        cam.captureFrame(buf.data(), (uint32_t)buf.size(), written);
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
    log(dashcam::log::LogLevel::INFO, "dashcam v0.1 starting");

    dashcam::camera::AttributeDictionary dict;
    std::string attrPath = configsDir + "/camera_attributes.xml";
    if (!fs::exists(attrPath))
        attrPath = dashcam::config::configDir() + "/camera_attributes.xml";
    if (!dashcam::camera::AttributeDictionary::load(attrPath, dict, log))
        log(dashcam::log::LogLevel::WARN, "attribute dictionary not loaded");

    const std::string footageDir = dashcam::config::resolveStorageDir(
        cfg.system.footagePath, dashcam::config::kFallbackFootageName, log);

    // ── camera discovery: first USB + the IMX296 (by sensor name) ────────────
    std::vector<cameraInfo> cams;
    if (getCameraList(cams, log) != ERROR_CODE::NONE || cams.empty()) {
        log(dashcam::log::LogLevel::ERROR, "no cameras discovered; aborting");
        dashcam::log::shutdown();
        return 1;
    }
    const cameraInfo *usbInfo = nullptr, *csiInfo = nullptr;
    for (const auto& c : cams) {
        if (c.type == CAMERA_TYPE::USB && !usbInfo) usbInfo = &c;
        if (c.type == CAMERA_TYPE::CSI && !csiInfo && isImx296(c)) csiInfo = &c;
    }
    if (!usbInfo) log(dashcam::log::LogLevel::ERROR, "no USB camera found");
    if (!csiInfo) log(dashcam::log::LogLevel::ERROR, "no IMX296 CSI camera found");
    if (!usbInfo || !csiInfo) { dashcam::log::shutdown(); return 1; }

    const int usbFmt = pickUsbRecordFormat(*usbInfo);
    if (usbFmt < 0) {
        log(dashcam::log::LogLevel::ERROR, "no usable USB format");
        dashcam::log::shutdown();
        return 1;
    }
    const auto& uFmt = usbInfo->videoFormats[(size_t)usbFmt];
    const auto& cFmt = csiInfo->videoFormats.at(0);   // IMX296: single sensor mode

    // IMX296 whole-output scale: per-camera config if set, else the validated
    // 640x480@20 debug-feed default.
    uint32_t csiW = 640, csiH = 480; float csiFps = 20.0f;
    for (const auto& cc : cfg.cameras) {
        const std::string dev = cc.device;
        if (cc.type != "CSI") continue;
        if (!dev.empty() && dev != csiInfo->address) continue;
        if ((int)cc.outWidth > 0 && (int)cc.outHeight > 0) {
            csiW = (uint32_t)(int)cc.outWidth;
            csiH = (uint32_t)(int)cc.outHeight;
            csiFps = (float)cc.outFps > 0.0f ? (float)cc.outFps : csiFps;
        }
        break;
    }

    log(dashcam::log::LogLevel::INFO,
        "primary USB " + usbInfo->address + " " + std::to_string(uFmt.width) + "x"
        + std::to_string(uFmt.height) + " | IMX296 " + csiInfo->address
        + " (sensor-id " + std::to_string(csiInfo->deviceId) + ") scaled to "
        + std::to_string(csiW) + "x" + std::to_string(csiH));

    // ── cameras + recorders ──────────────────────────────────────────────────
    const std::string ts = localTimestamp();
    const std::string usbFile = footageDir + "/primary_" + ts + ".mkv";
    const std::string csiFile = footageDir + "/debug_"   + ts + ".mkv";

    Camera_USB usbCam(*usbInfo);
    usbCam.setLogCallback(log);
    usbCam.setAttributeDictionary(dict);
    usbCam.setPipelineParams(makePipelineParams(cfg.pipeline));

    Camera_CSI csiCam(*csiInfo);
    csiCam.setLogCallback(log);
    csiCam.setAttributeDictionary(dict);
    csiCam.setPipelineParams(makePipelineParams(cfg.pipeline));
    csiCam.setOutputResolution(csiW, csiH, csiFps);

    dashcam::record::Recorder usbRec, csiRec;
    usbRec.setLogCallback(log);
    usbRec.setOverlayConfig(cfg.overlay);
    csiRec.setLogCallback(log);

    const float usbRecFps = std::min((float)cfg.recording.recordFps, uFmt.frameRate);
    uint32_t un, ud; Camera_GST::computeFpsRational(usbRecFps, un, ud);
    GstElement* usbBin = usbRec.createRecordingBin(
        usbFile, un, ud, cfg.encoder,
        (uint32_t)cfg.recording.queueDepth,
        dashcam::record::SourceMemory::System,
        (uint32_t)(int)cfg.recording.recordWidth,
        (uint32_t)(int)cfg.recording.recordHeight,
        cfg.overlay.enabled);

    uint32_t cn, cd; Camera_GST::computeFpsRational(csiFps, cn, cd);
    GstElement* csiBin = csiRec.createRecordingBin(
        csiFile, cn, cd, cfg.encoder,
        (uint32_t)cfg.recording.queueDepth,
        dashcam::record::SourceMemory::NVMM,
        0, 0, /*overlay=*/false);

    if (!usbBin || !csiBin) {
        log(dashcam::log::LogLevel::ERROR, "recording bin creation failed");
        dashcam::log::shutdown();
        return 1;
    }
    usbCam.addBranch("recording", usbBin, /*leaky=*/false, /*initialEnabled=*/false);
    csiCam.addBranch("debug",     csiBin, /*leaky=*/false, /*initialEnabled=*/false);

    // ── start both, warm up, open the valves ─────────────────────────────────
    usbCam.open(); usbCam.setCameraVideoFormat((uint16_t)usbFmt); usbCam.start();
    csiCam.open(); csiCam.setCameraVideoFormat(0);                csiCam.start();

    cameraStatus us, cs;
    usbCam.getCameraStatus(us); csiCam.getCameraStatus(cs);
    if (us.status != CAMERA_STATUS::RUNNING || cs.status != CAMERA_STATUS::RUNNING) {
        log(dashcam::log::LogLevel::ERROR,
            "camera start failed (usb=" + std::to_string((int)us.status)
            + " csi=" + std::to_string((int)cs.status) + "); aborting");
        usbRec.disconnect(); csiRec.disconnect();
        usbCam.close(); csiCam.close();
        dashcam::log::shutdown();
        return 1;
    }

    warmup(usbCam, uFmt.width * uFmt.height * 3 + 64, cfg.system.warmupFrames);
    warmup(csiCam, cFmt.width * cFmt.height * 3 + 64, cfg.system.warmupFrames);

    // Seed the overlay clock before the valve opens: the first recorded frames
    // are drawn before the main loop's first 200 ms tick and would otherwise
    // carry librecord's placeholder timestamp.
    dashcam::record::OverlayData od0 = usbRec.getOverlayData();
    od0.timestampMs = epochMs();
    usbRec.setOverlayData(od0);

    usbCam.setBranchEnabled("recording", true);
    csiCam.setBranchEnabled("debug",     true);
    log(dashcam::log::LogLevel::INFO,
        "recording: " + usbFile + " + " + csiFile + "  (SIGINT to stop)");

    // ── main loop: keep overlay clock live, watch camera health ─────────────
    auto lastStatus = std::chrono::steady_clock::now();
    while (g_run.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // v0.1 has no GPS/IMU source: motion fields stay at defaults, but the
        // overlay clock tracks wall time.
        dashcam::record::OverlayData od = usbRec.getOverlayData();
        od.timestampMs = epochMs();
        usbRec.setOverlayData(od);

        // getCameraStatus() also drains the pipeline bus, so a dead pipeline
        // (USB unplug, Argus failure) surfaces here as ERROR.
        auto now = std::chrono::steady_clock::now();
        if (now - lastStatus >= std::chrono::seconds(5)) {
            lastStatus = now;
            usbCam.getCameraStatus(us); csiCam.getCameraStatus(cs);
            if (us.status != CAMERA_STATUS::RUNNING) {
                log(dashcam::log::LogLevel::ERROR, "USB camera no longer RUNNING");
            }
            if (cs.status != CAMERA_STATUS::RUNNING) {
                log(dashcam::log::LogLevel::ERROR, "CSI camera no longer RUNNING");
            }
            if (us.status != CAMERA_STATUS::RUNNING &&
                cs.status != CAMERA_STATUS::RUNNING) {
                log(dashcam::log::LogLevel::ERROR, "both cameras dead; stopping");
                break;
            }
        }
    }

    // ── graceful shutdown: EOS-flush and finalise both MKVs ──────────────────
    log(dashcam::log::LogLevel::INFO, "shutting down (signal or camera loss)");
    usbRec.disconnect(); usbCam.stop(); usbCam.close();
    csiRec.disconnect(); csiCam.stop(); csiCam.close();

    auto sizeKb = [](const std::string& f) -> long {
        return fs::exists(f) ? (long)(fs::file_size(f) / 1024) : 0;
    };
    log(dashcam::log::LogLevel::INFO,
        "stopped. primary " + std::to_string(sizeKb(usbFile)) + " KB, debug "
        + std::to_string(sizeKb(csiFile)) + " KB");
    dashcam::log::shutdown();
    return 0;
}
