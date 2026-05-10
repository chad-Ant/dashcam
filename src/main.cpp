// Production dashcam application
//
// Camera architecture (Jetson Orin Nano, JetPack 6.2):
//   CSI  "front"        — IMX296 global shutter  1080p 60 fps  (recording + lane AI + sign AI)
//   USB  "stereo-left"  — laptop webcam           360p  10 fps  (stereo rangefinder left eye)
//   USB  "stereo-right" — laptop webcam           360p  10 fps  (stereo rangefinder right eye)
//   USB  "driver"       — laptop webcam           360p  10 fps  (driver behaviour analysis)
//
// Logger: async file logger in liblog.  Each library gets a callback via setLogCallback().
// Hot-path rule: no INFO inside the 60 fps CSI capture loop; WARN/ERROR only on events.

#include "libcamera_csi.h"
#include "libcamera_usb.h"
#include "libconfig.h"
#include "liblog.h"
#include "librecord.h"
#include "libstereocam.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdarg>
#include <cstdio>
#include <filesystem>
#include <gst/gst.h>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

// ─── signal handling ──────────────────────────────────────────────────────────

static std::atomic<bool> g_running{true};
static void onSignal(int) { g_running.store(false); }

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

// ─── helpers ──────────────────────────────────────────────────────────────────

static std::string utcTimestamp() {
    auto tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    struct tm tm{};
    gmtime_r(&tt, &tm);
    char buf[20];
    std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d",
        tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
        tm.tm_hour, tm.tm_min, tm.tm_sec);
    return buf;
}

// ─── camera role assignment ───────────────────────────────────────────────────
// Roles are resolved by config first (name + device path), then by discovery order.

struct CameraRoles {
    const dashcam::camera::cameraInfo* csi      = nullptr;
    const dashcam::camera::cameraInfo* stereoL  = nullptr;
    const dashcam::camera::cameraInfo* stereoR  = nullptr;
    const dashcam::camera::cameraInfo* driver   = nullptr;
    int csiFmtIdx    = 4;  // IMX296 default: format 4 = 1080p60
    int stereoFmtIdx = 0;
    int driverFmtIdx = 0;
};

static CameraRoles assignRoles(const std::vector<dashcam::camera::cameraInfo>& found,
                                const dashcam::config::AppConfig& cfg,
                                const dashcam::log::LogCallback& log) {
    CameraRoles roles;

    // Track which hardware entries have been claimed so the fallback doesn't double-assign.
    auto claimed = [&roles](const dashcam::camera::cameraInfo* p) {
        return p == roles.csi || p == roles.stereoL ||
               p == roles.stereoR || p == roles.driver;
    };

    // Config-driven pass: match by device path + role name.
    for (const auto& cc : cfg.cameras) {
        if (!cc.enabled) continue;
        for (const auto& info : found) {
            if (!cc.device.empty() && info.address != cc.device) continue;
            if (claimed(&info)) continue;
            bool ok = false;
            if (cc.type == "CSI" && !roles.csi) {
                roles.csi = &info;
                roles.csiFmtIdx = cc.formatIndex;
                ok = true;
            } else if (cc.type == "USB") {
                if      (cc.name == "stereo-left"  && !roles.stereoL) { roles.stereoL = &info; roles.stereoFmtIdx = cc.formatIndex; ok = true; }
                else if (cc.name == "stereo-right" && !roles.stereoR) { roles.stereoR = &info; ok = true; }
                else if (cc.name == "driver"       && !roles.driver)  { roles.driver  = &info; roles.driverFmtIdx = cc.formatIndex; ok = true; }
            }
            if (ok) break;
        }
    }

    // Fallback: fill empty slots with unclaimed cameras in discovery order.
    for (const auto& info : found) {
        if (claimed(&info)) continue;
        using CT = dashcam::camera::CAMERA_TYPE;
        if      (info.type == CT::CSI && !roles.csi)     roles.csi     = &info;
        else if (info.type == CT::USB && !roles.stereoL)  roles.stereoL = &info;
        else if (info.type == CT::USB && !roles.stereoR)  roles.stereoR = &info;
        else if (info.type == CT::USB && !roles.driver)   roles.driver  = &info;
    }

    if (!roles.csi)     doLog(log, dashcam::log::LogLevel::WARN, "no camera for 'front' CSI role");
    if (!roles.stereoL) doLog(log, dashcam::log::LogLevel::WARN, "no camera for 'stereo-left' role");
    if (!roles.stereoR) doLog(log, dashcam::log::LogLevel::WARN, "no camera for 'stereo-right' role");
    if (!roles.driver)  doLog(log, dashcam::log::LogLevel::WARN, "no camera for 'driver' role");
    return roles;
}

static void applyCaps(dashcam::camera::Camera_GST& cam,
                      const dashcam::config::AppConfig& cfg,
                      const std::string& roleName) {
    for (const auto& cc : cfg.cameras)
        if (cc.name == roleName)
            for (const auto& [k, v] : cc.capabilities)
                cam.setCameraAttribute(k, v);
}

// ─── stereo thread (10 fps — blocking on computeDepth) ───────────────────────

static void stereoLoop(dashcam::stereo::StereoRangefinder& sf,
                       std::atomic<bool>& running,
                       const dashcam::log::LogCallback& log) {
    doLog(log, dashcam::log::LogLevel::INFO, "stereo thread started");
    dashcam::stereo::DepthResult result;
    while (running.load()) {
        if (!sf.computeDepth(result)) {
            doLog(log, dashcam::log::LogLevel::WARN, "stereo: computeDepth failed");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        if (result.timeDeltaMs > 50)
            doLog(log, dashcam::log::LogLevel::WARN, "stereo: frame desync %lld ms",
                  static_cast<long long>(result.timeDeltaMs));
        // TODO: forward result to obstacle detection / ADAS module
    }
    doLog(log, dashcam::log::LogLevel::INFO, "stereo thread stopped");
}

// ─── driver-cam thread (10 fps) ───────────────────────────────────────────────

static void driverLoop(dashcam::camera::Camera_USB& cam, uint32_t bufSize,
                       std::atomic<bool>& running,
                       const dashcam::log::LogCallback& log) {
    doLog(log, dashcam::log::LogLevel::INFO, "driver-cam thread started");
    std::vector<uint8_t> buf(bufSize);
    uint32_t written = 0;
    while (running.load()) {
        cam.captureFrame(buf.data(), bufSize, written);
        if (written == 0) {
            doLog(log, dashcam::log::LogLevel::WARN, "driver-cam: empty frame");
            continue;
        }
        // TODO: driver behaviour inference on buf
    }
    doLog(log, dashcam::log::LogLevel::INFO, "driver-cam thread stopped");
}

// ─── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    std::signal(SIGTERM, onSignal);
    std::signal(SIGINT,  onSignal);
    gst_init(&argc, &argv);

    // ── logger ────────────────────────────────────────────────────────────────
    dashcam::log::init("/data/logs");
    auto log = dashcam::log::getCallback();
    doLog(log, dashcam::log::LogLevel::INFO, "dashcam starting");

    // ── config + attribute dictionary ─────────────────────────────────────────
    dashcam::config::AppConfig cfg;
    dashcam::camera::AttributeDictionary dict;
    if (!dashcam::config::ConfigReader::load("config/dashcam.xml", cfg, log))
        doLog(log, dashcam::log::LogLevel::WARN, "config load failed; using defaults");
    if (!dashcam::camera::AttributeDictionary::load("config/camera_attributes.xml", dict, log))
        doLog(log, dashcam::log::LogLevel::WARN, "attribute dictionary not loaded");
    fs::create_directories(cfg.system.archivePath);

    // ── camera discovery ──────────────────────────────────────────────────────
    std::vector<dashcam::camera::cameraInfo> found;
    if (dashcam::camera::getCameraList(found) != dashcam::camera::ERROR_CODE::NONE || found.empty()) {
        doLog(log, dashcam::log::LogLevel::ERROR, "no cameras discovered; aborting");
        dashcam::log::shutdown();
        return 1;
    }
    doLog(log, dashcam::log::LogLevel::INFO, "discovered %zu camera(s)", found.size());

    // First run: write a starter config the operator can fill in.
    if (cfg.cameras.empty()) {
        doLog(log, dashcam::log::LogLevel::INFO, "writing starter config from discovered hardware");
        for (const auto& info : found) {
            dashcam::config::CameraConfig cc;
            cc.name     = info.address;
            cc.type     = (info.type == dashcam::camera::CAMERA_TYPE::CSI) ? "CSI" : "USB";
            cc.device   = info.address;
            cc.sensorId = static_cast<int>(info.deviceId);
            cfg.cameras.push_back(cc);
        }
        dashcam::config::ConfigReader::save("config/dashcam.xml", cfg, log);
    }

    CameraRoles roles = assignRoles(found, cfg, log);

    // ── CSI recording pipeline ────────────────────────────────────────────────
    std::unique_ptr<dashcam::camera::Camera_CSI> csiCam;
    dashcam::record::Recorder recorder;
    bool csiRunning = false;

    if (roles.csi) {
        const auto& csiInfo = *roles.csi;
        if (static_cast<size_t>(roles.csiFmtIdx) < csiInfo.videoFormats.size()) {
            csiCam = std::make_unique<dashcam::camera::Camera_CSI>(csiInfo);
            csiCam->setLogCallback(log);
            csiCam->setAttributeDictionary(dict);
            applyCaps(*csiCam, cfg, "front");

            recorder.setLogCallback(log);
            recorder.setOverlayConfig(cfg.overlay);

            const auto& fmt = csiInfo.videoFormats.at(static_cast<size_t>(roles.csiFmtIdx));
            uint32_t frNum, frDen;
            dashcam::camera::Camera_GST::computeFpsRational(fmt.frameRate, frNum, frDen);

            std::string clip = cfg.system.archivePath + "/clip_" + utcTimestamp() + ".mkv";
            GstElement* recBin = recorder.createRecordingBin(clip, frNum, frDen, cfg.encoder);
            if (recBin)
                csiCam->addBranch("recording", recBin, false, false);
            else
                doLog(log, dashcam::log::LogLevel::ERROR, "recording bin failed; no video file");

            csiCam->open();
            csiCam->setCameraVideoFormat(static_cast<uint16_t>(roles.csiFmtIdx));
            csiCam->start();

            dashcam::camera::cameraStatus st;
            csiCam->getCameraStatus(st);
            csiRunning = (st.status == dashcam::camera::CAMERA_STATUS::RUNNING);
            if (csiRunning)
                doLog(log, dashcam::log::LogLevel::INFO, "CSI running: %s fmt[%d] %ux%u@%.0ffps",
                      csiInfo.address.c_str(), roles.csiFmtIdx,
                      fmt.width, fmt.height, static_cast<double>(fmt.frameRate));
            else
                doLog(log, dashcam::log::LogLevel::ERROR,
                      "CSI pipeline failed (status=%d err=%d)",
                      static_cast<int>(st.status), static_cast<int>(st.currentError));
        } else {
            doLog(log, dashcam::log::LogLevel::ERROR,
                  "CSI format index %d out of range (%zu formats)",
                  roles.csiFmtIdx, csiInfo.videoFormats.size());
        }
    }

    // ── stereo pair ───────────────────────────────────────────────────────────
    std::unique_ptr<dashcam::camera::Camera_USB> usbL, usbR;
    std::unique_ptr<dashcam::stereo::StereoRangefinder> stereo;
    std::thread stereoThread;

    if (roles.stereoL && roles.stereoR) {
        usbL = std::make_unique<dashcam::camera::Camera_USB>(*roles.stereoL);
        usbR = std::make_unique<dashcam::camera::Camera_USB>(*roles.stereoR);
        usbL->setLogCallback(log);
        usbR->setLogCallback(log);
        usbL->setAttributeDictionary(dict);
        usbR->setAttributeDictionary(dict);
        applyCaps(*usbL, cfg, "stereo-left");
        applyCaps(*usbR, cfg, "stereo-right");

        stereo = std::make_unique<dashcam::stereo::StereoRangefinder>(usbL.get(), usbR.get());
        stereo->setLogCallback(log);
        stereo->setFormatIndex(
            static_cast<uint16_t>(roles.stereoFmtIdx),
            static_cast<uint16_t>(roles.stereoFmtIdx));

        if (stereo->loadCalibration("config/stereo_calib.yml")) {
            auto err = stereo->open();
            if (err == dashcam::stereo::StereoError::NONE)
                err = stereo->start();
            if (err == dashcam::stereo::StereoError::NONE) {
                doLog(log, dashcam::log::LogLevel::INFO, "stereo started (%s)",
                      stereo->activeBackend() == dashcam::stereo::StereoRangefinder::Backend::VPI_CUDA
                          ? "VPI_CUDA" : "OpenCV_CPU");
                stereoThread = std::thread(stereoLoop,
                    std::ref(*stereo), std::ref(g_running), log);
            } else {
                doLog(log, dashcam::log::LogLevel::ERROR,
                      "stereo start failed (code %d)", static_cast<int>(err));
            }
        } else {
            doLog(log, dashcam::log::LogLevel::WARN,
                  "stereo_calib.yml missing — rangefinder disabled");
        }
    }

    // ── driver cam ────────────────────────────────────────────────────────────
    std::unique_ptr<dashcam::camera::Camera_USB> driverCam;
    std::thread driverThread;

    if (roles.driver && static_cast<size_t>(roles.driverFmtIdx) < roles.driver->videoFormats.size()) {
        driverCam = std::make_unique<dashcam::camera::Camera_USB>(*roles.driver);
        driverCam->setLogCallback(log);
        driverCam->setAttributeDictionary(dict);
        applyCaps(*driverCam, cfg, "driver");

        driverCam->open();
        driverCam->setCameraVideoFormat(static_cast<uint16_t>(roles.driverFmtIdx));
        driverCam->start();

        dashcam::camera::cameraStatus st;
        driverCam->getCameraStatus(st);
        if (st.status == dashcam::camera::CAMERA_STATUS::RUNNING) {
            const auto& fmt = roles.driver->videoFormats.at(static_cast<size_t>(roles.driverFmtIdx));
            doLog(log, dashcam::log::LogLevel::INFO, "driver cam running: %s %ux%u",
                  roles.driver->address.c_str(), fmt.width, fmt.height);
            driverThread = std::thread(driverLoop,
                std::ref(*driverCam), fmt.width * fmt.height * 3,
                std::ref(g_running), log);
        } else {
            doLog(log, dashcam::log::LogLevel::ERROR,
                  "driver cam failed (status=%d err=%d)",
                  static_cast<int>(st.status), static_cast<int>(st.currentError));
        }
    }

    // ── CSI hot loop (60 fps) ─────────────────────────────────────────────────
    // RULE: no INFO logging inside this loop — it runs at 60 Hz.
    // Only WARN/ERROR for fault events (pipeline crash, timeout, etc.).
    if (csiRunning && csiCam) {
        const auto& fmt = roles.csi->videoFormats.at(static_cast<size_t>(roles.csiFmtIdx));
        const uint32_t bufSize = fmt.width * fmt.height * 4;  // BGRx output from nvvidconv
        std::vector<uint8_t> buf(bufSize);
        uint32_t written = 0;

        doLog(log, dashcam::log::LogLevel::INFO,
              "CSI warmup — discarding %d frames", cfg.system.warmupFrames);
        for (int i = 0; i < cfg.system.warmupFrames && g_running.load(); ++i)
            csiCam->captureFrame(buf.data(), bufSize, written);

        csiCam->setBranchEnabled("recording", true);
        doLog(log, dashcam::log::LogLevel::INFO, "CSI recording enabled");

        while (g_running.load()) {
            csiCam->captureFrame(buf.data(), bufSize, written);

            // Telemetry hook: inject live GPS/IMU data before the next GStreamer render tick.
            // recorder.setOverlayData({lat, lon, altM, speedKmh, accMs2, headingDeg, epochMs});

            // Inference hook: buf contains 1080p BGRx at 60fps.
            // Lane detection runs every frame; sign recognition at ~5fps via throttle counter.

            dashcam::camera::cameraStatus st;
            csiCam->getCameraStatus(st);
            if (st.status != dashcam::camera::CAMERA_STATUS::RUNNING) {
                doLog(log, dashcam::log::LogLevel::ERROR,
                      "CSI pipeline fault (status=%d err=%d) — stopping",
                      static_cast<int>(st.status), static_cast<int>(st.currentError));
                g_running.store(false);
            }
        }
    } else {
        // Degrade gracefully: run stereo + driver cams without front recording.
        doLog(log, dashcam::log::LogLevel::WARN, "CSI unavailable — running without front camera");
        while (g_running.load())
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    // ── graceful shutdown ─────────────────────────────────────────────────────
    doLog(log, dashcam::log::LogLevel::INFO, "shutdown sequence started");

    // Join background threads before closing cameras they depend on.
    if (driverThread.joinable())  driverThread.join();
    if (stereoThread.joinable())  stereoThread.join();

    if (csiCam) {
        recorder.disconnect();
        csiCam->stop();
        csiCam->close();
    }
    if (stereo) {
        stereo->stop();   // stops and closes both usbL / usbR internally
        stereo->close();
    }
    if (driverCam) {
        driverCam->stop();
        driverCam->close();
    }

    doLog(log, dashcam::log::LogLevel::INFO, "dashcam stopped cleanly");
    dashcam::log::shutdown();
    return 0;
}
