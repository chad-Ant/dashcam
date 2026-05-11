// Graphical demo: libcamera + libconfig + liblog + librecord
//
// Opens the first available camera (CSI preferred, USB fallback), streams
// a live preview window, and optionally writes a Cairo-overlaid MKV clip.
//
// Keys:  'r' = toggle recording   |   'q' / Esc = quit

#include "libcamera_csi.h"
#include "libcamera_usb.h"
#include "libconfig.h"
#include "liblog.h"
#include "librecord.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <ctime>
#include <filesystem>
#include <gst/gst.h>
#include <mutex>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using LvL = dashcam::log::LogLevel;

static std::atomic<bool> g_quit{false};
static void onSignal(int) { g_quit.store(true); }

// ─── HUD helpers ──────────────────────────────────────────────────────────────

// Draw a semi-transparent dark panel then text lines inside it.
static void drawPanel(cv::Mat& frame,
                      cv::Rect rect,
                      const std::vector<std::string>& lines,
                      cv::Scalar textColor = {180, 230, 180}) {
    rect &= cv::Rect(0, 0, frame.cols, frame.rows);
    if (rect.empty()) return;

    cv::Mat roi = frame(rect);
    cv::Mat dark(roi.size(), roi.type(), cv::Scalar(0, 0, 0));
    cv::addWeighted(dark, 0.60, roi, 0.40, 0, roi);

    int ty = rect.y + 15;
    for (const auto& line : lines) {
        cv::putText(frame, line, cv::Point(rect.x + 7, ty),
                    cv::FONT_HERSHEY_SIMPLEX, 0.42, textColor, 1, cv::LINE_AA);
        ty += 17;
    }
}

static std::string fmtFps(double fps) {
    std::ostringstream o;
    o.precision(1);
    o << std::fixed << fps;
    return o.str();
}

static const char* logTag(LvL lvl) {
    switch (lvl) {
        case LvL::DEBUG: return "DBG";
        case LvL::INFO:  return "INF";
        case LvL::WARN:  return "WRN";
        case LvL::ERROR: return "ERR";
    }
    return "?";
}

// ─── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);
    gst_init(&argc, &argv);

    // ── liblog: start async file logger; also feed last N messages to HUD ────
    fs::create_directories("./logs");
    dashcam::log::init("./logs");
    auto fileLog = dashcam::log::getCallback();

    struct LogEntry { LvL lvl; std::string msg; };
    std::vector<LogEntry> hudLog;
    std::mutex hudMtx;

    auto log = [&](LvL lvl, const std::string& msg) {
        fileLog(lvl, msg);
        std::lock_guard<std::mutex> lk(hudMtx);
        hudLog.push_back({lvl, msg});
        if (hudLog.size() > 5) hudLog.erase(hudLog.begin());
    };

    log(LvL::INFO, "demo_graphical starting");

    // ── libconfig: parse dashcam.xml; built-in defaults on failure ────────────
    dashcam::config::AppConfig cfg;
    bool cfgOk = dashcam::config::ConfigReader::load("config/dashcam.xml", cfg, fileLog);
    log(LvL::INFO, cfgOk ? "config: dashcam.xml loaded" : "config: using defaults");

    dashcam::camera::AttributeDictionary dict;
    dashcam::camera::AttributeDictionary::load("config/camera_attributes.xml", dict, fileLog);

    // ── libcamera: enumerate devices; prefer CSI, fall back to first USB ─────
    std::vector<dashcam::camera::cameraInfo> camList;
    if (dashcam::camera::getCameraList(camList) != dashcam::camera::ERROR_CODE::NONE
            || camList.empty()) {
        log(LvL::ERROR, "no cameras found — aborting");
        dashcam::log::shutdown();
        return 1;
    }
    log(LvL::INFO, "found " + std::to_string(camList.size()) + " camera(s)");

    const dashcam::camera::cameraInfo* chosen = &camList[0];
    for (const auto& c : camList)
        if (c.type == dashcam::camera::CAMERA_TYPE::CSI) { chosen = &c; break; }

    const bool isCSI = (chosen->type == dashcam::camera::CAMERA_TYPE::CSI);

    // Match the format index from config; default to 0.
    uint16_t fmtIdx = 0;
    if (isCSI)
        for (const auto& cc : cfg.cameras)
            if (cc.type == "CSI") { fmtIdx = static_cast<uint16_t>(cc.formatIndex); break; }
    if (fmtIdx >= chosen->videoFormats.size()) fmtIdx = 0;

    if (chosen->videoFormats.empty()) {
        log(LvL::ERROR, "camera has no video formats — aborting");
        dashcam::log::shutdown();
        return 1;
    }
    const auto& fmt = chosen->videoFormats[fmtIdx];
    const uint32_t W = fmt.width, H = fmt.height;
    log(LvL::INFO, chosen->address + "  " + std::to_string(W) + "x" +
        std::to_string(H) + " @" + std::to_string(static_cast<int>(fmt.frameRate)) + "fps");

    // ── librecord: create recording bin (recording starts paused) ─────────────
    fs::create_directories("./archive");
    char ts[12];
    { auto t = std::time(nullptr); std::strftime(ts, sizeof(ts), "%H%M%S", std::localtime(&t)); }
    const std::string clipPath = "./archive/demo_" + std::string(ts) + ".mkv";

    dashcam::record::Recorder recorder;
    recorder.setOverlayConfig(cfg.overlay);
    recorder.setLogCallback(fileLog);

    // Instantiate the concrete camera type to retain access to Camera_GST methods.
    std::unique_ptr<dashcam::camera::Camera_CSI> csiOwner;
    std::unique_ptr<dashcam::camera::Camera_USB> usbOwner;
    dashcam::camera::Camera_GST* gstCam = nullptr;

    if (isCSI) {
        csiOwner = std::make_unique<dashcam::camera::Camera_CSI>(*chosen);
        gstCam = csiOwner.get();
    } else {
        usbOwner = std::make_unique<dashcam::camera::Camera_USB>(*chosen);
        gstCam = usbOwner.get();
    }
    gstCam->setAttributeDictionary(dict);
    gstCam->setLogCallback(fileLog);

    uint32_t frNum, frDen;
    dashcam::camera::Camera_GST::computeFpsRational(fmt.frameRate, frNum, frDen);

    bool recBinOk = false;
    GstElement* recBin = recorder.createRecordingBin(clipPath, frNum, frDen, cfg.encoder);
    if (recBin) {
        gstCam->addBranch("recording", recBin, /*leaky=*/false, /*initialEnabled=*/false);
        recBinOk = true;
    } else {
        log(LvL::WARN, "recording bin failed — preview only");
    }

    // Open and start the capture pipeline.
    gstCam->open();
    gstCam->setCameraVideoFormat(fmtIdx);
    gstCam->start();

    {
        dashcam::camera::cameraStatus st;
        gstCam->getCameraStatus(st);
        if (st.status != dashcam::camera::CAMERA_STATUS::RUNNING) {
            log(LvL::ERROR, "camera failed to start (status=" +
                std::to_string(static_cast<int>(st.status)) + ")");
            dashcam::log::shutdown();
            return 1;
        }
    }
    log(LvL::INFO, "camera running — 'r' record  'q' quit");

    // Discard warmup frames so AE/AWB converges before display.
    const uint32_t bufBytes = W * H * 4;  // over-allocate; actual is W*H*3 (BGR) or W*H*4 (BGRx)
    std::vector<uint8_t> frameBuf(bufBytes);
    uint32_t written = 0;
    for (int i = 0; i < cfg.system.warmupFrames && !g_quit.load(); ++i)
        gstCam->captureFrame(frameBuf.data(), bufBytes, written);

    // ── capture + display loop ────────────────────────────────────────────────
    const std::string WIN = "Dashcam Library Demo";
    cv::namedWindow(WIN, cv::WINDOW_NORMAL);
    cv::resizeWindow(WIN, std::min(W, 1280u), std::min(H, 720u));

    bool recording = false;
    uint64_t frameCount = 0;
    auto tStart = std::chrono::steady_clock::now();

    while (!g_quit.load()) {
        gstCam->captureFrame(frameBuf.data(), bufBytes, written);
        if (written == 0) continue;
        ++frameCount;

        // Convert raw buffer → BGR cv::Mat.
        // captureFrame() docs guarantee BGR (3 bytes/px); CSI appsink may emit
        // BGRx (4 bytes/px) depending on nvvidconv cap negotiation — handle both.
        cv::Mat display;
        if (written == W * H * 4) {
            cv::Mat raw(static_cast<int>(H), static_cast<int>(W), CV_8UC4, frameBuf.data());
            cv::cvtColor(raw, display, cv::COLOR_BGRA2BGR);
        } else {
            cv::Mat raw(static_cast<int>(H), static_cast<int>(W), CV_8UC3, frameBuf.data());
            display = raw.clone();
        }

        // Feed live telemetry into the recording overlay.
        if (recording) {
            recorder.setOverlayData({
                10.7725 + static_cast<double>(frameCount) * 5e-5,
                106.6581 + static_cast<double>(frameCount) * 5e-5,
                52.3,
                static_cast<float>(30 + (frameCount % 50)),
                0.2f,
                static_cast<float>(frameCount % 360),
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count()
            });
        }

        // ── HUD panels ────────────────────────────────────────────────────────
        const int lineH = 17;

        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - tStart).count();
        double liveFps = (elapsed > 0.5) ? static_cast<double>(frameCount) / elapsed : 0.0;

        // Top-left — libcamera + libconfig
        const int camPanW = 340;
        std::vector<std::string> camPanel = {
            "libcamera",
            "  device   " + chosen->address,
            "  type     " + std::string(isCSI ? "CSI (Argus)" : "USB (V4L2)"),
            "  format   " + std::to_string(W) + "x" + std::to_string(H) +
                " @ " + std::to_string(static_cast<int>(fmt.frameRate)) + " fps",
            "  frames   " + std::to_string(frameCount) +
                "   live " + fmtFps(liveFps) + " fps",
            "",
            "libconfig",
            "  source   " + std::string(cfgOk ? "dashcam.xml" : "(defaults)"),
            "  bitrate  " + std::to_string(cfg.encoder.bitrate) + " kbps",
            "  preset   " + cfg.encoder.speedPreset,
            "  overlay  " + std::string(cfg.overlay.enabled ? "on" : "off"),
        };
        drawPanel(display,
                  cv::Rect(8, 8, camPanW,
                           static_cast<int>(camPanel.size()) * lineH + 10),
                  camPanel);

        // Top-right — librecord
        const int recPanW = 270;
        const cv::Scalar recColor = recording ? cv::Scalar(80, 80, 255)
                                              : cv::Scalar(180, 230, 180);
        std::vector<std::string> recPanel = {
            "librecord",
            "  " + (recBinOk ? fs::path(clipPath).filename().string()
                              : "(bin failed — preview only)"),
            "  status   " + std::string(recording ? "RECORDING" : "standby"),
            "",
            "  [r] toggle recording",
            "  [q] quit",
        };
        drawPanel(display,
                  cv::Rect(display.cols - recPanW - 8, 8, recPanW,
                           static_cast<int>(recPanel.size()) * lineH + 10),
                  recPanel, recColor);

        // Bottom — liblog recent entries
        std::vector<std::string> logPanel = {"liblog  (recent)"};
        {
            std::lock_guard<std::mutex> lk(hudMtx);
            for (const auto& e : hudLog)
                logPanel.push_back("  [" + std::string(logTag(e.lvl)) + "] " + e.msg);
        }
        const int logH = static_cast<int>(logPanel.size()) * lineH + 10;
        drawPanel(display,
                  cv::Rect(8, display.rows - logH - 8, 560, logH),
                  logPanel);

        cv::imshow(WIN, display);

        const int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) {
            g_quit.store(true);
        } else if (key == 'r' && recBinOk) {
            recording = !recording;
            gstCam->setBranchEnabled("recording", recording);
            log(LvL::INFO, recording ? "recording → " + clipPath : "recording paused");
        }
    }

    cv::destroyAllWindows();

    // ── graceful shutdown ─────────────────────────────────────────────────────
    log(LvL::INFO, "shutting down");
    if (recBinOk) recorder.disconnect();
    gstCam->stop();
    gstCam->close();
    log(LvL::INFO, "done");
    dashcam::log::shutdown();
    return 0;
}
