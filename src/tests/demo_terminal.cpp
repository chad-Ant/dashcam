// Terminal demo: liblog + libconfig + libcamera + librecord
// No display required — all output goes through liblog.
// Ctrl-C stops the recording phase early and still shuts down cleanly.

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
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using LvL = dashcam::log::LogLevel;

static std::atomic<bool> g_stop{false};
static dashcam::log::LogCallback s_log;

static void onSignal(int) { g_stop.store(true); }

static void section(int n, int total, const char* lib) {
    std::ostringstream oss;
    oss << "[" << n << "/" << total << "] " << lib;
    s_log(LvL::INFO, std::string(44, '-'));
    s_log(LvL::INFO, oss.str());
    s_log(LvL::INFO, std::string(44, '-'));
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);
    gst_init(&argc, &argv);

    dashcam::log::init();  // build-local logs: <exe_dir>/logs (init creates the dir)
    s_log = dashcam::log::getCallback();

    s_log(LvL::INFO, "=== Dashcam Library Demo (terminal) ===");
    constexpr int N = 4;

    // ── [1/4] liblog ─────────────────────────────────────────────────────────
    section(1, N, "liblog");
    s_log(LvL::INFO, "  init()  OK -> " + dashcam::log::logDir());

    const struct { LvL lvl; const char* tag; } levels[] = {
        {LvL::DEBUG, "DEBUG"}, {LvL::INFO, "INFO"},
        {LvL::WARN,  "WARN"},  {LvL::ERROR, "ERROR"},
    };
    for (auto& l : levels) {
        s_log(l.lvl, std::string("test ") + l.tag + " message");
    }

    // ── [2/4] libconfig ──────────────────────────────────────────────────────
    section(2, N, "libconfig");

    // Build-local config: <exe_dir>/config/dashcam.xml, created with defaults if absent.
    const std::string cfgDir  = dashcam::config::configDir();
    const std::string cfgPath = cfgDir + "/dashcam.xml";
    dashcam::config::AppConfig cfg;
    bool cfgOk = dashcam::config::ConfigReader::loadOrCreate(cfgPath, cfg, s_log);
    s_log(LvL::INFO, "  ConfigReader::loadOrCreate(\"" + cfgPath + "\")  " +
          std::string(cfgOk ? "OK" : "FAIL"));

    {
        std::ostringstream o;
        o << "  encoder.bitrate     = " << cfg.encoder.bitrate << " kbps";
        s_log(LvL::INFO, o.str());
    }
    {
        std::ostringstream o;
        o << "  encoder.speedPreset = " << cfg.encoder.speedPreset;
        s_log(LvL::INFO, o.str());
    }
    {
        std::ostringstream o;
        o << "  encoder.keyIntMax   = " << cfg.encoder.keyIntMax;
        s_log(LvL::INFO, o.str());
    }
    // These fields are ConfigVar<std::string>; read into std::string locals so the
    // ternary member access (.empty()) and `"literal" + value` concatenation work
    // (ConfigVar's implicit conversion isn't considered in those contexts).
    const std::string encTune     = cfg.encoder.tune;
    const std::string fontFace    = cfg.overlay.fontFace;
    const std::string footagePath = cfg.system.footagePath;
    s_log(LvL::INFO, std::string("  encoder.tune        = ") +
          (encTune.empty() ? std::string("(none)") : encTune));
    s_log(LvL::INFO, std::string("  overlay.enabled     = ") +
          (cfg.overlay.enabled ? "true" : "false"));
    {
        std::ostringstream o;
        o << "  overlay.fontSize    = " << cfg.overlay.fontSize;
        s_log(LvL::INFO, o.str());
    }
    s_log(LvL::INFO, "  overlay.fontFace    = " + fontFace);
    s_log(LvL::INFO, "  system.footagePath  = " + footagePath);
    {
        std::ostringstream o;
        o << "  system.warmupFrames = " << cfg.system.warmupFrames;
        s_log(LvL::INFO, o.str());
    }

    if (cfg.cameras.empty()) {
        s_log(LvL::INFO, "  cameras: (none configured)");
    } else {
        std::ostringstream o;
        o << "  cameras (" << cfg.cameras.size() << "):";
        s_log(LvL::INFO, o.str());
        for (size_t i = 0; i < cfg.cameras.size(); ++i) {
            const auto& cc = cfg.cameras[i];
            std::ostringstream line;
            line << "    [" << i << "] name=" << cc.name
                 << "  type=" << cc.type
                 << "  device=" << cc.device
                 << "  enabled=" << (cc.enabled ? "true" : "false")
                 << "  fmt=" << cc.formatIndex;
            s_log(LvL::INFO, line.str());
        }
    }

    dashcam::camera::AttributeDictionary dict;
    dashcam::camera::AttributeDictionary::load(cfgDir + "/camera_attributes.xml", dict, s_log);
    {
        std::ostringstream o;
        o << "  AttributeDictionary entries: " << dict.entries.size();
        s_log(LvL::INFO, o.str());
    }

    // ── [3/4] libcamera ──────────────────────────────────────────────────────
    section(3, N, "libcamera");

    std::vector<dashcam::camera::cameraInfo> camList;
    auto rc = dashcam::camera::getCameraList(camList);
    if (rc != dashcam::camera::ERROR_CODE::NONE || camList.empty()) {
        s_log(LvL::WARN, "  getCameraList()  SKIP — no cameras found");
        s_log(LvL::WARN, "  Skipping camera and record sections.");
        s_log(LvL::INFO, "=== Done (no cameras) ===");
        dashcam::log::shutdown();
        return 0;
    }
    {
        std::ostringstream o;
        o << "  getCameraList()  " << camList.size() << " found";
        s_log(LvL::INFO, o.str());
    }

    for (const auto& info : camList) {
        const char* type = (info.type == dashcam::camera::CAMERA_TYPE::CSI) ? "CSI" : "USB";
        std::ostringstream hdr;
        hdr << "  " << info.address << "  [" << type << "]";
        if (info.type == dashcam::camera::CAMERA_TYPE::CSI)
            hdr << "  Argus sensor-id=" << info.deviceId;
        hdr << "  " << info.videoFormats.size() << " formats";
        if (!info.attributes.empty())
            hdr << "  " << info.attributes.size() << " controls";
        s_log(LvL::INFO, hdr.str());
    }

    // ── Update config: replace camera list from discovered hardware ──────────
    cfg.cameras.clear();
    int usbCount = 0;
    for (const auto& info : camList) {
        dashcam::config::CameraConfig cc;
        cc.type    = (info.type == dashcam::camera::CAMERA_TYPE::CSI) ? "CSI" : "USB";
        cc.device  = info.address;
        cc.enabled = true;

        if (info.type == dashcam::camera::CAMERA_TYPE::CSI) {
            cc.name     = "csi" + std::to_string(static_cast<int>(info.deviceId));
            cc.sensorId = static_cast<int>(info.deviceId);
            float bestFps = -1.0f;
            for (size_t i = 0; i < info.videoFormats.size(); ++i) {
                if (info.videoFormats[i].frameRate > bestFps) {
                    bestFps        = info.videoFormats[i].frameRate;
                    cc.formatIndex = static_cast<int>(i);
                }
            }
        } else {
            cc.name        = "usb" + std::to_string(usbCount++);
            cc.formatIndex = 0;
        }

        for (const auto& attr : info.attributes) {
            dashcam::config::CameraAttributeInfo ai;
            ai.name     = attr.name;
            ai.writable = attr.isWritable;
            ai.readable = attr.isReadable;
            ai.minValue = attr.minValue;
            ai.maxValue = attr.maxValue;
            ai.step     = attr.step;
            for (size_t i = 0; i < attr.menuOptions.size(); ++i) {
                if (i) ai.menuOptions += ";";
                ai.menuOptions += attr.menuOptions[i];
            }
            cc.attributeInfo.push_back(std::move(ai));
        }

        cfg.cameras.push_back(cc);
    }

    bool cfgSaved = dashcam::config::ConfigReader::save(cfgPath, cfg, s_log);
    {
        std::ostringstream o;
        o << "  config/dashcam.xml updated with " << cfg.cameras.size()
          << " camera(s)  " << (cfgSaved ? "OK" : "FAIL");
        s_log(cfgSaved ? LvL::INFO : LvL::ERROR, o.str());
    }

    // Pick camera: CSI preferred, then first USB.
    const dashcam::camera::cameraInfo* chosen = &camList[0];
    for (const auto& c : camList)
        if (c.type == dashcam::camera::CAMERA_TYPE::CSI) { chosen = &c; break; }

    const bool isCSI = (chosen->type == dashcam::camera::CAMERA_TYPE::CSI);

    // Use config-specified format if available.  For CSI without a config entry,
    // fall back to the highest-fps mode rather than index 0: index 0 is typically
    // the full-sensor resolution (e.g. 3280x2464@21fps on IMX219), which
    // overwhelms x264enc and produces a 32 MB frame buffer for no benefit.
    uint16_t fmtIdx = 0;
    if (isCSI) {
        bool fromConfig = false;
        for (const auto& cc : cfg.cameras)
            if (cc.type == "CSI") {
                fmtIdx = static_cast<uint16_t>(cc.formatIndex);
                fromConfig = true;
                break;
            }
        if (!fromConfig) {
            float bestFps = -1.0f;
            for (size_t i = 0; i < chosen->videoFormats.size(); ++i) {
                if (chosen->videoFormats[i].frameRate > bestFps) {
                    bestFps = chosen->videoFormats[i].frameRate;
                    fmtIdx  = static_cast<uint16_t>(i);
                }
            }
        }
    }
    if (fmtIdx >= chosen->videoFormats.size()) fmtIdx = 0;

    const auto& fmt = chosen->videoFormats[fmtIdx];
    {
        std::ostringstream o;
        o << "  Selected: " << chosen->address
          << "  fmt[" << fmtIdx << "]  "
          << fmt.width << "x" << fmt.height
          << " @ " << static_cast<int>(fmt.frameRate) << " fps";
        s_log(LvL::INFO, o.str());
    }

    // ── [4/4] librecord ──────────────────────────────────────────────────────
    section(4, N, "librecord");

    fs::create_directories("./archive");
    char ts[12];
    { auto t = std::time(nullptr); std::strftime(ts, sizeof(ts), "%H%M%S", std::localtime(&t)); }
    const std::string clipPath = "./archive/demo_terminal_" + std::string(ts) + ".mkv";

    dashcam::record::Recorder recorder;
    recorder.setOverlayConfig(cfg.overlay);
    recorder.setLogCallback(s_log);

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
    gstCam->setLogCallback(s_log);

    uint32_t frNum, frDen;
    dashcam::camera::Camera_GST::computeFpsRational(fmt.frameRate, frNum, frDen);

    bool recOk = false;
    GstElement* recBin = recorder.createRecordingBin(clipPath, frNum, frDen, cfg.encoder);
    if (recBin) {
        gstCam->addBranch("recording", recBin, /*leaky=*/false, /*initialEnabled=*/false);
        recOk = true;
        s_log(LvL::INFO, "  createRecordingBin(\"" +
              fs::path(clipPath).filename().string() + "\")  OK");
    } else {
        s_log(LvL::WARN, "  createRecordingBin()  FAIL (preview only)");
    }

    gstCam->open();
    gstCam->setCameraVideoFormat(fmtIdx);
    gstCam->start();
    {
        dashcam::camera::cameraStatus st;
        gstCam->getCameraStatus(st);
        if (st.status != dashcam::camera::CAMERA_STATUS::RUNNING) {
            std::ostringstream o;
            o << "  open + start  FAIL  status=" << static_cast<int>(st.status);
            s_log(LvL::ERROR, o.str());
            dashcam::log::shutdown();
            return 1;
        }
    }
    s_log(LvL::INFO, "  open + start  OK");

    const uint32_t W = fmt.width, H = fmt.height;
    const uint32_t bufBytes = W * H * 4;
    std::vector<uint8_t> frameBuf(bufBytes);
    uint32_t written = 0;

    {
        std::ostringstream o;
        o << "  warmup (" << cfg.system.warmupFrames << " frames) ...";
        s_log(LvL::INFO, o.str());
    }
    for (int i = 0; i < cfg.system.warmupFrames && !g_stop.load(); ++i)
        gstCam->captureFrame(frameBuf.data(), bufBytes, written);
    s_log(LvL::INFO, "  warmup  OK");

    if (recOk) {
        gstCam->setBranchEnabled("recording", true);
        s_log(LvL::INFO, "  recording enabled -> " + clipPath);
    }

    constexpr int RECORD_SECS = 5;
    {
        std::ostringstream o;
        o << "  Capturing " << RECORD_SECS << "s  (Ctrl-C to stop early)";
        s_log(LvL::INFO, o.str());
    }

    int frames = 0;
    auto tStart = std::chrono::steady_clock::now();
    auto tLast  = tStart;

    while (!g_stop.load()) {
        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - tStart).count();
        if (elapsed >= RECORD_SECS) break;

        gstCam->captureFrame(frameBuf.data(), bufBytes, written);
        if (written == 0) continue;
        ++frames;

        if (recOk) {
            recorder.setOverlayData({
                10.7725, 106.6581, 52.3,
                static_cast<float>(30 + (frames % 50)),
                0.2f,
                static_cast<float>(frames % 360),
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count()
            });
        }

        auto now = std::chrono::steady_clock::now();
        if (now - tLast >= std::chrono::seconds(1)) {
            double elapsed2 = std::chrono::duration<double>(now - tStart).count();
            std::ostringstream o;
            o << "    t=" << static_cast<int>(elapsed2)
              << "s  frames=" << std::setw(5) << frames
              << "  rate=" << static_cast<int>(frames / elapsed2) << " fps";
            s_log(LvL::INFO, o.str());
            tLast = now;
        }
    }

    double total = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - tStart).count();
    {
        std::ostringstream o;
        o << "  Captured " << frames << " frames in "
          << std::fixed << std::setprecision(1) << total << "s"
          << "  => " << static_cast<int>(frames / (total > 0 ? total : 1)) << " fps";
        s_log(LvL::INFO, o.str());
    }

    if (recOk) recorder.disconnect();
    gstCam->stop();
    gstCam->close();
    s_log(LvL::INFO, "  stop + close  OK");

    if (recOk && fs::exists(clipPath)) {
        auto sz = fs::file_size(clipPath);
        std::ostringstream o;
        o << "  output: " << clipPath << "  (" << sz / 1024 << " KB)";
        s_log(LvL::INFO, o.str());
    }

    s_log(LvL::INFO, "=== All done ===");
    dashcam::log::shutdown();
    return 0;
}
