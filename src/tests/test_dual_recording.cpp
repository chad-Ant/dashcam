// test_dual_recording.cpp
//
// Exercises the current dashcam recording architecture:
//   USB camera  -> PRIMARY footage, full resolution, WITH telemetry overlay
//                  (librecord System inlet: v4l2/videoconvert path).
//   CSI camera  -> DEBUG feed, whole IMX296 output scaled to 640x480@20, NO overlay
//                  (librecord NVMM inlet: nvarguscamerasrc/nvvidconv VIC path).
//
// Both clips are video, so both land in the footage dir (cfg.system.footagePath,
// default /user/output/footage) with timestamped primary_/debug_ names; the run log
// goes to /user/output/logs and a copy of the effective config to the configs dir
// (kDefaultConfigsDir, default /user/output/configs).  Each destination resolves
// through resolveStorageDir(), so a removed SD card degrades to build-local storage.
// The clips are then validated with ffprobe.  Both recording branches are gated
// (valve starts closed) so AE/exposure settle during warmup before recording begins,
// exactly like the production app.
//
// Orin Nano note: no NVENC — both streams are software-encoded (x264enc).  The
// IMX296 output is scaled on the VIC (~free) to 640x480@20 for the whole feed
// (Camera_GST::setOutputResolution), and its debug recording omits the Cairo
// overlay (createRecordingBin overlay=false) to keep the second encoder cheap.

#include "libcamera_csi.h"
#include "libcamera_usb.h"
#include "libconfig.h"
#include "liblog.h"
#include "librecord.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <gst/gst.h>
#include <iostream>
#include <linux/videodev2.h>
#include <string>
#include <thread>
#include <vector>

using namespace dashcam::camera;
namespace fs = std::filesystem;

// ─── tunables ─────────────────────────────────────────────────────────────────
// IMX296 whole-output default: the CSI camera scales its 1456x1088 sensor mode
// down to this size/rate for EVERYTHING (the live/appsink feed and the debug
// recording), via Camera_GST::setOutputResolution() (VIC nvvidconv + videorate).
static constexpr uint32_t CSI_OUT_W   = 640;  ///< IMX296 scaled output width.
static constexpr uint32_t CSI_OUT_H   = 480;  ///< IMX296 scaled output height.
static constexpr int      CSI_OUT_FPS = 20;   ///< IMX296 scaled output / debug fps.

// ─── helpers ──────────────────────────────────────────────────────────────────

static std::string utcTimestamp() {
    auto tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    struct tm tm{};
    gmtime_r(&tt, &tm);
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
    return pp;
}

// Pick a sensible USB record format: prefer MJPG at the largest size <=1080p @~30,
// else any MJPG @~30, else the largest raw @~30, else format 0.
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

static const char* fourccStr(uint32_t pf) {
    switch (pf) {
        case V4L2_PIX_FMT_MJPEG: return "MJPG";
        case V4L2_PIX_FMT_YUYV:  return "YUYV";
        case V4L2_PIX_FMT_NV12:  return "NV12";
        default: return "raw";
    }
}

// Drain a camera's appsink (the "inference feed") + count frames, until stop.
static void captureLoop(Camera_GST* cam, uint32_t bufBytes, std::atomic<bool>* run,
                        std::atomic<uint64_t>* frames) {
    std::vector<uint8_t> buf(bufBytes);
    uint32_t written = 0;
    while (run->load()) {
        cam->captureFrame(buf.data(), (uint32_t)buf.size(), written);
        if (written > 0) frames->fetch_add(1);
    }
}

static bool ffprobeOk(const std::string& file, uint32_t wantW, uint32_t wantH) {
    std::cout << "  ffprobe " << file << ":\n";
    std::string cmd =
        "ffprobe -v error -select_streams v:0 -show_entries "
        "stream=codec_name,width,height,avg_frame_rate:format=duration "
        "-of default=noprint_wrappers=1 " + file + " 2>&1";
    if (std::system(cmd.c_str()) != 0) {
        std::cerr << "  ffprobe rejected " << file << "\n";
        return false;
    }
    // Resolution assertion via a second, machine-readable probe.
    std::string dims;
    {
        std::string q = "ffprobe -v error -select_streams v:0 -show_entries "
                        "stream=width,height -of csv=p=0:s=x " + file + " 2>/dev/null";
        FILE* p = popen(q.c_str(), "r");
        if (p) { char b[64]; if (fgets(b, sizeof b, p)) dims = b; pclose(p); }
        if (!dims.empty() && dims.back() == '\n') dims.pop_back();
    }
    std::string want = std::to_string(wantW) + "x" + std::to_string(wantH);
    if (dims != want) {
        std::cerr << "  resolution mismatch: got '" << dims << "' want '" << want << "'\n";
        return false;
    }
    std::cout << "  resolution OK (" << dims << ")\n";
    return true;
}

// ─── test ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    gst_init(&argc, &argv);
    const int recSeconds = (argc > 1) ? std::max(2, std::atoi(argv[1])) : 8;

    dashcam::config::AppConfig cfg;
    dashcam::config::ConfigReader::load("config/dashcam.xml", cfg);
    dashcam::camera::AttributeDictionary dict;
    dashcam::camera::AttributeDictionary::load("config/camera_attributes.xml", dict);
    // Three destinations, each resolved with a build-local fallback for when the
    // configured mount is unavailable (SD card removed):
    //   footageDir : USB primary recording + CSI debug feed  -> <exe>/footage fallback.
    //   logDir     : run log                                 -> <exe>/logs fallback.
    //   configsDir : copy of the effective config            -> <exe>/configs fallback.
    // resolveStorageDir() creates the directory it returns, so no separate mkdir.
    // The log dir is not a config field (chosen before load); use kDefaultLogDir.
    const std::string logDir = dashcam::config::resolveStorageDir(
        dashcam::config::kDefaultLogDir, dashcam::config::kFallbackLogName);
    dashcam::log::init(logDir);   // run log lands in the (resolved) logs dir
    auto log = dashcam::log::getCallback();
    const std::string footageDir = dashcam::config::resolveStorageDir(
        cfg.system.footagePath, dashcam::config::kFallbackFootageName, log);
    const std::string configsDir = dashcam::config::resolveStorageDir(
        dashcam::config::kDefaultConfigsDir, dashcam::config::kFallbackConfigsName, log);
    // Drop a copy of the effective config into the configs dir.
    dashcam::config::ConfigReader::save(configsDir + "/dashcam.xml", cfg, log);

    std::vector<cameraInfo> cams;
    if (getCameraList(cams) != ERROR_CODE::NONE || cams.empty()) {
        std::cerr << "No cameras discovered.\n";
        return 1;
    }
    const cameraInfo *usbInfo = nullptr, *csiInfo = nullptr;
    for (const auto& c : cams) {
        if (c.type == CAMERA_TYPE::USB && !usbInfo) usbInfo = &c;
        if (c.type == CAMERA_TYPE::CSI && !csiInfo) csiInfo = &c;
    }
    if (!usbInfo) { std::cerr << "FAIL: no USB camera (needed for primary recording).\n"; return 1; }
    if (!csiInfo) { std::cerr << "FAIL: no CSI camera (needed for debug feed).\n"; return 1; }

    // ── format selection ────────────────────────────────────────────────────────
    int usbFmt = pickUsbRecordFormat(*usbInfo);
    if (usbFmt < 0 || (size_t)usbFmt >= usbInfo->videoFormats.size()) {
        std::cerr << "FAIL: no usable USB format.\n"; return 1;
    }
    const auto& uFmt = usbInfo->videoFormats[usbFmt];
    const auto& cFmt = csiInfo->videoFormats.at(0);   // IMX296: single sensor mode

    std::cout << "=== Dual recording test (USB primary + CSI debug) ===\n";
    std::cout << "USB   " << usbInfo->address << "  fmt[" << usbFmt << "] "
              << uFmt.width << "x" << uFmt.height << " @ " << uFmt.frameRate
              << " [" << fourccStr(uFmt.pixelFormat) << "]  -> primary record\n";
    std::cout << "CSI   " << csiInfo->address << "  sensor fmt[0] "
              << cFmt.width << "x" << cFmt.height << " @ " << cFmt.frameRate
              << "  -> whole output scaled to " << CSI_OUT_W << "x" << CSI_OUT_H
              << " @ " << CSI_OUT_FPS << "fps (live feed + debug record)\n";
    std::cout << "Footage (primary + debug) -> " << footageDir
              << "\n         log -> " << logDir << "   config -> " << configsDir
              << "\n         record " << recSeconds << "s   warmup " << cfg.system.warmupFrames << " frames\n";

    // ── build USB primary camera + recorder ─────────────────────────────────────
    // Both recordings are video, so both land in the footage dir; the CSI debug feed
    // is distinguished by its "debug_" filename prefix.
    const std::string ts = utcTimestamp();
    const std::string usbFile = footageDir + "/primary_" + ts + ".mkv";  // primary recorded video
    const std::string csiFile = footageDir + "/debug_"   + ts + ".mkv";  // CSI debug feed
    if (fs::exists(usbFile)) fs::remove(usbFile);
    if (fs::exists(csiFile)) fs::remove(csiFile);

    Camera_USB usbCam(*usbInfo);
    usbCam.setLogCallback(log);
    usbCam.setAttributeDictionary(dict);
    usbCam.setPipelineParams(makePipelineParams(cfg.pipeline));
    dashcam::record::Recorder usbRec;
    usbRec.setLogCallback(log);
    usbRec.setOverlayConfig(cfg.overlay);

    // Primary records at the configured fps, never above the camera's native rate.
    float usbRecFps = std::min(static_cast<float>(cfg.recording.recordFps), uFmt.frameRate);
    uint32_t un, ud; Camera_GST::computeFpsRational(usbRecFps, un, ud);
    GstElement* usbBin = usbRec.createRecordingBin(
        usbFile, un, ud, cfg.encoder,
        static_cast<uint32_t>(cfg.recording.queueDepth),
        dashcam::record::SourceMemory::System);          // USB -> videoconvert inlet
    if (!usbBin) { std::cerr << "FAIL: USB recording bin\n"; return 1; }
    usbCam.addBranch("recording", usbBin, /*leaky=*/false, /*initialEnabled=*/false);

    // ── build CSI debug camera + recorder ───────────────────────────────────────
    Camera_CSI csiCam(*csiInfo);
    csiCam.setLogCallback(log);
    csiCam.setAttributeDictionary(dict);
    csiCam.setPipelineParams(makePipelineParams(cfg.pipeline));
    // Scale the WHOLE IMX296 output (live feed + every branch) to 640x480@20.
    csiCam.setOutputResolution(CSI_OUT_W, CSI_OUT_H, static_cast<float>(CSI_OUT_FPS));
    dashcam::record::Recorder csiRec;   // no overlay config: the debug feed has none
    csiRec.setLogCallback(log);

    // The debug feed records the already-scaled 640x480@20 CSI tee output, so the
    // recording bin needs no further downscale.  overlay=false: the IMX296 debug
    // feed does not need the telemetry overlay (straight to I420, no Cairo — cheaper).
    uint32_t cn, cd; Camera_GST::computeFpsRational((float)CSI_OUT_FPS, cn, cd);
    GstElement* csiBin = csiRec.createRecordingBin(
        csiFile, cn, cd, cfg.encoder,
        static_cast<uint32_t>(cfg.recording.queueDepth),
        dashcam::record::SourceMemory::NVMM,             // CSI -> nvvidconv inlet
        /*outWidth=*/0, /*outHeight=*/0, /*overlay=*/false);
    if (!csiBin) { std::cerr << "FAIL: CSI recording bin\n"; return 1; }
    csiCam.addBranch("debug", csiBin, /*leaky=*/false, /*initialEnabled=*/false);

    // ── start both ──────────────────────────────────────────────────────────────
    usbCam.open(); usbCam.setCameraVideoFormat((uint16_t)usbFmt); usbCam.start();
    csiCam.open(); csiCam.setCameraVideoFormat(0);                csiCam.start();

    cameraStatus us, cs; usbCam.getCameraStatus(us); csiCam.getCameraStatus(cs);
    std::cout << "USB status=" << (int)us.status << " err=" << (int)us.currentError
              << " | CSI status=" << (int)cs.status << " err=" << (int)cs.currentError << "\n";
    bool ok = (us.status == CAMERA_STATUS::RUNNING) && (cs.status == CAMERA_STATUS::RUNNING);
    if (!ok) {
        std::cerr << "FAIL: one or both pipelines did not reach RUNNING\n";
        usbRec.disconnect(); csiRec.disconnect();
        usbCam.close(); csiCam.close();
        return 1;
    }

    // ── warmup (AE/exposure settle) with valves closed ──────────────────────────
    std::vector<uint8_t> uwarm((size_t)uFmt.width * uFmt.height * 3 + 64);
    std::vector<uint8_t> cwarm((size_t)CSI_OUT_W * CSI_OUT_H * 3 + 64);  // CSI output is scaled
    uint32_t wr = 0;
    for (int i = 0; i < cfg.system.warmupFrames; ++i) {
        usbCam.captureFrame(uwarm.data(), (uint32_t)uwarm.size(), wr);
        csiCam.captureFrame(cwarm.data(), (uint32_t)cwarm.size(), wr);
    }

    // ── enable both recorders and run ───────────────────────────────────────────
    usbCam.setBranchEnabled("recording", true);
    csiCam.setBranchEnabled("debug", true);
    std::cout << "Recording (" << recSeconds << "s)...\n";

    std::atomic<bool> run{true};
    std::atomic<uint64_t> uFrames{0}, cFrames{0};
    std::thread ut(captureLoop, &usbCam, (uint32_t)(uFmt.width * uFmt.height * 3 + 64), &run, &uFrames);
    std::thread ct(captureLoop, &csiCam, (uint32_t)(CSI_OUT_W * CSI_OUT_H * 3 + 64), &run, &cFrames);

    // Feed the SAME evolving telemetry to both overlays (~10 Hz), like a live GPS/IMU.
    auto t0 = std::chrono::steady_clock::now();
    int tick = 0;
    while (std::chrono::steady_clock::now() - t0 < std::chrono::seconds(recSeconds)) {
        ++tick;
        dashcam::record::OverlayData od{
            10.7725 + tick * 0.0002,          // latitude
            106.6581 + tick * 0.0003,         // longitude
            50.0 + tick * 0.1,                // altitude m
            static_cast<float>((tick * 2) % 120),  // speed km/h
            0.5f,                             // accel m/s^2
            static_cast<float>((tick * 5) % 360),  // heading deg
            epochMs()
        };
        usbRec.setOverlayData(od);   // CSI debug feed has no overlay — not fed
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    run.store(false);
    ut.join(); ct.join();
    std::cout << "USB appsink frames=" << uFrames.load()
              << "  CSI appsink frames=" << cFrames.load() << "\n";

    // ── graceful shutdown (disconnect recorders BEFORE stopping cameras) ─────────
    usbRec.disconnect(); usbCam.stop(); usbCam.close();
    csiRec.disconnect(); csiCam.stop(); csiCam.close();

    // ── validate both clips ─────────────────────────────────────────────────────
    auto sizeKB = [](const std::string& f){ return fs::exists(f) ? fs::file_size(f)/1024 : 0; };
    uintmax_t uKB = sizeKB(usbFile), cKB = sizeKB(csiFile);
    std::cout << "\nUSB primary: " << usbFile << "  (" << uKB << " KB)\n";
    bool uOk = uKB > 100 && ffprobeOk(usbFile, uFmt.width, uFmt.height);
    std::cout << "CSI debug:   " << csiFile << "  (" << cKB << " KB)\n";
    bool cOk = cKB > 50 && ffprobeOk(csiFile, CSI_OUT_W, CSI_OUT_H);

    // The configs dir must hold the config copy, and the logs dir a run log (proof of
    // the footage / logs / configs three-way split landing in the right places).
    const std::string cfgCopy = configsDir + "/dashcam.xml";
    bool cfgOk = fs::exists(cfgCopy);
    bool logOk = false;
    for (const auto& e : fs::directory_iterator(logDir)) {
        const std::string fn = e.path().filename().string();
        if (fn.rfind("log_", 0) == 0 && e.path().extension() == ".txt") { logOk = true; break; }
    }
    std::cout << "Config copy: " << cfgCopy << "  " << (cfgOk ? "OK" : "MISSING") << "\n";
    std::cout << "Run log in : " << logDir << "  " << (logOk ? "OK" : "MISSING") << "\n";

    bool pass = uOk && cOk && cfgOk && logOk && uFrames.load() > 0 && cFrames.load() > 0;
    std::cout << "\nUSB primary record: " << (uOk ? "PASS" : "FAIL") << "\n";
    std::cout << "CSI debug record:   " << (cOk ? "PASS" : "FAIL") << "\n";
    std::cout << "\nRESULT: " << (pass ? "PASS" : "FAIL") << "\n";
    dashcam::log::shutdown();
    return pass ? 0 : 1;
}
