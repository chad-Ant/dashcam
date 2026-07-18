// test_single_record.cpp — one-camera-at-a-time record + capture validation.
//
// Usage: single_record_test <CSI|USB> [index] [seconds] [csiFmtIdx]
//   type       Camera type to test (CSI or USB).
//   index      Which camera of that type, in getCameraList() discovery order
//              (default 0).  With two CSI sensors fitted, CSI 0 and CSI 1
//              exercise each port/sensor individually.
//   seconds    Recording duration (default 4).
//   csiFmtIdx  CSI only: record NATIVE at this videoFormats index (no VIC
//              downscale) — for image-quality inspection.  Framerate is capped
//              at cfg.recording.recordFps to spare the CPU encoder.  Omitted:
//              fmt[0] downscaled to 640x480@20 (the production debug feed).
//
// Validates the "any camera can record footage or feed inference, one at a
// time, given a correct config" requirement: the selected camera runs alone
// with a recording branch (librecord) AND a captureFrame() consumer (the
// inference feed), then the clip is ffprobe-verified.  Sensor-specific
// behaviour comes from enumeration (getCameraList) and the config
// (dashcam.xml pipeline/encoder sections) — nothing here is per-sensor.
//
// CSI cameras record the whole-output VIC downscale (640x480@20, like the
// production debug feed) so multi-megapixel sensors (IMX219 3280x2464) don't
// swamp the software encoder; USB cameras record their picked native mode.
// Orin Nano note: no NVENC — encoding is x264enc on the CPU.

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
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace dashcam::camera;
namespace fs = std::filesystem;

// CSI whole-output downscale (VIC), matching the production debug feed.
static constexpr uint32_t CSI_OUT_W   = 640;
static constexpr uint32_t CSI_OUT_H   = 480;
static constexpr int      CSI_OUT_FPS = 20;

// ─── helpers (same shapes as test_dual_recording.cpp) ─────────────────────────

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

// ─── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    gst_init(&argc, &argv);
    if (argc < 2) {
        std::cerr << "usage: " << argv[0] << " <CSI|USB> [index] [seconds]\n";
        return 2;
    }
    const std::string wantType = argv[1];
    if (wantType != "CSI" && wantType != "USB") {
        std::cerr << "type must be CSI or USB\n";
        return 2;
    }
    const int wantIdx    = (argc > 2) ? std::atoi(argv[2]) : 0;
    const int recSeconds = (argc > 3) ? std::max(2, std::atoi(argv[3])) : 4;
    const int csiFmtIdx  = (argc > 4) ? std::atoi(argv[4]) : -1;  // -1 = scaled debug feed

    // ── config + logging (same resolution chain as the app) ──────────────────
    dashcam::config::AppConfig cfg;
    dashcam::config::ConfigReader::load("config/dashcam.xml", cfg);
    const std::string logDir = dashcam::config::resolveStorageDir(
        dashcam::config::kDefaultLogDir, dashcam::config::kFallbackLogName);
    dashcam::log::init(logDir);
    auto log = dashcam::log::getCallback();
    dashcam::camera::AttributeDictionary dict;
    dashcam::camera::AttributeDictionary::load("config/camera_attributes.xml", dict);
    const std::string footageDir = dashcam::config::resolveStorageDir(
        cfg.system.footagePath, dashcam::config::kFallbackFootageName, log);

    // ── select the requested camera ───────────────────────────────────────────
    std::vector<cameraInfo> cams;
    if (getCameraList(cams, log) != ERROR_CODE::NONE || cams.empty()) {
        std::cerr << "FAIL: no cameras discovered\n";
        return 1;
    }
    const CAMERA_TYPE type = (wantType == "CSI") ? CAMERA_TYPE::CSI : CAMERA_TYPE::USB;
    const cameraInfo* info = nullptr;
    int seen = 0;
    for (const auto& c : cams) {
        if (c.type != type) continue;
        if (seen++ == wantIdx) { info = &c; break; }
    }
    if (!info) {
        std::cerr << "FAIL: no " << wantType << " camera at index " << wantIdx
                  << " (found " << seen << " of that type)\n";
        return 1;
    }

    // ── pick the capture format and expected recording geometry ──────────────
    const bool isCsi     = (type == CAMERA_TYPE::CSI);
    const bool csiNative = isCsi && csiFmtIdx >= 0;   // record full sensor output
    int fmtIdx = 0;
    if (type == CAMERA_TYPE::USB) fmtIdx = pickUsbRecordFormat(*info);
    if (csiNative)                fmtIdx = csiFmtIdx;
    if (fmtIdx < 0 || (size_t)fmtIdx >= info->videoFormats.size()) {
        std::cerr << "FAIL: no usable capture format (index " << fmtIdx << " of "
                  << info->videoFormats.size() << ")\n";
        return 1;
    }
    const auto& fmt = info->videoFormats[(size_t)fmtIdx];

    const uint32_t recW   = (isCsi && !csiNative) ? CSI_OUT_W : fmt.width;
    const uint32_t recH   = (isCsi && !csiNative) ? CSI_OUT_H : fmt.height;
    const float    recFps = (isCsi && !csiNative)
                          ? (float)CSI_OUT_FPS
                          : std::min((float)cfg.recording.recordFps, fmt.frameRate);

    std::cout << "=== Single-camera record test ===\n"
              << wantType << "[" << wantIdx << "]  " << info->address
              << "  deviceId=" << info->deviceId
              << "  fmt[" << fmtIdx << "] " << fmt.width << "x" << fmt.height
              << " @ " << fmt.frameRate << " (" << fmt.description << ")\n"
              << "record " << recW << "x" << recH << " @ " << recFps
              << " for " << recSeconds << "s -> " << footageDir << "\n";

    // ── camera + recorder ─────────────────────────────────────────────────────
    std::unique_ptr<Camera_GST> cam;
    if (isCsi) cam = std::make_unique<Camera_CSI>(*info);
    else       cam = std::make_unique<Camera_USB>(*info);
    cam->setLogCallback(log);
    cam->setAttributeDictionary(dict);
    cam->setPipelineParams(makePipelineParams(cfg.pipeline));
    if (isCsi && !csiNative)
        cam->setOutputResolution(CSI_OUT_W, CSI_OUT_H, (float)CSI_OUT_FPS);

    const std::string file = footageDir + "/single_" + wantType +
                             std::to_string(wantIdx) + "_" + utcTimestamp() + ".mkv";
    if (fs::exists(file)) fs::remove(file);

    dashcam::record::Recorder rec;
    rec.setLogCallback(log);
    uint32_t rn, rd; Camera_GST::computeFpsRational(recFps, rn, rd);
    GstElement* bin = rec.createRecordingBin(
        file, rn, rd, cfg.encoder,
        static_cast<uint32_t>(cfg.recording.queueDepth),
        isCsi ? dashcam::record::SourceMemory::NVMM
              : dashcam::record::SourceMemory::System,
        /*outWidth=*/0, /*outHeight=*/0, /*overlay=*/false);
    if (!bin) { std::cerr << "FAIL: recording bin\n"; return 1; }
    cam->addBranch("recording", bin, /*leaky=*/false, /*initialEnabled=*/false);

    // ── run: open → format → start → warmup → record → stop ──────────────────
    cam->open();
    cam->setCameraVideoFormat((uint16_t)fmtIdx);
    cam->start();
    cameraStatus st;
    cam->getCameraStatus(st);
    if (st.status != CAMERA_STATUS::RUNNING) {
        std::cerr << "FAIL: not RUNNING (status=" << (int)st.status
                  << " err=" << (int)st.currentError << ")\n";
        rec.disconnect(); cam->close();
        return 1;
    }

    std::vector<uint8_t> buf((size_t)recW * recH * 3 + 64);
    if (!isCsi) buf.resize((size_t)fmt.width * fmt.height * 3 + 64);
    uint32_t written = 0;
    for (int i = 0; i < cfg.system.warmupFrames; ++i)
        cam->captureFrame(buf.data(), (uint32_t)buf.size(), written);

    cam->setBranchEnabled("recording", true);
    uint64_t frames = 0;
    const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::seconds(recSeconds);
    while (std::chrono::steady_clock::now() < deadline) {
        cam->captureFrame(buf.data(), (uint32_t)buf.size(), written);
        if (written > 0) ++frames;
    }
    std::cout << "appsink frames=" << frames << " in " << recSeconds << "s\n";

    rec.disconnect();
    cam->stop();
    cam->close();

    // ── validate ──────────────────────────────────────────────────────────────
    const uintmax_t kb = fs::exists(file) ? fs::file_size(file) / 1024 : 0;
    std::cout << "clip: " << file << " (" << kb << " KB)\n";
    bool ok = frames > 0 && kb > 20 && ffprobeOk(file, recW, recH);
    std::cout << "RESULT: " << (ok ? "PASS" : "FAIL") << "\n";
    return ok ? 0 : 1;
}
