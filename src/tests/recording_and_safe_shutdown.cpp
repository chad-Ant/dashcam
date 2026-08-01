#include "libcamera_csi.h"
#include "libconfig.h"
#include "librecord.h"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <thread>
#include <vector>

using namespace dashcam::camera;

namespace fs = std::filesystem;

static constexpr uint16_t FALLBACK_FORMAT_INDEX = 4;  // 1080p60 on IMX296; used when config has no camera entries

// Return the formatIndex for the first CSI camera in cfg, or fallback if the list is empty.
static uint16_t csiFormatIndex(const dashcam::config::AppConfig& cfg) {
    for (const auto& cc : cfg.cameras)
        if (cc.type == "CSI")
            return static_cast<uint16_t>(cc.formatIndex);
    return FALLBACK_FORMAT_INDEX;
}

static bool checkRunning(Camera_CSI& cam, const char* phase) {
    cameraStatus st;
    cam.getCameraStatus(st);
    if (st.status != CAMERA_STATUS::RUNNING) {
        std::cerr << "[" << phase << "] camera not RUNNING after start(): "
                  << "status=" << static_cast<int>(st.status)
                  << " error=" << static_cast<int>(st.currentError) << "\n";
        return false;
    }
    return true;
}

static bool selectFormat(const cameraInfo& info, uint16_t idx) {
    if (idx >= info.videoFormats.size()) {
        std::cerr << "Format index " << idx << " out of range (have "
                  << info.videoFormats.size() << " formats).\n";
        return false;
    }
    const auto& fmt = info.videoFormats[idx];
    std::cout << "  Format[" << idx << "]: " << fmt.width << "x" << fmt.height
              << " @ " << fmt.frameRate << " fps (" << fmt.description << ")\n";
    return true;
}

static void ffprobe_streams(const std::string& filename) {
    std::cout << "  ffprobe streams:\n";
    std::string cmd =
        "ffprobe -v error -select_streams v:0"
        " -show_entries stream=codec_name,r_frame_rate,avg_frame_rate,nb_frames,duration"
        " -of default=noprint_wrappers=1 " + filename + " 2>&1";
    std::system(cmd.c_str());
}

bool test_recording_overlay(const cameraInfo& info,
                             const dashcam::config::AppConfig& cfg,
                             const dashcam::camera::AttributeDictionary& attrDict) {
    std::cout << "\n--- Test 1: Full Recording with Dynamic Telemetry ---\n";
    const uint16_t fmtIdx = csiFormatIndex(cfg);
    if (!selectFormat(info, fmtIdx)) return false;

    Camera_CSI cam(info);
    cam.setAttributeDictionary(attrDict);

    dashcam::record::Recorder recorder;
    recorder.setOverlayConfig(cfg.overlay);

    std::string filename = "./archive/test_overlay.mkv";
    if (fs::exists(filename)) fs::remove(filename);

    const auto& fmt = info.videoFormats[fmtIdx];
    // Record at the configured fps, never above the sensor rate — encoding a
    // 60 fps sensor natively is the classic no-NVENC CPU footgun.
    const float recFps = std::min(static_cast<float>(cfg.recording.recordFps), fmt.frameRate);
    uint32_t frNum, frDen;
    Camera_GST::computeFpsRational(recFps, frNum, frDen);
    std::cout << "  fps rational: " << frNum << "/" << frDen
              << " (record " << recFps << " from sensor " << fmt.frameRate << ")\n";

    GstElement* recBin = recorder.createRecordingBin(filename, frNum, frDen, cfg.encoder);
    if (!recBin) {
        std::cerr << "  createRecordingBin() failed\n";
        return false;
    }

    // initialEnabled=false: valve starts closed so AE converges before recording begins.
    cam.addBranch("recording", recBin, false, false);
    cam.open();
    cam.setCameraVideoFormat(fmtIdx);
    std::cout << "  Starting pipeline...\n";
    cam.start();
    if (!checkRunning(cam, "Test 1")) return false;
    std::cout << "  Pipeline RUNNING.\n";

    // Size the capture buffer from the ACTIVE format: captureFrame() discards
    // frames larger than the buffer, so a hardcoded 720p buffer silently
    // starves the appsink loop when the camera runs its native mode.
    std::vector<uint8_t> buffer((size_t)fmt.width * fmt.height * 3 + 64);
    int frames = 0;
    uint32_t written = 0;

    // Warm up so AE settles before recording starts.
    std::cout << "  Warming up (" << cfg.system.warmupFrames << " frames)...\n";
    for (int w = 0; w < cfg.system.warmupFrames; ++w)
        cam.captureFrame(buffer.data(), buffer.size(), written);
    cam.setBranchEnabled("recording", true);
    std::cout << "  Recording enabled.\n";

    auto start_time = std::chrono::steady_clock::now();
    auto last_report = start_time;
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5)) {
        cam.captureFrame(buffer.data(), buffer.size(), written);
        if (written > 0) {
            frames++;
            // Validity flags and per-source stamps set EXPLICITLY: they default
            // false (fail-closed, correct for a real feed that has not reported
            // yet), so the seven-field brace initialiser this replaces rendered
            // the whole overlay as dashes.
            const int64_t nowMs =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();
            dashcam::record::OverlayData od;
            od.latitude        = 10.7725  + (frames * 0.0001);
            od.longitude       = 106.6581 + (frames * 0.0001);
            od.altitudeM       = 52.3 + (frames * 0.05);
            od.speedKmh        = static_cast<float>(frames % 80);
            od.accelerationMs2 = 1.2f;
            od.headingDeg      = static_cast<float>(frames % 360);
            od.timestampMs     = nowMs;
            od.speedValid = od.accelValid = od.positionValid = od.headingValid = true;
            od.speedTimestampMs = od.accelTimestampMs = nowMs;
            od.positionTimestampMs = od.headingTimestampMs = nowMs;
            recorder.setOverlayData(od);
            // Print capture rate every second.
            auto now = std::chrono::steady_clock::now();
            if (now - last_report >= std::chrono::seconds(1)) {
                double elapsed = std::chrono::duration<double>(now - start_time).count();
                std::cout << "  t=" << static_cast<int>(elapsed)
                          << "s  frames=" << frames
                          << "  rate=" << static_cast<int>(frames / elapsed) << " fps\n";
                last_report = now;
            }
        }
    }

    double total_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time).count();
    std::cout << "  Recorded " << frames << " frames in " << total_s << "s"
              << " => " << static_cast<int>(frames / total_s) << " fps (appsink rate)\n";

    recorder.disconnect();
    cam.stop();
    cam.close();

    uintmax_t size = fs::exists(filename) ? fs::file_size(filename) : 0;
    std::cout << "  File size: " << size / 1024 << " KB\n";
    ffprobe_streams(filename);
    if (frames == 0) {
        std::cerr << "  FAIL: appsink delivered no frames during recording\n";
        return false;
    }
    return size > 100000;
}

bool test_graceful_mid_recording_stop(const cameraInfo& info,
                                       const dashcam::config::AppConfig& cfg,
                                       const dashcam::camera::AttributeDictionary& attrDict) {
    std::cout << "\n--- Test 2: Graceful Mid-Recording Stop (EOS path) ---\n";
    const uint16_t fmtIdx = csiFormatIndex(cfg);
    if (!selectFormat(info, fmtIdx)) return false;

    Camera_CSI cam(info);
    cam.setAttributeDictionary(attrDict);

    dashcam::record::Recorder recorder;
    recorder.setOverlayConfig(cfg.overlay);

    std::string filename = "./archive/test_shutdown.mkv";
    if (fs::exists(filename)) fs::remove(filename);

    const auto& fmt2 = info.videoFormats[fmtIdx];
    const float recFps2 = std::min(static_cast<float>(cfg.recording.recordFps), fmt2.frameRate);
    uint32_t frNum2, frDen2;
    Camera_GST::computeFpsRational(recFps2, frNum2, frDen2);
    std::cout << "  fps rational: " << frNum2 << "/" << frDen2
              << " (record " << recFps2 << " from sensor " << fmt2.frameRate << ")\n";

    GstElement* recBin = recorder.createRecordingBin(filename, frNum2, frDen2, cfg.encoder);
    if (!recBin) {
        std::cerr << "  createRecordingBin() failed\n";
        return false;
    }

    cam.addBranch("recording", recBin, false, false);
    cam.open();
    cam.setCameraVideoFormat(fmtIdx);
    std::cout << "  Starting pipeline...\n";
    cam.start();
    if (!checkRunning(cam, "Test 2")) return false;
    std::cout << "  Pipeline RUNNING.\n";

    std::vector<uint8_t> buffer((size_t)fmt2.width * fmt2.height * 3 + 64);
    int frames = 0;
    uint32_t written = 0;

    std::cout << "  Warming up (" << cfg.system.warmupFrames << " frames)...\n";
    for (int w = 0; w < cfg.system.warmupFrames; ++w)
        cam.captureFrame(buffer.data(), buffer.size(), written);
    cam.setBranchEnabled("recording", true);
    std::cout << "  Recording enabled.\n";

    auto start_time = std::chrono::steady_clock::now();
    auto last_report = start_time;
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(7)) {
        cam.captureFrame(buffer.data(), buffer.size(), written);
        if (written > 0) {
            frames++;
            auto now = std::chrono::steady_clock::now();
            if (now - last_report >= std::chrono::seconds(1)) {
                double elapsed = std::chrono::duration<double>(now - start_time).count();
                std::cout << "  t=" << static_cast<int>(elapsed)
                          << "s  frames=" << frames
                          << "  rate=" << static_cast<int>(frames / elapsed) << " fps\n";
                last_report = now;
            }
        }
    }

    double total_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time).count();
    std::cout << "  Recorded " << frames << " frames in " << total_s << "s"
              << " => " << static_cast<int>(frames / total_s) << " fps (appsink rate)\n";

    std::cout << "  Triggering mid-recording stop()...\n";
    recorder.disconnect();
    cam.stop();
    cam.close();

    uintmax_t size = fs::exists(filename) ? fs::file_size(filename) : 0;
    std::cout << "  File size: " << size / 1024 << " KB\n";
    if (size <= 50000) return false;
    if (frames == 0) {
        std::cerr << "  FAIL: appsink delivered no frames during recording\n";
        return false;
    }

    // Verify the file is a valid, playable MKV and print stream details.
    std::cout << "  Validating with ffprobe...\n";
    ffprobe_streams(filename);
    int rc = std::system(("ffprobe -v error -show_entries format=duration"
                          " -of default=noprint_wrappers=1:nokey=1 " + filename +
                          " 2>&1").c_str());
    if (rc != 0) {
        std::cerr << "  ffprobe rejected the file (rc=" << rc << ")\n";
        return false;
    }
    return true;
}

int main(int argc, char* argv[]) {
    gst_init(&argc, &argv);
    fs::create_directories("./archive");

    dashcam::config::AppConfig cfg;
    dashcam::config::ConfigReader::load("config/dashcam.xml", cfg);

    dashcam::camera::AttributeDictionary attrDict;
    dashcam::camera::AttributeDictionary::load("config/camera_attributes.xml", attrDict);

    std::vector<cameraInfo> cameras;
    if (getCameraList(cameras) != ERROR_CODE::NONE || cameras.empty()) {
        std::cerr << "No cameras found.\n";
        return 1;
    }

    auto it = std::find_if(cameras.begin(), cameras.end(),
        [](const cameraInfo& c){ return c.type == CAMERA_TYPE::CSI; });
    if (it == cameras.end()) {
        std::cerr << "No CSI camera found among " << cameras.size() << " devices.\n";
        return 1;
    }
    const cameraInfo& csiInfo = *it;

    std::cout << "Testing libcamera via " << csiInfo.address
              << " (Argus sensor-id " << csiInfo.deviceId << ")...\n";
    // speedPreset/tune are ConfigVar<std::string>: the free operator<<(ostream,string)
    // template won't apply ConfigVar's implicit conversion, so read into locals first.
    const std::string encPreset = cfg.encoder.speedPreset;
    const std::string encTune   = cfg.encoder.tune;
    std::cout << "Encoder: bitrate=" << cfg.encoder.bitrate
              << " preset=" << encPreset
              << " key-int-max=" << cfg.encoder.keyIntMax;
    if (!encTune.empty()) std::cout << " tune=" << encTune;
    std::cout << "\nWarmup frames: " << cfg.system.warmupFrames << "\n";

    bool t1 = test_recording_overlay(csiInfo, cfg, attrDict);
    bool t2 = test_graceful_mid_recording_stop(csiInfo, cfg, attrDict);

    std::cout << "\nTest 1 (Overlay):        " << (t1 ? "PASS" : "FAIL") << "\n";
    std::cout << "Test 2 (Mid-stop + MKV):  " << (t2 ? "PASS" : "FAIL") << "\n";

    return (t1 && t2) ? 0 : 1;
}
