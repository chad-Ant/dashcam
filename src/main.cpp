#include "libcamera_csi.h"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

static constexpr uint16_t TARGET_FORMAT_INDEX = 4;  // 720p60 on this sensor

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
    std::cout << "Selected format[" << idx << "]: " << fmt.width << "x" << fmt.height
              << "@" << fmt.frameRate << " (" << fmt.description << ")\n";
    return true;
}

bool test_recording_overlay(const cameraInfo& info) {
    std::cout << "\n--- Test 1: Full Recording with Dynamic Telemetry ---\n";
    if (!selectFormat(info, TARGET_FORMAT_INDEX)) return false;

    Camera_CSI cam(info);

    std::string filename = "./archive/test_overlay.mp4";
    if (fs::exists(filename)) fs::remove(filename);

    GstElement* recBin = cam.createRecordingBin(filename);
    if (!recBin) {
        std::cerr << "createRecordingBin() failed\n";
        return false;
    }

    cam.addBranch("recording", recBin, false);
    cam.open();
    cam.setCameraVideoFormat(TARGET_FORMAT_INDEX);
    cam.start();
    if (!checkRunning(cam, "Test 1")) return false;

    std::vector<uint8_t> buffer(1280 * 720 * 3);
    int frames = 0;
    uint32_t written = 0;

    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5)) {
        cam.captureFrame(buffer.data(), buffer.size(), written);
        if (written > 0) {
            frames++;
            cam.setOverlayData({
                10.7725 + (frames * 0.0001),
                106.6581 + (frames * 0.0001),
                52.3 + (frames * 0.05),
                static_cast<float>(frames % 80),
                1.2f,
                static_cast<float>(frames % 360),
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count()
            });
        }
    }

    cam.stop();
    cam.close();

    uintmax_t size = fs::exists(filename) ? fs::file_size(filename) : 0;
    std::cout << "Captured " << frames << " frames. File size: " << size / 1024 << " KB.\n";
    return size > 100000;
}

bool test_graceful_mid_recording_stop(const cameraInfo& info) {
    std::cout << "\n--- Test 2: Graceful Mid-Recording Stop (EOS path) ---\n";
    if (!selectFormat(info, TARGET_FORMAT_INDEX)) return false;

    Camera_CSI cam(info);

    std::string filename = "./archive/test_shutdown.mp4";
    if (fs::exists(filename)) fs::remove(filename);

    GstElement* recBin = cam.createRecordingBin(filename);
    if (!recBin) {
        std::cerr << "createRecordingBin() failed\n";
        return false;
    }

    cam.addBranch("recording", recBin, false);
    cam.open();
    cam.setCameraVideoFormat(TARGET_FORMAT_INDEX);
    cam.start();
    if (!checkRunning(cam, "Test 2")) return false;

    std::vector<uint8_t> buffer(1280 * 720 * 3);
    int frames = 0;
    uint32_t written = 0;

    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(2)) {
        cam.captureFrame(buffer.data(), buffer.size(), written);
        if (written > 0) frames++;
    }

    std::cout << "Triggering mid-recording stop()...\n";
    cam.stop();
    cam.close();

    uintmax_t size = fs::exists(filename) ? fs::file_size(filename) : 0;
    std::cout << "Captured " << frames << " frames. File size: " << size / 1024 << " KB.\n";
    if (size <= 50000) return false;

    // Verify the file is a valid, playable MP4 — exercises both the EOS-driven
    // moov finalisation in teardownPipeline() and the fragmented-MP4 fallback.
    std::cout << "Validating with ffprobe...\n";
    int rc = std::system(("ffprobe -v error -show_entries format=duration "
                          "-of default=noprint_wrappers=1:nokey=1 " + filename +
                          " > /dev/null 2>&1").c_str());
    if (rc != 0) {
        std::cerr << "ffprobe rejected the file (rc=" << rc << ")\n";
        return false;
    }
    std::cout << "ffprobe accepted the file.\n";
    return true;
}

int main(int argc, char* argv[]) {
    gst_init(&argc, &argv);
    fs::create_directories("./archive");

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

    bool t1 = test_recording_overlay(csiInfo);
    bool t2 = test_graceful_mid_recording_stop(csiInfo);

    std::cout << "\nTest 1 (Overlay):       " << (t1 ? "PASS" : "FAIL") << "\n";
    std::cout << "Test 2 (Mid-stop + MP4): " << (t2 ? "PASS" : "FAIL") << "\n";

    return (t1 && t2) ? 0 : 1;
}
