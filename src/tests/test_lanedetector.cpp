#include "libcamera.h"
#include "libcamera_csi.h"
#include "libcamera_usb.h"
#include "liblanedetector.h"
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

using namespace dashcam::camera;
using namespace dashcam::lane;

static const char* directionStr(LaneDirection d) {
    switch (d) {
        case LaneDirection::Straight: return "Straight";
        case LaneDirection::Left:     return "Left";
        case LaneDirection::Right:    return "Right";
        case LaneDirection::UTurn:    return "U-Turn";
    }
    return "Unknown";
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: lane_test <engine.engine>\n";
        return 1;
    }

    // ── discover cameras ──────────────────────────────────────────────────────

    std::vector<cameraInfo> cameras;
    if (getCameraList(cameras) != ERROR_CODE::NONE || cameras.empty()) {
        std::cerr << "ERROR: no cameras found\n";
        return 1;
    }

    const cameraInfo* info = nullptr;
    for (const auto& c : cameras) {
        if (c.type == CAMERA_TYPE::CSI || c.type == CAMERA_TYPE::USB) {
            info = &c;
            break;
        }
    }
    if (!info || info->videoFormats.empty()) {
        std::cerr << "ERROR: no usable camera with video formats found\n";
        return 1;
    }

    std::cout << "Using camera: " << info->address << "  ("
              << (info->type == CAMERA_TYPE::CSI ? "CSI" : "USB") << ")\n";

    // ── build camera ──────────────────────────────────────────────────────────

    std::unique_ptr<Camera_CSI> csiCam;
    std::unique_ptr<Camera_USB> usbCam;
    Camera_GST* cam = nullptr;

    LaneDetectorConfig cfg;
    cfg.enginePath = argv[1];

    if (info->type == CAMERA_TYPE::CSI) {
        csiCam = std::make_unique<Camera_CSI>(*info);
        cam    = csiCam.get();
        cfg.gstConversion = "nvvidconv ! video/x-raw,format=BGRx ! videoconvert";
    } else {
        usbCam = std::make_unique<Camera_USB>(*info);
        cam    = usbCam.get();
        cfg.gstConversion = "videoconvert";
    }

    // ── build detector ────────────────────────────────────────────────────────

    const uint32_t srcW = info->videoFormats[0].width;
    const uint32_t srcH = info->videoFormats[0].height;

    LaneDetector detector(srcW, srcH, cfg);
    cam->addBranch("lanes", detector.createBin(), /*leaky=*/true);

    // ── start pipeline ────────────────────────────────────────────────────────

    cam->open();
    cam->setCameraVideoFormat(0);
    cam->start();
    detector.start();

    std::cout << "Running for 10 seconds...\n\n";

    for (int t = 1; t <= 10; ++t) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        const LaneResult r = detector.poll();

        std::cout << "t+" << t << "s\n";
        std::cout << "  Number of lanes : " << static_cast<int>(r.numLanes) << "\n";

        if (r.numLanes == 0) {
            std::cout << "  Current lane    : none detected\n";
            std::cout << "  Direction       : unknown\n";
        } else if (r.currentLaneIndex < 0) {
            std::cout << "  Current lane    : out of bounds\n";
            std::cout << "  Direction       : unknown\n";
        } else {
            std::cout << "  Current lane    : " << static_cast<int>(r.currentLaneIndex) << "\n";
            std::cout << "  Direction       : "
                      << directionStr(r.laneAllowedDirections[r.currentLaneIndex]) << "\n";
        }
        std::cout << "\n";
    }

    // ── stop ──────────────────────────────────────────────────────────────────

    cam->stop();      // flushes appsink → inference thread drains
    detector.stop();  // joins inference thread
    cam->close();

    return 0;
}
