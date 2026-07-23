// lane_test — validates liblanedetector (UFLD v2 TensorRT) two ways:
//
//   lane_test <engine>                          live IMX296 smoke test (10 s)
//   lane_test <engine> --video <file> [w h [s]] real-footage decode validation
//
// Live mode targets the IMX296 CSI camera (selected by sysfs sensor name, so
// probe order can't hand us the wrong sensor) and proves the Camera_GST
// integration end-to-end: branch crop to the lower half, 20 fps inlet cap,
// inference thread, clean teardown.  Lane content is not asserted — the bench
// camera faces a room, not a road.
//
// Video mode is the functional check: frames from a real driving clip run
// through the same detector bin (crop + rate cap included) and the decoded
// results must contain lanes for a healthy fraction of the run.
//
// Operator-tunable knobs come from libconfig's DetectionConfig (<Detection>),
// mirroring how the production app consumes them.

#include "libcamera.h"
#include "libcamera_csi.h"
#include "libcamera_usb.h"
#include "libconfig.h"
#include "liblanedetector.h"

#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
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

static void printResult(int t, const LaneResult& r) {
    if (!r.valid) {
        std::cout << "t+" << t << "s  warming up (no valid result)\n";
        return;
    }
    std::cout << "t+" << t << "s  lanes=" << static_cast<int>(r.numLanes);
    if (r.numLanes == 0) {
        std::cout << "  (none detected)";
    } else if (r.currentLaneIndex < 0) {
        std::cout << "  ego=out-of-bounds";
    } else {
        std::cout << "  ego=" << static_cast<int>(r.currentLaneIndex)
                  << "  dir="
                  << directionStr(r.laneAllowedDirections[r.currentLaneIndex]);
        if (r.lateralValid)
            std::cout << "  offset=" << r.lateralOffset;
    }
    std::cout << "\n";
}

// Fill the library struct from libconfig's <Detection> section — the same
// consumption pattern the production app uses.
static LaneDetectorConfig makeLaneConfig(
        const dashcam::config::DetectionConfig& d, const std::string& engine) {
    LaneDetectorConfig c;
    c.enginePath      = engine.empty() ? (std::string)d.laneEnginePath : engine;
    c.targetHz        = static_cast<uint32_t>((int)d.laneTargetHz);
    c.branchMaxFps    = static_cast<uint32_t>((int)d.laneBranchMaxFps);
    c.inputCropTop    = (float)d.laneInputCropTop;
    c.inputCropBottom = (float)d.laneInputCropBottom;
    c.laneReferenceY  = (float)d.laneReferenceY;
    return c;
}

// True if this video node's sensor is the IMX296 (sysfs card name) — same
// probe-order-proof selection dashcam_v0_1 uses.
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

// ─── video-file mode ─────────────────────────────────────────────────────────

static int runVideoMode(const std::string& engine, const std::string& video,
                        uint32_t w, uint32_t h, int seconds) {
    dashcam::config::DetectionConfig det;   // defaults; file feed = system memory
    LaneDetectorConfig cfg = makeLaneConfig(det, engine);
    cfg.sourceIsNVMM = false;

    LaneDetector detector(w, h, cfg);
    GstElement* bin = detector.createBin();
    if (!bin) { std::cerr << "ERROR: createBin failed\n"; return 1; }

    const std::string desc =
        "filesrc location=\"" + video + "\" ! decodebin ! videoconvert "
        "! videoscale ! video/x-raw,format=BGR,width=" + std::to_string(w)
        + ",height=" + std::to_string(h) + " ! identity name=feed";
    GError* err = nullptr;
    GstElement* pipe = gst_parse_launch(desc.c_str(), &err);
    if (!pipe || err) {
        std::cerr << "ERROR: pipeline parse: "
                  << (err ? err->message : "?") << "\n";
        if (err) g_error_free(err);
        if (bin) gst_object_unref(bin);
        return 1;
    }

    gst_bin_add(GST_BIN(pipe), bin);
    GstElement* feed = gst_bin_get_by_name(GST_BIN(pipe), "feed");
    if (!feed || !gst_element_link(feed, bin)) {
        std::cerr << "ERROR: could not link feed -> detector bin\n";
        if (feed) gst_object_unref(feed);
        gst_object_unref(pipe);
        return 1;
    }
    gst_object_unref(feed);

    if (gst_element_set_state(pipe, GST_STATE_PLAYING)
            == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "ERROR: pipeline refused to play\n";
        gst_object_unref(pipe);
        return 1;
    }
    detector.start();

    GstBus* bus = gst_element_get_bus(pipe);
    int total = 0, withLanes = 0, withEgo = 0;
    bool error = false;

    const int ticks = seconds * 2;                    // poll every 500 ms
    for (int t = 0; t < ticks; ++t) {
        GstMessage* msg = gst_bus_timed_pop_filtered(
            bus, 500 * GST_MSECOND,
            static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
        if (msg) {
            const bool eos = GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS;
            if (!eos) {
                GError* e = nullptr;
                gst_message_parse_error(msg, &e, nullptr);
                std::cerr << "ERROR on bus: " << (e ? e->message : "?") << "\n";
                if (e) g_error_free(e);
                error = true;
            }
            gst_message_unref(msg);
            if (eos) std::cout << "(end of clip)\n";
            break;
        }

        const LaneResult r = detector.poll();
        if (t >= 2) {                                 // 1 s pipeline warm-up
            ++total;
            if (r.numLanes >= 1)        ++withLanes;
            if (r.currentLaneIndex >= 0) ++withEgo;
        }
        if (t % 2 == 0) printResult(t / 2, r);
    }

    gst_element_set_state(pipe, GST_STATE_NULL);
    detector.stop();
    gst_object_unref(bus);
    gst_object_unref(pipe);

    std::cout << "\npolls=" << total << "  with-lanes=" << withLanes
              << "  with-ego-lane=" << withEgo << "\n";
    const bool pass = !error && total > 0
                   && withLanes * 2 >= total          // lanes in >= 50% of polls
                   && withEgo   * 3 >= total;         // ego lane in >= 33%
    std::cout << (pass ? "VIDEO MODE: PASS" : "VIDEO MODE: FAIL") << "\n";
    return pass ? 0 : 1;
}

// ─── live-camera mode (IMX296 on CSI) ────────────────────────────────────────

static int runLiveMode(const std::string& engine) {
    std::vector<cameraInfo> cameras;
    if (getCameraList(cameras) != ERROR_CODE::NONE || cameras.empty()) {
        std::cerr << "ERROR: no cameras found\n";
        return 1;
    }

    const cameraInfo* info = nullptr;
    for (const auto& c : cameras)
        if (c.type == CAMERA_TYPE::CSI && isImx296(c)) { info = &c; break; }
    if (!info || info->videoFormats.empty()) {
        std::cerr << "ERROR: no IMX296 CSI camera found\n";
        return 1;
    }

    std::cout << "Using camera: " << info->address
              << "  (IMX296, Argus sensor-id " << info->deviceId << ")\n";

    Camera_CSI cam(*info);

    dashcam::config::DetectionConfig det;   // <Detection> defaults: 20 Hz,
    LaneDetectorConfig cfg = makeLaneConfig(det, engine);   // crop 0.5, 20 fps
    cfg.sourceIsNVMM = true;

    const uint32_t srcW = info->videoFormats[0].width;
    const uint32_t srcH = info->videoFormats[0].height;

    LaneDetector detector(srcW, srcH, cfg);
    cam.addBranch("lanes", detector.createBin(), /*leaky=*/true);

    cam.open();
    cam.setCameraVideoFormat(0);
    cam.start();

    cameraStatus st;
    cam.getCameraStatus(st);
    if (st.status != CAMERA_STATUS::RUNNING) {
        std::cerr << "ERROR: camera failed to start\n";
        cam.close();
        return 1;
    }
    cam.setCaptureEnabled(false);
    detector.start();

    std::cout << "Running for 10 seconds...\n\n";
    int validPolls = 0;
    for (int t = 1; t <= 10; ++t) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        const LaneResult result = detector.poll();
        if (result.valid) ++validPolls;
        printResult(t, result);
    }

    cam.stop();       // flushes appsink → inference thread drains
    detector.stop();  // joins inference thread
    cam.close();

    cam.getCameraStatus(st);
    // A clean teardown alone is not evidence that Argus delivered a frame.
    // Allow three seconds of warm-up, then require live inference results.
    const bool pass = st.status == CAMERA_STATUS::CLOSED
                   && detector.processedFrameCount() > 0
                   && validPolls >= 7;
    std::cout << "\nvalid-polls=" << validPolls << "/10"
              << " processed=" << detector.processedFrameCount() << "\n";
    std::cout << "\nLIVE MODE: " << (pass ? "PASS" : "FAIL")
              << " (live inference verified, teardown "
              << (pass ? "clean" : "dirty") << ")\n";
    return pass ? 0 : 1;
}

// ─── main ────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: lane_test <engine.engine>\n"
                     "       lane_test <engine.engine> --video <file> "
                     "[width height [seconds]]\n";
        return 1;
    }
    gst_init(&argc, &argv);

    const std::string engine = argv[1];
    if (argc >= 4 && std::strcmp(argv[2], "--video") == 0) {
        const std::string video = argv[3];
        const uint32_t w = argc > 5 ? std::stoul(argv[4]) : 1280;
        const uint32_t h = argc > 5 ? std::stoul(argv[5]) : 720;
        const int      s = argc > 6 ? std::stoi(argv[6])  : 60;
        return runVideoMode(engine, video, w, h, s);
    }
    return runLiveMode(engine);
}
