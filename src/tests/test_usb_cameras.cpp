// USB camera function test — Jetson Orin Nano
//
// Single-camera tests (Tests 1-7):  always run against the first USB camera found.
// Dual-camera tests   (Tests 8-10): run only when 2+ USB cameras are present.
//
// Usage: ./usb_test
// Requires: UVC cameras at /dev/videoN, GStreamer 1.0 with v4l2src + videoconvert

#include "libcamera_usb.h"
#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <gst/gst.h>
#include <iostream>
#include <thread>
#include <vector>

using namespace dashcam::camera;

namespace fs = std::filesystem;

// ─── helpers ──────────────────────────────────────────────────────────────────

static bool checkRunning(Camera_USB& cam, const char* phase) {
    cameraStatus st;
    cam.getCameraStatus(st);
    if (st.status != CAMERA_STATUS::RUNNING) {
        std::cerr << "  [" << phase << "] not RUNNING:"
                  << " status=" << static_cast<int>(st.status)
                  << " error="  << static_cast<int>(st.currentError) << "\n";
        return false;
    }
    return true;
}

static void printFormats(const cameraInfo& info) {
    for (size_t i = 0; i < info.videoFormats.size(); ++i) {
        const auto& f = info.videoFormats[i];
        printf("    [%zu] %ux%u @ %.1f fps  (%s)\n",
               i, f.width, f.height, static_cast<double>(f.frameRate), f.description.c_str());
    }
}

// Capture BGR frames for `duration_s` seconds; return total frame count.
// Allocates a buffer sized for the given format.
static int captureFor(Camera_USB& cam, const cameraVideoFormat& fmt, int duration_s) {
    const uint32_t bufSize = fmt.width * fmt.height * 3;
    std::vector<uint8_t> buf(bufSize);
    uint32_t written = 0;
    int frames = 0;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(duration_s);
    while (std::chrono::steady_clock::now() < deadline) {
        cam.captureFrame(buf.data(), bufSize, written);
        if (written > 0) frames++;
    }
    return frames;
}

// Minimum acceptable frame count: 50 % of theoretical max over `duration_s`.
static int minFrames(const cameraVideoFormat& fmt, int duration_s) {
    return static_cast<int>(fmt.frameRate * static_cast<float>(duration_s) * 0.5f);
}

// ─── single-camera tests ──────────────────────────────────────────────────────

// Test 1: open → close without ever starting the pipeline.
static bool test_open_close(const cameraInfo& info) {
    std::cout << "\n--- Test 1: Open/Close Lifecycle ---\n";
    Camera_USB cam(info);

    cam.open();
    cameraStatus st;
    cam.getCameraStatus(st);
    if (st.status != CAMERA_STATUS::OPEN) {
        std::cerr << "  open() did not reach OPEN (status=" << static_cast<int>(st.status)
                  << " error=" << static_cast<int>(st.currentError) << ")\n";
        return false;
    }
    std::cout << "  open() -> OPEN\n";

    cam.close();
    cam.getCameraStatus(st);
    if (st.status != CAMERA_STATUS::CLOSED) {
        std::cerr << "  close() did not reach CLOSED\n";
        return false;
    }
    std::cout << "  close() -> CLOSED\n";
    return true;
}

// Test 2: sustained frame delivery at the camera's advertised frame rate.
// Captures format[0] for 3 s; expects >= 50 % of theoretical frame count.
static bool test_frame_capture(const cameraInfo& info) {
    std::cout << "\n--- Test 2: Frame Capture (format[0], 3 s) ---\n";
    if (info.videoFormats.empty()) {
        std::cerr << "  No formats enumerated\n";
        return false;
    }

    Camera_USB cam(info);
    const auto& fmt = info.videoFormats[0];
    printf("  Using format[0]: %ux%u @ %.1f fps (%s)\n",
           fmt.width, fmt.height, static_cast<double>(fmt.frameRate), fmt.description.c_str());

    cam.open();
    cam.setCameraVideoFormat(0);
    cam.start();
    if (!checkRunning(cam, "Test 2")) { cam.close(); return false; }

    int frames = captureFor(cam, fmt, 3);
    cam.stop();
    cam.close();

    int expected = minFrames(fmt, 3);
    bool ok = frames >= expected;
    printf("  Received %d frames in 3 s (expected >= %d): %s\n",
           frames, expected, ok ? "OK" : "FAIL");
    return ok;
}

// Test 3: attribute queued before start() is applied without error.
// Sets brightness=128; the attribute should be flushed by start() with no pipeline error.
static bool test_attribute_before_start(const cameraInfo& info) {
    std::cout << "\n--- Test 3: Attribute Queued Before start() ---\n";
    if (info.videoFormats.empty()) { std::cerr << "  No formats\n"; return false; }

    Camera_USB cam(info);
    cam.setCameraAttribute("brightness", "128");
    cam.open();
    cam.setCameraVideoFormat(0);
    cam.start();
    if (!checkRunning(cam, "Test 3")) { cam.close(); return false; }

    cameraStatus st;
    cam.getCameraStatus(st);
    bool ok = (st.currentError == ERROR_CODE::NONE);
    printf("  currentError after start: %d %s\n",
           static_cast<int>(st.currentError), ok ? "(OK)" : "(FAIL — expected NONE)");

    cam.stop();
    cam.close();
    return ok;
}

// Test 4: attribute written while RUNNING is applied immediately.
static bool test_attribute_while_running(const cameraInfo& info) {
    std::cout << "\n--- Test 4: Attribute Set While Running ---\n";
    if (info.videoFormats.empty()) { std::cerr << "  No formats\n"; return false; }

    Camera_USB cam(info);
    cam.open();
    cam.setCameraVideoFormat(0);
    cam.start();
    if (!checkRunning(cam, "Test 4")) { cam.close(); return false; }

    cam.setCameraAttribute("brightness", "100");
    cameraStatus st;
    cam.getCameraStatus(st);
    bool ok = (st.currentError == ERROR_CODE::NONE);
    printf("  brightness=100 while RUNNING: error=%d %s\n",
           static_cast<int>(st.currentError), ok ? "(OK)" : "(FAIL)");

    cam.stop();
    cam.close();
    return ok;
}

// Test 5: unrecognised attribute name sets INVALID_ATTRIBUTE.
static bool test_attribute_invalid(const cameraInfo& info) {
    std::cout << "\n--- Test 5: Invalid Attribute Name ---\n";
    if (info.videoFormats.empty()) { std::cerr << "  No formats\n"; return false; }

    Camera_USB cam(info);
    cam.open();
    cam.setCameraVideoFormat(0);
    cam.start();
    if (!checkRunning(cam, "Test 5")) { cam.close(); return false; }

    cam.setCameraAttribute("not_a_real_control", "42");
    cameraStatus st;
    cam.getCameraStatus(st);
    bool ok = (st.currentError == ERROR_CODE::INVALID_ATTRIBUTE);
    printf("  \"not_a_real_control\" -> error=%d %s\n",
           static_cast<int>(st.currentError),
           ok ? "(OK — INVALID_ATTRIBUTE)" : "(FAIL — expected INVALID_ATTRIBUTE)");

    cam.stop();
    cam.close();
    return ok;
}

// Test 6: stop() → start() cycle without close/open.
// Verifies the pipeline can be fully torn down and rebuilt in-place.
static bool test_stop_restart(const cameraInfo& info) {
    std::cout << "\n--- Test 6: Stop/Restart Cycle (no close/open) ---\n";
    if (info.videoFormats.empty()) { std::cerr << "  No formats\n"; return false; }

    Camera_USB cam(info);
    const auto& fmt = info.videoFormats[0];

    cam.open();
    cam.setCameraVideoFormat(0);
    cam.start();
    if (!checkRunning(cam, "Test 6 first start")) { cam.close(); return false; }

    int frames1 = captureFor(cam, fmt, 2);
    printf("  First run: %d frames in 2 s\n", frames1);
    cam.stop();

    cameraStatus st;
    cam.getCameraStatus(st);
    if (st.status != CAMERA_STATUS::OPEN) {
        std::cerr << "  stop() did not return to OPEN\n";
        cam.close();
        return false;
    }

    cam.start();
    if (!checkRunning(cam, "Test 6 second start")) { cam.close(); return false; }

    int frames2 = captureFor(cam, fmt, 2);
    printf("  Second run: %d frames in 2 s\n", frames2);
    cam.stop();
    cam.close();

    int expected = minFrames(fmt, 2);
    bool ok = frames1 >= expected && frames2 >= expected;
    printf("  Both runs >= %d frames: %s\n", expected, ok ? "OK" : "FAIL");
    return ok;
}

// Test 7: format cycling — open with format[0], then with a format at a different
// resolution (if one exists).  Verifies the pipeline builds for multiple formats.
static bool test_format_cycling(const cameraInfo& info) {
    std::cout << "\n--- Test 7: Format Cycling ---\n";
    if (info.videoFormats.empty()) { std::cerr << "  No formats\n"; return false; }

    // Pick format[0] and the first format with a different width, if any.
    std::vector<uint16_t> indices = { 0 };
    for (uint16_t i = 1; i < static_cast<uint16_t>(info.videoFormats.size()); ++i) {
        if (info.videoFormats[i].width != info.videoFormats[0].width) {
            indices.push_back(i);
            break;
        }
    }

    for (uint16_t idx : indices) {
        Camera_USB cam(info);
        const auto& fmt = info.videoFormats[idx];
        printf("  Format[%u]: %ux%u @ %.1f fps\n",
               static_cast<unsigned>(idx), fmt.width, fmt.height,
               static_cast<double>(fmt.frameRate));

        cam.open();
        cam.setCameraVideoFormat(idx);
        cam.start();
        if (!checkRunning(cam, ("Test 7 fmt" + std::to_string(idx)).c_str())) {
            cam.close();
            return false;
        }

        int frames   = captureFor(cam, fmt, 2);
        int expected = minFrames(fmt, 2);
        printf("    Frames: %d (expected >= %d): %s\n",
               frames, expected, frames >= expected ? "OK" : "FAIL");
        cam.stop();
        cam.close();

        if (frames < expected) return false;
    }
    return true;
}

// ─── dual-camera tests ────────────────────────────────────────────────────────

// Test 8: both cameras open and running at the same time; capture alternates
// between them in a single thread to keep scheduling overhead minimal.
static bool test_dual_simultaneous(const cameraInfo& info0, const cameraInfo& info1) {
    std::cout << "\n--- Test 8: Dual Simultaneous Capture (serial alternating) ---\n";
    if (info0.videoFormats.empty() || info1.videoFormats.empty()) {
        std::cerr << "  Missing formats on one or both cameras\n";
        return false;
    }

    Camera_USB cam0(info0), cam1(info1);

    cam0.open(); cam0.setCameraVideoFormat(0); cam0.start();
    if (!checkRunning(cam0, "Test 8 cam0")) { cam0.close(); return false; }

    cam1.open(); cam1.setCameraVideoFormat(0); cam1.start();
    if (!checkRunning(cam1, "Test 8 cam1")) {
        cam0.stop(); cam0.close(); cam1.close(); return false;
    }

    std::cout << "  Both cameras RUNNING. Alternating capture for 3 s...\n";

    const auto& fmt0 = info0.videoFormats[0];
    const auto& fmt1 = info1.videoFormats[0];
    const uint32_t sz0 = fmt0.width * fmt0.height * 3;
    const uint32_t sz1 = fmt1.width * fmt1.height * 3;
    std::vector<uint8_t> buf0(sz0), buf1(sz1);
    uint32_t written = 0;
    int frames0 = 0, frames1 = 0;

    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
        cam0.captureFrame(buf0.data(), sz0, written); if (written > 0) frames0++;
        cam1.captureFrame(buf1.data(), sz1, written); if (written > 0) frames1++;
    }

    cam0.stop(); cam0.close();
    cam1.stop(); cam1.close();

    // Dual alternating is slower than single; use 40 % threshold.
    int min0 = static_cast<int>(fmt0.frameRate * 3.0f * 0.4f);
    int min1 = static_cast<int>(fmt1.frameRate * 3.0f * 0.4f);
    bool ok = frames0 >= min0 && frames1 >= min1;
    printf("  %s: %d frames (>= %d): %s\n",
           info0.address.c_str(), frames0, min0, frames0 >= min0 ? "OK" : "FAIL");
    printf("  %s: %d frames (>= %d): %s\n",
           info1.address.c_str(), frames1, min1, frames1 >= min1 ? "OK" : "FAIL");
    return ok;
}

// Test 9: concurrent capture from two separate threads, one thread per camera.
static bool test_dual_concurrent_threads(const cameraInfo& info0, const cameraInfo& info1) {
    std::cout << "\n--- Test 9: Dual Concurrent Capture (separate threads) ---\n";
    if (info0.videoFormats.empty() || info1.videoFormats.empty()) {
        std::cerr << "  Missing formats\n";
        return false;
    }

    Camera_USB cam0(info0), cam1(info1);

    cam0.open(); cam0.setCameraVideoFormat(0); cam0.start();
    if (!checkRunning(cam0, "Test 9 cam0")) { cam0.close(); return false; }

    cam1.open(); cam1.setCameraVideoFormat(0); cam1.start();
    if (!checkRunning(cam1, "Test 9 cam1")) {
        cam0.stop(); cam0.close(); cam1.close(); return false;
    }

    std::cout << "  Both cameras RUNNING. Concurrent capture for 3 s...\n";

    std::atomic<int> frames0{0}, frames1{0};

    auto worker = [](Camera_USB& cam, const cameraVideoFormat& fmt,
                     std::atomic<int>& counter, int secs) {
        const uint32_t bufSize = fmt.width * fmt.height * 3;
        std::vector<uint8_t> buf(bufSize);
        uint32_t written = 0;
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(secs);
        while (std::chrono::steady_clock::now() < deadline) {
            cam.captureFrame(buf.data(), bufSize, written);
            if (written > 0) counter++;
        }
    };

    std::thread t0(worker, std::ref(cam0), std::ref(info0.videoFormats[0]),
                   std::ref(frames0), 3);
    std::thread t1(worker, std::ref(cam1), std::ref(info1.videoFormats[0]),
                   std::ref(frames1), 3);
    t0.join();
    t1.join();

    cam0.stop(); cam0.close();
    cam1.stop(); cam1.close();

    int min0 = minFrames(info0.videoFormats[0], 3);
    int min1 = minFrames(info1.videoFormats[0], 3);
    bool ok = frames0.load() >= min0 && frames1.load() >= min1;
    printf("  %s: %d frames (>= %d): %s\n",
           info0.address.c_str(), frames0.load(), min0, frames0.load() >= min0 ? "OK" : "FAIL");
    printf("  %s: %d frames (>= %d): %s\n",
           info1.address.c_str(), frames1.load(), min1, frames1.load() >= min1 ? "OK" : "FAIL");
    return ok;
}

// Test 10: stop one camera mid-session and verify the other is unaffected.
// cam0 is stopped and closed while cam1 continues capturing.
static bool test_dual_independent_stop(const cameraInfo& info0, const cameraInfo& info1) {
    std::cout << "\n--- Test 10: Independent Stop (cam0 stops; cam1 keeps running) ---\n";
    if (info0.videoFormats.empty() || info1.videoFormats.empty()) {
        std::cerr << "  Missing formats\n";
        return false;
    }

    Camera_USB cam0(info0), cam1(info1);

    cam0.open(); cam0.setCameraVideoFormat(0); cam0.start();
    if (!checkRunning(cam0, "Test 10 cam0")) { cam0.close(); return false; }

    cam1.open(); cam1.setCameraVideoFormat(0); cam1.start();
    if (!checkRunning(cam1, "Test 10 cam1")) {
        cam0.stop(); cam0.close(); cam1.close(); return false;
    }

    const auto& fmt0 = info0.videoFormats[0];
    const auto& fmt1 = info1.videoFormats[0];

    // Baseline: confirm both deliver frames before the stop.
    int pre0 = captureFor(cam0, fmt0, 2);
    int pre1 = captureFor(cam1, fmt1, 2);
    printf("  Pre-stop  — %s: %d frames, %s: %d frames\n",
           info0.address.c_str(), pre0, info1.address.c_str(), pre1);

    // Tear down cam0; cam1 must be completely unaffected.
    cam0.stop();
    cam0.close();
    std::cout << "  cam0 stopped and closed.\n";

    cameraStatus st;
    cam1.getCameraStatus(st);
    if (st.status != CAMERA_STATUS::RUNNING) {
        std::cerr << "  cam1 no longer RUNNING after cam0 teardown (status="
                  << static_cast<int>(st.status) << ")\n";
        cam1.stop(); cam1.close();
        return false;
    }

    int post1 = captureFor(cam1, fmt1, 2);
    printf("  Post-stop — %s: %d frames\n", info1.address.c_str(), post1);
    cam1.stop();
    cam1.close();

    int expected0 = minFrames(fmt0, 2);
    int expected1 = minFrames(fmt1, 2);
    bool ok = pre0 >= expected0 && pre1 >= expected1 && post1 >= expected1;
    printf("  Isolation check (cam1 unaffected): %s\n", ok ? "OK" : "FAIL");
    return ok;
}

// ─── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    gst_init(&argc, &argv);
    fs::create_directories("./archive");

    std::vector<cameraInfo> cameras;
    if (getCameraList(cameras) != ERROR_CODE::NONE || cameras.empty()) {
        std::cerr << "No cameras found.\n";
        return 1;
    }

    std::vector<const cameraInfo*> usb;
    for (const auto& c : cameras) {
        if (c.type == CAMERA_TYPE::USB) usb.push_back(&c);
    }

    if (usb.empty()) {
        std::cerr << "No USB cameras found among " << cameras.size() << " device(s).\n";
        return 1;
    }

    printf("=== USB Camera Test Suite ===\n");
    printf("Found %zu USB camera(s):\n", usb.size());
    for (const auto* c : usb) {
        printf("  %s  (deviceId=%u)  %zu format(s)\n",
               c->address.c_str(), c->deviceId, c->videoFormats.size());
        printFormats(*c);
    }

    const cameraInfo& c0 = *usb[0];

    bool t1  = test_open_close(c0);
    bool t2  = test_frame_capture(c0);
    bool t3  = test_attribute_before_start(c0);
    bool t4  = test_attribute_while_running(c0);
    bool t5  = test_attribute_invalid(c0);
    bool t6  = test_stop_restart(c0);
    bool t7  = test_format_cycling(c0);

    bool t8 = false, t9 = false, t10 = false;
    if (usb.size() >= 2) {
        const cameraInfo& c1 = *usb[1];
        t8  = test_dual_simultaneous(c0, c1);
        t9  = test_dual_concurrent_threads(c0, c1);
        t10 = test_dual_independent_stop(c0, c1);
    } else {
        std::cout << "\n--- Tests 8-10 SKIPPED (need >= 2 USB cameras, found 1) ---\n";
    }

    printf("\n=== Results ===\n");
    auto r = [](bool ok) { return ok ? "PASS" : "FAIL"; };
    printf("Test  1  Open/close lifecycle:             %s\n", r(t1));
    printf("Test  2  Frame capture (3 s):              %s\n", r(t2));
    printf("Test  3  Attribute queued before start:    %s\n", r(t3));
    printf("Test  4  Attribute set while running:      %s\n", r(t4));
    printf("Test  5  Invalid attribute name:           %s\n", r(t5));
    printf("Test  6  Stop/restart cycle:               %s\n", r(t6));
    printf("Test  7  Format cycling:                   %s\n", r(t7));
    if (usb.size() >= 2) {
        printf("Test  8  Dual simultaneous (alternating):  %s\n", r(t8));
        printf("Test  9  Dual concurrent (threads):        %s\n", r(t9));
        printf("Test 10  Dual independent stop:            %s\n", r(t10));
    }

    bool passed = t1 && t2 && t3 && t4 && t5 && t6 && t7;
    if (usb.size() >= 2) passed = passed && t8 && t9 && t10;
    return passed ? 0 : 1;
}
