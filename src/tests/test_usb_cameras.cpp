// USB camera function test — Jetson Orin Nano
//
// Single-camera tests (Tests 1-7, 4b):  always run against the first USB camera found.
// Dual-camera tests   (Tests 8-10): run only when 2+ USB cameras are present.
//
// A UVC camera keeps its control values until it loses power, and this suite
// runs on the dashcam's own camera: main() reads the controls it writes
// (brightness, gain, backlight_compensation) before the first test, prints the
// v4l2-ctl command that puts them back, and writes them back after the last
// test, with the camera closed.  A value that does not read back as found
// fails the run.
//
// Ctrl-C, SIGTERM or SIGHUP (the SSH session dropped) stop the run after the
// test in progress (its capture loops end early), and the controls are still
// put back, once; the run then ends by that signal (exit status 128 + signal).
// A repeat within a second is the same stop delivered twice (timeout(1), an SSH
// drop) and is absorbed.  A second signal a second or more later, a crash or
// SIGKILL ends it at once: run the printed command.
//
// Usage: ./usb_test               the suite, on the first USB camera found
//        ./usb_test --self-test   the stop-and-restore path on a simulated
//                                 camera (no camera, no GStreamer)
// Requires: UVC cameras at /dev/videoN, GStreamer 1.0 with v4l2src + videoconvert

#include "libcamera_usb.h"
#include "libcamera_v4l2.h"
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <ctime>
#include <exception>
#include <filesystem>
#include <functional>
#include <gst/gst.h>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace dashcam::camera;

namespace fs = std::filesystem;

// Attribute dictionary shared by the attribute tests (3-5); loaded once in
// main() from config/camera_attributes.xml.  Camera_GST resolves attribute
// names against this dictionary, so tests must install it via
// setAttributeDictionary() before exercising setCameraAttribute().
static AttributeDictionary g_dict;

// ─── stop signals ─────────────────────────────────────────────────────────────

// An atomic, not a volatile sig_atomic_t: the handler can run on any thread (a
// GStreamer one), and Test 9's capture threads read it too.  Lock-free, so it
// is safe in a signal handler.
static std::atomic<int> g_stopSignal{0};   ///< the first stop signal, or 0
static_assert(std::atomic<int>::is_always_lock_free, "the stop flag is written in a signal handler");

static bool stopRequested() { return g_stopSignal.load() != 0; }

static std::atomic<long long> g_stopAtNs{0};   ///< CLOCK_MONOTONIC of the first stop signal, or 0
static_assert(std::atomic<long long>::is_always_lock_free, "the stop time is written in a signal handler");

/// Records the first stop signal.  The same stop often arrives twice, within
/// microseconds to milliseconds: timeout(1) signals the child and then its
/// process group, and on an SSH drop the shell and then the kernel (at the
/// shell's exit) each send SIGHUP.  A repeat within 1 s is that duplicate and
/// is absorbed; a later one is a deliberate second signal and kills at once
/// (its default action, delivered when the handler returns).
/// Async-signal-safe (clock_gettime, sigaction, raise, write, lock-free
/// atomics); errno is kept for the code it interrupted.
static void onStopSignal(int sig) {
    const int savedErrno = errno;
    struct timespec ts {};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    const long long now = ts.tv_sec * 1000000000LL + ts.tv_nsec;
    long long unset = 0;
    g_stopAtNs.compare_exchange_strong(unset, now);   // the first one's time, before its flag
    int none = 0;
    if (g_stopSignal.compare_exchange_strong(none, sig)) {   // the first one wins
        static const char msg[] = "\nusb_test: stopping after this test; the camera's controls are put back "
                                  "before the exit (signal again, a second later, to kill at once)\n";
        const ssize_t n = ::write(STDERR_FILENO, msg, sizeof(msg) - 1);
        (void)n;
    } else if (now - g_stopAtNs.load() >= 1000000000LL) {
        struct sigaction dfl {};
        dfl.sa_handler = SIG_DFL;
        ::sigaction(sig, &dfl, nullptr);
        ::raise(sig);   // blocked in here: pending until the handler returns
    }
    errno = savedErrno;
}

/// SIGINT, SIGTERM and SIGHUP stop the run (onStopSignal), except one already
/// ignored on entry: nohup, or a background job of a non-interactive shell.
/// SIGPIPE is ignored: a reader that dies (`usb_test | tee log`, the SSH
/// session dropped) fails the writes (EPIPE) instead of killing the run before
/// the restore.
static void installStopHandlers() {
    std::signal(SIGPIPE, SIG_IGN);
    g_stopSignal = 0;
    g_stopAtNs   = 0;
    struct sigaction sa {};
    sa.sa_handler = onStopSignal;
    sigemptyset(&sa.sa_mask);
    for (const int s : {SIGINT, SIGTERM, SIGHUP}) sigaddset(&sa.sa_mask, s);   // never nested
    sa.sa_flags = SA_RESTART;
    for (const int s : {SIGINT, SIGTERM, SIGHUP}) {
        struct sigaction now {};
        if (::sigaction(s, nullptr, &now) == 0 && now.sa_handler == SIG_IGN) continue;
        ::sigaction(s, &sa, nullptr);
    }
}

static const char* signalName(int sig) {
    return sig == SIGINT ? "SIGINT" : sig == SIGTERM ? "SIGTERM" : sig == SIGHUP ? "SIGHUP" : "a signal";
}

/// true while @p deadline is ahead and no stop signal has arrived: the capture
/// loops, the suite's long waits, end early on Ctrl-C.
static bool keepGoing(std::chrono::steady_clock::time_point deadline) {
    return !stopRequested() && std::chrono::steady_clock::now() < deadline;
}

// ─── helpers ──────────────────────────────────────────────────────────────────

/// A value for control @p cid inside its range and on its step grid, and not
/// its current value @p orig (read here).  false when the camera lacks the
/// control, or it is read-only or inactive right now.
static bool pickControlValue(int fd, uint32_t cid, int& want, int& orig,
                             struct v4l2_queryctrl& q) {
    if (!v4l2::queryControl(fd, cid, q) || !v4l2::getControl(fd, cid, orig)) return false;
    if (q.flags & (V4L2_CTRL_FLAG_INACTIVE | V4L2_CTRL_FLAG_READ_ONLY)) return false;
    const int step = q.step > 0 ? q.step : 1;
    want = q.minimum + ((q.maximum - q.minimum) / 2 / step) * step;   // mid-range, on the grid
    if (want == orig) want = (orig - step >= q.minimum) ? orig - step : orig + step;
    return true;
}

/// A camera's controls, read and written: the device itself (DeviceControls),
/// or a simulated camera (--self-test).
struct ControlIo {
    virtual ~ControlIo() = default;
    virtual bool opened() const = 0;
    virtual bool has(uint32_t cid) = 0;
    virtual bool get(uint32_t cid, int& value) = 0;
    virtual bool set(uint32_t cid, int value) = 0;
};

class DeviceControls : public ControlIo {
public:
    explicit DeviceControls(const std::string& device) : fd_(v4l2::openNode(device, O_RDWR)) {}
    bool opened() const override { return fd_.valid(); }
    bool has(uint32_t cid) override {
        struct v4l2_queryctrl q;
        return v4l2::queryControl(fd_.get(), cid, q);
    }
    bool get(uint32_t cid, int& value) override { return v4l2::getControl(fd_.get(), cid, value); }
    bool set(uint32_t cid, int value) override { return v4l2::setControl(fd_.get(), cid, value); }

private:
    v4l2::Fd fd_;
};

/// The controls this suite writes, as found before the first test.
struct ControlSnapshot {
    struct Entry { const char* name; uint32_t cid; int value; };
    std::vector<Entry> entries;

    void take(ControlIo& io) {
        const Entry all[] = {{"brightness", V4L2_CID_BRIGHTNESS, 0},
                             {"gain", V4L2_CID_GAIN, 0},
                             {"backlight_compensation", V4L2_CID_BACKLIGHT_COMPENSATION, 0}};
        for (Entry e : all)
            if (io.opened() && io.has(e.cid) && io.get(e.cid, e.value)) entries.push_back(e);
    }

    /// The shell command that writes the found values back, for a run that
    /// dies before restore(); empty when there is nothing to put back.
    std::string restoreCommand(const std::string& device) const {
        if (entries.empty()) return "";
        std::string cmd = "v4l2-ctl -d " + device + " --set-ctrl=";
        for (size_t i = 0; i < entries.size(); ++i)
            cmd += std::string(i ? "," : "") + entries[i].name + "=" + std::to_string(entries[i].value);
        return cmd;
    }

    /// Writes every value back, then reads each: true when all read as found.
    bool restore(ControlIo& io, const std::string& device) const {
        if (!io.opened()) {
            printf("  cannot open %s to restore the controls\n", device.c_str());
            return entries.empty();
        }
        bool ok = true;
        for (const Entry& e : entries) {
            io.set(e.cid, e.value);
            int now = -1;
            const bool same = io.get(e.cid, now) && now == e.value;
            printf("  %-24s %d%s\n", e.name, now, same ? " (as found)" :
                   (" (FAIL: found " + std::to_string(e.value) + ")").c_str());
            ok = ok && same;
        }
        return ok;
    }
};

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
    while (keepGoing(deadline)) {
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
    cameraStatus st;

    // start() before open() must be refused with CAMERA_NOT_OPEN and no
    // status transition.
    cam.start();
    cam.getCameraStatus(st);
    if (st.status != CAMERA_STATUS::CLOSED ||
        st.currentError != ERROR_CODE::CAMERA_NOT_OPEN) {
        std::cerr << "  start() before open(): expected CLOSED + CAMERA_NOT_OPEN,"
                  << " got status=" << static_cast<int>(st.status)
                  << " error=" << static_cast<int>(st.currentError) << "\n";
        return false;
    }
    std::cout << "  start() before open() -> refused (CAMERA_NOT_OPEN)\n";

    cam.open();
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

// Test 3: brightness queued before start() reaches the device.  The value is
// inside the camera's range (the driver clamps anything else and reports
// success) and is read back with VIDIOC_G_CTRL while running: until
// 2026-09-28 it went to v4l2src's brightness property, which v4l2src drops
// while the device is closed, and only the error code was checked.
static bool test_attribute_before_start(const cameraInfo& info) {
    std::cout << "\n--- Test 3: Attribute Queued Before start() ---\n";
    if (info.videoFormats.empty()) { std::cerr << "  No formats\n"; return false; }
    const v4l2::Fd fd = v4l2::openNode(info.address, O_RDWR);
    struct v4l2_queryctrl q;
    int want = 0, orig = 0;
    if (!fd.valid() || !pickControlValue(fd.get(), V4L2_CID_BRIGHTNESS, want, orig, q)) {
        printf("  brightness not settable on this camera (skipped)\n");
        return true;
    }

    Camera_USB cam(info);
    cam.setAttributeDictionary(g_dict);
    cam.setCameraAttribute("brightness", std::to_string(want));
    cam.open();
    cam.setCameraVideoFormat(0);
    cam.start();
    if (!checkRunning(cam, "Test 3")) { cam.close(); return false; }

    cameraStatus st;
    cam.getCameraStatus(st);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    int got = -1;
    v4l2::getControl(fd.get(), V4L2_CID_BRIGHTNESS, got);
    const bool ok = st.currentError == ERROR_CODE::NONE && got == want;
    printf("  brightness queued %d (range %d..%d, was %d), device reads %d, error=%d %s\n", want,
           q.minimum, q.maximum, orig, got, static_cast<int>(st.currentError), ok ? "(OK)" : "(FAIL)");

    cam.stop();
    cam.close();
    return ok;
}

// Test 4: brightness written while RUNNING reaches the device (in range, read back).
static bool test_attribute_while_running(const cameraInfo& info) {
    std::cout << "\n--- Test 4: Attribute Set While Running ---\n";
    if (info.videoFormats.empty()) { std::cerr << "  No formats\n"; return false; }
    const v4l2::Fd fd = v4l2::openNode(info.address, O_RDWR);
    struct v4l2_queryctrl q;
    int want = 0, orig = 0;
    if (!fd.valid() || !pickControlValue(fd.get(), V4L2_CID_BRIGHTNESS, want, orig, q)) {
        printf("  brightness not settable on this camera (skipped)\n");
        return true;
    }

    Camera_USB cam(info);
    cam.setAttributeDictionary(g_dict);
    cam.open();
    cam.setCameraVideoFormat(0);
    cam.start();
    if (!checkRunning(cam, "Test 4")) { cam.close(); return false; }

    cam.setCameraAttribute("brightness", std::to_string(want));
    cameraStatus st;
    cam.getCameraStatus(st);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    int got = -1;
    v4l2::getControl(fd.get(), V4L2_CID_BRIGHTNESS, got);
    const bool ok = st.currentError == ERROR_CODE::NONE && got == want;
    printf("  brightness=%d while RUNNING (range %d..%d), device reads %d, error=%d %s\n", want,
           q.minimum, q.maximum, got, static_cast<int>(st.currentError), ok ? "(OK)" : "(FAIL)");

    cam.stop();
    cam.close();
    return ok;
}

// Test 4b: several controls, set one after another while RUNNING, each reach
// the device.  All three are V4L2 controls written through v4l2src's
// extra-controls (valueType v4l2_control).  Each control the camera has, and
// that is active and writable, is set inside its range and read back with
// VIDIOC_G_CTRL, then put back through this fd.  A later write must not undo
// that: each write carries only its own control (it used to re-send them all,
// so gain went back to the test value).  uvcvideo does not flag gain inactive
// under the camera's auto-exposure; only exposure_time_absolute,
// white_balance_temperature, focus_absolute and hue follow an auto control.
static bool test_attribute_reaches_device(const cameraInfo& info) {
    std::cout << "\n--- Test 4b: Attribute Values Reach the Device ---\n";
    if (info.videoFormats.empty()) { std::cerr << "  No formats\n"; return false; }
    const v4l2::Fd fd = v4l2::openNode(info.address, O_RDWR);
    if (!fd.valid()) { std::cerr << "  cannot open " << info.address << "\n"; return false; }

    Camera_USB cam(info);
    cam.setAttributeDictionary(g_dict);
    cam.open();
    cam.setCameraVideoFormat(0);
    cam.start();
    if (!checkRunning(cam, "Test 4b")) { cam.close(); return false; }

    struct Probe { const char* alias; uint32_t cid; int orig; bool done; };
    Probe probes[] = {{"brightness",             V4L2_CID_BRIGHTNESS, 0, false},
                      {"gain",                   V4L2_CID_GAIN, 0, false},
                      {"backlight_compensation", V4L2_CID_BACKLIGHT_COMPENSATION, 0, false}};
    bool ok = true;
    int  tried = 0;
    for (Probe& p : probes) {
        struct v4l2_queryctrl q;
        int want = 0;
        if (!pickControlValue(fd.get(), p.cid, want, p.orig, q)) {
            printf("  %-24s not on this camera, or read-only / inactive now (skipped)\n", p.alias);
            continue;
        }
        cam.setCameraAttribute(p.alias, std::to_string(want));
        cameraStatus st;
        cam.getCameraStatus(st);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        int got = -1;
        v4l2::getControl(fd.get(), p.cid, got);
        const bool pass = st.currentError == ERROR_CODE::NONE && got == want;
        printf("  %-24s set %d (range %d..%d), device reads %d, error=%d %s\n", p.alias, want,
               q.minimum, q.maximum, got, static_cast<int>(st.currentError), pass ? "(OK)" : "(FAIL)");
        ok = ok && pass;
        ++tried;
        v4l2::setControl(fd.get(), p.cid, p.orig);   // leave the camera as found
        p.done = true;
    }
    if (tried == 0) printf("  none of the probed controls is settable on this camera\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    for (const Probe& p : probes) {   // no later write may have undone a put-back
        int now = -1;
        if (!p.done) continue;
        v4l2::getControl(fd.get(), p.cid, now);
        const bool kept = now == p.orig;
        printf("  %-24s put back to %d, reads %d %s\n", p.alias, p.orig, now, kept ? "(OK)" : "(FAIL)");
        ok = ok && kept;
    }

    cam.stop();
    cam.close();
    return ok;
}

// Test 5: unrecognised attribute name sets INVALID_ATTRIBUTE.
static bool test_attribute_invalid(const cameraInfo& info) {
    std::cout << "\n--- Test 5: Invalid Attribute Name ---\n";
    if (info.videoFormats.empty()) { std::cerr << "  No formats\n"; return false; }

    Camera_USB cam(info);
    cam.setAttributeDictionary(g_dict);
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
    while (keepGoing(deadline)) {
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
        while (keepGoing(deadline)) {
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

// ─── the run order ────────────────────────────────────────────────────────────

enum class Outcome { PASS, FAIL, INTERRUPTED, NOT_RUN };

static const char* outcomeName(Outcome o) {
    return o == Outcome::PASS ? "PASS" : o == Outcome::FAIL ? "FAIL"
         : o == Outcome::INTERRUPTED ? "INTERRUPTED" : "NOT RUN";
}

struct Step {
    std::string           label;   ///< its line in the results
    std::function<bool()> run;
    Outcome               outcome = Outcome::NOT_RUN;
};

/// Runs @p steps in turn, none started once a stop signal has arrived (the one
/// it cut short is INTERRUPTED, not judged; one that throws FAILs), then
/// @p restore exactly once, with every camera closed (each test closes its
/// own), and prints the results.  Returns the exit status: 0 when every step
/// and the restore passed, 1 when one failed, 128 + the signal when a stop
/// signal ended the run.
static int runSuite(std::vector<Step>& steps, const std::function<bool()>& restore) {
    for (Step& st : steps) {
        if (stopRequested()) break;
        bool ok = false;
        try {
            ok = st.run();
        } catch (const std::exception& e) {
            std::cerr << "  " << st.label << " threw: " << e.what() << "\n";
        } catch (...) {
            std::cerr << "  " << st.label << " threw\n";
        }
        st.outcome = stopRequested() ? Outcome::INTERRUPTED : ok ? Outcome::PASS : Outcome::FAIL;
    }

    std::cout << "\n--- Camera controls put back (camera closed) ---\n";
    const bool restored = restore();

    printf("\n=== Results ===\n");
    bool passed = restored;
    for (const Step& st : steps) {
        printf("%-42s %s\n", st.label.c_str(), outcomeName(st.outcome));
        passed = passed && st.outcome == Outcome::PASS;
    }
    printf("%-42s %s\n", "Controls restored as found:", restored ? "PASS" : "FAIL");
    const int sig = g_stopSignal.load();
    if (sig == 0) return passed ? 0 : 1;
    const bool skipped = !steps.empty() && steps.back().outcome == Outcome::NOT_RUN;
    printf("Stopped by %s%s\n", signalName(sig), skipped ? ": the tests after it did not run." : ".");
    return 128 + sig;
}

/// main()'s end for runSuite()'s @p status: after a stop signal the process
/// ends by that signal itself (its default action, the controls already put
/// back), so a calling shell or script sees it (bash: $? = 128 + signal, and a
/// script's own loop stops on Ctrl-C).  Otherwise returns @p status.
static int endRun(int status) {
    if (status > 128) {
        std::fflush(stdout);
        std::signal(status - 128, SIG_DFL);
        std::raise(status - 128);
    }
    return status;
}

// ─── --self-test: the stop-and-restore path on a simulated camera ─────────────

/// A simulated camera's controls, found at brightness 32, gain 0, backlight 0.
class FakeControls : public ControlIo {
public:
    std::map<uint32_t, int> values{{V4L2_CID_BRIGHTNESS, 32}, {V4L2_CID_GAIN, 0},
                                   {V4L2_CID_BACKLIGHT_COMPENSATION, 0}};
    uint32_t refuse = 0;   ///< a control whose writes fail

    bool opened() const override { return true; }
    bool has(uint32_t cid) override { return values.count(cid) != 0; }
    bool get(uint32_t cid, int& value) override {
        const auto it = values.find(cid);
        if (it == values.end()) return false;
        value = it->second;
        return true;
    }
    bool set(uint32_t cid, int value) override {
        if (cid == refuse || !has(cid)) return false;
        values[cid] = value;
        return true;
    }
};

/// The stop handlers, installed from the default dispositions: the self-test
/// does not depend on what its own caller ignores.
static void freshStopHandlers() {
    for (const int s : {SIGINT, SIGTERM, SIGHUP}) std::signal(s, SIG_DFL);
    installStopHandlers();
}

/// main()'s run in a child: one test that changes a control and calls
/// @p during, runSuite(), endRun().  The restore reports through a pipe, since
/// a signal death has no status: @p mark is 'R' when it put the controls back,
/// 0 when it never ran.  @p ws is the child's wait status.
static pid_t suiteInChild(const std::function<void()>& during, char& mark, int& ws) {
    mark = 0;
    ws   = 0;
    std::fflush(stdout);
    int fds[2] = {-1, -1};
    if (::pipe(fds) != 0) return -1;
    const pid_t pid = ::fork();
    if (pid == 0) {
        ::close(fds[0]);
        freshStopHandlers();
        FakeControls cam;
        const auto asFound = cam.values;
        ControlSnapshot found;
        found.take(cam);
        std::vector<Step> steps;
        steps.push_back({"Test  1", [&] {
                             cam.set(V4L2_CID_GAIN, 7);
                             during();
                             return true;
                         }});
        ::_exit(endRun(runSuite(steps, [&] {
            const bool ok = found.restore(cam, "/dev/sim");
            const char m  = ok && cam.values == asFound ? 'R' : 'F';
            const ssize_t n = ::write(fds[1], &m, 1);
            return ok && n == 1;
        })));
    }
    ::close(fds[1]);
    if (pid > 0) {
        if (::read(fds[0], &mark, 1) != 1) mark = 0;
        ::waitpid(pid, &ws, 0);
    }
    ::close(fds[0]);
    return pid;
}

static int selfTest() {
    int fails = 0;
    auto check = [&fails](bool ok, const std::string& what) {
        printf("  %s  %s\n", ok ? "ok  " : "FAIL", what.c_str());
        if (!ok) ++fails;
    };
    const auto asFound = FakeControls().values;
    printf("usb_test self-test (stop signals and the restore, simulated camera)\n");

    {
        FakeControls cam;
        ControlSnapshot found;
        found.take(cam);
        const std::string cmd = found.restoreCommand("/dev/video1");
        check(cmd == "v4l2-ctl -d /dev/video1 --set-ctrl=brightness=32,gain=0,backlight_compensation=0",
              "restore command: " + cmd);
        FakeControls noBacklight;
        noBacklight.values.erase(V4L2_CID_BACKLIGHT_COMPENSATION);
        ControlSnapshot fewer;
        fewer.take(noBacklight);
        check(fewer.restoreCommand("/dev/video1") ==
                  "v4l2-ctl -d /dev/video1 --set-ctrl=brightness=32,gain=0",
              "a control the camera lacks is left out of it");
        check(ControlSnapshot().restoreCommand("/dev/video1").empty(), "nothing to put back: no command");
    }

    // Three tests that each change a control; the signal arrives during the second.
    for (const int sig : {0, SIGINT, SIGTERM, SIGHUP}) {
        const std::string what = sig ? signalName(sig) : "no signal";
        printf("\n[%s]\n", what.c_str());
        FakeControls cam;
        ControlSnapshot found;
        found.take(cam);
        freshStopHandlers();
        std::vector<std::string> order;
        std::vector<Step> steps;
        steps.push_back({"Test  1", [&] { order.push_back("1"); return cam.set(V4L2_CID_BRIGHTNESS, 50); }});
        steps.push_back({"Test  2", [&] {
                             order.push_back("2");
                             cam.set(V4L2_CID_GAIN, 7);
                             if (sig) std::raise(sig);
                             return true;
                         }});
        steps.push_back({"Test  3", [&] {
                             order.push_back("3");
                             return cam.set(V4L2_CID_BACKLIGHT_COMPENSATION, 2);
                         }});
        const int status = runSuite(steps, [&] {
            order.push_back("restore");
            return found.restore(cam, "/dev/sim");
        });
        std::string seq;
        for (const auto& o : order) seq += (seq.empty() ? "" : " ") + o;
        const std::string want = sig ? "1 2 restore" : "1 2 3 restore";
        check(seq == want, what + ": ran " + seq + " (no test after the signal; the restore once, last)");
        check(cam.values == asFound, what + ": the controls are as found");
        check(status == (sig ? 128 + sig : 0), what + ": exit status " + std::to_string(status));
        if (sig) {
            check(steps[0].outcome == Outcome::PASS && steps[1].outcome == Outcome::INTERRUPTED &&
                      steps[2].outcome == Outcome::NOT_RUN,
                  what + ": test 1 PASS, test 2 INTERRUPTED, test 3 NOT RUN");
            std::raise(sig);   // the duplicate: timeout(1)'s group signal, the kernel's SIGHUP
            check(g_stopSignal.load() == sig, what + ": the same signal again at once is absorbed");
        }
    }

    {
        printf("\n[a signal ignored on entry (nohup)]\n");
        for (const int s : {SIGINT, SIGTERM}) std::signal(s, SIG_DFL);
        std::signal(SIGHUP, SIG_IGN);
        installStopHandlers();
        struct sigaction hup {}, intr {};
        ::sigaction(SIGHUP, nullptr, &hup);
        ::sigaction(SIGINT, nullptr, &intr);
        check(hup.sa_handler == SIG_IGN && intr.sa_handler == onStopSignal,
              "SIGHUP stays ignored, SIGINT is handled");
        std::raise(SIGINT);
        ::sigaction(SIGHUP, nullptr, &hup);
        check(g_stopSignal.load() == SIGINT && hup.sa_handler == SIG_IGN,
              "after a Ctrl-C, SIGHUP is still ignored");
    }

    {
        printf("\n[a signal during a capture loop]\n");
        freshStopHandlers();
        std::thread sender([] {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            ::kill(::getpid(), SIGTERM);
        });
        const auto t0 = std::chrono::steady_clock::now();
        while (keepGoing(t0 + std::chrono::seconds(5)))
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        sender.join();
        check(secs < 1.0 && g_stopSignal.load() == SIGTERM,
              "SIGTERM 0.1 s into a 5 s loop ends it after " + std::to_string(secs).substr(0, 4) + " s");
    }

    {
        printf("\n[a test that throws]\n");
        freshStopHandlers();
        FakeControls cam;
        ControlSnapshot found;
        found.take(cam);
        int ranAfter = 0, restores = 0;
        std::vector<Step> steps;
        steps.push_back({"Test  1", [&]() -> bool {
                             cam.set(V4L2_CID_GAIN, 7);
                             throw std::runtime_error("simulated");
                         }});
        steps.push_back({"Test  2", [&] { return ++ranAfter > 0; }});
        const int status = runSuite(steps, [&] {
            ++restores;
            return found.restore(cam, "/dev/sim");
        });
        check(steps[0].outcome == Outcome::FAIL && ranAfter == 1 && restores == 1 && cam.values == asFound &&
                  status == 1,
              "it FAILs, the next test still runs, the controls are put back, exit status 1");
    }

    {
        printf("\n[a control that does not go back]\n");
        freshStopHandlers();
        FakeControls cam;
        ControlSnapshot found;
        found.take(cam);
        std::vector<Step> steps;
        steps.push_back({"Test  1", [&] {
                             cam.set(V4L2_CID_GAIN, 7);
                             cam.refuse = V4L2_CID_GAIN;   // unplugged, say
                             return true;
                         }});
        const int status = runSuite(steps, [&] { return found.restore(cam, "/dev/sim"); });
        check(status == 1, "the run fails (exit status " + std::to_string(status) + ")");
    }

    {
        printf("\n[the process ends by the signal, after the restore]\n");
        char mark = 0;
        int ws = 0;
        const pid_t pid = suiteInChild([] { std::raise(SIGINT); }, mark, ws);
        check(pid > 0 && mark == 'R', "the controls were put back before the end");
        check(pid > 0 && WIFSIGNALED(ws) && WTERMSIG(ws) == SIGINT,
              "the process ended by SIGINT itself (a shell sees $? = 130), not by exit()");
    }

    {
        printf("\n[a second signal, a second later]\n");
        char mark = 0;
        int ws = 0;
        const pid_t pid = suiteInChild([] {
            std::raise(SIGTERM);
            std::this_thread::sleep_for(std::chrono::milliseconds(1100));
            std::raise(SIGTERM);
        }, mark, ws);
        check(pid > 0 && mark == 0 && WIFSIGNALED(ws) && WTERMSIG(ws) == SIGTERM,
              "it kills at once, before the restore (the printed command puts the controls back)");
    }

    {
        // timeout(1) signals the child, then its process group; an SSH drop
        // sends SIGHUP from the shell, then from the kernel at the shell's exit.
        // (Over a second after the last signal above: a first-signal time not
        // reset by installStopHandlers() would kill here.)
        printf("\n[the same stop delivered twice (timeout(1), an SSH drop)]\n");
        char mark = 0;
        int ws = 0;
        pid_t pid = suiteInChild([] { std::raise(SIGTERM); std::raise(SIGTERM); }, mark, ws);
        check(pid > 0 && mark == 'R' && WIFSIGNALED(ws) && WTERMSIG(ws) == SIGTERM,
              "SIGTERM twice at once: the controls are put back, then it ends by SIGTERM");
        pid = suiteInChild([] {
                  ::kill(::getpid(), SIGHUP);
                  std::this_thread::sleep_for(std::chrono::milliseconds(100));
                  ::kill(::getpid(), SIGHUP);
              }, mark, ws);
        check(pid > 0 && mark == 'R' && WIFSIGNALED(ws) && WTERMSIG(ws) == SIGHUP,
              "SIGHUP again 0.1 s later: the controls are put back, then it ends by SIGHUP");
    }

    {
        printf("\n[the output's reader gone (usb_test | tee log, the SSH session dropped)]\n");
        char mark = 0;
        int ws = 0;
        const pid_t pid = suiteInChild([] {
            std::raise(SIGHUP);
            int p[2];
            if (::pipe(p) == 0) {   // stdout becomes a pipe nobody reads
                ::close(p[0]);
                ::dup2(p[1], STDOUT_FILENO);
                ::close(p[1]);
            }
            printf("written to the dead pipe\n");
            std::fflush(stdout);
        }, mark, ws);
        check(pid > 0 && mark == 'R' && WIFSIGNALED(ws) && WTERMSIG(ws) == SIGHUP,
              "the write fails (no SIGPIPE death), the controls are put back, it ends by SIGHUP");
    }

    for (const int s : {SIGINT, SIGTERM, SIGHUP}) std::signal(s, SIG_DFL);
    printf("\nRESULT: %s (%d failure%s)\n", fails ? "FAIL" : "PASS", fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}

// ─── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    if (argc == 2 && std::string(argv[1]) == "--self-test") return selfTest();

    gst_init(&argc, &argv);
    fs::create_directories("./archive");

    if (!AttributeDictionary::load("config/camera_attributes.xml", g_dict)) {
        std::cerr << "WARNING: config/camera_attributes.xml not loaded; "
                     "attribute tests (3-5) will fail.\n";
    }

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
    ControlSnapshot found;
    {
        DeviceControls io(c0.address);
        found.take(io);
    }
    // Printed (and flushed, for a pipe) before anything changes: the way back
    // for a run that dies before the restore.
    const std::string restoreCmd = found.restoreCommand(c0.address);
    if (restoreCmd.empty())
        printf("None of the controls this suite writes could be read on %s.\n", c0.address.c_str());
    else
        printf("Controls this suite writes are put back after the last test, or after Ctrl-C/SIGTERM/SIGHUP.\n"
               "If the run dies before that (a second signal a second later, a crash), put them back with:\n"
               "  %s\n",
               restoreCmd.c_str());
    std::fflush(stdout);
    installStopHandlers();

    std::vector<Step> steps = {
        {"Test  1  Open/close lifecycle:",          [&] { return test_open_close(c0); }},
        {"Test  2  Frame capture (3 s):",           [&] { return test_frame_capture(c0); }},
        {"Test  3  Attribute queued before start:", [&] { return test_attribute_before_start(c0); }},
        {"Test  4  Attribute set while running:",   [&] { return test_attribute_while_running(c0); }},
        {"Test 4b  Attribute values reach device:", [&] { return test_attribute_reaches_device(c0); }},
        {"Test  5  Invalid attribute name:",        [&] { return test_attribute_invalid(c0); }},
        {"Test  6  Stop/restart cycle:",            [&] { return test_stop_restart(c0); }},
        {"Test  7  Format cycling:",                [&] { return test_format_cycling(c0); }},
    };
    if (usb.size() >= 2) {
        const cameraInfo* c1 = usb[1];
        steps.push_back({"Test  8  Dual simultaneous (alternating):",
                         [&c0, c1] { return test_dual_simultaneous(c0, *c1); }});
        steps.push_back({"Test  9  Dual concurrent (threads):",
                         [&c0, c1] { return test_dual_concurrent_threads(c0, *c1); }});
        steps.push_back({"Test 10  Dual independent stop:",
                         [&c0, c1] { return test_dual_independent_stop(c0, *c1); }});
    } else {
        std::cout << "\n--- Tests 8-10 SKIPPED (need >= 2 USB cameras, found 1) ---\n";
    }

    return endRun(runSuite(steps, [&] {
        DeviceControls io(c0.address);
        return found.restore(io, c0.address);
    }));
}
