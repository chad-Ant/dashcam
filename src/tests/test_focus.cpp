// focus_test — UvcFocusControl (libcamera_focus: the fixed-focus hold v0.4 uses
// with <FocusMode>fixed</FocusMode>) against a simulated UVC camera, no hardware.
//
// The test binary defines ioctl(): the library's calls resolve to it at link
// time.  It answers the focus controls (VIDIOC_QUERYCTRL / G_CTRL / S_CTRL) for
// one stand-in file, which the library opens as the "device", and can make any
// read or write fail.  Every other descriptor goes to the real ioctl.
//
// Checks that the camera's focus is handed back exactly as found, and that
// nothing is taken over when the state to hand back cannot be read (a failed
// read of the autofocus state used to "restore" autofocus on for a camera found
// in manual; a failed read of the lens position left it unrestored).  A camera
// found in autofocus gets its stored manual position back too (it was left at
// the held one), and a half-done hold that cannot be undone is reported (it
// was ignored: autofocus stayed off while the caller logged "left to the
// camera") and remembered: the object v0.4 reuses at its next restart hands
// that autofocus back (it took the state it had left for the state found).
//
// Usage: focus_test

#include "libcamera_focus.h"

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using dashcam::camera::UvcFocusControl;
using dashcam::log::LogLevel;

static int g_fails = 0;
static void check(bool ok, const std::string& what) {
    std::printf("  %s  %s\n", ok ? "ok  " : "FAIL", what.c_str());
    if (!ok) ++g_fails;
}

// ─── the simulated camera ────────────────────────────────────────────────────

struct FakeCamera {
    dev_t dev = 0;
    ino_t ino = 0;
    bool  hasFocus = true;             ///< false: a fixed-focus lens
    int   autoFocus = 1;               ///< V4L2_CID_FOCUS_AUTO
    int   absolute  = 0;               ///< V4L2_CID_FOCUS_ABSOLUTE
    int   absMin = 0, absMax = 1023, absStep = 1;
    bool  failGetAuto = false, failGetAbs = false;
    bool  failSetAuto = false, failSetAbs = false;
    bool  failSetAutoOn = false;       ///< refuse only turning autofocus on
    bool  busyWhileAuto = false;       ///< refuse a lens write under autofocus (EBUSY)
    int   writes = 0;                  ///< S_CTRL calls that changed something
};
static FakeCamera g_cam;

static bool isFake(int fd) {
    struct stat st;
    return g_cam.ino != 0 && ::fstat(fd, &st) == 0 && st.st_ino == g_cam.ino && st.st_dev == g_cam.dev;
}

static int fail(int err) {
    errno = err;
    return -1;
}

extern "C" int ioctl(int fd, unsigned long request, ...) __THROW {
    va_list ap;
    va_start(ap, request);
    void* arg = va_arg(ap, void*);
    va_end(ap);
    if (!isFake(fd)) return static_cast<int>(::syscall(SYS_ioctl, fd, request, arg));

    if (request == VIDIOC_QUERYCTRL) {
        auto* q = static_cast<struct v4l2_queryctrl*>(arg);
        if (!g_cam.hasFocus) return fail(EINVAL);
        if (q->id == V4L2_CID_FOCUS_AUTO) {
            q->type = V4L2_CTRL_TYPE_BOOLEAN;
            q->minimum = 0; q->maximum = 1; q->step = 1; q->default_value = 1;
            q->flags = 0;
            return 0;
        }
        if (q->id == V4L2_CID_FOCUS_ABSOLUTE) {
            q->type = V4L2_CTRL_TYPE_INTEGER;
            q->minimum = g_cam.absMin; q->maximum = g_cam.absMax; q->step = g_cam.absStep;
            q->default_value = g_cam.absMin;
            q->flags = 0;
            return 0;
        }
        return fail(EINVAL);
    }
    if (request == VIDIOC_G_CTRL) {
        auto* c = static_cast<struct v4l2_control*>(arg);
        if (c->id == V4L2_CID_FOCUS_AUTO) {
            if (g_cam.failGetAuto) return fail(EIO);
            c->value = g_cam.autoFocus;
            return 0;
        }
        if (c->id == V4L2_CID_FOCUS_ABSOLUTE) {
            if (g_cam.failGetAbs) return fail(EIO);
            c->value = g_cam.absolute;
            return 0;
        }
        return fail(EINVAL);
    }
    if (request == VIDIOC_S_CTRL) {
        auto* c = static_cast<struct v4l2_control*>(arg);
        if (c->id == V4L2_CID_FOCUS_AUTO) {
            if (g_cam.failSetAuto || (g_cam.failSetAutoOn && c->value != 0)) return fail(EIO);
            if (c->value != g_cam.autoFocus) ++g_cam.writes;
            g_cam.autoFocus = c->value;
            return 0;
        }
        if (c->id == V4L2_CID_FOCUS_ABSOLUTE) {
            if (g_cam.failSetAbs || (g_cam.busyWhileAuto && g_cam.autoFocus != 0)) return fail(EBUSY);
            if (c->value != g_cam.absolute) ++g_cam.writes;
            g_cam.absolute = c->value;
            return 0;
        }
        return fail(EINVAL);
    }
    return fail(ENOTTY);
}

// ─── fixtures ────────────────────────────────────────────────────────────────

static std::string g_device;   // the stand-in file

/// A fresh camera found with autofocus @p autoFocus and the lens at @p absolute.
static void reset(int autoFocus, int absolute) {
    struct stat st;
    ::stat(g_device.c_str(), &st);
    g_cam = FakeCamera{};
    g_cam.dev = st.st_dev;
    g_cam.ino = st.st_ino;
    g_cam.autoFocus = autoFocus;
    g_cam.absolute = absolute;
}

struct Log {
    std::vector<std::string> lines;
    dashcam::log::LogCallback callback() {
        return [this](LogLevel lvl, const std::string& m) {
            lines.push_back(std::string(lvl == LogLevel::WARN ? "W " : lvl == LogLevel::ERROR ? "E " : "I ") + m);
        };
    }
    bool has(const std::string& needle) const {
        for (const auto& l : lines)
            if (l.find(needle) != std::string::npos) return true;
        return false;
    }
};

static std::string state() {
    return "autofocus " + std::to_string(g_cam.autoFocus) + ", lens " + std::to_string(g_cam.absolute);
}

// ─── tests ───────────────────────────────────────────────────────────────────

static void testHandBack() {
    std::printf("\n--- hold and hand back ---\n");
    for (const bool foundManual : {false, true}) {
        const std::string found = foundManual ? "found in manual at 300" : "found in autofocus";
        reset(foundManual ? 0 : 1, 300);
        Log log;
        UvcFocusControl f;
        std::string why;
        const bool ok = f.open(g_device, 700, log.callback(), why);
        check(ok && f.isOpen() && g_cam.autoFocus == 0 && g_cam.absolute == 700,
              found + ": held at 700 with autofocus off (" + state() + ")");
        f.close();
        check(!f.isOpen() && g_cam.autoFocus == (foundManual ? 0 : 1) && g_cam.absolute == 300,
              found + ": handed back as found, lens position included (" + state() + ")");
        check(log.has("I focus: handed back"), found + ": logged");
    }
}

static void testUnreadableState() {
    std::printf("\n--- the state to hand back cannot be read ---\n");
    {
        reset(0, 300);   // manual, as someone set it by hand
        g_cam.failGetAuto = true;
        UvcFocusControl f;
        std::string why;
        const bool ok = f.open(g_device, 700, {}, why);
        check(!ok && !f.isOpen() && g_cam.writes == 0,
              "autofocus state unreadable: refused, nothing written (" + state() + ")");
        check(why.find("cannot read the autofocus state") != std::string::npos, "... and says why: " + why);
        f.close();
        check(g_cam.autoFocus == 0 && g_cam.absolute == 300,
              "... and autofocus is not turned on at close (it was, via the default)");
    }
    {
        reset(0, 300);
        g_cam.failGetAbs = true;
        UvcFocusControl f;
        std::string why;
        const bool ok = f.open(g_device, 700, {}, why);
        check(!ok && g_cam.writes == 0 && g_cam.absolute == 300,
              "autofocus off, lens position unreadable: refused, lens not moved (" + state() + ")");
        check(why.find("lens position cannot be read") != std::string::npos, "... and says why: " + why);
    }
    {
        reset(1, 300);
        g_cam.failGetAbs = true;   // with autofocus on, the position is the camera's own
        UvcFocusControl f;
        std::string why;
        const bool ok = f.open(g_device, 700, {}, why);
        check(ok && g_cam.autoFocus == 0 && g_cam.absolute == 700,
              "autofocus on, lens position unreadable: held (the stored position is not the focus)");
        f.close();
        check(g_cam.autoFocus == 1 && g_cam.absolute == 700,
              "... and autofocus is back on at close, no unread position written (" + state() + ")");
    }
}

// On the UGREEN, focus_absolute under autofocus is the stored manual position:
// it does not follow the autofocus, and a later switch to manual moves the lens
// there.  It used to be left at the held position.
static void testStoredPosition() {
    std::printf("\n--- found in autofocus: the stored manual position ---\n");
    {
        reset(1, 449);
        g_cam.busyWhileAuto = true;   // the lens is only writable with autofocus off
        Log log;
        UvcFocusControl f;
        std::string why;
        check(f.open(g_device, 300, log.callback(), why) && g_cam.autoFocus == 0 && g_cam.absolute == 300,
              "found in autofocus, stored position 449: held at 300 (" + state() + ")");
        f.close();
        check(g_cam.autoFocus == 1 && g_cam.absolute == 449,
              "handed back: the stored position 449, written before autofocus went on (" + state() + ")");
        check(log.has("I focus: handed back to the camera on " + g_device +
                      " (autofocus on, stored lens position 449)") &&
                  !log.has("W focus:"),
              "... logged with the position, no WARN");
    }
    {
        reset(1, 449);
        Log log;
        UvcFocusControl f;
        std::string why;
        f.open(g_device, 300, log.callback(), why);
        g_cam.failSetAbs = true;   // the stored position refused, the autofocus write not
        f.close();
        check(g_cam.autoFocus == 1,
              "stored position refused: autofocus is still turned back on (" + state() + ")");
        check(log.has("W focus: could not hand the focus back") && log.has("lens position") &&
                  !log.has("autofocus state"),
              "... and the WARN names the lens position only");
    }
}

// The lens write refused after autofocus went off: open() turns autofocus back
// on.  When that is refused too, the camera is left in manual, and open() must
// say so (it ignored the result, and v0.4 logged "left to the camera").
static void testFailedRollback() {
    std::printf("\n--- a half-done hold that cannot be undone ---\n");
    {
        reset(1, 300);
        g_cam.failSetAbs    = true;   // the lens write refused...
        g_cam.failSetAutoOn = true;   // ...and turning autofocus back on as well
        UvcFocusControl f;
        std::string why;
        const bool ok = f.open(g_device, 700, {}, why);
        check(!ok && !f.isOpen() && g_cam.autoFocus == 0,
              "lens write and rollback refused: refused, autofocus left off (" + state() + ")");
        check(f.leftChanged(), "... and reported: leftChanged()");
        check(why.find("cannot set focus to 700") != std::string::npos &&
                  why.find("autofocus could not be turned back on") != std::string::npos &&
                  why.find("left off") != std::string::npos,
              "... and says so: " + why);
        g_cam.failSetAbs    = false;
        g_cam.failSetAutoOn = false;
        check(f.open(g_device, 700, {}, why) && !f.leftChanged(), "... cleared by the next open()");
    }
    {
        reset(1, 300);
        g_cam.failSetAbs = true;   // the rollback itself works
        UvcFocusControl f;
        std::string why;
        check(!f.open(g_device, 700, {}, why) && !f.leftChanged() && g_cam.autoFocus == 1 &&
                  why.find("turned back on") == std::string::npos,
              "lens write refused, rollback done: not reported as changed (" + state() + ")");
    }
    {
        reset(0, 300);   // found in manual: the refused lens write changed nothing
        g_cam.failSetAbs    = true;
        g_cam.failSetAutoOn = true;
        UvcFocusControl f;
        std::string why;
        check(!f.open(g_device, 700, {}, why) && !f.leftChanged() && g_cam.autoFocus == 0 &&
                  g_cam.absolute == 300,
              "found in manual, lens write refused: nothing to undo, not reported (" + state() + ")");
    }
}

// v0.4 reuses the object at its next restart.  The autofocus it left off is
// what that open() reads, and it was taken for the state found: "left to the
// camera" was logged, or a later hold handed the camera back in manual.
static void testOwedAutofocus() {
    std::printf("\n--- autofocus left off, the object reused ---\n");
    auto leaveOff = [](UvcFocusControl& f, Log& log) {   // found in autofocus, stored position 449
        reset(1, 449);
        g_cam.failSetAbs    = true;
        g_cam.failSetAutoOn = true;
        std::string why;
        return !f.open(g_device, 300, log.callback(), why) && f.leftChanged() && g_cam.autoFocus == 0;
    };
    {
        Log log;
        UvcFocusControl f;
        check(leaveOff(f, log), "the first open() left autofocus off (" + state() + ")");
        std::string why;
        check(!f.open(g_device, 300, {}, why) && f.leftChanged() && g_cam.autoFocus == 0,
              "... both writes still refused at the next open(): still reported (" + state() + ")");
        g_cam.failSetAutoOn = false;   // the lens still refused, autofocus writable again
        check(!f.open(g_device, 300, {}, why) && !f.leftChanged() && g_cam.autoFocus == 1 &&
                  g_cam.absolute == 449 && why.find("left off") == std::string::npos,
              "... then its rollback turns autofocus back on, as found before (" + state() + ")");
    }
    {
        Log log;
        UvcFocusControl f;
        leaveOff(f, log);
        g_cam.failSetAbs    = false;   // the faults clear
        g_cam.failSetAutoOn = false;
        std::string why;
        check(f.open(g_device, 300, log.callback(), why) && !f.leftChanged() && g_cam.absolute == 300,
              "faults cleared at the next open(): held at 300 (" + state() + ")");
        f.close();
        check(g_cam.autoFocus == 1 && g_cam.absolute == 449 &&
                  log.has("I focus: handed back to the camera on " + g_device +
                          " (autofocus on, stored lens position 449)"),
              "... and handed back as found before the first open() (" + state() + ")");
    }
    {
        Log log;
        UvcFocusControl f;
        leaveOff(f, log);
        g_cam.autoFocus   = 1;      // power-cycled: autofocus on again, its own default
        g_cam.failSetAuto = true;   // (this open() then fails before any write)
        std::string why;
        check(!f.open(g_device, 300, {}, why) && !f.leftChanged(),
              "found on again at the next open() (a power cycle): nothing owed, not reported");
        g_cam.failSetAuto = false;
        leaveOff(f, log);
        check(!f.open("/nonexistent/video9", 300, {}, why) && !f.leftChanged(),
              "another device opened: not reported for it");
    }
    {
        reset(1, 449);
        Log log;
        UvcFocusControl f;
        std::string why;
        f.open(g_device, 300, log.callback(), why);
        g_cam.failSetAutoOn = true;   // the camera stalls at the hand-back
        f.close();
        check(f.leftChanged() && g_cam.autoFocus == 0 && log.has("W focus: could not hand the focus back"),
              "autofocus-on refused at close(): WARN, and reported (" + state() + ")");
        g_cam.failSetAutoOn = false;
        check(f.open(g_device, 300, {}, why) && !f.leftChanged(), "... the next open() holds");
        f.close();
        check(g_cam.autoFocus == 1 && g_cam.absolute == 449,
              "... and its close() turns autofocus back on, as found at first (" + state() + ")");
    }
}

static void testRefusedWrites() {
    std::printf("\n--- refused writes ---\n");
    for (const bool foundManual : {false, true}) {
        const std::string found = foundManual ? "found in manual" : "found in autofocus";
        reset(foundManual ? 0 : 1, 300);
        g_cam.failSetAbs = true;
        UvcFocusControl f;
        std::string why;
        const bool ok = f.open(g_device, 700, {}, why);
        check(!ok && g_cam.autoFocus == (foundManual ? 0 : 1) && g_cam.absolute == 300,
              found + ": focus write refused -> left as found (" + state() + ")");
    }
    {
        reset(1, 300);
        g_cam.failSetAuto = true;
        UvcFocusControl f;
        std::string why;
        check(!f.open(g_device, 700, {}, why) && g_cam.writes == 0 &&
                  why.find("cannot turn autofocus off") != std::string::npos,
              "autofocus cannot be turned off: refused, nothing written");
    }
    {
        reset(0, 300);
        Log log;
        UvcFocusControl f;
        std::string why;
        f.open(g_device, 700, log.callback(), why);
        g_cam.failSetAbs = true;   // unplugged, say
        g_cam.failSetAuto = true;
        f.close();
        check(!f.isOpen() && log.has("W focus: could not hand the focus back") &&
                  log.has("lens position") && log.has("autofocus state"),
              "a hand-back that fails is logged as WARN, naming both controls");
    }
}

static void testLimits() {
    std::printf("\n--- range, grid, no focus control ---\n");
    reset(1, 0);
    g_cam.absMax  = 1020;
    g_cam.absStep = 10;
    Log log;
    UvcFocusControl f;
    std::string why;
    check(f.open(g_device, 1500, log.callback(), why) && g_cam.absolute == 1020 && f.position() == 1020,
          "a position past the range is clamped (1500 -> 1020)");
    check(log.has("W focus: FocusAbsolute 1500 is not a focus position"), "... with a WARN");
    f.close();
    reset(1, 0);
    g_cam.absStep = 10;
    check(f.open(g_device, 457, {}, why) && g_cam.absolute == 450, "off the grid: snapped down (457 -> 450)");
    f.close();

    reset(1, 0);
    g_cam.hasFocus = false;
    bool ok = f.open(g_device, 500, {}, why);
    check(!ok && why.find("fixed-focus") != std::string::npos,
          "a camera without focus controls is refused: " + why);
    ok = f.open("/nonexistent/video9", 500, {}, why);
    check(!ok && why.find("cannot open") != std::string::npos,
          "a device that cannot be opened is refused: " + why);
}

int main() {
    std::printf("focus_test — UvcFocusControl against a simulated UVC camera\n");
    char path[] = "/tmp/focus_test.XXXXXX";
    const int fd = ::mkstemp(path);
    if (fd < 0) {
        std::perror("mkstemp");
        return 1;
    }
    ::close(fd);
    g_device = path;

    testHandBack();
    testUnreadableState();
    testStoredPosition();
    testFailedRollback();
    testOwedAutofocus();
    testRefusedWrites();
    testLimits();

    ::unlink(path);
    std::printf("\nRESULT: %s (%d failure%s)\n", g_fails ? "FAIL" : "PASS", g_fails, g_fails == 1 ? "" : "s");
    return g_fails ? 1 : 0;
}
