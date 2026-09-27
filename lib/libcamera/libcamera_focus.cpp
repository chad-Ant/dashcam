#include "libcamera_focus.h"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>

namespace dashcam::camera {

using dashcam::log::LogLevel;

int snapFocusPosition(int wanted, int min, int max, int step) {
    if (max < min) return min;
    const long long lo = min, hi = max;
    const long long s  = step < 1 ? 1 : step;
    const long long v  = std::max(lo, std::min(hi, static_cast<long long>(wanted)));
    return static_cast<int>(lo + ((v - lo) / s) * s);
}

UvcFocusControl::~UvcFocusControl() { close(); }

bool UvcFocusControl::setCtrl(unsigned id, int value) {
    struct v4l2_control c;
    std::memset(&c, 0, sizeof(c));
    c.id    = id;
    c.value = value;
    return ::ioctl(fd_, VIDIOC_S_CTRL, &c) == 0;
}

bool UvcFocusControl::open(const std::string& device, int position,
                           dashcam::log::LogCallback log, std::string& why) {
    close();
    log_ = std::move(log);
    const int fd = ::open(device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) { why = "cannot open " + device + ": " + std::strerror(errno); return false; }

    auto query = [fd](unsigned id, struct v4l2_queryctrl& q) {
        std::memset(&q, 0, sizeof(q));
        q.id = id;
        return ::ioctl(fd, VIDIOC_QUERYCTRL, &q) == 0 && !(q.flags & V4L2_CTRL_FLAG_DISABLED);
    };
    auto get = [fd](unsigned id, int& value) {
        struct v4l2_control c;
        std::memset(&c, 0, sizeof(c));
        c.id = id;
        if (::ioctl(fd, VIDIOC_G_CTRL, &c) != 0) return false;
        value = c.value;
        return true;
    };
    struct v4l2_queryctrl qAuto, qAbs;
    if (!query(V4L2_CID_FOCUS_AUTO, qAuto) || !query(V4L2_CID_FOCUS_ABSOLUTE, qAbs)) {
        ::close(fd);
        why = device + " has no autofocus / absolute focus control (fixed-focus lens?)";
        return false;
    }

    // What to hand back: the state as found, not the default — someone may have
    // set the lens by hand (v4l2-ctl) before this ran.
    fd_ = fd;
    if (!get(V4L2_CID_FOCUS_AUTO, prevAuto_)) prevAuto_ = qAuto.default_value;
    prevAbsOk_ = get(V4L2_CID_FOCUS_ABSOLUTE, prevAbs_);

    const int pos = snapFocusPosition(position, qAbs.minimum, qAbs.maximum, qAbs.step);
    if (pos != position && log_)
        log_(LogLevel::WARN, "focus: FocusAbsolute " + std::to_string(position) +
                             " is not a focus position " + device + " offers (range " +
                             std::to_string(qAbs.minimum) + ".." +
                             std::to_string(qAbs.maximum) + ", step " + std::to_string(qAbs.step) +
                             ") — using " + std::to_string(pos));

    // Autofocus off FIRST: many cameras refuse (EBUSY) or ignore an absolute
    // focus write while their own autofocus is running.
    if (!setCtrl(V4L2_CID_FOCUS_AUTO, 0)) {
        why = device + ": cannot turn autofocus off: " + std::strerror(errno);
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    if (!setCtrl(V4L2_CID_FOCUS_ABSOLUTE, pos)) {
        why = device + ": cannot set focus to " + std::to_string(pos) + ": " + std::strerror(errno);
        setCtrl(V4L2_CID_FOCUS_AUTO, prevAuto_);           // leave it as found
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    device_   = device;
    position_ = pos;
    if (log_)
        log_(LogLevel::INFO, "focus: fixed at " + std::to_string(pos) + " on " + device +
                             " (autofocus off; range " + std::to_string(qAbs.minimum) + ".." +
                             std::to_string(qAbs.maximum) + ")");
    return true;
}

void UvcFocusControl::close() {
    if (fd_ < 0) return;
    // The lens position only matters if autofocus was off when found: restore it
    // while autofocus is still off, then the autofocus state itself.
    if (prevAuto_ == 0 && prevAbsOk_) setCtrl(V4L2_CID_FOCUS_ABSOLUTE, prevAbs_);
    setCtrl(V4L2_CID_FOCUS_AUTO, prevAuto_);
    ::close(fd_);
    fd_ = -1;
    if (log_)
        log_(LogLevel::INFO, std::string("focus: handed back to the camera on ") + device_ +
                             (prevAuto_ ? " (autofocus on)" : " (autofocus off, as found)"));
}

} // namespace dashcam::camera
