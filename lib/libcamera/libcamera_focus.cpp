#include "libcamera_focus.h"
#include "libcamera_v4l2.h"

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
    return v4l2::setControl(fd_, id, value);
}

bool UvcFocusControl::open(const std::string& device, int position,
                           dashcam::log::LogCallback log, std::string& why) {
    close();
    log_ = std::move(log);
    // Owned locally until everything is in place: every early return closes it.
    v4l2::Fd fd = v4l2::openNode(device, O_RDWR);
    if (!fd.valid()) { why = "cannot open " + device + ": " + std::strerror(errno); return false; }

    struct v4l2_queryctrl qAuto, qAbs;
    if (!v4l2::queryControl(fd.get(), V4L2_CID_FOCUS_AUTO, qAuto) ||
        !v4l2::queryControl(fd.get(), V4L2_CID_FOCUS_ABSOLUTE, qAbs)) {
        why = device + " has no autofocus / absolute focus control (fixed-focus lens?)";
        return false;
    }

    // What to hand back: the state as found, not the default — someone may have
    // set the lens by hand (v4l2-ctl) before this ran.
    int prevAuto = 1, prevAbs = 0;
    if (!v4l2::getControl(fd.get(), V4L2_CID_FOCUS_AUTO, prevAuto)) prevAuto = qAuto.default_value;
    const bool prevAbsOk = v4l2::getControl(fd.get(), V4L2_CID_FOCUS_ABSOLUTE, prevAbs);

    const int pos = snapFocusPosition(position, qAbs.minimum, qAbs.maximum, qAbs.step);
    if (pos != position && log_)
        log_(LogLevel::WARN, "focus: FocusAbsolute " + std::to_string(position) +
                             " is not a focus position " + device + " offers (range " +
                             std::to_string(qAbs.minimum) + ".." +
                             std::to_string(qAbs.maximum) + ", step " + std::to_string(qAbs.step) +
                             ") — using " + std::to_string(pos));

    // Autofocus off FIRST: many cameras refuse (EBUSY) or ignore an absolute
    // focus write while their own autofocus is running.
    if (!v4l2::setControl(fd.get(), V4L2_CID_FOCUS_AUTO, 0)) {
        why = device + ": cannot turn autofocus off: " + std::strerror(errno);
        return false;
    }
    if (!v4l2::setControl(fd.get(), V4L2_CID_FOCUS_ABSOLUTE, pos)) {
        why = device + ": cannot set focus to " + std::to_string(pos) + ": " + std::strerror(errno);
        v4l2::setControl(fd.get(), V4L2_CID_FOCUS_AUTO, prevAuto);   // leave it as found
        return false;
    }
    fd_        = fd.release();
    device_    = device;
    position_  = pos;
    prevAuto_  = prevAuto;
    prevAbs_   = prevAbs;
    prevAbsOk_ = prevAbsOk;
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
