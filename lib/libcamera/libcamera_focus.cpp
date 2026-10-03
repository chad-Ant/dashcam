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
    if (device != owedDevice_) owedAuto_ = 0;   // owed to a camera this object no longer drives
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
    // set the lens by hand (v4l2-ctl) before this ran.  Nothing is taken over
    // unless it can be handed back: an autofocus state that cannot be read
    // would be "restored" to the default (turning autofocus on for a camera
    // found in manual), and with autofocus off the lens position is the state.
    int prevAuto = 1, prevAbs = 0;
    if (!v4l2::getControl(fd.get(), V4L2_CID_FOCUS_AUTO, prevAuto)) {
        why = device + ": cannot read the autofocus state (" + std::strerror(errno) +
              "), so it could not be restored; focus left as it is";
        return false;
    }
    // Autofocus off as this object left it (a refused hand-back, leftChanged()):
    // what was found before that is still owed.  Found on again (a power cycle,
    // or by hand), nothing is.
    if (prevAuto != 0)
        owedAuto_ = 0;
    else if (owedAuto_ != 0)
        prevAuto = owedAuto_;
    const bool prevAbsOk = v4l2::getControl(fd.get(), V4L2_CID_FOCUS_ABSOLUTE, prevAbs);
    if (!prevAbsOk && prevAuto == 0) {
        why = device + ": autofocus is off and the lens position cannot be read (" +
              std::strerror(errno) + "), so it could not be restored; focus left as it is";
        return false;
    }

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
        // Leave it as found: autofocus back on if it was on (found off, the
        // refused lens write changed nothing).  If that is refused too, the
        // camera is left in manual, and the caller must not say otherwise.
        if (prevAuto != 0) {
            if (v4l2::setControl(fd.get(), V4L2_CID_FOCUS_AUTO, prevAuto)) {
                owedAuto_ = 0;
            } else {
                why += std::string("; autofocus could not be turned back on (") + std::strerror(errno) +
                       "), so it is left off";
                owedAuto_   = prevAuto;
                owedDevice_ = device;
            }
        }
        return false;
    }
    owedAuto_  = 0;   // the hold's close() hands it back now
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
    // The lens position first, while autofocus is still off (cameras refuse or
    // ignore it under their own autofocus), then the autofocus state.  With
    // autofocus off when found, the position is the focus (open() required it
    // to be readable); with autofocus on, it is the stored manual position a
    // later switch to manual moves the lens to — handed back when it was read.
    std::string failed;
    if (prevAbsOk_ && !setCtrl(V4L2_CID_FOCUS_ABSOLUTE, prevAbs_))
        failed = std::string(" (lens position: ") + std::strerror(errno) + ")";
    if (!setCtrl(V4L2_CID_FOCUS_AUTO, prevAuto_)) {
        failed += std::string(" (autofocus state: ") + std::strerror(errno) + ")";
        if (prevAuto_ != 0) {   // left off: the next open() of this device hands it back
            owedAuto_   = prevAuto_;
            owedDevice_ = device_;
        }
    }
    ::close(fd_);
    fd_ = -1;
    if (!log_) return;
    std::string state = " (autofocus off at " + std::to_string(prevAbs_) + ", as found)";
    if (prevAuto_)
        state = prevAbsOk_ ? " (autofocus on, stored lens position " + std::to_string(prevAbs_) + ")"
                           : " (autofocus on; its stored lens position could not be read)";
    if (failed.empty())
        log_(LogLevel::INFO, "focus: handed back to the camera on " + device_ + state);
    else   // e.g. unplugged: it comes back with its power-on defaults anyway
        log_(LogLevel::WARN, "focus: could not hand the focus back on " + device_ + failed);
}

} // namespace dashcam::camera
