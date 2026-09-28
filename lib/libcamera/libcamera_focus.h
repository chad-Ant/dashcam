/**
 * @file libcamera_focus.h
 * @brief Fixed focus for UVC cameras — continuous autofocus off, lens held.
 *
 * A dashcam behind a windshield is the worst case for contrast autofocus: rain,
 * wiper blades, dirt and reflections on the glass, and night scenes of point
 * lights all give it something nearer than the road to lock onto, and every
 * hunt is seconds of blurred footage. Holding the lens at one position (the
 * road, i.e. infinity on most lenses) removes that at no cost for a camera
 * whose subject is always far away.
 *
 * UvcFocusControl turns V4L2_CID_FOCUS_AUTO off and sets V4L2_CID_FOCUS_ABSOLUTE,
 * and hands back on close() exactly the focus state it found. Like the
 * exposure controls, these can be written while another process or element
 * streams. The absolute scale is the camera's own: list it with
 * `v4l2-ctl -d <device> --list-ctrls` and pick the value by recording.
 */

#ifndef LIBCAMERA_FOCUS_H
#define LIBCAMERA_FOCUS_H

#include "liblog.h"

#include <string>

namespace dashcam::camera {

/**
 * @brief The focus_absolute value actually written for @p wanted: clamped into
 *        [@p min, @p max] and snapped down onto the control's @p step grid
 *        (a step below 1 counts as 1). Pure logic.
 */
int snapFocusPosition(int wanted, int min, int max, int step);

/// Holds a UVC camera's focus fixed for as long as it is open.
class UvcFocusControl {
public:
    UvcFocusControl() = default;
    ~UvcFocusControl();

    UvcFocusControl(const UvcFocusControl&)            = delete;
    UvcFocusControl& operator=(const UvcFocusControl&) = delete;

    /**
     * @brief Continuous autofocus off on @p device, lens held at @p position
     *        (clamped/snapped to the camera's range; a clamp is logged).
     * @return false (with @p why) when the device cannot be opened or has no
     *         autofocus / absolute-focus control, the state to hand back cannot
     *         be read (the autofocus state; with autofocus off, also the lens
     *         position), or a write is refused — the camera's focus is then left
     *         exactly as it was.
     */
    bool open(const std::string& device, int position,
              dashcam::log::LogCallback log, std::string& why);

    /// Restore the focus state found at open(). Idempotent; a refused write is
    /// logged as WARN and otherwise ignored (the device may be unplugged).
    void close();

    bool isOpen() const { return fd_ >= 0; }
    int  position() const { return position_; }   ///< Value written; valid while open.

private:
    int                       fd_ = -1;
    std::string               device_;
    dashcam::log::LogCallback log_;
    int                       position_  = 0;
    int                       prevAuto_  = 1;     ///< FOCUS_AUTO as found.
    int                       prevAbs_   = 0;     ///< FOCUS_ABSOLUTE as found.
    bool                      prevAbsOk_ = false; ///< prevAbs_ could be read.

    bool setCtrl(unsigned id, int value);
};

} // namespace dashcam::camera

#endif // LIBCAMERA_FOCUS_H
