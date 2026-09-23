/**
 * @file libcamera_exposure.h
 * @brief Frame-rate-priority auto-exposure for UVC cameras.
 *
 * Many UVC cameras (the UGREEN 4K among them) run their own auto-exposure in
 * "aperture priority" mode and, in low light, lengthen the exposure past one
 * frame time — silently halving the frame rate.  Some expose a control to
 * forbid that ("Exposure, Dynamic Framerate"); for cameras that do not, the
 * exposure has to be run from software instead.
 *
 * FrameRateExposure is that software loop, as pure logic: given the scene's
 * mean luma it returns the next (exposure, gain), never letting the exposure
 * exceed the frame time.  Brightness is raised with exposure first (cleanest
 * image) up to the frame-time cap, then with gain.  In the dark the picture
 * therefore gets darker and noisier instead of slower — and exposure stays
 * short, so there is less motion blur.
 *
 * UvcExposureControl applies it to a V4L2 device (manual exposure mode +
 * exposure_time_absolute + gain) and restores the camera's own auto-exposure
 * on close().  Controls can be set while another process/element streams.
 */

#ifndef LIBCAMERA_EXPOSURE_H
#define LIBCAMERA_EXPOSURE_H

#include "liblog.h"

#include <memory>
#include <string>

namespace dashcam::camera {

/// One exposure setting: V4L2 exposure_time_absolute (100 µs units) and gain.
struct ExposureSetting {
    int exposure = 0;
    int gain     = 0;
    bool operator==(const ExposureSetting& o) const {
        return exposure == o.exposure && gain == o.gain;
    }
    bool operator!=(const ExposureSetting& o) const { return !(*this == o); }
};

/// Frame-rate-priority exposure loop (pure logic, no I/O).
class FrameRateExposure {
public:
    struct Limits {
        int exposureMin = 1;    ///< Device minimum (100 µs units).
        int exposureCap = 330;  ///< Frame-time cap: never exceeded (≤ device max).
        int gainMin     = 0;
        int gainMax     = 15;
    };

    /// Frame-time cap for a camera running at @p fps, within [min, deviceMax]:
    /// a small margin below 1/fps so the sensor never has to stretch a frame.
    static int capForFps(float fps, int exposureMin, int exposureMax);

    FrameRateExposure(const Limits& limits, int targetLuma);

    /// Next setting for a measured mean luma (0..255).  Holds inside a ±10 %
    /// dead band; otherwise moves toward the target with a damped, clamped step.
    ExposureSetting update(float meanLuma);

    ExposureSetting current() const { return cur_; }

    /// True when the setting is at the cap with maximum gain (nothing left).
    bool atCeiling() const { return cur_.exposure >= lim_.exposureCap && cur_.gain >= lim_.gainMax; }

    /// True when gain is in use (exposure is at the frame-time cap).
    bool lowLight() const { return cur_.gain > lim_.gainMin; }

private:
    Limits          lim_;
    int             target_;
    ExposureSetting cur_;

    double          brightness(const ExposureSetting& s) const;   ///< exposure × gain factor
    ExposureSetting allocate(double b) const;                      ///< inverse of brightness()
};

/// Applies FrameRateExposure to a V4L2 device.
class UvcExposureControl {
public:
    UvcExposureControl() = default;
    ~UvcExposureControl();

    UvcExposureControl(const UvcExposureControl&)            = delete;
    UvcExposureControl& operator=(const UvcExposureControl&) = delete;

    /**
     * @brief Take over exposure on @p device (camera running at @p fps).
     * @return false (with @p why) when the device lacks manual exposure,
     *         exposure_time_absolute or gain controls, or cannot be opened —
     *         the camera's own auto-exposure is then left untouched.
     */
    bool open(const std::string& device, float fps, int targetLuma,
              dashcam::log::LogCallback log, std::string& why);

    /// Feed one mean-luma measurement; applies the new setting when it changed.
    void onLuma(float meanLuma);

    /// Hand exposure back to the camera (auto mode, default gain).  Idempotent;
    /// errors are ignored (the device may already be unplugged).
    void close();

    bool isOpen() const { return fd_ >= 0; }

private:
    int                       fd_ = -1;
    std::string               device_;
    dashcam::log::LogCallback log_;
    std::unique_ptr<FrameRateExposure> loop_;
    ExposureSetting           applied_{-1, -1};
    int                       autoMode_    = 3;   ///< V4L2_EXPOSURE_APERTURE_PRIORITY
    int                       gainDefault_ = 0;
    bool                      wasLowLight_ = false;

    bool setCtrl(unsigned id, int value);
};

} // namespace dashcam::camera

#endif // LIBCAMERA_EXPOSURE_H
