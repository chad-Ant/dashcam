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
 *
 * Night mode (NightModeSwitch, optional): at night the frame-time cap plus the
 * largest UVC gain is far too dark — the UGREEN's own auto-exposure reaches ~8x
 * the brightness because it uses internal gain the UVC gain control does not
 * expose (at the cost of ~20 fps).  So when frame-rate priority is pinned at
 * its ceiling and still too dark, exposure is handed to the camera; it is taken
 * back once the camera runs at full frame rate again with a picture at least as
 * bright as NightLuma (below that, frame-rate priority would be darker still —
 * e.g. a 60 fps mode, where the camera never slows down).  A take-back starts at
 * the ceiling, so if it is still too dark the camera gets exposure back within
 * seconds instead of the usual 10 s; a session start (night boot, recovery
 * restart) is treated the same way.  The price of starting at the ceiling is a
 * few over-exposed seconds when a take-back lands in daylight (e.g. 30 s after
 * leaving a long tunnel); failed take-backs at night are far more common.
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

    /// Jump to @p s (clamped to the limits) — e.g. the ceiling when taking
    /// exposure back from the camera at night.
    void setCurrent(const ExposureSetting& s);

    /// The brightest setting frame-rate priority allows.
    ExposureSetting ceiling() const { return {lim_.exposureCap, lim_.gainMax}; }

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

/// When to hand exposure to the camera at night and when to take it back (pure
/// logic, time injected — see the file comment).
class NightModeSwitch {
public:
    struct Policy {
        float  nightLuma    = 35.0f;  ///< At the ceiling and darker than this = "too dark".
        double enterSec     = 10.0;   ///< Too dark this long → camera (leaky: brief bright
                                      ///< spikes such as headlights slow the count, not reset it).
        double exitSec      = 30.0;   ///< Camera at full frame rate and >= nightLuma this long
                                      ///< → take it back.
        double exitSecMax   = 600.0;  ///< Cap for the doubling after failed take-backs.
        double probeSec     = 20.0;   ///< Probe after a take-back or a session start: this long
                                      ///< at most, and too dark for probeEnterSec in it →
                                      ///< camera (a failed take-back doubles the wait).
        double probeEnterSec = 2.0;   ///< Darkness at the ceiling that suffices while probing.
        double probeOkSec   = 2.0;    ///< Usable light (luma >= nightLuma) this long ends the
                                      ///< probe early: a later underpass needs enterSec.
        double stableSec    = 300.0;  ///< Frame-rate priority held this long → hold resets.
        float  fullRateFrac = 0.95f;  ///< "Full frame rate" = measured >= this x nominal.
    };
    enum class Mode { FrameRate, Camera };

    NightModeSwitch();                                   ///< Default policy.
    explicit NightModeSwitch(const Policy& policy) : pol_(policy), exitHold_(policy.exitSec) {}

    /**
     * @brief One observation at monotonic time @p nowSec.
     * @param atCeiling    frame-rate priority is at its brightest setting
     * @param meanLuma     measured scene brightness (0..255) — in Camera mode,
     *                     the brightness the camera's own auto-exposure achieves
     * @param measuredFps  frames per second actually arriving (NaN = unknown)
     * @param nominalFps   the camera mode's frame rate
     * @return the mode to be in now.
     */
    Mode update(double nowSec, bool atCeiling, float meanLuma, float measuredFps, float nominalFps);

    Mode   mode() const { return mode_; }
    double exitHoldSec() const { return exitHold_; }

private:
    Policy pol_;
    Mode   mode_      = Mode::FrameRate;
    double lastT_     = -1.0;
    double darkSec_   = 0.0;       ///< Leaky "too dark at the ceiling" time.
    double fullSec_   = 0.0;       ///< Leaky "camera at full rate" time.
    double exitHold_;
    double takenBackAt_ = -1.0;    ///< When exposure was last taken back from the camera.
    bool   probing_     = false;   ///< Short hand-over threshold in force (see Policy).
    bool   probeAfterTakeBack_ = false;  ///< A failed probe doubles the wait (not at start).
    double probeStart_  = 0.0;
    double okSec_       = 0.0;     ///< Consecutive usable-light time while probing.
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

    /// Enable night mode (call after open()).  Without it the controller never
    /// hands exposure to the camera (ExposureMode=framerate).
    void enableNightMode(const NightModeSwitch::Policy& policy);

    /// Feed one mean-luma measurement; applies the new setting when it changed.
    void onLuma(float meanLuma);

    /// As above, with the measured frame rate and a monotonic timestamp — what
    /// night mode needs (without night mode the extra arguments are ignored).
    void onLuma(float meanLuma, float measuredFps, double nowSec);

    /// True while the camera's own auto-exposure is in charge (night mode).
    bool nightMode() const { return night_ && night_->mode() == NightModeSwitch::Mode::Camera; }

    /// Hand exposure back to the camera (auto mode, default gain).  Idempotent;
    /// errors are ignored (the device may already be unplugged).
    void close();

    bool isOpen() const { return fd_ >= 0; }

private:
    int                       fd_ = -1;
    std::string               device_;
    dashcam::log::LogCallback log_;
    std::unique_ptr<FrameRateExposure> loop_;
    std::unique_ptr<NightModeSwitch>   night_;
    float                     nominalFps_ = 30.0f;
    ExposureSetting           applied_{-1, -1};
    int                       autoMode_    = 3;   ///< V4L2_EXPOSURE_APERTURE_PRIORITY
    int                       gainDefault_ = 0;
    bool                      wasLowLight_ = false;
    bool                      modePending_ = false;  ///< Night-mode V4L2 mode write not done yet.
    bool                      modeWarned_  = false;

    bool setCtrl(unsigned id, int value);
    void apply(float meanLuma);                   ///< Loop step + control writes.
    void writeMode(NightModeSwitch::Mode mode);   ///< Camera AE on/off; retried until it sticks.
};

} // namespace dashcam::camera

#endif // LIBCAMERA_EXPOSURE_H
