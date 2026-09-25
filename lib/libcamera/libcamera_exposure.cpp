#include "libcamera_exposure.h"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace dashcam::camera {

using dashcam::log::LogLevel;

// ─── tuning ───────────────────────────────────────────────────────────────────

// Measured on the UGREEN 4K: gain 0→15 brightened the picture as much as
// tripling the exposure (≈3.3×), so each gain step adds ≈15 % brightness.
// The loop corrects any error in this model; it only sets the step size.
static constexpr double kGainStep = 2.3 / 15.0;

// Luma is not linear in light (camera tone curve): 3.3× exposure raised the
// mean luma 2.6×, i.e. luma ∝ light^0.8.  Correct in light units.
static constexpr double kLumaExponent = 1.25;     // 1 / 0.8

static constexpr double kDeadBand = 0.10;          // hold within ±10 % of target
static constexpr double kStepMin  = 0.4;           // largest darkening step
static constexpr double kStepMax  = 2.5;           // largest brightening step
static constexpr float  kSaturated = 245.0f;       // clipped scene: step down hard

// exposure_time_absolute (100 µs units) as "12.3 ms".
static std::string ms(int units) {
    char b[16];
    std::snprintf(b, sizeof(b), "%.1f ms", units / 10.0);
    return b;
}

// ─── FrameRateExposure ────────────────────────────────────────────────────────

int FrameRateExposure::capForFps(float fps, int exposureMin, int exposureMax) {
    if (!(fps > 0.0f)) fps = 30.0f;
    // 100 µs units; keep ~1 ms margin under the frame time (30 fps → 323).
    const int frame = static_cast<int>(10000.0f / fps);
    return std::clamp(frame - 10, exposureMin, exposureMax);
}

FrameRateExposure::FrameRateExposure(const Limits& limits, int targetLuma)
    : lim_(limits), target_(std::clamp(targetLuma, 10, 240)) {
    if (lim_.exposureCap < lim_.exposureMin) lim_.exposureCap = lim_.exposureMin;
    if (lim_.gainMax < lim_.gainMin)         lim_.gainMax     = lim_.gainMin;
    // Start at a third of the frame time, no gain: a middle guess that the loop
    // corrects within one or two updates in daylight or at night.
    cur_ = {std::max(lim_.exposureMin, lim_.exposureCap / 3), lim_.gainMin};
}

double FrameRateExposure::brightness(const ExposureSetting& s) const {
    return s.exposure * (1.0 + kGainStep * (s.gain - lim_.gainMin));
}

ExposureSetting FrameRateExposure::allocate(double b) const {
    ExposureSetting s;
    if (b <= lim_.exposureCap) {                      // exposure alone suffices
        s.exposure = std::clamp(static_cast<int>(std::lround(b)), lim_.exposureMin, lim_.exposureCap);
        s.gain     = lim_.gainMin;
        return s;
    }
    s.exposure = lim_.exposureCap;                    // cap reached: add gain
    const double g = (b / lim_.exposureCap - 1.0) / kGainStep + lim_.gainMin;
    s.gain = std::clamp(static_cast<int>(std::lround(g)), lim_.gainMin, lim_.gainMax);
    return s;
}

ExposureSetting FrameRateExposure::update(float meanLuma) {
    if (!std::isfinite(meanLuma)) return cur_;
    const double luma = std::max(1.0, static_cast<double>(meanLuma));
    double step;
    if (meanLuma >= kSaturated) {
        step = kStepMin;                              // clipped: its value says little
    } else {
        const double ratio = target_ / luma;
        if (std::fabs(ratio - 1.0) <= kDeadBand) return cur_;
        step = std::clamp(std::pow(ratio, kLumaExponent), kStepMin, kStepMax);
    }
    const ExposureSetting next = allocate(brightness(cur_) * step);
    // Rounding can stall a small correction at the bottom of the range; nudge
    // one unit in the wanted direction so the loop always makes progress.
    if (next == cur_) {
        ExposureSetting n = cur_;
        if (step > 1.0) {
            if (n.exposure < lim_.exposureCap) ++n.exposure;
            else if (n.gain < lim_.gainMax)    ++n.gain;
        } else {
            if (n.gain > lim_.gainMin)            --n.gain;
            else if (n.exposure > lim_.exposureMin) --n.exposure;
        }
        cur_ = n;
        return cur_;
    }
    cur_ = next;
    return cur_;
}

void FrameRateExposure::setCurrent(const ExposureSetting& s) {
    cur_.exposure = std::clamp(s.exposure, lim_.exposureMin, lim_.exposureCap);
    cur_.gain     = std::clamp(s.gain, lim_.gainMin, lim_.gainMax);
}

// ─── NightModeSwitch ──────────────────────────────────────────────────────────

NightModeSwitch::NightModeSwitch() : NightModeSwitch(Policy{}) {}

NightModeSwitch::Mode NightModeSwitch::update(double nowSec, bool atCeiling, float meanLuma,
                                              float measuredFps, float nominalFps) {
    // Leaky accumulators: a condition that holds adds time; one that lapses
    // drains it twice as fast, so a headlight flash or a lit junction slows the
    // count without restarting it, while a real change of light wins quickly.
    const bool first = lastT_ < 0;
    const double dt = first ? 0.0 : std::clamp(nowSec - lastT_, 0.0, 2.0);
    lastT_ = nowSec;
    auto startProbe = [&](bool afterTakeBack) {
        probing_ = true;
        probeAfterTakeBack_ = afterTakeBack;
        probeStart_ = nowSec;
        okSec_ = 0.0;
    };
    // A session starts as a probe: the loop races to its ceiling within ~1.5 s
    // when it is dark, and a night boot or recovery restart should not record
    // 10 more near-black seconds before the camera takes over.
    if (first) startProbe(false);
    auto leak = [dt](double& acc, bool holds) {
        acc = holds ? acc + dt : std::max(0.0, acc - 2.0 * dt);
    };

    if (mode_ == Mode::FrameRate) {
        // Held long enough since the last take-back: the back-off is over.
        if (takenBackAt_ >= 0 && nowSec - takenBackAt_ >= pol_.stableSec) {
            exitHold_    = pol_.exitSec;
            takenBackAt_ = -1.0;
        }
        leak(darkSec_, atCeiling && std::isfinite(meanLuma) && meanLuma < pol_.nightLuma);
        // Probing (just after a take-back, which starts at the ceiling, or a
        // session start): a dark reading at the ceiling is conclusive — hand
        // over after probeEnterSec instead of a 10 s dark gap.  Usable light
        // ends the probe (the first reading after a take-back can still be a
        // camera-AE frame, hence probeOkSec rather than one sample).
        if (probing_) {
            okSec_ = std::isfinite(meanLuma) && meanLuma >= pol_.nightLuma ? okSec_ + dt : 0.0;
            if (okSec_ >= pol_.probeOkSec || nowSec - probeStart_ > pol_.probeSec) probing_ = false;
        }
        if (darkSec_ >= (probing_ ? pol_.probeEnterSec : pol_.enterSec)) {
            // Still dark when trying to take exposure back: that take-back was
            // premature — wait twice as long before the next one.
            if (probing_ && probeAfterTakeBack_)
                exitHold_ = std::min(exitHold_ * 2.0, pol_.exitSecMax);
            mode_    = Mode::Camera;
            darkSec_ = 0.0;
            fullSec_ = 0.0;
            probing_ = false;
        }
    } else {
        // The camera lengthens exposure (drops frames) only when it needs the
        // light; back at full rate for a while means there is light again —
        // unless its picture is still darker than nightLuma: frame-rate
        // priority, with less gain, could only be darker (a 60 fps mode never
        // slows down, so the frame rate alone says nothing there).
        const bool fullRate = std::isfinite(measuredFps) && nominalFps > 0 &&
                              measuredFps >= pol_.fullRateFrac * nominalFps;
        const bool bright   = std::isfinite(meanLuma) && meanLuma >= pol_.nightLuma;
        leak(fullSec_, fullRate && bright);
        if (fullSec_ >= exitHold_) {
            mode_        = Mode::FrameRate;
            fullSec_     = 0.0;
            darkSec_     = 0.0;
            takenBackAt_ = nowSec;
            startProbe(true);
        }
    }
    return mode_;
}

// ─── UvcExposureControl ───────────────────────────────────────────────────────

UvcExposureControl::~UvcExposureControl() { close(); }

bool UvcExposureControl::setCtrl(unsigned id, int value) {
    struct v4l2_control c;
    std::memset(&c, 0, sizeof(c));
    c.id    = id;
    c.value = value;
    return ::ioctl(fd_, VIDIOC_S_CTRL, &c) == 0;
}

bool UvcExposureControl::open(const std::string& device, float fps, int targetLuma,
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
    struct v4l2_queryctrl qAuto, qExp, qGain;
    if (!query(V4L2_CID_EXPOSURE_AUTO, qAuto) || !query(V4L2_CID_EXPOSURE_ABSOLUTE, qExp) ||
        !query(V4L2_CID_GAIN, qGain)) {
        ::close(fd);
        why = device + " lacks manual exposure / exposure time / gain controls";
        return false;
    }

    fd_          = fd;
    device_      = device;
    autoMode_    = qAuto.default_value;          // the camera's own AE mode
    gainDefault_ = qGain.default_value;
    if (!setCtrl(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL)) {
        why = device + ": cannot switch to manual exposure: " + std::strerror(errno);
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    FrameRateExposure::Limits lim;
    lim.exposureMin = std::max(1, qExp.minimum);
    lim.exposureCap = FrameRateExposure::capForFps(fps, lim.exposureMin, qExp.maximum);
    lim.gainMin     = qGain.minimum;
    lim.gainMax     = qGain.maximum;
    loop_    = std::make_unique<FrameRateExposure>(lim, targetLuma);
    night_.reset();
    nominalFps_ = fps > 0.0f ? fps : 30.0f;
    applied_ = {-1, -1};
    wasLowLight_ = false;
    modePending_ = false;
    modeWarned_  = false;
    onLuma(NAN);                                  // apply the starting setting

    if (log_)
        log_(LogLevel::INFO, "exposure: frame-rate priority on " + device + " (exposure <= " +
                             ms(lim.exposureCap) + ", gain " +
                             std::to_string(lim.gainMin) + ".." + std::to_string(lim.gainMax) +
                             ", target luma " + std::to_string(targetLuma) + ")");
    return true;
}

void UvcExposureControl::enableNightMode(const NightModeSwitch::Policy& policy) {
    night_ = std::make_unique<NightModeSwitch>(policy);
    if (log_)
        log_(LogLevel::INFO, "exposure: auto night mode on (camera takes over below luma " +
                             std::to_string(static_cast<int>(policy.nightLuma)) + " at the ceiling)");
}

void UvcExposureControl::onLuma(float meanLuma) { onLuma(meanLuma, NAN, -1.0); }

void UvcExposureControl::onLuma(float meanLuma, float measuredFps, double nowSec) {
    if (fd_ < 0 || !loop_) return;
    if (!night_ || nowSec < 0 || !std::isfinite(meanLuma)) {
        if (!nightMode() && !modePending_) apply(meanLuma);
        return;
    }
    const NightModeSwitch::Mode before = night_->mode();
    if (before == NightModeSwitch::Mode::FrameRate && !modePending_) apply(meanLuma);
    const NightModeSwitch::Mode now =
        night_->update(nowSec, loop_->atCeiling(), meanLuma, measuredFps, nominalFps_);
    if (now != before) {
        if (now == NightModeSwitch::Mode::Camera) {
            if (log_) log_(LogLevel::INFO, "exposure: night mode — the camera's own auto-exposure takes "
                                           "over (far brighter; frame rate may drop to ~20 fps)");
        } else {
            loop_->setCurrent(loop_->ceiling());   // it was dark: start bright, the loop trims
            char hold[32];
            std::snprintf(hold, sizeof(hold), "%.0f s", night_->exitHoldSec());
            if (log_) log_(LogLevel::INFO, std::string("exposure: light is back — frame-rate priority "
                                                       "again (next night-mode exit waits ") + hold + ")");
        }
        modePending_ = true;
    }
    if (modePending_) writeMode(now);
}

// Puts the camera in the mode the switch chose.  A control write can fail on a
// transient USB error; the switch has already moved, so the write is retried on
// every sample until it sticks rather than leaving the camera in the wrong mode
// (e.g. manual at default gain all night).
void UvcExposureControl::writeMode(NightModeSwitch::Mode mode) {
    const bool camera = mode == NightModeSwitch::Mode::Camera;
    // Camera: auto first, so a failure leaves the manual ceiling, not manual at
    // default gain.  Frame rate: manual, then the loop's setting.
    bool ok = camera ? setCtrl(V4L2_CID_EXPOSURE_AUTO, autoMode_) && setCtrl(V4L2_CID_GAIN, gainDefault_)
                     : setCtrl(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
    if (!ok) {
        if (!modeWarned_ && log_)
            log_(LogLevel::WARN, "exposure: cannot switch " + device_ + " to " +
                                 (camera ? "auto" : "manual") + " exposure (" + std::strerror(errno) +
                                 ") — retrying");
        modeWarned_ = true;
        return;
    }
    if (modeWarned_ && log_)
        log_(LogLevel::INFO, std::string("exposure: switched ") + device_ + " to " +
                             (camera ? "auto" : "manual") + " exposure after retrying");
    modePending_ = false;
    modeWarned_  = false;
    applied_     = {-1, -1};
    if (!camera) apply(NAN);                      // its own write failures retry via apply()
}

void UvcExposureControl::apply(float meanLuma) {
    const ExposureSetting s = std::isfinite(meanLuma) ? loop_->update(meanLuma) : loop_->current();
    if (s == applied_) return;
    bool ok = true;
    if (s.exposure != applied_.exposure) ok = setCtrl(V4L2_CID_EXPOSURE_ABSOLUTE, s.exposure) && ok;
    if (s.gain != applied_.gain)         ok = setCtrl(V4L2_CID_GAIN, s.gain) && ok;
    if (!ok) {
        if (log_) log_(LogLevel::DEBUG, "exposure: control write failed on " + device_ + ": " +
                                        std::strerror(errno));
        return;                                   // retried on the next sample
    }
    applied_ = s;
    if (log_) {
        log_(LogLevel::DEBUG, "exposure: " + ms(s.exposure) + ", gain " + std::to_string(s.gain) +
                              (std::isfinite(meanLuma) ? " (luma " +
                               std::to_string(static_cast<int>(meanLuma)) + ")" : std::string()));
        const bool low = loop_->lowLight();
        if (low != wasLowLight_)
            log_(LogLevel::INFO, low ? "exposure: low light — exposure at the frame-time cap, "
                                       "adding gain (frame rate held)"
                                     : "exposure: enough light — gain back to minimum");
        wasLowLight_ = low;
    }
}

void UvcExposureControl::close() {
    if (fd_ >= 0) {
        setCtrl(V4L2_CID_GAIN, gainDefault_);
        setCtrl(V4L2_CID_EXPOSURE_AUTO, autoMode_);
        ::close(fd_);
        fd_ = -1;
        if (log_) log_(LogLevel::INFO, "exposure: handed back to the camera on " + device_);
    }
    loop_.reset();
    night_.reset();
}

} // namespace dashcam::camera
