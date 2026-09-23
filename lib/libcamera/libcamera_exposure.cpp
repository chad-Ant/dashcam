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
    applied_ = {-1, -1};
    wasLowLight_ = false;
    onLuma(NAN);                                  // apply the starting setting

    if (log_)
        log_(LogLevel::INFO, "exposure: frame-rate priority on " + device + " (exposure <= " +
                             ms(lim.exposureCap) + ", gain " +
                             std::to_string(lim.gainMin) + ".." + std::to_string(lim.gainMax) +
                             ", target luma " + std::to_string(targetLuma) + ")");
    return true;
}

void UvcExposureControl::onLuma(float meanLuma) {
    if (fd_ < 0 || !loop_) return;
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
}

} // namespace dashcam::camera
