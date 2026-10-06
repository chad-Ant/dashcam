#include "libdeadreckoning.h"

#include <cmath>
#include <stdexcept>

namespace dashcam::deadreckoning {
namespace {
bool within(double value, double lo, double hi) {
    return std::isfinite(value) && value >= lo && value <= hi;
}
double wrap(double angle) {
    // Keep pose in double precision; do not quantise long trajectories to float.
    double result = std::remainder(angle, 2 * pi);
    return result >= pi ? result - 2 * pi : result;
}
double seconds(uint64_t newer, uint64_t older) {
    return static_cast<double>(newer - older) * 1e-9;
}
double conditioned(double raw, double previous, double dt, double tau,
                   double slew, double bound, bool& limited) {
    float value = static_cast<float>(previous);
    if (slew > 0) {
        // Reuse MKR primitive, with delta scaled by elapsed acquisition time.
        rateLimit(static_cast<float>(raw), value, static_cast<float>(slew * dt));
        if (std::abs(value - raw) > 1e-5) limited = true;
    } else {
        value = static_cast<float>(raw);
    }
    const float before = value;
    saturate(value, static_cast<float>(bound), static_cast<float>(-bound));
    limited = limited || before != value;
    const double alpha = tau > 0 ? -std::expm1(-dt / tau) : 1;
    return previous + alpha * (static_cast<double>(value) - previous);
}
}  // namespace

const char* statusName(Status status) noexcept {
    switch (status) {
    case Status::Uninitialized: return "uninitialized";
    case Status::Primed: return "primed";
    case Status::Tracking: return "tracking";
    case Status::Stationary: return "stationary-confirmed";
    case Status::InvalidInput: return "invalid-input";
    case Status::LowSpeed: return "wheel-cutoff";
    case Status::BadTime: return "bad-time";
    case Status::Stale: return "stale";
    case Status::Gap: return "gap";
    case Status::DriftLimit: return "reanchor-required";
    }
    return "unknown";
}

Estimator::Estimator(Config config) : config_(config) {
    if (!within(config.wheelbaseM, 1, 5) || !within(config.wheelKmhPerCount, .001, .1) ||
        !within(config.yawDpsPerCount, .005, 1) || !within(config.mismatchRatio, -.1, .1) ||
        !within(config.minWheelCount, 300, 3000) || !within(config.maxSpeedMps, 1, 100) ||
        !within(config.maxYawRadps, .1, 3) || !within(config.filterTauSec, 0, 2) ||
        !within(config.speedSlewMps2, 0, 50) || !within(config.yawSlewRadps2, 0, 20) ||
        !within(config.maxGapSec, .01, 1) || !within(config.maxAgeSec, .01, 5) ||
        !within(config.maxUnanchoredSec, 1, 3600) || !within(config.maxDistanceM, 1, 100000))
        throw std::invalid_argument("dead reckoning: invalid configuration");
    reset();
}

void Estimator::clearFilters() {
    speedMean_.reset();
    yawMean_.reset();
    havePrevious_ = false;
    previousSpeed_ = previousYaw_ = 0;
    previousGear_ = Gear::Unknown;
}

void Estimator::reset(double x, double y, double heading) {
    if (!within(x, -1e7, 1e7) || !within(y, -1e7, 1e7) || !std::isfinite(heading))
        throw std::invalid_argument("dead reckoning: invalid anchor");
    state_ = State{};
    state_.xM = x;
    state_.yM = y;
    state_.headingRad = wrap(heading);
    state_.calibrated = config_.calibrated;
    haveClock_ = haveOriginTime_ = false;
    highWaterNs_ = originNs_ = 0;
    clearFilters();
}

State Estimator::reject(Status reason) {
    if (haveOriginTime_) state_.continuous = false;
    state_.status = reason;
    state_.speedMps = state_.yawRadps = unavailable;
    state_.roadWheelAngleRad = state_.radiusM = unavailable;
    state_.meanSpeedMps = state_.meanYawRadps = unavailable;
    state_.limited = false;
    ++state_.rejected;
    clearFilters();
    return state_;
}

void Estimator::publish(double speed, double yaw) {
    state_.speedMps = speed;
    state_.yawRadps = yaw;
    state_.roadWheelAngleRad = state_.radiusM = unavailable;
    if (std::abs(speed) > 1e-6) {
        const double curvature = yaw / speed;
        state_.roadWheelAngleRad = std::atan(config_.wheelbaseM * curvature);
        if (std::abs(curvature) >= .002) state_.radiusM = 1 / std::abs(curvature);
    }
    float s = static_cast<float>(speed), w = static_cast<float>(yaw), mean = 0;
    state_.meanSpeedMps = speedMean_.calculate(s, mean) ? mean : unavailable;
    state_.meanYawRadps = yawMean_.calculate(w, mean) ? mean : unavailable;
}

State Estimator::update(const WheelSample& s, uint64_t now) {
    if (state_.status == Status::DriftLimit) return reject(Status::DriftLimit);
    if (s.monotonicNs > now || (haveClock_ && s.monotonicNs <= highWaterNs_))
        return reject(Status::BadTime); // never rewind the timestamp high-water mark
    highWaterNs_ = s.monotonicNs;
    haveClock_ = true;
    if (seconds(now, s.monotonicNs) > config_.maxAgeSec) return reject(Status::Stale);
    if (!s.valid || s.rearLeft > 32767 || s.rearRight > 32767 ||
        (s.gear != Gear::Forward && s.gear != Gear::Reverse && s.gear != Gear::Park && s.gear != Gear::Neutral))
        return reject(Status::InvalidInput);
    const bool stationary = s.stationaryConfirmed && s.rearLeft == 0 && s.rearRight == 0;
    if (s.stationaryConfirmed && !stationary) return reject(Status::InvalidInput);
    if (!stationary && s.gear != Gear::Forward && s.gear != Gear::Reverse)
        return reject(Status::InvalidInput);
    if (!stationary && (s.rearLeft < config_.minWheelCount || s.rearRight < config_.minWheelCount))
        return reject(Status::LowSpeed);
    const double mean = (static_cast<double>(s.rearLeft) + s.rearRight) / 2;
    const double sign = s.gear == Gear::Reverse ? -1 : 1;
    const double rawSpeed = stationary ? 0 : sign * mean * config_.wheelKmhPerCount / 3.6;
    const double rawYaw = stationary ? 0 : sign * (s.rearRight - s.rearLeft - config_.mismatchRatio * mean) * config_.yawDpsPerCount * pi / 180;
    // Reject impossible samples before filtering: saturation must not disguise faults.
    if (std::abs(rawSpeed) > config_.maxSpeedMps || std::abs(rawYaw) > config_.maxYawRadps)
        return reject(Status::InvalidInput);
    if (haveOriginTime_ && seconds(s.monotonicNs, originNs_) > config_.maxUnanchoredSec)
        return reject(Status::DriftLimit);
    if (havePrevious_ && seconds(s.monotonicNs, state_.timeNs) > config_.maxGapSec)
        return reject(Status::Gap);
    if (havePrevious_ && s.gear != previousGear_ && !stationary)
        return reject(Status::Gap); // do not interpolate unsigned wheels across a direction change
    state_.limited = false;
    double speed = rawSpeed, yaw = rawYaw;
    if (havePrevious_) {
        const double dt = seconds(s.monotonicNs, state_.timeNs);
        if (!stationary) {
            speed = conditioned(rawSpeed, previousSpeed_, dt, config_.filterTauSec,
                                config_.speedSlewMps2, config_.maxSpeedMps, state_.limited);
            yaw = conditioned(rawYaw, previousYaw_, dt, config_.filterTauSec,
                              config_.yawSlewRadps2, config_.maxYawRadps, state_.limited);
        }
        const double ds = .5 * (speed + previousSpeed_) * dt;
        const double dh = .5 * (yaw + previousYaw_) * dt;
        if (state_.distanceM + std::abs(ds) > config_.maxDistanceM)
            return reject(Status::DriftLimit);
        // Exact arc for constant (interval-averaged) v,w; stable as yaw -> 0.
        const double half = dh / 2;
        const double sinc = std::abs(half) < 1e-6 ? 1 - half * half / 6 : std::sin(half) / half;
        state_.xM += ds * sinc * std::cos(state_.headingRad + half);
        state_.yM += ds * sinc * std::sin(state_.headingRad + half);
        state_.headingRad = wrap(state_.headingRad + dh);
        state_.distanceM += std::abs(ds);
        state_.integratedSec += dt;
        state_.status = stationary ? Status::Stationary : Status::Tracking;
    } else {
        state_.status = stationary ? Status::Stationary : Status::Primed;
    }
    if (!haveOriginTime_) { originNs_ = s.monotonicNs; haveOriginTime_ = true; }
    havePrevious_ = true;
    previousSpeed_ = speed;
    previousYaw_ = yaw;
    previousGear_ = s.gear;
    state_.timeNs = s.monotonicNs;
    ++state_.accepted;
    publish(speed, yaw);
    return state_;
}

State Estimator::snapshot(uint64_t now) const noexcept {
    State result = state_;
    if (haveOriginTime_ && (now < state_.timeNs || seconds(now, state_.timeNs) > config_.maxAgeSec)) {
        result.status = Status::Stale;
        result.continuous = false;
        result.speedMps = result.yawRadps = result.roadWheelAngleRad = result.radiusM = unavailable;
        result.meanSpeedMps = result.meanYawRadps = unavailable;
    }
    return result;
}
} // namespace dashcam::deadreckoning
