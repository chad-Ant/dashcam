#include "EncoderPan.h"

namespace encoderpan {
namespace {
constexpr uint32_t kHalfClock = 0x80000000u;
int32_t clamp(int64_t n, int32_t lo, int32_t hi) {
    return static_cast<int32_t>(n < lo ? lo : n > hi ? hi : n);
}
int64_t magnitude(int64_t n) { return n < 0 ? -n : n; }
int32_t slew(int32_t from, int32_t to, int32_t step) {
    return from + clamp(int64_t(to)-from, -step, step);
}
}
Controller::Controller(const Config& config) : c_(config) {
    configured_ = validate();
    if (!configured_) fault_ = Fault::Configuration;
}
bool Controller::validate() const {
    return c_.calibrated && c_.minMdeg >= -180000 && c_.maxMdeg <= 180000
        && c_.minMdeg < c_.maxMdeg && c_.maxMdeg-c_.minMdeg < 360000
        && c_.pulseAtMinUs >= 600 && c_.pulseAtMinUs <= 2300
        && c_.pulseAtMaxUs >= 600 && c_.pulseAtMaxUs <= 2300
        && c_.pulseAtMinUs != c_.pulseAtMaxUs
        && c_.maxSpeedMdegS > 0 && c_.maxSpeedMdegS <= 360000
        && c_.encoderMaxSpeedMdegS >= c_.maxSpeedMdegS && c_.encoderMaxSpeedMdegS <= 720000
        && c_.encoderNoiseMdeg >= 0 && c_.encoderNoiseMdeg <= 2000
        && c_.maxEncoderErrorMdeg >= 0 && c_.maxEncoderErrorMdeg <= 5000
        && c_.settleToleranceMdeg > 0 && c_.settleToleranceMdeg <= 10000
        && c_.settleUs >= 10000 && c_.settleUs <= 2000000
        && c_.maxEncoderGapUs >= 1000 && c_.maxEncoderGapUs <= 100000
        && c_.maxCommandUs >= 10000 && c_.maxCommandUs <= 2000000
        && c_.maxTickGapUs >= 1000 && c_.maxTickGapUs <= 100000
        && c_.followingErrorMdeg > c_.settleToleranceMdeg && c_.followingErrorMdeg <= 90000
        && c_.followingErrorUs >= 100000 && c_.followingErrorUs <= 10000000
        && c_.trimGainPermille <= 1000 && c_.maxTrimMdeg >= 0 && c_.maxTrimMdeg <= 10000
        && c_.smoothingShift <= 8;
}
void Controller::fail(Fault why) {
    if (fault_ == Fault::None) fault_ = why;
    stop();
}
void Controller::stop() {
    armed_ = false; haveCommand_ = false; haveTick_ = false;
    stable_ = false; following_ = false; slewRemainder_ = 0;
    // Preserve sequence high-water mark: old commands cannot re-arm the motor.
}
bool Controller::clearFault() {
    if (armed_ || !configured_) return false;
    fault_ = Fault::None; haveEncoder_ = false; stable_ = false;
    return true;
}
void Controller::outputFailed() { fail(Fault::OutputFailure); }
bool Controller::fresh(uint32_t nowUs) const {
    return haveEncoder_ && uint32_t(nowUs-measured_.timeUs) <= c_.maxEncoderGapUs;
}
bool Controller::observe(const EncoderSample& s, uint32_t nowUs) {
    if (!configured_ || fault_ != Fault::None) return false;
    if (!s.healthy || !s.referenced || s.errorMdeg < 0 || s.errorMdeg > c_.maxEncoderErrorMdeg) {
        fail(Fault::EncoderInvalid); return false;
    }
    if (uint32_t(nowUs-s.timeUs) > c_.maxEncoderGapUs) {
        fail(Fault::EncoderStale); return false;
    }
    if (int64_t(s.angleMdeg)-s.errorMdeg < c_.minMdeg || int64_t(s.angleMdeg)+s.errorMdeg > c_.maxMdeg) {
        fail(Fault::TravelLimit); return false;
    }
    if (haveEncoder_) {
        const uint32_t dt = s.timeUs-measured_.timeUs;
        if (dt == 0 || dt >= kHalfClock) { fail(Fault::EncoderOrder); return false; }
        if (dt > c_.maxEncoderGapUs) { fail(Fault::EncoderStale); return false; }
        const int64_t allowed = int64_t(c_.encoderMaxSpeedMdegS)*dt/1000000 + 2*c_.encoderNoiseMdeg;
        if (magnitude(int64_t(s.angleMdeg)-measured_.angleMdeg) > allowed) {
            fail(Fault::EncoderJump); return false;
        }
        filtered_ += static_cast<int32_t>((int64_t(s.angleMdeg)-filtered_)/(1u << c_.smoothingShift));
    } else filtered_ = s.angleMdeg;
    measured_ = s; haveEncoder_ = true;
    updateStable(s);
    return true;
}
void Controller::updateStable(const EncoderSample& s) {
    // Raw encoder is used for safety and stability; filtering must not hide motion.
    if (!armed_ || !haveCommand_ || magnitude(int64_t(s.angleMdeg)-command_.targetMdeg)+s.errorMdeg > c_.settleToleranceMdeg) {
        stable_ = false; return;
    }
    const int32_t low = s.angleMdeg-s.errorMdeg;
    const int32_t high = s.angleMdeg+s.errorMdeg;
    if (!stable_) {
        stable_ = true; stableSinceUs_ = s.timeUs; stableMin_ = low; stableMax_ = high;
        return;
    }
    if (low < stableMin_) stableMin_ = low;
    if (high > stableMax_) stableMax_ = high;
    if (int64_t(stableMax_)-stableMin_ > 2*c_.settleToleranceMdeg
            || uint32_t(s.timeUs-stableSinceUs_) > 10000000u) {
        stableSinceUs_ = s.timeUs; stableMin_ = low; stableMax_ = high;
    }
}
bool Controller::arm(uint32_t nowUs) {
    if (!configured_ || fault_ != Fault::None || armed_ || !fresh(nowUs)) return false;
    armed_ = true; haveCommand_ = false; haveTick_ = true;
    lastTickUs_ = nowUs; trajectory_ = driven_ = measured_.angleMdeg;
    stable_ = false; following_ = false; slewRemainder_ = 0;
    return true;
}
bool Controller::request(const Command& cmd, uint32_t nowUs) {
    if (!armed_ || fault_ != Fault::None) return false;
    if (!fresh(nowUs)) { fail(Fault::EncoderStale); return false; }
    if (haveCommand_ && uint32_t(nowUs-command_.issuedUs) >= command_.lifetimeUs) {
        fail(Fault::CommandExpired); return false; // renewal cannot hide expiry
    }
    if (cmd.targetMdeg < c_.minMdeg || cmd.targetMdeg > c_.maxMdeg) {
        fail(Fault::TravelLimit); return false; // reject, never silently clamp request
    }
    if (cmd.lifetimeUs == 0 || cmd.lifetimeUs > c_.maxCommandUs
            || uint32_t(nowUs-cmd.issuedUs) >= cmd.lifetimeUs) {
        fail(Fault::CommandExpired); return false;
    }
    if (haveSequence_) {
        const uint32_t delta = cmd.sequence-lastSequence_;
        if (delta == 0 || delta >= kHalfClock) { fail(Fault::CommandOrder); return false; }
    }
    if (!haveCommand_ || cmd.targetMdeg != command_.targetMdeg) stable_ = false;
    command_ = cmd; haveCommand_ = true; lastSequence_ = cmd.sequence; haveSequence_ = true;
    return true;
}
bool Controller::checkFollowing(uint32_t nowUs) {
    if (magnitude(int64_t(driven_)-measured_.angleMdeg) > c_.followingErrorMdeg) {
        if (!following_) { following_ = true; followingSinceUs_ = nowUs; }
        else if (uint32_t(nowUs-followingSinceUs_) >= c_.followingErrorUs) {
            fail(Fault::FollowingError); return false;
        }
    } else following_ = false;
    return true;
}
uint16_t Controller::pulse(int32_t angleMdeg) const {
    const int64_t num = int64_t(angleMdeg-c_.minMdeg)*(int32_t(c_.pulseAtMaxUs)-c_.pulseAtMinUs);
    return static_cast<uint16_t>(c_.pulseAtMinUs + num/(c_.maxMdeg-c_.minMdeg));
}
Output Controller::tick(uint32_t nowUs) {
    Output out;
    if (!armed_ || fault_ != Fault::None) return out;
    if (!fresh(nowUs)) { fail(Fault::EncoderStale); return out; }
    const uint32_t dt = haveTick_ ? nowUs-lastTickUs_ : 0;
    if (dt > c_.maxTickGapUs) { fail(Fault::SchedulerGap); return out; }
    lastTickUs_ = nowUs; haveTick_ = true;
    if (!haveCommand_) return out;
    if (uint32_t(nowUs-command_.issuedUs) >= command_.lifetimeUs) {
        fail(Fault::CommandExpired); return out;
    }
    const uint64_t budget = uint64_t(c_.maxSpeedMdegS)*dt + slewRemainder_;
    const int32_t step = static_cast<int32_t>(budget/1000000u);
    slewRemainder_ = static_cast<uint32_t>(budget%1000000u);
    trajectory_ = slew(trajectory_,command_.targetMdeg,step);
    const int32_t trim = clamp((int64_t(trajectory_)-filtered_)*c_.trimGainPermille/1000,
                               -c_.maxTrimMdeg,c_.maxTrimMdeg);
    // Limit final output too: noise/trim must not bypass the trajectory limiter.
    driven_ = slew(driven_,clamp(int64_t(trajectory_)+trim,c_.minMdeg,c_.maxMdeg),step);
    if (!checkFollowing(nowUs)) return out;
    out.enabled = true; out.pulseUs = pulse(driven_);
    return out;
}
Feedback Controller::feedback(uint32_t nowUs) const {
    Feedback f;
    f.armed = armed_; f.fault = fault_;
    f.valid = configured_ && fault_ == Fault::None && fresh(nowUs);
    if (!f.valid) return f;
    f.timeUs = measured_.timeUs; f.validUntilUs = measured_.timeUs;
    f.angleMdeg = measured_.angleMdeg; f.errorMdeg = measured_.errorMdeg;
    const bool liveCommand = haveCommand_ && uint32_t(nowUs-command_.issuedUs) < command_.lifetimeUs;
    f.settled = armed_ && liveCommand && haveTick_ && uint32_t(nowUs-lastTickUs_) <= c_.maxTickGapUs
        && stable_ && uint32_t(measured_.timeUs-stableSinceUs_) >= c_.settleUs;
    if (f.settled) {
        f.stableSinceUs = stableSinceUs_;
        // Bound all observed readings relative to the current reported angle,
        // PLUS possible motion between samples at the calibrated speed bound.
        const int32_t span = static_cast<int32_t>(magnitude(int64_t(measured_.angleMdeg)-stableMin_)
            > magnitude(int64_t(stableMax_)-measured_.angleMdeg)
            ? magnitude(int64_t(measured_.angleMdeg)-stableMin_) : magnitude(int64_t(stableMax_)-measured_.angleMdeg));
        f.errorMdeg = span + static_cast<int32_t>(int64_t(c_.encoderMaxSpeedMdegS)*c_.maxEncoderGapUs/2000000);
    }
    return f;
}
} // namespace encoderpan
