#ifndef DASHCAM_ENCODER_PAN_H
#define DASHCAM_ENCODER_PAN_H

#include <stdint.h>

namespace encoderpan {
// All angles are signed millidegrees at the OUTPUT shaft; + points right.
// Times are the MKR's uint32 micros clock. All intervals < 2^31 us.
enum class Fault : uint8_t {
    None, Configuration, EncoderInvalid, EncoderStale, EncoderOrder,
    EncoderJump, TravelLimit, CommandExpired, CommandOrder, SchedulerGap,
    FollowingError, OutputFailure
};
struct Config {
    bool calibrated = false; // Must reflect measured mechanics, not a UI bypass.
    int32_t minMdeg = -45000, maxMdeg = 45000;
    uint16_t pulseAtMinUs = 1100, pulseAtMaxUs = 1900; // may be reversed
    int32_t maxSpeedMdegS = 30000;
    int32_t encoderMaxSpeedMdegS = 600000; // physical bound, NOT commanded speed
    int32_t encoderNoiseMdeg = 100; // actual quantization/noise, not filtered noise
    int32_t maxEncoderErrorMdeg = 5000;
    int32_t settleToleranceMdeg = 3000;
    uint32_t settleUs = 200000;
    uint32_t maxEncoderGapUs = 30000;
    uint32_t maxCommandUs = 500000;
    uint32_t maxTickGapUs = 50000;
    int32_t followingErrorMdeg = 5000;
    uint32_t followingErrorUs = 1000000;
    // Optional proportional outer trim for a POSITIONAL PWM servo. Default OFF.
    // Internal servo already closes its own loop. No integral / windup state.
    uint16_t trimGainPermille = 0;
    int32_t maxTrimMdeg = 2000;
    uint8_t smoothingShift = 2; // integer IIR 1/(2^shift), used for trim ONLY
};
struct EncoderSample {
    uint32_t timeUs = 0;
    uint16_t rawCounts = 0; // AS5048B calibration aid; controller does not use it
    int32_t angleMdeg = 0;
    int32_t errorMdeg = 0;
    bool healthy = false;
    bool referenced = false; // absolute calibrated zero OR physically homed
};
struct Command {
    uint32_t sequence = 0;
    uint32_t issuedUs = 0; // clock-mapped source time, not refreshed on replay
    uint32_t lifetimeUs = 0;
    int32_t targetMdeg = 0;
};
struct Output {
    bool enabled = false;
    uint16_t pulseUs = 0;
};
struct Feedback {
    bool valid = false;
    bool settled = false;
    bool armed = false;
    Fault fault = Fault::None;
    uint32_t timeUs = 0;
    int32_t angleMdeg = 0; // measured (never last commanded angle)
    int32_t errorMdeg = 0;
    uint32_t stableSinceUs = 0;
    uint32_t validUntilUs = 0; // observed latest sample, never a future promise
};

class Controller {
public:
    explicit Controller(const Config& config);
    bool configured() const { return configured_; }
    Fault fault() const { return fault_; }
    // Call tick each pass even with no new input. No method writes hardware.
    bool observe(const EncoderSample& sample, uint32_t nowUs);
    bool arm(uint32_t nowUs); // fresh healthy encoder required; no output yet
    bool request(const Command& command, uint32_t nowUs);
    Output tick(uint32_t nowUs);
    Feedback feedback(uint32_t nowUs) const;
    void stop(); // explicit disarm; no automatic restart or centering
    bool clearFault(); // disarmed only; discards encoder state, requires fresh data
    void outputFailed(); // adapter failure -> latched off
private:
    bool validate() const;
    void fail(Fault why);
    bool fresh(uint32_t nowUs) const;
    void updateStable(const EncoderSample& sample);
    bool checkFollowing(uint32_t nowUs);
    uint16_t pulse(int32_t angleMdeg) const;
    Config c_;
    bool configured_ = false, armed_ = false, haveEncoder_ = false;
    bool haveCommand_ = false, haveSequence_ = false, haveTick_ = false;
    bool stable_ = false, following_ = false;
    Fault fault_ = Fault::None;
    EncoderSample measured_;
    Command command_;
    uint32_t lastSequence_ = 0, lastTickUs_ = 0, stableSinceUs_ = 0, followingSinceUs_ = 0;
    int32_t filtered_ = 0, trajectory_ = 0, driven_ = 0;
    int32_t stableMin_ = 0, stableMax_ = 0;
    uint32_t slewRemainder_ = 0;
};
} // namespace encoderpan
#endif
