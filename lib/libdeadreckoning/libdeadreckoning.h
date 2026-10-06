#ifndef DASHCAM_DEAD_RECKONING_H
#define DASHCAM_DEAD_RECKONING_H

#include <cstdint>
#include <limits>
#include "SignalProcessingFunctions.h"

namespace dashcam::deadreckoning {

// Rear-axle-centre local frame: x initial-forward, y initial-left, CCW positive.
// Single-owner object; serialize calls externally. No IO, CAN transmit, or
// allocation in update(). MKR moving averages allocate once in the constructor.
constexpr double pi = 3.14159265358979323846;
constexpr double unavailable = std::numeric_limits<double>::quiet_NaN();
enum class Gear { Unknown, Park, Neutral, Forward, Reverse };
enum class Status { Uninitialized, Primed, Tracking, Stationary, InvalidInput,
                    LowSpeed, BadTime, Stale, Gap, DriftLimit };
const char* statusName(Status status) noexcept;

struct Config {
    double wheelbaseM = 2.405;
    double wheelKmhPerCount = 0.01;
    double yawDpsPerCount = 0.1083;
    double mismatchRatio = 0.0;  // (RR-RL)/mean on externally confirmed straight runs
    double minWheelCount = 300;
    double maxSpeedMps = 40;
    double maxYawRadps = 1.6;
    double filterTauSec = 0.08;   // time-aware IIR; 0 disables
    double speedSlewMps2 = 12;    // 0 disables slew limiting, not input validation
    double yawSlewRadps2 = 3;
    double maxGapSec = 0.25;
    double maxAgeSec = 0.25;
    double maxUnanchoredSec = 120;
    double maxDistanceM = 1000;
    bool calibrated = false;    // metadata, never learned from estimated yaw
};

struct WheelSample {
    uint64_t monotonicNs = 0;    // acquisition time, NOT wall clock or delivery time
    uint16_t rearLeft = 0;
    uint16_t rearRight = 0;
    Gear gear = Gear::Unknown;  // caller must gate freshness of the separate gear signal
    bool valid = false;         // both wheels decoded, same frame, transport quality checked
    bool stationaryConfirmed = false; // independent evidence; zeros alone do not establish this
};

struct State {
    double xM = 0, yM = 0, headingRad = 0; // heading in [-pi,pi)
    double distanceM = 0, integratedSec = 0;
    double speedMps = unavailable, yawRadps = unavailable;
    double roadWheelAngleRad = unavailable, radiusM = unavailable;
    // Original MKR SIZE_8 means, diagnostic only (lagged, NAN during warmup).
    double meanSpeedMps = unavailable, meanYawRadps = unavailable;
    Status status = Status::Uninitialized;
    bool continuous = true;    // latched false after unknown travel; only reset() restores it
    bool limited = false;      // slew/saturation affected the current sample
    bool calibrated = false;
    uint64_t timeNs = 0, accepted = 0, rejected = 0;
};

class Estimator {
public:
    explicit Estimator(Config config = {}); // throws invalid_argument on bad configuration
    // Explicit external re-anchor. Finite local coordinates, bounded to 1e7 m.
    void reset(double xM = 0, double yM = 0, double headingRad = 0);
    State update(const WheelSample& sample, uint64_t nowMonotonicNs);
    State snapshot(uint64_t nowMonotonicNs) const noexcept;
    const Config& config() const noexcept { return config_; }
private:
    State reject(Status reason);
    void clearFilters();
    void publish(double speed, double yaw);
    Config config_;
    State state_;
    SimpleMovingAverage speedMean_{SIZE_8}, yawMean_{SIZE_8};
    bool havePrevious_ = false, haveClock_ = false, haveOriginTime_ = false;
    uint64_t highWaterNs_ = 0, originNs_ = 0;
    double previousSpeed_ = 0, previousYaw_ = 0;
    Gear previousGear_ = Gear::Unknown;
};
}  // namespace dashcam::deadreckoning
#endif
