#include "c_api.h"
#include "libdeadreckoning.h"
using namespace dashcam::deadreckoning;
extern "C" {
void* dr_create(double wheelbase, double speedScale, double yawScale,
                double mismatch, double tau, double maxSpeed) {
    try {
        Config config;
        config.wheelbaseM = wheelbase;
        config.wheelKmhPerCount = speedScale;
        config.yawDpsPerCount = yawScale;
        config.mismatchRatio = mismatch;
        config.filterTauSec = tau;
        config.maxSpeedMps = maxSpeed;
        return new Estimator(config);
    } catch (...) { return nullptr; }
}
void dr_destroy(void* handle) { delete static_cast<Estimator*>(handle); }
int dr_reset(void* handle) {
    if (!handle) return 0;
    static_cast<Estimator*>(handle)->reset();
    return 1;
}
int dr_update(void* handle, uint64_t stamp, uint64_t now, uint16_t rl,
              uint16_t rr, int gear, int valid, dr_result* out) {
    if (!handle || !out || (gear != -1 && gear != 0 && gear != 1) || (valid != 0 && valid != 1)) return 0;
    WheelSample sample;
    sample.monotonicNs = stamp;
    sample.rearLeft = rl;
    sample.rearRight = rr;
    sample.gear = gear == 1 ? Gear::Forward : gear == -1 ? Gear::Reverse : Gear::Unknown;
    sample.valid = valid != 0;
    const auto state = static_cast<Estimator*>(handle)->update(sample, now);
    *out = {state.xM, state.yM, state.headingRad, state.distanceM, state.speedMps,
            state.yawRadps, static_cast<int>(state.status), state.continuous, state.limited};
    return 1;
}
const char* dr_status_name(int status) { return statusName(static_cast<Status>(status)); }
}
