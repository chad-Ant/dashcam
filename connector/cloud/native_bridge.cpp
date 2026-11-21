// Small C ABI for the asyncio owner. All calls are synchronous on the same thread.
#include "../../lib/libclouduplink/libclouduplink.h"
#include <cmath>
#include <cstdint>
using namespace dashcam::cloud;

extern "C" {
struct State {
    std::uint64_t attempt;
    std::int64_t generation;
    int requested, active;
    double remaining;
};
static Channels channels(int mask) { return {bool(mask & 1), bool(mask & 2)}; }
static int mask(Channels c) { return int(c.telemetry) + 2 * int(c.video); }
void* dc_create(const char* vehicle) {
    try { return new OnDemandUplink(vehicle, true); } catch (...) { return nullptr; }
}
void dc_destroy(OnDemandUplink* core) { delete core; }
int dc_apply(OnDemandUplink* core, const char* path, bool current, bool ready,
             std::int64_t version, std::int64_t generation, int wanted, double until) {
    ControlSnapshot s{ready ? ConnectionMode::ReadyWhilePowered : ConnectionMode::OnDemand,
                      version, generation, channels(wanted), std::nullopt};
    if (std::isfinite(until)) s.expiresAtUnixSeconds = until;
    return static_cast<int>(core->applyControl(path, current, s));
}
void dc_confirm(OnDemandUplink* core, std::uint64_t attempt, std::int64_t generation, int grant) {
    (void)core->relayConfirmed(attempt, generation, channels(grant));
}
State dc_state(OnDemandUplink* core) {
    const auto now = SteadyClock::now();
    const auto state = core->target(now);
    const auto deadline = core->nextDeadline();
    const double remaining = deadline ? std::chrono::duration<double>(*deadline - now).count() : -1;
    return {state.attempt, state.demandGeneration, mask(state.requested), mask(state.active), remaining};
}
}
