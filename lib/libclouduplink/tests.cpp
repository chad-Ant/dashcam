#include "libclouduplink.h"

#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace dashcam::cloud;
using namespace std::chrono_literals;

namespace {
constexpr Channels telemetry{true, false};
constexpr Channels video{false, true};
constexpr Channels both{true, true};
constexpr std::string_view path = "vehicles/A/control/current";
constexpr auto accepted = UpdateResult::Accepted;
constexpr auto ignored = UpdateResult::Ignored;
constexpr auto invalid = UpdateResult::Invalid;

void check(bool ok, const char* message) {
    if (!ok) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

struct Fixture {
    OnDemandUplink uplink{"A"};
    ReceiptTime now{SteadyClock::time_point{}, 1000};
    ControlSnapshot demand{ConnectionMode::OnDemand, 0, 0, telemetry, 1015};

    UpdateResult apply() { return uplink.applyControl(path, true, demand, now); }
    RelayTarget target() { return uplink.target(now.monotonic); }
    UpdateResult grant(std::uint64_t attempt, std::int64_t generation, Channels channels) {
        return uplink.relayConfirmed(attempt, generation, channels, now.monotonic);
    }
};

void wakeAndStop() {
    Fixture f;
    check(!f.uplink.nextDeadline(), "idle has no timer");
    check(f.apply() == accepted, "server demand accepted");
    const auto attempt = f.target().attempt;
    check(attempt && f.target().active.empty(), "generation zero wakes without a media grant");
    check(f.grant(attempt, 0, both) == invalid, "grant cannot exceed demand");
    check(f.grant(attempt, 0, telemetry) == accepted, "telemetry granted");
    check(f.apply() == accepted && f.target().active == telemetry, "duplicate preserves grant");
    ++f.demand.demandGeneration;
    f.demand.channels = both;
    check(f.apply() == accepted && f.target().attempt == attempt && f.target().active.empty(),
          "new demand reuses socket and requires confirmation");
    check(f.grant(attempt, 0, telemetry) == ignored, "old generation cannot start media");
    check(f.grant(attempt, 1, video) == accepted && f.target().active == video, "partial grant");
    ++f.demand.demandGeneration;
    f.demand.channels = {};
    f.demand.expiresAtUnixSeconds.reset();
    check(f.apply() == accepted && !f.target().attempt && !f.uplink.nextDeadline(), "release all");
    check(f.grant(attempt, 1, both) == ignored, "late grant after release");
}

void validateControl() {
    Fixture f;
    check(f.uplink.applyControl(path, false, f.demand, f.now) == ignored, "cached snapshot ignored");
    check(f.uplink.applyControl("vehicles/B/control/current", true, f.demand, f.now) == ignored,
          "foreign document ignored");
    f.demand.expiresAtUnixSeconds.reset();
    check(f.apply() == invalid, "missing lease");
    f.demand.expiresAtUnixSeconds = std::numeric_limits<double>::quiet_NaN();
    check(f.apply() == invalid && !f.target().attempt, "nonfinite lease");
    f.demand.expiresAtUnixSeconds = 999;
    check(f.apply() == accepted && !f.target().attempt, "already expired on receipt");
    f.demand.expiresAtUnixSeconds = 1015;
    f.demand.configVersion = 1;
    f.demand.demandGeneration = 5;
    check(f.apply() == accepted, "valid later demand");
    f.demand.channels = both;
    check(f.apply() == invalid && f.target().requested == telemetry, "conflicting demand");
    f.demand.channels = telemetry;
    f.demand.desiredMode = ConnectionMode::ReadyWhilePowered;
    check(f.apply() == invalid, "conflicting policy");
    ++f.demand.configVersion;
    check(f.apply() == accepted && !f.target().attempt, "other policy idles Plan A");
    --f.demand.configVersion;
    f.demand.desiredMode = ConnectionMode::OnDemand;
    check(f.apply() == ignored && !f.target().attempt, "old policy cannot resume Plan A");
    f.demand.configVersion = 3;
    --f.demand.demandGeneration;
    check(f.apply() == ignored && !f.target().attempt, "old demand cannot resume Plan A");
}

void renewalAndExpiry() {
    Fixture f;
    check(f.apply() == accepted, "initial lease");
    const auto attempt = f.target().attempt;
    check(f.grant(attempt, 0, telemetry) == accepted, "initial grant");
    f.now.monotonic += 10s;
    f.now.unixSeconds += 10;
    f.demand.expiresAtUnixSeconds = 1027;
    check(f.apply() == accepted && f.target().active == telemetry, "renewal keeps reader");
    const auto renewed = f.now.monotonic + 17s;
    check(f.uplink.nextDeadline() == renewed, "arm remaining lease only");
    f.demand.expiresAtUnixSeconds = 1015;
    check(f.apply() == ignored && f.uplink.nextDeadline() == renewed, "late old renewal");
    f.now.monotonic = renewed;
    check(!f.target().attempt && !f.uplink.nextDeadline(), "silent timer expiry at deadline");

    f.now.unixSeconds = 1027;
    f.demand.expiresAtUnixSeconds = 9999;
    check(f.apply() == accepted, "fresh lease");
    const auto capped = f.now.monotonic + 30s;
    check(f.uplink.nextDeadline() == capped, "30 second maximum");
    const auto next = f.target().attempt;
    f.now.monotonic = capped;
    f.now.unixSeconds -= 600;
    check(f.grant(next, 0, telemetry) == ignored, "expired attempt cannot confirm before timer dispatch");
    check(f.apply() == accepted && !f.target().attempt, "wall clock rollback cannot revive lease");
}

void reconnectAndClose() {
    Fixture f;
    check(f.apply() == accepted, "wake");
    const auto first = f.target().attempt;
    f.uplink.relayLost(first);
    check(!f.target().attempt && !f.uplink.nextDeadline(), "disconnect releases resources");
    check(f.apply() == accepted, "authoritative reread after backoff");
    const auto second = f.target().attempt;
    check(second && second != first, "new attempt identity");
    f.uplink.relayLost(first);
    check(f.grant(first, 0, telemetry) == ignored && f.target().attempt == second,
          "old close and grant cannot affect new socket");
    check(f.grant(second, 0, {}) == accepted && !f.target().attempt, "relay denial idles");
    check(f.apply() == accepted && !f.target().attempt, "denied generation stays idle");
    ++f.demand.demandGeneration;
    check(f.apply() == accepted && f.target().attempt, "new generation after denial");
    f.uplink.close();
    check(f.apply() == ignored && !f.target().attempt && !f.uplink.nextDeadline(), "terminal close");
}
} // namespace

int main() {
    wakeAndStop();
    validateControl();
    renewalAndExpiry();
    reconnectAndClose();
    std::cout << "Plan A uplink lifecycle tests passed\n";
}
