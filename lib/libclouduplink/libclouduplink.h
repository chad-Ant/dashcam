#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace dashcam::cloud {

using SteadyClock = std::chrono::steady_clock;

// Sample both clocks on receipt. Only the steady clock drives the resulting timer.
struct ReceiptTime {
    SteadyClock::time_point monotonic;
    double unixSeconds;
    static ReceiptTime now();
};

struct Channels {
    bool telemetry = false;
    bool video = false;
    bool empty() const { return !telemetry && !video; }
};

inline bool operator==(Channels a, Channels b) {
    return a.telemetry == b.telemetry && a.video == b.video;
}

enum class ConnectionMode { OnDemand, ReadyWhilePowered };
enum class UpdateResult { Accepted, Ignored, Invalid };

// Normalized fields from the device's control document. Advance demandGeneration
// for channel changes and configVersion for policy changes. Renewals advance expiry.
struct ControlSnapshot {
    ConnectionMode desiredMode = ConnectionMode::OnDemand;
    std::int64_t configVersion = 0;
    std::int64_t demandGeneration = 0;
    Channels channels;
    std::optional<double> expiresAtUnixSeconds;
};

struct RelayTarget {
    std::uint64_t attempt = 0; // Zero releases the socket and both local readers.
    std::int64_t demandGeneration = 0;
    Channels requested;
    Channels active;           // Open only these local subscriptions.
};

// One instance per enrolled Jetson, confined to the uplink's event loop.
// After each event, reconcile target() and arm the loop timer at nextDeadline().
// When it fires, reconcile again even if no network events have arrived.
class OnDemandUplink {
public:
    explicit OnDemandUplink(std::string_view deviceId);
    OnDemandUplink(const OnDemandUplink&) = delete;
    OnDemandUplink& operator=(const OnDemandUplink&) = delete;

    [[nodiscard]] UpdateResult applyControl(std::string_view documentPath, bool serverConfirmed,
                                            const ControlSnapshot& snapshot,
                                            ReceiptTime received = ReceiptTime::now());
    [[nodiscard]] UpdateResult relayConfirmed(std::uint64_t attempt, std::int64_t generation,
                                              Channels granted,
                                              SteadyClock::time_point now = SteadyClock::now());
    void relayLost(std::uint64_t attempt);
    void close(); // Terminal. Release the adapter's resources when reconciling the idle target.

    RelayTarget target(SteadyClock::time_point now = SteadyClock::now());
    std::optional<SteadyClock::time_point> nextDeadline() const;

private:
    void expire(SteadyClock::time_point now);

    std::string controlPath_;
    std::optional<ControlSnapshot> last_;
    SteadyClock::time_point deadline_{};
    std::optional<std::int64_t> deniedGeneration_;
    std::uint64_t nextAttempt_ = 1;
    RelayTarget target_{};
    bool closed_ = false;
};

} // namespace dashcam::cloud
