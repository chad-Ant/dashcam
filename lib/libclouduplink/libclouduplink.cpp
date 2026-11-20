#include "libclouduplink.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dashcam::cloud {
namespace {

constexpr double kMaxLeaseSeconds = 30.0;

UpdateResult validate(const ControlSnapshot& s, const std::optional<ControlSnapshot>& previous) {
    if (s.desiredMode != ConnectionMode::OnDemand && s.desiredMode != ConnectionMode::ReadyWhilePowered)
        return UpdateResult::Invalid;
    if (s.configVersion < 0 || s.demandGeneration < 0 ||
        (!s.channels.empty() && !s.expiresAtUnixSeconds) ||
        (s.expiresAtUnixSeconds && !std::isfinite(*s.expiresAtUnixSeconds)))
        return UpdateResult::Invalid;
    if (!previous) return UpdateResult::Accepted;

    const auto& p = *previous;
    if (s.configVersion < p.configVersion || s.demandGeneration < p.demandGeneration)
        return UpdateResult::Ignored;
    if (s.configVersion == p.configVersion && s.desiredMode != p.desiredMode)
        return UpdateResult::Invalid;
    if (s.demandGeneration == p.demandGeneration) {
        if (!(s.channels == p.channels)) return UpdateResult::Invalid;
        if (s.configVersion == p.configVersion && !s.channels.empty() &&
            s.expiresAtUnixSeconds < p.expiresAtUnixSeconds)
            return UpdateResult::Ignored;
    }
    return UpdateResult::Accepted;
}

} // namespace

ReceiptTime ReceiptTime::now() {
    const auto steady = SteadyClock::now();
    const std::chrono::duration<double> unixTime = std::chrono::system_clock::now().time_since_epoch();
    return {steady, unixTime.count()};
}

OnDemandUplink::OnDemandUplink(std::string_view deviceId) {
    if (deviceId.empty() || deviceId.find('/') != std::string_view::npos)
        throw std::invalid_argument("Expected a single vehicle document ID");
    controlPath_ = "vehicles/";
    controlPath_.append(deviceId);
    controlPath_ += "/control/current";
}

UpdateResult OnDemandUplink::applyControl(std::string_view path, bool serverConfirmed,
                                         const ControlSnapshot& s, ReceiptTime received) {
    expire(received.monotonic);
    if (closed_ || !serverConfirmed || path != controlPath_) return UpdateResult::Ignored;
    if (!std::isfinite(received.unixSeconds)) return UpdateResult::Invalid;
    const auto result = validate(s, last_);
    if (result != UpdateResult::Accepted) return result;

    const bool newLease = !last_ || s.demandGeneration != last_->demandGeneration ||
                          s.expiresAtUnixSeconds != last_->expiresAtUnixSeconds;
    last_ = s;
    if (s.channels.empty()) {
        target_ = {};
        return UpdateResult::Accepted;
    }

    const double seconds = std::clamp(*s.expiresAtUnixSeconds - received.unixSeconds, 0.0, kMaxLeaseSeconds);
    const auto remaining = std::chrono::duration_cast<SteadyClock::duration>(std::chrono::duration<double>(seconds));
    const auto candidate = received.monotonic + remaining;
    // Re-reading an identical lease after a wall-clock step cannot extend it.
    deadline_ = newLease ? candidate : std::min(deadline_, candidate);
    if (s.desiredMode != ConnectionMode::OnDemand || deadline_ <= received.monotonic ||
        deniedGeneration_ == s.demandGeneration) {
        target_ = {};
    } else if (target_.attempt == 0 || target_.demandGeneration != s.demandGeneration) {
        // Reuse the socket for channel changes, but require a fresh grant.
        const auto attempt = target_.attempt ? target_.attempt : nextAttempt_++;
        target_ = {attempt, s.demandGeneration, s.channels, {}};
    }
    return UpdateResult::Accepted;
}

UpdateResult OnDemandUplink::relayConfirmed(std::uint64_t attempt, std::int64_t generation,
                                           Channels granted, SteadyClock::time_point now) {
    expire(now);
    if (!target_.attempt || attempt != target_.attempt || generation != target_.demandGeneration)
        return UpdateResult::Ignored;
    if ((granted.telemetry && !target_.requested.telemetry) || (granted.video && !target_.requested.video))
        return UpdateResult::Invalid;
    if (granted.empty()) {
        deniedGeneration_ = generation;
        target_ = {};
    } else {
        target_.active = granted;
    }
    return UpdateResult::Accepted;
}

void OnDemandUplink::relayLost(std::uint64_t attempt) {
    if (attempt == target_.attempt) target_ = {};
}

void OnDemandUplink::close() {
    closed_ = true;
    target_ = {};
}

void OnDemandUplink::expire(SteadyClock::time_point now) {
    if (target_.attempt && now >= deadline_) target_ = {};
}

RelayTarget OnDemandUplink::target(SteadyClock::time_point now) {
    expire(now);
    return target_;
}

std::optional<SteadyClock::time_point> OnDemandUplink::nextDeadline() const {
    if (!target_.attempt) return std::nullopt;
    return deadline_;
}

} // namespace dashcam::cloud
