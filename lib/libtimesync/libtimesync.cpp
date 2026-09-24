#include "libtimesync.h"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>

namespace dashcam::timesync {

using dashcam::log::LogLevel;

const char* sourceName(Source s) {
    switch (s) {
    case Source::Ntp: return "NTP";
    case Source::Gps: return "GPS";
    default:          return "none";
    }
}

// Unix seconds as "2026-09-24 04:42:35 UTC".
static std::string utcText(double unixSeconds) {
    const time_t t = static_cast<time_t>(std::floor(unixSeconds));
    struct tm tmBuf;
    gmtime_r(&t, &tmBuf);
    char b[32];
    std::strftime(b, sizeof(b), "%Y-%m-%d %H:%M:%S UTC", &tmBuf);
    return b;
}

bool utcToUnix(const UtcFields& u, int64_t& unixSeconds) {
    if (u.month < 1 || u.month > 12 || u.day < 1 || u.day > 31 || u.hour < 0 || u.hour > 23 ||
        u.minute < 0 || u.minute > 59 || u.second < 0 || u.second > 60 || u.year < 1970)
        return false;
    struct tm tmBuf;
    std::memset(&tmBuf, 0, sizeof(tmBuf));
    tmBuf.tm_year = u.year - 1900;
    tmBuf.tm_mon  = u.month - 1;
    tmBuf.tm_mday = u.day;
    tmBuf.tm_hour = u.hour;
    tmBuf.tm_min  = u.minute;
    tmBuf.tm_sec  = u.second == 60 ? 59 : u.second;
    const time_t t = timegm(&tmBuf);
    if (t == static_cast<time_t>(-1)) return false;
    // timegm normalises 30 Feb into 2 Mar; a date that moved did not exist.
    if (tmBuf.tm_mday != u.day || tmBuf.tm_mon != u.month - 1) return false;
    unixSeconds = static_cast<int64_t>(t);
    return true;
}

// ─── clock access ─────────────────────────────────────────────────────────────

double TimeKeeper::systemNow() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

double TimeKeeper::monotonicNow() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

bool TimeKeeper::setSystemClock(double unixSeconds, std::string& error) {
    struct timespec ts;
    ts.tv_sec  = static_cast<time_t>(std::floor(unixSeconds));
    ts.tv_nsec = static_cast<long>((unixSeconds - std::floor(unixSeconds)) * 1e9);
    if (clock_settime(CLOCK_REALTIME, &ts) == 0) return true;
    error = std::strerror(errno);
    if (errno == EPERM) error += " (needs CAP_SYS_TIME: run the container --privileged)";
    return false;
}

// ─── TimeKeeper ───────────────────────────────────────────────────────────────

TimeKeeper::TimeKeeper(const ClockPolicy& policy, dashcam::log::LogCallback log, NowFn now,
                       MonoFn mono, SetFn set)
    : pol_(policy), log_(std::move(log)), now_(std::move(now)), mono_(std::move(mono)),
      set_(std::move(set)) {}

Source TimeKeeper::lastSource() const {
    std::lock_guard<std::mutex> lock(mu_);
    return last_;
}

bool TimeKeeper::step(double target, Source src, const std::string& detail) {
    const double before = now_();
    std::string err;
    if (!set_(target, err)) {
        if (log_) log_(LogLevel::ERROR, std::string("time: cannot set the clock from ") +
                                        sourceName(src) + ": " + err);
        return false;
    }
    char off[32];
    std::snprintf(off, sizeof(off), "%+.1f s", target - before);
    if (log_) log_(LogLevel::INFO, std::string("time: clock set from ") + sourceName(src) + " (" +
                                   detail + "): " + utcText(before) + " -> " + utcText(target) +
                                   " (" + off + ")");
    last_ = src;
    // The step moved the wall clock: GPS confirmations measured against the old
    // clock are void.
    gpsAgreeing_   = 0;
    gpsPrevSecond_ = -1;
    return true;
}

bool TimeKeeper::offerNtp(double offsetSeconds, const std::string& server) {
    if (!std::isfinite(offsetSeconds)) return false;
    std::lock_guard<std::mutex> lock(mu_);
    lastNtpMono_  = mono_();
    gpsWarnedNtp_ = false;
    if (std::fabs(offsetSeconds) <= pol_.stepThresholdSec) {
        if (last_ != Source::Ntp && log_) {
            char off[32];
            std::snprintf(off, sizeof(off), "%+.3f s", offsetSeconds);
            log_(LogLevel::INFO, "time: clock confirmed by NTP " + server + " (offset " + off + ")");
        }
        last_ = Source::Ntp;
        return false;
    }
    return step(now_() + offsetSeconds, Source::Ntp, server);
}

bool TimeKeeper::offerGps(const UtcFields& utc, bool timeValid) {
    if (!timeValid || utc.year < pol_.minYear || utc.year > pol_.maxYear) return false;
    int64_t sec = 0;
    if (!utcToUnix(utc, sec)) return false;

    std::lock_guard<std::mutex> lock(mu_);
    const double mono = mono_();
    // Only a second BOUNDARY pins GPS time to within the delivery latency: the
    // first sample showing a new second arrived just after that second began.
    const bool boundary = gpsPrevSecond_ >= 0 && sec == gpsPrevSecond_ + 1;
    const bool broken   = gpsPrevSecond_ >= 0 && sec != gpsPrevSecond_ && !boundary;
    gpsPrevSecond_ = sec;
    if (broken) { gpsAgreeing_ = 0; return false; }            // skipped or went backwards
    if (!boundary) return false;

    const double gpsNow = static_cast<double>(sec) + pol_.gpsLatencySec;
    const double offset = gpsNow - mono;                       // implied wall − monotonic
    if (gpsAgreeing_ > 0 && std::fabs(offset - gpsOffset_) > pol_.gpsAgreeSec) gpsAgreeing_ = 0;
    gpsOffset_ = gpsAgreeing_ > 0 ? gpsOffset_ : offset;
    if (++gpsAgreeing_ < pol_.gpsConfirmSamples) return false;

    const double wallNow = now_();
    const double error   = gpsNow - wallNow;
    const bool   ntpFresh = lastNtpMono_ >= 0 && mono - lastNtpMono_ < pol_.ntpAuthoritySec;
    if (ntpFresh) {
        // NTP is authoritative; a large disagreement is worth one warning.
        if (std::fabs(error) > pol_.stepThresholdSec && !gpsWarnedNtp_ && log_) {
            char off[32];
            std::snprintf(off, sizeof(off), "%+.1f s", error);
            log_(LogLevel::WARN, std::string("time: GPS disagrees with the NTP-set clock by ") + off +
                                 " — keeping NTP");
            gpsWarnedNtp_ = true;
        }
        return false;
    }
    if (std::fabs(error) <= pol_.stepThresholdSec) {
        if (last_ == Source::None && log_)
            log_(LogLevel::INFO, "time: clock confirmed by GPS");
        if (last_ == Source::None) last_ = Source::Gps;
        return false;
    }
    char detail[32];
    std::snprintf(detail, sizeof(detail), "%d agreeing fixes", gpsAgreeing_);
    return step(gpsNow + (mono_() - mono), Source::Gps, detail);
}

// ─── ClockJumpDetector ────────────────────────────────────────────────────────

ClockJumpDetector::ClockJumpDetector(double thresholdSec, TimeKeeper::NowFn now,
                                     TimeKeeper::MonoFn mono)
    : threshold_(thresholdSec), now_(std::move(now)), mono_(std::move(mono)),
      base_(now_() - mono_()) {}

bool ClockJumpDetector::poll(double& jumpSec) {
    const double base = now_() - mono_();
    jumpSec = base - base_;
    base_   = base;
    return std::fabs(jumpSec) > threshold_;
}

} // namespace dashcam::timesync
