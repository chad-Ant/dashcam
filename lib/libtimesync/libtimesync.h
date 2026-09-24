/**
 * @file libtimesync.h
 * @brief Keep the Jetson's clock right: network time first, GPS time fallback.
 *
 * The Orin Nano has no RTC battery here, so at boot the clock restarts from the
 * last shutdown time — hours or days wrong — until something corrects it.  On
 * the road there is often no network, so GPS (relayed by the ESP32-C3 bridge)
 * is the fallback time source.
 *
 * TimeKeeper is the decision logic, fed observations from wherever they come
 * from (libnetwork::queryTime(), CommLink telemetry).  Every observation is an
 * ABSOLUTE time estimate pinned to a monotonic instant, never a relative
 * offset: if anything steps the clock between the measurement and its use (GPS,
 * the host's NTP service, another thread), an offset would be applied on top of
 * the correction a second time, whereas an absolute estimate still says what
 * time it is now.  Applying one is idempotent.
 *   - An NTP observation more than @c stepThresholdSec off steps the clock.
 *   - GPS may step the clock only while no NTP observation succeeded in the last
 *     @c ntpAuthoritySec, and only after @c gpsConfirmSamples consecutive GPS
 *     second-boundaries agree with each other (one garbled frame cannot set the
 *     clock).  GPS UTC must be flagged valid and fall inside [minYear, maxYear].
 *   - Differences under @c stepThresholdSec are left to the host's own NTP
 *     service (systemd-timesyncd), which slews rather than steps.
 * The clock is read and set through injectable functions so the logic is
 * testable without privileges; the defaults use CLOCK_REALTIME (the dashcam
 * container runs --privileged, i.e. with CAP_SYS_TIME).
 *
 * ClockJumpDetector notices ANY clock step — made here or by the host — so the
 * application can react (dashcam_v0_4 starts a new segment, so file names and
 * the sidecar clock are right from that moment on).
 *
 * Thread safety: TimeKeeper's methods may be called from any thread (NTP from a
 * worker, GPS from the CommLink RX thread); ClockJumpDetector is single-thread.
 */

#ifndef LIBTIMESYNC_H
#define LIBTIMESYNC_H

#include "liblog.h"

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

namespace dashcam::timesync {

enum class Source { None, Ntp, Gps };
const char* sourceName(Source s);

/// A UTC calendar time as a GPS receiver reports it.
struct UtcFields {
    int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;
};

/// UTC fields → Unix seconds.  False for out-of-range or non-existent dates
/// (e.g. 30 February).  A leap second (:60) is read as :59.
bool utcToUnix(const UtcFields& utc, int64_t& unixSeconds);

struct ClockPolicy {
    double stepThresholdSec  = 2.0;    ///< Step only when further off than this.
    int    minYear           = 2024;   ///< Reject GPS dates outside [minYear, maxYear].
    int    maxYear           = 2100;
    int    gpsConfirmSamples = 3;      ///< Consecutive agreeing GPS second-boundaries needed.
    double gpsAgreeSec       = 0.3;    ///< How closely those must agree.
    double gpsLatencySec     = 0.05;   ///< MKR → C3 → Jetson delivery delay added to GPS time.
    double ntpAuthoritySec   = 3600.0; ///< GPS may not step within this long of an NTP success.
};

class TimeKeeper {
public:
    using NowFn    = std::function<double()>;       ///< Wall clock, Unix seconds.
    using MonoFn   = std::function<double()>;       ///< Monotonic clock, seconds.
    using SetFn    = std::function<bool(double unixSeconds, std::string& error)>;

    explicit TimeKeeper(const ClockPolicy& policy = {}, dashcam::log::LogCallback log = {},
                        NowFn now = systemNow, MonoFn mono = monotonicNow,
                        SetFn set = setSystemClock);

    /**
     * @brief An NTP/SNTP observation: the true UTC time @p utcAtReceipt (Unix
     *        seconds; server transmit time + half the round trip) at the
     *        monotonic instant @p monoAtReceipt (monotonicNow() when the reply
     *        arrived).  Samples older than an hour, or from the future, are
     *        rejected.
     * @return true when the clock was stepped.
     */
    bool offerNtp(double utcAtReceipt, double monoAtReceipt, const std::string& server);

    /**
     * @brief One GPS UTC sample, received now.  @p timeValid is the receiver's
     *        own "date and time valid" flag.
     * @return true when the clock was stepped.
     */
    bool offerGps(const UtcFields& utc, bool timeValid);

    /// The source that last confirmed or set the clock this run (None = not yet).
    Source lastSource() const;

    /// True once any trusted source has confirmed or set the clock this run.
    bool synced() const { return lastSource() != Source::None; }

    static double systemNow();
    static double monotonicNow();
    static bool   setSystemClock(double unixSeconds, std::string& error);

private:
    ClockPolicy               pol_;
    dashcam::log::LogCallback log_;
    NowFn                     now_;
    MonoFn                    mono_;
    SetFn                     set_;

    mutable std::mutex        mu_;
    Source                    last_ = Source::None;
    double                    lastNtpMono_ = -1.0;     ///< Monotonic time of the last NTP success.
    // GPS confirmation: the implied wall-minus-monotonic offset at each second
    // boundary; boundaries whose offsets agree build confidence.
    int64_t                   gpsPrevSecond_ = -1;
    double                    gpsOffset_ = 0.0;
    int                       gpsAgreeing_ = 0;
    bool                      gpsWarnedNtp_ = false;

    /// Set the clock to @p utcAtMono, the true time at monotonic instant
    /// @p monoRef — advanced to the moment of setting.  mu_ held.
    bool step(double utcAtMono, double monoRef, Source src, const std::string& detail);
};

/// Detects steps of the wall clock relative to a monotonic clock.
class ClockJumpDetector {
public:
    explicit ClockJumpDetector(double thresholdSec = 2.0,
                               TimeKeeper::NowFn now = TimeKeeper::systemNow,
                               TimeKeeper::MonoFn mono = TimeKeeper::monotonicNow);

    /// Call periodically.  Returns true (with the step size in @p jumpSec) when
    /// the wall clock moved by more than the threshold since the last call.
    bool poll(double& jumpSec);

private:
    double             threshold_;
    TimeKeeper::NowFn  now_;
    TimeKeeper::MonoFn mono_;
    double             base_;   ///< wall − monotonic at the last poll
};

} // namespace dashcam::timesync

#endif // LIBTIMESYNC_H
