#ifndef GPS_FUNCTIONS
#define GPS_FUNCTIONS 1

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

/// GPS module uses Serial1 on MKR 1000 WiFi, or I2C on MKR Zero.

/** Return codes used by GPS functions. */
enum class GPSReturnStatus{
    OK = 0,                   ///< Operation succeeded.
    NOK_INIT_FAILED = -1,     ///< Module did not respond during initialisation.
    DATA_STALE = 1,           ///< @c getPVT() returned false (no fresh fix).
    NO_FIX = 2,               ///< Fresh PVT received, but no valid GNSS fix is available.
    NOK_SET_RATE_FAILED = -2, ///< Navigation rate could not be changed.
    NOK_TIME_INVALID = -3,    ///< PVT received but time or date validity flags not set.
    NOK_CONFIG_FAILED = -6,   ///< Module responded but rejected its I2C/PVT configuration.
    /// The shared I2C bus is wedged, so nothing was attempted.
    ///
    /// Distinct from @c NOK_INIT_FAILED, and the distinction is the whole point:
    /// a stuck bus means the receiver was never asked, so it is not evidence
    /// about the receiver at all.  Reported rather than swallowed because the
    /// repair is different — a line held low, versus a module that is absent.
    NOK_BUS_STUCK = -7,
};


/** Qualitative GPS signal strength based on number of satellites in view. */
enum GPSSignalStrength{
    EXCELLENT, ///< 9+ satellites.
    GOOD,      ///< 6–8 satellites.
    AVERAGE,   ///< 4–5 satellites.
    BAD,       ///< 3 satellites (fix possible but unreliable).
    NOSIGNAL   ///< 0–2 satellites (no position fix possible).
};

/** UTC date and time reported by the GNSS receiver. */
struct GPSDateTime {
    uint16_t year;  ///< Four-digit UTC year.
    uint8_t month;  ///< UTC month [1, 12].
    uint8_t day;    ///< UTC day of month [1, 31].
    uint8_t hour;   ///< UTC hour [0, 23].
    uint8_t minute; ///< UTC minute [0, 59].
    uint8_t second; ///< UTC second [0, 60], including a possible leap second.
    bool valid;     ///< True when both GNSS date and time are valid.
};

/**
 * @brief One coherent UBX-NAV-PVT navigation snapshot.
 *
 * Position, velocity, heading, and altitude are @c NAN unless @c fixValid is
 * true, preventing no-fix coordinates from being used as measurements.
 */
struct GPSData {
    GPSDateTime utc;          ///< Receiver UTC date and time.
    float velocityKmh;        ///< Two-dimensional ground speed in km/h.
    float headingDegrees;     ///< Course over ground in degrees [0, 360).
    float latitudeDegrees;    ///< WGS-84 latitude in decimal degrees.
    float longitudeDegrees;   ///< WGS-84 longitude in decimal degrees.
    float altitudeM;          ///< Altitude above mean sea level in metres.
    uint8_t satellites;       ///< Satellites used in the navigation solution.
    uint8_t fixType;          ///< u-blox fix type (0=no fix, 2=2D, 3=3D, 4=GNSS+DR).
    bool fixValid;            ///< True when the receiver marks the GNSS fix valid.

    /**
     * The receiver is configured and still answering.
     *
     * Maintained by the caller, not by @c getGPSData() — which sees only
     * packets, not bring-up state.  Kept here so a consumer can tell "no
     * receiver fitted or dead" from "receiver healthy, no fix yet": both show
     * NAN coordinates and @c fixValid false, and without this they are the same
     * picture.  Same reasoning as @c IMUData::devicePresent.
     */
    bool devicePresent;
};

/**
 * @brief Initialises the u-blox GNSS module over Serial1 (UART).
 *
 * Attempts connection at the custom baud rate first; if that fails it
 * reconfigures the module to the custom rate at default baud, then retries.
 * Falls back to 9600 baud as a last resort.  Sets automotive dynamic model,
 * navigation frequency, and UBX-only output.
 *
 * @param[in,out] myGNSS  SparkFun GNSS object to initialise.
 * @return @c GPSReturnStatus::OK on success, @c NOK_INIT_FAILED if the
 *         module could not be reached at any baud rate.
 */
GPSReturnStatus initializeGPS(SFE_UBLOX_GNSS &myGNSS);

/**
 * @brief Initialises the u-blox GNSS module over I2C.
 *
 * Connects at the standard u-blox I2C address (0x42). Initialises the I2C bus
 * if not already done.
 *
 * BLOCKING.  Runs the staged machine to completion in one call, so it can hold
 * the CPU for the whole bring-up.  Kept for callers that have no @c loop() to
 * drive the machine — the validation and fault-injection sketches.  The
 * production sketch must use @c gpsInitTick() instead; see its documentation for
 * what a straight-line bring-up costs there.
 *
 * @param[in,out] myGNSS  SparkFun GNSS object to initialise.
 * @return @c GPSReturnStatus::OK on success, @c NOK_INIT_FAILED if the receiver
 *         does not respond at 0x42.
 */
GPSReturnStatus initializeGPS_I2C(SFE_UBLOX_GNSS &myGNSS);

/**
 * @brief Claims the SparkFun driver's heap buffers up front. Call once, in setup().
 *
 * The driver allocates lazily and idempotently — @c packetCfg's payload inside
 * @c begin(), the @c UBX_NAV_PVT_t inside @c setAutoPVTrate() — each behind an
 * "if still null" guard, so nothing is ever allocated twice and nothing is freed
 * before the destructor runs.  The problem is not churn, it is TIMING: when no
 * receiver answers at boot, @c begin() fails before reaching either allocation,
 * and the buffers are then claimed by the first successful retry from @c loop().
 * That is an allocation after initialisation, on a path taken precisely when the
 * hardware is already misbehaving.
 *
 * Both calls made here allocate BEFORE they transmit, so the RAM is claimed even
 * with nothing on the bus, which is the case that matters.
 *
 * ONE ALLOCATION REMAINS OUTSIDE OUR CONTROL, and it is worse than "lazy":
 * @c getPortSettingsInternal() allocates @c packetUBXCFGPRT with @c new on entry
 * and @c delete s it before returning, so @c isConnected() churns a
 * @c UBX_CFG_PRT_t on EVERY call — up to three per @c begin(), on every retry,
 * from @c loop().  That is a genuine new/delete cycle after initialisation and
 * cannot be prevented from outside the library; only vendoring the GNSS driver
 * would close it.  It is a small fixed-size object on a heap nothing else
 * allocates from after setup, so fragmentation risk is low — but the
 * no-allocation-after-init rule is NOT fully satisfied for GNSS, and claiming
 * otherwise would be wrong.
 *
 * @return @c OK, or @c NOK_BUS_STUCK if the bus could not be brought up.
 */
GPSReturnStatus preallocateGPS_I2C(SFE_UBLOX_GNSS &myGNSS);

/** Steps of the staged GNSS bring-up, in execution order. */
enum class GPSInitStage : uint8_t{
    Idle = 0,      ///< Not started.
    Begin,         ///< Establish communication (up to three isConnected probes).
    SetI2COutput,  ///< Restrict the DDC port to UBX.
    SetDynModel,   ///< Automotive dynamic model.
    SetNavFreq,    ///< Navigation solution rate.
    SetNavRate,    ///< Measurements per navigation solution.
    SetAutoPVT,    ///< Ask the receiver to push PVT automatically.
    Done,          ///< Receiver configured and streaming.
    Failed,        ///< A step refused; holds the retry backoff — NOT terminal.
    /**
     * Abandoned for this boot because the previous run hung.  TERMINAL.
     *
     * Distinct from @c Failed, and the distinction is the whole point.  Failed
     * retries, which is correct for a receiver that merely refused a setting —
     * and catastrophic after a hang, because the retry re-enters the same
     * unbounded SERCOM wait, the watchdog resets the board, and the reboot loop
     * resumes at the retry interval.  Nothing moves out of this state except a
     * non-watchdog reset.
     */
    Quarantined,
};

/** Fixed upper bound on steps for the blocking wrapper; one per stage plus slack. */
#define GPS_INIT_MAX_STEPS 12u

/**
 * @brief Progress of a staged GNSS bring-up.  One instance per receiver.
 *
 * Plain aggregate, owns no memory, safe to hold at file scope and drive from
 * @c loop().
 */
struct GPSInitState{
    GPSInitStage    stage      = GPSInitStage::Idle;
    GPSInitStage    failedAt   = GPSInitStage::Idle; ///< Stage that refused, for logs.
    GPSReturnStatus lastStatus = GPSReturnStatus::OK; ///< Why the last attempt failed.
    uint32_t        nextStepMs = 0;   ///< Earliest @c millis() for the next attempt.
    uint16_t        attempts   = 0;   ///< Steps executed since the last restart.
    uint16_t        failures   = 0;   ///< Consecutive failed bring-ups, for backoff.
};

/**
 * Failed bring-ups tolerated at the fast retry rate before backing off.
 *
 * The u-blox DDC bring-up is measurably flaky — repeated runs on the bench show
 * it refusing with NOK_INIT_FAILED, NOK_SET_RATE_FAILED or NOK_CONFIG_FAILED on
 * a good fraction of attempts, then succeeding unchanged moments later.  Going
 * straight to the GPS_RETRY_MS backoff treats that ordinary flakiness like an
 * absent receiver and turns a sub-second bring-up into tens of seconds with no
 * position.
 *
 * So retry quickly a few times first, and only escalate once the failures start
 * to look like real absence rather than noise.
 */
#define GPS_INIT_FAST_RETRIES 8u

/** Delay between fast retries (ms).  Long enough to let the DDC port settle. */
#define GPS_INIT_FAST_RETRY_MS 250UL

/**
 * @brief Advances the GNSS bring-up by ONE STAGE. Call from @c loop().
 *
 * A stage is not a single UBX exchange, and the difference matters — an earlier
 * version of this comment claimed a call costs at most @c GPS_CMD_TIMEOUT_MS,
 * which is false.  Counted from the installed SparkFun 2.2.29 source:
 *
 *   @c Begin           up to 3 x isConnected()  -> up to 750 ms
 *   poll-plus-set setters (I2COutput, DynModel, NavFreq, NavRate)
 *                      2 exchanges each         -> up to 500 ms
 *   @c SetAutoPVT      1 exchange               -> up to 250 ms
 *
 * So the worst single pass is ~750 ms, not 250 ms.  That is still an enormous
 * improvement on the ~3 s straight-line bring-up it replaces, and it is bounded,
 * but it EXCEEDS @c IMU_MAX_DATA_AGE_MS.  A slow bring-up will therefore leave
 * more than @c IMU_FIFO_BACKLOG_WORDS queued and be reported as a data gap.
 * That is correct behaviour, not a defect: samples older than the freshness
 * contract must not be published as current, and the flag says so.  It is also
 * the explanation for the gap events seen at startup.
 *
 * Driving it below the freshness window would mean issuing raw UBX
 * request/response substages instead of using the library's setters — worth
 * doing only if uninterrupted inertial capture during bring-up is required.
 *
 * @c Failed is a WAITING state carrying the @c GPS_RETRY_MS backoff, not a
 * terminal one — it re-enters @c Begin by itself, so the caller never has to
 * restart the machine.
 *
 * @return The stage AFTER this step; @c Done when the receiver is configured.
 */
GPSInitStage gpsInitTick(SFE_UBLOX_GNSS &myGNSS, GPSInitState &state);

/** @brief (Re)starts the staged bring-up from the first step, immediately. */
void gpsInitBegin(GPSInitState &state);

/** @brief Forces the machine into its backoff state with a stated reason. */
void gpsInitFail(GPSInitState &state, GPSReturnStatus why);

/**
 * @brief Clears the retry backoff. Call ONLY when a PVT packet has arrived.
 *
 * Reaching @c Done proves the receiver ACKed its configuration, which is not
 * the same as working.  A receiver that accepts every setting and then streams
 * nothing would loop at the fast retry rate indefinitely if @c Done cleared the
 * counter, because the escalation could never accumulate.
 */
void gpsInitConfirmStreaming(GPSInitState &state);

/**
 * @brief Abandons GNSS for this boot. Terminal — nothing retries afterwards.
 *
 * For use after a watchdog reset, where any retry would re-enter the hang that
 * caused it.  Use @c gpsInitFail() for an ordinary refusal that should be
 * retried; the two must not be confused.
 */
void gpsInitQuarantine(GPSInitState &state);

/** @brief Short human-readable name for a stage, for logs. */
const char *gpsInitStageName(GPSInitStage stage);

/** Resets a GPS snapshot to a known invalid state. */
void initGPSData(GPSData &data);

/**
 * @brief Clears every fix-dependent field, leaving the receiver-status fields.
 *
 * Enforces the invariant @c GPSData documents: position, velocity, heading and
 * altitude are @c NAN unless @c fixValid.  Clearing @c fixValid on its own does
 * not do that — it leaves the last coordinates sitting there as plausible
 * numbers, and any consumer that reads them without first checking the flag gets
 * a stale position presented exactly like a live one.
 *
 * UTC, satellite count and fix type are deliberately kept: they came from a real
 * packet and remain true of the receiver even after the fix ages out.  Use
 * @c initGPSData() instead when the receiver itself has gone silent.
 */
void invalidateGPSFix(GPSData &data);

/**
 * @brief Reads all requested navigation data from one automatic PVT packet.
 *
 * Non-blocking: @c getPVT(0) checks for a fresh automatic UBX-NAV-PVT packet.
 * UTC and satellite fields update for every fresh packet. Position, speed,
 * heading, and altitude update only when the receiver reports a valid fix.
 *
 * A fix has to clear four independent hurdles before it is published, because
 * any one of them alone lets through a position that is not a measurement:
 *  - @c fixType is 2-D, 3-D or GNSS+dead-reckoning (a receiver with no fix still
 *    reports its last known position, and @c gnssFixOK does not always cover it);
 *  - the @c gnssFixOK flag is set;
 *  - the @c invalidLlh flag is CLEAR — u-blox sets it precisely to mark
 *    longitude/latitude/height as unusable while the rest of the packet is fine;
 *  - the coordinates land inside their physical ranges, which catches a corrupted
 *    packet that satisfied all three flags.
 *
 * @return @c OK for a valid fix, @c NO_FIX for a fresh packet without a usable
 *         fix, @c DATA_STALE when no new packet is available, or
 *         @c NOK_BUS_STUCK when the I2C bus was not safe to transact on.
 */
GPSReturnStatus getGPSData(SFE_UBLOX_GNSS &myGNSS, GPSData &data);

/**
 * @brief Reads the current WGS-84 latitude and longitude (non-blocking).
 *
 * Calls @c getPVT(0) — returns immediately with @c DATA_STALE if no fresh
 * packet is buffered.  Requires @c setAutoPVTrate(1) during initialisation so
 * the module pushes PVT data automatically at the navigation rate.
 *
 * @param[in]  myGNSS    Initialised GNSS object.
 * @param[out] latitude  Latitude in decimal degrees (negative = South).
 * @param[out] longitude Longitude in decimal degrees (negative = West).
 * @return @c OK, @c DATA_STALE, or @c NO_FIX. Outputs are set to @c NAN when
 *         a fresh packet does not contain a valid fix.
 */
GPSReturnStatus getLatLong(SFE_UBLOX_GNSS &myGNSS, float &latitude, float &longitude);

/**
 * @brief Reads the current altitude above mean sea level (non-blocking).
 *
 * Calls @c getPVT(0) — returns immediately with @c DATA_STALE if no fresh
 * packet is buffered.
 *
 * @param[in]  myGNSS   Initialised GNSS object.
 * @param[out] altitude Altitude in metres (MSL).
 * @return @c OK, @c DATA_STALE, or @c NO_FIX. Output is set to @c NAN when a
 *         fresh packet does not contain a valid fix.
 */
GPSReturnStatus getAlt(SFE_UBLOX_GNSS &myGNSS, float &altitude);

/**
 * @brief Reads latitude, longitude, and altitude in a single non-blocking call.
 *
 * Calls @c getPVT(0) once; the individual getters return cached values from
 * that packet without additional I2C traffic.  Returns @c DATA_STALE
 * immediately if no fresh packet is buffered.
 *
 * @param[in]  myGNSS    Initialised GNSS object.
 * @param[out] latitude  Latitude in decimal degrees.
 * @param[out] longitude Longitude in decimal degrees.
 * @param[out] altitude  Altitude in metres (MSL).
 * @return @c OK, @c DATA_STALE, or @c NO_FIX. Outputs are set to @c NAN when
 *         a fresh packet does not contain a valid fix.
 */
GPSReturnStatus getLatLongAlt(SFE_UBLOX_GNSS &myGNSS, float &latitude, float &longitude, float &altitude);

/**
 * @brief Reads the current 2-D ground speed (non-blocking).
 *
 * Calls @c getPVT(0) — returns immediately with @c DATA_STALE if no fresh
 * packet is buffered.
 *
 * @param[in]  myGNSS  Initialised GNSS object.
 * @param[out] speed   Ground speed in km/h.
 * @return @c OK, @c DATA_STALE, or @c NO_FIX. Output is set to @c NAN when a
 *         fresh packet does not contain a valid fix.
 */
GPSReturnStatus getSpeed(SFE_UBLOX_GNSS &myGNSS, float &speed);

/**
 * @brief Reads the current course over ground / heading (non-blocking).
 *
 * Calls @c getPVT(0) — returns immediately with @c DATA_STALE if no fresh
 * packet is buffered.
 *
 * @param[in]  myGNSS   Initialised GNSS object.
 * @param[out] heading  Heading in decimal degrees (0–360, true north).
 * @return @c OK, @c DATA_STALE, or @c NO_FIX. Output is set to @c NAN when a
 *         fresh packet does not contain a valid fix.
 */
GPSReturnStatus getHeading(SFE_UBLOX_GNSS &myGNSS, float &heading);

/**
 * @brief Reads ground speed and heading in a single non-blocking call.
 *
 * Calls @c getPVT(0) once; the individual getters return cached values from
 * that packet without additional I2C traffic.  Returns @c DATA_STALE
 * immediately if no fresh packet is buffered.
 *
 * @param[in]  myGNSS   Initialised GNSS object.
 * @param[out] speed    Ground speed in km/h.
 * @param[out] heading  Heading in decimal degrees.
 * @return @c OK, @c DATA_STALE, or @c NO_FIX. Outputs are set to @c NAN when
 *         a fresh packet does not contain a valid fix.
 */
GPSReturnStatus getSpeedHeading(SFE_UBLOX_GNSS &myGNSS, float &speed, float &heading);

/**
 * @brief Changes the navigation solution output rate.
 *
 * @param[in,out] myGNSS  Initialised GNSS object.
 * @param[in]     rateHz  Desired update rate in Hz, clamped to [1, 10].
 * @return @c GPSReturnStatus::OK on success, @c NOK_SET_RATE_FAILED if the
 *         module rejected the new rate.
 */
GPSReturnStatus setAcquisitionFrequency(SFE_UBLOX_GNSS &myGNSS, uint8_t rateHz);

/**
 * @brief Estimates GPS signal quality from the number of satellites in view.
 *
 * Uses an already-acquired snapshot and therefore does not consume or depend
 * on the SparkFun library's fresh-PVT flag.
 *
 * @param[in] data  Navigation snapshot returned by @c getGPSData().
 * @return A @c GPSSignalStrength enum value.
 */
GPSSignalStrength evaluateSignal(const GPSData &data);

/**
 * @brief Reads the current UTC date and time from the u-blox GNSS module and
 *        applies a whole-hour timezone offset.
 *
 * Non-blocking: calls @c getPVT(0), returns @c DATA_STALE if no fresh packet is
 * buffered, and returns @c NOK_TIME_INVALID if the packet's time/date validity
 * flags are not set. Uses TimeLib @c makeTime() so timezone offsets that cross
 * midnight are handled correctly.
 *
 * Requires @c setAutoPVTrate(1) during initialisation so the module pushes
 * PVT packets automatically at the navigation rate.
 *
 * @param[in]  myGNSS    Initialised GNSS object.
 * @param[out] tHour     Local hours (0–23).
 * @param[out] tMinute   Local minutes (0–59).
 * @param[out] tSecond   Local seconds (0–59).
 * @param[out] tDate     Local day of month (1–31).
 * @param[out] tMonth    Local month (1–12).
 * @param[out] tYear     Local four-digit year (e.g. 2025).
 * @param[in]  timezone  UTC offset in whole hours, clamped to [−12, +12].
 *                       Fractional-hour zones are not supported.
 * @return @c GPSReturnStatus::OK on success,
 *         @c DATA_STALE if no fresh PVT packet is buffered,
 *         @c NOK_TIME_INVALID if the time or date validity flags are not set.
 */
GPSReturnStatus getGPSDateTime(SFE_UBLOX_GNSS &myGNSS,
                               uint8_t &tHour, uint8_t &tMinute, uint8_t &tSecond,
                               uint8_t &tDate, uint8_t &tMonth, uint16_t &tYear,
                               int8_t timezone = 0);

#endif
