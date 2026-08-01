#ifndef CAN_SNIFF_FUNCTIONS_H
#define CAN_SNIFF_FUNCTIONS_H 1

/**
 * @file CANSniffFunctions.h
 * @brief Passive decode of the vehicle's broadcast CAN traffic.
 *
 * Sniffing is the PRIMARY source: the ECUs already publish speed, rpm and gear
 * continuously at 50-100 Hz and far finer resolution than SAE J1979 polling can
 * reach (OBD-II quantises speed to whole km/h and needs ~500 ms for a full
 * sweep).  OBD-II remains available as a Jetson-commanded fallback.
 *
 * The ID map here is MEASURED on the target vehicle - a 2019 Honda Brio (DD1),
 * CVT, 500 kbps F-CAN tapped at OBD-II pins 6/14 - not inferred from a database.
 * Where it agrees with opendbc that is noted, but the vehicle is the authority:
 * the originally supplied table had four of seven rows wrong.
 *
 * ── SAFETY ───────────────────────────────────────────────────────────────────
 * Sniffing runs the MCP2515 in Listen-Only, where it emits neither ACK bits nor
 * error frames.  Sniff and query are therefore MUTUALLY EXCLUSIVE modes, and
 * the transition is explicit — see @c canSetMode().
 */

#include <Arduino.h>
#include <CAN.h>

#include "VehicleSignals.h"
#include "CANMap.h"

#ifndef DASHCAM_CANBUS_VENDORED_FIXES
#error "Stock arduino-CAN detected. Build with --library peripherals/mkr_zero/vendor/CANBus (see vendor/CANBus/PATCHES.md); upstream observe() requests Configuration mode, not Listen-Only, so the controller would receive nothing at all."
#endif

/** @brief Controller operating mode. Mirrors the Jetson's CMD_SET_CAN_MODE arg. */
enum class CanMode : uint8_t {
    OFF       = 0, ///< Controller not initialised.
    DISCOVER  = 1, ///< Listen-only, accept-all. Census/diagnostic use.
    SNIFF     = 2, ///< Listen-only, Honda filters, decoding.
    OBD2      = 3  ///< Normal + One-Shot, transmits 0x7DF. Bus-active.
};

/** @brief Outcome of a mode transition. */
enum class CanModeStatus : int8_t {
    OK             =  0, ///< Achieved and verified via CANSTAT.
    UNCHANGED      =  1, ///< Already in this mode; nothing done.
    NOK_RATE_LIMIT = -1, ///< Too soon after the last transition.
    NOK_CONFIG     = -2, ///< Could not reach Configuration mode.
    NOK_VERIFY     = -3, ///< CANSTAT never reported the requested mode.
    NOK_FILTER     = -4, ///< Filter programming failed.
    NOK_NO_OSM     = -5  ///< OBD2 requested but One-Shot Mode did not latch.
};

// ─── measured Honda F-CAN identifiers ─────────────────────────────────────────

#define CAN_ID_POWERTRAIN   0x17Cu ///< rpm, pedal, brake. opendbc POWERTRAIN_DATA.
#define CAN_ID_WHEEL_SPEEDS 0x1D0u ///< Four 15-bit wheel speeds. opendbc WHEEL_SPEEDS.
#define CAN_ID_GEARBOX      0x191u ///< Selector position, byte 5.
#define CAN_ID_STEER_TORQUE 0x1ABu ///< EPS motor assist torque, 9-bit unsigned.
#define CAN_ID_ENGINE_DATA  0x158u ///< Transmission speed + odometer. opendbc ENGINE_DATA.
#define CAN_ID_TURN_SIGNAL  0x294u ///< Indicator lamps, byte 0 bit 5 left / bit 6 right.

/// Wheel-speed counts are 0.01 km/h each. Never write this as `/ 100.0` — that
/// promotes to software double on this FPU-less part.
#define CAN_WHEEL_KMH_PER_COUNT   0.01f

/**
 * Yaw-rate scale: centi-degrees/s per count of rear-wheel difference.
 *
 * omega[deg/s] = (c_RR - c_RL) / (2*pi*T_rear) with counts at 0.01 km/h.
 * For a rear track near 1.47 m that is ~0.108 deg/s per count, i.e. ~10.83
 * centi-deg/s.  This constant absorbs the whole product, so the true track
 * width never has to be known — calibrate it against GNSS heading rate exactly
 * as the wheel-speed scale was calibrated against GNSS ground speed.
 */
#define CAN_YAW_CDPS_PER_COUNT    10.83f

/// Below this the reluctor wheel-speed sensors report exactly zero, so yaw rate
/// is UNAVAILABLE rather than zero. Measured: 0x1D0 read 0 while a
/// transmission-derived speed still read 1.41 km/h.
#define CAN_YAW_MIN_COUNTS        300u   ///< 3.00 km/h in wheel counts.

/// Reject anything past this as a decode fault or a locked wheel, not a turn.
#define CAN_YAW_MAX_CDPS          9000   ///< 90 deg/s.

/**
 * GNSS heading rate below which the vehicle counts as going straight, for the
 * purpose of learning the tyre-radius mismatch.
 *
 * Tight on purpose. Every degree/s of real turning that leaks into the estimate
 * biases the correction, and the correction is then applied to every subsequent
 * reading - so a loose threshold does not merely add noise, it adds a permanent
 * offset. GNSS heading is itself noisy at low speed, which is why
 * @c yawObserveStraight() also refuses samples below the wheel-sensor cutoff.
 */
#define YAW_STRAIGHT_MAX_DPS      1.0f

/// One transition per this interval, so a command storm cannot thrash the bus.
#define CAN_MODE_MIN_INTERVAL_MS  500UL

/// Frames drained per tick. Bounds the worst-case pass; leftovers wait.
#define CAN_SNIFF_MAX_PER_TICK    8u

/** @brief Freshness window for sniffed signals (ms). */
#define VEH_FRESH_SNIFF_MS        200UL
/** @brief Freshness window for OBD-II signals (ms) — a full sweep plus margin. */
#define VEH_FRESH_OBD2_MS         1000UL

/**
 * @brief Running state for the rear-wheel yaw estimator.
 *
 * Holds the tyre-radius mismatch correction, which is NOT a constant offset:
 * a radius difference produces a wheel-count difference PROPORTIONAL to speed.
 * Measured on this car at about +0.74 %, which at 40 km/h fakes ~3.5 deg/s and
 * turns straight motorway driving into a permanent 180 m left-hand curve.  It
 * moves with tyre pressure and wear, so it is re-estimated per trip rather than
 * compiled in.
 */
struct YawEstimator {
    float    epsilon;      ///< Fractional radius mismatch, (c_RR - c_RL) / c_avg.
    uint32_t straightN;    ///< Samples folded into @c epsilon so far.
    bool     calibrated;   ///< True once @c straightN passes the minimum.
    /**
     * Scales copied from the loaded map, so the hot path stays two multiplies.
     *
     * These are per-VEHICLE, not universal: CAN_MAP_DEFAULT_YAW_* are expressed
     * per 0.01 km/h count, so a map whose wheel scale differs would silently
     * mis-scale the yaw rate if the constants stayed compiled in.
     */
    float    cdpsPerCount;
    uint16_t minCounts;
};

/**
 * @brief Zeroes the estimator and takes its scales from @p m.
 * @param m Loaded map; pass nullptr to use the compiled-in defaults.
 */
void initYawEstimator(YawEstimator &y, const CanSignalMap *m = nullptr);

/**
 * @brief Folds one sample into the radius-mismatch estimate.
 *
 * Call ONLY when an independent reference says the vehicle is going straight —
 * GNSS heading rate near zero — because the whole point is to attribute the
 * residual difference to the tyres rather than to a turn.
 *
 * @param[in,out] y       Estimator state.
 * @param[in]     rawRL   Rear-left raw counts.
 * @param[in]     rawRR   Rear-right raw counts.
 */
void yawObserveStraight(YawEstimator &y, uint16_t rawRL, uint16_t rawRR);

/**
 * @brief Yaw rate in centi-deg/s, left positive, or @c INT16_MIN if unavailable.
 *
 * Uses the REAR pair only.  On this front-wheel-drive vehicle the front wheels
 * are driven, so they spin under throttle, and steered, so their speeds also
 * differ by Ackermann geometry on a different track width.  The rears are an
 * undriven, unsteered differential odometer.
 */
int16_t yawRateFromWheels(const YawEstimator &y, uint16_t rawRL, uint16_t rawRR);

/**
 * @brief Installs the signal map the decoder will use.
 *
 * Pass nullptr to fall back to the compiled-in Honda map, which exists so the
 * table-driven path can be A/B'd against the hardcoded one it replaces before
 * SD loading is in play.  Must be called before the first @c canSetMode(SNIFF):
 * the map supplies the IDs the hardware filter is programmed from.
 */
void canSniffSetMap(const CanSignalMap *m);

/** @brief The map currently installed. Never nullptr after canSniffSetMap(). */
const CanSignalMap *canSniffGetMap();

// ─── boot-time source decision ────────────────────────────────────────────────

/// Matching frames that end the probe EARLY. A live 50-100 Hz signal reaches
/// this inside half a second, so a correct map never waits the full window.
/// Twenty rather than one, so a single stray ID collision cannot lock in a map
/// meant for a different vehicle.
#define CAN_PROBE_MIN_MATCHES  20u

/// Upper bound on the probe, measured from the FIRST FRAME - not from boot.
/// Long enough to cover an ID that only wakes when the shifter leaves Park
/// (0x158 reads all zeros in P on this car), short enough that a wrong map does
/// not cost a useful minute of telemetry.
#define CAN_PROBE_WINDOW_MS    10000UL

/** @brief Where the boot-time source decision has got to. */
enum class CanProbeStage : uint8_t {
    Idle = 0,   ///< Never armed.
    Probing,    ///< Counting matching frames.
    Sniffing,   ///< TERMINAL. The map belongs to this vehicle.
    FellBack,   ///< TERMINAL. Window elapsed with no match; OBD2 entered.
    Skipped,    ///< TERMINAL. No map, or the host took the decision first.
};

/**
 * @brief Probe state.
 *
 * @c clockStarted is the load-bearing field. The window is measured from the
 * first frame of ANY id, NOT from boot: this board powers up when the rig does,
 * which on an ignition-switched supply is seconds before the driver turns the
 * key and on a permanent supply can be hours. A window measured from boot would
 * expire on a silent bus and fall back to OBD2 for a vehicle whose map was
 * perfectly good - and then transmit at a bit rate nothing had confirmed.
 * A silent bus is not evidence about the map.
 */
struct CanProbeState {
    CanProbeStage stage;
    uint32_t      startMs;
    bool          clockStarted;
};

/** @brief Arms the probe and restarts the counters. */
void canProbeArm(CanProbeState &p);

/** @brief Ends the probe without a verdict. Terminal. */
void canProbeSkip(CanProbeState &p);

/**
 * @brief Advances the probe. Non-blocking; safe to call every pass.
 * @return the stage AFTER this call. Terminal stages never change again.
 */
CanProbeStage canProbeTick(CanProbeState &p, uint32_t nowMs);

/** @brief Frames drained since the counters were reset. */
uint32_t canSniffFrameCount();
/** @brief Of those, the ones whose ID was in the map. */
uint32_t canSniffMatchCount();
/** @brief Restarts both counters. */
void canSniffResetCounters();

/**
 * @brief Brings the controller into @p mode, verified, and rate-limited.
 *
 * Never passes through Normal mode on the way to a listen-only mode: on a live
 * bus the controller would ACK frames and could emit error frames if the bit
 * timing were wrong, which is what a read-only tap must never do.
 *
 * Entering @c OBD2 sets Normal and One-Shot in ONE register write, and REFUSES
 * if OSM does not latch — OSM is the only thing that caps retransmission, and a
 * bus-active node that retries forever is precisely what must not be created
 * unattended.
 *
 * @param[in] mode   Target mode.
 * @param[in] csPin  MCP2515 chip select, for the raw OSM bit-modify.
 * @return @c CanModeStatus.
 */
CanModeStatus canSetMode(CanMode mode, int csPin);

/** @brief The mode last achieved and verified. */
CanMode canGetMode();

/**
 * @brief Drains up to @c CAN_SNIFF_MAX_PER_TICK frames and decodes them.
 *
 * No-op unless the current mode is @c SNIFF or @c DISCOVER.  Non-blocking.
 *
 * @param[in,out] v  Signals to update in place.
 * @param[in,out] y  Yaw estimator state.
 * @return Number of frames decoded this call.
 */
uint8_t tickCANSniff(VehicleSignals &v, YawEstimator &y);

#endif // CAN_SNIFF_FUNCTIONS_H
