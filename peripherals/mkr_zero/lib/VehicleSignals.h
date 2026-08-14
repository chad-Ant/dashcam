#ifndef VEHICLE_SIGNALS_H
#define VEHICLE_SIGNALS_H 1

/**
 * @file VehicleSignals.h
 * @brief The one structure both CAN sniffing and OBD-II polling fill in.
 *
 * Two sources, one template, so a consumer never has to know which one was
 * running.  What it DOES have to know is that the two do not cover the same set
 * of signals and do not arrive at the same rate, and both of those facts are
 * carried per-signal rather than globally:
 *
 *   - Sniffed IDs arrive at 50-100 Hz; a full OBD-II poll cycle takes ~500 ms.
 *     One shared timestamp is either too tight for one source or too loose for
 *     the other, so freshness is per signal.
 *   - Steering effort and yaw rate exist ONLY when sniffing; speed, rpm and
 *     gear exist in both.  A single "source" byte would leave the Jetson unable
 *     to tell "the sensor went quiet" from "this mode cannot supply it".
 *
 * @see peripherals/mkr_zero/lib/CANSniffFunctions.h for the decoders.
 */

#include <Arduino.h>
#include <math.h>

/** @brief Which source last wrote a given field. */
enum class VehSource : uint8_t {
    NONE      = 0, ///< Never written, or structurally unavailable in this mode.
    CAN_SNIFF = 1, ///< Decoded from a broadcast frame.
    OBD2      = 2  ///< Answered by an ECU to a Mode 01 request.
};

/**
 * @brief Gear selector position.
 *
 * Deliberately NOT collapsed into @c TelemetryPayload::gear, which is a float
 * carrying PID 0xA4's numeric "actual gear".  Different domains: a selector
 * position of "R" has no numeric value, and "4" from a ratio-derived gear is not
 * the same claim as the driver having selected D.
 */
enum class VehGear : uint8_t {
    UNKNOWN = 0,
    PARK,
    REVERSE,
    NEUTRAL,
    DRIVE,
    LOW,
    SPORT
};

/** @brief Sentinel for @c steerMotorTorque: no reading, as distinct from zero. */
#define VEH_TORQUE_INVALID   0xFFFFu

/**
 * @brief Sentinel for @c wheelRaw[]: no reading.
 *
 * Zero is what the car genuinely reports at rest, so it cannot double as
 * "unknown".
 */
#define VEH_WHEEL_INVALID    0xFFFFu

/** @brief Number of wheel-speed channels, in @c VehWheel order. */
#define VEH_WHEEL_COUNT      4

/** @brief Index into @c VehicleSignals::wheelRaw. */
enum VehWheel : uint8_t {
    VEH_WHEEL_FL = 0,
    VEH_WHEEL_FR,
    VEH_WHEEL_RL,
    VEH_WHEEL_RR
};

/**
 * @brief Everything the vehicle bus can tell us, however it was obtained.
 *
 * Float fields are @c NAN when unavailable, matching @c OBD2Data and
 * @c GPSData so every consumer already handles the case.  Integer fields use
 * the explicit sentinels above, because their natural "empty" value (zero) is a
 * real measurement.
 */
struct VehicleSignals {
    // ---- available from BOTH sources ----
    float    speedKmh;          ///< Road speed (km/h). NAN when unavailable.
    float    rpm;               ///< Engine speed (rpm). NAN when unavailable.
    VehGear  gear;              ///< Selector position.

    // ---- CAN sniffing only ----
    /**
     * EPS motor assist torque, raw counts 0-511, @c VEH_TORQUE_INVALID if none.
     *
     * NOT a steering angle, and not a substitute for one: it is an unsigned
     * MAGNITUDE that rises for either direction of turn and returns to exactly
     * zero when the driver stops applying effort.  This vehicle publishes no
     * angle anywhere on the bus (see the plan's §1), so there is no field here
     * that would ever be filled with one.
     */
    uint16_t steerMotorTorque;

    /**
     * Yaw rate in centi-degrees/s, left positive, derived from the REAR pair.
     *
     * @c INT16_MIN when unavailable, which is a distinct state from zero.  The
     * wheel-speed sensors report exactly zero below roughly 3 km/h, so below
     * that the yaw rate is unknown - and "going straight" is the most dangerous
     * possible lie at parking speeds, where a hard turn is most likely.
     */
    int16_t  yawRateCdps;

    /** Per-wheel speeds in raw 0.01 km/h counts, @c VEH_WHEEL_INVALID if none. */
    uint16_t wheelRaw[VEH_WHEEL_COUNT];

    uint8_t  pedalGas;          ///< Accelerator, raw 0-255; x0.5 = percent.
    bool     brakePressed;      ///< Brake pedal applied.
    bool     brakeSwitch;       ///< Redundant brake switch channel.

    /**
     * Indicator ACTIVE, not indicator lamp lit.  See below - the difference is
     * the whole reason these are computed here.
     *
     * Two independent bools rather than one three-state direction, so a vehicle
     * that does drive both lamps from this message can say so. This one does
     * NOT — see @c hazard, which had to become its own signal.
     *
     * These are what separate a deliberate lane change from a lane departure,
     * which is why lane keeping wants them.  Two honest limits on that: the
     * vehicle reports the LAMP, not intent, so a stalk the steering column has
     * self-cancelled stops reporting mid-manoeuvre on a long sweeping bend;
     * and a driver who changes lane without indicating produces exactly the
     * same signal as a genuine departure, which no bus can fix.
     *
     * ── WHY THESE ARE HELD ───────────────────────────────────────────────────
     * The lamp BLINKS, at roughly 1.5 Hz.  Telemetry leaves this board at
     * ~4 Hz, so forwarding the instantaneous bit would alias it: the host would
     * see the indicator dark for about half the samples of a manoeuvre it was
     * lit throughout, and no amount of care downstream can recover the truth
     * from those samples.  So the flash is held here, at the 24 Hz the frame
     * actually arrives at - the same reason the yaw rate is derived on this
     * board rather than on the Jetson.
     */
    bool     turnLeft;
    bool     turnRight;

    /**
     * Hazard lights. A SEPARATE signal, not "turnLeft && turnRight".
     *
     * That derivation was assumed and the vehicle disproved it: with the hazards
     * on, both turn bits of 0x294 stay clear. The indicator message reports the
     * STALK, and the hazard switch bypasses the stalk, so the two are genuinely
     * independent here. Deriving one from the other would publish hazards as
     * "not indicating" — the most reassuring possible reading for a car stopped
     * in a live lane, which is exactly when it must not be wrong.
     *
     * Its own timestamp because the map may put it on a different ID entirely;
     * on this platform it has not been located yet. Held like the indicators,
     * since it flashes on the same cadence.
     */
    bool     hazard;

    // ---- per-signal freshness (millis(), 0 = never) ----
    //
    // @c wheelMs and @c brakeMs exist because the couplings they replace stopped
    // being true.  @c wheelRaw was expired off @c speedMs on the grounds that
    // the two "arrive in the same frame" — which held only while speed came from
    // the wheel message, and stopped the moment 0x158 became the preferred
    // source.  The brake bits were expired off @c pedalMs for the same reason.
    // Once the signal map is loaded from a card, ANY two of these can be
    // different IDs at different rates, so no field may ride on another's clock.
    uint32_t speedMs;
    uint32_t rpmMs;
    uint32_t gearMs;
    uint32_t steerMs;
    uint32_t yawMs;
    uint32_t pedalMs;
    uint32_t wheelMs;
    uint32_t brakeMs;
    uint32_t turnMs;
    uint32_t hazardMs;

    // ---- per-signal provenance ----
    VehSource speedSrc;
    VehSource rpmSrc;
    VehSource gearSrc;
    VehSource steerSrc;
    VehSource yawSrc;
    VehSource pedalSrc;
    VehSource wheelSrc;
    VehSource brakeSrc;
    VehSource turnSrc;
    VehSource hazardSrc;
};

/** @brief Resets every field to its "unavailable" value. */
inline void initVehicleSignals(VehicleSignals &v)
{
    v.speedKmh         = NAN;
    v.rpm              = NAN;
    v.gear             = VehGear::UNKNOWN;
    v.steerMotorTorque = VEH_TORQUE_INVALID;
    v.yawRateCdps      = INT16_MIN;
    for (uint8_t i = 0; i < VEH_WHEEL_COUNT; ++i) v.wheelRaw[i] = VEH_WHEEL_INVALID;
    v.pedalGas         = 0;
    v.brakePressed     = false;
    v.brakeSwitch      = false;
    v.turnLeft         = false;
    v.turnRight        = false;
    v.hazard           = false;

    v.speedMs = v.rpmMs = v.gearMs = v.steerMs =
    v.yawMs   = v.pedalMs = v.wheelMs = v.brakeMs = v.turnMs = v.hazardMs = 0;

    v.speedSrc = v.rpmSrc = v.gearSrc = v.steerSrc =
    v.yawSrc   = v.pedalSrc = v.wheelSrc = v.brakeSrc =
    v.turnSrc  = v.hazardSrc = VehSource::NONE;
}

/**
 * @brief Expires any signal older than its source's freshness window.
 *
 * The two sources are held to different clocks on purpose: a sniffed ID that
 * has not appeared for 200 ms is genuinely gone, while an OBD-II field is
 * expected to be up to a full poll cycle old and expiring it that fast would
 * blank the payload permanently.  Passing one window for both is how a working
 * OBD-II mode gets mistaken for a broken one.
 *
 * @param[in,out] v         Signals to expire in place.
 * @param[in]     nowMs     Current @c millis().
 * @param[in]     sniffMs   Age limit for @c CAN_SNIFF fields.
 * @param[in]     obd2Ms    Age limit for @c OBD2 fields.
 */
void expireVehicleSignals(VehicleSignals &v, uint32_t nowMs,
                          uint32_t sniffMs, uint32_t obd2Ms);

/**
 * @brief Packs the six provenance fields into the wire's @c sigSource byte.
 *
 * Two bits each for speed, rpm, gear and steer - the four the Jetson overlay
 * actually branches on.  Yaw and pedal provenance are omitted rather than
 * squeezed in: both are CAN-only, so their source is fully determined by
 * @c canMode and a bit spent on them would carry no information.
 */
uint8_t packVehSourceByte(const VehicleSignals &v);

#endif // VEHICLE_SIGNALS_H
