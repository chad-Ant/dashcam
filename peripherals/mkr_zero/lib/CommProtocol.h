#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H 1

/**
 * @file CommProtocol.h
 * @brief Portable UART wire contract shared by the MKR Zero master and the
 *        ESP32-C3 receiver (ESP_Sentinel/lib/commLink/CommProtocol.h).
 *
 * Keep this file byte-identical on both sides. It depends only on Arduino /
 * string / stdint, so it drags in no board-specific or project libraries.
 *
 * Frame: SOF(0x7E) | VER | TYPE(1) | LEN(1) | PAYLOAD(LEN) | CRC16_LE(2)
 * VER is COMM_VERSION, currently 0x06; the payload is 160 bytes.
 * CRC-16/CCITT-FALSE over VER..last payload byte, transmitted low byte first.
 * Both MCUs are little-endian IEEE-754, so a packed struct copies verbatim.
 */

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

#define COMM_SOF             0x7Eu  ///< Start-of-frame delimiter.
/**
 * Protocol version byte.
 *
 * 0x02 added TelemetryPayload::accel (79 -> 83 bytes).  Bumped rather than
 * relying on the length check alone: a mismatched pair now fails at the version
 * byte and resyncs immediately, instead of decoding a frame that looks almost
 * right and silently discarding it on size.
 *
 * 0x03 added the nine-axis IMU block plus die temperature (83 -> 123 bytes).
 *
 * 0x04 added the windowed inertial peaks (123 -> 131 bytes), once the LSM6DSOX
 * FIFO began being drained in full rather than the output registers sampled at
 * 20 Hz, plus the IMU_DATA_GAP and IMU_LOWPOWER flags that qualify them.
 */
#define COMM_VERSION         0x06u
#define COMM_MAX_PAYLOAD     255u   ///< Largest payload (LEN is one byte).
#define COMM_FRAME_OVERHEAD  6u     ///< SOF+VER+TYPE+LEN + CRC16(2).
#define COMM_MAX_FRAME       (COMM_FRAME_OVERHEAD + COMM_MAX_PAYLOAD)
#define COMM_RX_TIMEOUT_MS   50UL   ///< Abort a partial frame after this inter-byte gap (ms).

/// Telemetry @c flags bits.
/**
 * @c TelemetryPayload::vehFlags bits.
 *
 * Named because a lane-keeping consumer reading bit 2 as bit 3 is a silent
 * left/right swap, and a magic number is how that happens.
 */
#define COMM_VEH_FLAG_BRAKE_PRESSED 0x01u
#define COMM_VEH_FLAG_BRAKE_SWITCH  0x02u
#define COMM_VEH_FLAG_TURN_LEFT     0x04u
#define COMM_VEH_FLAG_TURN_RIGHT    0x08u
/**
 * Both indicator bits. NOT a hazards predicate — see COMM_VEH_FLAG_HAZARD.
 *
 * Do not test hazards as `flags & TURN_MASK`: that is true for a single
 * indicator too. And do not test them as `== TURN_MASK` either, which is what
 * this comment used to recommend. Measured on the vehicle: switching the
 * hazards on leaves BOTH turn bits clear, because the indicator message reports
 * the stalk and the hazard switch bypasses it. Hazards have their own bit.
 */
#define COMM_VEH_FLAG_TURN_MASK     (COMM_VEH_FLAG_TURN_LEFT | COMM_VEH_FLAG_TURN_RIGHT)

/// Hazard lights, an independent signal — NOT both indicators at once.
#define COMM_VEH_FLAG_HAZARD        0x80u

/// Validity bits. Zero in these means "no data", NOT "released / not indicating".
///
/// Without them an absent or stale signal serialises identically to the safe-
/// looking state: brake released, indicators off. For lane keeping that inverts
/// the verdict - every lane change in OBD2 mode, where the indicator bits cannot
/// be populated at all, would read as an UNSIGNALLED departure. A consumer must
/// check these before believing a cleared bit.
#define COMM_VEH_FLAG_BRAKE_VALID   0x10u
#define COMM_VEH_FLAG_TURN_VALID    0x20u
#define COMM_VEH_FLAG_PEDAL_VALID   0x40u

#define COMM_FLAG_OBD2_VALID 0x01u  ///< OBD2 data is live (a reading arrived within the master's freshness window).
#define COMM_FLAG_GPS_FIX    0x02u  ///< GPS reported a valid fix.
#define COMM_FLAG_TIME_VALID 0x04u  ///< UTC date and time are valid.
/**
 * The IMU initialised and has not been declared lost.
 *
 * Distinct from the per-axis NAN sentinels, and the distinction is the point:
 * NAN with this flag CLEAR means no IMU is fitted, NAN with it SET means the
 * IMU is fitted but that channel is stale.  Same reasoning as
 * @c COMM_FLAG_GPS_FIX — a receiver that cannot tell absent hardware from a
 * silent sensor will misdiagnose both.
 */
#define COMM_FLAG_IMU_PRESENT 0x08u
/**
 * The GNSS receiver is configured and still emitting PVT packets.
 *
 * The counterpart of @c COMM_FLAG_IMU_PRESENT, and added for the same reason:
 * @c COMM_FLAG_GPS_FIX alone cannot distinguish a receiver that is absent or
 * failed its bring-up from a healthy one that simply has no fix yet.  Both show
 * NAN coordinates with GPS_FIX clear, and they call for opposite responses —
 * one is a fault to report, the other is a normal cold start.
 */
#define COMM_FLAG_GPS_PRESENT 0x10u
/**
 * The IMU is running, but NOT in the configuration that was asked for.
 *
 * v0x06 REDEFINED THIS FLAG, and a host reading it must know which. Through
 * v0x05 the master carried a two-chip breakout (LSM6DSOX + LIS3MDL) and this
 * meant "only one of the two devices is answering" — a power fault, because an
 * unpowered I2C slave keeps acknowledging on parasitic current through the bus
 * pull-ups while its neighbour dies.
 *
 * The BNO055 that replaced it is one chip, so that state cannot occur. The flag
 * now means the part came up on a fallback: the internal oscillator after the
 * external crystal was refused, or a mode other than the one requested. The
 * data is good; it is not the data that was configured.
 *
 * Read with @c COMM_VERSION, not on its own.
 */
#define COMM_FLAG_IMU_DEGRADED 0x20u
/**
 * There is a HOLE in the inertial record for this window.
 *
 * Raised for either of two causes, because the consequence is the same: the
 * sensor overwrote unread samples (a true FIFO overrun), or the master
 * discarded a backlog that was already older than its freshness window and
 * would otherwise have been transmitted stamped as current.
 *
 * So @c imuAccelPeak and @c imuGyroPeak are peaks over LESS than the interval
 * they claim.  A consumer doing incident detection must not read a quiet window
 * carrying this flag as evidence that nothing happened.
 *
 * Named for the consequence, not one cause: it was briefly called
 * IMU_FIFO_OVERRUN while already being raised by both, so anyone chasing an
 * overrun found a counter that had never incremented.
 */
#define COMM_FLAG_IMU_DATA_GAP 0x40u
/**
 * The IMU is in a coarser sampling regime than normal.
 *
 * RESERVED AND ALWAYS CLEAR as of v0x06. It described the LSM6DSOX's low-power
 * mode, selected from ignition state. The BNO055 has an equivalent, and it is
 * not enabled: the datasheet withdraws the High-G interrupt in low power, which
 * would remove the hardware impact backstop exactly when a car-park bump is the
 * thing being watched for. Kept so the bit position does not shift and so the
 * flag is available when parked operation is designed properly.
 */
#define COMM_FLAG_IMU_LOWPOWER 0x80u
/**
 * The IMU latched a High-G threshold crossing during this window.
 *
 * THE ONLY FIELD ON THIS FRAME THAT SURVIVES A STALLED MASTER. Every other
 * inertial value is the product of a poll, and the BNO055 has no FIFO — so a
 * poll that does not happen produces nothing, permanently. This is latched in
 * the sensor's own hardware and held until read, so an impact during a half
 * second of blocked loop is still reported afterwards, where the peak that
 * would have described it is simply gone.
 *
 * Held by the master for ~500 ms so it cannot fall between two frames.
 *
 * A consumer doing incident detection should treat this as a stronger trigger
 * than @c imuAccelPeak, not a weaker one: the peak is only ever as good as the
 * polls behind it.
 */
#define COMM_FLAG_IMU_HIGH_G 0x0100u
/**
 * The accelerometer hit its range limit, so the peaks are FLOORS.
 *
 * The BNO055 locks the accelerometer at ±4 g in every fusion mode, so anything
 * past 39.2 m/s² is clipped: a genuine 20 g collision reads as 4 g. Both
 * @c imuAccelPeak and @c imuLinAccelPeak are affected — linear acceleration is
 * derived from the same clipped measurement, and bench capture during clipping
 * showed it reaching 57 m/s², above the raw rail itself, because subtracting an
 * estimated gravity vector from a saturated reading is not a physical quantity.
 *
 * Grading severity from a saturated peak understates an impact several-fold.
 */
#define COMM_FLAG_IMU_SATURATED 0x0200u
/**
 * A CAN signal map is loaded and in use. @c canMapChecksum identifies which.
 *
 * Without this, a consumer cannot tell a vehicle whose map decoded nothing from
 * one running with no map at all — both publish the same sentinels.
 */
#define COMM_FLAG_CANMAP_LOADED 0x0400u
/**
 * The hardware High-G backstop is ARMED.
 *
 * The counterpart of @c COMM_FLAG_IMU_HIGH_G, and useless without it. That flag
 * says an impact was latched; this says the latch was capable of latching one.
 * Clear means an impact landing inside a stalled master goes unrecorded — the
 * single case the backstop exists for — and NOTHING ELSE ON THIS FRAME WOULD SAY
 * SO. The peaks look normal, every valid flag stays set, and the absence is
 * indistinguishable from a quiet drive.
 *
 * It goes clear when the interrupt could not be configured at bring-up, or when
 * the master found the latch no longer clearing. Both leave the sample data
 * fully usable, which is why neither is reported as a fault: the sensor is fine
 * and the safety net is not.
 *
 * Added in v0x06 without a version bump — a flag bit inside an existing
 * @c uint16_t changes no offset and no payload size, so a consumer built against
 * the earlier v0x06 header ignores it exactly as it ignores any bit it does not
 * know. A host that WANTS it should test it, not assume it.
 */
#define COMM_FLAG_IMU_HIGHG_ARMED 0x0800u
/**
 * The IMU is in a FUSION mode (IMUPLUS); clear means the raw mode (AMG).
 *
 * The two publish different fields and mean different things by the same ones,
 * so a consumer that does not know which it is holding cannot read the frame
 * correctly. In fusion the magnetometer is off and @c imuMagX/Y/Z are NAN while
 * @c imuLinAccelPeak and @c imuYawRelDeg are live; in raw there is no fusion, so
 * those two are NAN and the magnetometer is populated. @c imuAccelPeak saturates
 * at 4 g in fusion and 16 g in raw — the same number, two different rails.
 *
 * Inferring the mode from which fields happen to be NAN was the only option
 * before this bit existed, and it is not the same thing: a stale channel and a
 * mode that does not produce that channel look identical.
 *
 * It also closes the loop on @c CMD_SET_IMU_MODE, which otherwise asks for a
 * change the host has no way to observe.
 */
#define COMM_FLAG_IMU_FUSION_MODE 0x1000u
/**
 * NOTE: @c TelemetryPayload::flags became @c uint16_t in v0x06 and 0x1000 is in
 * use. Seven bits remain. The next widening is another payload size change and
 * therefore another COMM_VERSION bump — the v0x05 note said the same thing about
 * this one, and it was accurate.
 */

/// @c CMD_SET_IMU_MODE payload values.
///
/// 1-based, so 0 stays available as "nothing pending" in the master's latch —
/// the same convention @c CMD_SET_CAN_MODE uses, and for the same reason.
#define COMM_IMU_MODE_FUSION 1u  ///< IMUPLUS: on-chip fusion, magnetometer off.
#define COMM_IMU_MODE_RAW    2u  ///< AMG: raw accel/mag/gyro, no fusion, +/-16 g.

/** Message / command identifiers. High bit set = master (MKR) -> slave (C3). */
enum CommMsgType : uint8_t {
    CMD_GET_ONCE     = 0x01, ///< C3 -> MKR: request a single telemetry frame.
    CMD_START_STREAM = 0x10, ///< C3 -> MKR: begin the 10 Hz telemetry push.
    CMD_STOP_STREAM  = 0x11, ///< C3 -> MKR: stop streaming.
    CMD_PING         = 0x20, ///< C3 -> MKR: link check.
    /**
     * C3 -> MKR: payload uint8 CanMode (1=discover 2=sniff 3=obd2).
     *
     * The FIRST payload-bearing command on this hop. Everything before it was
     * zero-payload, and the receiver exploited that with a blanket
     * "len != 0 -> NACK" - so adding this without the helpers below would have
     * had the MKR reject its own new command as malformed.
     */
    CMD_SET_CAN_MODE = 0x13,
    /**
     * C3 -> MKR: replace the hardware receive filters. 13-byte payload.
     *
     * Layout: uint8 count, then 6 x uint16 little-endian CAN IDs. Entries past
     * @c count are ignored but must still be transmitted — a fixed-length
     * payload is what lets @c commandPayloadLen() stay a pure function of TYPE,
     * and the alternative was a variable length the receiver would have to trust
     * before it had validated anything.
     *
     * A count of 0 clears every filter, which on an MCP2515 means ACCEPT ALL
     * rather than accept none. That is worth stating because it is the opposite
     * of what "no filters" sounds like, and it is the correct behaviour for
     * discovery: the map's filter set is an optimisation for a known vehicle,
     * not a security boundary.
     *
     * Only meaningful in sniff mode. The controller must re-enter configuration
     * to change filter registers, so this costs a brief receive gap — the master
     * reports it as a gap rather than hiding it.
     */
    CMD_SET_CAN_FILTER = 0x14,
    /**
     * C3 -> MKR: select the IMU's operating mode. 1-byte payload.
     *
     * @c COMM_IMU_MODE_FUSION or @c COMM_IMU_MODE_RAW. The two are DIFFERENT
     * MEASUREMENTS, not a quality setting — see the mode constants — so this is
     * a deliberate session-level choice, never a per-event one.
     *
     * ⚠️ EXPENSIVE AND BLINDING. Applying it restarts the sensor's whole
     * bring-up: the IMU publishes nothing for about 700 ms and the trailing peak
     * window is discarded. A crash pulse lasts 10-50 ms, so a mode switch
     * triggered ON an impact would be over long after the event it was reacting
     * to, having thrown away the only record of it. The master rate-limits this
     * for that reason, and a request arriving too soon after the last one is
     * NACKed rather than queued.
     *
     * Not restored automatically after a master reset: the MKR comes back in its
     * compiled-in default, and a host that needs the other mode must ask again.
     * The mode is reported on every frame in @c COMM_FLAG_IMU_FUSION_MODE, so a
     * host can tell without keeping its own state.
     */
    CMD_SET_IMU_MODE = 0x15,
    MSG_TELEMETRY    = 0x81, ///< MKR -> C3: telemetry payload.
    MSG_PONG         = 0xA0, ///< MKR -> C3: ping acknowledgement.
    MSG_NACK         = 0xEE, ///< MKR -> C3: malformed or unknown command.
};

/**
 * @brief Largest payload any C3 -> MKR command carries.
 *
 * Exists so a command sender can size its frame buffer without either
 * hard-coding zero (correct only while every command was zero-payload) or
 * reserving the full COMM_MAX_PAYLOAD, which is sized for telemetry in the
 * other direction and would put 148 bytes of stack in the sender for a
 * one-byte command.
 */
#define COMM_MAX_CMD_PAYLOAD 13u

/// Filter slots @c CMD_SET_CAN_FILTER carries. Matches the MCP2515's six.
#define COMM_CAN_FILTER_SLOTS 6u
/// uint8 count + 6 x uint16.
#define COMM_SET_CAN_FILTER_LEN 13u

/** @brief True for TYPEs the C3 may send to the MKR. */
inline bool isCommand(uint8_t type)
{
    switch (type) {
    case CMD_GET_ONCE:
    case CMD_START_STREAM:
    case CMD_STOP_STREAM:
    case CMD_SET_CAN_MODE:
    case CMD_SET_CAN_FILTER:
    case CMD_SET_IMU_MODE:
    case CMD_PING:
        return true;
    default:
        return false;
    }
}

/** @brief Expected payload length for a command TYPE; 0 for zero-payload commands. */
inline uint8_t commandPayloadLen(uint8_t type)
{
    switch (type) {
    case CMD_SET_CAN_MODE:   return 1;
    case CMD_SET_IMU_MODE:   return 1;
    case CMD_SET_CAN_FILTER: return COMM_SET_CAN_FILTER_LEN;
    default:                 return 0;
    }
}

/** Result codes for protocol operations. */
enum class CommReturnStatus : int8_t {
    OK           =  0, ///< Generic success.
    FRAME_READY  =  1, ///< A complete, CRC-valid frame was decoded.
    NO_DATA      =  2, ///< No complete frame available yet (non-blocking).
    NOK_NULL     = -1, ///< NULL buffer argument.
    NOK_OVERFLOW = -2, ///< Decoded payload longer than the caller's buffer.
    NOK_CRC      = -3, ///< CRC mismatch; the frame was discarded and resynced.
    NOK_BUSY     = -4, ///< TX buffer lacked room; frame not sent (non-blocking).
};

/**
 * @brief Combined OBD2 + GPS telemetry snapshot (packed, little-endian).
 *
 * Missing float values are @c NAN (matching @c OBD2Data / @c GPSData), so a
 * receiver can distinguish "not supplied" from a real zero reading.
 */
struct __attribute__((packed)) TelemetryPayload {
    uint32_t masterMillis; ///< Master uptime (ms) for staleness detection.
    // ---- OBD2 ----
    float speed;           ///< Vehicle speed (km/h), as reported (whole km/h).
    /**
     * Longitudinal acceleration (m/s², +ve = accelerating).
     *
     * DERIVED, not a PID: @c AccelerationEstimator smooths the quantised speed
     * and differentiates it (see SignalProcessingFunctions.h).  Differentiating
     * @c speed on the receiving side instead does not work — the 10 Hz stream
     * repeats each 1 km/h reading many times, so the apparent derivative is a
     * train of spikes rather than a signal.
     *
     * NAN while the filter is warming up or the ECU has not supplied speed.
     */
    float accel;
    float rpm;             ///< Engine speed (rpm).
    float coolantTemp;     ///< Coolant temperature (°C).
    float fuelLevel;       ///< Fuel tank level (%).
    float fuelRate;        ///< Engine fuel rate (L/h).
    float throttle;        ///< Throttle position (%).
    float engineLoad;      ///< Calculated engine load (%).
    float airPressure;     ///< Barometric pressure (kPa).
    float gear;            ///< Commanded gear (0 = P/N, 1–8).
    float gearRatio;       ///< Transmission gear ratio.
    float odo;             ///< Odometer (km).
    // ---- GPS ----
    float latitude;        ///< WGS-84 latitude (deg).
    float longitude;       ///< WGS-84 longitude (deg).
    float altitude;        ///< Altitude above MSL (m).
    float gpsSpeedKmh;     ///< GPS ground speed (km/h).
    float heading;         ///< Course over ground (deg).
    uint8_t satellites;    ///< Satellites in the solution.
    uint8_t fixType;       ///< u-blox fix type (0=none,2=2D,3=3D).
    uint8_t fixValid;      ///< 1 when the fix is valid.
    // ---- UTC ----
    uint16_t year;         ///< Four-digit UTC year.
    uint8_t month;         ///< 1–12.
    uint8_t day;           ///< 1–31.
    uint8_t hour;          ///< 0–23.
    uint8_t minute;        ///< 0–59.
    uint8_t second;        ///< 0–60.
    /**
     * ---- IMU (BNO055, sensor frame) ----
     *
     * NOT to be confused with @c accel above.  That is a scalar LONGITUDINAL
     * acceleration derived from the ECU speed signal; these are the raw
     * three-axis measurements from the inertial sensor, in the sensor's own
     * frame as silkscreened on the breakout.  Mapping them to the vehicle frame
     * depends on how the board is bolted in and is deliberately left to the
     * consumer — the master does not guess a mounting orientation.
     *
     * Each is NAN when its channel is stale or absent; see
     * @c COMM_FLAG_IMU_PRESENT for telling "no IMU fitted" from "IMU fitted,
     * channel quiet".
     */
    float imuAccelX;       ///< Sensor-frame X acceleration (m/s², gravity included).
    float imuAccelY;       ///< Sensor-frame Y acceleration (m/s²).
    float imuAccelZ;       ///< Sensor-frame Z acceleration (m/s²).
    float imuGyroX;        ///< Angular rate about sensor X (deg/s).
    float imuGyroY;        ///< Angular rate about sensor Y (deg/s).
    float imuGyroZ;        ///< Angular rate about sensor Z (deg/s).
    /**
     * Magnetic flux density along each sensor axis (µT), uncalibrated.
     *
     * NAN IN NORMAL OPERATION as of v0x06, and that is deliberate rather than a
     * missing sensor. The master runs the BNO055 in its magnetometer-free fusion
     * mode on purpose: the magnetometer feeds the orientation quaternion that
     * linear acceleration is derived from, so magnetic disturbance would
     * contaminate the one channel incident detection depends on — and a car is
     * close to a worst case, with distortion that moves with electrical load.
     * Populated only in the raw (AMG) operating mode.
     */
    float imuMagX;
    float imuMagY;
    float imuMagZ;
    float imuTempC;        ///< Sensor die temperature (°C) — board, not cabin.
    /**
     * Peak |a| and |ω| over the master's trailing window, not over this instant.
     *
     * The axes above are ONE sample — the most recent of the ten the sensor
     * produced since the last frame at 10 Hz.  A pothole or kerb strike is a
     * 10-50 ms impulse, so the sample that catches it is usually not the sample
     * that gets transmitted.  These are folded on the master from every sample
     * it polled.
     *
     * WHAT BACKS THEM CHANGED IN v0x06. Through v0x05 the sensor batched into a
     * 512-word FIFO and the master drained it in full, so every converted sample
     * reached the peak however slowly the host polled. The BNO055 has no FIFO,
     * so the master polls at the sensor's own 100 Hz output rate instead — which
     * captures the same impulses, but means a poll the master fails to make is
     * data that no longer exists anywhere. @c COMM_FLAG_IMU_DATA_GAP therefore
     * carries more weight than it did, and @c COMM_FLAG_IMU_HIGH_G is the
     * hardware backstop for exactly that case.
     *
     * Magnitudes, so they do not depend on how the breakout is bolted in.
     * Gravity is included in @c imuAccelPeak: a stationary vehicle reads about
     * 9.81, not 0. @c imuLinAccelPeak below is the same quantity with gravity
     * already removed.
     *
     * NAN when the matching channel is stale or absent.  Check
     * @c COMM_FLAG_IMU_DATA_GAP before trusting a window, and
     * @c COMM_FLAG_IMU_SATURATED before grading severity from one.
     */
    float imuAccelPeak;    ///< Peak |a| over the window (m/s², gravity included).
    float imuGyroPeak;     ///< Peak |ω| over the window (deg/s).
    /**
     * ---- IMU fusion (v0x06) ----
     *
     * The BNO055 fuses on-chip, so these are computed by the sensor rather than
     * derived here. They did not exist before v0x06 because the previous part
     * could not produce them at all.
     */
    /**
     * Peak |linear a| over the window (m/s²), GRAVITY REMOVED.
     *
     * The field incident detection actually wants. @c imuAccelPeak includes
     * gravity, so a stationary vehicle reads 9.81 and every threshold has to
     * carry that offset around; this reads ~0 at rest, so a value of 1 g here
     * means the vehicle accelerated at 1 g.
     *
     * NAN in the raw (AMG) operating mode, which produces no fusion output.
     * Check @c COMM_FLAG_IMU_SATURATED before grading severity from it.
     */
    float imuLinAccelPeak;
    /**
     * Rotation about the vertical (deg), RELATIVE AND DRIFTING.
     *
     * NOT a heading, and named so it cannot be mistaken for one. The master runs
     * the sensor with its magnetometer switched off — deliberately, because a
     * magnetometer feeds the orientation quaternion that linear acceleration is
     * derived from, and a car is a steel shell full of motors and a harness
     * carrying tens of amps. So this has no north reference and drifts.
     *
     * What it IS good for is short-window rotation at 100 Hz: how far the
     * vehicle turned during a two-second event, where drift is negligible and
     * neither the 1 Hz GNSS course nor the wheel-speed pair — dead below about
     * 3 km/h — can answer. For an absolute bearing use @c heading or the
     * rear-wheel differential in @c yawRateCdps.
     */
    float imuYawRelDeg;
    /**
     * Sensor self-assessed calibration, 2 bits each:
     * bits 1:0 mag, 3:2 accel, 5:4 gyro, 7:6 system. 0 = uncalibrated, 3 = full.
     *
     * MAG AND SYSTEM ARE PERMANENTLY 0 in fusion mode and that is correct, not a
     * fault — the magnetometer is off, and the system figure cannot rise without
     * it. Judge readiness on the gyroscope field.
     *
     * The ACCELEROMETER field is reported but should not be used as a quality
     * gate. Bench measurement: it fell to 0 after 1400 polls and stayed there
     * for 9000 more while the sensor's gravity vector held 9.79-9.81 m/s²
     * throughout — so it was not measuring trustworthiness. The master gates
     * every sample on that gravity magnitude instead, which is a direct physical
     * test rather than the part's opinion of itself.
     */
    uint8_t imuCalib;
    /**
     * ---- CAN signal map identity (v0x06) ----
     *
     * Which decode table produced the vehicle fields below.
     *
     * The master loads a per-vehicle map from SD (canmap.<vehicle>.txt), so the
     * MEANING of @c gearPos, @c vehFlags and the rest depends on a file that can
     * be edited between drives without a firmware flash. A recording that does
     * not say which map decoded it cannot be re-interpreted later, and a wrong
     * map does not fail loudly — it produces plausible values from the wrong
     * bytes, which is the failure this field exists to make detectable.
     *
     * Zero when no map is loaded; check @c COMM_FLAG_CANMAP_LOADED, since zero
     * is also a legitimate checksum.
     */
    uint8_t canMapChecksum;
    /**
     * bit 0 hardware filters active, bit 1 filter set came from the map,
     * bits 2-7 reserved and zero.
     *
     * Filters matter to a consumer because they decide what CANNOT appear: an ID
     * absent from a filtered capture may be absent from the bus or merely
     * excluded by the controller, and only this says which.
     */
    uint8_t canMapFlags;
    // ---- status ----
    /**
     * COMM_FLAG_* bitfield.
     *
     * WIDENED FROM uint8 IN v0x06. The v0x05 header noted that 0x80 was the last
     * bit and that the next flag would force this change; three arrived at once
     * (High-G, saturation, map loaded), so it happened here.
     */
    uint16_t flags;
    /**
     * ---- vehicle bus (v0x05) ----
     *
     * Filled by CAN sniffing when it is running and by OBD-II polling when it
     * is not, so a consumer reads the same fields either way.  Which source
     * actually supplied each one is in @c sigSource, and it matters: the two do
     * not cover the same set.  @c steerMotorTorque, @c yawRateCdps and
     * @c wheelRaw exist ONLY while sniffing, and are at their sentinels in OBD2
     * mode because that mode structurally cannot supply them - which is a
     * different statement from "the sensor went quiet".
     */
    uint8_t canMode;       ///< CanMode: 0=off 1=discover 2=sniff 3=obd2.
    uint8_t sigSource;     ///< VehSource, 2 bits each: speed|rpm|gear|steer.
    uint8_t gearPos;       ///< VehGear selector position (0 = unknown).
    /**
     * bit 0 brakePressed, bit 1 brakeSwitch,
     * bit 2 turnLeft, bit 3 turnRight,
     * bit 4 brakeValid, bit 5 turnValid, bit 6 pedalValid, bit 7 hazard.
     *
     * Two brake bits because the car publishes two: a switch channel and a
     * pressed channel, in different bytes of 0x17C.  They normally agree, and a
     * disagreement is the interesting case, so collapsing them would discard
     * the only evidence that one has failed.
     *
     * The turn bits are indicator ACTIVE, not indicator lamp lit.  The lamp
     * blinks at ~1.5 Hz and this payload leaves at ~4 Hz, so the raw bit would
     * alias into something dark for half the samples of a manoeuvre it was lit
     * throughout; the sender holds each flash for 900 ms instead.  Both set at
     * once is hazard lights.  Reading them is what separates a deliberate lane
     * change from a lane departure.
     *
     * These bits were reserved-and-zero in earlier firmware, so a host built
     * against that contract reads them as "not indicating" rather than as
     * garbage - which is why adding them needed no version bump.
     */
    uint8_t vehFlags;
    uint8_t pedalGas;      ///< Accelerator, raw 0-255; x0.5 = percent.
    /**
     * EPS motor assist torque, raw 0-511. @c 0xFFFF when unavailable.
     *
     * NOT a steering angle.  It is an unsigned MAGNITUDE that rises for either
     * direction of turn and returns to exactly zero when the driver stops
     * applying effort.  This vehicle publishes no steering angle anywhere on
     * its bus, and SAE J1979 has no steering PID, so no angle field exists here
     * to be filled in later.  For heading change use @c yawRateCdps.
     */
    uint16_t steerMotorTorque;
    /**
     * Yaw rate, centi-degrees/s, LEFT POSITIVE. @c INT16_MIN when unavailable.
     *
     * Derived on the master from the rear wheel-speed pair at 50 Hz, because
     * this stream runs at ~10 Hz and the signal swings 30 deg/s inside a single
     * U-turn - reconstructing it here would be aliased beyond use.
     *
     * The sentinel is NOT zero.  Zero is the value for genuinely travelling
     * straight; the wheel-speed sensors report nothing below about 3 km/h, and
     * "going straight" is the most dangerous possible lie at parking speeds.
     *
     * Curvature and an effective steering angle are deliberately absent: both
     * are this, @c speed and one vehicle constant away, so the consumer can
     * derive them and revise the wheelbase without a firmware flash.
     */
    int16_t yawRateCdps;
    /**
     * Per-wheel speeds in raw 0.01 km/h counts, FL FR RL RR.
     * @c 0xFFFF per channel when unavailable.
     *
     * Carried as well as the scalar @c speed because per-wheel speed is what
     * reveals lockup, slip and ABS activity at the moment of an incident, and a
     * scalar can never reconstruct that afterwards.
     */
    uint16_t wheelRaw[4];
};

/// The wire contract depends on this exact size on both MCUs.
static_assert(sizeof(TelemetryPayload) == 160, "TelemetryPayload must be tightly packed to 160 bytes");
/**
 * The payload has to fit the frame's one-byte LEN field, and the bridge's
 * one-byte @c telemetryBytes self-check.  Worth stating now that the struct has
 * grown 79 -> 83 -> 123 -> 131 -> 148 -> 160: another addition the size of the IMU
 * block lands at 200, and the failure mode past 255 is a silently truncated length
 * rather than anything that looks like an error.
 */
static_assert(sizeof(TelemetryPayload) <= COMM_MAX_PAYLOAD,
              "TelemetryPayload no longer fits the frame's one-byte LEN field");

/** @brief One CRC-16/CCITT-FALSE step (poly 0x1021, init 0xFFFF). */
inline uint16_t crc16_update(uint16_t crc, uint8_t b) {
    crc ^= static_cast<uint16_t>(b) << 8;
    for (uint8_t i = 0; i < 8; ++i) {
        crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                             : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

/** @brief CRC-16/CCITT-FALSE over a byte range. */
inline uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) crc = crc16_update(crc, data[i]);
    return crc;
}

/**
 * @brief Serialises one frame into @p out.
 * @return Total frame length in bytes, or 0 on error (NULL/too small/bad args).
 */
size_t buildFrame(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out, size_t cap);

/** Parser phase for the incremental frame decoder. */
enum class RxPhase : uint8_t { WAIT_SOF, VER, TYPE, LEN, PAYLOAD, CRC_LO, CRC_HI };

/** @brief Incremental, non-blocking frame-decoder state (no heap). */
struct CommRxState {
    RxPhase  phase;                 ///< Current parser phase.
    uint8_t  type;                  ///< Type byte of the in-progress frame.
    uint8_t  len;                   ///< Declared payload length.
    uint16_t idx;                   ///< Payload bytes received so far.
    uint16_t crcRx;                 ///< CRC received from the wire.
    uint32_t lastByteMs;            ///< millis() of the last byte consumed by pollFrame() (partial-frame timeout).
    uint8_t  buf[COMM_MAX_PAYLOAD]; ///< Payload accumulator.
};

/** @brief Resets a decoder to WAIT_SOF. Call once before first use. */
void commRxInit(CommRxState &rx);

/**
 * @brief Feeds one byte into the decoder.
 * @param[out] payloadOut  Buffer to receive the payload when a frame completes.
 * @param[in]  payloadCap  Capacity of @p payloadOut.
 * @return @c FRAME_READY (type/payload/len filled), @c NO_DATA (need more bytes),
 *         @c NOK_CRC (frame failed CRC — parser resynced), or @c NOK_OVERFLOW
 *         (declared length exceeds @p payloadCap — frame dropped).
 */
CommReturnStatus commRxByte(CommRxState &rx, uint8_t b,
                            uint8_t &typeOut, uint8_t *payloadOut, uint8_t payloadCap, uint8_t &lenOut);

/**
 * @brief Drains up to @p maxBytes from @p in until one frame completes or the input
 *        empties. The byte cap keeps each call strictly bounded even under a flood of
 *        non-SOF noise that never forms a frame.
 * @return @c FRAME_READY, @c NO_DATA, @c NOK_CRC, or @c NOK_OVERFLOW.
 * @note Call in a loop to process multiple queued frames.
 */
CommReturnStatus pollFrame(CommRxState &rx, Stream &in,
                           uint8_t &typeOut, uint8_t *payloadOut, uint8_t payloadCap, uint8_t &lenOut,
                           size_t maxBytes = COMM_MAX_FRAME);

#endif
