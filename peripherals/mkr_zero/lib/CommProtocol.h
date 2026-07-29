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
 * Frame: SOF(0x7E) | VER(0x01) | TYPE(1) | LEN(1) | PAYLOAD(LEN) | CRC16_LE(2)
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
 */
#define COMM_VERSION         0x03u
#define COMM_MAX_PAYLOAD     255u   ///< Largest payload (LEN is one byte).
#define COMM_FRAME_OVERHEAD  6u     ///< SOF+VER+TYPE+LEN + CRC16(2).
#define COMM_MAX_FRAME       (COMM_FRAME_OVERHEAD + COMM_MAX_PAYLOAD)
#define COMM_RX_TIMEOUT_MS   50UL   ///< Abort a partial frame after this inter-byte gap (ms).

/// Telemetry @c flags bits.
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
 * The IMU is answering on only ONE of its two devices — a suspected power or
 * wiring fault, not merely reduced data.
 *
 * The LSM6DSOX and LIS3MDL share one PCB, one VIN and one ground, so there is no
 * benign way for exactly one of them to stop responding.  What produces this in
 * practice is a broken supply: an unpowered I2C slave still draws parasitic
 * power through the bus pull-ups and keeps acknowledging, so the lighter-draw
 * part looks alive while the other dies.  Observed on the bench by pulling VIN —
 * the magnetometer kept answering and the accelerometer did not.
 *
 * Treat as a connection warning for the whole module, not as a per-sensor
 * degradation: the readings that DO arrive came from a part running on parasitic
 * power and should not be trusted either.
 */
#define COMM_FLAG_IMU_DEGRADED 0x20u

/** Message / command identifiers. High bit set = master (MKR) -> slave (C3). */
enum CommMsgType : uint8_t {
    CMD_GET_ONCE     = 0x01, ///< C3 -> MKR: request a single telemetry frame.
    CMD_START_STREAM = 0x10, ///< C3 -> MKR: begin the 10 Hz telemetry push.
    CMD_STOP_STREAM  = 0x11, ///< C3 -> MKR: stop streaming.
    CMD_PING         = 0x20, ///< C3 -> MKR: link check.
    MSG_TELEMETRY    = 0x81, ///< MKR -> C3: telemetry payload.
    MSG_PONG         = 0xA0, ///< MKR -> C3: ping acknowledgement.
    MSG_NACK         = 0xEE, ///< MKR -> C3: malformed or unknown command.
};

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
     * ---- IMU (LSM6DSOX + LIS3MDL, sensor frame) ----
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
    float imuMagX;         ///< Magnetic flux density along sensor X (µT), uncalibrated.
    float imuMagY;         ///< Magnetic flux density along sensor Y (µT), uncalibrated.
    float imuMagZ;         ///< Magnetic flux density along sensor Z (µT), uncalibrated.
    float imuTempC;        ///< LSM6DSOX die temperature (°C) — board, not cabin.
    // ---- status ----
    uint8_t flags;         ///< COMM_FLAG_* bitfield.
};

/// The wire contract depends on this exact size on both MCUs.
static_assert(sizeof(TelemetryPayload) == 123, "TelemetryPayload must be tightly packed to 123 bytes");
/**
 * The payload has to fit the frame's one-byte LEN field, and the bridge's
 * one-byte @c telemetryBytes self-check.  Worth stating now that the struct has
 * grown 79 -> 83 -> 123: the next addition of this size lands at 163, and the
 * failure mode past 255 is a silently truncated length rather than anything that
 * looks like an error.
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
