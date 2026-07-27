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
#define COMM_VERSION         0x01u  ///< Protocol version byte.
#define COMM_MAX_PAYLOAD     255u   ///< Largest payload (LEN is one byte).
#define COMM_FRAME_OVERHEAD  6u     ///< SOF+VER+TYPE+LEN + CRC16(2).
#define COMM_MAX_FRAME       (COMM_FRAME_OVERHEAD + COMM_MAX_PAYLOAD)
#define COMM_RX_TIMEOUT_MS   50UL   ///< Abort a partial frame after this inter-byte gap (ms).

/// Telemetry @c flags bits.
#define COMM_FLAG_OBD2_VALID 0x01u  ///< OBD2 data is live (a reading arrived within the master's freshness window).
#define COMM_FLAG_GPS_FIX    0x02u  ///< GPS reported a valid fix.
#define COMM_FLAG_TIME_VALID 0x04u  ///< UTC date and time are valid.

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
    float speed;           ///< Vehicle speed (km/h).
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
    // ---- status ----
    uint8_t flags;         ///< COMM_FLAG_* bitfield.
};

/// The wire contract depends on this exact size on both MCUs.
static_assert(sizeof(TelemetryPayload) == 79, "TelemetryPayload must be tightly packed to 79 bytes");

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
