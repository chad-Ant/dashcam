#ifndef HOST_PROTOCOL_H
#define HOST_PROTOCOL_H 1

/**
 * @file HostProtocol.h
 * @brief Portable wire contract for the ESP32-C3 <-> Jetson Orin Nano link
 *        (ESP32-C3 native USB-C -> Jetson USB host port).
 *
 * Keep this file byte-identical on both sides:
 *   peripherals/esp32-c3/lib/hostLink/HostProtocol.h   (bridge firmware)
 *   lib/libcommlink/HostProtocol.h                     (Jetson application)
 *
 * It is header-only and depends on nothing but <stdint.h>/<stddef.h>/<string.h>
 * — no Arduino, no POSIX, no project headers — so one copy compiles unchanged
 * under riscv32-esp-elf-g++ and aarch64 g++.
 *
 * ── Relationship to CommProtocol.h ───────────────────────────────────────────
 * This is the SECOND hop of a two-hop chain:
 *
 *   MKR Zero ──UART 115200 (CommProtocol.h)──> ESP32-C3 ──USB CDC (this file)──> Jetson
 *
 * The framing is deliberately identical to the MKR hop (same SOF, same version
 * byte, same CRC-16/CCITT-FALSE, same incremental decoder), so the two links
 * behave the same way on the wire and under fault.  It lives in its own file,
 * in namespace @c hostproto, for two reasons:
 *   - CommProtocol.h is Arduino-only (it takes a @c Stream& and calls millis()),
 *     which cannot compile on the Jetson.
 *   - The MKR hop's contract stays frozen: adding bridge-specific messages here
 *     cannot perturb a link that is already working in the vehicle.
 *
 * @c hostproto::Telemetry is a byte-for-byte copy of @c TelemetryPayload from
 * CommProtocol.h.  The bridge memcpy()s between them and static_asserts the
 * sizes, so a change to one that is not mirrored in the other fails the build
 * rather than corrupting telemetry in the field.
 *
 * ── Frame layout ─────────────────────────────────────────────────────────────
 *   SOF(0x7E) | VER | TYPE(1) | LEN(1) | PAYLOAD(LEN) | CRC16_LE(2)
 *   VER is hostproto::VERSION, currently 0x04; Telemetry is 131 bytes.
 *
 * CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over VER..last payload byte,
 * transmitted low byte first.  Both ends are little-endian IEEE-754, so a
 * packed struct copies verbatim.
 *
 * ── Roles ────────────────────────────────────────────────────────────────────
 * The Jetson is the host/initiator; the C3 answers and pushes.  This mirrors
 * the MKR hop, where the C3 is the initiator and the MKR answers — the C3 is a
 * slave upward and a master downward.  High bit of TYPE set = C3 -> Jetson.
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace hostproto {

// ─── framing constants ────────────────────────────────────────────────────────

constexpr uint8_t  SOF            = 0x7E; ///< Start-of-frame delimiter.
/**
 * Protocol version byte.
 *
 * 0x02 added Telemetry::accel (79 -> 83 bytes), in step with COMM_VERSION on
 * the MKR hop.  MSG_HELLO also carries the struct sizes, so a mismatched pair
 * is reported as a PROTOCOL MISMATCH rather than merely failing to decode.
 *
 * 0x03 added the nine-axis IMU block plus die temperature (83 -> 123 bytes),
 * again in step with COMM_VERSION.
 *
 * 0x04 added the windowed inertial peaks (123 -> 131 bytes) once the LSM6DSOX
 * FIFO started being drained in full, together with the IMU_DATA_GAP and
 * IMU_LOWPOWER flags that say when a peak may not be trusted.
 */
constexpr uint8_t  VERSION        = 0x04;
constexpr uint16_t MAX_PAYLOAD    = 255;  ///< Largest payload (LEN is one byte).
constexpr uint16_t FRAME_OVERHEAD = 6;    ///< SOF+VER+TYPE+LEN + CRC16(2).
constexpr uint16_t MAX_FRAME      = FRAME_OVERHEAD + MAX_PAYLOAD;

/**
 * Abort a partial frame after this inter-byte gap (ms).
 *
 * Deliberately larger than the MKR hop's 50 ms.  USB CDC is not a paced serial
 * line: bytes arrive in bursty 64-byte packets, and either end can be starved
 * for tens of milliseconds by its own scheduler (Linux CDC-ACM URB completion,
 * FreeRTOS task preemption on the C3).  A 50 ms window would discard valid
 * frames that merely straddled a scheduling gap.
 */
constexpr uint32_t RX_TIMEOUT_MS  = 250;

// ─── message types ────────────────────────────────────────────────────────────

/** Message / command identifiers.  High bit set = C3 (bridge) -> Jetson (host). */
enum MsgType : uint8_t {
    // ── Jetson -> C3 ──
    CMD_GET_ONCE     = 0x01, ///< Forward a one-shot telemetry request to the MKR and relay the answer.
    CMD_GET_STATUS   = 0x02, ///< Request one MSG_STATUS immediately.
    /**
     * Announce a new host session: the bridge resets its session state
     * (streaming off, decimation 1) and replies @c MSG_HELLO.
     *
     * The host MUST send this on every port open.  Without it, session start is
     * inferred from the bridge's 5 s host-silence timeout, and a host that
     * reopens the port *faster* than that timeout never produces a rising edge —
     * so it never receives MSG_HELLO, never re-arms streaming, and inherits
     * whatever state the previous process left behind.  If that process had
     * stopped the stream, the link stays silent indefinitely.
     */
    CMD_HELLO        = 0x03,
    CMD_START_STREAM = 0x10, ///< Begin forwarding master telemetry as it arrives (~10 Hz).
    CMD_STOP_STREAM  = 0x11, ///< Stop forwarding telemetry (MSG_STATUS keeps flowing).
    CMD_SET_DECIM    = 0x12, ///< Payload uint8 N (1..255): forward every Nth master frame.
    CMD_PING         = 0x20, ///< Link check.

    // ── C3 -> Jetson ──
    MSG_TELEMETRY    = 0x81, ///< Payload @c Telemetry: newest OBD2+GPS snapshot from the MKR.
    MSG_STATUS       = 0x82, ///< Payload @c BridgeStatus: bridge health, pushed at 1 Hz.
    MSG_LOG          = 0x83, ///< Payload @c LogHeader + ASCII text (no NUL terminator).
    MSG_PONG         = 0xA0, ///< Ping acknowledgement.
    MSG_HELLO        = 0xA1, ///< Payload @c Hello: sent on every host (re)connect.
    MSG_NACK         = 0xEE, ///< Payload @c Nack: malformed or unknown command.
};

/** Result codes for protocol operations. */
enum class Status : int8_t {
    OK           =  0, ///< Generic success.
    FRAME_READY  =  1, ///< A complete, CRC-valid frame was decoded.
    NO_DATA      =  2, ///< No complete frame available yet (non-blocking).
    NOK_NULL     = -1, ///< NULL buffer argument.
    NOK_OVERFLOW = -2, ///< Decoded payload longer than the caller's buffer.
    NOK_CRC      = -3, ///< CRC mismatch; the frame was discarded and resynced.
    NOK_BUSY     = -4, ///< TX buffer lacked room; frame not sent (non-blocking).
};

/** Reason codes carried by @c MSG_NACK. */
enum NackReason : uint8_t {
    NACK_UNKNOWN_TYPE = 1, ///< TYPE is not a command this bridge implements.
    NACK_BAD_LENGTH   = 2, ///< Payload length wrong for that TYPE.
    NACK_BAD_VALUE    = 3, ///< Payload parsed but the value is out of range.
    NACK_NOT_READY    = 4, ///< Command valid but the bridge cannot serve it yet.
};

/** @c BridgeStatus::flags bits. */
constexpr uint8_t BRIDGE_FLAG_MASTER_LINK = 0x01; ///< Master telemetry arrived within the freshness window.
constexpr uint8_t BRIDGE_FLAG_STREAMING   = 0x02; ///< Telemetry forwarding is enabled.
constexpr uint8_t BRIDGE_FLAG_BATT_LOW    = 0x04; ///< Bridge battery below the low threshold.
constexpr uint8_t BRIDGE_FLAG_BATT_CRIT   = 0x08; ///< Bridge battery below the critical threshold.
constexpr uint8_t BRIDGE_FLAG_CHARGING    = 0x10; ///< Bridge battery is charging.
constexpr uint8_t BRIDGE_FLAG_PM_PRESENT  = 0x20; ///< A PowerManager is fitted; battery fields are real.

/** @c Telemetry::flags bits — identical to the COMM_FLAG_* set on the MKR hop. */
constexpr uint8_t TLM_FLAG_OBD2_VALID = 0x01; ///< OBD2 data is live.
constexpr uint8_t TLM_FLAG_GPS_FIX    = 0x02; ///< GPS reported a valid fix.
constexpr uint8_t TLM_FLAG_TIME_VALID = 0x04; ///< UTC date and time are valid.
/**
 * The IMU initialised and has not been declared lost.
 *
 * Distinct from the per-axis NAN sentinels: NAN with this flag CLEAR means no
 * IMU is fitted, NAN with it SET means the IMU is fitted but that channel is
 * stale. A consumer that cannot tell absent hardware from a silent sensor will
 * misdiagnose both.
 */
constexpr uint8_t TLM_FLAG_IMU_PRESENT = 0x08;
/**
 * The GNSS receiver is configured and still emitting PVT packets.
 *
 * Counterpart of @c TLM_FLAG_IMU_PRESENT: @c TLM_FLAG_GPS_FIX alone cannot
 * distinguish an absent or failed receiver from a healthy one with no fix yet.
 * Both show NAN coordinates with GPS_FIX clear, and they call for opposite
 * responses — one is a fault, the other is a normal cold start.
 */
constexpr uint8_t TLM_FLAG_GPS_PRESENT = 0x10;
/**
 * The IMU is answering on only ONE of its two devices — suspected power/wiring
 * fault for the whole module.
 *
 * Both parts share one PCB, one VIN and one ground, so exactly one responding
 * has no benign cause. An unpowered I2C slave keeps acknowledging via parasitic
 * power through the bus pull-ups, which is why the module can look present while
 * actually being unpowered. Readings still arriving in this state come from a
 * parasitically powered part and should not be trusted.
 */
constexpr uint8_t TLM_FLAG_IMU_DEGRADED = 0x20;
/**
 * There is a HOLE in the inertial record for this window.
 *
 * Either the sensor overwrote unread samples, or the master discarded a backlog
 * already older than its freshness window rather than send it stamped current.
 * Same consequence either way: @c imuAccelPeak and @c imuGyroPeak cover less
 * than the interval they claim, so a quiet window carrying this flag is not
 * evidence that nothing happened.
 */
constexpr uint8_t TLM_FLAG_IMU_DATA_GAP = 0x40;
/**
 * The IMU is in low-power sampling because the vehicle is powered off.
 *
 * Driven by IGNITION state — sustained OBD-II silence after the link has been up
 * — not by apparent stillness, so a vehicle waiting at a light stays in full
 * capture.  Readings are honest but coarse: reduced output rate, FIFO bypassed,
 * so @c imuAccelPeak is a peak over sampled points rather than over every sample.
 * Enough to show the vehicle is still, not enough to characterise an impact on a
 * parked car.
 */
constexpr uint8_t TLM_FLAG_IMU_LOWPOWER = 0x80;
/**
 * NOTE: @c Telemetry::flags is one byte and 0x80 is its last bit.  A further
 * flag requires widening the field, which changes the payload size and so
 * requires a VERSION bump on both hops.
 */

/** @c LogHeader::level values (mirrors dashcam::log::LogLevel). */
enum LogLevel : uint8_t { LOG_DEBUG = 0, LOG_INFO = 1, LOG_WARN = 2, LOG_ERROR = 3 };

// ─── payloads ─────────────────────────────────────────────────────────────────

/**
 * @brief Combined OBD2 + GPS snapshot (packed, little-endian).
 *
 * Byte-for-byte identical to @c TelemetryPayload in CommProtocol.h — the bridge
 * relays the master's payload without reinterpreting it.  Missing float values
 * are NaN, so a receiver can tell "not supplied" from a real zero reading.
 * ALWAYS test floats with std::isnan() before using them.
 */
struct __attribute__((packed)) Telemetry {
    uint32_t masterMillis; ///< MKR Zero uptime (ms) — freezes if the master stops publishing.
    // ---- OBD2 ----
    float speed;           ///< Vehicle speed (km/h), as reported (whole km/h).
    /**
     * Longitudinal acceleration (m/s², +ve = accelerating).
     *
     * Derived on the MKR Zero by smoothing the quantised speed and
     * differentiating it, because only the master knows when a SPEED reading
     * actually changed — this 10 Hz stream repeats each value several times, so
     * differentiating it here would produce spikes, not acceleration.
     *
     * NaN while the estimator is warming up or the ECU has not supplied speed.
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
     * NOT to be confused with @c accel above. That is a scalar LONGITUDINAL
     * acceleration derived from the ECU speed signal; these are the raw
     * three-axis measurements from the inertial sensor, in the sensor's own
     * frame as silkscreened on the breakout. Mapping them to the vehicle frame
     * depends on how the board is bolted in and is deliberately left to this
     * side — the master does not guess a mounting orientation.
     *
     * Each is NAN when its channel is stale or absent; see
     * @c TLM_FLAG_IMU_PRESENT for telling "no IMU fitted" from "IMU fitted,
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
    /**
     * Peak |a| and |ω| over the master's trailing window, not at this instant.
     *
     * The axes above are one sample; at 10 Hz that is one of roughly ten the
     * sensor produced since the last frame, so the sample that caught a 10-50 ms
     * impact is usually not the one transmitted.  These are folded on the master
     * from every sample the sensor converted.
     *
     * Magnitudes, so mounting orientation does not matter.  Gravity is included
     * in @c imuAccelPeak — a stationary vehicle reads about 9.81.
     *
     * NAN when the channel is stale or absent.  Check
     * @c TLM_FLAG_IMU_DATA_GAP before trusting a window, and
     * @c TLM_FLAG_IMU_LOWPOWER before trusting its resolution.
     */
    float imuAccelPeak;    ///< Peak |a| over the window (m/s², gravity included).
    float imuGyroPeak;     ///< Peak |ω| over the window (deg/s).
    // ---- status ----
    uint8_t flags;         ///< TLM_FLAG_* bitfield.
};

/**
 * @brief Bridge health, pushed at 1 Hz and on @c CMD_GET_STATUS.
 *
 * This is what lets the Jetson tell the three failure modes apart:
 *   - USB gone         -> no frames of any kind arrive.
 *   - Bridge alive, MKR dead -> MSG_STATUS keeps arriving with BRIDGE_FLAG_MASTER_LINK
 *                               clear and telemetryAgeMs climbing.
 *   - Both healthy     -> MSG_TELEMETRY at the master's rate / decimation.
 */
struct __attribute__((packed)) BridgeStatus {
    uint32_t bridgeMillis;    ///< C3 uptime (ms).
    uint32_t telemetryAgeMs;  ///< ms since the last good master telemetry; UINT32_MAX = never seen.
    uint32_t masterFrames;    ///< Telemetry frames accepted from the MKR since boot.
    uint32_t masterCrcErrors; ///< CRC failures on the MKR hop since boot.
    uint32_t hostFrames;      ///< Command frames accepted from the Jetson since boot.
    uint32_t hostTxDropped;   ///< Frames NOT sent to the Jetson (host absent or TX full).
    float    batteryVolts;    ///< Bridge battery (V); NaN when no PowerManager is fitted.
    float    batteryPercent;  ///< Bridge battery SOC (%); NaN when unavailable.
    float    tempC;           ///< ESP32-C3 die temperature (°C) — not ambient.
    uint16_t freeHeapKb;      ///< Free heap (KB), for leak watching over long runs.
    uint8_t  batteryStatus;   ///< BATTERY_* from powerManager.h; 0xFF when unavailable.
    uint8_t  chargeState;     ///< CHARGE_* from powerManager.h; 0xFF when unavailable.
    uint8_t  flags;           ///< BRIDGE_FLAG_* bitfield.
    uint8_t  decimation;      ///< Active forwarding divisor (1 = every master frame).
    uint8_t  reserved[2];     ///< Zero-filled; keeps the struct even-sized for future fields.
};

/**
 * @brief Sent by the bridge on every host (re)connect, before anything else.
 *
 * A USB device that reappears is indistinguishable from one that never left,
 * so the host uses MSG_HELLO to learn that the C3 rebooted and that its
 * streaming state has been reset to the power-on default.
 */
struct __attribute__((packed)) Hello {
    uint8_t  protoVersion;   ///< Always @c VERSION; a mismatch means incompatible firmware.
    uint8_t  fwMajor;        ///< Bridge firmware major version.
    uint8_t  fwMinor;        ///< Bridge firmware minor version.
    uint8_t  resetReason;    ///< esp_reset_reason() — the forensic trail for an unexplained reboot.
    uint8_t  telemetryBytes; ///< sizeof(Telemetry) as the bridge sees it — wire self-check.
    uint8_t  statusBytes;    ///< sizeof(BridgeStatus) as the bridge sees it — wire self-check.
    uint16_t bootCount;      ///< Boots since power-on (RTC-preserved across resets, not power loss).
    uint32_t bridgeMillis;   ///< C3 uptime (ms) at the moment of connect.
};

/** @brief Header of a @c MSG_LOG frame; ASCII text follows, length = LEN-1, no NUL. */
struct __attribute__((packed)) LogHeader {
    uint8_t level; ///< @c LogLevel.
};

/** @brief Payload of a @c MSG_NACK frame. */
struct __attribute__((packed)) Nack {
    uint8_t offendingType; ///< TYPE byte that was rejected.
    uint8_t reason;        ///< @c NackReason.
};

/// Longest ASCII payload a single MSG_LOG can carry.
constexpr size_t MAX_LOG_TEXT = MAX_PAYLOAD - sizeof(LogHeader);

// The wire contract depends on these exact sizes on both ends.  Telemetry must
// also equal sizeof(TelemetryPayload) in CommProtocol.h — the bridge asserts
// that separately, where both headers are visible.
static_assert(sizeof(Telemetry)    == 131, "hostproto::Telemetry must be tightly packed to 131 bytes");
/**
 * Must fit the frame's one-byte LEN field and @c Hello::telemetryBytes, which is
 * also one byte.  Stated explicitly now the struct has grown 79 -> 83 -> 123 -> 131:
 * past 255 the length silently truncates rather than failing visibly.
 */
static_assert(sizeof(Telemetry)    <= MAX_PAYLOAD,
              "hostproto::Telemetry no longer fits the frame's one-byte LEN field");
static_assert(sizeof(BridgeStatus) == 44, "hostproto::BridgeStatus must be tightly packed to 44 bytes");
static_assert(sizeof(Hello)        == 12, "hostproto::Hello must be tightly packed to 12 bytes");
static_assert(sizeof(Nack)         ==  2, "hostproto::Nack must be tightly packed to 2 bytes");

// ─── CRC-16/CCITT-FALSE ───────────────────────────────────────────────────────

/** @brief One CRC-16/CCITT-FALSE step (poly 0x1021, init 0xFFFF). */
inline uint16_t crc16Update(uint16_t crc, uint8_t b)
{
    crc ^= static_cast<uint16_t>(b) << 8;
    for (uint8_t i = 0; i < 8; ++i) {
        crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                             : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

/** @brief CRC-16/CCITT-FALSE over a byte range. */
inline uint16_t crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) crc = crc16Update(crc, data[i]);
    return crc;
}

// ─── encoder ──────────────────────────────────────────────────────────────────

/**
 * @brief Serialises one frame into @p out.
 * @param type     Message type (see @c MsgType).
 * @param payload  Payload bytes; may be NULL only when @p len is 0.
 * @param len      Payload length in bytes.
 * @param out      Destination buffer.
 * @param cap      Capacity of @p out; must be >= FRAME_OVERHEAD + len.
 * @return Total frame length in bytes, or 0 on error (NULL / too small / bad args).
 */
inline size_t buildFrame(uint8_t type, const uint8_t *payload, uint8_t len,
                         uint8_t *out, size_t cap)
{
    if (!out) return 0;
    if (len && !payload) return 0;

    const size_t need = static_cast<size_t>(FRAME_OVERHEAD) + len;
    if (cap < need) return 0;

    size_t n = 0;
    out[n++] = SOF;
    out[n++] = VERSION;
    out[n++] = type;
    out[n++] = len;

    uint16_t crc = 0xFFFF;
    crc = crc16Update(crc, VERSION);
    crc = crc16Update(crc, type);
    crc = crc16Update(crc, len);
    for (uint8_t i = 0; i < len; ++i) {
        out[n++] = payload[i];
        crc = crc16Update(crc, payload[i]);
    }

    out[n++] = static_cast<uint8_t>(crc & 0xFF);        // CRC low byte first
    out[n++] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    return n;
}

// ─── incremental decoder ──────────────────────────────────────────────────────

/** Parser phase for the incremental frame decoder. */
enum class RxPhase : uint8_t { WAIT_SOF, VER, TYPE, LEN, PAYLOAD, CRC_LO, CRC_HI };

/**
 * @brief Incremental, non-blocking frame-decoder state (no heap, no I/O).
 *
 * Unlike CommProtocol.h's decoder this one never touches a Stream or a clock:
 * the caller passes the current millisecond count into @c rxByte().  That keeps
 * the file portable between an MCU (millis()) and Linux (CLOCK_MONOTONIC).
 */
struct RxState {
    RxPhase  phase;                 ///< Current parser phase.
    uint8_t  type;                  ///< Type byte of the in-progress frame.
    uint8_t  len;                   ///< Declared payload length.
    uint16_t idx;                   ///< Payload bytes received so far.
    uint16_t crcRx;                 ///< CRC received from the wire.
    uint32_t lastByteMs;            ///< Timestamp of the previous byte (partial-frame timeout).
    uint32_t crcErrors;             ///< Running CRC-failure count since rxInit().
    uint32_t framesDecoded;         ///< Running count of CRC-valid frames since rxInit().
    uint8_t  buf[MAX_PAYLOAD];      ///< Payload accumulator.
};

/** @brief Drops any partial frame and returns the parser to WAIT_SOF. Counters survive. */
inline void rxResync(RxState &rx)
{
    rx.phase = RxPhase::WAIT_SOF;
    rx.type  = 0;
    rx.len   = 0;
    rx.idx   = 0;
    rx.crcRx = 0;
}

/** @brief Full reset including the statistics counters. Call once before first use. */
inline void rxInit(RxState &rx)
{
    rxResync(rx);
    rx.lastByteMs    = 0;
    rx.crcErrors     = 0;
    rx.framesDecoded = 0;
}

/**
 * @brief Feeds one byte into the decoder.
 *
 * @param rx          Decoder state.
 * @param b           The received byte.
 * @param nowMs       Current time in ms (millis() on the C3, monotonic clock on Linux).
 * @param typeOut     Receives the TYPE byte when a frame completes.
 * @param payloadOut  Buffer to receive the payload when a frame completes.
 * @param payloadCap  Capacity of @p payloadOut.
 * @param lenOut      Receives the payload length when a frame completes.
 * @return @c FRAME_READY (type/payload/len filled), @c NO_DATA (need more bytes),
 *         @c NOK_CRC (frame failed CRC — parser resynced), @c NOK_OVERFLOW
 *         (declared length exceeds @p payloadCap — frame dropped), or
 *         @c NOK_NULL (@p payloadOut was NULL).
 */
inline Status rxByte(RxState &rx, uint8_t b, uint32_t nowMs,
                     uint8_t &typeOut, uint8_t *payloadOut, uint8_t payloadCap, uint8_t &lenOut)
{
    if (!payloadOut) return Status::NOK_NULL;

    // Abort a stalled partial frame so a truncated transmission cannot swallow
    // the bytes of the next valid one.  Unsigned subtraction is rollover-safe.
    if (rx.phase != RxPhase::WAIT_SOF && (nowMs - rx.lastByteMs) > RX_TIMEOUT_MS) {
        rxResync(rx);
    }
    rx.lastByteMs = nowMs;

    switch (rx.phase) {
    case RxPhase::WAIT_SOF:
        if (b == SOF) rx.phase = RxPhase::VER;
        break;

    case RxPhase::VER:
        if (b == VERSION)   rx.phase = RxPhase::TYPE;
        else if (b == SOF)  rx.phase = RxPhase::VER;      // consecutive SOF: keep waiting for VER
        else                rx.phase = RxPhase::WAIT_SOF; // garbage: resync
        break;

    case RxPhase::TYPE:
        rx.type  = b;
        rx.phase = RxPhase::LEN;
        break;

    case RxPhase::LEN:
        rx.len   = b;
        rx.idx   = 0;
        rx.phase = (b == 0) ? RxPhase::CRC_LO : RxPhase::PAYLOAD;
        break;

    case RxPhase::PAYLOAD:
        rx.buf[rx.idx++] = b;
        if (rx.idx >= rx.len) rx.phase = RxPhase::CRC_LO;
        break;

    case RxPhase::CRC_LO:
        rx.crcRx = b;
        rx.phase = RxPhase::CRC_HI;
        break;

    case RxPhase::CRC_HI: {
        rx.crcRx |= static_cast<uint16_t>(b) << 8;
        rx.phase  = RxPhase::WAIT_SOF; // frame consumed regardless of outcome

        uint16_t c = 0xFFFF;
        c = crc16Update(c, VERSION);
        c = crc16Update(c, rx.type);
        c = crc16Update(c, rx.len);
        for (uint16_t i = 0; i < rx.len; ++i) c = crc16Update(c, rx.buf[i]);

        if (c != rx.crcRx) { ++rx.crcErrors; return Status::NOK_CRC; }
        if (rx.len > payloadCap) return Status::NOK_OVERFLOW;

        ++rx.framesDecoded;
        typeOut = rx.type;
        lenOut  = rx.len;
        memcpy(payloadOut, rx.buf, rx.len);
        return Status::FRAME_READY;
    }
    }

    return Status::NO_DATA;
}

// ─── diagnostics ──────────────────────────────────────────────────────────────

/** @brief Human-readable name for a TYPE byte; "UNKNOWN" for anything unrecognised. */
inline const char *typeName(uint8_t type)
{
    switch (type) {
    case CMD_GET_ONCE:     return "CMD_GET_ONCE";
    case CMD_GET_STATUS:   return "CMD_GET_STATUS";
    case CMD_HELLO:        return "CMD_HELLO";
    case CMD_START_STREAM: return "CMD_START_STREAM";
    case CMD_STOP_STREAM:  return "CMD_STOP_STREAM";
    case CMD_SET_DECIM:    return "CMD_SET_DECIM";
    case CMD_PING:         return "CMD_PING";
    case MSG_TELEMETRY:    return "MSG_TELEMETRY";
    case MSG_STATUS:       return "MSG_STATUS";
    case MSG_LOG:          return "MSG_LOG";
    case MSG_PONG:         return "MSG_PONG";
    case MSG_HELLO:        return "MSG_HELLO";
    case MSG_NACK:         return "MSG_NACK";
    default:               return "UNKNOWN";
    }
}

/**
 * @brief True if @p type is a command the bridge implements (Jetson -> C3).
 *
 * Checked before the length check so an unrecognised TYPE is reported as
 * @c NACK_UNKNOWN_TYPE rather than @c NACK_BAD_LENGTH — the two point at very
 * different faults (wrong firmware vs. corrupted frame), and a misleading
 * reason code sends the next person debugging this down the wrong path.
 */
inline bool isCommand(uint8_t type)
{
    switch (type) {
    case CMD_GET_ONCE:
    case CMD_GET_STATUS:
    case CMD_HELLO:
    case CMD_START_STREAM:
    case CMD_STOP_STREAM:
    case CMD_SET_DECIM:
    case CMD_PING:
        return true;
    default:
        return false;
    }
}

/** @brief Expected payload length for a command TYPE; 0 for the zero-payload commands. */
inline uint8_t commandPayloadLen(uint8_t type)
{
    return (type == CMD_SET_DECIM) ? 1 : 0;
}

} // namespace hostproto

#endif // HOST_PROTOCOL_H
