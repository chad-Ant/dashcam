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
 *   VER is hostproto::VERSION, currently 0x07; Telemetry is 180 bytes.
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
 *
 * 0x05 added the vehicle-bus block decoded from CAN sniffing (131 -> 148 bytes):
 * gear, brake and turn bits, pedal, EPS torque, yaw rate and per-wheel speeds.
 *
 * 0x06 (148 -> 160 bytes) is the largest single change since the IMU block, and
 * it is two changes that had to travel together because either alone would have
 * cost a version bump of its own:
 *
 *   - THE INERTIAL SENSOR CHANGED. The LSM6DSOX + LIS3MDL breakout was retired
 *     after it spent 40 % of its samples returning corrupt-but-plausible values
 *     through transactions that all succeeded, then latched up. Its replacement,
 *     a BNO055, fuses on-chip — so @c imuLinAccelPeak (gravity already removed),
 *     @c imuYawRelDeg and @c imuCalib exist for the first time, the magnetometer
 *     fields are NAN by design, and IMU_HIGH_G and IMU_SATURATED say things the
 *     previous part could not.
 *   - THE CAN DECODE TABLE BECAME IDENTIFIABLE. @c canMapChecksum and
 *     @c canMapFlags name the map that produced the vehicle fields, which a
 *     recording needs in order to be re-interpretable later.
 *
 * @c flags widened from uint8 to uint16 to carry the three new bits, and
 * IMU_DEGRADED changed meaning — read it against VERSION, not on its own.
 *
 * 0x07 (160 -> 180 bytes) closes two holes and adds a subsystem:
 *
 *   - DIRECTION. @c imuLinAccelPeak is a MAGNITUDE, so a frontal impact and a
 *     side impact of equal severity are the same number. @c imuLinAccelX/Y/Z
 *     carry the signed vector, which is what separates them and what any
 *     reconstruction needs.
 *   - A HIGH-G EVENT CAN NO LONGER BE MISSED. IMU_HIGH_G is held for a fixed
 *     window and then clears, so a decimated stream could step over one
 *     entirely. @c imuHighGCount only climbs, so any two frames bracket every
 *     event between them; @c imuHighGMs locates the most recent one in time.
 *   - PANEL SWITCHES. @c switchState and @c switchChanged, qualified by
 *     @c TLM_FLAG_SWITCHES_PRESENT.
 */
constexpr uint8_t  VERSION        = 0x07;
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
    /**
     * Payload uint8: 1=discover 2=sniff 3=obd2. Relayed to the MKR.
     *
     * The MKR never switches to OBD2 MID-DRIVE on its own. OBD2 transmits on a
     * live vehicle bus, and a node that decides that unattended, on a car in
     * motion, because a signal went quiet for a second, is not a decision
     * firmware should make.
     *
     * It DOES choose at boot, once, and then never revisits it: no vehicle map
     * on the SD card, or a map whose IDs never appear during the probe window,
     * selects OBD2 before the vehicle is moving. That is a human choosing - the
     * card is written by hand - and the alternative is an unconfigured install
     * that produces no telemetry at all. The boot decision is reported in
     * @c Telemetry::canMode rather than left to be inferred.
     *
     * This comment previously claimed there was no automatic fallback at all,
     * which the boot path had already contradicted. A host command still
     * outranks the boot decision and skips the probe entirely.
     */
    CMD_SET_CAN_MODE = 0x13,
    /**
     * Replace the master's hardware receive filters. 13-byte payload, relayed.
     *
     * Layout: uint8 count, then 6 x uint16 little-endian CAN IDs; entries past
     * @c count are ignored but must still be transmitted, so the expected length
     * stays a pure function of TYPE.
     *
     * A count of 0 clears every filter, which on the master's MCP2515 means
     * ACCEPT ALL rather than accept none — the opposite of what "no filters"
     * sounds like, and the correct behaviour for discovery. The map's filter set
     * is an optimisation for a known vehicle, not a security boundary.
     *
     * Exists so a host can widen the capture without editing the SD card and
     * rebooting the vehicle: @c canMapFlags tells it what is currently being
     * excluded, and this changes it.
     */
    CMD_SET_CAN_FILTER = 0x14,
    /**
     * Select the IMU's operating mode; relayed to the MKR. 1-byte payload.
     *
     * @c IMU_MODE_FUSION or @c IMU_MODE_RAW. The two are DIFFERENT MEASUREMENTS,
     * not a quality setting — fusion yields gravity-compensated linear
     * acceleration and relative yaw but clips at 4 g, while raw reaches 16 g and
     * produces no fusion at all. Which one is right depends on what the host is
     * trying to record, so it is a session-level decision.
     *
     * ⚠️ EXPENSIVE AND BLINDING. Applying it restarts the sensor's bring-up: the
     * IMU publishes nothing for about 700 ms and the trailing peak window is
     * discarded. A crash pulse lasts 10-50 ms, so switching mode ON an impact
     * would complete long after the event and would throw away the only record
     * of it. Choose the mode before the drive, not during an incident.
     *
     * Not restored automatically after a master or bridge reset — the MKR comes
     * back in its compiled-in default. @c TLM_FLAG_IMU_FUSION_MODE reports the
     * mode on every frame, so a host re-establishing a link should read it
     * rather than assume its last request survived.
     *
     * The bridge NACKs an out-of-range value; a request the master's TX cannot
     * accept is reported as a bridge log line, not retried.
     */
    CMD_SET_IMU_MODE = 0x15,
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

/** @brief Provenance codes packed two bits at a time into @c Telemetry::sigSource. */
enum class VehSigSource : uint8_t {
    NONE      = 0, ///< Never written, or structurally unavailable in this mode.
    CAN_SNIFF = 1, ///< Decoded from a broadcast frame.
    OBD2      = 2  ///< Answered by an ECU to a Mode 01 request.
};

/// Unpack @c Telemetry::sigSource. Two bits each, low pair first.
inline VehSigSource sigSourceSpeed(uint8_t s) { return static_cast<VehSigSource>( s        & 0x03); }
inline VehSigSource sigSourceRpm  (uint8_t s) { return static_cast<VehSigSource>((s >> 2)  & 0x03); }
inline VehSigSource sigSourceGear (uint8_t s) { return static_cast<VehSigSource>((s >> 4)  & 0x03); }
inline VehSigSource sigSourceSteer(uint8_t s) { return static_cast<VehSigSource>((s >> 6)  & 0x03); }

/**
 * @c Telemetry::vehFlags bits.
 *
 * Named because a lane-keeping consumer reading bit 2 as bit 3 is a silent
 * left/right swap, and a magic number is how that happens.
 */
constexpr uint8_t VEH_FLAG_BRAKE_PRESSED = 0x01;
constexpr uint8_t VEH_FLAG_BRAKE_SWITCH  = 0x02;
constexpr uint8_t VEH_FLAG_TURN_LEFT     = 0x04;
constexpr uint8_t VEH_FLAG_TURN_RIGHT    = 0x08;
/**
 * Both indicator bits. NOT a hazards predicate — see VEH_FLAG_HAZARD.
 *
 * Do not test hazards as `flags & TURN_MASK`: that is true for a single
 * indicator too. And do not test them as `== TURN_MASK` either, which is what
 * this comment used to recommend. Measured on the vehicle: switching the hazards
 * on leaves BOTH turn bits clear, because the indicator message reports the
 * stalk and the hazard switch bypasses it. Hazards have their own bit.
 */
constexpr uint8_t VEH_FLAG_TURN_MASK     = VEH_FLAG_TURN_LEFT | VEH_FLAG_TURN_RIGHT;

/// Hazard lights, an independent signal — NOT both indicators at once.
constexpr uint8_t VEH_FLAG_HAZARD        = 0x80;

/**
 * Validity bits. Zero means "no data", NOT "released / not indicating".
 *
 * Without them an absent or stale signal serialises identically to the safe-
 * looking state. For lane keeping that inverts the verdict: in OBD2 mode the
 * indicator bits cannot be populated at all, so every lane change would read as
 * an UNSIGNALLED departure. Check these before believing a cleared bit.
 */
constexpr uint8_t VEH_FLAG_BRAKE_VALID   = 0x10;
constexpr uint8_t VEH_FLAG_TURN_VALID    = 0x20;
constexpr uint8_t VEH_FLAG_PEDAL_VALID   = 0x40;

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
 * The IMU is running, but NOT in the configuration that was asked for.
 *
 * v0x06 REDEFINED THIS FLAG. Through v0x05 the master carried a two-chip
 * breakout and this meant "only one of the two devices is answering" — a power
 * fault, since an unpowered I2C slave keeps acknowledging on parasitic current
 * through the bus pull-ups while its neighbour dies.
 *
 * The single-chip BNO055 that replaced it cannot produce that state. The flag
 * now means the sensor came up on a fallback — the internal oscillator after the
 * external crystal was refused, or a mode other than the one requested. The data
 * is good; it is not the data that was configured.
 *
 * Interpret against @c VERSION, not on its own.
 */
constexpr uint16_t TLM_FLAG_IMU_DEGRADED = 0x20;
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
 * The IMU is in a coarser sampling regime than normal.
 *
 * RESERVED AND ALWAYS CLEAR as of v0x06. It described the previous sensor's
 * low-power mode, selected from ignition state. The BNO055 has an equivalent and
 * it is not enabled: the datasheet withdraws the High-G interrupt in low power,
 * which would remove the hardware impact backstop exactly when a car-park bump is
 * what is being watched for. The bit is kept so nothing below it shifts.
 */
constexpr uint16_t TLM_FLAG_IMU_LOWPOWER = 0x80;
/**
 * The IMU latched a High-G threshold crossing during this window.
 *
 * THE ONLY FIELD HERE THAT SURVIVES A STALLED MASTER. Every other inertial value
 * is the product of a poll, and the sensor has no FIFO — so a poll the master
 * fails to make is data that no longer exists. This is latched in the sensor's
 * own hardware and held until read, so an impact during a stalled loop is still
 * reported afterwards, where the peak that would have described it is gone.
 *
 * For incident detection this is a STRONGER trigger than @c imuAccelPeak, not a
 * weaker one: the peak is only ever as good as the polls behind it.
 */
constexpr uint16_t TLM_FLAG_IMU_HIGH_G = 0x0100;
/**
 * The accelerometer hit its range limit, so the peaks are FLOORS.
 *
 * The sensor locks the accelerometer at ±4 g in fusion mode, so anything past
 * 39.2 m/s² is clipped and a genuine 20 g collision reads as 4 g. Both
 * @c imuAccelPeak and @c imuLinAccelPeak are affected — linear acceleration is
 * derived from the same clipped measurement. Grading severity from a saturated
 * peak understates an impact several-fold.
 */
constexpr uint16_t TLM_FLAG_IMU_SATURATED = 0x0200;
/**
 * A CAN signal map is loaded and in use; @c canMapChecksum identifies which.
 *
 * Without this a consumer cannot tell a vehicle whose map decoded nothing from
 * one running with no map at all — both publish the same sentinels.
 */
constexpr uint16_t TLM_FLAG_CANMAP_LOADED = 0x0400;
/**
 * The hardware High-G backstop is ARMED.
 *
 * The counterpart of @c TLM_FLAG_IMU_HIGH_G, and useless without it. That flag
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
 * A consumer treating @c TLM_FLAG_IMU_HIGH_G as its incident trigger should read
 * this alongside it. Absence of the event flag means "no impact detected" only
 * while this is set; with it clear the honest reading is "not watched for".
 *
 * Added within v0x06 — a flag bit inside an existing @c uint16_t changes no
 * offset and no payload size, so a consumer built against the earlier v0x06
 * header ignores it exactly as it ignores any bit it does not know.
 */
constexpr uint16_t TLM_FLAG_IMU_HIGHG_ARMED = 0x0800;
/**
 * The IMU is in a FUSION mode (IMUPLUS); clear means the raw mode (AMG).
 *
 * The two publish different fields and mean different things by the same ones,
 * so a consumer that does not know which it is holding cannot read the frame
 * correctly. In fusion the magnetometer is off and @c imuMagX/Y/Z are NAN while
 * @c imuLinAccelPeak and @c imuYawRelDeg are live; in raw there is no fusion, so
 * those two are NAN and the magnetometer is populated. @c imuAccelPeak saturates
 * at 4 g in fusion and 16 g in raw — the same number, two different rails, which
 * is why grading an impact from it requires knowing this bit.
 *
 * Inferring the mode from which fields happen to be NAN was the only option
 * before this bit existed, and it is not the same thing: a stale channel and a
 * mode that does not produce that channel look identical.
 *
 * It also closes the loop on @c CMD_SET_IMU_MODE, which otherwise asks for a
 * change the host has no way to observe.
 */
constexpr uint16_t TLM_FLAG_IMU_FUSION_MODE = 0x1000;
/**
 * The 74HC165 switch chain answered its sentinels on the master's most recent
 * poll, so @c switchState describes measured positions.
 *
 * The counterpart of @c TLM_FLAG_IMU_PRESENT and @c TLM_FLAG_GPS_PRESENT, and
 * added for exactly their reason. Every switch on the panel latches and is
 * legitimately open most of the time, so an absent chain, a dead register and
 * fourteen open switches all produce the same word. Two of the sixteen inputs
 * are hard-wired to opposite rails and checked on every read; this bit is that
 * check.
 *
 * Clear means @c switchState is STALE, not that every switch is open — the
 * master holds the last measured positions rather than zeroing them, because
 * zeroed positions look measured.
 */
constexpr uint16_t TLM_FLAG_SWITCHES_PRESENT = 0x2000;
/**
 * NOTE: @c Telemetry::flags became @c uint16_t in v0x06. With 0x2000 in use,
 * TWO bits remain — 0x4000 and 0x8000.
 *
 * (The v0x06 note here said seven remained, which was a miscount: thirteen of
 * the sixteen were already allocated when it was written. Recorded rather than
 * quietly corrected, because that number is the input to deciding when the next
 * widening becomes unavoidable, and a wrong one postpones the decision past the
 * point where it is cheap.)
 *
 * A further widening is another payload size change and so another VERSION bump
 * on both hops.
 */

/// @c CMD_SET_IMU_MODE payload values.
///
/// 1-based, so 0 stays available as "nothing pending" in the bridge's latch —
/// the same convention @c CMD_SET_CAN_MODE uses, and for the same reason.
constexpr uint8_t IMU_MODE_FUSION = 1;  ///< IMUPLUS: on-chip fusion, magnetometer off.
constexpr uint8_t IMU_MODE_RAW    = 2;  ///< AMG: raw accel/mag/gyro, no fusion, +/-16 g.

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
     * ---- IMU (BNO055, sensor frame) ----
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
    /**
     * Magnetic flux density per sensor axis (µT), uncalibrated.
     *
     * NAN IN NORMAL OPERATION as of v0x06, deliberately rather than for want of
     * a sensor. The master runs the BNO055 in its magnetometer-free fusion mode
     * on purpose: the magnetometer feeds the orientation quaternion that linear
     * acceleration is derived from, so magnetic disturbance would contaminate
     * the one channel incident detection depends on — and a car, with its steel
     * shell and load-dependent currents, is close to a worst case. Populated
     * only in the raw (AMG) mode.
     */
    float imuMagX;
    float imuMagY;
    float imuMagZ;
    float imuTempC;        ///< Sensor die temperature (°C) — board, not cabin.
    /**
     * Peak |a| and |ω| over the master's trailing window, not at this instant.
     *
     * The axes above are one sample; at 10 Hz that is one of roughly ten the
     * sensor produced since the last frame, so the sample that caught a 10-50 ms
     * impact is usually not the one transmitted.  These are folded on the master
     * from every sample it polled.
     *
     * WHAT BACKS THEM CHANGED IN v0x06. Through v0x05 the sensor batched into a
     * 512-word FIFO that the master drained in full, so every converted sample
     * reached the peak however slowly the host polled. The BNO055 has no FIFO,
     * so the master polls at the sensor's own 100 Hz output rate — which catches
     * the same impulses, but means a poll the master fails to make is data that
     * no longer exists anywhere. @c TLM_FLAG_IMU_DATA_GAP therefore carries more
     * weight than it did, and @c TLM_FLAG_IMU_HIGH_G is the hardware backstop
     * for exactly that case.
     *
     * Magnitudes, so mounting orientation does not matter.  Gravity is included
     * in @c imuAccelPeak — a stationary vehicle reads about 9.81.
     *
     * NAN when the channel is stale or absent.  Check
     * @c TLM_FLAG_IMU_DATA_GAP before trusting a window, and
     * @c TLM_FLAG_IMU_SATURATED before grading severity from one.
     */
    float imuAccelPeak;    ///< Peak |a| over the window (m/s², gravity included).
    float imuGyroPeak;     ///< Peak |ω| over the window (deg/s).
    /**
     * ---- IMU fusion (v0x06) ----
     *
     * Computed by the sensor, not derived here. They did not exist before v0x06
     * because the previous part could not produce them at all.
     */
    /**
     * Peak |linear a| over the window (m/s²), GRAVITY REMOVED.
     *
     * The field incident detection actually wants. @c imuAccelPeak includes
     * gravity, so a stationary vehicle reads 9.81 and every threshold has to
     * carry that offset; this reads ~0 at rest, so 1 g here means the vehicle
     * accelerated at 1 g. NAN in the raw (AMG) mode, which produces no fusion.
     */
    float imuLinAccelPeak;
    /**
     * ---- signed linear acceleration (v0x07) ----
     *
     * Sensor-frame linear acceleration with gravity removed by the on-chip
     * fusion (m/s²). NAN in the raw (AMG) mode, which produces no fusion.
     *
     * WHY THE PEAK WAS NOT ENOUGH. @c imuLinAccelPeak is a magnitude, so a
     * frontal impact and a side impact of the same severity are the same
     * number. Direction is most of what an incident reconstruction wants —
     * which way the energy arrived from decides what the recording means — and
     * it cannot be recovered from a magnitude afterwards at any price.
     *
     * These are INSTANTANEOUS, from the frame's own sample; the peak beside
     * them is the window. A frame carrying a High-G event will usually not have
     * caught the peak instant in these three, which is the honest limit of
     * publishing at 10 Hz what is sampled at 100 Hz: they give the DIRECTION of
     * an event and @c imuLinAccelPeak gives its SIZE.
     */
    float imuLinAccelX;
    float imuLinAccelY;
    float imuLinAccelZ;
    /**
     * Age of the most recent High-G latch (ms). @c 0xFFFF when none has
     * occurred, or the last was longer ago than this can express.
     *
     * An AGE rather than a timestamp, so it needs no arithmetic against
     * @c masterMillis and carries its own "never" value. Saturating at 65.5 s
     * costs nothing: past a minute the only question left is the count.
     */
    uint16_t imuHighGMs;
    /**
     * Cumulative High-G latches since the master booted. Wraps at 65535.
     *
     * THE FIELD THAT MAKES THE EVENT UNMISSABLE. @c TLM_FLAG_IMU_HIGH_G is held
     * for a fixed window and then clears, so an application reading a decimated
     * stream — or one that lost a frame to a full ring — can step straight over
     * an impact and see nothing. A monotonic counter cannot be stepped over: any
     * two frames bracket every event between them, whatever was dropped in the
     * middle.
     *
     * Compare against the previous frame's value. Do not test for non-zero.
     */
    uint16_t imuHighGCount;
    /**
     * Rotation about the vertical (deg), RELATIVE AND DRIFTING.
     *
     * NOT a heading, and named so it cannot be mistaken for one — the sensor
     * runs with its magnetometer off, so this has no north reference. What it is
     * good for is short-window rotation at 100 Hz: how far the vehicle turned
     * during a two-second event, where drift is negligible and neither the 1 Hz
     * GNSS course nor the wheel-speed pair (dead below ~3 km/h) can answer. For
     * an absolute bearing use @c heading.
     */
    float imuYawRelDeg;
    /**
     * Sensor self-assessed calibration, 2 bits each:
     * bits 1:0 mag, 3:2 accel, 5:4 gyro, 7:6 system. 0 = uncalibrated, 3 = full.
     *
     * MAG AND SYSTEM ARE PERMANENTLY 0 in fusion mode and that is correct — the
     * magnetometer is off and the system figure cannot rise without it. Judge
     * readiness on the gyroscope field.
     *
     * The ACCELEROMETER field is reported but is a poor quality gate: on the
     * bench it fell to 0 and stayed there for 9000 polls while the sensor's own
     * gravity vector held 9.79-9.81 m/s². The master gates on that gravity
     * magnitude instead — a direct physical test rather than the part's opinion
     * of itself.
     */
    uint8_t imuCalib;
    /**
     * ---- CAN signal map identity (v0x06) ----
     *
     * Which decode table produced the vehicle fields below.
     *
     * The master loads a per-vehicle map from SD, so the MEANING of @c gearPos,
     * @c vehFlags and the rest depends on a file editable between drives without
     * a firmware flash. A recording that does not say which map decoded it
     * cannot be re-interpreted later — and a wrong map does not fail loudly, it
     * produces plausible values from the wrong bytes.
     *
     * Zero when no map is loaded; check @c TLM_FLAG_CANMAP_LOADED, since zero is
     * also a legitimate checksum.
     */
    uint8_t canMapChecksum;
    /**
     * bit 0 hardware filters active, bit 1 filter set came from the map,
     * bits 2-7 reserved and zero.
     *
     * Filters decide what CANNOT appear: an ID absent from a filtered capture
     * may be absent from the bus or merely excluded by the controller, and only
     * this says which.
     */
    uint8_t canMapFlags;
    // ---- status ----
    /**
     * TLM_FLAG_* bitfield.
     *
     * WIDENED FROM uint8 IN v0x06. The v0x05 header noted 0x80 was the last bit
     * and that the next flag would force this; three arrived at once.
     */
    uint16_t flags;
    /**
     * ---- vehicle bus (v0x05) ----
     *
     * Byte-for-byte the tail of @c TelemetryPayload on the MKR hop; the bridge
     * memcpy()s the whole struct, so these must stay in lockstep.
     *
     * Filled by CAN sniffing when it is running and by OBD-II polling when it
     * is not, so the host reads the same fields either way.  @c sigSource says
     * which source supplied each, and that matters: the two do not cover the
     * same set.  @c steerMotorTorque, @c yawRateCdps and @c wheelRaw exist ONLY
     * while sniffing and sit at their sentinels in OBD2 mode, because that mode
     * structurally cannot supply them - a different statement from "the sensor
     * went quiet".
     */
    uint8_t canMode;       ///< 0=off 1=discover 2=sniff 3=obd2.
    uint8_t sigSource;     ///< 2 bits each: speed|rpm|gear|steer. 0=none 1=CAN 2=OBD2.
    uint8_t gearPos;       ///< 0=unknown 1=P 2=R 3=N 4=D 5=L 6=S.
    /**
     * bit 0 brakePressed, bit 1 brakeSwitch,
     * bit 2 turnLeft, bit 3 turnRight,
     * bit 4 brakeValid, bit 5 turnValid, bit 6 pedalValid, bit 7 hazard.
     *
     * The turn bits are indicator ACTIVE, not indicator lamp lit: the lamp
     * blinks at ~1.5 Hz and telemetry arrives at ~4 Hz, so the sender holds
     * each flash for 900 ms rather than let the blink alias.  Both set at once
     * is hazard lights.  Earlier firmware left bits 2-7 zero, so a host that
     * predates this reads "not indicating" rather than garbage.
     */
    uint8_t vehFlags;
    uint8_t pedalGas;      ///< Accelerator, raw 0-255; x0.5 = percent.
    /**
     * EPS motor assist torque, raw 0-511. @c 0xFFFF when unavailable.
     *
     * NOT a steering angle - an unsigned MAGNITUDE that rises for either
     * direction of turn and returns to exactly zero when effort stops.  The
     * vehicle publishes no steering angle anywhere on its bus and J1979 has no
     * steering PID, so no angle field exists here to be filled in later.  For
     * heading change use @c yawRateCdps.
     */
    uint16_t steerMotorTorque;
    /**
     * Yaw rate, centi-degrees/s, LEFT POSITIVE. @c INT16_MIN when unavailable.
     *
     * Derived on the MKR from the rear wheel pair at 50 Hz: this stream runs at
     * ~10 Hz and the signal swings 30 deg/s inside one U-turn, so deriving it
     * host-side would be aliased beyond use.
     *
     * The sentinel is NOT zero - zero means genuinely straight.  The wheel
     * sensors report nothing below about 3 km/h, and asserting "straight" at
     * parking speeds is the most dangerous available error.
     *
     * Curvature and effective steering angle are absent by design: both are
     * this, @c speed and one vehicle constant away, so the host can derive them
     * and revise the wheelbase without reflashing the MKR.
     */
    int16_t yawRateCdps;
    /**
     * Per-wheel speeds in raw 0.01 km/h counts, FL FR RL RR.
     * @c 0xFFFF per channel when unavailable.
     *
     * Carried alongside the scalar @c speed because per-wheel speed is what
     * shows lockup, slip and ABS activity at the instant of an incident, which
     * a scalar cannot reconstruct afterwards.
     */
    uint16_t wheelRaw[4];
    /**
     * ---- panel switches (v0x07) ----
     *
     * Debounced positions behind the master's 74HC165 chain. Bit @e n is
     * @c SWn and 1 means CLOSED. Qualified by @c TLM_FLAG_SWITCHES_PRESENT:
     * when that is clear these are the last measured positions, not current
     * ones.
     *
     * DELIBERATELY UNNAMED, and this is the end of the chain where that pays
     * off. Neither MCU knows what any switch controls, and no firmware branches
     * on one — the mapping from index to function belongs in THIS
     * application's configuration, beside the CAN map, so a switch can be
     * re-purposed or moved to a different hole in the dash without reflashing
     * anything. Naming one here would undo that as surely as naming it in the
     * firmware.
     */
    uint16_t switchState;
    /**
     * Bits that changed at least once since the previous frame you received.
     *
     * A latching switch holds its own position, so @c switchState alone cannot
     * miss one — but a switch flipped and returned BETWEEN two frames is
     * invisible in level alone, and frames are genuinely lost to decimation and
     * to a full ring. Accumulated on the master, cleared only on a CONFIRMED
     * send, and OR-ed across any frames the bridge coalesced, so the guarantee
     * survives both hops rather than only the first.
     *
     * Also set when a chain that had gone away returns holding a different
     * position: something moved while nobody was looking, and the fact survives
     * even though the moment does not.
     */
    uint16_t switchChanged;
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
static_assert(sizeof(Telemetry)    == 180, "hostproto::Telemetry must be tightly packed to 180 bytes");
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
    case CMD_SET_CAN_MODE: return "CMD_SET_CAN_MODE";
    case CMD_SET_CAN_FILTER: return "CMD_SET_CAN_FILTER";
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
    case CMD_SET_CAN_MODE:
    case CMD_SET_CAN_FILTER:
    case CMD_SET_IMU_MODE:
    case CMD_PING:
        return true;
    default:
        return false;
    }
}

/// Filter slots @c CMD_SET_CAN_FILTER carries. Matches the master's MCP2515.
constexpr uint8_t CAN_FILTER_SLOTS   = 6;
/// uint8 count + 6 x uint16.
constexpr uint8_t SET_CAN_FILTER_LEN = 13;

/** @brief Expected payload length for a command TYPE; 0 for the zero-payload commands. */
inline uint8_t commandPayloadLen(uint8_t type)
{
    switch (type) {
    case CMD_SET_DECIM:
    case CMD_SET_CAN_MODE:
    case CMD_SET_IMU_MODE:   return 1;
    case CMD_SET_CAN_FILTER: return SET_CAN_FILTER_LEN;
    default:                 return 0;
    }
}

} // namespace hostproto

#endif // HOST_PROTOCOL_H
