#ifndef OBD2FUNCTIONS_H
#define OBD2FUNCTIONS_H 1

#include <cstdint>
#include <Arduino.h>
#include <SPI.h>
#include <CAN.h>
#include "DataDictionary.h"

// This module transmits on a live vehicle bus, so it depends on fixes that only
// exist in the vendored CAN library.  Building against the global Arduino copy
// would compile and link perfectly and then, on a bus that is not idle, spin
// inside CAN.endPacket() forever - which the watchdog turns into a reboot loop
// straight back into the same call, not a recovery.  It would also accept a DLC
// above 8 and overflow the driver's own 8-byte receive buffer into its SPI
// settings and pin numbers.  Neither failure is visible at build time, and the
// two library copies are similar enough that nothing would look wrong.
// See peripherals/mkr_zero/vendor/CANBus/PATCHES.md.
#ifndef DASHCAM_CANBUS_VENDORED_FIXES
#error "Stock arduino-CAN detected. Build with --library peripherals/mkr_zero/vendor/CANBus (see vendor/CANBus/PATCHES.md); upstream endPacket() has no deadline and parsePacket() overflows its RX buffer on DLC > 8."
#endif

/// Default timeout for OBD-II response polling (ms).
#define OBD2_TIMEOUT_MSEC 10000UL

/// SPI settings for the MCP2515 CAN controller: 10 MHz, MSB first, Mode 0.
const SPISettings SPICfg(10E6, MSBFIRST, SPI_MODE0);

/** Return codes used by OBD-II / CAN functions. */
enum class CANReturnStatus{
    OK = 0,              ///< Operation succeeded.
    BAD_CFG = 1,         ///< Invalid configuration passed.
    NOK_INIT_FAILED = -1,  ///< CAN module did not initialise.
    NOK_STATUS_BAD = -2,   ///< MCP2515 self-check failed.
    NOK_BAD_CMD = -3,      ///< Received response does not match the command sent.
    NOK_NULL_BUFFER = -4,  ///< Output buffer pointer is NULL.
    NOK_NOT_S1_CFG = -5,   ///< Rx address is not configured for Service 1.
    NOK_TIMEOUT = -6,      ///< No valid response within the timeout window.
    NOK_TX = -7            ///< A CAN transmit operation failed (arbitration / no ACK / bus-off).
};

/** Standard OBD-II 11-bit CAN transmit addresses. */
enum CAN_TxAddress{
    OBD2_TX_GLOBAL = 0x7DF, ///< Functional broadcast — all ECUs respond.
    OBD2_TX_ECM_1  = 0x7E0, ///< Physical address — ECM only.
};

/** Standard OBD-II 11-bit CAN receive addresses. */
enum CAN_RxAddress{
    OBD2_RX_ECM_1 = 0x7E8 ///< ECM response address.
};

/**
 * @brief OBD-II Mode 1 (current data) PID identifiers.
 * @see https://en.wikipedia.org/wiki/OBD-II_PIDs#Query
 */
enum OBD2_S1Command{
    NONE          = 0x00, ///< Placeholder / no command.
    RPM           = 0x0C, ///< Engine RPM.
    SPEED         = 0x0D, ///< Vehicle speed (km/h).
    /// Transmission actual gear — 4 bytes: A/B gear+support, C/D ratio.
    ///
    /// PID 0xA3 used to be polled here as "commanded gear". It is not: in the
    /// SAE J1979DA digital annex 0xA3 is a NINE-byte evaporative-system vapour
    /// pressure record. Two things followed from that. The decoder read byte 0
    /// of an evap-pressure record as a gear number, so @c gear was never once a
    /// gear; and a nine-byte response cannot fit a single ISO-TP frame, so it
    /// would arrive as a multi-frame transfer this poller cannot parse at all
    /// and would simply time out, costing one slot of every poll cycle.
    /// Both gear and ratio come from 0xA4.
    GEAR_RTIO     = 0xA4,
    AIR_PRES      = 0x33, ///< Barometric pressure (kPa).
    ODOMETER      = 0xA6, ///< Odometer reading (km).
    FUEL_LVL      = 0x2F, ///< Fuel tank level (%).
    ENGINE_TEMP   = 0x05, ///< Engine coolant temperature (°C).
    FUEL_RATE     = 0x5E, ///< Engine fuel rate (L/h).
    ENGINE_LOAD   = 0x04, ///< Calculated engine load (%).
    THROTTLE_POSN = 0x11  ///< Absolute throttle position (%).
};

/// Number of response data bytes expected for each PID.
/// These lengths and the scaling applied in @c storeReading() follow SAE J1979
/// Mode 01 (current data); see the SAE J1979DA Digital Annex for the governing table.
#define NONE_T          0
#define RPM_T           2
#define SPEED_T         1
#define GEAR_RTIO_T     4
#define AIR_PRES_T      1
#define ODOMETER_T      4
#define FUEL_LVL_T      1
#define ENGINE_TEMP_T   1
#define FUEL_RATE_T     2
#define ENGINE_LOAD_T   1
#define THROTTLE_POSN_T 1

/**
 * @brief Index into @c OBD2Data::fieldMs, one per published reading.
 *
 * Separate from @c OBD2_S1Command because the mapping is not one-to-one: PID
 * 0xA4 fills two fields (gear and ratio) from a single response.
 */
enum OBD2Field : uint8_t {
    OBD2F_RPM = 0,
    OBD2F_SPEED,
    OBD2F_COOLANT,
    OBD2F_FUEL_LVL,
    OBD2F_FUEL_RATE,
    OBD2F_THROTTLE,
    OBD2F_ENGINE_LOAD,
    OBD2F_AIR_PRES,
    OBD2F_GEAR,
    OBD2F_GEAR_RATIO,
    OBD2F_ODO,
    OBD2_FIELD_COUNT
};

/** Runtime configuration for a single OBD-II session. */
struct OBD2Config{
    CAN_TxAddress TxAddress;       ///< CAN ID used to transmit requests.
    CAN_RxAddress RxAddress;       ///< CAN ID expected in responses.
    uint32_t      supportedPIDs[7]; ///< Bitmask arrays from PID 0x00–0xC0.
};

/**
 * @brief Initialises the MCP2515 CAN controller and configures the OBD session.
 *
 * Sets CS/IRQ pins, starts the CAN bus at 500 kbps, stores the Tx/Rx
 * addresses in @p config, and installs a hardware filter for @p RxAddr.
 *
 * @param[out] config   OBD2Config to populate.
 * @param[in]  TxAddr   CAN ID to use when sending requests.
 * @param[in]  RxAddr   CAN ID to accept in responses.
 * @param[in]  csPin    SPI chip-select pin (default @c MCP2515_DEFAULT_CS_PIN).
 * @param[in]  irqPin   Interrupt pin (default @c MCP2515_DEFAULT_INT_PIN).
 * @return @c CANReturnStatus::OK on success, @c NOK_INIT_FAILED otherwise.
 */
CANReturnStatus initializeOBD2(OBD2Config &config, CAN_TxAddress TxAddr, CAN_RxAddress RxAddr, int csPin = MCP2515_DEFAULT_CS_PIN, int irqPin = MCP2515_DEFAULT_INT_PIN, bool stayInConfigurationMode = false);

/**
 * @brief Queries the ECU for its supported OBD-II PIDs and stores the result.
 *
 * Iterates through PID groups 0x00, 0x20, 0x40 … 0xC0 and populates
 * @c config.supportedPIDs[] with the 32-bit bitmasks returned by the ECU.
 *
 * @param[in,out] config          OBD2Config with Tx/Rx addresses already set.
 * @param[in]     timeoutInterval Per-group response timeout in milliseconds.
 * @return @c CANReturnStatus::OK, @c NOK_STATUS_BAD if the MCP2515
 *         self-check fails, or @c NOK_TIMEOUT if a group has no response.
 * @warning Disabled in @c initializeOBD2(), and it CANNOT simply be
 *          uncommented. It blocks: up to seven PID groups, each waiting
 *          @p timeoutInterval (default @c OBD2_TIMEOUT_MSEC = 10 s) inside a
 *          busy loop with no watchdog feed. Worst case is ~70 s against an 8 s
 *          watchdog, so a vehicle that does not answer group 0 turns every boot
 *          into a reset. Re-enabling it means converting it to the same
 *          non-blocking staged shape as @c gpsInitTick() first.
 */
CANReturnStatus getSupportedPIDs(OBD2Config &config, unsigned long timeoutInterval = OBD2_TIMEOUT_MSEC);

/**
 * @brief Transmits an OBD-II Mode 1 (current data) request frame.
 *
 * Sends a standard 8-byte ISO 15765-4 frame with the mode byte @c 0x01
 * and the given PID to @c config.TxAddress.
 *
 * @param[in] config   Active OBD2Config.
 * @param[in] command  PID to request (from @c OBD2_S1Command).
 * @return @c CANReturnStatus::OK on success, or @c NOK_TX if any CAN transmit
 *         operation failed (e.g. no ACK on a disconnected bus).
 *
 * @note This used to claim @c endPacket() was bounded by One-Shot Mode. It is
 *       not. OSM limits RE-transmission after an attempt; it does not bound
 *       waiting for the bus to go idle in the first place, so on a
 *       stuck-dominant bus upstream spun here forever. The bound now comes from
 *       an explicit deadline inside the vendored @c endPacket(), which is what
 *       the @c DASHCAM_CANBUS_VENDORED_FIXES guard above exists to enforce.
 */
CANReturnStatus sendS1Command(OBD2Config &config, const OBD2_S1Command command);

/**
 * @brief Waits for and reads an OBD-II Mode 1 response frame.
 *
 * Polls the CAN bus until a valid Service 1 response (0x41) with the
 * correct byte count is received, or the timeout expires.  Invalid or
 * unrelated frames are silently discarded.
 *
 * @param[in]  config       Active OBD2Config (must have @c OBD2_RX_ECM_1 set).
 * @param[out] outputBuffer Buffer to receive the raw data bytes.
 * @param[in]  bufferSize   Expected number of data bytes.
 * @param[out] commandRx    PID echoed back by the ECU.
 * @param[in]  timeout      Maximum wait time in milliseconds (default 1000).
 * @return @c CANReturnStatus::OK on success, @c NOK_NULL_BUFFER if
 *         @p outputBuffer is NULL, @c NOK_NOT_S1_CFG if the Rx address is
 *         not @c OBD2_RX_ECM_1, or @c NOK_TIMEOUT.
 */
CANReturnStatus receiveS1Command(OBD2Config &config, uint8_t *outputBuffer, byte bufferSize, OBD2_S1Command &commandRx, unsigned long timeout = 1000);

// ---- Non-blocking round-robin polling API ----

/// Per-PID response deadline for @c tickOBD2(); matches the ISO 15765-4 P2 maximum (ms).
#define OBD2_TICK_TIMEOUT_MS    50UL

/// A reading newer than this marks the OBD2 data "live" for telemetry (ms).
#define OBD2_FRESH_WINDOW_MS    1000UL

/// Retry throttle after a failed CAN transmit — prevents a dead-bus TX storm (ms).
#define OBD2_TX_BACKOFF_MS      50UL
/// Consecutive TX failures after which @c isOBD2LinkLost() reports the link down.
#define OBD2_TX_FAIL_LIMIT      20U

/**
 * Silence, in ms, after which the ECU is considered gone.
 *
 * A transmit "succeeding" only means some node on the bus ACKed the frame, and
 * on a vehicle bus something always will.  Without this, a wrong filter, a wrong
 * response ID or a powered-down ECM produced an endless run of clean sends and
 * silent timeouts while @c isOBD2LinkLost() stayed false forever — the poller
 * reported a healthy link to an ECU that was not there.
 *
 * A full pipeline sweep is ~10 x @c OBD2_TICK_TIMEOUT_MS, so this is several
 * complete cycles of total silence: long enough that a busy ECU dropping one
 * request cannot trip it.
 */
#define OBD2_RX_SILENCE_MS      3000UL

/// How long a link that has NEVER answered stays credible, measured from the
/// first request that reached the wire. Longer than OBD2_RX_SILENCE_MS on
/// purpose: an ECU can take a moment to start answering after ignition, and
/// this deadline is the only thing that ever fires when a live bus ACKs every
/// request while the addressed ECU says nothing.
#define OBD2_NO_REPLY_MS        8000UL

/**
 * Floor on the gap between consecutive requests.
 *
 * Previously the next request went out the instant a reply was decoded, which
 * on a healthy ECU meant ~20 functional broadcasts per second at 0x7DF. Real
 * scan tools pace themselves; flooding a live vehicle bus with diagnostic
 * requests is exactly the kind of guest behaviour that gets noticed.
 */
#define OBD2_MIN_REQUEST_GAP_MS 20UL

/**
 * @brief Latest OBD-II sensor readings maintained by the non-blocking @c tickOBD2() poller.
 *
 * Fields that have not yet been received from the ECU hold @c NAN.
 * Initialise with @c initOBD2Data() before the first @c tickOBD2() call.
 */
struct OBD2Data {
    float rpm;         ///< Engine speed (RPM) — PID @c RPM.
    float speed;       ///< Vehicle speed (km/h) — PID @c SPEED.
    float coolantTemp; ///< Coolant temperature (°C) — PID @c ENGINE_TEMP.
    float fuelLevel;   ///< Fuel tank level (%) — PID @c FUEL_LVL.
    float fuelRate;    ///< Fuel consumption rate (L/h) — PID @c FUEL_RATE.
    float throttle;    ///< Throttle plate position (%) — PID @c THROTTLE_POSN.
    float engineLoad;  ///< Calculated engine load (%) — PID @c ENGINE_LOAD.
    float airPressure; ///< Barometric pressure (kPa) — PID @c AIR_PRES.
    float gear;        ///< Transmission actual gear — PID @c GEAR_RTIO byte B.
    float gearRatio;   ///< Transmission gear ratio — PID @c GEAR_RTIO.
    float odo;         ///< Odometer reading (km) — PID @c ODOMETER.
    uint32_t lastUpdateMs; ///< @c millis() when @c tickOBD2() last stored ANY reading (0 = never).

    /**
     * Per-field @c millis() stamps, indexed by @c OBD2Field. 0 = never seen.
     *
     * @c lastUpdateMs alone cannot express what a round-robin poller actually
     * knows.  One cycle takes up to 550 ms and any single reply refreshed it,
     * so a live RPM response certified a speed reading ten PIDs stale as
     * current — and @c COMM_FLAG_OBD2_VALID then vouched for the entire
     * payload.  A frozen speed is the worst case: it fed the acceleration
     * estimator over and over, which reads as a real deceleration to zero.
     *
     * Kept ALONGSIDE @c lastUpdateMs rather than replacing it, because the two
     * answer different questions: "is this number current" and "is the ECU
     * answering at all".
     */
    uint32_t fieldMs[OBD2_FIELD_COUNT];
};

/**
 * @brief Overwrites any field older than @p maxAgeMs with @c NAN.
 *
 * Call immediately before publishing.  Staleness has to become absence at the
 * boundary, because @c NAN is the only "unavailable" every downstream consumer
 * already handles; anything else needs every one of them to remember a rule.
 *
 * @param[in,out] data      Readings to expire in place.
 * @param[in]     maxAgeMs  Age at which a reading stops counting as current.
 */
void expireStaleOBD2Fields(OBD2Data &data, uint32_t maxAgeMs = OBD2_FRESH_WINDOW_MS);

/**
 * @brief Sets all fields of @p data to @c NAN.
 *
 * Call once before the first @c tickOBD2() so callers can distinguish
 * "not yet received" from a legitimate zero reading.
 *
 * @param[out] data  @c OBD2Data struct to initialise.
 */
void initOBD2Data(OBD2Data &data);

/**
 * @brief Returns whether a PID is listed as supported in @c config.supportedPIDs.
 *
 * @c getSupportedPIDs() must have been called first to populate the bitmask.
 * Returns @c false for @c NONE and for PIDs above @c 0xE0.
 *
 * @param[in] config   @c OBD2Config with populated @c supportedPIDs bitmask.
 * @param[in] command  PID to query.
 * @return @c true if the ECU reported the PID as supported.
 */
bool isCommandSupported(const OBD2Config &config, const OBD2_S1Command command);

/**
 * @brief Resets the non-blocking OBD-II polling state machine to idle.
 *
 * Call when re-initialising the CAN bus or to temporarily suspend polling.
 * Also clears the consecutive-TX-failure counter used by @c isOBD2LinkLost().
 */
void resetOBD2Poll();

/**
 * @brief Reports whether the CAN link has failed at runtime.
 *
 * Returns @c true once @c tickOBD2() has seen @c OBD2_TX_FAIL_LIMIT consecutive
 * transmit failures (e.g. the ECU/bus was disconnected after a good init). The
 * application should then clear its "OBD ready" flag and re-initialise. Reset by
 * @c resetOBD2Poll() (and thus by a successful @c initializeOBD2() + reset).
 *
 * @return @c true when the transmit path has been failing continuously.
 */
bool isOBD2LinkLost();

/**
 * @brief Whether any ECU has EVER answered a request since the last reset.
 *
 * Distinct from "the controller initialised", which is all an @c obdReady flag
 * proves.  A status line that reports OBD-II as "up" on the strength of a
 * successful @c initializeOBD2() claims a working diagnostic link on a vehicle
 * that may never have said a word — which is exactly the reading that sends
 * someone hunting for a decode bug when the real answer is that no reply ever
 * arrived.
 *
 * @return @c true once at least one reply has been decoded.
 */
bool obd2EverReplied();

/**
 * @brief Advances the non-blocking round-robin OBD-II polling state machine by one step.
 *
 * Call once per @c loop() iteration.  Each call does exactly one of:
 * - fires the next PID request and returns immediately (no blocking);
 * - processes an ECU reply already in the MCP2515 RX buffer, stores the
 *   decoded value into @p data, then fires the following request; or
 * - skips the current PID and fires the next request if @c OBD2_TICK_TIMEOUT_MS
 *   elapsed without a reply.
 *
 * Polling order: @c RPM → @c SPEED → @c ENGINE_TEMP → @c FUEL_LVL →
 * @c FUEL_RATE → @c THROTTLE_POSN → @c ENGINE_LOAD → @c AIR_PRES →
 * @c GEAR_RTIO → @c ODOMETER, then repeats.
 *
 * @param[in,out] config  Active @c OBD2Config; Tx/Rx addresses must be set.
 * @param[out]    data    @c OBD2Data struct; updated when a valid reply arrives.
 * @return @c true if a new sensor value was written into @p data this call.
 */
bool tickOBD2(OBD2Config &config, OBD2Data &data);

#endif
