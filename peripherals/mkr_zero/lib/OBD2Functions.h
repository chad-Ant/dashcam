#ifndef OBD2FUNCTIONS_H
#define OBD2FUNCTIONS_H 1

#include <cstdint>
#include <Arduino.h>
#include <SPI.h>
#include "DataDictionary.h"

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
    GEAR_CMD      = 0xA3, ///< Commanded gear (0 = Park/Neutral, 1–8 = gears 1–8).
    GEAR_RTIO     = 0xA4, ///< Transmission actual gear ratio.
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
#define GEAR_CMD_T      1
#define GEAR_RTIO_T     4
#define AIR_PRES_T      1
#define ODOMETER_T      4
#define FUEL_LVL_T      1
#define ENGINE_TEMP_T   1
#define FUEL_RATE_T     2
#define ENGINE_LOAD_T   1
#define THROTTLE_POSN_T 1

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
CANReturnStatus initializeOBD2(OBD2Config &config, CAN_TxAddress TxAddr, CAN_RxAddress RxAddr, int csPin = MCP2515_DEFAULT_CS_PIN, int irqPin = MCP2515_DEFAULT_INT_PIN);

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
 * @note Currently disabled in @c initializeOBD2() pending validation.
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
 *         operation failed (e.g. no ACK on a disconnected bus). @c endPacket()
 *         is bounded because One-Shot Mode is enabled in @c initializeOBD2().
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
    float gear;        ///< Commanded gear (0 = Park/Neutral, 1–8) — PID @c GEAR_CMD.
    float gearRatio;   ///< Transmission gear ratio — PID @c GEAR_RTIO.
    float odo;         ///< Odometer reading (km) — PID @c ODOMETER.
    uint32_t lastUpdateMs; ///< @c millis() when @c tickOBD2() last stored a reading (0 = never), for liveness checks.
};

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
