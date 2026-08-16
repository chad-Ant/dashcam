#ifndef COMMUNICATION_FUNCTIONS
#define COMMUNICATION_FUNCTIONS 1

#include <Arduino.h>
#include <math.h>
#include "CommProtocol.h"
#include "VehicleSignals.h"
#include "DataDictionary.h"
#include "IMUFunctions.h"   // IMUData, carried in the telemetry payload

/// GPS uses I2C on the MKR Zero, leaving @c Serial1 free for the ESP32-C3 link.

// Forward declarations keep this header decoupled: the telemetry builder takes
// these by reference, which is legal on an incomplete type. Full definitions
// (which drag in ExternalLibConfig.h) are pulled only into the .cpp.
struct OBD2Data;
struct GPSData;

/**
 * @brief Signals computed on the master rather than read from a sensor.
 *
 * Grouped into a struct rather than passed as loose floats so adding the next
 * derived quantity does not change the signature of three functions again.
 *
 * Every field is @c NAN when its estimator has not warmed up or its input is
 * untrustworthy — never a plausible-looking substitute, so a consumer can tell
 * "not available" from a real measurement.
 */
struct DerivedSignals {
    float accelMs2   = NAN; ///< Longitudinal acceleration (m/s²) from @c AccelerationEstimator.
    float headingDeg = NAN; ///< Filtered course over ground (deg) from @c CircularMovingAverage.
};

/**
 * @brief Copies a sub-sequence of bytes from a C-string into an output buffer.
 *
 * Extracts @p byteLength bytes starting at @p byteOffset from @p input and
 * writes them into @p outputBuffer, always null-terminating the result.
 * If the requested range extends past the end of @p input the copy is
 * silently truncated to the available data.  The output buffer is zeroed
 * before the copy.
 *
 * @param[in]  input              Source C-string (must not be NULL).
 * @param[out] outputBuffer       Destination buffer (must not be NULL).
 * @param[in]  outputBufferLength Total capacity of @p outputBuffer including
 *                                the null terminator.
 * @param[in]  byteLength         Number of bytes to copy from @p input.
 * @param[in]  byteOffset         Zero-based start offset within @p input.
 * @return @c true on success, @c false if any pointer is NULL, the offset
 *         is out-of-range, or either length is zero.
 */
bool splitByte(const char* input, char* outputBuffer, const size_t outputBufferLength, size_t byteLength, size_t byteOffset);

/**
 * @brief Opens the ESP32-C3 UART link on @c Serial1.
 *
 * @param[in] baud  Line rate (default @c ESP32_UART_BAUD from DataDictionary.h).
 */
void initializeComm(unsigned long baud = ESP32_UART_BAUD);

/**
 * @brief Frames @p payload with the wire header + CRC and writes it to @c Serial1.
 *
 * @param[in] type     Message type (see @c CommMsgType).
 * @param[in] payload  Payload bytes (may be NULL when @p len is 0).
 * @param[in] len      Payload length in bytes.
 * @return @c OK, @c NOK_OVERFLOW if framing failed, or @c NOK_BUSY if the TX
 *         buffer lacked room for the whole frame (nothing sent — non-blocking).
 */
CommReturnStatus sendFrame(uint8_t type, const uint8_t *payload, uint8_t len);

/**
 * @brief Packs the current OBD2 and GPS readings into a @c TelemetryPayload.
 *
 * Copies fields verbatim (NAN passes through) and sets @c flags from data
 * validity: @c COMM_FLAG_OBD2_VALID, @c COMM_FLAG_GPS_FIX, @c COMM_FLAG_TIME_VALID.
 *
 * @param[in]  obd       Latest OBD2 readings (from @c tickOBD2()).
 * @param[in]  gps       Latest GPS snapshot (from @c getGPSData()).
 * @param[in]  derived   Master-computed signals (acceleration, filtered heading).
 *                       Passed in rather than read from @c OBD2Data / @c GPSData
 *                       because these are derived, not sensor readings.
 * @param[out] out       Telemetry struct to fill.
 */
void buildTelemetry(const OBD2Data &obd, const GPSData &gps, const IMUData &imu, const DerivedSignals &derived, const VehicleSignals &veh, uint8_t canMode, TelemetryPayload &out);

/**
 * @brief Builds and transmits one @c MSG_TELEMETRY frame on @c Serial1.
 *
 * @return @c CommReturnStatus::OK, or @c NOK_OVERFLOW if framing failed.
 */
CommReturnStatus sendTelemetry(const OBD2Data &obd, const GPSData &gps, const IMUData &imu, const DerivedSignals &derived, const VehicleSignals &veh, uint8_t canMode);

/**
 * @brief Runtime state for the master link: streaming toggle + RX decoder.
 *
 * Initialise with @c initCommMaster() before the first @c tickCommMaster().
 */
struct CommMaster {
    bool          streaming;     ///< True while pushing telemetry at 10 Hz.
    unsigned long lastPushMs;    ///< @c millis() of the last streamed frame.
    bool          oncePending;   ///< A @c CMD_GET_ONCE whose reply has not gone out yet.
    unsigned long onceRequestMs; ///< @c millis() when that request arrived (expiry clock).
    CommRxState   rx;            ///< Incremental inbound-command decoder.
    /**
     * @c millis() when the last valid command frame arrived from the C3.
     *
     * The MKR is the responder on this hop, so inbound frames are the ONLY
     * evidence the bridge is still there.  Without this the master happily
     * streams 10 Hz telemetry into an unplugged cable forever, reporting
     * nothing: TX into a disconnected UART completes normally, so a silent link
     * is indistinguishable from a healthy one unless arrival is tracked.
     * 0 = nothing has ever been received.
     */
    unsigned long lastCommandMs;
    /**
     * A CMD_SET_CAN_MODE the sketch has not acted on yet. 0 = none pending.
     *
     * Recorded rather than executed here: this module owns the wire, not the
     * CAN controller, and calling canSetMode() from inside the frame decoder
     * would drag a bus-mode transition - which passes through Configuration
     * mode and drops frames - into the middle of servicing a command budget.
     * The sketch applies it at a point of its own choosing.
     */
    uint8_t       canModeRequest;
    /**
     * A CMD_SET_IMU_MODE the sketch has not acted on yet. 0 = none pending.
     *
     * Latched for the same reason as @c canModeRequest and more urgently:
     * applying it calls setIMUSampleMode(), which restarts the sensor's entire
     * bring-up. That is roughly 700 ms during which the IMU publishes nothing
     * and the trailing peak window is discarded — far too much work to do from
     * inside the frame decoder, which is servicing a bounded command budget and
     * must return promptly to the loop.
     *
     * Values are @c COMM_IMU_MODE_FUSION and @c COMM_IMU_MODE_RAW, validated at
     * the wire boundary so an out-of-range byte is NACKed rather than reaching
     * the driver.
     */
    uint8_t       imuModeRequest;
    /**
     * @c millis() the last IMU mode change was applied. 0 = none this boot.
     *
     * The rate limiter's state. Without one, a host looping on the command
     * would hold the sensor in permanent re-initialisation and it would never
     * produce a sample again — every request individually reasonable, the
     * aggregate a denial of the sensor.
     */
    unsigned long imuModeAppliedMs;
    /**
     * A CMD_SET_CAN_FILTER the sketch has not acted on yet.
     *
     * Latched for the same reason as @c canModeRequest, and more strongly: the
     * MCP2515 cannot have its filter registers written while receiving, so
     * applying this means dropping into Configuration mode and back. Doing that
     * inside the frame decoder would put a receive gap in the middle of command
     * servicing, where nothing is watching for one.
     *
     * @c canFilterPending is a separate flag rather than "count != 0" because
     * ZERO IS A MEANINGFUL REQUEST: on an MCP2515 no filters means accept all,
     * so a host clearing the filter set to widen a capture sends count 0, and
     * treating that as "nothing pending" would silently discard the command.
     */
    bool          canFilterPending;
    uint8_t       canFilterCount;
    uint16_t      canFilterIds[COMM_CAN_FILTER_SLOTS];
};

/**
 * @brief True when no command has arrived from the C3 for @p timeoutMs.
 *
 * Reports only; the master keeps streaming regardless, because a bridge that
 * reboots and re-issues CMD_START_STREAM must find the link exactly as it left
 * it.  This exists so the silence is visible in the log rather than silently
 * normal.
 */
bool isCommLinkSilent(const CommMaster &m, unsigned long timeoutMs);

/** @brief Resets a @c CommMaster to idle with a clean decoder. */
void initCommMaster(CommMaster &m);

/**
 * @brief One non-blocking master step. Call every @c loop() iteration.
 *
 * Services queued C3 commands (@c CMD_GET_ONCE replies once, @c CMD_START_STREAM /
 * @c CMD_STOP_STREAM toggle streaming, @c CMD_PING replies @c MSG_PONG, unknown →
 * @c MSG_NACK) and, while streaming, pushes a telemetry frame every
 * @c COMM_STREAM_INTERVAL_MS (10 Hz).
 *
 * @param[in,out] m         Master state.
 * @param[in]     obd       Latest OBD2 readings to publish.
 * @param[in]     gps       Latest GPS snapshot to publish.
 * @param[in]     derived   Latest master-computed signals.
 */
void tickCommMaster(CommMaster &m, const OBD2Data &obd, const GPSData &gps, const IMUData &imu, const DerivedSignals &derived, const VehicleSignals &veh, uint8_t canMode);

#endif
