#ifndef COMMUNICATION_FUNCTIONS
#define COMMUNICATION_FUNCTIONS 1

#include <Arduino.h>
#include "CommProtocol.h"
#include "DataDictionary.h"

/// GPS uses I2C on the MKR Zero, leaving @c Serial1 free for the ESP32-C3 link.

// Forward declarations keep this header decoupled: the telemetry builder takes
// these by reference, which is legal on an incomplete type. Full definitions
// (which drag in ExternalLibConfig.h) are pulled only into the .cpp.
struct OBD2Data;
struct GPSData;

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
 * @param[in]  obd  Latest OBD2 readings (from @c tickOBD2()).
 * @param[in]  gps  Latest GPS snapshot (from @c getGPSData()).
 * @param[out] out  Telemetry struct to fill.
 */
void buildTelemetry(const OBD2Data &obd, const GPSData &gps, TelemetryPayload &out);

/**
 * @brief Builds and transmits one @c MSG_TELEMETRY frame on @c Serial1.
 *
 * @return @c CommReturnStatus::OK, or @c NOK_OVERFLOW if framing failed.
 */
CommReturnStatus sendTelemetry(const OBD2Data &obd, const GPSData &gps);

/**
 * @brief Runtime state for the master link: streaming toggle + RX decoder.
 *
 * Initialise with @c initCommMaster() before the first @c tickCommMaster().
 */
struct CommMaster {
    bool          streaming;   ///< True while pushing telemetry at 10 Hz.
    unsigned long lastPushMs;  ///< @c millis() of the last streamed frame.
    CommRxState   rx;          ///< Incremental inbound-command decoder.
};

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
 * @param[in,out] m    Master state.
 * @param[in]     obd  Latest OBD2 readings to publish.
 * @param[in]     gps  Latest GPS snapshot to publish.
 */
void tickCommMaster(CommMaster &m, const OBD2Data &obd, const GPSData &gps);

#endif
