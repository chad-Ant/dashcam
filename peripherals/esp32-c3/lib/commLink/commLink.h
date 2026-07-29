#pragma once

/**
 * @file commLink.h
 * @brief ESP32-C3 side of the telemetry link. Receives OBD2+GPS telemetry from
 *        the MKR Zero master over UART and drives the request/stream commands.
 *
 * Uses a hardware UART (default @c Serial1) on remappable GPIOs so the native
 * USB-CDC @c Serial stays free for the console. Fully non-blocking.
 *
 * The baud rate MUST match @c ESP32_UART_BAUD on the master (115200).
 */

#include <Arduino.h>
#include "CommProtocol.h"

/// Default UART1 pins — safe on the C3 (not flash GPIO12–17, not USB GPIO18/19,
/// not strapping GPIO2/8/9, not ADC GPIO0–4 reserved by powerManager). GPIO20/21
/// are UART0's pins but are free when the console runs over USB-CDC.
#ifndef COMMLINK_RX_PIN
#define COMMLINK_RX_PIN 20
#endif
#ifndef COMMLINK_TX_PIN
#define COMMLINK_TX_PIN 21
#endif
#ifndef COMMLINK_BAUD
#define COMMLINK_BAUD 115200UL
#endif
#ifndef COMMLINK_MAX_FRAMES_PER_POLL
#define COMMLINK_MAX_FRAMES_PER_POLL 8u  ///< RX budget per poll() call (bounds work per loop).
#endif

/** @brief Non-blocking receiver + command sender for the master telemetry link. */
class CommLink {
public:
    /**
     * @brief Opens the UART and resets the decoder.
     * @param baud   Line rate; must match the master (default @c COMMLINK_BAUD).
     * @param rxPin  UART RX GPIO, cross-wired to the master's TX.
     * @param txPin  UART TX GPIO, cross-wired to the master's RX.
     * @param uart   Hardware serial to use (default @c Serial1).
     */
    void begin(unsigned long baud = COMMLINK_BAUD,
               int8_t rxPin = COMMLINK_RX_PIN,
               int8_t txPin = COMMLINK_TX_PIN,
               HardwareSerial &uart = Serial1);

    /**
     * @brief Drains the UART, decoding frames. Non-blocking.
     * @return @c true if a fresh @c MSG_TELEMETRY updated @c latest() this call.
     */
    bool poll();

    bool requestOnce();  ///< Ask the master for a single telemetry frame. @return true if sent.
    bool startStream();  ///< Ask the master to push telemetry at 10 Hz. @return true if sent.
    bool stopStream();   ///< Ask the master to stop streaming. @return true if sent.
    bool ping();         ///< Send a link-check ping (expect a PONG). @return true if sent.

    const TelemetryPayload &latest() const { return latest_; } ///< Most recent telemetry.
    bool hasData() const { return hasData_; }                  ///< True once any telemetry arrived.

    /** @brief True if no telemetry arrived within the last @p ms milliseconds. */
    bool isStale(uint32_t ms) const { return !hasData_ || (millis() - lastRxMs_) > ms; }

    /** @brief Returns and clears the "PONG received" flag. */
    bool pongSeen() { bool p = pong_; pong_ = false; return p; }

    /** @brief Frames discarded on CRC mismatch since begin() — cable/EMI health. */
    uint32_t crcErrors() const { return crcErrors_; }

    /** @brief Telemetry frames accepted since begin(). */
    uint32_t framesRx() const { return framesRx_; }

private:
    bool sendCmd(uint8_t type);

    HardwareSerial  *uart_ = nullptr;
    CommRxState      rx_;
    TelemetryPayload latest_ = {};
    bool             hasData_ = false;
    bool             pong_ = false;
    uint32_t         lastRxMs_ = 0;
    uint32_t         crcErrors_ = 0;
    uint32_t         framesRx_ = 0;
};
