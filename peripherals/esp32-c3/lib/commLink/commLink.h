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

    /**
     * Flags that must survive frame coalescing rather than be overwritten by it.
     *
     * poll() can accept up to COMMLINK_MAX_FRAMES_PER_POLL master frames in one
     * call and the forwarder sends at most one, so seven snapshots can be
     * discarded per pass — and the host's decimation setting discards more. For
     * a value that describes an INSTANT, keeping the newest is right. For a flag
     * that says AN EVENT OCCURRED, keeping the newest is data loss: the impact
     * was real and the frame carrying it was thrown away for being early.
     *
     * A forwarded frame stands for every master frame since the last one
     * forwarded, so OR-ing these across that interval is not a distortion — it
     * is what the frame already claims to represent.
     */
    static constexpr uint16_t kStickyFlags =
        COMM_FLAG_IMU_HIGH_G | COMM_FLAG_IMU_DATA_GAP | COMM_FLAG_IMU_SATURATED;

    /**
     * @brief Folds the accumulated event flags and window maxima into @p out.
     *
     * Applied to the snapshot on its way upward. The peaks are maxima over the
     * coalesced interval for the same reason the flags are unions of it: the
     * newest frame's peak describes 100 ms of a window the forwarded frame
     * claims to cover in full.
     */
    void mergeCoalesced(TelemetryPayload &out) const;

    /** @brief True when an event flag is waiting that decimation must not delay. */
    bool hasUrgent() const { return (coalescedFlags_ & COMM_FLAG_IMU_HIGH_G) != 0u; }

    /**
     * @brief Discards the accumulator. Call ONLY after a successful send.
     *
     * Clearing on the attempt is what made a full TX ring lose an impact: the
     * frame was dropped, the evidence was cleared with it, and the next frame
     * reported a quiet window.
     */
    void clearCoalesced();

    /** @brief True if no telemetry arrived within the last @p ms milliseconds. */
    bool isStale(uint32_t ms) const { return !hasData_ || (millis() - lastRxMs_) > ms; }

    /** @brief Returns and clears the "PONG received" flag. */
    bool pongSeen() { bool p = pong_; pong_ = false; return p; }

    /** @brief Frames discarded on CRC mismatch since begin() — cable/EMI health. */
    uint32_t crcErrors() const { return crcErrors_; }

    /** @brief Telemetry frames accepted since begin(). */
    uint32_t framesRx() const { return framesRx_; }

    /**
     * @brief Asks the MKR to switch CAN mode (1=discover 2=sniff 3=obd2).
     * @return false if the value is out of range or the MKR's TX buffer is full.
     */
    bool setCanMode(uint8_t mode);

    /**
     * @brief Asks the MKR to switch IMU mode (1 = IMUPLUS fusion, 2 = AMG raw).
     *
     * ⚠️ The master goes blind for about 700 ms applying this and discards its
     * trailing peak window, so it is a session-level decision — never something
     * to send on an event. The master also rate-limits it and will refuse one
     * that arrives too soon after the last.
     *
     * @return false if the value is out of range or the MKR's TX buffer is full.
     */
    bool setImuMode(uint8_t mode);

    /**
     * @brief Relays a filter set to the master. @return false if TX was busy.
     *
     * @p count of 0 is valid and means ACCEPT ALL — an MCP2515 with a zero mask
     * compares no bits. Rejecting it as "empty" would remove the only way a host
     * can widen a capture.
     */
    bool setCanFilter(const uint16_t *ids, uint8_t count);

private:
    bool sendCmd(uint8_t type, const uint8_t *payload = nullptr, uint8_t len = 0);

    /** @brief Folds one accepted frame into the coalescing accumulator. */
    void accumulate(const TelemetryPayload &src);

    HardwareSerial  *uart_ = nullptr;
    CommRxState      rx_;
    TelemetryPayload latest_ = {};
    bool             hasData_ = false;
    bool             pong_ = false;
    uint32_t         lastRxMs_ = 0;
    uint32_t         crcErrors_ = 0;
    uint32_t         framesRx_ = 0;

    /// Union of kStickyFlags over every frame accepted since the last successful
    /// forward, and the maxima to match. Held here rather than folded into
    /// latest_ so the raw newest snapshot stays available unmodified.
    uint16_t coalescedFlags_ = 0;
    /**
     * OR of switchChanged across every frame coalesced behind the newest one.
     *
     * Carried for the same reason the sticky flags are. The master clears this
     * word only on a confirmed send, so it correctly describes the interval
     * since the last frame IT transmitted - but the bridge then drops some of
     * those frames to decimation or to a full host ring, and the one that
     * survives carries only its own interval. A switch flipped during a
     * discarded frame would vanish between the two hops, which is precisely the
     * gap the master's clear-on-confirmed-send discipline exists to close.
     */
    uint16_t coalescedSwitchChanged_ = 0;
    float    coalescedAccelPeak_ = NAN;
    float    coalescedGyroPeak_ = NAN;
    float    coalescedLinAccelPeak_ = NAN;
};
