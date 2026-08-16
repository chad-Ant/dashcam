#pragma once

/**
 * @file hostLink.h
 * @brief ESP32-C3 side of the Jetson link — the upward half of the bridge.
 *
 * Runs the @c hostproto wire contract over the C3's **native USB-C** connector
 * (the built-in USB Serial/JTAG on GPIO18/19), which plugs straight into a USB
 * port on the Jetson Orin Nano and enumerates there as @c /dev/ttyACM*.
 *
 * This is the mirror image of @c commLink.h:
 *   - @c CommLink is the C3 acting as *initiator* toward the MKR Zero.
 *   - @c HostLink is the C3 acting as *responder* toward the Jetson.
 * Same framing, same CRC, same bounded non-blocking poll — so both hops fail the
 * same way and can be reasoned about together.
 *
 * ── Board settings (required) ────────────────────────────────────────────────
 *   Tools -> USB CDC On Boot = **Enabled**
 * With that set, @c Serial is the native USB CDC (class @c HWCDC) and UART0's
 * GPIO20/21 are free — which is exactly what @c CommLink uses for the MKR hop.
 * The two links therefore coexist with no pin conflict:
 *
 *   MKR Zero ──UART1 GPIO20/21──> [ESP32-C3] ──USB-C native──> Jetson Orin Nano
 *
 * ── Serial is a binary channel now ───────────────────────────────────────────
 * Because the protocol owns @c Serial, **nothing may print to it**.  A stray
 * @c Serial.println() injects bytes mid-frame; the CRC would catch it, but the
 * frame is lost either way.  Use @c sendLog() instead — it wraps text in a
 * @c MSG_LOG frame that the Jetson feeds into liblog.
 *
 * ── Connection state is frame-driven, not CDC-driven ─────────────────────────
 * @c HWCDC::operator bool() has changed meaning across Arduino-ESP32 releases,
 * so this class does **not** use it.  The host is considered present while a
 * valid frame has arrived within @c HOSTLINK_HOST_TIMEOUT_MS; the Jetson keeps
 * that true with a 1 Hz @c CMD_PING keepalive.  TX safety needs no connection
 * flag either: writes are gated on @c availableForWrite() with a zero TX
 * timeout, so an absent or stalled host costs a dropped frame and a counter
 * bump, never a blocked @c loop().
 *
 * Session *start* is not inferred from that timeout, though — it cannot be, for
 * a host that reopens the port faster than the timeout, because there is no
 * silence to observe.  The host announces itself with @c CMD_HELLO on every
 * open, and that is what resets the session.  The timeout only detects the end.
 */

#include <Arduino.h>
#include "HostProtocol.h"

// Serial must be the native USB CDC for this link to exist at all.  Failing at
// compile time beats shipping firmware that silently talks to UART0 pins that
// commLink is already driving.
#if !defined(ARDUINO_USB_CDC_ON_BOOT) || (ARDUINO_USB_CDC_ON_BOOT == 0)
#error "hostLink requires 'USB CDC On Boot = Enabled' (or -DARDUINO_USB_CDC_ON_BOOT=1): Serial must be the native USB port wired to the Jetson."
#endif

#ifndef HOSTLINK_BAUD
#define HOSTLINK_BAUD 115200UL  ///< Ignored by native USB CDC (no line rate); set for tooling that asks.
#endif
#ifndef HOSTLINK_HOST_TIMEOUT_MS
#define HOSTLINK_HOST_TIMEOUT_MS 5000UL ///< Declare the host gone after this much frame silence.
#endif
#ifndef HOSTLINK_MAX_FRAMES_PER_POLL
#define HOSTLINK_MAX_FRAMES_PER_POLL 8u ///< Frame budget per poll() (bounds work per loop).
#endif
#ifndef HOSTLINK_MAX_BYTES_PER_POLL
#define HOSTLINK_MAX_BYTES_PER_POLL 512u ///< Byte budget per poll(); a flood of noise cannot scan forever.
#endif

/**
 * Assumed usable size of the USB CDC TX ring, in bytes.
 *
 * The Arduino core creates that ring itself when `USB CDC On Boot = Enabled`
 * (it begins @c Serial before setup() runs), and @c setTxBufferSize() is a
 * no-op once the ring exists — so this class cannot enlarge it and must not
 * pretend otherwise.  256 is the core's default and the safe lower bound.
 *
 * Every frame this class emits must fit, because sendFrame() refuses to write
 * a frame larger than the free space: a frame bigger than the whole ring could
 * never be sent at all, silently, forever.
 */
#ifndef HOSTLINK_TX_RING_ASSUMED
#define HOSTLINK_TX_RING_ASSUMED 256u
#endif

/**
 * Longest MSG_LOG text, in characters.
 *
 * Capped well below @c hostproto::MAX_LOG_TEXT (254) so the resulting frame
 * always fits @c HOSTLINK_TX_RING_ASSUMED.  A 254-character line would build a
 * 261-byte frame that can never fit a 256-byte ring — the log would vanish with
 * nothing but a dropped-frame counter to show for it.
 */
#ifndef HOSTLINK_MAX_LOG_TEXT
#define HOSTLINK_MAX_LOG_TEXT 160u
#endif

static_assert(HOSTLINK_MAX_LOG_TEXT <= hostproto::MAX_LOG_TEXT,
              "HOSTLINK_MAX_LOG_TEXT exceeds what one MSG_LOG frame can carry");

/// Largest frame this class can emit.  MSG_LOG is the worst case; telemetry is
/// 85 bytes and status 50, both comfortably smaller.
static constexpr size_t HOSTLINK_LARGEST_FRAME =
    hostproto::FRAME_OVERHEAD + sizeof(hostproto::LogHeader) + HOSTLINK_MAX_LOG_TEXT;

static_assert(HOSTLINK_LARGEST_FRAME <= HOSTLINK_TX_RING_ASSUMED,
              "A frame larger than the TX ring can never be sent: lower HOSTLINK_MAX_LOG_TEXT");

/** @brief Non-blocking responder for the Jetson USB link. */
class HostLink {
public:
    /**
     * @brief Opens the native USB CDC port and resets the decoder.
     *
     * Sets a **zero** TX timeout so @c Serial.write() can never block this task
     * when the host stops reading.  It deliberately does not resize the CDC
     * rings — see @c HOSTLINK_TX_RING_ASSUMED for why that cannot work here.
     */
    void begin();

    /**
     * @brief Services inbound commands. Non-blocking; call every @c loop().
     *
     * Handles @c CMD_PING (replies @c MSG_PONG), the stream toggles and
     * @c CMD_SET_DECIM internally, records @c CMD_GET_ONCE / @c CMD_GET_STATUS
     * for the sketch to act on, and NACKs anything malformed or unknown.
     * Bounded by @c HOSTLINK_MAX_BYTES_PER_POLL and @c HOSTLINK_MAX_FRAMES_PER_POLL.
     *
     * @param nowMs @c millis() sample for this pass.
     * @return @c true if at least one command frame was serviced.
     */
    bool poll(uint32_t nowMs);

    /** @brief True while the host is considered present (a frame arrived recently). */
    bool isConnected() const { return hostAlive_; }

    /**
     * @brief Returns and clears the "host just (re)appeared" flag.
     *
     * The sketch answers a true here by sending @c MSG_HELLO.  Streaming and
     * decimation have already been reset to their power-on defaults, so the
     * host must re-issue @c CMD_START_STREAM — that is the documented contract.
     */
    bool connectSeen() { const bool c = connectSeen_; connectSeen_ = false; return c; }

    bool onceRequested()   { const bool r = onceReq_;   onceReq_   = false; return r; } ///< Consume a @c CMD_GET_ONCE.
    bool statusRequested() { const bool r = statusReq_; statusReq_ = false; return r; } ///< Consume a @c CMD_GET_STATUS.

    /**
     * @brief Consumes a pending @c CMD_SET_CAN_MODE; 0 when none is pending.
     *
     * Consume-on-read like the flags above, so the bridge relays each request
     * exactly once. A latched value that survived reading would be re-sent to
     * the MKR every loop, and the MKR's own rate limiter would reject nearly
     * all of them - turning one host request into a stream of failures.
     */
    uint8_t takeCanModeRequest() { const uint8_t m = canModeReq_; canModeReq_ = 0; return m; }

    /**
     * @brief Consumes a pending @c CMD_SET_IMU_MODE; 0 when none is pending.
     *
     * Consume-on-read like the CAN mode above, and it matters more here: each
     * relayed request costs the sensor a ~700 ms bring-up, so a latched value
     * that survived reading would be re-sent every loop and hold the IMU in
     * permanent re-initialisation.
     */
    uint8_t takeImuModeRequest() { const uint8_t m = imuModeReq_; imuModeReq_ = 0; return m; }

    /**
     * @brief Consumes a pending @c CMD_SET_CAN_FILTER.
     *
     * Consume-on-read like the others, so each host request is relayed once.
     *
     * Returns a BOOL rather than the count, because zero is a real request —
     * clearing the filters means accept-all on the master's MCP2515 — and a
     * "0 means nothing pending" convention would make the one command that
     * widens a capture the one command that silently does nothing.
     *
     * @param[out] ids    @c CAN_FILTER_SLOTS entries, always written.
     * @param[out] count  How many are meaningful; may be 0.
     * @return true when a request was pending.
     */
    bool takeCanFilterRequest(uint16_t *ids, uint8_t &count)
    {
        if (!canFilterPending_) return false;
        for (uint8_t i = 0; i < hostproto::CAN_FILTER_SLOTS; ++i) ids[i] = canFilterIds_[i];
        count = canFilterCount_;
        canFilterPending_ = false;
        return true;
    }

    bool    streaming()  const { return streaming_; } ///< True while telemetry forwarding is enabled.
    uint8_t decimation() const { return decim_; }     ///< Forward every Nth master frame (>= 1).

    bool sendTelemetry(const hostproto::Telemetry &t);  ///< @return true if the frame went out.
    bool sendStatus(const hostproto::BridgeStatus &s);  ///< @return true if the frame went out.
    bool sendHello(const hostproto::Hello &h);          ///< @return true if the frame went out.

    /**
     * @brief Sends one ASCII log line to the Jetson as a @c MSG_LOG frame.
     * @param level @c hostproto::LogLevel.
     * @param text  NUL-terminated; silently truncated to @c HOSTLINK_MAX_LOG_TEXT.
     * @return @c true if the frame went out.
     */
    bool sendLog(uint8_t level, const char *text);

    uint32_t framesRx()   const { return rx_.framesDecoded; } ///< Command frames accepted since begin().
    uint32_t crcErrors()  const { return rx_.crcErrors; }     ///< CRC failures on this hop since begin().
    uint32_t txDropped()  const { return txDropped_; }        ///< Frames not sent (host absent / TX ring full).

private:
    bool sendFrame(uint8_t type, const uint8_t *payload, uint8_t len);
    void handleCommand(uint8_t type, const uint8_t *payload, uint8_t len);
    bool sendNack(uint8_t offendingType, uint8_t reason);
    void resetSessionState();

    hostproto::RxState rx_{};                         ///< Incremental decoder (owns its own 255 B accumulator).
    uint8_t  rxPayload_[hostproto::MAX_PAYLOAD] = {}; ///< Decoded payload staging.
    uint8_t  logBuf_[hostproto::MAX_PAYLOAD]    = {}; ///< MSG_LOG payload staging.
    uint8_t  txBuf_[hostproto::MAX_FRAME]       = {}; ///< Frame staging — a member, so no large stack frames.

    uint32_t lastHostRxMs_ = 0;
    uint32_t txDropped_    = 0;
    bool     hostAlive_    = false;
    bool     connectSeen_  = false;
    bool     onceReq_      = false;
    uint8_t canModeReq_ = 0;   ///< Pending CMD_SET_CAN_MODE arg; 0 = none.
    uint8_t imuModeReq_ = 0;   ///< Pending CMD_SET_IMU_MODE arg; 0 = none.
    /// Pending CMD_SET_CAN_FILTER. A separate flag, not "count != 0" — see
    /// takeCanFilterRequest() for why zero is a request rather than an absence.
    bool     canFilterPending_ = false;
    uint8_t  canFilterCount_   = 0;
    uint16_t canFilterIds_[hostproto::CAN_FILTER_SLOTS] = { 0, 0, 0, 0, 0, 0 };
    bool     statusReq_    = false;
    bool     streaming_    = false;
    uint8_t  decim_        = 1;
};
