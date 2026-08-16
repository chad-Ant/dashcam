/**
 * @file libcommlink.h
 * @brief Jetson Orin Nano side of the ESP32-C3 telemetry link (USB CDC).
 *
 * The ESP32-C3 XIAO plugs into a USB port on the Orin Nano by its own USB-C
 * connector and enumerates through cdc_acm as @c /dev/ttyACM*.  Over that pipe
 * it speaks @c HostProtocol.h — the same SOF/VER/TYPE/LEN/CRC16 framing the C3
 * already uses toward the MKR Zero:
 *
 *   MKR Zero ──UART 115200──> ESP32-C3 ──USB-C──> Jetson Orin Nano
 *  OBD2 + GPS + IMU            bridge              this library
 *
 * The bridge pushes:
 *   - @c MSG_TELEMETRY — one @c sizeof(Telemetry) OBD2 + GPS + IMU snapshot per
 *     master frame (~10 Hz).  The size is deliberately not written out here: it
 *     said "79-byte" through two payload revisions, which is what a duplicated
 *     constant in prose does.  @c HostProtocol.h holds the single definition.
 *   - @c MSG_STATUS    — bridge health at 1 Hz, whether or not telemetry flows.
 *   - @c MSG_LOG       — bridge log lines, forwarded into liblog by default.
 *   - @c MSG_HELLO     — identity + reset reason on every (re)connection.
 *
 * ── What USB adds over the UART hop ──────────────────────────────────────────
 * A device node can vanish mid-run (unplug, C3 reset, USB bus reset) and come
 * back under a different name.  This class therefore owns a reconnect loop: the
 * background thread reopens the port on its own, re-runs the handshake, and
 * re-applies the streaming settings.  Callers never see the port churn — only
 * @c isConnected() flipping and the connection callback firing.
 *
 * ── Device naming ────────────────────────────────────────────────────────────
 * @c /dev/ttyACM0 is a race, not an address: it depends on enumeration order.
 * With an empty @c CommLinkConfig::device the library resolves the port through
 * @c /dev/serial/by-id/, matching @c CommLinkConfig::idMatch against the stable
 * per-device symlink.  For production, install the shipped udev rule
 * (@c 99-dashcam-bridge.rules) and set @c device to @c /dev/dashcam-bridge —
 * it also stops ModemManager probing the port with AT commands at plug-in.
 *
 * Typical usage:
 * @code
 *   dashcam::commlink::CommLinkConfig cfg;   // auto-discovery + auto-stream
 *   dashcam::commlink::CommLink       bridge;
 *
 *   bridge.setTelemetryCallback([](const dashcam::commlink::Telemetry& t) {
 *       if (!std::isnan(t.speed)) overlay.setSpeed(t.speed);
 *   });
 *   if (!bridge.open(cfg, log)) return 1;
 *   bridge.start();
 *   ...
 *   bridge.stop();     // implied by close() and by the destructor
 * @endcode
 *
 * Thread-safety: every public method is safe to call from any thread.  Callbacks
 * run on the private RX thread — keep them short and do not call stop()/close()
 * from inside one.
 */

#ifndef LIBCOMMLINK_H
#define LIBCOMMLINK_H

#include "HostProtocol.h"
#include "liblog.h"
#include "libuart.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace dashcam::commlink {

// The wire structs are used unqualified throughout the application layer.
using Telemetry    = hostproto::Telemetry;
using BridgeStatus = hostproto::BridgeStatus;
using Hello        = hostproto::Hello;

// ─── configuration ────────────────────────────────────────────────────────────

struct CommLinkConfig {
    /**
     * Device node to open.  Empty = auto-discover via /dev/serial/by-id using
     * @c idMatch, then fall back to scanning /dev/ttyACM*.
     */
    std::string device;

    /**
     * Substring matched against /dev/serial/by-id entry names during
     * auto-discovery.  The C3's native USB port appears as
     * "usb-Espressif_USB_JTAG_serial_debug_unit_<mac>-if00".
     */
    std::string idMatch = "USB_JTAG";

    /**
     * Also probe bare /dev/ttyACM* nodes when the by-id lookup finds nothing.
     *
     * Off by default: probing opens each candidate exclusively for up to
     * @c handshakeMs, which would briefly steal an unrelated modem or Arduino
     * from whatever else is using it.  by-id identifies the bridge by its own
     * USB descriptor strings and is the correct path on any udev system; turn
     * this on only for a bench rig where the C3 is the sole ACM device.
     */
    bool allowAcmFallback = false;

    uint32_t baudRate = 115200; ///< Ignored by USB CDC (no line rate); set for termios validity.

    int reconnectMs = 1000;  ///< Delay between reopen attempts while the device is absent.
    int keepaliveMs = 1000;  ///< CMD_PING period.  Must stay well under the bridge's 5 s host timeout.
    int connectionTimeoutMs = 3000; ///< Declare the bridge gone after this much frame silence.
    int staleMs     = 500;   ///< Default freshness window for @c isStale().

    /**
     * How long a freshly opened candidate has to answer CMD_HELLO before it is
     * rejected and the next candidate is tried.
     *
     * open() succeeding proves only that a tty exists — every CDC-ACM device on
     * the machine opens just as happily.  Without this gate the library can
     * latch onto an unrelated modem or Arduino and stay there forever, because
     * the connection watchdog only runs once a frame has already arrived.  A
     * candidate must therefore *speak the protocol* to be accepted.
     */
    int handshakeMs = 2000;

    /**
     * Send CMD_SET_DECIM + CMD_START_STREAM automatically on every MSG_HELLO,
     * so a bridge reset resumes streaming with no application involvement.
     */
    bool    autoStream = true;
    uint8_t decimation = 1;     ///< Forward every Nth master frame (1 = all, ~10 Hz).

    bool exclusive     = true;  ///< TIOCEXCL: keep stray terminal programs off the port.

    /**
     * Forward MSG_LOG lines from the bridge into the liblog callback, tagged
     * "[c3]".  Independent of any callback set with setBridgeLogCallback().
     */
    bool forwardBridgeLogs = true;
};

// ─── statistics ───────────────────────────────────────────────────────────────

/** @brief Cumulative counters since open(); useful for a health page or a soak test. */
struct LinkStats {
    uint64_t bytesRx      = 0; ///< Bytes read from the device node.
    uint64_t framesRx     = 0; ///< CRC-valid frames decoded.
    uint64_t crcErrors    = 0; ///< Frames discarded on CRC mismatch.
    uint64_t telemetryRx  = 0; ///< MSG_TELEMETRY frames accepted.
    uint64_t statusRx     = 0; ///< MSG_STATUS frames accepted.
    uint64_t logRx        = 0; ///< MSG_LOG frames accepted.
    uint64_t nacksRx      = 0; ///< MSG_NACK frames received (a command we sent was rejected).
    uint64_t malformedRx  = 0; ///< Frames with a payload length wrong for their type.
    uint64_t txFrames     = 0; ///< Command frames written.
    uint64_t txErrors     = 0; ///< Command frames that could not be written.
    uint64_t reconnects   = 0; ///< Successful port (re)opens after the first.
};

// ─── callbacks ────────────────────────────────────────────────────────────────

using TelemetryCallback  = std::function<void(const Telemetry&)>;
using StatusCallback     = std::function<void(const BridgeStatus&)>;
using HelloCallback      = std::function<void(const Hello&)>;
using BridgeLogCallback  = std::function<void(dashcam::log::LogLevel, const std::string&)>;
using ConnectionCallback = std::function<void(bool connected)>; ///< Fired on every state change.

// ─── CommLink ─────────────────────────────────────────────────────────────────

class CommLink {
public:
    CommLink();
    ~CommLink();

    CommLink(const CommLink&)            = delete;
    CommLink& operator=(const CommLink&) = delete;

    /**
     * @brief Resolve and open the bridge's device node.
     *
     * A failure here is not terminal: call start() anyway and the RX thread
     * will keep retrying at @c reconnectMs until the C3 is plugged in.
     *
     * @return true if the port opened on this call.
     */
    bool open(const CommLinkConfig& cfg = {}, const dashcam::log::LogCallback& log = {});

    /** @brief Start the background RX / reconnect / keepalive thread. */
    bool start();

    /** @brief Signal the thread to stop and join it.  Safe to call twice. */
    void stop();

    /** @brief Close the port; implies stop(). */
    void close();

    // ── callbacks (set before start(); replacing one later is thread-safe) ────
    void setTelemetryCallback(TelemetryCallback cb);
    void setStatusCallback(StatusCallback cb);
    void setHelloCallback(HelloCallback cb);
    void setBridgeLogCallback(BridgeLogCallback cb);
    void setConnectionCallback(ConnectionCallback cb);

    // ── commands (thread-safe; all non-blocking) ──────────────────────────────
    bool requestOnce();                 ///< Ask the bridge to fetch one fresh snapshot from the MKR.
    bool requestStatus();               ///< Ask for an immediate MSG_STATUS.
    bool startStream();                 ///< Begin telemetry forwarding.
    bool stopStream();                  ///< Stop telemetry forwarding (status keeps arriving).
    bool ping();                        ///< Link check; the bridge answers MSG_PONG.

    /**
     * @brief Forward only every @p n-th master frame (1 = all, ~10 Hz).
     *
     * Also updates the value re-applied after a reconnect.
     * @return false if @p n is 0 or the write failed.
     */
    bool setDecimation(uint8_t n);

    /**
     * @brief Switches the MKR's CAN controller mode.
     *
     * @param mode 1 = discover (listen-only, accept-all), 2 = sniff
     *             (listen-only, filtered, decoding), 3 = obd2 (BUS-ACTIVE:
     *             the MKR transmits diagnostic requests on the vehicle bus).
     *
     * Not restored automatically after a reconnect - see the implementation.
     * @return false on an out-of-range mode or a failed write.
     */
    bool setCanMode(uint8_t mode);

    /**
     * @brief Selects the IMU's operating mode on the master.
     *
     * @param mode @c hostproto::IMU_MODE_FUSION (1) for IMUPLUS — on-chip
     *             fusion giving gravity-compensated linear acceleration and
     *             relative yaw, clipping at 4 g — or @c IMU_MODE_RAW (2) for
     *             AMG, which reaches 16 g and produces no fusion at all.
     *
     * The two are DIFFERENT MEASUREMENTS, not a quality setting, so this is a
     * session-level decision.
     *
     * ⚠️ Applying it restarts the sensor's bring-up: no IMU data for about
     * 700 ms and the trailing peak window is discarded. Never send it in
     * response to an impact — a crash pulse lasts 10-50 ms and would be long
     * over, with the only record of it thrown away. The master rate-limits
     * this and refuses one that arrives too soon after the last.
     *
     * Not restored automatically after a reconnect - see the implementation.
     * Read @c TLM_FLAG_IMU_FUSION_MODE to learn the mode actually in force.
     * @return false on an out-of-range mode or a failed write.
     */
    bool setImuMode(uint8_t mode);

    // ── state ─────────────────────────────────────────────────────────────────
    bool isOpen()      const; ///< The device node is currently open.
    bool isRunning()   const; ///< The RX thread is running.
    bool isConnected() const; ///< Frames arrived within @c connectionTimeoutMs.
    bool hasTelemetry() const; ///< At least one MSG_TELEMETRY has been received.

    /** @brief Newest telemetry snapshot.  Zero-filled until hasTelemetry() is true. */
    Telemetry telemetry() const;

    /** @brief Newest bridge status.  Zero-filled until the first MSG_STATUS. */
    BridgeStatus status() const;

    /**
     * @brief True if no telemetry arrived within @p ms milliseconds.
     * @param ms Freshness window; < 0 uses @c CommLinkConfig::staleMs.
     */
    bool isStale(int ms = -1) const;

    /** @brief Milliseconds since the last telemetry frame; -1 if none ever arrived. */
    int64_t telemetryAgeMs() const;

    LinkStats   stats() const;
    std::string devicePath() const; ///< The node currently open, or "" when closed.

    /**
     * @brief List candidate bridge device nodes, most specific first.
     *
     * Scans /dev/serial/by-id for entries containing @p idMatch and resolves
     * each symlink, then — only if @p includeAcmFallback — appends any
     * /dev/ttyACM* not already listed.
     */
    static std::vector<std::string> enumerate(const std::string& idMatch = "USB_JTAG",
                                              bool includeAcmFallback = true);

private:
    void rxLoop();
    bool openPort();
    void closePort();
    void feed(const uint8_t* data, size_t len, uint32_t nowMs);
    void onFrame(uint8_t type, const uint8_t* payload, uint8_t len);
    void setConnected(bool connected);
    bool sendFrame(uint8_t type, const uint8_t* payload, uint8_t len);
    void wake();

    CommLinkConfig            m_cfg;
    dashcam::log::LogCallback m_log;
    dashcam::uart::Uart       m_uart;
    std::string               m_devicePath;

    std::atomic<bool> m_running{false};
    std::atomic<bool> m_connected{false};
    std::atomic<bool> m_hasTelemetry{false};
    /// Live decimation, kept out of m_cfg because setDecimation() writes it from
    /// an application thread while the RX thread re-applies it on MSG_HELLO.
    std::atomic<uint8_t> m_decimation{1};
    /// Set by openPort(), consumed by the RX thread: send CMD_HELLO to announce
    /// the new session.  openPort() cannot send it itself — it holds m_portMtx,
    /// which sendFrame() also takes.
    std::atomic<bool> m_announcePending{false};
    /// Set ONLY by a MSG_HELLO whose protocol version and struct sizes match.
    /// This — not m_connected — is what satisfies the discovery handshake:
    /// m_connected rises on any CRC-valid frame, which a mismatched bridge or a
    /// foreign device emitting a coincidentally valid frame would also satisfy.
    std::atomic<bool> m_helloOk{false};
    std::thread       m_thread;
    int               m_pipe[2] = {-1, -1}; ///< Wake pipe: write [1] to unblock poll().

    hostproto::RxState m_rx{};              ///< RX-thread-only; no lock needed.
    uint8_t            m_payload[hostproto::MAX_PAYLOAD] = {};

    mutable std::mutex m_portMtx;  ///< Serialises port open/close/read/write.
    mutable std::mutex m_dataMtx;  ///< Guards the snapshots and timestamps.
    mutable std::mutex m_cbMtx;    ///< Guards the callback slots.
    mutable std::mutex m_statsMtx; ///< Guards m_stats.

    Telemetry    m_telemetry{};
    BridgeStatus m_status{};
    int64_t      m_lastTelemetryMs = -1; ///< steady_clock ms, -1 = never.
    int64_t      m_lastFrameMs     = -1;
    LinkStats    m_stats;
    uint64_t     m_openCount = 0;        ///< Successful openPort() calls; >1 means a reconnect.
    /// Rotates the candidate list start position so a device that opens but
    /// never answers the handshake is not retried ahead of the others forever.
    /// Atomic: the RX thread advances it while openPort() reads it.
    std::atomic<size_t> m_probeCursor{0};

    TelemetryCallback  m_telemetryCb;
    StatusCallback     m_statusCb;
    HelloCallback      m_helloCb;
    BridgeLogCallback  m_bridgeLogCb;
    ConnectionCallback m_connCb;
};

} // namespace dashcam::commlink

#endif // LIBCOMMLINK_H
