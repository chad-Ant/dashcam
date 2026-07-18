/**
 * @file libcan.h
 * @brief SocketCAN wrapper for CAN and CAN FD communication on the Jetson Orin Nano.
 *
 * Wraps the Linux SocketCAN interface (PF_CAN / SOCK_RAW) to provide:
 *   - Frame send/receive for classic CAN (up to 8 bytes) and CAN FD (up to 64 bytes).
 *   - Callback-based background receive thread with pipe-based wake-on-stop.
 *   - Kernel-level receive filters (CAN_RAW_FILTER).
 *   - A one-call interface configuration helper (requires CAP_NET_ADMIN).
 *
 * Hardware context:
 *   - CAN controller : NVIDIA MTTCAN (c310000.mttcan), exposed as can0.
 *   - CAN transceiver: Waveshare SN65HVD230 — CAN 2.0 only, max 1 Mbps.
 *     The SN65HVD230 does NOT support the CAN FD fast data phase (BRS).
 *     Set fdMode=false in CanBusConfig unless a CAN FD-capable transceiver
 *     (e.g. TJA1044, TCAN1042) is fitted.
 *
 * Typical usage:
 * @code
 *   // 1. Configure the interface (needs sudo / CAP_NET_ADMIN):
 *   dashcam::can::CanBusConfig cfg;
 *   cfg.bitrate = 500000;
 *   dashcam::can::configureInterface("can0", cfg, log);
 *
 *   // 2. Open and start:
 *   dashcam::can::CanBus bus;
 *   bus.open("can0", false, log);
 *   bus.setReceiveCallback([](const dashcam::can::CanFrame& f) {
 *       // handle f.id, f.len, f.data …
 *   });
 *   bus.start();
 *
 *   // 3. Send a frame:
 *   dashcam::can::CanFrame tx;
 *   tx.id  = 0x123;
 *   tx.len = 4;
 *   tx.data[0] = 0xDE; tx.data[1] = 0xAD;
 *   tx.data[2] = 0xBE; tx.data[3] = 0xEF;
 *   bus.send(tx);
 *
 *   // 4. Tear down:
 *   bus.stop();
 *   bus.close();
 * @endcode
 */

#ifndef LIBCAN_H
#define LIBCAN_H

#include "liblog.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace dashcam::can {

// ─── CAN frame ────────────────────────────────────────────────────────────────

static constexpr uint8_t CAN_CLASSIC_DLEN = 8;
static constexpr uint8_t CAN_FD_DLEN      = 64;

/**
 * @brief Unified CAN / CAN FD frame.
 *
 * Classic CAN: fdFrame=false, brs/esi unused, len ≤ 8.
 * CAN FD     : fdFrame=true,  rtr must be false,  len ≤ 64.
 */
struct CanFrame {
    uint32_t id       = 0;                  ///< 11-bit standard or 29-bit extended CAN ID.
    bool     extended = false;              ///< true = 29-bit extended frame format (EFF).
    bool     rtr      = false;              ///< Remote transmission request (classic only).
    bool     fdFrame  = false;              ///< true = CAN FD frame.
    bool     brs      = false;              ///< Bit-rate switch (FD only).
    bool     esi      = false;              ///< Error state indicator (FD receive, read-only).
    uint8_t  len      = 0;                  ///< Payload byte count (0–8 classic, 0–64 FD).
    uint8_t  data[CAN_FD_DLEN] = {};
};

// ─── Receive filter ───────────────────────────────────────────────────────────

/**
 * @brief Kernel-level CAN receive filter.
 *
 * A frame passes when (frame.id & mask) == (filter.id & mask).
 * Set extended=true to also match the EFF bit.
 * mask=0 matches every ID.
 */
struct CanFilter {
    uint32_t id       = 0;
    uint32_t mask     = 0;       ///< 0 = accept all IDs.
    bool     extended = false;   ///< Match extended (29-bit) IDs.
};

// ─── Callbacks ────────────────────────────────────────────────────────────────

using ReceiveCallback = std::function<void(const CanFrame&)>;
using ErrorCallback   = std::function<void(const std::string&)>;

// ─── Interface configuration ──────────────────────────────────────────────────

/**
 * @brief Parameters for configureInterface().
 *
 * Note: dataBitrate and fdMode require a CAN FD-capable transceiver.
 * The SN65HVD230 is a CAN 2.0 device — keep fdMode=false with it.
 */
struct CanBusConfig {
    uint32_t bitrate     = 500000;  ///< Nominal bit rate in bps (e.g. 125000, 250000, 500000, 1000000).
    uint32_t dataBitrate = 0;       ///< CAN FD data-phase bit rate; 0 = same as bitrate.
    bool     fdMode      = false;   ///< Enable CAN FD mode (requires FD-capable transceiver).
    bool     loopback    = false;   ///< Enable hardware loopback (useful for self-test).
    bool     listenOnly  = false;   ///< Listen-only mode: receive only, no ACKs or TX.
};

/**
 * @brief Bring down, reconfigure, and bring up a SocketCAN interface.
 *
 * Executes three `ip link` commands sequentially:
 *   ip link set <iface> down
 *   ip link set <iface> type can bitrate <N> [dbitrate <N> fd on] [loopback on] [listen-only on]
 *   ip link set <iface> up
 *
 * Requires CAP_NET_ADMIN (i.e. sudo or the capability granted to the process).
 *
 * @return true if all three commands exit successfully; false otherwise.
 */
bool configureInterface(const std::string& iface, const CanBusConfig& cfg,
                        const dashcam::log::LogCallback& log = {});

// ─── CanBus ───────────────────────────────────────────────────────────────────

/**
 * @brief SocketCAN raw socket: send and receive CAN / CAN FD frames.
 *
 * - send() is synchronous and thread-safe.
 * - Frames arrive via the receive callback on a private background thread.
 * - The background thread uses select() + a wake pipe so stop() returns promptly.
 */
class CanBus {
public:
    CanBus();
    ~CanBus();

    CanBus(const CanBus&)            = delete;
    CanBus& operator=(const CanBus&) = delete;

    /**
     * @brief Open a raw SocketCAN socket bound to @p iface.
     *
     * @param iface     Interface name, e.g. "can0".
     * @param fdEnabled Request CAN FD support (CAN_RAW_FD_FRAMES).
     *                  The interface must have been brought up with `fd on`.
     * @param log       Optional log callback.
     * @return true on success.
     */
    bool open(const std::string& iface, bool fdEnabled = false,
              const dashcam::log::LogCallback& log = {});

    /**
     * @brief Install kernel-level receive filters.
     *
     * Must be called before start().  An empty vector restores the default
     * pass-all filter.
     */
    void setFilters(const std::vector<CanFilter>& filters);

    void setReceiveCallback(ReceiveCallback cb);
    void setErrorCallback(ErrorCallback cb);

    /**
     * @brief Transmit one frame.  Thread-safe.
     * @return true if the kernel write() succeeded.
     */
    bool send(const CanFrame& frame);

    /**
     * @brief Start the background receive thread.
     * @return false if the socket is not open or the thread is already running.
     */
    bool start();

    /**
     * @brief Signal the receive thread to stop and join it.
     */
    void stop();

    /**
     * @brief Close the socket; implies stop().
     */
    void close();

    bool isOpen()    const;
    bool isRunning() const;

private:
    void rxLoop();

    int                        m_fd        = -1;
    bool                       m_fdEnabled = false;
    std::atomic<bool>          m_running{false};
    std::thread                m_thread;
    int                        m_pipe[2]   = {-1, -1};  ///< Wake pipe: write [1] to unblock select().
    ReceiveCallback            m_rxCb;
    ErrorCallback              m_errCb;
    dashcam::log::LogCallback  m_log;
    mutable std::mutex         m_sendMtx;
    mutable std::mutex         m_cbMtx;
};

} // namespace dashcam::can

#endif // LIBCAN_H
