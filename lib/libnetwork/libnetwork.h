/**
 * @file libnetwork.h
 * @brief TCP / UDP internet connectivity for the dashcam system.
 *
 * A thin, RAII wrapper over POSIX sockets that gives the rest of the system a
 * small, well-behaved transport layer instead of scattering raw socket calls
 * through the application.  Phase 1 provides:
 *
 *   - UdpSocket : bind / sendTo / recvFrom with a poll()-based receive timeout.
 *   - TcpSocket : connect / sendAll / recv, move-only so it can be handed back
 *                 from TcpServer::accept() by value.
 *   - TcpServer : listen / accept with a self-pipe wake so a blocked accept()
 *                 unblocks promptly when another thread calls close()
 *                 (the same shutdown idiom libcan uses for its receive thread).
 *   - SNTP time : queryTime() fetches an observation from an internet time
 *                 server and reports the local clock offset.  Plain SNTP is
 *                 not authenticated and must not be trusted to set a clock.
 *
 * Design notes:
 *   - IPv4 only for now (AF_INET).  getaddrinfo() is restricted to A records so
 *     the datagram socket family always matches the resolved destination.
 *   - send() uses MSG_NOSIGNAL: a write to a peer that has gone away returns an
 *     error rather than raising SIGPIPE and killing the process.
 *   - All blocking operations take an explicit millisecond timeout; nothing here
 *     blocks forever.
 *   - Logging is optional: pass a dashcam::log::LogCallback to surface errors;
 *     an empty callback is always safe (never invoked).
 *
 * Later phases add the video-streaming transport (RTP control for re-encoded
 * inference feeds; direct socket transport for precompressed UVC recording
 * feeds) on top of these primitives.
 *
 * Typical usage:
 * @code
 *   // Internet time observation (host chrony/systemd-timesyncd owns discipline):
 *   auto t = dashcam::network::queryTime();           // pool.ntp.org
 *   if (t.valid)
 *       log(INFO, "clock offset " + std::to_string(t.offsetSeconds));
 *
 *   // UDP:
 *   dashcam::network::UdpSocket u;
 *   u.open();                                          // ephemeral local port
 *   u.sendTo("192.168.1.50", 5000, buf, len);
 *
 *   // TCP server:
 *   dashcam::network::TcpServer srv;
 *   srv.listen(5001);
 *   dashcam::network::IoStatus st;
 *   dashcam::network::TcpSocket c = srv.accept(1000, &st);
 *   if (c.isOpen()) c.sendAll(payload, n);
 * @endcode
 */

#ifndef LIBNETWORK_H
#define LIBNETWORK_H

#include "liblog.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace dashcam::network {

// ─── I/O status ───────────────────────────────────────────────────────────────

/**
 * @brief Outcome of a stream (TCP) receive / accept operation.
 *
 * UDP uses a signed byte count instead (see UdpSocket::recvFrom): a datagram
 * socket has no orderly-close state, so Closed is meaningless there.
 */
enum class IoStatus {
    Ok,       ///< Data (or a connection) is available.
    Timeout,  ///< The timeout elapsed with nothing ready.
    Closed,   ///< The peer closed the connection, or the server was close()d.
    Error     ///< A socket error occurred (details logged if a callback was given).
};

// ─── UdpSocket ─────────────────────────────────────────────────────────────────

/**
 * @brief A datagram (UDP) socket: bind, sendTo, recvFrom.
 *
 * open() with bindPort == 0 gives an ephemeral local port suitable for a client
 * (e.g. the SNTP query); pass a fixed port to receive on a known port.  Send and
 * receive resolve / report peer addresses as dotted-quad strings.
 */
class UdpSocket {
public:
    UdpSocket() = default;
    ~UdpSocket();

    UdpSocket(const UdpSocket&)            = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;
    UdpSocket(UdpSocket&&) noexcept;
    UdpSocket& operator=(UdpSocket&&) noexcept;

    /**
     * @brief Create the socket and bind it.
     * @param bindPort  Local UDP port; 0 = OS-assigned ephemeral port.
     * @param log       Optional log callback.
     * @return true on success.
     */
    bool open(uint16_t bindPort = 0, const dashcam::log::LogCallback& log = {});

    /**
     * @brief Send one datagram to @p host : @p port.
     *
     * @p host may be a dotted-quad literal or a hostname (resolved to an A
     * record).  The socket must be open().  A single sendto() is issued; UDP
     * gives no delivery guarantee.
     * @return true if the datagram was handed to the kernel.
     */
    bool sendTo(const std::string& host, uint16_t port, const void* data, size_t len,
                std::string* resolvedHost = nullptr);

    /**
     * @brief Receive one datagram, waiting up to @p timeoutMs.
     *
     * @param buf        Destination buffer.
     * @param bufLen     Buffer capacity; a larger datagram is truncated.
     * @param timeoutMs  Maximum wait in ms (0 polls, <0 waits indefinitely).
     * @param srcHost    Optional out: sender IP as a dotted-quad string.
     * @param srcPort    Optional out: sender UDP port.
     * @return bytes received (>0), 0 on timeout, or -1 on error.
     */
    long recvFrom(void* buf, size_t bufLen, int timeoutMs,
                  std::string* srcHost = nullptr, uint16_t* srcPort = nullptr);

    /** @brief Local port the socket is bound to (useful after open(0)); 0 if unknown. */
    uint16_t localPort() const;

    void close();
    bool isOpen() const { return fd_.load() >= 0; }
    int  fd()     const { return fd_.load(); }

private:
    std::atomic<int>          fd_{-1};
    dashcam::log::LogCallback log_;
};

// ─── TcpSocket ─────────────────────────────────────────────────────────────────

/**
 * @brief A connected stream (TCP) socket.
 *
 * Move-only: TcpServer::accept() returns one by value, and ownership of the
 * underlying descriptor transfers on move (the moved-from socket is left
 * closed).  Copying is deleted so a descriptor is never double-closed.
 */
class TcpSocket {
public:
    TcpSocket() = default;

    /** @brief Adopt an already-connected descriptor (used by TcpServer::accept). */
    explicit TcpSocket(int fd, std::string peer = {});

    ~TcpSocket();

    TcpSocket(const TcpSocket&)            = delete;
    TcpSocket& operator=(const TcpSocket&) = delete;
    TcpSocket(TcpSocket&&) noexcept;
    TcpSocket& operator=(TcpSocket&&) noexcept;

    /**
     * @brief Connect to @p host : @p port with a bounded timeout.
     *
     * The connect is performed non-blocking and waited on with poll() so a dead
     * host cannot hang the caller; the socket is restored to blocking on return.
     * @return true once the connection is established.
     */
    bool connect(const std::string& host, uint16_t port, int timeoutMs,
                 const dashcam::log::LogCallback& log = {});

    /**
     * @brief Send exactly @p len bytes, looping over short writes.
     *
     * Uses MSG_NOSIGNAL, so a peer that has gone away yields false rather than
     * SIGPIPE.
     * @return true only if all @p len bytes were sent.
     */
    bool sendAll(const void* data, size_t len);

    /**
     * @brief Receive up to @p bufLen bytes, waiting up to @p timeoutMs.
     *
     * @param outBytes  Set to the number of bytes read (0 unless Ok).
     * @return IoStatus::Ok      bytes were read (outBytes > 0);
     *         IoStatus::Timeout nothing arrived within the timeout;
     *         IoStatus::Closed  the peer performed an orderly shutdown;
     *         IoStatus::Error   a socket error occurred.
     */
    IoStatus recv(void* buf, size_t bufLen, int timeoutMs, size_t& outBytes);

    /**
     * @brief Half-close both directions (::shutdown SHUT_RDWR) without closing
     *        the descriptor.
     *
     * Unblocks a send()/recv() in progress on another thread so it returns with
     * an error — the clean way to release a writer thread blocked on a slow or
     * stalled peer before joining it.  close() still owns the descriptor.
     */
    void shutdown();

    void close();
    bool isOpen() const { return fd_.load() >= 0; }
    int  fd()     const { return fd_.load(); }

    /** @brief Remote endpoint as "ip:port" (empty if unknown). */
    const std::string& peer() const { return peer_; }

private:
    std::atomic<int>          fd_{-1};
    std::string               peer_;
    dashcam::log::LogCallback log_;
};

// ─── TcpServer ─────────────────────────────────────────────────────────────────

/**
 * @brief A listening (TCP) socket that accepts inbound connections.
 *
 * accept() blocks up to a timeout on both the listening descriptor and an
 * internal self-pipe.  close() writes to that pipe, so an accept() running in
 * another thread returns IoStatus::Closed promptly instead of hanging — the same
 * wake-on-stop pattern libcan uses.  Coordinate shutdown by having the accept
 * loop stop when it observes Closed; do not call accept() after close().
 */
class TcpServer {
public:
    TcpServer() = default;
    ~TcpServer();

    TcpServer(const TcpServer&)            = delete;
    TcpServer& operator=(const TcpServer&) = delete;

    /**
     * @brief Bind and listen on @p port.
     * @param port     Local TCP port; 0 = OS-assigned (read back via port()).
     * @param backlog  Pending-connection queue depth.
     * @return true on success.
     */
    bool listen(uint16_t port, int backlog = 4, const dashcam::log::LogCallback& log = {});

    /**
     * @brief Wait up to @p timeoutMs for an inbound connection.
     *
     * @param status  Optional out: Ok (a connection was accepted), Timeout,
     *                Closed (woken by close()), or Error.
     * @return A connected TcpSocket on Ok (isOpen() == true); otherwise an
     *         unopened TcpSocket.
     */
    TcpSocket accept(int timeoutMs, IoStatus* status = nullptr);

    /** @brief Close the listener and wake any concurrent accept(). */
    void close();

    bool     isOpen() const { return fd_.load() >= 0; }
    uint16_t port()   const { return port_.load(); }

private:
    std::atomic<int>      fd_{-1};
    std::atomic<int>      wake_[2]{{-1}, {-1}}; ///< Self-pipe: write [1] wakes accept().
    std::atomic<uint16_t> port_{0};
    dashcam::log::LogCallback log_;
};

// ─── Media streaming ───────────────────────────────────────────────────────────

/**
 * @brief Wire format a MediaStreamServer speaks to its viewers.
 *
 * This is the "direct" streaming path for cameras whose feed is ALREADY
 * compressed (the UVC recording cameras): libnetwork owns the sockets and fans
 * each pushed frame out to every connected viewer, with no decode / re-encode.
 * (Re-encoded inference feeds take the GStreamer-native RTP path instead.)
 */
enum class StreamWire {
    MjpegHttp,  ///< HTTP multipart/x-mixed-replace; each pushFrame() is one JPEG. Open http://host:port/ in a browser or VLC.
    RawTcp,     ///< Raw byte passthrough (e.g. an H.264 Annex-B elementary stream). View with: ffplay tcp://host:port
};

/** @brief MediaStreamServer tuning. */
struct StreamServerConfig {
    uint16_t    port       = 8090;                 ///< TCP listen port (0 = OS-assigned, read back via port()).
    StreamWire  wire       = StreamWire::MjpegHttp;///< Viewer wire format.
    int         maxClients = 4;                    ///< Connections beyond this are refused.
    int         queueDepth = 4;                    ///< Per-client backlog (frames); the oldest are dropped when a viewer can't keep up (live view favours latest).
    std::string boundary   = "dashcamframe";       ///< MJPEG multipart boundary token (MjpegHttp only).
};

/**
 * @brief A TCP server that fans compressed frames out to multiple viewers.
 *
 * Built on TcpServer / TcpSocket.  The application pushes each already-compressed
 * frame with pushFrame(); the server delivers it to every connected viewer.  Each
 * viewer gets its own writer thread and a bounded latest-frame queue, so one slow
 * viewer drops its own frames instead of stalling the producer or the others.  A
 * viewer that disconnects is detected on the next send and reaped.
 *
 * Threading:
 *   - start()/stop() from the control thread; not re-entrant.
 *   - pushFrame() is safe from any single producer thread (e.g. the capture /
 *     record callback).  One framed payload is built per call and shared by all
 *     viewers (no per-viewer copy).
 *
 * Typical usage:
 * @code
 *   dashcam::network::MediaStreamServer srv;
 *   dashcam::network::StreamServerConfig sc;
 *   sc.port = 8090; sc.wire = dashcam::network::StreamWire::MjpegHttp;
 *   srv.start(sc, log);
 *   // in the compressed-frame callback:
 *   srv.pushFrame(jpegData, jpegLen);       // viewers see it at http://host:8090/
 *   ...
 *   srv.stop();
 * @endcode
 */
class MediaStreamServer {
public:
    // Both special members are out-of-line so the incomplete Client type only has
    // to be complete inside libnetwork_stream.cpp (the unique_ptr<Client> members
    // are destroyed there, never in a translation unit that just includes this).
    MediaStreamServer();
    ~MediaStreamServer();

    MediaStreamServer(const MediaStreamServer&)            = delete;
    MediaStreamServer& operator=(const MediaStreamServer&) = delete;

    /** @brief Bind, listen and start accepting viewers. @return true on success. */
    bool start(const StreamServerConfig& cfg, const dashcam::log::LogCallback& log = {});

    /** @brief Stop accepting, disconnect all viewers, join all threads. Idempotent. */
    void stop();

    /**
     * @brief Fan one compressed frame out to every connected viewer.
     *
     * MjpegHttp: @p data must be one complete JPEG (JFIF).  RawTcp: any bytes,
     * concatenated into the per-viewer stream.  No-op if not running or with no
     * viewers.  Never blocks on a slow viewer — that viewer's oldest queued
     * frames are dropped instead.
     */
    void pushFrame(const void* data, size_t len);

    /** @brief Number of currently connected viewers. */
    int      clientCount() const;
    /** @brief Actual bound port (useful after start() with port 0). */
    uint16_t port() const { return server_.port(); }
    bool     isRunning() const { return running_.load(); }

private:
    struct Client;                       ///< Defined in libnetwork_stream.cpp.

    void acceptLoop();
    void clientLoop(Client* c);
    void reapDoneLocked();               ///< clientsMtx_ must be held.

    StreamServerConfig        cfg_;
    dashcam::log::LogCallback log_;
    TcpServer                 server_;
    std::thread               acceptThread_;
    std::atomic<bool>         running_{false};

    mutable std::mutex                    clientsMtx_;
    std::vector<std::unique_ptr<Client>>  clients_;
};

// ─── RTP output control (inference-camera streaming) ───────────────────────────

/**
 * @brief Parameters for an H.264-over-RTP output stream.
 *
 * This is the "inference camera" streaming path: those feeds are raw, so
 * streaming means software-encoding them (x264enc — Orin Nano has no NVENC) and
 * sending RTP over UDP.  The actual GStreamer elements live in the camera
 * pipeline; libnetwork owns the control/session layer (destination, bitrate,
 * enable) and produces the branch description, the SDP, and a viewer hint.
 */
struct RtpStreamConfig {
    bool        enabled     = false;         ///< Whether the RTP branch should be attached.
    std::string host        = "127.0.0.1";   ///< Destination: unicast IP or a multicast group.
    uint16_t    port        = 5600;          ///< Destination UDP port.
    int         bitrateKbps = 4000;          ///< x264 target bitrate (kbps).
    int         keyIntSec   = 2;             ///< Keyframe interval in seconds (mid-stream join latency).
};

/**
 * @brief Control/session object for one H.264-over-RTP output stream.
 *
 * Holds the destination and encoder parameters and produces:
 *   - branchDescription(): the GStreamer bin (no leading queue — Camera_GST's
 *     addBranch() inserts queue+valve) that encodes raw video and udpsinks RTP.
 *   - sdp() / viewerHint(): what an operator needs to receive the stream.
 *
 * Pure string/parameter logic — no GStreamer dependency here; the application
 * parses branchDescription() with gst_parse_bin_from_description() and attaches
 * it to the inference camera's tee via Camera_GST::addBranch(..., leaky=true).
 */
class RtpSession {
public:
    RtpSession() = default;
    explicit RtpSession(const RtpStreamConfig& cfg) : cfg_(cfg) {}

    void                   configure(const RtpStreamConfig& cfg) { cfg_ = cfg; }
    const RtpStreamConfig& config() const { return cfg_; }

    /**
     * @brief GStreamer bin description: raw video → x264enc → rtph264pay → udpsink.
     *
     * @param nvmm  true for an NVMM (CSI/Argus) tee source → head with nvvidconv;
     *              false for a system-memory raw (USB) source → head with videoconvert.
     * @param fps   Source frame rate; sets key-int-max = keyIntSec*fps (<=0 → 30 fps).
     */
    std::string branchDescription(bool nvmm, float fps = 0.0f) const;

    /** @brief An SDP a receiver can save and open (ffplay/VLC). */
    std::string sdp() const;

    /** @brief A ready-to-run gst-launch receiver command for this stream. */
    std::string viewerHint() const;

private:
    RtpStreamConfig cfg_;
};

// ─── WiFi / connectivity control ───────────────────────────────────────────────

/**
 * @brief Whether the system reached the network at startup.
 *
 * "Offline mode" is the app's response to a WiFi connect that could not be
 * fulfilled: internet time-sync and network streaming are skipped.
 */
enum class ConnectivityState { Online, Offline };

/** @brief Parameters for connectWifi(). */
struct WifiConnectConfig {
    bool        enabled         = true;  ///< Attempt the connect at all.
    std::string ssid;                    ///< Target SSID; empty = the current/last-used SSID.
    int         timeoutSec      = 20;    ///< Max wait for association (passed to nmcli -w).
    bool        requireInternet = false; ///< Require full internet (nmcli connectivity==full), not just association.
};

/** @brief Result of connectWifi(). */
struct WifiStatus {
    ConnectivityState state = ConnectivityState::Offline;
    std::string       ssid;    ///< Connected SSID (empty when Offline).
    std::string       detail;  ///< Human-readable reason, for logging.
};

/**
 * @brief The current WiFi SSID: the active connection, or the last-used saved
 *        WiFi profile if none is active.
 *
 * Queried from NetworkManager via nmcli.  Returns "" if nmcli/NetworkManager is
 * unavailable or no WiFi profile is known.
 */
std::string currentWifiSsid(const dashcam::log::LogCallback& log = {});

/**
 * @brief Ask the OS (NetworkManager, via nmcli) to connect to a WiFi SSID.
 *
 * Connects to @p cfg.ssid, or the current SSID (currentWifiSsid()) when it is
 * empty.  Uses the saved profile's stored credentials.  Behaviour:
 *   - Already associated to the target: returns Online WITHOUT touching the link
 *     (non-destructive fast path — the common case at startup).
 *   - Otherwise issues `nmcli -w <timeout> device wifi connect <ssid>` and
 *     re-checks the state.
 *   - nmcli/NetworkManager unavailable, no known SSID, or the attempt fails:
 *     returns Offline with a reason.  Never throws.
 *
 * Runs nmcli via fork/exec with an argv array — no shell — so an SSID with
 * arbitrary characters cannot inject a command.
 */
WifiStatus connectWifi(const WifiConnectConfig& cfg, const dashcam::log::LogCallback& log = {});

// ─── SNTP internet-time client ─────────────────────────────────────────────────

/**
 * @brief Result of an SNTP time query.
 *
 * @c offsetSeconds is (server time − local system time): add it to the local
 * clock to correct it.  @c unixSeconds / @c unixNanos are the server's transmit
 * timestamp in the Unix epoch (the value stepSystemClock() would write).
 */
struct TimeResult {
    bool        valid            = false;  ///< True only if a well-formed reply was parsed.
    int64_t     unixSeconds      = 0;      ///< Server transmit time, whole seconds (Unix epoch).
    uint32_t    unixNanos        = 0;      ///< Fractional part of the transmit time (ns).
    double      offsetSeconds    = 0.0;    ///< server − local; add to local to correct it.
    double      roundTripSeconds = 0.0;    ///< Measured request/response round-trip time.
    std::string server;                    ///< Server that was queried.
};

/**
 * @brief Query an SNTP / NTP server for the current time.
 *
 * Sends a mode-3 (client) SNTP request over UDP and parses the mode-4 reply,
 * computing the clock offset and round-trip time from the four RFC 4330
 * timestamps (originate / receive / transmit / destination).  The echoed
 * originate timestamp and UDP source address/port are verified to reject stale
 * or unrelated packets.  This is correlation, not cryptographic authentication,
 * and the function does not change the system clock.
 *
 * @param server     Hostname or IP of the time server.
 * @param port       UDP port (123 for standard NTP).
 * @param timeoutMs  Maximum wait for the reply.
 * @param log        Optional log callback.
 * @return A TimeResult with valid == true on success; valid == false on any
 *         resolution / send / timeout / malformed-reply failure.
 */
TimeResult queryTime(const std::string& server = "pool.ntp.org",
                     uint16_t port = 123, int timeoutMs = 3000,
                     const dashcam::log::LogCallback& log = {});

/**
 * @brief Step the system real-time clock to @p t's transmit timestamp.
 *
 * @warning Do not pass a result from plain queryTime() here: SNTP has no
 * cryptographic server authentication.  This low-level helper is retained only
 * for callers whose TimeResult came from an independently authenticated source.
 *
 * Calls clock_settime(CLOCK_REALTIME); this requires CAP_SYS_TIME (root).  The
 * clock jumps rather than slews — intended for a dashcam booting with a wrong /
 * unset RTC, where correct footage and log timestamps matter more than
 * monotonic continuity.
 *
 * @return true on success; false if @p t is invalid or the call is not permitted
 *         (EPERM — run as root), with the reason logged when a callback is given.
 */
bool stepSystemClock(const TimeResult& t, const dashcam::log::LogCallback& log = {});

} // namespace dashcam::network

#endif // LIBNETWORK_H
