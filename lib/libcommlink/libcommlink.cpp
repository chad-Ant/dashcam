#include "libcommlink.h"

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <filesystem>

// ─── file-local helpers ───────────────────────────────────────────────────────

namespace {

namespace fs = std::filesystem;

void doLog(const dashcam::log::LogCallback& cb, dashcam::log::LogLevel lvl,
           const char* fmt, ...) {
    if (!cb) return;
    char buf[512];
    va_list ap; va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    cb(lvl, buf);
}

int64_t steadyMs() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

dashcam::log::LogLevel toLogLevel(uint8_t bridgeLevel) {
    switch (bridgeLevel) {
    case hostproto::LOG_DEBUG: return dashcam::log::LogLevel::DEBUG;
    case hostproto::LOG_WARN:  return dashcam::log::LogLevel::WARN;
    case hostproto::LOG_ERROR: return dashcam::log::LogLevel::ERROR;
    default:                   return dashcam::log::LogLevel::INFO;
    }
}

constexpr int  POLL_SLICE_MS = 100;  ///< Bounds how long a stop() waits and paces the watchdog.
constexpr size_t READ_CHUNK  = 512;  ///< One CDC-ACM read; frames are at most 261 bytes.
constexpr int  TX_TIMEOUT_MS = 250;  ///< Deadline for writing one command frame (6-7 bytes).

} // namespace

// ─── dashcam::commlink ────────────────────────────────────────────────────────

namespace dashcam::commlink {

CommLink::CommLink()  { hostproto::rxInit(m_rx); }
CommLink::~CommLink() { close(); }

// ─── device discovery ─────────────────────────────────────────────────────────

std::vector<std::string> CommLink::enumerate(const std::string& idMatch,
                                             bool includeAcmFallback) {
    std::vector<std::string> out;

    // This runs inside the RX thread's reconnect loop.  directory_iterator's
    // increment throws on a mid-scan error (a device node disappearing as it is
    // walked is exactly what USB does), and an escaped exception there would
    // std::terminate the process — so every scan is contained.
    try {
        std::error_code ec;

        // Preferred: the stable by-id symlink, which encodes the device's own
        // USB descriptor strings rather than the kernel's enumeration order.
        const fs::path byId{"/dev/serial/by-id"};
        if (fs::exists(byId, ec)) {
            std::vector<std::string> matches;
            for (const auto& entry : fs::directory_iterator(byId)) {
                const std::string name = entry.path().filename().string();
                if (!idMatch.empty() && name.find(idMatch) == std::string::npos) continue;
                ec.clear();
                const fs::path real = fs::canonical(entry.path(), ec);
                matches.push_back(ec ? entry.path().string() : real.string());
            }
            std::sort(matches.begin(), matches.end());
            out.insert(out.end(), matches.begin(), matches.end());
        }
    } catch (const std::exception&) {
        // Fall through to the ttyACM scan with whatever was collected.
    }

    // Fallback: any CDC-ACM node.  Correct on a rig where the C3 is the only
    // ACM device; ambiguous otherwise, which is why by-id is tried first and
    // why this is opt-in — probing takes each candidate exclusively.
    if (!includeAcmFallback) return out;

    try {
        std::vector<std::string> acm;
        for (const auto& entry : fs::directory_iterator("/dev")) {
            const std::string name = entry.path().filename().string();
            if (name.rfind("ttyACM", 0) != 0) continue;
            const std::string path = entry.path().string();
            if (std::find(out.begin(), out.end(), path) == out.end()) acm.push_back(path);
        }
        std::sort(acm.begin(), acm.end());
        out.insert(out.end(), acm.begin(), acm.end());
    } catch (const std::exception&) {
        // Keep the by-id results.
    }

    return out;
}

// ─── lifecycle ────────────────────────────────────────────────────────────────

bool CommLink::open(const CommLinkConfig& cfg, const dashcam::log::LogCallback& log) {
    close();

    m_cfg = cfg;
    m_log = log;

    if (m_cfg.decimation == 0) {
        doLog(m_log, dashcam::log::LogLevel::WARN,
              "CommLink::open: decimation 0 is invalid; using 1");
        m_cfg.decimation = 1;
    }
    // The bridge drops the host after 5 s of frame silence, so a keepalive
    // slower than that would make the link flap even on a healthy cable.
    if (m_cfg.keepaliveMs <= 0 || m_cfg.keepaliveMs > 4000) {
        doLog(m_log, dashcam::log::LogLevel::WARN,
              "CommLink::open: keepaliveMs %d out of range; using 1000",
              m_cfg.keepaliveMs);
        m_cfg.keepaliveMs = 1000;
    }

    if (::pipe(m_pipe) != 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "CommLink::open: pipe() failed: %s", ::strerror(errno));
        return false;
    }
    // Non-blocking read end so drains never stall the RX thread.
    const int flags = ::fcntl(m_pipe[0], F_GETFL, 0);
    if (flags >= 0) ::fcntl(m_pipe[0], F_SETFL, flags | O_NONBLOCK);

    m_decimation.store(m_cfg.decimation);
    m_helloOk.store(false);
    {
        std::lock_guard<std::mutex> lk(m_statsMtx);
        m_stats     = LinkStats{};
        m_openCount = 0;
    }
    {
        std::lock_guard<std::mutex> lk(m_dataMtx);
        m_lastTelemetryMs = -1;
        m_lastFrameMs     = -1;
    }
    m_hasTelemetry.store(false);

    // A closed port is a normal starting state — the RX thread keeps retrying —
    // so the return value is "opened now", not "usable".
    return openPort();
}

bool CommLink::openPort() {
    std::lock_guard<std::mutex> lk(m_portMtx);
    if (m_uart.isOpen()) return true;

    std::vector<std::string> candidates;
    if (!m_cfg.device.empty()) candidates.push_back(m_cfg.device);
    else                        candidates = enumerate(m_cfg.idMatch, m_cfg.allowAcmFallback);

    if (candidates.empty()) return false;

    // Start at the probe cursor and wrap.  A candidate that opens but fails the
    // handshake advances the cursor, so the scan moves on instead of retrying
    // the same unresponsive device every reconnect interval while the real
    // bridge sits untried further down the list.
    if (candidates.size() > 1) {
        const size_t start = m_probeCursor.load() % candidates.size();
        std::rotate(candidates.begin(), candidates.begin() + static_cast<long>(start),
                    candidates.end());
    }

    dashcam::uart::UartConfig ucfg;
    ucfg.baudRate  = m_cfg.baudRate;
    ucfg.exclusive = m_cfg.exclusive;
    // Critical for the ESP32-C3: HUPCL drops DTR/RTS on close, and on the C3's
    // native USB Serial/JTAG that line pair is the reset/download-mode request
    // esptool uses.  Leaving it set would reboot the bridge every time this
    // process exits or the port is reopened.
    ucfg.hangupOnClose = false;

    for (const auto& path : candidates) {
        // Pass no log callback: a failed probe during auto-discovery is an
        // expected outcome, not an error worth a line per second.
        if (!m_uart.open(path, ucfg, {})) continue;

        m_devicePath = path;
        hostproto::rxInit(m_rx);
        m_uart.flush();
        // Each candidate must prove itself again; a previous session's HELLO
        // says nothing about the device now behind this node.
        m_helloOk.store(false);

        {
            std::lock_guard<std::mutex> sl(m_statsMtx);
            if (m_openCount++ > 0) ++m_stats.reconnects;
        }
        // The RX thread sends CMD_HELLO on the next pass; it cannot be sent from
        // here because sendFrame() takes m_portMtx, which this function holds.
        m_announcePending.store(true);
        doLog(m_log, dashcam::log::LogLevel::INFO,
              "CommLink: bridge port open: %s", path.c_str());
        return true;
    }

    return false;
}

void CommLink::closePort() {
    std::lock_guard<std::mutex> lk(m_portMtx);
    if (!m_uart.isOpen()) return;
    m_uart.close();
    doLog(m_log, dashcam::log::LogLevel::WARN,
          "CommLink: bridge port closed: %s", m_devicePath.c_str());
    m_devicePath.clear();
}

bool CommLink::start() {
    if (m_running.load()) return false;
    // open() creates the wake pipe even when the device node is absent, so this
    // only fires when open() was never called (or the pipe itself failed).
    if (m_pipe[0] < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "CommLink::start: call open() first");
        return false;
    }
    m_running.store(true);
    m_thread = std::thread(&CommLink::rxLoop, this);
    return true;
}

void CommLink::stop() {
    if (!m_running.load()) return;
    m_running.store(false);
    wake();
    if (m_thread.joinable()) m_thread.join();

    // Mark disconnected without firing the callback: the thread that would
    // deliver it is already gone, and a notification raised from inside
    // ~CommLink() is a trap for callbacks that captured surrounding state.
    m_connected.store(false);
}

void CommLink::close() {
    stop();
    closePort();
    for (int& fd : m_pipe) {
        if (fd >= 0) { ::close(fd); fd = -1; }
    }
}

void CommLink::wake() {
    if (m_pipe[1] < 0) return;
    const uint8_t b = 1;
    const ssize_t n = ::write(m_pipe[1], &b, 1);
    (void)n; // a full wake pipe already means "wake up"; nothing to recover
}

// ─── callbacks ────────────────────────────────────────────────────────────────

void CommLink::setTelemetryCallback(TelemetryCallback cb) {
    std::lock_guard<std::mutex> lk(m_cbMtx); m_telemetryCb = std::move(cb);
}
void CommLink::setStatusCallback(StatusCallback cb) {
    std::lock_guard<std::mutex> lk(m_cbMtx); m_statusCb = std::move(cb);
}
void CommLink::setHelloCallback(HelloCallback cb) {
    std::lock_guard<std::mutex> lk(m_cbMtx); m_helloCb = std::move(cb);
}
void CommLink::setBridgeLogCallback(BridgeLogCallback cb) {
    std::lock_guard<std::mutex> lk(m_cbMtx); m_bridgeLogCb = std::move(cb);
}
void CommLink::setConnectionCallback(ConnectionCallback cb) {
    std::lock_guard<std::mutex> lk(m_cbMtx); m_connCb = std::move(cb);
}

// ─── commands ─────────────────────────────────────────────────────────────────

bool CommLink::sendFrame(uint8_t type, const uint8_t* payload, uint8_t len) {
    uint8_t frame[hostproto::MAX_FRAME];
    const size_t n = hostproto::buildFrame(type, payload, len, frame, sizeof(frame));
    if (n == 0) {
        std::lock_guard<std::mutex> lk(m_statsMtx); ++m_stats.txErrors;
        return false;
    }

    bool ok = false;
    {
        std::lock_guard<std::mutex> lk(m_portMtx);
        // Bounded write, not the blocking one.  m_portMtx is held here and the
        // RX thread needs it to read: a plain write() into a stalled CDC
        // endpoint would block in the kernel forever, starve the receiver, and
        // hang stop() at join() during shutdown.  Commands are 6-7 bytes, so
        // anything approaching this deadline means the peer is gone.
        ok = m_uart.isOpen() && m_uart.writeTimeout(frame, n, TX_TIMEOUT_MS);
    }

    std::lock_guard<std::mutex> lk(m_statsMtx);
    if (ok) ++m_stats.txFrames; else ++m_stats.txErrors;
    return ok;
}

bool CommLink::requestOnce()   { return sendFrame(hostproto::CMD_GET_ONCE,     nullptr, 0); }
bool CommLink::requestStatus() { return sendFrame(hostproto::CMD_GET_STATUS,   nullptr, 0); }
bool CommLink::startStream()   { return sendFrame(hostproto::CMD_START_STREAM, nullptr, 0); }
bool CommLink::stopStream()    { return sendFrame(hostproto::CMD_STOP_STREAM,  nullptr, 0); }
bool CommLink::ping()          { return sendFrame(hostproto::CMD_PING,         nullptr, 0); }

bool CommLink::setDecimation(uint8_t n) {
    if (n == 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "CommLink::setDecimation: 0 is invalid (use stopStream())");
        return false;
    }
    m_decimation.store(n); // re-applied by the RX thread after a reconnect
    return sendFrame(hostproto::CMD_SET_DECIM, &n, 1);
}

bool CommLink::setCanMode(uint8_t mode) {
    if (mode < 1 || mode > 3) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "CommLink::setCanMode: expected 1 (discover), 2 (sniff) or 3 (obd2)");
        return false;
    }
    // Deliberately NOT cached and re-applied on reconnect, unlike decimation.
    // Decimation is a preference about this link; CAN mode is a decision about
    // the vehicle bus, and mode 3 makes the MKR transmit on it. Silently
    // restoring that after a cable glitch would put a node back on a live bus
    // with nobody having asked for it in that session. The MKR boots into
    // listen-only sniffing on its own; if the host wants OBD2 again it asks.
    return sendFrame(hostproto::CMD_SET_CAN_MODE, &mode, 1);
}

// ─── state ────────────────────────────────────────────────────────────────────

bool CommLink::isOpen() const {
    std::lock_guard<std::mutex> lk(m_portMtx);
    return m_uart.isOpen();
}

bool CommLink::isRunning()    const { return m_running.load(); }
bool CommLink::isConnected()  const { return m_connected.load(); }
bool CommLink::hasTelemetry() const { return m_hasTelemetry.load(); }

Telemetry CommLink::telemetry() const {
    std::lock_guard<std::mutex> lk(m_dataMtx);
    return m_telemetry;
}

BridgeStatus CommLink::status() const {
    std::lock_guard<std::mutex> lk(m_dataMtx);
    return m_status;
}

int64_t CommLink::telemetryAgeMs() const {
    std::lock_guard<std::mutex> lk(m_dataMtx);
    if (m_lastTelemetryMs < 0) return -1;
    return steadyMs() - m_lastTelemetryMs;
}

bool CommLink::isStale(int ms) const {
    const int64_t age = telemetryAgeMs();
    if (age < 0) return true; // never received anything
    return age > (ms < 0 ? m_cfg.staleMs : ms);
}

LinkStats CommLink::stats() const {
    std::lock_guard<std::mutex> lk(m_statsMtx);
    return m_stats;
}

std::string CommLink::devicePath() const {
    std::lock_guard<std::mutex> lk(m_portMtx);
    return m_devicePath;
}

void CommLink::setConnected(bool connected) {
    if (m_connected.exchange(connected) == connected) return;

    ConnectionCallback cb;
    {
        std::lock_guard<std::mutex> lk(m_cbMtx);
        cb = m_connCb;
    }
    doLog(m_log, connected ? dashcam::log::LogLevel::INFO : dashcam::log::LogLevel::WARN,
          "CommLink: bridge %s", connected ? "connected" : "disconnected");
    if (cb) cb(connected);
}

// ─── frame handling ───────────────────────────────────────────────────────────

void CommLink::feed(const uint8_t* data, size_t len, uint32_t nowMs) {
    uint8_t type = 0, plen = 0;
    for (size_t i = 0; i < len; ++i) {
        const hostproto::Status s =
            hostproto::rxByte(m_rx, data[i], nowMs, type,
                              m_payload, static_cast<uint8_t>(sizeof(m_payload)), plen);

        if (s == hostproto::Status::NOK_CRC) {
            std::lock_guard<std::mutex> lk(m_statsMtx); ++m_stats.crcErrors;
            continue;
        }
        if (s != hostproto::Status::FRAME_READY) continue;

        {
            std::lock_guard<std::mutex> lk(m_statsMtx); ++m_stats.framesRx;
        }
        {
            std::lock_guard<std::mutex> lk(m_dataMtx); m_lastFrameMs = steadyMs();
        }
        setConnected(true);
        onFrame(type, m_payload, plen);
    }
}

void CommLink::onFrame(uint8_t type, const uint8_t* payload, uint8_t len) {
    // Copy the callback slot under the lock and invoke the copy outside it, so
    // application code never runs with a library mutex held.
    auto grab = [this](auto member) {
        std::lock_guard<std::mutex> lk(m_cbMtx);
        return this->*member;
    };

    switch (type) {
    case hostproto::MSG_TELEMETRY: {
        if (static_cast<size_t>(len) != sizeof(Telemetry)) {
            std::lock_guard<std::mutex> lk(m_statsMtx); ++m_stats.malformedRx;
            doLog(m_log, dashcam::log::LogLevel::WARN,
                  "CommLink: MSG_TELEMETRY length %u, expected %zu",
                  static_cast<unsigned>(len), sizeof(Telemetry));
            return;
        }
        Telemetry t;
        std::memcpy(&t, payload, sizeof(t)); // packed struct: copy, never alias
        {
            std::lock_guard<std::mutex> lk(m_dataMtx);
            m_telemetry       = t;
            m_lastTelemetryMs = steadyMs();
        }
        m_hasTelemetry.store(true);
        { std::lock_guard<std::mutex> lk(m_statsMtx); ++m_stats.telemetryRx; }

        if (auto cb = grab(&CommLink::m_telemetryCb)) cb(t);
        break;
    }

    case hostproto::MSG_STATUS: {
        if (static_cast<size_t>(len) != sizeof(BridgeStatus)) {
            std::lock_guard<std::mutex> lk(m_statsMtx); ++m_stats.malformedRx;
            return;
        }
        BridgeStatus s;
        std::memcpy(&s, payload, sizeof(s));
        {
            std::lock_guard<std::mutex> lk(m_dataMtx);
            m_status = s;
        }
        { std::lock_guard<std::mutex> lk(m_statsMtx); ++m_stats.statusRx; }

        if (auto cb = grab(&CommLink::m_statusCb)) cb(s);
        break;
    }

    case hostproto::MSG_LOG: {
        if (static_cast<size_t>(len) < sizeof(hostproto::LogHeader)) {
            std::lock_guard<std::mutex> lk(m_statsMtx); ++m_stats.malformedRx;
            return;
        }
        const uint8_t     level = payload[0];
        const std::string text(reinterpret_cast<const char*>(payload + 1),
                               static_cast<size_t>(len) - sizeof(hostproto::LogHeader));
        { std::lock_guard<std::mutex> lk(m_statsMtx); ++m_stats.logRx; }

        if (m_cfg.forwardBridgeLogs)
            doLog(m_log, toLogLevel(level), "[c3] %s", text.c_str());
        if (auto cb = grab(&CommLink::m_bridgeLogCb)) cb(toLogLevel(level), text);
        break;
    }

    case hostproto::MSG_HELLO: {
        if (static_cast<size_t>(len) != sizeof(Hello)) {
            std::lock_guard<std::mutex> lk(m_statsMtx); ++m_stats.malformedRx;
            return;
        }
        Hello h;
        std::memcpy(&h, payload, sizeof(h));

        doLog(m_log, dashcam::log::LogLevel::INFO,
              "CommLink: bridge fw %u.%u proto %u  boot #%u  reset reason %u  uptime %u ms",
              h.fwMajor, h.fwMinor, h.protoVersion, h.bootCount, h.resetReason,
              h.bridgeMillis);

        // The wire self-check: a bridge built against a different contract is
        // detected here rather than by silently misreading every field.
        const bool compatible = (h.protoVersion   == hostproto::VERSION &&
                                 h.telemetryBytes == sizeof(Telemetry) &&
                                 h.statusBytes    == sizeof(BridgeStatus));
        if (!compatible) {
            doLog(m_log, dashcam::log::LogLevel::ERROR,
                  "CommLink: PROTOCOL MISMATCH — bridge proto %u tlm %u status %u, "
                  "host proto %u tlm %zu status %zu; reflash the C3 with the matching "
                  "HostProtocol.h",
                  h.protoVersion, h.telemetryBytes, h.statusBytes,
                  hostproto::VERSION, sizeof(Telemetry), sizeof(BridgeStatus));
            // Deliberately do NOT set m_helloOk: an incompatible peer must fail
            // the discovery handshake and be released, not be driven anyway.
            // Streaming from it would decode every field at the wrong offset.
        } else {
            m_helloOk.store(true);

            // MSG_HELLO means the bridge reset its session state, so streaming
            // must be re-established here — this is what makes a C3 reboot
            // invisible to the application.
            if (m_cfg.autoStream) {
                const uint8_t decim = m_decimation.load();
                if (decim != 1) (void)setDecimation(decim);
                (void)startStream();
            }
        }

        // Fired either way: the application (and the test) must be able to see
        // an incompatible bridge, not just find the link mysteriously idle.
        if (auto cb = grab(&CommLink::m_helloCb)) cb(h);
        break;
    }

    case hostproto::MSG_NACK: {
        { std::lock_guard<std::mutex> lk(m_statsMtx); ++m_stats.nacksRx; }
        if (static_cast<size_t>(len) == sizeof(hostproto::Nack)) {
            doLog(m_log, dashcam::log::LogLevel::WARN,
                  "CommLink: bridge NACKed %s (reason %u)",
                  hostproto::typeName(payload[0]), static_cast<unsigned>(payload[1]));
        }
        break;
    }

    case hostproto::MSG_PONG:
        break; // liveness only; the frame itself already refreshed the timer

    default:
        doLog(m_log, dashcam::log::LogLevel::DEBUG,
              "CommLink: ignoring unknown frame type 0x%02X (%u bytes)",
              static_cast<unsigned>(type), static_cast<unsigned>(len));
        break;
    }
}

// ─── RX thread ────────────────────────────────────────────────────────────────

void CommLink::rxLoop() {
    int64_t lastKeepaliveMs   = 0;
    int64_t lastAttemptMs     = 0;
    int64_t handshakeDeadline = 0; ///< 0 = no handshake in progress.

    while (m_running.load()) {
        int fd = -1;
        {
            std::lock_guard<std::mutex> lk(m_portMtx);
            fd = m_uart.isOpen() ? m_uart.fd() : -1;
        }

        // ── Disconnected: retry on a timer, sleeping on the wake pipe so that
        //    stop() returns promptly instead of waiting out reconnectMs. ──────
        if (fd < 0) {
            setConnected(false);

            const int64_t now = steadyMs();
            if (now - lastAttemptMs >= m_cfg.reconnectMs) {
                lastAttemptMs = now;
                if (openPort()) continue; // the announce below runs on the next pass
            }

            struct pollfd wp;
            wp.fd = m_pipe[0]; wp.events = POLLIN; wp.revents = 0;
            const int waitMs = std::min<int>(POLL_SLICE_MS, m_cfg.reconnectMs);
            if (::poll(&wp, 1, waitMs) > 0 && (wp.revents & POLLIN)) {
                uint8_t drain[64];
                while (::read(m_pipe[0], drain, sizeof(drain)) > 0) { }
            }
            continue;
        }

        // Announce the session on every (re)open, including one that happened on
        // the application thread in open().  This is what makes reconnection
        // deterministic: the bridge resets its streaming state and answers
        // MSG_HELLO, which re-arms autoStream.  Waiting for the bridge's
        // host-silence timeout instead would hang a restart that reopens the
        // port faster than that timeout.
        if (m_announcePending.exchange(false)) {
            (void)sendFrame(hostproto::CMD_HELLO, nullptr, 0);
            lastKeepaliveMs   = steadyMs();
            handshakeDeadline = lastKeepaliveMs + m_cfg.handshakeMs;
        }

        // Handshake gate.  A candidate that opened but has not identified itself
        // with a COMPATIBLE MSG_HELLO within handshakeMs is rejected: opening a
        // tty proves nothing about what is on the other end, and without this
        // the library would sit on an unrelated ACM device indefinitely — the
        // connection watchdog below cannot help, because it only arms once a
        // frame has arrived.
        //
        // The gate is m_helloOk, NOT m_connected.  m_connected rises on any
        // CRC-valid frame, which a bridge running mismatched firmware — or a
        // foreign device whose output happens to frame and checksum correctly —
        // would also satisfy, letting exactly the peers this check exists to
        // exclude walk straight through it.
        if (handshakeDeadline != 0 && !m_helloOk.load() &&
            steadyMs() > handshakeDeadline) {
            doLog(m_log, dashcam::log::LogLevel::WARN,
                  "CommLink: %s did not identify itself in %d ms — trying the next candidate",
                  devicePath().c_str(), m_cfg.handshakeMs);
            m_probeCursor.fetch_add(1);
            handshakeDeadline = 0;
            setConnected(false);   // any frames it did send do not make it our bridge
            closePort();
            continue;
        }
        if (m_helloOk.load()) handshakeDeadline = 0; // handshake satisfied

        // ── Connected: wait for data, the wake pipe, or a slice timeout. ──────
        struct pollfd pfds[2];
        pfds[0].fd = fd;         pfds[0].events = POLLIN; pfds[0].revents = 0;
        pfds[1].fd = m_pipe[0];  pfds[1].events = POLLIN; pfds[1].revents = 0;

        const int ret = ::poll(pfds, 2, POLL_SLICE_MS);
        if (ret < 0) {
            if (errno == EINTR) continue;
            doLog(m_log, dashcam::log::LogLevel::ERROR,
                  "CommLink: poll failed: %s", ::strerror(errno));
            closePort();
            continue;
        }

        if (pfds[1].revents & POLLIN) {
            uint8_t drain[64];
            while (::read(m_pipe[0], drain, sizeof(drain)) > 0) { }
            if (!m_running.load()) break;
        }

        // Drain before acting on a hangup: POLLIN and POLLHUP arrive together
        // when the device sent its last bytes and then detached, and those
        // bytes may hold a complete frame.
        if (pfds[0].revents & POLLIN) {
            uint8_t buf[READ_CHUNK];
            int n = -1;
            {
                std::lock_guard<std::mutex> lk(m_portMtx);
                if (m_uart.isOpen()) n = m_uart.read(buf, sizeof(buf), 0);
            }
            if (n < 0) {
                doLog(m_log, dashcam::log::LogLevel::WARN,
                      "CommLink: read failed (%s) — bridge detached",
                      ::strerror(errno));
                closePort();
                setConnected(false);
                continue;
            }
            if (n > 0) {
                {
                    std::lock_guard<std::mutex> lk(m_statsMtx);
                    m_stats.bytesRx += static_cast<uint64_t>(n);
                }
                feed(buf, static_cast<size_t>(n), static_cast<uint32_t>(steadyMs()));
            }
            // n == 0 is either EOF or a spurious readable wakeup.  The two are
            // indistinguishable here, so the POLLHUP check below decides — a
            // spurious wakeup costs one loop iteration, not a reconnect.
        }

        // Hangup/error means the node is gone; without closing here poll() would
        // return immediately forever and spin this thread at 100 % CPU.
        if (pfds[0].revents & (POLLHUP | POLLERR | POLLNVAL)) {
            doLog(m_log, dashcam::log::LogLevel::WARN,
                  "CommLink: device hangup (revents 0x%X)",
                  static_cast<unsigned>(pfds[0].revents));
            closePort();
            setConnected(false);
            continue;
        }

        const int64_t now = steadyMs();

        // Keepalive: the bridge drops the host after 5 s of silence, and the
        // PONG it sends back is also this side's liveness evidence.
        if (now - lastKeepaliveMs >= m_cfg.keepaliveMs) {
            lastKeepaliveMs = now;
            (void)ping();
        }

        // Connection watchdog: the cable can stay enumerated while the C3 sits
        // in a reset loop, so silence — not fd state — defines "disconnected".
        if (m_connected.load()) {
            int64_t lastFrame = -1;
            {
                std::lock_guard<std::mutex> lk(m_dataMtx);
                lastFrame = m_lastFrameMs;
            }
            if (lastFrame >= 0 && (now - lastFrame) > m_cfg.connectionTimeoutMs)
                setConnected(false);
        }
    }
}

} // namespace dashcam::commlink
