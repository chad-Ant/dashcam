// commlink_sim_test — CommLink (libcommlink + libuart + HostProtocol.h) against
// a simulated ESP32-C3 bridge, no hardware.
//
// The fake bridge owns the master side of a pseudo-terminal; CommLink opens the
// slave side exactly as it opens the C3's /dev/ttyACM node, and the fake speaks
// the bridge's half of HostProtocol.h (the rules of
// peripherals/esp32-c3/lib/hostLink/hostLink.cpp): MSG_HELLO on CMD_HELLO, PONG
// on PING, telemetry while streaming, NACK on a bad argument.  Unplugging is
// closing the master (the slave sees a hangup, as with a USB detach); plugging
// back in is a new pty behind a symlink, which is what udev's stable name is.
//
// Covers the handshake and auto-stream, every frame type (and malformed, CRC-bad
// and split frames), the commands and their argument checks, keepalive, the
// silence watchdog, a mismatched and a silent device, hot unplug + replug,
// discovery over a fake /dev tree (a silent candidate is skipped), and a log
// callback that calls back into the link.
//
// Usage: commlink_sim_test        (runs anywhere: cloud sandbox, dev container, Jetson)

#include "libcommlink.h"
#include "libuart.h"

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using namespace dashcam::commlink;
using dashcam::log::LogLevel;

static int g_fails = 0;
static void check(bool ok, const std::string& what) {
    std::printf("  %s  %s\n", ok ? "ok  " : "FAIL", what.c_str());
    if (!ok) ++g_fails;
}

static bool waitFor(const std::function<bool()>& pred, int ms) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end) {
        if (pred()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return pred();
}

static void sleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

// ─── log capture ─────────────────────────────────────────────────────────────

struct LogSink {
    std::mutex               m;
    std::vector<std::string> lines;
    std::function<void(LogLevel, const std::string&)> hook;   ///< Runs inside the callback.
    dashcam::log::LogCallback callback() {
        return [this](LogLevel lvl, const std::string& msg) {
            if (hook) hook(lvl, msg);
            std::lock_guard<std::mutex> lk(m);
            lines.push_back(std::string(lvl == LogLevel::ERROR ? "E " :
                                        lvl == LogLevel::WARN  ? "W " :
                                        lvl == LogLevel::INFO  ? "I " : "D ") + msg);
        };
    }
    bool has(const std::string& needle) {
        std::lock_guard<std::mutex> lk(m);
        return std::any_of(lines.begin(), lines.end(),
                           [&](const std::string& l) { return l.find(needle) != std::string::npos; });
    }
    int count(const std::string& needle) {
        std::lock_guard<std::mutex> lk(m);
        return static_cast<int>(std::count_if(lines.begin(), lines.end(),
                           [&](const std::string& l) { return l.find(needle) != std::string::npos; }));
    }
};

// ─── the simulated bridge ────────────────────────────────────────────────────

class FakeBridge {
public:
    ~FakeBridge() { unplug(); }

    /// A fresh pty (a new USB enumeration); the reader thread starts on it.
    bool plug() {
        unplug();
        const int fd = ::posix_openpt(O_RDWR | O_NOCTTY | O_CLOEXEC);
        if (fd < 0 || ::grantpt(fd) != 0 || ::unlockpt(fd) != 0) {
            if (fd >= 0) ::close(fd);
            return false;
        }
        const char* name = ::ptsname(fd);
        if (!name) { ::close(fd); return false; }
        slave_ = name;
        ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
        master_ = fd;
        hostproto::rxInit(rx_);
        streaming = false;
        run_ = true;
        thread_ = std::thread(&FakeBridge::loop, this);
        return true;
    }

    /// Cable out: the reader stops and the master closes; the slave sees a hangup.
    void unplug() {
        run_ = false;
        if (thread_.joinable()) thread_.join();
        std::lock_guard<std::mutex> lk(txMtx_);
        if (master_ >= 0) { ::close(master_); master_ = -1; }
    }

    const std::string& slave() const { return slave_; }

    // ── behaviour switches ──
    std::atomic<bool>    answerHello{true};
    std::atomic<uint8_t> helloProto{hostproto::VERSION};
    std::atomic<bool>    mute{false};        ///< Send nothing at all (a hung C3).
    std::atomic<bool>    streaming{false};
    std::atomic<uint32_t> telemetrySent{0};

    // ── observations ──
    int count(uint8_t type) {
        std::lock_guard<std::mutex> lk(cmdMtx_);
        return static_cast<int>(std::count_if(cmds_.begin(), cmds_.end(),
                                               [type](const Cmd& c) { return c.type == type; }));
    }
    std::vector<uint8_t> lastPayload(uint8_t type) {
        std::lock_guard<std::mutex> lk(cmdMtx_);
        for (auto it = cmds_.rbegin(); it != cmds_.rend(); ++it)
            if (it->type == type) return it->payload;
        return {};
    }
    std::vector<uint8_t> order() {
        std::lock_guard<std::mutex> lk(cmdMtx_);
        std::vector<uint8_t> t;
        for (const auto& c : cmds_) t.push_back(c.type);
        return t;
    }
    void clearCommands() {
        std::lock_guard<std::mutex> lk(cmdMtx_);
        cmds_.clear();
    }

    // ── sending ──
    void send(uint8_t type, const void* payload, uint8_t len) {
        uint8_t frame[hostproto::MAX_FRAME];
        const size_t n = hostproto::buildFrame(type, static_cast<const uint8_t*>(payload), len,
                                               frame, sizeof(frame));
        sendRaw(frame, n);
    }
    void sendRaw(const uint8_t* bytes, size_t n) {
        std::lock_guard<std::mutex> lk(txMtx_);
        writeLocked(bytes, n);
    }
    /// One frame written in two parts @p gapMs apart, with nothing able to get
    /// in between: a real sender never interleaves another frame into one (the
    /// C3 sends whole frames from its TX ring), but this fake's reader thread
    /// answers PINGs, and a PONG landing mid-frame is a CRC error.
    void sendSplit(const uint8_t* bytes, size_t n, size_t at, int gapMs) {
        std::lock_guard<std::mutex> lk(txMtx_);
        writeLocked(bytes, at);
        sleepMs(gapMs);
        writeLocked(bytes + at, n - at);
    }
    void sendHello(uint8_t proto) {
        hostproto::Hello h{};
        h.protoVersion   = proto;
        h.fwMajor        = 1;
        h.fwMinor        = 0;
        h.resetReason    = 3;
        h.telemetryBytes = static_cast<uint8_t>(sizeof(hostproto::Telemetry));
        h.statusBytes    = static_cast<uint8_t>(sizeof(hostproto::BridgeStatus));
        h.bootCount      = 7;
        h.bridgeMillis   = 12345;
        send(hostproto::MSG_HELLO, &h, sizeof(h));
    }
    static hostproto::Telemetry telemetry(uint32_t n) {
        hostproto::Telemetry t{};
        t.masterMillis = n;
        t.speed        = 42.5f;
        t.flags        = hostproto::TLM_FLAG_TIME_VALID;
        return t;
    }

private:
    struct Cmd { uint8_t type; std::vector<uint8_t> payload; };

    void writeLocked(const uint8_t* bytes, size_t n) {
        if (master_ < 0) return;
        size_t off = 0;
        while (off < n) {
            const ssize_t w = ::write(master_, bytes + off, n - off);
            if (w > 0) { off += static_cast<size_t>(w); continue; }
            if (w < 0 && errno == EAGAIN) { sleepMs(1); continue; }
            return;   // slave closed (EIO): the bytes go nowhere, like a detached cable
        }
    }

    void loop() {
        auto nextTlm = std::chrono::steady_clock::now();
        uint32_t n = 0;
        while (run_) {
            struct pollfd p{master_, POLLIN, 0};
            const int r = ::poll(&p, 1, 10);
            if (r > 0 && (p.revents & POLLIN)) {
                uint8_t buf[256];
                const ssize_t got = ::read(master_, buf, sizeof(buf));
                const uint32_t now = static_cast<uint32_t>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count());
                for (ssize_t i = 0; i < got; ++i) {
                    uint8_t type = 0, len = 0;
                    if (hostproto::rxByte(rx_, buf[i], now, type, payload_, sizeof(payload_), len) ==
                        hostproto::Status::FRAME_READY)
                        onCommand(type, payload_, len);
                }
            } else if (r > 0) {
                sleepMs(5);   // POLLHUP: no slave open right now (host reconnecting)
            }
            if (streaming && !mute && std::chrono::steady_clock::now() >= nextTlm) {
                nextTlm += std::chrono::milliseconds(50);
                const hostproto::Telemetry t = telemetry(++n);
                send(hostproto::MSG_TELEMETRY, &t, sizeof(t));
                ++telemetrySent;
            }
        }
    }

    void onCommand(uint8_t type, const uint8_t* payload, uint8_t len) {
        {
            std::lock_guard<std::mutex> lk(cmdMtx_);
            cmds_.push_back({type, std::vector<uint8_t>(payload, payload + len)});
        }
        if (mute) return;
        auto nack = [&](uint8_t reason) {
            const hostproto::Nack k{type, reason};
            send(hostproto::MSG_NACK, &k, sizeof(k));
        };
        switch (type) {
        case hostproto::CMD_HELLO:
            streaming = false;                           // resetSessionState()
            if (answerHello) sendHello(helloProto);
            break;
        case hostproto::CMD_PING:         send(hostproto::MSG_PONG, nullptr, 0); break;
        case hostproto::CMD_START_STREAM: streaming = true;  break;
        case hostproto::CMD_STOP_STREAM:  streaming = false; break;
        case hostproto::CMD_SET_CAN_MODE:
            if (payload[0] < 1 || payload[0] > 3) nack(hostproto::NACK_BAD_VALUE);
            break;
        default: break;
        }
    }

    int                 master_ = -1;
    std::string         slave_;
    std::atomic<bool>   run_{false};
    std::thread         thread_;
    hostproto::RxState  rx_{};
    uint8_t             payload_[hostproto::MAX_PAYLOAD] = {};
    std::mutex          txMtx_;
    std::mutex          cmdMtx_;
    std::vector<Cmd>    cmds_;
};

// ─── fixtures ────────────────────────────────────────────────────────────────

/// A private directory for symlinks and the fake /dev tree.
struct TempDir {
    fs::path path;
    TempDir() {
        char tmpl[] = "/tmp/commlink_sim_XXXXXX";
        path = ::mkdtemp(tmpl);
    }
    ~TempDir() { std::error_code ec; fs::remove_all(path, ec); }
    /// Points @p name (relative) at @p target, replacing it atomically.
    void link(const std::string& name, const std::string& target) {
        const fs::path dst = path / name;
        fs::create_directories(dst.parent_path());
        const fs::path tmp = dst.string() + ".new";
        std::error_code ec;
        fs::remove(tmp, ec);
        fs::create_symlink(target, tmp);
        fs::rename(tmp, dst);
    }
};

static CommLinkConfig fastConfig(const std::string& device) {
    CommLinkConfig c;
    c.device              = device;
    c.reconnectMs         = 100;
    c.keepaliveMs         = 200;
    c.connectionTimeoutMs = 600;
    c.handshakeMs         = 400;
    c.staleMs             = 300;
    return c;
}

// ─── tests ───────────────────────────────────────────────────────────────────

static void testHandshakeAndStream() {
    std::printf("\n--- handshake, auto-stream, frames ---\n");
    FakeBridge br;
    const bool plugged = br.plug();
    check(plugged, "fake bridge on a pty (" + br.slave() + ")");
    LogSink log;
    CommLink link;
    std::atomic<int> hellos{0}, tlms{0}, statuses{0}, bridgeLogs{0};
    std::atomic<int> ups{0}, downs{0};
    std::string bridgeLogText;
    std::mutex blm;
    link.setHelloCallback([&](const Hello& h) { if (h.protoVersion == hostproto::VERSION) ++hellos; });
    link.setTelemetryCallback([&](const Telemetry&) { ++tlms; });
    link.setStatusCallback([&](const BridgeStatus&) { ++statuses; });
    link.setBridgeLogCallback([&](LogLevel, const std::string& s) {
        std::lock_guard<std::mutex> lk(blm); bridgeLogText = s; ++bridgeLogs; });
    link.setConnectionCallback([&](bool c) { c ? ++ups : ++downs; });

    check(link.open(fastConfig(br.slave()), log.callback()), "open() on the device node returns true");
    check(link.isOpen() && link.devicePath() == br.slave(), "isOpen(), devicePath() is the node");
    check(link.start() && link.isRunning(), "start() runs the RX thread");
    check(!link.start(), "a second start() is refused");

    check(waitFor([&] { return hellos > 0 && ups == 1; }, 1000) && link.isConnected(),
          "CMD_HELLO answered: hello callback, isConnected()");
    check(waitFor([&] { return br.count(hostproto::CMD_START_STREAM) == 1; }, 500),
          "autoStream sends CMD_START_STREAM after a compatible MSG_HELLO");
    const auto ord = br.order();
    check(!ord.empty() && ord.front() == hostproto::CMD_HELLO, "CMD_HELLO is the first frame of the session");
    check(br.count(hostproto::CMD_SET_DECIM) == 0, "no CMD_SET_DECIM at decimation 1");
    check(ups == 1, "connection callback fired once (true)");

    const bool flowing = waitFor([&] { return tlms >= 4; }, 1000);
    check(flowing, "telemetry arrives (" + std::to_string(tlms.load()) + " frames)");
    const Telemetry t = link.telemetry();
    check(link.hasTelemetry() && t.speed == 42.5f && (t.flags & hostproto::TLM_FLAG_TIME_VALID),
          "telemetry() is the newest snapshot (speed 42.5, TIME_VALID)");
    check(link.telemetryAgeMs() >= 0 && link.telemetryAgeMs() < 200 && !link.isStale(),
          "fresh: age " + std::to_string(link.telemetryAgeMs()) + " ms, not stale");

    hostproto::BridgeStatus st{};
    st.bridgeMillis = 777;
    st.decimation   = 1;
    br.send(hostproto::MSG_STATUS, &st, sizeof(st));
    check(waitFor([&] { return statuses == 1 && link.status().bridgeMillis == 777; }, 500),
          "MSG_STATUS: callback, status() snapshot");

    const char text[] = "hello from bridge";
    uint8_t logPayload[64];
    logPayload[0] = hostproto::LOG_WARN;
    std::memcpy(logPayload + 1, text, sizeof(text) - 1);
    br.send(hostproto::MSG_LOG, logPayload, static_cast<uint8_t>(sizeof(text)));
    check(waitFor([&] { return bridgeLogs == 1; }, 500) && log.has("W [c3] hello from bridge"),
          "MSG_LOG: bridge log callback, forwarded to the log as \"[c3] ...\" at its level");

    const hostproto::Nack nk{hostproto::CMD_SET_IMU_MODE, hostproto::NACK_BAD_VALUE};
    br.send(hostproto::MSG_NACK, &nk, sizeof(nk));
    check(waitFor([&] { return link.stats().nacksRx == 1; }, 500), "MSG_NACK counted");
    check(log.has("bridge NACKed CMD_SET_IMU_MODE"), "the NACK names the command (CMD_SET_IMU_MODE)");

    const LinkStats before = link.stats();
    const uint8_t shortTlm[10] = {};
    br.send(hostproto::MSG_TELEMETRY, shortTlm, sizeof(shortTlm));
    br.send(hostproto::MSG_STATUS, shortTlm, sizeof(shortTlm));
    check(waitFor([&] { return link.stats().malformedRx == before.malformedRx + 2; }, 500) &&
          log.has("MSG_TELEMETRY length 10"),
          "wrong-length telemetry and status are counted malformed, telemetry logged");

    uint8_t frame[hostproto::MAX_FRAME];
    const Telemetry good = FakeBridge::telemetry(999999);
    size_t n = hostproto::buildFrame(hostproto::MSG_TELEMETRY, reinterpret_cast<const uint8_t*>(&good),
                                     sizeof(good), frame, sizeof(frame));
    frame[10] ^= 0x55;   // corrupt a payload byte
    const uint64_t crcBefore = link.stats().crcErrors;
    br.sendRaw(frame, n);
    check(waitFor([&] { return link.stats().crcErrors == crcBefore + 1; }, 500),
          "a corrupted frame is a CRC error and is not delivered");

    // A frame split by a scheduling gap under RX_TIMEOUT_MS still decodes;
    // one stalled past it is dropped, and the next frame decodes normally.
    // Judged by content (masterMillis), not counts: stop the stream and let the
    // last streamed frame land first.
    br.streaming = false;
    uint64_t settled = link.stats().telemetryRx;
    for (int i = 0; i < 50; ++i) {
        sleepMs(60);
        const uint64_t now = link.stats().telemetryRx;
        if (now == settled) break;
        settled = now;
    }
    auto frameOf = [&](uint32_t millis, uint8_t* out) {
        const Telemetry tt = FakeBridge::telemetry(millis);
        return hostproto::buildFrame(hostproto::MSG_TELEMETRY, reinterpret_cast<const uint8_t*>(&tt),
                                     sizeof(tt), out, hostproto::MAX_FRAME);
    };
    n = frameOf(999999, frame);
    br.sendSplit(frame, n, 20, 30);
    check(waitFor([&] { return link.telemetry().masterMillis == 999999; }, 1000),
          "a frame split by a 30 ms gap decodes");
    n = frameOf(888888, frame);
    br.sendSplit(frame, n, 20, hostproto::RX_TIMEOUT_MS + 150);
    sleepMs(200);
    check(link.telemetry().masterMillis == 999999, "a frame stalled past RX_TIMEOUT_MS is dropped");
    n = frameOf(777777, frame);
    br.sendRaw(frame, n);
    check(waitFor([&] { return link.telemetry().masterMillis == 777777; }, 1000),
          "... and the next whole frame decodes");

    const LinkStats s = link.stats();
    check(s.framesRx > 0 && s.bytesRx > 0 && s.txFrames >= 2 && s.txErrors == 0 && s.reconnects == 0,
          "stats: frames and bytes in, commands out, no TX errors, no reconnects");

    const auto t0 = std::chrono::steady_clock::now();
    link.stop();
    const long stopMs = static_cast<long>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count());
    check(!link.isRunning() && stopMs < 300, "stop() joins promptly (" + std::to_string(stopMs) + " ms)");
    check(link.isOpen(), "stop() leaves the port open");
    link.close();
    check(!link.isOpen() && link.devicePath().empty(), "close() closes the port");
}

static void testCommands() {
    std::printf("\n--- commands ---\n");
    FakeBridge br;
    br.plug();
    LogSink log;
    CommLink link;
    link.open(fastConfig(br.slave()), log.callback());
    link.start();
    waitFor([&] { return link.isConnected(); }, 1000);
    br.clearCommands();

    check(!link.setDecimation(0), "setDecimation(0) refused");
    check(link.setDecimation(3) && waitFor([&] { return br.lastPayload(hostproto::CMD_SET_DECIM) ==
                                                        std::vector<uint8_t>{3}; }, 500),
          "setDecimation(3) sends CMD_SET_DECIM 3");
    check(!link.setCanMode(0) && !link.setCanMode(4), "setCanMode(0) and (4) refused locally");
    check(link.setCanMode(2) && waitFor([&] { return br.lastPayload(hostproto::CMD_SET_CAN_MODE) ==
                                                     std::vector<uint8_t>{2}; }, 500),
          "setCanMode(2) sends CMD_SET_CAN_MODE 2");
    check(!link.setImuMode(0) && !link.setImuMode(3), "setImuMode(0) and (3) refused locally");
    check(link.setImuMode(hostproto::IMU_MODE_RAW) &&
              waitFor([&] { return br.lastPayload(hostproto::CMD_SET_IMU_MODE) ==
                                   std::vector<uint8_t>{hostproto::IMU_MODE_RAW}; }, 500),
          "setImuMode(RAW) sends CMD_SET_IMU_MODE 2");
    check(link.requestOnce() && link.requestStatus() && link.stopStream() && link.startStream() && link.ping(),
          "requestOnce/requestStatus/stopStream/startStream/ping all written");
    check(waitFor([&] { return br.count(hostproto::CMD_GET_ONCE) == 1 && br.count(hostproto::CMD_GET_STATUS) == 1 &&
                               br.count(hostproto::CMD_STOP_STREAM) == 1 &&
                               br.count(hostproto::CMD_START_STREAM) == 1; }, 500),
          "... and each arrives once");

    // A bridge reboot: an unsolicited MSG_HELLO.  The session is re-established
    // with the live decimation, and streaming resumes — nothing else is replayed.
    br.clearCommands();
    br.streaming = false;
    br.sendHello(hostproto::VERSION);
    check(waitFor([&] { return br.lastPayload(hostproto::CMD_SET_DECIM) == std::vector<uint8_t>{3} &&
                               br.count(hostproto::CMD_START_STREAM) == 1; }, 500),
          "after a bridge reboot (MSG_HELLO): decimation 3 re-applied, stream restarted");
    sleepMs(100);
    check(br.count(hostproto::CMD_SET_CAN_MODE) == 0 && br.count(hostproto::CMD_SET_IMU_MODE) == 0,
          "... and CAN / IMU modes are NOT replayed (by design)");

    br.clearCommands();
    sleepMs(1000);
    const int pings = br.count(hostproto::CMD_PING);
    check(pings >= 3 && pings <= 8, "keepalive: " + std::to_string(pings) + " CMD_PING in 1 s at 200 ms");
    link.close();
}

static void testWatchdogAndHotplug() {
    std::printf("\n--- silence watchdog, hot unplug and replug ---\n");
    TempDir dir;
    FakeBridge br;
    br.plug();
    dir.link("bridge", br.slave());   // the udev-style stable name
    const std::string node = (dir.path / "bridge").string();

    LogSink log;
    CommLink link;
    std::atomic<int> ups{0}, downs{0}, hellos{0};
    link.setConnectionCallback([&](bool c) { c ? ++ups : ++downs; });
    link.setHelloCallback([&](const Hello&) { ++hellos; });
    link.open(fastConfig(node), log.callback());
    link.start();
    check(waitFor([&] { return link.isConnected() && link.hasTelemetry(); }, 1000), "connected through a symlink");

    // isConnected() flips before the callback runs: wait on the callback count.
    br.mute = true;   // the C3 hangs with the cable still in: no frames, not even PONG
    check(waitFor([&] { return downs == 1; }, 1200) && !link.isConnected(),
          "silence past connectionTimeoutMs: disconnected, callback(false)");
    check(link.isOpen(), "... while the port stays open (the node did not go away)");
    br.mute = false;
    check(waitFor([&] { return ups == 2; }, 1000) && link.isConnected(), "frames again: reconnected, callback(true)");

    br.unplug();
    check(waitFor([&] { return !link.isOpen(); }, 1000), "unplug: hangup seen, port closed");
    // The port closes just before the link is marked down: wait for both.
    check(waitFor([&] { return !link.isConnected() && downs == 2; }, 500) &&
              (log.has("bridge detached") || log.has("device hangup")),
          "... disconnected, and logged");
    sleepMs(300);
    check(link.isRunning(), "the RX thread keeps retrying with the device gone");

    br.plug();                         // re-enumerates under a new node
    dir.link("bridge", br.slave());
    check(waitFor([&] { return link.isConnected() && hellos >= 2; }, 1500),
          "replug: reopened, CMD_HELLO answered again");
    check(link.stats().reconnects == 1 && link.devicePath() == node, "stats().reconnects == 1");
    const uint32_t sent = br.telemetrySent;
    check(waitFor([&] { return br.telemetrySent > sent + 2 && !link.isStale(); }, 1000),
          "streaming resumed without the application doing anything");
    link.close();
}

static void testForeignDevices() {
    std::printf("\n--- mismatched and silent devices ---\n");
    {
        FakeBridge br;
        br.plug();
        br.helloProto = hostproto::VERSION + 1;   // firmware built against another contract
        LogSink log;
        CommLink link;
        std::atomic<int> hellos{0};
        link.setHelloCallback([&](const Hello& h) { if (h.protoVersion == hostproto::VERSION + 1) ++hellos; });
        link.open(fastConfig(br.slave()), log.callback());
        link.start();
        check(waitFor([&] { return hellos >= 1; }, 1000) && log.has("PROTOCOL MISMATCH"),
              "a mismatched MSG_HELLO reaches the hello callback, and is logged as an ERROR");
        check(waitFor([&] { return log.has("did not identify itself"); }, 1500),
              "... fails the handshake and is released");
        check(br.count(hostproto::CMD_START_STREAM) == 0, "... and is never asked to stream");
        link.close();
    }
    {
        FakeBridge br;
        br.plug();
        br.answerHello = false;                    // an ACM device that is not our bridge
        LogSink log;
        CommLink link;
        link.open(fastConfig(br.slave()), log.callback());
        link.start();
        check(waitFor([&] { return log.count("did not identify itself") >= 2; }, 2500),
              "a device that never says HELLO is released and retried, not latched onto");
        check(!link.hasTelemetry(), "... and nothing from it counts as telemetry");
        link.close();
    }
    {
        TempDir dir;
        const std::string node = (dir.path / "bridge").string();
        LogSink log;
        CommLink link;
        check(!link.open(fastConfig(node), log.callback()), "open() on a missing device returns false");
        check(link.start(), "... and start() still runs, retrying in the background");
        sleepMs(250);
        FakeBridge br;
        br.plug();
        dir.link("bridge", br.slave());
        check(waitFor([&] { return link.isConnected(); }, 1500), "the device appears later: connected");
        link.close();
    }
}

static void testDiscovery() {
    std::printf("\n--- discovery over a fake /dev tree ---\n");
    TempDir dev;
    FakeBridge silent, bridge;
    silent.plug();
    silent.answerHello = false;   // e.g. a second Espressif board running something else
    bridge.plug();
    // Sorted by name, the silent one comes first: discovery must move past it.
    dev.link("serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_AA-if00", silent.slave());
    dev.link("serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_BB-if00", bridge.slave());
    dev.link("serial/by-id/usb-Arduino_MKRZero_CC-if00", "/dev/null");
    { std::FILE* f = std::fopen((dev.path / "ttyACM7").c_str(), "w"); if (f) std::fclose(f); }

    const auto byId = CommLink::enumerate("USB_JTAG", false, dev.path.string());
    check(byId.size() == 2 && byId[0] == silent.slave() && byId[1] == bridge.slave(),
          "enumerate(): by-id matches only, resolved to their nodes, in name order");
    const auto all = CommLink::enumerate("USB_JTAG", true, dev.path.string());
    check(all.size() == 3 && all[2] == (dev.path / "ttyACM7").string(),
          "enumerate(fallback): ttyACM* appended after them");
    check(CommLink::enumerate("USB_JTAG", true, (dev.path / "nowhere").string()).empty(),
          "enumerate() on a missing tree: empty, no exception");

    LogSink log;
    CommLink link;
    CommLinkConfig cfg = fastConfig("");   // auto-discovery
    cfg.devRoot = dev.path.string();
    link.open(cfg, log.callback());
    link.start();
    check(waitFor([&] { return link.isConnected() && link.devicePath() == bridge.slave(); }, 3000),
          "auto-discovery skips the silent candidate and connects to the bridge");
    check(log.has(silent.slave() + " did not identify itself"), "... after logging the one it skipped");
    link.close();
}

static void testLogCallbackMayCallBack() {
    std::printf("\n--- a log callback that calls back into the link ---\n");
    FakeBridge br;
    br.plug();
    LogSink log;
    CommLink link;
    std::atomic<int> calls{0};
    // What an application might do: annotate a port line with the link's state.
    // The callback used to run with the port mutex held, so this deadlocked.
    log.hook = [&](LogLevel, const std::string& msg) {
        if (msg.find("bridge port") != std::string::npos) {
            (void)link.isOpen();
            (void)link.devicePath();
            ++calls;
        }
    };
    link.open(fastConfig(br.slave()), log.callback());   // logs "bridge port open"
    link.start();
    waitFor([&] { return link.isConnected(); }, 1000);
    br.unplug();                                          // logs "bridge port closed"
    check(waitFor([&] { return calls >= 2; }, 1500), "port open and port closed logged, and the callback's "
                                                      "calls into the link returned (no deadlock)");
    link.close();
}

static void testProtocolHelpers() {
    std::printf("\n--- HostProtocol.h helpers ---\n");
    std::string unnamed;
    for (int t = 0; t < 256; ++t)
        if (hostproto::isCommand(static_cast<uint8_t>(t)) &&
            std::strcmp(hostproto::typeName(static_cast<uint8_t>(t)), "UNKNOWN") == 0)
        {
            char hex[8];
            std::snprintf(hex, sizeof(hex), " 0x%02X", t);
            unnamed += hex;
        }
    check(unnamed.empty(), "typeName() names every command isCommand() accepts" +
                           (unnamed.empty() ? std::string() : " — missing:" + unnamed));
}

static void testUart() {
    std::printf("\n--- libuart on a pty ---\n");
    const int master = ::posix_openpt(O_RDWR | O_NOCTTY);
    ::grantpt(master);
    ::unlockpt(master);
    const std::string slave = ::ptsname(master);
    dashcam::uart::Uart u;
    LogSink log;
    check(u.open(slave, {}, log.callback()), "open() a pty");
    check((::fcntl(u.fd(), F_GETFD) & FD_CLOEXEC) != 0, "the fd is close-on-exec");
    check((::fcntl(u.fd(), F_GETFL) & O_NONBLOCK) == 0, "... and blocking (timeouts are poll()-based)");

    uint8_t buf[64];
    const auto t0 = std::chrono::steady_clock::now();
    check(u.read(buf, sizeof(buf), 0) == 0, "read(timeout 0) with nothing buffered returns 0");
    check(u.read(buf, sizeof(buf), 50) == 0, "read(timeout 50) with nothing buffered returns 0");
    const long waited = static_cast<long>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count());
    check(waited >= 45 && waited < 200, "... after its timeout (" + std::to_string(waited) + " ms)");
    (void)!::write(master, "abc", 3);
    sleepMs(20);
    check(u.read(buf, sizeof(buf), 0) == 3 && std::memcmp(buf, "abc", 3) == 0, "read(0) returns what is buffered");
    (void)!::write(master, "$GPRMC,1\r\nnext", 15);
    std::string line;
    check(u.readLine(line, 200) && line == "$GPRMC,1", "readLine() strips \\r\\n");

    // A peer that stops draining: writeTimeout() must give up at its deadline
    // (write() would block forever), and leave the fd blocking again.
    std::vector<uint8_t> big(256 * 1024, 0x55);
    const auto t1 = std::chrono::steady_clock::now();
    const bool wrote = u.writeTimeout(big.data(), big.size(), 200);
    const long took = static_cast<long>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t1).count());
    check(!wrote && took >= 190 && took < 600,
          "writeTimeout() into a stalled peer fails at its deadline (" + std::to_string(took) + " ms)");
    check((::fcntl(u.fd(), F_GETFL) & O_NONBLOCK) == 0, "... and restores blocking mode");
    u.close();
    check(!u.isOpen() && u.read(buf, 1, 0) == -1, "closed: read() returns -1");
    ::close(master);
}

int main() {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    // A hang is a failure too (a deadlock in the link would otherwise stall CI).
    ::signal(SIGALRM, [](int) {
        const char msg[] = "\nFAIL  test timed out (a hang in CommLink?)\n";
        (void)!::write(STDOUT_FILENO, msg, sizeof(msg) - 1);
        ::_exit(2);
    });
    ::alarm(60);   // the whole run takes about 6 s
    std::printf("commlink_sim_test — CommLink against a simulated ESP32-C3 bridge (pty)\n");
    testHandshakeAndStream();
    testCommands();
    testWatchdogAndHotplug();
    testForeignDevices();
    testDiscovery();
    testLogCallbackMayCallBack();
    testProtocolHelpers();
    testUart();
    std::printf("\nRESULT: %s (%d failure%s)\n", g_fails ? "FAIL" : "PASS", g_fails,
                g_fails == 1 ? "" : "s");
    return g_fails ? 1 : 0;
}
