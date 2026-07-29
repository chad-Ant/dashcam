// ESP32-C3 bridge link test: discovery, connect, stream, hot-unplug recovery.
//
// Plug the C3's USB-C port into any USB port on the Jetson, then run.  With no
// arguments it auto-discovers the device and streams until Ctrl-C.
//
//   ./commlink_test                    auto-discover, stream forever
//   ./commlink_test /dev/ttyACM0       explicit device node
//   ./commlink_test /dev/ttyACM0 30    ... and exit after 30 s
//
// Unplug and replug the cable while it runs: the link must recover on its own,
// with a reconnect counted and a fresh MSG_HELLO in the log.

#include "libcommlink.h"
#include "liblog.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>

namespace fs = std::filesystem;
using LvL = dashcam::log::LogLevel;

static std::atomic<bool> g_stop{false};
static void onSignal(int) { g_stop.store(true); }

// Telemetry floats are NAN when the master had no reading — never print a bare 0.
static std::string num(float v, int precision = 1) {
    if (std::isnan(v)) return "--";
    std::ostringstream o;
    o << std::fixed << std::setprecision(precision) << v;
    return o.str();
}

static std::string describe(const dashcam::commlink::Telemetry& t) {
    std::ostringstream o;
    o << "t=" << t.masterMillis << "ms"
      << "  spd=" << num(t.speed) << "km/h"
      << "  acc=" << num(t.accel, 2) << "m/s2"
      << "  rpm=" << num(t.rpm, 0)
      << "  cool=" << num(t.coolantTemp, 0) << "C"
      << "  fuel=" << num(t.fuelLevel, 0) << "%"
      << "  lat=" << num(t.latitude, 6)
      << "  lon=" << num(t.longitude, 6)
      << "  sats=" << static_cast<int>(t.satellites)
      << "  fix=" << static_cast<int>(t.fixType)
      // IMU, sensor frame. |a| rather than the three axes: at a glance the
      // magnitude is what tells you the sensor is sane (~9.8 at rest) without
      // needing to know how the board is mounted, and it stays "--" when any
      // axis is NAN so a partially stale reading cannot look plausible.
      << "  |a|=" << num(std::sqrt(t.imuAccelX * t.imuAccelX +
                                   t.imuAccelY * t.imuAccelY +
                                   t.imuAccelZ * t.imuAccelZ), 2) << "m/s2"
      << "  gyroZ=" << num(t.imuGyroZ, 1) << "deg/s"
      << "  imuT=" << num(t.imuTempC, 0) << "C"
      << "  flags=0x" << std::hex << std::uppercase << static_cast<int>(t.flags) << std::dec;
    return o.str();
}

static std::string describe(const dashcam::commlink::BridgeStatus& s) {
    std::ostringstream o;
    o << "up=" << s.bridgeMillis << "ms"
      << "  tlmAge=" << (s.telemetryAgeMs == UINT32_MAX ? std::string("never")
                                                        : std::to_string(s.telemetryAgeMs) + "ms")
      << "  master=" << s.masterFrames << "f"
      << "  hostRx=" << s.hostFrames << "f"
      << "  txDrop=" << s.hostTxDropped
      << "  die=" << num(s.tempC) << "C"
      << "  heap=" << s.freeHeapKb << "KB"
      << "  decim=" << static_cast<int>(s.decimation)
      << "  flags=0x" << std::hex << std::uppercase << static_cast<int>(s.flags) << std::dec;
    if (s.flags & hostproto::BRIDGE_FLAG_PM_PRESENT)
        o << "  batt=" << num(s.batteryVolts, 2) << "V/" << num(s.batteryPercent, 0) << "%";
    return o.str();
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    fs::create_directories("./logs");
    dashcam::log::init();
    auto log = dashcam::log::getCallback();

    log(LvL::INFO, "=== ESP32-C3 bridge link test ===");

    const std::string device   = (argc > 1) ? argv[1] : "";
    const int         runSecs  = (argc > 2) ? std::atoi(argv[2]) : 0;

    // ── [1] Discovery ─────────────────────────────────────────────────────────
    log(LvL::INFO, "--------------------------------------------");
    log(LvL::INFO, "[1] enumerate");
    log(LvL::INFO, "--------------------------------------------");

    const auto found = dashcam::commlink::CommLink::enumerate();
    if (found.empty()) {
        log(LvL::WARN, "  no candidate device found — is the C3 plugged into a USB port?");
    } else {
        for (const auto& p : found) log(LvL::INFO, "  candidate: " + p);
    }

    // ── [2] Open ──────────────────────────────────────────────────────────────
    log(LvL::INFO, "--------------------------------------------");
    log(LvL::INFO, "[2] open + start");
    log(LvL::INFO, "--------------------------------------------");

    dashcam::commlink::CommLinkConfig cfg;
    cfg.device = device;            // empty = auto-discover
    cfg.autoStream = true;          // stream resumes by itself after a bridge reset
    // This is a bench test, so allow the bare-ttyACM fallback that the
    // application deliberately leaves off.  The handshake gate rejects anything
    // that does not answer CMD_HELLO, so a wrong guess costs 2 s, not the run.
    cfg.allowAcmFallback = true;

    // Declared before `bridge` on purpose: ~CommLink() joins the RX thread, so
    // anything its callbacks capture must still be alive at that point.
    // Reversing these two lines makes teardown a use-after-free.
    std::atomic<uint64_t> telemetryCount{0};
    std::atomic<uint64_t> helloCount{0};
    std::atomic<bool>     mismatchSeen{false};

    dashcam::commlink::CommLink bridge;

    bridge.setHelloCallback([&](const dashcam::commlink::Hello& h) {
        // Counted, not latched: every session must produce its own MSG_HELLO.
        // A single historical flag would let a reconnect that silently failed to
        // re-handshake still report success.
        helloCount.fetch_add(1);
        // The library logs a mismatch; the test needs its own signal to fail on.
        if (h.protoVersion   != hostproto::VERSION ||
            h.telemetryBytes != sizeof(dashcam::commlink::Telemetry) ||
            h.statusBytes    != sizeof(dashcam::commlink::BridgeStatus)) {
            mismatchSeen.store(true);
        }
    });

    bridge.setTelemetryCallback([&](const dashcam::commlink::Telemetry& t) {
        // Telemetry lands at ~10 Hz; log one per second so the output stays readable.
        if (telemetryCount.fetch_add(1) % 10 == 0) log(LvL::INFO, "  TLM  " + describe(t));
    });
    bridge.setStatusCallback([&](const dashcam::commlink::BridgeStatus& s) {
        log(LvL::INFO, "  STAT " + describe(s));
    });
    bridge.setConnectionCallback([&](bool connected) {
        log(connected ? LvL::INFO : LvL::WARN,
            connected ? "  >>> bridge CONNECTED" : "  <<< bridge DISCONNECTED");
    });

    // A false return only means "not plugged in yet" — the RX thread keeps
    // retrying, so starting anyway is the correct behaviour for a vehicle boot
    // where the Jetson may come up before the C3 enumerates.
    if (!bridge.open(cfg, log))
        log(LvL::WARN, "  port not open yet; the RX thread will keep retrying");

    if (!bridge.start()) {
        log(LvL::ERROR, "  start() failed");
        dashcam::log::shutdown();
        return 1;
    }
    if (bridge.isOpen()) log(LvL::INFO, "  device: " + bridge.devicePath());

    // ── [3] Stream ────────────────────────────────────────────────────────────
    log(LvL::INFO, "--------------------------------------------");
    log(LvL::INFO, runSecs > 0 ? "[3] streaming (" + std::to_string(runSecs) + " s; Ctrl-C to stop early)"
                               : "[3] streaming (Ctrl-C to stop)");
    log(LvL::INFO, "--------------------------------------------");
    log(LvL::INFO, "  try unplugging and replugging the USB-C cable — the link must recover");

    const auto t0 = std::chrono::steady_clock::now();
    while (!g_stop.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (runSecs > 0) {
            const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                                     std::chrono::steady_clock::now() - t0).count();
            if (elapsed >= runSecs) break;
        }
    }

    // ── [4] Teardown + verdict ────────────────────────────────────────────────
    log(LvL::INFO, "--------------------------------------------");
    log(LvL::INFO, "[4] results");
    log(LvL::INFO, "--------------------------------------------");

    // Capture live state BEFORE stop() — stop() clears the connected flag, so
    // reading it afterwards would always report a disconnected link.
    const bool connectedAtExit = bridge.isConnected();
    const bool staleAtExit     = bridge.isStale(1000);

    bridge.stop();
    const auto s = bridge.stats();

    std::ostringstream o;
    o << "  bytesRx=" << s.bytesRx << "  frames=" << s.framesRx
      << "  tlm=" << s.telemetryRx << "  status=" << s.statusRx << "  log=" << s.logRx
      << "\n  crcErrors=" << s.crcErrors << "  malformed=" << s.malformedRx
      << "  nacks=" << s.nacksRx
      << "\n  txFrames=" << s.txFrames << "  txErrors=" << s.txErrors
      << "  reconnects=" << s.reconnects;
    log(LvL::INFO, o.str());

    // A pass must mean the link is working *now*, not that a frame arrived once
    // an hour ago.  Each condition below catches a distinct real failure, so
    // they are reported individually rather than as one opaque verdict.
    struct Check { const char* what; bool ok; const char* hint; };
    const Check checks[] = {
        { "MSG_HELLO received (bridge identified itself)", helloCount.load() > 0,
          "bridge never answered CMD_HELLO — wrong device, or firmware without hostLink" },
        // One session at start, plus one per reconnect: each must have produced
        // its own MSG_HELLO, or streaming was never re-armed after that
        // reconnect and the link is running on inherited state.
        { "one MSG_HELLO per session (start + each reconnect)",
          helloCount.load() >= 1 + s.reconnects,
          "a reconnect did not re-handshake — the bridge kept the previous "
          "session's streaming state instead of resetting it" },
        { "MSG_STATUS received (1 Hz heartbeat)", s.statusRx > 0,
          "no bridge heartbeat — the C3 is not running the bridge loop" },
        { "MSG_TELEMETRY received", s.telemetryRx > 0,
          "no telemetry — is the MKR Zero powered and wired to GPIO20/21?" },
        { "link still connected at exit", connectedAtExit,
          "the link dropped and did not recover before the run ended" },
        { "telemetry fresh at exit (<1 s)", !staleAtExit,
          "frames stopped arriving — check the master link flag in the status line" },
        { "no protocol mismatch", !mismatchSeen.load(),
          "HostProtocol.h differs between the C3 and this build — reflash the C3" },
    };

    bool ok = true;
    for (const auto& c : checks) {
        log(c.ok ? LvL::INFO : LvL::ERROR,
            std::string(c.ok ? "  [ OK ] " : "  [FAIL] ") + c.what);
        if (!c.ok) { log(LvL::ERROR, std::string("         ") + c.hint); ok = false; }
    }

    // Hot-replug is only *verified* if a reconnect actually happened; the test
    // invites it but cannot force it, so report it rather than failing on it.
    if (s.reconnects > 0)
        log(LvL::INFO, "  [ OK ] hot-replug recovery exercised (" +
                       std::to_string(s.reconnects) + " reconnect(s))");
    else
        log(LvL::WARN, "  [SKIP] hot-replug not exercised — unplug/replug the cable "
                       "during the run to cover it");

    log(ok ? LvL::INFO : LvL::ERROR, ok ? "  PASS" : "  FAIL");

    bridge.close();
    dashcam::log::shutdown();
    return ok ? 0 : 1;
}
