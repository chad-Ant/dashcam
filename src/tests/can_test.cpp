// CAN bus test: configureInterface + send + receive loopback
// Needs can0 to exist and CAP_NET_ADMIN (sudo) for configureInterface.
// Ctrl-C exits cleanly.

#include "libcan.h"
#include "liblog.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <thread>

namespace fs = std::filesystem;
using LvL = dashcam::log::LogLevel;

static std::atomic<bool> g_stop{false};
static void onSignal(int) { g_stop.store(true); }

static std::string hexFrame(const dashcam::can::CanFrame& f) {
    std::ostringstream oss;
    oss << (f.extended ? "EXT" : "STD")
        << " 0x" << std::hex << std::uppercase << std::setw(f.extended ? 8 : 3)
        << std::setfill('0') << f.id
        << (f.fdFrame ? " [FD]" : "")
        << "  [" << std::dec << static_cast<int>(f.len) << "]";
    for (uint8_t i = 0; i < f.len; ++i)
        oss << " " << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(f.data[i]);
    return oss.str();
}

int main(int argc, char* argv[]) {
    (void)argc; (void)argv;
    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    fs::create_directories("./logs");
    dashcam::log::init();  // build-local logs: <exe_dir>/logs
    auto log = dashcam::log::getCallback();

    log(LvL::INFO, "=== CAN bus test ===");

    const std::string iface = "can0";

    // ── [1] Configure interface ───────────────────────────────────────────────
    log(LvL::INFO, "--------------------------------------------");
    log(LvL::INFO, "[1] configureInterface");
    log(LvL::INFO, "--------------------------------------------");

    // Use loopback so the test is self-contained (no external device required).
    dashcam::can::CanBusConfig cfg;
    cfg.bitrate  = 500000;
    cfg.loopback = true;

    bool cfgOk = dashcam::can::configureInterface(iface, cfg, log);
    {
        std::ostringstream o;
        o << "  configureInterface(" << iface << ", 500kbps, loopback)  "
          << (cfgOk ? "OK" : "FAIL — is the binary running as root?");
        log(cfgOk ? LvL::INFO : LvL::WARN, o.str());
    }
    if (!cfgOk)
        log(LvL::WARN, "  Continuing without reconfiguring — using current interface state.");

    // ── [2] Open ──────────────────────────────────────────────────────────────
    log(LvL::INFO, "--------------------------------------------");
    log(LvL::INFO, "[2] CanBus::open");
    log(LvL::INFO, "--------------------------------------------");

    dashcam::can::CanBus bus;
    if (!bus.open(iface, /*fdEnabled=*/false, log)) {
        log(LvL::ERROR, "  open() failed — is can0 up?  Try: ip link set can0 up type can bitrate 500000");
        dashcam::log::shutdown();
        return 1;
    }
    log(LvL::INFO, "  open()  OK");

    // ── [3] Start receive thread ───────────────────────────────────────────────
    log(LvL::INFO, "--------------------------------------------");
    log(LvL::INFO, "[3] Receive thread");
    log(LvL::INFO, "--------------------------------------------");

    std::atomic<int> rxCount{0};
    bus.setReceiveCallback([&](const dashcam::can::CanFrame& f) {
        ++rxCount;
        std::ostringstream o;
        o << "  RX #" << rxCount.load() << "  " << hexFrame(f);
        log(LvL::INFO, o.str());
    });
    bus.setErrorCallback([&](const std::string& err) {
        log(LvL::WARN, "  CAN error: " + err);
    });
    bus.start();
    log(LvL::INFO, "  start()  OK");

    // ── [4] Send frames ───────────────────────────────────────────────────────
    log(LvL::INFO, "--------------------------------------------");
    log(LvL::INFO, "[4] Send frames");
    log(LvL::INFO, "--------------------------------------------");

    // Standard 11-bit frame
    {
        dashcam::can::CanFrame tx;
        tx.id      = 0x123;
        tx.len     = 4;
        tx.data[0] = 0xDE; tx.data[1] = 0xAD;
        tx.data[2] = 0xBE; tx.data[3] = 0xEF;
        bool ok = bus.send(tx);
        log(ok ? LvL::INFO : LvL::ERROR,
            std::string("  TX STD 0x123 [4] DE AD BE EF  ") + (ok ? "OK" : "FAIL"));
    }

    // Extended 29-bit frame
    {
        dashcam::can::CanFrame tx;
        tx.id       = 0x18DA00F1;
        tx.extended = true;
        tx.len      = 8;
        for (uint8_t i = 0; i < 8; ++i) tx.data[i] = i;
        bool ok = bus.send(tx);
        log(ok ? LvL::INFO : LvL::ERROR,
            std::string("  TX EXT 0x18DA00F1 [8] 00..07  ") + (ok ? "OK" : "FAIL"));
    }

    // ── [5] Wait and report ───────────────────────────────────────────────────
    log(LvL::INFO, "--------------------------------------------");
    log(LvL::INFO, "[5] Listening 5s (Ctrl-C to stop early)");
    log(LvL::INFO, "--------------------------------------------");

    constexpr int LISTEN_SECS = 5;
    auto tEnd = std::chrono::steady_clock::now() + std::chrono::seconds(LISTEN_SECS);
    while (!g_stop.load() && std::chrono::steady_clock::now() < tEnd)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    {
        std::ostringstream o;
        o << "  Received " << rxCount.load() << " frame(s)";
        log(LvL::INFO, o.str());
    }

    // ── [6] Tear down ─────────────────────────────────────────────────────────
    bus.stop();
    bus.close();
    log(LvL::INFO, "  stop + close  OK");
    log(LvL::INFO, "=== Done ===");
    dashcam::log::shutdown();
    return 0;
}
