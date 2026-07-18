// GPIO / UART / I2C / SPI integration test
// Runs on the Jetson Orin Nano 40-pin header without requiring specific external hardware.
// Tests that succeed unconditionally: I2C bus scan, SPI open.
// Tests that require wiring:           GPIO read/write, UART loopback.

#include "libgpio.h"
#include "libgpio_dashcam.h"
#include "libi2c.h"
#include "liblog.h"
#include "libspi.h"
#include "libuart.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <thread>

namespace fs = std::filesystem;
using LvL = dashcam::log::LogLevel;

static std::atomic<bool> g_stop{false};
static void onSignal(int) { g_stop.store(true); }

static void section(dashcam::log::LogCallback& log, int n, int total, const char* name) {
    log(LvL::INFO, std::string(50, '-'));
    std::ostringstream o;
    o << "[" << n << "/" << total << "] " << name;
    log(LvL::INFO, o.str());
    log(LvL::INFO, std::string(50, '-'));
}

int main(int argc, char* argv[]) {
    (void)argc; (void)argv;
    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    fs::create_directories("./logs");
    dashcam::log::init();  // build-local logs: <exe_dir>/logs
    auto log = dashcam::log::getCallback();

    log(LvL::INFO, "=== GPIO / UART / I2C / SPI test ===");
    constexpr int N = 4;

    // ── [1/4] libgpio ─────────────────────────────────────────────────────────
    section(log, 1, N, "libgpio");

    // Output pin: header pin 29 (GPIO05 = PAA.00)
    {
        dashcam::gpio::GpioPin out;
        bool ok = out.openByName(dashcam::gpio::pins::PIN29_GPIO05,
                                 dashcam::gpio::PinDirection::OUTPUT,
                                 dashcam::gpio::LogicLevel::LOW, log);
        log(ok ? LvL::INFO : LvL::WARN,
            std::string("  GpioPin OUTPUT (pin 29 / PAA.00)  ") + (ok ? "OK" : "SKIP — line in use?"));
        if (ok) {
            for (int i = 0; i < 4 && !g_stop.load(); ++i) {
                out.toggle();
                dashcam::gpio::LogicLevel lvl;
                out.read(lvl);
                std::ostringstream o;
                o << "    toggle  →  " << (lvl == dashcam::gpio::LogicLevel::HIGH ? "HIGH" : "LOW");
                log(LvL::INFO, o.str());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            out.write(dashcam::gpio::LogicLevel::LOW);
        }
    }

    // Input pin with edge watcher: header pin 31 (GPIO06 = PAA.03)
    {
        dashcam::gpio::GpioWatcher watcher;
        bool ok = watcher.openByName(dashcam::gpio::pins::PIN31_GPIO06,
                                     dashcam::gpio::EdgeTrigger::BOTH, log);
        log(ok ? LvL::INFO : LvL::WARN,
            std::string("  GpioWatcher INPUT (pin 31 / PAA.03)  ") + (ok ? "OK" : "SKIP — line in use?"));
        if (ok) {
            watcher.setCallback([&](dashcam::gpio::LogicLevel lvl, int64_t tsNs) {
                std::ostringstream o;
                o << "    edge  " << (lvl == dashcam::gpio::LogicLevel::HIGH ? "RISING " : "FALLING")
                  << "  ts=" << tsNs / 1'000'000 << " ms";
                log(LvL::INFO, o.str());
            });
            watcher.start();
            log(LvL::INFO, "    watching for 2s (apply signal to pin 31 to generate edges)");
            std::this_thread::sleep_for(std::chrono::seconds(2));
            watcher.stop();
        }
    }

    // ── [2/4] libuart ─────────────────────────────────────────────────────────
    section(log, 2, N, "libuart");

    {
        // Try ttyTHS1 first (header pins 8/10), fall back to ttyS0.
        const char* uartDev = "/dev/ttyTHS1";
        dashcam::uart::UartConfig ucfg;
        ucfg.baudRate = 115200;

        dashcam::uart::Uart uart;
        bool ok = uart.open(uartDev, ucfg, log);
        log(ok ? LvL::INFO : LvL::WARN,
            std::string("  Uart::open(") + uartDev + ", 115200)  " + (ok ? "OK" : "FAIL"));

        if (ok) {
            const std::string msg = "DASHCAM_UART_TEST\r\n";
            uart.write(msg);
            log(LvL::INFO, "  TX: \"DASHCAM_UART_TEST\"");

            // If TX is looped back to RX (short pin 8 to pin 10), this will read the echo.
            std::string line;
            bool got = uart.readLine(line, 200);
            if (got)
                log(LvL::INFO, "  RX (loopback): \"" + line + "\"");
            else
                log(LvL::INFO, "  RX: timeout (no loopback wired — expected)");
        }
    }

    // ── [3/4] libi2c ──────────────────────────────────────────────────────────
    section(log, 3, N, "libi2c");

    {
        dashcam::i2c::I2cBus i2c;
        bool ok = i2c.open("/dev/i2c-1", log);
        log(ok ? LvL::INFO : LvL::ERROR,
            std::string("  I2cBus::open(/dev/i2c-1)  ") + (ok ? "OK" : "FAIL"));

        if (ok) {
            std::vector<uint8_t> found;
            i2c.scanBus(found);
            {
                std::ostringstream o;
                o << "  Bus scan: " << found.size() << " device(s) found";
                log(LvL::INFO, o.str());
            }
            for (uint8_t addr : found) {
                std::ostringstream o;
                o << "    0x" << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int>(addr);
                log(LvL::INFO, o.str());
            }
        }
    }

    // ── [4/4] libspi ──────────────────────────────────────────────────────────
    section(log, 4, N, "libspi");

    {
        dashcam::spi::SpiConfig scfg;
        scfg.mode    = 0;
        scfg.speedHz = 1000000;  // 1 MHz

        dashcam::spi::SpiBus spi;
        bool ok = spi.open("/dev/spidev0.0", scfg, log);
        log(ok ? LvL::INFO : LvL::WARN,
            std::string("  SpiBus::open(/dev/spidev0.0)  ") + (ok ? "OK" : "FAIL"));

        if (ok) {
            // Transfer 4 bytes.  With MISO connected to MOSI (loopback), rxBuf == txBuf.
            uint8_t tx[4] = {0xDE, 0xAD, 0xBE, 0xEF};
            uint8_t rx[4] = {};
            bool txOk = spi.transfer(tx, rx, 4);
            {
                std::ostringstream o;
                o << "  transfer [DE AD BE EF]  " << (txOk ? "OK" : "FAIL");
                if (txOk)
                    o << "  RX: "
                      << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int>(rx[0]) << " "
                      << static_cast<int>(rx[1]) << " "
                      << static_cast<int>(rx[2]) << " "
                      << static_cast<int>(rx[3]);
                log(txOk ? LvL::INFO : LvL::ERROR, o.str());
            }
            if (txOk && rx[0] == tx[0] && rx[1] == tx[1] &&
                rx[2] == tx[2] && rx[3] == tx[3])
                log(LvL::INFO, "  loopback match  OK");
        }
    }

    log(LvL::INFO, "=== Done ===");
    dashcam::log::shutdown();
    return 0;
}
