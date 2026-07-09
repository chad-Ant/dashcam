/**
 * @file libgpio.h
 * @brief Digital GPIO input/output and edge-triggered interrupt via libgpiod.
 *
 * Uses the libgpiod 1.6.x API (the version shipped with Ubuntu 22.04 / JetPack 6.2).
 * Accesses /dev/gpiochipN character devices — no sysfs, no root required for
 * lines that are not claimed by a kernel driver.
 *
 * Typical usage:
 * @code
 *   #include "libgpio_dashcam.h"   // for dashcam::gpio::pins constants
 *
 *   // Digital output
 *   dashcam::gpio::GpioPin led;
 *   led.openByName(dashcam::gpio::pins::PIN29_GPIO05,
 *                  dashcam::gpio::PinDirection::OUTPUT, dashcam::gpio::LogicLevel::LOW, log);
 *   led.write(dashcam::gpio::LogicLevel::HIGH);
 *
 *   // Edge-triggered input
 *   dashcam::gpio::GpioWatcher btn;
 *   btn.openByName(dashcam::gpio::pins::PIN31_GPIO06,
 *                  dashcam::gpio::EdgeTrigger::BOTH, log);
 *   btn.setCallback([](dashcam::gpio::LogicLevel lvl, int64_t tsNs) {
 *       // called on the watcher thread — keep it short
 *   });
 *   btn.start();
 * @endcode
 */

#ifndef LIBGPIO_H
#define LIBGPIO_H

#include "ibus.h"
#include "liblog.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

// Forward-declare gpiod types so callers do not need gpiod.h in scope.
struct gpiod_chip;
struct gpiod_line;

namespace dashcam::gpio {

// ─── enumerations ─────────────────────────────────────────────────────────────

enum class PinDirection { INPUT, OUTPUT };
enum class LogicLevel   { LOW = 0, HIGH = 1 };
enum class EdgeTrigger  { RISING, FALLING, BOTH };

// ─── callbacks ────────────────────────────────────────────────────────────────

/// Called on the watcher thread; timestampNs is nanoseconds since boot.
using EdgeCallback = std::function<void(LogicLevel level, int64_t timestampNs)>;

// ─── GpioPin ──────────────────────────────────────────────────────────────────

/**
 * @brief A single digital I/O line managed via libgpiod.
 *
 * Open by the stable gpiod line name (e.g. "PBB.00") — searches gpiochip0
 * then gpiochip1.  Alternatively, supply the chip path and numeric offset.
 */
class GpioPin {
public:
    GpioPin();
    ~GpioPin();

    GpioPin(const GpioPin&)            = delete;
    GpioPin& operator=(const GpioPin&) = delete;

    /**
     * @brief Open by gpiochip device path and line offset.
     *
     * @param chipPath    e.g. "/dev/gpiochip0"
     * @param offset      Line offset within the chip (from gpioinfo).
     * @param dir         INPUT or OUTPUT.
     * @param initialVal  Initial value for OUTPUT lines; ignored for INPUT.
     */
    bool open(const std::string& chipPath, uint32_t offset,
              PinDirection dir,
              LogicLevel initialVal = LogicLevel::LOW,
              const dashcam::log::LogCallback& log = {});

    /**
     * @brief Open by gpiod line name (e.g. "PBB.00").
     *
     * Searches gpiochip0 then gpiochip1.  Equivalent to running
     * `gpioget $(gpiofind PBB.00)`.
     */
    bool openByName(const std::string& lineName,
                    PinDirection dir,
                    LogicLevel initialVal = LogicLevel::LOW,
                    const dashcam::log::LogCallback& log = {});

    bool         read (LogicLevel& level) const;
    bool         write(LogicLevel level);
    bool         toggle();   ///< OUTPUT only; reads current state and inverts.

    PinDirection direction() const { return m_dir; }
    void         close();
    bool         isOpen()    const;

private:
    bool configure(PinDirection dir, LogicLevel initialVal);

    struct gpiod_chip* m_chip = nullptr;
    struct gpiod_line* m_line = nullptr;
    PinDirection       m_dir  = PinDirection::INPUT;
    dashcam::log::LogCallback m_log;
};

// ─── GpioWatcher ──────────────────────────────────────────────────────────────

/**
 * @brief Interrupt-driven edge detection via a background select() thread.
 *
 * Uses gpiod_line_event_get_fd() + select() with a wake pipe so stop()
 * returns promptly without waiting for the next edge.
 */
class GpioWatcher {
public:
    GpioWatcher();
    ~GpioWatcher();

    GpioWatcher(const GpioWatcher&)            = delete;
    GpioWatcher& operator=(const GpioWatcher&) = delete;

    bool open(const std::string& chipPath, uint32_t offset,
              EdgeTrigger edge,
              const dashcam::log::LogCallback& log = {});

    bool openByName(const std::string& lineName,
                    EdgeTrigger edge,
                    const dashcam::log::LogCallback& log = {});

    void setCallback(EdgeCallback cb);

    bool start();
    void stop();
    void close();

    bool isRunning() const { return m_running.load(); }

private:
    void watchLoop();
    bool requestEdge(EdgeTrigger edge);

    struct gpiod_chip* m_chip    = nullptr;
    struct gpiod_line* m_line    = nullptr;
    EdgeTrigger        m_edge    = EdgeTrigger::BOTH;
    std::atomic<bool>  m_running{false};
    std::thread        m_thread;
    int                m_pipe[2] = {-1, -1};
    EdgeCallback       m_cb;
    std::mutex         m_cbMtx;
    dashcam::log::LogCallback m_log;
};

// ─── GpioPeripheral ───────────────────────────────────────────────────────────

/**
 * @brief Binds a GpioPin (CS / RESET / ENABLE) to a serial bus (IBus).
 *
 * Lets GPIO control a peripheral by driving its control line while delegating
 * data transfers to whichever bus type the peripheral is wired to.
 *
 * @code
 *   dashcam::spi::SpiBus spi;
 *   spi.open("/dev/spidev0.0", {});
 *
 *   dashcam::gpio::GpioPin cs;
 *   cs.openByName("PBB.00", dashcam::gpio::PinDirection::OUTPUT,
 *                 dashcam::gpio::LogicLevel::HIGH);   // CS idle-high
 *
 *   dashcam::gpio::GpioPeripheral dev(cs, spi);
 *   dev.assertCS();                                   // drive CS low
 *   uint8_t cmd = 0x9F;
 *   dev.send(&cmd, 1);                                // write via SPI
 *   uint8_t id[3];
 *   dev.receive(id, 3);                               // read reply
 *   dev.releaseCS();                                  // drive CS high
 * @endcode
 *
 * For I2C, there is no CS line; use assertCS()/releaseCS() to drive an
 * optional ENABLE pin, or simply call send()/receive() after setDevice().
 */
class GpioPeripheral {
public:
    /**
     * @param controlPin  GPIO output pin used for CS / RESET / ENABLE.
     *                    Must already be open and configured as OUTPUT.
     * @param bus         Open bus the peripheral communicates over.
     *                    Lifetime must exceed this object.
     */
    GpioPeripheral(GpioPin& controlPin, dashcam::bus::IBus& bus);

    GpioPeripheral(const GpioPeripheral&)            = delete;
    GpioPeripheral& operator=(const GpioPeripheral&) = delete;

    /**
     * @brief Pulse the control pin LOW for @p pulseMs milliseconds then HIGH.
     *
     * Typical use: hardware RESET — assert low, wait, de-assert.
     */
    void reset(uint32_t pulseMs = 10);

    /** @brief Drive the control pin LOW  (assert CS / enable active-low line). */
    void assertCS();

    /** @brief Drive the control pin HIGH (release CS / disable active-low line). */
    void releaseCS();

    /** @brief Drive an active-high ENABLE line; equivalent to write HIGH/LOW. */
    void setEnabled(bool enabled);

    // ── bus delegation ────────────────────────────────────────────────────────

    /** @brief Set I2C device address before send()/receive().  No-op on UART/SPI. */
    void setDevice(uint8_t addr) { m_bus.setDevice(addr); }

    bool send   (const uint8_t* buf, size_t len)              { return m_bus.send(buf, len); }
    int  receive(uint8_t* buf, size_t len, int timeoutMs = 1000) { return m_bus.receive(buf, len, timeoutMs); }
    void flush  ()                                             { m_bus.flush(); }

    GpioPin&            pin() { return m_pin; }
    dashcam::bus::IBus& bus() { return m_bus; }

private:
    GpioPin&            m_pin;
    dashcam::bus::IBus& m_bus;
};

} // namespace dashcam::gpio

#endif // LIBGPIO_H
