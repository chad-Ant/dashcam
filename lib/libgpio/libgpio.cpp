#include "libgpio.h"

#include <gpiod.h>
#include <unistd.h>
#include <sys/select.h>

#include <chrono>
#include <thread>

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstring>

// ─── file-local helpers ───────────────────────────────────────────────────────

namespace {

static void doLog(const dashcam::log::LogCallback& cb, dashcam::log::LogLevel lvl,
                  const char* fmt, ...) {
    if (!cb) return;
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    cb(lvl, buf);
}

// Try to find a named line on the given chip; returns nullptr on failure.
static struct gpiod_line* findLineOnChip(const char* chipPath,
                                          const std::string& name) {
    struct gpiod_chip* chip = gpiod_chip_open(chipPath);
    if (!chip) return nullptr;
    struct gpiod_line* line = gpiod_chip_find_line(chip, name.c_str());
    if (!line) { gpiod_chip_close(chip); return nullptr; }
    return line;  // chip stays open — caller must close via gpiod_line_get_chip()
}

} // namespace

// ─── dashcam::gpio ────────────────────────────────────────────────────────────

namespace dashcam::gpio {

// ─── GpioPin ──────────────────────────────────────────────────────────────────

GpioPin::GpioPin()  = default;
GpioPin::~GpioPin() { close(); }

bool GpioPin::open(const std::string& chipPath, uint32_t offset,
                   PinDirection dir, LogicLevel initialVal,
                   const dashcam::log::LogCallback& log) {
    close();
    m_log  = log;

    m_chip = gpiod_chip_open(chipPath.c_str());
    if (!m_chip) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "GpioPin::open: cannot open '%s': %s",
              chipPath.c_str(), ::strerror(errno));
        return false;
    }

    m_line = gpiod_chip_get_line(m_chip, offset);
    if (!m_line) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "GpioPin::open: line %u not found on '%s'",
              offset, chipPath.c_str());
        gpiod_chip_close(m_chip); m_chip = nullptr;
        return false;
    }

    return configure(dir, initialVal);
}

bool GpioPin::openByName(const std::string& lineName, PinDirection dir,
                          LogicLevel initialVal,
                          const dashcam::log::LogCallback& log) {
    close();
    m_log = log;

    // Search gpiochip0 then gpiochip1.
    for (const char* chip_path : {"/dev/gpiochip0", "/dev/gpiochip1"}) {
        m_line = findLineOnChip(chip_path, lineName);
        if (m_line) {
            m_chip = gpiod_line_get_chip(m_line);
            break;
        }
    }

    if (!m_line) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "GpioPin::openByName: line '%s' not found", lineName.c_str());
        return false;
    }

    return configure(dir, initialVal);
}

bool GpioPin::configure(PinDirection dir, LogicLevel initialVal) {
    m_dir = dir;
    int rc;
    if (dir == PinDirection::OUTPUT) {
        rc = gpiod_line_request_output(m_line, "dashcam",
                                       static_cast<int>(initialVal));
    } else {
        rc = gpiod_line_request_input(m_line, "dashcam");
    }
    if (rc < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "GpioPin::configure: request failed: %s", ::strerror(errno));
        gpiod_chip_close(m_chip); m_chip = nullptr; m_line = nullptr;
        return false;
    }
    doLog(m_log, dashcam::log::LogLevel::INFO,
          "GpioPin: '%s' open as %s",
          gpiod_line_name(m_line) ? gpiod_line_name(m_line) : "?",
          (dir == PinDirection::OUTPUT) ? "OUTPUT" : "INPUT");
    return true;
}

bool GpioPin::read(LogicLevel& level) const {
    if (!m_line) return false;
    int v = gpiod_line_get_value(m_line);
    if (v < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "GpioPin::read: %s", ::strerror(errno));
        return false;
    }
    level = static_cast<LogicLevel>(v);
    return true;
}

bool GpioPin::write(LogicLevel level) {
    if (!m_line || m_dir != PinDirection::OUTPUT) return false;
    if (gpiod_line_set_value(m_line, static_cast<int>(level)) < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "GpioPin::write: %s", ::strerror(errno));
        return false;
    }
    return true;
}

bool GpioPin::toggle() {
    LogicLevel cur;
    if (!read(cur)) return false;
    return write(cur == LogicLevel::LOW ? LogicLevel::HIGH : LogicLevel::LOW);
}

bool GpioPin::isOpen() const { return m_line != nullptr; }

void GpioPin::close() {
    if (m_line) { gpiod_line_release(m_line); m_line = nullptr; }
    if (m_chip) { gpiod_chip_close(m_chip);   m_chip = nullptr; }
}

// ─── GpioWatcher ──────────────────────────────────────────────────────────────

GpioWatcher::GpioWatcher()  = default;
GpioWatcher::~GpioWatcher() { close(); }

bool GpioWatcher::open(const std::string& chipPath, uint32_t offset,
                        EdgeTrigger edge,
                        const dashcam::log::LogCallback& log) {
    close();
    m_log  = log;
    m_edge = edge;

    m_chip = gpiod_chip_open(chipPath.c_str());
    if (!m_chip) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "GpioWatcher::open: cannot open '%s': %s",
              chipPath.c_str(), ::strerror(errno));
        return false;
    }
    m_line = gpiod_chip_get_line(m_chip, offset);
    if (!m_line) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "GpioWatcher::open: line %u not found", offset);
        gpiod_chip_close(m_chip); m_chip = nullptr;
        return false;
    }
    return requestEdge(edge);
}

bool GpioWatcher::openByName(const std::string& lineName, EdgeTrigger edge,
                               const dashcam::log::LogCallback& log) {
    close();
    m_log  = log;
    m_edge = edge;

    for (const char* chip_path : {"/dev/gpiochip0", "/dev/gpiochip1"}) {
        m_line = findLineOnChip(chip_path, lineName);
        if (m_line) { m_chip = gpiod_line_get_chip(m_line); break; }
    }
    if (!m_line) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "GpioWatcher::openByName: line '%s' not found", lineName.c_str());
        return false;
    }
    return requestEdge(edge);
}

bool GpioWatcher::requestEdge(EdgeTrigger edge) {
    int rc;
    switch (edge) {
    case EdgeTrigger::RISING:
        rc = gpiod_line_request_rising_edge_events(m_line, "dashcam");  break;
    case EdgeTrigger::FALLING:
        rc = gpiod_line_request_falling_edge_events(m_line, "dashcam"); break;
    default:
        rc = gpiod_line_request_both_edges_events(m_line, "dashcam");   break;
    }
    if (rc < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "GpioWatcher: edge request failed: %s", ::strerror(errno));
        gpiod_chip_close(m_chip); m_chip = nullptr; m_line = nullptr;
        return false;
    }
    if (::pipe(m_pipe) < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "GpioWatcher: pipe() failed: %s", ::strerror(errno));
        gpiod_line_release(m_line);
        gpiod_chip_close(m_chip); m_chip = nullptr; m_line = nullptr;
        return false;
    }
    doLog(m_log, dashcam::log::LogLevel::INFO,
          "GpioWatcher: '%s' watching %s edges",
          gpiod_line_name(m_line) ? gpiod_line_name(m_line) : "?",
          (edge == EdgeTrigger::RISING)  ? "RISING" :
          (edge == EdgeTrigger::FALLING) ? "FALLING" : "BOTH");
    return true;
}

void GpioWatcher::setCallback(EdgeCallback cb) {
    std::lock_guard<std::mutex> lk(m_cbMtx);
    m_cb = std::move(cb);
}

bool GpioWatcher::start() {
    if (!m_line) {
        doLog(m_log, dashcam::log::LogLevel::ERROR, "GpioWatcher::start: not open");
        return false;
    }
    if (m_running.load()) return true;
    m_running.store(true);
    m_thread = std::thread(&GpioWatcher::watchLoop, this);
    doLog(m_log, dashcam::log::LogLevel::INFO, "GpioWatcher::start: thread running");
    return true;
}

void GpioWatcher::stop() {
    if (!m_running.load()) return;
    m_running.store(false);
    if (m_pipe[1] >= 0) {
        uint8_t b = 1;
        if (::write(m_pipe[1], &b, 1) < 0)
            doLog(m_log, dashcam::log::LogLevel::WARN,
                  "GpioWatcher::stop: write to wake pipe failed: %s", ::strerror(errno));
    }
    if (m_thread.joinable()) m_thread.join();
    doLog(m_log, dashcam::log::LogLevel::INFO, "GpioWatcher::stop: thread stopped");
}

void GpioWatcher::close() {
    stop();
    if (m_line) { gpiod_line_release(m_line); m_line = nullptr; }
    if (m_chip) { gpiod_chip_close(m_chip);   m_chip = nullptr; }
    if (m_pipe[0] >= 0) { ::close(m_pipe[0]); m_pipe[0] = -1; }
    if (m_pipe[1] >= 0) { ::close(m_pipe[1]); m_pipe[1] = -1; }
}

void GpioWatcher::watchLoop() {
    int evtFd = gpiod_line_event_get_fd(m_line);
    if (evtFd < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "GpioWatcher: event fd unavailable");
        m_running.store(false);
        return;
    }

    while (m_running.load()) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(evtFd,     &rfds);
        FD_SET(m_pipe[0], &rfds);
        int nfds = std::max(evtFd, m_pipe[0]) + 1;

        int ret = ::select(nfds, &rfds, nullptr, nullptr, nullptr);
        if (ret < 0) {
            if (errno == EINTR) continue;
            break;
        }

        if (FD_ISSET(m_pipe[0], &rfds)) {
            uint8_t buf[16]; ::read(m_pipe[0], buf, sizeof(buf));
            break;
        }

        if (!FD_ISSET(evtFd, &rfds)) continue;

        struct gpiod_line_event evt{};
        if (gpiod_line_event_read(m_line, &evt) < 0) continue; //beware of bouncing button or noisy signal

        LogicLevel lvl = (evt.event_type == GPIOD_LINE_EVENT_RISING_EDGE)
                         ? LogicLevel::HIGH : LogicLevel::LOW;
        int64_t tsNs = static_cast<int64_t>(evt.ts.tv_sec) * 1'000'000'000LL
                     + evt.ts.tv_nsec;

        EdgeCallback localCb;
        { std::lock_guard<std::mutex> lk(m_cbMtx); localCb = m_cb; }
        if (localCb) localCb(lvl, tsNs);
    }
}

// ─── GpioPeripheral ──────────────────────────────────────────────────────────

GpioPeripheral::GpioPeripheral(GpioPin& controlPin, dashcam::bus::IBus& bus)
    : m_pin(controlPin), m_bus(bus)
{}

void GpioPeripheral::reset(uint32_t pulseMs) {
    m_pin.write(LogicLevel::LOW);
    std::this_thread::sleep_for(std::chrono::milliseconds(pulseMs));
    m_pin.write(LogicLevel::HIGH);
}

void GpioPeripheral::assertCS()          { m_pin.write(LogicLevel::LOW);  }
void GpioPeripheral::releaseCS()         { m_pin.write(LogicLevel::HIGH); }
void GpioPeripheral::setEnabled(bool on) { m_pin.write(on ? LogicLevel::HIGH : LogicLevel::LOW); }

} // namespace dashcam::gpio
