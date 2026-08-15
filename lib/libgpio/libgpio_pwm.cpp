#include "libgpio_pwm.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

namespace dashcam::gpio {

namespace {

void doLog(const dashcam::log::LogCallback& cb, dashcam::log::LogLevel lvl,
           const char* fmt, ...) {
    if (!cb) return;
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    cb(lvl, buf);
}

/// Single write(), not an ofstream. sysfs attributes expect one write per
/// value; a buffered stream is free to split it, and a split write to an
/// attribute like `period` is rejected rather than reassembled.
bool writeFileOnce(const std::string& path, const std::string& value) {
    const int fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
    if (fd < 0) return false;
    const ssize_t n = ::write(fd, value.data(), value.size());
    ::close(fd);
    return n == static_cast<ssize_t>(value.size());
}

bool readFileOnce(const std::string& path, std::string& out) {
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) return false;
    char buf[64];
    const ssize_t n = ::read(fd, buf, sizeof(buf) - 1);
    ::close(fd);
    if (n < 0) return false;
    buf[n] = '\0';
    out.assign(buf);
    while (!out.empty() && (out.back() == '\n' || out.back() == ' ')) out.pop_back();
    return true;
}

bool pathExists(const std::string& p) {
    struct stat st{};
    return ::stat(p.c_str(), &st) == 0;
}

} // namespace

PwmPin::PwmPin() = default;

PwmPin::~PwmPin() { close(); }

bool PwmPin::writeAttr(const std::string& attr, const std::string& value) const {
    if (m_chanPath.empty()) return false;
    return writeFileOnce(m_chanPath + "/" + attr, value);
}

bool PwmPin::readAttr(const std::string& attr, std::string& value) const {
    if (m_chanPath.empty()) return false;
    return readFileOnce(m_chanPath + "/" + attr, value);
}

bool PwmPin::writeAttrVerified(const std::string& attr, uint64_t value) const {
    const std::string s = std::to_string(value);
    if (!writeAttr(attr, s)) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "PWM: writing %s=%s failed: %s", attr.c_str(), s.c_str(), std::strerror(errno));
        return false;
    }

    // Read back. The kernel silently clamps or rejects some combinations —
    // notably a duty exceeding the current period — and an unverified write
    // surfaces much later as the wrong brightness, which reads as a bug in
    // whatever set the brightness rather than in the write that never landed.
    std::string back;
    if (!readAttr(attr, back)) return true;   // not readable on every driver
    if (std::strtoull(back.c_str(), nullptr, 10) != value) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "PWM: %s reads back %s after writing %s", attr.c_str(), back.c_str(), s.c_str());
        return false;
    }
    return true;
}

bool PwmPin::applyPeriodAndDuty(unsigned frequencyHz, float fraction) {
    if (frequencyHz == 0u) return false;
    if (fraction < 0.0f) fraction = 0.0f;
    if (fraction > 1.0f) fraction = 1.0f;

    const uint64_t periodNs = 1000000000ull / frequencyHz;
    if (periodNs == 0ull) return false;
    const uint64_t dutyNs = static_cast<uint64_t>(static_cast<double>(periodNs) * fraction);

    // Duty to zero FIRST, then the period, then the real duty.
    //
    // The kernel rejects any duty greater than the period, so lowering the
    // frequency while a large duty is still set fails — and the failure lands
    // on the period write, which looks like the frequency being unsupported
    // rather than an ordering mistake. Collapsing the duty first makes the
    // sequence valid for any pair of old and new values.
    if (!writeAttrVerified("duty_cycle", 0ull))      return false;
    if (!writeAttrVerified("period", periodNs))      return false;
    if (!writeAttrVerified("duty_cycle", dutyNs))    return false;

    m_frequencyHz = frequencyHz;
    m_duty        = fraction;
    return true;
}

bool PwmPin::open(unsigned chipIndex, unsigned channel, unsigned frequencyHz,
                  const dashcam::log::LogCallback& log) {
    close();
    m_log     = log;
    m_channel = channel;
    m_chipPath = "/sys/class/pwm/pwmchip" + std::to_string(chipIndex);
    m_chanPath = m_chipPath + "/pwm" + std::to_string(channel);

    if (!pathExists(m_chipPath)) {
        // Named explicitly. This is the pinmux step, it is easy to forget, and
        // the bare ENOENT gives no hint that a reboot is involved.
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "PWM: %s does not exist. The header pin is still muxed as GPIO - "
              "run /opt/nvidia/jetson-io/jetson-io.py to enable its PWM function, then REBOOT.",
              m_chipPath.c_str());
        m_chanPath.clear();
        return false;
    }

    if (pathExists(m_chanPath)) {
        // Already exported, almost always by a previous run that did not exit
        // cleanly. Adopted rather than refused: failing here would mean a crash
        // leaves the panel unusable until someone unexports it by hand. Not
        // marked as ours, so close() leaves it exported exactly as it was found.
        doLog(m_log, dashcam::log::LogLevel::WARN,
              "PWM: %s already exported - adopting it (previous run did not close cleanly)",
              m_chanPath.c_str());
        m_ownExport = false;
    } else {
        if (!writeFileOnce(m_chipPath + "/export", std::to_string(channel))) {
            doLog(m_log, dashcam::log::LogLevel::ERROR,
                  "PWM: exporting channel %u on %s failed: %s (root or a udev rule is usually needed)",
                  channel, m_chipPath.c_str(), std::strerror(errno));
            m_chanPath.clear();
            return false;
        }
        m_ownExport = true;

        // udev creates the attributes asynchronously, so the directory can
        // exist a moment before `period` is writable. Bounded wait — an
        // unbounded one would hang a startup path on a permissions problem.
        bool ready = false;
        for (int attempt = 0; attempt < 50 && !ready; ++attempt) {
            if (pathExists(m_chanPath + "/period")) { ready = true; break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (!ready) {
            doLog(m_log, dashcam::log::LogLevel::ERROR,
                  "PWM: %s/period did not appear within 500 ms of export", m_chanPath.c_str());
            writeFileOnce(m_chipPath + "/unexport", std::to_string(channel));
            m_chanPath.clear();
            return false;
        }
    }

    m_open = true;

    // Disabled while being configured, so the load never sees an unintended
    // pulse between the period and duty writes.
    if (!writeAttr("enable", "0")) {
        doLog(m_log, dashcam::log::LogLevel::WARN,
              "PWM: could not disable %s before configuring it", m_chanPath.c_str());
    }
    m_enabled = false;

    if (!applyPeriodAndDuty(frequencyHz, 0.0f)) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "PWM: configuring %s at %u Hz failed", m_chanPath.c_str(), frequencyHz);
        close();
        return false;
    }

    doLog(m_log, dashcam::log::LogLevel::INFO,
          "PWM: %s ready at %u Hz, duty 0 %%, output disabled", m_chanPath.c_str(), frequencyHz);
    return true;
}

bool PwmPin::setFrequency(unsigned frequencyHz) {
    if (!m_open) return false;
    // The FRACTION is preserved, not the nanosecond duty — see the header.
    return applyPeriodAndDuty(frequencyHz, m_duty);
}

bool PwmPin::setDuty(float fraction) {
    if (!m_open || m_frequencyHz == 0u) return false;
    if (fraction < 0.0f) fraction = 0.0f;
    if (fraction > 1.0f) fraction = 1.0f;

    const uint64_t periodNs = 1000000000ull / m_frequencyHz;
    const uint64_t dutyNs   = static_cast<uint64_t>(static_cast<double>(periodNs) * fraction);
    if (!writeAttrVerified("duty_cycle", dutyNs)) return false;
    m_duty = fraction;
    return true;
}

bool PwmPin::enable(bool on) {
    if (!m_open) return false;
    if (!writeAttr("enable", on ? "1" : "0")) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "PWM: %s enable=%d failed: %s", m_chanPath.c_str(), on ? 1 : 0, std::strerror(errno));
        return false;
    }
    m_enabled = on;
    return true;
}

void PwmPin::close() {
    if (!m_open) {
        m_chanPath.clear();
        m_chipPath.clear();
        return;
    }

    // Output down before the export goes, and in that order: unexporting an
    // enabled channel can leave the pin parked at whatever level it held, which
    // for an active-low output enable means the panel latches on and stays on.
    writeAttr("enable", "0");
    m_enabled = false;

    if (m_ownExport && !m_chipPath.empty()) {
        writeFileOnce(m_chipPath + "/unexport", std::to_string(m_channel));
    }

    m_open      = false;
    m_ownExport = false;
    m_duty      = 0.0f;
    m_chanPath.clear();
    m_chipPath.clear();
}

} // namespace dashcam::gpio
