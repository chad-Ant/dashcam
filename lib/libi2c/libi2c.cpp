#include "libi2c.h"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstring>

// ─── file-local helpers ───────────────────────────────────────────────────────

namespace {

static void doLog(const dashcam::log::LogCallback& cb, dashcam::log::LogLevel lvl,
                  const char* fmt, ...) {
    if (!cb) return;
    char buf[512];
    va_list ap; va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    cb(lvl, buf);
}

} // namespace

// ─── dashcam::i2c ─────────────────────────────────────────────────────────────

namespace dashcam::i2c {

I2cBus::I2cBus()  = default;
I2cBus::~I2cBus() { close(); }

bool I2cBus::open(const std::string& device, const dashcam::log::LogCallback& log) {
    close();
    m_log       = log;
    m_curTarget = -1;

    m_fd = ::open(device.c_str(), O_RDWR);
    if (m_fd < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "I2cBus::open: cannot open '%s': %s",
              device.c_str(), ::strerror(errno));
        return false;
    }
    doLog(m_log, dashcam::log::LogLevel::INFO,
          "I2cBus::open: %s  fd=%d", device.c_str(), m_fd);
    return true;
}

bool I2cBus::setTarget(uint8_t addr) {
    if (static_cast<int>(addr) == m_curTarget) return true;
    if (::ioctl(m_fd, I2C_SLAVE, static_cast<long>(addr)) < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "I2cBus: I2C_SLAVE 0x%02X failed: %s", addr, ::strerror(errno));
        return false;
    }
    m_curTarget = addr;
    return true;
}

bool I2cBus::write(uint8_t addr, const uint8_t* buf, size_t len) {
    if (m_fd < 0) return false;
    if (!setTarget(addr)) return false;
    ssize_t n = ::write(m_fd, buf, len);
    if (n != static_cast<ssize_t>(len)) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "I2cBus::write [0x%02X]: %s", addr, ::strerror(errno));
        return false;
    }
    return true;
}

bool I2cBus::read(uint8_t addr, uint8_t* buf, size_t len) {
    if (m_fd < 0) return false;
    if (!setTarget(addr)) return false;
    ssize_t n = ::read(m_fd, buf, len);
    if (n != static_cast<ssize_t>(len)) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "I2cBus::read [0x%02X]: %s", addr, ::strerror(errno));
        return false;
    }
    return true;
}

bool I2cBus::writeReg8(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return write(addr, buf, 2);
}

bool I2cBus::writeReg16(uint8_t addr, uint8_t reg, uint16_t value) {
    uint8_t buf[3] = {reg,
                      static_cast<uint8_t>(value >> 8),
                      static_cast<uint8_t>(value & 0xFF)};
    return write(addr, buf, 3);
}

bool I2cBus::readReg8(uint8_t addr, uint8_t reg, uint8_t& value) {
    return readRegs(addr, reg, &value, 1);
}

bool I2cBus::readReg16(uint8_t addr, uint8_t reg, uint16_t& value) {
    uint8_t buf[2];
    if (!readRegs(addr, reg, buf, 2)) return false;
    value = (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
    return true;
}

bool I2cBus::readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len) {
    if (m_fd < 0 || buf == nullptr || len == 0 || len > 0xFFFF) return false;

    // Single I2C_RDWR transaction: write the register pointer, then read, joined
    // by a repeated-START (no intervening STOP).  This is atomic on the bus and
    // works with devices that require repeated-START, unlike a separate
    // write() then read().  Requires an adapter with I2C_FUNC_I2C (standard on
    // the Tegra controllers).
    struct i2c_msg msgs[2];
    msgs[0].addr  = addr;
    msgs[0].flags = 0;                              // write
    msgs[0].len   = 1;
    msgs[0].buf   = &reg;
    msgs[1].addr  = addr;
    msgs[1].flags = I2C_M_RD;                       // read (repeated-START)
    msgs[1].len   = static_cast<uint16_t>(len);
    msgs[1].buf   = buf;

    struct i2c_rdwr_ioctl_data xfer;
    xfer.msgs  = msgs;
    xfer.nmsgs = 2;

    // I2C_RDWR carries the address per message and does not disturb the
    // I2C_SLAVE state, so m_curTarget stays valid for later read()/write().
    if (::ioctl(m_fd, I2C_RDWR, &xfer) < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "I2cBus::readRegs [0x%02X reg 0x%02X, %zu bytes]: %s",
              addr, reg, len, ::strerror(errno));
        return false;
    }
    return true;
}

bool I2cBus::scanBus(std::vector<uint8_t>& found) {
    if (m_fd < 0) return false;
    found.clear();
    for (uint8_t addr = 0x03; addr <= 0x77; ++addr) {
        if (::ioctl(m_fd, I2C_SLAVE, static_cast<long>(addr)) < 0) continue;
        m_curTarget = addr;
        uint8_t dummy;
        if (::read(m_fd, &dummy, 1) >= 0) {
            found.push_back(addr);
        }
    }
    return true;
}

void I2cBus::close() {
    if (m_fd >= 0) { ::close(m_fd); m_fd = -1; m_curTarget = -1; }
}

bool I2cBus::isOpen() const { return m_fd >= 0; }

// ── IBus interface ────────────────────────────────────────────────────────────

void I2cBus::setDevice(uint8_t addr) {
    setTarget(addr);
}

bool I2cBus::send(const uint8_t* buf, size_t len) {
    if (m_curTarget < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "I2cBus::send: no device selected — call setDevice() first");
        return false;
    }
    return write(static_cast<uint8_t>(m_curTarget), buf, len);
}

int I2cBus::receive(uint8_t* buf, size_t len, int /*timeoutMs*/) {
    if (m_curTarget < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "I2cBus::receive: no device selected — call setDevice() first");
        return -1;
    }
    return read(static_cast<uint8_t>(m_curTarget), buf, len)
        ? static_cast<int>(len) : -1;
}

} // namespace dashcam::i2c
