#include "libspi.h"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <vector>

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

// ─── dashcam::spi ─────────────────────────────────────────────────────────────

namespace dashcam::spi {

SpiBus::SpiBus()  = default;
SpiBus::~SpiBus() { close(); }

bool SpiBus::open(const std::string& device, const SpiConfig& cfg,
                  const dashcam::log::LogCallback& log) {
    close();
    m_log = log;
    m_cfg = cfg;

    m_fd = ::open(device.c_str(), O_RDWR);
    if (m_fd < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "SpiBus::open: cannot open '%s': %s",
              device.c_str(), ::strerror(errno));
        return false;
    }

    // Mode.  noCs sets SPI_NO_CS so the controller leaves its hardware CE alone
    // when the chip-select is driven by an external GPIO (see GpioPeripheral);
    // csHigh sets SPI_CS_HIGH for active-high select lines.
    uint8_t mode = cfg.mode & 0x03;
    if (cfg.lsbFirst) mode |= SPI_LSB_FIRST;
    if (cfg.noCs)     mode |= SPI_NO_CS;
    if (cfg.csHigh)   mode |= SPI_CS_HIGH;
    if (::ioctl(m_fd, SPI_IOC_WR_MODE, &mode) < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "SpiBus::open: SPI_IOC_WR_MODE failed: %s", ::strerror(errno));
        ::close(m_fd); m_fd = -1; return false;
    }

    // Bits per word
    uint8_t bits = cfg.bitsPerWord;
    if (::ioctl(m_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "SpiBus::open: SPI_IOC_WR_BITS_PER_WORD failed: %s", ::strerror(errno));
        ::close(m_fd); m_fd = -1; return false;
    }

    // Speed
    uint32_t speed = cfg.speedHz;
    if (::ioctl(m_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "SpiBus::open: SPI_IOC_WR_MAX_SPEED_HZ failed: %s", ::strerror(errno));
        ::close(m_fd); m_fd = -1; return false;
    }

    doLog(m_log, dashcam::log::LogLevel::INFO,
          "SpiBus::open: %s  mode=%u  %u bits  %u Hz",
          device.c_str(), cfg.mode, cfg.bitsPerWord, cfg.speedHz);
    return true;
}

bool SpiBus::transfer(const uint8_t* txBuf, uint8_t* rxBuf, size_t len) {
    if (m_fd < 0 || len == 0) return false;

    // spidev requires both tx and rx pointers in the ioctl struct; allocate
    // scratch buffers for whichever side the caller did not provide.
    std::vector<uint8_t> txScratch, rxScratch;
    if (!txBuf) { txScratch.assign(len, 0x00); txBuf = txScratch.data(); }
    if (!rxBuf) { rxScratch.resize(len);        rxBuf = rxScratch.data(); }

    struct spi_ioc_transfer tr{};
    tr.tx_buf        = reinterpret_cast<uintptr_t>(txBuf);
    tr.rx_buf        = reinterpret_cast<uintptr_t>(rxBuf);
    tr.len           = static_cast<uint32_t>(len);
    tr.speed_hz      = m_cfg.speedHz;
    tr.bits_per_word = m_cfg.bitsPerWord;
    tr.delay_usecs   = 0;

    if (::ioctl(m_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "SpiBus::transfer: ioctl failed: %s", ::strerror(errno));
        return false;
    }
    return true;
}

bool SpiBus::write(const uint8_t* buf, size_t len) {
    return transfer(buf, nullptr, len);
}

bool SpiBus::read(uint8_t* buf, size_t len) {
    return transfer(nullptr, buf, len);
}

bool SpiBus::setSpeedHz(uint32_t hz) {
    if (m_fd < 0) return false;
    if (::ioctl(m_fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "SpiBus::setSpeedHz: ioctl failed: %s", ::strerror(errno));
        return false;
    }
    m_cfg.speedHz = hz;
    return true;
}

void SpiBus::close() {
    if (m_fd >= 0) { ::close(m_fd); m_fd = -1; }
}

bool SpiBus::isOpen() const { return m_fd >= 0; }

} // namespace dashcam::spi
