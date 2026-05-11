/**
 * @file libspi.h
 * @brief SPI master via /dev/spidevN.N and Linux spidev ioctl.
 *
 * Supports full-duplex transfer and half-duplex read/write helpers.
 * Speed and mode can be overridden per-transfer if needed.
 *
 * SPI devices on the Jetson Orin Nano 40-pin header:
 *   /dev/spidev0.0  →  SPI0, CE0  (pins 19/21/23/24)
 *   /dev/spidev0.1  →  SPI0, CE1  (pins 19/21/23/26)
 *   /dev/spidev1.0  →  SPI1, CE0  (pins 38/35/40/11)
 *   /dev/spidev1.1  →  SPI1, CE1
 *
 * SPI modes (CPOL / CPHA):
 *   Mode 0: CPOL=0 CPHA=0  — most common (e.g. ICM-42688, MPU-6500)
 *   Mode 1: CPOL=0 CPHA=1
 *   Mode 2: CPOL=1 CPHA=0
 *   Mode 3: CPOL=1 CPHA=1  — e.g. SPI SD cards
 *
 * Typical usage:
 * @code
 *   dashcam::spi::SpiConfig cfg;
 *   cfg.mode    = 0;
 *   cfg.speedHz = 8000000;   // 8 MHz
 *
 *   dashcam::spi::SpiBus spi;
 *   spi.open("/dev/spidev0.0", cfg, log);
 *
 *   uint8_t tx[2] = {0x75 | 0x80, 0x00};   // read WHO_AM_I register
 *   uint8_t rx[2] = {};
 *   spi.transfer(tx, rx, 2);
 *   // rx[1] = device ID
 * @endcode
 */

#ifndef LIBSPI_H
#define LIBSPI_H

#include "liblog.h"

#include <cstdint>
#include <string>

namespace dashcam::spi {

// ─── SpiConfig ────────────────────────────────────────────────────────────────

struct SpiConfig {
    uint8_t  mode        = 0;        ///< SPI mode 0..3 (CPOL<<1 | CPHA).
    uint8_t  bitsPerWord = 8;        ///< Word width; 8 for almost all devices.
    uint32_t speedHz     = 1000000;  ///< Clock frequency in Hz.
    bool     lsbFirst    = false;    ///< MSB-first is standard; set true only if device requires it.
};

// ─── SpiBus ───────────────────────────────────────────────────────────────────

class SpiBus {
public:
    SpiBus();
    ~SpiBus();

    SpiBus(const SpiBus&)            = delete;
    SpiBus& operator=(const SpiBus&) = delete;

    /**
     * @brief Open a spidev device and apply configuration.
     * @param device  e.g. "/dev/spidev0.0"
     * @param cfg     Mode, clock speed, and word width.
     * @param log     Optional log callback.
     * @return true on success.
     */
    bool open(const std::string& device, const SpiConfig& cfg = {},
              const dashcam::log::LogCallback& log = {});

    /**
     * @brief Full-duplex transfer.
     *
     * @p txBuf and/or @p rxBuf may be nullptr.
     * If txBuf is nullptr, 0x00 is sent for every byte.
     * If rxBuf is nullptr, received bytes are discarded.
     * @return true if the ioctl succeeded.
     */
    bool transfer(const uint8_t* txBuf, uint8_t* rxBuf, size_t len);

    bool write(const uint8_t* buf, size_t len);  ///< TX only (rxBuf = nullptr).
    bool read (uint8_t* buf, size_t len);         ///< RX only (txBuf = nullptr).

    /**
     * @brief Change the clock speed after the bus is open.
     * @return true if the ioctl succeeded.
     */
    bool setSpeedHz(uint32_t hz);

    void close();
    bool isOpen() const;

private:
    int       m_fd  = -1;
    SpiConfig m_cfg;
    dashcam::log::LogCallback m_log;
};

} // namespace dashcam::spi

#endif // LIBSPI_H
