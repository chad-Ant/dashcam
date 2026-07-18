/**
 * @file libi2c.h
 * @brief I2C master bus access via /dev/i2c-N and Linux kernel ioctl.
 *
 * Uses the standard <linux/i2c-dev.h> interface — no third-party library.
 * Supports raw read/write, register-addressed byte/word access, and a bus
 * scan that probes all 7-bit addresses.
 *
 * I2C buses on the Jetson Orin Nano 40-pin header:
 *   /dev/i2c-1  →  header pins 3 (SDA) and 5 (SCL)   — GP I2C bus 1
 *   /dev/i2c-0  →  header pins 27 (SDA) and 28 (SCL)  — GP I2C bus 0
 *
 * Typical usage (read a byte register from a sensor at 0x40):
 * @code
 *   dashcam::i2c::I2cBus bus;
 *   bus.open("/dev/i2c-1", log);
 *
 *   uint8_t val;
 *   bus.readReg8(0x40, 0x00, val);
 * @endcode
 */

#ifndef LIBI2C_H
#define LIBI2C_H

#include "ibus.h"
#include "liblog.h"

#include <cstdint>
#include <string>
#include <vector>

namespace dashcam::i2c {

/**
 * @brief I2C master over /dev/i2c-N.
 *
 * @note Not thread-safe.  A single I2cBus owns one fd whose I2C_SLAVE target is
 *       shared mutable state; use one I2cBus per thread, or serialise all calls
 *       on a given instance externally.  Note that even per-call locking would
 *       not make a setDevice()+send()+receive() sequence atomic against another
 *       thread's transaction on the same bus — hold the external lock across the
 *       whole logical transaction.
 */
class I2cBus : public dashcam::bus::IBus {
public:
    I2cBus();
    ~I2cBus() override;

    I2cBus(const I2cBus&)            = delete;
    I2cBus& operator=(const I2cBus&) = delete;

    /**
     * @brief Open an I2C bus device.
     * @param device  e.g. "/dev/i2c-1"
     * @return true on success.
     */
    bool open(const std::string& device, const dashcam::log::LogCallback& log = {});

    // ── Raw read/write ────────────────────────────────────────────────────────

    /// Write @p len bytes from @p buf to device at @p addr.
    bool write(uint8_t addr, const uint8_t* buf, size_t len);

    /// Read @p len bytes into @p buf from device at @p addr.
    bool read (uint8_t addr, uint8_t* buf, size_t len);

    // ── Register-addressed access (most sensors / RTCs / IOexpanders) ─────────

    bool writeReg8 (uint8_t addr, uint8_t reg, uint8_t  value);
    bool writeReg16(uint8_t addr, uint8_t reg, uint16_t value); ///< Big-endian on bus.
    bool readReg8  (uint8_t addr, uint8_t reg, uint8_t&  value);
    bool readReg16 (uint8_t addr, uint8_t reg, uint16_t& value); ///< Big-endian from device.

    /// Read @p len register bytes starting at @p reg.
    bool readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len);

    // ── Bus scan ──────────────────────────────────────────────────────────────

    /**
     * @brief Probe all 7-bit addresses 0x03..0x77.
     *
     * Addresses that ACK are added to @p found.  Does NOT communicate with
     * reserved addresses (0x00..0x02, 0x78..0x7f).
     *
     * @return true if the scan completed without a fatal error.
     */
    bool scanBus(std::vector<uint8_t>& found);

    void close()    override;
    bool isOpen()   const override;

    // ── IBus interface ────────────────────────────────────────────────────────
    dashcam::bus::BusType type() const override { return dashcam::bus::BusType::I2C; }

    /**
     * @brief Select the 7-bit I2C device address for subsequent send/receive calls.
     *
     * Must be called before the first send() or receive() when using IBus.
     * Equivalent to calling I2C_SLAVE ioctl on the bus file descriptor.
     */
    void setDevice(uint8_t addr) override;

    /// Write to the device address set by setDevice().
    bool send   (const uint8_t* buf, size_t len) override;

    /// Read from the device address set by setDevice().  timeoutMs is ignored
    /// (I2C reads block until the device ACKs or the kernel returns an error).
    int  receive(uint8_t* buf, size_t len, int timeoutMs = 1000) override;

private:
    bool setTarget(uint8_t addr);

    int  m_fd          = -1;
    int  m_curTarget   = -1;   ///< Cached I2C_SLAVE address to avoid redundant ioctl.
    dashcam::log::LogCallback m_log;
};

} // namespace dashcam::i2c

#endif // LIBI2C_H
