/**
 * @file ibus.h
 * @brief Abstract serial bus interface implemented by Uart, SpiBus, and I2cBus.
 *
 * IBus captures the lifecycle and raw byte-transfer behaviour that is common
 * to all three bus drivers.  GpioPeripheral (in libgpio.h) uses IBus to drive
 * data transfers while a GpioPin controls the peripheral's CS / RESET / ENABLE
 * line.
 *
 * For I2C, call setDevice(addr) once before the first send() / receive() to
 * select the target device address.  The call is a no-op on UART and SPI.
 *
 * Ownership: IBus instances are not owned by the interface; their lifetime
 * is the caller's responsibility.
 */

#ifndef IBUS_H
#define IBUS_H

#include <cstddef>
#include <cstdint>

namespace dashcam::bus {

enum class BusType { UART, SPI, I2C };

class IBus {
public:
    virtual ~IBus() = default;

    /// Identifies the concrete bus type without dynamic_cast.
    virtual BusType type() const = 0;

    virtual bool isOpen() const = 0;
    virtual void close()        = 0;

    /**
     * @brief Select a device by address.
     *
     * Required for I2C before the first send() / receive(); sets the 7-bit
     * slave address used for subsequent transfers.  No-op on UART and SPI.
     */
    virtual void setDevice(uint8_t /*addr*/) {}

    /**
     * @brief Write @p len bytes from @p buf to the bus.
     *
     * UART/SPI: raw byte stream.
     * I2C:      directed to the address set by setDevice().
     *
     * @return true on success.
     */
    virtual bool send(const uint8_t* buf, size_t len) = 0;

    /**
     * @brief Read up to @p len bytes from the bus into @p buf.
     *
     * @param timeoutMs  < 0 = block; 0 = non-blocking; > 0 = wait up to N ms.
     *                   SPI transfers are always synchronous; timeoutMs is ignored.
     * @return Bytes received, or -1 on error.
     */
    virtual int receive(uint8_t* buf, size_t len, int timeoutMs = 1000) = 0;

    /**
     * @brief Discard pending RX / TX data.
     *
     * Implemented by UART (both directions).  No-op on SPI and I2C.
     */
    virtual void flush() {}
};

} // namespace dashcam::bus

#endif // IBUS_H
