/**
 * @file libuart.h
 * @brief UART / serial-port wrapper using POSIX termios.
 *
 * Configured for raw mode (no line discipline, no echo) by default — correct
 * for binary protocols and GPS NMEA streams.
 *
 * Hardware UART on the Jetson Orin Nano 40-pin header:
 *   Pin 8  TX  →  /dev/ttyTHS1
 *   Pin 10 RX  →  /dev/ttyTHS1
 *
 * Typical usage (GPS NMEA):
 * @code
 *   dashcam::uart::UartConfig cfg;
 *   cfg.baudRate = 9600;
 *
 *   dashcam::uart::Uart gps;
 *   gps.open("/dev/ttyTHS1", cfg, log);
 *
 *   std::string line;
 *   while (gps.readLine(line, 1500))
 *       log(LvL::INFO, line);
 * @endcode
 */

#ifndef LIBUART_H
#define LIBUART_H

#include "ibus.h"
#include "liblog.h"

#include <cstdint>
#include <string>

namespace dashcam::uart {

// ─── UartConfig ───────────────────────────────────────────────────────────────

struct UartConfig {
    uint32_t baudRate    = 115200; ///< Baud rate (e.g. 9600, 115200, 921600).
                                   ///< Ignored by USB CDC-ACM devices, which have no line rate.
    uint8_t  dataBits    = 8;      ///< Character width: 5, 6, 7, or 8.
    uint8_t  stopBits    = 1;      ///< Stop bits: 1 or 2.
    bool     parityEven  = false;  ///< Even parity (parityOdd takes precedence if both set).
    bool     parityOdd   = false;  ///< Odd parity.
    bool     flowControl = false;  ///< RTS/CTS hardware flow control.

    /**
     * Take the port exclusively (TIOCEXCL): further open() calls by other
     * processes fail with EBUSY.  Worth setting on a USB device node, where
     * a stray `screen`/`minicom` would otherwise silently steal bytes from
     * a running application.
     */
    bool     exclusive   = false;

    /**
     * Leave HUPCL set, so closing the port lowers DTR/RTS (the POSIX default).
     *
     * Set **false** for USB-CDC devices whose MCU is reset by modem-control
     * lines — notably the ESP32-C3's native USB Serial/JTAG, where the DTR/RTS
     * pair is what esptool uses to force a reboot into download mode.  With
     * HUPCL left set, every close() of the port can bounce the microcontroller.
     */
    bool     hangupOnClose = true;
};

// ─── Uart ─────────────────────────────────────────────────────────────────────

class Uart : public dashcam::bus::IBus {
public:
    Uart();
    ~Uart() override;

    Uart(const Uart&)            = delete;
    Uart& operator=(const Uart&) = delete;

    /**
     * @brief Open and configure a serial port.
     *
     * @param device  Path to the device node, e.g. "/dev/ttyTHS1".
     * @param cfg     Baud rate and framing settings.
     * @param log     Optional log callback.
     * @return true on success.
     */
    bool open(const std::string& device, const UartConfig& cfg = {},
              const dashcam::log::LogCallback& log = {});

    /**
     * @brief Read up to @p len bytes.
     *
     * @param buf        Destination buffer.
     * @param len        Maximum bytes to read.
     * @param timeoutMs  < 0 = block; 0 = non-blocking; > 0 = wait up to N ms.
     * @return Bytes actually read, or -1 on error.
     */
    int  read(uint8_t* buf, size_t len, int timeoutMs = 1000);

    /**
     * @brief Read characters until '\n' (or '\r\n') or timeout.
     *
     * @param line      Receives the line without the trailing newline.
     * @param timeoutMs Per-character timeout; overall timeout is not bounded.
     * @return true if a newline was received; false on timeout or error.
     */
    bool readLine(std::string& line, int timeoutMs = 1000);

    bool write(const uint8_t* buf, size_t len);
    bool write(const std::string& str);

    /**
     * @brief Write with a bounded overall deadline.
     *
     * Unlike write(), this never blocks indefinitely: it waits for writability
     * with poll(POLLOUT) against an absolute deadline and gives up when the
     * deadline passes.  Required for USB CDC-ACM peers, where a device that
     * stops draining its endpoint would otherwise block write() forever — and,
     * if the caller holds a lock across it, wedge everything behind that lock.
     *
     * @param buf        Source buffer.
     * @param len        Bytes to write.
     * @param timeoutMs  Overall deadline for the whole transfer, in ms.
     * @return true only if all @p len bytes were written before the deadline.
     */
    bool writeTimeout(const uint8_t* buf, size_t len, int timeoutMs);

    void flush()    override; ///< Discard both RX and TX buffers.
    void close()    override;
    bool isOpen()   const override;
    int  fd()       const;    ///< Raw fd — use for poll/select in application code.

    // ── IBus interface ────────────────────────────────────────────────────────
    dashcam::bus::BusType type() const override { return dashcam::bus::BusType::UART; }
    bool send   (const uint8_t* buf, size_t len) override { return write(buf, len); }
    int  receive(uint8_t* buf, size_t len, int timeoutMs = 1000) override { return read(buf, len, timeoutMs); }

private:
    int m_fd = -1;
    dashcam::log::LogCallback m_log;
};

} // namespace dashcam::uart

#endif // LIBUART_H
