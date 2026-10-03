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

    /**
     * How long close() waits on unsent output that has stopped moving before
     * it discards the rest, on top of the line time of what is still queued
     * at @c baudRate (a DMA-fed UART such as ttyTHS* reports no progress until
     * a whole transfer of up to 4 KiB completes).  Output that keeps draining
     * is waited for in full (up to 30 s), so a write() just before close()
     * still goes out, even on a slow UART; a peer that has stopped reading
     * holds close() for this long plus that line time instead of the kernel's
     * closing_wait (30 s): about 360 ms for the 1280 bytes cdc-acm reports
     * queued on the MKR Zero at 115200.  See close().
     */
    int      closeDrainMs = 250;
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

    /**
     * @brief Close the port without waiting on a peer that stopped reading.
     *
     * A plain ::close() waits for unsent output for the driver's closing_wait,
     * 30 s by default, and a USB-CDC device whose firmware never reads its
     * port never takes it.  So close() waits while the output keeps draining
     * (at most 30 s), gives output that has stopped moving
     * @c UartConfig::closeDrainMs plus its line time at the configured baud,
     * then discards the rest (TCOFLUSH).  cdc-acm in the L4T 5.15 kernel
     * cannot discard bytes already handed to the USB host; when some remain,
     * it also sets the port's closing_wait to NONE
     * (TIOCSSERIAL).  That needs CAP_SYS_ADMIN (the --privileged containers
     * have it), lasts until the device re-enumerates, and is only done on a
     * port that has stopped draining.  Without the capability the close waits
     * as before, and a WARN says so.  The exclusive lock taken by open() is
     * dropped first, so a tty that outlives the close can be opened again.
     */
    void close()    override;
    bool isOpen()   const override;
    int  fd()       const;    ///< Raw fd — use for poll/select in application code.

    // ── IBus interface ────────────────────────────────────────────────────────
    dashcam::bus::BusType type() const override { return dashcam::bus::BusType::UART; }
    bool send   (const uint8_t* buf, size_t len) override { return write(buf, len); }
    int  receive(uint8_t* buf, size_t len, int timeoutMs = 1000) override { return read(buf, len, timeoutMs); }

private:
    int  m_fd = -1;
    bool m_exclusive    = false; ///< TIOCEXCL is set on m_fd; close() clears it.
    int  m_closeDrainMs = 0;     ///< UartConfig::closeDrainMs of the open port.
    int64_t m_charNs    = 0;     ///< Line time of one character at the open port's framing.
    dashcam::log::LogCallback m_log;
};

} // namespace dashcam::uart

#endif // LIBUART_H
