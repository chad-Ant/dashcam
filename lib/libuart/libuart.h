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

#include "liblog.h"

#include <cstdint>
#include <string>

namespace dashcam::uart {

// ─── UartConfig ───────────────────────────────────────────────────────────────

struct UartConfig {
    uint32_t baudRate    = 115200; ///< Baud rate (e.g. 9600, 115200, 921600).
    uint8_t  dataBits    = 8;      ///< Character width: 5, 6, 7, or 8.
    uint8_t  stopBits    = 1;      ///< Stop bits: 1 or 2.
    bool     parityEven  = false;  ///< Even parity (parityOdd takes precedence if both set).
    bool     parityOdd   = false;  ///< Odd parity.
    bool     flowControl = false;  ///< RTS/CTS hardware flow control.
};

// ─── Uart ─────────────────────────────────────────────────────────────────────

class Uart {
public:
    Uart();
    ~Uart();

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

    void flush();    ///< Discard both RX and TX buffers.
    void close();
    bool isOpen() const;
    int  fd()     const;   ///< Raw fd — use for poll/select in application code.

private:
    int m_fd = -1;
    dashcam::log::LogCallback m_log;
};

} // namespace dashcam::uart

#endif // LIBUART_H
