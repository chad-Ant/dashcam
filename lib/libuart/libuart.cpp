#include "libuart.h"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
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

static speed_t baudToSpeed(uint32_t baud) {
    // Standard POSIX constants.
    switch (baud) {
    case     50: return B50;
    case     75: return B75;
    case    110: return B110;
    case    134: return B134;
    case    150: return B150;
    case    200: return B200;
    case    300: return B300;
    case    600: return B600;
    case   1200: return B1200;
    case   1800: return B1800;
    case   2400: return B2400;
    case   4800: return B4800;
    case   9600: return B9600;
    case  19200: return B19200;
    case  38400: return B38400;
    case  57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 1152000: return B1152000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    case 2500000: return B2500000;
    case 3000000: return B3000000;
    case 3500000: return B3500000;
    case 4000000: return B4000000;
    default:      return B0;  // Invalid — open() will reject it.
    }
}

} // namespace

// ─── dashcam::uart ────────────────────────────────────────────────────────────

namespace dashcam::uart {

Uart::Uart()  = default;
Uart::~Uart() { close(); }

bool Uart::open(const std::string& device, const UartConfig& cfg,
                const dashcam::log::LogCallback& log) {
    close();
    m_log = log;

    speed_t speed = baudToSpeed(cfg.baudRate);
    if (speed == B0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "Uart::open: unsupported baud rate %u", cfg.baudRate);
        return false;
    }

    m_fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (m_fd < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "Uart::open: cannot open '%s': %s",
              device.c_str(), ::strerror(errno));
        return false;
    }

    // Clear O_NONBLOCK — we manage timeouts via select().
    int flags = ::fcntl(m_fd, F_GETFL, 0);
    if (flags >= 0) ::fcntl(m_fd, F_SETFL, flags & ~O_NONBLOCK);

    struct termios tty{};
    if (::tcgetattr(m_fd, &tty) < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "Uart::open: tcgetattr failed: %s", ::strerror(errno));
        ::close(m_fd); m_fd = -1;
        return false;
    }

    ::cfsetispeed(&tty, speed);
    ::cfsetospeed(&tty, speed);
    ::cfmakeraw(&tty);          // raw mode: no processing, no echo, no signals

    // Data bits
    tty.c_cflag &= ~CSIZE;
    switch (cfg.dataBits) {
    case 5: tty.c_cflag |= CS5; break;
    case 6: tty.c_cflag |= CS6; break;
    case 7: tty.c_cflag |= CS7; break;
    default: tty.c_cflag |= CS8; break;
    }

    // Stop bits
    if (cfg.stopBits == 2) tty.c_cflag |= CSTOPB;
    else                    tty.c_cflag &= ~CSTOPB;

    // Parity
    if (cfg.parityOdd) {
        tty.c_cflag |= (PARENB | PARODD);
    } else if (cfg.parityEven) {
        tty.c_cflag |=  PARENB;
        tty.c_cflag &= ~PARODD;
    } else {
        tty.c_cflag &= ~(PARENB | PARODD);
    }

    // Flow control
    if (cfg.flowControl) tty.c_cflag |= CRTSCTS;
    else                  tty.c_cflag &= ~CRTSCTS;

    tty.c_cflag |= CREAD | CLOCAL;

    // Blocking read: return as soon as any byte arrives.
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (::tcsetattr(m_fd, TCSANOW, &tty) < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "Uart::open: tcsetattr failed: %s", ::strerror(errno));
        ::close(m_fd); m_fd = -1;
        return false;
    }

    ::tcflush(m_fd, TCIOFLUSH);

    doLog(m_log, dashcam::log::LogLevel::INFO,
          "Uart::open: %s  %u %d%s%d%s",
          device.c_str(), cfg.baudRate,
          cfg.dataBits,
          cfg.parityOdd ? "O" : (cfg.parityEven ? "E" : "N"),
          cfg.stopBits,
          cfg.flowControl ? " [RTS/CTS]" : "");
    return true;
}

int Uart::read(uint8_t* buf, size_t len, int timeoutMs) {
    if (m_fd < 0) return -1;

    if (timeoutMs == 0) {
        // Non-blocking: one shot, return whatever is already buffered.
        int flags = ::fcntl(m_fd, F_GETFL, 0);
        if (flags >= 0) ::fcntl(m_fd, F_SETFL, flags | O_NONBLOCK);
        ssize_t n = ::read(m_fd, buf, len);
        if (flags >= 0) ::fcntl(m_fd, F_SETFL, flags);
        return static_cast<int>(n < 0 ? (errno == EAGAIN ? 0 : -1) : n);
    }

    // timeoutMs < 0 blocks indefinitely; > 0 waits up to that long.  The port is
    // configured VMIN=0/VTIME=0, so a bare read() never blocks — we must wait via
    // poll() (also fd-number safe, unlike select()) before reading.
    struct pollfd pfd;
    pfd.fd = m_fd; pfd.events = POLLIN; pfd.revents = 0;
    int ret;
    do {
        ret = ::poll(&pfd, 1, (timeoutMs < 0) ? -1 : timeoutMs);
    } while (ret < 0 && errno == EINTR);
    if (ret < 0)  return -1;
    if (ret == 0) return 0;   // timed out (only reachable when timeoutMs > 0)

    ssize_t n = ::read(m_fd, buf, len);
    return static_cast<int>(n < 0 ? -1 : n);
}

bool Uart::readLine(std::string& line, int timeoutMs) {
    if (m_fd < 0) return false;
    line.clear();
    // Cap the accumulated length so a stream that never sends a newline (noise,
    // a wrong-baud sensor) can't grow the string without bound.
    constexpr size_t kMaxLine = 4096;
    while (line.size() < kMaxLine) {
        uint8_t c;
        int n = read(&c, 1, timeoutMs);
        if (n <= 0)   return false;
        if (c == '\r') continue;
        if (c == '\n') return true;
        line += static_cast<char>(c);
    }
    doLog(m_log, dashcam::log::LogLevel::WARN,
          "Uart::readLine: no newline within %zu bytes; discarding", kMaxLine);
    return false;
}

bool Uart::write(const uint8_t* buf, size_t len) {
    if (m_fd < 0) return false;
    // Loop until every byte is written: a blocking write() can still return a
    // short count on signal or under RTS/CTS backpressure.
    size_t total = 0;
    while (total < len) {
        ssize_t n = ::write(m_fd, buf + total, len - total);
        if (n < 0) {
            if (errno == EINTR) continue;
            doLog(m_log, dashcam::log::LogLevel::ERROR,
                  "Uart::write: failed after %zu/%zu bytes: %s",
                  total, len, ::strerror(errno));
            return false;
        }
        total += static_cast<size_t>(n);
    }
    return true;
}

bool Uart::write(const std::string& str) {
    return write(reinterpret_cast<const uint8_t*>(str.data()), str.size());
}

void Uart::flush() {
    if (m_fd >= 0) ::tcflush(m_fd, TCIOFLUSH);
}

void Uart::close() {
    if (m_fd >= 0) { ::close(m_fd); m_fd = -1; }
}

bool Uart::isOpen() const { return m_fd >= 0; }
int  Uart::fd()     const { return m_fd; }

} // namespace dashcam::uart
