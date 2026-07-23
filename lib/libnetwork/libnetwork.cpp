/**
 * @file libnetwork.cpp
 * @brief POSIX-socket implementation of UdpSocket / TcpSocket / TcpServer.
 *
 * IPv4 only (AF_INET).  All waits are bounded by poll(); no operation blocks
 * indefinitely unless the caller passes a negative timeout.
 */

#include "libnetwork.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <utility>

namespace dashcam::network {

using LvL = dashcam::log::LogLevel;

namespace {

// Log only when a callback was supplied (an empty std::function must never be
// invoked — that would throw std::bad_function_call).
void say(const dashcam::log::LogCallback& log, LvL lvl, const std::string& msg) {
    if (log) log(lvl, msg);
}

// strerror_r has two incompatible signatures: GNU returns char* (message may be
// a static string, not buf); XSI/POSIX returns int (message written into buf).
// Overload on the return type so the correct one is chosen regardless of which
// feature-test macros the build happens to enable.
inline std::string strerrPick(char* ret, char* /*buf*/) { return ret ? ret : "error"; }
inline std::string strerrPick(int ret, char* buf)       { return ret == 0 ? std::string(buf) : "error"; }

std::string errnoStr() {
    char buf[128] = {0};
    return strerrPick(::strerror_r(errno, buf, sizeof(buf)), buf);
}

// Wait for @p events on @p fd for up to @p timeoutMs.
// Returns 1 if ready, 0 on timeout, -1 on error.
int pollOne(int fd, short events, int timeoutMs) {
    struct pollfd p;
    p.fd = fd;
    p.events = events;
    p.revents = 0;
    for (;;) {
        int r = ::poll(&p, 1, timeoutMs);
        if (r < 0 && errno == EINTR) continue;  // retry on signal
        if (r < 0) return -1;
        if (r == 0) return 0;
        return 1;
    }
}

// Resolve @p host : @p port to a single IPv4 sockaddr_in.
// Returns true and fills @p out on success.
bool resolveV4(const std::string& host, uint16_t port, sockaddr_in& out,
               const dashcam::log::LogCallback& log) {
    struct addrinfo hints;
    std::memset(&hints, 0, sizeof(hints));
    hints.ai_family   = AF_INET;      // IPv4 only — keeps the socket family unambiguous.
    hints.ai_socktype = SOCK_DGRAM;   // socktype is irrelevant to the address itself.

    char portStr[8];
    std::snprintf(portStr, sizeof(portStr), "%u", static_cast<unsigned>(port));

    struct addrinfo* res = nullptr;
    int rc = ::getaddrinfo(host.c_str(), portStr, &hints, &res);
    if (rc != 0 || res == nullptr) {
        say(log, LvL::ERROR, "resolve failed for " + host + ": " + gai_strerror(rc));
        if (res) ::freeaddrinfo(res);
        return false;
    }
    std::memcpy(&out, res->ai_addr, sizeof(sockaddr_in));
    ::freeaddrinfo(res);
    return true;
}

std::string addrToString(const sockaddr_in& a) {
    char ip[INET_ADDRSTRLEN] = {0};
    ::inet_ntop(AF_INET, &a.sin_addr, ip, sizeof(ip));
    return std::string(ip) + ":" + std::to_string(ntohs(a.sin_port));
}

} // namespace

// ─── UdpSocket ─────────────────────────────────────────────────────────────────

UdpSocket::~UdpSocket() { close(); }

UdpSocket::UdpSocket(UdpSocket&& o) noexcept
    : fd_(o.fd_.exchange(-1)), log_(std::move(o.log_)) {}

UdpSocket& UdpSocket::operator=(UdpSocket&& o) noexcept {
    if (this != &o) {
        close();
        fd_    = o.fd_.exchange(-1);
        log_   = std::move(o.log_);
    }
    return *this;
}

bool UdpSocket::open(uint16_t bindPort, const dashcam::log::LogCallback& log) {
    close();
    log_ = log;

    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        say(log_, LvL::ERROR, "UDP socket() failed: " + errnoStr());
        return false;
    }

    int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(bindPort);
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        say(log_, LvL::ERROR, "UDP bind(" + std::to_string(bindPort) + ") failed: " + errnoStr());
        close();
        return false;
    }
    return true;
}

bool UdpSocket::sendTo(const std::string& host, uint16_t port, const void* data,
                       size_t len, std::string* resolvedHost) {
    if (fd_ < 0) {
        say(log_, LvL::ERROR, "UDP sendTo on a closed socket");
        return false;
    }
    sockaddr_in dst;
    if (!resolveV4(host, port, dst, log_)) return false;
    if (resolvedHost) {
        char ip[INET_ADDRSTRLEN] = {0};
        if (::inet_ntop(AF_INET, &dst.sin_addr, ip, sizeof(ip)))
            *resolvedHost = ip;
        else
            resolvedHost->clear();
    }

    ssize_t n = ::sendto(fd_, data, len, MSG_NOSIGNAL,
                         reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
    if (n < 0) {
        say(log_, LvL::ERROR, "UDP sendto(" + host + ") failed: " + errnoStr());
        return false;
    }
    if (static_cast<size_t>(n) != len)
        say(log_, LvL::WARN, "UDP sendto short write: " + std::to_string(n) + "/" + std::to_string(len));
    return true;
}

long UdpSocket::recvFrom(void* buf, size_t bufLen, int timeoutMs,
                         std::string* srcHost, uint16_t* srcPort) {
    if (fd_ < 0) return -1;

    int pr = pollOne(fd_, POLLIN, timeoutMs);
    if (pr < 0) { say(log_, LvL::ERROR, "UDP poll failed: " + errnoStr()); return -1; }
    if (pr == 0) return 0;  // timeout

    sockaddr_in src;
    socklen_t   srcLen = sizeof(src);
    std::memset(&src, 0, sizeof(src));
    ssize_t n = ::recvfrom(fd_, buf, bufLen, 0,
                          reinterpret_cast<sockaddr*>(&src), &srcLen);
    if (n < 0) {
        say(log_, LvL::ERROR, "UDP recvfrom failed: " + errnoStr());
        return -1;
    }
    if (srcHost) {
        char ip[INET_ADDRSTRLEN] = {0};
        ::inet_ntop(AF_INET, &src.sin_addr, ip, sizeof(ip));
        *srcHost = ip;
    }
    if (srcPort) *srcPort = ntohs(src.sin_port);
    return static_cast<long>(n);
}

uint16_t UdpSocket::localPort() const {
    if (fd_ < 0) return 0;
    sockaddr_in a;
    socklen_t   len = sizeof(a);
    std::memset(&a, 0, sizeof(a));
    if (::getsockname(fd_, reinterpret_cast<sockaddr*>(&a), &len) < 0) return 0;
    return ntohs(a.sin_port);
}

void UdpSocket::close() {
    const int fd = fd_.exchange(-1);
    if (fd >= 0) ::close(fd);
}

// ─── TcpSocket ─────────────────────────────────────────────────────────────────

TcpSocket::TcpSocket(int fd, std::string peer)
    : fd_(fd), peer_(std::move(peer)) {}

TcpSocket::~TcpSocket() { close(); }

TcpSocket::TcpSocket(TcpSocket&& o) noexcept
    : fd_(o.fd_.exchange(-1)), peer_(std::move(o.peer_)),
      log_(std::move(o.log_)) {}

TcpSocket& TcpSocket::operator=(TcpSocket&& o) noexcept {
    if (this != &o) {
        close();
        fd_    = o.fd_.exchange(-1);
        peer_  = std::move(o.peer_);
        log_   = std::move(o.log_);
    }
    return *this;
}

bool TcpSocket::connect(const std::string& host, uint16_t port, int timeoutMs,
                        const dashcam::log::LogCallback& log) {
    close();
    log_ = log;

    sockaddr_in dst;
    if (!resolveV4(host, port, dst, log_)) return false;

    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ < 0) {
        say(log_, LvL::ERROR, "TCP socket() failed: " + errnoStr());
        return false;
    }

    // Non-blocking connect so a dead host cannot hang past the timeout.
    int flags = ::fcntl(fd_, F_GETFL, 0);
    ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

    int rc = ::connect(fd_, reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
    if (rc < 0 && errno != EINPROGRESS) {
        say(log_, LvL::ERROR, "TCP connect(" + host + ") failed: " + errnoStr());
        close();
        return false;
    }
    if (rc < 0) {  // EINPROGRESS: wait for writability, then check SO_ERROR.
        int pr = pollOne(fd_, POLLOUT, timeoutMs);
        if (pr <= 0) {
            say(log_, LvL::ERROR, "TCP connect(" + host + ") " +
                                  (pr == 0 ? "timed out" : std::string("poll failed: ") + errnoStr()));
            close();
            return false;
        }
        int soErr = 0;
        socklen_t soLen = sizeof(soErr);
        if (::getsockopt(fd_, SOL_SOCKET, SO_ERROR, &soErr, &soLen) < 0 || soErr != 0) {
            errno = soErr;
            say(log_, LvL::ERROR, "TCP connect(" + host + ") failed: " + errnoStr());
            close();
            return false;
        }
    }

    // Restore blocking mode for straightforward sendAll()/recv().
    ::fcntl(fd_, F_SETFL, flags);
    peer_ = addrToString(dst);
    return true;
}

bool TcpSocket::sendAll(const void* data, size_t len) {
    if (fd_ < 0) return false;
    const char* p = static_cast<const char*>(data);
    size_t sent = 0;
    while (sent < len) {  // bounded: strictly increasing `sent`, capped at len
        ssize_t n = ::send(fd_, p + sent, len - sent, MSG_NOSIGNAL);
        if (n < 0) {
            if (errno == EINTR) continue;
            say(log_, LvL::ERROR, "TCP send failed: " + errnoStr());
            return false;
        }
        if (n == 0) {  // should not happen for a blocking socket
            say(log_, LvL::ERROR, "TCP send returned 0 (peer closed?)");
            return false;
        }
        sent += static_cast<size_t>(n);
    }
    return true;
}

IoStatus TcpSocket::recv(void* buf, size_t bufLen, int timeoutMs, size_t& outBytes) {
    outBytes = 0;
    if (fd_ < 0) return IoStatus::Error;

    int pr = pollOne(fd_, POLLIN, timeoutMs);
    if (pr < 0) { say(log_, LvL::ERROR, "TCP poll failed: " + errnoStr()); return IoStatus::Error; }
    if (pr == 0) return IoStatus::Timeout;

    ssize_t n = ::recv(fd_, buf, bufLen, 0);
    if (n < 0) { say(log_, LvL::ERROR, "TCP recv failed: " + errnoStr()); return IoStatus::Error; }
    if (n == 0) return IoStatus::Closed;  // orderly shutdown by peer
    outBytes = static_cast<size_t>(n);
    return IoStatus::Ok;
}

void TcpSocket::shutdown() {
    const int fd = fd_.load();
    if (fd >= 0) ::shutdown(fd, SHUT_RDWR);  // unblock a concurrent send/recv; fd stays open
}

void TcpSocket::close() {
    const int fd = fd_.exchange(-1);
    if (fd >= 0) ::close(fd);
}

// ─── TcpServer ─────────────────────────────────────────────────────────────────

TcpServer::~TcpServer() { close(); }

bool TcpServer::listen(uint16_t port, int backlog, const dashcam::log::LogCallback& log) {
    close();
    log_ = log;

    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ < 0) {
        say(log_, LvL::ERROR, "TCP server socket() failed: " + errnoStr());
        return false;
    }

    int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(port);
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        say(log_, LvL::ERROR, "TCP server bind(" + std::to_string(port) + ") failed: " + errnoStr());
        close();
        return false;
    }
    if (::listen(fd_, backlog) < 0) {
        say(log_, LvL::ERROR, "TCP server listen() failed: " + errnoStr());
        close();
        return false;
    }

    // Read back the actual port (informative when the caller passed 0).
    sockaddr_in bound;
    socklen_t   blen = sizeof(bound);
    if (::getsockname(fd_, reinterpret_cast<sockaddr*>(&bound), &blen) == 0)
        port_ = ntohs(bound.sin_port);
    else
        port_ = port;

    // Self-pipe wake so close() can unblock a concurrent accept().
    int wakeFds[2] = {-1, -1};
    if (::pipe(wakeFds) < 0) {
        say(log_, LvL::ERROR, "TCP server wake pipe() failed: " + errnoStr());
        close();
        return false;
    }
    wake_[0].store(wakeFds[0]);
    wake_[1].store(wakeFds[1]);
    return true;
}

TcpSocket TcpServer::accept(int timeoutMs, IoStatus* status) {
    auto setStatus = [&](IoStatus s) { if (status) *status = s; };

    const int listenFd = fd_.load();
    const int wakeRead = wake_[0].load();
    if (listenFd < 0 || wakeRead < 0) {
        setStatus(IoStatus::Error);
        return TcpSocket{};
    }

    struct pollfd p[2];
    p[0].fd = listenFd; p[0].events = POLLIN; p[0].revents = 0;
    p[1].fd = wakeRead; p[1].events = POLLIN; p[1].revents = 0;

    int r;
    for (;;) {
        r = ::poll(p, 2, timeoutMs);
        if (r < 0 && errno == EINTR) continue;
        break;
    }
    if (r < 0)  { say(log_, LvL::ERROR, "TCP accept poll failed: " + errnoStr()); setStatus(IoStatus::Error);   return TcpSocket{}; }
    if (r == 0) { setStatus(IoStatus::Timeout); return TcpSocket{}; }

    // Woken by close() (wake pipe), or the listener was torn down underneath us
    // (fd_ closed by a concurrent close() → POLLNVAL/POLLERR/POLLHUP): report
    // Closed rather than blindly calling ::accept() on a dead descriptor.
    if (p[1].revents) {  // any event on the wake pipe means stop()
        setStatus(IoStatus::Closed);
        return TcpSocket{};
    }
    if (p[0].revents & (POLLNVAL | POLLERR | POLLHUP)) {
        setStatus(IoStatus::Closed);
        return TcpSocket{};
    }
    if (!(p[0].revents & POLLIN)) {  // spurious wake with nothing to accept
        setStatus(IoStatus::Timeout);
        return TcpSocket{};
    }

    sockaddr_in peer;
    socklen_t   plen = sizeof(peer);
    std::memset(&peer, 0, sizeof(peer));
    int cfd = ::accept(listenFd, reinterpret_cast<sockaddr*>(&peer), &plen);
    if (cfd < 0) {
        say(log_, LvL::ERROR, "TCP accept() failed: " + errnoStr());
        setStatus(IoStatus::Error);
        return TcpSocket{};
    }
    setStatus(IoStatus::Ok);
    return TcpSocket{cfd, addrToString(peer)};
}

void TcpServer::close() {
    // Wake a concurrent accept() before tearing the descriptors down.
    const int wakeWrite = wake_[1].load();
    if (wakeWrite >= 0) {
        const char b = 1;
        ssize_t wr = ::write(wakeWrite, &b, 1);
        (void)wr;  // best-effort wake; nothing actionable if the pipe is full/closed
    }
    const int fd = fd_.exchange(-1);
    const int wake0 = wake_[0].exchange(-1);
    const int wake1 = wake_[1].exchange(-1);
    if (fd >= 0) ::close(fd);
    if (wake0 >= 0) ::close(wake0);
    if (wake1 >= 0) ::close(wake1);
    port_.store(0);
}

} // namespace dashcam::network
