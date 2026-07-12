#include "libcan.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cctype>
#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <sstream>

// ─── file-local helpers ───────────────────────────────────────────────────────

namespace {

static void doLog(const dashcam::log::LogCallback& cb, dashcam::log::LogLevel lvl,
                  const char* fmt, ...) {
    if (!cb) return;
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    cb(lvl, buf);
}

// Interface name: allow only alphanumeric + underscore to prevent shell injection.
static bool validIfaceName(const std::string& name) {
    if (name.empty() || name.size() >= IFNAMSIZ) return false;
    for (char c : name)
        if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') return false;
    return true;
}

static bool runCmd(const std::string& cmd, const dashcam::log::LogCallback& log) {
    doLog(log, dashcam::log::LogLevel::DEBUG, "  $ %s", cmd.c_str());
    int rc = ::system(cmd.c_str());
    if (rc != 0) {
        doLog(log, dashcam::log::LogLevel::ERROR,
              "command failed (rc=%d): %s", rc, cmd.c_str());
        return false;
    }
    return true;
}

} // namespace

// ─── dashcam::can ─────────────────────────────────────────────────────────────

namespace dashcam::can {

// ─── configureInterface ───────────────────────────────────────────────────────

bool configureInterface(const std::string& iface, const CanBusConfig& cfg,
                        const dashcam::log::LogCallback& log) {
    if (!validIfaceName(iface)) {
        doLog(log, dashcam::log::LogLevel::ERROR,
              "configureInterface: invalid interface name '%s'", iface.c_str());
        return false;
    }

    if (!runCmd("ip link set " + iface + " down", log)) return false;

    std::ostringstream cmd;
    cmd << "ip link set " << iface << " type can bitrate " << cfg.bitrate;
    if (cfg.fdMode) {
        uint32_t drate = (cfg.dataBitrate > 0) ? cfg.dataBitrate : cfg.bitrate;
        cmd << " dbitrate " << drate << " fd on";
    }
    if (cfg.loopback)   cmd << " loopback on";
    if (cfg.listenOnly) cmd << " listen-only on";
    if (!runCmd(cmd.str(), log)) return false;

    if (!runCmd("ip link set " + iface + " up", log)) return false;

    doLog(log, dashcam::log::LogLevel::INFO,
          "configureInterface: %s up at %u bps%s%s%s",
          iface.c_str(), cfg.bitrate,
          cfg.fdMode      ? " [CAN FD]"      : "",
          cfg.loopback    ? " [loopback]"    : "",
          cfg.listenOnly  ? " [listen-only]" : "");
    return true;
}

// ─── CanBus ───────────────────────────────────────────────────────────────────

CanBus::CanBus() = default;

CanBus::~CanBus() {
    close();
}

bool CanBus::open(const std::string& iface, bool fdEnabled,
                  const dashcam::log::LogCallback& log) {
    close();
    m_log       = log;
    m_fdEnabled = fdEnabled;

    m_fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (m_fd < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "CanBus::open: socket() failed: %s", ::strerror(errno));
        return false;
    }

    if (fdEnabled) {
        int en = 1;
        if (::setsockopt(m_fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &en, sizeof(en)) < 0) {
            doLog(m_log, dashcam::log::LogLevel::WARN,
                  "CanBus::open: CAN FD not available on '%s' (%s) — falling back to classic CAN",
                  iface.c_str(), ::strerror(errno));
            m_fdEnabled = false;
        }
    }

    // Without this the kernel silently drops all error frames (BUS_OFF, ACK errors, etc.).
    can_err_mask_t err_mask = CAN_ERR_MASK;
    if (::setsockopt(m_fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) < 0) {
        doLog(m_log, dashcam::log::LogLevel::WARN,
              "CanBus::open: CAN_RAW_ERR_FILTER failed: %s", ::strerror(errno));
    }

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
    if (::ioctl(m_fd, SIOCGIFINDEX, &ifr) < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "CanBus::open: interface '%s' not found: %s",
              iface.c_str(), ::strerror(errno));
        ::close(m_fd);
        m_fd = -1;
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(m_fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "CanBus::open: bind() on '%s' failed: %s",
              iface.c_str(), ::strerror(errno));
        ::close(m_fd);
        m_fd = -1;
        return false;
    }

    if (::pipe(m_pipe) < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "CanBus::open: pipe() failed: %s", ::strerror(errno));
        ::close(m_fd);
        m_fd = -1;
        return false;
    }

    doLog(m_log, dashcam::log::LogLevel::INFO,
          "CanBus::open: %s  fd=%d%s",
          iface.c_str(), m_fd, m_fdEnabled ? "  [CAN FD]" : "");
    return true;
}

void CanBus::setFilters(const std::vector<CanFilter>& filters) {
    if (m_fd < 0) return;
    if (filters.empty()) {
        ::setsockopt(m_fd, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0);
        return;
    }
    std::vector<struct can_filter> kf;
    kf.reserve(filters.size());
    for (const auto& f : filters) {
        struct can_filter k{};
        k.can_id   = f.id;
        k.can_mask = f.mask;
        if (f.extended) {
            k.can_id   |= CAN_EFF_FLAG;
            k.can_mask |= CAN_EFF_FLAG;
        }
        kf.push_back(k);
    }
    if (::setsockopt(m_fd, SOL_CAN_RAW, CAN_RAW_FILTER,
                     kf.data(),
                     static_cast<socklen_t>(kf.size() * sizeof(struct can_filter))) < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "CanBus::setFilters: CAN_RAW_FILTER failed: %s", ::strerror(errno));
    }
}

void CanBus::setReceiveCallback(ReceiveCallback cb) {
    std::lock_guard<std::mutex> lk(m_cbMtx);
    m_rxCb = std::move(cb);
}

void CanBus::setErrorCallback(ErrorCallback cb) {
    std::lock_guard<std::mutex> lk(m_cbMtx);
    m_errCb = std::move(cb);
}

bool CanBus::send(const CanFrame& frame) {
    if (m_fd < 0) return false;

    // Reject rather than silently truncate/downgrade — a caller sending more
    // bytes than the frame type holds, or an FD frame on a non-FD socket, is a
    // bug we want surfaced, not quietly corrupted data on the bus.
    if (frame.fdFrame) {
        if (!m_fdEnabled) {
            doLog(m_log, dashcam::log::LogLevel::ERROR,
                  "CanBus::send: FD frame requested but FD is not enabled on this socket");
            return false;
        }
        if (frame.len > CAN_FD_DLEN) {
            doLog(m_log, dashcam::log::LogLevel::ERROR,
                  "CanBus::send: FD payload %u exceeds %u bytes", frame.len, CAN_FD_DLEN);
            return false;
        }
    } else if (frame.len > CAN_CLASSIC_DLEN) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "CanBus::send: classic payload %u exceeds %u bytes", frame.len, CAN_CLASSIC_DLEN);
        return false;
    }

    std::lock_guard<std::mutex> lk(m_sendMtx);

    ssize_t written;
    if (frame.fdFrame) {   // m_fdEnabled and length already validated above
        struct canfd_frame cf{};
        cf.can_id = frame.id;
        if (frame.extended) cf.can_id |= CAN_EFF_FLAG;
        cf.flags = 0;
        if (frame.brs) cf.flags |= CANFD_BRS;
        cf.len = frame.len;
        std::memcpy(cf.data, frame.data, cf.len);
        written = ::write(m_fd, &cf, sizeof(cf));
        if (written != static_cast<ssize_t>(sizeof(cf))) {
            doLog(m_log, dashcam::log::LogLevel::ERROR,
                  "CanBus::send(FD): write failed: %s", ::strerror(errno));
            return false;
        }
    } else {
        struct can_frame cf{};
        cf.can_id = frame.id;
        if (frame.extended) cf.can_id |= CAN_EFF_FLAG;
        if (frame.rtr)      cf.can_id |= CAN_RTR_FLAG;
        cf.can_dlc = frame.len;
        std::memcpy(cf.data, frame.data, cf.can_dlc);
        written = ::write(m_fd, &cf, sizeof(cf));
        if (written != static_cast<ssize_t>(sizeof(cf))) {
            doLog(m_log, dashcam::log::LogLevel::ERROR,
                  "CanBus::send: write failed: %s", ::strerror(errno));
            return false;
        }
    }
    return true;
}

bool CanBus::start() {
    if (m_fd < 0) {
        doLog(m_log, dashcam::log::LogLevel::ERROR,
              "CanBus::start: socket not open");
        return false;
    }
    if (m_running.load()) return true;
    m_running.store(true);
    m_thread = std::thread(&CanBus::rxLoop, this);
    doLog(m_log, dashcam::log::LogLevel::INFO, "CanBus::start: receive thread running");
    return true;
}

void CanBus::stop() {
    if (!m_running.load()) return;
    m_running.store(false);
    if (m_pipe[1] >= 0) {
        uint8_t b = 1;
        if (::write(m_pipe[1], &b, 1) < 0)
            doLog(m_log, dashcam::log::LogLevel::WARN,
                  "CanBus::stop: write to wake pipe failed: %s", ::strerror(errno));
    }
    if (m_thread.joinable()) m_thread.join();
    doLog(m_log, dashcam::log::LogLevel::INFO, "CanBus::stop: receive thread stopped");
}

void CanBus::close() {
    stop();
    if (m_fd     >= 0) { ::close(m_fd);     m_fd     = -1; }
    if (m_pipe[0] >= 0) { ::close(m_pipe[0]); m_pipe[0] = -1; }
    if (m_pipe[1] >= 0) { ::close(m_pipe[1]); m_pipe[1] = -1; }
}

bool CanBus::isOpen()    const { return m_fd >= 0; }
bool CanBus::isRunning() const { return m_running.load(); }

// ─── rxLoop ───────────────────────────────────────────────────────────────────

void CanBus::rxLoop() {
    // poll() (not select()) so this keeps working when the CAN fd lands at or
    // above FD_SETSIZE (1024) — plausible on a box running several GStreamer
    // camera pipelines, where select() would smash the stack.
    while (m_running.load()) {
        struct pollfd pfds[2];
        pfds[0].fd = m_fd;      pfds[0].events = POLLIN; pfds[0].revents = 0;
        pfds[1].fd = m_pipe[0]; pfds[1].events = POLLIN; pfds[1].revents = 0;

        int ret = ::poll(pfds, 2, -1);
        if (ret < 0) {
            if (errno == EINTR) continue;
            ErrorCallback localErrCb;
            { std::lock_guard<std::mutex> lk(m_cbMtx); localErrCb = m_errCb; }
            if (localErrCb)
                localErrCb(std::string("CanBus: poll() error: ") + ::strerror(errno));
            break;
        }

        // Wake pipe: stop() was called.
        if (pfds[1].revents & (POLLIN | POLLHUP | POLLERR)) {
            uint8_t buf[16];
            (void)::read(m_pipe[0], buf, sizeof(buf));
            break;
        }

        // A persistent error on the socket would otherwise spin the loop.
        if (pfds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) {
            ErrorCallback localErrCb;
            { std::lock_guard<std::mutex> lk(m_cbMtx); localErrCb = m_errCb; }
            if (localErrCb) localErrCb("CanBus: socket poll error (POLLERR/POLLHUP)");
            break;
        }

        if (!(pfds[0].revents & POLLIN)) continue;

        // Read into a canfd_frame-sized buffer; the return value disambiguates.
        struct canfd_frame cf{};
        ssize_t n = ::read(m_fd, &cf, sizeof(cf));

        CanFrame frame;
        if (m_fdEnabled && n == static_cast<ssize_t>(CANFD_MTU)) {
            // CAN FD frame.
            bool isErr = (cf.can_id & CAN_ERR_FLAG) != 0;
            frame.fdFrame  = true;
            frame.id       = cf.can_id & CAN_EFF_MASK;
            frame.extended = (cf.can_id & CAN_EFF_FLAG) != 0;
            frame.brs      = (cf.flags  & CANFD_BRS)    != 0;
            frame.esi      = (cf.flags  & CANFD_ESI)    != 0;
            frame.len      = cf.len;
            std::memcpy(frame.data, cf.data, cf.len);
            if (isErr) {
                std::ostringstream oss;
                oss << "CAN error frame: 0x" << std::hex << cf.can_id;
                ErrorCallback localErrCb;
                { std::lock_guard<std::mutex> lk(m_cbMtx); localErrCb = m_errCb; }
                if (localErrCb) localErrCb(oss.str());
                continue;
            }
        } else if (n == static_cast<ssize_t>(CAN_MTU)) {
            // Classic CAN frame (also returned when FD is enabled but a classic frame arrives).
            struct can_frame* cp = reinterpret_cast<struct can_frame*>(&cf);
            bool isErr = (cp->can_id & CAN_ERR_FLAG) != 0;
            frame.fdFrame  = false;
            frame.id       = cp->can_id & CAN_EFF_MASK;
            frame.extended = (cp->can_id & CAN_EFF_FLAG) != 0;
            frame.rtr      = (cp->can_id & CAN_RTR_FLAG) != 0;
            frame.len      = cp->can_dlc;
            std::memcpy(frame.data, cp->data, cp->can_dlc);
            if (isErr) {
                std::ostringstream oss;
                oss << "CAN error frame: 0x" << std::hex << cp->can_id;
                ErrorCallback localErrCb;
                { std::lock_guard<std::mutex> lk(m_cbMtx); localErrCb = m_errCb; }
                if (localErrCb) localErrCb(oss.str());
                continue;
            }
        } else {
            // Short read or error; skip.
            continue;
        }

        ReceiveCallback localRxCb;
        { std::lock_guard<std::mutex> lk(m_cbMtx); localRxCb = m_rxCb; }
        if (localRxCb) localRxCb(frame);
    }
}

} // namespace dashcam::can
