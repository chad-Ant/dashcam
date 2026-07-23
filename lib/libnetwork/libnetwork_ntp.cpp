/**
 * @file libnetwork_ntp.cpp
 * @brief SNTP (RFC 4330) client built on UdpSocket.
 *
 * Sends a 48-byte mode-3 client request, parses the mode-4 server reply, and
 * computes the clock offset and round-trip time from the four timestamps:
 *
 *   t1 = originate   (local time the request was sent)
 *   t2 = receive     (server time the request arrived)      [reply bytes 32..39]
 *   t3 = transmit    (server time the reply was sent)       [reply bytes 40..47]
 *   t4 = destination (local time the reply was received)
 *
 *   offset     = ((t2 - t1) + (t3 - t4)) / 2
 *   round-trip = (t4 - t1) - (t3 - t2)
 *
 * NTP timestamps count seconds since 1900-01-01; Unix time subtracts the
 * 2208988800 s epoch delta.  Valid until the 2036 NTP era rollover (the 32-bit
 * seconds field wraps then), which is far beyond this project's horizon.
 */

#include "libnetwork.h"

#include <arpa/inet.h>   // ntohl
#include <ctime>         // clock_gettime, clock_settime, timespec

#include <cerrno>
#include <cmath>
#include <cstring>

namespace dashcam::network {

using LvL = dashcam::log::LogLevel;

namespace {

void say(const dashcam::log::LogCallback& log, LvL lvl, const std::string& msg) {
    if (log) log(lvl, msg);
}

constexpr uint64_t NTP_UNIX_DELTA = 2208988800ULL;  ///< Seconds between 1900 and 1970 epochs.
constexpr double   TWO32          = 4294967296.0;    ///< 2^32, for the fractional field.

/// 48-byte NTP/SNTP packet.  All multi-byte fields are network byte order.
struct NtpPacket {
    uint8_t  li_vn_mode;      ///< Leap (2) | Version (3) | Mode (3).
    uint8_t  stratum;
    uint8_t  poll;
    uint8_t  precision;
    uint32_t rootDelay;
    uint32_t rootDispersion;
    uint32_t refId;
    uint32_t refTs_s,  refTs_f;   ///< Reference timestamp.
    uint32_t origTs_s, origTs_f;  ///< Originate (t1, echoed by server).
    uint32_t rxTs_s,   rxTs_f;    ///< Receive   (t2).
    uint32_t txTs_s,   txTs_f;    ///< Transmit  (t3).
};
static_assert(sizeof(NtpPacket) == 48, "NTP packet must be exactly 48 bytes");

/// Current local real time expressed as NTP seconds (double).
double nowNtpSeconds() {
    struct timespec ts;
    ::clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(NTP_UNIX_DELTA)
         + static_cast<double>(ts.tv_nsec) / 1e9;
}

/// Convert a network-order NTP timestamp pair to seconds (double).
double ntpToSeconds(uint32_t sec_be, uint32_t frac_be) {
    return static_cast<double>(ntohl(sec_be))
         + static_cast<double>(ntohl(frac_be)) / TWO32;
}

} // namespace

TimeResult queryTime(const std::string& server, uint16_t port, int timeoutMs,
                     const dashcam::log::LogCallback& log) {
    TimeResult r;
    r.server = server;

    UdpSocket sock;
    if (!sock.open(0, log)) return r;  // ephemeral local port

    NtpPacket req;
    std::memset(&req, 0, sizeof(req));
    req.li_vn_mode = 0x1B;  // LI = 0, Version = 3, Mode = 3 (client)

    double t1 = nowNtpSeconds();
    if (!sock.sendTo(server, port, &req, sizeof(req))) return r;

    NtpPacket resp;
    std::memset(&resp, 0, sizeof(resp));
    long n = sock.recvFrom(&resp, sizeof(resp), timeoutMs);
    double t4 = nowNtpSeconds();

    if (n == 0) { say(log, LvL::WARN,  "NTP query to " + server + " timed out"); return r; }
    if (n < 0)  { return r; }         // recvFrom already logged
    if (static_cast<size_t>(n) < sizeof(resp)) {
        say(log, LvL::WARN, "NTP reply from " + server + " too short (" + std::to_string(n) + " bytes)");
        return r;
    }

    int mode = resp.li_vn_mode & 0x07;
    if (mode != 4 || resp.stratum == 0) {  // mode 4 = server; stratum 0 = KoD/unsynced
        say(log, LvL::WARN, "NTP reply from " + server + " not a valid server response (mode="
                            + std::to_string(mode) + " stratum=" + std::to_string(resp.stratum) + ")");
        return r;
    }

    double t2 = ntpToSeconds(resp.rxTs_s, resp.rxTs_f);
    double t3 = ntpToSeconds(resp.txTs_s, resp.txTs_f);
    if (t3 <= 0.0) { say(log, LvL::WARN, "NTP reply from " + server + " has no transmit timestamp"); return r; }

    r.offsetSeconds    = ((t2 - t1) + (t3 - t4)) / 2.0;
    r.roundTripSeconds = (t4 - t1) - (t3 - t2);

    // Derive Unix seconds/nanos directly from the integer transmit timestamp so
    // the stored value keeps full second precision (a double seconds value would
    // lose sub-microsecond resolution around 2^31 seconds).
    uint32_t txs = ntohl(resp.txTs_s);
    uint32_t txf = ntohl(resp.txTs_f);
    r.unixSeconds = static_cast<int64_t>(txs) - static_cast<int64_t>(NTP_UNIX_DELTA);
    r.unixNanos   = static_cast<uint32_t>(static_cast<double>(txf) / TWO32 * 1e9);
    r.valid       = true;
    return r;
}

bool stepSystemClock(const TimeResult& t, const dashcam::log::LogCallback& log) {
    if (!t.valid) {
        say(log, LvL::WARN, "stepSystemClock: no valid time to apply");
        return false;
    }
    struct timespec ts;
    ts.tv_sec  = static_cast<time_t>(t.unixSeconds);
    ts.tv_nsec = static_cast<long>(t.unixNanos);

    if (::clock_settime(CLOCK_REALTIME, &ts) != 0) {
        int e = errno;
        say(log, LvL::ERROR, std::string("stepSystemClock: clock_settime failed: ") + std::strerror(e)
                             + (e == EPERM ? " (needs CAP_SYS_TIME / root)" : ""));
        return false;
    }
    say(log, LvL::INFO, "stepSystemClock: system clock set to " + std::to_string(t.unixSeconds)
                        + "." + std::to_string(t.unixNanos) + " (offset "
                        + std::to_string(t.offsetSeconds) + " s)");
    return true;
}

} // namespace dashcam::network
