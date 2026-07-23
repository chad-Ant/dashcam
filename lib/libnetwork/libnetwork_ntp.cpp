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

#include <arpa/inet.h>   // htonl, ntohl
#include <ctime>         // clock_gettime, timespec

#include <algorithm>
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

void secondsToNtp(double value, uint32_t& sec_be, uint32_t& frac_be) {
    const double whole = std::floor(value);
    const double fraction = std::max(0.0, std::min(1.0 - 1.0 / TWO32,
                                                   value - whole));
    sec_be = htonl(static_cast<uint32_t>(whole));
    frac_be = htonl(static_cast<uint32_t>(fraction * TWO32));
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

    // A server must echo this nonce-like transmit timestamp in its originate
    // field.  This binds the response to this request and rejects unrelated or
    // stale UDP datagrams (plain SNTP is still not cryptographically
    // authenticated, so callers must not use it as authority to step a clock).
    secondsToNtp(nowNtpSeconds(), req.txTs_s, req.txTs_f);
    const double t1 = ntpToSeconds(req.txTs_s, req.txTs_f);
    std::string destinationIp;
    if (!sock.sendTo(server, port, &req, sizeof(req), &destinationIp)) return r;

    NtpPacket resp;
    std::memset(&resp, 0, sizeof(resp));
    std::string sourceIp;
    uint16_t sourcePort = 0;
    long n = sock.recvFrom(&resp, sizeof(resp), timeoutMs, &sourceIp, &sourcePort);
    double t4 = nowNtpSeconds();

    if (n == 0) { say(log, LvL::WARN,  "NTP query to " + server + " timed out"); return r; }
    if (n < 0)  { return r; }         // recvFrom already logged
    if (static_cast<size_t>(n) < sizeof(resp)) {
        say(log, LvL::WARN, "NTP reply from " + server + " too short (" + std::to_string(n) + " bytes)");
        return r;
    }
    if (sourceIp != destinationIp || sourcePort != port) {
        say(log, LvL::WARN, "NTP reply source mismatch (expected " +
                            destinationIp + ":" + std::to_string(port) +
                            ", got " + sourceIp + ":" +
                            std::to_string(sourcePort) + ")");
        return r;
    }

    const int leap = (resp.li_vn_mode >> 6) & 0x03;
    const int version = (resp.li_vn_mode >> 3) & 0x07;
    const int mode = resp.li_vn_mode & 0x07;
    if (mode != 4 || (version != 3 && version != 4) || leap == 3 ||
        resp.stratum == 0 || resp.stratum > 15) {
        say(log, LvL::WARN, "NTP reply from " + server + " not a valid server response (mode="
                            + std::to_string(mode) + " version=" +
                            std::to_string(version) + " leap=" +
                            std::to_string(leap) + " stratum=" +
                            std::to_string(resp.stratum) + ")");
        return r;
    }
    if (resp.origTs_s != req.txTs_s || resp.origTs_f != req.txTs_f) {
        say(log, LvL::WARN, "NTP reply from " + server +
                           " did not echo this request's transmit timestamp");
        return r;
    }

    double t2 = ntpToSeconds(resp.rxTs_s, resp.rxTs_f);
    double t3 = ntpToSeconds(resp.txTs_s, resp.txTs_f);
    if (t2 <= 0.0 || t3 <= 0.0 || t3 < t2) {
        say(log, LvL::WARN, "NTP reply from " + server +
                           " has invalid receive/transmit timestamps");
        return r;
    }

    r.offsetSeconds    = ((t2 - t1) + (t3 - t4)) / 2.0;
    r.roundTripSeconds = (t4 - t1) - (t3 - t2);
    const double maxRtt = std::max(1.0, timeoutMs / 1000.0 + 0.25);
    if (!std::isfinite(r.offsetSeconds) || !std::isfinite(r.roundTripSeconds) ||
        r.roundTripSeconds < -0.010 || r.roundTripSeconds > maxRtt) {
        say(log, LvL::WARN, "NTP reply from " + server +
                           " produced an implausible round-trip time");
        return TimeResult{};
    }

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

} // namespace dashcam::network
