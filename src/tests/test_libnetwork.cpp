// libnetwork smoke test:
//   [1] UDP datagram loopback  (sendTo / recvFrom on 127.0.0.1)
//   [2] TCP echo loopback      (TcpServer::accept in a thread + TcpSocket client)
//   [3] SNTP internet-time query (best-effort — WARN-only if offline)
//   [4] MJPEG/HTTP stream server (multi-viewer fan-out + reap)
//   [5] Raw-TCP stream passthrough
//   [6] RtpSession branch description / SDP / viewer hint / named sink
//   [7] WiFi connectivity (non-destructive: targets the current SSID)
//   [8] ControlServer: HMAC known-answer, no-auth + authenticated handshake,
//       unauthenticated telemetry denial, wrong-key rejection, IP allowlist
//
// All sections but [3] need no network and must pass; [3] needs internet and
// only reports.  The library is observation-only and never mutates the clock.
//
// Ctrl-C exits cleanly.  Returns 0 if the loopback tests pass, 1 otherwise.

#include "libnetwork.h"
#include "liblog.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <sstream>
#include <string>
#include <thread>

namespace fs  = std::filesystem;
namespace net = dashcam::network;
using LvL = dashcam::log::LogLevel;

static std::atomic<bool> g_stop{false};
static void onSignal(int) { g_stop.store(true); }

// Read from a connected socket, accumulating bytes until the peer goes idle for
// one recv timeout (after some data arrived) or the total window elapses.
static std::string drain(net::TcpSocket& s, int totalMs) {
    std::string acc;
    char buf[4096];
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(totalMs);
    while (std::chrono::steady_clock::now() < end) {
        size_t got = 0;
        net::IoStatus st = s.recv(buf, sizeof(buf), 150, got);
        if (st == net::IoStatus::Ok)        acc.append(buf, got);
        else if (st == net::IoStatus::Timeout) { if (!acc.empty()) break; }
        else                                 break;  // Closed / Error
    }
    return acc;
}

int main(int argc, char* argv[]) {
    // Optional arg: NTP server override (default pool.ntp.org).
    const std::string ntpServer = (argc > 1) ? argv[1] : "pool.ntp.org";

    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    fs::create_directories("./logs");
    dashcam::log::init();  // build-local logs: <exe_dir>/logs
    auto log = dashcam::log::getCallback();

    log(LvL::INFO, "=== libnetwork test ===");
    int failures = 0;

    // ── [1] UDP loopback ──────────────────────────────────────────────────────
    log(LvL::INFO, "--------------------------------------------");
    log(LvL::INFO, "[1] UDP datagram loopback");
    log(LvL::INFO, "--------------------------------------------");
    {
        net::UdpSocket rx, tx;
        if (!rx.open(0, log) || !tx.open(0, log)) {
            log(LvL::ERROR, "  open() failed");
            ++failures;
        } else {
            uint16_t rxPort = rx.localPort();
            log(LvL::INFO, "  receiver bound to 127.0.0.1:" + std::to_string(rxPort));

            const std::string msg = "hello-udp";
            bool sent = tx.sendTo("127.0.0.1", rxPort, msg.data(), msg.size());

            char        buf[64] = {0};
            std::string srcHost;
            uint16_t    srcPort = 0;
            long n = rx.recvFrom(buf, sizeof(buf), 1000, &srcHost, &srcPort);

            bool ok = sent && n == static_cast<long>(msg.size()) &&
                      std::memcmp(buf, msg.data(), msg.size()) == 0;
            std::ostringstream o;
            o << "  RX " << n << " byte(s) from " << srcHost << ":" << srcPort
              << " = \"" << std::string(buf, n > 0 ? n : 0) << "\"  " << (ok ? "OK" : "FAIL");
            log(ok ? LvL::INFO : LvL::ERROR, o.str());
            if (!ok) ++failures;
        }
    }

    // ── [2] TCP echo loopback ─────────────────────────────────────────────────
    log(LvL::INFO, "--------------------------------------------");
    log(LvL::INFO, "[2] TCP echo loopback");
    log(LvL::INFO, "--------------------------------------------");
    {
        net::TcpServer srv;
        if (!srv.listen(0, 4, log)) {
            log(LvL::ERROR, "  listen() failed");
            ++failures;
        } else {
            uint16_t port = srv.port();
            log(LvL::INFO, "  server listening on 127.0.0.1:" + std::to_string(port));

            // Server thread: accept one connection and echo one message back.
            std::atomic<bool> echoed{false};
            std::thread server([&] {
                net::IoStatus st;
                net::TcpSocket c = srv.accept(2000, &st);
                if (!c.isOpen()) {
                    log(LvL::ERROR, "  server accept() did not get a connection");
                    return;
                }
                log(LvL::INFO, "  server accepted " + c.peer());
                char   b[256] = {0};
                size_t got = 0;
                if (c.recv(b, sizeof(b), 1000, got) == net::IoStatus::Ok && got > 0) {
                    if (c.sendAll(b, got)) echoed.store(true);
                }
            });

            net::TcpSocket cli;
            bool ok = false;
            if (!cli.connect("127.0.0.1", port, 1000, log)) {
                log(LvL::ERROR, "  client connect() failed");
            } else {
                const std::string msg = "hello-tcp";
                char   rb[256] = {0};
                size_t got = 0;
                if (cli.sendAll(msg.data(), msg.size()) &&
                    cli.recv(rb, sizeof(rb), 1000, got) == net::IoStatus::Ok) {
                    ok = got == msg.size() && std::memcmp(rb, msg.data(), msg.size()) == 0;
                    log(LvL::INFO, "  echo returned \"" + std::string(rb, got) + "\"  " +
                                   (ok ? "OK" : "FAIL"));
                }
            }

            server.join();
            if (!ok || !echoed.load()) ++failures;

            // Exercise the wake-on-close path: a fresh accept() must return Closed
            // promptly when close() fires from another thread.
            net::TcpServer srv2;
            if (srv2.listen(0, 4, log)) {
                std::atomic<net::IoStatus> woke{net::IoStatus::Ok};
                std::thread waiter([&] {
                    net::IoStatus st;
                    (void)srv2.accept(5000, &st);   // would block ~5 s without the wake
                    woke.store(st);
                });
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                auto t0 = std::chrono::steady_clock::now();
                srv2.close();
                waiter.join();
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::steady_clock::now() - t0).count();
                bool wokeOk = woke.load() == net::IoStatus::Closed && ms < 1000;
                log(wokeOk ? LvL::INFO : LvL::ERROR,
                    "  accept() woke on close() in " + std::to_string(ms) + " ms  " +
                    (wokeOk ? "OK" : "FAIL"));
                if (!wokeOk) ++failures;
            }
        }
    }

    // ── [3] SNTP internet-time query (best-effort) ────────────────────────────
    log(LvL::INFO, "--------------------------------------------");
    log(LvL::INFO, "[3] SNTP internet time  (server: " + ntpServer + ")");
    log(LvL::INFO, "--------------------------------------------");
    {
        net::TimeResult t = net::queryTime(ntpServer, 123, 3000, log);
        if (!t.valid) {
            log(LvL::WARN, "  no reply (offline or NTP blocked) — skipping, not a failure");
        } else {
            std::time_t secs = static_cast<std::time_t>(t.unixSeconds);
            char        when[64] = {0};
            std::tm     tmv;
            gmtime_r(&secs, &tmv);
            std::strftime(when, sizeof(when), "%Y-%m-%d %H:%M:%S UTC", &tmv);

            std::ostringstream o;
            o << "  server time : " << when << "\n"
              << "                local offset: " << t.offsetSeconds << " s"
              << "  (round-trip " << (t.roundTripSeconds * 1000.0) << " ms)";
            log(LvL::INFO, o.str());
            log(LvL::INFO, "  SNTP observation timestamp: unix "
                           + std::to_string(t.unixSeconds) + " (not applied)");
        }
    }

    // ── [4] MJPEG/HTTP stream server: multi-viewer fan-out + reap ─────────────
    log(LvL::INFO, "--------------------------------------------");
    log(LvL::INFO, "[4] MJPEG/HTTP stream server");
    log(LvL::INFO, "--------------------------------------------");
    {
        net::MediaStreamServer srv;
        net::StreamServerConfig sc;
        sc.port = 0;                       // ephemeral
        sc.wire = net::StreamWire::MjpegHttp;
        sc.boundary = "dashcamframe";
        if (!srv.start(sc, log)) {
            log(LvL::ERROR, "  start() failed");
            ++failures;
        } else {
            uint16_t port = srv.port();
            log(LvL::INFO, "  server on 127.0.0.1:" + std::to_string(port));

            const std::string jpegA("\xFF\xD8" "AAA" "\xFF\xD9", 7);
            const std::string jpegB("\xFF\xD8" "BB"  "\xFF\xD9", 6);

            net::TcpSocket v1, v2;
            bool c1 = v1.connect("127.0.0.1", port, 1000, log);
            bool c2 = v2.connect("127.0.0.1", port, 1000, log);
            std::this_thread::sleep_for(std::chrono::milliseconds(150));  // let accept register both

            bool two = c1 && c2 && srv.clientCount() == 2;
            log(two ? LvL::INFO : LvL::ERROR,
                "  2 viewers connected (clientCount=" + std::to_string(srv.clientCount()) + ")  " +
                (two ? "OK" : "FAIL"));
            if (!two) ++failures;

            srv.pushFrame(jpegA.data(), jpegA.size());
            srv.pushFrame(jpegB.data(), jpegB.size());

            std::string s1 = drain(v1, 1000);
            std::string s2 = drain(v2, 1000);

            auto ok = [&](const std::string& s) {
                return s.find("HTTP/1.0 200 OK") != std::string::npos &&
                       s.find("multipart/x-mixed-replace; boundary=dashcamframe") != std::string::npos &&
                       s.find("--dashcamframe") != std::string::npos &&
                       s.find("Content-Type: image/jpeg") != std::string::npos &&
                       s.find("Content-Length: 7") != std::string::npos &&
                       s.find("Content-Length: 6") != std::string::npos &&
                       s.find(jpegA) != std::string::npos &&
                       s.find(jpegB) != std::string::npos;
            };
            bool fan = ok(s1) && ok(s2);
            log(fan ? LvL::INFO : LvL::ERROR,
                "  both viewers got the HTTP header + both JPEG parts  " + std::string(fan ? "OK" : "FAIL"));
            if (!fan) ++failures;

            // Disconnect one viewer.  A push-only server detects the drop only
            // when a send fails, and TCP lets the first send after a clean close
            // succeed — so push a short burst (a real 30 fps stream detects within
            // a frame or two).  clientCount() stops counting it once its writer
            // marks itself done; the physical join happens on the next accept tick.
            v2.close();
            for (int i = 0; i < 6; ++i) {
                srv.pushFrame(jpegA.data(), jpegA.size());
                std::this_thread::sleep_for(std::chrono::milliseconds(25));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            bool reaped = srv.clientCount() == 1;
            log(reaped ? LvL::INFO : LvL::ERROR,
                "  disconnected viewer reaped (clientCount=" + std::to_string(srv.clientCount()) + ")  " +
                (reaped ? "OK" : "FAIL"));
            if (!reaped) ++failures;

            // The survivor still receives new frames.
            srv.pushFrame(jpegB.data(), jpegB.size());
            std::string more = drain(v1, 1000);
            bool alive = more.find(jpegB) != std::string::npos;
            log(alive ? LvL::INFO : LvL::ERROR,
                "  surviving viewer still receives frames  " + std::string(alive ? "OK" : "FAIL"));
            if (!alive) ++failures;

            srv.stop();
        }
    }

    // ── [5] Raw-TCP stream passthrough (e.g. H.264 elementary stream) ─────────
    log(LvL::INFO, "--------------------------------------------");
    log(LvL::INFO, "[5] Raw-TCP stream passthrough");
    log(LvL::INFO, "--------------------------------------------");
    {
        net::MediaStreamServer srv;
        net::StreamServerConfig sc;
        sc.port = 0;
        sc.wire = net::StreamWire::RawTcp;
        if (!srv.start(sc, log)) {
            log(LvL::ERROR, "  start() failed");
            ++failures;
        } else {
            net::TcpSocket v;
            bool c = v.connect("127.0.0.1", srv.port(), 1000, log);
            std::this_thread::sleep_for(std::chrono::milliseconds(150));
            srv.pushFrame("ABC", 3);
            srv.pushFrame("DEF", 3);
            std::string s = drain(v, 1000);
            bool ok = c && s == "ABCDEF";
            log(ok ? LvL::INFO : LvL::ERROR,
                "  raw passthrough received \"" + s + "\" (expected \"ABCDEF\")  " +
                (ok ? "OK" : "FAIL"));
            if (!ok) ++failures;
            srv.stop();
        }
    }

    // ── [6] RtpSession: branch description / SDP / viewer hint ────────────────
    log(LvL::INFO, "--------------------------------------------");
    log(LvL::INFO, "[6] RtpSession (inference-camera RTP control)");
    log(LvL::INFO, "--------------------------------------------");
    {
        net::RtpStreamConfig rc;
        rc.host = "127.0.0.1";
        rc.port = 5600;
        rc.bitrateKbps = 6000;
        rc.keyIntSec = 2;
        net::RtpSession rtp(rc);

        const std::string usb = rtp.branchDescription(/*nvmm=*/false, 30.0f);
        const std::string csi = rtp.branchDescription(/*nvmm=*/true,  30.0f);
        const std::string sdp = rtp.sdp();

        auto has = [](const std::string& s, const char* sub) {
            return s.find(sub) != std::string::npos;
        };
        bool ok =
            has(usb, "videoconvert") && !has(usb, "nvvidconv") &&
            has(csi, "nvvidconv") &&
            has(usb, "x264enc") && has(usb, "bitrate=6000") &&
            has(usb, "key-int-max=60") &&                       // keyIntSec(2) * 30 fps
            has(usb, "rtph264pay config-interval=1 pt=96") &&
            has(usb, "udpsink host=\"127.0.0.1\" port=5600") &&
            has(sdp, "m=video 5600 RTP/AVP 96") &&
            has(sdp, "a=rtpmap:96 H264/90000");
        log(ok ? LvL::INFO : LvL::ERROR,
            std::string("  branch description / SDP well-formed  ") + (ok ? "OK" : "FAIL"));
        if (!ok) ++failures;

        // Named-sink variant (for runtime RTP re-pointing via ControlServer).
        const std::string named = rtp.branchDescription(/*nvmm=*/false, 30.0f, "lane-rtpsink");
        const bool okName = has(named, "udpsink name=lane-rtpsink host=\"127.0.0.1\" port=5600");
        log(okName ? LvL::INFO : LvL::ERROR,
            std::string("  named udpsink for live re-point         ") + (okName ? "OK" : "FAIL"));
        if (!okName) ++failures;

        // Emit the USB (videoconvert) description on a marker line so an external
        // harness can feed it through real GStreamer and confirm RTP actually flows.
        log(LvL::INFO, "  RTP_USB_DESC: " + usb);
        log(LvL::INFO, "  viewer: " + rtp.viewerHint());
    }

    // ── [7] WiFi connectivity (non-destructive: targets the current SSID) ─────
    log(LvL::INFO, "--------------------------------------------");
    log(LvL::INFO, "[7] WiFi connectivity");
    log(LvL::INFO, "--------------------------------------------");
    {
        // Read-only detection.  Non-empty on a host with nmcli + active WiFi;
        // empty where nmcli/NetworkManager is unavailable (e.g. this container).
        const std::string ssid = net::currentWifiSsid(log);
        log(LvL::INFO, "  currentWifiSsid() = " +
                       (ssid.empty() ? "(none / nmcli unavailable)" : "\"" + ssid + "\""));

        // connectWifi() with ssid="" targets the CURRENT SSID.  If already
        // connected it returns Online without touching the link; otherwise it
        // reports Offline gracefully.  Either way it must not crash, and Online
        // implies a non-empty SSID.
        net::WifiConnectConfig wc;   // enabled, ssid="", 20 s, no internet requirement
        net::WifiStatus ws = net::connectWifi(wc, log);
        const bool online = ws.state == net::ConnectivityState::Online;
        const bool coherent = !online || !ws.ssid.empty();   // Online ⟹ ssid set
        log(coherent ? LvL::INFO : LvL::ERROR,
            std::string("  connectWifi() -> ") + (online ? "ONLINE" : "OFFLINE") +
            " (" + ws.detail + ")  " + (coherent ? "OK" : "FAIL"));
        if (!coherent) ++failures;
    }

    // ── [8] ControlServer: HMAC + auth handshake + dispatch + telemetry ───────
    log(LvL::INFO, "--------------------------------------------");
    log(LvL::INFO, "[8] ControlServer (auth + remote control + telemetry)");
    log(LvL::INFO, "--------------------------------------------");
    {
        auto has = [](const std::string& s, const char* sub) {
            return s.find(sub) != std::string::npos;
        };
        // Pull the nonce out of an "AUTH-CHALLENGE <nonce>" line.
        auto nonceOf = [](const std::string& s) -> std::string {
            const std::string tag = "AUTH-CHALLENGE ";
            const size_t p = s.find(tag);
            if (p == std::string::npos) return "";
            const size_t b = p + tag.size();
            const size_t e = s.find_first_of("\r\n", b);
            return s.substr(b, e == std::string::npos ? std::string::npos : e - b);
        };
        // Handler echoes the parsed command back so the client can verify the
        // server tokenised verb/args/clientIp correctly — no shared state, no race.
        net::ControlHandler handler = [](const net::ControlCommand& cmd) -> std::string {
            std::ostringstream os;
            os << "GOT verb=" << cmd.verb << " nargs=" << cmd.args.size()
               << " ip=" << cmd.clientIp;
            if (!cmd.args.empty()) os << " a0=" << cmd.args[0];
            return os.str();
        };

        // (a) HMAC-SHA256 known-answer vectors (RFC 4231 test cases 1 & 2).  A
        //     wrong SHA-256/HMAC here would silently break every auth handshake.
        {
            const std::string k1(20, '\x0b');
            const std::string m1 = net::detail::hmacSha256Hex(k1, "Hi There");
            const std::string m2 = net::detail::hmacSha256Hex(
                "Jefe", "what do ya want for nothing?");
            const bool ok =
                m1 == "b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7" &&
                m2 == "5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843";
            log(ok ? LvL::INFO : LvL::ERROR,
                std::string("  HMAC-SHA256 RFC-4231 known-answer            ") +
                (ok ? "OK" : "FAIL"));
            if (!ok) { ++failures; log(LvL::ERROR, "    m1=" + m1 + " m2=" + m2); }
        }

        // (b) No-auth channel (empty token): immediate greeting, dispatch, telemetry.
        {
            net::ControlServer cs;
            net::ControlServerConfig cc;
            cc.port = 0;                 // OS-assigned
            cc.maxClients = 2;
            const bool started = cs.start(cc, handler, log);
            const uint16_t port = cs.port();
            bool ok = started;
            if (started) {
                net::TcpSocket cli;
                const bool conn = cli.connect("127.0.0.1", port, 1000, log);
                const std::string greet = drain(cli, 400);
                const std::string c1 = "SET foo bar\n";
                cli.sendAll(c1.data(), c1.size());
                const std::string r1 = drain(cli, 600);
                cs.pushTelemetry("{\"hello\":1}");
                const std::string r2 = drain(cli, 600);
                ok = conn && has(greet, "control ready") &&
                     has(r1, "verb=SET") && has(r1, "nargs=2") &&
                     has(r1, "ip=127.0.0.1") && has(r1, "a0=foo") &&
                     has(r2, "hello") && cs.clientCount() == 1;
                cli.close();
            }
            log(ok ? LvL::INFO : LvL::ERROR,
                std::string("  no-auth connect / dispatch / telemetry       ") +
                (ok ? "OK" : "FAIL"));
            if (!ok) ++failures;
            cs.stop();
        }

        // (c) Authenticated channel: correct HMAC unlocks dispatch + telemetry; a
        //     wrong key is rejected; a never-authenticated client is denied both
        //     command dispatch and telemetry.
        {
            const std::string token = "unit-test-psk";
            net::ControlServer cs;
            net::ControlServerConfig cc;
            cc.port        = 0;
            cc.maxClients  = 4;
            cc.bindAddress = "127.0.0.1";      // host-local bind
            cc.authToken   = token;
            cc.allowIps    = { "127.0.0.1" };  // loopback is permitted
            const bool started = cs.start(cc, handler, log);
            const uint16_t port = cs.port();
            if (!started) { ++failures; log(LvL::ERROR, "  auth start() FAILED"); }

            if (started) {
                // Good client: full handshake, then a command + telemetry.
                net::TcpSocket good;
                const bool conn = good.connect("127.0.0.1", port, 1000, log);
                const std::string chal = drain(good, 500);
                const std::string mac  = net::detail::hmacSha256Hex(token, nonceOf(chal));
                const std::string am   = "AUTH " + mac + "\n";
                good.sendAll(am.data(), am.size());
                const std::string authReply = drain(good, 600);
                const std::string c1 = "STATUS now\n";
                good.sendAll(c1.data(), c1.size());
                const std::string r1 = drain(good, 600);
                cs.pushTelemetry("{\"hello\":1}");
                const std::string r2 = drain(good, 600);
                const bool okGood =
                    conn && has(chal, "AUTH-CHALLENGE") && !nonceOf(chal).empty() &&
                    has(authReply, "AUTH-OK") && has(authReply, "control ready") &&
                    has(r1, "verb=STATUS") && has(r2, "hello");
                log(okGood ? LvL::INFO : LvL::ERROR,
                    std::string("  auth: correct HMAC -> dispatch + telemetry   ") +
                    (okGood ? "OK" : "FAIL"));
                if (!okGood) {
                    ++failures;
                    log(LvL::ERROR, "    chal=\"" + chal + "\" auth=\"" + authReply +
                                    "\" r1=\"" + r1 + "\" r2=\"" + r2 + "\"");
                }

                // Wrong-key client: AUTH-FAIL and no telemetry ever delivered.
                net::TcpSocket bad;
                bad.connect("127.0.0.1", port, 1000, log);
                const std::string chalB = drain(bad, 500);
                const std::string macB  = net::detail::hmacSha256Hex("wrong-key", nonceOf(chalB));
                const std::string amB   = "AUTH " + macB + "\n";
                bad.sendAll(amB.data(), amB.size());
                const std::string replyB = drain(bad, 600);
                cs.pushTelemetry("{\"secret\":1}");
                const std::string afterB = drain(bad, 400);
                const bool okBad = has(replyB, "AUTH-FAIL") &&
                                   afterB.find("secret") == std::string::npos;
                log(okBad ? LvL::INFO : LvL::ERROR,
                    std::string("  auth: wrong HMAC -> AUTH-FAIL, no telemetry  ") +
                    (okBad ? "OK" : "FAIL"));
                if (!okBad) ++failures;

                // Unauthenticated client: a command is refused and no telemetry
                // leaks while it stays unauthenticated.
                net::TcpSocket un;
                un.connect("127.0.0.1", port, 1000, log);
                const std::string chalU = drain(un, 500);
                const std::string cu = "STATUS\n";
                un.sendAll(cu.data(), cu.size());
                const std::string rU = drain(un, 500);
                cs.pushTelemetry("{\"secret\":2}");
                const std::string afterU = drain(un, 400);
                const bool okUn = has(chalU, "AUTH-CHALLENGE") &&
                                  has(rU, "authenticate first") &&
                                  afterU.find("secret") == std::string::npos;
                log(okUn ? LvL::INFO : LvL::ERROR,
                    std::string("  unauth: command refused, telemetry denied    ") +
                    (okUn ? "OK" : "FAIL"));
                if (!okUn) ++failures;

                good.close(); bad.close(); un.close();
            }
            cs.stop();
        }

        // (d) IP allowlist: a source not on the list is refused before it becomes
        //     a client (loopback connects, but the allowlist names a different IP).
        {
            net::ControlServer cs;
            net::ControlServerConfig cc;
            cc.port        = 0;
            cc.bindAddress = "127.0.0.1";
            cc.allowIps    = { "10.11.12.13" };   // deliberately NOT loopback
            const bool started = cs.start(cc, handler, log);
            bool ok = started;
            if (started) {
                net::TcpSocket cli;
                cli.connect("127.0.0.1", cs.port(), 1000, log);
                const std::string got = drain(cli, 500);   // server closes us immediately
                std::this_thread::sleep_for(std::chrono::milliseconds(150));
                ok = got.empty() && cs.clientCount() == 0;
                cli.close();
            }
            log(ok ? LvL::INFO : LvL::ERROR,
                std::string("  allowlist refuses a non-listed source IP      ") +
                (ok ? "OK" : "FAIL"));
            if (!ok) ++failures;
            cs.stop();
        }
    }

    // ── Summary ───────────────────────────────────────────────────────────────
    log(LvL::INFO, "--------------------------------------------");
    if (failures == 0)
        log(LvL::INFO, "=== All loopback tests passed ===");
    else
        log(LvL::ERROR, "=== " + std::to_string(failures) + " loopback test(s) FAILED ===");

    dashcam::log::shutdown();
    return failures == 0 ? 0 : 1;
}
