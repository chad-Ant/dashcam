/**
 * @file libnetwork_control.cpp
 * @brief ControlServer — an authenticated bidirectional TCP control + telemetry channel.
 *
 * One connection carries both directions: the remote operator SENDS
 * newline-delimited command lines (dispatched to the application's
 * ControlHandler — e.g. re-point an RTP stream), and the device PUSHES telemetry
 * lines back to every connected client.  A pure viewer just connects and reads.
 *
 * Structured like MediaStreamServer (libnetwork_stream.cpp): a TcpServer accept
 * loop plus one thread per client.  That client thread interleaves a
 * short-timeout recv (inbound commands) with an outbound-queue flush.
 *
 * Because a command can re-point the road/cabin video feed, the channel is
 * access-controlled entirely inside this server, so the application handler only
 * ever sees commands from authenticated clients:
 *   - Bind address / IP allowlist scope who may connect at all (ControlServerConfig).
 *   - A pre-shared key gates every session: on connect the server sends
 *     `AUTH-CHALLENGE <nonce>`; the client must reply `AUTH <hmac>` where
 *     hmac = HMAC-SHA256(authToken, nonce).  Until it matches (constant-time),
 *     no command is dispatched and no telemetry is delivered.  A wrong answer
 *     disconnects.  An empty authToken disables auth (logged as a warning).
 *
 * Outbound has two queues: a priority `replies` queue (greeting, auth messages,
 * command replies) that is never dropped, and a bounded latest-drop `out` queue
 * for telemetry — so a stalled operator loses only its own telemetry backlog, and
 * an OK/ERR reply to a state-changing command can never be silently discarded.
 */

#include "libnetwork.h"

#include <algorithm>
#include <cctype>
#include <deque>
#include <sstream>
#include <string>
#include <utility>

namespace dashcam::network {

using LvL = dashcam::log::LogLevel;

namespace {

// A client that never reads accumulates replies only up to what a single recv's
// worth of commands can enqueue (its own thread blocks in sendAll otherwise), but
// cap it defensively: past this, the client is abusive and gets disconnected.
constexpr size_t kMaxReplyBacklog = 64;

void say(const dashcam::log::LogCallback& log, LvL lvl, const std::string& msg) {
    if (log) log(lvl, msg);
}

// Split on any ASCII whitespace, dropping empty tokens.
std::vector<std::string> tokenize(const std::string& s) {
    std::vector<std::string> out;
    std::string tok;
    std::istringstream is(s);
    while (is >> tok) out.push_back(tok);
    return out;
}

std::string toUpper(std::string s) {
    for (char& c : s)
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

// "1.2.3.4:5678" -> "1.2.3.4"  (best-effort; returns the input if no colon).
std::string ipOnly(const std::string& peer) {
    const size_t colon = peer.rfind(':');
    return (colon == std::string::npos) ? peer : peer.substr(0, colon);
}

// Constant-time string equality — the auth answer must not be comparable byte by
// byte via response timing.  (Length is not secret: the HMAC hex is fixed at 64.)
bool constantTimeEquals(const std::string& a, const std::string& b) {
    if (a.size() != b.size()) return false;
    unsigned char diff = 0;
    for (size_t i = 0; i < a.size(); ++i)
        diff |= static_cast<unsigned char>(a[i]) ^ static_cast<unsigned char>(b[i]);
    return diff == 0;
}

} // namespace

// ─── per-client state ──────────────────────────────────────────────────────────

struct ControlServer::Client {
    TcpSocket               sock;
    std::thread             thread;
    std::mutex              mtx;
    std::deque<std::string> replies;      ///< Priority outbound (greeting/auth/command replies) — never dropped.
    std::deque<std::string> out;          ///< Telemetry — bounded latest-drop.
    bool                    stop = false;
    std::atomic<bool>       done{false};  ///< Client thread has exited.
    std::string             peer;         ///< "ip:port".
    std::string             ip;           ///< peer IP only.
    std::string             inbuf;        ///< Accumulates a partial inbound line.
    bool                    authed = false; ///< Passed the challenge (or auth disabled).
    std::string             nonce;          ///< Per-client challenge nonce (hex); empty when auth disabled.
};

// ─── ControlServer ─────────────────────────────────────────────────────────────

ControlServer::ControlServer()  = default;
ControlServer::~ControlServer() { stop(); }

bool ControlServer::start(const ControlServerConfig& cfg, ControlHandler handler,
                          const dashcam::log::LogCallback& log) {
    if (running_.load()) return false;
    cfg_     = cfg;
    handler_ = std::move(handler);
    log_     = log;

    if (!server_.listen(cfg_.port, /*backlog=*/cfg_.maxClients + 1, log_, cfg_.bindAddress)) {
        say(log_, LvL::ERROR, "control: listen on port " + std::to_string(cfg_.port) + " failed");
        return false;
    }
    running_.store(true);
    acceptThread_ = std::thread([this] { acceptLoop(); });

    std::string scope = "control: serving on " +
        (cfg_.bindAddress.empty() ? std::string("all interfaces") : cfg_.bindAddress) +
        ":" + std::to_string(server_.port()) +
        " (max " + std::to_string(cfg_.maxClients) + " clients)";
    say(log_, LvL::INFO, scope);
    if (cfg_.authToken.empty())
        say(log_, LvL::WARN, "control: no auth token set — the control channel is "
                             "UNAUTHENTICATED (any permitted client can re-point the video feeds)");
    if (!cfg_.allowIps.empty())
        say(log_, LvL::INFO, "control: IP allowlist active (" +
                             std::to_string(cfg_.allowIps.size()) + " address(es))");
    return true;
}

void ControlServer::acceptLoop() {
    while (running_.load()) {
        IoStatus st;
        TcpSocket s = server_.accept(500, &st);

        std::lock_guard<std::mutex> lk(clientsMtx_);
        reapDoneLocked();  // periodic cleanup of clients that disconnected

        if (st != IoStatus::Ok) {
            if (st == IoStatus::Closed) break;  // stop() closed the listener
            continue;                           // Timeout / transient Error
        }

        // Source-IP allowlist: refuse a peer that is not on the list outright,
        // before it can even see the challenge.
        const std::string peerIp = ipOnly(s.peer());
        if (!cfg_.allowIps.empty() &&
            std::find(cfg_.allowIps.begin(), cfg_.allowIps.end(), peerIp) == cfg_.allowIps.end()) {
            say(log_, LvL::WARN, "control: refusing client " + s.peer() + " (not in allowlist)");
            s.close();
            continue;
        }

        if (static_cast<int>(clients_.size()) >= cfg_.maxClients) {
            say(log_, LvL::WARN, "control: refusing client " + s.peer() +
                                 " (at maxClients=" + std::to_string(cfg_.maxClients) + ")");
            s.close();
            continue;
        }

        auto c   = std::make_unique<Client>();
        c->peer  = s.peer();
        c->ip    = peerIp;
        c->sock  = std::move(s);
        if (cfg_.authToken.empty()) {
            c->authed = true;                       // auth disabled
        } else {
            c->authed = false;
            c->nonce  = detail::randomHex(16);      // 128-bit per-client challenge
        }
        Client* cp = c.get();
        c->thread = std::thread([this, cp] { clientLoop(cp); });
        clients_.push_back(std::move(c));
        say(log_, LvL::INFO, "control: client connected " + cp->peer +
                             " (" + std::to_string(clients_.size()) + " total)");
    }
}

// Send every queued outbound line — priority replies first, then telemetry.
// Returns false if a send failed (the client is gone / was shut down).
bool ControlServer::flushOutbound(Client* c) {
    for (;;) {
        std::string line;
        {
            std::lock_guard<std::mutex> lk(c->mtx);
            if (!c->replies.empty()) {
                line = std::move(c->replies.front());
                c->replies.pop_front();
            } else if (!c->out.empty()) {
                line = std::move(c->out.front());
                c->out.pop_front();
            } else {
                return true;   // nothing pending
            }
        }
        if (!c->sock.sendAll(line.data(), line.size())) return false;
    }
}

void ControlServer::clientLoop(Client* c) {
    // First outbound line: the greeting, or (auth enabled) the nonce challenge.
    {
        std::lock_guard<std::mutex> lk(c->mtx);
        c->replies.push_back(c->authed ? "dashcam control ready — type HELP\n"
                                       : "AUTH-CHALLENGE " + c->nonce + "\n");
    }

    char buf[1024];
    for (;;) {
        // 1) Flush all pending outbound (replies win over telemetry).  A pending
        //    AUTH-FAIL is delivered here before the stop below closes the loop.
        if (!flushOutbound(c)) break;
        {
            std::lock_guard<std::mutex> lk(c->mtx);
            if (c->stop) break;
        }

        // 2) Read inbound commands, bounding telemetry latency to the timeout.
        size_t n = 0;
        const IoStatus st = c->sock.recv(buf, sizeof(buf), 100, n);
        if (st == IoStatus::Closed || st == IoStatus::Error) break;
        if (st != IoStatus::Ok || n == 0) continue;

        c->inbuf.append(buf, n);
        size_t pos;
        while ((pos = c->inbuf.find('\n')) != std::string::npos) {
            std::string line = c->inbuf.substr(0, pos);
            c->inbuf.erase(0, pos + 1);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            dispatchLine(c, line);
        }
        // A client that never sends a newline must not grow inbuf without bound.
        if (c->inbuf.size() > 4096) c->inbuf.clear();
    }
    c->done.store(true);
}

void ControlServer::dispatchLine(Client* c, const std::string& line) {
    const std::vector<std::string> toks = tokenize(line);
    if (toks.empty()) return;
    const std::string verb = toUpper(toks.front());

    // ── authentication gate ──────────────────────────────────────────────────
    bool authedNow;
    { std::lock_guard<std::mutex> lk(c->mtx); authedNow = c->authed; }
    if (!authedNow) {
        std::lock_guard<std::mutex> lk(c->mtx);
        if (verb == "AUTH") {
            const std::string got  = toks.size() >= 2 ? toks[1] : std::string{};
            const std::string want = detail::hmacSha256Hex(cfg_.authToken, c->nonce);
            if (constantTimeEquals(got, want)) {
                c->authed = true;
                c->replies.push_back("AUTH-OK\n");
                c->replies.push_back("dashcam control ready — type HELP\n");
                say(log_, LvL::INFO, "control: client authenticated " + c->peer);
            } else {
                c->replies.push_back("AUTH-FAIL\n");
                c->stop = true;   // delivered by the next flush, then the loop exits
                say(log_, LvL::WARN, "control: auth failed for " + c->peer + " — disconnecting");
            }
        } else {
            c->replies.push_back("ERR authenticate first: AUTH <hmac-sha256(token, nonce)>\n");
        }
        return;
    }

    // ── authenticated: dispatch to the application handler ───────────────────
    ControlCommand cmd;
    cmd.raw      = line;
    cmd.clientIp = c->ip;
    cmd.verb     = verb;
    cmd.args.assign(toks.begin() + 1, toks.end());

    std::string reply = handler_ ? handler_(cmd) : std::string{};
    if (reply.empty()) return;
    if (reply.back() != '\n') reply.push_back('\n');

    std::lock_guard<std::mutex> lk(c->mtx);
    c->replies.push_back(std::move(reply));
    if (c->replies.size() > kMaxReplyBacklog) {   // abusive: not draining its replies
        c->stop = true;
        say(log_, LvL::WARN, "control: client " + c->peer +
                             " reply backlog overflow — disconnecting");
    }
}

void ControlServer::pushTelemetry(const std::string& line) {
    if (!running_.load() || line.empty()) return;

    std::string framed = line;
    if (framed.back() != '\n') framed.push_back('\n');

    std::lock_guard<std::mutex> lk(clientsMtx_);
    for (auto& c : clients_) {
        if (c->done.load()) continue;  // reaped on the next accept tick
        std::lock_guard<std::mutex> clk(c->mtx);
        if (!c->authed) continue;      // never leak telemetry to an unauthenticated client
        c->out.push_back(framed);
        while (static_cast<int>(c->out.size()) > cfg_.queueDepth)
            c->out.pop_front();        // drop oldest: telemetry favours the latest sample
    }
}

void ControlServer::reapDoneLocked() {
    for (auto it = clients_.begin(); it != clients_.end();) {
        if ((*it)->done.load()) {
            if ((*it)->thread.joinable()) (*it)->thread.join();  // already exited: joins immediately
            const std::string peer = (*it)->peer;
            it = clients_.erase(it);
            say(log_, LvL::INFO, "control: client disconnected " + peer +
                                 " (" + std::to_string(clients_.size()) + " total)");
        } else {
            ++it;
        }
    }
}

int ControlServer::clientCount() const {
    std::lock_guard<std::mutex> lk(clientsMtx_);
    int n = 0;
    for (const auto& c : clients_)
        if (!c->done.load()) ++n;
    return n;
}

void ControlServer::stop() {
    if (!running_.exchange(false)) {
        // Not running, but a listener may still exist from a failed start().
        server_.close();
        return;
    }

    server_.close();  // wakes acceptLoop's accept() -> Closed -> loop exits
    if (acceptThread_.joinable()) acceptThread_.join();

    // Take ownership of the client list, then release each client thread.
    std::vector<std::unique_ptr<Client>> toStop;
    {
        std::lock_guard<std::mutex> lk(clientsMtx_);
        toStop.swap(clients_);
    }
    for (auto& c : toStop) {
        {
            std::lock_guard<std::mutex> clk(c->mtx);
            c->stop = true;
        }
        c->sock.shutdown();   // unblock a client thread parked in recv()
        if (c->thread.joinable()) c->thread.join();
    }
    say(log_, LvL::INFO, "control: stopped");
}

} // namespace dashcam::network
