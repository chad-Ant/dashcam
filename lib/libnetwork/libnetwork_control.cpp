/**
 * @file libnetwork_control.cpp
 * @brief ControlServer — a bidirectional TCP control + telemetry channel.
 *
 * One connection carries both directions: the remote operator SENDS
 * newline-delimited command lines (dispatched to the application's
 * ControlHandler — e.g. re-point an RTP stream), and the device PUSHES telemetry
 * lines back to every connected client.  A pure viewer just connects and reads.
 *
 * Structured like MediaStreamServer (libnetwork_stream.cpp): a TcpServer accept
 * loop plus one thread per client.  That client thread interleaves a
 * short-timeout recv (inbound commands) with a bounded outbound-queue flush
 * (telemetry + replies), so a stalled operator drops its own telemetry rather
 * than blocking the producer or the other clients.
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

} // namespace

// ─── per-client state ──────────────────────────────────────────────────────────

struct ControlServer::Client {
    TcpSocket               sock;
    std::thread             thread;
    std::mutex              mtx;
    std::deque<std::string> out;          ///< Pending outbound lines (telemetry + replies).
    bool                    stop = false;
    std::atomic<bool>       done{false};  ///< Client thread has exited.
    std::string             peer;         ///< "ip:port".
    std::string             ip;           ///< peer IP only.
    std::string             inbuf;        ///< Accumulates a partial inbound line.
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

    if (!server_.listen(cfg_.port, /*backlog=*/cfg_.maxClients + 1, log_)) {
        say(log_, LvL::ERROR, "control: listen on port " + std::to_string(cfg_.port) + " failed");
        return false;
    }
    running_.store(true);
    acceptThread_ = std::thread([this] { acceptLoop(); });
    say(log_, LvL::INFO, "control: serving on port " + std::to_string(server_.port()) +
                         " (max " + std::to_string(cfg_.maxClients) + " clients)");
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

        if (static_cast<int>(clients_.size()) >= cfg_.maxClients) {
            say(log_, LvL::WARN, "control: refusing client " + s.peer() +
                                 " (at maxClients=" + std::to_string(cfg_.maxClients) + ")");
            s.close();
            continue;
        }

        auto c   = std::make_unique<Client>();
        c->peer  = s.peer();
        c->ip    = ipOnly(c->peer);
        c->sock  = std::move(s);
        Client* cp = c.get();
        c->thread = std::thread([this, cp] { clientLoop(cp); });
        clients_.push_back(std::move(c));
        say(log_, LvL::INFO, "control: client connected " + cp->peer +
                             " (" + std::to_string(clients_.size()) + " total)");
    }
}

void ControlServer::clientLoop(Client* c) {
    // Greeting so an interactive operator (nc/telnet) knows the channel is live.
    const std::string hello = "dashcam control ready — type HELP\n";
    if (!c->sock.sendAll(hello.data(), hello.size())) {
        c->done.store(true);
        return;
    }

    char buf[1024];
    for (;;) {
        // 1) Flush any queued outbound lines (telemetry + replies).
        for (;;) {
            std::string line;
            {
                std::lock_guard<std::mutex> lk(c->mtx);
                if (c->stop || c->out.empty()) break;
                line = std::move(c->out.front());
                c->out.pop_front();
            }
            if (!c->sock.sendAll(line.data(), line.size())) {
                c->done.store(true);
                return;  // client gone or shutdown() called
            }
        }
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

    ControlCommand cmd;
    cmd.raw      = line;
    cmd.clientIp = c->ip;
    cmd.verb     = toUpper(toks.front());
    cmd.args.assign(toks.begin() + 1, toks.end());

    std::string reply = handler_ ? handler_(cmd) : std::string{};
    if (reply.empty()) return;
    if (reply.back() != '\n') reply.push_back('\n');

    std::lock_guard<std::mutex> lk(c->mtx);
    c->out.push_back(std::move(reply));
    while (static_cast<int>(c->out.size()) > cfg_.queueDepth)
        c->out.pop_front();
}

void ControlServer::pushTelemetry(const std::string& line) {
    if (!running_.load() || line.empty()) return;

    std::string framed = line;
    if (framed.back() != '\n') framed.push_back('\n');

    std::lock_guard<std::mutex> lk(clientsMtx_);
    for (auto& c : clients_) {
        if (c->done.load()) continue;  // reaped on the next accept tick
        std::lock_guard<std::mutex> clk(c->mtx);
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
