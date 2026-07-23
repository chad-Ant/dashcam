/**
 * @file libnetwork_stream.cpp
 * @brief MediaStreamServer — fan compressed frames out to multiple TCP viewers.
 *
 * The "direct" streaming path for already-compressed camera feeds (the UVC
 * recording cameras): no decode, no re-encode.  Each viewer runs on its own
 * writer thread with a bounded latest-frame queue, so a slow viewer drops its
 * own frames rather than stalling the producer or the other viewers.
 */

#include "libnetwork.h"

#include <condition_variable>
#include <deque>
#include <string>
#include <utility>

namespace dashcam::network {

using LvL = dashcam::log::LogLevel;

namespace {

void say(const dashcam::log::LogCallback& log, LvL lvl, const std::string& msg) {
    if (log) log(lvl, msg);
}

// One HTTP response header, sent once per viewer before any frame.
std::string httpHeader(const std::string& boundary) {
    return "HTTP/1.0 200 OK\r\n"
           "Connection: close\r\n"
           "Server: dashcam-libnetwork\r\n"
           "Cache-Control: no-cache, no-store, must-revalidate\r\n"
           "Pragma: no-cache\r\n"
           "Content-Type: multipart/x-mixed-replace; boundary=" + boundary + "\r\n"
           "\r\n";
}

// One multipart part wrapping a single JPEG frame.
std::string mjpegPart(const std::string& boundary, const void* data, size_t len) {
    std::string p;
    p.reserve(len + boundary.size() + 64);
    p += "--";
    p += boundary;
    p += "\r\nContent-Type: image/jpeg\r\nContent-Length: ";
    p += std::to_string(len);
    p += "\r\n\r\n";
    p.append(static_cast<const char*>(data), len);
    p += "\r\n";
    return p;
}

} // namespace

// ─── per-viewer state ──────────────────────────────────────────────────────────

struct MediaStreamServer::Client {
    TcpSocket                                       sock;
    std::thread                                     thread;
    std::mutex                                      mtx;
    std::condition_variable                         cv;
    std::deque<std::shared_ptr<const std::string>>  queue;   ///< Pending frames (latest-wins).
    bool                                            stop = false;
    std::atomic<bool>                               done{false};  ///< Writer thread has exited.
    std::string                                     peer;
};

// ─── MediaStreamServer ─────────────────────────────────────────────────────────

MediaStreamServer::MediaStreamServer()  = default;
MediaStreamServer::~MediaStreamServer() { stop(); }

bool MediaStreamServer::start(const StreamServerConfig& cfg,
                              const dashcam::log::LogCallback& log) {
    if (running_.load()) return false;
    cfg_ = cfg;
    log_ = log;

    if (!server_.listen(cfg_.port, /*backlog=*/cfg_.maxClients + 1, log_)) {
        say(log_, LvL::ERROR, "stream: listen on port " + std::to_string(cfg_.port) + " failed");
        return false;
    }
    running_.store(true);
    acceptThread_ = std::thread([this] { acceptLoop(); });
    say(log_, LvL::INFO, "stream: serving " +
        std::string(cfg_.wire == StreamWire::MjpegHttp ? "MJPEG/HTTP" : "raw-TCP") +
        " on port " + std::to_string(server_.port()) +
        " (max " + std::to_string(cfg_.maxClients) + " viewers)");
    return true;
}

void MediaStreamServer::acceptLoop() {
    while (running_.load()) {
        IoStatus st;
        TcpSocket s = server_.accept(500, &st);

        std::lock_guard<std::mutex> lk(clientsMtx_);
        reapDoneLocked();  // periodic cleanup of viewers that disconnected

        if (st != IoStatus::Ok) {
            if (st == IoStatus::Closed) break;  // stop() closed the listener
            continue;                           // Timeout / transient Error
        }

        if (static_cast<int>(clients_.size()) >= cfg_.maxClients) {
            say(log_, LvL::WARN, "stream: refusing viewer " + s.peer() +
                                 " (at maxClients=" + std::to_string(cfg_.maxClients) + ")");
            s.close();
            continue;
        }

        auto c   = std::make_unique<Client>();
        c->peer  = s.peer();
        c->sock  = std::move(s);
        Client* cp = c.get();
        c->thread = std::thread([this, cp] { clientLoop(cp); });
        clients_.push_back(std::move(c));
        say(log_, LvL::INFO, "stream: viewer connected " + cp->peer +
                             " (" + std::to_string(clients_.size()) + " total)");
    }
}

void MediaStreamServer::clientLoop(Client* c) {
    // Header first (once), then drain the frame queue in order.
    if (cfg_.wire == StreamWire::MjpegHttp) {
        const std::string hdr = httpHeader(cfg_.boundary);
        if (!c->sock.sendAll(hdr.data(), hdr.size())) {
            c->done.store(true);
            return;
        }
    }

    for (;;) {
        std::shared_ptr<const std::string> payload;
        {
            std::unique_lock<std::mutex> lk(c->mtx);
            c->cv.wait(lk, [&] { return c->stop || !c->queue.empty(); });
            if (c->stop && c->queue.empty()) break;
            payload = std::move(c->queue.front());
            c->queue.pop_front();
        }
        if (!c->sock.sendAll(payload->data(), payload->size()))
            break;  // viewer gone or shutdown() called
    }
    c->done.store(true);
}

void MediaStreamServer::pushFrame(const void* data, size_t len) {
    if (!running_.load() || data == nullptr || len == 0) return;

    // Build the framed payload once; every viewer shares it.
    std::shared_ptr<const std::string> payload =
        (cfg_.wire == StreamWire::MjpegHttp)
            ? std::make_shared<const std::string>(mjpegPart(cfg_.boundary, data, len))
            : std::make_shared<const std::string>(static_cast<const char*>(data), len);

    std::lock_guard<std::mutex> lk(clientsMtx_);
    for (auto& c : clients_) {
        if (c->done.load()) continue;  // reaped on the next accept tick
        std::lock_guard<std::mutex> clk(c->mtx);
        c->queue.push_back(payload);
        while (static_cast<int>(c->queue.size()) > cfg_.queueDepth)
            c->queue.pop_front();      // drop oldest: a live view favours the latest frame
        c->cv.notify_one();
    }
}

void MediaStreamServer::reapDoneLocked() {
    for (auto it = clients_.begin(); it != clients_.end();) {
        if ((*it)->done.load()) {
            if ((*it)->thread.joinable()) (*it)->thread.join();  // already exited: joins immediately
            const std::string peer = (*it)->peer;
            it = clients_.erase(it);
            say(log_, LvL::INFO, "stream: viewer disconnected " + peer +
                                 " (" + std::to_string(clients_.size()) + " total)");
        } else {
            ++it;
        }
    }
}

int MediaStreamServer::clientCount() const {
    std::lock_guard<std::mutex> lk(clientsMtx_);
    int n = 0;
    for (const auto& c : clients_)
        if (!c->done.load()) ++n;
    return n;
}

void MediaStreamServer::stop() {
    if (!running_.exchange(false)) {
        // Not running, but a listener may still exist from a failed start().
        server_.close();
        return;
    }

    server_.close();  // wakes acceptLoop's accept() -> Closed -> loop exits
    if (acceptThread_.joinable()) acceptThread_.join();

    // Take ownership of the viewer list, then release each writer thread.
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
        c->cv.notify_one();   // wake an idle writer
        c->sock.shutdown();   // unblock a writer stuck in send() on a slow viewer
        if (c->thread.joinable()) c->thread.join();
    }
    say(log_, LvL::INFO, "stream: stopped");
}

} // namespace dashcam::network
