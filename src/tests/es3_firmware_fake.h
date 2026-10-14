// es3_firmware_fake.h — the ES3C28P firmware's link and video logic on a pty,
// for the Jetson/firmware compatibility checks in test_libes3display.cpp.
//
// Unlike FakeDisplay (written from the frozen contract alone), this fake runs
// the colleague's FIRMWARE code where it is portable — es3video::VideoState
// (peripherals/es3c28p/lib/es3Video/VideoState.h, unmodified) and the shared
// MuxParser — and mirrors the Arduino-only parts line by line:
//   - FirmwareFake::service() is Es3UsbLink::service()
//     (peripherals/es3c28p/lib/es3UsbLink/es3UsbLink.cpp) on a 1 ms tick: the
//     5 s session timeout, the 1 s TX stall, VideoState faults, parser expiry,
//     ONE pending loop command during which NOTHING is read, a 4 KiB RX budget
//     per turn, control replies before queued video events;
//   - dispatch()/apply() are Es3UsbLink::dispatch()/apply(): HELLO, PING,
//     CAPABILITIES, START_VIDEO and STOP_VIDEO answered by the USB task;
//     STATUS/PAGE/ALERT/STOP_AUDIO handed to the "Arduino loop" thread;
//   - decodeLoop() is Es3Video::decodeLoop() with libjpeg standing in for
//     JPEGDEC (baseline only, exact viewport size, as JpegDecoder.h checks);
//   - loop() is the dashboard loop: setDemand() every turn, the pending
//     command, then one image drawn as 8-row strips with 1 ms between turns.
// Every VideoState call is serialised by one mutex, like Es3Video.
//
// Knobs: applyDelayMs (time the loop spends in a command, e.g. a PAGE redraw),
// blitMs (whole-image draw time), demand (the visible tab), rxBytesPerMs (the
// USB device's intake; 0 = unlimited).

#pragma once

#include "Es3VideoProtocol.h"
#include "../../peripherals/es3c28p/lib/es3Video/VideoState.h"

#include <deque>

class FirmwareFake {
public:
    std::atomic<int>     applyDelayMs{2};
    std::atomic<int>     blitMs{23};
    std::atomic<uint8_t> demand{es3video::DemandActive};
    std::atomic<int>     rxBytesPerMs{0};

    struct Counters {
        uint32_t hello = 0, ping = 0, capabilities = 0, startVideo = 0, stopVideo = 0, appCommands = 0;
        uint32_t resets = 0, sessionTimeouts = 0, txStalls = 0, faults = 0;
        uint32_t startsWhileHidden = 0;   ///< START records that arrived while demand != active.
        uint32_t decodeFailures = 0;
        es3video::VideoState::Stats video; ///< VideoState::stats snapshot.
    };

    FirmwareFake() {
        for (int i = 0; i < 2; ++i) {
            jpegMem_[i].resize(es3video::kMaxJpegBytes);
            pixMem_[i].resize(320 * 240);
            vs_.jpeg[i]   = jpegMem_[i].data();
            vs_.pixels[i] = pixMem_[i].data();
        }
        token_ = 0x2e5c0000u;
    }
    ~FirmwareFake() { unplug(); }

    bool plug(const std::string& link) {
        unplug();
        const int fd = ::posix_openpt(O_RDWR | O_NOCTTY | O_CLOEXEC);
        if (fd < 0 || ::grantpt(fd) != 0 || ::unlockpt(fd) != 0) {
            if (fd >= 0) ::close(fd);
            return false;
        }
        const char* name = ::ptsname(fd);
        struct stat sb;
        if (!name || ::stat(name, &sb) != 0) {
            ::close(fd);
            return false;
        }
        ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
        const std::string tmp = link + ".new";
        ::unlink(tmp.c_str());
        if (::symlink(name, tmp.c_str()) != 0 || ::rename(tmp.c_str(), link.c_str()) != 0) {
            ::close(fd);
            return false;
        }
        outq::arm(uint64_t(sb.st_rdev));
        master_ = fd;
        run_    = true;
        t0_     = nowMs();
        usb_    = std::thread([this] { while (run_) { service(); sleepMs(1); } });
        jpeg_   = std::thread([this] { decodeLoop(); });
        app_    = std::thread([this] { while (run_) { loop(); sleepMs(1); } });
        return true;
    }
    void unplug() {
        run_ = false;
        if (usb_.joinable()) usb_.join();
        if (jpeg_.joinable()) jpeg_.join();
        if (app_.joinable()) app_.join();
        if (master_ >= 0) {
            outq::armedRdev = 0;
            ::close(master_);
            master_ = -1;
        }
    }
    Counters counters() {
        std::lock_guard<std::mutex> lk(m_);
        Counters c = c_;
        c.video    = vs_.stats;
        return c;
    }

private:
    // ── Es3UsbLink state (USB task) ──────────────────────────────────────────
    struct Command  { es3proto::Frame frame; uint32_t epoch = 0; };
    struct Response { uint8_t bytes[es3proto::kMaxFrame]; size_t size = 0; uint32_t epoch = 0; };
    es3video::MuxParser parser_;
    std::deque<uint8_t> rx_;                 ///< Serial RX buffer (8 KiB on the board).
    uint8_t  tx_[es3proto::kMaxFrame]{};
    size_t   txSize_ = 0, txOffset_ = 0;
    uint32_t lastRequest_ = 0, txStarted_ = 0;
    bool     session_ = false, pending_ = false;
    std::atomic<uint32_t> epoch_{0};
    std::mutex cmdMu_;                        ///< The two depth-1 FreeRTOS queues.
    bool haveCommand_ = false, haveResponse_ = false;
    Command  command_;
    Response response_;

    // ── Es3Video state ───────────────────────────────────────────────────────
    std::mutex             m_;                ///< Es3Video::_mutex (+ counters).
    es3video::VideoState   vs_;
    std::vector<uint8_t>   jpegMem_[2];
    std::vector<uint16_t>  pixMem_[2];
    uint32_t               token_;
    Counters               c_;

    int                    master_ = -1;
    std::atomic<bool>      run_{false};
    std::thread            usb_, jpeg_, app_;
    int64_t                t0_ = 0;

    uint32_t millis() const { return uint32_t(nowMs() - t0_); }

    void writeSome() {
        if (!txSize_) return;
        const ssize_t n = ::write(master_, tx_ + txOffset_, txSize_ - txOffset_);
        if (n > 0) txOffset_ += size_t(n);
        if (txOffset_ == txSize_) txSize_ = txOffset_ = 0;
    }
    void queueReply(const es3proto::Frame& rq, const uint8_t* d = nullptr, size_t n = 0) {
        txSize_ = es3proto::encode(rq.sequence, rq.type | es3proto::Reply, d, n, tx_);
        txOffset_ = 0; txStarted_ = millis();
    }
    void queueError(const es3proto::Frame& rq, es3proto::ErrorCode code) {
        const uint8_t v = code;
        txSize_ = es3proto::encode(rq.sequence, es3proto::Error, &v, 1, tx_);
        txOffset_ = 0; txStarted_ = millis();
    }
    void reset() {
        session_ = false; epoch_.fetch_add(1);
        { std::lock_guard<std::mutex> lk(m_); vs_.reset(); ++c_.resets; }   // _video.stop()
        parser_.reset();
        txSize_ = txOffset_ = 0; pending_ = false;
        std::lock_guard<std::mutex> lk(cmdMu_);
        haveCommand_ = haveResponse_ = false;
    }
    es3video::Capabilities caps() const {
        es3video::Capabilities k;
        k.canvasWidth = k.maxWidth = 320;
        k.canvasHeight = k.maxHeight = 240;
        k.maxJpegBytes  = es3video::kMaxJpegBytes;
        k.maxCredits    = 1;
        k.maxChunkBytes = es3video::kDefaultChunkDataBytes;
        return k;
    }

    void dispatch(const es3proto::Frame& f) {
        using namespace es3proto;
        using namespace es3video;
        if (f.type == Hello) {
            if (f.length) { queueError(f, BadPayload); return; }
            { std::lock_guard<std::mutex> lk(m_); ++c_.hello; }
            reset(); session_ = true; lastRequest_ = millis();
            const uint8_t id[] = {'E', 'S', '3', 'C', '2', '8', 'P'};
            queueReply(f, id, sizeof(id));
            return;
        }
        if (!session_) { queueError(f, NoSession); return; }
        lastRequest_ = millis();
        if (f.type == Ping) {
            { std::lock_guard<std::mutex> lk(m_); ++c_.ping; }
            if (f.length) queueError(f, BadPayload); else queueReply(f);
        } else if (f.type == es3proto::Capabilities) {
            { std::lock_guard<std::mutex> lk(m_); ++c_.capabilities; }
            if (f.length) { queueError(f, BadPayload); return; }
            uint8_t d[kCapabilitiesReplyBytes];
            queueReply(f, d, encodeCapabilitiesReply(caps(), d, sizeof(d)));
        } else if (f.type == es3proto::StartVideo) {
            { std::lock_guard<std::mutex> lk(m_); ++c_.startVideo; }
            StartVideoRequest rq;
            if (!decodeStartVideoRequest(f.payload, f.length, caps(), rq)) { queueError(f, BadPayload); return; }
            StartVideoReply rp;
            {   // Es3Video::start()
                std::lock_guard<std::mutex> lk(m_);
                if (++token_ == 0) ++token_;
                vs_.start(token_, rq);
                rp.sessionToken = token_;
            }
            rp.initialGrant = 1;
            uint8_t d[kStartVideoReplyBytes];
            queueReply(f, d, encodeStartVideoReply(rp, d, sizeof(d)));
        } else if (f.type == es3proto::StopVideo) {
            { std::lock_guard<std::mutex> lk(m_); ++c_.stopVideo; }
            uint32_t token = 0;
            if (!decodeStopVideoRequest(f.payload, f.length, token)) { queueError(f, BadPayload); return; }
            { std::lock_guard<std::mutex> lk(m_); if (!token || token == vs_.token) vs_.reset(); }
            queueReply(f);
        } else {
            std::lock_guard<std::mutex> lk(cmdMu_);
            if (haveCommand_) { haveCommand_ = false; reset(); return; }   // xQueueSend failed
            command_.frame = f; command_.epoch = epoch_.load(); haveCommand_ = true;
            pending_ = true;
        }
    }

    void service() {
        const uint32_t now = millis();
        bool fault;
        { std::lock_guard<std::mutex> lk(m_); fault = vs_.fault(); }
        const bool expired = session_ && now - lastRequest_ > 5000;
        const bool stalled = txSize_ && now - txStarted_ > 1000;
        if (expired || stalled || fault) {
            std::lock_guard<std::mutex> lk(m_);
            if (expired) ++c_.sessionTimeouts;
            if (stalled) ++c_.txStalls;
            if (fault) ++c_.faults;
        }
        if (expired || stalled || fault) reset();
        { std::lock_guard<std::mutex> lk(m_); vs_.expire(now); }
        parser_.expire(now);
        if (!txSize_ && pending_) {
            std::lock_guard<std::mutex> lk(cmdMu_);
            if (haveResponse_ && response_.epoch == epoch_.load()) {
                std::memcpy(tx_, response_.bytes, response_.size); txSize_ = response_.size;
                txOffset_ = 0; txStarted_ = now; pending_ = false; haveResponse_ = false;
            }
        }
        writeSome();
        // The host's bytes land in the 8 KiB RX buffer whether or not the task
        // reads them (USB keeps NAKing once it is full).
        {
            const int    rate  = rxBytesPerMs.load();
            const size_t room  = rx_.size() < 8192 ? 8192 - rx_.size() : 0;
            const size_t limit = rate > 0 ? std::min<size_t>(room, size_t(rate)) : room;
            uint8_t b[8192];
            if (limit) {
                const ssize_t n = ::read(master_, b, limit);
                if (n > 0) { rx_.insert(rx_.end(), b, b + n); outq::consumed += n; }
            }
        }
        if (!txSize_ && !pending_) {
            es3video::MuxParser::Item item;
            for (size_t budget = 0; budget < 4096; ++budget) {
                parser_.feed(nullptr, 0, now, item);
                if (item.kind == es3video::MuxParser::ItemNone) {
                    if (rx_.empty()) break;
                    const uint8_t byte = rx_.front();
                    rx_.pop_front();
                    parser_.feed(&byte, 1, now, item);
                }
                if (item.kind == es3video::MuxParser::ItemControl) { dispatch(item.control); break; }
                if (item.kind == es3video::MuxParser::ItemVideo && session_) {
                    std::lock_guard<std::mutex> lk(m_);
                    if (item.video.type == es3video::RecordStart && vs_.demand != es3video::DemandActive)
                        ++c_.startsWhileHidden;
                    vs_.accept(item.video, now);
                }
            }
        }
        if (!txSize_ && !pending_) {
            es3video::VideoState::Event e;
            bool have;
            { std::lock_guard<std::mutex> lk(m_); have = vs_.pop(e); }
            if (have) { std::memcpy(tx_, e.bytes, e.size); txSize_ = e.size; txOffset_ = 0; txStarted_ = now; }
        }
        writeSome();
    }

    // ── Es3Video::decodeLoop() (JPEGDEC -> libjpeg) ──────────────────────────
    void decodeLoop() {
        std::vector<uint8_t> row(320 * 3 * 2);
        while (run_) {
            es3video::VideoState::Job job;
            bool have;
            { std::lock_guard<std::mutex> lk(m_); have = vs_.takeJob(job); }
            if (!have) { sleepMs(1); continue; }
            const int64_t t0 = nowUs();
            int w = 0, h = 0;
            bool ok = decodeJpeg(vs_.jpeg[job.jpeg], job.bytes, row, w, h) && w == job.width && h == job.height;
            const uint32_t us = uint32_t(nowUs() - t0);
            std::lock_guard<std::mutex> lk(m_);
            if (!ok) ++c_.decodeFailures;
            vs_.finishJob(job, ok, us);
        }
    }

    // ── the dashboard loop (core 1) ──────────────────────────────────────────
    es3video::VideoState::Image drawing_;
    int64_t drawEndUs_ = 0;

    void loop() {
        { std::lock_guard<std::mutex> lk(m_); vs_.setDemand(es3video::Demand(demand.load())); }
        // Es3UsbLink::update(): one queued command, answered for its epoch.
        Command cmd;
        bool have = false;
        {
            std::lock_guard<std::mutex> lk(cmdMu_);
            if (haveCommand_) { cmd = command_; haveCommand_ = false; have = cmd.epoch == epoch_.load(); }
        }
        if (have) {
            sleepMs(applyDelayMs.load());
            Response r;
            uint8_t  st[9] = {};
            const bool known = cmd.frame.type == es3proto::Status || cmd.frame.type == es3proto::Page ||
                               cmd.frame.type == es3proto::Alert || cmd.frame.type == es3proto::StopAudio;
            if (known) {
                r.size = es3proto::encode(cmd.frame.sequence, cmd.frame.type | es3proto::Reply, st,
                                          cmd.frame.type == es3proto::Status ? sizeof(st) : 0, r.bytes);
            } else {
                const uint8_t v = es3proto::BadCommand;
                r.size = es3proto::encode(cmd.frame.sequence, es3proto::Error, &v, 1, r.bytes);
            }
            r.epoch = cmd.epoch;
            {
                std::lock_guard<std::mutex> lk(m_);
                ++c_.appCommands;
            }
            std::lock_guard<std::mutex> lk(cmdMu_);
            if (cmd.epoch == epoch_.load()) { response_ = r; haveResponse_ = true; }
        }
        // One image as strips: hold the lease until the last strip, then release.
        std::lock_guard<std::mutex> lk(m_);
        if (drawing_.slot < 0) {
            if (vs_.acquire(drawing_)) drawEndUs_ = nowUs() + int64_t(blitMs.load()) * 1000;
        } else if (nowUs() >= drawEndUs_) {
            vs_.release(drawing_, true, uint32_t(blitMs.load()) * 1000);
        }
    }
};
