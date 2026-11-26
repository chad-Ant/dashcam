#pragma once
// Best-effort camera preview for the host monitoring agent (connector/README.md,
// "Camera video").  A second VideoPreview converts the recording camera's JPEGs
// to SD, 720x404, while a viewer watches; this sink sends each one as ONE nonblocking
// Unix datagram.  Never opens a camera, never waits for the agent: a missing or
// stalled agent only drops preview frames, recording is unaffected.
//
// Datagram (little-endian): 0-3 "DCV1", 4-7 uint32 header length (24), 8-15
// uint64 capture CLOCK_MONOTONIC us, 16-19 uint32 sequence, 20-21 uint16 width,
// 22-23 uint16 height, then the JPEG.
//
// Demand: the agent touches a file while someone watches.  pollDemand() (the
// supervisor, once per tick) stats it; demand() only reads the cached flag, so
// the camera's streaming thread never makes a syscall here.
#include "libvideopreview.h"
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/uio.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace dashcam::monitor {
constexpr size_t   kVideoHeaderBytes   = 24;
constexpr size_t   kVideoMaxJpegBytes  = 65536;   ///< = the monitor preview's PreviewConfig::maxJpegBytes.
// SD: a 16:9 camera fills 720x404 of this canvas (the letterbox keeps sizes even), which
// the converter decodes at 3/8 (1920x1080 -> 720x405) and crops by one row: no resampling.
constexpr uint16_t kVideoCanvasWidth   = 720;
constexpr uint16_t kVideoCanvasHeight  = 406;
constexpr double   kVideoDemandFreshSec = 3.0;    ///< Demand file older (or newer) than this = nobody watching.

inline void putLe(uint8_t* out, uint64_t v, size_t bytes) {
    for (size_t i = 0; i < bytes; ++i) out[i] = static_cast<uint8_t>(v >> (8 * i));
}
/// The 24-byte datagram header into @p out (at least kVideoHeaderBytes).
inline void writeVideoHeader(uint8_t* out, uint64_t captureMonotonicUs, uint32_t sequence,
                             uint16_t width, uint16_t height) {
    std::memcpy(out, "DCV1", 4);
    putLe(out + 4, kVideoHeaderBytes, 4);
    putLe(out + 8, captureMonotonicUs, 8);
    putLe(out + 16, sequence, 4);
    putLe(out + 20, width, 2);
    putLe(out + 22, height, 2);
}
/// Pure demand rule: a regular file whose mtime is within kVideoDemandFreshSec
/// of @p now (CLOCK_REALTIME), in either direction.
inline bool demandFresh(const struct stat& st, const timespec& now) {
    if (!S_ISREG(st.st_mode)) return false;
    const double age = static_cast<double>(now.tv_sec - st.st_mtim.tv_sec) +
                       static_cast<double>(now.tv_nsec - st.st_mtim.tv_nsec) * 1e-9;
    return age <= kVideoDemandFreshSec && age >= -kVideoDemandFreshSec;
}
inline bool demandFileFresh(const std::string& path) {
    struct stat st{};
    timespec now{};
    if (path.empty() || ::stat(path.c_str(), &st) != 0 || ::clock_gettime(CLOCK_REALTIME, &now) != 0) return false;
    return demandFresh(st, now);
}

struct MonitorVideoStats {
    uint64_t sent        = 0;
    uint64_t dropBusy    = 0;   ///< EAGAIN: the agent's queue is full (stalled reader).
    uint64_t dropMissing = 0;   ///< ENOENT: no agent socket.
    uint64_t dropRefused = 0;   ///< ECONNREFUSED: stale socket, no reader.
    uint64_t dropOther   = 0;   ///< Any other send error, or a frame larger than kVideoMaxJpegBytes.
    uint64_t dropped() const { return dropBusy + dropMissing + dropRefused + dropOther; }
};

/// IPreviewSink for the monitoring agent.  offerFrame() runs on the preview
/// worker only (one thread: it owns the send buffer); demand() is safe from
/// any thread; pollDemand() from one (the supervisor).
class MonitorVideoSink : public dashcam::preview::IPreviewSink {
public:
    MonitorVideoSink() = default;
    MonitorVideoSink(const MonitorVideoSink&) = delete;
    MonitorVideoSink& operator=(const MonitorVideoSink&) = delete;
    ~MonitorVideoSink() override { if (fd_ >= 0) ::close(fd_); }

    /// Unbound nonblocking sender towards @p socketPath; allocates the send buffer once.
    bool open(const std::string& socketPath, const std::string& demandPath) {
        if (fd_ >= 0 || socketPath.empty() || socketPath.size() >= sizeof(address_.sun_path)) return false;
        buffer_.reset(new (std::nothrow) uint8_t[kVideoHeaderBytes + kVideoMaxJpegBytes]);
        if (!buffer_) return false;
        address_.sun_family = AF_UNIX;
        std::memcpy(address_.sun_path, socketPath.c_str(), socketPath.size() + 1);
        demandPath_ = demandPath;
        fd_ = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        return fd_ >= 0;
    }
    bool enabled() const { return fd_ >= 0; }

    /// Re-read the demand file; returns (and caches for demand()) whether a viewer watches.
    bool pollDemand() {
        const bool active = fd_ >= 0 && demandFileFresh(demandPath_);
        demanded_.store(active, std::memory_order_relaxed);
        return active;
    }
    bool demanded() const { return demanded_.load(std::memory_order_relaxed); }

    dashcam::preview::PreviewDemand demand() const override {
        dashcam::preview::PreviewDemand d;
        d.active       = demanded_.load(std::memory_order_relaxed);
        d.canvasW      = kVideoCanvasWidth;
        d.canvasH      = kVideoCanvasHeight;
        d.maxJpegBytes = 0;
        return d;
    }

    /// Sends the frame (or counts why not) and always consumes it: the lease
    /// goes back to the pool when it leaves this scope.
    bool offerFrame(dashcam::preview::EncodedFrameLease&& frame) override {
        const dashcam::preview::EncodedFrameLease lease(std::move(frame));
        if (fd_ < 0 || !lease.valid() || lease.size() == 0 || lease.size() > kVideoMaxJpegBytes) {
            dropOther_.fetch_add(1, std::memory_order_relaxed);
            return true;
        }
        writeVideoHeader(buffer_.get(), lease.captureMonotonicUs(), static_cast<uint32_t>(lease.sourceSequence()),
                         lease.width(), lease.height());
        std::memcpy(buffer_.get() + kVideoHeaderBytes, lease.data(), lease.size());
        const size_t total = kVideoHeaderBytes + lease.size();
        const ssize_t n = ::sendto(fd_, buffer_.get(), total, MSG_DONTWAIT | MSG_NOSIGNAL,
                                   reinterpret_cast<const sockaddr*>(&address_), sizeof(address_));
        if (n == static_cast<ssize_t>(total)) sent_.fetch_add(1, std::memory_order_relaxed);
        else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) dropBusy_.fetch_add(1, std::memory_order_relaxed);
        else if (n < 0 && errno == ENOENT) dropMissing_.fetch_add(1, std::memory_order_relaxed);
        else if (n < 0 && errno == ECONNREFUSED) dropRefused_.fetch_add(1, std::memory_order_relaxed);
        else dropOther_.fetch_add(1, std::memory_order_relaxed);
        return true;
    }

    MonitorVideoStats stats() const {
        MonitorVideoStats s;
        s.sent        = sent_.load(std::memory_order_relaxed);
        s.dropBusy    = dropBusy_.load(std::memory_order_relaxed);
        s.dropMissing = dropMissing_.load(std::memory_order_relaxed);
        s.dropRefused = dropRefused_.load(std::memory_order_relaxed);
        s.dropOther   = dropOther_.load(std::memory_order_relaxed);
        return s;
    }

private:
    int fd_ = -1;
    sockaddr_un address_{};
    std::string demandPath_;
    std::unique_ptr<uint8_t[]> buffer_;
    std::atomic<bool> demanded_{false};
    std::atomic<uint64_t> sent_{0}, dropBusy_{0}, dropMissing_{0}, dropRefused_{0}, dropOther_{0};
};

// ── HD: the camera's original frames, unconverted ──────────────────────────────
// While the agent's HD demand file is fresh (a direct-LAN viewer selected HD),
// every recorded camera JPEG goes to the agent byte for byte, at the camera's
// rate.  A 1080p MJPEG frame (60-400 KB) does not fit one Unix datagram
// (net.core.wmem_max is 212992 on the Jetson), so each frame is cut into chunks,
// each its own datagram on the same socket as the SD preview:
//
//   0-3 "DCH1", 4-7 uint32 header length (32), 8-15 uint64 capture
//   CLOCK_MONOTONIC us, 16-19 uint32 source sequence, 20-21 uint16 width,
//   22-23 uint16 height, 24-27 uint32 total JPEG bytes, 28-31 uint32 offset of
//   this chunk, then up to kHdChunkBytes of the JPEG.
//
// Chunks go in order; a full queue is waited out by the worker for up to
// kHdFrameBudgetMs, and any other failure abandons the rest of that frame (the
// agent discards an incomplete one).  The camera's thread only copies the frame into
// a latest-frame slot; a worker thread sends it, so a slow agent only costs
// HD frames, never recording.
constexpr size_t   kHdHeaderBytes   = 32;
constexpr size_t   kHdChunkBytes    = 61440;     ///< JPEG bytes per datagram.
constexpr size_t   kHdMaxJpegBytes  = 1048576;   ///< = the agent's limit; larger frames are counted, not sent.
constexpr uint32_t kHdMaxDimension  = 4096;
constexpr int      kHdFrameBudgetMs = 100;       ///< Longest the worker waits on a full queue for one frame.
constexpr int      kHdRetryPauseMs  = 2;         ///< Shortest gap between two sends after EAGAIN: no spin.
constexpr int      kHdRetryPollMs   = 20;        ///< Longest single poll() while waiting for room.

/// The 32-byte chunk header into @p out (at least kHdHeaderBytes).
inline void writeHdChunkHeader(uint8_t* out, uint64_t captureMonotonicUs, uint32_t sequence, uint16_t width,
                               uint16_t height, uint32_t totalBytes, uint32_t offset) {
    std::memcpy(out, "DCH1", 4);
    putLe(out + 4, kHdHeaderBytes, 4);
    putLe(out + 8, captureMonotonicUs, 8);
    putLe(out + 16, sequence, 4);
    putLe(out + 20, width, 2);
    putLe(out + 22, height, 2);
    putLe(out + 24, totalBytes, 4);
    putLe(out + 28, offset, 4);
}

struct MonitorHdStats {
    uint64_t submitted   = 0;   ///< Frames copied while an HD viewer watched.
    uint64_t sent        = 0;   ///< Frames whose every chunk was sent.
    uint64_t replaced    = 0;   ///< Overwritten in the slot before the worker took them (latest only).
    uint64_t tooLarge    = 0;   ///< Empty, over kHdMaxJpegBytes, or over 4096 px.
    uint64_t dropBusy    = 0;   ///< EAGAIN on some chunk: the agent's queue is full.
    uint64_t dropMissing = 0;   ///< ENOENT: no agent socket.
    uint64_t dropRefused = 0;   ///< ECONNREFUSED: stale socket, no reader.
    uint64_t dropOther   = 0;   ///< Any other send error.
    uint64_t dropped() const { return tooLarge + dropBusy + dropMissing + dropRefused + dropOther; }
};

/// submit() runs on the camera's preview tap thread; pollDemand() on one thread
/// (the supervisor); open() and stop() on the owner's.
class MonitorHdSender {
public:
    MonitorHdSender() = default;
    MonitorHdSender(const MonitorHdSender&) = delete;
    MonitorHdSender& operator=(const MonitorHdSender&) = delete;
    ~MonitorHdSender() {
        stop();
        if (fd_ >= 0) ::close(fd_);
    }

    /// Nonblocking sender towards @p socketPath plus its worker; allocates both frame buffers once.
    bool open(const std::string& socketPath, const std::string& demandPath) {
        if (fd_ >= 0 || socketPath.empty() || socketPath.size() >= sizeof(address_.sun_path)) return false;
        pending_.reset(new (std::nothrow) uint8_t[kHdMaxJpegBytes]);
        sending_.reset(new (std::nothrow) uint8_t[kHdMaxJpegBytes]);
        if (!pending_ || !sending_) return false;
        address_.sun_family = AF_UNIX;
        std::memcpy(address_.sun_path, socketPath.c_str(), socketPath.size() + 1);
        demandPath_ = demandPath;
        fd_ = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (fd_ < 0) return false;
        // Room for a whole frame in flight: the kernel caps this at wmem_max and doubles it.
        const int want = static_cast<int>(kHdMaxJpegBytes);
        (void)::setsockopt(fd_, SOL_SOCKET, SO_SNDBUF, &want, sizeof(want));
        try {
            worker_ = std::thread([this] { run(); });
        } catch (...) {
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        return true;
    }
    bool enabled() const { return fd_ >= 0; }

    /// Re-read the HD demand file; returns (and caches for submit()) whether an HD viewer watches.
    bool pollDemand() {
        const bool active = fd_ >= 0 && demandFileFresh(demandPath_);
        demanded_.store(active, std::memory_order_relaxed);
        return active;
    }
    bool demanded() const { return demanded_.load(std::memory_order_relaxed); }

    /// Copies one camera JPEG for the worker, only while an HD viewer watches.  Never waits on I/O.
    void submit(const uint8_t* data, size_t size, uint32_t width, uint32_t height, uint64_t captureMonotonicUs,
                uint64_t sequence) {
        if (fd_ < 0 || !demanded_.load(std::memory_order_relaxed)) return;
        if (!data || size == 0 || size > kHdMaxJpegBytes || width == 0 || height == 0 ||
            width > kHdMaxDimension || height > kHdMaxDimension) {
            tooLarge_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopping_) return;
            if (pendingFull_) replaced_.fetch_add(1, std::memory_order_relaxed);
            std::memcpy(pending_.get(), data, size);
            pendingMeta_ = Meta{size, static_cast<uint16_t>(width), static_cast<uint16_t>(height),
                                captureMonotonicUs, static_cast<uint32_t>(sequence)};
            pendingFull_ = true;
        }
        submitted_.fetch_add(1, std::memory_order_relaxed);
        wake_.notify_one();
    }

    /// Stops the worker after the frame it is sending; idempotent.
    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        wake_.notify_one();
        if (worker_.joinable()) worker_.join();
    }

    MonitorHdStats stats() const {
        MonitorHdStats s;
        s.submitted   = submitted_.load(std::memory_order_relaxed);
        s.sent        = sent_.load(std::memory_order_relaxed);
        s.replaced    = replaced_.load(std::memory_order_relaxed);
        s.tooLarge    = tooLarge_.load(std::memory_order_relaxed);
        s.dropBusy    = dropBusy_.load(std::memory_order_relaxed);
        s.dropMissing = dropMissing_.load(std::memory_order_relaxed);
        s.dropRefused = dropRefused_.load(std::memory_order_relaxed);
        s.dropOther   = dropOther_.load(std::memory_order_relaxed);
        return s;
    }

private:
    struct Meta {
        size_t   size = 0;
        uint16_t width = 0, height = 0;
        uint64_t captureUs = 0;
        uint32_t sequence = 0;
    };

    void run() {
        for (;;) {
            Meta meta;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                wake_.wait(lock, [this] { return stopping_ || pendingFull_; });
                if (stopping_) return;
                std::swap(pending_, sending_);
                meta = pendingMeta_;
                pendingFull_ = false;
            }
            sendFrame(meta);
        }
    }

    void sendFrame(const Meta& m) {
        uint8_t header[kHdHeaderBytes];
        // The kernel holds at most ~416 KB in flight (2 x wmem_max), less than a large frame, so a
        // full queue is normal mid-frame: wait (this worker only) for the agent to drain it, up to
        // kHdFrameBudgetMs per frame.  Meanwhile newer frames replace each other in the slot.
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(kHdFrameBudgetMs);
        for (size_t offset = 0; offset < m.size; offset += kHdChunkBytes) {
            const size_t chunk = std::min(kHdChunkBytes, m.size - offset);
            writeHdChunkHeader(header, m.captureUs, m.sequence, m.width, m.height, static_cast<uint32_t>(m.size),
                               static_cast<uint32_t>(offset));
            iovec iov[2] = {{header, kHdHeaderBytes}, {sending_.get() + offset, chunk}};
            msghdr msg{};
            msg.msg_name    = &address_;
            msg.msg_namelen = sizeof(address_);
            msg.msg_iov     = iov;
            msg.msg_iovlen  = 2;
            ssize_t n = ::sendmsg(fd_, &msg, MSG_DONTWAIT | MSG_NOSIGNAL);
            while (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                const auto now = std::chrono::steady_clock::now();
                if (now >= deadline) break;
                // POLLOUT on this unconnected socket covers only our own send buffer.  A full
                // receive queue on the agent's side does not show in it, so poll can return at
                // once while every send still fails: never retry sooner than kHdRetryPauseMs.
                const auto pauseUntil = now + std::chrono::milliseconds(kHdRetryPauseMs);
                const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
                pollfd p{fd_, POLLOUT, 0};
                (void)::poll(&p, 1, static_cast<int>(std::min<long long>(left, kHdRetryPollMs)));
                if (std::chrono::steady_clock::now() < pauseUntil) std::this_thread::sleep_until(std::min(pauseUntil, deadline));
                n = ::sendmsg(fd_, &msg, MSG_DONTWAIT | MSG_NOSIGNAL);
            }
            if (n == static_cast<ssize_t>(kHdHeaderBytes + chunk)) continue;
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) dropBusy_.fetch_add(1, std::memory_order_relaxed);
            else if (n < 0 && errno == ENOENT) dropMissing_.fetch_add(1, std::memory_order_relaxed);
            else if (n < 0 && errno == ECONNREFUSED) dropRefused_.fetch_add(1, std::memory_order_relaxed);
            else dropOther_.fetch_add(1, std::memory_order_relaxed);
            return;   // the rest of this frame is useless to the agent
        }
        sent_.fetch_add(1, std::memory_order_relaxed);
    }

    int fd_ = -1;
    sockaddr_un address_{};
    std::string demandPath_;
    std::unique_ptr<uint8_t[]> pending_, sending_;
    Meta pendingMeta_;
    bool pendingFull_ = false, stopping_ = false;   // guarded by mutex_
    std::mutex mutex_;
    std::condition_variable wake_;
    std::thread worker_;
    std::atomic<bool> demanded_{false};
    std::atomic<uint64_t> submitted_{0}, sent_{0}, replaced_{0}, tooLarge_{0}, dropBusy_{0}, dropMissing_{0},
        dropRefused_{0}, dropOther_{0};
};
} // namespace dashcam::monitor
