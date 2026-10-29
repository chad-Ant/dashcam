#pragma once
// Best-effort camera preview for the host monitoring agent (connector/README.md,
// "Camera video").  A second VideoPreview converts the recording camera's JPEGs
// to 640x360 while a viewer watches; this sink sends each one as ONE nonblocking
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
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

namespace dashcam::monitor {
constexpr size_t   kVideoHeaderBytes   = 24;
constexpr size_t   kVideoMaxJpegBytes  = 65536;   ///< = the monitor preview's PreviewConfig::maxJpegBytes.
constexpr uint16_t kVideoCanvasWidth   = 640;
constexpr uint16_t kVideoCanvasHeight  = 360;
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
} // namespace dashcam::monitor
