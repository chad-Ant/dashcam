#include "../monitoring.h"
#include "../monitoring_video.h"
#include <fcntl.h>
#include <sys/stat.h>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <poll.h>
#include <vector>
#include <jpeglib.h>

namespace {
namespace mon = dashcam::monitor;
namespace pv  = dashcam::preview;

// A filled lease from a one-slot pool, as VideoPreview's worker would offer it.
pv::EncodedFrameLease makeLease(const std::shared_ptr<pv::EncodedFramePool>& pool,
                                const std::vector<uint8_t>& jpeg, uint64_t captureUs, uint64_t seq) {
    pv::EncodedFrameLease lease = pool->tryAcquire();
    assert(lease.valid() && lease.capacity() >= jpeg.size());
    std::memcpy(lease.writableData(), jpeg.data(), jpeg.size());
    assert(lease.setFrame(jpeg.size(), 640, 360, captureUs, seq, 65));
    return lease;
}

void setMtime(const std::string& path, double offsetSec) {
    timespec now{};
    assert(::clock_gettime(CLOCK_REALTIME, &now) == 0);
    const auto off = static_cast<int64_t>(offsetSec * 1e9);
    int64_t ns = static_cast<int64_t>(now.tv_sec) * 1000000000LL + now.tv_nsec + off;
    timespec times[2];
    times[0].tv_sec = times[1].tv_sec = static_cast<time_t>(ns / 1000000000LL);
    times[0].tv_nsec = times[1].tv_nsec = static_cast<long>(ns % 1000000000LL);
    assert(::utimensat(AT_FDCWD, path.c_str(), times, 0) == 0);
}

void testVideo() {
    // Header: byte-exact little-endian layout.
    uint8_t header[mon::kVideoHeaderBytes];
    mon::writeVideoHeader(header, 0x0102030405060708ULL, 0xA1B2C3D4u, 640, 360);
    const uint8_t expected[mon::kVideoHeaderBytes] = {
        'D', 'C', 'V', '1', 24, 0, 0, 0,
        0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
        0xD4, 0xC3, 0xB2, 0xA1, 0x80, 0x02, 0x68, 0x01};
    assert(std::memcmp(header, expected, sizeof(expected)) == 0);

    const std::string base = "/tmp/dashcam-monitor-video-test-" + std::to_string(::getpid());
    const std::string sockPath = base + ".sock", demandPath = base + ".demand", dirPath = base + ".dir";
    mon::MonitorVideoSink sink;
    assert(!sink.demand().active);
    assert(sink.open(sockPath, demandPath));
    assert(!sink.open(sockPath, demandPath));   // once
    const pv::PreviewDemand d0 = sink.demand();
    assert(!d0.active && d0.canvasW == 720 && d0.canvasH == 406 && d0.maxJpegBytes == 0);

    // Demand file: missing / fresh / 4 s old / future / a directory.
    assert(!sink.pollDemand() && !sink.demand().active);
    FILE* f = std::fopen(demandPath.c_str(), "w");
    assert(f != nullptr);
    std::fclose(f);
    assert(sink.pollDemand() && sink.demand().active);
    setMtime(demandPath, -2.5);
    assert(sink.pollDemand());
    setMtime(demandPath, -4.0);
    assert(!sink.pollDemand() && !sink.demand().active);
    setMtime(demandPath, +4.0);
    assert(!sink.pollDemand());
    setMtime(demandPath, 0.0);
    assert(sink.pollDemand());
    assert(::unlink(demandPath.c_str()) == 0);
    assert(!sink.pollDemand());
    assert(::mkdir(dirPath.c_str(), 0700) == 0);
    mon::MonitorVideoSink dirSink;
    assert(dirSink.open(sockPath, dirPath));
    assert(!dirSink.pollDemand());
    assert(::rmdir(dirPath.c_str()) == 0);
    struct stat st{};
    st.st_mode = S_IFREG | 0600;
    st.st_mtim.tv_sec = 1000;
    st.st_mtim.tv_nsec = 0;
    assert(mon::demandFresh(st, timespec{1003, 0}) && !mon::demandFresh(st, timespec{1003, 1}));
    assert(mon::demandFresh(st, timespec{997, 0}) && !mon::demandFresh(st, timespec{996, 999999999}));

    // A small "JPEG" (the sink forwards bytes; it does not decode).
    std::vector<uint8_t> jpeg(30000);
    for (size_t i = 0; i < jpeg.size(); ++i) jpeg[i] = static_cast<uint8_t>(i * 31u + 7u);
    jpeg[0] = 0xFF; jpeg[1] = 0xD8; jpeg[jpeg.size() - 2] = 0xFF; jpeg[jpeg.size() - 1] = 0xD9;
    auto pool = pv::EncodedFramePool::create(1, mon::kVideoMaxJpegBytes);
    assert(pool);

    // No receiver: returns at once, consumes the lease, counts a drop.
    auto start = std::chrono::steady_clock::now();
    assert(sink.offerFrame(makeLease(pool, jpeg, 123, 1)));
    assert(std::chrono::steady_clock::now() - start < std::chrono::milliseconds(100));
    assert(pool->freeSlots() == 1);
    assert(sink.stats().sent == 0 && sink.stats().dropMissing == 1 && sink.stats().dropped() == 1);

    // Bound receiver: one datagram, byte-exact.
    const int receiver = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    assert(receiver >= 0);
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, sockPath.c_str(), sockPath.size() + 1);
    assert(::bind(receiver, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) == 0);
    const uint64_t captureUs = 987654321012ULL;
    const uint64_t seq = (1ULL << 32) + 77;   // truncated to 77
    assert(sink.offerFrame(makeLease(pool, jpeg, captureUs, seq)));
    assert(pool->freeSlots() == 1);
    std::vector<uint8_t> rx(mon::kVideoHeaderBytes + mon::kVideoMaxJpegBytes + 16);
    const auto got = ::recv(receiver, rx.data(), rx.size(), 0);
    assert(got == static_cast<ssize_t>(mon::kVideoHeaderBytes + jpeg.size()));
    uint8_t want[mon::kVideoHeaderBytes];
    mon::writeVideoHeader(want, captureUs, 77, 640, 360);
    assert(std::memcmp(rx.data(), want, sizeof(want)) == 0);
    assert(std::memcmp(rx.data() + mon::kVideoHeaderBytes, jpeg.data(), jpeg.size()) == 0);
    assert(sink.stats().sent == 1);

    // Stalled receiver: 1000 offers never wait; the full queue counts as busy.
    start = std::chrono::steady_clock::now();
    for (int i = 0; i < 1000; ++i) assert(sink.offerFrame(makeLease(pool, jpeg, captureUs + i, seq + i)));
    assert(std::chrono::steady_clock::now() - start < std::chrono::seconds(1));
    const mon::MonitorVideoStats vs = sink.stats();
    assert(vs.sent + vs.dropped() == 1002 && vs.dropBusy > 0);

    // Stale socket file with no reader: refused, still nonblocking.
    ::close(receiver);
    assert(sink.offerFrame(makeLease(pool, jpeg, captureUs, seq)));
    assert(sink.stats().dropRefused == 1);
    assert(::unlink(sockPath.c_str()) == 0);

    // Oversized or empty leases are consumed and counted, never sent.
    auto big = pv::EncodedFramePool::create(1, mon::kVideoMaxJpegBytes + 1);
    assert(big);
    std::vector<uint8_t> huge(mon::kVideoMaxJpegBytes + 1, 0xAB);
    const uint64_t otherBefore = sink.stats().dropOther;
    assert(sink.offerFrame(makeLease(big, huge, 1, 1)));
    assert(sink.offerFrame(pv::EncodedFrameLease{}));
    assert(sink.stats().dropOther == otherBefore + 2 && big->freeSlots() == 1);
}
// A 1920x1080 gradient, like a camera frame, compressed with libjpeg.
std::vector<uint8_t> cameraJpeg() {
    constexpr int kW = 1920, kH = 1080;
    jpeg_compress_struct ci;
    jpeg_error_mgr jerr;
    ci.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&ci);
    unsigned char* mem = nullptr;
    unsigned long memSize = 0;
    jpeg_mem_dest(&ci, &mem, &memSize);
    ci.image_width = kW;
    ci.image_height = kH;
    ci.input_components = 3;
    ci.in_color_space = JCS_RGB;
    jpeg_set_defaults(&ci);
    jpeg_set_quality(&ci, 80, TRUE);
    jpeg_start_compress(&ci, TRUE);
    std::vector<uint8_t> row(kW * 3);
    while (ci.next_scanline < ci.image_height) {
        for (int x = 0; x < kW; ++x) {
            row[x * 3 + 0] = static_cast<uint8_t>(x * 255 / kW);
            row[x * 3 + 1] = static_cast<uint8_t>(ci.next_scanline * 255 / kH);
            row[x * 3 + 2] = static_cast<uint8_t>((x + ci.next_scanline) & 0xFF);
        }
        JSAMPROW rp = row.data();
        jpeg_write_scanlines(&ci, &rp, 1);
    }
    jpeg_finish_compress(&ci);
    std::vector<uint8_t> out(mem, mem + memSize);
    jpeg_destroy_compress(&ci);
    std::free(mem);
    return out;
}

// The v0.4 wiring: camera JPEG -> VideoPreview (monitor settings) -> sink ->
// one SD (720x404) datagram while the demand file is fresh, nothing once it is gone.
void testPreviewToAgent() {
    const std::string base = "/tmp/dashcam-monitor-e2e-test-" + std::to_string(::getpid());
    const std::string sockPath = base + ".sock", demandPath = base + ".demand";
    const int receiver = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    assert(receiver >= 0);
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, sockPath.c_str(), sockPath.size() + 1);
    assert(::bind(receiver, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) == 0);
    FILE* f = std::fopen(demandPath.c_str(), "w");
    assert(f != nullptr);
    std::fclose(f);

    mon::MonitorVideoSink sink;
    assert(sink.open(sockPath, demandPath) && sink.pollDemand());
    pv::VideoPreview preview;
    pv::PreviewConfig pc;
    pc.targetFps = 10;
    pc.jpegQuality = 65;
    pc.minimumJpegQuality = 45;
    pc.maxJpegBytes = mon::kVideoMaxJpegBytes;
    assert(preview.start(pc, sink));
    const std::vector<uint8_t> jpeg = cameraJpeg();
    const uint64_t captureUs = 5000000;
    const pv::SubmitResult r = preview.submitJpeg(jpeg.data(), jpeg.size(), 1920, 1080, captureUs);
    assert(r == pv::SubmitResult::Accepted);
    pollfd pfd{receiver, POLLIN, 0};
    assert(::poll(&pfd, 1, 3000) == 1);
    std::vector<uint8_t> rx(mon::kVideoHeaderBytes + mon::kVideoMaxJpegBytes + 16);
    const auto got = ::recv(receiver, rx.data(), rx.size(), 0);
    assert(got > static_cast<ssize_t>(mon::kVideoHeaderBytes + 4) &&
           got <= static_cast<ssize_t>(mon::kVideoHeaderBytes + mon::kVideoMaxJpegBytes));
    uint8_t want[mon::kVideoHeaderBytes];
    mon::writeVideoHeader(want, captureUs, 1, 720, 404);
    assert(std::memcmp(rx.data(), want, sizeof(want)) == 0);
    const size_t end = static_cast<size_t>(got);
    assert(rx[24] == 0xFF && rx[25] == 0xD8 && rx[end - 2] == 0xFF && rx[end - 1] == 0xD9);

    // Viewer gone: the converter does no work at all.
    assert(::unlink(demandPath.c_str()) == 0);
    assert(!sink.pollDemand());
    assert(preview.submitJpeg(jpeg.data(), jpeg.size(), 1920, 1080, captureUs + 1000000) ==
           pv::SubmitResult::Inactive);
    preview.stop();
    assert(sink.stats().sent == 1 && sink.stats().dropped() == 0);
    ::close(receiver);
    assert(::unlink(sockPath.c_str()) == 0);
}

// One HD frame as the agent would reassemble it from DCH1 chunks; empty on any gap.
std::vector<uint8_t> receiveHdFrame(int receiver, uint64_t wantCapture, uint32_t wantSeq, uint16_t w, uint16_t h) {
    std::vector<uint8_t> frame, rx(mon::kHdHeaderBytes + mon::kHdChunkBytes + 16);
    for (;;) {
        pollfd p{receiver, POLLIN, 0};
        if (::poll(&p, 1, 2000) != 1) return {};
        const auto got = ::recv(receiver, rx.data(), rx.size(), 0);
        assert(got > static_cast<ssize_t>(mon::kHdHeaderBytes));
        auto le = [&](size_t at, size_t bytes) {
            uint64_t v = 0;
            for (size_t i = 0; i < bytes; ++i) v |= static_cast<uint64_t>(rx[at + i]) << (8 * i);
            return v;
        };
        assert(std::memcmp(rx.data(), "DCH1", 4) == 0 && le(4, 4) == mon::kHdHeaderBytes);
        assert(le(8, 8) == wantCapture && le(16, 4) == wantSeq && le(20, 2) == w && le(22, 2) == h);
        const size_t total = le(24, 4), offset = le(28, 4);
        const size_t chunk = static_cast<size_t>(got) - mon::kHdHeaderBytes;
        assert(chunk <= mon::kHdChunkBytes && offset == frame.size() && offset + chunk <= total);
        frame.insert(frame.end(), rx.begin() + mon::kHdHeaderBytes, rx.begin() + got);
        if (frame.size() == total) return frame;
    }
}

void testHd() {
    uint8_t header[mon::kHdHeaderBytes];
    mon::writeHdChunkHeader(header, 0x0102030405060708ULL, 0xA1B2C3D4u, 1920, 1080, 0x00030201u, 0x0000F000u);
    const uint8_t expected[mon::kHdHeaderBytes] = {
        'D', 'C', 'H', '1', 32, 0, 0, 0,
        0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
        0xD4, 0xC3, 0xB2, 0xA1, 0x80, 0x07, 0x38, 0x04,
        0x01, 0x02, 0x03, 0x00, 0x00, 0xF0, 0x00, 0x00};
    assert(std::memcmp(header, expected, sizeof(expected)) == 0);

    const std::string base = "/tmp/dashcam-monitor-hd-test-" + std::to_string(::getpid());
    const std::string sockPath = base + ".sock", demandPath = base + ".demand";
    const int receiver = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    assert(receiver >= 0);
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, sockPath.c_str(), sockPath.size() + 1);
    assert(::bind(receiver, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) == 0);

    std::vector<uint8_t> jpeg(150001);   // a typical 1080p MJPEG frame: three chunks, the last one short
    for (size_t i = 0; i < jpeg.size(); ++i) jpeg[i] = static_cast<uint8_t>(i * 131u + 3u);
    jpeg[0] = 0xFF; jpeg[1] = 0xD8; jpeg[jpeg.size() - 2] = 0xFF; jpeg[jpeg.size() - 1] = 0xD9;

    mon::MonitorHdSender hd;
    assert(hd.open(sockPath, demandPath));
    assert(!hd.open(sockPath, demandPath));   // once

    // No demand: nothing is copied or sent.
    assert(!hd.pollDemand());
    hd.submit(jpeg.data(), jpeg.size(), 1920, 1080, 1000, 1);
    pollfd p{receiver, POLLIN, 0};
    assert(::poll(&p, 1, 200) == 0 && hd.stats().submitted == 0);

    // Demand: the original bytes arrive in order, reassembled byte for byte.
    FILE* f = std::fopen(demandPath.c_str(), "w");
    assert(f != nullptr);
    std::fclose(f);
    assert(hd.pollDemand() && hd.demanded());
    const uint64_t captureUs = 987654321012ULL;
    const auto start = std::chrono::steady_clock::now();
    hd.submit(jpeg.data(), jpeg.size(), 1920, 1080, captureUs, (1ULL << 32) + 9);   // sequence truncated to 9
    assert(std::chrono::steady_clock::now() - start < std::chrono::milliseconds(50));   // a copy, not the send
    assert(receiveHdFrame(receiver, captureUs, 9, 1920, 1080) == jpeg);
    for (int i = 0; i < 100 && hd.stats().sent < 1; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    assert(hd.stats().sent == 1 && hd.stats().submitted == 1 && hd.stats().dropped() == 0);

    // Rejected before the copy: empty, oversized, absurd geometry.
    hd.submit(jpeg.data(), 0, 1920, 1080, captureUs, 10);
    std::vector<uint8_t> huge(mon::kHdMaxJpegBytes + 1, 0xAB);
    hd.submit(huge.data(), huge.size(), 1920, 1080, captureUs, 11);
    hd.submit(jpeg.data(), jpeg.size(), 8192, 1080, captureUs, 12);
    assert(hd.stats().tooLarge == 3 && hd.stats().submitted == 1);

    // Stalled reader: 500 frames never block the camera's thread; the busy
    // queue and the latest-only slot account for every frame not sent.
    const auto burst = std::chrono::steady_clock::now();
    for (int i = 0; i < 500; ++i) hd.submit(jpeg.data(), jpeg.size(), 1920, 1080, captureUs + i, 100 + i);
    assert(std::chrono::steady_clock::now() - burst < std::chrono::seconds(1));
    // Keep offering, slower, until the worker has met the full queue.
    for (int i = 0; i < 2000 && hd.stats().dropBusy == 0; ++i) {
        hd.submit(jpeg.data(), jpeg.size(), 1920, 1080, captureUs + 500 + i, 600 + i);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    hd.stop();
    const mon::MonitorHdStats hs = hd.stats();
    assert(hs.submitted >= 501 && hs.dropBusy > 0);
    assert(hs.sent + hs.replaced + hs.dropBusy + hs.dropMissing + hs.dropRefused + hs.dropOther <= hs.submitted);
    hd.stop();   // idempotent

    ::close(receiver);
    assert(::unlink(sockPath.c_str()) == 0);
    assert(::unlink(demandPath.c_str()) == 0);
}
double processCpuMs() {
    timespec t{};
    ::clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t);
    return t.tv_sec * 1e3 + t.tv_nsec / 1e6;
}

// Review finding (2026-11-26): with the agent's receive queue full but the sender's own buffer
// empty, POLLOUT is ready at once while every send fails, so a poll-and-retry loop spins a core for
// the whole frame budget.  The worker must instead pause between retries, give up at the deadline,
// count the drop, and deliver a complete fresh frame once the reader drains the queue.
void testHdQueueFullDoesNotSpin() {
    const std::string base = "/tmp/dashcam-monitor-hdfull-test-" + std::to_string(::getpid());
    const std::string sockPath = base + ".sock", demandPath = base + ".demand";
    const int receiver = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    assert(receiver >= 0);
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, sockPath.c_str(), sockPath.size() + 1);
    assert(::bind(receiver, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) == 0);
    FILE* f = std::fopen(demandPath.c_str(), "w");
    assert(f != nullptr);
    std::fclose(f);

    // Fill the receiver's queue with tiny datagrams from other sockets until a fresh socket (empty
    // send buffer, like the HD sender's) is refused: the queue, not the sender, is now the limit.
    std::vector<int> fillers;
    auto refusedFromFreshSocket = [&] {
        const int probe = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        const char b = 0;
        const bool refused = ::sendto(probe, &b, 1, MSG_DONTWAIT, reinterpret_cast<const sockaddr*>(&address),
                                      sizeof(address)) < 0 && errno == EAGAIN;
        ::close(probe);
        return refused;
    };
    while (!refusedFromFreshSocket()) {
        assert(fillers.size() < 256);
        const int s = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        fillers.push_back(s);
        const char b = 1;
        while (::sendto(s, &b, 1, MSG_DONTWAIT, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) == 1) {}
    }
    pollfd p{0, POLLOUT, 0};
    {
        const int fresh = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        p.fd = fresh;
        assert(::poll(&p, 1, 0) == 1 && (p.revents & POLLOUT));   // the trap: "writable", yet every send fails
        ::close(fresh);
    }

    std::vector<uint8_t> jpeg(150001, 0x5A);
    jpeg[0] = 0xFF; jpeg[1] = 0xD8; jpeg[jpeg.size() - 2] = 0xFF; jpeg[jpeg.size() - 1] = 0xD9;
    mon::MonitorHdSender hd;
    assert(hd.open(sockPath, demandPath) && hd.pollDemand());
    const double cpu0 = processCpuMs();
    const auto wall0 = std::chrono::steady_clock::now();
    hd.submit(jpeg.data(), jpeg.size(), 1920, 1080, 1000, 1);
    for (int i = 0; i < 100 && hd.stats().dropBusy == 0; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    const auto wallMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall0).count();
    const double cpuMs = processCpuMs() - cpu0;   // this thread sleeps: almost all of it is the worker
    std::printf("  queue full: frame given up after %.0f ms wall, %.1f ms CPU (a spin would use ~%d ms)\n",
                wallMs, cpuMs, mon::kHdFrameBudgetMs);
    assert(hd.stats().dropBusy == 1 && hd.stats().sent == 0);
    assert(wallMs >= mon::kHdFrameBudgetMs * 0.8 && wallMs < mon::kHdFrameBudgetMs + 400);   // bounded by the deadline
    assert(cpuMs < mon::kHdFrameBudgetMs * 0.3);                                            // no spin

    // The reader catches up: a fresh frame arrives complete.
    std::vector<uint8_t> rx(mon::kHdHeaderBytes + mon::kHdChunkBytes + 16);
    while (::recv(receiver, rx.data(), rx.size(), MSG_DONTWAIT) >= 0) {}
    for (int s : fillers) ::close(s);
    jpeg[100] = 0x11;
    hd.submit(jpeg.data(), jpeg.size(), 1920, 1080, 2000, 2);
    assert(receiveHdFrame(receiver, 2000, 2, 1920, 1080) == jpeg);
    for (int i = 0; i < 100 && hd.stats().sent < 1; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    assert(hd.stats().sent == 1);
    hd.stop();
    ::close(receiver);
    assert(::unlink(sockPath.c_str()) == 0);
    assert(::unlink(demandPath.c_str()) == 0);
}
} // namespace

int main() {
    hostproto::Telemetry telemetry{};
    telemetry.speed = 42;
    telemetry.sigSource = static_cast<uint8_t>(hostproto::VehSigSource::CAN_SNIFF);
    telemetry.accel = std::numeric_limits<float>::quiet_NaN();
    const auto live = dashcam::monitor::snapshot(telemetry, true, 10, true, 100, 30, 1);
    assert(live.find("\"speed_kmh\":42") != std::string::npos);
    assert(live.find("\"speed_source\":\"CAN\"") != std::string::npos);
    assert(live.find("\"acceleration_ms2\":null") != std::string::npos);
    assert(live.find("\"driver_available\":false") != std::string::npos);
    const auto stale = dashcam::monitor::snapshot(telemetry, false, 3000, false, 0, -1, 2);
    assert(stale.find("\"speed_kmh\":null") != std::string::npos);
    assert(stale.find("\"fps\":null") != std::string::npos);
    telemetry.speed = std::numeric_limits<float>::infinity();
    assert(dashcam::monitor::snapshot(telemetry, true, 10, true, 100, 30, 3)
           .find("\"speed_kmh\":null") != std::string::npos);

    const std::string path = "/tmp/dashcam-monitor-test-" + std::to_string(::getpid()) + ".sock";
    dashcam::monitor::Publisher publisher;
    assert(publisher.open(path));
    assert(!publisher.offer(live)); // missing agent does not wait
    const int receiver = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    assert(receiver >= 0);
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
    assert(::bind(receiver, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) == 0);
    assert(publisher.offer(live));
    char buffer[16384];
    const auto bytes = ::recv(receiver, buffer, sizeof(buffer), 0);
    assert(bytes == static_cast<ssize_t>(live.size()));
    assert(std::string(buffer, static_cast<size_t>(bytes)) == live);
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < 10000; ++i) publisher.offer(live); // stopped reader fills queue
    assert(std::chrono::steady_clock::now() - start < std::chrono::seconds(2));
    ::close(receiver);
    assert(::unlink(path.c_str()) == 0);
    assert(!publisher.offer(live));
    std::puts("monitoring: serialization, stale values, missing/stalled agent and socket delivery passed");
    testVideo();
    testPreviewToAgent();
    std::puts("monitoring video: header layout, demand file freshness, missing/stalled/refused agent and "
              "byte-exact delivery, camera JPEG -> SD 720x404 datagram on demand passed");
    testHd();
    testHdQueueFullDoesNotSpin();
    std::puts("monitoring HD: chunk header layout, demand gate, byte-exact chunked delivery, size limits and "
              "a stalled agent never blocking the camera thread passed");
}
