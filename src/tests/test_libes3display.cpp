// es3display_test — Es3DisplayLink (libes3display + libuart) against a fake
// ES3C28P on a pseudo-terminal: no board, no camera, no files.
//
// The fake display owns the master side of a pty and the link opens the slave
// through a symlink in a temp dir, exactly as it opens /dev/dashcam-display.
// Unplugging is closing the master (the slave sees a hangup, as with a USB
// detach); plugging back in is a new pty behind the re-pointed symlink.  The
// fake is written from the frozen wire contract (Es3Protocol.h +
// Es3VideoProtocol.h; peripherals/es3c28p/VIDEO_STREAMING_PLAN.md "Frozen
// protocol v1"), not from the firmware: it answers HELLO, PING, STATUS, PAGE,
// ALERT, STOP_AUDIO, CAPABILITIES (or BadCommand / NotReady / an unusable
// reply), START_VIDEO and STOP_VIDEO; it assembles START/CHUNK/END with exact
// offsets, checks the whole-frame CRC-32 with its own bitwise implementation,
// decodes every JPEG with libjpeg, and answers RESULT, PRESENTED, CREDIT and
// VISIBILITY.  Fault knobs: a dropped reply, withheld credits, stale-token
// records, garbage bytes, a reader that stalls or is rate-limited, hidden /
// calibrating visibility, session expiry, unplug/replug.
//
// A pty's TIOCOUTQ always reads 0, so this binary interposes write(), ioctl()
// and tcflush() for the one slave the fake owns (matched by st_rdev): TIOCOUTQ
// then reports the bytes the link wrote that the fake has not read yet — what
// cdc-acm reports for URBs the device has not taken — and the suite can bound
// the real kernel queue the link builds.
//
// Covers: config validation and lifecycle (absent device, restart); the
// handshake, identity and negotiated limits; START_VIDEO fields and a viewport
// change; old firmware -> control-only with working controls; frames flowing
// and presented (count, size, CRC, decode); newest-frame replacement; credits
// vs grant and maxFramesInFlight 1 and 2; credit stall -> resync; stale-session
// records; garbage resync; hidden / calibrating visibility; control latency
// under video load (control priority) and the bounded kernel queue; a stalled
// reader that never blocks offerFrame()/enqueueControl(); heartbeats; reply
// timeout, NoSession and a lost HELLO reply; unplug/replug with every lease
// released; stop() bounded with traffic in flight; and end to end, a 1080p
// JPEG through the real VideoPreview converter to a 320x180 frame on the fake.
//
// [19] repeats the core cases against es3_firmware_fake.h: the colleague's
// firmware VideoState.h and parser, with es3UsbLink's service rules mirrored.
// [20] the map surface (extension v1), [21] board-side panning (extension v2:
// PAN_CAPABILITIES, START_PAN, background/chrome images, MAP_POSE rate and
// interleaving, fallback), [22] NavigationService -> link -> fake board.
//
// Usage: es3display_test            (ES3_TEST_VERBOSE=1 prints the link's log)
//        es3display_test --fake-board PATH   (that firmware-logic fake behind the
//                                             symlink PATH, for running the real app)
//        es3display_test --fake-map-board PATH [pan]   (the contract fake with the
//                                             map extension [and panning], tabs cycling)

#include "libes3display.h"
#include "libvideopreview.h"

#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <atomic>
#include <chrono>
#include <csetjmp>
#include <csignal>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <jpeglib.h>

namespace fs = std::filesystem;
using namespace dashcam::es3display;
using dashcam::log::LogLevel;
using dashcam::preview::EncodedFrameLease;
using dashcam::preview::EncodedFramePool;
using Clock = std::chrono::steady_clock;

static int g_fails = 0;
static void check(bool ok, const std::string& what) {
    std::printf("  %s  %s\n", ok ? "ok  " : "FAIL", what.c_str());
    if (!ok) ++g_fails;
}

static int64_t nowUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now().time_since_epoch()).count();
}
static int64_t nowMs() { return nowUs() / 1000; }
static void sleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

static bool waitFor(const std::function<bool()>& pred, int ms) {
    const int64_t end = nowMs() + ms;
    while (nowMs() < end) {
        if (pred()) return true;
        sleepMs(2);
    }
    return pred();
}

// ─── emulated kernel output queue (TIOCOUTQ) for the fake's slave ─────────────

namespace outq {
std::atomic<uint64_t> armedRdev{0};   ///< st_rdev of the slave in use; 0 = off.
std::atomic<int64_t>  written{0};     ///< Bytes the link wrote to it.
std::atomic<int64_t>  consumed{0};    ///< Bytes the fake read from the master.
std::atomic<int64_t>  flushed{0};     ///< Bytes discarded by TCOFLUSH.
std::atomic<int64_t>  maxQueued{0};   ///< Largest queue right after a write.

int64_t queued() { return std::max<int64_t>(0, written.load() - consumed.load() - flushed.load()); }
bool armed(int fd) {
    const uint64_t rdev = armedRdev.load();
    if (rdev == 0 || fd < 0) return false;
    struct stat sb;
    if (::fstat(fd, &sb) != 0) return false;
    return S_ISCHR(sb.st_mode) && uint64_t(sb.st_rdev) == rdev;
}
void arm(uint64_t rdev) {
    written = consumed = flushed = 0;
    maxQueued = 0;
    armedRdev = rdev;
}
void resetMax() { maxQueued = queued(); }
} // namespace outq

namespace wakeRace {
thread_local bool intercept = false;
std::atomic<bool> entered{false}, resume{false};
std::atomic<int> fd{-1};
}

extern "C" ssize_t write(int fd, const void* buf, size_t n) {
    if (wakeRace::intercept && n == sizeof(uint64_t)) {
        wakeRace::intercept = false;
        wakeRace::fd = fd;
        wakeRace::entered = true;
        while (!wakeRace::resume.load()) std::this_thread::yield();
    }
    const ssize_t r = static_cast<ssize_t>(::syscall(SYS_write, fd, buf, n));
    if (r > 0 && outq::armed(fd)) {
        outq::written += r;
        const int64_t q = outq::queued();
        int64_t       m = outq::maxQueued.load();
        while (q > m && !outq::maxQueued.compare_exchange_weak(m, q)) {}
    }
    return r;
}

extern "C" int ioctl(int fd, unsigned long req, ...) noexcept {
    va_list ap;
    va_start(ap, req);
    void* arg = va_arg(ap, void*);
    va_end(ap);
    if (req == TIOCOUTQ && outq::armed(fd)) {
        *static_cast<int*>(arg) = static_cast<int>(outq::queued());
        return 0;
    }
    return static_cast<int>(::syscall(SYS_ioctl, fd, req, arg));
}

extern "C" int tcflush(int fd, int selector) noexcept {
    // A pty discards the unread bytes on TCOFLUSH; so does the emulated queue.
    if ((selector == TCOFLUSH || selector == TCIOFLUSH) && outq::armed(fd)) outq::flushed += outq::queued();
    return static_cast<int>(::syscall(SYS_ioctl, fd, TCFLSH, selector));
}

// ─── log capture ─────────────────────────────────────────────────────────────

struct LogSink {
    std::mutex               m;
    std::vector<std::string> lines;
    bool                     verbose = std::getenv("ES3_TEST_VERBOSE") != nullptr;
    dashcam::log::LogCallback callback() {
        return [this](LogLevel lvl, const std::string& msg) {
            const char* tag = lvl == LogLevel::ERROR ? "E " : lvl == LogLevel::WARN ? "W " : lvl == LogLevel::INFO ? "I " : "D ";
            if (verbose) std::printf("      | %s%s\n", tag, msg.c_str());
            std::lock_guard<std::mutex> lk(m);
            lines.push_back(tag + msg);
        };
    }
    int count(const std::string& prefixAndNeedle) {
        std::lock_guard<std::mutex> lk(m);
        return static_cast<int>(std::count_if(lines.begin(), lines.end(), [&](const std::string& l) {
            return l.find(prefixAndNeedle) != std::string::npos;
        }));
    }
};

// ─── JPEG fixtures and an independent CRC-32 ──────────────────────────────────

/// Bitwise CRC-32/ISO-HDLC, deliberately not the protocol header's table version.
static uint32_t refCrc32(const uint8_t* d, size_t n) {
    uint32_t c = 0xffffffffu;
    for (size_t i = 0; i < n; ++i) {
        c ^= d[i];
        for (int k = 0; k < 8; ++k) c = (c >> 1) ^ (0xedb88320u & (0u - (c & 1u)));
    }
    return ~c;
}

struct Jpeg {
    std::vector<uint8_t> bytes;
    uint16_t             w = 0, h = 0;
};

using Pattern = std::function<void(uint32_t x, uint32_t y, uint8_t rgb[3])>;

static Jpeg makeJpeg(uint32_t w, uint32_t h, const Pattern& pat, int quality) {
    jpeg_compress_struct ci;
    jpeg_error_mgr       jerr;
    ci.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&ci);
    unsigned char* mem = nullptr;
    unsigned long  memSize = 0;
    jpeg_mem_dest(&ci, &mem, &memSize);
    ci.image_width      = w;
    ci.image_height     = h;
    ci.input_components = 3;
    ci.in_color_space   = JCS_RGB;
    jpeg_set_defaults(&ci);
    jpeg_set_quality(&ci, quality, TRUE);
    jpeg_start_compress(&ci, TRUE);
    std::vector<uint8_t> row(size_t(w) * 3);
    while (ci.next_scanline < h) {
        for (uint32_t x = 0; x < w; ++x) pat(x, ci.next_scanline, &row[size_t(x) * 3]);
        JSAMPROW rp = row.data();
        jpeg_write_scanlines(&ci, &rp, 1);
    }
    jpeg_finish_compress(&ci);
    Jpeg out;
    out.bytes.assign(mem, mem + memSize);
    out.w = static_cast<uint16_t>(w);
    out.h = static_cast<uint16_t>(h);
    std::free(mem);
    jpeg_destroy_compress(&ci);
    return out;
}

static Pattern gradient(int seed) {
    return [=](uint32_t x, uint32_t y, uint8_t* p) {
        p[0] = uint8_t(x + seed * 40);
        p[1] = uint8_t(y * 2 + seed * 20);
        p[2] = uint8_t((x + y) / 2);
    };
}

static Pattern noise(int amp, uint32_t seed) {
    return [=](uint32_t x, uint32_t y, uint8_t* p) {
        uint32_t s = (x * 73856093u) ^ (y * 19349663u) ^ seed;
        for (int c = 0; c < 3; ++c) {
            s = s * 1103515245u + 12345u;
            const int v = 128 + int((s >> 16) % uint32_t(2 * amp + 1)) - amp;
            p[c] = uint8_t(std::clamp(v, 0, 255));
        }
    };
}

/// A 320x180 noise frame between 30 and 60 KB (a hard-to-compress night scene).
static Jpeg bigFrame(uint32_t seed) {
    Jpeg j;
    for (int q = 95; q >= 30; q -= 5) {
        j = makeJpeg(320, 180, noise(110, seed), q);
        if (j.bytes.size() <= 60000) break;
    }
    return j;
}

struct JpegErr {
    jpeg_error_mgr pub;
    jmp_buf        jb;
};
static void jpegErrExit(j_common_ptr c) { std::longjmp(reinterpret_cast<JpegErr*>(c->err)->jb, 1); }
static void jpegQuiet(j_common_ptr) {}

/// Full decode; @p row must hold width*3 bytes (allocated before setjmp).
static bool decodeJpeg(const uint8_t* d, size_t n, std::vector<uint8_t>& row, int& w, int& h) {
    jpeg_decompress_struct ci;
    JpegErr                je;
    ci.err                 = jpeg_std_error(&je.pub);
    je.pub.error_exit      = jpegErrExit;
    je.pub.output_message  = jpegQuiet;
    if (setjmp(je.jb)) {
        jpeg_destroy_decompress(&ci);
        return false;
    }
    jpeg_create_decompress(&ci);
    jpeg_mem_src(&ci, d, static_cast<unsigned long>(n));
    jpeg_read_header(&ci, TRUE);
    jpeg_start_decompress(&ci);
    if (size_t(ci.output_width) * ci.output_components > row.size()) {
        jpeg_destroy_decompress(&ci);
        return false;
    }
    while (ci.output_scanline < ci.output_height) {
        JSAMPROW rp = row.data();
        jpeg_read_scanlines(&ci, &rp, 1);
    }
    w = int(ci.output_width);
    h = int(ci.output_height);
    jpeg_finish_decompress(&ci);
    jpeg_destroy_decompress(&ci);
    return true;
}

// ─── the fake ES3C28P ─────────────────────────────────────────────────────────

struct FakeFrame {
    bool     map = false;   ///< Assembled under a map session.
    uint8_t  panKind = 0;   ///< es3map::PanImage in a panning session.
    uint32_t token = 0, frameId = 0, bytes = 0;
    uint16_t w = 0, h = 0;
    uint64_t ts = 0;
    bool     decodeOk = false;
    int      decW = 0, decH = 0;
};

struct FakeCounters {
    int hello = 0, ping = 0, status = 0, page = 0, alert = 0, stopAudio = 0, caps = 0, startVideo = 0,
        startVideoRejected = 0, stopVideo = 0, noSession = 0, badCommand = 0, badPayload = 0, droppedReplies = 0,
        sessionExpiries = 0;
    int starts = 0, framesOk = 0, presented = 0, crcErrors = 0, badChunks = 0, badLength = 0, decodeFails = 0,
        dimMismatch = 0, noCredit = 0, staleFromHost = 0, orphan = 0, aborts = 0, assemblyTimeouts = 0,
        rejectedStarts = 0, startDuringAssembly = 0, withheldCredits = 0, startsWhileHidden = 0;
    int64_t  bytesWhileHidden = 0, videoBytes = 0;
    int mapCaps = 0, startMap = 0, startMapRejected = 0, stopMap = 0, busyRefusals = 0;
    int panCaps = 0, startPan = 0, startPanRefused = 0, poses = 0, poseSeqErrors = 0, poseBadFields = 0,
        posesWhileHidden = 0, posesOutsideSession = 0;
    es3map::MapPose lastPose;
    int64_t         firstPoseMs = 0, lastPoseMs = 0;
    int64_t         minPoseGapMs = 1 << 30;
    int      maxInFlight = 0;
    size_t   maxChunk = 0;
    uint64_t lastStartTs = 0;
    std::string lastPageTitle;
};

class FakeDisplay {
public:
    // Knobs (set by the tests at any time).
    std::atomic<int>     capsMode{0};        ///< 0 video, 1 BadCommand, 2 NotReady, 3 unusable reply.
    std::atomic<int>     devMaxChunk{4096};
    std::atomic<int>     devMaxJpeg{65536};
    std::atomic<int>     devMaxCredits{1};
    std::atomic<int>     initialGrant{1};
    std::atomic<bool>    duplicateOk{false}; ///< Replay assembly success before presentation.
    std::atomic<bool>    neverCredit{false}; ///< Withhold every CREDIT.
    std::atomic<int>     dropReplyTo{0};     ///< One-shot: swallow the next reply to this command.
    std::atomic<int>     resultDelayMs{0};   ///< END -> RESULT Ok.
    std::atomic<int>     presentDelayMs{3};  ///< RESULT Ok -> PRESENTED (+ CREDIT).
    std::atomic<int>     readRate{0};        ///< Bytes/s the fake reads (0 = as fast as it can).
    std::atomic<int64_t> stallUntilMs{0};    ///< Reads nothing before this time.
    std::atomic<int>     sessionTimeoutMs{5000};
    std::atomic<int>     mapMode{0};         ///< 0 firmware without the map (BadCommand), 1 map extension.
    std::atomic<int>     panMode{0};         ///< 0 no v2 (BadCommand to 0D), 1 panning, 2 advertised but START_PAN refused.
    std::atomic<int>     panMaxW{640}, panMaxH{480};   ///< PAN_CAPABILITIES background maximum.
    std::atomic<bool>    rejectNextPan{false};          ///< One-shot: the next pan image fails to decode.
    std::atomic<int>     initialPage{0};                ///< The tab shown when plugged in (the board's default).

    /// The user taps another tab (map firmware): the bound surface's VISIBILITY follows.
    void setPage(int page) {
        std::lock_guard<std::mutex> lk(m_);
        const uint8_t before = effectiveDemand();
        page_                = page;
        const uint8_t after  = effectiveDemand();
        if (after != es3video::DemandActive) hiddenSinceMs_ = nowMs();
        if (token_ && after != before) sendVisibility(token_, after);
    }
    int page() {
        std::lock_guard<std::mutex> lk(m_);
        return page_;
    }

    ~FakeDisplay() { unplug(); }

    /// A new USB enumeration: a fresh pty, @p link re-pointed at its slave.
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
        {
            std::lock_guard<std::mutex> lk(m_);
            c_       = FakeCounters{};
            frames_.clear();
            session_ = false;
            endVideoLocked();
            visibility_ = es3video::DemandActive;
            page_       = initialPage.load();
        }
        outq::arm(uint64_t(sb.st_rdev));
        master_ = fd;
        run_    = true;
        thread_ = std::thread(&FakeDisplay::loop, this);
        return true;
    }

    /// Cable out: the master closes and the slave sees a hangup.
    void unplug() {
        run_ = false;
        if (thread_.joinable()) thread_.join();
        std::lock_guard<std::mutex> lk(txMu_);
        if (master_ >= 0) {
            outq::armedRdev = 0;
            ::close(master_);
            master_ = -1;
        }
    }

    void setVisibility(uint8_t d) {
        std::lock_guard<std::mutex> lk(m_);
        visibility_ = d;
        if (d != es3video::DemandActive) hiddenSinceMs_ = nowMs();
        if (token_) sendVisibility(token_, effectiveDemand());
    }

    void grantOne() {
        std::lock_guard<std::mutex> lk(m_);
        if (!token_) return;
        ++grant_;
        uint8_t b[64];
        send(b, es3video::encodeCredit(token_, grant_, b, sizeof(b)));
    }

    void expireSession() {
        std::lock_guard<std::mutex> lk(m_);
        session_ = false;
        endVideoLocked();
    }

    /// Well-framed records carrying another session's token.
    void injectStaleRecords() {
        std::lock_guard<std::mutex> lk(m_);
        uint32_t stale = token_ ^ 0x5a5a5a5au;
        if (stale == 0) stale = 0x12345678u;
        uint8_t b[64];
        send(b, es3video::encodeCredit(stale, 100, b, sizeof(b)));
        send(b, es3video::encodeVisibility(stale, es3video::DemandHidden, b, sizeof(b)));
        send(b, es3video::encodeResult(stale, 1, es3video::ReasonOk, b, sizeof(b)));
        es3video::PresentedInfo pi;
        pi.decodeUs = 1;
        pi.blitUs   = 1;
        send(b, es3video::encodePresented(stale, 1, pi, b, sizeof(b)));
    }

    /// Line noise between records: random bytes plus false E3 28 / E3 29 headers.
    void injectGarbage(uint32_t seed, size_t n) {
        std::vector<uint8_t> g;
        uint32_t s = seed * 2654435761u + 1;
        for (size_t i = 0; i < n; ++i) {
            s = s * 1103515245u + 12345u;
            g.push_back(uint8_t(s >> 16));
        }
        const uint8_t fakeVideo[]   = {0xe3, 0x29, 0x01, 0x83, 0x04, 0x00, 0x11, 0x22};
        const uint8_t fakeControl[] = {0xe3, 0x28, 0x01, 0x07, 0x83, 0x40, 0x00};
        g.insert(g.begin() + long(n / 2), fakeVideo, fakeVideo + sizeof(fakeVideo));
        g.insert(g.end(), fakeControl, fakeControl + sizeof(fakeControl));
        std::lock_guard<std::mutex> lk(m_);
        send(g.data(), g.size());
    }

    void injectStalledHeader() {
        uint8_t header[es3video::kHeaderBytes] = {0xe3, 0x29, 1, es3video::RecordChunk};
        es3video::detail::put16(header + 4, 1004);
        es3video::detail::put32(header + 6, 1);
        es3video::detail::put32(header + 10, 1);
        std::lock_guard<std::mutex> lk(m_);
        send(header, sizeof(header));
    }

    FakeCounters counters() {
        std::lock_guard<std::mutex> lk(m_);
        return c_;
    }
    std::vector<FakeFrame> frames() {
        std::lock_guard<std::mutex> lk(m_);
        return frames_;
    }
    es3video::StartVideoRequest lastStartVideo() {
        std::lock_guard<std::mutex> lk(m_);
        return lastReq_;
    }

private:
    struct Event {
        int64_t  dueMs = 0;
        uint8_t  type = 0;       ///< RecordResult / RecordPresented.
        uint32_t frameId = 0;
        uint8_t  reason = 0;
        bool     first = false;  ///< The frame's first RESULT.
        bool     credit = false; ///< Its slot is free afterwards.
        uint32_t decodeUs = 0;
    };

    int               master_ = -1;
    std::atomic<bool> run_{false};
    std::thread       thread_;
    std::mutex        txMu_;
    std::mutex        m_;   // everything below; taken before txMu_

    FakeCounters                c_;
    std::vector<FakeFrame>      frames_;
    bool                        session_ = false;
    int64_t                     lastRequestMs_ = 0;
    es3video::Capabilities      caps_;
    uint32_t                    tokenCounter_ = 0x1000;
    uint32_t                    token_ = 0, grant_ = 0, credited_ = 0, firstResults_ = 0, lastFrameId_ = 0;
    es3video::StartVideoRequest req_, lastReq_;
    uint8_t                     visibility_ = es3video::DemandActive;
    int                         page_ = 0;
    bool                        mapSession_ = false;   ///< token_ belongs to a map session.
    es3map::MapCapabilities     mapCaps_;
    bool                        panSession_ = false;   ///< ... a panning one (extension v2).
    es3map::PanCapabilities     panCaps_;
    es3map::PanSession          panGeo_;
    uint32_t                    poseSeq_ = 0;
    uint8_t                     asmPanKind_ = 0;

    /// Demand of the bound surface: the visibility knob, and with the map
    /// extension also the page (camera on LIVE, map on page 5), as the firmware does.
    uint8_t effectiveDemand() const {
        if (visibility_ != es3video::DemandActive || mapMode.load() == 0) return visibility_;
        const bool onIt = mapSession_ ? page_ == 5 : page_ == 0;
        return onIt ? uint8_t(es3video::DemandActive) : uint8_t(es3video::DemandHidden);
    }
    int64_t                     hiddenSinceMs_ = 0;
    bool                        assembling_ = false;
    uint32_t                    asmId_ = 0, asmBytes_ = 0, asmGot_ = 0;
    uint16_t                    asmW_ = 0, asmH_ = 0;
    uint64_t                    asmTs_ = 0;
    int64_t                     asmStartMs_ = 0;
    std::vector<uint8_t>        asmBuf_ = std::vector<uint8_t>(es3video::kMaxJpegBytes);
    std::vector<uint8_t>        row_ = std::vector<uint8_t>(8192 * 3);
    std::vector<Event>          events_;

    void send(const uint8_t* p, size_t n) {
        std::lock_guard<std::mutex> lk(txMu_);
        size_t off = 0;
        const int64_t give = nowMs() + 100;
        while (off < n && master_ >= 0 && nowMs() < give) {
            const ssize_t k = ::write(master_, p + off, n - off);
            if (k > 0) { off += size_t(k); continue; }
            if (k < 0 && errno != EAGAIN && errno != EINTR) return;
            struct pollfd pf = {master_, POLLOUT, 0};
            ::poll(&pf, 1, 5);
        }
    }

    void reply(const es3proto::Frame& req, const uint8_t* p, size_t n) {
        if (dropReplyTo.load() == req.type) {
            dropReplyTo = 0;
            ++c_.droppedReplies;
            return;
        }
        uint8_t out[es3proto::kMaxFrame];
        send(out, es3proto::encode(req.sequence, uint8_t(req.type | es3proto::Reply), p, n, out));
    }
    void error(const es3proto::Frame& req, uint8_t code) {
        uint8_t out[es3proto::kMaxFrame];
        send(out, es3proto::encode(req.sequence, es3proto::Error, &code, 1, out));
    }
    void sendVisibility(uint32_t token, uint8_t d) {
        uint8_t b[64];
        send(b, es3video::encodeVisibility(token, static_cast<es3video::Demand>(d), b, sizeof(b)));
    }
    void sendResult(uint32_t frameId, uint8_t reason) {
        uint8_t b[64];
        send(b, es3video::encodeResult(token_, frameId, static_cast<es3video::Reason>(reason), b, sizeof(b)));
        if (reason == es3video::ReasonOk && duplicateOk.load())
            send(b, es3video::encodeResult(token_, frameId, es3video::ReasonOk, b, sizeof(b)));
    }
    void creditBack() {
        if (neverCredit.load()) {
            ++c_.withheldCredits;
            return;
        }
        ++grant_;
        uint8_t b[64];
        send(b, es3video::encodeCredit(token_, grant_, b, sizeof(b)));
    }
    /// A credited frame rejected now: its one RESULT, then its credit back.
    void rejectFrame(uint32_t frameId, uint8_t reason) {
        sendResult(frameId, reason);
        ++firstResults_;
        creditBack();
    }
    void endVideoLocked() {
        mapSession_ = false;
        panSession_ = false;
        poseSeq_    = 0;
        token_ = grant_ = credited_ = firstResults_ = lastFrameId_ = 0;
        assembling_ = false;
        events_.clear();
    }

    void loop() {
        es3video::MuxParser  parser(es3video::kMaxChunkDataBytes);
        std::vector<uint8_t> buf(8192);
        double               allowance = 0;
        int64_t              lastUs    = nowUs();
        while (run_) {
            {
                std::lock_guard<std::mutex> lk(m_);
                timers(nowMs());
            }
            if (nowMs() < stallUntilMs.load()) {
                sleepMs(1);
                lastUs = nowUs();
                continue;
            }
            size_t     budget = buf.size();
            const int  rate   = readRate.load();
            if (rate > 0) {
                const int64_t t = nowUs();
                allowance = std::min(allowance + double(t - lastUs) * rate / 1e6, double(std::max(2048, rate / 50)));
                lastUs    = t;
                if (allowance < std::min(512.0, rate / 500.0 + 1)) {
                    sleepMs(1);
                    continue;
                }
                budget = std::min(budget, size_t(allowance));
            }
            struct pollfd p = {master_, POLLIN, 0};
            if (::poll(&p, 1, 2) <= 0) continue;
            if (p.revents & POLLIN) {
                const ssize_t n = ::read(master_, buf.data(), budget);
                if (n > 0) {
                    outq::consumed += n;
                    if (rate > 0) allowance -= double(n);
                    std::lock_guard<std::mutex> lk(m_);
                    const int64_t now  = nowMs();
                    size_t        used = 0;
                    for (;;) {
                        es3video::MuxParser::Item it;
                        used += parser.feed(buf.data() + used, size_t(n) - used, uint32_t(now), it);
                        if (it.kind == es3video::MuxParser::ItemNone) break;
                        if (it.kind == es3video::MuxParser::ItemControl) onControl(it.control, now);
                        else onVideo(it.video, now);
                    }
                    continue;
                }
            }
            if (p.revents & (POLLHUP | POLLERR)) sleepMs(2);   // no slave open right now
        }
    }

    void timers(int64_t now) {
        if (session_ && now - lastRequestMs_ > sessionTimeoutMs.load()) {
            session_ = false;
            endVideoLocked();
            ++c_.sessionExpiries;
        }
        if (assembling_ && now - asmStartMs_ > req_.assemblyTimeoutMs) {
            assembling_ = false;
            ++c_.assemblyTimeouts;
            rejectFrame(asmId_, es3video::ReasonAssemblyTimeout);
        }
        for (size_t i = 0; i < events_.size();) {
            if (events_[i].dueMs > now) { ++i; continue; }
            const Event e = events_[i];
            events_.erase(events_.begin() + long(i));
            uint8_t b[64];
            if (e.type == es3video::RecordResult) {
                sendResult(e.frameId, e.reason);
            } else {
                es3video::PresentedInfo pi;
                pi.decodeUs = e.decodeUs;
                pi.blitUs   = 2300;
                send(b, es3video::encodePresented(token_, e.frameId, pi, b, sizeof(b)));
                ++c_.presented;
            }
            if (e.first) ++firstResults_;
            if (e.credit) creditBack();
        }
    }

    void onControl(const es3proto::Frame& f, int64_t now) {
        if (f.type == es3proto::Hello) {
            session_       = true;
            lastRequestMs_ = now;
            endVideoLocked();
            ++c_.hello;
            reply(f, reinterpret_cast<const uint8_t*>("ES3C28P"), 7);
            return;
        }
        if (!session_) {
            ++c_.noSession;
            error(f, es3proto::NoSession);
            return;
        }
        lastRequestMs_ = now;
        switch (f.type) {
        case es3proto::Ping:
            ++c_.ping;
            reply(f, nullptr, 0);
            return;
        case es3proto::Status: {
            const uint32_t up = uint32_t(now);
            const uint8_t  p[9] = {uint8_t(up), uint8_t(up >> 8), uint8_t(up >> 16), uint8_t(up >> 24),
                                   uint8_t(page_), 80, 1, 1, 0};
            ++c_.status;
            reply(f, p, sizeof(p));
            return;
        }
        case es3proto::Page:
            if (f.length != 1 + 5 * es3proto::kTextBytes || f.payload[0] >= (mapMode.load() ? 6 : 5)) break;
            ++c_.page;
            c_.lastPageTitle.assign(reinterpret_cast<const char*>(f.payload + 1),
                                    strnlen(reinterpret_cast<const char*>(f.payload + 1), es3proto::kTextBytes));
            reply(f, nullptr, 0);
            return;
        case es3proto::Alert:
            if (f.length != 1 || f.payload[0] > 100) break;
            ++c_.alert;
            reply(f, nullptr, 0);
            return;
        case es3proto::StopAudio:
            if (f.length != 0) break;
            ++c_.stopAudio;
            reply(f, nullptr, 0);
            return;
        case es3proto::Capabilities: {
            ++c_.caps;
            const int mode = capsMode.load();
            if (mode == 1) { error(f, es3proto::BadCommand); return; }
            if (mode == 2) { error(f, es3proto::NotReady); return; }
            uint8_t p[es3video::kCapabilitiesReplyBytes] = {};
            if (mode == 3) {   // a newer, unknown video protocol
                p[0] = 9;
                reply(f, p, sizeof(p));
                return;
            }
            caps_.canvasWidth   = es3video::kCanvasWidth;
            caps_.canvasHeight  = es3video::kCanvasHeight;
            caps_.maxWidth      = es3video::kCanvasWidth;
            caps_.maxHeight     = es3video::kCanvasHeight;
            caps_.maxJpegBytes  = uint32_t(devMaxJpeg.load());
            caps_.maxChunkBytes = uint16_t(devMaxChunk.load());
            caps_.maxCredits    = uint8_t(devMaxCredits.load());
            reply(f, p, es3video::encodeCapabilitiesReply(caps_, p, sizeof(p)));
            return;
        }
        case es3proto::StartVideo: {
            ++c_.startVideo;
            if (mapSession_ && token_) {   // one receiver: the map must be stopped first
                ++c_.busyRefusals;
                error(f, es3proto::NotReady);
                return;
            }
            es3video::StartVideoRequest r;
            if (!es3video::decodeStartVideoRequest(f.payload, f.length, caps_, r)) {
                ++c_.startVideoRejected;
                error(f, es3proto::BadPayload);
                return;
            }
            endVideoLocked();
            token_   = ++tokenCounter_;
            req_     = r;
            lastReq_ = r;
            grant_   = uint32_t(std::min(initialGrant.load(), devMaxCredits.load()));
            es3video::StartVideoReply rep;
            rep.sessionToken = token_;
            rep.initialGrant = grant_;
            uint8_t p[es3video::kStartVideoReplyBytes];
            reply(f, p, es3video::encodeStartVideoReply(rep, p, sizeof(p)));
            sendVisibility(token_, effectiveDemand());   // fresh demand for every new session
            return;
        }
        case es3map::CmdMapCapabilities: {
            ++c_.mapCaps;
            if (mapMode.load() == 0 || caps_.maxCredits == 0) { error(f, es3proto::BadCommand); return; }
            mapCaps_.pageCount    = 6;
            mapCaps_.mapPage      = 5;
            mapCaps_.viewportX    = 0;
            mapCaps_.viewportY    = 0;
            mapCaps_.viewportW    = 320;
            mapCaps_.viewportH    = 224;
            mapCaps_.maxJpegBytes = caps_.maxJpegBytes;
            mapCaps_.flags        = es3map::kFlagMapTab;
            uint8_t p[es3map::kMapCapabilitiesReplyBytes];
            reply(f, p, es3map::encodeMapCapabilitiesReply(mapCaps_, caps_, p, sizeof(p)));
            return;
        }
        case es3map::CmdStartMap: {
            ++c_.startMap;
            if (mapMode.load() == 0) { error(f, es3proto::BadCommand); return; }
            if (!mapSession_ && token_) {   // the camera still owns the receiver
                ++c_.busyRefusals;
                error(f, es3proto::NotReady);
                return;
            }
            es3map::StartMapRequest r;
            if (!es3map::decodeStartMapRequest(f.payload, f.length, mapCaps_, caps_, r)) {
                ++c_.startMapRejected;
                error(f, es3proto::BadPayload);
                return;
            }
            endVideoLocked();
            mapSession_ = true;
            token_      = ++tokenCounter_;
            req_        = es3map::sessionLimits(r);
            grant_      = uint32_t(std::min(initialGrant.load(), devMaxCredits.load()));
            es3map::StartMapReply rep;
            rep.sessionToken = token_;
            rep.initialGrant = grant_;
            uint8_t p[es3map::kStartMapReplyBytes];
            reply(f, p, es3map::encodeStartMapReply(rep, p, sizeof(p)));
            sendVisibility(token_, effectiveDemand());
            return;
        }
        case es3map::CmdPanCapabilities: {
            ++c_.panCaps;
            if (mapMode.load() == 0 || panMode.load() == 0) { error(f, es3proto::BadCommand); return; }
            panCaps_.maxBackgroundW = uint16_t(panMaxW.load());
            panCaps_.maxBackgroundH = uint16_t(panMaxH.load());
            panCaps_.chromeTopH     = 16;
            panCaps_.chromeBottomH  = 16;
            panCaps_.maxPoseHz      = 30;
            uint8_t p[es3map::kPanCapabilitiesReplyBytes];
            reply(f, p, es3map::encodePanCapabilitiesReply(panCaps_, mapCaps_, caps_, p, sizeof(p)));
            return;
        }
        case es3map::CmdStartPan: {
            ++c_.startPan;
            if (mapMode.load() == 0 || panMode.load() == 0) { error(f, es3proto::BadCommand); return; }
            if (!mapSession_ && token_) {
                ++c_.busyRefusals;
                error(f, es3proto::NotReady);
                return;
            }
            es3map::StartPanRequest r;
            if (panMode.load() == 2 || !es3map::decodeStartPanRequest(f.payload, f.length, panCaps_, mapCaps_, caps_, r)) {
                ++c_.startPanRefused;
                error(f, es3proto::BadPayload);
                return;
            }
            endVideoLocked();
            mapSession_ = true;
            panSession_ = true;
            panGeo_     = es3map::panSession(r, panCaps_);
            token_      = ++tokenCounter_;
            req_        = es3map::sessionLimits(r.map);
            grant_      = uint32_t(std::min(initialGrant.load(), devMaxCredits.load()));
            es3map::StartPanReply rep;
            rep.sessionToken = token_;
            rep.initialGrant = grant_;
            uint8_t p[es3map::kStartMapReplyBytes];
            reply(f, p, es3map::encodeStartPanReply(rep, p, sizeof(p)));
            sendVisibility(token_, effectiveDemand());
            return;
        }
        case es3map::CmdStopMap: {
            ++c_.stopMap;
            if (mapMode.load() == 0) { error(f, es3proto::BadCommand); return; }
            uint32_t t = 0;
            if (es3map::decodeStopMapRequest(f.payload, f.length, t) && mapSession_ && t == token_) endVideoLocked();
            reply(f, nullptr, 0);
            return;
        }
        case es3proto::StopVideo: {
            ++c_.stopVideo;
            uint32_t t = 0;
            if (es3video::decodeStopVideoRequest(f.payload, f.length, t) && !mapSession_ && t == token_)
                endVideoLocked();
            reply(f, nullptr, 0);   // a stale token is a no-op that still gets the reply
            return;
        }
        default:
            ++c_.badCommand;
            error(f, es3proto::BadCommand);
            return;
        }
        ++c_.badPayload;
        error(f, es3proto::BadPayload);
    }

    void onVideo(const es3video::Record& r, int64_t now) {
        c_.videoBytes += int64_t(es3video::kOverheadBytes + r.payloadLength);
        if (token_ == 0 || r.sessionToken != token_) {
            ++c_.staleFromHost;
            return;
        }
        if (effectiveDemand() != es3video::DemandActive && now - hiddenSinceMs_ > 100) {
            c_.bytesWhileHidden += int64_t(es3video::kOverheadBytes + r.payloadLength);
            if (r.type == es3video::RecordStart) ++c_.startsWhileHidden;
        }
        switch (r.type) {
        case es3video::RecordMapPose: {
            es3map::MapPose pose;
            if (!panSession_) { ++c_.posesOutsideSession; return; }
            if (es3map::decodeMapPose(r, pose) != es3video::ReasonOk) { ++c_.poseBadFields; return; }
            if (pose.seq <= poseSeq_) { ++c_.poseSeqErrors; return; }
            poseSeq_ = pose.seq;
            if (effectiveDemand() != es3video::DemandActive && now - hiddenSinceMs_ > 100) ++c_.posesWhileHidden;
            if (c_.poses) c_.minPoseGapMs = std::min(c_.minPoseGapMs, now - c_.lastPoseMs);
            else c_.firstPoseMs = now;
            ++c_.poses;
            c_.lastPose   = pose;
            c_.lastPoseMs = now;
            return;
        }
        case es3video::RecordStart: {
            es3video::StartInfo si;
            es3video::Reason    why = es3video::decodeStart(r, si);
            ++c_.starts;
            if (credited_ >= grant_) {   // never credited: no grant advance
                ++c_.noCredit;
                sendResult(r.frameId, es3video::ReasonNoCredit);
                return;
            }
            ++credited_;
            c_.maxInFlight = std::max(c_.maxInFlight, int(credited_ - firstResults_));
            if (assembling_) {
                ++c_.startDuringAssembly;
                assembling_ = false;
                rejectFrame(asmId_, es3video::ReasonSuperseded);
            }
            es3map::PanImage kind = es3map::PanImageNone;
            if (why == es3video::ReasonOk)
                why = panSession_ ? es3map::checkPanStart(si, panGeo_, kind) : es3video::checkStartForSession(si, req_);
            asmPanKind_ = uint8_t(kind);
            if (why == es3video::ReasonOk && r.frameId <= lastFrameId_) why = es3video::ReasonBadFrameId;
            if (why == es3video::ReasonOk && effectiveDemand() != es3video::DemandActive) why = es3video::ReasonHidden;
            lastFrameId_ = std::max(lastFrameId_, r.frameId);
            if (why != es3video::ReasonOk) {
                ++c_.rejectedStarts;
                rejectFrame(r.frameId, why);
                return;
            }
            assembling_ = true;
            asmId_      = r.frameId;
            asmBytes_   = si.jpegBytes;
            asmGot_     = 0;
            asmW_       = si.width;
            asmH_       = si.height;
            asmTs_      = si.sourceTimestampUs;
            asmStartMs_ = now;
            c_.lastStartTs = si.sourceTimestampUs;
            return;
        }
        case es3video::RecordChunk: {
            if (!assembling_ || r.frameId != asmId_) { ++c_.orphan; return; }
            es3video::ChunkInfo ci;
            es3video::Reason    why = es3video::decodeChunk(r, ci);
            if (why == es3video::ReasonOk) why = es3video::checkChunkForSession(ci, req_, asmGot_, asmBytes_);
            c_.maxChunk = std::max(c_.maxChunk, ci.size);
            if (why != es3video::ReasonOk) {
                ++c_.badChunks;
                assembling_ = false;
                rejectFrame(asmId_, why);
                return;
            }
            std::memcpy(asmBuf_.data() + asmGot_, ci.data, ci.size);
            asmGot_ += uint32_t(ci.size);
            return;
        }
        case es3video::RecordEnd: {
            if (!assembling_ || r.frameId != asmId_) { ++c_.orphan; return; }
            uint32_t crc = 0;
            es3video::decodeEnd(r, crc);
            assembling_ = false;
            if (asmGot_ != asmBytes_) { ++c_.badLength; rejectFrame(asmId_, es3video::ReasonBadLength); return; }
            if (refCrc32(asmBuf_.data(), asmGot_) != crc) {
                ++c_.crcErrors;
                rejectFrame(asmId_, es3video::ReasonBadCrc);
                return;
            }
            FakeFrame fr;
            fr.map = mapSession_;
            fr.panKind = asmPanKind_;
            fr.token   = token_;
            fr.frameId = asmId_;
            fr.bytes   = asmGot_;
            fr.w       = asmW_;
            fr.h       = asmH_;
            fr.ts      = asmTs_;
            const int64_t t0 = nowUs();
            fr.decodeOk = decodeJpeg(asmBuf_.data(), asmGot_, row_, fr.decW, fr.decH);
            const uint32_t decodeUs = uint32_t(nowUs() - t0) + 1;
            frames_.push_back(fr);
            const int64_t okAt = now + resultDelayMs.load();
            Event first;
            first.dueMs   = okAt;
            first.type    = es3video::RecordResult;
            first.frameId = asmId_;
            first.reason  = es3video::ReasonOk;
            first.first   = true;
            events_.push_back(first);
            Event done;
            done.dueMs    = okAt + presentDelayMs.load();
            done.frameId  = asmId_;
            done.credit   = true;
            done.decodeUs = decodeUs;
            const bool forcedFail = fr.panKind && rejectNextPan.exchange(false);
            if (!fr.decodeOk || forcedFail) {
                ++c_.decodeFails;
                done.type   = es3video::RecordResult;
                done.reason = es3video::ReasonDecodeFailed;
            } else {
                ++c_.framesOk;
                if (fr.decW != fr.w || fr.decH != fr.h) ++c_.dimMismatch;
                done.type = es3video::RecordPresented;
            }
            events_.push_back(done);
            return;
        }
        case es3video::RecordAbort: {
            es3video::Reason reason = es3video::ReasonOk;
            es3video::decodeAbort(r, reason);
            if (!assembling_ || r.frameId != asmId_) { ++c_.orphan; return; }
            assembling_ = false;
            ++c_.aborts;
            rejectFrame(asmId_, reason);
            return;
        }
        default:
            return;
        }
    }
};

// ─── rig: temp dir + symlink, fake, link, frames ──────────────────────────────

struct Rig {
    std::string   dir, dev;
    FakeDisplay   fake;
    LogSink       logs;
    Es3DisplayLink link;

    Rig() {
        char tmpl[] = "/tmp/es3display_test_XXXXXX";
        const char* d = ::mkdtemp(tmpl);
        dir = d ? d : "/tmp";
        dev = dir + "/dashcam-display";
    }
    ~Rig() {
        link.stop();
        fake.unplug();
        std::error_code ec;
        if (dir != "/tmp") fs::remove_all(dir, ec);
    }
    DisplayLinkConfig cfg() const {
        DisplayLinkConfig c;
        c.device         = dev;
        c.reconnectMs    = 200;
        c.replyTimeoutMs = 500;
        c.heartbeatMs    = 1000;
        return c;
    }
    bool up(const DisplayLinkConfig& c, LinkState want = LinkState::Negotiated, int ms = 2000) {
        return fake.plug(dev) && link.start(c, logs.callback()) &&
               waitFor([&] { return link.state() == want; }, ms);
    }
};

/// Fills a lease from @p j and offers it: 1 accepted, 0 refused (lease kept, then freed), -1 no free slot.
static int offerJpeg(dashcam::preview::IPreviewSink& link, EncodedFramePool& pool, const Jpeg& j, uint64_t ts, uint16_t w = 0,
                     uint16_t h = 0, int64_t* durUs = nullptr) {
    EncodedFrameLease l = pool.tryAcquire();
    if (!l) return -1;
    std::memcpy(l.writableData(), j.bytes.data(), j.bytes.size());
    l.setFrame(j.bytes.size(), w ? w : j.w, h ? h : j.h, ts, ts, 65);
    const int64_t t0 = nowUs();
    const bool    ok = link.offerFrame(std::move(l));
    if (durUs) *durUs = nowUs() - t0;
    if (!ok && !l.valid()) return -2;   // ownership must stay with the caller on false
    return ok ? 1 : 0;
}

/// Offers frames from @p jpegs round robin every @p periodMs until stopped.
class Producer {
public:
    Producer(dashcam::preview::IPreviewSink& link, std::vector<Jpeg> jpegs, int periodMs, size_t slots = 3)
        : link_(link), jpegs_(std::move(jpegs)), periodMs_(periodMs),
          pool(EncodedFramePool::create(slots, es3video::kMaxJpegBytes)) {}
    ~Producer() { stop(); }
    void start() {
        stop();
        run_    = true;
        thread_ = std::thread([this] {
            size_t i = 0;
            while (run_) {
                int64_t   d = 0;
                const int r = offerJpeg(link_, *pool, jpegs_[i++ % jpegs_.size()], uint64_t(nowUs()), 0, 0, &d);
                if (r == 1) ++accepted; else if (r == 0) ++refused; else if (r == -1) ++noSlot; else ++ownershipBugs;
                int64_t m = maxOfferUs.load();
                while (d > m && !maxOfferUs.compare_exchange_weak(m, d)) {}
                sleepMs(periodMs_);
            }
        });
    }
    void stop() {
        run_ = false;
        if (thread_.joinable()) thread_.join();
    }
    std::atomic<uint64_t> accepted{0}, refused{0}, noSlot{0}, ownershipBugs{0};
    std::atomic<int64_t>  maxOfferUs{0};

private:
    dashcam::preview::IPreviewSink& link_;
    std::vector<Jpeg> jpegs_;
    int               periodMs_;
    std::atomic<bool> run_{false};
    std::thread       thread_;

public:
    std::shared_ptr<EncodedFramePool> pool;
};

static std::vector<Jpeg> smallFrames() {
    return {makeJpeg(320, 180, gradient(1), 65), makeJpeg(320, 180, gradient(2), 65),
            makeJpeg(320, 180, gradient(3), 65)};
}

/// Collects control outcomes from callbacks.
struct Outcomes {
    std::mutex                 m;
    std::vector<ControlResult> results;
    ControlCallback cb() {
        return [this](const ControlResult& r) {
            std::lock_guard<std::mutex> lk(m);
            results.push_back(r);
        };
    }
    size_t size() {
        std::lock_guard<std::mutex> lk(m);
        return results.size();
    }
    std::vector<ControlResult> all() {
        std::lock_guard<std::mutex> lk(m);
        return results;
    }
};

static ControlRequest withCb(ControlRequest r, ControlCallback cb) {
    r.onDone = std::move(cb);
    return r;
}

static const uint32_t kRecordBytes = es3video::kOverheadBytes + es3video::kChunkOffsetBytes + 1024;

// ─── tests ────────────────────────────────────────────────────────────────────

// ─── compatibility: the ES3C28P firmware's own logic as the device ─────────────

#include "es3_firmware_fake.h"

struct FwRig {
    std::string    dir, dev;
    FirmwareFake   fw;
    LogSink        logs;
    Es3DisplayLink link;
    FwRig() {
        char tmpl[] = "/tmp/es3display_fw_XXXXXX";
        const char* d = ::mkdtemp(tmpl);
        dir = d ? d : "/tmp";
        dev = dir + "/dashcam-display";
    }
    ~FwRig() {
        link.stop();
        fw.unplug();
        std::error_code ec;
        if (dir != "/tmp") fs::remove_all(dir, ec);
    }
    DisplayLinkConfig cfg() const {   // the app's defaults (<DisplayPreview>)
        DisplayLinkConfig c;
        c.device = dev;
        return c;
    }
    bool up() {
        return fw.plug(dev) && link.start(cfg(), logs.callback()) &&
               waitFor([&] { return link.state() == LinkState::Negotiated; }, 3000);
    }
};

static void testFirmwareCompatibility() {
    std::printf("\n[19] compatibility: Es3DisplayLink vs the ES3C28P firmware logic (VideoState.h, es3UsbLink rules)\n");
    FwRig r;
    check(r.up(), "HELLO + CAPABILITIES with the firmware's dispatch");
    DisplayLinkStats s = r.link.stats();
    check(s.identity == "ES3C28P" && s.videoSupported && s.capabilities.canvasWidth == 320 &&
              s.capabilities.canvasHeight == 240 && s.capabilities.maxCredits == 1 &&
              s.capabilities.maxChunkBytes == 1024,
          "identity ES3C28P, canvas 320x240, 1 credit, 1024-byte chunks");

    // 10 fps of 320x180 frames for 3 s, the app's default rate.
    Producer p(r.link, smallFrames(), 100);
    p.start();
    sleepMs(3000);
    FirmwareFake::Counters c = r.fw.counters();
    s = r.link.stats();
    check(c.startVideo == 1 && s.sessions == 1 && s.creditWindow == 1, "one START_VIDEO, window 1");
    check(c.video.presented >= 25 && s.framesPresented >= 25,
          std::to_string(c.video.presented) + " frames presented in 3 s at 10 fps (host saw " +
              std::to_string(s.framesPresented) + ")");
    check(c.video.dropped == 0 && c.decodeFailures == 0 && s.framesDropped == 0,
          "no drops, no decode failures (" + std::to_string(c.video.dropped) + " dropped by the device)");
    check(c.sessionTimeouts == 0 && c.txStalls == 0 && c.faults == 0 && s.rehellos == 0 && s.resyncs == 0,
          "no session timeout, TX stall, event-queue fault, re-HELLO or resync");
    check(s.resultReasons[es3video::ReasonNoCredit] == 0 && s.resultReasons[es3video::ReasonBadFrameId] == 0,
          "never a START beyond the grant, frame ids accepted");
    std::printf("        latency START->PRESENTED p50 %.1f / p95 %.1f ms (decode by libjpeg, 23 ms blit)\n",
                s.presentLatency.p50Ms, s.presentLatency.p95Ms);

    // As fast as the host can offer: the pipeline limit with one credit.
    p.stop();
    const uint32_t before = r.fw.counters().video.presented;
    Producer fast(r.link, smallFrames(), 1);
    fast.start();
    sleepMs(2000);
    fast.stop();
    const double fps = (r.fw.counters().video.presented - before) / 2.0;
    char line[160];
    std::snprintf(line, sizeof(line), "unthrottled: %.1f fps presented with one credit (>= 15)", fps);
    check(fps >= 15.0, line);

    // Another tab: the device says hidden, the host stops starting frames.
    p.start();
    sleepMs(300);
    r.fw.demand = es3video::DemandHidden;
    check(waitFor([&] { return !r.link.demand().active; }, 1000), "hidden tab -> host demand inactive");
    const uint32_t hiddenStarts = r.fw.counters().startsWhileHidden;
    sleepMs(1000);
    check(r.fw.counters().startsWhileHidden <= hiddenStarts + 1,
          "no new frames started while hidden (" + std::to_string(r.fw.counters().startsWhileHidden) + ")");
    const uint32_t presentedHidden = r.fw.counters().video.presented;
    r.fw.demand = es3video::DemandActive;
    check(waitFor([&] { return r.fw.counters().video.presented >= presentedHidden + 5; }, 2000),
          "LIVE again -> frames resume");

    // Loop-handled controls while streaming: the USB task reads nothing until the
    // loop answers, so the loop's time per command is the host's reply time.
    r.fw.applyDelayMs = 300;
    Outcomes o;
    for (int i = 0; i < 8; ++i) {
        r.link.enqueueControl(withCb(ControlRequest::status(), o.cb()));
        sleepMs(400);
    }
    check(waitFor([&] { return o.size() == 8; }, 2000), "8 STATUS requests answered while streaming (300 ms each)");
    const auto res = o.all();
    s = r.link.stats();
    check(std::all_of(res.begin(), res.end(), [](const ControlResult& x) { return x.status == ControlStatus::Ok; }) &&
              s.controlTimeouts == 0 && s.rehellos == 0,
          "all Ok, no timeout, no re-HELLO");
    check(r.fw.counters().sessionTimeouts == 0, "heartbeats kept the 5 s device session alive");

    // A loop command slower than replyTimeoutMs (500 ms) but within
    // loopReplyTimeoutMs (2 s): answered, the video session survives.
    r.fw.applyDelayMs = 700;
    const uint64_t sessionsBefore = r.link.stats().sessions, rehellosBefore = r.link.stats().rehellos;
    Outcomes slow;
    r.link.enqueueControl(withCb(ControlRequest::page(1, "TRIP", {"a", "b", "c", "d"}), slow.cb()));
    check(waitFor([&] { return slow.size() == 1; }, 3000) && slow.all()[0].status == ControlStatus::Ok,
          "a 700 ms PAGE is answered Ok (" + std::to_string(int(slow.size() ? slow.all()[0].latencyMs : 0)) + " ms)");
    check(r.link.stats().rehellos == rehellosBefore && r.link.stats().sessions == sessionsBefore,
          "... no re-HELLO, the video session survives");
    const uint64_t presentedBefore = r.link.stats().framesPresented;
    r.fw.applyDelayMs = 2;
    check(waitFor([&] { return r.link.stats().framesPresented >= presentedBefore + 10; }, 3000),
          "video keeps flowing afterwards");

    // Past loopReplyTimeoutMs: HELLO again, video renegotiates.
    r.fw.applyDelayMs = 2500;
    Outcomes stuck;
    r.link.enqueueControl(withCb(ControlRequest::page(1, "TRIP", {"a", "b", "c", "d"}), stuck.cb()));
    check(waitFor([&] { return stuck.size() == 1; }, 4000), "a 2.5 s PAGE completes");
    // The callback fires at the timeout; the re-HELLO is counted just after it.
    check(stuck.all()[0].status == ControlStatus::Timeout &&
              waitFor([&] { return r.link.stats().rehellos > rehellosBefore; }, 500),
          "... as Timeout after " + std::to_string(int(stuck.all()[0].latencyMs)) + " ms, then HELLO again");
    r.fw.applyDelayMs = 2;
    const uint64_t presentedStuck = r.link.stats().framesPresented;
    check(waitFor([&] { return r.link.stats().framesPresented >= presentedStuck + 10; }, 5000),
          "video renegotiates and resumes");
    p.stop();

    // Idle: only the host's heartbeats keep the device session.
    const uint32_t timeouts = r.fw.counters().sessionTimeouts;
    sleepMs(6000);
    check(r.fw.counters().sessionTimeouts == timeouts && r.link.state() == LinkState::Active,
          "6 s idle: no device session timeout, link still active");

    // The real converter in front.
    const Jpeg src = makeJpeg(1920, 1080, gradient(5), 85);
    dashcam::preview::VideoPreview  pv;
    dashcam::preview::PreviewConfig pc;
    check(pv.start(pc, r.link, r.logs.callback()), "converter started");
    const uint32_t pre = r.fw.counters().video.presented;
    for (int i = 0; i < 30; ++i) {
        pv.submitJpeg(src.bytes.data(), src.bytes.size(), 1920, 1080, uint64_t(nowUs()));
        sleepMs(33);
    }
    pv.stop();
    check(waitFor([&] { return r.fw.counters().video.presented >= pre + 7; }, 2000),
          "1080p -> converter -> link -> firmware logic: " + std::to_string(r.fw.counters().video.presented - pre) +
              " presented");
    check(r.fw.counters().decodeFailures == 0, "every converter frame passes the firmware's size/baseline checks");
}

static void testConfigAndLifecycle() {
    std::printf("\n[1] config validation, lifecycle, absent device\n");
    DisplayLinkConfig good;
    good.device = "/dev/null-display";
    check(validateConfig(good), "defaults plus a device path are valid");
    struct Bad {
        const char*                               what;
        std::function<void(DisplayLinkConfig&)> f;
    };
    const Bad bads[] = {
        {"empty device", [](DisplayLinkConfig& c) { c.device.clear(); }},
        {"heartbeatMs 50", [](DisplayLinkConfig& c) { c.heartbeatMs = 50; }},
        {"heartbeatMs 5000 (past the device's 5 s session)", [](DisplayLinkConfig& c) { c.heartbeatMs = 5000; }},
        {"reconnectMs 10", [](DisplayLinkConfig& c) { c.reconnectMs = 10; }},
        {"replyTimeoutMs 10", [](DisplayLinkConfig& c) { c.replyTimeoutMs = 10; }},
        {"chunkDataBytes 0", [](DisplayLinkConfig& c) { c.chunkDataBytes = 0; }},
        {"chunkDataBytes 5000", [](DisplayLinkConfig& c) { c.chunkDataBytes = 5000; }},
        {"maxFramesInFlight 0", [](DisplayLinkConfig& c) { c.maxFramesInFlight = 0; }},
        {"maxFramesInFlight 3", [](DisplayLinkConfig& c) { c.maxFramesInFlight = 3; }},
        {"targetFps 0", [](DisplayLinkConfig& c) { c.targetFps = 0; }},
        {"targetFps 31", [](DisplayLinkConfig& c) { c.targetFps = 31; }},
        {"maxJpegBytes 100", [](DisplayLinkConfig& c) { c.maxJpegBytes = 100; }},
        {"maxJpegBytes 70000", [](DisplayLinkConfig& c) { c.maxJpegBytes = 70000; }},
        {"assemblyTimeoutMs 0", [](DisplayLinkConfig& c) { c.assemblyTimeoutMs = 0; }},
        {"staleAfterMs 0", [](DisplayLinkConfig& c) { c.staleAfterMs = 0; }},
        {"placeholderAfterMs < staleAfterMs", [](DisplayLinkConfig& c) { c.placeholderAfterMs = 400; }},
        {"placeholderAfterMs 20000", [](DisplayLinkConfig& c) { c.placeholderAfterMs = 20000; }},
        {"creditStallMs 50", [](DisplayLinkConfig& c) { c.creditStallMs = 50; }},
    };
    for (const Bad& b : bads) {
        DisplayLinkConfig c = good;
        b.f(c);
        std::string why;
        check(!validateConfig(c, &why) && !why.empty(), std::string("rejects ") + b.what + " (" + why + ")");
    }

    Es3DisplayLink idle;
    idle.stop();
    idle.stop();
    check(!idle.running() && idle.state() == LinkState::Disconnected && !idle.demand().active,
          "a never-started link: stop() twice is a no-op, demand inactive");
    check(!idle.enqueueControl(ControlRequest::status()), "a never-started link rejects controls");
    {
        auto pool = EncodedFramePool::create(1, 4096);
        const Jpeg j = makeJpeg(32, 18, gradient(0), 60);
        check(offerJpeg(idle, *pool, j, 1) == 0 && pool->freeSlots() == 1,
              "a never-started link refuses frames; the lease stays with the caller");
    }
    DisplayLinkConfig bad = good;
    bad.chunkDataBytes    = 0;
    LogSink badLog;
    check(!idle.start(bad, badLog.callback()) && badLog.count("E ES3 display: invalid config") == 1,
          "start() with an invalid config returns false and logs why");

    Rig r;
    DisplayLinkConfig c = r.cfg();
    check(r.link.start(c, r.logs.callback()), "start() with the device absent succeeds (retries in the background)");
    check(!r.link.start(c, r.logs.callback()), "a second start() is refused");
    sleepMs(500);
    DisplayLinkStats s = r.link.stats();
    check(s.state == LinkState::Disconnected && s.openFailures >= 2,
          "absent device: disconnected, retried (" + std::to_string(s.openFailures) + " attempts)");
    check(r.logs.count("W ES3 display: cannot open") == 1 && r.logs.count("D ES3 display: cannot open") >= 1,
          "the open failure is logged once as WARN, repeats at DEBUG");
    check(!r.link.enqueueControl(ControlRequest::status()), "controls are rejected while no tty is open");
    check(r.fake.plug(r.dev), "plug the fake display in");
    check(waitFor([&] { return r.link.state() == LinkState::Negotiated; }, 1500),
          "the link finds it on the next retry: negotiated");
    s = r.link.stats();
    check(s.connects == 1 && s.identity == "ES3C28P", "one connect, identity \"" + s.identity + "\"");
    r.link.stop();
    r.link.stop();
    check(r.link.state() == LinkState::Disconnected && !r.link.running() && !r.link.demand().active,
          "stop() (twice) leaves it disconnected, demand inactive");
    check(r.link.start(c, r.logs.callback()) && waitFor([&] { return r.link.state() == LinkState::Negotiated; }, 1500),
          "restart after stop() negotiates again");
    check(r.link.stats().connects == 1 && r.fake.counters().hello == 2, "stats restart with start(); a fresh HELLO");
}

static void testNegotiation() {
    std::printf("\n[2] handshake, identity, negotiated limits, START_VIDEO fields, viewport change\n");
    Rig r;
    r.fake.devMaxChunk   = 512;
    r.fake.devMaxJpeg    = 40000;
    r.fake.devMaxCredits = 1;
    DisplayLinkConfig c  = r.cfg();
    c.chunkDataBytes     = 1024;
    c.maxJpegBytes       = 65536;
    c.targetFps          = 12;
    c.assemblyTimeoutMs  = 700;
    c.staleAfterMs       = 600;
    c.placeholderAfterMs = 2500;
    check(r.up(c), "HELLO + CAPABILITIES -> negotiated");
    DisplayLinkStats s = r.link.stats();
    check(s.identity == "ES3C28P" && s.haveCapabilities && s.videoSupported, "identity and capabilities recorded");
    check(s.capabilities.canvasWidth == 320 && s.capabilities.canvasHeight == 240 && s.capabilities.maxChunkBytes == 512 &&
              s.capabilities.maxJpegBytes == 40000,
          "advertised limits recorded as sent");
    check(s.maxJpegBytes == 40000 && s.creditWindow == 1, "maxJpeg = min(config, device), window = min(config, credits)");
    const dashcam::preview::PreviewDemand d = r.link.demand();
    check(d.active && d.canvasW == 320 && d.canvasH == 240 && d.maxJpegBytes == 40000,
          "demand(): active, the device canvas and min(config, device) JPEG cap before any session");
    check(r.fake.counters().hello == 1 && r.fake.counters().caps == 1 && r.fake.counters().startVideo == 0,
          "nothing negotiated before a frame arrives");

    auto pool = EncodedFramePool::create(4, es3video::kMaxJpegBytes);
    const Jpeg wide = makeJpeg(320, 180, gradient(4), 65);
    check(offerJpeg(r.link, *pool, wide, uint64_t(nowUs())) == 1, "a 320x180 frame is accepted");
    check(waitFor([&] { return r.fake.counters().framesOk >= 1; }, 1500), "... negotiated and assembled by the device");
    es3video::StartVideoRequest q = r.fake.lastStartVideo();
    const dashcam::preview::Viewport lb = dashcam::preview::computeLetterbox(1920, 1080, 320, 240);
    check(q.videoProtocolVersion == 1 && q.codec == 0 && q.viewportX == lb.x && q.viewportY == lb.y &&
              q.viewportW == lb.width && q.viewportH == lb.height,
          "START_VIDEO viewport " + std::to_string(q.viewportX) + "," + std::to_string(q.viewportY) + " " +
              std::to_string(q.viewportW) + "x" + std::to_string(q.viewportH) + " = computeLetterbox(16:9)");
    check(q.chunkBytes == 512 && q.maxJpegBytes == 40000 && q.targetFps == 12 && q.assemblyTimeoutMs == 700 &&
              q.staleAfterMs == 600 && q.placeholderAfterMs == 2500,
          "START_VIDEO: chunk min(1024, 512), JPEG min, fps and the three timeouts from the config");
    check(r.fake.counters().maxChunk > 0 && r.fake.counters().maxChunk <= 512, "CHUNK records respect the 512 B limit");
    check(waitFor([&] { return r.link.stats().framesPresented >= 1; }, 1000) && r.link.state() == LinkState::Active,
          "PRESENTED received, state active");

    const Jpeg full = makeJpeg(320, 240, gradient(5), 65);
    check(offerJpeg(r.link, *pool, full, uint64_t(nowUs())) == 1, "a 320x240 (4:3) frame is accepted");
    check(waitFor([&] { return r.fake.counters().startVideo == 2 && r.fake.counters().framesOk >= 2; }, 1500),
          "a new viewport renegotiates");
    q = r.fake.lastStartVideo();
    check(q.viewportX == 0 && q.viewportY == 0 && q.viewportW == 320 && q.viewportH == 240,
          "START_VIDEO viewport 0,0 320x240");
    check(waitFor([&] { return r.link.stats().sessions == 2 && r.link.stats().framesPresented >= 2; }, 1000),
          "two sessions, both frames presented");

    const Jpeg big = bigFrame(7);
    check(big.bytes.size() > 40000, "fixture: a " + std::to_string(big.bytes.size()) + " B frame");
    check(offerJpeg(r.link, *pool, big, uint64_t(nowUs())) == 0, "a frame above the negotiated JPEG cap is refused");
    check(offerJpeg(r.link, *pool, wide, uint64_t(nowUs()), 400, 180) == 0,
          "a frame wider than the device's max_width is refused");
    check(waitFor([&] { return pool->freeSlots() == 4; }, 1000), "no lease is left behind");
}

static void testControlOnly() {
    std::printf("\n[3] old firmware / no video memory / unusable capabilities -> control-only\n");
    const char* names[] = {"", "BadCommand (old firmware)", "NotReady (no video memory)", "unusable reply (version 9)"};
    for (int mode = 1; mode <= 3; ++mode) {
        Rig r;
        r.fake.capsMode = mode;
        check(r.up(r.cfg(), LinkState::ControlOnly), std::string(names[mode]) + ": control-only");
        DisplayLinkStats s = r.link.stats();
        check(!s.videoSupported && !r.link.demand().active, "video unsupported, demand inactive");
        auto pool = EncodedFramePool::create(1, es3video::kMaxJpegBytes);
        check(offerJpeg(r.link, *pool, makeJpeg(320, 180, gradient(1), 65), 1) == 0 && pool->freeSlots() == 1,
              "frames are refused (lease kept)");
        Outcomes o;
        check(r.link.enqueueControl(withCb(ControlRequest::page(0, "DASHCAM LIVE", {"Record: ON", "Speed: 42 km/h",
                                                                                    "GPS: fix", "Event: none"}),
                                           o.cb())) &&
                  r.link.enqueueControl(withCb(ControlRequest::alert(60), o.cb())) &&
                  r.link.enqueueControl(withCb(ControlRequest::stopAudio(), o.cb())) &&
                  r.link.enqueueControl(withCb(ControlRequest::status(), o.cb())),
              "page, alert, stop audio and status are queued");
        check(waitFor([&] { return o.size() == 4; }, 1500), "all four complete");
        const auto res = o.all();
        const bool allOk = std::all_of(res.begin(), res.end(), [](const ControlResult& x) {
            return x.status == ControlStatus::Ok;
        });
        DeviceStatus ds;
        check(allOk && res.size() == 4 && parseDeviceStatus(res[3], ds) && ds.brightness == 80 && ds.sdCard,
              "all Ok; STATUS parses (brightness 80, SD present)");
        const FakeCounters fc = r.fake.counters();
        check(fc.page == 1 && fc.alert == 1 && fc.stopAudio == 1 && fc.status == 1 && fc.lastPageTitle == "DASHCAM LIVE",
              "the device saw each command once (page title \"" + fc.lastPageTitle + "\")");
        sleepMs(mode == 1 ? 1200 : 300);
        check(r.fake.counters().caps == 1 && r.link.state() == LinkState::ControlOnly,
              "CAPABILITIES is not asked again (" + std::to_string(r.fake.counters().caps) + ")");
    }
    // video = false: never asks.
    Rig r;
    DisplayLinkConfig c = r.cfg();
    c.video = false;
    check(r.up(c, LinkState::ControlOnly) && r.fake.counters().caps == 0, "video = false: control-only, no CAPABILITIES");
}

static void testFramesFlow() {
    std::printf("\n[4] frames flow and are presented\n");
    Rig r;
    check(r.up(r.cfg()), "negotiated");
    Producer p(r.link, smallFrames(), 25);
    p.start();
    sleepMs(1200);
    p.stop();
    check(waitFor([&] {
              const DisplayLinkStats s = r.link.stats();
              return s.framesStarted > 0 && s.framesPresented + s.framesDropped + s.framesLost == s.framesStarted;
          }, 1000),
          "every started frame resolved");
    const FakeCounters     fc = r.fake.counters();
    const DisplayLinkStats s  = r.link.stats();
    const auto             fr = r.fake.frames();
    const bool allGood = !fr.empty() && std::all_of(fr.begin(), fr.end(), [](const FakeFrame& f) {
        return f.decodeOk && f.decW == 320 && f.decH == 180 && f.w == 320 && f.h == 180;
    });
    check(fc.framesOk >= 25, std::to_string(fc.framesOk) + " frames assembled with a good CRC-32 (>= 25)");
    check(allGood, "every frame decodes as a 320x180 JPEG matching its START");
    check(fc.crcErrors == 0 && fc.badChunks == 0 && fc.badLength == 0 && fc.decodeFails == 0 && fc.dimMismatch == 0 &&
              fc.orphan == 0 && fc.noCredit == 0 && fc.startDuringAssembly == 0,
          "no CRC, offset, length, decode, size, credit or sequencing errors at the device");
    check(s.framesPresented == uint64_t(fc.presented) && s.framesStarted == uint64_t(fc.starts),
          "host and device agree: " + std::to_string(s.framesStarted) + " started, " +
              std::to_string(s.framesPresented) + " presented");
    check(s.framesAccepted == p.accepted.load() && p.ownershipBugs == 0 &&
              s.framesAccepted == s.framesReplaced + s.framesReleased + s.framesStarted,
          "accepted = replaced + released + started (" + std::to_string(s.framesAccepted) + ")");
    check(s.presentLatency.samples > 0 && s.presentLatency.p95Ms < 200.0 && s.captureLatency.samples > 0,
          "latency START->PRESENTED p50 " + std::to_string(s.presentLatency.p50Ms) + " / p95 " +
              std::to_string(s.presentLatency.p95Ms) + " ms; capture->PRESENTED p95 " +
              std::to_string(s.captureLatency.p95Ms) + " ms");
    check(s.lastDecodeUs > 0 && s.lastBlitUs == 2300, "decode/blit us from the last PRESENTED");
    check(s.resultReasons[es3video::ReasonOk] == uint64_t(fc.framesOk), "RESULT Ok counted per frame");
    check(p.pool->freeSlots() == p.pool->slotCount(), "every lease back in the pool");
    r.link.stop();
    check(r.fake.counters().stopVideo == 1, "stop() ends the session with STOP_VIDEO");
}

static void testStreamingRegressions() {
    std::printf("\n[23] TX fairness, duplicate acknowledgements, wake/close race\n");
    {
        Rig r;
        DisplayLinkConfig c = r.cfg();
        c.chunkDataBytes = 64;
        c.txQueueLimitBytes = 8192; // Let a batch exhaust its work budget before TIOCOUTQ gates it.
        c.assemblyTimeoutMs = 500;
        r.fake.duplicateOk = true;
        check(r.up(c), "small-chunk link negotiated");
        const Jpeg j = bigFrame(77);
        auto pool = EncodedFramePool::create(1, 65536);
        check(j.bytes.size() > 30000 && offerJpeg(r.link, *pool, j, nowUs()) == 1,
              "one large JPEG queued with no producer wakeups afterward");
        check(waitFor([&] { return r.link.stats().framesPresented == 1; }, 2000),
              "TX fairness yields to RX/timers without sleeping through assembly deadline");
        const auto s = r.link.stats();
        check(s.framesCompleted == 1 && s.framesDropped == 0 && s.unknownFrameRecords == 0,
              "duplicate RESULT Ok keeps the frame awaiting PRESENTED, counted once");
        check(r.fake.counters().assemblyTimeouts == 0 && pool->freeSlots() == 1,
              "no assembly timeout and the output lease returned");
    }
    {
        Rig r;
        r.fake.duplicateOk = true;
        check(r.up(r.cfg()), "duplicate-result link negotiated");
        const Jpeg j = smallFrames().front();
        auto pool = EncodedFramePool::create(1, 65536);
        check(offerJpeg(r.link, *pool, j, nowUs()) == 1, "frame queued");
        check(waitFor([&] { return r.fake.counters().presented == 1; }, 1000) &&
                  waitFor([&] { return r.link.stats().framesPresented == 1; }, 500),
              "replayed RESULT Ok does not discard the later PRESENTED");
        check(r.link.stats().framesCompleted == 1 && r.link.stats().framesDropped == 0,
              "one completion, no false drop");
    }
    {
        Rig r;
        check(r.up(r.cfg()), "wake-race link negotiated");
        wakeRace::entered = wakeRace::resume = false;
        std::atomic<bool> stopped{false}, accepted{false};
        std::thread producer([&] {
            wakeRace::intercept = true;
            accepted = r.link.enqueueControl(ControlRequest::status());
        });
        const bool intercepted = waitFor([] { return wakeRace::entered.load(); }, 1000);
        check(intercepted, "producer paused immediately before its eventfd syscall");
        std::thread stopper([&] { r.link.stop(); stopped = true; });
        // Old code closes the captured descriptor while this producer is paused.
        const bool premature = waitFor([&] { return stopped.load(); }, 600);
        check(intercepted && !premature && ::fcntl(wakeRace::fd.load(), F_GETFD) >= 0,
              "stop cannot close/reuse the descriptor underneath an in-flight wake");
        wakeRace::resume = true;
        producer.join();
        stopper.join();
        check(accepted && stopped, "producer and stop finish after wake completes");
    }
    {
        Rig r;
        r.fake.mapMode = 1;
        r.fake.neverCredit = true;
        auto c = r.cfg();
        c.creditStallMs = 400;
        c.map = true;
        check(r.up(c), "map credit-regression link negotiated");
        r.fake.setPage(5);
        check(waitFor([&] { return r.link.mapSink().demand().active; }, 2000), "map active");
        Producer p(r.link.mapSink(), {makeJpeg(320, 224, gradient(9), 65)}, 20);
        p.start();
        check(waitFor([&] { return r.link.stats().resyncs >= 1; }, 2000),
              "pending map with returned RESULT but missing CREDIT triggers resync");
        p.stop();
    }
}

static void testReplacement() {
    std::printf("\n[5] newest-frame replacement while waiting for a credit\n");
    Rig r;
    r.fake.neverCredit  = true;
    DisplayLinkConfig c = r.cfg();
    c.creditStallMs     = 10000;
    check(r.up(c), "negotiated");
    auto       pool = EncodedFramePool::create(4, es3video::kMaxJpegBytes);
    const Jpeg j    = makeJpeg(320, 180, gradient(6), 65);
    check(offerJpeg(r.link, *pool, j, 1000) == 1, "frame 1 accepted");
    check(waitFor([&] { return r.fake.counters().presented == 1; }, 1500), "frame 1 presented; grant used up");
    for (uint64_t ts = 2001; ts <= 2006; ++ts) {
        offerJpeg(r.link, *pool, j, ts);
        sleepMs(10);
    }
    sleepMs(50);
    DisplayLinkStats s = r.link.stats();
    check(s.framesReplaced == 5 && r.fake.counters().starts == 1,
          "frames 2..7 queued without a credit: 5 replaced, nothing started");
    check(pool->freeSlots() == 3, "the link holds exactly one pending lease");
    check(s.creditStalls >= 1, "the wait for a credit is counted");
    r.fake.neverCredit = false;
    r.fake.grantOne();
    check(waitFor([&] { return r.fake.counters().starts == 2; }, 1000), "a credit starts the next frame");
    check(r.fake.counters().lastStartTs == 2006, "... and it is the newest one (ts " +
                                                    std::to_string(r.fake.counters().lastStartTs) + ")");
    check(waitFor([&] { return pool->freeSlots() == 4; }, 1000), "all leases released after sending");
}

static void testCredits() {
    std::printf("\n[6] credits never exceed the grant; maxFramesInFlight 1 and 2\n");
    struct Case { int devCredits, initGrant, window, expect; };
    const Case cases[] = {{2, 2, 1, 1}, {2, 2, 2, 2}, {1, 1, 2, 1}};
    for (const Case& k : cases) {
        Rig r;
        r.fake.devMaxCredits  = k.devCredits;
        r.fake.initialGrant   = k.initGrant;
        r.fake.resultDelayMs  = 25;
        r.fake.presentDelayMs = 10;
        DisplayLinkConfig c   = r.cfg();
        c.maxFramesInFlight   = uint8_t(k.window);
        char what[96];
        std::snprintf(what, sizeof(what), "device credits %d, grant %d, maxFramesInFlight %d", k.devCredits,
                      k.initGrant, k.window);
        check(r.up(c), std::string(what) + ": negotiated");
        Producer p(r.link, smallFrames(), 4);
        p.start();
        sleepMs(1000);
        p.stop();
        sleepMs(150);
        const FakeCounters     fc = r.fake.counters();
        const DisplayLinkStats s  = r.link.stats();
        check(fc.noCredit == 0, "never a START beyond the grant (" + std::to_string(fc.starts) + " starts)");
        check(fc.maxInFlight == k.expect,
              "frames in flight at the device peaked at " + std::to_string(fc.maxInFlight) + " (expected " +
                  std::to_string(k.expect) + ")");
        check(s.creditWindow == uint8_t(std::min(k.window, k.devCredits)) && s.sessionStarted <= s.grant,
              "window " + std::to_string(s.creditWindow) + ", started " + std::to_string(s.sessionStarted) +
                  " <= grant " + std::to_string(s.grant));
        check(fc.framesOk >= 10, std::to_string(fc.framesOk) + " frames delivered");
    }
}

static void testCreditStall() {
    std::printf("\n[7] credit stall -> resync\n");
    Rig r;
    r.fake.neverCredit  = true;
    DisplayLinkConfig c = r.cfg();
    c.creditStallMs     = 400;
    check(r.up(c), "negotiated");
    Producer p(r.link, smallFrames(), 20);
    p.start();
    check(waitFor([&] { return r.link.stats().resyncs >= 1; }, 2000), "no credit for 400 ms -> resync");
    // resyncs counts the decision; the new START_VIDEO round trip follows it.
    check(waitFor([&] { return r.fake.counters().startVideo >= 2 && r.link.stats().sessions >= 2; }, 1000),
          "a new START_VIDEO / session");
    r.fake.neverCredit = false;
    const int before = r.fake.counters().presented;
    check(waitFor([&] { return r.fake.counters().presented >= before + 5; }, 2000),
          "frames flow again in the new session");
    p.stop();
    check(r.fake.counters().noCredit == 0 && r.fake.counters().staleFromHost == 0,
          "never beyond the grant, never an old-session record after the resync");
    check(r.logs.count("W ES3 display: no credit or RESULT") == 1, "the stall is logged (WARN once)");
}

static void testStaleRecords() {
    std::printf("\n[8] records with another session's token are ignored\n");
    Rig r;
    check(r.up(r.cfg()), "negotiated");
    Producer p(r.link, smallFrames(), 25);
    p.start();
    check(waitFor([&] { return r.fake.counters().framesOk >= 3; }, 1500), "frames flowing");
    for (int i = 0; i < 3; ++i) {
        r.fake.injectStaleRecords();
        sleepMs(50);
    }
    const int before = r.fake.counters().presented;
    sleepMs(500);
    const DisplayLinkStats s = r.link.stats();
    check(s.staleRecords >= 12, std::to_string(s.staleRecords) + " stale records ignored");
    check(s.state == LinkState::Active && r.link.demand().active, "a stale VISIBILITY hidden changed nothing");
    check(s.grant < 100 && r.fake.counters().noCredit == 0, "a stale CREDIT 100 was not used");
    check(r.fake.counters().presented >= before + 5, "frames kept flowing");
    p.stop();
}

static void testGarbage() {
    std::printf("\n[9] garbage on the line: the parser resynchronises\n");
    Rig r;
    check(r.up(r.cfg()), "negotiated");
    Producer p(r.link, smallFrames(), 25);
    p.start();
    Outcomes o;
    int      queued = 0;
    for (int i = 0; i < 20; ++i) {
        r.fake.injectGarbage(uint32_t(i + 1), 64);
        if (i % 4 == 0 && r.link.enqueueControl(withCb(ControlRequest::status(), o.cb()))) ++queued;
        sleepMs(50);
    }
    check(waitFor([&] { return int(o.size()) == queued; }, 1500), "every STATUS completed");
    const auto res = o.all();
    const bool ok  = std::all_of(res.begin(), res.end(), [](const ControlResult& x) { return x.status == ControlStatus::Ok; });
    p.stop();
    const DisplayLinkStats s = r.link.stats();
    check(ok && queued == 5, "STATUS replies parsed through the noise");
    check(s.disconnects == 0 && s.rehellos == 0 && s.parserGarbageBytes > 0,
          std::to_string(s.parserGarbageBytes) + " garbage bytes skipped, no reconnect or re-HELLO");
    check(r.fake.counters().framesOk >= 20, std::to_string(r.fake.counters().framesOk) + " frames presented meanwhile");
}

static void testBufferedReplyTimeoutRecovery() {
    std::printf("\n[9b] expired candidate preserves and dispatches an idle buffered reply\n");
    Rig r;
    auto cfg = r.cfg();
    cfg.heartbeatMs = 4000;
    cfg.loopReplyTimeoutMs = 700;
    check(r.up(cfg), "negotiated without video traffic");
    r.fake.injectStalledHeader();
    Outcomes result;
    check(r.link.enqueueControl(withCb(ControlRequest::status(), result.cb())), "STATUS queued behind a partial header");
    check(waitFor([&] { return result.size() == 1; }, 1200), "buffered reply delivered without another read");
    const auto replies = result.all();
    check(replies.size() == 1 && replies[0].status == ControlStatus::Ok && replies[0].latencyMs < 600,
          "STATUS recovered after parser expiry, before its command deadline");
    const auto stats = r.link.stats();
    check(stats.controlTimeouts == 0 && stats.rehellos == 0 && stats.connects == 1,
          "no request timeout or reconnect needed");
}

static void testVisibility() {
    std::printf("\n[10] VISIBILITY hidden / calibrating pauses video\n");
    const uint8_t modes[] = {es3video::DemandHidden, es3video::DemandCalibrating};
    for (uint8_t mode : modes) {
        Rig r;
        check(r.up(r.cfg()), std::string(mode == es3video::DemandHidden ? "hidden" : "calibrating") + ": negotiated");
        Producer p(r.link, smallFrames(), 20);
        p.start();
        check(waitFor([&] { return r.fake.counters().framesOk >= 3; }, 1500), "frames flowing");
        r.fake.setVisibility(mode);
        check(waitFor([&] { return r.link.state() == LinkState::Paused && !r.link.demand().active; }, 300),
              "paused, demand inactive");
        sleepMs(600);
        const FakeCounters fc = r.fake.counters();
        check(fc.startsWhileHidden == 0 && fc.bytesWhileHidden == 0,
              "no video bytes while hidden (" + std::to_string(fc.bytesWhileHidden) + ")");
        auto pool = EncodedFramePool::create(1, es3video::kMaxJpegBytes);
        check(offerJpeg(r.link, *pool, makeJpeg(320, 180, gradient(1), 65), 1) == 0, "offers refused while hidden");
        const int before = fc.framesOk;
        r.fake.setVisibility(es3video::DemandActive);
        check(waitFor([&] { return r.link.state() == LinkState::Active && r.link.demand().active; }, 300),
              "active again");
        check(waitFor([&] { return r.fake.counters().framesOk >= before + 3; }, 1500), "frames resume");
        p.stop();
    }
}

static void testControlPriority() {
    std::printf("\n[11] control latency under video load; bounded kernel queue\n");
    Rig r;
    r.fake.readRate     = 200000;   // a slow USB link: one 50 KB frame takes 250 ms
    DisplayLinkConfig c = r.cfg();
    c.assemblyTimeoutMs = 3000;
    check(r.up(c), "negotiated");
    Producer p(r.link, {bigFrame(1), bigFrame(2)}, 10);
    p.start();
    check(waitFor([&] { return r.fake.counters().starts >= 1; }, 1500), "video streaming");
    sleepMs(100);
    outq::resetMax();
    Outcomes o;
    double   worst = 0, sum = 0;
    int      n = 0;
    for (int i = 0; i < 20; ++i) {
        const size_t before = o.size();
        if (!r.link.enqueueControl(withCb(ControlRequest::status(), o.cb()))) continue;
        if (!waitFor([&] { return o.size() > before; }, 1000)) break;
        const ControlResult x = o.all().back();
        if (x.status != ControlStatus::Ok) break;
        worst = std::max(worst, x.latencyMs);
        sum += x.latencyMs;
        ++n;
        sleepMs(40);
    }
    const FakeCounters fc = r.fake.counters();
    p.stop();
    const DisplayLinkStats s = r.link.stats();
    check(n == 20, std::to_string(n) + "/20 STATUS requests answered Ok");
    char line[160];
    std::snprintf(line, sizeof(line), "control latency while streaming: worst %.1f ms, mean %.1f ms (< 100 ms)", worst,
                  n ? sum / n : 0.0);
    check(n > 0 && worst < 100.0, line);
    check(fc.presented >= 3, std::to_string(fc.presented) + " big frames presented meanwhile");
    const int64_t bound = 2 * kRecordBytes + 2 * int64_t(es3proto::kMaxFrame);
    check(outq::maxQueued.load() <= bound, "kernel queue peaked at " + std::to_string(outq::maxQueued.load()) +
                                               " B (bound " + std::to_string(bound) + ")");
    check(s.kernelQueueWaits > 0 && s.kernelQueueLimit == kRecordBytes,
          "video records were held for TIOCOUTQ (" + std::to_string(s.kernelQueueWaits) + " waits, limit " +
              std::to_string(s.kernelQueueLimit) + " B)");
}

static void testSlowReader() {
    std::printf("\n[12] a stalled reader never blocks offerFrame() / enqueueControl()\n");
    Rig r;
    DisplayLinkConfig c = r.cfg();
    c.replyTimeoutMs    = 1500;
    check(r.up(c), "negotiated");
    Producer p(r.link, {bigFrame(3), bigFrame(4)}, 10);
    p.start();
    check(waitFor([&] { return r.fake.counters().framesOk >= 2; }, 2000), "frames flowing");
    outq::resetMax();
    p.maxOfferUs = 0;
    const int before = r.fake.counters().framesOk;
    r.fake.stallUntilMs = nowMs() + 500;   // outlasts the 400 ms window below
    Outcomes o;
    int      accepted = 0, full = 0;
    int64_t  worstEnqUs = 0;
    const int64_t until = nowMs() + 400;
    while (nowMs() < until) {
        const int64_t t0 = nowUs();
        const bool    ok = r.link.enqueueControl(withCb(ControlRequest::page(1, "TRIP", {"a", "b", "c", "d"}), o.cb()));
        worstEnqUs = std::max(worstEnqUs, nowUs() - t0);
        (ok ? accepted : full)++;
        sleepMs(15);
    }
    check(worstEnqUs < 20000 && p.maxOfferUs.load() < 20000,
          "enqueueControl() worst " + std::to_string(worstEnqUs) + " us, offerFrame() worst " +
              std::to_string(p.maxOfferUs.load()) + " us during the stall");
    // A request the fake had already read when the stall began can still be
    // answered inside the window, freeing its slot: count only the requests
    // still outstanding (queued + the one in flight) when the window closed.
    const int doneInWindow = int(o.size());
    check(full > 0 && accepted - doneInWindow <= int(kControlQueueDepth) + 1,
          std::to_string(accepted) + " queued (" + std::to_string(doneInWindow) + " answered inside the window), " +
              std::to_string(full) + " rejected when the queue was full");
    const int64_t bound = 2 * kRecordBytes + 2 * int64_t(es3proto::kMaxFrame);
    check(outq::maxQueued.load() <= bound, "kernel queue peaked at " + std::to_string(outq::maxQueued.load()) +
                                               " B during the stall (bound " + std::to_string(bound) + ")");
    check(waitFor([&] { return int(o.size()) == accepted; }, 3000), "every accepted request completed once");
    const auto res = o.all();
    check(std::all_of(res.begin(), res.end(), [](const ControlResult& x) { return x.status == ControlStatus::Ok; }) &&
              r.fake.counters().page == accepted,
          "... Ok, and the device saw exactly those " + std::to_string(accepted));
    check(waitFor([&] { return r.fake.counters().framesOk >= before + 3; }, 2000), "frames resume after the stall");
    p.stop();
    const DisplayLinkStats s = r.link.stats();
    check(s.disconnects == 0 && s.controlTimeouts == 0 && s.controlQueueFull == uint64_t(full),
          "no reconnect, no timeout; queue-full rejections counted");
}

static void testHeartbeat() {
    std::printf("\n[13] heartbeats keep the device session alive\n");
    Rig r;
    r.fake.sessionTimeoutMs = 1000;
    DisplayLinkConfig c     = r.cfg();
    c.heartbeatMs           = 300;
    check(r.up(c), "negotiated");
    sleepMs(2500);
    const FakeCounters fc = r.fake.counters();
    check(fc.ping >= 6 && fc.ping <= 10, std::to_string(fc.ping) + " PINGs in 2.5 s at 300 ms");
    check(fc.sessionExpiries == 0 && fc.hello == 1 && fc.noSession == 0 && r.link.stats().rehellos == 0,
          "the session never expired (1 s device timeout)");
}

static void testTimeouts() {
    std::printf("\n[14] reply timeout, NoSession and a lost HELLO reply restart the handshake\n");
    Rig r;
    DisplayLinkConfig c  = r.cfg();
    c.replyTimeoutMs     = 300;
    c.loopReplyTimeoutMs = 300;   // STATUS is a loop command
    check(r.up(c), "negotiated");
    Outcomes o;
    r.fake.dropReplyTo = es3proto::Status;
    check(r.link.enqueueControl(withCb(ControlRequest::status(), o.cb())), "STATUS queued (its reply will be lost)");
    check(waitFor([&] { return o.size() == 1; }, 1500), "the request completes");
    ControlResult x = o.all().back();
    check(x.status == ControlStatus::Timeout && x.latencyMs >= 290,
          std::string("... as Timeout after ") + std::to_string(int(x.latencyMs)) + " ms");
    check(waitFor([&] { return r.fake.counters().hello == 2 && r.link.state() == LinkState::Negotiated; }, 1000),
          "HELLO again; negotiated (capabilities are not re-asked)");
    check(r.fake.counters().caps == 1 && r.fake.counters().status == 1,
          "the timed-out STATUS was not replayed, CAPABILITIES asked once");
    DisplayLinkStats s = r.link.stats();
    check(s.controlTimeouts == 1 && s.rehellos == 1, "one timeout, one re-HELLO");
    check(r.link.enqueueControl(withCb(ControlRequest::status(), o.cb())) &&
              waitFor([&] { return o.size() == 2 && o.all().back().status == ControlStatus::Ok; }, 1000),
          "STATUS works afterwards");

    r.fake.expireSession();
    check(r.link.enqueueControl(withCb(ControlRequest::status(), o.cb())) && waitFor([&] { return o.size() == 3; }, 1000),
          "STATUS after the device dropped our session completes");
    x = o.all().back();
    check(x.status == ControlStatus::DeviceError && x.errorCode == es3proto::NoSession, "... with NoSession");
    check(waitFor([&] { return r.fake.counters().hello == 3 && r.link.state() == LinkState::Negotiated; }, 1000) &&
              r.link.stats().noSessionErrors == 1,
          "NoSession -> HELLO again");

    // Lost HELLO reply: the link reopens the tty after reconnectMs.
    r.fake.dropReplyTo = es3proto::Hello;
    r.fake.expireSession();
    r.link.enqueueControl(ControlRequest::ping());
    check(waitFor([&] { return r.link.stats().helloTimeouts == 1; }, 1500), "a lost HELLO reply times out");
    check(waitFor([&] { return r.link.state() == LinkState::Negotiated && r.link.stats().connects == 2; }, 1500),
          "the tty is reopened and the handshake succeeds");
    s = r.link.stats();
    check(s.disconnects == 1 && s.opens == 2, "one close / reopen");

    // Video still works after all that.
    Producer p(r.link, smallFrames(), 25);
    p.start();
    check(waitFor([&] { return r.fake.counters().presented >= 3; }, 1500), "video negotiates and flows afterwards");
    p.stop();
}

static void testUnplug() {
    std::printf("\n[15] unplug -> disconnected, leases released; replug -> video resumes\n");
    Rig r;
    check(r.up(r.cfg()), "negotiated");
    Producer p(r.link, smallFrames(), 20);
    p.start();
    check(waitFor([&] { return r.fake.counters().framesOk >= 5; }, 1500), "frames flowing");
    r.fake.unplug();
    check(waitFor([&] { return r.link.state() == LinkState::Disconnected; }, 500), "unplug -> disconnected");
    sleepMs(100);
    p.stop();
    check(p.pool->freeSlots() == p.pool->slotCount(), "every lease back in the pool");
    DisplayLinkStats s = r.link.stats();
    check(s.disconnects == 1 && !r.link.demand().active, "one disconnect, demand inactive");
    check(!r.link.enqueueControl(ControlRequest::status()), "controls rejected while unplugged");
    check(r.logs.count("W ES3 display: " + r.dev + " closed") == 1, "the loss is logged as WARN");
    check(r.fake.plug(r.dev), "replug (a new pty behind the symlink)");
    check(waitFor([&] { return r.link.state() == LinkState::Negotiated; }, 1500), "renegotiated");
    p.start();
    check(waitFor([&] { return r.fake.counters().framesOk >= 5; }, 2000), "video resumes on the new device");
    p.stop();
    s = r.link.stats();
    check(s.connects == 2 && s.sessions >= 2, "two connects, a new session");
}

static void testStopBounded() {
    std::printf("\n[16] stop() is bounded with traffic in flight\n");
    Rig r;
    r.fake.readRate     = 30000;
    DisplayLinkConfig c = r.cfg();
    c.assemblyTimeoutMs = 5000;
    check(r.up(c), "negotiated");
    Producer p(r.link, {bigFrame(5)}, 10);
    p.start();
    check(waitFor([&] { return r.fake.counters().starts >= 1; }, 1500), "a big frame in flight on a slow link");
    Outcomes o;
    int      queued = 0;
    for (int i = 0; i < 3; ++i) queued += r.link.enqueueControl(withCb(ControlRequest::status(), o.cb())) ? 1 : 0;
    const int64_t t0 = nowUs();
    r.link.stop();
    const double ms = double(nowUs() - t0) / 1000.0;
    p.stop();
    check(ms < 1000.0, "stop() took " + std::to_string(int(ms)) + " ms (< 1000)");
    check(p.pool->freeSlots() == p.pool->slotCount(), "every lease released");
    check(queued == 3 && o.size() == 3, "each queued control completed exactly once");
    const auto res = o.all();
    check(std::all_of(res.begin(), res.end(), [](const ControlResult& x) {
              return x.status == ControlStatus::Ok || x.status == ControlStatus::Cancelled;
          }),
          "... as Ok or Cancelled");
    check(r.link.state() == LinkState::Disconnected && !r.link.demand().active, "disconnected afterwards");
}

static void testEndToEnd() {
    std::printf("\n[17] end to end: 1080p JPEG -> VideoPreview -> Es3DisplayLink -> fake display\n");
    Rig r;
    check(r.up(r.cfg()), "negotiated");
    const Jpeg src = makeJpeg(1920, 1080, gradient(9), 85);
    dashcam::preview::VideoPreview  pv;
    dashcam::preview::PreviewConfig pc;
    check(pv.start(pc, r.link, r.logs.callback()), "the converter starts with the link as its sink");
    for (int i = 0; i < 25; ++i) {
        pv.submitJpeg(src.bytes.data(), src.bytes.size(), 1920, 1080, uint64_t(nowUs()));
        sleepMs(34);
    }
    check(waitFor([&] { return r.fake.counters().framesOk >= 5; }, 2000),
          std::to_string(r.fake.counters().framesOk) + " converted frames reached the display");
    pv.stop();
    const auto fr = r.fake.frames();
    check(!fr.empty() && std::all_of(fr.begin(), fr.end(), [](const FakeFrame& f) {
              return f.decodeOk && f.decW == 320 && f.decH == 180;
          }),
          "each decodes as 320x180");
    const es3video::StartVideoRequest q = r.fake.lastStartVideo();
    check(q.viewportX == 0 && q.viewportY == 30 && q.viewportW == 320 && q.viewportH == 180,
          "negotiated viewport 0,30 320x180 (letterbox)");
    const dashcam::preview::PreviewStats ps = pv.stats();
    check(ps.offered >= 5 && ps.refused == 0, std::to_string(ps.offered) + " offered by the converter, none refused");
    check(r.link.stats().framesPresented >= 5 && r.link.stats().captureLatency.samples > 0,
          "presented, capture->PRESENTED p95 " + std::to_string(r.link.stats().captureLatency.p95Ms) + " ms");
}

static void testRequests() {
    std::printf("\n[18] control request factories\n");
    const ControlRequest p = ControlRequest::page(2, "CAMERA", {"one", "two", "three", "four"});
    check(p.valid() && p.command == es3proto::Page && p.length == 121 && p.payload[0] == 2 &&
              std::memcmp(p.payload + 1, "CAMERA", 6) == 0 && p.payload[7] == 0 &&
              std::memcmp(p.payload + 1 + 4 * 24, "four", 4) == 0,
          "PAGE: index + five NUL-padded 24-byte fields");
    check(!ControlRequest::page(8, "X", {"", "", "", ""}).valid() && ControlRequest::page(5, "MAP", {"", "", "", ""}).valid(),
          "page index 8 rejected by the factory, 5 (MAP) built");
    check(!ControlRequest::page(0, std::string(24, 'A'), {"", "", "", ""}).valid(), "24-char title rejected");
    check(ControlRequest::page(0, std::string(23, 'A'), {"", "", "", ""}).valid(), "23-char title accepted");
    check(!ControlRequest::page(0, "tab\there", {"", "", "", ""}).valid(), "non-printable text rejected");
    check(!ControlRequest::alert(101).valid() && ControlRequest::alert(100).valid(), "alert volume 0-100");
    check(ControlRequest::status().command == es3proto::Status && ControlRequest::ping().command == es3proto::Ping &&
              ControlRequest::stopAudio().command == es3proto::StopAudio,
          "status / ping / stop audio");
    Rig r;
    check(r.up(r.cfg()), "negotiated");
    ControlRequest hello;
    hello.command = es3proto::Hello;
    ControlRequest video;
    video.command = es3proto::StartVideo;
    check(!r.link.enqueueControl(hello) && !r.link.enqueueControl(video) &&
              !r.link.enqueueControl(ControlRequest::alert(200)),
          "protocol-internal commands and invalid requests are rejected");
    check(!r.link.enqueueControl(ControlRequest::page(5, "MAP", {"", "", "", ""})),
          "page 5 refused by a link whose board has no MAP tab");
    check(r.link.stats().controlRejected == 4, "rejections counted");
}

static std::vector<Jpeg> mapFrames() {
    return {makeJpeg(320, 224, gradient(7), 80), makeJpeg(320, 224, gradient(8), 80)};
}

static void testMapSurface() {
    std::printf("\n[20] map surface: MAP_CAPABILITIES, page-driven camera/map switching, single receiver\n");
    {   // Firmware without the extension: the map stays off, the camera is unchanged.
        Rig r;
        DisplayLinkConfig c = r.cfg();
        c.map               = true;
        check(r.up(c), "negotiated (old firmware)");
        check(waitFor([&] { return r.fake.counters().mapCaps == 1; }, 1000), "MAP_CAPABILITIES asked once");
        sleepMs(300);
        const DisplayLinkStats s = r.link.stats();
        const auto md = r.link.mapSink().demand();
        check(!s.mapSupported && !md.active && md.canvasW == 0 && s.pagePolls == 0,
              "BadCommand: no map, map sink inactive with no canvas, no page polling");
        Producer p(r.link, smallFrames(), 30);
        p.start();
        check(waitFor([&] { return r.fake.counters().framesOk >= 3; }, 2000), "camera preview flows as before");
        p.stop();
        check(!r.link.enqueueControl(ControlRequest::page(5, "MAP", {"", "", "", ""})), "page 5 refused");
    }
    {
        Rig r;
        r.fake.mapMode      = 1;
        DisplayLinkConfig c = r.cfg();
        c.map               = true;
        check(r.up(c), "negotiated (map firmware)");
        check(waitFor([&] { return r.link.stats().mapSupported && r.link.stats().activePage == 0; }, 1500),
              "map v1 found; first STATUS says page 0 (LIVE)");
        DisplayLinkStats s = r.link.stats();
        check(s.mapCapabilities.mapPage == 5 && s.mapCapabilities.viewportH == 224 && s.mapCapabilities.pageCount == 6,
              "map page 5 of 6, viewport 320x224");
        auto md = r.link.mapSink().demand();
        check(!md.active && md.canvasW == 320 && md.canvasH == 224 && md.maxJpegBytes == 65536,
              "on LIVE: map sink inactive, but reports its 320x224 canvas (warm rendering)");
        check(r.link.enqueueControl(ControlRequest::page(5, "MAP", {"", "", "", ""})), "page 5 accepted");

        Producer cam(r.link, smallFrames(), 30);
        Producer map(r.link.mapSink(), mapFrames(), 100);
        cam.start();
        map.start();
        check(waitFor([&] { return r.fake.counters().framesOk >= 3; }, 2000), "LIVE: camera frames flow");
        check(map.accepted.load() == 0, "LIVE: map frames refused (the caller keeps them)");

        // The user taps MAP.
        const int64_t t0       = nowMs();
        const int     camBefore = r.fake.counters().framesOk;
        r.fake.setPage(5);
        bool firstMap = waitFor([&] {
            for (const FakeFrame& f : r.fake.frames())
                if (f.map) return true;
            return false;
        }, 2000);
        const int64_t switchMs = nowMs() - t0;
        check(firstMap, "MAP tab: STOP_VIDEO, START_MAP, map frames arrive (" + std::to_string(switchMs) + " ms)");
        check(switchMs < 600, "switch to the first map frame within 600 ms (poll 250 ms + two round trips)");
        sleepMs(600);
        int camAfter = 0, mapOk = 0;
        bool sizesOk = true;
        for (const FakeFrame& f : r.fake.frames()) {
            if (f.map) {
                ++mapOk;
                sizesOk = sizesOk && f.decodeOk && f.decW == 320 && f.decH == 224;
            } else {
                ++camAfter;
            }
        }
        check(mapOk >= 3 && sizesOk, std::to_string(mapOk) + " map frames, 320x224, decoded");
        check(camAfter <= camBefore + 1, "no camera frame after the switch (" + std::to_string(camAfter - camBefore) +
                                             " in flight at most)");
        s = r.link.stats();
        check(r.fake.counters().stopVideo >= 1 && r.fake.counters().startMap == 1 && r.fake.counters().busyRefusals == 0 &&
                  s.sessionKind == 2 && s.sourceSwitches == 1,
              "one switch, the camera stopped before the map started (no NotReady)");
        check(!r.link.demand().active && r.link.mapSink().demand().active,
              "camera demand inactive (converter idle), map demand active");

        // A wrong-size map frame is refused.
        auto pool = EncodedFramePool::create(1, es3video::kMaxJpegBytes);
        const Jpeg wrong = makeJpeg(320, 180, gradient(3), 80);
        check(offerJpeg(r.link.mapSink(), *pool, wrong, nowUs()) == 0, "a 320x180 frame is refused by the map sink");

        // TRIP: the map session stays, paused; nothing is sent.
        r.fake.setPage(1);
        check(waitFor([&] { return r.link.state() == LinkState::Paused; }, 1000), "TRIP: paused (VISIBILITY hidden)");
        sleepMs(400);
        const int64_t hiddenBytes = r.fake.counters().bytesWhileHidden;
        sleepMs(600);
        check(r.fake.counters().bytesWhileHidden == hiddenBytes && r.link.stats().sourceSwitches == 1,
              "no image bytes while another tab shows, no switch");
        check(!r.link.mapSink().demand().active && !r.link.demand().active, "both demands inactive");

        // Back to LIVE: STOP_MAP, START_VIDEO, camera frames again.
        const size_t nFrames = r.fake.frames().size();
        r.fake.setPage(0);
        check(waitFor([&] {
                  const auto fr = r.fake.frames();
                  for (size_t i = nFrames; i < fr.size(); ++i)
                      if (!fr[i].map) return true;
                  return false;
              }, 2000),
              "LIVE again: STOP_MAP, START_VIDEO, camera frames");
        check(r.fake.counters().stopMap >= 1 && r.link.stats().sourceSwitches == 2 && r.link.stats().sessionKind == 1 &&
                  r.fake.counters().busyRefusals == 0,
              "second switch clean");
        cam.stop();
        map.stop();
        check(cam.ownershipBugs.load() == 0 && map.ownershipBugs.load() == 0, "lease ownership kept on every refusal");

        // A new HELLO (timeout) restarts from LIVE with the camera; map known.
        s = r.link.stats();
        check(s.pagePolls >= 4, std::to_string(s.pagePolls) + " page polls");
    }
}

/// Offers @p j through a panning sink as a background (origin) or chrome.
static int offerPan(dashcam::navigation::IMapPanSink& sink, EncodedFramePool& pool, const Jpeg& j, bool background,
                    uint32_t ox = 0, uint32_t oy = 0) {
    EncodedFrameLease l = pool.tryAcquire();
    if (!l) return -1;
    std::memcpy(l.writableData(), j.bytes.data(), j.bytes.size());
    l.setFrame(j.bytes.size(), j.w, j.h, 0, 0, 80);
    const bool ok = background ? sink.offerPanBackground(std::move(l), ox, oy) : sink.offerPanChrome(std::move(l));
    if (!ok && !l.valid()) return -2;
    return ok ? 1 : 0;
}

static Pattern noise(uint32_t seed) {
    return [seed](uint32_t x, uint32_t y, uint8_t rgb[3]) {
        uint32_t v = (x * 73856093u) ^ (y * 19349663u) ^ (seed * 83492791u);
        v ^= v >> 13;
        v *= 0x5bd1e995u;
        rgb[0] = uint8_t(v), rgb[1] = uint8_t(v >> 8), rgb[2] = uint8_t(v >> 16);
    };
}

static void testPanSurface() {
    std::printf("\n[21] board-side panning: PAN_CAPABILITIES, START_PAN, background/chrome images, MAP_POSE\n");
    using dashcam::navigation::Marker;
    using dashcam::navigation::PanPose;
    {
        Rig r;
        r.fake.mapMode      = 1;
        r.fake.panMode      = 1;
        DisplayLinkConfig c = r.cfg();
        c.map               = true;
        check(r.up(c), "negotiated (panning firmware)");
        check(waitFor([&] { return r.link.stats().panSupported && r.link.stats().activePage == 0; }, 1500),
              "pan v1 found after MAP_CAPABILITIES");
        DisplayLinkStats s = r.link.stats();
        check(s.panCapabilities.maxBackgroundW == 640 && s.panCapabilities.chromeTopH == 16 &&
                  s.panCapabilities.maxPoseHz == 30 && r.fake.counters().panCaps == 1,
              "PAN_CAPABILITIES asked once: 640x480, strips 16+16, 30 poses/s");
        check(!r.link.panSink().panDemand().active && r.link.panSink().panDemand().session == 0,
              "on LIVE: no panning session yet");
        PanPose early;
        early.marker = Marker::Dot;
        check(!r.link.panSink().offerPanPose(early), "a pose without a shown panning session is refused");

        r.fake.setPage(5);
        check(waitFor([&] { return r.link.panSink().panDemand().active; }, 1500), "MAP: panning session, demand active");
        const auto pd = r.link.panSink().panDemand();
        check(pd.backgroundW == 512 && pd.backgroundH == 384 && pd.mapW == 320 && pd.mapH == 192 &&
                  pd.chromeTopH == 16 && pd.chromeBottomH == 16 && pd.maxJpegBytes == 65536 && pd.maxPoseHz == 30 &&
                  pd.session != 0,
              "pan demand: background 512x384, map area 320x192, chrome 320x32, 30 poses/s");
        check(r.fake.counters().startPan == 1 && r.fake.counters().startMap == 0 && !r.link.mapSink().demand().active &&
                  r.link.stats().panSession,
              "START_PAN (not START_MAP); the whole-frame map sink stays inactive");

        auto pool = EncodedFramePool::create(4, es3video::kMaxJpegBytes);
        const Jpeg bg     = makeJpeg(512, 384, gradient(4), 80);
        const Jpeg chrome = makeJpeg(320, 32, gradient(5), 80);
        check(offerPan(r.link.panSink(), *pool, bg, true, 1000, 2000) == 1, "background accepted");
        check(offerPan(r.link.panSink(), *pool, chrome, false) == 1, "chrome accepted");
        check(offerPan(r.link.panSink(), *pool, makeJpeg(320, 224, gradient(6), 80), true) == 0 &&
                  offerPan(r.link.panSink(), *pool, makeJpeg(320, 16, gradient(6), 80), false) == 0,
              "wrong-size background / chrome refused (the caller keeps them)");
        check(offerJpeg(r.link.mapSink(), *pool, makeJpeg(320, 224, gradient(7), 80), nowUs()) == 0,
              "a whole map frame is refused during a panning session");
        check(waitFor([&] {
                  int bgs = 0, chromes = 0;
                  for (const FakeFrame& f : r.fake.frames()) {
                      bgs += f.panKind == es3map::PanImageBackground;
                      chromes += f.panKind == es3map::PanImageChrome;
                  }
                  return bgs == 1 && chromes == 1;
              }, 2000),
              "one background and one chrome assembled");
        bool imagesOk = true;
        for (const FakeFrame& f : r.fake.frames()) {
            if (f.panKind == es3map::PanImageBackground)
                imagesOk = imagesOk && f.decodeOk && f.decW == 512 && f.decH == 384 &&
                           f.ts == es3map::packOrigin(1000, 2000);
            if (f.panKind == es3map::PanImageChrome) imagesOk = imagesOk && f.decodeOk && f.decW == 320 && f.decH == 32;
        }
        check(imagesOk, "decoded at 512x384 / 320x32; the background START carries its origin");

        // Poses offered at ~100 Hz: the link sends the newest at most 30 times a second.
        PanPose pose;
        pose.marker = Marker::Arrow;
        const int64_t t0 = nowMs();
        int offered = 0;
        while (nowMs() - t0 < 1000) {
            pose.xQ4 = (1000 + 256) * 16 + uint32_t(offered);
            pose.yQ4 = (2000 + 192) * 16;
            pose.courseCdeg = uint16_t((offered * 100) % 36000);
            if (r.link.panSink().offerPanPose(pose)) ++offered;
            sleepMs(10);
        }
        sleepMs(150);
        FakeCounters fc = r.fake.counters();
        s = r.link.stats();
        check(offered >= 80, std::to_string(offered) + " poses offered");
        check(fc.poses >= 20 && fc.poses <= 33, std::to_string(fc.poses) + " MAP_POSE records in ~1 s (cap 30/s)");
        check(fc.poseSeqErrors == 0 && fc.poseBadFields == 0 && fc.posesOutsideSession == 0,
              "seq increasing, fields valid");
        check(fc.lastPose.xQ4 == pose.xQ4 && fc.lastPose.courseCdeg == pose.courseCdeg &&
                  fc.lastPose.marker == es3map::MarkerArrow,
              "the newest pose arrived last");
        check(s.posesReplaced > 0 && s.posesSent == uint64_t(fc.poses), "older poses replaced, not queued");

        // A large background over a slow reader: poses keep coming while it streams.
        r.fake.readRate = 120000;
        Jpeg big;
        for (int q = 60; q >= 10; q -= 5) {   // the largest noisy background that still fits
            big = makeJpeg(512, 384, noise(3), q);
            if (big.bytes.size() <= 60000) break;
        }
        check(big.bytes.size() > 40000 && big.bytes.size() <= 65536,
              "noisy background " + std::to_string(big.bytes.size()) + " B");
        const int posesBefore = r.fake.counters().poses;
        check(offerPan(r.link.panSink(), *pool, big, true, 1100, 2000) == 1, "large background accepted");
        const int64_t t1 = nowMs();
        while (nowMs() - t1 < 500) {
            pose.xQ4 += 16;
            r.link.panSink().offerPanPose(pose);
            sleepMs(20);
        }
        check(waitFor([&] {
                  int bgs = 0;
                  for (const FakeFrame& f : r.fake.frames()) bgs += f.panKind == es3map::PanImageBackground;
                  return bgs == 2;
              }, 3000),
              "large background assembled");
        check(r.fake.counters().poses - posesBefore >= 8,
              std::to_string(r.fake.counters().poses - posesBefore) + " poses interleaved with the background");
        r.fake.readRate = 0;

        // TRIP: paused; no poses or images go out; demand inactive, same session.
        const uint32_t session = r.link.panSink().panDemand().session;
        r.fake.setPage(1);
        check(waitFor([&] { return !r.link.panSink().panDemand().active; }, 1000), "TRIP: pan demand inactive");
        check(!r.link.panSink().offerPanPose(pose) && offerPan(r.link.panSink(), *pool, bg, true) == 0,
              "poses and images refused while hidden");
        sleepMs(300);
        check(r.fake.counters().posesWhileHidden == 0 && r.fake.counters().bytesWhileHidden == 0, "nothing sent while hidden");
        const uint32_t resendBefore = r.link.panSink().panDemand().resend;
        const int64_t  shownAt      = nowMs();
        r.fake.setPage(5);
        check(waitFor([&] { return r.link.panSink().panDemand().active; }, 1000) &&
                  r.link.panSink().panDemand().session == session && r.fake.counters().startPan == 1,
              "MAP again: the same session resumes");
        const int64_t backMs = nowMs() - shownAt;
        check(backMs < 150, "active again " + std::to_string(backMs) +
                                " ms after the tab (VISIBILITY tells the page; no STATUS poll wait)");
        check(r.link.panSink().panDemand().resend != resendBefore,
              "and the producer is asked to resend (the board dropped its images when hidden)");
        // A pan image the board fails to decode: resend asked for too.
        const uint32_t r1 = r.link.panSink().panDemand().resend;
        r.fake.rejectNextPan = true;
        check(offerPan(r.link.panSink(), *pool, chrome, false) == 1, "chrome offered");
        check(waitFor([&] { return r.link.panSink().panDemand().resend != r1; }, 1500),
              "an image the board rejected: the producer is asked to resend");

        // LIVE: the camera takes the receiver back.
        Producer cam(r.link, smallFrames(), 30);
        cam.start();
        r.fake.setPage(0);
        check(waitFor([&] {
                  for (const FakeFrame& f : r.fake.frames())
                      if (!f.map) return true;
                  return false;
              }, 2000),
              "LIVE: STOP_MAP, START_VIDEO, camera frames");
        cam.stop();
        check(!r.link.panSink().panDemand().active && r.link.panSink().panDemand().session == 0 &&
                  r.fake.counters().busyRefusals == 0,
              "pan demand gone with the session");
    }
    {   // Advertised but refused: whole frames on this connection.
        Rig r;
        r.fake.mapMode      = 1;
        r.fake.panMode      = 2;
        DisplayLinkConfig c = r.cfg();
        c.map               = true;
        check(r.up(c), "negotiated (START_PAN refused)");
        check(waitFor([&] { return r.link.stats().panSupported; }, 1500), "pan advertised");
        r.fake.setPage(5);
        check(waitFor([&] { return r.link.mapSink().demand().active; }, 2500),
              "START_PAN refused -> START_MAP, whole-frame demand active");
        const DisplayLinkStats s = r.link.stats();
        check(s.panFallbacks == 1 && r.fake.counters().startPanRefused == 1 && r.fake.counters().startMap == 1 &&
                  !s.panSession && !r.link.panSink().panDemand().active,
              "one fallback, no panning session");
    }
    {   // mapPan off: never asks.
        Rig r;
        r.fake.mapMode      = 1;
        r.fake.panMode      = 1;
        DisplayLinkConfig c = r.cfg();
        c.map               = true;
        c.mapPan            = false;
        check(r.up(c), "negotiated (mapPan off)");
        r.fake.setPage(5);
        check(waitFor([&] { return r.link.mapSink().demand().active; }, 2000), "whole-frame map session");
        check(r.fake.counters().panCaps == 0 && r.fake.counters().startPan == 0, "PAN_CAPABILITIES never asked");
    }
}

static void testPanEndToEnd() {
    std::printf("\n[22] end to end: NavigationService -> Es3DisplayLink -> fake panning board, over the pty\n");
    Rig r;
    r.fake.mapMode      = 1;
    r.fake.panMode      = 1;
    DisplayLinkConfig c = r.cfg();
    c.map               = true;
    check(r.up(c), "negotiated (panning firmware)");
    check(waitFor([&] { return r.link.stats().panSupported; }, 1500), "pan capabilities known");
    r.fake.setPage(5);
    check(waitFor([&] { return r.link.panSink().panDemand().active; }, 1500), "MAP tab: panning session");

    dashcam::navigation::NavigationService nav;
    dashcam::navigation::NavigationServiceConfig nc;
    nc.providerUrl = "http://127.0.0.1:9/{z}/{x}/{y}.png";   // refused: tiles stay missing (hatched), no network
    nc.poseHz      = 20;
    check(nav.start(nc, r.link.mapSink(), &r.link.panSink()), "navigation starts on the link's sinks");
    std::atomic<bool> run{true};
    std::thread feed([&] {   // 10 Hz telemetry carrying 4 Hz fixes, east at 100 km/h
        const double lat = 48.8584, lon0 = 2.2945;
        const int64_t t0 = nowMs();
        int64_t fixMs = 0;
        double lon = lon0;
        while (run) {
            const int64_t now = nowMs();
            if (now - fixMs >= 250) {
                fixMs = now;
                lon = lon0 + 100.0 / 3.6 * double(now - t0) / 1000.0 / (111320.0 * std::cos(lat * M_PI / 180.0));
            }
            dashcam::navigation::GpsSample s;
            s.latitude = lat, s.longitude = lon, s.speedKmh = 100.0f, s.courseDeg = 90.0f;
            s.fixType = 3, s.satellites = 9, s.fixValid = s.gpsFix = s.gpsPresent = true;
            s.receivedUs = uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(
                                        std::chrono::steady_clock::now().time_since_epoch()).count());
            nav.offerGps(s);
            sleepMs(100);
        }
    });
    sleepMs(3000);
    const FakeCounters fc = r.fake.counters();
    int bgs = 0, chromes = 0;
    uint64_t origin = 0;
    bool decoded = true;
    for (const FakeFrame& f : r.fake.frames()) {
        if (f.panKind == es3map::PanImageBackground) {
            ++bgs;
            origin = f.ts;
            decoded = decoded && f.decodeOk && f.decW == 512 && f.decH == 384;
        }
        if (f.panKind == es3map::PanImageChrome) {
            ++chromes;
            decoded = decoded && f.decodeOk && f.decW == 320 && f.decH == 32;
        }
    }
    check(bgs >= 1 && chromes >= 1 && decoded,
          std::to_string(bgs) + " background(s), " + std::to_string(chromes) + " chrome image(s) decoded on the board");
    check(fc.poses >= 45 && fc.poses <= 64 && fc.poseSeqErrors == 0 && fc.poseBadFields == 0,
          std::to_string(fc.poses) + " MAP_POSE records in ~3 s at 20 Hz, seq increasing");
    const es3map::PanView pv = es3map::computePanView(origin, 512, 384, 320, 192, fc.lastPose);
    char why[200];
    std::snprintf(why, sizeof(why), "the newest pose: an east arrow inside the map area of the background the board "
                  "has (marker %u, course %u, origin %u,%u, pose %.1f,%.1f, marker at %d,%d)",
                  unsigned(fc.lastPose.marker), unsigned(fc.lastPose.courseCdeg), es3map::originX(origin),
                  es3map::originY(origin), fc.lastPose.xQ4 / 16.0, fc.lastPose.yQ4 / 16.0, pv.markerXQ4 / 16,
                  pv.markerYQ4 / 16);
    check(fc.lastPose.marker == es3map::MarkerArrow && fc.lastPose.courseCdeg == 9000 && pv.markerVisible, why);
    run = false;
    feed.join();
    const auto ns = nav.stats();
    nav.stop();
    check(ns.panning && ns.panBackgrounds >= 1 && ns.posesRefused == 0, "navigation: panning, no pose refused");
    check(r.fake.counters().busyRefusals == 0 && r.fake.counters().startMap == 0, "no START_MAP, no busy refusal");
}

static void testPanReconnect() {
    std::printf("\n[24] panning across a lost reply, a re-HELLO and a USB replug on the MAP tab\n");
    {   // PAN_CAPABILITIES reply lost once: re-HELLO, asked again, panning works.
        Rig r;
        r.fake.mapMode      = 1;
        r.fake.panMode      = 1;
        r.fake.dropReplyTo  = es3map::CmdPanCapabilities;
        DisplayLinkConfig c = r.cfg();
        c.map               = true;
        check(r.up(c), "negotiated");
        check(waitFor([&] { return r.link.stats().panSupported; }, 4000),
              "PAN_CAPABILITIES asked again after the re-HELLO (" + std::to_string(r.fake.counters().panCaps) + " asks)");
        r.fake.setPage(5);
        check(waitFor([&] { return r.link.panSink().panDemand().active; }, 1500), "and the panning session opens");
    }
    {   // Replugged while the board shows MAP: everything is negotiated again and the producer resends.
        Rig r;
        r.fake.mapMode      = 1;
        r.fake.panMode      = 1;
        DisplayLinkConfig c = r.cfg();
        c.map               = true;
        check(r.up(c), "negotiated");
        r.fake.setPage(5);
        check(waitFor([&] { return r.link.panSink().panDemand().active; }, 1500), "panning session");
        const uint32_t first = r.link.panSink().panDemand().session;
        r.fake.unplug();
        check(waitFor([&] { return !r.link.panSink().panDemand().active; }, 1500), "unplugged: demand inactive");
        r.fake.initialPage = 5;
        check(r.fake.plug(r.dev), "replugged, showing MAP");
        check(waitFor([&] { return r.link.panSink().panDemand().active; }, 4000),
              "the link learns the page and opens a new panning session");
        const auto pd = r.link.panSink().panDemand();
        check(pd.session != first && r.fake.counters().panCaps == 1 && r.fake.counters().startPan == 1 &&
                  r.fake.counters().startMap == 0,
              "a new session (fresh 0D and 0E on the new connection)");
    }
    {   // A board whose largest background leaves no room to scroll: whole frames instead.
        Rig r;
        r.fake.mapMode      = 1;
        r.fake.panMode      = 1;
        r.fake.panMaxW      = 352;
        r.fake.panMaxH      = 224;
        DisplayLinkConfig c = r.cfg();
        c.map               = true;
        check(r.up(c), "negotiated");
        r.fake.setPage(5);
        check(waitFor([&] { return r.link.mapSink().demand().active; }, 2000),
              "352x224 maximum (16 px margin): START_MAP and whole frames");
        check(!r.link.stats().panSupported && r.fake.counters().startPan == 0, "no panning session");
    }
}

static std::atomic<bool> g_fakeRun{true};
static void onFakeSignal(int) { g_fakeRun = false; }

/// es3display_test --fake-board PATH: the firmware-logic fake on a pty behind
/// the symlink PATH until SIGINT/SIGTERM, for an end-to-end run of the real app
/// (point <DisplayPreview><Device> at PATH).  Prints the device's counters every 5 s.
static int runFakeBoard(const std::string& path) {
    std::signal(SIGINT, onFakeSignal);
    std::signal(SIGTERM, onFakeSignal);
    FirmwareFake fw;
    if (!fw.plug(path)) {
        std::fprintf(stderr, "cannot create the pty / symlink %s\n", path.c_str());
        return 1;
    }
    std::printf("fake ES3C28P (firmware logic) at %s\n", path.c_str());
    std::fflush(stdout);
    int64_t next = nowMs() + 5000;
    while (g_fakeRun) {
        sleepMs(50);
        if (nowMs() < next) continue;
        next += 5000;
        const FirmwareFake::Counters c = fw.counters();
        std::printf("hello %u ping %u startVideo %u | assembled %u decoded %u presented %u dropped %u | "
                    "decodeFail %u sessionTimeouts %u | last jpeg %u B, blit %u us\n",
                    c.hello, c.ping, c.startVideo, c.video.assembled, c.video.decoded, c.video.presented,
                    c.video.dropped, c.decodeFailures, c.sessionTimeouts, c.video.jpegBytes, c.video.blitUs);
        std::fflush(stdout);
    }
    fw.unplug();
    ::unlink(path.c_str());
    return 0;
}

/// es3display_test --fake-map-board PATH [pan]: the contract fake WITH the map
/// extension (and with "pan" board-side panning, extension v2), cycling the tabs
/// LIVE, MAP, TRIP, MAP every 4 s, for an end-to-end run of the app with
/// preview + navigation.
static int runFakeMapBoard(const std::string& path, bool pan) {
    std::signal(SIGINT, onFakeSignal);
    std::signal(SIGTERM, onFakeSignal);
    FakeDisplay fd;
    fd.mapMode = 1;
    fd.panMode = pan ? 1 : 0;
    if (!fd.plug(path)) {
        std::fprintf(stderr, "cannot create the pty / symlink %s\n", path.c_str());
        return 1;
    }
    std::printf("fake ES3C28P with the map extension%s at %s (tabs cycle every 4 s)\n",
                pan ? " and board-side panning" : "", path.c_str());
    std::fflush(stdout);
    static const int kCycle[] = {0, 5, 1, 5};
    int     step = 0;
    int64_t next = nowMs() + 4000;
    while (g_fakeRun) {
        sleepMs(50);
        if (nowMs() < next) continue;
        next += 4000;
        int cam = 0, map = 0, bad = 0, bgs = 0, chromes = 0;
        for (const FakeFrame& f : fd.frames()) {
            (f.map ? map : cam)++;
            bgs += f.panKind == es3map::PanImageBackground;
            chromes += f.panKind == es3map::PanImageChrome;
            if (!f.decodeOk || (f.map && !f.panKind && (f.decW != 320 || f.decH != 224))) ++bad;
        }
        const FakeCounters c = fd.counters();
        std::printf("page %d | camera frames %d, map frames %d (backgrounds %d, chrome %d), bad %d | startVideo %d "
                    "stopVideo %d startMap %d startPan %d stopMap %d busy %d | poses %d | status polls %d\n",
                    fd.page(), cam, map, bgs, chromes, bad, c.startVideo, c.stopVideo, c.startMap, c.startPan,
                    c.stopMap, c.busyRefusals, c.poses, c.status);
        std::fflush(stdout);
        step = (step + 1) % 4;
        fd.setPage(kCycle[step]);
    }
    fd.unplug();
    ::unlink(path.c_str());
    return 0;
}

int main(int argc, char** argv) {
    if (argc == 2 && std::strcmp(argv[1], "--streaming-regressions") == 0) {
        testStreamingRegressions();
        return g_fails ? 1 : 0;
    }
    if (argc == 3 && std::string(argv[1]) == "--fake-board") return runFakeBoard(argv[2]);
    if ((argc == 3 || argc == 4) && std::string(argv[1]) == "--fake-map-board")
        return runFakeMapBoard(argv[2], argc == 4 && std::string(argv[3]) == "pan");
    std::printf("es3display_test: Es3DisplayLink vs a fake ES3C28P on a pty\n");
    const uint8_t chk[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    check(refCrc32(chk, 9) == 0xCBF43926u && es3video::crc32(chk, 9) == 0xCBF43926u,
          "CRC-32 check value (test reference and protocol header)");
    const int64_t t0 = nowMs();
    testConfigAndLifecycle();
    testNegotiation();
    testControlOnly();
    testFramesFlow();
    testStreamingRegressions();
    testReplacement();
    testCredits();
    testCreditStall();
    testStaleRecords();
    testGarbage();
    testBufferedReplyTimeoutRecovery();
    testVisibility();
    testControlPriority();
    testSlowReader();
    testHeartbeat();
    testTimeouts();
    testUnplug();
    testStopBounded();
    testEndToEnd();
    testRequests();
    testFirmwareCompatibility();
    testMapSurface();
    testPanSurface();
    testPanEndToEnd();
    testPanReconnect();
    std::printf("\n%s (%d failure%s, %.1f s)\n", g_fails ? "FAILED" : "ALL PASSED", g_fails, g_fails == 1 ? "" : "s",
                double(nowMs() - t0) / 1000.0);
    return g_fails ? 1 : 0;
}
