// videopreview_test — validates libvideopreview with no camera, no display and
// no files: every JPEG is synthesized in memory with libjpeg, and the display
// is a mock IPreviewSink.
//
// Covers: letterbox geometry (16:9, 4:3, portrait, odd sizes, extreme aspect,
// zero) plus an invariant sweep; the DCT scale choice; FrameConverter output —
// decodes back as a baseline 4:2:0 JPEG at the viewport size, solid colours
// preserved, left/right and top/bottom splits stay put — for 1080p 4:2:0 and
// 4:2:2, 720p, 640x480, 1080x1920 portrait, 4K, grayscale and a 16x16 upscale;
// the byte cap (one retry at the floor quality, then drop); corrupt, truncated,
// CMYK, progressive, oversized-header and non-JPEG input; the lease pool;
// VideoPreview's rate limit, inactive demand, waiting-frame replacement, sink
// refusal and held leases (no pool leak), invalidateSource() (waiting and
// in-progress frames), stop() with work in flight, submit after stop, restart,
// leases outliving the preview, once-per-kind logging, and a concurrent
// submitter stress whose counters must add up.
//
// Optional benchmark: PREVIEW_BENCH_DIR=<directory of *.jpg> converts every
// file to the 320x240 canvas (q65, 4:2:0) and prints p50/p95 decode, resize,
// encode and total ms and the output sizes.  The frames are only read.
//
// Usage: videopreview_test

#include "libvideopreview.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csetjmp>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <jpeglib.h>

using namespace dashcam::preview;
using dashcam::log::LogLevel;
using Clock = std::chrono::steady_clock;

static int g_fails = 0;
static void check(bool ok, const std::string& what) {
    std::printf("  %s  %s\n", ok ? "ok  " : "FAIL", what.c_str());
    if (!ok) ++g_fails;
}

template <typename Pred>
static bool waitFor(Pred pred, int timeoutMs = 5000) {
    const auto end = Clock::now() + std::chrono::milliseconds(timeoutMs);
    while (!pred()) {
        if (Clock::now() > end) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return true;
}

// ─── JPEG synthesis (test fixture; a libjpeg error here exits the test) ───────

enum class Fmt { S420, S422, S444, Gray, Cmyk };
using Pattern = std::function<void(uint32_t x, uint32_t y, uint8_t rgb[3])>;

static std::vector<uint8_t> makeJpeg(uint32_t w, uint32_t h, Fmt fmt, const Pattern& pat,
                                     int quality = 90, bool progressive = false) {
    jpeg_compress_struct ci;
    jpeg_error_mgr       jerr;
    ci.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&ci);
    unsigned char* mem = nullptr;
    unsigned long  memSize = 0;
    jpeg_mem_dest(&ci, &mem, &memSize);
    const int comps = fmt == Fmt::Gray ? 1 : fmt == Fmt::Cmyk ? 4 : 3;
    ci.image_width      = w;
    ci.image_height     = h;
    ci.input_components = comps;
    ci.in_color_space   = fmt == Fmt::Gray ? JCS_GRAYSCALE : fmt == Fmt::Cmyk ? JCS_CMYK : JCS_RGB;
    jpeg_set_defaults(&ci);
    if (fmt == Fmt::Cmyk) jpeg_set_colorspace(&ci, JCS_CMYK);
    jpeg_set_quality(&ci, quality, TRUE);
    if (fmt == Fmt::S422) { ci.comp_info[0].h_samp_factor = 2; ci.comp_info[0].v_samp_factor = 1; }
    if (fmt == Fmt::S444) { ci.comp_info[0].h_samp_factor = 1; ci.comp_info[0].v_samp_factor = 1; }
    if (progressive) jpeg_simple_progression(&ci);
    jpeg_start_compress(&ci, TRUE);
    std::vector<uint8_t> row(size_t(w) * comps);
    while (ci.next_scanline < h) {
        const uint32_t y = ci.next_scanline;
        for (uint32_t x = 0; x < w; ++x) {
            uint8_t rgb[3];
            pat(x, y, rgb);
            uint8_t* p = &row[size_t(x) * comps];
            if (fmt == Fmt::Gray) {
                p[0] = uint8_t((rgb[0] * 77 + rgb[1] * 150 + rgb[2] * 29) >> 8);
            } else if (fmt == Fmt::Cmyk) {
                p[0] = uint8_t(255 - rgb[0]); p[1] = uint8_t(255 - rgb[1]);
                p[2] = uint8_t(255 - rgb[2]); p[3] = 0;
            } else {
                p[0] = rgb[0]; p[1] = rgb[1]; p[2] = rgb[2];
            }
        }
        JSAMPROW rp = row.data();
        jpeg_write_scanlines(&ci, &rp, 1);
    }
    jpeg_finish_compress(&ci);
    std::vector<uint8_t> out(mem, mem + memSize);
    std::free(mem);
    jpeg_destroy_compress(&ci);
    return out;
}

static Pattern solid(uint8_t r, uint8_t g, uint8_t b) {
    return [=](uint32_t, uint32_t, uint8_t* p) { p[0] = r; p[1] = g; p[2] = b; };
}
// Left half red, right half blue.
static Pattern leftRight(uint32_t w) {
    return [=](uint32_t x, uint32_t, uint8_t* p) {
        if (x < w / 2) { p[0] = 230; p[1] = 30; p[2] = 30; } else { p[0] = 30; p[1] = 30; p[2] = 230; }
    };
}
// Top half green, bottom half white.
static Pattern topBottom(uint32_t h) {
    return [=](uint32_t, uint32_t y, uint8_t* p) {
        if (y < h / 2) { p[0] = 30; p[1] = 200; p[2] = 30; } else { p[0] = p[1] = p[2] = 240; }
    };
}
// Deterministic noise of ±amp around mid grey (hard to compress).
static Pattern noise(int amp) {
    return [=](uint32_t x, uint32_t y, uint8_t* p) {
        uint32_t s = x * 73856093u ^ y * 19349663u;
        for (int c = 0; c < 3; ++c) {
            s = s * 1103515245u + 12345u;
            const int v = 128 + int((s >> 16) % uint32_t(2 * amp + 1)) - amp;
            p[c] = uint8_t(std::clamp(v, 0, 255));
        }
    };
}

// ─── JPEG decoding for verification (setjmp-guarded, PODs across longjmp) ─────

struct TestErr {
    jpeg_error_mgr pub;
    std::jmp_buf   jump;
};
static void testErrorExit(j_common_ptr c) { std::longjmp(reinterpret_cast<TestErr*>(c->err)->jump, 1); }
static void testSilence(j_common_ptr, int) {}

struct TestDecode {
    jpeg_decompress_struct ci;
    TestErr                err;
    const uint8_t*         data;
    size_t                 size;
    uint8_t*               out;       // null = header only
    size_t                 outCap;
    uint32_t               w, h, comps, numComponents, h0, v0;
    bool                   progressive;
};

static bool testDecodeBody(TestDecode* c) {
    jpeg_mem_src(&c->ci, c->data, static_cast<unsigned long>(c->size));
    jpeg_read_header(&c->ci, TRUE);
    jpeg_calc_output_dimensions(&c->ci);
    c->w             = c->ci.output_width;
    c->h             = c->ci.output_height;
    c->comps         = uint32_t(c->ci.output_components);
    c->numComponents = uint32_t(c->ci.num_components);
    c->h0            = uint32_t(c->ci.comp_info[0].h_samp_factor);
    c->v0            = uint32_t(c->ci.comp_info[0].v_samp_factor);
    c->progressive   = c->ci.progressive_mode != 0;
    if (c->out == nullptr) return true;
    jpeg_start_decompress(&c->ci);
    const size_t stride = size_t(c->w) * c->comps;
    if (stride * c->h > c->outCap) return false;
    while (c->ci.output_scanline < c->ci.output_height) {
        JSAMPROW rp = c->out + size_t(c->ci.output_scanline) * stride;
        jpeg_read_scanlines(&c->ci, &rp, 1);
    }
    jpeg_finish_decompress(&c->ci);
    return true;
}

static bool testDecodeGuarded(TestDecode* c) {
    c->ci.err = jpeg_std_error(&c->err.pub);
    c->err.pub.error_exit   = testErrorExit;
    c->err.pub.emit_message = testSilence;
    if (setjmp(c->err.jump)) {
        jpeg_destroy_decompress(&c->ci);
        return false;
    }
    jpeg_create_decompress(&c->ci);
    const bool ok = testDecodeBody(c);
    jpeg_destroy_decompress(&c->ci);
    return ok;
}

struct Img {
    bool                 ok = false;
    uint32_t             w = 0, h = 0, comps = 0, numComponents = 0, h0 = 0, v0 = 0;
    bool                 progressive = false;
    std::vector<uint8_t> px;
    double mean(uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1, int ch) const {
        double s = 0;
        for (uint32_t y = y0; y < y1; ++y)
            for (uint32_t x = x0; x < x1; ++x) s += px[(size_t(y) * w + x) * comps + ch];
        return s / double((x1 - x0) * (y1 - y0));
    }
    int maxDiff(const uint8_t want[3]) const {
        int m = 0;
        for (size_t i = 0; i < px.size(); i += comps)
            for (uint32_t c = 0; c < comps; ++c) m = std::max(m, std::abs(int(px[i + c]) - int(want[c])));
        return m;
    }
};

static Img decodeJpeg(const uint8_t* data, size_t size) {
    Img img;
    TestDecode c{};
    c.data = data;
    c.size = size;
    if (!testDecodeGuarded(&c)) return img;
    img.w = c.w; img.h = c.h; img.comps = c.comps; img.numComponents = c.numComponents;
    img.h0 = c.h0; img.v0 = c.v0; img.progressive = c.progressive;
    img.px.assign(size_t(c.w) * c.h * c.comps, 0);
    TestDecode d{};
    d.data = data; d.size = size; d.out = img.px.data(); d.outCap = img.px.size();
    img.ok = testDecodeGuarded(&d);
    return img;
}

// Offset of the first occurrence of marker FF <code>, or 0.
static size_t findMarker(const std::vector<uint8_t>& j, uint8_t code) {
    for (size_t i = 2; i + 1 < j.size(); ++i)
        if (j[i] == 0xFF && j[i + 1] == code) return i;
    return 0;
}

// ─── mock display ─────────────────────────────────────────────────────────────

class MockSink : public IPreviewSink {
public:
    std::atomic<bool>     active{true};
    std::atomic<uint16_t> canvasW{320}, canvasH{240};
    std::atomic<size_t>   maxBytes{0};
    std::atomic<bool>     refuse{false};
    std::atomic<bool>     hold{false};         ///< keep taken leases in `held`
    std::atomic<bool>     refuseEveryThird{false};
    std::atomic<uint64_t> offers{0}, takes{0};

    struct Seen {
        size_t size; uint16_t w, h; uint64_t captureUs, seq; int quality;
        std::vector<uint8_t> bytes;
    };

    PreviewDemand demand() const override {
        if (blockWorkerDemand_.load() && std::this_thread::get_id() != mainThread_) {
            std::unique_lock<std::mutex> lk(mu_);
            inDemand_ = true;
            cv_.notify_all();
            cv_.wait(lk, [&] { return !blockWorkerDemand_.load(); });
        }
        PreviewDemand d;
        d.active       = active.load();
        d.canvasW      = canvasW.load();
        d.canvasH      = canvasH.load();
        d.maxJpegBytes = maxBytes.load();
        return d;
    }

    bool offerFrame(EncodedFrameLease&& f) override {
        const uint64_t n = ++offers;
        std::unique_lock<std::mutex> lk(mu_);
        ++inOffer_;
        cv_.notify_all();
        cv_.wait(lk, [&] { return !gateClosed_; });
        --inOffer_;
        if (refuse.load() || (refuseEveryThird.load() && n % 3 == 0)) return false;
        Seen s{f.size(), f.width(), f.height(), f.captureMonotonicUs(), f.sourceSequence(),
               f.quality(), {}};
        if (keepBytes) s.bytes.assign(f.data(), f.data() + f.size());
        seen_.push_back(std::move(s));
        if (hold.load()) {
            held_.push_back(std::move(f));
        } else if (holdOne) {           // stress: keep one, release it on the next offer
            held_.clear();
            held_.push_back(std::move(f));
        } else {
            f.release();
        }
        ++takes;
        return true;
    }

    bool keepBytes = true;
    bool holdOne   = false;

    void closeGate() { std::lock_guard<std::mutex> lk(mu_); gateClosed_ = true; }
    void openGate() { { std::lock_guard<std::mutex> lk(mu_); gateClosed_ = false; } cv_.notify_all(); }
    bool waitInOffer(int ms = 5000) {
        std::unique_lock<std::mutex> lk(mu_);
        return cv_.wait_for(lk, std::chrono::milliseconds(ms), [&] { return inOffer_ > 0; });
    }
    void blockWorkerDemand() {
        std::lock_guard<std::mutex> lk(mu_);
        mainThread_ = std::this_thread::get_id();
        inDemand_   = false;
        blockWorkerDemand_ = true;
    }
    bool waitInDemand(int ms = 5000) {
        std::unique_lock<std::mutex> lk(mu_);
        return cv_.wait_for(lk, std::chrono::milliseconds(ms), [&] { return inDemand_; });
    }
    void unblockWorkerDemand() { { std::lock_guard<std::mutex> lk(mu_); blockWorkerDemand_ = false; } cv_.notify_all(); }
    void releaseHeld() { std::lock_guard<std::mutex> lk(mu_); held_.clear(); }
    std::vector<EncodedFrameLease> takeHeld() { std::lock_guard<std::mutex> lk(mu_); return std::move(held_); }
    std::vector<Seen> seen() { std::lock_guard<std::mutex> lk(mu_); return seen_; }
    size_t seenCount() { std::lock_guard<std::mutex> lk(mu_); return seen_.size(); }

private:
    mutable std::mutex              mu_;
    mutable std::condition_variable cv_;
    bool                            gateClosed_ = false;
    int                             inOffer_    = 0;
    mutable bool                    inDemand_   = false;
    std::atomic<bool>               blockWorkerDemand_{false};
    std::thread::id                 mainThread_;
    std::vector<Seen>               seen_;
    std::vector<EncodedFrameLease>  held_;
};

struct LogCapture {
    std::mutex               mu;
    std::vector<std::string> lines;
    dashcam::log::LogCallback cb() {
        return [this](LogLevel, const std::string& m) { std::lock_guard<std::mutex> lk(mu); lines.push_back(m); };
    }
    size_t count(const std::string& needle) {
        std::lock_guard<std::mutex> lk(mu);
        return size_t(std::count_if(lines.begin(), lines.end(),
                                    [&](const std::string& l) { return l.find(needle) != std::string::npos; }));
    }
};

static uint64_t sr(const PreviewStats& s, SubmitResult r) { return s.submit[size_t(r)]; }
static uint64_t outcomes(const PreviewStats& s) {
    return s.inactiveDropped + s.noOutputSlot + s.decodeFailed + s.unsupportedSource +
           s.encodeOversizedDropped + s.encodeFailed + s.invalidatedDropped + s.offered;
}
// Every accepted frame has reached exactly one end, and the sink has answered.
static bool settled(const VideoPreview& p, const MockSink& sink) {
    const PreviewStats s = p.stats();
    return s.processed + s.discardedWaiting == sr(s, SubmitResult::Accepted) &&
           outcomes(s) == s.processed && s.offered == s.refused + sink.takes.load();
}
// Submit, retrying while Busy (the worker held the slot lock for an instant).
static SubmitResult submitRaw(VideoPreview& p, const uint8_t* bytes, size_t size, uint32_t w, uint32_t h,
                              uint64_t t) {
    for (int i = 0; i < 100000; ++i) {
        const SubmitResult r = p.submitJpeg(bytes, size, w, h, t);
        if (r != SubmitResult::Busy) return r;
        std::this_thread::yield();
    }
    return SubmitResult::Busy;
}
static SubmitResult submit(VideoPreview& p, const std::vector<uint8_t>& j, uint32_t w, uint32_t h, uint64_t t) {
    return submitRaw(p, j.data(), j.size(), w, h, t);
}

static PreviewConfig defaultCfg() { return PreviewConfig{}; }

// ─── tests ────────────────────────────────────────────────────────────────────

static void testLetterbox() {
    std::printf("--- letterbox geometry ---\n");
    struct Case { uint32_t sw, sh; uint16_t cw, ch; Viewport want; const char* what; };
    const Case cases[] = {
        {1920, 1080, 320, 240, {0, 30, 320, 180}, "16:9 1080p -> 320x180, 30 px bars"},
        {1280, 720, 320, 240, {0, 30, 320, 180}, "16:9 720p -> 320x180"},
        {3840, 2160, 320, 240, {0, 30, 320, 180}, "16:9 4K -> 320x180"},
        {640, 480, 320, 240, {0, 0, 320, 240}, "4:3 fills the canvas"},
        {1080, 1920, 320, 240, {93, 0, 134, 240}, "9:16 portrait pillarbox (135 -> 134, tie down)"},
        {1366, 768, 320, 240, {0, 30, 320, 180}, "1366x768 (179.9 -> 180)"},
        {720, 480, 320, 240, {0, 13, 320, 214}, "3:2 (213.3 -> 214)"},
        {1921, 1081, 320, 240, {0, 30, 320, 180}, "odd source 1921x1081"},
        {1920, 1080, 321, 241, {0, 30, 320, 180}, "odd canvas 321x241: even 320x180, centred"},
        {16, 16, 320, 240, {40, 0, 240, 240}, "square 16x16 -> 240x240 pillarbox"},
        {10000, 1, 320, 240, {0, 119, 320, 2}, "extreme wide: at least 2 rows"},
        {1, 10000, 320, 240, {159, 0, 2, 240}, "extreme tall: at least 2 columns"},
        {2, 2, 2, 2, {0, 0, 2, 2}, "2x2 into 2x2"},
        {0, 1080, 320, 240, {}, "zero source width rejected"},
        {1920, 0, 320, 240, {}, "zero source height rejected"},
        {1920, 1080, 0, 240, {}, "zero canvas rejected"},
        {1920, 1080, 1, 1, {}, "1x1 canvas rejected"},
    };
    for (const Case& c : cases) {
        const Viewport v = computeLetterbox(c.sw, c.sh, c.cw, c.ch);
        char buf[160];
        std::snprintf(buf, sizeof(buf), "%s: {%u,%u,%u,%u}", c.what, v.x, v.y, v.width, v.height);
        check(v == c.want, buf);
    }

    // Invariants over a sweep: even, inside the canvas, centred, one axis full,
    // aspect within the rounding of one pixel pair.
    bool inv = true;
    int  n   = 0;
    for (uint32_t sw = 1; sw <= 4000; sw += 37)
        for (uint32_t sh = 1; sh <= 4000; sh += 41) {
            for (const auto& cv : {std::pair<uint16_t, uint16_t>{320, 240}, {321, 199}, {64, 480}}) {
                const Viewport v = computeLetterbox(sw, sh, cv.first, cv.second);
                const uint32_t aw = cv.first & ~1u, ah = cv.second & ~1u;
                ++n;
                bool ok = v.width % 2 == 0 && v.height % 2 == 0 && v.width >= 2 && v.height >= 2 &&
                          v.x + v.width <= cv.first && v.y + v.height <= cv.second &&
                          (cv.first - v.width) / 2 == v.x && (cv.second - v.height) / 2 == v.y &&
                          (v.width == aw || v.height == ah);
                // |w/h - sw/sh| as cross products: the rounded axis is off by ≤ 1 px.
                const int64_t err = int64_t(v.width) * sh - int64_t(v.height) * sw;
                if (v.width == aw && v.height > 2 && v.height < ah) ok = ok && std::llabs(err) <= int64_t(sw);
                if (v.height == ah && v.width > 2 && v.width < aw) ok = ok && std::llabs(err) <= int64_t(sh);
                if (!ok && inv) {
                    std::printf("    first violation: %ux%u in %ux%u -> {%u,%u,%u,%u}\n", sw, sh, cv.first,
                                cv.second, v.x, v.y, v.width, v.height);
                }
                inv = inv && ok;
            }
        }
    check(inv, "sweep of " + std::to_string(n) + " sizes: even, inside, centred, aspect kept");
}

static void testDctScale() {
    std::printf("--- DCT scale choice ---\n");
    struct Case { uint32_t sw, sh, vw, vh, want; const char* what; };
    const Case cases[] = {
        {1920, 1080, 320, 180, 4, "1080p -> 320x180: 1/4 (480x270)"},
        {1280, 720, 320, 180, 4, "720p -> 320x180: 1/4 (exactly 320x180)"},
        {3840, 2160, 320, 180, 8, "4K -> 320x180: 1/8 (480x270)"},
        {7680, 4320, 320, 180, 8, "8K -> 320x180: 1/8 is the largest offered"},
        {640, 480, 320, 240, 2, "640x480 -> 320x240: 1/2"},
        {1080, 1920, 134, 240, 8, "1080x1920 -> 134x240: 1/8 (135x240)"},
        {1280, 720, 321, 180, 2, "one pixel short of 1/4 falls back to 1/2"},
        {16, 16, 240, 240, 1, "16x16 upscale: full size"},
        {320, 180, 320, 180, 1, "same size: full size"},
        {1919, 1079, 240, 135, 8, "odd source: ceil() sizes (240x135 at 1/8)"},
    };
    for (const Case& c : cases) check(chooseDctScaleDenom(c.sw, c.sh, c.vw, c.vh) == c.want, c.what);
}

static void testPool() {
    std::printf("--- lease pool ---\n");
    check(EncodedFramePool::create(0, 1024) == nullptr, "zero slots rejected");
    check(EncodedFramePool::create(2, 0) == nullptr, "zero-byte slots rejected");
    auto pool = EncodedFramePool::create(2, 1024);
    check(pool && pool->freeSlots() == 2 && pool->slotBytes() == 1024, "2 x 1 KiB pool");
    EncodedFrameLease a = pool->tryAcquire();
    EncodedFrameLease b = pool->tryAcquire();
    EncodedFrameLease c = pool->tryAcquire();
    check(a && b && !c && pool->freeSlots() == 0, "two leases, the third is empty");
    check(a.writableData() != b.writableData() && a.capacity() == 1024, "distinct slots of full capacity");
    check(!a.setFrame(1025, 1, 1, 0, 0, 0) && a.setFrame(10, 4, 2, 77, 5, 65) && a.size() == 10 &&
              a.width() == 4 && a.captureMonotonicUs() == 77 && a.sourceSequence() == 5 && a.quality() == 65,
          "setFrame bounds-checks and records metadata");
    EncodedFrameLease moved(std::move(a));
    check(!a && moved && moved.size() == 10 && pool->freeSlots() == 0, "move transfers, source empty");
    a = std::move(moved);
    check(a && !moved, "move-assign transfers back");
    a.release();
    a.release();
    check(!a && pool->freeSlots() == 1, "release once (second call is a no-op)");
    b = pool->tryAcquire();   // move-assign over a live lease releases the old slot first
    check(b && pool->freeSlots() == 1, "move-assign over a live lease releases it");
    std::weak_ptr<EncodedFramePool> weak = pool;
    pool.reset();
    check(!weak.expired() && b.data() != nullptr, "a live lease keeps the pool alive");
    b.release();
    check(weak.expired(), "last lease released frees the pool");
}

struct ConvCase {
    const char* what;
    uint32_t    w, h;
    Fmt         fmt;
    Viewport    want;
    uint32_t    denom;
};

static void testConverter() {
    std::printf("--- converter: geometry, format, colour ---\n");
    FrameConverter conv;
    check(conv.init() && conv.ready(), "init() allocates");
    std::vector<uint8_t> out(65536);
    auto run = [&](const std::vector<uint8_t>& j, uint16_t cw = 320, uint16_t ch = 240) {
        ConvertRequest rq;
        rq.jpeg = j.data(); rq.size = j.size(); rq.canvasW = cw; rq.canvasH = ch;
        rq.quality = 65; rq.minimumQuality = 45; rq.out = out.data(); rq.outCapacity = out.size();
        return conv.convert(rq);
    };

    const ConvCase cases[] = {
        {"1920x1080 4:2:0", 1920, 1080, Fmt::S420, {0, 30, 320, 180}, 4},
        {"1920x1080 4:2:2", 1920, 1080, Fmt::S422, {0, 30, 320, 180}, 4},
        {"1280x720 4:4:4", 1280, 720, Fmt::S444, {0, 30, 320, 180}, 4},
        {"640x480 4:3", 640, 480, Fmt::S420, {0, 0, 320, 240}, 2},
        {"1080x1920 portrait", 1080, 1920, Fmt::S420, {93, 0, 134, 240}, 8},
        {"3840x2160 4K", 3840, 2160, Fmt::S420, {0, 30, 320, 180}, 8},
        {"16x16 tiny (upscaled)", 16, 16, Fmt::S420, {40, 0, 240, 240}, 1},
    };
    const uint8_t col[3] = {200, 40, 90};
    for (const ConvCase& c : cases) {
        const std::string tag = std::string(c.what) + ": ";
        // Solid colour survives decode → resample → q65 encode → decode.
        const auto src  = makeJpeg(c.w, c.h, c.fmt, solid(col[0], col[1], col[2]));
        const ConvertResult r = run(src);
        check(r.status == ConvertStatus::Ok && r.viewport == c.want && r.width == c.want.width &&
                  r.height == c.want.height && r.scaleDenom == c.denom && r.sourceWidth == c.w &&
                  r.sourceHeight == c.h && r.quality == 65 && !r.retried,
              tag + "viewport " + std::to_string(r.width) + "x" + std::to_string(r.height) +
                  ", DCT 1/" + std::to_string(r.scaleDenom));
        const Img img = decodeJpeg(out.data(), r.jpegBytes);
        check(img.ok && img.w == c.want.width && img.h == c.want.height && img.comps == 3 &&
                  img.numComponents == 3 && img.h0 == 2 && img.v0 == 2 && !img.progressive,
              tag + "output decodes as baseline 4:2:0 colour at the viewport size");
        const int md = img.ok ? img.maxDiff(col) : 999;
        check(img.ok && md <= 10, tag + "solid colour kept (max channel error " + std::to_string(md) + ")");

        // A left/right split stays left/right (and top/bottom for portrait).
        const bool portrait = c.h > c.w;
        const auto split = makeJpeg(c.w, c.h, c.fmt, portrait ? topBottom(c.h) : leftRight(c.w));
        const ConvertResult rs = run(split);
        const Img s = rs.status == ConvertStatus::Ok ? decodeJpeg(out.data(), rs.jpegBytes) : Img{};
        bool placed = false;
        if (s.ok && portrait) {
            const uint32_t q = s.h / 4;
            placed = s.mean(0, 0, s.w, q, 1) > 170 && s.mean(0, 0, s.w, q, 0) < 70 &&
                     s.mean(0, s.h - q, s.w, s.h, 0) > 210 && s.mean(0, s.h - q, s.w, s.h, 2) > 210;
        } else if (s.ok) {
            const uint32_t q = s.w / 4;
            placed = s.mean(0, 0, q, s.h, 0) > 190 && s.mean(0, 0, q, s.h, 2) < 70 &&
                     s.mean(s.w - q, 0, s.w, s.h, 2) > 190 && s.mean(s.w - q, 0, s.w, s.h, 0) < 70;
        }
        check(placed, tag + (portrait ? "top/bottom split stays put" : "left/right split stays put"));
    }

    // Grayscale source: libjpeg expands it; the preview is still a colour JPEG.
    {
        const auto g = makeJpeg(1920, 1080, Fmt::Gray, solid(128, 128, 128));
        const ConvertResult r = run(g);
        const Img img = r.status == ConvertStatus::Ok ? decodeJpeg(out.data(), r.jpegBytes) : Img{};
        const uint8_t grey[3] = {128, 128, 128};
        check(r.status == ConvertStatus::Ok && r.width == 320 && r.height == 180 && img.ok &&
                  img.comps == 3 && img.maxDiff(grey) <= 6,
              "grayscale 1080p -> 320x180 colour JPEG, grey kept");
    }
    // Other canvases.
    {
        const auto j = makeJpeg(1920, 1080, Fmt::S420, solid(10, 120, 250));
        const ConvertResult r = run(j, 640, 480);
        check(r.status == ConvertStatus::Ok && r.viewport == Viewport{0, 60, 640, 360} && r.scaleDenom == 2,
              "640x480 canvas: 640x360 at DCT 1/2");
        const ConvertResult r2 = run(j, 160, 120);
        check(r2.status == ConvertStatus::Ok && r2.viewport == Viewport{0, 15, 160, 90} && r2.scaleDenom == 8,
              "160x120 canvas: 160x90 at DCT 1/8");
    }
    // Determinism: the same input gives byte-identical output.
    {
        const auto j = makeJpeg(1280, 720, Fmt::S420, noise(40));
        const ConvertResult a = run(j);
        const std::vector<uint8_t> first(out.begin(), out.begin() + long(a.jpegBytes));
        conv.invalidateGeometry();
        const ConvertResult b = run(j);
        check(a.status == ConvertStatus::Ok && b.jpegBytes == a.jpegBytes &&
                  std::equal(first.begin(), first.end(), out.begin()),
              "deterministic: identical output twice (and after invalidateGeometry())");
    }
}

static void testConverterCapAndErrors() {
    std::printf("--- converter: byte cap, retry, bad input ---\n");
    FrameConverter conv;
    conv.init();
    std::vector<uint8_t> out(65536);
    auto run = [&](const std::vector<uint8_t>& j, size_t cap, int q = 65, int qmin = 45) {
        ConvertRequest rq;
        rq.jpeg = j.data(); rq.size = j.size(); rq.canvasW = 320; rq.canvasH = 240;
        rq.quality = q; rq.minimumQuality = qmin; rq.out = out.data(); rq.outCapacity = cap;
        return conv.convert(rq);
    };

    const auto noisy = makeJpeg(640, 480, Fmt::S420, noise(60), 92);
    const ConvertResult r65 = run(noisy, out.size(), 65, 65);
    const ConvertResult r45 = run(noisy, out.size(), 45, 45);
    std::printf("    noisy 320x240: q65 %zu B, q45 %zu B\n", r65.jpegBytes, r45.jpegBytes);
    check(r65.status == ConvertStatus::Ok && r45.status == ConvertStatus::Ok && r45.jpegBytes < r65.jpegBytes,
          "noisy frame: q45 is smaller than q65");
    const size_t mid = (r45.jpegBytes + r65.jpegBytes) / 2;
    const ConvertResult rr = run(noisy, mid);
    check(rr.status == ConvertStatus::Ok && rr.retried && rr.quality == 45 && rr.jpegBytes == r45.jpegBytes &&
              rr.jpegBytes <= mid,
          "over the cap at q65 -> one retry at q45 fits");
    const ConvertResult rd = run(noisy, r45.jpegBytes / 2);
    check(rd.status == ConvertStatus::EncodeOversized && rd.retried && rd.jpegBytes == 0,
          "still over the cap at q45 -> EncodeOversized");
    const ConvertResult rn = run(noisy, r65.jpegBytes / 2, 65, 65);
    check(rn.status == ConvertStatus::EncodeOversized && !rn.retried,
          "minimum == quality: no pointless retry");
    const ConvertResult exact = run(noisy, r65.jpegBytes);
    check(exact.status == ConvertStatus::Ok && !exact.retried && exact.jpegBytes == r65.jpegBytes,
          "a frame of exactly the cap fits");

    const auto good = makeJpeg(1920, 1080, Fmt::S420, leftRight(1920));
    {
        std::vector<uint8_t> t(good.begin(), good.begin() + long(good.size() / 2));
        const ConvertResult r = run(t, out.size());
        check(r.status == ConvertStatus::DecodeFailed && r.detail[0] != '\0',
              std::string("truncated JPEG -> DecodeFailed (") + r.detail + ")");
    }
    {
        std::vector<uint8_t> t = good;
        const size_t sos = findMarker(t, 0xDA);
        const size_t at  = sos + (t.size() - sos) / 2;
        t[at] = 0xFF; t[at + 1] = 0xD9;   // premature EOI in the middle of the scan
        const ConvertResult r = run(t, out.size());
        check(sos != 0 && r.status == ConvertStatus::DecodeFailed,
              std::string("premature marker mid-scan -> DecodeFailed (") + r.detail + ")");
    }
    {
        std::vector<uint8_t> t = good;
        const size_t sof = findMarker(t, 0xC0);
        t[sof + 5] = 0; t[sof + 6] = 0;   // height 0
        const ConvertResult r = run(t, out.size());
        check(sof != 0 && r.status == ConvertStatus::DecodeFailed,
              std::string("zero height in SOF -> DecodeFailed (") + r.detail + ")");
    }
    {
        std::vector<uint8_t> t = good;
        const size_t sof = findMarker(t, 0xC0);
        t[sof + 5] = 0x23; t[sof + 6] = 0x28;   // height 9000
        t[sof + 7] = 0x23; t[sof + 8] = 0x28;   // width 9000
        const ConvertResult r = run(t, out.size());
        check(r.status == ConvertStatus::UnsupportedSource && r.sourceWidth == 9000,
              std::string("9000x9000 header -> UnsupportedSource (") + r.detail + ")");
    }
    {
        std::vector<uint8_t> t = {0xFF, 0xD8, 0xFF};
        uint32_t s = 1;
        for (int i = 0; i < 5000; ++i) { s = s * 1103515245u + 12345u; t.push_back(uint8_t(s >> 16)); }
        const ConvertResult r = run(t, out.size());
        check(r.status == ConvertStatus::DecodeFailed, "SOI followed by garbage -> DecodeFailed");
    }
    {
        // Random damage inside the scan may or may not be detectable (JPEG has
        // no checksum): it must just never crash or overrun.
        int failed = 0, ok = 0;
        for (uint32_t seed = 1; seed <= 20; ++seed) {
            std::vector<uint8_t> t = good;
            const size_t sos = findMarker(t, 0xDA);
            uint32_t s = seed;
            for (int i = 0; i < 64; ++i) {
                s = s * 1103515245u + 12345u;
                t[sos + 20 + (s >> 8) % (t.size() - sos - 22)] = uint8_t(s >> 16);
            }
            const ConvertResult r = run(t, out.size());
            (r.status == ConvertStatus::Ok ? ok : failed)++;
        }
        check(ok + failed == 20, "20 randomly damaged scans: no crash (" + std::to_string(failed) +
                                     " rejected, " + std::to_string(ok) + " decoded as garbage)");
    }
    {
        const auto cmyk = makeJpeg(640, 480, Fmt::Cmyk, solid(10, 20, 30));
        const ConvertResult r = run(cmyk, out.size());
        check(r.status == ConvertStatus::UnsupportedSource,
              std::string("CMYK -> UnsupportedSource (") + r.detail + ")");
    }
    {
        const auto prog = makeJpeg(640, 480, Fmt::S420, solid(10, 20, 30), 90, true);
        const ConvertResult r = run(prog, out.size());
        check(r.status == ConvertStatus::UnsupportedSource,
              std::string("progressive -> UnsupportedSource (") + r.detail + ")");
    }
    {
        ConvertRequest rq;
        check(conv.convert(rq).status == ConvertStatus::InvalidArgument, "empty request -> InvalidArgument");
        rq.jpeg = good.data(); rq.size = good.size(); rq.out = out.data(); rq.outCapacity = out.size();
        rq.canvasW = 0; rq.canvasH = 240;
        check(conv.convert(rq).status == ConvertStatus::InvalidArgument, "zero canvas -> InvalidArgument");
        rq.canvasW = 1000; rq.canvasH = 1000;
        check(conv.convert(rq).status == ConvertStatus::InvalidArgument, "canvas over the limit -> InvalidArgument");
        rq.canvasW = 320; rq.canvasH = 240; rq.quality = 40; rq.minimumQuality = 45;
        check(conv.convert(rq).status == ConvertStatus::InvalidArgument, "minimum above quality -> InvalidArgument");
        FrameConverter uninit;
        rq.quality = 65;
        check(uninit.convert(rq).status == ConvertStatus::InvalidArgument, "convert() before init() -> InvalidArgument");
    }
    {
        // After all that, the converter still works.
        const ConvertResult r = run(good, out.size());
        check(r.status == ConvertStatus::Ok && r.width == 320, "converter healthy after the bad inputs");
    }
}

static void testConfigAndSubmitRejections() {
    std::printf("--- VideoPreview: config, rejections ---\n");
    MockSink   sink;
    LogCapture logs;
    {
        VideoPreview p;
        PreviewConfig bad = defaultCfg();
        bad.targetFps = 0;
        check(!p.start(bad, sink, logs.cb()), "targetFps 0 rejected");
        bad = defaultCfg(); bad.targetFps = 16;
        check(!p.start(bad, sink), "targetFps 16 rejected");
        bad = defaultCfg(); bad.minimumJpegQuality = 70;
        check(!p.start(bad, sink), "minimum quality above quality rejected");
        bad = defaultCfg(); bad.maxJpegBytes = 100;
        check(!p.start(bad, sink), "100-byte output cap rejected");
        bad = defaultCfg(); bad.maxSourceJpegBytes = 100;
        check(!p.start(bad, sink), "100-byte source slot rejected");
        check(!p.running() && p.submitJpeg(nullptr, 0, 0, 0, 0) == SubmitResult::Inactive,
              "never started: submit -> Inactive");
        p.stop();
        p.stop();
        check(true, "stop() on a never-started preview is a safe no-op (twice)");
    }

    VideoPreview  p;
    PreviewConfig cfg = defaultCfg();
    cfg.maxSourceJpegBytes = 64 * 1024;
    check(p.start(cfg, sink, logs.cb()) && p.running(), "start()");
    check(!p.start(cfg, sink), "second start() while running rejected");
    const auto j    = makeJpeg(640, 480, Fmt::S420, solid(50, 100, 150));
    const auto big  = makeJpeg(1280, 720, Fmt::S420, noise(60), 95);
    const uint8_t png[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    uint64_t t = 1000000;
    check(big.size() > cfg.maxSourceJpegBytes && submit(p, big, 1280, 720, t) == SubmitResult::Oversized,
          "frame over the source slot -> Oversized (" + std::to_string(big.size()) + " B)");
    check(submitRaw(p, png, sizeof(png), 640, 480, t) == SubmitResult::Unsupported, "PNG -> Unsupported");
    check(submitRaw(p, nullptr, 100, 640, 480, t) == SubmitResult::Unsupported, "null bytes -> Unsupported");
    check(submitRaw(p, j.data(), 0, 640, 480, t) == SubmitResult::Unsupported, "zero size -> Unsupported");
    check(submitRaw(p, j.data(), j.size(), 0, 480, t) == SubmitResult::Unsupported, "zero width -> Unsupported");
    sink.canvasW = 0;
    check(submit(p, j, 640, 480, t) == SubmitResult::Unsupported, "zero canvas -> Unsupported");
    sink.canvasW = 1000; sink.canvasH = 1000;
    check(submit(p, j, 640, 480, t) == SubmitResult::Unsupported, "1000x1000 canvas -> Unsupported");
    sink.canvasW = 320; sink.canvasH = 240;
    sink.active = false;
    check(submit(p, j, 640, 480, t) == SubmitResult::Inactive, "inactive demand -> Inactive");
    sink.active = true;
    PreviewStats s = p.stats();
    check(s.processed == 0 && sr(s, SubmitResult::RateLimited) == 0,
          "rejections never reach the worker or the rate schedule");
    check(submit(p, j, 640, 480, t) == SubmitResult::Accepted, "a good frame -> Accepted");
    check(waitFor([&] { return settled(p, sink) && sink.seenCount() == 1; }), "... converted and delivered");
    const auto seen = sink.seen();
    check(seen.size() == 1 && seen[0].w == 320 && seen[0].h == 240 && seen[0].captureUs == t &&
              seen[0].seq == 1 && seen[0].quality == 65,
          "lease metadata: 320x240, capture time, sequence 1, q65");

    // Corrupt-but-JPEG-looking input passes submit and fails in the worker,
    // logged once however often it happens.
    std::vector<uint8_t> trunc(j.begin(), j.begin() + long(j.size() / 2));
    for (int i = 0; i < 5; ++i) {
        t += 200000;
        submit(p, trunc, 640, 480, t);
        waitFor([&] { return settled(p, sink); });
    }
    s = p.stats();
    check(s.decodeFailed == 5 && s.offered == 1, "5 truncated frames -> 5 decodeFailed, nothing offered");
    // Announced size disagrees with the JPEG: the JPEG's size wins, counted.
    t += 200000;
    submit(p, j, 1920, 1080, t);
    check(waitFor([&] { return settled(p, sink) && sink.seenCount() == 2; }) &&
              p.stats().geometryMismatch == 1 && sink.seen().back().h == 240,
          "announced 1920x1080 for a 640x480 JPEG: converted as 640x480, mismatch counted");
    p.stop();
    check(logs.count("undecodable") == 1, "decode failure logged once (5 failures)");
    check(logs.count("source frame too large") == 1, "oversized source logged once");
    check(logs.count("not a JPEG") == 1, "unsupported input logged once (several rejections)");
    check(logs.count("canvas unusable") == 1, "bad canvas logged once");
    check(logs.count("video preview started") == 1 && logs.count("video preview stopped") == 1,
          "start and stop logged");
    check(logs.count("disabled") == 1, "bad config logged");
}

static void testRateLimit() {
    std::printf("--- VideoPreview: rate limit ---\n");
    MockSink sink;
    sink.keepBytes = false;
    const auto j = makeJpeg(320, 240, Fmt::S420, solid(1, 2, 3));
    auto acceptedOf = [](const PreviewStats& s) {
        return sr(s, SubmitResult::Accepted) + sr(s, SubmitResult::ReplacedWaiting);
    };
    {
        VideoPreview p;
        p.start(defaultCfg(), sink);
        for (int i = 0; i < 30; ++i) submit(p, j, 320, 240, 5000000 + uint64_t(i) * 33333);
        PreviewStats s = p.stats();
        check(acceptedOf(s) == 10 && sr(s, SubmitResult::RateLimited) == 20,
              "30 fps for 1 s at TargetFps 10 -> 10 accepted, 20 rate-limited");
        // Jitter of ±3 ms does not change the count.
        uint64_t base = 7000000;
        for (int i = 0; i < 30; ++i)
            submit(p, j, 320, 240, base + uint64_t(i) * 33333 + uint64_t((i % 7) * 1000) - 3000);
        s = p.stats();
        check(acceptedOf(s) == 20, "jittered 30 fps (+-3 ms) -> 10 more accepted");
        // Capture clock steps back (camera restart without invalidateSource()).
        waitFor([&] { return settled(p, sink); });   // nothing waiting: the next is Accepted
        check(submit(p, j, 320, 240, 1000) == SubmitResult::Accepted,
              "clock stepped back by seconds -> schedule restarts (accepted)");
        check(submit(p, j, 320, 240, 1000 + 33333) == SubmitResult::RateLimited, "... and limits again");
        // A long gap does not allow a burst afterwards.
        waitFor([&] { return settled(p, sink); });
        const SubmitResult g1 = submit(p, j, 320, 240, 60000000);
        const SubmitResult g2 = submit(p, j, 320, 240, 60000000 + 33333);
        const SubmitResult g3 = submit(p, j, 320, 240, 60000000 + 66666);
        const SubmitResult g4 = submit(p, j, 320, 240, 60000000 + 99999);
        check(g1 == SubmitResult::Accepted && g2 == SubmitResult::RateLimited &&
                  g3 == SubmitResult::RateLimited && g4 != SubmitResult::RateLimited,
              "after a 1-minute gap: re-phased, next accepted 100 ms later (no burst)");
        waitFor([&] { return settled(p, sink); });
        p.stop();
    }
    {
        VideoPreview  p;
        PreviewConfig cfg = defaultCfg();
        cfg.targetFps = 15;
        p.start(cfg, sink);
        for (int i = 0; i < 30; ++i) submit(p, j, 320, 240, 5000000 + uint64_t(i) * 33333);
        const PreviewStats s = p.stats();
        check(acceptedOf(s) == 15, "30 fps at TargetFps 15 -> 15 accepted");
        p.stop();
    }
}

static void testReplaceInvalidateStop() {
    std::printf("--- VideoPreview: replacement, invalidate, stop in flight ---\n");
    const auto j = makeJpeg(640, 480, Fmt::S420, solid(90, 60, 30));
    {
        MockSink     sink;
        VideoPreview p;
        p.start(defaultCfg(), sink);
        sink.closeGate();
        check(submit(p, j, 640, 480, 0) == SubmitResult::Accepted, "f1 accepted");
        check(sink.waitInOffer(), "f1 converted, the sink is blocked in offerFrame()");
        check(submit(p, j, 640, 480, 100000) == SubmitResult::Accepted, "f2 accepted (waiting)");
        check(submit(p, j, 640, 480, 200000) == SubmitResult::ReplacedWaiting, "f3 replaces the waiting f2");
        sink.openGate();
        check(waitFor([&] { return settled(p, sink) && sink.seenCount() == 2; }), "both delivered after the sink unblocks");
        const auto seen = sink.seen();
        check(seen.size() == 2 && seen[0].seq == 1 && seen[1].seq == 3 && seen[1].captureUs == 200000,
              "delivered f1 and f3 (newest), never the replaced f2");
        const PreviewStats s = p.stats();
        check(s.submit[size_t(SubmitResult::ReplacedWaiting)] == 1 && s.processed == 2,
              "counted: 1 replacement, 2 processed");

        // invalidateSource() drops the waiting frame and resets the schedule.
        sink.closeGate();
        submit(p, j, 640, 480, 300000);
        sink.waitInOffer();
        check(submit(p, j, 640, 480, 400000) == SubmitResult::Accepted, "f5 waiting");
        p.invalidateSource();
        sink.openGate();
        check(waitFor([&] { return settled(p, sink) && sink.seenCount() == 3; }) &&
                  p.stats().discardedWaiting == 1,
              "invalidateSource(): waiting frame discarded, not delivered");
        check(submit(p, j, 640, 480, 50) == SubmitResult::Accepted, "rate schedule restarted (old timestamps accepted)");
        waitFor([&] { return settled(p, sink); });

        // ... and a frame already in progress is converted but not offered.
        sink.blockWorkerDemand();
        submit(p, j, 640, 480, 200050);
        check(sink.waitInDemand(), "worker picked up the next frame");
        p.invalidateSource();
        sink.unblockWorkerDemand();
        check(waitFor([&] { return settled(p, sink); }) && p.stats().invalidatedDropped == 1 &&
                  sink.seenCount() == 4,
              "in-progress frame from before the invalidate is dropped, not offered");

        // stop() with a frame in the sink and another waiting: joins the worker
        // once the sink returns, discards the waiting one.
        sink.closeGate();
        submit(p, j, 640, 480, 10000000);
        sink.waitInOffer();
        submit(p, j, 640, 480, 10100000);
        std::atomic<bool> stopped{false};
        std::thread stopper([&] { p.stop(); stopped = true; });
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        check(!stopped.load() && !p.running(), "stop() stops accepting at once, waits for the busy sink");
        check(p.submitJpeg(j.data(), j.size(), 640, 480, 20000000) == SubmitResult::Inactive,
              "submit during stop() -> Inactive");
        sink.openGate();
        stopper.join();
        const PreviewStats f = p.stats();
        check(stopped.load() && f.processed + f.discardedWaiting == sr(f, SubmitResult::Accepted) &&
                  outcomes(f) == f.processed,
              "stop() returned; every accepted frame accounted for");
        check(f.timingSamples > 0 && f.total.p95Ms > 0 && f.decode.lastMs > 0 && f.lastJpegBytes > 0 &&
                  f.lastScaleDenom == 2 && f.lastViewport == Viewport{0, 0, 320, 240},
              "timing and last-frame stats populated");
        check(p.submitJpeg(j.data(), j.size(), 640, 480, 30000000) == SubmitResult::Inactive,
              "submit after stop() -> Inactive");
        p.stop();
        check(true, "second stop() is a no-op");

        // Restart: fresh counters, works again.
        sink.takes = 0;
        check(p.start(defaultCfg(), sink) && submit(p, j, 640, 480, 0) == SubmitResult::Accepted &&
                  waitFor([&] { return settled(p, sink); }) && p.stats().submitted == 1,
              "restart after stop(): fresh counters, frame delivered");
        p.stop();
    }
    {
        // Burst then immediate stop: no hang, no crash, counters consistent.
        MockSink sink;
        sink.keepBytes = false;
        VideoPreview p;
        p.start(defaultCfg(), sink);
        for (int i = 0; i < 50; ++i) p.submitJpeg(j.data(), j.size(), 640, 480, uint64_t(i) * 100000);
        p.stop();
        const PreviewStats s = p.stats();
        check(s.processed + s.discardedWaiting == sr(s, SubmitResult::Accepted) && outcomes(s) == s.processed,
              "burst + immediate stop(): consistent (" + std::to_string(s.processed) + " processed)");
    }
}

static void testSinkBackpressure() {
    std::printf("--- VideoPreview: refusal, held leases, byte cap ---\n");
    const auto j = makeJpeg(640, 480, Fmt::S420, solid(20, 200, 20));
    MockSink   sink;
    sink.keepBytes = false;
    VideoPreview p;
    p.start(defaultCfg(), sink);
    sink.refuse = true;
    uint64_t t = 0;
    for (int i = 0; i < 100; ++i) {
        t += 100000;
        submit(p, j, 640, 480, t);
        waitFor([&] { return settled(p, sink); });
    }
    PreviewStats s = p.stats();
    check(s.offered == 100 && s.refused == 100 && s.outputSlotsFree == 2 && s.bytesOut == 0,
          "100 refused offers: slots all back (no leak), nothing counted as sent");
    sink.refuse = false;
    t += 100000;
    submit(p, j, 640, 480, t);
    check(waitFor([&] { return settled(p, sink) && sink.takes == 1; }) && p.stats().bytesOut > 0,
          "after refusals a frame is delivered");

    sink.hold = true;
    for (int i = 0; i < 2; ++i) {
        t += 100000;
        submit(p, j, 640, 480, t);
        waitFor([&] { return settled(p, sink); });
    }
    check(sink.takes == 3 && p.stats().outputSlotsFree == 0, "sink holds both output slots");
    t += 100000;
    submit(p, j, 640, 480, t);
    waitFor([&] { return settled(p, sink); });
    s = p.stats();
    check(s.noOutputSlot == 1 && s.decoded == 103 && sink.takes == 3,
          "no free output slot -> dropped before decoding (noOutputSlot)");
    sink.hold = false;
    sink.releaseHeld();
    t += 100000;
    submit(p, j, 640, 480, t);
    check(waitFor([&] { return settled(p, sink) && sink.takes == 4; }) && p.stats().outputSlotsFree == 2,
          "held leases released -> delivery resumes, both slots free");

    // Device byte cap below anything a frame can reach: retry, then drop.
    const auto noisy = makeJpeg(640, 480, Fmt::S420, noise(60), 92);
    sink.maxBytes = 1500;
    t += 100000;
    submit(p, noisy, 640, 480, t);
    waitFor([&] { return settled(p, sink); });
    s = p.stats();
    check(s.encodeRetries == 1 && s.encodeOversizedDropped == 1 && sink.takes == 4 && s.outputSlotsFree == 2,
          "device cap 1500 B: retried once at q45, dropped, slot returned");
    sink.maxBytes = 0;
    t += 100000;
    submit(p, noisy, 640, 480, t);
    check(waitFor([&] { return settled(p, sink) && sink.takes == 5; }), "cap lifted: the noisy frame is delivered");

    // Demand turns inactive while a frame waits: the worker drops it unconverted.
    sink.closeGate();
    t += 100000;
    submit(p, j, 640, 480, t);
    sink.waitInOffer();
    t += 100000;
    submit(p, j, 640, 480, t);
    sink.active = false;
    sink.openGate();
    check(waitFor([&] { return settled(p, sink); }) && p.stats().inactiveDropped == 1,
          "demand inactive at pickup -> dropped (inactiveDropped)");
    sink.active = true;
    p.stop();

    // A lease the sink still holds outlives the VideoPreview that made it.
    MockSink holder;
    holder.hold = true;
    {
        VideoPreview q;
        q.start(defaultCfg(), holder);
        submit(q, j, 640, 480, 0);
        waitFor([&] { return settled(q, holder) && holder.takes == 1; });
    }   // destroyed
    std::vector<EncodedFrameLease> kept = holder.takeHeld();
    const Img img = kept.size() == 1 ? decodeJpeg(kept[0].data(), kept[0].size()) : Img{};
    check(img.ok && img.w == 320 && img.h == 240, "lease held past the preview's destruction is still valid");
    kept.clear();
    check(true, "... and releases cleanly afterwards");
}

static void testStress() {
    std::printf("--- VideoPreview: concurrent submitters ---\n");
    const auto j = makeJpeg(640, 480, Fmt::S420, leftRight(640));
    MockSink   sink;
    sink.keepBytes        = false;
    sink.holdOne          = true;
    sink.refuseEveryThird = true;
    VideoPreview  p;
    PreviewConfig cfg = defaultCfg();
    cfg.targetFps = 15;
    p.start(cfg, sink);

    constexpr int kThreads = 4, kPerThread = 1500;
    std::atomic<uint64_t> clock{1000000};
    std::atomic<bool>     go{false}, done{false};
    std::vector<std::array<uint64_t, kSubmitResultCount>> tally(kThreads);
    std::vector<std::thread> threads;
    for (int k = 0; k < kThreads; ++k) {
        tally[k].fill(0);
        threads.emplace_back([&, k] {
            while (!go.load()) std::this_thread::yield();
            for (int i = 0; i < kPerThread; ++i) {
                const uint64_t t = clock.fetch_add(5000);
                ++tally[k][size_t(p.submitJpeg(j.data(), j.size(), 640, 480, t))];
                std::this_thread::sleep_for(std::chrono::microseconds(400));   // ~2.5 kHz per thread
            }
        });
    }
    std::thread chaos([&] {   // demand flips and camera "recoveries" meanwhile
        int n = 0;
        while (!done.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            if (++n % 7 == 0) sink.active = !sink.active.load();
            if (n % 11 == 0) p.invalidateSource();
        }
        sink.active = true;
    });
    go = true;
    for (auto& t : threads) t.join();
    done = true;
    chaos.join();
    p.stop();

    const PreviewStats s = p.stats();
    std::array<uint64_t, kSubmitResultCount> sum{};
    for (const auto& t : tally)
        for (size_t i = 0; i < kSubmitResultCount; ++i) sum[i] += t[i];
    bool same = true;
    for (size_t i = 0; i < kSubmitResultCount; ++i) same = same && sum[i] == s.submit[i];
    std::printf("    %llu submitted: %llu accepted, %llu replaced, %llu inactive, %llu rate-limited, %llu busy;"
                " %llu offered (%llu refused), %llu invalidated\n",
                (unsigned long long)s.submitted, (unsigned long long)sr(s, SubmitResult::Accepted),
                (unsigned long long)sr(s, SubmitResult::ReplacedWaiting),
                (unsigned long long)sr(s, SubmitResult::Inactive),
                (unsigned long long)sr(s, SubmitResult::RateLimited), (unsigned long long)sr(s, SubmitResult::Busy),
                (unsigned long long)s.offered, (unsigned long long)s.refused,
                (unsigned long long)(s.invalidatedDropped + s.discardedWaiting));
    check(s.submitted == uint64_t(kThreads) * kPerThread && same, "per-result counts match what the submitters saw");
    check(s.processed + s.discardedWaiting == sr(s, SubmitResult::Accepted), "accepted = processed + discarded");
    check(outcomes(s) == s.processed, "every processed frame has exactly one outcome");
    check(s.offered == s.refused + sink.takes.load() && s.offered > 0, "offered = refused + taken");
    check(s.decodeFailed == 0 && s.encodeFailed == 0, "no conversion failures under contention");
    check(sr(s, SubmitResult::Inactive) > 0 && s.invalidatedDropped + s.discardedWaiting > 0 && s.processed > 20,
          "the run exercised inactive demand, invalidation and steady conversion");
    sink.releaseHeld();
}

// ─── optional benchmark ───────────────────────────────────────────────────────

static double pct(std::vector<double> v, double p) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    size_t idx = size_t(std::ceil(p * double(v.size()))) - 1;
    return v[std::min(idx, v.size() - 1)];
}

static void runBench(const char* dir) {
    std::printf("--- benchmark: %s ---\n", dir);
    std::vector<std::string> files;
    std::error_code ec;
    for (const auto& e : std::filesystem::directory_iterator(dir, ec))
        if (e.is_regular_file() && e.path().extension() == ".jpg") files.push_back(e.path().string());
    std::sort(files.begin(), files.end());
    if (files.empty()) { std::printf("  no *.jpg in %s\n", dir); return; }
    std::vector<std::vector<uint8_t>> frames;
    for (const auto& f : files) {
        std::ifstream in(f, std::ios::binary);
        frames.emplace_back(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    }
    FrameConverter conv;
    conv.init();
    std::vector<uint8_t> out(65536);
    auto run = [&](const std::vector<uint8_t>& j) {
        ConvertRequest rq;
        rq.jpeg = j.data(); rq.size = j.size(); rq.canvasW = 320; rq.canvasH = 240;
        rq.quality = 65; rq.minimumQuality = 45; rq.out = out.data(); rq.outCapacity = out.size();
        return conv.convert(rq);
    };
    for (size_t i = 0; i < std::min<size_t>(5, frames.size()); ++i) run(frames[i]);   // warm-up
    std::vector<double> dec, rs, enc, tot, bytes, srcBytes;
    int bad = 0, warnings = 0;
    ConvertResult last;
    for (const auto& f : frames) {
        const auto t0 = Clock::now();
        const ConvertResult r = run(f);
        const double ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        if (r.status != ConvertStatus::Ok) {
            ++bad;
            std::printf("  %s: %s\n", convertStatusName(r.status), r.detail);
            continue;
        }
        warnings += int(r.warnings);
        dec.push_back(r.decodeMs); rs.push_back(r.resizeMs); enc.push_back(r.encodeMs); tot.push_back(ms);
        bytes.push_back(double(r.jpegBytes)); srcBytes.push_back(double(f.size()));
        last = r;
    }
    std::printf("  %zu frames (%d failed, %d benign warnings), source %ux%u p50 %.1f KiB, DCT 1/%u -> %ux%u q%d\n",
                frames.size(), bad, warnings, last.sourceWidth, last.sourceHeight, pct(srcBytes, 0.5) / 1024.0,
                last.scaleDenom, unsigned(last.width), unsigned(last.height), last.quality);
    std::printf("  decode  p50 %6.2f ms  p95 %6.2f ms\n", pct(dec, 0.5), pct(dec, 0.95));
    std::printf("  resize  p50 %6.2f ms  p95 %6.2f ms\n", pct(rs, 0.5), pct(rs, 0.95));
    std::printf("  encode  p50 %6.2f ms  p95 %6.2f ms\n", pct(enc, 0.5), pct(enc, 0.95));
    std::printf("  total   p50 %6.2f ms  p95 %6.2f ms\n", pct(tot, 0.5), pct(tot, 0.95));
    std::printf("  output  p50 %.1f KiB  p95 %.1f KiB  max %.1f KiB\n", pct(bytes, 0.5) / 1024.0,
                pct(bytes, 0.95) / 1024.0, pct(bytes, 1.0) / 1024.0);
    check(bad == 0, "every benchmark frame converted");
}

int main() {
    testLetterbox();
    testDctScale();
    testPool();
    testConverter();
    testConverterCapAndErrors();
    testConfigAndSubmitRejections();
    testRateLimit();
    testReplaceInvalidateStop();
    testSinkBackpressure();
    testStress();
    if (const char* dir = std::getenv("PREVIEW_BENCH_DIR")) runBench(dir);

    std::printf("\n%s (%d failure%s)\n", g_fails ? "FAILED" : "ALL PASSED", g_fails, g_fails == 1 ? "" : "s");
    return g_fails ? 1 : 0;
}
