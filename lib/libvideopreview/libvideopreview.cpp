/**
 * @file libvideopreview.cpp
 * @brief EncodedFramePool / EncodedFrameLease and the VideoPreview pipeline
 *        (never-blocking submission, one conversion worker, sink handoff).
 *
 * Input handoff: two preallocated slots guarded by inMu_.  Submitters only
 * try_lock() it (contention → Busy) and copy the camera bytes while holding
 * it; the worker holds it only to swap slot indices.  At any time at most one
 * slot is "waiting" (newest accepted frame) and one "in progress" (being
 * converted, read by the worker without the lock — submitters never write it).
 */

#include "libvideopreview.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <limits>
#include <new>
#include <system_error>

namespace dashcam::preview {

using dashcam::log::LogLevel;
using dashcam::log::logPrintf;

// ─── EncodedFrameLease ────────────────────────────────────────────────────────

EncodedFrameLease::EncodedFrameLease(std::shared_ptr<EncodedFramePool> pool, uint32_t slot) noexcept
    : pool_(std::move(pool)), slot_(slot) {}

EncodedFrameLease::~EncodedFrameLease() { release(); }

EncodedFrameLease::EncodedFrameLease(EncodedFrameLease&& o) noexcept
    : pool_(std::move(o.pool_)), slot_(o.slot_), size_(o.size_), width_(o.width_),
      height_(o.height_), captureUs_(o.captureUs_), sequence_(o.sequence_), quality_(o.quality_) {
    o.pool_.reset();
    o.size_ = 0;
}

EncodedFrameLease& EncodedFrameLease::operator=(EncodedFrameLease&& o) noexcept {
    if (this != &o) {
        release();
        pool_      = std::move(o.pool_);
        slot_      = o.slot_;
        size_      = o.size_;
        width_     = o.width_;
        height_    = o.height_;
        captureUs_ = o.captureUs_;
        sequence_  = o.sequence_;
        quality_   = o.quality_;
        o.pool_.reset();
        o.size_ = 0;
    }
    return *this;
}

void EncodedFrameLease::release() noexcept {
    if (!pool_) return;
    pool_->releaseSlot(slot_);
    pool_.reset();   // may free the pool if VideoPreview already let go of it
    size_ = 0;
}

const uint8_t* EncodedFrameLease::data() const noexcept {
    return pool_ ? pool_->slotData(slot_) : nullptr;
}

uint8_t* EncodedFrameLease::writableData() noexcept {
    return pool_ ? pool_->slotData(slot_) : nullptr;
}

size_t EncodedFrameLease::capacity() const noexcept { return pool_ ? pool_->slotBytes() : 0; }

bool EncodedFrameLease::setFrame(size_t size, uint16_t width, uint16_t height,
                                 uint64_t captureMonotonicUs, uint64_t sourceSequence,
                                 int quality) noexcept {
    if (!pool_ || size > pool_->slotBytes()) return false;
    size_      = size;
    width_     = width;
    height_    = height;
    captureUs_ = captureMonotonicUs;
    sequence_  = sourceSequence;
    quality_   = quality;
    return true;
}

// ─── EncodedFramePool ─────────────────────────────────────────────────────────

EncodedFramePool::EncodedFramePool(size_t slotCount, size_t slotBytes)
    : slotCount_(slotCount), slotBytes_(slotBytes),
      storage_(new uint8_t[slotCount * slotBytes]()),   // zeroed: no page faults later
      inUse_(new std::atomic<bool>[slotCount]()) {
    for (size_t i = 0; i < slotCount_; ++i) inUse_[i].store(false, std::memory_order_relaxed);
}

std::shared_ptr<EncodedFramePool> EncodedFramePool::create(size_t slotCount, size_t slotBytes) {
    if (slotCount == 0 || slotCount > 64 || slotBytes == 0 ||
        slotBytes > std::numeric_limits<size_t>::max() / slotCount)
        return nullptr;
    try {
        return std::shared_ptr<EncodedFramePool>(new EncodedFramePool(slotCount, slotBytes));
    } catch (const std::bad_alloc&) {
        return nullptr;
    }
}

EncodedFrameLease EncodedFramePool::tryAcquire() {
    for (uint32_t i = 0; i < slotCount_; ++i) {
        bool expected = false;
        if (inUse_[i].compare_exchange_strong(expected, true, std::memory_order_acquire,
                                              std::memory_order_relaxed))
            return EncodedFrameLease(shared_from_this(), i);
    }
    return {};
}

size_t EncodedFramePool::freeSlots() const noexcept {
    size_t n = 0;
    for (size_t i = 0; i < slotCount_; ++i)
        if (!inUse_[i].load(std::memory_order_acquire)) ++n;
    return n;
}

void EncodedFramePool::releaseSlot(uint32_t slot) noexcept {
    if (slot < slotCount_) inUse_[slot].store(false, std::memory_order_release);
}

// ─── helpers ──────────────────────────────────────────────────────────────────

const char* submitResultName(SubmitResult r) {
    switch (r) {
    case SubmitResult::Accepted:        return "accepted";
    case SubmitResult::ReplacedWaiting: return "replaced-waiting";
    case SubmitResult::Inactive:        return "inactive";
    case SubmitResult::RateLimited:     return "rate-limited";
    case SubmitResult::Oversized:       return "oversized";
    case SubmitResult::Unsupported:     return "unsupported";
    case SubmitResult::Busy:            return "busy";
    }
    return "?";
}

namespace {

// First-occurrence log kinds (VideoPreview::loggedMask_ / pendingLog_ bits).
constexpr uint32_t kLogOversized      = 1u << 0;   // submit side
constexpr uint32_t kLogUnsupported    = 1u << 1;   // submit side
constexpr uint32_t kLogBadCanvas      = 1u << 2;   // submit side
constexpr uint32_t kLogDecode         = 1u << 3;
constexpr uint32_t kLogSourceFormat   = 1u << 4;
constexpr uint32_t kLogEncodeTooBig   = 1u << 5;
constexpr uint32_t kLogEncodeFailed   = 1u << 6;
constexpr uint32_t kLogNoSlot         = 1u << 7;
constexpr uint32_t kLogRefused        = 1u << 8;
constexpr uint32_t kLogGeometry       = 1u << 9;
constexpr uint32_t kLogRetry          = 1u << 10;

bool canvasUsable(const PreviewDemand& d) {
    return d.canvasW >= 2 && d.canvasH >= 2 && d.canvasW <= kMaxCanvasWidth &&
           d.canvasH <= kMaxCanvasHeight;
}

/// nullptr when @p c is usable, else why not.
const char* validateConfig(const PreviewConfig& c) {
    if (c.targetFps < kMinTargetFps || c.targetFps > kMaxTargetFps) return "TargetFps outside 1-15";
    if (c.jpegQuality < 1 || c.jpegQuality > 100) return "JpegQuality outside 1-100";
    if (c.minimumJpegQuality < 1 || c.minimumJpegQuality > c.jpegQuality)
        return "MinimumJpegQuality outside 1-JpegQuality";
    if (c.maxJpegBytes < kMinJpegBytes || c.maxJpegBytes > kMaxJpegBytesLimit)
        return "MaxJpegBytes outside 1 KiB - 1 MiB";
    if (c.maxSourceJpegBytes < kMinSourceJpegBytes || c.maxSourceJpegBytes > kMaxSourceJpegBytesLimit)
        return "source slot size outside 4 KiB - 16 MiB";
    return nullptr;
}

double msBetween(std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
}

} // namespace

void VideoPreview::TimingRing::push(double ms) {
    v[head] = static_cast<float>(ms);
    head    = (head + 1) % kTimingWindow;
    if (count < kTimingWindow) ++count;
}

PreviewTiming VideoPreview::TimingRing::summary(double lastMs) const {
    PreviewTiming t;
    t.lastMs = lastMs;
    if (count == 0) return t;
    std::array<float, kTimingWindow> s;
    std::copy(v.begin(), v.begin() + count, s.begin());
    const size_t idx = (size_t(count) * 95 + 99) / 100 - 1;   // ceil(0.95·n) − 1
    std::nth_element(s.begin(), s.begin() + idx, s.begin() + count);
    t.p95Ms = s[idx];
    return t;
}

// ─── VideoPreview: lifecycle ──────────────────────────────────────────────────

VideoPreview::VideoPreview() = default;

VideoPreview::~VideoPreview() { stop(); }

bool VideoPreview::start(const PreviewConfig& cfg, IPreviewSink& sink, dashcam::log::LogCallback log) {
    std::lock_guard<std::mutex> life(lifeMu_);
    if (worker_.joinable()) {
        logPrintf(log, LogLevel::WARN, "video preview: start() while already running - ignored");
        return false;
    }
    if (const char* why = validateConfig(cfg)) {
        logPrintf(log, LogLevel::ERROR, "video preview disabled: %s", why);
        return false;
    }
    log_ = std::move(log);

    try {
        auto pool = EncodedFramePool::create(2, cfg.maxJpegBytes);
        auto conv = std::make_unique<FrameConverter>();
        if (!pool || !conv->init()) throw std::bad_alloc();
        std::lock_guard<std::mutex> lk(inMu_);
        for (InputSlot& s : in_) {
            s = InputSlot{};
            s.buf.assign(cfg.maxSourceJpegBytes, 0);   // zeroed: no page faults on the camera thread
        }
        conv_ = std::move(conv);
        std::lock_guard<std::mutex> sl(statsMu_);
        pool_ = std::move(pool);
    } catch (const std::bad_alloc&) {
        logPrintf(log_, LogLevel::ERROR, "video preview disabled: buffer allocation failed");
        releaseResources();
        return false;
    }

    // Fresh counters for this session.
    for (auto& c : submit_) c.store(0, std::memory_order_relaxed);
    for (std::atomic<uint64_t>* c :
         {&processed_, &discardedWaiting_, &inactiveDropped_, &noOutputSlot_, &decodeFailed_,
          &unsupportedSource_, &encodeOversized_, &encodeFailed_, &invalidatedDropped_, &offered_,
          &refused_, &decoded_, &encodeRetries_, &geometryMismatch_, &decodeWarnings_, &bytesOut_})
        c->store(0, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> sl(statsMu_);
        lastJpegBytes_ = maxJpegBytes_ = 0;
        lastQuality_ = 0;
        lastSourceW_ = lastSourceH_ = lastScaleDenom_ = 0;
        lastViewport_ = Viewport{};
        lastDecodeMs_ = lastResizeMs_ = lastEncodeMs_ = lastTotalMs_ = 0;
        decodeRing_ = resizeRing_ = encodeRing_ = totalRing_ = TimingRing{};
    }
    pendingLog_.store(0);
    loggedMask_.store(0);
    geometryInvalid_.store(false);
    {
        std::lock_guard<std::mutex> lk(inMu_);
        cfg_            = cfg;
        sink_           = &sink;
        waiting_        = -1;
        inProgress_     = -1;
        stopping_       = false;
        running_        = true;
        haveRate_       = false;
        nextSequence_   = 1;
    }

    try {
        worker_ = std::thread([this] { workerLoop(); });
    } catch (const std::system_error& e) {
        logPrintf(log_, LogLevel::ERROR, "video preview disabled: worker thread failed: %s", e.what());
        {
            std::lock_guard<std::mutex> lk(inMu_);
            running_ = false;
            sink_    = nullptr;
        }
        releaseResources();
        return false;
    }
    accepting_.store(true, std::memory_order_release);
    logPrintf(log_, LogLevel::INFO,
              "video preview started: <= %u fps, JPEG q%d (retry q%d), <= %zu B per frame, "
              "%zu B input slots",
              cfg.targetFps, cfg.jpegQuality, cfg.minimumJpegQuality, cfg.maxJpegBytes,
              cfg.maxSourceJpegBytes);
    return true;
}

void VideoPreview::stop() {
    std::lock_guard<std::mutex> life(lifeMu_);
    accepting_.store(false, std::memory_order_release);
    bool wasRunning = false;
    {
        std::lock_guard<std::mutex> lk(inMu_);   // waits out a submitter's copy
        wasRunning = running_;
        stopping_  = true;
    }
    inCv_.notify_all();
    if (worker_.joinable()) worker_.join();
    {
        std::lock_guard<std::mutex> lk(inMu_);
        if (waiting_ >= 0) {
            discardedWaiting_.fetch_add(1, std::memory_order_relaxed);
            waiting_ = -1;
        }
        inProgress_ = -1;
        running_    = false;
        sink_       = nullptr;
    }
    if (wasRunning) {
        const PreviewStats s = stats();
        logPrintf(log_, LogLevel::INFO,
                  "video preview stopped: %llu submitted, %llu converted, %llu offered "
                  "(%llu refused), %llu decode failures, %llu oversized drops",
                  (unsigned long long)s.submitted, (unsigned long long)s.decoded,
                  (unsigned long long)s.offered, (unsigned long long)s.refused,
                  (unsigned long long)s.decodeFailed, (unsigned long long)s.encodeOversizedDropped);
    }
    releaseResources();
}

void VideoPreview::releaseResources() {
    {
        std::lock_guard<std::mutex> lk(inMu_);
        for (InputSlot& s : in_) s = InputSlot{};   // frees the buffers
    }
    {
        std::lock_guard<std::mutex> sl(statsMu_);
        pool_.reset();   // leases still held by the sink keep the pool alive
    }
    conv_.reset();
}

// ─── VideoPreview: submission (camera streaming thread) ──────────────────────

SubmitResult VideoPreview::count(SubmitResult r) {
    submit_[static_cast<size_t>(r)].fetch_add(1, std::memory_order_relaxed);
    return r;
}

void VideoPreview::noteOnce(uint32_t bit) {
    if (loggedMask_.fetch_or(bit, std::memory_order_relaxed) & bit) return;
    pendingLog_.fetch_or(bit, std::memory_order_relaxed);   // the worker logs it
}

SubmitResult VideoPreview::submitJpeg(const uint8_t* bytes, size_t size, uint32_t srcW, uint32_t srcH,
                                      uint64_t captureMonotonicUs) {
    if (!accepting_.load(std::memory_order_acquire)) return count(SubmitResult::Inactive);
    std::unique_lock<std::mutex> lk(inMu_, std::try_to_lock);
    if (!lk.owns_lock()) return count(SubmitResult::Busy);
    if (stopping_ || !running_ || sink_ == nullptr) return count(SubmitResult::Inactive);

    const uint32_t pendingBefore = pendingLog_.load(std::memory_order_relaxed);
    SubmitResult   result;
    const PreviewDemand d = sink_->demand();
    if (!d.active) {
        result = SubmitResult::Inactive;
    } else if (!canvasUsable(d)) {
        noteOnce(kLogBadCanvas);
        result = SubmitResult::Unsupported;
    } else if (bytes == nullptr || size < 4 || srcW == 0 || srcH == 0) {
        noteOnce(kLogUnsupported);
        result = SubmitResult::Unsupported;
    } else if (size > cfg_.maxSourceJpegBytes) {
        noteOnce(kLogOversized);
        result = SubmitResult::Oversized;
    } else if (bytes[0] != 0xFF || bytes[1] != 0xD8 || bytes[2] != 0xFF) {   // SOI + next marker
        noteOnce(kLogUnsupported);
        result = SubmitResult::Unsupported;
    } else {
        // Rate limit on capture time.  A frame is due once it is within a
        // quarter interval of the schedule (source jitter: 30 fps → 10 fps
        // keeps every third frame).  An on-time frame advances the schedule by
        // one interval, keeping its phase; a late one (gap, slow source)
        // re-phases it to its own time, so a gap never allows a burst.
        const uint64_t interval = 1000000ull / cfg_.targetFps;
        const uint64_t tol      = interval / 4;
        const uint64_t t        = captureMonotonicUs;
        const bool resync = !haveRate_ || t + interval < lastAcceptedUs_;   // first frame / clock went back
        if (!resync && t + tol < nextDueUs_) {
            result = SubmitResult::RateLimited;
        } else {
            const bool onTime = !resync && t <= nextDueUs_ + tol;
            nextDueUs_      = onTime ? nextDueUs_ + interval : t + interval;
            lastAcceptedUs_ = t;
            haveRate_       = true;

            int idx;
            if (waiting_ >= 0) {
                idx    = waiting_;
                result = SubmitResult::ReplacedWaiting;
            } else {
                idx    = (inProgress_ == 0) ? 1 : 0;
                result = SubmitResult::Accepted;
            }
            InputSlot& s = in_[idx];
            std::memcpy(s.buf.data(), bytes, size);
            s.size       = size;
            s.srcW       = srcW;
            s.srcH       = srcH;
            s.captureUs  = t;
            s.sequence   = nextSequence_++;
            s.generation = generation_.load(std::memory_order_relaxed);
            waiting_     = idx;
        }
    }
    const bool wake = result == SubmitResult::Accepted || result == SubmitResult::ReplacedWaiting ||
                      pendingLog_.load(std::memory_order_relaxed) != pendingBefore;
    lk.unlock();
    if (wake) inCv_.notify_one();   // never blocks
    return count(result);
}

void VideoPreview::invalidateSource() {
    std::lock_guard<std::mutex> lk(inMu_);   // bounded: held by others for one copy at most
    if (waiting_ >= 0) {
        discardedWaiting_.fetch_add(1, std::memory_order_relaxed);
        waiting_ = -1;
    }
    generation_.fetch_add(1, std::memory_order_relaxed);   // the in-progress frame is not offered
    haveRate_ = false;                                     // capture clock may restart
    geometryInvalid_.store(true, std::memory_order_relaxed);
}

// ─── VideoPreview: worker ─────────────────────────────────────────────────────

void VideoPreview::flushPendingLogs() {
    const uint32_t bits = pendingLog_.exchange(0, std::memory_order_relaxed);
    if (bits & kLogOversized)
        logPrintf(log_, LogLevel::WARN,
                  "video preview: source frame too large (> %zu B input slot; camera mode changed?) "
                  "- dropping such frames, recording unaffected",
                  cfg_.maxSourceJpegBytes);
    if (bits & kLogUnsupported)
        logPrintf(log_, LogLevel::WARN,
                  "video preview: source frame is not a JPEG (or has no size) - preview supports "
                  "the MJPEG camera path only");
    if (bits & kLogBadCanvas)
        logPrintf(log_, LogLevel::WARN,
                  "video preview: display canvas unusable (needs 2x2 to %ux%u) - no frames converted",
                  unsigned(kMaxCanvasWidth), unsigned(kMaxCanvasHeight));
}

void VideoPreview::workerLoop() {
    for (;;) {
        int idx = -1;
        {
            std::unique_lock<std::mutex> lk(inMu_);
            inCv_.wait(lk, [this] {
                return stopping_ || waiting_ >= 0 || pendingLog_.load(std::memory_order_relaxed) != 0;
            });
            if (stopping_) break;
            if (waiting_ >= 0) {
                idx         = waiting_;
                waiting_    = -1;
                inProgress_ = idx;
            }
        }
        flushPendingLogs();
        if (idx < 0) continue;
        processFrame(in_[idx]);   // submitters never write the in-progress slot
        std::lock_guard<std::mutex> lk(inMu_);
        inProgress_ = -1;
    }
    flushPendingLogs();
}

void VideoPreview::processFrame(const InputSlot& in) {
    const auto t0 = std::chrono::steady_clock::now();
    processed_.fetch_add(1, std::memory_order_relaxed);
    auto firstTime = [this](uint32_t bit) {
        return !(loggedMask_.fetch_or(bit, std::memory_order_relaxed) & bit);
    };

    const PreviewDemand d = sink_->demand();
    if (!d.active || !canvasUsable(d)) {
        inactiveDropped_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    // No free output slot: the sink still holds both.  Drop before spending
    // any CPU on a frame that could not be delivered.
    EncodedFrameLease lease = pool_->tryAcquire();
    if (!lease) {
        noOutputSlot_.fetch_add(1, std::memory_order_relaxed);
        if (firstTime(kLogNoSlot))
            logPrintf(log_, LogLevel::INFO,
                      "video preview: display still holds both output frames - dropping (slow sink)");
        return;
    }
    if (geometryInvalid_.exchange(false, std::memory_order_relaxed)) conv_->invalidateGeometry();

    ConvertRequest rq;
    rq.jpeg           = in.buf.data();
    rq.size           = in.size;
    rq.canvasW        = d.canvasW;
    rq.canvasH        = d.canvasH;
    rq.quality        = cfg_.jpegQuality;
    rq.minimumQuality = cfg_.minimumJpegQuality;
    rq.out            = lease.writableData();
    rq.outCapacity    = std::min(cfg_.maxJpegBytes, lease.capacity());
    if (d.maxJpegBytes != 0) rq.outCapacity = std::min(rq.outCapacity, d.maxJpegBytes);
    const ConvertResult r = conv_->convert(rq);

    if (r.retried) {
        encodeRetries_.fetch_add(1, std::memory_order_relaxed);
        if (firstTime(kLogRetry))
            logPrintf(log_, LogLevel::INFO,
                      "video preview: frame over %zu B at q%d, re-encoded at q%d", rq.outCapacity,
                      cfg_.jpegQuality, cfg_.minimumJpegQuality);
    }
    if (r.sourceWidth != 0 && (r.sourceWidth != in.srcW || r.sourceHeight != in.srcH)) {
        geometryMismatch_.fetch_add(1, std::memory_order_relaxed);
        if (firstTime(kLogGeometry))
            logPrintf(log_, LogLevel::WARN,
                      "video preview: JPEG is %ux%u but the source was announced as %ux%u - "
                      "using the JPEG's size",
                      r.sourceWidth, r.sourceHeight, in.srcW, in.srcH);
    }

    const bool decodedOk = r.status == ConvertStatus::Ok || r.status == ConvertStatus::EncodeOversized ||
                           r.status == ConvertStatus::EncodeFailed;
    switch (r.status) {
    case ConvertStatus::Ok:
        break;
    case ConvertStatus::DecodeFailed:
        decodeFailed_.fetch_add(1, std::memory_order_relaxed);
        if (firstTime(kLogDecode))
            logPrintf(log_, LogLevel::WARN, "video preview: undecodable camera JPEG dropped (%s)",
                      r.detail[0] ? r.detail : "no detail");
        break;
    case ConvertStatus::UnsupportedSource:
    case ConvertStatus::InvalidArgument:
        unsupportedSource_.fetch_add(1, std::memory_order_relaxed);
        if (firstTime(kLogSourceFormat))
            logPrintf(log_, LogLevel::WARN, "video preview: unsupported camera JPEG dropped (%s)",
                      r.detail[0] ? r.detail : convertStatusName(r.status));
        break;
    case ConvertStatus::EncodeOversized:
        encodeOversized_.fetch_add(1, std::memory_order_relaxed);
        if (firstTime(kLogEncodeTooBig))
            logPrintf(log_, LogLevel::WARN,
                      "video preview: frame still over %zu B at q%d - dropped (noisy scene?)",
                      rq.outCapacity, cfg_.minimumJpegQuality);
        break;
    case ConvertStatus::EncodeFailed:
        encodeFailed_.fetch_add(1, std::memory_order_relaxed);
        if (firstTime(kLogEncodeFailed))
            logPrintf(log_, LogLevel::ERROR, "video preview: JPEG encode failed (%s)", r.detail);
        break;
    }
    if (decodedOk) {
        decoded_.fetch_add(1, std::memory_order_relaxed);
        decodeWarnings_.fetch_add(r.warnings, std::memory_order_relaxed);
    }

    bool taken = false;
    if (r.status == ConvertStatus::Ok) {
        if (in.generation != generation_.load(std::memory_order_relaxed)) {
            invalidatedDropped_.fetch_add(1, std::memory_order_relaxed);   // camera recovered meanwhile
        } else {
            lease.setFrame(r.jpegBytes, r.width, r.height, in.captureUs, in.sequence, r.quality);
            offered_.fetch_add(1, std::memory_order_relaxed);
            taken = sink_->offerFrame(std::move(lease));
            if (taken) {
                bytesOut_.fetch_add(r.jpegBytes, std::memory_order_relaxed);
            } else {
                refused_.fetch_add(1, std::memory_order_relaxed);
                if (firstTime(kLogRefused))
                    logPrintf(log_, LogLevel::DEBUG, "video preview: display refused a frame");
            }
        }
    }
    lease.release();   // no-op when the sink took it

    if (!decodedOk) return;
    const double totalMs = msBetween(t0, std::chrono::steady_clock::now());
    std::lock_guard<std::mutex> sl(statsMu_);
    lastSourceW_    = r.sourceWidth;
    lastSourceH_    = r.sourceHeight;
    lastScaleDenom_ = r.scaleDenom;
    lastViewport_   = r.viewport;
    lastDecodeMs_   = r.decodeMs;
    lastResizeMs_   = r.resizeMs;
    lastEncodeMs_   = r.encodeMs;
    lastTotalMs_    = totalMs;
    decodeRing_.push(r.decodeMs);
    resizeRing_.push(r.resizeMs);
    encodeRing_.push(r.encodeMs);
    totalRing_.push(totalMs);
    if (r.status == ConvertStatus::Ok) {
        lastJpegBytes_ = r.jpegBytes;
        maxJpegBytes_  = std::max(maxJpegBytes_, r.jpegBytes);
        lastQuality_   = r.quality;
    }
}

// ─── VideoPreview: stats ──────────────────────────────────────────────────────

PreviewStats VideoPreview::stats() const {
    PreviewStats s;
    for (size_t i = 0; i < kSubmitResultCount; ++i) {
        s.submit[i] = submit_[i].load(std::memory_order_relaxed);
        s.submitted += s.submit[i];
    }
    s.processed              = processed_.load(std::memory_order_relaxed);
    s.discardedWaiting       = discardedWaiting_.load(std::memory_order_relaxed);
    s.inactiveDropped        = inactiveDropped_.load(std::memory_order_relaxed);
    s.noOutputSlot           = noOutputSlot_.load(std::memory_order_relaxed);
    s.decodeFailed           = decodeFailed_.load(std::memory_order_relaxed);
    s.unsupportedSource      = unsupportedSource_.load(std::memory_order_relaxed);
    s.encodeOversizedDropped = encodeOversized_.load(std::memory_order_relaxed);
    s.encodeFailed           = encodeFailed_.load(std::memory_order_relaxed);
    s.invalidatedDropped     = invalidatedDropped_.load(std::memory_order_relaxed);
    s.offered                = offered_.load(std::memory_order_relaxed);
    s.refused                = refused_.load(std::memory_order_relaxed);
    s.decoded                = decoded_.load(std::memory_order_relaxed);
    s.encodeRetries          = encodeRetries_.load(std::memory_order_relaxed);
    s.geometryMismatch       = geometryMismatch_.load(std::memory_order_relaxed);
    s.decodeWarnings         = decodeWarnings_.load(std::memory_order_relaxed);
    s.bytesOut               = bytesOut_.load(std::memory_order_relaxed);

    std::lock_guard<std::mutex> sl(statsMu_);
    s.lastJpegBytes    = lastJpegBytes_;
    s.maxJpegBytes     = maxJpegBytes_;
    s.lastQuality      = lastQuality_;
    s.lastSourceWidth  = lastSourceW_;
    s.lastSourceHeight = lastSourceH_;
    s.lastScaleDenom   = lastScaleDenom_;
    s.lastViewport     = lastViewport_;
    s.outputSlotsFree  = pool_ ? pool_->freeSlots() : 0;
    s.timingSamples    = totalRing_.count;
    s.decode           = decodeRing_.summary(lastDecodeMs_);
    s.resize           = resizeRing_.summary(lastResizeMs_);
    s.encode           = encodeRing_.summary(lastEncodeMs_);
    s.total            = totalRing_.summary(lastTotalMs_);
    return s;
}

} // namespace dashcam::preview
