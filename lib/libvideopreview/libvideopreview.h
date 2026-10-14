/**
 * @file libvideopreview.h
 * @brief Recording-camera MJPEG → small letterboxed JPEG preview frames for the
 *        ES3C28P display (peripherals/es3c28p/VIDEO_STREAMING_PLAN.md).
 *
 * The recorder's compressed tap hands every camera JPEG to submitJpeg() on a
 * GStreamer streaming thread.  submitJpeg() never waits: it rejects what the
 * preview cannot use, rate-limits to the preview FPS, and copies the frame
 * into one of two preallocated input slots (a newer frame replaces the one
 * still waiting).  One worker thread then
 *   1. reads the JPEG header and fits the source into the sink's canvas
 *      (computeLetterbox(): aspect preserved, never stretched or cropped);
 *   2. decodes with libjpeg DCT scaling (chooseDctScaleDenom(): 1080p → 1/4,
 *      480×270 — the plan's measured 3× saving over a full decode);
 *   3. area-averages to the viewport (exact integer box filter, deterministic);
 *   4. encodes a baseline 4:2:0 JPEG straight into a free slot of a fixed
 *      two-slot output pool, retrying ONCE at the minimum quality when the
 *      result would exceed the byte cap;
 *   5. offers the slot to the IPreviewSink as a move-only EncodedFrameLease.
 * Every stage that cannot proceed drops the frame and counts why (stats()).
 *
 * Memory is fixed at start(): 2 × PreviewConfig::maxSourceJpegBytes input
 * slots, 2 × maxJpegBytes output slots, and the converter's row buffers and
 * resampling tables (sized for kMaxSourceDimension / kMaxCanvas*).  libjpeg's
 * own per-image working memory (proportional to the image width) is allocated
 * and freed by libjpeg inside each decode/encode.
 *
 * A preview failure never affects recording: start() returning false just
 * means "no preview", and nothing here touches the recording pipeline.
 *
 * Typical usage:
 * @code
 *   dashcam::preview::VideoPreview preview;
 *   dashcam::preview::PreviewConfig pc;          // from <DisplayPreview>
 *   if (!preview.start(pc, displayLink, log)) { ... preview off, keep recording ... }
 *   // GStreamer streaming thread (recorder tap):
 *   preview.submitJpeg(data, len, 1920, 1080, captureMonotonicUs);
 *   // Camera recovery:
 *   preview.invalidateSource();
 *   // Shutdown: stop the tap first, then
 *   preview.stop();
 * @endcode
 *
 * Thread safety: submitJpeg() may be called from any number of threads
 * (contention returns Busy).  invalidateSource() and stats() from any thread.
 * start()/stop() are serialised against each other; stop() must not be called
 * from inside an IPreviewSink callback (it joins the thread making the call).
 */

#ifndef LIBVIDEOPREVIEW_H
#define LIBVIDEOPREVIEW_H

#include "liblog.h"

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace dashcam::preview {

// ─── hard limits ──────────────────────────────────────────────────────────────

constexpr uint32_t kMaxSourceDimension = 8192;     ///< Larger JPEG headers are rejected.
constexpr uint16_t kMaxCanvasWidth     = 640;      ///< Largest display canvas served
constexpr uint16_t kMaxCanvasHeight    = 480;      ///< (the ES3C28P is 320×240).
constexpr uint32_t kMinTargetFps       = 1;        ///< PreviewConfig::targetFps range —
constexpr uint32_t kMaxTargetFps       = 15;       ///< the plan's supported 1–15 fps.
constexpr size_t   kMinJpegBytes       = 1024;     ///< PreviewConfig::maxJpegBytes range.
constexpr size_t   kMaxJpegBytesLimit  = 1u << 20;
constexpr size_t   kMinSourceJpegBytes = 4096;     ///< PreviewConfig::maxSourceJpegBytes range.
constexpr size_t   kMaxSourceJpegBytesLimit = 16u << 20;

// ─── configuration and geometry ───────────────────────────────────────────────

/**
 * @brief Preview converter settings; the application fills them from the
 *        optional <DisplayPreview> config section.  start() rejects values
 *        outside the documented ranges.
 */
struct PreviewConfig {
    uint32_t targetFps          = 10;       ///< Rate ceiling by capture timestamp, 1–15.
    int      jpegQuality        = 65;       ///< First encode attempt, 1–100.
    int      minimumJpegQuality = 45;       ///< The single retry after an oversized encode, 1–jpegQuality.
    size_t   maxJpegBytes       = 65536;    ///< Output cap and output-slot size (1 KiB – 1 MiB).
    size_t   maxSourceJpegBytes = 1u << 20; ///< Input-slot size; larger camera frames are Oversized.
};

/// Where the image sits on the display canvas.  All zero = invalid.
struct Viewport {
    uint16_t x = 0, y = 0, width = 0, height = 0;
    bool valid() const { return width != 0 && height != 0; }
    bool operator==(const Viewport& o) const {
        return x == o.x && y == o.y && width == o.width && height == o.height;
    }
};

/**
 * @brief Fit a srcW×srcH image into a canvasW×canvasH canvas.
 *
 * Preserves the aspect ratio (letterbox or pillarbox; never stretched or
 * cropped beyond rounding), uses even width and height (nearest even size,
 * ties toward the smaller, at least 2), and centres the result.  16:9 into
 * 320×240 → {0,30,320,180}; 4:3 → {0,0,320,240}; 9:16 → {93,0,134,240}.
 * Pure.  Returns an all-zero Viewport for a zero source size or a canvas
 * smaller than 2×2.
 */
Viewport computeLetterbox(uint32_t srcW, uint32_t srcH, uint16_t canvasW, uint16_t canvasH);

/**
 * @brief The libjpeg DCT scale for decoding srcW×srcH toward a viewW×viewH
 *        viewport: the largest denominator in {1, 2, 4, 8} (scale_num = 1)
 *        whose output (ceil(src / d) per axis) is still at least the viewport
 *        in both dimensions.  1 when even the full size is smaller (upscale).
 */
uint32_t chooseDctScaleDenom(uint32_t srcW, uint32_t srcH, uint32_t viewW, uint32_t viewH);

// ─── output frames ────────────────────────────────────────────────────────────

class EncodedFramePool;

/**
 * @brief Move-only handle on one slot of an EncodedFramePool, holding one
 *        encoded preview JPEG.
 *
 * Released exactly once: by release() or by the destructor, whichever comes
 * first (a moved-from lease is empty and releases nothing).  The lease keeps
 * its pool alive, so a sink may hold a lease past VideoPreview::stop() or its
 * destruction.  A lease is not itself thread-safe: hand it between threads
 * with the sink's own synchronisation.
 */
class EncodedFrameLease {
public:
    EncodedFrameLease() = default;
    ~EncodedFrameLease();
    EncodedFrameLease(EncodedFrameLease&& other) noexcept;
    EncodedFrameLease& operator=(EncodedFrameLease&& other) noexcept;
    EncodedFrameLease(const EncodedFrameLease&)            = delete;
    EncodedFrameLease& operator=(const EncodedFrameLease&) = delete;

    bool valid() const noexcept { return pool_ != nullptr; }
    explicit operator bool() const noexcept { return valid(); }

    const uint8_t* data() const noexcept;                         ///< size() JPEG bytes; nullptr when empty.
    size_t   size() const noexcept { return size_; }
    uint16_t width() const noexcept { return width_; }             ///< Encoded image = viewport size.
    uint16_t height() const noexcept { return height_; }
    uint64_t captureMonotonicUs() const noexcept { return captureUs_; } ///< As passed to submitJpeg().
    uint64_t sourceSequence() const noexcept { return sequence_; } ///< Per-start() count of accepted submissions, from 1.
    int      quality() const noexcept { return quality_; }         ///< Encoder quality actually used.

    /// Return the slot to its pool now.  Idempotent.
    void release() noexcept;

    // Producer side (VideoPreview's worker, or a test building leases by hand).
    uint8_t* writableData() noexcept;                              ///< capacity() bytes; nullptr when empty.
    size_t   capacity() const noexcept;
    /// Describe the bytes written into writableData().  False (nothing changed)
    /// when the lease is empty or @p size exceeds capacity().
    bool setFrame(size_t size, uint16_t width, uint16_t height, uint64_t captureMonotonicUs,
                  uint64_t sourceSequence, int quality) noexcept;

private:
    friend class EncodedFramePool;
    EncodedFrameLease(std::shared_ptr<EncodedFramePool> pool, uint32_t slot) noexcept;

    std::shared_ptr<EncodedFramePool> pool_;
    uint32_t slot_      = 0;
    size_t   size_      = 0;
    uint16_t width_     = 0;
    uint16_t height_    = 0;
    uint64_t captureUs_ = 0;
    uint64_t sequence_  = 0;
    int      quality_   = 0;
};

/**
 * @brief A fixed set of equally sized byte slots, allocated once.
 *
 * tryAcquire() never blocks and never allocates; slots go back to the pool
 * when their lease is released, from any thread.
 */
class EncodedFramePool : public std::enable_shared_from_this<EncodedFramePool> {
public:
    /// nullptr for a zero size, more than 64 slots, or allocation failure.
    static std::shared_ptr<EncodedFramePool> create(size_t slotCount, size_t slotBytes);

    /// A free slot as an (unfilled) lease, or an empty lease when none is free.
    EncodedFrameLease tryAcquire();

    size_t slotCount() const noexcept { return slotCount_; }
    size_t slotBytes() const noexcept { return slotBytes_; }
    size_t freeSlots() const noexcept;

    EncodedFramePool(const EncodedFramePool&)            = delete;
    EncodedFramePool& operator=(const EncodedFramePool&) = delete;

private:
    friend class EncodedFrameLease;
    EncodedFramePool(size_t slotCount, size_t slotBytes);
    void     releaseSlot(uint32_t slot) noexcept;
    uint8_t* slotData(uint32_t slot) noexcept { return storage_.get() + slot * slotBytes_; }

    size_t                                slotCount_;
    size_t                                slotBytes_;
    std::unique_ptr<uint8_t[]>            storage_;
    std::unique_ptr<std::atomic<bool>[]>  inUse_;
};

// ─── the display side ─────────────────────────────────────────────────────────

/// What the display currently wants.  Inactive = hidden tab, calibration, or
/// no negotiated session: no conversion work is done at all.
struct PreviewDemand {
    bool     active       = false;
    uint16_t canvasW      = 0;   ///< Display canvas (e.g. 320×240), up to kMaxCanvas*.
    uint16_t canvasH      = 0;
    size_t   maxJpegBytes = 0;   ///< Device frame limit; 0 = only PreviewConfig::maxJpegBytes applies.
};

/**
 * @brief Receiver of finished preview frames (libes3display's link, or a test).
 *
 * demand() is called on every submitJpeg() (on the camera's streaming thread)
 * and once per converted frame: it must be cheap, thread-safe and
 * non-blocking, and must not call into VideoPreview at all (submitJpeg()
 * holds its slot lock around the call).  offerFrame() is called on the
 * preview worker: it must not block either (queue the lease, or refuse — a
 * blocked offer delays stop()), and it takes ownership ONLY when it returns
 * true — on false the caller still owns the lease and returns its slot to the
 * pool.  It may call stats() or invalidateSource(), never start()/stop().
 */
class IPreviewSink {
public:
    virtual ~IPreviewSink() = default;
    virtual PreviewDemand demand() const = 0;
    virtual bool offerFrame(EncodedFrameLease&& frame) = 0;
};

// ─── the converter (worker-side, also usable on its own) ─────────────────────

enum class ConvertStatus {
    Ok,
    InvalidArgument,     ///< Null/empty buffer, bad canvas or quality.
    DecodeFailed,        ///< Corrupt or truncated JPEG (libjpeg error or data warning).
    UnsupportedSource,   ///< CMYK/YCCK, multi-scan (progressive), or > kMaxSourceDimension.
    EncodeOversized,     ///< Still above the cap after the retry at minimumQuality.
    EncodeFailed,        ///< libjpeg compressor error.
};
const char* convertStatusName(ConvertStatus s);

/// One conversion request.  @p out receives the encoded JPEG; at most
/// @p outCapacity bytes are written (that is also the size cap).
struct ConvertRequest {
    const uint8_t* jpeg            = nullptr;
    size_t         size            = 0;
    uint16_t       canvasW         = 0;
    uint16_t       canvasH         = 0;
    int            quality         = 65;
    int            minimumQuality  = 45;
    uint8_t*       out             = nullptr;
    size_t         outCapacity     = 0;
};

struct ConvertResult {
    ConvertStatus status       = ConvertStatus::InvalidArgument;
    size_t        jpegBytes    = 0;
    uint16_t      width        = 0;      ///< Encoded size = viewport size.
    uint16_t      height       = 0;
    Viewport      viewport;               ///< Position on the canvas.
    int           quality      = 0;      ///< Quality of the encode that fitted.
    bool          retried      = false;  ///< The first encode was over the cap.
    uint32_t      sourceWidth  = 0;      ///< From the JPEG header (0 if unreadable).
    uint32_t      sourceHeight = 0;
    uint32_t      scaleDenom   = 0;      ///< libjpeg DCT scale used (1/2/4/8).
    uint32_t      warnings     = 0;      ///< Benign libjpeg warnings (metadata quirks).
    double        decodeMs     = 0.0;
    double        resizeMs     = 0.0;
    double        encodeMs     = 0.0;    ///< Both attempts when retried.
    char          detail[200]  = {};     ///< libjpeg's message for a failure, else empty.
};

/**
 * @brief JPEG → letterboxed, DCT-scaled, area-averaged → baseline JPEG.
 *
 * Single-threaded (VideoPreview's worker owns one).  All buffers are
 * allocated by init(); convert() allocates nothing of its own.  libjpeg
 * errors never abort or exit: they come back as a status.
 */
class FrameConverter {
public:
    FrameConverter();
    ~FrameConverter();
    FrameConverter(const FrameConverter&)            = delete;
    FrameConverter& operator=(const FrameConverter&) = delete;

    /// Allocate the buffers.  False (and unusable) on allocation failure.
    bool init();
    bool ready() const { return ready_; }

    ConvertResult convert(const ConvertRequest& req);

    /// Forget the cached resampling geometry (the next frame rebuilds it).
    void invalidateGeometry();

    struct Impl;   ///< Buffers, resampling tables and the libjpeg glue (.cpp).

private:
    std::unique_ptr<Impl> impl_;
    bool                  ready_ = false;
};

// ─── the preview pipeline ─────────────────────────────────────────────────────

enum class SubmitResult {
    Accepted,          ///< Copied into a free input slot.
    ReplacedWaiting,   ///< Copied over an older frame still waiting for the worker.
    Inactive,          ///< Not started / stopped, or the sink's demand is inactive.
    RateLimited,       ///< Too soon after the last accepted frame (targetFps).
    Oversized,         ///< Larger than PreviewConfig::maxSourceJpegBytes.
    Unsupported,       ///< Not a JPEG (no SOI), zero size/dimensions, or an unusable canvas.
    Busy,              ///< Another thread held the slot lock: dropped, never waited.
};
constexpr size_t kSubmitResultCount = 7;
const char* submitResultName(SubmitResult r);

/// Last value and p95 over the most recent kTimingWindow frames.
struct PreviewTiming {
    double lastMs = 0.0;
    double p95Ms  = 0.0;
};

/// Snapshot of the preview counters (cumulative since start()).
struct PreviewStats {
    std::array<uint64_t, kSubmitResultCount> submit{};  ///< Indexed by SubmitResult.
    uint64_t submitted = 0;            ///< submitJpeg() calls (= sum of submit[]).
    // Accepted frames end in exactly one of: worker pickup (processed), or
    // discarded while waiting (invalidateSource()/stop()).  ReplacedWaiting
    // frames take over the waiting frame they displaced.
    uint64_t processed            = 0;
    uint64_t discardedWaiting     = 0;
    // Worker outcomes (sum = processed).
    uint64_t inactiveDropped      = 0; ///< Demand went inactive/invalid before conversion.
    uint64_t noOutputSlot         = 0; ///< Both output slots still held by the sink.
    uint64_t decodeFailed         = 0;
    uint64_t unsupportedSource    = 0; ///< CMYK, progressive/multi-scan, too large.
    uint64_t encodeOversizedDropped = 0;
    uint64_t encodeFailed         = 0;
    uint64_t invalidatedDropped   = 0; ///< Source invalidated while the frame was in progress.
    uint64_t offered              = 0; ///< offerFrame() calls ...
    uint64_t refused              = 0; ///< ... of which returned false.
    // Details.
    uint64_t decoded              = 0; ///< Successful decodes.
    uint64_t encodeRetries        = 0;
    uint64_t geometryMismatch     = 0; ///< JPEG header size ≠ size passed to submitJpeg().
    uint64_t decodeWarnings       = 0; ///< Benign libjpeg warnings on frames that were kept.
    uint64_t bytesOut             = 0; ///< JPEG bytes of frames the sink took.
    size_t   lastJpegBytes        = 0; ///< Size of the last frame that fitted the cap.
    size_t   maxJpegBytes         = 0; ///< Largest frame that fitted the cap.
    int      lastQuality          = 0;
    uint32_t lastSourceWidth      = 0;
    uint32_t lastSourceHeight     = 0;
    uint32_t lastScaleDenom       = 0;
    Viewport lastViewport;
    size_t   outputSlotsFree      = 0; ///< Free output slots now (0 when stopped).
    uint32_t timingSamples        = 0; ///< Frames in the timing window.
    PreviewTiming decode, resize, encode, total;  ///< total = worker pickup → offer returned.
};

constexpr size_t kTimingWindow = 128;

/**
 * @brief Rate-limited, never-blocking MJPEG → preview JPEG pipeline (one worker).
 *
 * See the file comment.  All methods are safe on a never-started or stopped
 * instance; the destructor stop()s.
 */
class VideoPreview {
public:
    VideoPreview();
    ~VideoPreview();
    VideoPreview(const VideoPreview&)            = delete;
    VideoPreview& operator=(const VideoPreview&) = delete;

    /**
     * @brief Validate @p cfg, allocate every buffer and start the worker.
     * @return false = preview off (bad config, already running, allocation or
     *         thread failure — logged); recording is unaffected.  @p sink must
     *         outlive stop().
     */
    bool start(const PreviewConfig& cfg, IPreviewSink& sink, dashcam::log::LogCallback log = {});

    /**
     * @brief Offer one camera JPEG.  Never blocks or waits on the worker, the
     *        sink, USB or the display; the bytes are copied before returning.
     * @param captureMonotonicUs  Monotonic capture time; drives the rate limit
     *        and is passed through to the lease.  A step backwards of more
     *        than one frame interval restarts the rate schedule.
     */
    SubmitResult submitJpeg(const uint8_t* bytes, size_t size, uint32_t srcW, uint32_t srcH,
                            uint64_t captureMonotonicUs);

    /// Camera recovery: drop the waiting frame, discard the one in progress
    /// before it is offered, forget cached geometry and the rate schedule.
    void invalidateSource();

    PreviewStats stats() const;

    /// Stop accepting, join the worker, free the buffers.  Idempotent; safe
    /// after a failed start().  Leases the sink still holds stay valid.
    void stop();

    bool running() const { return accepting_.load(std::memory_order_acquire); }

private:
    struct InputSlot {
        std::vector<uint8_t> buf;
        size_t   size       = 0;
        uint32_t srcW       = 0;
        uint32_t srcH       = 0;
        uint64_t captureUs  = 0;
        uint64_t sequence   = 0;
        uint64_t generation = 0;
    };
    struct TimingRing {
        std::array<float, kTimingWindow> v{};
        uint32_t count = 0;
        uint32_t head  = 0;
        void push(double ms);
        PreviewTiming summary(double lastMs) const;
    };

    void         workerLoop();
    void         processFrame(const InputSlot& in);
    void         flushPendingLogs();
    void         noteOnce(uint32_t bit);  ///< inMu_ held by the caller.
    SubmitResult count(SubmitResult r);
    void         releaseResources();      ///< Worker joined; lifeMu_ held.

    // Lifecycle (lifeMu_ serialises start/stop).
    std::mutex                        lifeMu_;
    PreviewConfig                     cfg_;
    IPreviewSink*                     sink_ = nullptr;
    dashcam::log::LogCallback         log_;
    std::thread                       worker_;
    std::shared_ptr<EncodedFramePool> pool_;
    std::unique_ptr<FrameConverter>   conv_;      ///< Worker-owned while running.
    std::atomic<bool>                 accepting_{false};

    // Input handoff (inMu_; submitters only try_lock it).
    std::mutex              inMu_;
    std::condition_variable inCv_;
    InputSlot               in_[2];
    int                     waiting_    = -1;
    int                     inProgress_ = -1;
    bool                    stopping_   = false;
    bool                    running_    = false;
    bool                    haveRate_   = false;
    uint64_t                nextDueUs_  = 0;
    uint64_t                lastAcceptedUs_ = 0;
    uint64_t                nextSequence_   = 1;
    std::atomic<uint64_t>   generation_{0};
    std::atomic<bool>       geometryInvalid_{false};
    std::atomic<uint32_t>   pendingLog_{0};   ///< Submit-side first-time events for the worker to log.
    std::atomic<uint32_t>   loggedMask_{0};   ///< Kinds already logged this start().

    // Counters (relaxed atomics; snapshot by stats()).
    std::array<std::atomic<uint64_t>, kSubmitResultCount> submit_{};
    std::atomic<uint64_t> processed_{0}, discardedWaiting_{0}, inactiveDropped_{0}, noOutputSlot_{0},
        decodeFailed_{0}, unsupportedSource_{0}, encodeOversized_{0}, encodeFailed_{0},
        invalidatedDropped_{0}, offered_{0}, refused_{0}, decoded_{0}, encodeRetries_{0},
        geometryMismatch_{0}, decodeWarnings_{0}, bytesOut_{0};

    // Worker-written details (statsMu_).
    mutable std::mutex statsMu_;
    size_t     lastJpegBytes_ = 0, maxJpegBytes_ = 0;
    int        lastQuality_ = 0;
    uint32_t   lastSourceW_ = 0, lastSourceH_ = 0, lastScaleDenom_ = 0;
    Viewport   lastViewport_;
    double     lastDecodeMs_ = 0, lastResizeMs_ = 0, lastEncodeMs_ = 0, lastTotalMs_ = 0;
    TimingRing decodeRing_, resizeRing_, encodeRing_, totalRing_;
};

} // namespace dashcam::preview

#endif // LIBVIDEOPREVIEW_H
