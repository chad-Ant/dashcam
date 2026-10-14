/**
 * @file libvideopreview_convert.cpp
 * @brief FrameConverter — libjpeg DCT-scaled decode, integer area-average
 *        resample, baseline 4:2:0 encode — and the pure geometry helpers.
 *
 * libjpeg reports errors through error_exit(), which here longjmp()s back to a
 * guard instead of exiting.  To keep that well-defined in C++:
 *   - the *Body() functions a longjmp can unwind hold no object with a
 *     destructor (only PODs and raw pointers into preallocated buffers);
 *   - the *Guarded() functions that call setjmp() keep their libjpeg state in a
 *     context struct owned by the caller, never in locals modified after setjmp.
 * Data-corruption warnings (truncated file, bad Huffman code, premature marker)
 * abort the decode the same way: a preview frame is published only when the
 * whole image decoded cleanly.  Metadata warnings (unknown JFIF revision, odd
 * Adobe/ICC markers, extraneous bytes between segments) are counted and kept.
 */

#include "libvideopreview.h"

#include <algorithm>
#include <csetjmp>
#include <cstdio>     // jpeglib.h needs FILE and size_t declared first
#include <cstring>
#include <ctime>

#include <jpeglib.h>
#include <jerror.h>     // after jpeglib.h: its message list depends on JPEG_LIB_VERSION

namespace dashcam::preview {

// ─── geometry ─────────────────────────────────────────────────────────────────

namespace {

/// Nearest even integer to num/den (den > 0), ties toward the smaller one.
uint64_t nearestEven(uint64_t num, uint64_t den) {
    return ((num + den - 1) / (2 * den)) * 2;
}

} // namespace

Viewport computeLetterbox(uint32_t srcW, uint32_t srcH, uint16_t canvasW, uint16_t canvasH) {
    const uint64_t availW = canvasW & ~1u;   // even sizes only
    const uint64_t availH = canvasH & ~1u;
    if (srcW == 0 || srcH == 0 || availW < 2 || availH < 2) return {};

    uint64_t w, h;
    if (uint64_t(srcW) * availH >= uint64_t(srcH) * availW) {
        // At least as wide as the canvas: full width, bars above and below.
        w = availW;
        h = std::clamp<uint64_t>(nearestEven(availW * srcH, srcW), 2, availH);
    } else {
        // Taller: full height, bars left and right.
        h = availH;
        w = std::clamp<uint64_t>(nearestEven(availH * srcW, srcH), 2, availW);
    }
    Viewport v;
    v.width  = static_cast<uint16_t>(w);
    v.height = static_cast<uint16_t>(h);
    v.x      = static_cast<uint16_t>((canvasW - w) / 2);
    v.y      = static_cast<uint16_t>((canvasH - h) / 2);
    return v;
}

uint32_t chooseDctScaleDenom(uint32_t srcW, uint32_t srcH, uint32_t viewW, uint32_t viewH) {
    for (uint32_t d : {8u, 4u, 2u}) {
        // libjpeg's scaled output size is ceil(image size / d).
        if ((uint64_t(srcW) + d - 1) / d >= viewW && (uint64_t(srcH) + d - 1) / d >= viewH) return d;
    }
    return 1;
}

const char* convertStatusName(ConvertStatus s) {
    switch (s) {
    case ConvertStatus::Ok:                return "ok";
    case ConvertStatus::InvalidArgument:   return "invalid argument";
    case ConvertStatus::DecodeFailed:      return "decode failed";
    case ConvertStatus::UnsupportedSource: return "unsupported source";
    case ConvertStatus::EncodeOversized:   return "encode oversized";
    case ConvertStatus::EncodeFailed:      return "encode failed";
    }
    return "?";
}

// ─── resampling tables and working buffers ────────────────────────────────────

namespace {

constexpr uint32_t kWeightOne = 16384;   ///< Box-filter weights are fixed point /2^14.

struct AxisSeg {
    uint16_t src;     ///< Source column/row.
    uint16_t weight;  ///< Share of the output pixel, /kWeightOne.
    uint16_t emit;    ///< 1 on the last segment of an output pixel.
};

constexpr size_t kAxisCapacity =
    kMaxSourceDimension + (kMaxCanvasWidth > kMaxCanvasHeight ? kMaxCanvasWidth : kMaxCanvasHeight) + 1;

/**
 * Area-average (box filter) plan for resampling srcLen samples to dstLen.
 * On a common integer axis, source sample i spans [i*dstLen, (i+1)*dstLen) and
 * output o spans [o*srcLen, (o+1)*srcLen); every overlap becomes one segment
 * weighted by its share of the output.  The last segment of each output takes
 * the rounding remainder, so each output's weights sum to exactly kWeightOne.
 * Works for down- and upscaling.  Returns the segment count (at most
 * srcLen + dstLen), or 0 when @p cap is too small.
 */
size_t buildAxis(uint32_t srcLen, uint32_t dstLen, AxisSeg* seg, size_t cap) {
    size_t   n = 0;
    uint32_t pos = 0, i = 0, o = 0, wsum = 0;
    uint32_t srcEnd = dstLen, outEnd = srcLen;
    while (o < dstLen) {
        if (n >= cap) return 0;
        const uint32_t next = std::min(srcEnd, outEnd);
        const uint32_t w    = static_cast<uint32_t>(uint64_t(next - pos) * kWeightOne / srcLen);
        seg[n++] = {static_cast<uint16_t>(i), static_cast<uint16_t>(w), 0};
        wsum += w;
        pos = next;
        if (pos == outEnd) {
            seg[n - 1].weight = static_cast<uint16_t>(seg[n - 1].weight + (kWeightOne - wsum));
            seg[n - 1].emit   = 1;
            wsum = 0;
            ++o;
            outEnd += srcLen;
        }
        if (pos == srcEnd) {
            ++i;
            srcEnd += dstLen;
        }
    }
    return n;
}

uint64_t monoNs() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_t(ts.tv_sec) * 1000000000ull + uint64_t(ts.tv_nsec);
}

} // namespace

struct FrameConverter::Impl {
    std::vector<uint8_t>  row;     ///< One decoded RGB row (up to kMaxSourceDimension wide).
    std::vector<uint16_t> hrow;    ///< That row resampled horizontally, ×256.
    std::vector<uint32_t> acc;     ///< Vertical accumulator row.
    std::vector<uint8_t>  image;   ///< The resampled RGB viewport image.
    std::vector<AxisSeg>  hseg, vseg;

    struct Geometry {
        uint32_t scaledW = 0, scaledH = 0, viewW = 0, viewH = 0;
        size_t   hCount = 0, vCount = 0;
    } geo;

    /// Resampling tables for a scaled decode → viewport (cached until it changes).
    bool prepare(uint32_t scaledW, uint32_t scaledH, uint32_t viewW, uint32_t viewH) {
        if (geo.hCount && geo.scaledW == scaledW && geo.scaledH == scaledH && geo.viewW == viewW &&
            geo.viewH == viewH)
            return true;
        geo = Geometry{};
        if (scaledW == 0 || scaledH == 0 || scaledW > kMaxSourceDimension ||
            scaledH > kMaxSourceDimension || viewW == 0 || viewH == 0 || viewW > kMaxCanvasWidth ||
            viewH > kMaxCanvasHeight)
            return false;
        const size_t h = buildAxis(scaledW, viewW, hseg.data(), hseg.size());
        const size_t v = buildAxis(scaledH, viewH, vseg.data(), vseg.size());
        if (h == 0 || v == 0) return false;
        geo = {scaledW, scaledH, viewW, viewH, h, v};
        return true;
    }

    /**
     * Fold decoded source row @p y (in row) into the output: resample it
     * horizontally into hrow, add it to every output row it overlaps, and
     * write out each output row whose last source row this is.
     */
    void resampleRow(uint32_t y, size_t& vcur, uint32_t& outRow) {
        const uint8_t* src = row.data();
        uint16_t*      h   = hrow.data();
        uint32_t a0 = 0, a1 = 0, a2 = 0;
        for (size_t s = 0; s < geo.hCount; ++s) {
            const AxisSeg  g = hseg[s];
            const uint8_t* p = src + size_t(g.src) * 3;
            a0 += uint32_t(p[0]) * g.weight;
            a1 += uint32_t(p[1]) * g.weight;
            a2 += uint32_t(p[2]) * g.weight;
            if (g.emit) {   // ≤ 255·2^14 → ×256 fixed point ≤ 65280
                h[0] = static_cast<uint16_t>((a0 + 32) >> 6);
                h[1] = static_cast<uint16_t>((a1 + 32) >> 6);
                h[2] = static_cast<uint16_t>((a2 + 32) >> 6);
                h += 3;
                a0 = a1 = a2 = 0;
            }
        }
        const size_t n = size_t(geo.viewW) * 3;
        uint32_t*    a = acc.data();
        while (vcur < geo.vCount && vseg[vcur].src == y) {
            const AxisSeg g = vseg[vcur++];
            const uint32_t w = g.weight;
            if (!g.emit) {
                for (size_t k = 0; k < n; ++k) a[k] += uint32_t(hrow[k]) * w;
                continue;
            }
            // Last contribution: ≤ 65280·2^14 + 2^21 < 2^32, then /2^22 → 0..255.
            if (outRow < geo.viewH) {
                uint8_t* out = image.data() + size_t(outRow) * n;
                for (size_t k = 0; k < n; ++k) {
                    out[k] = static_cast<uint8_t>((a[k] + uint32_t(hrow[k]) * w + (1u << 21)) >> 22);
                    a[k]   = 0;
                }
            }
            ++outRow;
        }
    }
};

FrameConverter::FrameConverter() = default;
FrameConverter::~FrameConverter() = default;

bool FrameConverter::init() {
    if (ready_) return true;
    try {
        auto impl = std::make_unique<Impl>();
        impl->row.assign(size_t(kMaxSourceDimension) * 3, 0);
        impl->hrow.assign(size_t(kMaxCanvasWidth) * 3, 0);
        impl->acc.assign(size_t(kMaxCanvasWidth) * 3, 0);
        impl->image.assign(size_t(kMaxCanvasWidth) * kMaxCanvasHeight * 3, 0);
        impl->hseg.assign(kAxisCapacity, AxisSeg{0, 0, 0});
        impl->vseg.assign(kAxisCapacity, AxisSeg{0, 0, 0});
        impl_ = std::move(impl);
    } catch (const std::bad_alloc&) {
        impl_.reset();
        return false;
    }
    ready_ = true;
    return true;
}

void FrameConverter::invalidateGeometry() {
    if (impl_) impl_->geo = Impl::Geometry{};
}

// ─── libjpeg glue ─────────────────────────────────────────────────────────────

namespace {

/// libjpeg error manager; `pub` must stay first (libjpeg casts cinfo->err).
struct JpegError {
    jpeg_error_mgr pub;
    std::jmp_buf   jump;
    bool           overflow;          ///< Encoder output cap reached.
    uint32_t       benignWarnings;
    char           message[JMSG_LENGTH_MAX];
};

void jpegErrorExit(j_common_ptr cinfo) {
    JpegError* e = reinterpret_cast<JpegError*>(cinfo->err);
    if (!e->overflow) (*cinfo->err->format_message)(cinfo, e->message);
    std::longjmp(e->jump, 1);
}

/// Warnings that mean the image data itself is damaged abort the decode.
void jpegEmitMessage(j_common_ptr cinfo, int msgLevel) {
    if (msgLevel >= 0) return;   // trace messages
    JpegError* e = reinterpret_cast<JpegError*>(cinfo->err);
    switch (cinfo->err->msg_code) {
    case JWRN_HIT_MARKER:
    case JWRN_HUFF_BAD_CODE:
    case JWRN_JPEG_EOF:
    case JWRN_MUST_RESYNC:
    case JWRN_NOT_SEQUENTIAL:
    case JWRN_BOGUS_PROGRESSION:
    case JWRN_TOO_MUCH_DATA:
#if JPEG_LIB_VERSION >= 70
    case JWRN_ARITH_BAD_CODE:
#endif
        (*cinfo->err->format_message)(cinfo, e->message);
        std::longjmp(e->jump, 1);
    default:
        ++e->benignWarnings;
        ++cinfo->err->num_warnings;
        break;
    }
}

void installErrorManager(JpegError& e, j_common_ptr cinfo) {
    cinfo->err             = jpeg_std_error(&e.pub);
    e.pub.error_exit       = jpegErrorExit;
    e.pub.emit_message     = jpegEmitMessage;
    e.overflow             = false;
    e.benignWarnings       = 0;
    e.message[0]           = '\0';
}

// ── decode + resample ──

struct DecodeCtx {
    jpeg_decompress_struct cinfo;
    JpegError              err;
    FrameConverter::Impl*  impl;
    const uint8_t*         jpeg;
    size_t                 size;
    uint16_t               canvasW, canvasH;
    // Out.
    uint32_t               srcW, srcH, denom;
    Viewport               viewport;
    double                 decodeMs, resizeMs;
    char                   reason[96];   ///< Why a source is unsupported.
};

/// Runs between setjmp and longjmp: PODs only.
ConvertStatus decodeBody(DecodeCtx* c) {
    jpeg_decompress_struct* ci = &c->cinfo;
    const uint64_t t0 = monoNs();
    jpeg_mem_src(ci, c->jpeg, static_cast<unsigned long>(c->size));
    jpeg_read_header(ci, TRUE);
    c->srcW = ci->image_width;
    c->srcH = ci->image_height;
    if (c->srcW > kMaxSourceDimension || c->srcH > kMaxSourceDimension) {
        std::snprintf(c->reason, sizeof(c->reason), "%ux%u is larger than %ux%u", c->srcW, c->srcH,
                      kMaxSourceDimension, kMaxSourceDimension);
        return ConvertStatus::UnsupportedSource;
    }
    if (ci->jpeg_color_space != JCS_YCbCr && ci->jpeg_color_space != JCS_GRAYSCALE &&
        ci->jpeg_color_space != JCS_RGB) {
        std::snprintf(c->reason, sizeof(c->reason), "colour space %d (CMYK/YCCK?) is not supported",
                      int(ci->jpeg_color_space));
        return ConvertStatus::UnsupportedSource;
    }
    if (jpeg_has_multiple_scans(ci)) {
        // Progressive / multi-scan: libjpeg would buffer the whole image's
        // coefficients (megabytes per frame).  Camera MJPEG is single-scan.
        std::snprintf(c->reason, sizeof(c->reason), "progressive/multi-scan JPEG is not supported");
        return ConvertStatus::UnsupportedSource;
    }
    c->viewport = computeLetterbox(c->srcW, c->srcH, c->canvasW, c->canvasH);
    if (!c->viewport.valid()) return ConvertStatus::InvalidArgument;
    c->denom          = chooseDctScaleDenom(c->srcW, c->srcH, c->viewport.width, c->viewport.height);
    ci->scale_num     = 1;
    ci->scale_denom   = c->denom;
    ci->out_color_space = JCS_RGB;   // grayscale sources are expanded by libjpeg
    jpeg_calc_output_dimensions(ci);
    if (ci->output_components != 3 ||
        !c->impl->prepare(ci->output_width, ci->output_height, c->viewport.width, c->viewport.height))
        return ConvertStatus::InvalidArgument;

    jpeg_start_decompress(ci);
    FrameConverter::Impl* im = c->impl;
    std::memset(im->acc.data(), 0, size_t(im->geo.viewW) * 3 * sizeof(uint32_t));
    JSAMPROW rowPtr   = im->row.data();
    size_t   vcur     = 0;
    uint32_t outRow   = 0;
    uint64_t resizeNs = 0;
    while (ci->output_scanline < ci->output_height) {
        const JDIMENSION y = ci->output_scanline;
        if (jpeg_read_scanlines(ci, &rowPtr, 1) != 1) return ConvertStatus::DecodeFailed;
        const uint64_t r0 = monoNs();
        im->resampleRow(y, vcur, outRow);
        resizeNs += monoNs() - r0;
    }
    jpeg_finish_decompress(ci);
    if (outRow != im->geo.viewH) return ConvertStatus::DecodeFailed;
    const uint64_t totalNs = monoNs() - t0;
    c->decodeMs = double(totalNs - resizeNs) / 1e6;
    c->resizeMs = double(resizeNs) / 1e6;
    return ConvertStatus::Ok;
}

ConvertStatus decodeGuarded(DecodeCtx* c) {
    installErrorManager(c->err, reinterpret_cast<j_common_ptr>(&c->cinfo));
    if (setjmp(c->err.jump)) {
        jpeg_destroy_decompress(&c->cinfo);
        return ConvertStatus::DecodeFailed;
    }
    jpeg_create_decompress(&c->cinfo);
    const ConvertStatus s = decodeBody(c);
    jpeg_destroy_decompress(&c->cinfo);
    return s;
}

// ── encode ──

/// Destination manager writing into a fixed buffer; running out of room
/// aborts the encode (the frame would exceed the cap anyway).  libjpeg asks
/// for more room as soon as the buffer is full, even if nothing follows, so
/// the first request switches to a small spill area: a frame of exactly `cap`
/// bytes leaves it untouched and still fits.
struct FixedDest {
    jpeg_destination_mgr pub;   // first: libjpeg casts cinfo->dest
    uint8_t*             buf;
    size_t               cap;
    bool                 spilled;
    uint8_t              spill[64];
};

void destInit(j_compress_ptr ci) {
    FixedDest* d = reinterpret_cast<FixedDest*>(ci->dest);
    d->pub.next_output_byte = d->buf;
    d->pub.free_in_buffer   = d->cap;
}

boolean destEmpty(j_compress_ptr ci) {
    FixedDest* d = reinterpret_cast<FixedDest*>(ci->dest);
    if (!d->spilled) {
        d->spilled              = true;
        d->pub.next_output_byte = d->spill;
        d->pub.free_in_buffer   = sizeof(d->spill);
        return TRUE;
    }
    reinterpret_cast<JpegError*>(ci->err)->overflow = true;
    (*ci->err->error_exit)(reinterpret_cast<j_common_ptr>(ci));
    return FALSE;   // not reached
}

void destTerm(j_compress_ptr) {}

struct EncodeCtx {
    jpeg_compress_struct cinfo;
    JpegError            err;
    FixedDest            dest;
    const uint8_t*       rgb;
    uint32_t             width, height;
    int                  quality;
    size_t               outBytes;
};

/// Runs between setjmp and longjmp: PODs only.
ConvertStatus encodeBody(EncodeCtx* c) {
    jpeg_compress_struct* ci = &c->cinfo;
    c->dest.pub.init_destination    = destInit;
    c->dest.pub.empty_output_buffer = destEmpty;
    c->dest.pub.term_destination    = destTerm;
    ci->dest             = &c->dest.pub;
    ci->image_width      = c->width;
    ci->image_height     = c->height;
    ci->input_components = 3;
    ci->in_color_space   = JCS_RGB;
    jpeg_set_defaults(ci);
    jpeg_set_quality(ci, c->quality, TRUE);   // baseline tables
    ci->comp_info[0].h_samp_factor = 2;       // 4:2:0 (the libjpeg default, made explicit)
    ci->comp_info[0].v_samp_factor = 2;
    ci->comp_info[1].h_samp_factor = ci->comp_info[1].v_samp_factor = 1;
    ci->comp_info[2].h_samp_factor = ci->comp_info[2].v_samp_factor = 1;
    ci->optimize_coding = FALSE;
    ci->dct_method      = JDCT_ISLOW;
    jpeg_start_compress(ci, TRUE);
    const size_t stride = size_t(c->width) * 3;
    JSAMPROW rows[16];
    while (ci->next_scanline < ci->image_height) {
        const JDIMENSION y0 = ci->next_scanline;
        const JDIMENSION n  = std::min<JDIMENSION>(16, ci->image_height - y0);
        for (JDIMENSION k = 0; k < n; ++k)
            rows[k] = const_cast<JSAMPROW>(c->rgb + size_t(y0 + k) * stride);   // read only
        jpeg_write_scanlines(ci, rows, n);
    }
    jpeg_finish_compress(ci);
    if (c->dest.spilled) {
        if (c->dest.pub.free_in_buffer < sizeof(c->dest.spill)) return ConvertStatus::EncodeOversized;
        c->outBytes = c->dest.cap;   // exactly full
    } else {
        c->outBytes = c->dest.cap - c->dest.pub.free_in_buffer;
    }
    return ConvertStatus::Ok;
}

ConvertStatus encodeGuarded(EncodeCtx* c) {
    installErrorManager(c->err, reinterpret_cast<j_common_ptr>(&c->cinfo));
    if (setjmp(c->err.jump)) {
        jpeg_destroy_compress(&c->cinfo);
        return c->err.overflow ? ConvertStatus::EncodeOversized : ConvertStatus::EncodeFailed;
    }
    jpeg_create_compress(&c->cinfo);
    const ConvertStatus s = encodeBody(c);
    jpeg_destroy_compress(&c->cinfo);
    return s;
}

} // namespace

// ─── FrameConverter::convert ──────────────────────────────────────────────────

ConvertResult FrameConverter::convert(const ConvertRequest& req) {
    ConvertResult r;
    if (!ready_ || req.jpeg == nullptr || req.size == 0 || req.out == nullptr || req.outCapacity == 0 ||
        req.canvasW < 2 || req.canvasH < 2 || req.canvasW > kMaxCanvasWidth ||
        req.canvasH > kMaxCanvasHeight || req.quality < 1 || req.quality > 100 ||
        req.minimumQuality < 1 || req.minimumQuality > req.quality) {
        r.status = ConvertStatus::InvalidArgument;
        return r;
    }

    DecodeCtx dc{};
    dc.impl    = impl_.get();
    dc.jpeg    = req.jpeg;
    dc.size    = req.size;
    dc.canvasW = req.canvasW;
    dc.canvasH = req.canvasH;
    r.status       = decodeGuarded(&dc);
    r.sourceWidth  = dc.srcW;
    r.sourceHeight = dc.srcH;
    r.warnings     = dc.err.benignWarnings;
    if (r.status != ConvertStatus::Ok) {
        if (dc.err.message[0] != '\0')
            std::snprintf(r.detail, sizeof(r.detail), "%s", dc.err.message);
        else if (dc.reason[0] != '\0')
            std::snprintf(r.detail, sizeof(r.detail), "%s", dc.reason);
        return r;
    }
    r.viewport   = dc.viewport;
    r.width      = dc.viewport.width;
    r.height     = dc.viewport.height;
    r.scaleDenom = dc.denom;
    r.decodeMs   = dc.decodeMs;
    r.resizeMs   = dc.resizeMs;

    // Encode at the configured quality; once more at the floor if too big.
    const uint64_t e0 = monoNs();
    int quality = req.quality;
    for (int attempt = 0; attempt < 2; ++attempt) {
        EncodeCtx ec{};
        ec.rgb      = impl_->image.data();
        ec.width    = r.width;
        ec.height   = r.height;
        ec.quality  = quality;
        ec.dest.buf = req.out;
        ec.dest.cap = req.outCapacity;
        r.status  = encodeGuarded(&ec);
        r.quality = quality;
        if (r.status == ConvertStatus::Ok) {
            r.jpegBytes = ec.outBytes;
            break;
        }
        if (r.status == ConvertStatus::EncodeFailed) {
            std::snprintf(r.detail, sizeof(r.detail), "%s", ec.err.message);
            break;
        }
        if (quality == req.minimumQuality) break;   // nothing lower to try
        quality   = req.minimumQuality;
        r.retried = true;
    }
    r.encodeMs = double(monoNs() - e0) / 1e6;
    return r;
}

} // namespace dashcam::preview
