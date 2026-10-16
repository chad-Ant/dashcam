#pragma once
/**
 * @file Es3VideoProtocol.h
 * @brief Portable wire contract for the ES3C28P USB video preview (protocol v1,
 *        frozen 2026-10-13): E3 29 video records multiplexed with the E3 28
 *        control frames of Es3Protocol.h on one USB CDC byte stream.
 *
 * Keep this file AND Es3Protocol.h byte-identical in both locations:
 *   peripherals/es3c28p/lib/es3UsbLink/   (ESP32-S3 firmware)
 *   lib/libes3display/                    (Jetson host link)
 * peripherals/es3c28p/tests/run_tests.sh cmp-checks the copies, and
 * tests/video_golden_vectors.txt freezes the bytes for C++ and Python alike.
 *
 * Header-only; depends on nothing but <stdint.h>/<stddef.h>/<string.h> and
 * Es3Protocol.h. No Arduino, no POSIX, no exceptions, no heap: one copy
 * compiles unchanged (C++11 or later) with xtensa-esp32s3-elf-g++ and aarch64 g++.
 *
 * ── Record layout (all integers little-endian, encoded field by field) ───────
 *   E3 29 | version u8 | type u8 | payload_length u16 |
 *   session_token u32 | frame_id u32 | payload | crc16 u16
 * 16 bytes of overhead; payload_length excludes header and CRC. The CRC is
 * es3proto::crc16 (CRC-16/CCITT-FALSE) over version..last payload byte.
 *
 * ── Record types ─────────────────────────────────────────────────────────────
 *   host -> device                       device -> host
 *   01 START  w u16, h u16, jpeg u32,    81 RESULT     reason u8
 *             source_timestamp_us u64    82 PRESENTED  decode_us u32, blit_us u32
 *   02 CHUNK  offset u32, data[1..N]     83 CREDIT     cumulative_grant u32
 *   03 END    crc32 u32 (whole JPEG)     84 VISIBILITY demand u8 (0 active,
 *   04 ABORT  reason u8                                 1 hidden, 2 calibrating)
 * Map extension v2 (Es3MapProtocol.h, 2026-10-14) adds one host -> device type:
 *   10 MAP_POSE  16-byte pose of a panning map session (fields and rules there)
 * It is session-wide (frame_id 0) and is never credited or answered. Firmware
 * without the extension never opens a panning session, so it never sees one.
 * Every record carries the nonzero session token from the START_VIDEO reply.
 * frame_id is 1, 2, 3... within a session for types 01..04, 81 and 82, and
 * zero for CREDIT and VISIBILITY. N is the CHUNK data ceiling: kMaxChunkDataBytes
 * (4096) on the wire, the negotiated chunk_bytes (initially 1024) per session.
 *
 * ── Framing rules (the parser drops violations silently, no reply) ──────────
 * Known version and type, payload_length valid for the type, nonzero token,
 * the frame_id zero rule, and the CRC. A conforming encoder cannot produce a
 * violation, so a drop means line noise or a broken peer. A well-framed record
 * whose FIELDS are wrong (zero width, oversize JPEG, unknown reason...) is
 * delivered to the receiver; the device answers such a host record with RESULT
 * and the matching Reason — otherwise a START's credit would vanish and the
 * stream would stall.
 *
 * ── Frame flow and credits ───────────────────────────────────────────────────
 * START opens frame F and consumes one credit; CHUNKs must arrive at exactly
 * the next offset; END publishes only when the byte count and the CRC-32/ISO-HDLC
 * of the whole JPEG match. START width x height must equal the session viewport.
 * The device answers each START with exactly one RESULT: ReasonOk (assembled,
 * queued for decode) or a rejection; after ReasonOk either PRESENTED (last strip
 * blitted) or a second RESULT (DecodeFailed, Superseded, Hidden) follows. ABORT
 * discards F while it is being assembled (its RESULT echoes the ABORT reason);
 * CHUNK / END / ABORT for a frame not being assembled are dropped.
 *
 * CREDIT carries the CUMULATIVE number of frames the host may have started this
 * session (initial value in the START_VIDEO reply, at most max_credits): repeats
 * are harmless, the host keeps the latest value and never starts frame number
 * grant + 1. The device advances the grant once per credited START, when its
 * receive slot is released or when the START was rejected before taking one,
 * so no error path loses a credit; a NoCredit START was never credited. The
 * device sends VISIBILITY after every START_VIDEO reply and on each change. A
 * new HELLO, STOP_VIDEO, session timeout or USB loss invalidates the token and
 * all its grants; records carrying an old token are dropped.
 *
 * ── New control commands (E3 28 frames, payload <= 128 bytes) ──────────────
 *   07 CAPABILITIES  request empty; reply kCapabilitiesReplyBytes, see Capabilities
 *   08 START_VIDEO   request kStartVideoRequestBytes, see StartVideoRequest;
 *                    reply {session_token u32 != 0, initial_grant u32};
 *                    out-of-limit requests get Error BadPayload
 *   09 STOP_VIDEO    request {session_token u32}; reply empty
 * Error BadCommand (older firmware) or NotReady (no video memory) to
 * CAPABILITIES means "video unsupported": stay control-only, do not retry.
 * START_VIDEO during a session replaces it with a new token. STOP_VIDEO ends
 * the session only when the token matches; a stale token is a no-op that still
 * gets the empty reply, so shutdown and reconnect paths need no special case.
 *
 * Limits below are protocol ceilings. Each side advertises/configures its own
 * smaller limits; the firmware keeps compile-time ceilings regardless.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "Es3Protocol.h"

namespace es3video {

// ─── framing constants ────────────────────────────────────────────────────────

constexpr uint8_t  kMagic0         = 0xe3; ///< First sync byte, shared with control frames.
constexpr uint8_t  kControlMagic1  = 0x28; ///< Second sync byte of an es3proto control frame.
constexpr uint8_t  kMagic1         = 0x29; ///< Second sync byte of a video record.
constexpr uint8_t  kVersion        = 1;    ///< Video record / video protocol version.
constexpr size_t   kHeaderBytes    = 14;   ///< Magic(2) + version + type + length(2) + token(4) + frame(4).
constexpr size_t   kCrcBytes       = 2;
constexpr size_t   kOverheadBytes  = kHeaderBytes + kCrcBytes;
constexpr size_t   kChunkOffsetBytes = 4;
/** Hard ceiling of CHUNK data bytes; buffers are sized for it. */
constexpr size_t   kMaxChunkDataBytes = 4096;
/** Initial negotiated CHUNK data size (plan: start with 1024-byte chunks). */
constexpr size_t   kDefaultChunkDataBytes = 1024;
constexpr size_t   kMaxRecordPayload = kChunkOffsetBytes + kMaxChunkDataBytes;
constexpr size_t   kMaxRecordBytes = kOverheadBytes + kMaxRecordPayload;
/** Partial-record inter-byte timeout; the same as the control parser's. */
constexpr uint32_t kPartialTimeoutMs = es3proto::kPartialTimeoutMs;

// ─── protocol ceilings ────────────────────────────────────────────────────────

constexpr uint32_t kMaxJpegBytes   = 65536; ///< One compressed frame (= firmware JPEG slot).
constexpr uint8_t  kMaxCredits     = 2;     ///< Credited frames in flight.
constexpr uint8_t  kMaxTargetFps   = 30;    ///< START_VIDEO target_fps ceiling (config allows 1-15).
constexpr uint16_t kMaxTimeoutMs   = 10000; ///< Assembly / stale / placeholder ceiling.
constexpr uint16_t kCanvasWidth    = 320;   ///< ES3C28P panel, landscape.
constexpr uint16_t kCanvasHeight   = 240;

// ─── payload sizes ────────────────────────────────────────────────────────────

constexpr size_t kStartPayloadBytes      = 16;
constexpr size_t kEndPayloadBytes        = 4;
constexpr size_t kAbortPayloadBytes      = 1;
constexpr size_t kResultPayloadBytes     = 1;
constexpr size_t kPresentedPayloadBytes  = 8;
constexpr size_t kCreditPayloadBytes     = 4;
constexpr size_t kVisibilityPayloadBytes = 1;
constexpr size_t kMapPosePayloadBytes    = 16;  ///< Map extension v2; fields in Es3MapProtocol.h.
constexpr size_t kCapabilitiesRequestBytes = 0;
constexpr size_t kCapabilitiesReplyBytes = 18;
constexpr size_t kStartVideoRequestBytes = 23;
constexpr size_t kStartVideoReplyBytes   = 8;
constexpr size_t kStopVideoRequestBytes  = 4;
constexpr size_t kStopVideoReplyBytes    = 0;

static_assert(kMaxRecordBytes == 4116, "E3 29 record ceiling changed: update buffers and the plan");
static_assert(kMaxRecordPayload <= 0xffff, "payload_length is a u16");
static_assert(kCapabilitiesReplyBytes <= es3proto::kMaxPayload &&
              kStartVideoRequestBytes <= es3proto::kMaxPayload,
              "video control payloads must fit an E3 28 frame");
static_assert(kMaxRecordBytes >= es3proto::kMaxFrame, "MuxParser buffer must hold a control frame");

// ─── enums (frozen wire values) ───────────────────────────────────────────────

/** Record TYPE byte. High bit set = device -> host. */
enum RecordType : uint8_t {
    RecordStart      = 0x01,
    RecordChunk      = 0x02,
    RecordEnd        = 0x03,
    RecordAbort      = 0x04,
    RecordMapPose    = 0x10,  ///< Map extension v2 (Es3MapProtocol.h).
    RecordResult     = 0x81,
    RecordPresented  = 0x82,
    RecordCredit     = 0x83,
    RecordVisibility = 0x84
};

/**
 * Reason byte of ABORT (host) and RESULT (device); also what the decode helpers
 * return. ReasonOk in a RESULT means "assembled and accepted for decode".
 */
enum Reason : uint8_t {
    ReasonOk              = 0x00, ///< Accepted / valid.
    ReasonBadLength       = 0x01, ///< Payload length wrong for the type, zero jpeg_bytes, or data past jpeg_bytes.
    ReasonBadOffset       = 0x02, ///< CHUNK offset is not the exact next byte, or runs past kMaxJpegBytes.
    ReasonBadCrc          = 0x03, ///< END CRC-32 differs from the assembled JPEG.
    ReasonStaleSession    = 0x04, ///< Token zero or not the current session.
    ReasonNoCredit        = 0x05, ///< START beyond the cumulative grant.
    ReasonTooLarge        = 0x06, ///< jpeg_bytes / CHUNK data above the negotiated or protocol ceiling.
    ReasonBadDimensions   = 0x07, ///< Width/height zero or not the session viewport.
    ReasonAssemblyTimeout = 0x08, ///< END not received within assembly_timeout_ms of START.
    ReasonDecodeFailed    = 0x09, ///< The JPEG decoder rejected the image.
    ReasonSuperseded      = 0x0a, ///< A newer frame replaced it (device), or the host dropped it for newer footage.
    ReasonHidden          = 0x0b, ///< Video demand is not active (hidden or calibrating).
    ReasonBusy            = 0x0c, ///< No free receive slot despite a credit (device accounting fault).
    ReasonBadFrameId      = 0x0d, ///< frame_id zero, not increasing, or not the frame being assembled.
    ReasonMalformed       = 0x0e, ///< Wrong record type for the helper, or an unknown enum value in a field.
    ReasonCancelled       = 0x0f, ///< Sender stopped the frame: STOP_VIDEO, shutdown or reconnect.
    ReasonInternal        = 0x10  ///< Device-internal failure (memory, decoder state).
};
constexpr uint8_t kReasonCount = 0x11; ///< Reasons are 0..kReasonCount-1; anything else is Malformed.

/** VISIBILITY demand byte. */
enum Demand : uint8_t { DemandActive = 0, DemandHidden = 1, DemandCalibrating = 2 };

/** Codec id; Capabilities::codecMask bit n advertises codec n. */
enum Codec : uint8_t { CodecBaselineJpeg = 0 };
constexpr uint8_t kCodecMaskBaselineJpeg = 1u << CodecBaselineJpeg;

// ─── little-endian field access ───────────────────────────────────────────────

namespace detail {
inline void put16(uint8_t* p, uint16_t v) { p[0] = uint8_t(v); p[1] = uint8_t(v >> 8); }
inline void put32(uint8_t* p, uint32_t v) { put16(p, uint16_t(v)); put16(p + 2, uint16_t(v >> 16)); }
inline void put64(uint8_t* p, uint64_t v) { put32(p, uint32_t(v)); put32(p + 4, uint32_t(v >> 32)); }
inline uint16_t get16(const uint8_t* p) { return uint16_t(p[0] | (uint16_t(p[1]) << 8)); }
inline uint32_t get32(const uint8_t* p) { return get16(p) | (uint32_t(get16(p + 2)) << 16); }
inline uint64_t get64(const uint8_t* p) { return get32(p) | (uint64_t(get32(p + 4)) << 32); }
}  // namespace detail

// ─── CRC-32/ISO-HDLC (whole-JPEG checksum) ────────────────────────────────────

/**
 * @brief Continues a CRC-32/ISO-HDLC (reflected 0xEDB88320, init and xorout
 *        0xFFFFFFFF) over @p data.
 *
 * zlib.crc32-compatible chaining: start from 0 and pass each result back in, so
 * crc32Update(crc32Update(0, a), b) == crc32(a || b). A 16-entry nibble table
 * keeps the flash/RAM cost at 64 bytes while staying ~4x faster than bitwise.
 * Check value: crc32("123456789") == 0xCBF43926.
 */
inline uint32_t crc32Update(uint32_t crc, const uint8_t* data, size_t size)
{
    static const uint32_t kNibble[16] = {
        0x00000000u, 0x1db71064u, 0x3b6e20c8u, 0x26d930acu, 0x76dc4190u, 0x6b6b51f4u,
        0x4db26158u, 0x5005713cu, 0xedb88320u, 0xf00f9344u, 0xd6d6a3e8u, 0xcb61b38cu,
        0x9b64c2b0u, 0x86d3d2d4u, 0xa00ae278u, 0xbdbdf21cu
    };
    if (!data) return crc;
    crc = ~crc;
    for (size_t i = 0; i < size; ++i) {
        crc = kNibble[(crc ^ data[i]) & 0x0f] ^ (crc >> 4);
        crc = kNibble[(crc ^ (data[i] >> 4)) & 0x0f] ^ (crc >> 4);
    }
    return ~crc;
}

/** @brief One-shot CRC-32/ISO-HDLC. */
inline uint32_t crc32(const uint8_t* data, size_t size) { return crc32Update(0, data, size); }

// ─── type predicates ──────────────────────────────────────────────────────────

inline bool isHostToDevice(uint8_t type)
{
    return (type >= RecordStart && type <= RecordAbort) || type == RecordMapPose;
}
inline bool isDeviceToHost(uint8_t type) { return type >= RecordResult && type <= RecordVisibility; }
inline bool knownType(uint8_t type) { return isHostToDevice(type) || isDeviceToHost(type); }
/** CREDIT, VISIBILITY and MAP_POSE are session-wide and carry frame_id 0; all others name a frame. */
inline bool namesFrame(uint8_t type)
{
    return type != RecordCredit && type != RecordVisibility && type != RecordMapPose;
}

/**
 * @brief Framing check of the fields every header carries.
 * @param maxChunkData CHUNK data ceiling (<= kMaxChunkDataBytes).
 * @return ReasonOk, ReasonMalformed (unknown type), ReasonBadLength,
 *         ReasonTooLarge (CHUNK above @p maxChunkData), ReasonStaleSession
 *         (zero token) or ReasonBadFrameId.
 */
inline Reason checkFraming(uint8_t type, size_t payloadLength, uint32_t sessionToken,
                           uint32_t frameId, size_t maxChunkData = kMaxChunkDataBytes)
{
    size_t expected = 0;
    switch (type) {
    case RecordStart:      expected = kStartPayloadBytes; break;
    case RecordEnd:        expected = kEndPayloadBytes; break;
    case RecordAbort:      expected = kAbortPayloadBytes; break;
    case RecordResult:     expected = kResultPayloadBytes; break;
    case RecordPresented:  expected = kPresentedPayloadBytes; break;
    case RecordCredit:     expected = kCreditPayloadBytes; break;
    case RecordVisibility: expected = kVisibilityPayloadBytes; break;
    case RecordMapPose:    expected = kMapPosePayloadBytes; break;
    case RecordChunk:
        if (maxChunkData > kMaxChunkDataBytes) maxChunkData = kMaxChunkDataBytes;
        if (payloadLength <= kChunkOffsetBytes) return ReasonBadLength;
        if (payloadLength > kChunkOffsetBytes + maxChunkData) return ReasonTooLarge;
        expected = payloadLength;
        break;
    default:
        return ReasonMalformed;
    }
    if (payloadLength != expected) return ReasonBadLength;
    if (sessionToken == 0) return ReasonStaleSession;
    if (namesFrame(type) != (frameId != 0)) return ReasonBadFrameId;
    return ReasonOk;
}

// ─── records ──────────────────────────────────────────────────────────────────

/**
 * A decoded record header plus a view of its payload. The payload pointer
 * borrows the buffer it was decoded from (MuxParser: valid until its next
 * feed()/expire()/reset()).
 */
struct Record {
    uint8_t type = 0;
    uint16_t payloadLength = 0;
    uint32_t sessionToken = 0;
    uint32_t frameId = 0;
    const uint8_t* payload = nullptr;
};

struct StartInfo {
    uint16_t width = 0;
    uint16_t height = 0;
    uint32_t jpegBytes = 0;
    uint64_t sourceTimestampUs = 0; ///< Host monotonic capture time; opaque to the device.
};

struct ChunkInfo {
    uint32_t offset = 0;
    const uint8_t* data = nullptr; ///< Borrowed from the Record payload.
    size_t size = 0;
};

struct PresentedInfo {
    uint32_t decodeUs = 0;
    uint32_t blitUs = 0;
};

/** @brief Wraps header fields and a payload view into a Record (no copy, no checks). */
inline Record makeRecord(uint8_t type, uint32_t sessionToken, uint32_t frameId,
                         const uint8_t* payload, size_t payloadLength)
{
    Record r;
    r.type = type;
    r.payloadLength = uint16_t(payloadLength > 0xffff ? 0xffff : payloadLength);
    r.sessionToken = sessionToken;
    r.frameId = frameId;
    r.payload = payload;
    return r;
}

namespace detail {
/** Framing check of a Record view against the helper's expected type. */
inline Reason checkTyped(const Record& r, uint8_t type)
{
    if (r.type != type) return ReasonMalformed;
    if (r.payloadLength && !r.payload) return ReasonBadLength;
    return checkFraming(r.type, r.payloadLength, r.sessionToken, r.frameId);
}
}  // namespace detail

// Typed decoders: return ReasonOk on success. When the framing is valid, @p out
// is filled even if a field check then fails, so the caller can log it.

/** @brief START fields; BadDimensions for zero width/height, BadLength/TooLarge for jpeg_bytes. */
inline Reason decodeStart(const Record& r, StartInfo& out)
{
    const Reason framing = detail::checkTyped(r, RecordStart);
    if (framing != ReasonOk) return framing;
    out.width = detail::get16(r.payload);
    out.height = detail::get16(r.payload + 2);
    out.jpegBytes = detail::get32(r.payload + 4);
    out.sourceTimestampUs = detail::get64(r.payload + 8);
    if (!out.width || !out.height) return ReasonBadDimensions;
    if (!out.jpegBytes) return ReasonBadLength;
    if (out.jpegBytes > kMaxJpegBytes) return ReasonTooLarge;
    return ReasonOk;
}

/** @brief CHUNK offset and data view; BadOffset when it would run past kMaxJpegBytes. */
inline Reason decodeChunk(const Record& r, ChunkInfo& out)
{
    const Reason framing = detail::checkTyped(r, RecordChunk);
    if (framing != ReasonOk) return framing;
    out.offset = detail::get32(r.payload);
    out.data = r.payload + kChunkOffsetBytes;
    out.size = r.payloadLength - kChunkOffsetBytes;
    if (out.offset > kMaxJpegBytes - out.size) return ReasonBadOffset;
    return ReasonOk;
}

/** @brief END: CRC-32/ISO-HDLC of the complete JPEG. */
inline Reason decodeEnd(const Record& r, uint32_t& jpegCrc32)
{
    const Reason framing = detail::checkTyped(r, RecordEnd);
    if (framing != ReasonOk) return framing;
    jpegCrc32 = detail::get32(r.payload);
    return ReasonOk;
}

namespace detail {
inline Reason decodeReason(const Record& r, uint8_t type, Reason& reason)
{
    const Reason framing = checkTyped(r, type);
    if (framing != ReasonOk) return framing;
    reason = static_cast<Reason>(r.payload[0]);
    return r.payload[0] < kReasonCount ? ReasonOk : ReasonMalformed;
}
}  // namespace detail

/** @brief ABORT requires a non-Ok reason (host -> device). */
inline Reason decodeAbort(const Record& r, Reason& reason)
{
    const Reason parsed = detail::decodeReason(r, RecordAbort, reason);
    return parsed == ReasonOk && reason == ReasonOk ? ReasonMalformed : parsed;
}

/** @brief RESULT reason (device -> host). */
inline Reason decodeResult(const Record& r, Reason& reason)
{
    return detail::decodeReason(r, RecordResult, reason);
}

/** @brief PRESENTED decode/blit durations for r.frameId. */
inline Reason decodePresented(const Record& r, PresentedInfo& out)
{
    const Reason framing = detail::checkTyped(r, RecordPresented);
    if (framing != ReasonOk) return framing;
    out.decodeUs = detail::get32(r.payload);
    out.blitUs = detail::get32(r.payload + 4);
    return ReasonOk;
}

/** @brief CREDIT: cumulative frames the host may have started this session. */
inline Reason decodeCredit(const Record& r, uint32_t& cumulativeGrant)
{
    const Reason framing = detail::checkTyped(r, RecordCredit);
    if (framing != ReasonOk) return framing;
    cumulativeGrant = detail::get32(r.payload);
    return ReasonOk;
}

/** @brief VISIBILITY demand; Malformed for values above DemandCalibrating. */
inline Reason decodeVisibility(const Record& r, Demand& demand)
{
    const Reason framing = detail::checkTyped(r, RecordVisibility);
    if (framing != ReasonOk) return framing;
    demand = static_cast<Demand>(r.payload[0]);
    return r.payload[0] <= DemandCalibrating ? ReasonOk : ReasonMalformed;
}

/**
 * @brief Full validation of one record: framing plus every field check of the
 *        typed decoder. encodeRecord() refuses anything this rejects, so
 *        whatever a conforming peer sends passes the receiver's decoders.
 */
inline Reason checkRecord(uint8_t type, uint32_t sessionToken, uint32_t frameId,
                          const uint8_t* payload, size_t payloadLength)
{
    if (payloadLength > kMaxRecordPayload) return type == RecordChunk ? ReasonTooLarge : ReasonBadLength;
    const Record r = makeRecord(type, sessionToken, frameId, payload, payloadLength);
    switch (type) {
    case RecordStart:      { StartInfo v; return decodeStart(r, v); }
    case RecordChunk:      { ChunkInfo v; return decodeChunk(r, v); }
    case RecordEnd:        { uint32_t v = 0; return decodeEnd(r, v); }
    case RecordAbort:      { Reason v = ReasonOk; return decodeAbort(r, v); }
    case RecordResult:     { Reason v = ReasonOk; return decodeResult(r, v); }
    case RecordPresented:  { PresentedInfo v; return decodePresented(r, v); }
    case RecordCredit:     { uint32_t v = 0; return decodeCredit(r, v); }
    case RecordVisibility: { Demand v = DemandActive; return decodeVisibility(r, v); }
    // Field rules live with the map extension (es3map::decodeMapPose).
    case RecordMapPose:    return checkFraming(type, payloadLength, sessionToken, frameId);
    default:               return ReasonMalformed;
    }
}

namespace detail {
/**
 * Writes header + head + body + CRC into @p out, validating the result with
 * checkRecord(). Two payload parts let CHUNK data go straight from the JPEG
 * buffer to the wire without an intermediate copy.
 */
inline size_t encodeParts(uint8_t type, uint32_t sessionToken, uint32_t frameId,
                          const uint8_t* head, size_t headLength,
                          const uint8_t* body, size_t bodyLength,
                          uint8_t* out, size_t capacity)
{
    if (!out || (headLength && !head) || (bodyLength && !body)) return 0;
    if (headLength > kMaxRecordPayload || bodyLength > kMaxRecordPayload - headLength) return 0;
    const size_t length = headLength + bodyLength;
    const size_t total = kOverheadBytes + length;
    if (capacity < total) return 0;
    out[0] = kMagic0;
    out[1] = kMagic1;
    out[2] = kVersion;
    out[3] = type;
    put16(out + 4, uint16_t(length));
    put32(out + 6, sessionToken);
    put32(out + 10, frameId);
    if (headLength) memcpy(out + kHeaderBytes, head, headLength);
    if (bodyLength) memcpy(out + kHeaderBytes + headLength, body, bodyLength);
    if (checkRecord(type, sessionToken, frameId, out + kHeaderBytes, length) != ReasonOk) return 0;
    put16(out + kHeaderBytes + length, es3proto::crc16(out + 2, kHeaderBytes - 2 + length));
    return total;
}
}  // namespace detail

/**
 * @brief Serialises one E3 29 record into @p out.
 * @param type          RecordType value.
 * @param sessionToken  Nonzero session token.
 * @param frameId       Nonzero, except 0 for CREDIT / VISIBILITY.
 * @param payload       Payload bytes; may be NULL only when @p payloadLength is 0.
 * @param payloadLength Payload bytes (CHUNK: offset + data).
 * @param out           Destination; contents unspecified when 0 is returned.
 * @param capacity      Capacity of @p out; needs kOverheadBytes + payloadLength.
 * @return Bytes written, or 0 when anything fails checkRecord() or does not fit.
 */
inline size_t encodeRecord(uint8_t type, uint32_t sessionToken, uint32_t frameId,
                           const uint8_t* payload, size_t payloadLength,
                           uint8_t* out, size_t capacity)
{
    return detail::encodeParts(type, sessionToken, frameId, payload, payloadLength,
                               nullptr, 0, out, capacity);
}

inline size_t encodeStart(uint32_t sessionToken, uint32_t frameId, const StartInfo& info,
                          uint8_t* out, size_t capacity)
{
    uint8_t p[kStartPayloadBytes];
    detail::put16(p, info.width);
    detail::put16(p + 2, info.height);
    detail::put32(p + 4, info.jpegBytes);
    detail::put64(p + 8, info.sourceTimestampUs);
    return encodeRecord(RecordStart, sessionToken, frameId, p, sizeof(p), out, capacity);
}

/** @brief CHUNK with @p size (1..kMaxChunkDataBytes) data bytes copied once, into @p out. */
inline size_t encodeChunk(uint32_t sessionToken, uint32_t frameId, uint32_t offset,
                          const uint8_t* data, size_t size, uint8_t* out, size_t capacity)
{
    uint8_t p[kChunkOffsetBytes];
    detail::put32(p, offset);
    return detail::encodeParts(RecordChunk, sessionToken, frameId, p, sizeof(p),
                               data, size, out, capacity);
}

inline size_t encodeEnd(uint32_t sessionToken, uint32_t frameId, uint32_t jpegCrc32,
                        uint8_t* out, size_t capacity)
{
    uint8_t p[kEndPayloadBytes];
    detail::put32(p, jpegCrc32);
    return encodeRecord(RecordEnd, sessionToken, frameId, p, sizeof(p), out, capacity);
}

inline size_t encodeAbort(uint32_t sessionToken, uint32_t frameId, Reason reason,
                          uint8_t* out, size_t capacity)
{
    const uint8_t p = reason;
    return encodeRecord(RecordAbort, sessionToken, frameId, &p, 1, out, capacity);
}

inline size_t encodeResult(uint32_t sessionToken, uint32_t frameId, Reason reason,
                           uint8_t* out, size_t capacity)
{
    const uint8_t p = reason;
    return encodeRecord(RecordResult, sessionToken, frameId, &p, 1, out, capacity);
}

inline size_t encodePresented(uint32_t sessionToken, uint32_t frameId, const PresentedInfo& info,
                              uint8_t* out, size_t capacity)
{
    uint8_t p[kPresentedPayloadBytes];
    detail::put32(p, info.decodeUs);
    detail::put32(p + 4, info.blitUs);
    return encodeRecord(RecordPresented, sessionToken, frameId, p, sizeof(p), out, capacity);
}

inline size_t encodeCredit(uint32_t sessionToken, uint32_t cumulativeGrant,
                           uint8_t* out, size_t capacity)
{
    uint8_t p[kCreditPayloadBytes];
    detail::put32(p, cumulativeGrant);
    return encodeRecord(RecordCredit, sessionToken, 0, p, sizeof(p), out, capacity);
}

inline size_t encodeVisibility(uint32_t sessionToken, Demand demand, uint8_t* out, size_t capacity)
{
    const uint8_t p = demand;
    return encodeRecord(RecordVisibility, sessionToken, 0, &p, 1, out, capacity);
}

/**
 * @brief Decodes one complete record held in @p wire[0..size).
 *
 * Applies exactly the MuxParser framing rules (magic, version, framing fields,
 * CRC, and @p size == the declared record size). Field checks are left to the
 * typed decoders, as on the receive path.
 * @return ReasonOk, ReasonMalformed (magic/version/type), ReasonBadLength,
 *         ReasonTooLarge, ReasonStaleSession, ReasonBadFrameId or ReasonBadCrc.
 */
inline Reason decodeRecord(const uint8_t* wire, size_t size, Record& out,
                           size_t maxChunkData = kMaxChunkDataBytes)
{
    if (!wire || size < kOverheadBytes) return ReasonBadLength;
    if (wire[0] != kMagic0 || wire[1] != kMagic1 || wire[2] != kVersion) return ReasonMalformed;
    const size_t length = detail::get16(wire + 4);
    const Reason framing = checkFraming(wire[3], length, detail::get32(wire + 6),
                                        detail::get32(wire + 10), maxChunkData);
    if (framing != ReasonOk) return framing;
    if (size != kOverheadBytes + length) return ReasonBadLength;
    if (es3proto::crc16(wire + 2, kHeaderBytes - 2 + length) != detail::get16(wire + kHeaderBytes + length)) {
        return ReasonBadCrc;
    }
    out = makeRecord(wire[3], detail::get32(wire + 6), detail::get32(wire + 10),
                     wire + kHeaderBytes, length);
    return ReasonOk;
}

// ─── control payloads for CAPABILITIES / START_VIDEO / STOP_VIDEO ─────────────

/**
 * CAPABILITIES reply (18 bytes): video_protocol_version u8, codec_mask u8,
 * canvas_width u16, canvas_height u16, max_width u16, max_height u16,
 * max_jpeg_bytes u32, max_chunk_bytes u16, max_credits u8, flags u8.
 * flags is reserved in v1: senders write 0, receivers ignore it.
 */
struct Capabilities {
    uint8_t videoProtocolVersion = kVersion;
    uint8_t codecMask = kCodecMaskBaselineJpeg;
    uint16_t canvasWidth = 0;
    uint16_t canvasHeight = 0;
    uint16_t maxWidth = 0;     ///< Largest decodable image (= largest viewport).
    uint16_t maxHeight = 0;
    uint32_t maxJpegBytes = 0;
    uint16_t maxChunkBytes = 0; ///< CHUNK data bytes per record.
    uint8_t maxCredits = 0;
    uint8_t flags = 0;
};

/**
 * START_VIDEO request (23 bytes): video_protocol_version u8, codec u8,
 * viewport_x u16, viewport_y u16, viewport_w u16, viewport_h u16,
 * max_jpeg_bytes u32, chunk_bytes u16, target_fps u8, assembly_timeout_ms u16,
 * stale_after_ms u16, placeholder_after_ms u16. The viewport is where the
 * image is drawn on the canvas; every START must be exactly viewport_w x viewport_h.
 */
struct StartVideoRequest {
    uint8_t videoProtocolVersion = kVersion;
    uint8_t codec = CodecBaselineJpeg;
    uint16_t viewportX = 0;
    uint16_t viewportY = 0;
    uint16_t viewportW = 0;
    uint16_t viewportH = 0;
    uint32_t maxJpegBytes = 0;
    uint16_t chunkBytes = 0;
    uint8_t targetFps = 0;
    uint16_t assemblyTimeoutMs = 0;
    uint16_t staleAfterMs = 0;
    uint16_t placeholderAfterMs = 0;
};

/** START_VIDEO reply (8 bytes): session_token u32 (nonzero), initial_grant u32 (cumulative). */
struct StartVideoReply {
    uint32_t sessionToken = 0;
    uint32_t initialGrant = 0;
};

/** @brief True when @p c is a usable v1 capabilities set (flags ignored). */
inline bool validCapabilities(const Capabilities& c)
{
    return c.videoProtocolVersion == kVersion &&
           (c.codecMask & kCodecMaskBaselineJpeg) &&
           c.canvasWidth && c.canvasHeight &&
           c.maxWidth && c.maxWidth <= c.canvasWidth &&
           c.maxHeight && c.maxHeight <= c.canvasHeight &&
           c.maxJpegBytes && c.maxJpegBytes <= kMaxJpegBytes &&
           c.maxChunkBytes && c.maxChunkBytes <= kMaxChunkDataBytes &&
           c.maxCredits && c.maxCredits <= kMaxCredits;
}

/** @return kCapabilitiesReplyBytes, or 0 if @p c is invalid or @p out too small. */
inline size_t encodeCapabilitiesReply(const Capabilities& c, uint8_t* out, size_t capacity)
{
    if (!out || capacity < kCapabilitiesReplyBytes || !validCapabilities(c)) return 0;
    out[0] = c.videoProtocolVersion;
    out[1] = c.codecMask;
    detail::put16(out + 2, c.canvasWidth);
    detail::put16(out + 4, c.canvasHeight);
    detail::put16(out + 6, c.maxWidth);
    detail::put16(out + 8, c.maxHeight);
    detail::put32(out + 10, c.maxJpegBytes);
    detail::put16(out + 14, c.maxChunkBytes);
    out[16] = c.maxCredits;
    out[17] = c.flags;
    return kCapabilitiesReplyBytes;
}

/**
 * @brief Parses a CAPABILITIES reply. @p out is filled whenever the length is
 *        right, so an unsupported peer (e.g. a newer version) can be logged.
 * @return True only for a valid v1 capabilities set.
 */
inline bool decodeCapabilitiesReply(const uint8_t* payload, size_t length, Capabilities& out)
{
    if (!payload || length != kCapabilitiesReplyBytes) return false;
    out.videoProtocolVersion = payload[0];
    out.codecMask = payload[1];
    out.canvasWidth = detail::get16(payload + 2);
    out.canvasHeight = detail::get16(payload + 4);
    out.maxWidth = detail::get16(payload + 6);
    out.maxHeight = detail::get16(payload + 8);
    out.maxJpegBytes = detail::get32(payload + 10);
    out.maxChunkBytes = detail::get16(payload + 14);
    out.maxCredits = payload[16];
    out.flags = payload[17];
    return validCapabilities(out);
}

/**
 * @brief True when @p r fits the device limits @p device: same version, an
 *        advertised codec, a nonzero viewport inside the canvas and within
 *        max_width x max_height, JPEG/chunk limits within the device's, 1..30 fps,
 *        nonzero timeouts <= kMaxTimeoutMs and placeholder >= stale.
 */
inline bool validStartVideo(const StartVideoRequest& r, const Capabilities& device)
{
    if (!validCapabilities(device)) return false;
    if (r.videoProtocolVersion != kVersion || r.codec >= 8 ||
        !(device.codecMask & (1u << r.codec))) {
        return false;
    }
    if (!r.viewportW || !r.viewportH || r.viewportW > device.maxWidth || r.viewportH > device.maxHeight ||
        uint32_t(r.viewportX) + r.viewportW > device.canvasWidth ||
        uint32_t(r.viewportY) + r.viewportH > device.canvasHeight) {
        return false;
    }
    if (!r.maxJpegBytes || r.maxJpegBytes > device.maxJpegBytes ||
        !r.chunkBytes || r.chunkBytes > device.maxChunkBytes ||
        !r.targetFps || r.targetFps > kMaxTargetFps) {
        return false;
    }
    return r.assemblyTimeoutMs && r.assemblyTimeoutMs <= kMaxTimeoutMs &&
           r.staleAfterMs && r.staleAfterMs <= kMaxTimeoutMs &&
           r.placeholderAfterMs >= r.staleAfterMs && r.placeholderAfterMs <= kMaxTimeoutMs;
}

/** @return kStartVideoRequestBytes, or 0 if !validStartVideo(r, device) or @p out too small. */
inline size_t encodeStartVideoRequest(const StartVideoRequest& r, const Capabilities& device,
                                      uint8_t* out, size_t capacity)
{
    if (!out || capacity < kStartVideoRequestBytes || !validStartVideo(r, device)) return 0;
    out[0] = r.videoProtocolVersion;
    out[1] = r.codec;
    detail::put16(out + 2, r.viewportX);
    detail::put16(out + 4, r.viewportY);
    detail::put16(out + 6, r.viewportW);
    detail::put16(out + 8, r.viewportH);
    detail::put32(out + 10, r.maxJpegBytes);
    detail::put16(out + 14, r.chunkBytes);
    out[16] = r.targetFps;
    detail::put16(out + 17, r.assemblyTimeoutMs);
    detail::put16(out + 19, r.staleAfterMs);
    detail::put16(out + 21, r.placeholderAfterMs);
    return kStartVideoRequestBytes;
}

/**
 * @brief Device side: parses a START_VIDEO request against its own limits.
 *        False means reply Error BadPayload and keep the previous state.
 */
inline bool decodeStartVideoRequest(const uint8_t* payload, size_t length, const Capabilities& device,
                                    StartVideoRequest& out)
{
    if (!payload || length != kStartVideoRequestBytes) return false;
    out.videoProtocolVersion = payload[0];
    out.codec = payload[1];
    out.viewportX = detail::get16(payload + 2);
    out.viewportY = detail::get16(payload + 4);
    out.viewportW = detail::get16(payload + 6);
    out.viewportH = detail::get16(payload + 8);
    out.maxJpegBytes = detail::get32(payload + 10);
    out.chunkBytes = detail::get16(payload + 14);
    out.targetFps = payload[16];
    out.assemblyTimeoutMs = detail::get16(payload + 17);
    out.staleAfterMs = detail::get16(payload + 19);
    out.placeholderAfterMs = detail::get16(payload + 21);
    return validStartVideo(out, device);
}

/** @brief Valid reply: nonzero token, initial grant 1..kMaxCredits. */
inline bool validStartVideoReply(const StartVideoReply& r)
{
    return r.sessionToken != 0 && r.initialGrant >= 1 && r.initialGrant <= kMaxCredits;
}

inline size_t encodeStartVideoReply(const StartVideoReply& r, uint8_t* out, size_t capacity)
{
    if (!out || capacity < kStartVideoReplyBytes || !validStartVideoReply(r)) return 0;
    detail::put32(out, r.sessionToken);
    detail::put32(out + 4, r.initialGrant);
    return kStartVideoReplyBytes;
}

inline bool decodeStartVideoReply(const uint8_t* payload, size_t length, StartVideoReply& out)
{
    if (!payload || length != kStartVideoReplyBytes) return false;
    out.sessionToken = detail::get32(payload);
    out.initialGrant = detail::get32(payload + 4);
    return validStartVideoReply(out);
}

inline size_t encodeStopVideoRequest(uint32_t sessionToken, uint8_t* out, size_t capacity)
{
    if (!out || capacity < kStopVideoRequestBytes || !sessionToken) return 0;
    detail::put32(out, sessionToken);
    return kStopVideoRequestBytes;
}

inline bool decodeStopVideoRequest(const uint8_t* payload, size_t length, uint32_t& sessionToken)
{
    if (!payload || length != kStopVideoRequestBytes) return false;
    sessionToken = detail::get32(payload);
    return sessionToken != 0;
}

// ─── per-session assembly checks (device enforces, host self-checks) ──────────

/** @brief START against the session: viewport-sized image, JPEG within the negotiated cap. */
inline Reason checkStartForSession(const StartInfo& s, const StartVideoRequest& session)
{
    if (s.width != session.viewportW || s.height != session.viewportH) return ReasonBadDimensions;
    if (!s.jpegBytes) return ReasonBadLength;
    if (s.jpegBytes > session.maxJpegBytes) return ReasonTooLarge;
    return ReasonOk;
}

/**
 * @brief CHUNK against the frame being assembled.
 * @param expectedOffset Bytes of the frame received so far.
 * @param jpegBytes      The frame's START jpeg_bytes.
 */
inline Reason checkChunkForSession(const ChunkInfo& c, const StartVideoRequest& session,
                                   uint32_t expectedOffset, uint32_t jpegBytes)
{
    if (c.size > session.chunkBytes) return ReasonTooLarge;
    if (c.offset != expectedOffset) return ReasonBadOffset;
    if (c.offset > jpegBytes || c.size > jpegBytes - c.offset) return ReasonBadLength;
    return ReasonOk;
}

// ─── diagnostics ──────────────────────────────────────────────────────────────

/** @brief "Ok", "BadLength", ...; "Unknown" outside 0..kReasonCount-1. */
inline const char* reasonName(uint8_t reason)
{
    static const char* const kNames[kReasonCount] = {
        "Ok", "BadLength", "BadOffset", "BadCrc", "StaleSession", "NoCredit", "TooLarge",
        "BadDimensions", "AssemblyTimeout", "DecodeFailed", "Superseded", "Hidden", "Busy",
        "BadFrameId", "Malformed", "Cancelled", "Internal"
    };
    return reason < kReasonCount ? kNames[reason] : "Unknown";
}

inline const char* recordTypeName(uint8_t type)
{
    switch (type) {
    case RecordStart:      return "START";
    case RecordChunk:      return "CHUNK";
    case RecordEnd:        return "END";
    case RecordAbort:      return "ABORT";
    case RecordResult:     return "RESULT";
    case RecordPresented:  return "PRESENTED";
    case RecordCredit:     return "CREDIT";
    case RecordVisibility: return "VISIBILITY";
    case RecordMapPose:    return "MAP_POSE";
    default:               return "UNKNOWN";
    }
}

// ─── multiplexing parser ──────────────────────────────────────────────────────

/**
 * @brief Bounded parser for one byte stream carrying E3 28 control frames and
 *        E3 29 video records (either direction).
 *
 * - Fixed buffer of kMaxRecordBytes; no heap. Only the bytes of the current
 *   candidate are copied in, so large CHUNKs cost one memcpy plus the CRC.
 * - A candidate that fails a header check or its CRC loses ONE byte and the
 *   rest of its buffered bytes are rescanned from the next E3, so a false
 *   header in noise cannot swallow the records behind it.
 * - Accepted records are never rescanned: E3 28 / E3 29 inside JPEG data are
 *   ordinary bytes and nothing found inside an accepted payload is dispatched.
 * - After the inter-byte timeout, feed() (including empty polls) or expire()
 *   discards stale partial candidates and retains complete CRC-valid records
 *   buffered behind them (rollover-safe). Accepted payloads remain opaque.
 *
 * Usage (yields one item per call; loop until it reports ItemNone):
 * @code
 *   size_t used = 0;
 *   for (;;) {
 *       used += parser.feed(data + used, size - used, nowMs, item);
 *       if (item.kind == MuxParser::ItemNone) break;  // all input consumed
 *       handle(item);                                  // before the next feed()
 *   }
 * @endcode
 * Not thread-safe: one owner (the USB I/O task / worker).
 */
class MuxParser {
public:
    enum ItemKind : uint8_t { ItemNone = 0, ItemControl = 1, ItemVideo = 2 };

    struct Item {
        ItemKind kind = ItemNone;
        es3proto::Frame control; ///< kind == ItemControl: a copy, CRC-checked.
        Record video;            ///< kind == ItemVideo: framing-checked; payload borrows the parser.
    };

    /** Running counters since construction / resetStats(); reset() keeps them. */
    struct Stats {
        uint32_t controlFrames = 0;
        uint32_t videoRecords = 0;
        uint32_t garbageBytes = 0;  ///< Bytes discarded outside accepted items.
        uint32_t headerErrors = 0;  ///< Candidates rejected by a header check (incl. oversize).
        uint32_t oversize = 0;      ///< CHUNK headers above maxChunkData().
        uint32_t crcErrors = 0;
        uint32_t timeouts = 0;      ///< Partial candidates dropped by the timeout.
    };

    /**
     * @param maxChunkData     CHUNK data ceiling, clamped to 1..kMaxChunkDataBytes.
     *                         A longer CHUNK header is rejected before buffering.
     * @param partialTimeoutMs Inter-byte gap that abandons a partial candidate.
     */
    explicit MuxParser(size_t maxChunkData = kDefaultChunkDataBytes,
                       uint32_t partialTimeoutMs = kPartialTimeoutMs)
        : _maxChunkData(maxChunkData < 1 ? 1 : (maxChunkData > kMaxChunkDataBytes ? kMaxChunkDataBytes : maxChunkData)),
          _timeoutMs(partialTimeoutMs) {}

    /** @brief Drops every buffered byte (and the last item's view). Stats survive. */
    void reset() { _count = 0; _release = 0; }
    void resetStats() { _stats = Stats(); }
    size_t maxChunkData() const { return _maxChunkData; }
    /** @brief Bytes of an incomplete candidate (excludes the last returned item). */
    size_t buffered() const { return _count - _release; }
    const Stats& stats() const { return _stats; }

    /**
     * @brief Consumes bytes until one item completes or the input is exhausted.
     * @param data  Received bytes (may be NULL when @p size is 0).
     * @param nowMs Arrival time in ms (millis() / CLOCK_MONOTONIC).
     * @param item  Receives the item; item.kind == ItemNone means all input was
     *              consumed and nothing complete is buffered.
     * @return Bytes consumed from @p data. When an item is returned, call again
     *         with the remaining bytes (possibly none): buffered bytes may hold
     *         further items. The previous item's video payload is invalid after
     *         this call.
     */
    size_t feed(const uint8_t* data, size_t size, uint32_t nowMs, Item& item)
    {
        item.kind = ItemNone;
        releaseItem();
        if (!data) size = 0;
        // Recover old buffered records before appending bytes from a new
        // burst. Also service timeouts on empty feeds used by polling clients.
        expireBuffered(nowMs);
        if (size) _lastByteMs = nowMs;
        size_t used = 0;
        // Each pass appends >= 1 input byte, discards >= 1 buffered byte, hunts
        // to a sync byte (then appends), or returns: <= 3 * size + _count + 4.
        const size_t passes = 3 * size + _count + 4;
        for (size_t pass = 0; pass < passes; ++pass) {
            if (_count == 0) {
                if (used == size) return used;
                // Hunt for a sync byte in the input itself: garbage is never copied.
                const void* sync = memchr(data + used, kMagic0, size - used);
                const size_t skip = sync ? size_t(static_cast<const uint8_t*>(sync) - (data + used))
                                         : size - used;
                _stats.garbageBytes += uint32_t(skip);
                used += skip;
                if (!sync) return used;
            }
            size_t need = 0;
            switch (examine(need)) {
            case StepNeed: {
                if (used == size) return used;
                size_t take = need - _count;
                if (take > size - used) take = size - used;
                memcpy(_buf + _count, data + used, take);
                _count += take;
                used += take;
                break;
            }
            case StepGarbage:   skipToSync(); break;
            case StepBadHeader: ++_stats.headerErrors; skipToSync(); break;
            case StepOversize:  ++_stats.headerErrors; ++_stats.oversize; skipToSync(); break;
            case StepBadCrc:    ++_stats.crcErrors; skipToSync(); break;
            case StepControl:
                item.kind = ItemControl;
                item.control.sequence = _buf[3];
                item.control.type = _buf[4];
                item.control.length = _buf[5];
                if (_buf[5]) memcpy(item.control.payload, _buf + 6, _buf[5]);
                ++_stats.controlFrames;
                _release = need;
                return used;
            case StepVideo:
                item.kind = ItemVideo;
                item.video = makeRecord(_buf[3], detail::get32(_buf + 6), detail::get32(_buf + 10),
                                        _buf + kHeaderBytes, detail::get16(_buf + 4));
                ++_stats.videoRecords;
                _release = need;
                return used;
            }
        }
        // Unreachable by the bound above; never leave a wedged candidate.
        dropAll(false);
        return size;
    }

    /**
     * @brief Drops timed-out partial candidates but preserves complete records
     *        for the next feed(). Invalidates the last returned item's view.
     * @return True if something was dropped.
     */
    bool expire(uint32_t nowMs)
    {
        releaseItem();
        return expireBuffered(nowMs);
    }

private:
    enum Step : uint8_t {
        StepNeed, StepGarbage, StepBadHeader, StepOversize, StepBadCrc, StepControl, StepVideo
    };

    uint8_t _buf[kMaxRecordBytes]{};
    size_t _count = 0;        ///< Buffered bytes; _buf[0] is the candidate's first byte.
    size_t _release = 0;      ///< Size of the item returned by the previous feed().
    size_t _maxChunkData;
    uint32_t _timeoutMs;
    uint32_t _lastByteMs = 0;
    Stats _stats;

    /**
     * Classifies the candidate at _buf[offset]. Checks run as soon as their
     * bytes are present, rejecting false headers after 3-14 bytes. StepNeed
     * sets need to the required candidate size; complete items set its total
     * size. Timeout recovery may examine a suffix without copying it.
     */
    Step examine(size_t& need, size_t offset = 0) const
    {
        const uint8_t* buf = _buf + offset;
        const size_t count = _count - offset;
        if (count == 0) { need = 1; return StepNeed; }
        if (buf[0] != kMagic0) return StepGarbage;
        if (count < 2) { need = 2; return StepNeed; }
        if (buf[1] == kControlMagic1) {
            if (count < 6) { need = 6; return StepNeed; }
            if (buf[2] != es3proto::kVersion || buf[5] > es3proto::kMaxPayload) return StepBadHeader;
            need = size_t(buf[5]) + 8;
            if (count < need) return StepNeed;
            return es3proto::crc16(buf + 2, need - 4) == detail::get16(buf + need - 2) ? StepControl
                                                                                         : StepBadCrc;
        }
        if (buf[1] != kMagic1) return StepGarbage;
        if (count < 3) { need = 3; return StepNeed; }
        if (buf[2] != kVersion) return StepBadHeader;
        if (count < 4) { need = 4; return StepNeed; }
        if (!knownType(buf[3])) return StepBadHeader;
        if (count < 6) { need = 6; return StepNeed; }
        const size_t length = detail::get16(buf + 4);
        // Token and frame id are not yet received: pass placeholders that the
        // length checks cannot reject, then re-check everything at 14 bytes.
        const Reason early = checkFraming(buf[3], length, 1, namesFrame(buf[3]) ? 1 : 0, _maxChunkData);
        if (early == ReasonTooLarge) return StepOversize;
        if (early != ReasonOk) return StepBadHeader;
        if (count < kHeaderBytes) { need = kHeaderBytes; return StepNeed; }
        if (checkFraming(buf[3], length, detail::get32(buf + 6), detail::get32(buf + 10),
                         _maxChunkData) != ReasonOk) {
            return StepBadHeader;
        }
        need = kOverheadBytes + length;
        if (count < need) return StepNeed;
        return es3proto::crc16(buf + 2, kHeaderBytes - 2 + length) ==
                       detail::get16(buf + kHeaderBytes + length)
                   ? StepVideo
                   : StepBadCrc;
    }

    // In-place bounded compaction: abandon timed-out candidates, retaining
    // complete CRC-valid records in order. Jump over each accepted record as
    // a whole; magic inside its payload must never be dispatched separately.
    // Remove stale partial suffixes too, before fresh bytes can complete them.
    bool expireBuffered(uint32_t nowMs)
    {
        if (!_count || nowMs - _lastByteMs <= _timeoutMs) return false;
        size_t kept = 0;
        bool timedOut = false;
        for (size_t scan = 0; scan < _count;) {
            size_t need = 0;
            const Step step = examine(need, scan);
            if (step == StepControl || step == StepVideo) {
                memmove(_buf + kept, _buf + scan, need);
                kept += need;
                scan += need;
            } else {
                timedOut = timedOut || step == StepNeed;
                if (step == StepBadCrc) ++_stats.crcErrors;
                if (step == StepBadHeader || step == StepOversize) ++_stats.headerErrors;
                if (step == StepOversize) ++_stats.oversize;
                ++scan;
            }
        }
        const size_t dropped = _count - kept;
        _stats.garbageBytes += uint32_t(dropped);
        if (timedOut) ++_stats.timeouts;
        _count = kept;
        return dropped != 0;
    }

    void discard(size_t count)
    {
        if (count >= _count) { _count = 0; return; }
        _count -= count;
        memmove(_buf, _buf + count, _count);
    }

    /** Rejects the candidate's first byte and everything before the next buffered E3. */
    void skipToSync()
    {
        const void* sync = _count > 1 ? memchr(_buf + 1, kMagic0, _count - 1) : nullptr;
        const size_t drop = sync ? size_t(static_cast<const uint8_t*>(sync) - _buf) : _count;
        _stats.garbageBytes += uint32_t(drop);
        discard(drop);
    }

    void releaseItem()
    {
        if (!_release) return;
        discard(_release);
        _release = 0;
    }

    void dropAll(bool timedOut)
    {
        _stats.garbageBytes += uint32_t(_count);
        if (timedOut) ++_stats.timeouts;
        _count = 0;
        _release = 0;
    }
};

}  // namespace es3video
