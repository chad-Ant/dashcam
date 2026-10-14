#pragma once
/**
 * @file Es3MapProtocol.h
 * @brief Portable wire contract for the ES3C28P map surface (map extension v1,
 *        2026-10-14): three E3 28 control commands that bind the board's image
 *        receiver to the MAP tab instead of the camera preview.
 *
 * Keep this file byte-identical in both locations:
 *   peripherals/es3c28p/lib/es3UsbLink/   (ESP32-S3 firmware)
 *   lib/libes3display/                    (Jetson host link)
 * peripherals/es3c28p/tests/run_tests.sh cmp-checks the copies, and
 * tests/map_golden_vectors.txt freezes the bytes for C++ and Python alike.
 * Header-only, C++11, no heap/exceptions/Arduino/POSIX, like Es3VideoProtocol.h.
 *
 * ── What it adds (and what it leaves alone) ──────────────────────────────────
 * E3 28 framing, E3 29 records, CRCs, reason codes, the video golden vectors
 * and the START_VIDEO semantics are unchanged. A map image is an ordinary
 * E3 29 frame (START/CHUNK/END, RESULT/PRESENTED/CREDIT/VISIBILITY, the same
 * JPEG limits and credit rules) carried under a MAP session token. The board
 * has ONE image receiver: at most one session (video or map) exists at a time.
 * A host stops the current session (STOP_VIDEO / STOP_MAP) before starting the
 * other; a START_MAP while a video session exists, or a START_VIDEO while a map
 * session exists, is refused with NotReady and changes nothing.
 *
 *   0A MAP_CAPABILITIES  request empty; reply kMapCapabilitiesReplyBytes (16)
 *   0B START_MAP         request kStartMapRequestBytes; reply kStartMapReplyBytes
 *                        (session_token u32 nonzero, initial_grant u32, as START_VIDEO)
 *   0C STOP_MAP          request session_token u32; reply empty. A token that is
 *                        not the current map session is a no-op that still gets
 *                        the empty reply (it never ends a newer session).
 *
 * Firmware without the extension answers 0A..0C with BadCommand (or NotReady
 * when its image memory is missing): the host then treats the map as
 * unsupported and keeps every other feature. 0A..0C are only valid after
 * HELLO, like every command; a new HELLO ends any map session.
 *
 * ── Pages and visibility ─────────────────────────────────────────────────────
 * MAP_CAPABILITIES reports the board's tab count and which page is the map
 * (page_count 6, map_page 5 on the ES3C28P: LIVE TRIP CAM SYS SET MAP). The
 * active page is read with the existing STATUS command (reply byte 4). A map
 * session's VISIBILITY is active only while the map page is shown (and no
 * calibration/overlay owns the screen), exactly as video's is on LIVE.
 *
 * ── Geometry ─────────────────────────────────────────────────────────────────
 * The board fixes the map viewport (map_viewport_*: 0,0,320,224 on the
 * ES3C28P, above the 16-px tab strip). START_MAP must repeat it exactly, and
 * every map START must be exactly viewport_w x viewport_h. The host composes
 * every pixel inside it (status strip, map, attribution); the board may draw
 * only its own local stale/unavailable notice over the top status strip.
 *
 * ── generation ───────────────────────────────────────────────────────────────
 * An opaque host counter echoed nowhere: the board stores it with the binding
 * so its logs can tell map sessions apart. It never affects validation.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "Es3VideoProtocol.h"

namespace es3map {

constexpr uint8_t kVersion = 1;

/** E3 28 command ids of the map extension (07-09 are video, 01-06 the base set). */
enum Command : uint8_t {
    CmdMapCapabilities = 0x0A,
    CmdStartMap = 0x0B,
    CmdStopMap = 0x0C,
};

constexpr size_t kMapCapabilitiesReplyBytes = 16;
constexpr size_t kStartMapRequestBytes = 27;
constexpr size_t kStartMapReplyBytes = 8;
constexpr size_t kStopMapRequestBytes = 4;
static_assert(kMapCapabilitiesReplyBytes <= es3proto::kMaxPayload &&
                  kStartMapRequestBytes <= es3proto::kMaxPayload,
              "map control payloads must fit one E3 28 frame");

constexpr uint8_t kMaxPages = 8;   ///< Sanity ceiling for page_count.

/** MAP_CAPABILITIES flags. */
constexpr uint8_t kFlagMapTab = 0x01;   ///< The map page is a selectable tab.

/**
 * MAP_CAPABILITIES reply (16 bytes): map_protocol_version u8, page_count u8,
 * map_page u8, map_viewport_x u16, map_viewport_y u16, map_viewport_w u16,
 * map_viewport_h u16, max_jpeg_bytes u32, flags u8.
 */
struct MapCapabilities {
    uint8_t mapProtocolVersion = kVersion;
    uint8_t pageCount = 0;
    uint8_t mapPage = 0;
    uint16_t viewportX = 0;
    uint16_t viewportY = 0;
    uint16_t viewportW = 0;
    uint16_t viewportH = 0;
    uint32_t maxJpegBytes = 0;
    uint8_t flags = 0;
};

/**
 * START_MAP request (27 bytes): map_protocol_version u8, page u8,
 * viewport_x u16, viewport_y u16, viewport_w u16, viewport_h u16,
 * max_jpeg_bytes u32, chunk_bytes u16, target_fps u8, assembly_timeout_ms u16,
 * stale_after_ms u16, placeholder_after_ms u16, generation u32.
 */
struct StartMapRequest {
    uint8_t mapProtocolVersion = kVersion;
    uint8_t page = 0;
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
    uint32_t generation = 0;
};

/** START_MAP reply: the same 8 bytes as the START_VIDEO reply. */
typedef es3video::StartVideoReply StartMapReply;

/**
 * @brief True for a usable v1 map capability set, given the board's video
 *        capabilities @p video (the canvas and the decoder limits it shares).
 */
inline bool validMapCapabilities(const MapCapabilities& m, const es3video::Capabilities& video)
{
    if (!es3video::validCapabilities(video)) return false;
    return m.mapProtocolVersion == kVersion &&
           m.pageCount >= 1 && m.pageCount <= kMaxPages && m.mapPage < m.pageCount &&
           m.viewportW && m.viewportH && m.viewportW <= video.maxWidth && m.viewportH <= video.maxHeight &&
           uint32_t(m.viewportX) + m.viewportW <= video.canvasWidth &&
           uint32_t(m.viewportY) + m.viewportH <= video.canvasHeight &&
           m.maxJpegBytes && m.maxJpegBytes <= video.maxJpegBytes;
}

/** @return kMapCapabilitiesReplyBytes, or 0 if invalid / @p out too small. */
inline size_t encodeMapCapabilitiesReply(const MapCapabilities& m, const es3video::Capabilities& video,
                                         uint8_t* out, size_t capacity)
{
    if (!out || capacity < kMapCapabilitiesReplyBytes || !validMapCapabilities(m, video)) return 0;
    out[0] = m.mapProtocolVersion;
    out[1] = m.pageCount;
    out[2] = m.mapPage;
    es3video::detail::put16(out + 3, m.viewportX);
    es3video::detail::put16(out + 5, m.viewportY);
    es3video::detail::put16(out + 7, m.viewportW);
    es3video::detail::put16(out + 9, m.viewportH);
    es3video::detail::put32(out + 11, m.maxJpegBytes);
    out[15] = m.flags;
    return kMapCapabilitiesReplyBytes;
}

/**
 * @brief Parses a MAP_CAPABILITIES reply against the board's video
 *        capabilities. @p out is filled whenever the length is right.
 * @return True only for a valid v1 map capability set.
 */
inline bool decodeMapCapabilitiesReply(const uint8_t* payload, size_t length,
                                       const es3video::Capabilities& video, MapCapabilities& out)
{
    if (!payload || length != kMapCapabilitiesReplyBytes) return false;
    out.mapProtocolVersion = payload[0];
    out.pageCount = payload[1];
    out.mapPage = payload[2];
    out.viewportX = es3video::detail::get16(payload + 3);
    out.viewportY = es3video::detail::get16(payload + 5);
    out.viewportW = es3video::detail::get16(payload + 7);
    out.viewportH = es3video::detail::get16(payload + 9);
    out.maxJpegBytes = es3video::detail::get32(payload + 11);
    out.flags = payload[15];
    return validMapCapabilities(out, video);
}

/**
 * @brief True when @p r fits the board: same version, the advertised map page
 *        and EXACTLY the advertised map viewport, JPEG within the map limit,
 *        chunk within the video limit, 1..30 fps, nonzero timeouts <=
 *        es3video::kMaxTimeoutMs and placeholder >= stale. generation is free.
 */
inline bool validStartMap(const StartMapRequest& r, const MapCapabilities& m, const es3video::Capabilities& video)
{
    if (!validMapCapabilities(m, video)) return false;
    if (r.mapProtocolVersion != kVersion || r.page != m.mapPage) return false;
    if (r.viewportX != m.viewportX || r.viewportY != m.viewportY ||
        r.viewportW != m.viewportW || r.viewportH != m.viewportH) {
        return false;
    }
    if (!r.maxJpegBytes || r.maxJpegBytes > m.maxJpegBytes ||
        !r.chunkBytes || r.chunkBytes > video.maxChunkBytes ||
        !r.targetFps || r.targetFps > es3video::kMaxTargetFps) {
        return false;
    }
    return r.assemblyTimeoutMs && r.assemblyTimeoutMs <= es3video::kMaxTimeoutMs &&
           r.staleAfterMs && r.staleAfterMs <= es3video::kMaxTimeoutMs &&
           r.placeholderAfterMs >= r.staleAfterMs && r.placeholderAfterMs <= es3video::kMaxTimeoutMs;
}

/** @return kStartMapRequestBytes, or 0 if !validStartMap() or @p out too small. */
inline size_t encodeStartMapRequest(const StartMapRequest& r, const MapCapabilities& m,
                                    const es3video::Capabilities& video, uint8_t* out, size_t capacity)
{
    if (!out || capacity < kStartMapRequestBytes || !validStartMap(r, m, video)) return 0;
    out[0] = r.mapProtocolVersion;
    out[1] = r.page;
    es3video::detail::put16(out + 2, r.viewportX);
    es3video::detail::put16(out + 4, r.viewportY);
    es3video::detail::put16(out + 6, r.viewportW);
    es3video::detail::put16(out + 8, r.viewportH);
    es3video::detail::put32(out + 10, r.maxJpegBytes);
    es3video::detail::put16(out + 14, r.chunkBytes);
    out[16] = r.targetFps;
    es3video::detail::put16(out + 17, r.assemblyTimeoutMs);
    es3video::detail::put16(out + 19, r.staleAfterMs);
    es3video::detail::put16(out + 21, r.placeholderAfterMs);
    es3video::detail::put32(out + 23, r.generation);
    return kStartMapRequestBytes;
}

/**
 * @brief Board side: parses a START_MAP request against its own limits.
 *        False means reply Error BadPayload and keep the previous state.
 */
inline bool decodeStartMapRequest(const uint8_t* payload, size_t length, const MapCapabilities& m,
                                  const es3video::Capabilities& video, StartMapRequest& out)
{
    if (!payload || length != kStartMapRequestBytes) return false;
    out.mapProtocolVersion = payload[0];
    out.page = payload[1];
    out.viewportX = es3video::detail::get16(payload + 2);
    out.viewportY = es3video::detail::get16(payload + 4);
    out.viewportW = es3video::detail::get16(payload + 6);
    out.viewportH = es3video::detail::get16(payload + 8);
    out.maxJpegBytes = es3video::detail::get32(payload + 10);
    out.chunkBytes = es3video::detail::get16(payload + 14);
    out.targetFps = payload[16];
    out.assemblyTimeoutMs = es3video::detail::get16(payload + 17);
    out.staleAfterMs = es3video::detail::get16(payload + 19);
    out.placeholderAfterMs = es3video::detail::get16(payload + 21);
    out.generation = es3video::detail::get32(payload + 23);
    return validStartMap(out, m, video);
}

inline size_t encodeStartMapReply(const StartMapReply& r, uint8_t* out, size_t capacity)
{
    return es3video::encodeStartVideoReply(r, out, capacity);
}

inline bool decodeStartMapReply(const uint8_t* payload, size_t length, StartMapReply& out)
{
    return es3video::decodeStartVideoReply(payload, length, out);
}

inline size_t encodeStopMapRequest(uint32_t sessionToken, uint8_t* out, size_t capacity)
{
    return es3video::encodeStopVideoRequest(sessionToken, out, capacity);
}

inline bool decodeStopMapRequest(const uint8_t* payload, size_t length, uint32_t& sessionToken)
{
    return es3video::decodeStopVideoRequest(payload, length, sessionToken);
}

/**
 * @brief The session limits a map session imposes on its E3 29 frames, in the
 *        form the video helpers (checkStartForSession, checkChunkForSession,
 *        VideoState) already take: same viewport, JPEG/chunk limits and timers.
 */
inline es3video::StartVideoRequest sessionLimits(const StartMapRequest& r)
{
    es3video::StartVideoRequest s;
    s.videoProtocolVersion = es3video::kVersion;
    s.codec = es3video::CodecBaselineJpeg;
    s.viewportX = r.viewportX;
    s.viewportY = r.viewportY;
    s.viewportW = r.viewportW;
    s.viewportH = r.viewportH;
    s.maxJpegBytes = r.maxJpegBytes;
    s.chunkBytes = r.chunkBytes;
    s.targetFps = r.targetFps;
    s.assemblyTimeoutMs = r.assemblyTimeoutMs;
    s.staleAfterMs = r.staleAfterMs;
    s.placeholderAfterMs = r.placeholderAfterMs;
    return s;
}

}  // namespace es3map
