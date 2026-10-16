#pragma once
/**
 * @file Es3MapProtocol.h
 * @brief Portable wire contract for the ES3C28P map surface (map extension v1,
 *        2026-10-14): three E3 28 control commands that bind the board's image
 *        receiver to the MAP tab instead of the camera preview; extension v2
 *        (same day, additive) lets the board pan a larger map itself.
 *
 * Keep this file byte-identical in both locations:
 *   peripherals/es3c28p/lib/es3UsbLink/   (ESP32-S3 firmware)
 *   lib/libes3display/                    (Jetson host link)
 * peripherals/es3c28p/tests/run_tests.sh cmp-checks the copies, and
 * tests/map_golden_vectors.txt freezes the v1 bytes for C++ and Python alike
 * (tests/pan_golden_vectors.txt the v2 bytes, C++ only).
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
 *
 * ── Extension v2: board-side panning (2026-10-14, additive) ──────────────────
 * Everything above is unchanged and still required. v2 adds a second way to
 * fill the same map session, in which the board scrolls the map itself:
 *
 *   0D PAN_CAPABILITIES  request empty; reply kPanCapabilitiesReplyBytes (9)
 *   0E START_PAN         request kStartPanRequestBytes (31): the 27 START_MAP
 *                        bytes, then background_w u16, background_h u16;
 *                        reply as START_MAP. The session it opens is a map
 *                        session: STOP_MAP ends it, START_VIDEO is refused
 *                        while it exists, START_MAP / START_PAN replace it.
 *   E3 29 type 10 MAP_POSE (Es3VideoProtocol.h), see MapPose.
 *
 * Firmware without v2 answers 0D with BadCommand: the host keeps using
 * START_MAP and whole composed frames. Ids above 0E are free for other plans.
 *
 * A panning session carries two kinds of ordinary E3 29 images, told apart
 * by their START dimensions (anything else is BadDimensions):
 *   background  background_w x background_h: the map imagery only, larger than
 *               the map area so the board can move it. START source_timestamp_us
 *               carries its origin instead of a time: packOrigin(x, y), the world
 *               pixel of its top-left corner at the host's zoom (opaque to the
 *               board apart from the subtraction in computePanView()).
 *   chrome      viewport_w x (chrome_top_h + chrome_bottom_h): the fixed strips.
 *               Its first chrome_top_h rows go to the top of the viewport, the
 *               rest to its bottom (status strip and attribution on the ES3C28P).
 * The map area is the viewport between the strips (320x192 at 0,16). Credits,
 * RESULT/PRESENTED and VISIBILITY work as for START_MAP; a newer image
 * supersedes only a not yet presented image OF THE SAME KIND (a chrome never
 * replaces a background). The board keeps the newest presented image of each
 * kind while the map is shown; it may drop them when the map is hidden,
 * because the host sends both again whenever VISIBILITY turns active (and
 * after every START_PAN). Until both have arrived the board shows its own
 * waiting notice in their place.
 *
 * MAP_POSE says where the map area is centred and where the marker is, in the
 * same world pixels (1/16 px). The host sends it often (typically 20 Hz, at most
 * max_pose_hz) and interpolates between GPS fixes itself; the board applies the
 * newest pose as is, to whichever background it shows (computePanView()), and
 * redraws when the crop or the marker changes. A pose, like a complete image,
 * restarts the stale/placeholder timers: in a panning session images may be
 * seconds apart while the poses keep coming. Poses carry no credit and get no
 * reply; one with an older seq, another token or bad fields is ignored.
 * Coordinates are unwrapped: near the antimeridian the host may send x beyond
 * the world width so that origin and position stay in one continuous frame.
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

// ─── extension v2: board-side panning ────────────────────────────────────────

constexpr uint8_t kPanVersion = 1;

enum PanCommand : uint8_t {
    CmdPanCapabilities = 0x0D,
    CmdStartPan = 0x0E,
};

constexpr size_t kPanCapabilitiesReplyBytes = 9;
constexpr size_t kStartPanRequestBytes = kStartMapRequestBytes + 4;
static_assert(kStartPanRequestBytes <= es3proto::kMaxPayload, "START_PAN must fit one E3 28 frame");

constexpr uint16_t kMaxPanBackground = 1024;   ///< Protocol ceiling per background dimension.
constexpr uint8_t kMaxPoseHz = 60;
constexpr uint32_t kPoseSubpixels = 16;        ///< MAP_POSE coordinates are 1/16 world pixel.

/**
 * PAN_CAPABILITIES reply (9 bytes): pan_version u8, max_background_w u16,
 * max_background_h u16, chrome_top_h u8, chrome_bottom_h u8, max_pose_hz u8,
 * flags u8 (none defined; the host ignores unknown bits).
 * ES3C28P target: 1, (its PSRAM budget), 16, 16, 30, 0.
 */
struct PanCapabilities {
    uint8_t panVersion = kPanVersion;
    uint16_t maxBackgroundW = 0;
    uint16_t maxBackgroundH = 0;
    uint8_t chromeTopH = 0;
    uint8_t chromeBottomH = 0;
    uint8_t maxPoseHz = 0;
    uint8_t flags = 0;
};

/** START_PAN request: START_MAP's fields, then the background size. */
struct StartPanRequest {
    StartMapRequest map;
    uint16_t backgroundW = 0;
    uint16_t backgroundH = 0;
};

/** START_PAN reply: the same 8 bytes as START_MAP / START_VIDEO. */
typedef es3video::StartVideoReply StartPanReply;

/** Height of the map area: the viewport between the two chrome strips. */
inline uint16_t panMapHeight(const MapCapabilities& m, const PanCapabilities& p)
{
    const uint32_t chrome = uint32_t(p.chromeTopH) + p.chromeBottomH;
    return chrome < m.viewportH ? uint16_t(m.viewportH - chrome) : 0;
}

/**
 * @brief True for a usable v2 pan capability set on a board whose map
 *        capabilities @p m are valid: strips leave a map area, the largest
 *        background covers it and stays within kMaxPanBackground, and the
 *        board takes 1..kMaxPoseHz poses a second.
 */
inline bool validPanCapabilities(const PanCapabilities& p, const MapCapabilities& m,
                                 const es3video::Capabilities& video)
{
    if (!validMapCapabilities(m, video) || p.panVersion != kPanVersion) return false;
    const uint16_t mapH = panMapHeight(m, p);
    return mapH &&
           p.maxBackgroundW >= m.viewportW && p.maxBackgroundW <= kMaxPanBackground &&
           p.maxBackgroundH >= mapH && p.maxBackgroundH <= kMaxPanBackground &&
           p.maxPoseHz >= 1 && p.maxPoseHz <= kMaxPoseHz;
}

/** @return kPanCapabilitiesReplyBytes, or 0 if invalid / @p out too small. */
inline size_t encodePanCapabilitiesReply(const PanCapabilities& p, const MapCapabilities& m,
                                         const es3video::Capabilities& video, uint8_t* out, size_t capacity)
{
    if (!out || capacity < kPanCapabilitiesReplyBytes || !validPanCapabilities(p, m, video)) return 0;
    out[0] = p.panVersion;
    es3video::detail::put16(out + 1, p.maxBackgroundW);
    es3video::detail::put16(out + 3, p.maxBackgroundH);
    out[5] = p.chromeTopH;
    out[6] = p.chromeBottomH;
    out[7] = p.maxPoseHz;
    out[8] = p.flags;
    return kPanCapabilitiesReplyBytes;
}

/** @brief Parses a PAN_CAPABILITIES reply; @p out is filled whenever the length is right. */
inline bool decodePanCapabilitiesReply(const uint8_t* payload, size_t length, const MapCapabilities& m,
                                       const es3video::Capabilities& video, PanCapabilities& out)
{
    if (!payload || length != kPanCapabilitiesReplyBytes) return false;
    out.panVersion = payload[0];
    out.maxBackgroundW = es3video::detail::get16(payload + 1);
    out.maxBackgroundH = es3video::detail::get16(payload + 3);
    out.chromeTopH = payload[5];
    out.chromeBottomH = payload[6];
    out.maxPoseHz = payload[7];
    out.flags = payload[8];
    return validPanCapabilities(out, m, video);
}

/**
 * @brief True when @p r fits the board: a valid START_MAP part (validStartMap)
 *        and a background at least as large as the map area and at most the
 *        advertised maximum.
 */
inline bool validStartPan(const StartPanRequest& r, const PanCapabilities& p, const MapCapabilities& m,
                          const es3video::Capabilities& video)
{
    if (!validPanCapabilities(p, m, video) || !validStartMap(r.map, m, video)) return false;
    return r.backgroundW >= m.viewportW && r.backgroundW <= p.maxBackgroundW &&
           r.backgroundH >= panMapHeight(m, p) && r.backgroundH <= p.maxBackgroundH;
}

/** @return kStartPanRequestBytes, or 0 if !validStartPan() or @p out too small. */
inline size_t encodeStartPanRequest(const StartPanRequest& r, const PanCapabilities& p, const MapCapabilities& m,
                                    const es3video::Capabilities& video, uint8_t* out, size_t capacity)
{
    if (!out || capacity < kStartPanRequestBytes || !validStartPan(r, p, m, video)) return 0;
    if (encodeStartMapRequest(r.map, m, video, out, capacity) != kStartMapRequestBytes) return 0;
    es3video::detail::put16(out + kStartMapRequestBytes, r.backgroundW);
    es3video::detail::put16(out + kStartMapRequestBytes + 2, r.backgroundH);
    return kStartPanRequestBytes;
}

/** @brief Board side: parses START_PAN; false means Error BadPayload, state unchanged. */
inline bool decodeStartPanRequest(const uint8_t* payload, size_t length, const PanCapabilities& p,
                                  const MapCapabilities& m, const es3video::Capabilities& video,
                                  StartPanRequest& out)
{
    if (!payload || length != kStartPanRequestBytes) return false;
    // The START_MAP part alone; decodeStartMapRequest() wants its exact length.
    if (!decodeStartMapRequest(payload, kStartMapRequestBytes, m, video, out.map)) return false;
    out.backgroundW = es3video::detail::get16(payload + kStartMapRequestBytes);
    out.backgroundH = es3video::detail::get16(payload + kStartMapRequestBytes + 2);
    return validStartPan(out, p, m, video);
}

inline size_t encodeStartPanReply(const StartPanReply& r, uint8_t* out, size_t capacity)
{
    return es3video::encodeStartVideoReply(r, out, capacity);
}

inline bool decodeStartPanReply(const uint8_t* payload, size_t length, StartPanReply& out)
{
    return es3video::decodeStartVideoReply(payload, length, out);
}

/** What one image of a panning session is (by its START dimensions). */
enum PanImage : uint8_t { PanImageNone = 0, PanImageBackground = 1, PanImageChrome = 2 };

/** The geometry a panning session fixes; the board keeps one per session. */
struct PanSession {
    StartPanRequest request;
    uint8_t chromeTopH = 0;
    uint8_t chromeBottomH = 0;
    uint16_t mapX = 0, mapY = 0, mapW = 0, mapH = 0;   ///< Map area on the canvas.
};

/** @brief The geometry of a validated START_PAN (call after validStartPan()). */
inline PanSession panSession(const StartPanRequest& r, const PanCapabilities& p)
{
    PanSession s;
    s.request = r;
    s.chromeTopH = p.chromeTopH;
    s.chromeBottomH = p.chromeBottomH;
    s.mapX = r.map.viewportX;
    s.mapY = uint16_t(r.map.viewportY + p.chromeTopH);
    s.mapW = r.map.viewportW;
    const uint32_t chrome = uint32_t(p.chromeTopH) + p.chromeBottomH;
    s.mapH = chrome < r.map.viewportH ? uint16_t(r.map.viewportH - chrome) : 0;
    return s;
}

/**
 * @brief START check for a panning session, used instead of
 *        es3video::checkStartForSession(): background or chrome size,
 *        nonzero JPEG within the session limit. @p kind says which.
 */
inline es3video::Reason checkPanStart(const es3video::StartInfo& s, const PanSession& session, PanImage& kind)
{
    kind = PanImageNone;
    if (s.width == session.request.backgroundW && s.height == session.request.backgroundH) {
        kind = PanImageBackground;
    } else if (s.width == session.request.map.viewportW &&
               s.height == uint32_t(session.chromeTopH) + session.chromeBottomH) {
        kind = PanImageChrome;
    } else {
        return es3video::ReasonBadDimensions;
    }
    if (!s.jpegBytes) return es3video::ReasonBadLength;
    if (s.jpegBytes > session.request.map.maxJpegBytes) return es3video::ReasonTooLarge;
    return es3video::ReasonOk;
}

/** Background origin <-> START source_timestamp_us. */
inline uint64_t packOrigin(uint32_t worldX, uint32_t worldY) { return uint64_t(worldX) | (uint64_t(worldY) << 32); }
inline uint32_t originX(uint64_t packed) { return uint32_t(packed); }
inline uint32_t originY(uint64_t packed) { return uint32_t(packed >> 32); }

/** MAP_POSE marker; the board draws it (see NAVIGATION_PLAN.md for the shapes). */
enum PoseMarker : uint8_t { MarkerNone = 0, MarkerDot = 1, MarkerArrow = 2, MarkerLastKnown = 3 };

/**
 * MAP_POSE payload (16 bytes): seq u32 (1, 2, 3... per session; the board keeps
 * only increasing ones), x_q4 u32, y_q4 u32 (world pixels x 16 at the host's
 * zoom: the centre of the map area and the marker position), course_cdeg u16
 * (0..35999, clockwise from north; used by MarkerArrow only), marker u8
 * (PoseMarker), flags u8 (none defined; receivers ignore unknown bits).
 */
struct MapPose {
    uint32_t seq = 0;
    uint32_t xQ4 = 0;
    uint32_t yQ4 = 0;
    uint16_t courseCdeg = 0;
    uint8_t marker = MarkerNone;
    uint8_t flags = 0;
};

inline bool validMapPose(const MapPose& p)
{
    return p.seq != 0 && p.courseCdeg < 36000 && p.marker <= MarkerLastKnown;
}

/** @return The complete E3 29 MAP_POSE record length, or 0 if invalid / too small. */
inline size_t encodeMapPose(uint32_t sessionToken, const MapPose& p, uint8_t* out, size_t capacity)
{
    if (!validMapPose(p)) return 0;
    uint8_t payload[es3video::kMapPosePayloadBytes];
    es3video::detail::put32(payload, p.seq);
    es3video::detail::put32(payload + 4, p.xQ4);
    es3video::detail::put32(payload + 8, p.yQ4);
    es3video::detail::put16(payload + 12, p.courseCdeg);
    payload[14] = p.marker;
    payload[15] = p.flags;
    return es3video::encodeRecord(es3video::RecordMapPose, sessionToken, 0, payload, sizeof(payload), out, capacity);
}

/** @brief Fields of a framed MAP_POSE record; Malformed for bad field values. */
inline es3video::Reason decodeMapPose(const es3video::Record& r, MapPose& out)
{
    const es3video::Reason framing = es3video::detail::checkTyped(r, es3video::RecordMapPose);
    if (framing != es3video::ReasonOk) return framing;
    out.seq = es3video::detail::get32(r.payload);
    out.xQ4 = es3video::detail::get32(r.payload + 4);
    out.yQ4 = es3video::detail::get32(r.payload + 8);
    out.courseCdeg = es3video::detail::get16(r.payload + 12);
    out.marker = r.payload[14];
    out.flags = r.payload[15];
    return validMapPose(out) ? es3video::ReasonOk : es3video::ReasonMalformed;
}

/**
 * Where the board draws: the background pixel at the map area's top-left
 * (cropX, cropY), and the marker relative to the map area's top-left in 1/16 px
 * (also when it lies outside the area; markerVisible says whether to draw it).
 */
struct PanView {
    int32_t cropX = 0, cropY = 0;
    int32_t markerXQ4 = 0, markerYQ4 = 0;
    bool markerVisible = false;
};

namespace detail {
inline int64_t floorDiv(int64_t a, int64_t b) { return a >= 0 ? a / b : -((-a + b - 1) / b); }
inline int32_t clampCrop(int64_t v, int64_t span) { return int32_t(v < 0 ? 0 : (v > span ? span : v)); }
}  // namespace detail

/**
 * @brief Integer crop and marker placement, identical on both ends: centre the
 *        map area on the pose, clamp the crop inside the background, place the
 *        marker relative to the crop (hidden for MarkerNone or outside the area).
 *        Integer only, so the board needs no floating point.
 */
inline PanView computePanView(uint64_t packedOrigin, uint16_t backgroundW, uint16_t backgroundH, uint16_t mapW,
                              uint16_t mapH, const MapPose& pose)
{
    PanView v;
    const int64_t q = kPoseSubpixels;
    const int64_t relX = int64_t(pose.xQ4) - int64_t(originX(packedOrigin)) * q;
    const int64_t relY = int64_t(pose.yQ4) - int64_t(originY(packedOrigin)) * q;
    const int64_t spanX = backgroundW > mapW ? backgroundW - mapW : 0;
    const int64_t spanY = backgroundH > mapH ? backgroundH - mapH : 0;
    v.cropX = detail::clampCrop(detail::floorDiv(relX - int64_t(mapW) * q / 2, q), spanX);
    v.cropY = detail::clampCrop(detail::floorDiv(relY - int64_t(mapH) * q / 2, q), spanY);
    const int64_t mx = relX - int64_t(v.cropX) * q;
    const int64_t my = relY - int64_t(v.cropY) * q;
    const bool inside = mx >= 0 && my >= 0 && mx < int64_t(mapW) * q && my < int64_t(mapH) * q;
    v.markerXQ4 = int32_t(mx);
    v.markerYQ4 = int32_t(my);
    v.markerVisible = inside && pose.marker != MarkerNone;
    return v;
}

}  // namespace es3map
