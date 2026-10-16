// Map extension v2 (board-side panning, Es3MapProtocol.h) against
// tests/pan_golden_vectors.txt: PAN_CAPABILITIES, START_PAN, the MAP_POSE
// record, background/chrome START classification and computePanView().
// Usage: pan_protocol_test <pan_golden_vectors.txt>
//        pan_protocol_test --print   (prints the vectors this header builds)
#include "../lib/es3UsbLink/Es3MapProtocol.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {

int g_fails = 0;
void check(bool ok, const std::string& what)
{
    if (!ok) {
        std::printf("FAIL %s\n", what.c_str());
        ++g_fails;
    }
}

std::string hex(const uint8_t* p, size_t n)
{
    static const char* d = "0123456789abcdef";
    std::string s;
    for (size_t i = 0; i < n; ++i) {
        s += d[p[i] >> 4];
        s += d[p[i] & 15];
    }
    return s;
}

std::vector<uint8_t> unhex(const std::string& s)
{
    std::vector<uint8_t> v;
    for (size_t i = 0; i + 1 < s.size(); i += 2) v.push_back(uint8_t(std::strtoul(s.substr(i, 2).c_str(), nullptr, 16)));
    return v;
}

es3video::Capabilities boardCaps()
{
    es3video::Capabilities c;
    c.canvasWidth = c.maxWidth = 320;
    c.canvasHeight = c.maxHeight = 240;
    c.maxJpegBytes = 65536;
    c.maxChunkBytes = 1024;
    c.maxCredits = 1;
    return c;
}

es3map::MapCapabilities mapCaps()
{
    es3map::MapCapabilities m;
    m.pageCount = 6;
    m.mapPage = 5;
    m.viewportW = 320;
    m.viewportH = 224;
    m.maxJpegBytes = 65536;
    m.flags = es3map::kFlagMapTab;
    return m;
}

es3map::PanCapabilities panCaps()
{
    es3map::PanCapabilities p;
    p.maxBackgroundW = 640;
    p.maxBackgroundH = 480;
    p.chromeTopH = 16;
    p.chromeBottomH = 16;
    p.maxPoseHz = 30;
    return p;
}

es3map::StartPanRequest startPan()
{
    const es3map::MapCapabilities m = mapCaps();
    es3map::StartPanRequest r;
    r.map.page = m.mapPage;
    r.map.viewportW = m.viewportW;
    r.map.viewportH = m.viewportH;
    r.map.maxJpegBytes = 65536;
    r.map.chunkBytes = 1024;
    r.map.targetFps = 2;
    r.map.assemblyTimeoutMs = 500;
    r.map.staleAfterMs = 2000;
    r.map.placeholderAfterMs = 5000;
    r.map.generation = 9;
    r.backgroundW = 512;
    r.backgroundH = 384;
    return r;
}

const uint32_t kToken = 0x2468ACE0u;

std::string control(uint8_t seq, uint8_t type, const uint8_t* payload, size_t n)
{
    uint8_t out[es3proto::kMaxFrame];
    const size_t len = es3proto::encode(seq, type, payload, n, out);
    return hex(out, len);
}

/// Raw START_PAN bytes with no validation, for the rejection vectors.
std::string startPanHex(const es3map::StartPanRequest& r)
{
    uint8_t p[es3map::kStartPanRequestBytes];
    const es3map::StartMapRequest& m = r.map;
    p[0] = m.mapProtocolVersion;
    p[1] = m.page;
    es3video::detail::put16(p + 2, m.viewportX);
    es3video::detail::put16(p + 4, m.viewportY);
    es3video::detail::put16(p + 6, m.viewportW);
    es3video::detail::put16(p + 8, m.viewportH);
    es3video::detail::put32(p + 10, m.maxJpegBytes);
    es3video::detail::put16(p + 14, m.chunkBytes);
    p[16] = m.targetFps;
    es3video::detail::put16(p + 17, m.assemblyTimeoutMs);
    es3video::detail::put16(p + 19, m.staleAfterMs);
    es3video::detail::put16(p + 21, m.placeholderAfterMs);
    es3video::detail::put32(p + 23, m.generation);
    es3video::detail::put16(p + 27, r.backgroundW);
    es3video::detail::put16(p + 29, r.backgroundH);
    return hex(p, sizeof(p));
}

std::string panCapsHex(const es3map::PanCapabilities& c)
{
    uint8_t p[es3map::kPanCapabilitiesReplyBytes];
    p[0] = c.panVersion;
    es3video::detail::put16(p + 1, c.maxBackgroundW);
    es3video::detail::put16(p + 3, c.maxBackgroundH);
    p[5] = c.chromeTopH;
    p[6] = c.chromeBottomH;
    p[7] = c.maxPoseHz;
    p[8] = c.flags;
    return hex(p, sizeof(p));
}

/// A raw MAP_POSE record with arbitrary fields (the encoder refuses bad ones).
std::string rawPose(uint32_t seq, uint32_t x, uint32_t y, uint16_t course, uint8_t marker)
{
    uint8_t payload[es3video::kMapPosePayloadBytes];
    es3video::detail::put32(payload, seq);
    es3video::detail::put32(payload + 4, x);
    es3video::detail::put32(payload + 8, y);
    es3video::detail::put16(payload + 12, course);
    payload[14] = marker;
    payload[15] = 0;
    uint8_t rec[64];
    rec[0] = es3video::kMagic0;
    rec[1] = es3video::kMagic1;
    rec[2] = es3video::kVersion;
    rec[3] = es3video::RecordMapPose;
    es3video::detail::put16(rec + 4, sizeof(payload));
    es3video::detail::put32(rec + 6, kToken);
    es3video::detail::put32(rec + 10, 0);
    std::memcpy(rec + 14, payload, sizeof(payload));
    const uint16_t crc = es3proto::crc16(rec + 2, 12 + sizeof(payload));
    es3video::detail::put16(rec + 14 + sizeof(payload), crc);
    return hex(rec, 16 + sizeof(payload));
}

es3map::MapPose pose()
{
    es3map::MapPose p;
    p.seq = 3;
    // Paris, zoom 16: world pixel (8601944.5, 5842437.25) x 16.
    p.xQ4 = 137631112u;
    p.yQ4 = 93478996u;
    p.courseCdeg = 27050;
    p.marker = es3map::MarkerArrow;
    return p;
}

typedef std::map<std::string, std::pair<std::string, std::string> > Vectors;

Vectors buildVectors()
{
    const es3video::Capabilities video = boardCaps();
    const es3map::MapCapabilities m = mapCaps();
    const es3map::PanCapabilities pc = panCaps();
    Vectors v;
    uint8_t p[64];
    const uint8_t badCommand = es3proto::BadCommand;

    v["ctl.pan_capabilities_request"] = std::make_pair(control(0x20, es3map::CmdPanCapabilities, nullptr, 0), "");
    size_t n = es3map::encodePanCapabilitiesReply(pc, m, video, p, sizeof(p));
    check(n == es3map::kPanCapabilitiesReplyBytes, "pan caps encode");
    v["ctl.pan_capabilities_reply"] =
        std::make_pair(control(0x20, es3map::CmdPanCapabilities | es3proto::Reply, p, n), "");
    v["ctl.pan_capabilities_unsupported"] = std::make_pair(control(0x20, es3proto::Error, &badCommand, 1), "");

    n = es3map::encodeStartPanRequest(startPan(), pc, m, video, p, sizeof(p));
    check(n == es3map::kStartPanRequestBytes, "start pan encode");
    v["ctl.start_pan_request"] = std::make_pair(control(0x21, es3map::CmdStartPan, p, n), "");
    es3map::StartPanReply reply;
    reply.sessionToken = kToken;
    reply.initialGrant = 1;
    n = es3map::encodeStartPanReply(reply, p, sizeof(p));
    v["ctl.start_pan_reply"] = std::make_pair(control(0x21, es3map::CmdStartPan | es3proto::Reply, p, n), "");

    uint8_t rec[es3video::kMaxRecordBytes];
    es3video::StartInfo info;
    info.width = 512;
    info.height = 384;
    info.jpegBytes = 48000;
    info.sourceTimestampUs = es3map::packOrigin(8601700u, 5842250u);
    n = es3video::encodeStart(kToken, 1, info, rec, sizeof(rec));
    v["rec.pan_background_start"] = std::make_pair(hex(rec, n), "");
    info.width = 320;
    info.height = 32;
    info.jpegBytes = 2500;
    info.sourceTimestampUs = 0;
    n = es3video::encodeStart(kToken, 2, info, rec, sizeof(rec));
    v["rec.pan_chrome_start"] = std::make_pair(hex(rec, n), "");
    n = es3map::encodeMapPose(kToken, pose(), rec, sizeof(rec));
    check(n == es3video::kOverheadBytes + es3video::kMapPosePayloadBytes, "pose encode");
    v["rec.map_pose"] = std::make_pair(hex(rec, n), "");
    es3map::MapPose dot = pose();
    dot.seq = 4;
    dot.marker = es3map::MarkerDot;
    dot.courseCdeg = 0;
    n = es3map::encodeMapPose(kToken, dot, rec, sizeof(rec));
    v["rec.map_pose_dot"] = std::make_pair(hex(rec, n), "");

    // Well framed, bad fields: the receiver ignores them.
    v["bad.map_pose_course"] = std::make_pair(rawPose(5, 16, 16, 36000, es3map::MarkerArrow), "ignore");
    v["bad.map_pose_marker"] = std::make_pair(rawPose(5, 16, 16, 0, 4), "ignore");
    v["bad.map_pose_seq_zero"] = std::make_pair(rawPose(0, 16, 16, 0, es3map::MarkerDot), "ignore");

    es3map::StartPanRequest r = startPan();
    r.backgroundW = 319;
    v["pay.start_pan_background_narrow"] = std::make_pair(startPanHex(r), "reject");
    r = startPan();
    r.backgroundH = 191;
    v["pay.start_pan_background_short"] = std::make_pair(startPanHex(r), "reject");
    r = startPan();
    r.backgroundW = 641;
    v["pay.start_pan_background_too_wide"] = std::make_pair(startPanHex(r), "reject");
    r = startPan();
    r.map.page = 0;
    v["pay.start_pan_wrong_page"] = std::make_pair(startPanHex(r), "reject");
    es3map::PanCapabilities c = panCaps();
    c.maxBackgroundH = 100;
    v["pay.pan_caps_background_below_map"] = std::make_pair(panCapsHex(c), "reject");
    c = panCaps();
    c.chromeTopH = 112;
    c.chromeBottomH = 112;
    v["pay.pan_caps_no_map_area"] = std::make_pair(panCapsHex(c), "reject");
    c = panCaps();
    c.maxPoseHz = 0;
    v["pay.pan_caps_pose_rate"] = std::make_pair(panCapsHex(c), "reject");
    c = panCaps();
    c.panVersion = 2;
    v["pay.pan_caps_version"] = std::make_pair(panCapsHex(c), "reject");
    return v;
}

Vectors readFile(const char* path)
{
    std::ifstream in(path);
    check(bool(in), "vector file opens");
    Vectors file;
    std::string line;
    while (std::getline(in, line)) {
        const size_t hash = line.find('#');
        if (hash != std::string::npos) line.erase(hash);
        std::istringstream ss(line);
        std::string name, bytes, extra;
        if (!(ss >> name)) continue;
        ss >> bytes >> extra;
        check(!file.count(name), "unique vector " + name);
        file[name] = std::make_pair(bytes, extra);
    }
    return file;
}

bool parseRecord(const std::string& h, es3video::Record& out, std::vector<uint8_t>& storage)
{
    storage = unhex(h);
    return es3video::decodeRecord(storage.data(), storage.size(), out) == es3video::ReasonOk;
}

void checkPanView()
{
    using es3map::computePanView;
    const uint64_t origin = es3map::packOrigin(1000, 2000);
    es3map::MapPose p;
    p.seq = 1;
    p.marker = es3map::MarkerArrow;
    // Centred: background 512x384, map 320x192; pose at the background centre.
    p.xQ4 = (1000 + 256) * 16;
    p.yQ4 = (2000 + 192) * 16;
    es3map::PanView v = computePanView(origin, 512, 384, 320, 192, p);
    check(v.cropX == 96 && v.cropY == 96 && v.markerXQ4 == 160 * 16 && v.markerYQ4 == 96 * 16 && v.markerVisible,
          "centred view");
    // Sub-pixel: half a pixel right moves the marker, not (yet) the crop.
    p.xQ4 += 8;
    v = computePanView(origin, 512, 384, 320, 192, p);
    check(v.cropX == 96 && v.markerXQ4 == 160 * 16 + 8, "sub-pixel marker");
    // Near the left/top edge: the crop clamps at 0 and the marker leaves the centre.
    p.xQ4 = (1000 + 40) * 16;
    p.yQ4 = (2000 + 10) * 16;
    v = computePanView(origin, 512, 384, 320, 192, p);
    check(v.cropX == 0 && v.cropY == 0 && v.markerXQ4 == 40 * 16 && v.markerYQ4 == 10 * 16 && v.markerVisible,
          "clamped at the top-left");
    // Past the right/bottom edge: clamp at the far side.
    p.xQ4 = (1000 + 600) * 16;
    p.yQ4 = (2000 + 380) * 16;
    v = computePanView(origin, 512, 384, 320, 192, p);
    check(v.cropX == 192 && v.cropY == 192 && v.markerXQ4 == (600 - 192) * 16 && !v.markerVisible,
          "clamped at the bottom-right, marker outside");
    // Left of the background (negative relative coordinates): floor, not truncation.
    p.xQ4 = 990 * 16 + 1;
    p.yQ4 = (2000 + 192) * 16;
    v = computePanView(origin, 512, 384, 320, 192, p);
    check(v.cropX == 0 && !v.markerVisible, "left of the background");
    // MarkerNone: the crop still follows, no marker.
    p.marker = es3map::MarkerNone;
    p.xQ4 = (1000 + 256) * 16;
    v = computePanView(origin, 512, 384, 320, 192, p);
    check(v.cropX == 96 && !v.markerVisible, "no marker");
    // A background exactly the map size never moves.
    p.marker = es3map::MarkerDot;
    p.xQ4 = (1000 + 160) * 16;
    p.yQ4 = (2000 + 96) * 16;
    v = computePanView(origin, 320, 192, 320, 192, p);
    check(v.cropX == 0 && v.cropY == 0 && v.markerVisible, "background = map area");
    // An odd map width: floor of (rel - w/2).
    p.xQ4 = (1000 + 100) * 16;
    v = computePanView(origin, 512, 384, 101, 192, p);
    check(v.cropX == 49 && v.markerXQ4 == 51 * 16, "odd map width");
    // Large zoom-19 coordinates do not overflow.
    const uint32_t big = 130000000u;
    p.xQ4 = (big + 256) * 16u;
    p.yQ4 = (big + 192) * 16u;
    v = computePanView(es3map::packOrigin(big, big), 512, 384, 320, 192, p);
    check(v.cropX == 96 && v.cropY == 96 && v.markerVisible, "zoom-19 coordinates");
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc == 2 && std::strcmp(argv[1], "--print") == 0) {
        const Vectors built = buildVectors();
        for (Vectors::const_iterator it = built.begin(); it != built.end(); ++it)
            std::printf("%s %s%s%s\n", it->first.c_str(), it->second.first.c_str(),
                        it->second.second.empty() ? "" : " ", it->second.second.c_str());
        return 0;
    }
    if (argc != 2) {
        std::printf("usage: pan_protocol_test <pan_golden_vectors.txt> | --print\n");
        return 2;
    }
    Vectors file = readFile(argv[1]);
    const Vectors built = buildVectors();
    check(file.size() == built.size(), "vector count " + std::to_string(file.size()) + " vs " +
                                           std::to_string(built.size()));
    for (Vectors::const_iterator it = built.begin(); it != built.end(); ++it) {
        const Vectors::const_iterator f = file.find(it->first);
        check(f != file.end() && f->second == it->second, "vector " + it->first + " = " + it->second.first);
    }

    const es3video::Capabilities video = boardCaps();
    const es3map::MapCapabilities m = mapCaps();
    const es3map::PanCapabilities pc = panCaps();

    // Control payloads decode back.
    {
        const std::vector<uint8_t> f = unhex(file["ctl.pan_capabilities_reply"].first);
        es3map::PanCapabilities got;
        check(f.size() == 8 + es3map::kPanCapabilitiesReplyBytes &&
                  es3map::decodePanCapabilitiesReply(f.data() + 6, f[5], m, video, got) && got.maxBackgroundW == 640 &&
                  got.maxBackgroundH == 480 && got.chromeTopH == 16 && got.chromeBottomH == 16 && got.maxPoseHz == 30,
              "pan caps decode");
        check(es3map::panMapHeight(m, got) == 192, "map area 192 px high");
        const std::vector<uint8_t> s = unhex(file["ctl.start_pan_request"].first);
        es3map::StartPanRequest r;
        check(s.size() == 8 + es3map::kStartPanRequestBytes &&
                  es3map::decodeStartPanRequest(s.data() + 6, s[5], pc, m, video, r) && r.backgroundW == 512 &&
                  r.backgroundH == 384 && r.map.generation == 9 && r.map.targetFps == 2,
              "start pan decode");
        // A START_PAN is not a START_MAP (length) and vice versa.
        es3map::StartMapRequest asMap;
        check(!es3map::decodeStartMapRequest(s.data() + 6, s[5], m, video, asMap), "START_PAN bytes are not START_MAP");
        es3map::StartPanReply reply;
        const std::vector<uint8_t> rp = unhex(file["ctl.start_pan_reply"].first);
        check(es3map::decodeStartPanReply(rp.data() + 6, rp[5], reply) && reply.sessionToken == kToken &&
                  reply.initialGrant == 1,
              "start pan reply decode");

        // START classification in the session.
        const es3map::PanSession sess = es3map::panSession(r, pc);
        check(sess.mapX == 0 && sess.mapY == 16 && sess.mapW == 320 && sess.mapH == 192, "session map area");
        es3video::Record rec;
        std::vector<uint8_t> buf;
        es3video::StartInfo info;
        es3map::PanImage kind = es3map::PanImageNone;
        check(parseRecord(file["rec.pan_background_start"].first, rec, buf) &&
                  es3video::decodeStart(rec, info) == es3video::ReasonOk &&
                  es3map::checkPanStart(info, sess, kind) == es3video::ReasonOk && kind == es3map::PanImageBackground &&
                  es3map::originX(info.sourceTimestampUs) == 8601700u &&
                  es3map::originY(info.sourceTimestampUs) == 5842250u,
              "background START");
        check(parseRecord(file["rec.pan_chrome_start"].first, rec, buf) &&
                  es3video::decodeStart(rec, info) == es3video::ReasonOk &&
                  es3map::checkPanStart(info, sess, kind) == es3video::ReasonOk && kind == es3map::PanImageChrome,
              "chrome START");
        info.width = 320;
        info.height = 224;   // a v1 whole frame is not a pan image
        check(es3map::checkPanStart(info, sess, kind) == es3video::ReasonBadDimensions && kind == es3map::PanImageNone,
              "whole frame refused in a pan session");
        info.width = 512;
        info.height = 384;
        info.jpegBytes = 65537;
        check(es3map::checkPanStart(info, sess, kind) == es3video::ReasonTooLarge, "background over the JPEG limit");
        info.jpegBytes = 0;
        check(es3map::checkPanStart(info, sess, kind) == es3video::ReasonBadLength, "empty background");
    }

    // Poses: the good ones decode, the bad ones are framed but malformed.
    {
        es3video::Record rec;
        std::vector<uint8_t> buf;
        es3map::MapPose got;
        check(parseRecord(file["rec.map_pose"].first, rec, buf) && rec.type == es3video::RecordMapPose &&
                  rec.frameId == 0 && rec.sessionToken == kToken &&
                  es3map::decodeMapPose(rec, got) == es3video::ReasonOk && got.seq == 3 && got.xQ4 == pose().xQ4 &&
                  got.yQ4 == pose().yQ4 && got.courseCdeg == 27050 && got.marker == es3map::MarkerArrow,
              "pose decode");
        check(parseRecord(file["rec.map_pose_dot"].first, rec, buf) &&
                  es3map::decodeMapPose(rec, got) == es3video::ReasonOk && got.marker == es3map::MarkerDot,
              "dot pose decode");
        for (Vectors::const_iterator it = file.begin(); it != file.end(); ++it) {
            if (it->first.compare(0, 4, "bad.") != 0) continue;
            check(it->second.second == "ignore", it->first + " marked ignore");
            check(parseRecord(it->second.first, rec, buf), it->first + " is well framed");
            check(es3map::decodeMapPose(rec, got) == es3video::ReasonMalformed, it->first + " malformed");
        }
        // The encoder refuses what the decoder rejects; frame_id must be 0.
        uint8_t out[64];
        es3map::MapPose bad = pose();
        bad.marker = 4;
        check(es3map::encodeMapPose(kToken, bad, out, sizeof(out)) == 0, "bad marker not encodable");
        bad = pose();
        bad.seq = 0;
        check(es3map::encodeMapPose(kToken, bad, out, sizeof(out)) == 0, "seq 0 not encodable");
        check(es3map::encodeMapPose(0, pose(), out, sizeof(out)) == 0, "zero token not encodable");
        uint8_t payload[es3video::kMapPosePayloadBytes] = {1};
        check(es3video::encodeRecord(es3video::RecordMapPose, kToken, 1, payload, sizeof(payload), out, sizeof(out)) == 0,
              "pose with a frame id not encodable");
        check(es3video::encodeRecord(es3video::RecordMapPose, kToken, 0, payload, 15, out, sizeof(out)) == 0,
              "short pose not encodable");
        check(es3video::isHostToDevice(es3video::RecordMapPose) && es3video::knownType(es3video::RecordMapPose) &&
                  !es3video::namesFrame(es3video::RecordMapPose) &&
                  std::strcmp(es3video::recordTypeName(es3video::RecordMapPose), "MAP_POSE") == 0,
              "MAP_POSE type predicates");
        // The MuxParser delivers it like any record.
        es3video::MuxParser parser(1024);
        const std::vector<uint8_t> wire = unhex(file["rec.map_pose"].first);
        int records = 0;
        size_t used = 0;
        es3video::MuxParser::Item item;
        do {
            used += parser.feed(wire.data() + used, wire.size() - used, 0, item);
            if (item.kind == es3video::MuxParser::ItemVideo && item.video.type == es3video::RecordMapPose) ++records;
        } while (item.kind != es3video::MuxParser::ItemNone);
        check(records == 1, "MuxParser yields one MAP_POSE record");
    }

    // Every pay.* is rejected and not encodable.
    for (Vectors::const_iterator it = file.begin(); it != file.end(); ++it) {
        if (it->first.compare(0, 4, "pay.") != 0) continue;
        const std::vector<uint8_t> b = unhex(it->second.first);
        check(it->second.second == "reject", it->first + " marked reject");
        uint8_t out[64];
        if (it->first.compare(0, 12, "pay.pan_caps") == 0) {
            es3map::PanCapabilities got;
            check(!es3map::decodePanCapabilitiesReply(b.data(), b.size(), m, video, got), it->first + " rejected");
            check(es3map::encodePanCapabilitiesReply(got, m, video, out, sizeof(out)) == 0, it->first + " not encodable");
        } else {
            es3map::StartPanRequest got;
            check(!es3map::decodeStartPanRequest(b.data(), b.size(), pc, m, video, got), it->first + " rejected");
            check(es3map::encodeStartPanRequest(got, pc, m, video, out, sizeof(out)) == 0, it->first + " not encodable");
        }
    }

    check(int(es3map::CmdPanCapabilities) > int(es3map::CmdStopMap) &&
              int(es3map::CmdStartPan) == int(es3map::CmdPanCapabilities) + 1,
          "pan command ids follow the v1 ones");
    checkPanView();

    if (g_fails) {
        std::printf("pan_protocol_test: %d failure(s)\n", g_fails);
        return 1;
    }
    std::printf("pan_protocol_test: all checks passed (%zu vectors)\n", file.size());
    return 0;
}
