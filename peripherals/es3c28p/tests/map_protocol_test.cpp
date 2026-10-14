// Map extension v1 (Es3MapProtocol.h) against tests/map_golden_vectors.txt.
// Builds every vector independently with the C++ header (the Python side
// builds them with host/es3_display.py), decodes them back, checks every
// rejection case, and that a map session's limits drive the video helpers.
// Usage: map_protocol_test <map_golden_vectors.txt>
#include "../lib/es3UsbLink/Es3MapProtocol.h"

#include <cstdio>
#include <cstdlib>
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

es3map::StartMapRequest startMap()
{
    const es3map::MapCapabilities m = mapCaps();
    es3map::StartMapRequest r;
    r.page = m.mapPage;
    r.viewportX = m.viewportX;
    r.viewportY = m.viewportY;
    r.viewportW = m.viewportW;
    r.viewportH = m.viewportH;
    r.maxJpegBytes = 65536;
    r.chunkBytes = 1024;
    r.targetFps = 4;
    r.assemblyTimeoutMs = 500;
    r.staleAfterMs = 2000;
    r.placeholderAfterMs = 5000;
    r.generation = 7;
    return r;
}

const uint32_t kToken = 0x2468ACE0u;

std::string control(uint8_t seq, uint8_t type, const uint8_t* payload, size_t n)
{
    uint8_t out[es3proto::kMaxFrame];
    const size_t len = es3proto::encode(seq, type, payload, n, out);
    return hex(out, len);
}

std::string payloadHex(const es3map::StartMapRequest& r)
{
    // Raw field encoding with no validation, for the rejection vectors.
    uint8_t p[es3map::kStartMapRequestBytes];
    p[0] = r.mapProtocolVersion;
    p[1] = r.page;
    es3video::detail::put16(p + 2, r.viewportX);
    es3video::detail::put16(p + 4, r.viewportY);
    es3video::detail::put16(p + 6, r.viewportW);
    es3video::detail::put16(p + 8, r.viewportH);
    es3video::detail::put32(p + 10, r.maxJpegBytes);
    es3video::detail::put16(p + 14, r.chunkBytes);
    p[16] = r.targetFps;
    es3video::detail::put16(p + 17, r.assemblyTimeoutMs);
    es3video::detail::put16(p + 19, r.staleAfterMs);
    es3video::detail::put16(p + 21, r.placeholderAfterMs);
    es3video::detail::put32(p + 23, r.generation);
    return hex(p, sizeof(p));
}

std::string capsHex(uint8_t pages, uint8_t page, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint32_t jpeg)
{
    uint8_t p[es3map::kMapCapabilitiesReplyBytes];
    p[0] = 1;
    p[1] = pages;
    p[2] = page;
    es3video::detail::put16(p + 3, x);
    es3video::detail::put16(p + 5, y);
    es3video::detail::put16(p + 7, w);
    es3video::detail::put16(p + 9, h);
    es3video::detail::put32(p + 11, jpeg);
    p[15] = es3map::kFlagMapTab;
    return hex(p, sizeof(p));
}

std::map<std::string, std::pair<std::string, std::string> > buildVectors()
{
    const es3video::Capabilities video = boardCaps();
    const es3map::MapCapabilities m = mapCaps();
    std::map<std::string, std::pair<std::string, std::string> > v;
    uint8_t p[64];
    const uint8_t badCommand = es3proto::BadCommand, notReady = es3proto::NotReady;

    v["ctl.map_capabilities_request"] = std::make_pair(control(0x10, es3map::CmdMapCapabilities, nullptr, 0), "");
    size_t n = es3map::encodeMapCapabilitiesReply(m, video, p, sizeof(p));
    check(n == es3map::kMapCapabilitiesReplyBytes, "map caps encode");
    v["ctl.map_capabilities_reply"] = std::make_pair(control(0x10, es3map::CmdMapCapabilities | es3proto::Reply, p, n), "");
    v["ctl.map_capabilities_unsupported"] = std::make_pair(control(0x10, es3proto::Error, &badCommand, 1), "");

    n = es3map::encodeStartMapRequest(startMap(), m, video, p, sizeof(p));
    check(n == es3map::kStartMapRequestBytes, "start map encode");
    v["ctl.start_map_request"] = std::make_pair(control(0x11, es3map::CmdStartMap, p, n), "");
    es3map::StartMapReply reply;
    reply.sessionToken = kToken;
    reply.initialGrant = 1;
    n = es3map::encodeStartMapReply(reply, p, sizeof(p));
    v["ctl.start_map_reply"] = std::make_pair(control(0x11, es3map::CmdStartMap | es3proto::Reply, p, n), "");
    v["ctl.start_map_busy"] = std::make_pair(control(0x11, es3proto::Error, &notReady, 1), "");
    n = es3map::encodeStopMapRequest(kToken, p, sizeof(p));
    v["ctl.stop_map_request"] = std::make_pair(control(0x12, es3map::CmdStopMap, p, n), "");
    v["ctl.stop_map_reply"] = std::make_pair(control(0x12, es3map::CmdStopMap | es3proto::Reply, nullptr, 0), "");

    es3video::StartInfo info;
    info.width = 320;
    info.height = 224;
    info.jpegBytes = 30000;
    info.sourceTimestampUs = 0x0102030405060708ull;
    uint8_t rec[es3video::kMaxRecordBytes];
    n = es3video::encodeStart(kToken, 1, info, rec, sizeof(rec));
    v["rec.map_start"] = std::make_pair(hex(rec, n), "");

    es3map::StartMapRequest r = startMap();
    r.page = 1;
    v["pay.start_map_wrong_page"] = std::make_pair(payloadHex(r), "reject");
    r = startMap();
    r.viewportY = 8;
    v["pay.start_map_wrong_viewport"] = std::make_pair(payloadHex(r), "reject");
    r = startMap();
    r.maxJpegBytes = 65537;
    v["pay.start_map_jpeg_too_large"] = std::make_pair(payloadHex(r), "reject");
    r = startMap();
    r.chunkBytes = 1025;
    v["pay.start_map_chunk_too_large"] = std::make_pair(payloadHex(r), "reject");
    r = startMap();
    r.placeholderAfterMs = 1999;
    v["pay.start_map_placeholder_before_stale"] = std::make_pair(payloadHex(r), "reject");
    r = startMap();
    r.mapProtocolVersion = 2;
    v["pay.start_map_version"] = std::make_pair(payloadHex(r), "reject");
    v["pay.map_caps_page_out_of_range"] = std::make_pair(capsHex(6, 6, 0, 0, 320, 224, 65536), "reject");
    v["pay.map_caps_viewport_off_canvas"] = std::make_pair(capsHex(6, 5, 0, 30, 320, 224, 65536), "reject");
    return v;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::printf("usage: map_protocol_test <map_golden_vectors.txt>\n");
        return 2;
    }
    std::ifstream in(argv[1]);
    check(bool(in), "vector file opens");
    std::map<std::string, std::pair<std::string, std::string> > file;
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
    const std::map<std::string, std::pair<std::string, std::string> > built = buildVectors();
    check(file.size() == built.size(), "vector count " + std::to_string(file.size()) + " vs " +
                                           std::to_string(built.size()));
    for (const auto& kv : built) {
        const auto it = file.find(kv.first);
        check(it != file.end() && it->second == kv.second, "vector " + kv.first + " = " + kv.second.first);
    }

    const es3video::Capabilities video = boardCaps();
    const es3map::MapCapabilities m = mapCaps();

    // Decode the valid control payloads back.
    {
        const std::vector<uint8_t> f = unhex(file["ctl.map_capabilities_reply"].first);
        es3map::MapCapabilities got;
        check(f.size() == 8 + es3map::kMapCapabilitiesReplyBytes &&
                  es3map::decodeMapCapabilitiesReply(f.data() + 6, f[5], video, got) && got.pageCount == 6 &&
                  got.mapPage == 5 && got.viewportW == 320 && got.viewportH == 224 && got.flags == 1,
              "map caps decode");
        const std::vector<uint8_t> s = unhex(file["ctl.start_map_request"].first);
        es3map::StartMapRequest r;
        check(es3map::decodeStartMapRequest(s.data() + 6, s[5], m, video, r) && r.generation == 7 &&
                  r.targetFps == 4 && r.placeholderAfterMs == 5000,
              "start map decode");
        const es3video::StartVideoRequest lim = es3map::sessionLimits(r);
        es3video::StartInfo info;
        info.width = 320;
        info.height = 224;
        info.jpegBytes = 30000;
        check(es3video::checkStartForSession(info, lim) == es3video::ReasonOk, "a 320x224 START fits a map session");
        info.height = 180;
        check(es3video::checkStartForSession(info, lim) != es3video::ReasonOk, "a 320x180 START does not");
        const std::vector<uint8_t> st = unhex(file["ctl.stop_map_request"].first);
        uint32_t token = 0;
        check(es3map::decodeStopMapRequest(st.data() + 6, st[5], token) && token == kToken, "stop map decode");
    }
    // Every pay.* is rejected; the encoder refuses to produce them too.
    for (const auto& kv : file) {
        if (kv.first.compare(0, 4, "pay.") != 0) continue;
        const std::vector<uint8_t> b = unhex(kv.second.first);
        check(kv.second.second == "reject", kv.first + " marked reject");
        if (kv.first.compare(0, 12, "pay.map_caps") == 0) {
            es3map::MapCapabilities got;
            check(!es3map::decodeMapCapabilitiesReply(b.data(), b.size(), video, got), kv.first + " rejected");
        } else {
            es3map::StartMapRequest got;
            check(!es3map::decodeStartMapRequest(b.data(), b.size(), m, video, got), kv.first + " rejected");
            uint8_t out[64];
            check(es3map::encodeStartMapRequest(got, m, video, out, sizeof(out)) == 0, kv.first + " not encodable");
        }
    }
    // Wrong lengths, invalid board video caps, zero tokens.
    {
        uint8_t p[64] = {};
        es3map::MapCapabilities got;
        check(!es3map::decodeMapCapabilitiesReply(p, es3map::kMapCapabilitiesReplyBytes - 1, video, got),
              "short map caps");
        es3video::Capabilities broken = video;
        broken.maxCredits = 0;
        check(es3map::encodeMapCapabilitiesReply(m, broken, p, sizeof(p)) == 0, "map caps need valid video caps");
        es3map::MapCapabilities big = m;
        big.maxJpegBytes = 65537;
        check(!es3map::validMapCapabilities(big, video), "map JPEG above the decoder limit");
        check(es3map::encodeStopMapRequest(0, p, sizeof(p)) == 0, "zero stop token");
        es3map::StartMapReply zero;
        zero.initialGrant = 1;
        check(es3map::encodeStartMapReply(zero, p, sizeof(p)) == 0, "zero session token");
        check(int(es3map::CmdMapCapabilities) != int(es3proto::Capabilities) &&
                  int(es3map::CmdStartMap) != int(es3proto::StartVideo) &&
                  int(es3map::CmdStopMap) != int(es3proto::StopVideo) && int(es3map::CmdMapCapabilities) > 9,
              "map commands do not collide with the video ones");
    }
    if (g_fails) {
        std::printf("map_protocol_test: %d failure(s)\n", g_fails);
        return 1;
    }
    std::printf("map_protocol_test: all checks passed (%zu vectors)\n", file.size());
    return 0;
}
