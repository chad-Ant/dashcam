// Host-side tests of the portable video wire contract (Es3VideoProtocol.h):
// golden vectors shared with test_usb_client.py, typed codecs, control
// payloads and MuxParser recovery. Usage: video_protocol_test <vectors.txt>
#include "../lib/es3UsbLink/Es3VideoProtocol.h"
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <initializer_list>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using namespace es3video;
typedef std::vector<uint8_t> Bytes;

namespace {

int g_failures = 0;
#define CHECK(cond) expectTrue((cond), #cond, __LINE__)
void expectTrue(bool ok, const char* what, int line)
{
    if (ok) return;
    std::fprintf(stderr, "video_protocol_test:%d: CHECK failed: %s\n", line, what);
    ++g_failures;
}

const uint32_t kToken = 0x89abcdefu;

std::string hex(const Bytes& b)
{
    static const char* digits = "0123456789abcdef";
    std::string s;
    for (uint8_t v : b) { s += digits[v >> 4]; s += digits[v & 15]; }
    return s;
}

Bytes unhex(const std::string& s)
{
    Bytes b;
    for (size_t i = 0; i + 1 < s.size(); i += 2) b.push_back(uint8_t(std::stoul(s.substr(i, 2), nullptr, 16)));
    return b;
}

Bytes control(uint8_t seq, uint8_t type, const Bytes& payload = Bytes())
{
    uint8_t wire[es3proto::kMaxFrame];
    const size_t n = es3proto::encode(seq, type, payload.data(), payload.size(), wire);
    return Bytes(wire, wire + n);
}

/** Any header with a correct CRC: builds records the encoder refuses to make. */
Bytes rawRecord(uint8_t type, uint16_t length, uint32_t token, uint32_t frameId,
                const Bytes& payload, uint8_t version = kVersion)
{
    Bytes b(kHeaderBytes);
    b[0] = kMagic0; b[1] = kMagic1; b[2] = version; b[3] = type;
    detail::put16(&b[4], length);
    detail::put32(&b[6], token);
    detail::put32(&b[10], frameId);
    b.insert(b.end(), payload.begin(), payload.end());
    const uint16_t crc = es3proto::crc16(b.data() + 2, b.size() - 2);
    b.push_back(uint8_t(crc)); b.push_back(uint8_t(crc >> 8));
    return b;
}

template <typename Encode>
Bytes record(Encode encode)
{
    Bytes b(kMaxRecordBytes);
    b.resize(encode(b.data(), b.size()));
    return b;
}

Bytes chunkData()
{
    // JPEG-looking bytes with a complete control frame and a video header inside.
    Bytes d = {0xff, 0xd8};
    const Bytes ping = control(5, es3proto::Ping);
    d.insert(d.end(), ping.begin(), ping.end());
    const uint8_t tail[] = {0xe3, 0x29, 0x01, 0x02, 0xff, 0xd9};
    d.insert(d.end(), tail, tail + sizeof(tail));
    return d;
}

Capabilities deviceCaps()
{
    Capabilities c;
    c.canvasWidth = 320; c.canvasHeight = 240; c.maxWidth = 320; c.maxHeight = 240;
    c.maxJpegBytes = 65536; c.maxChunkBytes = 1024; c.maxCredits = 2;
    return c;
}

StartVideoRequest startRequest()
{
    StartVideoRequest r;
    r.viewportX = 0; r.viewportY = 30; r.viewportW = 320; r.viewportH = 180;
    r.maxJpegBytes = 65536; r.chunkBytes = 1024; r.targetFps = 10;
    r.assemblyTimeoutMs = 500; r.staleAfterMs = 500; r.placeholderAfterMs = 2000;
    return r;
}

Bytes startPayload(uint16_t w, uint16_t h, uint32_t jpeg, uint64_t ts)
{
    Bytes p(16);
    detail::put16(&p[0], w); detail::put16(&p[2], h);
    detail::put32(&p[4], jpeg); detail::put64(&p[8], ts);
    return p;
}

struct Vector { Bytes wire; std::string extra; };

/** Same names and field values as test_usb_client.py expected_vectors(). */
std::map<std::string, Vector> buildVectors()
{
    std::map<std::string, Vector> v;
    StartInfo s; s.width = 320; s.height = 180; s.jpegBytes = 8195; s.sourceTimestampUs = 0x8877665544332211ull;
    const Bytes start = record([&](uint8_t* o, size_t c) { return encodeStart(kToken, 1, s, o, c); });
    const Bytes data = chunkData();
    Bytes next; for (int i = 0xe0; i < 0xf0; ++i) next.push_back(uint8_t(i));
    PresentedInfo pr; pr.decodeUs = 12345; pr.blitUs = 23456;
    v["rec.start"].wire = start;
    v["rec.chunk_first"].wire = record([&](uint8_t* o, size_t c) { return encodeChunk(kToken, 1, 0, data.data(), data.size(), o, c); });
    v["rec.chunk_next"].wire = record([&](uint8_t* o, size_t c) { return encodeChunk(kToken, 1, 1024, next.data(), next.size(), o, c); });
    v["rec.end"].wire = record([&](uint8_t* o, size_t c) { return encodeEnd(kToken, 1, 0xcbf43926u, o, c); });
    v["rec.abort"].wire = record([&](uint8_t* o, size_t c) { return encodeAbort(kToken, 2, ReasonSuperseded, o, c); });
    v["rec.result_ok"].wire = record([&](uint8_t* o, size_t c) { return encodeResult(kToken, 1, ReasonOk, o, c); });
    v["rec.result_decode_failed"].wire = record([&](uint8_t* o, size_t c) { return encodeResult(kToken, 3, ReasonDecodeFailed, o, c); });
    v["rec.presented"].wire = record([&](uint8_t* o, size_t c) { return encodePresented(kToken, 1, pr, o, c); });
    v["rec.credit"].wire = record([&](uint8_t* o, size_t c) { return encodeCredit(kToken, 3, o, c); });
    v["rec.visibility"].wire = record([&](uint8_t* o, size_t c) { return encodeVisibility(kToken, DemandCalibrating, o, c); });

    uint8_t p[es3proto::kMaxPayload];
    const Bytes request = control(1, es3proto::Capabilities);
    v["ctl.capabilities_request"].wire = request;
    v["ctl.capabilities_reply"].wire = control(1, es3proto::Capabilities | es3proto::Reply,
        Bytes(p, p + encodeCapabilitiesReply(deviceCaps(), p, sizeof(p))));
    v["ctl.capabilities_unsupported"].wire = control(1, es3proto::Error, Bytes(1, es3proto::BadCommand));
    v["ctl.start_video_request"].wire = control(2, es3proto::StartVideo,
        Bytes(p, p + encodeStartVideoRequest(startRequest(), deviceCaps(), p, sizeof(p))));
    StartVideoReply reply; reply.sessionToken = kToken; reply.initialGrant = 1;
    v["ctl.start_video_reply"].wire = control(2, es3proto::StartVideo | es3proto::Reply,
        Bytes(p, p + encodeStartVideoReply(reply, p, sizeof(p))));
    v["ctl.stop_video_request"].wire = control(3, es3proto::StopVideo,
        Bytes(p, p + encodeStopVideoRequest(kToken, p, sizeof(p))));
    v["ctl.stop_video_reply"].wire = control(3, es3proto::StopVideo | es3proto::Reply);

    const std::string check = "123456789";
    v["crc32.check"].wire = Bytes(check.begin(), check.end());
    v["crc32.check"].extra = "cbf43926";
    for (int i = 0; i < 256; ++i) v["crc32.bytes_0_255"].wire.push_back(uint8_t(i));
    v["crc32.bytes_0_255"].extra = "29058c73";

    const Bytes credit3 = {3, 0, 0, 0};
    Bytes badCrc = start; badCrc.back() ^= 0xff;
    Bytes badControl = request; badControl.back() ^= 0xff;
    Bytes oversize = rawRecord(RecordChunk, 4 + 4097, kToken, 1, Bytes());
    oversize.resize(kHeaderBytes); oversize.insert(oversize.end(), 8, 0x55);
    Bytes pastCeiling(20, 0); detail::put32(&pastCeiling[0], 65530);
    v["bad.crc"] = Vector{badCrc, "drop"};
    v["bad.length"] = Vector{rawRecord(RecordStart, 15, kToken, 1, Bytes(15)), "drop"};
    v["bad.version"] = Vector{rawRecord(RecordCredit, 4, kToken, 0, credit3, 2), "drop"};
    v["bad.zero_token"] = Vector{rawRecord(RecordCredit, 4, 0, 0, credit3), "drop"};
    v["bad.frame_id_on_credit"] = Vector{rawRecord(RecordCredit, 4, kToken, 7, credit3), "drop"};
    v["bad.zero_frame_id"] = Vector{rawRecord(RecordEnd, 4, kToken, 0, Bytes(4)), "drop"};
    v["bad.unknown_type"] = Vector{rawRecord(0x05, 1, kToken, 1, Bytes(1)), "drop"};
    v["bad.empty_chunk"] = Vector{rawRecord(RecordChunk, 4, kToken, 1, Bytes(4)), "drop"};
    v["bad.oversize_chunk"] = Vector{oversize, "drop"};
    v["bad.control_crc"] = Vector{badControl, "drop"};
    v["bad.start_too_large"] = Vector{rawRecord(RecordStart, 16, kToken, 1, startPayload(320, 180, 65537, 0)), "TooLarge"};
    v["bad.start_zero_width"] = Vector{rawRecord(RecordStart, 16, kToken, 1, startPayload(0, 180, 8195, 0)), "BadDimensions"};
    v["bad.chunk_past_ceiling"] = Vector{rawRecord(RecordChunk, 20, kToken, 1, pastCeiling), "BadOffset"};
    v["bad.result_reason"] = Vector{rawRecord(RecordResult, 1, kToken, 1, Bytes(1, 0x7f)), "Malformed"};
    v["bad.visibility_demand"] = Vector{rawRecord(RecordVisibility, 1, kToken, 0, Bytes(1, 3)), "Malformed"};
    return v;
}

std::map<std::string, Vector> loadVectors(const char* path)
{
    std::map<std::string, Vector> v;
    std::ifstream in(path);
    if (!in) { std::fprintf(stderr, "cannot read %s\n", path); std::exit(2); }
    std::string line;
    for (int number = 1; std::getline(in, line); ++number) {
        line = line.substr(0, line.find('#'));
        std::istringstream fields(line);
        std::string name, wire, extra, more;
        if (!(fields >> name)) continue;
        fields >> wire >> extra >> more;
        if (wire.empty() || !more.empty() || v.count(name) || wire.size() % 2) {
            std::fprintf(stderr, "%s:%d: malformed or duplicate vector\n", path, number);
            std::exit(2);
        }
        v[name] = Vector{unhex(wire), extra};
    }
    return v;
}

struct Got {
    MuxParser::ItemKind kind;
    Bytes bytes;  // control: seq,type,payload; video: type,token,frame,payload
    bool operator==(const Got& o) const { return kind == o.kind && bytes == o.bytes; }
};

Got capture(const MuxParser::Item& item)
{
    Got g; g.kind = item.kind;
    if (item.kind == MuxParser::ItemControl) {
        g.bytes.push_back(item.control.sequence);
        g.bytes.push_back(item.control.type);
        g.bytes.insert(g.bytes.end(), item.control.payload, item.control.payload + item.control.length);
    } else {
        g.bytes.resize(9);
        g.bytes[0] = item.video.type;
        detail::put32(&g.bytes[1], item.video.sessionToken);
        detail::put32(&g.bytes[5], item.video.frameId);
        g.bytes.insert(g.bytes.end(), item.video.payload, item.video.payload + item.video.payloadLength);
    }
    return g;
}

/** Feeds @p data in pieces cut at @p cuts, collecting every item. */
std::vector<Got> feedAll(MuxParser& parser, const Bytes& data, const std::vector<size_t>& cuts,
                         uint32_t nowMs = 0)
{
    std::vector<Got> got;
    size_t begin = 0;
    for (size_t k = 0; k <= cuts.size(); ++k) {
        const size_t end = k < cuts.size() ? cuts[k] : data.size();
        size_t used = 0;
        const size_t size = end - begin;
        for (int guard = 0; guard < 100000; ++guard) {
            MuxParser::Item item;
            used += parser.feed(data.data() + begin + used, size - used, nowMs, item);
            if (item.kind == MuxParser::ItemNone) break;
            got.push_back(capture(item));
        }
        CHECK(used == size);
        begin = end;
    }
    return got;
}

std::vector<Got> feedOnce(MuxParser& parser, const Bytes& data, uint32_t nowMs = 0)
{
    return feedAll(parser, data, std::vector<size_t>(), nowMs);
}

Got videoItem(const Bytes& wire)
{
    Record r;
    CHECK(decodeRecord(wire.data(), wire.size(), r) == ReasonOk);
    MuxParser::Item item; item.kind = MuxParser::ItemVideo; item.video = r;
    return capture(item);
}

Bytes concat(std::initializer_list<Bytes> parts)
{
    Bytes all;
    for (const Bytes& p : parts) all.insert(all.end(), p.begin(), p.end());
    return all;
}

uint8_t reasonByName(const std::string& name)
{
    for (uint8_t r = 0; r < kReasonCount; ++r) if (name == reasonName(r)) return r;
    return 0xff;
}

void testGoldenVectors(const char* path)
{
    const std::map<std::string, Vector> built = buildVectors();
    const std::map<std::string, Vector> frozen = loadVectors(path);
    for (const auto& kv : built) {
        const auto it = frozen.find(kv.first);
        if (it == frozen.end() || it->second.wire != kv.second.wire || it->second.extra != kv.second.extra) {
            // Printed in file format: apply only for a deliberate protocol version change.
            std::fprintf(stderr, "vector mismatch, C++ builds: %s %s %s\n", kv.first.c_str(),
                         hex(kv.second.wire).c_str(), kv.second.extra.c_str());
            ++g_failures;
        }
    }
    for (const auto& kv : frozen) {
        if (!built.count(kv.first)) { std::fprintf(stderr, "unknown vector %s\n", kv.first.c_str()); ++g_failures; }
    }
    CHECK(frozen.size() == built.size());

    const Bytes valid = built.at("rec.credit").wire;
    for (const auto& kv : built) {
        const std::string& name = kv.first;
        const Bytes& wire = kv.second.wire;
        if (name.compare(0, 6, "crc32.") == 0) {
            char text[9];
            std::snprintf(text, sizeof(text), "%08x", unsigned(crc32(wire.data(), wire.size())));
            CHECK(kv.second.extra == text);
            continue;
        }
        MuxParser parser(kMaxChunkDataBytes);
        const std::vector<Got> got = feedOnce(parser, concat({wire, valid}));
        Record r;
        if (name.compare(0, 4, "rec.") == 0) {
            CHECK(decodeRecord(wire.data(), wire.size(), r) == ReasonOk);
            CHECK(checkRecord(r.type, r.sessionToken, r.frameId, r.payload, r.payloadLength) == ReasonOk);
            CHECK(got.size() == 2 && got[0] == videoItem(wire));
        } else if (name.compare(0, 4, "ctl.") == 0) {
            CHECK(got.size() == 2 && got[0].kind == MuxParser::ItemControl);
        } else if (kv.second.extra == "drop") {
            CHECK(got.size() == 1 && got[0] == videoItem(valid));
            if (wire[1] == kMagic1) CHECK(decodeRecord(wire.data(), wire.size(), r) != ReasonOk);
        } else {
            CHECK(decodeRecord(wire.data(), wire.size(), r) == ReasonOk);
            CHECK(got.size() == 2 && got[0] == videoItem(wire));
            const uint8_t expected = reasonByName(kv.second.extra);
            CHECK(expected != 0xff);
            CHECK(checkRecord(r.type, r.sessionToken, r.frameId, r.payload, r.payloadLength) == expected);
        }
        if (got.size() < 1 || !(got.back() == videoItem(valid))) {
            std::fprintf(stderr, "  (while checking %s)\n", name.c_str());
        }
    }
    // Explicit layout of one record, independent of the encoder.
    CHECK(hex(Bytes(valid.begin(), valid.begin() + 14)) == "e3290183" "0400" "efcdab89" "00000000");
}

void testTypedDecoders()
{
    const std::map<std::string, Vector> v = buildVectors();
    Record r;
    StartInfo s;
    CHECK(decodeRecord(v.at("rec.start").wire.data(), v.at("rec.start").wire.size(), r) == ReasonOk);
    CHECK(decodeStart(r, s) == ReasonOk);
    CHECK(s.width == 320 && s.height == 180 && s.jpegBytes == 8195 && s.sourceTimestampUs == 0x8877665544332211ull);
    ChunkInfo c;
    CHECK(decodeChunk(r, c) == ReasonMalformed);  // wrong helper for the type
    const Bytes& chunk = v.at("rec.chunk_first").wire;
    CHECK(decodeRecord(chunk.data(), chunk.size(), r) == ReasonOk);
    CHECK(decodeChunk(r, c) == ReasonOk && c.offset == 0 && Bytes(c.data, c.data + c.size) == chunkData());
    const Bytes& chunk2 = v.at("rec.chunk_next").wire;
    CHECK(decodeRecord(chunk2.data(), chunk2.size(), r) == ReasonOk);
    CHECK(decodeChunk(r, c) == ReasonOk && c.offset == 1024 && c.size == 16);
    uint32_t value = 0;
    CHECK(decodeRecord(v.at("rec.end").wire.data(), v.at("rec.end").wire.size(), r) == ReasonOk);
    CHECK(decodeEnd(r, value) == ReasonOk && value == 0xcbf43926u);
    Reason reason = ReasonOk;
    CHECK(decodeRecord(v.at("rec.abort").wire.data(), v.at("rec.abort").wire.size(), r) == ReasonOk);
    CHECK(decodeAbort(r, reason) == ReasonOk && reason == ReasonSuperseded && r.frameId == 2);
    CHECK(decodeRecord(v.at("rec.result_decode_failed").wire.data(), v.at("rec.result_decode_failed").wire.size(), r) == ReasonOk);
    CHECK(decodeResult(r, reason) == ReasonOk && reason == ReasonDecodeFailed && r.frameId == 3);
    PresentedInfo p;
    CHECK(decodeRecord(v.at("rec.presented").wire.data(), v.at("rec.presented").wire.size(), r) == ReasonOk);
    CHECK(decodePresented(r, p) == ReasonOk && p.decodeUs == 12345 && p.blitUs == 23456);
    CHECK(decodeRecord(v.at("rec.credit").wire.data(), v.at("rec.credit").wire.size(), r) == ReasonOk);
    CHECK(decodeCredit(r, value) == ReasonOk && value == 3 && r.frameId == 0);
    Demand demand = DemandActive;
    CHECK(decodeRecord(v.at("rec.visibility").wire.data(), v.at("rec.visibility").wire.size(), r) == ReasonOk);
    CHECK(decodeVisibility(r, demand) == ReasonOk && demand == DemandCalibrating);

    // Fields are still reported when a field check fails (for logging).
    const Bytes& big = v.at("bad.start_too_large").wire;
    CHECK(decodeRecord(big.data(), big.size(), r) == ReasonOk);
    CHECK(decodeStart(r, s) == ReasonTooLarge && s.jpegBytes == 65537);
    // decodeRecord framing reasons.
    const Bytes& start = v.at("rec.start").wire;
    CHECK(decodeRecord(start.data(), start.size() - 1, r) == ReasonBadLength);
    CHECK(decodeRecord(v.at("bad.crc").wire.data(), v.at("bad.crc").wire.size(), r) == ReasonBadCrc);
    CHECK(decodeRecord(v.at("bad.version").wire.data(), v.at("bad.version").wire.size(), r) == ReasonMalformed);
    CHECK(decodeRecord(v.at("bad.zero_token").wire.data(), v.at("bad.zero_token").wire.size(), r) == ReasonStaleSession);
    CHECK(decodeRecord(v.at("bad.zero_frame_id").wire.data(), v.at("bad.zero_frame_id").wire.size(), r) == ReasonBadFrameId);
    CHECK(decodeRecord(v.at("bad.oversize_chunk").wire.data(), v.at("bad.oversize_chunk").wire.size(), r) == ReasonTooLarge);
    CHECK(decodeRecord(nullptr, 0, r) == ReasonBadLength);
}

void testEncoderRefusals()
{
    uint8_t out[kMaxRecordBytes];
    uint8_t data[kMaxChunkDataBytes + 1] = {};
    StartInfo s; s.width = 320; s.height = 180; s.jpegBytes = 100;
    CHECK(encodeStart(kToken, 1, s, out, sizeof(out)) == kOverheadBytes + kStartPayloadBytes);
    CHECK(encodeStart(kToken, 1, s, out, kOverheadBytes + kStartPayloadBytes - 1) == 0);  // capacity
    CHECK(encodeStart(0, 1, s, out, sizeof(out)) == 0);           // zero token
    CHECK(encodeStart(kToken, 0, s, out, sizeof(out)) == 0);      // zero frame id
    s.height = 0;
    CHECK(encodeStart(kToken, 1, s, out, sizeof(out)) == 0);
    s.height = 180; s.jpegBytes = kMaxJpegBytes + 1;
    CHECK(encodeStart(kToken, 1, s, out, sizeof(out)) == 0);
    s.jpegBytes = 0;
    CHECK(encodeStart(kToken, 1, s, out, sizeof(out)) == 0);
    s.jpegBytes = 100;
    CHECK(encodeStart(kToken, 1, s, nullptr, sizeof(out)) == 0);
    CHECK(encodeChunk(kToken, 1, 0, data, 0, out, sizeof(out)) == 0);
    CHECK(encodeChunk(kToken, 1, 0, data, kMaxChunkDataBytes, out, sizeof(out)) == kMaxRecordBytes);
    CHECK(encodeChunk(kToken, 1, 0, data, kMaxChunkDataBytes + 1, out, sizeof(out)) == 0);
    CHECK(encodeChunk(kToken, 1, kMaxJpegBytes - 1, data, 2, out, sizeof(out)) == 0);
    CHECK(encodeChunk(kToken, 1, kMaxJpegBytes - 2, data, 2, out, sizeof(out)) != 0);
    CHECK(encodeChunk(kToken, 1, 0, nullptr, 4, out, sizeof(out)) == 0);
    CHECK(encodeCredit(0, 1, out, sizeof(out)) == 0);
    CHECK(encodeVisibility(kToken, static_cast<Demand>(3), out, sizeof(out)) == 0);
    CHECK(encodeResult(kToken, 1, static_cast<Reason>(kReasonCount), out, sizeof(out)) == 0);
    CHECK(encodeAbort(kToken, 0, ReasonCancelled, out, sizeof(out)) == 0);
    CHECK(encodeAbort(kToken, 1, ReasonOk, out, sizeof(out)) == 0);
    const uint8_t okReason = ReasonOk;
    Reason reason = ReasonCancelled;
    CHECK(decodeAbort(makeRecord(RecordAbort, kToken, 1, &okReason, 1), reason) == ReasonMalformed);
    CHECK(decodeResult(makeRecord(RecordResult, kToken, 1, &okReason, 1), reason) == ReasonOk);
    CHECK(encodeResult(kToken, 1, ReasonOk, out, sizeof(out)) != 0);
    CHECK(encodeRecord(RecordCredit, kToken, 1, data, 4, out, sizeof(out)) == 0);  // frame id on CREDIT
    CHECK(encodeRecord(0x05, kToken, 1, data, 1, out, sizeof(out)) == 0);
    CHECK(encodeRecord(RecordEnd, kToken, 1, data, 3, out, sizeof(out)) == 0);
    CHECK(encodeRecord(RecordEnd, kToken, 1, nullptr, 4, out, sizeof(out)) == 0);
    CHECK(checkRecord(RecordChunk, kToken, 1, data, kMaxRecordPayload + 1) == ReasonTooLarge);
}

void testCrc32()
{
    const uint8_t check[] = {'1','2','3','4','5','6','7','8','9'};
    CHECK(crc32(check, sizeof(check)) == 0xcbf43926u);
    for (size_t split = 0; split <= sizeof(check); ++split) {
        CHECK(crc32Update(crc32Update(0, check, split), check + split, sizeof(check) - split) == 0xcbf43926u);
    }
    CHECK(crc32(nullptr, 0) == 0);
    CHECK(crc32(check, 0) == 0);
}

void testControlPayloads()
{
    uint8_t p[es3proto::kMaxPayload];
    const Capabilities caps = deviceCaps();
    CHECK(validCapabilities(caps));
    CHECK(encodeCapabilitiesReply(caps, p, sizeof(p)) == kCapabilitiesReplyBytes);
    Capabilities back;
    CHECK(decodeCapabilitiesReply(p, kCapabilitiesReplyBytes, back));
    CHECK(back.maxChunkBytes == 1024 && back.maxCredits == 2 && back.canvasHeight == 240);
    for (size_t n = 0; n < sizeof(p); ++n) {
        if (n != kCapabilitiesReplyBytes) CHECK(!decodeCapabilitiesReply(p, n, back));
    }
    CHECK(!decodeCapabilitiesReply(nullptr, kCapabilitiesReplyBytes, back));
    p[0] = 2;  // a newer device: rejected, but reported
    CHECK(!decodeCapabilitiesReply(p, kCapabilitiesReplyBytes, back) && back.videoProtocolVersion == 2);
    {
        Capabilities c = caps; c.maxCredits = 3; CHECK(!validCapabilities(c));
        c = caps; c.maxCredits = 0; CHECK(!validCapabilities(c));
        c = caps; c.maxWidth = 321; CHECK(!validCapabilities(c));
        c = caps; c.maxChunkBytes = 4097; CHECK(!validCapabilities(c));
        c = caps; c.maxJpegBytes = 65537; CHECK(!validCapabilities(c));
        c = caps; c.codecMask = 2; CHECK(!validCapabilities(c));
        c = caps; c.flags = 0xff; CHECK(validCapabilities(c));  // reserved: ignored
        CHECK(encodeCapabilitiesReply(caps, p, kCapabilitiesReplyBytes - 1) == 0);
    }

    const StartVideoRequest good = startRequest();
    CHECK(validStartVideo(good, caps));
    CHECK(encodeStartVideoRequest(good, caps, p, sizeof(p)) == kStartVideoRequestBytes);
    StartVideoRequest req;
    CHECK(decodeStartVideoRequest(p, kStartVideoRequestBytes, caps, req));
    CHECK(req.viewportY == 30 && req.viewportH == 180 && req.placeholderAfterMs == 2000 && req.targetFps == 10);
    for (size_t n = 0; n < sizeof(p); ++n) {
        if (n != kStartVideoRequestBytes) CHECK(!decodeStartVideoRequest(p, n, caps, req));
    }
    struct Change { const char* what; void (*apply)(StartVideoRequest&); };
    const Change changes[] = {
        {"zero width", [](StartVideoRequest& r) { r.viewportW = 0; }},
        {"zero height", [](StartVideoRequest& r) { r.viewportH = 0; }},
        {"below canvas", [](StartVideoRequest& r) { r.viewportY = 61; }},
        {"right of canvas", [](StartVideoRequest& r) { r.viewportX = 1; }},
        {"huge x", [](StartVideoRequest& r) { r.viewportX = 0xffff; }},
        {"chunk above device", [](StartVideoRequest& r) { r.chunkBytes = 1025; }},
        {"zero chunk", [](StartVideoRequest& r) { r.chunkBytes = 0; }},
        {"zero jpeg", [](StartVideoRequest& r) { r.maxJpegBytes = 0; }},
        {"jpeg above device", [](StartVideoRequest& r) { r.maxJpegBytes = 65537; }},
        {"zero fps", [](StartVideoRequest& r) { r.targetFps = 0; }},
        {"fps above ceiling", [](StartVideoRequest& r) { r.targetFps = kMaxTargetFps + 1; }},
        {"placeholder before stale", [](StartVideoRequest& r) { r.placeholderAfterMs = 499; }},
        {"zero assembly timeout", [](StartVideoRequest& r) { r.assemblyTimeoutMs = 0; }},
        {"timeout above ceiling", [](StartVideoRequest& r) { r.placeholderAfterMs = kMaxTimeoutMs + 1; }},
        {"unadvertised codec", [](StartVideoRequest& r) { r.codec = 1; }},
        {"wrong version", [](StartVideoRequest& r) { r.videoProtocolVersion = 2; }},
    };
    for (const Change& change : changes) {
        StartVideoRequest bad = good;
        change.apply(bad);
        if (validStartVideo(bad, caps) || encodeStartVideoRequest(bad, caps, p, sizeof(p))) {
            std::fprintf(stderr, "START_VIDEO accepted: %s\n", change.what);
            ++g_failures;
        }
    }
    Capabilities broken = caps; broken.maxCredits = 0;
    CHECK(!validStartVideo(good, broken));
    // The device decodes against its own limits: a smaller device refuses.
    Capabilities small = caps; small.maxWidth = 160; small.maxHeight = 120;
    CHECK(encodeStartVideoRequest(good, caps, p, sizeof(p)) == kStartVideoRequestBytes);
    CHECK(!decodeStartVideoRequest(p, kStartVideoRequestBytes, small, req));

    StartVideoReply reply; reply.sessionToken = kToken; reply.initialGrant = 2;
    CHECK(encodeStartVideoReply(reply, p, sizeof(p)) == kStartVideoReplyBytes);
    StartVideoReply r2;
    CHECK(decodeStartVideoReply(p, kStartVideoReplyBytes, r2) && r2.sessionToken == kToken && r2.initialGrant == 2);
    CHECK(!decodeStartVideoReply(p, kStartVideoReplyBytes - 1, r2));
    reply.sessionToken = 0; CHECK(encodeStartVideoReply(reply, p, sizeof(p)) == 0);
    reply.sessionToken = kToken; reply.initialGrant = 0; CHECK(encodeStartVideoReply(reply, p, sizeof(p)) == 0);
    reply.initialGrant = 3; CHECK(encodeStartVideoReply(reply, p, sizeof(p)) == 0);
    detail::put32(p, 0); detail::put32(p + 4, 1);
    CHECK(!decodeStartVideoReply(p, kStartVideoReplyBytes, r2));

    uint32_t token = 0;
    CHECK(encodeStopVideoRequest(kToken, p, sizeof(p)) == kStopVideoRequestBytes);
    CHECK(decodeStopVideoRequest(p, kStopVideoRequestBytes, token) && token == kToken);
    CHECK(!decodeStopVideoRequest(p, 3, token) && !decodeStopVideoRequest(p, 5, token));
    CHECK(encodeStopVideoRequest(0, p, sizeof(p)) == 0);
    detail::put32(p, 0);
    CHECK(!decodeStopVideoRequest(p, kStopVideoRequestBytes, token));

    // Command ids added to the control protocol.
    CHECK(es3proto::Capabilities == 7 && es3proto::StartVideo == 8 && es3proto::StopVideo == 9);
}

void testSessionChecks()
{
    const StartVideoRequest session = startRequest();
    StartInfo s; s.width = 320; s.height = 180; s.jpegBytes = 8000;
    CHECK(checkStartForSession(s, session) == ReasonOk);
    s.height = 240; CHECK(checkStartForSession(s, session) == ReasonBadDimensions);
    s.height = 180; s.jpegBytes = 65537; CHECK(checkStartForSession(s, session) == ReasonTooLarge);
    s.jpegBytes = 0; CHECK(checkStartForSession(s, session) == ReasonBadLength);
    uint8_t data[2048] = {};
    ChunkInfo c; c.offset = 1024; c.data = data; c.size = 1024;
    CHECK(checkChunkForSession(c, session, 1024, 8000) == ReasonOk);
    CHECK(checkChunkForSession(c, session, 0, 8000) == ReasonBadOffset);
    CHECK(checkChunkForSession(c, session, 1024, 1500) == ReasonBadLength);
    c.size = 1025; CHECK(checkChunkForSession(c, session, 1024, 8000) == ReasonTooLarge);
    CHECK(std::string(reasonName(ReasonInternal)) == "Internal" && std::string(reasonName(200)) == "Unknown");
    CHECK(std::string(recordTypeName(RecordVisibility)) == "VISIBILITY");
}

/** All valid rec./ctl. vectors back to back. */
Bytes validStream(std::vector<Got>& expected)
{
    const std::map<std::string, Vector> v = buildVectors();
    Bytes all;
    for (const auto& kv : v) {
        if (kv.first.compare(0, 4, "rec.") && kv.first.compare(0, 4, "ctl.")) continue;
        all.insert(all.end(), kv.second.wire.begin(), kv.second.wire.end());
        MuxParser p;
        const std::vector<Got> one = feedOnce(p, kv.second.wire);
        CHECK(one.size() == 1);
        expected.insert(expected.end(), one.begin(), one.end());
    }
    return all;
}

void testMuxSplitAndCoalesced()
{
    std::vector<Got> expected;
    const Bytes stream = validStream(expected);
    CHECK(expected.size() == 17);
    MuxParser whole;
    CHECK(feedOnce(whole, stream) == expected);
    // Every single split point, and every pair of split points around the
    // largest record, must yield the same items.
    for (size_t cut = 0; cut <= stream.size(); ++cut) {
        MuxParser p;
        if (!(feedAll(p, stream, std::vector<size_t>(1, cut)) == expected)) {
            std::fprintf(stderr, "split at %zu differs\n", cut);
            ++g_failures;
        }
    }
    std::vector<size_t> bytewise;
    for (size_t i = 1; i < stream.size(); ++i) bytewise.push_back(i);
    MuxParser p;
    CHECK(feedAll(p, stream, bytewise) == expected);
    // The PING frame and E3 29 header inside CHUNK data are never items.
    for (const Got& g : expected) CHECK(!(g.kind == MuxParser::ItemControl && g.bytes[1] == es3proto::Ping));
    CHECK(p.stats().crcErrors == 0 && p.stats().headerErrors == 0 && p.stats().garbageBytes == 0);
}

void testMuxEmbeddedFraming()
{
    // A CHUNK whose data is a complete control frame followed by a complete
    // video record: exactly one item (the CHUNK) comes out.
    const std::map<std::string, Vector> v = buildVectors();
    const Bytes inner = concat({control(9, es3proto::StopAudio), v.at("rec.credit").wire,
                                v.at("rec.start").wire});
    uint8_t out[kMaxRecordBytes];
    const size_t n = encodeChunk(kToken, 4, 0, inner.data(), inner.size(), out, sizeof(out));
    CHECK(n != 0);
    const Bytes chunk(out, out + n);
    for (size_t cut = 0; cut <= chunk.size(); ++cut) {
        MuxParser p;
        const std::vector<Got> got = feedAll(p, chunk, std::vector<size_t>(1, cut));
        CHECK(got.size() == 1 && got[0] == videoItem(chunk));
    }
}

void testMuxGarbageAndRecovery()
{
    const std::map<std::string, Vector> v = buildVectors();
    const Bytes& credit = v.at("rec.credit").wire;
    const Bytes& start = v.at("rec.start").wire;
    const Bytes ping = control(9, es3proto::Ping);
    const Bytes noise = {0x00, 0xe3, 0xe3, 0x28, 0xe3, 0x29, 0x01, 0xe3, 0x29, 0x02, 0xff, 0x00, 0xe3, 0x28, 0x02};
    {
        MuxParser p;
        const std::vector<Got> got = feedOnce(p, concat({noise, Bytes(start.begin(), start.begin() + 20),
                                                         credit, {'j', 'u', 'n', 'k', 0xe3}, ping, start}));
        CHECK(got.size() == 3 && got[0] == videoItem(credit) && got[2] == videoItem(start));
        CHECK(got.size() == 3 && got[1].kind == MuxParser::ItemControl);
        CHECK(p.stats().headerErrors > 0 && p.stats().crcErrors >= 1 && p.stats().garbageBytes > 0);
    }
    {
        // A false header whose declared length covers later records: when its
        // CRC finally fails, the rescan recovers them in order.
        Bytes falseChunk = rawRecord(RecordChunk, 4 + 1000, kToken, 1, Bytes());
        falseChunk.resize(kHeaderBytes);
        MuxParser p;
        const std::vector<Got> got = feedOnce(p, concat({falseChunk, credit, ping, Bytes(1100, 0), credit}));
        CHECK(got.size() == 3 && got[0] == videoItem(credit) && got[1].kind == MuxParser::ItemControl &&
              got[2] == videoItem(credit));
        CHECK(p.stats().crcErrors == 1);
    }
    {
        // Truncated record without a gap: the next record's bytes complete the
        // candidate, its CRC fails, and the rescan still finds the record.
        MuxParser p;
        CHECK(feedOnce(p, concat({Bytes(start.begin(), start.begin() + 20), start})).size() == 1);
    }
    {
        // Truncated record + timeout (also across millis() rollover).
        MuxParser p;
        CHECK(feedOnce(p, Bytes(start.begin(), start.begin() + 10), 0xfffffff0u).empty());
        CHECK(p.buffered() == 10);
        const std::vector<Got> got = feedOnce(p, credit, 0xfffffff0u + kPartialTimeoutMs + 1);
        CHECK(got.size() == 1 && got[0] == videoItem(credit) && p.stats().timeouts == 1);
        // Exactly at the timeout the partial record survives.
        MuxParser q;
        CHECK(feedOnce(q, Bytes(start.begin(), start.begin() + 10), 100).empty());
        CHECK(feedOnce(q, Bytes(start.begin() + 10, start.end()), 100 + kPartialTimeoutMs).size() == 1);
        // expire() drops an idle partial record without new bytes.
        MuxParser e;
        CHECK(feedOnce(e, Bytes(start.begin(), start.begin() + 10), 0).empty());
        CHECK(!e.expire(kPartialTimeoutMs) && e.expire(kPartialTimeoutMs + 1) && e.buffered() == 0);
        CHECK(e.stats().timeouts == 1);
    }
    {
        // A stalled long CHUNK hides complete records (including a CHUNK with
        // embedded magic), another stalled header, and an incomplete suffix.
        Bytes stalled = rawRecord(RecordChunk, 1004, kToken, 1, Bytes());
        stalled.resize(kHeaderBytes);
        const Bytes nested = concat({ping, credit});
        const Bytes chunk = record([&](uint8_t* o, size_t c) {
            return encodeChunk(kToken, 9, 0, nested.data(), nested.size(), o, c);
        });
        const Bytes suffix(start.begin(), start.begin() + 10);
        const Bytes buffered = concat({stalled, credit, stalled, chunk, ping, suffix});
        for (int mode = 0; mode < 3; ++mode) {
            MuxParser p;
            const uint32_t then = 0xfffffff0u;
            const uint32_t now = then + kPartialTimeoutMs + 1;
            CHECK(feedOnce(p, buffered, then).empty());
            if (mode == 0) {
                CHECK(p.expire(now));
                CHECK(!p.expire(now)); // preserved complete records do not expire
            }
            const Bytes incoming = mode == 2 ? start : Bytes();
            const auto got = feedOnce(p, incoming, now);
            CHECK(got.size() == (mode == 2 ? 4u : 3u));
            CHECK(got.size() >= 3 && got[0] == videoItem(credit) && got[1] == videoItem(chunk) &&
                  got[2].kind == MuxParser::ItemControl);
            if (mode == 2) CHECK(got.size() == 4 && got[3] == videoItem(start));
            CHECK(p.stats().timeouts == 1 && p.buffered() == 0);
        }
    }
    {
        // Oversize: rejected from the length field, then the next record parses.
        MuxParser p;  // initial configuration: 1024 data bytes
        CHECK(p.maxChunkData() == kDefaultChunkDataBytes);
        Bytes data(1025, 0xe3);
        uint8_t out[kMaxRecordBytes];
        const size_t n = encodeChunk(kToken, 1, 0, data.data(), data.size(), out, sizeof(out));
        const Bytes big(out, out + n);
        std::vector<Got> got = feedOnce(p, concat({big, credit}));
        CHECK(got.size() == 1 && got[0] == videoItem(credit) && p.stats().oversize == 1);
        MuxParser wide(kMaxChunkDataBytes);
        got = feedOnce(wide, big);
        CHECK(got.size() == 1 && got[0] == videoItem(big));
        CHECK(MuxParser(0).maxChunkData() == 1 && MuxParser(100000).maxChunkData() == kMaxChunkDataBytes);
        // The largest possible record and the largest control frame.
        Bytes max(kMaxChunkDataBytes, 0x29);
        const size_t m = encodeChunk(kToken, 1, 0, max.data(), max.size(), out, sizeof(out));
        CHECK(m == kMaxRecordBytes);
        got = feedOnce(wide, concat({Bytes(out, out + m), control(1, es3proto::Page, Bytes(es3proto::kMaxPayload, 0xe3))}));
        CHECK(got.size() == 2 && got[0].bytes.size() == 9 + kMaxRecordPayload &&
              got[1].bytes.size() == 2 + es3proto::kMaxPayload);
    }
    {
        // Wrong version / zero token / frame-id rule / unknown type / bad
        // control version: all dropped, the following record survives.
        MuxParser p;
        const Bytes control2 = {0xe3, 0x28, 0x02, 0x01, 0x01, 0x00, 0x00, 0x00};
        const std::vector<Got> got = feedOnce(p, concat({v.at("bad.version").wire, v.at("bad.zero_token").wire,
                                                         v.at("bad.frame_id_on_credit").wire,
                                                         v.at("bad.zero_frame_id").wire,
                                                         v.at("bad.unknown_type").wire, control2, credit}));
        CHECK(got.size() == 1 && got[0] == videoItem(credit));
        CHECK(p.stats().headerErrors == 6);
    }
    {
        // reset() drops a partial record and the last item's view; stats survive.
        MuxParser p;
        CHECK(feedOnce(p, Bytes(start.begin(), start.begin() + 5)).empty());
        p.reset();
        CHECK(p.buffered() == 0 && feedOnce(p, credit).size() == 1 && p.stats().videoRecords == 1);
        p.resetStats();
        CHECK(p.stats().videoRecords == 0);
    }
}

void testMuxRandomized()
{
    std::mt19937 rng(20261013u);
    const std::map<std::string, Vector> v = buildVectors();
    std::vector<Bytes> pool;
    for (const auto& kv : v) {
        if (!kv.first.compare(0, 4, "rec.") || !kv.first.compare(0, 4, "ctl.")) pool.push_back(kv.second.wire);
    }
    for (int round = 0; round < 300; ++round) {
        // Valid items separated by garbage that contains no sync byte: every
        // item must come out, whatever the delivery pattern.
        Bytes stream;
        std::vector<Got> expected;
        const int count = 1 + int(rng() % 12);
        for (int i = 0; i < count; ++i) {
            const size_t gap = rng() % 6;
            for (size_t g = 0; g < gap; ++g) {
                uint8_t b = uint8_t(rng());
                stream.push_back(b == kMagic0 ? 0 : b);
            }
            Bytes item;
            if (rng() % 3 == 0) {
                Bytes data(1 + rng() % kDefaultChunkDataBytes);
                for (uint8_t& b : data) b = uint8_t(rng() % 4 ? rng() : kMagic0);
                uint8_t out[kMaxRecordBytes];
                item.assign(out, out + encodeChunk(kToken, 1 + rng() % 100, rng() % 60000,
                                                   data.data(), data.size(), out, sizeof(out)));
            } else {
                item = pool[rng() % pool.size()];
            }
            CHECK(!item.empty());
            stream.insert(stream.end(), item.begin(), item.end());
            MuxParser single(kMaxChunkDataBytes);
            const std::vector<Got> one = feedOnce(single, item);
            expected.insert(expected.end(), one.begin(), one.end());
        }
        std::vector<size_t> cuts;
        for (size_t at = 0; at < stream.size(); at += 1 + rng() % 300) cuts.push_back(at);
        MuxParser p;
        if (!(feedAll(p, stream, cuts) == expected)) {
            std::fprintf(stderr, "randomized round %d lost items\n", round);
            ++g_failures;
        }
    }
    // Pure noise biased towards sync bytes: never crashes, never stalls, and
    // anything it yields passed the CRC (so stays rare).
    MuxParser p(kMaxChunkDataBytes);
    Bytes noise(200000);
    const uint8_t biased[] = {0xe3, 0x28, 0x29, 0x01, 0x02, 0x83, 0x00};
    for (uint8_t& b : noise) b = rng() % 2 ? biased[rng() % sizeof(biased)] : uint8_t(rng());
    std::vector<size_t> cuts;
    for (size_t at = 0; at < noise.size(); at += 1 + rng() % 2000) cuts.push_back(at);
    const std::vector<Got> got = feedAll(p, noise, cuts);
    CHECK(got.size() < 10);
    CHECK(p.buffered() <= kMaxRecordBytes);
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::fprintf(stderr, "usage: %s <video_golden_vectors.txt>\n", argv[0]);
        return 2;
    }
    testGoldenVectors(argv[1]);
    testTypedDecoders();
    testEncoderRefusals();
    testCrc32();
    testControlPayloads();
    testSessionChecks();
    testMuxSplitAndCoalesced();
    testMuxEmbeddedFraming();
    testMuxGarbageAndRecovery();
    testMuxRandomized();
    if (g_failures) {
        std::fprintf(stderr, "video_protocol_test: %d failure(s)\n", g_failures);
        return 1;
    }
    std::puts("video_protocol_test: all checks passed");
    return 0;
}
