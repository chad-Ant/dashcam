#include "../lib/cydNavigation/cydNavigation.h"
#include <cassert>
#include <fstream>
#include <iterator>
#include <cstdio>
uint32_t testMillis = 0;
TestSerial Serial;
static CydGraphics graphics;
static CydAudio audio;
static CydNavigation link(graphics, audio);
static std::vector<es3video::MuxParser::Item> received;
static es3video::MuxParser parser;
static void pump(unsigned turns = 100) {
    for (unsigned i=0; i<turns; ++i) {
        link.update(); ++testMillis;
        for (uint8_t byte : Serial.output) {
            es3video::MuxParser::Item item;
            parser.feed(&byte, 1, testMillis, item);
            if (item.kind != es3video::MuxParser::ItemNone) received.push_back(item);
        }
        Serial.output.clear();
    }
}
static void feed(const uint8_t* data, size_t n) { Serial.input.insert(Serial.input.end(), data, data+n); }
static es3proto::Frame command(uint8_t type, const uint8_t* data = nullptr, size_t n = 0) {
    uint8_t wire[es3proto::kMaxFrame];
    received.clear(); feed(wire, es3proto::encode(1, type, data, n, wire)); pump();
    for (const auto& item : received) if (item.kind == es3video::MuxParser::ItemControl) return item.control;
    assert(false); return {};
}
int main(int argc, char** argv) {
    using namespace es3video;
    assert(argc == 2 && link.begin());
    auto f = command(es3proto::Status); assert(f.type == es3proto::Error && f.payload[0] == es3proto::NoSession);
    f = command(es3proto::Hello); assert(f.length == 3 && memcmp(f.payload,"CYD",3) == 0);
    f = command(es3proto::Capabilities); Capabilities caps;
    assert(decodeCapabilitiesReply(f.payload,f.length,caps) && caps.maxJpegBytes == 32768);
    f = command(es3map::CmdMapCapabilities); es3map::MapCapabilities map;
    assert(es3map::decodeMapCapabilitiesReply(f.payload,f.length,caps,map) && map.mapPage == 5);
    f = command(es3map::CmdPanCapabilities); assert(f.type == es3proto::Error && f.payload[0] == es3proto::BadCommand);
    f = command(es3proto::StartVideo); assert(f.type == es3proto::Error);
    es3map::StartMapRequest req;
    req.page = 5; req.viewportW = 320; req.viewportH = 224; req.chunkBytes = 1024;
    req.maxJpegBytes = 32768; req.targetFps = 1; req.assemblyTimeoutMs = 5000;
    req.staleAfterMs = 500; req.placeholderAfterMs = 1000;
    uint8_t control[es3proto::kMaxPayload];
    f = command(es3map::CmdStartMap,control,es3map::encodeStartMapRequest(req,map,caps,control,sizeof(control)));
    es3map::StartMapReply session; assert(es3map::decodeStartMapReply(f.payload,f.length,session));
    std::ifstream file(std::string(argv[1])+"/420.jpg", std::ios::binary);
    std::vector<uint8_t> data((std::istreambuf_iterator<char>(file)), {});
    uint8_t wire[kMaxRecordBytes]; StartInfo info;
    info.width=320; info.height=224; info.jpegBytes=data.size();
    received.clear();
    feed(wire,encodeStart(session.sessionToken,1,info,wire,sizeof(wire)));
    for (size_t off=0; off<data.size(); off+=1024)
        feed(wire,encodeChunk(session.sessionToken,1,off,data.data()+off,min<size_t>(1024,data.size()-off),wire,sizeof(wire)));
    feed(wire,encodeEnd(session.sessionToken,1,crc32(data.data(),data.size()),wire,sizeof(wire)));
    pump();
    unsigned results=0,presents=0,credits=0;
    for (const auto& item : received) if (item.kind == MuxParser::ItemVideo) {
        results += item.video.type == RecordResult;
        presents += item.video.type == RecordPresented;
        credits += item.video.type == RecordCredit;
    }
    assert(results==1 && presents==1 && credits==1);
    pump(600); assert(graphics.stale == 1);
    const auto placeholders = graphics.placeholders; pump(500); assert(graphics.placeholders > placeholders);
    graphics.page = 0; received.clear(); pump();
    bool hidden = false;
    for (const auto& item : received) if (item.kind == MuxParser::ItemVideo && item.video.type == RecordVisibility)
        hidden |= item.video.payload[0] == DemandHidden;
    assert(hidden);
    f=command(es3proto::Status); assert(f.length == 9 && f.payload[4] == 0);
    testMillis+=5001; pump(); f=command(es3proto::Ping);
    assert(f.type == es3proto::Error && f.payload[0] == es3proto::NoSession);
    f=command(es3proto::Hello); assert(f.length == 3); f=command(es3proto::Ping);
    assert(f.type == (es3proto::Ping | es3proto::Reply));
    puts("CYD real controller: handshake, fallback, partial writes, map presentation, stale/hidden, heartbeat recovery passed");
}
