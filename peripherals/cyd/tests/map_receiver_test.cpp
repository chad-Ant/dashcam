#include "../lib/cydNavigation/MapReceiver.h"
#include <cassert>
#include <cstdio>
using namespace es3video;
static cyd::MapReceiver receiver;
static uint8_t wire[kMaxRecordBytes];
static const uint8_t sample[] = {0xff, 0xd8, 0xe3, 0x29, 0xff, 0xd9};
static es3map::StartMapRequest config() {
    es3map::StartMapRequest r;
    r.page = 5; r.viewportW = 320; r.viewportH = 224; r.maxJpegBytes = 32768;
    r.chunkBytes = 1024; r.targetFps = 1; r.assemblyTimeoutMs = 5000;
    r.staleAfterMs = 5000; r.placeholderAfterMs = 10000;
    return r;
}
static void drain() { cyd::MapReceiver::Event e; while (receiver.pop(e)) {} }
static bool event(uint8_t type, uint32_t frame, Reason reason = ReasonOk) {
    cyd::MapReceiver::Event e; bool found = false;
    while (receiver.pop(e)) {
        Record r; assert(decodeRecord(e.bytes, e.size, r) == ReasonOk);
        if (r.type == type && r.frameId == frame && (type != RecordResult || r.payload[0] == reason)) found = true;
    }
    return found;
}
static void deliver(size_t size, uint32_t now = 0) {
    Record r; assert(size && decodeRecord(wire, size, r) == ReasonOk); receiver.accept(r, now);
}
static void start(uint32_t frame, uint32_t bytes = sizeof(sample), uint32_t token = 7, uint32_t now = 0) {
    StartInfo i; i.width = 320; i.height = 224; i.jpegBytes = bytes;
    deliver(encodeStart(token, frame, i, wire, sizeof(wire)), now);
}
static void complete(uint32_t frame) {
    start(frame);
    deliver(encodeChunk(7, frame, 0, sample, sizeof(sample), wire, sizeof(wire)));
    deliver(encodeEnd(7, frame, crc32(sample, sizeof(sample)), wire, sizeof(wire)));
}
int main() {
    assert(validCapabilities(receiver.capabilities()));
    assert(es3map::validMapCapabilities(receiver.mapCapabilities(), receiver.capabilities()));
    auto r = config(); r.maxJpegBytes = 65536; assert(!receiver.start(7, r));
    r = config(); r.viewportH = 240; assert(!receiver.start(7, r));
    receiver.view(5, false); assert(receiver.start(7, config())); drain();
    complete(1); assert(receiver.phase == cyd::MapReceiver::Ready && receiver.grant == 1);
    assert(event(RecordResult, 1));
    start(2); assert(event(RecordResult, 2, ReasonNoCredit));
    assert(receiver.frame == 1 && receiver.phase == cyd::MapReceiver::Ready);
    receiver.finish(true, 123); assert(event(RecordPresented, 1)); assert(receiver.grant == 2);
    receiver.finish(true, 123); assert(receiver.grant == 2); // no duplicate credit
    start(2); deliver(encodeChunk(7, 2, 1, sample, 2, wire, sizeof(wire)));
    assert(event(RecordResult, 2, ReasonBadOffset)); assert(receiver.grant == 3);
    start(2); assert(event(RecordResult, 2, ReasonBadFrameId)); assert(receiver.grant == 3);
    start(3); deliver(encodeChunk(7, 3, 0, sample, sizeof(sample), wire, sizeof(wire)));
    deliver(encodeEnd(7, 3, 0, wire, sizeof(wire))); assert(event(RecordResult, 3, ReasonBadCrc));
    start(4, sizeof(sample), 7, UINT32_MAX - 1000); receiver.expire(3999);
    assert(event(RecordResult, 4, ReasonAssemblyTimeout));
    complete(5); receiver.view(0, false); assert(event(RecordResult, 5, ReasonHidden));
    receiver.view(5, true); start(6); assert(event(RecordResult, 6, ReasonHidden));
    receiver.view(5, false); complete(7); receiver.finish(false, 1);
    assert(event(RecordResult, 7, ReasonDecodeFailed));
    start(8, 32769); assert(event(RecordResult, 8, ReasonTooLarge));
    start(9); deliver(encodeAbort(7, 9, ReasonCancelled, wire, sizeof(wire)));
    assert(event(RecordResult, 9, ReasonCancelled));
    assert(receiver.start(8, config())); drain(); start(1, sizeof(sample), 7);
    assert(receiver.phase == cyd::MapReceiver::Idle && receiver.grant == 1);
    receiver.reset(); assert(receiver.token == 0 && receiver.phase == cyd::MapReceiver::Idle);
    // Fill the exact advertised limit, including a full final chunk.
    assert(receiver.start(7, config())); drain(); start(1, 32768);
    uint8_t chunk[1024]{}; uint32_t crc = 0;
    for (uint32_t off = 0; off < 32768; off += sizeof(chunk)) {
        deliver(encodeChunk(7, 1, off, chunk, sizeof(chunk), wire, sizeof(wire)));
        crc = crc32Update(crc, chunk, sizeof(chunk));
    }
    deliver(encodeEnd(7, 1, crc, wire, sizeof(wire)));
    assert(receiver.phase == cyd::MapReceiver::Ready);
    receiver.finish(true, 1); drain();
    for (unsigned i = 0; i < 20; ++i) receiver.view(i % 2 ? 5 : 0, false);
    assert(receiver.fault()); receiver.reset(); assert(!receiver.fault());
    puts("CYD map receiver: bounds, CRC, credits, visibility, timeout, reconnect passed");
}
