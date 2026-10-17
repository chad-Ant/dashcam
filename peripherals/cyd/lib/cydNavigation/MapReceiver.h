#pragma once
// Single-threaded, single-buffer adaptation of the ES3 receiver contract.
// Credit is returned after drawing, so the next JPEG cannot overwrite this one.
#include "Es3MapProtocol.h"
#include <string.h>

namespace cyd {
class MapReceiver {
public:
    static constexpr uint32_t kJpegBytes = 32768;
    static constexpr uint8_t kMapPage = 5;
    struct Event { uint8_t bytes[24]{}; size_t size = 0; };
    enum Phase { Idle, Receiving, Ready };
    uint8_t jpeg[kJpegBytes];
    es3video::StartVideoRequest config;
    uint32_t token = 0, grant = 0, frame = 0, bytes = 0;
    es3video::Demand demand = es3video::DemandCalibrating;
    Phase phase = Idle;

    static es3video::Capabilities capabilities() {
        es3video::Capabilities c;
        c.canvasWidth = c.maxWidth = 320;
        c.canvasHeight = 240; c.maxHeight = 224;
        c.maxJpegBytes = kJpegBytes; c.maxChunkBytes = 1024; c.maxCredits = 1;
        return c;
    }
    static es3map::MapCapabilities mapCapabilities() {
        es3map::MapCapabilities c;
        c.pageCount = 6; c.mapPage = kMapPage;
        c.viewportW = 320; c.viewportH = 224;
        c.maxJpegBytes = kJpegBytes; c.flags = es3map::kFlagMapTab;
        return c;
    }
    void reset() {
        token = grant = frame = bytes = _started = _lastFrame = 0;
        phase = Idle; _head = _count = 0; _fault = false;
    }
    bool start(uint32_t nextToken, const es3map::StartMapRequest& r) {
        if (!nextToken || !es3map::validStartMap(r, mapCapabilities(), capabilities())) return false;
        reset(); config = es3map::sessionLimits(r); token = nextToken; grant = 1;
        visibility(); return true;
    }
    void view(uint8_t page, bool blocked) {
        const auto d = blocked ? es3video::DemandCalibrating :
            (page == kMapPage ? es3video::DemandActive : es3video::DemandHidden);
        if (d == demand) return;
        demand = d;
        if (!token) return;
        visibility();
        if (d != es3video::DemandActive && phase != Idle) reject(es3video::ReasonHidden);
    }
    void expire(uint32_t now) {
        if (phase == Receiving && now - _since >= config.assemblyTimeoutMs)
            reject(es3video::ReasonAssemblyTimeout);
    }
    void accept(const es3video::Record& r, uint32_t now) {
        using namespace es3video;
        if (!token || r.sessionToken != token || !isHostToDevice(r.type)) return;
        expire(now);
        if (r.type == RecordStart) {
            if (r.frameId <= _lastFrame) { result(r.frameId, ReasonBadFrameId); return; }
            if (_started >= grant) { result(r.frameId, ReasonNoCredit); return; }
            _lastFrame = r.frameId; ++_started;
            StartInfo info;
            Reason reason = decodeStart(r, info);
            if (reason == ReasonOk) reason = checkStartForSession(info, config);
            if (reason == ReasonOk && demand != DemandActive) reason = ReasonHidden;
            if (reason == ReasonOk && phase != Idle) reason = ReasonBusy;
            if (reason != ReasonOk) { result(r.frameId, reason); credit(); return; }
            frame = r.frameId; bytes = info.jpegBytes; _received = _crc = 0;
            _since = now; phase = Receiving; return;
        }
        if (phase != Receiving || r.frameId != frame) return;
        if (r.type == RecordChunk) {
            ChunkInfo chunk;
            Reason reason = decodeChunk(r, chunk);
            if (reason == ReasonOk) reason = checkChunkForSession(chunk, config, _received, bytes);
            if (reason != ReasonOk) { reject(reason); return; }
            memcpy(jpeg + _received, chunk.data, chunk.size);
            _received += chunk.size; _crc = crc32Update(_crc, chunk.data, chunk.size);
        } else if (r.type == RecordEnd) {
            uint32_t crc;
            Reason reason = decodeEnd(r, crc);
            if (reason == ReasonOk && _received != bytes) reason = ReasonBadLength;
            if (reason == ReasonOk && crc != _crc) reason = ReasonBadCrc;
            if (reason != ReasonOk) { reject(reason); return; }
            phase = Ready; result(frame, ReasonOk);
        } else if (r.type == RecordAbort) {
            Reason reason = ReasonCancelled;
            const Reason parsed = decodeAbort(r, reason);
            reject(parsed == ReasonOk ? reason : parsed);
        }
    }
    void finish(bool ok, uint32_t durationUs) {
        if (phase != Ready) return;
        if (ok && demand == es3video::DemandActive) {
            es3video::PresentedInfo p; p.decodeUs = durationUs; // decode includes synchronous SPI writes
            Event e; e.size = es3video::encodePresented(token, frame, p, e.bytes, sizeof(e.bytes)); emit(e);
        } else result(frame, ok ? es3video::ReasonHidden : es3video::ReasonDecodeFailed);
        phase = Idle; credit();
    }
    bool pop(Event& e) {
        if (!_count) return false;
        e = _events[_head]; _head = (_head + 1) % 16; --_count; return true;
    }
    bool fault() const { return _fault; }
private:
    uint32_t _started = 0, _lastFrame = 0, _received = 0, _crc = 0, _since = 0;
    Event _events[16]; unsigned _head = 0, _count = 0;
    bool _fault = false;
    void emit(const Event& e) {
        if (!e.size || _count == 16) { _fault = true; return; }
        _events[(_head + _count) % 16] = e; ++_count;
    }
    void visibility() {
        Event e; e.size = es3video::encodeVisibility(token, demand, e.bytes, sizeof(e.bytes)); emit(e);
    }
    void result(uint32_t id, es3video::Reason reason) {
        Event e; e.size = es3video::encodeResult(token, id, reason, e.bytes, sizeof(e.bytes)); emit(e);
    }
    void credit() {
        if (grant == UINT32_MAX) { _fault = true; return; }
        Event e; e.size = es3video::encodeCredit(token, ++grant, e.bytes, sizeof(e.bytes)); emit(e);
    }
    void reject(es3video::Reason reason) { result(frame, reason); phase = Idle; credit(); }
};
}
