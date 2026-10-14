#pragma once
// Portable ownership/assembly engine. Caller serializes every method; decode
// and TFT operations occur OUTSIDE that lock while holding the returned lease.
#include "Es3VideoProtocol.h"

namespace es3video {
class VideoState {
public:
    enum State { Free, Receiving, Ready, Working };
    struct Slot {
        State state = Free;
        uint32_t token = 0, frame = 0, bytes = 0, received = 0, crc = 0;
        uint32_t started = 0, decodeUs = 0;
    };
    struct Event { uint8_t bytes[24]; size_t size = 0; };
    struct Job {
        int jpeg = -1, image = -1;
        uint32_t token = 0, frame = 0, bytes = 0;
        uint16_t width = 0, height = 0;
        uint32_t visibilityEpoch = 0;
    };
    struct Image {
        int slot = -1;
        uint32_t token = 0, frame = 0, decodeUs = 0;
        uint16_t* pixels = nullptr;
    };
    struct Stats {
        uint32_t assembled = 0, decoded = 0, presented = 0;
        uint32_t dropped = 0, receivedBytes = 0, decodeUs = 0, blitUs = 0, jpegBytes = 0;
        uint32_t reasons[kReasonCount]{};
    } stats;
    uint8_t* jpeg[2]{};
    uint16_t* pixels[2]{};
    Slot inputs[2], images[2];
    StartVideoRequest config;
    uint32_t token = 0, grant = 0;
    Demand demand = DemandCalibrating;

    void reset() {
        token = 0; grant = 0; _lastFrame = _started = 0;
        _head = _count = 0; _fault = false;
        // Leased buffers cannot be reclaimed until their owners return them.
        for (int i = 0; i < 2; ++i) {
            if (inputs[i].state != Working) inputs[i].state = Free;
            if (images[i].state != Working) images[i].state = Free;
        }
    }
    void start(uint32_t session, const StartVideoRequest& request) {
        reset(); config = request; token = session; grant = 1;
        visibility();
    }
    bool fault() const { return _fault; }
    bool pop(Event& e) {
        if (!_count) return false;
        e = _events[_head]; _head = (_head + 1) % 32; --_count; return true;
    }
    void setDemand(Demand value) {
        if (value == demand) return;
        demand = value;
        if (value != DemandActive) ++_visibilityEpoch;
        if (!token) return;
        visibility();
        if (value != DemandActive) {
            for (int i = 0; i < 2; ++i) {
                if (inputs[i].state == Receiving || inputs[i].state == Ready)
                    reject(i, ReasonHidden);
                if (images[i].state == Ready) {
                    result(images[i].frame, ReasonHidden); images[i].state = Free;
                }
            }
        }
    }
    void expire(uint32_t now) {
        for (int i = 0; i < 2; ++i)
            if (inputs[i].state == Receiving && now - inputs[i].started >= config.assemblyTimeoutMs)
                reject(i, ReasonAssemblyTimeout);
    }
    void accept(const Record& r, uint32_t now) {
        if (!token || r.sessionToken != token || r.type >= 0x80) return;
        expire(now);
        if (r.type == RecordStart) {
            if (r.frameId <= _lastFrame) { result(r.frameId, ReasonBadFrameId); return; }
            if (_started >= grant) { result(r.frameId, ReasonNoCredit); return; }
            _lastFrame = r.frameId; ++_started;
            StartInfo info;
            Reason reason = decodeStart(r, info);
            if (reason == ReasonOk) reason = checkStartForSession(info, config);
            if (reason == ReasonOk && demand != DemandActive) reason = ReasonHidden;
            int slot = -1;
            for (int i = 0; i < 2; ++i) if (inputs[i].state == Free) { slot = i; break; }
            if (reason == ReasonOk && slot < 0) reason = ReasonBusy;
            if (reason != ReasonOk) { result(r.frameId, reason); credit(); return; }
            Slot& s = inputs[slot]; s = Slot(); s.state = Receiving;
            s.token = token; s.frame = r.frameId; s.bytes = info.jpegBytes; s.started = now;
            stats.jpegBytes = info.jpegBytes;
            return;
        }
        int slot = -1;
        for (int i = 0; i < 2; ++i)
            if (inputs[i].state == Receiving && inputs[i].frame == r.frameId) slot = i;
        if (slot < 0) return;
        Slot& s = inputs[slot];
        if (r.type == RecordChunk) {
            ChunkInfo chunk;
            Reason reason = decodeChunk(r, chunk);
            if (reason == ReasonOk) reason = checkChunkForSession(chunk, config, s.received, s.bytes);
            if (reason != ReasonOk) { reject(slot, reason); return; }
            memcpy(jpeg[slot] + s.received, chunk.data, chunk.size);
            s.crc = crc32Update(s.crc, chunk.data, chunk.size);
            s.received += chunk.size; stats.receivedBytes += chunk.size;
        } else if (r.type == RecordEnd) {
            uint32_t crc = 0;
            Reason reason = decodeEnd(r, crc);
            if (reason == ReasonOk && s.received != s.bytes) reason = ReasonBadLength;
            if (reason == ReasonOk && crc != s.crc) reason = ReasonBadCrc;
            if (reason != ReasonOk) { reject(slot, reason); return; }
            s.state = Ready; ++stats.assembled; result(s.frame, ReasonOk);
        } else if (r.type == RecordAbort) {
            Reason reason = ReasonCancelled;
            const Reason parsed = decodeAbort(r, reason);
            reject(slot, parsed == ReasonOk ? reason : parsed);
        }
    }
    bool takeJob(Job& job) {
        if (!token || demand != DemandActive) return false;
        int source = -1, destination = -1;
        for (int i = 0; i < 2; ++i) {
            if (inputs[i].state == Ready) source = i;
            if (images[i].state == Free) destination = i;
        }
        if (source < 0) return false;
        if (destination < 0) {
            for (int i = 0; i < 2; ++i) if (images[i].state == Ready) {
                result(images[i].frame, ReasonSuperseded); destination = i; break;
            }
        }
        if (destination < 0) { reject(source, ReasonBusy); return false; }
        inputs[source].state = Working;
        images[destination] = inputs[source];
        job.jpeg = source; job.image = destination; job.token = token;
        job.frame = inputs[source].frame; job.bytes = inputs[source].bytes;
        job.width = config.viewportW; job.height = config.viewportH;
        job.visibilityEpoch = _visibilityEpoch;
        return true;
    }
    void finishJob(const Job& job, bool ok, uint32_t duration) {
        inputs[job.jpeg].state = Free;
        Slot& image = images[job.image];
        image.state = Free;
        if (job.token != token || !token) return;
        credit();
        if (!ok || demand != DemandActive || job.visibilityEpoch != _visibilityEpoch) {
            result(job.frame, ok ? ReasonHidden : ReasonDecodeFailed); return;
        }
        // Retain only the newest completed image, never replace a drawing lease.
        for (int i = 0; i < 2; ++i) if (images[i].state == Ready) {
            result(images[i].frame, ReasonSuperseded); images[i].state = Free;
        }
        image.state = Ready; image.decodeUs = duration;
        ++stats.decoded; stats.decodeUs = duration;
    }
    bool acquire(Image& image) {
        if (!token || demand != DemandActive) return false;
        for (int i = 0; i < 2; ++i) if (images[i].state == Ready) {
            Slot& s = images[i]; s.state = Working;
            image.slot = i; image.token = s.token; image.frame = s.frame;
            image.decodeUs = s.decodeUs; image.pixels = pixels[i]; return true;
        }
        return false;
    }
    void release(Image& image, bool presented, uint32_t blitUs) {
        if (image.slot < 0) return;
        images[image.slot].state = Free;
        if (token && token == image.token) {
            if (presented) {
                PresentedInfo info; info.decodeUs = image.decodeUs; info.blitUs = blitUs;
                Event e; e.size = encodePresented(token, image.frame, info, e.bytes, sizeof(e.bytes)); emit(e);
                ++stats.presented; stats.blitUs = blitUs;
            } else result(image.frame, ReasonHidden);
        }
        image = Image();
    }
private:
    uint32_t _lastFrame = 0, _started = 0;
    uint32_t _visibilityEpoch = 0;
    Event _events[32]; unsigned _head = 0, _count = 0;
    bool _fault = false;
    void emit(const Event& e) {
        if (!e.size || _count == 32) { _fault = true; return; }
        _events[(_head + _count) % 32] = e; ++_count;
    }
    void result(uint32_t frame, Reason reason) {
        Event e; e.size = encodeResult(token, frame, reason, e.bytes, sizeof(e.bytes)); emit(e);
        if (reason != ReasonOk) { ++stats.dropped; ++stats.reasons[reason]; }
    }
    void visibility() {
        Event e; e.size = encodeVisibility(token, demand, e.bytes, sizeof(e.bytes)); emit(e);
    }
    void credit() {
        if (grant == UINT32_MAX) { _fault = true; return; }
        Event e; e.size = encodeCredit(token, ++grant, e.bytes, sizeof(e.bytes)); emit(e);
    }
    void reject(int slot, Reason reason) {
        result(inputs[slot].frame, reason); inputs[slot].state = Free; credit();
    }
};
} // namespace es3video
