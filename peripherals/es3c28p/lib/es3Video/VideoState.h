#pragma once
// Portable ownership/assembly engine. Caller serializes every method; decode
// and TFT operations occur OUTSIDE that lock while holding the returned lease.
#include "Es3VideoProtocol.h"
#include "Es3MapProtocol.h"

namespace es3video {
class VideoState {
public:
    enum State { Free, Receiving, Ready, Decoding, Drawing };
    struct Slot {
        State state = Free;
        uint32_t token = 0, frame = 0, bytes = 0, received = 0, crc = 0;
        uint32_t started = 0, decodeUs = 0;
        uint16_t width = 0, height = 0;
        uint64_t origin = 0;
        es3map::PanImage kind = es3map::PanImageNone;
        bool presented = false;
    };
    struct Event { uint8_t bytes[24]; size_t size = 0; };
    // Handles identify a specific token/frame and slot. Return them once; stale
    // copies are ignored after return/reuse. The owner must not overwrite a live handle.
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
        uint16_t width = 0, height = 0;
        uint64_t origin = 0;
        es3map::PanImage kind = es3map::PanImageNone;
    };
    struct Stats {
        uint32_t assembled = 0, decoded = 0, presented = 0;
        uint32_t dropped = 0, receivedBytes = 0, decodeUs = 0, blitUs = 0, jpegBytes = 0;
        uint32_t reasons[kReasonCount]{};
    } stats;
    uint8_t* jpeg[2]{};
    // Slots 0/1: camera, whole map, or panning backgrounds. Slots 2/3:
    // panning strips only. No image of one kind can evict the other kind.
    uint16_t* pixels[4]{};
    Slot inputs[2], images[4];
    bool panning = false;
    es3map::PanSession panConfig;
    es3map::MapPose pose;
    uint32_t poseReceivedMs = 0;
    StartVideoRequest config;
    uint32_t token = 0, grant = 0;
    Demand demand = DemandCalibrating;

    void reset() {
        token = 0; grant = 0; _lastFrame = _started = 0;
        panning = false; pose = es3map::MapPose(); poseReceivedMs = 0;
        _head = _count = 0; _fault = false;
        // Leased buffers cannot be reclaimed until their owners return them.
        for (int i = 0; i < 2; ++i) {
            if (inputs[i].state != Decoding) inputs[i].state = Free;
        }
        for (int i = 0; i < 4; ++i)
            if (images[i].state != Decoding && images[i].state != Drawing) images[i].state = Free;
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
            }
            for (int i = 0; i < 4; ++i) {
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
        if (r.type == RecordMapPose) {
            es3map::MapPose candidate;
            if (panning && demand == DemandActive &&
                es3map::decodeMapPose(r, candidate) == ReasonOk && candidate.seq > pose.seq) {
                pose = candidate; poseReceivedMs = now;
            }
            return; // no credit and no reply, including malformed/stale poses
        }
        if (r.type == RecordStart) {
            if (r.frameId <= _lastFrame) { result(r.frameId, ReasonBadFrameId); return; }
            if (_started >= grant) { result(r.frameId, ReasonNoCredit); return; }
            _lastFrame = r.frameId; ++_started;
            StartInfo info;
            Reason reason = decodeStart(r, info);
            es3map::PanImage kind = es3map::PanImageNone;
            if (reason == ReasonOk) reason = panning ? es3map::checkPanStart(info, panConfig, kind) :
                checkStartForSession(info, config);
            if (reason == ReasonOk && demand != DemandActive) reason = ReasonHidden;
            int slot = -1;
            for (int i = 0; i < 2; ++i) if (inputs[i].state == Free) { slot = i; break; }
            if (reason == ReasonOk && slot < 0) reason = ReasonBusy;
            if (reason != ReasonOk) { result(r.frameId, reason); credit(); return; }
            Slot& s = inputs[slot]; s = Slot(); s.state = Receiving;
            s.token = token; s.frame = r.frameId; s.bytes = info.jpegBytes; s.started = now;
            s.width = info.width; s.height = info.height;
            s.origin = info.sourceTimestampUs; s.kind = kind;
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
    // JPEG input and output are exclusively leased to the decoder together.
    bool takeJob(Job& job) {
        if (!token || demand != DemandActive || ownsJob(job)) return false;
        int source = -1, destination = -1;
        for (int i = 0; i < 2; ++i) {
            if (inputs[i].state == Ready) source = i;
        }
        if (source < 0) return false;
        const int first = inputs[source].kind == es3map::PanImageChrome ? 2 : 0;
        for (int i = first; i < first + 2; ++i)
            if (images[i].state == Free) destination = i;
        if (destination < 0) {
            for (int i = first; i < first + 2; ++i) if (images[i].state == Ready) {
                result(images[i].frame, ReasonSuperseded); destination = i; break;
            }
        }
        if (destination < 0) {
            // A panning draw may briefly lease both old and replacement images
            // while swapping. Retain the ready input until the old lease is
            // returned; do not lose a rarely updated strip/background to Busy.
            if (!panning) reject(source, ReasonBusy);
            return false;
        }
        inputs[source].state = Decoding;
        images[destination] = inputs[source];
        job.jpeg = source; job.image = destination; job.token = token;
        job.frame = inputs[source].frame; job.bytes = inputs[source].bytes;
        job.width = inputs[source].width; job.height = inputs[source].height;
        job.visibilityEpoch = _visibilityEpoch;
        return true;
    }
    void finishJob(const Job& job, bool ok, uint32_t duration) {
        if (!ownsJob(job)) return; // duplicate/stale completion cannot free a reused slot
        inputs[job.jpeg].state = Free;
        Slot& image = images[job.image];
        image.state = Free;
        if (job.token != token || !token) return;
        credit();
        if (!ok || demand != DemandActive || job.visibilityEpoch != _visibilityEpoch) {
            result(job.frame, ok ? ReasonHidden : ReasonDecodeFailed); return;
        }
        // Retain only the newest completed image, never replace a drawing lease.
        for (int i = 0; i < 4; ++i) if (images[i].state == Ready && images[i].kind == image.kind) {
            result(images[i].frame, ReasonSuperseded); images[i].state = Free;
        }
        image.state = Ready; image.decodeUs = duration;
        ++stats.decoded; stats.decodeUs = duration;
    }
    bool acquire(Image& image, uint32_t expectedToken = 0) {
        if (expectedToken && token != expectedToken) return false;
        if (image.slot >= 0) return false; // never overwrite an outstanding lease
        if (!token || demand != DemandActive) return false;
        for (int i = 0; i < 4; ++i) if (images[i].state == Ready) {
            Slot& s = images[i]; s.state = Drawing;
            image.slot = i; image.token = s.token; image.frame = s.frame;
            image.decodeUs = s.decodeUs; image.pixels = pixels[i];
            image.width = s.width; image.height = s.height; image.origin = s.origin; image.kind = s.kind;
            return true;
        }
        return false;
    }
    // Panning holds its displayed images across many poses. Acknowledge once,
    // separately from releasing the lease, so they cannot be decoded over.
    void present(const Image& image, uint32_t blitUs) {
        if (!ownsImage(image) || !token || image.token != token || demand != DemandActive) return;
        Slot& slot = images[image.slot];
        if (slot.presented) return;
        slot.presented = true;
        PresentedInfo info; info.decodeUs = image.decodeUs; info.blitUs = blitUs;
        Event e; e.size = encodePresented(token, image.frame, info, e.bytes, sizeof(e.bytes)); emit(e);
        ++stats.presented; stats.blitUs = blitUs;
    }
    void release(Image& image, bool presented, uint32_t blitUs) {
        if (ownsImage(image)) {
            if (presented && demand == DemandActive) present(image, blitUs);
            else if (token && token == image.token && !images[image.slot].presented)
                result(image.frame, ReasonHidden);
            images[image.slot].state = Free;
        }
        image = Image();
    }

private:
    uint32_t _lastFrame = 0, _started = 0;
    uint32_t _visibilityEpoch = 0;
    Event _events[32]; unsigned _head = 0, _count = 0;
    bool _fault = false;
    bool ownsJob(const Job& job) const {
        if (job.jpeg < 0 || job.jpeg >= 2 || job.image < 0 || job.image >= 4) return false;
        const Slot& input = inputs[job.jpeg];
        const Slot& output = images[job.image];
        return input.state == Decoding && output.state == Decoding &&
            input.token == job.token && output.token == job.token &&
            input.frame == job.frame && output.frame == job.frame;
    }
    bool ownsImage(const Image& image) const {
        if (image.slot < 0 || image.slot >= 4) return false;
        const Slot& slot = images[image.slot];
        return slot.state == Drawing && slot.token == image.token &&
            slot.frame == image.frame && pixels[image.slot] == image.pixels;
    }
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
