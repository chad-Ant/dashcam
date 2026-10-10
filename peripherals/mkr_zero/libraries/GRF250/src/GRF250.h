#ifndef DASHCAM_GRF250_H
#define DASHCAM_GRF250_H

// Portable, allocation-free GRF-250 UART protocol (guide rev 3 / 5.2).
// RS-422 is an electrical transport: two full-duplex transceivers are needed.
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace grf250 {
constexpr size_t kMaxPayload = 33; // ID + all eight ID44 int32 fields
constexpr size_t kMaxFrame = kMaxPayload + 5;
inline uint16_t crc(const uint8_t* p, size_t n) {
    uint16_t c = 0;
    for (size_t i = 0; i < n; ++i) {
        c ^= static_cast<uint16_t>(p[i]) << 8;
        for (unsigned j = 0; j < 8; ++j)
            c = static_cast<uint16_t>((c << 1) ^ ((c & 0x8000u) ? 0x1021u : 0u));
    }
    return c;
}
inline uint32_t get32(const uint8_t* p) {
    return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
}
inline int32_t signed32(const uint8_t* p) {
    const uint32_t u = get32(p);
    return u <= 0x7fffffffu ? static_cast<int32_t>(u) : -1 - static_cast<int32_t>(~u);
}
inline void put32(uint8_t* p, uint32_t n) {
    for (unsigned i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(n >> (8 * i));
}
inline size_t command(uint8_t id, bool write, uint32_t value, uint8_t* out, size_t cap) {
    const size_t n = write ? 10u : 6u;
    if (!out || cap < n) return 0;
    out[0] = 0xaa;
    const uint16_t flags = static_cast<uint16_t>(((write ? 5u : 1u) << 6) | (write ? 1u : 0u));
    out[1] = static_cast<uint8_t>(flags); out[2] = static_cast<uint8_t>(flags >> 8);
    out[3] = id;
    if (write) put32(out + 4, value);
    const uint16_t c = crc(out, n - 2);
    out[n-2] = static_cast<uint8_t>(c); out[n-1] = static_cast<uint8_t>(c >> 8);
    return n;
}
struct Packet { uint8_t payload[kMaxPayload] = {}; size_t size = 0; };
class Parser {
public:
    uint32_t errors = 0;
    void reset() { used_ = 0; }
    bool feed(uint8_t b, uint32_t nowUs, Packet& packet) {
        if (used_ && uint32_t(nowUs - last_) > 20000u) { reset(); ++errors; }
        last_ = nowUs;
        if (used_ == kMaxFrame) { discard(); ++errors; }
        buf_[used_++] = b;
        // Bounded sliding resynchronization also recovers an AA inside a bad frame.
        for (size_t attempt = 0; attempt < kMaxFrame; ++attempt) {
            if (!used_) return false;
            if (buf_[0] != 0xaa) { discard(); continue; }
            if (used_ < 3) return false;
            const uint16_t flags = uint16_t(buf_[1]) | uint16_t(buf_[2]) << 8;
            const size_t payload = flags >> 6;
            // Reserved flags are deliberately NOT assumed to be zero.
            if (payload < 1 || payload > kMaxPayload) { discard(); ++errors; continue; }
            const size_t total = payload + 5;
            if (used_ < total) return false;
            const uint16_t expected = uint16_t(buf_[total-2]) | uint16_t(buf_[total-1]) << 8;
            if (crc(buf_, total-2) != expected) { discard(); ++errors; continue; }
            memcpy(packet.payload, buf_ + 3, payload); packet.size = payload;
            used_ -= total; memmove(buf_, buf_ + total, used_);
            return true;
        }
        return false;
    }
private:
    void discard() { if (used_) { --used_; memmove(buf_, buf_ + 1, used_); } }
    uint8_t buf_[kMaxFrame] = {};
    size_t used_ = 0;
    uint32_t last_ = 0;
};
struct Sample {
    uint32_t sequence = 0, receivedUs = 0;
    int32_t firstCm = -10, firstDb = 0, lastCm = -10, lastDb = 0;
};

// Single-owner main-loop FSM. A caller MUST service nextCommand and receive
// frequently and must discard buffered UART bytes after any lengthy pause.
// Writes only RAM configuration, never Save/Reset/firmware/laser-enable commands.
class Driver {
public:
    uint32_t faults = 0, firmware = 0;
    bool ready() const { return step_ == kRunning; }
    uint32_t parseErrors() const { return parser_.errors; }
    void transportFault(uint32_t nowUs) { fail(nowUs); }
    size_t nextCommand(uint32_t nowUs, uint8_t* out, size_t capacity) {
        if (cooling_) {
            if (uint32_t(nowUs - sent_) < 2000000u) return 0;
            cooling_ = false;
        }
        if (ready()) {
            if (uint32_t(nowUs - lastSample_) > 500000u) fail(nowUs);
            return 0;
        }
        if (waiting_ && uint32_t(nowUs - sent_) < 250000u) return 0;
        if (tries_ >= 5) { fail(nowUs); return 0; }
        const size_t n = command(id(), isWrite(), value(), out, capacity);
        if (n) { waiting_ = true; sent_ = nowUs; ++tries_; }
        return n;
    }
    bool receive(uint8_t byte, uint32_t nowUs, Sample& sample) {
        Packet p;
        if (!parser_.feed(byte, nowUs, p)) return false;
        if (ready()) {
            if (p.payload[0] != 44 || p.size != 17) return false;
            lastSample_ = nowUs;
            sample.sequence = ++sequence_; sample.receivedUs = nowUs;
            sample.firstCm = signed32(p.payload + 1); sample.firstDb = signed32(p.payload + 5);
            sample.lastCm = signed32(p.payload + 9); sample.lastDb = signed32(p.payload + 13);
            return true; // Lost-signal sentinels MUST reach the host, not retain old distance.
        }
        if (!waiting_ || cooling_ || p.payload[0] != id()) return false;
        if (step_ == 0) {
            if (p.size != 17 || memcmp(p.payload + 1, "GRF250\0", 7) != 0) { fail(nowUs); return false; }
        } else if (step_ == 1) {
            if (p.size != 5) return false;
            firmware = get32(p.payload + 1);
        } else if (p.size != 5 || get32(p.payload + 1) != value()) {
            fail(nowUs); return false;
        }
        ++step_; waiting_ = false; tries_ = 0; lastSample_ = nowUs;
        return false;
    }
private:
    static constexpr uint8_t kRunning = 12;
    uint8_t id() const {
        static const uint8_t ids[kRunning] = {0,2,30,30,27,27,78,78,74,74,30,30};
        return step_ < kRunning ? ids[step_] : 0;
    }
    bool isWrite() const { return step_ >= 2 && step_ % 2 == 0; }
    uint32_t value() const {
        if (step_ == 4 || step_ == 5) return 0x2du;
        if (step_ == 6 || step_ == 7) return 1u;
        if (step_ == 8 || step_ == 9) return 20u;
        if (step_ == 10 || step_ == 11) return 5u;
        return 0;
    }
    void fail(uint32_t nowUs) {
        ++faults; step_ = 0; waiting_ = false; tries_ = 0;
        cooling_ = true; sent_ = nowUs; parser_.reset();
    }
    Parser parser_;
    uint8_t step_ = 0, tries_ = 0;
    bool waiting_ = false, cooling_ = false;
    uint32_t sent_ = 0, lastSample_ = 0, sequence_ = 0;
};
} // namespace grf250
#endif
