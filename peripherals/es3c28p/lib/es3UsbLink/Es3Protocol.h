#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// ES3C28P USB display protocol v1, independent of the C3 telemetry protocol.
// E3 28 | version | sequence | type | length | payload | CRC16 little endian.
// CRC-16/CCITT-FALSE covers version through payload. See host/es3_display.py.
// Keep byte-identical: peripherals/es3c28p/lib/es3UsbLink/ and lib/libes3display/.
namespace es3proto {
constexpr uint8_t kVersion = 1;
constexpr size_t kMaxPayload = 128;
constexpr size_t kMaxFrame = kMaxPayload + 8;
constexpr size_t kTextBytes = 24;
constexpr uint32_t kPartialTimeoutMs = 250;
// 7..9 negotiate the E3 29 video preview; payloads are in Es3VideoProtocol.h.
// Firmware without video answers them with BadCommand: "video unsupported".
enum Type : uint8_t {
    Hello = 1, Ping = 2, Status = 3, Page = 4, Alert = 5, StopAudio = 6,
    Capabilities = 7, StartVideo = 8, StopVideo = 9,
    Reply = 0x80, Error = 0xff
};
enum ErrorCode : uint8_t { BadCommand = 1, BadPayload = 2, NotReady = 3, NoSession = 4 };

inline uint16_t crc16(const uint8_t* data, size_t size)
{
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < size; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        }
    }
    return crc;
}

struct Frame {
    uint8_t sequence = 0;
    uint8_t type = 0;
    uint8_t length = 0;
    uint8_t payload[kMaxPayload]{};
};

inline size_t encode(uint8_t sequence, uint8_t type, const uint8_t* payload,
                     size_t length, uint8_t (&out)[kMaxFrame])
{
    if (length > kMaxPayload || (length && !payload)) return 0;
    out[0] = 0xe3; out[1] = 0x28; out[2] = kVersion;
    out[3] = sequence; out[4] = type; out[5] = static_cast<uint8_t>(length);
    if (length) memcpy(out + 6, payload, length);
    const uint16_t crc = crc16(out + 2, length + 4);
    out[6 + length] = crc & 0xff;
    out[7 + length] = crc >> 8;
    return length + 8;
}

class Parser {
public:
    void reset() { _count = 0; }
    bool feed(uint8_t byte, uint32_t now, Frame& frame)
    {
        if (_count && now - _lastByte > kPartialTimeoutMs) reset();
        _lastByte = now;
        if (_count == sizeof(_buffer)) reset();
        _buffer[_count++] = byte;
        // Each iteration discards at least one byte or returns; <= kMaxFrame.
        for (size_t step = 0; step < kMaxFrame && _count >= 2; ++step) {
            if (_buffer[0] != 0xe3 || _buffer[1] != 0x28) { discard(1); continue; }
            if (_count < 6) return false;
            if (_buffer[2] != kVersion || _buffer[5] > kMaxPayload) { discard(1); continue; }
            const size_t length = _buffer[5];
            const size_t total = length + 8;
            if (_count < total) return false;
            const uint16_t received = _buffer[total - 2] | (uint16_t(_buffer[total - 1]) << 8);
            if (crc16(_buffer + 2, length + 4) != received) { discard(1); continue; }
            frame.sequence = _buffer[3]; frame.type = _buffer[4];
            frame.length = static_cast<uint8_t>(length);
            if (length) memcpy(frame.payload, _buffer + 6, length);
            discard(total);
            return true;
        }
        return false;
    }
private:
    uint8_t _buffer[kMaxFrame]{};
    size_t _count = 0;
    uint32_t _lastByte = 0;
    void discard(size_t count)
    {
        _count -= count;
        memmove(_buffer, _buffer + count, _count);
    }
};
}  // namespace es3proto
