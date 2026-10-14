#include "../lib/es3UsbLink/Es3Protocol.h"
#include <cassert>
#include <cstdio>
#include <vector>

using namespace es3proto;

int main()
{
    const uint8_t check[] = {'1','2','3','4','5','6','7','8','9'};
    assert(crc16(check, sizeof(check)) == 0x29b1);
    uint8_t wire[kMaxFrame];
    const uint8_t payload[] = {'E','S','3','C','2','8','P'};
    const size_t size = encode(42, Hello | Reply, payload, sizeof(payload), wire);
    assert(size == 15);
    Parser parser;
    Frame frame;
    for (size_t i = 0; i < size; ++i) {
        assert(parser.feed(wire[i], uint32_t(i), frame) == (i + 1 == size));
    }
    assert(frame.sequence == 42 && frame.type == (Hello | Reply));
    assert(frame.length == sizeof(payload) && !memcmp(frame.payload, payload, sizeof(payload)));
    // This vector is checked independently by Python, preventing codec drift.
    for (size_t i = 0; i < size; ++i) std::printf("%02x", wire[i]);
    std::puts("");

    // Noise, a bad CRC, and a valid frame in one contiguous byte stream.
    std::vector<uint8_t> stream{0, 0x7e, 0xe3, 0xe3};
    stream.insert(stream.end(), wire, wire + size);
    stream.back() ^= 0xff;
    stream.insert(stream.end(), wire, wire + size);
    unsigned accepted = 0;
    for (auto byte : stream) accepted += parser.feed(byte, 100, frame);
    assert(accepted == 1);

    // Invalid version/length do not poison the following handshake.
    const uint8_t invalid[] = {0xe3, 0x28, 2, 0, 1, 0, 0xe3, 0x28, 1, 0, 1, 255};
    for (auto byte : invalid) assert(!parser.feed(byte, 101, frame));
    for (size_t i = 0; i < size; ++i) {
        assert(parser.feed(wire[i], 101, frame) == (i + 1 == size));
    }
    // Stalled fragments expire, including when millis() wraps around.
    parser.reset();
    assert(!parser.feed(wire[0], 0xfffffff0U, frame));
    assert(!parser.feed(wire[1], 0xfffffff1U, frame));
    for (size_t i = 0; i < size; ++i) {
        assert(parser.feed(wire[i], 300, frame) == (i + 1 == size));
    }
    uint8_t largest[kMaxPayload]{};
    memset(largest, 0xe3, sizeof(largest));
    const size_t maximum = encode(255, Page, largest, sizeof(largest), wire);
    assert(maximum == kMaxFrame);
    for (size_t i = 0; i < maximum; ++i) {
        assert(parser.feed(wire[i], 400, frame) == (i + 1 == maximum));
    }
    assert(frame.length == kMaxPayload && !memcmp(frame.payload, largest, sizeof(largest)));
    assert(encode(0, Page, largest, kMaxPayload + 1, wire) == 0);
    assert(encode(0, Page, nullptr, 1, wire) == 0);
}
