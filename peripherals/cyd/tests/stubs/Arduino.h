#pragma once
#include <stdint.h>
#include <stddef.h>
#include <algorithm>
#include <deque>
#include <vector>
using std::min;
extern uint32_t testMillis;
inline uint32_t millis() { return testMillis; }
inline uint32_t micros() { return testMillis * 1000; }
struct TestSerial {
    std::deque<uint8_t> input;
    std::vector<uint8_t> output;
    int writable = 7; // force partial writes across update() calls
    void setRxBufferSize(size_t) {}
    void setTxBufferSize(size_t) {}
    void begin(unsigned baud) { if (baud != 115200) __builtin_abort(); }
    int available() const { return int(input.size()); }
    int read() { if (input.empty()) return -1; int n = input.front(); input.pop_front(); return n; }
    int availableForWrite() const { return writable; }
    size_t write(const uint8_t* data, size_t size) { output.insert(output.end(), data, data+size); return size; }
};
extern TestSerial Serial;
