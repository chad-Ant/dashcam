#ifndef HOST_C3_ARDUINO_STUB_H
#define HOST_C3_ARDUINO_STUB_H 1

/**
 * @file Arduino.h
 * @brief Host stand-in for the ESP32 Arduino core — only what the bridge uses.
 *
 * Resolved INSTEAD of the core's Arduino.h because the Makefile puts this
 * directory first on the include path, so src/main.cpp, commLink.cpp and
 * CommProtocol.cpp compile unmodified. Not an emulator: add the smallest thing
 * that compiles when a module under test needs more.
 *
 * Serial1 is a byte queue in each direction — the MKR Zero's side of the UART.
 * A test feeds it real frames built with the production buildFrame(), so the
 * bridge's own decoder, CRC and framing are under test too, not bypassed.
 */

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

uint32_t millis();
inline void delay(uint32_t) {}      // time is the test's, never the wall clock's

/// Sets the value millis() returns. The clock never advances on its own.
void hostSetMillis(uint32_t ms);

#define SERIAL_8N1 0x800001cu

class Stream {
public:
    virtual ~Stream() {}
    virtual int available() = 0;
    virtual int read() = 0;
};

class HardwareSerial : public Stream {
public:
    void   begin(unsigned long, uint32_t = SERIAL_8N1, int8_t = -1, int8_t = -1) {}
    int    available() override;
    int    read() override;
    int    availableForWrite();
    size_t write(const uint8_t *buf, size_t n);

    // ── test side ──
    void   hostReset();
    void   hostFeed(const uint8_t *buf, size_t n);   ///< Bytes from the MKR.
    size_t hostTxBytes() const { return txBytes_; }  ///< Bytes the bridge sent it.

private:
    static const size_t kRx = 8192;
    uint8_t rx_[kRx];
    size_t  head_ = 0, tail_ = 0;
    size_t  txBytes_ = 0;
};

extern HardwareSerial Serial1;

inline float temperatureRead() { return 40.0f; }

struct EspClass {
    uint32_t getFreeHeap() { return 250000u; }
};
extern EspClass ESP;

#define RTC_NOINIT_ATTR

#endif // HOST_C3_ARDUINO_STUB_H
