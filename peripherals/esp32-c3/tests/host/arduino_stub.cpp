#include "Arduino.h"

HardwareSerial Serial1;
EspClass       ESP;

static uint32_t g_millis = 0;

uint32_t millis()               { return g_millis; }
void hostSetMillis(uint32_t ms) { g_millis = ms; }

void HardwareSerial::hostReset() { head_ = tail_ = 0; txBytes_ = 0; }

void HardwareSerial::hostFeed(const uint8_t *buf, size_t n)
{
    for (size_t i = 0; i < n && (head_ - tail_) < kRx; ++i) rx_[head_++ % kRx] = buf[i];
}

int HardwareSerial::available() { return static_cast<int>(head_ - tail_); }

int HardwareSerial::read()
{
    if (head_ == tail_) return -1;
    return rx_[tail_++ % kRx];
}

int    HardwareSerial::availableForWrite()                  { return 256; }
size_t HardwareSerial::write(const uint8_t *, size_t n)     { txBytes_ += n; return n; }
