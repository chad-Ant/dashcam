#ifndef DASHCAM_AS5048B_H
#define DASHCAM_AS5048B_H
#include "EncoderPan.h"

namespace encoderpan {
struct AS5048Config {
    uint8_t address = 0x40; // A1/A2 low; avoid production GNSS at 0x42
    uint16_t zeroCounts = 0;
    bool reversed = false, calibrated = false;
    int32_t calibratedErrorMdeg = 1500; // NOT the 0.022 degree resolution
    int32_t maxSpeedMdegS = 600000;
    uint32_t maxReadUs = 4000;
};

// Bus contract: read(address,register,uint8_t&) -> bool; bounded, no retries.
// Clock contract: now() -> uint32_t microseconds. No ownership of shared bus.
// Read-only: never burn OTP or change the chip's zero/address registers.
class AS5048B {
public:
    explicit AS5048B(const AS5048Config& config) : c_(config) {}
    static bool healthy(uint8_t diag) { return (diag & 0x0fu) == 1; }
    static int32_t angle(uint16_t counts, uint16_t zero, bool reversed) {
        int32_t delta = (int32_t(counts)-zero+16384) % 16384;
        if (delta >= 8192) delta -= 16384;
        return static_cast<int32_t>(int64_t(reversed ? -delta : delta)*360000/16384);
    }
    template<class Bus, class Clock>
    EncoderSample sample(Bus& bus, Clock& clock) const {
        EncoderSample s;
        const uint32_t start = clock.now();
        s.timeUs = start;
        if (c_.address < 0x08 || c_.address > 0x77 || c_.zeroCounts >= 16384
                || c_.calibratedErrorMdeg < 0 || c_.calibratedErrorMdeg > 5000
                || c_.maxSpeedMdegS <= 0 || c_.maxSpeedMdegS > 720000
                || c_.maxReadUs == 0 || c_.maxReadUs > 10000) return s;
        uint8_t diag = 0, hi = 0, lo = 0, verify = 0;
        if (!bus.read(c_.address, 0xfb, diag) || !healthy(diag)) return s;
        bool coherent = false;
        // Avoid assuming multi-byte hardware latching. High-low-high with a
        // bounded retry at the 6-bit carry. Reserved bits are ignored.
        for (uint8_t attempt = 0; attempt < 3; ++attempt) {
            if (!bus.read(c_.address, 0xfe, hi) || !bus.read(c_.address, 0xff, lo)
                    || !bus.read(c_.address, 0xfe, verify)) return s;
            if (hi == verify) { coherent = true; break; }
            if (uint32_t(clock.now()-start) > c_.maxReadUs) return s;
        }
        if (!coherent || !bus.read(c_.address, 0xfb, diag) || !healthy(diag)) return s;
        s.timeUs = clock.now();
        const uint32_t elapsed = s.timeUs-start;
        if (elapsed > c_.maxReadUs) return s;
        const uint16_t counts = static_cast<uint16_t>((uint16_t(hi)<<6) | (lo & 0x3f));
        s.rawCounts = counts;
        s.angleMdeg = angle(counts, c_.zeroCounts, c_.reversed);
        // Include full acquisition span + sensor's specified <=110.2us delay.
        s.errorMdeg = c_.calibratedErrorMdeg + static_cast<int32_t>(
            (int64_t(c_.maxSpeedMdegS)*(elapsed+111)+999999)/1000000);
        s.healthy = true; s.referenced = c_.calibrated;
        return s;
    }
private:
    AS5048Config c_;
};
} // namespace encoderpan
#endif
