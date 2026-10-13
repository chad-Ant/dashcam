#ifndef DASHCAM_BOUNDED_WIRE_ENCODER_H
#define DASHCAM_BOUNDED_WIRE_ENCODER_H
#include <Wire.h>
#if !defined(DASHCAM_WIRE_BOUNDED) || !defined(DASHCAM_SAMD_WIRE_REQUESTFROM1_FIX)
#error "EncoderPan requires this repository's bounded vendor/Wire (including one-byte read fix)"
#endif
namespace encoderpan {
class BoundedWireEncoder {
public:
    explicit BoundedWireEncoder(TwoWire& wire) : wire_(wire) {}
    bool read(uint8_t address, uint8_t reg, uint8_t& value) {
        const uint32_t errors = wire_.busErrorCount(), timeouts = wire_.timeoutCount();
        wire_.beginTransmission(address);
        if (wire_.write(reg) != 1 || wire_.endTransmission(false) != 0) return false;
        if (wire_.requestFrom(address, size_t(1), true) != 1 || wire_.available() != 1) return false;
        const int n = wire_.read();
        if (n < 0 || errors != wire_.busErrorCount() || timeouts != wire_.timeoutCount()) return false;
        value = static_cast<uint8_t>(n);
        return true;
    }
private:
    TwoWire& wire_;
};
struct MicrosClock { uint32_t now() const { return micros(); } };
} // namespace encoderpan
#endif
