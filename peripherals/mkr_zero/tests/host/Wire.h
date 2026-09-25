#ifndef HOST_WIRE_STUB_H
#define HOST_WIRE_STUB_H 1

/**
 * @file Wire.h
 * @brief Host stand-in for the Arduino Wire API — just enough to compile.
 *
 * lib/I2CBus.h includes <Wire.h>, and IMUFunctions.cpp's bus survey addresses
 * Wire directly. The IMU tests never reach that survey: every transaction they
 * care about goes through the counted transport (bno055BusRead/Write), which
 * imu_tests.cpp fakes. So nothing here answers — an address probe NACKs.
 */

#include <stdint.h>

/// lib/I2CBus.h refuses to compile against a stock SAMD Wire; the stand-in has
/// no requestFrom() at all, so it is neither, and saying it carries the fix is
/// what lets the real header through.
#define DASHCAM_SAMD_WIRE_REQUESTFROM1_FIX 1

class TwoWire {
public:
    void    begin() {}
    void    beginTransmission(uint8_t) {}
    uint8_t endTransmission(bool = true) { return 2u; }   // 2 = address NACK
};

extern TwoWire Wire;

#endif // HOST_WIRE_STUB_H
