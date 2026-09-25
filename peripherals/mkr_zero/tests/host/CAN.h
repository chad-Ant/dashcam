#ifndef HOST_CAN_STUB_H
#define HOST_CAN_STUB_H 1

/**
 * @file CAN.h
 * @brief Host stand-in for vendor/CANBus — the one call the sniffer makes.
 *
 * lib/CANSniffFunctions.cpp programs filters through the vendored library's
 * setFilterRegisters() and does everything else over raw SPI. The stand-in
 * writes the same masks, filters and RXBnCTRL values into the MCP2515 model and
 * ends in the mode asked for, exactly as the patched library does.
 */

#include <stdint.h>

/// lib/CANSniffFunctions.h refuses to compile against stock arduino-CAN. The
/// stand-in models the vendored library's one call, so it says it is vendored.
#define DASHCAM_CANBUS_VENDORED_FIXES 1

class MCP2515Class {
public:
    bool setFilterRegisters(uint16_t mask0, uint16_t filter0, uint16_t filter1,
                            uint16_t mask1, uint16_t filter2, uint16_t filter3,
                            uint16_t filter4, uint16_t filter5,
                            bool allowRollover, uint8_t targetMode);
};

extern MCP2515Class CAN;

#endif // HOST_CAN_STUB_H
