#include <Wire.h>

#include "BNO055Transport.h"
#include "I2CBus.h"

#ifndef DASHCAM_SAMD_WIRE_REQUESTFROM1_FIX
#error "Stock Wire detected. Build with --library peripherals/mkr_zero/vendor/Wire (see vendor/Wire/README.md); the stock TwoWire::requestFrom() reads an uninitialised busOwner on 1-byte transfers, and single-byte register reads are the most common transaction this driver makes."
#endif

/// Bosch's convention: zero is success, anything else is a failure.
static const int BNO_OK  = 0;
static const int BNO_ERR = -1;

static uint16_t gFaults     = 0;   ///< Consecutive.
static uint32_t gErrorTotal = 0;   ///< Since boot.

uint16_t bno055TransportFaults()     { return gFaults; }
uint32_t bno055TransportErrorCount() { return gErrorTotal; }
void     bno055TransportResetFaults(){ gFaults = 0; }

/** @brief Books a failed transfer against both counters. */
static int noteTransferFault()
{
    if (gFaults < 0xFFFFu) ++gFaults;
    ++gErrorTotal;
    return BNO_ERR;
}

/**
 * @brief Is it safe to touch the bus at all?
 *
 * Refuses on a clamped SDA rather than transacting into it. That is not just
 * futile: if the clamp is a latched-up device, every attempt pushes current
 * through it, and this board's pins are on the other end. The same reasoning
 * stopped the GNSS bring-up and the IMU retry loop hammering a dead bus.
 */
static bool busUsable()
{
    if (i2cStuckLines() & I2C_STUCK_SDA_NEVER_MOVED) return false;
    return i2cBusBegin() == I2CBusState::Ready;
}

/**
 * @brief Register read for the driver's @c bus_read hook.
 *
 * Signature is fixed by BNO055_RD_FUNC_PTR and cannot be changed.
 *
 * The length check on the reply is the point of writing this by hand. Wire can
 * return fewer bytes than asked for, and the shipped glue looped over whatever
 * arrived and returned success regardless — so a short read silently left the
 * tail of the caller's buffer holding the PREVIOUS sample. That is a corrupt
 * reading dressed as a fresh one, which is the exact failure mode that retired
 * the last IMU.
 */
/**
 * @brief The read itself.
 *
 * @param book  Charge a failure to the fault counters.  False for address
 *              PROBES, where silence is the answer to a question rather than a
 *              fault: scanning 0x28 on a board strapped to 0x29 must not leave
 *              a healthy part reporting one I/O error for the rest of the boot.
 */
static int busReadInto(unsigned char dev_addr, unsigned char reg_addr,
                       unsigned char *reg_data, unsigned char cnt, bool book)
{
    if (reg_data == nullptr || cnt == 0u) return BNO_ERR;
    if (!busUsable())                     return book ? noteTransferFault() : BNO_ERR;

    Wire.beginTransmission(dev_addr);
    if (Wire.write(reg_addr) != 1u)        return book ? noteTransferFault() : BNO_ERR;
    // No STOP: a repeated START is what keeps another master (there is none
    // here) or a bus glitch from landing between the address write and the read.
    if (Wire.endTransmission(false) != 0u) return book ? noteTransferFault() : BNO_ERR;

    const uint8_t got = Wire.requestFrom(dev_addr, cnt);
    if (got != cnt)                        return book ? noteTransferFault() : BNO_ERR;

    for (uint8_t i = 0; i < cnt; ++i) {
        const int b = Wire.read();
        // Short despite the count Wire just reported.
        if (b < 0) return book ? noteTransferFault() : BNO_ERR;
        reg_data[i] = (unsigned char)b;
    }

    if (book) gFaults = 0;
    return BNO_OK;
}

extern "C" int bno055BusRead(unsigned char dev_addr, unsigned char reg_addr,
                             unsigned char *reg_data, unsigned char cnt)
{
    return busReadInto(dev_addr, reg_addr, reg_data, cnt, true);
}

/** @brief Register write for the driver's @c bus_write hook. */
extern "C" int bno055BusWrite(unsigned char dev_addr, unsigned char reg_addr,
                              unsigned char *reg_data, unsigned char cnt)
{
    if (reg_data == nullptr || cnt == 0u) return BNO_ERR;
    if (!busUsable())                     return noteTransferFault();

    Wire.beginTransmission(dev_addr);
    if (Wire.write(reg_addr) != 1u) return noteTransferFault();
    for (uint8_t i = 0; i < cnt; ++i) {
        if (Wire.write(reg_data[i]) != 1u) return noteTransferFault();
    }
    if (Wire.endTransmission(true) != 0u) return noteTransferFault();

    gFaults = 0;
    return BNO_OK;
}

/**
 * @brief The driver's @c delay_msec hook.
 *
 * Bounded and watchdog-fed. The driver asks for short waits around register
 * writes, which are fine; the long ones the part needs (400 ms from power-on,
 * 650 ms from reset, 19 ms leaving an operating mode) are handled by the staged
 * bring-up instead, so they never arrive here.
 *
 * The clamp is a guard against that assumption quietly becoming false: a driver
 * path that asks to block for a second would otherwise take the watchdog with
 * it, and this is precisely the failure the GNSS bring-up was restructured to
 * eliminate. Feeding the watchdog while waiting keeps a legitimate long wait
 * survivable; clamping keeps it from being silent.
 */
extern "C" void bno055Delay(BNO055_MDELAY_DATA_TYPE ms)
{
    uint32_t remaining = (uint32_t)ms;
    if (remaining > 50u) remaining = 50u;
    while (remaining > 0u) {
        const uint32_t slice = (remaining > 5u) ? 5u : remaining;
        delay(slice);
        watchdogFeed();
        remaining -= slice;
    }
}

void bno055TransportBind(struct bno055_t &dev, uint8_t address)
{
    dev.dev_addr   = address;   // honoured thanks to the vendored init patch
    dev.bus_read   = bno055BusRead;
    dev.bus_write  = bno055BusWrite;
    dev.delay_msec = bno055Delay;
}

uint8_t bno055FindAddress()
{
    // Both strappings, low first. The datasheet default is 0x29 and the driver's
    // built-in constant is 0x28, so neither is a safe assumption — a GY board
    // with COM3 floating sits at 0x29 while an Adafruit one is strapped to 0x28.
    static const uint8_t kCandidates[2] = { 0x28u, 0x29u };

    if (!busUsable()) return 0u;

    for (uint8_t i = 0; i < 2u; ++i) {
        uint8_t id = 0u;
        // Probe, so a silent candidate address is not charged to the counters.
        if (busReadInto(kCandidates[i], BNO055_CHIP_ID_ADDR, &id, 1u, false) != BNO_OK) continue;
        // Identified by CHIP_ID, not by a bare address ACK. Something else could
        // sit at either address, and bno055_init() cannot tell you - it returns
        // only the status of its LAST read, so a wrong device passes it.
        if (id == BNO055_EXPECTED_CHIP_ID) return kCandidates[i];
    }
    return 0u;
}
