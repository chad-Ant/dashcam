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

/*
 * bno055Delay() and bno055TransportBind() stood here and are both gone.
 *
 * They existed to satisfy the vendored driver's function-pointer interface:
 * bind() installed bus_read, bus_write and delay_msec into its context struct so
 * it could reach a bus, and the delay hook was a bounded, watchdog-fed wrapper
 * so a driver path asking to block for a second could not take the watchdog with
 * it. That clamp guarded an assumption the driver turned out never to test — it
 * does not call delay_msec at all, not once in 16 000 lines.
 *
 * With the driver gone there is no indirection to install: this file's functions
 * are called directly. One fewer way for the hooks to be unset.
 */

bool bno055Identify(BNO055Device &dev, uint8_t address)
{
    dev = BNO055Device{};
    dev.address = address;
    if (address == 0u) return false;

    uint8_t id = 0u, acc = 0u, mag = 0u, gyr = 0u, bl = 0u, page = 0u;
    uint8_t sw[2] = { 0u, 0u };

    // EVERY read checked, and the verdict is the AND of all of them. The
    // function this replaces assigned its status from each read in turn and so
    // returned only the last one's — a success that said nothing about whether
    // the part had answered. That trap needed a note in the vendored copy
    // telling callers to check chip_id themselves; here there is nothing left
    // for a caller to remember.
    bool ok = true;
    ok = ok && (bno055BusRead(address, BNO055_CHIP_ID_ADDR,       &id,   1u) == 0);
    ok = ok && (bno055BusRead(address, BNO055_ACC_REV_ID_ADDR,     &acc,  1u) == 0);
    ok = ok && (bno055BusRead(address, BNO055_MAG_REV_ID_ADDR,     &mag,  1u) == 0);
    ok = ok && (bno055BusRead(address, BNO055_GYR_REV_ID_ADDR,     &gyr,  1u) == 0);
    ok = ok && (bno055BusRead(address, BNO055_BL_REV_ID_ADDR,      &bl,   1u) == 0);
    ok = ok && (bno055BusRead(address, BNO055_SW_REV_ID_LSB_ADDR,  sw,    2u) == 0);
    ok = ok && (bno055BusRead(address, BNO055_PAGE_ID_ADDR,        &page, 1u) == 0);
    if (!ok) return false;

    dev.chipId          = id;
    dev.accelRevId      = acc;
    dev.magRevId        = mag;
    dev.gyroRevId       = gyr;
    dev.bootloaderRevId = bl;
    // Both bytes. See BNO055Device::swRevId for the one that used to be dropped.
    dev.swRevId         = (uint16_t)(sw[0] | ((uint16_t)sw[1] << 8));
    dev.pageId          = page;

    return id == BNO055_EXPECTED_CHIP_ID;
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
        // Identified by CHIP_ID, not by a bare address ACK. Something else can
        // sit at either address, and an ACK only proves that something is there.
        if (id == BNO055_EXPECTED_CHIP_ID) return kCandidates[i];
    }
    return 0u;
}
