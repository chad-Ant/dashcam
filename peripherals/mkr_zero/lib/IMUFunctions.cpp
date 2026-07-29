#include <Wire.h>
#include <math.h>

#include "DataDictionary.h"
#include "GlobalVariables.h"
#include "I2CBus.h"
#include "IMUFunctions.h"

// ─── compile-time bus allocation check ────────────────────────────────────────
//
// The cheapest conflict check is the one that never reaches hardware.  These
// catch an editing mistake in DataDictionary.h — a copied address, a jumper
// setting written into the wrong macro — at build time, where it costs nothing.
// The runtime scan in checkI2CBusConflict() covers what this cannot see: a
// device that is not in the dictionary at all.

static_assert(IMU_ACCEL_I2C_ADDRESS != GPS_DEFAULT_I2C_ADDRESS,
              "LSM6DSOX address collides with the GNSS receiver");
static_assert(IMU_MAG_I2C_ADDRESS != GPS_DEFAULT_I2C_ADDRESS,
              "LIS3MDL address collides with the GNSS receiver");
static_assert(IMU_ACCEL_I2C_ADDRESS != SEGLED_ADDRESS,
              "LSM6DSOX address collides with the segment LED");
static_assert(IMU_MAG_I2C_ADDRESS != SEGLED_ADDRESS,
              "LIS3MDL address collides with the segment LED");
static_assert(IMU_ACCEL_I2C_ADDRESS != IMU_MAG_I2C_ADDRESS,
              "LSM6DSOX and LIS3MDL cannot share one address");

// ─── LSM6DSOX registers ───────────────────────────────────────────────────────

static constexpr uint8_t SOX_REG_WHO_AM_I    = 0x0Fu;
static constexpr uint8_t SOX_ID              = 0x6Cu;
/// CTRL1_XL: ODR_XL[7:4], FS_XL[3:2].
static constexpr uint8_t SOX_REG_CTRL1_XL    = 0x10u;
/// CTRL2_G: ODR_G[7:4], FS_G[3:2], FS_125[1].
static constexpr uint8_t SOX_REG_CTRL2_G     = 0x11u;
/// CTRL3_C: BOOT[7], BDU[6], IF_INC[2], SW_RESET[0].
static constexpr uint8_t SOX_REG_CTRL3_C     = 0x12u;
static constexpr uint8_t SOX_CTRL3_SW_RESET  = 0x01u;
static constexpr uint8_t SOX_CTRL3_IF_INC    = 0x04u;
static constexpr uint8_t SOX_CTRL3_BDU       = 0x40u;
/// CTRL9_XL: I3C_disable[1].  Read-modify-written — the other bits are DEN
/// configuration whose reset value must survive.
static constexpr uint8_t SOX_REG_CTRL9_XL    = 0x18u;
static constexpr uint8_t SOX_CTRL9_I3C_DISABLE = 0x02u;
/// STATUS_REG: TDA[2], GDA[1], XLDA[0]; bits 7:3 reserved, read back zero.
static constexpr uint8_t SOX_REG_STATUS      = 0x1Eu;
static constexpr uint8_t SOX_STATUS_XLDA     = 0x01u;
static constexpr uint8_t SOX_STATUS_GDA      = 0x02u;
static constexpr uint8_t SOX_STATUS_TDA      = 0x04u;
/// Any reserved bit set means the byte did not come from a healthy LSM6DSOX —
/// in particular 0xFF, which is what a bus read that returns nothing looks like.
static constexpr uint8_t SOX_STATUS_RESERVED_MASK = 0xF8u;
/// First of the 14 contiguous output bytes: temp, gyro XYZ, accel XYZ, LSB first.
static constexpr uint8_t SOX_REG_OUT_TEMP_L  = 0x20u;
/// Die temperature: 256 LSB per degC, zero at +25 degC.
static constexpr float SOX_TEMP_LSB_PER_C    = 256.0f;
static constexpr float SOX_TEMP_OFFSET_C     = 25.0f;

// ─── LIS3MDL registers ────────────────────────────────────────────────────────

static constexpr uint8_t MDL_REG_WHO_AM_I    = 0x0Fu;
static constexpr uint8_t MDL_ID              = 0x3Du;
/// CTRL_REG1: TEMP_EN[7], OM[6:5], DO[4:2], FAST_ODR[1], ST[0].
static constexpr uint8_t MDL_REG_CTRL1       = 0x20u;
/// CTRL_REG2: FS[6:5], REBOOT[3], SOFT_RST[2].
static constexpr uint8_t MDL_REG_CTRL2       = 0x21u;
static constexpr uint8_t MDL_CTRL2_SOFT_RST  = 0x04u;
/// CTRL_REG3: MD[1:0] operating mode (00 = continuous conversion).
static constexpr uint8_t MDL_REG_CTRL3       = 0x22u;
/// CTRL_REG4: OMZ[3:2] z-axis performance mode.
static constexpr uint8_t MDL_REG_CTRL4       = 0x23u;
/// CTRL_REG5: FAST_READ[7], BDU[6].
static constexpr uint8_t MDL_REG_CTRL5       = 0x24u;
static constexpr uint8_t MDL_CTRL5_BDU       = 0x40u;
/// STATUS_REG: ZYXOR[7], ZYXDA[3], ZDA[2], YDA[1], XDA[0].  Every bit is
/// defined, so unlike the LSM6DSOX there is no all-ones signature to test.
static constexpr uint8_t MDL_REG_STATUS      = 0x27u;
static constexpr uint8_t MDL_STATUS_ZYXDA    = 0x08u;
/// First of the six output bytes, X/Y/Z LSB first.
static constexpr uint8_t MDL_REG_OUT_X_L     = 0x28u;
/// The LIS3MDL datasheet requires bit 7 of the sub-address to be set for a
/// multi-byte read, over I2C as well as SPI — without it the part is not
/// obliged to auto-increment.  Set it for bursts; deliberately NOT set for
/// single-register reads, where the resulting repeat of the same register is
/// exactly the harmless second byte readReg8() needs (see below).
static constexpr uint8_t MDL_AUTO_INCREMENT  = 0x80u;

// ─── configuration ────────────────────────────────────────────────────────────
//
// Ranges are a vehicle choice, not a datasheet default:
//
//  * +/-8 g rather than the +/-4 g most examples use.  Braking and cornering
//    live under 1.2 g, but a pothole or a kerb strike puts a short several-g
//    transient through the chassis, and a clipped impact is exactly the sample
//    an incident detector must not lose.  The cost is 2.4 mg resolution instead
//    of 1.2 mg, far below the vibration floor of a car anyway.
//  * +/-500 dps rather than +/-2000 dps.  A vehicle yaws at well under
//    100 deg/s even in a spin, so 2000 dps throws away four bits of resolution
//    for range that never gets used.
//  * +/-4 gauss (400 uT) for the magnetometer, the most sensitive setting.
//    Earth's field is roughly 50 uT; the headroom absorbs the car's own
//    magnetic distortion.
//
// These are raw register field values.  The scale helpers below re-derive their
// conversion from the SAME field values, so changing a range here cannot leave
// a stale scale factor behind.

/// ODR field value for 104 Hz, shared by accelerometer and gyroscope.
static constexpr uint8_t SOX_ODR_104_HZ   = 0x04u;
/// FS_XL field value for +/-8 g (00=2g, 01=16g, 10=4g, 11=8g — not in order).
static constexpr uint8_t SOX_FS_XL_8G     = 0x03u;
/// CTRL2_G[3:0] for +/-500 dps: FS_G=01, FS_125=0.
static constexpr uint8_t SOX_FS_G_500DPS  = 0x04u;

static constexpr uint8_t SOX_CTRL1_XL_VALUE =
    static_cast<uint8_t>((SOX_ODR_104_HZ << 4) | (SOX_FS_XL_8G << 2));
static constexpr uint8_t SOX_CTRL2_G_VALUE =
    static_cast<uint8_t>((SOX_ODR_104_HZ << 4) | SOX_FS_G_500DPS);

/// OM field value for high-performance mode, used for XY (CTRL1) and Z (CTRL4).
static constexpr uint8_t MDL_OM_HIGH       = 0x02u;
/// DO+FAST_ODR field value for 40 Hz.
static constexpr uint8_t MDL_ODR_40_HZ     = 0x0Cu;
/// FS field value for +/-4 gauss.
static constexpr uint8_t MDL_FS_4_GAUSS    = 0x00u;

static constexpr uint8_t MDL_CTRL1_VALUE =
    static_cast<uint8_t>((MDL_OM_HIGH << 5) | (MDL_ODR_40_HZ << 1));
static constexpr uint8_t MDL_CTRL2_VALUE = static_cast<uint8_t>(MDL_FS_4_GAUSS << 5);
static constexpr uint8_t MDL_CTRL3_VALUE = 0x00u;   ///< Continuous conversion.
static constexpr uint8_t MDL_CTRL4_VALUE = static_cast<uint8_t>(MDL_OM_HIGH << 2);
static constexpr uint8_t MDL_CTRL5_VALUE = MDL_CTRL5_BDU;

/// Standard gravity, matching the Adafruit unified-sensor constant.
static constexpr float STANDARD_GRAVITY_MS2 = 9.80665f;

/// Deadline for a device software reset to self-clear.  Both parts specify
/// well under 10 ms; this is generous and, unlike the vendor driver's
/// unbounded spin, it ends.
static constexpr uint32_t RESET_TIMEOUT_MS = 50u;
/// Settling time after a reset before the part is configured.
static constexpr uint32_t RESET_SETTLE_MS = 10u;

// ─── low-level bus access ─────────────────────────────────────────────────────

/**
 * @brief Reads @p len bytes starting at @p reg.
 * @return @c true only when the addressed device ACKed and supplied every byte.
 */
static bool readRegs(uint8_t address, uint8_t reg, uint8_t *buf, uint8_t len){
    if (buf == nullptr || len == 0u) return false;

    Wire.beginTransmission(address);
    if (Wire.write(reg) != 1u) {
        // Nothing has gone onto the bus yet; endTransmission() releases the
        // half-built transaction rather than leaving it queued for the next one.
        (void)Wire.endTransmission(true);
        return false;
    }
    // Repeated start: no STOP between the sub-address write and the read, so no
    // other master can slip in between.  A failed endTransmission() emits a STOP
    // internally, so the bus is never left held on this path.
    if (Wire.endTransmission(false) != 0u) return false;

    if (Wire.requestFrom(address, static_cast<size_t>(len), true) != static_cast<size_t>(len)) {
        return false;
    }

    for (uint8_t i = 0u; i < len; i++){
        const int value = Wire.read();
        if (value < 0) return false;   // buffer under-run; do not fabricate a byte
        buf[i] = static_cast<uint8_t>(value);
    }
    return true;
}

/**
 * @brief Reads a single register.
 *
 * A genuine one-byte request.  This used to pad to two bytes to dodge the stock
 * SAMD core's undefined behaviour on @c quantity==1, but that fix now lives
 * where it belongs — in the vendored, patched Wire (see
 * @c peripherals/mkr_zero/vendor/Wire/README.md, and the @c #error in
 * @c I2CBus.h that refuses to build without it).
 *
 * Keeping the padding on top of the real fix would have been worse than
 * useless: it doubles the traffic of the most frequent transaction on the bus,
 * it bakes in per-device assumptions about what the NEXT register does when
 * read, and — worst — it would mask a regression of the global fix, because the
 * IMU would keep working after someone dropped the vendor library from the
 * build while the GNSS driver silently went back to undefined behaviour.
 *
 * @return @c true on success.
 */
static bool readReg8(uint8_t address, uint8_t reg, uint8_t &value){
    return readRegs(address, reg, &value, 1u);
}

/** @brief Writes a single register. @return @c true when the device ACKed. */
static bool writeReg8(uint8_t address, uint8_t reg, uint8_t value){
    Wire.beginTransmission(address);
    if (Wire.write(reg) != 1u || Wire.write(value) != 1u){
        (void)Wire.endTransmission(true);
        return false;
    }
    return Wire.endTransmission(true) == 0u;
}

/** @brief Writes a register and reads it back. @return @c true when they agree. */
static bool writeVerifyReg8(uint8_t address, uint8_t reg, uint8_t value){
    if (!writeReg8(address, reg, value)) return false;
    uint8_t readback = 0u;
    if (!readReg8(address, reg, readback)) return false;
    return readback == value;
}

/** @brief Assembles a signed 16-bit little-endian sample from a burst buffer. */
static inline int16_t toInt16LE(const uint8_t *buf){
    return static_cast<int16_t>(static_cast<uint16_t>(buf[0]) |
                               (static_cast<uint16_t>(buf[1]) << 8));
}

// ─── scale factors ────────────────────────────────────────────────────────────

/** @brief m/s2 per LSB for an FS_XL field value (datasheet mg/LSB x g). */
static float accelScaleMs2(uint8_t fsXl){
    float milliGPerLsb = 0.061f;
    switch (fsXl){
        case 0x00u: milliGPerLsb = 0.061f; break;   // +/-2 g
        case 0x01u: milliGPerLsb = 0.488f; break;   // +/-16 g
        case 0x02u: milliGPerLsb = 0.122f; break;   // +/-4 g
        case 0x03u: milliGPerLsb = 0.244f; break;   // +/-8 g
        default:    milliGPerLsb = 0.061f; break;
    }
    return milliGPerLsb * STANDARD_GRAVITY_MS2 * 0.001f;
}

/** @brief deg/s per LSB for a CTRL2_G[3:0] field value (datasheet mdps/LSB). */
static float gyroScaleDps(uint8_t fsG){
    // FS_125 (bit 1) overrides the FS_G pair when set.
    if ((fsG & 0x02u) != 0u) return 4.375f * 0.001f;

    float milliDpsPerLsb = 8.75f;
    switch ((fsG >> 2) & 0x03u){
        case 0x00u: milliDpsPerLsb = 8.75f; break;  // +/-250 dps
        case 0x01u: milliDpsPerLsb = 17.5f; break;  // +/-500 dps
        case 0x02u: milliDpsPerLsb = 35.0f; break;  // +/-1000 dps
        case 0x03u: milliDpsPerLsb = 70.0f; break;  // +/-2000 dps
        default:    milliDpsPerLsb = 8.75f; break;
    }
    return milliDpsPerLsb * 0.001f;
}

/** @brief uT per LSB for an FS field value (datasheet LSB/gauss, 1 G = 100 uT). */
static float magScaleUt(uint8_t fs){
    float lsbPerGauss = 6842.0f;
    switch (fs){
        case 0x00u: lsbPerGauss = 6842.0f; break;   // +/-4 gauss
        case 0x01u: lsbPerGauss = 3421.0f; break;   // +/-8 gauss
        case 0x02u: lsbPerGauss = 2281.0f; break;   // +/-12 gauss
        case 0x03u: lsbPerGauss = 1711.0f; break;   // +/-16 gauss
        default:    lsbPerGauss = 6842.0f; break;
    }
    return 100.0f / lsbPerGauss;
}

// ─── bus housekeeping ─────────────────────────────────────────────────────────
//
// Opening, clocking and recovering the bus belong to I2CBus.h, not here: the
// GNSS receiver shares this bus and is often the first client on it, so the
// recover-before-first-transaction rule has to hold for whoever arrives first.

/** @brief True for an address in the usable 7-bit range (reserved blocks excluded). */
static inline bool isUsableI2CAddress(uint8_t address){
    return (address >= 0x08u) && (address <= 0x77u);
}

// ─── conflict checking ────────────────────────────────────────────────────────

IMUReturnStatus checkI2CBusConflict(I2CBusReport &report){
    report.deviceCount     = 0u;
    report.accelPresent    = false;
    report.magPresent      = false;
    report.gpsPresent      = false;
    report.accelIdentified = false;
    report.magIdentified   = false;
    report.conflict        = false;
    for (uint8_t i = 0u; i < I2C_SCAN_MAX_DEVICES; i++) report.addresses[i] = 0u;

    // Ask the bus manager BEFORE scanning.  scanI2CBus() correctly refuses a
    // stuck bus by returning zero, but zero devices is indistinguishable from an
    // empty bus once the number is all you have — and this function would then
    // report NOK_INIT_FAILED, which bring-up prints as "bus empty, check wiring
    // and power".  That sends someone hunting a disconnected sensor when the
    // real fault is a line held low, which is the opposite repair.
    if (i2cBusBegin() != I2CBusState::Ready) return IMUReturnStatus::NOK_BUS_STUCK;

    report.deviceCount = scanI2CBus(report.addresses, I2C_SCAN_MAX_DEVICES);

    // Probed directly rather than looked up in report.addresses[]: that buffer
    // holds only the first I2C_SCAN_MAX_DEVICES responders, so on a crowded bus
    // a device past the cut-off would be reported absent when it is sitting
    // right there.  Three extra transactions buy an answer that cannot lie.
    report.accelPresent = i2cProbeAddress(IMU_ACCEL_I2C_ADDRESS);
    report.magPresent   = i2cProbeAddress(IMU_MAG_I2C_ADDRESS);
    report.gpsPresent   = i2cProbeAddress(GPS_DEFAULT_I2C_ADDRESS);

    // Presence is not identity.  An address that ACKs proves only that SOMETHING
    // is there; reading WHO_AM_I is what distinguishes our IMU from another
    // board that happens to have been strapped to the same address.
    uint8_t id = 0u;
    if (report.accelPresent && readReg8(IMU_ACCEL_I2C_ADDRESS, SOX_REG_WHO_AM_I, id)){
        report.accelIdentified = (id == SOX_ID);
    }
    if (report.magPresent && readReg8(IMU_MAG_I2C_ADDRESS, MDL_REG_WHO_AM_I, id)){
        report.magIdentified = (id == MDL_ID);
    }

    report.conflict = (report.accelPresent && !report.accelIdentified) ||
                      (report.magPresent && !report.magIdentified);

    if (report.conflict)          return IMUReturnStatus::NOK_ADDRESS_CONFLICT;
    if (report.deviceCount == 0u) return IMUReturnStatus::NOK_INIT_FAILED;
    if (!report.accelPresent)     return IMUReturnStatus::NOK_ACCEL_MISSING;
    if (!report.magPresent)       return IMUReturnStatus::NOK_MAG_MISSING;
    return IMUReturnStatus::OK;
}

// ─── device configuration ─────────────────────────────────────────────────────

/**
 * @brief Waits for a self-clearing reset bit, with a deadline.
 * @return @c true when the bit cleared in time.
 */
static bool waitResetComplete(uint8_t address, uint8_t reg, uint8_t bit){
    const uint32_t start = millis();
    // Bounded on BOTH the deadline and the bus: a read failure ends the wait
    // immediately rather than spinning against a device that is not there.
    while ((millis() - start) < RESET_TIMEOUT_MS){
        uint8_t value = 0u;
        if (!readReg8(address, reg, value)) return false;
        if ((value & bit) == 0u) return true;
        delay(1);
    }
    return false;
}

/**
 * @brief Resets and configures the LSM6DSOX, verifying every write.
 *
 * A silently rejected range is the worst failure mode available here: the part
 * keeps converting at its previous full scale while this library keeps applying
 * the scale factor for the requested one, so every reading is wrong by a
 * constant multiple — plausible, self-consistent and undetectable downstream.
 * Hence write-then-read-back on the registers that define the scale.
 *
 * @return @c true when the part is present, identified and correctly configured.
 */
static bool configureAccel(IMUDevice &dev){
    const uint8_t address = dev.accelAddress;

    uint8_t id = 0u;
    if (!readReg8(address, SOX_REG_WHO_AM_I, id) || (id != SOX_ID)) return false;

    if (!writeReg8(address, SOX_REG_CTRL3_C, SOX_CTRL3_SW_RESET)) return false;
    if (!waitResetComplete(address, SOX_REG_CTRL3_C, SOX_CTRL3_SW_RESET)) return false;
    delay(RESET_SETTLE_MS);

    // Block Data Update stops a burst splicing the MSB of one sample onto the
    // LSB of the next; IF_INC is what makes the 14-byte burst walk the register
    // map at all.  Both are reset defaults on this part, but a device that came
    // up oddly is exactly the case worth being explicit about.
    if (!writeVerifyReg8(address, SOX_REG_CTRL3_C,
                         static_cast<uint8_t>(SOX_CTRL3_BDU | SOX_CTRL3_IF_INC))) return false;

    // Disable the I3C interface, which otherwise can reset the digital block on
    // an unrelated bus event.  Read-modify-write: the rest of CTRL9_XL is DEN
    // configuration whose reset value must survive.
    uint8_t ctrl9 = 0u;
    if (!readReg8(address, SOX_REG_CTRL9_XL, ctrl9)) return false;
    const uint8_t ctrl9Wanted = static_cast<uint8_t>(ctrl9 | SOX_CTRL9_I3C_DISABLE);
    // Read back like every other configuration write here.  A silently dropped
    // I3C-disable does not stop the part working, so nothing downstream would
    // ever report it — the sensor would simply become mysteriously resettable by
    // unrelated bus activity, months later, with no clue pointing here.
    if (!writeVerifyReg8(address, SOX_REG_CTRL9_XL, ctrl9Wanted)) return false;

    if (!writeVerifyReg8(address, SOX_REG_CTRL1_XL, SOX_CTRL1_XL_VALUE)) return false;
    if (!writeVerifyReg8(address, SOX_REG_CTRL2_G, SOX_CTRL2_G_VALUE)) return false;

    // Derived from the values just verified on the device, not from a constant
    // held in parallel — the scale and the hardware cannot drift apart.
    dev.accelScaleMs2 = accelScaleMs2((SOX_CTRL1_XL_VALUE >> 2) & 0x03u);
    dev.gyroScaleDps  = gyroScaleDps(SOX_CTRL2_G_VALUE & 0x0Fu);

    // Seed the stall clocks from now.  Without this they read zero, and the
    // first getIMUData() would measure a stall of however long the board had
    // been powered and immediately declare a freshly configured part faulty.
    const uint32_t now = millis();
    dev.lastAccelReadyMs = now;
    dev.lastGyroReadyMs  = now;
    return true;
}

/** @brief Resets and configures the LIS3MDL, verifying every write. */
static bool configureMag(IMUDevice &dev){
    const uint8_t address = dev.magAddress;

    uint8_t id = 0u;
    if (!readReg8(address, MDL_REG_WHO_AM_I, id) || (id != MDL_ID)) return false;

    if (!writeReg8(address, MDL_REG_CTRL2, MDL_CTRL2_SOFT_RST)) return false;
    if (!waitResetComplete(address, MDL_REG_CTRL2, MDL_CTRL2_SOFT_RST)) return false;
    delay(RESET_SETTLE_MS);

    // CTRL_REG3 last: it is what takes the part out of power-down into
    // continuous conversion, so writing it after the rate, range and BDU means
    // the first conversion already uses the intended configuration.
    if (!writeVerifyReg8(address, MDL_REG_CTRL1, MDL_CTRL1_VALUE)) return false;
    if (!writeVerifyReg8(address, MDL_REG_CTRL2, MDL_CTRL2_VALUE)) return false;
    if (!writeVerifyReg8(address, MDL_REG_CTRL4, MDL_CTRL4_VALUE)) return false;
    if (!writeVerifyReg8(address, MDL_REG_CTRL5, MDL_CTRL5_VALUE)) return false;
    if (!writeVerifyReg8(address, MDL_REG_CTRL3, MDL_CTRL3_VALUE)) return false;

    dev.magScaleUt = magScaleUt((MDL_CTRL2_VALUE >> 5) & 0x03u);
    dev.lastMagReadyMs = millis();   // see configureAccel() for why
    return true;
}

// ─── initialisation ───────────────────────────────────────────────────────────

/** @brief True when @p address is already spoken for by another project device. */
static bool addressIsAllocated(uint8_t address){
    return (address == GPS_DEFAULT_I2C_ADDRESS) || (address == SEGLED_ADDRESS);
}

/** @brief Collapses the two ready flags into the documented return code. */
static IMUReturnStatus deviceStatus(const IMUDevice &dev){
    if (dev.accelReady && dev.magReady) return IMUReturnStatus::OK;
    if (dev.accelReady || dev.magReady) return IMUReturnStatus::PARTIAL;
    return IMUReturnStatus::NOK_INIT_FAILED;
}

IMUReturnStatus initializeIMU(IMUDevice &dev, uint8_t accelAddress, uint8_t magAddress){
    dev.accelAddress  = accelAddress;
    dev.magAddress    = magAddress;
    dev.accelReady    = false;
    dev.magReady      = false;
    dev.accelFaults   = 0u;
    dev.magFaults     = 0u;
    // Cleared only here, never in recoverIMU(): these count the whole run, so a
    // part that has been retired and reconfigured five times still shows it.
    dev.accelIOErrors = 0u;
    dev.magIOErrors   = 0u;
    dev.accelScaleMs2 = NAN;
    dev.gyroScaleDps  = NAN;
    dev.magScaleUt    = NAN;

    if (!isUsableI2CAddress(accelAddress) || !isUsableI2CAddress(magAddress) ||
        (accelAddress == magAddress) ||
        addressIsAllocated(accelAddress) || addressIsAllocated(magAddress)){
        return IMUReturnStatus::NOK_ADDRESS_CONFLICT;
    }

    // The bus manager unwedges before the session's first transaction, whichever
    // client gets here first.  If it reports Stuck, transacting anyway risks
    // hanging in the SAMD driver's unbounded flag waits, so stop here instead.
    if (i2cBusBegin() != I2CBusState::Ready) return IMUReturnStatus::NOK_BUS_STUCK;

    // Presence and configuration are separate questions, and conflating them
    // sends a technician looking for the wrong fault: NOK_INIT_FAILED says
    // nothing answered at the address, NOK_CONFIG_FAILED says the part is right
    // there and refused its settings — a bus-integrity problem, not a wiring one.
    const bool accelPresent = i2cProbeAddress(accelAddress);
    const bool magPresent   = i2cProbeAddress(magAddress);

    dev.accelReady = configureAccel(dev);
    dev.magReady   = configureMag(dev);

    if (!dev.accelReady && !dev.magReady && (accelPresent || magPresent)){
        return IMUReturnStatus::NOK_CONFIG_FAILED;
    }
    return deviceStatus(dev);
}

IMUReturnStatus initializeIMU(IMUDevice &dev){
    return initializeIMU(dev, IMU_ACCEL_I2C_ADDRESS, IMU_MAG_I2C_ADDRESS);
}

IMUReturnStatus recoverIMU(IMUDevice &dev){
    if (dev.accelReady && dev.magReady) return IMUReturnStatus::OK;

    if (i2cBusBegin() != I2CBusState::Ready) return IMUReturnStatus::NOK_BUS_STUCK;

    // Force a bus-level recovery only when NOTHING is working.  With one device
    // still answering, the bus is demonstrably not stuck, and the recovery
    // clocking would push a STOP into a bus the healthy device is sharing for
    // no reason.
    if (!dev.accelReady && !dev.magReady){
        if (i2cBusRecover() != I2CBusState::Ready) return IMUReturnStatus::NOK_BUS_STUCK;
    }

    if (!dev.accelReady){
        dev.accelReady = configureAccel(dev);
        if (dev.accelReady) dev.accelFaults = 0u;
    }
    if (!dev.magReady){
        dev.magReady = configureMag(dev);
        if (dev.magReady) dev.magFaults = 0u;
    }

    return deviceStatus(dev);
}

void initIMUData(IMUData &data){
    data.accelX = NAN;
    data.accelY = NAN;
    data.accelZ = NAN;

    data.gyroX = NAN;
    data.gyroY = NAN;
    data.gyroZ = NAN;

    data.magX = NAN;
    data.magY = NAN;
    data.magZ = NAN;

    data.temperatureC = NAN;

    data.accelSampleMs = 0u;
    data.gyroSampleMs  = 0u;
    data.tempSampleMs  = 0u;
    data.magSampleMs   = 0u;

    data.accelValid = false;
    data.gyroValid  = false;
    data.magValid   = false;
    data.tempValid  = false;
    data.devicePresent     = false;
    data.allDevicesPresent = false;
}

// ─── reading ──────────────────────────────────────────────────────────────────

static void invalidateAccel(IMUData &data){
    data.accelX = NAN;
    data.accelY = NAN;
    data.accelZ = NAN;
    data.accelValid = false;
}

static void invalidateGyro(IMUData &data){
    data.gyroX = NAN;
    data.gyroY = NAN;
    data.gyroZ = NAN;
    data.gyroValid = false;
}

static void invalidateTemp(IMUData &data){
    data.temperatureC = NAN;
    data.tempValid = false;
}

static void invalidateMagnetic(IMUData &data){
    data.magX = NAN;
    data.magY = NAN;
    data.magZ = NAN;
    data.magValid = false;
}

/**
 * @brief Records a failed bus read and retires the device once faults pile up.
 *
 * One NACK is a glitch worth riding out; @c IMU_MAX_CONSECUTIVE_FAULTS in a row
 * is a sensor that is gone.  Retiring it is what makes @c isIMUDegraded() true
 * and hands the caller a defined recovery path instead of an endless retry
 * inside the read.
 */
static void noteFault(uint8_t &faults, uint16_t &ioErrors, bool &ready){
    if (faults < 0xFFu) faults++;
    // Saturating rather than wrapping: "65535" reads as "a lot, stopped
    // counting", where a wrapped 3 would read as a nearly clean run.
    if (ioErrors < 0xFFFFu) ioErrors++;
    if (faults >= IMU_MAX_CONSECUTIVE_FAULTS) ready = false;
}

/** @brief True when a timestamped reading is older than the freshness window. */
static inline bool isExpired(uint32_t sampleMs, uint32_t nowMs){
    // Unsigned subtraction, so this stays correct across the millis() rollover
    // at 49.7 days — a comparison against (sampleMs + window) would not.
    return (nowMs - sampleMs) > IMU_MAX_DATA_AGE_MS;
}

/**
 * @brief True when a channel has not signalled data-ready for too long.
 *
 * Distinct from @c isExpired(), and the distinction matters.  Expiry describes
 * the DATA — it goes NaN and the consumer is told so.  A stall describes the
 * DEVICE, and is the only way to catch a channel that dies while its sibling
 * keeps working: the part still ACKs every status read, so the fault counters
 * never trip and the device never looks lost, yet one signal is gone for good.
 * Only a stall can force the reconfigure that might actually bring it back.
 */
static inline bool isStalled(uint32_t lastReadyMs, uint32_t nowMs){
    return (nowMs - lastReadyMs) > IMU_MAX_CHANNEL_STALL_MS;
}

/** @brief Ages out any channel past its freshness window, without touching the bus. */
static void expireChannels(const IMUDevice &dev, IMUData &data, uint32_t now){
    if (!dev.accelReady || isExpired(data.accelSampleMs, now)) invalidateAccel(data);
    if (!dev.accelReady || isExpired(data.gyroSampleMs, now))  invalidateGyro(data);
    if (!dev.accelReady || isExpired(data.tempSampleMs, now))  invalidateTemp(data);
    if (!dev.magReady   || isExpired(data.magSampleMs, now))   invalidateMagnetic(data);
}

/** @brief Publishes hardware presence from device state, not from this poll's luck. */
static void publishPresence(const IMUDevice &dev, IMUData &data){
    data.devicePresent     = dev.accelReady || dev.magReady;
    data.allDevicesPresent = dev.accelReady && dev.magReady;
}

IMUReturnStatus getIMUData(IMUDevice &dev, IMUData &data){
    const uint32_t now = millis();
    bool fresh = false;

    // Checked on EVERY poll, not just at bring-up.  A slave that browns out or
    // resets mid-drive holds SDA from that moment, and the reads below go
    // straight into SERCOM::startTransmissionWIRE()'s undeadlined
    // `while (!isBusIdleWIRE() && !isBusOwnerWIRE());`.  Without this the only
    // thing that ends the hang is the watchdog, and a reboot is not the required
    // response to a peripheral fault — staying up and logging it is.
    //
    // A stuck bus is charged to NEITHER device's fault counters.  The bus is the
    // one thing both parts share, so counting it against them would retire two
    // healthy sensors for a fault that is not theirs, and then hide the real
    // cause behind an IMU that reads as absent.
    if (i2cBusBegin() != I2CBusState::Ready){
        expireChannels(dev, data, now);
        publishPresence(dev, data);
        return IMUReturnStatus::NOK_BUS_STUCK;
    }

    // ── LSM6DSOX: accelerometer, gyroscope, die temperature ──────────────────
    if (dev.accelReady){
        uint8_t status = 0u;
        if (!readReg8(dev.accelAddress, SOX_REG_STATUS, status) ||
            ((status & SOX_STATUS_RESERVED_MASK) != 0u)){
            noteFault(dev.accelFaults, dev.accelIOErrors, dev.accelReady);
        } else if (((status & (SOX_STATUS_XLDA | SOX_STATUS_GDA | SOX_STATUS_TDA)) == 0u)){
            // A clean status read with nothing ready is a SUCCESSFUL transaction
            // — the part answered, it simply has no new sample yet.  The fault
            // counter tracks CONSECUTIVE failures, so it has to be cleared here
            // too; otherwise occasional glitches accumulate across thousands of
            // healthy polls and eventually retire a device that is working fine.
            dev.accelFaults = 0u;
        } else {
            // One burst covers temperature, gyro and accelerometer: they are
            // contiguous, and Block Data Update freezes the whole set until it
            // has been read out.
            uint8_t buf[14];
            if (!readRegs(dev.accelAddress, SOX_REG_OUT_TEMP_L, buf, sizeof(buf))){
                noteFault(dev.accelFaults, dev.accelIOErrors, dev.accelReady);
            } else {
                dev.accelFaults = 0u;

                // Each channel is validated from ITS OWN ready bit.  The burst
                // returns all 14 bytes whichever bit triggered it, but a gyro
                // that has stopped converting still has its previous sample
                // sitting in those registers — certifying it because the
                // accelerometer happened to be ready would republish an old
                // reading as a new one.
                if ((status & SOX_STATUS_TDA) != 0u){
                    data.temperatureC = (static_cast<float>(toInt16LE(&buf[0])) / SOX_TEMP_LSB_PER_C)
                                        + SOX_TEMP_OFFSET_C;
                    data.tempSampleMs = now;
                    data.tempValid = true;
                    fresh = true;
                }

                if ((status & SOX_STATUS_GDA) != 0u){
                    data.gyroX = static_cast<float>(toInt16LE(&buf[2])) * dev.gyroScaleDps;
                    data.gyroY = static_cast<float>(toInt16LE(&buf[4])) * dev.gyroScaleDps;
                    data.gyroZ = static_cast<float>(toInt16LE(&buf[6])) * dev.gyroScaleDps;
                    data.gyroSampleMs = now;
                    data.gyroValid = true;
                    dev.lastGyroReadyMs = now;
                    fresh = true;
                }

                if ((status & SOX_STATUS_XLDA) != 0u){
                    data.accelX = static_cast<float>(toInt16LE(&buf[8]))  * dev.accelScaleMs2;
                    data.accelY = static_cast<float>(toInt16LE(&buf[10])) * dev.accelScaleMs2;
                    data.accelZ = static_cast<float>(toInt16LE(&buf[12])) * dev.accelScaleMs2;
                    data.accelSampleMs = now;
                    data.accelValid = true;
                    dev.lastAccelReadyMs = now;
                    fresh = true;
                }
            }
        }

        // A channel that has gone quiet for far longer than its output period,
        // on a part that is otherwise answering every status read, is a broken
        // device rather than a slow one.  Retiring it here is what makes
        // isIMUDegraded() true and gets it reconfigured; expiry alone would
        // blank the signal and leave it blank for the rest of the drive.
        //
        // Temperature is deliberately NOT a demotion trigger: it is a
        // diagnostic, it runs at a lower rate than the inertial channels, and
        // resetting a working accelerometer and gyroscope over it would trade a
        // real signal for a nice-to-have.
        if (dev.accelReady &&
            (isStalled(dev.lastAccelReadyMs, now) || isStalled(dev.lastGyroReadyMs, now))){
            dev.accelReady = false;
        }
    }

    // ── LIS3MDL: magnetometer ────────────────────────────────────────────────
    if (dev.magReady){
        uint8_t status = 0u;
        if (!readReg8(dev.magAddress, MDL_REG_STATUS, status)){
            noteFault(dev.magFaults, dev.magIOErrors, dev.magReady);
        } else if ((status & MDL_STATUS_ZYXDA) == 0u){
            dev.magFaults = 0u;   // answered, just nothing new — see the accel branch
        } else {
            uint8_t buf[6];
            const uint8_t burstReg =
                static_cast<uint8_t>(MDL_REG_OUT_X_L | MDL_AUTO_INCREMENT);
            if (!readRegs(dev.magAddress, burstReg, buf, sizeof(buf))){
                noteFault(dev.magFaults, dev.magIOErrors, dev.magReady);
            } else {
                dev.magFaults = 0u;

                data.magX = static_cast<float>(toInt16LE(&buf[0])) * dev.magScaleUt;
                data.magY = static_cast<float>(toInt16LE(&buf[2])) * dev.magScaleUt;
                data.magZ = static_cast<float>(toInt16LE(&buf[4])) * dev.magScaleUt;

                data.magSampleMs = now;
                data.magValid = true;
                dev.lastMagReadyMs = now;
                fresh = true;
            }
        }
        // The LIS3MDL STATUS_REG has no reserved bits, so unlike the LSM6DSOX
        // there is no all-ones signature to test: 0xFF is a legitimate "all
        // axes ready, all overrun".  A magnetometer that answers but has quietly
        // stopped converting — dropped back to power-down, say — shows up only
        // as a stall, which is what this catches.
        if (dev.magReady && isStalled(dev.lastMagReadyMs, now)){
            dev.magReady = false;
        }
    }

    // Every channel aged out in one place, AFTER both device blocks have had
    // their say, so the stall demotions above are already reflected in the ready
    // flags this reads.
    expireChannels(dev, data, now);
    // Hardware presence, taken from the device state rather than from whether
    // this particular poll happened to land on fresh data.
    publishPresence(dev, data);

    if (!dev.accelReady && !dev.magReady) return IMUReturnStatus::NOK_LINK_LOST;
    if (!dev.accelReady || !dev.magReady) return IMUReturnStatus::PARTIAL;
    return fresh ? IMUReturnStatus::OK : IMUReturnStatus::DATA_STALE;
}

bool isIMULinkLost(const IMUDevice &dev){
    return !dev.accelReady && !dev.magReady;
}

bool isIMUDegraded(const IMUDevice &dev){
    return !dev.accelReady || !dev.magReady;
}
