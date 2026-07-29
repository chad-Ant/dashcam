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
/// Retained for the bring-up self-test; the steady-state path reads the FIFO.
static constexpr uint8_t SOX_REG_OUT_TEMP_L  = 0x20u;

// ─── LSM6DSOX FIFO ────────────────────────────────────────────────────────────

/// FIFO_CTRL1: WTM[7:0].  Unused — the drain polls the fill level instead of
/// waiting on a watermark interrupt, because no INT pin is wired on this build.
static constexpr uint8_t SOX_REG_FIFO_CTRL1  = 0x07u;
/// FIFO_CTRL2: STOP_ON_WTM[7], FIFO_COMPR_RT_EN[6], ODRCHG_EN[4], WTM[8].
static constexpr uint8_t SOX_REG_FIFO_CTRL2  = 0x08u;
/// FIFO_CTRL3: BDR_GY[7:4], BDR_XL[3:0] — per-sensor batch data rates.
static constexpr uint8_t SOX_REG_FIFO_CTRL3  = 0x09u;
/// FIFO_CTRL4: DEC_TS_BATCH[7:6], ODR_T_BATCH[5:4], FIFO_MODE[2:0].
static constexpr uint8_t SOX_REG_FIFO_CTRL4  = 0x0Au;
/// FIFO_STATUS1: DIFF_FIFO[7:0] — unread words currently buffered.
static constexpr uint8_t SOX_REG_FIFO_STATUS1 = 0x3Au;
/// FIFO_STATUS2: WTM_IA[7], OVR_IA[6], FULL_IA[5], COUNTER_BDR_IA[4],
/// OVR_LATCHED[3], reserved[2], DIFF_FIFO[9:8][1:0].
static constexpr uint8_t SOX_FIFO_ST2_DIFF_MASK   = 0x03u;
/// Bit 2 is reserved and reads back zero, so a set bit here means the byte did
/// not come from a healthy part — 0xFF in particular.
static constexpr uint8_t SOX_FIFO_ST2_RESERVED    = 0x04u;
static constexpr uint8_t SOX_FIFO_ST2_OVR_LATCHED = 0x08u;
static constexpr uint8_t SOX_FIFO_ST2_OVR_IA      = 0x40u;

/// First byte of a FIFO word: TAG_SENSOR[7:3], TAG_CNT[2:1], TAG_PARITY[0].
/// A word is this tag plus six data bytes, read as one seven-byte burst.
static constexpr uint8_t SOX_REG_FIFO_DATA_OUT_TAG = 0x78u;
static constexpr uint8_t SOX_FIFO_WORD_BYTES       = 7u;

/// TAG_SENSOR values for the three streams enabled here.  Anything else — the
/// compression, sensor-hub and step-counter tags — is skipped rather than
/// mis-decoded, since none of those features is turned on and a word carrying
/// one would mean the configuration is not what this code believes.
static constexpr uint8_t SOX_TAG_GYRO  = 0x01u;
static constexpr uint8_t SOX_TAG_ACCEL = 0x02u;
static constexpr uint8_t SOX_TAG_TEMP  = 0x03u;
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
/// ODR field value for 26 Hz — the low-power rate.
///
/// Chosen over the lower 12.5 Hz step so the 38.5 ms output period stays shorter
/// than the 50 ms poll interval: every poll then finds a sample waiting, and the
/// data-age and stall logic behave exactly as they do at 104 Hz instead of
/// needing a second set of thresholds for the second mode.
static constexpr uint8_t SOX_ODR_26_HZ    = 0x02u;
/// FS_XL field value for +/-8 g (00=2g, 01=16g, 10=4g, 11=8g — not in order).
static constexpr uint8_t SOX_FS_XL_8G     = 0x03u;
/// CTRL2_G[3:0] for +/-500 dps: FS_G=01, FS_125=0.
static constexpr uint8_t SOX_FS_G_500DPS  = 0x04u;

static constexpr uint8_t SOX_CTRL1_XL_VALUE =
    static_cast<uint8_t>((SOX_ODR_104_HZ << 4) | (SOX_FS_XL_8G << 2));
static constexpr uint8_t SOX_CTRL2_G_VALUE =
    static_cast<uint8_t>((SOX_ODR_104_HZ << 4) | SOX_FS_G_500DPS);

/// BDR field value for 104 Hz, matching the ODR so nothing is decimated on its
/// way into the FIFO.  Batching below the ODR would silently reintroduce the
/// sample loss the FIFO was added to remove.
static constexpr uint8_t SOX_BDR_104_HZ    = 0x04u;
/// ODR_T_BATCH field value for 12.5 Hz.
///
/// The lowest available rate is 1.6 Hz, and it cannot be used: a 625 ms period
/// is longer than IMU_MAX_DATA_AGE_MS, so every temperature reading would be
/// stale on arrival and tempValid could never be true.  12.5 Hz gives an 80 ms
/// period, comfortably inside the window, and costs 12.5 of 220.5 words/s.
static constexpr uint8_t SOX_ODR_T_12_5_HZ = 0x02u;
/// FIFO_MODE for Continuous ("stream") — when full, the OLDEST word is dropped.
///
/// The alternative, plain FIFO mode, stops collecting when full and would hold a
/// snapshot of whatever happened at the moment the drain fell behind, then keep
/// it forever.  For a dashcam the newest data is the data worth having, so the
/// loss is taken at the old end.
static constexpr uint8_t SOX_FIFO_MODE_STREAM = 0x06u;
static constexpr uint8_t SOX_FIFO_MODE_BYPASS = 0x00u;

static constexpr uint8_t SOX_FIFO_CTRL3_VALUE =
    static_cast<uint8_t>((SOX_BDR_104_HZ << 4) | SOX_BDR_104_HZ);
static constexpr uint8_t SOX_FIFO_CTRL4_VALUE =
    static_cast<uint8_t>((SOX_ODR_T_12_5_HZ << 4) | SOX_FIFO_MODE_STREAM);

/// Low-power counterparts.  Same full-scale ranges — only the rate changes, so a
/// mode switch cannot silently rescale the readings either side of it.
static constexpr uint8_t SOX_CTRL1_XL_LOWPOWER =
    static_cast<uint8_t>((SOX_ODR_26_HZ << 4) | (SOX_FS_XL_8G << 2));
static constexpr uint8_t SOX_CTRL2_G_LOWPOWER =
    static_cast<uint8_t>((SOX_ODR_26_HZ << 4) | SOX_FS_G_500DPS);

/// CTRL6_C: XL_HM_MODE[4] — set to LEAVE accelerometer high-performance mode.
static constexpr uint8_t SOX_REG_CTRL6_C     = 0x15u;
static constexpr uint8_t SOX_CTRL6_XL_HM_OFF = 0x10u;
/// CTRL7_G: G_HM_MODE[7] — set to LEAVE gyroscope high-performance mode.
static constexpr uint8_t SOX_REG_CTRL7_G     = 0x16u;
static constexpr uint8_t SOX_CTRL7_G_HM_OFF  = 0x80u;

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

static constexpr uint32_t IMU_PEAK_BUCKET_MS = IMU_PEAK_WINDOW_MS / IMU_PEAK_BUCKETS;

/** @brief Empties every bucket — after a mode change, a gap, or at bring-up. */
static void resetPeakRing(IMUDevice &dev, uint32_t now){
    for (uint8_t i = 0u; i < IMU_PEAK_BUCKETS; i++){
        dev.accelPeakSq[i]  = NAN;
        dev.gyroPeakSq[i]   = NAN;
        dev.peakBucketMs[i] = now;
    }
    dev.peakBucketHead = 0u;
}


/**
 * @brief Empties the FIFO and restarts it, after data has already been lost.
 *
 * Called on overrun, where the buffer holds up to 2.3 s of backlog.  Draining
 * that backlog would take several polls and deliver samples as "current" that
 * are seconds old, so the newest data is worth more than the queue: bypass to
 * discard, stream to resume.  The peak for the affected window is wrong either
 * way — which is what @c IMUData::dataGap exists to say.
 */
static bool resetAccelFifo(IMUDevice &dev){
    const uint8_t address = dev.accelAddress;
    // Read back, not fire-and-forget.  These two writes ARE the recovery, and an
    // unverified write only proves the part ACKed its address — a device that
    // acknowledges and then ignores the mode change would be treated as
    // successfully recovered on every gap, forever, while the FIFO stayed
    // exactly as wedged as before.  The failure would present as endless data
    // gaps with no other symptom.
    if (!writeVerifyReg8(address, SOX_REG_FIFO_CTRL4, SOX_FIFO_MODE_BYPASS)) return false;
    return writeVerifyReg8(address, SOX_REG_FIFO_CTRL4, SOX_FIFO_CTRL4_VALUE);
}

/**
 * @brief Writes the rate, power and FIFO registers for one sampling mode.
 *
 * The order is deliberate: FIFO to Bypass FIRST, so collection stops and the
 * buffer empties before the rates move.  Changing an ODR while the FIFO is
 * streaming leaves words in the buffer that were sampled at the old rate, and
 * nothing in a FIFO word says which rate produced it — they would be decoded as
 * current data taken at the new one.
 *
 * @return @c true when every register read back as written.
 */
static bool applySampleMode(IMUDevice &dev, IMUSampleMode mode){
    const uint8_t address = dev.accelAddress;

    // The FIRST write puts the FIFO into Bypass, so from here until the last
    // write lands the hardware matches NEITHER mode.  dev.mode is only committed
    // at the very end, which means a failure in between leaves software decoding
    // against a configuration the part no longer has — in the worst case polling
    // a bypassed FIFO forever, seeing zero words, and being retired by the stall
    // check for a fault that is really a half-finished write.
    //
    // Marking the part unconfigured up front makes that state impossible to
    // mistake for a working one: if any write below fails, the function returns
    // with accelReady false, and the caller's existing IMU_RETRY_MS path does
    // ONE full reconfiguration instead of hammering a half-configured device
    // every 50 ms.  On success it is restored before returning.
    const bool wasReady = dev.accelReady;
    dev.accelReady = false;

    // Bypass also FLUSHES: the buffer empties on entry, so the first word read
    // after this function is guaranteed to belong to the configuration it just
    // installed rather than to whatever the previous one left behind.
    if (!writeVerifyReg8(address, SOX_REG_FIFO_CTRL4, SOX_FIFO_MODE_BYPASS)) return false;

    // No watermark and no compression, in either mode.  The watermark exists to
    // raise an interrupt and no INT pin is wired on this build, so the fill
    // level is polled instead; compression is a lossy encoding that would trade
    // away the sample fidelity this whole arrangement is for, to buy FIFO depth
    // already in surplus.
    if (!writeVerifyReg8(address, SOX_REG_FIFO_CTRL1, 0x00u)) return false;
    if (!writeVerifyReg8(address, SOX_REG_FIFO_CTRL2, 0x00u)) return false;

    const bool lowPower = (mode == IMUSampleMode::LowPower);

    // High-performance mode off is where the current saving actually comes from;
    // the lower ODR alone changes little, because the analogue front end stays
    // fully powered in HP mode regardless of how often it is sampled.
    if (!writeVerifyReg8(address, SOX_REG_CTRL6_C,
                         lowPower ? SOX_CTRL6_XL_HM_OFF : 0x00u)) return false;
    if (!writeVerifyReg8(address, SOX_REG_CTRL7_G,
                         lowPower ? SOX_CTRL7_G_HM_OFF : 0x00u)) return false;

    if (!writeVerifyReg8(address, SOX_REG_CTRL1_XL,
                         lowPower ? SOX_CTRL1_XL_LOWPOWER : SOX_CTRL1_XL_VALUE)) return false;
    if (!writeVerifyReg8(address, SOX_REG_CTRL2_G,
                         lowPower ? SOX_CTRL2_G_LOWPOWER : SOX_CTRL2_G_VALUE)) return false;

    // Batching and stream mode only in Fifo mode; LowPower reads the output
    // registers directly and leaves the FIFO bypassed.
    if (!lowPower){
        if (!writeVerifyReg8(address, SOX_REG_FIFO_CTRL3, SOX_FIFO_CTRL3_VALUE)) return false;
        if (!writeVerifyReg8(address, SOX_REG_FIFO_CTRL4, SOX_FIFO_CTRL4_VALUE)) return false;
    }

    // Committed together, and only now: the registers all read back as written,
    // so the part and dev.mode agree from this instant on.
    dev.mode       = mode;
    dev.accelReady = wasReady;
    return true;
}

IMUSampleMode imuSampleMode(const IMUDevice &dev){
    return dev.mode;
}

IMUReturnStatus setIMUSampleMode(IMUDevice &dev, IMUSampleMode mode){
    // Same reasoning as recoverIMU(): this writes seven registers, so it is a
    // bus transaction like any other and must respect the quarantine.
    if (dev.quarantined) return IMUReturnStatus::NOK_LINK_LOST;
    if (dev.mode == mode) return IMUReturnStatus::OK;
    if (!dev.accelReady)  return IMUReturnStatus::NOK_ACCEL_MISSING;
    if (i2cBusBegin() != I2CBusState::Ready) return IMUReturnStatus::NOK_BUS_STUCK;

    if (!applySampleMode(dev, mode)) return IMUReturnStatus::NOK_CONFIG_FAILED;

    // The rate just changed, so the stall clocks have to be forgiven: a part
    // moving from 104 Hz to 26 Hz produces nothing for up to 38 ms, and a clock
    // left running across the switch could read that as a dead channel.
    const uint32_t now = millis();
    dev.lastAccelReadyMs = now;
    dev.lastGyroReadyMs  = now;

    // Peaks are discarded, not carried over.  They were folded from samples the
    // part took under the OLD rate and power mode, and the digital filters need
    // a few output periods to settle after an ODR change — so the first samples
    // either side of a switch are not comparable with each other.  Keeping the
    // old peak would attribute a 104 Hz measurement to a 26 Hz window, and
    // keeping the settling samples would report the switch itself as motion.
    resetPeakRing(dev, now);
    return IMUReturnStatus::OK;
}

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

    // Rates, power mode and FIFO all come from applySampleMode(), which is the
    // single place that knows what each mode means.  Writing CTRL1_XL here as
    // well would fork that knowledge in two, and the copy that is wrong is
    // always the one nobody re-reads — a recovery would then silently restore
    // 104 Hz on a parked car that had deliberately been put into low power.
    if (!applySampleMode(dev, dev.mode)) return false;

    // Both modes use the SAME full-scale ranges, so the scale factors are a
    // property of the part's configuration rather than of the mode — which is
    // what lets a mode switch happen mid-drive without rescaling anything.
    dev.accelScaleMs2 = accelScaleMs2((SOX_CTRL1_XL_VALUE >> 2) & 0x03u);
    dev.gyroScaleDps  = gyroScaleDps(SOX_CTRL2_G_VALUE & 0x0Fu);
    static_assert(((SOX_CTRL1_XL_VALUE >> 2) & 0x03u) == ((SOX_CTRL1_XL_LOWPOWER >> 2) & 0x03u),
                  "accelerometer full scale must match across sample modes");
    static_assert((SOX_CTRL2_G_VALUE & 0x0Fu) == (SOX_CTRL2_G_LOWPOWER & 0x0Fu),
                  "gyroscope full scale must match across sample modes");

    // Seed the stall clocks from now.  Without this they read zero, and the
    // first getIMUData() would measure a stall of however long the board had
    // been powered and immediately declare a freshly configured part faulty.
    const uint32_t now = millis();
    dev.lastAccelReadyMs = now;
    dev.lastGyroReadyMs  = now;

    // Buckets start empty rather than at zero: zero is a legitimate reading, and
    // seeding with it would publish a 0 m/s2 peak for a stationary vehicle that
    // is in fact sitting in a 9.81 field.  NAN says "nothing measured yet".
    resetPeakRing(dev, now);
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

void imuMarkAbsent(IMUDevice &dev, uint8_t accelAddress, uint8_t magAddress){
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
    dev.fifoOverruns  = 0u;
    dev.fifoGapFlushes = 0u;
    dev.lastOverrunMs  = millis();
    dev.gapFlagActive  = false;
    dev.gapFlagUntilMs = millis();
    dev.quarantined   = false;
    // Full capture until something establishes the vehicle is parked.  The safe
    // default is the expensive one: starting in LowPower would mean a boot that
    // happens to coincide with a collision records it at reduced fidelity.
    dev.mode          = IMUSampleMode::Fifo;
    dev.accelScaleMs2 = NAN;
    dev.gyroScaleDps  = NAN;
    dev.magScaleUt    = NAN;
    resetPeakRing(dev, millis());
}

void imuQuarantine(IMUDevice &dev){
    imuMarkAbsent(dev, IMU_ACCEL_I2C_ADDRESS, IMU_MAG_I2C_ADDRESS);
    // Terminal for the boot.  isIMUDegraded() is what schedules recoverIMU(),
    // and recovery transacts — so without this flag the caller's retry timer
    // would walk straight back into the hang that caused the reset, which is the
    // loop the quarantine exists to break.
    dev.quarantined = true;
}

IMUReturnStatus initializeIMU(IMUDevice &dev, uint8_t accelAddress, uint8_t magAddress){
    imuMarkAbsent(dev, accelAddress, magAddress);

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
    // Guarded here as well as at the caller.  isIMUDegraded() already returns
    // false while quarantined so the scheduler will not call this, but recovery
    // is the single most dangerous thing to run on a bus that just hung the
    // board — it probes, resets and reconfigures both parts — and it must not
    // depend on one caller remembering to ask the right question first.
    if (dev.quarantined) return IMUReturnStatus::NOK_LINK_LOST;
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

    data.accelPeakMs2 = NAN;
    data.gyroPeakDps  = NAN;
    data.dataGap      = false;
    data.lowPower     = false;

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
    // The peak goes with the axes.  It is derived from the same samples, so a
    // peak surviving its own channel's expiry would be the one number on the
    // frame still claiming a measurement after the sensor stopped supplying one.
    data.accelPeakMs2 = NAN;
    data.accelValid = false;
}

static void invalidateGyro(IMUData &data){
    data.gyroX = NAN;
    data.gyroY = NAN;
    data.gyroZ = NAN;
    data.gyroPeakDps = NAN;
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
static void expireChannels(IMUDevice &dev, IMUData &data, uint32_t now){
    if (!dev.accelReady || isExpired(data.accelSampleMs, now)) invalidateAccel(data);
    if (!dev.accelReady || isExpired(data.gyroSampleMs, now))  invalidateGyro(data);
    if (!dev.accelReady || isExpired(data.tempSampleMs, now))  invalidateTemp(data);
    if (!dev.magReady   || isExpired(data.magSampleMs, now))   invalidateMagnetic(data);

    // The overrun notice is HELD for the peak window rather than cleared on the
    // next poll.  Polls run at 20 Hz and telemetry at 10 Hz, so a flag that
    // lasted one poll would be missed by half the frames — and the frames it
    // would be missed by are exactly the ones whose peak is untrustworthy.
    // An explicit deadline compared with SIGNED arithmetic, not "counter is
    // nonzero AND the timestamp looks recent".  That older form resurrects the
    // flag at the millis() rollover: 49.7 days after a gap, `now` comes back
    // around to the neighbourhood of lastOverrunMs, the elapsed test reads as
    // ~0 again, and a long-finished gap is republished for 250 ms.  Rare, but it
    // is a false report of missing data on a system whose whole point is not
    // making those.  A deadline goes stale exactly once and stays stale.
    if (dev.gapFlagActive && (static_cast<int32_t>(now - dev.gapFlagUntilMs) >= 0)){
        dev.gapFlagActive = false;
    }
    data.dataGap = dev.gapFlagActive;
}

/** @brief Publishes hardware presence from device state, not from this poll's luck. */
static void publishPresence(const IMUDevice &dev, IMUData &data){
    data.devicePresent     = dev.accelReady || dev.magReady;
    data.allDevicesPresent = dev.accelReady && dev.magReady;
    data.lowPower          = (dev.mode == IMUSampleMode::LowPower);
}

// ─── FIFO drain ───────────────────────────────────────────────────────────────

/**
 * @brief Folds one sample's squared magnitude into a windowed running peak.
 *
 * Replaces the stored peak when the new sample is larger, OR when the stored one
 * has aged past @c IMU_PEAK_WINDOW_MS — the second half is what stops a single
 * hard impact pinning the reading high for the rest of the drive.
 *
 * Squared throughout; the caller takes one square root per channel per poll
 * rather than one per sample.
 */
/**
 * @brief Rotates the ring so the head bucket covers @p now, retiring stale ones.
 *
 * Advances at most @c IMU_PEAK_BUCKETS steps however long the gap: after a full
 * window of silence every bucket is stale anyway, so spinning once per elapsed
 * bucket would be wasted work with a peripheral-controlled bound.
 */
static void rollPeakRing(IMUDevice &dev, uint32_t now){
    for (uint8_t i = 0u; i < IMU_PEAK_BUCKETS; i++){
        if ((now - dev.peakBucketMs[dev.peakBucketHead]) < IMU_PEAK_BUCKET_MS) return;

        dev.peakBucketHead = static_cast<uint8_t>((dev.peakBucketHead + 1u) % IMU_PEAK_BUCKETS);
        dev.accelPeakSq[dev.peakBucketHead]  = NAN;
        dev.gyroPeakSq[dev.peakBucketHead]   = NAN;
        dev.peakBucketMs[dev.peakBucketHead] = now;
    }
}

/** @brief Folds one sample's squared magnitude into the current bucket. */
static void notePeakSq(float valueSq, float *ring, uint8_t head){
    if (isnan(ring[head]) || (valueSq > ring[head])) ring[head] = valueSq;
}

/**
 * @brief Largest value across the buckets still inside the window.
 *
 * @return @c NAN when every bucket is empty, which is honest: no sample has
 *         arrived recently enough to support a peak.
 */
static float ringMaxSq(const IMUDevice &dev, const float *ring, uint32_t now){
    float best = NAN;
    for (uint8_t i = 0u; i < IMU_PEAK_BUCKETS; i++){
        if (isnan(ring[i])) continue;
        if ((now - dev.peakBucketMs[i]) > IMU_PEAK_WINDOW_MS) continue;
        if (isnan(best) || (ring[i] > best)) best = ring[i];
    }
    return best;
}

/**
 * @brief Converts the tracked squared peaks into the published magnitudes.
 *
 * One square root per channel per poll, taken once the whole window has been
 * folded in rather than once per sample — on a Cortex-M0+ every sqrtf is a
 * software routine, so where it is called from is not a detail.
 */
static void publishPeaks(const IMUDevice &dev, IMUData &data, uint32_t now){
    const float aSq = ringMaxSq(dev, dev.accelPeakSq, now);
    const float gSq = ringMaxSq(dev, dev.gyroPeakSq,  now);
    data.accelPeakMs2 = isnan(aSq) ? NAN : sqrtf(aSq);
    data.gyroPeakDps  = isnan(gSq) ? NAN : sqrtf(gSq);
}

/**
 * @brief Reads the FIFO fill level and overrun state.
 *
 * @param[out] words    Unread words currently buffered.
 * @param[out] overrun  True when the part reports samples were overwritten.
 * @return @c false when the status bytes could not be read or did not come from
 *         a healthy part.
 */
static bool readFifoStatus(const IMUDevice &dev, uint16_t &words, bool &overrun){
    uint8_t st[2] = { 0u, 0u };
    if (!readRegs(dev.accelAddress, SOX_REG_FIFO_STATUS1, st, sizeof(st))) return false;

    // Reserved bit 2 reads back zero on a healthy part, so a set bit is the
    // all-ones signature of a read that returned nothing — the same trick the
    // old STATUS_REG path used, and the reason it is worth having here too.
    if ((st[1] & SOX_FIFO_ST2_RESERVED) != 0u) return false;

    const uint16_t count =
        static_cast<uint16_t>((static_cast<uint16_t>(st[1] & SOX_FIFO_ST2_DIFF_MASK) << 8) | st[0]);
    // A count past the physical depth cannot be true, so it is corruption rather
    // than a very full buffer.  Rejecting it stops a bogus value driving the
    // drain loop, which is the one place a peripheral gets to influence how much
    // work this function does.
    if (count > IMU_FIFO_DEPTH_WORDS) return false;

    words   = count;
    overrun = ((st[1] & (SOX_FIFO_ST2_OVR_IA | SOX_FIFO_ST2_OVR_LATCHED)) != 0u);
    return true;
}

/**
 * @brief Empties up to @c IMU_FIFO_MAX_WORDS_PER_POLL words, decoding each by tag.
 *
 * Every sample is folded into the peak; only the last of each kind is published
 * as the current reading.  That asymmetry is the point of the FIFO — publishing
 * at 10 Hz while measuring at 104 Hz is fine, so long as nothing between frames
 * is thrown away unexamined.
 *
 * Words are read one at a time, seven bytes each, rather than as one long burst.
 * The FIFO output registers are documented to roll over from 0x7E back to 0x78,
 * which would allow several words per transaction — but ST's own driver does not
 * rely on it, and a wrong assumption there does not fail loudly: it silently
 * decodes whatever follows 0x7E as sensor data.  The saving is about a quarter
 * of the transfer time and is not worth buying with that.
 */
static bool drainAccelFifo(IMUDevice &dev, IMUData &data, uint32_t now, bool &fresh){
    uint16_t pending = 0u;
    bool overrun = false;

    if (!readFifoStatus(dev, pending, overrun)){
        noteFault(dev.accelFaults, dev.accelIOErrors, dev.accelReady);
        return false;
    }

    // The part answered, so the transaction succeeded regardless of whether it
    // had anything buffered.  Consecutive-fault counters measure bus health, not
    // data availability, and conflating the two retires healthy sensors.
    dev.accelFaults = 0u;

    // Overrun means words were overwritten; backlog means the words still there
    // are older than the freshness contract.  Both are data gaps, and both are
    // handled the same way, because draining either one publishes samples that
    // are not from the moment they would be stamped with.  Treating only the
    // overrun case left the more common one — a loop stall of a few hundred
    // milliseconds — silently mislabelling seconds-old motion as current.
    if (overrun || (pending > IMU_FIFO_BACKLOG_WORDS)){
        // Counted apart, because they are different faults with different fixes:
        // an overrun says the drain fell far enough behind that the part
        // overwrote unread words, a freshness discard says the loop was blocked
        // long enough that the queue head aged out. One combined counter cannot
        // tell a technician which happened.
        if (overrun) { if (dev.fifoOverruns  < 0xFFFFu) dev.fifoOverruns++; }
        else         { if (dev.fifoGapFlushes < 0xFFFFu) dev.fifoGapFlushes++; }
        dev.lastOverrunMs  = now;
        dev.gapFlagActive  = true;
        dev.gapFlagUntilMs = now + IMU_PEAK_WINDOW_MS;

        if (!resetAccelFifo(dev)){
            noteFault(dev.accelFaults, dev.accelIOErrors, dev.accelReady);
            return false;
        }

        // A SUCCESSFUL recovery must not look like a dead sensor.  The stall
        // clocks still hold the timestamp of the last sample before the gap, so
        // leaving them alone means the caller's one-second stall test fires on
        // this very pass and retires a part that has just been put right — the
        // recovery would trigger the failure it exists to repair.  Same for the
        // peak ring: its buckets describe the window that was just discarded.
        dev.lastAccelReadyMs = now;
        dev.lastGyroReadyMs  = now;
        resetPeakRing(dev, now);

        // The samples in hand pre-date the gap, so they are not evidence about
        // now.  Blank them rather than carry them across the discontinuity.
        invalidateAccel(data);
        invalidateGyro(data);
        publishPeaks(dev, data, now);
        return true;   // tells the caller to skip the stall check this pass
    }

    const uint16_t toRead = (pending < IMU_FIFO_MAX_WORDS_PER_POLL)
                                ? pending : IMU_FIFO_MAX_WORDS_PER_POLL;

    for (uint16_t i = 0u; i < toRead; i++){
        uint8_t word[SOX_FIFO_WORD_BYTES];
        if (!readRegs(dev.accelAddress, SOX_REG_FIFO_DATA_OUT_TAG, word, sizeof(word))){
            noteFault(dev.accelFaults, dev.accelIOErrors, dev.accelReady);
            return false; // the FIFO is a queue; carrying on past a failed read
                          // would decode the rest against the wrong boundary
        }

        // TAG_SENSOR occupies bits 7:3; the low three bits are a 2-bit sample
        // counter and a parity bit, neither of which this code needs.
        switch (static_cast<uint8_t>(word[0] >> 3)){
            case SOX_TAG_ACCEL: {
                const float x = static_cast<float>(toInt16LE(&word[1])) * dev.accelScaleMs2;
                const float y = static_cast<float>(toInt16LE(&word[3])) * dev.accelScaleMs2;
                const float z = static_cast<float>(toInt16LE(&word[5])) * dev.accelScaleMs2;
                notePeakSq((x * x) + (y * y) + (z * z), dev.accelPeakSq, dev.peakBucketHead);
                data.accelX = x;
                data.accelY = y;
                data.accelZ = z;
                data.accelSampleMs  = now;
                data.accelValid     = true;
                dev.lastAccelReadyMs = now;
                fresh = true;
                break;
            }
            case SOX_TAG_GYRO: {
                const float x = static_cast<float>(toInt16LE(&word[1])) * dev.gyroScaleDps;
                const float y = static_cast<float>(toInt16LE(&word[3])) * dev.gyroScaleDps;
                const float z = static_cast<float>(toInt16LE(&word[5])) * dev.gyroScaleDps;
                notePeakSq((x * x) + (y * y) + (z * z), dev.gyroPeakSq, dev.peakBucketHead);
                data.gyroX = x;
                data.gyroY = y;
                data.gyroZ = z;
                data.gyroSampleMs   = now;
                data.gyroValid      = true;
                dev.lastGyroReadyMs = now;
                fresh = true;
                break;
            }
            case SOX_TAG_TEMP: {
                data.temperatureC = (static_cast<float>(toInt16LE(&word[1])) / SOX_TEMP_LSB_PER_C)
                                    + SOX_TEMP_OFFSET_C;
                data.tempSampleMs = now;
                data.tempValid    = true;
                fresh = true;
                break;
            }
            default:
                // Compression, sensor-hub and step-counter tags.  None of those
                // features is enabled, so a word carrying one means the device
                // is not configured the way this code believes — skip it rather
                // than decode six bytes of something else as acceleration.
                break;
        }
    }

    publishPeaks(dev, data, now);
    return false;
}

/**
 * @brief Low-power read: STATUS register, then one burst of the output registers.
 *
 * The pre-FIFO scheme, retained rather than deleted because it is the right
 * answer for a parked vehicle.  It sees only the samples a 20 Hz poll lands on —
 * against a 26 Hz ODR that is most of them, but nothing here is buffered, so
 * anything between two polls is gone.  That is an acceptable trade for a car
 * that is not moving and an unacceptable one for a car that is, which is the
 * whole reason the mode is switched rather than chosen once.
 *
 * Peaks are still maintained from every sample this DOES see.  They are what a
 * motion detector watches to decide the vehicle has started moving, so leaving
 * them stale in this mode would strand the system in low power.
 */
static void pollAccelRegisters(IMUDevice &dev, IMUData &data, uint32_t now, bool &fresh){
    uint8_t status = 0u;
    if (!readReg8(dev.accelAddress, SOX_REG_STATUS, status) ||
        ((status & SOX_STATUS_RESERVED_MASK) != 0u)){
        noteFault(dev.accelFaults, dev.accelIOErrors, dev.accelReady);
        return;
    }

    if ((status & (SOX_STATUS_XLDA | SOX_STATUS_GDA | SOX_STATUS_TDA)) == 0u){
        // A clean status read with nothing ready is a SUCCESSFUL transaction —
        // the part answered, it simply has no new sample yet.  The fault counter
        // tracks CONSECUTIVE failures, so it has to be cleared here too;
        // otherwise occasional glitches accumulate across thousands of healthy
        // polls and eventually retire a device that is working fine.
        dev.accelFaults = 0u;
        return;
    }

    // One burst covers temperature, gyro and accelerometer: they are contiguous,
    // and Block Data Update freezes the whole set until it has been read out.
    uint8_t buf[14];
    if (!readRegs(dev.accelAddress, SOX_REG_OUT_TEMP_L, buf, sizeof(buf))){
        noteFault(dev.accelFaults, dev.accelIOErrors, dev.accelReady);
        return;
    }
    dev.accelFaults = 0u;

    // Each channel is validated from ITS OWN ready bit.  The burst returns all
    // 14 bytes whichever bit triggered it, but a gyro that has stopped
    // converting still has its previous sample sitting in those registers —
    // certifying it because the accelerometer happened to be ready would
    // republish an old reading as a new one.
    if ((status & SOX_STATUS_TDA) != 0u){
        data.temperatureC = (static_cast<float>(toInt16LE(&buf[0])) / SOX_TEMP_LSB_PER_C)
                            + SOX_TEMP_OFFSET_C;
        data.tempSampleMs = now;
        data.tempValid = true;
        fresh = true;
    }

    if ((status & SOX_STATUS_GDA) != 0u){
        const float x = static_cast<float>(toInt16LE(&buf[2])) * dev.gyroScaleDps;
        const float y = static_cast<float>(toInt16LE(&buf[4])) * dev.gyroScaleDps;
        const float z = static_cast<float>(toInt16LE(&buf[6])) * dev.gyroScaleDps;
        notePeakSq((x * x) + (y * y) + (z * z), dev.gyroPeakSq, dev.peakBucketHead);
        data.gyroX = x;
        data.gyroY = y;
        data.gyroZ = z;
        data.gyroSampleMs = now;
        data.gyroValid = true;
        dev.lastGyroReadyMs = now;
        fresh = true;
    }

    if ((status & SOX_STATUS_XLDA) != 0u){
        const float x = static_cast<float>(toInt16LE(&buf[8]))  * dev.accelScaleMs2;
        const float y = static_cast<float>(toInt16LE(&buf[10])) * dev.accelScaleMs2;
        const float z = static_cast<float>(toInt16LE(&buf[12])) * dev.accelScaleMs2;
        notePeakSq((x * x) + (y * y) + (z * z), dev.accelPeakSq, dev.peakBucketHead);
        data.accelX = x;
        data.accelY = y;
        data.accelZ = z;
        data.accelSampleMs = now;
        data.accelValid = true;
        dev.lastAccelReadyMs = now;
        fresh = true;
    }

    publishPeaks(dev, data, now);
}

IMUReturnStatus getIMUData(IMUDevice &dev, IMUData &data){
    const uint32_t now = millis();
    bool fresh = false;

    // BEFORE i2cBusBegin(), and that ordering is the whole point.  A quarantined
    // boot promises to touch no I2C at all, and i2cBusBegin() is not a passive
    // question: on a stuck bus it performs GPIO-level recovery — nine clock
    // pulses, a STOP, and Wire.begin() — every I2C_BUS_RECOVER_RETRY_MS.  With
    // this call left below the bus check, a 20 Hz poll drove that recovery four
    // times a second for the entire quarantined boot.  It issued no addressed
    // transaction, so it did not reproduce the hang, but it plainly broke the
    // promise the quarantine makes.
    if (dev.quarantined){
        invalidateAccel(data);
        invalidateGyro(data);
        invalidateTemp(data);
        invalidateMagnetic(data);
        publishPresence(dev, data);
        return IMUReturnStatus::NOK_LINK_LOST;
    }

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
        // Dispatch on what the DEVICE was configured to do, not on what the
        // application would like it to be doing.  dev.mode is only ever written
        // by applySampleMode(), after the registers have read back — so the
        // decoder and the part can never disagree about whether a FIFO is
        // being filled.
        // Rolled HERE, once, for both paths.  It used to live inside the FIFO
        // drain only, which meant the low-power path wrote every sample into
        // whichever bucket was current at the last mode change and never
        // advanced it.  That bucket's timestamp then aged past the window and
        // ringMaxSq() excluded it — permanently.  So in low power the peaks read
        // correctly for 250 ms after entering the mode and were NAN from then
        // on, deterministically.  A soak that stayed in FIFO mode could not see
        // it, and the one that was run did.
        rollPeakRing(dev, now);

        bool gapRecovered = false;
        if (dev.mode == IMUSampleMode::Fifo) gapRecovered = drainAccelFifo(dev, data, now, fresh);
        else                                 pollAccelRegisters(dev, data, now, fresh);

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
        //
        // Skipped entirely on the pass that recovered a data gap.  The stall
        // that caused the gap is exactly what this test measures, so running it
        // here would retire the part for the fault the recovery has already
        // repaired — punishing success.  The clocks were refreshed inside the
        // recovery, so the next pass judges the part on what it does AFTER the
        // gap, which is the only fair question.
        if (!gapRecovered && dev.accelReady &&
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
    // A quarantined device is NOT reported as degraded, deliberately.  Degraded
    // is what schedules recoverIMU(), and recovery transacts on a bus that has
    // just hung the board — so answering true here would defeat the quarantine
    // a few seconds after it was applied.
    if (dev.quarantined) return false;
    return !dev.accelReady || !dev.magReady;
}

bool isIMUQuarantined(const IMUDevice &dev){
    return dev.quarantined;
}
