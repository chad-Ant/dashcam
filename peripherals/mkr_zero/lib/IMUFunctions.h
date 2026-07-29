#ifndef IMU_FUNCTIONS
#define IMU_FUNCTIONS 1

#include <Arduino.h>

#include "I2CBus.h"

/**
 * @file IMUFunctions.h
 * @brief Adafruit LSM6DSOX + LIS3MDL 9-DoF breakout (PID 4517) on the MKR Zero.
 *
 * The breakout carries two independent I2C devices on ONE bus segment:
 *   - LSM6DSOX  @c 0x6A  accelerometer + gyroscope + die temperature
 *   - LIS3MDL   @c 0x1C  three-axis magnetometer
 *
 * Both share @c Wire (D11/SDA, D12/SCL) with the u-blox GNSS receiver at
 * @c 0x42 and, when fitted, the segment LED at @c 0x70.  No address collides —
 * see @c checkI2CBusConflict(), which verifies that at RUN time rather than
 * trusting the datasheet defaults.
 *
 * WHY THIS IS A REGISTER-LEVEL DRIVER AND NOT A WRAPPER
 * ----------------------------------------------------
 * The obvious implementation delegates to Adafruit_LSM6DSOX / Adafruit_LIS3MDL.
 * Three properties this project needs rule that out:
 *
 *   1. HONEST FAILURE.  @c Adafruit_LSM6DS::getEvent() always returns @c true,
 *      and the @c _read() behind it ignores the return of its 14-byte I2C burst
 *      into an UNINITIALISED stack buffer.  A sensor that has lost power does
 *      not report an error — it publishes accelerations decoded from stack
 *      garbage, which silently poisons incident detection.
 *   2. NO ALLOCATION AFTER INIT.  @c begin_I2C() runs @c delete / @c new, and so
 *      does the @c _init() behind it (three unified-sensor helpers).  A retry
 *      path that re-runs those churns a 32 KB heap from @c loop().
 *   3. BOUNDED WAITS.  @c Adafruit_LSM6DS::reset() spins on
 *      @c while (sw_reset.read()) with no timeout, so a sensor that fails
 *      mid-reset hangs the node forever.
 *
 * This library therefore talks to both parts over @c Wire directly: every
 * transaction's return is checked, nothing is allocated at any point, and every
 * wait has a deadline.  An absent sensor yields @c NAN and a cleared valid flag.
 *
 * AXES
 * ----
 * Readings are in the SENSOR frame as silkscreened on the breakout.  Mapping
 * to the vehicle frame (which depends on how the board is bolted in) is left
 * to the caller — this library does not guess a mounting orientation.
 *
 * HARDWARE
 * --------
 * Power the breakout from the MKR Zero's @b VCC (3.3 V) pin, never 5 V.  The
 * STEMMA QT level shifters reference the board's input rail, so a 5 V supply
 * puts 5 V on SDA/SCL — and no MKR Zero I/O pin is 5 V tolerant.
 *
 * BUS OWNERSHIP
 * -------------
 * This library does not open, configure or recover the I2C bus itself — that
 * belongs to @c I2CBus.h, which every client on the bus enters through so the
 * first one to arrive is the one that unwedges it.  See that header for why a
 * shared "already initialised" boolean cannot express that ordering.
 *
 * KNOWN PLATFORM HAZARD
 * ---------------------
 * The Arduino SAMD core's I2C driver waits on its bus flags in UNBOUNDED loops
 * (@c SERCOM::startTransmissionWIRE, @c SERCOM::readDataWIRE).  Nothing in
 * userland can bound them — the vendored Wire patch fixes the one-byte
 * @c requestFrom() defect only.  @c i2cBusBegin() clears the common power-on
 * case, but a wedge that happens mid-drive still needs a watchdog to turn a
 * hang into a reboot.
 */

/** Return codes used by IMU functions. */
enum class IMUReturnStatus{
    OK = 0,                     ///< Requested operation succeeded.
    DATA_STALE = 1,             ///< No new sample since the last call; cached values retained.
    PARTIAL = 2,                ///< Only one of the two devices is live.
    NOK_INIT_FAILED = -1,       ///< Neither device answered.
    NOK_ACCEL_MISSING = -2,     ///< LSM6DSOX absent or wrong WHO_AM_I.
    NOK_MAG_MISSING = -3,       ///< LIS3MDL absent or wrong WHO_AM_I.
    NOK_ADDRESS_CONFLICT = -4,  ///< A foreign device occupies an IMU address.
    NOK_BUS_STUCK = -5,         ///< SDA held low; recovery clocking did not free it.
    NOK_LINK_LOST = -6,         ///< Both devices stopped answering after a good init.
    NOK_CONFIG_FAILED = -7,     ///< Device answered but rejected its configuration.
};

/**
 * @brief One coherent 9-DoF snapshot.
 *
 * Every reading is @c NAN unless its matching valid flag is true, so a consumer
 * can never mistake "sensor gone" for "sitting still at 0 m/s2" — zero is a
 * perfectly plausible gyro reading, which is exactly why it must not double as
 * the missing-data sentinel.
 *
 * Accelerometer and gyroscope carry SEPARATE timestamps and validity even
 * though they live in one chip: the LSM6DSOX has an independent data-ready flag
 * per channel, and a gyro that has stopped converting while the accelerometer
 * keeps going must be reported as exactly that.
 */
struct IMUData{
    float accelX;         ///< Acceleration along sensor X (m/s2, gravity included).
    float accelY;         ///< Acceleration along sensor Y (m/s2, gravity included).
    float accelZ;         ///< Acceleration along sensor Z (m/s2, gravity included).

    float gyroX;          ///< Angular rate about sensor X (deg/s).
    float gyroY;          ///< Angular rate about sensor Y (deg/s).
    float gyroZ;          ///< Angular rate about sensor Z (deg/s).

    float magX;           ///< Magnetic flux density along sensor X (uT).
    float magY;           ///< Magnetic flux density along sensor Y (uT).
    float magZ;           ///< Magnetic flux density along sensor Z (uT).

    float temperatureC;   ///< LSM6DSOX die temperature (degC) — board, not cabin.

    uint32_t accelSampleMs; ///< @c millis() of the newest accepted accelerometer sample.
    uint32_t gyroSampleMs;  ///< @c millis() of the newest accepted gyroscope sample.
    uint32_t tempSampleMs;  ///< @c millis() of the newest accepted temperature sample.
    uint32_t magSampleMs;   ///< @c millis() of the newest accepted magnetometer sample.

    bool accelValid;      ///< Accelerometer readings are fresh and trustworthy.
    bool gyroValid;       ///< Gyroscope readings are fresh and trustworthy.
    bool magValid;        ///< Magnetometer readings are fresh and trustworthy.
    bool tempValid;       ///< Die temperature is fresh and trustworthy.

    /**
     * At least one device is configured and has not been declared lost.
     *
     * Deliberately NOT derived from the valid flags above.  Those describe this
     * SAMPLE; this describes the HARDWARE, and the two differ in exactly the
     * case that matters: an IMU that is fitted and simply quiet right now has
     * every valid flag clear but is emphatically still present.  Collapsing the
     * two would make "sensor absent" and "sensor briefly silent" indis-
     * tinguishable to a consumer, which is the misdiagnosis this whole struct
     * is arranged to prevent.
     */
    bool devicePresent;

    /**
     * BOTH devices are answering.
     *
     * Kept separate from @c devicePresent because the difference is a fault
     * signature, not a detail: the LSM6DSOX and LIS3MDL share one PCB, one VIN
     * and one ground, so exactly one responding cannot happen benignly. An
     * unpowered I2C slave keeps acknowledging through the bus pull-ups, so the
     * lighter-draw part survives a power loss the other does not — which is what
     * pulling VIN on the bench produced.
     */
    bool allDevicesPresent;
};

/**
 * @brief Device addresses plus the health state needed to fail honestly.
 *
 * Declare ONE of these at file scope.  It is a plain aggregate holding no
 * pointers and owning no memory, so neither @c initializeIMU() nor
 * @c recoverIMU() allocates — they may be called from @c loop() on a retry
 * timer without touching the heap.
 */
struct IMUDevice{
    uint8_t accelAddress;     ///< 7-bit address the LSM6DSOX answers on.
    uint8_t magAddress;       ///< 7-bit address the LIS3MDL answers on.

    bool accelReady;          ///< LSM6DSOX configured and not yet declared lost.
    bool magReady;            ///< LIS3MDL configured and not yet declared lost.

    uint8_t accelFaults;      ///< Consecutive failed/implausible LSM6DSOX bus reads.
    uint8_t magFaults;        ///< Consecutive failed/implausible LIS3MDL bus reads.

    /// Cumulative failed bus transactions since @c initializeIMU(), saturating.
    ///
    /// The consecutive counters above cannot answer "did any read fail?", and
    /// that is by design — they reset on every success, because a lone NACK is a
    /// glitch to ride out rather than a fault to act on.  The consequence is that
    /// transient failures are invisible from outside: @c getIMUData() still
    /// returns @c OK when the other device supplied fresh data, so a caller
    /// counting non-OK returns measures nothing and can report a clean run over a
    /// bus that was NACKing steadily.
    ///
    /// These are the diagnostic the consecutive counters deliberately are not:
    /// monotonic, independent of whether a sample happened to be available, and
    /// carried across @c recoverIMU() so a device that keeps dropping out is
    /// distinguishable from one that failed once.
    uint16_t accelIOErrors;
    uint16_t magIOErrors;

    /// @c millis() when each channel last asserted its data-ready bit.  These
    /// detect a channel that has gone silent while its device still answers the
    /// bus — a fault no data-age check can act on, because the device never
    /// looks lost and so is never retried.  See @c IMU_MAX_CHANNEL_STALL_MS.
    uint32_t lastAccelReadyMs;
    uint32_t lastGyroReadyMs;
    uint32_t lastMagReadyMs;

    float accelScaleMs2;      ///< m/s2 per LSB for the configured accelerometer range.
    float gyroScaleDps;       ///< deg/s per LSB for the configured gyroscope range.
    float magScaleUt;         ///< uT per LSB for the configured magnetometer range.
};

/** What answered on the bus, and whether it is what we expected. */
struct I2CBusReport{
    uint8_t deviceCount;                        ///< Number of responders found.
    uint8_t addresses[I2C_SCAN_MAX_DEVICES];    ///< 7-bit addresses that ACKed.
    bool accelPresent;    ///< Something ACKed at @c IMU_ACCEL_I2C_ADDRESS.
    bool magPresent;      ///< Something ACKed at @c IMU_MAG_I2C_ADDRESS.
    bool gpsPresent;      ///< Something ACKed at @c GPS_DEFAULT_I2C_ADDRESS.
    bool accelIdentified; ///< That responder returned the LSM6DSOX WHO_AM_I.
    bool magIdentified;   ///< That responder returned the LIS3MDL WHO_AM_I.
    bool conflict;        ///< An IMU address is occupied by a device that is not the IMU.
};

/**
 * @brief Brings up both sensors at their default addresses.
 *
 * Enters through @c i2cBusBegin(), which opens the bus once per session and
 * recovers it FIRST, before any transaction: a slave wedged onto SDA would
 * otherwise hang the very first probe inside the SAMD core's unbounded wait,
 * and recovery placed after that probe could never run.
 *
 * Resets, configures and verifies each device independently, so one missing
 * sensor never prevents the other from coming up.
 *
 * @param[in,out] dev  Device bundle to initialise.
 * @return @c OK when both devices came up, @c PARTIAL when exactly one did,
 *         @c NOK_INIT_FAILED when neither answered, @c NOK_ADDRESS_CONFLICT
 *         when an address is held by a foreign device, or @c NOK_BUS_STUCK
 *         when SDA could not be freed.
 */
IMUReturnStatus initializeIMU(IMUDevice &dev);

/**
 * @brief Brings up both sensors at caller-chosen addresses.
 *
 * Use when the breakout's address jumpers are closed (LSM6DSOX 0x6B,
 * LIS3MDL 0x1E) or when two breakouts share the bus.
 *
 * @param[in,out] dev           Device bundle to initialise.
 * @param[in]     accelAddress  7-bit LSM6DSOX address, 0x6A or 0x6B.
 * @param[in]     magAddress    7-bit LIS3MDL address, 0x1C or 0x1E.
 * @return As @c initializeIMU(IMUDevice&).  Returns @c NOK_ADDRESS_CONFLICT if
 *         either address is one this project has already allocated to another
 *         device (the GNSS receiver or the segment LED).
 */
IMUReturnStatus initializeIMU(IMUDevice &dev, uint8_t accelAddress, uint8_t magAddress);

/**
 * @brief Re-configures ONLY the devices currently marked not-ready.
 *
 * This is the retry path, and it is deliberately not @c initializeIMU(): a
 * magnetometer that dropped off the bus must not cost a reset of a perfectly
 * healthy accelerometer, which would blank a working signal for the ~30 ms the
 * part takes to reset and refill its filters.
 *
 * Allocation-free and safe to call from @c loop() on a bounded retry timer.
 * Calling it when nothing is down is a no-op that returns @c OK.
 *
 * @param[in,out] dev  Device bundle previously passed to @c initializeIMU().
 * @return As @c initializeIMU(IMUDevice&).
 */
IMUReturnStatus recoverIMU(IMUDevice &dev);

/** @brief Resets a snapshot to the "nothing received" state (all @c NAN). */
void initIMUData(IMUData &data);

/**
 * @brief Reads whatever new data the sensors have (non-blocking).
 *
 * Checks each device's STATUS register first and only bursts the output
 * registers when a fresh conversion is waiting, so polling faster than the
 * output data rate costs one short transaction instead of a wasted 14-byte
 * read.  Block Data Update is enabled on both parts, so a burst can never
 * splice the MSB of one sample onto the LSB of the next.
 *
 * Accelerometer, gyroscope and temperature are validated independently from
 * their own data-ready bits — a burst triggered by the accelerometer does not
 * certify the gyro half of the same 14 bytes.
 *
 * Readings older than @c IMU_MAX_DATA_AGE_MS are replaced with @c NAN and
 * their valid flag cleared: a value that stopped updating is not a measurement,
 * however plausible it still looks.
 *
 * Refuses to transact at all when the shared bus is not idle, returning
 * @c NOK_BUS_STUCK without charging the failure to either device.  That check
 * belongs on every poll, not only on bring-up: the SAMD core's I2C waits have no
 * deadline, so one slave holding SDA mid-drive would otherwise hang the CPU until
 * the watchdog reset it.
 *
 * A caller that needs to know whether reads have been FAILING, as opposed to
 * whether a sample was available, must read @c IMUDevice::accelIOErrors and
 * @c magIOErrors — this return code cannot express it, and returns @c OK for a
 * poll in which one device NACKed and the other supplied fresh data.
 *
 * @param[in,out] dev   Initialised device bundle.
 * @param[in,out] data  Snapshot to update in place.
 * @return @c OK when at least one device supplied a fresh sample,
 *         @c DATA_STALE when neither had new data but the cached values are
 *         still inside the freshness window, @c PARTIAL when one device has
 *         been declared lost, @c NOK_LINK_LOST when both have, or
 *         @c NOK_BUS_STUCK when the bus was not safe to use.
 */
IMUReturnStatus getIMUData(IMUDevice &dev, IMUData &data);

/**
 * @brief True once every configured device has been declared lost.
 *
 * Mirrors @c isOBD2LinkLost().  Note this is the BOTH-gone case — schedule
 * recovery on @c isIMUDegraded() instead, or a single failed sensor never gets
 * retried.
 */
bool isIMULinkLost(const IMUDevice &dev);

/** @brief True when any device that should be running has gone silent. */
bool isIMUDegraded(const IMUDevice &dev);

/**
 * @brief Checks that the IMU addresses hold the IMU and nothing else.
 *
 * Answers the question a datasheet cannot: an address being free "by default"
 * is worthless if the GNSS receiver on the same bus was reconfigured, or a
 * second breakout was added with its jumper closed.  Scans the bus, then reads
 * WHO_AM_I at each IMU address and compares it against the expected part ID.
 *
 * Call it BEFORE @c initializeIMU() during bring-up.  Does not require the
 * sensors to be initialised; it opens the bus itself via @c i2cBusBegin().
 *
 * @param[out] report  Filled with what was found.
 * @return @c OK when both IMU addresses hold the expected parts,
 *         @c NOK_ADDRESS_CONFLICT when one holds something else,
 *         @c NOK_ACCEL_MISSING / @c NOK_MAG_MISSING when an address is silent,
 *         @c NOK_INIT_FAILED when the bus is genuinely empty, or
 *         @c NOK_BUS_STUCK when a held line made the scan impossible — which is
 *         a different fault from an empty bus and must not be reported as one.
 */
IMUReturnStatus checkI2CBusConflict(I2CBusReport &report);

#endif
