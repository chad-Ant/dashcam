#ifndef IMU_FUNCTIONS
#define IMU_FUNCTIONS 1

#include <Arduino.h>

#include "BNO055Init.h"
#include "I2CBus.h"

/**
 * @file IMUFunctions.h
 * @brief Bosch BNO055 9-DoF with on-chip sensor fusion, on the MKR Zero.
 *
 * ONE device at @c 0x29 (or @c 0x28), sharing @c Wire (D11/SDA, D12/SCL) with
 * the u-blox GNSS receiver at @c 0x42 and, when fitted, the segment LED at
 * @c 0x70.
 *
 * WHAT REPLACED WHAT, AND WHY
 * ---------------------------
 * This replaces a register-level driver for an Adafruit LSM6DSOX + LIS3MDL
 * breakout.  That part was retired after it spent a recorded 40 % of its samples
 * returning corrupt-but-plausible data — a fixed 27.8 m/s2 and 104 C on a 25 C
 * bench — from I2C transactions that SUCCEEDED, and then latched up with SDA
 * clamped at 0.6 V on a good rail.  For a dashcam whose acceleration peak
 * triggers incident capture, a steady fabricated 2.85 g is worse than a dead
 * sensor, and that shaped most of what follows.
 *
 * THREE THINGS ARE ARCHITECTURALLY DIFFERENT, not just renamed:
 *
 *   1. THERE IS NO FIFO.  Zero occurrences of the word in the datasheet, and
 *      zero in the Bosch driver.  The old peak capture leaned on a 512-word FIFO
 *      that batched 220 samples/s for a 20 Hz drain to collect in full, so every
 *      sample reached the peak ring.  Here the poll rate IS the sample rate: a
 *      20 Hz poll sees 20 of the 100 samples the part produced.  Phase 4 raises
 *      the poll to 100 Hz and adds the High-G interrupt, which latches an impact
 *      in hardware and so survives a loop stall the FIFO could only have covered
 *      for as long as its depth allowed.
 *
 *   2. THERE ARE NO PER-CHANNEL DATA-READY BITS.  The LSM6DSOX published one per
 *      channel, and this driver's predecessor validated each channel from its
 *      own bit — so a gyro that stopped converting could be caught while the
 *      accelerometer beside it kept working.  The BNO055 offers nothing
 *      equivalent, so @c fresh here means "a plausible burst was read", not "the
 *      part produced a new sample".  What replaces the stall check is
 *      @c IMUDevice::accelChangeMs and @c IMUDevice::gyroChangeMs: real inertial
 *      data is never bit-identical twice in a row, because gyro noise alone
 *      guarantees the low bits move.  Frozen bytes are therefore a positive
 *      signal, and one that would have caught the previous sensor's failure
 *      directly — but ONLY because the two channels are timed separately.  With
 *      one timer over both, the gyro noise that makes the check work at all is
 *      also what hides a frozen accelerometer beside it.
 *
 *   3. FUSION IS ON-CHIP.  @c linAccelX/Y/Z is gravity-compensated acceleration
 *      computed by the part, which is the signal incident detection actually
 *      wants; @c gravityX/Y/Z is its estimate of the gravity vector, and its
 *      MAGNITUDE is a free, physics-backed integrity check — it must be about
 *      9.81 whatever the vehicle is doing, so a departure means the output is
 *      not to be trusted.  Nothing the old part offered could do that.
 *
 * BUS OWNERSHIP is unchanged: this library never opens or recovers the bus
 * itself.  That belongs to @c I2CBus.h, which every client enters through so the
 * first to arrive is the one that unwedges it.
 *
 * AXES are in the SENSOR frame as silkscreened.  Mapping to the vehicle frame
 * depends on how the board is bolted in, and this library does not guess a
 * mounting orientation — the same rule the previous driver followed.
 */

/**
 * What the part is configured to produce.  Chosen per session, NOT per event.
 *
 * Switching costs a full re-configuration — the datasheet's 19 ms out of an
 * operating mode plus 7 ms back in, and in practice a reset — so it cannot be
 * done on a trigger: a crash pulse is over in 10-50 ms.  It is also not a
 * volume knob. The two modes produce DIFFERENT MEASUREMENTS, and the difference
 * is recorded on every sample rather than left for a consumer to infer.
 */
enum class IMUSampleMode : uint8_t{
    /**
     * IMUPLUS: accelerometer + gyroscope fused on-chip, magnetometer OFF.
     *
     * The primary mode.  Yields linear acceleration, gravity and relative yaw,
     * none of which the raw mode can produce.
     *
     * The accelerometer is LOCKED AT +/-4 g in every fusion mode (datasheet
     * 3.5), so acceleration saturates at 39.2 m/s2 and a real collision pulse of
     * 15-40 g clips hard.  @c IMUData::accelSaturated says when that has
     * happened, because a clipped 4 g reported as a plain number is the same
     * class of lie as the corrupt 2.85 g that retired the last sensor.
     *
     * The magnetometer stays off deliberately.  It is not additive: it feeds the
     * orientation quaternion, and linear acceleration is derived FROM that
     * quaternion, so magnetic disturbance would contaminate the one channel
     * incident detection depends on.  A car is close to a worst case — steel
     * shell, dash speakers, a harness carrying tens of amps — and the distortion
     * moves with electrical load, so calibration cannot converge.  GNSS course
     * and the CAN rear-wheel differential already provide heading, drift-free.
     */
    Fusion = 0,
    /**
     * AMG: raw accelerometer, magnetometer and gyroscope. No fusion.
     *
     * Not a mere fallback — it is the only mode that can select an accelerometer
     * range, up to +/-16 g, so it is the only one that can CHARACTERISE a severe
     * impact rather than clip it.  In exchange there is no linear acceleration,
     * no gravity vector and no orientation, so @c accelPeakMs2 includes gravity
     * and the fused fields are @c NAN.
     */
    Raw = 1,
};

/// @c IMUDevice::lifecycle sentinels.  @c UNINIT is the member initialiser, so
/// the field is never read before it is written — see @c IMUDevice::lifecycle.
#define IMU_LIFECYCLE_UNINIT      0u
#define IMU_LIFECYCLE_ACTIVE      0x9A1C0DE1u
#define IMU_LIFECYCLE_QUARANTINED 0x9A1CDEADu

/** Return codes used by IMU functions. */
enum class IMUReturnStatus{
    OK = 0,                     ///< A fresh, plausible sample was read.
    DATA_STALE = 1,             ///< No new sample; cached values still inside the window.
    /**
     * The part is running, and its FUSED output has not passed the local gate.
     *
     * Raised while the fusion has produced no estimate yet, or while GYROSCOPE
     * calibration is below usable.  The accelerometer figure is deliberately NOT
     * a criterion — see @c IMU_CALIB_MIN_GYRO — so this is a MINIMUM LOCAL
     * USABILITY GATE rather than a verdict on quality, and a consumer wanting an
     * accelerometer-calibration policy must apply its own: the figure is on the
     * wire in @c imuCalib for exactly that purpose.
     *
     * Kept distinct from @c OK because IMUPLUS publishes linear acceleration
     * from the moment it starts, and those first values are not wrong-looking —
     * they are simply not yet right, and nothing downstream could otherwise
     * tell.
     */
    PARTIAL = 2,
    NOK_INIT_FAILED = -1,       ///< Bring-up failed; see @c IMUDevice::init.
    NOK_NOT_READY = -2,         ///< Bring-up still in progress. Not an error.
    NOK_ADDRESS_CONFLICT = -4,  ///< A foreign device occupies the IMU address.
    NOK_BUS_STUCK = -5,         ///< SDA held low; recovery clocking did not free it.
    NOK_LINK_LOST = -6,         ///< The part stopped answering after a good init.
    NOK_CONFIG_FAILED = -7,     ///< Device answered but rejected its configuration.
};

/**
 * @brief One coherent snapshot.
 *
 * Every reading is @c NAN unless its matching valid flag is true, so a consumer
 * can never mistake "sensor gone" for "sitting still at 0 m/s2" — zero is a
 * perfectly plausible reading, which is exactly why it must not double as the
 * missing-data sentinel.
 */
struct IMUData{
    // ── raw channels, present in both modes ──────────────────────────────────
    float accelX;         ///< Acceleration along sensor X (m/s2, gravity included).
    float accelY;         ///< Acceleration along sensor Y (m/s2, gravity included).
    float accelZ;         ///< Acceleration along sensor Z (m/s2, gravity included).

    float gyroX;          ///< Angular rate about sensor X (deg/s).
    float gyroY;          ///< Angular rate about sensor Y (deg/s).
    float gyroZ;          ///< Angular rate about sensor Z (deg/s).

    float magX;           ///< Magnetic flux density along sensor X (uT). @c Raw mode only.
    float magY;           ///< Magnetic flux density along sensor Y (uT). @c Raw mode only.
    float magZ;           ///< Magnetic flux density along sensor Z (uT). @c Raw mode only.

    float temperatureC;   ///< Die temperature (degC) — board, not cabin.

    // ── fused channels, @c Fusion mode only ──────────────────────────────────
    /**
     * Gravity-compensated acceleration (m/s2).
     *
     * What incident detection actually wants, and what the previous hardware
     * could not supply: a 1 g reading here means the vehicle accelerated at 1 g,
     * not that it is sitting still on a planet.
     */
    float linAccelX;
    float linAccelY;
    float linAccelZ;

    /**
     * The part's estimate of the gravity vector (m/s2).
     *
     * Published because its MAGNITUDE is an integrity check available no other
     * way.  It must be about 9.81 whatever the vehicle is doing — the fusion
     * constructs it that way — so a departure means the output is not to be
     * trusted, whatever the transaction status said.  See
     * @c IMU_GRAVITY_MIN_VALID_MS2.
     */
    float gravityX;
    float gravityY;
    float gravityZ;

    /**
     * Rotation about the vertical, in degrees, RELATIVE and drifting.
     *
     * Named for what it is.  In @c Fusion the magnetometer is off, so this has
     * no north reference and no absolute meaning; calling it "heading" invites
     * precisely the misreading that would put a drifting number where a bearing
     * belongs.  GNSS course-over-ground and the CAN rear-wheel differential are
     * the heading sources.
     *
     * What it IS good for is short-window rotation at the fusion rate: how far
     * the car turned during a two-second event, where drift is negligible and
     * neither the 1 Hz GNSS nor the wheel pair — dead below about 3 km/h — can
     * answer.
     */
    float yawRelDeg;
    float pitchDeg;       ///< Pitch (deg). Sign convention follows @c IMUDevice::eulerAndroid.
    float rollDeg;        ///< Roll (deg).

    // ── windowed peaks ───────────────────────────────────────────────────────
    /**
     * Largest magnitudes seen over the trailing @c IMU_PEAK_WINDOW_MS.
     *
     * These exist because the axes above are the MOST RECENT sample, and at a
     * 10 Hz publication rate that sample is one of the ten the part produced
     * since the last frame — a pothole hit by the other nine is simply not in
     * the data.  Peaks are folded from every polled sample, so a transient is
     * reported even though the vector that caused it is not.
     *
     * Magnitudes, so they do not depend on how the board is bolted in.
     */
    float accelPeakMs2;   ///< Peak |a| (m/s2, gravity included).
    float linAccelPeakMs2;///< Peak |linear a| (m/s2). @c Fusion mode only.
    float gyroPeakDps;    ///< Peak |w| (deg/s).

    /**
     * The accelerometer hit its range limit during this window.
     *
     * @c accelPeakMs2 is then a FLOOR, not a measurement.  Fusion modes lock the
     * range at +/-4 g, so anything past 39.2 m/s2 is clipped and a genuine
     * 20 g collision reads as 4 g.  A saturated peak published as a plain number
     * would understate an impact by a factor of five.
     *
     * @c linAccelPeakMs2 IS EQUALLY SUSPECT while this is set, and there is no
     * separate flag for it: linear acceleration is derived from the same clipped
     * accelerometer, so a rail the raw channel hit propagates straight into it.
     * Bench measurement during clipping showed the linear peak reaching
     * 56.97 m/s2 — above the raw rail itself, because subtracting an estimated
     * gravity vector from a clipped measurement is not a physical quantity.
     * Treat both peaks as lower bounds whenever this is true.
     */
    bool accelSaturated;

    /**
     * The part latched a High-G threshold crossing.
     *
     * THIS IS THE ONE FIELD THAT SURVIVES A STALLED LOOP. Everything else here
     * is the product of a poll, and a poll that does not happen produces
     * nothing — there is no FIFO to cover the gap. The BNO055 latches a
     * threshold crossing in hardware and holds it until cleared, so an impact
     * during a half-second stall is still reported afterwards, where the peak
     * that would have described it is simply gone.
     *
     * Held for @c IMU_HIGHG_HOLD_MS so it cannot fall between two published
     * frames.
     */
    bool highGEvent;

    /// @c millis() the High-G latch was first seen. Meaningless unless
    /// @c highGEvent. Read from the INT pin when one is wired and from the poll
    /// that noticed the latch otherwise — the second is later, but it is the
    /// event's own timestamp either way, not the frame's.
    uint32_t highGMs;

    /// The hardware backstop is armed. When false, an impact inside a loop
    /// stall goes unrecorded, and nothing else on the frame would say so.
    bool highGArmed;

    /**
     * There is a HOLE in the inertial record for this window.
     *
     * Named for the consequence rather than the cause: a peak computed across a
     * gap is not a peak over the window it claims, and an incident detector has
     * to know that.  Raised when polls were missed — with no FIFO, a poll that
     * does not happen is data that does not exist, which is why this flag is
     * far more load-bearing here than it was on the previous part.
     */
    bool dataGap;

    /// Sampling regime was coarser than normal.  Reserved for the parked
    /// low-power mode, which is not enabled yet: the datasheet withdraws the
    /// High-G interrupt in low power, leaving no hardware impact backstop
    /// exactly when a car-park bump is the thing being watched for.
    bool lowPower;

    /// True when the part is in @c IMUSampleMode::Fusion.  Carried on the sample
    /// because the peak means something different in each mode — one includes
    /// gravity and saturates at 4 g, the other does not.
    bool fusionMode;

    // ── calibration, 0..3 each ───────────────────────────────────────────────
    /// System calibration.  STAYS 0 in @c Fusion mode and that is correct, not a
    /// fault: the system figure cannot rise without the magnetometer, which is
    /// deliberately off.  Judge fusion readiness by @c calibGyro and
    /// @c calibAccel.
    uint8_t calibSys;
    uint8_t calibGyro;    ///< Reaches 3 within seconds of sitting still.
    uint8_t calibAccel;   ///< Needs the board held in several orientations.
    uint8_t calibMag;     ///< Always 0 in @c Fusion mode; the magnetometer is off.

    // ── freshness ────────────────────────────────────────────────────────────
    uint32_t accelSampleMs; ///< @c millis() of the newest accepted sample.
    uint32_t gyroSampleMs;
    uint32_t tempSampleMs;
    uint32_t magSampleMs;
    uint32_t fusionSampleMs;

    bool accelValid;      ///< Accelerometer readings are fresh and trustworthy.
    bool gyroValid;       ///< Gyroscope readings are fresh and trustworthy.
    bool magValid;        ///< Magnetometer readings are fresh and trustworthy.
    bool tempValid;       ///< Die temperature is fresh and trustworthy.
    bool fusionValid;     ///< Linear accel, gravity and Euler are fresh and trustworthy.

    /**
     * The part is configured and has not been declared lost.
     *
     * Deliberately NOT derived from the valid flags.  Those describe this
     * SAMPLE; this describes the HARDWARE, and the two differ in exactly the
     * case that matters — an IMU that is fitted and briefly quiet has every
     * valid flag clear and is emphatically still present.
     */
    bool devicePresent;

    /**
     * The part is running EXACTLY as configured.
     *
     * Retained under its old name because the telemetry flag built on it has not
     * been re-cut yet; the protocol bump owns that.  Its meaning has necessarily
     * changed: with one chip instead of two there is no "both answered" to
     * report, so it now means present AND in the requested mode AND on the
     * requested clock.  False therefore means running in a fallback — still
     * useful data, not the data that was asked for.
     */
    bool allDevicesPresent;
};

/**
 * @brief The device: bring-up state, health counters and the peak ring.
 *
 * Plain aggregate holding no pointers of its own and owning no memory, so
 * neither @c initializeIMU() nor @c recoverIMU() allocates.
 *
 * MUST have static storage duration — @c BNO055InitState::dev is handed to the
 * Bosch driver, which keeps a file-static pointer to it.
 */
struct IMUDevice{
    BNO055InitState init;     ///< Address, mode, clock and driver context.

    bool     ready;           ///< Configured and not yet declared lost.
    uint8_t  faults;          ///< Consecutive failed or implausible reads.

    /// Cumulative failed transactions since @c initializeIMU(), saturating.
    ///
    /// The consecutive counter above cannot answer "did any read fail?", by
    /// design — it resets on every success, because a lone NACK is a glitch to
    /// ride out.  The consequence is that transients are invisible from outside,
    /// so this is the diagnostic that counter deliberately is not: monotonic,
    /// and carried across @c recoverIMU() so a part that keeps dropping out is
    /// distinguishable from one that failed once.
    uint16_t ioErrors;

    /// Reads discarded for failing a plausibility gate rather than the bus.
    /// Counted separately from @c ioErrors because they mean something entirely
    /// different: the transaction SUCCEEDED and the data was wrong, which is the
    /// failure that retired the previous sensor and the one no transport-level
    /// counter can see.
    uint16_t implausible;

    /// @c millis() at which each channel's raw bytes last CHANGED.
    ///
    /// This replaces the per-channel data-ready stall check, which the BNO055
    /// gives no way to perform.  Real inertial data is never bit-identical twice
    /// running — gyro noise alone moves the low bits — so bytes that stop
    /// changing are a frozen data path, on a part that is still answering every
    /// transaction perfectly.  See @c IMU_MAX_CHANNEL_STALL_MS.
    ///
    /// TWO TIMERS, NOT ONE, and the split is the whole point of the check.  A
    /// single flag over all twelve bytes is satisfied by ANY of them moving, so
    /// ordinary gyro noise — which never stops — held the timer open forever and
    /// a completely frozen accelerometer could never trip it.  That is not a
    /// hypothetical: the sensor this driver replaced failed by returning a fixed
    /// 27.8 m/s2 accelerometer, and the combined check this file documented as
    /// catching that failure "directly" would have masked it.
    uint32_t accelChangeMs;
    uint32_t gyroChangeMs;
    uint8_t  lastRaw[12];     ///< Accel (0..5) + gyro (6..11) bytes from the previous poll.
    bool     lastRawValid;

    /// @c millis() of the last burst that passed EVERY plausibility gate.
    ///
    /// The gap flag is measured from this rather than from the last poll
    /// ATTEMPT, because the two differ in the case that was being missed: a poll
    /// that runs on time and has its burst rejected by the temperature, gravity
    /// or accel-limit gate leaves exactly the same hole in the record as a poll
    /// that never happened.  Keying on the attempt reset the timer every 10 ms
    /// while every sample was being discarded, so a 200 ms hole reported none.
    uint32_t lastGoodMs;


    /// Windowed peak tracking, held as SQUARED magnitudes in a bucket ring.
    ///
    /// Squares keep the hot path free of square roots: one sqrtf per channel per
    /// poll replaces one per sample.  On a Cortex-M0+ with no FPU that is not a
    /// micro-optimisation — every sqrtf is a software routine.
    ///
    /// The RING is what makes the window honest.  A single max plus a timestamp
    /// cannot represent one: when the stored maximum aged out it was replaced by
    /// whatever sample was current, so a large hit followed by a medium one
    /// reported the medium value and then dropped to the idle level, discarding
    /// an impact still well inside the window.
    float    accelPeakSq[IMU_PEAK_BUCKETS];
    float    linAccelPeakSq[IMU_PEAK_BUCKETS];
    float    gyroPeakSq[IMU_PEAK_BUCKETS];
    uint32_t peakBucketMs[IMU_PEAK_BUCKETS];
    uint8_t  peakBucketHead;

    /// Saturation, tracked PER BUCKET for the same reason the peaks are.
    ///
    /// It began as a single latch and that was wrong: nothing cleared it, so one
    /// clipped sample marked every subsequent frame as saturated for the rest of
    /// the run. On the bench it stuck on at poll 12 000 and was still asserted
    /// 4 000 polls later with the sensor sitting still reading 9.83.
    ///
    /// That is worse than a cosmetic bug. The flag's meaning is "the peak beside
    /// me is a floor, not a measurement", so a stuck one devalues every honest
    /// reading that follows — and this is the field an incident detector would
    /// consult before deciding an impact was under-reported.
    bool     satBucket[IMU_PEAK_BUCKETS];

    /// Polls that should have happened and did not, saturating.  With no FIFO a
    /// missed poll is data that no longer exists anywhere, so this is the only
    /// record that the window has a hole in it.
    uint16_t missedPolls;
    uint32_t lastPollMs;

    /// Cumulative High-G latches since bring-up, saturating. The published flag
    /// lasts IMU_HIGHG_HOLD_MS, so a soak watching the console can miss every
    /// one and still look clean — this only ever climbs, so one glance answers
    /// "were there any?".
    uint16_t highGCount;
    /// Times the latch could not be cleared. Each one disarms the backstop, so
    /// this should be 0; a non-zero value with highGArmed false is the record of
    /// why the hardware impact detector stopped working.
    uint8_t  highGClearFails;
    bool     highGActive;      ///< Currently inside the republication hold.
    uint32_t highGUntilMs;     ///< Deadline for that hold.
    uint32_t highGAtMs;        ///< When the latch was first seen.
    /// Deadline form, not elapsed-time form: the gap notice is HELD until
    /// @c gapFlagUntilMs and then latched off.  Deriving it from "counter
    /// nonzero and the timestamp looks recent" republished a long-finished gap
    /// for one window every time @c millis() rolled over at 49.7 days.
    bool     gapFlagActive;
    uint32_t gapFlagUntilMs;

    /// Euler angles use the Android sign convention.  Read from the part rather
    /// than assumed, because it decides the sign of pitch and getting it wrong
    /// inverts a channel silently.
    bool     eulerAndroid;

    bool     quarantined;

    /**
     * Lifecycle latch, so quarantine survives a later @c initializeIMU().
     *
     * A separate field is needed because @c initializeIMU() resets every other
     * one — it has to, it is the initialiser — so @c quarantined alone would be
     * cleared by any caller that simply called it again.
     *
     * THE MEMBER INITIALISER IS LOAD-BEARING.  @c initializeIMU() and
     * @c imuMarkAbsent() both READ this before anything has written it, and the
     * helper sketches declare @c IMUDevice as an automatic.  Without an
     * initialiser that read is of an indeterminate value, which is undefined
     * behaviour outright — not a small chance of a wrong answer.
     */
    uint32_t lifecycle = IMU_LIFECYCLE_UNINIT;
};

/** What answered on the bus, and whether it is what we expected. */
struct I2CBusReport{
    uint8_t deviceCount;                        ///< Number of responders found.
    uint8_t addresses[I2C_SCAN_MAX_DEVICES];    ///< 7-bit addresses that ACKed.
    bool imuPresent;      ///< Something ACKed at a BNO055 address.
    bool imuIdentified;   ///< That responder returned CHIP_ID 0xA0.
    uint8_t imuAddress;   ///< Where it was identified. 0 when it was not.
    bool gpsPresent;      ///< Something ACKed at @c GPS_DEFAULT_I2C_ADDRESS.
    bool conflict;        ///< An IMU address is occupied by something that is not the IMU.
};

/**
 * @brief Starts the staged bring-up. Returns IMMEDIATELY — it does not block.
 *
 * The previous implementation configured the part inline and returned a verdict.
 * That cannot be done here without holding the CPU for the 650 ms the part needs
 * after a reset, and doing so is the specific mistake @c gpsInitTick() exists to
 * correct: nothing else runs, so the CAN drain stops and the telemetry push
 * jitters, and a hang lands at the same point on every boot under the same armed
 * watchdog — a reboot loop rather than a fault.
 *
 * So the caller must drive @c imuInitTick() from @c loop() until
 * @c imuIsReady(). Bring-up takes about 700 ms of WALL CLOCK while the loop
 * keeps turning over throughout.
 *
 * @param[in,out] dev   Device to bring up.
 * @param[in]     mode  @c Fusion (recommended) or @c Raw.
 * @return @c OK when the machine was armed, @c NOK_INIT_FAILED when the device
 *         is quarantined or the mode was rejected.
 */
IMUReturnStatus initializeIMU(IMUDevice &dev, IMUSampleMode mode);

/** @brief As above, in @c IMUSampleMode::Fusion. */
IMUReturnStatus initializeIMU(IMUDevice &dev);

/**
 * @brief Advances the bring-up by one step. Call from @c loop(), every pass.
 *
 * Cheap — at most a few single-byte transfers, a few hundred microseconds. Call
 * it unconditionally rather than on the poll timer: the sequence has about a
 * dozen steps, and running them at 20 Hz would add half a second to a bring-up
 * whose real cost is the part's own reset delay.
 *
 * A no-op once configured, so an unconditional call costs one comparison.
 *
 * @return The bring-up stage after this step.
 */
BNO055InitStage imuInitTick(IMUDevice &dev);

/** @brief True once the part is configured and running in the requested mode. */
bool imuIsReady(const IMUDevice &dev);

/**
 * @brief Restarts bring-up after the part was declared lost.
 *
 * Non-blocking, like @c initializeIMU(). Allocation-free and safe to call from
 * @c loop() on a rate limiter. A no-op returning @c OK when nothing is wrong.
 */
IMUReturnStatus recoverIMU(IMUDevice &dev);

/** @brief Resets a snapshot to the "nothing received" state (all @c NAN). */
void initIMUData(IMUData &data);

/**
 * @brief Reads one sample (non-blocking).
 *
 * ONE 46-byte burst covers accelerometer, magnetometer, gyroscope, Euler
 * angles, quaternion, linear acceleration, gravity, temperature and calibration
 * — registers 0x08 to 0x35 are contiguous.  Reading them separately would cost
 * five transactions and their addressing overhead for the same bytes; the
 * quaternion and, in fusion mode, the magnetometer come along unused and are
 * cheaper to discard than to avoid.
 *
 * TWO PLAUSIBILITY GATES stand between a successful transaction and a published
 * reading, because on this project a successful transaction has already proved
 * not to mean a valid one:
 *
 *   - TEMPERATURE against the part's own -40..+85 C rating.  Carried forward
 *     from the previous driver, where it caught 100 % of a failing sensor's
 *     corrupt bursts with no false positives over 520 good ones.
 *   - GRAVITY MAGNITUDE against about 9.81, in fusion mode.  New, and stronger:
 *     the fusion constructs that vector, so its length is near-constant whatever
 *     the vehicle does.  A burst that fails it is discarded whole.
 *
 * A failed gate discards the ENTIRE burst — it arrived in one transaction, so a
 * reading that cannot be trusted condemns the bytes beside it.
 *
 * @return @c OK on a fresh plausible sample, @c PARTIAL when the fused output is
 *         not yet trustworthy, @c DATA_STALE when nothing new arrived,
 *         @c NOK_NOT_READY during bring-up, @c NOK_BUS_STUCK when the bus was
 *         unsafe, or @c NOK_LINK_LOST once the part has been declared lost.
 */
IMUReturnStatus getIMUData(IMUDevice &dev, IMUData &data);

/** @brief True once the part has been declared lost. */
bool isIMULinkLost(const IMUDevice &dev);

/** @brief True when the part is not delivering usable data and should be retried. */
bool isIMUDegraded(const IMUDevice &dev);

/**
 * @brief Puts the device into a valid "nothing fitted" state WITHOUT touching the bus.
 *
 * ONE EXCEPTION, deliberate: a quarantine is CARRIED ACROSS rather than cleared.
 * Without that this would be a public way around @c imuQuarantine() — resetting
 * the struct would put a device that hung the board back into the polling and
 * recovery paths.
 */
void imuMarkAbsent(IMUDevice &dev);

/**
 * @brief Abandons the IMU for this boot, without any bus access. TERMINAL.
 *
 * For use after a watchdog reset.  Both bring-up and the recovery path that
 * @c isIMUDegraded() schedules transact, so on a boot following a hang BOTH have
 * to be suppressed — suppressing only initialisation leaves the retry timer to
 * re-enter the hang seconds later, which merely lengthens the reboot loop.
 */
void imuQuarantine(IMUDevice &dev);

/** @brief True when the IMU has been abandoned for this boot. */
bool isIMUQuarantined(const IMUDevice &dev);

/**
 * @brief Records a High-G edge seen on the INT pin. SAFE TO CALL FROM AN ISR.
 *
 * Optional. The latch is read over I2C on every poll regardless, so the wire
 * only removes poll latency — up to @c IMU_POLL_MS of it. What the pin buys is
 * the event's true timestamp, which matters for a dashcam deciding which frames
 * belong to an incident.
 *
 * Touches no bus and takes no lock: it stores a timestamp and sets a flag, both
 * volatile. Calling I2C from an interrupt would deadlock against the poll that
 * is very likely already inside Wire.
 */
void imuNoteHighGPin(uint32_t whenMs);

/** @brief The mode the part is currently configured for. */
IMUSampleMode imuSampleMode(const IMUDevice &dev);

/**
 * @brief Reconfigures the part into another mode. NON-BLOCKING, and expensive.
 *
 * Restarts the whole bring-up, so the IMU is unavailable for about 700 ms and
 * the peak window is discarded.  Call it on a session-level decision, never on a
 * trigger: a crash pulse lasts 10-50 ms and would be over before the part
 * finished switching.
 *
 * A no-op returning @c OK when already in @p mode.
 */
IMUReturnStatus setIMUSampleMode(IMUDevice &dev, IMUSampleMode mode);

/**
 * @brief Checks that the IMU address holds the IMU and nothing else.
 *
 * Answers what a datasheet cannot: an address being free "by default" is
 * worthless if the GNSS receiver was reconfigured or a second board was added.
 * Scans the bus, then identifies the IMU by CHIP_ID rather than by a bare
 * address ACK — something else can answer at 0x28 or 0x29, and a device that
 * answers but is not a BNO055 must read as a conflict, not as the IMU.
 *
 * Does not require the sensor to be initialised; opens the bus via
 * @c i2cBusBegin() itself.
 */
IMUReturnStatus checkI2CBusConflict(I2CBusReport &report);

#endif
