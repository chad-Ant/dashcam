#include <math.h>

#include "IMUFunctions.h"

static_assert(BNO055_I2C_ADDRESS_DEFAULT != GPS_DEFAULT_I2C_ADDRESS,
              "BNO055 default address collides with the GNSS receiver");
static_assert(BNO055_I2C_ADDRESS_ALT != GPS_DEFAULT_I2C_ADDRESS,
              "BNO055 alternative address collides with the GNSS receiver");
static_assert(BNO055_I2C_ADDRESS_DEFAULT != SEGLED_ADDRESS,
              "BNO055 default address collides with the segment LED");
static_assert(BNO055_I2C_ADDRESS_ALT != SEGLED_ADDRESS,
              "BNO055 alternative address collides with the segment LED");

// ─── the data burst ───────────────────────────────────────────────────────────
//
// Registers 0x08 to 0x35 are contiguous and cover everything this driver reads.
// One transaction instead of five: the addressing overhead of the extra four
// costs more than the fourteen bytes of quaternion and magnetometer that come
// along unused, and the whole burst is then a single coherent instant rather
// than five samples stitched together.

#define IMU_BURST_START   0x08u                 ///< ACC_DATA_X_LSB.
/// Through INT_STA at 0x37.
///
/// Two bytes longer than the data itself needs, and those two bytes are the
/// whole High-G mechanism. INT_STA carries the latched threshold crossing, and
/// including it here means the latch is read on EVERY poll at no extra
/// transaction — which is what lets the interrupt work with no wire to the INT
/// pin at all. ST_RESULT at 0x36 comes along in between and is discarded.
#define IMU_BURST_LEN     48u

#define IMU_OFF_ACCEL      0u                   ///< 0x08, 6 bytes.
#define IMU_OFF_MAG        6u                   ///< 0x0E, 6 bytes.
#define IMU_OFF_GYRO      12u                   ///< 0x14, 6 bytes.
#define IMU_OFF_EULER     18u                   ///< 0x1A, 6 bytes: heading, roll, pitch.
#define IMU_OFF_QUAT      24u                   ///< 0x20, 8 bytes. Read, not used.
#define IMU_OFF_LINACC    32u                   ///< 0x28, 6 bytes.
#define IMU_OFF_GRAVITY   38u                   ///< 0x2E, 6 bytes.
#define IMU_OFF_TEMP      44u                   ///< 0x34, 1 byte, signed degC.
#define IMU_OFF_CALIB     45u                   ///< 0x35, 1 byte, four 2-bit fields.
#define IMU_OFF_INT_STA   47u                   ///< 0x37, 1 byte. 0x36 is skipped.

// ─── scale factors, from the datasheet, with UNIT_SEL = 0 ─────────────────────
//
// MULTIPLIES, never divides. The SAMD21 is a Cortex-M0+ with no FPU, so every
// float operation is a software routine and a division is several times the cost
// of a multiply. This is a standing rule in this codebase, not a flourish.

#define IMU_SCALE_ACCEL_MS2   0.01f     ///< 100 LSB = 1 m/s2. Also linear accel and gravity.
#define IMU_SCALE_GYRO_DPS    0.0625f   ///< 16 LSB = 1 deg/s.
#define IMU_SCALE_EULER_DEG   0.0625f   ///< 16 LSB = 1 degree.
#define IMU_SCALE_MAG_UT      0.0625f   ///< 16 LSB = 1 microtesla.

/// Bytes of the burst compared poll-to-poll to detect a frozen data path, per
/// channel: accelerometer at burst offset 0 and gyroscope at 12, six each.
#define IMU_FREEZE_BYTES_PER_CHANNEL  6u

/// INT_STA bit 5 — accelerometer High-G.
#define IMU_INT_STA_HIGH_G    0x20u
/// SYS_TRIGGER bit 6 — RST_INT, clears the latch.
#define IMU_SYS_TRIGGER_RST_INT 0x40u
/// SYS_TRIGGER bit 7 — CLK_SEL. Preserved whenever the register is written, or
/// clearing an interrupt would silently deselect the external crystal.
#define IMU_SYS_TRIGGER_CLK_SEL 0x80u
/// Attempts to clear the High-G latch before declaring the backstop degraded.
#define IMU_HIGHG_CLEAR_RETRIES 3u

/// Set from the INT-pin interrupt handler, consumed by the next poll.
///
/// Two separate flags rather than one, because the pin and the register are
/// different evidence: the pin gives the true instant, the register proves the
/// part actually latched. The poll reconciles them.
static volatile bool     gPinEvent   = false;
static volatile uint32_t gPinEventMs = 0;

void imuNoteHighGPin(uint32_t whenMs){
    gPinEventMs = whenMs;
    gPinEvent   = true;
}

static inline int16_t toInt16LE(const uint8_t *buf){
    return static_cast<int16_t>(static_cast<uint16_t>(buf[0]) |
                                (static_cast<uint16_t>(buf[1]) << 8));
}

/** @brief Reads @p len bytes from @p reg through the counted transport. */
static bool readBurst(IMUDevice &dev, uint8_t reg, uint8_t *buf, uint8_t len){
    return bno055BusRead(dev.init.address, reg, buf, len) == 0;
}

/** @brief Single-register write through the counted transport. */
static bool regWrite8Imu(IMUDevice &dev, uint8_t reg, uint8_t value){
    unsigned char v = value;
    return bno055BusWrite(dev.init.address, reg, &v, 1u) == 0;
}

// ─── fault accounting ─────────────────────────────────────────────────────────

/**
 * @brief Records a failed read and retires the part once failures pile up.
 *
 * One NACK is a glitch worth riding out; @c IMU_MAX_CONSECUTIVE_FAULTS in a row
 * is a sensor that is gone. Retiring it is what makes @c isIMUDegraded() true
 * and hands the caller a defined recovery path, instead of an endless retry
 * inside the read.
 */
static void noteFault(IMUDevice &dev){
    if (dev.faults < 0xFFu) dev.faults++;
    // Saturating rather than wrapping: 65535 reads as "a lot, stopped counting",
    // where a wrapped 3 would read as a nearly clean run.
    if (dev.ioErrors < 0xFFFFu) dev.ioErrors++;
    if (dev.faults >= IMU_MAX_CONSECUTIVE_FAULTS) dev.ready = false;
}

/**
 * @brief Records a burst that arrived intact and failed a plausibility gate.
 *
 * Counted apart from @c noteFault() because the two mean opposite things about
 * the hardware. A NACK says the part did not answer. This says it answered
 * perfectly and the contents were impossible — which is how the previous IMU
 * failed, and the case no transport-level counter can see.
 */
static void noteImplausible(IMUDevice &dev){
    if (dev.implausible < 0xFFFFu) dev.implausible++;
    if (dev.faults < 0xFFu) dev.faults++;
    if (dev.faults >= IMU_MAX_CONSECUTIVE_FAULTS) dev.ready = false;
}

/**
 * @brief Records a High-G latch and clears it so the next impact can arm.
 *
 * Called immediately after the burst, before any validation gate — see the call
 * site for why that ordering is not incidental.
 *
 * @param intSta  The INT_STA byte from this burst.
 */
static void noteHighG(IMUDevice &dev, uint8_t intSta, uint32_t now)
{
    const bool latched = (intSta & IMU_INT_STA_HIGH_G) != 0u;

    // ── is the latch actually clearing? ──────────────────────────────────────
    //
    // Checked BEFORE anything else uses the bit, and checked against the bus
    // rather than against the write's acknowledgement. A clear that is ACKed and
    // discarded — by a part on the wrong register page, the failure this project
    // has already had once — used to leave INT asserted forever while
    // highGArmed went on advertising a working backstop. The next burst answers
    // the question at no extra transaction: this one is that burst.
    if (!latched){
        dev.highGStuckTracking = false;
    } else if (!dev.highGStuckTracking){
        dev.highGStuckTracking = true;
        dev.highGStuckSinceMs  = now;
    } else if (dev.init.highGArmed &&
               ((now - dev.highGStuckSinceMs) > IMU_HIGHG_STUCK_MS)){
        // Degraded, not fatal, and for the same reason a failed clear write is:
        // the sample data is unaffected and the peaks still work. What is gone
        // is the backstop, and a consumer weighing an incident has to be told
        // that the latch it would have relied on has stopped arming.
        dev.init.highGArmed = false;
        if (dev.highGClearFails < 0xFFu) dev.highGClearFails++;
    }

    bool     fired  = latched;
    uint32_t whenMs = now;

    // Snapshot the ISR flags with interrupts masked. Reading a flag and its
    // timestamp separately allows an edge to land between the two, which would
    // pair a stale time with a fresh event — on a field whose whole job is
    // saying WHEN an impact happened.
    noInterrupts();
    const bool     pin   = gPinEvent;
    const uint32_t pinMs = gPinEventMs;
    gPinEvent = false;
    interrupts();

    if (pin){
        fired  = true;
        whenMs = pinMs;
    }
    if (!fired) return;

    if (!dev.highGActive){
        dev.highGAtMs = whenMs;
        if (dev.highGCount < 0xFFFFu) dev.highGCount++;
    }
    dev.highGActive  = true;
    dev.highGUntilMs = now + IMU_HIGHG_HOLD_MS;

    // Clearing the latch, and CHECKING that it cleared.
    //
    // Two defects lived in the single line this replaces. The return was
    // discarded, so a failed clear left INT asserted — after which no further
    // rising edge can occur and no later latch is distinguishable from the stale
    // one, while @c highGArmed went on claiming the backstop was live. That is
    // silent loss of the one signal designed to survive things going wrong.
    //
    // And the write was a bare RST_INT, which zeroes the whole register —
    // including CLK_SEL. With the external crystal currently disabled that is
    // harmless, but it would have switched the part back to its internal
    // oscillator the first time anyone enabled it, on a path that runs only when
    // an impact is detected. The bit is preserved from what bring-up settled on.
    const uint8_t trigger = (uint8_t)(IMU_SYS_TRIGGER_RST_INT |
                                      (dev.init.externalCrystal ? IMU_SYS_TRIGGER_CLK_SEL : 0u));

    bool cleared = false;
    for (uint8_t attempt = 0; attempt < IMU_HIGHG_CLEAR_RETRIES && !cleared; ++attempt){
        cleared = regWrite8Imu(dev, BNO055_SYS_TRIGGER_ADDR, trigger);
    }
    if (!cleared){
        // Degraded, not fatal: sample data is unaffected and the peaks still
        // work. What is gone is the hardware backstop, and saying so is the
        // point — a consumer weighing an incident needs to know the latch it
        // would have relied on has stopped arming.
        dev.init.highGArmed = false;
        if (dev.highGClearFails < 0xFFu) dev.highGClearFails++;
    }
}

// ─── peak ring ────────────────────────────────────────────────────────────────
//
// The two properties the ring's sizing exists to deliver, asserted rather than
// reasoned about in a comment. Both were violated by the previous constants and
// neither failure was visible: a peak simply came out lower than it should have,
// on the field incident severity is graded from, and nothing anywhere said so.

/// A bucket must stop being eligible BEFORE the head wraps round and clears it.
/// Otherwise a bucket contributes to a published peak and is then wiped while
/// still inside the window, which is the under-report arriving early.
static_assert(IMU_PEAK_ELIGIBLE_MS < ((uint32_t)IMU_PEAK_BUCKETS * IMU_PEAK_BUCKET_MS),
              "peak ring is too short: buckets are cleared while still eligible");

/// Every sample must live at least the window it is published as covering,
/// whatever its phase within its bucket. A sample landing at the very end of a
/// bucket is the worst case, and it is the one the old sizing failed.
static_assert((IMU_PEAK_ELIGIBLE_MS - IMU_PEAK_BUCKET_MS) >= IMU_PEAK_WINDOW_MS,
              "peak retention is shorter than the window it advertises");

static void resetPeakRing(IMUDevice &dev, uint32_t now){
    for (uint8_t i = 0u; i < IMU_PEAK_BUCKETS; i++){
        dev.accelPeakSq[i]    = NAN;
        dev.linAccelPeakSq[i] = NAN;
        dev.gyroPeakSq[i]     = NAN;
        dev.satBucket[i]      = false;
        dev.peakBucketMs[i]   = now;
    }
    dev.peakBucketHead = 0u;
}

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
        dev.accelPeakSq[dev.peakBucketHead]    = NAN;
        dev.linAccelPeakSq[dev.peakBucketHead] = NAN;
        dev.gyroPeakSq[dev.peakBucketHead]     = NAN;
        dev.satBucket[dev.peakBucketHead]      = false;
        dev.peakBucketMs[dev.peakBucketHead]   = now;
    }
}

/** @brief Folds one sample's squared magnitude into the current bucket. */
static void notePeakSq(float valueSq, float *ring, uint8_t head){
    if (isnan(ring[head]) || (valueSq > ring[head])) ring[head] = valueSq;
}

/**
 * @brief Largest value across the buckets still inside the window.
 *
 * Eligibility is measured to the bucket's END, not its start — see
 * @c IMU_PEAK_ELIGIBLE_MS. Comparing against the start retired a bucket whose
 * newest sample was still well inside the window, which under-reported a peak by
 * up to a whole bucket on the field incident severity is graded from.
 *
 * @return @c NAN when every bucket is empty, which is honest: no sample has
 *         arrived recently enough to support a peak.
 */
static float ringMaxSq(const IMUDevice &dev, const float *ring, uint32_t now){
    float best = NAN;
    for (uint8_t i = 0u; i < IMU_PEAK_BUCKETS; i++){
        if (isnan(ring[i])) continue;
        if ((now - dev.peakBucketMs[i]) > IMU_PEAK_ELIGIBLE_MS) continue;
        if (isnan(best) || (ring[i] > best)) best = ring[i];
    }
    return best;
}

/**
 * @brief Converts the tracked squared peaks into the published magnitudes.
 *
 * Three square roots per poll, taken once the window has been folded in rather
 * than once per sample. On a part with no FPU, where sqrtf is called from is not
 * a detail.
 */
static void publishPeaks(const IMUDevice &dev, IMUData &data, uint32_t now){
    const float aSq = ringMaxSq(dev, dev.accelPeakSq,    now);
    const float lSq = ringMaxSq(dev, dev.linAccelPeakSq, now);
    const float gSq = ringMaxSq(dev, dev.gyroPeakSq,     now);
    data.accelPeakMs2    = isnan(aSq) ? NAN : sqrtf(aSq);
    data.linAccelPeakMs2 = isnan(lSq) ? NAN : sqrtf(lSq);
    data.gyroPeakDps     = isnan(gSq) ? NAN : sqrtf(gSq);

    // Saturation over the SAME window as the peaks it qualifies, expiring by the
    // same rule. A bucket outside the window describes a peak that is no longer
    // published, so its saturation must not be either.
    bool sat = false;
    for (uint8_t i = 0u; i < IMU_PEAK_BUCKETS; i++){
        if (!dev.satBucket[i]) continue;
        if ((now - dev.peakBucketMs[i]) > IMU_PEAK_ELIGIBLE_MS) continue;
        sat = true;
    }
    data.accelSaturated = sat;
}

// ─── snapshot handling ────────────────────────────────────────────────────────

void initIMUData(IMUData &data){
    data.accelX = NAN;  data.accelY = NAN;  data.accelZ = NAN;
    data.gyroX  = NAN;  data.gyroY  = NAN;  data.gyroZ  = NAN;
    data.magX   = NAN;  data.magY   = NAN;  data.magZ   = NAN;

    data.temperatureC = NAN;

    data.linAccelX = NAN; data.linAccelY = NAN; data.linAccelZ = NAN;
    data.gravityX  = NAN; data.gravityY  = NAN; data.gravityZ  = NAN;
    data.yawRelDeg = NAN; data.pitchDeg  = NAN; data.rollDeg   = NAN;

    data.accelPeakMs2    = NAN;
    data.linAccelPeakMs2 = NAN;
    data.gyroPeakDps     = NAN;
    data.accelSaturated  = false;
    data.highGEvent      = false;
    data.highGMs         = 0u;
    data.highGArmed      = false;
    data.dataGap         = false;
    data.lowPower        = false;
    data.fusionMode      = false;

    data.calibSys = 0u; data.calibGyro = 0u; data.calibAccel = 0u; data.calibMag = 0u;

    data.accelSampleMs  = 0u;
    data.gyroSampleMs   = 0u;
    data.tempSampleMs   = 0u;
    data.magSampleMs    = 0u;
    data.fusionSampleMs = 0u;

    data.accelValid  = false;
    data.gyroValid   = false;
    data.magValid    = false;
    data.tempValid   = false;
    data.fusionValid = false;

    data.devicePresent     = false;
    data.allDevicesPresent = false;
}

static void invalidateAccel(IMUData &data){
    data.accelX = NAN; data.accelY = NAN; data.accelZ = NAN;
    // The peak goes with the axes. It is derived from the same samples, so a
    // peak surviving its own channel's expiry would be the one number on the
    // frame still claiming a measurement after the sensor stopped supplying one.
    data.accelPeakMs2   = NAN;
    data.accelSaturated = false;
    data.accelValid     = false;
}

static void invalidateGyro(IMUData &data){
    data.gyroX = NAN; data.gyroY = NAN; data.gyroZ = NAN;
    data.gyroPeakDps = NAN;
    data.gyroValid   = false;
}

static void invalidateMagnetic(IMUData &data){
    data.magX = NAN; data.magY = NAN; data.magZ = NAN;
    data.magValid = false;
}

static void invalidateTemp(IMUData &data){
    data.temperatureC = NAN;
    data.tempValid    = false;
}

static void invalidateFusion(IMUData &data){
    data.linAccelX = NAN; data.linAccelY = NAN; data.linAccelZ = NAN;
    data.gravityX  = NAN; data.gravityY  = NAN; data.gravityZ  = NAN;
    data.yawRelDeg = NAN; data.pitchDeg  = NAN; data.rollDeg   = NAN;
    data.linAccelPeakMs2 = NAN;
    data.fusionValid     = false;
}

/** @brief True when a timestamped reading is older than the freshness window. */
static inline bool isExpired(uint32_t sampleMs, uint32_t nowMs){
    // Unsigned subtraction, so this stays correct across the millis() rollover
    // at 49.7 days — a comparison against (sampleMs + window) would not.
    return (nowMs - sampleMs) > IMU_MAX_DATA_AGE_MS;
}

/**
 * @brief Raises the gap notice when the record has been without a sample too long.
 *
 * MEASURED FROM THE LAST ACCEPTED BURST, not from the last poll ATTEMPT, and the
 * difference is a whole class of hole that was going unreported. A poll that runs
 * exactly on time and then has its burst rejected — by the temperature gate, the
 * gravity gate or the accel-limit gate — leaves precisely the same absence in the
 * inertial record as a poll that never happened. There is no FIFO to fill it in
 * either case.
 *
 * The old test compared against the attempt and reset its timer before the
 * transaction, so a sensor failing every gate for 200 ms restarted the clock
 * every 10 ms and reported a continuous, healthy record over a window in which
 * nothing at all had been accepted. That is the exact shape of the failure this
 * driver exists to catch: the part answering perfectly and the contents being
 * unusable.
 *
 * One threshold serves both causes because the consequence is identical, and it
 * is the same @c IMU_GAP_MIN_MS with the same reasoning behind it — below that
 * the hardware High-G latch covers an impact landing in the hole.
 */
static void noteGapIfStarved(IMUDevice &dev, uint32_t now){
    if (!dev.ready) return;   // absent hardware is reported as absent, not as a gap
    if ((now - dev.lastGoodMs) <= IMU_GAP_MIN_MS) return;

    dev.gapFlagActive  = true;
    dev.gapFlagUntilMs = now + IMU_PEAK_WINDOW_MS;
}

/** @brief Ages out any channel past its freshness window, without touching the bus. */
static void expireChannels(IMUDevice &dev, IMUData &data, uint32_t now){
    noteGapIfStarved(dev, now);

    if (!dev.ready || isExpired(data.accelSampleMs,  now)) invalidateAccel(data);
    if (!dev.ready || isExpired(data.gyroSampleMs,   now)) invalidateGyro(data);
    if (!dev.ready || isExpired(data.tempSampleMs,   now)) invalidateTemp(data);
    if (!dev.ready || isExpired(data.magSampleMs,    now)) invalidateMagnetic(data);
    if (!dev.ready || isExpired(data.fusionSampleMs, now)) invalidateFusion(data);

    // The gap notice is HELD for a full window rather than cleared on the next
    // poll. Polls run faster than telemetry publishes, so a flag lasting one
    // poll would be missed by most frames — and the frames missing it are
    // exactly the ones whose peak is untrustworthy.
    //
    // An explicit deadline compared with SIGNED arithmetic, not "counter nonzero
    // AND the timestamp looks recent". That older form resurrects the flag at
    // the millis() rollover: 49.7 days after a gap the elapsed test reads as ~0
    // again and a long-finished gap is republished. Rare, but a false report of
    // missing data on a system whose whole point is not making those.
    if (dev.gapFlagActive && (static_cast<int32_t>(now - dev.gapFlagUntilMs) >= 0)){
        dev.gapFlagActive = false;
    }
    data.dataGap = dev.gapFlagActive;

    // The High-G hold, on the same deadline discipline and for the same reason.
    if (dev.highGActive && (static_cast<int32_t>(now - dev.highGUntilMs) >= 0)){
        dev.highGActive = false;
    }
    data.highGEvent = dev.highGActive;
    data.highGMs    = dev.highGAtMs;
}

/**
 * @brief The MINIMUM LOCAL USABILITY GATE on the fused output.
 *
 * Named for what it is rather than for what it sounds like. It is not a verdict
 * on quality and cannot be one: it consults a single field.
 *
 * Gyroscope only. The accelerometer figure was a criterion until a bench run
 * showed it decaying to 0 within about 19 seconds of a successful profile
 * restore and staying there for thousands of polls, while the gravity magnitude
 * held 9.79-9.81 throughout — so it was reporting PARTIAL indefinitely on output
 * that a gross physical check says is fine. See @c IMU_CALIB_MIN_GYRO.
 *
 * That gravity magnitude is a fusion OUTPUT, so its steadiness is a
 * self-consistency test rather than an independent physical bound on
 * accelerometer bias — which is the other reason this is a floor and not a
 * judgement. A consumer wanting an accelerometer-calibration policy has the raw
 * figure on the wire in @c imuCalib and should enforce it there.
 */
static bool fusionTrustworthy(const IMUData &data){
    return data.calibGyro >= IMU_CALIB_MIN_GYRO;
}

/** @brief Publishes hardware presence from device state, not from this poll's luck. */
static void publishPresence(const IMUDevice &dev, IMUData &data){
    data.devicePresent = dev.ready;
    data.fusionMode    = (dev.init.opMode == OPERATION_MODE_IMUPLUS);
    data.lowPower      = false;   // parked low-power mode is not enabled; see the header

    // "Running exactly as configured" — see the field's documentation for why
    // the name outlived its original meaning. A part that fell back to the
    // internal oscillator is still supplying good data, and is not supplying the
    // configuration that was asked for.
    data.allDevicesPresent = dev.ready && !dev.init.clockFallback;
    data.highGArmed        = dev.ready && dev.init.highGArmed;
}

// ─── lifecycle ────────────────────────────────────────────────────────────────

void imuMarkAbsent(IMUDevice &dev){
    const bool wasQuarantined = (dev.lifecycle == IMU_LIFECYCLE_QUARANTINED) || dev.quarantined;

    dev.ready       = false;
    dev.faults      = 0u;
    dev.ioErrors    = 0u;
    dev.implausible = 0u;

    dev.accelChangeMs = millis();
    dev.gyroChangeMs  = millis();
    dev.lastRawValid  = false;
    dev.missedPolls   = 0u;
    dev.lastPollMs    = millis();
    // Seeded to now, not to zero. The gap flag is an elapsed-time test against
    // this, so a zero would read as a 49-day hole on the very first poll.
    dev.lastGoodMs    = millis();
    dev.gapFlagActive = false;
    dev.gapFlagUntilMs = millis();
    dev.eulerAndroid  = false;

    dev.highGCount      = 0u;
    dev.highGClearFails = 0u;
    dev.highGActive  = false;
    dev.highGUntilMs = millis();
    dev.highGAtMs    = 0u;
    dev.highGStuckTracking = false;
    dev.highGStuckSinceMs  = millis();
    // Any pin edge from before this reset describes a device that is being
    // re-initialised, so it belongs to nothing.
    noInterrupts();
    gPinEvent = false;
    interrupts();

    resetPeakRing(dev, millis());

    // Carried across rather than cleared. Without this, resetting the struct was
    // a public way around imuQuarantine() — it would put a device that hung the
    // board back into the polling and recovery paths.
    dev.quarantined = wasQuarantined;
    dev.lifecycle   = wasQuarantined ? IMU_LIFECYCLE_QUARANTINED : IMU_LIFECYCLE_ACTIVE;
}

void imuQuarantine(IMUDevice &dev){
    imuMarkAbsent(dev);
    dev.quarantined = true;
    dev.lifecycle   = IMU_LIFECYCLE_QUARANTINED;
    bno055InitQuarantine(dev.init, BNO055InitStatus::NOK_BUS_STUCK);
}

bool isIMUQuarantined(const IMUDevice &dev){
    return dev.quarantined;
}

IMUReturnStatus initializeIMU(IMUDevice &dev, IMUSampleMode mode){
    if (dev.quarantined || (dev.lifecycle == IMU_LIFECYCLE_QUARANTINED)){
        return IMUReturnStatus::NOK_INIT_FAILED;
    }

    imuMarkAbsent(dev);

    const uint8_t opMode = (mode == IMUSampleMode::Raw) ? OPERATION_MODE_AMG
                                                        : OPERATION_MODE_IMUPLUS;
    if (!bno055InitBegin(dev.init, opMode)) return IMUReturnStatus::NOK_INIT_FAILED;
    return IMUReturnStatus::OK;
}

IMUReturnStatus initializeIMU(IMUDevice &dev){
    return initializeIMU(dev, IMUSampleMode::Fusion);
}

BNO055InitStage imuInitTick(IMUDevice &dev){
    if (dev.quarantined) return BNO055InitStage::Quarantined;

    const BNO055InitStage stage = bno055InitTick(dev.init);

    // The edge into Configured is where the device becomes usable, and it is
    // taken once: dev.ready is false until here, so a mid-bring-up poll cannot
    // publish a reading from a part that has not finished being told what to do.
    if ((stage == BNO055InitStage::Configured) && !dev.ready){
        const uint32_t now = millis();
        dev.ready         = true;
        dev.faults        = 0u;
        dev.accelChangeMs = now;
        dev.gyroChangeMs  = now;
        dev.lastRawValid  = false;
        dev.lastPollMs    = now;
        // The record starts here, so the first poll is not charged for the
        // bring-up that preceded it.
        dev.lastGoodMs    = now;
        dev.eulerAndroid  = (dev.init.unitSelSeen & 0x80u) != 0u;
        resetPeakRing(dev, now);
        dev.lifecycle    = IMU_LIFECYCLE_ACTIVE;
    }
    return stage;
}

bool imuIsReady(const IMUDevice &dev){
    return dev.ready && bno055InitReady(dev.init);
}

IMUReturnStatus recoverIMU(IMUDevice &dev){
    if (dev.quarantined) return IMUReturnStatus::NOK_INIT_FAILED;
    if (imuIsReady(dev))  return IMUReturnStatus::OK;
    return initializeIMU(dev, imuSampleMode(dev));
}

IMUSampleMode imuSampleMode(const IMUDevice &dev){
    return (dev.init.opMode == OPERATION_MODE_AMG) ? IMUSampleMode::Raw
                                                   : IMUSampleMode::Fusion;
}

IMUReturnStatus setIMUSampleMode(IMUDevice &dev, IMUSampleMode mode){
    if (imuSampleMode(dev) == mode) return IMUReturnStatus::OK;
    return initializeIMU(dev, mode);
}

// ─── the poll ─────────────────────────────────────────────────────────────────

IMUReturnStatus getIMUData(IMUDevice &dev, IMUData &data){
    const uint32_t now = millis();

    // BEFORE i2cBusBegin(), and that ordering is the whole point. A quarantined
    // boot promises to touch no I2C at all, and i2cBusBegin() is not a passive
    // question: on a stuck bus it performs GPIO-level recovery — nine clock
    // pulses, a STOP and Wire.begin() — every I2C_BUS_RECOVER_RETRY_MS. With
    // this call below the bus check, a 20 Hz poll drove that recovery four times
    // a second for an entire quarantined boot. It issued no addressed
    // transaction, so it did not reproduce the hang, but it plainly broke the
    // promise the quarantine makes.
    if (dev.quarantined){
        invalidateAccel(data); invalidateGyro(data); invalidateMagnetic(data);
        invalidateTemp(data);  invalidateFusion(data);
        publishPresence(dev, data);
        return IMUReturnStatus::NOK_LINK_LOST;
    }

    if (!dev.ready){
        expireChannels(dev, data, now);
        publishPresence(dev, data);
        // Bring-up in progress is NOT a failure, and saying so matters: the
        // caller's recovery timer keys off isIMUDegraded(), and reporting a
        // link loss during the part's own 650 ms reset would restart a bring-up
        // that was progressing normally.
        return bno055InitReady(dev.init) ? IMUReturnStatus::NOK_LINK_LOST
                                         : IMUReturnStatus::NOK_NOT_READY;
    }

    // Checked on EVERY poll, not just at bring-up. A slave that browns out or
    // resets mid-drive holds SDA from that moment, and the read below goes
    // straight into SERCOM::startTransmissionWIRE()'s undeadlined
    // `while (!isBusIdleWIRE() && !isBusOwnerWIRE());`. Without this the only
    // thing that ends the hang is the watchdog, and a reboot is not the required
    // response to a peripheral fault — staying up and logging it is.
    //
    // A stuck bus is charged to NO fault counter. The bus is shared, so counting
    // it against the IMU would retire a healthy sensor for a fault that is not
    // its own and then hide the real cause behind an IMU that reads as absent.
    if (i2cBusBegin() != I2CBusState::Ready){
        expireChannels(dev, data, now);
        publishPresence(dev, data);
        return IMUReturnStatus::NOK_BUS_STUCK;
    }

    // A poll that did not happen is data that no longer exists. The previous
    // part's FIFO covered a stalled loop up to its 2.3 s depth; this one has no
    // buffer at all, so the gap flag is the only record that the window has a
    // hole in it — which makes it far more load-bearing here than it was there.
    //
    // This counts the LOOP's failures to arrive on time, which is a diagnostic
    // about the firmware. The gap FLAG is raised elsewhere, from the last burst
    // actually accepted, because a poll arriving punctually and being thrown out
    // by a plausibility gate leaves the identical hole — see noteGapIfStarved().
    if ((now - dev.lastPollMs) > IMU_GAP_MIN_MS){
        if (dev.missedPolls < 0xFFFFu) dev.missedPolls++;
    }
    dev.lastPollMs = now;

    rollPeakRing(dev, now);

    uint8_t buf[IMU_BURST_LEN];
    if (!readBurst(dev, IMU_BURST_START, buf, IMU_BURST_LEN)){
        noteFault(dev);
        expireChannels(dev, data, now);
        publishPresence(dev, data);
        return dev.ready ? IMUReturnStatus::DATA_STALE : IMUReturnStatus::NOK_LINK_LOST;
    }

    // ── High-G latch, BEFORE any validation gate ─────────────────────────────
    //
    // Ordering is load-bearing and this used to be wrong. The latch was handled
    // after the temperature and gravity checks, both of which discard the burst
    // and return — so an impact that arrived in the same burst as one bad byte
    // was thrown away with it.
    //
    // The two facts are independent. A temperature outside the part's rated
    // range says the DATA path is untrustworthy; it says nothing about whether
    // the accelerometer comparator latched a threshold crossing, which happened
    // in hardware before any of these bytes were assembled. Discarding an impact
    // notification because the byte next to it looked wrong is exactly backwards
    // for the one signal that is supposed to survive things going wrong.
    //
    // There is a second reason, which could not be settled from the datasheet on
    // hand: if INT_STA turns out to be cleared BY READING, then the burst above
    // has already consumed the event and a later return loses it permanently.
    // Handling it here is correct whether or not that is true, which is the
    // better position to be in than being right about the register.
    noteHighG(dev, buf[IMU_OFF_INT_STA], now);

    // ── gate 1: temperature ──────────────────────────────────────────────────
    // A transaction that SUCCEEDS can still deliver the wrong bytes, and the
    // fault counter above only ever sees NACKs.
    //
    // Measured on the failing LSM6DSOX this replaces: 861 samples, 341 of them
    // corrupt, every corrupt one carrying 104.35 or 116.82 C on a 25 C bench —
    // only two distinct values across all 341, so a broken data path rather than
    // noise. The accelerometer in those same samples read a fixed 27.8 m/s2,
    // which is 2.85 g: a fabricated crash, on the exact field that triggers
    // incident capture, and well inside the wire format's range so nothing
    // downstream could have caught it.
    const float tempC = static_cast<float>(static_cast<int8_t>(buf[IMU_OFF_TEMP]));
    if (!(tempC > IMU_TEMP_MIN_VALID_C && tempC < IMU_TEMP_MAX_VALID_C)){
        noteImplausible(dev);
        expireChannels(dev, data, now);
        publishPresence(dev, data);
        return dev.ready ? IMUReturnStatus::DATA_STALE : IMUReturnStatus::NOK_LINK_LOST;
    }

    const bool fusion = (dev.init.opMode == OPERATION_MODE_IMUPLUS);

    const int16_t grx = toInt16LE(&buf[IMU_OFF_GRAVITY    ]);
    const int16_t gry = toInt16LE(&buf[IMU_OFF_GRAVITY + 2]);
    const int16_t grz = toInt16LE(&buf[IMU_OFF_GRAVITY + 4]);

    // The fusion has not produced an estimate yet.
    //
    // Distinguished from corrupt data, and it has to be. The registers read as
    // exact zeros for the first poll or two after the part enters an operating
    // mode, before the algorithm has run — which fails the magnitude gate below
    // and, until this check existed, was counted as "the part answered and the
    // contents were impossible". Two of those on EVERY boot, on the one counter
    // that exists to raise the alarm for the corrupt-data failure that retired
    // the last sensor. A counter that is never zero is a counter nobody reads.
    //
    // Exactly zero is unambiguous: gravity has a magnitude of 9.81 by
    // construction, so all three components being precisely 0 cannot be a
    // measurement. The raw accelerometer and gyroscope in the same burst are
    // unaffected and are still published; only the fused fields wait.
    const bool fusionIdle = fusion && (grx == 0) && (gry == 0) && (grz == 0);

    const float gx = static_cast<float>(grx) * IMU_SCALE_ACCEL_MS2;
    const float gy = static_cast<float>(gry) * IMU_SCALE_ACCEL_MS2;
    const float gz = static_cast<float>(grz) * IMU_SCALE_ACCEL_MS2;

    // ── gate 2: gravity magnitude ────────────────────────────────────────────
    // Stronger than the temperature gate, and available only because this part
    // fuses on-chip. The algorithm CONSTRUCTS the gravity vector, so its length
    // is near-constant at 9.81 whatever the vehicle is doing — cornering,
    // braking and potholes rotate it, they do not stretch it. A burst that fails
    // this is discarded whole: it arrived in one transaction, so bytes that
    // cannot be trusted condemn the bytes beside them.
    //
    // Measured over 1682 polls on the bench: 9.79 to 9.81, zero failures. That
    // spread is what makes the gate worth having — a band this tight around a
    // constant is a strong test.
    if (fusion && !fusionIdle){
        const float gMagSq = (gx * gx) + (gy * gy) + (gz * gz);
        if (!(gMagSq > (IMU_GRAVITY_MIN_VALID_MS2 * IMU_GRAVITY_MIN_VALID_MS2) &&
              gMagSq < (IMU_GRAVITY_MAX_VALID_MS2 * IMU_GRAVITY_MAX_VALID_MS2))){
            noteImplausible(dev);
            expireChannels(dev, data, now);
            publishPresence(dev, data);
            return dev.ready ? IMUReturnStatus::DATA_STALE : IMUReturnStatus::NOK_LINK_LOST;
        }
    }

    // ── frozen-data check ────────────────────────────────────────────────────
    // What replaces the per-channel data-ready stall test the LSM6DSOX offered
    // and this part does not. Real inertial data is never bit-identical twice
    // running: gyro noise alone moves the low bits every sample. So bytes that
    // stop changing are a frozen data path on a device that is still answering
    // every transaction perfectly — which is precisely how the previous sensor
    // failed, and a fault no bus-level counter could ever see.
    //
    // PER CHANNEL, with a timer each. One flag over all twelve bytes is set by
    // ANY of them moving, and gyro noise never stops — so the combined form could
    // not detect a frozen accelerometer at all, however long it stayed frozen.
    // The sensor this driver replaced failed by returning a FIXED 27.8 m/s2
    // accelerometer, which is to say the check documented as catching that exact
    // failure would have been held open by the noise beside it. The masking runs
    // both ways: driving over a rough surface would equally have hidden a frozen
    // gyroscope.
    {
        const bool first = !dev.lastRawValid;

        bool accelChanged = first;
        for (uint8_t i = 0u; i < IMU_FREEZE_BYTES_PER_CHANNEL; i++){
            const uint8_t src = buf[IMU_OFF_ACCEL + i];
            if (dev.lastRaw[i] != src) accelChanged = true;
            dev.lastRaw[i] = src;
        }

        bool gyroChanged = first;
        for (uint8_t i = 0u; i < IMU_FREEZE_BYTES_PER_CHANNEL; i++){
            const uint8_t src = buf[IMU_OFF_GYRO + i];
            if (dev.lastRaw[IMU_FREEZE_BYTES_PER_CHANNEL + i] != src) gyroChanged = true;
            dev.lastRaw[IMU_FREEZE_BYTES_PER_CHANNEL + i] = src;
        }

        dev.lastRawValid = true;
        if (accelChanged) dev.accelChangeMs = now;
        if (gyroChanged)  dev.gyroChangeMs  = now;

        // EITHER channel stalling retires the part. They share one die, one
        // regulator and one bus, so a channel that has stopped converting is
        // evidence about the device rather than about that channel — and there
        // is no per-channel repair available in any case: the recovery path is
        // a full reconfiguration either way.
        if (((now - dev.accelChangeMs) > IMU_MAX_CHANNEL_STALL_MS) ||
            ((now - dev.gyroChangeMs)  > IMU_MAX_CHANNEL_STALL_MS)){
            // Retired rather than merely expired. Expiry blanks the signal and
            // leaves it blank; retiring is what makes isIMUDegraded() true and
            // gets the part reconfigured, which is the only thing that might
            // actually bring it back.
            dev.ready = false;
            expireChannels(dev, data, now);
            publishPresence(dev, data);
            return IMUReturnStatus::NOK_LINK_LOST;
        }
    }

    dev.faults = 0u;

    // ── decode ───────────────────────────────────────────────────────────────
    const float ax = static_cast<float>(toInt16LE(&buf[IMU_OFF_ACCEL    ])) * IMU_SCALE_ACCEL_MS2;
    const float ay = static_cast<float>(toInt16LE(&buf[IMU_OFF_ACCEL + 2])) * IMU_SCALE_ACCEL_MS2;
    const float az = static_cast<float>(toInt16LE(&buf[IMU_OFF_ACCEL + 4])) * IMU_SCALE_ACCEL_MS2;

    const float wx = static_cast<float>(toInt16LE(&buf[IMU_OFF_GYRO    ])) * IMU_SCALE_GYRO_DPS;
    const float wy = static_cast<float>(toInt16LE(&buf[IMU_OFF_GYRO + 2])) * IMU_SCALE_GYRO_DPS;
    const float wz = static_cast<float>(toInt16LE(&buf[IMU_OFF_GYRO + 4])) * IMU_SCALE_GYRO_DPS;

    // ── gate 3: physically impossible acceleration ───────────────────────────
    // Saturation and corruption are different things and were being conflated.
    // The part cannot report beyond its configured rail, so a value past it did
    // not come from the accelerometer — but the only check was the saturation
    // FLAG, which would happily publish 196 m/s2 as a real 20 g reading with a
    // "this may be clipped" note attached. That is the corrupt-but-plausible
    // failure that retired the previous sensor, wearing a different hat: a
    // fabricated impact, on the field incident capture triggers from.
    //
    // The rail plus a margin, so genuine clipping at the limit still reads as
    // saturated rather than being thrown away.
    const float accelLimit = fusion ? IMU_ACCEL_MAX_VALID_MS2_FUSION
                                    : IMU_ACCEL_MAX_VALID_MS2_RAW;
    if (fabsf(ax) > accelLimit || fabsf(ay) > accelLimit || fabsf(az) > accelLimit){
        noteImplausible(dev);
        expireChannels(dev, data, now);
        publishPresence(dev, data);
        return dev.ready ? IMUReturnStatus::DATA_STALE : IMUReturnStatus::NOK_LINK_LOST;
    }
    // The gyroscope gets no equivalent check, deliberately: at +/-2000 dps and
    // 16 LSB/dps the rail is 32000 counts against an int16 maximum of 32767, so
    // there is no room for an impossible value to exist in the encoding. A test
    // that cannot fail is not a test.

    // The burst has passed every gate, so this is the moment the inertial record
    // gained a sample. The gap flag is measured from here — see
    // noteGapIfStarved() for why the poll's own timestamp will not do.
    dev.lastGoodMs = now;

    data.accelX = ax; data.accelY = ay; data.accelZ = az;
    data.accelSampleMs = now;
    data.accelValid    = true;

    data.gyroX = wx; data.gyroY = wy; data.gyroZ = wz;
    data.gyroSampleMs = now;
    data.gyroValid    = true;

    data.temperatureC = tempC;
    data.tempSampleMs = now;
    data.tempValid    = true;

    notePeakSq((ax * ax) + (ay * ay) + (az * az), dev.accelPeakSq, dev.peakBucketHead);
    notePeakSq((wx * wx) + (wy * wy) + (wz * wz), dev.gyroPeakSq,  dev.peakBucketHead);

    // Saturation is judged per AXIS, not on the magnitude. The rail is per-axis,
    // so a single clipped component makes the vector wrong even while its length
    // is nowhere near the limit — and a magnitude test would miss exactly that.
    //
    // AGAINST THE RAIL OF THE MODE IN USE. A single 4 g threshold was applied to
    // both, so AMG — selected at +/-16 g precisely so a severe impact can be
    // characterised instead of clipped — marked every honest reading above 4 g as
    // clipped, on the one mode that exists to measure past it.
    const float satLimit = fusion ? IMU_ACCEL_SATURATION_MS2_FUSION
                                  : IMU_ACCEL_SATURATION_MS2_RAW;
    if (fabsf(ax) >= satLimit || fabsf(ay) >= satLimit || fabsf(az) >= satLimit){
        dev.satBucket[dev.peakBucketHead] = true;
    }


    const uint8_t calib = buf[IMU_OFF_CALIB];
    data.calibMag   = static_cast<uint8_t>( calib       & 0x03u);
    data.calibAccel = static_cast<uint8_t>((calib >> 2) & 0x03u);
    data.calibGyro  = static_cast<uint8_t>((calib >> 4) & 0x03u);
    data.calibSys   = static_cast<uint8_t>((calib >> 6) & 0x03u);

    if (fusion && fusionIdle){
        // Raw channels above are already published; the fused ones simply are
        // not available yet. PARTIAL below says so.
        invalidateFusion(data);
        invalidateMagnetic(data);
    } else if (fusion){
        const float lx = static_cast<float>(toInt16LE(&buf[IMU_OFF_LINACC    ])) * IMU_SCALE_ACCEL_MS2;
        const float ly = static_cast<float>(toInt16LE(&buf[IMU_OFF_LINACC + 2])) * IMU_SCALE_ACCEL_MS2;
        const float lz = static_cast<float>(toInt16LE(&buf[IMU_OFF_LINACC + 4])) * IMU_SCALE_ACCEL_MS2;

        data.linAccelX = lx; data.linAccelY = ly; data.linAccelZ = lz;
        data.gravityX  = gx; data.gravityY  = gy; data.gravityZ  = gz;

        data.yawRelDeg = static_cast<float>(toInt16LE(&buf[IMU_OFF_EULER    ])) * IMU_SCALE_EULER_DEG;
        data.rollDeg   = static_cast<float>(toInt16LE(&buf[IMU_OFF_EULER + 2])) * IMU_SCALE_EULER_DEG;
        data.pitchDeg  = static_cast<float>(toInt16LE(&buf[IMU_OFF_EULER + 4])) * IMU_SCALE_EULER_DEG;

        notePeakSq((lx * lx) + (ly * ly) + (lz * lz), dev.linAccelPeakSq, dev.peakBucketHead);

        data.fusionSampleMs = now;
        data.fusionValid    = true;

        // The magnetometer is switched off in fusion mode, so its registers hold
        // zeros. Publishing those as a measurement would be a fabricated reading
        // — and zero is a perfectly plausible field strength, which is exactly
        // why it must not double as the missing-data sentinel.
        invalidateMagnetic(data);
    } else {
        const float mx = static_cast<float>(toInt16LE(&buf[IMU_OFF_MAG    ])) * IMU_SCALE_MAG_UT;
        const float my = static_cast<float>(toInt16LE(&buf[IMU_OFF_MAG + 2])) * IMU_SCALE_MAG_UT;
        const float mz = static_cast<float>(toInt16LE(&buf[IMU_OFF_MAG + 4])) * IMU_SCALE_MAG_UT;

        data.magX = mx; data.magY = my; data.magZ = mz;
        data.magSampleMs = now;
        data.magValid    = true;

        // No fusion in AMG, so there is nothing to publish and nothing to
        // pretend. The linear-acceleration peak is NAN rather than zero for the
        // same reason as above.
        invalidateFusion(data);
    }

    publishPeaks(dev, data, now);
    expireChannels(dev, data, now);
    publishPresence(dev, data);

    if (!dev.ready) return IMUReturnStatus::NOK_LINK_LOST;
    // A part whose fused output has not yet earned belief reports PARTIAL, and
    // the raw channels above are published regardless — they are unaffected by
    // calibration, and withholding good accelerometer data because the fusion is
    // still settling would be the wrong trade for a dashcam.
    if (fusion && (fusionIdle || !fusionTrustworthy(data))) return IMUReturnStatus::PARTIAL;
    return IMUReturnStatus::OK;
}

bool isIMULinkLost(const IMUDevice &dev){
    // Bring-up in progress is NOT a lost link. Reporting it as one restarts a
    // sequence that is merely part-way through the part's own 650 ms reset.
    if (dev.quarantined) return true;
    return !dev.ready && bno055InitReady(dev.init);
}

bool isIMUDegraded(const IMUDevice &dev){
    // A quarantined device is NOT reported as degraded, deliberately. Degraded
    // is what schedules recoverIMU(), and recovery transacts on a bus that has
    // just hung the board — so answering true here would defeat the quarantine a
    // few seconds after it was applied.
    if (dev.quarantined) return false;
    // Nor is a bring-up still running: the machine has its own retry backoff,
    // and a second one layered on top would restart it mid-sequence.
    if (dev.init.stage != BNO055InitStage::Configured &&
        dev.init.stage != BNO055InitStage::Failed) return false;
    return !dev.ready;
}

// ─── bus survey ───────────────────────────────────────────────────────────────

static inline bool isUsableI2CAddress(uint8_t address){
    return (address >= 0x08u) && (address <= 0x77u);
}

IMUReturnStatus checkI2CBusConflict(I2CBusReport &report){
    report.deviceCount   = 0u;
    report.imuPresent    = false;
    report.imuIdentified = false;
    report.imuAddress    = 0u;
    report.gpsPresent    = false;
    report.conflict      = false;
    for (uint8_t i = 0u; i < I2C_SCAN_MAX_DEVICES; i++) report.addresses[i] = 0u;

    if (i2cBusBegin() != I2CBusState::Ready) return IMUReturnStatus::NOK_BUS_STUCK;

    for (uint8_t addr = 0x08u; addr <= 0x77u; addr++){
        if (!isUsableI2CAddress(addr)) continue;
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() != 0u) continue;

        if (report.deviceCount < I2C_SCAN_MAX_DEVICES){
            report.addresses[report.deviceCount] = addr;
        }
        if (report.deviceCount < 0xFFu) report.deviceCount++;

        if (addr == GPS_DEFAULT_I2C_ADDRESS) report.gpsPresent = true;
        if (addr == BNO055_I2C_ADDRESS_DEFAULT || addr == BNO055_I2C_ADDRESS_ALT){
            report.imuPresent = true;
        }
    }

    // Identified by CHIP_ID, never by a bare ACK. Something else can sit at 0x28
    // or 0x29, and a responder that is not a BNO055 has to read as a CONFLICT
    // rather than as the IMU — otherwise bring-up proceeds to configure whatever
    // answered.
    const uint8_t found = bno055FindAddress();
    if (found != 0u){
        report.imuIdentified = true;
        report.imuAddress    = found;
    } else if (report.imuPresent){
        report.conflict = true;
    }

    if (report.conflict)       return IMUReturnStatus::NOK_ADDRESS_CONFLICT;
    if (report.imuIdentified)  return IMUReturnStatus::OK;
    if (report.deviceCount == 0u) return IMUReturnStatus::NOK_INIT_FAILED;
    return IMUReturnStatus::NOK_LINK_LOST;
}
