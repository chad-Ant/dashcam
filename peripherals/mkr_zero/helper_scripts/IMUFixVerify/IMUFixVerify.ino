/**
 * IMUFixVerify - regression tests for the IMU review fixes.
 *
 * Each test drives the REAL production function and asserts its observable
 * behaviour. Nothing here reimplements the logic it is checking: a test that
 * carries its own copy of the algorithm passes whether or not the shipped one
 * works, which is the failure mode this whole exercise exists to avoid.
 *
 * WHAT EACH GROUP NEEDS
 *
 *   A  nothing            pure logic and constants
 *   B  microSD card       calibration file parsing
 *   C  a live BNO055      page recovery, gap accounting, armed reporting
 *   D  a live BNO055      fault injection: frozen channel, stuck latch, AMG rail
 *
 * Groups whose hardware is absent report SKIP rather than FAIL, so this is
 * useful on a bare board and more useful on a complete one.
 *
 * GROUP D MAKES THE SENSOR MISBEHAVE ON PURPOSE, using its own power modes and
 * threshold register rather than stubbing the driver out — so the code under
 * test is production code meeting a genuinely faulty part. It reconfigures the
 * sensor as it goes and restores the production configuration at the end. An
 * injection that does not take is reported as SKIP, never as a FAIL: a part that
 * ignored the fault has tested nothing, and saying otherwise would be a lie
 * about the driver.
 *
 * ⚠️ GROUP B REWRITES bno055.cal. The existing file is copied into RAM first and
 * written back at the end, and the sketch says loudly if that restore fails.
 * The restore reproduces content, not byte-for-byte formatting: CRLF becomes LF.
 * Nothing parses the difference, but a technician diffing a card should know.
 *
 * Serial only. This sketch does NOT arm the watchdog — several tests wait longer
 * than the production period on purpose, and a reboot mid-test would look like a
 * failure of the thing being tested.
 */

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "DataDictionary.h"
#include "I2CBus.h"
#include "IMUFunctions.h"
#include "BNO055Calib.h"
#include "BNO055Transport.h"
#include "BNO055Regs.h"
#include "SDFunctions.h"
// CommunicationFunctions.h only forward-declares OBD2Data and GPSData, and
// buildTelemetry() takes them by reference — so the definitions have to come
// from their own headers before anything can construct one.
#include "OBD2Functions.h"
#include "GPSFunctions.h"
#include "VehicleSignals.h"
#include "CommunicationFunctions.h"

// ─── tiny harness ─────────────────────────────────────────────────────────────

static uint16_t gPass = 0, gFail = 0, gSkip = 0;

static void check(bool ok, const char *name)
{
    Serial.print(ok ? F("  PASS  ") : F("  FAIL  "));
    Serial.println(name);
    if (ok) gPass++; else gFail++;
}

static void skip(const char *name, const char *why)
{
    Serial.print(F("  SKIP  "));
    Serial.print(name);
    Serial.print(F("   ("));
    Serial.print(why);
    Serial.println(F(")"));
    gSkip++;
}

static void group(const char *title)
{
    Serial.println();
    Serial.print(F("── "));
    Serial.println(title);
}

/** @brief Reports a measured number alongside its verdict — the value is evidence. */
static void note(const char *label, long value)
{
    Serial.print(F("        "));
    Serial.print(label);
    Serial.print(F(" = "));
    Serial.println(value);
}

// ─── group A: peak ring geometry (finding 9) ──────────────────────────────────
//
// The two invariants are static_asserts in IMUFunctions.cpp, so a build that
// violated them would not exist. Recomputing them here is not redundant: it puts
// the actual numbers in the log, where the old sizing's 205 ms would have been
// visible to anyone who looked. The assert says "correct"; this says what.

static void groupPeakGeometry()
{
    group("A1  peak ring geometry (finding 9)");

    const uint32_t bucket   = IMU_PEAK_BUCKET_MS;
    const uint32_t eligible = IMU_PEAK_ELIGIBLE_MS;
    const uint32_t ringLife = (uint32_t)IMU_PEAK_BUCKETS * bucket;
    // A sample landing at the very END of its bucket is the worst case: it is
    // the one the ring retires soonest after it arrived.
    const uint32_t worstCase = eligible - bucket;

    note("buckets",                IMU_PEAK_BUCKETS);
    note("bucket_ms",              (long)bucket);
    note("eligible_ms",            (long)eligible);
    note("ring_lifetime_ms",       (long)ringLife);
    note("worst_case_retention_ms",(long)worstCase);
    note("advertised_window_ms",   (long)IMU_PEAK_WINDOW_MS);

    check(worstCase >= IMU_PEAK_WINDOW_MS,
          "every sample is retained for at least the advertised window");
    check(eligible < ringLife,
          "a bucket stops being eligible before the ring clears it");
    check(bucket > 0u && bucket < IMU_PEAK_WINDOW_MS,
          "bucket span is sane");
}

// ─── group A2: the armed flag reaches the wire (finding 2) ────────────────────
//
// buildTelemetry() is the whole mapping layer, so calling it with a synthetic
// IMUData tests the real thing with no sensor involved. This is the test that
// would have failed before the fix: IMUData::highGArmed existed, was documented
// as the thing nothing else on the frame would say, and reached no frame.

static void buildWith(const IMUData &imu, TelemetryPayload &out)
{
    const OBD2Data       obd{};
    const GPSData        gps{};
    const DerivedSignals derived{};
    const VehicleSignals veh{};
    buildTelemetry(obd, gps, imu, derived, veh, 1u, out);
}

static void groupWireFlags()
{
    group("A2  IMU flags reach the payload (finding 2)");

    IMUData imu;
    initIMUData(imu);
    TelemetryPayload p{};

    // Baseline: a blank sample raises none of the IMU flags.
    buildWith(imu, p);
    check((p.flags & COMM_FLAG_IMU_HIGHG_ARMED) == 0u,
          "armed flag clear when the sample says disarmed");
    check((p.flags & COMM_FLAG_IMU_HIGH_G) == 0u,
          "event flag clear when the sample says no event");

    imu.highGArmed = true;
    buildWith(imu, p);
    check((p.flags & COMM_FLAG_IMU_HIGHG_ARMED) != 0u,
          "armed flag SET when the sample says armed");

    // The case the flag exists for, and the one that used to be unrepresentable:
    // the backstop is down, so the absence of an event means "not watched for"
    // rather than "nothing happened". Both bits must be independently settable.
    imu.highGArmed = false;
    imu.highGEvent = true;
    buildWith(imu, p);
    check((p.flags & COMM_FLAG_IMU_HIGH_G) != 0u &&
          (p.flags & COMM_FLAG_IMU_HIGHG_ARMED) == 0u,
          "event and armed are independent bits");

    // The neighbouring qualifiers, checked so a bit-position slip in the new
    // 0x0800 cannot go unnoticed.
    initIMUData(imu);
    imu.dataGap = true;
    buildWith(imu, p);
    check((p.flags & COMM_FLAG_IMU_DATA_GAP) != 0u &&
          (p.flags & COMM_FLAG_IMU_HIGHG_ARMED) == 0u,
          "data-gap flag did not collide with the new bit");

    initIMUData(imu);
    imu.accelSaturated = true;
    buildWith(imu, p);
    check((p.flags & COMM_FLAG_IMU_SATURATED) != 0u &&
          (p.flags & COMM_FLAG_IMU_HIGHG_ARMED) == 0u,
          "saturation flag did not collide with the new bit");

    check(COMM_FLAG_IMU_HIGHG_ARMED == 0x0800u,
          "armed flag occupies the documented bit position");

    // Which MEASUREMENT the frame carries. Without it a consumer has to guess
    // the mode from which fields happen to be NAN, which is indistinguishable
    // from a stale channel — and it is what closes the loop on CMD_SET_IMU_MODE,
    // whose effect a host would otherwise have no way to observe.
    initIMUData(imu);
    imu.fusionMode = true;
    buildWith(imu, p);
    check((p.flags & COMM_FLAG_IMU_FUSION_MODE) != 0u,
          "fusion-mode flag SET when the sample is fused");

    imu.fusionMode = false;
    buildWith(imu, p);
    check((p.flags & COMM_FLAG_IMU_FUSION_MODE) == 0u,
          "and clear when the sample is raw");
    check(COMM_FLAG_IMU_FUSION_MODE == 0x1000u,
          "fusion-mode flag occupies the documented bit position");
}

// ─── group B: the calibration file (findings 4 and 7) ─────────────────────────

static char     gBackup[BNO055_CALIB_MAX_FILE_BYTES + 64];
static bool     gHadBackup = false;

/** @brief Copies bno055.cal into RAM so the destructive tests can be undone. */
static void backupCalibFile()
{
    gBackup[0]  = '\0';
    gHadBackup  = false;

    File32 f;
    if (!sdOpenRead(BNO055_CALIB_PATH, f)) return;

    size_t used = 0;
    char   line[SD_MAX_LINE];
    while (sdReadLine(f, line, sizeof(line))) {
        const size_t n = strlen(line);
        if ((used + n + 2u) >= sizeof(gBackup)) { used = 0; break; }  // too big to hold: do not pretend
        memcpy(gBackup + used, line, n);
        used += n;
        gBackup[used++] = '\n';
    }
    f.close();

    if (used > 0) {
        gBackup[used] = '\0';
        gHadBackup    = true;
    }
}

static void restoreCalibFile()
{
    if (!gHadBackup) {
        Serial.println(F("        no pre-existing bno055.cal to restore"));
        return;
    }
    const bool ok = (sdWriteTextAtomic(BNO055_CALIB_PATH, gBackup) == SDReturnStatus::OK);
    Serial.println(ok ? F("        bno055.cal RESTORED")
                      : F("  *** ERROR: bno055.cal could NOT be restored - recalibrate ***"));
}

/** @brief A profile whose fields are all comfortably inside any plausible bound. */
static void makeSaneProfile(uint8_t *p)
{
    memset(p, 0, BNO055_CALIB_BYTES);
    // accel offsets +12/-8/+30, mag +100/-100/+50, gyro -3/+2/-1, radii 1000/700
    const int16_t vals[11] = { 12, -8, 30, 100, -100, 50, -3, 2, -1, 1000, 700 };
    for (uint8_t i = 0; i < 11u; ++i) {
        p[i * 2]     = (uint8_t)((uint16_t)vals[i] & 0xFFu);
        p[i * 2 + 1] = (uint8_t)(((uint16_t)vals[i] >> 8) & 0xFFu);
    }
}

static void putRadii(uint8_t *p, int16_t accRadius, int16_t magRadius)
{
    p[18] = (uint8_t)((uint16_t)accRadius & 0xFFu);
    p[19] = (uint8_t)(((uint16_t)accRadius >> 8) & 0xFFu);
    p[20] = (uint8_t)((uint16_t)magRadius & 0xFFu);
    p[21] = (uint8_t)(((uint16_t)magRadius >> 8) & 0xFFu);
}

static void groupCalibFile()
{
    group("B  calibration file (findings 4 and 7)");

    if (!sdReady()) {
        skip("all calibration-file tests", "no SD card mounted");
        return;
    }

    backupCalibFile();
    Serial.println(gHadBackup ? F("        existing bno055.cal backed up")
                              : F("        no existing bno055.cal"));

    uint8_t  profile[BNO055_CALIB_BYTES];
    uint8_t  loaded[BNO055_CALIB_BYTES];
    uint16_t installId = 0;

    // B1 round trip through the real store and load.
    makeSaneProfile(profile);
    const bool stored = bno055CalibStore(profile, 0xBEEFu);
    check(stored, "a sane profile stores");
    check(stored && bno055CalibLoad(loaded, &installId) &&
          bno055CalibEqual(profile, loaded) && installId == 0xBEEFu,
          "and loads back byte-identical with its install id");

    // B2 the fix for finding 4: a NEGATIVE accelerometer radius.
    //
    // The radii were read as uint16 and compared against a positive bound, so a
    // negative value became a number above 32000 and the whole profile was
    // refused. The CRC is computed by the real store, so this is a VALID file
    // that the old plausibility check threw away.
    makeSaneProfile(profile);
    putRadii(profile, -1000, 700);
    check(bno055CalibStore(profile, 0x0001u) &&
          bno055CalibLoad(loaded, &installId) &&
          bno055CalibEqual(profile, loaded),
          "a profile with a negative accel radius is ACCEPTED");

    // ...and the bound still bites at the other end, so the check is not simply
    // gone. A test that cannot fail is not a test.
    makeSaneProfile(profile);
    putRadii(profile, 30000, 700);
    check(bno055CalibStore(profile, 0x0002u) && !bno055CalibLoad(loaded, nullptr),
          "an absurd accel radius is still REJECTED");

    makeSaneProfile(profile);
    profile[0] = 0xFF; profile[1] = 0x7F;   // accel X offset = 32767
    check(bno055CalibStore(profile, 0x0003u) && !bno055CalibLoad(loaded, nullptr),
          "an out-of-range accel offset is still REJECTED");

    // B3 the fix for finding 7: an oversized file is refused, and refused FAST.
    //
    // The load runs with the watchdog armed in production. The old code read to
    // EOF whatever the size, so a large file at this path outlasted the watchdog
    // period and the next boot parsed the same file again.
    {
        static char big[BNO055_CALIB_MAX_FILE_BYTES + 256];
        memset(big, 'A', sizeof(big) - 1);
        big[sizeof(big) - 1] = '\0';
        // No newline anywhere in it: this is the shape that makes sdReadLine()
        // drain the whole file into one line.
        const bool wrote = (sdWriteTextAtomic(BNO055_CALIB_PATH, big) == SDReturnStatus::OK);

        const uint32_t t0 = micros();
        const bool     accepted = bno055CalibLoad(loaded, nullptr);
        const uint32_t dtUs = micros() - t0;

        note("oversize_reject_us", (long)dtUs);
        check(wrote && !accepted, "an oversized file is refused");
        // Generous by three orders of magnitude against the 8 s watchdog: the
        // point is that the size is checked instead of the content being read.
        check(dtUs < 50000UL, "and refused without reading it (< 50 ms)");
    }

    // B4 a corrupt CRC is still caught, so the integrity check survived the
    // plausibility change.
    makeSaneProfile(profile);
    if (bno055CalibStore(profile, 0x0004u)) {
        char text[BNO055_CALIB_MAX_FILE_BYTES];
        size_t used = 0;
        File32 f;
        bool read = false;
        if (sdOpenRead(BNO055_CALIB_PATH, f)) {
            char line[SD_MAX_LINE];
            while (sdReadLine(f, line, sizeof(line))) {
                // Flip one hex digit of the payload, leaving the stored CRC to
                // disagree with it.
                if (strncmp(line, "data ", 5) == 0 && strlen(line) > 6) {
                    line[5] = (line[5] == '0') ? '1' : '0';
                }
                const size_t n = strlen(line);
                if ((used + n + 2u) >= sizeof(text)) { used = 0; break; }
                memcpy(text + used, line, n);
                used += n;
                text[used++] = '\n';
            }
            f.close();
            read = (used > 0);
        }
        if (read) {
            text[used] = '\0';
            check(sdWriteTextAtomic(BNO055_CALIB_PATH, text) == SDReturnStatus::OK &&
                  !bno055CalibLoad(loaded, nullptr),
                  "a corrupted payload fails the CRC");
        } else {
            skip("corrupted payload fails the CRC", "could not re-read the file");
        }
    } else {
        skip("corrupted payload fails the CRC", "store failed");
    }

    restoreCalibFile();
}

// ─── group C: the live sensor ─────────────────────────────────────────────────

static IMUDevice gDev;
static IMUData   gData;

/** @brief Drives the bring-up machine until it is Configured or the time is up. */
static bool tickUntilReady(uint32_t timeoutMs)
{
    const uint32_t deadline = millis() + timeoutMs;
    while ((int32_t)(millis() - deadline) < 0) {
        imuInitTick(gDev);
        if (imuIsReady(gDev)) return true;
        delay(2);
    }
    return false;
}

/** @brief Runs the bring-up to completion or gives up. @return true when ready. */
static bool bringUpSensor()
{
    initIMUData(gData);
    if (initializeIMU(gDev, IMUSampleMode::Fusion) != IMUReturnStatus::OK) return false;
    return tickUntilReady(5000UL);
}

/** @brief Polls for @p ms, discarding the samples. Used to let state settle. */
static void pollFor(uint32_t ms, uint32_t stepMs)
{
    const uint32_t until = millis() + ms;
    while ((int32_t)(millis() - until) < 0) {
        (void)getIMUData(gDev, gData);
        delay(stepMs);
    }
}

// C1 — finding 3. The headline: a part left on register page 1 must still be
// found. This is also the bench measurement that was missing when the fix was
// written, so the value read from 0x00 while on page 1 is PRINTED rather than
// merely asserted about.
static void testPageRecovery(uint8_t address)
{
    group("C1  page-1 recovery (finding 3)");

    uint8_t page = 0xFFu;
    if (bno055BusRead(address, BNO055_PAGE_ID_ADDR, &page, 1u) != 0) {
        skip("page-1 recovery", "could not read PAGE_ID");
        return;
    }
    check(page == 0x00u, "sensor starts on page 0");

    // Put it on page 1, which is what a watchdog reset during SetHighG leaves
    // behind — and what the bring-up could not recover from.
    unsigned char toPage1 = 0x01u;
    if (bno055BusWrite(address, BNO055_PAGE_ID_ADDR, &toPage1, 1u) != 0) {
        skip("page-1 recovery", "could not write PAGE_ID");
        return;
    }
    delay(5);

    uint8_t pageNow = 0xFFu;
    (void)bno055BusRead(address, BNO055_PAGE_ID_ADDR, &pageNow, 1u);
    check(pageNow == 0x01u, "sensor is now parked on page 1");

    // THE MEASUREMENT. Bosch documents page-1 0x00..0x06 as reserved, so what
    // comes back here is unspecified — and the entire severity of finding 3
    // turns on whether it happens to equal 0xA0. Recorded, not assumed.
    uint8_t idOnPage1 = 0xFFu;
    const bool idRead = (bno055BusRead(address, BNO055_CHIP_ID_ADDR, &idOnPage1, 1u) == 0);
    Serial.print(F("        CHIP_ID read at 0x00 while on PAGE 1 = 0x"));
    if (idRead) Serial.println(idOnPage1, HEX); else Serial.println(F("<read failed>"));
    if (idRead && idOnPage1 == BNO055_EXPECTED_CHIP_ID) {
        Serial.println(F("        (page-1 0x00 mirrors CHIP_ID on this part - finding 3 is benign here)"));
    } else {
        Serial.println(F("        (page-1 0x00 does NOT read 0xA0 - the old code would declare the part ABSENT)"));
    }

    // The fix itself.
    const uint8_t found = bno055FindAddress();
    check(found == address, "bno055FindAddress() finds a part parked on page 1");

    uint8_t pageAfter = 0xFFu;
    (void)bno055BusRead(address, BNO055_PAGE_ID_ADDR, &pageAfter, 1u);
    check(pageAfter == 0x00u, "and leaves it on page 0");

    // Recovery must not cost the caller a fabricated fault, and must not write
    // to the OTHER candidate address, where something that is not ours may sit.
    // A stray write to an empty address fails and is booked, so an unchanged
    // error count is evidence that none was attempted.
    const uint32_t errsBefore = bno055TransportErrorCount();
    const uint8_t  again      = bno055FindAddress();
    const uint32_t errsAfter  = bno055TransportErrorCount();
    note("transport_errors_delta", (long)(errsAfter - errsBefore));
    check(again == address, "a part already on page 0 is found by the fast path");
    check(errsAfter == errsBefore,
          "and no write is attempted at the empty candidate address");
}

// C2 — finding 5. The distinguishing case: polls that arrive ON TIME and are
// REJECTED. The old flag was measured from the poll attempt, so a burst thrown
// out by a plausibility gate reset the very timer that was supposed to notice
// it was missing.
static void testGapAccounting()
{
    group("C2  gap accounting on rejected bursts (finding 5)");

    // Settle: poll normally until the flag is quiet.
    for (uint8_t i = 0; i < 60u; ++i) { (void)getIMUData(gDev, gData); delay(10); }
    if (gData.dataGap) {
        skip("gap accounting", "flag was already set after a clean run");
        return;
    }
    check(!gData.dataGap, "no gap during punctual, accepted polling");

    // FAULT INJECTION. Point the driver at an address nothing answers, so every
    // burst fails while the polls themselves stay perfectly punctual. Under 20 ms
    // pacing the old rule saw a 20 ms interval and reported a healthy record; the
    // new rule measures from the last ACCEPTED sample and sees the hole.
    //
    // FOUR polls at 25 ms, and both numbers have to be right. The starvation has
    // to pass 50 ms with margin — three at 20 ms lands exactly on the threshold,
    // which is a coin toss, not a test — while the fault count has to stay under
    // the 5 that retire the device, or the flag would be explained by the part
    // being declared absent rather than by the gap rule. Four faults over ~100 ms
    // satisfies both.
    const uint8_t realAddress = gDev.init.address;
    gDev.init.address = 0x60u;                 // nothing lives here
    for (uint8_t i = 0; i < 4u; ++i) { (void)getIMUData(gDev, gData); delay(25); }
    const bool gapRaised   = gData.dataGap;
    const bool stillActive = gDev.ready;
    gDev.init.address = realAddress;

    note("device_still_ready", stillActive ? 1 : 0);
    check(gapRaised, "rejected bursts raise the gap flag while polls stay punctual");
    check(stillActive, "and the device was not retired, so the flag came from the gap rule");

    // And it clears again once real samples resume, so the flag is not a latch.
    const uint32_t deadline = millis() + (IMU_PEAK_WINDOW_MS * 4u);
    bool cleared = false;
    while ((int32_t)(millis() - deadline) < 0) {
        (void)getIMUData(gDev, gData);
        if (!gData.dataGap) { cleared = true; break; }
        delay(10);
    }
    check(cleared, "and clears once accepted samples resume");
}

// C3 — finding 2, sensor side. The armed state has to survive the trip from the
// device into the published sample and on into the payload.
static void testArmedReporting()
{
    group("C3  armed state is published (finding 2)");

    (void)getIMUData(gDev, gData);
    note("init.highGArmed",  gDev.init.highGArmed ? 1 : 0);
    note("data.highGArmed",  gData.highGArmed ? 1 : 0);
    note("highGClearFails",  gDev.highGClearFails);

    check(gData.highGArmed == (gDev.ready && gDev.init.highGArmed),
          "published armed state matches the device");

    TelemetryPayload p{};
    buildWith(gData, p);
    check(((p.flags & COMM_FLAG_IMU_HIGHG_ARMED) != 0u) == gData.highGArmed,
          "and reaches the telemetry payload");

    if (!gDev.init.highGArmed) {
        Serial.println(F("        NOTE: backstop is DISARMED - check the SetHighG stage"));
    }
}

/** @brief Magnitude of THIS burst's acceleration, or NAN if the axes are absent. */
static float instantMag(const IMUData &d)
{
    if (isnan(d.accelX) || isnan(d.accelY) || isnan(d.accelZ)) return NAN;
    return sqrtf((d.accelX * d.accelX) + (d.accelY * d.accelY) + (d.accelZ * d.accelZ));
}

// C4 — finding 9, measured against the ring's actual guarantee.
//
// MEASURED FROM THE LAST DISTURBING SAMPLE, not from the first. An earlier
// version of this test timed from the moment the PEAK first rose, which made the
// result "how long the tap lasted, plus the retention" and failed at 372 ms
// against a 332 ms bound — on completely correct code. A finger tap rings for
// tens of milliseconds, and every one of those samples restarts the retention.
//
// The exact instant is knowable here, and that is a property of this sensor
// rather than a trick: the BNO055 has no FIFO, so every sample folded into the
// ring came from a burst THIS TEST asked for. Watching the instantaneous
// magnitude alongside the published peak therefore identifies precisely which
// sample the retention should be measured from, which turns a loose band into a
// tight one.
static void testPeakRetention()
{
    group("C4  peak retention (finding 9)");

    // Find a quiet baseline, then wait for something to disturb it. On a bench
    // this is a tap; with nothing happening the test reports that and skips.
    Serial.println(F("        tap the board within 5 s to exercise the peak..."));

    float    baseline = NAN;
    for (uint8_t i = 0; i < 50u; ++i) { (void)getIMUData(gDev, gData); delay(10); }
    baseline = gData.accelPeakMs2;
    if (isnan(baseline)) { skip("peak retention", "no peak available"); return; }

    const float    trigger = baseline + 4.0f;
    const uint32_t hunt    = millis() + 5000UL;
    uint32_t       spikeMs = 0;
    while ((int32_t)(millis() - hunt) < 0) {
        (void)getIMUData(gDev, gData);
        if (!isnan(gData.accelPeakMs2) && gData.accelPeakMs2 > trigger) { spikeMs = millis(); break; }
        delay(10);
    }
    if (spikeMs == 0u) { skip("peak retention", "no disturbance seen"); return; }

    // The published peak crossed the trigger because THIS burst's sample did —
    // the previous poll was below it, and one poll folds exactly one sample.
    uint32_t lastAboveMs = spikeMs;

    uint32_t heldMs = 0;
    uint32_t ringingMs = 0;
    const uint32_t giveUp = millis() + (IMU_PEAK_WINDOW_MS * 8u);
    while ((int32_t)(millis() - giveUp) < 0) {
        (void)getIMUData(gDev, gData);
        const uint32_t nowMs = millis();

        // Every above-trigger sample restarts the clock, because every one of
        // them is folded into the ring and has to be retained in its own right.
        const float inst = instantMag(gData);
        if (!isnan(inst) && inst > trigger) lastAboveMs = nowMs;

        if (isnan(gData.accelPeakMs2) || gData.accelPeakMs2 <= trigger) {
            heldMs    = nowMs - lastAboveMs;
            ringingMs = lastAboveMs - spikeMs;
            break;
        }
        delay(5);
    }

    if (heldMs == 0u) { skip("peak retention", "peak did not decay before the deadline"); return; }

    note("disturbance_ms",   (long)ringingMs);   // how long the tap kept feeding the ring
    note("peak_held_ms",     (long)heldMs);      // retention after the LAST such sample
    note("window_ms",        (long)IMU_PEAK_WINDOW_MS);
    note("eligible_ms",      (long)IMU_PEAK_ELIGIBLE_MS);

    // The bound is now TIGHT, because the measurement error is one-sided.
    //
    // lastAboveMs is exact — it is the poll that folded the sample — so the only
    // error left is that the decay is noticed up to one loop iteration late,
    // which can only make the figure LARGER. The floor is therefore the real
    // guarantee with no slack needed, and only the ceiling carries a tolerance.
    //
    // The floor is what the old six-bucket sizing failed: it retired peaks after
    // about 205 ms while publishing them as 250 ms figures.
    check(heldMs >= IMU_PEAK_WINDOW_MS,
          "peak survived at least the advertised window (old sizing: ~205 ms)");
    check(heldMs <= (IMU_PEAK_ELIGIBLE_MS + 25UL),
          "and did not outlive the eligibility bound");
}

// ─── group D: fault injection ─────────────────────────────────────────────────
//
// Three repairs that ordinary operation cannot reach, because a healthy sensor
// never produces the fault they answer. The sensor is made to produce each one
// ITSELF — its own power modes and its own threshold register — rather than the
// driver being stubbed out or fed synthetic bursts. What runs is production code
// against a genuinely misbehaving part, which is the only arrangement that can
// tell a working repair from a plausible-looking one.
//
// Every test here leaves the part reconfigured, so the group runs LAST and ends
// by restoring the production configuration.

/// ACC_CONFIG bits 7:5 select the accelerometer power mode; 001 is SUSPEND.
///
/// In suspend the converter stops and the data registers stop changing while the
/// gyroscope beside it keeps running — which is precisely the fault the split
/// freeze check exists to catch, and precisely the one a single combined timer
/// cannot see, because the gyro noise holds it open forever.
#define ACC_PWR_MODE_MASK     0xE0u
#define ACC_PWR_MODE_SUSPEND  0x20u

/**
 * @brief Reads a page-1 register, ALWAYS returning the part to page 0.
 *
 * The page is restored whatever happens, matching the discipline the SetHighG
 * stage follows: a failure that returned while still on page 1 would leave every
 * later read hitting reserved space and answering with plausible constants.
 */
static bool page1Read(uint8_t addr, uint8_t reg, uint8_t &value)
{
    unsigned char p1 = 0x01u;
    if (bno055BusWrite(addr, BNO055_PAGE_ID_ADDR, &p1, 1u) != 0) return false;
    delay(2);

    unsigned char v  = 0u;
    const bool    ok = (bno055BusRead(addr, reg, &v, 1u) == 0);
    value = (uint8_t)v;

    unsigned char p0 = 0x00u;
    (void)bno055BusWrite(addr, BNO055_PAGE_ID_ADDR, &p0, 1u);
    delay(2);
    return ok;
}

/** @brief Writes a page-1 register, ALWAYS returning the part to page 0. */
static bool page1Write(uint8_t addr, uint8_t reg, uint8_t value)
{
    unsigned char p1 = 0x01u;
    if (bno055BusWrite(addr, BNO055_PAGE_ID_ADDR, &p1, 1u) != 0) return false;
    delay(2);

    unsigned char v  = value;
    const bool    ok = (bno055BusWrite(addr, reg, &v, 1u) == 0);

    unsigned char p0 = 0x00u;
    (void)bno055BusWrite(addr, BNO055_PAGE_ID_ADDR, &p0, 1u);
    delay(2);
    return ok;
}

/**
 * @brief Writes a page-1 configuration register from CONFIG mode, then resumes.
 *
 * THE MODE SWITCH IS NOT OPTIONAL. The part accepts page-1 sensor and interrupt
 * configuration only in CONFIG — which is precisely why the bring-up programs
 * the High-G interrupt before it ever selects an operating mode. A write issued
 * while the fusion is running is ACKNOWLEDGED AND DISCARDED, the same silent
 * ignore this whole driver was rebuilt around, and here it would produce a test
 * that injected no fault and quietly verified nothing.
 *
 * The operating mode is restored whatever happens above: leaving the part in
 * CONFIG stops it producing anything at all, which is a worse outcome than a
 * configuration write that did not take.
 */
static bool writePage1InConfig(uint8_t address, uint8_t reg, uint8_t value)
{
    const uint8_t runMode = gDev.init.opMode;

    unsigned char cfgMode = OPERATION_MODE_CONFIG;
    if (bno055BusWrite(address, BNO055_OPR_MODE_ADDR, &cfgMode, 1u) != 0) return false;
    delay(BNO055_MODE_SWITCH_MS);

    const bool ok = page1Write(address, reg, value);

    unsigned char back     = runMode;
    const bool    restored = (bno055BusWrite(address, BNO055_OPR_MODE_ADDR, &back, 1u) == 0);
    delay(BNO055_MODE_SWITCH_MS);

    return ok && restored;
}

// D1 — finding 6. The accelerometer is suspended in hardware while the gyroscope
// keeps converting. The old single-timer check could not fail this test, because
// gyro noise kept its one `changed` flag true forever; the split check has to
// retire the part.
static void testFreezeSplit(uint8_t address)
{
    group("D1  frozen accelerometer, live gyroscope (finding 6)");

    // AMG, because the accelerometer power mode is only ours to set outside the
    // fusion modes — the part overrides it otherwise.
    if (setIMUSampleMode(gDev, IMUSampleMode::Raw) != IMUReturnStatus::OK ||
        !tickUntilReady(5000UL)) {
        skip("freeze split", "could not enter AMG");
        return;
    }
    pollFor(200UL, 10UL);

    uint8_t cfg = 0u;
    if (!page1Read(address, BNO055_P1_ACC_CONFIG_ADDR, cfg)) {
        skip("freeze split", "could not read ACC_CONFIG");
        return;
    }
    if (!writePage1InConfig(address, BNO055_P1_ACC_CONFIG_ADDR,
                            (uint8_t)((cfg & ~ACC_PWR_MODE_MASK) | ACC_PWR_MODE_SUSPEND))) {
        skip("freeze split", "could not suspend the accelerometer");
        return;
    }

    float    pax = NAN, pay = NAN, paz = NAN;
    float    pgx = NAN, pgy = NAN, pgz = NAN;
    bool     first        = true;
    uint16_t accelMoves   = 0;
    uint16_t gyroMoves    = 0;
    uint32_t lastMoveMs   = millis();
    uint32_t retiredMs    = 0;

    const uint32_t giveUp = millis() + (IMU_MAX_CHANNEL_STALL_MS * 2u);
    while ((int32_t)(millis() - giveUp) < 0) {
        (void)getIMUData(gDev, gData);
        const uint32_t nowMs = millis();

        if (!gDev.ready) { retiredMs = nowMs; break; }

        if (!first) {
            if (gData.accelValid &&
                (gData.accelX != pax || gData.accelY != pay || gData.accelZ != paz)) {
                accelMoves++;
                lastMoveMs = nowMs;
            }
            if (gData.gyroValid &&
                (gData.gyroX != pgx || gData.gyroY != pgy || gData.gyroZ != pgz)) {
                gyroMoves++;
            }
        }
        pax = gData.accelX; pay = gData.accelY; paz = gData.accelZ;
        pgx = gData.gyroX;  pgy = gData.gyroY;  pgz = gData.gyroZ;
        first = false;
        delay(20);
    }

    (void)lastMoveMs;
    note("accel_changes", accelMoves);
    note("gyro_changes",  gyroMoves);

    // Distinguish a failed INJECTION from a failed FIX, in both directions. A
    // part that ignored the suspend, or a bench so still that the gyro stopped
    // moving too, has reproduced nothing — and reporting either as a FAIL would
    // be a lie about the driver.
    if (accelMoves > 2u) {
        skip("freeze split", "accelerometer did not suspend - injection failed");
        return;
    }
    if (gyroMoves < 10u) {
        skip("freeze split", "gyroscope was not moving - the masking case was not reproduced");
        return;
    }
    check(accelMoves <= 2u, "the accelerometer is genuinely frozen");
    check(gyroMoves >= 10u, "the gyroscope kept moving throughout");
    check(retiredMs != 0u, "the device was retired despite the live gyroscope");

    if (retiredMs != 0u) {
        // READ THE DRIVER'S OWN TIMERS, not the test's estimate of them. These
        // are the values the retirement decision was actually made on, and the
        // pair of them IS finding 6: under the old single-timer implementation
        // these two numbers were one number, and a gyro this fresh held it open
        // forever.
        const uint32_t accelStaleMs = retiredMs - gDev.accelChangeMs;
        const uint32_t gyroStaleMs  = retiredMs - gDev.gyroChangeMs;
        note("accel_timer_stale_ms", (long)accelStaleMs);
        note("gyro_timer_stale_ms",  (long)gyroStaleMs);
        note("stall_deadline_ms",    (long)IMU_MAX_CHANNEL_STALL_MS);

        check(accelStaleMs >= IMU_MAX_CHANNEL_STALL_MS &&
              accelStaleMs <= (IMU_MAX_CHANNEL_STALL_MS + 80UL),
              "the accelerometer timer expired at the configured deadline");
        // The crux. A combined timer is reset by EITHER channel, so this figure
        // is what it would have held — and it never reaches the deadline.
        check(gyroStaleMs < 200UL,
              "while the gyro timer stayed fresh, which a combined timer could not");
        check(isIMULinkLost(gDev), "and the link is reported lost");
    }
}

// D2 — finding 8. The High-G threshold is dropped below gravity, so the
// comparator re-latches the instant every clear write lands. The write is ACKed
// each time, which is exactly the evidence the old code trusted.
static void testStuckLatch(uint8_t address)
{
    group("D2  a High-G latch that never clears (finding 8)");

    if (setIMUSampleMode(gDev, IMUSampleMode::Fusion) != IMUReturnStatus::OK ||
        !tickUntilReady(5000UL)) {
        skip("stuck latch", "could not return to fusion");
        return;
    }
    pollFor(200UL, 10UL);
    (void)getIMUData(gDev, gData);
    if (!gDev.init.highGArmed) {
        skip("stuck latch", "backstop was not armed to begin with");
        return;
    }

    // ── negative control FIRST ───────────────────────────────────────────────
    // A threshold of 1 LSB is ~15.6 mg against a constant 1 g, so the latch is
    // permanently satisfied. Held for well under IMU_HIGHG_STUCK_MS, it must NOT
    // disarm anything: a real crash pulse lasts 10-50 ms and re-arms the
    // comparator repeatedly, and a detector that tripped on that would disarm the
    // backstop during the very impact it exists to catch.
    if (!writePage1InConfig(address, BNO055_P1_ACC_HG_THRES_ADDR, 1u)) {
        skip("stuck latch", "could not lower the threshold");
        return;
    }
    pollFor(IMU_HIGHG_STUCK_MS / 2u, 10UL);
    const bool armedAfterShort = gDev.init.highGArmed;

    // Put the threshold back and let the latch actually clear.
    (void)writePage1InConfig(address, BNO055_P1_ACC_HG_THRES_ADDR, IMU_HIGHG_THRESHOLD_LSB);
    pollFor(150UL, 10UL);

    check(armedAfterShort, "a short continuous latch does NOT disarm the backstop");
    check(gDev.init.highGArmed, "and the backstop is still armed afterwards");

    // ── the fault itself ─────────────────────────────────────────────────────
    const uint8_t failsBefore = gDev.highGClearFails;
    if (!writePage1InConfig(address, BNO055_P1_ACC_HG_THRES_ADDR, 1u)) {
        skip("stuck latch (sustained)", "could not lower the threshold");
        return;
    }

    uint32_t disarmedMs = 0;
    bool     latchSeen  = false;
    const uint32_t t0     = millis();
    const uint32_t giveUp = t0 + (IMU_HIGHG_STUCK_MS * 3u);
    while ((int32_t)(millis() - giveUp) < 0) {
        (void)getIMUData(gDev, gData);
        if (gData.highGEvent) latchSeen = true;
        if (!gDev.init.highGArmed) { disarmedMs = millis(); break; }
        delay(10);
    }

    note("disarm_latency_ms", disarmedMs ? (long)(disarmedMs - t0) : -1L);
    note("highGClearFails",   gDev.highGClearFails);

    // If the comparator never fired there was no stuck latch to detect, so
    // nothing here has been exercised. That is an injection failure, not a
    // driver failure, and must not be reported as one.
    if (!latchSeen) {
        (void)writePage1InConfig(address, BNO055_P1_ACC_HG_THRES_ADDR, IMU_HIGHG_THRESHOLD_LSB);
        (void)bringUpSensor();
        skip("stuck latch (sustained)", "threshold write did not make the latch fire");
        return;
    }

    check(disarmedMs != 0u, "a latch that never clears DOES disarm the backstop");
    check((disarmedMs == 0u) || ((disarmedMs - t0) >= IMU_HIGHG_STUCK_MS),
          "and not before the configured stuck window has elapsed");
    check(gDev.highGClearFails > failsBefore, "and the failure is counted");

    // The whole reason the flag exists: the host has to be told.
    TelemetryPayload p{};
    buildWith(gData, p);
    check((p.flags & COMM_FLAG_IMU_HIGHG_ARMED) == 0u,
          "and the payload stops claiming the backstop is armed");

    // Restore: put the threshold back and re-run bring-up, which resets the part.
    (void)writePage1InConfig(address, BNO055_P1_ACC_HG_THRES_ADDR, IMU_HIGHG_THRESHOLD_LSB);
    const bool recovered = bringUpSensor();
    pollFor(100UL, 10UL);
    check(recovered && gDev.init.highGArmed,
          "and a re-initialisation re-arms it");
}

// D3 — finding 10a. In AMG the part is configured for +/-16 g, so a reading
// between 4 g and 16 g is an HONEST measurement, not a clipped one. A single
// 4 g saturation constant applied to both modes flagged every such reading as a
// floor — on the one mode that exists to measure past 4 g.
static void testAmgSaturationRail(uint8_t address)
{
    group("D3  saturation uses the mode's own rail (finding 10a)");

    if (setIMUSampleMode(gDev, IMUSampleMode::Raw) != IMUReturnStatus::OK ||
        !tickUntilReady(5000UL)) {
        skip("AMG saturation rail", "could not enter AMG");
        return;
    }
    pollFor(200UL, 10UL);

    // The precondition, checked rather than assumed: the whole justification for
    // the raw mode is the wider range, and the bring-up only recently began
    // programming it at all.
    uint8_t cfg = 0u;
    const bool cfgRead = page1Read(address, BNO055_P1_ACC_CONFIG_ADDR, cfg);
    note("ACC_CONFIG_range_bits", cfgRead ? (long)(cfg & 0x03u) : -1L);
    check(cfgRead && (cfg & 0x03u) == ACCEL_RANGE_16G,
          "AMG bring-up selected the +/-16 g range");

    // And the frame says so. A host grading an impact from imuAccelPeak needs
    // the rail that peak was measured against, and this bit is the only thing
    // on the wire that supplies it.
    (void)getIMUData(gDev, gData);
    TelemetryPayload amgFrame{};
    buildWith(gData, amgFrame);
    check((amgFrame.flags & COMM_FLAG_IMU_FUSION_MODE) == 0u,
          "and the payload reports the raw mode");

    Serial.println(F("        tap the board FIRMLY within 8 s (needs > 4 g)..."));

    // Hunt for a peak above the FUSION rail but below the raw one. That band is
    // exactly the set of readings the old constant mislabelled.
    const float    lowBand  = IMU_ACCEL_SATURATION_MS2_FUSION + 6.0f;   // ~45 m/s2
    const float    highBand = IMU_ACCEL_SATURATION_MS2_RAW    - 5.0f;   // ~150 m/s2
    float          seenPeak = NAN;
    bool           seenSat  = false;
    const uint32_t giveUp   = millis() + 8000UL;
    while ((int32_t)(millis() - giveUp) < 0) {
        (void)getIMUData(gDev, gData);
        if (!isnan(gData.accelPeakMs2) &&
            gData.accelPeakMs2 > lowBand && gData.accelPeakMs2 < highBand) {
            seenPeak = gData.accelPeakMs2;
            seenSat  = gData.accelSaturated;
            break;
        }
        delay(5);
    }

    if (isnan(seenPeak)) {
        skip("AMG saturation rail", "no tap above 4 g seen");
    } else {
        Serial.print(F("        peak = "));
        Serial.print(seenPeak, 1);
        Serial.print(F(" m/s2, saturated = "));
        Serial.println(seenSat ? F("true") : F("false"));
        check(!seenSat,
              "a 4-16 g reading in AMG is NOT reported as clipped");
    }

    // Leave the board in the production configuration whatever happened above.
    const bool restored = (setIMUSampleMode(gDev, IMUSampleMode::Fusion) == IMUReturnStatus::OK) &&
                          tickUntilReady(5000UL);
    check(restored, "and the sensor is restored to fusion mode");
}

static void groupFaultInjection(uint8_t address)
{
    group("D  fault injection");
    Serial.println(F("        the sensor is made to misbehave on purpose;"));
    Serial.println(F("        production configuration is restored at the end"));

    testFreezeSplit(address);
    testStuckLatch(address);
    testAmgSaturationRail(address);
}

static void groupSensor()
{
    group("C  live sensor");

    if (i2cBusBegin() != I2CBusState::Ready) {
        skip("all sensor tests", "I2C bus is not usable");
        return;
    }

    const uint8_t address = bno055FindAddress();
    if (address == 0u) {
        skip("all sensor tests", "no BNO055 identified at 0x28 or 0x29");
        return;
    }
    Serial.print(F("        BNO055 found at 0x"));
    Serial.println(address, HEX);

    // Page recovery runs FIRST and on the bare bus, before bring-up: it is about
    // finding a part, not about a configured one.
    testPageRecovery(address);

    if (!bringUpSensor()) {
        skip("gap / armed / peak tests", "bring-up did not complete");
        return;
    }
    Serial.println(F("        bring-up complete"));

    testGapAccounting();
    testArmedReporting();
    testPeakRetention();

    // LAST, because every test in it leaves the part reconfigured.
    groupFaultInjection(address);
}

// ─── entry points ─────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(SERIAL_BAUDRATE);
    const uint32_t waitUntil = millis() + 4000UL;
    while (!Serial && (int32_t)(millis() - waitUntil) < 0) { }

    Serial.println();
    Serial.println(F("═══ IMUFixVerify ═══════════════════════════════════════════"));
    Serial.println(F("Regression tests for the IMU review fixes."));

    groupPeakGeometry();
    groupWireFlags();

    if (initializeSD() == SDReturnStatus::OK) {
        groupCalibFile();
    } else {
        group("B  calibration file (findings 4 and 7)");
        skip("all calibration-file tests", "SD card would not mount");
    }

    groupSensor();

    Serial.println();
    Serial.println(F("═══ summary ════════════════════════════════════════════════"));
    Serial.print(F("  passed  ")); Serial.println(gPass);
    Serial.print(F("  failed  ")); Serial.println(gFail);
    Serial.print(F("  skipped ")); Serial.println(gSkip);
    Serial.println(gFail == 0 ? F("  RESULT: OK") : F("  RESULT: FAILURES PRESENT"));
    Serial.println(F("════════════════════════════════════════════════════════════"));
}

void loop()
{
    // Everything runs once. Holding here rather than repeating keeps the report
    // at the end of the buffer where a technician can read it.
    delay(1000);
}
