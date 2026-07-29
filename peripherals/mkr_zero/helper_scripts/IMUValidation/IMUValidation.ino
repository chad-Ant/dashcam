/**
 * IMUValidation — bring-up and coexistence test for the Adafruit LSM6DSOX +
 * LIS3MDL 9-DoF breakout on the MKR Zero's shared I2C bus.
 *
 * It answers three questions in order, and stops being interesting only when
 * all three are boring:
 *
 *   1. WHAT IS ON THE BUS?  Full address sweep, every responder named against
 *      the project's address map (DataDictionary.h).
 *   2. IS THERE A CONFLICT WITH THE GPS?  Both the GNSS receiver and the IMU
 *      are brought up on the same Wire, then polled together at their real
 *      production rates (GPS 4 Hz, IMU 20 Hz) while every failed transaction is
 *      counted.  An address clash shows up in step 1; the subtler failures —
 *      one device wedging the bus, pull-ups too weak for three boards, a driver
 *      resetting the clock underneath another — only show up here, under load.
 *   3. ARE THE READINGS PLAUSIBLE?  Live values plus a stationary sanity check:
 *      the accelerometer vector should measure 1 g, the gyro should read zero,
 *      and the magnetometer should see roughly Earth's field.
 *
 * Wiring (Adafruit STEMMA QT breakout, PID 4517):
 *   MKR Zero VCC (3.3 V) -> breakout VIN      <-- NOT 5 V.  The breakout's level
 *   MKR Zero GND         -> breakout GND          shifters reference VIN, so a
 *   MKR Zero D11 / SDA   -> breakout SDA          5 V supply puts 5 V on SDA and
 *   MKR Zero D12 / SCL   -> breakout SCL          SCL, and no MKR Zero I/O pin
 *                                                 is 5 V tolerant.
 *
 * Comment out USE_GPS below to test the IMU on its own.
 */

#define USE_GPS 1

#include "DataDictionary.h"
#include "TimerFunctions.h"
#include "IMUFunctions.h"
#ifdef USE_GPS
#include "GPSFunctions.h"
#endif

static constexpr unsigned long SERIAL_READY_TIMEOUT_MS = 2000;
// GPS_POLL_MS comes from DataDictionary.h so this test polls at exactly the
// production cadence — a soak that used a different interval would not be
// measuring the thing that ships.
static constexpr unsigned long REPORT_MS               = 1000;
static constexpr unsigned long IMU_RETRY_INTERVAL_MS   = IMU_RETRY_MS;

/// Stationary sanity limits.  Loose on purpose: this is a "the sensor is wired
/// up and pointing at reality" check, not a calibration.
static constexpr float GRAVITY_MIN_MS2   = 9.0f;   ///< |a| at rest, lower bound.
static constexpr float GRAVITY_MAX_MS2   = 10.6f;  ///< |a| at rest, upper bound.
static constexpr float GYRO_REST_MAX_DPS = 5.0f;   ///< Bias-dominated rate at rest.
static constexpr float MAG_MIN_UT        = 20.0f;  ///< Earth's field is 25-65 uT;
static constexpr float MAG_MAX_UT        = 120.0f; ///< car ironwork widens both ends.

IMUDevice imu;
IMUData   imuData;

static unsigned long lastIMUPoll   = 0;
static unsigned long lastIMURetry  = 0;
static unsigned long lastReport    = 0;

/// Counters, not just flags: a bus problem that bites once an hour is invisible
/// in a live readout and obvious in a total.
///
/// There is no local "imuReady" mirror of the library's state — the IMUDevice
/// ready flags are the single source of truth.  A second copy in the sketch is
/// exactly how a partially-failed sensor gets stranded: the mirror says "fine"
/// while the library knows better.
static uint32_t imuPolls        = 0;
static uint32_t imuFreshSamples = 0;  ///< At least one device returned new data.
static uint32_t imuStalePolls   = 0;  ///< Polled faster than the output data rate.
static uint32_t imuDegraded     = 0;  ///< Polls taken with one device down.
static uint32_t imuLost         = 0;  ///< Polls taken with both devices down.
static uint32_t imuErrors       = 0;  ///< Unexpected status codes.
static uint32_t imuReinits      = 0;
static uint32_t imuBusStuck     = 0;  ///< Recoveries that failed to free the lines.
static uint32_t maxIMUPollUs    = 0;

#ifdef USE_GPS
SFE_UBLOX_GNSS myGNSS;
GPSData        gpsData;
static bool          gpsReady     = false;
static unsigned long lastGPSPoll  = 0;
static unsigned long lastGPSRetry = 0;
static unsigned long lastGPSFresh = 0;
static uint32_t gpsPolls     = 0;
static uint32_t gpsFresh     = 0;   ///< Fresh PVT packet WITH a valid fix.
/// Fresh PVT packet WITHOUT a fix.  Counted separately from gpsStale on
/// purpose: both mean "no position", but this one proves the receiver is
/// talking to us over I2C, which is the only thing this test can conclude
/// indoors.  Folding them together turns "no sky view" into a false bus fault.
static uint32_t gpsNoFix     = 0;
/// Polls that found no packet buffered.
///
/// EXPECTED to be roughly 20 % of polls, and that is not a fault: the host polls
/// at 5 Hz (GPS_POLL_MS 200) against a 4 Hz receiver precisely so it can never
/// miss a packet, which arithmetically means one poll in five finds nothing new.
/// Judge the link by the packet RATE below, not by this counter — reading a
/// high stale count as a problem is how the previous 250 ms cadence looked
/// healthy while quietly running a backlog.
static uint32_t gpsStale     = 0;
static uint32_t gpsOther     = 0;   ///< Any other GPS return status.
static uint32_t maxGPSPollUs = 0;
static uint32_t gpsProbeOk   = 0;   ///< 1 Hz address probes that ACKed.
static uint32_t gpsProbeFail = 0;   ///< 1 Hz address probes that did not.
/// @c millis() when GNSS polling actually began.
///
/// The packet rate must be measured from here, not from boot: setup() spends
/// several seconds on the bus census and three device bring-ups, and counting
/// that dead time in the denominator makes a perfectly healthy link read 3.69
/// against a target of 4 — a false alarm in the very metric added to prevent
/// false alarms.
static uint32_t gpsStartMs   = 0;
/// Raw GPSReturnStatus from the last init attempt.  Kept as an int because the
/// interesting information is WHICH step refused: -1 no response at all,
/// -2 rate rejected, -6 configuration rejected.  "GPS: module failed" on its
/// own cannot distinguish a missing receiver from a rejected setting.
static int gpsInitStatus = 99;

/** @brief (Re)runs the GNSS I2C bring-up and records the exact status. */
static bool startGPS()
{
    const GPSReturnStatus st = initializeGPS_I2C(myGNSS);
    gpsInitStatus = static_cast<int>(st);
    return (st == GPSReturnStatus::OK);
}
#endif

// ─── reporting helpers ────────────────────────────────────────────────────────

/** @brief Names a scanned address against the project's I2C allocation. */
static const char *describeAddress(uint8_t address)
{
    switch (address) {
        case IMU_ACCEL_I2C_ADDRESS:     return "LSM6DSOX accel/gyro";
        case IMU_ACCEL_I2C_ADDRESS_ALT: return "LSM6DSOX (jumper closed)";
        case IMU_MAG_I2C_ADDRESS:       return "LIS3MDL magnetometer";
        case IMU_MAG_I2C_ADDRESS_ALT:   return "LIS3MDL (jumper closed)";
        case GPS_DEFAULT_I2C_ADDRESS:   return "u-blox GNSS receiver";
        case SEGLED_ADDRESS:            return "segment LED backpack";
        case 0x60:                      return "ATECC508A (MKR Zero onboard)";
        default:                        return "UNEXPECTED - not in DataDictionary.h";
    }
}

static void printHex8(uint8_t value)
{
    Serial.print("0x");
    if (value < 0x10) Serial.print("0");
    Serial.print(value, HEX);
}

/** @brief Prints a float, or "--" when it is NAN. */
static void printFloatOrDash(float value, uint8_t decimals)
{
    if (isnan(value)) Serial.print("--");
    else              Serial.print(value, decimals);
}

/** @brief Step 1 and 2: what is on the bus, and does anything clash. */
static void reportBusCensus()
{
    Serial.println();
    Serial.println("--- I2C bus census ---");

    I2CBusReport report;
    const IMUReturnStatus st = checkI2CBusConflict(report);

    Serial.print("devices found: ");
    Serial.println(report.deviceCount);

    const uint8_t shown = (report.deviceCount < I2C_SCAN_MAX_DEVICES)
                              ? report.deviceCount : I2C_SCAN_MAX_DEVICES;
    for (uint8_t i = 0; i < shown; i++) {
        Serial.print("  ");
        printHex8(report.addresses[i]);
        Serial.print("  ");
        Serial.println(describeAddress(report.addresses[i]));
    }
    if (report.deviceCount > shown) {
        Serial.print("  (");
        Serial.print(report.deviceCount - shown);
        Serial.println(" more not recorded)");
    }

    Serial.print("LSM6DSOX @ ");
    printHex8(IMU_ACCEL_I2C_ADDRESS);
    Serial.println(report.accelIdentified ? ": present, WHO_AM_I ok"
                                          : (report.accelPresent ? ": PRESENT BUT WRONG WHO_AM_I"
                                                                 : ": absent"));
    Serial.print("LIS3MDL  @ ");
    printHex8(IMU_MAG_I2C_ADDRESS);
    Serial.println(report.magIdentified ? ": present, WHO_AM_I ok"
                                        : (report.magPresent ? ": PRESENT BUT WRONG WHO_AM_I"
                                                             : ": absent"));
    Serial.print("u-blox   @ ");
    printHex8(GPS_DEFAULT_I2C_ADDRESS);
    Serial.println(report.gpsPresent ? ": present" : ": absent");

    Serial.print("verdict: ");
    if (st == IMUReturnStatus::NOK_BUS_STUCK) {
        // Its own message on purpose. A stuck bus and an empty bus look
        // identical in a device count, but they are opposite repairs: one is a
        // line held low, the other is a missing connection.
        Serial.println("BUS STUCK - a line is held low; scan impossible. NOT an empty bus:");
        Serial.println("         check for a shorted SDA/SCL or a slave wedged mid-transaction.");
    } else if (report.conflict) {
        Serial.println("ADDRESS CONFLICT - an IMU address is held by another device");
    } else if (st == IMUReturnStatus::OK) {
        Serial.println("no conflict, both IMU parts identified");
    } else if (st == IMUReturnStatus::NOK_INIT_FAILED) {
        Serial.println("bus empty - check wiring, power and pull-ups");
    } else {
        Serial.println("no conflict, but a device is missing (see above)");
    }
    Serial.println();
}

/** @brief Step 3: does a stationary board read like a stationary board. */
static void reportSanityCheck()
{
    if (imuData.accelValid) {
        const float magnitude = sqrtf(imuData.accelX * imuData.accelX +
                                      imuData.accelY * imuData.accelY +
                                      imuData.accelZ * imuData.accelZ);
        Serial.print("  |a|=");
        Serial.print(magnitude, 2);
        Serial.print(" m/s2 ");
        Serial.println((magnitude >= GRAVITY_MIN_MS2 && magnitude <= GRAVITY_MAX_MS2)
                           ? "[ok, 1 g]" : "[OUT OF RANGE at rest]");
    }

    if (imuData.gyroValid) {
        const float worst = fmaxf(fmaxf(fabsf(imuData.gyroX), fabsf(imuData.gyroY)),
                                  fabsf(imuData.gyroZ));
        Serial.print("  gyro max=");
        Serial.print(worst, 2);
        Serial.print(" deg/s ");
        Serial.println((worst <= GYRO_REST_MAX_DPS) ? "[ok at rest]" : "[MOVING or biased]");
    }

    if (imuData.magValid) {
        const float field = sqrtf(imuData.magX * imuData.magX +
                                  imuData.magY * imuData.magY +
                                  imuData.magZ * imuData.magZ);
        Serial.print("  |B|=");
        Serial.print(field, 1);
        Serial.print(" uT ");
        Serial.println((field >= MAG_MIN_UT && field <= MAG_MAX_UT)
                           ? "[ok, Earth field]" : "[OUT OF RANGE - magnet or interference?]");
    }
}

static void reportLive()
{
    Serial.println("--- live ---");

    Serial.print("  accel  x=");
    printFloatOrDash(imuData.accelX, 2);
    Serial.print(" y=");
    printFloatOrDash(imuData.accelY, 2);
    Serial.print(" z=");
    printFloatOrDash(imuData.accelZ, 2);
    Serial.println(" m/s2");

    Serial.print("  gyro   x=");
    printFloatOrDash(imuData.gyroX, 2);
    Serial.print(" y=");
    printFloatOrDash(imuData.gyroY, 2);
    Serial.print(" z=");
    printFloatOrDash(imuData.gyroZ, 2);
    Serial.println(" deg/s");

    Serial.print("  mag    x=");
    printFloatOrDash(imuData.magX, 1);
    Serial.print(" y=");
    printFloatOrDash(imuData.magY, 1);
    Serial.print(" z=");
    printFloatOrDash(imuData.magZ, 1);
    Serial.print(" uT   temp=");
    printFloatOrDash(imuData.temperatureC, 1);
    Serial.println(" C");

    reportSanityCheck();

    Serial.print("  imu    polls=");
    Serial.print(imuPolls);
    Serial.print(" fresh=");
    Serial.print(imuFreshSamples);
    Serial.print(" stale=");
    Serial.print(imuStalePolls);
    Serial.print(" degraded=");
    Serial.print(imuDegraded);
    Serial.print(" lost=");
    Serial.print(imuLost);
    Serial.print(" errors=");
    Serial.print(imuErrors);
    // The counter that can actually answer "did any read fail?".  `errors=`
    // above counts unexpected RETURN CODES, and getIMUData() returns OK whenever
    // either device supplied a fresh sample — so a device NACKing every single
    // poll while its sibling works leaves errors= at zero for the whole run.
    // A previous run reported errors=0 over 30,259 samples; that was evidence
    // about status codes, not about the bus, and this pair is what makes the
    // stronger claim checkable.
    Serial.print(" io=");
    Serial.print(imu.accelIOErrors);
    Serial.print("/");
    Serial.print(imu.magIOErrors);
    Serial.print(" reinits=");
    Serial.print(imuReinits);
    Serial.print(" worst=");
    Serial.print(maxIMUPollUs);
    Serial.println(" us");

    Serial.print("  imu    accel=");
    Serial.print(imu.accelReady ? "up" : "DOWN");
    Serial.print(" mag=");
    Serial.print(imu.magReady ? "up" : "DOWN");
    Serial.print("  busStuck=");
    Serial.print(imuBusStuck);
    Serial.print(" busState=");
    Serial.println(static_cast<int>(i2cBusState()));

#ifdef USE_GPS
    // One address probe per report.  This is the measurement that separates
    // "receiver is not on the bus" from "receiver is on the bus but has no sky
    // view" — the packet counters alone cannot tell those apart indoors.
    if (i2cProbeAddress(GPS_DEFAULT_I2C_ADDRESS)) gpsProbeOk++;
    else                                          gpsProbeFail++;

    Serial.print("  gps    polls=");
    Serial.print(gpsPolls);
    Serial.print(" fix=");
    Serial.print(gpsFresh);
    Serial.print(" nofix=");
    Serial.print(gpsNoFix);
    Serial.print(" stale=");
    Serial.print(gpsStale);
    Serial.print(" other=");
    Serial.print(gpsOther);
    Serial.print(" sats=");
    Serial.print(gpsData.satellites);
    Serial.print(" worst=");
    Serial.print(maxGPSPollUs);
    Serial.println(" us");

    // Packets per second, x100 so it prints without float formatting. This is
    // the honest health metric for the link: it should sit at the receiver's
    // navigation rate (GPS_REFRESH_RATE) regardless of how often we poll.
    const uint32_t elapsedMs = millis() - gpsStartMs;
    const uint32_t packets   = gpsFresh + gpsNoFix;
    const uint32_t rateX100  = (elapsedMs > 0UL) ? ((packets * 100000UL) / elapsedMs) : 0UL;

    Serial.print("  gps    rate=");
    Serial.print(rateX100 / 100UL);
    Serial.print(".");
    if ((rateX100 % 100UL) < 10UL) Serial.print("0");
    Serial.print(rateX100 % 100UL);
    Serial.print(" pkt/s (want ");
    Serial.print(GPS_REFRESH_RATE);
    Serial.print(")  ack=");
    Serial.print(gpsProbeOk);
    Serial.print("/");
    Serial.print(gpsProbeOk + gpsProbeFail);
    Serial.print("  initStatus=");
    Serial.println(gpsInitStatus);

    // The coexistence verdict.  What matters is whether each device still
    // ANSWERS while the other is hammering the bus — not whether the GNSS has a
    // fix, which depends on the sky, not on I2C.
    Serial.print("  coexistence: ");
    if (!gpsReady) {
        Serial.println("GPS did not initialise - IMU-only run");
    } else if (gpsProbeFail > 0) {
        Serial.println("GPS STOPPED ACKING - real bus fault, check wiring/pull-ups");
    } else if (imuErrors > 0 || imu.accelIOErrors > 0 || imu.magIOErrors > 0) {
        Serial.println("IMU bus errors seen - check pull-ups and wiring");
    } else if ((gpsFresh + gpsNoFix) == 0 && gpsPolls > 40) {
        Serial.println("GPS acks but sends no PVT - receiver configured but not streaming");
    } else if (gpsFresh == 0) {
        Serial.println("bus OK both ways; GNSS has no fix yet (indoors?)");
    } else {
        Serial.println("bus OK both ways, GNSS has a fix");
    }
#endif
    Serial.println();
}

// ─── Arduino entry points ─────────────────────────────────────────────────────

void setup()
{
    pinMode(STATUS_INDICATOR, OUTPUT);
    digitalWrite(STATUS_INDICATOR, HIGH);

    Serial.begin(SERIAL_BAUDRATE);
    // Bounded: this sketch must still run when it is powered from the car
    // rather than a laptop, with nothing listening on USB.
    const unsigned long serialWaitStart = millis();
    while (!Serial && !isTimeout(SERIAL_READY_TIMEOUT_MS, serialWaitStart)) {
        // spin briefly; the timeout is the guarantee
    }
    Serial.println("MKR Zero IMU validation (LSM6DSOX + LIS3MDL)");

    // Armed before the first I2C transaction, exactly as mkr_zero.ino does.
    // Without it this sketch could not observe the one failure that matters
    // most — a bring-up or a poll that hangs long enough to reset the board —
    // because there was nothing to reset it, so a hang here simply stopped, and
    // "no output after line 3" is not a result anyone can act on.
    watchdogArm(WATCHDOG_PERIOD_MS);
    if (watchdogCausedReset()) Serial.println("BOOT: previous run was ended by the watchdog");

    initIMUData(imuData);

    // GNSS FIRST, then the IMU, and the census AFTER BOTH — matching
    // mkr_zero.ino.  The census used to run first, on the reasoning that it
    // catches the bus in its power-on state.  That reasoning is sound and the
    // placement was still wrong: scanI2CBus() enters through i2cBusBegin(), so
    // running it first made the CENSUS the first I2C client and performed the
    // one-time bus recovery on its behalf.  The property under test is that the
    // GNSS receiver — the first client in production — recovers a wedged bus for
    // everyone else, and a test that quietly recovers the bus before GNSS ever
    // runs cannot fail when that property is broken.
#ifdef USE_GPS
    // I2C only.  initializeGPS() would seize Serial1, which belongs to the
    // ESP32-C3 link in the production firmware.
    initGPSData(gpsData);
    watchdogFeed();
    (void)preallocateGPS_I2C(myGNSS);
    gpsReady = startGPS();
    watchdogFeed();
    Serial.print(gpsReady ? "GPS: module started on I2C" : "GPS: module FAILED");
    Serial.print("  (status ");
    Serial.print(gpsInitStatus);
    Serial.println(")");
    lastGPSPoll  = millis();
    lastGPSRetry = lastGPSPoll;
    gpsStartMs   = lastGPSPoll;
    // Seeded, not left at its zero initialiser.  sinceFresh is computed as
    // millis() - lastGPSFresh, so a zero here reads as "silent since boot" and
    // the silence teardown fires on the very first pass — tearing down the
    // receiver that had just come up, and reporting a fault the retry path then
    // "recovered" from. The production sketch seeds it for the same reason.
    lastGPSFresh = lastGPSPoll;
#endif

    const IMUReturnStatus st = initializeIMU(imu);
    switch (st) {
        case IMUReturnStatus::OK:
            Serial.println("IMU: both devices up");
            break;
        case IMUReturnStatus::PARTIAL:
            // Run with whatever answered — but the degraded-recovery timer in
            // loop() keeps retrying the missing half, so this is not a
            // permanent state.
            Serial.print("IMU: PARTIAL - accel=");
            Serial.print(imu.accelReady ? "up" : "down");
            Serial.print(" mag=");
            Serial.println(imu.magReady ? "up" : "down");
            break;
        case IMUReturnStatus::NOK_ADDRESS_CONFLICT:
            Serial.println("IMU: ADDRESS CONFLICT - see census above");
            break;
        case IMUReturnStatus::NOK_BUS_STUCK:
            Serial.println("IMU: bus stuck low - a slave is holding SDA");
            break;
        default:
            Serial.println("IMU: init failed - no device answered");
            break;
    }

    lastIMUPoll  = millis();
    lastIMURetry = millis();
    lastReport   = millis();

    // Census LAST — see the note above.  By now GNSS and the IMU have both been
    // through i2cBusBegin(), so this reports what is answering on a bus that
    // production has already brought up, which is the state the rest of the run
    // is measured in.
    reportBusCensus();
    watchdogFeed();
}

void loop()
{
    watchdogFeed();
    // ── IMU poll ─────────────────────────────────────────────────────────────
    // Unconditional on the timer: getIMUData() returns immediately without
    // touching the bus when a device is down, so there is nothing to gate on.
    if (isTimeout(IMU_POLL_MS, lastIMUPoll)) {
        lastIMUPoll = millis();

        const uint32_t t0 = micros();
        const IMUReturnStatus st = getIMUData(imu, imuData);
        const uint32_t elapsed = micros() - t0;
        if (elapsed > maxIMUPollUs) maxIMUPollUs = elapsed;

        imuPolls++;
        switch (st) {
            case IMUReturnStatus::OK:            imuFreshSamples++; break;
            case IMUReturnStatus::DATA_STALE:    imuStalePolls++;   break;
            case IMUReturnStatus::PARTIAL:       imuDegraded++;     break;
            case IMUReturnStatus::NOK_LINK_LOST: imuLost++;         break;
            default:                             imuErrors++;       break;
        }
    }

    // ── bounded recovery, driven by isIMUDegraded() not isIMULinkLost() ──────
    // A PARTIAL start-up (say the magnetometer absent while the accelerometer
    // is fine) is a state "both devices lost" never becomes, so gating recovery
    // on isIMULinkLost() would strand the missing sensor offline for the whole
    // trip.  recoverIMU() reconfigures ONLY what is down, allocates nothing,
    // and leaves the healthy device's stream untouched.
    if (isIMUDegraded(imu) && isTimeout(IMU_RETRY_INTERVAL_MS, lastIMURetry)) {
        lastIMURetry = millis();
        imuReinits++;

        // The return code is not redundant with the ready flags: NOK_BUS_STUCK
        // says the bus itself could not be freed, which is a wiring or
        // pull-up fault, whereas both-flags-down with a Ready bus means the
        // devices are gone. Same symptom in the flags, different thing to fix.
        const IMUReturnStatus rst = recoverIMU(imu);
        if (rst == IMUReturnStatus::NOK_BUS_STUCK) imuBusStuck++;

        Serial.print("IMU: recovery status=");
        Serial.print(static_cast<int>(rst));
        Serial.print(" -> accel=");
        Serial.print(imu.accelReady ? "up" : "down");
        Serial.print(" mag=");
        Serial.println(imu.magReady ? "up" : "down");
    }

#ifdef USE_GPS
    // ── GPS poll at the production rate, sharing the same bus ────────────────
    // Automatic retry, mirroring production.  Manual re-init via 'g' is still
    // available, but a test that only recovers when an operator types something
    // cannot reproduce what the shipping firmware does unattended.
    if (!gpsReady && isTimeout(GPS_RETRY_MS, lastGPSRetry)) {
        lastGPSRetry = millis();
        gpsReady     = startGPS();
        lastGPSPoll  = lastGPSRetry;
        lastGPSFresh = lastGPSRetry;
        Serial.print("GPS: auto re-init ");
        Serial.print(gpsReady ? "ok" : "FAILED");
        Serial.print(" (status ");
        Serial.print(gpsInitStatus);
        Serial.println(")");
    }

    if (gpsReady && isTimeout(GPS_POLL_MS, lastGPSPoll)) {
        lastGPSPoll = millis();

        const uint32_t t0 = micros();
        const GPSReturnStatus st = getGPSData(myGNSS, gpsData);
        const uint32_t elapsed = micros() - t0;
        if (elapsed > maxGPSPollUs) maxGPSPollUs = elapsed;

        gpsPolls++;
        switch (st) {
            case GPSReturnStatus::OK:         gpsFresh++; break;
            case GPSReturnStatus::NO_FIX:     gpsNoFix++; break;
            case GPSReturnStatus::DATA_STALE: gpsStale++; break;
            default:                          gpsOther++; break;
        }

        if (st == GPSReturnStatus::OK || st == GPSReturnStatus::NO_FIX) lastGPSFresh = lastGPSPoll;

        // Same freshness rules as production, so the soak measures what ships.
        const unsigned long sinceFresh = millis() - lastGPSFresh;
        if (sinceFresh > GPS_FIX_MAX_AGE_MS)  gpsData.fixValid = false;
        if (sinceFresh > GPS_MAX_SILENCE_MS) {
            Serial.println("GPS: silence window exceeded; will re-init");
            gpsReady     = false;
            lastGPSRetry = millis();
        }
    }
#endif

    // ── console commands ─────────────────────────────────────────────────────
    // The setup() census is easy to miss: after an upload the board reboots and
    // re-enumerates before the host can reopen the port, so the one-shot scan
    // has usually already scrolled past. 's' re-runs it on demand.  Bounded by
    // COMMAND_BUDGET so a stream of junk on the port cannot stall loop().
    static constexpr uint8_t COMMAND_BUDGET = 4;
    for (uint8_t i = 0; i < COMMAND_BUDGET && Serial.available() > 0; i++) {
        const int command = Serial.read();
        if (command == 's' || command == 'S') {
            reportBusCensus();
#ifdef USE_GPS
        } else if (command == 'g' || command == 'G') {
            // Re-run the GNSS bring-up on demand.  If it succeeds here but
            // failed in setup(), the fault is ordering or timing against the
            // IMU init, not the receiver.
            gpsReady = startGPS();
            Serial.print("GPS: re-init ");
            Serial.print(gpsReady ? "ok" : "FAILED");
            Serial.print("  (status ");
            Serial.print(gpsInitStatus);
            Serial.println(")");
#endif
        } else if (command == 'r' || command == 'R') {
            // Soft reset, so bring-up reliability can be sampled over many
            // boots instead of inferred from one.  An intermittent init fault
            // is invisible in a single run and obvious over twenty.
            Serial.println("resetting...");
            Serial.flush();
            NVIC_SystemReset();
        } else if (command == 'h' || command == 'H') {
            Serial.println("commands: s = rescan bus, g = re-init GPS, r = reset, h = help");
        }
    }

    // ── 1 Hz report ──────────────────────────────────────────────────────────
    if (isTimeout(REPORT_MS, lastReport)) {
        lastReport = millis();
        digitalWrite(STATUS_INDICATOR, !digitalRead(STATUS_INDICATOR));
        reportLive();
    }
}
