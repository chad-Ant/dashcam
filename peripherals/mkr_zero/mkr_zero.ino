/**
 * MKR Zero — vehicle telemetry master.
 *
 * Owns the vehicle sensors and publishes them to the ESP32-C3 bridge, which
 * relays them to the Jetson Orin Nano:
 *
 *   [MKR Zero] ──UART1 115200, CommProtocol.h──> ESP32-C3 ──USB-C──> Jetson
 *
 * The C3 is the initiator on this hop: it sends CMD_START_STREAM and the MKR
 * pushes MSG_TELEMETRY at COMM_STREAM_INTERVAL_MS (10 Hz) until told to stop.
 * All of that lives in tickCommMaster(), which must be called every loop().
 *
 * Serial1 (pins 13/14) belongs to the C3 link.  Serial is the USB CDC console
 * and is safe to print to — it is a different peripheral entirely.
 *
 * Wiring:
 *   Serial1 TX (pin 14) -> C3 RX (GPIO20)
 *   Serial1 RX (pin 13) <- C3 TX (GPIO21)
 *   GND <-> GND
 */

// Bare include names, NOT "lib/X.h".  A path-qualified include resolves as a
// plain file relative to the sketch, so the Arduino library resolver never
// matches it to the lib/ folder and never compiles the .cpp files beside these
// headers: the sketch compiles and then fails at link with "undefined reference"
// to every function it calls.  Bare names bind to the library passed with
// --library (see BuildAndUpload.cmd).
#include "DataDictionary.h"
#include "TimerFunctions.h"
#include "MathFunctions.h"
#include "OBD2Functions.h"
#include "GPSFunctions.h"
#include "CommunicationFunctions.h"
#include "SignalProcessingFunctions.h"
#include "IMUFunctions.h"

/**
 * ENABLED: the u-blox module is fitted on I2C, sharing the bus with the IMU.
 *
 * IMPORTANT: initializeGPS() drives Serial1, which belongs to the ESP32-C3
 * link — only initializeGPS_I2C() is safe on this board.
 *
 * Comment this out again if the receiver is removed.  Doing so is safe rather
 * than merely tolerable: the telemetry frame still goes out, carrying NaN
 * positions and a clear COMM_FLAG_GPS_FIX, so the Jetson can tell "no GPS
 * fitted" from "fitted but no fix".  That distinction is also why the staged
 * bring-up retries — an unlucky attempt must not masquerade as absent hardware.
 */
#define USE_GPS 1

// ─── timings ──────────────────────────────────────────────────────────────────

static constexpr unsigned long SERIAL_READY_TIMEOUT_MS = 2000; ///< Bounded wait for a USB console.
static constexpr unsigned long OBD2_RETRY_MS           = 5000; ///< Re-init interval after a CAN failure.
// GPS_POLL_MS now comes from DataDictionary.h, next to GPS_REFRESH_RATE.  It was
// a local 250 here, which silently equalled the navigation period and made the
// two clocks beat; keeping the pair together is what lets GPSFunctions.cpp
// static_assert the relationship between them.
/**
 * Acceleration sampling interval (10 Hz).
 *
 * Fixed cadence on purpose: the estimator differentiates a SMOOTHED speed, so
 * it needs evenly spaced samples. Sampling only when the ECU happens to report
 * a new speed would give an irregular and unbounded dt (speed can hold steady
 * for a long time), which is exactly the input a derivative handles worst.
 * With the SIZE_16 window this is a 1.6 s smoothing span.
 */
static constexpr unsigned long ACCEL_SAMPLE_MS         = 100;
static constexpr unsigned long LED_BLINK_MS            = 2000; ///< Heartbeat LED period.
static constexpr unsigned long DEBUG_PRINT_MS          = 1000; ///< Console status period.

// ─── state ────────────────────────────────────────────────────────────────────

/// Vehicle power state, which decides the IMU sampling mode. See vehiclePowerState().
///
/// Declared HERE rather than beside the function that returns it: the Arduino
/// preprocessor generates prototypes for every function in a .ino and inserts
/// them ahead of the file's own definitions, so a type introduced mid-file is
/// not in scope for the prototype of the function returning it.  The build
/// failure is "does not name a type" pointing at a line that is obviously fine.
enum class VehiclePower : uint8_t { Unknown, On, Off };

OBD2Config OBD2S1Commands;
OBD2Data   obdData;
GPSData    gpsData;
CommMaster commMaster;

/// Nine-axis IMU on the same I2C bus as the GNSS receiver.
///
/// Not behind a USE_* guard, unlike GPS: the driver reports absent hardware
/// honestly all by itself — every reading stays NAN and COMM_FLAG_IMU_PRESENT
/// stays clear — so a board without the breakout fitted needs no compile-time
/// switch, and there is no second build configuration to keep working.
IMUDevice imuDev;
IMUData   imuData;
static unsigned long lastIMUPoll  = 0;
static unsigned long lastIMURetry = 0;
/// Master-computed signals published alongside the raw sensor readings.
DerivedSignals derived;

/// Longitudinal acceleration, derived from the quantised OBD2 speed by the
/// project signal-processing library (smooth-then-differentiate, jerk limited,
/// saturated). Global so its internal filter buffer is allocated once at
/// startup rather than churning the heap in loop().
AccelerationEstimator accelEst(SIZE_16);
static unsigned long  lastAccelSample = 0;

/// Course over ground is a CIRCULAR quantity: 359 deg and 1 deg are 2 deg
/// apart, so a plain SimpleMovingAverage would answer 180 (due south) for a
/// vehicle heading due north. CircularMovingAverage averages unit vectors
/// instead, which is correct across the wrap.
CircularMovingAverage headingFilter(SIZE_8);

#ifdef USE_GPS
SFE_UBLOX_GNSS       myGNSS;
static bool          gpsReady     = false;
static unsigned long lastGPSPoll  = 0;
/// millis() of the last poll that actually returned a PVT packet (OK or NO_FIX).
/// Distinct from lastGPSPoll, which advances even when nothing was waiting.
static unsigned long lastGPSFresh = 0;
/// Progress of the staged bring-up, advanced ONE STAGE per loop() pass.
/// A stage is up to 750 ms, not one exchange — see gpsInitTick().
static GPSInitState  gpsInit;
/// Stage reported by the last log line, so progress is logged on CHANGE only —
/// the machine is ticked every pass and would otherwise flood the console.
static GPSInitStage  prevGPSStage = GPSInitStage::Idle;
#endif

/// Last-reported state for each subsystem's edge log.  Seeded false so the
/// first successful bring-up is itself reported as a transition.
static bool prevOBDUp  = false;
static bool prevGPSUp  = false;
/// Tracked PER DEVICE, not as one composite.  Pulling the breakout's VIN on the
/// bench left the LIS3MDL still acknowledging — an unpowered I2C slave draws
/// parasitic power through the bus pull-ups — while the LSM6DSOX went down.  A
/// single OR-ed "IMU present" flag therefore reported a physically unpowered
/// module as healthy, and no LOST line was ever logged.  Two flags cannot hide
/// that.
static bool prevIMUAccelUp = false;
/// Edge memory for the "exactly one device answering" warning.
static bool prevIMUDegraded = false;
static bool prevIMUMagUp   = false;
static bool prevLinkUp = false;

/// The MCP2515 controller initialised.  Says NOTHING about the ECU.
static bool          obdReady       = false;
static unsigned long lastOBD2Retry  = 0;
/// True once an ECU has actually ANSWERED a request — not merely once the CAN
/// controller came up.
///
/// The distinction is the whole of the vehicle-power signal, and conflating the
/// two is what previously made low-power mode unreachable.  initializeOBD2()
/// configures a chip on the SPI bus; it succeeds with no vehicle attached at
/// all.  Only a decoded reply proves an ECU is powered and talking, which is the
/// thing "is the vehicle on?" is really asking.
static bool          obdEcuEverLive = false;
/// @c millis() of the last decoded ECU reply, for the shutdown timer.
static unsigned long lastEcuReplyMs = 0;
static unsigned long lastLEDBlink   = 0;
static unsigned long lastDebugPrint = 0;
static int           LED_on         = 1;

// ─── helpers ──────────────────────────────────────────────────────────────────

/**
 * @brief Logs a subsystem coming up or going down, once per change.
 *
 * Edge-triggered on purpose.  The retry paths already log every ATTEMPT, which
 * is what tells you a module is absent and being chased; this logs the
 * TRANSITION, which is what tells you when it changed and is otherwise lost in
 * the repetition.  A sensor that drops out at minute 40 of a drive and returns
 * at minute 41 leaves two lines here and nothing anywhere else.
 *
 * @param[in]     name    Subsystem label as it should appear in the log.
 * @param[in]     nowUp   Current state.
 * @param[in,out] prevUp  Caller-owned memory of the last reported state.
 */
static void logSubsystemEdge(const char *name, bool nowUp, bool &prevUp)
{
    if (nowUp == prevUp) return;
    prevUp = nowUp;
    Serial.print(name);
    Serial.println(nowUp ? ": RECOVERED" : ": LOST");
}

/**
 * @brief Reads vehicle power state from the OBD-II link.
 *
 * The ECU is the only thing on this node that knows whether the vehicle is
 * running.  When the ignition goes off the ECU stops answering and the CAN bus
 * goes quiet; when it is on, tickOBD2() keeps storing readings.  That signal is
 * binary and has no noise floor, which is precisely what the inertial and GNSS
 * heuristics it replaced did not have.
 *
 * A link that has NEVER come up this session yields Unknown rather than Off.
 * The distinction is the same one COMM_FLAG_*_PRESENT draws everywhere else in
 * this system: absent hardware and a failed peripheral are different diagnoses,
 * and only one of them is evidence about the vehicle.
 */
static VehiclePower vehiclePowerState()
{
    // No ECU has ever answered, so there is no evidence about the vehicle —
    // whether that is because no CAN shield is fitted, the wiring is wrong, or
    // the car was already off at boot.  Unknown, and Unknown behaves like On.
    if (!obdEcuEverLive) return VehiclePower::Unknown;

    // Deliberately NOT gated on obdReady.  Controller state answers "can I
    // transmit?", which is a different question: the MCP2515 stays perfectly
    // healthy while the ignition is off and every request goes unanswered.
    // Reading readiness as liveness is what pinned this at On forever.
    return ((millis() - lastEcuReplyMs) > VEHICLE_POWEROFF_CONFIRM_MS)
               ? VehiclePower::Off
               : VehiclePower::On;   // silent, but not yet long enough to judge
}

/**
 * @brief Selects the IMU sampling mode from vehicle power state.
 *
 * Low power exactly when the vehicle is powered off; full capture otherwise,
 * including whenever the power state cannot be established.
 *
 * WHAT THIS GIVES UP, stated plainly: a parked vehicle is sampled at 26 Hz with
 * no buffering, so an impact while parked — another car reversing into it — is
 * recorded coarsely and its peak is not a true peak.  That is inherent to
 * powering the sensor down, not an oversight.  Recovering it needs the
 * LSM6DSOX's own wake-up interrupt on a wired INT pin, which detects motion in
 * hardware at microamps and needs no threshold guessing at all; this build has
 * no INT line to the MKR.  COMM_FLAG_IMU_LOWPOWER is on the wire so a consumer
 * always knows which regime produced a frame.
 */
static void updateIMUSampleMode()
{
    if (!imuDev.accelReady) return;   // nothing to configure; recovery owns this

    const VehiclePower power = vehiclePowerState();
    const IMUSampleMode wanted = (power == VehiclePower::Off)
                                     ? IMUSampleMode::LowPower
                                     : IMUSampleMode::Fifo;

    if (imuSampleMode(imuDev) == wanted) return;

    const IMUReturnStatus st = setIMUSampleMode(imuDev, wanted);
    if (st == IMUReturnStatus::OK) {
        Serial.print("IMU: sample mode -> ");
        Serial.println((wanted == IMUSampleMode::LowPower)
                           ? "LOW POWER (vehicle powered off)"
                           : "FIFO (vehicle powered on)");
    } else {
        // Logged, not retried here: the next pass re-evaluates and tries again,
        // and a failed switch leaves the PREVIOUS mode running rather than an
        // undefined one — applySampleMode() only updates dev.mode once every
        // register has read back.
        Serial.print("IMU: sample mode change failed, status ");
        Serial.println(static_cast<int>(st));
    }
}

/**
 * @brief Updates the filtered course over ground.
 *
 * Publishes NAN unless three things hold: the receiver has a fix, the vehicle
 * is actually moving, and the filter window points consistently one way.  A
 * heading that fails any of those is not a slightly worse reading — it is a
 * direction sampled from noise, and burning it into recorded footage as though
 * it were a measurement is worse than showing nothing.
 *
 * Deliberately outside the USE_GPS guard so it stays compiled: with GPS
 * disabled gpsData holds NAN, the filter skips the sample, and the result is
 * NAN — the same answer, reached without a second code path to rot.
 */
static void updateHeading()
{
    if (!gpsData.fixValid || isnan(gpsData.velocityKmh) ||
        gpsData.velocityKmh < HEADING_MIN_SPEED_KMH) {
        derived.headingDeg = NAN;
        return;
    }

    float mean = NAN;
    if (!headingFilter.calculate(gpsData.headingDegrees, mean)) {
        derived.headingDeg = NAN;   // still warming up
        return;
    }

    const float r = headingFilter.resultantLength();
    derived.headingDeg = (!isnan(r) && r >= HEADING_MIN_RESULTANT) ? mean : NAN;
}

/** @brief Brings up the CAN/OBD-II interface. @return true on success. */
static bool startOBD2()
{
    const CANReturnStatus st =
        initializeOBD2(OBD2S1Commands, OBD2_TX_GLOBAL, OBD2_RX_ECM_1);
    if (st != CANReturnStatus::OK) {
        Serial.println("OBD2: CAN init failed");
        return false;
    }
    Serial.println("OBD2: CAN started");
    return true;
}

// ─── Arduino entry points ─────────────────────────────────────────────────────

void setup()
{
    pinMode(STATUS_INDICATOR, OUTPUT);
    digitalWrite(STATUS_INDICATOR, HIGH);

    Serial.begin(SERIAL_BAUDRATE);
    // Bounded.  An unbounded `while (!Serial)` is field-fatal on this board: on
    // battery or in a headless vehicle install there is no USB host to open the
    // port, so setup() would never reach CAN, GPS, or the C3 link and the whole
    // telemetry chain would sit dead with no indication why.
    const unsigned long serialWaitStart = millis();
    while (!Serial && !isTimeout(SERIAL_READY_TIMEOUT_MS, serialWaitStart)) {
        // spin briefly; the timeout is the guarantee
    }
    Serial.println("MKR Zero telemetry master");

    // Forensics first: RCAUSE is captured inside watchdogArm() before anything
    // can overwrite it, and a watchdog reset is the one boot reason that says
    // "the last run hung" rather than "someone power-cycled it".
    watchdogArm(WATCHDOG_PERIOD_MS);
    if (watchdogCausedReset()) Serial.println("BOOT: previous run was ended by the watchdog");

    initOBD2Data(obdData);
    initGPSData(gpsData);
    initIMUData(imuData);

    // The C3 link comes up FIRST and unconditionally.  It must answer even when
    // CAN and GPS never initialise, so the Jetson can distinguish "MKR alive,
    // no ECU" (telemetry arrives with COMM_FLAG_OBD2_VALID clear) from "MKR
    // dead" (no frames at all).  Getting this order wrong turns a diagnosable
    // sensor fault into a silent link.
    initializeComm();
    initCommMaster(commMaster);
    Serial.println("C3 link: Serial1 open");

    // A boot that FOLLOWS A HANG skips every I2C bring-up, and skips it for the
    // whole boot rather than for one attempt.  Both peripherals share one bus,
    // so either can be what wedged it, and initializeIMU() transacts just as
    // GNSS does — quarantining only GNSS would still let the IMU hang setup()
    // before loop() is ever reached, which is the same reboot loop by a
    // different route.
    const bool hangQuarantine = bootAfterHang();
    if (hangQuarantine) {
        Serial.println("BOOT: *** I2C QUARANTINE - the previous run was ended by the watchdog ***");
        Serial.println("      Skipping IMU and GNSS bring-up for THIS ENTIRE BOOT, including");
        Serial.println("      the retry paths. Telemetry keeps flowing with the inertial and");
        Serial.println("      position fields NaN and their PRESENT flags clear, so the Jetson");
        Serial.println("      sees 'not fitted' rather than silence. Power-cycle to retry.");
        Serial.println("      Suspect a device holding SDA low, or a wiring fault.");
    }

    // IMU BEFORE GNSS, which is the reverse of the old order and not a style
    // choice.  GNSS bring-up is the longest and least reliable step in setup(),
    // and while it ran first a hang there meant the IMU was never configured at
    // all — the watchdog reset the board, the same sequence ran again, and the
    // inertial sensor stayed dead across every boot of the loop.  Configuring
    // the IMU first means the most failure-prone peripheral can no longer starve
    // the most reliable one.
    if (hangQuarantine) {
        imuQuarantine(imuDev);   // valid struct, no bus access, no recovery
        Serial.println("IMU: quarantined");
    } else {
        const IMUReturnStatus st = initializeIMU(imuDev);
        Serial.print("IMU: ");
        if (st == IMUReturnStatus::OK)                        Serial.println("both devices up");
        else if (st == IMUReturnStatus::PARTIAL)              Serial.println("PARTIAL - one device down");
        else if (st == IMUReturnStatus::NOK_BUS_STUCK)        Serial.println("bus stuck - a line is held low");
        else if (st == IMUReturnStatus::NOK_ADDRESS_CONFLICT) Serial.println("address conflict");
        else                                                  Serial.println("not detected");
    }
    lastIMUPoll  = millis();
    lastIMURetry = lastIMUPoll;
    watchdogFeed();

#ifdef USE_GPS
    // Allocation only — this no longer touches the bus, so it cannot hang and
    // cannot leave a queued reply for the real bring-up to trip over.
    //
    // Checked, not discarded.  These are the only two GNSS allocations this
    // code can force to happen during setup(), so a failure here means the
    // driver will allocate from loop() instead — the exact thing the call
    // exists to prevent — and it must not pass silently.
    const bool gpsMemoryOk = (preallocateGPS_I2C(myGNSS) == GPSReturnStatus::OK);
    if (!gpsMemoryOk) {
        // TERMINAL for this boot, not merely logged.  A failed allocation is not
        // a degraded-but-usable state: setPacketCfgPayloadSize() leaves
        // payloadCfg NULL and packetCfgPayloadSize 0, begin() then retries it
        // WITHOUT checking the result, and the configuration path afterwards
        // writes payloadCfg[0] unconditionally.  Continuing into the bring-up
        // therefore risks a null dereference rather than another clean failure.
        Serial.println("GPS: *** preallocation FAILED - out of RAM ***");
        Serial.println("     GNSS disabled for this boot. Proceeding would risk a null");
        Serial.println("     dereference inside the driver, which cannot be caught here.");
    }

    // setup() no longer performs the bring-up: it only ARMS the state machine,
    // which loop() then advances one stage at a time.  Nothing here blocks, so
    // a receiver that never answers costs the boot nothing.
    if (hangQuarantine || !gpsMemoryOk) {
        // gpsInitQuarantine(), NOT gpsInitFail().  Fail() schedules a retry, and
        // a retry after a hang re-enters the hang: the earlier version used it
        // here and the machine re-entered Begin 250 ms later, so a PERSISTENT
        // wedge still reset the board every few seconds.  The bench test that
        // "passed" only ever hung once, which is why it looked fixed.
        // Quarantined is terminal — nothing leaves it before the next
        // non-watchdog reset.
        gpsInitQuarantine(gpsInit);
        Serial.println("GPS: quarantined");
    } else {
        gpsInitBegin(gpsInit);
    }
    lastGPSPoll  = millis();
    lastGPSFresh = lastGPSPoll;
    watchdogFeed();
#endif

    // Controller only.  Liveness is stamped in loop(), where an actual ECU reply
    // can be observed — see vehiclePowerState().
    obdReady      = startOBD2();
    lastOBD2Retry = millis();

    watchdogFeed();
}

void loop()
{
    // One feed per pass.  Every branch below is non-blocking by construction, so
    // reaching here at all is the health condition the watchdog is testing.
    watchdogFeed();
    // ── 1) OBD-II poll, with bounded recovery ────────────────────────────────
    if (obdReady) {
        // The return value is the vehicle-power signal: true means a reply was
        // decoded and stored, which only happens when an ECU answered.  It used
        // to be discarded and the clock stamped unconditionally, which made
        // every pass look like proof of life and kept the vehicle "on" forever.
        if (tickOBD2(OBD2S1Commands, obdData)) {
            obdEcuEverLive = true;
            lastEcuReplyMs = millis();
        }
        if (isOBD2LinkLost()) {
            // Defined fallback rather than silently publishing frozen readings:
            // drop the ready flag so the retry path below re-initialises, and
            // let buildTelemetry()'s freshness window clear COMM_FLAG_OBD2_VALID.
            obdReady      = false;
            lastOBD2Retry = millis();
            Serial.println("OBD2: link lost; will re-init");
        }
    } else if (isTimeout(OBD2_RETRY_MS, lastOBD2Retry)) {
        // Only the controller comes back here.  Nothing about this success says
        // an ECU is present, so the liveness clock is deliberately NOT stamped:
        // a parked vehicle re-initialises the MCP2515 every 5 s quite happily,
        // and refreshing liveness on each attempt would hold the system awake
        // for as long as it stayed parked.
        obdReady      = startOBD2();
        lastOBD2Retry = millis();
    }

#ifdef USE_GPS
    // ── 2) GPS poll, with bounded recovery ───────────────────────────────────
    if (gpsReady) {
        if (isTimeout(GPS_POLL_MS, lastGPSPoll)) {
            const GPSReturnStatus st = getGPSData(myGNSS, gpsData);
            lastGPSPoll = millis();

            // OK and NO_FIX both mean a PVT packet arrived — the receiver is
            // alive and talking.  Only DATA_STALE means nothing was waiting.
            if (st == GPSReturnStatus::OK || st == GPSReturnStatus::NO_FIX) {
                lastGPSFresh = lastGPSPoll;
                // A packet arrived, so the receiver is genuinely working — the
                // only evidence that earns a reset of the retry backoff.
                // Clearing it at Done instead let a receiver that configures
                // cleanly but never streams retry fast forever, because the
                // counter was wiped before it could ever escalate.
                gpsInitConfirmStreaming(gpsInit);
                // A fresh packet is authoritative either way: OK sets fixValid
                // inside getGPSData(), NO_FIX clears it there too.
            }
        }

        const unsigned long sinceFresh = millis() - lastGPSFresh;

        // Age the fix out on TIME, not on a single empty poll.  Polling at 5 Hz
        // against a 4 Hz receiver makes ~20 % of polls legitimately empty, and
        // the previous code cleared fixValid on every one of them — so
        // COMM_FLAG_GPS_FIX flickered off a fifth of the time with a perfectly
        // good fix.  That was visible in the bench capture as "sats=5 fix=no"
        // between healthy lines, and as flags=0x0C among 0x0E frames on the
        // wire; it read like a marginal fix and was not.
        //
        // The coordinates go with it.  Clearing fixValid alone left the last
        // latitude, longitude, speed and altitude sitting there as ordinary
        // numbers, which contradicts the invariant GPSData documents and puts
        // every consumer one forgotten flag check away from treating a position
        // minutes out of date as current.  A vehicle at 100 km/h moves 28 m per
        // second, so "stale but plausible" is the most dangerous form this can
        // take — it is wrong by an amount too small to look wrong.
        if (sinceFresh > GPS_FIX_MAX_AGE_MS) invalidateGPSFix(gpsData);

        // Silence for far longer than any beat can explain means the receiver
        // is gone — unplugged, browned out, reset — not merely between packets.
        // Without this gpsReady stayed true forever on the strength of one
        // successful bring-up, so a receiver that died mid-drive was never
        // retried and never reported as absent.
        if (sinceFresh > GPS_MAX_SILENCE_MS) {
            Serial.println("GPS: no PVT within the silence window; will re-init");
            gpsReady = false;
            // Rearm the state machine.  It is sitting at Done after the last
            // successful bring-up, and gpsInitTick() returns immediately from
            // there — without this the receiver would be marked not-ready and
            // then never re-initialised, which is the exact failure the silence
            // window exists to escape.  Fail() rather than Begin() so the retry
            // observes GPS_RETRY_MS instead of hammering a dead receiver.
            gpsInitFail(gpsInit, GPSReturnStatus::NOK_INIT_FAILED);
            prevGPSStage = GPSInitStage::Failed;   // already logged above
            // Everything, not just the fix: with the receiver gone, the UTC
            // stamp, satellite count and fix type are as stale as the position,
            // and a satellite count frozen at 7 is a particularly convincing
            // way to report a receiver that has been unplugged for a minute.
            initGPSData(gpsData);
        }
    } else {
        // Bring-up, ONE STAGE per pass.  This replaces a call that ran the whole
        // chain synchronously and could hold the CPU for ~3 s: long enough to
        // starve the IMU past its FIFO depth, and — if the bus wedged — long
        // enough to be reset by the watchdog at the same point on every boot.
        //
        // A stage is NOT one exchange: Begin is up to 750 ms and the
        // poll-plus-set setters up to 500 ms, so the C3 link and IMU drain do
        // still pause for that long during bring-up, and a pause past
        // IMU_MAX_DATA_AGE_MS is reported as an inertial data gap.  Bounded and
        // visible rather than hidden, which is the improvement; not eliminated.
        // The machine owns its own backoff, so there is no retry timer here.
        const GPSInitStage stage = gpsInitTick(myGNSS, gpsInit);

        if (stage != prevGPSStage) {
            prevGPSStage = stage;
            if (stage == GPSInitStage::Failed) {
                // The status matters: -1 means nothing answered at 0x42 (absent
                // receiver, dead bus), -2/-6 mean it answered and then refused a
                // setting, -7 means the bus was not safe to touch.  A retry
                // fixes the second, never the first.
                Serial.print("GPS: bring-up refused at ");
                Serial.print(gpsInitStageName(gpsInit.failedAt));
                Serial.print(", status ");
                Serial.print(static_cast<int>(gpsInit.lastStatus));
                Serial.print(", failure ");
                Serial.print(gpsInit.failures);
                Serial.println("; will retry");
            } else if (stage == GPSInitStage::Done) {
                Serial.println("GPS: module started");
            }
        }

        if (stage == GPSInitStage::Done) {
            gpsReady = true;
            // Seed both clocks, or the first poll fires before the receiver has
            // produced anything and the silence test tears down the receiver we
            // just brought up.
            lastGPSPoll  = millis();
            lastGPSFresh = lastGPSPoll;
        }
    }

    // Hardware presence for the wire, kept in step with the retry state above.
    gpsData.devicePresent = gpsReady;
#endif

    // ── 2c) IMU poll, with bounded recovery ──────────────────────────────────
    // Unconditional on the timer: getIMUData() returns immediately without
    // touching the bus when a device is down, so there is nothing to gate on.
    if (isTimeout(IMU_POLL_MS, lastIMUPoll)) {
        lastIMUPoll = millis();
        (void)getIMUData(imuDev, imuData);   // status is carried by the data's own valid flags
        // Immediately after the poll, so a wake acts on THIS pass's peaks rather
        // than on the previous 50 ms.  That one poll of latency is the entire
        // budget the low-power mode is allowed to cost on the way back up.
        updateIMUSampleMode();
    }
    // Driven by isIMUDegraded(), not isIMULinkLost(): with only one of the two
    // devices down the link is not "lost", and gating on that would strand the
    // missing sensor offline for the whole drive.  recoverIMU() reconfigures
    // only what is down and allocates nothing.
    if (isIMUDegraded(imuDev) && isTimeout(IMU_RETRY_MS, lastIMURetry)) {
        lastIMURetry = millis();
        // Outcome logged, not discarded: an IMU that drops off mid-drive was
        // previously retried forever in complete silence, so the console gave
        // no hint that anything had changed.
        const IMUReturnStatus rst = recoverIMU(imuDev);
        // Logged on EVERY attempt, matching the OBD2 and GPS retry paths.  A
        // module that is simply not fitted should keep saying so: silence would
        // read as "fine" to anyone scanning the log.
        Serial.print("IMU: retry status=");
        Serial.print(static_cast<int>(rst));
        Serial.print(" accel=");
        Serial.print(imuDev.accelReady ? "up" : "down");
        Serial.print(" mag=");
        Serial.println(imuDev.magReady ? "up" : "down");
    }

    // ── 2b) Derived signals ──────────────────────────────────────────────────
    // Sampled on a fixed cadence, not on ECU updates: see ACCEL_SAMPLE_MS.
    if (isTimeout(ACCEL_SAMPLE_MS, lastAccelSample)) {
        lastAccelSample = millis();
        (void)accelEst.update(obdData.speed, lastAccelSample); // NAN handled inside
        derived.accelMs2 = accelEst.value();
        updateHeading();
    }

    // ── 3) Serve the ESP32-C3 link ───────────────────────────────────────────
    // Non-blocking: services queued commands and pushes the 10 Hz stream.
    // Must run every pass — this is the only thing that answers the bridge.
    tickCommMaster(commMaster, obdData, gpsData, imuData, derived);

    // ── 3b) Subsystem transitions ────────────────────────────────────────────
    // Every module is optional at runtime: any of them can be absent at boot or
    // vanish mid-drive, and the system keeps producing telemetry either way
    // (missing values are NAN with their *_PRESENT flag clear).  What must never
    // happen is that it changes silently.
    logSubsystemEdge("OBD2", obdReady, prevOBDUp);
    logSubsystemEdge("IMU accel/gyro", imuDev.accelReady, prevIMUAccelUp);
    logSubsystemEdge("IMU mag",        imuDev.magReady,   prevIMUMagUp);

    // Exactly one device answering is a CONNECTION fault, not partial data.
    // Both parts sit on one PCB behind one VIN and one ground, so there is no
    // benign path to this state.  What produces it is a broken supply: an
    // unpowered I2C slave keeps acknowledging on parasitic current through the
    // bus pull-ups, so the lighter-draw part goes on answering while the other
    // dies.  Reported loudly because the readings that DO still arrive come from
    // a part running on stolen power and are not trustworthy either.
    const bool imuDegradedNow = (imuDev.accelReady != imuDev.magReady);
    if (imuDegradedNow != prevIMUDegraded) {
        prevIMUDegraded = imuDegradedNow;
        if (imuDegradedNow) {
            Serial.println("IMU: *** WARNING - only one of two devices answering ***");
            Serial.println("     Both share one VIN/GND on the breakout, so suspect POWER or");
            Serial.println("     WIRING, not a failed chip. An unpowered part still ACKs via");
            Serial.println("     parasitic power on the bus - do not trust the half that replies.");
        } else {
            Serial.println("IMU: both devices answering again");
        }
    }
#ifdef USE_GPS
    logSubsystemEdge("GPS",  gpsReady, prevGPSUp);
#endif
    // The C3 is the initiator on this hop, so inbound frames are the only proof
    // it is still attached — an unplugged UART accepts writes exactly like a
    // healthy one, and the master would otherwise stream into the void forever.
    logSubsystemEdge("C3 link", !isCommLinkSilent(commMaster, COMM_LINK_SILENT_MS), prevLinkUp);

    // ── 4) Heartbeat + console status ────────────────────────────────────────
    if (isTimeout(LED_BLINK_MS, lastLEDBlink)) {
        LED_on ^= 1;
        digitalWrite(STATUS_INDICATOR, LED_on);
        lastLEDBlink = millis();
    }

    if (isTimeout(DEBUG_PRINT_MS, lastDebugPrint)) {
        // Serial is the USB console, a different peripheral from Serial1 —
        // printing here cannot corrupt the C3 link.
        Serial.print("spd=");
        if (isnan(obdData.speed)) Serial.print("--"); else Serial.print(obdData.speed, 1);
        Serial.print(" rpm=");
        if (isnan(obdData.rpm)) Serial.print("--"); else Serial.print(obdData.rpm, 0);
        Serial.print("  obd=");
        Serial.print(obdReady ? "up" : "down");
#ifdef USE_GPS
        // Reported alongside OBD-II because the receiver is now a live
        // subsystem that can fail on its own.  Without this the only GNSS
        // evidence is a one-off line at boot, so a receiver that never came up
        // — or came up and later went quiet — is invisible for the rest of the
        // drive, which is precisely the state the retry exists to escape.
        Serial.print("  gps=");
        // While the receiver is down, show WHICH bring-up step the staged
        // machine is sitting on.  "down" alone cannot distinguish a receiver
        // that never answered from one that answered and then refused a
        // setting, and those are opposite repairs.
        if (gpsReady) Serial.print("up");
        else          Serial.print(gpsInitStageName(gpsInit.stage));
        Serial.print(" sats=");
        Serial.print(gpsData.satellites);
        Serial.print(" fix=");
        Serial.print(gpsData.fixValid ? "yes" : "no");
        // GNSS ground speed, which is a motion witness and therefore decides
        // whether the IMU may drop to low power.  Absent from this line before,
        // and its absence cost a bench session: the system sat in full capture
        // indefinitely and nothing displayed said why.
        Serial.print(" gspd=");
        if (isnan(gpsData.velocityKmh)) Serial.print("--");
        else                            Serial.print(gpsData.velocityKmh, 1);
#endif
        // Three states, not two: "partial" is the case a boolean cannot express
        // and the one that actually happened — half the breakout alive on
        // parasitic bus power while the other half was unpowered.
        Serial.print("  imu=");
        if (isIMUQuarantined(imuDev))                  Serial.print("QUARANTINED");
        else if (imuDev.accelReady && imuDev.magReady) Serial.print("up");
        else if (imuDev.accelReady || imuDev.magReady) Serial.print("PARTIAL");
        else                                           Serial.print("down");
        Serial.print(" |a|=");
        if (imuData.accelValid) {
            Serial.print(sqrtf(imuData.accelX * imuData.accelX +
                               imuData.accelY * imuData.accelY +
                               imuData.accelZ * imuData.accelZ), 2);
        } else {
            Serial.print("--");
        }
        // The peak is the number the FIFO exists to produce, so it is worth as
        // much bench visibility as the instantaneous magnitude beside it.  On a
        // still bench the two should track; under a tap on the desk only the
        // peak should jump, and that difference is the feature working.
        Serial.print(" pk=");
        if (isnan(imuData.accelPeakMs2)) Serial.print("--");
        else                             Serial.print(imuData.accelPeakMs2, 2);
        Serial.print("/");
        if (isnan(imuData.gyroPeakDps)) Serial.print("--");
        else                            Serial.print(imuData.gyroPeakDps, 1);
        Serial.print(" mode=");
        Serial.print(imuData.lowPower ? "LP" : "fifo");
        if (imuData.dataGap) Serial.print(" GAP");
        // The state that DECIDES the mode above, so the two can be compared.  A
        // mode that looks wrong is almost always a power state that is not what
        // was assumed — most often "unk", meaning no CAN interface has ever come
        // up and there is therefore no evidence about the vehicle at all.
        Serial.print(" veh=");
        switch (vehiclePowerState()) {
            case VehiclePower::On:  Serial.print("on");  break;
            case VehiclePower::Off: Serial.print("off"); break;
            default:                Serial.print("unk"); break;
        }
        // Cumulative failed IMU transactions, accel/mag.  A device can NACK
        // steadily and still never be declared lost — the consecutive-fault
        // counter resets on every success in between — so a bus that is quietly
        // degrading looks identical to a healthy one in every other field on
        // this line.  Slowly climbing numbers here are the only warning.
        Serial.print(" ioerr=");
        Serial.print(imuDev.accelIOErrors);
        Serial.print("/");
        Serial.print(imuDev.magIOErrors);
        // CUMULATIVE gap counts, hardware overrun / freshness discard.  The
        // GAP flag above lasts only IMU_PEAK_WINDOW_MS, so a soak watching the
        // console can miss every one of them and still look clean — which is
        // exactly what a 150 s run showing no GAP proved, and did not prove.
        // These only ever climb, so one glance answers "were there any?".
        Serial.print(" gaps=");
        Serial.print(imuDev.fifoOverruns);
        Serial.print("/");
        Serial.print(imuDev.fifoGapFlushes);
        // c3= is the LINK, stream= is the session on top of it.  They are
        // different failures: a bridge that is attached but not asking for
        // telemetry is healthy, one that has been unplugged is not, and
        // stream=off alone cannot tell them apart.  This also covers the case
        // the edge log cannot — a module absent from boot never transitions, so
        // without a continuous readout it would never appear anywhere.
        Serial.print("  c3=");
        Serial.print(isCommLinkSilent(commMaster, COMM_LINK_SILENT_MS) ? "down" : "up");
        Serial.print(" stream=");
        Serial.println(commMaster.streaming ? "on" : "off");
        lastDebugPrint = millis();
    }
}
