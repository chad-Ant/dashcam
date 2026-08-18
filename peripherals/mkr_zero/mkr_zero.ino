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
#include "CANSniffFunctions.h"
#include "SDFunctions.h"
#include "VehicleSignals.h"
#include "SignalProcessingFunctions.h"
#include "IMUFunctions.h"
#include "BNO055Calib.h"   // calibration offsets, saved to and restored from SD
#include "SwitchFunctions.h"

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

/**
 * The common template both sources fill, and the CAN controller's current mode.
 *
 * Boot default is SNIFF, not OBD2 — inverting what this sketch used to do.
 * Listen-only emits neither ACK bits nor error frames, so until the bit timing
 * has been proven on a given vehicle the node cannot disturb the bus at all;
 * and sniffing is the better source anyway (50-100 Hz and 0.01 km/h, against a
 * ~500 ms poll cycle quantised to whole km/h). OBD2 is entered only when the
 * Jetson asks, via CMD_SET_CAN_MODE.
 */
VehicleSignals vehSignals;
YawEstimator   yawEst;
CanMode        canMode = CanMode::OFF;
/// The vehicle signal map, read from the card at boot. Falls back to the
/// compiled-in Honda map when the card holds none.
CanSignalMap   canMap;
/// Boot-time "is this map for this car?" decision. Runs to a verdict ONCE.
CanProbeState  canProbe = { CanProbeStage::Idle, 0, false };
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
/// Calibration offsets read from the card at boot.
///
/// FILE SCOPE IS LOAD-BEARING: BNO055InitState holds a POINTER to this, not a
/// copy, and the bring-up that dereferences it runs from loop(). A buffer local
/// to setup() would be gone by then, and the sensor would be configured from
/// whatever the stack had become.
static uint8_t imuCalibProfile[BNO055_CALIB_BYTES];
static bool    imuCalibProfileValid = false;
/// Rate limiter for saving a newly converged calibration.
static unsigned long lastCalibSaveMs = 0;
static unsigned long lastIMUPoll  = 0;
static unsigned long lastIMURetry = 0;

/// Panel switches behind the 74HC165 chain. Positions only — this firmware
/// never branches on one; see SwitchFunctions.h for why that is deliberate.
static SwitchData    switches;
static unsigned long lastSwitchPoll = 0;
/// Edge-detect for logging the chain's presence, matching the other subsystems.
static bool          prevSwitchesUp = false;

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
/// Latches the one-off "bus is clamped, not attempting GNSS" notice.
static bool          gpsBusBlockedLogged = false;
/// Latches the one-off latch-up notice, cleared if the bus ever frees.
static bool          imuLatchLogged = false;
/// Which preserved failure has already been printed, so the block below is
/// reported once per episode rather than on every five-second retry. Keyed on
/// the failure NUMBER rather than a bool: a second, different failure after a
/// recovery is new evidence and has to print again.
static uint16_t      imuFailureLogged = 0;
#endif

/// Last-reported state for each subsystem's edge log.  Seeded false so the
/// first successful bring-up is itself reported as a transition.
static bool prevOBDUp  = false;
static bool prevGPSUp  = false;
/// One flag now, where the LSM6DSOX + LIS3MDL breakout needed two.
///
/// That pair was tracked per device because pulling the breakout's VIN left the
/// LIS3MDL still acknowledging — an unpowered I2C slave draws parasitic power
/// through the bus pull-ups — while the LSM6DSOX went down, so a single OR-ed
/// flag reported a physically unpowered module as healthy. One chip cannot
/// produce that state, and the second flag went with it.
static bool prevIMUUp = false;
/// Edge memory for the "fusion not yet trustworthy" notice.
static bool prevIMUDegraded = false;
static bool prevLinkUp = false;

/// The MCP2515 controller initialised.  Says NOTHING about the ECU.
static bool          obdReady       = false;
/// Consecutive OBD-II sessions that ended without a single ECU reply. Bounds
/// the retry loop; see OBD2_DEAD_SESSIONS.
static uint8_t       obdDeadSessions = 0;
static unsigned long lastOBD2Retry  = 0;
/// True once an ECU has actually ANSWERED a request — not merely once the CAN
/// controller came up.
///
/// The distinction is the whole of the vehicle-power signal, and conflating the
/// two is what previously made low-power mode unreachable.  initializeOBD2()
/// configures a chip on the SPI bus; it succeeds with no vehicle attached at
/// all.  Only a decoded reply proves an ECU is powered and talking, which is the
/// thing "is the vehicle on?" is really asking.
static bool          vehBusEverLive = false;
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
    // Nothing has ever arrived from the vehicle — no decoded broadcast frame in
    // a listen-only mode, and no ECU reply in OBD2 mode.  No evidence either
    // way: no CAN shield fitted, wrong wiring, or the car was already off at
    // boot.  Unknown, and Unknown behaves like On.
    if (!vehBusEverLive) return VehiclePower::Unknown;

    // Deliberately NOT gated on obdReady.  Controller state answers "can I
    // transmit?", which is a different question: the MCP2515 stays perfectly
    // healthy while the ignition is off and every request goes unanswered.
    // Reading readiness as liveness is what pinned this at On forever.
    return ((millis() - lastEcuReplyMs) > VEHICLE_POWEROFF_CONFIRM_MS)
               ? VehiclePower::Off
               : VehiclePower::On;   // silent, but not yet long enough to judge
}

/**
 * @brief BNO055 High-G interrupt handler. Timestamps an impact, nothing more.
 *
 * Does NOT touch I2C, and must not: the poll it would race is very likely
 * already inside Wire, and a nested transaction on a bus whose driver has
 * unbounded waits is a hang rather than a corrupted read.
 *
 * The library reads the same latch over I2C on the next poll regardless, so
 * this handler is not what makes High-G work — it only makes the timestamp the
 * instant of the impact rather than the instant it was noticed.
 */
static void onIMUHighG()
{
    imuNoteHighGPin(millis());
}

/**
 * @brief Prints the preserved bring-up failure, once per episode.
 *
 * THE STATE THIS PRINTS IS ALREADY GONE FROM EVERYWHERE ELSE. Recovery restarts
 * the bring-up machine, which resets @c lastStatus, re-runs the reads behind
 * @c sysStatus / @c sysError / the register read-back, and clears the fault
 * counters. So a rig found retrying after a minute could previously only report
 * the LAST attempt — invariably "not found" — while the fault that started it
 * had been overwritten by the attempts to fix it. @c BNO055FailureRecord latches
 * the first failure; this is where it reaches a human.
 *
 * Deliberately verbose and deliberately RARE. It is the block that decides
 * whether the next recurrence is diagnosed or merely reproduced, and it prints
 * once per failure episode rather than on every five-second retry.
 */
static void printIMUFailureRecord()
{
    const BNO055FailureRecord &r = imuDev.init.firstFailure;
    if (!r.valid || r.failureNo == imuFailureLogged) return;
    imuFailureLogged = r.failureNo;

    Serial.println(F("IMU: ---- preserved failure evidence ------------------------"));
    Serial.print(F("     first failure #")); Serial.print(r.failureNo);
    Serial.print(F(" at "));                 Serial.print(r.atMs);
    Serial.print(F(" ms, total since boot ")); Serial.println(imuDev.init.failuresTotal);

    Serial.print(F("     stage=")); Serial.print(bno055InitStageName(r.failedAt));
    Serial.print(F(" why="));       Serial.println(bno055InitStatusName(r.lastStatus));

    // Address 0 means identification never succeeded, which is a different fault
    // from a part that answered and then refused its configuration. The chip and
    // page IDs say which: a page of 1 is the recoverable case, and a chip ID that
    // is neither 0xA0 nor 0x00 suggests something else is at that address.
    Serial.print(F("     addr=0x"));   Serial.print(r.address, HEX);
    Serial.print(F(" chipId=0x"));     Serial.print(r.chipId, HEX);
    Serial.print(F(" pageId="));       Serial.print(r.pageId);
    Serial.print(F(" opMode=0x"));     Serial.print(r.opMode, HEX);
    Serial.print(F(" seen=0x"));       Serial.println(r.opModeSeen, HEX);

    Serial.print(F("     sysStatus=0x")); Serial.print(r.sysStatus, HEX);
    Serial.print(F(" sysErr=0x"));        Serial.println(r.sysError, HEX);

    // The three-way distinction a bare "config-failed" cannot make: a NACKed
    // write, a failed read-back, or a write the part accepted and ignored.
    Serial.print(F("     reg 0x"));  Serial.print(r.lastRegAddr, HEX);
    Serial.print(F(" wrote 0x"));    Serial.print(r.lastRegWrote, HEX);
    Serial.print(F(" read 0x"));     Serial.print(r.lastRegRead, HEX);
    Serial.print(F(" writeOk="));    Serial.print(r.lastRegWriteOk ? 1 : 0);
    Serial.print(F(" readOk="));     Serial.println(r.lastRegReadOk ? 1 : 0);
    if (r.lastRegWriteOk && r.lastRegReadOk && r.lastRegWrote != r.lastRegRead) {
        Serial.println(F("     ^ write was ACKED AND IGNORED - wrong page, wrong mode, or no clock"));
    }

    Serial.print(F("     transport faults=")); Serial.print(r.transportFaults);
    Serial.print(F(" errors="));               Serial.print(r.transportErrors);
    Serial.print(F(" i2cStuck=0x"));           Serial.println(r.stuckLines, HEX);
    // The first fork in the diagnosis: a stuck line is the SHARED bus, so the
    // GNSS is affected too and this part is not the culprit.
    if (r.stuckLines != 0u) {
        Serial.println(F("     ^ the shared bus was stuck - check the GNSS too, this may not be the IMU"));
    }
    Serial.println(F("IMU: -------------------------------------------------------"));
}

/*
 * THE IMU MODE IS NO LONGER DRIVEN BY VEHICLE POWER, and that is a deliberate
 * removal rather than a feature lost in the port.
 *
 * The previous sensor's two modes were a SAMPLING RATE choice — full FIFO
 * capture while moving, a slower unbuffered poll while parked to save current —
 * so following the ignition was the right rule and the cost was only resolution.
 *
 * The BNO055's two modes are a MEASUREMENT choice. Fusion yields linear
 * acceleration and relative yaw and locks the accelerometer at ±4 g; Raw yields
 * neither and can reach ±16 g. Switching between them changes what the numbers
 * mean, discards the peak window, and costs a full re-configuration of roughly
 * 700 ms — during which there is no inertial data at all. Doing that at every
 * traffic light would blank the sensor exactly as often as the vehicle stops.
 *
 * So the mode is a session-level decision, set once at bring-up. Current draw
 * while parked is a separate axis (PWR_MODE), and it stays at normal until the
 * parked-mode work lands: the datasheet withdraws the High-G interrupt in low
 * power, which would remove the hardware impact backstop precisely when a
 * car-park bump is the thing being watched for.
 */

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
static void updateHeading(bool freshSample)
{
    if (!gpsData.fixValid || isnan(gpsData.velocityKmh) ||
        gpsData.velocityKmh < HEADING_MIN_SPEED_KMH) {
        // RESET the window, not merely blank the output.  Clearing only the
        // output left the filter holding the courses recorded before the stop,
        // so when the vehicle pulled away in a NEW direction those stale samples
        // were still in the window — and being mutually consistent, they held
        // resultantLength() above the threshold and certified the OLD course as
        // current for the first second or two of the new one.  A heading that is
        // confidently wrong is worse than no heading, which is the whole reason
        // this function publishes NAN so readily.
        headingFilter.reset();
        derived.headingDeg = NAN;
        return;
    }

    // Fed ONLY on a fresh PVT packet.  This used to run on the 10 Hz derived
    // signal tick against a 4 Hz receiver, so each course was inserted two or
    // three times.  Duplicates are not merely wasted window slots: identical
    // samples are perfectly coherent, so they INFLATE resultantLength() — the
    // one statistic whose job is to reject an incoherent window — and make a
    // noisy heading look trustworthy.  With SIZE_8 and 2.5x oversampling the
    // window also spanned barely three real fixes.
    if (!freshSample) return;

    float mean = NAN;
    if (!headingFilter.calculate(gpsData.headingDegrees, mean)) {
        derived.headingDeg = NAN;   // still warming up
        return;
    }

    const float r = headingFilter.resultantLength();
    derived.headingDeg = (!isnan(r) && r >= HEADING_MIN_RESULTANT) ? mean : NAN;

    // ── Teach the yaw estimator what "straight" looks like ───────────────────
    //
    // GNSS is the independent reference here, which is the whole point: the
    // wheel pair cannot tell a genuine turn from a tyre-radius mismatch, because
    // both produce a persistent left-right difference. Only something that
    // measures heading by other means can attribute the residual to the tyres.
    //
    // The mismatch is worth correcting: measured on this vehicle at about
    // +0.74 %, which at 40 km/h fakes ~3.5 deg/s and renders straight motorway
    // driving as a permanent 180 m left-hand curve.
    static float    prevHeadingDeg = NAN;
    static uint32_t prevHeadingMs  = 0;

    const uint32_t nowMs = millis();
    if (!isnan(derived.headingDeg)) {
        if (!isnan(prevHeadingDeg) && prevHeadingMs != 0) {
            const uint32_t dtMs = nowMs - prevHeadingMs;
            // Bounded both ways: too short and the rate is quantisation noise,
            // too long and the vehicle may have turned and come back between
            // samples, which reads as straight and is not.
            if (dtMs >= 100u && dtMs <= 1000u) {
                // angleDiff360 because heading wraps - 359 to 1 is +2, not -358.
                const float rateDps =
                    angleDiff360(derived.headingDeg, prevHeadingDeg) * 1000.0f / (float)dtMs;
                if (fabsf(rateDps) < YAW_STRAIGHT_MAX_DPS &&
                    vehSignals.wheelRaw[VEH_WHEEL_RL] != VEH_WHEEL_INVALID &&
                    vehSignals.wheelRaw[VEH_WHEEL_RR] != VEH_WHEEL_INVALID) {
                    yawObserveStraight(yawEst,
                                       vehSignals.wheelRaw[VEH_WHEEL_RL],
                                       vehSignals.wheelRaw[VEH_WHEEL_RR]);
                }
            }
        }
        prevHeadingDeg = derived.headingDeg;
        prevHeadingMs  = nowMs;
    } else {
        // No trustworthy course: drop the anchor rather than measure a rate
        // across the gap, which would span an unknown amount of turning.
        prevHeadingDeg = NAN;
        prevHeadingMs  = 0;
    }
}

/**
 * @brief Brings up the CAN/OBD-II interface. @return true on success.
 *
 * REFUSES unless the controller is off or already in OBD2 mode.  Both calls
 * inside `initializeOBD2()` end in Normal mode — `CAN.begin()` because
 * `stayInConfigurationMode` defaults false, and `CAN.filter()` by upstream's own
 * documented contract — so running this during a listen-only session silently
 * puts the node on the bus with a 0x7E8-only filter while `canMode` still says
 * SNIFF.  That is not a theoretical risk: it used to happen five seconds into
 * every boot, and the guard is here because a comment claiming it could not was
 * not enough.
 */
/**
 * @brief Brings the MCP2515 up.
 *
 * @param stayInConfig Leave the controller in Configuration mode — off the bus —
 *        for the caller to move with @c applyCanMode(). The boot path uses this:
 *        the default bring-up ends in Normal, which made the node ACK-capable on
 *        a live vehicle bus for the window between here and the first mode
 *        selection. Short, but listen-only is a property this firmware claims
 *        from power-on, and a property with a hole in it is a hope.
 */
static bool startOBD2(bool stayInConfig = false)
{
    const CanMode m = canGetMode();
    if (m != CanMode::OFF && m != CanMode::OBD2) {
        Serial.print("OBD2: refusing CAN init while in ");
        Serial.println(canModeName(m));
        return false;
    }

    const CANReturnStatus st =
        initializeOBD2(OBD2S1Commands, OBD2_TX_GLOBAL, OBD2_RX_ECM_1,
                       MCP2515_DEFAULT_CS_PIN, MCP2515_DEFAULT_INT_PIN, stayInConfig);
    if (st != CANReturnStatus::OK) {
        // Which failure, not just that there was one. NOK_INIT_FAILED means the
        // MCP2515 never acknowledged a mode change across a 50 ms bounded poll —
        // i.e. the chip is not answering SPI, which is a wiring or supply fault
        // and not something a retry will argue with. NOK_STATUS_BAD means it
        // answered but One-Shot Mode would not latch. Those need opposite
        // investigations, and one message for both sent this bring-up looking
        // for a software cause that was never there.
        Serial.print("OBD2: CAN init failed - ");
        switch (st) {
            case CANReturnStatus::NOK_INIT_FAILED:
                Serial.println("controller not responding (check power and SPI wiring)");
                break;
            case CANReturnStatus::NOK_STATUS_BAD:
                Serial.println("One-Shot Mode did not latch; refusing to transmit");
                break;
            default:
                Serial.println((int)st);
                break;
        }
        return false;
    }
    Serial.println("OBD2: CAN started");
    return true;
}

/** @brief Human-readable name for a mode, for the console log only. */
static const char *canModeName(CanMode m)
{
    switch (m) {
        case CanMode::DISCOVER: return "discover";
        case CanMode::SNIFF:    return "sniff";
        case CanMode::OBD2:     return "obd2";
        default:                return "off";
    }
}

/**
 * @brief Applies a mode transition and re-arms whatever the new mode owns.
 *
 * Both directions have to be handled, not just the interesting one.  Leaving
 * SNIFF must clear the sniffed values, because they are about to stop being
 * updated and a consumer would otherwise keep reading a speed frozen at the
 * instant of the switch; leaving OBD2 must clear the poller, because its
 * failure counters describe a session that has ended.
 */
static bool applyCanMode(CanMode want)
{
    const CanModeStatus st = canSetMode(want, MCP2515_DEFAULT_CS_PIN);
    if (st == CanModeStatus::UNCHANGED) return true;
    if (st != CanModeStatus::OK) {
        Serial.print("CAN: mode ");
        Serial.print(canModeName(want));
        Serial.print(" REFUSED, status ");
        Serial.println(static_cast<int>(st));
        // Resynchronise with the driver instead of keeping the old value.
        //
        // A rejected transition still changed hardware on every path except the
        // rate limit, and canSetMode() parks the controller and reports OFF. If
        // this kept reporting the PREVIOUS mode, telemetry, the poller gate and
        // the retry gate would all be steering off a mode the controller is
        // provably not in — and the one that matters is claiming SNIFF, because
        // everything downstream then treats a possibly bus-active node as a
        // silent one. NOK_RATE_LIMIT alone changed nothing, so canGetMode()
        // still returns the real mode there and this is a no-op.
        canMode = canGetMode();
        return false;
    }

    canMode = want;

    // Whatever the previous mode was publishing is now unmaintained. Blank it
    // rather than let it age out: the freshness window would hold values live
    // for up to a second after the source that fed them was switched off.
    initVehicleSignals(vehSignals);
    initYawEstimator(yawEst, canSniffGetMap());

    if (want == CanMode::OBD2) {
        obdReady = true;
        resetOBD2Poll();
    } else {
        // Just "not polling". This comment used to claim obdReady=false also
        // stopped the retry path re-initialising into Normal mode; it did the
        // opposite — see the retry branch in loop(), which is now gated on the
        // mode instead. Keeping obdReady false here is still right, it simply
        // is not the thing protecting listen-only.
        obdReady = false;
    }

    Serial.print("CAN: mode -> ");
    Serial.println(canModeName(want));
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
    Serial.print("BOOT: reset cause 0x");
    Serial.print(resetCauseRaw(), HEX);
    Serial.print(" - ");
    Serial.println(resetCauseName());

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
        // The High-G INT line, if one is fitted. Pulled DOWN, so an unconnected
        // pin reads low forever and produces no events — the feature degrades to
        // reading the same latch over I2C on the next poll, which is the only
        // reason the wire is optional at all.
        pinMode(IMU_HIGHG_INT_PIN, INPUT_PULLDOWN);
        attachInterrupt(digitalPinToInterrupt(IMU_HIGHG_INT_PIN), onIMUHighG, RISING);

        // Starts the staged bring-up and returns immediately; imuInitTick() in
        // loop() carries it to completion over roughly 700 ms of wall clock.
        // setup() no longer waits for a verdict, because waiting means holding
        // the CPU through the part's own 650 ms reset — with the C3 link, the
        // CAN drain and the watchdog feed all stopped, and a hang landing at the
        // same point on every boot under the same armed watchdog.
        // DASHCAM_IMU_MODE_AMG selects the raw mode instead, and is UNSET in
        // production — the default below is the shipped configuration and this
        // block compiles to exactly what it did before.
        //
        // It exists because the mode is chosen once, here, and there is no
        // command to change it at runtime: the raw path therefore has no way of
        // being exercised end to end without a rebuild. Selecting it at compile
        // time keeps the alternative honest — what gets flashed is the
        // production firmware in its other documented mode, not a test harness
        // wearing its name.
        //
        //   arduino-cli compile --build-property "build.extra_flags=-DDASHCAM_IMU_MODE_AMG" ...
#ifdef DASHCAM_IMU_MODE_AMG
        const IMUSampleMode bootMode = IMUSampleMode::Raw;
#else
        const IMUSampleMode bootMode = IMUSampleMode::Fusion;
#endif
        const IMUReturnStatus st = initializeIMU(imuDev, bootMode);
        Serial.print("IMU: ");
        if (st != IMUReturnStatus::OK)                 Serial.println("bring-up REFUSED");
        else if (bootMode == IMUSampleMode::Raw)       Serial.println("bring-up started (AMG)");
        else                                           Serial.println("bring-up started (IMUPLUS)");

        // The bus is still worth naming at BOOT, before anything has run. A bus
        // already down on a cold start is the single most diagnostic line in the
        // log: nothing this firmware has done yet can be the cause, so it rules
        // out the CAN drain, the SD card and every runtime path at once.
        if (i2cBusBegin() != I2CBusState::Ready) {
            Serial.print("     bus stuck at boot - ");
            Serial.println(i2cStuckReason());
            Serial.println("     Nothing has run yet, so this is wiring or a slave holding the");
            Serial.println("     line from power-up - NOT anything this firmware did. The GNSS");
            Serial.println("     shares this bus, so it will fail too until the line is freed.");
        }
    }
    lastIMUPoll  = millis();
    lastIMURetry = lastIMUPoll;
    watchdogFeed();

    // ── panel switches ───────────────────────────────────────────────────────
    //
    // Unconditional, and safe on a quarantined boot: the 74HC165 chain has three
    // pins of its own and touches neither the I2C bus nor SPI, so nothing it can
    // do affects the two subsystems that quarantine exists to protect. It is
    // also the only peripheral here that cannot hang — the read is a fixed count
    // of GPIO toggles with no acknowledgement to wait for.
    initializeSwitches(switches);
    lastSwitchPoll = millis();
    prevSwitchesUp = switches.present;
    Serial.print("SW: 74HC165 x");
    Serial.print(SWITCH_REGISTERS);
    Serial.print(" -> ");
    if (switches.present) {
        Serial.print("present, ");
        Serial.print(SWITCH_COUNT);
        Serial.print(" inputs, state 0x");
        Serial.println(switches.state, HEX);
    } else {
        // Named at boot for the same reason the I2C bus is: nothing has run yet,
        // so this is wiring or an unpowered chain, not something a later path
        // did. The sentinel pattern is the whole test — see SwitchFunctions.h.
        Serial.print("ABSENT (raw 0x");
        Serial.print(switches.lastRaw, HEX);
        Serial.println(") - sentinels not answering.");
        Serial.println("    Expect #A D0 (pin 11) tied to GND and #A D1 (pin 12) tied to 3V3.");
        Serial.println("    A raw word of 0xFFFF with no chain fitted is the normal reading here.");
    }
    watchdogFeed();

#ifdef USE_GPS
    // Skipped entirely on a quarantined boot: GNSS will not be brought up at
    // all, so claiming its buffers would only consume RAM nothing will read.
    //
    // Otherwise allocation only — it touches no bus, so it cannot hang and
    // cannot leave a queued reply for the real bring-up to trip over.  Checked,
    // not discarded: these are the only two GNSS allocations this code can force
    // into setup(), so a failure means the driver would allocate from loop()
    // instead, which is the exact thing the call exists to prevent.
    const bool gpsMemoryOk =
        hangQuarantine || (preallocateGPS_I2C(myGNSS) == GPSReturnStatus::OK);
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
        //
        // The reason is passed, not assumed: these two paths are different
        // faults with different repairs, and reporting both as a stuck bus (as
        // this did) sent a technician looking for a held line on a board that
        // had simply run out of RAM.
        gpsInitQuarantine(gpsInit, hangQuarantine ? GPSReturnStatus::NOK_BUS_STUCK
                                                 : GPSReturnStatus::NOK_OUT_OF_MEMORY);
        Serial.println("GPS: quarantined");
    } else {
        gpsInitBegin(gpsInit);
    }
    lastGPSPoll  = millis();
    lastGPSFresh = lastGPSPoll;
    watchdogFeed();
#endif

    // ── Onboard microSD ──────────────────────────────────────────────────────
    //
    // Mounted here, before anything touches the CAN controller, so the vehicle
    // signal map (Phase B) can be read before the first mode is chosen. The card
    // is on SPI1, a different SERCOM from the MCP2515, so this cannot contend
    // with the CAN bring-up below; it is still ordered first so there is no
    // question about interleaved chip selects.
    //
    // NOT skipped on a quarantined boot. The quarantine exists for the shared
    // I2C bus, whose SERCOM waits are unbounded; SdFat's begin() is bounded by
    // its own card-init timeout and fails fast on an empty slot.
    //
    // Non-fatal by design. A rig with no card is still a working telemetry node.
    // config.txt is gone, not merely unread. It was parsed into an SDConfig that
    // then went out of scope untouched, and the boot printed "config.txt loaded"
    // for a file that changed nothing — a capability that looked supported and
    // was not. Every field it carried mirrors a compile-time constant consumed
    // directly elsewhere (the CS pin reaches the vendored library's own default),
    // so honouring it is a feature to design, not a line to restore.
    if (initializeSD() == SDReturnStatus::OK) {
        Serial.println("SD: card mounted (SPI1)");
    } else {
        Serial.println("SD: no card - defaults only");
    }
    watchdogFeed();

    // ── IMU calibration profile ──────────────────────────────────────────────
    //
    // Loaded here rather than beside initializeIMU() above, because the card is
    // not mounted until now — and it does not need to be earlier: bring-up is
    // staged, so nothing has reached the RestoreCalib step yet. That step runs
    // from imuInitTick() in loop(), and setup() finishes first.
    //
    // The BNO055 loses its calibration on every power-on and nothing on the part
    // remembers it, so without this each drive starts with an uncalibrated
    // sensor. On the bench the accelerometer figure fell to 0 and stayed there
    // for 9000 polls: a board bolted into a bracket does not see the
    // orientations Bosch's algorithm wants, so it may never recover on its own.
    {
        uint16_t storedInstall = 0;
        if (bno055CalibLoad(imuCalibProfile, &storedInstall)) {
            // Refused if it was captured under a different install ID. These
            // offsets are properties of the silicon, so a profile survives the
            // board being unbolted and remounted — but not the sensor being
            // replaced, and a stale one would bias every reading with nothing to
            // show for it. The part has no serial number, so this is an operator
            // -managed ID rather than an automatic check; see the constant.
            if (storedInstall != BNO055_CALIB_INSTALL_ID) {
                Serial.print("IMU: stored calibration is for install 0x");
                Serial.print(storedInstall, HEX);
                Serial.print(", this firmware is 0x");
                Serial.println(BNO055_CALIB_INSTALL_ID, HEX);
                Serial.println("     IGNORED. Delete bno055.cal, or recalibrate and re-save.");
            } else {
                imuCalibProfileValid = true;
                bno055InitSetCalibProfile(imuDev.init, imuCalibProfile);
                char desc[80];
                bno055CalibDescribe(imuCalibProfile, desc, sizeof(desc));
                Serial.print("IMU: calibration profile loaded - ");
                Serial.println(desc);
            }
        } else {
            // Not an error, and deliberately not logged as one: this is the
            // ordinary state of a rig that has never been calibrated. Press 's'
            // in IMUValidation once the figures reach 3 to create one.
            Serial.println("IMU: no stored calibration - the sensor will earn its own");
        }
    }
    watchdogFeed();

    // Bring the controller up and go straight to listen-only sniffing.
    //
    // startOBD2() is still what configures the MCP2515 (bit timing, pins, OSM),
    // so it runs first — but the node does NOT stay bus-active. applyCanMode()
    // immediately moves it to Listen-Only, where it emits neither ACK bits nor
    // error frames. That ordering matters on a vehicle whose bit timing has not
    // been proven: a wrong CNF setting in Normal mode produces error frames and
    // can set a VSA light, while in listen-only it costs nothing but frames.
    //
    // If the controller fails to come up at all, the retry path below handles
    // it exactly as before.
    // The vehicle signal map decides what happens next. Read before anything
    // touches the CAN controller, because canSetMode(SNIFF) programs the
    // hardware filter from the map's ID set.
    // canmap.<vehicle>.txt, found by pattern — see CAN_MAP_PREFIX. The vehicle
    // is named so the log can say WHICH map is live, which a fixed filename
    // could never report and which is the first thing to check when the numbers
    // look like another car's.
    char mapPath[CAN_MAP_NAME_MAX];
    char mapVehicle[CAN_MAP_VEHICLE_MAX];
    const uint8_t mapMatches = canMapFindFile(mapPath, sizeof(mapPath),
                                              mapVehicle, sizeof(mapVehicle));
    if (mapMatches > 1u) {
        // Loaded anyway rather than refused: a second map on the card is an
        // operator slip, and refusing would cost the whole drive's telemetry to
        // punish it. The choice is deterministic and named, so it is checkable.
        Serial.print("CANMAP: ");
        Serial.print(mapMatches);
        Serial.println(" maps on the card; remove the ones you are not using");
    }

    const CanMapStatus ms = (mapMatches == 0u)
        ? CanMapStatus::NOK_NOT_FOUND
        : canMapLoad(canMap, mapPath);
    Serial.print("CANMAP: ");
    Serial.println(canMapStatusName(ms));
    if (ms == CanMapStatus::OK) {
        Serial.print("CANMAP: ");
        Serial.print(mapVehicle);
        Serial.print(" - ");
        Serial.print(canMap.rowCount);
        Serial.print(" signals across ");
        Serial.print(canMap.idCount);
        Serial.print(" ids, id=0x");
        Serial.println(canMap.checksum, HEX);
    } else {
        // NOT "falling back to the compiled-in map", which is what this said and
        // which the very next line contradicted. The compiled-in map is installed
        // below for null-safety only; it is deliberately NOT used to sniff, so
        // the node goes to OBD-II. Say the actionable thing instead — the fix is
        // one file copy, and an operator should not have to read the source to
        // find that out.
        Serial.print("CANMAP: no usable map -> OBD2 only. Copy ");
        Serial.print("config/canmap.<vehicle>.txt to the card root, e.g. ");
        Serial.println("canmap.brio.txt");
    }
    canSniffSetMap(ms == CanMapStatus::OK ? &canMap : nullptr);
    initYawEstimator(yawEst, canSniffGetMap());
    watchdogFeed();

    // Configure-only: the controller comes up off the bus and the mode choice
    // below is what first puts it on. applyCanMode(OBD2) programs its own filter
    // and sets Normal+OSM atomically, so the OBD-II path loses nothing.
    if (!startOBD2(/*stayInConfig=*/true)) {
        obdReady = false;
        canProbeSkip(canProbe);
    } else if (ms != CanMapStatus::OK) {
        // No usable map: OBD-II, immediately, without a probe.
        //
        // The compiled-in map is deliberately NOT used to sniff here. It exists
        // as the Phase 2 A/B artefact and as null-safety for canSniffGetMap();
        // sniffing a hardcoded Honda map on an unknown car would decode another
        // manufacturer's bits and publish the results as measurements. OBD-II is
        // a standard every compliant vehicle answers, so an unconfigured install
        // still produces telemetry - just slower and coarser.
        //
        // And there is nothing to probe FOR: with no map, a ten-second window
        // could only reach the same answer ten seconds later.
        canProbeSkip(canProbe);
        applyCanMode(CanMode::OBD2);
        Serial.println("CAN: no vehicle map; using OBD2 query");
    } else {
        if (applyCanMode(CanMode::SNIFF)) {
            // ARMED, not run. setup() must not hold the boot for the probe
            // window: the C3 link, the IMU drain and the GNSS machine all need
            // loop() passes during it, and the 8 s watchdog would fire long
            // before ten seconds elapsed.
            canProbeArm(canProbe);
        } else {
            // Sniffing refused: fall back to a mode we can verify rather than
            // to a controller in an unknown state.
            canProbeSkip(canProbe);
            applyCanMode(CanMode::OBD2);
            Serial.println("CAN: sniff unavailable; using OBD2");
        }
    }
    lastOBD2Retry = millis();
    initVehicleSignals(vehSignals);
    initYawEstimator(yawEst, canSniffGetMap());

    watchdogFeed();
}

void loop()
{
    // One feed per pass.  Reaching here at all is the health condition the
    // watchdog is testing.
    //
    // "Non-blocking" would overstate it: the GNSS bring-up stage below can hold
    // this pass for up to 750 ms (see gpsInitTick), which is bounded and far
    // under WATCHDOG_PERIOD_MS but long enough to age the IMU FIFO past its
    // freshness window and be reported as a data gap.  Every OTHER branch is
    // non-blocking by construction.
    watchdogFeed();
    // ── 0) CAN mode requests from the Jetson, relayed by the C3 ──────────────
    //
    // Applied HERE, once per pass, rather than inside the frame decoder: a
    // transition passes through Configuration mode and drops frames, which has
    // no business happening in the middle of servicing a command budget.
    if (commMaster.canModeRequest != 0) {
        // The host outranks the boot heuristic. A probe that later "decided" to
        // switch modes out from under an explicit CMD_SET_CAN_MODE would be
        // overriding a human with a guess.
        canProbeSkip(canProbe);
        // The give-up tally is cleared too. It bounds an AUTONOMOUS retry loop;
        // a host that explicitly asks for OBD2 again is entitled to a full fresh
        // budget rather than one attempt before the old count trips it again.
        obdDeadSessions = 0;
        applyCanMode(static_cast<CanMode>(commMaster.canModeRequest));
        // Cleared whether or not it succeeded. A refused mode that stayed
        // latched would be retried every pass forever, and canSetMode()'s rate
        // limiter would reject most of those - producing a steady stream of
        // failures for a request the host made exactly once.
        commMaster.canModeRequest = 0;
    }

    // Filters, applied on the same principle and in the same place: writing the
    // MCP2515's filter registers means dropping into Configuration mode, so the
    // receive gap belongs here rather than inside the frame decoder.
    if (commMaster.canFilterPending) {
        const bool ok = canSniffSetFilters(commMaster.canFilterIds,
                                           commMaster.canFilterCount);
        Serial.print("CAN: host filter set -> ");
        if (!ok) {
            // Named rather than silently dropped. The overwhelmingly likely
            // cause is not being in sniff mode, and a host that sent the command
            // in OBD2 mode would otherwise see no effect and no explanation.
            Serial.println("REFUSED (sniff mode only, or the controller declined)");
        } else if (commMaster.canFilterCount == 0u) {
            // Said out loud because it is the opposite of how it sounds, and on
            // this bus it is a real capacity decision rather than a formality.
            Serial.println("cleared - ACCEPT ALL. The drain holds ~267 frames/s "
                           "against ~1100 arriving, so losses become random.");
        } else {
            Serial.print(commMaster.canFilterCount);
            Serial.print(" ids:");
            for (uint8_t i = 0; i < commMaster.canFilterCount; ++i) {
                Serial.print(" 0x");
                Serial.print(commMaster.canFilterIds[i], HEX);
            }
            Serial.println();
        }
        // Cleared whether or not it succeeded, for the same reason as the mode
        // request above: a refused command retried every pass forever produces a
        // stream of failures for something the host asked once.
        commMaster.canFilterPending = false;
    }

    // ── 0a-ii) Host-requested IMU mode change ────────────────────────────────
    //
    // Applied HERE and not in the frame decoder, because setIMUSampleMode()
    // restarts the whole bring-up: about 700 ms in which the sensor publishes
    // nothing and the trailing peak window is thrown away.
    if (commMaster.imuModeRequest != 0) {
        const IMUSampleMode want = (commMaster.imuModeRequest == COMM_IMU_MODE_RAW)
                                       ? IMUSampleMode::Raw
                                       : IMUSampleMode::Fusion;

        Serial.print("IMU: host requested ");
        Serial.print(want == IMUSampleMode::Raw ? "AMG" : "IMUPLUS");
        Serial.print(" -> ");

        // RATE LIMITED, and this is the whole reason the timestamp exists. Each
        // switch blinds the sensor for ~700 ms, so a host looping on the command
        // would hold it in permanent re-initialisation and it would never
        // produce another sample — every request individually reasonable, the
        // aggregate a denial of the sensor.
        const unsigned long sinceLast = millis() - commMaster.imuModeAppliedMs;
        if (isIMUQuarantined(imuDev)) {
            // Not a refusal to be fixed by asking again: the quarantine is
            // terminal for the boot, and honouring this would re-enter the bus
            // hang it exists to prevent.
            Serial.println("REFUSED (IMU quarantined for this boot)");
        } else if (imuSampleMode(imuDev) == want) {
            // Not an error, and worth saying: a host re-asserting the mode it
            // already has should not be charged a 700 ms blind window for it.
            Serial.println("already in that mode, no change");
        } else if (commMaster.imuModeAppliedMs != 0 && sinceLast < IMU_MODE_MIN_INTERVAL_MS) {
            Serial.print("REFUSED (last change ");
            Serial.print(sinceLast);
            Serial.println(" ms ago)");
        } else if (setIMUSampleMode(imuDev, want) == IMUReturnStatus::OK) {
            commMaster.imuModeAppliedMs = millis();
            // The snapshot describes a sensor that no longer exists in that
            // configuration, so it is cleared rather than left to age out. Its
            // peaks were folded against the OTHER mode's rail.
            initIMUData(imuData);
            Serial.println("bring-up restarted (~700 ms blind)");
        } else {
            Serial.println("REFUSED (driver declined)");
        }

        // Cleared whether or not it was applied, on the same principle as the
        // CAN requests above.
        commMaster.imuModeRequest = 0;
    }

    // ── 0b) Passive decode, whenever a listen-only mode is active ────────────
    //
    // The match counter is read either side so a decoded frame can stamp vehicle
    // liveness. Sniffed traffic is BETTER evidence of a live vehicle than an
    // OBD-II reply: it is passive, arrives at 50-100 Hz, and needs no request.
    // Without this the liveness clock was only ever stamped inside the OBD2
    // poll block, so booting into SNIFF left the vehicle state at Unknown for
    // the whole drive - and the IMU, which picks its sampling mode from that
    // state, never dropped to low power on a parked car.
    const uint32_t matchesBefore = canSniffMatchCount();
    tickCANSniff(vehSignals, yawEst);
    if (canSniffMatchCount() != matchesBefore) {
        vehBusEverLive = true;
        lastEcuReplyMs = millis();
    }

    // ── 0c) Boot-time source decision. Runs to a verdict ONCE, never again ───
    if (canProbe.stage == CanProbeStage::Probing) {
        const CanProbeStage st = canProbeTick(canProbe, millis());
        if (st == CanProbeStage::Sniffing) {
            Serial.print("CAN: probe passed (");
            Serial.print(canSniffMatchCount());
            Serial.println(" matching frames); sniffing this vehicle");
        } else if (st == CanProbeStage::FellBack) {
            // Two different faults needing different repairs, told apart by the
            // frame count: traffic but no matches means the map is for another
            // vehicle; no traffic at all means wrong bit rate, wrong wiring, or
            // a sleeping bus. Reporting both as "no CAN" sends someone looking
            // for a broken cable on a car whose only problem is a config file.
            Serial.print("CAN: probe saw ");
            Serial.print(canSniffMatchCount());
            Serial.print(" matching of ");
            Serial.print(canSniffFrameCount());
            Serial.println(canSniffFrameCount()
                ? " accepted - map is for another vehicle; using OBD2"
                : " accepted - nothing on the bus; using OBD2");
            applyCanMode(CanMode::OBD2);
        }
    }

    // Age each signal on its own source's clock. Sniffed IDs repeat every
    // 10-20 ms so 200 ms of silence is a genuine outage; an OBD-II field is
    // expected to be most of a poll cycle old, and holding it to the sniff
    // window would blank a perfectly healthy fallback mode.
    expireVehicleSignals(vehSignals, millis(), VEH_FRESH_SNIFF_MS, VEH_FRESH_OBD2_MS);

    // ── 1) OBD-II poll, with bounded recovery ────────────────────────────────
    if (obdReady) {
        // The return value is the vehicle-power signal: true means a reply was
        // decoded and stored, which only happens when an ECU answered.  It used
        // to be discarded and the clock stamped unconditionally, which made
        // every pass look like proof of life and kept the vehicle "on" forever.
        if (tickOBD2(OBD2S1Commands, obdData)) {
            vehBusEverLive = true;
            lastEcuReplyMs = millis();
        }
        // Age each reading out on its OWN clock, every pass.  A full sweep of
        // the pipeline takes ~10 x OBD2_TICK_TIMEOUT_MS, so without this a live
        // RPM response certified a speed value most of a second old as current,
        // and COMM_FLAG_OBD2_VALID then vouched for the whole payload.  Speed is
        // the one that matters: it is fed to the acceleration estimator, and a
        // frozen value re-fed repeatedly reads as a genuine deceleration.
        expireStaleOBD2Fields(obdData);
        if (isOBD2LinkLost()) {
            // Defined fallback rather than silently publishing frozen readings:
            // drop the ready flag so the retry path below re-initialises, and
            // let buildTelemetry()'s freshness window clear COMM_FLAG_OBD2_VALID.
            obdReady      = false;
            lastOBD2Retry = millis();
            Serial.println("OBD2: link lost; will re-init");

            // A session that produced at least one reply is a link that WORKS
            // and merely dropped out; one that never did is a link that has
            // never existed. Only the second kind counts toward giving up, and
            // any reply at all resets the tally.
            if (obd2EverReplied()) {
                obdDeadSessions = 0;
            } else if (obdDeadSessions < 0xFFu) {
                ++obdDeadSessions;
            }

            if (obdDeadSessions >= OBD2_DEAD_SESSIONS) {
                // OFF, not another retry. Every one of those retries takes the
                // controller bus-active on a live vehicle to transmit requests
                // that have never once been answered - unattended, indefinitely,
                // for no telemetry. Stopping is the honest outcome, and OFF
                // parks it in Configuration where it emits nothing at all.
                applyCanMode(CanMode::OFF);
                Serial.print("OBD2: no ECU replied in ");
                Serial.print((unsigned)obdDeadSessions);
                Serial.println(" sessions - giving up, CAN now OFF (bus-idle)");
                Serial.println("  This vehicle does not answer Mode 01 at this tap.");
                Serial.println("  Fit a vehicle map (canmap.<vehicle>.txt) to sniff instead,");
                Serial.println("  or send CMD_SET_CAN_MODE from the host to retry.");
            }
        }
    } else if ((canMode == CanMode::OBD2 || canMode == CanMode::OFF) &&
               obdDeadSessions < OBD2_DEAD_SESSIONS &&
               isTimeout(OBD2_RETRY_MS, lastOBD2Retry)) {
        // The tally gate is load-bearing. Giving up sets the mode to OFF, and
        // OFF is one of the modes this branch retries from - so without it the
        // very next pass would re-initialise and the give-up would last five
        // seconds. A controller that never came up at all still retries, because
        // that path never ran a session and never incremented the tally.
        // Gated on the MODE, not on obdReady.
        //
        // This used to read `else if (isTimeout(...))`, which fires precisely
        // BECAUSE obdReady is false — and obdReady is false for the whole of
        // every listen-only session. So five seconds into every sniff the
        // controller was re-initialised into Normal mode with a 0x7E8-only
        // filter, while canMode still reported SNIFF and tickCANSniff() polled
        // a filter that admitted nothing. Sniffing silently stopped working
        // after five seconds and the node went bus-active on a vehicle whose
        // bit timing nothing had confirmed.
        //
        // A comment in applyCanMode() asserted that obdReady=false PREVENTED
        // this. It was exactly backwards, which is why the guard is now a
        // condition rather than a claim.
        //
        // Only the controller comes back here.  Nothing about this success says
        // an ECU is present, so the liveness clock is deliberately NOT stamped:
        // a parked vehicle re-initialises the MCP2515 every 5 s quite happily,
        // and refreshing liveness on each attempt would hold the system awake
        // for as long as it stayed parked.
        // OFF is included deliberately. It means the controller never came up at
        // all, which is exactly the fault this retry exists for — gating on OBD2
        // alone would have made a failed boot permanent.
        const bool wasOff = (canMode == CanMode::OFF);
        obdReady      = startOBD2();
        lastOBD2Retry = millis();

        // Recovering from OFF means the controller is back but no mode has been
        // chosen. Try sniffing again rather than silently settling for OBD2:
        // one unlucky boot used to cost the whole drive, because nothing ever
        // re-attempted SNIFF after setup().
        if (wasOff && obdReady) {
            // Gated on a loaded map, exactly as the boot path is.
            //
            // This used to re-attempt SNIFF unconditionally, which quietly
            // bypassed the whole "no map -> OBD2" policy: a boot whose CAN init
            // failed would recover here and start sniffing with the COMPILED-IN
            // Honda map, on whatever vehicle it happened to be plugged into.
            // Observed doing exactly that. It looked fine because the car was a
            // Brio; on anything else it would have decoded another
            // manufacturer's bits and published them as measurements, which is
            // worse than publishing nothing.
            if (!canMap.loaded) {
                canProbeSkip(canProbe);
                applyCanMode(CanMode::OBD2);
                Serial.println("CAN: controller back, but no vehicle map; using OBD2 query");
            } else if (applyCanMode(CanMode::SNIFF)) {
                // Probe armed, for the same reason the boot arms it: this is the
                // first time the map has met the bus, so it is still unproven.
                canProbeArm(canProbe);
            } else {
                obdReady = true;   // applyCanMode() left state untouched on failure
                Serial.println("CAN: sniff still unavailable; OBD2 poller armed");
            }
        }
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
                // The ONE place the heading filter is fed: one packet, one
                // sample.  OK means a fix; NO_FIX still counts as the receiver
                // talking, and updateHeading() resets the window for it.
                updateHeading(st == GPSReturnStatus::OK);
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
            // Discard the course window too.  It holds pre-outage samples, and
            // a receiver that returns after a gap may well be pointing a
            // different way — re-using them would certify a course from before
            // the outage as current.
            headingFilter.reset();
            derived.headingDeg = NAN;
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
        //
        // Not attempted at all while a line is clamped low. Driving transactions
        // into a bus whose SDA never moves is not merely futile - if the clamp
        // is a latched-up device, every attempt pushes current into it, and the
        // MKR's own pins are on the other end of that. The receiver may be
        // perfectly healthy and simply unreachable; saying so once beats 11
        // identical status -7 lines that all describe the OTHER device's fault.
        if (i2cStuckLines() & I2C_STUCK_SDA_NEVER_MOVED) {
            if (!gpsBusBlockedLogged) {
                gpsBusBlockedLogged = true;
                Serial.println("GPS: not attempted - I2C SDA is clamped low; free the bus first");
            }
            gpsReady = false;
        } else {
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
        }   // bus not clamped
    }

    // Hardware presence for the wire, kept in step with the retry state above.
    gpsData.devicePresent = gpsReady;
#endif

    // ── 2c) IMU poll, with bounded recovery ──────────────────────────────────
    // Unconditional on the timer: getIMUData() returns immediately without
    // touching the bus when a device is down, so there is nothing to gate on.
    // EVERY pass, not on the poll timer. The bring-up has about a dozen steps
    // and each is a few hundred microseconds; running them at 20 Hz would add
    // half a second to a sequence whose real cost is the part's own reset delay.
    // A no-op once configured, so the unconditional call costs one comparison.
    (void)imuInitTick(imuDev);

    if (isTimeout(IMU_POLL_MS, lastIMUPoll)) {
        lastIMUPoll = millis();
        (void)getIMUData(imuDev, imuData);   // status is carried by the data's own valid flags
    }

    // ── 2b-ii) Panel switches ────────────────────────────────────────────────
    //
    // ~80 us of GPIO toggling every 20 ms, on three pins shared with nothing.
    // The return is deliberately discarded: a false is the ordinary report of a
    // chain that is not fitted, and the state it describes is carried by
    // SwitchData::present, which the edge log below reads. Escalating "no
    // switch panel" every 20 ms would bury the log for a build that simply does
    // not have one.
    //
    // The debounce window is counted in POLLS, so this interval is part of the
    // 80 ms figure in DataDictionary.h rather than an independent choice.
    if (isTimeout(SWITCH_POLL_MS, lastSwitchPoll)) {
        lastSwitchPoll = millis();
        (void)pollSwitches(switches);
        logSubsystemEdge("SW", switches.present, prevSwitchesUp);
    }

    // ── 2c-i) Persist a calibration once it is fully converged ───────────────
    //
    // Only at 3/3. A partial profile is worse than none: it would be restored at
    // every subsequent boot and would anchor the fusion to a half-finished
    // estimate, which it would then have to climb back out of. Waiting for full
    // convergence means the file is written rarely and is worth having when it is.
    //
    // NOT during a High-G event. Capturing costs about 60 ms with the sensor in
    // CONFIG producing nothing, and the one moment that must never have a hole
    // in it is an impact — which is precisely when this would otherwise fire, as
    // a hard jolt is also what finally moves the accelerometer figure to 3.
    //
    // AND ONLY WHILE THE SENSOR IS QUIET. The capture spends ~60 ms in CONFIG
    // producing nothing AND with the High-G comparator inactive, so it is a true
    // blind window — the one interval where neither the poll nor the hardware
    // backstop is watching. Excluding an active High-G event is not enough,
    // because that only covers an impact already detected. Requiring the peak
    // over the last window to be near gravity means nothing is happening at all,
    // which is a stronger and entirely self-contained test — no vehicle speed,
    // no ignition state, nothing that can be wrong about the car.
    const bool imuQuiet = !isnan(imuData.linAccelPeakMs2) &&
                          imuData.linAccelPeakMs2 < IMU_CALIB_SAVE_QUIET_MS2;

    if (imuIsReady(imuDev) && sdReady() && !imuData.highGEvent && imuQuiet &&
        imuData.calibGyro >= 3u && imuData.calibAccel >= 3u &&
        (lastCalibSaveMs == 0 || isTimeout(IMU_CALIB_SAVE_INTERVAL_MS, lastCalibSaveMs))) {

        uint8_t fresh[BNO055_CALIB_BYTES];
        if (!bno055CalibCapture(imuDev.init, fresh)) {
            // The capture verifies both mode transitions, so a failure means the
            // part may be STRANDED IN CONFIG: answering every transaction,
            // devicePresent true, and producing no data at all. Nothing else
            // detects that — the frozen-data check would eventually fire, a
            // second at a time, on a sensor we already know is wrong.
            Serial.println("IMU: calibration capture FAILED - mode may not have been restored.");
            Serial.println("     Re-running bring-up: a part left in CONFIG answers normally");
            Serial.println("     and reports nothing, which no other check catches quickly.");
            (void)initializeIMU(imuDev, imuSampleMode(imuDev));
            lastCalibSaveMs = millis();   // do not retry every pass
        } else if (imuCalibProfileValid && bno055CalibEqual(fresh, imuCalibProfile)) {
            // Identical to what is already stored. Skipped silently and the
            // timer reset, so a converged sensor does not rewrite the same 22
            // bytes every ten minutes for the life of the vehicle.
            lastCalibSaveMs = millis();
        } else {
            const bool ok = bno055CalibStore(fresh, BNO055_CALIB_INSTALL_ID);
            if (ok) {
                memcpy(imuCalibProfile, fresh, sizeof(fresh));
                imuCalibProfileValid = true;
                char desc[80];
                bno055CalibDescribe(fresh, desc, sizeof(desc));
                Serial.print("IMU: calibration saved - ");
                Serial.println(desc);
            } else {
                Serial.println("IMU: calibration save FAILED - previous profile left intact");
            }
            lastCalibSaveMs = millis();
        }
    }
    // Driven by isIMUDegraded(), not isIMULinkLost(). The distinction survives
    // the move to a single chip: degraded now means "configured, then stopped
    // delivering usable data", which includes a frozen data path on a part that
    // is still answering every transaction — the failure that retired the last
    // sensor. isIMUDegraded() is false while bring-up is still running, so this
    // cannot restart a sequence that is merely part-way through.
    // A clamped SDA stops the retry entirely rather than slowing it.
    //
    // This is protective, not cosmetic. A line held at a diode drop with the
    // rail good is a LATCHED-UP device — a parasitic SCR conducting between the
    // rails — and it clears only when the supply is fully discharged. Confirmed
    // on this bench: the part came back after a night disconnected, so it was
    // never destroyed. What keeps a latch alive is current, and every recovery
    // attempt drives the bus and feeds it. Retrying every five seconds for a
    // whole drive is how a recoverable latch-up becomes permanent damage.
    //
    // Nothing is lost by stopping: no amount of clocking clears a latch, and the
    // fix is a power cycle the firmware cannot perform.
    if (isIMUDegraded(imuDev) &&
        (i2cStuckLines() & I2C_STUCK_SDA_NEVER_MOVED) != 0u) {
        if (!imuLatchLogged) {
            imuLatchLogged = true;
            Serial.println("IMU: SDA clamped with the rail good - this is LATCH-UP, not a dead chip.");
            Serial.println("     Retries STOPPED: driving the bus feeds the latch and can make it");
            Serial.println("     permanent. Fully power down the rig (drain it, seconds are not");
            Serial.println("     enough) and the part will come back. Then fix what triggers it:");
            Serial.println("     I2C driven while the breakout's VIN is absent or sagging.");
        }
    } else if (isIMUDegraded(imuDev) && isTimeout(IMU_RETRY_MS, lastIMURetry)) {
        imuLatchLogged = false;   // bus is free again; a real fault may still recover
        lastIMURetry = millis();

        // The preserved evidence, printed ONCE per failure episode.
        //
        // Not on every retry: the retry line below already repeats, and a block
        // this size repeating every five seconds would bury the one thing worth
        // reading. Not at the moment of failure either — the first failure often
        // happens during setup() before anyone is watching the console, and this
        // way it is still on screen when someone connects.
        printIMUFailureRecord();
        // Outcome logged, not discarded: an IMU that drops off mid-drive was
        // previously retried forever in complete silence, so the console gave
        // no hint that anything had changed.
        const IMUReturnStatus rst = recoverIMU(imuDev);
        // Logged on EVERY attempt, matching the OBD2 and GPS retry paths.  A
        // module that is simply not fitted should keep saying so: silence would
        // read as "fine" to anyone scanning the log.
        Serial.print("IMU: retry status=");
        Serial.print(static_cast<int>(rst));
        Serial.print(" stage=");
        Serial.print(bno055InitStageName(imuDev.init.stage));
        Serial.print(" why=");
        Serial.print(bno055InitStatusName(imuDev.init.lastStatus));
        // A stuck bus is not an IMU fault and must not read as one. Status -5
        // repeated twenty times says only "recovery failed"; naming the line
        // says which fault it is, and the GNSS shares this bus so the same
        // answer explains both devices going quiet together.
        if (rst == IMUReturnStatus::NOK_BUS_STUCK) {
            Serial.print("  I2C: ");
            Serial.print(i2cStuckReason());
        }
        Serial.println();
    }

    // ── 2b) Derived signals ──────────────────────────────────────────────────
    // Sampled on a fixed cadence, not on ECU updates: see ACCEL_SAMPLE_MS.
    if (isTimeout(ACCEL_SAMPLE_MS, lastAccelSample)) {
        lastAccelSample = millis();
        // Sniffed speed first, OBD-II second — the same precedence buildTelemetry()
        // applies to the primary `speed` field, and for the same reason.
        //
        // This fed obdData.speed unconditionally, which meant SNIFF mode had NO
        // acceleration at all: obdData.speed is NAN whenever the poller is not
        // running, so the estimator was handed nothing for the entire sniffing
        // session and derived.accelMs2 stayed NAN. Sniffed speed is also the
        // better input by a wide margin — 0.01 km/h at 50-100 Hz against whole
        // km/h at ~2 Hz — and a differentiator is exactly where that resolution
        // pays, because quantisation noise is what a difference amplifies.
        const float accelInput =
            (vehSignals.speedSrc != VehSource::NONE && !isnan(vehSignals.speedKmh))
                ? vehSignals.speedKmh
                : obdData.speed;
        (void)accelEst.update(accelInput, lastAccelSample); // NAN handled inside
        derived.accelMs2 = accelEst.value();
        // false: this tick may only INVALIDATE. Feeding here is what
        // oversampled a 4 Hz receiver at 10 Hz; see updateHeading().
        updateHeading(false);
    }

    // ── 3) Serve the ESP32-C3 link ───────────────────────────────────────────
    // Non-blocking: services queued commands and pushes the 10 Hz stream.
    // Must run every pass — this is the only thing that answers the bridge.
    tickCommMaster(commMaster, obdData, gpsData, imuData, switches, derived, vehSignals, (uint8_t)canMode);

    // ── 3b) Subsystem transitions ────────────────────────────────────────────
    // Every module is optional at runtime: any of them can be absent at boot or
    // vanish mid-drive, and the system keeps producing telemetry either way
    // (missing values are NAN with their *_PRESENT flag clear).  What must never
    // happen is that it changes silently.
    logSubsystemEdge("OBD2", obdReady, prevOBDUp);
    logSubsystemEdge("IMU", imuIsReady(imuDev), prevIMUUp);

    // The old two-chip breakout could report exactly one device answering, which
    // was a POWER fault rather than partial data — an unpowered I2C slave keeps
    // acknowledging on parasitic current through the bus pull-ups, so the
    // lighter-draw part went on replying while the other died. One chip cannot
    // produce that state, so the warning is gone with it.
    //
    // What takes its place is the fused output not yet being trustworthy: the
    // part is up and publishing linear acceleration whose calibration has not
    // converged. Those values are not wrong-looking, merely not yet right, and
    // nothing downstream could otherwise tell.
    const bool imuFusionUnready =
        imuIsReady(imuDev) && (imuData.calibGyro < IMU_CALIB_MIN_GYRO);
    if (imuFusionUnready != prevIMUDegraded) {
        prevIMUDegraded = imuFusionUnready;
        if (imuFusionUnready) {
            Serial.println("IMU: fusion output not yet trustworthy - gyro calibration converging");
            Serial.println("     Reaches 3 within seconds of standing still. The accelerometer");
            Serial.println("     figure is reported but does NOT gate this: on the bench it sat");
            Serial.println("     at 0 for 9000 polls while gravity held 9.79-9.81, so it was not");
            Serial.println("     measuring trustworthiness. The gravity gate in the driver is.");
        } else {
            Serial.println("IMU: fusion calibrated");
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
        // Vehicle signals, from whichever source is live. These used to read
        // obdData only, which showed "--" for the whole of a healthy sniffing
        // session because sniffed values land in vehSignals instead.
        Serial.print("spd=");
        if      (!isnan(vehSignals.speedKmh)) Serial.print(vehSignals.speedKmh, 2);
        else if (!isnan(obdData.speed))       Serial.print(obdData.speed, 1);
        else                                  Serial.print("--");
        Serial.print(" rpm=");
        if      (!isnan(vehSignals.rpm)) Serial.print(vehSignals.rpm, 0);
        else if (!isnan(obdData.rpm))    Serial.print(obdData.rpm, 0);
        else                             Serial.print("--");

        // Held indicator state, so this reads steadily while indicating rather
        // than flickering with the lamp. Both arrows at once is hazards.
        // "<>" is now driven by the hazard SIGNAL, not by both turn bits being
        // set. On this vehicle the hazards leave both turn bits clear, so the
        // old test could never fire — which is exactly how it was found.
        Serial.print(" turn=");
        if      (vehSignals.hazard)                           Serial.print("<>");
        else if (vehSignals.turnLeft && vehSignals.turnRight) Serial.print("<>");
        else if (vehSignals.turnLeft)                         Serial.print("<-");
        else if (vehSignals.turnRight)                        Serial.print("->");
        else if (vehSignals.turnSrc == VehSource::NONE)       Serial.print("--");
        else                                                  Serial.print("..");

        // The MODE, not obdReady. "obd=down" during a healthy listen-only
        // session reads as a fault and is not one — obdReady is false because
        // we are deliberately not transmitting. Printing the mode says which of
        // the three the node is actually in, which is the thing worth knowing.
        Serial.print("  can=");
        Serial.print(canModeName(canMode));
        // matched/total frames off the drain. Cheap, and it settles a question
        // no amount of datasheet reading does: if the hardware filter is doing
        // its job these two track each other, because only mapped IDs are
        // admitted. A matched count that is a small fraction of the total means
        // the filter is being bypassed and the drain is spending its budget on
        // traffic it throws away — which is the difference between "sniffing
        // works" and "sniffing works until the bus gets busy".
        if (canMode == CanMode::SNIFF) {
            Serial.print(" rx=");
            Serial.print(canSniffMatchCount());
            Serial.print("/");
            Serial.print(canSniffFrameCount());
        }
        // Three states, not two. obdReady only says the CONTROLLER came up;
        // printing that as "/up" claims a working diagnostic link on a vehicle
        // that may never have answered, which reads as "OBD-II is fine, the
        // decode must be broken" when the truth is the opposite. "/noecu" is
        // the case where we are transmitting into silence.
        if (canMode == CanMode::OBD2) {
            if      (!obdReady)         Serial.print("/down");
            else if (obd2EverReplied()) Serial.print("/up");
            else                        Serial.print("/noecu");
        }
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
        // Four states. "configuring" is new and load-bearing: bring-up now takes
        // about 700 ms of wall clock, and without it the first console lines of
        // every boot would report a healthy IMU as "down".
        Serial.print("  imu=");
        if (isIMUQuarantined(imuDev))       Serial.print("QUARANTINED");
        else if (imuIsReady(imuDev))        Serial.print("up");
        else if (imuDev.init.stage != BNO055InitStage::Failed)
                                            Serial.print("configuring");
        else                                Serial.print("down");
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
        // The peak that actually drives incident detection, and the one the
        // previous hardware could not produce: gravity already removed, so a
        // stationary vehicle reads ~0 here while |a| beside it reads 9.81.
        Serial.print(" lin=");
        if (isnan(imuData.linAccelPeakMs2)) Serial.print("--");
        else                                Serial.print(imuData.linAccelPeakMs2, 2);
        // SAT is not a detail. Fusion locks the accelerometer at ±4 g, so a
        // clipped peak understates a real collision fivefold, and a saturated
        // number printed plainly is indistinguishable from a measured one.
        if (imuData.accelSaturated) Serial.print(" SAT");
        // The hardware latch. Distinct from a high peak, and the distinction is
        // the point: a peak is only ever as good as the polls that produced it,
        // while this survived whatever the loop was doing at the time.
        if (imuData.highGEvent) Serial.print(" HIGH-G");
        else if (!imuData.highGArmed && imuIsReady(imuDev)) Serial.print(" nohg");
        Serial.print(" mode=");
        Serial.print(imuData.fusionMode ? "fus" : "amg");
        Serial.print(" cal=");
        Serial.print(imuData.calibGyro);
        Serial.print(imuData.calibAccel);
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
        Serial.print(imuDev.ioErrors);
        // Bursts that arrived intact and failed a plausibility gate, counted
        // apart from bus errors because they mean the opposite thing: the part
        // answered perfectly and the CONTENTS were impossible. That is how the
        // previous IMU failed — 40 % of its samples corrupt, every transaction a
        // success — and no bus-level counter can see it.
        Serial.print("/");
        Serial.print(imuDev.implausible);
        // CUMULATIVE gap counts, hardware overrun / freshness discard.  The
        // GAP flag above lasts only IMU_PEAK_WINDOW_MS, so a soak watching the
        // console can miss every one of them and still look clean — which is
        // exactly what a 150 s run showing no GAP proved, and did not prove.
        // These only ever climb, so one glance answers "were there any?".
        Serial.print(" gaps=");
        Serial.print(imuDev.missedPolls);
        // Cumulative High-G latches. The published flag lasts IMU_HIGHG_HOLD_MS,
        // so a soak watching the console can miss every one of them and still
        // look clean. This only climbs, so one glance answers "were there any?".
        Serial.print(" hg=");
        Serial.print(imuDev.highGCount);
        // c3= is the LINK, stream= is the session on top of it.  They are
        // different failures: a bridge that is attached but not asking for
        // telemetry is healthy, one that has been unplugged is not, and
        // stream=off alone cannot tell them apart.  This also covers the case
        // the edge log cannot — a module absent from boot never transitions, so
        // without a continuous readout it would never appear anywhere.
        Serial.print("  c3=");
        Serial.print(isCommLinkSilent(commMaster, COMM_LINK_SILENT_MS) ? "down" : "up");
        Serial.print(" stream=");
        Serial.print(commMaster.streaming ? "on" : "off");

        // Switch panel, on the same principle as c3= above: a chain absent from
        // boot never transitions, so the edge log alone would never mention it.
        // Positions are printed as a hex word rather than as named signals
        // because this firmware does not know what any of them mean, and a
        // console label would be the first place that knowledge crept in.
        Serial.print("  sw=");
        if (!switches.present) {
            Serial.print("absent");
        } else {
            Serial.print("0x");
            Serial.print(switches.state, HEX);
            // Only when non-zero. A chatter word of 0 on every line trains the
            // eye to skip the field, which is where the one that matters would
            // then also be skipped.
            if (switches.chatter != 0u) {
                Serial.print(" CHATTER=0x");
                Serial.print(switches.chatter, HEX);
            }
            // Cumulative, so one glance answers "were there any?" over a soak —
            // the same reason gaps= and hg= are printed above rather than only
            // being flagged while they are current.
            if (switches.rejected != 0u) {
                Serial.print(" rej=");
                Serial.print(switches.rejected);
            }
        }
        Serial.println();
        lastDebugPrint = millis();
    }
}
