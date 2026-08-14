/**
 * @file IMUValidation.ino
 * @brief Phase 3's gate: proves the BNO055 is producing data worth believing.
 *
 * Bring-up succeeding says the part accepted its configuration. It says nothing
 * about whether the numbers coming out mean anything, and on this project that
 * gap is not theoretical — the sensor this replaces spent 40 % of its samples
 * returning a fixed 27.8 m/s2 and 104 C from transactions that all succeeded.
 * A driver that reports "OK" is exactly what that failure looked like.
 *
 * So this sketch checks the readings against physics rather than against status
 * codes:
 *
 *   GRAVITY    |g| must be about 9.81 whatever the board is doing. The fusion
 *              CONSTRUCTS this vector, so its length is near-constant —
 *              rotating the board moves its direction, not its size. This is
 *              the strongest single check available, and the previous hardware
 *              could not offer it at all.
 *   LINEAR     |linear a| must be about 0 on a still bench. It is acceleration
 *              with gravity already removed, so anything else means either
 *              motion or a fusion that has not converged.
 *   SUM        linear + gravity must reconstruct the raw accelerometer vector.
 *              That is the fusion's own internal consistency, checked from
 *              outside.
 *   PEAK       a tap on the desk must move the peak while the instantaneous
 *              magnitude beside it stays near 9.81. That difference IS the
 *              feature: the peak sees samples the published frame does not.
 *
 * Commands:  c calibration   z zero the stats   r restart bring-up
 *            a AMG mode      i IMUPLUS mode     b bus survey
 *            s inject a 500 ms loop stall  <- the Phase 4 gate
 *            w save calibration to SD      <- the Phase 6 gate
 *            l show the stored profile
 */

#include <Wire.h>

#include "IMUFunctions.h"
#include "BNO055Calib.h"
#include "SDFunctions.h"
#include "I2CBus.h"

/// Loaded at boot and handed to the bring-up. File scope because the bring-up
/// state holds a POINTER to it and dereferences it from loop().
static uint8_t gCalibProfile[BNO055_CALIB_BYTES];
static bool    gCalibValid = false;

/// Static storage duration is REQUIRED: the Bosch driver keeps a pointer to
/// dev.init.dev, so an automatic here would leave it dangling.
static IMUDevice gDev;
static IMUData   gData;

static uint32_t gPolls      = 0;
static uint32_t gOk         = 0;
static uint32_t gPartial    = 0;
static uint32_t gGravityBad = 0;
static uint32_t gLinearBad  = 0;
static uint32_t gSumBad     = 0;

static float gGravityMin =  1e9f;
static float gGravityMax = -1e9f;
static float gLinearMax  =  0.0f;

static BNO055InitStage gLastStage = BNO055InitStage::Idle;

/// Tolerance on |gravity| for the PASS verdict (m/s2). Tighter than the driver's
/// own plausibility gate on purpose: that one is a discard threshold sized not
/// to fire while the algorithm converges, this one is a quality bar.
static const float GRAVITY_NOMINAL_MS2 = 9.81f;
static const float GRAVITY_TOLERANCE   = 0.5f;
/// Largest |linear a| a bench that is not being touched should ever show.
static const float LINEAR_STILL_MAX    = 0.6f;
/// Largest |w| that still counts as stationary (deg/s). A hand-held board that
/// is "not moving" drifts a degree or two a second; a deliberate tilt is tens.
static const float GYRO_STILL_MAX      = 3.0f;
/// Tolerance on |raw - (linear + gravity)|, the fusion's internal consistency.
static const float SUM_TOLERANCE       = 0.7f;
/// Consecutive quiet samples before the sum check is believed. At 100 Hz this
/// is a fifth of a second of stillness, comfortably past the fusion's catch-up.
static const uint8_t QUIET_RUN_REQUIRED = 20u;
static uint8_t gQuietRun = 0;

static void onHighG(){
    imuNoteHighGPin(millis());
}

static float magnitude(float x, float y, float z){
    return sqrtf((x * x) + (y * y) + (z * z));
}

static void printFloat(float v, uint8_t dp){
    if (isnan(v)) Serial.print("--");
    else          Serial.print(v, dp);
}

static void busSurvey(){
    I2CBusReport report;
    const IMUReturnStatus st = checkI2CBusConflict(report);

    Serial.println();
    Serial.println(F("---- bus survey ----"));
    Serial.print(F("  responders: "));
    Serial.println(report.deviceCount);
    for (uint8_t i = 0u; i < report.deviceCount && i < I2C_SCAN_MAX_DEVICES; i++){
        Serial.print(F("    0x"));
        Serial.print(report.addresses[i], HEX);
        if (report.addresses[i] == GPS_DEFAULT_I2C_ADDRESS) Serial.print(F("  u-blox GNSS"));
        if (report.addresses[i] == SEGLED_ADDRESS)          Serial.print(F("  segment LED"));
        if (report.addresses[i] == report.imuAddress)       Serial.print(F("  BNO055"));
        Serial.println();
    }
    Serial.print(F("  IMU: "));
    if (report.imuIdentified){
        Serial.print(F("identified at 0x"));
        Serial.print(report.imuAddress, HEX);
        Serial.println(F(" by CHIP_ID 0xA0"));
    } else if (report.conflict){
        // Identified by CHIP_ID, never by a bare ACK: something else can sit at
        // 0x28 or 0x29, and configuring whatever answered is how a bus conflict
        // turns into a sensor that reports plausible nonsense.
        Serial.println(F("*** SOMETHING ANSWERS AT AN IMU ADDRESS BUT IS NOT A BNO055 ***"));
    } else {
        Serial.println(F("absent"));
    }
    Serial.print(F("  status: "));
    Serial.println(static_cast<int>(st));
    Serial.println();
}

static void printCalibration(){
    Serial.print(F("calib  gyro "));  Serial.print(gData.calibGyro);
    Serial.print(F("  accel "));      Serial.print(gData.calibAccel);
    Serial.print(F("  mag "));        Serial.print(gData.calibMag);
    Serial.print(F("  sys "));        Serial.print(gData.calibSys);
    if (gData.fusionMode){
        // Said out loud every time, because two zeros here look like a fault and
        // are not one: fusion mode switches the magnetometer off, and the system
        // figure cannot rise without it.
        Serial.print(F("   (mag and sys stay 0 in IMUPLUS - the magnetometer is off)"));
    }
    Serial.println();

    // The accelerometer figure needs a procedure, not patience, and the
    // difference is not obvious from a number that simply refuses to move.
    // Bosch raises it from STILL HOLDS in distinct orientations; vibration
    // actively prevents it. On the bench it is routinely mistaken for a fault
    // because the natural thing to do while watching an IMU is tap the desk —
    // which is exactly the input that keeps it at 0.
    if (!gCalibValid && imuIsReady(gDev) && gData.calibAccel < 3u){
        Serial.println(F("   accel < 3: rest the board on each of its 6 faces, ~4 s each,"));
        Serial.println(F("   moving SLOWLY between. Do not tap or shake - that undoes it."));
        Serial.println(F("   At 3/3 press 'w' to store the profile. Do this BEFORE mounting:"));
        Serial.println(F("   a bolted-in board never sees six orientations again."));
    }
}

static void resetStats(){
    gPolls = 0; gOk = 0; gPartial = 0;
    gGravityBad = 0; gLinearBad = 0; gSumBad = 0;
    gGravityMin =  1e9f;
    gGravityMax = -1e9f;
    gLinearMax  =  0.0f;
}

static void restart(IMUSampleMode mode){
    resetStats();
    initIMUData(gData);
    gLastStage = BNO055InitStage::Idle;
    // Offered on every restart, so 'r' after a save exercises the restore path
    // rather than only the capture path — the two fail differently and the
    // second is the one that runs in the vehicle.
    bno055InitSetCalibProfile(gDev.init, gCalibValid ? gCalibProfile : nullptr);
    const IMUReturnStatus st = initializeIMU(gDev, mode);
    Serial.println();
    Serial.print(F("Bring-up starting in "));
    Serial.print(mode == IMUSampleMode::Raw ? F("AMG (raw)") : F("IMUPLUS (fusion)"));
    Serial.print(F(", armed="));
    Serial.println(static_cast<int>(st));
}

void setup(){
    Serial.begin(115200);
    const uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 3000) { }

    watchdogArm(8000UL);

    Serial.println();
    Serial.println(F("============ BNO055 data validation ============"));
    Serial.println(F("Phase 3 gate: the readings must agree with physics."));
    Serial.println();

    // The clock is NOT set here. i2cBusBegin() runs i2cBusRecover() on the first
    // transaction from any client, and that ends with Wire.begin() followed by
    // Wire.setClock(IMU_I2C_CLOCK_HZ) — so a rate set in setup() is overwritten
    // before the first register is read. Every earlier bench run printed 100 kHz
    // and ran at 400. One owner for the bus clock, and it is not this sketch.
    Wire.begin();

    // The INT pin is optional: the latch is read over I2C on every poll
    // regardless, so an unwired pin costs only latency, not the feature.
    pinMode(IMU_HIGHG_INT_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(IMU_HIGHG_INT_PIN), onHighG, RISING);

    Serial.print(F("I2C clock: "));
    Serial.print(IMU_I2C_CLOCK_HZ / 1000UL);
    Serial.print(F(" kHz   poll: "));
    Serial.print(IMU_POLL_MS);
    Serial.print(F(" ms   High-G INT: D"));
    Serial.println(IMU_HIGHG_INT_PIN);

    if (initializeSD() == SDReturnStatus::OK) {
        Serial.println(F("SD: mounted"));
        uint8_t chip = 0;
        if (bno055CalibLoad(gCalibProfile, &chip)) {
            if (chip != BNO055_EXPECTED_CHIP_ID) {
                Serial.print(F("calibration on card is from chip 0x"));
                Serial.print(chip, HEX);
                Serial.println(F(" - IGNORED"));
            } else {
                gCalibValid = true;
                char desc[80];
                bno055CalibDescribe(gCalibProfile, desc, sizeof(desc));
                Serial.print(F("calibration profile on card: "));
                Serial.println(desc);
            }
        } else {
            Serial.println(F("no calibration profile on card - press 'w' at 3/3 to make one"));
        }
    } else {
        Serial.println(F("SD: no card - calibration cannot be saved or restored"));
    }

    busSurvey();
    restart(IMUSampleMode::Fusion);
}

void loop(){
    watchdogFeed();

    const BNO055InitStage stage = imuInitTick(gDev);
    if (stage != gLastStage){
        gLastStage = stage;
        Serial.print(F("  bring-up: "));
        Serial.print(bno055InitStageName(stage));
        if (stage == BNO055InitStage::Failed){
            Serial.print(F("  at "));
            Serial.print(bno055InitStageName(gDev.init.failedAt));
            Serial.print(F("  why "));
            Serial.print(bno055InitStatusName(gDev.init.lastStatus));
        }
        Serial.println();
        if (stage == BNO055InitStage::Configured){
            Serial.print(F("  address 0x"));  Serial.print(gDev.init.address, HEX);
            Serial.print(F("  clock "));
            Serial.print(gDev.init.externalCrystal ? F("external") : F("internal"));
            Serial.print(F("  euler "));
            Serial.print(gDev.eulerAndroid ? F("Android") : F("Windows"));
            Serial.print(F("  calib-restore "));
            if (!gDev.init.calibOffered)      Serial.println(F("none offered"));
            else if (gDev.init.calibRestored) Serial.println(F("OK"));
            else                              Serial.println(F("*** OFFERED BUT FAILED ***"));
            if (gDev.init.calibRestored){
                // Said here because the next thing that happens looks like a
                // failure and is not. The figures start at 3/3 — which is the
                // restore working — and then DECAY as the part runs. Bosch's
                // CALIB_STAT is a live confidence estimate, not a record of what
                // was loaded: sitting still gives the algorithm nothing to
                // confirm the accelerometer against, so its confidence falls
                // while the loaded offsets stay in force and keep working.
                //
                // This is why the accelerometer figure does not gate the fused
                // output. Trusting it would put the system back to reporting
                // PARTIAL forever, minutes after a successful restore.
                Serial.println(F("  (3/3 now; the accel figure will decay as the algorithm"));
                Serial.println(F("   re-estimates. The offsets stay applied - that is normal.)"));
            }
            Serial.println();
        }
    }

    static uint32_t lastPoll = 0;
    if (imuIsReady(gDev) && (millis() - lastPoll) >= IMU_POLL_MS){
        lastPoll = millis();
        const IMUReturnStatus st = getIMUData(gDev, gData);
        gPolls++;
        if (st == IMUReturnStatus::OK)      gOk++;
        if (st == IMUReturnStatus::PARTIAL) gPartial++;

        if (gData.fusionValid){
            const float g = magnitude(gData.gravityX, gData.gravityY, gData.gravityZ);
            const float l = magnitude(gData.linAccelX, gData.linAccelY, gData.linAccelZ);

            if (g < gGravityMin) gGravityMin = g;
            if (g > gGravityMax) gGravityMax = g;
            if (l > gLinearMax)  gLinearMax  = l;

            if (fabsf(g - GRAVITY_NOMINAL_MS2) > GRAVITY_TOLERANCE) gGravityBad++;
            if (l > LINEAR_STILL_MAX) gLinearBad++;

            // The fusion's own arithmetic, checked from outside: whatever it
            // decides gravity and linear acceleration are, they have to add back
            // up to what the accelerometer actually measured.
            //
            // ONLY WHILE QUASI-STATIC, and that restriction is a correction. The
            // identity holds at rest and breaks under motion, because the fused
            // vectors are the algorithm's output at the fusion rate while the
            // raw accelerometer register is the sensor's own latest conversion —
            // during a fast transient the fusion is still catching up, so the
            // three are simply not the same instant. Checking it regardless
            // produced 333 failures in 1682 polls, every one of them during a
            // deliberate bench tap, and none of them a fault. At rest, where the
            // identity does hold, the failure count was exactly zero.
            // SUSTAINED quiet, not merely quiet at this instant. A tap's decay
            // passes back down through the threshold while the fusion is still
            // catching up, so an instantaneous test still caught the tail of
            // every transient — 3 failures in 801 polls, all of them at the
            // boundary, none of them a fault. Quasi-static means quiet for a
            // while, and the check now says so.
            // Quiet means NOT TRANSLATING AND NOT ROTATING. The rotation half
            // was missing and this run exposed it: during the slow tilts that
            // accelerometer calibration requires, linear acceleration stays
            // small — a slow move produces almost none — while the gravity
            // VECTOR swings right through the sensor frame. The fusion lags that
            // rotation, so the identity breaks for the same reason it breaks
            // during a tap, and the sum count crept 2 -> 6 -> 9 with the bench
            // apparently still. A quiet test that ignores the gyroscope is not
            // testing quiet.
            const float w = magnitude(gData.gyroX, gData.gyroY, gData.gyroZ);
            if (l <= LINEAR_STILL_MAX && w <= GYRO_STILL_MAX) {
                if (gQuietRun < 255u) gQuietRun++;
            } else {
                gQuietRun = 0u;
            }

            if (gData.accelValid && (gQuietRun >= QUIET_RUN_REQUIRED)){
                const float sx = gData.linAccelX + gData.gravityX - gData.accelX;
                const float sy = gData.linAccelY + gData.gravityY - gData.accelY;
                const float sz = gData.linAccelZ + gData.gravityZ - gData.accelZ;
                if (magnitude(sx, sy, sz) > SUM_TOLERANCE) gSumBad++;
            }
        }
    }

    static uint32_t lastReport = 0;
    if (imuIsReady(gDev) && (millis() - lastReport) >= 2000UL){
        lastReport = millis();

        Serial.print(F("|a|="));
        printFloat(gData.accelValid ? magnitude(gData.accelX, gData.accelY, gData.accelZ) : NAN, 2);
        Serial.print(F("  |grav|="));
        printFloat(gData.fusionValid ? magnitude(gData.gravityX, gData.gravityY, gData.gravityZ) : NAN, 2);
        Serial.print(F("  |lin|="));
        printFloat(gData.fusionValid ? magnitude(gData.linAccelX, gData.linAccelY, gData.linAccelZ) : NAN, 2);
        Serial.print(F("  pk="));
        printFloat(gData.accelPeakMs2, 2);
        Serial.print(F("/"));
        printFloat(gData.linAccelPeakMs2, 2);
        Serial.print(F("/"));
        printFloat(gData.gyroPeakDps, 1);
        if (gData.accelSaturated) Serial.print(F(" SAT"));
        if (gData.highGEvent)     Serial.print(F(" HIGH-G"));
        Serial.print(F("  hg="));
        Serial.print(gDev.highGCount);
        if (!gData.highGArmed) Serial.print(F(" NOT-ARMED"));
        Serial.print(F("  yaw="));
        printFloat(gData.yawRelDeg, 1);
        Serial.print(F("  T="));
        printFloat(gData.temperatureC, 0);
        Serial.print(F("  ioerr/bad="));
        Serial.print(gDev.ioErrors);
        Serial.print(F("/"));
        Serial.print(gDev.implausible);
        if (gData.dataGap) Serial.print(F("  GAP"));
        Serial.println();

        Serial.print(F("   over "));
        Serial.print(gPolls);
        Serial.print(F(" polls: ok "));      Serial.print(gOk);
        Serial.print(F("  partial "));       Serial.print(gPartial);
        Serial.print(F("  |grav| "));
        // The sentinels are never printed as numbers. Before this, the first
        // report of every run showed "1000000000.00..-1000000000.00", which
        // reads as a catastrophic sensor fault and is only an empty min/max.
        if (gGravityMax < gGravityMin){
            Serial.print(F("(no sample yet)"));
        } else {
            printFloat(gGravityMin, 2); Serial.print(F(".."));
            printFloat(gGravityMax, 2);
        }
        Serial.print(F("  worst |lin| "));   printFloat(gLinearMax, 2);
        Serial.println();

        Serial.print(F("   FAILS: gravity "));  Serial.print(gGravityBad);
        Serial.print(F("  linear "));           Serial.print(gLinearBad);
        Serial.print(F("  sum "));              Serial.print(gSumBad);
        // The linear count is expected to be non-zero the moment anyone touches
        // the bench, which is the whole point of the tap test. GRAVITY AND SUM
        // MUST BOTH STAY AT ZERO — those are the gate.
        Serial.println(F("   (linear rises on a tap; gravity and sum must stay 0)"));
        printCalibration();
        Serial.println();
    }

    while (Serial.available()){
        const int c = Serial.read();
        if      (c == 'c') printCalibration();
        else if (c == 'z') { resetStats(); Serial.println(F("stats zeroed")); }
        else if (c == 'r') restart(imuSampleMode(gDev));
        else if (c == 'a') restart(IMUSampleMode::Raw);
        else if (c == 'i') restart(IMUSampleMode::Fusion);
        else if (c == 'b') busSurvey();
        else if (c == 'l'){
            uint8_t chip = 0;
            uint8_t p[BNO055_CALIB_BYTES];
            if (bno055CalibLoad(p, &chip)){
                char desc[80];
                bno055CalibDescribe(p, desc, sizeof(desc));
                Serial.print(F("stored profile (chip 0x"));
                Serial.print(chip, HEX);
                Serial.print(F("): "));
                Serial.println(desc);
            } else {
                Serial.println(F("no valid profile on the card"));
            }
        }
        else if (c == 'w'){
            // THE PHASE 6 GATE, and it is worth doing at 3/3 rather than
            // whenever: a partial profile restored at every future boot anchors
            // the fusion to a half-finished estimate it then has to climb out of.
            if (!imuIsReady(gDev)){
                Serial.println(F("not configured yet"));
            } else if (gData.calibGyro < 3u || gData.calibAccel < 3u){
                Serial.print(F("calibration is "));
                Serial.print(gData.calibGyro);
                Serial.print(F("/"));
                Serial.print(gData.calibAccel);
                Serial.println(F(" - want 3/3."));
                Serial.println(F("  Gyro:  leave it completely still for a few seconds."));
                Serial.println(F("  Accel: rest the board on each of its 6 faces for ~4 s,"));
                Serial.println(F("         moving SLOWLY between. Tapping or shaking undoes it."));
                Serial.println(F("  Refused below 3/3 on purpose: a partial profile would be"));
                Serial.println(F("  restored at every future boot and anchor the fusion to a"));
                Serial.println(F("  half-finished estimate it then has to climb back out of."));
            } else {
                uint8_t fresh[BNO055_CALIB_BYTES];
                // Costs ~60 ms in CONFIG producing nothing. Fine at a bench
                // prompt; the production path rate-limits it and skips it during
                // a High-G event for exactly this reason.
                if (!bno055CalibCapture(gDev.init, fresh)){
                    Serial.println(F("capture FAILED - sensor returned to its operating mode"));
                } else if (!bno055CalibStore(fresh, gDev.init.dev.chip_id)){
                    Serial.println(F("save FAILED - is a card fitted? previous profile intact"));
                } else {
                    memcpy(gCalibProfile, fresh, sizeof(fresh));
                    gCalibValid = true;
                    char desc[80];
                    bno055CalibDescribe(fresh, desc, sizeof(desc));
                    Serial.print(F("saved: "));
                    Serial.println(desc);
                    Serial.println(F("Press 'r' to restart bring-up and confirm it restores."));
                }
            }
        }
        else if (c == 's'){
            // THE PHASE 4 GATE. With no FIFO, a poll that does not happen is
            // data that no longer exists — so a peak from inside this window is
            // gone for good, and nothing can bring it back. The High-G latch is
            // held in the part's own register until cleared, so it is still
            // there afterwards. That difference is the entire reason the
            // interrupt was worth wiring, and this is the only way to see it.
            Serial.println();
            Serial.println(F("STALL: blocking the loop for 500 ms - TAP THE BENCH HARD NOW"));
            const uint32_t stallStart = millis();
            const uint16_t hgBefore   = gDev.highGCount;
            // Genuinely blocked: no poll, no init tick, no watchdog feed. 500 ms
            // is well inside the 8 s watchdog, so this stalls without resetting.
            while ((millis() - stallStart) < 500UL) { }
            Serial.print(F("STALL over after "));
            Serial.print(millis() - stallStart);
            Serial.println(F(" ms."));
            Serial.print(F("  High-G latches during the stall: "));
            Serial.println(gDev.highGCount - hgBefore);
            Serial.println(F("  (0 here does not yet mean failure - the latch is read on the"));
            Serial.println(F("   NEXT poll, so watch for HIGH-G on the line below.)"));
        }
    }
}
