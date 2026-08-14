/**
 * @file BNO055BringUp.ino
 * @brief Phase 2's gate: proves the staged bring-up never blocks the loop.
 *
 * The claim being tested is not "the sensor configures" — that is the easy half
 * and any blocking driver manages it. The claim is that it configures WITHOUT
 * holding the CPU, because the previous inertial driver's bring-up did hold it,
 * and the consequences were a stalled CAN drain, a telemetry push that jittered,
 * and a hang that landed at the same point on every boot under the same armed
 * watchdog.
 *
 * So this sketch measures two things the console output of a working sensor
 * would never show you:
 *
 *   worst tick   the longest single bno055InitTick() call. Should stay in the
 *                hundreds of MICROseconds. If a datasheet wait ever migrates
 *                into a stage instead of a deadline, this is where it appears.
 *   worst gap    the longest interval between consecutive loop() passes. This
 *                is the number that matters, because it is what the CAN drain
 *                and the C3 link would actually experience.
 *
 * The 650 ms reset wait is deliberately part of the run. If the machine is
 * right, the bring-up takes most of a second in WALL CLOCK while the loop keeps
 * turning over at full speed throughout — the two numbers above are what
 * separate that from a delay(650).
 *
 * Commands, once running:  r restart   i IMUPLUS   a AMG   c calibration
 */

#include <Wire.h>
#include <BNO055.h>

#include "BNO055Init.h"
#include "BNO055Transport.h"
#include "I2CBus.h"

/// Static storage duration is REQUIRED, not stylistic: bno055_init() keeps a
/// pointer to dev inside the driver, and every driver call afterwards
/// dereferences it.
static BNO055InitState gInit;

static BNO055InitStage gLastStage = BNO055InitStage::Idle;
static uint32_t gBeginMs   = 0;

static uint32_t gLoops      = 0;
static uint32_t gMaxTickUs  = 0;
static uint32_t gMaxGapUs   = 0;
static uint32_t gLastLoopUs = 0;

/// Gap measurement is armed only after the first pass, so the time spent in
/// setup() printing a banner is not reported as a stalled loop.
static bool gGapArmed = false;

static void printStage(BNO055InitStage stage)
{
    Serial.print(F("  t+"));
    Serial.print(millis() - gBeginMs);
    Serial.print(F(" ms  "));
    Serial.print(bno055InitStageName(stage));

    if (stage == BNO055InitStage::Failed){
        Serial.print(F("  at "));
        Serial.print(bno055InitStageName(gInit.failedAt));
        Serial.print(F("  reason "));
        Serial.print(bno055InitStatusName(gInit.lastStatus));
        Serial.print(F("  attempt "));
        Serial.println(gInit.failures);

        // The bytes, not just the verdict. "config-failed" alone covers a
        // NACKed write, a failed read-back and a write the part accepted and
        // ignored — three faults, three repairs, and no way to tell them apart
        // from a status name.
        Serial.print(F("      reg 0x"));   Serial.print(gInit.lastRegAddr, HEX);
        Serial.print(F("  wrote 0x"));     Serial.print(gInit.lastRegWrote, HEX);
        if (!gInit.lastRegWriteOk){
            Serial.print(F("  WRITE NACKED"));
        } else if (!gInit.lastRegReadOk){
            Serial.print(F("  write ok, READ-BACK FAILED"));
        } else {
            Serial.print(F("  read 0x"));  Serial.print(gInit.lastRegRead, HEX);
            Serial.print(F("  (write accepted then ignored)"));
        }
        Serial.println();

        Serial.print(F("      opr_mode 0x")); Serial.print(gInit.opModeSeen, HEX);
        Serial.print(F("  sys_status "));     Serial.print(gInit.sysStatus);
        Serial.print(F("  sys_err "));        Serial.print(gInit.sysError);
        Serial.print(F("  clock "));
        Serial.print(gInit.externalCrystal ? F("external") : F("internal"));
        if (gInit.clockFallback) Serial.print(F(" (fell back)"));
        Serial.println();
        return;
    }
    Serial.println();
}

static void printConfigured()
{
    Serial.println();
    Serial.println(F("---- CONFIGURED ----"));
    Serial.print(F("  address        0x")); Serial.println(gInit.address, HEX);
    Serial.print(F("  mode           "));
    Serial.println(gInit.opMode == OPERATION_MODE_AMG ? F("AMG (raw, no fusion)")
                                                      : F("IMUPLUS (fusion, no magnetometer)"));
    Serial.print(F("  clock          "));
    if (gInit.externalCrystal)   Serial.println(F("external crystal"));
    else if (gInit.clockFallback) Serial.println(F("INTERNAL - crystal requested but the part refused it"));
    else                          Serial.println(F("internal (external not requested)"));
    Serial.print(F("  unit_sel       0x")); Serial.print(gInit.unitSelSeen, HEX);
    Serial.print(F("  -> "));
    // Reported rather than assumed: bit 7 decides the Euler sign convention,
    // and Phase 3 decodes pitch against this byte.
    Serial.print((gInit.unitSelSeen & 0x80u) ? F("Android") : F("Windows"));
    Serial.println(F(" Euler convention"));
    Serial.print(F("  sys status     ")); Serial.print(gInit.sysStatus);
    Serial.println(gInit.sysStatus == BNO055_SYS_STATUS_FUSION_RUNNING
                       ? F("  (fusion running)")
                       : (gInit.sysStatus == BNO055_SYS_STATUS_NO_FUSION_RUNNING
                              ? F("  (running, no fusion)") : F("")));
    Serial.print(F("  sys error      ")); Serial.println(gInit.sysError);
    Serial.print(F("  bring-up took  ")); Serial.print(gInit.configuredAtMs - gBeginMs);
    Serial.println(F(" ms of wall clock"));
    Serial.println();
    Serial.println(F("The two numbers that decide this gate:"));
    Serial.print(F("  worst tick     ")); Serial.print(gMaxTickUs);
    Serial.println(F(" us   (one bno055InitTick call)"));
    Serial.print(F("  worst loop gap ")); Serial.print(gMaxGapUs);
    Serial.println(F(" us   (what the CAN drain would have felt)"));
    Serial.print(F("  loop passes    ")); Serial.println(gLoops);
    Serial.println();
    Serial.println(F("A blocking bring-up would put ~650000 us in BOTH of those."));
    Serial.println(F("Commands: r restart   i IMUPLUS   a AMG   c calibration"));
    Serial.println();
}

static void printCalibration()
{
    uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;
    if (!bno055ReadCalibration(gInit, sys, gyro, accel, mag)){
        Serial.println(F("calibration read FAILED"));
        return;
    }
    Serial.print(F("calib  sys ")); Serial.print(sys);
    Serial.print(F("  gyro "));     Serial.print(gyro);
    Serial.print(F("  accel "));    Serial.print(accel);
    Serial.print(F("  mag "));      Serial.print(mag);
    // Said out loud because a zero here looks like a fault and is not one: in
    // IMUPLUS the magnetometer is switched off, so its counter cannot move.
    if (gInit.opMode == OPERATION_MODE_IMUPLUS) Serial.print(F("   (mag unused in IMUPLUS)"));
    Serial.println();
}

static void restart(uint8_t mode)
{
    gBeginMs   = millis();
    gLastStage = BNO055InitStage::Idle;
    gMaxTickUs = 0;
    gMaxGapUs  = 0;
    gLoops     = 0;
    gGapArmed  = false;
    if (!bno055InitBegin(gInit, mode)){
        Serial.println(F("bno055InitBegin REFUSED (quarantined, or unsupported mode)"));
        return;
    }
    Serial.println();
    Serial.print(F("Bring-up starting, target mode "));
    Serial.println(mode == OPERATION_MODE_AMG ? F("AMG") : F("IMUPLUS"));
}

void setup()
{
    Serial.begin(115200);
    const uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 3000) { }

    watchdogArm(8000UL);

    Serial.println();
    Serial.println(F("============ BNO055 staged bring-up ============"));
    Serial.println(F("Phase 2 gate: the sequence must not block loop()."));

    Wire.begin();
    Wire.setClock(BNO055_I2C_CLOCK_HZ);

    Serial.print(F("I2C clock: "));
    Serial.print(BNO055_I2C_CLOCK_HZ / 1000UL);
    Serial.println(F(" kHz"));
    Serial.print(F("Reset wait: "));
    Serial.print(BNO055_RESET_TO_CONFIG_MS);
    Serial.println(F(" ms - this is the one a blocking driver would sit through."));
    Serial.println();

    restart(OPERATION_MODE_IMUPLUS);
}

void loop()
{
    watchdogFeed();

    const uint32_t nowUs = micros();
    if (gGapArmed){
        const uint32_t gap = nowUs - gLastLoopUs;
        if (gap > gMaxGapUs) gMaxGapUs = gap;
    }
    gLastLoopUs = nowUs;
    gGapArmed   = true;
    gLoops++;

    const uint32_t tickStart = micros();
    const BNO055InitStage stage = bno055InitTick(gInit);
    const uint32_t tickUs = micros() - tickStart;
    if (tickUs > gMaxTickUs) gMaxTickUs = tickUs;

    if (stage != gLastStage){
        printStage(stage);
        gLastStage = stage;
        if (stage == BNO055InitStage::Configured) printConfigured();
    }

    // Periodic health line once it is up. Also proves the loop is still turning.
    static uint32_t lastReport = 0;
    if (bno055InitReady(gInit) && (millis() - lastReport) >= 5000UL){
        lastReport = millis();
        Serial.print(F("running  loops ")); Serial.print(gLoops);
        Serial.print(F("  worst gap "));    Serial.print(gMaxGapUs);
        Serial.print(F(" us  transport errors ")); Serial.print(bno055TransportErrorCount());
        Serial.println();
        printCalibration();
    }

    while (Serial.available()){
        const int c = Serial.read();
        if      (c == 'r') restart(gInit.opMode);
        else if (c == 'i') restart(OPERATION_MODE_IMUPLUS);
        else if (c == 'a') restart(OPERATION_MODE_AMG);
        else if (c == 'c') printCalibration();
    }
}
