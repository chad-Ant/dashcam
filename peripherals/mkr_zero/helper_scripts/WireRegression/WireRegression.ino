/**
 * WireRegression — proves the vendored SAMD Wire patch actually works.
 *
 * The stock Arduino SAMD 1.8.14 TwoWire::requestFrom() reads an uninitialised
 * `busOwner` whenever quantity == 1, because the only assignment to it lives
 * inside the read loop's condition and `1 < 1` short-circuits before it runs.
 * The two uses that follow decide (a) whether to emit the STOP condition and
 * (b) whether to decrement the returned byte count. So the observable symptoms
 * of the bug are exactly: a one-byte read that reports 0 bytes, or one that
 * leaves the bus without a STOP.
 *
 * This sketch tests for both, directly:
 *
 *   T1  one-byte reads return one byte, with the right content
 *   T2  the bus is IDLE afterwards — i.e. the STOP really was emitted. Read
 *       from SERCOM2's own BUSSTATE field, which is the hardware's opinion
 *       rather than an inference. This is the test a logic analyser would
 *       otherwise be needed for.
 *   T3  quantity 2 and 32 still behave (the patch must not disturb them)
 *   T4  repeated-start (write sub-address, no STOP, then read) still works
 *   T5  a u-blox NAV-PVT read with the SparkFun transaction size shrunk to 9,
 *       so the transfer ends on a one-byte chunk. This is the real-world path
 *       the IMU's own two-byte workaround could never protect, because u-blox
 *       register 0xFF is a CONSUMING byte stream — padding a read there eats a
 *       byte of a real message. T5 asserts on an instrumented Wire counter, not
 *       on "packets still parsed": shrinking the chunk size only makes a
 *       singleton LIKELY (it depends on the pending count modulo 9), so packets
 *       arriving proves nothing about whether quantity==1 ever occurred.
 *
 * STOCK CONTROL
 * -------------
 * Build with `BuildAndUpload.cmd /stock` to run this same suite against the
 * UNPATCHED core Wire. No hand-editing required.
 *
 * Recorded result, so nobody re-derives it: on arm-none-eabi-gcc 7.2.1 at -Os,
 * the stock control PASSES T1-T4 and SKIPS T5. Stating that as "passes all five"
 * would overclaim in the one direction that matters — T5 is not a test the stock
 * build survived, it is a test the stock build cannot answer, because the
 * instrumented counters it asserts on exist only in the vendored Wire. Four
 * observable passes and one unanswerable case is a weaker result than five
 * passes, and the wording has to say so.
 *
 * T1-T4 pass because the generated code happens to carry a nonzero register
 * value into both uses of the indeterminate `busOwner`. That is the latent
 * nature of the defect, not evidence against it — source inspection proves
 * undefined behaviour on every acknowledged quantity==1 path regardless of what
 * this particular binary does.
 */

#include <Wire.h>

#include "DataDictionary.h"
#include "TimerFunctions.h"
#include "I2CBus.h"
#include "GPSFunctions.h"

#if !defined(DASHCAM_SAMD_WIRE_REQUESTFROM1_FIX) && !defined(DASHCAM_WIRE_STOCK_CONTROL)
#error "Build with vendor/Wire, or with /stock to run the deliberate stock control."
#endif

#if defined(DASHCAM_WIRE_STOCK_CONTROL)
#define WIRE_BUILD_LABEL "STOCK CONTROL / marker absent"
#else
#define WIRE_BUILD_LABEL "vendored Wire / marker present"
#endif

static constexpr unsigned long SERIAL_READY_TIMEOUT_MS = 2000;
/// Enough repeats that an intermittent UB outcome cannot hide in the noise.
static constexpr uint16_t REPEATS = 500;

/// LSM6DSOX WHO_AM_I, used as a known-answer target for the read tests.
static constexpr uint8_t SOX_ADDR    = IMU_ACCEL_I2C_ADDRESS;
static constexpr uint8_t SOX_WHOAMI  = 0x0Fu;
static constexpr uint8_t SOX_ID      = 0x6Cu;
/// 32 contiguous readable output/config registers starting here.
static constexpr uint8_t SOX_OUT_TEMP_L = 0x20u;

/// SERCOM2 I2CM BUSSTATE encoding (SAMD21 datasheet 28.10.3).
static constexpr uint8_t BUSSTATE_UNKNOWN = 0;
static constexpr uint8_t BUSSTATE_IDLE    = 1;
static constexpr uint8_t BUSSTATE_OWNER   = 2;
static constexpr uint8_t BUSSTATE_BUSY    = 3;

SFE_UBLOX_GNSS myGNSS;

static uint16_t passCount = 0;
static uint16_t failCount = 0;
static uint16_t skipCount = 0;

/** @brief The hardware's own view of who currently owns the bus. */
static inline uint8_t busState(void)
{
    return (uint8_t)SERCOM2->I2CM.STATUS.bit.BUSSTATE;
}

/**
 * @brief Waits, bounded, for the controller to release the bus after a STOP.
 *
 * Sampling BUSSTATE the instant requestFrom() returns is a race, not a test:
 * prepareCommandBitsWire() only spins on SYSOP, which is register-write
 * synchronisation, so the STOP is still being clocked onto the wire when the
 * call returns and the controller is legitimately still OWNER. Polling with a
 * deadline asks the question that actually matters — does the bus become idle
 * at all — and still fails fast when the STOP was never issued.
 *
 * A STOP at 100 kHz costs well under 20 us; 500 us is generous.
 */
static bool waitBusIdle(void)
{
    const uint32_t start = micros();
    while ((micros() - start) < 500u) {
        if (busState() == BUSSTATE_IDLE) return true;
    }
    return false;
}

/**
 * @brief Records a test that could not be run, as distinct from one that failed.
 *
 * The stock control arm cannot carry the instrumented counters — they only exist
 * in the vendored Wire — so T5 is unanswerable there. Counting that as a failure
 * would make the control look worse than the patched build for a reason that has
 * nothing to do with the patch, which is precisely the kind of misreading a
 * control experiment exists to avoid.
 */
static void reportSkip(const char *name, const char *detail)
{
    skipCount++;
    Serial.print("SKIP  ");
    Serial.print(name);
    if (detail != nullptr && detail[0] != '\0') {
        Serial.print("  -- ");
        Serial.print(detail);
    }
    Serial.println();
}

static void report(const char *name, bool ok, const char *detail)
{
    if (ok) passCount++; else failCount++;
    Serial.print(ok ? "PASS  " : "FAIL  ");
    Serial.print(name);
    if (detail != nullptr && detail[0] != '\0') {
        Serial.print("  -- ");
        Serial.print(detail);
    }
    Serial.println();
}

/** @brief Points the device's register pointer at @p reg without a STOP. */
static bool setRegPointer(uint8_t address, uint8_t reg, bool stopBit)
{
    Wire.beginTransmission(address);
    if (Wire.write(reg) != 1) { (void)Wire.endTransmission(true); return false; }
    return Wire.endTransmission(stopBit) == 0;
}

// ── T1 + T2: the one-byte path ───────────────────────────────────────────────

static void testOneByteReads(void)
{
    uint16_t shortReads = 0;
    uint16_t badContent = 0;
    uint16_t notIdle    = 0;

    for (uint16_t i = 0; i < REPEATS; i++) {
        if (!setRegPointer(SOX_ADDR, SOX_WHOAMI, false)) { shortReads++; continue; }

        const size_t got = Wire.requestFrom((uint8_t)SOX_ADDR, (size_t)1, true);
        if (got != 1) { shortReads++; continue; }

        const int value = Wire.read();
        if (value != (int)SOX_ID) badContent++;

        // The STOP check. With the bug, `stopBit && busOwner` can be false on
        // this path and the controller stays the bus owner instead of releasing.
        if (!waitBusIdle()) notIdle++;
    }

    char detail[72];
    snprintf(detail, sizeof(detail), "%u reps: %u short, %u bad byte, %u not-idle",
             (unsigned)REPEATS, (unsigned)shortReads, (unsigned)badContent, (unsigned)notIdle);
    report("T1/T2 one-byte requestFrom + STOP", (shortReads == 0 && badContent == 0 && notIdle == 0), detail);
}

// ── T3: the patch must not disturb multi-byte reads ──────────────────────────

static void testMultiByteReads(void)
{
    uint16_t bad2 = 0;
    uint16_t bad32 = 0;

    for (uint16_t i = 0; i < REPEATS; i++) {
        if (!setRegPointer(SOX_ADDR, SOX_OUT_TEMP_L, false) ||
            Wire.requestFrom((uint8_t)SOX_ADDR, (size_t)2, true) != 2) { bad2++; continue; }
        (void)Wire.read();
        (void)Wire.read();
        if (!waitBusIdle()) bad2++;
    }

    for (uint16_t i = 0; i < REPEATS; i++) {
        if (!setRegPointer(SOX_ADDR, SOX_OUT_TEMP_L, false) ||
            Wire.requestFrom((uint8_t)SOX_ADDR, (size_t)32, true) != 32) { bad32++; continue; }
        for (uint8_t b = 0; b < 32; b++) (void)Wire.read();
        if (!waitBusIdle()) bad32++;
    }

    char detail[64];
    snprintf(detail, sizeof(detail), "qty2 failures=%u, qty32 failures=%u",
             (unsigned)bad2, (unsigned)bad32);
    report("T3 quantity 2 / 32", (bad2 == 0 && bad32 == 0), detail);
}

// ── T4: repeated start ───────────────────────────────────────────────────────

static void testRepeatedStart(void)
{
    // WHO_AM_I read via repeated start (no STOP between address write and read)
    // is exactly what IMUFunctions::readRegs() does on every poll.
    uint16_t bad = 0;
    for (uint16_t i = 0; i < REPEATS; i++) {
        if (!setRegPointer(SOX_ADDR, SOX_WHOAMI, false)) { bad++; continue; }
        if (Wire.requestFrom((uint8_t)SOX_ADDR, (size_t)1, true) != 1) { bad++; continue; }
        if (Wire.read() != (int)SOX_ID) bad++;
    }

    char detail[48];
    snprintf(detail, sizeof(detail), "%u reps, %u failures", (unsigned)REPEATS, (unsigned)bad);
    report("T4 repeated-start register read", bad == 0, detail);
}

// ── T5: u-blox NAV-PVT forced to end on a one-byte chunk ─────────────────────

static void testGnssOneByteTail(void)
{
    if (initializeGPS_I2C(myGNSS) != GPSReturnStatus::OK) {
        report("T5 GNSS 1-byte tail chunk", false, "receiver did not initialise");
        return;
    }

    // Small transactions so the driver is forced to chunk, which eventually
    // leaves a one-byte remainder. Note this only makes a singleton LIKELY —
    // the final chunk is (pending bytes mod 9) — which is exactly why the
    // assertion below is on an instrumented counter and not on the chunk size.
    myGNSS.setI2CTransactionSize(9);

#ifdef DASHCAM_WIRE_INSTRUMENT
    dashcamWireQty1AddrFilter = GPS_DEFAULT_I2C_ADDRESS;
    dashcamWireQty1Filtered   = 0;
#endif

    GPSData sample;
    initGPSData(sample);

    uint16_t packets = 0;
    const unsigned long start = millis();
    // Bounded by time, not by packet count: with no fix the receiver still
    // streams PVT, but a broken read path would simply never yield one and an
    // unbounded wait would hang the test.
    while (!isTimeout(6000UL, start)) {
        const GPSReturnStatus st = getGPSData(myGNSS, sample);
        if (st == GPSReturnStatus::OK || st == GPSReturnStatus::NO_FIX) {
            packets++;
            if (packets >= 20) break;
        }
    }

    myGNSS.setI2CTransactionSize(32);   // restore the default

#ifdef DASHCAM_WIRE_INSTRUMENT
    const uint32_t singletons = dashcamWireQty1Filtered;
    dashcamWireQty1AddrFilter = 0xFFu;

    char detail[80];
    snprintf(detail, sizeof(detail), "%u packets, %lu one-byte reads to 0x42",
             (unsigned)packets, (unsigned long)singletons);
    // BOTH conditions. Packets alone would pass even if the driver never issued
    // a single one-byte request, which would make this test a decoration.
    report("T5 GNSS drives a real 1-byte requestFrom",
           (packets >= 20) && (singletons > 0UL), detail);
#else
    char detail[80];
    snprintf(detail, sizeof(detail), "%u packets; counters absent in this build",
             (unsigned)packets);
    // Not a failure and not a pass. Reporting "pass" would claim evidence that
    // does not exist; reporting "fail" would blame the build for a question it
    // was never able to ask.
    reportSkip("T5 GNSS drives a real 1-byte requestFrom", detail);
#endif
}

/** @brief Runs the whole suite and prints a verdict. */
static void runAllTests(void)
{
    passCount = 0;
    failCount = 0;
    skipCount = 0;

    Serial.println();
    Serial.println("=== SAMD Wire requestFrom() regression suite ===");
    Serial.print("build: ");
    Serial.println(WIRE_BUILD_LABEL);

    if (i2cBusBegin() != I2CBusState::Ready) {
        Serial.println("FATAL: I2C bus not Ready - lines held low, cannot test");
        return;
    }

    // Everything below assumes the LSM6DSOX answers; without it the read tests
    // would "pass" by failing uniformly, which is worse than not running.
    if (!i2cProbeAddress(SOX_ADDR)) {
        Serial.println("FATAL: no device at the LSM6DSOX address - cannot test");
        return;
    }

    testOneByteReads();
    testMultiByteReads();
    testRepeatedStart();
    testGnssOneByteTail();

    Serial.println();
    Serial.print("RESULT: ");
    Serial.print(passCount);
    Serial.print(" passed, ");
    Serial.print(failCount);
    Serial.print(" failed, ");
    Serial.print(skipCount);
    Serial.println(" skipped");
    Serial.print(failCount == 0 ? "NO REGRESSIONS" : "REGRESSIONS PRESENT");
    Serial.print("  [");
    Serial.print(WIRE_BUILD_LABEL);
    Serial.println("]");
}

void setup()
{
    pinMode(STATUS_INDICATOR, OUTPUT);
    digitalWrite(STATUS_INDICATOR, HIGH);

    Serial.begin(SERIAL_BAUDRATE);
    const unsigned long serialWaitStart = millis();
    while (!Serial && !isTimeout(SERIAL_READY_TIMEOUT_MS, serialWaitStart)) {
        // bounded; the timeout is the guarantee
    }

    runAllTests();
    Serial.println("(send any character to re-run)");
}

void loop()
{
    // Re-runnable on demand. A suite that only reports once in setup() is
    // unreadable over USB CDC: the board finishes its bounded Serial wait and
    // prints before a host can attach, so the results scroll past unseen.
    if (Serial.available() > 0) {
        while (Serial.available() > 0) (void)Serial.read();   // drain
        runAllTests();
        Serial.println("(send any character to re-run)");
    }

    // Blink so a running board is distinguishable from a hung one.
    static unsigned long lastBlink = 0;
    if (isTimeout(500UL, lastBlink)) {
        lastBlink = millis();
        digitalWrite(STATUS_INDICATOR, !digitalRead(STATUS_INDICATOR));
    }
}
