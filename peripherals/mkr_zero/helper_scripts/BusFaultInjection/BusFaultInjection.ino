/**
 * BusFaultInjection — wedges the I2C bus on purpose, then proves recovery.
 *
 * The recovery path in lib/I2CBus.cpp had never actually run against a wedged
 * bus. Everything about it was argued from the datasheet, which is exactly the
 * kind of code that turns out to be wrong the one time it matters.
 *
 * HOW THE WEDGE IS MADE
 * ---------------------
 * Not by shorting SDA to ground — that tests a permanent hardware fault, which
 * no amount of clocking can fix and which the code is supposed to REPORT rather
 * than repair. The interesting fault is the recoverable one: a controller reset
 * that lands mid-transaction, leaving a slave part-way through shifting out a
 * byte, still driving SDA and waiting for clocks that never come.
 *
 * So this sketch bit-bangs a real read transaction against the LSM6DSOX —
 * START, address+R, ACK — and then simply stops clocking. The register pointer
 * is parked on WHO_AM_I first, whose value 0x6C has bit 7 = 0, so the very
 * first data bit the slave presents is a zero: it holds SDA low and the bus is
 * genuinely stuck, by the slave, exactly as after an unclean reset.
 *
 * Deliberately NOT tested: attempting a Wire transaction while wedged. The SAMD
 * driver waits on its bus flags in unbounded loops, so that would hang the board
 * rather than report anything. Avoiding that hang is the whole reason recovery
 * runs before the first transaction.
 *
 * Commands:  a = full automatic test    w = wedge only
 *            r = recover only           h = help
 */

#include <Wire.h>

#include "DataDictionary.h"
#include "TimerFunctions.h"
#include "I2CBus.h"
#include "IMUFunctions.h"
#include "GPSFunctions.h"

SFE_UBLOX_GNSS myGNSS;

static constexpr unsigned long SERIAL_READY_TIMEOUT_MS = 2000;

static constexpr uint8_t SOX_ADDR   = IMU_ACCEL_I2C_ADDRESS;
static constexpr uint8_t SOX_WHOAMI = 0x0Fu;
static constexpr uint8_t SOX_ID     = 0x6Cu;   ///< 0b01101100 — bit 7 is 0.

/// Half-bit period for the bit-banged transaction (~100 kHz).
static constexpr uint32_t HALF_BIT_US = 5u;
/// Bounded wait for SCL to rise, so a stretching slave cannot hang the bang.
static constexpr uint32_t SCL_RISE_TIMEOUT_US = 200u;

static uint16_t passCount = 0;
static uint16_t failCount = 0;

// ── open-drain bit-banging ───────────────────────────────────────────────────
//
// Same latch-preload order as lib/I2CBus.cpp, for the same reason: pinMode()
// with INPUT_PULLUP leaves the SAMD21 output latch HIGH, so switching straight
// to OUTPUT would drive the line push-pull high for an instant — into a slave
// that may be holding it low.

static void driveLow(uint8_t pin)
{
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
}

static inline void releaseLine(uint8_t pin)
{
    pinMode(pin, INPUT_PULLUP);
}

static bool releaseSclAndWait(void)
{
    releaseLine(PIN_WIRE_SCL);
    const uint32_t start = micros();
    while ((micros() - start) < SCL_RISE_TIMEOUT_US) {
        if (digitalRead(PIN_WIRE_SCL) == HIGH) return true;
    }
    return false;
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

/**
 * @brief Leaves the LSM6DSOX mid-byte, holding SDA low.
 * @return @c true if SDA is actually low afterwards (the wedge took).
 */
static bool wedgeBus(void)
{
    // Park the register pointer on WHO_AM_I using the normal driver, so the
    // aborted read below returns 0x6C and its first bit is a guaranteed zero.
    Wire.beginTransmission(SOX_ADDR);
    if (Wire.write(SOX_WHOAMI) != 1) { (void)Wire.endTransmission(true); return false; }
    if (Wire.endTransmission(true) != 0) return false;

    // Take the pins off SERCOM2 and drive them by hand.
    releaseLine(PIN_WIRE_SCL);
    releaseLine(PIN_WIRE_SDA);
    delayMicroseconds(HALF_BIT_US * 2u);

    // START: SDA falls while SCL is high.
    driveLow(PIN_WIRE_SDA);
    delayMicroseconds(HALF_BIT_US);
    driveLow(PIN_WIRE_SCL);
    delayMicroseconds(HALF_BIT_US);

    // Address byte, MSB first: 7-bit address then the READ bit.
    const uint8_t frame = static_cast<uint8_t>((SOX_ADDR << 1) | 0x01u);
    for (int8_t bit = 7; bit >= 0; bit--) {
        if (((frame >> bit) & 0x01u) != 0u) releaseLine(PIN_WIRE_SDA);
        else                                driveLow(PIN_WIRE_SDA);
        delayMicroseconds(HALF_BIT_US);
        if (!releaseSclAndWait()) return false;
        delayMicroseconds(HALF_BIT_US);
        driveLow(PIN_WIRE_SCL);
        delayMicroseconds(HALF_BIT_US);
    }

    // ACK clock: release SDA so the slave can pull it down.
    releaseLine(PIN_WIRE_SDA);
    delayMicroseconds(HALF_BIT_US);
    if (!releaseSclAndWait()) return false;
    delayMicroseconds(HALF_BIT_US / 2u);
    const bool acked = (digitalRead(PIN_WIRE_SDA) == LOW);
    delayMicroseconds(HALF_BIT_US / 2u);
    driveLow(PIN_WIRE_SCL);
    delayMicroseconds(HALF_BIT_US);

    if (!acked) {
        // No slave answered; unwind rather than leaving a half-transaction.
        releaseLine(PIN_WIRE_SCL);
        releaseLine(PIN_WIRE_SDA);
        return false;
    }

    // ABORT HERE. With SCL low the slave has already presented data bit 7 of
    // 0x6C, which is 0 — so it is driving SDA low and waiting for a clock edge
    // that this sketch will never send. Release both lines and walk away; SCL
    // floats high, SDA stays down, held by the slave.
    if (!releaseSclAndWait()) return false;
    delayMicroseconds(HALF_BIT_US * 4u);

    return (digitalRead(PIN_WIRE_SDA) == LOW);
}

/** @brief Reads SDA/SCL as GPIO without disturbing whoever holds them. */
static void sampleLines(bool &sdaHigh, bool &sclHigh)
{
    pinMode(PIN_WIRE_SDA, INPUT_PULLUP);
    pinMode(PIN_WIRE_SCL, INPUT_PULLUP);
    delayMicroseconds(10);
    sdaHigh = (digitalRead(PIN_WIRE_SDA) == HIGH);
    sclHigh = (digitalRead(PIN_WIRE_SCL) == HIGH);
}

static void runFullTest(void)
{
    passCount = 0;
    failCount = 0;

    Serial.println();
    Serial.println("=== I2C bus fault injection ===");

    // ---- 0: healthy baseline -----------------------------------------------
    if (i2cBusBegin() != I2CBusState::Ready) {
        Serial.println("FATAL: bus not Ready before the test even starts");
        return;
    }
    const bool baselineAccel = i2cProbeAddress(IMU_ACCEL_I2C_ADDRESS);
    const bool baselineMag   = i2cProbeAddress(IMU_MAG_I2C_ADDRESS);
    const bool baselineGps   = i2cProbeAddress(GPS_DEFAULT_I2C_ADDRESS);
    {
        char detail[64];
        snprintf(detail, sizeof(detail), "accel=%d mag=%d gnss=%d",
                 (int)baselineAccel, (int)baselineMag, (int)baselineGps);
        report("T0 baseline: all three devices ACK",
               baselineAccel && baselineMag && baselineGps, detail);
    }
    if (!baselineAccel) {
        Serial.println("ABORT: no LSM6DSOX, nothing to wedge the bus with");
        return;
    }

    // ---- 1: wedge -----------------------------------------------------------
    const bool wedged = wedgeBus();
    bool sdaHigh = false;
    bool sclHigh = false;
    sampleLines(sdaHigh, sclHigh);
    {
        char detail[64];
        snprintf(detail, sizeof(detail), "SDA=%s SCL=%s",
                 sdaHigh ? "HIGH" : "LOW", sclHigh ? "HIGH" : "LOW");
        report("T1 slave left holding SDA low", wedged && !sdaHigh, detail);
    }
    if (!wedged) {
        Serial.println("Could not wedge the bus; recovery result below is meaningless.");
    }

    // ---- 2: recover ---------------------------------------------------------
    const uint32_t t0 = micros();
    const I2CBusState st = i2cBusRecover();
    const uint32_t elapsed = micros() - t0;

    sampleLines(sdaHigh, sclHigh);
    {
        char detail[80];
        snprintf(detail, sizeof(detail), "state=%d in %lu us, SDA=%s SCL=%s",
                 (int)st, (unsigned long)elapsed,
                 sdaHigh ? "HIGH" : "LOW", sclHigh ? "HIGH" : "LOW");
        report("T2 recovery frees the bus",
               (st == I2CBusState::Ready) && sdaHigh && sclHigh, detail);
    }

    // ---- 3: the bus actually works again ------------------------------------
    // Re-open through the manager, because sampleLines() just took the pins to
    // GPIO and they have to go back to SERCOM2.
    Wire.begin();
    Wire.setClock(IMU_I2C_CLOCK_HZ);

    const bool afterAccel = i2cProbeAddress(IMU_ACCEL_I2C_ADDRESS);
    const bool afterMag   = i2cProbeAddress(IMU_MAG_I2C_ADDRESS);
    const bool afterGps   = i2cProbeAddress(GPS_DEFAULT_I2C_ADDRESS);
    {
        char detail[64];
        snprintf(detail, sizeof(detail), "accel=%d mag=%d gnss=%d",
                 (int)afterAccel, (int)afterMag, (int)afterGps);
        report("T3 all three devices ACK after recovery",
               afterAccel && afterMag && afterGps, detail);
    }

    // ---- 4: a real sensor still reads correctly -----------------------------
    IMUDevice dev{};   // value-initialised: initializeIMU() reads dev.lifecycle
    IMUData   data;
    initIMUData(data);
    const IMUReturnStatus ist = initializeIMU(dev);

    bool sane = false;
    float mag = NAN;
    uint16_t samples = 0;
    if (ist == IMUReturnStatus::OK) {
        // Keep sampling for a fixed window and judge the LAST reading, not the
        // first. configureAccel() software-resets the part, and the first
        // conversions out of a reset come from a digital filter that has not
        // settled yet — grabbing sample number one and calling it the answer
        // measures the settling transient, not the sensor.
        const unsigned long start = millis();
        while (!isTimeout(400UL, start)) {
            if (getIMUData(dev, data) == IMUReturnStatus::OK && data.accelValid) samples++;
        }
        if (data.accelValid) {
            mag = sqrtf(data.accelX * data.accelX +
                        data.accelY * data.accelY +
                        data.accelZ * data.accelZ);
            sane = (mag > 9.0f && mag < 10.6f);
        }
    }
    {
        char detail[80];
        char magText[16];
        if (isnan(mag)) snprintf(magText, sizeof(magText), "--");
        else            snprintf(magText, sizeof(magText), "%d.%02d",
                                 (int)mag, (int)((mag - (int)mag) * 100.0f));
        snprintf(detail, sizeof(detail), "init=%d, %u samples, |a|=%s m/s2",
                 (int)ist, (unsigned)samples, magText);
        report("T4 IMU re-inits and reads 1 g after recovery", sane, detail);
    }

    Serial.println();
    Serial.print("RESULT: ");
    Serial.print(passCount);
    Serial.print(" passed, ");
    Serial.print(failCount);
    Serial.println(" failed");
    Serial.println(failCount == 0 ? "ALL TESTS PASSED" : "FAILURES PRESENT");
    Serial.println("(a = run again, w = wedge only, r = recover only, h = help)");
}

// ── the unrecoverable case: SDA physically shorted to GND ────────────────────
//
// Software cannot stage this one. Holding SDA low from the MCU proves nothing,
// because i2cBusRecover() releases both pins to INPUT_PULLUP as its very first
// action — it would drop the MCU's own hold and then report success. Only an
// external short exercises the path where nine clocks change nothing.
//
// What must be true while shorted, and is the whole point of the test: NOTHING
// HANGS. Every entry point is supposed to notice the bus is unusable and return,
// rather than walk into the SAMD driver's unbounded flag waits. Each call below
// is therefore timed, and a call that returns in microseconds is a call that
// did not transact.

/** @brief Runs a call, prints its elapsed time, and returns it. */
static uint32_t timedCall(const char *label, bool &okOut, bool expectedFalse)
{
    const uint32_t t0 = micros();
    const bool result = i2cProbeAddress(IMU_ACCEL_I2C_ADDRESS);
    const uint32_t elapsed = micros() - t0;
    okOut = (result == !expectedFalse);
    Serial.print("    ");
    Serial.print(label);
    Serial.print(" -> ");
    Serial.print(result ? "true" : "false");
    Serial.print("  in ");
    Serial.print(elapsed);
    Serial.println(" us");
    return elapsed;
}

static void testStuckPhase1(void)
{
    passCount = 0;
    failCount = 0;

    Serial.println();
    Serial.println("=== Stuck-bus test, phase 1 ===");
    Serial.println("Hold the jumper on SDA (D11) to GND. Waiting up to 20 s...");

    // Self-trigger on the short rather than running the instant the command
    // arrives. A hand-held jumper cannot be synchronised with a serial command,
    // and a test that runs while contact happens to be open reports a healthy
    // bus and calls it a failure — which is exactly what happened on the first
    // attempt. Waiting for the line to actually go low removes the coordination
    // problem: the operator holds the probe, the board decides when to start.
    pinMode(PIN_WIRE_SDA, INPUT_PULLUP);
    pinMode(PIN_WIRE_SCL, INPUT_PULLUP);

    const unsigned long waitStart = millis();
    bool triggered = false;
    while (!isTimeout(20000UL, waitStart)) {
        if (digitalRead(PIN_WIRE_SDA) == LOW) {
            // Require it to stay down briefly: a contact bounce must not start a
            // test the jumper cannot then hold through.
            delay(20);
            if (digitalRead(PIN_WIRE_SDA) == LOW) { triggered = true; break; }
        }
    }

    if (!triggered) {
        Serial.println("ABORT: SDA never went low. Use 'l' to find contact first.");
        Wire.begin();
        Wire.setClock(IMU_I2C_CLOCK_HZ);
        return;
    }
    Serial.println("short detected -- running now, KEEP HOLDING");

    // Force a fresh evaluation: without this the manager could still be holding
    // a cached Ready from before the short was applied.
    const uint32_t t0 = micros();
    const I2CBusState st = i2cBusRecover();
    const uint32_t recoverUs = micros() - t0;

    {
        char detail[64];
        snprintf(detail, sizeof(detail), "state=%d (want 2=Stuck) in %lu us",
                 (int)st, (unsigned long)recoverUs);
        report("S1 recovery reports Stuck", st == I2CBusState::Stuck, detail);
    }

    if (st != I2CBusState::Stuck) {
        Serial.println("    (SDA does not read low - is the short actually on D11?)");
    }

    // Every client entry point must refuse, quickly, without transacting.
    bool ok = false;
    const uint32_t probeUs = timedCall("i2cProbeAddress", ok, true);
    report("S2 probe refuses without transacting", ok && (probeUs < 5000u), "");

    IMUDevice dev{};   // value-initialised: initializeIMU() reads dev.lifecycle
    const uint32_t t1 = micros();
    const IMUReturnStatus ist = initializeIMU(dev);
    const uint32_t imuUs = micros() - t1;
    {
        char detail[64];
        snprintf(detail, sizeof(detail), "status=%d (want -5) in %lu us",
                 (int)ist, (unsigned long)imuUs);
        report("S3 initializeIMU returns NOK_BUS_STUCK",
               ist == IMUReturnStatus::NOK_BUS_STUCK, detail);
    }

    const uint32_t t2 = micros();
    const GPSReturnStatus gst = initializeGPS_I2C(myGNSS);
    const uint32_t gpsUs = micros() - t2;
    {
        // NOK_BUS_STUCK (-7), not NOK_INIT_FAILED (-1).  This test predates the
        // status, and kept passing only because the old code could not tell the
        // two apart.  The distinction is the point of the fault injection: -1
        // says nothing answered at 0x42, which on a held-low bus would be a
        // WRONG diagnosis — nothing was ever asked.  -7 says the bus was unsafe
        // to touch, which is what is actually true here and what sends someone
        // looking at the lines rather than at the receiver.
        char detail[72];
        snprintf(detail, sizeof(detail), "status=%d (want %d) in %lu us",
                 (int)gst, (int)GPSReturnStatus::NOK_BUS_STUCK,
                 (unsigned long)gpsUs);
        report("S4 initializeGPS_I2C refuses with NOK_BUS_STUCK",
               gst == GPSReturnStatus::NOK_BUS_STUCK, detail);
    }

    // The retry rate limit: hammering i2cBusBegin() while stuck must not run
    // recovery on every call, or an address sweep would recover 112 times.
    const uint32_t t3 = micros();
    for (uint8_t i = 0; i < 20; i++) (void)i2cBusBegin();
    const uint32_t burstUs = micros() - t3;
    {
        char detail[64];
        snprintf(detail, sizeof(detail), "20 calls in %lu us", (unsigned long)burstUs);
        // One recovery is ~120 us; 20 unthrottled would be >2000 us.
        report("S5 stuck retry is rate limited", burstUs < 1500u, detail);
    }

    // Did the jumper survive the test? Without this every result above is
    // unfalsifiable: a short that let go halfway makes a genuine failure look
    // like a genuine pass, and nothing in the numbers would say so.
    pinMode(PIN_WIRE_SDA, INPUT_PULLUP);
    delayMicroseconds(10);
    const bool stillLow = (digitalRead(PIN_WIRE_SDA) == LOW);
    report("S6 short held for the whole test", stillLow,
           stillLow ? "SDA still low" : "JUMPER LOST CONTACT - results above are void");
    Wire.begin();
    Wire.setClock(IMU_I2C_CLOCK_HZ);

    Serial.println();
    Serial.print("phase 1: ");
    Serial.print(passCount);
    Serial.print(" passed, ");
    Serial.print(failCount);
    Serial.println(" failed");
    Serial.println(">>> Now REMOVE the short, then send '2'");
}

static void testStuckPhase2(void)
{
    passCount = 0;
    failCount = 0;

    Serial.println();
    Serial.println("=== Stuck-bus test, phase 2 (short MUST be removed now) ===");

    // Deliberately NOT calling i2cBusRecover() by hand. The point is that the
    // ordinary entry point heals itself: a node that came up on a stuck bus has
    // to recover on its own, with no operator and no reset.
    delay(I2C_BUS_RECOVER_RETRY_MS + 50UL);   // let the rate limiter open

    const I2CBusState st = i2cBusBegin();
    {
        char detail[48];
        snprintf(detail, sizeof(detail), "state=%d (want 1=Ready)", (int)st);
        report("S7 i2cBusBegin recovers by itself", st == I2CBusState::Ready, detail);
    }

    const bool a = i2cProbeAddress(IMU_ACCEL_I2C_ADDRESS);
    const bool m = i2cProbeAddress(IMU_MAG_I2C_ADDRESS);
    const bool g = i2cProbeAddress(GPS_DEFAULT_I2C_ADDRESS);
    {
        char detail[48];
        snprintf(detail, sizeof(detail), "accel=%d mag=%d gnss=%d", (int)a, (int)m, (int)g);
        report("S8 all three devices ACK again", a && m && g, detail);
    }

    IMUDevice dev{};   // value-initialised: initializeIMU() reads dev.lifecycle
    IMUData   data;
    initIMUData(data);
    const IMUReturnStatus ist = initializeIMU(dev);

    bool sane = false;
    float mag = NAN;
    if (ist == IMUReturnStatus::OK) {
        const unsigned long start = millis();
        while (!isTimeout(400UL, start)) (void)getIMUData(dev, data);
        if (data.accelValid) {
            mag = sqrtf(data.accelX * data.accelX +
                        data.accelY * data.accelY +
                        data.accelZ * data.accelZ);
            sane = (mag > 9.0f && mag < 10.6f);
        }
    }
    {
        char detail[64];
        char magText[16];
        if (isnan(mag)) snprintf(magText, sizeof(magText), "--");
        else            snprintf(magText, sizeof(magText), "%d.%02d",
                                 (int)mag, (int)((mag - (int)mag) * 100.0f));
        snprintf(detail, sizeof(detail), "init=%d, |a|=%s m/s2", (int)ist, magText);
        report("S9 IMU works after the short is removed", sane, detail);
    }

    // Bounded retry, not a single shot.
    //
    // initializeGPS_I2C() is a chain of UBX config commands, each waiting for an
    // ACK, issued against a module that is already streaming PVT at 4 Hz. That
    // is measurably not reliable first time: over four wedge/recover cycles the
    // call failed twice (-2 rate rejected, -6 config rejected) on the UNDISTURBED
    // attempt while succeeding 4/4 on the attempt straight after a bus recovery.
    // So the flakiness belongs to the config exchange, not to bus recovery — an
    // earlier reading of this test blamed recovery and the data disagreed.
    //
    // Asserting single-shot success would therefore test something the hardware
    // does not promise. What matters is that it comes up within a bounded number
    // of tries, which is exactly what production should be doing and currently
    // does not: mkr_zero.ino calls this once in setup() with no retry at all.
    static constexpr uint8_t GPS_INIT_ATTEMPTS = 3;
    GPSReturnStatus gst = GPSReturnStatus::NOK_INIT_FAILED;
    uint8_t attempt = 0;
    for (attempt = 1; attempt <= GPS_INIT_ATTEMPTS; attempt++) {
        gst = initializeGPS_I2C(myGNSS);
        if (gst == GPSReturnStatus::OK) break;
        delay(250);   // let the module finish whatever it was busy with
    }
    {
        char detail[64];
        snprintf(detail, sizeof(detail), "status=%d after %u attempt(s)",
                 (int)gst, (unsigned)attempt);
        report("S10 GNSS re-initialises within 3 attempts",
               gst == GPSReturnStatus::OK, detail);
    }

    Serial.println();
    Serial.print("phase 2: ");
    Serial.print(passCount);
    Serial.print(" passed, ");
    Serial.print(failCount);
    Serial.println(" failed");
}

/**
 * @brief Live SDA/SCL level readout, for finding the jumper contact.
 *
 * Phase 1 can only report "SDA is not low", which does not distinguish a jumper
 * on the wrong pin from one that is simply not making contact. Watching the
 * levels while the probe is moved answers that in seconds, and costs nothing
 * once the test is over.
 *
 * The pins are plain GPIO for the duration, so no I2C happens here; Wire is
 * handed back at the end.
 */
static void monitorLines(void)
{
    Serial.println();
    Serial.println("=== line monitor: SDA=D11 (PA22), SCL=D12 (PA23) ===");
    Serial.println("Touch the jumper to a GND pin and watch SDA go LOW. 15 s.");

    pinMode(PIN_WIRE_SDA, INPUT_PULLUP);
    pinMode(PIN_WIRE_SCL, INPUT_PULLUP);

    const unsigned long start = millis();
    unsigned long lastPrint = 0;
    while (!isTimeout(15000UL, start)) {
        if (isTimeout(250UL, lastPrint)) {
            lastPrint = millis();
            const bool sdaHigh = (digitalRead(PIN_WIRE_SDA) == HIGH);
            const bool sclHigh = (digitalRead(PIN_WIRE_SCL) == HIGH);
            Serial.print("  SDA(D11)=");
            Serial.print(sdaHigh ? "HIGH" : "LOW ");
            Serial.print("   SCL(D12)=");
            Serial.print(sclHigh ? "HIGH" : "LOW ");
            Serial.println(sdaHigh ? "" : "   <-- SDA is being held low");
        }
    }

    // Give the pins back to SERCOM2 so ordinary traffic works again.
    Wire.begin();
    Wire.setClock(IMU_I2C_CLOCK_HZ);
    Serial.println("monitor done, bus handed back to Wire");
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

    runFullTest();
}

void loop()
{
    if (Serial.available() > 0) {
        const int command = Serial.read();
        while (Serial.available() > 0) (void)Serial.read();   // drain

        if (command == 'a' || command == 'A') {
            runFullTest();
        } else if (command == 'w' || command == 'W') {
            const bool ok = wedgeBus();
            bool sdaHigh = false, sclHigh = false;
            sampleLines(sdaHigh, sclHigh);
            Serial.print("wedge: ");
            Serial.print(ok ? "applied" : "FAILED");
            Serial.print("  SDA=");
            Serial.print(sdaHigh ? "HIGH" : "LOW");
            Serial.print(" SCL=");
            Serial.println(sclHigh ? "HIGH" : "LOW");
        } else if (command == 'r' || command == 'R') {
            const I2CBusState st = i2cBusRecover();
            Serial.print("recover: state=");
            Serial.println((int)st);
        } else if (command == 'l' || command == 'L') {
            monitorLines();
        } else if (command == '1') {
            testStuckPhase1();
        } else if (command == '2') {
            testStuckPhase2();
        } else if (command == 'h' || command == 'H') {
            Serial.println("a = full wedge/recover test (no hardware needed)");
            Serial.println("w = wedge only, r = recover only");
            Serial.println("l = live SDA/SCL level monitor (find the jumper contact)");
            Serial.println("1 = stuck-bus phase 1 -- SHORT SDA (D11) to GND first");
            Serial.println("2 = stuck-bus phase 2 -- REMOVE the short first");
        }
    }

    static unsigned long lastBlink = 0;
    if (isTimeout(500UL, lastBlink)) {
        lastBlink = millis();
        digitalWrite(STATUS_INDICATOR, !digitalRead(STATUS_INDICATOR));
    }
}
