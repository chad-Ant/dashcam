/**
 * @file BNO055Validation.ino
 * @brief Proves the BNO055 transport before anything is built on top of it.
 *
 * Phase 1's gate. It answers, in order, the questions that have to be settled
 * before the driver can be trusted with vehicle data:
 *
 *   1. Which address is the part actually at? The datasheet default is 0x29,
 *      the driver's built-in constant is 0x28, and guessing is a coin toss.
 *   2. Does CHIP_ID read 0xA0 REPEATEDLY, with zero faults? One good read
 *      proves nothing - the sensor this replaces returned corrupt values from
 *      transactions that succeeded, 40 % of the time, and only a long run
 *      showed it.
 *   3. Does the shared bus survive? The datasheet says the BNO055 stretches the
 *      clock, and the GNSS is on the same wires.
 *
 * No fusion, no modes, no orientation. Those come in later phases; this sketch
 * exists so that when they misbehave, the transport is already ruled out.
 */

#include <Wire.h>

#include "BNO055Transport.h"
#include "I2CBus.h"

static BNO055Device gDev;
static uint8_t  gAddr    = 0;

static uint32_t gReads   = 0;
static uint32_t gBadId   = 0;
static uint32_t gFailed  = 0;
static uint8_t  gLastId  = 0;

/// Reads per second. Well above the 100 Hz the fusion path will eventually poll
/// at, so if the bus is marginal this finds it faster than production would.
static const uint32_t READ_INTERVAL_MS = 5;
static const uint32_t REPORT_MS        = 5000;

void setup()
{
    Serial.begin(115200);
    const uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 3000) { }

    watchdogArm(8000UL);

    Serial.println();
    Serial.println(F("============ BNO055 transport validation ============"));
    Serial.println(F("Phase 1 gate: address, CHIP_ID stability, bus health."));

    Wire.begin();
    Wire.setClock(BNO055_I2C_CLOCK_HZ);
    Serial.print(F("I2C clock: "));
    Serial.print(BNO055_I2C_CLOCK_HZ / 1000UL);
    Serial.println(F(" kHz (conservative - the part stretches the clock)"));

    // The part needs 400 ms from power-on before it answers at all. The sketch
    // has usually spent longer than that waiting for Serial, but not always.
    if (millis() < 500UL) delay(500UL - millis());
    watchdogFeed();

    gAddr = bno055FindAddress();
    if (gAddr == 0u) {
        Serial.println(F("\nNo BNO055 found at 0x28 or 0x29."));
        Serial.print  (F("  I2C bus state: "));
        Serial.println(i2cStuckReason());
        Serial.println(F("  Identified by CHIP_ID 0xA0, not by a bare address ACK,"));
        Serial.println(F("  so a device that answers but is not a BNO055 reads as absent."));
        return;
    }

    Serial.print(F("\nFound at 0x"));
    Serial.print(gAddr, HEX);
    Serial.println(gAddr == 0x29u ? F("  (datasheet default)")
                                  : F("  (alternative strapping)"));

    const bool identified = bno055Identify(gDev, gAddr);

    Serial.print(F("bno055Identify() -> "));
    Serial.print(identified ? F("true") : F("FALSE"));
    Serial.print(F("   chip_id 0x"));
    Serial.print(gDev.chipId, HEX);
    // The return now means what it looks like. Its predecessor, bno055_init(),
    // reassigned its status from each of its reads in turn and so reported only
    // the last one — a success that said nothing about whether the part had
    // answered, which is why this line used to check chip_id separately.
    Serial.println(identified ? F("  OK") : F("  *** WRONG or UNREADABLE ***"));

    // Full 16 bits. The vendored driver stored this in a uint8 and dropped the
    // major version, so a part running 3.08 printed as 0x8 and looked exactly
    // like a failed second byte of a two-byte read.
    Serial.print(F("sw rev 0x"));   Serial.print(gDev.swRevId, HEX);
    Serial.print(F("  accel 0x"));  Serial.print(gDev.accelRevId, HEX);
    Serial.print(F("  mag 0x"));    Serial.print(gDev.magRevId, HEX);
    Serial.print(F("  gyro 0x"));   Serial.println(gDev.gyroRevId, HEX);

    Serial.println(F("\nHammering CHIP_ID. Any non-zero fault or wrong id is a FAIL.\n"));
}

void loop()
{
    watchdogFeed();
    if (gAddr == 0u) { delay(1000); return; }

    static uint32_t lastRead   = 0;
    static uint32_t lastReport = 0;

    if ((millis() - lastRead) >= READ_INTERVAL_MS) {
        lastRead = millis();
        uint8_t id = 0u;
        ++gReads;
        if (bno055BusRead(gAddr, BNO055_CHIP_ID_ADDR, &id, 1u) != 0) {
            ++gFailed;
        } else {
            gLastId = id;
            // A successful transfer returning the wrong byte is the failure that
            // matters here, and it is counted separately: the previous IMU died
            // exactly this way, with the bus reporting success throughout.
            if (id != BNO055_EXPECTED_CHIP_ID) ++gBadId;
        }
    }

    if ((millis() - lastReport) >= REPORT_MS) {
        lastReport = millis();
        Serial.print(F("reads "));       Serial.print(gReads);
        Serial.print(F("  failed "));    Serial.print(gFailed);
        Serial.print(F("  wrong id "));  Serial.print(gBadId);
        Serial.print(F("  last 0x"));    Serial.print(gLastId, HEX);
        Serial.print(F("  consecutive faults ")); Serial.print(bno055TransportFaults());
        if (gFailed == 0u && gBadId == 0u) Serial.println(F("   CLEAN"));
        else                               Serial.println(F("   *** FAULTS ***"));
    }
}
