/**
 * BNO055SelfTest - asks the chip itself whether it is healthy, then stresses
 * the read path, through the SAME transport production uses.
 *
 * Written to separate "the part is faulty" from "the wiring/environment is"
 * after the car drives of 2026-09-26, where corrupt bursts arrived together
 * with false High-G latches. Four checks, then a summary repeated every 5 s:
 *
 *   1. IDENTITY - CHIP/ACC/MAG/GYR IDs and firmware revisions. A counterfeit or
 *      remarked part often gets these wrong.
 *   2. POWER-ON SELF TEST, five times - full system reset (SYS_TRIGGER RST_SYS),
 *      then ST_RESULT (bit 0 ACC, 1 MAG, 2 GYR, 3 MCU; 0x0F = all pass),
 *      SYS_STATUS and SYS_ERR.
 *   3. BUILT-IN SELF TEST, ten times - SYS_TRIGGER bit 0 in CONFIG mode, then
 *      the same registers. SYS_ERR 3 means "self test result failed".
 *   4. DATA, 60 s in IMUPLUS at 100 Hz, plus a back-to-back stress of CHIP_ID
 *      reads and 48-byte bursts. After a reset NO interrupt is enabled, so
 *      INT_STA must read 0 on every burst: a non-zero byte there can only be a
 *      corrupt read — the mechanism suspected of the false High-G events.
 *
 * Read-only apart from the resets and mode changes it needs; production
 * re-initialises the part at its next boot. Build with the production
 * --library flags (lib, vendor/Wire, vendor/CANBus, vendor/SdFat).
 */

#include <Wire.h>
#include <math.h>
#include <stdarg.h>
#include "DataDictionary.h"
#include "I2CBus.h"
#include "BNO055Regs.h"
#include "BNO055Transport.h"

static uint8_t gAddr = 0;
static char    gSum[900];

static void add(const char *fmt, ...)
{
    const size_t n = strlen(gSum);
    va_list ap; va_start(ap, fmt);
    vsnprintf(gSum + n, sizeof(gSum) - n, fmt, ap);
    va_end(ap);
}

static bool rd(uint8_t reg, uint8_t *buf, uint8_t n) { return bno055BusRead(gAddr, reg, buf, n) == 0; }
static bool wr(uint8_t reg, uint8_t v)               { return bno055BusWrite(gAddr, reg, &v, 1) == 0; }
static int  rd8(uint8_t reg)                         { uint8_t v; return rd(reg, &v, 1) ? v : -1; }

static bool waitChipId(uint32_t ms)
{
    const uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        if (rd8(BNO055_CHIP_ID_ADDR) == BNO055_EXPECTED_CHIP_ID) return true;
        delay(10);
    }
    return false;
}

static bool configMode()
{
    const bool ok = wr(BNO055_PAGE_ID_ADDR, 0) && wr(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
    delay(25);
    return ok;
}

static void runTests()
{
    gSum[0] = '\0';
    if (i2cBusBegin() != I2CBusState::Ready) { add("RESULT FAIL: I2C bus not ready (%s)\n", i2cStuckReason()); return; }
    gAddr = bno055FindAddress();
    if (!gAddr) { add("RESULT FAIL: no BNO055 at 0x28/0x29\n"); return; }

    // 1. identity
    uint8_t id[7] = {0};
    const bool idOk = rd(BNO055_CHIP_ID_ADDR, id, 7);
    add("addr 0x%02X  CHIP 0x%02X ACC 0x%02X MAG 0x%02X GYR 0x%02X  SW 0x%02X%02X  BL 0x%02X  (expect A0 FB 32 0F)%s\n",
        gAddr, id[0], id[1], id[2], id[3], id[5], id[4], id[6], idOk ? "" : "  READ FAILED");
    const bool idsGood = idOk && id[0] == 0xA0 && id[1] == 0xFB && id[2] == 0x32 && id[3] == 0x0F;

    // 2. power-on self test after full resets
    int postPass = 0;
    add("POST:");
    for (int i = 0; i < 5; ++i) {
        wr(BNO055_PAGE_ID_ADDR, 0);
        wr(BNO055_SYS_TRIGGER_ADDR, 0x20);        // RST_SYS
        delay(700);
        if (!waitChipId(1500)) { add(" noreply"); continue; }
        const int st = rd8(BNO055_ST_RESULT_ADDR), ss = rd8(BNO055_SYS_STATUS_ADDR), se = rd8(BNO055_SYS_ERR_ADDR);
        add(" %02X/%d/%d", st, ss, se);
        if (st >= 0 && (st & 0x0F) == 0x0F && se == 0) ++postPass;
    }
    add("  (ST_RESULT/SYS_STATUS/SYS_ERR) pass %d/5\n", postPass);

    // 3. built-in self test
    int bistPass = 0;
    add("BIST:");
    for (int i = 0; i < 10; ++i) {
        configMode();
        wr(BNO055_SYS_TRIGGER_ADDR, 0x01);        // Self_Test
        delay(600);
        const int st = rd8(BNO055_ST_RESULT_ADDR), se = rd8(BNO055_SYS_ERR_ADDR);
        add(" %02X/%d", st, se);
        if (st >= 0 && (st & 0x0F) == 0x0F && se == 0) ++bistPass;
    }
    add("  pass %d/10\n", bistPass);

    // 4a. data in IMUPLUS, 100 Hz for 60 s
    configMode();
    wr(BNO055_UNIT_SEL_ADDR, 0x00);
    wr(BNO055_OPR_MODE_ADDR, OPERATION_MODE_IMUPLUS);
    delay(1500);                                 // let the fusion produce its first estimates
    uint32_t reads = 0, fails = 0, badTemp = 0, badGrav = 0, intSta = 0, stChange = 0, idle = 0;
    double aSum = 0, aSq = 0, gSumD = 0, gSq = 0, tSum = 0; uint32_t nOk = 0;
    uint8_t stFirst = 0xFF;
    const uint32_t tEnd = millis() + 60000UL;
    uint32_t next = millis();
    while ((int32_t)(millis() - tEnd) < 0) {
        if ((int32_t)(millis() - next) < 0) continue;
        next += 10;
        uint8_t b[48];
        ++reads;
        if (!rd(0x08, b, 48)) { ++fails; continue; }
        const int16_t ax = (int16_t)(b[0] | b[1] << 8), ay = (int16_t)(b[2] | b[3] << 8), az = (int16_t)(b[4] | b[5] << 8);
        const int16_t gx = (int16_t)(b[12] | b[13] << 8), gy = (int16_t)(b[14] | b[15] << 8), gz = (int16_t)(b[16] | b[17] << 8);
        const int16_t vx = (int16_t)(b[38] | b[39] << 8), vy = (int16_t)(b[40] | b[41] << 8), vz = (int16_t)(b[42] | b[43] << 8);
        const int8_t  t  = (int8_t)b[44];
        if (b[47] != 0) ++intSta;
        if (stFirst == 0xFF) stFirst = b[46]; else if (b[46] != stFirst) ++stChange;
        if (t <= -40 || t >= 85) { ++badTemp; continue; }
        if (vx == 0 && vy == 0 && vz == 0) { ++idle; continue; }
        const float gm = sqrtf((float)vx * vx + (float)vy * vy + (float)vz * vz) * 0.01f;
        if (gm < 7.0f || gm > 12.0f) { ++badGrav; continue; }
        const float am = sqrtf((float)ax * ax + (float)ay * ay + (float)az * az) * 0.01f;
        const float wm = sqrtf((float)gx * gx + (float)gy * gy + (float)gz * gz) * 0.0625f;
        aSum += am; aSq += (double)am * am; gSumD += wm; gSq += (double)wm * wm; tSum += t; ++nOk;
    }
    const double aMean = nOk ? aSum / nOk : 0, aSd = nOk ? sqrt(fmax(0, aSq / nOk - aMean * aMean)) : 0;
    const double wMean = nOk ? gSumD / nOk : 0, wSd = nOk ? sqrt(fmax(0, gSq / nOk - wMean * wMean)) : 0;
    add("DATA 60 s @100 Hz: reads %lu fail %lu | bad temp %lu bad gravity %lu INT_STA!=0 %lu ST changes %lu (ST 0x%02X) idle %lu\n",
        reads, fails, badTemp, badGrav, intSta, stChange, stFirst, idle);
    // newlib-nano printf has no %f: milli-units as integers.
    add("  at rest: |a| %ld +/- %ld mm/s2, |w| %ld +/- %ld mdps, temp %ld C, calib 0x%02X\n",
        lround(aMean * 1000), lround(aSd * 1000), lround(wMean * 1000), lround(wSd * 1000),
        nOk ? lround(tSum / nOk) : 0L, rd8(BNO055_CALIB_STAT_ADDR));

    // 4b. back-to-back stress
    uint32_t idReads = 0, idBad = 0, idFail = 0, bReads = 0, bFail = 0, bInt = 0;
    const uint32_t s0 = millis();
    for (int i = 0; i < 20000; ++i) {
        uint8_t v; ++idReads;
        if (!rd(BNO055_CHIP_ID_ADDR, &v, 1)) ++idFail; else if (v != 0xA0) ++idBad;
    }
    for (int i = 0; i < 5000; ++i) {
        uint8_t b[48]; ++bReads;
        if (!rd(0x08, b, 48)) ++bFail; else if (b[47] != 0) ++bInt;
    }
    add("STRESS %lu ms: CHIP_ID %lu reads, %lu failed, %lu wrong | bursts %lu, %lu failed, %lu with INT_STA!=0 | transport errors total %lu\n",
        millis() - s0, idReads, idFail, idBad, bReads, bFail, bInt, (unsigned long)bno055TransportErrorCount());

    const bool clean = idsGood && postPass == 5 && bistPass == 10 && fails == 0 && badTemp == 0 && badGrav == 0 &&
                       intSta == 0 && idFail == 0 && idBad == 0 && bFail == 0 && bInt == 0;
    add("RESULT %s\n", clean ? "CLEAN - chip passes every check" : "ISSUES FOUND - see lines above");
}

void setup()
{
    Serial.begin(115200);
    const uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 5000u) {}
    Serial.println(F("# BNO055SelfTest: identity, POST x5, BIST x10, 60 s data, stress - about 90 s"));
    runTests();
    Serial.print(gSum);
}

void loop()
{
    delay(5000);
    Serial.print(gSum);
}
