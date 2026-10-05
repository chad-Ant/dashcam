/**
 * @file can_telemetry_tests.cpp
 * @brief What the C3 is sent while the master is in DISCOVER: no CAN-derived
 *        value at all, every one of them at its "unavailable" sentinel.
 *
 * The real path end to end: frames from the bus side of the MCP2515 model, the
 * drain ISR, the ring, tickCANSniff(), and the real telemetry builder
 * (lib/CommunicationFunctions.cpp), all compiled unmodified.
 *
 * Its own suite, apart from can_drain_tests, because CommProtocol.h declares
 * the payload with a GCC packing attribute: this one needs g++ or clang and is
 * not in RunTests.cmd (MSVC), while the drain suite stays portable.
 *
 * Exit status is the number of failures, so `make check` fails the build.
 */

#include "CANSniffFunctions.h"
#include "CANStreamHw.h"
#include "CommunicationFunctions.h"
#include "OBD2Functions.h"
#include "GPSFunctions.h"
#include "DataDictionary.h"
#include "SDFunctions.h"
#include "mcp2515_model.h"
#include "can_stream_hw_stub.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// ─── minimal harness ─────────────────────────────────────────────────────────

static int         g_checks = 0;
static int         g_fails  = 0;
static const char *g_case   = "";

static void check(bool ok, const char *expr, int line)
{
    ++g_checks;
    if (!ok) {
        ++g_fails;
        printf("  FAIL  line %-4d  %s\n              in: %s\n", line, expr, g_case);
    }
}

#define CHECK(cond) check((cond), #cond, __LINE__)

// ─── no card on the host ─────────────────────────────────────────────────────

bool sdReady()                             { return false; }
bool sdOpenRoot(File32 &)                  { return false; }
bool sdOpenRead(const char *, File32 &)    { return false; }
bool sdReadLine(File32 &, char *, size_t)  { return false; }

// ─── helpers ─────────────────────────────────────────────────────────────────

static VehicleSignals g_v;
static YawEstimator   g_y;
static uint32_t       g_t = 0;   ///< millis(); micros() is kept at 1000x it

static void setClock(uint32_t ms)
{
    g_t = ms;
    hostSetMillis(g_t);
    hostSetMicros(g_t * 1000u);
}

static void advance(uint32_t ms) { setClock(g_t + ms); }

static bool setMode(CanMode m)
{
    // Clear of the driver's rate limit, as every transition in the sketch is.
    advance(CAN_MODE_MIN_INTERVAL_MS + 100u);
    const CanModeStatus st = canSetMode(m, MCP2515_DEFAULT_CS_PIN);
    return st == CanModeStatus::OK || st == CanModeStatus::UNCHANGED;
}

static void begin(const char *name)
{
    g_case = name;
    hostReset();
    fakeCanReset();
    hostDrainReset();
    canDrainTimerBegin();
    canSniffSetMap(nullptr);
    // Forward only, across tests too: the driver's rate limiter and the
    // indicator holds keep their own clocks between them.
    setClock(g_t + 100000u);
    (void)setMode(CanMode::OFF);
    (void)canRawDiscard(canRawStreamable());   // leftovers of earlier tests
    initVehicleSignals(g_v);
    initYawEstimator(g_y, canSniffGetMap());
}

/// Writes @p value into @p d as the decoder will read it back (Motorola, MSB at
/// @p start, walking down and wrapping to bit 7 of the next byte).
static void putMotorola(uint8_t *d, uint8_t start, uint8_t len, uint32_t value)
{
    uint8_t byteIx = (uint8_t)(start >> 3);
    uint8_t bitIx  = (uint8_t)(start & 7u);
    for (int8_t b = (int8_t)len - 1; b >= 0; --b) {
        const uint8_t bit = (uint8_t)((value >> b) & 1u);
        d[byteIx] = (uint8_t)((d[byteIx] & ~(1u << bitIx)) | (bit << bitIx));
        if (bitIx == 0u) { bitIx = 7u; ++byteIx; } else { --bitIx; }
    }
}

/// One pass of the receive path, in the sketch's order. The release stands in
/// for the USB stream, which in SNIFF never passes the decoder.
static void pass()
{
    hostDrainTick();
    (void)tickCANSniff(g_v, g_y);
    (void)canRawDiscard(canRawStreamable());
}

/// One frame per pass: under SNIFF's filters an id that only RXB1's filters
/// admit has a single buffer (BUKT rolls over from RXB0 only), exactly as on
/// the silicon, so two such frames before a drain would lose one.
static void sendOne(uint16_t id, uint8_t dlc, const uint8_t *d)
{
    CHECK(fakeCanFrame(id, dlc, d));
    pass();
}

static void sendGoldenFrames()
{
    uint8_t d[8];

    memset(d, 0, 8); d[0] = 0x12; d[1] = 0x34;                   // 0x158 speed 4660 x 0.01
    sendOne(0x158, 8, d);
    memset(d, 0, 8); d[0] = 0x80; d[2] = 0x0B; d[3] = 0xB8;      // 0x17C pedal 128, rpm 3000
    d[4] = 0x01; d[6] = 0x20;                                    //       brake switch, pressed
    sendOne(0x17C, 8, d);
    memset(d, 0, 8); d[5] = 0x04;                                // 0x191 gear raw 4 = D
    sendOne(0x191, 8, d);
    memset(d, 0, 8); d[0] = 0x01; d[1] = 0x2C;                   // 0x1AB torque 300
    sendOne(0x1AB, 3, d);
    memset(d, 0, 8);                                             // 0x1D0 wheels
    putMotorola(d,  7, 15, 4000u);
    putMotorola(d,  8, 15, 4010u);
    putMotorola(d, 25, 15, 4020u);
    putMotorola(d, 42, 15, 4030u);
    sendOne(0x1D0, 8, d);
    memset(d, 0, 8); d[0] = 0x20;                                // 0x294 left lamp lit
    sendOne(0x294, 8, d);
}

// ═════════════════════════════════════════════════════════════════════════════
// what the C3 is sent in DISCOVER
// ═════════════════════════════════════════════════════════════════════════════

/// OBD2Data as setup() and every applyCanMode() leave it (initOBD2Data()).
static void obdUnavailable(OBD2Data &o)
{
    o.rpm = o.speed = o.coolantTemp = o.fuelLevel = o.fuelRate = o.throttle =
    o.engineLoad = o.airPressure = o.gear = o.gearRatio = o.odo = NAN;
    o.lastUpdateMs = 0u;
    for (uint8_t i = 0; i < OBD2_FIELD_COUNT; ++i) o.fieldMs[i] = 0u;
}

static void test_discover_telemetry_unavailable()
{
    // Decision 3: no CAN decoding on the MKR by default, and the C3 telemetry's
    // CAN-derived fields must read as UNAVAILABLE — never zeros dressed as
    // measurements. Live map traffic flows through the whole DISCOVER path
    // first, after a SNIFF session, so stale state would have every chance.
    begin("telemetry: in DISCOVER every CAN-derived field is at its sentinel");

    // A SNIFF session that decodes real values...
    CHECK(setMode(CanMode::SNIFF));
    sendGoldenFrames();
    CHECK(g_v.speedSrc == VehSource::CAN_SNIFF);
    OBD2Data obd; obdUnavailable(obd);
    GPSData gps = GPSData();
    IMUData imu = IMUData();
    SwitchData sw = SwitchData();
    DerivedSignals derived;
    TelemetryPayload sniffOut;
    buildTelemetry(obd, gps, imu, sw, derived, g_v, (uint8_t)canGetMode(), sniffOut);
    CHECK(sniffOut.sigSource != 0u);               // the check below is not vacuous
    CHECK(sniffOut.canMapFlags == 0x03u);          // map filters, from the map

    // ...then DISCOVER, as applyCanMode() does it: the template blanked, and
    // the same traffic still flowing.
    CHECK(setMode(CanMode::DISCOVER));
    initVehicleSignals(g_v);
    initYawEstimator(g_y, canSniffGetMap());
    obdUnavailable(obd);
    for (unsigned i = 0; i < 20u; ++i) { sendGoldenFrames(); advance(10u); }
    expireVehicleSignals(g_v, g_t, VEH_FRESH_SNIFF_MS, VEH_FRESH_OBD2_MS);

    // The acceleration estimator's input in loop(): sniffed speed if any, else
    // OBD-II speed. Both NaN, so it is fed NaN and publishes NaN — never a
    // derivative of a fake zero speed.
    const float accelInput = (g_v.speedSrc != VehSource::NONE && !isnan(g_v.speedKmh))
                                 ? g_v.speedKmh : obd.speed;
    CHECK(isnan(accelInput));

    TelemetryPayload out;
    buildTelemetry(obd, gps, imu, sw, derived, g_v, (uint8_t)canGetMode(), out);
    CHECK(out.canMode == 1u);
    CHECK(isnan(out.speed) && isnan(out.rpm) && isnan(out.gear) && isnan(out.accel));
    CHECK(out.sigSource == 0u);
    CHECK(out.gearPos == 0u);
    CHECK(out.vehFlags == 0u);                     // validity bits clear, no hazard
    CHECK(out.pedalGas == 0u);                     // qualified by PEDAL_VALID, clear
    CHECK(out.steerMotorTorque == 0xFFFFu);
    CHECK(out.yawRateCdps == INT16_MIN);
    for (uint8_t i = 0; i < 4u; ++i) CHECK(out.wheelRaw[i] == 0xFFFFu);
    CHECK((out.flags & COMM_FLAG_OBD2_VALID) == 0u);
    // Accept-all, and said so: the filter bookkeeping used to keep SNIFF's set.
    CHECK(out.canMapFlags == 0u);
}

// ─── runner ──────────────────────────────────────────────────────────────────

int main()
{
    test_discover_telemetry_unavailable();

    printf("can_telemetry_tests: %d checks, %d failed\n", g_checks, g_fails);
    return g_fails;
}
