/**
 * @file can_drain_tests.cpp
 * @brief The timer-driven drain in lib/CANSniffFunctions.cpp, the modes that arm
 *        it and the SPI masking around them, and SNIFF's decode from the ring.
 *
 * CANSniffFunctions.cpp, CANMap.cpp and VehicleSignals.cpp compile UNMODIFIED. Frames enter the
 * MCP2515 model from the bus side (mcp2515_model.cpp), the drain ISR runs when
 * the test ticks the timer (can_stream_hw_stub.cpp), and every main-loop SPI
 * transaction is checked against the drain's mask.
 *
 * The map is the compiled-in Honda one: 0x158 0x17C 0x191 0x1AB 0x1D0 0x294.
 *
 * Exit status is the number of failures, so `make check` fails the build.
 */

#include "CANSniffFunctions.h"
#include "CANStreamHw.h"
#include "DataDictionary.h"
#include "SDFunctions.h"
#include "mcp2515_model.h"
#include "can_stream_hw_stub.h"

#include <SPI.h>
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

static const uint8_t REG_CANINTF      = 0x2C;
static const uint8_t REG_EFLG         = 0x2D;
static const uint8_t REG_TXB0CTRL     = 0x30;
static const uint8_t MODE_LISTEN_ONLY = 0x60;
static const uint8_t MODE_CONFIG      = 0x80;

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

/// Frames waiting for the stream, oldest first, taken out of the ring.
static uint16_t takeAll(CanRawFrame *out, uint16_t cap)
{
    uint16_t n = 0;
    CanRawFrame f;
    while (n < cap && canRawFront(f)) { out[n++] = f; canRawPop(); }
    return n;
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

// ═════════════════════════════════════════════════════════════════════════════
// the ISR drain
// ═════════════════════════════════════════════════════════════════════════════

static void test_drain_both_buffers_in_order()
{
    begin("drain: one tick empties RXB0 then RXB1, stamped, in arrival order");

    CHECK(setMode(CanMode::DISCOVER));
    CHECK(canDrainArmed());
    CHECK(fakeCanOpMode() == MODE_LISTEN_ONLY);

    const uint8_t a[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    const uint8_t b[3] = { 0xAA, 0xBB, 0xCC };
    CHECK(fakeCanFrame(0x100, 8, a));       // RXB0
    CHECK(fakeCanFrame(0x7FF, 3, b));       // BUKT rollover into RXB1
    CHECK((fakeCanReg(REG_CANINTF) & 0x03u) == 0x03u);

    const uint32_t drained = canDrainedCount();
    hostSetMicros(0x12345678u);
    hostDrainTick();

    CHECK(canDrainedCount() == drained + 2u);
    CHECK((fakeCanReg(REG_CANINTF) & 0x03u) == 0u);   // both buffers free again

    CanRawFrame f[4];
    CHECK(takeAll(f, 4) == 2u);
    CHECK(canRawId(f[0]) == 0x100u && canRawDlc(f[0]) == 8u && memcmp(f[0].data, a, 8) == 0);
    CHECK(canRawId(f[1]) == 0x7FFu && canRawDlc(f[1]) == 3u && memcmp(f[1].data, b, 3) == 0);
    CHECK(f[0].us == 0x12345678u && f[1].us == 0x12345678u);
    CHECK(!canRawExt(f[0]) && !canRawRtr(f[0]));
}

static void test_drain_timestamps_follow_micros()
{
    begin("drain: each frame carries micros() at the tick that read it");

    CHECK(setMode(CanMode::DISCOVER));
    const uint32_t stamps[3] = { 1000u, 0xFFFFFF00u, 0x00000100u };   // across the wrap
    for (unsigned i = 0; i < 3u; ++i) {
        CHECK(fakeCanFrame((uint16_t)(0x200 + i), 1));
        hostSetMicros(stamps[i]);
        hostDrainTick();
    }
    CanRawFrame f[4];
    CHECK(takeAll(f, 4) == 3u);
    for (unsigned i = 0; i < 3u; ++i) CHECK(f[i].us == stamps[i]);
}

static void test_drain_extended_and_remote()
{
    // Every frame is evidence for the Orin now; rejecting them is the decoder's
    // job, not the drain's.
    begin("drain: extended and remote frames are queued, flagged, with no stale payload");

    CHECK(setMode(CanMode::DISCOVER));
    const uint8_t full[8] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04 };

    CHECK(fakeCanFrameEx(0x18DAF110u, true, false, 8, full));
    hostDrainTick();
    // A remote frame stores no data: RXB0 still holds the previous payload.
    CHECK(fakeCanFrameEx(0x123u, false, true, 4, nullptr));
    hostDrainTick();
    CHECK(fakeCanFrameEx(0x0ABCDEFu, true, true, 0, nullptr));
    CHECK(fakeCanFrameEx(0x158u, true, false, 2, full));   // ext id with a mapped id's value
    hostDrainTick();

    CanRawFrame f[8];
    CHECK(takeAll(f, 8) == 4u);
    CHECK(canRawExt(f[0]) && !canRawRtr(f[0]) && canRawId(f[0]) == 0x18DAF110u);
    CHECK(canRawDlc(f[0]) == 8u && memcmp(f[0].data, full, 8) == 0);

    CHECK(!canRawExt(f[1]) && canRawRtr(f[1]) && canRawId(f[1]) == 0x123u && canRawDlc(f[1]) == 4u);
    for (unsigned i = 0; i < 7u; ++i) CHECK(f[1].data[i] == 0u);   // not 0xDE 0xAD...

    CHECK(canRawExt(f[2]) && canRawRtr(f[2]) && canRawId(f[2]) == 0x0ABCDEFu && canRawDlc(f[2]) == 0u);
    CHECK(canRawExt(f[3]) && !canRawRtr(f[3]) && canRawId(f[3]) == 0x158u && canRawDlc(f[3]) == 2u);
}

static void test_drain_dlc_clamped()
{
    begin("drain: a raw DLC of 9-15 is queued as 8");

    CHECK(setMode(CanMode::DISCOVER));
    const uint8_t d[8] = { 9, 8, 7, 6, 5, 4, 3, 2 };
    CHECK(fakeCanFrameEx(0x321u, false, false, 15, d));
    hostDrainTick();
    CanRawFrame f[2];
    CHECK(takeAll(f, 2) == 1u);
    CHECK(canRawDlc(f[0]) == 8u && memcmp(f[0].data, d, 8) == 0);
}

static void test_drain_counts_overruns()
{
    begin("drain: overruns are counted per latched flag, cleared, and boot-cumulative");

    CHECK(setMode(CanMode::DISCOVER));
    const uint32_t before = canSniffOverrunCount();

    // RXB0, RXB1 by rollover, then three with nowhere to go — one RX1OVR.
    for (unsigned k = 0; k < 5u; ++k) (void)fakeCanFrame((uint16_t)(0x100 + k));
    CHECK(fakeCanOverflowed() == 3u);
    const uint32_t drained = canDrainedCount();
    hostDrainTick();
    CHECK(canDrainedCount() == drained + 2u);         // the three lost were never read
    CHECK(canSniffOverrunCount() == before + 1u);     // a lower bound: 3 frames, 1 event
    CHECK((fakeCanReg(REG_EFLG) & 0xC0u) == 0u);

    // A tick that finds nothing adds nothing.
    hostDrainTick();
    CHECK(canSniffOverrunCount() == before + 1u);

    // Both flags at once are two events.
    (void)fakeCanFrame(0x101);
    fakeCanSetReg(REG_EFLG, 0xC0u);
    hostDrainTick();
    CHECK(canSniffOverrunCount() == before + 3u);
}

static void test_drain_refuses_a_floating_miso()
{
    // REGRESSION GUARD. With the shield gone MISO floats; 0xFF claims both
    // buffers full on every tick and would queue frames made of 0xFF.
    begin("drain: a reply no listen-only controller could give queues nothing");

    CHECK(setMode(CanMode::DISCOVER));
    const uint32_t drained = canDrainedCount();
    const uint16_t queued  = canRawStreamable();

    fakeCanStickMiso(true, 0xFF);
    for (unsigned i = 0; i < 10u; ++i) hostDrainTick();
    fakeCanStickMiso(true, 0x00);
    for (unsigned i = 0; i < 10u; ++i) hostDrainTick();
    fakeCanStickMiso(false);

    CHECK(canDrainedCount() == drained);
    CHECK(canRawStreamable() == queued);
}

static void test_drain_bounded_per_tick()
{
    // A reply that looks plausible (RX0IF|RX1IF, no TX flags) but never clears:
    // without the bound the ISR would loop for ever.
    begin("drain: a controller that never empties costs at most CAN_DRAIN_MAX_PER_TICK reads a tick");

    CHECK(setMode(CanMode::DISCOVER));
    const uint32_t drained = canDrainedCount();
    fakeCanStickMiso(true, 0x03);
    hostDrainTick();
    fakeCanStickMiso(false);
    CHECK(canDrainedCount() == drained + CAN_DRAIN_MAX_PER_TICK);
}

static void test_drain_ignores_pending_txreq()
{
    // An OBD2 request left pending can keep a TXREQ bit set through the switch
    // to DISCOVER; treating that as "not the controller" would silence the
    // drain for the whole session.
    begin("drain: a TXREQ bit left set does not stop the drain");

    CHECK(setMode(CanMode::DISCOVER));
    fakeCanSetReg(REG_TXB0CTRL, 0x08u);
    CHECK(fakeCanFrame(0x300, 2));
    const uint32_t drained = canDrainedCount();
    hostDrainTick();
    CHECK(canDrainedCount() == drained + 1u);
}

static void test_drain_full_ring_drops_newest()
{
    begin("drain: a full ring refuses and counts — and the frame still counts as drained");

    CHECK(setMode(CanMode::DISCOVER));
    const uint32_t drained = canDrainedCount();
    const uint32_t drops   = canRingDropCount();

    for (uint32_t n = 0; n < CAN_RING_SLOTS + 10u; ++n) {
        (void)fakeCanFrame((uint16_t)(n & 0x7FFu), 1);
        hostDrainTick();
    }
    CHECK(canRawStreamable() == CAN_RING_SLOTS);
    CHECK(canRingDropCount() == drops + 10u);
    CHECK(canDrainedCount() == drained + CAN_RING_SLOTS + 10u);

    CanRawFrame f;
    CHECK(canRawFront(f) && canRawId(f) == 0u);   // the oldest were kept
}

// ═════════════════════════════════════════════════════════════════════════════
// modes arm the drain; every main-loop bus access masks it
// ═════════════════════════════════════════════════════════════════════════════

static void test_modes_arm_and_disarm()
{
    begin("modes: DISCOVER and SNIFF arm the drain; OFF, OBD2 and every failure disarm it");

    CHECK(!canDrainArmed());
    CHECK(setMode(CanMode::DISCOVER)); CHECK(canDrainArmed());
    CHECK(setMode(CanMode::SNIFF));    CHECK(canDrainArmed());
    CHECK(setMode(CanMode::OBD2));     CHECK(!canDrainArmed());
    CHECK(setMode(CanMode::DISCOVER)); CHECK(canDrainArmed());
    CHECK(setMode(CanMode::OFF));      CHECK(!canDrainArmed());

    // A refused filter write on the way into SNIFF parks the controller OFF —
    // and the drain with it.
    CHECK(setMode(CanMode::DISCOVER));
    fakeCanRefuseFilterWrites(true);
    CHECK(!setMode(CanMode::SNIFF));
    fakeCanRefuseFilterWrites(false);
    CHECK(canGetMode() == CanMode::OFF);
    CHECK(!canDrainArmed());
    CHECK(fakeCanOpMode() == MODE_CONFIG);

    // A refused host filter write inside SNIFF: the same.
    CHECK(setMode(CanMode::SNIFF));
    const uint16_t host[] = { 0x158 };
    fakeCanRefuseFilterWrites(true);
    CHECK(!canSniffSetFilters(host, 1u));
    fakeCanRefuseFilterWrites(false);
    CHECK(!canDrainArmed());
    CHECK(hostDrainMaskDepth() == 0u);
}

static void test_obd2_replies_are_not_drained()
{
    // The receive buffers in OBD2 hold the ECU's answer tickOBD2() is waiting
    // for; a drain that took them would starve the poller.
    begin("modes: in OBD2 the drain leaves the buffers alone");

    CHECK(setMode(CanMode::OBD2));
    CHECK(fakeCanOpMode() == 0x00);
    CHECK(fakeCanFrame(0x7E8, 8));
    const uint32_t drained = canDrainedCount();
    hostDrainTick();
    CHECK(canDrainedCount() == drained);
    CHECK((fakeCanReg(REG_CANINTF) & 0x01u) != 0u);
}

static void test_main_loop_spi_is_masked()
{
    // REGRESSION GUARD for SPI exclusivity. Preemption on: a tick falls due at
    // every byte the main loop clocks, and runs right there unless masked. A
    // single unmasked transaction while armed is a violation, and would also
    // corrupt the model's transaction the way a real ISR corrupts the bus.
    begin("masking: every bus access from the main loop holds the drain off");

    hostDrainSetPreempt(true);
    const uint16_t host[] = { 0x158, 0x17C, 0x191 };
    CanProbeState p = { CanProbeStage::Idle, 0, false };

    CHECK(setMode(CanMode::DISCOVER));
    CHECK(fakeCanFrame(0x100, 1));
    hostDrainTick();
    CHECK(setMode(CanMode::SNIFF));
    CHECK(canSniffFiltersFromMap());
    CHECK(canSniffSetFilters(host, 3u));          // armed throughout
    CHECK(canSniffFilterCount() == 3u);
    CHECK(canProbeArm(p));                         // accept-all write, armed
    CHECK(canSniffFilterCount() == 0u);
    canProbeSkip(p);                               // map filters back, armed
    CHECK(canSniffFiltersFromMap());
    CHECK(canSniffApplyMapFilters());
    CHECK(setMode(CanMode::DISCOVER));
    CHECK(setMode(CanMode::OBD2));
    CHECK(setMode(CanMode::DISCOVER));
    CHECK(fakeCanOpMode() == MODE_LISTEN_ONLY);

    CHECK(hostDrainViolations() == 0u);
    CHECK(hostDrainMaskDepth() == 0u);
    CHECK(hostDrainUnbalanced() == 0u);

    // The detector itself works: an unmasked transaction while armed counts.
    hostDrainSetPreempt(false);
    SPI.beginTransaction(SPISettings(10000000u, MSBFIRST, SPI_MODE0));
    SPI.endTransaction();
    CHECK(hostDrainViolations() == 1u);
}

static void test_tick_due_while_masked_runs_at_unmask()
{
    begin("masking: a tick that falls due during a filter write runs when it ends");

    CHECK(setMode(CanMode::SNIFF));
    CHECK(fakeCanFrame(0x158, 8));                 // waiting in RXB0 meanwhile
    const uint32_t runs    = hostDrainIsrRuns();
    const uint32_t drained = canDrainedCount();

    hostDrainSetPreempt(true);
    const uint16_t ids[] = { 0x158 };
    CHECK(canSniffSetFilters(ids, 1u));
    hostDrainSetPreempt(false);

    CHECK(hostDrainIsrRuns() == runs + 1u);        // exactly once, after the unmask
    CHECK(canDrainedCount() == drained + 1u);
    CHECK(!hostDrainPending());
    CHECK(hostDrainViolations() == 0u);
}

// ═════════════════════════════════════════════════════════════════════════════
// liveness: the drain, not the decoder
// ═════════════════════════════════════════════════════════════════════════════

static void test_liveness_counter()
{
    // The sketch stamps vehicle liveness whenever canDrainedCount() moves. In
    // DISCOVER nothing decodes, so this is the ONLY evidence a moving car is
    // moving, and it must not depend on the ring or the USB host.
    begin("liveness: every frame read moves the drained count; nothing else does");

    CHECK(setMode(CanMode::DISCOVER));
    uint32_t seen = canDrainedCount();
    hostDrainTick();
    CHECK(canDrainedCount() == seen);              // a quiet bus is not evidence

    CHECK(fakeCanFrame(0x1DC, 7));
    hostDrainTick();
    CHECK(canDrainedCount() == seen + 1u);
    seen = canDrainedCount();

    // Ring full and nobody streaming: still evidence.
    for (uint32_t n = 0; n < CAN_RING_SLOTS + 5u; ++n) { (void)fakeCanFrame(0x1DC, 7); hostDrainTick(); }
    CHECK(canDrainedCount() == seen + CAN_RING_SLOTS + 5u);
    CHECK(tickCANSniff(g_v, g_y) == 0u);           // and no decoding was involved
}

// ═════════════════════════════════════════════════════════════════════════════
// SNIFF decodes from the ring; DISCOVER decodes nothing
// ═════════════════════════════════════════════════════════════════════════════

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

static void test_sniff_golden_decode()
{
    begin("sniff: frames drained by the ISR decode to the map's values");

    CHECK(setMode(CanMode::SNIFF));
    sendGoldenFrames();

    CHECK(fabsf(g_v.speedKmh - 46.60f) < 0.001f && g_v.speedSrc == VehSource::CAN_SNIFF);
    CHECK(g_v.rpm == 3000.0f && g_v.rpmSrc == VehSource::CAN_SNIFF);
    CHECK(g_v.pedalGas == 128u);
    CHECK(g_v.brakeSwitch && g_v.brakePressed);
    CHECK(g_v.gear == VehGear::DRIVE);
    CHECK(g_v.steerMotorTorque == 300u);
    CHECK(g_v.wheelRaw[VEH_WHEEL_FL] == 4000u && g_v.wheelRaw[VEH_WHEEL_FR] == 4010u);
    CHECK(g_v.wheelRaw[VEH_WHEEL_RL] == 4020u && g_v.wheelRaw[VEH_WHEEL_RR] == 4030u);
    CHECK(g_v.turnLeft && !g_v.turnRight);
    CHECK(canSniffMatchCount() >= 6u);
}

static void test_sniff_stamps_arrival_time()
{
    // A frame can sit in the ring for most of a second behind a long pass. Its
    // signals must age from when the controller delivered it.
    begin("sniff: a frame decoded late is stamped with its arrival time");

    CHECK(setMode(CanMode::SNIFF));
    const uint8_t d[8] = { 0x01, 0x00, 0, 0, 0, 0, 0, 0 };
    CHECK(fakeCanFrame(0x158, 8, d));
    const uint32_t arrival = g_t;
    hostDrainTick();
    advance(300u);                                 // a 300 ms pass
    CHECK(tickCANSniff(g_v, g_y) == 1u);
    CHECK(g_v.speedMs == arrival);
}

static void test_sniff_skips_remote_frames()
{
    // A remote frame stores no data, so RXB0 still holds the previous frame's
    // payload; decoding it would republish a stale speed under a live ID.
    // (Extended ids cannot reach SNIFF's decoder at all: its filters are
    // standard ones, and an MCP2515 filter with EXIDE clear never matches an
    // extended frame, whatever the mask. The decoder refuses them anyway.)
    begin("sniff: a remote frame is taken and counted, never decoded");

    CHECK(setMode(CanMode::SNIFF));
    const uint8_t d[8] = { 0x7F, 0xFF, 0, 0, 0, 0, 0, 0 };
    CHECK(fakeCanFrame(0x17C, 8, d));              // leaves 0x7F 0xFF in RXB0's data
    pass();
    initVehicleSignals(g_v);
    CHECK(fakeCanFrameEx(0x158u, false, true, 8, nullptr));   // remote, mapped id
    CHECK(fakeCanReg(0x66) == 0x7Fu);              // the stale byte really is there
    const uint32_t frames = canSniffFrameCount(), matches = canSniffMatchCount();
    pass();
    CHECK(canSniffFrameCount() == frames + 1u);
    CHECK(canSniffMatchCount() == matches);
    CHECK(isnan(g_v.speedKmh) && g_v.speedSrc == VehSource::NONE);
    CHECK(!fakeCanFrameEx(0x158u, true, false, 8, d));        // filtered in hardware
}

static void test_discover_decodes_nothing()
{
    begin("discover: the ring fills with map frames and nothing is decoded");

    CHECK(setMode(CanMode::DISCOVER));
    const uint32_t matches = canSniffMatchCount();
    sendGoldenFrames();
    CHECK(canSniffMatchCount() == matches);
    CHECK(isnan(g_v.speedKmh) && isnan(g_v.rpm));
    CHECK(g_v.speedSrc == VehSource::NONE && g_v.rpmSrc == VehSource::NONE);
    CHECK(g_v.gear == VehGear::UNKNOWN && g_v.steerMotorTorque == VEH_TORQUE_INVALID);
    CHECK(g_v.wheelRaw[0] == VEH_WHEEL_INVALID && g_v.yawRateCdps == INT16_MIN);
    CHECK(!g_v.turnLeft && g_v.turnSrc == VehSource::NONE);
}

static void test_stream_never_passes_the_decoder()
{
    begin("sniff: the stream may take only what the decoder has read");

    CHECK(setMode(CanMode::SNIFF));
    CanProbeState p = { CanProbeStage::Idle, 0, false };
    CHECK(canProbeArm(p));
    for (unsigned i = 0; i < 100u; ++i) { (void)fakeCanFrame(0x158, 8); hostDrainTick(); }
    CHECK(canRawStreamable() == 0u);               // nothing decoded yet
    CHECK(tickCANSniff(g_v, g_y) == CAN_SNIFF_MAX_PER_TICK);
    CHECK(canRawStreamable() == CAN_SNIFF_MAX_PER_TICK);
    CHECK(tickCANSniff(g_v, g_y) == 100u - CAN_SNIFF_MAX_PER_TICK);
    CHECK(canRawStreamable() == 100u);
    canProbeSkip(p);
}

static void test_sniff_entry_does_not_decode_old_frames()
{
    begin("sniff: frames queued before SNIFF are streamed but not decoded");

    CHECK(setMode(CanMode::DISCOVER));
    const uint8_t d[8] = { 0x10, 0x00, 0, 0, 0, 0, 0, 0 };
    CHECK(fakeCanFrame(0x158, 8, d));
    hostDrainTick();
    CHECK(setMode(CanMode::SNIFF));
    CHECK(canRawStreamable() == 1u);               // still owed to the Orin
    CHECK(tickCANSniff(g_v, g_y) == 0u);
    CHECK(isnan(g_v.speedKmh));
}

// ─── decode-from-ring equals the decoder called directly ─────────────────────

static uint32_t g_rng = 12345u;
static uint32_t rnd() { g_rng = g_rng * 1103515245u + 12345u; return g_rng >> 8; }

struct Gen { uint32_t id; bool ext, rtr; uint8_t dlc; uint8_t d[8]; };

static const uint16_t kMap[] = { 0x158, 0x17C, 0x191, 0x1AB, 0x1D0, 0x294 };

static Gen genFrame(bool mustDecode)
{
    Gen g;
    memset(&g, 0, sizeof(g));
    const uint32_t r = rnd() % 20u;
    g.id  = kMap[rnd() % 6u];
    g.dlc = (g.id == 0x1AB) ? 3u : 8u;
    if (!mustDecode) {
        if (r == 0u)      { g.rtr = true; }
        else if (r == 1u) { g.ext = true; }
        else if (r < 5u)  { g.id = 0x100u + (rnd() % 0x500u); }
        else if (r == 5u) { g.dlc = (uint8_t)(rnd() % 9u); }   // short frames too
    }
    for (uint8_t i = 0; i < 8u; ++i) g.d[i] = (uint8_t)rnd();
    // Indicator bits: blink-like patterns, with direction changes.
    if (g.id == 0x294) g.d[0] = (uint8_t)((rnd() & 1u) ? 0x20u : ((rnd() & 1u) ? 0x40u : 0x00u));
    return g;
}

static bool sameSignals(const VehicleSignals &a, const VehicleSignals &b, uint32_t offset)
{
    #define SAME_F(f) ((isnan(a.f) && isnan(b.f)) || a.f == b.f)
    #define SAME_MS(f) ((a.f == 0u && b.f == 0u) || (b.f == a.f + offset))
    bool ok = SAME_F(speedKmh) && SAME_F(rpm) && a.gear == b.gear &&
              a.steerMotorTorque == b.steerMotorTorque && a.yawRateCdps == b.yawRateCdps &&
              a.pedalGas == b.pedalGas && a.brakePressed == b.brakePressed &&
              a.brakeSwitch == b.brakeSwitch && a.turnLeft == b.turnLeft &&
              a.turnRight == b.turnRight && a.hazard == b.hazard &&
              SAME_MS(speedMs) && SAME_MS(rpmMs) && SAME_MS(gearMs) && SAME_MS(steerMs) &&
              SAME_MS(yawMs) && SAME_MS(pedalMs) && SAME_MS(wheelMs) && SAME_MS(brakeMs) &&
              SAME_MS(turnMs) && SAME_MS(hazardMs) &&
              a.speedSrc == b.speedSrc && a.rpmSrc == b.rpmSrc && a.gearSrc == b.gearSrc &&
              a.steerSrc == b.steerSrc && a.yawSrc == b.yawSrc && a.pedalSrc == b.pedalSrc &&
              a.wheelSrc == b.wheelSrc && a.brakeSrc == b.brakeSrc && a.turnSrc == b.turnSrc &&
              a.hazardSrc == b.hazardSrc;
    for (uint8_t i = 0; i < VEH_WHEEL_COUNT; ++i) ok = ok && a.wheelRaw[i] == b.wheelRaw[i];
    #undef SAME_F
    #undef SAME_MS
    return ok;
}

static void test_ring_decode_equals_direct_decode()
{
    // Contract C: SNIFF's path IS canDecodeFrame(), so the Orin's offline
    // decoder reproduces it. The same 3000 passes are decoded twice — through
    // the controller, the ISR, the ring and tickCANSniff(), then by calling
    // canDecodeFrame() with each frame's arrival time — and compared after
    // every pass. The second run is a long time later, so neither run's
    // indicator holds can reach into the other; stamps are compared shifted.
    begin("sniff: decoding from the ring equals canDecodeFrame() on the same frames");

    static const unsigned PASSES = 3000u;
    static Gen  frames[PASSES][2];
    static VehicleSignals snap[PASSES];

    CHECK(setMode(CanMode::SNIFF));
    CanProbeState p = { CanProbeStage::Idle, 0, false };
    CHECK(canProbeArm(p));                         // accept-all: foreign, ext, rtr all arrive
    g_rng = 2024u;
    const uint32_t t0 = g_t;
    for (unsigned i = 0; i < PASSES; ++i) {
        advance(10u);
        frames[i][0] = genFrame(true);             // each pass decodes at least one frame
        frames[i][1] = genFrame(false);
        for (unsigned k = 0; k < 2u; ++k) {
            const Gen &g = frames[i][k];
            (void)fakeCanFrameEx(g.id, g.ext, g.rtr, g.dlc, g.d);
        }
        pass();
        snap[i] = g_v;
    }

    const uint32_t offset = 10000000u;
    VehicleSignals v;
    YawEstimator   y;
    initVehicleSignals(v);
    initYawEstimator(y, canSniffGetMap());
    unsigned mismatches = 0;
    for (unsigned i = 0; i < PASSES; ++i) {
        const uint32_t now = t0 + offset + 10u * (i + 1u);
        for (unsigned k = 0; k < 2u; ++k) {
            const Gen &g = frames[i][k];
            if (g.rtr || g.ext) continue;          // exactly what SNIFF skips
            (void)canDecodeFrame(v, y, g.id, g.dlc, g.d, now);
        }
        if (!sameSignals(snap[i], v, offset)) ++mismatches;
    }
    CHECK(mismatches == 0u);
    CHECK(snap[PASSES - 1].speedSrc == VehSource::CAN_SNIFF);   // the run really decoded
    canProbeSkip(p);
}

// ─── runner ──────────────────────────────────────────────────────────────────

int main()
{
    test_drain_both_buffers_in_order();
    test_drain_timestamps_follow_micros();
    test_drain_extended_and_remote();
    test_drain_dlc_clamped();
    test_drain_counts_overruns();
    test_drain_refuses_a_floating_miso();
    test_drain_bounded_per_tick();
    test_drain_ignores_pending_txreq();
    test_drain_full_ring_drops_newest();

    test_modes_arm_and_disarm();
    test_obd2_replies_are_not_drained();
    test_main_loop_spi_is_masked();
    test_tick_due_while_masked_runs_at_unmask();

    test_liveness_counter();

    test_sniff_golden_decode();
    test_sniff_stamps_arrival_time();
    test_sniff_skips_remote_frames();
    test_discover_decodes_nothing();
    test_stream_never_passes_the_decoder();
    test_sniff_entry_does_not_decode_old_frames();
    test_ring_decode_equals_direct_decode();

    printf("can_drain_tests: %d checks, %d failed\n", g_checks, g_fails);
    return g_fails;
}
