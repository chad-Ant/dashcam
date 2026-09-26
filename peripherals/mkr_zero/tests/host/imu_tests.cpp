/**
 * @file imu_tests.cpp
 * @brief Deterministic tests for the lifecycle and fault accounting in
 *        lib/IMUFunctions.cpp.
 *
 * IMUFunctions.cpp compiles UNMODIFIED. What sits under it is faked here, at
 * the two seams it already has: the staged bring-up (bno055Init*) and the
 * counted transport (bno055BusRead/Write). The fakes are deliberately dumb — a
 * bring-up that reaches Configured a few ticks after it is begun, and a burst
 * the test writes byte for byte — because the logic under test is what
 * IMUFunctions.cpp DOES with a stage and a burst, not how the BNO055 gets there.
 *
 * Every poll below runs the sketch's own pass order: imuInitTick() on every
 * pass, then getIMUData() on the poll timer. The first two REGRESSION cases
 * only fail with that ordering, which is the ordering loop() uses.
 *
 * Exit status is nonzero on failure (not a count that could wrap at 256).
 */

#include "IMUFunctions.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

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

// ─── fakes: the staged bring-up ──────────────────────────────────────────────
//
// Just the contract IMUFunctions.cpp relies on: begin arms the machine (and is
// refused once quarantined), a few ticks later the stage reads Configured, and
// it STAYS Configured afterwards — which is exactly the property the first
// regression below is about.

static const int kBringUpTicks = 3;

static int      g_ticksLeft  = 0;
static unsigned g_beginCalls = 0;
static int      g_failNext   = 0;   ///< Bring-ups that end in Failed before one succeeds.

bool bno055InitBegin(BNO055InitState &st, uint8_t opMode)
{
    if (st.stage == BNO055InitStage::Quarantined) return false;
    if (opMode != OPERATION_MODE_IMUPLUS && opMode != OPERATION_MODE_AMG) return false;
    st.opMode   = opMode;
    st.stage    = BNO055InitStage::WaitBoot;
    g_ticksLeft = kBringUpTicks;
    ++g_beginCalls;
    return true;
}

BNO055InitStage bno055InitTick(BNO055InitState &st)
{
    if (st.stage == BNO055InitStage::Configured ||
        st.stage == BNO055InitStage::Quarantined ||
        st.stage == BNO055InitStage::Idle) {
        return st.stage;
    }
    // Failed is a waiting state the real machine leaves BY ITSELF, re-running
    // bno055InitBegin() after its backoff — no initializeIMU() involved.
    if (st.stage == BNO055InitStage::Failed) {
        (void)bno055InitBegin(st, st.opMode);
        return st.stage;
    }
    if (--g_ticksLeft <= 0) {
        if (g_failNext > 0) {
            --g_failNext;
            st.stage = BNO055InitStage::Failed;
            return st.stage;
        }
        st.stage      = BNO055InitStage::Configured;
        st.highGArmed = true;
    } else {
        st.stage = BNO055InitStage::FindDevice;   // any stage short of Configured
    }
    return st.stage;
}

bool bno055InitReady(const BNO055InitState &st)
{
    return st.stage == BNO055InitStage::Configured;
}

void bno055InitQuarantine(BNO055InitState &st, BNO055InitStatus)
{
    st.stage = BNO055InitStage::Quarantined;
}

I2CBusState i2cBusBegin(void) { return I2CBusState::Ready; }
uint8_t     bno055FindAddress() { return BNO055_I2C_ADDRESS_DEFAULT; }
TwoWire     Wire;

// ─── fakes: the transport and the burst it returns ───────────────────────────

/// What the part "measures". Raw register counts, exactly as they sit on the
/// wire, so a test states the fault it is injecting in the sensor's own terms.
struct Burst {
    int16_t accel[3];      ///< 100 LSB = 1 m/s2.
    int16_t gyro[3];       ///< 16 LSB = 1 deg/s.
    int16_t gravity[3];    ///< 100 LSB = 1 m/s2.
    int8_t  tempC;
    uint8_t calib;
    uint8_t stResult;      ///< ST_RESULT, 0x0F on a healthy running part.
    uint8_t intSta;        ///< INT_STA, clear-on-read; 0x20 = High-G latched.
};

static Burst    g_burst;
static bool     g_busFails  = false;   ///< Every read NACKs.
static bool     g_frozen    = false;   ///< Bytes stop changing: a dead data path.
static bool     g_zeroBurst = false;   ///< Every byte zero: a part that has just reset.
static uint16_t g_noise     = 0;       ///< Real sensors never repeat; this is why.
static unsigned g_reads     = 0;
static unsigned g_clearWrites = 0;
static bool g_clearFails = false;

static Burst restingBurst()
{
    Burst b;
    b.accel[0]   = 0;   b.accel[1]   = 0;   b.accel[2]   = 981;
    b.gyro[0]    = 0;   b.gyro[1]    = 0;   b.gyro[2]    = 0;
    b.gravity[0] = 0;   b.gravity[1] = 0;   b.gravity[2] = 981;
    b.tempC      = 25;
    b.calib      = 0xFFu;                   // sys/gyro/accel/mag all 3
    b.stResult   = 0x0Fu;                   // every power-on self-test passed
    b.intSta     = 0x00u;
    return b;
}

static void put16(unsigned char *p, int16_t v)
{
    p[0] = static_cast<unsigned char>(static_cast<uint16_t>(v) & 0xFFu);
    p[1] = static_cast<unsigned char>(static_cast<uint16_t>(v) >> 8);
}

extern "C" int bno055BusRead(unsigned char, unsigned char reg,
                             unsigned char *buf, unsigned char cnt)
{
    ++g_reads;
    if (g_busFails) return -1;
    if (reg != 0x08u || cnt != 48u) return -1;   // the driver reads one burst only

    memset(buf, 0, cnt);
    if (g_zeroBurst) return 0;
    // One count of noise on the lowest bit of an axis the gates never look at
    // closely: enough to prove the data path is alive, as real noise does.
    const int16_t n = g_frozen ? 0 : static_cast<int16_t>(g_noise++ & 1u);
    for (int i = 0; i < 3; ++i) {
        put16(&buf[0  + 2 * i], static_cast<int16_t>(g_burst.accel[i] + (i == 0 ? n : 0)));
        put16(&buf[12 + 2 * i], static_cast<int16_t>(g_burst.gyro[i]  + (i == 0 ? n : 0)));
        put16(&buf[38 + 2 * i], g_burst.gravity[i]);
    }
    buf[44] = static_cast<unsigned char>(g_burst.tempC);
    buf[45] = g_burst.calib;
    buf[46] = g_burst.stResult;
    buf[47] = g_burst.intSta;
    g_burst.intSta = 0x00u;                 // clear-on-read, like the part
    return 0;
}

extern "C" int bno055BusWrite(unsigned char, unsigned char reg, unsigned char *buf, unsigned char cnt)
{
    if (reg == BNO055_SYS_TRIGGER_ADDR && cnt == 1u && (buf[0] & 0x40u)) {
        ++g_clearWrites;
        if (g_clearFails) return -1;
        digitalWrite(IMU_HIGHG_INT_PIN, LOW);  // RST_INT releases the hardware latch
    }
    return 0;
}

// ─── helpers ─────────────────────────────────────────────────────────────────

static IMUDevice g_dev;
static IMUData   g_data;
static uint32_t  g_t = 0;

static void begin(const char *name)
{
    g_case = name;
    hostReset();
    g_t = 1000u;
    hostSetMillis(g_t);
    // Value-initialised, so every field starts defined — the way the sketch's
    // global starts — rather than as an automatic's indeterminate garbage.
    g_dev = IMUDevice();
    initIMUData(g_data);
    g_burst      = restingBurst();
    g_busFails   = false;
    g_frozen     = false;
    g_zeroBurst  = false;
    g_beginCalls = 0;
    g_reads      = 0;
    g_failNext   = 0;
    g_clearWrites = 0;
    g_clearFails = false;
}

/// One loop() pass on the poll timer, in the sketch's order.
static IMUReturnStatus pass()
{
    g_t += IMU_POLL_MS;
    hostSetMillis(g_t);
    (void)imuInitTick(g_dev);
    return getIMUData(g_dev, g_data);
}

/// Starts a bring-up and runs passes until it completes. True when it did.
static bool bringUp()
{
    if (initializeIMU(g_dev) != IMUReturnStatus::OK) return false;
    for (int i = 0; i < 2 * kBringUpTicks; ++i) {
        (void)pass();
        if (imuIsReady(g_dev)) return true;
    }
    return false;
}

// ═════════════════════════════════════════════════════════════════════════════
// baseline — the fakes and the harness do what the tests below assume
// ═════════════════════════════════════════════════════════════════════════════

static void test_bring_up_then_healthy_polls()
{
    begin("baseline: a bring-up completes and resting data is accepted");

    CHECK(bringUp());
    CHECK(g_beginCalls == 1u);

    for (int i = 0; i < 50; ++i) (void)pass();
    CHECK(imuIsReady(g_dev));
    CHECK(!isIMUDegraded(g_dev));
    CHECK(g_dev.faults == 0u);
    CHECK(g_data.accelValid);
    CHECK(g_data.accelZ > 9.7f && g_data.accelZ < 9.9f);
}

// ═════════════════════════════════════════════════════════════════════════════
// lifecycle — only a genuine bring-up restores readiness
// ═════════════════════════════════════════════════════════════════════════════

static void test_failed_reads_stay_retired_until_recovered()
{
    // REGRESSION. The init machine stays in Configured once a bring-up reached
    // it, and imuInitTick() took "Configured and not ready" as the edge into
    // readiness. So five failed reads retired the part and the very next
    // pass declared it ready again — fault count wiped, nothing reconfigured,
    // and isIMUDegraded() never true long enough for recovery to run.
    begin("lifecycle: a part retired by failed reads stays retired until recovery");

    CHECK(bringUp());

    g_busFails = true;
    for (unsigned i = 0; i < IMU_MAX_CONSECUTIVE_FAULTS; ++i) (void)pass();
    CHECK(!g_dev.ready);
    CHECK(isIMUDegraded(g_dev));

    // The bus answers again — but the part was never reconfigured, so whatever
    // put it in a bad state is still there. Passes alone must not revive it.
    g_busFails = false;
    for (int i = 0; i < 10; ++i) (void)pass();
    CHECK(!g_dev.ready);
    CHECK(!imuIsReady(g_dev));
    CHECK(isIMUDegraded(g_dev));
    CHECK(!g_data.accelValid);
    CHECK(g_beginCalls == 1u);

    // The recovery the sketch schedules off isIMUDegraded(): a full bring-up.
    CHECK(recoverIMU(g_dev) == IMUReturnStatus::OK);
    CHECK(g_beginCalls == 2u);
    for (int i = 0; i < 2 * kBringUpTicks && !imuIsReady(g_dev); ++i) (void)pass();
    CHECK(imuIsReady(g_dev));
    CHECK(!isIMUDegraded(g_dev));

    (void)pass();
    CHECK(g_data.accelValid);
}

static void test_frozen_channel_stays_retired_until_recovered()
{
    // REGRESSION, the same edge reached from the frozen-data check: a data path
    // that stopped converting retires the part so that it gets reconfigured.
    // The very next pass used to un-retire it instead.
    begin("lifecycle: a part retired for frozen data stays retired until recovery");

    CHECK(bringUp());

    g_frozen = true;
    const uint32_t polls = (IMU_MAX_CHANNEL_STALL_MS / IMU_POLL_MS) + 10u;
    for (uint32_t i = 0; i < polls && g_dev.ready; ++i) (void)pass();
    CHECK(!g_dev.ready);

    g_frozen = false;   // bytes move again; still nobody reconfigured the part
    for (int i = 0; i < 10; ++i) (void)pass();
    CHECK(!g_dev.ready);
    CHECK(isIMUDegraded(g_dev));
    CHECK(g_beginCalls == 1u);
}

static void test_bring_up_that_retries_itself_still_arms()
{
    // The init machine retries a failed bring-up on its own, from Failed, with
    // no initializeIMU() in between. The armed edge must survive that, or a
    // part that needed two attempts would never become ready.
    begin("lifecycle: a bring-up the machine retries by itself still ends ready");

    g_failNext = 2;
    CHECK(initializeIMU(g_dev) == IMUReturnStatus::OK);
    for (int i = 0; i < 12 * kBringUpTicks && !imuIsReady(g_dev); ++i) (void)pass();
    CHECK(imuIsReady(g_dev));
    CHECK(g_beginCalls == 3u);          // one armed by us, two by the machine
    (void)pass();
    CHECK(g_data.accelValid);
}

static void test_reset_without_bring_up_is_not_ready()
{
    // imuMarkAbsent() is "nothing fitted", and the machine may still read
    // Configured from an earlier bring-up. That must not count as one.
    begin("lifecycle: imuMarkAbsent() after a bring-up leaves the device absent");

    CHECK(bringUp());
    imuMarkAbsent(g_dev);
    for (int i = 0; i < 10; ++i) (void)pass();
    CHECK(!g_dev.ready);
    CHECK(!g_data.accelValid);
}

static void test_mode_switch_is_a_bring_up()
{
    // setIMUSampleMode() goes through initializeIMU(), so it arms the edge like
    // any bring-up and the part comes back in the new mode.
    begin("lifecycle: a mode switch re-arms the bring-up and completes");

    CHECK(bringUp());
    CHECK(setIMUSampleMode(g_dev, IMUSampleMode::Raw) == IMUReturnStatus::OK);
    CHECK(!g_dev.ready);
    for (int i = 0; i < 2 * kBringUpTicks && !imuIsReady(g_dev); ++i) (void)pass();
    CHECK(imuIsReady(g_dev));
    CHECK(imuSampleMode(g_dev) == IMUSampleMode::Raw);
    CHECK(g_beginCalls == 2u);
}

static void test_quarantine_is_never_ready()
{
    begin("lifecycle: a quarantined device never becomes ready");

    imuQuarantine(g_dev);
    CHECK(initializeIMU(g_dev) == IMUReturnStatus::NOK_INIT_FAILED);
    for (int i = 0; i < 20; ++i) (void)pass();
    CHECK(!g_dev.ready);
    CHECK(!isIMUDegraded(g_dev));   // degraded would schedule a bus retry
    CHECK(g_reads == 0u);           // and it touched no bus at all
}

// ═════════════════════════════════════════════════════════════════════════════
// fault accounting — a run ends only on an ACCEPTED burst
// ═════════════════════════════════════════════════════════════════════════════

static void test_impossible_acceleration_retires_the_part()
{
    // REGRESSION. The consecutive-fault counter was cleared before the
    // accel-limit gate, so a burst failing that gate reset the run to zero and
    // then counted one: a hundred impossible readings and the counter never got
    // past 1. The part was never retired and never reconfigured.
    begin("faults: repeated impossible acceleration retires the part");

    CHECK(bringUp());

    g_burst.accel[0] = 32000;    // 320 m/s2 on X: past any rail, temp and gravity fine
    unsigned retiredAt = 0;
    for (unsigned i = 1; i <= 100; ++i) {
        (void)pass();
        if (!g_dev.ready && retiredAt == 0) retiredAt = i;
    }
    CHECK(retiredAt == IMU_MAX_CONSECUTIVE_FAULTS);
    CHECK(!g_dev.ready);
    CHECK(isIMUDegraded(g_dev));
    CHECK(g_dev.implausible >= IMU_MAX_CONSECUTIVE_FAULTS);
    CHECK(!g_data.accelValid);   // and none of it was ever published
}

static void test_impossible_temperature_retires_the_part()
{
    // The gates before the old reset point already counted correctly; kept so
    // the move of the reset cannot quietly break them.
    begin("faults: repeated impossible temperature retires the part");

    CHECK(bringUp());
    g_burst.tempC = 110;
    for (unsigned i = 0; i < IMU_MAX_CONSECUTIVE_FAULTS; ++i) (void)pass();
    CHECK(!g_dev.ready);
}

static void test_one_good_burst_ends_the_run()
{
    // The other half of the contract, so the fix cannot over-correct into
    // counting faults cumulatively: a lone bad burst is a glitch to ride out,
    // and an accepted burst between two short runs resets the count.
    begin("faults: an accepted burst between two short runs keeps the part");

    CHECK(bringUp());

    for (int round = 0; round < 3; ++round) {
        g_burst.accel[0] = 32000;
        for (unsigned i = 0; i + 1 < IMU_MAX_CONSECUTIVE_FAULTS; ++i) (void)pass();
        CHECK(g_dev.ready);
        g_burst.accel[0] = 0;
        (void)pass();
        CHECK(g_dev.faults == 0u);
    }
    CHECK(g_dev.ready);
    CHECK(g_data.accelValid);
}


// ═════════════════════════════════════════════════════════════════════════════
// burst integrity — a read that is not a sound read of a RUNNING part
// ═════════════════════════════════════════════════════════════════════════════

static bool publishedZeroG()
{
    return g_data.accelValid && g_data.accelX == 0.0f && g_data.accelY == 0.0f && g_data.accelZ == 0.0f;
}

static void test_reset_part_burst_not_published()
{
    // REGRESSION (bench, 2026-09-26). A contact dropout reset the part, whose
    // registers then read all zero. Temperature 0 passed its gate, gravity 0
    // took the fusion-idle exemption, and the burst was published as a genuine
    // 0 g, 0 deg/s sample. It must be charged as implausible instead — and a
    // part that keeps returning it is retired, so it gets reconfigured.
    begin("integrity: an all-zero burst (a part that just reset) is never published");

    CHECK(bringUp());
    g_zeroBurst = true;
    bool leaked = false; unsigned retiredAt = 0;
    for (unsigned i = 1; i <= 20; ++i) {
        (void)pass();
        if (publishedZeroG()) leaked = true;
        if (!g_dev.ready && retiredAt == 0) retiredAt = i;
    }
    CHECK(!leaked);
    CHECK(retiredAt == IMU_MAX_CONSECUTIVE_FAULTS);
    CHECK(g_dev.implausible >= IMU_MAX_CONSECUTIVE_FAULTS);
    CHECK(g_dev.highGCount == 0u);
}

static void test_zero_raw_channels_rejected_after_post()
{
    // The same part a moment later: POST has finished (ST_RESULT 0x0F again)
    // but it sits in CONFIG, not converting, every data register still zero.
    begin("integrity: zero accel AND gyro is rejected even with a passing self-test byte");

    CHECK(bringUp());
    g_frozen = true;                       // no noise: exact zeros
    for (int i = 0; i < 3; ++i) { g_burst.accel[i] = 0; g_burst.gyro[i] = 0; g_burst.gravity[i] = 0; }
    g_burst.tempC = 0;
    bool leaked = false;
    for (unsigned i = 0; i < IMU_MAX_CONSECUTIVE_FAULTS; ++i) { (void)pass(); if (publishedZeroG()) leaked = true; }
    CHECK(!leaked);
    CHECK(!g_dev.ready);
}

static void test_zero_gyro_alone_is_fine()
{
    // The exemption that must survive: a still part can read exactly zero on
    // all three gyro axes. Only accel AND gyro both zero is the reset pattern.
    begin("integrity: an exactly-zero gyroscope alone is a normal still reading");

    CHECK(bringUp());
    g_frozen = false;
    for (int i = 0; i < 20; ++i) (void)pass();
    CHECK(g_dev.ready);
    CHECK(g_dev.implausible == 0u);
    CHECK(g_data.accelValid);
}

static void test_self_test_byte_gate()
{
    begin("integrity: ST_RESULT requires defined ACC, GYR and MCU passes only");

    const struct { uint8_t st; bool accepted; const char *why; } kCases[] = {
        { 0x0F, true,  "all passed" },
        { 0x0D, true,  "MAG failed - unused in fusion" },
        { 0x0E, false, "ACC failed" },
        { 0x0B, false, "GYR failed" },
        { 0x07, false, "MCU failed" },
        { 0x00, false, "just reset" },
        { 0x1F, true,  "reserved bit set, required passes present" },
        { 0xFF, true,  "reserved bits cannot diagnose a corrupt burst on their own" },
    };
    for (unsigned k = 0; k < sizeof(kCases) / sizeof(kCases[0]); ++k) {
        begin("integrity: ST_RESULT requires defined ACC, GYR and MCU passes only");
        CHECK(bringUp());
        g_burst.stResult = kCases[k].st;
        const uint16_t before = g_dev.implausible;
        (void)pass();
        const bool accepted = (g_dev.implausible == before);
        if (accepted != kCases[k].accepted) printf("    ST_RESULT 0x%02X (%s): accepted=%d\n", kCases[k].st, kCases[k].why, (int)accepted);
        CHECK(accepted == kCases[k].accepted);
    }
}

static void test_mag_failure_is_channel_local()
{
    begin("integrity: MAG-only failure in AMG preserves the inertial channels");

    CHECK(bringUp());
    CHECK(setIMUSampleMode(g_dev, IMUSampleMode::Raw) == IMUReturnStatus::OK);
    for (int i = 0; i < 2 * kBringUpTicks && !imuIsReady(g_dev); ++i) (void)pass();
    CHECK(imuIsReady(g_dev));
    g_burst.stResult = 0x0D;
    const uint16_t before = g_dev.implausible;
    for (unsigned i = 0; i < 2u * IMU_MAX_CONSECUTIVE_FAULTS; ++i) {
        CHECK(pass() == IMUReturnStatus::PARTIAL);
    }
    CHECK(g_dev.implausible == before);
    CHECK(g_dev.ready && g_data.accelValid && g_data.gyroValid);
    CHECK(!g_data.magValid && isnan(g_data.magX) && isnan(g_data.magY) && isnan(g_data.magZ));
    g_burst.stResult = 0x0Fu;
    (void)pass();
    CHECK(g_data.magValid);
}

// ═════════════════════════════════════════════════════════════════════════════
// High-G — only from a sound burst, and only as the one bit that is enabled
// ═════════════════════════════════════════════════════════════════════════════

static void test_genuine_high_g_counted()
{
    begin("high-g: a genuine latch in a sound burst is counted");

    CHECK(bringUp());
    g_burst.intSta = 0x20u;
    (void)pass();
    CHECK(g_dev.highGCount == 1u);
    CHECK(g_dev.highGRejected == 0u);
    CHECK(g_data.highGEvent);
}

static void test_corrupt_int_sta_is_not_an_impact()
{
    // REGRESSION (car, 2026-09-26): 21 High-G events in 5 minutes, each with an
    // implausible burst and no motion. Only High-G is enabled, so an INT_STA
    // with any other bit set is a corrupt byte, not a latch.
    begin("high-g: INT_STA with a bit that was never enabled is corruption, not an impact");

    CHECK(bringUp());
    g_burst.intSta = 0xFFu;                // all ones: a read that lost sync
    (void)pass();
    CHECK(g_dev.highGCount == 0u);
    CHECK(g_dev.highGRejected == 1u);
    CHECK(g_dev.implausible == 1u);
    CHECK(!g_data.highGEvent);

    g_burst.intSta = 0x24u;                // High-G plus a stray bit
    (void)pass();
    CHECK(g_dev.highGCount == 0u);
    CHECK(g_dev.highGRejected == 2u);
}

static void test_high_g_in_reset_burst_is_rejected()
{
    begin("high-g: a latch riding in a burst that failed the self-test byte is thrown away");

    CHECK(bringUp());
    g_burst.intSta   = 0x20u;
    g_burst.stResult = 0x00u;
    (void)pass();
    CHECK(g_dev.highGCount == 0u);
    CHECK(g_dev.highGRejected == 1u);
}

static void test_high_g_survives_a_bad_temperature_byte()
{
    // The original ordering, preserved: a sound burst whose DATA fails a later
    // gate still delivers its latch — the comparator fired in hardware whatever
    // the byte next to it says.
    begin("high-g: a sound latch still counts when the temperature gate rejects the data");

    CHECK(bringUp());
    g_burst.intSta = 0x20u;
    g_burst.tempC  = 110;
    (void)pass();
    CHECK(g_dev.highGCount == 1u);
    CHECK(g_dev.highGRejected == 0u);
}

static void test_pin_edge_with_dead_burst_is_rejected()
{
    // The contact glitch that corrupts the reads also glitches the INT line.
    // That edge must go with the burst, not wait for the next good poll.
    begin("high-g: an INT-pin edge that coincides with a failed burst is thrown away");

    CHECK(bringUp());
    imuNoteHighGPin(g_t);
    g_zeroBurst = true;
    (void)pass();
    g_zeroBurst = false;
    (void)pass(); (void)pass();
    CHECK(g_dev.highGCount == 0u);
    CHECK(g_dev.highGRejected == 1u);
}

static void test_confirmed_pin_edge_with_sound_burst_counts()
{
    begin("high-g: a confirmed INT-pin edge is timed by the pin");

    CHECK(bringUp());
    const uint32_t edge = g_t + 3u;
    imuNoteHighGPin(edge);
    g_burst.intSta = 0x20u;
    (void)pass();
    CHECK(g_dev.highGCount == 1u);
    CHECK(g_dev.highGAtMs == edge);
}

static void test_pin_edge_during_bring_up_is_cleared()
{
    // An edge from the reset / INT_EN writes / the glitch that got the part
    // retired, arriving while bring-up runs, is not an impact on the part that
    // bring-up then configures.
    begin("high-g: an INT-pin edge from during bring-up is not reported after it");

    CHECK(initializeIMU(g_dev) == IMUReturnStatus::OK);
    (void)pass();
    imuNoteHighGPin(g_t);
    for (int i = 0; i < 2 * kBringUpTicks && !imuIsReady(g_dev); ++i) (void)pass();
    CHECK(imuIsReady(g_dev));
    for (int i = 0; i < 5; ++i) (void)pass();
    CHECK(g_dev.highGCount == 0u);
    CHECK(g_dev.highGRejected == 0u);
}

static void test_reserved_bits_are_ignored()
{
    // Every combination of ST_RESULT's upper bits and the legacy reserved
    // INT_STA bits, not just one observed module's power-on values.
    for (unsigned st = 0; st < 16; ++st) {
        begin("reserved status bits do not invalidate a healthy burst");
        CHECK(bringUp());
        for (unsigned bits = 0; bits < 8; ++bits) {
            g_burst.stResult = static_cast<uint8_t>((st << 4) | 0x0Du);
            g_burst.intSta = static_cast<uint8_t>((bits & 3u) | ((bits & 4u) << 2));
            (void)pass();
            CHECK(g_dev.ready && g_dev.implausible == 0u && g_data.accelValid);
            CHECK(g_dev.highGCount == 0u);
        }
    }
}

static void test_unconfirmed_pin_edges_are_rejected()
{
    begin("pin glitch after a rejected burst is not an impact");
    CHECK(bringUp());
    g_zeroBurst = true;
    (void)pass();
    g_zeroBurst = false;
    imuNoteHighGPin(g_t + 1u);
    (void)pass();
    CHECK(g_dev.highGCount == 0u && g_dev.highGRejected == 1u);
    CHECK(g_clearWrites == 0u);

    begin("pin glitch across a NACK is not an impact");
    CHECK(bringUp());
    imuNoteHighGPin(g_t);
    g_busFails = true;
    (void)pass();
    CHECK(g_dev.highGRejected == 1u);  // recorded even if no good read follows
    g_busFails = false;
    (void)pass();
    CHECK(g_dev.highGCount == 0u && g_dev.highGRejected == 1u);
    CHECK(g_clearWrites == 0u);
}

static void test_held_pin_survives_bad_reads()
{
    // A real latch whose INT_STA was eaten by a corrupt read: the line is the
    // only evidence left. It must still be high a poll later (a latch holds),
    // is then probed (RST_INT), and is confirmed when it lets go — recorded
    // then, with the edge's own time.
    begin("held INT confirms a real latch even when the burst is corrupt");
    CHECK(bringUp());
    digitalWrite(IMU_HIGHG_INT_PIN, HIGH);
    const uint32_t edge = g_t;
    imuNoteHighGPin(edge);
    g_burst.intSta = 0x20u;
    g_burst.stResult = 0x00u;
    (void)pass();
    CHECK(g_dev.highGCount == 0u && g_dev.lineCandidate);           // seen, not yet probed
    CHECK(g_clearWrites == 0u && g_dev.highGRejected == 0u);
    g_burst.stResult = 0x0Fu;
    (void)pass();
    CHECK(g_dev.highGCount == 0u && g_dev.lineProbe);               // held: probing, not yet believed
    CHECK(g_clearWrites == 1u && digitalRead(IMU_HIGHG_INT_PIN) == LOW);
    (void)pass();
    CHECK(g_dev.highGCount == 1u && g_dev.highGRejected == 0u);     // it let go: real
    CHECK(g_dev.highGAtMs == edge);
    CHECK(g_clearWrites == 1u);                                     // no second clear

    // REGRESSION (review 2026-09-26): a genuine held latch across a NACK was
    // counted as rejected at the NACK AND as an event later, losing its time.
    begin("held INT survives a NACK: one event, not also a rejection, and its edge time kept");
    CHECK(bringUp());
    digitalWrite(IMU_HIGHG_INT_PIN, HIGH);
    const uint32_t edge2 = g_t;
    imuNoteHighGPin(edge2);
    g_busFails = true;
    (void)pass();
    CHECK(g_dev.highGRejected == 0u && g_dev.lineCandidate);        // pending: the line is high
    CHECK(g_clearWrites == 0u);                                     // no probe without a bus
    g_busFails = false;
    (void)pass();                                                   // probe
    (void)pass();                                                   // released -> confirmed
    CHECK(g_dev.highGCount == 1u && g_dev.highGRejected == 0u);
    CHECK(g_dev.highGAtMs == edge2);
    CHECK(g_clearWrites == 1u && digitalRead(IMU_HIGHG_INT_PIN) == LOW);
}

static void test_stuck_high_int_line_is_not_an_impact()
{
    // REGRESSION (review 2026-09-26). A line stuck high by a wiring fault is
    // not driven by the part. Believing a held line outright produced one fake
    // High-G, then a flag held forever and an RST_INT write on every poll.
    begin("int: a line stuck high is detected by the probe and never becomes an impact");
    CHECK(bringUp());
    for (int i = 0; i < 500; ++i) {                  // 5 s, the line never releases
        digitalWrite(IMU_HIGHG_INT_PIN, HIGH);
        (void)pass();
    }
    CHECK(g_dev.highGCount == 0u);
    CHECK(!g_data.highGEvent);
    CHECK(g_dev.intLineDistrusted);
    CHECK(g_dev.highGRejected == 1u);
    CHECK(g_clearWrites == 1u);                       // one probe, not one per poll
    CHECK(g_dev.init.highGArmed);                     // the clear itself worked

    // The fault clears: seen low, trusted again, and a real held latch after
    // that is probed and counted normally.
    digitalWrite(IMU_HIGHG_INT_PIN, LOW);
    (void)pass();
    CHECK(!g_dev.intLineDistrusted);
    digitalWrite(IMU_HIGHG_INT_PIN, HIGH);
    (void)pass(); (void)pass(); (void)pass();        // candidate, probe, released
    CHECK(g_dev.highGCount == 1u && g_clearWrites == 2u);
}

static void test_stuck_probe_waits_for_a_sound_read()
{
    // "Stuck" is only concluded from a SOUND read with no latch: a corrupt
    // read cannot say whether INT_STA was set.
    begin("int: the stuck verdict waits out a corrupt read and a NACK");
    CHECK(bringUp());
    digitalWrite(IMU_HIGHG_INT_PIN, HIGH);
    (void)pass();                                     // candidate
    (void)pass();                                     // held: probe issued
    CHECK(g_dev.lineProbe && g_clearWrites == 1u);
    digitalWrite(IMU_HIGHG_INT_PIN, HIGH);
    g_burst.stResult = 0x00u;                         // corrupt read, line still high
    (void)pass();
    CHECK(g_dev.lineProbe && !g_dev.intLineDistrusted && g_clearWrites == 1u);
    g_busFails = true;                                // nor can a read that never arrived
    (void)pass();
    CHECK(g_dev.lineProbe && !g_dev.intLineDistrusted && g_clearWrites == 1u);
    g_busFails = false;
    g_burst.stResult = 0x0Fu;
    (void)pass();
    CHECK(g_dev.intLineDistrusted && g_dev.highGCount == 0u && g_dev.highGRejected == 1u);
}

static void test_sustained_impact_is_one_event_not_stuck()
{
    // A long impact re-latches on every poll: the part re-raises the line AND
    // INT_STA each time. The register confirms it, so it is never "stuck".
    begin("int: a sustained impact re-latching every poll is one event, never INTSTUCK");
    CHECK(bringUp());
    for (int i = 0; i < 30; ++i) {
        digitalWrite(IMU_HIGHG_INT_PIN, HIGH);
        g_burst.intSta = 0x20u;
        (void)pass();
    }
    CHECK(g_dev.highGCount == 1u);
    CHECK(!g_dev.intLineDistrusted && g_dev.highGRejected == 0u);
}

static void test_failed_clear_does_not_repeat_held_pin_forever()
{
    // A held line that cannot even be probed (RST_INT fails) cannot be
    // confirmed: counted as rejected, the pin backstop disarmed — and good
    // register events still arrive.
    begin("failed RST_INT disables the pin backstop but not good register events");
    CHECK(bringUp());
    digitalWrite(IMU_HIGHG_INT_PIN, HIGH);
    g_clearFails = true;
    (void)pass();                                     // candidate
    CHECK(g_clearWrites == 0u);
    (void)pass();                                     // the probe cannot be written
    CHECK(g_clearWrites == 3u && !g_dev.init.highGArmed);
    CHECK(g_dev.highGCount == 0u && g_dev.highGRejected == 1u);
    const uint32_t until = g_t + IMU_HIGHG_HOLD_MS + 2u * IMU_POLL_MS;
    while (g_t < until) (void)pass();
    CHECK(!g_data.highGEvent && g_dev.highGCount == 0u && g_clearWrites == 3u);
    g_clearFails = false;
    g_burst.intSta = 0x20u;
    (void)pass();
    CHECK(g_dev.highGCount == 1u);
}

static void test_line_that_drops_by_itself_is_not_an_impact()
{
    // REVIEW (2026-09-26): probing on first sight could not tell a latch let go
    // by RST_INT from a flickering contact that was high when sampled and low
    // 10 ms later — every flicker became a "release", an event, and a HIGH-G
    // flag held for as long as the flicker lasted. A latch holds until RST_INT;
    // a line that lets go of its own accord was never one.
    begin("int: a line high for one poll and low the next is rejected, not probed");
    CHECK(bringUp());
    digitalWrite(IMU_HIGHG_INT_PIN, HIGH);
    (void)pass();
    digitalWrite(IMU_HIGHG_INT_PIN, LOW);
    (void)pass();
    CHECK(g_dev.highGCount == 0u && g_dev.highGRejected == 1u);
    CHECK(g_clearWrites == 0u && !g_dev.lineCandidate);

    begin("int: a line flickering every poll never raises High-G or writes RST_INT");
    CHECK(bringUp());
    bool everFlagged = false;
    for (int i = 0; i < 1000; ++i) {                  // 10 s of high / low
        digitalWrite(IMU_HIGHG_INT_PIN, (i & 1) ? LOW : HIGH);
        (void)pass();
        everFlagged = everFlagged || g_data.highGEvent;
    }
    CHECK(!everFlagged && g_dev.highGCount == 0u);
    CHECK(g_clearWrites == 0u);
    CHECK(g_dev.highGRejected == 500u);

    begin("int: a line that bounces between polls (fresh edge, still high) is rejected, not probed");
    CHECK(bringUp());
    digitalWrite(IMU_HIGHG_INT_PIN, HIGH);
    (void)pass();                                     // candidate
    const uint32_t bounce = g_t + 4u;
    imuNoteHighGPin(bounce);                          // it fell and rose: a latch cannot
    (void)pass();
    CHECK(g_dev.highGRejected == 1u && g_clearWrites == 0u);
    CHECK(g_dev.lineCandidate && g_dev.lineSinceMs == bounce);  // the bounce starts afresh
}

static void test_relatch_during_probe_is_confirmed()
{
    // REVIEW (2026-09-26): an impact still running when the probe's RST_INT
    // lands latches again. Its edge proves the line let go and was driven
    // again; with the next read corrupt, that evidence was thrown away, the
    // sound read after it concluded "stuck", and the impact was lost.
    begin("int: a re-latch after the probe, seen through a corrupt read, is the impact - not INTSTUCK");
    CHECK(bringUp());
    digitalWrite(IMU_HIGHG_INT_PIN, HIGH);
    const uint32_t edge = g_t;
    imuNoteHighGPin(edge);
    g_burst.stResult = 0x00u;                         // corrupt reads throughout
    (void)pass();                                     // candidate
    (void)pass();                                     // probe: the line lets go...
    CHECK(g_dev.lineProbe && g_clearWrites == 1u);
    digitalWrite(IMU_HIGHG_INT_PIN, HIGH);            // ...and the impact latches again
    imuNoteHighGPin(g_t + 3u);
    g_burst.intSta = 0x20u;                           // eaten by the corrupt read
    (void)pass();
    CHECK(g_dev.highGCount == 1u && g_dev.highGAtMs == edge);
    CHECK(g_clearWrites == 2u && digitalRead(IMU_HIGHG_INT_PIN) == LOW);  // the new latch cleared
    g_burst.stResult = 0x0Fu;
    (void)pass();
    CHECK(!g_dev.intLineDistrusted && g_dev.highGRejected == 0u);
    CHECK(g_dev.highGCount == 1u);

    begin("int: a re-latch seen on a NACK is recorded, and its latch dealt with once the bus is back");
    CHECK(bringUp());
    digitalWrite(IMU_HIGHG_INT_PIN, HIGH);
    const uint32_t edge2 = g_t;
    imuNoteHighGPin(edge2);
    (void)pass(); (void)pass();                       // candidate, probe
    CHECK(g_dev.lineProbe && g_clearWrites == 1u);
    digitalWrite(IMU_HIGHG_INT_PIN, HIGH);
    imuNoteHighGPin(g_t + 3u);
    g_busFails = true;
    (void)pass();
    CHECK(g_dev.highGCount == 1u && g_dev.highGAtMs == edge2 && g_data.highGEvent);
    CHECK(g_clearWrites == 1u);                       // nothing written to a bus that NACKs
    g_busFails = false;
    for (int i = 0; i < 5; ++i) (void)pass();         // held again: candidate, probe, release
    CHECK(g_dev.highGCount == 1u && g_clearWrites == 2u);
    CHECK(!g_dev.intLineDistrusted && g_dev.highGRejected == 0u);
    CHECK(digitalRead(IMU_HIGHG_INT_PIN) == LOW);
}

static void test_nack_decides_a_released_probe()
{
    // REVIEW (2026-09-26): the line is readable without the bus. A probe that
    // let go is confirmed even when every read after it NACKs — otherwise the
    // impact that knocked the connector loose vanished when the part retired.
    begin("int: a probe released during NACKs is confirmed before the part retires");
    CHECK(bringUp());
    digitalWrite(IMU_HIGHG_INT_PIN, HIGH);
    const uint32_t edge = g_t;
    imuNoteHighGPin(edge);
    g_burst.stResult = 0x00u;
    (void)pass(); (void)pass();                       // corrupt reads: candidate, probe
    CHECK(g_dev.lineProbe && g_dev.highGCount == 0u);
    g_busFails = true;
    (void)pass();
    CHECK(g_dev.highGCount == 1u && g_dev.highGAtMs == edge && g_data.highGEvent);
    for (int i = 0; i < 5; ++i) (void)pass();
    CHECK(!g_dev.ready && g_dev.highGCount == 1u && g_dev.highGRejected == 0u);
}

static void test_retirement_settles_an_open_episode()
{
    // REVIEW (2026-09-26): recovery wipes an open episode and can run in the
    // same loop pass as the retirement. Whatever is open is settled first: a
    // released probe is an impact, anything undecided is rejected evidence.
    begin("int: a candidate still open when the part retires is counted as rejected");
    CHECK(bringUp());
    digitalWrite(IMU_HIGHG_INT_PIN, HIGH);
    imuNoteHighGPin(g_t);
    g_busFails = true;
    for (unsigned i = 0; i < IMU_MAX_CONSECUTIVE_FAULTS; ++i) (void)pass();
    CHECK(!g_dev.ready);
    CHECK(g_dev.highGCount == 0u && g_dev.highGRejected == 1u);
    CHECK(!g_dev.lineCandidate && !g_dev.lineProbe);

    begin("int: a probe written on the retiring poll that lets go is an impact");
    CHECK(bringUp());
    digitalWrite(IMU_HIGHG_INT_PIN, HIGH);
    const uint32_t edge = g_t;
    imuNoteHighGPin(edge);
    g_busFails = true;
    for (unsigned i = 0; i + 1u < IMU_MAX_CONSECUTIVE_FAULTS; ++i) (void)pass();
    CHECK(g_dev.ready && g_dev.lineCandidate && g_clearWrites == 0u);  // held, no bus to probe
    g_busFails = false;
    g_burst.stResult = 0x00u;                         // the last fault: a corrupt read
    (void)pass();
    CHECK(!g_dev.ready && g_clearWrites == 1u);
    CHECK(g_dev.highGCount == 1u && g_dev.highGAtMs == edge && g_data.highGEvent);
    CHECK(g_dev.highGRejected == 0u);

    begin("int: a probe still undecided when the part retires is counted as rejected");
    CHECK(bringUp());
    digitalWrite(IMU_HIGHG_INT_PIN, HIGH);
    (void)pass(); (void)pass();                       // candidate, probe
    CHECK(g_dev.lineProbe);
    digitalWrite(IMU_HIGHG_INT_PIN, HIGH);            // still high: undecided
    g_busFails = true;
    for (unsigned i = 0; i < IMU_MAX_CONSECUTIVE_FAULTS; ++i) (void)pass();
    CHECK(!g_dev.ready && g_dev.highGCount == 0u && g_dev.highGRejected == 1u);
}

static uint32_t g_hookEdgeMs = 0;

/// The part latching: its line rises and the RISING-edge ISR runs.
static void latchNow()
{
    digitalWrite(IMU_HIGHG_INT_PIN, HIGH);
    imuNoteHighGPin(g_hookEdgeMs);
}

static void test_edge_landing_mid_sample_is_counted_once()
{
    // REVIEW (2026-09-26): the line and the ISR flag are two views of one
    // event, read at two instants. A latch landing between them must still be
    // one impact at its edge's time — not a rejected naked edge now and an
    // event, timed a poll late, on the next.
    begin("int: a latch landing just after the line is sampled is one event, timed by its edge");
    CHECK(bringUp());
    g_hookEdgeMs = g_t + IMU_POLL_MS;
    hostOnNextRead(IMU_HIGHG_INT_PIN, false, latchNow);
    (void)pass();
    CHECK(g_dev.highGRejected == 0u && g_dev.highGCount == 0u);  // left pending, not thrown away
    g_burst.intSta = 0x20u;                           // latched after this poll's burst was read
    (void)pass();
    CHECK(g_dev.highGCount == 1u && g_dev.highGRejected == 0u);
    CHECK(g_dev.highGAtMs == g_hookEdgeMs);

    begin("int: a latch landing just before the line is sampled belongs to that sighting");
    CHECK(bringUp());
    g_hookEdgeMs = g_t + IMU_POLL_MS;
    hostOnNextRead(IMU_HIGHG_INT_PIN, true, latchNow);
    (void)pass();
    CHECK(g_dev.lineCandidate && g_dev.lineSinceMs == g_hookEdgeMs);
    (void)pass();                                     // held, no fresh edge: probed, not a bounce
    CHECK(g_dev.lineProbe && g_dev.highGRejected == 0u);
    (void)pass();
    CHECK(g_dev.highGCount == 1u && g_dev.highGAtMs == g_hookEdgeMs);
    CHECK(g_dev.highGRejected == 0u);
}

static void test_rejection_count_is_boot_cumulative()
{
    begin("recovery and mode changes preserve rejected-evidence diagnostics");
    CHECK(bringUp());
    g_burst.intSta = 0xFFu;
    (void)pass();
    CHECK(g_dev.highGRejected == 1u);
    CHECK(bringUp());
    CHECK(g_dev.highGRejected == 1u);
    CHECK(setIMUSampleMode(g_dev, IMUSampleMode::Raw) == IMUReturnStatus::OK);
    CHECK(g_dev.highGRejected == 1u);
    for (int i = 0; i < 2 * kBringUpTicks; ++i) (void)pass();
    g_dev.highGRejected = 0xFFFFu;
    imuNoteHighGPin(g_t);
    (void)pass();
    CHECK(g_dev.highGRejected == 0xFFFFu);
}

// ─── runner ──────────────────────────────────────────────────────────────────

int main()
{
    test_bring_up_then_healthy_polls();

    test_failed_reads_stay_retired_until_recovered();
    test_frozen_channel_stays_retired_until_recovered();
    test_bring_up_that_retries_itself_still_arms();
    test_reset_without_bring_up_is_not_ready();
    test_mode_switch_is_a_bring_up();
    test_quarantine_is_never_ready();

    test_impossible_acceleration_retires_the_part();
    test_impossible_temperature_retires_the_part();
    test_one_good_burst_ends_the_run();

    test_reset_part_burst_not_published();
    test_zero_raw_channels_rejected_after_post();
    test_zero_gyro_alone_is_fine();
    test_self_test_byte_gate();
    test_mag_failure_is_channel_local();

    test_genuine_high_g_counted();
    test_corrupt_int_sta_is_not_an_impact();
    test_high_g_in_reset_burst_is_rejected();
    test_high_g_survives_a_bad_temperature_byte();
    test_pin_edge_with_dead_burst_is_rejected();
    test_confirmed_pin_edge_with_sound_burst_counts();
    test_pin_edge_during_bring_up_is_cleared();
    test_reserved_bits_are_ignored();
    test_unconfirmed_pin_edges_are_rejected();
    test_held_pin_survives_bad_reads();
    test_stuck_high_int_line_is_not_an_impact();
    test_stuck_probe_waits_for_a_sound_read();
    test_sustained_impact_is_one_event_not_stuck();
    test_failed_clear_does_not_repeat_held_pin_forever();
    test_line_that_drops_by_itself_is_not_an_impact();
    test_relatch_during_probe_is_confirmed();
    test_nack_decides_a_released_probe();
    test_retirement_settles_an_open_episode();
    test_edge_landing_mid_sample_is_counted_once();
    test_rejection_count_is_boot_cumulative();

    printf("imu_tests: %d checks, %d failed\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
