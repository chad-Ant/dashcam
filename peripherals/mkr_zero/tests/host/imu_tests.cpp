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
 * Exit status is the number of failures, so `make check` fails the build.
 */

#include "IMUFunctions.h"

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
};

static Burst    g_burst;
static bool     g_busFails  = false;   ///< Every read NACKs.
static bool     g_frozen    = false;   ///< Bytes stop changing: a dead data path.
static uint16_t g_noise     = 0;       ///< Real sensors never repeat; this is why.
static unsigned g_reads     = 0;

static Burst restingBurst()
{
    Burst b;
    b.accel[0]   = 0;   b.accel[1]   = 0;   b.accel[2]   = 981;
    b.gyro[0]    = 0;   b.gyro[1]    = 0;   b.gyro[2]    = 0;
    b.gravity[0] = 0;   b.gravity[1] = 0;   b.gravity[2] = 981;
    b.tempC      = 25;
    b.calib      = 0xFFu;                   // sys/gyro/accel/mag all 3
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
    return 0;
}

extern "C" int bno055BusWrite(unsigned char, unsigned char, unsigned char *, unsigned char)
{
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
    g_beginCalls = 0;
    g_reads      = 0;
    g_failNext   = 0;
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

    printf("imu_tests: %d checks, %d failed\n", g_checks, g_fails);
    return g_fails;
}
