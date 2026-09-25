/**
 * @file bridge_tests.cpp
 * @brief Deterministic tests for the bridge's session handling: src/main.cpp
 *        driving lib/commLink over a scripted MKR Zero and a scripted Jetson.
 *
 * src/main.cpp is INCLUDED, not linked, so the tests can reach its file-static
 * state and reset it between cases — it still compiles unmodified, from the
 * same file the firmware builds. Its loop() runs exactly as on the chip. The
 * MKR side is real bytes: frames built with the production buildFrame() and fed
 * through the production decoder. The Jetson side is hostLink.h's stand-in.
 *
 * Exit status is the number of failures, so `make check` fails the build.
 */

#include "../../src/main.cpp"

#include <stdio.h>

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

// ─── helpers ─────────────────────────────────────────────────────────────────

static uint32_t g_now = 0;

static void advance(uint32_t ms)
{
    g_now += ms;
    hostSetMillis(g_now);
}

/// A freshly booted bridge: main.cpp's state back to its initialisers, setup() run.
static void begin(const char *name)
{
    g_case = name;
    g_now += 100000u;            // forward only, like a real clock
    hostSetMillis(g_now);
    Serial1.hostReset();

    gLink = CommLink();
    gHost = HostLink();
    gLastStatusMs   = 0;
    gLastMasterMs   = 0;
    gLastRetryMs    = 0;
    gMasterSeen     = false;
    gMasterUp       = false;
    gForwardOnce    = false;
    gForwardOnceMs  = 0;
    gOncePendingTx  = false;
    gDecimCount     = 0;
    gMasterRestarts = 0;

    setup();
}

/// One loop() pass, @p ms after the previous one.
static void pass(uint32_t ms = 1u)
{
    advance(ms);
    loop();
}

/// A snapshot as the MKR would build it, with the fields these tests read.
static TelemetryPayload snapshot(uint32_t masterMs, float accelPeak,
                                 uint16_t flags = 0, uint16_t highGCount = 0)
{
    TelemetryPayload p;
    memset(&p, 0, sizeof(p));
    p.masterMillis    = masterMs;
    p.imuAccelPeak    = accelPeak;
    p.imuGyroPeak     = NAN;
    p.imuLinAccelPeak = NAN;
    p.imuHighGCount   = highGCount;
    p.imuHighGMs      = (highGCount == 0) ? 0xFFFFu : 0u;
    p.flags           = flags;
    return p;
}

/// Puts one telemetry frame on the MKR's UART, byte for byte.
static void mkrSends(const TelemetryPayload &p)
{
    uint8_t buf[COMM_MAX_FRAME];
    const size_t n = buildFrame(MSG_TELEMETRY, reinterpret_cast<const uint8_t *>(&p),
                                static_cast<uint8_t>(sizeof(p)), buf, sizeof(buf));
    Serial1.hostFeed(buf, n);
}

/// A master streaming at 10 Hz: one frame, then 100 ms of loop() on both clocks.
static void mkrFrameThenWait(const TelemetryPayload &p)
{
    mkrSends(p);
    pass(1u);
    advance(99u);
}

static bool logged(const char *needle)
{
    for (size_t i = 0; i < gHost.logs.size(); ++i) {
        if (gHost.logs[i].find(needle) != std::string::npos) return true;
    }
    return false;
}

// ═════════════════════════════════════════════════════════════════════════════
// host sessions — a new session inherits nothing it did not ask for
// ═════════════════════════════════════════════════════════════════════════════

static void test_old_one_shot_not_delivered_to_new_session()
{
    // REGRESSION. A HELLO reset hostLink's own pending requests and the
    // decimation counter, but not the one-shot this sketch had already
    // latched. A host that asked for one frame and reconnected before it
    // arrived had that frame delivered to its SUCCESSOR — which had not asked
    // for anything and had not enabled streaming.
    begin("sessions: a one-shot latched by the old host is not answered to the new one");

    gHost.hostHello();
    pass();
    gHost.hostGetOnce();
    pass();
    CHECK(gForwardOnce);

    gHost.hostHello();           // fast reconnect: new process, no request, no stream
    pass();
    CHECK(!gForwardOnce);
    CHECK(!gOncePendingTx);

    mkrSends(snapshot(50000u, 9.8f));
    pass();
    CHECK(gHost.telemetry.empty());
    CHECK(gHost.hellos == 2u);
}

static void test_new_session_one_shot_still_answered()
{
    // The other half: the NEW host's own request, arriving in the same USB read
    // as its HELLO, must survive the reset and be answered.
    begin("sessions: a GET_ONCE sent with the new HELLO is answered");

    gHost.hostHello();
    gHost.hostGetOnce();
    pass();
    CHECK(gForwardOnce);

    mkrSends(snapshot(50000u, 9.8f));
    pass();
    CHECK(gHost.telemetry.size() == 1u);
    CHECK(!gForwardOnce);
}

// ═════════════════════════════════════════════════════════════════════════════
// master reboots — nothing is merged across one
// ═════════════════════════════════════════════════════════════════════════════

static void test_old_boot_event_not_merged_into_new_boot()
{
    // REGRESSION. With no host streaming, every master frame is accumulated
    // and none is forwarded. A High-G and a 30 m/s2 peak from before the MKR
    // rebooted then rode out on the first frame the new boot produced — beside
    // an imuHighGCount of zero, i.e. an impact claimed by a boot that never saw
    // one. Now the old boot's events arrive as the old boot's own frame, first,
    // and the new boot's frame carries only its own.
    begin("reboot: an old boot's High-G and peak do not reach the new boot's frames");

    gHost.hostHello();
    pass();

    uint32_t m = 600000u;        // the old boot has been up ten minutes
    mkrFrameThenWait(snapshot(m += 100u,  9.8f, 0, 0));
    mkrFrameThenWait(snapshot(m += 100u, 30.0f, COMM_FLAG_IMU_HIGH_G, 1));
    mkrFrameThenWait(snapshot(m += 100u, 12.0f, COMM_FLAG_IMU_HIGH_G, 1));
    CHECK(gHost.telemetry.empty());          // nobody is streaming yet

    advance(3000u);                          // the MKR reboots
    mkrFrameThenWait(snapshot(2500u, 9.8f, 0, 0));
    CHECK(gLink.masterRestarts() == 1u);
    CHECK(logged("master restarted"));
    CHECK(gLink.hasEndedSession());          // held, not dropped, while nobody streams

    gHost.hostStartStream();
    mkrFrameThenWait(snapshot(2600u, 9.8f, 0, 0));

    CHECK(gHost.telemetry.size() == 2u);
    if (gHost.telemetry.size() == 2u) {
        const hostproto::Telemetry &old = gHost.telemetry[0];
        CHECK(old.masterMillis == 600300u);                    // the old boot's own
        CHECK((old.flags & COMM_FLAG_IMU_HIGH_G) != 0u);       // its event survived
        CHECK(old.imuAccelPeak > 29.9f);
        CHECK(old.imuHighGCount == 1u);

        const hostproto::Telemetry &t = gHost.telemetry[1];
        CHECK(t.masterMillis == 2600u);
        CHECK((t.flags & COMM_FLAG_IMU_HIGH_G) == 0u);         // and only there
        CHECK(t.imuAccelPeak < 10.0f);
        CHECK(t.imuHighGCount == 0u);
    }
    CHECK(!gLink.hasEndedSession());
}

static void test_old_boot_event_survives_a_full_ring()
{
    // The owed frame follows the same clear-on-confirmed-send rule as the
    // accumulator it came from: a full host TX ring across the reboot delays
    // the impact, it does not lose it.
    begin("reboot: an old boot's High-G survives a full host TX ring");

    gHost.hostHello();
    gHost.hostStartStream();
    pass();

    gHost.txFull = true;
    mkrFrameThenWait(snapshot(700100u, 30.0f, COMM_FLAG_IMU_HIGH_G, 1));
    mkrFrameThenWait(snapshot(700200u, 11.0f, COMM_FLAG_IMU_HIGH_G, 1));
    advance(3000u);
    mkrFrameThenWait(snapshot(2500u, 9.8f));
    mkrFrameThenWait(snapshot(2600u, 9.8f));
    CHECK(gHost.telemetry.empty());
    CHECK(gLink.hasEndedSession());

    gHost.txFull = false;
    mkrFrameThenWait(snapshot(2700u, 9.8f));
    CHECK(gHost.telemetry.size() == 2u);
    if (gHost.telemetry.size() == 2u) {
        CHECK(gHost.telemetry[0].masterMillis == 700200u);
        CHECK((gHost.telemetry[0].flags & COMM_FLAG_IMU_HIGH_G) != 0u);
        CHECK(gHost.telemetry[0].imuAccelPeak > 29.9f);
        CHECK(gHost.telemetry[1].masterMillis == 2700u);
        CHECK((gHost.telemetry[1].flags & COMM_FLAG_IMU_HIGH_G) == 0u);
    }
}

static void test_old_boot_frame_claims_nothing_live()
{
    // By the time the old boot's frame goes up its readings are a reboot old —
    // possibly much older, if nobody was streaming. A consumer that takes the
    // newest frame as current must not be shown a pre-reboot speed, position
    // or time as live, so only the events and the boot's identity survive.
    begin("reboot: the old boot's frame carries its events and no live readings");

    gHost.hostHello();
    pass();

    TelemetryPayload p = snapshot(600100u, 30.0f,
                                  COMM_FLAG_IMU_HIGH_G | COMM_FLAG_IMU_HIGHG_ARMED |
                                  COMM_FLAG_TIME_VALID | COMM_FLAG_GPS_FIX | COMM_FLAG_OBD2_VALID |
                                  COMM_FLAG_IMU_PRESENT, 2);
    p.speed = 50.0f;  p.rpm = 2000.0f;  p.latitude = 14.5f;  p.longitude = 121.0f;
    p.satellites = 9; p.fixType = 3;    p.fixValid = 1;
    p.year = 2026;    p.month = 9;      p.day = 25;          p.hour = 12;
    p.imuAccelX = 1.0f; p.imuLinAccelX = 20.0f; p.imuTempC = 30.0f;
    p.yawRateCdps = 150; p.steerMotorTorque = 40; p.wheelRaw[0] = 5000;
    p.vehFlags = COMM_VEH_FLAG_BRAKE_VALID | COMM_VEH_FLAG_BRAKE_PRESSED;
    p.imuCalib = 0x30; p.switchState = 0x0005; p.switchChanged = 0x0004;
    mkrFrameThenWait(p);

    advance(3000u);
    mkrFrameThenWait(snapshot(2500u, 9.8f));
    gHost.hostStartStream();
    mkrFrameThenWait(snapshot(2600u, 9.8f));

    CHECK(gHost.telemetry.size() == 2u);
    if (!gHost.telemetry.empty()) {
        const hostproto::Telemetry &o = gHost.telemetry[0];
        // kept: identity, events, the context they were measured in
        CHECK(o.masterMillis == 600100u);
        CHECK(o.imuHighGCount == 2u);
        CHECK(o.imuAccelPeak > 29.9f);
        CHECK((o.flags & COMM_FLAG_IMU_HIGH_G) != 0u);
        CHECK((o.flags & COMM_FLAG_IMU_HIGHG_ARMED) != 0u);
        CHECK((o.flags & COMM_FLAG_IMU_PRESENT) != 0u);
        CHECK(o.switchChanged == 0x0004u);
        CHECK(o.switchState == 0x0005u);
        CHECK(o.imuCalib == 0x30u);
        // blanked: everything that would read as a live measurement
        CHECK((o.flags & (COMM_FLAG_TIME_VALID | COMM_FLAG_GPS_FIX | COMM_FLAG_OBD2_VALID)) == 0u);
        CHECK(isnan(o.speed) && isnan(o.rpm) && isnan(o.latitude) && isnan(o.longitude));
        CHECK(o.satellites == 0u && o.fixType == 0u && o.fixValid == 0u);
        CHECK(o.year == 0u && o.month == 0u && o.day == 0u && o.hour == 0u);
        CHECK(isnan(o.imuAccelX) && isnan(o.imuLinAccelX) && isnan(o.imuTempC));
        CHECK(o.yawRateCdps == INT16_MIN);
        CHECK(o.steerMotorTorque == 0xFFFFu);
        CHECK(o.wheelRaw[0] == 0xFFFFu);
        CHECK(o.vehFlags == 0u);
    }
}

static void test_one_shot_client_gets_owed_frame_first()
{
    // A host that only ever asks with GET_ONCE still receives the old boot's
    // events — ahead of its answer, which is the new boot's next frame.
    begin("reboot: a one-shot client receives the owed frame ahead of its answer");

    gHost.hostHello();
    pass();
    mkrFrameThenWait(snapshot(600100u, 30.0f, COMM_FLAG_IMU_HIGH_G, 1));
    advance(3000u);
    mkrFrameThenWait(snapshot(2500u, 9.8f));
    CHECK(gHost.telemetry.empty());

    gHost.hostGetOnce();
    pass();
    mkrFrameThenWait(snapshot(2600u, 9.8f));
    CHECK(gHost.telemetry.size() == 2u);
    if (gHost.telemetry.size() == 2u) {
        CHECK(gHost.telemetry[0].masterMillis == 600100u);
        CHECK(gHost.telemetry[1].masterMillis == 2600u);
    }
    CHECK(!gForwardOnce);
}

static void test_second_reboot_keeps_the_first_owed_frame()
{
    begin("reboot: a second reboot while a frame is owed keeps the first");

    gHost.hostHello();
    pass();
    mkrFrameThenWait(snapshot(600100u, 30.0f, COMM_FLAG_IMU_HIGH_G, 1));
    advance(3000u);
    mkrFrameThenWait(snapshot(2500u, 20.0f));
    advance(3000u);
    mkrFrameThenWait(snapshot(2400u, 9.8f));        // and again
    CHECK(gLink.masterRestarts() == 2u);
    CHECK(gLink.endedDropped() == 1u);

    gHost.hostStartStream();
    mkrFrameThenWait(snapshot(2500u, 9.8f));
    CHECK(gHost.telemetry.size() == 2u);
    if (gHost.telemetry.size() == 2u) {
        CHECK(gHost.telemetry[0].masterMillis == 600100u);
        CHECK(gHost.telemetry[1].imuAccelPeak < 10.0f);
    }
}

static void test_old_boot_unsent_state_goes_up_as_its_own_frame()
{
    // While a host IS streaming, what the old boot had not yet forwarded (held
    // back by decimation here) is not dropped at the reboot: it goes up as the
    // old boot's own frame — its masterMillis, its peak — ahead of the new boot.
    begin("reboot: a streaming host gets the old boot's unsent window, then the new boot");

    gHost.hostHello();
    gHost.hostStartStream();
    gHost.hostSetDecim(5);
    pass();

    mkrFrameThenWait(snapshot(700100u,  9.8f));
    mkrFrameThenWait(snapshot(700200u, 25.0f));   // a hard bump, not a High-G
    mkrFrameThenWait(snapshot(700300u, 11.0f));
    CHECK(gHost.telemetry.empty());               // decimation held all three

    advance(3000u);
    mkrFrameThenWait(snapshot(2500u, 9.8f));

    CHECK(gHost.telemetry.size() >= 1u);
    if (!gHost.telemetry.empty()) {
        const hostproto::Telemetry &old = gHost.telemetry.front();
        CHECK(old.masterMillis == 700300u);       // the old boot's newest frame
        CHECK(old.imuAccelPeak > 24.9f);          // carrying the old boot's window
    }

    // Everything after it is the new boot's alone.
    for (int i = 1; i <= 10; ++i) mkrFrameThenWait(snapshot(2500u + 100u * i, 9.8f));
    CHECK(gHost.telemetry.size() >= 2u);
    for (size_t i = 1; i < gHost.telemetry.size(); ++i) {
        CHECK(gHost.telemetry[i].masterMillis < 700000u);
        CHECK(gHost.telemetry[i].imuAccelPeak < 10.0f);
    }
}

static void test_fully_forwarded_boot_adds_no_frame()
{
    // Nothing owed, nothing sent: at decimation 1 every frame already went up,
    // so a reboot must not invent an extra one.
    begin("reboot: a boot with nothing unsent adds no frame at the boundary");

    gHost.hostHello();
    gHost.hostStartStream();
    pass();

    mkrFrameThenWait(snapshot(800100u, 9.8f));
    mkrFrameThenWait(snapshot(800200u, 9.8f));
    CHECK(gHost.telemetry.size() == 2u);

    advance(3000u);
    mkrFrameThenWait(snapshot(2500u, 9.8f));
    CHECK(gLink.masterRestarts() == 1u);
    CHECK(gHost.telemetry.size() == 3u);
    if (gHost.telemetry.size() == 3u) CHECK(gHost.telemetry[2].masterMillis == 2500u);
}

// ═════════════════════════════════════════════════════════════════════════════
// the boundary test itself — reboots caught, continuity never mistaken for one
// ═════════════════════════════════════════════════════════════════════════════

static void test_millis_wrap_is_not_a_reboot()
{
    begin("clock: the master's 49.7-day millis() wrap is not a reboot");

    gHost.hostHello();
    gHost.hostStartStream();
    pass();
    uint32_t m = 0xFFFFFC00u;                 // ~1 s before the wrap
    for (int i = 0; i < 30; ++i) mkrFrameThenWait(snapshot(m += 100u, 9.8f));
    CHECK(gLink.masterRestarts() == 0u);
    CHECK(gHost.telemetry.size() == 30u);
}

static void test_long_silence_is_not_a_reboot()
{
    // Cable pulled for an hour, master still running: both clocks moved an
    // hour, so they still agree.
    begin("clock: an hour of silence from a running master is not a reboot");

    gHost.hostHello();
    pass();
    mkrFrameThenWait(snapshot(900000u, 9.8f));
    advance(3600000u);
    mkrFrameThenWait(snapshot(900100u + 3600000u, 9.8f));
    CHECK(gLink.masterRestarts() == 0u);
}

static void test_reboot_after_young_boot_is_caught()
{
    // The old boot was younger than the gap, so the clock did not go back —
    // it just moved far less than the bridge's did.
    begin("clock: a reboot shortly after a previous boot is caught");

    gHost.hostHello();
    pass();
    mkrFrameThenWait(snapshot(700u, 9.8f));
    mkrFrameThenWait(snapshot(800u, 9.8f));
    advance(8000u);
    mkrFrameThenWait(snapshot(3000u, 9.8f));
    CHECK(gLink.masterRestarts() == 1u);
}

static void test_reboot_after_long_uptime_is_caught()
{
    // Past 24.8 days of uptime the modular difference to a fresh boot's clock
    // is positive and huge rather than negative; still a reboot.
    begin("clock: a reboot after more than 24.8 days of uptime is caught");

    gHost.hostHello();
    pass();
    mkrFrameThenWait(snapshot(0xC0000000u, 9.8f));
    advance(3000u);
    mkrFrameThenWait(snapshot(2500u, 9.8f));
    CHECK(gLink.masterRestarts() == 1u);
}

// ─── runner ──────────────────────────────────────────────────────────────────

int main()
{
    test_old_one_shot_not_delivered_to_new_session();
    test_new_session_one_shot_still_answered();

    test_old_boot_event_not_merged_into_new_boot();
    test_old_boot_event_survives_a_full_ring();
    test_old_boot_frame_claims_nothing_live();
    test_one_shot_client_gets_owed_frame_first();
    test_second_reboot_keeps_the_first_owed_frame();
    test_old_boot_unsent_state_goes_up_as_its_own_frame();
    test_fully_forwarded_boot_adds_no_frame();

    test_millis_wrap_is_not_a_reboot();
    test_long_silence_is_not_a_reboot();
    test_reboot_after_young_boot_is_caught();
    test_reboot_after_long_uptime_is_caught();

    printf("bridge_tests: %d checks, %d failed\n", g_checks, g_fails);
    return g_fails;
}
