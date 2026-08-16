/**
 * BridgeCoalesceVerify - regression tests for the High-G delivery fixes.
 *
 * The bridge can accept COMMLINK_MAX_FRAMES_PER_POLL master frames in one
 * poll() and forwards ONE of them. Everything the other seven carried used to be
 * discarded, including a High-G latch the master had held for 500 ms
 * specifically so it could not fall between two frames. It fell between two
 * frames here instead.
 *
 * These tests push REAL frames through the REAL decoder — buildFrame() on one
 * side, CommLink::poll() on the other — and then ask what survived. Nothing here
 * reimplements the coalescing it is testing.
 *
 * HOW THE FRAMES GET IN
 *
 * Serial1 is put into loopback, so the sketch transmits to itself and no second
 * board or jumper is needed. If the chip refuses internal loopback the sketch
 * says so and asks for a wire between the Serial1 TX and RX pins, which produces
 * exactly the same path.
 *
 * Serial (USB-CDC) carries the report. Nothing else on the board is touched.
 */

#include <Arduino.h>
#include <math.h>
#include "commLink.h"

#include <driver/uart.h>

/// Which UART CommLink drives. Must match the port behind Serial1.
#define TEST_UART_NUM  UART_NUM_1

/// Frames pushed per test. Exactly the drain budget, because the whole defect
/// lives in what poll() does when it services more than one frame.
#define TEST_FRAMES    COMMLINK_MAX_FRAMES_PER_POLL

static CommLink gLink;
static uint16_t gPass = 0, gFail = 0, gSkip = 0;
static bool     gLoopbackOk = false;

// ─── tiny harness ─────────────────────────────────────────────────────────────

static void check(bool ok, const char *name)
{
    Serial.print(ok ? "  PASS  " : "  FAIL  ");
    Serial.println(name);
    if (ok) gPass++; else gFail++;
}

static void skip(const char *name, const char *why)
{
    Serial.printf("  SKIP  %s   (%s)\n", name, why);
    gSkip++;
}

static void group(const char *title)
{
    Serial.println();
    Serial.printf("-- %s\n", title);
}

// ─── frame injection ──────────────────────────────────────────────────────────

/** @brief A payload with every float at NAN, which is the wire's "not supplied". */
static void blankPayload(TelemetryPayload &p)
{
    memset(&p, 0, sizeof(p));
    p.imuAccelPeak    = NAN;
    p.imuGyroPeak     = NAN;
    p.imuLinAccelPeak = NAN;
}

/**
 * @brief Transmits @p n frames, waits for them all, then runs ONE poll().
 *
 * One poll() rather than one per frame, deliberately: servicing several frames
 * in a single call is the exact condition under which the snapshot was being
 * overwritten, so a test that polled between frames would never reach the bug.
 *
 * @return frames the decoder accepted.
 */
static uint32_t feedFrames(const TelemetryPayload *frames, uint8_t n)
{
    const uint32_t before = gLink.framesRx();

    for (uint8_t i = 0; i < n; ++i) {
        uint8_t frame[COMM_FRAME_OVERHEAD + sizeof(TelemetryPayload)];
        const size_t len = buildFrame(MSG_TELEMETRY,
                                      reinterpret_cast<const uint8_t *>(&frames[i]),
                                      (uint8_t)sizeof(TelemetryPayload),
                                      frame, sizeof(frame));
        if (len == 0) return 0;
        Serial1.write(frame, len);
    }
    Serial1.flush();          // wait for the last byte to leave the shift register

    // The bytes are in the RX ring by now; give the driver a moment regardless,
    // then drain in a single call.
    delay(20);
    (void)gLink.poll();

    return gLink.framesRx() - before;
}

/** @brief Snapshot + merge, which is exactly what forwardTelemetry() does. */
static void forwardedSnapshot(TelemetryPayload &out)
{
    out = gLink.latest();
    gLink.mergeCoalesced(out);
}

// ─── the tests ────────────────────────────────────────────────────────────────

// T1 — the defect itself. An event lands in the middle of a burst of frames and
// the newest frame, the only one that used to survive, is quiet.
static void testStickyHighG()
{
    group("T1  a High-G latch survives frame coalescing");

    TelemetryPayload frames[TEST_FRAMES];
    for (uint8_t i = 0; i < TEST_FRAMES; ++i) blankPayload(frames[i]);
    frames[2].flags = COMM_FLAG_IMU_HIGH_G;      // the impact
    // every other frame, INCLUDING THE LAST, reports a quiet window

    gLink.clearCoalesced();
    const uint32_t got = feedFrames(frames, TEST_FRAMES);
    if (got != TEST_FRAMES) {
        skip("sticky High-G", "frames did not arrive - is loopback working?");
        return;
    }

    check((gLink.latest().flags & COMM_FLAG_IMU_HIGH_G) == 0u,
          "the surviving snapshot is the quiet one (the old behaviour)");
    check(gLink.hasUrgent(),
          "hasUrgent() reports the pending event, so decimation is bypassed");

    TelemetryPayload out;
    forwardedSnapshot(out);
    check((out.flags & COMM_FLAG_IMU_HIGH_G) != 0u,
          "and the forwarded frame carries the High-G anyway");
}

// T2 — the peaks describe the interval the forwarded frame claims to cover, so
// coalescing has to take the maximum rather than the last.
static void testPeakMaxima()
{
    group("T2  peaks coalesce as maxima, not as the newest value");

    TelemetryPayload frames[TEST_FRAMES];
    for (uint8_t i = 0; i < TEST_FRAMES; ++i) {
        blankPayload(frames[i]);
        frames[i].imuAccelPeak    = 10.0f + (float)i;
        frames[i].imuGyroPeak     = 20.0f + (float)i;
        frames[i].imuLinAccelPeak = 1.0f  + (float)i;
    }
    frames[3].imuAccelPeak    = 57.0f;   // the impact, mid-burst
    frames[3].imuGyroPeak     = 480.0f;
    frames[3].imuLinAccelPeak = 47.0f;
    // and the newest frame is back to an ordinary level
    frames[TEST_FRAMES - 1].imuAccelPeak    = 9.0f;
    frames[TEST_FRAMES - 1].imuGyroPeak     = 19.0f;
    frames[TEST_FRAMES - 1].imuLinAccelPeak = 0.5f;

    gLink.clearCoalesced();
    if (feedFrames(frames, TEST_FRAMES) != TEST_FRAMES) {
        skip("peak maxima", "frames did not arrive");
        return;
    }

    TelemetryPayload out;
    forwardedSnapshot(out);
    Serial.printf("        accelPeak snapshot=%.1f merged=%.1f\n",
                  (double)gLink.latest().imuAccelPeak, (double)out.imuAccelPeak);

    check(fabsf(out.imuAccelPeak    - 57.0f)  < 0.01f, "accel peak is the interval maximum");
    check(fabsf(out.imuGyroPeak     - 480.0f) < 0.01f, "gyro peak is the interval maximum");
    check(fabsf(out.imuLinAccelPeak - 47.0f)  < 0.01f, "linear accel peak is the interval maximum");
}

// T3 — NAN is the wire's "not supplied", so it must lose to any real value and
// must not survive one. Every comparison against NAN is false, which is how a
// naive max latches NAN forever the first time a channel goes stale.
static void testNanHandling()
{
    group("T3  NAN loses to a measurement and never latches");

    TelemetryPayload frames[TEST_FRAMES];
    for (uint8_t i = 0; i < TEST_FRAMES; ++i) blankPayload(frames[i]);   // all NAN
    frames[4].imuAccelPeak = 33.0f;                                      // one real reading

    gLink.clearCoalesced();
    if (feedFrames(frames, TEST_FRAMES) != TEST_FRAMES) {
        skip("NAN handling", "frames did not arrive");
        return;
    }

    TelemetryPayload out;
    forwardedSnapshot(out);
    check(!isnan(out.imuAccelPeak) && fabsf(out.imuAccelPeak - 33.0f) < 0.01f,
          "a lone measurement survives a burst of NANs");

    // And a genuinely empty interval stays empty rather than becoming zero —
    // zero is a plausible reading, which is why it must not double as the
    // missing-data sentinel.
    for (uint8_t i = 0; i < TEST_FRAMES; ++i) blankPayload(frames[i]);
    gLink.clearCoalesced();
    if (feedFrames(frames, TEST_FRAMES) != TEST_FRAMES) {
        skip("NAN handling (empty interval)", "frames did not arrive");
        return;
    }
    forwardedSnapshot(out);
    check(isnan(out.imuAccelPeak), "an interval with no measurement stays NAN");
}

// T4 — the delivery guarantee. A dropped frame must not take the event with it.
static void testClearOnlyOnSend()
{
    group("T4  the accumulator survives a failed send");

    TelemetryPayload frames[TEST_FRAMES];
    for (uint8_t i = 0; i < TEST_FRAMES; ++i) blankPayload(frames[i]);
    frames[1].flags = COMM_FLAG_IMU_HIGH_G;

    gLink.clearCoalesced();
    if (feedFrames(frames, TEST_FRAMES) != TEST_FRAMES) {
        skip("clear only on send", "frames did not arrive");
        return;
    }

    // A send that failed: the snapshot was built and handed to the host link,
    // which refused it. forwardTelemetry() does NOT clear in that case.
    TelemetryPayload out;
    forwardedSnapshot(out);
    check((out.flags & COMM_FLAG_IMU_HIGH_G) != 0u, "first attempt carries the event");

    forwardedSnapshot(out);
    check((out.flags & COMM_FLAG_IMU_HIGH_G) != 0u,
          "and so does the retry, because merging does not consume");
    check(gLink.hasUrgent(), "the event is still pending after a failed send");

    // Only a confirmed send retires it.
    gLink.clearCoalesced();
    forwardedSnapshot(out);
    check((out.flags & COMM_FLAG_IMU_HIGH_G) == 0u, "a confirmed send retires the event");
    check(!gLink.hasUrgent(), "and nothing is left pending");
}

// T5 — the sticky set has to be a SET. If everything were sticky, a stale OBD2
// or GPS bit would be republished long after it stopped being true.
static void testStickyScope()
{
    group("T5  only event flags are sticky");

    TelemetryPayload frames[TEST_FRAMES];
    for (uint8_t i = 0; i < TEST_FRAMES; ++i) blankPayload(frames[i]);
    // State flags that are true early and false by the end of the interval.
    // COMM_FLAG_IMU_FUSION_MODE is in here deliberately: it says which
    // MEASUREMENT the frame carries, so a sticky one would keep claiming fusion
    // output after a CMD_SET_IMU_MODE had switched the sensor to raw — the
    // newest frame is the only truthful answer.
    frames[0].flags = COMM_FLAG_OBD2_VALID | COMM_FLAG_GPS_FIX |
                      COMM_FLAG_IMU_FUSION_MODE;
    // ...and the event qualifiers, which must behave the opposite way.
    frames[1].flags = COMM_FLAG_IMU_DATA_GAP;
    frames[2].flags = COMM_FLAG_IMU_SATURATED;

    gLink.clearCoalesced();
    if (feedFrames(frames, TEST_FRAMES) != TEST_FRAMES) {
        skip("sticky scope", "frames did not arrive");
        return;
    }

    TelemetryPayload out;
    forwardedSnapshot(out);

    check((out.flags & COMM_FLAG_OBD2_VALID) == 0u,
          "a stale OBD2-valid bit is NOT resurrected");
    check((out.flags & COMM_FLAG_GPS_FIX) == 0u,
          "a stale GPS-fix bit is NOT resurrected");
    check((out.flags & COMM_FLAG_IMU_FUSION_MODE) == 0u,
          "a mode changed mid-interval reports the NEW mode, not the old one");
    check((out.flags & COMM_FLAG_IMU_DATA_GAP) != 0u,
          "a data-gap seen in the interval IS carried");
    check((out.flags & COMM_FLAG_IMU_SATURATED) != 0u,
          "a saturation seen in the interval IS carried");
    check(!gLink.hasUrgent(),
          "but neither qualifier claims urgency - only High-G bypasses decimation");
}

// T6 — the armed flag has to make it across the hop untouched. It is a plain
// state bit, so the test is that coalescing neither invents nor drops it.
static void testArmedPassThrough()
{
    group("T6  the armed flag crosses the bridge intact");

    TelemetryPayload frames[TEST_FRAMES];
    for (uint8_t i = 0; i < TEST_FRAMES; ++i) {
        blankPayload(frames[i]);
        frames[i].flags = COMM_FLAG_IMU_HIGHG_ARMED;
    }

    gLink.clearCoalesced();
    if (feedFrames(frames, TEST_FRAMES) != TEST_FRAMES) {
        skip("armed pass-through", "frames did not arrive");
        return;
    }

    TelemetryPayload out;
    forwardedSnapshot(out);
    check((out.flags & COMM_FLAG_IMU_HIGHG_ARMED) != 0u, "armed is carried when set");

    // The case the flag exists for: the backstop went down mid-interval, so the
    // newest frame is the truth and staleness must not paper over it.
    for (uint8_t i = 0; i < TEST_FRAMES; ++i) {
        blankPayload(frames[i]);
        frames[i].flags = (i < 3u) ? COMM_FLAG_IMU_HIGHG_ARMED : 0u;
    }
    gLink.clearCoalesced();
    if (feedFrames(frames, TEST_FRAMES) != TEST_FRAMES) {
        skip("armed pass-through (disarm)", "frames did not arrive");
        return;
    }
    forwardedSnapshot(out);
    check((out.flags & COMM_FLAG_IMU_HIGHG_ARMED) == 0u,
          "and a backstop that went down is NOT reported as still armed");
}

// ─── loopback bring-up ────────────────────────────────────────────────────────

/**
 * @brief Puts Serial1 into loopback and proves it by round-tripping one frame.
 *
 * Proved rather than assumed: internal loopback is a chip feature that a core
 * update could move or withdraw, and a test suite that silently stopped
 * receiving anything would report SKIP for every case and look tidy doing it.
 */
static bool startLoopback()
{
    Serial1.setRxBufferSize(2048);      // must precede begin(); 8 frames is 1328 bytes
    gLink.begin(COMMLINK_BAUD, COMMLINK_RX_PIN, COMMLINK_TX_PIN, Serial1);

    const esp_err_t err = uart_set_loop_back(TEST_UART_NUM, true);
    if (err != ESP_OK) {
        Serial.printf("  uart_set_loop_back() failed (%d)\n", (int)err);
    }

    TelemetryPayload probe;
    blankPayload(probe);
    probe.flags = COMM_FLAG_IMU_HIGH_G;

    const bool ok = (feedFrames(&probe, 1u) == 1u);
    gLink.clearCoalesced();
    return ok;
}

// ─── entry points ─────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(115200);
    const uint32_t waitUntil = millis() + 4000UL;
    while (!Serial && (int32_t)(millis() - waitUntil) < 0) { }

    Serial.println();
    Serial.println("=== BridgeCoalesceVerify ===================================");
    Serial.println("Regression tests for High-G delivery across the bridge hop.");
    Serial.printf("  frames per poll = %u, payload = %u bytes\n",
                  (unsigned)TEST_FRAMES, (unsigned)sizeof(TelemetryPayload));

    gLoopbackOk = startLoopback();
    if (!gLoopbackOk) {
        Serial.println();
        Serial.println("  *** Serial1 loopback is not working. ***");
        Serial.printf("  Fit a jumper between GPIO%d (TX) and GPIO%d (RX) and reset.\n",
                      COMMLINK_TX_PIN, COMMLINK_RX_PIN);
        Serial.println("  Every test will SKIP until frames can be received.");
    } else {
        Serial.println("  loopback confirmed by a round-tripped frame");
    }

    testStickyHighG();
    testPeakMaxima();
    testNanHandling();
    testClearOnlyOnSend();
    testStickyScope();
    testArmedPassThrough();

    Serial.println();
    Serial.println("=== summary ================================================");
    Serial.printf("  passed  %u\n", gPass);
    Serial.printf("  failed  %u\n", gFail);
    Serial.printf("  skipped %u\n", gSkip);
    Serial.printf("  crc errors on the loopback: %u\n", (unsigned)gLink.crcErrors());
    Serial.println(gFail == 0 ? "  RESULT: OK" : "  RESULT: FAILURES PRESENT");
    Serial.println("============================================================");
}

void loop()
{
    // Everything runs once. Holding here keeps the report at the end of the
    // buffer where it can be read.
    delay(1000);
}
