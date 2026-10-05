/**
 * @file can_selftest_tests.cpp
 * @brief The bench self-test generator (DASHCAM_CAN_STREAM_SELFTEST) in
 *        lib/CANSniffFunctions.cpp: its rate, its sequence and payload, that it
 *        runs only while the drain is armed, and that its frames take the real
 *        route — counted as drained, through the ring, out as F lines.
 *
 * Built twice by the Makefile, at -DDASHCAM_CAN_STREAM_SELFTEST=2400 (the
 * build script's default, a fraction of a frame per drain tick) and =10000 (the
 * cap, exactly one per tick), so the expectations below are written in terms of
 * RATE and never assume one of them. Everything else is the production code the
 * other CAN suites compile; only this define differs.
 *
 * Exit status is the number of failures, so `make check` fails the build.
 */

#include "CANRawStream.h"
#include "CANSniffFunctions.h"
#include "CANStreamHw.h"
#include "DataDictionary.h"
#include "SDFunctions.h"
#include "mcp2515_model.h"
#include "can_stream_hw_stub.h"

#include <stdio.h>
#include <string.h>

#ifndef DASHCAM_CAN_STREAM_SELFTEST
#error "can_selftest_tests is built with -DDASHCAM_CAN_STREAM_SELFTEST=<frames per second>"
#endif

static const uint32_t RATE = DASHCAM_CAN_STREAM_SELFTEST;
static const uint32_t TPS  = 1000000UL / CAN_DRAIN_PERIOD_US;   ///< drain ticks per second

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
static uint32_t       g_t = 0;

static void setClock(uint32_t ms) { g_t = ms; hostSetMillis(g_t); hostSetMicros(g_t * 1000u); }
static void advance(uint32_t ms)  { setClock(g_t + ms); }

static bool setMode(CanMode m)
{
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
    setClock(g_t + 100000u);
    (void)setMode(CanMode::OFF);
    (void)canRawDiscard(canRawStreamable());
    initVehicleSignals(g_v);
    initYawEstimator(g_y, canSniffGetMap());
}

/// The fields a synthetic frame carries, read back out of its payload.
struct Synth { uint32_t seq; uint16_t fill; uint16_t chk; };

static bool isSynthetic(const CanRawFrame &f)
{
    return canRawId(f) == CAN_SELFTEST_ID && !canRawExt(f) && !canRawRtr(f) && canRawDlc(f) == 8u;
}

static Synth decodeSynth(const CanRawFrame &f)
{
    Synth s;
    s.seq  = ((uint32_t)f.data[0] << 24) | ((uint32_t)f.data[1] << 16) | ((uint32_t)f.data[2] << 8) | f.data[3];
    s.fill = (uint16_t)(((uint16_t)f.data[4] << 8) | f.data[5]);
    s.chk  = (uint16_t)(((uint16_t)f.data[6] << 8) | f.data[7]);
    return s;
}

/// Frames waiting for the stream, oldest first, taken out of the ring.
static uint16_t takeAll(CanRawFrame *out, uint16_t cap)
{
    uint16_t n = 0;
    CanRawFrame f;
    while (n < cap && canRawFront(f)) { out[n++] = f; canRawPop(); }
    return n;
}

static void flushStats()
{
    hostUsbSetDtr(true);
    canStreamService();
    hostUsbCollect();
    hostUsbClearOutput();
}

// ═════════════════════════════════════════════════════════════════════════════
// rate
// ═════════════════════════════════════════════════════════════════════════════

static void test_rate_exact_over_a_second()
{
    // The accumulator is exact over any whole second of ticks, and within one
    // frame of the ideal at every tick in between.
    begin("rate: RATE frames in every second of armed ticks, never more than one off between");

    CHECK(setMode(CanMode::DISCOVER));
    for (unsigned sec = 0; sec < 3u; ++sec) {
        const uint32_t n0 = canSelfTestCount();
        const uint32_t d0 = canDrainedCount();
        bool tight = true, single = true;
        for (uint32_t k = 1; k <= TPS; ++k) {
            const uint32_t before = canSelfTestCount();
            hostDrainTick();
            const uint32_t made = canSelfTestCount() - before;
            if (made > 1u) single = false;
            if (RATE == TPS && made != 1u) single = false;
            const uint32_t ideal = (uint32_t)(((uint64_t)k * RATE) / TPS);
            const uint32_t got   = canSelfTestCount() - n0;
            if (got + 1u < ideal || got > ideal + 1u) tight = false;
            if ((k % 256u) == 0u) (void)canRawDiscard(canRawStreamable());   // keep the ring clear
        }
        (void)canRawDiscard(canRawStreamable());
        CHECK(canSelfTestCount() - n0 == RATE);
        CHECK(canDrainedCount() - d0 == RATE);      // and every one counted as drained
        CHECK(tight);
        CHECK(single);                              // one a tick at most; exactly one at the cap
    }
    CHECK(canRingDropCount() == 0u);
}

// ═════════════════════════════════════════════════════════════════════════════
// sequence and payload
// ═════════════════════════════════════════════════════════════════════════════

static void test_sequence_and_payload()
{
    begin("payload: consecutive sequence, ring fill, check word, micros() stamp, id 0x7F0 DLC 8");

    CHECK(setMode(CanMode::DISCOVER));
    const uint32_t seq0 = canSelfTestCount();
    uint32_t stamps[64];
    unsigned made = 0;
    for (uint32_t k = 0; k < 4u * TPS && made < 40u; ++k) {
        hostSetMicros(0x10000000u + k * CAN_DRAIN_PERIOD_US);
        const uint32_t before = canSelfTestCount();
        hostDrainTick();
        if (canSelfTestCount() != before) stamps[made++] = 0x10000000u + k * CAN_DRAIN_PERIOD_US;
    }
    CHECK(made == 40u);

    CanRawFrame f[64];
    const uint16_t n = takeAll(f, 64u);
    CHECK(n == 40u);
    for (uint16_t i = 0; i < n && i < 40u; ++i) {
        CHECK(isSynthetic(f[i]));
        const Synth s = decodeSynth(f[i]);
        CHECK(s.seq == seq0 + i);
        CHECK(s.fill == i);                          // nothing taken out: i frames ahead of it
        CHECK(s.chk == (uint16_t)~(seq0 + i));
        CHECK(f[i].us == stamps[i]);
    }
}

static void test_sequence_runs_on_across_modes()
{
    // Armed ticks only: OFF and OBD2 make nothing and lose nothing — the
    // sequence resumes where it stopped, so a pause is never mistaken for loss.
    begin("armed only: nothing in OFF or OBD2, frames in DISCOVER and SNIFF, sequence unbroken");

    CHECK(setMode(CanMode::DISCOVER));
    for (uint32_t k = 0; k < TPS / 10u; ++k) hostDrainTick();
    (void)canRawDiscard(canRawStreamable());

    CHECK(setMode(CanMode::OFF));
    const uint32_t nOff = canSelfTestCount();
    const uint32_t dOff = canDrainedCount();
    for (uint32_t k = 0; k < TPS; ++k) hostDrainTick();
    CHECK(canSelfTestCount() == nOff);
    CHECK(canDrainedCount() == dOff);
    CHECK(canRawStreamable() == 0u);

    CHECK(setMode(CanMode::OBD2));
    for (uint32_t k = 0; k < TPS; ++k) hostDrainTick();
    CHECK(canSelfTestCount() == nOff);
    CHECK(canRawStreamable() == 0u);

    // SNIFF is armed too. Its decoder takes the frames and decodes nothing:
    // 0x7F0 is in no map.
    CHECK(setMode(CanMode::SNIFF));
    const uint32_t matched0 = canSniffMatchCount();
    for (uint32_t k = 0; k < TPS / 20u; ++k) hostDrainTick();
    const uint32_t inSniff = canSelfTestCount() - nOff;
    CHECK(inSniff == RATE / 20u || inSniff + 1u == RATE / 20u || inSniff == RATE / 20u + 1u);
    while (tickCANSniff(g_v, g_y) != 0u) {}
    CHECK(g_v.speedSrc == VehSource::NONE && g_v.rpmSrc == VehSource::NONE);
    CHECK(canSniffMatchCount() == matched0);

    // Half a second in total between the two armed spells: under the ring's
    // 1024 slots even at the cap, so nothing here is lost to a full ring.
    CHECK(setMode(CanMode::DISCOVER));
    CanRawFrame f[2048];
    for (uint32_t k = 0; k < TPS / 20u; ++k) hostDrainTick();
    const uint16_t n = takeAll(f, 2048u);
    CHECK(n > 0u);
    // Everything queued since OFF, from SNIFF and DISCOVER alike, is one
    // unbroken run starting at the number OFF stopped at.
    bool consecutive = true;
    for (uint16_t i = 0; i < n; ++i) {
        if (!isSynthetic(f[i]) || decodeSynth(f[i]).seq != nOff + i) consecutive = false;
    }
    CHECK(consecutive);
}

// ═════════════════════════════════════════════════════════════════════════════
// the real route: drained, ring, stream, FS
// ═════════════════════════════════════════════════════════════════════════════

static unsigned parseSeqs(const char *out, uint32_t *seqs, unsigned cap)
{
    unsigned n = 0;
    for (const char *p = out; *p && n < cap; ) {
        unsigned ts = 0, d0 = 0, d1 = 0, d2 = 0;
        char idbuf[9] = { 0 };
        if (sscanf(p, "F %8x %8s 8 %8x%4x%4x", &ts, idbuf, &d0, &d1, &d2) == 5 && strcmp(idbuf, "7F0") == 0) {
            seqs[n++] = d0;
        }
        const char *nl = strchr(p, '\n');
        if (!nl) break;
        p = nl + 1;
    }
    return n;
}

static void test_streamed_to_a_reading_host()
{
    begin("route: synthetic frames reach the host as F lines, every sequence number, in order");

    CHECK(setMode(CanMode::DISCOVER));
    flushStats();
    hostUsbSetHostLatency(1u);
    const uint32_t n0 = canSelfTestCount();
    const uint32_t s0 = canStreamStreamedCount();
    for (uint32_t k = 0; k < TPS / 2u; ++k) {
        hostSetMicros(k * CAN_DRAIN_PERIOD_US);
        hostDrainTick();
        if ((k % 10u) == 0u) canStreamService();    // a pass every millisecond
    }
    for (unsigned i = 0; i < 8u; ++i) canStreamService();
    const uint32_t made = canSelfTestCount() - n0;
    CHECK(made >= RATE / 2u - 1u && made <= RATE / 2u + 1u);
    CHECK(canStreamStreamedCount() - s0 == made);
    CHECK(canRawStreamable() == 0u);

    static uint32_t seqs[TPS];
    const unsigned got = parseSeqs(hostUsbOutput(), seqs, TPS);
    CHECK(got == made);
    bool consecutive = true;
    for (unsigned i = 0; i < got; ++i) if (seqs[i] != n0 + i) consecutive = false;
    CHECK(consecutive);
    CHECK(hostUsbBlockingWrites() == 0u);

    // The line itself, byte for byte, for the first frame.
    char golden[64];
    snprintf(golden, sizeof(golden), "F %08X 7F0 8 %08X%04X%04X\n", 0u, (unsigned)n0, 0u,
             (unsigned)(uint16_t)~n0);
    const char *first = strstr(hostUsbOutput(), "F ");
    CHECK(first != nullptr);
    if (first != nullptr) {
        // Its stamp is the tick that made it; rebuild the golden with it.
        unsigned ts = 0;
        CHECK(sscanf(first, "F %8x", &ts) == 1);
        snprintf(golden, sizeof(golden), "F %08X 7F0 8 %08X%04X%04X\n", ts, (unsigned)n0, 0u,
                 (unsigned)(uint16_t)~n0);
        CHECK(strncmp(first, golden, strlen(golden)) == 0);
    }
}

static void test_ring_loss_is_a_sequence_gap()
{
    // What the Orin sees when the ring overflows: a gap in the sequence exactly
    // as long as the FS line's ringdrop grew.
    begin("route: frames the full ring refuses show up as a sequence gap of exactly ringdrop");

    CHECK(setMode(CanMode::DISCOVER));
    flushStats();
    const uint32_t drop0 = canRingDropCount();
    const uint32_t dr0   = canDrainedCount();
    const uint32_t st0   = canStreamStreamedCount();
    const uint32_t nh0   = canStreamNoHostCount();
    const uint32_t n0    = canSelfTestCount();
    // Host stalled (latency 0, never collected): at most one packet armed, and
    // then the ring fills and refuses the newest.
    for (uint32_t k = 0; canSelfTestCount() - n0 < CAN_RING_SLOTS + 50u && k < 20u * TPS; ++k) {
        hostDrainTick();
        if (k == TPS / 100u) canStreamService();
    }
    const uint32_t dropped = canRingDropCount() - drop0;
    CHECK(dropped > 0u);

    hostUsbSetHostLatency(1u);                       // the host reads again
    hostUsbCollect();
    for (unsigned i = 0; i < 400u && canRawStreamable() != 0u; ++i) canStreamService();
    CHECK(canRawStreamable() == 0u);
    // A few more after the loss, so the refused numbers sit BETWEEN two that
    // arrived — which is how the Orin will meet them.
    const uint32_t nAfter = canSelfTestCount();
    for (uint32_t k = 0; canSelfTestCount() < nAfter + 3u && k < TPS; ++k) hostDrainTick();
    for (unsigned i = 0; i < 4u; ++i) canStreamService();
    CHECK(canRawStreamable() == 0u);

    static uint32_t seqs[2 * CAN_RING_SLOTS];
    const unsigned got = parseSeqs(hostUsbOutput(), seqs, 2u * CAN_RING_SLOTS);
    uint32_t gaps = 0;
    for (unsigned i = 1; i < got; ++i) gaps += seqs[i] - seqs[i - 1] - 1u;
    CHECK(got >= 2u);
    CHECK(gaps == dropped);
    if (got >= 1u) {
        CHECK(seqs[0] == n0);
        CHECK(seqs[got - 1u] + 1u == canSelfTestCount());
    }
    // Conservation over the test, synthetic frames included.
    CHECK((canStreamStreamedCount() - st0) + (canStreamNoHostCount() - nh0) + dropped ==
          canDrainedCount() - dr0);
}

static void test_real_frames_alongside()
{
    // The real drain keeps working: a frame on the bus is read in the same tick,
    // queued after the synthetic one the tick made, and counted with it.
    begin("route: a real bus frame in the same tick follows the synthetic one, both drained");

    CHECK(setMode(CanMode::DISCOVER));
    const uint8_t d[2] = { 0xCA, 0xFE };
    bool seen = false;
    for (uint32_t k = 0; k < TPS && !seen; ++k) {
        CHECK(fakeCanFrame(0x123, 2, d));
        const uint32_t n0 = canSelfTestCount();
        const uint32_t d0 = canDrainedCount();
        hostDrainTick();
        CanRawFrame f[4];
        const uint16_t n = takeAll(f, 4u);
        if (canSelfTestCount() == n0) {
            CHECK(n == 1u && canRawId(f[0]) == 0x123u);   // a tick without a synthetic frame
            continue;
        }
        seen = true;
        CHECK(n == 2u);
        CHECK(isSynthetic(f[0]) && decodeSynth(f[0]).seq == n0);
        CHECK(canRawId(f[1]) == 0x123u && canRawDlc(f[1]) == 2u && f[1].data[0] == 0xCA);
        CHECK(canDrainedCount() - d0 == 2u);
    }
    CHECK(seen);
}

// ─── runner ──────────────────────────────────────────────────────────────────

int main()
{
    printf("can_selftest_tests at %lu frames/s\n", (unsigned long)RATE);

    test_rate_exact_over_a_second();
    test_sequence_and_payload();
    test_sequence_runs_on_across_modes();

    test_streamed_to_a_reading_host();
    test_ring_loss_is_a_sequence_gap();
    test_real_frames_alongside();

    printf("can_selftest_tests: %d checks, %d failed\n", g_checks, g_fails);
    return g_fails;
}
