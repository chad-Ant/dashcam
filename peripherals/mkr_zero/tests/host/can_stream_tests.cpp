/**
 * @file can_stream_tests.cpp
 * @brief The raw CAN stream to the Orin, lib/CANRawStream.cpp: the F and FS
 *        lines byte for byte, and the USB gating that keeps loop() from ever
 *        waiting on the host.
 *
 * Frames take the production route — bus side of the MCP2515 model, drain ISR
 * (can_stream_hw_stub.cpp), ring, canStreamService() — and leave through the
 * stub Serial in arduino_stub.cpp, which captures every write and models the
 * one thing in arduino:samd's USB stack that decides blocking: the bulk IN bank
 * stays armed after a write until the host collects it, and a write issued
 * while it is armed is where the core's send() would spin for up to 70 ms.
 *
 * Golden strings are the contract with the Orin tools; a change here is a
 * change to that contract.
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

/// Counters as the FS line reports them, for conservation checks.
struct Totals { uint32_t drained, streamed, ovf, drop, nohost; };
static Totals totals()
{
    Totals t = { canDrainedCount(), canStreamStreamedCount(), canSniffOverrunCount(),
                 canRingDropCount(), canStreamNoHostCount() };
    return t;
}

static void begin(const char *name)
{
    g_case = name;
    hostReset();               // USB: no host, enumerated, bank free, room 63
    fakeCanReset();
    hostDrainReset();
    canDrainTimerBegin();
    canSniffSetMap(nullptr);
    setClock(g_t + 100000u);
    (void)setMode(CanMode::OFF);
    (void)canRawDiscard(canRawStreamable());
    initVehicleSignals(g_v);
    initYawEstimator(g_y, canSniffGetMap());
    // An FS line is due at the first service of every test (a 100 s gap);
    // tests that are not about FS take it out of the way first.
}

/// Clears the FS line that falls due at the start of a test.
static void flushStats()
{
    hostUsbSetDtr(true);
    canStreamService();
    hostUsbCollect();
    hostUsbClearOutput();
}

static void frameOnBus(uint16_t id, uint8_t dlc, const uint8_t *d, uint32_t us)
{
    CHECK(fakeCanFrame(id, dlc, d));
    hostSetMicros(us);
    hostDrainTick();
}

/// Services until nothing more comes out, collecting each packet as a reading
/// host would, and checks every write as it goes.
static void drainToHost(unsigned maxCalls = 10000u)
{
    for (unsigned i = 0; i < maxCalls; ++i) {
        const uint32_t before = hostUsbWrites();
        const size_t   len0   = strlen(hostUsbOutput());
        canStreamService();
        if (hostUsbWrites() == before) break;
        const char *out = hostUsbOutput();
        const size_t len1 = strlen(out);
        CHECK(len1 > len0 && out[len1 - 1] == '\n');   // whole lines, every write
        hostUsbCollect();
    }
}

static unsigned countLines(const char *s, const char *prefix)
{
    unsigned n = 0;
    const size_t pl = strlen(prefix);
    for (const char *p = s; *p; ) {
        if (strncmp(p, prefix, pl) == 0) ++n;
        const char *nl = strchr(p, '\n');
        if (!nl) break;
        p = nl + 1;
    }
    return n;
}

// ═════════════════════════════════════════════════════════════════════════════
// the lines, byte for byte
// ═════════════════════════════════════════════════════════════════════════════

static void expectLine(const CanRawFrame &f, const char *golden, int line)
{
    char buf[64];
    memset(buf, 0, sizeof(buf));
    const uint8_t n = canFormatFrameLine(buf, (uint8_t)sizeof(buf), f);
    check(n == strlen(golden) && strcmp(buf, golden) == 0, golden, line);
}
#define EXPECT_LINE(f, golden) expectLine((f), (golden), __LINE__)

static void test_frame_lines_golden()
{
    begin("format: F lines exactly as the Orin tools parse them");

    const uint8_t d8[8] = { 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0 };
    const uint8_t d3[3] = { 0x0a, 0x0b, 0x0c };
    CanRawFrame f;

    canRawPack(f, 0x0001ABCDu, 0x158u, false, false, 8, d8);
    EXPECT_LINE(f, "F 0001ABCD 158 8 123456789ABCDEF0\n");

    canRawPack(f, 0u, 0x7FFu, false, false, 0, nullptr);       // DLC 0: no trailing space
    EXPECT_LINE(f, "F 00000000 7FF 0\n");

    canRawPack(f, 0xFFFFFFFFu, 0x001u, false, false, 3, d3);   // zero-padded id, uppercase
    EXPECT_LINE(f, "F FFFFFFFF 001 3 0A0B0C\n");

    canRawPack(f, 0x00C0FFEEu, 0x18DAF110u, true, false, 2, d8);
    EXPECT_LINE(f, "F 00C0FFEE 18DAF110 2 1234\n");

    canRawPack(f, 0x00C0FFEEu, 0x00000158u, true, false, 1, d8);   // ext keeps 8 digits
    EXPECT_LINE(f, "F 00C0FFEE 00000158 1 12\n");

    canRawPack(f, 0x10u, 0x123u, false, true, 4, d8);
    EXPECT_LINE(f, "F 00000010 123 R 4\n");

    canRawPack(f, 0x10u, 0x0ABCDEFu, true, true, 0, nullptr);
    EXPECT_LINE(f, "F 00000010 00ABCDEF R 0\n");

    canRawPack(f, 0x10u, 0x7E8u, false, true, 8, d8);
    EXPECT_LINE(f, "F 00000010 7E8 R 8\n");
}

static void test_frame_line_never_partial()
{
    begin("format: a line that does not fit writes nothing at all");

    const uint8_t d8[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    CanRawFrame f;
    canRawPack(f, 1u, 0x158u, false, false, 8, d8);   // 34 bytes
    char buf[64];
    memset(buf, '#', sizeof(buf));
    CHECK(canFormatFrameLine(buf, 33u, f) == 0u);
    for (unsigned i = 0; i < sizeof(buf); ++i) CHECK(buf[i] == '#');
    CHECK(canFormatFrameLine(buf, 34u, f) == 34u);
    CHECK(canFormatFrameLine(buf, 64u, f) == CAN_STREAM_FRAME_LINE_MAX - 5u);   // std, not ext

    canRawPack(f, 1u, 0x1FFFFFFFu, true, false, 8, d8);
    CHECK(canFormatFrameLine(buf, 64u, f) == CAN_STREAM_FRAME_LINE_MAX);
}

static void test_stats_line_golden()
{
    begin("format: the FS line, from small counters to the 67-byte worst case");

    char buf[80];
    memset(buf, 0, sizeof(buf));
    CHECK(canFormatStatsLine(buf, 80u, 0x0000BEEFu, 1u, 2u, 3u, 4u, 5u) == 22u);
    CHECK(strcmp(buf, "FS 0000BEEF 1 2 3 4 5\n") == 0);

    memset(buf, 0, sizeof(buf));
    CHECK(canFormatStatsLine(buf, 80u, 0u, 0u, 0u, 0u, 0u, 0u) == 22u);
    CHECK(strcmp(buf, "FS 00000000 0 0 0 0 0\n") == 0);

    const uint32_t M = 0xFFFFFFFFu;
    memset(buf, 0, sizeof(buf));
    CHECK(canFormatStatsLine(buf, 80u, M, M, M, M, M, M) == CAN_STREAM_STATS_LINE_MAX);
    CHECK(strcmp(buf, "FS FFFFFFFF 4294967295 4294967295 4294967295 4294967295 4294967295\n") == 0);

    // Over one write's worth: refused whole, never cut.
    memset(buf, '#', sizeof(buf));
    CHECK(canFormatStatsLine(buf, 63u, M, M, M, M, M, M) == 0u);
    CHECK(buf[0] == '#');
}

// ═════════════════════════════════════════════════════════════════════════════
// gating: who is listening, and is the endpoint free
// ═════════════════════════════════════════════════════════════════════════════

static void test_no_host_counts_nohost()
{
    begin("gating: with DTR low nothing is written, and the frames are counted as nohost");

    CHECK(setMode(CanMode::DISCOVER));
    const Totals t0 = totals();
    const uint8_t d[2] = { 1, 2 };
    for (unsigned i = 0; i < 50u; ++i) frameOnBus(0x200, 2, d, 1000u + i);

    for (unsigned i = 0; i < 5u; ++i) { canStreamService(); advance(400u); }
    CHECK(hostUsbWrites() == 0u);
    CHECK(strlen(hostUsbOutput()) == 0u);          // no FS line for nobody either
    CHECK(canStreamNoHostCount() == t0.nohost + 50u);
    CHECK(canStreamStreamedCount() == t0.streamed);
    CHECK(canRawStreamable() == 0u);               // released: the ring stays free
}

static void test_one_packet_of_whole_lines_per_free_bank()
{
    begin("gating: each write is whole lines within availableForWrite, only into a free bank");

    CHECK(setMode(CanMode::DISCOVER));
    flushStats();
    const uint8_t d8[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
    const uint8_t d2[2] = { 0xAB, 0xCD };
    frameOnBus(0x158, 8, d8, 0x100u);
    frameOnBus(0x1AB, 2, d2, 0x200u);
    frameOnBus(0x1ED, 2, d2, 0x300u);

    const uint32_t s0 = canStreamStreamedCount();
    canStreamService();                            // 34 + 22 = 56; the next would be 78
    CHECK(hostUsbWrites() == 1u);
    CHECK(strcmp(hostUsbOutput(), "F 00000100 158 8 0001020304050607\n"
                                  "F 00000200 1AB 2 ABCD\n") == 0);

    canStreamService();                            // bank still armed: must not write
    canStreamService();
    CHECK(hostUsbWrites() == 1u);
    CHECK(hostUsbBlockingWrites() == 0u);

    hostUsbCollect();
    canStreamService();                            // the line that did not fit
    CHECK(hostUsbWrites() == 2u);
    CHECK(strcmp(hostUsbOutput(),
                 "F 00000100 158 8 0001020304050607\n"
                 "F 00000200 1AB 2 ABCD\n"
                 "F 00000300 1ED 2 ABCD\n") == 0);
    CHECK(canStreamStreamedCount() == s0 + 3u);
    CHECK(hostUsbLargestWrite() <= 63u);
    CHECK(hostUsbBlockingWrites() == 0u);
}

static void test_room_limits_and_never_splits()
{
    begin("gating: availableForWrite caps every write, and a line never straddles two");

    CHECK(setMode(CanMode::DISCOVER));
    flushStats();
    const uint8_t d8[8] = { 9, 9, 9, 9, 9, 9, 9, 9 };
    for (unsigned i = 0; i < 6u; ++i) frameOnBus((uint16_t)(0x300 + i), 8, d8, i);   // 34-byte lines

    hostUsbSetRoom(20);                            // smaller than any 8-byte line
    canStreamService();
    CHECK(hostUsbWrites() == 0u);                  // nothing written...
    CHECK(canRawStreamable() == 6u);               // ...and nothing lost

    hostUsbSetRoom(40);                            // room for one line, not two
    drainToHost();
    CHECK(hostUsbWrites() == 6u);
    CHECK(hostUsbLargestWrite() == 34u);
    CHECK(countLines(hostUsbOutput(), "F ") == 6u);
    CHECK(hostUsbBlockingWrites() == 0u);

    hostUsbSetRoom(0);                             // a core that says "nothing"
    frameOnBus(0x400, 0, nullptr, 7u);
    canStreamService();
    CHECK(hostUsbWrites() == 6u);
    hostUsbSetRoom(17);                            // exactly one DLC-0 line
    canStreamService();
    CHECK(hostUsbWrites() == 7u);
    CHECK(hostUsbLargestWrite() == 34u);
}

static void test_host_open_but_not_reading()
{
    // The host has the port open (DTR high) and stops reading: the bank stays
    // armed for good. Nothing may block; frames wait, then the ring drops the
    // newest and says so; and when the host comes back, FS goes first.
    begin("gating: a stalled host never blocks loop(); the ring absorbs, then counts drops");

    CHECK(setMode(CanMode::DISCOVER));
    flushStats();
    hostUsbSetDtr(true);
    frameOnBus(0x100, 1, nullptr, 0u);
    canStreamService();                            // one packet armed...
    CHECK(hostUsbWrites() == 1u);
    // ...and never collected.
    const Totals t0 = totals();
    for (uint32_t n = 0; n < CAN_RING_SLOTS + 40u; ++n) {
        (void)fakeCanFrame((uint16_t)(n & 0x7FFu), 1);
        hostDrainTick();
        canStreamService();
        if ((n % 100u) == 0u) advance(250u);
    }
    CHECK(hostUsbWrites() == 1u);
    CHECK(hostUsbBlockingWrites() == 0u);
    CHECK(canRawStreamable() == CAN_RING_SLOTS);
    CHECK(canRingDropCount() == t0.drop + 40u);
    CHECK(canDrainedCount() == t0.drained + CAN_RING_SLOTS + 40u);

    hostUsbCollect();                              // the host reads again
    hostUsbClearOutput();
    canStreamService();
    CHECK(strncmp(hostUsbOutput(), "FS ", 3) == 0); // the loss figures first
    hostUsbCollect();
    drainToHost(4000u);
    CHECK(canRawStreamable() == 0u);
    CHECK(hostUsbBlockingWrites() == 0u);
}

static void test_not_enumerated()
{
    // lineState can say DTR while no configuration is active (after a bus
    // reset). The core's send() returns at once then; writing would only lose
    // the frames.
    begin("gating: DTR set but not enumerated writes nothing and blocks on nothing");

    CHECK(setMode(CanMode::DISCOVER));
    hostUsbSetDtr(true);
    hostUsbSetConfigured(false);
    frameOnBus(0x100, 1, nullptr, 0u);
    canStreamService();
    CHECK(hostUsbWrites() == 0u);
    CHECK(hostUsbBlockingWrites() == 0u);
    CHECK(canRawStreamable() == 1u);
}

static void test_write_failure_counts_nohost()
{
    // The USB ISR can reset the bus between the checks and the write; the core
    // then returns (size_t)-1. Those lines reached no host.
    begin("gating: a write the core refuses counts its lines as nohost, not streamed");

    CHECK(setMode(CanMode::DISCOVER));
    flushStats();
    frameOnBus(0x100, 1, nullptr, 0u);
    const Totals t0 = totals();
    hostUsbSetWriteFails(true);
    canStreamService();
    CHECK(canStreamStreamedCount() == t0.streamed);
    CHECK(canStreamNoHostCount() == t0.nohost + 1u);
    CHECK(canRawStreamable() == 0u);
}

// ═════════════════════════════════════════════════════════════════════════════
// the FS line
// ═════════════════════════════════════════════════════════════════════════════

static void test_stats_once_per_second()
{
    begin("stats: one FS line a second, first in its packet, counters consistent");

    CHECK(setMode(CanMode::DISCOVER));
    hostUsbSetDtr(true);
    const Totals t0 = totals();
    const uint8_t d[1] = { 0x42 };

    unsigned fs = 0;
    for (unsigned ms = 0; ms < 5000u; ms += 10u) {
        frameOnBus(0x1DC, 1, d, g_t * 1000u);
        canStreamService();
        hostUsbCollect();
        advance(10u);
    }
    fs = countLines(hostUsbOutput(), "FS ");
    CHECK(fs >= 5u && fs <= 6u);                   // the one due at entry, then every second

    // Every line is an F or an FS line — the stream adds nothing else.
    CHECK(countLines(hostUsbOutput(), "F ") + fs ==
          countLines(hostUsbOutput(), ""));

    // The last FS line's counters against the module's: drained is everything
    // read so far, and streamed + nohost + ringdrop + still queued == drained.
    const char *out = hostUsbOutput();
    const char *last = nullptr;
    for (const char *p = out; (p = strstr(p, "FS ")) != nullptr; ++p) last = p;
    CHECK(last != nullptr);
    if (last != nullptr) {
        unsigned us = 0, dr = 0, st = 0, ov = 0, rd = 0, nh = 0;
        CHECK(sscanf(last, "FS %8x %u %u %u %u %u", &us, &dr, &st, &ov, &rd, &nh) == 6);
        CHECK(dr <= canDrainedCount() && st <= canStreamStreamedCount());
    }

    // Conservation over the test: every frame read is streamed, refused for
    // want of a host, dropped by a full ring, or still queued.
    const Totals t = totals();
    CHECK((t.streamed - t0.streamed) + (t.nohost - t0.nohost) + (t.drop - t0.drop) +
          canRawStreamable() == t.drained - t0.drained);
}

static void test_stats_not_owed_without_host()
{
    begin("stats: no FS line is queued up while nobody listens");

    CHECK(setMode(CanMode::DISCOVER));
    for (unsigned i = 0; i < 10u; ++i) { canStreamService(); advance(1000u); }
    CHECK(hostUsbWrites() == 0u);
    hostUsbSetDtr(true);
    canStreamService();                            // one due now, not ten
    CHECK(countLines(hostUsbOutput(), "FS ") == 1u);
}

// ═════════════════════════════════════════════════════════════════════════════
// SNIFF streams too, behind the decoder
// ═════════════════════════════════════════════════════════════════════════════

static void test_sniff_streams_decoded_frames()
{
    begin("sniff: frames are streamed as well as decoded, never before decoding");

    CHECK(setMode(CanMode::SNIFF));
    flushStats();
    const uint8_t d[8] = { 0x12, 0x34, 0, 0, 0, 0, 0, 0 };
    frameOnBus(0x158, 8, d, 0xABCu);
    canStreamService();
    CHECK(hostUsbWrites() == 0u);                  // the decoder has not read it yet
    CHECK(tickCANSniff(g_v, g_y) == 1u);
    canStreamService();
    CHECK(strcmp(hostUsbOutput(), "F 00000ABC 158 8 1234000000000000\n") == 0);
    CHECK(g_v.speedSrc == VehSource::CAN_SNIFF);
}

static void test_stats_columns_are_the_counters()
{
    // Each of the five counters is made distinct and non-zero, so a column fed
    // from the wrong counter — or from none — cannot pass by coincidence.
    begin("stats: each FS column is its own counter, read when the line is formatted");

    CHECK(setMode(CanMode::DISCOVER));
    flushStats();
    const Totals t0 = totals();

    // nohost: 3 frames while nobody listens.
    hostUsbSetDtr(false);
    for (unsigned i = 0; i < 3u; ++i) frameOnBus((uint16_t)(0x120 + i), 1, nullptr, i);
    canStreamService();
    // ovf: 2 events (both flags latched by the next tick that finds a frame).
    (void)fakeCanFrame(0x130);
    fakeCanSetReg(0x2D, 0xC0u);                    // EFLG: RX0OVR | RX1OVR
    hostDrainTick();
    canStreamService();                            // still no host: one more nohost
    // streamed: 5 frames to a reading host.
    hostUsbSetDtr(true);
    hostUsbCollect();
    advance(1u);
    for (unsigned i = 0; i < 5u; ++i) frameOnBus((uint16_t)(0x140 + i), 1, nullptr, 100u + i);
    drainToHost();
    // ringdrop: a stalled host and a ring filled 7 past full.
    frameOnBus(0x150, 1, nullptr, 200u);
    canStreamService();                            // armed, never collected
    for (uint32_t n = 0; n < CAN_RING_SLOTS + 7u; ++n) { (void)fakeCanFrame(0x151, 1); hostDrainTick(); }
    hostUsbCollect();
    hostUsbClearOutput();

    const Totals t = totals();
    CHECK(t.nohost - t0.nohost == 4u);
    CHECK(t.ovf - t0.ovf == 2u);
    CHECK(t.streamed - t0.streamed == 6u);
    CHECK(t.drop - t0.drop == 7u);
    CHECK(t.drained - t0.drained == 4u + 5u + 1u + CAN_RING_SLOTS + 7u);

    advance(CAN_STREAM_STATS_MS);                  // FS due: first in the next packet
    hostSetMicros(0x00ABCDEFu);
    canStreamService();
    unsigned us = 0, dr = 0, st = 0, ov = 0, rd = 0, nh = 0;
    CHECK(sscanf(hostUsbOutput(), "FS %8x %u %u %u %u %u", &us, &dr, &st, &ov, &rd, &nh) == 6);
    CHECK(us == 0x00ABCDEFu);
    CHECK(dr == t.drained);
    CHECK(st == t.streamed);                       // as of formatting: this packet's lines not yet
    CHECK(ov == t.ovf);
    CHECK(rd == t.drop);
    CHECK(nh == t.nohost);
}

static void test_oversized_stats_line_goes_out_alone()
{
    // The FS line is the one line that can outgrow a write. When it does it is
    // handed over whole and ALONE: nothing may be packed behind it into a
    // buffer it has already overfilled.
    begin("stats: an FS line longer than the room is written alone, never packed past the room");

    CHECK(setMode(CanMode::DISCOVER));
    hostUsbSetDtr(true);
    hostUsbSetRoom(21);                            // under the 22-byte shortest FS line
    const uint8_t d[1] = { 0x5A };
    frameOnBus(0x101, 1, d, 1u);                   // 20-byte lines: one fits the room
    frameOnBus(0x102, 1, d, 2u);
    canStreamService();
    CHECK(hostUsbWrites() == 1u);
    CHECK(strncmp(hostUsbOutput(), "FS ", 3) == 0);
    CHECK(countLines(hostUsbOutput(), "F ") == 0u);   // the frames wait for a packet of their own
    CHECK(canRawStreamable() == 2u);
    hostUsbCollect();
    hostUsbClearOutput();
    drainToHost();
    CHECK(strcmp(hostUsbOutput(), "F 00000001 101 1 5A\nF 00000002 102 1 5A\n") == 0);
    CHECK(hostUsbLargestWrite() <= 21u);
}

static void test_sniff_no_host_releases_only_decoded()
{
    // In SNIFF the stream releases slots only behind the decoder. Discarding
    // for want of a host is a release too, and must stop at the same place, or
    // the decoder's index falls behind the tail and it reads slots the drain is
    // already refilling.
    begin("sniff: with no host, only frames the decoder has read are discarded");

    CHECK(setMode(CanMode::SNIFF));
    hostUsbSetDtr(false);
    const Totals t0 = totals();
    const uint8_t d[8] = { 0x12, 0x34, 0, 0, 0, 0, 0, 0 };
    for (unsigned i = 0; i < 4u; ++i) { CHECK(fakeCanFrame(0x158, 8, d)); hostDrainTick(); }

    canStreamService();                            // decoder has read nothing yet
    CHECK(canStreamNoHostCount() == t0.nohost);
    CHECK(canRawStreamable() == 0u);
    CHECK(tickCANSniff(g_v, g_y) == 4u);           // all four still there to decode
    CHECK(canRawStreamable() == 4u);
    canStreamService();
    CHECK(canStreamNoHostCount() == t0.nohost + 4u);
    CHECK(canRawStreamable() == 0u);
    CHECK(tickCANSniff(g_v, g_y) == 0u);
}

static void test_line_break_owed_after_half_lines()
{
    // A host opening the port first collects whatever the bank held — with
    // nobody reading, that can be the start of a console line whose "\r\n"
    // never went out. The first stream line must not continue it.
    begin("framing: after the port was closed, or a console write failed, the next packet starts a new line");

    CHECK(setMode(CanMode::DISCOVER));
    flushStats();
    const uint8_t d[1] = { 0x42 };

    // Port closed, then opened.
    hostUsbSetDtr(false);
    canStreamService();
    hostUsbSetDtr(true);
    frameOnBus(0x111, 1, d, 0x10u);
    canStreamService();
    CHECK(strcmp(hostUsbOutput(), "\nF 00000010 111 1 42\n") == 0);
    hostUsbCollect();
    hostUsbClearOutput();
    frameOnBus(0x112, 1, d, 0x20u);                // owed once, not every packet
    canStreamService();
    CHECK(strcmp(hostUsbOutput(), "F 00000020 112 1 42\n") == 0);
    hostUsbCollect();
    hostUsbClearOutput();

    // A console write that went out whole owes nothing...
    const uint8_t text[] = "IMU: up\r\n";
    CHECK(canStreamConsoleWrite(text, sizeof(text) - 1u) == sizeof(text) - 1u);
    hostUsbCollect();
    hostUsbClearOutput();
    frameOnBus(0x113, 1, d, 0x30u);
    canStreamService();
    CHECK(strcmp(hostUsbOutput(), "F 00000030 113 1 42\n") == 0);
    hostUsbCollect();
    hostUsbClearOutput();

    // ...one dropped because no free bank came (here: not enumerated) may have
    // left half a line behind...
    hostUsbSetConfigured(false);
    CHECK(canStreamConsoleWrite(text, sizeof(text) - 1u) == 0u);
    hostUsbSetConfigured(true);
    frameOnBus(0x114, 1, d, 0x40u);
    canStreamService();
    CHECK(strcmp(hostUsbOutput(), "\nF 00000040 114 1 42\n") == 0);
    hostUsbCollect();
    hostUsbClearOutput();

    // ...and so may one the core refused with the bank free (enumeration lost
    // between the bank check and the write).
    hostUsbSetWriteFails(true);
    CHECK(canStreamConsoleWrite(text, sizeof(text) - 1u) == 0u);
    hostUsbSetWriteFails(false);
    frameOnBus(0x115, 1, d, 0x50u);
    canStreamService();
    CHECK(strcmp(hostUsbOutput(), "\nF 00000050 115 1 42\n") == 0);
    hostUsbCollect();
    hostUsbClearOutput();

    // The oversized FS line carries the break as well, still whole and alone.
    hostUsbSetDtr(false);
    canStreamService();
    hostUsbSetDtr(true);
    hostUsbSetRoom(21);
    advance(CAN_STREAM_STATS_MS);
    canStreamService();
    CHECK(strncmp(hostUsbOutput(), "\nFS ", 4) == 0);
    CHECK(countLines(hostUsbOutput(), "FS ") == 1u);
}

// ═════════════════════════════════════════════════════════════════════════════
// throughput: keeping the bank full, bounded
// ═════════════════════════════════════════════════════════════════════════════

static void queueDlc8(unsigned n, uint32_t us0)
{
    const uint8_t d8[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    for (unsigned i = 0; i < n; ++i) frameOnBus((uint16_t)(0x200 + (i & 0x1FFu)), 8, d8, us0 + i);
}

static void test_reading_host_gets_a_packet_per_collection()
{
    // A host that collects each packet ~30 us after it is armed: one call
    // keeps the bank full, packet after packet, until the ring is empty.
    begin("throughput: one call writes packet after packet to a reading host");

    CHECK(setMode(CanMode::DISCOVER));
    flushStats();
    queueDlc8(10u, 0x1000u);                       // 34-byte lines: one per packet
    hostUsbSetHostLatency(3u);
    hostUsbSetUsPerPoll(10u);
    const uint32_t s0 = canStreamStreamedCount();
    canStreamService();
    CHECK(hostUsbWrites() == 10u);
    CHECK(canStreamStreamedCount() == s0 + 10u);
    CHECK(canRawStreamable() == 0u);
    CHECK(hostUsbBlockingWrites() == 0u);          // every write found the bank free
    CHECK(hostUsbLargestWrite() <= CAN_STREAM_PACKET_MAX);
    CHECK(countLines(hostUsbOutput(), "F ") == 10u);
    // In order: the first and last lines are the first and last frames.
    CHECK(strncmp(hostUsbOutput(), "F 00001000 200 8 ", 17) == 0);
    CHECK(strstr(hostUsbOutput(), "F 00001009 209 8 0102030405060708\n") != nullptr);
}

static void test_packets_per_call_bounded()
{
    begin("throughput: a call never writes more than CAN_STREAM_MAX_PACKETS_PER_CALL packets");

    CHECK(setMode(CanMode::DISCOVER));
    flushStats();
    queueDlc8(CAN_STREAM_MAX_PACKETS_PER_CALL + 9u, 0u);
    hostUsbSetHostLatency(1u);                     // collected at the first look
    hostUsbSetUsPerPoll(1u);
    canStreamService();
    CHECK(hostUsbWrites() == CAN_STREAM_MAX_PACKETS_PER_CALL);
    CHECK(canRawStreamable() == 9u);
    canStreamService();                            // the rest, next call
    CHECK(hostUsbWrites() == CAN_STREAM_MAX_PACKETS_PER_CALL + 9u);
    CHECK(hostUsbBlockingWrites() == 0u);
}

static void test_stalled_host_costs_one_bounded_wait()
{
    // The host has the port open and has stopped reading. The call that armed
    // the last packet waits for it — CAN_STREAM_WAIT_US, no longer — and every
    // later call takes one look at the armed bank and leaves.
    begin("throughput: a stalled host costs one wait of CAN_STREAM_WAIT_US, then none");

    CHECK(setMode(CanMode::DISCOVER));
    flushStats();
    queueDlc8(5u, 0u);
    hostUsbSetUsPerPoll(10u);                      // latency 0: never collected
    const uint32_t us0 = micros();
    const uint32_t p0  = hostUsbIdlePolls();
    canStreamService();
    const uint32_t waited = micros() - us0;
    CHECK(hostUsbWrites() == 1u);
    CHECK(waited >= CAN_STREAM_WAIT_US && waited <= CAN_STREAM_WAIT_US + 10u);
    CHECK(hostUsbIdlePolls() - p0 <= CAN_STREAM_WAIT_US / 10u + 2u);

    for (unsigned i = 0; i < 20u; ++i) {
        const uint32_t p1 = hostUsbIdlePolls();
        const uint32_t u1 = micros();
        canStreamService();
        CHECK(hostUsbIdlePolls() - p1 == 1u);      // one look...
        CHECK(micros() - u1 == 10u);               // ...and no waiting
    }
    CHECK(hostUsbWrites() == 1u);
    CHECK(hostUsbBlockingWrites() == 0u);
    CHECK(canRawStreamable() == 4u);
}

static void test_call_budget_caps_waiting()
{
    // A slow but live host: 280 us a packet, inside the per-packet bound, so
    // only the call's budget can end the call.
    begin("throughput: a slow host is served until CAN_STREAM_CALL_BUDGET_US, then the call returns");

    CHECK(setMode(CanMode::DISCOVER));
    flushStats();
    queueDlc8(12u, 0u);
    hostUsbSetHostLatency(28u);
    hostUsbSetUsPerPoll(10u);
    const uint32_t us0 = micros();
    canStreamService();
    const uint32_t spent = micros() - us0;
    CHECK(spent <= CAN_STREAM_CALL_BUDGET_US + 10u);
    CHECK(hostUsbWrites() == 1u + CAN_STREAM_CALL_BUDGET_US / 280u);
    CHECK(hostUsbBlockingWrites() == 0u);
}

static void test_idle_stream_never_looks()
{
    begin("throughput: with nothing (more) queued a call neither polls the bank nor waits");

    CHECK(setMode(CanMode::DISCOVER));
    flushStats();
    hostUsbSetUsPerPoll(10u);
    const uint32_t p0 = hostUsbIdlePolls();
    const uint32_t u0 = micros();
    for (unsigned i = 0; i < 50u; ++i) canStreamService();
    CHECK(hostUsbIdlePolls() == p0);
    CHECK(micros() == u0);
    CHECK(hostUsbWrites() == 0u);

    // Nor does a call whose packet emptied the ring wait for the bank after
    // it — even with a host that never collects.
    frameOnBus(0x321, 2, nullptr, u0);
    canStreamService();
    CHECK(hostUsbWrites() == 1u);
    CHECK(hostUsbIdlePolls() == p0 + 1u);          // the one look before the write
    CHECK(micros() == u0 + 10u);
}

// ═════════════════════════════════════════════════════════════════════════════
// console text in whole packets
// ═════════════════════════════════════════════════════════════════════════════

static void test_console_write_whole_packets()
{
    begin("console: text goes out in packets of at most 63 bytes, each into a free bank");

    char text[151];
    for (unsigned i = 0; i < 150u; ++i) text[i] = (char)('a' + (i % 26u));
    text[149] = '\n';
    text[150] = '\0';

    hostUsbSetDtr(true);
    hostUsbSetHostLatency(3u);
    hostUsbSetUsPerPoll(10u);
    CHECK(canStreamConsoleWrite(reinterpret_cast<const uint8_t *>(text), 150u) == 150u);
    CHECK(strcmp(hostUsbOutput(), text) == 0);
    CHECK(hostUsbWrites() == 3u);                  // 63 + 63 + 24
    CHECK(hostUsbLargestWrite() == CAN_STREAM_PACKET_MAX);
    CHECK(hostUsbBlockingWrites() == 0u);          // the core's send() never had to wait
}

static void test_console_write_stalled_host_drops_text()
{
    // A host that stopped reading: the first packet goes into the free bank,
    // the wait for the second is bounded, and the rest is DROPPED rather than
    // handed to the core, whose send() would spin 70 ms on the armed bank (an
    // IMU data gap on the board). The stream's next packet starts a new line.
    begin("console: a stalled host bounds the wait, and text it will not take is dropped, never armed");

    hostUsbSetDtr(true);
    hostUsbSetUsPerPoll(10u);                      // never collected
    const char *text = "0123456789012345678901234567890123456789012345678901234567890123456789\n";
    const size_t n = strlen(text);
    const uint32_t u0 = micros();
    CHECK(canStreamConsoleWrite(reinterpret_cast<const uint8_t *>(text), n) == CAN_STREAM_PACKET_MAX);
    CHECK(strncmp(hostUsbOutput(), text, CAN_STREAM_PACKET_MAX) == 0);
    CHECK(strlen(hostUsbOutput()) == CAN_STREAM_PACKET_MAX);
    CHECK(hostUsbWrites() == 1u);
    CHECK(hostUsbBlockingWrites() == 0u);          // the core's send() never waited
    CHECK(micros() - u0 <= CAN_CONSOLE_WAIT_US + 10u);

    // Nobody listening: nothing written, no waiting at all.
    hostUsbClearOutput();
    hostUsbCollect();
    hostUsbSetDtr(false);
    const uint32_t p0 = hostUsbIdlePolls();
    const uint32_t w0 = hostUsbWrites();
    CHECK(canStreamConsoleWrite(reinterpret_cast<const uint8_t *>(text), n) == 0u);
    CHECK(hostUsbIdlePolls() == p0);
    CHECK(hostUsbWrites() == w0);

    // Not enumerated: the core refuses at once, and so does this.
    hostUsbSetDtr(true);
    hostUsbSetConfigured(false);
    CHECK(canStreamConsoleWrite(reinterpret_cast<const uint8_t *>(text), n) == 0u);
}

// ─── runner ──────────────────────────────────────────────────────────────────

int main()
{
    test_frame_lines_golden();
    test_frame_line_never_partial();
    test_stats_line_golden();

    test_no_host_counts_nohost();
    test_one_packet_of_whole_lines_per_free_bank();
    test_room_limits_and_never_splits();
    test_host_open_but_not_reading();
    test_not_enumerated();
    test_write_failure_counts_nohost();

    test_stats_once_per_second();
    test_stats_not_owed_without_host();

    test_sniff_streams_decoded_frames();
    test_stats_columns_are_the_counters();
    test_oversized_stats_line_goes_out_alone();
    test_sniff_no_host_releases_only_decoded();
    test_line_break_owed_after_half_lines();

    test_reading_host_gets_a_packet_per_collection();
    test_packets_per_call_bounded();
    test_stalled_host_costs_one_bounded_wait();
    test_call_budget_caps_waiting();
    test_idle_stream_never_looks();

    test_console_write_whole_packets();
    test_console_write_stalled_host_drops_text();

    printf("can_stream_tests: %d checks, %d failed\n", g_checks, g_fails);
    return g_fails;
}
