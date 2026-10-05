/**
 * @file can_ring_tests.cpp
 * @brief The ISR -> loop() frame ring in lib/CANFrameRing.h, on its own.
 *
 * Header-only and hardware-free, so the ring is tested exactly as compiled into
 * the firmware, with no MCP2515 or drain in the way: the frame packing (the DLC
 * trick that keeps a frame at 16 bytes), FIFO order, the full ring refusing and
 * counting, free-running indices wrapping past 65535, and the consumer peeking
 * ahead of its own tail the way SNIFF's decoder does.
 *
 * A single thread cannot interleave a real ISR with loop(), so "the producer
 * runs between the consumer's peek and its release" is staged explicitly
 * instead — which is the only interleaving that can corrupt a slot.
 *
 * Exit status is the number of failures, so `make check` fails the build.
 */

#include "CANFrameRing.h"

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

// Large: the ring is 16 KB and must not live on the stack of a test.
static CanFrameRing g_ring;

static void begin(const char *name)
{
    g_case = name;
    memset(&g_ring, 0xA5, sizeof(g_ring));   // garbage everywhere a reset must cover
    canRingReset(g_ring);
}

/// A frame whose every byte says which one it is, so order and integrity are
/// both checked by comparing against makeFrame(n).
static CanRawFrame makeFrame(uint32_t n)
{
    uint8_t d[8];
    for (uint8_t i = 0; i < 8; ++i) d[i] = (uint8_t)(n * 7u + i);
    CanRawFrame f;
    canRawPack(f, 0x10000000u + n, n & 0x7FFu, false, false, (uint8_t)(n % 9u), d);
    return f;
}

static bool sameFrame(const CanRawFrame &a, const CanRawFrame &b)
{
    return memcmp(&a, &b, sizeof(CanRawFrame)) == 0;
}

// ═════════════════════════════════════════════════════════════════════════════
// packing
// ═════════════════════════════════════════════════════════════════════════════

static void test_pack_every_dlc()
{
    begin("pack: a standard data frame of every length round-trips");

    const uint8_t d[8] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 };
    for (uint8_t dlc = 0; dlc <= 8u; ++dlc) {
        CanRawFrame f;
        canRawPack(f, 0xDEADBEEFu, 0x158u, false, false, dlc, d);
        CHECK(f.us == 0xDEADBEEFu);
        CHECK(canRawId(f) == 0x158u);
        CHECK(!canRawExt(f));
        CHECK(!canRawRtr(f));
        CHECK(canRawDlc(f) == dlc);
        for (uint8_t i = 0; i < dlc; ++i) CHECK(f.data[i] == d[i]);
        // Unused bytes are zero — except data[7], which carries a DLC below 8.
        for (uint8_t i = dlc; i < 7u; ++i) CHECK(f.data[i] == 0u);
    }
}

static void test_pack_dlc_above_8()
{
    // ISO 11898-1: 9-15 still means eight bytes. The raw field is four bits and
    // conforming senders do use the high values.
    begin("pack: DLC 9-15 is clamped to 8, with all eight bytes kept");

    const uint8_t d[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    for (uint8_t raw = 9; raw <= 15u; ++raw) {
        CanRawFrame f;
        canRawPack(f, 0u, 0x100u, false, false, raw, d);
        CHECK(canRawDlc(f) == 8u);
        CHECK(f.data[7] == 8u);   // a payload byte here, not a length
    }
}

static void test_pack_extended_and_remote()
{
    begin("pack: extended identifiers keep all 29 bits; remote frames carry no data");

    const uint8_t d[8] = { 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE };
    CanRawFrame f;

    canRawPack(f, 1u, 0x1FFFFFFFu, true, false, 8u, d);
    CHECK(canRawExt(f));
    CHECK(canRawId(f) == 0x1FFFFFFFu);
    CHECK(canRawDlc(f) == 8u);

    // An extended id whose low 11 bits equal a mapped standard one must stay
    // distinguishable: the flag, not the value, says which it is.
    canRawPack(f, 1u, 0x158u, true, false, 2u, d);
    CHECK(canRawExt(f));
    CHECK(canRawId(f) == 0x158u);

    for (uint8_t dlc = 0; dlc <= 8u; ++dlc) {
        canRawPack(f, 2u, 0x123u, false, true, dlc, d);
        CHECK(canRawRtr(f));
        CHECK(canRawDlc(f) == dlc);            // the length REQUESTED survives
        for (uint8_t i = 0; i < 7u; ++i) CHECK(f.data[i] == 0u);   // and no stale payload
    }

    canRawPack(f, 3u, 0x0ABCDEFu, true, true, 0u, d);
    CHECK(canRawExt(f) && canRawRtr(f));
    CHECK(canRawId(f) == 0x0ABCDEFu);
    CHECK(canRawDlc(f) == 0u);
}

// ═════════════════════════════════════════════════════════════════════════════
// FIFO, full, wrap
// ═════════════════════════════════════════════════════════════════════════════

static void test_empty_ring()
{
    begin("ring: a reset ring is empty and peeking at its head finds nothing");

    CanRawFrame f;
    CHECK(canRingCount(g_ring) == 0u);
    CHECK(!canRingPeek(g_ring, g_ring.tail, f));
    CHECK(g_ring.drops == 0u);
}

static void test_fifo_order()
{
    begin("ring: frames come out in the order they went in, intact");

    for (uint32_t n = 0; n < 100u; ++n) CHECK(canRingPush(g_ring, makeFrame(n)));
    CHECK(canRingCount(g_ring) == 100u);

    for (uint32_t n = 0; n < 100u; ++n) {
        CanRawFrame f;
        CHECK(canRingPeek(g_ring, g_ring.tail, f));
        CHECK(sameFrame(f, makeFrame(n)));
        canRingRelease(g_ring, 1u);
    }
    CHECK(canRingCount(g_ring) == 0u);
}

static void test_full_refuses_and_counts()
{
    begin("ring: a full ring refuses the NEWEST frame and counts it");

    for (uint32_t n = 0; n < CAN_RING_SLOTS; ++n) CHECK(canRingPush(g_ring, makeFrame(n)));
    CHECK(canRingCount(g_ring) == CAN_RING_SLOTS);
    CHECK(g_ring.drops == 0u);

    // Full is exactly SLOTS, with no slot sacrificed to tell full from empty.
    CHECK(!canRingPush(g_ring, makeFrame(999999u)));
    CHECK(!canRingPush(g_ring, makeFrame(999998u)));
    CHECK(g_ring.drops == 2u);
    CHECK(canRingCount(g_ring) == CAN_RING_SLOTS);

    // The oldest frames were kept, not overwritten by the refused ones.
    CanRawFrame f;
    CHECK(canRingPeek(g_ring, g_ring.tail, f));
    CHECK(sameFrame(f, makeFrame(0u)));
    CHECK(canRingPeek(g_ring, (uint16_t)(g_ring.tail + CAN_RING_SLOTS - 1u), f));
    CHECK(sameFrame(f, makeFrame(CAN_RING_SLOTS - 1u)));

    // One released slot is one more frame accepted, and no more.
    canRingRelease(g_ring, 1u);
    CHECK(canRingPush(g_ring, makeFrame(CAN_RING_SLOTS)));
    CHECK(!canRingPush(g_ring, makeFrame(CAN_RING_SLOTS + 1u)));
    CHECK(g_ring.drops == 3u);
}

static void test_indices_wrap()
{
    // The indices run free and are never folded to the slot count, so they
    // cross 65535 -> 0 every 64 laps. Count and order must not notice.
    begin("ring: free-running indices wrap past 65535 without losing count or order");

    uint32_t in = 0, out = 0;
    for (uint32_t step = 0; step < 300000u; ++step) {
        // Uneven bursts, so head and tail cross the wrap at different moments
        // and the fill level sweeps from empty to nearly full.
        const uint32_t burst = (step % 7u) + 1u;
        for (uint32_t k = 0; k < burst && canRingCount(g_ring) < CAN_RING_SLOTS; ++k) {
            if (!canRingPush(g_ring, makeFrame(in))) break;
            ++in;
        }
        const uint32_t take = (step % 5u) + 1u;
        for (uint32_t k = 0; k < take; ++k) {
            CanRawFrame f;
            if (!canRingPeek(g_ring, g_ring.tail, f)) break;
            if (!sameFrame(f, makeFrame(out))) { CHECK(sameFrame(f, makeFrame(out))); return; }
            canRingRelease(g_ring, 1u);
            ++out;
        }
        if (canRingCount(g_ring) != (uint16_t)(in - out)) { CHECK(canRingCount(g_ring) == (uint16_t)(in - out)); return; }
    }
    CHECK(in > 3u * 65536u);   // the head really did wrap, several times
    CHECK(g_ring.drops == 0u);
}

// ═════════════════════════════════════════════════════════════════════════════
// the consumer side as the firmware uses it
// ═════════════════════════════════════════════════════════════════════════════

static void test_peek_ahead_of_tail()
{
    // SNIFF's decoder reads at its own index, ahead of the stream's tail, and
    // only the tail frees slots. Peeking anywhere in [tail, head) must work and
    // must not release anything.
    begin("ring: the consumer can peek ahead of its tail without releasing");

    for (uint32_t n = 0; n < 10u; ++n) CHECK(canRingPush(g_ring, makeFrame(n)));
    const uint16_t tail = g_ring.tail;
    for (uint16_t i = 0; i < 10u; ++i) {
        CanRawFrame f;
        CHECK(canRingPeek(g_ring, (uint16_t)(tail + i), f));
        CHECK(sameFrame(f, makeFrame(i)));
    }
    CanRawFrame f;
    CHECK(!canRingPeek(g_ring, (uint16_t)(tail + 10u), f));   // that is head
    CHECK(canRingCount(g_ring) == 10u);
    CHECK(g_ring.tail == tail);
}

static void test_producer_between_peek_and_release()
{
    // REGRESSION GUARD for the SPSC contract. The ISR may run at any point of a
    // consumer's peek-then-release. Released slots are the only ones it may
    // write, so a frame still being read is safe even if the ISR fills every
    // other slot in the meantime.
    begin("ring: an ISR filling the ring between peek and release cannot touch the peeked slot");

    for (uint32_t n = 0; n < CAN_RING_SLOTS - 1u; ++n) CHECK(canRingPush(g_ring, makeFrame(n)));

    CanRawFrame peeked;
    CHECK(canRingPeek(g_ring, g_ring.tail, peeked));      // consumer starts on frame 0

    CHECK(canRingPush(g_ring, makeFrame(5000u)));          // "ISR": fills the last slot
    CHECK(!canRingPush(g_ring, makeFrame(5001u)));         // and is refused after that
    CHECK(g_ring.drops == 1u);

    CanRawFrame again;
    CHECK(canRingPeek(g_ring, g_ring.tail, again));
    CHECK(sameFrame(again, makeFrame(0u)));                // the slot was not overwritten
    CHECK(sameFrame(peeked, makeFrame(0u)));
    canRingRelease(g_ring, 1u);                            // consumer finishes
    CHECK(canRingPush(g_ring, makeFrame(5002u)));          // and only now is it reusable
}

static void test_release_many()
{
    begin("ring: releasing n frames at once frees exactly n");

    for (uint32_t n = 0; n < 50u; ++n) CHECK(canRingPush(g_ring, makeFrame(n)));
    canRingRelease(g_ring, 20u);
    CHECK(canRingCount(g_ring) == 30u);
    CanRawFrame f;
    CHECK(canRingPeek(g_ring, g_ring.tail, f));
    CHECK(sameFrame(f, makeFrame(20u)));
}

// ─── runner ──────────────────────────────────────────────────────────────────

int main()
{
    test_pack_every_dlc();
    test_pack_dlc_above_8();
    test_pack_extended_and_remote();

    test_empty_ring();
    test_fifo_order();
    test_full_refuses_and_counts();
    test_indices_wrap();

    test_peek_ahead_of_tail();
    test_producer_between_peek_and_release();
    test_release_many();

    printf("can_ring_tests: %d checks, %d failed\n", g_checks, g_fails);
    return g_fails;
}
