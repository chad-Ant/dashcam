/**
 * @file switch_tests.cpp
 * @brief Deterministic tests for lib/SwitchFunctions.cpp.
 *
 * Every case below corresponds either to a contract stated in
 * SwitchFunctions.h or to a defect that was actually shipped and found in
 * review.  The ones marked REGRESSION fail against the code as it was written
 * the first time, which is the only evidence that a test is testing anything.
 *
 * The real transport is exercised too, against the 74HC165 model in
 * arduino_stub.cpp — so bit order, the read-before-clock rule and the sentinel
 * positions are all covered, not merely the state machine.
 *
 * Exit status is the number of failures, so `make check` fails the build.
 */

#include "SwitchFunctions.h"

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

static void begin(const char *name)
{
    g_case = name;
    hostReset();
}

// ─── helpers ─────────────────────────────────────────────────────────────────

/// Raw word for a set of CLOSED switches, with the sentinels correct.
static uint16_t rawFor(uint16_t closed)
{
    uint16_t raw = static_cast<uint16_t>(~static_cast<uint16_t>(closed << SWITCH_SENTINEL_BITS));
    raw = static_cast<uint16_t>(raw & static_cast<uint16_t>(~0x0003u));
    return static_cast<uint16_t>(raw | 0x0002u);
}

/// n polls of the same word, advancing the clock one interval each.
static void feed(SwitchData &sw, uint16_t raw, uint32_t &t, unsigned n)
{
    for (unsigned i = 0u; i < n; ++i) {
        t += SWITCH_POLL_MS;
        (void)switchIngest(sw, raw, t);
    }
}

/// Brings sw to a qualified baseline of @p closed. Leaves changed cleared.
static void qualifyAt(SwitchData &sw, uint16_t closed, uint32_t &t)
{
    switchResetState(sw, t);
    feed(sw, rawFor(closed), t, SWITCH_QUALIFY_READS);
    switchClearChanged(sw);
}

// ═════════════════════════════════════════════════════════════════════════════
// transport — the shift loop against the device model
// ═════════════════════════════════════════════════════════════════════════════

static void test_transport_bit_order()
{
    begin("transport: a read returns the panel word unchanged");
    SwitchData sw;

    // Deliberately asymmetric so a reversed shift, an off-by-one, or a swapped
    // byte order all produce a different answer. Sentinels valid: bit0=0, bit1=1.
    const uint16_t panel = 0xA5FAu;
    fakeChainSetPanel(panel);
    initializeSwitches(sw);

    CHECK(switchReadRaw() == panel);

    // One load pulse and exactly one clock per bit. A driver that clocked
    // before reading would need seventeen edges to see the same data.
    CHECK(fakeChainLoadPulses() == 1u);
    CHECK(fakeChainClocks() == SWITCH_CHAIN_BITS);
}

static void test_transport_msb_first()
{
    begin("transport: the first bit out is the far register's D7");
    SwitchData sw;
    initializeSwitches(sw);

    // Toggling only bit 15 of the panel must move only bit 15 of the result.
    fakeChainSetPanel(0x0002u);
    const uint16_t low = switchReadRaw();
    fakeChainSetPanel(0x8002u);
    const uint16_t high = switchReadRaw();

    CHECK(low == 0x0002u);
    CHECK(high == 0x8002u);
    CHECK(static_cast<uint16_t>(low ^ high) == 0x8000u);
}

static void test_absent_chain_reads_all_ones()
{
    begin("transport: an absent chain reads 0xFFFF and fails the sentinels");
    SwitchData sw;
    initializeSwitches(sw);
    fakeChainSetPresent(false);

    // REGRESSION. The data pin was configured as a bare INPUT on the argument
    // that a pull-up would "mask" an absent chain at 0xFFFF. It does the
    // opposite: 0xFFFF fails on bit 0, which must read LOW. Floating was the
    // unsafe choice, because random-looking data satisfies a two-bit pattern
    // roughly one read in four.
    CHECK(hostPinMode(SWITCH_DATA_PIN) == INPUT_PULLUP);
    CHECK(switchReadRaw() == 0xFFFFu);

    uint32_t t = 0u;
    switchResetState(sw, t);
    t += SWITCH_POLL_MS;
    CHECK(switchIngest(sw, 0xFFFFu, t) == false);
}

// ═════════════════════════════════════════════════════════════════════════════
// sentinel gate
// ═════════════════════════════════════════════════════════════════════════════

static void test_sentinel_polarity()
{
    begin("sentinels: bit0 must be low and bit1 high, independently");
    SwitchData sw;
    uint32_t t = 0u;
    switchResetState(sw, t);

    const uint16_t good = rawFor(0u);
    t += SWITCH_POLL_MS;  CHECK(switchIngest(sw, good, t) == true);

    // bit0 stuck high — the GND sentinel open.
    t += SWITCH_POLL_MS;  CHECK(switchIngest(sw, static_cast<uint16_t>(good | 0x0001u), t) == false);
    // bit1 stuck low — the 3V3 sentinel open.
    t += SWITCH_POLL_MS;  CHECK(switchIngest(sw, static_cast<uint16_t>(good & ~0x0002u), t) == false);
}

static void test_rejected_read_leaves_state_alone()
{
    begin("sentinels: a rejected read never reaches state or changed");
    SwitchData sw;
    uint32_t t = 0u;
    qualifyAt(sw, 0x0005u, t);

    const uint16_t before = sw.state;
    feed(sw, 0xFFFFu, t, 8u);

    CHECK(sw.state == before);      // held, not zeroed
    CHECK(sw.changed == 0u);
    CHECK(sw.present == false);
    CHECK(sw.rejected == 8u);
}

static void test_absence_needs_repeated_failures()
{
    begin("sentinels: one bad read is not an absent chain");
    SwitchData sw;
    uint32_t t = 0u;
    qualifyAt(sw, 0u, t);

    for (unsigned i = 1u; i < SWITCH_ABSENT_READS; ++i) {
        feed(sw, 0xFFFFu, t, 1u);
        CHECK(sw.present == true);
    }
    feed(sw, 0xFFFFu, t, 1u);
    CHECK(sw.present == false);
}

// ═════════════════════════════════════════════════════════════════════════════
// qualification
// ═════════════════════════════════════════════════════════════════════════════

static void test_no_state_published_before_qualification()
{
    begin("qualify: present stays false until the baseline holds still");
    SwitchData sw;
    uint32_t t = 0u;
    switchResetState(sw, t);

    CHECK(sw.present == false);

    feed(sw, rawFor(0x0003u), t, SWITCH_QUALIFY_READS - 1u);
    CHECK(sw.present == false);
    CHECK(sw.state == 0u);

    feed(sw, rawFor(0x0003u), t, 1u);
    CHECK(sw.present == true);
    CHECK(sw.state == 0x0003u);

    // REGRESSION. Initialisation used to prime state from a single raw read, so
    // one sample landing mid-throw became the published position and was then
    // "corrected" 80 ms later by a changed bit describing nothing an operator
    // had done. A first baseline reports no change, ever.
    CHECK(sw.changed == 0u);
}

static void test_unstable_sample_restarts_qualification()
{
    begin("qualify: a moving sample restarts the count");
    SwitchData sw;
    uint32_t t = 0u;
    switchResetState(sw, t);

    feed(sw, rawFor(0x0001u), t, SWITCH_QUALIFY_READS - 1u);
    feed(sw, rawFor(0x0002u), t, 1u);            // disagrees: back to one
    CHECK(sw.present == false);

    feed(sw, rawFor(0x0002u), t, SWITCH_QUALIFY_READS - 2u);
    CHECK(sw.present == false);                   // still one short

    feed(sw, rawFor(0x0002u), t, 1u);
    CHECK(sw.present == true);
    CHECK(sw.state == 0x0002u);
}

static void test_requalify_after_outage_reports_the_difference()
{
    begin("qualify: a panel that moved during an outage is reported changed");
    SwitchData sw;
    uint32_t t = 0u;
    qualifyAt(sw, 0x0001u, t);

    feed(sw, 0xFFFFu, t, SWITCH_ABSENT_READS);
    CHECK(sw.present == false);

    feed(sw, rawFor(0x0009u), t, SWITCH_QUALIFY_READS);
    CHECK(sw.present == true);
    CHECK(sw.state == 0x0009u);
    // The moment is unrecoverable; the fact is not. A host resuming to a
    // different panel should not have to diff two frames minutes apart.
    CHECK(sw.changed == static_cast<uint16_t>(0x0001u ^ 0x0009u));
}

static void test_requalify_unchanged_reports_nothing()
{
    begin("qualify: an outage that changed nothing reports nothing");
    SwitchData sw;
    uint32_t t = 0u;
    qualifyAt(sw, 0x0021u, t);

    feed(sw, 0xFFFFu, t, SWITCH_ABSENT_READS + 4u);
    feed(sw, rawFor(0x0021u), t, SWITCH_QUALIFY_READS);

    CHECK(sw.present == true);
    CHECK(sw.state == 0x0021u);
    CHECK(sw.changed == 0u);
}

// ═════════════════════════════════════════════════════════════════════════════
// debounce
// ═════════════════════════════════════════════════════════════════════════════

static void test_commit_requires_full_run()
{
    begin("debounce: a change commits on the Nth consecutive sample, not before");
    SwitchData sw;
    uint32_t t = 0u;
    qualifyAt(sw, 0u, t);

    feed(sw, rawFor(0x0001u), t, SWITCH_DEBOUNCE_SAMPLES - 1u);
    CHECK(sw.state == 0u);
    CHECK(sw.changed == 0u);

    feed(sw, rawFor(0x0001u), t, 1u);
    CHECK(sw.state == 0x0001u);
    CHECK(sw.changed == 0x0001u);
}

static void test_transient_never_commits()
{
    begin("debounce: a sample that reverts in time is discarded");
    SwitchData sw;
    uint32_t t = 0u;
    qualifyAt(sw, 0u, t);

    feed(sw, rawFor(0x0001u), t, SWITCH_DEBOUNCE_SAMPLES - 1u);
    feed(sw, rawFor(0u), t, 1u);
    feed(sw, rawFor(0x0001u), t, SWITCH_DEBOUNCE_SAMPLES - 1u);

    CHECK(sw.state == 0u);
    CHECK(sw.changed == 0u);
}

static void test_one_bad_input_cannot_block_another()
{
    begin("debounce: integrators are per input, not per word");
    SwitchData sw;
    uint32_t t = 0u;
    qualifyAt(sw, 0u, t);

    // SW00 alternates every poll; SW01 is held closed throughout.
    for (unsigned i = 0u; i < 8u; ++i) {
        const uint16_t closed = static_cast<uint16_t>(0x0002u | ((i & 1u) ? 0x0001u : 0u));
        feed(sw, rawFor(closed), t, 1u);
    }

    // A shared "N identical reads in a row" counter would be reset by SW00 on
    // every pass and SW01 would never commit — and the input most likely to be
    // chattering is precisely the one that would hold the others hostage.
    CHECK((sw.state & 0x0002u) == 0x0002u);
    CHECK((sw.state & 0x0001u) == 0u);
}

static void test_outage_destroys_debounce_evidence()
{
    begin("debounce: evidence does not survive an outage");
    SwitchData sw;
    uint32_t t = 0u;
    qualifyAt(sw, 0u, t);

    // REGRESSION. The rejection path used to leave agree[] standing, so three
    // disagreeing samples, then an outage of any length, then ONE valid sample
    // satisfied a supposedly four-consecutive-sample debounce — four samples
    // that agreed, separated by an interval in which the switch could have been
    // moved twice.
    feed(sw, rawFor(0x0001u), t, SWITCH_DEBOUNCE_SAMPLES - 1u);
    feed(sw, 0xFFFFu, t, 1u);                    // one bad read: not yet absent
    CHECK(sw.present == true);

    feed(sw, rawFor(0x0001u), t, 1u);
    CHECK(sw.state == 0u);                        // must NOT have committed
    CHECK(sw.changed == 0u);

    feed(sw, rawFor(0x0001u), t, SWITCH_DEBOUNCE_SAMPLES - 1u);
    CHECK(sw.state == 0x0001u);                   // a full fresh run does commit
}

static void test_changed_is_sticky_until_cleared()
{
    begin("changed: sticky across polls, cleared only on demand");
    SwitchData sw;
    uint32_t t = 0u;
    qualifyAt(sw, 0u, t);

    feed(sw, rawFor(0x0004u), t, SWITCH_DEBOUNCE_SAMPLES);
    CHECK(sw.changed == 0x0004u);

    // A switch flipped and returned between two published frames is invisible
    // in level alone; it must remain visible here.
    feed(sw, rawFor(0u), t, SWITCH_DEBOUNCE_SAMPLES);
    CHECK(sw.state == 0u);
    CHECK(sw.changed == 0x0004u);

    switchClearChanged(sw);
    CHECK(sw.changed == 0u);
    feed(sw, rawFor(0u), t, 4u);
    CHECK(sw.changed == 0u);
}

// ═════════════════════════════════════════════════════════════════════════════
// chatter
// ═════════════════════════════════════════════════════════════════════════════

/// Alternates SW00 for @p n polls, keeping the other inputs still.
static void alternate(SwitchData &sw, uint32_t &t, unsigned n)
{
    for (unsigned i = 0u; i < n; ++i) {
        feed(sw, rawFor(static_cast<uint16_t>((i & 1u) ? 0x0001u : 0u)), t, 1u);
    }
}

static void test_fast_chatter_is_detected()
{
    begin("chatter: an input alternating every poll is flagged");
    SwitchData sw;
    uint32_t t = 0u;
    qualifyAt(sw, 0u, t);

    // REGRESSION, and the worst of the shipped defects. Transitions used to be
    // counted only after a fully debounced commit — but an input alternating
    // every poll resets the integrator on every pass, so it NEVER commits, so
    // it reported zero transitions and perfect health while flapping at 50 Hz.
    // Only slow chatter was counted, and slow chatter is also what a person
    // working a switch briskly looks like. Exactly the wrong way round.
    alternate(sw, t, static_cast<unsigned>(SWITCH_CHATTER_BUCKET_MS / SWITCH_POLL_MS) + 2u);

    CHECK((sw.chatter & 0x0001u) == 0x0001u);
    // And it must not be reported as switch activity: never committing is the
    // correct behaviour, which is why the detector is what tells anyone.
    CHECK(sw.state == 0u);
}

static void test_brisk_human_use_is_not_chatter()
{
    begin("chatter: vigorous but legitimate operation is not flagged");
    SwitchData sw;
    uint32_t t = 0u;
    qualifyAt(sw, 0u, t);

    // Ten throws inside one bucket — far more than anyone manages on a rocker,
    // and still an order of magnitude under the threshold because each throw is
    // one raw edge at a 20 ms poll.
    const unsigned pollsPerBucket =
        static_cast<unsigned>(SWITCH_CHATTER_BUCKET_MS / SWITCH_POLL_MS);
    const unsigned dwell = pollsPerBucket / 10u;
    for (unsigned k = 0u; k < 10u; ++k) {
        feed(sw, rawFor(static_cast<uint16_t>((k & 1u) ? 0x0001u : 0u)), t, dwell);
    }
    feed(sw, rawFor(0u), t, 4u);   // cross the bucket boundary

    CHECK(sw.chatter == 0u);
}

static void test_burst_across_a_bucket_boundary_is_caught()
{
    begin("chatter: a burst straddling a boundary is still seen");
    SwitchData sw;
    uint32_t t = 0u;
    qualifyAt(sw, 0u, t);

    const unsigned pollsPerBucket =
        static_cast<unsigned>(SWITCH_CHATTER_BUCKET_MS / SWITCH_POLL_MS);
    const unsigned half = (SWITCH_CHATTER_MAX / 2u) + 6u;   // under the threshold alone

    // Quiet, then a burst that ends right at the first boundary.
    feed(sw, rawFor(0u), t, pollsPerBucket - half - 2u);
    alternate(sw, t, half);
    feed(sw, rawFor(0u), t, 3u);                  // roll happens in here
    CHECK(sw.chatter == 0u);                      // half a burst is not chatter

    // The rest of the same burst, immediately after the boundary.
    alternate(sw, t, half);
    feed(sw, rawFor(0u), t, pollsPerBucket);      // roll again

    // A single fixed bucket sees only the second half and reports nothing. Two
    // summed half-buckets see the whole burst, which is what a badly
    // intermittent connector actually looks like.
    CHECK((sw.chatter & 0x0001u) == 0x0001u);
}

static void test_chatter_clears_when_the_input_settles()
{
    begin("chatter: the mask follows the present, not the past");
    SwitchData sw;
    uint32_t t = 0u;
    qualifyAt(sw, 0u, t);

    const unsigned pollsPerBucket =
        static_cast<unsigned>(SWITCH_CHATTER_BUCKET_MS / SWITCH_POLL_MS);
    alternate(sw, t, pollsPerBucket + 2u);
    CHECK((sw.chatter & 0x0001u) == 0x0001u);

    // Two quiet buckets flush both halves of the window. A bit that latched
    // forever after one bad window would still be asserted long after the
    // connector was reseated, and would train whoever reads it to ignore it.
    feed(sw, rawFor(0u), t, pollsPerBucket * 2u + 4u);
    CHECK(sw.chatter == 0u);
}

// ═════════════════════════════════════════════════════════════════════════════

int main()
{
    printf("SwitchFunctions host tests\n");
    printf("  chain=%u bits  switches=%u  debounce=%u  qualify=%u  chatter>%u/%lums\n\n",
           static_cast<unsigned>(SWITCH_CHAIN_BITS),
           static_cast<unsigned>(SWITCH_COUNT),
           static_cast<unsigned>(SWITCH_DEBOUNCE_SAMPLES),
           static_cast<unsigned>(SWITCH_QUALIFY_READS),
           static_cast<unsigned>(SWITCH_CHATTER_MAX),
           static_cast<unsigned long>(SWITCH_CHATTER_WINDOW_MS));

    test_transport_bit_order();
    test_transport_msb_first();
    test_absent_chain_reads_all_ones();

    test_sentinel_polarity();
    test_rejected_read_leaves_state_alone();
    test_absence_needs_repeated_failures();

    test_no_state_published_before_qualification();
    test_unstable_sample_restarts_qualification();
    test_requalify_after_outage_reports_the_difference();
    test_requalify_unchanged_reports_nothing();

    test_commit_requires_full_run();
    test_transient_never_commits();
    test_one_bad_input_cannot_block_another();
    test_outage_destroys_debounce_evidence();
    test_changed_is_sticky_until_cleared();

    test_fast_chatter_is_detected();
    test_brisk_human_use_is_not_chatter();
    test_burst_across_a_bucket_boundary_is_caught();
    test_chatter_clears_when_the_input_settles();

    printf("\n%d checks, %d failures\n", g_checks, g_fails);
    return g_fails;
}
