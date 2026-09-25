/**
 * @file can_probe_tests.cpp
 * @brief Deterministic tests for the boot-time map probe in
 *        lib/CANSniffFunctions.cpp.
 *
 * CANSniffFunctions.cpp and CANMap.cpp compile UNMODIFIED, against an MCP2515
 * model (mcp2515_model.cpp) that receives frames from the BUS side and runs
 * them through the acceptance filters the driver actually programmed. That is
 * the whole point: the defect here was invisible to anything that handed the
 * driver frames directly, because the frames it never saw were the ones the
 * controller's own filters threw away.
 *
 * The map is the compiled-in Honda one (canSniffSetMap(nullptr)): six IDs,
 * 0x158 0x17C 0x191 0x1AB 0x1D0 0x294, all of which fit the six hardware
 * filter slots.
 *
 * Exit status is the number of failures, so `make check` fails the build.
 */

#include "CANSniffFunctions.h"
#include "DataDictionary.h"
#include "SDFunctions.h"
#include "mcp2515_model.h"

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

// ─── no card on the host ─────────────────────────────────────────────────────
//
// CANMap.cpp shares its file with the SD loader. Nothing here loads a map from
// a card, so the loader's four calls just report the card absent.

bool sdReady()                             { return false; }
bool sdOpenRoot(File32 &)                  { return false; }
bool sdOpenRead(const char *, File32 &)    { return false; }
bool sdReadLine(File32 &, char *, size_t)  { return false; }

// ─── helpers ─────────────────────────────────────────────────────────────────

static const uint8_t MODE_LISTEN_ONLY = 0x60;
static const uint8_t MODE_CONFIG      = 0x80;

static const uint16_t kMapIds[]     = { 0x158, 0x17C, 0x191, 0x1AB, 0x1D0, 0x294 };
/// Another vehicle's traffic: none of these is in the map.
static const uint16_t kForeignIds[] = { 0x100, 0x13A, 0x1A0, 0x200, 0x2E4, 0x305, 0x3FF };

static VehicleSignals g_v;
static YawEstimator   g_y;
static CanProbeState  g_p;
static uint32_t       g_t = 0;
static unsigned       g_next;   ///< Round-robin position in the bus's ID list.

static void advance(uint32_t ms)
{
    g_t += ms;
    hostSetMillis(g_t);
}

static void begin(const char *name)
{
    g_case = name;
    hostReset();
    fakeCanReset();
    // The clock only ever moves FORWARD, across tests too. The driver keeps its
    // own static state between them — the mode and the rate limiter's last
    // transition among it — and a clock restarted per test would put "now"
    // before that transition, which no real board can do.
    g_t   += 100000u;
    g_next = 0;
    hostSetMillis(g_t);
    g_p.stage        = CanProbeStage::Idle;
    g_p.startMs      = 0;
    g_p.clockStarted = false;
}

/// Into SNIFF from a known OFF, the way the sketch's boot path gets there.
static bool enterSniff()
{
    canSniffSetMap(nullptr);
    // Each transition clear of the driver's rate limit, as the sketch's are.
    advance(CAN_MODE_MIN_INTERVAL_MS + 100u);
    const CanModeStatus off = canSetMode(CanMode::OFF, MCP2515_DEFAULT_CS_PIN);
    advance(CAN_MODE_MIN_INTERVAL_MS + 100u);
    const bool ok = (off == CanModeStatus::OK || off == CanModeStatus::UNCHANGED) &&
                    (canSetMode(CanMode::SNIFF, MCP2515_DEFAULT_CS_PIN) == CanModeStatus::OK);
    initVehicleSignals(g_v);
    initYawEstimator(g_y, canSniffGetMap());
    return ok;
}

/**
 * @brief Runs loop() passes against a bus carrying @p ids round-robin.
 *
 * Two frames arrive per 10 ms pass — one per receive buffer — then the drain and
 * the probe tick run, in the sketch's order. Stops at the first verdict.
 *
 * @param verdictAtMs  Set to the clock at the verdict, when there is one.
 */
static CanProbeStage run(const uint16_t *ids, unsigned n, uint32_t forMs,
                         uint32_t stepMs = 10u, uint32_t *verdictAtMs = nullptr)
{
    CanProbeStage st = g_p.stage;
    for (uint32_t el = 0; el < forMs; el += stepMs) {
        advance(stepMs);
        for (unsigned k = 0; k < 2u && n > 0u; ++k) (void)fakeCanFrame(ids[g_next++ % n]);
        (void)tickCANSniff(g_v, g_y);
        st = canProbeTick(g_p, g_t);
        if (st != CanProbeStage::Probing) {
            if (verdictAtMs != nullptr) *verdictAtMs = g_t;
            break;
        }
    }
    return st;
}

static const unsigned kMapN     = sizeof(kMapIds) / sizeof(kMapIds[0]);
static const unsigned kForeignN = sizeof(kForeignIds) / sizeof(kForeignIds[0]);

// ═════════════════════════════════════════════════════════════════════════════
// the probe must see the bus, not just the map
// ═════════════════════════════════════════════════════════════════════════════

static void test_wrong_map_on_busy_bus_falls_back()
{
    // REGRESSION. Entering SNIFF programs the map's filters, and the probe's
    // window only opens on the first frame. On a busy bus carrying none of the
    // map's IDs the filters admitted nothing, the frame count stayed at zero,
    // and the probe sat in Probing forever — no fallback to OBD2, ever.
    begin("probe: a map for another vehicle on a busy bus falls back");

    CHECK(enterSniff());
    CHECK(canProbeArm(g_p));
    CHECK(canSniffFilterCount() == 0u);          // accept-all while unproven
    CHECK(!canSniffFiltersFromMap());
    CHECK(fakeCanAccepts(0x100));
    CHECK(fakeCanOpMode() == MODE_LISTEN_ONLY);  // opened, and still silent

    const uint32_t armedAt = g_t;
    uint32_t verdictAt = 0;
    const CanProbeStage st = run(kForeignIds, kForeignN, 60000u, 10u, &verdictAt);
    CHECK(st == CanProbeStage::FellBack);
    CHECK(canSniffFrameCount() > 0u);
    CHECK(canSniffMatchCount() == 0u);
    // The window runs from the first frame, which arrived on the first pass.
    CHECK(verdictAt != 0u && (verdictAt - armedAt) <= CAN_PROBE_WINDOW_MS + 100u);
    CHECK(!fakeCanWasEverNormal());              // the probe itself never transmits
}

static void test_right_map_passes_then_narrows()
{
    begin("probe: the right map passes, then its filters go in");

    CHECK(enterSniff());
    CHECK(canProbeArm(g_p));

    // The map's IDs mixed into other traffic, as on the real bus.
    const uint16_t bus[] = { 0x158, 0x100, 0x17C, 0x191, 0x200, 0x1AB, 0x1D0, 0x3FF, 0x294 };
    const CanProbeStage st = run(bus, sizeof(bus) / sizeof(bus[0]), 60000u);
    CHECK(st == CanProbeStage::Sniffing);
    CHECK(canSniffIdsSeen() >= canProbeIdsNeeded());

    CHECK(canSniffApplyMapFilters());
    CHECK(canSniffFiltersFromMap());
    CHECK(canSniffFilterCount() == kMapN);
    for (unsigned i = 0; i < kMapN; ++i) CHECK(fakeCanAccepts(kMapIds[i]));
    for (unsigned i = 0; i < kForeignN; ++i) CHECK(!fakeCanAccepts(kForeignIds[i]));
    CHECK(canGetMode() == CanMode::SNIFF);
    CHECK(fakeCanOpMode() == MODE_LISTEN_ONLY);
    CHECK(!fakeCanWasEverNormal());
}

static void test_silent_bus_waits()
{
    // The property the first-frame clock exists for, which opening the filters
    // must not cost: a bus that says nothing is not evidence against the map.
    // An ignition-switched rig can sit on a sleeping bus for a long time.
    begin("probe: a silent bus is waited out, not judged");

    CHECK(enterSniff());
    CHECK(canProbeArm(g_p));
    const CanProbeStage st = run(nullptr, 0u, 3600u * 1000u, 1000u);
    CHECK(st == CanProbeStage::Probing);
    CHECK(!g_p.clockStarted);
    CHECK(canSniffFrameCount() == 0u);
}

// ═════════════════════════════════════════════════════════════════════════════
// the host outranks the heuristic
// ═════════════════════════════════════════════════════════════════════════════

static void test_host_filters_survive_the_verdict()
{
    begin("probe: a host filter set installed mid-probe is not overwritten");

    CHECK(enterSniff());
    CHECK(canProbeArm(g_p));

    const uint16_t host[] = { 0x158, 0x17C, 0x191 };
    CHECK(canSniffSetFilters(host, 3u));

    const CanProbeStage st = run(kMapIds, kMapN, 60000u);
    CHECK(st == CanProbeStage::Sniffing);
    CHECK(canSniffApplyMapFilters());
    CHECK(canSniffFilterCount() == 3u);
    CHECK(!canSniffFiltersFromMap());
    CHECK(!fakeCanAccepts(0x294));   // the host left it out, and it stays out
}

static void test_rearm_forgets_the_last_host_filters()
{
    // The protection belongs to one sniff session. A mode change reprograms the
    // filters, so an old host set no longer exists to protect.
    begin("probe: after a mode change the verdict installs the map again");

    CHECK(enterSniff());
    CHECK(canProbeArm(g_p));
    const uint16_t host[] = { 0x158, 0x17C, 0x191 };
    CHECK(canSniffSetFilters(host, 3u));

    CHECK(enterSniff());   // OFF, then SNIFF again
    CHECK(canProbeArm(g_p));
    CHECK(run(kMapIds, kMapN, 60000u) == CanProbeStage::Sniffing);
    CHECK(canSniffApplyMapFilters());
    CHECK(canSniffFiltersFromMap());
    CHECK(canSniffFilterCount() == kMapN);
}

static void test_skipping_a_running_probe_closes_its_filters()
{
    // A host CMD_SET_CAN_MODE skips the probe, and asking for SNIFF while
    // already sniffing is UNCHANGED — which reprograms nothing. Without the
    // skip closing them, the probe's accept-all filters would stay for good.
    begin("probe: skipping a running probe installs the map's filters");

    CHECK(enterSniff());
    CHECK(canProbeArm(g_p));
    CHECK(canSniffFilterCount() == 0u);

    canProbeSkip(g_p);                      // what the sketch's host handler does
    CHECK(g_p.stage == CanProbeStage::Skipped);
    CHECK(canSetMode(CanMode::SNIFF, MCP2515_DEFAULT_CS_PIN) == CanModeStatus::UNCHANGED);
    CHECK(canSniffFiltersFromMap());
    CHECK(canSniffFilterCount() == kMapN);
    CHECK(!fakeCanAccepts(0x100));
    CHECK(fakeCanOpMode() == MODE_LISTEN_ONLY);

    // Skipping again, with no probe running, writes nothing.
    fakeCanRefuseFilterWrites(true);        // any write would now park it
    canProbeSkip(g_p);
    fakeCanRefuseFilterWrites(false);
    CHECK(canGetMode() == CanMode::SNIFF);
}

// ═════════════════════════════════════════════════════════════════════════════
// refused filter writes park the controller in a state that is true
// ═════════════════════════════════════════════════════════════════════════════

static void test_refused_open_parks_off()
{
    begin("probe: a refused open parks the controller and reports OFF");

    CHECK(enterSniff());
    fakeCanRefuseFilterWrites(true);
    CHECK(!canProbeArm(g_p));
    fakeCanRefuseFilterWrites(false);

    CHECK(g_p.stage == CanProbeStage::Skipped);
    CHECK(canGetMode() == CanMode::OFF);
    CHECK(fakeCanOpMode() == MODE_CONFIG);
    CHECK(!fakeCanWasEverNormal());
}

static void test_refused_narrowing_parks_off()
{
    begin("probe: refused map filters after a pass park the controller");

    CHECK(enterSniff());
    CHECK(canProbeArm(g_p));
    CHECK(run(kMapIds, kMapN, 60000u) == CanProbeStage::Sniffing);

    fakeCanRefuseFilterWrites(true);
    CHECK(!canSniffApplyMapFilters());
    fakeCanRefuseFilterWrites(false);

    CHECK(canGetMode() == CanMode::OFF);
    CHECK(fakeCanOpMode() == MODE_CONFIG);
}

static void test_refused_host_filters_park_off()
{
    // The host's own filter write goes through Configuration like the probe's,
    // so its refusal must park the controller too, rather than leave it
    // receiving nothing while every flag still says SNIFF.
    begin("host: a refused host filter set parks the controller and reports OFF");

    CHECK(enterSniff());
    const uint16_t host[] = { 0x158 };
    fakeCanRefuseFilterWrites(true);
    CHECK(!canSniffSetFilters(host, 1u));
    fakeCanRefuseFilterWrites(false);
    CHECK(canGetMode() == CanMode::OFF);
    CHECK(fakeCanOpMode() == MODE_CONFIG);
}

static void test_host_filter_refused_outside_sniff_changes_nothing()
{
    begin("host: a filter set outside SNIFF is refused without touching the controller");

    canSniffSetMap(nullptr);
    advance(CAN_MODE_MIN_INTERVAL_MS + 100u);
    const CanModeStatus off = canSetMode(CanMode::OFF, MCP2515_DEFAULT_CS_PIN);
    CHECK(off == CanModeStatus::OK || off == CanModeStatus::UNCHANGED);
    const uint16_t host[] = { 0x158 };
    CHECK(!canSniffSetFilters(host, 1u));
    CHECK(canGetMode() == CanMode::OFF);
}

static void test_arm_outside_sniff_is_refused()
{
    begin("probe: arming outside SNIFF is refused and touches nothing");

    canSniffSetMap(nullptr);
    advance(CAN_MODE_MIN_INTERVAL_MS + 100u);
    const CanModeStatus off = canSetMode(CanMode::OFF, MCP2515_DEFAULT_CS_PIN);
    CHECK(off == CanModeStatus::OK || off == CanModeStatus::UNCHANGED);
    CHECK(!canProbeArm(g_p));
    CHECK(g_p.stage == CanProbeStage::Skipped);
    CHECK(canGetMode() == CanMode::OFF);
}

// ─── runner ──────────────────────────────────────────────────────────────────

int main()
{
    test_wrong_map_on_busy_bus_falls_back();
    test_right_map_passes_then_narrows();
    test_silent_bus_waits();

    test_host_filters_survive_the_verdict();
    test_rearm_forgets_the_last_host_filters();
    test_skipping_a_running_probe_closes_its_filters();

    test_refused_open_parks_off();
    test_refused_narrowing_parks_off();
    test_arm_outside_sniff_is_refused();
    test_refused_host_filters_park_off();
    test_host_filter_refused_outside_sniff_changes_nothing();

    printf("can_probe_tests: %d checks, %d failed\n", g_checks, g_fails);
    return g_fails;
}
