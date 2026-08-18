#ifndef SWITCHFUNCTIONS_H
#define SWITCHFUNCTIONS_H 1

/**
 * @file SwitchFunctions.h
 * @brief Physical panel switches, read through a 74HC165 chain.
 *
 * ── THE SWITCHES HAVE NO MEANING HERE, ON PURPOSE ────────────────────────────
 * This module reports POSITIONS.  It does not know, and must never learn, what
 * any of them controls.  Inputs are numbered @c SW00 upward and nothing in this
 * firmware branches on one.
 *
 * That is a design constraint rather than an oversight, and it has a concrete
 * consequence worth stating: THE MASTER TAKES NO LOCAL ACTION ON ANY SWITCH.
 * If purpose is not known when this is compiled, there is no switch it could
 * correctly act upon.  The mapping from index to function lives in Jetson-side
 * configuration alongside the CAN map, so a switch can be re-purposed, or moved
 * to a different hole in the dash, without reflashing anything.
 *
 * If a genuine low-latency local binding is ever needed, the precedent already
 * exists — @c CMD_SET_CAN_FILTER pushes runtime configuration down from the
 * host — and that is the shape it should take.  Building it before there is a
 * switch that needs it would be inventing a requirement.
 *
 * ── WHY NOT THE SPI BUS ──────────────────────────────────────────────────────
 * The 74HC165 is an SPI-shaped device and it is tempting to hang it off D8-D10
 * for the price of one chip-select.  It cannot go there.  Its Q7 output is
 * push-pull with no tri-state and no select line that releases it, so it would
 * drive MISO permanently and fight the MCP2515 and the SD card — taking out CAN
 * and logging, not merely failing to read switches.  Three bit-banged pins of
 * its own cost less than the tri-state buffer that would be needed otherwise,
 * and an 80 us read against a 20 ms poll leaves nothing on the table.
 *
 * ── WHAT A READ CAN AND CANNOT TELL YOU ──────────────────────────────────────
 * Two of the sixteen inputs are hard-wired sentinels rather than switches (see
 * @c SWITCH_SENTINEL_BITS).  They are what makes @c SwitchData::present a
 * claim instead of an assumption, and they are checked BEFORE debouncing: a
 * read that fails the sentinel test is discarded whole and never reaches the
 * state machine, because a transaction that completed is not the same thing as
 * data that is real.  This module has an ancestor's mistakes to avoid — the
 * IMU that was replaced returned corrupt-but-plausible values from I2C
 * transactions that succeeded.
 *
 * What the sentinels cannot cover is a single broken switch wire, which reads
 * as "open", or a chassis short, which reads as "closed".  With latching
 * switches both are legitimate positions and no amount of software separates
 * them.  Contact chatter IS detectable and is reported in
 * @c SwitchData::chatter.  See @c SWITCH_CHATTER_MAX.
 *
 * ── WHAT THE DATASHEET SAYS THAT CHANGES THE DESIGN ──────────────────────────
 * Nexperia 74HC/HCT165 rev 8, in docs/.  Three facts are load-bearing here:
 *
 * §1: "Inputs are overvoltage tolerant to 15 V", explicitly for high-to-low
 * level shifting.  So the SWITCH LOOM'S HAZARDS STOP AT THE REGISTER — a switch
 * line shorted to 12 V does not damage it, and the MKR never sees the loom at
 * all, only Q7 driven at 3.3 V.  That makes this part a protection boundary
 * rather than only a multiplexer, and it is the strongest argument for reading
 * dash switches this way rather than into GPIO directly.  It does NOT cover
 * load dump or ESD past the rated 2 kV HBM, so a long loom still wants series
 * protection; it does cover the fault that actually happens.
 *
 * Table 4: I_O absolute maximum ±25 mA, with V_OL characterised at 4 mA.  The
 * SAMD21's internal pull-up on the data line draws 55-165 uA against that —
 * about 4 % of the characterisation current, worth roughly 6 mV of V_OL.  Q7 is
 * push-pull and wins by two orders of magnitude, which is what makes
 * @c INPUT_PULLUP on @c SWITCH_DATA_PIN free rather than a compromise.
 *
 * Table 5: an input transition rate limit of 139 ns/V at 4.5 V — these are NOT
 * Schmitt inputs.  A 10 kOhm pull-up charging harness capacitance is far slower
 * than that, so the D0-D7 loom inputs run outside the specified slew.  Stated
 * rather than fixed: those pins are LEVEL-sampled during the load pulse, not
 * edge-triggered, so the cost is crowbar current for a few microseconds on each
 * switch release — a handful of times a day, into a package with 100 nF across
 * it.  CP and PL, where a slow edge WOULD matter, are driven directly by the
 * SAMD21 over a short trace.
 */

#include <Arduino.h>

#include "DataDictionary.h"

static_assert(SWITCH_REGISTERS >= 1U && SWITCH_REGISTERS <= 2U,
              "SWITCH_REGISTERS must be 1 or 2 - the chain is read into a uint16_t");
static_assert(SWITCH_CHAIN_BITS <= 16U,
              "SWITCH_CHAIN_BITS exceeds the width of SwitchData::state");
static_assert(SWITCH_DEBOUNCE_SAMPLES >= 1U,
              "A debounce of zero samples commits every glitch");

/**
 * @brief Switch panel state, published and working, in one place.
 *
 * Owned by the caller and passed to every call, matching @c GPSInitState and
 * @c BNO055InitState.  Nothing here is static, so a second chain — or a test
 * harness driving a synthetic one — costs no changes.
 */
struct SwitchData {
    // ── published ────────────────────────────────────────────────────────────

    /**
     * Debounced positions.  Bit @e n is @c SW@e n, and 1 means CLOSED.
     *
     * The hardware reads the other way round: pull-ups to 3V3 with switches to
     * ground, so a closed switch pulls its input LOW.  The inversion happens
     * once, here at the boundary, because every consumer upward expects
     * "asserted = 1" and a sign convention that leaks past this line is a sign
     * convention that eventually gets applied twice.
     */
    uint16_t state;

    /**
     * Sticky: bits that changed at least once since @c switchClearChanged().
     *
     * A latching switch holds its own position, so @c state alone can never
     * miss one at a 10 Hz publish rate — but a switch flipped and returned
     * BETWEEN two published frames is invisible in level alone, and frames are
     * genuinely lost to decimation and to a full host TX ring.  This is the
     * same latch-and-clear-on-confirmed-send discipline the High-G flag and the
     * coalescing accumulator already use, for the same reason.
     *
     * Also set when a chain that has been away comes back holding a DIFFERENT
     * position from the one last published.  Something moved while nobody was
     * looking, and the host is entitled to know that even though the moment it
     * happened is unrecoverable.  Not set by the FIRST qualification after
     * boot, where there is no previously published position to differ from.
     *
     * ⚠️ UNTIL PROTOCOL v0x07 THIS MEANS "SINCE BOOT".  @c switchClearChanged()
     * has no caller yet, because nothing transmits these fields — the wire
     * format is a four-header lockstep edit deferred to the same bump that
     * carries the outstanding IMU fields.  The word is correct and monotonic in
     * the meantime; it simply is not yet an interval.
     */
    uint16_t changed;

    /**
     * VALIDITY MASK.  A set bit means the matching @c state bit is not to be
     * trusted, not merely that the input is busy.
     *
     * Deliberately not enforced by freezing the offending bit.  A fast chatterer
     * never satisfies the debounce integrator, so its published position stays
     * put on its own, and adding a freeze would couple the detector to the
     * committer for a case the committer already handles.  A slow chatterer does
     * commit, and this word is how a consumer knows to disregard it.
     */
    uint16_t chatter;

    /**
     * True once a stable baseline has been established and is still being
     * confirmed by the sentinels.
     *
     * False means the chain is absent, unpowered, miswired, its cascade link is
     * broken, or it is present but has not yet held still for
     * @c SWITCH_QUALIFY_READS reads — and, critically, that @c state is STALE
     * rather than wrong.  The last good positions are held rather than zeroed,
     * because zeroing would publish "every switch is open" as though it had
     * been measured.
     */
    bool present;

    // ── diagnostics ──────────────────────────────────────────────────────────

    uint32_t reads;        ///< Reads attempted since init.
    uint32_t rejected;     ///< Reads discarded on the sentinel check.
    uint16_t lastRaw;      ///< Last raw word, sentinels included. For bring-up.

    // ── working state — not for consumers ────────────────────────────────────

    uint16_t lastSampled;      ///< Previous ACCEPTED sample. Feeds the edge count.
    bool     haveSample;       ///< @c lastSampled holds something.
    bool     everQualified;    ///< A baseline has been published at least once.
    uint8_t  qualify;          ///< Identical accepted reads while qualifying.
    uint8_t  agree[16];        ///< Consecutive samples disagreeing with @c state.
    uint8_t  edges[16];        ///< Raw input edges in the current half-window.
    uint8_t  edgesPrev[16];    ///< The half-window before it. Summed with above.
    uint8_t  absentReads;      ///< Consecutive sentinel failures.
    uint32_t bucketStartMs;    ///< Start of the current half-window.
};

/**
 * @brief Configures the three pins.  Does NOT publish a state.
 *
 * @c present is false on return even with a healthy chain: the baseline is
 * established by the first @c SWITCH_QUALIFY_READS polls, which takes about
 * 80 ms and costs nothing, because there is no consumer inside that window
 * anyway.
 *
 * That is a deliberate change from priming off a single read here.  One
 * un-debounced sample was being adopted as truth, so a read landing mid-throw
 * became the published position and was then "corrected" 80 ms later by a
 * @c changed bit describing nothing an operator had done.  Qualifying instead
 * of priming also removes the asymmetry where boot trusted one sample while
 * every later change needed four.
 *
 * Does not block and cannot fail in a way worth reporting.
 */
void initializeSwitches(SwitchData &sw);

/**
 * @brief Reads the chain once and folds the result into @p sw.
 *
 * Call at @c SWITCH_POLL_MS.  The debounce window is counted in POLLS, not
 * milliseconds, so calling at a different rate silently changes it.
 *
 * Blocking for roughly 80 us, which is two orders of magnitude below the IMU's
 * I2C burst on the same loop — worth stating only because "bit-banged" tends to
 * read as "slow".
 *
 * @return true if the read passed the sentinel check and was used.  A false
 *         return is not an error to escalate: it is the normal report of a
 *         chain that is not fitted, and the caller's own logging of
 *         @c SwitchData::present is the right place to notice.
 */
bool pollSwitches(SwitchData &sw);

/**
 * @brief Clears the sticky @c changed word.
 *
 * ONLY AFTER A CONFIRMED SEND.  Clearing it on the attempt discards the event
 * along with the frame that failed to carry it, after which the next frame
 * reports a quiet interval over a window in which a switch moved.  That exact
 * bug was found and fixed in the bridge's coalescing path; this is the same
 * shape and the fix is the same.
 */
void switchClearChanged(SwitchData &sw);

/**
 * @brief One raw read, sentinels included, bypassing debounce entirely.
 *
 * For bring-up only — @c helper_scripts/SwitchProbe uses it to show wiring as
 * it actually is.  Production code wants @c pollSwitches().
 */
uint16_t switchReadRaw();

// ─── the pure core ───────────────────────────────────────────────────────────
//
// Everything that DECIDES anything lives below, taking the raw word and the
// clock as arguments rather than reading them from the world. @c pollSwitches()
// and @c initializeSwitches() are the thin hardware wrappers over these two.
//
// Split out to make the logic testable, and worth doing on its own merits: the
// sentinel gate, the qualification counter, the debounce integrator and the
// sliding chatter window are pure functions of a sample sequence, and they were
// only ever reachable through a GPIO read and an ambient millis(). Nothing
// about them needed to be.
//
// The seam sits exactly where the review found the gap. SwitchProbe already
// exercises the shift loop against real silicon; what had no coverage at all is
// the state machine, and that is precisely what these two expose.

/**
 * @brief Resets @p sw to its just-initialised condition. Touches no pins.
 *
 * @param nowMs Value the chatter window should start from.
 */
void switchResetState(SwitchData &sw, uint32_t nowMs);

/**
 * @brief Folds one raw word into @p sw at time @p nowMs.
 *
 * @param raw   The chain word exactly as shifted in, sentinels included.
 * @param nowMs Monotonic milliseconds. Must not go backwards.
 * @return Same contract as @c pollSwitches(): true if the word passed the
 *         sentinel check and was used.
 */
bool switchIngest(SwitchData &sw, uint16_t raw, uint32_t nowMs);

#endif // SWITCHFUNCTIONS_H
