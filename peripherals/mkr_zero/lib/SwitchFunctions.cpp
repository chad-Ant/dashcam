#include "SwitchFunctions.h"

#include <string.h>

// ─── sentinel contract ───────────────────────────────────────────────────────
//
// The read below shifts MSB-first from the far end of the chain, so the LAST
// two bits to arrive are #A's D0 and D1 — the two hard-wired inputs. They land
// in raw bits 0 and 1 for a one-register chain and a two-register chain alike,
// which is why SWITCH_REGISTERS can change without touching any of this.
//
//   raw bit 0  =  #A D0  =  74HC165 pin 11  ->  GND   (must read 0)
//   raw bit 1  =  #A D1  =  74HC165 pin 12  ->  3V3   (must read 1)
//
// Switches therefore occupy raw bits 2 upward, and SW00 is raw bit 2.

static constexpr uint16_t SENTINEL_MASK     = 0x0003u;
static constexpr uint16_t SENTINEL_EXPECTED = 0x0002u;  ///< bit1 high, bit0 low.

/// Mask of the bits SwitchData::state actually carries.
static constexpr uint16_t STATE_MASK =
    static_cast<uint16_t>((1UL << SWITCH_COUNT) - 1UL);

static_assert(SWITCH_QUALIFY_READS <= 250U,
              "SWITCH_QUALIFY_READS must fit the uint8 counter with headroom");

// ─── raw transport ───────────────────────────────────────────────────────────

uint16_t switchReadRaw()
{
    // Parallel load. Every input is captured on this pulse, so all switches are
    // sampled at one instant rather than smeared across the shift that follows.
    //
    // The settle delay after PL rises is not decoration: the datasheet gives a
    // recovery time PL-to-CP (t_rec, 20 ns at 4.5 V) during which a clock edge
    // must not arrive. A SAMD21 digitalWrite already costs more than that, so
    // the delay is margin on top of margin — but the requirement is real and
    // removing the delay would leave nothing documenting it.
    digitalWrite(SWITCH_LOAD_PIN, LOW);
    delayMicroseconds(SWITCH_SETTLE_US);
    digitalWrite(SWITCH_LOAD_PIN, HIGH);
    delayMicroseconds(SWITCH_SETTLE_US);

    uint16_t raw = 0u;

    // READ BEFORE CLOCKING, every time.
    //
    // After the load pulse Q7 is ALREADY presenting D7 — the first bit is
    // available with no clock edge at all. Clocking first would discard it and
    // shift every remaining bit one position, which reads as a wiring fault and
    // is not one. Bound by construction: SWITCH_CHAIN_BITS is a compile-time
    // constant and the loop has no early exit.
    for (uint8_t i = 0u; i < SWITCH_CHAIN_BITS; ++i) {
        raw = static_cast<uint16_t>(raw << 1);
        if (digitalRead(SWITCH_DATA_PIN) == HIGH) {
            raw = static_cast<uint16_t>(raw | 1u);
        }
        digitalWrite(SWITCH_CLK_PIN, HIGH);
        digitalWrite(SWITCH_CLK_PIN, LOW);
    }

    return raw;
}

/**
 * @brief Sentinel test. The whole of this module's trust rests on it.
 */
static inline bool sentinelsGood(uint16_t raw)
{
    return (raw & SENTINEL_MASK) == SENTINEL_EXPECTED;
}

/**
 * @brief Raw word to switch positions: drop the sentinels, invert the sense.
 *
 * Pull-ups mean a CLOSED switch reads low, and every consumer upward wants
 * "asserted = 1". The inversion happens exactly once, here.
 */
static inline uint16_t rawToState(uint16_t raw)
{
    const uint16_t inverted = static_cast<uint16_t>(~raw);
    return static_cast<uint16_t>((inverted >> SWITCH_SENTINEL_BITS) & STATE_MASK);
}

// ─── lifecycle ───────────────────────────────────────────────────────────────

void switchResetState(SwitchData &sw, uint32_t nowMs)
{
    memset(&sw, 0, sizeof(sw));
    sw.bucketStartMs = nowMs;

    // NO PRIMING READ. present stays false until switchIngest() has seen
    // SWITCH_QUALIFY_READS identical words — see the header for why one
    // un-debounced sample was the wrong thing to publish.
}

void initializeSwitches(SwitchData &sw)
{
    // Order matters. The clock is driven low and the load line high — both
    // inactive — BEFORE either is configured as an output, so configuring them
    // cannot emit a spurious edge into a chain that is already powered. On the
    // SAMD21 digitalWrite() on a pin still configured as an input sets the
    // output latch as well as the pull, so the subsequent pinMode(OUTPUT),
    // which only touches DIRSET, drives the value already latched. I2CBus.cpp
    // documents the same ordering for the same reason.
    digitalWrite(SWITCH_LOAD_PIN, HIGH);
    digitalWrite(SWITCH_CLK_PIN, LOW);
    pinMode(SWITCH_LOAD_PIN, OUTPUT);
    pinMode(SWITCH_CLK_PIN, OUTPUT);
    digitalWrite(SWITCH_LOAD_PIN, HIGH);
    digitalWrite(SWITCH_CLK_PIN, LOW);

    // INPUT_PULLUP, and the reason is the opposite of what this comment used to
    // claim. It said a pull-up would mask an absent chain by holding the line at
    // 0xFFFF — which is exactly backwards: 0xFFFF fails the sentinel test,
    // because bit 0 must read LOW and a pull-up holds it HIGH. So a pull-up
    // makes absence DETERMINISTIC. Leaving the pin floating is what was unsafe:
    // a floating input picks up clock coupling from the pin next to it, and
    // random-looking data satisfies a two-bit pattern roughly one read in four.
    // An absence detector that passes a quarter of the time is not a detector.
    //
    // Free, per the datasheet: Q7 is push-pull with an absolute maximum output
    // current of +/-25 mA and V_OL characterised at 4 mA, against 55-165 uA from
    // the SAMD21's internal pull-up. About 6 mV of V_OL, and the register wins
    // by two orders of magnitude.
    pinMode(SWITCH_DATA_PIN, INPUT_PULLUP);

    switchResetState(sw, millis());
}

// ─── chatter ─────────────────────────────────────────────────────────────────

/**
 * @brief Rolls the sliding chatter window and republishes the mask.
 *
 * Two half-buckets summed, rather than one bucket reset on a boundary. A single
 * bucket cannot see a burst that straddles its edge — twenty-five edges either
 * side of a reset is a badly intermittent connector reported as two quiet
 * windows, and bursts around a boundary are not rare.
 *
 * Runs whether or not the read that follows is usable, so a chain that goes
 * away mid-window does not leave stale counts to be added to whenever it
 * returns and reported as a burst that never happened.
 */
static void serviceChatterBuckets(SwitchData &sw, uint32_t nowMs)
{
    if ((nowMs - sw.bucketStartMs) < SWITCH_CHATTER_BUCKET_MS) return;

    uint16_t flagged = 0u;
    for (uint8_t i = 0u; i < SWITCH_COUNT; ++i) {
        // Widened before the add: two saturated uint8 buckets sum to 510.
        const uint16_t total =
            static_cast<uint16_t>(static_cast<uint16_t>(sw.edges[i]) + sw.edgesPrev[i]);
        if (total > SWITCH_CHATTER_MAX) {
            flagged = static_cast<uint16_t>(flagged | (1u << i));
        }
        sw.edgesPrev[i] = sw.edges[i];
        sw.edges[i]     = 0u;
    }

    // Replaced, not accumulated. This word answers "is this input misbehaving
    // NOW" — a bit that latched forever after one bad window would still be
    // asserted long after the connector was reseated, and would train whoever
    // reads it to ignore the field.
    sw.chatter       = flagged;
    sw.bucketStartMs = nowMs;
}

/** @brief Adds this read's raw edges to the current bucket, saturating. */
static void countRawEdges(SwitchData &sw, uint16_t rawEdges)
{
    if (rawEdges == 0u) return;

    for (uint8_t i = 0u; i < SWITCH_COUNT; ++i) {
        if ((rawEdges & static_cast<uint16_t>(1u << i)) == 0u) continue;
        // Saturating. A wrapped uint8 would report a violently chattering
        // input as quiet, which is the one case this counter exists for.
        if (sw.edges[i] < 255u) ++sw.edges[i];
    }
}

// ─── poll ────────────────────────────────────────────────────────────────────

bool pollSwitches(SwitchData &sw)
{
    // The whole of the hardware dependency, in one line. Everything below the
    // seam is a pure function of the word and the clock.
    return switchIngest(sw, switchReadRaw(), millis());
}

bool switchIngest(SwitchData &sw, uint16_t raw, uint32_t nowMs)
{
    serviceChatterBuckets(sw, nowMs);

    ++sw.reads;
    sw.lastRaw = raw;

    // THE GATE. A read that fails its own positive control is discarded whole —
    // it never touches state, changed, or the debounce counters. Feeding it in
    // "just in case some bits are good" is how a bus fault becomes a phantom
    // switch position, and this project has already paid for that lesson once
    // with an IMU that returned plausible numbers from successful transactions.
    if (!sentinelsGood(raw)) {
        ++sw.rejected;

        // THE DEBOUNCE EVIDENCE IS DESTROYED, NOT SUSPENDED.
        //
        // Leaving it standing broke the one guarantee the integrator makes.
        // Three disagreeing samples, then a ten-minute outage, then a single
        // valid sample used to reach the threshold and commit — four samples
        // that agreed, but not four CONSECUTIVE ones, and separated by an
        // interval in which the switch could have been moved twice. Evidence
        // gathered before an outage says nothing about the state after it.
        memset(sw.agree, 0, sizeof(sw.agree));
        sw.qualify    = 0u;
        sw.haveSample = false;   // no continuity across the gap, so no edge

        if (sw.absentReads < SWITCH_ABSENT_READS) {
            ++sw.absentReads;
        }
        if (sw.absentReads >= SWITCH_ABSENT_READS) {
            // state is deliberately LEFT ALONE. Stale positions that are
            // labelled stale beat zeroed positions that look measured.
            sw.present = false;
        }
        return false;
    }

    sw.absentReads = 0u;

    const uint16_t sampled  = rawToState(raw);
    const bool     firstOne = !sw.haveSample;
    const uint16_t rawEdges =
        firstOne ? 0u : static_cast<uint16_t>(sampled ^ sw.lastSampled);

    // COUNTED ON RAW SAMPLES, BEFORE AND INDEPENDENTLY OF DEBOUNCE.
    //
    // This is the whole correction to the chatter detector. Counting committed
    // state changes instead made it blind to the fault it exists for: an input
    // alternating every poll resets the integrator on every pass, so it never
    // commits, so it reported zero transitions while flapping at 50 Hz. Only
    // slow chatter was counted — and slow chatter also describes a person
    // working a switch briskly. Exactly the wrong way round.
    countRawEdges(sw, rawEdges);

    sw.lastSampled = sampled;
    sw.haveSample  = true;

    // ── qualification: at boot, and after every outage ───────────────────────
    if (!sw.present) {
        // Consecutive IDENTICAL accepted reads, not merely consecutive good
        // ones: the point is a baseline that has held still, and a read taken
        // the instant a connector reseats is the least trustworthy one there is.
        if (firstOne || rawEdges != 0u) sw.qualify = 1u;
        else                            ++sw.qualify;

        if (sw.qualify < SWITCH_QUALIFY_READS) return true;

        // Something moved while nobody was looking. The moment is unrecoverable
        // but the fact is not, and a host that resumes to a different panel is
        // entitled to know it changed rather than having to diff two frames
        // that may be minutes apart. Suppressed on the FIRST qualification,
        // where there is no previously published position to differ from.
        if (sw.everQualified) {
            sw.changed = static_cast<uint16_t>(sw.changed | (sampled ^ sw.state));
        }

        sw.state         = sampled;
        sw.present       = true;
        sw.everQualified = true;
        memset(sw.agree, 0, sizeof(sw.agree));
        return true;
    }

    // ── steady state ─────────────────────────────────────────────────────────
    const uint16_t differs = static_cast<uint16_t>(sampled ^ sw.state);

    // Per-input integrator, not a whole-word one. A single chattering input
    // must not be able to freeze the other thirteen — which is exactly what a
    // shared "N identical reads in a row" counter does, and the failing input
    // is the one most likely to be chattering.
    for (uint8_t i = 0u; i < SWITCH_COUNT; ++i) {
        const uint16_t bit = static_cast<uint16_t>(1u << i);

        if ((differs & bit) == 0u) {
            sw.agree[i] = 0u;
            continue;
        }

        if (++sw.agree[i] < SWITCH_DEBOUNCE_SAMPLES) continue;

        sw.agree[i] = 0u;
        sw.state    = static_cast<uint16_t>(sw.state ^ bit);
        sw.changed  = static_cast<uint16_t>(sw.changed | bit);
    }

    return true;
}

void switchClearChanged(SwitchData &sw)
{
    sw.changed = 0u;
}
