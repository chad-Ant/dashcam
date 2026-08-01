#ifndef CAN_DISCOVERY_TYPES_H
#define CAN_DISCOVERY_TYPES_H 1

/**
 * @file CANTypes.h
 * @brief Types for CANDiscovery.ino.
 *
 * These live in a header rather than in the .ino for a mechanical reason: the
 * Arduino build auto-generates function prototypes and inserts them near the
 * top of the sketch, ABOVE any type declared in the .ino body. A function
 * taking `RawFrame&` then gets a prototype referring to a type that does not
 * exist yet, and the build fails with "'RawFrame' has not been declared".
 * Types pulled in by #include are visible before that insertion point.
 */

#include <Arduino.h>

/** @brief One received CAN frame, as read straight out of an MCP2515 RX buffer. */
struct RawFrame {
    uint16_t id;        ///< 11-bit standard identifier.
    uint8_t  dlc;       ///< Payload length, clamped to 8.
    uint8_t  data[8];
    bool     extended;  ///< IDE bit: a 29-bit frame (unexpected on Honda F-CAN).
};

/**
 * @brief Per-ID statistics accumulated over one census window.
 *
 * @c orMask and @c andMask together locate signals: `orMask[i] & ~andMask[i]`
 * is exactly the set of bits in byte @c i that were not constant across the
 * window. Per-bit rather than per-byte, because an unaligned 15-bit field
 * (Honda's wheel speeds, for one) cannot be found from byte-granular flags.
 */
struct CanIdStat {
    uint16_t id;
    uint32_t count;
    /// Count at the previous report. The Hz column is (count - lastCount) over
    /// the interval since that report - an INSTANTANEOUS rate. Dividing the
    /// cumulative count by the total elapsed time instead reports a lifetime
    /// average, which for any signal whose rate changes is a number that was
    /// never true: a 48 Hz message that briefly floods reads as ~4000 Hz
    /// forever afterwards.
    uint32_t lastCount;
    uint32_t firstMs;
    uint32_t lastMs;
    uint16_t minGapMs;
    uint16_t maxGapMs;
    uint8_t  dlc;
    uint8_t  orMask[8];   ///< running OR  of every payload byte, whole run
    uint8_t  andMask[8];  ///< running AND of every payload byte, whole run
    /**
     * The same OR/AND pair, but reset after every report.
     *
     * The whole-run masks saturate and then stay saturated, which destroys the
     * one thing they exist for. Turning the ignition on moves bytes that a
     * parked car froze at 0xFF; from that moment those bytes read "changed"
     * forever, and a field that later sweeps during a steering or throttle test
     * is indistinguishable from one that moved once, an hour ago, for an
     * unrelated reason. The per-window pair answers "what is moving RIGHT NOW",
     * which is what differential identification actually needs - hold a state
     * for one report interval and only the bits belonging to it light up.
     */
    uint8_t  winOr[8];
    uint8_t  winAnd[8];
    uint8_t  last8[8];
};

#endif // CAN_DISCOVERY_TYPES_H
