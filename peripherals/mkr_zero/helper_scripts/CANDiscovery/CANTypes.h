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
    uint32_t firstMs;
    uint32_t lastMs;
    uint16_t minGapMs;
    uint16_t maxGapMs;
    uint8_t  dlc;
    uint8_t  orMask[8];   ///< running OR  of every payload byte
    uint8_t  andMask[8];  ///< running AND of every payload byte
    uint8_t  last8[8];
};

#endif // CAN_DISCOVERY_TYPES_H
