#ifndef OBD2_PROBE_TYPES_H
#define OBD2_PROBE_TYPES_H 1

/**
 * @file CANTypes.h
 * @brief Types for OBD2Probe.ino.
 *
 * In a header rather than the .ino because the Arduino build auto-generates
 * function prototypes and inserts them ABOVE anything declared in the sketch
 * body; a function taking `RawFrame&` would then be prototyped against a type
 * that does not exist yet. Types reached by #include are visible in time.
 */

#include <Arduino.h>

/** @brief One received CAN frame, read straight out of an MCP2515 RX buffer. */
struct RawFrame {
    uint16_t id;
    uint8_t  dlc;
    uint8_t  data[8];
    bool     extended;
};

/**
 * @brief Outcome of one bounded transmit attempt.
 *
 * @c TX_NEVER_STARTED is the diagnostically valuable one: TXREQ stayed set with
 * no error flag, meaning the controller never even began sending because the
 * bus never went idle. That is a stuck-dominant bus - shorted CANH/CANL, or far
 * more commonly an unpowered transceiver whose RXD output sits low.
 */
enum TxResult {
    TX_OK = 0,          ///< Sent and acknowledged by at least one other node.
    TX_NO_ACK,          ///< Sent, but nobody acknowledged (TXERR).
    TX_ARB_LOST,        ///< Lost arbitration (MLOA) - proves other traffic exists.
    TX_ABORTED,         ///< Aborted (ABTF).
    TX_NEVER_STARTED    ///< TXREQ never cleared: bus never became idle.
};

#endif // OBD2_PROBE_TYPES_H
