#ifndef HOST_CAN_STREAM_HW_STUB_H
#define HOST_CAN_STREAM_HW_STUB_H 1

/**
 * @file can_stream_hw_stub.h
 * @brief Test control for the model of lib/CANStreamHw.cpp.
 *
 * The real file programs TC3 and the NVIC and reads the USB endpoint; none of
 * that exists on the host, so can_stream_hw_stub.cpp implements the same four
 * functions over a model:
 *
 *   - the TIMER is the test: hostDrainTick() is one TC3 match;
 *   - the NVIC MASK nests, and a tick that falls due while it is held is left
 *     PENDING and runs the moment the outermost unmask lands — which is what
 *     the silicon does, and what makes a missing mask observable;
 *   - usbCdcTxIdle() reads the bank model in arduino_stub.cpp.
 *
 * ── CATCHING A MISSING MASK ──────────────────────────────────────────────────
 * mcp2515_model.cpp reports every SPI transaction and every byte here. A
 * main-loop transaction made while the drain is ARMED and its interrupt is NOT
 * masked is counted as a violation: on the board, a tick could land inside it.
 * With hostDrainSetPreempt(true) the model goes further: a tick falls due at
 * every byte of main-loop SPI. Unmasked, the ISR runs right there, in the
 * middle of the transaction, so a missing mask also corrupts the transaction
 * the way it would on the board; masked, the tick is left pending and runs at
 * the outermost unmask, which is how a correct caller is seen to lose nothing.
 */

#include <stdint.h>

/// Back to power-on: no mask held, nothing pending, timer stopped, counters zero.
void     hostDrainReset();

/// One TC3 match: runs canDrainIsr() now, or leaves it pending if masked.
/// Does nothing until canDrainTimerBegin() has run, like the real timer.
void     hostDrainTick();

/// Fire the ISR inside unmasked main-loop SPI transactions (see the header).
void     hostDrainSetPreempt(bool on);

uint8_t  hostDrainMaskDepth();     ///< Current canDrainIrqMask() nesting.
bool     hostDrainPending();       ///< A tick is waiting for the unmask.
bool     hostDrainTimerRunning();  ///< canDrainTimerBegin() has run.
uint32_t hostDrainIsrRuns();       ///< canDrainIsr() invocations.
uint32_t hostDrainViolations();    ///< Unmasked main-loop SPI transactions while armed.
uint32_t hostDrainUnbalanced();    ///< Unmasks with no mask held.

// ─── the host on the other end of the bulk IN bank ───────────────────────────
//
// By default the test collects every packet itself (hostUsbCollect()) and
// micros() stands still. The stream now WAITS, bounded, for a packet it just
// armed, so two more knobs make a reading host and the passing of time
// observable — both reset by hostDrainReset():
//
//   - hostUsbSetHostLatency(n): the host collects an armed packet on the n-th
//     usbCdcTxIdle() poll that finds it armed (0: never on its own);
//   - hostUsbSetUsPerPoll(us): every usbCdcTxIdle() poll advances micros() by
//     this much, standing in for the poll's own cost on the board, so a wait
//     bounded in microseconds can be seen to end where it should.

void     hostUsbSetHostLatency(uint32_t polls);
void     hostUsbSetUsPerPoll(uint32_t us);
uint32_t hostUsbIdlePolls();       ///< usbCdcTxIdle() calls since hostDrainReset().

// ─── called by mcp2515_model.cpp ─────────────────────────────────────────────

void hostDrainOnSpiBegin();        ///< At every beginTransaction().
void hostDrainOnSpiByte();         ///< At every transfer(), before the byte.

#endif // HOST_CAN_STREAM_HW_STUB_H
