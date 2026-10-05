#ifndef CAN_STREAM_HW_H
#define CAN_STREAM_HW_H 1

/**
 * @file CANStreamHw.h
 * @brief The SAMD21-specific half of the raw CAN stream: the drain timer, the
 *        mask that keeps it off the SPI bus, and the USB CDC transmit state.
 *
 * Everything here touches registers, so CANStreamHw.cpp is the one file of the
 * stream the host tests do NOT compile; tests/host supplies a model of these
 * functions instead (can_stream_hw_stub.cpp). The logic that uses them lives in
 * CANSniffFunctions.cpp and CANRawStream.cpp, which do compile on the host.
 *
 * ── TIMER CHOICE: TC3 ────────────────────────────────────────────────────────
 * Every other timer is spoken for, or could be by a library this firmware
 * links. Checked against arduino:samd 1.8.14 and the link map of this build:
 *   - TC4: the Servo library's only timer (ServoTimers.h, _useTimer1), and
 *     ServoFunctions.cpp is compiled in, so TC4_Handler is already defined.
 *   - TC5: tone() (Tone.cpp, TONE_TC). Unused today, but it is the core's.
 *   - TCC0/1/2: analogWrite() PWM on many pins.
 *   - SysTick: millis()/micros(); RTC: RTCZero.
 * TC3 is used by nothing but analogWrite() on its own two pins, which this
 * firmware never calls; TC3_Handler is the core's weak alias to Dummy_Handler
 * until this file defines it. Clocked from GCLK0 (48 MHz) through the shared
 * TCC2/TC3 channel, which analogWrite() would also point at GCLK0.
 *
 * ── PERIOD: CAN_DRAIN_PERIOD_US ──────────────────────────────────────────────
 * The MCP2515 holds two frames (RXB0, then RXB1 by BUKT rollover); the third to
 * complete before a buffer is read is lost. Three completions are at least
 * T(f2) + T(f3) apart, where T is a frame's length on the wire plus the 3-bit
 * interframe space at 2 us/bit (500 kbps), unstuffed:
 *   - two DLC-0 standard frames, the theoretical worst:  2 x 47 bits = 188 us
 *   - the tightest pair in the 2026-10-03 baseline (DLC 2 + DLC 3):    268 us
 *   - two DLC-8 frames, most of this bus:               2 x 111 bits = 444 us
 * A frame is lost only if the window from one drain finding both buffers empty
 * to the next freeing RXB0 exceeds that. The window is P + J + R:
 *   P = 100 us  the period;
 *   J <= 25 us  latency at priority 1: only USB, EIC and RTC (priority 0) can
 *               hold it off, and the USB ISR is a few to ~20 us outside the
 *               rare control transfers of enumeration and port open;
 *   R ~  35 us  the status read plus the RXB0 read that frees the buffer.
 * 160 us < 188 us, so even back-to-back DLC-0 frames are caught, and this bus's
 * real traffic (>= 268 us) with ~110 us to spare. A 125 us period would leave
 * the theoretical case 3 us; a 200 us one would fit only the real traffic.
 *
 * Cost, estimated from the disassembly of this build (instruction counts, one
 * flash wait state at 48 MHz; SPI.transfer() is ~55 cycles a byte at the
 * 12 MHz SCK the core actually derives from the 10 MHz request): an idle tick —
 * one 2-byte READ STATUS — is ~280 instructions, ~6-8 us, so the drain's
 * standing cost is ~7 % of the CPU at 10 kHz. Each frame adds ~25-30 us (a
 * 14-byte READ RX BUFFER, micros(), the ring copy): ~3.5 % at the measured
 * ~1160 frames/s, ~11 % in the 3870 frames/s burst the baseline log caught.
 *
 * ── PRIORITY: CAN_DRAIN_IRQ_PRIORITY = 1 ─────────────────────────────────────
 * The SAMD21 has four levels (0 highest). The core sets USB and EIC to 0
 * (USBCore.cpp, WInterrupts.c), SysTick to 2 (wiring.c), every SERCOM to 3
 * (SERCOM.h SERCOM_NVIC_PRIORITY: Serial1 to the C3, and the Wire SERCOM,
 * which this firmware uses as a polled master and never interrupts on).
 *   - Above SERCOM (3): Serial1's receiver holds two characters plus one in
 *     the shift register before BUFOVF, ~174 us at 115200. The drain's own
 *     worst case — CAN_DRAIN_MAX_PER_TICK = 3 frames, which is also all a tick
 *     can physically meet (two buffered, one completing during the reads), plus
 *     two status reads and the EFLG read and clear — is ~120 us, which fits
 *     inside that with the USB ISR's ~20 us on top. So preempting the UART
 *     costs it nothing, while letting the UART preempt the drain would add its
 *     latency to J.
 *   - Above SysTick (2): a tick held off for ~100 us is still counted, and
 *     micros() is written to be called from exactly here — it adds a pending
 *     SysTick (PENDSTSET) rather than missing it (delay.c).
 *   - Below USB (0): the USB stack's timing is not this firmware's to take.
 * The I2C waits in vendor/Wire are measured with micros(), so time the drain
 * takes shortens nothing; and at most ~10 % of the CPU it is nowhere near the
 * 8 s watchdog.
 */

#include <stdint.h>

#define CAN_DRAIN_PERIOD_US     100u ///< Drain tick. See the PERIOD section above.
#define CAN_DRAIN_IRQ_PRIORITY  1u   ///< NVIC level, 0 highest. See PRIORITY above.

/**
 * @brief Configures TC3 for @c CAN_DRAIN_PERIOD_US and starts it.
 *
 * Call once, from setup(). The interrupt is harmless before any listen-only
 * mode is entered: @c canDrainIsr() returns at once unless the drain is armed,
 * and only @c canSetMode() arms it.
 */
void canDrainTimerBegin();

/**
 * @brief Keeps the drain ISR off the bus until the matching unmask. Nests.
 *
 * Masks TC3 in the NVIC only — USB, the UART, SysTick and the High-G line keep
 * running, which is why this is not noInterrupts(): a mode transition can poll
 * CANSTAT for tens of milliseconds. A tick that falls due while masked stays
 * PENDING and runs the moment the outermost unmask lands, so nothing is lost
 * but time.
 *
 * Main-loop context only; the ISR never calls it, which is what makes a plain
 * depth counter correct.
 */
void canDrainIrqMask();

/** @brief Undoes one @c canDrainIrqMask(); the outermost re-enables TC3. */
void canDrainIrqUnmask();

/**
 * @brief True when a Serial.write() of up to one packet cannot block.
 *
 * The core's availableForWrite() is NOT that test: Serial_::availableForWrite()
 * returns the constant EPX_SIZE - 1 = 63 (CDC.cpp), whatever the endpoint is
 * doing. What blocks is USBDeviceClass::send() (USBCore.cpp): when the bulk IN
 * bank is still armed with a packet the host has not collected (BK1RDY), it
 * spins for up to TX_TIMEOUT_MS = 70 ms. With the bank free it copies at most
 * 64 bytes into the endpoint buffer, arms it and returns — no wait at all. So
 * this answers exactly the question send() would otherwise answer by blocking.
 *
 * False also while the device is not configured (no host has enumerated it).
 */
bool usbCdcTxIdle();

#endif // CAN_STREAM_HW_H
