#ifndef CAN_RAW_STREAM_H
#define CAN_RAW_STREAM_H 1

/**
 * @file CANRawStream.h
 * @brief Raw CAN frames to the Orin, as text lines on the MKR's native USB.
 *
 * The MKR's USB port used to be a development link — flashing and a console —
 * with production data going only through the ESP32-C3. It is now a production
 * data path: every frame the drain reads goes out here, and the Orin decodes
 * offline. The console shares the port and stays human text.
 *
 * ── THE LINES (fixed contract with the Orin tools) ───────────────────────────
 *   F tttttttt iii d hhhh...      data frame, standard id
 *   F tttttttt iiiiiiii d hhhh... data frame, extended id
 *   F tttttttt iii R d            remote frame (or the 8-digit id)
 *   FS tttttttt drained streamed ovf ringdrop nohost
 *
 * tttttttt is micros() as 8 uppercase hex digits (wraps every 2^32 us): on an F
 * line, when the frame was read out of the MCP2515; on the FS line, when the
 * line was written. Ids are 3 or 8 uppercase hex digits, d is the DLC as one hex
 * digit 0-8, then exactly 2*d uppercase hex data digits — and for d = 0 the line
 * ends after the DLC digit, with no trailing space. FS comes once a second; its
 * five counters are decimal, cumulative since boot, uint32 and wrapping:
 *   drained   frames read out of the MCP2515             canDrainedCount()
 *   streamed  frame lines handed to USB                  canStreamStreamedCount()
 *   ovf       MCP2515 overrun events (each >= 1 frame)   canSniffOverrunCount()
 *   ringdrop  frames lost because the RAM ring was full  canRingDropCount()
 *   nohost    frames not streamed: no host had the port  canStreamNoHostCount()
 * (A bench self-test build counts its synthetic frames as drained too, so the
 * conservation streamed + nohost + ringdrop + queued == drained still holds;
 * see DASHCAM_CAN_STREAM_SELFTEST in CANSniffFunctions.h.)
 *
 * A packet may also begin with a bare "\n" — one empty console line — when the
 * port has just been opened or a console write was refused: the core can leave
 * the START of a console line in the bank, and the stream's next line must not
 * continue it (see gBreakOwed in CANRawStream.cpp). A parser that skips lines
 * not beginning "F " or "FS " needs nothing for it.
 *
 * ── NO OTHER CONSOLE LINE MAY BEGIN WITH "F " OR "FS " ───────────────────────
 * That prefix is how the Orin tells data from console. Every line here is
 * written whole by ONE Serial.write() from loop() — never from an interrupt —
 * so console output, which is also only ever printed from loop(), can come
 * before or after a frame line but never inside one. Callers must therefore
 * call canStreamService() only BETWEEN console messages, never part-way through
 * printing one (the seams in loop() are chosen for that).
 *
 * ── NEVER BLOCKS ─────────────────────────────────────────────────────────────
 * The core's Serial.write() can spin for up to 70 ms (USBDeviceClass::send(),
 * TX_TIMEOUT_MS) when the host has not collected the previous packet, and
 * Serial.availableForWrite() does not warn of it — on arduino:samd 1.8.14 it
 * returns the constant 63. So a write is issued only when all three hold:
 *   1. Serial.dtr()          a host has the port open. Low: frames are counted
 *                            as nohost and released, nothing is written.
 *   2. usbCdcTxIdle()        the bulk IN bank is free, so send() takes its
 *                            no-wait path (see CANStreamHw.h).
 *   3. at most availableForWrite() bytes, whole lines only — under 64, so the
 *                            core sends them as ONE packet, never splitting a
 *                            line across a wait.
 * A host that has the port open but stops reading is case 2 forever: nothing is
 * written, frames wait in the ring, and once it is full the newest are counted
 * as ringdrop. The one exception to (3) is noted at canStreamService().
 *
 * ── WAITING, BOUNDED, ONLY ON OUR OWN PACKET ─────────────────────────────────
 * The bank holds ONE packet, and an 8-byte frame is a 34-byte line, so a packet
 * carries one such frame (two short ones). A call that wrote one packet and
 * returned would make the stream's rate the number of calls per second that
 * happen to find the bank free — and after a write the bank is free again only
 * once the host's next IN poll has collected it, while a bank found free LATE
 * leaves the host polling into NAKs at whatever retry pace its controller
 * keeps. Neither number is this firmware's to know. So a call that still has
 * frames after a write waits for THAT packet to be collected — bounded by
 * @c CAN_STREAM_WAIT_US per packet and @c CAN_STREAM_CALL_BUDGET_US per call —
 * and writes the next one the moment it is: the bank is kept full, which is
 * how the 2026-10-03 baseline recorder (blocking writes, ~5 IN transactions
 * per ~0.3-0.6 ms flush) kept up with the whole bus. A call never waits before
 * its FIRST packet: a bank already armed when the call starts is the host's to
 * collect, and a host that has stopped reading therefore costs one bounded wait
 * in total, not one per call.
 */

#include <stddef.h>
#include <stdint.h>
#include "CANFrameRing.h"

/// FS line period.
#define CAN_STREAM_STATS_MS             1000UL
/**
 * Packets one canStreamService() call may write: a hard bound on its loop. The
 * time budget below is what normally ends a busy call; at the ~60-130 us a
 * packet the baseline recorder measured, 16 packets is ~1-2 ms of a call.
 */
#define CAN_STREAM_MAX_PACKETS_PER_CALL 16u
/**
 * Longest wait for the host to collect the packet this call just armed.
 *
 * Covers a split transaction behind a high-speed hub (one or two 125 us
 * microframes) with margin; a host slower than this is treated as stalled for
 * the rest of the call. Waiting longer buys nothing: the ring, not the call,
 * is what rides out a slow host.
 */
#define CAN_STREAM_WAIT_US              300UL
/**
 * Longest one call may spend waiting on the host, summed over its packets.
 *
 * The cost side of the wait. loop() calls the stream at four seams, so a busy
 * pass gives it ~4 ms at most — invisible next to the 10 ms IMU poll and the
 * 22 ms the C3 link's 256-byte receive buffer holds at 115200 — and an idle
 * stream (nothing queued) waits for nothing at all. At ~100 us a packet a call
 * moves ~10 packets, ~40 a pass: a 920-frame backlog after the worst 750 ms
 * GNSS stage drains in a few tenths of a second.
 */
#define CAN_STREAM_CALL_BUDGET_US       1000UL
/**
 * Polls of the bank per wait, at most: the loop's own bound, so it ends even if
 * micros() were not moving. On the board one poll (usbCdcTxIdle() + micros())
 * is ~2-4 us, so CAN_STREAM_WAIT_US ends a wait long before this does.
 */
#define CAN_STREAM_WAIT_MAX_POLLS       1000u
/// Bytes one write may carry: availableForWrite()'s constant on this core
/// (EPX_SIZE - 1), so a write is always ONE packet.
#define CAN_STREAM_PACKET_MAX           63u
/**
 * Longest wait for a free bank before one packet of console text.
 *
 * Longer than the stream's: a reading host collects within a millisecond, and
 * giving up drops the rest of the text (see canStreamConsoleWrite()).
 */
#define CAN_CONSOLE_WAIT_US             2000UL
/// Longest F line: "F " + 8 + " " + 8 + " " + "8" + " " + 16 + "\n".
#define CAN_STREAM_FRAME_LINE_MAX       39u
/// Longest FS line: "FS " + 8 + five times (" " + 10 digits) + "\n".
#define CAN_STREAM_STATS_LINE_MAX       67u

/**
 * @brief Moves frames from the ring to the USB port; call between console lines.
 *
 * Several times per loop() pass. A call writes whole lines, at most a packet
 * per write, into a free bank only. When frames remain after a write it waits
 * for that packet to be collected and writes the next (see "WAITING, BOUNDED"
 * above), until the ring is empty, @c CAN_STREAM_CALL_BUDGET_US is spent or
 * @c CAN_STREAM_MAX_PACKETS_PER_CALL packets are written; with nothing queued
 * it returns after one look. When the FS line is due it goes first in the next
 * packet.
 *
 * THE ONE EXCEPTION. An FS line is 17 bytes plus its counters' digits, at most
 * 67, and one write may carry 63. It only exceeds that once the counters hold
 * 47+ digits between them — three of them past 10^9 AND ovf and ringdrop with
 * seventeen digits between them (both past 10^8, say): weeks of uptime on a
 * bus losing frames by the hundred million. Then the line is handed to the core
 * whole, which sends it as two packets back to back: still whole and still
 * uninterleaved (nothing else runs inside send()), but send() may wait up to
 * 70 ms for the host to collect the first. Reported rather than hidden: the
 * contract fixes both the format and the cap, and this is the one input where
 * they cannot both hold.
 */
void canStreamService();

/** @brief Frame lines handed to USB since boot (uint32, wrapping). */
uint32_t canStreamStreamedCount();

/** @brief Frames not streamed because no host had the port open (uint32, wrapping). */
uint32_t canStreamNoHostCount();

/**
 * @brief Writes console text in whole packets, each into a free bank.
 *
 * For text that would otherwise go out as many small Serial.print()s — the
 * once-a-second status line is ~110 of them, one packet each. Every print that
 * finds the bank armed enters the core's send() wait, and that wait watches
 * only the transfer-complete flag (TRCPT1), which the USB interrupt clears on
 * every start-of-frame: if the thread is preempted at the instant the host
 * collects — and the 10 kHz drain makes that ~10 % of the time — a 1 kHz
 * start-of-frame interrupt can clear the flag first, and send() then spins its
 * full 70 ms and DROPS the text. Polling BK1RDY before each packet, as the
 * stream does, means send() never enters that wait at all; and ~4 packets a
 * line instead of ~110 leave the bank to the stream.
 *
 * Splits @p buf into packets of at most @c CAN_STREAM_PACKET_MAX bytes and
 * writes them back to back, so nothing else can land inside the text — call it
 * only between stream services, like any console print. Before each packet it
 * waits up to @c CAN_CONSOLE_WAIT_US for a free bank.
 *
 * TEXT NO HOST TAKES IS DROPPED. With DTR low (nobody has the port open) or the
 * bank still armed after the wait (a host that stopped reading), the rest of the
 * text is discarded rather than handed to the core. The core would arm it, and
 * the next send() of anything would then spin TX_TIMEOUT_MS = 70 ms on a packet
 * nobody collects: a 70 ms hole in loop(), which the 100 Hz IMU poll reports as
 * a data gap — measured on the bench, once per spell without a reader, while
 * this function still passed such text through. A dropped line owes a break,
 * so the stream's next packet starts a fresh line.
 *
 * @return bytes written; less than @p n when the rest was dropped or refused.
 */
size_t canStreamConsoleWrite(const uint8_t *buf, size_t n);

/**
 * @brief Formats @p f as one F line, '\n' included, no terminator.
 * @return its length, or 0 — writing nothing — when it would not fit in @p cap.
 */
uint8_t canFormatFrameLine(char *out, uint8_t cap, const CanRawFrame &f);

/**
 * @brief Formats the FS line, '\n' included, no terminator.
 * @return its length, or 0 — writing nothing — when it would not fit in @p cap.
 */
uint8_t canFormatStatsLine(char *out, uint8_t cap, uint32_t us, uint32_t drained,
                           uint32_t streamed, uint32_t ovf, uint32_t ringdrop,
                           uint32_t nohost);

#endif // CAN_RAW_STREAM_H
