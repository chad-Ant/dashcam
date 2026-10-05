#include <Arduino.h>
#include <string.h>

#include "CANRawStream.h"
#include "CANSniffFunctions.h"   // the ring's consumer side and the drain's counters
#include "CANStreamHw.h"         // usbCdcTxIdle()

static uint32_t gStreamed   = 0;
static uint32_t gNoHost     = 0;
/// millis() of the last FS deadline, and whether that line is still owed.
static uint32_t gStatsDueMs = 0;
static bool     gStatsDue   = false;
/// One packet of whole lines. EPX_SIZE is 64; availableForWrite() caps a write
/// at 63, and that cap is applied per packet rather than assumed here.
static char     gPkt[64];
/**
 * A line break is owed before the next stream line.
 *
 * Console text is not always written whole. With nobody reading (DTR low) the
 * core still arms the first packet of a print — "GPS: module started" — and the
 * "\r\n" that follows in its own write times out; a console write can also be
 * refused part-way through a line. Either way the bank or the host's buffer
 * ends in the START of a line, and the next thing written would continue it: a
 * host opening the port would read "GPS: module startedF 0001ABCD 158 8 ..." and
 * lose that frame line to the parser. So after any such moment the next stream
 * packet starts with "\n" — at worst one empty console line. True at boot: the
 * bank may already hold a fragment of setup()'s output.
 */
static bool     gBreakOwed  = true;

static const char kHex[] = "0123456789ABCDEF";

static char *putHex(char *p, uint32_t v, uint8_t digits)
{
    for (uint8_t i = digits; i > 0u; --i) *p++ = kHex[(v >> (4u * (i - 1u))) & 0xFu];
    return p;
}

static uint8_t decDigits(uint32_t v)
{
    uint8_t n = 1;
    while (v >= 10u) { v /= 10u; ++n; }
    return n;
}

static char *putDec(char *p, uint32_t v)
{
    const uint8_t n = decDigits(v);
    for (uint8_t i = n; i > 0u; --i) { p[i - 1u] = (char)('0' + (v % 10u)); v /= 10u; }
    return p + n;
}

uint8_t canFormatFrameLine(char *out, uint8_t cap, const CanRawFrame &f)
{
    const bool    ext = canRawExt(f);
    const bool    rtr = canRawRtr(f);
    const uint8_t dlc = canRawDlc(f);
    const uint8_t idDigits = ext ? 8u : 3u;

    // Length first, so a line that does not fit writes nothing at all — the
    // caller is packing whole lines into a packet and must never send half.
    // "F " ts " " id " " then "R d" for a remote frame, else d [" " data]; "\n".
    const uint8_t len = (uint8_t)(2u + 8u + 1u + idDigits + 1u +
                                  (rtr ? 3u : (1u + (dlc ? 1u + 2u * dlc : 0u))) + 1u);
    if (len > cap) return 0u;

    char *p = out;
    *p++ = 'F';
    *p++ = ' ';
    p = putHex(p, f.us, 8u);
    *p++ = ' ';
    p = putHex(p, canRawId(f), idDigits);
    *p++ = ' ';
    if (rtr) {
        *p++ = 'R';
        *p++ = ' ';
        *p++ = kHex[dlc];
    } else {
        *p++ = kHex[dlc];
        // DLC 0 ends here, with no trailing space: the contract's exact shape,
        // so a parser splitting on single spaces never sees an empty field.
        if (dlc != 0u) {
            *p++ = ' ';
            for (uint8_t i = 0; i < dlc; ++i) p = putHex(p, f.data[i], 2u);
        }
    }
    *p++ = '\n';
    return len;
}

uint8_t canFormatStatsLine(char *out, uint8_t cap, uint32_t us, uint32_t drained,
                           uint32_t streamed, uint32_t ovf, uint32_t ringdrop,
                           uint32_t nohost)
{
    const uint8_t len = (uint8_t)(3u + 8u + 5u + 1u +
                                  decDigits(drained) + decDigits(streamed) + decDigits(ovf) +
                                  decDigits(ringdrop) + decDigits(nohost));
    if (len > cap) return 0u;

    char *p = out;
    *p++ = 'F';
    *p++ = 'S';
    *p++ = ' ';
    p = putHex(p, us, 8u);
    *p++ = ' '; p = putDec(p, drained);
    *p++ = ' '; p = putDec(p, streamed);
    *p++ = ' '; p = putDec(p, ovf);
    *p++ = ' '; p = putDec(p, ringdrop);
    *p++ = ' '; p = putDec(p, nohost);
    *p++ = '\n';
    return len;
}

uint32_t canStreamStreamedCount() { return gStreamed; }
uint32_t canStreamNoHostCount()   { return gNoHost; }

/**
 * Waits for the host to collect the armed packet: true once the bank is free.
 *
 * Bounded three ways — @p maxUs from now, the caller's @p deadlineUs (both on
 * micros(), wrap-safe), and @c CAN_STREAM_WAIT_MAX_POLLS polls — so it ends
 * even with micros() frozen. Polls BK1RDY through usbCdcTxIdle(), never the
 * core's TRCPT1 (see canStreamConsoleWrite() in the header for why that flag
 * cannot be trusted). The drain ISR preempts it as it preempts everything.
 */
static bool waitTxIdle(uint32_t maxUs, uint32_t deadlineUs)
{
    const uint32_t t0 = micros();
    for (uint16_t polls = 0; polls < CAN_STREAM_WAIT_MAX_POLLS; ++polls) {
        if (usbCdcTxIdle()) return true;
        const uint32_t now = micros();
        if ((uint32_t)(now - t0) >= maxUs) return false;
        if ((int32_t)(now - deadlineUs) >= 0) return false;
    }
    return false;
}

size_t canStreamConsoleWrite(const uint8_t *buf, size_t n)
{
    size_t done = 0;
    while (done < n) {
        const size_t len = (n - done > CAN_STREAM_PACKET_MAX) ? CAN_STREAM_PACKET_MAX : (n - done);
        // Only into a bank a reading host will empty. Text nobody collects is
        // DROPPED, not armed: a packet left armed makes the core's NEXT send() —
        // any print, the sketch's own included — spin its full 70 ms, and that
        // hole in loop() is an IMU data gap (bench, 2026-10-06: gaps= rose by one
        // for each spell with the port closed or its reader stalled, while this
        // still handed such text to the core). The console is read only by a
        // host, so text no host takes is lost either way; the break is owed so
        // the stream never continues the dropped line.
        if (!Serial.dtr() || !waitTxIdle(CAN_CONSOLE_WAIT_US, micros() + CAN_CONSOLE_WAIT_US)) {
            gBreakOwed = true;
            break;
        }
        const size_t w = Serial.write(buf + done, len);
        if (w != len) {
            // Refused ((size_t)-1): not enumerated, or the core timed out. What
            // went before may have been the first part of a line; the stream
            // must not continue it (see gBreakOwed).
            gBreakOwed = true;
            break;
        }
        done += len;
    }
    return done;
}

void canStreamService()
{
    // The FS deadline runs whether or not anyone is listening; the line itself
    // is owed only while someone is (below).
    const uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - gStatsDueMs) >= CAN_STREAM_STATS_MS) {
        gStatsDueMs = nowMs;
        gStatsDue   = true;
    }

    // 1) Nobody has the port open (DTR low, CDC.cpp Serial_::dtr()). Writing
    //    anyway would arm a packet no host will collect, and the core's next
    //    send() — ours or any console print — would then wait 70 ms for it. The
    //    frames are released and counted instead, so the ring stays free for
    //    the moment a host appears and SNIFF's decoder is never held up.
    if (!Serial.dtr()) {
        gNoHost   += canRawDiscard(canRawStreamable());
        gStatsDue  = false;   // an FS line for nobody is not owed later
        gBreakOwed = true;    // console prints meanwhile may leave half a line armed
        return;
    }

    // Nothing to send: the common case at 1230 frames/s and four seams a pass,
    // so it costs one comparison — no look at the bank, not even a micros().
    if (!gStatsDue && canRawStreamable() == 0u) return;

    // The call's waiting budget, on micros(): see CAN_STREAM_CALL_BUDGET_US.
    const uint32_t deadlineUs = micros() + CAN_STREAM_CALL_BUDGET_US;

    for (uint8_t k = 0; k < CAN_STREAM_MAX_PACKETS_PER_CALL; ++k) {
        // A call whose last packet emptied the ring ends here, before waiting
        // for a bank it has nothing more to put in.
        if (!gStatsDue && canRawStreamable() == 0u) return;

        // 2) The endpoint must be free, or send() would spin on it. The first
        //    packet takes it as found: an armed bank at the start of a call is
        //    a packet the host has not come for, and waiting on it every call
        //    would charge a stalled host's silence to every pass. Later packets
        //    follow one THIS call armed microseconds ago, which a reading host
        //    collects within a poll or two — so those wait, bounded.
        if (k == 0u) {
            if (!usbCdcTxIdle()) return;
        } else if (!waitTxIdle(CAN_STREAM_WAIT_US, deadlineUs)) {
            return;
        }

        // 3) No more than the core says one write may carry (63 on this core),
        //    and only whole lines — so it is one packet, sent without a wait.
        int room = Serial.availableForWrite();
        if (room > (int)sizeof(gPkt)) room = (int)sizeof(gPkt);
        if (room <= 0) return;

        uint8_t len   = 0;
        uint8_t lines = 0;
        bool    stats = false;
        if (gBreakOwed) gPkt[len++] = '\n';   // ends whatever console text came before

        // FS goes FIRST when due: with frames queued, every packet would
        // otherwise fill before it fitted, and the loss counters would stop
        // reaching the Orin exactly when they matter.
        if (gStatsDue) {
            // One spare byte in front, for an owed break in the case below.
            char line[1u + CAN_STREAM_STATS_LINE_MAX];
            const uint8_t n = canFormatStatsLine(line + 1, CAN_STREAM_STATS_LINE_MAX, micros(),
                                                 canDrainedCount(), gStreamed,
                                                 canSniffOverrunCount(), canRingDropCount(),
                                                 gNoHost);
            if ((int)len + (int)n <= room) {
                memcpy(gPkt + len, line + 1, n);
                len   = (uint8_t)(len + n);
                stats = true;
            } else {
                // THE ONE EXCEPTION — see canStreamService() in the header. Whole,
                // in one call, so nothing can interleave; the core may wait for
                // the host to collect the first of its two packets.
                const char *start = line + 1;
                uint8_t     total = n;
                if (gBreakOwed) { line[0] = '\n'; start = line; ++total; }
                if (Serial.write(reinterpret_cast<const uint8_t *>(start), total) != total) {
                    return;   // refused: owed to the next call, not retried here
                }
                gStatsDue  = false;
                gBreakOwed = false;
                continue;     // the bank is armed now; the next packet waits for it
            }
        }

        CanRawFrame f;
        while (canRawFront(f)) {
            const uint8_t n = canFormatFrameLine(gPkt + len, (uint8_t)(room - len), f);
            if (n == 0u) break;          // whole lines only: it goes in the next packet
            canRawPop();
            len = (uint8_t)(len + n);
            ++lines;
        }
        if (lines == 0u && !stats) return;   // an owed break alone waits for a line

        // Checked against len, not against zero: on failure the core's
        // Serial_::write() returns (size_t)-1 — send()'s -1 passed through the
        // "r > 0" test as a huge unsigned count.
        const size_t w = Serial.write(reinterpret_cast<const uint8_t *>(gPkt), len);
        if (w == len) {
            gStreamed += lines;
            if (stats) gStatsDue = false;
            gBreakOwed = false;
        } else {
            // Enumeration went away between the checks and the write (a bus
            // reset in the USB ISR). The lines are gone and no host received
            // them; the FS line stays owed.
            gNoHost += lines;
            return;
        }
    }
}
