/**
 * @file can_contract_emit.cpp
 * @brief The firmware's side of the end-to-end contract test: real F/FS lines
 *        for a known set of frames.
 *
 *   can_contract_emit format TRUTH   stream lines on stdout, made by the
 *                                    firmware's canFormatFrameLine() /
 *                                    canFormatStatsLine()
 *   can_contract_emit stream TRUTH   USB packets on stdout, one per line as hex,
 *                                    made by the whole production path: frames
 *                                    enter the MCP2515 model from the bus side,
 *                                    the drain ISR reads them into the ring, and
 *                                    canStreamService() packs them (FS first,
 *                                    whole lines, <= 63 bytes) into the bulk-IN
 *                                    bank the stub Serial captures
 *
 * Built from the UNMODIFIED lib/CANRawStream.cpp (and, for `stream`, the drain
 * in lib/CANSniffFunctions.cpp) against the host stand-ins in ../../tests/host,
 * like the firmware's own suites. tests/test_contract.py feeds the output
 * through mkr_stream_log.py on a pty and requires can_raw.log, can_stats.csv
 * and can_decode's view of it to reproduce TRUTH exactly: so a change to the
 * line format on either side of the USB cable fails here, not in a car.
 *
 * TRUTH, one line per item, in stream order:
 *   F <us> <id hex> <ext 0|1> <rtr 0|1> <dlc> <data hex | ->
 *   S <us> <drained> <streamed> <ovf> <ringdrop> <nohost>     (format mode only;
 *                                    in stream mode the FS lines are whatever the
 *                                    firmware emitted, and are read back from
 *                                    the packets)
 *
 * The frames cover what the parser must get exactly right: every DLC 0-8,
 * standard and extended identifiers at both ends of their ranges, remote frames
 * of every length, the map's identifiers with payloads whose decoded values the
 * test checks, and timestamps that cross the micros() wrap with FS lines
 * stamped ahead of the frames that follow them.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "CANRawStream.h"
#include "CANSniffFunctions.h"
#include "CANStreamHw.h"
#include "SDFunctions.h"
#include "mcp2515_model.h"
#include "can_stream_hw_stub.h"

// The SD side is never reached; these satisfy the link, as in the suites.
bool sdReady()                             { return false; }
bool sdOpenRoot(File32 &)                  { return false; }
bool sdOpenRead(const char *, File32 &)    { return false; }
bool sdReadLine(File32 &, char *, size_t)  { return false; }

/// Deterministic, so a failure reproduces: a 32-bit LCG (Numerical Recipes constants).
static uint32_t g_rng = 0x2026A5u;
static uint32_t rnd() { g_rng = g_rng * 1664525u + 1013904223u; return g_rng >> 8; }

struct Spec { uint32_t id; bool ext; bool rtr; uint8_t dlc; uint8_t data[8]; };

/// The frame set, in order. Returns how many were written to @p out.
static unsigned buildFrames(Spec *out, unsigned cap)
{
    unsigned n = 0;
    auto add = [&](uint32_t id, bool ext, bool rtr, uint8_t dlc, const uint8_t *d) {
        if (n >= cap) return;
        Spec &s = out[n++];
        s.id = id; s.ext = ext; s.rtr = rtr; s.dlc = dlc;
        for (uint8_t i = 0; i < 8u; ++i) s.data[i] = (!rtr && d != nullptr && i < dlc) ? d[i] : 0u;
    };
    uint8_t d[8];
    auto randData = [&]() { for (uint8_t i = 0; i < 8u; ++i) d[i] = (uint8_t)rnd(); };

    // Every DLC, both id widths, both ends of each range, and remote frames of every length.
    for (uint8_t dlc = 0; dlc <= 8u; ++dlc) {
        randData(); add(dlc == 0 ? 0x000u : 0x7FFu, false, false, dlc, d);
        randData(); add(rnd() & 0x7FFu, false, false, dlc, d);
        randData(); add(dlc & 1u ? 0x1FFFFFFFu : 0x00000000u, true, false, dlc, d);
        randData(); add(rnd() & 0x1FFFFFFFu, true, false, dlc, d);
        add(rnd() & 0x7FFu, false, true, dlc, nullptr);
        add(rnd() & 0x1FFFFFFFu, true, true, dlc, nullptr);
    }
    // The map's identifiers (config/canmap.brio.txt == the built-in map) with known payloads.
    for (unsigned k = 0; k < 40u; ++k) {
        const uint16_t rpm   = (uint16_t)(700u + 60u * k);
        const uint8_t  pedal = (uint8_t)(k * 6u);
        const uint8_t p17c[8] = { pedal, 0x00, (uint8_t)(rpm >> 8), (uint8_t)rpm,
                                  (uint8_t)((k & 1u) ? 0x01u : 0x00u),          // byte 4 bit 0 = brake_switch (32|1)
                                  0x00, (uint8_t)((k & 2u) ? 0x20u : 0x00u),     // byte 6 bit 5 = brake_pressed (53|1)
                                  (uint8_t)k };
        add(0x17Cu, false, false, 8, p17c);
        const uint16_t spd = (uint16_t)(k * 137u);                               // 0.01 km/h
        const uint8_t p158[8] = { (uint8_t)(spd >> 8), (uint8_t)spd, 0, 0, 0, 0, 0, (uint8_t)k };
        add(0x158u, false, false, 8, p158);
        static const uint8_t gears[] = { 1, 2, 3, 4, 7, 0x0A, 5 };               // 5: not in the gearmap
        const uint8_t p191[8] = { 0, 0, 0, 0, 0, (uint8_t)(0xE0u | gears[k % 7u]), 0, 0 };  // top 3 bits outside 44|5
        add(0x191u, false, false, 8, p191);
        const uint16_t tq = (uint16_t)((k * 29u) & 0x3FFu);                      // 1|10: byte0 bits1-0, byte1
        const uint8_t p1ab[3] = { (uint8_t)(0xFCu | (tq >> 8)), (uint8_t)tq, 0x5A };  // bits outside the field set
        add(0x1ABu, false, false, 3, p1ab);
        const uint8_t p294[8] = { (uint8_t)((k % 3u == 1u) ? 0x20u : (k % 3u == 2u) ? 0x40u : 0x00u), 0, 0, 0, 0, 0, 0, 0 };
        add(0x294u, false, false, 8, p294);
    }
    // Wheel speeds: four 15-bit Motorola fields at 7, 8, 25, 42 — packed here bit by bit, independently
    // of the firmware's extractor, so the decode check is not the extractor agreeing with itself.
    for (unsigned k = 0; k < 10u; ++k) {
        const uint16_t w[4] = { (uint16_t)(1000u + k), (uint16_t)(2000u + 3u * k), (uint16_t)(3000u + 5u * k),
                                (uint16_t)(3100u + 7u * k) };
        static const uint8_t start[4] = { 7, 8, 25, 42 };
        uint8_t p[8] = { 0 };
        for (unsigned i = 0; i < 4u; ++i) {
            uint8_t pos = start[i];
            for (int b = 14; b >= 0; --b) {                       // MSB first, walking down, wrapping to bit 7 of the next byte
                if ((w[i] >> b) & 1u) p[pos / 8u] |= (uint8_t)(1u << (pos % 8u));
                pos = (pos % 8u == 0u) ? (uint8_t)(pos + 15u) : (uint8_t)(pos - 1u);
            }
        }
        add(0x1D0u, false, false, 8, p);
    }
    return n;
}

static void putTruthFrame(FILE *t, uint32_t us, const Spec &s)
{
    fprintf(t, "F %u %X %d %d %u ", (unsigned)us, (unsigned)s.id, s.ext ? 1 : 0, s.rtr ? 1 : 0, (unsigned)s.dlc);
    if (s.rtr || s.dlc == 0u) fputc('-', t);
    for (uint8_t i = 0; !s.rtr && i < s.dlc; ++i) fprintf(t, "%02X", s.data[i]);
    fputc('\n', t);
}

/// First micros() value: 0.1 s before the wrap. The frames span ~0.16 s, so the run crosses it about
/// two thirds of the way through.
static const uint32_t START_US = 0xFFFFFFFFu - 100000u;

static int emitFormat(FILE *truth)
{
    static Spec specs[600];
    const unsigned n = buildFrames(specs, 600);
    uint32_t us = START_US;
    uint32_t drained = 0, streamed = 0, ovf = 0, drop = 0, nohost = 0;
    char line[80];
    for (unsigned i = 0; i < n; ++i) {
        // Every 40 frames an FS line, stamped 3 ms AHEAD of the frames that follow it — as a packet
        // that leads with FS and then carries frames read out earlier. Counters span the digit widths.
        if (i % 40u == 0u) {
            const uint32_t fsUs = us + 3000u;
            const uint8_t k = canFormatStatsLine(line, (uint8_t)sizeof(line), fsUs, drained, streamed, ovf, drop, nohost);
            if (k == 0u) return 1;
            fwrite(line, 1, k, stdout);
            fprintf(truth, "S %u %u %u %u %u %u\n", (unsigned)fsUs, (unsigned)drained, (unsigned)streamed,
                    (unsigned)ovf, (unsigned)drop, (unsigned)nohost);
            drained = drained * 7u + 4294967295u / 3u;    // wraps: uint32 counters are wrapping by contract
            streamed = drained - 5u;
            ovf = ovf * 10u + 9u;
            drop += 1000003u;
            nohost = 4294967295u - i;
        }
        CanRawFrame f;
        canRawPack(f, us, specs[i].id, specs[i].ext, specs[i].rtr, specs[i].dlc, specs[i].data);
        const uint8_t k = canFormatFrameLine(line, (uint8_t)sizeof(line), f);
        if (k == 0u) return 1;
        fwrite(line, 1, k, stdout);
        putTruthFrame(truth, us, specs[i]);
        us += 300u + (rnd() % 600u);                       // wraps past 0xFFFFFFFF part-way through
    }
    return 0;
}

static int emitStream(FILE *truth)
{
    static Spec specs[600];
    const unsigned n = buildFrames(specs, 600);

    // As the firmware's suites set up DISCOVER: controller and drain from power-on, built-in map, a host
    // holding the port (DTR high), the timer running.
    hostReset();
    fakeCanReset();
    hostDrainReset();
    canDrainTimerBegin();
    canSniffSetMap(nullptr);
    uint64_t simUs = 100000000ull;                         // 100 s of uptime, for the mode rate limit
    auto setClock = [&]() {
        hostSetMillis((uint32_t)(simUs / 1000u));
        hostSetMicros((uint32_t)(START_US + (uint32_t)(simUs - 100000000ull)));
    };
    setClock();
    if (canSetMode(CanMode::DISCOVER, MCP2515_DEFAULT_CS_PIN) != CanModeStatus::OK) return 1;
    hostUsbSetDtr(true);

    auto service = [&]() {
        // One packet per call while the bank is armed; collect each as a reading host would.
        for (unsigned guard = 0; guard < 10000u; ++guard) {
            hostUsbClearOutput();
            canStreamService();
            const char *out = hostUsbOutput();
            const size_t len = strlen(out);
            if (len == 0u) return;
            for (size_t i = 0; i < len; ++i) printf("%02X", (unsigned char)out[i]);
            putchar('\n');
            hostUsbCollect();
        }
    };

    for (unsigned i = 0; i < n; ++i) {
        const Spec &s = specs[i];
        if (!fakeCanFrameEx(s.id, s.ext, s.rtr, s.dlc, s.data)) return 1;
        setClock();
        const uint32_t us = (uint32_t)(START_US + (uint32_t)(simUs - 100000000ull));
        hostDrainTick();                                   // the drain stamps micros() as it reads
        putTruthFrame(truth, us, s);
        simUs += 300u + (rnd() % 600u);
        // The host collects in bursts, so packets carry several frames and FS lines lead older frames.
        if (i % 7u == 6u) { setClock(); service(); }
    }
    setClock();
    service();
    simUs += 1100000u;                                     // one more FS, after the last frame
    setClock();
    service();
    return 0;
}

int main(int argc, char **argv)
{
    if (argc != 3 || (strcmp(argv[1], "format") != 0 && strcmp(argv[1], "stream") != 0)) {
        fprintf(stderr, "usage: can_contract_emit format|stream TRUTH_FILE\n");
        return 2;
    }
    FILE *truth = fopen(argv[2], "w");
    if (truth == nullptr) { perror(argv[2]); return 1; }
    const int rc = (strcmp(argv[1], "format") == 0) ? emitFormat(truth) : emitStream(truth);
    fclose(truth);
    fflush(stdout);
    if (rc != 0) fprintf(stderr, "can_contract_emit: the firmware refused a step (rc %d)\n", rc);
    return rc;
}
