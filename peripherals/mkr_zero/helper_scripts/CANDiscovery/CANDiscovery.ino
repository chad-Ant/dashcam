/**
 * CANDiscovery - passive CAN bus census for the MKR Zero + MCP2515.
 *
 * Answers the two questions that gate the whole CAN-sniffing effort:
 *   1. Does this tap point carry raw broadcast frames at all, or only
 *      diagnostic (0x7E8) responses?  On an OBD-II pins 6/14 tap the gateway
 *      may bridge nothing, in which case sniffing is impossible from there.
 *   2. Which CAN IDs does THIS car actually use, and which bytes inside them
 *      carry the signals we want?
 *
 * Standalone on purpose: it touches no project source and cannot disturb the
 * working telemetry firmware.  Same role as I2CAddressScanner.ino next door.
 *
 * ── SAFETY ───────────────────────────────────────────────────────────────────
 * This sketch is STRICTLY READ-ONLY on the bus.  It never calls beginPacket() /
 * endPacket() and it runs the MCP2515 in Listen-Only mode, so it emits neither
 * ACK bits nor error frames.  Driving either onto a live vehicle bus can set a
 * VSA/ABS fault light.
 *
 * It does NOT use CAN.observe(): in the installed library that writes CANCTRL
 * 0x80, which is CONFIGURATION mode, not Listen-Only (the library's own source
 * carries a "TODO: These should probably be 0x60, not 0x80" beside it).  In
 * Configuration mode the controller is off the bus and receives nothing at all,
 * which would look exactly like "the gateway blocks broadcast traffic" and send
 * you chasing a hardware fault that isn't there.  Mode is set by raw SPI here
 * and verified by reading CANSTAT, not by trusting the CANCTRL readback.
 *
 * ── WIRING / HARDWARE ────────────────────────────────────────────────────────
 * MCP2515 on SPI, CS = D3, INT = D7, 16 MHz crystal, 500 kbps.
 * WARNING: the MKR Zero is NOT 5 V tolerant.  Generic "MCP2515 + TJA1050"
 * modules run at 5 V and drive MISO and INT at 5 V.  Confirm your module is a
 * 3.3 V variant or level-shifted before connecting anything.
 *
 * ── USE ──────────────────────────────────────────────────────────────────────
 * Build/flash with BuildAndUpload.cmd, open the serial monitor at 115200.
 * A table prints every 5 s.  Keys: r = reset stats, d = dump now, h = help.
 *
 * Identification is DIFFERENTIAL - capture, change one thing, capture again:
 *   idle vs 2500 rpm ............ finds the rpm bytes
 *   stationary vs rolling ....... finds speed / wheel speeds
 *   lock-to-lock steering sweep . finds steering angle
 *   P -> R -> N -> D ............ finds gear position
 * The "changed bits" column is the tool for this: it shows, per byte, exactly
 * which bits ever changed during the window.
 */

#include <CAN.h>
#include <SPI.h>

#include "CANTypes.h"   // RawFrame / CanIdStat — see the header for why they live there

// ─── configuration (mirrors DataDictionary.h; standalone so nothing is shared) ─

static const int  CS_PIN    = 3;
static const int  INT_PIN   = 7;
static const long CLOCK_HZ  = 16000000L;   // MUST match the module crystal
static const long BITRATE   = 500000L;     // Honda F-CAN
static const uint32_t REPORT_INTERVAL_MS = 5000;

/// Frames drained per loop() pass. Bounds the worst-case pass; leftovers are
/// picked up next time.
static const uint8_t MAX_FRAMES_PER_POLL = 16;

/// Distinct IDs tracked. 64 x 44 B = 2.8 KB of the MKR Zero's 32 KB.
static const uint8_t CENSUS_SLOTS = 64;

/// Bounded wait for a USB host, so the sketch still runs headless.
static const uint32_t SERIAL_WAIT_MS = 2000;

static const SPISettings SPICfg(10000000, MSBFIRST, SPI_MODE0);

// ─── MCP2515 registers and instructions ───────────────────────────────────────

static const uint8_t REG_CANSTAT  = 0x0E;
static const uint8_t REG_CANCTRL  = 0x0F;
static const uint8_t REG_TEC      = 0x1C;
static const uint8_t REG_REC      = 0x1D;
static const uint8_t REG_CANINTF  = 0x2C;
static const uint8_t REG_EFLG     = 0x2D;
static const uint8_t REG_RXB0CTRL = 0x60;
static const uint8_t REG_RXB1CTRL = 0x70;

static const uint8_t INSTR_WRITE       = 0x02;
static const uint8_t INSTR_READ        = 0x03;
static const uint8_t INSTR_BITMOD      = 0x05;
static const uint8_t INSTR_READ_RXB0   = 0x90; // from RXB0SIDH; auto-clears RX0IF on CS rise
static const uint8_t INSTR_READ_RXB1   = 0x94; // from RXB1SIDH; auto-clears RX1IF on CS rise

static const uint8_t MODE_CONFIG      = 0x80; // CANCTRL REQOP = 100
static const uint8_t MODE_LISTEN_ONLY = 0x60; // CANCTRL REQOP = 011
static const uint8_t OPMOD_MASK       = 0xE0; // CANSTAT bits 7:5

static const uint8_t EFLG_RX0OVR = 0x40;
static const uint8_t EFLG_RX1OVR = 0x80;

// ─── raw SPI helpers ──────────────────────────────────────────────────────────
// The library keeps its register accessors private, so mode/filter/diagnostic
// registers are reached directly. Safe alongside the library: every one of its
// accessors brackets its own transaction and releases CS between calls.

static uint8_t mcpRead(uint8_t reg)
{
    SPI.beginTransaction(SPICfg);
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(INSTR_READ);
    SPI.transfer(reg);
    const uint8_t v = SPI.transfer(0x00);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
    return v;
}

static void mcpWrite(uint8_t reg, uint8_t value)
{
    SPI.beginTransaction(SPICfg);
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(INSTR_WRITE);
    SPI.transfer(reg);
    SPI.transfer(value);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
}

static void mcpBitModify(uint8_t reg, uint8_t mask, uint8_t value)
{
    SPI.beginTransaction(SPICfg);
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(INSTR_BITMOD);
    SPI.transfer(reg);
    SPI.transfer(mask);
    SPI.transfer(value);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
}

/**
 * @brief Requests an operating mode and confirms it actually took effect.
 *
 * CANCTRL only echoes the REQUEST; the achieved mode lives in CANSTAT OPMOD,
 * and a mode change waits for any in-progress frame to finish. Polling CANSTAT
 * is the only way to know. Bounded so a wedged controller cannot hang setup().
 */
static bool mcpSetMode(uint8_t mode)
{
    mcpWrite(REG_CANCTRL, mode);
    for (uint8_t tries = 0; tries < 50; ++tries) {   // ~50 ms ceiling
        if ((mcpRead(REG_CANSTAT) & OPMOD_MASK) == mode) return true;
        delay(1);
    }
    return false;
}

// ─── received frame ───────────────────────────────────────────────────────────

/**
 * @brief Reads one RX buffer with a single READ RX BUFFER instruction.
 *
 * One CS pair and 14 bytes, versus the library's parsePacket() which issues
 * ~15 separate register reads for the same frame. At 500 kbps an 8-byte frame
 * occupies the bus for only ~222 us, so the cheaper read is what makes keeping
 * up with a busy bus possible. The instruction also auto-clears RXnIF on the
 * CS rising edge, so no follow-up write is needed.
 */
static void mcpReadFrame(uint8_t instr, RawFrame &f)
{
    uint8_t b[13];
    SPI.beginTransaction(SPICfg);
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(instr);
    for (uint8_t i = 0; i < 13; ++i) b[i] = SPI.transfer(0x00);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();

    const uint8_t sidh = b[0];
    const uint8_t sidl = b[1];

    f.extended = (sidl & 0x08) != 0;             // IDE
    f.id  = ((uint16_t)sidh << 3) | (sidl >> 5); // 11-bit standard identifier
    f.dlc = b[4] & 0x0F;
    if (f.dlc > 8) f.dlc = 8;                    // malformed DLC guard
    memcpy(f.data, &b[5], 8);
}

// ─── census table ─────────────────────────────────────────────────────────────

static CanIdStat gStat[CENSUS_SLOTS];
static uint8_t   gUsed;              ///< slots occupied
static uint16_t  gSlotOverflow;      ///< frames whose ID found no free slot
static uint32_t  gTotalFrames;
static uint32_t  gExtendedFrames;    ///< 29-bit frames (unexpected on Honda F-CAN)
static uint32_t  gRxOverruns;        ///< EFLG RX0OVR|RX1OVR events
static uint32_t  gWindowStartMs;

static void censusReset()
{
    memset(gStat, 0, sizeof(gStat));
    gUsed           = 0;
    gSlotOverflow   = 0;
    gTotalFrames    = 0;
    gExtendedFrames = 0;
    gRxOverruns     = 0;
    gWindowStartMs  = millis();
}

static void censusAdd(const RawFrame &f, uint32_t nowMs)
{
    ++gTotalFrames;
    if (f.extended) { ++gExtendedFrames; return; }   // table is 11-bit only

    uint8_t slot = 0xFF;
    for (uint8_t i = 0; i < gUsed; ++i) {
        if (gStat[i].id == f.id) { slot = i; break; }
    }

    if (slot == 0xFF) {
        if (gUsed >= CENSUS_SLOTS) { ++gSlotOverflow; return; }
        slot = gUsed++;
        CanIdStat &s = gStat[slot];
        s.id       = f.id;
        s.count    = 0;
        s.firstMs  = nowMs;
        s.minGapMs = 0xFFFF;
        s.maxGapMs = 0;
        memset(s.orMask,  0x00, 8);
        memset(s.andMask, 0xFF, 8);
    }

    CanIdStat &s = gStat[slot];
    if (s.count > 0) {
        const uint32_t gap = nowMs - s.lastMs;
        const uint16_t g   = (gap > 0xFFFF) ? 0xFFFF : (uint16_t)gap;
        if (g < s.minGapMs) s.minGapMs = g;
        if (g > s.maxGapMs) s.maxGapMs = g;
    }
    ++s.count;
    s.lastMs = nowMs;
    s.dlc    = f.dlc;

    for (uint8_t i = 0; i < 8; ++i) {
        s.orMask[i]  |= f.data[i];
        s.andMask[i] &= f.data[i];
        s.last8[i]    = f.data[i];
    }
}

// ─── reporting ────────────────────────────────────────────────────────────────

static void printHex8(uint8_t v)
{
    if (v < 0x10) Serial.print('0');
    Serial.print(v, HEX);
}

/** @brief Prints @p v right-aligned in @p width columns, so the table stays square. */
static void printPadded(uint32_t v, uint8_t width)
{
    uint8_t digits = 1;
    for (uint32_t t = v; t >= 10UL; t /= 10UL) ++digits;
    for (uint8_t i = digits; i < width; ++i) Serial.print(' ');
    Serial.print(v);
}

static void printReport()
{
    const uint32_t nowMs   = millis();
    const uint32_t elapsed = nowMs - gWindowStartMs;
    if (elapsed == 0) return;

    const uint8_t eflg = mcpRead(REG_EFLG);
    const uint8_t tec  = mcpRead(REG_TEC);
    const uint8_t rec  = mcpRead(REG_REC);

    Serial.println();
    Serial.println(F("================ CAN census ================"));
    Serial.print(F("window "));      Serial.print(elapsed / 1000);
    Serial.print(F(" s   frames ")); Serial.print(gTotalFrames);
    Serial.print(F("   ids "));      Serial.print(gUsed);
    Serial.print(F("/"));            Serial.println(CENSUS_SLOTS);

    Serial.print(F("TEC "));           Serial.print(tec);
    Serial.print(F("  REC "));         Serial.print(rec);
    Serial.print(F("  EFLG 0x"));      printHex8(eflg);
    Serial.print(F("  overruns "));    Serial.print(gRxOverruns);
    Serial.print(F("  ext "));         Serial.print(gExtendedFrames);
    Serial.print(F("  slotOverflow ")); Serial.println(gSlotOverflow);

    if (gTotalFrames == 0) {
        Serial.println();
        Serial.println(F("*** NO FRAMES RECEIVED ***"));
        Serial.println(F("  - This tap may not carry broadcast traffic (an OBD-II"));
        Serial.println(F("    gateway often bridges only diagnostic responses)."));
        Serial.println(F("  - Or the bit timing is wrong: check the module crystal"));
        Serial.println(F("    really is 16 MHz, and that the bus is 500 kbps."));
        Serial.println(F("  - Or CANH/CANL are swapped / not connected."));
        Serial.println(F("  TEC/REC stay 0 in listen-only even on a timing"));
        Serial.println(F("  mismatch, so they cannot rule this out on their own."));
        return;
    }

    // Sorted by ID so two captures can be diffed line by line - which is how
    // signals actually get located (idle vs revving, parked vs rolling, ...).
    Serial.println();
    // maxGap flags intermittent / event-driven IDs: a message whose max gap far
    // exceeds its average period is not on a fixed schedule, which is a strong
    // hint on its own (and a reason a signal may look "missing" in a capture).
    Serial.println(F("  ID   Hz  DLC    count maxGap  payload (last)           changed bits"));
    Serial.println(F("----- ---- --- -------- ------ ------------------------ ------------------------"));

    // Selection sort by ID, in place of building an index array (no heap, and
    // 64 entries makes the O(n^2) irrelevant). IDs in the table are unique, so
    // "smallest ID greater than the last one printed" is a total order.
    int32_t lastPrinted = -1;
    for (uint8_t n = 0; n < gUsed; ++n) {
        uint8_t  pick = 0xFF;
        uint32_t best = 0xFFFFFFFFUL;
        for (uint8_t i = 0; i < gUsed; ++i) {
            if ((int32_t)gStat[i].id <= lastPrinted) continue;   // already printed
            if (gStat[i].id < best) { best = gStat[i].id; pick = i; }
        }
        if (pick == 0xFF) break;
        lastPrinted = (int32_t)gStat[pick].id;

        const CanIdStat &s = gStat[pick];
        const uint32_t hz  = (s.count * 1000UL) / elapsed;   // integer, no float

        Serial.print(F("0x"));
        if (s.id < 0x100) Serial.print('0');
        Serial.print(s.id, HEX);
        printPadded(hz, 5);
        Serial.print(F("   "));
        Serial.print(s.dlc);
        printPadded(s.count, 9);
        printPadded((s.count > 1) ? s.maxGapMs : 0, 7);
        Serial.print(' ');

        for (uint8_t i = 0; i < 8; ++i) { printHex8(s.last8[i]); Serial.print(' '); }
        Serial.print(' ');
        // orMask & ~andMask = bits that were not constant across the window.
        // Per-BIT, not per-byte: an unaligned 15-bit wheel-speed field cannot be
        // located from a byte-granular "changed" flag.
        for (uint8_t i = 0; i < 8; ++i) {
            printHex8((uint8_t)(s.orMask[i] & (uint8_t)~s.andMask[i]));
            Serial.print(' ');
        }
        Serial.println();
    }

    if (gRxOverruns > 0) {
        Serial.println();
        Serial.println(F("NOTE: RX overruns occurred - this census is INCOMPLETE."));
        Serial.println(F("      Rates are undercounted and rare IDs may be missing."));
    }
    if (gSlotOverflow > 0) {
        Serial.println();
        Serial.println(F("NOTE: more distinct IDs than slots - raise CENSUS_SLOTS."));
    }
}

static void printHelp()
{
    Serial.println();
    Serial.println(F("keys:  r = reset stats   d = dump now   h = this help"));
}

// ─── bus setup ────────────────────────────────────────────────────────────────

/**
 * @brief Brings the controller up directly into Listen-Only.
 *
 * begin(bitrate, stayInConfigurationMode=true) programs the bit timing but
 * leaves the chip in Configuration mode, so it never passes through Normal mode
 * on the way to Listen-Only. That matters on a live bus: in Normal mode the
 * controller would ACK frames and could emit error frames if the bit timing is
 * wrong, which is exactly what a read-only tap must never do.
 */
static bool busBeginListenOnly()
{
    pinMode(INT_PIN, INPUT_PULLUP);

    CAN.setPins(CS_PIN, INT_PIN);
    CAN.setClockFrequency(CLOCK_HZ);
    if (!CAN.begin(BITRATE, /* stayInConfigurationMode = */ true)) {
        Serial.println(F("CAN.begin() failed - check wiring, CS pin, and that"));
        Serial.println(F("CLOCK_HZ matches the module crystal (8 vs 16 MHz)."));
        return false;
    }

    // Still in Configuration mode here, which is the only time RXBnCTRL is
    // writable. RXM=11 accepts every ID (no filtering - this is a census), and
    // BUKT lets RXB0 spill into RXB1, doubling the depth from 1 frame to 2.
    mcpWrite(REG_RXB0CTRL, 0x64);   // RXM=11 | BUKT
    mcpWrite(REG_RXB1CTRL, 0x60);   // RXM=11
    mcpWrite(REG_CANINTF, 0x00);
    mcpBitModify(REG_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0x00);

    if (!mcpSetMode(MODE_LISTEN_ONLY)) {
        Serial.print(F("Listen-only NOT confirmed. CANSTAT=0x"));
        printHex8(mcpRead(REG_CANSTAT));
        Serial.println();
        Serial.println(F("REFUSING to continue - the controller may be bus-active."));
        return false;
    }
    return true;
}

// ─── Arduino entry points ─────────────────────────────────────────────────────

static bool     gBusReady      = false;
static uint32_t gLastReportMs  = 0;

void setup()
{
    Serial.begin(115200);
    const uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < SERIAL_WAIT_MS) { /* bounded */ }

    Serial.println();
    Serial.println(F("CANDiscovery - passive CAN census (listen-only, read-only)"));
    Serial.print(F("500 kbps, 16 MHz crystal, CS=D"));
    Serial.print(CS_PIN);
    Serial.print(F(" INT=D"));
    Serial.println(INT_PIN);

    gBusReady = busBeginListenOnly();
    if (gBusReady) {
        Serial.print(F("Listen-only CONFIRMED (CANSTAT=0x"));
        printHex8(mcpRead(REG_CANSTAT));
        Serial.println(F(") - the bus is not being driven."));
        printHelp();
    }

    censusReset();
    gLastReportMs = millis();
}

void loop()
{
    if (!gBusReady) { delay(1000); return; }   // refuse to run half-configured

    const uint32_t nowMs = millis();

    // Drain. INT is asserted LOW while either RX buffer holds a frame (CANINTE
    // enables only RX0IE|RX1IE), so this pin read is a ~0.7 us gate that avoids
    // an SPI transaction whenever the bus is idle.
    uint8_t serviced = 0;
    while (serviced < MAX_FRAMES_PER_POLL && digitalRead(INT_PIN) == LOW) {
        const uint8_t intf = mcpRead(REG_CANINTF);
        RawFrame f;
        if (intf & 0x01)      mcpReadFrame(INSTR_READ_RXB0, f);
        else if (intf & 0x02) mcpReadFrame(INSTR_READ_RXB1, f);
        else break;                            // INT low for some other reason
        censusAdd(f, nowMs);
        ++serviced;
    }

    // Overruns must be visible: without this an undercount reads as truth.
    const uint8_t eflg = mcpRead(REG_EFLG);
    if (eflg & (EFLG_RX0OVR | EFLG_RX1OVR)) {
        ++gRxOverruns;
        mcpBitModify(REG_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0x00);
    }

    while (Serial.available() > 0) {
        switch (Serial.read()) {
        case 'r': censusReset(); Serial.println(F("\n[stats reset]")); break;
        case 'd': printReport(); gLastReportMs = nowMs;                break;
        case 'h': printHelp();                                         break;
        default:  break;
        }
    }

    if (nowMs - gLastReportMs >= REPORT_INTERVAL_MS) {
        gLastReportMs = nowMs;
        printReport();
    }
}
