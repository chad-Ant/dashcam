/**
 * OBD2Probe - staged CAN connectivity diagnostic for the MKR Zero + MCP2515.
 *
 * Written because CANDiscovery returned zero frames in 45 s with TEC/REC/EFLG
 * all clean. That result already proves a lot: CAN.begin() succeeded, and
 * Listen-Only was confirmed by reading CANSTAT, so the MCP2515 is present, SPI
 * works, and the bit-timing registers were programmed. Whatever is wrong is
 * OUTSIDE the controller. This sketch isolates which outside thing.
 *
 * Four stages, cheapest and safest first, each answering one question:
 *
 *   1. SPI/controller  - can we read and write MCP2515 registers?     (no bus)
 *   2. Loopback        - do TX framing, RX path and our decode work?  (no bus)
 *   3. Bitrate scan    - is there ANY traffic, at any common rate?    (listen-only)
 *   4. OBD-II query    - does anything ACK and answer us at 500 kbps? (BUS-ACTIVE)
 *
 * Stage 2 is the key isolator: loopback is internal to the MCP2515 and works
 * even with CANH/CANL disconnected. Loopback passing + stages 3/4 failing means
 * the fault is the transceiver, the wiring, or the bus itself - never the
 * controller, the SPI wiring, or this code.
 *
 * Stage 3 matters because CANDiscovery only ever looked at 500 kbps. If the bus
 * is 250 kbps, a 500 kbps listener sees exactly what was reported: nothing at
 * all, with no errors, because in Listen-Only the controller never signals.
 *
 * ── SAFETY ───────────────────────────────────────────────────────────────────
 * Stages 1-3 are strictly passive. **Stage 4 transmits** - it is the only way to
 * learn whether anything ACKs us, which is the single most informative bit here.
 * Precautions:
 *   - Transmission uses mcpTransmit(), which enforces its own deadline. It does
 *     NOT use CAN.endPacket(): that spins on TXREQ with no timeout and only arms
 *     its abort path if TXERR appears, so on a stuck-dominant bus - where the
 *     controller never starts sending and never raises TXERR - it hangs forever.
 *     One-Shot Mode does not prevent this; OSM bounds retransmission attempts,
 *     not the wait for the bus to become idle. An earlier version of this sketch
 *     hung exactly there.
 *   - A bounded number of attempts, then stop. No retry storm.
 *   - TEC is read after every attempt. A climbing TEC means our frames are not
 *     being ACKed and the controller is emitting error frames; the sketch aborts
 *     stage 4 rather than keep provoking the bus.
 *
 * ── WIRING ───────────────────────────────────────────────────────────────────
 * MCP2515 on SPI, CS = D3, INT = D7, 16 MHz crystal.
 * WARNING: the MKR Zero is NOT 5 V tolerant. Generic "MCP2515 + TJA1050" boards
 * run at 5 V and drive MISO and INT at 5 V. Confirm 3.3 V or level-shifted.
 *
 * Build/flash with BuildAndUpload.cmd, serial monitor at 115200.
 * Runs once at boot; press 'r' to run again.
 */

#include <CAN.h>
#include <SPI.h>

#include "CANTypes.h"

// ─── configuration ────────────────────────────────────────────────────────────

static const int  CS_PIN   = 3;
static const int  INT_PIN  = 7;
static const long CLOCK_HZ = 16000000L;   // MUST match the module crystal

/// Rates tried in stage 3. 500 k is the OBD-II standard and what CANDiscovery
/// assumed; the others are here precisely because that assumption is unproven.
static const long SCAN_RATES[] = { 500000L, 250000L, 125000L, 1000000L };
static const uint8_t SCAN_RATE_COUNT = sizeof(SCAN_RATES) / sizeof(SCAN_RATES[0]);
static const uint32_t SCAN_DWELL_MS  = 3000;   // per rate

static const long OBD2_RATE = 500000L;
static const uint8_t OBD2_ATTEMPTS   = 5;
static const uint32_t OBD2_REPLY_MS  = 200;    // ISO 15765-4 P2max is 50 ms; 200 is generous

static const uint32_t SERIAL_WAIT_MS = 2000;

static const SPISettings SPICfg(10000000, MSBFIRST, SPI_MODE0);

// ─── MCP2515 registers / instructions ─────────────────────────────────────────

static const uint8_t REG_CANSTAT  = 0x0E;
static const uint8_t REG_CANCTRL  = 0x0F;
static const uint8_t REG_TEC      = 0x1C;
static const uint8_t REG_REC      = 0x1D;
static const uint8_t REG_CANINTF  = 0x2C;
static const uint8_t REG_EFLG     = 0x2D;
static const uint8_t REG_TXB0CTRL = 0x30;
static const uint8_t REG_TXB0SIDH = 0x31;   ///< Also scratch for the SPI round-trip test.
static const uint8_t REG_RXB0CTRL = 0x60;
static const uint8_t REG_RXB1CTRL = 0x70;

/**
 * Writable bits of RXB0CTRL: RXM[1:0] (6:5) and BUKT (2).
 *
 * Bit 3 is RXRTR, bit 0 is FILHIT0, and bit 1 is BUKT1 - a read-only MIRROR of
 * BUKT that the chip sets itself. Never compare a readback of this register
 * against the value written; mask with this first.
 */
static const uint8_t RXB0CTRL_WRITABLE = 0x64;

static const uint8_t INSTR_WRITE     = 0x02;
static const uint8_t INSTR_READ      = 0x03;
static const uint8_t INSTR_BITMOD    = 0x05;
static const uint8_t INSTR_READ_RXB0 = 0x90;
static const uint8_t INSTR_READ_RXB1 = 0x94;

static const uint8_t MODE_NORMAL      = 0x00;
static const uint8_t MODE_LISTEN_ONLY = 0x60;
static const uint8_t MODE_LOOPBACK    = 0x40;
static const uint8_t OPMOD_MASK       = 0xE0;
static const uint8_t CANCTRL_OSM      = 0x08;

// ─── raw SPI ──────────────────────────────────────────────────────────────────

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

/** @brief Requests a mode and confirms it via CANSTAT OPMOD (not the CANCTRL echo). */
static bool mcpSetMode(uint8_t mode)
{
    mcpWrite(REG_CANCTRL, mode);
    for (uint8_t tries = 0; tries < 50; ++tries) {
        if ((mcpRead(REG_CANSTAT) & OPMOD_MASK) == (mode & OPMOD_MASK)) return true;
        delay(1);
    }
    return false;
}

static void mcpReadFrame(uint8_t instr, RawFrame &f)
{
    uint8_t b[13];
    SPI.beginTransaction(SPICfg);
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(instr);
    for (uint8_t i = 0; i < 13; ++i) b[i] = SPI.transfer(0x00);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();

    f.extended = (b[1] & 0x08) != 0;
    f.id  = ((uint16_t)b[0] << 3) | (b[1] >> 5);
    f.dlc = b[4] & 0x0F;
    if (f.dlc > 8) f.dlc = 8;
    memcpy(f.data, &b[5], 8);
}

/** @brief Pops one frame if either RX buffer holds one. @return true if @p f was filled. */
static bool mcpPoll(RawFrame &f)
{
    const uint8_t intf = mcpRead(REG_CANINTF);
    if (intf & 0x01)      { mcpReadFrame(INSTR_READ_RXB0, f); return true; }
    else if (intf & 0x02) { mcpReadFrame(INSTR_READ_RXB1, f); return true; }
    return false;
}

/**
 * @brief Transmits one standard frame with a hard deadline.
 *
 * Deliberately does NOT use CAN.endPacket(). That function spins on TXREQ with
 * no timeout and only arms its abort path if TXERR appears:
 *
 *     while (readRegister(REG_TXBnCTRL(n)) & 0x08) {
 *         if (readRegister(REG_TXBnCTRL(n)) & 0x10) { ...set ABAT... }
 *         yield();
 *     }
 *
 * On a stuck-dominant bus the controller never starts transmitting at all, so
 * TXREQ stays set and TXERR is never raised - and the loop never exits. One-Shot
 * Mode does not save it: OSM bounds RETRANSMISSION attempts after a failed try,
 * not the wait for the bus to become idle in the first place. This is exactly
 * how the earlier run of this sketch hung.
 *
 * @return see @c TxResult; the frame is always aborted cleanly on timeout.
 */
static TxResult mcpTransmit(uint16_t id, const uint8_t *data, uint8_t len, uint32_t timeoutMs)
{
    if (len > 8) len = 8;

    mcpBitModify(REG_CANCTRL, 0x10, 0x00);   // clear any stale ABAT

    // LOAD TX BUFFER starting at TXB0SIDH: SIDH, SIDL, EID8, EID0, DLC, data.
    SPI.beginTransaction(SPICfg);
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(0x40);                       // LOAD TX BUFFER 0, from SIDH
    SPI.transfer((uint8_t)(id >> 3));         // SIDH
    SPI.transfer((uint8_t)(id << 5));         // SIDL (standard frame, EXIDE clear)
    SPI.transfer(0x00);                       // EID8
    SPI.transfer(0x00);                       // EID0
    SPI.transfer(len);                        // DLC
    for (uint8_t i = 0; i < len; ++i) SPI.transfer(data[i]);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();

    SPI.beginTransaction(SPICfg);
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(0x81);                       // RTS for TXB0 (sets TXREQ)
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();

    uint8_t ctrl = 0;
    const uint32_t t0 = millis();
    do {
        ctrl = mcpRead(REG_TXB0CTRL);
        if ((ctrl & 0x08) == 0) break;        // TXREQ cleared: attempt finished
    } while ((millis() - t0) < timeoutMs);

    if (ctrl & 0x08) {
        // Never even got on the bus. Abort so the controller is left clean.
        mcpBitModify(REG_CANCTRL, 0x10, 0x10);        // ABAT
        const uint32_t t1 = millis();
        while ((mcpRead(REG_TXB0CTRL) & 0x08) && (millis() - t1) < 50) { /* bounded */ }
        mcpBitModify(REG_CANCTRL, 0x10, 0x00);        // release ABAT
        mcpBitModify(REG_CANINTF, 0x04, 0x00);        // clear TX0IF
        return TX_NEVER_STARTED;
    }

    mcpBitModify(REG_CANINTF, 0x04, 0x00);            // clear TX0IF

    if (ctrl & 0x40) return TX_ABORTED;    // ABTF
    if (ctrl & 0x20) return TX_ARB_LOST;   // MLOA
    if (ctrl & 0x10) return TX_NO_ACK;     // TXERR
    return TX_OK;
}

static void printHex8(uint8_t v)
{
    if (v < 0x10) Serial.print('0');
    Serial.print(v, HEX);
}

static void printFrame(const RawFrame &f)
{
    Serial.print(F("      0x"));
    if (f.id < 0x100) Serial.print('0');
    Serial.print(f.id, HEX);
    if (f.extended) Serial.print(F(" (ext)"));
    Serial.print(F("  ["));
    Serial.print(f.dlc);
    Serial.print(F("] "));
    for (uint8_t i = 0; i < f.dlc; ++i) { printHex8(f.data[i]); Serial.print(' '); }
    Serial.println();
}

/** @brief Opens the bus at @p rate, leaving the controller in Configuration mode. */
static bool busOpen(long rate)
{
    CAN.end();
    CAN.setPins(CS_PIN, INT_PIN);
    CAN.setClockFrequency(CLOCK_HZ);
    // stayInConfigurationMode: never transit Normal mode implicitly. Each stage
    // then chooses its own mode deliberately.
    return CAN.begin(rate, /* stayInConfigurationMode = */ true) != 0;
}

/**
 * @brief Accept every ID, enable RXB0->RXB1 rollover. Configuration mode only.
 * @return true if the settings read back (masked to the writable bits).
 */
static bool busAcceptAll()
{
    mcpWrite(REG_RXB0CTRL, RXB0CTRL_WRITABLE);   // RXM=11 | BUKT
    mcpWrite(REG_RXB1CTRL, 0x60);                // RXM=11
    mcpWrite(REG_CANINTF, 0x00);
    mcpBitModify(REG_EFLG, 0xC0, 0x00);

    // Masked comparison: BUKT1 (bit 1) mirrors BUKT and would fail a raw compare.
    return (mcpRead(REG_RXB0CTRL) & RXB0CTRL_WRITABLE) == RXB0CTRL_WRITABLE;
}

// ─── stage 1: SPI / controller ────────────────────────────────────────────────

static bool stage1_spi()
{
    Serial.println(F("\n[1] MCP2515 / SPI"));

    if (!busOpen(500000L)) {
        Serial.println(F("    FAIL  CAN.begin() rejected the setup."));
        Serial.println(F("          - CS pin wrong, or MISO/MOSI/SCK miswired"));
        Serial.println(F("          - or CLOCK_HZ does not match the crystal (8 vs 16 MHz)"));
        return false;
    }

    // Round-trip a SCRATCH register, deliberately not a control register.
    //
    // TXB0SIDH has eight fully writable bits and no side effects while TXREQ is
    // clear (beginPacket() rewrites it before any transmission). Control
    // registers are a trap here: RXB0CTRL bit 1 is BUKT1, a documented READ-ONLY
    // MIRROR of the BUKT bit, so writing 0x64 correctly reads back 0x66. Testing
    // against the written value flags a perfectly healthy controller as broken.
    //
    // 0xA5/0x5A are complementary, so between them every bit is driven both high
    // and low - that is what actually detects a stuck or shorted data line.
    mcpWrite(REG_TXB0SIDH, 0xA5);
    const uint8_t rb1 = mcpRead(REG_TXB0SIDH);
    mcpWrite(REG_TXB0SIDH, 0x5A);
    const uint8_t rb2 = mcpRead(REG_TXB0SIDH);
    mcpWrite(REG_TXB0SIDH, 0x00);

    if (rb1 != 0xA5 || rb2 != 0x5A) {
        Serial.print(F("    FAIL  register round-trip: wrote 0xA5/0x5A, read 0x"));
        printHex8(rb1); Serial.print(F("/0x")); printHex8(rb2);
        Serial.println();
        Serial.print(F("          stuck bits: 0x"));
        printHex8((uint8_t)((rb1 ^ 0xA5) | (rb2 ^ 0x5A)));
        Serial.println();
        Serial.println(F("          SPI is unreliable - check wiring and 3.3 V levels."));
        return false;
    }

    Serial.print(F("    PASS  registers R/W OK, CANSTAT=0x"));
    printHex8(mcpRead(REG_CANSTAT));
    Serial.println();
    return true;
}

// ─── stage 2: loopback self-test (no bus) ─────────────────────────────────────

static bool stage2_loopback()
{
    Serial.println(F("\n[2] Loopback self-test (internal, bus not involved)"));

    if (!busOpen(500000L)) { Serial.println(F("    FAIL  begin()")); return false; }
    if (!busAcceptAll()) {
        Serial.println(F("    FAIL  RXB0CTRL did not accept the accept-all setting."));
        return false;
    }

    if (!mcpSetMode(MODE_LOOPBACK)) {
        Serial.println(F("    FAIL  could not enter Loopback mode."));
        return false;
    }

    // In Loopback the controller ACKs itself, so endPacket() cannot hang here
    // even without One-Shot Mode.
    CAN.beginPacket(0x123, 8);
    for (uint8_t i = 0; i < 8; ++i) CAN.write((uint8_t)(0xA0 + i));
    const int txOk = CAN.endPacket();

    if (!txOk) {
        Serial.println(F("    FAIL  endPacket() failed even in loopback."));
        return false;
    }

    RawFrame f;
    bool got = false;
    const uint32_t t0 = millis();
    while (!got && (millis() - t0) < 100) { got = mcpPoll(f); }

    if (!got) {
        Serial.println(F("    FAIL  transmitted in loopback but received nothing."));
        Serial.println(F("          RX path or filter configuration is broken."));
        return false;
    }

    const bool match = (f.id == 0x123 && f.dlc == 8 && f.data[0] == 0xA0 && f.data[7] == 0xA7);
    Serial.print(match ? F("    PASS  ") : F("    FAIL  "));
    Serial.println(F("looped frame back:"));
    printFrame(f);
    if (match) {
        Serial.println(F("          Controller, SPI, TX framing and RX decode all work."));
        Serial.println(F("          Any bus failure below is EXTERNAL to the MCP2515."));
    }
    return match;
}

// ─── stage 3: passive bitrate scan ────────────────────────────────────────────

static long stage3_scan()
{
    Serial.println(F("\n[3] Passive bitrate scan (listen-only, bus not driven)"));

    long found = 0;
    for (uint8_t r = 0; r < SCAN_RATE_COUNT; ++r) {
        const long rate = SCAN_RATES[r];

        Serial.print(F("    "));
        Serial.print(rate / 1000);
        Serial.print(F(" kbps ... "));

        if (!busOpen(rate)) { Serial.println(F("begin() rejected (unsupported pair)")); continue; }
        (void)busAcceptAll();
        if (!mcpSetMode(MODE_LISTEN_ONLY)) { Serial.println(F("listen-only NOT confirmed - skipped")); continue; }

        uint32_t frames = 0;
        uint16_t firstId = 0xFFFF;
        const uint32_t t0 = millis();
        while ((millis() - t0) < SCAN_DWELL_MS) {
            RawFrame f;
            if (mcpPoll(f)) {
                if (firstId == 0xFFFF) firstId = f.id;
                ++frames;
            }
        }

        Serial.print(frames);
        Serial.print(F(" frames"));
        if (frames > 0) {
            Serial.print(F("  (first ID 0x"));
            Serial.print(firstId, HEX);
            Serial.print(F(")  <-- TRAFFIC HERE"));
            if (found == 0) found = rate;
        }
        Serial.println();
    }

    if (found == 0)
        Serial.println(F("    No traffic at any scanned rate."));
    return found;
}

// ─── stage 4: OBD-II query (BUS-ACTIVE) ───────────────────────────────────────

/// Outcome of the last transmit attempt, so verdict() can distinguish
/// "transmitted but nobody answered" from "never got onto the bus at all".
static TxResult gLastTx = TX_NEVER_STARTED;

static bool stage4_obd2()
{
    Serial.println(F("\n[4] OBD-II query at 500 kbps  ** THIS TRANSMITS **"));

    if (!busOpen(OBD2_RATE)) { Serial.println(F("    FAIL  begin()")); return false; }
    (void)busAcceptAll();   // accept everything, so we see ANY reply, not just 0x7E8

    // One-Shot Mode still helps - it stops the controller retrying a failed
    // frame forever - but it is NOT what makes this safe. The bounded transmit
    // in mcpTransmit() is. See the comment there for why OSM is insufficient.
    if (!mcpSetMode(MODE_NORMAL | CANCTRL_OSM)) {
        Serial.println(F("    FAIL  could not enter Normal mode."));
        return false;
    }
    const bool osm = (mcpRead(REG_CANCTRL) & CANCTRL_OSM) != 0;
    Serial.print(F("    Normal mode entered, One-Shot "));
    Serial.println(osm ? F("confirmed.") : F("NOT set (bounded TX still applies)."));
    Serial.println(F("    Querying Mode 01 PID 0x0C (rpm)."));

    bool anyAck   = false;
    bool anyReply = false;
    gLastTx = TX_NEVER_STARTED;

    for (uint8_t attempt = 0; attempt < OBD2_ATTEMPTS; ++attempt) {
        // 0x7DF = functional (broadcast) request. 0x7E0 = physical to ECM #1.
        // Some gateways answer only one of the two, so alternate.
        const uint16_t txId = (attempt & 1) ? 0x7E0 : 0x7DF;
        const uint8_t  req[8] = { 0x02, 0x01, 0x0C, 0, 0, 0, 0, 0 };

        const TxResult r = mcpTransmit(txId, req, 8, 100);
        gLastTx = r;
        const uint8_t tec = mcpRead(REG_TEC);

        Serial.print(F("    tx 0x"));
        Serial.print(txId, HEX);
        switch (r) {
        case TX_OK:            Serial.print(F("  ACKed"));                 anyAck = true; break;
        case TX_NO_ACK:        Serial.print(F("  sent, NO ACK"));                         break;
        case TX_ARB_LOST:      Serial.print(F("  lost arbitration"));      anyAck = true; break;
        case TX_ABORTED:       Serial.print(F("  aborted"));                              break;
        case TX_NEVER_STARTED: Serial.print(F("  NEVER STARTED (bus not idle)"));         break;
        }
        Serial.print(F("   TEC="));
        Serial.print(tec);

        uint8_t replies = 0;
        const uint32_t t0 = millis();
        while ((millis() - t0) < OBD2_REPLY_MS) {
            RawFrame f;
            if (mcpPoll(f)) {
                if (replies == 0) Serial.println();
                printFrame(f);
                ++replies;
                anyReply = true;
            }
        }
        if (replies == 0) Serial.println(F("   (no reply)"));

        // TXREQ never clearing means the bus is held dominant. Retrying cannot
        // change that, and each attempt burns the full 100 ms timeout.
        if (r == TX_NEVER_STARTED && attempt >= 1) {
            Serial.println(F("    ABORT bus never goes idle - further attempts are pointless."));
            break;
        }
        // A climbing TEC means our frames are going unacknowledged and the
        // controller is signalling errors on the bus. Stop provoking it.
        if (tec >= 96) {
            Serial.println(F("    ABORT TEC >= 96 (error-warning). Stopping to avoid"));
            Serial.println(F("          driving error frames onto a live bus."));
            break;
        }
        delay(100);
    }

    // Leave the controller off the bus regardless of outcome.
    mcpSetMode(MODE_LISTEN_ONLY);

    Serial.print(F("    final TEC="));
    Serial.print(mcpRead(REG_TEC));
    Serial.print(F("  REC="));
    Serial.print(mcpRead(REG_REC));
    Serial.print(F("  EFLG=0x"));
    printHex8(mcpRead(REG_EFLG));
    Serial.println();

    return anyAck || anyReply;
}

// ─── verdict ──────────────────────────────────────────────────────────────────

static void verdict(bool spiOk, bool loopOk, long scanRate, bool obdOk)
{
    Serial.println(F("\n================= VERDICT ================="));

    if (!spiOk) {
        Serial.println(F("MCP2515 not reachable over SPI. Fix wiring/CS/levels first;"));
        Serial.println(F("nothing else in this sketch means anything until it passes."));
        return;
    }
    if (!loopOk) {
        Serial.println(F("Controller reachable but loopback failed - the MCP2515 or"));
        Serial.println(F("this sketch's TX/RX path is at fault, not the vehicle."));
        return;
    }

    Serial.println(F("Controller, SPI and framing are PROVEN GOOD (loopback passed)."));
    Serial.println(F("So the fault is external: transceiver, wiring, or the bus.\n"));

    if (scanRate != 0 && scanRate != 500000L) {
        Serial.print(F("=> The bus is running at "));
        Serial.print(scanRate / 1000);
        Serial.println(F(" kbps, NOT 500 kbps."));
        Serial.println(F("   CANDiscovery saw nothing because it only listened at 500 k."));
        Serial.println(F("   Re-run CANDiscovery with BITRATE set to this rate."));
        return;
    }
    if (scanRate == 500000L) {
        Serial.println(F("=> Broadcast traffic IS present at 500 kbps."));
        Serial.println(F("   CANDiscovery should have seen this - re-run it."));
        return;
    }

    if (obdOk) {
        Serial.println(F("=> No broadcast traffic, but OBD-II WORKS."));
        Serial.println(F("   Wiring, transceiver and 500 kbps are all correct, and the"));
        Serial.println(F("   bus is alive. The gateway simply does not bridge broadcast"));
        Serial.println(F("   frames to pins 6/14 - it answers diagnostic requests only."));
        Serial.println();
        Serial.println(F("   CAN sniffing is NOT possible from the OBD-II port on this"));
        Serial.println(F("   car. Options: (a) tap F-CAN behind the dash, or (b) keep"));
        Serial.println(F("   OBD-II polling as the only source (no steering angle)."));
        return;
    }

    if (gLastTx == TX_NEVER_STARTED) {
        Serial.println(F("=> BUS IS NEVER IDLE - the controller could not even BEGIN to"));
        Serial.println(F("   transmit. TXREQ stayed set with no error flag raised, which"));
        Serial.println(F("   means the RX line is held DOMINANT continuously."));
        Serial.println();
        Serial.println(F("   Physical-layer fault, in likelihood order:"));
        Serial.println(F("     1. TRANSCEIVER NOT POWERED. Most MCP2515 modules carry a"));
        Serial.println(F("        TJA1050/MCP2551 needing 5 V. Powered from 3.3 V the"));
        Serial.println(F("        MCP2515 runs fine - which is exactly why stages 1 and 2"));
        Serial.println(F("        passed - while the transceiver does not, and its RXD"));
        Serial.println(F("        output sits low = permanent dominant. This matches"));
        Serial.println(F("        every result above."));
        Serial.println(F("     2. CANH/CANL shorted together or to ground."));
        Serial.println(F("     3. MCP2515 RXCAN pin shorted low / solder bridge."));
        Serial.println();
        Serial.println(F("   Measure first: VCC at the transceiver IC, and CANH-CANL at"));
        Serial.println(F("   the OBD-II port with ignition ON. A healthy idle bus sits at"));
        Serial.println(F("   roughly 2.5 V on both lines; both near 0 V, or shorted,"));
        Serial.println(F("   confirms this."));
        Serial.println(F("   CAUTION: if the module IS 5 V powered, its MISO and INT drive"));
        Serial.println(F("   5 V into the MKR Zero, which is NOT 5 V tolerant."));
        return;
    }

    Serial.println(F("=> Nothing received AND nothing ACKed, but the controller DID"));
    Serial.println(F("   get onto the bus (it saw an idle bus and transmitted)."));
    Serial.println(F("   Suspect, in order:"));
    Serial.println(F("     1. CANH/CANL open or swapped - transmitting into nothing"));
    Serial.println(F("     2. Ignition off / bus asleep - turn the key to ON"));
    Serial.println(F("     3. Missing 120 ohm termination on a bench rig"));
    Serial.println(F("   A live bus ACKs even a request no ECU chooses to answer, so"));
    Serial.println(F("   'NO ACK' everywhere points at the physical layer."));
}

// ─── Arduino entry points ─────────────────────────────────────────────────────

static void runAll()
{
    Serial.println(F("\n=========================================="));
    Serial.println(F("OBD2Probe - staged CAN connectivity check"));
    Serial.println(F("=========================================="));

    const bool spiOk  = stage1_spi();
    const bool loopOk = spiOk  ? stage2_loopback() : false;
    const long scan   = loopOk ? stage3_scan()     : 0;
    const bool obdOk  = loopOk ? stage4_obd2()     : false;

    verdict(spiOk, loopOk, scan, obdOk);
    Serial.println(F("\n(press 'r' to run again)"));
}

void setup()
{
    pinMode(INT_PIN, INPUT_PULLUP);

    Serial.begin(115200);
    const uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < SERIAL_WAIT_MS) { /* bounded */ }

    runAll();
}

void loop()
{
    while (Serial.available() > 0) {
        if (Serial.read() == 'r') runAll();
    }
    delay(10);
}
