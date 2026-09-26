/**
 * CANRawLog - timestamped raw CAN frame log over the MKR Zero's USB port.
 *
 * The companion to CANDiscovery. That sketch is a census you read by eye; this
 * one streams EVERY frame it drains, with a microsecond timestamp, so a host can
 * record a drive and correlate fields offline against each other, against the
 * decoded wheel speeds, and against the dashcam footage. It is how a signal is
 * found when the question is "which bits move with the steering wheel", not
 * "which bytes changed in the last five seconds".
 *
 * ── SAFETY ───────────────────────────────────────────────────────────────────
 * STRICTLY READ-ONLY on the bus, like CANDiscovery: the MCP2515 is brought up
 * in Configuration mode (vendored CAN.begin(..., stayInConfigurationMode=true))
 * and moved straight to Listen-Only by raw SPI, verified through CANSTAT. It
 * never passes through Normal, never transmits, and emits neither ACK bits nor
 * error frames.
 *
 * ── HARDWARE ─────────────────────────────────────────────────────────────────
 * MKR CAN Shield: MCP2515, CS = D3, 16 MHz crystal, 500 kbps. The INT line is
 * not used — it is not wired on this shield (see CANSniffFunctions.cpp).
 *
 * ── OUTPUT (USB serial, text, one record per line) ───────────────────────────
 *   # ...                               comment / banner
 *   F <us> <id> <dlc> <data>            one frame: us = micros() at drain, hex;
 *                                       id = 11-bit hex; data = 2*dlc hex digits
 *   S <us> <frames> <ovf> <rtr> <ext>   once a second: totals since boot, decimal
 *                                       ovf = receive-buffer overruns (frames the
 *                                       controller dropped because both buffers
 *                                       were full) — the log's own loss figure
 *
 * NO GNSS, for now. A v2 that also logged the u-blox's NAV-PVT over I2C ran
 * correctly (CAN + 5 Hz GNSS) after a full power-up, but right after it was
 * first flashed on the car (2026-09-26) the MKR stopped enumerating on USB
 * (-110/-71), ignored RESET double-taps, and came back only with a full rig
 * power cycle. Cause not isolated, so this file stays at the CAN-only v1 that
 * never did that. A GNSS version should go through the project's own bring-up
 * (lib/I2CBus + gpsInitTick), as CANDiscovery does.
 *
 * Build and flash like any helper (production --library flags); record with
 * can_log.py beside this sketch. Re-flash the production firmware
 * afterwards: while this runs the C3 bridge receives no telemetry.
 */

#include <SPI.h>
#include <CAN.h>

static const int      CS_PIN   = 3;
static const long     OSC_HZ   = 16000000L;
static const long     BAUD     = 500000L;

static const uint8_t REG_CANSTAT  = 0x0E;
static const uint8_t REG_CANCTRL  = 0x0F;
static const uint8_t REG_CANINTF  = 0x2C;
static const uint8_t REG_EFLG     = 0x2D;
static const uint8_t REG_RXB0CTRL = 0x60;
static const uint8_t REG_RXB1CTRL = 0x70;
static const uint8_t MODE_LISTEN_ONLY = 0x60;
static const uint8_t OPMOD_MASK       = 0xE0;

static const SPISettings kSPI(10000000, MSBFIRST, SPI_MODE0);

static uint8_t rawRead(uint8_t reg)
{
    SPI.beginTransaction(kSPI);
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(0x03);
    SPI.transfer(reg);
    const uint8_t v = SPI.transfer(0x00);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
    return v;
}

static void rawWrite(uint8_t reg, uint8_t value)
{
    SPI.beginTransaction(kSPI);
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(0x02);
    SPI.transfer(reg);
    SPI.transfer(value);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
}

static void rawBitModify(uint8_t reg, uint8_t mask, uint8_t value)
{
    SPI.beginTransaction(kSPI);
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(0x05);
    SPI.transfer(reg);
    SPI.transfer(mask);
    SPI.transfer(value);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
}

// ─── output buffering ─────────────────────────────────────────────────────────
//
// One USB write per ~400 bytes rather than one per frame: the per-call overhead
// of the CDC stack is what would otherwise cap the drain.

static char     gOut[512];
static uint16_t gOutLen = 0;
static uint32_t gLastFlushUs = 0;

static void flushOut()
{
    if (gOutLen == 0) return;
    Serial.write(reinterpret_cast<const uint8_t *>(gOut), gOutLen);
    gOutLen = 0;
    gLastFlushUs = micros();
}

static inline void putc_(char c) { gOut[gOutLen++] = c; }

static inline void putHex(uint32_t v, uint8_t digits)
{
    static const char kHex[] = "0123456789ABCDEF";
    for (int8_t i = (int8_t)digits - 1; i >= 0; --i) putc_(kHex[(v >> (4 * i)) & 0xFu]);
}

static void putDec(uint32_t v)
{
    char tmp[11];
    uint8_t n = 0;
    do { tmp[n++] = (char)('0' + (v % 10u)); v /= 10u; } while (v != 0u);
    while (n) putc_(tmp[--n]);
}

// ─── state ────────────────────────────────────────────────────────────────────

static uint32_t gFrames = 0;
static uint32_t gOverflows = 0;
static uint32_t gRtr = 0;
static uint32_t gExt = 0;
static uint32_t gLastStatsMs = 0;
static uint32_t gLastEflgUs = 0;
static bool     gListening = false;

/** @brief Reads one receive buffer; READ RX BUFFER clears its flag on CS rise. */
static void drainBuffer(uint8_t instr)
{
    uint8_t b[13];
    SPI.beginTransaction(kSPI);
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(instr);
    for (uint8_t i = 0; i < 13; ++i) b[i] = SPI.transfer(0x00);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();

    const uint32_t us = micros();
    ++gFrames;
    if (b[1] & 0x08u) { ++gExt; return; }   // IDE: extended ID — not on this bus's map
    if (b[1] & 0x10u) { ++gRtr; return; }   // SRR: standard remote request, no data

    const uint16_t id  = (uint16_t)(((uint16_t)b[0] << 3) | (b[1] >> 5));
    uint8_t        dlc = b[4] & 0x0Fu;
    if (dlc > 8u) dlc = 8u;

    if (gOutLen > sizeof(gOut) - 40u) flushOut();
    putc_('F'); putc_(' ');
    putHex(us, 8);  putc_(' ');
    putHex(id, 3);  putc_(' ');
    putHex(dlc, 1); putc_(' ');
    for (uint8_t i = 0; i < dlc; ++i) putHex(b[5 + i], 2);
    putc_('\n');
}

void setup()
{
    Serial.begin(115200);
    const uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 3000u) {}

    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);

    CAN.setPins(CS_PIN, 7);
    CAN.setClockFrequency(OSC_HZ);
    if (!CAN.begin(BAUD, /*stayInConfigurationMode=*/true)) {
        Serial.println(F("# ERROR: MCP2515 did not answer / reach Configuration - check the shield"));
        return;
    }

    // Accept everything into both buffers, with rollover from RXB0 into RXB1.
    rawWrite(REG_RXB0CTRL, 0x64);   // RXM = 11 | BUKT
    rawWrite(REG_RXB1CTRL, 0x60);   // RXM = 11
    rawWrite(REG_CANINTF, 0x00);
    rawBitModify(REG_EFLG, 0xC0, 0x00);

    rawBitModify(REG_CANCTRL, OPMOD_MASK, MODE_LISTEN_ONLY);
    for (uint16_t tries = 0; tries < 500u; ++tries) {
        if ((rawRead(REG_CANSTAT) & OPMOD_MASK) == MODE_LISTEN_ONLY) { gListening = true; break; }
        delayMicroseconds(100);
    }
    if (!gListening) {
        rawBitModify(REG_CANCTRL, OPMOD_MASK, 0x80);   // park in Configuration
        Serial.println(F("# ERROR: Listen-Only not confirmed; controller parked in Configuration"));
        return;
    }
    Serial.println(F("# CANRawLog v1  listen-only  500 kbps  accept-all  (F us id dlc data | S us frames ovf rtr ext)"));
    gLastStatsMs = millis();
}

void loop()
{
    if (!gListening) { delay(1000); return; }

    const uint8_t intf = rawRead(REG_CANINTF);
    if (intf & 0x01u) drainBuffer(0x90);   // RXB0
    if (intf & 0x02u) drainBuffer(0x94);   // RXB1

    const uint32_t us = micros();
    // Overruns: counted, then cleared so the next one is visible.
    if ((us - gLastEflgUs) > 1000u) {
        gLastEflgUs = us;
        const uint8_t eflg = rawRead(REG_EFLG);
        if (eflg & 0xC0u) {
            gOverflows += ((eflg & 0x40u) ? 1u : 0u) + ((eflg & 0x80u) ? 1u : 0u);
            rawBitModify(REG_EFLG, 0xC0, 0x00);
        }
    }

    if (gOutLen > 400u || (gOutLen > 0u && (us - gLastFlushUs) > 5000u)) flushOut();

    if ((millis() - gLastStatsMs) >= 1000u) {
        gLastStatsMs = millis();
        if (gOutLen > sizeof(gOut) - 64u) flushOut();
        putc_('S'); putc_(' ');
        putHex(micros(), 8); putc_(' ');
        putDec(gFrames);     putc_(' ');
        putDec(gOverflows);  putc_(' ');
        putDec(gRtr);        putc_(' ');
        putDec(gExt);        putc_('\n');
        flushOut();
    }
}
