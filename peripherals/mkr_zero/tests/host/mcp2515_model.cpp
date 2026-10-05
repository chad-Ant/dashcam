#include "mcp2515_model.h"
#include "can_stream_hw_stub.h"

#include <SPI.h>
#include <CAN.h>

#include <string.h>

SPIClass     SPI;
MCP2515Class CAN;

// ─── register file ───────────────────────────────────────────────────────────

static const uint8_t REG_CANSTAT  = 0x0E;
static const uint8_t REG_CANCTRL  = 0x0F;
static const uint8_t REG_CANINTF  = 0x2C;
static const uint8_t REG_EFLG     = 0x2D;
static const uint8_t REG_RXB0CTRL = 0x60;
static const uint8_t REG_RXB0SIDH = 0x61;
static const uint8_t REG_RXB1CTRL = 0x70;
static const uint8_t REG_RXB1SIDH = 0x71;
static const uint8_t REG_TXB0CTRL = 0x30;
static const uint8_t REG_TXB1CTRL = 0x40;
static const uint8_t REG_TXB2CTRL = 0x50;

static const uint8_t OPMOD_MASK   = 0xE0;
static const uint8_t MODE_NORMAL  = 0x00;
static const uint8_t MODE_CONFIG  = 0x80;

static const uint8_t RX0IF  = 0x01;
static const uint8_t RX1IF  = 0x02;
static const uint8_t RX0OVR = 0x40;
static const uint8_t RX1OVR = 0x80;
static const uint8_t BUKT   = 0x04;

static uint8_t  g_reg[256];
static uint16_t g_mask[2];
static uint16_t g_filt[6];
static bool     g_refuse;
static uint32_t g_rejected;
static uint32_t g_overflowed;
static bool     g_everNormal;
static bool     g_misoStuck;
static uint8_t  g_misoValue;

static uint8_t opMode() { return static_cast<uint8_t>(g_reg[REG_CANSTAT] & OPMOD_MASK); }

static void setOpMode(uint8_t mode)
{
    mode = static_cast<uint8_t>(mode & OPMOD_MASK);
    g_reg[REG_CANSTAT] = static_cast<uint8_t>((g_reg[REG_CANSTAT] & ~OPMOD_MASK) | mode);
    if (mode == MODE_NORMAL) g_everNormal = true;
}

/// Every write lands here, so a CANCTRL write requests a mode by any route.
static void writeReg(uint8_t addr, uint8_t value)
{
    g_reg[addr] = value;
    if (addr == REG_CANCTRL) setOpMode(value);   // REQOP completes at once
}

void fakeCanReset()
{
    memset(g_reg, 0, sizeof(g_reg));
    memset(g_mask, 0, sizeof(g_mask));
    memset(g_filt, 0, sizeof(g_filt));
    g_refuse     = false;
    g_rejected   = 0;
    g_overflowed = 0;
    g_everNormal = false;
    g_misoStuck  = false;
    g_reg[REG_CANCTRL] = 0x87;   // datasheet reset value: REQOP=100, CLKEN, CLKPRE
    setOpMode(MODE_CONFIG);
}

// ─── the SPI instruction decoder ─────────────────────────────────────────────
//
// State per transaction. The first byte is the instruction; what follows means
// whatever that instruction says it means.

enum class Op : uint8_t { None, Read, Write, BitMod, ReadRx, Status };

static bool    g_inTx;
static Op      g_op;
static uint8_t g_pos;      ///< Bytes after the instruction byte.
static uint8_t g_addr;
static uint8_t g_bitMask;
static uint8_t g_rxClear;  ///< RXnIF a READ RX BUFFER clears when CS rises.

void SPIClass::beginTransaction(const SPISettings &)
{
    hostDrainOnSpiBegin();   // who is using the bus, and is the drain kept out?
    g_inTx    = true;
    g_op      = Op::None;
    g_pos     = 0;
    g_rxClear = 0;
}

void SPIClass::endTransaction()
{
    // READ RX BUFFER clears its flag on the CS rising edge — which is what the
    // driver relies on to never read the same frame twice.
    if (g_rxClear) g_reg[REG_CANINTF] = static_cast<uint8_t>(g_reg[REG_CANINTF] & ~g_rxClear);
    g_inTx = false;
}

uint8_t SPIClass::transfer(uint8_t b)
{
    hostDrainOnSpiByte();    // where an unmasked drain tick would land
    if (!g_inTx) return 0xFF;
    // A shield that is not there: MISO floats, and every byte reads the same.
    if (g_misoStuck) return g_misoValue;

    if (g_op == Op::None) {
        switch (b) {
        case 0x03: g_op = Op::Read;   break;
        case 0x02: g_op = Op::Write;  break;
        case 0x05: g_op = Op::BitMod; break;
        // READ RX BUFFER: bits 2:1 pick the buffer and the start (SIDH or D0).
        case 0x90: g_op = Op::ReadRx; g_addr = REG_RXB0SIDH;     g_rxClear = RX0IF; break;
        case 0x92: g_op = Op::ReadRx; g_addr = REG_RXB0SIDH + 5; g_rxClear = RX0IF; break;
        case 0x94: g_op = Op::ReadRx; g_addr = REG_RXB1SIDH;     g_rxClear = RX1IF; break;
        case 0x96: g_op = Op::ReadRx; g_addr = REG_RXB1SIDH + 5; g_rxClear = RX1IF; break;
        case 0xA0: g_op = Op::Status; break;
        default:   break;   // an instruction this model does not need
        }
        return 0xFF;
    }

    const uint8_t pos = g_pos++;
    switch (g_op) {
    case Op::Read:
        if (pos == 0) { g_addr = b; return 0xFF; }
        return g_reg[g_addr++];
    case Op::Write:
        if (pos == 0) { g_addr = b; return 0xFF; }
        writeReg(g_addr++, b);
        return 0xFF;
    case Op::BitMod:
        if (pos == 0) { g_addr = b; return 0xFF; }
        if (pos == 1) { g_bitMask = b; return 0xFF; }
        if (pos == 2) {
            writeReg(g_addr, static_cast<uint8_t>((g_reg[g_addr] & ~g_bitMask) | (b & g_bitMask)));
        }
        return 0xFF;
    case Op::ReadRx:
        return g_reg[g_addr++];
    case Op::Status: {
        // READ STATUS (DS20001801 Fig. 12-8): the same byte for as long as it
        // is clocked. RXnIF and TXnIF from CANINTF, TXREQ from each TXBnCTRL.
        const uint8_t intf = g_reg[REG_CANINTF];
        return (uint8_t)(((intf & RX0IF) ? 0x01u : 0u) | ((intf & RX1IF) ? 0x02u : 0u) |
                         ((g_reg[REG_TXB0CTRL] & 0x08u) ? 0x04u : 0u) | ((intf & 0x04u) ? 0x08u : 0u) |
                         ((g_reg[REG_TXB1CTRL] & 0x08u) ? 0x10u : 0u) | ((intf & 0x08u) ? 0x20u : 0u) |
                         ((g_reg[REG_TXB2CTRL] & 0x08u) ? 0x40u : 0u) | ((intf & 0x10u) ? 0x80u : 0u));
    }
    default:
        return 0xFF;
    }
}

// ─── the vendored library's filter write ─────────────────────────────────────

bool MCP2515Class::setFilterRegisters(uint16_t mask0, uint16_t filter0, uint16_t filter1,
                                      uint16_t mask1, uint16_t filter2, uint16_t filter3,
                                      uint16_t filter4, uint16_t filter5,
                                      bool allowRollover, uint8_t targetMode)
{
    // A refusal is modelled as the library's first failure point: its request
    // for Configuration never completes, so OPMOD stays where it was — usually
    // Listen-Only. That is the case where the driver's own park matters, and
    // modelling it this way is what lets a test see whether the park happened.
    //
    // The real library does this over ~30 SPI transactions of its own, which
    // are as exposed to the drain ISR as the driver's raw ones; report one, so
    // the mask check covers them too.
    hostDrainOnSpiBegin();
    hostDrainOnSpiByte();
    if (g_refuse) return false;
    setOpMode(MODE_CONFIG);

    g_reg[REG_RXB0CTRL] = allowRollover ? BUKT : 0x00;   // RXM = 00: filters apply
    g_reg[REG_RXB1CTRL] = 0x00;
    g_mask[0] = static_cast<uint16_t>(mask0 & 0x7FFu);
    g_mask[1] = static_cast<uint16_t>(mask1 & 0x7FFu);
    const uint16_t f[6] = { filter0, filter1, filter2, filter3, filter4, filter5 };
    for (int i = 0; i < 6; ++i) g_filt[i] = static_cast<uint16_t>(f[i] & 0x7FFu);

    setOpMode(targetMode);
    return true;
}

// ─── the bus side ────────────────────────────────────────────────────────────

static bool matches(uint16_t id, uint16_t mask, uint16_t filt)
{
    return ((id ^ filt) & mask) == 0u;
}

static bool filtersOff(uint8_t ctrlReg) { return ((g_reg[ctrlReg] >> 5) & 0x03u) == 0x03u; }

// The driver programs standard filters only (EXIDE = 0), and a standard filter
// never matches an extended frame — so with the filters on, extended frames are
// rejected outright, and only RXM = 11 admits them.
static bool rxb0Accepts(uint32_t id, bool ext)
{
    if (filtersOff(REG_RXB0CTRL)) return true;
    if (ext) return false;
    return matches((uint16_t)id, g_mask[0], g_filt[0]) || matches((uint16_t)id, g_mask[0], g_filt[1]);
}

static bool rxb1Accepts(uint32_t id, bool ext)
{
    if (filtersOff(REG_RXB1CTRL)) return true;
    if (ext) return false;
    for (int i = 2; i < 6; ++i) {
        if (matches((uint16_t)id, g_mask[1], g_filt[i])) return true;
    }
    return false;
}

bool fakeCanAccepts(uint16_t id)
{
    id = static_cast<uint16_t>(id & 0x7FFu);
    return rxb0Accepts(id, false) || rxb1Accepts(id, false);
}

/// Lays a frame into one buffer's registers exactly as the controller does
/// (DS20001801 Register 4-4..4-9). A remote frame stores no data: the data
/// registers keep whatever the previous frame left, which is the trap the
/// driver must not fall into.
static void store(uint8_t sidh, uint32_t id, bool ext, bool rtr, uint8_t rawDlc, const uint8_t *data)
{
    if (ext) {
        g_reg[sidh + 0] = static_cast<uint8_t>(id >> 21);
        g_reg[sidh + 1] = static_cast<uint8_t>((((id >> 18) & 0x07u) << 5) | 0x08u | ((id >> 16) & 0x03u));
        g_reg[sidh + 2] = static_cast<uint8_t>(id >> 8);
        g_reg[sidh + 3] = static_cast<uint8_t>(id);
        g_reg[sidh + 4] = static_cast<uint8_t>((rtr ? 0x40u : 0u) | (rawDlc & 0x0Fu));
    } else {
        g_reg[sidh + 0] = static_cast<uint8_t>(id >> 3);
        g_reg[sidh + 1] = static_cast<uint8_t>(((id & 0x07u) << 5) | (rtr ? 0x10u : 0u));   // SRR, IDE = 0
        g_reg[sidh + 2] = 0;
        g_reg[sidh + 3] = 0;
        g_reg[sidh + 4] = static_cast<uint8_t>(rawDlc & 0x0Fu);
    }
    if (rtr) return;
    const uint8_t n = (rawDlc > 8u) ? 8u : rawDlc;
    for (uint8_t i = 0; i < 8; ++i) {
        g_reg[sidh + 5 + i] = (data != nullptr && i < n) ? data[i] : 0x00;
    }
}

bool fakeCanFrameEx(uint32_t id, bool ext, bool rtr, uint8_t rawDlc, const uint8_t *data)
{
    id = ext ? (id & 0x1FFFFFFFu) : (id & 0x7FFu);
    rawDlc = static_cast<uint8_t>(rawDlc & 0x0Fu);

    // Configuration and Sleep receive nothing. Listen-Only and Normal do.
    const uint8_t mode = opMode();
    if (mode != 0x00 && mode != 0x40 && mode != 0x60) return false;

    uint8_t &intf = g_reg[REG_CANINTF];
    uint8_t &eflg = g_reg[REG_EFLG];

    if (rxb0Accepts(id, ext)) {
        if (!(intf & RX0IF)) {
            store(REG_RXB0SIDH, id, ext, rtr, rawDlc, data);
            intf = static_cast<uint8_t>(intf | RX0IF);
            return true;
        }
        if (g_reg[REG_RXB0CTRL] & BUKT) {
            if (!(intf & RX1IF)) {
                store(REG_RXB1SIDH, id, ext, rtr, rawDlc, data);
                intf = static_cast<uint8_t>(intf | RX1IF);
                return true;
            }
            eflg = static_cast<uint8_t>(eflg | RX1OVR);
        } else {
            eflg = static_cast<uint8_t>(eflg | RX0OVR);
        }
        ++g_overflowed;
        return false;
    }
    if (rxb1Accepts(id, ext)) {
        if (!(intf & RX1IF)) {
            store(REG_RXB1SIDH, id, ext, rtr, rawDlc, data);
            intf = static_cast<uint8_t>(intf | RX1IF);
            return true;
        }
        eflg = static_cast<uint8_t>(eflg | RX1OVR);
        ++g_overflowed;
        return false;
    }
    ++g_rejected;
    return false;
}

bool fakeCanFrame(uint16_t id, uint8_t dlc, const uint8_t *data)
{
    if (dlc > 8) dlc = 8;
    return fakeCanFrameEx(id, false, false, dlc, data);
}

void fakeCanStickMiso(bool stuck, uint8_t value) { g_misoStuck = stuck; g_misoValue = value; }
uint8_t fakeCanReg(uint8_t addr)                  { return g_reg[addr]; }
void fakeCanSetReg(uint8_t addr, uint8_t value)   { g_reg[addr] = value; }

uint8_t  fakeCanOpMode()                    { return opMode(); }
void     fakeCanRefuseFilterWrites(bool r)  { g_refuse = r; }
uint32_t fakeCanRejected()                  { return g_rejected; }
uint32_t fakeCanOverflowed()                { return g_overflowed; }
bool     fakeCanWasEverNormal()             { return g_everNormal; }
