// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ARDUINO_ARCH_ESP32

#include "MCP2515.h"

#define REG_BFPCTRL                0x0c
#define REG_TXRTSCTRL              0x0d

#define REG_CANCTRL                0x0f
// DASHCAM PATCH: the achieved mode lives here, not in CANCTRL. CANCTRL only
// echoes the REQUEST, and a mode change waits for any in-progress frame to
// finish, so a readback of CANCTRL reports success before the controller has
// actually left the old mode. Every mode switch below verifies CANSTAT instead.
#define REG_CANSTAT                0x0e
#define OPMOD_MASK                 0xe0
#define REQOP_MASK                 0xe0
#define MODE_NORMAL                0x00
#define MODE_SLEEP                 0x20
#define MODE_LOOPBACK              0x40
#define MODE_LISTEN_ONLY           0x60
#define MODE_CONFIG                0x80

// DASHCAM PATCH: bound on every busy-wait in this file. At the slowest
// supported bit rate an 8-byte frame occupies the bus for ~1.1 ms, and a mode
// change costs at most one frame time, so ~50 ms is several orders of magnitude
// of headroom while still being far below any sane watchdog period.
#define DASHCAM_WAIT_TRIES         500
#define DASHCAM_WAIT_STEP_US       100

#define REG_CNF3                   0x28
#define REG_CNF2                   0x29
#define REG_CNF1                   0x2a

#define REG_CANINTE                0x2b

// Whenever changing the CANINTF register, use BIT MODIFY instead of WRITE.
#define REG_CANINTF                0x2c

#define FLAG_RXnIE(n)              (0x01 << n)
#define FLAG_RXnIF(n)              (0x01 << n)
#define FLAG_TXnIF(n)              (0x04 << n)

// There is a 4-register gap between RXF2EID0 and RXF3SIDH.
#define REG_RXFnSIDH(n)            (0x00 + ((n + (n >= 3)) * 4))
#define REG_RXFnSIDL(n)            (0x01 + ((n + (n >= 3)) * 4))
#define REG_RXFnEID8(n)            (0x02 + ((n + (n >= 3)) * 4))
#define REG_RXFnEID0(n)            (0x03 + ((n + (n >= 3)) * 4))

#define REG_RXMnSIDH(n)            (0x20 + (n * 0x04))
#define REG_RXMnSIDL(n)            (0x21 + (n * 0x04))
#define REG_RXMnEID8(n)            (0x22 + (n * 0x04))
#define REG_RXMnEID0(n)            (0x23 + (n * 0x04))

#define REG_TXBnCTRL(n)            (0x30 + (n * 0x10))
#define REG_TXBnSIDH(n)            (0x31 + (n * 0x10))
#define REG_TXBnSIDL(n)            (0x32 + (n * 0x10))
#define REG_TXBnEID8(n)            (0x33 + (n * 0x10))
#define REG_TXBnEID0(n)            (0x34 + (n * 0x10))
#define REG_TXBnDLC(n)             (0x35 + (n * 0x10))
#define REG_TXBnD0(n)              (0x36 + (n * 0x10))

#define REG_RXBnCTRL(n)            (0x60 + (n * 0x10))
#define REG_RXBnSIDH(n)            (0x61 + (n * 0x10))
#define REG_RXBnSIDL(n)            (0x62 + (n * 0x10))
#define REG_RXBnEID8(n)            (0x63 + (n * 0x10))
#define REG_RXBnEID0(n)            (0x64 + (n * 0x10))
#define REG_RXBnDLC(n)             (0x65 + (n * 0x10))
#define REG_RXBnD0(n)              (0x66 + (n * 0x10))

#define FLAG_IDE                   0x08
#define FLAG_SRR                   0x10
#define FLAG_RTR                   0x40
#define FLAG_EXIDE                 0x08
#define FLAG_RXB0CTRL_BUKT         0x04

#define FLAG_RXM0                  0x20
#define FLAG_RXM1                  0x40


MCP2515Class::MCP2515Class() :
  CANControllerClass(),
  _spiSettings(10E6, MSBFIRST, SPI_MODE0),
  _csPin(MCP2515_DEFAULT_CS_PIN),
  _intPin(MCP2515_DEFAULT_INT_PIN),
  _clockFrequency(MCP2515_DEFAULT_CLOCK_FREQUENCY)
{
}

MCP2515Class::~MCP2515Class()
{
}

int MCP2515Class::begin(long baudRate, bool stayInConfigurationMode)
{
  CANControllerClass::begin(baudRate);

  pinMode(_csPin, OUTPUT);

  // start SPI
  SPI.begin();

  reset();

  if (!switchToConfigurationMode()) {
    return 0;
  }

  const struct {
    long clockFrequency;
    long baudRate;
    uint8_t cnf[3];
  } CNF_MAPPER[] = {
    {  (long)8E6, (long)1000E3, { 0x00, 0x80, 0x00 } },
    {  (long)8E6,  (long)500E3, { 0x00, 0x90, 0x02 } },
    {  (long)8E6,  (long)250E3, { 0x00, 0xb1, 0x05 } },
    {  (long)8E6,  (long)200E3, { 0x00, 0xb4, 0x06 } },
    {  (long)8E6,  (long)125E3, { 0x01, 0xb1, 0x05 } },
    {  (long)8E6,  (long)100E3, { 0x01, 0xb4, 0x06 } },
    {  (long)8E6,   (long)80E3, { 0x01, 0xbf, 0x07 } },
    {  (long)8E6,   (long)50E3, { 0x03, 0xb4, 0x06 } },
    {  (long)8E6,   (long)40E3, { 0x03, 0xbf, 0x07 } },
    {  (long)8E6,   (long)20E3, { 0x07, 0xbf, 0x07 } },
    {  (long)8E6,   (long)10E3, { 0x0f, 0xbf, 0x07 } },
    {  (long)8E6,    (long)5E3, { 0x1f, 0xbf, 0x07 } },

    { (long)16E6, (long)1000E3, { 0x00, 0xd0, 0x82 } },
    { (long)16E6,  (long)500E3, { 0x00, 0xf0, 0x86 } },
    { (long)16E6,  (long)250E3, { 0x41, 0xf1, 0x85 } },
    { (long)16E6,  (long)200E3, { 0x01, 0xfa, 0x87 } },
    { (long)16E6,  (long)125E3, { 0x03, 0xf0, 0x86 } },
    { (long)16E6,  (long)100E3, { 0x03, 0xfa, 0x87 } },
    { (long)16E6,   (long)80E3, { 0x03, 0xff, 0x87 } },
    { (long)16E6,   (long)50E3, { 0x07, 0xfa, 0x87 } },
    { (long)16E6,   (long)40E3, { 0x07, 0xff, 0x87 } },
    { (long)16E6,   (long)20E3, { 0x0f, 0xff, 0x87 } },
    { (long)16E6,   (long)10E3, { 0x1f, 0xff, 0x87 } },
    { (long)16E6,    (long)5E3, { 0x3f, 0xff, 0x87 } },
  };

  const uint8_t* cnf = NULL;

  for (unsigned int i = 0; i < (sizeof(CNF_MAPPER) / sizeof(CNF_MAPPER[0])); i++) {
    if (CNF_MAPPER[i].clockFrequency == _clockFrequency && CNF_MAPPER[i].baudRate == baudRate) {
      cnf = CNF_MAPPER[i].cnf;
      break;
    }
  }

  if (cnf == NULL) {
    return 0;
  }

  writeRegister(REG_CNF1, cnf[0]);
  writeRegister(REG_CNF2, cnf[1]);
  writeRegister(REG_CNF3, cnf[2]);

  writeRegister(REG_CANINTE, FLAG_RXnIE(1) | FLAG_RXnIE(0));
  writeRegister(REG_BFPCTRL, 0x00);
  writeRegister(REG_TXRTSCTRL, 0x00);

  // A combination of RXM1 and RXM0 is "Turns mask/filters off; receives any message".
  writeRegister(REG_RXBnCTRL(0), FLAG_RXM1 | FLAG_RXM0);
  writeRegister(REG_RXBnCTRL(1), FLAG_RXM1 | FLAG_RXM0);

  if (!stayInConfigurationMode) {
    if (!switchToNormalMode()) {
      return 0;
    }
  }

  return 1;
}

void MCP2515Class::end()
{
  SPI.end();

  CANControllerClass::end();
}

int MCP2515Class::endPacket()
{
  if (!CANControllerClass::endPacket()) {
    return 0;
  }

  // Currently, we don't need to use more than one TX buffer as we always wait
  // until the data has been fully transmitted. For the same reason, we don't
  // need to check in the beginning whether there is any data in the TX buffer
  // pending transmission. The performance can be optimized by utilizing all
  // three TX buffers of the MCP2515, but this will come at extra complexity.
  int n = 0;

  // Pre-calculate values for all registers so that we can write them
  // sequentially via the LOAD TX BUFFER instruction.
  // TX BUFFER
  uint8_t regSIDH;
  uint8_t regSIDL;
  uint8_t regEID8;
  uint8_t regEID0;
  if (_txExtended) {
    regSIDH = _txId >> 21;
    regSIDL =
        (((_txId >> 18) & 0x07) << 5) | FLAG_EXIDE | ((_txId >> 16) & 0x03);
    regEID8 = (_txId >> 8) & 0xff;
    regEID0 = _txId & 0xff;
  } else {
    regSIDH = _txId >> 3;
    regSIDL = _txId << 5;
    regEID8 = 0x00;
    regEID0 = 0x00;
  }

  uint8_t regDLC;
  if (_txRtr) {
    regDLC = 0x40 | _txLength;
  } else {
    regDLC = _txLength;
  }

  SPI.beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  // Send the LOAD TX BUFFER instruction to sequentially write registers,
  // starting from TXBnSIDH(n).
  SPI.transfer(0b01000000 | (n << 1));
  SPI.transfer(regSIDH);
  SPI.transfer(regSIDL);
  SPI.transfer(regEID8);
  SPI.transfer(regEID0);
  SPI.transfer(regDLC);
  if (!_txRtr) {
    for (uint8_t i = 0; i < _txLength; i++) {
      SPI.transfer(_txData[i]);
    }
  }
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();

  SPI.beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  // Send the RTS instruction, which sets the TXREQ (TXBnCTRL[3]) bit for the
  // respective buffer, and clears the ABTF, MLOA and TXERR bits.
  SPI.transfer(0b10000000 | (1 << n));
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();

  // Wait until the transmission completes, or gets aborted.
  // Transmission is pending while TXREQ (TXBnCTRL[3]) bit is set.
  //
  // DASHCAM PATCH: upstream spins here with NO deadline, and arms its abort only
  // once TXERR appears. On a stuck-dominant bus neither happens - the controller
  // never starts sending because the bus never goes idle, so TXREQ stays set and
  // TXERR stays clear, forever. One-Shot Mode does NOT bound this: OSM limits
  // RE-transmission after an attempt, it does not bound waiting for an idle bus.
  // Observed in this project: OBD2Probe hung exactly here, and in production the
  // watchdog would reset the board straight back into the same call - a reboot
  // loop, not a recovery.
  //
  // So: a hard deadline, then an unconditional abort, then a bounded wait for
  // the abort itself to land. A transmit that cannot start is reported as a
  // failure, which the caller can act on; a hang is not.
  bool aborted = false;
  bool timedOut = true;
  bool txreqCleared = false;   // DASHCAM PATCH: gates the ABAT release below.
  for (uint16_t tries = 0; tries < DASHCAM_WAIT_TRIES; tries++) {
    if (!(readRegister(REG_TXBnCTRL(n)) & 0x08)) {
      timedOut = false;
      break;
    }
    // Read the TXERR (TXBnCTRL[4]) bit to check for errors.
    if (readRegister(REG_TXBnCTRL(n)) & 0x10) {
      // Abort on errors by setting the ABAT bit. The MCP2515 will should the
      // TXREQ bit shortly. We'll keep running the loop until TXREQ is cleared.
      modifyRegister(REG_CANCTRL, 0x10, 0x10);
      aborted = true;
    }

    delayMicroseconds(DASHCAM_WAIT_STEP_US);
    yield();
  }

  if (timedOut) {
    // Never started, or never finished. Abort unconditionally and give the
    // controller a bounded chance to drop TXREQ, so the buffer is not left
    // armed to fire the moment the bus recovers - by then the request is stale
    // and the caller has already been told it failed.
    modifyRegister(REG_CANCTRL, 0x10, 0x10);
    aborted = true;
    for (uint16_t tries = 0; tries < DASHCAM_WAIT_TRIES; tries++) {
      if (!(readRegister(REG_TXBnCTRL(n)) & 0x08)) {
        txreqCleared = true;
        break;
      }
      delayMicroseconds(DASHCAM_WAIT_STEP_US);
      yield();
    }
  }

  // DASHCAM PATCH: ABAT is released only once TXREQ is CONFIRMED clear.
  //
  // It used to be cleared unconditionally after the wait loop above, with the
  // loop's result discarded. If TXREQ had not dropped - the case the abort
  // exists for - releasing ABAT re-arms the buffer, and a stale request fires
  // the instant the bus recovers: a diagnostic frame the caller was already
  // told had failed, transmitted seconds later, unattended, on a live vehicle.
  //
  // Leaving ABAT asserted is the safe direction. It blocks further transmission
  // rather than permitting an unintended one, and the next initializeOBD2()
  // resets the controller, which clears it.
  if (aborted && txreqCleared) {
    // Reset the ABAT bit.
    modifyRegister(REG_CANCTRL, 0x10, 0x00);
  }

  // Clear the pending TX interrupt, if any.
  modifyRegister(REG_CANINTF, FLAG_TXnIF(n), 0x00);

  // DASHCAM PATCH: a timeout is a FAILURE, unconditionally.
  //
  // This used to fall through to the error-bit test below, which asks a
  // different question: "did the controller record an error", not "did the
  // frame go out". After the abort path above clears ABAT, ABTF does not
  // necessarily remain latched, so a transmission that never completed and was
  // then aborted could read back clean and be reported as SUCCESS. The caller
  // then counts a delivered request that no ECU ever saw, and the TX-failure
  // link detector never fires — the link looks healthy precisely because it is
  // too broken to answer.
  if (timedOut) {
    return 0;
  }

  // Report failure if either of the ABTF, MLOA or TXERR bits are set.
  // TODO: perhaps we can reuse the last value read from this register // earlier?
  return (readRegister(REG_TXBnCTRL(n)) & 0x70) ? 0 : 1;
}

int MCP2515Class::parsePacket()
{
  SPI.beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  SPI.transfer(0xb0);  // RX STATUS
  uint8_t rxStatus = SPI.transfer(0x00);
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();

  int n;
  if (rxStatus & 0x40) {
    n = 0;
  } else if (rxStatus & 0x80) {
    n = 1;
  } else {
    _rxId = -1;
    _rxExtended = false;
    _rxRtr = false;
    _rxDlc = 0;
    _rxIndex = 0;
    _rxLength = 0;
    return 0;
  }

  SPI.beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  // Send READ RX BUFFER instruction to sequentially read registers, starting
  // from RXBnSIDH(n).
  SPI.transfer(0b10010000 | (n * 0x04));
  uint8_t regSIDH = SPI.transfer(0x00);
  uint8_t regSIDL = SPI.transfer(0x00);
  _rxExtended = (regSIDL & FLAG_IDE) ? true : false;

  // We could just skip the extended registers for standard frames, but that
  // would actually add more overhead, and increase complexity.
  uint8_t regEID8 = SPI.transfer(0x00);
  uint8_t regEID0 = SPI.transfer(0x00);
  uint8_t regDLC = SPI.transfer(0x00);
  uint32_t idA = (regSIDH << 3) | (regSIDL >> 5);
  if (_rxExtended) {
    uint32_t idB =
        ((uint32_t)(regSIDL & 0x03) << 16)
        | ((uint32_t)regEID8 << 8)
        | regEID0;

    _rxId = (idA << 18) | idB;
    _rxRtr = (regDLC & FLAG_RTR) ? true : false;
  } else {
    _rxId = idA;
    _rxRtr = (regSIDL & FLAG_SRR) ? true : false;
  }

  // DASHCAM PATCH: clamp to 8. The DLC field is four bits, and ISO 11898-1
  // states that any value above 8 still means eight data bytes - a conforming
  // transmitter may legally send DLC 9..15, and some ECUs do. Upstream copied
  // _rxDlc bytes into _rxData[8] unclamped, so DLC 15 wrote SEVEN bytes past the
  // end of the buffer. _rxData is the LAST member of CANControllerClass, so the
  // overflow lands directly in MCP2515Class's own _spiSettings, _csPin and
  // _intPin: one malformed frame silently repoints the driver's chip-select.
  // The MCP2515 only stores eight data bytes anyway, so nothing is lost.
  _rxDlc = regDLC & 0x0f;
  if (_rxDlc > 8) {
    _rxDlc = 8;
  }
  _rxIndex = 0;

  if (_rxRtr) {
    _rxLength = 0;
  } else {
    _rxLength = _rxDlc;

    // Get the data.
    for (uint8_t i = 0; i < _rxLength; i++) {
      _rxData[i] = SPI.transfer(0x00);
    }
  }

  // Don't need to unset the RXnIF(n) flag as this is done automatically when
  // setting the CS high after a READ RX BUFFER instruction.
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
  return _rxDlc;
}

void MCP2515Class::onReceive(void(*callback)(int))
{
  CANControllerClass::onReceive(callback);

  pinMode(_intPin, INPUT);

  if (callback) {
    SPI.usingInterrupt(digitalPinToInterrupt(_intPin));
    attachInterrupt(digitalPinToInterrupt(_intPin), MCP2515Class::onInterrupt, LOW);
  } else {
    detachInterrupt(digitalPinToInterrupt(_intPin));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.notUsingInterrupt(digitalPinToInterrupt(_intPin));
#endif
  }
}

int MCP2515Class::filter(int id, int mask)
{
  id &= 0x7ff;
  mask &= 0x7ff;

  // DASHCAM PATCH: verify CANSTAT, not the CANCTRL readback. Upstream declared
  // success as soon as CANCTRL echoed the request, but the controller finishes
  // any frame in progress before the mode actually changes - so on an ACTIVE bus
  // the filter register writes below could land while still in Normal mode,
  // where they are ignored, and initialisation would report success anyway.
  if (!switchToConfigurationMode()) {
    return 0;
  }

  for (int n = 0; n < 2; n++) {
    // DASHCAM PATCH: RXM<1:0> = 00, "receive all valid messages that meet the
    // filter criteria" - the only setting that actually applies the filters
    // being written three lines below. Upstream wrote FLAG_RXM0 (RXM = 01),
    // twice to the same register, with its own TODO doubting it. RXM 01 and 10
    // are documented as reserved/not-supported on current silicon revisions.
    // RXB0 also gets BUKT so a full RXB0 rolls over into RXB1, doubling the
    // usable receive depth from one frame to two.
    writeRegister(REG_RXBnCTRL(n), (n == 0) ? FLAG_RXB0CTRL_BUKT : 0x00);

    writeRegister(REG_RXMnSIDH(n), (uint8_t)(mask >> 3));
    writeRegister(REG_RXMnSIDL(n), (uint8_t)((mask & 0x07) << 5));
    writeRegister(REG_RXMnEID8(n), 0);
    writeRegister(REG_RXMnEID0(n), 0);
  }

  for (int n = 0; n < 6; n++) {
    writeRegister(REG_RXFnSIDH(n), id >> 3);
    writeRegister(REG_RXFnSIDL(n), id << 5);
    writeRegister(REG_RXFnEID8(n), 0);
    writeRegister(REG_RXFnEID0(n), 0);
  }

  // DASHCAM PATCH: verified via CANSTAT. NOTE for callers: this function still
  // ends in Normal mode, which CANCELS Listen-Only. That is upstream's contract
  // and is left intact so existing callers keep working - but it means filter()
  // must never appear in a read-only path. Use setFilterRegisters(), which takes
  // the target mode as an argument.
  return switchToNormalMode() ? 1 : 0;
}

boolean MCP2515Class::setFilterRegisters(
    uint16_t mask0, uint16_t filter0, uint16_t filter1,
    uint16_t mask1, uint16_t filter2, uint16_t filter3, uint16_t filter4, uint16_t filter5,
    bool allowRollover, uint8_t targetMode)
{
  mask0 &= 0x7ff;
  filter0 &= 0x7ff;
  filter1 &= 0x7ff;
  mask1 &= 0x7ff;
  filter2 &= 0x7ff;
  filter3 &= 0x7ff;
  filter4 &= 0x7ff;
  filter5 &= 0x7ff;

  if (!switchToConfigurationMode()) {
    return false;
  }

  writeRegister(REG_RXBnCTRL(0), allowRollover ? FLAG_RXB0CTRL_BUKT : 0);
  writeRegister(REG_RXBnCTRL(1), 0);

  // DASHCAM PATCH: uint16_t, not uint8_t. Upstream copied the uint16_t
  // parameters into uint8_t locals, so every 11-bit value lost its top three
  // bits. The failure is quiet and asymmetric: mask 0x7FF became an effective
  // 0x0FF and filter 0x158 became 0x058, so 0x158 was STILL accepted - along
  // with 0x058, 0x258, 0x358 ... 0x758. The filter silently became eight times
  // more permissive than asked for, which on a busy bus is precisely the RX
  // overrun it was installed to prevent. The compiler said so: six narrowing
  // warnings at -Wall.
  const uint16_t masks[2] = { mask0, mask1 };
  for (int n = 0; n < 2; n++) {
    const uint16_t mask = masks[n];
    writeRegister(REG_RXMnSIDH(n), (uint8_t)(mask >> 3));
    writeRegister(REG_RXMnSIDL(n), (uint8_t)((mask & 0x07) << 5));
    writeRegister(REG_RXMnEID8(n), 0);
    writeRegister(REG_RXMnEID0(n), 0);
  }

  const uint16_t filter_array[6] =
      {filter0, filter1, filter2, filter3, filter4, filter5};
  for (int n = 0; n < 6; n++) {
    const uint16_t id = filter_array[n];
    writeRegister(REG_RXFnSIDH(n), (uint8_t)(id >> 3));
    writeRegister(REG_RXFnSIDL(n), (uint8_t)((id & 0x07) << 5));
    writeRegister(REG_RXFnEID8(n), 0);
    writeRegister(REG_RXFnEID0(n), 0);
  }

  // DASHCAM PATCH: the caller says where to end up. Upstream forced Normal mode
  // here, which silently cancels Listen-Only - so "install filters, then sniff
  // read-only" was impossible to express, and any listen-only path that
  // configured filters became bus-active without saying so.
  return switchToMode(targetMode);
}

int MCP2515Class::filterExtended(long id, long mask)
{
  id &= 0x1FFFFFFF;
  mask &= 0x1FFFFFFF;

  // DASHCAM PATCH: verified mode switch - see filter() for why the CANCTRL
  // readback upstream used is not evidence.
  if (!switchToConfigurationMode()) {
    return 0;
  }

  for (int n = 0; n < 2; n++) {
    // DASHCAM PATCH: RXM<1:0> = 00 so the filters below actually apply, plus
    // BUKT on RXB0 for rollover. Upstream wrote FLAG_RXM1 (RXM = 10) twice.
    // The EXIDE bit in each FILTER decides standard-vs-extended matching; the
    // reserved RXM encodings were never needed for that.
    writeRegister(REG_RXBnCTRL(n), (n == 0) ? FLAG_RXB0CTRL_BUKT : 0x00);

    // DASHCAM PATCH: 0x07, not 0x03. SIDL bits 7:5 carry EID bits 20:18 - three
    // bits. Masking with 0x03 kept only bits 19:18 and silently dropped bit 20,
    // so any 29-bit filter or mask differing only in that bit was wrong.
    writeRegister(REG_RXMnSIDH(n), (uint8_t)(mask >> 21));
    writeRegister(REG_RXMnSIDL(n), (uint8_t)((((mask >> 18) & 0x07) << 5) | FLAG_EXIDE | ((mask >> 16) & 0x03)));
    writeRegister(REG_RXMnEID8(n), (uint8_t)((mask >> 8) & 0xff));
    writeRegister(REG_RXMnEID0(n), (uint8_t)(mask & 0xff));
  }

  for (int n = 0; n < 6; n++) {
    writeRegister(REG_RXFnSIDH(n), (uint8_t)(id >> 21));
    writeRegister(REG_RXFnSIDL(n), (uint8_t)((((id >> 18) & 0x07) << 5) | FLAG_EXIDE | ((id >> 16) & 0x03)));
    writeRegister(REG_RXFnEID8(n), (uint8_t)((id >> 8) & 0xff));
    writeRegister(REG_RXFnEID0(n), (uint8_t)(id & 0xff));
  }

  // DASHCAM PATCH: verified via CANSTAT. NOTE for callers: this function still
  // ends in Normal mode, which CANCELS Listen-Only. That is upstream's contract
  // and is left intact so existing callers keep working - but it means filter()
  // must never appear in a read-only path. Use setFilterRegisters(), which takes
  // the target mode as an argument.
  return switchToNormalMode() ? 1 : 0;
}

/**
 * DASHCAM PATCH: the single mode-switch primitive every other path now uses.
 *
 * Two changes against upstream, both load-bearing:
 *
 * 1. modifyRegister, not writeRegister. A full CANCTRL write clears ABAT, OSM,
 *    CLKEN and CLKPRE along with the mode. Clearing OSM matters most: One-Shot
 *    Mode is the only thing that stops endPacket() retrying forever on a bus
 *    with no other node, so a mode switch used to silently disarm the very
 *    protection the caller had just enabled.
 *
 * 2. It polls CANSTAT OPMOD, bounded. CANCTRL echoes the request immediately
 *    while the controller finishes any frame in progress, so upstream's
 *    readback of CANCTRL returns success before the mode has actually changed.
 */
bool MCP2515Class::switchToMode(uint8_t mode)
{
  modifyRegister(REG_CANCTRL, REQOP_MASK, mode);

  for (uint16_t tries = 0; tries < DASHCAM_WAIT_TRIES; tries++) {
    if ((readRegister(REG_CANSTAT) & OPMOD_MASK) == mode) {
      return true;
    }
    delayMicroseconds(DASHCAM_WAIT_STEP_US);
    yield();
  }
  return false;
}

bool MCP2515Class::switchToNormalMode() {
  return switchToMode(MODE_NORMAL);
}

bool MCP2515Class::switchToConfigurationMode()
{
  return switchToMode(MODE_CONFIG);
}

int MCP2515Class::observe()
{
  // DASHCAM PATCH: was writeRegister(REG_CANCTRL, 0x80) - CONFIGURATION mode,
  // with an upstream TODO admitting it. In Configuration mode the controller is
  // off the bus and receives NOTHING, which is indistinguishable from "the
  // gateway bridges no broadcast traffic" and cost this project two sessions of
  // chasing a hardware fault that did not exist. Listen-Only is 0x60.
  return switchToMode(MODE_LISTEN_ONLY) ? 1 : 0;
}

int MCP2515Class::loopback()
{
  return switchToMode(MODE_LOOPBACK) ? 1 : 0;
}

int MCP2515Class::sleep()
{
  // DASHCAM PATCH: was writeRegister(REG_CANCTRL, 0x01). CANCTRL bits 1:0 are
  // CLKPRE, the CLKOUT prescaler - not REQOP. Upstream therefore left the
  // controller in NORMAL mode (REQOP 000) with a divided clock output, and then
  // reported success because the readback matched what it wrote. A node that
  // believes it is asleep while still ACKing every frame on the bus is a
  // considerably worse outcome than a failed sleep. Sleep is REQOP 001 = 0x20.
  return switchToMode(MODE_SLEEP) ? 1 : 0;
}

int MCP2515Class::wakeup()
{
  return switchToMode(MODE_NORMAL) ? 1 : 0;
}

void MCP2515Class::setPins(int cs, int irq)
{
  _csPin = cs;
  _intPin = irq;
}

void MCP2515Class::setSPIFrequency(uint32_t frequency)
{
  _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}

void MCP2515Class::setClockFrequency(long clockFrequency)
{
  _clockFrequency = clockFrequency;
}

void MCP2515Class::dumpImportantRegisters(Stream& out) {
  out.print("TEC: ");
  out.println(readRegister(0x1C), HEX);
  out.print("REC: ");
  out.println(readRegister(0x1D), HEX);
  out.print("CANINTE: ");
  out.println(readRegister(0x2B), HEX);

  out.print("CANINTF: ");
  uint8_t regCANINTF = readRegister(0x2C);
  out.print(regCANINTF, HEX);
  if (regCANINTF & 0x80) {
    out.print(" MERRF");
  }
  if (regCANINTF & 0x40) {
    out.print(" WAKIF");
  }
  if (regCANINTF & 0x20) {
    out.print(" ERRIF");
  }
  if (regCANINTF & 0x10) {
    out.print(" TX2IF");
  }
  if (regCANINTF & 0x08) {
    out.print(" TX1IF");
  }
  if (regCANINTF & 0x04) {
    out.print(" TX0IF");
  }
  if (regCANINTF & 0x02) {
    out.print(" RX1IF");
  }
  if (regCANINTF & 0x01) {
    out.print(" RX0IF");
  }
  out.println();

  out.print("EFLG: ");
  uint8_t regEFLG = readRegister(0x2D);
  out.print(regEFLG, HEX);
  if (regEFLG & 0x80) {
    out.print(" RX1OVR");
  }
  if (regEFLG & 0x40) {
    out.print(" RX0OVR");
  }
  if (regEFLG & 0x20) {
    out.print(" TXBO");
  }
  if (regEFLG & 0x10) {
    out.print(" TXEP");
  }
  if (regEFLG & 0x08) {
    out.print(" RXEP");
  }
  if (regEFLG & 0x04) {
    out.print(" TXWAR");
  }
  if (regEFLG & 0x02) {
    out.print(" RXWAR");
  }
  if (regEFLG & 0x01) {
    out.print(" EWARN");
  }
  out.println();
}

void MCP2515Class::dumpRegisters(Stream& out)
{
  for (int i = 0; i < 128; i++) {
    byte b = readRegister(i);

    out.print("0x");
    if (i < 16) {
      out.print('0');
    }
    out.print(i, HEX);
    out.print(": 0x");
    if (b < 16) {
      out.print('0');
    }
    out.println(b, HEX);
  }
}

void MCP2515Class::reset()
{
  SPI.beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  SPI.transfer(0xc0);
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();

  // From the data sheet:
  // The OST keeps the device in a Reset state for 128 OSC1 clock cycles after
  // the occurrence of a Power-on Reset, SPI Reset, after the assertion of the
  // RESET pin, and after a wake-up from Sleep mode. It should be noted that no
  // SPI protocol operations should be attempted until after the OST has
  // expired.
  // We sleep for 160 cycles to match the old behavior with 16 MHz quartz, and
  // to be on the safe side for 8 MHz devices.
  delayMicroseconds(ceil(160 * 1000000.0 / _clockFrequency));
}

void MCP2515Class::handleInterrupt()
{
  if (readRegister(REG_CANINTF) == 0) {
    return;
  }

  while (parsePacket()) {
    _onReceive(available());
  }
}

uint8_t MCP2515Class::readRegister(uint8_t address)
{
  uint8_t value;

  SPI.beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  SPI.transfer(0x03);
  SPI.transfer(address);
  value = SPI.transfer(0x00);
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();

  return value;
}

void MCP2515Class::modifyRegister(uint8_t address, uint8_t mask, uint8_t value)
{
  SPI.beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  SPI.transfer(0x05);
  SPI.transfer(address);
  SPI.transfer(mask);
  SPI.transfer(value);
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
}

void MCP2515Class::writeRegister(uint8_t address, uint8_t value)
{
  SPI.beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  SPI.transfer(0x02);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
}

void MCP2515Class::onInterrupt()
{
  CAN.handleInterrupt();
}

MCP2515Class CAN;

#endif
