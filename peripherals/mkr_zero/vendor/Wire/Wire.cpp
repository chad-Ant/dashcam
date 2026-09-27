/*
 * TWI/I2C library for Arduino Zero
 * Copyright (c) 2015 Arduino LLC. All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

extern "C" {
#include <string.h>
}

#include <Arduino.h>
#include <wiring_private.h>

#include "Wire.h"

#ifdef DASHCAM_WIRE_INSTRUMENT
// Definitions for the test-only counters declared in Wire.h.  Present only when
// DASHCAM_WIRE_INSTRUMENT is defined, which the production build scripts never do.
volatile uint32_t dashcamWireQty1Total      = 0;
volatile uint32_t dashcamWireQty1Filtered   = 0;
volatile uint8_t  dashcamWireQty1AddrFilter = 0xFFu;   // matches no real address
volatile bool     dashcamWireCoreWaits      = false;   // control experiment only
volatile uint32_t dashcamWireLongestWaitUs  = 0;       // longest completed bounded wait
#endif

using namespace arduino;

// The core's SERCOM class keeps its register pointer private, so the bounded
// waits below find this instance's registers by which core SERCOM object it is.
static Sercom *registersOf(SERCOM *s)
{
  if (s == &sercom0) return SERCOM0;
  if (s == &sercom1) return SERCOM1;
  if (s == &sercom2) return SERCOM2;
  if (s == &sercom3) return SERCOM3;
#if defined(SERCOM4)
  if (s == &sercom4) return SERCOM4;
#endif
#if defined(SERCOM5)
  if (s == &sercom5) return SERCOM5;
#endif
  return nullptr;
}

TwoWire::TwoWire(SERCOM * s, uint8_t pinSDA, uint8_t pinSCL)
{
  this->sercom = s;
  this->_uc_pinSDA=pinSDA;
  this->_uc_pinSCL=pinSCL;
  transmissionBegun = false;
  hw = registersOf(s);
  clockHz = TWI_CLOCK;
  timeouts = 0;
}

void TwoWire::begin(void) {
  //Master Mode
  clockHz = TWI_CLOCK;
  sercom->initMasterWIRE(TWI_CLOCK);
  sercom->enableWIRE();

  pinPeripheral(_uc_pinSDA, g_APinDescription[_uc_pinSDA].ulPinType);
  pinPeripheral(_uc_pinSCL, g_APinDescription[_uc_pinSCL].ulPinType);
}

void TwoWire::begin(uint8_t address, bool enableGeneralCall) {
  //Slave mode
  sercom->initSlaveWIRE(address, enableGeneralCall);
  sercom->enableWIRE();

  pinPeripheral(_uc_pinSDA, g_APinDescription[_uc_pinSDA].ulPinType);
  pinPeripheral(_uc_pinSCL, g_APinDescription[_uc_pinSCL].ulPinType);
}

void TwoWire::setClock(uint32_t baudrate) {
  clockHz = baudrate;
  sercom->disableWIRE();
  sercom->initMasterWIRE(baudrate);
  sercom->enableWIRE();
}

void TwoWire::end() {
  sercom->disableWIRE();
}

size_t TwoWire::requestFrom(uint8_t address, size_t quantity, bool stopBit)
{
  if(quantity == 0)
  {
    return 0;
  }

  size_t byteRead = 0;

#ifdef DASHCAM_WIRE_INSTRUMENT
  // TEST BUILDS ONLY, never defined by the production BuildAndUpload scripts.
  // Counts the transfers that exercise the patched path, so a regression test
  // can assert that a one-byte request genuinely occurred instead of inferring
  // it from a chunk-size calculation that only makes one LIKELY.
  if (quantity == 1) {
    dashcamWireQty1Total++;
    if (address == dashcamWireQty1AddrFilter) dashcamWireQty1Filtered++;
  }
#endif

  rxBuffer.clear();

  if(startBounded(address, WIRE_READ_FLAG) == Xfer::Ok)
  {
    // Read first data. Every read below is bounded (DASHCAM_WIRE_BOUNDED): a
    // byte that never arrives abandons the whole transfer, which reports 0
    // bytes so no caller can mistake a partial buffer for a register image.
    uint8_t data = 0;
    if (readBounded(data) != Xfer::Ok) { rxBuffer.clear(); return 0; }
    rxBuffer.store_char(data);

    // ---- DASHCAM PATCH (see Wire.h, DASHCAM_SAMD_WIRE_REQUESTFROM1_FIX) ------
    // Upstream reads:
    //     bool busOwner;                                   // never initialised
    //     for (byteRead = 1; byteRead < quantity && (busOwner = ...); ++byteRead)
    // The assignment lives inside the loop condition, so a quantity of 1 makes
    // `1 < quantity` false and short-circuits BEFORE busOwner is ever written.
    // The two uses below then read an indeterminate value, which can drop the
    // STOP condition or decrement byteRead to 0 on a transfer that in fact
    // succeeded.  Initialising at the declaration makes the one-byte read
    // defined; the loop still refreshes it every iteration, so multi-byte
    // SEMANTICS are unchanged.
    //
    // Cost, measured rather than asserted: this adds one SERCOM status read at
    // function entry, growing requestFrom(uint8_t,size_t,bool) from 0x90 to 0x96
    // bytes (arm-none-eabi-gcc 7.2.1, -Os).  It adds no I2C bus traffic.  The
    // generated code is NOT identical to upstream — do not claim otherwise.
    bool busOwner = sercom->isBusOwnerWIRE();
    // Connected to slave
    for (byteRead = 1; byteRead < quantity && (busOwner = sercom->isBusOwnerWIRE()); ++byteRead)
    {
      sercom->prepareAckBitWIRE();                          // Prepare Acknowledge
      sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_READ); // Prepare the ACK command for the slave
      if (readBounded(data) != Xfer::Ok) { rxBuffer.clear(); return 0; }  // bounded (see above)
      rxBuffer.store_char(data);                            // Read data and send the ACK
    }
    sercom->prepareNackBitWIRE();                           // Prepare NACK to stop slave transmission
    //sercom->readDataWIRE();                               // Clear data register to send NACK

    if (stopBit && busOwner)
    {
      sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);   // Send Stop unless arbitration was lost
    }

    if (!busOwner)
    {
      byteRead--;   // because last read byte was garbage/invalid
    }
  }

  return byteRead;
}

size_t TwoWire::requestFrom(uint8_t address, size_t quantity)
{
  return requestFrom(address, quantity, true);
}

void TwoWire::beginTransmission(uint8_t address) {
  // save address of target and clear buffer
  txAddress = address;
  txBuffer.clear();

  transmissionBegun = true;
}

// Errors:
//  0 : Success
//  1 : Data too long
//  2 : NACK on transmit of address
//  3 : NACK on transmit of data
//  4 : Other error
uint8_t TwoWire::endTransmission(bool stopBit)
{
  transmissionBegun = false ;

  // Start I2C transmission. Bounded (DASHCAM_WIRE_BOUNDED): a timeout has
  // already reset the SERCOM, so there is no STOP to send - return 4.
  const Xfer started = startBounded( txAddress, WIRE_WRITE_FLAG );
  if ( started == Xfer::Timeout )
  {
    return 4 ;  // Other error: bus wedged, transfer abandoned
  }
  if ( started != Xfer::Ok )
  {
    sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
    return 2 ;  // Address error
  }

  // Send all buffer
  while( txBuffer.available() )
  {
    // Trying to send data
    const Xfer sent = sendBounded( txBuffer.read_char() );
    if ( sent == Xfer::Timeout )
    {
      return 4 ;  // Other error: bus wedged, transfer abandoned
    }
    if ( sent != Xfer::Ok )
    {
      sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
      return 3 ;  // Nack or error
    }
  }

  if (stopBit)
  {
    sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
  }

  return 0;
}

// ---- DASHCAM: bounded master transfers (DASHCAM_WIRE_BOUNDED) ----------------
//
// The core's SERCOM::startTransmissionWIRE, sendDataMasterWIRE and readDataWIRE
// spin on INTFLAG.MB / INTFLAG.SB with no deadline. A slave that holds SCL low
// mid-transfer - a BNO055 whose own controller has wedged, a contact glitch -
// never lets either flag set, and the MKR sat in that loop until the 8 s
// watchdog reset it; the next boot then quarantines the IMU AND the GNSS
// (lib/I2CBus.h, bootAfterHang). Seen on the car, 2026-09-26 20:47.
//
// These are the same register sequences, polled against DASHCAM_WIRE_WAIT_US.
// Deliberate differences from the core, each a fix:
//  - a deadline on every flag wait; on expiry abortTransfer() resets the SERCOM
//    and counts it, and lib/I2CBus.cpp recovers the bus before the next use;
//  - no recursive restart on lost arbitration in the write address phase (the
//    core recursed without bound): lost arbitration is reported as a failure
//    and the caller retries, as it does for any other failed transfer.
// Only the waits that depend on the bus are bounded. SYNCBUSY waits (enable,
// reset, command synchronisation) depend on the peripheral clock alone.

bool TwoWire::waitFlags(uint8_t mask)
{
  const uint32_t t0 = micros();
  while ((hw->I2CM.INTFLAG.reg & mask) == 0u)
  {
    if ((uint32_t)(micros() - t0) > DASHCAM_WIRE_WAIT_US) return false;
  }
#ifdef DASHCAM_WIRE_INSTRUMENT
  const uint32_t took = micros() - t0;
  if (took > dashcamWireLongestWaitUs) dashcamWireLongestWaitUs = took;
#endif
  return true;
}

void TwoWire::abortTransfer(void)
{
  timeouts++;
  // No STOP: on a wedged bus it would not complete. Reset the controller so its
  // own state (bus owner, pending command) is clean; the slave is freed by the
  // GPIO clock-out in i2cBusRecover(), which the changed timeoutCount() triggers.
  sercom->disableWIRE();
  sercom->initMasterWIRE(clockHz);
  sercom->enableWIRE();
}

TwoWire::Xfer TwoWire::startBounded(uint8_t address, SercomWireReadWriteFlag flag)
{
#ifdef DASHCAM_WIRE_INSTRUMENT
  if (dashcamWireCoreWaits)
    return sercom->startTransmissionWIRE(address, flag) ? Xfer::Ok : Xfer::Nack;
#endif
  if (hw == nullptr)   // not a known SERCOM: the core's own path, unbounded
    return sercom->startTransmissionWIRE(address, flag) ? Xfer::Ok : Xfer::Nack;

  // Same early refusal as the core: another master holds the bus, or the last
  // owner never sent its STOP.
  if (!sercom->isBusOwnerWIRE())
  {
    if (sercom->isBusBusyWIRE() || (sercom->isArbLostWIRE() && !sercom->isBusIdleWIRE()))
      return Xfer::Nack;
  }

  // Send start and address (7-bit address + R/W)
  hw->I2CM.ADDR.bit.ADDR = (uint32_t)((address << 0x1ul) | flag);

  if (flag == WIRE_WRITE_FLAG)
  {
    if (!waitFlags(SERCOM_I2CM_INTFLAG_MB)) { abortTransfer(); return Xfer::Timeout; }
    if (!sercom->isBusOwnerWIRE()) return Xfer::Nack;   // arbitration lost
  }
  else
  {
    // SB: address ACKed, first byte in. MB alone: address NACKed (or lost).
    if (!waitFlags(SERCOM_I2CM_INTFLAG_SB | SERCOM_I2CM_INTFLAG_MB)) { abortTransfer(); return Xfer::Timeout; }
    if (!hw->I2CM.INTFLAG.bit.SB)
    {
      hw->I2CM.CTRLB.bit.CMD = 3;   // STOP, as the core does on a NACKed read address
      return Xfer::Nack;
    }
  }

  return hw->I2CM.STATUS.bit.RXNACK ? Xfer::Nack : Xfer::Ok;
}

TwoWire::Xfer TwoWire::sendBounded(uint8_t data)
{
#ifdef DASHCAM_WIRE_INSTRUMENT
  if (dashcamWireCoreWaits)
    return sercom->sendDataMasterWIRE(data) ? Xfer::Ok : Xfer::Nack;
#endif
  if (hw == nullptr)
    return sercom->sendDataMasterWIRE(data) ? Xfer::Ok : Xfer::Nack;

  hw->I2CM.DATA.bit.DATA = data;

  const uint32_t t0 = micros();
  while (!hw->I2CM.INTFLAG.bit.MB)
  {
    // As the core: a bus error or lost arbitration may mean MB never sets.
    if (hw->I2CM.STATUS.bit.BUSERR || hw->I2CM.STATUS.bit.ARBLOST) return Xfer::Nack;
    if ((uint32_t)(micros() - t0) > DASHCAM_WIRE_WAIT_US) { abortTransfer(); return Xfer::Timeout; }
  }
#ifdef DASHCAM_WIRE_INSTRUMENT
  { const uint32_t took = micros() - t0; if (took > dashcamWireLongestWaitUs) dashcamWireLongestWaitUs = took; }
#endif

  return hw->I2CM.STATUS.bit.RXNACK ? Xfer::Nack : Xfer::Ok;
}

TwoWire::Xfer TwoWire::readBounded(uint8_t &out)
{
#ifdef DASHCAM_WIRE_INSTRUMENT
  if (dashcamWireCoreWaits) { out = sercom->readDataWIRE(); return Xfer::Ok; }
#endif
  if (hw == nullptr) { out = sercom->readDataWIRE(); return Xfer::Ok; }

  if (!waitFlags(SERCOM_I2CM_INTFLAG_SB | SERCOM_I2CM_INTFLAG_MB)) { abortTransfer(); return Xfer::Timeout; }
  out = hw->I2CM.DATA.bit.DATA;
  return Xfer::Ok;
}

uint8_t TwoWire::endTransmission()
{
  return endTransmission(true);
}

size_t TwoWire::write(uint8_t ucData)
{
  // No writing, without begun transmission or a full buffer
  if ( !transmissionBegun || txBuffer.isFull() )
  {
    return 0 ;
  }

  txBuffer.store_char( ucData ) ;

  return 1 ;
}

size_t TwoWire::write(const uint8_t *data, size_t quantity)
{
  //Try to store all data
  for(size_t i = 0; i < quantity; ++i)
  {
    //Return the number of data stored, when the buffer is full (if write return 0)
    if(!write(data[i]))
      return i;
  }

  //All data stored
  return quantity;
}

int TwoWire::available(void)
{
  return rxBuffer.available();
}

int TwoWire::read(void)
{
  return rxBuffer.read_char();
}

int TwoWire::peek(void)
{
  return rxBuffer.peek();
}

void TwoWire::flush(void)
{
  // Do nothing, use endTransmission(..) to force
  // data transfer.
}

void TwoWire::onReceive(void(*function)(int))
{
  onReceiveCallback = function;
}

void TwoWire::onRequest(void(*function)(void))
{
  onRequestCallback = function;
}

void TwoWire::onService(void)
{
  if ( sercom->isSlaveWIRE() )
  {
    if(sercom->isStopDetectedWIRE() || 
        (sercom->isAddressMatch() && sercom->isRestartDetectedWIRE() && !sercom->isMasterReadOperationWIRE())) //Stop or Restart detected
    {
      sercom->prepareAckBitWIRE();
      sercom->prepareCommandBitsWire(0x03);

      //Calling onReceiveCallback, if exists
      if(onReceiveCallback)
      {
        onReceiveCallback(available());
      }
      
      rxBuffer.clear();
    }
    else if(sercom->isAddressMatch())  //Address Match
    {
      sercom->prepareAckBitWIRE();
      sercom->prepareCommandBitsWire(0x03);

      if(sercom->isMasterReadOperationWIRE()) //Is a request ?
      {
        txBuffer.clear();

        transmissionBegun = true;

        //Calling onRequestCallback, if exists
        if(onRequestCallback)
        {
          onRequestCallback();
        }
      }
    }
    else if(sercom->isDataReadyWIRE())
    {
      if (sercom->isMasterReadOperationWIRE())
      {
        uint8_t c = 0xff;

        if( txBuffer.available() ) {
          c = txBuffer.read_char();
        }

        transmissionBegun = sercom->sendDataSlaveWIRE(c);
      } else { //Received data
        if (rxBuffer.isFull()) {
          sercom->prepareNackBitWIRE(); 
        } else {
          //Store data
          rxBuffer.store_char(sercom->readDataWIRE());

          sercom->prepareAckBitWIRE(); 
        }

        sercom->prepareCommandBitsWire(0x03);
      }
    }
  }
}

#if WIRE_INTERFACES_COUNT > 0
  /* In case new variant doesn't define these macros,
   * we put here the ones for Arduino Zero.
   *
   * These values should be different on some variants!
   */
  #ifndef PERIPH_WIRE
    #define PERIPH_WIRE          sercom3
    #define WIRE_IT_HANDLER      SERCOM3_Handler
  #endif // PERIPH_WIRE
  arduino::TwoWire Wire(&PERIPH_WIRE, PIN_WIRE_SDA, PIN_WIRE_SCL);

  void WIRE_IT_HANDLER(void) {
    Wire.onService();
  }
#endif

#if WIRE_INTERFACES_COUNT > 1
  arduino::TwoWire Wire1(&PERIPH_WIRE1, PIN_WIRE1_SDA, PIN_WIRE1_SCL);

  void WIRE1_IT_HANDLER(void) {
    Wire1.onService();
  }
#endif

#if WIRE_INTERFACES_COUNT > 2
  arduino::TwoWire Wire2(&PERIPH_WIRE2, PIN_WIRE2_SDA, PIN_WIRE2_SCL);

  void WIRE2_IT_HANDLER(void) {
    Wire2.onService();
  }
#endif

#if WIRE_INTERFACES_COUNT > 3
  arduino::TwoWire Wire3(&PERIPH_WIRE3, PIN_WIRE3_SDA, PIN_WIRE3_SCL);

  void WIRE3_IT_HANDLER(void) {
    Wire3.onService();
  }
#endif

#if WIRE_INTERFACES_COUNT > 4
  arduino::TwoWire Wire4(&PERIPH_WIRE4, PIN_WIRE4_SDA, PIN_WIRE4_SCL);

  void WIRE4_IT_HANDLER(void) {
    Wire4.onService();
  }
#endif

#if WIRE_INTERFACES_COUNT > 5
  arduino::TwoWire Wire5(&PERIPH_WIRE5, PIN_WIRE5_SDA, PIN_WIRE5_SCL);

  void WIRE5_IT_HANDLER(void) {
    Wire5.onService();
  }
#endif
