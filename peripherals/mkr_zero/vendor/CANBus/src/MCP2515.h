// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ARDUINO_ARCH_ESP32

#ifndef MCP2515_H
#define MCP2515_H

#include <SPI.h>

#include "CANController.h"

#define MCP2515_DEFAULT_CLOCK_FREQUENCY 16e6

// Build-time proof that the VENDORED copy of this library is the one being
// compiled, not the unpinned copy in the user's global Arduino libraries
// folder. arduino-cli resolves <CAN.h> from the sketchbook unless an explicit
// --library flag outranks it, and the two copies are byte-similar enough that
// nothing would look wrong if the fixes below silently stopped applying.
// Project headers that depend on those fixes #error on this being absent.
// Mirrors the vendor/Wire marker; see vendor/CANBus/PATCHES.md.
#define DASHCAM_CANBUS_VENDORED_FIXES 1

#if defined(ARDUINO_ARCH_SAMD) && defined(PIN_SPI_MISO) && defined(PIN_SPI_MOSI) && defined(PIN_SPI_SCK) && (PIN_SPI_MISO == 10) && (PIN_SPI_MOSI == 8) && (PIN_SPI_SCK == 9)
// Arduino MKR board: MKR CAN shield CS is pin 3, INT is pin 7
#define MCP2515_DEFAULT_CS_PIN          3
#define MCP2515_DEFAULT_INT_PIN         7
#else
#define MCP2515_DEFAULT_CS_PIN          10
#define MCP2515_DEFAULT_INT_PIN         2
#endif

class MCP2515Class : public CANControllerClass {

public:
  MCP2515Class();
  virtual ~MCP2515Class();

  int begin(long baudRate, bool stayInConfigurationMode);
  virtual int begin(long baudRate) {
    return begin(baudRate, /* stayInConfigurationMode= */ false);
  }

  virtual void end();

  virtual int endPacket();

  virtual int parsePacket();

  virtual void onReceive(void(*callback)(int));

  using CANControllerClass::filter;
  virtual int filter(int id, int mask);

  // mask0 is applied to filter0 and filter1 for RXB0.
  // mask1 is applied to filter2, filter3, filter4, filter5 for RXB1.
  // allowRollover controls whether messages that would otherwise overflow RXB0
  //   should be put in RXB1 instead.
  //
  // See the MCP2515 datasheet for more info.
  //
  // DASHCAM PATCH: targetMode says where to leave the controller. Upstream
  // forced Normal mode, which cancels Listen-Only - so a read-only sniffer
  // could not install filters without going bus-active. Defaults to Normal so
  // existing callers are unaffected. Pass 0x60 for Listen-Only.
  boolean setFilterRegisters(
      uint16_t mask0, uint16_t filter0, uint16_t filter1,
      uint16_t mask1, uint16_t filter2, uint16_t filter3, uint16_t filter4, uint16_t filter5,
      bool allowRollover, uint8_t targetMode = 0x00);

  using CANControllerClass::filterExtended;
  virtual int filterExtended(long id, long mask);
  // TODO: add setFilterRegistersExtended().

  bool switchToNormalMode();
  bool switchToConfigurationMode();
  // DASHCAM PATCH: requests a REQOP mode and confirms it via CANSTAT OPMOD,
  // preserving the other CANCTRL bits (notably One-Shot Mode). Bounded.
  bool switchToMode(uint8_t mode);
  virtual int observe();
  virtual int loopback();
  virtual int sleep();
  virtual int wakeup();

  void setPins(int cs = MCP2515_DEFAULT_CS_PIN, int irq = MCP2515_DEFAULT_INT_PIN);
  void setSPIFrequency(uint32_t frequency);
  void setClockFrequency(long clockFrequency);

  void dumpImportantRegisters(Stream& out);
  void dumpRegisters(Stream& out);

private:
  void reset();

  void handleInterrupt();

  uint8_t readRegister(uint8_t address);
  void modifyRegister(uint8_t address, uint8_t mask, uint8_t value);
  void writeRegister(uint8_t address, uint8_t value);

  static void onInterrupt();

private:
  SPISettings _spiSettings;
  int _csPin;
  int _intPin;
  long _clockFrequency;
};

extern MCP2515Class CAN;

#endif

#endif
