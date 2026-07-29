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

#ifndef TwoWire_h
#define TwoWire_h

/*
 * ---------------------------------------------------------------------------
 * VENDORED AND PATCHED COPY - see peripherals/mkr_zero/vendor/Wire/README.md
 * ---------------------------------------------------------------------------
 * Upstream: arduino/ArduinoCore-samd, libraries/Wire, as shipped in the
 * Arduino SAMD Boards core version 1.8.14 (the version this project pins).
 * Licence unchanged: LGPL 2.1 or later, notice above retained verbatim.
 *
 * Sole modification: TwoWire::requestFrom() initialises `busOwner` at its
 * declaration.  Upstream leaves it indeterminate on the quantity==1 path, which
 * is undefined behaviour on every single-byte register read.  See the comment
 * at the patch site in Wire.cpp.
 *
 * This marker is how the project proves it compiled against THIS copy and not
 * the core's stock one.  Shared I2C code #errors when it is absent, so an
 * accidental build without the vendor --library path fails loudly instead of
 * silently reintroducing the bug.  Do not define it anywhere else.
 */
#define DASHCAM_SAMD_WIRE_REQUESTFROM1_FIX 1

#ifdef DASHCAM_WIRE_INSTRUMENT
#include <stdint.h>
/*
 * Test-only transfer counters. Defined only when DASHCAM_WIRE_INSTRUMENT is set,
 * which no production build script does — the counters and their increments
 * compile out entirely otherwise.
 *
 * They exist because a test cannot otherwise prove a one-byte requestFrom()
 * actually happened. Shrinking a driver's chunk size only makes a singleton
 * LIKELY (it depends on the pending byte count modulo the chunk size), and
 * "the packets still parsed" is evidence of nothing in particular.
 */
extern volatile uint32_t dashcamWireQty1Total;      ///< All quantity==1 requests.
extern volatile uint32_t dashcamWireQty1Filtered;   ///< Those to dashcamWireQty1AddrFilter.
extern volatile uint8_t  dashcamWireQty1AddrFilter; ///< 7-bit address to watch.
#endif

#include "api/HardwareI2C.h"
#include "variant.h"
#include "SERCOM.h"

 // WIRE_HAS_END means Wire has end()
#define WIRE_HAS_END 1

namespace arduino {

class TwoWire : public HardwareI2C
{
  public:
    TwoWire(SERCOM *s, uint8_t pinSDA, uint8_t pinSCL);
    void begin();
    void begin(uint8_t address, bool enableGeneralCall);
    void begin(uint8_t address) {
        begin(address, false);
    }
    void end();
    void setClock(uint32_t);

    void beginTransmission(uint8_t);
    uint8_t endTransmission(bool stopBit);
    uint8_t endTransmission(void);

    size_t requestFrom(uint8_t address, size_t quantity, bool stopBit);
    size_t requestFrom(uint8_t address, size_t quantity);

    size_t write(uint8_t data);
    size_t write(const uint8_t * data, size_t quantity);

    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
    virtual void flush(void);
    void onReceive(void(*)(int));
    void onRequest(void(*)(void));

    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    using Print::write;

    void onService(void);

  private:
    SERCOM * sercom;
    uint8_t _uc_pinSDA;
    uint8_t _uc_pinSCL;

    bool transmissionBegun;

    // RX Buffer
    arduino::RingBufferN<256> rxBuffer;

    //TX buffer
    arduino::RingBufferN<256> txBuffer;
    uint8_t txAddress;

    // Callback user functions
    void (*onRequestCallback)(void);
    void (*onReceiveCallback)(int);

    // TWI clock frequency
    static const uint32_t TWI_CLOCK = 100000;
};

}

#if WIRE_INTERFACES_COUNT > 0
  extern arduino::TwoWire Wire;
#endif
#if WIRE_INTERFACES_COUNT > 1
  extern arduino::TwoWire Wire1;
#endif
#if WIRE_INTERFACES_COUNT > 2
  extern arduino::TwoWire Wire2;
#endif
#if WIRE_INTERFACES_COUNT > 3
  extern arduino::TwoWire Wire3;
#endif
#if WIRE_INTERFACES_COUNT > 4
  extern arduino::TwoWire Wire4;
#endif
#if WIRE_INTERFACES_COUNT > 5
  extern arduino::TwoWire Wire5;
#endif

#endif
