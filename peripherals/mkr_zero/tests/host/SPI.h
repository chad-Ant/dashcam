#ifndef HOST_SPI_STUB_H
#define HOST_SPI_STUB_H 1

/**
 * @file SPI.h
 * @brief Host stand-in for the Arduino SPI API, wired to the MCP2515 model.
 *
 * Every byte lib/CANSniffFunctions.cpp clocks out lands in mcp2515_model.cpp,
 * which decodes the controller's SPI instruction set — READ, WRITE, BIT MODIFY
 * and READ RX BUFFER — against a register file. One transaction is one
 * beginTransaction()/endTransaction() pair; the driver brackets every CS
 * low/high inside one, so the pair stands in for the chip-select edges.
 */

#include <stdint.h>

#define MSBFIRST  1
#define SPI_MODE0 0

class SPISettings {
public:
    SPISettings() {}
    SPISettings(uint32_t, uint8_t, uint8_t) {}
};

class SPIClass {
public:
    void    begin() {}
    void    beginTransaction(const SPISettings &);
    void    endTransaction();
    uint8_t transfer(uint8_t b);
};

extern SPIClass SPI;

#endif // HOST_SPI_STUB_H
