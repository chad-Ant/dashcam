#ifndef HOST_MCP2515_MODEL_H
#define HOST_MCP2515_MODEL_H 1

/**
 * @file mcp2515_model.h
 * @brief Test control for the MCP2515 model behind SPI.h and CAN.h.
 *
 * ── WHY A MODEL AND NOT CANNED CANINTF BITS ──────────────────────────────────
 * The defect these tests exist for lives in ACCEPTANCE FILTERING: with a map's
 * filters programmed, a bus full of other IDs delivers nothing, and a probe
 * waiting for "the first frame" waits forever. A stub that hands the driver
 * frames directly would deliver them regardless of the filters and hide exactly
 * that. So frames enter here from the BUS side and are accepted or rejected by
 * the masks and filters the driver actually programmed, per the datasheet
 * (DS20001801, section 4): RXB0 against mask 0 and filters 0-1, RXB1 against
 * mask 1 and filters 2-5, RXM = 11 accepting everything, BUKT rolling a full
 * RXB0 over into RXB1, and nothing received in Configuration mode.
 *
 * Standard and extended identifiers, data and remote frames, laid into the
 * buffer registers as the controller lays them: a remote frame stores no data,
 * so the data registers keep the previous frame's bytes. READ STATUS is
 * modelled for the drain ISR. Mode changes complete instantly; the driver's
 * bounded CANSTAT poll passes on its first read, which is the uninteresting
 * half of that loop.
 *
 * Every transaction and byte is reported to can_stream_hw_stub.cpp, which is
 * how a main-loop transaction made with the drain interrupt unmasked is caught.
 */

#include <stdint.h>

/// Power-on state: Configuration mode, empty buffers, masks and filters zero.
void fakeCanReset();

/**
 * @brief One standard data frame on the bus.
 * @return true when a receive buffer took it; false when acceptance filtering
 *         rejected it, both eligible buffers were full, or the controller was
 *         not receiving.
 */
bool fakeCanFrame(uint16_t id, uint8_t dlc = 8, const uint8_t *data = nullptr);

/**
 * @brief Any frame on the bus: standard or extended, data or remote.
 * @param rawDlc  The 4-bit DLC field as sent, 0-15; above 8 still means 8 bytes.
 * @return as fakeCanFrame().
 */
bool fakeCanFrameEx(uint32_t id, bool ext, bool rtr, uint8_t rawDlc, const uint8_t *data);

/// MISO stuck at @p value for every byte — a missing or unpowered shield.
void fakeCanStickMiso(bool stuck, uint8_t value = 0xFF);

/// Direct register access, for tests that set up a state no frame produces.
uint8_t fakeCanReg(uint8_t addr);
void    fakeCanSetReg(uint8_t addr, uint8_t value);

/// Whether the filters now programmed would accept @p id (buffers aside).
bool fakeCanAccepts(uint16_t id);

/// CANSTAT OPMOD bits: 0x00 Normal, 0x60 Listen-Only, 0x80 Configuration.
uint8_t fakeCanOpMode();

/// Make setFilterRegisters() fail, as the real one does when its request for
/// Configuration mode never completes: nothing is written and OPMOD stays where
/// it was. The driver's raw SPI mode writes still work, so a test that then
/// finds Configuration knows the DRIVER parked the controller.
void fakeCanRefuseFilterWrites(bool refuse);

uint32_t fakeCanRejected();    ///< Frames acceptance filtering threw away.
uint32_t fakeCanOverflowed();  ///< Frames accepted with no free buffer.

/// True if the controller was ever in Normal (bus-active) since reset.
bool fakeCanWasEverNormal();

#endif // HOST_MCP2515_MODEL_H
