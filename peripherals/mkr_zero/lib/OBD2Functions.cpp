#include <CAN.h>
#include <math.h>

#include "OBD2Functions.h"
#include "TimerFunctions.h"
#include "MathFunctions.h"

/****** for debug purposes
void writeRegister(uint8_t address, uint8_t value)
{
  SPI.beginTransaction(SPICfg);
  digitalWrite(MCP2515_DEFAULT_CS_PIN, LOW);
  SPI.transfer(0x02);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(MCP2515_DEFAULT_CS_PIN, HIGH);
  SPI.endTransaction();
}

uint8_t readRegister(uint8_t address)
{
  uint8_t value;

  SPI.beginTransaction(SPICfg);
  digitalWrite(MCP2515_DEFAULT_CS_PIN, LOW);
  SPI.transfer(0x03);
  SPI.transfer(address);
  value = SPI.transfer(0x00);
  digitalWrite(MCP2515_DEFAULT_CS_PIN, HIGH);
  SPI.endTransaction();

  return value;
}

bool checkCANModule(){
    writeRegister(0x0f, 0x80);          //0x0f = REG_CANCTRL
    if (readRegister(0x0f) != 0x80){
        return false;
    }
    return true;
}
******/
CANReturnStatus getSupportedPIDs(OBD2Config &config, unsigned long timeoutInterval){
    uint32_t PIDs = 0;
    uint8_t tempPID = 0x00;

    for (uint8_t i = 0x00; i < sizeof(config.supportedPIDs) / sizeof(config.supportedPIDs[0]); i++){
        // Reuse the checked transmit path (returns NOK_TX on a failed send).
        if (sendS1Command(config, static_cast<OBD2_S1Command>(tempPID)) != CANReturnStatus::OK)
            return CANReturnStatus::NOK_TX;

        unsigned long thisRun = millis();
        while (true) {
            if (isTimeout(timeoutInterval, thisRun)) return CANReturnStatus::NOK_TIMEOUT;
            if (CAN.parsePacket() == 0) continue;

            // Require the whole frame (PCI + service + PID + 4-byte bitmap) before reading;
            // otherwise CAN.read() returns -1 (0xFF) and corrupts the bitmap.
            if (CAN.available() < 7) { while (CAN.available()) CAN.read(); continue; }

            uint8_t len  = (uint8_t)CAN.read();
            uint8_t svc  = (uint8_t)CAN.read();
            uint8_t pid  = (uint8_t)CAN.read();
            if (len < 6 || svc != 0x41 || pid != tempPID) {
                while (CAN.available()) CAN.read();
                continue;
            }
            break;
        }

        PIDs = 0;
        for (int j = 0; j < 4; j++){
            PIDs <<= 8;
            PIDs |= (uint8_t)CAN.read();
        }

        config.supportedPIDs[i] = PIDs;
        // The LSB of each range's bitmap flags whether the *next* range is supported;
        // if it is clear, querying further ranges is pointless — stop early.
        if ((PIDs & 0x1u) == 0u) break;
        tempPID += 0x20;
    }
    return CANReturnStatus::OK;
}

// Enable One-Shot Mode (CANCTRL.OSM) via a direct SPI bit-modify — the arduino-CAN
// API does not expose it. In OSM the MCP2515 makes a single transmit attempt and then
// clears TXREQ instead of retransmitting forever, so MCP2515Class::endPacket() cannot
// block indefinitely on a dead bus (disconnected ECU, wrong bitrate, bus-off).
// Returns true only if the OSM bit reads back set — this bit is the anti-hang
// guard, so initialization must fail safely when it does not latch.
static bool mcp2515EnableOneShot(int csPin)
{
    SPI.beginTransaction(SPICfg);
    digitalWrite(csPin, LOW);
    SPI.transfer(0x05);   // BIT MODIFY
    SPI.transfer(0x0F);   // CANCTRL register
    SPI.transfer(0x08);   // mask  = OSM bit
    SPI.transfer(0x08);   // value = OSM set
    digitalWrite(csPin, HIGH);
    SPI.endTransaction();

    SPI.beginTransaction(SPICfg);
    digitalWrite(csPin, LOW);
    SPI.transfer(0x03);   // READ
    SPI.transfer(0x0F);   // CANCTRL register
    uint8_t canctrl = SPI.transfer(0x00);
    digitalWrite(csPin, HIGH);
    SPI.endTransaction();

    return (canctrl & 0x08) != 0;   // OSM latched?
}

CANReturnStatus initializeOBD2(OBD2Config &config, CAN_TxAddress TxAddr, CAN_RxAddress RxAddr, int csPin, int irqPin, bool stayInConfigurationMode){
    // Zero the supported-PID bitmap so a later discovery failure can't expose stale bits.
    for (uint8_t i = 0; i < sizeof(config.supportedPIDs) / sizeof(config.supportedPIDs[0]); ++i) {
        config.supportedPIDs[i] = 0;
    }

    CAN.setPins(csPin, irqPin);
    CAN.setClockFrequency(MCP2515_OSC_FREQ);   // MUST match the module crystal (DataDictionary.h)
    if (!CAN.begin(CAN_BAUDRATE_DEFAULT, stayInConfigurationMode)) {
        return CANReturnStatus::NOK_INIT_FAILED;
    }
    config.TxAddress = TxAddr;
    config.RxAddress = RxAddr;
    //CANReturnStatus status = getSupportedPIDs(config); //currently bugged
    //if (status != CANReturnStatus::OK) return status;

    // Configure-only bring-up stops here, still off the bus.
    //
    // The boot path uses it because everything below ends in Normal mode:
    // CAN.filter() does, and so does the default begin(). That left the
    // controller ACK-capable for the whole window between bring-up and the
    // canSetMode(SNIFF) that follows - short, but a window in which a node
    // whose bit timing nothing had yet confirmed was participating in a live
    // vehicle bus. canSetMode() programs its own filters and sets Normal+OSM in
    // one atomic write for OBD2, so nothing here is lost by deferring to it.
    if (stayInConfigurationMode) {
        resetOBD2Poll();
        return CANReturnStatus::OK;
    }

    if (!CAN.filter(config.RxAddress)) return CANReturnStatus::NOK_INIT_FAILED;
    // OSM caps retransmission so a missing ACK cannot make the controller retry
    // forever — verify it latched. It does NOT bound endPacket(); that bound is
    // the explicit deadline in the vendored driver.
    if (!mcp2515EnableOneShot(csPin)) return CANReturnStatus::NOK_STATUS_BAD;

    // Called from INSIDE init, not left to the caller. resetOBD2Poll() was dead
    // code: after a re-init txFailures was still at OBD2_TX_FAIL_LIMIT, so the
    // very next failed transmit re-declared link loss instead of granting a
    // fresh budget — a re-init that could never actually recover. Putting it
    // here makes it impossible to forget at a future call site.
    resetOBD2Poll();
    return CANReturnStatus::OK;
}

bool isCommandSupported(const OBD2Config &config, const OBD2_S1Command command){
    uint8_t cmdInt = static_cast<uint8_t>(command);
    // PID 0 is reserved; above 0xE0 exceeds the 7-group (224-PID) supportedPIDs array.
    if (cmdInt == 0 || cmdInt > 0xE0) return false;

    int PIDGroup = (cmdInt - 1) >> 5; // PIDs 1-32 → group 0; 33-64 → group 1; etc.
    int PIDIndex = (cmdInt - 1) % 32; // position within group, 0-based
    return ((config.supportedPIDs[PIDGroup] >> (31 - PIDIndex)) & 1) == 1;
}

CANReturnStatus sendS1Command(OBD2Config &config, const OBD2_S1Command command){
    // Check every CAN operation. endPacket() is bounded by an explicit deadline
    // in the vendored driver - NOT by One-Shot Mode, which this comment used to
    // claim. OSM limits retransmission after an attempt; it does nothing about
    // waiting for an idle bus, so the unpatched call could spin forever.
    if (!CAN.beginPacket(config.TxAddress, 8)) return CANReturnStatus::NOK_TX;
    if (CAN.write(0x02)    != 1) return CANReturnStatus::NOK_TX; // additional data byte count
    if (CAN.write(0x01)    != 1) return CANReturnStatus::NOK_TX; // service 01 (current data)
    if (CAN.write(command) != 1) return CANReturnStatus::NOK_TX; // requested PID
    if (CAN.endPacket()    != 1) return CANReturnStatus::NOK_TX;
    return CANReturnStatus::OK;
}

CANReturnStatus receiveS1Command(OBD2Config &config, uint8_t *outputBuffer, byte bufferSize, OBD2_S1Command &commandRx, unsigned long timeout){
    if (!outputBuffer) return CANReturnStatus::NOK_NULL_BUFFER;
    if (config.RxAddress != OBD2_RX_ECM_1) return CANReturnStatus::NOK_NOT_S1_CFG;

    unsigned long timeStart = millis();
    while (true) {
        if (isTimeout(timeout, timeStart)) return CANReturnStatus::NOK_TIMEOUT;
        if (CAN.parsePacket() == 0) continue;

        // Require the whole frame (PCI + service + PID + payload) to have physically
        // arrived; otherwise CAN.read() returns -1 (0xFF) and decodes as garbage.
        if (CAN.available() < static_cast<int>(3u + bufferSize)) {
            while (CAN.available()) CAN.read();
            continue;
        }

        uint8_t RxLength = (uint8_t)CAN.read() - 2;
        bool isS1Response = CAN.read() == 0x41;
        commandRx = static_cast<OBD2_S1Command>(CAN.read());
        if (RxLength != bufferSize || !isS1Response) {
            while (CAN.available()) CAN.read();
            continue;
        }
        break;
    }

    for (int i = 0; i < bufferSize; i++){
        outputBuffer[i] = (uint8_t)CAN.read();
    }
    return CANReturnStatus::OK;
}
// ---- Non-blocking round-robin polling state machine ----

static const OBD2_S1Command kPollPipeline[] = {
    RPM, SPEED, ENGINE_TEMP, FUEL_LVL, FUEL_RATE,
    THROTTLE_POSN, ENGINE_LOAD, AIR_PRES, GEAR_RTIO, ODOMETER
};
static const uint8_t kPollBytes[] = {
    RPM_T, SPEED_T, ENGINE_TEMP_T, FUEL_LVL_T, FUEL_RATE_T,
    THROTTLE_POSN_T, ENGINE_LOAD_T, AIR_PRES_T, GEAR_RTIO_T, ODOMETER_T
};
static const uint8_t kPollCount = sizeof(kPollPipeline) / sizeof(kPollPipeline[0]);

static uint8_t       pollIdx       = 0;
static bool          awaitingReply = false;
static unsigned long requestedAt   = 0;
static uint16_t      txFailures    = 0;   // consecutive TX failures (link-loss detector)
static unsigned long lastTxAttempt = 0;   // for retry backoff after a failed transmit
static unsigned long lastReplyMs   = 0;   // last DECODED ECU reply — see isOBD2LinkLost()
static bool          silenceArmed  = false; // true once one reply has ever been decoded
static unsigned long firstTxMs     = 0;   // first transmit since reset — see isOBD2LinkLost()

void initOBD2Data(OBD2Data &data)
{
    data.rpm = data.speed = data.coolantTemp = data.fuelLevel = data.fuelRate =
    data.throttle = data.engineLoad = data.airPressure = data.gear = data.gearRatio = data.odo = NAN;
    data.lastUpdateMs = 0;
    for (uint8_t i = 0; i < OBD2_FIELD_COUNT; ++i) data.fieldMs[i] = 0;
}

void expireStaleOBD2Fields(OBD2Data &data, uint32_t maxAgeMs)
{
    const uint32_t now = millis();

    // Table rather than eleven if-statements, so adding a field cannot silently
    // skip its expiry. A field that is never expired is worse than one that is
    // never published: it is published, and believed.
    float *const values[OBD2_FIELD_COUNT] = {
        &data.rpm, &data.speed, &data.coolantTemp, &data.fuelLevel,
        &data.fuelRate, &data.throttle, &data.engineLoad, &data.airPressure,
        &data.gear, &data.gearRatio, &data.odo
    };

    for (uint8_t i = 0; i < OBD2_FIELD_COUNT; ++i) {
        // Stamp 0 means never seen. Those are already NAN from initOBD2Data()
        // and must not be treated as "infinitely old but once valid".
        if (data.fieldMs[i] == 0) continue;
        if ((now - data.fieldMs[i]) > maxAgeMs) *values[i] = NAN;
    }
}

static void storeReading(OBD2_S1Command pid, const uint8_t *buf, OBD2Data &data)
{
    const uint32_t now = millis();
    switch (pid) {
        case RPM:
            data.rpm         = ((float)buf[0] * 256.0f + (float)buf[1]) * 0.25f;
            data.fieldMs[OBD2F_RPM] = now;                                            break;
        case SPEED:
            data.speed       = (float)buf[0];
            data.fieldMs[OBD2F_SPEED] = now;                                          break;
        case ENGINE_TEMP:
            data.coolantTemp = (float)buf[0] - 40.0f;
            data.fieldMs[OBD2F_COOLANT] = now;                                        break;
        case FUEL_LVL:
            data.fuelLevel   = interpolate((float)buf[0], 255.0f, 0.0f);
            data.fieldMs[OBD2F_FUEL_LVL] = now;                                       break;
        case FUEL_RATE:
            data.fuelRate    = ((float)buf[0] * 256.0f + (float)buf[1]) * 0.05f;
            data.fieldMs[OBD2F_FUEL_RATE] = now;                                      break;
        case THROTTLE_POSN:
            data.throttle    = interpolate((float)buf[0], 255.0f, 0.0f);
            data.fieldMs[OBD2F_THROTTLE] = now;                                       break;
        case ENGINE_LOAD:
            data.engineLoad  = interpolate((float)buf[0], 255.0f, 0.0f);
            data.fieldMs[OBD2F_ENGINE_LOAD] = now;                                    break;
        case AIR_PRES:
            data.airPressure = (float)buf[0];
            data.fieldMs[OBD2F_AIR_PRES] = now;                                       break;
        case GEAR_RTIO:
            // J1979 PID 0xA4, 4 bytes: A = support bits, B = actual gear,
            // C:D = ratio / 1000. One response carries both, which is why 0xA3
            // is gone rather than replaced - see the enum comment.
            //
            // Byte A says which of the two the ECU actually supports, and it is
            // honoured per field rather than ignored: an unsupported field is
            // returned as a defined value (usually zero), so publishing it
            // unconditionally reported "gear 0, ratio 0.000" as a measurement on
            // any vehicle that implements only one half of the PID.
            if (buf[0] & 0x01u) {
                data.gear      = (float)buf[1];
                data.fieldMs[OBD2F_GEAR] = now;
            }
            if (buf[0] & 0x02u) {
                data.gearRatio = ((float)buf[2] * 256.0f + (float)buf[3]) * 0.001f;
                data.fieldMs[OBD2F_GEAR_RATIO] = now;
            }
            break;
        case ODOMETER:
            data.odo         = (float)(((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                                       ((uint32_t)buf[2] << 8)  |  (uint32_t)buf[3]) * 0.1f;
            data.fieldMs[OBD2F_ODO] = now;                                            break;
        default: break;
    }
}

void resetOBD2Poll()
{
    pollIdx       = 0;
    awaitingReply = false;
    txFailures    = 0;
    lastTxAttempt = 0;
    lastReplyMs   = 0;
    silenceArmed  = false;
    firstTxMs     = 0;
}

bool obd2EverReplied()
{
    // silenceArmed is set by the first decoded reply and cleared only by
    // resetOBD2Poll(), so it answers "has this link ever worked" rather than
    // "is it working now" — which is the question a bring-up needs.
    return silenceArmed;
}

bool isOBD2LinkLost()
{
    // Two independent ways the link can be gone, and they are NOT the same
    // question. txFailures covers "the frame never got onto the wire". The
    // silence window covers the far more likely vehicle case: the frame went
    // out cleanly, some other node ACKed it because a live bus always has one,
    // and the ECU we are actually talking to said nothing at all. Only the
    // second catches a wrong filter, a wrong response ID, or a dead ECM.
    if (txFailures >= OBD2_TX_FAIL_LIMIT) return true;
    if (silenceArmed) {
        return (millis() - lastReplyMs) > OBD2_RX_SILENCE_MS;
    }
    // No reply has EVER been decoded. This used to return false unconditionally
    // here, which left a link that never worked looking permanently healthy: on
    // a live bus some other node ACKs every request, so txFailures stays at zero
    // forever and the silence window — armed only by a first reply that never
    // comes — never gets to fire. The node would sit at "OBD2 up" indefinitely
    // publishing nothing, which is the most expensive possible way to be broken.
    //
    // Measured from the first transmit rather than from init, because before a
    // request goes out there is genuinely nothing to be silent about. The window
    // is generous: an ECU may take a moment after ignition to answer.
    if (firstTxMs != 0u && (millis() - firstTxMs) > OBD2_NO_REPLY_MS) return true;
    return false;
}

// Sends the request for pipeline entry [idx] with TX-failure bookkeeping that drives
// the backoff throttle (retry-storm guard) and isOBD2LinkLost() (runtime link loss).
static CANReturnStatus fireRequest(OBD2Config &config, uint8_t idx)
{
    lastTxAttempt = millis();
    CANReturnStatus st = sendS1Command(config, kPollPipeline[idx]);
    // NOTE: OK here means the frame was ACKed by SOME node, which on a vehicle
    // bus proves only that the bus is alive - not that the ECU we addressed is.
    // That is why clearing txFailures is no longer sufficient evidence of a
    // healthy link; see isOBD2LinkLost().
    if (st == CANReturnStatus::OK) {
        txFailures = 0;
        // Starts the no-reply deadline in isOBD2LinkLost(). Stamped on the first
        // request that actually reached the wire, not on init: a request that
        // failed to transmit is already covered by txFailures, and starting the
        // clock on it would blame the ECU for a fault below it.
        if (firstTxMs == 0u) firstTxMs = lastTxAttempt;
    } else if (txFailures < 0xFFFFu) {
        ++txFailures;
    }
    return st;
}

/**
 * @brief Fires the next request once the pacing floor has elapsed.
 *
 * Returns false without transmitting if it is too soon, leaving the caller
 * un-armed so the next loop() pass retries. Never busy-waits.
 */
static bool fireRequestPaced(OBD2Config &config, uint8_t idx)
{
    if ((millis() - lastTxAttempt) < OBD2_MIN_REQUEST_GAP_MS) return false;
    return fireRequest(config, idx) == CANReturnStatus::OK;
}

bool tickOBD2(OBD2Config &config, OBD2Data &data)
{
    // First call, or after resetOBD2Poll(): fire the initial request. After a TX
    // failure, throttle retries (OBD2_TX_BACKOFF_MS) so a dead bus can't storm the
    // SPI/CAN interface with thousands of failed transactions per second.
    if (!awaitingReply) {
        if (txFailures > 0 && (millis() - lastTxAttempt) < OBD2_TX_BACKOFF_MS) return false;
        if (fireRequestPaced(config, pollIdx)) {
            requestedAt   = millis();
            awaitingReply = true;
        }
        return false;
    }

    // Read BEFORE testing the timeout. The old order tested the clock first, so
    // a reply that had already physically arrived and was sitting in the RX
    // buffer got thrown away whenever this loop was serviced late - and because
    // the timeout path also advances pollIdx, the poller then ran one PID out of
    // phase with the ECU and every subsequent reply failed its PID check too.
    // A frame in hand is evidence; the clock is not.
    const int rxDlc = CAN.parsePacket();
    if (rxDlc == 0 && CAN.packetId() == -1) {
        // Genuinely nothing waiting. parsePacket() returns the DLC, so a valid
        // DLC-0 frame also reads as 0 - packetId() is what distinguishes them.
        if (isTimeout(OBD2_TICK_TIMEOUT_MS, requestedAt)) {
            pollIdx = (pollIdx + 1u) % kPollCount;
            if (fireRequestPaced(config, pollIdx)) requestedAt = millis();
            else                                   awaitingReply = false;
        }
        return false;
    }

    // Frame-level checks the hardware filter is not a substitute for. The filter
    // narrows what reaches the buffer; it does not prove what did. A truncated
    // filter (which upstream's did produce), a bus with 11-bit and 29-bit
    // traffic, or a remote-transmission request would all otherwise be decoded
    // as if they were the ECU's answer.
    if (CAN.packetId() != static_cast<long>(config.RxAddress) ||
        CAN.packetExtended() || CAN.packetRtr()) {
        while (CAN.available()) CAN.read();
        return false;
    }

    // Require the whole single-frame response (PCI + service + PID + data) to have
    // physically arrived before reading. Otherwise a short frame lets CAN.read()
    // return -1 (0xFF), which would decode as a plausible high measurement.
    const uint8_t expectedBytes = kPollBytes[pollIdx];
    if (CAN.available() < static_cast<int>(3u + expectedBytes)) {
        while (CAN.available()) CAN.read();
        return false;
    }

    // Decode frame header.
    uint8_t        lenByte = static_cast<uint8_t>(CAN.read());
    uint8_t        dataLen = (lenByte >= 2u) ? lenByte - 2u : 0u;
    bool           isSvc1  = (static_cast<uint8_t>(CAN.read()) == 0x41u);
    OBD2_S1Command rxPID   = static_cast<OBD2_S1Command>(CAN.read());

    if (!isSvc1 || rxPID != kPollPipeline[pollIdx] || dataLen != expectedBytes) {
        while (CAN.available()) CAN.read(); // drain unrelated frame, keep waiting
        return false;
    }

    // Read payload into local buffer.
    uint8_t buf[4] = {};
    for (uint8_t i = 0; i < expectedBytes; ++i) buf[i] = static_cast<uint8_t>(CAN.read());
    while (CAN.available()) CAN.read();

    storeReading(rxPID, buf, data);
    data.lastUpdateMs = millis();
    // The ECU itself answered - the only evidence that clears the silence
    // watchdog. A clean transmit does not, because any node can ACK.
    lastReplyMs  = data.lastUpdateMs;
    silenceArmed = true;
    pollIdx = (pollIdx + 1u) % kPollCount;

    // Fire the next request; only remain "awaiting" if it actually went out.
    // Paced: if the floor has not elapsed we simply drop out un-armed and the
    // next loop() pass sends it, rather than blocking here for the remainder.
    if (fireRequestPaced(config, pollIdx)) requestedAt = millis();
    else                                   awaitingReply = false;
    return true;
}