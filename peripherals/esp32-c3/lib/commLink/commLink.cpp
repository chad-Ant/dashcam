#include "commLink.h"

void CommLink::begin(unsigned long baud, int8_t rxPin, int8_t txPin, HardwareSerial &uart)
{
    uart_ = &uart;
    uart_->begin(baud, SERIAL_8N1, rxPin, txPin);
    commRxInit(rx_);
    hasData_  = false;
    pong_     = false;
    lastRxMs_ = 0;
}

bool CommLink::sendCmd(uint8_t type)
{
    if (!uart_) return false;
    uint8_t frame[COMM_FRAME_OVERHEAD]; // zero-length-payload command frames
    size_t n = buildFrame(type, nullptr, 0, frame, sizeof(frame));
    if (n == 0) return false;
    // Non-blocking: only write when the whole frame fits, and confirm the byte count.
    if (uart_->availableForWrite() < static_cast<int>(n)) return false;
    return uart_->write(frame, n) == n;
}

bool CommLink::requestOnce() { return sendCmd(CMD_GET_ONCE); }
bool CommLink::startStream() { return sendCmd(CMD_START_STREAM); }
bool CommLink::stopStream()  { return sendCmd(CMD_STOP_STREAM); }
bool CommLink::ping()        { return sendCmd(CMD_PING); }

bool CommLink::poll()
{
    if (!uart_) return false;

    bool    fresh = false;
    uint8_t type = 0, len = 0;
    uint8_t payload[COMM_MAX_PAYLOAD];

    // Bounded to a fixed frame budget per call so a continuous inbound stream
    // cannot monopolise the loop; leftover bytes are picked up next poll().
    for (uint8_t serviced = 0; serviced < COMMLINK_MAX_FRAMES_PER_POLL; ++serviced) {
        CommReturnStatus r = pollFrame(rx_, *uart_, type, payload, sizeof(payload), len);
        if (r == CommReturnStatus::NO_DATA) break;
        if (r != CommReturnStatus::FRAME_READY) continue; // bad CRC / overflow: keep draining

        switch (type) {
        case MSG_TELEMETRY:
            if (len == sizeof(TelemetryPayload)) {
                memcpy(&latest_, payload, sizeof(TelemetryPayload));
                hasData_  = true;
                lastRxMs_ = millis();
                fresh     = true;
            }
            break;
        case MSG_PONG:
            pong_ = true;
            break;
        default:
            break; // NACK / unknown: ignore
        }
    }

    return fresh;
}
