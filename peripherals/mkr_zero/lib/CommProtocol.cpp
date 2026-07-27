#include "CommProtocol.h"

size_t buildFrame(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out, size_t cap)
{
    if (!out) return 0;
    if (len && !payload) return 0;

    const size_t need = COMM_FRAME_OVERHEAD + len;
    if (cap < need) return 0;

    size_t n = 0;
    out[n++] = COMM_SOF;
    out[n++] = COMM_VERSION;
    out[n++] = type;
    out[n++] = len;

    uint16_t crc = 0xFFFF;
    crc = crc16_update(crc, COMM_VERSION);
    crc = crc16_update(crc, type);
    crc = crc16_update(crc, len);
    for (uint8_t i = 0; i < len; ++i) {
        out[n++] = payload[i];
        crc = crc16_update(crc, payload[i]);
    }

    out[n++] = static_cast<uint8_t>(crc & 0xFF);        // CRC low byte first
    out[n++] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    return n;
}

void commRxInit(CommRxState &rx)
{
    rx.phase = RxPhase::WAIT_SOF;
    rx.type  = 0;
    rx.len   = 0;
    rx.idx   = 0;
    rx.crcRx = 0;
    rx.lastByteMs = 0;
}

CommReturnStatus commRxByte(CommRxState &rx, uint8_t b,
                            uint8_t &typeOut, uint8_t *payloadOut, uint8_t payloadCap, uint8_t &lenOut)
{
    if (!payloadOut) return CommReturnStatus::NOK_NULL;

    switch (rx.phase) {
    case RxPhase::WAIT_SOF:
        if (b == COMM_SOF) rx.phase = RxPhase::VER;
        break;

    case RxPhase::VER:
        if (b == COMM_VERSION)   rx.phase = RxPhase::TYPE;
        else if (b == COMM_SOF)  rx.phase = RxPhase::VER;      // consecutive SOF: keep waiting for VER
        else                     rx.phase = RxPhase::WAIT_SOF; // garbage: resync
        break;

    case RxPhase::TYPE:
        rx.type  = b;
        rx.phase = RxPhase::LEN;
        break;

    case RxPhase::LEN:
        rx.len   = b;
        rx.idx   = 0;
        rx.phase = (b == 0) ? RxPhase::CRC_LO : RxPhase::PAYLOAD;
        break;

    case RxPhase::PAYLOAD:
        rx.buf[rx.idx++] = b;
        if (rx.idx >= rx.len) rx.phase = RxPhase::CRC_LO;
        break;

    case RxPhase::CRC_LO:
        rx.crcRx = b;
        rx.phase = RxPhase::CRC_HI;
        break;

    case RxPhase::CRC_HI: {
        rx.crcRx |= static_cast<uint16_t>(b) << 8;
        rx.phase  = RxPhase::WAIT_SOF; // frame consumed regardless of outcome

        uint16_t c = 0xFFFF;
        c = crc16_update(c, COMM_VERSION);
        c = crc16_update(c, rx.type);
        c = crc16_update(c, rx.len);
        for (uint16_t i = 0; i < rx.len; ++i) c = crc16_update(c, rx.buf[i]);

        if (c != rx.crcRx) return CommReturnStatus::NOK_CRC;
        if (rx.len > payloadCap) return CommReturnStatus::NOK_OVERFLOW;

        typeOut = rx.type;
        lenOut  = rx.len;
        memcpy(payloadOut, rx.buf, rx.len);
        return CommReturnStatus::FRAME_READY;
    }
    }

    return CommReturnStatus::NO_DATA;
}

CommReturnStatus pollFrame(CommRxState &rx, Stream &in,
                           uint8_t &typeOut, uint8_t *payloadOut, uint8_t payloadCap, uint8_t &lenOut,
                           size_t maxBytes)
{
    // Abort a stalled partial frame so a truncated transmission cannot swallow
    // the bytes of the next valid frame.
    if (rx.phase != RxPhase::WAIT_SOF && (millis() - rx.lastByteMs) > COMM_RX_TIMEOUT_MS) {
        commRxInit(rx);
    }

    // Bounded by maxBytes so a flood of non-SOF noise can't scan indefinitely.
    for (size_t processed = 0; processed < maxBytes && in.available() > 0; ++processed) {
        const uint8_t b = static_cast<uint8_t>(in.read());
        rx.lastByteMs = millis();
        const CommReturnStatus r = commRxByte(rx, b, typeOut, payloadOut, payloadCap, lenOut);
        if (r != CommReturnStatus::NO_DATA) return r; // one frame (or error) per call
    }
    return CommReturnStatus::NO_DATA;
}
