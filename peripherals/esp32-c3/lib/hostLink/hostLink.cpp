#include "hostLink.h"

#include <string.h>

void HostLink::begin()
{
    // With USB CDC On Boot enabled the core has already begun Serial before
    // setup() runs, so this is a re-begin, and the CDC ring buffers already
    // exist.  No attempt is made to resize them: setTxBufferSize() is a no-op
    // once the ring is allocated, so calling it would only look like it worked.
    // HOSTLINK_MAX_LOG_TEXT is sized against the ring instead.
    Serial.begin(HOSTLINK_BAUD); // native USB CDC ignores the rate; harmless and explicit

    // Zero timeout = write() never waits for the host to drain the ring.  With
    // the availableForWrite() precheck in sendFrame() this makes every TX path
    // strictly bounded, which is what keeps loop() responsive when the Jetson
    // is busy, suspended, or unplugged.
    Serial.setTxTimeoutMs(0);

    hostproto::rxInit(rx_);
    lastHostRxMs_ = 0;
    txDropped_    = 0;
    hostAlive_    = false;
    connectSeen_  = false;
    resetSessionState();
}

void HostLink::resetSessionState()
{
    // Power-on defaults, re-applied on every fresh host connection: a Jetson
    // that reconnects must not inherit the stream settings of a previous
    // process that died without sending CMD_STOP_STREAM.
    streaming_ = false;
    decim_     = 1;
    onceReq_   = false;
    statusReq_ = false;
}

bool HostLink::sendFrame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    const size_t n = hostproto::buildFrame(type, payload, len, txBuf_, sizeof(txBuf_));
    if (n == 0) { ++txDropped_; return false; }

    // Send only when the whole frame fits the CDC ring.  A partial write would
    // desynchronise the host decoder until its 250 ms inter-byte timeout fires;
    // dropping the frame instead costs one sample and keeps the stream framed.
    if (Serial.availableForWrite() < static_cast<int>(n)) { ++txDropped_; return false; }

    if (Serial.write(txBuf_, n) != n) { ++txDropped_; return false; }
    return true;
}

bool HostLink::sendNack(uint8_t offendingType, uint8_t reason)
{
    hostproto::Nack n;
    n.offendingType = offendingType;
    n.reason        = reason;
    return sendFrame(hostproto::MSG_NACK, reinterpret_cast<const uint8_t *>(&n), sizeof(n));
}

bool HostLink::sendTelemetry(const hostproto::Telemetry &t)
{
    return sendFrame(hostproto::MSG_TELEMETRY,
                     reinterpret_cast<const uint8_t *>(&t), sizeof(t));
}

bool HostLink::sendStatus(const hostproto::BridgeStatus &s)
{
    return sendFrame(hostproto::MSG_STATUS,
                     reinterpret_cast<const uint8_t *>(&s), sizeof(s));
}

bool HostLink::sendHello(const hostproto::Hello &h)
{
    return sendFrame(hostproto::MSG_HELLO,
                     reinterpret_cast<const uint8_t *>(&h), sizeof(h));
}

bool HostLink::sendLog(uint8_t level, const char *text)
{
    if (!text) return false;

    // strnlen bounds the scan even if the caller hands over an unterminated
    // buffer; the text is then truncated to what one frame can carry AND fit in
    // the TX ring (see HOSTLINK_MAX_LOG_TEXT).
    const size_t textLen = strnlen(text, HOSTLINK_MAX_LOG_TEXT);

    logBuf_[0] = level;
    memcpy(&logBuf_[1], text, textLen);       // no NUL on the wire: LEN-1 gives the length
    const uint8_t len = static_cast<uint8_t>(sizeof(hostproto::LogHeader) + textLen);

    return sendFrame(hostproto::MSG_LOG, logBuf_, len);
}

void HostLink::handleCommand(uint8_t type, const uint8_t *payload, uint8_t len)
{
    // Order matters: an unknown TYPE must not be reported as a length fault.
    // "Unknown type" means the host is built against a different contract;
    // "bad length" means a corrupted frame slipped past CRC.  Diagnosing one as
    // the other wastes the next person's afternoon.
    if (!hostproto::isCommand(type)) {
        (void)sendNack(type, hostproto::NACK_UNKNOWN_TYPE);
        return;
    }

    // Every command has a fixed payload length; a mismatch means a malformed
    // frame that happened to pass CRC, or a host built against a newer contract.
    if (len != hostproto::commandPayloadLen(type)) {
        (void)sendNack(type, hostproto::NACK_BAD_LENGTH);
        return;
    }

    switch (type) {
    case hostproto::CMD_PING:
        (void)sendFrame(hostproto::MSG_PONG, nullptr, 0);
        break;

    case hostproto::CMD_GET_ONCE:
        onceReq_ = true;   // the sketch forwards this to the MKR and relays the answer
        break;

    case hostproto::CMD_GET_STATUS:
        statusReq_ = true;
        break;

    case hostproto::CMD_HELLO:
        // Explicit new-session announcement.  This is the only path that makes
        // a fast reconnect deterministic: the rising edge in poll() below fires
        // solely on the host-silence timeout, so a host that reopens the port
        // within HOSTLINK_HOST_TIMEOUT_MS would otherwise inherit the previous
        // session's streaming state and never be sent a MSG_HELLO.
        resetSessionState();
        connectSeen_ = true;
        break;

    case hostproto::CMD_START_STREAM:
        streaming_ = true;
        break;

    case hostproto::CMD_STOP_STREAM:
        streaming_ = false;
        break;

    case hostproto::CMD_SET_DECIM:
        // 0 would mean "forward no frames", which CMD_STOP_STREAM already says
        // and which would make the sketch's "every Nth frame" counter never fire.
        if (payload[0] == 0) { (void)sendNack(type, hostproto::NACK_BAD_VALUE); break; }
        decim_ = payload[0];
        break;

    default:
        break; // unreachable: isCommand() already filtered unknown types
    }
}

bool HostLink::poll(uint32_t nowMs)
{
    bool    serviced = false;
    uint8_t type = 0, len = 0;

    // Bounded twice over: by the byte budget (so a flood of non-SOF noise cannot
    // scan indefinitely) and by the frame budget (so a back-to-back command
    // burst cannot monopolise loop()).  Leftovers are picked up next poll().
    unsigned bytes = 0, frames = 0;
    while (bytes < HOSTLINK_MAX_BYTES_PER_POLL &&
           frames < HOSTLINK_MAX_FRAMES_PER_POLL &&
           Serial.available() > 0) {
        const int r = Serial.read();
        if (r < 0) break;             // ring emptied between available() and read()
        ++bytes;

        const hostproto::Status s =
            hostproto::rxByte(rx_, static_cast<uint8_t>(r), nowMs, type,
                              rxPayload_, static_cast<uint8_t>(sizeof(rxPayload_)), len);
        if (s != hostproto::Status::FRAME_READY) continue; // bad CRC / overflow: keep draining

        ++frames;
        serviced      = true;
        lastHostRxMs_ = nowMs;

        // A frame from the host is the liveness signal.  Rising edge = a new
        // session, so wipe any state the previous one left behind before the
        // command is applied (a START_STREAM in this very frame must survive).
        if (!hostAlive_) {
            hostAlive_   = true;
            connectSeen_ = true;
            resetSessionState();
        }

        handleCommand(type, rxPayload_, len);
    }

    // Falling edge: the host stopped sending its keepalive.  Stop streaming so
    // the bridge is not pushing 10 Hz of telemetry into a ring nobody drains.
    if (hostAlive_ && (nowMs - lastHostRxMs_) > HOSTLINK_HOST_TIMEOUT_MS) {
        hostAlive_ = false;
        resetSessionState();
    }

    return serviced;
}
