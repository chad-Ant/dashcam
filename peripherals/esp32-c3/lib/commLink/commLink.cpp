#include "commLink.h"

void CommLink::begin(unsigned long baud, int8_t rxPin, int8_t txPin, HardwareSerial &uart)
{
    uart_ = &uart;
    uart_->begin(baud, SERIAL_8N1, rxPin, txPin);
    commRxInit(rx_);
    hasData_   = false;
    pong_      = false;
    lastRxMs_  = 0;
    crcErrors_ = 0;
    framesRx_  = 0;
}

bool CommLink::sendCmd(uint8_t type, const uint8_t *payload, uint8_t len)
{
    if (!uart_) return false;
    // Sized for the largest command this hop can carry, not for zero. It used
    // to be exactly COMM_FRAME_OVERHEAD, which was correct only while every
    // command was zero-payload: buildFrame() would have refused the first
    // payload-bearing one for want of a single byte of buffer, and the caller
    // would have read that as a busy link rather than a bug.
    uint8_t frame[COMM_FRAME_OVERHEAD + COMM_MAX_CMD_PAYLOAD];
    if (len > COMM_MAX_CMD_PAYLOAD) return false;
    size_t n = buildFrame(type, payload, len, frame, sizeof(frame));
    if (n == 0) return false;
    // Non-blocking: only write when the whole frame fits, and confirm the byte count.
    if (uart_->availableForWrite() < static_cast<int>(n)) return false;
    return uart_->write(frame, n) == n;
}

bool CommLink::requestOnce() { return sendCmd(CMD_GET_ONCE); }
bool CommLink::startStream() { return sendCmd(CMD_START_STREAM); }
bool CommLink::stopStream()  { return sendCmd(CMD_STOP_STREAM); }
bool CommLink::ping()        { return sendCmd(CMD_PING); }

bool CommLink::setCanMode(uint8_t mode)
{
    // Validated at this boundary as well as at the MKR. The bridge is where an
    // out-of-range host value should die: relaying it would spend a UART frame
    // to earn a NACK the host has no way to attribute to its own bad argument.
    if (mode < 1u || mode > 3u) return false;
    return sendCmd(CMD_SET_CAN_MODE, &mode, 1);
}

bool CommLink::setImuMode(uint8_t mode)
{
    // Validated at this boundary as well as at the MKR, for the same reason
    // setCanMode() is: relaying an out-of-range value spends a UART frame to
    // earn a NACK the host has no way to attribute to its own argument.
    if (mode != COMM_IMU_MODE_FUSION && mode != COMM_IMU_MODE_RAW) return false;
    return sendCmd(CMD_SET_IMU_MODE, &mode, 1);
}

bool CommLink::setCanFilter(const uint16_t *ids, uint8_t count)
{
    if (count > COMM_CAN_FILTER_SLOTS) return false;
    if (ids == nullptr && count != 0u)  return false;

    // Fixed length regardless of count: unused slots are transmitted as zeros so
    // the receiver can check the frame size before trusting any of its contents.
    // A variable-length command would have to be parsed to know how long it
    // should have been.
    uint8_t p[COMM_SET_CAN_FILTER_LEN] = { 0 };
    p[0] = count;
    for (uint8_t i = 0; i < count; ++i) {
        // Little-endian on the wire, matching every other multi-byte field on
        // this link — both ends are little-endian, but writing it out by hand
        // keeps the framing independent of that happening to be true.
        p[1 + i * 2] = (uint8_t)(ids[i] & 0xFFu);
        p[2 + i * 2] = (uint8_t)(ids[i] >> 8);
    }
    return sendCmd(CMD_SET_CAN_FILTER, p, sizeof(p));
}

/**
 * @brief The larger of two peaks, treating NAN as "no measurement".
 *
 * NAN is the payload's own "not supplied", so it must lose to any real value and
 * must not survive one. A plain `>` gets both wrong: every comparison against NAN
 * is false, so the accumulator would latch NAN forever the first time a channel
 * went stale.
 *
 * BY VALUE, not through a reference parameter. TelemetryPayload is packed, so its
 * members may be unaligned and a `float&` to one of them does not compile — which
 * is the compiler stopping an unaligned access rather than a nuisance to work
 * around.
 */
static inline float maxOf(float acc, float sample)
{
    if (isnan(sample)) return acc;
    if (isnan(acc) || (sample > acc)) return sample;
    return acc;
}

void CommLink::accumulate(const TelemetryPayload &src)
{
    coalescedFlags_        |= (uint16_t)(src.flags & kStickyFlags);
    coalescedAccelPeak_     = maxOf(coalescedAccelPeak_,    src.imuAccelPeak);
    coalescedGyroPeak_      = maxOf(coalescedGyroPeak_,     src.imuGyroPeak);
    coalescedLinAccelPeak_  = maxOf(coalescedLinAccelPeak_, src.imuLinAccelPeak);
    // A set bit is an event that happened; OR is the only merge that cannot
    // lose one. (v0x07)
    coalescedSwitchChanged_ |= src.switchChanged;
}

void CommLink::mergeCoalesced(TelemetryPayload &out) const
{
    out.flags           = (uint16_t)(out.flags | coalescedFlags_);
    out.imuAccelPeak    = maxOf(out.imuAccelPeak,    coalescedAccelPeak_);
    out.imuGyroPeak     = maxOf(out.imuGyroPeak,     coalescedGyroPeak_);
    out.imuLinAccelPeak = maxOf(out.imuLinAccelPeak, coalescedLinAccelPeak_);
    out.switchChanged   = (uint16_t)(out.switchChanged | coalescedSwitchChanged_);
}

void CommLink::clearCoalesced()
{
    coalescedFlags_        = 0;
    coalescedAccelPeak_    = NAN;
    coalescedGyroPeak_     = NAN;
    coalescedLinAccelPeak_ = NAN;
    coalescedSwitchChanged_ = 0;
}

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
        // Count CRC failures: on a vehicle harness they are the first symptom of
        // a marginal ground or an EMI problem, and the bridge reports them to
        // the Jetson in BridgeStatus::masterCrcErrors.
        if (r == CommReturnStatus::NOK_CRC) { ++crcErrors_; continue; }
        if (r != CommReturnStatus::FRAME_READY) continue; // overflow: keep draining

        switch (type) {
        case MSG_TELEMETRY:
            if (len == sizeof(TelemetryPayload)) {
                memcpy(&latest_, payload, sizeof(TelemetryPayload));
                // BEFORE the overwrite is allowed to matter. This loop can accept
                // eight frames and the forwarder sends one, so without this the
                // other seven are simply gone — including a High-G latch the
                // master held for 500 ms specifically so it could not fall
                // between two frames. It fell between two frames here instead.
                accumulate(latest_);
                hasData_  = true;
                lastRxMs_ = millis();
                fresh     = true;
                ++framesRx_;
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
