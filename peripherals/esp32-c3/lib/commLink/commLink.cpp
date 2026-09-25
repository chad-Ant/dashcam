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
    clearCoalesced();
    masterRestarts_ = 0;
    endedDropped_   = 0;
    endedPending_   = false;
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
    if (unsent_ < UINT32_MAX) ++unsent_;
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
    unsent_                = 0;
}

/**
 * @brief True when @p nextMaster cannot come from the master boot that sent
 *        @p prevMaster, @p bridgeElapsedMs of this bridge's time earlier.
 *
 * masterMillis is the master's millis() at the moment it built the frame, so
 * within one boot it advances with this bridge's own clock. Two ways out:
 *
 *  - It went BACKWARDS. A running millis() never does — its 49.7-day wrap is a
 *    small forward step in modular arithmetic — so this is a restart. It covers
 *    every reboot after less than 24.8 days of uptime.
 *  - It moved by something other than the elapsed time: a restart after a
 *    very young boot (it lost less time than the gap), or after more than
 *    24.8 days (the modular difference turns positive and huge).
 *
 * A master that merely went quiet — cable pulled, loop stalled — agrees with
 * the elapsed time however long the silence, because both clocks kept running.
 * Only a reboot, which throws the master's clock away, disagrees.
 */
static bool masterClockBroke(uint32_t prevMaster, uint32_t nextMaster, uint32_t bridgeElapsedMs)
{
    const uint32_t masterElapsed = nextMaster - prevMaster;
    if (static_cast<int32_t>(masterElapsed) < 0) return true;

    const int64_t skew  = static_cast<int64_t>(masterElapsed) - static_cast<int64_t>(bridgeElapsedMs);
    const int64_t slack = static_cast<int64_t>(COMMLINK_MASTER_CLOCK_SLACK_MS) +
                          static_cast<int64_t>(bridgeElapsedMs / 1024u);
    return (skew > slack) || (skew < -slack);
}

/**
 * @brief Reduces a snapshot to what stays true after its boot has ended.
 *
 * Kept: the boot's identity (masterMillis, imuHighGCount/imuHighGMs), its
 * events and window maxima (the sticky flags, the three peaks, switchChanged),
 * and what describes the hardware and configuration those were measured with
 * (presence, mode and calibration flags, canMode, the map identity, switchState
 * — documented as last-measured already). Blanked to the wire's own "not
 * supplied" values: every INSTANTANEOUS reading, because by the time this frame
 * goes up it is at least a reboot old, and a consumer that takes the newest
 * frame as current would otherwise show a speed, a position or a time from
 * before the reboot as live.
 */
static void stripToEvents(TelemetryPayload &p)
{
    p.speed = p.accel = p.rpm = p.coolantTemp = p.fuelLevel = p.fuelRate = NAN;
    p.throttle = p.engineLoad = p.airPressure = p.gear = p.gearRatio = p.odo = NAN;
    p.latitude = p.longitude = p.altitude = p.gpsSpeedKmh = p.heading = NAN;
    p.satellites = 0; p.fixType = 0; p.fixValid = 0;
    p.year = 0; p.month = 0; p.day = 0; p.hour = 0; p.minute = 0; p.second = 0;

    p.imuAccelX = p.imuAccelY = p.imuAccelZ = NAN;
    p.imuGyroX  = p.imuGyroY  = p.imuGyroZ  = NAN;
    p.imuMagX   = p.imuMagY   = p.imuMagZ   = NAN;
    p.imuTempC  = NAN;
    p.imuLinAccelX = p.imuLinAccelY = p.imuLinAccelZ = NAN;
    p.imuYawRelDeg = NAN;

    p.flags = (uint16_t)(p.flags & ~(COMM_FLAG_OBD2_VALID | COMM_FLAG_GPS_FIX | COMM_FLAG_TIME_VALID));

    p.sigSource        = 0;          // every source NONE
    p.gearPos          = 0;          // unknown
    p.vehFlags         = 0;          // no *_VALID bit, so no brake/turn/pedal claim
    p.pedalGas         = 0;
    p.steerMotorTorque = 0xFFFFu;
    p.yawRateCdps      = INT16_MIN;
    for (uint8_t i = 0; i < 4; ++i) p.wheelRaw[i] = 0xFFFFu;
}

void CommLink::endMasterSession()
{
    if (masterRestarts_ < UINT32_MAX) ++masterRestarts_;

    // Only when the old boot has frames nobody has been given yet. Its newest
    // snapshot with its own accumulator merged in carries exactly the events
    // the next forward would have, reduced to what is still true.
    if (unsent_ > 0) {
        if (!endedPending_) {
            ended_ = latest_;
            mergeCoalesced(ended_);
            stripToEvents(ended_);
            endedPending_ = true;
        } else if (endedDropped_ < UINT32_MAX) {
            ++endedDropped_;      // keep-first; see endedSession()
        }
    }
    clearCoalesced();
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
                TelemetryPayload next;
                memcpy(&next, payload, sizeof(TelemetryPayload));
                const uint32_t nowMs = millis();

                // A master reboot closes the accumulator BEFORE the new boot's
                // first frame can be merged into it — see endedSession().
                const bool boundary =
                    hasData_ && masterClockBroke(latest_.masterMillis, next.masterMillis,
                                                 nowMs - lastRxMs_);
                if (boundary) endMasterSession();

                latest_ = next;
                // BEFORE the overwrite is allowed to matter. This loop can accept
                // eight frames and the forwarder sends one, so without this the
                // other seven are simply gone — including a High-G latch the
                // master held for 500 ms specifically so it could not fall
                // between two frames. It fell between two frames here instead.
                accumulate(latest_);
                hasData_  = true;
                lastRxMs_ = nowMs;
                fresh     = true;
                ++framesRx_;

                // Stop at the boundary, so the caller sees the ended boot's
                // frame before any further frame is accepted — two reboots can
                // never be folded into one batch. The rest stay in the UART
                // ring for the next poll().
                if (boundary) return fresh;
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
