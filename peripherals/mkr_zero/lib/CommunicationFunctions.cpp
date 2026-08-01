#include "CommunicationFunctions.h"

#include <string.h>
#include <math.h>

// Full struct definitions for the telemetry builder. These headers still pull
// in ExternalLibConfig.h, so they are included here (in the .cpp) only — the
// public header above stays decoupled via forward declarations.
#include "OBD2Functions.h"
#include "GPSFunctions.h"

bool splitByte(const char* input, char* outputBuffer, const size_t outputBufferLength, size_t byteLength, size_t byteOffset){
    if (!input || !outputBuffer){
        return false;
    }

    if (byteOffset >= strlen(input)){
        return false;
    }

    if (byteLength == 0 || outputBufferLength == 0){
        return false;
    }

    if (byteLength + byteOffset > strlen(input)){
        byteLength = strlen(input) - byteOffset;
    }

    memset(outputBuffer, 0, outputBufferLength);
    size_t lenToCopy = (byteLength < outputBufferLength - 1) ? byteLength : outputBufferLength - 1;
    memcpy(outputBuffer, input + byteOffset, lenToCopy);
    outputBuffer[lenToCopy] = '\0';
    return true;
}

void initializeComm(unsigned long baud){
    Serial1.begin(baud);
}

CommReturnStatus sendFrame(uint8_t type, const uint8_t *payload, uint8_t len){
    uint8_t frame[COMM_MAX_FRAME];
    size_t n = buildFrame(type, payload, len, frame, sizeof(frame));
    if (n == 0) return CommReturnStatus::NOK_OVERFLOW;
    // Non-blocking: transmit only when the whole frame fits the TX buffer, so a
    // congested link drops a frame instead of stalling the loop inside write().
    if (Serial1.availableForWrite() < static_cast<int>(n)) return CommReturnStatus::NOK_BUSY;
    size_t written = Serial1.write(frame, n);
    return (written == n) ? CommReturnStatus::OK : CommReturnStatus::NOK_BUSY;
}

void buildTelemetry(const OBD2Data &obd, const GPSData &gps, const IMUData &imu, const DerivedSignals &derived, const VehicleSignals &veh, uint8_t canMode, TelemetryPayload &out){
    out.masterMillis = millis();

    out.speed       = obd.speed;
    // Derived, not a PID: NAN passes through unchanged so the receiver can tell
    // "estimator still warming up" from "coasting at 0 m/s2".
    out.accel       = derived.accelMs2;
    out.rpm         = obd.rpm;
    out.coolantTemp = obd.coolantTemp;
    out.fuelLevel   = obd.fuelLevel;
    out.fuelRate    = obd.fuelRate;
    out.throttle    = obd.throttle;
    out.engineLoad  = obd.engineLoad;
    out.airPressure = obd.airPressure;
    out.gear        = obd.gear;
    out.gearRatio   = obd.gearRatio;
    out.odo         = obd.odo;

    out.latitude    = gps.latitudeDegrees;
    out.longitude   = gps.longitudeDegrees;
    out.altitude    = gps.altitudeM;
    out.gpsSpeedKmh = gps.velocityKmh;
    // Filtered course, not the raw receiver value: NAN when the vector mean
    // is not trustworthy (see updateHeading() in the sketch).
    out.heading     = derived.headingDeg;
    out.satellites  = gps.satellites;
    out.fixType     = gps.fixType;
    out.fixValid    = gps.fixValid ? 1 : 0;

    out.year   = gps.utc.year;
    out.month  = gps.utc.month;
    out.day    = gps.utc.day;
    out.hour   = gps.utc.hour;
    out.minute = gps.utc.minute;
    out.second = gps.utc.second;

    // IMU, sensor frame, copied straight through.  IMUData already publishes NAN
    // for any channel that is stale or whose device has been declared lost, and
    // that is exactly the payload's own convention for "not supplied" — so there
    // is nothing to translate here, and adding a second staleness rule on top
    // would only create a way for the two to disagree.
    out.imuAccelX = imu.accelX;
    out.imuAccelY = imu.accelY;
    out.imuAccelZ = imu.accelZ;
    out.imuGyroX  = imu.gyroX;
    out.imuGyroY  = imu.gyroY;
    out.imuGyroZ  = imu.gyroZ;
    out.imuMagX   = imu.magX;
    out.imuMagY   = imu.magY;
    out.imuMagZ   = imu.magZ;
    out.imuTempC  = imu.temperatureC;
    // Peaks, not another instantaneous sample.  At 10 Hz the axes above carry
    // one of roughly ten samples the sensor produced since the last frame, so
    // these are the only fields in which a transient between frames survives.
    out.imuAccelPeak = imu.accelPeakMs2;
    out.imuGyroPeak  = imu.gyroPeakDps;

    uint8_t flags = 0;
    // OBD2 is "live" only if tickOBD2() stored a reading within the freshness window,
    // so the flag clears within OBD2_FRESH_WINDOW_MS of the ECU going quiet.
    if (obd.lastUpdateMs != 0 && (millis() - obd.lastUpdateMs) < OBD2_FRESH_WINDOW_MS) flags |= COMM_FLAG_OBD2_VALID;
    if (gps.fixValid)  flags |= COMM_FLAG_GPS_FIX;
    if (gps.utc.valid) flags |= COMM_FLAG_TIME_VALID;
    // Receiver present, which is not the same question as "has a fix" — see
    // COMM_FLAG_GPS_PRESENT.  Maintained by the caller alongside its retry state.
    if (gps.devicePresent) flags |= COMM_FLAG_GPS_PRESENT;
    // Hardware presence, not sample freshness — per-channel freshness is already
    // carried by the NANs above.  Deriving this from the valid flags instead
    // would clear it whenever the IMU merely had nothing new this poll, making a
    // fitted-but-quiet sensor indistinguishable from no sensor at all.
    if (imu.devicePresent) flags |= COMM_FLAG_IMU_PRESENT;
    // Present but incomplete is a wiring warning for the WHOLE module, so it is
    // raised as its own flag rather than left for the consumer to infer from
    // which axes happen to be NAN.
    if (imu.devicePresent && !imu.allDevicesPresent) flags |= COMM_FLAG_IMU_DEGRADED;
    // Both of these qualify the peaks above rather than the axes.  Sent as flags
    // instead of poisoning the peak with NAN because the value is still the best
    // available estimate — it just is not the guarantee the field normally is,
    // and a consumer weighing an incident needs to know which it is holding.
    if (imu.dataGap) flags |= COMM_FLAG_IMU_DATA_GAP;
    if (imu.lowPower) flags |= COMM_FLAG_IMU_LOWPOWER;
    out.flags = flags;

    // ---- vehicle bus ----
    // Copied verbatim from the template, sentinels included. expireVehicleSignals()
    // has already turned staleness into absence upstream, so nothing here needs
    // to re-judge freshness - and re-judging it against a second window is how
    // the two would drift apart.
    out.canMode          = canMode;
    out.sigSource        = packVehSourceByte(veh);
    out.gearPos          = (uint8_t)veh.gear;
    // Turn bits are the HELD indicator state, not the raw lamp — the blink is
    // ~1.5 Hz and this payload leaves at ~4 Hz, so the decoder holds each flash
    // and what ships is "indicating", which is the question a lane-keeping
    // consumer is actually asking.
    out.vehFlags         = (uint8_t)((veh.brakePressed ? 0x01u : 0x00u) |
                                     (veh.brakeSwitch  ? 0x02u : 0x00u) |
                                     (veh.turnLeft     ? 0x04u : 0x00u) |
                                     (veh.turnRight    ? 0x08u : 0x00u));
    out.pedalGas         = veh.pedalGas;
    out.steerMotorTorque = veh.steerMotorTorque;
    out.yawRateCdps      = veh.yawRateCdps;
    for (uint8_t i = 0; i < VEH_WHEEL_COUNT; ++i) out.wheelRaw[i] = veh.wheelRaw[i];

    // Sniffed speed and rpm outrank the OBD2 copies when both exist: 50-100 Hz
    // against a ~500 ms poll cycle, and 0.01 km/h against whole km/h. The OBD2
    // values stay in their own fields; this only decides what the primary
    // `speed`/`rpm` carry, which is what the overlay reads.
    if (veh.speedSrc != VehSource::NONE && !isnan(veh.speedKmh)) out.speed = veh.speedKmh;
    if (veh.rpmSrc   != VehSource::NONE && !isnan(veh.rpm))      out.rpm   = veh.rpm;
}

CommReturnStatus sendTelemetry(const OBD2Data &obd, const GPSData &gps, const IMUData &imu, const DerivedSignals &derived, const VehicleSignals &veh, uint8_t canMode){
    TelemetryPayload p;
    buildTelemetry(obd, gps, imu, derived, veh, canMode, p);
    return sendFrame(MSG_TELEMETRY, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
}

void initCommMaster(CommMaster &m){
    m.streaming     = false;
    m.lastPushMs    = 0;
    m.lastCommandMs = 0;
    m.oncePending   = false;
    m.onceRequestMs = 0;
    m.canModeRequest = 0;
    commRxInit(m.rx);
}

void tickCommMaster(CommMaster &m, const OBD2Data &obd, const GPSData &gps, const IMUData &imu, const DerivedSignals &derived, const VehicleSignals &veh, uint8_t canMode){
    // 1) Service inbound C3 commands: non-blocking and bounded to a fixed budget
    //    per call so a command flood cannot monopolise the loop.
    uint8_t  type = 0, len = 0;
    uint8_t  payload[COMM_MAX_PAYLOAD];
    for (uint8_t serviced = 0; serviced < COMM_MAX_CMDS_PER_TICK; ++serviced){
        CommReturnStatus r = pollFrame(m.rx, Serial1, type, payload, sizeof(payload), len);
        if (r == CommReturnStatus::NO_DATA) break;          // input exhausted
        if (r != CommReturnStatus::FRAME_READY) continue;   // bad CRC/overflow: counts toward budget

        // A CRC-valid frame is proof the bridge is alive and wired correctly.
        // Recorded here rather than per command type so a PING keeps the link
        // marked healthy even when nothing is streaming.
        m.lastCommandMs = millis();

        // Unknown TYPE before bad LENGTH, deliberately. The reverse order tells
        // a peer that speaks a LATER protocol version its command was the right
        // command with the wrong length, which sends the investigation looking
        // at framing instead of at the version mismatch that actually caused it.
        //
        // This replaces a blanket "len != 0 -> NACK", which was correct only for
        // as long as every command was zero-payload. CMD_SET_CAN_MODE is the
        // first that is not, and under the old rule the MKR would have rejected
        // it as malformed - a bug that would only ever appear in the car.
        if (!isCommand(type)){
            sendFrame(MSG_NACK, nullptr, 0);
            continue;
        }
        if (len != commandPayloadLen(type)){
            sendFrame(MSG_NACK, nullptr, 0);
            continue;
        }

        switch (type){
            case CMD_GET_ONCE:
                // A one-shot must not be lost to a momentarily full TX buffer:
                // the requester gets no answer and no error, and simply waits.
                // Latch it instead and let the retry below deliver it.
                if (sendTelemetry(obd, gps, imu, derived, veh, canMode) != CommReturnStatus::OK){
                    m.oncePending   = true;
                    m.onceRequestMs = millis();
                }
                break;
            case CMD_START_STREAM:
                m.streaming  = true;
                m.lastPushMs = millis() - COMM_STREAM_INTERVAL_MS; // push promptly
                break;
            case CMD_STOP_STREAM:
                m.streaming = false;
                break;
            case CMD_SET_CAN_MODE:
                // Latched, not applied. See CommMaster::canModeRequest for why
                // the transition does not happen inside the frame decoder.
                // Validated here so an out-of-range value is rejected at the
                // wire boundary rather than reaching the CAN driver.
                if (payload[0] >= 1u && payload[0] <= 3u){
                    m.canModeRequest = payload[0];
                } else {
                    sendFrame(MSG_NACK, nullptr, 0);
                }
                break;
            case CMD_PING:
                sendFrame(MSG_PONG, nullptr, 0);
                break;
            default:
                sendFrame(MSG_NACK, nullptr, 0);
                break;
        }
    }

    // 1b) Retry a latched one-shot until it goes out or the request expires.
    //     Bounded by COMM_ONCE_TIMEOUT_MS so a permanently congested link drops
    //     the request instead of answering it minutes later, out of context.
    if (m.oncePending){
        if ((millis() - m.onceRequestMs) >= COMM_ONCE_TIMEOUT_MS){
            m.oncePending = false;  // give up: the answer would be stale anyway
        } else if (sendTelemetry(obd, gps, imu, derived, veh, canMode) == CommReturnStatus::OK){
            m.oncePending = false;
        }
    }

    // 2) Streaming push at COMM_STREAM_INTERVAL_MS (unsigned subtraction is
    //    millis()-rollover safe, mirroring isTimeout() in TimerFunctions.h). Advance
    //    the clock only when the frame actually went out, so a NOK_BUSY (congested TX)
    //    retries on the next loop instead of being silently dropped for a full period.
    if (m.streaming && (millis() - m.lastPushMs) >= COMM_STREAM_INTERVAL_MS){
        if (sendTelemetry(obd, gps, imu, derived, veh, canMode) == CommReturnStatus::OK){
            m.lastPushMs = millis();
        }
    }
}

bool isCommLinkSilent(const CommMaster &m, unsigned long timeoutMs){
    // Never-heard-from counts as silent: at boot the bridge may genuinely be
    // absent, and reporting that is the point.
    if (m.lastCommandMs == 0UL) return true;
    return (millis() - m.lastCommandMs) > timeoutMs;
}
