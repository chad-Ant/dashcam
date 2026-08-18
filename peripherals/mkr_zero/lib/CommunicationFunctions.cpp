#include "CommunicationFunctions.h"

#include <string.h>
#include <math.h>

// Full struct definitions for the telemetry builder. These headers still pull
// in ExternalLibConfig.h, so they are included here (in the .cpp) only — the
// public header above stays decoupled via forward declarations.
#include "OBD2Functions.h"
#include "GPSFunctions.h"
// The map's identity and the controller's live filter state both go on the wire
// as of v0x06, so the builder needs the sniffer's accessors.
#include "CANSniffFunctions.h"

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

void buildTelemetry(const OBD2Data &obd, const GPSData &gps, const IMUData &imu, const SwitchData &sw, const DerivedSignals &derived, const VehicleSignals &veh, uint8_t canMode, TelemetryPayload &out){
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

    // Fusion output (v0x06). The sensor computes these; nothing is derived here.
    out.imuLinAccelPeak = imu.linAccelPeakMs2;
    // The signed vector behind that magnitude (v0x07). Instantaneous, not
    // windowed: these say which way, the peak beside them says how hard.
    out.imuLinAccelX    = imu.linAccelX;
    out.imuLinAccelY    = imu.linAccelY;
    out.imuLinAccelZ    = imu.linAccelZ;
    out.imuYawRelDeg    = imu.yawRelDeg;

    // ---- High-G durability (v0x07) ----
    //
    // AGE, not a timestamp, and saturating rather than wrapping. UINT16_MAX is
    // the sentinel for "no latch has ever happened", which is a different
    // statement from "the last one was a long time ago" only in principle:
    // both mean nothing recent, and collapsing them costs a consumer nothing
    // while removing a special case.
    //
    // The subtraction is on unsigned millis() and is therefore correct across
    // the 49-day wrap; the clamp below is what keeps the result meaningful.
    if (imu.highGCount == 0u) {
        out.imuHighGMs = UINT16_MAX;
    } else {
        const uint32_t age = millis() - imu.highGMs;
        out.imuHighGMs = (age >= (uint32_t)UINT16_MAX) ? UINT16_MAX : (uint16_t)age;
    }
    out.imuHighGCount = imu.highGCount;
    // Packed in the sensor's own CALIB_STAT order — mag, accel, gyro, system —
    // so a consumer holding the datasheet reads it without a translation table.
    out.imuCalib = (uint8_t)(( imu.calibMag         & 0x03u)        |
                             ((imu.calibAccel & 0x03u) << 2) |
                             ((imu.calibGyro  & 0x03u) << 4) |
                             ((imu.calibSys   & 0x03u) << 6));

    // ---- CAN map identity (v0x06) ----
    // Taken from the sniffer rather than passed in: it already owns the pointer,
    // and threading a seventh parameter through four call sites to reach a value
    // one module already holds is how signatures rot.
    const CanSignalMap *map = canSniffGetMap();
    out.canMapChecksum = (map != nullptr && map->loaded) ? map->checksum : 0u;
    out.canMapFlags    = 0u;
    // Read from the CONTROLLER's live state, not from the map's intentions. The
    // two diverge the moment a host sends CMD_SET_CAN_FILTER, and the field
    // exists to describe what a capture actually excluded.
    if (canSniffFilterCount() > 0u)  out.canMapFlags |= 0x01u;
    if (canSniffFiltersFromMap())    out.canMapFlags |= 0x02u;

    uint16_t flags = 0;
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
    // The hardware latch, and the only inertial evidence that survives a stalled
    // master — so it is raised independently of whether the peaks look eventful.
    if (imu.highGEvent) flags |= COMM_FLAG_IMU_HIGH_G;
    // And whether that latch was armed to begin with. Without this the flag above
    // is unfalsifiable: its absence reads as "no impact" whether the backstop was
    // watching or had been disarmed at bring-up, and IMUData::highGArmed —
    // documented as the thing nothing else on the frame would say — did not reach
    // the frame at all.
    if (imu.highGArmed) flags |= COMM_FLAG_IMU_HIGHG_ARMED;
    // Which MEASUREMENT this frame carries, not how good it is. The two modes
    // populate different fields and put the same peak against different rails,
    // so a consumer that cannot tell them apart is reading the frame wrong —
    // and until this bit existed the only way to guess was which fields happened
    // to be NAN, which is indistinguishable from a stale channel.
    if (imu.fusionMode) flags |= COMM_FLAG_IMU_FUSION_MODE;
    // Qualifies BOTH peaks: linear acceleration is derived from the same clipped
    // accelerometer, so a rail the raw channel hit propagates straight into it.
    if (imu.accelSaturated) flags |= COMM_FLAG_IMU_SATURATED;
    if (map != nullptr && map->loaded) flags |= COMM_FLAG_CANMAP_LOADED;
    // The switch chain's own positive control (v0x07). Without it an absent
    // chain, a dead register and fourteen open latching switches all serialise
    // as the same word — the same trap COMM_FLAG_IMU_PRESENT and
    // COMM_FLAG_GPS_PRESENT were added to close for their subsystems.
    if (sw.present) flags |= COMM_FLAG_SWITCHES_PRESENT;
    out.flags = flags;

    // ---- panel switches (v0x07) ----
    //
    // Copied whatever `present` says. When it is clear these are the last
    // MEASURED positions rather than current ones, which is strictly better
    // than zeroing them: zeroed positions are indistinguishable from fourteen
    // open switches that were actually read.
    out.switchState   = sw.state;
    out.switchChanged = sw.changed;

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
    //
    // The *_VALID bits are what stop a cleared state bit being read as a
    // measurement. Every one of these signals has a false value that looks
    // perfectly safe — brake released, not indicating — so an absent or expired
    // signal is indistinguishable from a live negative reading without them.
    // In OBD2 mode none of the three can be populated at all, and a lane-keeping
    // consumer that trusted the raw bits there would call every signalled lane
    // change unsignalled.
    out.vehFlags         = (uint8_t)((veh.brakePressed ? COMM_VEH_FLAG_BRAKE_PRESSED : 0x00u) |
                                     (veh.brakeSwitch  ? COMM_VEH_FLAG_BRAKE_SWITCH  : 0x00u) |
                                     (veh.turnLeft     ? COMM_VEH_FLAG_TURN_LEFT     : 0x00u) |
                                     (veh.turnRight    ? COMM_VEH_FLAG_TURN_RIGHT    : 0x00u) |
                                     (veh.brakeSrc != VehSource::NONE ? COMM_VEH_FLAG_BRAKE_VALID : 0x00u) |
                                     (veh.turnSrc  != VehSource::NONE ? COMM_VEH_FLAG_TURN_VALID  : 0x00u) |
                                     (veh.pedalSrc != VehSource::NONE ? COMM_VEH_FLAG_PEDAL_VALID : 0x00u) |
                                     (veh.hazard       ? COMM_VEH_FLAG_HAZARD      : 0x00u));
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

    // Provenance for the OBD-II case, which nothing used to fill.
    //
    // VehSource::OBD2 is never assigned anywhere: OBD-II readings land in
    // OBD2Data, not in the VehicleSignals template, so packVehSourceByte() had
    // no way to see them and reported NONE for a speed the payload was carrying
    // perfectly well. A host reading sigSource then concluded there was no
    // speed source while `speed` held a live number - the two fields
    // contradicting each other, with the more authoritative-looking one wrong.
    //
    // Stamped here rather than by writing OBD2 into the template, because the
    // template's per-signal ages drive expireVehicleSignals() and OBD-II
    // freshness is already tracked, differently, by COMM_FLAG_OBD2_VALID.
    // Duplicating it would give one reading two clocks.
    if ((flags & COMM_FLAG_OBD2_VALID) != 0u) {
        if (veh.speedSrc == VehSource::NONE && !isnan(obd.speed)) {
            out.sigSource = (uint8_t)((out.sigSource & ~0x03u) | (uint8_t)VehSource::OBD2);
        }
        if (veh.rpmSrc == VehSource::NONE && !isnan(obd.rpm)) {
            out.sigSource = (uint8_t)((out.sigSource & ~0x0Cu) | ((uint8_t)VehSource::OBD2 << 2));
        }
        // Gear provenance is deliberately NOT stamped for OBD-II.
        //
        // sigSource's gear field qualifies `gearPos`, the VehGear SELECTOR
        // position, which only sniffing fills. OBD-II's PID 0xA4 gives a numeric
        // ratio-derived gear that lives in the separate `gear` float — a
        // different quantity in a different field. Marking the source OBD2 here
        // certified a gearPos of "unknown" as an OBD-II reading, which is worse
        // than leaving it NONE: NONE is true.
    }
}

CommReturnStatus sendTelemetry(const OBD2Data &obd, const GPSData &gps, const IMUData &imu, SwitchData &sw, const DerivedSignals &derived, const VehicleSignals &veh, uint8_t canMode){
    TelemetryPayload p;
    buildTelemetry(obd, gps, imu, sw, derived, veh, canMode, p);
    const CommReturnStatus r = sendFrame(MSG_TELEMETRY, reinterpret_cast<const uint8_t*>(&p), sizeof(p));

    // ONLY ON A CONFIRMED SEND (v0x07).
    //
    // sendFrame() returns NOK_BUSY when the frame did not fit the TX buffer, and
    // clearing on the ATTEMPT would discard the switch event along with the
    // frame that failed to carry it — after which the next frame reports a quiet
    // interval over a window in which a switch moved. Exactly the bug that was
    // found and fixed in the bridge's coalescing accumulator, which is the same
    // shape one hop further up.
    //
    // Held rather than cleared means at worst a change is reported twice, which
    // is a consumer's problem to idempotently ignore. The other way round loses
    // it permanently.
    if (r == CommReturnStatus::OK) switchClearChanged(sw);
    return r;
}

void initCommMaster(CommMaster &m){
    m.streaming     = false;
    m.lastPushMs    = 0;
    m.lastCommandMs = 0;
    m.oncePending   = false;
    m.onceRequestMs = 0;
    m.canModeRequest = 0;
    m.imuModeRequest = 0;
    m.imuModeAppliedMs = 0;
    commRxInit(m.rx);
}

void tickCommMaster(CommMaster &m, const OBD2Data &obd, const GPSData &gps, const IMUData &imu, SwitchData &sw, const DerivedSignals &derived, const VehicleSignals &veh, uint8_t canMode){
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
                if (sendTelemetry(obd, gps, imu, sw, derived, veh, canMode) != CommReturnStatus::OK){
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
            case CMD_SET_IMU_MODE:
                // Latched, not applied, and for a stronger reason than the CAN
                // mode above: applying it restarts the sensor's bring-up, which
                // is ~700 ms of the IMU publishing nothing. Doing that from
                // inside the frame decoder would stall the command budget and
                // the telemetry push behind it.
                if (payload[0] == COMM_IMU_MODE_FUSION || payload[0] == COMM_IMU_MODE_RAW){
                    m.imuModeRequest = payload[0];
                } else {
                    sendFrame(MSG_NACK, nullptr, 0);
                }
                break;
            case CMD_SET_CAN_FILTER: {
                // Latched, not applied — see CommMaster::canFilterPending.
                //
                // Validated at the wire boundary: a count past the hardware's
                // six slots, or an ID outside the 11-bit standard range, is
                // rejected here rather than reaching the CAN driver. This
                // vehicle uses standard IDs throughout, and an extended ID
                // silently truncated into a filter register would produce a
                // capture that quietly excludes traffic the host asked for.
                const uint8_t count = payload[0];
                bool ok = (count <= COMM_CAN_FILTER_SLOTS);
                for (uint8_t i = 0; ok && i < COMM_CAN_FILTER_SLOTS; ++i){
                    const uint16_t id = (uint16_t)(payload[1 + i * 2] |
                                                   ((uint16_t)payload[2 + i * 2] << 8));
                    // Entries past count are ignored but must still decode: the
                    // payload is fixed-length so the receiver can check its size
                    // before trusting any of it.
                    if (i < count && id > 0x7FFu) ok = false;
                }
                if (!ok){
                    sendFrame(MSG_NACK, nullptr, 0);
                    break;
                }
                for (uint8_t i = 0; i < COMM_CAN_FILTER_SLOTS; ++i){
                    m.canFilterIds[i] = (uint16_t)(payload[1 + i * 2] |
                                                   ((uint16_t)payload[2 + i * 2] << 8));
                }
                m.canFilterCount   = count;
                m.canFilterPending = true;
                break;
            }
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
        } else if (sendTelemetry(obd, gps, imu, sw, derived, veh, canMode) == CommReturnStatus::OK){
            m.oncePending = false;
        }
    }

    // 2) Streaming push at COMM_STREAM_INTERVAL_MS (unsigned subtraction is
    //    millis()-rollover safe, mirroring isTimeout() in TimerFunctions.h). Advance
    //    the clock only when the frame actually went out, so a NOK_BUSY (congested TX)
    //    retries on the next loop instead of being silently dropped for a full period.
    if (m.streaming && (millis() - m.lastPushMs) >= COMM_STREAM_INTERVAL_MS){
        if (sendTelemetry(obd, gps, imu, sw, derived, veh, canMode) == CommReturnStatus::OK){
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
