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

void buildTelemetry(const OBD2Data &obd, const GPSData &gps, TelemetryPayload &out){
    out.masterMillis = millis();

    out.speed       = obd.speed;
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
    out.heading     = gps.headingDegrees;
    out.satellites  = gps.satellites;
    out.fixType     = gps.fixType;
    out.fixValid    = gps.fixValid ? 1 : 0;

    out.year   = gps.utc.year;
    out.month  = gps.utc.month;
    out.day    = gps.utc.day;
    out.hour   = gps.utc.hour;
    out.minute = gps.utc.minute;
    out.second = gps.utc.second;

    uint8_t flags = 0;
    // OBD2 is "live" only if tickOBD2() stored a reading within the freshness window,
    // so the flag clears within OBD2_FRESH_WINDOW_MS of the ECU going quiet.
    if (obd.lastUpdateMs != 0 && (millis() - obd.lastUpdateMs) < OBD2_FRESH_WINDOW_MS) flags |= COMM_FLAG_OBD2_VALID;
    if (gps.fixValid)  flags |= COMM_FLAG_GPS_FIX;
    if (gps.utc.valid) flags |= COMM_FLAG_TIME_VALID;
    out.flags = flags;
}

CommReturnStatus sendTelemetry(const OBD2Data &obd, const GPSData &gps){
    TelemetryPayload p;
    buildTelemetry(obd, gps, p);
    return sendFrame(MSG_TELEMETRY, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
}

void initCommMaster(CommMaster &m){
    m.streaming  = false;
    m.lastPushMs = 0;
    commRxInit(m.rx);
}

void tickCommMaster(CommMaster &m, const OBD2Data &obd, const GPSData &gps){
    // 1) Service inbound C3 commands: non-blocking and bounded to a fixed budget
    //    per call so a command flood cannot monopolise the loop.
    uint8_t  type = 0, len = 0;
    uint8_t  payload[COMM_MAX_PAYLOAD];
    for (uint8_t serviced = 0; serviced < COMM_MAX_CMDS_PER_TICK; ++serviced){
        CommReturnStatus r = pollFrame(m.rx, Serial1, type, payload, sizeof(payload), len);
        if (r == CommReturnStatus::NO_DATA) break;          // input exhausted
        if (r != CommReturnStatus::FRAME_READY) continue;   // bad CRC/overflow: counts toward budget

        // Every defined command is zero-payload; a payload marks a malformed frame.
        if (len != 0){
            sendFrame(MSG_NACK, nullptr, 0);
            continue;
        }

        switch (type){
            case CMD_GET_ONCE:
                sendTelemetry(obd, gps);
                break;
            case CMD_START_STREAM:
                m.streaming  = true;
                m.lastPushMs = millis() - COMM_STREAM_INTERVAL_MS; // push promptly
                break;
            case CMD_STOP_STREAM:
                m.streaming = false;
                break;
            case CMD_PING:
                sendFrame(MSG_PONG, nullptr, 0);
                break;
            default:
                sendFrame(MSG_NACK, nullptr, 0);
                break;
        }
    }

    // 2) Streaming push at COMM_STREAM_INTERVAL_MS (unsigned subtraction is
    //    millis()-rollover safe, mirroring isTimeout() in TimerFunctions.h). Advance
    //    the clock only when the frame actually went out, so a NOK_BUSY (congested TX)
    //    retries on the next loop instead of being silently dropped for a full period.
    if (m.streaming && (millis() - m.lastPushMs) >= COMM_STREAM_INTERVAL_MS){
        if (sendTelemetry(obd, gps) == CommReturnStatus::OK){
            m.lastPushMs = millis();
        }
    }
}
