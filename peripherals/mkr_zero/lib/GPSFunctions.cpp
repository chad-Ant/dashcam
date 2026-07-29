#include <Wire.h>
#include <TimeLib.h>
#include <math.h>

#include "DataDictionary.h"
#include "GlobalVariables.h"
#include "I2CBus.h"
#include "GPSFunctions.h"


// The host must read faster than the receiver produces, or the two same-rate
// clocks beat and polls periodically land just before a packet is ready. Stated
// as a compile-time rule because it is a RELATIONSHIP between two constants
// that live in different sections of DataDictionary.h: either one can be edited
// alone, and nothing at runtime would report the mistake — just a slow drip of
// stale polls and doubled worst-case read times.
static_assert((GPS_POLL_MS * GPS_REFRESH_RATE) < 1000UL,
              "GPS_POLL_MS must be shorter than the navigation period "
              "(1000 / GPS_REFRESH_RATE); polling at the production rate makes "
              "the two clocks beat and periodically miss a packet");

static void configureGNSSUART(SFE_UBLOX_GNSS &myGNSS, uint8_t freqHz){
#ifndef GPS_ENABLE_NMEA
    myGNSS.setUART1Output(COM_TYPE_UBX);
#endif
    myGNSS.setDynamicModel(DYN_MODEL_AUTOMOTIVE);
    myGNSS.setNavigationFrequency(freqHz);
    myGNSS.setNavigationRate(1);
    myGNSS.setAutoPVTrate(1);
}

GPSReturnStatus initializeGPS(SFE_UBLOX_GNSS &myGNSS){
    Serial1.begin(GPS_BAUDRATE_CUSTOM);
    if (myGNSS.begin(Serial1)){
        configureGNSSUART(myGNSS, GPS_REFRESH_RATE);
        return GPSReturnStatus::OK;
    }

    // Module still at factory baud; reprogram it to the custom rate and retry.
    Serial1.begin(GPS_BAUDRATE_DEFAULT);
    if (myGNSS.begin(Serial1)){
        myGNSS.setSerialRate(GPS_BAUDRATE_CUSTOM);
        myGNSS.saveConfiguration();
    }
    Serial1.begin(GPS_BAUDRATE_CUSTOM);
    if (myGNSS.begin(Serial1)){
        configureGNSSUART(myGNSS, GPS_REFRESH_RATE);
        return GPSReturnStatus::OK;
    }

    // Fall back to default baud at 1 Hz.
    Serial1.begin(GPS_BAUDRATE_DEFAULT);
    if (myGNSS.begin(Serial1)){
        configureGNSSUART(myGNSS, 1);
        return GPSReturnStatus::OK;
    }

    return GPSReturnStatus::NOK_INIT_FAILED;
}

GPSReturnStatus preallocateGPS_I2C(SFE_UBLOX_GNSS &myGNSS){
    // NOTHING here touches the bus, which is the whole point of the rewrite.
    //
    // This used to call begin(..., 0) and setAutoPVTrate(..., 0) for their
    // allocation side effects.  Both transact: begin() probes the address and
    // then polls CFG-PRT, setAutoPVTrate() sends CFG-MSG, and a zero deadline
    // means neither waits for the reply it just asked for.  The receiver still
    // queues those replies, so the REAL bring-up moments later could parse an
    // answer to a question this function asked — and could pass on it.  A
    // pre-flight step that leaves stale traffic in the DDC buffer is worse than
    // no pre-flight step at all.
    //
    // Both allocations are reachable without any of that:
    //   setPacketCfgPayloadSize() allocates payloadCfg directly.
    //   assumeAutoPVT() calls initPacketUBXNAVPVT() and then only writes flags.
    // Neither sends a byte, so this is safe to call before the bus is known good
    // and cannot hang in the SAMD driver's undeadlined waits.
    if (!myGNSS.setPacketCfgPayloadSize(MAX_PAYLOAD_SIZE)) return GPSReturnStatus::NOK_INIT_FAILED;

    // false, not true: this claims the RAM without asserting that automatic PVT
    // is actually running.  Setting the flag before the real CFG-MSG has been
    // acknowledged would make getPVT() believe the receiver is streaming when a
    // failed bring-up means it is not.
    //
    // The return is deliberately NOT the allocation result — assumeAutoPVT()
    // returns whether the FLAGS CHANGED, and passing false when they are already
    // false legitimately returns false.  So allocation success is checked
    // directly instead, via the pointer the call exists to populate.
    (void)myGNSS.assumeAutoPVT(false, true);
    if (myGNSS.packetUBXNAVPVT == nullptr) return GPSReturnStatus::NOK_INIT_FAILED;

    return GPSReturnStatus::OK;
}

// ─── staged bring-up ──────────────────────────────────────────────────────────

const char *gpsInitStageName(GPSInitStage stage){
    switch (stage){
        case GPSInitStage::Idle:          return "idle";
        case GPSInitStage::Begin:         return "begin";
        case GPSInitStage::SetI2COutput:  return "i2c-output";
        case GPSInitStage::SetDynModel:   return "dynamic-model";
        case GPSInitStage::SetNavFreq:    return "nav-frequency";
        case GPSInitStage::SetNavRate:    return "nav-rate";
        case GPSInitStage::SetAutoPVT:    return "auto-pvt";
        case GPSInitStage::Done:          return "done";
        case GPSInitStage::Quarantined:   return "quarantined";
        default:                          return "failed";
    }
}

void gpsInitBegin(GPSInitState &state){
    // Quarantined is TERMINAL, and it has to be terminal here rather than only
    // at the one call site that currently respects it.  gpsInitTick() refusing
    // to leave the state is not enough on its own: this function assigns the
    // stage directly, so a caller that armed the machine again would walk it
    // straight back into the hang the quarantine exists to prevent.
    if (state.stage == GPSInitStage::Quarantined) return;

    state.stage      = GPSInitStage::Begin;
    state.lastStatus = GPSReturnStatus::OK;
    state.nextStepMs = millis();
}

void gpsInitFail(GPSInitState &state, GPSReturnStatus why){
    // Same reason as gpsInitBegin(): Failed schedules a retry, so downgrading a
    // quarantine to a failure would silently re-arm the machine.
    if (state.stage == GPSInitStage::Quarantined) return;

    // Remember WHICH step refused before overwriting the stage, or the log can
    // only report "failed", which says nothing: -1 at Begin means nothing
    // answered at 0x42, while -6 at SetNavRate means the receiver is right there
    // and rejected a setting.  Those are opposite repairs.
    if (state.stage != GPSInitStage::Failed) state.failedAt = state.stage;

    state.stage      = GPSInitStage::Failed;
    state.lastStatus = why;
    if (state.failures < 0xFFFFu) state.failures++;

    // Escalating backoff.  The first several retries come quickly because this
    // bring-up is known to refuse and then succeed unchanged; only once that
    // stops looking like flakiness does the interval stretch out to the rate
    // used for hardware that is genuinely absent.
    state.nextStepMs = millis() + ((state.failures <= GPS_INIT_FAST_RETRIES)
                                       ? GPS_INIT_FAST_RETRY_MS
                                       : GPS_RETRY_MS);
}

/**
 * @brief Runs the next stage. ONE STAGE per call — which is not one exchange.
 *
 * The bring-up used to be a straight-line function holding the CPU for as long
 * as the receiver took to answer twelve exchanges — up to 3 s even with bounded
 * deadlines, and unbounded if the bus wedged mid-way.  Nothing else ran during
 * that: not the C3 link, not the IMU drain, not the watchdog feed in loop().
 * Two separate defects came out of it.  A hang repeated identically on every
 * boot because the same stage ran at the same point under the same armed
 * watchdog; and a 3 s block outlasts the LSM6DSOX FIFO's 2.32 s depth, so the
 * drain afterwards published seconds-old backlog stamped with the current time.
 *
 * Splitting the chain across loop() bounds it, but does NOT reduce it to one
 * exchange, and an earlier version of this comment wrongly claimed it did.  A
 * stage is up to 750 ms (@c Begin, three isConnected probes) or 500 ms (the
 * poll-plus-set setters) — see @c gpsInitTick() in the header for the counted
 * breakdown.  So other subsystems still pause for that long during bring-up,
 * and a pause beyond IMU_MAX_DATA_AGE_MS is correctly reported as an inertial
 * data gap rather than hidden.  That is accepted behaviour, not a closed
 * optimisation: driving it under the freshness window needs raw UBX substages
 * in place of the library's setters.
 */
void gpsInitConfirmStreaming(GPSInitState &state){
    // Called when a PVT packet actually arrives.  This — not a successful
    // configuration exchange — is what proves the receiver is working, so it is
    // the only thing entitled to clear the backoff.
    state.failures = 0u;
}

void gpsInitQuarantine(GPSInitState &state){
    state.stage      = GPSInitStage::Quarantined;
    state.failedAt   = GPSInitStage::Begin;
    state.lastStatus = GPSReturnStatus::NOK_BUS_STUCK;
}

GPSInitStage gpsInitTick(SFE_UBLOX_GNSS &myGNSS, GPSInitState &state){
    if (state.stage == GPSInitStage::Done) return state.stage;

    // Terminal, checked FIRST and with no timer.  This is the state that breaks
    // a persistent reboot loop, and it can only do that by never leaving: an
    // earlier version put the machine into Failed here instead, which re-entered
    // Begin 250 ms later, hung again, and reset the board again.  The loop was
    // never broken, only slowed to the retry interval.
    if (state.stage == GPSInitStage::Quarantined) return state.stage;

    // Failed is a WAITING state, not a terminal one: it holds the retry backoff
    // and re-enters Begin once GPS_RETRY_MS has passed.  Structured this way so
    // the caller never has to remember to restart the machine.
    if (state.stage == GPSInitStage::Failed || state.stage == GPSInitStage::Idle){
        if (static_cast<int32_t>(millis() - state.nextStepMs) < 0) return state.stage;
        gpsInitBegin(state);
        return state.stage;
    }

    // The bus check is per STEP, not once per bring-up.  A slave can wedge the
    // lines between two stages just as easily as before the first one.
    if (i2cBusBegin() != I2CBusState::Ready){
        gpsInitFail(state, GPSReturnStatus::NOK_BUS_STUCK);
        return state.stage;
    }

    switch (state.stage){
        case GPSInitStage::Begin:
            // Buffer readiness enforced HERE, in the library, not left to each
            // caller.  Production checks preallocateGPS_I2C() and quarantines on
            // failure, but the helper sketches call the blocking wrapper and one
            // of them discarded the result entirely — and the consequence is not
            // a clean failure.  setPacketCfgPayloadSize() leaves payloadCfg NULL
            // on OOM, begin() retries it without checking, and the configuration
            // path then writes payloadCfg[0] unconditionally.  A null dereference
            // is not something a caller can be trusted to avoid by convention.
            //
            // packetUBXNAVPVT is the one allocation this code can observe (the
            // payload members are private), and it is allocated by the same
            // preallocation step, so it stands in for both.
            if (myGNSS.packetUBXNAVPVT == nullptr){
                gpsInitFail(state, GPSReturnStatus::NOK_INIT_FAILED);
                break;
            }
            if (!myGNSS.begin(Wire, GPS_DEFAULT_I2C_ADDRESS, GPS_CMD_TIMEOUT_MS)){
                gpsInitFail(state, GPSReturnStatus::NOK_INIT_FAILED);
                break;
            }
#ifdef GPS_ENABLE_NMEA
            state.stage = GPSInitStage::SetDynModel;
#else
            state.stage = GPSInitStage::SetI2COutput;
#endif
            break;

        case GPSInitStage::SetI2COutput:
            if (!myGNSS.setI2COutput(COM_TYPE_UBX, GPS_CMD_TIMEOUT_MS)){
                gpsInitFail(state, GPSReturnStatus::NOK_CONFIG_FAILED);
                break;
            }
            state.stage = GPSInitStage::SetDynModel;
            break;

        case GPSInitStage::SetDynModel:
            if (!myGNSS.setDynamicModel(DYN_MODEL_AUTOMOTIVE, GPS_CMD_TIMEOUT_MS)){
                gpsInitFail(state, GPSReturnStatus::NOK_CONFIG_FAILED);
                break;
            }
            state.stage = GPSInitStage::SetNavFreq;
            break;

        case GPSInitStage::SetNavFreq:
            if (!myGNSS.setNavigationFrequency(GPS_REFRESH_RATE, GPS_CMD_TIMEOUT_MS)){
                gpsInitFail(state, GPSReturnStatus::NOK_SET_RATE_FAILED);
                break;
            }
            state.stage = GPSInitStage::SetNavRate;
            break;

        case GPSInitStage::SetNavRate:
            if (!myGNSS.setNavigationRate(1, GPS_CMD_TIMEOUT_MS)){
                gpsInitFail(state, GPSReturnStatus::NOK_CONFIG_FAILED);
                break;
            }
            state.stage = GPSInitStage::SetAutoPVT;
            break;

        case GPSInitStage::SetAutoPVT:
            if (!myGNSS.setAutoPVTrate(1, true, GPS_CMD_TIMEOUT_MS)){
                gpsInitFail(state, GPSReturnStatus::NOK_CONFIG_FAILED);
                break;
            }
            state.stage      = GPSInitStage::Done;
            state.lastStatus = GPSReturnStatus::OK;
            // The backoff counter is deliberately NOT reset here.  Reaching Done
            // only means the receiver ACKed its configuration; it does not mean
            // a single PVT packet has arrived.  A receiver that accepts every
            // setting and then streams nothing would otherwise sit in a loop
            // forever at the FAST retry rate: silence window fires, retry
            // succeeds, counter clears, silence window fires again — the
            // escalation to GPS_RETRY_MS could never happen because the counter
            // was wiped before it could grow.  gpsInitConfirmStreaming() clears
            // it, and only real data calls that.
            break;

        default:
            break;
    }

    return state.stage;
}

GPSReturnStatus initializeGPS_I2C(SFE_UBLOX_GNSS &myGNSS){
    // Blocking wrapper around the staged machine, kept for callers that have no
    // loop() to drive it — the validation and fault-injection sketches.  The
    // production sketch must NOT use this: see gpsInitTick() for why a
    // straight-line bring-up is a hazard there.
    // Preallocation is performed HERE rather than assumed.  Every caller of this
    // wrapper is a helper sketch, and they did not all check it: one discarded
    // the result and two never called it at all, so they could enter SparkFun's
    // unchecked null-payload path.  Doing it inside the wrapper means the
    // guarantee belongs to the function rather than to whoever remembers.
    const GPSReturnStatus alloc = preallocateGPS_I2C(myGNSS);
    if (alloc != GPSReturnStatus::OK) return alloc;

    GPSInitState state;
    gpsInitBegin(state);

    // Bounded by construction: the chain is a fixed number of stages, each one
    // GPS_CMD_TIMEOUT_MS, and Failed short-circuits rather than retrying here.
    for (uint8_t guard = 0u; guard < GPS_INIT_MAX_STEPS; guard++){
        const GPSInitStage stage = gpsInitTick(myGNSS, state);
        watchdogFeed();
        if (stage == GPSInitStage::Done)   return GPSReturnStatus::OK;
        if (stage == GPSInitStage::Failed) return state.lastStatus;
    }
    return GPSReturnStatus::NOK_INIT_FAILED;
}

void invalidateGPSFix(GPSData &data){
    data.velocityKmh      = NAN;
    data.headingDegrees   = NAN;
    data.latitudeDegrees  = NAN;
    data.longitudeDegrees = NAN;
    data.altitudeM        = NAN;
    data.fixValid         = false;
}

void initGPSData(GPSData &data){
    data.utc.year = 0;
    data.utc.month = 0;
    data.utc.day = 0;
    data.utc.hour = 0;
    data.utc.minute = 0;
    data.utc.second = 0;
    data.utc.valid = false;

    invalidateGPSFix(data);

    data.satellites = 0;
    data.fixType = 0;
    data.devicePresent = false;
}

/// Widest values a genuine terrestrial fix can carry.  Not a plausibility filter
/// on the VEHICLE — a dashcam has no business deciding a speed is too high — but
/// a last check that the numbers came out of a well-formed packet at all.  A
/// corrupted UBX payload that happened to satisfy every validity flag is the case
/// this catches, and it is the only one it is meant to.
static constexpr float GPS_MIN_ALTITUDE_M   = -1000.0f;   ///< Below the Dead Sea shore.
static constexpr float GPS_MAX_ALTITUDE_M   = 20000.0f;   ///< Above any road on Earth.
static constexpr float GPS_MAX_SPEED_KMH    = 1000.0f;

/** @brief True when a decoded fix lies inside the physically possible ranges. */
static bool fixInRange(const GPSData &data){
    return (data.latitudeDegrees  >= -90.0f)  && (data.latitudeDegrees  <= 90.0f)  &&
           (data.longitudeDegrees >= -180.0f) && (data.longitudeDegrees <= 180.0f) &&
           (data.altitudeM  >= GPS_MIN_ALTITUDE_M) && (data.altitudeM <= GPS_MAX_ALTITUDE_M) &&
           (data.velocityKmh >= 0.0f) && (data.velocityKmh <= GPS_MAX_SPEED_KMH) &&
           (data.headingDegrees >= 0.0f) && (data.headingDegrees <= 360.0f);
}

/**
 * @brief True when the receiver's own flags say this packet's position is usable.
 *
 * Three tests, none of which subsumes the others.  @c fixType rejects the
 * no-fix and time-only modes, in which the receiver still reports its last known
 * position rather than nothing.  @c gnssFixOK is the receiver's within-limits
 * verdict on the solution.  @c invalidLlh exists precisely because the first two
 * can pass while longitude, latitude and height are individually unusable, which
 * is why u-blox gave it a separate bit.
 *
 * @c getInvalidLlh(0) is non-blocking here: the caller has just taken a fresh
 * packet with @c getPVT(0), so the field is already resident and no second poll
 * is issued.
 */
static bool fixFlagsUsable(SFE_UBLOX_GNSS &myGNSS, const GPSData &data){
    const bool fixTypeUsable = (data.fixType >= 2u) && (data.fixType <= 4u);
    return fixTypeUsable && myGNSS.getGnssFixOk(0) && !myGNSS.getInvalidLlh(0);
}

/**
 * @brief Shared preamble for every read: bus safe to touch, packet waiting.
 *
 * Bring-up is not the only moment the bus can be wedged.  A slave that browns
 * out or resets mid-drive holds SDA from that instant on, and every poll after
 * it walks into @c SERCOM::startTransmissionWIRE()'s
 * @code while (!isBusIdleWIRE() && !isBusOwnerWIRE()); @endcode
 * — a wait with no deadline, living in the core where no vendoring reaches.
 *
 * Guarding only the bring-up entry points left the watchdog as the sole answer
 * for the steady state, and a reset is not the behaviour this system is required
 * to have: it is supposed to keep running and log the event.  Two register reads
 * buy that.
 *
 * @return @c OK when a fresh PVT packet is available, @c DATA_STALE when the bus
 *         is healthy but nothing is buffered, @c NOK_BUS_STUCK when a line is
 *         held low and nothing was attempted.
 */
static GPSReturnStatus pollGuard(SFE_UBLOX_GNSS &myGNSS){
    if (i2cBusBegin() != I2CBusState::Ready) return GPSReturnStatus::NOK_BUS_STUCK;
    return myGNSS.getPVT(0) ? GPSReturnStatus::OK : GPSReturnStatus::DATA_STALE;
}

GPSReturnStatus getGPSData(SFE_UBLOX_GNSS &myGNSS, GPSData &data){
    const GPSReturnStatus guard = pollGuard(myGNSS);
    if (guard != GPSReturnStatus::OK) return guard;

    data.utc.year = myGNSS.getYear(0);
    data.utc.month = myGNSS.getMonth(0);
    data.utc.day = myGNSS.getDay(0);
    data.utc.hour = myGNSS.getHour(0);
    data.utc.minute = myGNSS.getMinute(0);
    data.utc.second = myGNSS.getSecond(0);
    data.utc.valid = myGNSS.getDateValid(0) && myGNSS.getTimeValid(0);

    data.satellites = myGNSS.getSIV(0);
    data.fixType = myGNSS.getFixType(0);

    if (!fixFlagsUsable(myGNSS, data)){
        invalidateGPSFix(data);
        return GPSReturnStatus::NO_FIX;
    }

    data.velocityKmh = static_cast<float>(myGNSS.getGroundSpeed(0)) * 0.0036f;
    data.headingDegrees = static_cast<float>(myGNSS.getHeading(0)) * 1e-5f;
    data.latitudeDegrees = static_cast<float>(myGNSS.getLatitude(0)) * 1e-7f;
    data.longitudeDegrees = static_cast<float>(myGNSS.getLongitude(0)) * 1e-7f;
    data.altitudeM = static_cast<float>(myGNSS.getAltitudeMSL(0)) * 0.001f;

    // Published only once the decoded numbers have been read back and checked,
    // so fixValid can never be true over a field this function itself rejected.
    if (!fixInRange(data)){
        invalidateGPSFix(data);
        return GPSReturnStatus::NO_FIX;
    }

    data.fixValid = true;
    return GPSReturnStatus::OK;
}

GPSReturnStatus getLatLong(SFE_UBLOX_GNSS &myGNSS, float &latitude, float &longitude){
    const GPSReturnStatus guard = pollGuard(myGNSS);
    if (guard != GPSReturnStatus::OK) return guard;
    if (!myGNSS.getGnssFixOk(0)){
        latitude = NAN;
        longitude = NAN;
        return GPSReturnStatus::NO_FIX;
    }
    latitude  = (float)(myGNSS.getLatitude())  * 1e-7f;
    longitude = (float)(myGNSS.getLongitude()) * 1e-7f;
    return GPSReturnStatus::OK;
}

GPSReturnStatus getAlt(SFE_UBLOX_GNSS &myGNSS, float &altitude){
    const GPSReturnStatus guard = pollGuard(myGNSS);
    if (guard != GPSReturnStatus::OK) return guard;
    if (!myGNSS.getGnssFixOk(0)){
        altitude = NAN;
        return GPSReturnStatus::NO_FIX;
    }
    altitude = (float)(myGNSS.getAltitudeMSL()) * 0.001f;
    return GPSReturnStatus::OK;
}

GPSReturnStatus getLatLongAlt(SFE_UBLOX_GNSS &myGNSS, float &latitude, float &longitude, float &altitude){
    const GPSReturnStatus guard = pollGuard(myGNSS);
    if (guard != GPSReturnStatus::OK) return guard;
    if (!myGNSS.getGnssFixOk(0)){
        latitude = NAN;
        longitude = NAN;
        altitude = NAN;
        return GPSReturnStatus::NO_FIX;
    }
    latitude  = (float)(myGNSS.getLatitude())   * 1e-7f;
    longitude = (float)(myGNSS.getLongitude())  * 1e-7f;
    altitude  = (float)(myGNSS.getAltitudeMSL()) * 0.001f;
    return GPSReturnStatus::OK;
}

GPSReturnStatus getSpeed(SFE_UBLOX_GNSS &myGNSS, float &speed){
    const GPSReturnStatus guard = pollGuard(myGNSS);
    if (guard != GPSReturnStatus::OK) return guard;
    if (!myGNSS.getGnssFixOk(0)){
        speed = NAN;
        return GPSReturnStatus::NO_FIX;
    }
    speed = (float)(myGNSS.getGroundSpeed()) * 0.0036f; // mm/s → km/h
    return GPSReturnStatus::OK;
}

GPSReturnStatus getHeading(SFE_UBLOX_GNSS &myGNSS, float &heading){
    const GPSReturnStatus guard = pollGuard(myGNSS);
    if (guard != GPSReturnStatus::OK) return guard;
    if (!myGNSS.getGnssFixOk(0)){
        heading = NAN;
        return GPSReturnStatus::NO_FIX;
    }
    heading = (float)(myGNSS.getHeading()) * 1e-5f; // 1e-5 deg → deg
    return GPSReturnStatus::OK;
}

GPSReturnStatus getSpeedHeading(SFE_UBLOX_GNSS &myGNSS, float &speed, float &heading){
    const GPSReturnStatus guard = pollGuard(myGNSS);
    if (guard != GPSReturnStatus::OK) return guard;
    if (!myGNSS.getGnssFixOk(0)){
        speed = NAN;
        heading = NAN;
        return GPSReturnStatus::NO_FIX;
    }
    speed   = (float)(myGNSS.getGroundSpeed()) * 0.0036f; // mm/s → km/h
    heading = (float)(myGNSS.getHeading())     * 1e-5f;  // 1e-5 deg → deg
    return GPSReturnStatus::OK;
}

GPSReturnStatus setAcquisitionFrequency(SFE_UBLOX_GNSS &myGNSS, uint8_t rateHz){
    rateHz = rateHz >= 1 ? (rateHz < 10 ? rateHz : 10) : 1;
    return myGNSS.setNavigationFrequency(rateHz) ? GPSReturnStatus::OK : GPSReturnStatus::NOK_SET_RATE_FAILED;
}

GPSSignalStrength evaluateSignal(const GPSData &data){
    if (!data.fixValid || data.fixType == 0) return GPSSignalStrength::NOSIGNAL;

    switch (data.satellites) {
        case 0: case 1: case 2: return GPSSignalStrength::NOSIGNAL;
        case 3:                 return GPSSignalStrength::BAD;
        case 4: case 5:         return GPSSignalStrength::AVERAGE;
        case 6: case 7: case 8: return GPSSignalStrength::GOOD;
        default:                return GPSSignalStrength::EXCELLENT;
    }
}

GPSReturnStatus getGPSDateTime(SFE_UBLOX_GNSS &myGNSS,
                               uint8_t &tHour, uint8_t &tMinute, uint8_t &tSecond,
                               uint8_t &tDate, uint8_t &tMonth, uint16_t &tYear,
                               int8_t timezone)
{
    const GPSReturnStatus guard = pollGuard(myGNSS);
    if (guard != GPSReturnStatus::OK) return guard;
    if (!myGNSS.getTimeValid(0)) return GPSReturnStatus::NOK_TIME_INVALID;
    if (!myGNSS.getDateValid(0)) return GPSReturnStatus::NOK_TIME_INVALID;

    timezone = timezone < -12 ? -12 : (timezone > 12 ? 12 : timezone);

    tmElements_t te;
    te.Year   = CalendarYrToTm(myGNSS.getYear(0));
    te.Month  = myGNSS.getMonth(0);
    te.Day    = myGNSS.getDay(0);
    te.Hour   = myGNSS.getHour(0);
    te.Minute = myGNSS.getMinute(0);
    te.Second = myGNSS.getSecond(0);
    time_t t  = makeTime(te) + (long)timezone * 3600L;

    tYear   = year(t);
    tMonth  = month(t);
    tDate   = day(t);
    tHour   = hour(t);
    tMinute = minute(t);
    tSecond = second(t);
    return GPSReturnStatus::OK;
}
