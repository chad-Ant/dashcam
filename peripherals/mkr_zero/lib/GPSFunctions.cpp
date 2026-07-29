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
    if (i2cBusBegin() != I2CBusState::Ready) return GPSReturnStatus::NOK_BUS_STUCK;

    // A zero deadline, and the result is deliberately discarded.  This call is
    // not trying to reach the receiver — it is here to bind Wire to the driver
    // and to run the "if (packetCfgPayloadSize == 0) setPacketCfgPayloadSize()"
    // branch inside begin(), both of which happen before the first probe.  With
    // maxWait 0 the three isConnected() attempts cost only their I2C address
    // probes, so this is bounded at a few hundred microseconds either way.
    (void)myGNSS.begin(Wire, GPS_DEFAULT_I2C_ADDRESS, 0u);

    // Allocates UBX_NAV_PVT_t and only then transmits, so the storage is claimed
    // whether or not anything is listening.  The rate set here is overwritten by
    // the real bring-up; only the allocation side effect is wanted.
    (void)myGNSS.setAutoPVTrate(1, true, 0u);

    return GPSReturnStatus::OK;
}

GPSReturnStatus initializeGPS_I2C(SFE_UBLOX_GNSS &myGNSS){
    // Enter through the shared bus manager rather than opening Wire here.  The
    // receiver is often the FIRST client on the bus, and whoever is first is
    // the one that has to unwedge it: a slave left holding SDA by an unclean
    // reset would otherwise hang myGNSS.begin() inside the SAMD driver's
    // unbounded flag wait, with no code left to run that could have recovered.
    if (i2cBusBegin() != I2CBusState::Ready) return GPSReturnStatus::NOK_BUS_STUCK;

    // Every exchange below carries an EXPLICIT deadline instead of SparkFun's
    // 1100 ms default, and the watchdog is fed between them.  Both halves are
    // needed.  The deadline is what keeps the total bounded — the chain is
    // twelve ACK waits deep, so at the default it can outlast WATCHDOG_PERIOD_MS
    // on a slow but SUCCESSFUL bring-up and reboot the board before the receiver
    // is ever usable.  The feeds are what keep the watchdog honest across a
    // legitimately slow bring-up without blinding it: each feed sits between two
    // individually bounded operations, so the hazard the watchdog actually
    // exists for — an unbounded SERCOM flag wait inside a single Wire call — is
    // still fully exposed to it.  Feeding inside a loop with no deadline is what
    // would defeat it, and there is none here.
    if (!myGNSS.begin(Wire, GPS_DEFAULT_I2C_ADDRESS, GPS_CMD_TIMEOUT_MS)) return GPSReturnStatus::NOK_INIT_FAILED;
    watchdogFeed();
#ifndef GPS_ENABLE_NMEA
    if (!myGNSS.setI2COutput(COM_TYPE_UBX, GPS_CMD_TIMEOUT_MS)) return GPSReturnStatus::NOK_CONFIG_FAILED;
    watchdogFeed();
#endif
    if (!myGNSS.setDynamicModel(DYN_MODEL_AUTOMOTIVE, GPS_CMD_TIMEOUT_MS)) return GPSReturnStatus::NOK_CONFIG_FAILED;
    watchdogFeed();
    if (!myGNSS.setNavigationFrequency(GPS_REFRESH_RATE, GPS_CMD_TIMEOUT_MS)) return GPSReturnStatus::NOK_SET_RATE_FAILED;
    watchdogFeed();
    if (!myGNSS.setNavigationRate(1, GPS_CMD_TIMEOUT_MS)) return GPSReturnStatus::NOK_CONFIG_FAILED;
    watchdogFeed();
    if (!myGNSS.setAutoPVTrate(1, true, GPS_CMD_TIMEOUT_MS)) return GPSReturnStatus::NOK_CONFIG_FAILED;
    watchdogFeed();

    return GPSReturnStatus::OK;
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
