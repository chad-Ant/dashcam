#include <Wire.h>
#include <TimeLib.h>
#include <math.h>

#include "DataDictionary.h"
#include "GlobalVariables.h"
#include "GPSFunctions.h"


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

GPSReturnStatus initializeGPS_I2C(SFE_UBLOX_GNSS &myGNSS){
    if (!i2cInitialized){
        Wire.begin();
        i2cInitialized = true;
    }

    if (!myGNSS.begin(Wire, GPS_DEFAULT_I2C_ADDRESS)) return GPSReturnStatus::NOK_INIT_FAILED;
#ifndef GPS_ENABLE_NMEA
    if (!myGNSS.setI2COutput(COM_TYPE_UBX)) return GPSReturnStatus::NOK_CONFIG_FAILED;
#endif
    if (!myGNSS.setDynamicModel(DYN_MODEL_AUTOMOTIVE)) return GPSReturnStatus::NOK_CONFIG_FAILED;
    if (!myGNSS.setNavigationFrequency(GPS_REFRESH_RATE)) return GPSReturnStatus::NOK_SET_RATE_FAILED;
    if (!myGNSS.setNavigationRate(1)) return GPSReturnStatus::NOK_CONFIG_FAILED;
    if (!myGNSS.setAutoPVTrate(1)) return GPSReturnStatus::NOK_CONFIG_FAILED;

    return GPSReturnStatus::OK;
}

void initGPSData(GPSData &data){
    data.utc.year = 0;
    data.utc.month = 0;
    data.utc.day = 0;
    data.utc.hour = 0;
    data.utc.minute = 0;
    data.utc.second = 0;
    data.utc.valid = false;

    data.velocityKmh = NAN;
    data.headingDegrees = NAN;
    data.latitudeDegrees = NAN;
    data.longitudeDegrees = NAN;
    data.altitudeM = NAN;
    data.satellites = 0;
    data.fixType = 0;
    data.fixValid = false;
}

GPSReturnStatus getGPSData(SFE_UBLOX_GNSS &myGNSS, GPSData &data){
    if (!myGNSS.getPVT(0)) return GPSReturnStatus::DATA_STALE;

    data.utc.year = myGNSS.getYear(0);
    data.utc.month = myGNSS.getMonth(0);
    data.utc.day = myGNSS.getDay(0);
    data.utc.hour = myGNSS.getHour(0);
    data.utc.minute = myGNSS.getMinute(0);
    data.utc.second = myGNSS.getSecond(0);
    data.utc.valid = myGNSS.getDateValid(0) && myGNSS.getTimeValid(0);

    data.satellites = myGNSS.getSIV(0);
    data.fixType = myGNSS.getFixType(0);
    data.fixValid = myGNSS.getGnssFixOk(0);

    if (!data.fixValid){
        data.velocityKmh = NAN;
        data.headingDegrees = NAN;
        data.latitudeDegrees = NAN;
        data.longitudeDegrees = NAN;
        data.altitudeM = NAN;
        return GPSReturnStatus::NO_FIX;
    }

    data.velocityKmh = static_cast<float>(myGNSS.getGroundSpeed(0)) * 0.0036f;
    data.headingDegrees = static_cast<float>(myGNSS.getHeading(0)) * 1e-5f;
    data.latitudeDegrees = static_cast<float>(myGNSS.getLatitude(0)) * 1e-7f;
    data.longitudeDegrees = static_cast<float>(myGNSS.getLongitude(0)) * 1e-7f;
    data.altitudeM = static_cast<float>(myGNSS.getAltitudeMSL(0)) * 0.001f;

    return GPSReturnStatus::OK;
}

GPSReturnStatus getLatLong(SFE_UBLOX_GNSS &myGNSS, float &latitude, float &longitude){
    if (!myGNSS.getPVT(0)) return GPSReturnStatus::DATA_STALE;
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
    if (!myGNSS.getPVT(0)) return GPSReturnStatus::DATA_STALE;
    if (!myGNSS.getGnssFixOk(0)){
        altitude = NAN;
        return GPSReturnStatus::NO_FIX;
    }
    altitude = (float)(myGNSS.getAltitudeMSL()) * 0.001f;
    return GPSReturnStatus::OK;
}

GPSReturnStatus getLatLongAlt(SFE_UBLOX_GNSS &myGNSS, float &latitude, float &longitude, float &altitude){
    if (!myGNSS.getPVT(0)) return GPSReturnStatus::DATA_STALE;
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
    if (!myGNSS.getPVT(0)) return GPSReturnStatus::DATA_STALE;
    if (!myGNSS.getGnssFixOk(0)){
        speed = NAN;
        return GPSReturnStatus::NO_FIX;
    }
    speed = (float)(myGNSS.getGroundSpeed()) * 0.0036f; // mm/s → km/h
    return GPSReturnStatus::OK;
}

GPSReturnStatus getHeading(SFE_UBLOX_GNSS &myGNSS, float &heading){
    if (!myGNSS.getPVT(0)) return GPSReturnStatus::DATA_STALE;
    if (!myGNSS.getGnssFixOk(0)){
        heading = NAN;
        return GPSReturnStatus::NO_FIX;
    }
    heading = (float)(myGNSS.getHeading()) * 1e-5f; // 1e-5 deg → deg
    return GPSReturnStatus::OK;
}

GPSReturnStatus getSpeedHeading(SFE_UBLOX_GNSS &myGNSS, float &speed, float &heading){
    if (!myGNSS.getPVT(0)) return GPSReturnStatus::DATA_STALE;
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
    if (!myGNSS.getPVT(0))       return GPSReturnStatus::DATA_STALE;
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
