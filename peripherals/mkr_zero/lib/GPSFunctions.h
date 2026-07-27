#ifndef GPS_FUNCTIONS
#define GPS_FUNCTIONS 1

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

/// GPS module uses Serial1 on MKR 1000 WiFi, or I2C on MKR Zero.

/** Return codes used by GPS functions. */
enum class GPSReturnStatus{
    OK = 0,                   ///< Operation succeeded.
    NOK_INIT_FAILED = -1,     ///< Module did not respond during initialisation.
    DATA_STALE = 1,           ///< @c getPVT() returned false (no fresh fix).
    NO_FIX = 2,               ///< Fresh PVT received, but no valid GNSS fix is available.
    NOK_SET_RATE_FAILED = -2, ///< Navigation rate could not be changed.
    NOK_TIME_INVALID = -3,    ///< PVT received but time or date validity flags not set.
    NOK_CONFIG_FAILED = -6,   ///< Module responded but rejected its I2C/PVT configuration.
};


/** Qualitative GPS signal strength based on number of satellites in view. */
enum GPSSignalStrength{
    EXCELLENT, ///< 9+ satellites.
    GOOD,      ///< 6–8 satellites.
    AVERAGE,   ///< 4–5 satellites.
    BAD,       ///< 3 satellites (fix possible but unreliable).
    NOSIGNAL   ///< 0–2 satellites (no position fix possible).
};

/** UTC date and time reported by the GNSS receiver. */
struct GPSDateTime {
    uint16_t year;  ///< Four-digit UTC year.
    uint8_t month;  ///< UTC month [1, 12].
    uint8_t day;    ///< UTC day of month [1, 31].
    uint8_t hour;   ///< UTC hour [0, 23].
    uint8_t minute; ///< UTC minute [0, 59].
    uint8_t second; ///< UTC second [0, 60], including a possible leap second.
    bool valid;     ///< True when both GNSS date and time are valid.
};

/**
 * @brief One coherent UBX-NAV-PVT navigation snapshot.
 *
 * Position, velocity, heading, and altitude are @c NAN unless @c fixValid is
 * true, preventing no-fix coordinates from being used as measurements.
 */
struct GPSData {
    GPSDateTime utc;          ///< Receiver UTC date and time.
    float velocityKmh;        ///< Two-dimensional ground speed in km/h.
    float headingDegrees;     ///< Course over ground in degrees [0, 360).
    float latitudeDegrees;    ///< WGS-84 latitude in decimal degrees.
    float longitudeDegrees;   ///< WGS-84 longitude in decimal degrees.
    float altitudeM;          ///< Altitude above mean sea level in metres.
    uint8_t satellites;       ///< Satellites used in the navigation solution.
    uint8_t fixType;          ///< u-blox fix type (0=no fix, 2=2D, 3=3D, 4=GNSS+DR).
    bool fixValid;            ///< True when the receiver marks the GNSS fix valid.
};

/**
 * @brief Initialises the u-blox GNSS module over Serial1 (UART).
 *
 * Attempts connection at the custom baud rate first; if that fails it
 * reconfigures the module to the custom rate at default baud, then retries.
 * Falls back to 9600 baud as a last resort.  Sets automotive dynamic model,
 * navigation frequency, and UBX-only output.
 *
 * @param[in,out] myGNSS  SparkFun GNSS object to initialise.
 * @return @c GPSReturnStatus::OK on success, @c NOK_INIT_FAILED if the
 *         module could not be reached at any baud rate.
 */
GPSReturnStatus initializeGPS(SFE_UBLOX_GNSS &myGNSS);

/**
 * @brief Initialises the u-blox GNSS module over I2C.
 *
 * Connects at the standard u-blox I2C address (0x42). Initialises the I2C bus
 * if not already done.
 *
 * @param[in,out] myGNSS  SparkFun GNSS object to initialise.
 * @return @c GPSReturnStatus::OK on success, @c NOK_INIT_FAILED if the receiver
 *         does not respond at 0x42.
 */
GPSReturnStatus initializeGPS_I2C(SFE_UBLOX_GNSS &myGNSS);

/** Resets a GPS snapshot to a known invalid state. */
void initGPSData(GPSData &data);

/**
 * @brief Reads all requested navigation data from one automatic PVT packet.
 *
 * Non-blocking: @c getPVT(0) checks for a fresh automatic UBX-NAV-PVT packet.
 * UTC and satellite fields update for every fresh packet. Position, speed,
 * heading, and altitude update only when the receiver reports a valid fix.
 *
 * @return @c OK for a valid fix, @c NO_FIX for a fresh packet without a valid
 *         fix, or @c DATA_STALE when no new packet is available.
 */
GPSReturnStatus getGPSData(SFE_UBLOX_GNSS &myGNSS, GPSData &data);

/**
 * @brief Reads the current WGS-84 latitude and longitude (non-blocking).
 *
 * Calls @c getPVT(0) — returns immediately with @c DATA_STALE if no fresh
 * packet is buffered.  Requires @c setAutoPVTrate(1) during initialisation so
 * the module pushes PVT data automatically at the navigation rate.
 *
 * @param[in]  myGNSS    Initialised GNSS object.
 * @param[out] latitude  Latitude in decimal degrees (negative = South).
 * @param[out] longitude Longitude in decimal degrees (negative = West).
 * @return @c OK, @c DATA_STALE, or @c NO_FIX. Outputs are set to @c NAN when
 *         a fresh packet does not contain a valid fix.
 */
GPSReturnStatus getLatLong(SFE_UBLOX_GNSS &myGNSS, float &latitude, float &longitude);

/**
 * @brief Reads the current altitude above mean sea level (non-blocking).
 *
 * Calls @c getPVT(0) — returns immediately with @c DATA_STALE if no fresh
 * packet is buffered.
 *
 * @param[in]  myGNSS   Initialised GNSS object.
 * @param[out] altitude Altitude in metres (MSL).
 * @return @c OK, @c DATA_STALE, or @c NO_FIX. Output is set to @c NAN when a
 *         fresh packet does not contain a valid fix.
 */
GPSReturnStatus getAlt(SFE_UBLOX_GNSS &myGNSS, float &altitude);

/**
 * @brief Reads latitude, longitude, and altitude in a single non-blocking call.
 *
 * Calls @c getPVT(0) once; the individual getters return cached values from
 * that packet without additional I2C traffic.  Returns @c DATA_STALE
 * immediately if no fresh packet is buffered.
 *
 * @param[in]  myGNSS    Initialised GNSS object.
 * @param[out] latitude  Latitude in decimal degrees.
 * @param[out] longitude Longitude in decimal degrees.
 * @param[out] altitude  Altitude in metres (MSL).
 * @return @c OK, @c DATA_STALE, or @c NO_FIX. Outputs are set to @c NAN when
 *         a fresh packet does not contain a valid fix.
 */
GPSReturnStatus getLatLongAlt(SFE_UBLOX_GNSS &myGNSS, float &latitude, float &longitude, float &altitude);

/**
 * @brief Reads the current 2-D ground speed (non-blocking).
 *
 * Calls @c getPVT(0) — returns immediately with @c DATA_STALE if no fresh
 * packet is buffered.
 *
 * @param[in]  myGNSS  Initialised GNSS object.
 * @param[out] speed   Ground speed in km/h.
 * @return @c OK, @c DATA_STALE, or @c NO_FIX. Output is set to @c NAN when a
 *         fresh packet does not contain a valid fix.
 */
GPSReturnStatus getSpeed(SFE_UBLOX_GNSS &myGNSS, float &speed);

/**
 * @brief Reads the current course over ground / heading (non-blocking).
 *
 * Calls @c getPVT(0) — returns immediately with @c DATA_STALE if no fresh
 * packet is buffered.
 *
 * @param[in]  myGNSS   Initialised GNSS object.
 * @param[out] heading  Heading in decimal degrees (0–360, true north).
 * @return @c OK, @c DATA_STALE, or @c NO_FIX. Output is set to @c NAN when a
 *         fresh packet does not contain a valid fix.
 */
GPSReturnStatus getHeading(SFE_UBLOX_GNSS &myGNSS, float &heading);

/**
 * @brief Reads ground speed and heading in a single non-blocking call.
 *
 * Calls @c getPVT(0) once; the individual getters return cached values from
 * that packet without additional I2C traffic.  Returns @c DATA_STALE
 * immediately if no fresh packet is buffered.
 *
 * @param[in]  myGNSS   Initialised GNSS object.
 * @param[out] speed    Ground speed in km/h.
 * @param[out] heading  Heading in decimal degrees.
 * @return @c OK, @c DATA_STALE, or @c NO_FIX. Outputs are set to @c NAN when
 *         a fresh packet does not contain a valid fix.
 */
GPSReturnStatus getSpeedHeading(SFE_UBLOX_GNSS &myGNSS, float &speed, float &heading);

/**
 * @brief Changes the navigation solution output rate.
 *
 * @param[in,out] myGNSS  Initialised GNSS object.
 * @param[in]     rateHz  Desired update rate in Hz, clamped to [1, 10].
 * @return @c GPSReturnStatus::OK on success, @c NOK_SET_RATE_FAILED if the
 *         module rejected the new rate.
 */
GPSReturnStatus setAcquisitionFrequency(SFE_UBLOX_GNSS &myGNSS, uint8_t rateHz);

/**
 * @brief Estimates GPS signal quality from the number of satellites in view.
 *
 * Uses an already-acquired snapshot and therefore does not consume or depend
 * on the SparkFun library's fresh-PVT flag.
 *
 * @param[in] data  Navigation snapshot returned by @c getGPSData().
 * @return A @c GPSSignalStrength enum value.
 */
GPSSignalStrength evaluateSignal(const GPSData &data);

/**
 * @brief Reads the current UTC date and time from the u-blox GNSS module and
 *        applies a whole-hour timezone offset.
 *
 * Non-blocking: calls @c getPVT(0), returns @c DATA_STALE if no fresh packet is
 * buffered, and returns @c NOK_TIME_INVALID if the packet's time/date validity
 * flags are not set. Uses TimeLib @c makeTime() so timezone offsets that cross
 * midnight are handled correctly.
 *
 * Requires @c setAutoPVTrate(1) during initialisation so the module pushes
 * PVT packets automatically at the navigation rate.
 *
 * @param[in]  myGNSS    Initialised GNSS object.
 * @param[out] tHour     Local hours (0–23).
 * @param[out] tMinute   Local minutes (0–59).
 * @param[out] tSecond   Local seconds (0–59).
 * @param[out] tDate     Local day of month (1–31).
 * @param[out] tMonth    Local month (1–12).
 * @param[out] tYear     Local four-digit year (e.g. 2025).
 * @param[in]  timezone  UTC offset in whole hours, clamped to [−12, +12].
 *                       Fractional-hour zones are not supported.
 * @return @c GPSReturnStatus::OK on success,
 *         @c DATA_STALE if no fresh PVT packet is buffered,
 *         @c NOK_TIME_INVALID if the time or date validity flags are not set.
 */
GPSReturnStatus getGPSDateTime(SFE_UBLOX_GNSS &myGNSS,
                               uint8_t &tHour, uint8_t &tMinute, uint8_t &tSecond,
                               uint8_t &tDate, uint8_t &tMonth, uint16_t &tYear,
                               int8_t timezone = 0);

#endif
