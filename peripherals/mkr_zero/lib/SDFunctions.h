#ifndef SDFUNCTIONS_H
#define SDFUNCTIONS_H 1

#include <Arduino.h>
#include "DataDictionary.h"

/// Default INI configuration filename on the SD card root.
#define SD_CONFIG_FILENAME    "config.txt"
/// Default OBD-II log filename on the SD card root.
#define SD_OBD2_LOG_FILENAME  "obd2log.csv"

/** Return codes for all SD-card operations. */
enum class SDReturnStatus {
    OK               =  0, ///< Operation succeeded.
    NOK              = -1, ///< Unspecified failure.
    NOK_INIT_FAILED  = -2, ///< @c sd.begin() returned false; card absent or wiring fault.
    NOK_NOT_FOUND    = -3, ///< Target file does not exist on the card.
    NOK_WRITE_FAILED = -4, ///< File open, write, or sync operation failed.
    NOK_PARSE_ERROR  = -5, ///< INI file opened successfully but contained no valid key=value pairs.
};

/**
 * @brief Configuration values loaded from a Windows INI-style file on the SD card.
 *
 * All @c char[] fields are null-terminated C-strings.  Pre-populate the struct
 * with @c initSDConfigDefaults() before calling @c readConfig() so that keys
 * absent from the file retain their compile-time defaults.
 */
struct SDConfig {
    char    wifiSSID[33];         ///< Primary WiFi SSID (max 32 chars).
    char    wifiPass[64];         ///< Primary WiFi password (max 63 chars).
    char    wifiSSIDBackup1[33];  ///< Backup-1 SSID.
    char    wifiPassBackup1[64];  ///< Backup-1 password.
    char    wifiSSIDBackup2[33];  ///< Backup-2 SSID.
    char    wifiPassBackup2[64];  ///< Backup-2 password.
    float   defaultLat;           ///< Fallback GPS latitude (decimal degrees).
    float   defaultLon;           ///< Fallback GPS longitude (decimal degrees).
    float   defaultAlt;           ///< Fallback altitude above mean sea level (m).
    float   defaultPacc;          ///< Fallback position-accuracy estimate (m).
    int8_t  timezone;             ///< UTC offset in whole hours (−12 … +12).
    uint8_t servoXPin;            ///< Servo X-axis PWM pin number.
    uint8_t servoYPin;            ///< Servo Y-axis PWM pin number.
    uint8_t canCSPin;             ///< MCP2515 SPI chip-select pin number.
    uint8_t canIntPin;            ///< MCP2515 interrupt input pin number.
};

/**
 * @brief One timestamped OBD-II data row for the CSV log.
 *
 * Fields that the vehicle ECU did not supply should be set to @c NAN
 * before passing to @c writeOBD2LogEntry(); they are written as empty
 * CSV cells.
 */
struct OBD2LogEntry {
    uint8_t  hour;        ///< RTC hour (0–23).
    uint8_t  minute;      ///< RTC minute (0–59).
    uint8_t  second;      ///< RTC second (0–59).
    uint8_t  day;         ///< RTC day of month (1–31).
    uint8_t  month;       ///< RTC month (1–12).
    uint16_t year;        ///< RTC four-digit year.
    float    speed;       ///< Vehicle speed — @c SPEED PID (km/h).
    float    rpm;         ///< Engine speed — @c RPM PID.
    float    coolantTemp; ///< Coolant temperature — @c ENGINE_TEMP PID (°C).
    float    fuelLevel;   ///< Fuel tank level — @c FUEL_LVL PID (%).
    float    fuelRate;    ///< Engine fuel rate — @c FUEL_RATE PID (L/h).
    float    throttle;    ///< Throttle position — @c THROTTLE_POSN PID (%).
    float    engineLoad;  ///< Calculated engine load — @c ENGINE_LOAD PID (%).
    float    airPressure; ///< Barometric pressure — @c AIR_PRES PID (kPa).
    float    gearRatio;   ///< Transmission gear ratio — @c GEAR_RTIO PID.
    float    odo;         ///< Odometer reading — @c ODOMETER PID (km).
};

/**
 * @brief Initialises the SdFat32 filesystem and verifies card presence.
 *
 * Must be called once before any other SD function.  Uses the hardware SPI
 * bus; @c SPI.begin() is invoked internally if not already done.
 *
 * @param[in] csPin  Chip-select pin of the SD card module
 *                   (default @c SD_CS_PIN from DataDictionary.h).
 * @return @c SDReturnStatus::OK on success,
 *         @c NOK_INIT_FAILED if the card could not be reached.
 */
SDReturnStatus initializeSD(uint8_t csPin = SD_CS_PIN);

/**
 * @brief Pre-fills an @c SDConfig with the compile-time defaults from DataDictionary.h.
 *
 * Call this before @c readConfig() so that keys absent from the INI file
 * retain their compiled-in values rather than uninitialised memory.
 *
 * @param[out] config  Config struct to initialise.
 */
void initSDConfigDefaults(SDConfig &config);

/**
 * @brief Parses a Windows INI-style @c .txt file from the SD card into @p config.
 *
 * File format — lines beginning with @c ; or @c # are comments; Windows
 * (CRLF) and Unix (LF) line endings are both accepted:
 * @code
 * ; comment
 * [Section]
 * key = value
 * @endcode
 *
 * Supported sections and keys:
 *
 * | Section | Key              | SDConfig field      |
 * |---------|------------------|---------------------|
 * | WiFi    | ssid             | wifiSSID            |
 * | WiFi    | password         | wifiPass            |
 * | WiFi    | ssid_backup1     | wifiSSIDBackup1     |
 * | WiFi    | password_backup1 | wifiPassBackup1     |
 * | WiFi    | ssid_backup2     | wifiSSIDBackup2     |
 * | WiFi    | password_backup2 | wifiPassBackup2     |
 * | GPS     | lat              | defaultLat          |
 * | GPS     | lon              | defaultLon          |
 * | GPS     | alt              | defaultAlt          |
 * | GPS     | pacc             | defaultPacc         |
 * | GPS     | timezone         | timezone            |
 * | Pins    | servo_x          | servoXPin           |
 * | Pins    | servo_y          | servoYPin           |
 * | Pins    | can_cs           | canCSPin            |
 * | Pins    | can_int          | canIntPin           |
 *
 * Keys not present in the file leave the corresponding @p config field
 * unchanged.  Pre-populate with @c initSDConfigDefaults() to ensure sensible
 * defaults for missing keys.
 *
 * @param[in]  filename  Path on the SD card root (e.g. @c SD_CONFIG_FILENAME).
 * @param[out] config    Config struct to populate.
 * @return @c SDReturnStatus::OK on success,
 *         @c NOK_INIT_FAILED if @c initializeSD() has not been called,
 *         @c NOK_NOT_FOUND if @p filename does not exist,
 *         @c NOK_PARSE_ERROR if the file contained no valid key=value pairs.
 */
SDReturnStatus readConfig(const char *filename, SDConfig &config);

/**
 * @brief Opens (or creates) the OBD-II CSV log and writes the column header.
 *
 * If the file already exists and is non-empty the header row is not repeated.
 * The file stays open until @c closeOBD2Log() is called; subsequent
 * @c writeOBD2LogEntry() calls append without re-opening.
 *
 * CSV columns:
 * @c date,time,speed_kmh,rpm,coolant_c,fuel_pct,fuel_rate_lph,throttle_pct,load_pct,air_kpa,gear_ratio,odo_km
 *
 * @param[in] filename  Log filename on the SD card root
 *                      (default @c SD_OBD2_LOG_FILENAME).
 * @return @c SDReturnStatus::OK on success,
 *         @c NOK_INIT_FAILED if @c initializeSD() has not been called,
 *         @c NOK_WRITE_FAILED if the file could not be opened or the header
 *         could not be written.
 */
SDReturnStatus openOBD2Log(const char *filename = SD_OBD2_LOG_FILENAME);

/**
 * @brief Appends one timestamped OBD-II data row to the open log.
 *
 * @c NAN-valued fields are written as empty CSV cells.  @c sync() is called
 * after every row to minimise data loss on unexpected power loss.
 *
 * @param[in] entry  Populated @c OBD2LogEntry to record.
 * @return @c SDReturnStatus::OK on success,
 *         @c NOK_WRITE_FAILED if the log file is not open or the write fails.
 */
SDReturnStatus writeOBD2LogEntry(const OBD2LogEntry &entry);

/**
 * @brief Flushes and closes the OBD-II log file.
 *
 * Safe to call even if @c openOBD2Log() was never called or the file is
 * already closed.
 */
void closeOBD2Log();

#endif
