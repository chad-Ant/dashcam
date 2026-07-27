#ifndef TIMER_FUNCTIONS_H
#define TIMER_FUNCTIONS_H 1

#include <Arduino.h>
#include <RTCZero.h>
#include <climits>
#include "DataDictionary.h"

#define MIN_INTERVAL_MS  1UL         ///< Minimum meaningful polling interval (ms).
#define MAX_INTERVAL_MS  ULONG_MAX   ///< Maximum interval / rollover sentinel.

/** Return codes used by timer and RTC functions. */
enum class TimerReturnStatus{
    OK = 0,                    ///< Operation succeeded.
    NOK_RTC_FAILED = -1,       ///< RTCZero did not initialise or is not configured.
    NOK_SET_ALARM_FAILED = -2, ///< Alarm registers did not read back the written values.
    NOK_VOID_CALLBACK = -3,    ///< NULL callback passed to @c armAlarm() while enabling.
    NOK_TIME_OUT_OF_RANGE = -4,///< Adjusted time falls outside the RTC-supported range [2000, 2099].
};

/**
 * @brief Returns the elapsed time between two @c millis() snapshots.
 *
 * Unsigned arithmetic handles the 32-bit @c millis() rollover (~49.7 days)
 * automatically: subtracting an earlier timestamp from a later one always
 * gives the correct positive interval even across the wrap boundary.
 *
 * @param[in] lastTime    Timestamp captured at the start of the interval.
 * @param[in] currentTime Current timestamp (typically @c millis()).
 * @return Elapsed time in milliseconds.
 */
inline unsigned long getInterval(unsigned long lastTime, unsigned long currentTime){
    return currentTime - lastTime; // unsigned subtraction handles millis() rollover correctly
}

/**
 * @brief Returns @c true if at least @p timer milliseconds have elapsed since @p startTime.
 *
 * Usage pattern:
 * @code
 *   unsigned long t = millis();
 *   // ... do work ...
 *   if (isTimeout(500, t)) { // 500 ms have passed
 *       t = millis();        // reset for next cycle
 *   }
 * @endcode
 *
 * @param[in] timer     Desired interval in milliseconds.
 * @param[in] startTime Timestamp captured at the start of the interval.
 * @return @c true if the interval has elapsed, @c false otherwise.
 */
inline bool isTimeout(unsigned long timer, unsigned long startTime){
    return getInterval(startTime, millis()) >= timer;
}

/**
 * @brief Resets @p startTime to the current @c millis() value.
 *
 * Call this after @c isTimeout() returns @c true to restart the interval.
 *
 * @param[out] startTime  Reference timestamp to update.
 */
inline void resetTask(unsigned long &startTime){
    startTime = millis();
}

/**
 * @brief Initialises the RTCZero RTC peripheral.
 *
 * @param[in,out] rtc  RTCZero instance to initialise.
 * @return @c TimerReturnStatus::OK if the RTC reports configured,
 *         @c NOK_RTC_FAILED otherwise.
 */
TimerReturnStatus initializeRTC(RTCZero &rtc);

/**
 * @brief Reads the current date from the RTCZero peripheral.
 *
 * @param[in]  rtc     Initialised RTCZero instance.
 * @param[out] tDate   Day of month (1–31).
 * @param[out] tMonth  Month (1–12).
 * @param[out] tYear   Four-digit year (e.g. 2025).
 */
void getRTCDate(RTCZero &rtc, uint8_t &tDate, uint8_t &tMonth, uint16_t &tYear);

/**
 * @brief Reads the current time from the RTCZero peripheral.
 *
 * @param[in]  rtc      Initialised RTCZero instance.
 * @param[out] tHour    Hours (0–23).
 * @param[out] tMinute  Minutes (0–59).
 * @param[out] tSecond  Seconds (0–59).
 */
void getRTCTime(RTCZero &rtc, uint8_t &tHour, uint8_t &tMinute, uint8_t &tSecond);

/**
 * @brief Reads the current date and time from the RTCZero peripheral in one call.
 *
 * Reads all six registers in a single burst, avoiding the inconsistency that
 * would occur if date and time were fetched separately across midnight.
 *
 * @param[in]  rtc      Initialised RTCZero instance.
 * @param[out] tHour    Hours (0–23).
 * @param[out] tMinute  Minutes (0–59).
 * @param[out] tSecond  Seconds (0–59).
 * @param[out] tDate    Day of month (1–31).
 * @param[out] tMonth   Month (1–12).
 * @param[out] tYear    Four-digit year (e.g. 2025).
 */
void getRTCDateTime(RTCZero &rtc,
                    uint8_t &tHour, uint8_t &tMinute, uint8_t &tSecond,
                    uint8_t &tDate, uint8_t &tMonth,  uint16_t &tYear);

/**
 * @brief Sets the RTC time registers manually.
 *
 * Values are clamped to valid ranges before being written.
 *
 * @param[in,out] rtc     Initialised RTCZero instance.
 * @param[in]     tHour   Hours (0–23).
 * @param[in]     tMinute Minutes (0–59).
 * @param[in]     tSecond Seconds (0–59).
 * @return @c TimerReturnStatus::OK, or @c NOK_RTC_FAILED.
 */
TimerReturnStatus setRTCTime(RTCZero &rtc, uint8_t tHour, uint8_t tMinute, uint8_t tSecond);

/**
 * @brief Sets the RTC date registers manually.
 *
 * Values are clamped: day [1–31], month [1–12], year [2000–2099].
 *
 * @param[in,out] rtc    Initialised RTCZero instance.
 * @param[in]     tDate  Day of month (1–31).
 * @param[in]     tMonth Month (1–12).
 * @param[in]     tYear  Four-digit year (2000–2099).
 * @return @c TimerReturnStatus::OK, or @c NOK_RTC_FAILED.
 */
TimerReturnStatus setRTCDate(RTCZero &rtc, uint8_t tDate, uint8_t tMonth, uint16_t tYear);

/**
 * @brief Sets the RTC date and time registers in one call.
 *
 * Equivalent to calling @c setRTCTime() then @c setRTCDate(), but checks
 * @c isConfigured() only once and clamps all six values before writing.
 *
 * @param[in,out] rtc     Initialised RTCZero instance.
 * @param[in]     tHour   Hours (0–23).
 * @param[in]     tMinute Minutes (0–59).
 * @param[in]     tSecond Seconds (0–59).
 * @param[in]     tDate   Day of month (1–31).
 * @param[in]     tMonth  Month (1–12).
 * @param[in]     tYear   Four-digit year (2000–2099).
 * @return @c TimerReturnStatus::OK, or @c NOK_RTC_FAILED.
 */
TimerReturnStatus setRTCDateTime(RTCZero &rtc,
                                 uint8_t tHour, uint8_t tMinute, uint8_t tSecond,
                                 uint8_t tDate, uint8_t tMonth,  uint16_t tYear);

/**
 * @brief Shifts the RTC's current stored time by a whole-hour offset.
 *
 * Reads the current RTC value, adds @p offset hours, and writes the result
 * back — including any date rollover across midnight.  This is a one-shot
 * mutation: calling it twice with the same offset double-applies the shift.
 *
 * Prefer passing the timezone directly to @c getGPSDateTime() or
 * @c getNTPDateTime() so the RTC is always seeded with local time and this
 * function is never needed.
 *
 * @param[in,out] rtc     Initialised RTCZero instance.
 * @param[in]     offset  UTC offset in whole hours, clamped to [−12, +12].
 * @return @c TimerReturnStatus::OK, @c NOK_RTC_FAILED, or
 *         @c NOK_TIME_OUT_OF_RANGE if the adjusted time falls outside the
 *         RTC-supported year range [2000, 2099] (e.g. negative offset applied
 *         before the RTC is seeded with a valid GPS/NTP time).
 */
TimerReturnStatus applyTimezoneOffset(RTCZero &rtc, int8_t offset);

/**
 * @brief Programs the RTC alarm time registers.
 *
 * Sets the alarm to fire at the given hour, minute, and second.  Values are
 * clamped to valid ranges before being written.
 *
 * @param[in,out] rtc      Initialised RTCZero instance.
 * @param[in]     tHour    Alarm hour (0–23).
 * @param[in]     tMinute  Alarm minute (0–59).
 * @param[in]     tSecond  Alarm second (0–59).
 * @return @c TimerReturnStatus::OK if the registers read back correctly,
 *         @c NOK_RTC_FAILED if the RTC is not configured, or
 *         @c NOK_SET_ALARM_FAILED if verification fails.
 */
TimerReturnStatus setAlarmTime(RTCZero &rtc, uint8_t tHour, uint8_t tMinute, uint8_t tSecond);

/**
 * @brief Enables or disables the RTC alarm interrupt.
 *
 * When enabling, attaches @p callback to the RTC alarm interrupt and sets
 * the match type.  When disabling, detaches the interrupt and clears the
 * alarm enable flag.
 *
 * @param[in,out] rtc        Initialised RTCZero instance.
 * @param[in]     alarmType  Match type (e.g. @c RTCZero::MATCH_SS).
 * @param[in]     callback   ISR to call on alarm; must not be NULL when enabling.
 * @param[in]     enable     @c true to arm, @c false to disarm.
 * @return @c TimerReturnStatus::OK, @c NOK_RTC_FAILED, or
 *         @c NOK_VOID_CALLBACK.
 */
TimerReturnStatus armAlarm(RTCZero &rtc, RTCZero::Alarm_Match alarmType, voidFuncPtr callback, bool enable);

#endif
