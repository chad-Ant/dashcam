#include <TimeLib.h>   // tmElements_t, makeTime, CalendarYrToTm for applyTimezoneOffset()

#include "TimerFunctions.h"

TimerReturnStatus initializeRTC(RTCZero &rtc){
    rtc.begin();
    if (rtc.isConfigured()) return TimerReturnStatus::OK;
    return TimerReturnStatus::NOK_RTC_FAILED;
}

static void checkTimeValidity(uint8_t &tHour, uint8_t &tMinute, uint8_t &tSecond){
    tHour   = tHour   > 23 ? 23 : tHour;
    tMinute = tMinute > 59 ? 59 : tMinute;
    tSecond = tSecond > 59 ? 59 : tSecond;
}

static void checkTimezoneValidity(int8_t &timezone){
    timezone = timezone < -12 ? -12 : (timezone > 12 ? 12 : timezone);
}

static void checkDateValidity(uint8_t &tDate, uint8_t &tMonth, uint16_t &tYear){
    tDate  = tDate  < 1  ? 1  : (tDate  > 31   ? 31   : tDate);
    tMonth = tMonth < 1  ? 1  : (tMonth > 12   ? 12   : tMonth);
    tYear  = tYear  < 2000u ? 2000u : (tYear > 2099u ? 2099u : tYear);
}

void getRTCDate(RTCZero &rtc, uint8_t &tDate, uint8_t &tMonth, uint16_t &tYear){
    tDate  = rtc.getDay();
    tMonth = rtc.getMonth();
    tYear  = 2000u + rtc.getYear(); // RTCZero stores 0-99; return full 4-digit year
}

void getRTCTime(RTCZero &rtc, uint8_t &tHour, uint8_t &tMinute, uint8_t &tSecond){
    tHour   = rtc.getHours();
    tMinute = rtc.getMinutes();
    tSecond = rtc.getSeconds();
}

void getRTCDateTime(RTCZero &rtc,
                    uint8_t &tHour, uint8_t &tMinute, uint8_t &tSecond,
                    uint8_t &tDate, uint8_t &tMonth,  uint16_t &tYear){
    tHour   = rtc.getHours();
    tMinute = rtc.getMinutes();
    tSecond = rtc.getSeconds();
    tDate   = rtc.getDay();
    tMonth  = rtc.getMonth();
    tYear   = 2000u + rtc.getYear();
}

TimerReturnStatus setRTCTime(RTCZero &rtc, uint8_t tHour, uint8_t tMinute, uint8_t tSecond){
    if (!rtc.isConfigured()) return TimerReturnStatus::NOK_RTC_FAILED;
    checkTimeValidity(tHour, tMinute, tSecond);
    rtc.setTime(tHour, tMinute, tSecond);
    return TimerReturnStatus::OK;
}

TimerReturnStatus setRTCDate(RTCZero &rtc, uint8_t tDate, uint8_t tMonth, uint16_t tYear){
    if (!rtc.isConfigured()) return TimerReturnStatus::NOK_RTC_FAILED;
    checkDateValidity(tDate, tMonth, tYear);
    rtc.setDate(tDate, tMonth, (uint8_t)(tYear - 2000u));
    return TimerReturnStatus::OK;
}

TimerReturnStatus setRTCDateTime(RTCZero &rtc,
                                 uint8_t tHour, uint8_t tMinute, uint8_t tSecond,
                                 uint8_t tDate, uint8_t tMonth,  uint16_t tYear){
    if (!rtc.isConfigured()) return TimerReturnStatus::NOK_RTC_FAILED;
    checkTimeValidity(tHour, tMinute, tSecond);
    checkDateValidity(tDate, tMonth, tYear);
    rtc.setTime(tHour, tMinute, tSecond);
    rtc.setDate(tDate, tMonth, (uint8_t)(tYear - 2000u));
    return TimerReturnStatus::OK;
}

TimerReturnStatus applyTimezoneOffset(RTCZero &rtc, int8_t offset){
    if (!rtc.isConfigured()) return TimerReturnStatus::NOK_RTC_FAILED;
    checkTimezoneValidity(offset);

    tmElements_t te;
    te.Year   = CalendarYrToTm(2000u + rtc.getYear());
    te.Month  = rtc.getMonth();
    te.Day    = rtc.getDay();
    te.Hour   = rtc.getHours();
    te.Minute = rtc.getMinutes();
    te.Second = rtc.getSeconds();
    time_t t  = makeTime(te) + (long)offset * 3600L;

    // Guard before cast: year(t) < 2000 underflows uint8_t to 255 (invalid BCD).
    int adjustedYear = year(t);
    if (adjustedYear < 2000 || adjustedYear > 2099) return TimerReturnStatus::NOK_TIME_OUT_OF_RANGE;

    rtc.setTime((uint8_t)hour(t), (uint8_t)minute(t), (uint8_t)second(t));
    rtc.setDate((uint8_t)day(t), (uint8_t)month(t), (uint8_t)(adjustedYear - 2000));
    return TimerReturnStatus::OK;
}

TimerReturnStatus setAlarmTime(RTCZero &rtc, uint8_t tHour, uint8_t tMinute, uint8_t tSecond){
    if (!rtc.isConfigured()) return TimerReturnStatus::NOK_RTC_FAILED;
    checkTimeValidity(tHour, tMinute, tSecond);
    rtc.setAlarmTime(tHour, tMinute, tSecond);
    if (rtc.getAlarmSeconds() == tSecond && rtc.getAlarmMinutes() == tMinute && rtc.getAlarmHours() == tHour) return TimerReturnStatus::OK;
    return TimerReturnStatus::NOK_SET_ALARM_FAILED;
}

TimerReturnStatus armAlarm(RTCZero &rtc, RTCZero::Alarm_Match alarmType, voidFuncPtr callback, bool enable){
    if (!rtc.isConfigured()) return TimerReturnStatus::NOK_RTC_FAILED;

    if (enable){
        if (callback == nullptr) return TimerReturnStatus::NOK_VOID_CALLBACK;
        rtc.enableAlarm(alarmType);
        rtc.attachInterrupt(callback);
    } else {
        rtc.detachInterrupt();
        rtc.disableAlarm();
    }
    return TimerReturnStatus::OK;
}

