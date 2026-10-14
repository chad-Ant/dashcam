#pragma once

#include <Arduino.h>
#include <Wire.h>

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "ES3C28P requires an ESP32-S3 target and Arduino ESP32 core 3.x"
#endif

// LCDWiki ES3C28P N16R8. See README.md for schematic and driver references.
namespace Es3Board {
constexpr int tftMiso = 13;
constexpr int tftMosi = 11;
constexpr int tftClock = 12;
constexpr int tftCs = 10;
constexpr int tftDc = 46;
constexpr int backlight = 45;
constexpr int sda = 16;
constexpr int scl = 15;
constexpr int touchReset = 18;
constexpr int touchIrq = 17;
constexpr uint8_t touchAddress = 0x38;
constexpr uint8_t codecAddress = 0x18;
constexpr int sdClock = 38;
constexpr int sdCommand = 40;
constexpr int sdD0 = 39;
constexpr int sdD1 = 41;
constexpr int sdD2 = 48;
constexpr int sdD3 = 47;
constexpr int amplifierDisable = 1;  // active LOW enable
constexpr int audioMclk = 4;
constexpr int audioBclk = 5;
constexpr int audioWs = 7;
constexpr int audioOut = 8;
constexpr int externalLdr = 2;

// Touch and codec share Wire. Call from the Arduino loop task only.
inline bool beginI2c()
{
    static bool started = false;
    if (!started) {
        started = Wire.begin(sda, scl, 400000);
        Wire.setTimeOut(20);
    }
    return started;
}
}  // namespace Es3Board
