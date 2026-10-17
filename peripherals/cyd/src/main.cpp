/**
 * CYD dashboard demo application.
 *
 * This is intentionally just the page data and application lifecycle. All
 * graphics, touch, calibration, SD storage, and tab handling live in
 * lib/cydGraphics so the production dashcam UI can replace these placeholders
 * without touching hardware code.
 */

#include <Arduino.h>
#include "cydAudio.h"
#include "cydNavigation.h"
#include "CydLog.h"
#include "cydGraphics.h"

static constexpr CydDashboardPage kDemoPages[] = {
    {"LIVE", "DASHCAM LIVE", "Record:  WAITING", "Speed:   -- km/h", "GPS:     no fix", "Event:   none", false, true, false},
    {"TRIP", "TRIP SUMMARY", "Distance: --.- km", "Drive:    00:00", "Average:  -- km/h", "Score:    -- / 100", false, false, false},
    {"CAM", "CAMERAS", "Road:    standby", "Cabin:   standby", "Lanes:   waiting", "Storage: -- GB free", false, false, false},
    {"SYS", "SYSTEM HEALTH", "Jetson:  offline", "C3 link: waiting", "CPU temp: -- C", "Battery: --.- V", false, false, false},
    {"SET", "QUICK CONTROLS", "Recording: start", "Audio alert: on", "Screen:      auto", "Tap SET to calibrate", true, false, true},
    {"MAP", "NAVIGATION", "", "", "", "", false, false, false},
};

// The ESP32-2432S028's onboard LDR is wired to ADC1 GPIO34. On this board,
// its raw value is lower in bright light, hence darkRaw is intentionally above
// brightRaw. It has two target backlight levels: daytime begins at L = 20;
// PWM then moves 4 steps every 50 ms, creating a roughly 2.8 second fade.
static constexpr CydBrightnessConfig kBrightness = {
    true, 34, 1200, 400, 32, 255, 20, 4, 50
};

static CydStorage gStorage;
static CydGraphics gGraphics(gStorage);
static CydAudio gAudio(gStorage);
static CydNavigation gNavigation(gGraphics, gAudio);

void setup()
{


    const uint8_t pageCount = static_cast<uint8_t>(sizeof(kDemoPages) / sizeof(kDemoPages[0]));
    const bool storageOk = gStorage.begin();
    cydLog("[cyd] storage %s\n", storageOk ? "ready" : "unavailable");
    if (!gGraphics.setPages(kDemoPages, pageCount) ||
        !gGraphics.setBrightnessConfig(kBrightness) || !gGraphics.begin()) {
        cydLog("[cyd] graphics startup failed");
    }
    if (!gAudio.begin()) {
        cydLog("[cyd] audio startup failed");
    } else if (gStorage.exists("/alert.wav")) {
        // Optional short boot proof. It remains silent when the test asset is
        // not copied to the card; future notifications call playWav directly.
        (void)gAudio.playWav("/alert.wav");
    }
    (void)gNavigation.begin();
}

void loop()
{
    gGraphics.update();
    gNavigation.update();
    gAudio.update();
    delay(1);
}
