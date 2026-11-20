#pragma once
#include "PageConfig.h"
#include "SettingsStore.h"

// Enable only with the external LDR divider described in main.cpp.
static constexpr bool kAutoBrightness = false;
// Turns the image and touch upside down for the rig's inverted mounting.
static constexpr bool kRotateDisplay180 = true;
// SET-tab values used until /settings.xml exists on the SD card (percent):
// master volume, screen brightness, microphone threshold. Master volume also
// scales the boot tone and USB alerts; kept low while on the bench.
static constexpr uint8_t kDefaultMasterVolume = 10;
static constexpr Es3Settings kDefaultSettings{kDefaultMasterVolume, 100, 50};
static constexpr Es3DashboardPage kDemoPages[] = {
    {"LIVE", "DASHCAM LIVE", "Record:  WAITING", "Speed:   -- km/h", "GPS:     no fix", "Event:   none", false, true, false},
    {"TRIP", "TRIP SUMMARY", "Distance: --.- km", "Drive:    00:00", "Average:  -- km/h", "Score:    -- / 100", false, false, false},
    {"CAM", "CAMERAS", "Road:    standby", "Cabin:   standby", "Lanes:   waiting", "Storage: -- GB free", false, false, false},
    {"SYS", "SYSTEM HEALTH", "Jetson:  offline", "C3 link: waiting", "CPU temp: -- C", "Battery: --.- V", false, false, false},
    // The settings page draws its own controls; its text lines are not shown.
    {"SET", "SETTINGS", "", "", "", "", true, false, true},
    {"MAP", "POSITION MAP", "Waiting for Jetson", "", "", "", false, false, false},
};
