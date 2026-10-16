#pragma once
#include "PageConfig.h"

// Enable only with the external LDR divider described in main.cpp.
static constexpr bool kAutoBrightness = false;
static constexpr Es3DashboardPage kDemoPages[] = {
    {"LIVE", "DASHCAM LIVE", "Record:  WAITING", "Speed:   -- km/h", "GPS:     no fix", "Event:   none", false, true, false},
    {"TRIP", "TRIP SUMMARY", "Distance: --.- km", "Drive:    00:00", "Average:  -- km/h", "Score:    -- / 100", false, false, false},
    {"CAM", "CAMERAS", "Road:    standby", "Cabin:   standby", "Lanes:   waiting", "Storage: -- GB free", false, false, false},
    {"SYS", "SYSTEM HEALTH", "Jetson:  offline", "C3 link: waiting", "CPU temp: -- C", "Battery: --.- V", false, false, false},
    {"SET", "QUICK CONTROLS", "Recording: start", "Audio alert: on", kAutoBrightness ? "Screen:      auto" : "Screen:      manual", "Tap SET to calibrate", true, false, true},
    {"MAP", "POSITION MAP", "Waiting for Jetson", "", "", "", false, false, false},
};
