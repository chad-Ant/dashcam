// bridge_overlay_test — validates src/bridge_overlay.h: how one ESP32-C3 bridge
// telemetry sample becomes the recording overlay's vehicle fields.
//
// No hardware. Each case builds a hostproto::Telemetry the way the MKR sends it
// and checks the overlay the recorder would draw: which sources win, and that
// a field whose source is not live is dashed — never inherited, never a
// placeholder. v0.4 recorded dashes for every vehicle field until this was
// wired in (drive of 2026-09-26: SPD/ACC/HDG/LAT/LON/ALT all "--" with a live
// GNSS fix and CAN speed on the bus).
//
// Usage: bridge_overlay_test

#include "../bridge_overlay.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

using dashcam::app::applyBridgeTelemetry;
using dashcam::record::OverlayData;
using hostproto::Telemetry;

static int g_fails = 0;
static void check(bool ok, const std::string& what) {
    std::printf("  %s  %s\n", ok ? "ok  " : "FAIL", what.c_str());
    if (!ok) ++g_fails;
}

/// Everything absent, as the master sends it before any source is up.
static Telemetry blank() {
    Telemetry t;
    std::memset(&t, 0, sizeof(t));
    t.speed = t.accel = NAN;
    t.latitude = t.longitude = t.altitude = t.gpsSpeedKmh = t.heading = NAN;
    return t;
}

/// A healthy drive: CAN-sniffed speed and acceleration, 3-D fix, moving.
static Telemetry driving() {
    Telemetry t = blank();
    t.speed = 52.34f;  t.accel = 0.8f;
    t.sigSource = static_cast<uint8_t>(hostproto::VehSigSource::CAN_SNIFF);   // speed bits
    t.fixValid = 1;    t.flags = hostproto::TLM_FLAG_GPS_FIX;
    t.latitude = 10.8231f;  t.longitude = 106.6297f;  t.altitude = 12.5f;
    t.gpsSpeedKmh = 51.9f;  t.heading = 271.0f;
    return t;
}

int main() {
    const int64_t tick = 1'790'000'000'000LL;

    std::printf("--- healthy drive ---\n");
    {
        OverlayData od;
        applyBridgeTelemetry(od, driving(), true, tick);
        check(od.speedValid && std::fabs(od.speedKmh - 52.34f) < 1e-4f, "CAN-sniffed speed wins over GNSS");
        check(od.accelValid && std::fabs(od.accelerationMs2 - 0.8f) < 1e-6f, "acceleration from the master");
        check(od.positionValid && std::fabs(od.latitude - 10.8231) < 1e-4 &&
              std::fabs(od.longitude - 106.6297) < 1e-4 && std::fabs(od.altitudeM - 12.5) < 1e-4, "position from the fix");
        check(od.headingValid && std::fabs(od.headingDeg - 271.0f) < 1e-4f, "heading from the fix");
        check(od.speedTimestampMs == tick && od.accelTimestampMs == tick &&
              od.positionTimestampMs == tick && od.headingTimestampMs == tick, "every refreshed field stamped");
    }

    std::printf("--- a stale bridge dashes everything ---\n");
    {
        OverlayData od;
        applyBridgeTelemetry(od, driving(), true, tick);
        applyBridgeTelemetry(od, driving(), false, tick + 2000);
        check(!od.speedValid && !od.accelValid && !od.positionValid && !od.headingValid,
              "bridge silent past the freshness window: no field survives");
        check(od.speedTimestampMs == tick, "no stamp advanced by a stale sample");
    }

    std::printf("--- sources fail independently ---\n");
    {
        Telemetry t = driving();
        t.sigSource = 0;                                   // CAN speed gone, fix still good
        OverlayData od;
        applyBridgeTelemetry(od, t, true, tick);
        check(od.speedValid && std::fabs(od.speedKmh - 51.9f) < 1e-4f, "no CAN speed: GNSS ground speed");
        check(!od.accelValid, "no vehicle-speed source: acceleration dashed (never from GNSS)");

        t = driving();
        t.fixValid = 0;                                    // fix lost (tunnel), CAN fine
        applyBridgeTelemetry(od, t, true, tick);
        check(od.speedValid && std::fabs(od.speedKmh - 52.34f) < 1e-4f, "no fix: CAN speed still live");
        check(!od.positionValid && !od.headingValid, "no fix: position and heading dashed, not inherited");
        check(od.latitude == 0.0 && od.longitude == 0.0, "no fix: coordinates cleared");

        t = blank();                                       // OBD-II poller only
        t.speed = 40.0f;  t.accel = -1.2f;
        t.flags = hostproto::TLM_FLAG_OBD2_VALID;
        t.sigSource = static_cast<uint8_t>(hostproto::VehSigSource::OBD2);
        applyBridgeTelemetry(od, t, true, tick);
        check(od.speedValid && od.speedKmh == 40.0f && od.accelValid && od.accelerationMs2 == -1.2f,
              "OBD-II speed and acceleration when that is the only source");

        t.flags = 0;                                       // poller's own verdict: stale
        applyBridgeTelemetry(od, t, true, tick);
        check(!od.speedValid && !od.accelValid, "OBD2_VALID clear: OBD-II values dashed");
    }

    std::printf("--- NaN means not supplied ---\n");
    {
        Telemetry t = driving();
        t.heading = NAN;                                   // stopped: course untrustworthy
        OverlayData od;
        applyBridgeTelemetry(od, t, true, tick);
        check(od.positionValid && !od.headingValid, "stopped: fix live, heading dashed on its own");

        t = driving();
        t.altitude = NAN;                                  // 2-D fix
        applyBridgeTelemetry(od, t, true, tick);
        check(od.positionValid && std::isnan(od.altitudeM),
              "2-D fix: altitude carried as NaN (drawn as a dash), not the placeholder");

        t = driving();
        t.accel = NAN;                                     // estimator warming up
        applyBridgeTelemetry(od, t, true, tick);
        check(od.speedValid && !od.accelValid, "acceleration NaN: dashed, speed kept");

        t = driving();
        t.speed = NAN;                                     // CAN source flagged but no value yet
        applyBridgeTelemetry(od, t, true, tick);
        check(od.speedValid && std::fabs(od.speedKmh - 51.9f) < 1e-4f, "CAN speed NaN: falls back to GNSS");
    }

    std::printf("--- nothing inherited from the placeholders ---\n");
    {
        OverlayData od;                                    // 10.7725 N / 52.3 m / 90 deg defaults
        applyBridgeTelemetry(od, blank(), true, tick);
        check(!od.speedValid && !od.accelValid && !od.positionValid && !od.headingValid,
              "all sources absent: every field dashed");
        check(od.latitude == 0.0 && od.altitudeM == 0.0 && od.headingDeg == 0.0f,
              "placeholder coordinates, altitude and heading cleared");
    }

    std::printf("\nRESULT: %s (%d failure%s)\n", g_fails ? "FAIL" : "PASS", g_fails,
                g_fails == 1 ? "" : "s");
    return g_fails ? 1 : 0;
}
