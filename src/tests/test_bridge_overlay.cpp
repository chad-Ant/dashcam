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
#include "../telemetry_log.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using dashcam::app::applyBridgeTelemetry;
using dashcam::app::formatBridgeDetail;
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

    std::printf("--- detail block: every field, each gated on its own source ---\n");
    auto has = [](const std::string& s, const std::string& sub) { return s.find(sub) != std::string::npos; };
    {
        const std::string d = formatBridgeDetail(driving(), false);
        check(d == "TLM -- no live bridge sample", "stale bridge: nothing from the sample is printed");
    }
    {
        // Everything absent: the master's boot snapshot (sentinels as the master sends them).
        Telemetry t = blank();
        t.rpm = NAN; t.steerMotorTorque = 0xFFFF; t.yawRateCdps = INT16_MIN;
        for (int i = 0; i < 4; ++i) t.wheelRaw[i] = 0xFFFF;
        t.imuHighGMs = 0xFFFF;
        t.coolantTemp = t.fuelLevel = t.fuelRate = t.throttle = t.engineLoad = t.airPressure = NAN;
        t.gear = t.gearRatio = t.odo = NAN;
        const std::string d = formatBridgeDetail(t, true);
        check(has(d, "VSS -- --  RPM -- --  GEAR -- --  PEDAL --  ACC -- m/s2"), "absent powertrain: dashes, no source");
        check(has(d, "BRAKE --  TURN --  HAZ --  STEER TQ -- --  YAW -- deg/s"),
              "brake/turn bits without their validity bits: dashes, not 'off'");
        check(has(d, "WHEEL FL -- FR -- RL -- RR -- km/h"), "wheel sentinels dashed");
        check(has(d, "GNSS absent  UTC --"), "no receiver: GNSS absent");
        check(has(d, "IMU absent  HG 0"), "no IMU: IMU absent");
        check(has(d, "OBD --  COOL -- C") && has(d, "ODO -- km"), "OBD PIDs NaN: dashed");
        check(has(d, "CAN off  MAP --  SW --  MKR 0.0 s  FLAGS 0000"), "bus/map/switch status");
        check(!has(d, "nan") && !has(d, "65535") && !has(d, "-327"), "no sentinel or nan leaks into the text");
    }
    {
        Telemetry t = driving();
        t.rpm = 1234.0f; t.pedalGas = 25; t.gearPos = 4;
        t.sigSource = 0x01 | (0x01 << 2) | (0x01 << 4) | (0x01 << 6);
        t.vehFlags = hostproto::VEH_FLAG_BRAKE_VALID | hostproto::VEH_FLAG_BRAKE_SWITCH |
                     hostproto::VEH_FLAG_TURN_VALID | hostproto::VEH_FLAG_TURN_LEFT | hostproto::VEH_FLAG_PEDAL_VALID;
        t.steerMotorTorque = 87; t.yawRateCdps = -1234;
        t.wheelRaw[0] = 5231; t.wheelRaw[1] = 5240; t.wheelRaw[2] = 5220; t.wheelRaw[3] = 0xFFFF;
        t.satellites = 9; t.fixType = 3;
        t.flags |= hostproto::TLM_FLAG_GPS_PRESENT | hostproto::TLM_FLAG_TIME_VALID | hostproto::TLM_FLAG_IMU_PRESENT |
                   hostproto::TLM_FLAG_IMU_FUSION_MODE | hostproto::TLM_FLAG_CANMAP_LOADED | hostproto::TLM_FLAG_IMU_DATA_GAP;
        t.year = 2026; t.month = 10; t.day = 3; t.hour = 13; t.minute = 4; t.second = 5;
        t.imuAccelX = 0.12f; t.imuAccelY = -0.03f; t.imuAccelZ = 9.81f;
        t.imuGyroX = 0.1f; t.imuGyroY = -0.2f; t.imuGyroZ = 0.3f; t.imuTempC = 31.0f;
        t.imuMagX = t.imuMagY = t.imuMagZ = NAN;
        t.imuLinAccelX = 0.5f; t.imuLinAccelY = NAN; t.imuLinAccelZ = -0.1f;
        t.imuAccelPeak = 10.6f; t.imuLinAccelPeak = 1.1f; t.imuGyroPeak = 7.1f; t.imuYawRelDeg = 12.3f;
        t.imuHighGCount = 2; t.imuHighGMs = 1500; t.imuCalib = 0x34;   // g3 a1
        t.canMode = 2; t.canMapChecksum = 0x0B; t.canMapFlags = 0x03;
        t.rpm = 1234.0f; t.coolantTemp = t.fuelLevel = t.fuelRate = t.throttle = t.engineLoad = NAN;
        t.airPressure = t.gear = t.gearRatio = NAN; t.odo = 12345.6f;
        t.masterMillis = 61500;
        const std::string d = formatBridgeDetail(t, true);
        check(has(d, "VSS 52.34 CAN  RPM 1234 CAN  GEAR D CAN  PEDAL 12.5%  ACC +0.80 m/s2"), "powertrain line");
        check(has(d, "BRAKE off sw ON  TURN <-  HAZ off  STEER TQ 87 CAN  YAW -12.34 deg/s"),
              "brake (pedal off, switch on), left indicator, effort, yaw (left positive)");
        check(has(d, "WHEEL FL 52.31 FR 52.40 RL 52.20 RR -- km/h"), "wheels, one channel unavailable");
        check(has(d, "GNSS 3D 9 sats  GSPD 51.9 km/h  UTC 2026-10-03 13:04:05"), "GNSS line");
        check(has(d, "IMU a +0.12 -0.03 +9.81 m/s2  w +0.1 -0.2 +0.3 deg/s  T 31 C") && !has(d, "MAG"),
              "IMU raw line; no MAG in fusion mode");
        check(has(d, "LIN +0.50 -- -0.10  PEAK |a| 10.60 |lin| 1.10 |w| 7.1  YAWREL 12.3"), "one stale lin channel dashed");
        check(has(d, "HG 2 age 1.5 s  CAL s0 g3 a1 m0  MODE fusion  GAP"), "High-G count/age, calibration, flags");
        check(has(d, "ODO 12345.6 km"), "odometer while sniffing");
        check(has(d, "CAN sniff  MAP 0x0B filt map  SW --  MKR 61.5 s"), "map identity and filter origin");
        t.vehFlags |= hostproto::VEH_FLAG_TURN_RIGHT | hostproto::VEH_FLAG_HAZARD;
        check(has(formatBridgeDetail(t, true), "TURN <->  HAZ ON"), "both indicators and hazard");
        dashcam::record::OverlayData od;
        dashcam::app::applyBridgeDetail(od, t, true, tick);
        check(od.detailTimestampMs == tick && od.detailText == formatBridgeDetail(t, true), "applyBridgeDetail stamps the text");

        // CSV: same values, decoded bits empty when not measured.
        auto cells = [](const std::string& row) {
            std::vector<std::string> v; std::string c;
            for (char ch : row) { if (ch == ',') { v.push_back(c); c.clear(); } else c += ch; }
            v.push_back(c); return v;
        };
        const auto hdr = cells(dashcam::app::telemetryCsvHeader());
        const auto row = cells(dashcam::app::telemetryCsvRow(t, tick));
        check(hdr.size() == row.size(), "CSV row has one cell per header column (" + std::to_string(hdr.size()) + ")");
        auto col = [&](const std::string& name) {
            for (size_t i = 0; i < hdr.size() && i < row.size(); ++i) if (hdr[i] == name) return row[i];
            return std::string("<missing>");
        };
        check(col("host_ms") == std::to_string(tick) && col("master_ms") == "61500", "CSV: host and master time");
        check(col("speed_kmh") == "52.34" && col("rpm") == "1234" && col("gear_pos") == "4", "CSV: powertrain");
        check(col("coolant_c").empty() && col("imu_lin_y").empty(), "CSV: NaN is an empty cell");
        check(col("brake_pressed") == "0" && col("brake_switch") == "1" && col("turn_left") == "1" &&
              col("turn_right") == "1" && col("hazard") == "1", "CSV: decoded bits");
        check(col("pedal_pct") == "12.5" && col("steer_torque") == "87" && col("yaw_rate_dps") == "-12.34", "CSV: pedal/effort/yaw");
        check(col("wheel_fl_kmh") == "52.31" && col("wheel_rr_kmh").empty(), "CSV: wheels, sentinel empty");
        check(col("utc") == "2026-10-03T13:04:05Z" && col("highg_age_ms") == "1500", "CSV: UTC and High-G age");
        check(col("src_speed") == "1" && col("flags").rfind("0x", 0) == 0, "CSV: source and flags");
        Telemetry u = blank();
        u.steerMotorTorque = 0xFFFF; u.yawRateCdps = INT16_MIN; u.imuHighGMs = 0xFFFF;
        for (int i = 0; i < 4; ++i) u.wheelRaw[i] = 0xFFFF;
        const auto row2 = cells(dashcam::app::telemetryCsvRow(u, tick));
        auto col2 = [&](const std::string& name) {
            for (size_t i = 0; i < hdr.size() && i < row2.size(); ++i) if (hdr[i] == name) return row2[i];
            return std::string("<missing>");
        };
        check(col2("brake_pressed").empty() && col2("turn_left").empty() && col2("pedal_pct").empty() &&
              col2("steer_torque").empty() && col2("yaw_rate_dps").empty() && col2("highg_age_ms").empty() &&
              col2("utc").empty() && col2("switch_state").empty(), "CSV: unmeasured and sentinel fields are empty");
        hostproto::BridgeStatus bs{};
        bs.telemetryAgeMs = UINT32_MAX; bs.batteryVolts = NAN; bs.batteryPercent = NAN; bs.tempC = 41.5f;
        check(cells(dashcam::app::statusCsvRow(bs, tick)).size() == cells(dashcam::app::statusCsvHeader()).size(),
              "status CSV row has one cell per header column");
    }

    std::printf("\nRESULT: %s (%d failure%s)\n", g_fails ? "FAIL" : "PASS", g_fails,
                g_fails == 1 ? "" : "s");
    return g_fails ? 1 : 0;
}
