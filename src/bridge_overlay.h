#pragma once
/**
 * @file bridge_overlay.h
 * @brief Maps one ESP32-C3 bridge telemetry sample onto the recording overlay.
 *
 * The vehicle half of the overlay — speed, acceleration, position, heading —
 * as dashcam v0.3 computed it inline, lifted out so v0.4 uses the same rules
 * and a test can pin them (src/tests/test_bridge_overlay.cpp).
 *
 * THE RULE: every field is gated on ITS OWN source, never on the bridge being
 * alive. A live bridge says the C3 and the MKR are talking; it says nothing
 * about whether the ECU answered or the receiver has a fix, and those fail
 * independently of the link and of each other. A field whose source is not
 * live is cleared and flagged invalid, so the recorder draws a dash — never an
 * inherited or placeholder value. A frozen speed or a minutes-old position
 * burned into a recording and presented as live is false evidence, which is
 * the one output a dashcam must never produce.
 */

#include "HostProtocol.h"
#include "librecord.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

namespace dashcam {
namespace app {

/**
 * @brief Fills @p od's vehicle fields from telemetry sample @p t.
 *
 * @param od          Overlay to update. Only the vehicle fields and their
 *                    validity flags and timestamps are written; ADAS fields
 *                    and timestampMs are left to the caller.
 * @param t           The bridge's newest sample.
 * @param bridgeFresh The sample is recent enough to describe now. When false,
 *                    every vehicle field is invalidated whatever @p t says.
 * @param tickMs      Epoch ms stamped on each field this call refreshes.
 */
inline void applyBridgeTelemetry(dashcam::record::OverlayData& od,
                                 const hostproto::Telemetry& t,
                                 bool bridgeFresh, int64_t tickMs)
{
    // OBD2_VALID is the master's own freshness verdict on the OBD-II poller.
    // It says nothing about sniffed CAN, which has its own source bits.
    const bool obdLive      = bridgeFresh && (t.flags & hostproto::TLM_FLAG_OBD2_VALID) != 0;
    const bool fixLive      = bridgeFresh && t.fixValid != 0;
    const bool canSpeedLive = bridgeFresh &&
        hostproto::sigSourceSpeed(t.sigSource) == hostproto::VehSigSource::CAN_SNIFF;

    // Speed: CAN-sniffed first (the transmission's own figure, 0.01 km/h at
    // 50-100 Hz, and it survives tunnels), then GNSS ground speed, then OBD-II
    // (whole km/h, ~2 Hz).
    if (canSpeedLive && !std::isnan(t.speed)) {
        od.speedKmh = t.speed;         od.speedValid = true; od.speedTimestampMs = tickMs;
    } else if (fixLive && !std::isnan(t.gpsSpeedKmh)) {
        od.speedKmh = t.gpsSpeedKmh;   od.speedValid = true; od.speedTimestampMs = tickMs;
    } else if (obdLive && !std::isnan(t.speed)) {
        od.speedKmh = t.speed;         od.speedValid = true; od.speedTimestampMs = tickMs;
    } else {
        od.speedKmh = 0.0f;            od.speedValid = false;
    }

    // Acceleration is estimated on the master from vehicle speed (smoothed,
    // differentiated, jerk-limited); NaN until its estimator has warmed up. It
    // follows the vehicle-speed sources only — never GNSS.
    if ((canSpeedLive || obdLive) && !std::isnan(t.accel)) {
        od.accelerationMs2 = t.accel;  od.accelValid = true; od.accelTimestampMs = tickMs;
    } else {
        od.accelerationMs2 = 0.0f;     od.accelValid = false;
    }

    // Position: the fix and both coordinates. Altitude is NaN without a 3-D
    // solution; it is carried as NaN (drawn as a dash), never inherited — the
    // previous value, or OverlayData's 52.3 m placeholder, would render as live.
    if (fixLive && !std::isnan(t.latitude) && !std::isnan(t.longitude)) {
        od.latitude  = t.latitude;
        od.longitude = t.longitude;
        od.altitudeM = t.altitude;     // may be NaN
        od.positionValid = true;       od.positionTimestampMs = tickMs;
    } else {
        od.latitude = 0.0; od.longitude = 0.0; od.altitudeM = 0.0;
        od.positionValid = false;
    }

    // Heading apart from the coordinates: the master sends NaN whenever the
    // course is untrustworthy (below ~5 km/h it wanders the whole circle),
    // which happens at every stop with the fix itself perfectly good.
    if (fixLive && !std::isnan(t.heading)) {
        od.headingDeg = t.heading;     od.headingValid = true; od.headingTimestampMs = tickMs;
    } else {
        od.headingDeg = 0.0f;          od.headingValid = false;
    }
}

// ─── detail block: every telemetry field ─────────────────────────────────────

namespace detail_fmt {

/// printf into a std::string (each detail line is short).
template <typename... A>
inline std::string fmt(const char* f, A... a) {
    char b[160];
    std::snprintf(b, sizeof(b), f, a...);
    return b;
}
/// A float field, or "--" when NaN / not finite (absent, stale, structurally unavailable).
inline std::string num(float v, const char* f) {
    return std::isfinite(v) ? fmt(f, static_cast<double>(v)) : std::string("--");
}
/// Which source supplied a signal: CAN (sniffed), OBD (polled), "" (none).
inline const char* src(hostproto::VehSigSource s) {
    return s == hostproto::VehSigSource::CAN_SNIFF ? "CAN"
         : s == hostproto::VehSigSource::OBD2      ? "OBD" : "--";
}
/// One raw wheel count (0.01 km/h) in km/h, or "--" for the 0xFFFF sentinel.
inline std::string wheel(uint16_t raw) {
    return raw == 0xFFFFu ? std::string("--") : fmt("%.2f", raw * 0.01);
}

} // namespace detail_fmt

/**
 * @brief Every field of telemetry sample @p t as overlay text, '\n' between lines.
 *
 * The detail block of the ASS sidecar. Same rule as applyBridgeTelemetry():
 * a field whose source is not live is a dash, and a cleared bit is printed only
 * when its validity bit says it was measured. With @p bridgeFresh false the
 * whole sample is old, so nothing in it is printed.
 */
inline std::string formatBridgeDetail(const hostproto::Telemetry& t, bool bridgeFresh)
{
    using namespace detail_fmt;
    using namespace hostproto;
    if (!bridgeFresh) return "TLM -- no live bridge sample";

    const uint16_t fl = t.flags;
    const uint8_t  vf = t.vehFlags;
    std::string out;
    auto line = [&out](const std::string& l) { if (!out.empty()) out += '\n'; out += l; };

    // Powertrain.
    static const char* kGear[7] = { "--", "P", "R", "N", "D", "L", "S" };
    const std::string pedal = (vf & VEH_FLAG_PEDAL_VALID) ? fmt("%.1f%%", t.pedalGas * 0.5) : std::string("--");
    line("VSS " + num(t.speed, "%.2f") + " " + src(sigSourceSpeed(t.sigSource)) +
         "  RPM " + num(t.rpm, "%.0f") + " " + src(sigSourceRpm(t.sigSource)) +
         "  GEAR " + (t.gearPos < 7 ? kGear[t.gearPos] : "?") + " " + src(sigSourceGear(t.sigSource)) +
         "  PEDAL " + pedal + "  ACC " + num(t.accel, "%+.2f") + " m/s2");

    // Brake, indicators, steering effort, yaw.
    std::string brake = "--";
    if (vf & VEH_FLAG_BRAKE_VALID)
        brake = std::string((vf & VEH_FLAG_BRAKE_PRESSED) ? "ON" : "off") +
                " sw " + ((vf & VEH_FLAG_BRAKE_SWITCH) ? "ON" : "off");
    std::string turn = "--", haz = "--";
    if (vf & VEH_FLAG_TURN_VALID) {
        const bool l = vf & VEH_FLAG_TURN_LEFT, r = vf & VEH_FLAG_TURN_RIGHT;
        turn = l && r ? "<->" : l ? "<-" : r ? "->" : "off";
        haz  = (vf & VEH_FLAG_HAZARD) ? "ON" : "off";
    }
    const std::string steer = t.steerMotorTorque == 0xFFFFu ? std::string("--") : fmt("%u", t.steerMotorTorque);
    const std::string yaw   = t.yawRateCdps == INT16_MIN ? std::string("--") : fmt("%+.2f", t.yawRateCdps / 100.0);
    line("BRAKE " + brake + "  TURN " + turn + "  HAZ " + haz + "  STEER TQ " + steer + " " +
         src(sigSourceSteer(t.sigSource)) + "  YAW " + yaw + " deg/s");

    line("WHEEL FL " + wheel(t.wheelRaw[0]) + " FR " + wheel(t.wheelRaw[1]) +
         " RL " + wheel(t.wheelRaw[2]) + " RR " + wheel(t.wheelRaw[3]) + " km/h");

    // GNSS: an absent receiver and a healthy one without a fix look alike in the
    // coordinates; GPS_PRESENT tells them apart.
    std::string gnss;
    if (!(fl & TLM_FLAG_GPS_PRESENT) && !t.fixValid) gnss = "GNSS absent";
    else gnss = std::string("GNSS ") + (t.fixValid ? (t.fixType == 3 ? "3D" : t.fixType == 2 ? "2D" : "fix")
                                                   : "no fix") +
                fmt(" %u sats", t.satellites) + "  GSPD " +
                (t.fixValid ? num(t.gpsSpeedKmh, "%.1f") : std::string("--")) + " km/h";
    gnss += "  UTC " + ((fl & TLM_FLAG_TIME_VALID)
                ? fmt("%04u-%02u-%02u %02u:%02u:%02u", t.year, t.month, t.day, t.hour, t.minute, t.second)
                : std::string("--"));
    line(gnss);

    // IMU: NaN with IMU_PRESENT clear = no IMU; NaN with it set = that channel stale.
    if (!(fl & TLM_FLAG_IMU_PRESENT)) {
        line("IMU absent  HG " + fmt("%u", t.imuHighGCount));
    } else {
        std::string a = "IMU a " + num(t.imuAccelX, "%+.2f") + " " + num(t.imuAccelY, "%+.2f") + " " +
                        num(t.imuAccelZ, "%+.2f") + " m/s2  w " + num(t.imuGyroX, "%+.1f") + " " +
                        num(t.imuGyroY, "%+.1f") + " " + num(t.imuGyroZ, "%+.1f") + " deg/s  T " +
                        num(t.imuTempC, "%.0f") + " C";
        if (std::isfinite(t.imuMagX) || std::isfinite(t.imuMagY) || std::isfinite(t.imuMagZ))
            a += "  MAG " + num(t.imuMagX, "%.0f") + " " + num(t.imuMagY, "%.0f") + " " + num(t.imuMagZ, "%.0f") + " uT";
        line(a);
        line("LIN " + num(t.imuLinAccelX, "%+.2f") + " " + num(t.imuLinAccelY, "%+.2f") + " " +
             num(t.imuLinAccelZ, "%+.2f") + "  PEAK |a| " + num(t.imuAccelPeak, "%.2f") + " |lin| " +
             num(t.imuLinAccelPeak, "%.2f") + " |w| " + num(t.imuGyroPeak, "%.1f") + "  YAWREL " +
             num(t.imuYawRelDeg, "%.1f"));
        std::string st = "HG " + fmt("%u", t.imuHighGCount) + " age " +
                         (t.imuHighGMs == 0xFFFFu ? std::string("--") : fmt("%.1f s", t.imuHighGMs / 1000.0)) +
                         fmt("  CAL s%u g%u a%u m%u", (t.imuCalib >> 6) & 3u, (t.imuCalib >> 4) & 3u,
                             (t.imuCalib >> 2) & 3u, t.imuCalib & 3u) +
                         "  MODE " + ((fl & TLM_FLAG_IMU_FUSION_MODE) ? "fusion" : "raw");
        if (fl & TLM_FLAG_IMU_HIGH_G)       st += "  HIGH-G";
        if (fl & TLM_FLAG_IMU_HIGHG_ARMED)  st += "  armed";
        if (fl & TLM_FLAG_IMU_DATA_GAP)     st += "  GAP";
        if (fl & TLM_FLAG_IMU_SATURATED)    st += "  SATURATED";
        if (fl & TLM_FLAG_IMU_DEGRADED)     st += "  degraded";
        if (fl & TLM_FLAG_IMU_LOWPOWER)     st += "  lowpower";
        line(st);
    }

    // OBD-II PIDs: NaN while sniffing (the polled-only fields), so mostly dashes then.
    line("OBD " + std::string((fl & TLM_FLAG_OBD2_VALID) ? "live" : "--") +
         "  COOL " + num(t.coolantTemp, "%.0f") + " C  FUEL " + num(t.fuelLevel, "%.0f") + "%  RATE " +
         num(t.fuelRate, "%.1f") + " L/h  THR " + num(t.throttle, "%.0f") + "%  LOAD " +
         num(t.engineLoad, "%.0f") + "%  BARO " + num(t.airPressure, "%.0f") + " kPa  G " +
         num(t.gear, "%.0f") + " RATIO " + num(t.gearRatio, "%.2f") + "  ODO " + num(t.odo, "%.1f") + " km");

    // Bus, map, switches, master.
    static const char* kMode[4] = { "off", "discover", "sniff", "obd2" };
    std::string map = "--";
    if (fl & TLM_FLAG_CANMAP_LOADED)
        map = fmt("0x%02X", t.canMapChecksum) + ((t.canMapFlags & 1u) ? ((t.canMapFlags & 2u) ? " filt map" : " filt host")
                                                                      : " accept-all");
    line(std::string("CAN ") + (t.canMode < 4 ? kMode[t.canMode] : "?") + "  MAP " + map + "  SW " +
         ((fl & TLM_FLAG_SWITCHES_PRESENT) ? fmt("%04X chg %04X", t.switchState, t.switchChanged) : std::string("--")) +
         fmt("  MKR %.1f s  FLAGS %04X", t.masterMillis / 1000.0, fl));
    return out;
}

/**
 * @brief Composes @p od's detail block from @p t (see formatBridgeDetail()).
 * @param tickMs Epoch ms stamped as OverlayData::detailTimestampMs.
 */
inline void applyBridgeDetail(dashcam::record::OverlayData& od, const hostproto::Telemetry& t,
                              bool bridgeFresh, int64_t tickMs)
{
    od.detailText        = formatBridgeDetail(t, bridgeFresh);
    od.detailTimestampMs = tickMs;
}

} // namespace app
} // namespace dashcam
