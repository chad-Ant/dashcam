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

} // namespace app
} // namespace dashcam
