// exposure_test — validates the frame-rate-priority auto-exposure loop
// (libcamera_exposure FrameRateExposure) against a simulated camera, no hardware.
//
// The simulated camera deliberately uses a DIFFERENT tone curve and gain curve
// than the controller's internal model (luma ∝ light^0.7, gain 15 = 2.8×), so
// the tests show convergence does not depend on the model being exact.
//
// Invariants checked on every step of every scenario:
//   - exposure never exceeds the frame-time cap (the whole point: fps held)
//   - exposure and gain stay inside the device limits
// Scenarios: daylight, dusk, night beyond reach, day→night→day, clipped
// highlights, dead band, NaN input, cap math.
//
// Usage: exposure_test

#include "libcamera_exposure.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

using dashcam::camera::ExposureSetting;
using dashcam::camera::FrameRateExposure;

static int g_fails = 0;
static void check(bool ok, const std::string& what) {
    std::printf("  %s  %s\n", ok ? "ok  " : "FAIL", what.c_str());
    if (!ok) ++g_fails;
}

// Simulated camera: scene light L (arbitrary units) → mean luma 0..255.
static float cameraLuma(double sceneLight, const ExposureSetting& s) {
    const double gainFactor = 1.0 + (2.8 - 1.0) * s.gain / 15.0;
    const double light = sceneLight * s.exposure * gainFactor;
    return static_cast<float>(std::min(255.0, std::pow(light, 0.7)));
}

struct RunResult {
    ExposureSetting last;
    float           luma = 0;
    bool            capHeld = true;     // exposure <= cap on every step
    bool            inLimits = true;
    int             settledAt = -1;     // first step inside ±10 % of target, -1 = never
};

static RunResult run(FrameRateExposure& fe, const FrameRateExposure::Limits& lim, double scene,
                     int steps, int target) {
    RunResult r;
    for (int i = 0; i < steps; ++i) {
        const float luma = cameraLuma(scene, fe.current());
        const ExposureSetting s = fe.update(luma);
        r.capHeld  = r.capHeld && s.exposure <= lim.exposureCap;
        r.inLimits = r.inLimits && s.exposure >= lim.exposureMin && s.gain >= lim.gainMin &&
                     s.gain <= lim.gainMax;
        if (r.settledAt < 0 && std::fabs(luma - target) <= 0.1 * target) r.settledAt = i;
    }
    r.last = fe.current();
    r.luma = cameraLuma(scene, r.last);
    return r;
}

int main() {
    FrameRateExposure::Limits lim;
    lim.exposureMin = 1;
    lim.exposureCap = FrameRateExposure::capForFps(30.0f, 1, 5000);
    lim.gainMin     = 0;
    lim.gainMax     = 15;
    const int target = 120;
    // Scene light giving target luma at the cap with no gain (the day/night edge).
    const double edge = std::pow(double(target), 1.0 / 0.7) / lim.exposureCap;

    std::printf("--- cap math ---\n");
    check(FrameRateExposure::capForFps(30.0f, 1, 5000) == 323, "30 fps -> 32.3 ms cap");
    check(FrameRateExposure::capForFps(60.0f, 1, 5000) == 156, "60 fps -> 15.6 ms cap");
    check(FrameRateExposure::capForFps(0.0f, 1, 5000) == 323, "unknown fps treated as 30");
    check(FrameRateExposure::capForFps(5.0f, 1, 1000) == 1000, "cap never above the device max");

    std::printf("--- daylight (20x brighter than the edge) ---\n");
    {
        FrameRateExposure fe(lim, target);
        const RunResult r = run(fe, lim, edge * 20, 20, target);
        check(r.settledAt >= 0 && r.settledAt <= 8, "settles near target within 8 samples (4 s at 2 Hz): " +
              std::to_string(r.settledAt));
        check(r.last.gain == 0, "no gain in daylight");
        check(r.last.exposure < lim.exposureCap / 4, "short exposure in daylight (" +
              std::to_string(r.last.exposure) + ")");
        check(r.capHeld && r.inLimits, "cap and limits held");
    }

    std::printf("--- dusk (half the edge light: needs gain, not longer exposure) ---\n");
    {
        FrameRateExposure fe(lim, target);
        const RunResult r = run(fe, lim, edge * 0.5, 30, target);
        check(std::fabs(r.luma - target) <= 0.1 * target + 1, "reaches target (luma " +
              std::to_string(int(r.luma)) + ")");
        check(r.last.exposure == lim.exposureCap && r.last.gain > 0, "exposure at cap + gain " +
              std::to_string(r.last.gain));
        check(fe.lowLight(), "reports low light");
        check(r.capHeld && r.inLimits, "cap and limits held");
    }

    std::printf("--- night (target out of reach) ---\n");
    {
        FrameRateExposure fe(lim, target);
        const RunResult r = run(fe, lim, edge * 0.05, 30, target);
        check(fe.atCeiling(), "ends at the ceiling (cap + max gain)");
        check(r.last.exposure == lim.exposureCap, "exposure never past the cap even when too dark");
        check(r.capHeld && r.inLimits, "cap and limits held on every step");
    }

    std::printf("--- day -> night -> day ---\n");
    {
        FrameRateExposure fe(lim, target);
        RunResult a = run(fe, lim, edge * 20, 20, target);
        RunResult b = run(fe, lim, edge * 0.3, 30, target);
        RunResult c = run(fe, lim, edge * 20, 30, target);
        check(a.last.gain == 0 && b.last.gain > 0, "gain added at night");
        check(c.last.gain == 0 && c.last.exposure < lim.exposureCap / 4,
              "back in daylight: gain removed first, exposure short again");
        check(std::fabs(c.luma - target) <= 0.1 * target + 1, "daylight luma back at target");
        check(a.capHeld && b.capHeld && c.capHeld, "cap held throughout");
    }

    std::printf("--- clipped, dead band, bad input ---\n");
    {
        FrameRateExposure fe(lim, target);
        const ExposureSetting before = fe.current();
        const ExposureSetting after  = fe.update(255.0f);
        check(after.exposure < before.exposure, "clipped frame steps exposure down");
        const ExposureSetting held = fe.update(float(target) * 1.05f);
        check(held == after, "inside the dead band: no change (no hunting)");
        check(fe.update(NAN) == after, "NaN luma ignored");
        check(fe.update(0.0f).exposure > after.exposure, "black frame steps exposure up");
    }

    std::printf("\nRESULT: %s (%d failure%s)\n", g_fails ? "FAIL" : "PASS", g_fails,
                g_fails == 1 ? "" : "s");
    return g_fails ? 1 : 0;
}
