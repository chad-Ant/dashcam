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
// highlights, dead band, NaN input, cap math; and the night-mode switch
// (hand-over to the camera's own auto-exposure and back, dark-picture exit
// guard, fast probe after a take-back or at session start, back-off).
//
// Usage: exposure_test

#include "libcamera_exposure.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

using dashcam::camera::ExposureSetting;
using dashcam::camera::FrameRateExposure;
using dashcam::camera::NightModeSwitch;
using Mode = NightModeSwitch::Mode;

// Drive a NightModeSwitch at 2 Hz for @p seconds with a scene function
// scene(t) -> {atCeiling, luma, fps}; returns the time (s since start of this
// call) of the first mode change, or -1.
struct Obs { bool ceiling; float luma; float fps; };
template <typename Scene>
static double drive(NightModeSwitch& ns, double& t, double seconds, Scene scene) {
    const Mode start = ns.mode();
    const double t0 = t;
    double changedAt = -1;
    for (int i = 0; i < int(seconds * 2); ++i) {
        t += 0.5;
        const Obs o = scene(t - t0);
        if (ns.update(t, o.ceiling, o.luma, o.fps, 30.0f) != start && changedAt < 0) changedAt = t - t0;
    }
    return changedAt;
}

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

    std::printf("--- night mode switch ---\n");
    {
        NightModeSwitch ns; double t = 0;
        check(drive(ns, t, 60, [](double) { return Obs{false, 20, 30}; }) < 0 && ns.mode() == Mode::FrameRate,
              "not at the ceiling (loop still has headroom): never hands over");
        check(drive(ns, t, 60, [](double) { return Obs{true, 60, 30}; }) < 0,
              "at the ceiling but usable (luma 60 > 35): stays at 30 fps");
    }
    {
        // Started in daylight (the session-start probe ends), then night falls.
        NightModeSwitch ns; double t = 0;
        drive(ns, t, 5, [](double) { return Obs{false, 120, 30}; });
        const double at = drive(ns, t, 30, [](double) { return Obs{true, 5, 30}; });
        check(at >= 10 && at <= 11, "night (ceiling, luma 5): camera takes over after ~10 s (" + std::to_string(at) + ")");
    }
    {
        // Night boot or recovery restart: no 10 s of near-black first.
        NightModeSwitch ns; double t = 0;
        const double at = drive(ns, t, 30, [](double) { return Obs{true, 5, 30}; });
        check(at >= 2 && at <= 3, "session starts in the dark: camera after ~2 s, not 10 (" + std::to_string(at) + ")");
    }
    {
        NightModeSwitch ns; double t = 0;
        drive(ns, t, 5, [](double) { return Obs{false, 120, 30}; });
        // Oncoming headlights: 1 s bright in every 4 s.
        const double at = drive(ns, t, 60, [](double x) {
            return Obs{true, std::fmod(x, 4.0) < 1.0 ? 90.f : 5.f, 30}; });
        check(at > 10 && at < 40, "headlight flashes slow the hand-over, don't prevent it (" + std::to_string(at) + " s)");
    }
    {
        NightModeSwitch ns; double t = 0;
        drive(ns, t, 15, [](double) { return Obs{true, 5, 30}; });
        check(ns.mode() == Mode::Camera, "setup: in night mode");
        check(drive(ns, t, 120, [](double) { return Obs{true, 60, 20}; }) < 0,
              "camera at 20 fps (still lengthening exposure): stays in night mode");
        check(drive(ns, t, 120, [](double) { return Obs{true, 60, NAN}; }) < 0,
              "frame rate unknown: stays in night mode");
        const double at = drive(ns, t, 60, [](double) { return Obs{false, 120, 30}; });
        check(at >= 30 && at <= 31, "camera back at 30 fps for 30 s: 30 fps priority again (" + std::to_string(at) + ")");
    }
    {
        // 60 fps mode: the camera's auto-exposure never slows down, so "full
        // rate" is always true; its picture (luma 18) is what says it is night.
        NightModeSwitch ns; double t = 0;
        drive(ns, t, 15, [](double) { return Obs{true, 4, 30}; });
        check(ns.mode() == Mode::Camera, "setup: in night mode");
        check(drive(ns, t, 1200, [](double) { return Obs{true, 18, 30}; }) < 0,
              "camera at full rate but darker than NightLuma: never takes back (no dark probes)");
        const double at = drive(ns, t, 60, [](double) { return Obs{true, 80, 30}; });
        check(at >= 30 && at <= 31, "full rate and bright enough: takes back after 30 s (" + std::to_string(at) + ")");
    }
    {
        // A take-back that finds it still dark hands back within seconds.
        NightModeSwitch ns; double t = 0;
        drive(ns, t, 15, [](double) { return Obs{true, 5, 30}; });
        drive(ns, t, 31, [](double) { return Obs{true, 60, 30}; });
        check(ns.mode() == Mode::FrameRate, "setup: taken back");
        // First sample after the take-back still shows the camera's bright frame.
        const double at = drive(ns, t, 15, [](double x) { return Obs{true, x < 0.6 ? 60.f : 5.f, 30}; });
        check(at > 0 && at <= 3, "still dark at the ceiling: camera again after " + std::to_string(at) +
              " s (not 10 s)");
        check(ns.exitHoldSec() == 60, "...and that counts as a failed take-back (wait doubles to 60 s)");
    }
    {
        // A good take-back, then a short unlit underpass 8 s later: the probe
        // already ended (usable light), so no quick hand-back and no doubling.
        NightModeSwitch ns; double t = 0;
        drive(ns, t, 15, [](double) { return Obs{true, 5, 30}; });
        drive(ns, t, 30.5, [](double) { return Obs{true, 60, 30}; });
        check(ns.mode() == Mode::FrameRate, "setup: taken back");
        const double at = drive(ns, t, 21, [](double x) {
            return x >= 8 && x < 11 ? Obs{true, 5, 30} : Obs{false, 120, 30}; });
        check(at < 0 && ns.exitHoldSec() == 30, "3 s underpass 8 s after a good take-back: no hand-over, wait still 30 s");
    }
    {
        // A take-back into murky light (loop below its ceiling, luma 30):
        // neither dark nor usable — the probe runs out after probeSec.
        NightModeSwitch ns; double t = 0;
        drive(ns, t, 15, [](double) { return Obs{true, 5, 30}; });
        drive(ns, t, 30.5, [](double) { return Obs{true, 60, 30}; });
        drive(ns, t, 25, [](double) { return Obs{false, 30, 30}; });
        const double at = drive(ns, t, 30, [](double) { return Obs{true, 5, 30}; });
        check(at >= 10 && at <= 11 && ns.exitHoldSec() == 30,
              "probe expires after 20 s: later darkness needs the normal ~10 s, no doubling (" + std::to_string(at) + ")");
    }
    {
        // Past the probe window the normal 10 s applies again (a tunnel after
        // a good take-back must not flip exposure after 2 s).
        NightModeSwitch ns; double t = 0;
        drive(ns, t, 15, [](double) { return Obs{true, 5, 30}; });
        drive(ns, t, 31, [](double) { return Obs{true, 60, 30}; });
        drive(ns, t, 25, [](double) { return Obs{false, 120, 30}; });
        const double at = drive(ns, t, 30, [](double) { return Obs{true, 5, 30}; });
        check(at >= 10 && at <= 11, "dark again 25 s after a take-back: normal ~10 s hand-over (" +
              std::to_string(at) + ")");
    }
    {
        // Dusk: the camera's auto-exposure runs at full rate (luma 60), but
        // frame-rate priority at its ceiling only reaches luma 5 — every
        // take-back fails, so the wait doubles up to the cap.  (The first
        // hand-over is the session-start probe: no doubling.)
        NightModeSwitch ns; double t = 0;
        std::string holds;
        Mode last = ns.mode();
        for (int i = 0; i < 20000 && std::count(holds.begin(), holds.end(), ' ') < 7; ++i) {
            t += 0.5;
            const bool cam = ns.mode() == Mode::Camera;
            const Mode m = ns.update(t, true, cam ? 60.f : 5.f, 30, 30.0f);
            if (m == Mode::Camera && last != Mode::Camera) holds += std::to_string(int(ns.exitHoldSec())) + " ";
            last = m;
        }
        check(holds == "30 60 120 240 480 600 600 ", "premature take-backs double the wait, capped at 10 min (" + holds + ")");
        drive(ns, t, ns.exitHoldSec() + 2, [](double) { return Obs{true, 60, 30}; });
        drive(ns, t, 310, [](double) { return Obs{false, 120, 30}; });                     // real daylight
        check(ns.mode() == Mode::FrameRate && ns.exitHoldSec() == 30, "5 min of daylight resets the wait to 30 s");
    }

    std::printf("\nRESULT: %s (%d failure%s)\n", g_fails ? "FAIL" : "PASS", g_fails,
                g_fails == 1 ? "" : "s");
    return g_fails ? 1 : 0;
}
