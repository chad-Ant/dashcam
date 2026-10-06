#include "libdeadreckoning.h"
#include "c_api.h"
#include <cmath>
#include <cstdio>
#include <stdexcept>
using namespace dashcam::deadreckoning;
namespace {
int checks = 0, failures = 0;
void check(bool pass, const char* name) {
    ++checks;
    if (!pass) { ++failures; std::fprintf(stderr, "FAIL %s\n", name); }
}
bool near(double a, double b, double tolerance = 1e-5) { return std::abs(a - b) < tolerance; }
Config raw() {
    Config c;
    c.filterTauSec = c.speedSlewMps2 = c.yawSlewRadps2 = 0;
    c.maxUnanchoredSec = 600;
    return c;
}
WheelSample sample(uint64_t ns, uint16_t rl = 720, uint16_t rr = 720, Gear gear = Gear::Forward) {
    WheelSample s;
    s.monotonicNs = ns; s.rearLeft = rl; s.rearRight = rr;
    s.gear = gear; s.valid = true;
    return s;
}
State feed(Estimator& e, uint64_t ns, uint16_t rl = 720, uint16_t rr = 720, Gear gear = Gear::Forward) {
    return e.update(sample(ns, rl, rr, gear), ns);
}
void geometryTests() {
    Estimator e(raw());
    check(feed(e, 0).status == Status::Primed, "first sample establishes acquisition origin");
    State s;
    for (unsigned i = 1; i <= 100; ++i) s = feed(e, i * 100000000ULL);
    check(near(s.xM, 20) && near(s.yM, 0), "straight 2 m/s for 10 seconds");
    check(near(s.distanceM, 20) && s.continuous, "continuous distance");
    check(std::isnan(s.radiusM) && near(s.roadWheelAngleRad, 0), "straight radius unavailable");
    check(near(s.meanSpeedMps, 2), "reused MKR moving average");
    e.reset();
    feed(e, 0);
    uint64_t jitterTime = 0;
    for (unsigned i = 0; i < 100; ++i) {
        jitterTime += (i % 2 ? 150000000ULL : 50000000ULL);
        s = feed(e, jitterTime);
    }
    check(near(s.xM, 20), "acquisition timestamp jitter does not change straight distance");
    e.reset(3, 4, pi / 2);
    feed(e, 0);
    s = feed(e, 100000000);
    check(near(s.xM, 3) && near(s.yM, 4.2), "external local anchor");
    Config c = raw();
    c.yawDpsPerCount = (0.2 * 180 / pi) / 100;
    Estimator circle(c);
    feed(circle, 0, 670, 770);
    for (unsigned i = 1; i <= 300; ++i) s = feed(circle, i * 100000000ULL, 670, 770);
    check(near(s.xM, 10 * std::sin(6), 1e-4), "exact left arc x");
    check(near(s.yM, 10 * (1 - std::cos(6)), 1e-4), "exact left arc y");
    check(near(s.radiusM, 10), "rear centre radius");
    check(near(s.roadWheelAngleRad, std::atan(2.405 / 10)), "bicycle wheel angle");
    const uint64_t fullLap = static_cast<uint64_t>(2 * pi / .2 * 1e9);
    for (uint64_t t = 30100000000ULL; t < fullLap; t += 100000000ULL) feed(circle, t, 670, 770);
    s = feed(circle, fullLap, 670, 770);
    check(std::hypot(s.xM, s.yM) < 1e-4 && std::abs(s.headingRad) < 1e-5, "full lap closure");
    circle.reset();
    feed(circle, 0, 770, 670);
    s = feed(circle, 100000000, 770, 670);
    check(s.yM < 0 && s.headingRad < 0 && s.roadWheelAngleRad < 0, "right turn signs");
    circle.reset();
    feed(circle, 0, 670, 770, Gear::Reverse);
    s = feed(circle, 100000000, 670, 770, Gear::Reverse);
    check(s.xM < 0 && s.headingRad < 0 && s.roadWheelAngleRad > 0, "reverse yaw and equivalent angle");
}
void faultTests() {
    Estimator e(raw());
    feed(e, 1000000000);
    auto s = feed(e, 1100000000);
    const double before = s.xM;
    s = feed(e, 2000000000);
    check(s.status == Status::Gap && !s.continuous && near(s.xM, before), "no integration across gap");
    s = feed(e, 2100000000);
    check(s.status == Status::Primed && !s.continuous && near(s.xM, before), "recovery never repairs unknown travel");
    s = feed(e, 2200000000);
    check(s.status == Status::Tracking && !s.continuous, "partial trajectory remains labelled");
    s = feed(e, 2100000000);
    check(s.status == Status::BadTime, "backward timestamp refused");
    check(feed(e, 2150000000).status == Status::BadTime, "high-water clock not rewound");
    s = feed(e, 2300000000, 0, 0);
    check(s.status == Status::LowSpeed && std::isnan(s.yawRadps), "zero wheels not standstill");
    check(feed(e, 2400000000, 299, 800).status == Status::LowSpeed, "one wheel below cutoff");
    check(feed(e, 2500000000, 65535, 800).status == Status::InvalidInput, "sentinel rejected");
    check(feed(e, 2600000000, 800, 800, Gear::Unknown).status == Status::InvalidInput, "missing gear rejected");
    check(e.update(sample(2700000000), 2600000000).status == Status::BadTime, "future timestamp");
    check(e.update(sample(2700000000), 4000000000).status == Status::Stale, "old sample");
    e.reset();
    check(feed(e, 0).continuous, "only explicit reset restores continuity");
    s = e.snapshot(1000000000);
    check(s.status == Status::Stale && std::isnan(s.speedMps) && !s.continuous, "silence snapshot stale");
    check(feed(e, 100000000, 720, 720, Gear::Reverse).status == Status::Gap, "gear reversal breaks integration");
    e.reset();
    auto parked = sample(0, 0, 0, Gear::Park);
    parked.stationaryConfirmed = true;
    e.update(parked, 0);
    parked.monotonicNs = 100000000;
    s = e.update(parked, parked.monotonicNs);
    check(s.status == Status::Stationary && near(s.xM, 0), "independently confirmed stationary");
    parked.monotonicNs += 100000000;
    parked.rearLeft = 100;
    check(e.update(parked, parked.monotonicNs).status == Status::InvalidInput, "contradictory stationary evidence");
}
void signalTests() {
    float value = 10;
    saturate(value, -2, 2);
    check(value == 2, "MKR saturation swaps bounds");
    rateLimit(-10, value, .5f);
    check(value == 1.5f, "MKR slew limiter");
    check(near(angleDiff360(359, 1), 2), "MKR circular difference wraps");
    SimpleMovingAverage average(SIZE_8);
    float input = 3, output = 0;
    for (int i = 0; i < 7; ++i) check(!average.calculate(input, output), "MKR average warmup");
    check(average.calculate(input, output) && output == 3, "MKR average valid");
    average.reset();
    check(!average.calculate(input, output), "MKR average reset");
    Config c = raw(); c.mismatchRatio = .02;
    Estimator matched(c);
    const auto s = feed(matched, 0, 990, 1010);
    check(near(s.yawRadps, 0), "ratio tyre mismatch correction");
    Estimator smooth;
    feed(smooth, 0, 720, 720);
    const auto a = feed(smooth, 100000000, 1500, 1500);
    check(a.limited && a.speedMps > 2 && a.speedMps < 3.2, "dt slew plus time-aware lowpass");
}
void limitsTests() {
    Config c = raw(); c.maxDistanceM = 1;
    Estimator e(c);
    feed(e, 0);
    for (unsigned i = 1; i < 5; ++i) feed(e, i * 100000000ULL);
    const auto s = feed(e, 600000000);
    check(s.status == Status::DriftLimit && s.distanceM <= 1, "distance budget freezes before exceedance");
    check(feed(e, 700000000).status == Status::DriftLimit, "distance limit stays latched");
    c = raw(); c.maxUnanchoredSec = 1;
    Estimator timed(c);
    feed(timed, 0);
    for (unsigned i = 1; i <= 10; ++i) feed(timed, i * 100000000ULL);
    check(feed(timed, 1100000000).status == Status::DriftLimit, "time budget");
    check(feed(timed, 1200000000).status == Status::DriftLimit, "time limit stays latched");
    bool caught = false;
    try { Config bad; bad.filterTauSec = unavailable; Estimator invalid(bad); }
    catch (const std::invalid_argument&) { caught = true; }
    check(caught, "nonfinite config rejected");
    caught = false;
    try { e.reset(unavailable, 0, 0); } catch (const std::invalid_argument&) { caught = true; }
    check(caught, "nonfinite anchor rejected");
    void* handle = dr_create(2.405, .01, .1083, 0, 0, 10);
    check(handle != nullptr, "C API create");
    dr_result result{};
    check(dr_update(handle, 0, 0, 720, 720, 1, 1, &result) == 1 && result.continuous, "C API update");
    check(dr_update(handle, 1, 1, 720, 720, 5, 1, &result) == 0, "C API bad argument");
    check(dr_reset(handle) == 1, "C API reset");
    dr_destroy(handle);
    check(dr_create(0, .01, .1083, 0, 0, 10) == nullptr, "C API config error contained");
}
}
int main() {
    geometryTests(); faultTests(); signalTests(); limitsTests();
    std::printf("deadreckoning: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
