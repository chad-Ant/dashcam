// Microphone level meter (MicLevel.h) and MIC IN hysteresis (SettingsLayout.h).
#include "../lib/es3Audio/MicLevel.h"
#include "../lib/es3Graphics/SettingsLayout.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace es3audio;

namespace {
constexpr double kPi = 3.14159265358979323846;
const uint32_t kWindow = 800;    // 50 ms at 16 kHz
const uint32_t kSettle = 1600;   // 100 ms

// Mono frames: a sine with the given RMS in dBFS (plus DC), length n.
std::vector<int16_t> sine(double rmsDbfs, uint32_t n, double dc = 0, double hz = 1000) {
    const double amplitude = std::sqrt(2.0) * 32768.0 * std::pow(10.0, rmsDbfs / 20.0);
    std::vector<int16_t> out(n);
    for (uint32_t i = 0; i < n; ++i) {
        double v = dc + amplitude * std::sin(2 * kPi * hz * i / 16000.0);
        v = v > 32767 ? 32767 : (v < -32768 ? -32768 : v);
        out[i] = static_cast<int16_t>(std::lround(v));
    }
    return out;
}
int levelOf(const std::vector<int16_t>& mono) {
    MicLevel meter;
    meter.addFrames(mono.data(), mono.size(), 1, 0);
    return meter.level();
}
bool near(int a, int b, int tolerance) { return std::abs(a - b) <= tolerance; }

void mapping() {
    assert(dbfsToLevel(-70, -70, -20) == 0 && dbfsToLevel(-80, -70, -20) == 0);
    assert(dbfsToLevel(-20, -70, -20) == 100 && dbfsToLevel(0, -70, -20) == 100);
    assert(dbfsToLevel(-45, -70, -20) == 50);
    assert(dbfsToLevel(-50, -20, -20) == 0);                 // degenerate range
    assert(rmsToDbfs(0) == -120.0f && std::fabs(rmsToDbfs(32768) - 0.0f) < 1e-4f);
}

void meter() {
    // Settling: no level until 100 ms plus one window.
    MicLevel m;
    const auto quiet = sine(-90, kSettle + kWindow - 1);
    m.addFrames(quiet.data(), quiet.size(), 1, 0);
    assert(m.level() == kMicLevelNone && m.windows() == 0);
    const int16_t one = 0;
    m.addFrames(&one, 1, 1, 0);
    assert(m.level() == 0 && m.windows() == 1);

    // Known levels (one settle + several windows): the bench's -70..-20 range.
    assert(levelOf(std::vector<int16_t>(kSettle + 4 * kWindow, 0)) == 0);
    assert(near(levelOf(sine(-45, kSettle + 4 * kWindow)), 50, 1));
    assert(near(levelOf(sine(-33, kSettle + 4 * kWindow)), 74, 1));   // speech p90 at 54 dB
    assert(near(levelOf(sine(-62, kSettle + 4 * kWindow)), 16, 1));   // quiet p90 at 54 dB
    assert(levelOf(sine(-10, kSettle + 4 * kWindow)) == 100);
    assert(levelOf(sine(3, kSettle + 4 * kWindow)) == 100);            // clipped square-ish wave

    // DC is removed: a large offset does not raise the level.
    assert(near(levelOf(sine(-45, kSettle + 4 * kWindow, 12000)), 50, 1));
    assert(levelOf(std::vector<int16_t>(kSettle + 4 * kWindow, -20000)) == 0);

    // Slot selection from interleaved stereo.
    const auto loud = sine(-30, kSettle + 4 * kWindow);
    std::vector<int16_t> stereo(loud.size() * 2, 0);
    for (size_t i = 0; i < loud.size(); ++i) stereo[2 * i] = loud[i];
    MicLevel left, right;
    left.addFrames(stereo.data(), loud.size(), 2, 0);
    right.addFrames(stereo.data(), loud.size(), 2, 1);
    assert(near(left.level(), 80, 1) && right.level() == 0);
    MicLevel bad;
    bad.addFrames(stereo.data(), loud.size(), 2, 2);                  // slot out of range
    bad.addFrames(nullptr, 10, 1, 0);
    assert(bad.level() == kMicLevelNone && bad.windows() == 0);

    // Chunking does not change the result.
    const auto signal = sine(-40, kSettle + 7 * kWindow + 123);
    MicLevel whole, pieces;
    whole.addFrames(signal.data(), signal.size(), 1, 0);
    for (size_t i = 0; i < signal.size(); i += 37)
        pieces.addFrames(signal.data() + i, signal.size() - i < 37 ? signal.size() - i : 37, 1, 0);
    assert(whole.level() == pieces.level() && whole.windows() == pieces.windows());

    // Instant attack, limited release (4 points per window), then reset().
    MicLevel r;
    const auto speech = sine(-25, kSettle + 2 * kWindow);
    r.addFrames(speech.data(), speech.size(), 1, 0);
    const int peak = r.level();
    assert(near(peak, 90, 1));
    const std::vector<int16_t> silence(kWindow, 0);
    r.addFrames(silence.data(), silence.size(), 1, 0);
    assert(r.level() == peak - 4);
    for (int i = 0; i < 30; ++i) r.addFrames(silence.data(), silence.size(), 1, 0);
    assert(r.level() == 0);
    r.addFrames(speech.data() + kSettle, kWindow, 1, 0);
    assert(near(r.level(), 90, 1));                                    // attack is immediate
    r.reset();
    assert(r.level() == kMicLevelNone && r.windows() == 0);
}

void detector() {
    using es3settings::MicDetector;
    MicDetector d;
    assert(!d.update(49, 50, 0) && d.update(50, 50, 10));             // on at the threshold
    assert(d.update(47, 50, 20));                                      // inside the margin: hold
    assert(d.update(45, 50, 30) && d.update(40, 50, 329));            // below: 299 ms, still on
    assert(!d.update(40, 50, 330));                                    // 300 ms below: off
    assert(d.update(60, 50, 400));
    assert(d.update(30, 50, 500) && d.update(48, 50, 700));           // back in the margin resets
    assert(d.update(30, 50, 750) && d.update(30, 50, 1049) && !d.update(30, 50, 1050));
    assert(d.update(70, 50, 1100) && !d.update(es3settings::kMicPaused, 50, 1101));  // paused: off at once
    assert(d.update(70, 50, 1200) && !d.update(es3settings::kNoMicLevel, 50, 1201)); // no input: off at once
    assert(!d.update(0, 0, 1300) && d.update(1, 0, 1301));            // threshold 0: any sound
    d.clear(); assert(!d.detected());
    // Millisecond wrap-around keeps the 300 ms hold.
    MicDetector w;
    assert(w.update(80, 50, 0xFFFFFF00u) && w.update(10, 50, 0xFFFFFFF0u));
    assert(w.update(10, 50, 0x000000FFu));                            // 271 ms later, wrapped
    assert(!w.update(10, 50, 0xFFFFFFF0u + 300u));
}
}  // namespace

int main() {
    mapping();
    meter();
    detector();
    puts("microphone level meter and MIC IN hysteresis checks passed");
}
