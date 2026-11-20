/**
 * @file SettingsLayout.h
 * @brief Portable geometry, hit testing and value mapping for the SET tab.
 *
 * Screen area above the tab strip is 320 x 224. Rows, top to bottom:
 * title bar; master volume slider + TEST; brightness slider; microphone
 * threshold slider + input indicator; RECALIBRATE TOUCH; SD/USB footer.
 */
#pragma once
#include <stdint.h>

namespace es3settings {
struct Box {
    int x, y, w, h;
    bool contains(int px, int py) const { return px >= x && px < x + w && py >= y && py < y + h; }
};

enum Slider : uint8_t { VolumeSlider = 0, BrightnessSlider = 1, MicSlider = 2, kSliderCount = 3 };
enum Target : uint8_t { NoTarget, VolumeTarget, BrightnessTarget, MicTarget, TestTarget, RecalibrateTarget };

constexpr int kTitleHeight = 22;
constexpr int kTrackLeft = 16;     // value 0 at this x
constexpr int kTrackRight = 228;   // value 100 at this x
constexpr int kRowPitch = 42;
constexpr int kKnobRadius = 7;
constexpr int kFooterTop = 196;    // SD / USB status lines start here

/** Label baseline row (text size 1) above slider s. */
inline int labelY(uint8_t slider) { return 26 + kRowPitch * slider; }
/** Touch band of slider s; wider than the drawn track so a finger can catch it. */
inline Box sliderZone(uint8_t slider) { return Box{0, 36 + kRowPitch * slider, 242, 28}; }
inline int trackCenterY(uint8_t slider) { return sliderZone(slider).y + sliderZone(slider).h / 2; }
inline Box testButton() { return Box{248, 30, 64, 34}; }
inline Box micIndicator() { return Box{248, 114, 64, 34}; }
inline Box recalibrateButton() { return Box{16, 160, 168, 28}; }

/** Slider value 0..100 for a touch at screen x (clamped to the track). */
inline uint8_t sliderValue(int x) {
    if (x <= kTrackLeft) return 0;
    if (x >= kTrackRight) return 100;
    const int span = kTrackRight - kTrackLeft;
    return static_cast<uint8_t>(((x - kTrackLeft) * 100 + span / 2) / span);
}
/** Knob centre x for a value 0..100 (clamped). */
inline int knobX(uint8_t value) {
    const int v = value > 100 ? 100 : value;
    return kTrackLeft + (kTrackRight - kTrackLeft) * v / 100;
}

inline Target hitTest(int x, int y) {
    if (sliderZone(VolumeSlider).contains(x, y)) return VolumeTarget;
    if (sliderZone(BrightnessSlider).contains(x, y)) return BrightnessTarget;
    if (sliderZone(MicSlider).contains(x, y)) return MicTarget;
    if (testButton().contains(x, y)) return TestTarget;
    if (recalibrateButton().contains(x, y)) return RecalibrateTarget;
    return NoTarget;
}

/** Backlight PWM duty for a brightness percentage; never fully dark (8..255). */
inline uint8_t brightnessDuty(uint8_t percent) {
    const int p = percent > 100 ? 100 : percent;
    return static_cast<uint8_t>(8 + (247 * p + 50) / 100);
}

/** Microphone input level: no source yet (the mic library is not implemented). */
constexpr int kNoMicLevel = -1;
/** Detected when a level is available, non-zero and at or above the threshold. */
inline bool micDetected(int level, uint8_t threshold) {
    return level > 0 && level >= threshold;
}
}  // namespace es3settings
