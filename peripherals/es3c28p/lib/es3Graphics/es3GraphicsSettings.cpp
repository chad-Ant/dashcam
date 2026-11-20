/** @file es3GraphicsSettings.cpp - SET tab: sliders, buttons and persistence. */

#include "GraphicsInternal.h"
#include <Adafruit_GFX.h>
#include "es3Board.h"
#include <cstdio>
#include <cstdlib>

using namespace es3settings;

namespace {
const char* const kSliderNames[kSliderCount] = {"MASTER VOLUME", "BRIGHTNESS", "MIC THRESHOLD"};
const uint16_t kSliderColours[kSliderCount] = {ILI9341_ORANGE, ILI9341_YELLOW, ILI9341_GREEN};
}  // namespace

bool Es3Graphics::setDefaultSettings(const Es3Settings& defaults)
{
    if (_started || defaults.masterVolume > 100 || defaults.brightness > 100 ||
        defaults.micThreshold > 100) {
        return false;
    }
    _settings = defaults;
    return true;
}

bool Es3Graphics::takeSettingsChange(Es3Settings& settings)
{
    if (!_settingsChanged) return false;
    _settingsChanged = false;
    settings = _settings;
    return true;
}

bool Es3Graphics::takeAudioTestRequest()
{
    const bool requested = _audioTestRequested;
    _audioTestRequested = false;
    return requested;
}

void Es3Graphics::setMicLevel(int level)
{
    const int clamped = level == kMicPaused ? kMicPaused : (level < 0 ? kNoMicLevel : (level > 100 ? 100 : level));
    const bool wasDetected = _micDetector.detected();
    const int oldState = _micLevel < 0 ? _micLevel : 0;   // none / paused / measuring
    _micLevel = clamped;
    const bool detected = _micDetector.update(_micLevel, _settings.micThreshold, millis());
    if (!settingsVisible()) return;
    if (detected != wasDetected || (_micLevel < 0 ? _micLevel : 0) != oldState) drawMicIndicator();
    drawMicLevelBar(false);
}

bool Es3Graphics::micMeterWanted() const { return settingsVisible(); }

void Es3Graphics::drawMicLevelBar(bool force)
{
    // Live level under the threshold track, so speech can be compared with
    // the knob. At most 10 redraws/s, and only when it moves 2+ points.
    const int level = _micLevel > 0 ? _micLevel : 0;
    const uint32_t now = millis();
    if (!force && (std::abs(level - _micBarLevel) < 2 || now - _micBarDrawnMs < 100) &&
        _micDetector.detected() == _micBarDetected) {
        return;
    }
    _micBarLevel = level;
    _micBarDrawnMs = now;
    _micBarDetected = _micDetector.detected();
    const int y = trackCenterY(MicSlider) + kKnobRadius + 2;
    const int end = knobX(static_cast<uint8_t>(level));
    panel.fillRect(kTrackLeft, y, end - kTrackLeft, 3, _micBarDetected ? ILI9341_GREEN : ILI9341_LIGHTGREY);
    panel.fillRect(end, y, kTrackRight + 1 - end, 3, ILI9341_BLACK);
}

bool Es3Graphics::settingsVisible() const
{
    return _started && !_calibrating && !_calibrationTesting && !_videoVisible &&
           _pages[_activePage].settingsPage;
}

void Es3Graphics::applyBrightnessSetting()
{
    const uint8_t duty = es3settings::brightnessDuty(_settings.brightness);
    if (_brightnessConfigured) {
        // With the LDR, the slider sets the bright level; dim never exceeds it.
        _brightnessConfig.maxDuty = duty;
        _brightnessConfig.minDuty = _autoMinDuty < duty ? _autoMinDuty : duty;
        return;  // updateBrightness() eases towards the new target
    }
    _brightnessDuty = duty;
    _brightnessTargetDuty = duty;
    if (_brightnessPwmAttached) ledcWrite(Es3Board::backlight, _brightnessDuty);
}

void Es3Graphics::drawSettingsPage()
{
    panel.fillRect(0, 0, kScreenWidth, kTitleHeight, ILI9341_DARKCYAN);
    panel.setTextSize(2);
    panel.setTextColor(ILI9341_WHITE);
    panel.setCursor(8, 4);
    panel.print(_pages[_activePage].title);
    drawSettingsState();

    for (uint8_t slider = 0; slider < kSliderCount; ++slider) drawSlider(slider);

    const Box test = testButton();
    panel.fillRoundRect(test.x, test.y, test.w, test.h, 5, ILI9341_ORANGE);
    panel.setTextSize(2);
    panel.setTextColor(ILI9341_BLACK);
    panel.setCursor(test.x + (test.w - 4 * 12) / 2, test.y + (test.h - 16) / 2 + 1);
    panel.print("TEST");
    drawMicIndicator();

    const Box recal = recalibrateButton();
    panel.fillRoundRect(recal.x, recal.y, recal.w, recal.h, 5, ILI9341_BLUE);
    panel.setTextColor(ILI9341_WHITE);
    panel.setCursor(recal.x + (recal.w - 11 * 12) / 2, recal.y + (recal.h - 16) / 2 + 1);
    panel.print("RECALIBRATE");
    drawFooter();
}

void Es3Graphics::drawSettingsState()
{
    const char* text = "DEFAULTS";
    uint16_t colour = ILI9341_LIGHTGREY;
    switch (_settingsState) {
    case SettingsLoaded: text = "FROM SD"; colour = ILI9341_GREEN; break;
    case SettingsSaved: text = "SAVED"; colour = ILI9341_GREEN; break;
    case SettingsSaveFailed: text = "NOT SAVED"; colour = ILI9341_RED; break;
    case SettingsDefaults: break;
    }
    constexpr int kStateLeft = 236;
    panel.fillRect(kStateLeft, 0, kScreenWidth - kStateLeft, kTitleHeight, ILI9341_DARKCYAN);
    panel.setTextSize(1);
    panel.setTextColor(colour);
    panel.setCursor(kScreenWidth - 8 - static_cast<int>(std::strlen(text)) * 6, 8);
    panel.print(text);
}

void Es3Graphics::drawSlider(uint8_t slider)
{
    if (slider >= kSliderCount) return;
    const uint8_t values[kSliderCount] = {_settings.masterVolume, _settings.brightness,
                                          _settings.micThreshold};
    const uint8_t value = values[slider];
    const Box zone = sliderZone(slider);
    const int labelTop = labelY(slider);
    const int y = trackCenterY(slider);
    const int knob = knobX(value);
    const uint16_t colour = kSliderColours[slider];

    // Compose the label line and the track strip off screen, then send each to
    // the panel in one write. Clearing on the panel first made every drag step
    // flash black (visible flicker of the text and the bar).
    static GFXcanvas16 labelCanvas(242, 8);                  // one text line
    static GFXcanvas16 trackCanvas(242, 2 * kKnobRadius + 1); // knob height
    char text[32];
    if (slider == VolumeSlider && value == 0) {
        snprintf(text, sizeof(text), "%s  MUTED", kSliderNames[slider]);
    } else {
        snprintf(text, sizeof(text), "%s  %u%%", kSliderNames[slider], value);
    }
    if (labelCanvas.getBuffer() == nullptr || trackCanvas.getBuffer() == nullptr ||
        zone.w > labelCanvas.width()) {
        // No canvas memory: draw directly (flickers while dragging, still correct).
        panel.fillRect(zone.x, labelTop - 1, zone.w, zone.y - labelTop + 1, ILI9341_BLACK);
        panel.setTextSize(1);
        panel.setTextColor(ILI9341_CYAN);
        panel.setCursor(kTrackLeft, labelTop);
        panel.print(text);
        panel.fillRect(zone.x, y - kKnobRadius, zone.w, 2 * kKnobRadius + 1, ILI9341_BLACK);
        panel.fillRoundRect(kTrackLeft, y - 3, kTrackRight - kTrackLeft + 1, 7, 3, ILI9341_DARKGREY);
        if (knob > kTrackLeft) panel.fillRoundRect(kTrackLeft, y - 3, knob - kTrackLeft + 1, 7, 3, colour);
        panel.fillCircle(knob, y, kKnobRadius, ILI9341_WHITE);
        panel.drawCircle(knob, y, kKnobRadius, colour);
    } else {
        labelCanvas.fillScreen(ILI9341_BLACK);
        labelCanvas.setTextSize(1);
        labelCanvas.setTextColor(ILI9341_CYAN);
        labelCanvas.setCursor(kTrackLeft - zone.x, 0);
        labelCanvas.print(text);
        panel.drawRGBBitmap(zone.x, labelTop, labelCanvas.getBuffer(), zone.w, labelCanvas.height());

        const int cy = kKnobRadius;   // track centre inside the canvas
        const int left = kTrackLeft - zone.x, right = kTrackRight - zone.x, k = knob - zone.x;
        trackCanvas.fillScreen(ILI9341_BLACK);
        trackCanvas.fillRoundRect(left, cy - 3, right - left + 1, 7, 3, ILI9341_DARKGREY);
        if (k > left) trackCanvas.fillRoundRect(left, cy - 3, k - left + 1, 7, 3, colour);
        trackCanvas.fillCircle(k, cy, kKnobRadius, ILI9341_WHITE);
        trackCanvas.drawCircle(k, cy, kKnobRadius, colour);
        panel.drawRGBBitmap(zone.x, y - kKnobRadius, trackCanvas.getBuffer(), zone.w, trackCanvas.height());
    }
    // The live level bar sits below the knob rows and is not touched above.
    if (slider == MicSlider) drawMicLevelBar(true);
}

void Es3Graphics::drawMicIndicator()
{
    const Box box = micIndicator();
    const bool detected = _micDetector.detected();
    const uint16_t fill = detected ? ILI9341_GREEN : ILI9341_DARKGREY;
    const uint16_t ink = detected ? ILI9341_BLACK : ILI9341_LIGHTGREY;
    panel.fillRoundRect(box.x, box.y, box.w, box.h, 5, fill);
    printCentered("MIC IN", box.x + box.w / 2, box.y + 7, ink);
    printCentered(_micLevel == kMicPaused ? "paused" : (_micLevel == kNoMicLevel ? "no input" : (detected ? "detected" : "quiet")),
                  box.x + box.w / 2, box.y + 20, ink);
}

void Es3Graphics::handleSettingsPress(int screenX, int screenY)
{
    const Target target = hitTest(screenX, screenY);
    switch (target) {
    case VolumeTarget:
    case BrightnessTarget:
    case MicTarget:
        _dragSlider = target == VolumeTarget ? VolumeSlider
                      : (target == BrightnessTarget ? BrightnessSlider : MicSlider);
        _settingsEdited = false;
        setSliderValue(_dragSlider, sliderValue(screenX));
        break;
    case TestTarget:
        _audioTestRequested = true;
        Serial0.printf("[es3] volume test at %u%%\n", _settings.masterVolume);
        break;
    case RecalibrateTarget:
        requestCalibration();
        break;
    case NoTarget:
        break;
    }
}

void Es3Graphics::setSliderValue(uint8_t slider, uint8_t value)
{
    uint8_t* fields[kSliderCount] = {&_settings.masterVolume, &_settings.brightness,
                                     &_settings.micThreshold};
    if (slider >= kSliderCount || *fields[slider] == value) return;
    const bool wasDetected = _micDetector.detected();
    *fields[slider] = value;
    _settingsEdited = true;
    if (slider == BrightnessSlider) applyBrightnessSetting();
    if (!settingsVisible()) return;
    drawSlider(slider);
    if (slider == MicSlider) {
        // drawSlider() above already redrew the level bar.
        if (_micDetector.update(_micLevel, _settings.micThreshold, millis()) != wasDetected) drawMicIndicator();
    }
}

void Es3Graphics::finishSliderDrag()
{
    _dragSlider = kNoSlider;
    if (!_settingsEdited) return;
    _settingsEdited = false;
    // One write per finger lift, not per drag step.
    const bool saved = _storage.saveSettings(_settings);
    _settingsState = saved ? SettingsSaved : SettingsSaveFailed;
    _settingsChanged = true;
    Serial0.printf("[es3] settings volume %u%% brightness %u%% mic threshold %u%%; SD save %s\n",
                   _settings.masterVolume, _settings.brightness, _settings.micThreshold,
                   saved ? "OK" : "failed");
    if (settingsVisible()) drawSettingsState();
}
