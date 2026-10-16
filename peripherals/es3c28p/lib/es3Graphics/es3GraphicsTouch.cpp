#include "GraphicsInternal.h"
#include "es3Board.h"
#include <cstdlib>
namespace {
constexpr int kCalibrationTargetX[12] = {
    24, 114, 205, 295,
    24, 114, 205, 295,
    24, 114, 205, 295,
};
constexpr int kCalibrationTargetY[12] = {
    24, 24, 24, 24,
    120, 120, 120, 120,
    215, 215, 215, 215,
};
}

bool Es3Graphics::readTouchRaw(TouchRaw& raw)
{
    // Poll TD_STATUS and first point together. IRQ may be a pulse, so gating
    // reads on its level would lose held touches and release events.
    Wire.beginTransmission(Es3Board::touchAddress);
    Wire.write(0x02);
    if (Wire.endTransmission(false) != 0 ||
        Wire.requestFrom(Es3Board::touchAddress, uint8_t{5}) != 5) {
        return false;
    }
    uint8_t data[5];
    for (uint8_t& value : data) {
        value = Wire.read();
    }
    const uint8_t count = data[0] & 0x0f;
    const uint8_t event = data[1] >> 6;
    if (count == 0 || count > 2 || event == 1 || event == 3) {
        return false;
    }
    raw.x = ((data[1] & 0x0f) << 8) | data[2];
    raw.y = ((data[3] & 0x0f) << 8) | data[4];
    // Native capacitive coordinates; the existing wizard learns orientation.
    return raw.x < 240 && raw.y < 320;
}

int Es3Graphics::mapCalibrationAxis(int raw, int atLowTarget, int atMiddleTarget,
                                    int atHighTarget, int lowPixel, int middlePixel,
                                    int highPixel, int screenSize) const
{
    const bool inFirstSegment = atMiddleTarget >= atLowTarget
                                    ? raw <= atMiddleTarget
                                    : raw >= atMiddleTarget;
    const long mapped = inFirstSegment
                            ? map(raw, atLowTarget, atMiddleTarget, lowPixel, middlePixel)
                            : map(raw, atMiddleTarget, atHighTarget, middlePixel, highPixel);
    return constrain(mapped, 0L, static_cast<long>(screenSize - 1));
}

int Es3Graphics::mapCalibrationAxisFourPoint(int raw, int atLowTarget,
                                              int atLeftMiddleTarget,
                                              int atRightMiddleTarget,
                                              int atHighTarget, int lowPixel,
                                              int leftMiddlePixel,
                                              int rightMiddlePixel, int highPixel,
                                              int screenSize) const
{
    const bool inFirstSegment = atLeftMiddleTarget >= atLowTarget
                                    ? raw <= atLeftMiddleTarget
                                    : raw >= atLeftMiddleTarget;
    const bool inSecondSegment = atRightMiddleTarget >= atLeftMiddleTarget
                                     ? raw <= atRightMiddleTarget
                                     : raw >= atRightMiddleTarget;
    const long mapped = inFirstSegment
                            ? map(raw, atLowTarget, atLeftMiddleTarget,
                                  lowPixel, leftMiddlePixel)
                            : (inSecondSegment
                                   ? map(raw, atLeftMiddleTarget, atRightMiddleTarget,
                                         leftMiddlePixel, rightMiddlePixel)
                                   : map(raw, atRightMiddleTarget, atHighTarget,
                                         rightMiddlePixel, highPixel));
    return constrain(mapped, 0L, static_cast<long>(screenSize - 1));
}

bool Es3Graphics::mapTouch(const TouchRaw& raw, int& screenX, int& screenY) const
{
    if (!_calibrationValid) {
        return false;
    }
    const int rawForX = _calibration.xAxisIsRawX ? raw.x : raw.y;
    const int rawForY = _calibration.xAxisIsRawX ? raw.y : raw.x;
    screenX = mapCalibrationAxisFourPoint(rawForX, _calibration.xAtLeft,
                                          _calibration.xAtLeftMiddle,
                                          _calibration.xAtRightMiddle,
                                          _calibration.xAtRight,
                                          kCalibrationMargin, kCalibrationFirstThirdX,
                                          kCalibrationSecondThirdX,
                                          kScreenWidth - 1 - kCalibrationMargin,
                                          kScreenWidth);
    screenY = mapCalibrationAxis(rawForY, _calibration.yAtTop,
                                 _calibration.yAtCenter, _calibration.yAtBottom,
                                 kCalibrationMargin, kCalibrationCenterY,
                                 kScreenHeight - 1 - kCalibrationMargin, kScreenHeight);
    return true;
}

void Es3Graphics::drawCalibrationTarget()
{
    panel.fillScreen(ILI9341_BLACK);
    panel.setTextSize(2);
    panel.setTextColor(ILI9341_WHITE);
    panel.setCursor(18, 146);
    panel.print("TOUCH CALIBRATION");
    panel.setTextSize(1);
    panel.setCursor(62, 170);
    panel.printf("Tap target %u of %u", _calibrationTarget + 1, kCalibrationPointCount);
    panel.setCursor(46, 184);
    panel.print("Use the center of each cross");
    panel.setCursor(46, 198);
    panel.print("Lift your finger between taps");

    const int x = kCalibrationTargetX[_calibrationTarget];
    const int y = kCalibrationTargetY[_calibrationTarget];
    panel.drawFastHLine(x - 12, y, 25, ILI9341_YELLOW);
    panel.drawFastVLine(x, y - 12, 25, ILI9341_YELLOW);
    panel.drawCircle(x, y, 7, ILI9341_YELLOW);
}

bool Es3Graphics::buildCalibration()
{
    const int rawXAcross = std::abs(static_cast<int>(_calibrationSamples[3].x) - _calibrationSamples[0].x) +
                            std::abs(static_cast<int>(_calibrationSamples[7].x) - _calibrationSamples[4].x) +
                            std::abs(static_cast<int>(_calibrationSamples[11].x) - _calibrationSamples[8].x);
    const int rawYAcross = std::abs(static_cast<int>(_calibrationSamples[3].y) - _calibrationSamples[0].y) +
                            std::abs(static_cast<int>(_calibrationSamples[7].y) - _calibrationSamples[4].y) +
                            std::abs(static_cast<int>(_calibrationSamples[11].y) - _calibrationSamples[8].y);
    Es3TouchCalibrationRecord calibration{};
    calibration.xAxisIsRawX = rawXAcross >= rawYAcross;

    const auto xAxis = [&calibration](const TouchRaw& sample) -> int {
        return calibration.xAxisIsRawX ? sample.x : sample.y;
    };
    const auto yAxis = [&calibration](const TouchRaw& sample) -> int {
        return calibration.xAxisIsRawX ? sample.y : sample.x;
    };
    calibration.xAtLeft = (xAxis(_calibrationSamples[0]) + xAxis(_calibrationSamples[4]) +
                           xAxis(_calibrationSamples[8])) / 3;
    calibration.xAtLeftMiddle = (xAxis(_calibrationSamples[1]) + xAxis(_calibrationSamples[5]) +
                                 xAxis(_calibrationSamples[9])) / 3;
    calibration.xAtRightMiddle = (xAxis(_calibrationSamples[2]) + xAxis(_calibrationSamples[6]) +
                                  xAxis(_calibrationSamples[10])) / 3;
    calibration.xAtRight = (xAxis(_calibrationSamples[3]) + xAxis(_calibrationSamples[7]) +
                            xAxis(_calibrationSamples[11])) / 3;
    calibration.yAtTop = (yAxis(_calibrationSamples[0]) + yAxis(_calibrationSamples[1]) +
                          yAxis(_calibrationSamples[2]) + yAxis(_calibrationSamples[3])) / 4;
    calibration.yAtCenter = (yAxis(_calibrationSamples[4]) + yAxis(_calibrationSamples[5]) +
                             yAxis(_calibrationSamples[6]) + yAxis(_calibrationSamples[7])) / 4;
    calibration.yAtBottom = (yAxis(_calibrationSamples[8]) + yAxis(_calibrationSamples[9]) +
                             yAxis(_calibrationSamples[10]) + yAxis(_calibrationSamples[11])) / 4;
    if (!es3CalibrationIsSane(calibration)) {
        return false;
    }
    _calibration = calibration;
    return true;
}

void Es3Graphics::startCalibration()
{
    _calibrationValid = false;
    _calibrating = true;
    _calibrationTesting = false;
    _calibrationTouchHeld = false;
    _calibrationCompletePending = false;
    _calibrationTarget = 0;
    drawCalibrationTarget();
}

void Es3Graphics::finishCalibration()
{
    _calibrating = false;
    _calibrationValid = buildCalibration();
    if (!_calibrationValid) {
        startCalibration();
        return;
    }
    startCalibrationTest();
}

void Es3Graphics::handleCalibration()
{
    TouchRaw raw{};
    const bool pressed = readTouchRaw(raw);
    if (!pressed) {
        if (_calibrationCompletePending) {
            finishCalibration();
        }
        _calibrationTouchHeld = false;
        return;
    }
    if (_calibrationTouchHeld) {
        return;
    }

    _calibrationTouchHeld = true;
    _calibrationSamples[_calibrationTarget] = raw;
    Serial0.printf("[es3] calibration %u raw=%u,%u\n", _calibrationTarget + 1, raw.x, raw.y);
    ++_calibrationTarget;
    if (_calibrationTarget == kCalibrationPointCount) {
        _calibrationCompletePending = true;
        return;
    }
    drawCalibrationTarget();
}

void Es3Graphics::startCalibrationTest()
{
    _calibrationTesting = true;
    _calibrationTestTouchHeld = false;
    drawCalibrationTest();
}

void Es3Graphics::drawCalibrationTest()
{
    panel.fillScreen(ILI9341_BLACK);
    panel.setTextSize(2);
    panel.setTextColor(ILI9341_WHITE);
    panel.setCursor(28, 10);
    panel.print("TOUCH CHECK");
    panel.setTextSize(1);
    panel.setTextColor(ILI9341_LIGHTGREY);
    panel.setCursor(32, 34);
    panel.print("Tap the targets: dot follows touch");

    panel.fillRoundRect(14, 52, 130, 28, 4, ILI9341_ORANGE);
    panel.fillRoundRect(176, 52, 130, 28, 4, ILI9341_GREEN);
    printCentered("RETRY", 79, 62, ILI9341_BLACK);
    printCentered("SAVE", 241, 62, ILI9341_BLACK);

    panel.setTextColor(ILI9341_CYAN);
    panel.setCursor(82, 98);
    panel.print("Brightness-control targets");
    panel.drawRoundRect(kBrightnessDecreaseLeft, kBrightnessControlTop,
                       kBrightnessDecreaseWidth, kBrightnessControlHeight, 4,
                       ILI9341_WHITE);
    panel.drawRoundRect(kBrightnessIncreaseLeft, kBrightnessControlTop,
                       kBrightnessIncreaseWidth, kBrightnessControlHeight, 4,
                       ILI9341_WHITE);
    printCentered("-", kBrightnessDecreaseLeft + kBrightnessDecreaseWidth / 2,
                  kBrightnessControlTop + 8, ILI9341_WHITE);
    printCentered("+", kBrightnessIncreaseLeft + kBrightnessIncreaseWidth / 2,
                  kBrightnessControlTop + 8, ILI9341_WHITE);

    panel.setTextColor(ILI9341_CYAN);
    panel.setCursor(110, 204);
    panel.print("Tab targets");
    for (uint8_t tab = 0; tab < _pageCount; ++tab) {
        const int x = es3map::tabEdge(tab, _pageCount);
        const int width = es3map::tabEdge(tab + 1, _pageCount) - x;
        panel.drawRect(x, kTabTop, width, kTabHeight, ILI9341_WHITE);
        panel.drawFastVLine(x + width / 2, kTabTop + 3, 10, ILI9341_YELLOW);
    }
}

void Es3Graphics::saveCalibration()
{
    _calibrationTesting = false;
    const bool saved = _storage.saveTouchCalibration(_calibration);
    Serial0.printf("[es3] touch calibration verified; SD save %s\n", saved ? "OK" : "failed");
    drawPage();
}

void Es3Graphics::handleCalibrationTest()
{
    TouchRaw raw{};
    if (!readTouchRaw(raw)) {
        _calibrationTestTouchHeld = false;
        return;
    }
    if (_calibrationTestTouchHeld) {
        return;
    }
    _calibrationTestTouchHeld = true;

    int screenX = 0;
    int screenY = 0;
    if (!mapTouch(raw, screenX, screenY)) {
        return;
    }
    Serial0.printf("[es3] touch check raw=%u,%u mapped=%d,%d\n",
                  raw.x, raw.y, screenX, screenY);
    if (screenY >= 52 && screenY < 80) {
        if (screenX >= 14 && screenX < 144) {
            startCalibration();
            return;
        }
        if (screenX >= 176 && screenX < 306) {
            saveCalibration();
            return;
        }
    }
    panel.fillCircle(screenX, screenY, 4, ILI9341_CYAN);
    panel.drawCircle(screenX, screenY, 7, ILI9341_WHITE);
}
