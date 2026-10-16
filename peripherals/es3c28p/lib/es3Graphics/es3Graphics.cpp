/** @file es3Graphics.cpp */

#include "GraphicsInternal.h"

#include <SPI.h>
#include "es3Board.h"
#include "Es3Panel.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <cstring>
#include <cstdlib>

namespace {
constexpr int kTftMiso = Es3Board::tftMiso;
constexpr int kTftMosi = Es3Board::tftMosi;
constexpr int kTftSclk = Es3Board::tftClock;
constexpr int kTftCs = Es3Board::tftCs;
constexpr int kTftDc = Es3Board::tftDc;
constexpr int kTftRst = -1;  // shared with CPU EN
constexpr int kTftBacklight = Es3Board::backlight;
constexpr uint32_t kBacklightPwmHz = 5000;
constexpr uint8_t kBacklightPwmBits = 8;

constexpr uint32_t kTouchDebounceMs = 180;
constexpr char kSplashRgb565File[] = "/splash.rgb565";
constexpr char kSplashBmpFile[] = "/splash.bmp";
SPIClass gTftSpi(FSPI);

}  // namespace

namespace es3graphics {
Adafruit_ILI9341 panel(&gTftSpi, kTftDc, kTftCs, kTftRst);
}

Es3Graphics::Es3Graphics(Es3Storage& storage)
    : _storage(storage)
    , _pages(nullptr)
    , _pageCount(0)
    , _activePage(0)
    , _started(false)
    , _calibrationValid(false)
    , _calibrating(false)
    , _calibrationTesting(false)
    , _calibrationTouchHeld(false)
    , _calibrationTestTouchHeld(false)
    , _calibrationCompletePending(false)
    , _calibrationTarget(0)
    , _calibration{}
    , _touchHeld(false)
    , _lastTouchMs(0)
    , _brightnessConfig{false, -1, 0, 0, 255, 255, 0, 1, 1000}
    , _brightnessConfigured(false)
    , _brightnessPwmAttached(false)
    , _brightnessDuty(255)
    , _brightnessTargetDuty(255)
    , _filteredLightRaw(0)
    , _lastBrightnessUpdateMs(0)
    , _lastBrightnessLogMs(0)
{
    for (TouchRaw& sample : _calibrationSamples) {
        sample = {0, 0};
    }
}

bool Es3Graphics::setPages(const Es3DashboardPage* pages, uint8_t count)
{
    if (_started || !es3ValidPages(pages, count, kMaxPages)) return false;
    // Own the text so USB updates never retain pointers into an RX buffer.
    for (uint8_t i = 0; i < count; ++i) {
        _pageStorage[i] = pages[i];
        const char* fields[] = {pages[i].title, pages[i].line1, pages[i].line2,
                                pages[i].line3, pages[i].line4};
        for (uint8_t j = 0; j < 5; ++j) {
            std::strcpy(_pageText[i][j], fields[j]);
        }
        _pageStorage[i].title = _pageText[i][0];
        _pageStorage[i].line1 = _pageText[i][1];
        _pageStorage[i].line2 = _pageText[i][2];
        _pageStorage[i].line3 = _pageText[i][3];
        _pageStorage[i].line4 = _pageText[i][4];
    }
    _pages = _pageStorage;
    _pageCount = count;
    return true;
}

bool Es3Graphics::updatePageText(uint8_t page, const char (&text)[5][24])
{
    if (_pages == nullptr || page >= _pageCount) return false;
    for (const auto& field : text) {
        const char* end = static_cast<const char*>(std::memchr(field, 0, sizeof(field)));
        if (end == nullptr) return false;
        for (const char* p = field; p != end; ++p) {
            if (*p < 32 || *p > 126) return false;
        }
    }
    std::memcpy(_pageText[page], text, sizeof(_pageText[page]));
    _pageStorage[page].showSdSplash = false;
    if (page == _activePage) _pageDirty = true;
    return true;
}

void Es3Graphics::setHostConnected(bool connected)
{
    if (_hostConnected == connected) return;
    _hostConnected = connected;
    _pageDirty = true;
}

bool Es3Graphics::brightnessConfigIsSane(const Es3BrightnessConfig& config) const
{
    if (!config.enabled) {
        return true;
    }
    return config.ldrPin == Es3Board::externalLdr &&
           config.darkRaw != config.brightRaw &&
           config.minDuty <= config.maxDuty &&
           config.brightThresholdRaw <= 4095 &&
           config.transitionStep > 0 &&
           config.updateIntervalMs >= 50;
}

bool Es3Graphics::setBrightnessConfig(const Es3BrightnessConfig& config)
{
    if (_started || !brightnessConfigIsSane(config)) {
        return false;
    }
    _brightnessConfig = config;
    _brightnessConfigured = config.enabled;
    return true;
}

bool Es3Graphics::setBrightnessThresholdRaw(uint16_t raw)
{
    if (!_brightnessConfigured || raw > 4095) {
        return false;
    }
    _brightnessConfig.brightThresholdRaw = raw;
    Serial0.printf("[es3] brightness threshold L=%u\n", raw);
    if (_started && _pages[_activePage].showBrightnessControl) {
        drawBrightnessControls();
    }
    return true;
}

bool Es3Graphics::begin()
{
    if (_started || _pages == nullptr || _pageCount == 0) {
        return false;
    }

    pinMode(kTftBacklight, OUTPUT);
    if (_brightnessConfigured) {
        analogReadResolution(12);
        // External 3.3 V LDR divider on the expansion connector.
        analogSetPinAttenuation(_brightnessConfig.ldrPin, ADC_11db);
        _brightnessPwmAttached = ledcAttach(kTftBacklight, kBacklightPwmHz, kBacklightPwmBits);
        if (!_brightnessPwmAttached) {
            return false;
        }
        updateBrightness(true);
    } else {
        _brightnessPwmAttached = ledcAttach(kTftBacklight, kBacklightPwmHz, kBacklightPwmBits);
        if (!_brightnessPwmAttached) {
            return false;
        }
        ledcWrite(kTftBacklight, _brightnessDuty);
    }
    gTftSpi.begin(kTftSclk, kTftMiso, kTftMosi, kTftCs);
    panel.begin();
    es3ConfigureIpsPanel(panel);
    panel.setRotation(1);

    if (!Es3Board::beginI2c()) {
        return false;
    }
    pinMode(Es3Board::touchReset, OUTPUT);
    digitalWrite(Es3Board::touchReset, LOW);
    delay(10);
    digitalWrite(Es3Board::touchReset, HIGH);
    delay(200);
    pinMode(Es3Board::touchIrq, INPUT);
    Wire.beginTransmission(Es3Board::touchAddress);
    if (Wire.endTransmission() != 0) {
        Serial0.println("[es3] FT6336G not found at 0x38");
        return false;
    }

    _calibrationValid = _storage.loadTouchCalibration(_calibration);
    _started = true;
    Serial0.printf("[es3] graphics ready; SD %s; calibration %s\n",
                  _storage.available() ? "OK" : "not found",
                  _calibrationValid ? "loaded" : "needed");
    if (_calibrationValid) {
        drawPage();
    } else {
        startCalibration();
    }
    return true;
}

void Es3Graphics::update()
{
    if (!_started) {
        return;
    }
    if (_calibrating) {
        handleCalibration();
    } else if (_calibrationTesting) {
        handleCalibrationTest();
    } else {
        handleDashboardTouch();
    }
    if (_pageDirty && !_calibrating && !_calibrationTesting) {
        if (!_videoVisible || _mapInfo) drawPage();
        else _videoStatus = 255;
        _pageDirty = false;
    }
    updateBrightness(false);
    updateVideo();
}

void Es3Graphics::requestCalibration()
{
    if (!_started) {
        return;
    }
    startCalibration();
}

bool Es3Graphics::sdAvailable() const { return _storage.available(); }
bool Es3Graphics::calibrationLoaded() const { return _calibrationValid; }
uint8_t Es3Graphics::activePage() const { return _activePage; }
uint8_t Es3Graphics::brightnessDuty() const { return _brightnessDuty; }
uint16_t Es3Graphics::brightnessThresholdRaw() const
{
    return _brightnessConfig.brightThresholdRaw;
}
int Es3Graphics::filteredLightRaw() const { return _filteredLightRaw; }

void Es3Graphics::updateBrightness(bool force)
{
    if (!_brightnessConfigured || !_brightnessPwmAttached) {
        return;
    }
    const uint32_t now = millis();
    if (!force && now - _lastBrightnessUpdateMs < _brightnessConfig.updateIntervalMs) {
        return;
    }
    _lastBrightnessUpdateMs = now;

    const int sample = constrain(analogRead(_brightnessConfig.ldrPin), 0, 4095);
    if (_filteredLightRaw == 0 || force) {
        _filteredLightRaw = sample;
    } else {
        // First-order 1/4 low-pass filter prevents visible flicker from noise.
        _filteredLightRaw = (_filteredLightRaw * 3 + sample) / 4;
    }
    // The board needs only two intentional brightness levels. Compare the
    // actual LDR reading with a numeric threshold. The comparison reverses
    // automatically for boards whose divider rises rather than falls in light.
    const bool brighterThanThreshold = _brightnessConfig.brightRaw < _brightnessConfig.darkRaw
                                           ? _filteredLightRaw <= _brightnessConfig.brightThresholdRaw
                                           : _filteredLightRaw >= _brightnessConfig.brightThresholdRaw;
    _brightnessTargetDuty = brighterThanThreshold ? _brightnessConfig.maxDuty
                                                   : _brightnessConfig.minDuty;

    // Ease into a target instead of producing a distracting full-scale jump.
    const uint8_t step = _brightnessConfig.transitionStep;
    if (_brightnessDuty < _brightnessTargetDuty) {
        _brightnessDuty = static_cast<uint8_t>(min<int>(
            _brightnessDuty + step, _brightnessTargetDuty));
    } else if (_brightnessDuty > _brightnessTargetDuty) {
        _brightnessDuty = static_cast<uint8_t>(max<int>(
            _brightnessDuty - step, _brightnessTargetDuty));
    }
    ledcWrite(kTftBacklight, _brightnessDuty);

    if (_started && !_calibrating) {
        drawBrightnessStatus();
    }
    if (now - _lastBrightnessLogMs >= 1000U) {
        _lastBrightnessLogMs = now;
        Serial0.printf("[es3] LDR raw=%d, backlight=%u->%u/255\n",
                      _filteredLightRaw, _brightnessDuty, _brightnessTargetDuty);
    }
}

void Es3Graphics::drawBrightnessStatus()
{
    if (_videoVisible) return;
    if (!_brightnessConfigured) {
        return;
    }
    // This diagnostic remains on every dashboard page. It makes it possible
    // to distinguish an unresponsive sensor from an unresponsive backlight
    // without a serial monitor: L is the filtered LDR ADC value and B is PWM.
    panel.fillRect(184, 198, 132, 13, ILI9341_BLACK);
    panel.setTextSize(1);
    panel.setTextColor(ILI9341_ORANGE);
    panel.setCursor(184, 202);
    panel.printf("L:%4d B:%3u", _filteredLightRaw, _brightnessDuty);
}

void Es3Graphics::drawBrightnessControls()
{
    if (!_pages[_activePage].showBrightnessControl) {
        return;
    }

    panel.fillRect(0, kBrightnessControlTop - 2, kScreenWidth,
                  kBrightnessControlHeight + 4, ILI9341_BLACK);
    panel.fillRoundRect(kBrightnessDecreaseLeft, kBrightnessControlTop,
                       kBrightnessDecreaseWidth, kBrightnessControlHeight, 4,
                       ILI9341_WHITE);
    panel.fillRoundRect(kBrightnessIncreaseLeft, kBrightnessControlTop,
                       kBrightnessIncreaseWidth, kBrightnessControlHeight, 4,
                       ILI9341_WHITE);
    panel.setTextSize(3);
    panel.setTextColor(ILI9341_BLACK);
    panel.setCursor(kBrightnessDecreaseLeft + 25, kBrightnessControlTop + 3);
    panel.print("-");
    panel.setCursor(kBrightnessIncreaseLeft + 25, kBrightnessControlTop + 3);
    panel.print("+");

    panel.setTextSize(1);
    panel.setTextColor(ILI9341_CYAN);
    panel.setCursor(106, kBrightnessControlTop + 1);
    panel.print(_brightnessConfigured ? "BRIGHT AT" : "BACKLIGHT");
    panel.setTextSize(2);
    panel.setTextColor(ILI9341_WHITE);
    panel.setCursor(126, kBrightnessControlTop + 12);
    panel.printf("%c=%u", _brightnessConfigured ? 'L' : 'B',
                _brightnessConfigured ? _brightnessConfig.brightThresholdRaw : _brightnessDuty);
}

bool Es3Graphics::handleBrightnessControlTouch(int screenX, int screenY)
{
    if (!_pages[_activePage].showBrightnessControl ||
        screenY < kBrightnessControlTop ||
        screenY >= kBrightnessControlTop + kBrightnessControlHeight) {
        return false;
    }

    constexpr uint16_t kThresholdStep = 10;
    const uint16_t maximum = _brightnessConfigured ? 4095 : 255;
    uint16_t threshold = _brightnessConfigured ? _brightnessConfig.brightThresholdRaw : _brightnessDuty;
    if (screenX >= kBrightnessDecreaseLeft &&
        screenX < kBrightnessDecreaseLeft + kBrightnessDecreaseWidth) {
        threshold = threshold >= kThresholdStep ? threshold - kThresholdStep : 0;
    } else if (screenX >= kBrightnessIncreaseLeft &&
               screenX < kBrightnessIncreaseLeft + kBrightnessIncreaseWidth) {
        threshold = threshold <= maximum - kThresholdStep ? threshold + kThresholdStep : maximum;
    } else {
        return false;
    }
    if (_brightnessConfigured) {
        return setBrightnessThresholdRaw(threshold);
    }
    _brightnessDuty = static_cast<uint8_t>(max<uint16_t>(threshold, 8));
    ledcWrite(kTftBacklight, _brightnessDuty);
    drawBrightnessControls();
    return true;
}

void Es3Graphics::printCentered(const char* text, int centerX, int y, uint16_t colour)
{
    const int width = static_cast<int>(std::strlen(text)) * 6;
    panel.setTextSize(1);
    panel.setTextColor(colour);
    panel.setCursor(centerX - width / 2, y);
    panel.print(text);
}

void Es3Graphics::drawTabs()
{
    for (uint8_t i = 0; i < _pageCount; ++i) {
        const int x = es3map::tabEdge(i, _pageCount);
        const int width = es3map::tabEdge(i + 1, _pageCount) - x;
        panel.fillRect(x, kTabTop, width, kTabHeight, ILI9341_WHITE);
        panel.drawRect(x, kTabTop, width, kTabHeight, ILI9341_DARKGREY);
        if (i == _activePage) {
            panel.drawFastHLine(x + 1, kTabTop + 1, width - 2, ILI9341_BLUE);
            panel.drawFastHLine(x + 1, kTabTop + 2, width - 2, ILI9341_BLUE);
        }
        printCentered(_pages[i].tab, x + width / 2, kTabTop + 4, ILI9341_BLACK);
    }
}

void Es3Graphics::drawField(const char* label, int y, uint16_t colour)
{
    panel.setTextSize(2);
    panel.setTextColor(colour);
    panel.setCursor(20, y);
    panel.print(label);
}

void Es3Graphics::drawPage()
{
    if (_activePage == es3map::kMapPage) { drawMapPage(); return; }
    if (_videoEver && _activePage == 0) {
        panel.fillScreen(ILI9341_BLACK);
        _videoStatus = 255;
        if (!videoCoversTabs() || _navigation) drawTabs();
        return;
    }
    const Es3DashboardPage& page = _pages[_activePage];
    panel.fillRect(0, 0, kScreenWidth, kTabTop, ILI9341_BLACK);
    if (page.showSdSplash) {
        bool splashDrawn = false;
        if (_storage.exists(kSplashRgb565File)) {
            splashDrawn = drawRgb565FromSd(kSplashRgb565File, 0, 0,
                                            kScreenWidth, kTabTop);
        } else if (_storage.exists(kSplashBmpFile)) {
            // Keep the existing asset usable while cards are migrated to raw.
            Serial0.println("[es3] legacy BMP splash; use /splash.rgb565 for faster loading");
            splashDrawn = drawBmp24FromSd(kSplashBmpFile, 0, 0,
                                           kScreenWidth, kTabTop);
        } else {
            // Missing assets fall back without unmounting active audio files.
            splashDrawn = drawRgb565FromSd(kSplashRgb565File, 0, 0,
                                            kScreenWidth, kTabTop);
        }
        if (!splashDrawn) {
            drawSplashFallback();
        }
        drawTabs();
        return;
    }
    panel.fillRect(0, 0, kScreenWidth, 30, ILI9341_DARKCYAN);
    panel.setTextSize(2);
    panel.setTextColor(ILI9341_WHITE);
    panel.setCursor(12, 8);
    panel.print(page.title);

    drawField(page.line1, 52, ILI9341_CYAN);
    drawField(page.line2, 86, ILI9341_WHITE);
    drawField(page.line3, 120, ILI9341_WHITE);
    drawField(page.line4, 154, ILI9341_WHITE);

    panel.setTextSize(1);
    panel.setTextColor(_storage.available() ? ILI9341_GREEN : ILI9341_RED);
    panel.setCursor(12, 202);
    if (_storage.available()) {
        panel.printf("SD: OK  %llu MiB", static_cast<unsigned long long>(_storage.capacityBytes() / (1024ULL * 1024ULL)));
    } else {
        panel.print("SD: NOT FOUND");
    }
    drawBrightnessStatus();
    panel.setTextColor(ILI9341_LIGHTGREY);
    panel.setCursor(12, 214);
    panel.print(_hostConnected ? "USB: Jetson online" : "USB: Jetson offline (data stale)");
    drawBrightnessControls();
    drawTabs();
}

void Es3Graphics::drawSplashFallback()
{
    panel.fillRect(0, 0, kScreenWidth, kTabTop, ILI9341_BLACK);
    panel.fillRect(0, 0, kScreenWidth, 34, ILI9341_DARKCYAN);
    panel.setTextColor(ILI9341_WHITE);
    panel.setTextSize(2);
    panel.setCursor(16, 10);
    panel.print("DASHCAM LIVE");
    panel.setTextColor(ILI9341_ORANGE);
    panel.setTextSize(2);
    panel.setCursor(20, 82);
    panel.print("SPLASH NOT FOUND");
    panel.setTextColor(ILI9341_LIGHTGREY);
    panel.setTextSize(1);
    panel.setCursor(28, 112);
    panel.print("Copy 320x224 RGB565 to:");
    panel.setCursor(108, 130);
    panel.print("/splash.rgb565");
    panel.setTextColor(_storage.available() ? ILI9341_GREEN : ILI9341_RED);
    panel.setCursor(20, 190);
    panel.print(_storage.available() ? "SD: READY" : "SD: NOT FOUND");
}

void Es3Graphics::handleDashboardTouch()
{
    TouchRaw raw{};
    if (!readTouchRaw(raw)) {
        _touchHeld = false;
        return;
    }
    if (_touchHeld || millis() - _lastTouchMs < kTouchDebounceMs) {
        return;
    }
    _touchHeld = true;
    _lastTouchMs = millis();

    int screenX = 0;
    int screenY = 0;
    if (!mapTouch(raw, screenX, screenY)) {
        return;
    }
    Serial0.printf("[es3] touch raw=%u,%u mapped=%d,%d\n", raw.x, raw.y, screenX, screenY);
    if (_videoVisible && videoCoversTabs() && !_navigation) {
        _navigation = true; _navigationUntil = millis() + 3000;
        drawTabs();
        return; // first tap reveals navigation; second tap selects a page
    }
    if (_activePage == es3map::kMapPage && screenY < kTabTop &&
        (_mapInfo || screenY >= es3map::kAttributionTop)) {
        _mapInfo = !_mapInfo;
        drawPage();
        return;
    }
    if (screenY < kTabTop) {
        (void)handleBrightnessControlTouch(screenX, screenY);
        return;
    }
    const int selectedPage = es3map::tabAt(screenX, _pageCount);
    if (selectedPage < 0 || selectedPage >= _pageCount) {
        return;
    }
    if (selectedPage == _activePage) {
        if (_pages[_activePage].allowTouchCalibration) {
            requestCalibration();
        }
        return;
    }
    _mapInfo = false;
    _activePage = static_cast<uint8_t>(selectedPage);
    drawPage();
}
