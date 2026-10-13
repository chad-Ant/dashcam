/** @file cydGraphics.cpp */

#include "cydGraphics.h"

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <cstring>
#include <cstdlib>

namespace {
constexpr int kTftMiso = 12;
constexpr int kTftMosi = 13;
constexpr int kTftSclk = 14;
constexpr int kTftCs = 15;
constexpr int kTftDc = 2;
constexpr int kTftRst = -1;  // panel reset is tied to EN; GPIO4 is the red LED
constexpr int kTftBacklight = 21;
constexpr uint32_t kBacklightPwmHz = 5000;
constexpr uint8_t kBacklightPwmBits = 8;

constexpr int kTouchIrq = 36;
constexpr int kTouchMosi = 32;
constexpr int kTouchMiso = 39;
constexpr int kTouchSclk = 25;
constexpr int kTouchCs = 33;

constexpr uint32_t kTouchDebounceMs = 180;
constexpr char kSplashRgb565File[] = "/splash.rgb565";
constexpr char kSplashBmpFile[] = "/splash.bmp";
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
SPIClass gTftSpi(HSPI);
Adafruit_ILI9341 gTft(&gTftSpi, kTftDc, kTftCs, kTftRst);

bool readLittleEndian16(File& file, uint16_t& value)
{
    uint8_t bytes[2]{};
    if (file.read(bytes, sizeof(bytes)) != sizeof(bytes)) {
        return false;
    }
    value = static_cast<uint16_t>(bytes[0]) |
            static_cast<uint16_t>(bytes[1]) << 8;
    return true;
}

bool readLittleEndian32(File& file, uint32_t& value)
{
    uint8_t bytes[4]{};
    if (file.read(bytes, sizeof(bytes)) != sizeof(bytes)) {
        return false;
    }
    value = static_cast<uint32_t>(bytes[0]) |
            static_cast<uint32_t>(bytes[1]) << 8 |
            static_cast<uint32_t>(bytes[2]) << 16 |
            static_cast<uint32_t>(bytes[3]) << 24;
    return true;
}
}  // namespace

CydGraphics::CydGraphics(CydStorage& storage)
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

bool CydGraphics::setPages(const CydDashboardPage* pages, uint8_t count)
{
    if (_started || pages == nullptr || count == 0 || count > kMaxPages) {
        return false;
    }
    for (uint8_t i = 0; i < count; ++i) {
        if (pages[i].tab == nullptr || pages[i].title == nullptr ||
            pages[i].line1 == nullptr || pages[i].line2 == nullptr ||
            pages[i].line3 == nullptr || pages[i].line4 == nullptr) {
            return false;
        }
    }
    _pages = pages;
    _pageCount = count;
    return true;
}

bool CydGraphics::brightnessConfigIsSane(const CydBrightnessConfig& config) const
{
    if (!config.enabled) {
        return true;
    }
    return config.ldrPin >= 0 && config.ldrPin <= 39 &&
           config.darkRaw != config.brightRaw &&
           config.minDuty <= config.maxDuty &&
           config.brightThresholdRaw <= 4095 &&
           config.transitionStep > 0 &&
           config.updateIntervalMs >= 50;
}

bool CydGraphics::setBrightnessConfig(const CydBrightnessConfig& config)
{
    if (_started || !brightnessConfigIsSane(config)) {
        return false;
    }
    _brightnessConfig = config;
    _brightnessConfigured = config.enabled;
    return true;
}

bool CydGraphics::setBrightnessThresholdRaw(uint16_t raw)
{
    if (!_brightnessConfigured || raw > 4095) {
        return false;
    }
    _brightnessConfig.brightThresholdRaw = raw;
    Serial.printf("[cyd] brightness threshold L=%u\n", raw);
    if (_started && _pages[_activePage].showBrightnessControl) {
        drawBrightnessControls();
    }
    return true;
}

bool CydGraphics::drawBmp24FromSd(const char* path, int x, int y,
                                   uint16_t width, uint16_t height)
{
    if (!_started || path == nullptr || x < 0 || y < 0 ||
        x + width > kScreenWidth || y + height > kTabTop) {
        return false;
    }

    File file;
    if (!_storage.openRead(path, file)) {
        Serial.printf("[cyd] splash open failed: %s\n", path);
        return false;
    }
    const uint32_t startedUs = micros();

    uint16_t signature = 0;
    uint32_t fileSize = 0;
    uint16_t reserved = 0;
    uint32_t pixelOffset = 0;
    uint32_t dibSize = 0;
    uint32_t bitmapWidth = 0;
    uint32_t bitmapHeightBits = 0;
    uint16_t planes = 0;
    uint16_t bitsPerPixel = 0;
    uint32_t compression = 0;
    const bool headerOk = readLittleEndian16(file, signature) &&
                          readLittleEndian32(file, fileSize) &&
                          readLittleEndian16(file, reserved) &&
                          readLittleEndian16(file, reserved) &&
                          readLittleEndian32(file, pixelOffset) &&
                          readLittleEndian32(file, dibSize) &&
                          readLittleEndian32(file, bitmapWidth) &&
                          readLittleEndian32(file, bitmapHeightBits) &&
                          readLittleEndian16(file, planes) &&
                          readLittleEndian16(file, bitsPerPixel) &&
                          readLittleEndian32(file, compression);
    const int32_t bitmapHeight = static_cast<int32_t>(bitmapHeightBits);
    const uint32_t absoluteHeight = bitmapHeight < 0
                                        ? static_cast<uint32_t>(-bitmapHeight)
                                        : static_cast<uint32_t>(bitmapHeight);
    if (!headerOk || signature != 0x4D42 || dibSize < 40 ||
        bitmapWidth != width || absoluteHeight != height || planes != 1 ||
        bitsPerPixel != 24 || compression != 0) {
        Serial.printf("[cyd] splash format error: sig=%04X %lux%ld bpp=%u compression=%lu\n",
                      signature, static_cast<unsigned long>(bitmapWidth),
                      static_cast<long>(bitmapHeight), bitsPerPixel,
                      static_cast<unsigned long>(compression));
        file.close();
        return false;
    }

    const uint32_t rowBytes = (static_cast<uint32_t>(width) * 3U + 3U) & ~3U;
    const uint32_t pixelEnd = pixelOffset + rowBytes * height;
    // The BMP file-size field is optional in practice; trust the filesystem
    // size, which makes the loader work with common exporters that leave it 0.
    if (pixelEnd > file.size()) {
        Serial.printf("[cyd] splash truncated: offset=%lu need=%lu actual=%lu header=%lu\n",
                      static_cast<unsigned long>(pixelOffset),
                      static_cast<unsigned long>(pixelEnd),
                      static_cast<unsigned long>(file.size()),
                      static_cast<unsigned long>(fileSize));
        file.close();
        return false;
    }

    uint8_t bgr[3 * kScreenWidth]{};
    uint16_t rgb565[kScreenWidth]{};
    for (uint16_t sourceRow = 0; sourceRow < height; ++sourceRow) {
        const uint32_t offset = pixelOffset + static_cast<uint32_t>(sourceRow) * rowBytes;
        if (!file.seek(offset) || file.read(bgr, rowBytes) != rowBytes) {
            Serial.printf("[cyd] splash read error at row %u\n", sourceRow);
            file.close();
            return false;
        }
        for (uint16_t column = 0; column < width; ++column) {
            rgb565[column] = gTft.color565(bgr[column * 3 + 2],
                                             bgr[column * 3 + 1],
                                             bgr[column * 3]);
        }
        const int destinationRow = bitmapHeight > 0
                                       ? y + height - 1 - sourceRow  // bottom-up BMP
                                       : y + sourceRow;              // top-down BMP
        gTft.drawRGBBitmap(x, destinationRow, rgb565, width, 1);
    }
    file.close();
    const uint32_t elapsedUs = micros() - startedUs;
    Serial.printf("[cyd] BMP splash rendered: %s (%lu bytes, %lu ms, SD %lu MHz)\n",
                  path, static_cast<unsigned long>(pixelEnd),
                  static_cast<unsigned long>((elapsedUs + 500U) / 1000U),
                  static_cast<unsigned long>(_storage.spiFrequencyHz() / 1000000U));
    return true;
}

bool CydGraphics::drawRgb565FromSd(const char* path, int x, int y,
                                    uint16_t width, uint16_t height)
{
    if (!_started || path == nullptr || x < 0 || y < 0 ||
        x + width > kScreenWidth || y + height > kTabTop) {
        return false;
    }

    File file;
    if (!_storage.openRead(path, file)) {
        Serial.printf("[cyd] RGB565 splash open failed: %s\n", path);
        return false;
    }

    const uint32_t expectedBytes = static_cast<uint32_t>(width) * height * 2U;
    if (file.size() != expectedBytes) {
        Serial.printf("[cyd] RGB565 splash size error: %s expected=%lu actual=%lu\n",
                      path, static_cast<unsigned long>(expectedBytes),
                      static_cast<unsigned long>(file.size()));
        file.close();
        return false;
    }

    // The asset is explicitly big-endian (MSB first) RGB565, identical to
    // ILI9341 SPI wire order. Keeping it this way avoids a per-pixel byte swap.
    constexpr uint16_t kPixelsPerBlock = 320;
    uint16_t pixels[kPixelsPerBlock];
    const uint32_t startedUs = micros();
    uint32_t pixelsRemaining = static_cast<uint32_t>(width) * height;

    gTft.startWrite();
    gTft.setAddrWindow(x, y, width, height);
    while (pixelsRemaining > 0) {
        const uint16_t pixelsThisBlock = static_cast<uint16_t>(min<uint32_t>(
            pixelsRemaining, kPixelsPerBlock));
        const size_t byteCount = static_cast<size_t>(pixelsThisBlock) * 2U;
        if (file.read(reinterpret_cast<uint8_t*>(pixels), byteCount) != byteCount) {
            gTft.endWrite();
            file.close();
            Serial.printf("[cyd] RGB565 splash read error: %s\n", path);
            return false;
        }
        gTft.writePixels(pixels, pixelsThisBlock, true, true);
        pixelsRemaining -= pixelsThisBlock;
    }
    gTft.endWrite();
    file.close();

    const uint32_t elapsedUs = micros() - startedUs;
    Serial.printf("[cyd] RGB565 splash rendered: %s (%lu bytes, %lu ms, SD %lu MHz)\n",
                  path, static_cast<unsigned long>(expectedBytes),
                  static_cast<unsigned long>((elapsedUs + 500U) / 1000U),
                  static_cast<unsigned long>(_storage.spiFrequencyHz() / 1000000U));
    return true;
}

bool CydGraphics::begin()
{
    if (_started || _pages == nullptr || _pageCount == 0) {
        return false;
    }

    pinMode(kTftBacklight, OUTPUT);
    if (_brightnessConfigured) {
        analogReadResolution(12);
        // The CYD's CdS cell is behind a very high-resistance divider. 0 dB
        // attenuation gives its small voltage change useful ADC resolution;
        // 11 dB makes typical indoor readings occupy too little of the range.
        analogSetPinAttenuation(_brightnessConfig.ldrPin, ADC_0db);
        _brightnessPwmAttached = ledcAttach(kTftBacklight, kBacklightPwmHz, kBacklightPwmBits);
        if (!_brightnessPwmAttached) {
            return false;
        }
        updateBrightness(true);
    } else {
        digitalWrite(kTftBacklight, HIGH);
    }
    gTftSpi.begin(kTftSclk, kTftMiso, kTftMosi, kTftCs);
    gTft.begin();
    gTft.setRotation(1);

    pinMode(kTouchCs, OUTPUT);
    digitalWrite(kTouchCs, HIGH);
    pinMode(kTouchSclk, OUTPUT);
    digitalWrite(kTouchSclk, LOW);
    pinMode(kTouchMosi, OUTPUT);
    digitalWrite(kTouchMosi, LOW);
    pinMode(kTouchMiso, INPUT);
    // GPIO36 is input-only and has no internal pull-up on classic ESP32.
    // The XPT2046 IRQ line is already pulled high by the CYD touch circuit.
    pinMode(kTouchIrq, INPUT);

    _calibrationValid = _storage.loadTouchCalibration(_calibration);
    _started = true;
    Serial.printf("[cyd] graphics ready; SD %s; calibration %s\n",
                  _storage.available() ? "OK" : "not found",
                  _calibrationValid ? "loaded" : "needed");
    if (_calibrationValid) {
        drawPage();
    } else {
        startCalibration();
    }
    return true;
}

void CydGraphics::update()
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
    updateBrightness(false);
}

void CydGraphics::requestCalibration()
{
    if (!_started) {
        return;
    }
    startCalibration();
}

bool CydGraphics::sdAvailable() const { return _storage.available(); }
bool CydGraphics::calibrationLoaded() const { return _calibrationValid; }
uint8_t CydGraphics::activePage() const { return _activePage; }
uint8_t CydGraphics::brightnessDuty() const { return _brightnessDuty; }
uint16_t CydGraphics::brightnessThresholdRaw() const
{
    return _brightnessConfig.brightThresholdRaw;
}
int CydGraphics::filteredLightRaw() const { return _filteredLightRaw; }

void CydGraphics::updateBrightness(bool force)
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
        Serial.printf("[cyd] LDR raw=%d, backlight=%u->%u/255\n",
                      _filteredLightRaw, _brightnessDuty, _brightnessTargetDuty);
    }
}

void CydGraphics::drawBrightnessStatus()
{
    if (!_brightnessConfigured) {
        return;
    }
    // This diagnostic remains on every dashboard page. It makes it possible
    // to distinguish an unresponsive sensor from an unresponsive backlight
    // without a serial monitor: L is the filtered LDR ADC value and B is PWM.
    gTft.fillRect(184, 198, 132, 13, ILI9341_BLACK);
    gTft.setTextSize(1);
    gTft.setTextColor(ILI9341_ORANGE);
    gTft.setCursor(184, 202);
    gTft.printf("L:%4d B:%3u", _filteredLightRaw, _brightnessDuty);
}

void CydGraphics::drawBrightnessControls()
{
    if (!_brightnessConfigured || !_pages[_activePage].showBrightnessControl) {
        return;
    }

    gTft.fillRect(0, kBrightnessControlTop - 2, kScreenWidth,
                  kBrightnessControlHeight + 4, ILI9341_BLACK);
    gTft.fillRoundRect(kBrightnessDecreaseLeft, kBrightnessControlTop,
                       kBrightnessDecreaseWidth, kBrightnessControlHeight, 4,
                       ILI9341_WHITE);
    gTft.fillRoundRect(kBrightnessIncreaseLeft, kBrightnessControlTop,
                       kBrightnessIncreaseWidth, kBrightnessControlHeight, 4,
                       ILI9341_WHITE);
    gTft.setTextSize(3);
    gTft.setTextColor(ILI9341_BLACK);
    gTft.setCursor(kBrightnessDecreaseLeft + 25, kBrightnessControlTop + 3);
    gTft.print("-");
    gTft.setCursor(kBrightnessIncreaseLeft + 25, kBrightnessControlTop + 3);
    gTft.print("+");

    gTft.setTextSize(1);
    gTft.setTextColor(ILI9341_CYAN);
    gTft.setCursor(106, kBrightnessControlTop + 1);
    gTft.print("BRIGHT AT");
    gTft.setTextSize(2);
    gTft.setTextColor(ILI9341_WHITE);
    gTft.setCursor(126, kBrightnessControlTop + 12);
    gTft.printf("L=%u", _brightnessConfig.brightThresholdRaw);
}

bool CydGraphics::handleBrightnessControlTouch(int screenX, int screenY)
{
    if (!_brightnessConfigured || !_pages[_activePage].showBrightnessControl ||
        screenY < kBrightnessControlTop ||
        screenY >= kBrightnessControlTop + kBrightnessControlHeight) {
        return false;
    }

    constexpr uint16_t kThresholdStep = 10;
    uint16_t threshold = _brightnessConfig.brightThresholdRaw;
    if (screenX >= kBrightnessDecreaseLeft &&
        screenX < kBrightnessDecreaseLeft + kBrightnessDecreaseWidth) {
        threshold = threshold >= kThresholdStep ? threshold - kThresholdStep : 0;
    } else if (screenX >= kBrightnessIncreaseLeft &&
               screenX < kBrightnessIncreaseLeft + kBrightnessIncreaseWidth) {
        threshold = threshold <= 4095 - kThresholdStep ? threshold + kThresholdStep : 4095;
    } else {
        return false;
    }
    return setBrightnessThresholdRaw(threshold);
}

void CydGraphics::printCentered(const char* text, int centerX, int y, uint16_t colour)
{
    const int width = static_cast<int>(std::strlen(text)) * 6;
    gTft.setTextSize(1);
    gTft.setTextColor(colour);
    gTft.setCursor(centerX - width / 2, y);
    gTft.print(text);
}

void CydGraphics::drawTabs()
{
    for (uint8_t i = 0; i < _pageCount; ++i) {
        const int x = static_cast<int>(i) * kTabWidth;
        gTft.fillRect(x, kTabTop, kTabWidth, kTabHeight, ILI9341_WHITE);
        gTft.drawRect(x, kTabTop, kTabWidth, kTabHeight, ILI9341_DARKGREY);
        if (i == _activePage) {
            gTft.drawFastHLine(x + 1, kTabTop + 1, kTabWidth - 2, ILI9341_BLUE);
            gTft.drawFastHLine(x + 1, kTabTop + 2, kTabWidth - 2, ILI9341_BLUE);
        }
        printCentered(_pages[i].tab, x + kTabWidth / 2, kTabTop + 4, ILI9341_BLACK);
    }
}

void CydGraphics::drawField(const char* label, int y, uint16_t colour)
{
    gTft.setTextSize(2);
    gTft.setTextColor(colour);
    gTft.setCursor(20, y);
    gTft.print(label);
}

void CydGraphics::drawPage()
{
    const CydDashboardPage& page = _pages[_activePage];
    gTft.fillRect(0, 0, kScreenWidth, kTabTop, ILI9341_BLACK);
    if (page.showSdSplash) {
        bool splashDrawn = false;
        if (_storage.exists(kSplashRgb565File)) {
            splashDrawn = drawRgb565FromSd(kSplashRgb565File, 0, 0,
                                            kScreenWidth, kTabTop);
        } else if (_storage.exists(kSplashBmpFile)) {
            // Keep the existing asset usable while cards are migrated to raw.
            Serial.println("[cyd] legacy BMP splash; use /splash.rgb565 for faster loading");
            splashDrawn = drawBmp24FromSd(kSplashBmpFile, 0, 0,
                                           kScreenWidth, kTabTop);
        } else {
            // openRead remounts once, so hot-swapped cards are still detected.
            splashDrawn = drawRgb565FromSd(kSplashRgb565File, 0, 0,
                                            kScreenWidth, kTabTop);
        }
        if (!splashDrawn) {
            drawSplashFallback();
        }
        drawTabs();
        return;
    }
    gTft.fillRect(0, 0, kScreenWidth, 30, ILI9341_DARKCYAN);
    gTft.setTextSize(2);
    gTft.setTextColor(ILI9341_WHITE);
    gTft.setCursor(12, 8);
    gTft.print(page.title);

    drawField(page.line1, 52, ILI9341_CYAN);
    drawField(page.line2, 86, ILI9341_WHITE);
    drawField(page.line3, 120, ILI9341_WHITE);
    drawField(page.line4, 154, ILI9341_WHITE);

    gTft.setTextSize(1);
    gTft.setTextColor(_storage.available() ? ILI9341_GREEN : ILI9341_RED);
    gTft.setCursor(12, 202);
    if (_storage.available()) {
        gTft.printf("SD: OK  %llu MiB", static_cast<unsigned long long>(_storage.capacityBytes() / (1024ULL * 1024ULL)));
    } else {
        gTft.print("SD: NOT FOUND");
    }
    drawBrightnessStatus();
    gTft.setTextColor(ILI9341_LIGHTGREY);
    gTft.setCursor(12, 214);
    gTft.print("Touch a tab to switch view");
    drawBrightnessControls();
    drawTabs();
}

void CydGraphics::drawSplashFallback()
{
    gTft.fillRect(0, 0, kScreenWidth, kTabTop, ILI9341_BLACK);
    gTft.fillRect(0, 0, kScreenWidth, 34, ILI9341_DARKCYAN);
    gTft.setTextColor(ILI9341_WHITE);
    gTft.setTextSize(2);
    gTft.setCursor(16, 10);
    gTft.print("DASHCAM LIVE");
    gTft.setTextColor(ILI9341_ORANGE);
    gTft.setTextSize(2);
    gTft.setCursor(20, 82);
    gTft.print("SPLASH NOT FOUND");
    gTft.setTextColor(ILI9341_LIGHTGREY);
    gTft.setTextSize(1);
    gTft.setCursor(28, 112);
    gTft.print("Copy 320x224 RGB565 to:");
    gTft.setCursor(108, 130);
    gTft.print("/splash.rgb565");
    gTft.setTextColor(_storage.available() ? ILI9341_GREEN : ILI9341_RED);
    gTft.setCursor(20, 190);
    gTft.print(_storage.available() ? "SD: READY" : "SD: NOT FOUND");
}

uint8_t CydGraphics::touchTransfer(uint8_t out)
{
    uint8_t in = 0;
    for (int bit = 0; bit < 8; ++bit) {
        digitalWrite(kTouchSclk, LOW);
        digitalWrite(kTouchMosi, (out & 0x80U) != 0U ? HIGH : LOW);
        out <<= 1;
        delayMicroseconds(1);
        digitalWrite(kTouchSclk, HIGH);
        in = static_cast<uint8_t>((in << 1) | (digitalRead(kTouchMiso) == HIGH ? 1U : 0U));
        delayMicroseconds(1);
    }
    return in;
}

uint16_t CydGraphics::touchReadAxis(uint8_t command)
{
    digitalWrite(kTouchCs, LOW);
    touchTransfer(command);
    const uint16_t value = static_cast<uint16_t>(touchTransfer(0)) << 8 | touchTransfer(0);
    digitalWrite(kTouchCs, HIGH);
    return value >> 3;
}

bool CydGraphics::readTouchRaw(TouchRaw& raw)
{
    if (digitalRead(kTouchIrq) != LOW) {
        return false;
    }
    uint32_t sumX = 0;
    uint32_t sumY = 0;
    for (int sample = 0; sample < 7; ++sample) {
        sumX += touchReadAxis(0xD0);
        sumY += touchReadAxis(0x90);
    }
    raw.x = static_cast<uint16_t>(sumX / 7U);
    raw.y = static_cast<uint16_t>(sumY / 7U);
    return true;
}

int CydGraphics::mapCalibrationAxis(int raw, int atLowTarget, int atMiddleTarget,
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

int CydGraphics::mapCalibrationAxisFourPoint(int raw, int atLowTarget,
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

bool CydGraphics::mapTouch(const TouchRaw& raw, int& screenX, int& screenY) const
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

void CydGraphics::drawCalibrationTarget()
{
    gTft.fillScreen(ILI9341_BLACK);
    gTft.setTextSize(2);
    gTft.setTextColor(ILI9341_WHITE);
    gTft.setCursor(18, 146);
    gTft.print("TOUCH CALIBRATION");
    gTft.setTextSize(1);
    gTft.setCursor(62, 170);
    gTft.printf("Tap target %u of %u", _calibrationTarget + 1, kCalibrationPointCount);
    gTft.setCursor(46, 184);
    gTft.print("Use the center of each cross");
    gTft.setCursor(46, 198);
    gTft.print("Lift your finger between taps");

    const int x = kCalibrationTargetX[_calibrationTarget];
    const int y = kCalibrationTargetY[_calibrationTarget];
    gTft.drawFastHLine(x - 12, y, 25, ILI9341_YELLOW);
    gTft.drawFastVLine(x, y - 12, 25, ILI9341_YELLOW);
    gTft.drawCircle(x, y, 7, ILI9341_YELLOW);
}

bool CydGraphics::buildCalibration()
{
    const int rawXAcross = std::abs(static_cast<int>(_calibrationSamples[3].x) - _calibrationSamples[0].x) +
                            std::abs(static_cast<int>(_calibrationSamples[7].x) - _calibrationSamples[4].x) +
                            std::abs(static_cast<int>(_calibrationSamples[11].x) - _calibrationSamples[8].x);
    const int rawYAcross = std::abs(static_cast<int>(_calibrationSamples[3].y) - _calibrationSamples[0].y) +
                            std::abs(static_cast<int>(_calibrationSamples[7].y) - _calibrationSamples[4].y) +
                            std::abs(static_cast<int>(_calibrationSamples[11].y) - _calibrationSamples[8].y);
    CydTouchCalibrationRecord calibration{};
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
    if (std::abs(calibration.xAtRight - calibration.xAtLeft) <= 600 ||
        std::abs(calibration.yAtBottom - calibration.yAtTop) <= 600) {
        return false;
    }
    _calibration = calibration;
    return true;
}

void CydGraphics::startCalibration()
{
    _calibrationValid = false;
    _calibrating = true;
    _calibrationTesting = false;
    _calibrationTouchHeld = false;
    _calibrationCompletePending = false;
    _calibrationTarget = 0;
    drawCalibrationTarget();
}

void CydGraphics::finishCalibration()
{
    _calibrating = false;
    _calibrationValid = buildCalibration();
    if (!_calibrationValid) {
        startCalibration();
        return;
    }
    startCalibrationTest();
}

void CydGraphics::handleCalibration()
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
    Serial.printf("[cyd] calibration %u raw=%u,%u\n", _calibrationTarget + 1, raw.x, raw.y);
    ++_calibrationTarget;
    if (_calibrationTarget == kCalibrationPointCount) {
        _calibrationCompletePending = true;
        return;
    }
    drawCalibrationTarget();
}

void CydGraphics::startCalibrationTest()
{
    _calibrationTesting = true;
    _calibrationTestTouchHeld = false;
    drawCalibrationTest();
}

void CydGraphics::drawCalibrationTest()
{
    gTft.fillScreen(ILI9341_BLACK);
    gTft.setTextSize(2);
    gTft.setTextColor(ILI9341_WHITE);
    gTft.setCursor(28, 10);
    gTft.print("TOUCH CHECK");
    gTft.setTextSize(1);
    gTft.setTextColor(ILI9341_LIGHTGREY);
    gTft.setCursor(32, 34);
    gTft.print("Tap the targets: dot follows touch");

    gTft.fillRoundRect(14, 52, 130, 28, 4, ILI9341_ORANGE);
    gTft.fillRoundRect(176, 52, 130, 28, 4, ILI9341_GREEN);
    printCentered("RETRY", 79, 62, ILI9341_BLACK);
    printCentered("SAVE", 241, 62, ILI9341_BLACK);

    gTft.setTextColor(ILI9341_CYAN);
    gTft.setCursor(82, 98);
    gTft.print("Brightness-control targets");
    gTft.drawRoundRect(kBrightnessDecreaseLeft, kBrightnessControlTop,
                       kBrightnessDecreaseWidth, kBrightnessControlHeight, 4,
                       ILI9341_WHITE);
    gTft.drawRoundRect(kBrightnessIncreaseLeft, kBrightnessControlTop,
                       kBrightnessIncreaseWidth, kBrightnessControlHeight, 4,
                       ILI9341_WHITE);
    printCentered("-", kBrightnessDecreaseLeft + kBrightnessDecreaseWidth / 2,
                  kBrightnessControlTop + 8, ILI9341_WHITE);
    printCentered("+", kBrightnessIncreaseLeft + kBrightnessIncreaseWidth / 2,
                  kBrightnessControlTop + 8, ILI9341_WHITE);

    gTft.setTextColor(ILI9341_CYAN);
    gTft.setCursor(110, 204);
    gTft.print("Tab targets");
    for (int tab = 0; tab < kTabCount; ++tab) {
        const int x = tab * kTabWidth;
        gTft.drawRect(x, kTabTop, kTabWidth, kTabHeight, ILI9341_WHITE);
        gTft.drawFastVLine(x + kTabWidth / 2, kTabTop + 3, 10, ILI9341_YELLOW);
    }
}

void CydGraphics::saveCalibration()
{
    _calibrationTesting = false;
    const bool saved = _storage.saveTouchCalibration(_calibration);
    Serial.printf("[cyd] touch calibration verified; SD save %s\n", saved ? "OK" : "failed");
    drawPage();
}

void CydGraphics::handleCalibrationTest()
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
    Serial.printf("[cyd] touch check raw=%u,%u mapped=%d,%d\n",
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
    gTft.fillCircle(screenX, screenY, 4, ILI9341_CYAN);
    gTft.drawCircle(screenX, screenY, 7, ILI9341_WHITE);
}

void CydGraphics::handleDashboardTouch()
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
    Serial.printf("[cyd] touch raw=%u,%u mapped=%d,%d\n", raw.x, raw.y, screenX, screenY);
    if (screenY < kTabTop) {
        (void)handleBrightnessControlTouch(screenX, screenY);
        return;
    }
    const int selectedPage = screenX / kTabWidth;
    if (selectedPage < 0 || selectedPage >= _pageCount) {
        return;
    }
    if (selectedPage == _activePage) {
        if (_pages[_activePage].allowTouchCalibration) {
            requestCalibration();
        }
        return;
    }
    _activePage = static_cast<uint8_t>(selectedPage);
    drawPage();
}
