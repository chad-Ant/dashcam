/** @file es3Graphics.cpp */

#include "es3Graphics.h"

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
SPIClass gTftSpi(FSPI);
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
    // Own the text so USB updates never retain pointers into an RX buffer.
    for (uint8_t i = 0; i < count; ++i) {
        const char* fields[] = {pages[i].title, pages[i].line1, pages[i].line2,
                                pages[i].line3, pages[i].line4};
        for (const char* field : fields) {
            if (std::strlen(field) >= sizeof(_pageText[0][0])) return false;
        }
    }
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

bool Es3Graphics::drawBmp24FromSd(const char* path, int x, int y,
                                   uint16_t width, uint16_t height)
{
    if (!_started || path == nullptr || x < 0 || y < 0 ||
        x + width > kScreenWidth || y + height > kTabTop) {
        return false;
    }

    File file;
    if (!_storage.openRead(path, file)) {
        Serial0.printf("[es3] splash open failed: %s\n", path);
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
        Serial0.printf("[es3] splash format error: sig=%04X %lux%ld bpp=%u compression=%lu\n",
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
        Serial0.printf("[es3] splash truncated: offset=%lu need=%lu actual=%lu header=%lu\n",
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
            Serial0.printf("[es3] splash read error at row %u\n", sourceRow);
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
    Serial0.printf("[es3] BMP splash rendered: %s (%lu bytes, %lu ms, SD %lu MHz)\n",
                  path, static_cast<unsigned long>(pixelEnd),
                  static_cast<unsigned long>((elapsedUs + 500U) / 1000U),
                  static_cast<unsigned long>(_storage.busFrequencyHz() / 1000000U));
    return true;
}

bool Es3Graphics::drawRgb565FromSd(const char* path, int x, int y,
                                    uint16_t width, uint16_t height)
{
    if (!_started || path == nullptr || x < 0 || y < 0 ||
        x + width > kScreenWidth || y + height > kTabTop) {
        return false;
    }

    File file;
    if (!_storage.openRead(path, file)) {
        Serial0.printf("[es3] RGB565 splash open failed: %s\n", path);
        return false;
    }

    const uint32_t expectedBytes = static_cast<uint32_t>(width) * height * 2U;
    if (file.size() != expectedBytes) {
        Serial0.printf("[es3] RGB565 splash size error: %s expected=%lu actual=%lu\n",
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
            Serial0.printf("[es3] RGB565 splash read error: %s\n", path);
            return false;
        }
        gTft.writePixels(pixels, pixelsThisBlock, true, true);
        pixelsRemaining -= pixelsThisBlock;
    }
    gTft.endWrite();
    file.close();

    const uint32_t elapsedUs = micros() - startedUs;
    Serial0.printf("[es3] RGB565 splash rendered: %s (%lu bytes, %lu ms, SD %lu MHz)\n",
                  path, static_cast<unsigned long>(expectedBytes),
                  static_cast<unsigned long>((elapsedUs + 500U) / 1000U),
                  static_cast<unsigned long>(_storage.busFrequencyHz() / 1000000U));
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
    gTft.begin();
    es3ConfigureIpsPanel(gTft);
    gTft.setRotation(1);

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
    gTft.fillRect(184, 198, 132, 13, ILI9341_BLACK);
    gTft.setTextSize(1);
    gTft.setTextColor(ILI9341_ORANGE);
    gTft.setCursor(184, 202);
    gTft.printf("L:%4d B:%3u", _filteredLightRaw, _brightnessDuty);
}

void Es3Graphics::drawBrightnessControls()
{
    if (!_pages[_activePage].showBrightnessControl) {
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
    gTft.print(_brightnessConfigured ? "BRIGHT AT" : "BACKLIGHT");
    gTft.setTextSize(2);
    gTft.setTextColor(ILI9341_WHITE);
    gTft.setCursor(126, kBrightnessControlTop + 12);
    gTft.printf("%c=%u", _brightnessConfigured ? 'L' : 'B',
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
    gTft.setTextSize(1);
    gTft.setTextColor(colour);
    gTft.setCursor(centerX - width / 2, y);
    gTft.print(text);
}

void Es3Graphics::drawTabs()
{
    for (uint8_t i = 0; i < _pageCount; ++i) {
        const int x = es3map::tabEdge(i, _pageCount);
        const int width = es3map::tabEdge(i + 1, _pageCount) - x;
        gTft.fillRect(x, kTabTop, width, kTabHeight, ILI9341_WHITE);
        gTft.drawRect(x, kTabTop, width, kTabHeight, ILI9341_DARKGREY);
        if (i == _activePage) {
            gTft.drawFastHLine(x + 1, kTabTop + 1, width - 2, ILI9341_BLUE);
            gTft.drawFastHLine(x + 1, kTabTop + 2, width - 2, ILI9341_BLUE);
        }
        printCentered(_pages[i].tab, x + width / 2, kTabTop + 4, ILI9341_BLACK);
    }
}

void Es3Graphics::drawField(const char* label, int y, uint16_t colour)
{
    gTft.setTextSize(2);
    gTft.setTextColor(colour);
    gTft.setCursor(20, y);
    gTft.print(label);
}

void Es3Graphics::drawPage()
{
    if (_activePage == es3map::kMapPage) { drawMapPage(); return; }
    if (_videoEver && _activePage == 0) {
        gTft.fillScreen(ILI9341_BLACK);
        _videoStatus = 255;
        if (!videoCoversTabs() || _navigation) drawTabs();
        return;
    }
    const Es3DashboardPage& page = _pages[_activePage];
    gTft.fillRect(0, 0, kScreenWidth, kTabTop, ILI9341_BLACK);
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
    gTft.print(_hostConnected ? "USB: Jetson online" : "USB: Jetson offline (data stale)");
    drawBrightnessControls();
    drawTabs();
}

void Es3Graphics::drawSplashFallback()
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
    for (uint8_t tab = 0; tab < _pageCount; ++tab) {
        const int x = es3map::tabEdge(tab, _pageCount);
        const int width = es3map::tabEdge(tab + 1, _pageCount) - x;
        gTft.drawRect(x, kTabTop, width, kTabHeight, ILI9341_WHITE);
        gTft.drawFastVLine(x + width / 2, kTabTop + 3, 10, ILI9341_YELLOW);
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
    gTft.fillCircle(screenX, screenY, 4, ILI9341_CYAN);
    gTft.drawCircle(screenX, screenY, 7, ILI9341_WHITE);
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


bool Es3Graphics::videoCoversTabs() const
{
    return uint32_t(_videoConfig.viewportY) + _videoConfig.viewportH > kTabTop;
}

void Es3Graphics::drawMapPage()
{
    gTft.fillRect(0, 0, 320, es3map::kTabTop, ILI9341_BLACK);
    gTft.setTextSize(1); gTft.setTextColor(ILI9341_WHITE);
    gTft.setCursor(2, 4);
    gTft.print(_video && !_video->available() ? "Map unavailable - image memory" :
        (_hostConnected ? "Waiting for map" : "JETSON OFFLINE"));
    if (_mapInfo) {
        printCentered("MAP INFORMATION", 160, 40, ILI9341_CYAN);
        printCentered("Map data: OpenStreetMap contributors", 160, 72, ILI9341_WHITE);
        printCentered("Open Database License (ODbL)", 160, 96, ILI9341_WHITE);
        printCentered("openstreetmap.org/copyright", 160, 120, ILI9341_WHITE);
        printCentered("Tap to return to map", 160, 164, ILI9341_CYAN);
    }
    // Built-in fallback credit; actual streamed frames include host/provider
    // attribution. Never overwrite that strip when marking a retained map stale.
    static const uint8_t copyrightGlyph[] = {0x3c, 0x42, 0x99, 0xa1, 0xa1, 0x99, 0x42, 0x3c};
    gTft.drawBitmap(2, 212, copyrightGlyph, 8, 8, ILI9341_WHITE);
    gTft.setCursor(12, 212);
    gTft.print("OpenStreetMap contributors");
    drawTabs();
}

void Es3Graphics::updateVideo()
{
    if (!_video) return;
    const uint32_t now = millis();
    const bool blocked = _calibrating || _calibrationTesting || _mapInfo;
    es3video::StartVideoRequest config;
    es3map::Binding binding;
    const uint32_t token = _video->view(_activePage, blocked, config, binding);
    const bool map = binding.surface == es3map::Surface::Map;
    if (token && !map) _videoEver = true;
    const bool visible = !blocked && _activePage == binding.page() &&
        (token || (_videoVisible && binding.surface == _binding.surface));
    if (token != _videoToken || visible != _videoVisible || binding.surface != _binding.surface) {
        _video->release(_videoImage, false, 0);
        const bool keepLastFrame = !token && visible && _videoVisible;
        _videoToken = token; _videoVisible = visible;
        _videoConfig = config; _binding = binding;
        _videoStatus = 255; _navigation = false;
        if (!keepLastFrame) {
            _lastFrameMs = now;
            if (!_calibrating && !_calibrationTesting) drawPage();
        }
    }
    if (!visible) return;
    if (_navigation && int32_t(now - _navigationUntil) >= 0) {
        _navigation = false;
        gTft.fillRect(0, kTabTop, kScreenWidth, kTabHeight, ILI9341_BLACK);
    }
    if (_videoImage.slot < 0 && _video->acquire(_videoImage)) {
        if (_videoImage.token != _videoToken) { _video->release(_videoImage, false, 0); return; }
        _videoRow = 0; _blitStarted = micros();
    }
    if (_videoImage.slot >= 0) {
        const int width = config.viewportW;
        const int rows = min<int>(8, config.viewportH - _videoRow);
        memcpy(_videoStrip, _videoImage.pixels + _videoRow * width, width * rows * 2);
        // One bounded SPI transaction; touch and controls run between strips.
        gTft.drawRGBBitmap(config.viewportX, config.viewportY + _videoRow, _videoStrip, width, rows);
        _videoRow += rows;
        if (_navigation && config.viewportY + _videoRow > kTabTop) drawTabs();
        if (_videoRow == config.viewportH) {
            _video->release(_videoImage, true, micros() - _blitStarted);
            _lastFrameMs = now; _videoStatus = 255;
        }
    }
    const uint32_t age = now - _lastFrameMs;
    const uint8_t status = age >= config.placeholderAfterMs ? 2 :
        (age >= config.staleAfterMs || !token || !_hostConnected ? 1 : 0);
    if (status != _videoStatus && _videoImage.slot < 0) {
        _videoStatus = status;
        if (map) {
            if (status) {
                gTft.fillRect(0, 0, 320, es3map::kStatusHeight, ILI9341_BLACK);
                gTft.setTextSize(1); gTft.setTextColor(ILI9341_YELLOW);
                gTft.setCursor(2, 4);
                gTft.print(!_hostConnected ? "JETSON OFFLINE - MAP STALE" :
                    (status == 2 ? "Map unavailable - last known" : "MAP STALE - last known"));
            }
            // Fresh JPEG restores its own status. Neither stale threshold
            // clears imagery/attribution or paints over the tab strip.
        } else {
            if (status == 2) {
                gTft.fillRect(config.viewportX, config.viewportY, config.viewportW, config.viewportH, ILI9341_BLACK);
                printCentered("Video unavailable", 160, 112, ILI9341_WHITE);
            }
            const int y = config.viewportY >= 10 ? 1 : config.viewportY;
            if (status || config.viewportY >= 10) gTft.fillRect(0, y, 150, 9, ILI9341_BLACK);
            gTft.setTextSize(1); gTft.setTextColor(status ? ILI9341_YELLOW : ILI9341_GREEN);
            gTft.setCursor(2, y);
            if (status) gTft.print(_hostConnected ? "VIDEO STALE" : "JETSON OFFLINE");
            else if (config.viewportY >= 10) gTft.print("LIVE USB");
        }
        if (!videoCoversTabs() || _navigation) drawTabs();
    }
}
