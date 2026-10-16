/**
 * @file es3Graphics.h
 * @brief Reusable ES3C28P display, touch, SD, and tab-dashboard UI.
 *
 * The class owns the ES3C28P's ILI9341, FT6336G, and tab renderer. It receives a
 * Es3Storage service for calibration persistence, keeping SD access separate.
 */

#pragma once

#include <Arduino.h>
#include "es3Storage.h"
#include "es3Video.h"
#include "PanRaster.h"

#include "PageConfig.h"

/**
 * Ambient-light to screen-backlight mapping.
 *
 * Set darkRaw and brightRaw from actual readings. They may be in either order:
 * a divider that reads lower in bright light is supported without rewiring.
 * The feature is disabled by default because the ES3C28P has no onboard LDR.
 * An optional external divider can use GPIO2 on the expansion connector.
 */
struct Es3BrightnessConfig {
    bool enabled;
    int ldrPin;
    uint16_t darkRaw;
    uint16_t brightRaw;
    uint8_t minDuty;
    uint8_t maxDuty;
    uint16_t brightThresholdRaw;
    uint8_t transitionStep;
    uint32_t updateIntervalMs;
};

class Es3Graphics {
public:
    static constexpr uint8_t kMaxPages = es3map::kPageCount;

    explicit Es3Graphics(Es3Storage& storage);

    /** Sets the fixed application page table. Call before begin(). */
    bool setPages(const Es3DashboardPage* pages, uint8_t count);

    /** Copies a title and four printable ASCII lines (23 characters each).
     * Updates the selected page on the next loop; a Live update replaces splash.
     */
    bool updatePageText(uint8_t page, const char (&text)[5][24]);
    void setHostConnected(bool connected);

    /** Configures LDR-driven PWM backlight control. Call before begin(). */
    bool setBrightnessConfig(const Es3BrightnessConfig& config);

    /** Changes the bright/dim raw-LDR switching point while the UI is running. */
    bool setBrightnessThresholdRaw(uint16_t raw);

    /**
     * Streams an uncompressed 24-bit BMP from the SD card to the display.
     * The image dimensions must match width and height exactly.
     */
    bool drawBmp24FromSd(const char* path, int x, int y,
                         uint16_t width, uint16_t height);

    /**
     * Streams a headerless RGB565 image in display-wire (big-endian) order.
     * The file must contain exactly width * height pixels, top to bottom.
     */
    bool drawRgb565FromSd(const char* path, int x, int y,
                          uint16_t width, uint16_t height);

    /** Initialises display/touch and loads or begins touch calibration. */
    bool begin();

    /** Non-blocking UI update. Call once per loop() iteration. */
    void update();
    void attachVideo(Es3Video& video) { _video = &video; }

    /** Begins fresh calibration without replacing saved values until SAVE. */
    void requestCalibration();

    bool sdAvailable() const;
    bool calibrationLoaded() const;
    uint8_t activePage() const;
    uint8_t brightnessDuty() const;
    uint16_t brightnessThresholdRaw() const;
    int filteredLightRaw() const;

private:
    Es3Video* _video = nullptr;
    es3video::VideoState::Image _videoImage;
    es3video::StartVideoRequest _videoConfig;
    uint32_t _videoToken = 0, _lastFrameMs = 0, _blitStarted = 0;
    uint32_t _navigationUntil = 0;
    uint16_t _videoRow = 0;
    uint8_t _videoStatus = 255;
    bool _videoVisible = false, _navigation = false, _videoEver = false;
    uint16_t _videoStrip[320 * 8]; // internal RAM; only loop owns TFT/SPI
    es3map::Binding _binding;
    bool _mapInfo = false;
    void drawMapPage();
    es3video::VideoState::Image _panBackground, _panChrome;
    es3map::PanRaster _panRaster;
    uint32_t _panSeenSeq = 0;
    uint16_t _panRow = 0;
    bool _panDrawing = false, _panDrawMap = false, _panDrawChrome = false;
    bool _panBackgroundDirty = false, _panChromeDirty = false;
    void clearPan();
    void updatePan(const es3map::PanSnapshot& pan, uint32_t now);
    void updateVideo();
    bool videoCoversTabs() const;

    struct TouchRaw {
        uint16_t x;
        uint16_t y;
    };

    static constexpr int kScreenWidth = 320;
    static constexpr int kScreenHeight = 240;
    static constexpr int kTabHeight = 16;
    static constexpr int kTabTop = kScreenHeight - kTabHeight;
    static constexpr int kCalibrationMargin = 24;
    static constexpr uint8_t kCalibrationPointCount = 12;
    static constexpr int kCalibrationFirstThirdX = 114;
    static constexpr int kCalibrationSecondThirdX = 205;
    static constexpr int kCalibrationCenterY = kScreenHeight / 2;
    static constexpr int kBrightnessControlTop = 170;
    static constexpr int kBrightnessControlHeight = 26;
    static constexpr int kBrightnessDecreaseLeft = 18;
    static constexpr int kBrightnessDecreaseWidth = 70;
    static constexpr int kBrightnessIncreaseLeft = 232;
    static constexpr int kBrightnessIncreaseWidth = 70;

    Es3Storage& _storage;
    const Es3DashboardPage* _pages;
    Es3DashboardPage _pageStorage[kMaxPages]{};
    char _pageText[kMaxPages][5][24]{};
    bool _pageDirty = false;
    bool _hostConnected = false;
    uint8_t _pageCount;
    uint8_t _activePage;
    bool _started;
    bool _calibrationValid;
    bool _calibrating;
    bool _calibrationTesting;
    bool _calibrationTouchHeld;
    bool _calibrationTestTouchHeld;
    bool _calibrationCompletePending;
    uint8_t _calibrationTarget;
    TouchRaw _calibrationSamples[kCalibrationPointCount];
    Es3TouchCalibrationRecord _calibration;
    bool _touchHeld;
    uint32_t _lastTouchMs;
    Es3BrightnessConfig _brightnessConfig;
    bool _brightnessConfigured;
    bool _brightnessPwmAttached;
    uint8_t _brightnessDuty;
    uint8_t _brightnessTargetDuty;
    int _filteredLightRaw;
    uint32_t _lastBrightnessUpdateMs;
    uint32_t _lastBrightnessLogMs;

    void drawPage();
    void drawTabs();
    void drawField(const char* label, int y, uint16_t colour);
    void printCentered(const char* text, int centerX, int y, uint16_t colour);
    bool brightnessConfigIsSane(const Es3BrightnessConfig& config) const;
    void updateBrightness(bool force);
    void drawBrightnessStatus();
    void drawBrightnessControls();
    bool handleBrightnessControlTouch(int screenX, int screenY);
    void drawSplashFallback();

    bool buildCalibration();
    void startCalibration();
    void finishCalibration();
    void drawCalibrationTarget();
    void handleCalibration();
    void startCalibrationTest();
    void drawCalibrationTest();
    void handleCalibrationTest();
    void saveCalibration();

    bool readTouchRaw(TouchRaw& raw);
    bool mapTouch(const TouchRaw& raw, int& screenX, int& screenY) const;
    int mapCalibrationAxis(int raw, int atLowTarget, int atMiddleTarget,
                           int atHighTarget, int lowPixel, int middlePixel,
                           int highPixel, int screenSize) const;
    int mapCalibrationAxisFourPoint(int raw, int atLowTarget, int atLeftMiddleTarget,
                                    int atRightMiddleTarget, int atHighTarget,
                                    int lowPixel, int leftMiddlePixel,
                                    int rightMiddlePixel, int highPixel,
                                    int screenSize) const;
    void handleDashboardTouch();
};
