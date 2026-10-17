/**
 * @file cydGraphics.h
 * @brief Reusable ESP32-2432S028 display, touch, SD, and tab-dashboard UI.
 *
 * The class owns the CYD's ILI9341, XPT2046, and tab renderer. It receives a
 * CydStorage service for calibration persistence, keeping SD access separate.
 */

#pragma once

#include <Arduino.h>
#include "cydStorage.h"

struct CydDashboardPage {
    const char* tab;
    const char* title;
    const char* line1;
    const char* line2;
    const char* line3;
    const char* line4;
    bool showBrightnessControl;
    bool showSdSplash;
    bool allowTouchCalibration;
};

/**
 * Ambient-light to screen-backlight mapping.
 *
 * Set darkRaw and brightRaw from actual readings. They may be in either order:
 * a divider that reads lower in bright light is supported without rewiring.
 * The feature is disabled by default because the CYD does not standardise the
 * external LDR's ADC pin.
 */
struct CydBrightnessConfig {
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

class CydGraphics {
public:
    static constexpr uint8_t kMaxPages = 6;

    explicit CydGraphics(CydStorage& storage);

    /** Sets the fixed application page table. Call before begin(). */
    bool setPages(const CydDashboardPage* pages, uint8_t count);

    /** Configures LDR-driven PWM backlight control. Call before begin(). */
    bool setBrightnessConfig(const CydBrightnessConfig& config);

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

    /** Begins fresh calibration without replacing saved values until SAVE. */
    void requestCalibration();

    bool mapVisible() const { return _started && !_calibrating && !_calibrationTesting && _activePage == 5; }
    bool calibrationActive() const { return !_started || _calibrating || _calibrationTesting; }
    void mapPlaceholder(const char* message);
    void mapStale();
    bool drawMapBlock(int x, int y, int width, int height, uint16_t* pixels);
    bool updatePageText(uint8_t page, const char text[5][24]);

    bool sdAvailable() const;
    bool calibrationLoaded() const;
    uint8_t activePage() const;
    uint8_t brightnessDuty() const;
    uint16_t brightnessThresholdRaw() const;
    int filteredLightRaw() const;

private:
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

    CydStorage& _storage;
    const CydDashboardPage* _pages;
    CydDashboardPage _ownedPages[kMaxPages]{};
    char _pageText[kMaxPages][5][24]{};
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
    CydTouchCalibrationRecord _calibration;
    bool _touchHeld;
    uint32_t _lastTouchMs;
    CydBrightnessConfig _brightnessConfig;
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
    bool brightnessConfigIsSane(const CydBrightnessConfig& config) const;
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

    uint8_t touchTransfer(uint8_t out);
    uint16_t touchReadAxis(uint8_t command);
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
