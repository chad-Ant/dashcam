/*
 * CYD bring-up test for the common ESP32-2432S028R "Cheap Yellow Display".
 *
 * Checks the ILI9341 display and reads SD-card capacity without writing files.
 * It intentionally does not initialise the future I2C link to the XIAO C3:
 * the address, bus owner, connector pins, and command protocol still need to
 * be agreed before either controller drives that bus.
 *
 * Arduino IDE board: ESP32 Dev Module
 * Libraries: Adafruit GFX Library, Adafruit ILI9341, SD (all already installed
 *            in this development environment).
 */

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// Standard ESP32-2432S028R / 2.8-inch CYD SPI wiring. Verify against the board
// silk-screen or seller schematic before changing hardware connections.
static constexpr int TFT_MISO = 12;
static constexpr int TFT_MOSI = 13;
static constexpr int TFT_SCLK = 14;
static constexpr int TFT_CS   = 15;
static constexpr int TFT_DC   = 2;
static constexpr int TFT_RST  = 4;
static constexpr int SD_CS    = 5;

static Adafruit_ILI9341 gTft(TFT_CS, TFT_DC, TFT_RST);

static void drawHeader(const char *title)
{
  gTft.fillScreen(ILI9341_BLACK);
  gTft.fillRect(0, 0, gTft.width(), 30, ILI9341_YELLOW);
  gTft.setTextColor(ILI9341_BLACK);
  gTft.setTextSize(2);
  gTft.setCursor(8, 8);
  gTft.print(title);
}

static void drawColourCheck()
{
  static constexpr uint16_t colours[] = {
    ILI9341_RED, ILI9341_GREEN, ILI9341_BLUE, ILI9341_WHITE, ILI9341_YELLOW
  };
  const int stripeWidth = gTft.width() / static_cast<int>(sizeof(colours) / sizeof(colours[0]));
  for (size_t i = 0; i < sizeof(colours) / sizeof(colours[0]); ++i) {
    gTft.fillRect(static_cast<int>(i) * stripeWidth, 44, stripeWidth, 48, colours[i]);
  }
}

static bool probeSd(uint64_t &sizeBytes)
{
  if (!SD.begin(SD_CS, SPI, 10000000)) return false;
  const uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) return false;
  sizeBytes = SD.cardSize();
  return sizeBytes > 0;
}

static void showSdStatus()
{
  uint64_t sizeBytes = 0;
  const bool sdOk = probeSd(sizeBytes);

  gTft.setTextSize(2);
  gTft.setCursor(12, 112);
  gTft.setTextColor(sdOk ? ILI9341_GREEN : ILI9341_RED);
  gTft.print(sdOk ? "SD card: OK" : "SD card: NOT FOUND");

  if (sdOk) {
    gTft.setTextColor(ILI9341_WHITE);
    gTft.setCursor(12, 140);
    gTft.printf("Capacity: %llu MB", static_cast<unsigned long long>(sizeBytes / (1024ULL * 1024ULL)));
  }

  Serial.printf("[cyd] display OK; SD %s", sdOk ? "OK" : "not found");
  if (sdOk) Serial.printf(" (%llu MiB)", static_cast<unsigned long long>(sizeBytes / (1024ULL * 1024ULL)));
  Serial.println();
}

void setup()
{
  Serial.begin(115200);
  delay(200);

  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
  gTft.begin();
  gTft.setRotation(1);  // landscape: 320 x 240

  drawHeader("CYD BRING-UP");
  drawColourCheck();
  showSdStatus();

  gTft.setTextColor(ILI9341_CYAN);
  gTft.setTextSize(1);
  gTft.setCursor(12, 210);
  gTft.print("Display + read-only SD test");
}

void loop()
{
  static uint32_t previousMs = 0;
  static bool markerOn = false;
  const uint32_t now = millis();
  if (now - previousMs < 500U) return;
  previousMs = now;
  markerOn = !markerOn;
  gTft.fillCircle(300, 220, 7, markerOn ? ILI9341_GREEN : ILI9341_DARKGREY);
}
