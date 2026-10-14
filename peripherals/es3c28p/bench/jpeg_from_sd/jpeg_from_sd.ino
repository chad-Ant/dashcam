// Copy a baseline <=64KiB 320x180 JPEG to /preview.jpg. All SD and TFT access
// stays on loop. UART prints decode/blit timing; USB is not used by this bench.
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <SD_MMC.h>
#include <esp_heap_caps.h>
#include "es3Board.h"
#include "Es3Panel.h"
#include "JpegDecoder.h"
static SPIClass spi(FSPI);
static Adafruit_ILI9341 tft(&spi, Es3Board::tftDc, Es3Board::tftCs, -1);
static Es3JpegDecoder decoder;
static uint8_t* jpeg = nullptr;
static uint16_t* pixels = nullptr;
static uint16_t strip[320*8];
static size_t jpegBytes = 0;
void setup() {
    Serial0.begin(115200);
    SD_MMC.setPins(Es3Board::sdClock, Es3Board::sdCommand, Es3Board::sdD0,
                   Es3Board::sdD1, Es3Board::sdD2, Es3Board::sdD3);
    if (!SD_MMC.begin("/sdcard", false, false, 20000)) { Serial0.println("SD unavailable"); return; }
    File file=SD_MMC.open("/preview.jpg", FILE_READ);
    if (!file || file.size()<4 || file.size()>65536) { Serial0.println("preview.jpg absent/oversized"); return; }
    jpegBytes=file.size();
    jpeg=static_cast<uint8_t*>(heap_caps_malloc(65536,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
    pixels=static_cast<uint16_t*>(heap_caps_malloc(320*180*2,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
    if (!jpeg || !pixels || file.read(jpeg,jpegBytes)!=jpegBytes) { jpegBytes=0; return; }
    file.close();
    spi.begin(Es3Board::tftClock, Es3Board::tftMiso, Es3Board::tftMosi, Es3Board::tftCs);
    tft.begin(40000000);es3ConfigureIpsPanel(tft);tft.setRotation(1);
    pinMode(Es3Board::backlight,OUTPUT);digitalWrite(Es3Board::backlight,HIGH);
    tft.fillScreen(ILI9341_BLACK);
}
void loop() {
    if (!jpegBytes) { delay(1000); return; }
    uint32_t started=micros();
    const bool ok=decoder.decode(jpeg,jpegBytes,pixels,320,180);
    const uint32_t decodeUs=micros()-started;
    started=micros();
    if (ok) for (int row=0;row<180;row+=8) {
        const int rows=min(8,180-row);
        memcpy(strip,pixels+row*320,320*rows*2);
        tft.drawRGBBitmap(0,row+30,strip,320,rows);
        yield();
    }
    Serial0.printf("decode_ok=%d decode_us=%lu blit_us=%lu\n",ok,(unsigned long)decodeUs,
                   (unsigned long)(micros()-started));
    delay(100);
}
