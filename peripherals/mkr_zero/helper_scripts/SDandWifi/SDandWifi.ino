#include <SPI.h>
#include "wiring_private.h"
#include <Arduino.h>
#include <WiFi101.h>
#include <SdFat.h>

const char SSID[] = "401 402 403";
const char PASS[] = "0938690720kien";

#define SD_MOSI 10
#define SD_MISO 9
#define SD_SCK  24
#define SD_CS   3
SPIClassSAMD SPI_SD(&sercom3, SD_MISO, SD_SCK, SD_MOSI, SPI_PAD_2_SCK_3, SERCOM_RX_PAD_1);

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
