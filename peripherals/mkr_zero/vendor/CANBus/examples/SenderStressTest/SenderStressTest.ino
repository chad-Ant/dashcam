#include <CAN.h>

// This is a demo program that sends as many messages over the CAN bus as it
// can, and can be used as a stress test for the hardware and a CAN receiver
// device.
//
// DO NOT USE IT IN THE CAN NETWORK OF A REAL VEHICLE as it can cause unexpected
// side effects.
//
// It was tested on Arduino Uno, Arduino Micro, Adafruit Feather nRF52832 and
// Adafruit ItsyBitsy nRF52840 Express, and should be trivial to tweak to
// support pretty much any other board with SPI.
//
// Connections:
//  MCP | BOARD
//  INT | Not used, can connect to Pin 9
//  SCK | SCK
//   SI | MO
//   SO | MI
//   CS | Pin 7
//  GND | GND
//  VCC | 3.3V

const int CS_PIN = 7;
const int IRQ_PIN = 9;
const int QUARTZ_MHZ = 16;  // Some MCP2515 boards have 8 MHz quartz.
const int SPI_MHZ = 10;
const long BAUD_RATE = 500 * 1E3;  // 500k baud rate.

void setup() {
  Serial.begin(115200);

  uint32_t startTimeMs = millis();
  while (!Serial && millis() - startTimeMs < 5000);
  if (!Serial) {
    Serial.println("Started!");
  } else {
    // Whatever, noone's going to see anyways.
  }

  CAN.setClockFrequency(QUARTZ_MHZ * 1E6);
  CAN.setSPIFrequency(SPI_MHZ * 1E6);
  CAN.setPins(CS_PIN, IRQ_PIN);

  while (!CAN.begin(BAUD_RATE)) {
    Serial.println("Failed to connect to the CAN controller!");
    delay(1000);
  }

  Serial.println("CAN controller connected");
}

// Interval in seconds between printing reports.
const uint32_t REPORT_INTERVAL_SECONDS = 1;

uint32_t last_stats_ms = millis();
uint32_t num_messages_sent = 0;
uint32_t num_errors = 0;

void loop() {
  for (uint16_t pid = 1; pid <= 0xff; pid++) {
    uint32_t current_time_ms = millis();
    if (current_time_ms - last_stats_ms > REPORT_INTERVAL_SECONDS * 1000) {
      Serial.print("Packets sent over 1 second:\t");
      Serial.print(num_messages_sent);
      Serial.print(", errors:\t");
      Serial.println(num_errors);
      last_stats_ms = current_time_ms;
      num_messages_sent = 0;
      num_errors = 0;
    }

    if (!CAN.beginPacket(pid)) {
      Serial.println("beginPacket() failed.");
      num_errors++;
      continue;
    }

    uint8_t payload[8] = { 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, pid };
    CAN.write(payload, 8);
    if (CAN.endPacket()) {
      num_messages_sent++;
    } else {
      Serial.println("endPacket() failed.");
      num_errors++;
      continue;
    }
  }
}
