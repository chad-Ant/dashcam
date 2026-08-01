#include <CAN.h>

// This is a demo program that listens to messages on the CAN bus sent by a
// device using the code from the SenderStressTest example, and verifies that
// correct bytes were received.
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
uint32_t num_messages_received;
uint32_t num_errors;

void loop() {
  if (millis() - last_stats_ms > REPORT_INTERVAL_SECONDS * 1000) {
    Serial.print("Received in ");
    Serial.print(REPORT_INTERVAL_SECONDS);
    Serial.print(" seconds:\t");
    Serial.print(num_messages_received);
    if (num_messages_received > 0) {
      Serial.print(", errors:\t");
      Serial.print(num_errors);
      Serial.print(" (");
      Serial.print(100.0 * num_errors / num_messages_received);
      Serial.print("%)");
    }
    Serial.println("");

    last_stats_ms = millis();
    num_messages_received = 0;
    num_errors = 0;
  }

  int packet_size = CAN.parsePacket();
  if (packet_size <= 0) {
    return;
  }

  if (CAN.packetRtr()) {
    // Ignore RTRs for now.
    return;
  }

  num_messages_received++;

  uint32_t pid = CAN.packetId();
  uint8_t data[8];
  int data_length = 0;
  while (data_length < packet_size && data_length < sizeof(data)) {
    int byte_read = CAN.read();
    if (byte_read == -1) {
      break;
    }

    data[data_length++] = byte_read;
  }

  uint8_t expected_payload[8] =
      { 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, pid };
  bool mismatch = data_length != 8;
  for (int i = 0; i < 8; i++) {
    if (data[i] != expected_payload[i]) {
      mismatch = true;
    }
  }

  if (mismatch) {
    num_errors++;
    Serial.print("Unexpected data received! pid = ");
    Serial.print(pid);
    Serial.print(" (0x");
    Serial.print(pid, HEX);
    Serial.print("), data: ");
    for (int i = 0; i < data_length; i++) {
      if (i != 0) {
        Serial.print(" ");
      }
      Serial.print(data[i], HEX);
    }
    Serial.println();
  }
}
