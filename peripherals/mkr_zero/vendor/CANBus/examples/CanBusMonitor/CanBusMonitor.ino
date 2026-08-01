#include <CAN.h>

// This is a demo program that listens to messages on the CAN bus and prints them out to Serial.
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
const int SPI_MHZ = 16;

void setup() {
  Serial.begin(115200);

  uint32_t startTimeMs = millis();
  while (!Serial);

  Serial.println("Started!");

  CAN.setClockFrequency(QUARTZ_MHZ * 1E6);
  CAN.setSPIFrequency(SPI_MHZ * 1E6);
  CAN.setPins(CS_PIN, IRQ_PIN);

  // Subaru BRZ uses a 500k baud rate.
  while (!CAN.begin(500000)) {
    Serial.println("Failed to connect to the CAN controller!");
    delay(1000);
  }

  Serial.println("CAN controller connected");
}

// Forward declarations for helper functions.
void handle_message(uint32_t pid);
void print_report();

void loop() {
  int packet_size = CAN.parsePacket();
  if (packet_size <= 0) {
    return;
  }

  if (CAN.packetRtr()) {
    // Ignore RTRs for now.
    return;
  }

  uint8_t data[8] = {0};
  int data_length = 0;
  while (data_length < packet_size && data_length < sizeof(data)) {
    int byte_read = CAN.read();
    if (byte_read == -1) {
      break;
    }

    data[data_length++] = byte_read;
  }

  uint32_t packet_id = CAN.packetId();
  handle_message(packet_id, data, data_length);
}

void handle_message(uint32_t packet_id, uint8_t *data, int data_length) {
  // Optional: add something like
  //   if (packet_id != 0x7E8) {
  //     return;
  //   }
  // to only show a subset of messages that match a certain criteria.

  // TODO: Add something smart to avoid spamming Serial.
  // For example, limit the number of messages printed over 10 seconds to 25?

  Serial.print("0x");
  Serial.print(packet_id, HEX);
  Serial.print(", data:");
  for (int i = 0; i < data_length; i++) {
    Serial.print(" ");
    if (data[i] < 0x10) {
      Serial.print("0");  // Add leading zero for readability.
    }
    Serial.print(data[i], HEX);
  }
  Serial.println();
}
