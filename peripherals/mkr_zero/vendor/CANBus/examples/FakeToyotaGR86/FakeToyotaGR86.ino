#include <CAN.h>

// This is a demo program that sends messages over the CAN bus in
// a way that resembles real messages you can receive if you listen
// to messages on the CAN bus of a 2022 Toyota GR86.
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
const int SPI_MHZ = 8;

void setup() {
  Serial.begin(115200);

  uint32_t startTimeMs = millis();
  while (!Serial && millis() - startTimeMs < 1000);
  if (!Serial) {
    Serial.println("Started!");
  } else {
    // Whatever, noone's going to see anyways.
  }

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

class FakeTpmsEcu {
public:
  FakeTpmsEcu() {
    has_continuation_frame = false;
    original_frame_acked = false;
  }

  void scheduleNextFrame(uint16_t pid, uint8_t *data, uint8_t len) {
    if (has_continuation_frame) {
      Serial.print("Scheduling new frame, even though there is one pending already. Original was ");
      if (original_frame_acked) {
        Serial.println("acked.");
      } else {
        Serial.println("NOT acked.");
      }
    }
    has_continuation_frame = true;
    original_frame_acked = false;
    next_frame_pid = pid;
    memcpy(next_frame_data, data, len);
    next_frame_length = len;
  }

  void handleFrameAck(uint8_t separation_time_millis = 0) {
    if (!has_continuation_frame) {
      return;
    }

    original_frame_acked = true;
    next_frame_timestamp_millis = millis() + separation_time_millis;
  }

  void sendNextFrameIfNeeded() {
    if (!has_continuation_frame || !original_frame_acked) {
      return;
    }

    // Unsigned math magic to check if "time_diff" is "negative":
    unsigned long time_diff = millis() - next_frame_timestamp_millis;
    if (time_diff >> (8 * sizeof(time_diff) - 1)) {
      return;
    }

    send_data(next_frame_pid, next_frame_data, next_frame_length);
    has_continuation_frame = false;
    original_frame_acked = false;
  }

private:
  bool has_continuation_frame;
  uint16_t next_frame_pid;
  uint8_t next_frame_data[8];
  uint8_t next_frame_length;

  bool original_frame_acked;
  unsigned long next_frame_timestamp_millis;
} fake_tpms_ecu;

// Forward declaration for a helper.
void try_to_receive_data();
void generate_payload(uint16_t pid, uint8_t *payload);
boolean send_data(uint16_t pid, uint8_t *payload, uint8_t len);

// TODO: Refactor the loop() function. It should first try to see if there's anything to send,
// then if there's anything to receive.
void loop() {
  // Many PIDs are sent 50 times per second.
  uint16_t num_cycles_per_second = 50;

  // 0x40 and 0x41 are intentionally duplicated in this array as they are sent 100 times
  // per second (double that for other PIDs) in the real car.
  uint16_t pids[] = {
    // These are sent 100 times per second:
    0x40, 0x41,
    // These are sent 50 times per second, part 1:
    0x118, 0x138, 0x139, 0x13B, 0x13C,
    // These are sent 100 times per second (duplicates):
    0x40, 0x41,
    // These are sent 50 times per second, part 2:
    0x143, 0x146,
    // TODO: These are actually sent less frequently than 50 times per second:
    0x241, // 20 times per second
    0x345, // 10 times per second
  };
  uint16_t num_messages_per_cycle = sizeof(pids) / sizeof(pids[0]);
  uint16_t num_messages_per_second =
      num_cycles_per_second * num_messages_per_cycle;

  unsigned long first_message_sent_micros = micros();
  unsigned long num_messages_sent = 0;
  for (int i = 0; i < num_messages_per_cycle; i++, num_messages_sent++) {
    uint16_t pid = pids[i];
    uint8_t payload[8];
    generate_payload(pid, payload);

    if (!send_data(pid, payload, 8)) {
      Serial.println("Failed to send a message");
    }

    unsigned long next_message_time_micros =
        first_message_sent_micros
            + (num_messages_sent * 1000000) / num_messages_per_second;
    if ((long)(micros() - next_message_time_micros) < 0) {
      try_to_receive_data();
      fake_tpms_ecu.sendNextFrameIfNeeded();
      delayMicroseconds(10);
    }
  }
}

boolean send_data(uint16_t id, uint8_t *payload, uint8_t len) {
  if (!CAN.beginPacket(id)) {
    Serial.println("beginPacket() failed.");
    return false;
  }

  CAN.write(payload, len);
  if (!CAN.endPacket()) {
    Serial.println("endPacket() failed.");
    return false;
  }

  return true;
}

void try_to_receive_data() {
  int packet_size = CAN.parsePacket();
  if (packet_size <= 0) {
    return;
  }

  if (CAN.packetRtr()) {
    // Ignore RTRs.
    return;
  }

  uint32_t id = CAN.packetId();
  uint8_t data[8] = {0};
  int data_length = 0;
  while (data_length < packet_size && data_length < sizeof(data)) {
    int byte_read = CAN.read();
    if (byte_read == -1) {
      break;
    }

    data[data_length++] = byte_read;
  }

  if (id == 0x750 && data_length >= 1 && data[0] == 0x2a) {
    if (data_length >= 3 && data[1] == 0x02 && data[2] == 0x21) {
      if (data[3] == 0x30) {
        // TPMS pressures request.
        uint8_t response[8] = {0};
        response[0] = 0x2a;
        response[1] = 0x10;  // "1" means "first frame in a sequence"
        response[2] = 0x07;
        response[3] = 0x61;
        response[4] = 0x30;
        response[5] = 0xAB;  // FL tire pressure
        response[6] = 0xAC;  // FR tire pressure
        response[7] = 0xAD;  // RR tire pressure
        send_data(0x758, response, 8);

        response[0] = 0x2a;
        response[1] = 0x21;  // "2" means "continuation frame", "1" means "first continuation frame".
        response[2] = 0xAE;  // RL tire pressure
        response[3] = 0x00;
        response[4] = 0x00;
        response[5] = 0x00;
        response[6] = 0x00;
        response[7] = 0x00;
        fake_tpms_ecu.scheduleNextFrame(0x758, response, 8);
      } else if (data[3] == 0x16) {
        // TPMS temperatures request.
        uint8_t response[8] = {0};
        response[0] = 0x2a;
        response[1] = 0x10;  // "1" means "first frame in a sequence"
        response[2] = 0x07;
        response[3] = 0x61;
        response[4] = 0x16;
        response[5] = 40 + 21;  // FL tire temperature: 21ºC
        response[6] = 40 + 22;  // FR tire temperature
        response[7] = 40 + 23;  // RR tire temperature
        send_data(0x758, response, 8);

        response[0] = 0x2a;
        response[1] = 0x21;  // "2" means "continuation frame", "1" means "first continuation frame".
        response[2] = 40 + 24;  // RL tire temperature
        response[3] = 0x00;
        response[4] = 0x00;
        response[5] = 0x00;
        response[6] = 0x00;
        response[7] = 0x00;
        fake_tpms_ecu.scheduleNextFrame(0x758, response, 8);
      }
    } else if (data_length >= 3 && data[1] == 0x30 && data[2] == 0x00) {
      fake_tpms_ecu.handleFrameAck(data[3]);
    }
  }
}

void generate_payload(uint16_t pid, uint8_t *payload) {
  memset(payload, /* value= */ 0, /* size= */ 8);

  switch (pid) {
    case 0x40: {
      uint8_t accelerator_pedal_percent = 42;
      payload[4] = accelerator_pedal_percent * 255 / 100;
      payload[5] = payload[4];
      payload[6] = payload[4];

      // The clutch pedal has two sensors:
      // - 0% and >0% (used here)
      // - 100% and <100% (haven't found yet)
      // TODO: Find where data from the second sensor is.
      bool clutch_down = false;
      payload[1] = (clutch_down ? 0x80 : 0x00);

      // RPMs are believed to be encoded with just 14 bits.
      uint16_t rpm = 3456;
      payload[2] = rpm & 0xFF;
      payload[3] = (rpm >> 8) & 0x3F;
      break;
    }


    case 0x138: {
      // Pretend that the steering wheel is turned by 123 degrees to the left
      int16_t steering_angle_degrees = -123;
      int16_t steering_value = steering_angle_degrees * 10;
      payload[2] = steering_value & 0xFF;
      payload[3] = (steering_value >> 8) & 0xFF;

      // TODO: Verify the scale for this value. The current scale is suspicious,
      // but matches real-world testing so far. Need to go to a skid pad to
      // verify for sure.
      float yaw_rate_degrees_per_second = -12.3;
      int16_t yaw_rate_value = (int16_t)(-yaw_rate_degrees_per_second / 0.2725);
      payload[4] = yaw_rate_value & 0xFF;
      payload[5] = (yaw_rate_value >> 8) & 0xFF;
      break;
    }

    case 0x139: {
      uint16_t speed_m_s = 10;  // 36 km/h, ~22.4 mph.
      // The encoding seems to be roughly radians per second x100.
      // The coefficient was tuned by comparing the values against an external
      // GPS from a session where I drove in a straight line on a highway at
      // constant speed on cruise control.
      uint16_t speed_value = (uint16_t)(speed_m_s * 63.72);
      payload[2] = speed_value & 0xFF;
      payload[3] = (speed_value >> 8) & 0xFF;

      // The units used for the master brake cylinder pressure are believed to
      // be 1/128 kPa.
      float brake_pressure_kPa = 1024;
      payload[4] = 0x0C;
      payload[5] = (uint8_t)(brake_pressure_kPa / 128);
      break;
    }

    case 0x345: {
      uint8_t oil_temperature_celsius = 100;
      payload[3] = oil_temperature_celsius + 40;

      uint8_t coolant_temperature_celsius = 90;
      payload[4] = coolant_temperature_celsius + 40;
      break;
    }
  }
}
