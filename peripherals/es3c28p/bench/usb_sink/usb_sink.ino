// Destructive bench firmware: replaces the dashboard. USB is a raw byte sink;
// statistics go ONLY to Serial0. Compare hwcdc and TinyUSB on the actual board.
#include <Arduino.h>
uint32_t received = 0, lastReport = 0;
void setup() {
    Serial0.begin(115200);
    Serial.setRxBufferSize(8192);
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);
    lastReport = millis();
}
void loop() {
    for (size_t budget = 0; budget < 8192 && Serial.available(); ++budget) {
        if (Serial.read() >= 0) ++received;
    }
    const uint32_t now = millis(), elapsed = now - lastReport;
    if (elapsed >= 1000) {
        Serial0.printf("sink bytes/s=%lu usb=%s\n", (unsigned long)(uint64_t(received)*1000/elapsed),
                       ARDUINO_USB_MODE ? "hwcdc" : "tinyusb");
        received = 0; lastReport = now;
    }
    delay(1);
}
