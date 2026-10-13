// STANDALONE bench firmware, NOT the production CAN/IMU/GNSS sketch.
// First run is encoder-only. Read README before changing either interlock.
#include <Arduino.h>
#include <Servo.h>
#include <EncoderPan.h>
#include <AS5048B.h>
#include <BoundedWireEncoder.h>
#include <GatedPwm.h>
#include <stdio.h>
using namespace encoderpan;

constexpr bool kCalibrationMeasured = false;
constexpr bool kInstallationVerified = false;
constexpr uint8_t kPwmPin = A5;
constexpr uint8_t kGatePin = A4; // proposed single-axis assignment; external pulldown

Config panConfig() {
    Config c;
    c.calibrated = kCalibrationMeasured;
    // PLACEHOLDERS: measure endpoints, direction, speed/error bounds first.
    return c;
}
AS5048Config encoderConfig() {
    AS5048Config c;
    c.calibrated = kCalibrationMeasured;
    // Set zeroCounts from an independently aligned forward reference.
    return c;
}
class MkrPwmBackend {
public:
    void begin() {
        digitalWrite(kGatePin, LOW); pinMode(kGatePin, OUTPUT);
        digitalWrite(kPwmPin, LOW); pinMode(kPwmPin, OUTPUT);
    }
    void gateOff() { digitalWrite(kGatePin, LOW); }
    bool attach() { return servo_.attach(kPwmPin) != INVALID_SERVO; }
    void detach() { servo_.detach(); digitalWrite(kPwmPin, LOW); }
    bool write(uint16_t us) {
        servo_.writeMicroseconds(us);
        return servo_.readMicroseconds() == us; // command check, NOT feedback
    }
    bool enableWhenLow() {
        // SAMD Servo toggles pin in TC4 ISR: avoid enabling mid-pulse.
        const uint32_t mask = __get_PRIMASK();
        __disable_irq();
        const bool low = digitalRead(kPwmPin) == LOW;
        if (low) digitalWrite(kGatePin, HIGH);
        __set_PRIMASK(mask);
        return low;
    }
private:
    Servo servo_; // TC4; don't combine with another TC4 owner
};
Controller pan(panConfig());
AS5048B encoder(encoderConfig());
BoundedWireEncoder bus(Wire);
MicrosClock clockSource;
MkrPwmBackend hardware;
GatedPwm<MkrPwmBackend> output(hardware);
uint32_t lastSample = 0, lastPrint = 0, sequence = 0;
EncoderSample latest;

void setup() {
    hardware.begin();
    output.begin(kInstallationVerified);
    Serial.begin(115200); // no while(!Serial): headless boot must work
    Wire.begin(); // standalone ONLY; production uses I2CBus owner
    Wire.setClock(400000);
}
void loop() {
    uint32_t now = micros();
    if (uint32_t(now-lastSample) >= 5000) {
        lastSample = now;
        latest = encoder.sample(bus, clockSource);
        now = micros();
        // Uncalibrated monitoring still displays the raw-derived angle.
        if (kCalibrationMeasured) (void)pan.observe(latest, now);
    }
    // Process deadlines BEFORE any input can renew a command.
    if (!output.apply(pan.tick(now), now)) pan.outputFailed();
    // At most one manual command/pass. No automatic scans or TTL renewal.
    // a=arm; 1/2/3=-10/0/+10 deg for 250ms; s=stop; x=clear fault.
    // NOT a production host protocol: no source timestamp/sequence on USB.
    if (Serial.available() > 0) {
        const int key = Serial.read();
        if (key == 's') { pan.stop(); output.off(); }
        else if (key == 'x') { (void)pan.clearFault(); }
        else if (key == 'a' && kInstallationVerified) { (void)pan.arm(now); }
        else if (key >= '1' && key <= '3' && kInstallationVerified) {
            Command command;
            command.sequence = ++sequence; command.issuedUs = now;
            command.lifetimeUs = 250000; command.targetMdeg = (key-'2')*10000;
            (void)pan.request(command, now);
        } else if (key != '\r' && key != '\n') { pan.stop(); output.off(); }
    }
    now = micros();
    if (!output.apply(pan.tick(now), now)) pan.outputFailed();
    if (uint32_t(now-lastPrint) >= 100000) {
        lastPrint = now;
        char line[96];
        const Feedback f = pan.feedback(micros());
        const int n = snprintf(line, sizeof(line), "pan,%lu,%u,%ld,%ld,%u,%u,%u,%u\n",
            static_cast<unsigned long>(latest.timeUs), static_cast<unsigned>(latest.rawCounts), static_cast<long>(latest.angleMdeg),
            static_cast<long>(latest.errorMdeg), latest.healthy ? 1u : 0u,
            f.settled ? 1u : 0u, static_cast<unsigned>(f.fault), output.enabled() ? 1u : 0u);
        if (n > 0 && static_cast<size_t>(n) < sizeof(line) && Serial
                && Serial.availableForWrite() >= n)
            (void)Serial.write(reinterpret_cast<const uint8_t*>(line), static_cast<size_t>(n));
    }
}
