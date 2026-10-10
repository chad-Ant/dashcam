// BENCH FIRMWARE ONLY. Replaces, does not run alongside, production CAN/GPS.
// Compile with --library peripherals/mkr_zero/libraries/GRF250 on SAMD 1.8.14.
#include <Arduino.h>
#include <wiring_private.h>
#include <GRF250.h>

// A3=PA04 SERCOM0/PAD0 TX; A6=PA07 SERCOM0/PAD3 RX, both ALT.
// Serial1 stays reserved for the C3. A4/A5 servo reservations are untouched.
Uart lidarUart(&sercom0, A6, A3, SERCOM_RX_PAD_3, UART_TX_PAD_0);
void SERCOM0_Handler() { lidarUart.IrqHandler(); }
static grf250::Driver lidar;
static bool pinsOk = false;
static uint32_t usbDrops = 0, lastLoopUs = 0;

// Reuses the production CANStreamHw nonblocking endpoint check. This standalone
// sketch has only CDC; no extra USB class may be added without revisiting this.
static bool usbIdle() {
    if (!USBDevice.configured() || !Serial.dtr()) return false;
    for (uint8_t ep = 1; ep < USB_EPT_NUM; ++ep)
        if (USB->DEVICE.DeviceEndpoint[ep].EPCFG.bit.EPTYPE1 == 3u)
            return USB->DEVICE.DeviceEndpoint[ep].EPSTATUS.bit.BK1RDY == 0u;
    return false;
}
static bool sendLine(char* buf, int n, size_t capacity) {
    if (n <= 0 || static_cast<size_t>(n) + 7 > capacity || n + 6 > 63 || !usbIdle()) return false;
    const uint16_t crc = grf250::crc(reinterpret_cast<const uint8_t*>(buf), static_cast<size_t>(n));
    const int added = snprintf(buf + n, capacity - static_cast<size_t>(n), " %04X\n", unsigned(crc));
    if (added != 6) return false;
    return Serial.write(reinterpret_cast<const uint8_t*>(buf), static_cast<size_t>(n + added)) == static_cast<size_t>(n + added);
}
static void emit(const grf250::Sample& s) {
    char line[64];
    const int n = snprintf(line, sizeof(line), "L1 %08lX %08lX %08lX %08lX %08lX %08lX",
        static_cast<unsigned long>(s.sequence), static_cast<unsigned long>(s.receivedUs),
        static_cast<unsigned long>(static_cast<uint32_t>(s.firstCm)),
        static_cast<unsigned long>(static_cast<uint32_t>(s.firstDb)),
        static_cast<unsigned long>(static_cast<uint32_t>(s.lastCm)),
        static_cast<unsigned long>(static_cast<uint32_t>(s.lastDb)));
    if (!sendLine(line, n, sizeof(line))) ++usbDrops;
}
static void emitHealth(uint32_t now) {
    static uint32_t previous = 0;
    if (uint32_t(now - previous) < 1000000u) return;
    previous = now;
    char line[64];
    const int n = snprintf(line, sizeof(line), "S1 %08lX %08lX %08lX %08lX %08lX %08lX",
        static_cast<unsigned long>(now), static_cast<unsigned long>(lidar.ready()),
        static_cast<unsigned long>(lidar.firmware), static_cast<unsigned long>(lidar.faults),
        static_cast<unsigned long>(usbDrops), static_cast<unsigned long>(lidar.parseErrors()));
    if (!sendLine(line, n, sizeof(line))) ++usbDrops;
}
// Q1 <8 hex nonce> -> T1 <nonce> <micros> + CRC. No retained response queue:
// host measures round-trip; a busy USB endpoint drops the sync reply for retry.
static void serviceClock() {
    static char request[12]; static uint8_t used = 0; static bool overflow = false;
    for (unsigned i = 0; i < 32 && Serial.available() > 0; ++i) {
        const int ch = Serial.read();
        if (ch < 0) break;
        if (ch == '\n') {
            if (!overflow && used == 11 && memcmp(request, "Q1 ", 3) == 0) {
                bool valid = true;
                for (unsigned j = 3; j < 11; ++j)
                    if (!((request[j] >= '0' && request[j] <= '9') || (request[j] >= 'A' && request[j] <= 'F'))) valid = false;
                if (valid) {
                    request[11] = '\0'; char reply[40];
                    const int n = snprintf(reply, sizeof(reply), "T1 %s %08lX", request + 3, static_cast<unsigned long>(micros()));
                    if (!sendLine(reply, n, sizeof(reply))) ++usbDrops;
                }
            }
            used = 0; overflow = false;
        } else if (used < 11) request[used++] = static_cast<char>(ch);
        else overflow = true;
    }
}
void setup() {
    Serial.begin(115200); // Never wait for USB.
    lidarUart.begin(115200);
    pinsOk = pinPeripheral(A3, PIO_SERCOM_ALT) == 0 && pinPeripheral(A6, PIO_SERCOM_ALT) == 0;
    pinMode(LED_BUILTIN, OUTPUT);
    lastLoopUs = micros();
}
void loop() {
    if (!pinsOk) return;
    const uint32_t now = micros();
    if (uint32_t(now - lastLoopUs) > 10000u) {
        // UART arrival timestamps are not recoverable after a stalled loop.
        for (unsigned i = 0; i < 256 && lidarUart.available() > 0; ++i) (void)lidarUart.read();
        lidar.transportFault(now);
    }
    lastLoopUs = now;
    serviceClock();
    uint8_t tx[10];
    if (lidarUart.availableForWrite() >= 10) {
        const size_t n = lidar.nextCommand(micros(), tx, sizeof(tx));
        if (n && lidarUart.write(tx, n) != n) lidar.transportFault(micros());
    }
    for (unsigned i = 0; i < 128 && lidarUart.available() > 0; ++i) {
        const int b = lidarUart.read();
        if (b < 0) break;
        grf250::Sample sample;
        if (lidar.receive(static_cast<uint8_t>(b), micros(), sample)) emit(sample);
    }
    digitalWrite(LED_BUILTIN, lidar.ready() ? HIGH : LOW);
    emitHealth(micros());
}
