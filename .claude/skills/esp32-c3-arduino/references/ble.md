# Bluetooth LE

The C3 supports **Bluetooth 5 LE only** — there is **no Bluetooth Classic**, so
`BluetoothSerial` / SPP does not exist. Anything that needs a classic serial
profile won't work; use BLE, or for a serial-like experience use the **Nordic
UART Service (NUS)** over BLE.

Two library choices:

- **Built-in `BLEDevice`** (ships with the core, Bluedroid-based). Familiar API,
  heavier RAM use.
- **NimBLE-Arduino** (install from Library Manager). Much smaller footprint —
  often the better pick on the RAM-limited C3, especially alongside WiFi. Same
  concepts, slightly different class names.

> Core 3.x changed the built-in BLE API: strings are Arduino `String` (not
> `std::string`), UUIDs use the `BLEUUID` class, and `BLEScan` results come back
> as a **pointer** (`BLEScanResults*`). Old 2.x BLE snippets often won't compile
> as-is.

## BLE server with a notifying characteristic (built-in lib, 3.x)

```cpp
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcd1234-5678-90ab-cdef-1234567890ab"

BLECharacteristic* ch;
bool connected = false;

class SrvCb : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { connected = true; }
  void onDisconnect(BLEServer* s) override { connected = false; s->startAdvertising(); }
};

void setup() {
  BLEDevice::init("C3-Sensor");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new SrvCb());

  BLEService* svc = server->createService(SERVICE_UUID);
  ch = svc->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  ch->addDescriptor(new BLE2902());        // enables client notifications
  svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

void loop() {
  if (connected) {
    static uint32_t v = 0;
    ch->setValue(String(v++));            // 3.x: Arduino String
    ch->notify();
  }
  delay(1000);
}
```

## BLE scanner / central (built-in lib, 3.x)

```cpp
#include <BLEDevice.h>
#include <BLEScan.h>
void setup() {
  Serial.begin(115200);
  BLEDevice::init("");
  BLEScan* scan = BLEDevice::getScan();
  scan->setActiveScan(true);
  BLEScanResults* res = scan->start(5, false);   // 3.x: returns a pointer
  if (res == nullptr) { Serial.println("scan failed"); return; }
  for (int i = 0; i < res->getCount(); i++) {
    BLEAdvertisedDevice d = res->getDevice(i);
    Serial.printf("%s  RSSI %d\n", d.toString().c_str(), d.getRSSI());
  }
  scan->clearResults();
}
void loop() {}
```

## C3-specific BLE notes

- **RAM is the real constraint.** The Bluedroid stack is large; if a sketch also
  runs WiFi, a TLS client, and buffers sensor data, you can run out of heap.
  Reach for **NimBLE-Arduino** when memory is tight — it's a near drop-in with a
  much smaller footprint.
- **WiFi + BLE coexist** on the single 2.4 GHz radio but share airtime; expect
  reduced throughput when both are busy. For battery devices, doing one thing at
  a time (BLE *or* a quick WiFi burst) and sleeping between is far more efficient
  than running both continuously.
- **BLE keeps the chip awake.** To save power you generally advertise/connect,
  do the work, then deep sleep — maintaining a live connection costs mA
  continuously. See `power.md`.
