# WiFi & networking

The C3 has 2.4 GHz WiFi 4. The Arduino `WiFi`, `HTTPClient`, `WiFiClientSecure`,
`WebServer`, and `esp_now` APIs are the same as on other ESP32s — the C3 caveats
are about power and the ADC2 clash, not the networking API itself.

## Connect as a station (STA)

```cpp
#include <WiFi.h>
const char* SSID = "your-ssid";
const char* PASS = "your-pass";
const uint32_t WIFI_TIMEOUT_MS = 15000;     // bounded wait — never spin forever

bool connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_TIMEOUT_MS) {
    delay(200);
  }
  return WiFi.status() == WL_CONNECTED;      // caller decides what to do on failure
}

void setup() {
  Serial.begin(115200);
  delay(100);
  if (connectWiFi()) {
    Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("WiFi failed — sleeping and retrying next cycle");
    // defined fallback: e.g. esp_sleep_enable_timer_wakeup(...) + esp_deep_sleep_start()
  }
}
```

The timeout is the point: on an unattended node a bare
`while (WiFi.status() != WL_CONNECTED)` hangs forever the first time the router
is down. See Rule 2 in `mission-critical.md`.

For robustness in long-running sketches, react to events instead of polling:

```cpp
WiFi.onEvent([](WiFiEvent_t e, WiFiEventInfo_t info){
  if (e == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) WiFi.reconnect();
});
WiFi.setAutoReconnect(true);
```

## HTTP request (and HTTPS)

```cpp
#include <HTTPClient.h>
if (WiFi.status() == WL_CONNECTED) {
  HTTPClient http;
  http.begin("http://example.com/api");          // plain HTTP
  int code = http.GET();
  if (code > 0) Serial.println(http.getString());
  http.end();
}
```

For HTTPS, use `WiFiClientSecure`. For real security pin a root CA with
`client.setCACert(rootCA)`; `client.setInsecure()` skips validation and is fine
only for quick tests — flag that trade-off to the user rather than shipping it
silently.

## Minimal web server

```cpp
#include <WiFi.h>
#include <WebServer.h>
WebServer server(80);
void setup() {
  /* connect to WiFi as above */
  server.on("/", []() { server.send(200, "text/plain", "Hello from C3"); });
  server.begin();
}
void loop() { server.handleClient(); }
```

## Access point (AP) mode

```cpp
WiFi.mode(WIFI_AP);
WiFi.softAP("C3-Setup", "12345678");     // password ≥ 8 chars
Serial.println(WiFi.softAPIP());          // default 192.168.4.1
```

## ESP-NOW (low-power peer-to-peer, no router)

Great fit for the C3 in battery sensor nodes — short bursts, then deep sleep.
Get a peer's MAC with `WiFi.macAddress()` on that device first.

```cpp
#include <WiFi.h>
#include <esp_now.h>
uint8_t peer[] = {0x24,0x6F,0x28,0x00,0x00,0x00};   // replace with real MAC

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {                    // check every esp_err_t
    Serial.println("ESP-NOW init failed"); return;
  }
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, peer, 6);
  if (esp_now_add_peer(&p) != ESP_OK) {
    Serial.println("add_peer failed"); return;
  }
  const char* msg = "hi";
  esp_err_t r = esp_now_send(peer, (const uint8_t*)msg, strlen(msg));
  Serial.printf("send -> %s\n", r == ESP_OK ? "queued" : "error");
}
```

## C3-specific networking notes

- **CPU floor:** keep `setCpuFrequencyMhz()` at **≥ 80** while WiFi is active.
  Dropping to 40/20 MHz with the radio on causes instability.
- **ADC2 clash:** don't `analogRead(GPIO5)` while connected — use GPIO0–4.
- **Brownouts:** WiFi TX current spikes. On flaky USB power you'll see random
  resets with a "brownout detector" message — recommend a solid 5V/USB source or
  a cap across 3V3/GND before blaming the code.
- **Power vs. uptime:** for battery sensors, the winning pattern is *connect →
  send → deep sleep*, not staying associated. See `power.md`.
- **OTA updates** work normally via `ArduinoOTA` or `Update.h`; nothing
  C3-specific beyond fitting within flash/partition sizes (`project-setup.md`).
