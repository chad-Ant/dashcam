/**
 * ESP_Sentinel — telemetry receiver demo.
 *
 * Requests a 10 Hz telemetry stream from the MKR Zero master over UART and
 * prints each decoded OBD2+GPS snapshot to the USB-CDC console. Re-requests the
 * stream if the link goes quiet (e.g. the master reset).
 *
 * Wiring (cross-over + common ground):
 *   MKR Zero Serial1 TX (pin 14) -> C3 RX (GPIO20)
 *   MKR Zero Serial1 RX (pin 13) <- C3 TX (GPIO21)
 *   GND <-> GND
 * Board setting: USB CDC On Boot = Enabled (native USB, so Serial is the console
 * and UART0's GPIO20/21 are free for Serial1).
 */

#include <Arduino.h>
#include "commLink.h"

static CommLink   gLink;  // not 'link' — collides with POSIX link() from <unistd.h>
static uint32_t   lastHealthMs = 0;

void setup()
{
    Serial.begin(115200);
    delay(100); // let native USB-CDC enumerate before the first prints

    Serial.println("ESP_Sentinel telemetry receiver");
    gLink.begin(115200, COMMLINK_RX_PIN, COMMLINK_TX_PIN);
    if (gLink.startStream()) Serial.println("Requested 10 Hz telemetry stream from master.");
    else                     Serial.println("startStream() TX busy; will retry on the stale-link check.");
}

void loop()
{
    if (gLink.poll()) {
        const TelemetryPayload &t = gLink.latest();
        Serial.printf(
            "t=%lu ms  spd=%.1f km/h  rpm=%.0f  temp=%.0fC  fuel=%.0f%%  "
            "lat=%.6f lon=%.6f alt=%.1f m  sats=%u fix=%u  "
            "%04u-%02u-%02u %02u:%02u:%02uZ  flags=0x%02X\n",
            (unsigned long)t.masterMillis, t.speed, t.rpm, t.coolantTemp, t.fuelLevel,
            (double)t.latitude, (double)t.longitude, (double)t.altitude,
            t.satellites, t.fixType,
            t.year, t.month, t.day, t.hour, t.minute, t.second, t.flags);
    }

    // Link health: if the stream has been silent for >1 s, re-request it.
    if (millis() - lastHealthMs >= 1000UL) {
        lastHealthMs = millis();
        if (gLink.isStale(1000)) {
            Serial.println("[stale] no telemetry in 1 s; re-requesting stream");
            (void)gLink.startStream();   // best-effort; this stale-check path is itself the retry
        }
    }
}
