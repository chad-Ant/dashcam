/**
 * ES3C28P dashboard demo application.
 *
 * This is intentionally just the page data and application lifecycle. All
 * hardware services live in lib/ so the production dashcam UI can replace
 * these placeholders without touching hardware code.
 */

#include <Arduino.h>
#include "es3Audio.h"
#include "es3Graphics.h"
#include "es3UsbLink.h"

// No onboard LDR. Enable only after connecting a 3.3 V divider to GPIO2;
// measure darkRaw/brightRaw and choose a threshold for that divider.
static constexpr Es3BrightnessConfig kBrightness = {
    false, 2, 3000, 500, 32, 255, 1500, 4, 50
};

static constexpr Es3DashboardPage kDemoPages[] = {
    {"LIVE", "DASHCAM LIVE", "Record:  WAITING", "Speed:   -- km/h", "GPS:     no fix", "Event:   none", false, true, false},
    {"TRIP", "TRIP SUMMARY", "Distance: --.- km", "Drive:    00:00", "Average:  -- km/h", "Score:    -- / 100", false, false, false},
    {"CAM", "CAMERAS", "Road:    standby", "Cabin:   standby", "Lanes:   waiting", "Storage: -- GB free", false, false, false},
    {"SYS", "SYSTEM HEALTH", "Jetson:  offline", "C3 link: waiting", "CPU temp: -- C", "Battery: --.- V", false, false, false},
    {"SET", "QUICK CONTROLS", "Recording: start", "Audio alert: on", kBrightness.enabled ? "Screen:      auto" : "Screen:      manual", "Tap SET tab to calibrate", true, false, true},
    {"MAP", "POSITION MAP", "Waiting for Jetson", "", "", "", false, false, false},
};

static Es3Storage gStorage;
static Es3Graphics gGraphics(gStorage);
static Es3Audio gAudio(gStorage);
static Es3Video gVideo;
static Es3UsbLink gUsb(gGraphics, gAudio, gVideo);

void setup()
{
    Serial0.begin(115200);  // UART diagnostics; USB is reserved for framed traffic.
    delay(200);

    const uint8_t pageCount = static_cast<uint8_t>(sizeof(kDemoPages) / sizeof(kDemoPages[0]));
    const bool storageOk = gStorage.begin();
    Serial0.printf("[es3] storage %s\n", storageOk ? "ready" : "unavailable");
    if (!gGraphics.setPages(kDemoPages, pageCount) ||
        !gGraphics.setBrightnessConfig(kBrightness) || !gGraphics.begin()) {
        Serial0.println("[es3] graphics startup failed");
    }
    if (!gAudio.begin()) {
        Serial0.println("[es3] audio startup failed");
    } else if (gStorage.exists("/alert.wav")) {
        // Optional short boot proof. It remains silent when the test asset is
        // not copied to the card; future notifications call playWav directly.
        (void)gAudio.playWav("/alert.wav");
    }
    if (!gVideo.begin()) Serial0.println("[es3] video unavailable: PSRAM/task allocation failed");
    gGraphics.attachVideo(gVideo);
    if (!gUsb.begin()) Serial0.println("[es3] USB task startup failed");
}

void loop()
{
    gUsb.update();
    gGraphics.update();
    gAudio.update();
    static uint32_t lastReport = 0;
    static es3video::VideoState::Stats previous;
    const uint32_t now = millis(), elapsed = now - lastReport;
    if (elapsed >= 5000 && gVideo.available()) {
        const auto stats = gVideo.stats();
        Serial0.printf("[video] assembled/decoded/presented=%lu/%lu/%lu fps_x10=%lu Bps=%lu jpeg=%lu drop=%lu crc=%lu timeout=%lu decode_fail=%lu decode_us=%lu blit_us=%lu heap=%lu psram=%lu\n",
            (unsigned long)stats.assembled, (unsigned long)stats.decoded, (unsigned long)stats.presented,
            (unsigned long)(uint64_t(stats.presented - previous.presented) * 10000 / elapsed),
            (unsigned long)(uint64_t(stats.receivedBytes - previous.receivedBytes) * 1000 / elapsed),
            (unsigned long)stats.jpegBytes, (unsigned long)stats.dropped,
            (unsigned long)stats.reasons[es3video::ReasonBadCrc],
            (unsigned long)stats.reasons[es3video::ReasonAssemblyTimeout],
            (unsigned long)stats.reasons[es3video::ReasonDecodeFailed],
            (unsigned long)stats.decodeUs, (unsigned long)stats.blitUs,
            (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getFreePsram());
        previous = stats; lastReport = now;
    }
    delay(1);
}
