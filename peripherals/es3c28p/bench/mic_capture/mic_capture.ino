// Microphone bench (MIC_LEVEL_PLAN.md, P1). Replaces the dashboard until the
// main firmware is reflashed. Captures the ES8311 ADC at 16 kHz on I2S0 (DIN
// GPIO6) and shows, per I2S slot, the RMS and peak level (dBFS), DC offset and
// clipped samples on the TFT. A tap anywhere steps through the gain table.
// The amplifier stays disabled and the DAC muted: this sketch never plays.
// A capture task on core 0 does all I2S reads, so drawing cannot starve it.
// USB CDC carries one "W ..." line per 200 ms window and "[tap]" events, and
// accepts "g<step>" (set gain), "p<text>" (show a prompt) and "c" (clear it);
// bench/mic_session.py uses this for a guided run. UART0 prints the same lines.
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <driver/i2s_std.h>
#include <math.h>
#include "es3Board.h"
#include "Es3Panel.h"

namespace {
constexpr bool kRotate180 = true;           // the rig's inverted mounting
constexpr uint32_t kSampleRate = 16000;     // MCLK 256 x Fs = 4.096 MHz
constexpr uint32_t kWindowFrames = kSampleRate / 5;   // 200 ms statistics
constexpr uint32_t kSettleMs = 300;         // ignored after start / gain change
constexpr uint32_t kHoldMs = 3000;          // "max" RMS hold
constexpr uint32_t kDmaDescriptors = 8;
constexpr uint32_t kDmaFrames = 256;        // 8 x 256 frames = 128 ms of buffering

// Total gain = analog PGA (reg 0x14 bits 3..0, 3 dB steps) + ADC scale
// (reg 0x16, 0..7 = 0..42 dB). Index 3 is the driver's PGA 30 dB, scale 0.
struct Gain { uint8_t pga; uint8_t scale; };
constexpr Gain kGains[] = {{0, 0}, {4, 0}, {8, 0}, {10, 0}, {10, 1}, {10, 2}, {10, 3}, {10, 4}};
constexpr uint8_t kGainCount = sizeof(kGains) / sizeof(kGains[0]);
uint8_t gGain = 3;

SPIClass gSpi(FSPI);
Adafruit_ILI9341 gTft(&gSpi, Es3Board::tftDc, Es3Board::tftCs, -1);
i2s_chan_handle_t gRx = nullptr;
volatile uint32_t gOverflows = 0;
portMUX_TYPE gLock = portMUX_INITIALIZER_UNLOCKED;

struct SlotStats {
    int64_t sum = 0;
    int64_t sumSquares = 0;
    int32_t minimum = 32767, maximum = -32768;
    uint32_t clipped = 0;
};
struct SlotResult {
    float rmsDb = -99.9f, peakDb = -99.9f, maxRmsDb = -99.9f;
    int32_t dc = 0;
    uint32_t clipped = 0;
    uint32_t holdUntil = 0;
};
// Capture task state (task only), and what it publishes under gLock.
SlotStats gStats[2];
int64_t gDiffSquares = 0;               // sum of (L - R)^2: are the slots copies?
uint32_t gWindowCount = 0;
volatile uint32_t gSettleUntil = 0;     // written by loop, read by the task
volatile bool gRestart = false;         // loop asks the task to drop the window
struct Window {
    uint32_t sequence = 0;
    double mean[2] = {0, 0}, variance[2] = {0, 0}, peak[2] = {0, 0}, diffRms = 0;
    uint32_t clipped[2] = {0, 0};
};
Window gPublished;                      // guarded by gLock
volatile uint32_t gTotalFrames = 0, gReadErrors = 0;

// Loop state.
SlotResult gResult[2];
float gDiffDb = -99.9f;
uint32_t gSeenSequence = 0, gDrawMaxUs = 0, gDrawLastUs = 0;
bool gRedraw = true;
char gPrompt[27] = "";
bool gPromptDirty = true;

bool IRAM_ATTR onOverflow(i2s_chan_handle_t, i2s_event_data_t*, void*)
{
    gOverflows = gOverflows + 1;
    return false;
}

bool codecWrite(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(Es3Board::codecAddress);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool applyGain()
{
    const Gain& g = kGains[gGain];
    return codecWrite(0x14, static_cast<uint8_t>(0x10 | g.pga)) && codecWrite(0x16, g.scale);
}

// Playback's configureCodec() sequence with the capture differences from
// MIC_LEVEL_PLAN.md: ADC serial output on (0x0A), ADC volume 0 dB (0x17), DAC
// input muted (0x09) and powered down (0x12), DAC volume 0 (0x32).
bool configureCodecForCapture()
{
    (void)codecWrite(0x44, 0x08);
    if (!codecWrite(0x44, 0x08)) return false;
    const Gain& g = kGains[gGain];
    const uint8_t registers[][2] = {
        {0x01, 0x30}, {0x02, 0x00}, {0x03, 0x10}, {0x16, g.scale},
        {0x04, 0x20}, {0x05, 0x00}, {0x0b, 0x00}, {0x0c, 0x00},
        {0x10, 0x1f}, {0x11, 0x7f}, {0x00, 0x80}, {0x01, 0x3f},
        {0x06, 0x03}, {0x07, 0x00}, {0x08, 0xff},
        {0x09, 0x4c}, {0x0a, 0x0c},  // DAC input muted, ADC serial output on
        {0x13, 0x10}, {0x1b, 0x0a}, {0x1c, 0x6a},
        {0x0e, 0x02}, {0x12, 0x02}, {0x14, static_cast<uint8_t>(0x10 | g.pga)}, {0x0d, 0x01},
        {0x15, 0x40}, {0x37, 0x08}, {0x45, 0x00},
        {0x17, 0xbf}, {0x31, 0x60}, {0x32, 0x00},
    };
    for (const auto& entry : registers) {
        if (!codecWrite(entry[0], entry[1])) return false;
    }
    return true;
}

bool openCapture()
{
    i2s_chan_config_t channel = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    channel.dma_desc_num = kDmaDescriptors;
    channel.dma_frame_num = kDmaFrames;
    if (i2s_new_channel(&channel, nullptr, &gRx) != ESP_OK) return false;
    i2s_std_config_t config{};
    config.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(kSampleRate);
    config.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
    config.gpio_cfg.mclk = static_cast<gpio_num_t>(Es3Board::audioMclk);
    config.gpio_cfg.bclk = static_cast<gpio_num_t>(Es3Board::audioBclk);
    config.gpio_cfg.ws = static_cast<gpio_num_t>(Es3Board::audioWs);
    config.gpio_cfg.dout = I2S_GPIO_UNUSED;
    config.gpio_cfg.din = GPIO_NUM_6;   // ES8311 ASDOUT (schematic net I2S_DO)
    i2s_event_callbacks_t callbacks{};
    callbacks.on_recv_q_ovf = onOverflow;
    if (i2s_channel_init_std_mode(gRx, &config) != ESP_OK ||
        i2s_channel_register_event_callback(gRx, &callbacks, nullptr) != ESP_OK ||
        i2s_channel_enable(gRx) != ESP_OK) {
        i2s_del_channel(gRx);
        gRx = nullptr;
        return false;
    }
    return configureCodecForCapture();   // codec needs MCLK running first
}

float toDb(double amplitude)
{
    return amplitude < 1.0 ? -99.9f : static_cast<float>(20.0 * log10(amplitude / 32768.0));
}

void resetWindow()
{
    for (SlotStats& st : gStats) st = SlotStats{};
    gDiffSquares = 0;
    gWindowCount = 0;
}

void publishWindow()
{
    Window w;
    for (int slot = 0; slot < 2; ++slot) {
        const SlotStats& st = gStats[slot];
        w.mean[slot] = static_cast<double>(st.sum) / gWindowCount;
        w.variance[slot] = static_cast<double>(st.sumSquares) / gWindowCount - w.mean[slot] * w.mean[slot];
        w.peak[slot] = fmax(fabs(st.maximum - w.mean[slot]), fabs(st.minimum - w.mean[slot]));
        w.clipped[slot] = st.clipped;
    }
    w.diffRms = sqrt(static_cast<double>(gDiffSquares) / gWindowCount);
    portENTER_CRITICAL(&gLock);
    w.sequence = gPublished.sequence + 1;
    gPublished = w;
    portEXIT_CRITICAL(&gLock);
    resetWindow();
}

void captureTask(void*)
{
    static int16_t frames[256 * 2];
    for (;;) {
        size_t bytes = 0;
        const esp_err_t result = i2s_channel_read(gRx, frames, sizeof(frames), &bytes, pdMS_TO_TICKS(100));
        if (result != ESP_OK && result != ESP_ERR_TIMEOUT) { gReadErrors = gReadErrors + 1; continue; }
        const size_t count = bytes / (2 * sizeof(int16_t));
        gTotalFrames = gTotalFrames + count;
        if (gRestart) { gRestart = false; resetWindow(); }
        if (static_cast<int32_t>(millis() - gSettleUntil) < 0) continue;
        for (size_t i = 0; i < count; ++i) {
            const int32_t left = frames[i * 2], right = frames[i * 2 + 1];
            for (int slot = 0; slot < 2; ++slot) {
                const int32_t x = slot == 0 ? left : right;
                SlotStats& st = gStats[slot];
                st.sum += x;
                st.sumSquares += static_cast<int64_t>(x) * x;
                if (x < st.minimum) st.minimum = x;
                if (x > st.maximum) st.maximum = x;
                if (x >= 32767 || x <= -32768) ++st.clipped;
            }
            gDiffSquares += static_cast<int64_t>(left - right) * (left - right);
            if (++gWindowCount == kWindowFrames) publishWindow();
        }
    }
}

// Loop side: take a newly published window, update results, report it.
void takeWindow()
{
    Window w;
    portENTER_CRITICAL(&gLock);
    w = gPublished;
    portEXIT_CRITICAL(&gLock);
    if (w.sequence == gSeenSequence) return;
    gSeenSequence = w.sequence;
    const uint32_t now = millis();
    for (int slot = 0; slot < 2; ++slot) {
        SlotResult& r = gResult[slot];
        r.dc = static_cast<int32_t>(lround(w.mean[slot]));
        r.rmsDb = toDb(sqrt(w.variance[slot] > 0 ? w.variance[slot] : 0));
        r.peakDb = toDb(w.peak[slot]);
        r.clipped += w.clipped[slot];
        if (r.rmsDb >= r.maxRmsDb || static_cast<int32_t>(now - r.holdUntil) >= 0) {
            r.maxRmsDb = r.rmsDb;
            r.holdUntil = now + kHoldMs;
        }
    }
    gDiffDb = toDb(w.diffRms);
    char line[200];
    snprintf(line, sizeof(line),
             "W %lu g%u %u L %.1f %.1f %ld %lu R %.1f %.1f %ld %lu D %.1f ovf %lu err %lu draw %lu %lu\n",
             static_cast<unsigned long>(w.sequence), gGain, kGains[gGain].pga * 3 + kGains[gGain].scale * 6,
             gResult[0].rmsDb, gResult[0].peakDb, static_cast<long>(gResult[0].dc),
             static_cast<unsigned long>(w.clipped[0]),
             gResult[1].rmsDb, gResult[1].peakDb, static_cast<long>(gResult[1].dc),
             static_cast<unsigned long>(w.clipped[1]), gDiffDb,
             static_cast<unsigned long>(gOverflows), static_cast<unsigned long>(gReadErrors),
             static_cast<unsigned long>(gDrawLastUs), static_cast<unsigned long>(gDrawMaxUs));
    Serial.print(line);
    Serial0.print(line);
    gRedraw = true;
}

bool touchPressed()
{
    Wire.beginTransmission(Es3Board::touchAddress);
    Wire.write(0x02);
    if (Wire.endTransmission(false) != 0 || Wire.requestFrom(Es3Board::touchAddress, uint8_t{3}) != 3) return false;
    const uint8_t count = Wire.read() & 0x0f;
    const uint8_t event = Wire.read() >> 6;
    (void)Wire.read();
    return count >= 1 && count <= 2 && event != 1 && event != 3;
}

void startSettle()
{
    gSettleUntil = millis() + kSettleMs;
    gRestart = true;
    for (SlotResult& r : gResult) r = SlotResult{};
}

void setGain(uint8_t step)
{
    gGain = static_cast<uint8_t>(step % kGainCount);
    if (!applyGain()) gReadErrors = gReadErrors + 1;
    startSettle();
    gRedraw = true;
}

void drawPrompt()
{
    gTft.fillRect(0, 196, 320, 28, gPrompt[0] ? ILI9341_BLUE : ILI9341_BLACK);
    gTft.setTextSize(gPrompt[0] ? 2 : 1);
    gTft.setTextColor(gPrompt[0] ? ILI9341_YELLOW : ILI9341_LIGHTGREY);
    if (gPrompt[0]) {
        gTft.setCursor((320 - static_cast<int>(strlen(gPrompt)) * 12) / 2, 202);
        gTft.print(gPrompt);
    } else {
        gTft.setCursor(6, 200);
        gTft.print("Read: quiet room, speech at 30 cm, a clap.");
        gTft.setCursor(6, 212);
        gTft.print("Tap anywhere for the next gain step.");
    }
}

// USB commands, one per line: g<step>, p<prompt text>, c.
void pollCommands()
{
    static char command[40];
    static size_t length = 0;
    while (Serial.available() > 0) {
        const int c = Serial.read();
        if (c != '\n' && c != '\r') {
            if (length + 1 < sizeof(command)) command[length++] = static_cast<char>(c);
            continue;
        }
        command[length] = '\0';
        if (command[0] == 'g' && command[1] >= '0' && command[1] <= '9') {
            setGain(static_cast<uint8_t>(atoi(command + 1)));
        } else if (command[0] == 'p') {
            snprintf(gPrompt, sizeof(gPrompt), "%.26s", command + 1);  // 26 chars fit the panel
            gPromptDirty = true;
        } else if (command[0] == 'c') {
            gPrompt[0] = '\0';
            gPromptDirty = true;
        }
        length = 0;
    }
}

void drawStatic()
{
    gTft.fillScreen(ILI9341_BLACK);
    gTft.fillRect(0, 0, 320, 22, ILI9341_DARKCYAN);
    gTft.setTextSize(2);
    gTft.setTextColor(ILI9341_WHITE);
    gTft.setCursor(6, 4);
    gTft.print("MIC BENCH 16 kHz");
}

void drawSlot(int slot, int top)
{
    const SlotResult& r = gResult[slot];
    gTft.setTextSize(2);
    gTft.setTextColor(slot == 0 ? ILI9341_CYAN : ILI9341_ORANGE, ILI9341_BLACK);
    gTft.setCursor(6, top);
    gTft.printf("%c RMS %6.1f dBFS ", slot == 0 ? 'L' : 'R', r.rmsDb);
    gTft.setTextSize(1);
    gTft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    gTft.setCursor(6, top + 20);
    gTft.printf("peak %6.1f  max3s %6.1f  DC %6ld  clip %lu   ",
                r.peakDb, r.maxRmsDb, static_cast<long>(r.dc), static_cast<unsigned long>(r.clipped));
    // Bar: -90..0 dBFS over 308 px; white tick = 3 s max.
    const auto x = [](float db) { return static_cast<int>(constrain((db + 90.0f) * 308.0f / 90.0f, 0.0f, 308.0f)); };
    const int barTop = top + 32;
    gTft.fillRect(6, barTop, 308, 12, ILI9341_DARKGREY);
    gTft.fillRect(6, barTop, x(r.rmsDb), 12, slot == 0 ? ILI9341_CYAN : ILI9341_ORANGE);
    gTft.drawFastVLine(6 + x(r.maxRmsDb), barTop - 2, 16, ILI9341_WHITE);
}

void drawDynamic()
{
    const Gain& g = kGains[gGain];
    gTft.setTextSize(2);
    gTft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
    gTft.setCursor(6, 30);
    gTft.printf("GAIN %2u dB  %u/%u   ", g.pga * 3 + g.scale * 6, gGain + 1, kGainCount);
    gTft.setTextSize(1);
    gTft.setCursor(6, 50);
    gTft.setTextColor(ILI9341_LIGHTGREY, ILI9341_BLACK);
    gTft.printf("PGA %2u dB (0x14=%02X)  ADC scale %2u dB (0x16=%02X)  ",
                g.pga * 3, 0x10 | g.pga, g.scale * 6, g.scale);
    drawSlot(0, 66);
    drawSlot(1, 120);
    gTft.setTextSize(1);
    gTft.setTextColor(static_cast<int32_t>(millis() - gSettleUntil) < 0 ? ILI9341_YELLOW : ILI9341_LIGHTGREY,
                      ILI9341_BLACK);
    gTft.setCursor(6, 174);
    gTft.printf("frames %-9lu ovf %-5lu err %-4lu %s   ", static_cast<unsigned long>(gTotalFrames),
                static_cast<unsigned long>(gOverflows), static_cast<unsigned long>(gReadErrors),
                static_cast<int32_t>(millis() - gSettleUntil) < 0 ? "settling" : "        ");
    gTft.setCursor(6, 184);
    gTft.printf("L-R %6.1f dBFS  draw %3lu ms (max %3lu)   ", gDiffDb,
                static_cast<unsigned long>(gDrawLastUs / 1000), static_cast<unsigned long>(gDrawMaxUs / 1000));
}
}  // namespace

void setup()
{
    Serial0.begin(115200);
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);   // never block when no host reads the USB port
    pinMode(Es3Board::amplifierDisable, OUTPUT);
    digitalWrite(Es3Board::amplifierDisable, HIGH);   // never drive the speaker
    gSpi.begin(Es3Board::tftClock, Es3Board::tftMiso, Es3Board::tftMosi, Es3Board::tftCs);
    gTft.begin(40000000);
    es3ConfigureIpsPanel(gTft, kRotate180);
    pinMode(Es3Board::backlight, OUTPUT);
    digitalWrite(Es3Board::backlight, HIGH);
    drawStatic();

    Es3Board::beginI2c();
    pinMode(Es3Board::touchReset, OUTPUT);
    digitalWrite(Es3Board::touchReset, LOW);
    delay(10);
    digitalWrite(Es3Board::touchReset, HIGH);
    delay(200);

    if (!openCapture()) {
        gTft.setTextSize(2);
        gTft.setTextColor(ILI9341_RED, ILI9341_BLACK);
        gTft.setCursor(6, 100);
        gTft.print("CAPTURE START FAILED");
        Serial0.println("[mic] capture start failed (I2S or codec I2C)");
        gRx = nullptr;
        return;
    }
    Serial0.println("[mic] capturing 16 kHz, 16-bit stereo slots, DIN GPIO6");
    startSettle();
    xTaskCreatePinnedToCore(captureTask, "mic", 4096, nullptr, 5, nullptr, 0);
}

void loop()
{
    if (gRx == nullptr) { delay(1000); return; }
    pollCommands();
    takeWindow();

    static uint32_t lastTouchPoll = 0, lastDraw = 0;
    static bool held = false;
    const uint32_t now = millis();
    if (now - lastTouchPoll >= 20) {
        lastTouchPoll = now;
        const bool pressed = touchPressed();
        if (pressed && !held) {
            Serial.print("[tap]\n");
            setGain(static_cast<uint8_t>(gGain + 1));
        }
        held = pressed;
    }
    if (gPromptDirty) {
        gPromptDirty = false;
        drawPrompt();
    }
    if (gRedraw && now - lastDraw >= 200) {
        lastDraw = now;
        gRedraw = false;
        const uint32_t started = micros();
        drawDynamic();
        gDrawLastUs = micros() - started;
        if (gDrawLastUs > gDrawMaxUs) gDrawMaxUs = gDrawLastUs;
    }
}
