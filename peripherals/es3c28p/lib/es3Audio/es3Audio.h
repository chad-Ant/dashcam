/** @file es3Audio.h - ES8311 / I2S0 owner: SD WAV playback and microphone capture. */
#pragma once
#include <Arduino.h>
#include <atomic>
#include <driver/i2s_std.h>
#include "es3Storage.h"
#include "MicLevel.h"

// PCM mono unsigned 8-bit WAV, 8--16 kHz. All calls belong to the loop task.
// Playback and microphone capture take turns on I2S0 (MIC_LEVEL_PLAN.md):
// playWav() stops capture first; startCapture() refuses while a sound plays.
class Es3Audio {
public:
    explicit Es3Audio(Es3Storage& storage);

    /** Checks that the ES8311 and I2S channel and its DMA can be allocated. */
    bool begin();

    /** Starts non-blocking playback of a supported WAV file on the SD card. */
    bool playWav(const char* path);

    /** Supplies the next small audio block. Call every loop() iteration. */
    void update();

    /** Stops playback immediately and releases the SD file. */
    void stop();

    /** Adjusts software volume from silent (0) to full scale (100). */
    void setVolume(uint8_t percent);

    /** User master volume (0..100). Scales every playback, including alerts
     * requested over USB; takes effect from the next streamed block. */
    void setMasterVolume(uint8_t percent);

    /**
     * Starts microphone capture (16 kHz, left slot, gain 54 dB) and a capture
     * task on core 0 that keeps micLevel() current. False while playing.
     */
    bool startCapture();

    /** Stops the capture task (waits for it), then releases I2S0. */
    void stopCapture();

    bool capturing() const;
    /** 0..100, or es3audio::kMicLevelNone while settling / not capturing. */
    int micLevel() const;
    /** Receive DMA overflows (dropped capture blocks) since boot. */
    uint32_t captureOverflows() const;

    bool ready() const;
    bool isPlaying() const;
    uint8_t volume() const;
    uint8_t masterVolume() const;

private:
    static constexpr size_t kStreamBlockSamples = 128;

    Es3Storage& _storage;
    File _file;
    i2s_chan_handle_t _tx;
    bool _ready;
    bool _playing;
    uint8_t _volume;
    uint8_t _masterVolume;
    uint32_t _sampleRate;
    uint32_t _dataRemaining;
    uint32_t _drainDeadline;
    bool _draining;
    int16_t _pcm[kStreamBlockSamples * 2];
    size_t _pcmLength;
    size_t _pcmOffset;

    // Capture: the task owns _rx reads and _meter while _captureRunning.
    i2s_chan_handle_t _rx = nullptr;
    TaskHandle_t _captureTask = nullptr;
    es3audio::MicLevel _meter;
    std::atomic<bool> _captureStop{false};
    std::atomic<bool> _captureRunning{false};
    std::atomic<int> _micLevel{es3audio::kMicLevelNone};
    std::atomic<uint32_t> _captureOverflows{0};

    bool openOutput(uint32_t sampleRate);
    void closeOutput();
    bool readBlock();
    static void captureTask(void* self);
    static bool IRAM_ATTR onCaptureOverflow(i2s_chan_handle_t, i2s_event_data_t*, void* self);
};
