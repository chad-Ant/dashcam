/** @file es3Audio.h - Bounded SD WAV streaming through ES8311 / I2S DMA. */
#pragma once
#include <Arduino.h>
#include <driver/i2s_std.h>
#include "es3Storage.h"

// PCM mono unsigned 8-bit WAV, 8--16 kHz. All calls belong to the loop task.
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

    bool ready() const;
    bool isPlaying() const;
    uint8_t volume() const;

private:
    static constexpr size_t kStreamBlockSamples = 128;

    Es3Storage& _storage;
    File _file;
    i2s_chan_handle_t _tx;
    bool _ready;
    bool _playing;
    uint8_t _volume;
    uint32_t _sampleRate;
    uint32_t _dataRemaining;
    uint32_t _drainDeadline;
    bool _draining;
    int16_t _pcm[kStreamBlockSamples * 2];
    size_t _pcmLength;
    size_t _pcmOffset;

    bool openOutput(uint32_t sampleRate);
    void closeOutput();
    bool readBlock();
    bool parseWavHeader();
    bool readU16(uint16_t& value);
    bool readU32(uint32_t& value);
    bool readExact(uint8_t* destination, size_t count);
};
