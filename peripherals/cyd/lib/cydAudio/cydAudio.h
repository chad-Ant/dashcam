/**
 * @file cydAudio.h
 * @brief Low-overhead SD WAV notification playback for the ESP32-2432S028.
 */

#pragma once

#include <Arduino.h>
#include <driver/dac_continuous.h>
#include "cydStorage.h"

/**
 * Streams notification sounds through the CYD's GPIO26 DAC and speaker amp.
 *
 * Supported media: RIFF/WAVE, PCM, mono, unsigned 8-bit, 8--16 kHz.
 * This deliberately avoids a compressed-audio decoder: SD bytes are read and
 * passed unchanged (apart from volume) to the DAC continuous-mode DMA driver.
 *
 * It must not use the legacy driver/i2s.h DAC mode: that links ESP-IDF's
 * legacy ADC driver, which aborts at boot when analogRead() (the new ADC
 * driver, used for the LDR) is linked too.
 */
class CydAudio {
public:
    explicit CydAudio(CydStorage& storage);

    /** Checks that the GPIO26 DAC channel and its DMA can be allocated. */
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

    CydStorage& _storage;
    File _file;
    dac_continuous_handle_t _dac;
    bool _ready;
    bool _playing;
    uint8_t _volume;
    uint32_t _sampleRate;
    uint32_t _dataRemaining;
    uint8_t _pcm[kStreamBlockSamples];
    size_t _pcmLength;
    size_t _pcmOffset;

    bool openDac(uint32_t sampleRate);
    void closeDac();
    bool readBlock();
    bool parseWavHeader();
    bool readU16(uint16_t& value);
    bool readU32(uint32_t& value);
    bool readExact(uint8_t* destination, size_t count);
};
