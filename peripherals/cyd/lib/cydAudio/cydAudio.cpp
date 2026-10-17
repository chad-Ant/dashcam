#include "CydLog.h"
/** @file cydAudio.cpp */

#include "cydAudio.h"

#include <cstring>
#include <esp_log.h>

namespace {
constexpr int kSpeakerDacPin = 26;  // DAC channel 1
constexpr uint32_t kMinimumSampleRate = 8000;
constexpr uint32_t kMaximumSampleRate = 16000;
// Eight 256-byte DMA buffers queue 64--256 ms of audio, enough to ride out a
// splash redraw; update() tops the queue up by at most four blocks per call.
constexpr uint32_t kDmaBufferCount = 8;
constexpr size_t kDmaBufferBytes = 256;
constexpr uint8_t kBlocksPerUpdate = 4;
}  // namespace

CydAudio::CydAudio(CydStorage& storage)
    : _storage(storage)
    , _dac(nullptr)
    , _ready(false)
    , _playing(false)
    , _volume(80)
    , _sampleRate(0)
    , _dataRemaining(0)
    , _pcm{}
    , _pcmLength(0)
    , _pcmOffset(0)
{
}

bool CydAudio::begin()
{
    if (_ready) {
        return true;
    }
    // Prove the DAC channel and DMA can be allocated, then release them: each
    // file opens the channel at its own sample rate.
    if (!openDac(kMinimumSampleRate)) {
        return false;
    }
    closeDac();
    // update() writes with a zero timeout, and the driver logs every full
    // queue as an error. Real write failures are reported by update().
    esp_log_level_set("dac_continuous", ESP_LOG_NONE);
    _ready = true;
    cydLog("[cyd] audio ready: GPIO%d DAC\n", kSpeakerDacPin);
    return true;
}

bool CydAudio::openDac(uint32_t sampleRate)
{
    dac_continuous_config_t config{};
    config.chan_mask = DAC_CHANNEL_MASK_CH1;
    config.desc_num = kDmaBufferCount;
    config.buf_size = kDmaBufferBytes;
    config.freq_hz = sampleRate;
    config.offset = 0;
    // The default PLL_D2 clock cannot go below 19.6 kHz on the ESP32; the
    // APLL reaches 648 Hz, which covers the 8--16 kHz notification range.
    config.clk_src = DAC_DIGI_CLK_SRC_APLL;
    config.chan_mode = DAC_CHANNEL_MODE_SIMUL;
    if (dac_continuous_new_channels(&config, &_dac) != ESP_OK) {
        _dac = nullptr;
        return false;
    }
    if (dac_continuous_enable(_dac) != ESP_OK) {
        dac_continuous_del_channels(_dac);
        _dac = nullptr;
        return false;
    }
    return true;
}

void CydAudio::closeDac()
{
    if (_dac == nullptr) {
        return;
    }
    dac_continuous_disable(_dac);
    dac_continuous_del_channels(_dac);
    _dac = nullptr;
}

bool CydAudio::readExact(uint8_t* destination, size_t count)
{
    return _file.read(destination, count) == count;
}

bool CydAudio::readU16(uint16_t& value)
{
    uint8_t bytes[2]{};
    if (!readExact(bytes, sizeof(bytes))) {
        return false;
    }
    value = static_cast<uint16_t>(bytes[0]) |
            static_cast<uint16_t>(bytes[1]) << 8;
    return true;
}

bool CydAudio::readU32(uint32_t& value)
{
    uint8_t bytes[4]{};
    if (!readExact(bytes, sizeof(bytes))) {
        return false;
    }
    value = static_cast<uint32_t>(bytes[0]) |
            static_cast<uint32_t>(bytes[1]) << 8 |
            static_cast<uint32_t>(bytes[2]) << 16 |
            static_cast<uint32_t>(bytes[3]) << 24;
    return true;
}

bool CydAudio::parseWavHeader()
{
    uint8_t riff[4]{};
    uint32_t riffSize = 0;
    uint8_t wave[4]{};
    if (!readExact(riff, sizeof(riff)) || !readU32(riffSize) || !readExact(wave, sizeof(wave)) ||
        std::memcmp(riff, "RIFF", 4) != 0 || std::memcmp(wave, "WAVE", 4) != 0) {
        return false;
    }

    bool foundFormat = false;
    while (_file.available()) {
        uint8_t chunkId[4]{};
        uint32_t chunkSize = 0;
        if (!readExact(chunkId, sizeof(chunkId)) || !readU32(chunkSize)) {
            return false;
        }
        const uint32_t chunkStart = _file.position();
        if (std::memcmp(chunkId, "fmt ", 4) == 0) {
            uint16_t format = 0;
            uint16_t channels = 0;
            uint32_t sampleRate = 0;
            uint32_t byteRate = 0;
            uint16_t blockAlign = 0;
            uint16_t bitsPerSample = 0;
            if (chunkSize < 16 || !readU16(format) || !readU16(channels) ||
                !readU32(sampleRate) || !readU32(byteRate) || !readU16(blockAlign) ||
                !readU16(bitsPerSample) || format != 1 || channels != 1 ||
                bitsPerSample != 8 || blockAlign != 1 || byteRate != sampleRate ||
                sampleRate < kMinimumSampleRate || sampleRate > kMaximumSampleRate) {
                return false;
            }
            _sampleRate = sampleRate;
            foundFormat = true;
        } else if (std::memcmp(chunkId, "data", 4) == 0) {
            if (!foundFormat || chunkStart + chunkSize > _file.size()) {
                return false;
            }
            _dataRemaining = chunkSize;
            return true;
        }

        const uint32_t nextChunk = chunkStart + chunkSize + (chunkSize & 1U);
        if (nextChunk > _file.size() || !_file.seek(nextChunk)) {
            return false;
        }
    }
    return false;
}

bool CydAudio::playWav(const char* path)
{
    if (!_ready || path == nullptr) {
        return false;
    }
    stop();
    if (!_storage.openRead(path, _file) || !parseWavHeader() || !openDac(_sampleRate)) {
        cydLog("[cyd] audio unsupported or unavailable: %s\n", path);
        _file.close();
        return false;
    }
    _playing = _dataRemaining > 0;
    cydLog("[cyd] audio playing: %s\n", path);
    return _playing;
}

bool CydAudio::readBlock()
{
    const size_t requested = min<size_t>(kStreamBlockSamples, _dataRemaining);
    const size_t read = _file.read(_pcm, requested);
    if (read == 0) {
        return false;
    }
    _dataRemaining -= read;
    for (size_t index = 0; index < read; ++index) {
        const int centered = static_cast<int>(_pcm[index]) - 128;
        const int scaled = 128 + centered * _volume / 100;
        _pcm[index] = static_cast<uint8_t>(constrain(scaled, 0, 255));
    }
    _pcmLength = read;
    _pcmOffset = 0;
    return true;
}

void CydAudio::update()
{
    if (!_playing) {
        return;
    }
    for (uint8_t block = 0; block < kBlocksPerUpdate; ++block) {
        if (_pcmOffset == _pcmLength) {
            if (_dataRemaining == 0) {
                // Everything is queued. The channel stays enabled so the queued
                // tail plays out; the DAC then holds the last sample.
                _playing = false;
                _file.close();
                return;
            }
            if (!readBlock()) {
                stop();
                return;
            }
        }
        // A zero timeout loads only what fits; when the DMA queue is full the
        // rest of the block waits for the next call instead of blocking the UI.
        size_t loaded = 0;
        const esp_err_t result = dac_continuous_write(_dac, _pcm + _pcmOffset,
                                                      _pcmLength - _pcmOffset, &loaded, 0);
        _pcmOffset += loaded;
        if (result == ESP_ERR_TIMEOUT) {
            return;
        }
        if (result != ESP_OK) {
            cydLog("[cyd] audio DMA write failed");
            stop();
            return;
        }
    }
}

void CydAudio::stop()
{
    _playing = false;
    _dataRemaining = 0;
    _pcmLength = 0;
    _pcmOffset = 0;
    if (_file) {
        _file.close();
    }
    // Deleting the channel drops anything still queued, so the sound ends now.
    closeDac();
}

void CydAudio::setVolume(uint8_t percent)
{
    _volume = min<uint8_t>(percent, 100);
}

bool CydAudio::ready() const { return _ready; }
bool CydAudio::isPlaying() const { return _playing; }
uint8_t CydAudio::volume() const { return _volume; }
