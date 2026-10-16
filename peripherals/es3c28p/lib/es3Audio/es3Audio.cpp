/** @file es3Audio.cpp */

#include "es3Audio.h"
#include "WavHeader.h"

#include <cstring>
#include "es3Board.h"

namespace {
constexpr uint32_t kMinimumSampleRate = 8000;
constexpr uint32_t kDmaBufferCount = 8;
constexpr uint32_t kDmaFrames = 128;
constexpr uint8_t kBlocksPerUpdate = 4;

bool codecWrite(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(Es3Board::codecAddress);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool configureCodec()
{
    // Slave, external MCLK = 256 * Fs, normal polarity, 16-bit Philips I2S.
    // The 8--16 kHz coefficients all use prediv=1, mult=1, DAC OSR=32.
    // Register meanings/sequence: Espressif esp_codec_dev ES8311 driver.
    // A first write may fail at power-on; repeat for I2C noise immunity.
    (void)codecWrite(0x44, 0x08);
    if (!codecWrite(0x44, 0x08)) {
        return false;
    }
    const uint8_t registers[][2] = {
        {0x01, 0x30}, {0x02, 0x00}, {0x03, 0x10}, {0x16, 0x24},
        {0x04, 0x20}, {0x05, 0x00}, {0x0b, 0x00}, {0x0c, 0x00},
        {0x10, 0x1f}, {0x11, 0x7f}, {0x00, 0x80}, {0x01, 0x3f},
        {0x06, 0x03}, {0x07, 0x00}, {0x08, 0xff},
        {0x09, 0x0c}, {0x0a, 0x4c},  // DAC enabled, ADC serial out disabled
        {0x13, 0x10}, {0x1b, 0x0a}, {0x1c, 0x6a},
        {0x0e, 0x02}, {0x12, 0x00}, {0x14, 0x1a}, {0x0d, 0x01},
        {0x15, 0x40}, {0x37, 0x08}, {0x45, 0x00},
        {0x31, 0x00}, {0x32, 0xbf},  // unmute; 0 dB (software volume below)
    };
    for (const auto& entry : registers) {
        if (!codecWrite(entry[0], entry[1])) {
            return false;
        }
    }
    return true;
}
}  // namespace

Es3Audio::Es3Audio(Es3Storage& storage)
    : _storage(storage)
    , _tx(nullptr)
    , _ready(false)
    , _playing(false)
    , _volume(80)
    , _sampleRate(0)
    , _dataRemaining(0)
    , _drainDeadline(0)
    , _draining(false)
    , _pcm{}
    , _pcmLength(0)
    , _pcmOffset(0)
{
}

bool Es3Audio::begin()
{
    if (_ready) {
        return true;
    }
    digitalWrite(Es3Board::amplifierDisable, HIGH);
    pinMode(Es3Board::amplifierDisable, OUTPUT);
    if (!Es3Board::beginI2c() || !openOutput(kMinimumSampleRate)) {
        return false;
    }
    closeOutput();
    _ready = true;
    Serial0.println("[es3] audio ready: ES8311 I2S");
    return true;
}

bool Es3Audio::openOutput(uint32_t sampleRate)
{
    i2s_chan_config_t channel = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    channel.dma_desc_num = kDmaBufferCount;
    channel.dma_frame_num = kDmaFrames;
    channel.auto_clear = true;  // underruns output silence, never repeat stale DMA
    if (i2s_new_channel(&channel, &_tx, nullptr) != ESP_OK) {
        _tx = nullptr;
        return false;
    }
    i2s_std_config_t config{};
    config.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sampleRate);
    config.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO);
    config.gpio_cfg.mclk = static_cast<gpio_num_t>(Es3Board::audioMclk);
    config.gpio_cfg.bclk = static_cast<gpio_num_t>(Es3Board::audioBclk);
    config.gpio_cfg.ws = static_cast<gpio_num_t>(Es3Board::audioWs);
    config.gpio_cfg.dout = static_cast<gpio_num_t>(Es3Board::audioOut);
    config.gpio_cfg.din = I2S_GPIO_UNUSED;
    if (i2s_channel_init_std_mode(_tx, &config) != ESP_OK ||
        i2s_channel_enable(_tx) != ESP_OK) {
        i2s_del_channel(_tx);
        _tx = nullptr;
        return false;
    }
    if (!configureCodec()) {
        closeOutput();
        return false;
    }
    return true;
}

void Es3Audio::closeOutput()
{
    digitalWrite(Es3Board::amplifierDisable, HIGH);
    if (_tx != nullptr) {
        (void)codecWrite(0x31, 0x60);
        i2s_channel_disable(_tx);
        i2s_del_channel(_tx);
        _tx = nullptr;
    }
}

bool Es3Audio::playWav(const char* path)
{
    if (!_ready || path == nullptr) {
        return false;
    }
    stop();
    es3audio::WavHeader<File> header(_file);
    if (!_storage.openRead(path, _file) || !header.parse() || !openOutput(header.sampleRate())) {
        Serial0.printf("[es3] audio unsupported or unavailable: %s\n", path);
        stop();
        return false;
    }
    _sampleRate = header.sampleRate();
    _dataRemaining = header.dataBytes();
    _playing = _dataRemaining > 0;
    digitalWrite(Es3Board::amplifierDisable, LOW);
    Serial0.printf("[es3] audio playing: %s\n", path);
    return _playing;
}

bool Es3Audio::readBlock()
{
    const size_t requested = min<size_t>(kStreamBlockSamples, _dataRemaining);
    uint8_t input[kStreamBlockSamples];
    const size_t read = _file.read(input, requested);
    if (read != requested) {
        return false;
    }
    _dataRemaining -= read;
    for (size_t index = 0; index < read; ++index) {
        const int centered = static_cast<int>(input[index]) - 128;
        const int16_t scaled = static_cast<int16_t>(centered * 256 * _volume / 100);
        // Duplicate mono into both slots; ES8311 receives the left slot.
        _pcm[index * 2] = scaled;
        _pcm[index * 2 + 1] = scaled;
    }
    _pcmLength = read * 2 * sizeof(int16_t);  // offsets are bytes, not samples
    _pcmOffset = 0;
    return true;
}

void Es3Audio::update()
{
    if (!_playing) {
        return;
    }
    if (_draining) {
        if (static_cast<int32_t>(millis() - _drainDeadline) >= 0) {
            stop();
        }
        return;
    }
    for (uint8_t block = 0; block < kBlocksPerUpdate; ++block) {
        if (_pcmOffset == _pcmLength) {
            if (_dataRemaining == 0) {
                // Allow the entire DMA queue to drain without blocking the UI.
                _draining = true;
                _drainDeadline = millis() +
                    ((kDmaBufferCount + 1) * kDmaFrames * 1000U + _sampleRate - 1) /
                    _sampleRate + 10;
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
        const esp_err_t result = i2s_channel_write(_tx, reinterpret_cast<const uint8_t*>(_pcm) + _pcmOffset,
                                                      _pcmLength - _pcmOffset, &loaded, 0);
        _pcmOffset += loaded;
        if (result == ESP_ERR_TIMEOUT) {
            return;
        }
        if (result != ESP_OK) {
            Serial0.println("[es3] audio DMA write failed");
            stop();
            return;
        }
    }
}

void Es3Audio::stop()
{
    _playing = false;
    _draining = false;
    _dataRemaining = 0;
    _pcmLength = 0;
    _pcmOffset = 0;
    if (_file) {
        _file.close();
    }
    // Deleting the channel drops anything still queued, so the sound ends now.
    closeOutput();
}

void Es3Audio::setVolume(uint8_t percent)
{
    _volume = min<uint8_t>(percent, 100);
}

bool Es3Audio::ready() const { return _ready; }
bool Es3Audio::isPlaying() const { return _playing; }
uint8_t Es3Audio::volume() const { return _volume; }
