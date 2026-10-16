#pragma once
#include <stdint.h>
#include <stddef.h>
#include <cstring>

namespace es3audio {
// Reader supplies read, size, position and seek (Arduino File or a test reader).
// Bound metadata scanning so a crafted SD file cannot monopolize the UI loop.
template<class Reader>
class WavHeader {
public:
    explicit WavHeader(Reader& reader) : _file(reader) {}
    uint32_t sampleRate() const { return _sampleRate; }
    uint32_t dataBytes() const { return _dataRemaining; }
    bool parse()
    {
        uint8_t riff[4]{};
        uint32_t riffSize = 0;
        uint8_t wave[4]{};
        if (!readExact(riff, sizeof(riff)) || !readU32(riffSize) || !readExact(wave, sizeof(wave)) ||
            std::memcmp(riff, "RIFF", 4) != 0 || std::memcmp(wave, "WAVE", 4) != 0) {
            return false;
        }

        if (riffSize < 4 || riffSize > UINT32_MAX - 8 || _file.size() < 12 ||
            riffSize > _file.size() - 8) {
            return false;
        }
        const uint32_t riffEnd = riffSize + 8;
        bool foundFormat = false;
        for (unsigned chunks = 0; chunks < 256 && _file.position() <= 65536 &&
             _file.position() <= riffEnd && riffEnd - _file.position() >= 8; ++chunks) {
            uint8_t chunkId[4]{};
            uint32_t chunkSize = 0;
            if (!readExact(chunkId, sizeof(chunkId)) || !readU32(chunkSize)) {
                return false;
            }
            const uint32_t chunkStart = _file.position();
            if (chunkSize > riffEnd - chunkStart) {
                return false;
            }
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
                    sampleRate < 8000 || sampleRate > 16000) {
                    return false;
                }
                _sampleRate = sampleRate;
                foundFormat = true;
            } else if (std::memcmp(chunkId, "data", 4) == 0) {
                if (!foundFormat || chunkSize == 0) {
                    return false;
                }
                _dataRemaining = chunkSize;
                return true;
            }

            if ((chunkSize & 1U) > riffEnd - chunkStart - chunkSize) {
                return false;
            }
            const uint32_t nextChunk = chunkStart + chunkSize + (chunkSize & 1U);
            if (nextChunk > _file.size() || !_file.seek(nextChunk)) {
                return false;
            }
        }
        return false;
    }

private:
    bool readExact(uint8_t* destination, size_t count)
    {
        return _file.read(destination, count) == count;
    }

    bool readU16(uint16_t& value)
    {
        uint8_t bytes[2]{};
        if (!readExact(bytes, sizeof(bytes))) {
            return false;
        }
        value = static_cast<uint16_t>(bytes[0]) |
                static_cast<uint16_t>(bytes[1]) << 8;
        return true;
    }

    bool readU32(uint32_t& value)
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

    Reader& _file;
    uint32_t _sampleRate = 0, _dataRemaining = 0;
};
} // namespace es3audio
