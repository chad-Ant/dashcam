#pragma once
#include <JPEGDEC.h>
#include <stdint.h>
#include <string.h>

// Shared by the firmware worker, SD benchmark and native codec tests.
// One decoder instance per worker. Output is valid only on true.
class Es3JpegDecoder {
public:
    bool decode(uint8_t* data, uint32_t bytes, uint16_t* output, int width, int height) {
        if (!data || !output || bytes < 4 || bytes > 65536 || width < 1 || width > 320 ||
            height < 1 || height > 240 || data[0] != 0xff || data[1] != 0xd8 ||
            data[bytes - 2] != 0xff || data[bytes - 1] != 0xd9) return false;
        if (!baselineHeader(data, bytes, width, height)) return false;
        _output = output; _width = width; _height = height; _written = 0; _badBlock = false;
        if (!_decoder.openRAM(data, bytes, draw)) return false;
        bool ok = _decoder.getWidth() == width && _decoder.getHeight() == height &&
                  _decoder.getJPEGType() == JPEG_MODE_BASELINE;
        if (ok) {
            _decoder.setUserPointer(this);
            _decoder.setPixelType(RGB565_LITTLE_ENDIAN);
            ok = _decoder.decode(0, 0, 0) != 0 && !_badBlock &&
                 _written == uint32_t(width * height);
        }
        _decoder.close();
        return ok;
    }
private:
    // Check segment bounds and SOF dimensions before the third-party parser.
    // The entropy stream remains JPEGDEC's responsibility; header scanning
    // stops at SOS so framing-like bytes in compressed data are never parsed.
    static bool baselineHeader(const uint8_t* data, uint32_t bytes, int width, int height) {
        uint32_t pos = 2;
        bool haveFrame = false;
        while (pos + 1 < bytes) {
            if (data[pos++] != 0xff) return false;
            while (pos < bytes && data[pos] == 0xff) ++pos;
            if (pos >= bytes) return false;
            const uint8_t marker = data[pos++];
            if (marker == 0 || marker == 0xd8 || marker == 0xd9 || marker == 1 ||
                (marker >= 0xd0 && marker <= 0xd7) || pos + 2 > bytes) return false;
            const uint32_t size = (uint32_t(data[pos]) << 8) | data[pos+1];
            if (size < 2 || size > bytes - pos) return false;
            if (marker == 0xda) return haveFrame && size >= 6;
            if (marker >= 0xc0 && marker <= 0xcf && marker != 0xc4) {
                if (marker != 0xc0 || haveFrame || size < 11 || data[pos+2] != 8) return false;
                const int h = (int(data[pos+3]) << 8) | data[pos+4];
                const int w = (int(data[pos+5]) << 8) | data[pos+6];
                const uint8_t components = data[pos+7];
                if (w != width || h != height || (components != 1 && components != 3) ||
                    size != 8u + 3u * components) return false;
                haveFrame = true;
            }
            pos += size;
        }
        return false;
    }
    JPEGDEC _decoder;
    uint16_t* _output = nullptr;
    int _width = 0, _height = 0;
    uint32_t _written = 0;
    bool _badBlock = false;
    static int draw(JPEGDRAW* block);
};

inline int Es3JpegDecoder::draw(JPEGDRAW* block)
{
    auto* self = static_cast<Es3JpegDecoder*>(block->pUser);
    if (!self || !block->pPixels || block->x < 0 || block->y < 0 ||
        block->iWidthUsed <= 0 || block->iWidthUsed > block->iWidth || block->iHeight <= 0 ||
        block->x + block->iWidthUsed > self->_width || block->y >= self->_height) {
        if (self) self->_badBlock = true;
        return 0;
    }
    const int rows = (block->iHeight < self->_height - block->y ? block->iHeight : self->_height - block->y);
    for (int row = 0; row < rows; ++row)
        memcpy(self->_output + (block->y + row) * self->_width + block->x,
               block->pPixels + row * block->iWidth, block->iWidthUsed * 2);
    self->_written += rows * block->iWidthUsed;
    return 1;
}
