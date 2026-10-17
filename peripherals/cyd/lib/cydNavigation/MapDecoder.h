#pragma once
#include <JPEGDEC.h>
#include "cydGraphics.h"

class CydMapDecoder {
public:
    bool draw(uint8_t* data, uint32_t bytes, CydGraphics& graphics) {
        if (!data || bytes < 4 || bytes > 32768 || data[0] != 0xff || data[1] != 0xd8 ||
            data[bytes-2] != 0xff || data[bytes-1] != 0xd9 || !baselineHeader(data, bytes, 320, 224)) return false;
        _graphics = &graphics; _count = 0; _bad = false;
        if (!_decoder.openRAM(data, bytes, block)) return false;
        bool ok = _decoder.getWidth() == 320 && _decoder.getHeight() == 224 &&
                  _decoder.getJPEGType() == JPEG_MODE_BASELINE;
        if (ok) {
            _decoder.setUserPointer(this);
            _decoder.setPixelType(RGB565_LITTLE_ENDIAN);
            ok = _decoder.decode(0, 0, 0) != 0 && !_bad && _count == 320u * 224;
        }
        _decoder.close(); _graphics = nullptr;
        return ok;
    }
private:
    // Same baseline/header bounds checks as the ES3C28P decoder.
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
    CydGraphics* _graphics = nullptr;
    uint32_t _count = 0;
    bool _bad = false;
    static int block(JPEGDRAW* b) {
        if (!b || !b->pUser) return 0;
        auto* self = static_cast<CydMapDecoder*>(b->pUser);
        if (!b->pPixels || b->x < 0 || b->y < 0 || b->x >= 320 || b->y >= 224 ||
            b->iWidthUsed <= 0 || b->iWidthUsed > 320 - b->x || b->iWidth < b->iWidthUsed ||
            b->iWidth > 320 || b->iHeight <= 0 || b->iHeight > 224) {
            self->_bad = true; return 0;
        }
        const int rows = b->iHeight < 224 - b->y ? b->iHeight : 224 - b->y;
        const uint32_t count = uint32_t(b->iWidthUsed) * rows;
        if (count > 320u * 224 - self->_count) { self->_bad = true; return 0; }
        // JPEGDEC may pad MCU rows: honor source stride and never touch the tabs.
        for (int row = 0; row < rows; ++row) {
            if (!self->_graphics->drawMapBlock(b->x, b->y + row, b->iWidthUsed, 1,
                                              b->pPixels + row * b->iWidth)) {
                self->_bad = true; return 0;
            }
        }
        self->_count += count; return 1;
    }
};
