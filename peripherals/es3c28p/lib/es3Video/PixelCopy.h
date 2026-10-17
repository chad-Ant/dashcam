#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace es3video {
// Decoder output accounting and clipping, independent of JPEGDEC callbacks.
class DecodedPixels {
public:
    bool begin(uint16_t* output, unsigned width, unsigned height, uint32_t capacity) {
        _output = nullptr; _written = _total = 0;
        if (!output || !width || width > 512 || !height || height > 384 || width * height > capacity)
            return false;
        _output = output; _width = width; _height = height; _total = width * height;
        return true;
    }
    bool copy(int x, int y, int usedWidth, int stride, int rows, const uint16_t* source) {
        if (!_output || !source || x < 0 || y < 0 || unsigned(x) >= _width || unsigned(y) >= _height ||
            usedWidth <= 0 || stride < usedWidth || stride > 512 || rows <= 0 || rows > 384 ||
            unsigned(usedWidth) > _width - unsigned(x)) return false;
        // JPEGDEC may pad the bottom MCU; only copy visible rows.
        const unsigned visibleRows = unsigned(rows) < _height - unsigned(y) ? unsigned(rows) : _height - unsigned(y);
        const uint32_t count = unsigned(usedWidth) * visibleRows;
        if (count > _total - _written) return false;
        for (unsigned row = 0; row < visibleRows; ++row)
            memcpy(_output + (unsigned(y) + row) * _width + unsigned(x),
                   source + row * unsigned(stride), unsigned(usedWidth) * sizeof(uint16_t));
        _written += count;
        return true;
    }
    bool complete() const { return _output && _written == _total; }
private:
    uint16_t* _output = nullptr;
    unsigned _width = 0, _height = 0;
    uint32_t _written = 0, _total = 0;
};

// A full-frame draw must match the negotiated viewport. Return the number of
// copied rows, or zero without writing if metadata/capacity/cursor is invalid.
inline unsigned copyFrameStrip(uint16_t* output, size_t capacityPixels, const uint16_t* source,
                               unsigned imageW, unsigned imageH, unsigned viewportW,
                               unsigned viewportH, unsigned firstRow) {
    if (!output || !source || !imageW || imageW > 320 || !imageH || imageH > 240 ||
        imageW != viewportW || imageH != viewportH || firstRow >= imageH) return 0;
    const unsigned rows = imageH - firstRow < 8 ? imageH - firstRow : 8;
    const size_t count = imageW * rows;
    if (count > capacityPixels) return 0;
    memcpy(output, source + firstRow * imageW, count * sizeof(uint16_t));
    return rows;
}
} // namespace es3video
