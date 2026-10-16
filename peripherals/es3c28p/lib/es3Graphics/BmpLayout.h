#pragma once
#include <stdint.h>
#include <stddef.h>

namespace es3graphics {
struct BmpLayout {
    uint32_t offset = 0, rowBytes = 0;
    bool bottomUp = false;
};
inline uint32_t bmpU32(const uint8_t* p) {
    return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
}
// Validate all arithmetic before seeking or drawing. The file-size field is
// optional; actual filesystem size bounds both the DIB and pixel data.
inline bool bmpLayout(const uint8_t* header, size_t bytes, uint64_t fileBytes,
                      uint16_t width, uint16_t height, BmpLayout& result) {
    if (!header || bytes < 54 || fileBytes < 54 || !width || width > 320 || !height || height > 224 ||
        header[0] != 'B' || header[1] != 'M' || header[26] != 1 || header[27] != 0 ||
        header[28] != 24 || header[29] != 0 || bmpU32(header + 30) != 0) return false;
    const uint32_t dib = bmpU32(header + 14), offset = bmpU32(header + 10);
    const uint32_t rawHeight = bmpU32(header + 22);
    const bool bottomUp = (rawHeight & 0x80000000u) == 0;
    const uint32_t magnitude = bottomUp ? rawHeight : uint32_t(0u - rawHeight);
    if (bmpU32(header + 18) != width || magnitude != height || dib < 40 ||
        uint64_t(dib) + 14 > offset || offset > fileBytes) return false;
    const uint32_t rowBytes = (uint32_t(width) * 3 + 3) & ~3u;
    const uint64_t end = uint64_t(offset) + uint64_t(rowBytes) * height;
    if (end > fileBytes || end > UINT32_MAX) return false;
    result.offset = offset; result.rowBytes = rowBytes; result.bottomUp = bottomUp;
    return true;
}
} // namespace es3graphics
