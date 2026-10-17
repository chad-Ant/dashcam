#pragma once
#include <stdint.h>
#include <cassert>
struct CydGraphics {
    uint16_t pixels[320 * 240]{};
    bool visible = true;
    uint8_t page = 5;
    unsigned placeholders = 0, stale = 0;
    bool mapVisible() const { return visible && page == 5; }
    bool calibrationActive() const { return !visible; }
    uint8_t activePage() const { return page; }
    uint8_t brightnessDuty() const { return 255; }
    bool sdAvailable() const { return false; }
    bool calibrationLoaded() const { return true; }
    bool updatePageText(uint8_t index, const char[5][24]) { return index < 6; }
    void mapPlaceholder(const char*) { if (mapVisible()) ++placeholders; }
    void mapStale() { if (mapVisible()) ++stale; }
    bool drawMapBlock(int x, int y, int w, int h, uint16_t* source) {
        if (!mapVisible()) return false;
        assert(x >= 0 && y >= 0 && w > 0 && h > 0 && x + w <= 320 && y + h <= 224);
        for (int row = 0; row < h; ++row)
            for (int col = 0; col < w; ++col) pixels[(y+row)*320+x+col] = source[row*w+col];
        return true;
    }
};
