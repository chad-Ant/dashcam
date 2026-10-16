#pragma once
#include "VideoState.h"

namespace es3video {
// Startup-only allocation. The caller must stop workers before releasing these
// buffers. Keep capacity metadata coupled to the allocation on every retry.
constexpr uint32_t kFramePixels = 320 * 240;
constexpr uint32_t kBackgroundPixels = 512 * 384;
constexpr uint32_t kChromePixels = 320 * 32;

template<class Free>
void freeBuffers(VideoState& state, uint32_t (&capacity)[4], Free free) {
    for (auto& buffer : state.jpeg) { free(buffer); buffer = nullptr; }
    for (unsigned i = 0; i < 4; ++i) {
        free(state.pixels[i]); state.pixels[i] = nullptr; capacity[i] = 0;
    }
}

template<class Allocate, class Free>
bool allocateBuffers(VideoState& state, uint32_t (&capacity)[4], bool& panReady,
                     Allocate allocate, Free free) {
    panReady = false;
    for (unsigned i = 0; i < 2; ++i) {
        state.jpeg[i] = static_cast<uint8_t*>(allocate(kMaxJpegBytes));
        state.pixels[i] = static_cast<uint16_t*>(allocate(kFramePixels * sizeof(uint16_t)));
        capacity[i] = state.pixels[i] ? kFramePixels : 0;
        if (!state.jpeg[i] || !state.pixels[i]) {
            freeBuffers(state, capacity, free); return false;
        }
    }
    uint16_t* upgraded[4]{};
    for (unsigned i = 0; i < 4; ++i) {
        const uint32_t count = i < 2 ? kBackgroundPixels : kChromePixels;
        upgraded[i] = static_cast<uint16_t*>(allocate(count * sizeof(uint16_t)));
        if (!upgraded[i]) {
            for (auto* buffer : upgraded) free(buffer);
            return true; // Whole-frame buffers remain usable.
        }
    }
    for (unsigned i = 0; i < 4; ++i) {
        free(state.pixels[i]); state.pixels[i] = upgraded[i];
        capacity[i] = i < 2 ? kBackgroundPixels : kChromePixels;
    }
    panReady = true;
    return true;
}
} // namespace es3video
