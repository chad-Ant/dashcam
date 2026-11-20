/** @file AudioLevel.h - Portable sample scaling for alert and master volume. */
#pragma once
#include <stdint.h>

namespace es3audio {
/**
 * Converts one unsigned 8-bit PCM sample to signed 16-bit, scaled linearly by
 * the per-alert volume and the user's master volume (both 0..100, clamped).
 * The product stays within int32: 128 * 256 * 100 * 100 < 2^31.
 */
inline int16_t scaleSample(uint8_t sample, uint8_t volume, uint8_t master)
{
    const int32_t v = volume > 100 ? 100 : volume;
    const int32_t m = master > 100 ? 100 : master;
    const int32_t centered = static_cast<int32_t>(sample) - 128;
    return static_cast<int16_t>(centered * 256 * v * m / 10000);
}
}  // namespace es3audio
