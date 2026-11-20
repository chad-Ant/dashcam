/**
 * @file MicLevel.h
 * @brief Portable microphone level meter (MIC_LEVEL_PLAN.md).
 *
 * Feed interleaved 16-bit I2S frames; read a 0..100 level. Per window: pick
 * one slot, remove DC with a first-order high-pass, take the RMS, convert to
 * dBFS and map [floorDb, ceilingDb] linearly to 0..100. The displayed level
 * rises at once and falls by at most releasePerWindow points per window, so
 * speech reads steadily. The first settleMs after reset() are discarded (ADC
 * start-up transient). Single owner: the capture task; publish level() from it.
 */
#pragma once
#include <math.h>
#include <stddef.h>
#include <stdint.h>

namespace es3audio {
struct MicLevelConfig {
    uint32_t sampleRate;
    uint16_t windowMs;
    uint16_t settleMs;
    float floorDb;          // maps to 0
    float ceilingDb;        // maps to 100
    uint8_t releasePerWindow;
};
/** P1 bench values: 16 kHz, 50 ms windows, -70..-20 dBFS, 100 ms settling. */
constexpr MicLevelConfig kDefaultMicLevel{16000, 50, 100, -70.0f, -20.0f, 4};

/** No level yet (settling, or no capture running). */
constexpr int kMicLevelNone = -1;

/** dBFS of an RMS value in 16-bit sample units (floor -120). */
inline float rmsToDbfs(float rms) {
    return rms <= 0.0f ? -120.0f : 20.0f * log10f(rms / 32768.0f);
}
/** Maps dBFS linearly to 0..100 over [floorDb, ceilingDb], clamped. */
inline int dbfsToLevel(float dbfs, float floorDb, float ceilingDb) {
    if (!(ceilingDb > floorDb) || dbfs <= floorDb) return 0;
    if (dbfs >= ceilingDb) return 100;
    return static_cast<int>((dbfs - floorDb) * 100.0f / (ceilingDb - floorDb) + 0.5f);
}

class MicLevel {
public:
    explicit MicLevel(const MicLevelConfig& config = kDefaultMicLevel)
        : _config(config),
          _windowSamples(config.sampleRate * config.windowMs / 1000 ? config.sampleRate * config.windowMs / 1000 : 1),
          _settleSamples(config.sampleRate * config.settleMs / 1000) {
        reset();
    }

    /** Restarts settling and clears the filter and level. */
    void reset() {
        _settleRemaining = _settleSamples;
        _previousInput = 0.0f;
        _previousOutput = 0.0f;
        _sumSquares = 0.0f;
        _count = 0;
        _level = kMicLevelNone;
        _lastDbfs = -120.0f;
        _windows = 0;
    }

    /**
     * Adds frames of `channels` interleaved samples, measuring slot `slot`.
     * Ignores calls with a null pointer or an out-of-range slot.
     */
    void addFrames(const int16_t* samples, size_t frames, uint8_t channels, uint8_t slot) {
        if (!samples || !channels || slot >= channels) return;
        for (size_t i = 0; i < frames; ++i) {
            const float x = samples[i * channels + slot];
            // DC blocker, cutoff about 13 Hz at 16 kHz (time constant 12.5 ms).
            const float y = x - _previousInput + kPole * _previousOutput;
            _previousInput = x;
            _previousOutput = y;
            if (_settleRemaining) { --_settleRemaining; continue; }
            _sumSquares += y * y;
            if (++_count == _windowSamples) finishWindow();
        }
    }

    /** 0..100, or kMicLevelNone until the first window after settling. */
    int level() const { return _level; }
    /** RMS of the last window in dBFS (-120 before the first). */
    float lastDbfs() const { return _lastDbfs; }
    uint32_t windows() const { return _windows; }

private:
    static constexpr float kPole = 0.995f;

    void finishWindow() {
        _lastDbfs = rmsToDbfs(sqrtf(_sumSquares / static_cast<float>(_count)));
        const int target = dbfsToLevel(_lastDbfs, _config.floorDb, _config.ceilingDb);
        if (_level < 0 || target >= _level) {
            _level = target;                                   // instant attack
        } else {
            const int fallen = _level - _config.releasePerWindow;
            _level = fallen > target ? fallen : target;        // limited release
        }
        _sumSquares = 0.0f;
        _count = 0;
        ++_windows;
    }

    MicLevelConfig _config;
    uint32_t _windowSamples;
    uint32_t _settleSamples;
    uint32_t _settleRemaining = 0;
    float _previousInput = 0.0f, _previousOutput = 0.0f;
    float _sumSquares = 0.0f;
    uint32_t _count = 0;
    int _level = kMicLevelNone;
    float _lastDbfs = -120.0f;
    uint32_t _windows = 0;
};
}  // namespace es3audio
