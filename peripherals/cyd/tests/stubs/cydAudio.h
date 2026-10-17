#pragma once
#include <stdint.h>
struct CydAudio {
    bool playing = false;
    void setVolume(uint8_t) {}
    bool playWav(const char*) { playing = true; return true; }
    void stop() { playing = false; }
    bool isPlaying() const { return playing; }
};
