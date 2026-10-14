#pragma once
#include <Arduino.h>
#include "JpegDecoder.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "VideoState.h"
#include "MapSurface.h"

// No peripheral access here. All state transitions use a short mutex; decode
// operates on leased buffers outside it. Objects have application lifetime.
class Es3Video {
public:
    bool begin();
    bool available() const { return _ready; }
    es3video::Capabilities capabilities() const;
    uint32_t start(const es3video::StartVideoRequest& request);
    uint32_t startMap(const es3map::StartMapRequest& request);
    void stopSurface(es3map::Surface surface, uint32_t token);
    void stop(uint32_t token = 0);
    void accept(const es3video::Record& record, uint32_t now);
    void expire(uint32_t now);
    bool pop(es3video::VideoState::Event& event);
    bool fault();
    void setDemand(es3video::Demand demand);
    uint32_t snapshot(es3video::StartVideoRequest& config);
    uint32_t view(uint8_t page, bool blocked, es3video::StartVideoRequest& config,
                  es3map::Binding& binding);
    bool acquire(es3video::VideoState::Image& image);
    void release(es3video::VideoState::Image& image, bool presented, uint32_t duration);
    es3video::VideoState::Stats stats();
private:
    es3map::SurfaceState _state;
    SemaphoreHandle_t _mutex = nullptr;
    TaskHandle_t _task = nullptr;
    bool _ready = false;
    uint32_t _nextToken = 0;
    Es3JpegDecoder _decoder;
    static void task(void* context);
    void decodeLoop();
    void lock() { xSemaphoreTake(_mutex, portMAX_DELAY); }
    void unlock() { xSemaphoreGive(_mutex); }
};
