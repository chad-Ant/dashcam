#include "es3Video.h"
#include "VideoBuffers.h"
#include <esp_heap_caps.h>
#include <esp_system.h>

using namespace es3video;

bool Es3Video::begin()
{
    if (_ready) return true;
    if (!psramFound()) return false;
    _mutex = xSemaphoreCreateMutex();
    if (!_mutex) return false;
    const bool ok = allocateBuffers(_state, _pixelCapacity, _panReady,
        [](size_t bytes) { return heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT); },
        [](void* buffer) { heap_caps_free(buffer); });
    _nextToken = esp_random();
    if (ok && xTaskCreatePinnedToCore(task, "es3-jpeg", 8192, this, 1, &_task, 0) == pdPASS) {
        _ready = true; return true;
    }
    freeBuffers(_state, _pixelCapacity, [](void* buffer) { heap_caps_free(buffer); });
    _panReady = false;
    vSemaphoreDelete(_mutex); _mutex = nullptr;
    return false;
}
Capabilities Es3Video::capabilities() const
{
    Capabilities caps;
    caps.canvasWidth = caps.maxWidth = 320;
    caps.canvasHeight = caps.maxHeight = 240;
    caps.maxJpegBytes = kMaxJpegBytes;
    caps.maxCredits = 1;
    caps.maxChunkBytes = kDefaultChunkDataBytes;
    return caps;
}
uint32_t Es3Video::start(const StartVideoRequest& request)
{
    if (!_ready || !validStartVideo(request, capabilities())) return 0;
    lock();
    if (++_nextToken == 0) ++_nextToken;
    const uint32_t token = _state.startCamera(_nextToken, request, capabilities()) ? _nextToken : 0;
    unlock(); return token;
}
uint32_t Es3Video::startMap(const es3map::StartMapRequest& request)
{
    if (!_ready) return 0;
    lock();
    if (++_nextToken == 0) ++_nextToken;
    const uint32_t token = _state.startMap(_nextToken, request, capabilities()) ? _nextToken : 0;
    unlock(); return token;
}
uint32_t Es3Video::startPan(const es3map::StartPanRequest& request)
{
    if (!panAvailable()) return 0;
    lock();
    if (++_nextToken == 0) ++_nextToken;
    const uint32_t token = _state.startPan(_nextToken, request, capabilities()) ? _nextToken : 0;
    unlock(); return token;
}
void Es3Video::stopSurface(es3map::Surface surface, uint32_t token)
{
    if (!_ready) return;
    lock(); _state.stopSurface(surface, token); unlock();
}
uint32_t Es3Video::view(uint8_t page, bool blocked, StartVideoRequest& config, es3map::Binding& binding, es3map::PanSnapshot* pan)
{
    if (!_ready) return 0;
    lock();
    _state.view(page, blocked);
    config = _state.config; binding = _state.binding;
    if (pan) { pan->session = _state.panConfig; pan->pose = _state.pose; pan->poseReceivedMs = _state.poseReceivedMs; }
    const uint32_t token = _state.token;
    unlock(); return token;
}
void Es3Video::stop(uint32_t token)
{
    if (!_ready) return;
    lock(); if (!token || token == _state.token) _state.reset(); unlock();
}
void Es3Video::accept(const Record& record, uint32_t now)
{
    if (!_ready) return;
    lock(); _state.accept(record, now); unlock();
}
void Es3Video::expire(uint32_t now)
{
    if (!_ready) return;
    lock(); _state.expire(now); unlock();
}
bool Es3Video::pop(VideoState::Event& event)
{
    if (!_ready) return false;
    lock(); const bool result = _state.pop(event); unlock(); return result;
}
bool Es3Video::fault()
{
    if (!_ready) return false;
    lock(); const bool result = _state.fault(); unlock(); return result;
}
void Es3Video::setDemand(Demand demand)
{
    if (!_ready) return;
    lock(); _state.setDemand(demand); unlock();
}
uint32_t Es3Video::snapshot(StartVideoRequest& config)
{
    if (!_ready) return 0;
    lock(); config = _state.config; const uint32_t token = _state.token; unlock(); return token;
}
bool Es3Video::acquire(VideoState::Image& image)
{
    if (!_ready) return false;
    lock(); const bool result = _state.acquire(image); unlock(); return result;
}
void Es3Video::present(const VideoState::Image& image, uint32_t duration)
{
    if (!_ready) return;
    lock(); _state.present(image, duration); unlock();
}
void Es3Video::release(VideoState::Image& image, bool presented, uint32_t duration)
{
    if (!_ready) return;
    lock(); _state.release(image, presented, duration); unlock();
}
VideoState::Stats Es3Video::stats()
{
    if (!_ready) return VideoState::Stats();
    lock(); const auto result = _state.stats; unlock(); return result;
}
void Es3Video::task(void* context) { static_cast<Es3Video*>(context)->decodeLoop(); }
void Es3Video::decodeLoop()
{
    for (;;) {
        VideoState::Job job;
        lock(); const bool haveJob = _state.takeJob(job); unlock();
        if (!haveJob) { vTaskDelay(1); continue; }
        const uint32_t started = micros();
        const bool ok = _decoder.decode(_state.jpeg[job.jpeg], job.bytes,
                                        _state.pixels[job.image], job.width, job.height, _pixelCapacity[job.image]);
        const uint32_t duration = micros() - started;
        lock(); _state.finishJob(job, ok, duration); unlock();
        vTaskDelay(1);
    }
}
