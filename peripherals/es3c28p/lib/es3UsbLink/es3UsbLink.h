#pragma once
#include <Arduino.h>
#include <atomic>
#include <freertos/queue.h>
#include "Es3VideoProtocol.h"
#include "es3Graphics.h"
#include "es3Audio.h"
#include "es3Video.h"

// USB task owns Serial/parser/TX. Application loop alone invokes graphics/audio.
class Es3UsbLink {
public:
    Es3UsbLink(Es3Graphics& graphics, Es3Audio& audio, Es3Video& video)
        : _graphics(graphics), _audio(audio), _video(video) {}
    bool begin();
    void update(); // application loop
    bool connected() const { return _session.load(); }
private:
    struct Command { es3proto::Frame frame; uint32_t epoch; };
    struct Response { uint8_t bytes[es3proto::kMaxFrame]; size_t size; uint32_t epoch; };
    Es3Graphics& _graphics;
    Es3Audio& _audio;
    Es3Video& _video;
    es3video::MuxParser _parser;
    QueueHandle_t _commands = nullptr, _responses = nullptr;
    TaskHandle_t _task = nullptr;
    std::atomic<bool> _session{false};
    std::atomic<uint32_t> _epoch{0};
    uint8_t _tx[es3proto::kMaxFrame]{};
    size_t _txSize = 0, _txOffset = 0;
    uint32_t _lastRequest = 0, _txStarted = 0;
    bool _pending = false;
    Response _appResponse{}; // application-task only
    void dispatch(const es3proto::Frame& frame);
    void apply(const es3proto::Frame& frame);
    void reply(const es3proto::Frame& request, const uint8_t* data = nullptr, size_t size = 0);
    void error(const es3proto::Frame& request, es3proto::ErrorCode code);
    void queueReply(const es3proto::Frame& request, const uint8_t* data = nullptr, size_t size = 0);
    void queueError(const es3proto::Frame& request, es3proto::ErrorCode code);
    void reset();
    void service();
    static void task(void* context);
};
