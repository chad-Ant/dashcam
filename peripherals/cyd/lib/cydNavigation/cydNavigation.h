#pragma once
#include <Arduino.h>
#include "MapReceiver.h"
#include "MapDecoder.h"
#include "cydAudio.h"

// The application loop exclusively owns UART0, receiver, JPEGDEC and TFT.
// CH340 disconnect is detected by the same five-second heartbeat lease as ES3.
class CydNavigation {
public:
    CydNavigation(CydGraphics& graphics, CydAudio& audio) : _graphics(graphics), _audio(audio) {}
    bool begin();
    void update();
private:
    CydGraphics& _graphics;
    CydAudio& _audio;
    cyd::MapReceiver _receiver;
    CydMapDecoder _decoder;
    es3video::MuxParser _parser{1024};
    bool _session = false, _wasVisible = false, _haveImage = false;
    uint8_t _displayState = 0;
    uint32_t _nextToken = 0, _lastRequest = 0, _lastImage = 0, _txSince = 0;
    uint8_t _tx[es3proto::kMaxFrame]{};
    size_t _size = 0, _offset = 0;
    void reset();
    void dispatch(const es3proto::Frame& frame);
    void reply(const es3proto::Frame& frame, const uint8_t* data = nullptr, size_t size = 0);
    void error(const es3proto::Frame& frame, es3proto::ErrorCode code);
    void displayHealth();
};
