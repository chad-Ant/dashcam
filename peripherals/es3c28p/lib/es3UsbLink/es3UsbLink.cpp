#include "es3UsbLink.h"

#if !ARDUINO_USB_CDC_ON_BOOT
#error "ES3C28P communication requires native USB CDC enabled on boot"
#endif

bool Es3UsbLink::begin()
{
    if (_task) return true;
    _commands = xQueueCreate(1, sizeof(Command));
    _responses = xQueueCreate(1, sizeof(Response));
    if (!_commands || !_responses) {
        if (_commands) vQueueDelete(_commands);
        if (_responses) vQueueDelete(_responses);
        _commands = _responses = nullptr; return false;
    }
    Serial.setRxBufferSize(8192);
#if ARDUINO_USB_MODE
    Serial.setTxBufferSize(512);
#endif
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);
    if (xTaskCreatePinnedToCore(task, "es3-usb", 8192, this, 2, &_task, 0) != pdPASS) {
        vQueueDelete(_commands); vQueueDelete(_responses);
        _commands = _responses = nullptr; return false;
    }
    return true;
}
void Es3UsbLink::reset()
{
    _session.store(false); _epoch.fetch_add(1);
    _video.stop(); _parser.reset();
    _txSize = _txOffset = 0; _pending = false;
    xQueueReset(_commands); xQueueReset(_responses);
}
void Es3UsbLink::queueReply(const es3proto::Frame& request, const uint8_t* data, size_t size)
{
    _txSize = es3proto::encode(request.sequence, request.type | es3proto::Reply, data, size, _tx);
    _txOffset = 0; _txStarted = millis();
}
void Es3UsbLink::queueError(const es3proto::Frame& request, es3proto::ErrorCode code)
{
    const uint8_t value = code;
    _txSize = es3proto::encode(request.sequence, es3proto::Error, &value, 1, _tx);
    _txOffset = 0; _txStarted = millis();
}
void Es3UsbLink::reply(const es3proto::Frame& request, const uint8_t* data, size_t size)
{
    _appResponse.size = es3proto::encode(request.sequence, request.type | es3proto::Reply, data, size, _appResponse.bytes);
}
void Es3UsbLink::error(const es3proto::Frame& request, es3proto::ErrorCode code)
{
    const uint8_t value = code;
    _appResponse.size = es3proto::encode(request.sequence, es3proto::Error, &value, 1, _appResponse.bytes);
}
void Es3UsbLink::dispatch(const es3proto::Frame& frame)
{
    using namespace es3proto;
    using namespace es3video;
    if (frame.type == Hello) {
        if (frame.length) { queueError(frame, BadPayload); return; }
        reset(); _session.store(true); _lastRequest = millis();
        const uint8_t identity[] = {'E','S','3','C','2','8','P'};
        queueReply(frame, identity, sizeof(identity)); return;
    }
    if (!_session.load()) { queueError(frame, NoSession); return; }
    _lastRequest = millis();
    if (frame.type == Ping) {
        if (frame.length) queueError(frame, BadPayload); else queueReply(frame);
    } else if (frame.type == es3proto::Capabilities) {
        if (frame.length) { queueError(frame, BadPayload); return; }
        if (!_video.available()) { queueError(frame, NotReady); return; }
        uint8_t data[kCapabilitiesReplyBytes];
        const size_t size = encodeCapabilitiesReply(_video.capabilities(), data, sizeof(data));
        queueReply(frame, data, size);
    } else if (frame.type == es3proto::StartVideo) {
        if (!_video.available()) { queueError(frame, NotReady); return; }
        StartVideoRequest config;
        if (!decodeStartVideoRequest(frame.payload, frame.length, _video.capabilities(), config)) {
            queueError(frame, BadPayload); return;
        }
        StartVideoReply response; response.sessionToken = _video.start(config); response.initialGrant = 1;
        if (!response.sessionToken) { queueError(frame, NotReady); return; }
        uint8_t data[kStartVideoReplyBytes];
        const size_t size = encodeStartVideoReply(response, data, sizeof(data));
        queueReply(frame, data, size);
    } else if (frame.type == es3proto::StopVideo) {
        uint32_t token = 0;
        if (!decodeStopVideoRequest(frame.payload, frame.length, token)) { queueError(frame, BadPayload); return; }
        _video.stopSurface(es3map::Surface::Camera, token); queueReply(frame);
    } else if (frame.type == es3map::CmdPanCapabilities) {
        if (frame.length) { queueError(frame, BadPayload); return; }
        if (!_video.panAvailable()) { queueError(frame, NotReady); return; }
        const auto caps = _video.capabilities();
        uint8_t data[es3map::kPanCapabilitiesReplyBytes];
        const size_t size = es3map::encodePanCapabilitiesReply(es3map::panCapabilities(),
            es3map::capabilities(caps), caps, data, sizeof(data));
        queueReply(frame, data, size);
    } else if (frame.type == es3map::CmdStartPan) {
        if (!_video.panAvailable()) { queueError(frame, NotReady); return; }
        const auto caps = _video.capabilities();
        es3map::StartPanRequest config;
        if (!es3map::decodeStartPanRequest(frame.payload, frame.length, es3map::panCapabilities(),
                es3map::capabilities(caps), caps, config)) { queueError(frame, BadPayload); return; }
        es3map::StartPanReply response;
        response.sessionToken = _video.startPan(config); response.initialGrant = 1;
        if (!response.sessionToken) { queueError(frame, NotReady); return; }
        uint8_t data[es3map::kStartMapReplyBytes];
        const size_t size = es3map::encodeStartPanReply(response, data, sizeof(data));
        queueReply(frame, data, size);
    } else if (frame.type == es3map::CmdMapCapabilities) {
        if (frame.length) { queueError(frame, BadPayload); return; }
        if (!_video.available()) { queueError(frame, NotReady); return; }
        uint8_t data[es3map::kMapCapabilitiesReplyBytes];
        const auto caps = _video.capabilities();
        const size_t size = es3map::encodeMapCapabilitiesReply(es3map::capabilities(caps), caps, data, sizeof(data));
        queueReply(frame, data, size);
    } else if (frame.type == es3map::CmdStartMap) {
        if (!_video.available()) { queueError(frame, NotReady); return; }
        const auto caps = _video.capabilities();
        es3map::StartMapRequest config;
        if (!es3map::decodeStartMapRequest(frame.payload, frame.length, es3map::capabilities(caps), caps, config)) {
            queueError(frame, BadPayload); return;
        }
        es3map::StartMapReply response;
        response.sessionToken = _video.startMap(config); response.initialGrant = 1;
        if (!response.sessionToken) { queueError(frame, NotReady); return; }
        uint8_t data[es3map::kStartMapReplyBytes];
        const size_t size = es3map::encodeStartMapReply(response, data, sizeof(data));
        queueReply(frame, data, size);
    } else if (frame.type == es3map::CmdStopMap) {
        uint32_t token = 0;
        if (!es3map::decodeStopMapRequest(frame.payload, frame.length, token)) { queueError(frame, BadPayload); return; }
        _video.stopSurface(es3map::Surface::Map, token); queueReply(frame);
    } else {
        Command command; command.frame = frame; command.epoch = _epoch.load();
        if (xQueueSend(_commands, &command, 0) != pdTRUE) { reset(); return; }
        _pending = true;
    }
}
void Es3UsbLink::apply(const es3proto::Frame& frame)
{
    using namespace es3proto;
    switch (frame.type) {
    case Status: {
        if (frame.length) { error(frame, BadPayload); return; }
        const uint32_t uptime = millis();
        const uint8_t status[] = {
            uint8_t(uptime), uint8_t(uptime >> 8), uint8_t(uptime >> 16), uint8_t(uptime >> 24),
            _graphics.activePage(), _graphics.brightnessDuty(), uint8_t(_graphics.sdAvailable()),
            uint8_t(_graphics.calibrationLoaded()), uint8_t(_audio.isPlaying())
        };
        reply(frame, status, sizeof(status));
        break;
    }
    case Page: {
        if (frame.length != 1 + 5 * kTextBytes) { error(frame, BadPayload); return; }
        char text[5][kTextBytes];
        memcpy(text, frame.payload + 1, sizeof(text));
        if (!_graphics.updatePageText(frame.payload[0], text)) { error(frame, BadPayload); return; }
        reply(frame);
        break;
    }
    case Alert:
        if (frame.length != 1 || frame.payload[0] > 100) { error(frame, BadPayload); return; }
        _audio.setVolume(frame.payload[0]);
        if (!_audio.playWav("/alert.wav")) { error(frame, NotReady); return; }
        reply(frame);
        break;
    case StopAudio:
        if (frame.length) { error(frame, BadPayload); return; }
        _audio.stop();
        reply(frame);
        break;
    default:
        error(frame, BadCommand);
        break;
    }
}
void Es3UsbLink::update()
{
    _graphics.setHostConnected(_session.load());
    if (!_commands) return;
    Command command;
    if (xQueueReceive(_commands, &command, 0) == pdTRUE && command.epoch == _epoch.load()) {
        _appResponse.epoch = command.epoch;
        apply(command.frame);
        // A late response from an old session is discarded by the USB owner.
        if (command.epoch == _epoch.load()) xQueueOverwrite(_responses, &_appResponse);
    }
}
void Es3UsbLink::task(void* context)
{
    auto* self = static_cast<Es3UsbLink*>(context);
    for (;;) { self->service(); vTaskDelay(1); }
}
void Es3UsbLink::service()
{
    const uint32_t now = millis();
    if (!Serial) {
        if (_session.load() || _pending || _txSize) reset();
        return;
    }
    if ((_session.load() && now - _lastRequest > 5000) ||
        (_txSize && now - _txStarted > 1000) || _video.fault()) reset();
    _video.expire(now);
    _parser.expire(now);
    if (!_txSize && _pending) {
        Response response;
        if (xQueueReceive(_responses, &response, 0) == pdTRUE && response.epoch == _epoch.load()) {
            memcpy(_tx, response.bytes, response.size); _txSize = response.size;
            _txOffset = 0; _txStarted = now; _pending = false;
        }
    }
    if (_txSize) {
        const int space = Serial.availableForWrite();
        if (space > 0) _txOffset += Serial.write(_tx + _txOffset, min<size_t>(_txSize - _txOffset, size_t(space)));
        if (_txOffset == _txSize) _txSize = _txOffset = 0;
    }
    // Drain at most 4 KiB per turn. The task yields every turn; decoder and
    // Arduino loop can run even under continuous host traffic.
    if (!_txSize && !_pending) {
        es3video::MuxParser::Item item;
        for (size_t budget = 0; budget < 4096; ++budget) {
            // Drain parser-buffered records before fetching the next byte.
            _parser.feed(nullptr, 0, now, item);
            if (item.kind == es3video::MuxParser::ItemNone) {
                if (!Serial.available()) break;
                const int value = Serial.read();
                if (value < 0) break;
                const uint8_t byte = value;
                _parser.feed(&byte, 1, now, item);
            }
            if (item.kind == es3video::MuxParser::ItemControl) { dispatch(item.control); break; }
            if (item.kind == es3video::MuxParser::ItemVideo && _session.load()) _video.accept(item.video, now);
        }
    }
    if (!_txSize && !_pending) {
        es3video::VideoState::Event event;
        if (_video.pop(event)) {
            memcpy(_tx, event.bytes, event.size); _txSize = event.size;
            _txOffset = 0; _txStarted = now;
        }
    }
}
