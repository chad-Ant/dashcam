#include "cydNavigation.h"
#include <esp_system.h>

bool CydNavigation::begin()
{
    Serial.setRxBufferSize(8192);
    Serial.setTxBufferSize(512);
    Serial.begin(115200);
    _nextToken = esp_random();
    return true;
}
void CydNavigation::reset()
{
    _session = false; _receiver.reset(); _parser.reset();
    _size = _offset = 0; _haveImage = false; _displayState = 0;
}
void CydNavigation::reply(const es3proto::Frame& f, const uint8_t* data, size_t size)
{
    _size = es3proto::encode(f.sequence, f.type | es3proto::Reply, data, size, _tx);
    _offset = 0; _txSince = millis();
}
void CydNavigation::error(const es3proto::Frame& f, es3proto::ErrorCode code)
{
    const uint8_t value = code;
    _size = es3proto::encode(f.sequence, es3proto::Error, &value, 1, _tx);
    _offset = 0; _txSince = millis();
}
void CydNavigation::dispatch(const es3proto::Frame& f)
{
    using namespace es3proto;
    if (f.type == Hello) {
        if (f.length) { error(f, BadPayload); return; }
        reset(); _session = true; _lastRequest = millis();
        const uint8_t identity[] = {'C','Y','D'};
        reply(f, identity, sizeof(identity)); return;
    }
    if (!_session) { error(f, NoSession); return; }
    _lastRequest = millis();
    switch (f.type) {
    case Ping:
        if (f.length) error(f, BadPayload); else reply(f);
        break;
    case Status: {
        if (f.length) { error(f, BadPayload); return; }
        const uint32_t now = millis();
        const uint8_t status[] = {uint8_t(now), uint8_t(now >> 8), uint8_t(now >> 16), uint8_t(now >> 24),
            _graphics.activePage(), _graphics.brightnessDuty(), uint8_t(_graphics.sdAvailable()),
            uint8_t(_graphics.calibrationLoaded()), uint8_t(_audio.isPlaying())};
        reply(f, status, sizeof(status)); break;
    }
    case Capabilities: {
        if (f.length) { error(f, BadPayload); return; }
        uint8_t data[es3video::kCapabilitiesReplyBytes];
        const size_t n = es3video::encodeCapabilitiesReply(_receiver.capabilities(), data, sizeof(data));
        reply(f, data, n); break;
    }
    case es3map::CmdMapCapabilities: {
        if (f.length) { error(f, BadPayload); return; }
        uint8_t data[es3map::kMapCapabilitiesReplyBytes];
        const size_t n = es3map::encodeMapCapabilitiesReply(_receiver.mapCapabilities(),
            _receiver.capabilities(), data, sizeof(data));
        reply(f, data, n); break;
    }
    case es3map::CmdStartMap: {
        es3map::StartMapRequest request;
        if (!es3map::decodeStartMapRequest(f.payload, f.length, _receiver.mapCapabilities(),
                                          _receiver.capabilities(), request)) { error(f, BadPayload); return; }
        if (++_nextToken == 0) ++_nextToken;
        if (!_receiver.start(_nextToken, request)) { error(f, BadPayload); return; }
        _haveImage = false; _displayState = 0;
        es3map::StartMapReply r; r.sessionToken = _nextToken; r.initialGrant = 1;
        uint8_t data[es3map::kStartMapReplyBytes];
        const size_t n = es3map::encodeStartMapReply(r, data, sizeof(data));
        reply(f, data, n); break;
    }
    case es3map::CmdStopMap: {
        uint32_t token;
        if (!es3map::decodeStopMapRequest(f.payload, f.length, token)) { error(f, BadPayload); return; }
        if (token == _receiver.token) { _receiver.reset(); _haveImage = false; _displayState = 0; }
        reply(f); break;
    }
    case Page: {
        if (f.length != 1 + 5 * 24) { error(f, BadPayload); return; }
        char text[5][24]; memcpy(text, f.payload + 1, sizeof(text));
        if (!_graphics.updatePageText(f.payload[0], text)) error(f, BadPayload); else reply(f);
        break;
    }
    case Alert:
        if (f.length != 1 || f.payload[0] > 100) { error(f, BadPayload); return; }
        _audio.setVolume(f.payload[0]);
        if (!_audio.playWav("/alert.wav")) error(f, NotReady); else reply(f);
        break;
    case StopAudio:
        if (f.length) { error(f, BadPayload); return; }
        _audio.stop(); reply(f); break;
    default:
        // No framebuffer/PSRAM: decline camera and panning; host uses START_MAP.
        error(f, BadCommand); break;
    }
}
void CydNavigation::displayHealth()
{
    const bool visible = _graphics.mapVisible();
    if (!visible) _haveImage = false; // re-entered map has been cleared by the page renderer
    if (visible && !_wasVisible) _displayState = 0;
    _wasVisible = visible;
    if (!visible) return;
    const uint32_t age = millis() - _lastImage;
    const uint8_t state = !_session ? 1 : (!_receiver.token || !_haveImage ||
        age >= _receiver.config.placeholderAfterMs) ? 2 : age >= _receiver.config.staleAfterMs ? 3 : 4;
    if (state == _displayState) return;
    _displayState = state;
    if (state == 1) _graphics.mapPlaceholder("Jetson disconnected");
    else if (state == 2) _graphics.mapPlaceholder("Waiting for map");
    else if (state == 3) _graphics.mapStale();
}
void CydNavigation::update()
{
    const uint32_t now = millis();
    if ((_session && now - _lastRequest > 5000) || (_size && now - _txSince > 1000) || _receiver.fault()) reset();
    _receiver.view(_graphics.activePage(), _graphics.calibrationActive());
    _receiver.expire(now); _parser.expire(now);
    if (_size) {
        const int room = Serial.availableForWrite();
        if (room > 0) _offset += Serial.write(_tx + _offset, min<size_t>(_size - _offset, size_t(room)));
        if (_offset == _size) _size = _offset = 0;
    }
    if (!_size) {
        es3video::MuxParser::Item item;
        for (unsigned budget = 0; budget < 4096; ++budget) {
            _parser.feed(nullptr, 0, now, item);
            if (item.kind == es3video::MuxParser::ItemNone) {
                if (!Serial.available()) break;
                const int value = Serial.read(); if (value < 0) break;
                const uint8_t byte = uint8_t(value);
                _parser.feed(&byte, 1, now, item);
            }
            if (item.kind == es3video::MuxParser::ItemControl) { dispatch(item.control); break; }
            if (item.kind == es3video::MuxParser::ItemVideo && _session) _receiver.accept(item.video, now);
        }
    }
    if (_receiver.phase == cyd::MapReceiver::Ready) {
        const uint32_t began = micros();
        const bool ok = _decoder.draw(_receiver.jpeg, _receiver.bytes, _graphics);
        _receiver.finish(ok, micros() - began);
        _haveImage = ok;
        if (ok) { _lastImage = millis(); _displayState = 4; }
        else { _graphics.mapPlaceholder("Map decode failed"); _displayState = 0; }
    }
    if (!_size) {
        cyd::MapReceiver::Event e;
        if (_receiver.pop(e)) {
            memcpy(_tx, e.bytes, e.size); _size = e.size; _offset = 0; _txSince = millis();
        }
    }
    displayHealth();
}
