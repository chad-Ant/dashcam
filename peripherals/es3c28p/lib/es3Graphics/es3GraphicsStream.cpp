#include "GraphicsInternal.h"
#include "PixelCopy.h"

bool Es3Graphics::videoCoversTabs() const
{
    return uint32_t(_videoConfig.viewportY) + _videoConfig.viewportH > kTabTop;
}

void Es3Graphics::drawMapPage()
{
    panel.fillRect(0, 0, 320, es3map::kTabTop, ILI9341_BLACK);
    panel.setTextSize(1); panel.setTextColor(ILI9341_WHITE);
    panel.setCursor(2, 4);
    panel.print(_video && !_video->available() ? "Map unavailable - image memory" :
        (_hostConnected ? "Waiting for map" : "JETSON OFFLINE"));
    if (_mapInfo) {
        printCentered("MAP INFORMATION", 160, 40, ILI9341_CYAN);
        printCentered("Map data: OpenStreetMap contributors", 160, 72, ILI9341_WHITE);
        printCentered("Open Database License (ODbL)", 160, 96, ILI9341_WHITE);
        printCentered("openstreetmap.org/copyright", 160, 120, ILI9341_WHITE);
        printCentered("Tap to return to map", 160, 164, ILI9341_CYAN);
    }
    // Built-in fallback credit; actual streamed frames include host/provider
    // attribution. Never overwrite that strip when marking a retained map stale.
    static const uint8_t copyrightGlyph[] = {0x3c, 0x42, 0x99, 0xa1, 0xa1, 0x99, 0x42, 0x3c};
    panel.drawBitmap(2, 212, copyrightGlyph, 8, 8, ILI9341_WHITE);
    panel.setCursor(12, 212);
    panel.print("OpenStreetMap contributors");
    drawTabs();
}

void Es3Graphics::updateVideo()
{
    if (!_video) return;
    const uint32_t now = millis();
    const bool blocked = _calibrating || _calibrationTesting || _mapInfo;
    es3video::StartVideoRequest config;
    es3map::Binding binding;
    es3map::PanSnapshot pan;
    const uint32_t token = _video->view(_activePage, blocked, config, binding, &pan);
    const bool map = binding.surface == es3map::Surface::Map;
    if (token && !map) _videoEver = true;
    const bool visible = !blocked && _activePage == binding.page() &&
        (token || (_videoVisible && binding.surface == _binding.surface));
    if (token != _videoToken || visible != _videoVisible || binding.surface != _binding.surface) {
        _video->release(_videoImage, false, 0);
        clearPan();
        const bool keepLastFrame = !token && visible && _videoVisible;
        _videoToken = token; _videoVisible = visible;
        _videoConfig = config; _binding = binding;
        _videoStatus = 255; _navigation = false;
        if (!keepLastFrame) {
            _lastFrameMs = now;
            if (!_calibrating && !_calibrationTesting) drawPage();
        }
    }
    if (!visible) return;
    if (binding.panning) { updatePan(pan, now); return; }
    if (_navigation && int32_t(now - _navigationUntil) >= 0) {
        _navigation = false;
        panel.fillRect(0, kTabTop, kScreenWidth, kTabHeight, ILI9341_BLACK);
    }
    if (_videoToken && _videoImage.slot < 0 && _video->acquire(_videoImage, _videoToken)) {
        if (_videoImage.token != _videoToken) { _video->release(_videoImage, false, 0); return; }
        _videoRow = 0; _blitStarted = micros();
    }
    if (_videoImage.slot >= 0) {
        const int width = config.viewportW;
        const unsigned rows = es3video::copyFrameStrip(_videoStrip,
            sizeof(_videoStrip) / sizeof(_videoStrip[0]), _videoImage.pixels,
            _videoImage.width, _videoImage.height, config.viewportW, config.viewportH, _videoRow);
        if (!rows) {
            _video->release(_videoImage, false, 0);
            return; // Invalid geometry must never read pixels or claim PRESENTED.
        }
        // One bounded SPI transaction; touch and controls run between strips.
        panel.drawRGBBitmap(config.viewportX, config.viewportY + _videoRow, _videoStrip, width, rows);
        _videoRow += rows;
        if (_navigation && config.viewportY + _videoRow > kTabTop) drawTabs();
        if (_videoRow == config.viewportH) {
            _video->release(_videoImage, true, micros() - _blitStarted);
            _lastFrameMs = now; _videoStatus = 255;
        }
    }
    const uint32_t age = now - _lastFrameMs;
    const uint8_t status = age >= config.placeholderAfterMs ? 2 :
        (age >= config.staleAfterMs || !token || !_hostConnected ? 1 : 0);
    if (status != _videoStatus && _videoImage.slot < 0) {
        _videoStatus = status;
        if (map) {
            if (status) {
                panel.fillRect(0, 0, 320, es3map::kStatusHeight, ILI9341_BLACK);
                panel.setTextSize(1); panel.setTextColor(ILI9341_YELLOW);
                panel.setCursor(2, 4);
                panel.print(!_hostConnected ? "JETSON OFFLINE - MAP STALE" :
                    (status == 2 ? "Map unavailable - last known" : "MAP STALE - last known"));
            }
            // Fresh JPEG restores its own status. Neither stale threshold
            // clears imagery/attribution or paints over the tab strip.
        } else {
            if (status == 2) {
                panel.fillRect(config.viewportX, config.viewportY, config.viewportW, config.viewportH, ILI9341_BLACK);
                printCentered("Video unavailable", 160, 112, ILI9341_WHITE);
            }
            const int y = config.viewportY >= 10 ? 1 : config.viewportY;
            if (status || config.viewportY >= 10) panel.fillRect(0, y, 150, 9, ILI9341_BLACK);
            panel.setTextSize(1); panel.setTextColor(status ? ILI9341_YELLOW : ILI9341_GREEN);
            panel.setCursor(2, y);
            if (status) panel.print(_hostConnected ? "VIDEO STALE" : "JETSON OFFLINE");
            else if (config.viewportY >= 10) panel.print("LIVE USB");
        }
        if (!videoCoversTabs() || _navigation) drawTabs();
    }
}


void Es3Graphics::clearPan()
{
    _video->release(_panBackground, false, 0);
    _video->release(_panChrome, false, 0);
    _panDrawing = _panDrawMap = _panDrawChrome = false;
    _panBackgroundDirty = _panChromeDirty = false;
    _panSeenSeq = 0; _panRow = 0;
    _panRaster = es3map::PanRaster();
}

void Es3Graphics::updatePan(const es3map::PanSnapshot& pan, uint32_t now)
{
    if (_videoToken && pan.pose.seq > _panSeenSeq) {
        _panSeenSeq = pan.pose.seq;
        _lastFrameMs = pan.poseReceivedMs;
    }
    if (!_panDrawing && _videoToken) {
        // At most four ready slots. Swap only between draw passes; every
        // pointer remains leased while the graphics task reads its pixels.
        for (int n = 0; n < 4; ++n) {
            es3video::VideoState::Image next;
            if (!_video->acquire(next, _videoToken)) break;
            if (next.token != _videoToken || next.kind == es3map::PanImageNone) {
                _video->release(next, false, 0); continue;
            }
            auto& retained = next.kind == es3map::PanImageBackground ? _panBackground : _panChrome;
            _video->release(retained, false, 0); retained = next;
            if (next.kind == es3map::PanImageBackground) _panBackgroundDirty = true;
            else _panChromeDirty = true;
            _lastFrameMs = now;
        }
    }
    const auto& geometry = pan.session;
    const bool haveMap = _panBackground.slot >= 0 && pan.pose.seq;
    const bool complete = haveMap && _panChrome.slot >= 0;
    const uint32_t age = now - _lastFrameMs;
    const uint8_t status = age >= _videoConfig.placeholderAfterMs ? 2 :
        (age >= _videoConfig.staleAfterMs || !_videoToken || !_hostConnected ? 1 : (complete ? 0 : 3));
    if (!_panDrawing) {
        _panDrawChrome = _panChrome.slot >= 0 && (_panChromeDirty || (status == 0 && _videoStatus != 0));
        _panDrawMap = false;
        if (haveMap) {
            const auto view = es3map::computePanView(_panBackground.origin, _panBackground.width,
                _panBackground.height, geometry.mapW, geometry.mapH, pan.pose);
            _panDrawMap = _panBackgroundDirty || !_panRaster.sameView(view, pan.pose);
            if (_panDrawMap) _panRaster.prepare(view, pan.pose);
        }
        _panDrawing = _panDrawChrome || _panDrawMap;
        if (_panDrawing) { _panRow = 0; _blitStarted = micros(); }
    }
    if (_panDrawing) {
        // Advance over clean regions; perform only one <=8-row SPI transfer.
        const unsigned top = geometry.chromeTopH;
        const unsigned bottom = top + geometry.mapH;
        if (_panRow < top && !_panDrawChrome) _panRow = top;
        if (_panRow >= top && _panRow < bottom && !_panDrawMap) _panRow = bottom;
        if (_panRow >= bottom && !_panDrawChrome) _panRow = _videoConfig.viewportH;
        if (_panRow < _videoConfig.viewportH) {
            const bool mapRow = _panRow >= top && _panRow < bottom;
            const unsigned end = _panRow < top ? top : (mapRow ? bottom : _videoConfig.viewportH);
            const unsigned rows = min<unsigned>(8, end - _panRow);
            bool ok = true;
            if (mapRow) {
                ok = _panRaster.rows(_videoStrip, _panRow - top, rows, _panBackground.pixels,
                    _panBackground.width, _panBackground.height, geometry.mapW, geometry.mapH);
            } else {
                const unsigned sourceRow = _panRow < top ? _panRow : top + _panRow - bottom;
                memcpy(_videoStrip, _panChrome.pixels + sourceRow * _videoConfig.viewportW,
                       rows * _videoConfig.viewportW * sizeof(uint16_t));
            }
            if (ok) panel.drawRGBBitmap(_videoConfig.viewportX, _videoConfig.viewportY + _panRow,
                                       _videoStrip, _videoConfig.viewportW, rows);
            _panRow += rows;
        }
        if (_panRow >= _videoConfig.viewportH) {
            const uint32_t elapsed = micros() - _blitStarted;
            if (_panDrawMap) _video->present(_panBackground, elapsed);
            if (_panDrawChrome) _video->present(_panChrome, elapsed);
            if (_panDrawMap) _panBackgroundDirty = false;
            if (_panDrawChrome) _panChromeDirty = false;
            _panDrawing = false;
            // A strip refresh can overwrite a local notice; restore it below.
            if (_panDrawChrome) _videoStatus = 255;
        }
    }
    if (!_panDrawing && status != _videoStatus) {
        _videoStatus = status;
        if (status) {
            panel.fillRect(0, 0, 320, es3map::kStatusHeight, ILI9341_BLACK);
            panel.setTextSize(1); panel.setTextColor(ILI9341_YELLOW); panel.setCursor(2, 4);
            panel.print(!_hostConnected ? "JETSON OFFLINE - MAP STALE" :
                (status == 3 ? "Waiting for map" :
                 (status == 2 ? "Map unavailable - last known" : "MAP STALE - last known")));
        }
    }
}
