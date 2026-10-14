#include <cassert>
#include <cstdio>
#include "MapSurface.h"

using namespace es3video;
using namespace es3map;
static SurfaceState state;
static uint8_t jpeg[2][kMaxJpegBytes];
static uint16_t pixels[2][320 * 240];
static uint8_t wire[128];
static const uint8_t sample[] = {1, 2, 3, 4};

static Capabilities caps() {
    Capabilities c;
    c.canvasWidth = c.maxWidth = 320; c.canvasHeight = c.maxHeight = 240;
    c.maxJpegBytes = kMaxJpegBytes; c.maxChunkBytes = 1024; c.maxCredits = 1;
    return c;
}
static StartMapRequest mapRequest() {
    StartMapRequest r;
    r.page = 5; r.viewportW = 320; r.viewportH = 224;
    r.maxJpegBytes = kMaxJpegBytes; r.chunkBytes = 1024; r.targetFps = 4;
    r.assemblyTimeoutMs = 500; r.staleAfterMs = 2000; r.placeholderAfterMs = 5000;
    r.generation = 0x12345678;
    return r;
}
static StartVideoRequest cameraRequest() {
    auto r = sessionLimits(mapRequest());
    r.viewportY = 30; r.viewportH = 180; r.targetFps = 15;
    return r;
}
static void drain() { VideoState::Event e; while (state.pop(e)) {} }
static void visibility(Demand expected, uint32_t token) {
    VideoState::Event e; assert(state.pop(e));
    Record r; assert(decodeRecord(e.bytes, e.size, r) == ReasonOk);
    assert(r.type == RecordVisibility && r.sessionToken == token && r.payload[0] == expected);
}
static void deliver(size_t n) {
    Record r; assert(n && decodeRecord(wire, n, r) == ReasonOk); state.accept(r, 0);
}
static void frame(uint32_t token, uint32_t id, uint16_t height = 224) {
    StartInfo i; i.width = 320; i.height = height; i.jpegBytes = sizeof(sample);
    deliver(encodeStart(token, id, i, wire, sizeof(wire)));
    deliver(encodeChunk(token, id, 0, sample, sizeof(sample), wire, sizeof(wire)));
    deliver(encodeEnd(token, id, crc32(sample, sizeof(sample)), wire, sizeof(wire)));
}
static bool reason(Reason expected) {
    bool found = false; VideoState::Event e;
    while (state.pop(e)) {
        Record r; assert(decodeRecord(e.bytes, e.size, r) == ReasonOk);
        if (r.type == RecordResult && r.payload[0] == expected) found = true;
    }
    return found;
}

int main() {
    const int edges[] = {0, 53, 106, 160, 213, 266, 320};
    for (unsigned i = 0; i <= 6; ++i) assert(tabEdge(i, 6) == edges[i]);
    for (int i = 0; i < 6; ++i)
        for (int x = edges[i]; x < edges[i + 1]; ++x) assert(tabAt(x, 6) == i);
    assert(tabAt(-1, 6) == -1 && tabAt(320, 6) == -1 && tabAt(10, 0) == -1);
    assert(tabAt(319, 5) == 4); // old layouts still have complete coverage
    const auto c = caps(); const auto m = capabilities(c); const auto request = mapRequest();
    assert(validMapCapabilities(m, c));
    assert(m.pageCount == 6 && m.mapPage == 5 && m.viewportX == 0 && m.viewportY == 0);
    assert(m.viewportW == 320 && m.viewportH == 224 && m.flags == 1);
    for (int i = 0; i < 2; ++i) { state.jpeg[i] = jpeg[i]; state.pixels[i] = pixels[i]; }

    // The initial binding cannot claim visible until graphics publishes a view.
    assert(state.startMap(1, request, c)); visibility(DemandCalibrating, 1);
    assert(state.binding.generation == request.generation && state.binding.page() == 5);
    state.view(0, false); visibility(DemandHidden, 1);
    state.view(5, false); visibility(DemandActive, 1);
    state.view(5, false); VideoState::Event e; assert(!state.pop(e));

    // Shared receiver: reject the other source without touching queued images,
    // credit, binding, generation or config. STOP of the wrong source is inert.
    assert(!state.startCamera(2, cameraRequest(), c));
    state.stopSurface(Surface::Camera, 1); assert(state.token == 1);
    state.stopSurface(Surface::Map, 99); assert(state.token == 1);
    state.stopSurface(Surface::Map, 0); assert(state.token == 1);
    assert(state.grant == 1 && state.config.viewportH == 224 && !state.pop(e));
    auto invalid = request; invalid.viewportY = 8;
    assert(!state.startMap(2, invalid, c));
    invalid = request; invalid.page = 1; assert(!state.startMap(2, invalid, c));
    invalid = request; invalid.mapProtocolVersion = 2; assert(!state.startMap(2, invalid, c));
    assert(state.token == 1 && state.binding.generation == request.generation);

    // Dimensions must match the map viewport exactly, even for valid records.
    frame(1, 1, 180); assert(reason(ReasonBadDimensions)); assert(state.grant == 2);
    frame(1, 2); assert(reason(ReasonOk));
    VideoState::Job job; assert(state.takeJob(job));
    assert(job.width == 320 && job.height == 224);
    state.finishJob(job, true, 100); drain();
    VideoState::Image image; assert(state.acquire(image));

    // A drawing lease survives a source switch; its old completion cannot
    // present or issue a credit in the camera session.
    const int held = image.slot;
    state.stopSurface(Surface::Map, 1);
    assert(state.token == 0 && state.images[held].state == VideoState::Working);
    assert(state.startCamera(3, cameraRequest(), c)); visibility(DemandHidden, 3);
    state.release(image, true, 100); assert(!state.pop(e));
    assert(state.stats.presented == 0 && state.grant == 1);
    assert(!state.startMap(4, request, c)); assert(state.token == 3);
    state.stopSurface(Surface::Map, 3); assert(state.token == 3);
    state.view(0, false); visibility(DemandActive, 3);
    frame(3, 1, 180); drain(); assert(state.takeJob(job));

    // A decoder lease also survives STOP + START_MAP; no old pixels published.
    state.stopSurface(Surface::Camera, 3);
    state.view(5, false);
    assert(state.startMap(5, request, c)); visibility(DemandActive, 5);
    assert(state.inputs[job.jpeg].state == VideoState::Working);
    state.finishJob(job, true, 100); assert(!state.pop(e) && !state.acquire(image));
    assert(state.grant == 1);

    // Calibration and information overlays hide the map. Hidden frames are
    // rejected with one returned grant, never retained for later presentation.
    state.view(5, true); visibility(DemandCalibrating, 5);
    frame(5, 1); assert(reason(ReasonHidden)); assert(state.grant == 2);
    state.view(5, false); visibility(DemandActive, 5);
    frame(5, 2); drain(); assert(state.takeJob(job));
    state.view(4, false); visibility(DemandHidden, 5);
    state.finishJob(job, true, 100); assert(reason(ReasonHidden));
    assert(!state.acquire(image));

    // Leave and return before a decode finishes: it still belongs to the old
    // view, even though the token and final demand happen to match again.
    state.view(5, false); visibility(DemandActive, 5);
    frame(5, 3); drain(); assert(state.takeJob(job));
    state.view(5, true); visibility(DemandCalibrating, 5);
    state.view(5, false); visibility(DemandActive, 5);
    state.finishJob(job, true, 100); assert(reason(ReasonHidden));
    assert(!state.acquire(image));

    // HELLO/disconnect reset cancels the map and leaves a fresh camera usable.
    state.reset(); state.view(0, false);
    assert(state.startCamera(6, cameraRequest(), c)); visibility(DemandActive, 6);
    state.stopSurface(Surface::Map, 5); assert(state.token == 6);
    state.reset(); state.view(5, false);
    assert(state.startMap(7, request, c)); visibility(DemandActive, 7);
    frame(7, 1); drain(); assert(state.takeJob(job)); state.finishJob(job, true, 10); drain();
    assert(state.acquire(image)); state.release(image, true, 10);
    assert(state.stats.presented == 1 && !state.fault());
    puts("map surface routing, geometry, visibility and lease checks passed");
}
