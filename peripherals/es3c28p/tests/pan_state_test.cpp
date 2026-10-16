#include <cassert>
#include <cstdio>
#include <algorithm>
#include "MapSurface.h"
#include "PanRaster.h"

using namespace es3video;
using namespace es3map;
static SurfaceState state;
static uint8_t jpeg[2][65536], wire[128];
static uint16_t bg[2][512 * 384], chrome[2][320 * 32];
static uint16_t output[320 * 192], strip[320 * 8 + 2];
static const uint8_t sample[] = {1,2,3,4};
static Capabilities caps() {
    Capabilities c; c.canvasWidth = c.maxWidth = 320; c.canvasHeight = c.maxHeight = 240;
    c.maxJpegBytes = 65536; c.maxChunkBytes = 1024; c.maxCredits = 1; return c;
}
static StartPanRequest request() {
    StartPanRequest p; p.backgroundW = 512; p.backgroundH = 384;
    auto& m = p.map; m.page = 5; m.viewportW = 320; m.viewportH = 224;
    m.maxJpegBytes = 65536; m.chunkBytes = 1024; m.targetFps = 4;
    m.assemblyTimeoutMs = 500; m.staleAfterMs = 2000; m.placeholderAfterMs = 5000;
    m.generation = 91; return p;
}
static void drain() { VideoState::Event e; while (state.pop(e)) {} }
static unsigned events(uint8_t type, uint32_t frame, unsigned value = 0) {
    unsigned count = 0; VideoState::Event e;
    while (state.pop(e)) {
        Record r; assert(decodeRecord(e.bytes,e.size,r) == ReasonOk);
        if (r.type == type && r.frameId == frame && (type != RecordResult || r.payload[0] == value)) ++count;
    }
    return count;
}
static void deliver(size_t size, uint32_t now = 0) {
    Record r; assert(size && decodeRecord(wire,size,r) == ReasonOk); state.accept(r,now);
}
static void startFrame(uint32_t id, bool isChrome = false) {
    StartInfo info; info.width = isChrome ? 320 : 512; info.height = isChrome ? 32 : 384;
    info.jpegBytes = sizeof(sample); info.sourceTimestampUs = packOrigin(1000,2000);
    deliver(encodeStart(state.token,id,info,wire,sizeof(wire)));
}
static void finishFrame(uint32_t id) {
    deliver(encodeChunk(state.token,id,0,sample,sizeof(sample),wire,sizeof(wire)));
    deliver(encodeEnd(state.token,id,crc32(sample,sizeof(sample)),wire,sizeof(wire)));
}
static VideoState::Job decode(uint32_t id, bool isChrome = false) {
    startFrame(id,isChrome); finishFrame(id);
    VideoState::Job job; assert(state.takeJob(job));
    assert(job.width == (isChrome ? 320 : 512) && job.height == (isChrome ? 32 : 384));
    return job;
}
static void pose(uint32_t seq, uint32_t now = 10, uint32_t token = 7) {
    MapPose p; p.seq = seq; p.xQ4 = 1256 * 16; p.yQ4 = 2192 * 16; p.marker = MarkerArrow;
    deliver(encodeMapPose(token,p,wire,sizeof(wire)),now);
}
static void render(PanRaster& raster) {
    for (unsigned row = 0; row < 192; row += 8) {
        strip[0] = 0xbeef; strip[2561] = 0xcafe;
        assert(raster.rows(strip+1,row,8,bg[0],512,384,320,192));
        assert(strip[0] == 0xbeef && strip[2561] == 0xcafe);
        std::copy(strip+1,strip+2561,output+row*320);
    }
}

int main() {
    for (int i=0; i<2; ++i) { state.jpeg[i]=jpeg[i]; state.pixels[i]=bg[i]; state.pixels[i+2]=chrome[i]; }
    const auto r = request();
    assert(validPanCapabilities(panCapabilities(),capabilities(caps()),caps()));
    assert(panCapabilities().maxPoseHz == 30);
    state.view(5,false); assert(state.startPan(7,r,caps())); drain();
    assert(state.binding.panning && state.panning && state.binding.generation == 91);
    auto bad = r; bad.backgroundW = 513;
    assert(!state.startPan(8,bad,caps())); assert(state.token == 7);
    bad = r; bad.backgroundH = 191;
    assert(!state.startPan(8,bad,caps()));
    auto camera = sessionLimits(r.map);
    assert(!state.startCamera(8,camera,caps()));
    state.stopSurface(Surface::Camera,7); assert(state.token == 7);

    // Interleave pose with an incomplete JPEG: it must not disturb assembly
    // offsets, consume/return credits, or acknowledge the pose.
    startFrame(1); const auto grant = state.grant;
    pose(1,100); assert(state.pose.seq == 1 && state.poseReceivedMs == 100);
    assert(state.grant == grant); VideoState::Event e; assert(!state.pop(e));
    pose(1,101); pose(0xffffffffu,102,9); // duplicate, foreign token
    assert(state.pose.seq == 1 && state.poseReceivedMs == 100);
    MapPose p; p.seq=2;
    const size_t size = encodeMapPose(7,p,wire,sizeof(wire));
    Record invalid; assert(decodeRecord(wire,size,invalid) == ReasonOk);
    uint8_t payload[16]; memcpy(payload,invalid.payload,16); payload[14]=4;
    invalid.payload=payload; state.accept(invalid,103);
    assert(state.pose.seq == 1 && state.poseReceivedMs == 100 && !state.pop(e));
    finishFrame(1); VideoState::Job job; assert(state.takeJob(job));
    assert(job.width == 512 && job.height == 384);
    state.finishJob(job,true,10); drain();
    job=decode(2,true); state.finishJob(job,true,10); drain();
    // New background supersedes only the waiting background; strips survive.
    job=decode(3); state.finishJob(job,true,10);
    assert(events(RecordResult,1,ReasonSuperseded) == 1);
    VideoState::Image background, strips;
    assert(state.acquire(background) && background.kind == PanImageBackground && background.frame == 3);
    assert(background.origin == packOrigin(1000,2000));
    assert(state.acquire(strips) && strips.kind == PanImageChrome && strips.frame == 2);
    assert(background.width == 512 && strips.width == 320);
    state.present(background,100); state.present(background,100);
    assert(events(RecordPresented,3) == 1);
    state.present(strips,100); assert(events(RecordPresented,2) == 1);
    // Both displayed images stay leased while another image decodes.
    job=decode(4); assert(job.image != background.slot && job.image != strips.slot);
    state.finishJob(job,true,10); drain();
    state.release(background,false,0); assert(!state.pop(e));
    assert(state.acquire(background) && background.frame == 4);
    job=decode(5,true); state.finishJob(job,true,10); drain();
    job=decode(6,true); state.finishJob(job,true,10);
    assert(events(RecordResult,5,ReasonSuperseded) == 1);
    assert(state.images[background.slot].state == VideoState::Working);
    assert(state.images[strips.slot].state == VideoState::Working);

    // Hiding a map suppresses poses. A source replacement cannot reclaim a
    // retained image or a decoder lease, nor publish old work into the new token.
    state.view(4,false); drain(); pose(2,200);
    assert(state.pose.seq == 1 && state.poseReceivedMs == 100);
    state.view(5,false); drain(); pose(2,201); assert(state.pose.seq == 2);
    job=decode(7); drain();
    state.stopSurface(Surface::Map,7);
    assert(state.startMap(8,r.map,caps()) && !state.panning && !state.binding.panning);
    drain(); state.finishJob(job,true,10); assert(!state.pop(e) && state.grant == 1);
    state.release(background,true,100); state.release(strips,true,100); assert(!state.pop(e));
    pose(3,202,8); assert(state.pose.seq == 0 && !state.pop(e)); // whole-map mode ignores poses
    state.stopSurface(Surface::Map,8); assert(state.startPan(9,r,caps())); drain();
    pose(0xffffffffu,300,9); pose(1,301,9);
    assert(state.pose.seq == 0xffffffffu && state.poseReceivedMs == 300);
    state.reset(); assert(state.startPan(10,r,caps())); drain(); pose(1,302,10);
    assert(state.pose.seq == 1); drain();
    job=decode(1); state.finishJob(job,true,1); drain();
    VideoState::Image oldImage, newImage;
    assert(state.acquire(oldImage));
    job=decode(2); state.finishJob(job,true,1); drain(); assert(state.acquire(newImage));
    startFrame(3); finishFrame(3); drain(); const auto heldGrant=state.grant;
    assert(!state.takeJob(job) && state.grant == heldGrant && !state.pop(e));
    state.release(oldImage,false,0); drain(); assert(state.takeJob(job));
    state.reset(); state.finishJob(job,true,1); state.release(newImage,false,0);
    assert(!state.pop(e));

    // Real compositor: origin/crop alignment, markers, exact edge clipping,
    // and output canaries. Rendering does not alter the retained background.
    for (unsigned y=0; y<384; ++y) for (unsigned x=0; x<512; ++x) bg[0][y*512+x]=uint16_t(y*512+x);
    p.seq=1; p.xQ4=1256*16; p.yQ4=2192*16; p.marker=MarkerNone;
    auto view=computePanView(packOrigin(1000,2000),512,384,320,192,p);
    assert(view.cropX==96 && view.cropY==96);
    PanRaster raster; raster.prepare(view,p); render(raster);
    assert(output[0]==bg[0][96*512+96] && output[191*320+319]==bg[0][287*512+415]);
    assert(raster.sameView(view,p));
    std::fill(bg[0],bg[0]+512*384,0x0421);
    p.marker=MarkerDot; view=computePanView(packOrigin(1000,2000),512,384,320,192,p);
    raster.prepare(view,p); render(raster);
    assert(output[96*320+160]==0x133f && output[96*320+167]==0xffff);
    assert(output[96*320+170]==0x0421 && bg[0][192*512+256]==0x0421);
    p.marker=MarkerLastKnown; raster.prepare(view,p); render(raster);
    assert(output[96*320+160]==0xffff && output[96*320+166]==0x7bcf);
    p.marker=MarkerArrow;
    const int tipX[]={160,172,160,147}, tipY[]={83,96,108,96};
    for (int i=0;i<4;++i) {
        p.courseCdeg=i*9000; raster.prepare(view,p); render(raster);
        assert(output[tipY[i]*320+tipX[i]]==0xffff);
        assert(output[96*320+160]==0x133f);
    }
    p.marker=MarkerDot; p.xQ4=1000*16; p.yQ4=2000*16;
    view=computePanView(packOrigin(1000,2000),512,384,320,192,p);
    assert(view.cropX==0 && view.cropY==0 && view.markerVisible);
    raster.prepare(view,p); render(raster); assert(output[0]==0x133f);
    p.xQ4=0; p.yQ4=0; view=computePanView(packOrigin(1000,2000),512,384,320,192,p);
    assert(!view.markerVisible); raster.prepare(view,p); render(raster);
    assert(output[0]==0x0421);
    assert(!raster.rows(strip,190,8,bg[0],512,384,320,192));
    view.cropX=193; raster.prepare(view,p);
    assert(!raster.rows(strip,0,8,bg[0],512,384,320,192));
    puts("panning receiver, poses, retained leases, crops and marker raster checks passed");
}
