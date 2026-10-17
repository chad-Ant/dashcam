#include "../lib/es3Video/VideoState.h"
#include <assert.h>
#include <stdio.h>
using namespace es3video;
static uint8_t jpeg[2][kMaxJpegBytes];
static uint16_t pixels[2][320*240];
static const uint8_t sample[] = {0xff,0xd8,0xe3,0x28,0xe3,0x29,0xff,0xd9};
static uint8_t wire[kMaxRecordBytes];
static VideoState state;
static void drain() { VideoState::Event e; while (state.pop(e)) {} }
static bool event(uint8_t type, uint32_t frame, uint32_t value) {
    VideoState::Event e; bool found = false;
    while (state.pop(e)) {
        Record r; assert(decodeRecord(e.bytes,e.size,r) == ReasonOk);
        if (r.type == type && r.frameId == frame)
            found |= (type == RecordCredit ? detail::get32(r.payload) : r.payload[0]) == value;
    }
    return found;
}
static void deliver(size_t n, uint32_t now=0) {
    Record r; assert(decodeRecord(wire,n,r) == ReasonOk); state.accept(r,now);
}
static void start(uint32_t frame, uint32_t token=7) {
    StartInfo info; info.width=320; info.height=180; info.jpegBytes=sizeof(sample);
    deliver(encodeStart(token,frame,info,wire,sizeof(wire)));
}
static void complete(uint32_t frame) {
    start(frame);
    deliver(encodeChunk(7,frame,0,sample,sizeof(sample),wire,sizeof(wire)));
    deliver(encodeEnd(7,frame,crc32(sample,sizeof(sample)),wire,sizeof(wire)));
}
static void setup() {
    state.reset();
    for (int i=0;i<2;++i) {state.jpeg[i]=jpeg[i];state.pixels[i]=pixels[i];}
    StartVideoRequest config;
    config.viewportW=320;config.viewportH=180;config.viewportY=30;
    config.maxJpegBytes=kMaxJpegBytes;config.chunkBytes=1024;
    config.assemblyTimeoutMs=500;config.staleAfterMs=500;config.placeholderAfterMs=2000;
    state.setDemand(DemandActive);state.start(7,config);drain();
}
int main() {
    setup(); complete(1); assert(event(RecordResult,1,ReasonOk));
    assert(state.grant==1); VideoState::Job job; assert(state.takeJob(job));
    state.finishJob(job,true,3000); assert(state.grant==2);
    VideoState::Image image; assert(state.acquire(image));
    state.release(image,true,20000); assert(state.stats.presented==1);
    // Wrong offsets return the consumed grant once; trailing END/ABORT do not.
    start(2); deliver(encodeChunk(7,2,1,sample,2,wire,sizeof(wire)));
    assert(event(RecordResult,2,ReasonBadOffset)); assert(state.grant==3);
    deliver(encodeAbort(7,2,ReasonCancelled,wire,sizeof(wire))); assert(state.grant==3);
    start(2); assert(event(RecordResult,2,ReasonBadFrameId)); assert(state.grant==3);
    // CRC failure, assembly expiry, hidden START and stale token recovery.
    start(3); deliver(encodeChunk(7,3,0,sample,sizeof(sample),wire,sizeof(wire)));
    deliver(encodeEnd(7,3,0,wire,sizeof(wire))); assert(event(RecordResult,3,ReasonBadCrc));
    start(4); state.expire(500); assert(event(RecordResult,4,ReasonAssemblyTimeout));
    state.setDemand(DemandHidden); start(5); assert(event(RecordResult,5,ReasonHidden));
    const uint32_t grant=state.grant; start(6,6); assert(state.grant==grant);
    state.setDemand(DemandActive); complete(6); assert(state.takeJob(job));
    state.finishJob(job,false,100); assert(event(RecordResult,6,ReasonDecodeFailed));
    // NoCredit cannot mint credits or replace the current assembly.
    start(7);start(8);assert(event(RecordResult,8,ReasonNoCredit));
    deliver(encodeAbort(7,7,ReasonCancelled,wire,sizeof(wire)));
    assert(event(RecordResult,7,ReasonCancelled));
    // A reconnect cannot recycle a buffer still owned by decoder or TFT.
    complete(8);assert(state.takeJob(job));
    const int input=job.jpeg, output=job.image;
    const auto config=state.config; state.start(8,config);
    assert(state.inputs[input].state==VideoState::Decoding);
    assert(state.images[output].state==VideoState::Decoding);
    drain();state.finishJob(job,true,100);assert(state.grant==1);
    VideoState::Event e; assert(!state.pop(e));assert(!state.acquire(image));
    setup();complete(1);assert(state.takeJob(job));state.finishJob(job,true,10);
    assert(state.acquire(image));state.start(8,config);
    assert(state.images[image.slot].state==VideoState::Drawing);
    drain();state.release(image,true,100);assert(!state.pop(e));
    // Newest ready frame replaces waiting output, never a drawing lease.
    setup();complete(1);assert(state.takeJob(job));state.finishJob(job,true,1);drain();
    complete(2);assert(state.takeJob(job));state.finishJob(job,true,1);
    assert(event(RecordResult,1,ReasonSuperseded));assert(state.acquire(image));assert(image.frame==2);
    state.release(image,true,1);
    // Hidden/calibration releases incomplete frames and cancels a decode result.
    complete(3);assert(state.takeJob(job));state.setDemand(DemandCalibrating);
    state.finishJob(job,true,1);assert(event(RecordResult,3,ReasonHidden));
    // Header limits are checked before copying and credited rejects recover.
    setup();
    uint8_t payload[16]{};
    detail::put16(payload, 321); detail::put16(payload+2, 180); detail::put32(payload+4, 8);
    state.accept(makeRecord(RecordStart,7,1,payload,sizeof(payload)),0);
    assert(event(RecordResult,1,ReasonBadDimensions));assert(state.grant==2);
    detail::put16(payload,320);detail::put32(payload+4,65537);
    state.accept(makeRecord(RecordStart,7,2,payload,sizeof(payload)),0);
    assert(event(RecordResult,2,ReasonTooLarge));assert(state.grant==3);
    uint8_t oversized[1025]{};
    start(3);deliver(encodeChunk(7,3,0,oversized,sizeof(oversized),wire,sizeof(wire)));
    assert(event(RecordResult,3,ReasonTooLarge));assert(state.grant==4);
    start(4);deliver(encodeEnd(7,4,0,wire,sizeof(wire)));
    assert(event(RecordResult,4,ReasonBadLength));assert(state.grant==5);
    // A well-framed ABORT(Ok) is a field error, never an assembly-success ACK.
    start(5); const uint8_t okReason = ReasonOk;
    state.accept(makeRecord(RecordAbort,7,5,&okReason,1),0);
    assert(event(RecordResult,5,ReasonMalformed));assert(state.grant==6);
    // A view/acquire race must not consume the replacement session's image.
    setup(); const auto replacementConfig = state.config;
    state.start(6,replacementConfig); const uint32_t oldViewToken = state.token;
    state.start(7,replacementConfig); // USB replaces the session after the view snapshot
    complete(1); assert(state.takeJob(job)); state.finishJob(job,true,1); drain();
    assert(!state.acquire(image,oldViewToken));
    assert(state.acquire(image,7) && image.frame==1);
    state.release(image,true,1); drain();
    // Lease returns are idempotent and cannot act on a reused slot.
    setup(); complete(1); assert(state.takeJob(job));
    const auto oldJob = job;
    VideoState::Image decoderBuffer;
    decoderBuffer.slot=job.image; decoderBuffer.token=job.token; decoderBuffer.frame=job.frame;
    decoderBuffer.pixels=state.pixels[job.image]; drain();
    state.present(decoderBuffer,1); state.release(decoderBuffer,true,1);
    assert(state.images[job.image].state==VideoState::Decoding && !state.pop(e));
    assert(!state.takeJob(job)); // do not overwrite a live decoder handle
    state.finishJob(job,true,1); drain();
    state.finishJob(oldJob,true,1); assert(state.grant==2 && !state.pop(e));
    assert(state.acquire(image)); const auto held = image;
    assert(!state.acquire(image) && image.slot==held.slot && image.frame==held.frame);
    auto badImage = image; badImage.slot=4; state.release(badImage,true,1);
    assert(badImage.slot==-1 && state.images[image.slot].state==VideoState::Drawing);
    VideoState::Job badJob; badJob.jpeg=2; badJob.image=4;
    state.finishJob(badJob,true,1); assert(state.grant==2 && !state.pop(e));
    state.present(image,1); drain();
    state.present(image,1); assert(!state.pop(e));
    auto staleImage = image;
    state.release(image,true,1); assert(!state.pop(e));
    complete(2); assert(state.takeJob(job)); drain();
    state.finishJob(oldJob,true,1); // old handle while the same input/output decode a new frame
    assert(state.inputs[job.jpeg].state==VideoState::Decoding && state.grant==2);
    state.finishJob(job,true,1); drain(); assert(state.acquire(image));
    state.present(staleImage,1); state.release(staleImage,false,0);
    assert(state.images[image.slot].state==VideoState::Drawing && !state.pop(e));
    complete(3); assert(state.takeJob(job)); state.finishJob(job,true,1); drain();
    const auto beforeAcquire = image;
    assert(!state.acquire(image) && image.frame==beforeAcquire.frame && image.slot==beforeAcquire.slot);
    VideoState::Image waiting; assert(state.acquire(waiting) && waiting.frame==3);
    state.release(waiting,false,0); drain();
    state.setDemand(DemandHidden); drain();
    state.present(image,1); assert(!state.pop(e));
    state.release(image,true,1); assert(event(RecordResult,2,ReasonHidden));
    // Millis rollover must preserve the exact assembly timeout boundary.
    setup(); StartInfo rollover; rollover.width=320; rollover.height=180;
    rollover.jpegBytes=sizeof(sample);
    deliver(encodeStart(7,1,rollover,wire,sizeof(wire)),UINT32_MAX-200);
    state.expire(298); assert(state.grant==1);
    state.expire(299); assert(state.grant==2);
    assert(event(RecordResult,1,ReasonAssemblyTimeout));
    // Fixed event queue fails closed instead of silently losing grants.
    setup();for (unsigned i=1;i<40;++i) { state.setDemand(DemandHidden);start(i); }
    assert(state.fault());state.reset();assert(!state.fault());
    puts("video assembly/credit/ownership tests passed");
}
