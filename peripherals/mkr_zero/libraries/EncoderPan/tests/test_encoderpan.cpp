#include "EncoderPan.h"
#include "AS5048B.h"
#include "GatedPwm.h"
#include <cstdio>
#include <cstdlib>
#include <limits>
using namespace encoderpan;
static unsigned checks = 0;
#define CHECK(x) do { ++checks; if (!(x)) { std::fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#x); std::exit(1); } } while (0)
static Config config() {
    Config c; c.calibrated = true; c.encoderMaxSpeedMdegS = 180000;
    c.settleToleranceMdeg = 500; c.followingErrorUs = 200000;
    return c;
}
static EncoderSample sample(uint32_t t, int32_t angle = 0) {
    EncoderSample s; s.timeUs=t; s.angleMdeg=angle; s.errorMdeg=50;
    s.healthy=true; s.referenced=true; return s;
}
static Command cmd(uint32_t seq, uint32_t t, int32_t angle = 0, uint32_t ttl = 500000) {
    Command c; c.sequence=seq; c.issuedUs=t; c.targetMdeg=angle; c.lifetimeUs=ttl; return c;
}
static void start(Controller& c, uint32_t t = 1000, int32_t target = 0) {
    CHECK(c.observe(sample(t),t)); CHECK(c.arm(t)); CHECK(c.request(cmd(1,t,target),t));
}
static void faultsAndLimits() {
    Controller bad{Config()}; CHECK(!bad.configured()); CHECK(!bad.arm(0)); CHECK(!bad.clearFault());
    Config cfg=config(); cfg.pulseAtMaxUs=2500; CHECK(!Controller(cfg).configured());
    cfg=config(); cfg.maxMdeg=cfg.minMdeg; CHECK(!Controller(cfg).configured());
    Controller c(config()); CHECK(!c.arm(100)); CHECK(!c.tick(100).enabled);
    start(c); CHECK(c.tick(1000).pulseUs==1500);
    CHECK(!c.request(cmd(2,1000,46000),1000)); CHECK(c.fault()==Fault::TravelLimit);
    CHECK(!c.tick(1001).enabled); CHECK(c.clearFault()); CHECK(!c.arm(1001));
    CHECK(c.observe(sample(2000),2000)); CHECK(c.arm(2000));
    CHECK(!c.request(cmd(1,2000),2000)); CHECK(c.fault()==Fault::CommandOrder);
    Controller e(config()); start(e); CHECK(!e.tick(32000).enabled); CHECK(e.fault()==Fault::EncoderStale);
    Controller h(config()); start(h); auto s=sample(2000); s.healthy=false;
    CHECK(!h.observe(s,2000)); CHECK(h.fault()==Fault::EncoderInvalid);
    Controller r(config()); start(r); s=sample(2000); s.referenced=false;
    CHECK(!r.observe(s,2000)); CHECK(r.fault()==Fault::EncoderInvalid);
    Controller o(config()); start(o); CHECK(!o.observe(sample(1000),1000)); CHECK(o.fault()==Fault::EncoderOrder);
    Controller future(config()); CHECK(!future.observe(sample(2000),1000));
    Controller j(config()); start(j); CHECK(!j.observe(sample(2000,5000),2000)); CHECK(j.fault()==Fault::EncoderJump);
    Controller travel(config()); CHECK(!travel.observe(sample(1000,45000),1000));
    Controller error(config()); s=sample(1000); s.errorMdeg=5001; CHECK(!error.observe(s,1000));
    Controller timeout(config()); start(timeout); CHECK(!timeout.request(cmd(2,2000,0,1000),4000));
    CHECK(timeout.fault()==Fault::CommandExpired);
    Controller futureCommand(config()); start(futureCommand);
    CHECK(!futureCommand.request(cmd(2,2000),1000));
    CHECK(futureCommand.fault()==Fault::CommandExpired);
    Controller extreme(config()); s=sample(1000); s.angleMdeg=std::numeric_limits<int32_t>::min();
    CHECK(!extreme.observe(s,1000)); CHECK(extreme.fault()==Fault::TravelLimit);
    Controller replay(config()); start(replay); CHECK(!replay.request(cmd(1,2000),2000));
    CHECK(replay.fault()==Fault::CommandOrder);
    Controller expiry(config()); CHECK(expiry.observe(sample(1000),1000)); CHECK(expiry.arm(1000));
    CHECK(expiry.request(cmd(1,1000,0,10000),1000)); CHECK(expiry.observe(sample(11000),11000));
    CHECK(!expiry.request(cmd(2,11000),11000)); CHECK(expiry.fault()==Fault::CommandExpired);
    Controller hw(config()); start(hw); hw.outputFailed(); CHECK(!hw.tick(1000).enabled);
    CHECK(hw.fault()==Fault::OutputFailure);
    cfg=config(); cfg.maxTickGapUs=10000;
    Controller gap(cfg); start(gap); CHECK(gap.observe(sample(21000),21000));
    CHECK(!gap.tick(21000).enabled); CHECK(gap.fault()==Fault::SchedulerGap);
}
static void movementAndStability() {
    Controller c(config()); start(c,1000,30000);
    CHECK(c.observe(sample(11000),11000)); CHECK(c.tick(11000).pulseUs==1502);
    CHECK(c.observe(sample(21000,300),21000)); CHECK(c.tick(21000).pulseUs==1505);
    c.stop(); CHECK(!c.tick(21000).enabled); CHECK(!c.feedback(21000).settled);
    Config reverse=config(); reverse.pulseAtMinUs=1900; reverse.pulseAtMaxUs=1100;
    Controller reversed(reverse); start(reversed,1000,30000);
    CHECK(reversed.observe(sample(11000),11000)); CHECK(reversed.tick(11000).pulseUs==1498);
    Controller stable(config()); start(stable);
    for (uint32_t t=11000;t<=231000;t+=10000) {
        CHECK(stable.observe(sample(t),t)); CHECK(stable.tick(t).enabled);
    }
    auto f=stable.feedback(231000); CHECK(f.valid); CHECK(f.settled);
    CHECK(f.angleMdeg==0); CHECK(f.validUntilUs==231000); CHECK(f.stableSinceUs==11000);
    CHECK(f.errorMdeg==2750); CHECK(!stable.feedback(500001).settled);
    CHECK(stable.request(cmd(2,231000,10000),231000)); CHECK(!stable.feedback(231000).settled);
    Controller stalled(config()); start(stalled,1000,40000);
    for (uint32_t t=11000;t<450000 && stalled.fault()==Fault::None;t+=10000) {
        CHECK(stalled.observe(sample(t),t));
        CHECK(stalled.request(cmd(t,t,40000),t)); // renewals must not mask a stall
        (void)stalled.tick(t);
    }
    CHECK(stalled.fault()==Fault::FollowingError);
    Controller wrap(config()); const uint32_t t=0xfffff000u; start(wrap,t);
    CHECK(wrap.observe(sample(t+10000),t+10000)); CHECK(wrap.tick(t+10000).enabled);
    CHECK(wrap.feedback(t+10000).valid);
    Controller sequences(config()); CHECK(sequences.observe(sample(1000),1000)); CHECK(sequences.arm(1000));
    CHECK(sequences.request(cmd(0xffffffffu,1000),1000)); CHECK(sequences.request(cmd(0,1000),1000));
    CHECK(!sequences.request(cmd(0x80000000u,1000),1000));
    // Negative trim direction and narrow fractional rate: no steps lost at 1ms.
    Config slow=config(); slow.maxSpeedMdegS=1;
    Controller fractional(slow); start(fractional,1000,1000);
    for (uint32_t now=2000;now<100000;now+=1000) {
        CHECK(fractional.observe(sample(now),now)); CHECK(fractional.tick(now).enabled);
    }
    Config trim=config(); trim.trimGainPermille=1000; trim.maxTrimMdeg=2000;
    Controller trimmed(trim); start(trimmed,1000,20000); uint16_t previous=1500;
    for (uint32_t now=11000;now<200000;now+=10000) {
        CHECK(trimmed.observe(sample(now,static_cast<int32_t>((now-1000)*3/100)),now));
        auto out=trimmed.tick(now); CHECK(out.enabled);
        CHECK(out.pulseUs>=previous && out.pulseUs-previous<=3); previous=out.pulseUs;
    }
}
struct FakeClock { uint32_t t=0; uint32_t now() const { return t; } };
struct FakeBus {
    FakeClock& clock;
    explicit FakeBus(FakeClock& source) : clock(source) {}
    uint16_t counts=8192; uint8_t diag=1; bool broken=false, tear=false;
    unsigned calls=0, tearReadsLeft=0; uint32_t readUs=50;
    bool read(uint8_t address,uint8_t reg,uint8_t& value) {
        ++calls; clock.t+=readUs;
        CHECK(address==0x40);
        if (broken) return false;
        if (reg==0xfb) value=diag;
        else if (reg==0xfe) {
            value=static_cast<uint8_t>(counts>>6);
            if (tear || tearReadsLeft>0) counts+=64;
            if (tearReadsLeft>0) --tearReadsLeft;
        }
        else if (reg==0xff) value=static_cast<uint8_t>((counts&0x3f)|0xc0); // undefined bits
        else { CHECK(false); }
        return true;
    }
};
static void encoderTests() {
    CHECK(AS5048B::angle(0,0,false)==0); CHECK(AS5048B::angle(4096,0,false)==90000);
    CHECK(AS5048B::angle(8192,0,false)==-180000);
    CHECK(AS5048B::angle(16383,0,false)==-21); CHECK(AS5048B::angle(0,16383,false)==21);
    CHECK(AS5048B::angle(4096,0,true)==-90000);
    CHECK(AS5048B::healthy(0xf1));
    for (unsigned d=0;d<16;++d) CHECK(AS5048B::healthy(static_cast<uint8_t>(d))==(d==1));
    AS5048Config cfg; cfg.calibrated=true; cfg.zeroCounts=8192;
    AS5048B encoder(cfg); FakeClock clock; FakeBus bus{clock};
    auto s=encoder.sample(bus,clock); CHECK(s.healthy && s.referenced); CHECK(s.angleMdeg==0);
    CHECK(s.rawCounts==8192);
    CHECK(s.timeUs==250); CHECK(s.errorMdeg==1717);
    bus.diag=3; CHECK(!encoder.sample(bus,clock).healthy);
    bus.diag=0; CHECK(!encoder.sample(bus,clock).healthy);
    bus.diag=0xf1; CHECK(encoder.sample(bus,clock).healthy);
    bus.broken=true; CHECK(!encoder.sample(bus,clock).healthy); bus.broken=false;
    bus.tear=true; bus.calls=0; CHECK(!encoder.sample(bus,clock).healthy); CHECK(bus.calls<=11);
    bus.tear=false; bus.readUs=1000; CHECK(!encoder.sample(bus,clock).healthy);
    bus.readUs=50; bus.tearReadsLeft=1; CHECK(encoder.sample(bus,clock).healthy);
    bus.counts=0; CHECK(encoder.sample(bus,clock).healthy); // zero itself isn't invalid
    clock.t=0xfffffff0; CHECK(encoder.sample(bus,clock).healthy); // acquisition wraps
    cfg.calibrated=false; CHECK(!AS5048B(cfg).sample(bus,clock).referenced);
    cfg.address=0; bus.calls=0; CHECK(!AS5048B(cfg).sample(bus,clock).healthy); CHECK(bus.calls==0);
}
struct FakePwm {
    bool gate=false, attached=false, attachOk=true, writeOk=true, low=true;
    unsigned attaches=0; uint16_t us=0;
    void gateOff() { gate=false; }
    void detach() { CHECK(attached); attached=false; }
    bool attach() { ++attaches; attached=attachOk; CHECK(!gate); return attachOk; }
    bool write(uint16_t pulse) { CHECK(attached); us=pulse; return writeOk; }
    bool enableWhenLow() { if (low) gate=true; return gate; }
};
static void pwmTests() {
    FakePwm hw; GatedPwm<FakePwm> output(hw); Output on; on.enabled=true; on.pulseUs=1600;
    output.begin(false); CHECK(!output.apply(on,0)); CHECK(!hw.gate && !hw.attached);
    output.begin(true); CHECK(output.apply(on,0)); CHECK(hw.attached && !hw.gate);
    CHECK(output.apply(on,39999)); CHECK(!hw.gate);
    CHECK(output.apply(on,40000)); CHECK(hw.gate && hw.us==1600);
    output.off(); CHECK(!hw.gate && !hw.attached);
    hw.attachOk=false; CHECK(!output.apply(on,50000)); CHECK(!hw.gate);
    hw.attachOk=true; output.begin(true); hw.writeOk=false;
    CHECK(!output.apply(on,60000)); CHECK(!hw.gate && !hw.attached);
    hw.writeOk=true; output.begin(true); hw.low=false;
    CHECK(output.apply(on,100000)); CHECK(!output.apply(on,200001)); CHECK(!hw.gate);
    output.begin(true); hw.low=true;
    CHECK(output.apply(on,0xfffffff0)); CHECK(output.apply(on,uint32_t(0xfffffff0u+40000u)));
    CHECK(hw.gate); CHECK(output.apply(Output(),40000)); CHECK(!hw.gate);
    on.pulseUs=0; CHECK(!output.apply(on,50000));
}
int main() {
    faultsAndLimits(); movementAndStability(); encoderTests(); pwmTests();
    std::printf("EncoderPan: %u checks passed\n",checks);
}
