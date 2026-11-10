#include "GRF250.h"
#include <cassert>
#include <cstdio>
#include <vector>

static std::vector<uint8_t> packet(uint8_t id, const std::vector<uint8_t>& data) {
    const uint16_t flags = static_cast<uint16_t>((data.size()+1) << 6);
    std::vector<uint8_t> result(data.size()+6);
    result[0]=0xaa; result[1]=static_cast<uint8_t>(flags);
    result[2]=static_cast<uint8_t>(flags>>8); result[3]=id;
    for (size_t i=0;i<data.size();++i) result[4+i]=data[i];
    const uint16_t crc = grf250::crc(result.data(),result.size()-2);
    result[result.size()-2]=static_cast<uint8_t>(crc); result.back()=static_cast<uint8_t>(crc>>8);
    return result;
}
static std::vector<uint8_t> u32(uint32_t v) {
    std::vector<uint8_t> result(4); grf250::put32(result.data(),v); return result;
}
static bool receive(grf250::Driver& driver, const std::vector<uint8_t>& wire, uint32_t now, grf250::Sample& out) {
    bool got = false;
    for (const uint8_t byte : wire) if (driver.receive(byte,now,out)) got = true;
    return got;
}
static void configure(grf250::Driver& driver, uint32_t now, uint32_t stream = 5) {
    uint8_t command[10]; grf250::Sample sample;
    for (unsigned i=0; i<12; ++i) {
        const size_t n = driver.nextCommand(now,command,sizeof(command));
        assert(n == (i >= 2 && i%2 == 0 ? 10u : 6u));
        std::vector<uint8_t> data;
        if (i == 0) { data.resize(16); memcpy(data.data(),"GRF250",6); }
        else if (i == 1) data = u32(0x00050200);
        else {
            const uint32_t expected = i < 4 ? 0u : i < 6 ? 0x2du : i < 8 ? 1u : i < 10 ? 20u : stream;
            if (n == 10) assert(grf250::get32(command+4) == expected);
            data = u32(expected);
        }
        assert(!receive(driver,packet(command[3],data),now,sample));
    }
    assert(driver.ready());
}
int main() {
    const uint8_t golden[] = {'1','2','3','4','5','6','7','8','9'};
    assert(grf250::crc(golden,9) == 0x31c3);
    uint8_t output[10];
    assert(grf250::command(0,false,0,output,5) == 0);
    assert(grf250::command(0,false,0,nullptr,10) == 0);
    assert(grf250::command(0,false,0,output,10) == 6);
    assert(output[0] == 0xaa && output[1] == 0x40 && output[2] == 0 && output[3] == 0);
    assert(grf250::signed32(u32(0xfffffff6u).data()) == -10);
    grf250::Parser parser; grf250::Packet parsed;
    auto wire = packet(2,u32(1234));
    for (size_t i=0;i<wire.size();++i) assert(parser.feed(wire[i],100,parsed) == (i+1==wire.size()));
    assert(parsed.size==5 && grf250::get32(parsed.payload+1)==1234);
    // Every single-bit mutation of this frame must be rejected.
    for (size_t i=0;i<wire.size();++i) {
        for (unsigned bit=0;bit<8;++bit) {
            auto changed=wire; changed[i] ^= static_cast<uint8_t>(1u<<bit);
            grf250::Parser isolated;
            for (const auto b:changed) assert(!isolated.feed(b,1,parsed));
        }
    }
    // Unknown reserved flag bits are accepted when the CRC is correct.
    auto reserved=wire; reserved[1] |= 0x3eu;
    const uint16_t reservedCrc=grf250::crc(reserved.data(),reserved.size()-2);
    reserved[reserved.size()-2]=static_cast<uint8_t>(reservedCrc);
    reserved.back()=static_cast<uint8_t>(reservedCrc>>8);
    grf250::Parser reservedParser; unsigned reservedCount=0;
    for (const auto b:reserved) reservedCount += reservedParser.feed(b,10,parsed);
    assert(reservedCount==1);
    auto corrupt = wire; corrupt.back() ^= 1;
    for (const auto b: corrupt) assert(!parser.feed(b,200,parsed));
    unsigned count=0;
    for (const auto b: wire) count += parser.feed(b,300,parsed);
    assert(count==1 && parser.errors > 0);
    // Partial frame gap and uint32 microsecond rollover.
    parser.reset(); assert(!parser.feed(0xaa,0xfffffff0u,parsed));
    assert(!parser.feed(0x40,0xfffffff5u,parsed));
    for (const auto b: wire) count += parser.feed(b,30000,parsed);
    assert(count==2);
    // Arbitrary junk remains bounded and recovers after a frame-gap timeout.
    uint32_t seed=13;
    for (unsigned i=0;i<100000;++i) { seed=seed*1664525u+1013904223u; (void)parser.feed(seed>>24,i,parsed); }
    count=0; for (const auto b: wire) count += parser.feed(b,200000,parsed);
    assert(count==1);
    grf250::Driver driver; grf250::Sample sample;
    assert(!receive(driver,packet(44,std::vector<uint8_t>(16)),100,sample));
    configure(driver,0xfffffff0u);
    std::vector<uint8_t> fields;
    for (const auto value: {1000u,25u,1000u,25u}) { auto bytes=u32(value); fields.insert(fields.end(),bytes.begin(),bytes.end()); }
    assert(receive(driver,packet(44,fields),100,sample));
    assert(sample.firstDm==1000 && sample.firstDb==25 && sample.sequence==1); // 100 m
    assert(!receive(driver,packet(44,std::vector<uint8_t>(12)),200,sample));
    grf250::put32(fields.data(),0xfffffff6u);
    assert(receive(driver,packet(44,fields),300,sample));
    assert(sample.firstDm==-10 && sample.sequence==2);
    // Real GRF-250 bytes (fw 2.0.5, 2026-11-09): stream frame for a taped 2.9 m
    // target, mask 0x2D. Proves CRC, flags and field order on actual hardware data.
    const std::vector<uint8_t> real44 = {0xaa,0x40,0x04,0x2c,0x1d,0x00,0x00,0x00,0x43,0x00,0x00,0x00,
                                         0x1d,0x00,0x00,0x00,0x43,0x00,0x00,0x00,0xfd,0xff};
    assert(receive(driver,real44,400,sample));
    assert(sample.firstDm==29 && sample.firstDb==67 && sample.lastDm==29 && sample.lastDb==67);
    // The stale all-fields (mask 0xFF) frame a previously streaming unit sent once
    // after reconfiguration: a valid frame, but the wrong size, so it is ignored.
    const std::vector<uint8_t> stale44 = {0xaa,0x40,0x08,0x2c,0x1d,0x00,0x00,0x00,0x1d,0x00,0x00,0x00,
        0x4a,0x00,0x00,0x00,0x1d,0x00,0x00,0x00,0x1d,0x00,0x00,0x00,0x4a,0x00,0x00,0x00,
        0x2d,0x0f,0x00,0x00,0x01,0x01,0x00,0x00,0x7b,0xda};
    const uint32_t before = driver.parseErrors();
    assert(!receive(driver,stale44,450,sample) && driver.parseErrors()==before && driver.ready());
    // Real write acknowledgement: ID27 echoes the 4-byte mask with the write bit clear.
    const std::vector<uint8_t> realAck27 = {0xaa,0x40,0x01,0x1b,0x2d,0x00,0x00,0x00,0xb4,0x69};
    grf250::Parser realParser;
    count=0; for (const auto b: realAck27) count += realParser.feed(b,1,parsed);
    assert(count==1 && parsed.size==5 && parsed.payload[0]==27 && grf250::get32(parsed.payload+1)==0x2du);
    // First/last mode ignores ID45 frames without counting a parse error.
    assert(!receive(driver,packet(45,std::vector<uint8_t>(44)),500,sample) && driver.parseErrors()==before);
    assert(driver.nextCommand(600301,output,sizeof(output))==0 && !driver.ready());
    assert(driver.nextCommand(600302,output,sizeof(output))==0);
    configure(driver,2600302);
    // Five-return mode: configures stream 6 and parses ID45 (mm -> dm, raw slots).
    assert(grf250::mmToDm(7149)==71 && grf250::mmToDm(7150)==72 && grf250::mmToDm(-10)==0
           && grf250::mmToDm(-150)==-2 && grf250::mmToDm(INT32_MIN)==-21474836);
    grf250::Driver five(true);
    assert(five.fiveReturns());
    configure(five,0,6);
    std::vector<uint8_t> returns;
    for (const int32_t value: {7100,49,20400,63,0,0,0,0,0,0,3008}) {
        auto bytes=u32(static_cast<uint32_t>(value)); returns.insert(returns.end(),bytes.begin(),bytes.end());
    }
    assert(receive(five,packet(45,returns),100,sample));
    assert(sample.slots==5 && sample.returnDm[0]==71 && sample.returnDb[0]==49
           && sample.returnDm[1]==204 && sample.returnDb[1]==63 && sample.returnDm[2]==0
           && sample.returnDb[4]==0 && sample.firstDm==-10 && sample.sequence==1);
    // Real ID45 frames (fw 2.0.5, 2026-11-10): a 0.9 m blanked return at 73 dB
    // (blanking 1 m), and an empty reading. Temperature (4151 = 41.51 C) ignored.
    const std::vector<uint8_t> real45 = {0xaa,0x40,0x0b,0x2d,0x84,0x03,0,0,0x49,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x37,0x10,0,0,0x0e,0x48};
    assert(receive(five,real45,200,sample));
    assert(sample.returnDm[0]==9 && sample.returnDb[0]==73 && sample.returnDm[1]==0 && sample.sequence==2);
    const std::vector<uint8_t> empty45 = {0xaa,0x40,0x0b,0x2d,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xc0,0x0b,0,0,0xa9,0xa2};
    assert(receive(five,empty45,300,sample));
    for (size_t k=0;k<grf250::kReturnSlots;++k) assert(sample.returnDm[k]==0 && sample.returnDb[k]==0);
    // Real three-return frame through the café window: 4.1 m (glass) + 31.4 m +
    // 120.6 m. The third return fills slot 3; the "last" slot stays empty.
    const std::vector<uint8_t> three45 = {0xaa,0x40,0x0b,0x2d,0x04,0x10,0,0,0x36,0,0,0,
        0xa8,0x7a,0,0,0x36,0,0,0,0x18,0xd7,0x01,0,0x16,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0x7b,0x10,0,0,0xac,0xa1};
    assert(receive(five,three45,350,sample));
    assert(sample.returnDm[0]==41 && sample.returnDm[1]==314 && sample.returnDm[2]==1206
           && sample.returnDb[2]==22 && sample.returnDm[4]==0 && sample.returnDb[4]==0);
    assert(!receive(five,packet(44,fields),400,sample) && five.parseErrors()==0); // ID44 ignored
    // Five bounded retries incl. UART interface-selection first command loss.
    grf250::Driver absent;
    for (uint32_t i=0;i<5;++i) assert(absent.nextCommand(i*250000,output,10)==6);
    assert(absent.nextCommand(1250000,output,10)==0 && absent.faults==1);
    assert(absent.nextCommand(3250000,output,10)==6);
    // Wrong identity and configuration mismatch never become ready.
    grf250::Driver wrong;
    assert(wrong.nextCommand(0,output,10)==6);
    assert(!receive(wrong,packet(0,std::vector<uint8_t>(16)),1,sample));
    assert(wrong.faults==1 && !wrong.ready());
    // A mismatched parameter acknowledgement fails configuration immediately.
    grf250::Driver mismatch;
    assert(mismatch.nextCommand(0,output,10)==6);
    std::vector<uint8_t> identity(16); memcpy(identity.data(),"GRF250",6);
    assert(!receive(mismatch,packet(0,identity),1,sample));
    assert(mismatch.nextCommand(2,output,10)==6);
    assert(!receive(mismatch,packet(2,u32(123)),3,sample));
    assert(mismatch.nextCommand(4,output,10)==10 && output[3]==30);
    assert(!receive(mismatch,packet(30,u32(5)),5,sample));
    assert(mismatch.faults==1 && !mismatch.ready());
    std::puts("GRF250 protocol / FSM tests passed (including 100000-byte noise stream)");
}
