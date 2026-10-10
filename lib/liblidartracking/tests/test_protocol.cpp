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
static void configure(grf250::Driver& driver, uint32_t now) {
    uint8_t command[10]; grf250::Sample sample;
    for (unsigned i=0; i<12; ++i) {
        const size_t n = driver.nextCommand(now,command,sizeof(command));
        assert(n == (i >= 2 && i%2 == 0 ? 10u : 6u));
        std::vector<uint8_t> data;
        if (i == 0) { data.resize(16); memcpy(data.data(),"GRF250",6); }
        else if (i == 1) data = u32(0x00050200);
        else {
            const uint32_t expected = i < 4 ? 0u : i < 6 ? 0x2du : i < 8 ? 1u : i < 10 ? 20u : 5u;
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
    for (const auto value: {10000u,25u,10000u,25u}) { auto bytes=u32(value); fields.insert(fields.end(),bytes.begin(),bytes.end()); }
    assert(receive(driver,packet(44,fields),100,sample));
    assert(sample.firstCm==10000 && sample.firstDb==25 && sample.sequence==1);
    assert(!receive(driver,packet(44,std::vector<uint8_t>(12)),200,sample));
    grf250::put32(fields.data(),0xfffffff6u);
    assert(receive(driver,packet(44,fields),300,sample));
    assert(sample.firstCm==-10 && sample.sequence==2);
    assert(driver.nextCommand(600301,output,sizeof(output))==0 && !driver.ready());
    assert(driver.nextCommand(600302,output,sizeof(output))==0);
    configure(driver,2600302);
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
