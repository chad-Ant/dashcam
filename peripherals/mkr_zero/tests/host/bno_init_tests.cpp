// The real BNO055Init.cpp runs against a two-page register model. Only the
// transport, identity/calibration helpers and bus health are faked.
#include "BNO055Init.h"
#include "BNO055Calib.h"
#include <stdio.h>
#include <string.h>
#include <initializer_list>

static unsigned checks = 0, failures = 0;
#define CHECK(x) do { ++checks; if (!(x)) { ++failures; printf("FAIL %d: %s\n", __LINE__, #x); } } while (0)
static uint8_t regs[2][128];
static uint8_t page, selfTest, reservedBits, stuckReg, stuckBits;
static bool failSelfTestRead;
static unsigned selfTestReads;
TwoWire Wire;

I2CBusState i2cBusBegin() { return I2CBusState::Ready; }
uint8_t i2cStuckLines() { return 0; }
uint8_t bno055FindAddress() { return BNO055_I2C_ADDRESS_DEFAULT; }
uint16_t bno055TransportFaults() { return 0; }
uint32_t bno055TransportErrorCount() { return 0; }
void bno055TransportResetFaults() {}
bool bno055Identify(BNO055Device &d, uint8_t addr) {
    d.address = addr; d.chipId = BNO055_EXPECTED_CHIP_ID; d.pageId = page;
    return true;
}
bool bno055CalibWrite(const BNO055InitState &, const uint8_t *) { return true; }

static void resetRegisters() {
    memset(regs, 0, sizeof(regs));
    page = 0;
    regs[0][BNO055_CHIP_ID_ADDR] = BNO055_EXPECTED_CHIP_ID;
    // Deliberately dirty interrupt enables: setup must replace, not OR them.
    regs[1][BNO055_P1_INT_EN_ADDR] = 0xCCu;
    regs[1][BNO055_P1_INT_MSK_ADDR] = 0xCCu;
}

extern "C" int bno055BusRead(unsigned char, unsigned char reg, unsigned char *buf, unsigned char n) {
    if (n != 1u || reg >= 128u) return -1;
    if (reg == BNO055_PAGE_ID_ADDR) { *buf = page; return 0; }
    if (page == 0 && reg == BNO055_ST_RESULT_ADDR) {
        ++selfTestReads;
        if (failSelfTestRead) return -1;
        *buf = selfTest; return 0;
    }
    *buf = regs[page][reg];
    if (page == 1 && (reg == BNO055_P1_INT_EN_ADDR || reg == BNO055_P1_INT_MSK_ADDR)) {
        *buf |= reservedBits;
        if (reg == stuckReg) *buf |= stuckBits;
    }
    return 0;
}

extern "C" int bno055BusWrite(unsigned char, unsigned char reg, unsigned char *buf, unsigned char n) {
    if (n != 1u || reg >= 128u) return -1;
    if (reg == BNO055_PAGE_ID_ADDR) { if (*buf > 1u) return -1; page = *buf; return 0; }
    if (page == 0 && reg == BNO055_SYS_TRIGGER_ADDR && (*buf & 0x20u)) {
        resetRegisters(); return 0;
    }
    regs[page][reg] = *buf;
    if (page == 0 && reg == BNO055_OPR_MODE_ADDR) {
        regs[0][BNO055_SYS_STATUS_ADDR] = *buf == OPERATION_MODE_IMUPLUS ? 5u : 6u;
    }
    return 0;
}

static BNO055InitState run(uint8_t st, uint8_t mode, bool nack = false,
                          uint8_t badReg = 0, uint8_t badBits = 0) {
    hostReset(); hostSetMillis(1000u);
    resetRegisters();
    selfTest = st; reservedBits = 0x13u; stuckReg = badReg; stuckBits = badBits;
    failSelfTestRead = nack; selfTestReads = 0;
    BNO055InitState s;
    CHECK(bno055InitBegin(s, mode));
    for (unsigned i = 0; i < 100u; ++i) {
        hostSetMillis(s.nextStepMs + 1000u);
        bno055InitTick(s);
        if (s.stage == BNO055InitStage::Configured || s.stage == BNO055InitStage::Failed) break;
    }
    CHECK(page == 0u);
    return s;
}

int main() {
    for (uint8_t mode : {uint8_t(OPERATION_MODE_IMUPLUS), uint8_t(OPERATION_MODE_AMG)}) {
        for (unsigned upper = 0; upper < 16u; ++upper) {
            for (uint8_t lower : {uint8_t(0x0Fu), uint8_t(0x0Du)}) {
                const uint8_t st = static_cast<uint8_t>((upper << 4) | lower);
                const BNO055InitState s = run(st, mode);
                CHECK(s.stage == BNO055InitStage::Configured && s.highGArmed);
                CHECK(s.selfTestReadOk && s.selfTestResult == st && selfTestReads > 0u);
                CHECK(regs[1][BNO055_P1_INT_EN_ADDR] == 0x20u);
                CHECK(regs[1][BNO055_P1_INT_MSK_ADDR] == 0x20u);
            }
        }
        for (uint8_t st : {uint8_t(0x0Eu), uint8_t(0x0Bu), uint8_t(0x07u), uint8_t(0x00u)}) {
            const BNO055InitState s = run(st, mode);
            CHECK(s.stage == BNO055InitStage::Failed);
            CHECK(s.lastStatus == BNO055InitStatus::NOK_SELF_TEST_FAILED);
            CHECK(s.firstFailure.valid && s.firstFailure.selfTestReadOk);
            CHECK(s.firstFailure.selfTestResult == st);
            CHECK(s.firstFailure.failedAt == BNO055InitStage::VerifyOpMode);
        }
    }
    const BNO055InitState nack = run(0x0F, OPERATION_MODE_IMUPLUS, true);
    CHECK(nack.stage == BNO055InitStage::Failed && !nack.firstFailure.selfTestReadOk);
    CHECK(nack.lastStatus == BNO055InitStatus::NOK_SELF_TEST_FAILED);
    CHECK(strcmp(bno055InitStatusName(nack.lastStatus), "self-test-failed") == 0);
    for (uint8_t reg : {uint8_t(BNO055_P1_INT_EN_ADDR), uint8_t(BNO055_P1_INT_MSK_ADDR)}) {
        const BNO055InitState s = run(0x0F, OPERATION_MODE_IMUPLUS, false, reg, 0x04u);
        CHECK(s.stage == BNO055InitStage::Configured && !s.highGArmed);
    }
    printf("bno_init_tests: %u checks, %u failed\n", checks, failures);
    return failures ? 1 : 0;
}
