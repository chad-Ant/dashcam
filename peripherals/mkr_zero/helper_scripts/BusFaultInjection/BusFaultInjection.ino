/**
 * BusFaultInjection — wedges the I2C bus on purpose, then proves recovery.
 *
 * The recovery path in lib/I2CBus.cpp had never actually run against a wedged
 * bus. Everything about it was argued from the datasheet, which is exactly the
 * kind of code that turns out to be wrong the one time it matters.
 *
 * HOW THE WEDGE IS MADE
 * ---------------------
 * Not by shorting SDA to ground — that tests a permanent hardware fault, which
 * no amount of clocking can fix and which the code is supposed to REPORT rather
 * than repair. The interesting fault is the recoverable one: a controller reset
 * that lands mid-transaction, leaving a slave part-way through shifting out a
 * byte, still driving SDA and waiting for clocks that never come.
 *
 * So this sketch bit-bangs a real read transaction against the BNO055 —
 * START, address+R, ACK — and then simply stops clocking. The register pointer
 * is parked on GYR_ID first, whose value 0x0F has bit 7 = 0, so the very first
 * data bit the slave presents is a zero: it holds SDA low and the bus is
 * genuinely stuck, by the slave, exactly as after an unclean reset. (CHIP_ID,
 * parked on before, reads 0xA0: its first bit is a 1, so the BNO055 let go of
 * SDA and the wedge never took.)
 *
 * TRANSFERS ON A WEDGED BUS (vendor/Wire, DASHCAM_WIRE_BOUNDED)
 * ------------------------------------------------------------
 * T1b makes a real Wire transfer on the SDA-wedged bus. T5 holds SCL low
 * under the controller while it reads — the wedge the core's unbounded SERCOM
 * waits could not survive (a slave stretching forever never lets MB or SB set).
 * Both must fail within the deadline and leave the bus recoverable, with no
 * watchdog reset. The 'c' command repeats T5 through the core's own waits in an
 * instrumented build, to show the hang the bounds remove: it ENDS IN A WATCHDOG
 * RESET by design.
 *
 * GLITCHES (T7-T9)
 * ----------------
 * A flickering contact is a different fault from a wedge: a START or STOP where
 * none belongs. T7 stages a START with no STOP on an idle bus; the controller
 * then believes another master holds the bus while both lines read idle. C7
 * (instrumented builds) shows the stock driver refusing every transfer from
 * there on; T7/T8 show the bounded one failing one transfer as a bus error and
 * recovering. T9 pulls SDA low under a high SCL part-way through a read: the
 * read must come back as 0 bytes and a bus error, never as bytes.
 *
 * Commands:  a = full automatic test    w = wedge only
 *            r = recover only           c = control (stock waits: hangs, resets)
 *            h = help
 */

#include <Wire.h>

#include "DataDictionary.h"
#include "TimerFunctions.h"
#include "I2CBus.h"
#include "IMUFunctions.h"
#include "GPSFunctions.h"

SFE_UBLOX_GNSS myGNSS;

static constexpr unsigned long SERIAL_READY_TIMEOUT_MS = 2000;

/// The IMU, used here only as something to hold the bus down with.
/// Names kept from the LSM6DSOX this replaced so the bit-banged wedge below
/// reads unchanged; only the part behind them is different.
static constexpr uint8_t SOX_ADDR   = BNO055_I2C_ADDRESS_DEFAULT;  ///< 0x29.
static constexpr uint8_t SOX_WHOAMI = 0x03u;   ///< GYR_ID (page 0): first data bit 0.
static constexpr uint8_t SOX_ID     = 0x0Fu;   ///< Its fixed value.
static constexpr uint8_t BNO_PAGE_ID  = 0x07u; ///< Register map page select.
static constexpr uint8_t BNO_CHIP_ID  = 0x00u; ///< Page 0, reads 0xA0.

/// Longest a transfer on a wedged bus may take: the deadline plus slack for the
/// SERCOM reset in the abort path.
static constexpr uint32_t WEDGED_XFER_MAX_US = 2u * DASHCAM_WIRE_WAIT_US;

/// Half-bit period for the bit-banged transaction (~100 kHz).
static constexpr uint32_t HALF_BIT_US = 5u;
/// Bounded wait for SCL to rise, so a stretching slave cannot hang the bang.
static constexpr uint32_t SCL_RISE_TIMEOUT_US = 200u;

static uint16_t passCount = 0;
static uint16_t failCount = 0;

// ── open-drain bit-banging ───────────────────────────────────────────────────
//
// Same latch-preload order as lib/I2CBus.cpp, for the same reason: pinMode()
// with INPUT_PULLUP leaves the SAMD21 output latch HIGH, so switching straight
// to OUTPUT would drive the line push-pull high for an instant — into a slave
// that may be holding it low.

static void driveLow(uint8_t pin)
{
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
}

static inline void releaseLine(uint8_t pin)
{
    pinMode(pin, INPUT_PULLUP);
}

static bool releaseSclAndWait(void)
{
    releaseLine(PIN_WIRE_SCL);
    const uint32_t start = micros();
    while ((micros() - start) < SCL_RISE_TIMEOUT_US) {
        if (digitalRead(PIN_WIRE_SCL) == HIGH) return true;
    }
    return false;
}

static void report(const char *name, bool ok, const char *detail)
{
    if (ok) passCount++; else failCount++;
    Serial.print(ok ? "PASS  " : "FAIL  ");
    Serial.print(name);
    if (detail != nullptr && detail[0] != '\0') {
        Serial.print("  -- ");
        Serial.print(detail);
    }
    Serial.println();
}

/**
 * @brief Leaves the LSM6DSOX mid-byte, holding SDA low.
 * @return @c true if SDA is actually low afterwards (the wedge took).
 */
static bool wedgeBus(void)
{
    // GYR_ID lives on page 0. A production bring-up leaves page 0 selected,
    // but the BNO055 keeps its state across a reflash, so select it anyway.
    Wire.beginTransmission(SOX_ADDR);
    if (Wire.write(BNO_PAGE_ID) != 1 || Wire.write((uint8_t)0x00u) != 1) {
        (void)Wire.endTransmission(true); return false;
    }
    if (Wire.endTransmission(true) != 0) return false;

    // Park the register pointer on GYR_ID using the normal driver, so the
    // aborted read below returns 0x0F and its first bit is a guaranteed zero.
    Wire.beginTransmission(SOX_ADDR);
    if (Wire.write(SOX_WHOAMI) != 1) { (void)Wire.endTransmission(true); return false; }
    if (Wire.endTransmission(true) != 0) return false;

    // Take the pins off SERCOM2 and drive them by hand.
    releaseLine(PIN_WIRE_SCL);
    releaseLine(PIN_WIRE_SDA);
    delayMicroseconds(HALF_BIT_US * 2u);

    // START: SDA falls while SCL is high.
    driveLow(PIN_WIRE_SDA);
    delayMicroseconds(HALF_BIT_US);
    driveLow(PIN_WIRE_SCL);
    delayMicroseconds(HALF_BIT_US);

    // Address byte, MSB first: 7-bit address then the READ bit.
    const uint8_t frame = static_cast<uint8_t>((SOX_ADDR << 1) | 0x01u);
    for (int8_t bit = 7; bit >= 0; bit--) {
        if (((frame >> bit) & 0x01u) != 0u) releaseLine(PIN_WIRE_SDA);
        else                                driveLow(PIN_WIRE_SDA);
        delayMicroseconds(HALF_BIT_US);
        if (!releaseSclAndWait()) return false;
        delayMicroseconds(HALF_BIT_US);
        driveLow(PIN_WIRE_SCL);
        delayMicroseconds(HALF_BIT_US);
    }

    // ACK clock: release SDA so the slave can pull it down.
    releaseLine(PIN_WIRE_SDA);
    delayMicroseconds(HALF_BIT_US);
    if (!releaseSclAndWait()) return false;
    delayMicroseconds(HALF_BIT_US / 2u);
    const bool acked = (digitalRead(PIN_WIRE_SDA) == LOW);
    delayMicroseconds(HALF_BIT_US / 2u);
    driveLow(PIN_WIRE_SCL);
    delayMicroseconds(HALF_BIT_US);

    if (!acked) {
        // No slave answered; unwind rather than leaving a half-transaction.
        releaseLine(PIN_WIRE_SCL);
        releaseLine(PIN_WIRE_SDA);
        return false;
    }

    // ABORT HERE. With SCL low the slave has already presented data bit 7 of
    // 0x0F, which is 0 — so it is driving SDA low and waiting for a clock edge
    // that this sketch will never send. Release both lines and walk away; SCL
    // floats high, SDA stays down, held by the slave.
    if (!releaseSclAndWait()) return false;
    delayMicroseconds(HALF_BIT_US * 4u);

    return (digitalRead(PIN_WIRE_SDA) == LOW);
}

/** @brief Reads SDA/SCL as GPIO without disturbing whoever holds them. */
static void sampleLines(bool &sdaHigh, bool &sclHigh)
{
    pinMode(PIN_WIRE_SDA, INPUT_PULLUP);
    pinMode(PIN_WIRE_SCL, INPUT_PULLUP);
    delayMicroseconds(10);
    sdaHigh = (digitalRead(PIN_WIRE_SDA) == HIGH);
    sclHigh = (digitalRead(PIN_WIRE_SCL) == HIGH);
}

// ── SCL held low: the wedge the core's waits could not survive ───────────────
//
// A slave that stretches SCL forever never lets the controller set MB or SB, so
// SERCOM::readDataWIRE() / startTransmissionWIRE() spin until the watchdog. No
// BNO055 will do that on demand, so the controller is made to see it: the SCL
// pad is taken off SERCOM2 and driven low by the port while the peripheral stays
// enabled and mid-request. The controller can no longer clock, which is the
// state a stretching slave leaves it in.

static void holdSclLow(void)
{
    const EPortType port = g_APinDescription[PIN_WIRE_SCL].ulPort;
    const uint32_t  pin  = g_APinDescription[PIN_WIRE_SCL].ulPin;
    PORT->Group[port].OUTCLR.reg = (1ul << pin);               // latch low first
    PORT->Group[port].DIRSET.reg = (1ul << pin);
    PORT->Group[port].PINCFG[pin].reg &= (uint8_t)~PORT_PINCFG_PMUXEN;  // port owns the pad now
}

static void releaseScl(void)
{
    const EPortType port = g_APinDescription[PIN_WIRE_SCL].ulPort;
    const uint32_t  pin  = g_APinDescription[PIN_WIRE_SCL].ulPin;
    PORT->Group[port].DIRCLR.reg = (1ul << pin);               // back to the pull-ups
}

/// Reads one register through the normal path; true when it returned @p want.
/// (Defined before T5b, which uses it too.)
static bool readRegIs(uint8_t reg, uint8_t want)
{
    Wire.beginTransmission(SOX_ADDR);
    if (Wire.write(reg) != 1) { (void)Wire.endTransmission(true); return false; }
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(SOX_ADDR, (size_t)1, true) != 1) return false;
    return Wire.read() == want;
}

/**
 * @brief T5: a read with SCL held low must fail within the deadline, count one
 *        abandoned transfer, and leave the bus recoverable and working.
 * @param control Route through the core's unbounded waits instead (instrumented
 *        builds only). Expected to hang until the watchdog resets the board.
 */
static void sclStallTest(bool control)
{
    if (i2cBusBegin() != I2CBusState::Ready) {
        report("T5 SCL held low", false, "bus not Ready before the test");
        return;
    }
#ifdef DASHCAM_WIRE_INSTRUMENT
    dashcamWireCoreWaits = control;
#else
    if (control) {
        Serial.println("control needs an instrumented build (build_and_upload.sh adds DASHCAM_WIRE_INSTRUMENT)");
        return;
    }
#endif
    // Point the BNO055 at CHIP_ID, then read with SCL held low. The pause lets
    // the pointer write's STOP reach the wire: endTransmission() returns once
    // the command is synchronised, not once the STOP is clocked out, and
    // pulling SCL low under it would leave the bus in no defined state.
    Wire.beginTransmission(SOX_ADDR);
    (void)Wire.write(BNO_CHIP_ID);
    (void)Wire.endTransmission(true);
    delayMicroseconds(200);

    const uint32_t toBefore = i2cWireTimeouts();
    holdSclLow();
    const uint32_t t0 = micros();
    const size_t got = Wire.requestFrom(SOX_ADDR, (size_t)6, true);
    const uint32_t tookUs = micros() - t0;
    releaseScl();
#ifdef DASHCAM_WIRE_INSTRUMENT
    dashcamWireCoreWaits = false;
#endif
    const uint32_t aborted = i2cWireTimeouts() - toBefore;
    {
        char detail[96];
        snprintf(detail, sizeof(detail), "got %u bytes in %lu us, timeouts +%lu",
                 (unsigned)got, (unsigned long)tookUs, (unsigned long)aborted);
        report(control ? "C1 stock waits returned (expected a hang)"
                       : "T5 SCL held low before a read's START: fails within the deadline",
               !control && (got == 0u) && (aborted == 1u) &&
               (tookUs >= DASHCAM_WIRE_WAIT_US) && (tookUs < WEDGED_XFER_MAX_US),
               detail);
    }
    if (control) return;

    // The manager sees the new abandon and runs the full recovery on its own.
    const I2CBusState st = i2cBusBegin();
    const bool chip = readRegIs(BNO_CHIP_ID, 0xA0u);
    const bool gnss = i2cProbeAddress(GPS_DEFAULT_I2C_ADDRESS);
    char detail[80];
    snprintf(detail, sizeof(detail), "state=%d, CHIP_ID %s, gnss=%d",
             (int)st, chip ? "0xA0" : "WRONG", (int)gnss);
    report("T6 bus recovers by itself and both devices answer",
           (st == I2CBusState::Ready) && chip && gnss, detail);
}

// ── T5b: SCL pulled low PART-WAY THROUGH a long read ─────────────────────────
//
// T5 stalls the address phase. This one lets the read start and pulls SCL low
// while bytes are flowing, from a TC5 one-shot interrupt, so the per-byte waits
// (readBounded) expire with the controller the bus OWNER and the BNO055 part-way
// through a byte — the case i2cBusBegin()'s forced recovery exists for: a slave
// left mid-byte that must be clocked out before the bus is usable again.

static volatile bool tc5Fired = false;

/// What the TC5 one-shot does when it fires: T5b's SCL stall, or T9's glitch.
enum class Tc5Action : uint8_t { SclStall, SdaGlitch };
static volatile Tc5Action tc5Action = Tc5Action::SclStall;
/// T9: the glitch went in while SCL was high (a START/STOP, not a data bit).
static volatile bool glitchLanded = false;
/// T9: one-shots spent looking for a byte with a 1 bit left in it.
static volatile uint8_t glitchTries = 0;
static constexpr uint8_t  GLITCH_MAX_TRIES   = 24u;
static constexpr uint32_t GLITCH_RETRY_US    = 60u;

static bool sdaGlitchUnderHighScl(void);   // T9, defined below

/// Fires the (stopped) TC5 one-shot again in @p us.
static void tc5Rearm(uint32_t us)
{
    TC5->COUNT16.COUNT.reg = 0u;
    while (TC5->COUNT16.STATUS.bit.SYNCBUSY) { }
    TC5->COUNT16.CC[0].reg = (uint16_t)(us * 3u);
    while (TC5->COUNT16.STATUS.bit.SYNCBUSY) { }
    TC5->COUNT16.CTRLA.bit.ENABLE = 1;
    while (TC5->COUNT16.STATUS.bit.SYNCBUSY) { }
}

void TC5_Handler(void)
{
    TC5->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
    TC5->COUNT16.CTRLA.bit.ENABLE = 0;      // one shot
    if (tc5Action == Tc5Action::SdaGlitch) {
        glitchLanded = sdaGlitchUnderHighScl();
        // Nothing to glitch in this byte (all zeros, or the controller is
        // stretching SCL waiting for this very CPU): try a later byte.
        if (!glitchLanded && ++glitchTries < GLITCH_MAX_TRIES) tc5Rearm(GLITCH_RETRY_US);
    } else {
        holdSclLow();
    }
    tc5Fired = true;
}

/** @brief Arms TC5 to call holdSclLow() in @p us microseconds (48 MHz / 16). */
static void armSclStall(uint32_t us)
{
    PM->APBCMASK.reg |= PM_APBCMASK_TC5;
    GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 |
                                   GCLK_CLKCTRL_ID(GCM_TC4_TC5));
    while (GCLK->STATUS.bit.SYNCBUSY) { }
    TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
    while (TC5->COUNT16.CTRLA.bit.SWRST || TC5->COUNT16.STATUS.bit.SYNCBUSY) { }
    TC5->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MFRQ | TC_CTRLA_PRESCALER_DIV16;
    while (TC5->COUNT16.STATUS.bit.SYNCBUSY) { }
    TC5->COUNT16.CC[0].reg = (uint16_t)(us * 3u);   // 3 counts per us
    while (TC5->COUNT16.STATUS.bit.SYNCBUSY) { }
    TC5->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
    TC5->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
    NVIC_ClearPendingIRQ(TC5_IRQn);
    NVIC_SetPriority(TC5_IRQn, 0);
    NVIC_EnableIRQ(TC5_IRQn);
    tc5Fired = false;
    glitchLanded = false;
    glitchTries = 0u;
    TC5->COUNT16.CTRLA.bit.ENABLE = 1;
    while (TC5->COUNT16.STATUS.bit.SYNCBUSY) { }
}

static void midReadStallTest(void)
{
    if (i2cBusBegin() != I2CBusState::Ready) {
        report("T5b SCL low mid-read", false, "bus not Ready before the test");
        return;
    }
    // 64 bytes from GYR_ID take ~1.5 ms at 400 kHz; the stall lands ~400 us in,
    // about a quarter of the way.
    Wire.beginTransmission(SOX_ADDR);
    (void)Wire.write(SOX_WHOAMI);
    (void)Wire.endTransmission(true);
    delayMicroseconds(200);

    const uint32_t toBefore = i2cWireTimeouts();
    tc5Action = Tc5Action::SclStall;
    armSclStall(400u);
    const uint32_t t0 = micros();
    const size_t got = Wire.requestFrom(SOX_ADDR, (size_t)64, true);
    const uint32_t tookUs = micros() - t0;
    NVIC_DisableIRQ(TC5_IRQn);
    TC5->COUNT16.CTRLA.bit.ENABLE = 0;
    const bool fired = tc5Fired;
    releaseScl();
    const uint32_t aborted = i2cWireTimeouts() - toBefore;
    {
        char detail[96];
        snprintf(detail, sizeof(detail), "stall %s, got %u bytes in %lu us, timeouts +%lu",
                 fired ? "injected" : "MISSED (read finished first)",
                 (unsigned)got, (unsigned long)tookUs, (unsigned long)aborted);
        report("T5b SCL held low mid-read: fails within the deadline",
               fired && (got == 0u) && (aborted == 1u) &&
               (tookUs >= DASHCAM_WIRE_WAIT_US) && (tookUs < WEDGED_XFER_MAX_US),
               detail);
    }

    // The forced recovery must clock the mid-byte BNO055 free on its own.
    const I2CBusState st = i2cBusBegin();
    const bool chip = readRegIs(BNO_CHIP_ID, 0xA0u);
    const bool gnss = i2cProbeAddress(GPS_DEFAULT_I2C_ADDRESS);
    char detail[96];
    snprintf(detail, sizeof(detail), "state=%d lines=\"%s\", CHIP_ID %s, gnss=%d",
             (int)st, i2cStuckReason(), chip ? "0xA0" : "WRONG", (int)gnss);
    report("T6b bus recovers from a slave left mid-byte",
           (st == I2CBusState::Ready) && chip && gnss, detail);
}

// ── T7/T8: a glitch the controller takes for a START ─────────────────────────
//
// The breadboard contacts behind 2026-09-26's corrupt reads do not wedge the
// bus, they flicker it. SDA falling while SCL is high is a START to everything
// on the bus, the controller included, and on a single-master bus nothing ever
// sends the STOP that ends it: the controller's bus state stays BUSY - "another
// master is talking" - while both lines read idle. The stock driver refuses
// every transfer from then on (its early check), and i2cBusBegin()'s line check
// sees a healthy bus. vendor/Wire now fails that transfer as a bus error and
// resets the controller, and i2cBusBegin() recovers the bus on the new count.
//
// Staged from the port on the real bus: SDA low under a high SCL (the START),
// SCL low, SDA back up under the low SCL (data, not a STOP), SCL back up (a
// clock with SDA high). The BNO055 and the GNSS see a START and one address bit,
// which the next real START discards. The pads go straight back to SERCOM2 by
// PMUXEN, NOT Wire.begin(), which would reset the very state under test.

static inline PortGroup &padGroup(uint8_t pinNo) { return PORT->Group[g_APinDescription[pinNo].ulPort]; }
static inline uint32_t   padMask(uint8_t pinNo)  { return 1ul << g_APinDescription[pinNo].ulPin; }
static inline uint32_t   padIndex(uint8_t pinNo) { return g_APinDescription[pinNo].ulPin; }

/// Input buffer on for both bus pads, so padHigh() reads them while muxed.
static void senseBusPads(void)
{
    padGroup(PIN_WIRE_SDA).PINCFG[padIndex(PIN_WIRE_SDA)].reg |= (uint8_t)PORT_PINCFG_INEN;
    padGroup(PIN_WIRE_SCL).PINCFG[padIndex(PIN_WIRE_SCL)].reg |= (uint8_t)PORT_PINCFG_INEN;
}

static inline bool padHigh(uint8_t pinNo)
{
    return (padGroup(pinNo).IN.reg & padMask(pinNo)) != 0u;
}

/// Takes a bus pad off SERCOM2 and pulls it low from the port (latch low first).
static inline void padDriveLow(uint8_t pinNo)
{
    PortGroup &g = padGroup(pinNo);
    g.OUTCLR.reg = padMask(pinNo);
    g.DIRSET.reg = padMask(pinNo);
    g.PINCFG[padIndex(pinNo)].reg &= (uint8_t)~PORT_PINCFG_PMUXEN;
}

/// Lets the pad go (input first, then the pull back to UP) and hands it back to
/// SERCOM2 with its PMUX setting untouched.
static inline void padRelease(uint8_t pinNo)
{
    PortGroup &g = padGroup(pinNo);
    g.DIRCLR.reg = padMask(pinNo);
    g.OUTSET.reg = padMask(pinNo);
    g.PINCFG[padIndex(pinNo)].reg |= (uint8_t)PORT_PINCFG_PMUXEN;
}

/** @brief The controller's bus state, as the core's own accessors report it. */
static const char *controllerBusState(void)
{
    if (PERIPH_WIRE.isBusOwnerWIRE()) return "owner";
    if (PERIPH_WIRE.isBusBusyWIRE())  return "busy";
    if (PERIPH_WIRE.isBusIdleWIRE())  return "idle";
    return "unknown";
}

/** @brief A START with no STOP on the idle bus. @return false if it was not idle. */
static bool stagePhantomStart(void)
{
    if (!padHigh(PIN_WIRE_SDA) || !padHigh(PIN_WIRE_SCL)) return false;
    padDriveLow(PIN_WIRE_SDA);          // SDA falls, SCL high: START
    delayMicroseconds(HALF_BIT_US);
    padDriveLow(PIN_WIRE_SCL);
    delayMicroseconds(HALF_BIT_US);
    padRelease(PIN_WIRE_SDA);           // SDA rises under a LOW SCL: no STOP
    delayMicroseconds(HALF_BIT_US);
    padRelease(PIN_WIRE_SCL);           // one clock, SDA high
    delayMicroseconds(HALF_BIT_US * 2u);
    return true;
}

static void phantomStartTest(void)
{
    if (i2cBusBegin() != I2CBusState::Ready) {
        report("T7 glitch START", false, "bus not Ready before the test");
        return;
    }
    senseBusPads();
    delayMicroseconds(200);   // the last transfer's STOP is on the wire

    if (!stagePhantomStart()) {
        report("T7 glitch START", false, "lines not idle, nothing staged");
        return;
    }
    const char *state = controllerBusState();
    if (!PERIPH_WIRE.isBusBusyWIRE()) {
        // The premise did not reproduce: say so rather than pass or fail on it.
        Serial.print("SKIP  T7/T8 glitch START  -- controller bus state after it: ");
        Serial.print(state);
        Serial.println(" (expected busy)");
        (void)i2cBusRecover();
        return;
    }

#ifdef DASHCAM_WIRE_INSTRUMENT
    // Control: the stock driver's early refusal, as every build before this one
    // ran it. The manager calls the bus Ready (lines idle) and every read fails.
    {
        dashcamWireCoreWaits = true;
        uint8_t ready = 0u, failed = 0u;
        for (uint8_t i = 0u; i < 3u; i++) {
            if (i2cBusBegin() != I2CBusState::Ready) continue;
            ready++;
            if (!readRegIs(BNO_CHIP_ID, 0xA0u)) failed++;
        }
        dashcamWireCoreWaits = false;
        char detail[96];
        snprintf(detail, sizeof(detail), "bus Ready %u/3, CHIP_ID read failed %u/3, state %s",
                 (unsigned)ready, (unsigned)failed, controllerBusState());
        report("C7 stock driver: refused every transfer on an idle-looking bus",
               (ready == 3u) && (failed == 3u), detail);
    }
#endif

    const uint32_t errBefore = i2cWireBusErrors();
    const uint32_t toBefore  = i2cWireTimeouts();
    const uint32_t t0 = micros();
    const bool read = readRegIs(BNO_CHIP_ID, 0xA0u);
    const uint32_t tookUs = micros() - t0;
    const uint32_t errs = i2cWireBusErrors() - errBefore;
    const uint32_t tos  = i2cWireTimeouts() - toBefore;
    {
        char detail[112];
        snprintf(detail, sizeof(detail), "state %s, read %s in %lu us, bus errors +%lu, timeouts +%lu",
                 state, read ? "SUCCEEDED" : "failed", (unsigned long)tookUs,
                 (unsigned long)errs, (unsigned long)tos);
        report("T7 glitch START: the next transfer fails fast as one bus error",
               !read && (errs == 1u) && (tos == 0u) && (tookUs < WEDGED_XFER_MAX_US), detail);
    }

    // The manager sees the new abort and recovers without being asked.
    const I2CBusState st = i2cBusBegin();
    const bool chip = readRegIs(BNO_CHIP_ID, 0xA0u);
    const bool gnss = i2cProbeAddress(GPS_DEFAULT_I2C_ADDRESS);
    char detail[80];
    snprintf(detail, sizeof(detail), "state=%d, CHIP_ID %s, gnss=%d",
             (int)st, chip ? "0xA0" : "WRONG", (int)gnss);
    report("T8 bus back in service by itself; both devices answer",
           (st == I2CBusState::Ready) && chip && gnss, detail);
}

// ── T9: a glitch in the middle of a read ─────────────────────────────────────
//
// SDA pulled low while SCL is high part-way through a 64-byte read: a START
// where a data bit belongs. The controller, owning the bus, sets BUSERR/ARBLOST
// and MB - never SB, which means a byte received cleanly - and lets go; SDA
// released under the same high SCL is a STOP, so everyone ends idle. The stock
// driver took that MB for a byte: requestFrom() returned a short count with a
// leftover byte in the buffer, or the FULL count when the glitch hit the last
// byte. The read must come back as 0 bytes and one bus error.
//
// Run at 100 kHz so SCL's high phase (5 us) comfortably holds the ISR's
// edge-to-pin latency; the glitch is placed only on a high SCL with SDA
// released (a 1 bit), and glitchLanded says whether it was.

/**
 * @brief From the TC5 one-shot: the next high SCL with SDA released gets a glitch.
 *
 * Only the byte in flight can be glitched: while this ISR holds the CPU the
 * controller finishes that byte and then stretches SCL low, waiting for the
 * ACK command the interrupted code has not issued yet. So the edge waits are
 * short (~100 us, ten bit periods at 100 kHz) and a byte with no 1 bit left
 * returns false for the handler to try a later one.
 */
static bool sdaGlitchUnderHighScl(void)
{
    static constexpr uint16_t EDGE_SPIN = 400u;   // ~100 us of polling
    for (uint8_t bits = 0u; bits < 10u; bits++) {
        uint16_t spin = 0u;
        while (padHigh(PIN_WIRE_SCL))  { if (++spin == EDGE_SPIN) return false; }   // SCL low
        spin = 0u;
        while (!padHigh(PIN_WIRE_SCL)) { if (++spin == EDGE_SPIN) return false; }   // rising edge
        if (!padHigh(PIN_WIRE_SDA)) continue;   // a 0 bit or an ACK: a low pull changes nothing
        padDriveLow(PIN_WIRE_SDA);              // START under the high SCL
        const bool sclHigh = padHigh(PIN_WIRE_SCL);
        delayMicroseconds(1);
        padRelease(PIN_WIRE_SDA);               // and a STOP, if SCL is still high
        return sclHigh;
    }
    return false;
}

static void midReadGlitchTest(void)
{
    if (i2cBusBegin() != I2CBusState::Ready) {
        report("T9 glitch mid-read", false, "bus not Ready before the test");
        return;
    }
    senseBusPads();
    Wire.setClock(100000UL);
    Wire.beginTransmission(SOX_ADDR);
    (void)Wire.write(SOX_WHOAMI);
    (void)Wire.endTransmission(true);
    delayMicroseconds(200);

    // 64 bytes at 100 kHz take ~5.8 ms. The first try is ~300 us in (byte 2 or
    // so), then every 60 us until a byte with a 1 bit left takes the glitch.
    const uint32_t errBefore = i2cWireBusErrors();
    const uint32_t toBefore  = i2cWireTimeouts();
    tc5Action = Tc5Action::SdaGlitch;
    armSclStall(300u);
    const uint32_t t0 = micros();
    const size_t got = Wire.requestFrom(SOX_ADDR, (size_t)64, true);
    const uint32_t tookUs = micros() - t0;
    NVIC_DisableIRQ(TC5_IRQn);
    TC5->COUNT16.CTRLA.bit.ENABLE = 0;
    tc5Action = Tc5Action::SclStall;
    const bool fired = tc5Fired;
    const bool landed = glitchLanded;
    const uint32_t errs = i2cWireBusErrors() - errBefore;
    const uint32_t tos  = i2cWireTimeouts() - toBefore;

    char detail[112];
    snprintf(detail, sizeof(detail), "got %u bytes in %lu us, bus errors +%lu, timeouts +%lu",
             (unsigned)got, (unsigned long)tookUs, (unsigned long)errs, (unsigned long)tos);
    if (!fired || !landed) {
        Serial.print("SKIP  T9 glitch mid-read  -- ");
        Serial.print(!fired ? "the read finished before the glitch" : "no high SCL with SDA released was caught");
        Serial.print("; ");
        Serial.println(detail);
    } else {
        report("T9 glitch mid-read: 0 bytes and one bus error, never bytes",
               (got == 0u) && (errs == 1u) && (tos == 0u), detail);
    }

    const I2CBusState st = i2cBusBegin();
    Wire.setClock(IMU_I2C_CLOCK_HZ);   // the recovery already did, unless it never ran
    const bool chip = readRegIs(BNO_CHIP_ID, 0xA0u);
    const bool gnss = i2cProbeAddress(GPS_DEFAULT_I2C_ADDRESS);
    snprintf(detail, sizeof(detail), "state=%d, CHIP_ID %s, gnss=%d",
             (int)st, chip ? "0xA0" : "WRONG", (int)gnss);
    report("T9b bus back in service after the glitch", (st == I2CBusState::Ready) && chip && gnss, detail);
}

static void runFullTest(void)
{
    passCount = 0;
    failCount = 0;

    Serial.println();
    Serial.println("=== I2C bus fault injection ===");

    // ---- 0: healthy baseline -----------------------------------------------
    if (i2cBusBegin() != I2CBusState::Ready) {
        Serial.println("FATAL: bus not Ready before the test even starts");
        return;
    }
    const bool baselineImu = i2cProbeAddress(SOX_ADDR);
    const bool baselineGps = i2cProbeAddress(GPS_DEFAULT_I2C_ADDRESS);
    {
        char detail[64];
        snprintf(detail, sizeof(detail), "imu=%d gnss=%d",
                 (int)baselineImu, (int)baselineGps);
        report("T0 baseline: both devices ACK", baselineImu && baselineGps, detail);
    }
    if (!baselineImu) {
        Serial.println("ABORT: no BNO055, nothing to wedge the bus with");
        return;
    }

    // ---- 1: wedge -----------------------------------------------------------
    const bool wedged = wedgeBus();
    bool sdaHigh = false;
    bool sclHigh = false;
    sampleLines(sdaHigh, sclHigh);
    {
        char detail[64];
        snprintf(detail, sizeof(detail), "SDA=%s SCL=%s",
                 sdaHigh ? "HIGH" : "LOW", sclHigh ? "HIGH" : "LOW");
        report("T1 slave left holding SDA low", wedged && !sdaHigh, detail);
    }
    if (!wedged) {
        Serial.println("Could not wedge the bus; recovery result below is meaningless.");
    }

    // ---- 1b: a real transfer on the wedged bus fails, and fails bounded -----
    // Pins straight back to SERCOM2, deliberately WITHOUT recovery.
    if (wedged) {
        Wire.begin();
        Wire.setClock(IMU_I2C_CLOCK_HZ);
        const uint32_t toBefore = i2cWireTimeouts();
        const uint32_t errBefore = i2cWireBusErrors();
        const uint32_t tb = micros();
        Wire.beginTransmission(GPS_DEFAULT_I2C_ADDRESS);
        const uint8_t rc = Wire.endTransmission(true);
        const uint32_t tookUs = micros() - tb;
        char detail[112];
        snprintf(detail, sizeof(detail), "endTransmission=%u in %lu us, timeouts +%lu, bus errors +%lu",
                 (unsigned)rc, (unsigned long)tookUs,
                 (unsigned long)(i2cWireTimeouts() - toBefore),
                 (unsigned long)(i2cWireBusErrors() - errBefore));
        report("T1b transfer on the wedged bus fails within the deadline",
               (rc != 0u) && (tookUs < WEDGED_XFER_MAX_US), detail);
    }

    // ---- 2: recover ---------------------------------------------------------
    const uint32_t t0 = micros();
    const I2CBusState st = i2cBusRecover();
    const uint32_t elapsed = micros() - t0;

    sampleLines(sdaHigh, sclHigh);
    {
        char detail[80];
        snprintf(detail, sizeof(detail), "state=%d in %lu us, SDA=%s SCL=%s",
                 (int)st, (unsigned long)elapsed,
                 sdaHigh ? "HIGH" : "LOW", sclHigh ? "HIGH" : "LOW");
        report("T2 recovery frees the bus",
               (st == I2CBusState::Ready) && sdaHigh && sclHigh, detail);
    }

    // ---- 3: the bus actually works again ------------------------------------
    // Re-open through the manager, because sampleLines() just took the pins to
    // GPIO and they have to go back to SERCOM2.
    Wire.begin();
    Wire.setClock(IMU_I2C_CLOCK_HZ);

    const bool afterImu = i2cProbeAddress(SOX_ADDR);
    const bool afterGps = i2cProbeAddress(GPS_DEFAULT_I2C_ADDRESS);
    {
        char detail[64];
        snprintf(detail, sizeof(detail), "imu=%d gnss=%d",
                 (int)afterImu, (int)afterGps);
        report("T3 both devices ACK after recovery", afterImu && afterGps, detail);
    }

    // ---- 4: a real sensor still reads correctly -----------------------------
    IMUDevice dev{};   // value-initialised: initializeIMU() reads dev.lifecycle
    IMUData   data;
    initIMUData(data);
    const IMUReturnStatus ist = initializeIMU(dev);

    bool sane = false;
    float mag = NAN;
    uint16_t samples = 0;
    if (ist == IMUReturnStatus::OK) {
        // initializeIMU() no longer configures the part — it ARMS a staged
        // bring-up that takes about 700 ms of wall clock, most of it the
        // BNO055's own reset delay. So this window has to drive imuInitTick()
        // and be long enough to contain that, where the previous part was
        // configured by the time the call returned.
        //
        // Judge the LAST reading, not the first: the fusion algorithm needs a
        // moment after a reset, and grabbing sample number one measures the
        // settling transient rather than the sensor.
        const unsigned long start = millis();
        while (!isTimeout(2000UL, start)) {
            watchdogFeed();
            (void)imuInitTick(dev);
            if (!imuIsReady(dev)) continue;
            const IMUReturnStatus rst = getIMUData(dev, data);
            // PARTIAL counts here. It means the fused output has not earned
            // belief yet — calibration takes longer than this window — and the
            // raw accelerometer this test judges is unaffected by that.
            if ((rst == IMUReturnStatus::OK || rst == IMUReturnStatus::PARTIAL) &&
                data.accelValid) samples++;
        }
        if (data.accelValid) {
            mag = sqrtf(data.accelX * data.accelX +
                        data.accelY * data.accelY +
                        data.accelZ * data.accelZ);
            sane = (mag > 9.0f && mag < 10.6f);
        }
    }
    {
        char detail[80];
        char magText[16];
        if (isnan(mag)) snprintf(magText, sizeof(magText), "--");
        else            snprintf(magText, sizeof(magText), "%d.%02d",
                                 (int)mag, (int)((mag - (int)mag) * 100.0f));
        snprintf(detail, sizeof(detail), "init=%d, %u samples, |a|=%s m/s2",
                 (int)ist, (unsigned)samples, magText);
        report("T4 IMU re-inits and reads 1 g after recovery", sane, detail);
    }

    // ---- 5: SCL held low under a read: the hang the bounds remove -----------
    sclStallTest(false);
    midReadStallTest();

    // ---- 7-9: glitches: a START/STOP where none belongs ---------------------
    phantomStartTest();
    midReadGlitchTest();

#ifdef DASHCAM_WIRE_INSTRUMENT
    // The stretch margin, measured: the longest bounded wait that COMPLETED this
    // boot (IMU bring-up and reads, GNSS probes) against the 25 ms deadline.
    Serial.print("longest completed bus wait: ");
    Serial.print(dashcamWireLongestWaitUs);
    Serial.print(" us (deadline ");
    Serial.print((unsigned long)DASHCAM_WIRE_WAIT_US);
    Serial.println(" us)");
#endif

    Serial.println();
    Serial.print("RESULT: ");
    Serial.print(passCount);
    Serial.print(" passed, ");
    Serial.print(failCount);
    Serial.println(" failed");
    Serial.println(failCount == 0 ? "ALL TESTS PASSED" : "FAILURES PRESENT");
    Serial.println("(a = run again, w = wedge only, r = recover only, c = control, h = help)");
}

// ── the unrecoverable case: SDA physically shorted to GND ────────────────────
//
// Software cannot stage this one. Holding SDA low from the MCU proves nothing,
// because i2cBusRecover() releases both pins to INPUT_PULLUP as its very first
// action — it would drop the MCU's own hold and then report success. Only an
// external short exercises the path where nine clocks change nothing.
//
// What must be true while shorted, and is the whole point of the test: NOTHING
// HANGS. Every entry point is supposed to notice the bus is unusable and return,
// rather than walk into the SAMD driver's unbounded flag waits. Each call below
// is therefore timed, and a call that returns in microseconds is a call that
// did not transact.

/** @brief Runs a call, prints its elapsed time, and returns it. */
static uint32_t timedCall(const char *label, bool &okOut, bool expectedFalse)
{
    const uint32_t t0 = micros();
    const bool result = i2cProbeAddress(SOX_ADDR);
    const uint32_t elapsed = micros() - t0;
    okOut = (result == !expectedFalse);
    Serial.print("    ");
    Serial.print(label);
    Serial.print(" -> ");
    Serial.print(result ? "true" : "false");
    Serial.print("  in ");
    Serial.print(elapsed);
    Serial.println(" us");
    return elapsed;
}

static void testStuckPhase1(void)
{
    passCount = 0;
    failCount = 0;

    Serial.println();
    Serial.println("=== Stuck-bus test, phase 1 ===");
    Serial.println("Hold the jumper on SDA (D11) to GND. Waiting up to 20 s...");

    // Self-trigger on the short rather than running the instant the command
    // arrives. A hand-held jumper cannot be synchronised with a serial command,
    // and a test that runs while contact happens to be open reports a healthy
    // bus and calls it a failure — which is exactly what happened on the first
    // attempt. Waiting for the line to actually go low removes the coordination
    // problem: the operator holds the probe, the board decides when to start.
    pinMode(PIN_WIRE_SDA, INPUT_PULLUP);
    pinMode(PIN_WIRE_SCL, INPUT_PULLUP);

    const unsigned long waitStart = millis();
    bool triggered = false;
    while (!isTimeout(20000UL, waitStart)) {
        if (digitalRead(PIN_WIRE_SDA) == LOW) {
            // Require it to stay down briefly: a contact bounce must not start a
            // test the jumper cannot then hold through.
            delay(20);
            if (digitalRead(PIN_WIRE_SDA) == LOW) { triggered = true; break; }
        }
    }

    if (!triggered) {
        Serial.println("ABORT: SDA never went low. Use 'l' to find contact first.");
        Wire.begin();
        Wire.setClock(IMU_I2C_CLOCK_HZ);
        return;
    }
    Serial.println("short detected -- running now, KEEP HOLDING");

    // Force a fresh evaluation: without this the manager could still be holding
    // a cached Ready from before the short was applied.
    const uint32_t t0 = micros();
    const I2CBusState st = i2cBusRecover();
    const uint32_t recoverUs = micros() - t0;

    {
        char detail[64];
        snprintf(detail, sizeof(detail), "state=%d (want 2=Stuck) in %lu us",
                 (int)st, (unsigned long)recoverUs);
        report("S1 recovery reports Stuck", st == I2CBusState::Stuck, detail);
    }

    if (st != I2CBusState::Stuck) {
        Serial.println("    (SDA does not read low - is the short actually on D11?)");
    }

    // Every client entry point must refuse, quickly, without transacting.
    bool ok = false;
    const uint32_t probeUs = timedCall("i2cProbeAddress", ok, true);
    report("S2 probe refuses without transacting", ok && (probeUs < 5000u), "");

    IMUDevice dev{};   // value-initialised: initializeIMU() reads dev.lifecycle
    const uint32_t t1 = micros();
    // The bus check moved. initializeIMU() no longer transacts at all — it arms
    // the staged bring-up — so asking IT for NOK_BUS_STUCK now tests nothing.
    // imuInitTick() is what touches the bus, and it checks per STEP rather than
    // once, which is the stronger property: a slave can wedge the lines between
    // two steps as easily as before the first one.
    //
    // Three ticks because the first only arms the machine and the second is the
    // power-on hold; the third is the first that would transact.
    (void)initializeIMU(dev);
    for (uint8_t i = 0u; i < 3u; i++) (void)imuInitTick(dev);
    const uint32_t imuUs = micros() - t1;
    {
        char detail[80];
        snprintf(detail, sizeof(detail), "why=%s (want bus-stuck) in %lu us",
                 bno055InitStatusName(dev.init.lastStatus), (unsigned long)imuUs);
        report("S3 IMU bring-up refuses on a stuck bus",
               dev.init.lastStatus == BNO055InitStatus::NOK_BUS_STUCK, detail);
    }

    const uint32_t t2 = micros();
    const GPSReturnStatus gst = initializeGPS_I2C(myGNSS);
    const uint32_t gpsUs = micros() - t2;
    {
        // NOK_BUS_STUCK (-7), not NOK_INIT_FAILED (-1).  This test predates the
        // status, and kept passing only because the old code could not tell the
        // two apart.  The distinction is the point of the fault injection: -1
        // says nothing answered at 0x42, which on a held-low bus would be a
        // WRONG diagnosis — nothing was ever asked.  -7 says the bus was unsafe
        // to touch, which is what is actually true here and what sends someone
        // looking at the lines rather than at the receiver.
        char detail[72];
        snprintf(detail, sizeof(detail), "status=%d (want %d) in %lu us",
                 (int)gst, (int)GPSReturnStatus::NOK_BUS_STUCK,
                 (unsigned long)gpsUs);
        report("S4 initializeGPS_I2C refuses with NOK_BUS_STUCK",
               gst == GPSReturnStatus::NOK_BUS_STUCK, detail);
    }

    // The retry rate limit: hammering i2cBusBegin() while stuck must not run
    // recovery on every call, or an address sweep would recover 112 times.
    const uint32_t t3 = micros();
    for (uint8_t i = 0; i < 20; i++) (void)i2cBusBegin();
    const uint32_t burstUs = micros() - t3;
    {
        char detail[64];
        snprintf(detail, sizeof(detail), "20 calls in %lu us", (unsigned long)burstUs);
        // One recovery is ~120 us; 20 unthrottled would be >2000 us.
        report("S5 stuck retry is rate limited", burstUs < 1500u, detail);
    }

    // Did the jumper survive the test? Without this every result above is
    // unfalsifiable: a short that let go halfway makes a genuine failure look
    // like a genuine pass, and nothing in the numbers would say so.
    pinMode(PIN_WIRE_SDA, INPUT_PULLUP);
    delayMicroseconds(10);
    const bool stillLow = (digitalRead(PIN_WIRE_SDA) == LOW);
    report("S6 short held for the whole test", stillLow,
           stillLow ? "SDA still low" : "JUMPER LOST CONTACT - results above are void");
    Wire.begin();
    Wire.setClock(IMU_I2C_CLOCK_HZ);

    Serial.println();
    Serial.print("phase 1: ");
    Serial.print(passCount);
    Serial.print(" passed, ");
    Serial.print(failCount);
    Serial.println(" failed");
    Serial.println(">>> Now REMOVE the short, then send '2'");
}

static void testStuckPhase2(void)
{
    passCount = 0;
    failCount = 0;

    Serial.println();
    Serial.println("=== Stuck-bus test, phase 2 (short MUST be removed now) ===");

    // Deliberately NOT calling i2cBusRecover() by hand. The point is that the
    // ordinary entry point heals itself: a node that came up on a stuck bus has
    // to recover on its own, with no operator and no reset.
    delay(I2C_BUS_RECOVER_RETRY_MS + 50UL);   // let the rate limiter open

    const I2CBusState st = i2cBusBegin();
    {
        char detail[48];
        snprintf(detail, sizeof(detail), "state=%d (want 1=Ready)", (int)st);
        report("S7 i2cBusBegin recovers by itself", st == I2CBusState::Ready, detail);
    }

    const bool a = i2cProbeAddress(SOX_ADDR);
    const bool g = i2cProbeAddress(GPS_DEFAULT_I2C_ADDRESS);
    {
        char detail[48];
        snprintf(detail, sizeof(detail), "imu=%d gnss=%d", (int)a, (int)g);
        report("S8 both devices ACK again", a && g, detail);
    }

    IMUDevice dev{};   // value-initialised: initializeIMU() reads dev.lifecycle
    IMUData   data;
    initIMUData(data);
    const IMUReturnStatus ist = initializeIMU(dev);

    bool sane = false;
    float mag = NAN;
    if (ist == IMUReturnStatus::OK) {
        // Ticked, and given room for the part's own 650 ms reset.
        //
        // This block still polled a device that was never configured:
        // initializeIMU() only ARMS the staged bring-up now, and without
        // imuInitTick() the machine never leaves its first stage — so
        // getIMUData() returned NOK_NOT_READY for the whole window and the test
        // reported a dead sensor as a failure of bus recovery. A test that
        // cannot pass is worse than a missing one, because it accuses the wrong
        // subsystem.
        const unsigned long start = millis();
        while (!isTimeout(2000UL, start)) {
            watchdogFeed();
            (void)imuInitTick(dev);
            if (imuIsReady(dev)) (void)getIMUData(dev, data);
        }
        if (data.accelValid) {
            mag = sqrtf(data.accelX * data.accelX +
                        data.accelY * data.accelY +
                        data.accelZ * data.accelZ);
            sane = (mag > 9.0f && mag < 10.6f);
        }
    }
    {
        char detail[64];
        char magText[16];
        if (isnan(mag)) snprintf(magText, sizeof(magText), "--");
        else            snprintf(magText, sizeof(magText), "%d.%02d",
                                 (int)mag, (int)((mag - (int)mag) * 100.0f));
        snprintf(detail, sizeof(detail), "init=%d, |a|=%s m/s2", (int)ist, magText);
        report("S9 IMU works after the short is removed", sane, detail);
    }

    // Bounded retry, not a single shot.
    //
    // initializeGPS_I2C() is a chain of UBX config commands, each waiting for an
    // ACK, issued against a module that is already streaming PVT at 4 Hz. That
    // is measurably not reliable first time: over four wedge/recover cycles the
    // call failed twice (-2 rate rejected, -6 config rejected) on the UNDISTURBED
    // attempt while succeeding 4/4 on the attempt straight after a bus recovery.
    // So the flakiness belongs to the config exchange, not to bus recovery — an
    // earlier reading of this test blamed recovery and the data disagreed.
    //
    // Asserting single-shot success would therefore test something the hardware
    // does not promise. What matters is that it comes up within a bounded number
    // of tries, which is exactly what production should be doing and currently
    // does not: mkr_zero.ino calls this once in setup() with no retry at all.
    static constexpr uint8_t GPS_INIT_ATTEMPTS = 3;
    GPSReturnStatus gst = GPSReturnStatus::NOK_INIT_FAILED;
    uint8_t attempt = 0;
    for (attempt = 1; attempt <= GPS_INIT_ATTEMPTS; attempt++) {
        gst = initializeGPS_I2C(myGNSS);
        if (gst == GPSReturnStatus::OK) break;
        delay(250);   // let the module finish whatever it was busy with
    }
    {
        char detail[64];
        snprintf(detail, sizeof(detail), "status=%d after %u attempt(s)",
                 (int)gst, (unsigned)attempt);
        report("S10 GNSS re-initialises within 3 attempts",
               gst == GPSReturnStatus::OK, detail);
    }

    Serial.println();
    Serial.print("phase 2: ");
    Serial.print(passCount);
    Serial.print(" passed, ");
    Serial.print(failCount);
    Serial.println(" failed");
}

/**
 * @brief Live SDA/SCL level readout, for finding the jumper contact.
 *
 * Phase 1 can only report "SDA is not low", which does not distinguish a jumper
 * on the wrong pin from one that is simply not making contact. Watching the
 * levels while the probe is moved answers that in seconds, and costs nothing
 * once the test is over.
 *
 * The pins are plain GPIO for the duration, so no I2C happens here; Wire is
 * handed back at the end.
 */
static void monitorLines(void)
{
    Serial.println();
    Serial.println("=== line monitor: SDA=D11 (PA22), SCL=D12 (PA23) ===");
    Serial.println("Touch the jumper to a GND pin and watch SDA go LOW. 15 s.");

    pinMode(PIN_WIRE_SDA, INPUT_PULLUP);
    pinMode(PIN_WIRE_SCL, INPUT_PULLUP);

    const unsigned long start = millis();
    unsigned long lastPrint = 0;
    while (!isTimeout(15000UL, start)) {
        if (isTimeout(250UL, lastPrint)) {
            lastPrint = millis();
            const bool sdaHigh = (digitalRead(PIN_WIRE_SDA) == HIGH);
            const bool sclHigh = (digitalRead(PIN_WIRE_SCL) == HIGH);
            Serial.print("  SDA(D11)=");
            Serial.print(sdaHigh ? "HIGH" : "LOW ");
            Serial.print("   SCL(D12)=");
            Serial.print(sclHigh ? "HIGH" : "LOW ");
            Serial.println(sdaHigh ? "" : "   <-- SDA is being held low");
        }
    }

    // Give the pins back to SERCOM2 so ordinary traffic works again.
    Wire.begin();
    Wire.setClock(IMU_I2C_CLOCK_HZ);
    Serial.println("monitor done, bus handed back to Wire");
}

void setup()
{
    pinMode(STATUS_INDICATOR, OUTPUT);
    digitalWrite(STATUS_INDICATOR, HIGH);

    Serial.begin(SERIAL_BAUDRATE);
    const unsigned long serialWaitStart = millis();
    while (!Serial && !isTimeout(SERIAL_READY_TIMEOUT_MS, serialWaitStart)) {
        // bounded; the timeout is the guarantee
    }

    // Read directly: resetCauseName() only knows it once watchdogArm() ran.
    const uint8_t rcause = PM->RCAUSE.reg;
    Serial.print("BOOT: RCAUSE=0x");
    Serial.print(rcause, HEX);
    Serial.println((rcause & PM_RCAUSE_WDT) ? " (WATCHDOG - the previous run hung)" : "");
    runFullTest();
}

void loop()
{
    if (Serial.available() > 0) {
        const int command = Serial.read();
        while (Serial.available() > 0) (void)Serial.read();   // drain

        if (command == 'a' || command == 'A') {
            runFullTest();
        } else if (command == 'w' || command == 'W') {
            const bool ok = wedgeBus();
            bool sdaHigh = false, sclHigh = false;
            sampleLines(sdaHigh, sclHigh);
            Serial.print("wedge: ");
            Serial.print(ok ? "applied" : "FAILED");
            Serial.print("  SDA=");
            Serial.print(sdaHigh ? "HIGH" : "LOW");
            Serial.print(" SCL=");
            Serial.println(sclHigh ? "HIGH" : "LOW");
        } else if (command == 'r' || command == 'R') {
            const I2CBusState st = i2cBusRecover();
            Serial.print("recover: state=");
            Serial.println((int)st);
        } else if (command == 'l' || command == 'L') {
            monitorLines();
        } else if (command == 'c' || command == 'C') {
            Serial.println("CONTROL: T5 through the core's unbounded SERCOM waits.");
            Serial.println("Expect silence, then a watchdog reset (~8 s) and 'BOOT: RCAUSE ... WATCHDOG'.");
            Serial.flush();
            watchdogArm(8000UL);
            sclStallTest(true);
            Serial.println("CONTROL DID NOT HANG - the injection does not reproduce the stall");
        } else if (command == '1') {
            testStuckPhase1();
        } else if (command == '2') {
            testStuckPhase2();
        } else if (command == 'h' || command == 'H') {
            Serial.println("a = full wedge/recover/glitch test (no hardware needed)");
            Serial.println("w = wedge only, r = recover only");
            Serial.println("c = control: T5 via the core's unbounded waits (instrumented build; ends in a watchdog reset)");
            Serial.println("l = live SDA/SCL level monitor (find the jumper contact)");
            Serial.println("1 = stuck-bus phase 1 -- SHORT SDA (D11) to GND first");
            Serial.println("2 = stuck-bus phase 2 -- REMOVE the short first");
        }
    }

    static unsigned long lastBlink = 0;
    if (isTimeout(500UL, lastBlink)) {
        lastBlink = millis();
        digitalWrite(STATUS_INDICATOR, !digitalRead(STATUS_INDICATOR));
    }
}
