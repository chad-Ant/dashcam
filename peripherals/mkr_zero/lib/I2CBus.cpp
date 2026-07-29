#include <Wire.h>

#include "DataDictionary.h"
#include "I2CBus.h"

/// Session state.  Static, because there is exactly one SERCOM2 and exactly one
/// set of pins behind it; a per-client copy is precisely the arrangement this
/// module exists to remove.
static I2CBusState busState = I2CBusState::Uninitialized;

/// @c millis() of the last recovery attempt, so a stuck bus is retried at a
/// bounded rate rather than on every caller's every call.
static uint32_t lastRecoverAttemptMs = 0u;

/// True once watchdogArm() has run, so watchdogFeed() is a no-op before then.
static bool watchdogArmed = false;
/// RCAUSE captured at first arm, before anything can overwrite it.
static uint8_t resetCause = 0u;

// ─── watchdog ─────────────────────────────────────────────────────────────────

void watchdogArm(uint32_t periodMs){
    if (watchdogArmed) return;

    resetCause = PM->RCAUSE.reg;   // capture before arming; see watchdogCausedReset()

    // Clock the WDT from the always-on ultra-low-power 32 kHz oscillator divided
    // to 1024 Hz, so the period does not move with CPU clock changes.
    GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(4);          // 32768 / 2^(4+1) = 1024 Hz
    while (GCLK->STATUS.bit.SYNCBUSY) { }
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_SRC_OSCULP32K |
                        GCLK_GENCTRL_DIVSEL | GCLK_GENCTRL_GENEN;
    while (GCLK->STATUS.bit.SYNCBUSY) { }
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT | GCLK_CLKCTRL_GEN_GCLK2 | GCLK_CLKCTRL_CLKEN;
    while (GCLK->STATUS.bit.SYNCBUSY) { }

    WDT->CTRL.reg = 0;                                   // disable while configuring
    while (WDT->STATUS.bit.SYNCBUSY) { }

    // Round UP to the next supported period: a watchdog that fires early is a
    // reboot loop, which is a worse failure than a slightly longer hang.
    uint8_t per = WDT_CONFIG_PER_16K_Val;                // ~16 s ceiling
    const uint32_t cycles = (periodMs * 1024UL) / 1000UL;
    if      (cycles <= 1024UL) per = WDT_CONFIG_PER_1K_Val;   // ~1 s
    else if (cycles <= 2048UL) per = WDT_CONFIG_PER_2K_Val;   // ~2 s
    else if (cycles <= 4096UL) per = WDT_CONFIG_PER_4K_Val;   // ~4 s
    else if (cycles <= 8192UL) per = WDT_CONFIG_PER_8K_Val;   // ~8 s

    WDT->CONFIG.reg = WDT_CONFIG_PER(per);
    while (WDT->STATUS.bit.SYNCBUSY) { }
    WDT->CTRL.reg = WDT_CTRL_ENABLE;
    while (WDT->STATUS.bit.SYNCBUSY) { }

    watchdogArmed = true;
}

void watchdogFeed(void){
    if (!watchdogArmed) return;
    if (WDT->STATUS.bit.SYNCBUSY) return;   // a clear still in flight is as good as a feed
    WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
}

bool watchdogCausedReset(void){
    return (resetCause & PM_RCAUSE_WDT) != 0u;
}

/// Deadline for SCL to rise after being released during recovery.  A slave that
/// is clock-stretching holds SCL down; counting that as a delivered clock pulse
/// would burn all nine pulses without the slave seeing any of them.
static constexpr uint32_t SCL_RELEASE_TIMEOUT_US = 100u;

/**
 * @brief Turns on the input buffer for SDA/SCL while they stay muxed to SERCOM2.
 *
 * pinPeripheral() sets PMUXEN so the peripheral owns the pad, but never sets
 * PINCFG.INEN — so PORT->IN reads zero for those pins and digitalRead() lies.
 * Setting INEN alongside PMUXEN is explicitly allowed on the SAMD21: the
 * peripheral keeps control of the pad while the input synchroniser also samples
 * it, which is what makes a cheap "are the lines actually idle?" test possible
 * without stealing the pins back from the SERCOM.
 */
static void enableLineSense(void){
    const uint8_t pins[2] = { PIN_WIRE_SDA, PIN_WIRE_SCL };
    for (uint8_t i = 0u; i < 2u; i++){
        const EPortType port = g_APinDescription[pins[i]].ulPort;
        const uint32_t  pin  = g_APinDescription[pins[i]].ulPin;
        PORT->Group[port].PINCFG[pin].reg |= static_cast<uint8_t>(PORT_PINCFG_INEN);
    }
}

/**
 * @brief Drives a bus line low without ever driving it high.
 *
 * The order is the whole point.  @c pinMode(INPUT_PULLUP) sets the SAMD21
 * output latch HIGH (that is how the chip selects pull-up over pull-down), and
 * @c pinMode(OUTPUT) only touches DIRSET — it leaves the latch alone.  So the
 * intuitive sequence
 * @code
 *   pinMode(pin, OUTPUT); digitalWrite(pin, LOW);
 * @endcode
 * drives the pin push-pull HIGH for the instant between the two calls.  On a
 * bus being recovered that is the worst possible moment for it: the line is
 * held low by a wedged slave, so the MCU's high driver fights the slave's low
 * driver directly across the pins.
 *
 * Writing the latch first is safe because @c digitalWrite() on a pin still
 * configured as an input clears PULLEN rather than driving anything, leaving
 * the line floating for the external pull-ups to hold.
 */
static void driveLow(uint8_t pin){
    digitalWrite(pin, LOW);   // preload the output latch while still an input
    pinMode(pin, OUTPUT);     // now DIRSET can only ever drive LOW
}

/** @brief Releases a line to the external pull-ups. */
static inline void releaseLine(uint8_t pin){
    pinMode(pin, INPUT_PULLUP);
}

/**
 * @brief Releases SCL and waits, bounded, for it to actually read high.
 * @return @c true when SCL rose within the timeout.
 */
static bool releaseSclAndWait(void){
    releaseLine(PIN_WIRE_SCL);
    const uint32_t start = micros();
    while ((micros() - start) < SCL_RELEASE_TIMEOUT_US){
        if (digitalRead(PIN_WIRE_SCL) == HIGH) return true;
    }
    return false;   // slave is stretching, or SCL is shorted low
}

I2CBusState i2cBusRecover(void){
    lastRecoverAttemptMs = millis();

    // Take the pins back from SERCOM2 so they can be worked as plain GPIO.
    releaseLine(PIN_WIRE_SCL);
    releaseLine(PIN_WIRE_SDA);
    delayMicroseconds(10);

    // Up to nine clocks: enough for a slave to finish the byte plus the ACK it
    // believes it is in the middle of.  Fixed bound, so this terminates however
    // badly the slave behaves.
    for (uint8_t i = 0u; i < 9u; i++){
        if (digitalRead(PIN_WIRE_SDA) == HIGH) break;   // already released

        driveLow(PIN_WIRE_SCL);
        delayMicroseconds(5);
        // A pulse the slave never saw is not a pulse.  If SCL cannot rise we
        // stop rather than spending the remaining budget clocking a line that
        // is not moving.
        if (!releaseSclAndWait()) break;
        delayMicroseconds(5);
    }

    // STOP, and only a STOP.  SCL must go low BEFORE SDA: pulling SDA low while
    // SCL is already high is a START condition, and a START never followed by
    // an address leaves slaves waiting for a transaction that never comes.  The
    // u-blox DDC port is one that notices — emitting START-then-STOP here made
    // its next bring-up fail intermittently, which looked exactly like an
    // IMU/GNSS bus conflict and was really this routine being impolite.
    driveLow(PIN_WIRE_SCL);
    delayMicroseconds(5);
    driveLow(PIN_WIRE_SDA);
    delayMicroseconds(5);

    // Whether the STOP happened at all hinges on this result, so it must not be
    // discarded.  A STOP is SDA rising while SCL is HIGH; if SCL never came up,
    // releasing SDA below produces no STOP whatsoever.  The final both-lines-high
    // check cannot tell the difference — SCL drifting high during the settle
    // delay leaves exactly the same picture — so without this flag a bus that was
    // never released would be handed back as Ready.
    const bool sclHighForStop = releaseSclAndWait();
    if (sclHighForStop) delayMicroseconds(5);

    // Released unconditionally: even on failure the MCU must not walk away still
    // driving SDA low, which would wedge the bus on our own account.
    releaseLine(PIN_WIRE_SDA);
    delayMicroseconds(5);

    const bool released = sclHighForStop &&
                          (digitalRead(PIN_WIRE_SDA) == HIGH) &&
                          (digitalRead(PIN_WIRE_SCL) == HIGH);

    // Hand the pins back to the SERCOM.  Wire.begin() re-runs pinPeripheral() on
    // both, which is what actually undoes the pinMode() calls above.
    Wire.begin();
    Wire.setClock(IMU_I2C_CLOCK_HZ);
    enableLineSense();   // must follow Wire.begin(), which re-writes PINCFG

    busState = released ? I2CBusState::Ready : I2CBusState::Stuck;
    return busState;
}

/// How long both lines are given to come up before the bus is called stuck.
///
/// A STOP is not instantaneous, and the SAMD driver does not wait for it: the
/// last thing @c TwoWire::endTransmission() does is @c prepareCommandBitsWire(),
/// which synchronises the CTRLB REGISTER write and returns — leaving the STOP
/// still being clocked onto the wire.  So a caller that transacts and then
/// immediately asks whether the lines are idle can catch them legitimately low.
/// Sampling once made that a false @c Stuck verdict, and back-to-back probes
/// (@c i2cProbeAddress twice, or a bus scan) are exactly the pattern that hits
/// it.
///
/// 500 us is 50 bit periods at 100 kHz, far more than any STOP needs, and taken
/// from the settle allowance the Wire regression suite already uses for the same
/// reason.  It costs nothing on a healthy bus — the loop exits on its first
/// sample — and is only ever paid while the bus is genuinely down.
static constexpr uint32_t LINE_SETTLE_TIMEOUT_US = 500u;

/**
 * @brief True when both bus lines are released (idle high), after a bounded settle.
 *
 * Readable even though the pins belong to SERCOM2, because @c enableLineSense()
 * turns on the SAMD21 input buffer for them.  pinPeripheral() sets PMUXEN but
 * leaves PINCFG.INEN clear, which is why a naive digitalRead() of a muxed pin
 * always returns LOW — the trap an earlier revision of this file fell into.
 *
 * Only meaningful BETWEEN transactions: mid-transfer both lines are legitimately
 * low, so this must never be used as a transfer-in-progress test.
 */
static bool linesIdle(void){
    const uint32_t start = micros();
    do {
        if ((digitalRead(PIN_WIRE_SDA) == HIGH) &&
            (digitalRead(PIN_WIRE_SCL) == HIGH)) return true;
    } while ((micros() - start) < LINE_SETTLE_TIMEOUT_US);
    return false;
}

I2CBusState i2cBusBegin(void){
    // Cached Ready is not proof the bus is still usable.  A device that browns
    // out or resets mid-drive can hold SDA at any time AFTER a successful
    // bring-up, and every later caller would then walk into the SAMD driver's
    // unbounded flag waits on the strength of a stale flag.  Re-checking costs
    // two register reads, against a hang that only a watchdog can end.
    if (busState == I2CBusState::Ready){
        if (linesIdle()) return busState;
        busState = I2CBusState::Stuck;   // fall through to the rate-limited retry
    }

    // Stuck is a symptom, not a verdict.  The slave holding SDA may release it,
    // and a shorted line may be unshorted, so latching Stuck permanently would
    // turn a temporary fault into a dead node — the exact failure this whole
    // module exists to prevent.  Retry, but rate-limited: probes and scans also
    // enter here, and an address sweep must not run recovery 112 times.
    if (busState == I2CBusState::Stuck){
        if ((millis() - lastRecoverAttemptMs) < I2C_BUS_RECOVER_RETRY_MS) return busState;
        return i2cBusRecover();
    }

    // Recovery runs BEFORE Wire.begin(), on the very first call, whichever
    // client gets here first.  That ordering is the reason this module exists:
    // a wedged slave must be released before anyone transacts, and "detect then
    // recover" is unimplementable when the detecting probe is itself what hangs.
    return i2cBusRecover();
}

I2CBusState i2cBusState(void){
    return busState;
}

/** @brief True for an address in the usable 7-bit range (reserved blocks excluded). */
static inline bool isUsableI2CAddress(uint8_t address){
    return (address >= 0x08u) && (address <= 0x77u);
}

bool i2cProbeAddress(uint8_t address){
    if (!isUsableI2CAddress(address)) return false;
    if (i2cBusBegin() != I2CBusState::Ready) return false;

    Wire.beginTransmission(address);
    return Wire.endTransmission(true) == 0u;
}

uint8_t scanI2CBus(uint8_t *found, uint8_t maxCount){
    if (i2cBusBegin() != I2CBusState::Ready) return 0u;

    uint8_t count = 0u;
    // Fixed bound: the 7-bit address space minus the reserved blocks at both
    // ends.  Nothing here depends on what the bus answers, so the sweep cannot
    // run long however badly a device misbehaves.
    for (uint8_t address = 0x08u; address <= 0x77u; address++){
        Wire.beginTransmission(address);
        if (Wire.endTransmission(true) != 0u) continue;

        if ((found != nullptr) && (count < maxCount)) found[count] = address;
        if (count < 0xFFu) count++;
    }
    return count;
}
