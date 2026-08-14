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

uint8_t resetCauseRaw(void){
    return resetCause;
}

/// Which lines were still low at the end of the last recovery attempt.
static uint8_t lastStuckMask = 0u;

uint8_t i2cStuckLines(void){
    return lastStuckMask;
}

const char *i2cStuckReason(void){
    // Ordered by what each one implicates. Both lines low is checked first
    // because it is the supply case, and a supply fault explains the other two
    // rather than sitting alongside them.
    const uint8_t m = lastStuckMask;
    if (m == 0u) return "bus free";
    if ((m & I2C_STUCK_SDA_LOW) && (m & I2C_STUCK_SCL_LOW)) {
        return "BOTH lines low - supply collapse or a short, not a bus hang";
    }
    if (m & I2C_STUCK_SDA_LOW) {
        // The distinction that survives once both modules measure 3.3 V at VIN.
        if (m & I2C_STUCK_SDA_NEVER_MOVED) {
            return "SDA NEVER moved through 18 clocks - shorted/clamped, not a stuck slave";
        }
        return "SDA moved while clocked but settled low - a slave is re-asserting it";
    }
    if (m & (I2C_STUCK_SCL_LOW | I2C_STUCK_SCL_NO_RISE)) {
        return "SCL will not rise - check the pull-up, or a slave stretching forever";
    }
    return "released late";
}

const char *resetCauseName(void){
    // Order matters: WDT and the brownout detectors are the two that say
    // something WENT WRONG, so they are tested before the benign causes.
    // Reported unconditionally by the caller rather than only when it was the
    // watchdog — an earlier version printed a line only for WDT, so every other
    // cause had to be inferred from the ABSENCE of that line. A supply brownout
    // and a clean power-cycle then looked identical, which on a vehicle rig is
    // the single most useful distinction there is.
    if (resetCause & PM_RCAUSE_WDT)   return "watchdog (previous run hung)";
    if (resetCause & PM_RCAUSE_BOD33) return "BROWNOUT on VDDANA (3.3V rail sagged)";
    if (resetCause & PM_RCAUSE_BOD12) return "BROWNOUT on the core 1.2V regulator";
    if (resetCause & PM_RCAUSE_EXT)   return "external reset pin";
    if (resetCause & PM_RCAUSE_SYST)  return "software request";
    if (resetCause & PM_RCAUSE_POR)   return "power-on (cold start)";
    return "unknown";
}

// ─── hang quarantine ──────────────────────────────────────────────────────────

bool bootAfterHang(void){
    // RCAUSE, captured in watchdogArm() before anything can overwrite it, is the
    // only reset-surviving state this chip offers.  It is enough, because it
    // answers the one question that matters: did the previous run HANG?
    //
    // A finer marker naming WHICH step hung would be better, and is what an
    // earlier revision attempted with a __attribute__((section(".noinit")))
    // variable.  It does not work on this platform and the failure is silent.
    // The mkrzero linker script (variants/mkrzero/linker_scripts/gcc/
    // flash_with_bootloader.ld) defines no .noinit output section, so such a
    // variable becomes an orphan placed inside the __bss_start__..__bss_end__
    // range that the C runtime zeroes before setup() runs.  Measured on the
    // bench: after a watchdog reset the marker read back magic=0x0 stage=0 on
    // every single boot, so the quarantine never triggered and the board
    // reboot-looped every ~9 s exactly as it had before.
    //
    // The alternatives were all worse than losing the stage detail: vendoring
    // the variant linker script is per-variant, unguarded and easy to drop; a
    // fixed absolute SRAM address collides with the descending stack; and flash
    // would be worn out fastest by the reboot loop this exists to break.
    //
    // NOT consumed on read, and that is the correction that matters.  An earlier
    // version cleared the flag for the first caller, which had two consequences,
    // both wrong: a second subsystem asking the same question got "no", and —
    // far worse — the quarantine only suppressed ONE attempt.  Anything that
    // retried afterwards walked straight back into the hang, the watchdog reset
    // the board, and the loop continued at the retry interval instead of being
    // broken.  A boot either follows a hang or it does not; that fact is true
    // for the whole boot and every caller must see the same answer.
    return watchdogCausedReset();
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
/**
 * @brief Releases @p pin and waits, bounded, for it to actually reach a high.
 *
 * Both lines need this, not just SCL. A released line does not rise instantly:
 * it is an RC edge against the pull-ups, and this bus carries an ESLOV cable to
 * the GNSS, so the capacitance is whatever the cable and a second module happen
 * to add. Sampling once, immediately, measures the rise time rather than the
 * state of the bus.
 */
static bool releaseAndWait(uint8_t pin){
    releaseLine(pin);
    const uint32_t start = micros();
    while ((micros() - start) < SCL_RELEASE_TIMEOUT_US){
        if (digitalRead(pin) == HIGH) return true;
    }
    return false;   // held low by a slave, or shorted
}

static bool releaseSclAndWait(void){
    return releaseAndWait(PIN_WIRE_SCL);   // stretching, or SCL shorted low
}

I2CBusState i2cBusRecover(void){
    lastRecoverAttemptMs = millis();

    // Take the pins back from SERCOM2 so they can be worked as plain GPIO.
    releaseLine(PIN_WIRE_SCL);
    releaseLine(PIN_WIRE_SDA);
    delayMicroseconds(10);

    // Whether SDA EVER let go during clocking, at any point.
    //
    // This is the measurement that separates the two faults still standing once
    // supply has been ruled out at both modules. A slave stuck mid-byte releases
    // SDA the moment it is clocked past the bit it was holding — so SDA moves,
    // even if it ends up low again. A line shorted to ground, or a damaged pad
    // clamping it, NEVER moves, no matter how many clocks it is given. Both look
    // identical in a pass/fail verdict and need completely different repairs.
    bool sdaEverHigh = false;

    // 18 clocks, not 9. Nine covers a byte plus its ACK, which is the textbook
    // case; doubling it costs 180 us on a bus that is already broken and covers
    // a slave that has more than one byte queued. The bound still guarantees
    // termination however badly the slave behaves.
    for (uint8_t i = 0u; i < I2C_RECOVER_CLOCKS; i++){
        if (digitalRead(PIN_WIRE_SDA) == HIGH) { sdaEverHigh = true; break; }

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
    //
    // SDA is then given the SAME bounded settle SCL already gets. It used to be
    // released, given a flat 5 us, and sampled ONCE - so the verdict on half the
    // bus rested on a single reading taken before a slow line could possibly
    // have risen.
    //
    // 5 us is not a margin, it is roughly the RC rise time itself. The GNSS
    // hangs off an ESLOV cable, and cable plus connector plus a second module's
    // pin capacitance easily reaches a few hundred pF; against the board
    // pull-ups that is microseconds to reach a valid high. A perfectly healthy
    // bus therefore reads LOW at 5 us and is declared Stuck - permanently, since
    // every later retry repeats the same too-early sample and reaches the same
    // verdict. That is the shape of these logs exactly: never a recovery, on a
    // rail measured steady at 3.3 V with no short.
    const bool sdaHigh = releaseAndWait(PIN_WIRE_SDA);
    const bool sclHigh = (digitalRead(PIN_WIRE_SCL) == HIGH);
    const bool released = sclHighForStop && sdaHigh && sclHigh;

    // WHICH line failed, recorded before the pins go back to the SERCOM.
    //
    // "Stuck" alone collapses three unrelated hardware faults into one code,
    // and a run that retries twenty times reports the same -5 every time while
    // saying nothing about where to put the probe. SDA held low is a slave stuck
    // mid-byte or running on parasitic power; SCL that will not rise is a dead
    // pull-up, a short, or a slave clock-stretching forever; both low is a
    // supply collapse. Different investigations entirely.
    lastStuckMask = 0u;
    if (!sdaHigh)        lastStuckMask |= I2C_STUCK_SDA_LOW;
    if (!sclHigh)        lastStuckMask |= I2C_STUCK_SCL_LOW;
    if (!sclHighForStop) lastStuckMask |= I2C_STUCK_SCL_NO_RISE;
    // Only meaningful when SDA ended low: it says whether the line was ever
    // observed to move while being clocked.
    if (!sdaHigh && !sdaEverHigh) lastStuckMask |= I2C_STUCK_SDA_NEVER_MOVED;

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
