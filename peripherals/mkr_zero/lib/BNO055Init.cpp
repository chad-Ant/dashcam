#include "BNO055Init.h"
// For bno055CalibWrite() only. That function is register work with no storage in
// it — the file side of BNO055Calib lives entirely in its .cpp — so this does
// not drag the SD card into the bring-up machine.
#include "BNO055Calib.h"

// ─── register bits this file writes ───────────────────────────────────────────
//
// Spelled out rather than assembled from the driver's BITSLICE macros.  These
// are single-bit commands into one register, and `SYS_TRIGGER = 0x20` with a
// name beside it is legible at the point of use, where a SET_BITSLICE call
// expands to three lines that have to be read back into the same fact.

/// SYS_TRIGGER bit 5 — reset the whole device.
#define BNO055_SYS_TRIGGER_RST_SYS   0x20u
/// SYS_TRIGGER bit 7 — clock from the external 32.768 kHz crystal.
#define BNO055_SYS_TRIGGER_CLK_SEL   0x80u

/// UNIT_SEL, written explicitly rather than trusted as a power-on default.
///
///   bit 0 = 0  acceleration in m/s2   (1 would be mg)
///   bit 1 = 0  angular rate in deg/s  (1 would be rad/s)
///   bit 2 = 0  Euler angles in degrees
///   bit 4 = 0  temperature in Celsius (1 would be Fahrenheit)
///   bit 7 = 0  Windows orientation convention
///
/// Every one of those is what this project wants and, as far as the datasheet
/// says, what the part comes up with.  It is written anyway because the cost is
/// one register and the alternative is a silent unit change if a future part,
/// clone or restored calibration blob disagrees — and a factor of 1000 hiding in
/// an acceleration reading is exactly the class of error this driver exists to
/// make impossible.
#define BNO055_UNIT_SEL_VALUE        0x00u

/// The bits of UNIT_SEL that are verified: 7, 4, 2, 1, 0.  Bits 6, 5 and 3 are
/// reserved, and a reserved bit is by definition not one whose read-back value
/// is specified.
///
/// Bit 7 — the Euler orientation convention — WAS excluded for a while, on the
/// strength of it reading back as 1 across nine bring-ups whatever was written.
/// That reasoning was wrong and the exclusion is reverted: the part was on
/// register PAGE 1 for all of those runs, so 0x3B was not UNIT_SEL at all. With
/// PAGE_ID written first it reads and writes exactly as documented.
///
/// Recorded because it is a good illustration of a bad inference — nine
/// consistent observations of a stuck bit, and the bit was never being read.
#define BNO055_UNIT_SEL_MASK         0x97u

/// PAGE_ID, spelled to match the driver header's mixed-case macro.
#define BNO055_PAGE_ID_ADDR          BNO055_Page_ID_ADDR

// ─── page 1 registers ─────────────────────────────────────────────────────────
//
// Spelled out here because the vendored driver's header does not name them: it
// defines the page-0 map and the bit positions for INT_STA, but not the page-1
// addresses that arm the interrupts. Datasheet rev 1.4, table 4-3.

#define BNO055_P1_INT_MSK_ADDR        0x0Fu   ///< Which interrupts drive the INT PIN.
#define BNO055_P1_INT_EN_ADDR         0x10u   ///< Which interrupts are evaluated at all.
#define BNO055_P1_ACC_INT_SET_ADDR    0x12u   ///< Per-axis enables and sample count.
#define BNO055_P1_ACC_HG_DUR_ADDR     0x13u   ///< High-G duration.
#define BNO055_P1_ACC_HG_THRES_ADDR   0x14u   ///< High-G threshold.

/// INT_EN / INT_MSK bit 5 — accelerometer High-G.
#define BNO055_INT_BIT_ACC_HIGH_G     0x20u

/// ACC_INT_Settings bits 7:5 — High-G on the Z, Y and X axes.
///
/// ALL THREE, deliberately. The obvious economy is to disable whichever axis
/// carries gravity, so the threshold can sit lower — but this project does not
/// guess a mounting orientation anywhere else and must not start here. Which
/// axis is vertical depends on how the board is bolted into the vehicle, and a
/// driver that assumed wrongly would disable the one axis an impact arrives on.
#define BNO055_ACC_INT_HG_ALL_AXES    0xE0u

/// The bit @c ProbeWrite uses to prove writes take effect: GYR_UNIT, bit 1.
///
/// Chosen because it is 0 after reset — so seeing it appear can only mean the
/// part acted — because it is harmless while set, and because @c SetUnits
/// clears it moments later as part of its normal job, which closes the round
/// trip without a second special-purpose write.
#define BNO055_UNIT_SEL_PROBE        0x02u

/// OPR_MODE occupies the low nibble; the upper bits read back as zero but are
/// masked rather than assumed.
#define BNO055_OPR_MODE_MASK         0x0Fu

/// Settle time between a configuration write and reading it back (ms).
///
/// Bounded and tiny, and deliberately not zero: the write and the read-back
/// were previously issued back to back with no gap at all, which is the one
/// access pattern the bring-up had never exercised before it started failing.
/// Two milliseconds inside a step is not the kind of blocking this machine
/// exists to avoid — that is the 650 ms reset, which remains a deadline.
#define BNO055_REG_SETTLE_MS         2UL

// ─── register helpers ─────────────────────────────────────────────────────────
//
// Straight through the transport hooks, NOT through the driver's register
// helpers.  bno055_write_register() would work, but it reaches the device
// through the driver's file-static context pointer, and routing bring-up
// through global state that bno055_init() must already have installed makes the
// ordering a matter of trust.  These take the address explicitly.

static bool regRead8(uint8_t addr, uint8_t reg, uint8_t &value)
{
    unsigned char v = 0u;
    if (bno055BusRead(addr, reg, &v, 1u) != 0) return false;
    value = (uint8_t)v;
    return true;
}

static bool regWrite8(uint8_t addr, uint8_t reg, uint8_t value)
{
    unsigned char v = (unsigned char)value;
    return bno055BusWrite(addr, reg, &v, 1u) == 0;
}

/**
 * @brief Writes a configuration register, reads it back, and records both.
 *
 * The read-back is the point.  A write that is NACKed is reported by the
 * transport, but a write the part accepts and then ignores — because it is in
 * the wrong mode, or still switching out of one, or has no working clock — is
 * not, and that is the failure this whole file is arranged around.  @p mask
 * exists because not every register reads back every bit it is written.
 *
 * Everything it learns goes into @p state, because "config-failed" on its own
 * is not a diagnosis.  It covers a NACKed write, a failed read-back and a
 * silently ignored write, which are three different faults with three different
 * repairs, and telling them apart afterwards is impossible if the bytes were
 * thrown away.
 */
static bool configWrite(BNO055InitState &state, uint8_t reg, uint8_t value, uint8_t mask)
{
    state.lastRegAddr    = reg;
    state.lastRegWrote   = value;
    state.lastRegRead    = 0u;
    state.lastRegReadOk  = false;

    state.lastRegWriteOk = regWrite8(state.address, reg, value);
    if (!state.lastRegWriteOk) return false;

    delay(BNO055_REG_SETTLE_MS);

    uint8_t readback = 0u;
    if (!regRead8(state.address, reg, readback)) return false;
    state.lastRegReadOk = true;
    state.lastRegRead   = readback;

    return (uint8_t)(readback & mask) == (uint8_t)(value & mask);
}

/**
 * @brief Best-effort capture of the part's own status, for a failure log.
 *
 * Failures are allowed to fail here: this runs on a path that is already going
 * wrong, and a status read that also fails is itself informative.
 */
static void captureDiagnostics(BNO055InitState &state)
{
    (void)regRead8(state.address, BNO055_OPR_MODE_ADDR,   state.opModeSeen);
    (void)regRead8(state.address, BNO055_SYS_STATUS_ADDR, state.sysStatus);
    (void)regRead8(state.address, BNO055_SYS_ERR_ADDR,    state.sysError);
}

/**
 * @brief Gives up on the external crystal and switches back to the internal one.
 *
 * Called when a configuration write fails while the crystal is selected, which
 * is the only test of a crystal that is worth anything.  Reading SYS_ERR after
 * selecting it — which is what this machine did first, and what let a broken
 * configuration through — asks the part whether it noticed a problem; a part
 * whose clock has stopped is in no position to answer.  Whether it will accept
 * a register write is a functional question, and the part answers it by
 * behaving or not behaving.
 *
 * @return true when a fallback was performed and the caller should retry.
 */
static bool fallBackToInternalClock(BNO055InitState &state)
{
    if (!state.externalCrystal || state.clockFallback) return false;

    (void)regWrite8(state.address, BNO055_SYS_TRIGGER_ADDR, 0x00u);
    state.externalCrystal = false;
    state.clockFallback   = true;

    // Back through the RESET, not merely onward from here.  Bench evidence:
    // clearing CLK_SEL on a part that had already failed with the crystal
    // selected did not recover it, and the immediate retry failed identically.
    // A part that cannot run on the clock it was given does not come back
    // because the bit was cleared; it needs the reboot.
    state.stage       = BNO055InitStage::Reset;
    state.nextStepMs  = millis();
    state.stepRetries = 0u;
    return true;
}

// ─── stage plumbing ───────────────────────────────────────────────────────────

const char *bno055InitStageName(BNO055InitStage stage)
{
    switch (stage){
        case BNO055InitStage::Idle:         return "idle";
        case BNO055InitStage::WaitBoot:     return "wait-boot";
        case BNO055InitStage::FindDevice:   return "find-device";
        case BNO055InitStage::Reset:        return "reset";
        case BNO055InitStage::Reacquire:    return "reacquire";
        case BNO055InitStage::SetPageId:    return "set-page-id";
        case BNO055InitStage::ProbeWrite:   return "probe-write";
        case BNO055InitStage::SetPowerMode: return "set-power-mode";
        case BNO055InitStage::ClearSysTrigger: return "clear-sys-trigger";
        case BNO055InitStage::SetUnits:     return "set-units";
        case BNO055InitStage::RestoreCalib: return "restore-calib";
        case BNO055InitStage::SetHighG:     return "set-high-g";
        case BNO055InitStage::SetClock:     return "set-clock";
        case BNO055InitStage::CheckClock:   return "check-clock";
        case BNO055InitStage::SetOpMode:    return "set-op-mode";
        case BNO055InitStage::VerifyOpMode: return "verify-op-mode";
        case BNO055InitStage::Configured:   return "configured";
        case BNO055InitStage::Quarantined:  return "quarantined";
        default:                            return "failed";
    }
}

const char *bno055InitStatusName(BNO055InitStatus status)
{
    switch (status){
        case BNO055InitStatus::OK:                   return "ok";
        case BNO055InitStatus::NOK_BUS_STUCK:        return "bus-stuck";
        case BNO055InitStatus::NOK_NOT_FOUND:        return "not-found";
        case BNO055InitStatus::NOK_LOST_AFTER_RESET: return "lost-after-reset";
        case BNO055InitStatus::NOK_CONFIG_FAILED:    return "config-failed";
        case BNO055InitStatus::NOK_MODE_FAILED:      return "mode-failed";
        case BNO055InitStatus::NOK_SYSTEM_ERROR:     return "system-error";
        case BNO055InitStatus::NOK_WRITES_IGNORED:   return "writes-ignored";
        default:                                     return "unknown";
    }
}

bool bno055InitBegin(BNO055InitState &state, uint8_t opMode)
{
    // Terminal, and it has to be terminal HERE rather than only inside the tick:
    // this function assigns the stage directly, so a caller that re-armed the
    // machine would walk it straight back into the hang the quarantine exists to
    // prevent.  Same defect the GNSS machine had.
    if (state.stage == BNO055InitStage::Quarantined) return false;

    // IMUPLUS or AMG only.  NDOF is rejected rather than merely undocumented,
    // because enabling the magnetometer is not additive: it feeds the
    // orientation quaternion, and linear acceleration — the signal incident
    // detection reads — is derived from that quaternion.  A car is a steel shell
    // full of speakers, motors and a harness carrying tens of amps, so magnetic
    // distortion would contaminate exactly the channel that must stay clean.
    if (opMode != OPERATION_MODE_IMUPLUS && opMode != OPERATION_MODE_AMG) return false;

    state.opMode      = opMode;
    state.stage       = BNO055InitStage::WaitBoot;
    state.lastStatus  = BNO055InitStatus::OK;
    state.nextStepMs  = millis();
    state.stepRetries = 0u;

    // Cleared per ATTEMPT, not per boot.  Leaving them set was a real defect and
    // not a cosmetic one: fallBackToInternalClock() refuses to act when
    // clockFallback is already true, so the SECOND and every later attempt
    // skipped the fallback entirely while the log still reported one had
    // happened.  Nine bring-ups appeared to have tried both clock sources when
    // only the first had tried either.  A latch that outlives the thing it
    // describes stops being a record and becomes a lie.
    state.clockFallback   = false;
    state.externalCrystal = false;
    return true;
}

void bno055InitSetCalibProfile(BNO055InitState &state, const uint8_t *profile)
{
    state.calibProfile = profile;
}

void bno055InitFail(BNO055InitState &state, BNO055InitStatus why)
{
    if (state.stage == BNO055InitStage::Quarantined) return;

    // Record WHICH step refused before overwriting the stage.  Without it the
    // log can only say "failed", and the two most common failures want opposite
    // repairs: not-found at FindDevice is a part or a cable, while mode-failed
    // at VerifyOpMode is a part that is right there and would not switch.
    if (state.stage != BNO055InitStage::Failed) state.failedAt = state.stage;

    state.stage      = BNO055InitStage::Failed;
    state.lastStatus = why;
    if (state.failures < 0xFFFFu) state.failures++;

    state.nextStepMs = millis() + ((state.failures <= BNO055_INIT_FAST_RETRIES)
                                       ? BNO055_INIT_FAST_RETRY_MS
                                       : IMU_RETRY_MS);
}

void bno055InitQuarantine(BNO055InitState &state, BNO055InitStatus why)
{
    state.stage      = BNO055InitStage::Quarantined;
    state.failedAt   = BNO055InitStage::FindDevice;
    state.lastStatus = why;
}

bool bno055InitReady(const BNO055InitState &state)
{
    return state.stage == BNO055InitStage::Configured;
}

/**
 * @brief Retries the current stage after @p delayMs, up to @p maxRetries times.
 *
 * @return true when a retry was scheduled and the caller should stay put.
 */
static bool retryStage(BNO055InitState &state, uint32_t delayMs, uint8_t maxRetries)
{
    if (state.stepRetries >= maxRetries) return false;
    state.stepRetries++;
    state.nextStepMs = millis() + delayMs;
    return true;
}

// ─── the machine ──────────────────────────────────────────────────────────────

BNO055InitStage bno055InitTick(BNO055InitState &state)
{
    if (state.stage == BNO055InitStage::Configured)  return state.stage;

    // Terminal, checked FIRST and with no timer.  This is the state that breaks
    // a reboot loop, and it can only do that by never leaving.
    if (state.stage == BNO055InitStage::Quarantined) return state.stage;

    // Failed is a WAITING state holding the backoff; it re-enters by itself.
    if (state.stage == BNO055InitStage::Failed || state.stage == BNO055InitStage::Idle){
        if ((int32_t)(millis() - state.nextStepMs) < 0) return state.stage;
        bno055InitBegin(state, state.opMode);
        return state.stage;
    }

    // Every wait in this sequence lands here — the 400 ms power-on hold, the
    // 650 ms reset, the 20 ms mode switch.  Expressing them as deadlines rather
    // than as blocking stages is the whole reason this machine exists, and it
    // also keeps the bus untouched while the part is rebooting: a CHIP_ID read
    // issued during the reset window NACKs, and counting that as a fault would
    // make a healthy part look intermittent.
    if ((int32_t)(millis() - state.nextStepMs) < 0) return state.stage;

    // Per STEP, not once per bring-up.  A slave can wedge the lines between two
    // steps as easily as before the first one.
    if (i2cBusBegin() != I2CBusState::Ready){
        bno055InitFail(state, BNO055InitStatus::NOK_BUS_STUCK);
        return state.stage;
    }

    const BNO055InitStage entry = state.stage;

    switch (state.stage){

        case BNO055InitStage::WaitBoot:
            // Measured from millis() zero, not from entry to this stage, because
            // it is the PART's power-on time and the part powered up with the
            // board.  By the time the sketch has opened Serial this has usually
            // already elapsed; on a retry it always has.
            if (millis() < BNO055_POR_TO_CONFIG_MS){
                state.nextStepMs = BNO055_POR_TO_CONFIG_MS;
                break;
            }
            state.stage = BNO055InitStage::FindDevice;
            break;

        case BNO055InitStage::FindDevice: {
            const uint8_t addr = bno055FindAddress();
            if (addr == 0u){
                bno055InitFail(state, BNO055InitStatus::NOK_NOT_FOUND);
                break;
            }
            state.address = addr;
            bno055TransportBind(state.dev, addr);

            // Return deliberately discarded: it reports only the LAST of the
            // several reads this makes, so it cannot distinguish a part that
            // answered everything from one that answered nothing but the final
            // register.  chip_id is the honest check, and it is checked below.
            // The call is still needed — it installs the driver's context
            // pointer, which every driver call after this dereferences.
            (void)bno055_init(&state.dev);

            if (state.dev.chip_id != BNO055_EXPECTED_CHIP_ID){
                bno055InitFail(state, BNO055InitStatus::NOK_NOT_FOUND);
                break;
            }
            state.stage = BNO055InitStage::SetPageId;
            break;
        }

        case BNO055InitStage::Reset:
            // Reset unconditionally, even on a first boot when the part is
            // already in CONFIG.  It costs 650 ms once and buys a known starting
            // state: without it, bring-up after a watchdog reset inherits
            // whatever mode and units the previous run left behind, and the
            // first drive of the day would be configured differently from every
            // one after it.  Phase 6's calibration restore also requires it.
            //
            // The return is ignored ON PURPOSE.  The part may reset before it
            // ACKs, so a failed write here is an expected outcome of a
            // successful command.  Reacquire is what proves the reset worked,
            // and it is the only thing that can.
            (void)regWrite8(state.address, BNO055_SYS_TRIGGER_ADDR, BNO055_SYS_TRIGGER_RST_SYS);
            bno055TransportResetFaults();

            state.nextStepMs = millis() + BNO055_RESET_TO_CONFIG_MS;
            state.stage      = BNO055InitStage::Reacquire;
            break;

        case BNO055InitStage::Reacquire: {
            uint8_t id = 0u;
            if (!regRead8(state.address, BNO055_CHIP_ID_ADDR, id) ||
                id != BNO055_EXPECTED_CHIP_ID){
                // A few in-stage retries before declaring it lost.  650 ms is
                // the datasheet figure and parts vary; failing outright would
                // restart the whole sequence — including another reset and
                // another 650 ms — to recover from being 10 ms early.
                if (retryStage(state, 50UL, 4u)) break;
                bno055InitFail(state, BNO055InitStatus::NOK_LOST_AFTER_RESET);
                break;
            }
            state.stage = BNO055InitStage::ProbeWrite;
            break;
        }

        case BNO055InitStage::SetPageId:
            // BEFORE the reset, and that ordering is the whole fix.
            //
            // The BNO055 has two register pages, and PAGE_ID is the only
            // register reachable from both. A part left on page 1 by whatever
            // ran last — an example sketch, a previous firmware — answers every
            // access at the page-1 address instead, which is reserved space for
            // most of what bring-up touches. So reads return plausible-looking
            // constants and writes are acknowledged and discarded.
            //
            // Including the reset. RST_SYS lives at 0x3F, and on page 1 that is
            // reserved, so the reset command itself was being thrown away and
            // the part stayed on page 1 across every retry. Thirty attempts, all
            // identical, none of which had reset anything.
            //
            // Every symptom chased for three bench runs was this: UNIT_SEL
            // reading a fixed 0x80, OPR_MODE reading 0x10 with a "reserved bit"
            // set, writes ACKed and ignored, and a suspicion of counterfeit
            // silicon. One register write, in the right place, and all of it
            // went away.
            if (!configWrite(state, BNO055_PAGE_ID_ADDR, 0x00u, 0xFFu)){
                if (retryStage(state, 5UL, 2u)) break;
                captureDiagnostics(state);
                bno055InitFail(state, BNO055InitStatus::NOK_CONFIG_FAILED);
                break;
            }
            state.stage = BNO055InitStage::Reset;
            break;

        case BNO055InitStage::ProbeWrite:
            // Writes GYR_UNIT (bit 1, rad/s) — a bit that is 0 after reset, so
            // its appearance can only mean the part acted on the write. The
            // value is put back at SetUnits, which verifies bit 1 has returned
            // to 0, closing the round trip.
            //
            // Deliberately the FIRST thing tried after the reset. Everything
            // downstream — units, power mode, operating mode — is meaningless on
            // a part that discards writes, and finding that out at the operating
            // mode reports it as "the mode would not take", which sends the
            // repair after the mode.
            if (!configWrite(state, BNO055_UNIT_SEL_ADDR,
                             BNO055_UNIT_SEL_PROBE, BNO055_UNIT_SEL_PROBE)){
                if (retryStage(state, 5UL, 2u)) break;
                captureDiagnostics(state);
                bno055InitFail(state, BNO055InitStatus::NOK_WRITES_IGNORED);
                break;
            }
            state.stage = BNO055InitStage::SetPowerMode;
            break;

        case BNO055InitStage::ClearSysTrigger:
            // Explicitly clears CLK_SEL and the self-test bit, so the operating
            // mode is entered from a defined trigger state rather than whatever
            // the previous run or the reset left behind.
            if (!regWrite8(state.address, BNO055_SYS_TRIGGER_ADDR, 0x00u)){
                bno055InitFail(state, BNO055InitStatus::NOK_CONFIG_FAILED);
                break;
            }
            state.nextStepMs = millis() + BNO055_REG_SETTLE_MS;
            state.stage      = BNO055InitStage::SetUnits;
            break;

        case BNO055InitStage::RestoreCalib:
            state.calibOffered  = (state.calibProfile != nullptr);
            state.calibRestored = false;
            if (state.calibOffered) {
                // NOT a failure when it does not take. The sensor is entirely
                // usable with default offsets — it simply has to earn its
                // calibration again over the drive — so refusing to bring up an
                // IMU because a stored profile would not write would trade a
                // working sensor for a warm-up. The flag records it instead.
                state.calibRestored = bno055CalibWrite(state, state.calibProfile);
            }
            state.stage = BNO055InitStage::SetHighG;
            break;

        case BNO055InitStage::SetHighG: {
            // Everything here is on PAGE 1, and the page is put back before the
            // stage ends whatever happens. A failure that returned while still
            // on page 1 would leave every later read hitting reserved space and
            // returning plausible constants — the exact fault that made thirty
            // bring-ups look like counterfeit silicon.
            bool ok = configWrite(state, BNO055_PAGE_ID_ADDR, 0x01u, 0xFFu);

            if (ok) ok = configWrite(state, BNO055_P1_ACC_HG_THRES_ADDR,
                                     IMU_HIGHG_THRESHOLD_LSB, 0xFFu);
            if (ok) ok = configWrite(state, BNO055_P1_ACC_HG_DUR_ADDR,
                                     IMU_HIGHG_DURATION_LSB, 0xFFu);
            // Read-modify-write: ACC_INT_Settings also holds the any-motion and
            // no-motion axis enables, which are not ours to clear.
            if (ok){
                uint8_t axes = 0u;
                ok = regRead8(state.address, BNO055_P1_ACC_INT_SET_ADDR, axes);
                if (ok) ok = configWrite(state, BNO055_P1_ACC_INT_SET_ADDR,
                                         (uint8_t)(axes | BNO055_ACC_INT_HG_ALL_AXES),
                                         BNO055_ACC_INT_HG_ALL_AXES);
            }
            // INT_EN evaluates the condition; INT_MSK also drives the physical
            // pin. Both are set, and the difference is why the wire is optional:
            // with INT_EN alone the latch still appears in INT_STA over I2C, so
            // the feature works unwired and the pin only removes poll latency.
            if (ok){
                uint8_t en = 0u;
                ok = regRead8(state.address, BNO055_P1_INT_EN_ADDR, en);
                if (ok) ok = configWrite(state, BNO055_P1_INT_EN_ADDR,
                                         (uint8_t)(en | BNO055_INT_BIT_ACC_HIGH_G),
                                         BNO055_INT_BIT_ACC_HIGH_G);
            }
            if (ok){
                uint8_t msk = 0u;
                ok = regRead8(state.address, BNO055_P1_INT_MSK_ADDR, msk);
                if (ok) ok = configWrite(state, BNO055_P1_INT_MSK_ADDR,
                                         (uint8_t)(msk | BNO055_INT_BIT_ACC_HIGH_G),
                                         BNO055_INT_BIT_ACC_HIGH_G);
            }

            // UNCONDITIONAL, and its result outranks everything above.
            const bool backToPage0 = configWrite(state, BNO055_PAGE_ID_ADDR, 0x00u, 0xFFu);
            if (!backToPage0){
                captureDiagnostics(state);
                bno055InitFail(state, BNO055InitStatus::NOK_CONFIG_FAILED);
                break;
            }

            if (!ok){
                if (retryStage(state, 5UL, 2u)) break;
                captureDiagnostics(state);
                // NOT fatal. The part is fully usable without the hardware
                // backstop — it simply loses the ability to catch an impact
                // that lands inside a stalled loop. Refusing to run the IMU at
                // all over a missing interrupt would trade a working sensor for
                // a missing safety net, which is the wrong way round.
                state.highGArmed = false;
            } else {
                state.highGArmed = true;
            }
            state.stage = BNO055InitStage::SetClock;
            break;
        }

        case BNO055InitStage::SetClock:
#if BNO055_USE_EXTERNAL_CRYSTAL
            if (!regWrite8(state.address, BNO055_SYS_TRIGGER_ADDR, BNO055_SYS_TRIGGER_CLK_SEL)){
                bno055InitFail(state, BNO055InitStatus::NOK_CONFIG_FAILED);
                break;
            }
            // Not verified by read-back: SYS_TRIGGER's other bits are
            // self-clearing commands, so what comes back is not what went in.
            // CheckClock asks the part how it is instead, which is the better
            // question anyway — CLK_SEL being set proves the write landed, not
            // that a crystal exists to run from.
            state.nextStepMs = millis() + BNO055_CLOCK_SETTLE_MS;
            state.stage      = BNO055InitStage::CheckClock;
#else
            state.externalCrystal = false;
            state.stage           = BNO055InitStage::SetOpMode;
#endif
            break;

        case BNO055InitStage::CheckClock: {
            uint8_t err = 0u, stat = 0u;
            const bool read = regRead8(state.address, BNO055_SYS_ERR_ADDR, err) &&
                              regRead8(state.address, BNO055_SYS_STATUS_ADDR, stat);
            state.sysError  = err;
            state.sysStatus = stat;

            // ADVISORY ONLY, and that is a correction rather than a caveat.
            // This check was originally the whole crystal test, and it let a
            // broken configuration through twelve times: a part whose selected
            // clock is not running is in no position to report a system error
            // about it, so SYS_ERR came back zero and bring-up walked on into a
            // device that would answer reads and refuse every write.  What
            // settles it is whether a configuration write sticks, which is
            // SetUnits' job, and the fallback lives there.
            if (read && err == 0u && stat != BNO055_SYS_STATUS_ERROR){
                state.externalCrystal = true;
                state.stage           = BNO055InitStage::SetOpMode;
                break;
            }

            // No crystal fitted, or one that will not start.  Fall back instead
            // of failing: the internal oscillator runs fusion perfectly well,
            // just with more drift, and a dashcam that records nothing is worse
            // than one whose relative yaw is a little loose.  The fallback is
            // RECORDED, not absorbed — clockFallback is what stops "configured"
            // from meaning two different things on two boards.
            (void)regWrite8(state.address, BNO055_SYS_TRIGGER_ADDR, 0x00u);
            state.externalCrystal = false;
            state.clockFallback   = true;
            state.nextStepMs      = millis() + BNO055_CLOCK_SETTLE_MS;
            state.stage           = BNO055InitStage::SetOpMode;
            break;
        }

        case BNO055InitStage::SetUnits:
            // The FIRST configuration write of the sequence, and therefore the
            // first thing that proves the part will accept one.  Everything
            // before this point is reads, a reset it need not acknowledge, and a
            // clock-source write whose effect nothing had yet tested — so a part
            // that has been left unable to configure itself gets this far
            // looking healthy.  That is exactly what happened.
            if (configWrite(state, BNO055_UNIT_SEL_ADDR,
                            BNO055_UNIT_SEL_VALUE, BNO055_UNIT_SEL_MASK)){
                state.unitSelSeen = state.lastRegRead;
                state.stage       = BNO055InitStage::RestoreCalib;
                break;
            }
            if (retryStage(state, 5UL, 2u)) break;
            // A crystal that is not fitted, or will not start, leaves the part
            // answering reads and refusing configuration.  Try the internal
            // oscillator before declaring the part faulty.
            if (fallBackToInternalClock(state)) break;

            captureDiagnostics(state);
            bno055InitFail(state, BNO055InitStatus::NOK_CONFIG_FAILED);
            break;

        case BNO055InitStage::SetPowerMode:
            // Normal, not low power.  Low power would be the obvious choice for
            // a parked vehicle, but the datasheet takes High-G away in that mode
            // — leaving only No-motion and Any-motion — so the hardware impact
            // backstop disappears exactly when a car-park bump is the thing
            // being watched for.  Parked behaviour is a later, deliberate
            // change, not something bring-up should decide.
            if (!configWrite(state, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL, 0x03u)){
                if (retryStage(state, 5UL, 2u)) break;
                captureDiagnostics(state);
                bno055InitFail(state, BNO055InitStatus::NOK_CONFIG_FAILED);
                break;
            }
            state.stage = BNO055InitStage::ClearSysTrigger;
            break;

        case BNO055InitStage::SetOpMode:
            // Written directly, and NOT through bno055_set_operation_mode().
            // That function does not wait for the switch it commands — the
            // driver never calls its own delay hook — so it returns while the
            // part is still changing mode.  Here the wait is the stage boundary
            // below, and VerifyOpMode confirms it afterwards.
            if (!regWrite8(state.address, BNO055_OPR_MODE_ADDR, state.opMode)){
                bno055InitFail(state, BNO055InitStatus::NOK_MODE_FAILED);
                break;
            }
            state.nextStepMs = millis() + BNO055_MODE_SWITCH_MS;
            state.stage      = BNO055InitStage::VerifyOpMode;
            break;

        case BNO055InitStage::VerifyOpMode: {
            uint8_t mode = 0u;
            if (!regRead8(state.address, BNO055_OPR_MODE_ADDR, mode) ||
                (uint8_t)(mode & BNO055_OPR_MODE_MASK) != state.opMode){
                // Record THIS register, not whatever configWrite() last touched.
                // Without it the failure log named PWR_MODE — the previous
                // write — while reporting a mode failure, which is a diagnostic
                // pointing at the wrong register.
                state.lastRegAddr    = BNO055_OPR_MODE_ADDR;
                state.lastRegWrote   = state.opMode;
                state.lastRegRead    = mode;
                state.lastRegWriteOk = true;
                state.lastRegReadOk  = true;
                captureDiagnostics(state);
                // THIS is where a dead clock shows itself, and it is why the
                // crystal fallback lives here rather than at SetUnits.  Every
                // earlier write is one whose effect cannot be distinguished
                // from the part's own power-on state — writing CONFIG's own
                // value into a register that already holds it proves nothing.
                // This one commands a change, from CONFIG to IMUPLUS, that
                // either appears in the register or does not.
                if (fallBackToInternalClock(state)) break;
                bno055InitFail(state, BNO055InitStatus::NOK_MODE_FAILED);
                break;
            }

            uint8_t err = 0u, stat = 0u;
            if (!regRead8(state.address, BNO055_SYS_ERR_ADDR, err) ||
                !regRead8(state.address, BNO055_SYS_STATUS_ADDR, stat)){
                bno055InitFail(state, BNO055InitStatus::NOK_CONFIG_FAILED);
                break;
            }
            state.sysError  = err;
            state.sysStatus = stat;

            // The part's own account of itself, and the reason Configured means
            // more here than the GNSS machine's Done.  A mode register that
            // reads back correctly only proves the write landed; SYS_STATUS is
            // the part saying the algorithm is actually running.
            if (err != 0u || stat == BNO055_SYS_STATUS_ERROR){
                bno055InitFail(state, BNO055InitStatus::NOK_SYSTEM_ERROR);
                break;
            }

            const uint8_t expected = (state.opMode == OPERATION_MODE_AMG)
                                         ? BNO055_SYS_STATUS_NO_FUSION_RUNNING
                                         : BNO055_SYS_STATUS_FUSION_RUNNING;
            if (stat != expected){
                // Still starting the algorithm, most likely.  Give it a few more
                // mode-switch intervals before calling it a failure.
                if (retryStage(state, BNO055_MODE_SWITCH_MS, 5u)) break;
                bno055InitFail(state, BNO055InitStatus::NOK_SYSTEM_ERROR);
                break;
            }

            state.stage          = BNO055InitStage::Configured;
            state.lastStatus     = BNO055InitStatus::OK;
            state.configuredAtMs = millis();
            // Cleared here, unlike the GNSS machine, and the difference is not
            // an inconsistency.  That one withholds the reset because reaching
            // Done says nothing about whether data follows, so clearing it would
            // let a receiver that configures and then streams nothing retry
            // forever at the fast rate.  This machine has already read
            // SYS_STATUS and been told the algorithm is running, so there is no
            // equivalent silent-success case to guard against.
            state.failures = 0u;
            break;
        }

        default:
            // Unreachable: Idle, Failed, Configured and Quarantined all return
            // above.  Restart rather than sit in a state with no handler.
            bno055InitFail(state, BNO055InitStatus::NOK_CONFIG_FAILED);
            break;
    }

    if (state.stage != entry) state.stepRetries = 0u;
    return state.stage;
}

bool bno055ReadCalibration(const BNO055InitState &state,
                           uint8_t &sys, uint8_t &gyro,
                           uint8_t &accel, uint8_t &mag)
{
    uint8_t raw = 0u;
    if (state.address == 0u) return false;
    if (!regRead8(state.address, BNO055_CALIB_STAT_ADDR, raw)) return false;

    mag   = (uint8_t)( raw       & 0x03u);
    accel = (uint8_t)((raw >> 2) & 0x03u);
    gyro  = (uint8_t)((raw >> 4) & 0x03u);
    sys   = (uint8_t)((raw >> 6) & 0x03u);
    return true;
}
