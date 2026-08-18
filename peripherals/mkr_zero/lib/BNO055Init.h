#ifndef BNO055_INIT_H
#define BNO055_INIT_H 1

/**
 * @file BNO055Init.h
 * @brief Staged, non-blocking bring-up for the BNO055.
 *
 * The part needs 400 ms from power-on, 650 ms from a reset, and up to 19 ms per
 * operating-mode change before it will answer.  Spending those inline is the
 * mistake @c gpsInitTick() exists to correct: a straight-line bring-up holds the
 * CPU across every one of those waits, so the CAN drain stops, the telemetry
 * push stops, and a bring-up that hangs does so at the same point on every boot
 * under the same armed watchdog — which is a reboot loop, not a fault.
 *
 * So the sequence is a state machine driven one step per @c loop() pass, exactly
 * like the GNSS one, and the waits are expressed as SCHEDULED TIME rather than
 * as stages that block.  A tick is one or two single-byte register transfers,
 * which at 100 kHz is a few hundred microseconds — over an order of magnitude
 * shorter than the GNSS bring-up's worst stage, because nothing here needs a
 * request/response exchange with a timeout.
 *
 * WHY THIS IS NOT THE DRIVER'S OWN SEQUENCE
 * -----------------------------------------
 * @c bno055_set_operation_mode() looks like it does this job.  It does not wait.
 * The vendored Bosch driver never calls its own @c delay_msec hook — not once in
 * 16 000 lines — so from an operating mode it writes CONFIG and then immediately
 * writes the target mode from inside the 19 ms window in which the part is still
 * switching.  The failure that produces is not an error return: it is a mode
 * change that silently did not take, or output registers read while the part is
 * still in the previous mode.  Every convenience setter that bounces through
 * CONFIG inherits it.
 *
 * This machine therefore owns every transition itself — write @c OPR_MODE, hold
 * the datasheet delay as a stage boundary, read the register back — and uses
 * none of the driver's mode-changing setters.  @c vendor/BNO055/PATCHES.md has
 * the details and the line numbers.
 *
 * THERE IS NO DRIVER
 * ------------------
 * A vendored Bosch reference driver used to sit under this, and this comment
 * used to say @c getIMUData() would eventually use its conversion functions.
 * It never did — the poll reads one 48-byte burst and decodes it directly — so
 * the whole 16 000-line library ended up being called for exactly one function,
 * @c bno055_init(), which is seven single-byte reads. It has been replaced by
 * @c bno055Identify() and the register map in @c BNO055Regs.h, which also
 * settles a GPLv3-against-MIT licence conflict that was being deferred rather
 * than resolved.
 *
 * @see peripherals/mkr_zero/lib/BNO055Transport.h  — the I2C hooks under this
 * @see peripherals/mkr_zero/vendor/BNO055/PATCHES.md
 */

#include <Arduino.h>

#include "BNO055Transport.h"
#include "DataDictionary.h"
#include "I2CBus.h"

/** Why a bring-up step refused.  Negative values are failures, as elsewhere. */
enum class BNO055InitStatus : int8_t{
    OK = 0,
    /// A held bus line made the step impossible.  Distinct from an absent part:
    /// the repair is the harness, not the sensor.
    NOK_BUS_STUCK = -1,
    /// Nothing returned CHIP_ID 0xA0 at either 0x28 or 0x29.
    NOK_NOT_FOUND = -2,
    /// The part answered before the reset and not after it.  Distinct from
    /// NOK_NOT_FOUND because it means the device IS fitted and IS wired: it
    /// answered moments ago.  Reporting that as "not found" sends the next
    /// person to check a cable that is already good.
    NOK_LOST_AFTER_RESET = -3,
    /// A configuration register did not read back as written.
    NOK_CONFIG_FAILED = -4,
    /// OPR_MODE did not hold the requested mode after the switching delay.
    NOK_MODE_FAILED = -5,
    /// The part reported a non-zero SYS_ERR, or SYS_STATUS showed a system
    /// error.  The part is present and talking and says it is unwell.
    NOK_SYSTEM_ERROR = -6,
    /**
     * The part acknowledges register writes on the bus and acts on none of them.
     *
     * Its own status code because it is a different fault from every other one
     * here, and points somewhere else entirely.  A part that answers reads
     * perfectly — 20 000 of them without a fault — while discarding every write
     * is not misconfigured, not absent, and not on a wedged bus.  Nothing in the
     * firmware can fix it.
     *
     * Proved by @c ProbeWrite rather than inferred, because inferring it is what
     * went wrong: bring-up reported "mode-failed" for thirty attempts, which
     * describes a mode that would not take and quietly implies the rest of the
     * sequence worked. It had not — every earlier write had been a write of a
     * value the register already held.
     */
    NOK_WRITES_IGNORED = -7,
};

/**
 * Steps of the staged bring-up, in execution order.
 *
 * Waiting is NOT a stage.  Every wait is a deadline in
 * @c BNO055InitState::nextStepMs, so the reset delay costs one comparison per
 * @c loop() pass rather than a stage that has to be re-entered to do nothing.
 */
enum class BNO055InitStage : uint8_t{
    Idle = 0,       ///< Not started.
    WaitBoot,       ///< Holding off until the part has had its power-on time.
    FindDevice,     ///< Scan 0x28/0x29 by CHIP_ID; bind the transport.
    Reset,          ///< Command a system reset, for a known starting state.
    Reacquire,      ///< CHIP_ID again — proves it came back from the reset.
    SetPageId,      ///< PAGE_ID: 0. Every register below lives on page 0.
    /**
     * Prove the part accepts a register write at all, before trusting any.
     *
     * Writes a bit that is DIFFERENT from what the register already holds and
     * checks it appears — then @c SetUnits puts it back and checks it clears,
     * so the round trip is 0 -> 1 -> 0 and neither half can pass by accident.
     *
     * This exists because its absence cost thirty bring-ups. Every configuration
     * write in the original sequence happened to write the value the register
     * already contained: @c UNIT_SEL 0x00 into a register reading 0x00 in the
     * bits being checked, @c PWR_MODE normal into a part already in normal. Both
     * "verified" perfectly on a device that was discarding every write, and the
     * failure surfaced at the first register whose value actually had to change.
     * A test that writes what is already there tests nothing.
     */
    ProbeWrite,
    SetPowerMode,   ///< PWR_MODE: normal.
    ClearSysTrigger,///< SYS_TRIGGER: 0, matching the sequence known to work.
    SetUnits,       ///< UNIT_SEL: m/s2, deg/s, degrees, Celsius.
    /**
     * Writes a stored calibration profile back into the offset registers.
     *
     * Skipped when no profile was supplied, which is the ordinary state of a rig
     * that has never been calibrated.
     *
     * HERE, and not later, because the offset registers are writable only in
     * CONFIG mode — the machine is already parked there, so this costs nothing,
     * whereas doing it after the mode switch would cost another round trip
     * through CONFIG and a gap in the data.
     */
    RestoreCalib,
    /**
     * Arms the High-G interrupt. Writes PAGE 1, then returns to page 0.
     *
     * The only stage that leaves page 0, and it must not be left there — every
     * data register this project reads lives on page 0, and a part stranded on
     * page 1 answers every transaction while returning and accepting nothing
     * meaningful. That is not hypothetical: it is what cost thirty bring-ups
     * before @c SetPageId existed.
     */
    SetHighG,
    SetClock,       ///< Select the external crystal, if enabled.
    CheckClock,     ///< Confirm the part accepted it; fall back if not.
    SetOpMode,      ///< OPR_MODE: the requested fusion or raw mode.
    VerifyOpMode,   ///< Read it back and check SYS_STATUS/SYS_ERR.
    /**
     * Running, in the requested mode, and it said so.
     *
     * Stronger than the GNSS machine's @c Done, deliberately.  That one means
     * only that the receiver ACKed its settings and says nothing about whether
     * data follows.  This one is reached only after @c OPR_MODE reads back as
     * requested and @c SYS_STATUS reports the algorithm running, so it is a
     * statement about the part rather than about the writes.
     */
    Configured,
    Failed,         ///< A step refused; holds the retry backoff — NOT terminal.
    /**
     * Abandoned for this boot because the previous run hung.  TERMINAL.
     *
     * Same reasoning as @c GPSInitStage::Quarantined: retrying after a hang
     * re-enters the SAMD core's unbounded I2C wait, the watchdog resets the
     * board, and the reboot loop resumes at the retry interval — slowed, not
     * broken.  Nothing leaves this state except a non-watchdog reset.
     */
    Quarantined,
};

/**
 * @brief Progress of one BNO055 bring-up.
 *
 * Plain aggregate: owns no memory, allocates nothing, safe to drive from
 * @c loop().
 *
 * It used to carry a static-storage-duration requirement, because the vendored
 * driver stored @c &dev in a file-static pointer and dereferenced it on every
 * later call — so a state on the stack left the driver holding a dangling
 * pointer the moment the caller returned. That driver is gone and nothing keeps
 * a pointer to this any more, so the requirement is gone with it. The production
 * instance is still at file scope for its own reasons.
 */
/**
 * @brief Everything worth knowing about a bring-up failure, LATCHED.
 *
 * WHY THIS EXISTS AS A SEPARATE RECORD. Every field it copies already lives in
 * @c BNO055InitState — and every one of them is destroyed by the retry.
 * @c bno055InitBegin() resets @c lastStatus, walks the stage machine from the
 * top, and re-runs the reads that overwrite @c sysStatus, @c sysError and the
 * register read-back; @c imuMarkAbsent() clears the fault counters. So by the
 * time anybody looks at a rig that has been retrying for a minute, the state
 * that would explain WHY has been overwritten by the attempts to fix it, and
 * what is left says only "not found" over and over.
 *
 * FIRST FAILURE WINS. Later failures are usually consequences of the first —
 * once the part is unreachable every subsequent attempt reports not-found — so
 * the record keeps the one that started it and does not let the noise after it
 * overwrite the signal. It is cleared only when a bring-up actually succeeds,
 * because at that point the evidence describes a fault that no longer exists.
 */
struct BNO055FailureRecord{
    bool     valid     = false;  ///< False when nothing has failed since the last success.
    uint32_t atMs      = 0;      ///< @c millis() of the FIRST failure in this run.
    uint16_t failureNo = 0;      ///< Which consecutive failure this was.

    BNO055InitStage  failedAt   = BNO055InitStage::Idle;  ///< Which step refused.
    BNO055InitStatus lastStatus = BNO055InitStatus::OK;   ///< Why it refused.

    uint8_t address = 0;   ///< Where it was being addressed. 0 = never identified.
    uint8_t opMode  = 0;   ///< Mode being configured when it failed.
    uint8_t chipId  = 0;   ///< What identification actually read back.
    uint8_t pageId  = 0;   ///< The register page identification saw.

    uint8_t sysStatus  = 0;  ///< The part's own account of itself...
    uint8_t sysError   = 0;  ///< ...and its error code, if either could be read.
    uint8_t opModeSeen = 0;  ///< OPR_MODE as last read.

    /// The last configuration write and its read-back — the three-way
    /// distinction between a NACKed write, a failed read-back and a write the
    /// part accepted and ignored.
    uint8_t lastRegAddr    = 0;
    uint8_t lastRegWrote   = 0;
    uint8_t lastRegRead    = 0;
    bool    lastRegWriteOk = false;
    bool    lastRegReadOk  = false;

    uint16_t transportFaults = 0;  ///< Consecutive transport faults at that moment.
    uint32_t transportErrors = 0;  ///< Cumulative since boot.
    /// @c I2C_STUCK_* mask from the last recovery attempt. The one field that
    /// says the fault was the shared bus rather than this part.
    uint8_t  stuckLines      = 0;
};

struct BNO055InitState{
    /// What the part reported at identification: address, chip and revision IDs.
    BNO055Device dev = {};

    BNO055InitStage  stage      = BNO055InitStage::Idle;
    BNO055InitStage  failedAt   = BNO055InitStage::Idle;  ///< Which step refused, for logs.
    BNO055InitStatus lastStatus = BNO055InitStatus::OK;   ///< Why it refused.

    uint32_t nextStepMs = 0;   ///< Earliest @c millis() for the next step.
    uint16_t failures   = 0;   ///< Consecutive failed bring-ups, for backoff.
    uint8_t  stepRetries = 0;  ///< In-stage retries used by the current step.

    /// Failed bring-ups since BOOT, never reset.
    ///
    /// @c failures above is the backoff counter and clears on every success, so
    /// it cannot answer "has this rig ever had trouble?" — and a part that fails
    /// and recovers repeatedly is a different diagnosis from one that has been
    /// solid since power-on. Distinguishing them needs a counter that success
    /// does not erase.
    uint16_t failuresTotal = 0;

    /// The first failure since the last successful bring-up, preserved from the
    /// retries that would otherwise overwrite it. See @c BNO055FailureRecord.
    BNO055FailureRecord firstFailure = {};

    uint8_t  address = 0;      ///< Where it actually answered.  0 until found.
    uint8_t  opMode  = OPERATION_MODE_IMUPLUS;  ///< Mode being configured.

    /// Last SYS_STATUS / SYS_ERR read.  Kept because they are the part's own
    /// account of itself and cost nothing to carry — and because a bring-up
    /// that failed at @c VerifyOpMode is unreadable without them.
    uint8_t  sysStatus = 0;
    uint8_t  sysError  = 0;
    uint8_t  opModeSeen = 0;   ///< OPR_MODE as last read, captured on failure.

    /// UNIT_SEL as the part actually reports it after configuration.
    ///
    /// Carried so Phase 3 decodes against what the part reports rather than
    /// what it was told — bit 7 selects the Euler orientation convention, and
    /// getting that wrong inverts pitch silently. It now verifies like every
    /// other bit; it appeared stuck at 1 only while the part was on register
    /// page 1 and 0x3B was not UNIT_SEL at all.
    uint8_t  unitSelSeen = 0;

    /// The last configuration write and its read-back.
    ///
    /// Added after a bring-up failed identically twelve times reporting only
    /// "config-failed", which does not distinguish the three things that can go
    /// wrong — a NACKed write, a failed read-back, or a write the part accepted
    /// and ignored. Those want three different repairs, and a status code that
    /// cannot tell them apart sends the next person guessing. A verification
    /// step that reports only pass/fail is half a diagnostic.
    uint8_t lastRegAddr    = 0;
    uint8_t lastRegWrote   = 0;
    uint8_t lastRegRead    = 0;
    bool    lastRegWriteOk = false;
    bool    lastRegReadOk  = false;

    /// The crystal was requested and the part would not run on it, so bring-up
    /// fell back to the internal oscillator.  Not a failure, but not the
    /// configuration that was asked for either, and fusion drifts more without
    /// it — so it is recorded rather than absorbed silently.
    bool clockFallback = false;
    /// True when the external crystal is the clock actually in use.
    bool externalCrystal = false;

    /**
     * Calibration offsets to write during bring-up, or nullptr for none.
     *
     * A POINTER SUPPLIED BY THE CALLER, so this machine never learns that a card
     * exists. Storage is the sketch's business — it already owns the SD mount,
     * and threading the filesystem into a state machine whose job is register
     * sequencing would make the IMU unbringable on a rig with no card.
     *
     * Must outlive the bring-up. Set it through @c bno055InitSetCalibProfile().
     */
    const uint8_t *calibProfile = nullptr;
    /// A supplied profile was written and read back. False when none was offered
    /// OR when the write failed — @c calibOffered separates those.
    bool calibRestored = false;
    bool calibOffered  = false;

    /// The High-G interrupt is configured and enabled.
    ///
    /// False is not a bring-up failure — the part is fully usable without it,
    /// having only lost the ability to catch an impact that lands inside a
    /// stalled loop. It is recorded because that is a real reduction in what the
    /// system can promise, and silently running without a safety net is how a
    /// safety net comes to be assumed.
    bool highGArmed = false;

    uint32_t configuredAtMs = 0;  ///< @c millis() when @c Configured was reached.
};

/**
 * @brief Advances the bring-up by ONE STEP. Call from @c loop().
 *
 * Each call performs at most a few single-byte register transfers, so unlike
 * @c gpsInitTick() — whose worst stage is three 250 ms probes — this one cannot
 * meaningfully delay the loop.  The long waits the part needs are held in
 * @c nextStepMs and cost one comparison.
 *
 * @c Failed is a WAITING state carrying the retry backoff, not a terminal one:
 * it re-enters the sequence by itself, so the caller never has to restart the
 * machine.  Only @c Quarantined is terminal.
 *
 * @param[in,out] state  Bring-up progress, static storage duration.
 * @return The stage AFTER this step.
 */
BNO055InitStage bno055InitTick(BNO055InitState &state);

/**
 * @brief (Re)starts the bring-up from the first step, immediately.
 *
 * @param[in,out] state   Bring-up progress.
 * @param[in]     opMode  @c OPERATION_MODE_IMUPLUS (fusion, no magnetometer) or
 *                        @c OPERATION_MODE_AMG (raw, no fusion).  Anything else
 *                        is rejected and the previous mode retained — NDOF in
 *                        particular is not a supported choice here, because the
 *                        magnetometer feeds the orientation quaternion that
 *                        linear acceleration is derived from, and a car is close
 *                        to a worst case for magnetometers.
 * @return false when @p opMode was rejected or the machine is quarantined.
 */
bool bno055InitBegin(BNO055InitState &state, uint8_t opMode);

/** @brief Forces the machine into its backoff state with a stated reason. */
void bno055InitFail(BNO055InitState &state, BNO055InitStatus why);

/**
 * @brief Offers calibration offsets for the next bring-up to restore.
 *
 * @param profile @c BNO055_CALIB_BYTES bytes that MUST outlive the bring-up, or
 *                nullptr to restore nothing. Not copied: the state struct is a
 *                plain aggregate that owns no memory, and a 22-byte buffer
 *                inside it would be 22 bytes carried by every rig including the
 *                ones with no card.
 */
void bno055InitSetCalibProfile(BNO055InitState &state, const uint8_t *profile);

/**
 * @brief Abandons the part for this boot, without any bus access. TERMINAL.
 *
 * For use after a watchdog reset, exactly as @c gpsInitQuarantine().
 */
void bno055InitQuarantine(BNO055InitState &state, BNO055InitStatus why);

/** @brief True once the part is configured and running in the requested mode. */
bool bno055InitReady(const BNO055InitState &state);

/** @brief Stage name for logs. */
const char *bno055InitStageName(BNO055InitStage stage);

/** @brief Status name for logs. */
const char *bno055InitStatusName(BNO055InitStatus status);

/**
 * @brief Reads the four 2-bit calibration counters. Cheap: one register.
 *
 * Meaningful only once @c bno055InitReady().  In IMUPLUS the magnetometer is
 * not used and its counter stays at zero, which is correct and not a fault.
 *
 * @param[in]  state  Configured bring-up state.
 * @param[out] sys    System calibration, 0-3.
 * @param[out] gyro   Gyroscope calibration, 0-3.
 * @param[out] accel  Accelerometer calibration, 0-3.
 * @param[out] mag    Magnetometer calibration, 0-3.
 * @return false when the read failed; the outputs are then untouched.
 */
bool bno055ReadCalibration(const BNO055InitState &state,
                           uint8_t &sys, uint8_t &gyro,
                           uint8_t &accel, uint8_t &mag);

#endif // BNO055_INIT_H
