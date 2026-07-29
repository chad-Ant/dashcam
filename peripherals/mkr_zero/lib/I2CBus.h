#ifndef I2C_BUS
#define I2C_BUS 1

#include <Arduino.h>
#include <Wire.h>

/**
 * @file I2CBus.h
 * @brief Single owner of the MKR Zero's shared I2C bus (SERCOM2, D11/D12).
 *
 * Three devices share this bus — the u-blox GNSS receiver at 0x42, the
 * LSM6DSOX at 0x6A and the LIS3MDL at 0x1C — and before this module each
 * client opened the bus for itself behind a bare @c i2cInitialized flag.  That
 * arrangement has no way to express the one ordering that actually matters:
 * WHOEVER touches the bus first must unwedge it first.  A boolean cannot,
 * because by the time it is consulted the caller is already about to transact,
 * and any probe used to detect a stuck bus can itself hang forever inside the
 * SAMD core's unbounded flag waits.
 *
 * So every entry point — GNSS bring-up, IMU bring-up, address probes, the
 * bring-up scan — goes through @c i2cBusBegin(), which is idempotent and
 * performs GPIO-level recovery exactly once, before the first transaction of
 * the session, whichever client happens to be first.
 *
 * NOT INTERRUPT SAFE.  There is one bus, one owner, and all callers run from
 * @c loop().  Nothing here may be called from an ISR.
 */

// The shared bus depends on a patched Wire: the stock SAMD 1.8.14
// TwoWire::requestFrom() reads an uninitialised `busOwner` on every
// single-byte transfer, which can drop the STOP or report zero bytes read for a
// transfer that succeeded.  Single-byte register reads (WHO_AM_I, STATUS) are
// the most common transaction on this bus, so building against the stock
// library is not an acceptable outcome to discover at runtime.
// See peripherals/mkr_zero/vendor/Wire/README.md.
// DASHCAM_WIRE_STOCK_CONTROL is the ONE sanctioned bypass, and exists solely so
// the stock-versus-vendored control experiment is reproducible from this tree
// instead of requiring someone to hand-edit guards (which is how a "temporary"
// edit gets committed). It is set only by
// helper_scripts/WireRegression/BuildAndUpload.cmd /stock, never by any
// production script, and the sketch that uses it prints STOCK CONTROL on every
// run so a stock binary can never be mistaken for a patched one.
#if !defined(DASHCAM_SAMD_WIRE_REQUESTFROM1_FIX) && !defined(DASHCAM_WIRE_STOCK_CONTROL)
#error "Stock SAMD Wire detected. Build with --library peripherals/mkr_zero/vendor/Wire (see vendor/Wire/README.md); the stock requestFrom() has undefined behaviour on 1-byte reads."
#endif

/** Maximum responders recorded by @c scanI2CBus(). */
#define I2C_SCAN_MAX_DEVICES 16U

/**
 * @brief Arms the SAMD21 watchdog, in early-warning-free normal mode.
 *
 * Containment, NOT the fix.  The SAMD core's I2C driver waits on its bus flags
 * in loops with no deadline (@c SERCOM::startTransmissionWIRE,
 * @c SERCOM::readDataWIRE), and those live in the core rather than in any
 * library, so no amount of vendoring reaches them.  @c i2cBusBegin() refuses to
 * transact on a bus whose lines are not idle, which closes the common cases —
 * but a device that grabs SDA in the microseconds after that check still hangs
 * the CPU, and only a watchdog ends that.
 *
 * Call BEFORE the first I2C transaction.  Every path that can block for longer
 * than the period must call @c watchdogFeed().
 *
 * @param[in] periodMs Approximate timeout; rounded up to the nearest supported
 *                     WDT period (8 s max on the 1024 Hz OSCULP32K clock).
 */
void watchdogArm(uint32_t periodMs);

/** @brief Feeds the watchdog. Safe to call when the watchdog was never armed. */
void watchdogFeed(void);

/** @brief True when the last reset was caused by the watchdog (forensics). */
bool watchdogCausedReset(void);

/**
 * @brief True for the WHOLE of a boot that follows a watchdog reset.
 *
 * The watchdog turns a hang into a reboot, which is containment — but on its own
 * it produces an INFINITE reboot loop, because the next boot runs the same code
 * in the same order and hangs in the same place.  Measured on the bench with a
 * deliberate hang: a reset every ~9 s, indefinitely.
 *
 * The escape is for a boot that follows a hang to come up WITHOUT the I2C
 * peripherals that are the likely cause — degraded, loudly, but alive,
 * streaming telemetry, and diagnosable.
 *
 * QUARANTINE MUST BE TERMINAL FOR THE BOOT.  Skipping one attempt is not enough:
 * anything that retries afterwards re-enters the same hang and the loop simply
 * resumes at the retry interval.  So callers must not merely defer the risky
 * work, they must abandon it until the next non-watchdog reset.  A transient
 * hang therefore costs one boot's sensors; a persistent one costs sensors until
 * the vehicle is power-cycled, which is the price of guaranteeing the board
 * always reaches @c loop() and keeps reporting.
 *
 * Blunter than it looks like it should be, deliberately.  Naming the exact step
 * that hung needs state that survives a reset, and this chip has none to offer:
 * the SAMD21 has no backup registers, and @c .noinit does not survive here (see
 * the implementation for the measured evidence).  @c RCAUSE is the one thing
 * that does persist, and it answers "did the last run hang?" — not "where".
 *
 * The definitive fix is bounding the SAMD core's undeadlined SERCOM waits, which
 * would remove the hang rather than contain it; that needs the core's SERCOM
 * vendored, not just Wire.
 *
 * Idempotent and NOT consumed: every caller sees the same answer all boot.
 */
bool bootAfterHang(void);

/** Lifecycle of the shared bus. */
enum class I2CBusState{
    Uninitialized = 0, ///< @c Wire has not been opened this session.
    Ready = 1,         ///< Bus opened, lines released, safe to transact.
    Stuck = 2,         ///< SDA and/or SCL still held low after recovery clocking.
};

/**
 * @brief Opens the bus, recovering it first, exactly once per session.
 *
 * The first call runs @c i2cBusRecover() BEFORE @c Wire.begin() so a slave left
 * mid-byte by an unclean reset is released before anything tries to talk to it.
 * Once the bus is @c Ready, later calls are free.
 *
 * A @c Stuck bus is retried, at most once per @c I2C_BUS_RECOVER_RETRY_MS.
 * Latching @c Stuck permanently would be the wrong call: the holder may let go,
 * or a shorted line may be unshorted, and refusing to look again would turn a
 * temporary fault into a node that never comes back.
 *
 * Safe to call from every client's initialiser; that is the intent.
 *
 * @return @c I2CBusState::Ready when the bus is usable, @c Stuck when the lines
 *         could not be freed (transacting anyway risks hanging in the SAMD
 *         driver's unbounded waits).
 */
I2CBusState i2cBusBegin(void);

/** @brief Current bus state without changing anything. */
I2CBusState i2cBusState(void);

/**
 * @brief Frees a bus wedged by a slave holding SDA low.
 *
 * A reset that lands mid-transaction leaves the slave part-way through a byte,
 * still driving SDA and waiting for clocks that will never come.  The recovery
 * is the standard one: release the SERCOM pins to GPIO, clock SCL up to nine
 * times so the slave can finish the byte and ACK it believes it is sending,
 * issue a STOP, then hand the pins back to the SERCOM.
 *
 * Harmless on a healthy bus — SDA already reads high, the clocking is skipped,
 * and a lone STOP is ignored by idle slaves.
 *
 * Call this directly only to force a mid-session recovery; normal clients
 * should use @c i2cBusBegin().
 *
 * @return @c Ready if both lines are released afterwards, otherwise @c Stuck.
 */
I2CBusState i2cBusRecover(void);

/**
 * @brief Probes one 7-bit address.
 *
 * @param[in] address  7-bit address; anything outside 0x08–0x77 is rejected
 *                     without touching the bus.
 * @return @c true if a device acknowledged.
 */
bool i2cProbeAddress(uint8_t address);

/**
 * @brief Lists every device currently answering on the bus.
 *
 * Bring-up only, never @c loop(): a full sweep is 112 transactions, roughly
 * 12 ms at 100 kHz, during which nothing else can use the bus.
 *
 * @param[out] found     Buffer receiving the 7-bit addresses that ACKed.
 * @param[in]  maxCount  Capacity of @p found; the scan keeps counting past it
 *                       but stops recording.
 * @return Number of responders found, which may exceed @p maxCount.
 */
uint8_t scanI2CBus(uint8_t *found, uint8_t maxCount);

#endif
