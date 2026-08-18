#ifndef DATA_DICTIONARY
#define DATA_DICTIONARY 1

#define SERVO_XAXIS_PIN                             20U
#define SERVO_YAXIS_PIN                             19U
#define SEGLED_ADDRESS                              112U

#define SERIAL_BAUDRATE                             115200U
#define STATUS_INDICATOR                            LED_BUILTIN

#define WIFI_SSID                                   "TP-Link_2890"
#define WIFI_PASS                                   "52000393"
#define WIFI_SSID_BACKUP_1                          "Pixel_5101"
#define WIFI_PASS_BACKUP_1                          "AndroidPixel6"
#define WIFI_SSID_BACKUP_2                          "401 402 403"
#define WIFI_PASS_BACKUP_2                          "0938690720kien"

#define GPS_DEFAULT_I2C_ADDRESS                      66U
#define GPS_REFRESH_RATE                             4U

/// How often the host reads the receiver (ms).
///
/// MUST be shorter than the navigation period (1000 / GPS_REFRESH_RATE), and
/// deliberately so.  Polling AT the production rate means two unsynchronised
/// clocks of the same frequency slowly beat against each other, so a poll
/// periodically lands just before the packet is ready.  A 25-minute soak at
/// 250 ms measured that directly: 10 of 6051 polls found nothing, arriving in
/// clusters minutes apart exactly as a beat predicts, and the poll that
/// followed each miss had to drain a backlogged DDC buffer — which is where the
/// worst-case read time of 24.8 ms came from, against 13.6 ms normally.
///
/// At 200 ms the poll interval is strictly shorter than the 250 ms production
/// interval, so at least one poll always falls between consecutive packets and
/// the beat cannot happen.  The cost is one extra empty check per second, and an
/// empty check is a few bytes of DDC byte-count registers, not a packet read.
/// Enforced by a static_assert in GPSFunctions.cpp.
#define GPS_POLL_MS                                  200UL
/// How long a fix may go unrefreshed before it stops being published (ms).
///
/// NOT the poll interval.  Polling at 5 Hz against a 4 Hz receiver means ~20 %
/// of polls legitimately find no new packet, so clearing the fix on every empty
/// poll makes COMM_FLAG_GPS_FIX flicker off a fifth of the time while the
/// receiver is perfectly healthy — a consumer gating recording or overlays on
/// that flag would see it stutter constantly.  Four missed navigation periods
/// is unambiguous; one is just the poll landing between packets.
#define GPS_FIX_MAX_AGE_MS                           1000UL

/// No PVT packet at all for this long means the receiver is gone, not quiet (ms).
///
/// Twelve missed navigation periods.  This is what makes a receiver that
/// configured successfully and then died — unplugged, browned out, reset —
/// re-enter the retry path instead of being trusted forever on the strength of
/// one successful bring-up.
#define GPS_MAX_SILENCE_MS                           3000UL

/// Retry interval after a failed GNSS bring-up (ms), mirroring OBD2_RETRY_MS.
///
/// initializeGPS_I2C() is a chain of UBX config commands that each wait for an
/// ACK, and it is measurably not reliable first time: on the bench it returned
/// NOK_SET_RATE_FAILED or NOK_CONFIG_FAILED on roughly a third of attempts,
/// while a bounded retry reached 8/8. Without a retry a single unlucky boot
/// leaves the receiver dead for the whole drive and looks exactly like "no GNSS
/// fitted", which is the one diagnosis that stops anyone looking further.
#define GPS_RETRY_MS                                 5000UL

/// Deadline for ONE UBX configuration exchange during bring-up (ms).
///
/// SparkFun's defaultMaxWait is 1100 ms, and initializeGPS_I2C() issues enough
/// exchanges that the default cannot fit under the watchdog.  Counted from the
/// library source: begin() polls isConnected() up to three times; setI2COutput,
/// setDynamicModel, setNavigationFrequency and setNavigationRate are each a poll
/// plus a set; setAutoPVTrate is one more.  That is 3 + 8 + 1 = 12 deadlines, so
/// a receiver that answers slowly but SUCCESSFULLY takes up to 12*1100 = 13.2 s
/// — over 1.5x the WATCHDOG_PERIOD_MS budget, on the success path.  A bring-up
/// that reboots the board is worse than one that fails, because the retry that
/// exists to handle failure never gets to run.
///
/// 250 ms is SparkFun's own documented floor for I2C ("A default of 250ms for
/// maxWait seems fine for I2C but is not enough for SerialUSB", their header) —
/// the DDC port ACKs in single-digit milliseconds at 100 kHz, so this is still
/// more than an order of magnitude of headroom.  It bounds the whole chain to
/// 12*250 = 3.0 s, and initializeGPS_I2C() feeds the watchdog between exchanges
/// so the longest un-fed span is begin()'s three probes at 750 ms.
#define GPS_CMD_TIMEOUT_MS                           250U

#define GPS_BAUDRATE_DEFAULT                         9600U
#define GPS_BAUDRATE_CUSTOM                          115200UL
#define USE_DEFAULT_LOCATION 1
#ifdef USE_HOCHIMINH    //10.81532915851147, 106.6573371901137
#define GPS_DEFAULT_POSITION                         "lat=10.81532915851147;lon=106.6573371901137;alt=10.000000;pacc=50000.000000"
#endif
#ifdef USE_VUNGTAU      //10.367506732991703, 107.08795322622392
#define GPS_DEFAULT_POSITION                         "lat=10.367506732991703;lon=107.08795322622392;alt=7.000000;pacc=50000.000000"
#endif
#ifdef USE_DEFAULT_LOCATION //10.81532915851147, 106.6573371901137, 100km radius
#define GPS_DEFAULT_POSITION                         "lat=10.81532915851147;lon=106.6573371901137;alt=10.000000;pacc=100000.000000"
#endif
#define LOCAL_TIMEZONE                               7

#define CAN_BAUDRATE_DEFAULT                         500000UL
/// MCP2515 crystal frequency — MUST match the physical module (16 MHz on the MKR CAN
/// shield; many bare red/blue MCP2515 boards use 8 MHz — set 8000000UL for those).
#define MCP2515_OSC_FREQ                             16000000UL
//SPI interface pins, avoid using these for other purposes
#define MCP2515_DEFAULT_CS_PIN                       3
#define MCP2515_DEFAULT_INT_PIN                      7

/// Chip-select pin for the external SD card module (SPI).
#define SD_CS_PIN                                    4U

// ─── Physical switch panel: 2x 74HC165 parallel-in shift registers ───────────
//
// DELIBERATELY OFF THE SPI BUS, on three bit-banged pins of its own.  The
// 74HC165's Q7 is push-pull and has no chip-select that releases it, so a
// register hung on D10/MISO would fight the MCP2515 and the SD card
// CONTINUOUSLY - taking out CAN and logging rather than merely failing to read
// switches.  A tri-state buffer would fix that at the cost of a package; three
// spare pins are cheaper, and buttons have never needed SPI's speed.  A whole
// 16-bit read costs about 80 us against a 20 ms poll interval.
//
// The parts are plain 74HC, NOT the HCT used for the LED panel's 74HCT595s.
// That distinction is the opposite of the one made there and for the opposite
// reason: the 595s run at 5 V driven by 3.3 V logic and need HCT's TTL
// threshold, whereas these run entirely on the MKR's own 3.3 V rail, where HC's
// VIH of 0.7 x VCC is 2.31 V.  74HCT is specified only from 4.5 V and would be
// the WRONG part here.

/// SH/LD (74HC165 pin 1), active low.  Common to both packages.
#define SWITCH_LOAD_PIN                              0U
/// Shift clock (pin 2).  Common to both packages.
#define SWITCH_CLK_PIN                               1U
/// Serial data in, from the LAST register's Q7 (pin 9).
#define SWITCH_DATA_PIN                              2U

/**
 * Packages fitted, 1 or 2.  Chain order is #A (first, holds the sentinels) then
 * #B, with #A's Q7 feeding #B's DS and #B's Q7 reaching @c SWITCH_DATA_PIN.
 *
 * The read is MSB-first from the far end, so #B's D7 arrives first and #A's D0
 * last.  Both chain lengths therefore land the sentinels in the same two bits,
 * and @c SwitchData::state is the same expression either way.
 */
#define SWITCH_REGISTERS                             2U
#define SWITCH_CHAIN_BITS                            (SWITCH_REGISTERS * 8U)

/**
 * The two inputs that are NOT switches: #A D0 (pin 11) hard-wired to GND and
 * #A D1 (pin 12) hard-wired to 3V3.
 *
 * A POSITIVE CONTROL IN COPPER, and the only reason this peripheral can report
 * its own absence.  Every other input is a switch that is legitimately open
 * most of the time, so an absent chain, a dead register and sixteen open
 * switches all read as 0xFFFF and are otherwise indistinguishable.  A fixed
 * pattern that every read must reproduce is what turns that into a testable
 * claim - the same reasoning behind COMM_FLAG_IMU_PRESENT and
 * COMM_FLAG_GPS_PRESENT, both added because idle hardware and absent hardware
 * looked identical on the wire.
 *
 * OPPOSITE POLARITIES ARE THE POINT.  They sit on the FIRST register in the
 * chain, so their arrival proves the cascade link as well as the reading - and
 * if that link breaks, #B's serial input floats.  A floating CMOS input settles
 * somewhere and stays there, so it can consistently satisfy one sentinel; it
 * cannot satisfy two that disagree.
 */
#define SWITCH_SENTINEL_BITS                         2U
/// Usable switch inputs: SW00 .. SW(SWITCH_COUNT-1).
#define SWITCH_COUNT                                 (SWITCH_CHAIN_BITS - SWITCH_SENTINEL_BITS)

/// Poll interval (ms).  50 Hz - the debounce window below is measured in polls.
#define SWITCH_POLL_MS                               20UL

/**
 * Agreeing samples before a change is committed: 4 polls = 80 ms.
 *
 * Longer than a button design would take, deliberately.  Every input here is a
 * LATCHING switch, so response latency is worth nothing - nobody flips a rocker
 * expecting a reply inside 100 ms - and the budget is better spent on noise
 * immunity, which matters because no RC filtering is fitted.  Sixteen
 * capacitors would be real board area for something four samples solve free.
 */
#define SWITCH_DEBOUNCE_SAMPLES                      4U

/// Consecutive sentinel failures before the chain is declared absent.
#define SWITCH_ABSENT_READS                          4U

/**
 * Identical good reads required to establish a baseline before @c present is
 * asserted, at boot and after every outage.
 *
 * Without it a single un-debounced sample became the published state - so one
 * read landing mid-throw was adopted as truth, then corrected 80 ms later by a
 * @c changed bit describing nothing an operator did.  The same qualification
 * runs after a chain outage, because a read arriving the instant a connector
 * reseats is exactly the read least worth trusting.
 */
#define SWITCH_QUALIFY_READS                         4U

/**
 * Chatter detector: more RAW input edges than this inside the sliding window
 * below, on one input, marks that input's position untrustworthy.
 *
 * ⚠️ COUNTED ON RAW EDGES, NOT ON COMMITTED STATE CHANGES.  Counting committed
 * changes - which is what this did at first - makes the detector blind to the
 * one thing it exists for: an input alternating every poll resets the debounce
 * integrator continuously, so it NEVER commits, so it would report zero
 * transitions and perfect health while flapping at 50 Hz.  Only slow chatter
 * got counted, and slow chatter is also what a person operating a switch
 * briskly looks like.  It was exactly the wrong way round.
 *
 * The threshold follows from that change.  At a 20 ms poll a clean throw is one
 * edge (contact bounce is mostly shorter than the interval and lands between
 * samples), so even brisk human use is tens of edges over ten seconds, while a
 * chattering contact approaches 500.  40 sits in a very wide gap.
 *
 * ⚠️ THIS IS THE ONLY WIRING FAULT THIS DESIGN CAN DETECT.  Because every input
 * is latching, a position held indefinitely is CORRECT behaviour, so there is
 * no stuck-input test to write.  A broken wire reads as "open" and a chassis
 * short reads as "closed", and neither is distinguishable from a legitimate
 * switch position without a different sense topology.  Both are accepted blind
 * spots, recorded here so they are not rediscovered later as a mystery.
 */
#define SWITCH_CHATTER_MAX                           40U
#define SWITCH_CHATTER_WINDOW_MS                     10000UL
/**
 * Half-window.  Two buckets are summed, so the window SLIDES in 5 s steps
 * rather than resetting on a boundary.
 *
 * A single fixed bucket cannot see a burst that straddles its edge - the
 * classic case being 25 edges either side of a reset, which is a badly
 * intermittent connector reported as two quiet windows.
 */
#define SWITCH_CHATTER_BUCKET_MS                     (SWITCH_CHATTER_WINDOW_MS / 2UL)

/**
 * Settling margin around the parallel-load pulse (us).
 *
 * NOT a datasheet requirement - the 74HC165 needs tens of nanoseconds and a
 * SAMD21 @c digitalWrite() already costs more than a microsecond.  It is margin
 * for the one place this design has real RC: a switch loom running to the dash,
 * pulled up through 10 kOhm against whatever capacitance the harness has.
 */
#define SWITCH_SETTLE_US                             1U

/// Below this ground speed the GNSS course over ground is noise, not travel:
/// a stationary receiver reports a heading that wanders the full circle.
/// Heading is published as NAN below this threshold (km/h).
#define HEADING_MIN_SPEED_KMH                        5.0f
/// Minimum resultant length (0-1) from @c CircularMovingAverage for the mean
/// heading to be trusted.  Below this the samples largely cancel, so the vector
/// mean is an arbitrary direction that merely looks confident.
#define HEADING_MIN_RESULTANT                        0.7f

/// Default GPS position floats — match the @c USE_DEFAULT_LOCATION string in @c GPS_DEFAULT_POSITION.
#define GPS_DEFAULT_LAT                              10.81532915851147f
#define GPS_DEFAULT_LON                              106.6573371901137f
#define GPS_DEFAULT_ALT                              10.0f
#define GPS_DEFAULT_PACC                             100000.0f

// ─── IMU: Adafruit LSM6DSOX + LIS3MDL 9-DoF breakout (I2C) ────────────────────
// Shares the Wire bus with the GNSS receiver.  Addresses are decimal to match
// GPS_DEFAULT_I2C_ADDRESS above; hex is given because every datasheet uses it.

/// LSM6DSOX accelerometer + gyroscope, 0x6A (breakout default, SDO/SA0 low).
/// 0x6B (107) if the address jumper is closed.
#define IMU_ACCEL_I2C_ADDRESS                        106U
/// LIS3MDL magnetometer, 0x1C (breakout default).  0x1E (30) with the jumper closed.
#define IMU_MAG_I2C_ADDRESS                          28U
/// Alternate addresses, selected by the breakout's address jumpers.
#define IMU_ACCEL_I2C_ADDRESS_ALT                    107U
#define IMU_MAG_I2C_ADDRESS_ALT                      30U

// ─── IMU: Bosch BNO055 9-DoF with on-chip fusion (I2C) ───────────────────────
// Replaces the LSM6DSOX + LIS3MDL breakout above, which latched up after a run
// of successful-but-corrupt readings.  The addresses above are still referenced
// by the bus-conflict scan and are removed with the rest of that driver.

/// BNO055 at 0x29, the DATASHEET DEFAULT (COM3 floating, as GY breakouts leave
/// it).  0x28 is the alternative — and is what the Bosch driver hardcodes, so
/// the two disagree and neither may be assumed.  Bring-up scans both by CHIP_ID.
#define BNO055_I2C_ADDRESS_DEFAULT                   41U
#define BNO055_I2C_ADDRESS_ALT                       40U

/// Power-on to the part answering in CONFIG mode (ms).  Datasheet rev 1.4.
#define BNO055_POR_TO_CONFIG_MS                      400UL

/// Reset command to the part answering again (ms).  Longer than power-on, and
/// the two are genuinely different numbers — using 400 here reads CHIP_ID into
/// a device that is still rebooting and calls a healthy part missing.
#define BNO055_RESET_TO_CONFIG_MS                    650UL

/// Settling time allowed after ANY operating-mode write (ms).
///
/// The datasheet gives two: 7 ms CONFIG->operation and 19 ms operation->CONFIG.
/// One constant covering the worse of the two removes the need for the caller to
/// know which direction it is going, which is the kind of bookkeeping that gets
/// it wrong once and then produces a mode change that silently did not happen.
///
/// 30 rather than 19, matching the drivers observed to work on this part. The
/// extra 11 ms is spent once per bring-up and buys margin on a figure the
/// datasheet gives as typical rather than maximum.
///
/// This wait is entirely the caller's job.  bno055_set_operation_mode() does NOT
/// wait — the vendored driver never calls its own delay hook, not once — so from
/// an operating mode it writes CONFIG and then immediately writes the target
/// mode from inside the 19 ms window.  See vendor/BNO055/PATCHES.md.
#define BNO055_MODE_SWITCH_MS                        30UL

/// Settling time after selecting the clock source (ms).
#define BNO055_CLOCK_SETTLE_MS                       20UL

/// Prefer the external 32.768 kHz crystal over the internal oscillator.
///
/// Bosch recommends it for fusion, and the reference design fits one — but this
/// is a generic GY breakout, and clones do omit it.  Selecting a crystal that is
/// not there leaves the part unable to run, so bring-up does not simply trust
/// this flag: it selects the crystal, then reads SYS_ERR, and falls back to the
/// internal oscillator if the part complains.  @c BNO055InitState::clockFallback
/// records which one is actually in use, because "configured" must not mean two
/// different things depending on hardware nobody checked.
///
/// OFF at present, and that is a diagnostic decision rather than a conclusion.
/// The base sequence does not yet configure this part at all, and selecting a
/// clock source is an unproven improvement layered on an unproven foundation —
/// it adds a variable to every failure while the failure is still being read.
/// Bench evidence so far says the crystal is NOT implicated: an attempt on the
/// internal oscillator failed identically. Turn this back on, and confirm the
/// fallback path still works, once bring-up reaches Configured.
#define BNO055_USE_EXTERNAL_CRYSTAL                  0

/// Bring-up attempts at the fast rate before backing off to @c IMU_RETRY_MS.
/// Fewer than the GNSS gets: that receiver is measurably flaky on first contact,
/// whereas a BNO055 that does not answer twice in a row is usually not fitted.
#define BNO055_INIT_FAST_RETRIES                     4U

/// Delay between fast bring-up retries (ms).
#define BNO055_INIT_FAST_RETRY_MS                    250UL

/// Die-temperature band a reading must fall in to be believed (degC).
///
/// The part's own rated operating range, so anything outside cannot be a
/// measurement whatever the vehicle is doing. Carried over from the LSM6DSOX
/// driver, where on a captured failure it flagged 100 % of the corrupt bursts —
/// 341 of 861 samples, all reporting 104 or 116 C on a 25 C bench — with zero
/// false positives across 520 good ones. The burst is one transaction, so a bad
/// temperature condemns the inertial bytes beside it.
#define IMU_TEMP_MIN_VALID_C                         (-40.0f)
#define IMU_TEMP_MAX_VALID_C                         (85.0f)

/// Band the fused GRAVITY VECTOR's magnitude must fall in to be believed (m/s2).
///
/// A check the previous hardware could not offer, and a stronger one than
/// temperature: the fusion algorithm constructs this vector, so its length is
/// near-constant at 9.81 whatever the vehicle is doing — cornering, braking and
/// potholes move its DIRECTION, not its size. A departure means the fusion
/// output is not to be trusted, however healthy the transaction looked.
///
/// Wide enough not to fire while the algorithm is still converging after
/// bring-up, which is a real state and not a fault.
#define IMU_GRAVITY_MIN_VALID_MS2                    (7.0f)
#define IMU_GRAVITY_MAX_VALID_MS2                    (12.0f)

/// Acceleration at which the part is at its range limit (m/s2).
///
/// Fusion modes lock the accelerometer at +/-4 g = 39.24 m/s2 (datasheet 3.5),
/// and this sits just under it. A peak at or above this is a FLOOR rather than
/// a measurement: a genuine 20 g collision reads as 4 g, understating the impact
/// fivefold, so @c IMUData::accelSaturated must accompany it.
///
/// PER MODE, because the rail is per mode. One constant was applied to both, so
/// AMG — which this project selects +/-16 g for, precisely so a severe impact can
/// be characterised rather than clipped — flagged every honest reading above 4 g
/// as saturated. That inverts the flag's meaning on the one mode that exists to
/// measure past 4 g: it says "this peak is a floor" about the only readings that
/// are not.
#define IMU_ACCEL_SATURATION_MS2_FUSION              (39.0f)   ///< +/-4 g rail = 39.24.
#define IMU_ACCEL_SATURATION_MS2_RAW                 (155.0f)  ///< +/-16 g rail = 156.9.

/// Acceleration beyond which a reading is IMPOSSIBLE, not merely clipped (m/s2).
///
/// The part cannot report past its configured rail, so anything beyond it did
/// not come from the accelerometer. Distinct from @c IMU_ACCEL_SATURATION_MS2,
/// and the distinction matters: saturation says "this reading is a floor",
/// while this says "this reading is not a reading". Conflating them let a value
/// of 196 m/s2 be published as a genuine 20 g impact carrying a may-be-clipped
/// note — a fabricated crash on the field that triggers incident capture, which
/// is the exact failure mode that retired the previous sensor.
///
/// The rail plus roughly 10 %, so genuine clipping at the limit still reports as
/// saturated instead of being discarded.
#define IMU_ACCEL_MAX_VALID_MS2_FUSION               (43.0f)   ///< +/-4 g rail = 39.24.
#define IMU_ACCEL_MAX_VALID_MS2_RAW                  (173.0f)  ///< +/-16 g rail = 156.9.

// ─── High-G interrupt: the hardware impact backstop ──────────────────────────
//
// The reason this exists is that the BNO055 has no FIFO. The previous part
// buffered 2.3 seconds of samples, so an impact survived a stalled loop for as
// long as the FIFO was deep enough. Polling cannot do that: a poll that does not
// happen is data that no longer exists.
//
// The High-G interrupt is strictly better than what it replaces. The part
// latches a threshold crossing IN HARDWARE and holds it until it is explicitly
// cleared, so an impact survives a stall of any length — not merely one shorter
// than the buffer.

/// GPIO the BNO055 INT pin is wired to.
///
/// D6, chosen because it is genuinely free: 3, 4, 7, 19 and 20 are allocated,
/// 8/9/10 are SPI, 11/12 are the I2C this part sits on, and 13/14 are Serial1.
///
/// THE WIRE IS OPTIONAL, and the design deliberately keeps it so. The latch
/// lives in the part's INT_STA register, which this driver reads over I2C in
/// the same burst as everything else — so High-G works with no wire at all,
/// just with poll latency instead of pin latency. The pin only makes the
/// detection immediate. An unconnected input is pulled down and reads low
/// forever, so an absent wire produces no false events.
#define IMU_HIGHG_INT_PIN                            6U

/// High-G threshold, as the register wants it.
///
/// At the ±4 g the fusion modes lock the accelerometer to, 1 LSB is 15.63 mg,
/// so 128 is very close to 2.0 g.
///
/// ⚠️ THE LSB SCALES WITH THE SELECTED RANGE, so this byte does NOT mean 2 g in
/// every mode. AMG selects ±16 g, where 1 LSB is 62.5 mg and 128 is 8 g — a
/// threshold that ignores the pothole strikes the 2 g figure was chosen to
/// catch. Production is fixed to IMUPLUS so the 2 g reading is the operative
/// one, but anything presenting AMG as supported has to program this per mode.
///
/// 2 g rather than something smaller because GRAVITY COUNTS. The interrupt
/// watches the raw accelerometer, in which the vertical axis reads a steady 1 g
/// while the vehicle does nothing at all — and this project's standing rule is
/// not to guess a mounting orientation, so the axis that carries gravity is
/// unknown and none can be excluded. A threshold below 1 g would therefore fire
/// continuously on a parked car. 2 g leaves a full 1 g of headroom above
/// gravity, which ordinary driving does not reach: hard braking is around
/// 0.8 g and a sharp pothole strike is well past 2 g.
#define IMU_HIGHG_THRESHOLD_LSB                      128U

/// High-G duration, as the register wants it: the event must persist for
/// (value + 1) × 2 ms.
///
/// 1 gives 4 ms, long enough to reject a single noisy conversion and far shorter
/// than the 10-50 ms a real impact pulse lasts.
#define IMU_HIGHG_DURATION_LSB                       1U

/// Shortest interval between two host-commanded IMU mode changes (ms).
///
/// Each change restarts the sensor's bring-up, so it costs about 700 ms in which
/// the IMU publishes nothing and the trailing peak window is discarded. Without
/// a floor, a host polling this command holds the part in permanent
/// re-initialisation and it never produces another sample — every request
/// individually reasonable, the aggregate a denial of the sensor.
///
/// 5 s is several times the cost of one switch, so a legitimate session-level
/// decision is never refused, while a loop is.
#define IMU_MODE_MIN_INTERVAL_MS                     5000UL

/// How long a latched High-G event is republished on the wire (ms).
///
/// The same reasoning as the data-gap flag: telemetry publishes at 10 Hz, so an
/// event flag lasting a single 10 ms poll would be missed by nine frames out of
/// ten — and the frames missing it are precisely the ones an incident detector
/// most needs to see.
#define IMU_HIGHG_HOLD_MS                            500UL

/// Missing inertial time that makes a peak window untrustworthy (ms).
///
/// Was "twice the poll interval", which at the old 20 Hz poll meant 100 ms and
/// was reasonable. At 100 Hz the same rule means 20 ms — and a single console
/// status line at 115200 baud takes about 30 ms of blocking Serial writes, so
/// the flag fired on every debug print. Caught on the bench: a poll counter
/// advancing 196 times in two seconds instead of 200, alongside a status line
/// with a character dropped out of the middle of it.
///
/// A flag that fires whenever the firmware talks about itself is worse than no
/// flag: it trains a reader to ignore the one field that says a peak is not a
/// peak over the window it claims.
///
/// 50 ms as an ABSOLUTE, not a multiple of the poll rate, so changing the poll
/// no longer silently changes what counts as a hole. Below this the hardware
/// High-G latch covers an impact that lands in the gap — which is exactly why
/// that interrupt exists — and above it the window genuinely under-reports.
#define IMU_GAP_MIN_MS                               50UL

/// Identifies WHICH physical sensor a stored calibration belongs to.
///
/// BUMP THIS WHENEVER THE BNO055 IS REPLACED, or delete bno055.cal from the
/// card. Either invalidates the stored profile.
///
/// It exists because the automatic check that used to be here did not work. The
/// file recorded CHIP_ID and refused a profile whose chip did not match — but
/// every BNO055 answers 0xA0, so the test passed for any BNO055 at all and a
/// replacement sensor silently inherited the previous unit's biases. The part
/// has no serial number to do better with, so the guard is handed to the only
/// participant who actually knows the sensor changed: the person who changed it.
///
/// Remounting the SAME sensor does not need a bump — these offsets are
/// properties of the silicon, not of the bracket.
#define BNO055_CALIB_INSTALL_ID                      0x0001u

/// Peak linear acceleration below which the vehicle counts as quiet enough to
/// spend 60 ms not watching (m/s2).
///
/// Gravity is already removed from that peak, so this is a direct statement that
/// nothing is happening: well under the ~1 m/s2 of ordinary acceleration and far
/// under a pothole. Used to gate the calibration capture, which is the only
/// operation that blinds BOTH the poll and the hardware High-G comparator at
/// once.
#define IMU_CALIB_SAVE_QUIET_MS2                     (0.5f)

/// Shortest interval between calibration saves to the card (ms).
///
/// Ten minutes. The save is cheap in bytes but not free in time — capturing the
/// offsets means dropping the sensor to CONFIG and back, roughly 60 ms with no
/// samples produced — and the card is flash with a finite erase budget. A
/// calibration that has genuinely improved is still there ten minutes later;
/// one that is oscillating between two states is not worth recording twice.
#define IMU_CALIB_SAVE_INTERVAL_MS                   600000UL

/// @c SYS_STATUS values this project acts on.  Datasheet section 4.3.58.
#define BNO055_SYS_STATUS_ERROR                      1U   ///< System error; read SYS_ERR.
#define BNO055_SYS_STATUS_FUSION_RUNNING             5U   ///< Fusion mode, running.
#define BNO055_SYS_STATUS_NO_FUSION_RUNNING          6U   ///< Non-fusion mode (AMG), running.

/// Watchdog period (ms), rounded up to the next supported WDT period.
///
/// Generous on purpose. It exists to end a hang inside the SAMD core's unbounded
/// I2C waits, not to police loop timing — a loop pass is under 30 ms even with
/// both sensors polling, so 8 s cannot fire on a healthy system. Tightening it
/// buys nothing and risks a reboot loop during a slow bring-up, which is a worse
/// failure than the hang it guards against.
#define WATCHDOG_PERIOD_MS                           8000UL

/// Minimum interval between automatic bus-recovery attempts while the bus is
/// stuck (ms).
///
/// "Stuck" is not a terminal state — a slave holding SDA may let go, and a
/// shorted line may be unshorted — so i2cBusBegin() has to keep trying or a
/// transient fault becomes permanent. It must NOT retry on every call, though:
/// probes and scans go through i2cBusBegin() too, and a 112-address sweep would
/// otherwise run recovery 112 times.
#define I2C_BUS_RECOVER_RETRY_MS                     250UL

/// Shared-bus clock — Fast-mode, raised from the 100 kHz SAMD21 default.
///
/// All three parts are rated for it: LSM6DSOX and LIS3MDL to 400 kHz, and the
/// u-blox DDC port likewise.  The reason to use it is the FIFO drain, which
/// moves 220.5 words/s of 7 bytes and made the bus the binding constraint at
/// 100 kHz rather than anything about the sensors.
///
/// Measured on the bench 2026-07-29, same firmware, only this constant changed:
///
///   clock    IMU worst poll   GNSS worst poll   IMU I/O errors
///   100 kHz     16233 us          14020 us        0 / 2199 polls
///   400 kHz      6057 us           6238 us        0 / 2359 polls
///
/// 2.7x on the IMU and 2.2x on GNSS — NOT the 4x the clock ratio suggests, and
/// worth knowing why: a sizeable part of each transaction is fixed cost that no
/// clock speed touches (per-transfer setup in the SAMD driver, the sensor's own
/// response latency, and for the FIFO path one addressing phase per 7-byte
/// word).  Only the bits on the wire got four times faster.
///
/// Zero I/O errors at either speed, GNSS packet rate 3.99/s against a target of
/// 4, and every address probe ACKed, so Fast-mode is comfortable on this
/// wiring — which is jumper leads to the IMU and an ESLOV cable to the receiver,
/// i.e. not a favourable case.
///
/// This is the ONE place the clock is set, via Wire.setClock() in
/// i2cBusRecover() after every Wire.begin().  Any library that calls
/// Wire.begin() behind our back silently reverts the bus to 100 kHz — the
/// Adafruit segment-LED backpack is one such (see SegmentLEDFunctions.cpp), and
/// it is not fitted on this build.
///
/// If a longer harness or weaker pull-ups ever make Fast-mode marginal, the
/// symptom is IMU ioerr climbing or GNSS going stale, and the fix is to put this
/// back to 100000UL — nothing else needs to change.
#define IMU_I2C_CLOCK_HZ                             400000UL

/// IMU poll cadence (10 ms = 100 Hz).
///
/// With the BNO055 the poll rate IS the sample rate. The LSM6DSOX this replaced
/// batched 220 samples/s into a 512-word FIFO for a 20 Hz drain to collect in
/// full, so every sample reached the peak ring however slowly the host polled.
/// The BNO055 has no FIFO at all, so a 20 Hz poll simply discards four of every
/// five samples the part produced — and the discarded ones are exactly where a
/// pothole or kerb strike lives, since those are 10-50 ms impulses.
///
/// 100 Hz matches the part's fusion output rate, so nothing is thrown away.
///
/// THE BUS COST WAS THE OPEN QUESTION, and it is answered rather than assumed.
/// One poll is a single 48-byte burst; at IMU_I2C_CLOCK_HZ that is about 1.2 ms
/// including addressing, so 100 Hz occupies roughly 12 % of the bus. The GNSS,
/// measured at the same clock, needs about 6.2 ms four times a second — another
/// 2.5 %. At the 100 kHz this driver was believed to be using, the same burst
/// would have taken 4.5 ms and 100 Hz would have wanted 45 % of a shared bus,
/// which is why the figure was worth checking before changing this constant.
#define IMU_POLL_MS                                  10UL

// The superseded 50 ms FIFO drain cadence stood here. It is DELETED rather than
// left as history, because a second #define of IMU_POLL_MS below the new one is
// not a comment — the last definition wins, so the 100 Hz poll above would have
// silently reverted to 20 Hz while every comment claimed otherwise. GCC said so
// ("IMU_POLL_MS redefined") and a cached build had been swallowing it.
//
// The reasoning it carried is preserved above, where it still applies: polling
// slower than the sensor samples throws away the 10-50 ms impulses that a
// pothole or kerb strike consists of. The LSM6DSOX solved that with a FIFO; the
// BNO055 has none, so the poll rate has to solve it directly.

/// Largest number of FIFO words read in one poll.
///
/// The drain has to be bounded — an unbounded "read until empty" is a loop whose
/// length is decided by a peripheral, which is exactly what mission-critical
/// rules forbid.  Steady state produces 104 + 104 + 12.5 = 220.5 words/s, so a
/// 20 Hz drain needs 11.03 words per poll; 16 leaves 45% headroom to catch up
/// after a slow pass without any single pass being able to run long.
///
/// The bound is also the latency budget: 16 words is 16 * 7 bytes plus per-word
/// addressing, about 15 ms at IMU_I2C_CLOCK_HZ, which has to stay comfortably
/// inside COMM_STREAM_INTERVAL_MS or the 10 Hz telemetry push starts to jitter.
#define IMU_FIFO_MAX_WORDS_PER_POLL                  16U

/// LSM6DSOX FIFO capacity in words, from the datasheet.  Used as a sanity bound
/// on the reported fill level: a count above this did not come from a healthy
/// part, and 0x3FF is what a failed read of both status bytes looks like.
#define IMU_FIFO_DEPTH_WORDS                         512U

// ─── vehicle power state (drives the IMU sample mode) ─────────────────────────────
//
// Low power is entered when the VEHICLE IS POWERED OFF, not when it merely
// happens to be stationary.  That distinction is the whole design, and it was
// reached the hard way: an earlier revision inferred "parked" from GNSS speed
// and inertial peaks, which meant deciding, per sample, whether a reading was
// noise or movement.  That is not solvable by amplitude.  Measured on a
// motionless bench, every threshold raised was exceeded again — gyro peaks of
// 5.88 then 15.08 dps, GNSS ground speed excursions of 6.7 then 9.3 then
// 10.9 km/h — because a noise floor has a tail and a long enough run finds it.
//
// Ignition state has no tail.  The ECU either answers or it does not.

/// CAN silence this long means the ignition is off, not merely a gap in traffic.
///
/// Comfortably longer than OBD2_RETRY_MS, so the seconds of silence around
/// cranking — where the ECU drops off the bus and the retry path is already
/// re-initialising — cannot be mistaken for a shutdown.  Erring long costs a
/// little sensor current after a genuine power-off; erring short would drop the
/// IMU into its coarse mode while the engine is being started, which is exactly
/// the moment before the vehicle moves.
#define VEHICLE_POWEROFF_CONFIRM_MS                  30000UL
/// Trailing window over which the peak acceleration and rate are held (ms).
///
/// Must exceed COMM_STREAM_INTERVAL_MS, or a transient could occur and decay
/// entirely between two telemetry frames and never be transmitted — which would
/// defeat the FIFO.  At 250 ms every peak appears in at least two consecutive
/// frames.
///
/// CORRECTION.  An earlier note here claimed the hold "only ever reports a
/// transient for longer than it lasted — never shorter, which is the direction
/// that would lose an impact."  That was wrong, and wrong in the dangerous
/// direction.  The old implementation kept ONE maximum and one timestamp, and on
/// expiry replaced it with whatever sample was current — so a hard hit at t=0
/// masked a medium hit at t=200 ms (the medium one never became the maximum),
/// and at t=250 ms the reading collapsed to idle even though the medium hit was
/// still inside the window.  It could and did under-report.
///
/// The bucket ring below is what actually delivers the guarantee the comment
/// claimed.  Expiry now removes only buckets that are genuinely too old.
#define IMU_PEAK_WINDOW_MS                           250UL

/// Buckets in the peak ring.
///
/// SEVEN, NOT SIX, AND THE EXTRA ONE IS NOT SPARE CAPACITY.  A bucket is cleared
/// when the ring head wraps back onto it, which is BUCKETS x BUCKET_MS after that
/// bucket STARTED — so a sample arriving near the end of its bucket is erased
/// almost one whole bucket earlier than one arriving at the start.  With six
/// buckets of 250/6 = 41 ms (integer division, so 246 ms of ring against a 250 ms
/// window) the worst case retained a peak for 205 ms and published it as a 250 ms
/// figure.  Under-reporting an impact, on the field incident detection grades
/// severity from.
///
/// The guarantee wanted is (BUCKETS - 1) x BUCKET_MS >= IMU_PEAK_WINDOW_MS, which
/// is what makes the ADVERTISED window the WORST case rather than the best.
#define IMU_PEAK_BUCKETS                             7U

/// Span of one peak bucket (ms).  Derived, never set independently.
///
/// Ceiling division over BUCKETS-1, for the reason above: truncation here is what
/// silently shortened the window, and (250 + 5) / 6 = 42 ms gives 6 x 42 = 252 ms
/// of guaranteed coverage for every sample whatever its phase within a bucket.
/// Still comfortably finer than the poll, so expiry granularity stays well below
/// the window it protects.  The cost is one extra float per channel.
#define IMU_PEAK_BUCKET_MS                           ((IMU_PEAK_WINDOW_MS + (IMU_PEAK_BUCKETS - 2U)) / (IMU_PEAK_BUCKETS - 1U))

/// How long a bucket stays eligible, measured from the bucket's START (ms).
///
/// The window filter used to compare @c now against the bucket start, which
/// retired a bucket whose NEWEST sample was still well inside the window — the
/// same under-reporting as above, arriving by a second route.  Comparing against
/// the bucket's END instead makes every sample live at least
/// @c IMU_PEAK_WINDOW_MS and at most one bucket longer.
///
/// That asymmetry is deliberate.  Over-retaining a peak reports a real impact for
/// 42 ms longer than it lasted; under-retaining discards one.  For a dashcam
/// those are not comparable errors.
#define IMU_PEAK_ELIGIBLE_MS                         (IMU_PEAK_WINDOW_MS + IMU_PEAK_BUCKET_MS)

/// Gyroscope calibration (0-3) required before the FUSED output is reported as
/// trustworthy rather than @c PARTIAL.
///
/// Cheap and load-bearing: it arrives within seconds of the vehicle standing
/// still, and relative yaw is worthless without it.
///
/// THE ACCELEROMETER FIGURE IS DELIBERATELY NOT A CRITERION, and that is a
/// correction made on measurement. It was one, set at 1 on the argument that a
/// board bolted into a bracket might never exceed that. The argument was right
/// about the ceiling and wrong about the floor: on the bench the figure fell to
/// 0 after 1400 polls and stayed there for 9000 more, so the IMU reported
/// PARTIAL continuously — a quality flag that never clears tells a consumer
/// exactly as little as one that is never set.
///
/// What settled it is that the gravity magnitude held 9.79-9.81 across every one
/// of those 9000 polls. That is a DIRECT physical test of whether the fusion
/// output is sane, it already gates every sample in getIMUData(), and it passed
/// while the calibration bits said not to trust anything. Bosch's figures are
/// the part's self-assessment; the gravity gate is a measurement.
///
/// All four figures still go to the consumer, so anything downstream that wants
/// to be stricter can be. And Phase 6's calibration persistence is the real fix
/// for an accelerometer figure sitting at 0 — restoring the stored offsets at
/// key-on is precisely what the datasheet says it is for.
///
/// System calibration is not a criterion either: it cannot rise above 0 without
/// the magnetometer, which fusion mode switches off on purpose.
#define IMU_CALIB_MIN_GYRO                           3U

/// FIFO fill above which the OLDEST buffered words are older than the freshness
/// contract, so draining them would publish stale samples stamped as current.
///
/// Derived, not chosen: the batched rates are 104 + 104 + 12.5 = 220.5 words/s,
/// so IMU_MAX_DATA_AGE_MS of production is 220.5 * 0.25 = ~55 words.  A backlog
/// beyond that means the head of the queue already violates the age guarantee
/// the data carries, whatever the drain does with it.
///
/// This matters because the drain reads the OLDEST words and stamps them with
/// the current time — the FIFO carries no per-word timestamp in this
/// configuration.  So a long stall must be reported as a data gap rather than
/// quietly flattened into "fresh" samples.  Anything that blocks the loop for
/// more than a quarter second can produce it.
#define IMU_FIFO_BACKLOG_WORDS                       55U

/// Readings older than this are published as NAN rather than as measurements.
/// Five missed 20 Hz polls — long enough to ride out a busy loop, short enough
/// that a frozen sensor cannot masquerade as a stationary vehicle.
#define IMU_MAX_DATA_AGE_MS                          250UL

/// Consecutive failed bus reads before a device is declared lost and the
/// caller is expected to re-initialise it.  One NACK is a glitch; five in a row
/// at 20 Hz is a quarter second of silence.
#define IMU_MAX_CONSECUTIVE_FAULTS                   5U

/// Retry interval after the IMU link is lost (ms), mirroring OBD2_RETRY_MS.
#define IMU_RETRY_MS                                 5000UL

/// How long a channel that should be converting may go without asserting its
/// data-ready bit before the device is declared faulty and reconfigured.
///
/// This catches the failure the freshness window alone cannot: a gyroscope that
/// silently stops while the accelerometer beside it keeps reporting. Data-age
/// checks would blank the gyro and leave it blank forever, because the device
/// still answers the bus and so never looks "lost". One second is 100 missed
/// samples at the 104 Hz output rate — unambiguous, not a scheduling hiccup.
#define IMU_MAX_CHANNEL_STALL_MS                     1000UL

/// ESP32-C3 telemetry link (MKR Zero @c Serial1 UART).
#define ESP32_UART_BAUD                              115200UL
/// Streaming push period — 100 ms = 10 Hz.
#define COMM_STREAM_INTERVAL_MS                      100UL
/// No command from the C3 for this long means the bridge is gone (ms).
///
/// The MKR is the responder on this hop, so inbound frames are the only proof
/// the bridge is attached: writing to an unplugged UART succeeds exactly like
/// writing to a healthy one, so without an arrival clock a severed link looks
/// identical to a working one. Generous against the bridge's own ~5 s
/// re-request cadence so a quiet-but-alive link is not called dead.
#define COMM_LINK_SILENT_MS                          8000UL

/// Max inbound command frames serviced per @c tickCommMaster() call (RX budget).
#define COMM_MAX_CMDS_PER_TICK                       8U
/// How long a latched @c CMD_GET_ONCE keeps retrying before it is abandoned (ms).
/// Past this the answer would be too old to be what the requester asked for.
#define COMM_ONCE_TIMEOUT_MS                         500UL

#endif