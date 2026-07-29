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

/// IMU FIFO drain cadence (50 ms = 20 Hz).
///
/// This is the rate the HOST empties the LSM6DSOX FIFO, not the rate the sensor
/// samples at.  Those were the same thing before the FIFO existed, and that was
/// the defect: reading the output registers at 20 Hz against a 104 Hz ODR threw
/// away four samples in five, so a pothole or kerb strike — a 10-50 ms impulse,
/// the exact event the +/-8 g range was chosen to capture — was usually gone
/// before the next poll looked.  Now every converted sample is buffered in the
/// part and collected here, and the peak across the whole window survives even
/// though telemetry still publishes at 10 Hz.
#define IMU_POLL_MS                                  50UL

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

/// Buckets in the peak ring.  Each covers IMU_PEAK_WINDOW_MS / IMU_PEAK_BUCKETS.
///
/// Six at a 250 ms window is ~42 ms per bucket, comfortably finer than the 50 ms
/// poll, so a single drain lands in one or two buckets and expiry granularity is
/// well below the window it is protecting.  The cost is 6 floats per channel.
#define IMU_PEAK_BUCKETS                             6U

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