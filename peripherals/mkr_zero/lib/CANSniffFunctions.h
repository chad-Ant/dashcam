#ifndef CAN_SNIFF_FUNCTIONS_H
#define CAN_SNIFF_FUNCTIONS_H 1

/**
 * @file CANSniffFunctions.h
 * @brief The MCP2515's receive path: the timer-driven drain, the listen-only
 *        modes, and the decoder the Orin's offline tool shares.
 *
 * ── RAW FIRST, DECODE ELSEWHERE ──────────────────────────────────────────────
 * The node boots into DISCOVER and stays there unless the Jetson says otherwise:
 * listen-only, accept-all, every frame read out of the controller by a timer
 * interrupt (@c canDrainIsr()), queued in a RAM ring (CANFrameRing.h) and
 * streamed to the Orin over the MKR's native USB (CANRawStream.h). Nothing is
 * decoded on this board in that mode, so the C3 telemetry carries no
 * CAN-derived values — they read as unavailable, never as zeros.
 *
 * Why: the 2026-10-03 night drive lost >= 13.4 % of frames even with the six-ID
 * hardware filter, because the drain ran once per loop() pass and the IMU, GNSS
 * and SD work block the pass. Decoding needs only the frames, so it moved to
 * the Orin, where a missed signal can be re-decoded from the recording; the
 * MKR's job shrank to not losing frames.
 *
 * SNIFF still exists, reachable only by an explicit CMD_SET_CAN_MODE: the map's
 * hardware filters, and decoding of the frames taken from the same ring. Both
 * SNIFF and the offline decoder go through @c canDecodeFrame(), so what the Orin
 * computes from a recording is what SNIFF would have computed live.
 *
 * The ID map is MEASURED on the target vehicle - a 2019 Honda Brio (DD1), CVT,
 * 500 kbps F-CAN tapped at OBD-II pins 6/14 - not inferred from a database.
 * Where it agrees with opendbc that is noted, but the vehicle is the authority:
 * the originally supplied table had four of seven rows wrong.
 *
 * ── SAFETY ───────────────────────────────────────────────────────────────────
 * DISCOVER and SNIFF run the MCP2515 in Listen-Only, where it emits neither ACK
 * bits nor error frames. Listening and querying are therefore MUTUALLY EXCLUSIVE
 * modes, and the transition is explicit — see @c canSetMode(). Nothing on this
 * board enters OBD2 (bus-active) except a host command.
 *
 * ── ONE SPI BUS, TWO CONTEXTS ────────────────────────────────────────────────
 * The drain ISR and the main loop both talk to the MCP2515 over the same SPI
 * bus, and a transaction interrupted by another corrupts both. The ISR touches
 * the bus only while the drain is ARMED, which @c canSetMode() does for
 * DISCOVER and SNIFF alone; and every function below that touches the bus from
 * the main loop holds the drain interrupt masked for its whole duration
 * (@c canDrainIrqMask()). Code outside this module that drives the controller —
 * the OBD-II path through the CAN library — runs only in OFF and OBD2, where
 * the drain is disarmed, and masks it as well.
 */

#include <Arduino.h>
#include <CAN.h>

#include "VehicleSignals.h"
#include "CANMap.h"
#include "CANFrameRing.h"

#ifndef DASHCAM_CANBUS_VENDORED_FIXES
#error "Stock arduino-CAN detected. Build with --library peripherals/mkr_zero/vendor/CANBus (see vendor/CANBus/PATCHES.md); upstream observe() requests Configuration mode, not Listen-Only, so the controller would receive nothing at all."
#endif

/** @brief Controller operating mode. Mirrors the Jetson's CMD_SET_CAN_MODE arg. */
enum class CanMode : uint8_t {
    OFF       = 0, ///< Controller not initialised, or parked in Configuration.
    /**
     * Listen-only, accept-all, every frame streamed raw over USB, NOTHING
     * decoded. The boot mode, and where the node returns on its own.
     */
    DISCOVER  = 1,
    SNIFF     = 2, ///< Listen-only, map filters, decoded from the ring and streamed. Host command only.
    OBD2      = 3  ///< Normal + One-Shot, transmits 0x7DF. Bus-active. Host command only.
};

/** @brief Outcome of a mode transition. */
enum class CanModeStatus : int8_t {
    OK             =  0, ///< Achieved and verified via CANSTAT.
    UNCHANGED      =  1, ///< Already in this mode; nothing done.
    NOK_RATE_LIMIT = -1, ///< Too soon after the last transition.
    NOK_CONFIG     = -2, ///< Could not reach Configuration mode.
    NOK_VERIFY     = -3, ///< CANSTAT never reported the requested mode.
    NOK_FILTER     = -4, ///< Filter programming failed.
    NOK_NO_OSM     = -5  ///< OBD2 requested but One-Shot Mode did not latch.
};

// ─── measured Honda F-CAN identifiers ─────────────────────────────────────────

#define CAN_ID_POWERTRAIN   0x17Cu ///< rpm, pedal, brake. opendbc POWERTRAIN_DATA.
#define CAN_ID_WHEEL_SPEEDS 0x1D0u ///< Four 15-bit wheel speeds. opendbc WHEEL_SPEEDS.
#define CAN_ID_GEARBOX      0x191u ///< Selector position, byte 5.
#define CAN_ID_STEER_TORQUE 0x1ABu ///< EPS motor assist torque, 9-bit unsigned.
#define CAN_ID_ENGINE_DATA  0x158u ///< Transmission speed + odometer. opendbc ENGINE_DATA.
#define CAN_ID_TURN_SIGNAL  0x294u ///< Indicator STALK position (steady, not the lamp), byte 0 bit 5 left / bit 6 right.

/// Wheel-speed counts are 0.01 km/h each. Never write this as `/ 100.0` — that
/// promotes to software double on this FPU-less part.
#define CAN_WHEEL_KMH_PER_COUNT   0.01f

/**
 * Yaw-rate scale: centi-degrees/s per count of rear-wheel difference.
 *
 * omega[deg/s] = (c_RR - c_RL) / (2*pi*T_rear) with counts at 0.01 km/h.
 * For a rear track near 1.47 m that is ~0.108 deg/s per count, i.e. ~10.83
 * centi-deg/s.  This constant absorbs the whole product, so the true track
 * width never has to be known — calibrate it against GNSS heading rate exactly
 * as the wheel-speed scale was calibrated against GNSS ground speed.
 */
#define CAN_YAW_CDPS_PER_COUNT    10.83f

/// Below this the reluctor wheel-speed sensors report exactly zero, so yaw rate
/// is UNAVAILABLE rather than zero. Measured: 0x1D0 read 0 while a
/// transmission-derived speed still read 1.41 km/h.
#define CAN_YAW_MIN_COUNTS        300u   ///< 3.00 km/h in wheel counts.

/// Reject anything past this as a decode fault or a locked wheel, not a turn.
#define CAN_YAW_MAX_CDPS          9000   ///< 90 deg/s.

/**
 * GNSS heading rate below which the vehicle counts as going straight, for the
 * purpose of learning the tyre-radius mismatch.
 *
 * Tight on purpose. Every degree/s of real turning that leaks into the estimate
 * biases the correction, and the correction is then applied to every subsequent
 * reading - so a loose threshold does not merely add noise, it adds a permanent
 * offset. GNSS heading is itself noisy at low speed, which is why
 * @c yawObserveStraight() also refuses samples below the wheel-sensor cutoff.
 */
#define YAW_STRAIGHT_MAX_DPS      1.0f

/// One transition per this interval, so a command storm cannot thrash the bus.
#define CAN_MODE_MIN_INTERVAL_MS  500UL

/**
 * Frames DECODED per tickCANSniff() call, from the ring, in SNIFF.
 *
 * No longer what drains the controller — the timer does that — so this bounds
 * only how long one pass spends decoding. Leftovers wait in the ring, and the
 * USB stream never overtakes the decoder (see @c canRawStreamable()). 64 frames
 * is a few milliseconds of software float, and clears even the backlog of a
 * 750 ms pass (~870 frames) within about fourteen ordinary passes.
 */
#define CAN_SNIFF_MAX_PER_TICK    64u

/**
 * Frames the drain ISR reads per tick, at most.
 *
 * Exactly what a tick can physically meet, so it never limits a healthy
 * controller: two frames waiting in RXB0/RXB1, and at most one more completing
 * during their reads (~60 us of SPI against >= 94 us for the shortest frame on
 * the wire). Anything further is left for the next tick, 100 us later, still in
 * its buffer. The bound is for a controller answering nonsense, and it caps the
 * ISR at ~120 us — inside the UART's receive margin; see CANStreamHw.h.
 */
#define CAN_DRAIN_MAX_PER_TICK    3u

/** @brief Freshness window for sniffed signals (ms). */
#define VEH_FRESH_SNIFF_MS        200UL
/** @brief Freshness window for OBD-II signals (ms) — a full sweep plus margin. */
#define VEH_FRESH_OBD2_MS         1000UL

/**
 * @brief Running state for the rear-wheel yaw estimator.
 *
 * Holds the tyre-radius mismatch correction, which is NOT a constant offset:
 * a radius difference produces a wheel-count difference PROPORTIONAL to speed.
 * Measured on this car at about +0.74 %, which at 40 km/h fakes ~3.5 deg/s and
 * turns straight motorway driving into a permanent 180 m left-hand curve.  It
 * moves with tyre pressure and wear, so it is re-estimated per trip rather than
 * compiled in.
 */
struct YawEstimator {
    float    epsilon;      ///< Fractional radius mismatch, (c_RR - c_RL) / c_avg.
    uint32_t straightN;    ///< Samples folded into @c epsilon so far.
    bool     calibrated;   ///< True once @c straightN passes the minimum.
    /**
     * Scales copied from the loaded map, so the hot path stays two multiplies.
     *
     * These are per-VEHICLE, not universal: CAN_MAP_DEFAULT_YAW_* are expressed
     * per 0.01 km/h count, so a map whose wheel scale differs would silently
     * mis-scale the yaw rate if the constants stayed compiled in.
     */
    float    cdpsPerCount;
    uint16_t minCounts;
};

/**
 * @brief Zeroes the estimator and takes its scales from @p m.
 * @param m Loaded map; pass nullptr to use the compiled-in defaults.
 */
void initYawEstimator(YawEstimator &y, const CanSignalMap *m = nullptr);

/**
 * @brief Folds one sample into the radius-mismatch estimate.
 *
 * Call ONLY when an independent reference says the vehicle is going straight —
 * GNSS heading rate near zero — because the whole point is to attribute the
 * residual difference to the tyres rather than to a turn.
 *
 * @param[in,out] y       Estimator state.
 * @param[in]     rawRL   Rear-left raw counts.
 * @param[in]     rawRR   Rear-right raw counts.
 */
void yawObserveStraight(YawEstimator &y, uint16_t rawRL, uint16_t rawRR);

/**
 * @brief Yaw rate in centi-deg/s, left positive, or @c INT16_MIN if unavailable.
 *
 * Uses the REAR pair only.  On this front-wheel-drive vehicle the front wheels
 * are driven, so they spin under throttle, and steered, so their speeds also
 * differ by Ackermann geometry on a different track width.  The rears are an
 * undriven, unsteered differential odometer.
 */
int16_t yawRateFromWheels(const YawEstimator &y, uint16_t rawRL, uint16_t rawRR);

/**
 * @brief Installs the signal map the decoder will use.
 *
 * Pass nullptr to fall back to the compiled-in Honda map, which exists so the
 * table-driven path can be A/B'd against the hardcoded one it replaces before
 * SD loading is in play.  Must be called before the first @c canSetMode(SNIFF):
 * the map supplies the IDs the hardware filter is programmed from.
 */
void canSniffSetMap(const CanSignalMap *m);

/** @brief The map currently installed. Never nullptr after canSniffSetMap(). */
const CanSignalMap *canSniffGetMap();

/**
 * @brief Reprograms the hardware receive filters at runtime.
 *
 * For @c CMD_SET_CAN_FILTER, so a host can widen or narrow a capture without
 * editing the SD card and rebooting the vehicle.
 *
 * @p count of 0 means ACCEPT ALL, not accept none — an MCP2515 with a zero mask
 * compares no bits. That is the correct reading of "clear the filters" and the
 * opposite of how it sounds. It no longer costs frames: the timer drain keeps
 * up with the whole bus (DISCOVER runs accept-all permanently), so the filters
 * now decide what SNIFF decodes and streams, not what survives.
 *
 * COSTS A RECEIVE GAP. The controller must re-enter Configuration mode to have
 * its filter registers written, so frames in flight are lost. Only valid in
 * sniff mode; OBD2 mode programs these registers for the 0x7E8 response and
 * refuses to have them overwritten.
 *
 * @param ids    Up to @c CAN_MAP_FILTER_SLOTS standard 11-bit IDs. Fewer than
 *               the hardware's six repeats the last, because a slot left at 0
 *               would accept ID 0 — a real and very high priority identifier.
 * @param count  How many of @p ids are meaningful. Surplus entries are dropped.
 * @return false when not sniffing, or when the controller refused — then it
 *         has been parked in Configuration and @c canGetMode() reports OFF,
 *         which the caller must adopt.
 */
bool canSniffSetFilters(const uint16_t *ids, uint8_t count);

/** @brief Filter IDs currently programmed; 0 means accept-all. */
uint8_t canSniffFilterCount();

/**
 * @brief True when the live filters came from the loaded map.
 *
 * Distinct from "filters are active", and the difference belongs on the wire: a
 * capture filtered by the map excludes what the map's author chose to exclude,
 * while one filtered by a host command excludes whatever that host asked for —
 * and a consumer reading a recording months later can tell them apart only if
 * this was recorded at the time.
 */
bool canSniffFiltersFromMap();

// ─── boot-time source decision ────────────────────────────────────────────────
//
// NOT ARMED BY THE PRODUCTION SKETCH any more. The probe decided at boot between
// SNIFF and an automatic OBD2 fallback; the node now boots into DISCOVER
// whatever the card holds, enters SNIFF only on a host command — and a host
// command always outranked the probe. Kept, with its tests, because the
// question it answers ("is this map for this car?") is still the right one to
// ask before trusting a map, and a later host-side flow may want it back.

/// Matching frames that end the probe EARLY. A live 50-100 Hz signal reaches
/// this inside half a second, so a correct map never waits the full window.
/// Twenty rather than one, so a single stray ID collision cannot lock in a map
/// meant for a different vehicle.
#define CAN_PROBE_MIN_MATCHES  20u

/// Upper bound on the probe, measured from the FIRST FRAME - not from boot.
/// Long enough to cover an ID that only wakes when the shifter leaves Park
/// (0x158 reads all zeros in P on this car), short enough that a wrong map does
/// not cost a useful minute of telemetry.
#define CAN_PROBE_WINDOW_MS    10000UL

/** @brief Where the boot-time source decision has got to. */
enum class CanProbeStage : uint8_t {
    Idle = 0,   ///< Never armed.
    Probing,    ///< Counting matching frames.
    Sniffing,   ///< TERMINAL. The map belongs to this vehicle.
    FellBack,   ///< TERMINAL. Window elapsed with no match; OBD2 entered.
    Skipped,    ///< TERMINAL. No map, or the host took the decision first.
};

/**
 * @brief Probe state.
 *
 * @c clockStarted is the load-bearing field. The window is measured from the
 * first frame of ANY id, NOT from boot: this board powers up when the rig does,
 * which on an ignition-switched supply is seconds before the driver turns the
 * key and on a permanent supply can be hours. A window measured from boot would
 * expire on a silent bus and fall back to OBD2 for a vehicle whose map was
 * perfectly good - and then transmit at a bit rate nothing had confirmed.
 * A silent bus is not evidence about the map.
 */
struct CanProbeState {
    CanProbeStage stage;
    uint32_t      startMs;
    bool          clockStarted;
};

/**
 * @brief Arms the probe, restarts the counters and OPENS THE FILTERS.
 *
 * Call right after a successful @c canSetMode(SNIFF). Entering sniff mode
 * installs the map's hardware filters, which would hide every ID the map does
 * not name — and with them the evidence that the bus is alive at all, so a map
 * for another vehicle on a busy bus would never start the window. The probe
 * therefore runs ACCEPT-ALL (still listen-only); @c canSniffApplyMapFilters()
 * narrows the filters once the verdict is Sniffing.
 *
 * @return false when not in sniff mode, or when the controller refused the
 *         filter write — then it has been parked in Configuration and
 *         @c canGetMode() reports OFF, which the caller must adopt. The probe is
 *         left Skipped either way.
 */
bool canProbeArm(CanProbeState &p);

/**
 * @brief Installs the map's filters once the probe has proven the map.
 *
 * A no-op returning true when a host filter set arrived during the probe — the
 * host outranks the heuristic. Costs a receive gap, like any filter write.
 *
 * @return false when not in sniff mode, or when the controller refused — then
 *         it has been parked in Configuration and @c canGetMode() reports OFF.
 */
bool canSniffApplyMapFilters();

/**
 * @brief Ends the probe without a verdict. Terminal.
 *
 * Skipping a probe that is still running closes the filters it opened, by way
 * of @c canSniffApplyMapFilters() — so a refused write can park the controller,
 * and the caller must adopt @c canGetMode() afterwards.
 */
void canProbeSkip(CanProbeState &p);

/**
 * @brief Advances the probe. Non-blocking; safe to call every pass.
 * @return the stage AFTER this call. Terminal stages never change again.
 */
CanProbeStage canProbeTick(CanProbeState &p, uint32_t nowMs);

/** @brief Frames the SNIFF decoder took from the ring since the counters were reset. */
uint32_t canSniffFrameCount();
/** @brief Of those, the ones whose ID was in the map. */
uint32_t canSniffMatchCount();

/**
 * @brief Receive-buffer overrun events since boot (uint32, wrapping).
 *
 * Each event is at least one frame lost: the MCP2515's overrun flags latch
 * rather than count, and the drain ISR reads and clears them after every tick
 * that found frames, so this is a LOWER BOUND on frames lost in the controller.
 * Boot-cumulative — @c canSniffResetCounters() leaves it alone, because loss
 * does not stop mattering when the probe restarts. It is the @c ovf of the
 * Orin's FS line; compare it with @c canDrainedCount().
 *
 * Wrapping rather than saturating since the FS line declares all its counters
 * uint32 and wrapping, so a consumer can difference any two of them.
 */
uint32_t canSniffOverrunCount();

/**
 * @brief How many DISTINCT map IDs have been seen since the counters were reset.
 *
 * The probe's second condition. A match count alone can be satisfied entirely by
 * one popular identifier — a map for a different model of the same make would
 * clear twenty matches on a shared 100 Hz message while every other row it
 * defines never appeared once.
 */
uint8_t canSniffIdsSeen();

/** @brief Distinct IDs the probe requires before it will lock in: a majority. */
uint8_t canProbeIdsNeeded();
/** @brief Restarts both counters. */
void canSniffResetCounters();

/**
 * @brief Brings the controller into @p mode, verified, and rate-limited.
 *
 * Never passes through Normal mode on the way to a listen-only mode: on a live
 * bus the controller would ACK frames and could emit error frames if the bit
 * timing were wrong, which is what a read-only tap must never do.
 *
 * Entering @c OBD2 sets Normal and One-Shot in ONE register write, and REFUSES
 * if OSM does not latch — OSM is the only thing that caps retransmission, and a
 * bus-active node that retries forever is precisely what must not be created
 * unattended.
 *
 * Owns the drain: it is disarmed for the whole transition (with its interrupt
 * masked), and re-armed only on success into DISCOVER or SNIFF. Every failure
 * path leaves it disarmed, because they all leave the mode OFF.
 *
 * @param[in] mode   Target mode.
 * @param[in] csPin  MCP2515 chip select, for the raw OSM bit-modify.
 * @return @c CanModeStatus.
 */
CanModeStatus canSetMode(CanMode mode, int csPin);

/** @brief The mode last achieved and verified. */
CanMode canGetMode();

/**
 * @brief SNIFF only: decodes up to @c CAN_SNIFF_MAX_PER_TICK frames from the ring.
 *
 * No-op returning 0 in every other mode — DISCOVER in particular decodes
 * nothing, by design. Touches no hardware: the drain ISR has already read the
 * frames out of the controller, so this cannot lose any by running late.
 *
 * Each frame is stamped with the millis() at which the controller delivered it,
 * not the time it happens to be decoded — after a long pass a frame can sit in
 * the ring for hundreds of milliseconds, and the freshness windows must age it
 * from arrival. Remote and extended frames are taken and counted but not
 * decoded, exactly as before (see @c canDecodeFrame()).
 *
 * @param[in,out] v  Signals to update in place.
 * @param[in,out] y  Yaw estimator state.
 * @return Frames taken from the ring this call.
 */
uint8_t tickCANSniff(VehicleSignals &v, YawEstimator &y);

/**
 * @brief Applies ONE received frame to the signals, exactly as SNIFF does.
 *
 * THE decoder: @c tickCANSniff() calls nothing else to decode, and the Orin's
 * offline tool compiles this same function against a recording — so the two
 * cannot drift. Uses the installed map (@c canSniffSetMap(), or the compiled-in
 * default) and touches no hardware. Besides @p v and @p y it updates only this
 * module's decode bookkeeping: the indicator holds, and the map-ID-seen bits
 * the probe reads.
 *
 * Pass STANDARD DATA frames only, as SNIFF does. A remote frame carries no
 * payload (the MCP2515 leaves the previous frame's bytes in the buffer, so
 * decoding one would publish a stale value under a live ID), and an extended
 * identifier whose low bits happened to equal a mapped standard one would be
 * decoded as that message. An @p id above 0x7FF simply matches nothing.
 *
 * @param[in,out] v      Signals to update in place.
 * @param[in,out] y      Yaw estimator state.
 * @param[in]     id     11-bit identifier.
 * @param[in]     dlc    Data length; above 8 means 8.
 * @param[in]     data   @p dlc payload bytes (only those are read).
 * @param[in]     nowMs  millis()-domain arrival time of the frame: the freshness
 *                       stamp of every signal it carries, and the clock of the
 *                       indicator holds.
 * @return 1 if @p id is in the map (the frame was decoded), else 0.
 */
uint8_t canDecodeFrame(VehicleSignals &v, YawEstimator &y, uint32_t id, uint8_t dlc,
                       const uint8_t *data, uint32_t nowMs);

// ─── the drain: timer ISR -> ring ─────────────────────────────────────────────

/**
 * @brief The drain timer's interrupt body. Called from TC3_Handler (CANStreamHw.cpp).
 *
 * Returns at once unless the drain is armed — only @c canSetMode() arms it, for
 * DISCOVER and SNIFF. Armed, it reads READ STATUS, empties whichever receive
 * buffers are full with one READ RX BUFFER each (RXB0 before RXB1: with BUKT
 * and accept-all, RXB1 only ever fills while RXB0 is occupied, so RXB0 holds the
 * older frame), stamps each with micros(), and pushes it into the ring. After a
 * tick that found frames it reads EFLG and counts and clears any overrun.
 * Bounded by @c CAN_DRAIN_MAX_PER_TICK.
 *
 * A status byte with any transmit bit set is impossible in Listen-Only — the
 * node never transmits there, and @c canSetMode() clears CANINTF on entry — so
 * it means the reply is not the controller talking (an unplugged shield floats
 * MISO high, and 0xFF claims both buffers full); the tick is abandoned rather
 * than fill the ring with frames of 0xFF.
 *
 * Plain C++ with no register access, so the host tests call it directly.
 */
void canDrainIsr();

/** @brief True while the drain ISR may touch the bus (DISCOVER or SNIFF). */
bool canDrainArmed();

/**
 * @brief Frames read out of the MCP2515 since boot (uint32, wrapping).
 *
 * Counted by the ISR at the moment of the read, before the ring can refuse the
 * frame, so it is also the vehicle-liveness signal: a change since the last
 * pass means the bus is talking, whatever became of the frames afterwards. The
 * @c drained of the Orin's FS line.
 */
uint32_t canDrainedCount();

/** @brief Frames lost because the ring was full, since boot (uint32, wrapping). */
uint32_t canRingDropCount();

// ─── the ring's consumer side, for the USB stream ─────────────────────────────

/**
 * @brief Frames the USB stream may take now.
 *
 * Everything published, except in SNIFF, where the stream stops at the decoder:
 * a slot is released only when both are done with it, and the stream is the one
 * that releases. Frames that arrived before SNIFF was entered are streamed but
 * not decoded — they were received under the previous mode's filters.
 */
uint16_t canRawStreamable();

/** @brief Copies the oldest streamable frame; false when there is none. */
bool canRawFront(CanRawFrame &f);

/** @brief Releases the frame @c canRawFront() returned. */
void canRawPop();

/** @brief Releases up to @p n streamable frames unread. @return how many. */
uint16_t canRawDiscard(uint16_t n);

// ─── bench self-test: synthetic frames, no bus (build option) ─────────────────
//
// -DDASHCAM_CAN_STREAM_SELFTEST=<frames per second>, selected by
// build_and_upload.sh's "selftest" / "selftest=N" argument. A BENCH build,
// never for the car: it measures the USB stream's throughput without a CAN bus.
//
// While the drain is armed (DISCOVER, or a host-commanded SNIFF) its ISR also
// synthesizes frames into the ring at that rate, with NO SPI, ahead of the real
// MCP2515 drain — which keeps running alongside and, with no bus on the bench,
// reads nothing. Each synthetic frame is a standard data frame, DLC 8, id
// @c CAN_SELFTEST_ID, stamped with micros() like a real one:
//   data[0..3]  sequence number, big-endian: 0 at boot, +1 per frame, so the
//               Orin counts every lost frame exactly, as a gap;
//   data[4..5]  ring fill when it was made (frames queued ahead of it), big-
//               endian: the stream's margin, visible frame by frame;
//   data[6..7]  low 16 bits of the bitwise complement of the sequence number:
//               a self-check that the line arrived as it was formatted.
// On the wire: "F tttttttt 7F0 8 SSSSSSSSFFFFCCCC".
//
// Synthetic frames are COUNTED AS DRAINED — the FS line's drained, and the
// vehicle-liveness signal derived from it. The first keeps the FS line's frame
// conservation exact (streamed + nohost + ringdrop + queued == drained); the
// second makes the bench report a running vehicle, so the IMU stays in full
// capture: the production load with the car on, which is the load the stream
// has to keep up under.
//
// The rate is capped at one frame per drain tick (@c CAN_SELFTEST_MAX_FPS),
// so the generator adds at most one frame's cost (~5 us) to an ISR budgeted in
// CANStreamHw.h. Without the option NONE of this is compiled: a production build
// is the same code it would be if this section did not exist.
#if defined(DASHCAM_CAN_STREAM_SELFTEST)
#define CAN_SELFTEST_ID       0x7F0u   ///< Not on the Brio's bus: its highest id in the 2026-10-03 baseline is 0x590.
#define CAN_SELFTEST_MAX_FPS  10000UL  ///< One per drain tick: 1 s / CAN_DRAIN_PERIOD_US.

/** @brief Synthetic frames made since boot (uint32, wrapping) — the next sequence number. */
uint32_t canSelfTestCount();
#endif

#endif // CAN_SNIFF_FUNCTIONS_H
