#include <SPI.h>
#include <math.h>

#include "CANSniffFunctions.h"
#include "CANStreamHw.h"
#include "DataDictionary.h"

// ─── raw SPI, for the registers the library keeps private ─────────────────────
// Safe alongside the library: every one of its accessors brackets its own
// transaction and releases CS between calls, so interleaving is fine. Safe
// alongside the drain ISR, which uses these same helpers, only because the
// main-loop callers below hold a CanSpiLock — see there.

/// Own settings object rather than OBD2Functions.h's SPICfg. Including that
/// header here would make the passive sniffer depend on the transmit path, and
/// the whole point of the mode split is that those two are never both live.
/// Must stay identical to it: same device, same bus.
static const SPISettings kSniffSPI(10E6, MSBFIRST, SPI_MODE0);

static const uint8_t REG_CANCTRL  = 0x0F;
static const uint8_t REG_CANSTAT  = 0x0E;
static const uint8_t REG_CANINTF  = 0x2C;
static const uint8_t REG_EFLG     = 0x2D;
static const uint8_t REG_RXB0CTRL = 0x60;

static const uint8_t OPMOD_MASK       = 0xE0;
static const uint8_t MODE_NORMAL      = 0x00;
static const uint8_t MODE_LISTEN_ONLY = 0x60;
static const uint8_t MODE_CONFIG      = 0x80;
static const uint8_t FLAG_OSM         = 0x08;

static const uint8_t INSTR_READ        = 0x03;
static const uint8_t INSTR_WRITE       = 0x02;
static const uint8_t INSTR_BITMOD      = 0x05;
static const uint8_t INSTR_READ_RXB0   = 0x90; ///< auto-clears RX0IF on CS rise
static const uint8_t INSTR_READ_RXB1   = 0x94;
/// RX0IF, RX1IF and the transmit flags in one byte, for two bytes on the wire
/// against four for a READ of CANINTF — and the drain asks 10 000 times a second.
static const uint8_t INSTR_READ_STATUS = 0xA0;

static CanMode  gMode          = CanMode::OFF;
static uint32_t gLastModeMs    = 0;
/// Read by the drain ISR; written only under a CanSpiLock, so never mid-tick.
static int      gCsPin         = MCP2515_DEFAULT_CS_PIN;

/**
 * Keeps the drain ISR off the SPI bus for the lifetime of the object.
 *
 * Both contexts drive the same controller over the same bus, and an ISR landing
 * between a main-loop CS-low and its CS-high would clock its own instruction
 * into the middle of that transaction — corrupting both, and quite possibly
 * writing a register nobody asked for. So every function here that touches the
 * bus from the main loop takes one of these first, for its WHOLE duration: a
 * mode transition is several transactions and a CANSTAT poll, and the ISR has
 * no business reading receive buffers half-way through one.
 *
 * A scope object rather than a mask/unmask pair because canSetMode() alone has
 * ten return paths; a forgotten unmask on one of them would stop the drain for
 * good, silently. Nests, so a locked function may call another.
 */
namespace {
struct CanSpiLock {
    CanSpiLock()  { canDrainIrqMask(); }
    ~CanSpiLock() { canDrainIrqUnmask(); }
    CanSpiLock(const CanSpiLock &) = delete;
    CanSpiLock &operator=(const CanSpiLock &) = delete;
};
}

// ─── the drain's state ────────────────────────────────────────────────────────

/// Every frame the drain reads. Zero-initialised storage is an empty ring.
static CanFrameRing gRing;
/// The ISR may touch the bus. Set only by a successful canSetMode() into a
/// listen-only mode, cleared at the start of every transition and on every
/// failure — always under a CanSpiLock.
static volatile bool     gDrainArmed = false;
/// Frames read out of the controller since boot. ISR-written; see canDrainedCount().
static volatile uint32_t gDrained    = 0;
/// SNIFF's decoder position: free-running index in [tail, head] of gRing.
/// Main-loop only.
static uint16_t          gDecodeIdx  = 0;

static uint8_t rawRead(uint8_t reg)
{
    SPI.beginTransaction(kSniffSPI);
    digitalWrite(gCsPin, LOW);
    SPI.transfer(INSTR_READ);
    SPI.transfer(reg);
    const uint8_t v = SPI.transfer(0x00);
    digitalWrite(gCsPin, HIGH);
    SPI.endTransaction();
    return v;
}

static void rawWrite(uint8_t reg, uint8_t value)
{
    SPI.beginTransaction(kSniffSPI);
    digitalWrite(gCsPin, LOW);
    SPI.transfer(INSTR_WRITE);
    SPI.transfer(reg);
    SPI.transfer(value);
    digitalWrite(gCsPin, HIGH);
    SPI.endTransaction();
}

static void rawBitModify(uint8_t reg, uint8_t mask, uint8_t value)
{
    SPI.beginTransaction(kSniffSPI);
    digitalWrite(gCsPin, LOW);
    SPI.transfer(INSTR_BITMOD);
    SPI.transfer(reg);
    SPI.transfer(mask);
    SPI.transfer(value);
    digitalWrite(gCsPin, HIGH);
    SPI.endTransaction();
}

/**
 * @brief Requests a mode, possibly with extra CANCTRL bits, and confirms it.
 *
 * The poll is BOUNDED, never a single read. The MCP2515 completes a mode change
 * only at the end of the message currently in progress, so an immediate readback
 * can still show the old mode on a busy bus. Reading once and giving up reports
 * failure on a transition that then completes anyway — and if the target was
 * Normal, that leaves the controller bus-active while the caller believes it
 * refused. A caller cannot recover from a state it was told does not exist.
 *
 * @param mask        CANCTRL bits to write.
 * @param value       Values for those bits.
 * @param expectMode  OPMOD value CANSTAT must report.
 */
static bool rawSetModeMasked(uint8_t mask, uint8_t value, uint8_t expectMode)
{
    rawBitModify(REG_CANCTRL, mask, value);
    for (uint16_t tries = 0; tries < 500u; ++tries) {
        if ((rawRead(REG_CANSTAT) & OPMOD_MASK) == expectMode) return true;
        delayMicroseconds(100);
    }
    return false;
}

/** @brief Requests a mode and confirms it via CANSTAT OPMOD. Bounded. */
static bool rawSetMode(uint8_t mode)
{
    return rawSetModeMasked(OPMOD_MASK, mode, mode);
}

// ─── the installed map ────────────────────────────────────────────────────────

static CanSignalMap  gDefaultMap;
static const CanSignalMap *gMap = nullptr;
static uint32_t gFrames  = 0;
static uint32_t gMatches = 0;
/// Receive-buffer overrun EVENTS for the whole boot; see canSniffOverrunCount().
/// ISR-written since the drain moved into the timer interrupt.
static volatile uint32_t gOverruns = 0;

/**
 * Last millis() at which each indicator lamp was seen LIT.
 *
 * Written for a lamp blinking at ~1.5 Hz, which telemetry at ~4 Hz would alias
 * into a signal dark half the time, so each lit frame is held here. On the Brio
 * the mapped bits turned out to be the STALK position, steady while set (the
 * 2026-10-03 all-ID baseline: no dark frame in 15 s at 24 Hz; see the map), so
 * there the hold only delays "off" by CAN_TURN_HOLD_MS. It stays because a map
 * for another vehicle may point at a real lamp.
 *
 * Zero means never seen. Unsigned subtraction makes the comparison correct
 * across the millis() wrap, so no reset is needed on mode changes.
 */
static uint32_t gTurnLeftLitMs  = 0;
static uint32_t gTurnRightLitMs = 0;
/// Same treatment for hazards. No hazard signal has been found on the Brio's bus
/// (see the map), so on that car this stays zero.
static uint32_t gHazardLitMs    = 0;

/**
 * The two indicator lamps as seen in the CURRENT frame: -1 absent, 0 dark, 1 lit.
 *
 * The hold has to be cancellable, not just decaying. A driver moving the stalk
 * from left to right lights the right lamp while the left hold is still running,
 * so both read active for up to CAN_TURN_HOLD_MS and the payload says HAZARDS —
 * observed on the vehicle at every single direction change.
 *
 * One frame settles it. A frame that lights right while left is DARK proves left
 * has stopped now, so its hold is dropped immediately. A frame that lights BOTH
 * is a genuine simultaneous flash and leaves both holds alone, which is what
 * keeps this from breaking a vehicle that really does drive hazards through
 * these bits. The dark half of an ordinary blink lights neither, so it cancels
 * nothing and the hold survives the gap — which is the whole point of it.
 */
static int8_t gFrameTurnL = -1;
static int8_t gFrameTurnR = -1;

/**
 * How long an indicator stays "active" after its last flash.
 *
 * Comfortably longer than the dark half of a blink (~330 ms at the ~1.5 Hz
 * every jurisdiction mandates, and the ~2.5 Hz a bulb-out fault produces), and
 * short enough that it expires well inside the gap between one manoeuvre and
 * the next. This is the only tuned constant in the turn-signal path.
 */
#define CAN_TURN_HOLD_MS  900UL

/**
 * The compiled-in Honda map.
 *
 * Exists so the table-driven decoder can be A/B'd against the hardcoded one it
 * replaces BEFORE SD loading, the boot probe or the protocol change are in play.
 * If the car reports different numbers with this installed, the generic path is
 * wrong — not the card, not the parser, not the probe. That separation is worth
 * the ~200 bytes of flash it costs.
 *
 * Byte-for-byte the same rows as config/canmap.brio.txt.
 */
static void buildDefaultMap(CanSignalMap &m)
{
    static const struct { uint8_t slot; uint16_t id; uint8_t start, len; float scale; } kRows[] = {
        { CAN_SIG_SPEED,         0x158u,  7, 16, 0.01f },
        { CAN_SIG_RPM,           0x17Cu, 23, 16, 1.0f  },
        { CAN_SIG_PEDAL,         0x17Cu,  7,  8, 1.0f  },
        { CAN_SIG_BRAKE_SWITCH,  0x17Cu, 32,  1, 1.0f  },
        { CAN_SIG_BRAKE_PRESSED, 0x17Cu, 53,  1, 1.0f  },
        { CAN_SIG_GEAR,          0x191u, 44,  5, 1.0f  },
        { CAN_SIG_STEER_TORQUE,  0x1ABu,  1, 10, 1.0f  },
        { CAN_SIG_TURN_LEFT,     0x294u,  5,  1, 1.0f  },
        { CAN_SIG_TURN_RIGHT,    0x294u,  6,  1, 1.0f  },
        { CAN_SIG_WHEEL_FL,      0x1D0u,  7, 15, 0.01f },
        { CAN_SIG_WHEEL_FR,      0x1D0u,  8, 15, 0.01f },
        { CAN_SIG_WHEEL_RL,      0x1D0u, 25, 15, 0.01f },
        { CAN_SIG_WHEEL_RR,      0x1D0u, 42, 15, 0.01f },
    };

    canMapInitDefaults(m);
    for (uint8_t i = 0; i < (uint8_t)(sizeof(kRows) / sizeof(kRows[0])); ++i) {
        CanSigRow &r = m.row[m.rowCount];
        r.scale    = kRows[i].scale;
        r.canId    = kRows[i].id;
        r.startBit = kRows[i].start;
        r.len      = kRows[i].len;
        r.minDlc   = canRowMinDlc(kRows[i].start, kRows[i].len);
        r.slot     = kRows[i].slot;
        r.flags    = 0;
        if (kRows[i].len == 1u) r.flags |= CAN_ROW_SINGLEBIT;
        else if ((kRows[i].start & 7u) == 7u && (kRows[i].len % 8u) == 0u) r.flags |= CAN_ROW_ALIGNED;
        m.slotRow[kRows[i].slot] = m.rowCount;
        ++m.rowCount;
    }
    // opendbc GEAR_SHIFTER, confirmed on this car through the full selector sweep.
    m.gearRaw[(uint8_t)VehGear::PARK]    = 1u;
    m.gearRaw[(uint8_t)VehGear::REVERSE] = 2u;
    m.gearRaw[(uint8_t)VehGear::NEUTRAL] = 3u;
    m.gearRaw[(uint8_t)VehGear::DRIVE]   = 4u;
    m.gearRaw[(uint8_t)VehGear::LOW]     = 7u;
    m.gearRaw[(uint8_t)VehGear::SPORT]   = 0x0Au;
    (void)canMapFinalise(m);
}

void canSniffSetMap(const CanSignalMap *m)
{
    if (m != nullptr && m->loaded) { gMap = m; return; }
    if (!gDefaultMap.loaded) buildDefaultMap(gDefaultMap);
    gMap = &gDefaultMap;
}

const CanSignalMap *canSniffGetMap()
{
    if (gMap == nullptr) canSniffSetMap(nullptr);
    return gMap;
}

uint32_t canSniffFrameCount() { return gFrames; }
uint32_t canSniffMatchCount() { return gMatches; }
uint32_t canSniffOverrunCount() { return gOverruns; }

/// One bit per directory entry, set the first time that ID is decoded. This is
/// what turns "twenty frames arrived" into "twenty frames arrived from the IDs
/// this map actually names".
static uint16_t gIdsSeenMask = 0;

uint8_t canSniffIdsSeen()
{
    uint8_t n = 0;
    for (uint16_t m = gIdsSeenMask; m; m >>= 1) n = (uint8_t)(n + (m & 1u));
    return n;
}

uint8_t canProbeIdsNeeded()
{
    const CanSignalMap &m = *canSniffGetMap();
    // A strict majority, rounded up, and never more than the map defines.
    // One ID is one ID: a single-signal map cannot be cross-checked, and
    // demanding two would make it permanently unprovable.
    return (m.idCount <= 1u) ? m.idCount : (uint8_t)((m.idCount + 1u) / 2u);
}

void canSniffResetCounters() { gFrames = 0; gMatches = 0; gIdsSeenMask = 0; }

// ─── boot-time source decision ────────────────────────────────────────────────

// Defined with the mode state machine below; the probe borrows both.
static bool          applyFilterSet(const uint16_t *ids, uint8_t count);
static CanModeStatus failToConfig(CanModeStatus why);

/// A host CMD_SET_CAN_FILTER landed during this sniff session. The host
/// outranks the map, so the probe's verdict must not replace that filter set.
static bool gHostFilters = false;

bool canProbeArm(CanProbeState &p)
{
    const CanSpiLock lock;   // the filter write below goes through Configuration
    p.stage        = CanProbeStage::Probing;
    p.startMs      = 0;
    p.clockStarted = false;
    canSniffResetCounters();

    // ACCEPT ALL while the map is unproven — the probe cannot work otherwise.
    //
    // Entering SNIFF programs the MAP's filters, and they reject every ID the
    // map does not name in hardware. The probe's clock waits for the first
    // frame, and with the map's filters in place a map for another vehicle on a
    // busy bus admits NOTHING: the frame count stayed at zero, the window never
    // opened, and the fallback to OBD2 never came — a simulated probe was still
    // pending after an hour. "Is the bus alive?" has to be asked of every ID;
    // only "is this the right map?" is a question about the map's own IDs, and
    // the directory scan in canDecodeFrame() still answers that one in software.
    //
    // Listen-only either way: applyFilterSet() lands back in Listen-Only, so
    // opening the filters never makes the node bus-active.
    gHostFilters = false;
    if (gMode != CanMode::SNIFF) {
        p.stage = CanProbeStage::Skipped;
        return false;
    }
    if (!applyFilterSet(nullptr, 0u)) {
        // The write goes through Configuration, so a refusal leaves the
        // controller in a state nobody verified. Park it where every recovery
        // path can start from, and say so: the caller resynchronises its mode
        // and the OFF retry brings the controller back and re-arms.
        p.stage = CanProbeStage::Skipped;
        (void)failToConfig(CanModeStatus::NOK_FILTER);
        return false;
    }
    return true;
}

void canProbeSkip(CanProbeState &p)
{
    const bool wasProbing = (p.stage == CanProbeStage::Probing);
    p.stage = CanProbeStage::Skipped;

    // A probe skipped mid-run leaves behind the accept-all filters it opened,
    // and nothing else would ever narrow them: a host that asks for SNIFF while
    // already sniffing gets UNCHANGED, which reprograms nothing. Install the
    // map's, exactly as a Sniffing verdict would (a host filter set is kept).
    // A refusal parks the controller and canGetMode() says OFF.
    if (wasProbing) (void)canSniffApplyMapFilters();
}

CanProbeStage canProbeTick(CanProbeState &p, uint32_t nowMs)
{
    if (p.stage != CanProbeStage::Probing) return p.stage;   // terminal

    // Wait, indefinitely and safely, for the bus to say anything at all. This is
    // the whole reason the clock is not started at boot. Listen-only emits
    // neither ACK bits nor error frames, so sitting here costs nothing.
    if (!p.clockStarted) {
        if (canSniffFrameCount() == 0u) return p.stage;
        p.clockStarted = true;
        p.startMs      = nowMs;
    }

    // Two conditions, not one. A raw match count can be reached entirely by a
    // single ID, and popular identifiers are shared across a manufacturer's
    // whole range — a Civic map plugged into this Brio would see 0x17C at 100 Hz
    // and clear twenty matches in 200 ms while every other row it defines never
    // appeared. That is the exact failure the probe exists to catch, and a
    // count alone cannot see it.
    //
    // Requiring most of the map's IDs to show up is what makes the test about
    // THIS map rather than about the bus being busy. Not all of them: a row may
    // legitimately stay silent early on — 0x158 reads zero until the shifter
    // leaves Park — so the bar is a majority, and the window still has to elapse
    // before a shortfall is called a mismatch.
    const uint8_t seen   = canSniffIdsSeen();
    const uint8_t needed = canProbeIdsNeeded();

    if (canSniffMatchCount() >= CAN_PROBE_MIN_MATCHES && seen >= needed) {
        p.stage = CanProbeStage::Sniffing;
    } else if ((nowMs - p.startMs) > CAN_PROBE_WINDOW_MS) {
        p.stage = CanProbeStage::FellBack;
    }
    return p.stage;
}

/** @brief Reverse gear lookup: raw code -> VehGear. 7 entries, cheaper than 256. */
static VehGear gearFromRaw(const CanSignalMap &m, uint8_t raw)
{
    for (uint8_t g = 1; g < 7u; ++g) {
        if (m.gearRaw[g] == raw) return (VehGear)g;
    }
    return VehGear::UNKNOWN;
}

// ─── yaw estimator ────────────────────────────────────────────────────────────

/// Straight-line samples required before the mismatch correction is trusted.
static const uint32_t YAW_CAL_MIN_SAMPLES = 50u;

void initYawEstimator(YawEstimator &y, const CanSignalMap *m)
{
    y.epsilon    = 0.0f;
    y.straightN  = 0;
    y.calibrated = false;
    // Per-vehicle, taken from the map. Leaving these compiled in would silently
    // mis-scale the yaw rate for any map whose wheel scale is not 0.01 km/h.
    y.cdpsPerCount = (m != nullptr && m->loaded) ? m->yawCdpsPerCount : CAN_MAP_DEFAULT_YAW_CDPS_PER_COUNT;
    y.minCounts    = (m != nullptr && m->loaded) ? m->yawMinCounts    : (uint16_t)CAN_MAP_DEFAULT_YAW_MIN_COUNTS;
}

void yawObserveStraight(YawEstimator &y, uint16_t rawRL, uint16_t rawRR)
{
    const uint32_t sum = (uint32_t)rawRL + (uint32_t)rawRR;
    if (sum == 0u) return;                       // stopped: nothing to learn
    if (rawRL < y.minCounts) return;             // below the sensor cutoff

    // The mismatch is a RATIO, not a count. A radius difference scales the
    // difference with speed, so folding raw counts would give an estimate that
    // is only right at the speed it was measured at.
    const float avg    = (float)sum * 0.5f;
    const float sample = ((float)rawRR - (float)rawRL) / avg;

    // Running mean. A fixed-alpha IIR would keep chasing the most recent
    // corner; tyre pressure does not change on that timescale.
    ++y.straightN;
    y.epsilon += (sample - y.epsilon) / (float)y.straightN;
    if (y.straightN >= YAW_CAL_MIN_SAMPLES) y.calibrated = true;
}

int16_t yawRateFromWheels(const YawEstimator &y, uint16_t rawRL, uint16_t rawRR)
{
    // Below the reluctor cutoff both wheels read exactly zero, so the difference
    // is zero for a reason that has nothing to do with heading. Unavailable.
    if (rawRL < y.minCounts && rawRR < y.minCounts) return INT16_MIN;

    float diff = (float)rawRR - (float)rawRL;

    // Proportional correction, applied only once there is enough evidence to
    // support it. An uncalibrated correction is worse than none: it would
    // subtract a number derived from two or three samples of whatever the car
    // happened to be doing.
    if (y.calibrated) {
        const float avg = ((float)rawRL + (float)rawRR) * 0.5f;
        diff -= y.epsilon * avg;
    }

    const float cdps = diff * y.cdpsPerCount;
    if (cdps >  (float)CAN_YAW_MAX_CDPS) return INT16_MIN;   // decode fault or lockup
    if (cdps < -(float)CAN_YAW_MAX_CDPS) return INT16_MIN;
    return (int16_t)lroundf(cdps);
}

// ─── mode state machine ───────────────────────────────────────────────────────

CanMode canGetMode() { return gMode; }

/// Whether the filters currently programmed came from the loaded map, as
/// opposed to a host command. Reported on the wire, because "filtered" and
/// "filtered the way the map intended" are different statements about a capture.
static bool     gFiltersFromMap = false;
/// Filter IDs currently programmed, and how many are meaningful.
static uint16_t gFilterIds[CAN_MAP_FILTER_SLOTS] = { 0, 0, 0, 0, 0, 0 };
static uint8_t  gFilterCount = 0;

/**
 * @brief Writes a filter set to the controller. NO MODE GUARD — internal.
 *
 * Separate from @c canSniffSetFilters() because the bring-up path calls it
 * DURING the transition into sniff mode, when @c gMode does not yet say Sniff.
 * A single guarded function would either refuse its own bring-up or have to
 * exempt it with a flag, and both were tried before this split.
 */
static bool applyFilterSet(const uint16_t *ids, uint8_t count)
{
    uint16_t f[CAN_MAP_FILTER_SLOTS] = { 0, 0, 0, 0, 0, 0 };
    uint16_t mask = 0x7FFu;

    if (count == 0u) {
        // ACCEPT ALL, which is what an empty filter set means on an MCP2515 —
        // a zero mask compares no bits, so every ID matches whatever the filter
        // registers hold. Worth stating because it is the opposite of what "no
        // filters" sounds like, and because leaving the mask exact with filters
        // at 0 would accept ID 0 alone: a real, and very high priority,
        // identifier rather than a harmless default.
        mask = 0x000u;
    } else {
        if (count > CAN_MAP_FILTER_SLOTS) count = CAN_MAP_FILTER_SLOTS;
        // Fewer IDs than slots: repeat the last. An unused slot left at 0 would
        // accept ID 0 for the same reason as above.
        for (uint8_t i = 0; i < CAN_MAP_FILTER_SLOTS; ++i) {
            f[i] = (i < count) ? ids[i] : ids[count - 1u];
        }
    }

    if (!CAN.setFilterRegisters(
            /* mask0   */ mask, f[0], f[1],
            /* mask1   */ mask, f[2], f[3], f[4], f[5],
            /* allowRollover */ true,
            /* targetMode    */ MODE_LISTEN_ONLY)) {
        return false;
    }

    for (uint8_t i = 0; i < CAN_MAP_FILTER_SLOTS; ++i) gFilterIds[i] = f[i];
    gFilterCount    = count;
    gFiltersFromMap = false;
    return true;
}

bool canSniffSetFilters(const uint16_t *ids, uint8_t count)
{
    // Held across the whole write, and the drain stays ARMED: SNIFF goes on
    // afterwards, so the ISR only has to sit out the Configuration window.
    const CanSpiLock lock;
    if (ids == nullptr && count != 0u) return false;
    // Only in sniff mode. The filter registers belong to the receive path, and
    // in OBD2 mode they are programmed for the 0x7E8 response and must not be
    // overwritten by a host that is thinking about a different bus role.
    if (gMode != CanMode::SNIFF) return false;
    if (!applyFilterSet(ids, count)) {
        // The write went through Configuration, so a refusal can leave the
        // controller receiving nothing while everything still says SNIFF.
        // Park it in a verified state instead, as the probe's writes do; the
        // caller adopts canGetMode() and the OFF retry brings it back.
        (void)failToConfig(CanModeStatus::NOK_FILTER);
        return false;
    }
    gHostFilters = true;
    return true;
}

uint8_t canSniffFilterCount()   { return gFilterCount; }
bool    canSniffFiltersFromMap(){ return gFiltersFromMap; }

/**
 * Programs the Honda filters. Six filters across two masks, which is the whole
 * reason this project vendored a fork that can express them: the stock library
 * could only write one (id, mask) pair to all six, forcing a permissive mask
 * plus a software compare on every frame.
 */
static bool programSniffFilters()
{
    const CanSignalMap &m = *canSniffGetMap();

    // Slot filling — six slots across two masks, repeating the last when the map
    // supplies fewer — lives in applyFilterSet() now that a host command needs
    // exactly the same treatment. It was duplicated here until the two could
    // disagree, which is one copy too many for a rule that decides what a
    // capture contains.

    // More IDs than slots: the surplus are DROPPED, and the mask stays exact.
    //
    // Loosening it to admit them would admit traffic the decoder then discards.
    // That once cost the wanted IDs frames, while the drain ran once per loop()
    // pass at ~267 frames/s against ~1100 arriving; the timer drain keeps up with
    // the whole bus now, so the cost today is decode time and a SNIFF stream
    // that is no longer what the map describes. An exact, smaller filter remains
    // the honest one. WHICH ones go is by slot priority, not by ID order,
    // and is logged by name at boot rather than left to be discovered months
    // later as a signal that never updates.
    if (m.idCount > CAN_MAP_FILTER_SLOTS) {
        Serial.print(F("CANMAP: only "));
        Serial.print((unsigned)CAN_MAP_FILTER_SLOTS);
        Serial.print(F(" of "));
        Serial.print(m.idCount);
        Serial.println(F(" ids fit the hardware filter; dropping:"));
        for (uint8_t i = 0; i < m.idCount; ++i) {
            if (canMapIdIsFiltered(m, m.id[i].canId)) continue;
            Serial.print(F("  0x"));
            Serial.print(m.id[i].canId, HEX);
            Serial.print(F("  ("));
            for (uint8_t k = 0; k < m.id[i].count; ++k) {
                if (k) Serial.print(F(", "));
                Serial.print(canSlotName(m.row[m.id[i].first + k].slot));
            }
            Serial.println(F(")"));
        }
    }

    if (!applyFilterSet(m.filterId, m.filterCount)) return false;
    gFiltersFromMap = true;
    return true;
}

/**
 * @brief Fails a transition into a KNOWN state instead of an unknown one.
 *
 * Every failure path below has already changed hardware. Returning while gMode
 * still holds the PREVIOUS mode leaves software asserting a mode the controller
 * is not in — and if that stale value is SNIFF while the chip actually reached
 * Normal, the node is bus-active and nothing in the system knows. Listen-only
 * is a property this firmware claims; a claim that survives its own failure
 * path is not a property.
 *
 * Configuration is where a failure parks: off the bus, and the only state every
 * other transition can be entered from without a further reset. gMode becomes
 * OFF to match, which also makes the next attempt a real transition rather than
 * an UNCHANGED no-op.
 */
static CanModeStatus failToConfig(CanModeStatus why)
{
    // Verified, not best-effort. If the controller will not even reach
    // Configuration then we do not know what it is doing, and reporting a
    // definite state we have not confirmed is the failure this function exists
    // to prevent. gMode is OFF either way — it means "no usable mode", which is
    // true whether the chip parked or stopped answering — but the caller is told
    // which, so a chip stuck in Normal on a live bus is a reported fault rather
    // than a silent one.
    const bool parked = rawSetMode(MODE_CONFIG);
    gMode = CanMode::OFF;
    // Every caller holds a CanSpiLock, so the ISR is not running; disarmed here
    // it stays off the bus once the lock is released, because the controller it
    // would be reading is parked and receives nothing.
    gDrainArmed = false;

    // gLastModeMs is deliberately NOT stamped.
    //
    // It did, and that made the documented immediate fallback unreachable: a
    // failed SNIFF stamped the clock, and the applyCanMode(OBD2) on the very
    // next line was then rejected by the 500 ms rate limit, leaving the node in
    // NO mode at all. The rate limit exists to stop a command storm thrashing
    // successful transitions; a transition that failed changed nothing worth
    // protecting, and recovery must not be throttled by the fault it recovers
    // from.

    return parked ? why : CanModeStatus::NOK_CONFIG;
}

bool canSniffApplyMapFilters()
{
    const CanSpiLock lock;
    if (gMode != CanMode::SNIFF) return false;
    // The host outranks the map. A filter set it installed while the probe ran
    // is an explicit choice about this capture, and the verdict is only the
    // boot heuristic concluding — it has no business overwriting that.
    if (gHostFilters) return true;
    if (programSniffFilters()) return true;
    // Same reasoning as a refused open in canProbeArm(): the write went through
    // Configuration, so park in a verified state rather than keep claiming SNIFF.
    (void)failToConfig(CanModeStatus::NOK_FILTER);
    return false;
}

CanModeStatus canSetMode(CanMode mode, int csPin)
{
    // Masked for the whole function, on every one of its return paths — gCsPin
    // below is the ISR's chip select too.
    const CanSpiLock lock;
    gCsPin = csPin;

    if (mode == gMode) return CanModeStatus::UNCHANGED;

    // Idempotent above, rate-limited here: a repeated command costs nothing,
    // but an oscillating one would drag the controller through Configuration
    // mode repeatedly and drop frames the whole time.
    if (gLastModeMs != 0 && (millis() - gLastModeMs) < CAN_MODE_MIN_INTERVAL_MS) {
        return CanModeStatus::NOK_RATE_LIMIT;
    }

    // Disarmed, not merely masked, from here on: the controller is about to
    // pass through Configuration and possibly into a mode the drain must never
    // read in (OBD2, whose frames belong to the CAN library's poller). Only
    // success into a listen-only mode, at the bottom, arms it again.
    gDrainArmed = false;

    // Configuration first, always. It is the only mode in which RXBnCTRL and the
    // filter registers are writable, and going via Configuration rather than
    // Normal means a read-only target is never bus-active even momentarily.
    if (!rawSetMode(MODE_CONFIG)) return failToConfig(CanModeStatus::NOK_CONFIG);

    // Clear anything the previous mode left pending, so the first frame decoded
    // after the switch belongs to the new mode.
    rawWrite(REG_CANINTF, 0x00);
    rawBitModify(REG_EFLG, 0xC0, 0x00);   // RX0OVR | RX1OVR

    switch (mode) {
    case CanMode::DISCOVER:
        // RXM=11 accepts everything. The raw stream must not filter — the Orin
        // decides offline what it wanted, and it can only decide about frames
        // it was sent. BUKT doubles the depth the drain has to beat.
        rawWrite(REG_RXB0CTRL, 0x64);     // RXM=11 | BUKT
        rawWrite(REG_RXB0CTRL + 0x10, 0x60);
        if (!rawSetMode(MODE_LISTEN_ONLY)) return failToConfig(CanModeStatus::NOK_VERIFY);
        // The bookkeeping says so too. It used to keep whatever SNIFF had
        // programmed, so telemetry's canMapFlags went on claiming "filtered,
        // by the map" for an accept-all capture.
        for (uint8_t i = 0; i < CAN_MAP_FILTER_SLOTS; ++i) gFilterIds[i] = 0u;
        gFilterCount    = 0u;
        gFiltersFromMap = false;
        break;

    case CanMode::SNIFF:
        // setFilterRegisters() ends in the mode we ask for, so the controller
        // goes Configuration -> Listen-Only without touching Normal.
        if (!programSniffFilters()) return failToConfig(CanModeStatus::NOK_FILTER);
        // Bounded, for the same reason as everywhere else: a single read can
        // catch the controller mid-transition and call a good change a failure.
        {
            bool ok = false;
            for (uint16_t tries = 0; tries < 500u && !ok; ++tries) {
                ok = ((rawRead(REG_CANSTAT) & OPMOD_MASK) == MODE_LISTEN_ONLY);
                if (!ok) delayMicroseconds(100);
            }
            if (!ok) return failToConfig(CanModeStatus::NOK_VERIFY);
        }
        break;

    case CanMode::OBD2: {
        // Hardware-filter the single response ID, then go bus-active.
        if (!CAN.setFilterRegisters(0x7FFu, 0x7E8u, 0x7E8u,
                                    0x7FFu, 0x7E8u, 0x7E8u, 0x7E8u, 0x7E8u,
                                    true, MODE_CONFIG)) {
            return failToConfig(CanModeStatus::NOK_FILTER);
        }
        // Normal AND One-Shot in one write. Doing it as begin-then-bit-modify
        // leaves the controller bus-active without OSM for a few microseconds,
        // which is a window in which it can start retrying forever.
        //
        // The verify is BOUNDED. It used to be one immediate read, which is the
        // one place in this function that could hand back NOK_VERIFY for a
        // transition that then completed a few microseconds later — leaving the
        // chip in Normal while gMode still said SNIFF. Failing here now parks
        // the controller in Configuration, so the reported state is true either
        // way.
        if (!rawSetModeMasked(OPMOD_MASK | FLAG_OSM,
                              MODE_NORMAL | FLAG_OSM, MODE_NORMAL)) {
            return failToConfig(CanModeStatus::NOK_VERIFY);
        }
        // REFUSE if OSM did not latch. It is the only cap on retransmission,
        // and the alternative is an unattended node that hammers a live vehicle
        // bus indefinitely.
        if ((rawRead(REG_CANCTRL) & FLAG_OSM) == 0u) {
            return failToConfig(CanModeStatus::NOK_NO_OSM);
        }
        // Recorded as what it is — one exact filter, not the map's — for the
        // same reason as DISCOVER above.
        for (uint8_t i = 0; i < CAN_MAP_FILTER_SLOTS; ++i) gFilterIds[i] = 0x7E8u;
        gFilterCount    = 1u;
        gFiltersFromMap = false;
        break;
    }

    case CanMode::OFF:
    default:
        // Left in Configuration: off the bus, and the only state from which any
        // other mode can be entered without a further reset.
        break;
    }

    gMode        = mode;
    gLastModeMs  = millis();
    // Every transition reprograms the filters, so a host's set does not survive
    // into the new mode and must not be protected there.
    gHostFilters = false;

    if (mode == CanMode::DISCOVER || mode == CanMode::SNIFF) {
        // SNIFF decodes from what arrives from now on: frames already queued
        // were received under the previous mode's filters, and are streamed
        // but not decoded. The ISR is masked, so head cannot move under us.
        gDecodeIdx  = gRing.head;
        gDrainArmed = true;
    }
    return CanModeStatus::OK;
}

// ─── receive path ─────────────────────────────────────────────────────────────

/**
 * Reads one RX buffer with a single READ RX BUFFER instruction and queues it.
 * Drain ISR only.
 *
 * One CS pair and 13 bytes, against the ~15 separate register reads
 * parsePacket() issues for the same frame. At 500 kbps an 8-byte frame occupies
 * the bus for only ~222 us, so the cheaper read is what makes keeping up
 * possible. The instruction also auto-clears RXnIF on the CS rising edge, so
 * the buffer is free for the next frame the moment this returns.
 *
 * NOTHING is rejected here any more. Remote and extended frames used to be
 * dropped at this point because the decoder was the only consumer; now every
 * frame is evidence for the Orin, and the rule that they are not DECODED moved
 * to tickCANSniff(), which applies it exactly as before. Every answer is in the
 * bytes just read, so this still costs no extra transaction: RXBnSIDL.IDE
 * (bit 3) marks an extended identifier, RXBnSIDL.SRR (bit 4) a standard remote
 * request, and an extended frame carries its remote bit in RXBnDLC bit 6. (An
 * earlier version read RXBnCTRL.RXRTR separately on the belief that a standard
 * remote frame was flagged nowhere else — SRR is exactly that flag.)
 */
static void drainBuffer(uint8_t instr)
{
    uint8_t b[13];
    SPI.beginTransaction(kSniffSPI);
    digitalWrite(gCsPin, LOW);
    SPI.transfer(instr);
    for (uint8_t i = 0; i < 13; ++i) b[i] = SPI.transfer(0x00);
    digitalWrite(gCsPin, HIGH);
    SPI.endTransaction();

    // Stamped after the CS rise: the moment the frame left the controller, and
    // the moment its buffer became free again. micros() is written to be
    // callable here with SysTick held off (delay.c adds a pending tick).
    const uint32_t us = micros();
    // Counted before the ring can refuse it: "read out of the MCP2515" is the
    // liveness signal and the FS line's drained, whatever happens next.
    gDrained = gDrained + 1u;

    const bool ext = (b[1] & 0x08u) != 0u;
    uint32_t   id;
    bool       rtr;
    if (ext) {
        // SIDH = ID28..21, SIDL[7:5] = ID20..18, SIDL[1:0] = ID17..16,
        // EID8 = ID15..8, EID0 = ID7..0.
        id  = ((uint32_t)b[0] << 21) | ((uint32_t)(b[1] >> 5) << 18) |
              ((uint32_t)(b[1] & 0x03u) << 16) | ((uint32_t)b[2] << 8) | (uint32_t)b[3];
        rtr = (b[4] & 0x40u) != 0u;
    } else {
        id  = ((uint32_t)b[0] << 3) | (uint32_t)(b[1] >> 5);
        rtr = (b[1] & 0x10u) != 0u;
    }

    // DLC 9-15 still means eight bytes (ISO 11898-1); canRawPack() clamps it.
    CanRawFrame f;
    canRawPack(f, us, id, ext, rtr, (uint8_t)(b[4] & 0x0Fu), &b[5]);
    (void)canRingPush(gRing, f);   // a full ring counts the loss itself
}

#if defined(DASHCAM_CAN_STREAM_SELFTEST)
// ─── bench self-test generator (see the header; never in a production build) ──

#pragma message "DASHCAM_CAN_STREAM_SELFTEST: the drain ISR synthesizes test frames - a BENCH build, never for the car"

/// Drain ticks per second: the generator's clock, so the rate costs no micros().
#define CAN_DRAIN_TICKS_PER_S (1000000UL / CAN_DRAIN_PERIOD_US)
static_assert(CAN_SELFTEST_MAX_FPS == CAN_DRAIN_TICKS_PER_S,
              "the self-test cap is one synthetic frame per drain tick");
static_assert(DASHCAM_CAN_STREAM_SELFTEST >= 1 && DASHCAM_CAN_STREAM_SELFTEST <= CAN_SELFTEST_MAX_FPS,
              "DASHCAM_CAN_STREAM_SELFTEST is frames per second, 1 to CAN_SELFTEST_MAX_FPS");

/// Rate accumulator, in frames x ticks-per-second: +rate every armed tick, one
/// frame per CAN_DRAIN_TICKS_PER_S. Exact over any whole second of ticks, and
/// below one frame's worth between ticks, so a long disarmed spell owes nothing.
static uint32_t          gSelfAcc = 0;
/// Frames made since boot: the next sequence number. ISR-written only.
static volatile uint32_t gSelfSeq = 0;

uint32_t canSelfTestCount() { return gSelfSeq; }

/** One armed tick of the generator. Drain ISR only; no SPI. */
static void selfTestTick()
{
    gSelfAcc += (uint32_t)DASHCAM_CAN_STREAM_SELFTEST;
    if (gSelfAcc < CAN_DRAIN_TICKS_PER_S) return;
    gSelfAcc -= CAN_DRAIN_TICKS_PER_S;   // the cap makes one per tick the most there can be

    const uint32_t seq  = gSelfSeq;
    const uint16_t fill = canRingCount(gRing);
    const uint32_t chk  = ~seq;
    const uint8_t  d[8] = { (uint8_t)(seq >> 24), (uint8_t)(seq >> 16), (uint8_t)(seq >> 8), (uint8_t)seq,
                            (uint8_t)(fill >> 8), (uint8_t)fill, (uint8_t)(chk >> 8), (uint8_t)chk };
    CanRawFrame f;
    canRawPack(f, micros(), CAN_SELFTEST_ID, false, false, 8u, d);
    gSelfSeq = seq + 1u;
    // Counted as drained, exactly where a real frame is: before the ring can
    // refuse it — see the header for why the bench wants it counted at all.
    gDrained = gDrained + 1u;
    (void)canRingPush(gRing, f);
}
#endif // DASHCAM_CAN_STREAM_SELFTEST

void canDrainIsr()
{
    // Disarmed is the normal state outside DISCOVER and SNIFF: OFF and OBD2
    // belong to the main loop's own CAN calls, and in OBD2 the receive buffers
    // hold the ECU's replies that tickOBD2() is waiting for.
    if (!gDrainArmed) return;

#if defined(DASHCAM_CAN_STREAM_SELFTEST)
    selfTestTick();   // bench build only; armed ticks only, like the real drain below
#endif

    // Bounded twice over: frames read, and status reads. Every round that
    // finds a flag reads at least one buffer, so the second bound never bites
    // on a healthy controller — it is there so that no misreading of the
    // status byte, now or after a later edit, can turn the ISR into a loop.
    uint8_t taken = 0;
    for (uint8_t round = 0; round < CAN_DRAIN_MAX_PER_TICK && taken < CAN_DRAIN_MAX_PER_TICK; ++round) {
        // Gated on the RXnIF flags over SPI, NOT on the INT pin. INT is not
        // wired on this shield — measured 0 asserted against 181790 missed
        // across a full census — and a gate that is never satisfied silently
        // reports an empty bus rather than being merely slower.
        SPI.beginTransaction(kSniffSPI);
        digitalWrite(gCsPin, LOW);
        SPI.transfer(INSTR_READ_STATUS);
        const uint8_t st = SPI.transfer(0x00);
        digitalWrite(gCsPin, HIGH);
        SPI.endTransaction();

        // TX0IF / TX1IF / TX2IF (bits 3, 5, 7) cannot be set here: nothing
        // transmits in Listen-Only, and canSetMode() cleared CANINTF on entry.
        // So this is not the controller answering — a missing shield floats
        // MISO high and 0xFF would claim both buffers full on every tick. Give
        // up on the tick rather than queue frames made of 0xFF. (The TXREQ
        // bits are not tested: an OBD2 request left pending can keep one set
        // through the transition, and that would silence the drain for good.)
        if (st & 0xA8u) return;
        if ((st & 0x03u) == 0u) break;

        // RXB0 first. With BUKT and accept-all, RXB1 only ever fills while
        // RXB0 is occupied, so when both are full RXB0 holds the older frame;
        // and a frame cannot complete between the status read and these reads
        // (>= 94 us on the wire against ~60 us for both), so the order read is
        // the order received. Under SNIFF's filters an ID can go straight to
        // RXB1, and two frames from the same tick may then swap — their
        // timestamps are ~30 us apart either way.
        if (st & 0x01u) { drainBuffer(INSTR_READ_RXB0); ++taken; }
        if ((st & 0x02u) != 0u && taken < CAN_DRAIN_MAX_PER_TICK) {
            drainBuffer(INSTR_READ_RXB1);
            ++taken;
        }
    }
    if (taken == 0u) return;

    // Overruns: counted, then cleared — but only after a tick that found
    // frames, which costs nothing in coverage: an overrun needs both buffers
    // full, and only this ISR empties them, so the tick after one always finds
    // frames. Reading EFLG on every idle tick as well would put a second
    // transaction on each of 10 000 idle ticks a second, nearly doubling the
    // drain's standing cost. Each set flag is one EVENT (the flags latch, they
    // do not count), so the total is a lower bound on frames lost; with BUKT a
    // full RXB0 rolls over into RXB1, so it is mostly RX1OVR.
    const uint8_t eflg = rawRead(REG_EFLG);
    if (eflg & 0xC0u) {
        gOverruns = gOverruns + ((eflg & 0x40u) ? 1u : 0u) + ((eflg & 0x80u) ? 1u : 0u);
        rawBitModify(REG_EFLG, 0xC0u, 0x00u);
    }
}

bool     canDrainArmed()    { return gDrainArmed; }
uint32_t canDrainedCount()  { return gDrained; }
uint32_t canRingDropCount() { return gRing.drops; }

// ─── the ring's consumer side ─────────────────────────────────────────────────

/// Where the stream must stop: at the decoder in SNIFF, so a slot is never
/// released before the decoder has read it; at the producer otherwise.
static uint16_t streamLimit()
{
    return (gMode == CanMode::SNIFF) ? gDecodeIdx : gRing.head;
}

uint16_t canRawStreamable()
{
    return (uint16_t)(streamLimit() - gRing.tail);
}

bool canRawFront(CanRawFrame &f)
{
    if (canRawStreamable() == 0u) return false;
    return canRingPeek(gRing, gRing.tail, f);
}

void canRawPop()
{
    if (canRawStreamable() != 0u) canRingRelease(gRing, 1u);
}

uint16_t canRawDiscard(uint16_t n)
{
    const uint16_t avail = canRawStreamable();
    if (n > avail) n = avail;
    if (n != 0u) canRingRelease(gRing, n);
    return n;
}

/**
 * @brief Applies one decoded row to the signal template.
 *
 * The slot decides what the raw value MEANS, which is what lets the file carry
 * one uniform `scale` column: it is applied to FLOAT slots only. `wheelRaw[]`
 * and `steerMotorTorque` are documented on the wire as raw counts, and scaling
 * them here would break the contract CommProtocol.h states.
 */
static void applyRow(VehicleSignals &v, const CanSignalMap &m,
                     const CanSigRow &r, uint32_t raw, uint32_t now)
{
    const int32_t sv = (r.flags & CAN_ROW_SIGNED) ? canSignExtend(raw, r.len) : (int32_t)raw;

    switch (r.slot) {
    case CAN_SIG_SPEED:
        v.speedKmh = (float)sv * r.scale;
        v.speedMs  = now;
        v.speedSrc = VehSource::CAN_SNIFF;
        break;
    case CAN_SIG_RPM:
        v.rpm    = (float)sv * r.scale;
        v.rpmMs  = now;
        v.rpmSrc = VehSource::CAN_SNIFF;
        break;
    case CAN_SIG_GEAR:
        v.gear    = gearFromRaw(m, (uint8_t)raw);
        v.gearMs  = now;
        v.gearSrc = VehSource::CAN_SNIFF;
        break;
    case CAN_SIG_PEDAL:
        v.pedalGas = (uint8_t)raw;
        v.pedalMs  = now;
        v.pedalSrc = VehSource::CAN_SNIFF;
        break;
    case CAN_SIG_BRAKE_PRESSED:
        v.brakePressed = (raw != 0u);
        v.brakeMs      = now;
        v.brakeSrc     = VehSource::CAN_SNIFF;
        break;
    case CAN_SIG_BRAKE_SWITCH:
        v.brakeSwitch = (raw != 0u);
        v.brakeMs     = now;
        v.brakeSrc    = VehSource::CAN_SNIFF;
        break;
    case CAN_SIG_STEER_TORQUE:
        v.steerMotorTorque = (uint16_t)raw;
        v.steerMs          = now;
        v.steerSrc         = VehSource::CAN_SNIFF;
        break;
    // Only the LIT edge is recorded. The dark half of a blink is not evidence
    // that the indicator stopped, so it must not clear anything; the hold in
    // evaluateHolds() decides when it is genuinely off. The freshness stamp is
    // taken on every frame either way, because the MESSAGE is what went stale.
    case CAN_SIG_TURN_LEFT:
        if (raw != 0u) gTurnLeftLitMs = now;
        gFrameTurnL = (raw != 0u) ? 1 : 0;
        v.turnMs  = now;
        v.turnSrc = VehSource::CAN_SNIFF;
        break;
    case CAN_SIG_TURN_RIGHT:
        if (raw != 0u) gTurnRightLitMs = now;
        gFrameTurnR = (raw != 0u) ? 1 : 0;
        v.turnMs  = now;
        v.turnSrc = VehSource::CAN_SNIFF;
        break;
    case CAN_SIG_HAZARD:
        if (raw != 0u) gHazardLitMs = now;
        v.hazardMs  = now;
        v.hazardSrc = VehSource::CAN_SNIFF;
        break;
    case CAN_SIG_WHEEL_FL: case CAN_SIG_WHEEL_FR:
    case CAN_SIG_WHEEL_RL: case CAN_SIG_WHEEL_RR:
        v.wheelRaw[r.slot - CAN_SIG_WHEEL_FL] = (uint16_t)raw;
        v.wheelMs  = now;
        v.wheelSrc = VehSource::CAN_SNIFF;
        break;
    default:
        break;
    }
}

/**
 * @brief Re-evaluates the indicator holds against @p nowMs.
 *
 * Every pass, not only on frames that carried the message — otherwise a lamp
 * that stopped flashing would stay "active" until the next 0x294 arrived to
 * disprove it, and on a quiet bus that is exactly when it would not. And after
 * every frame canDecodeFrame() applies, so the Orin's offline decoder, which has
 * no passes, sees the held state as of each frame.
 */
static void evaluateHolds(VehicleSignals &v, uint32_t nowMs)
{
    if (v.turnSrc == VehSource::CAN_SNIFF) {
        v.turnLeft  = (gTurnLeftLitMs  != 0u) && ((nowMs - gTurnLeftLitMs)  <= CAN_TURN_HOLD_MS);
        v.turnRight = (gTurnRightLitMs != 0u) && ((nowMs - gTurnRightLitMs) <= CAN_TURN_HOLD_MS);
    }
    if (v.hazardSrc == VehSource::CAN_SNIFF) {
        v.hazard = (gHazardLitMs != 0u) && ((nowMs - gHazardLitMs) <= CAN_TURN_HOLD_MS);
    }
}

uint8_t canDecodeFrame(VehicleSignals &v, YawEstimator &y, uint32_t id, uint8_t dlc,
                       const uint8_t *data, uint32_t nowMs)
{
    const CanSignalMap &m = *canSniffGetMap();
    if (dlc > 8u) dlc = 8u;                     // DLC > 8 still means 8 bytes

    // The buffer SNIFF always decoded from: the payload, zero-padded to eight.
    // A row never reads past its own minDlc, but the offline decoder hands over
    // exactly dlc bytes, so nothing beyond them may even be touched.
    uint8_t d[8];
    for (uint8_t i = 0; i < 8u; ++i) d[i] = (i < dlc) ? data[i] : 0x00u;

    // Directory scan. Ascending by ID, so it early-exits rather than always
    // walking the whole table. A frame that matches nothing was let through
    // by a loose hardware mask and costs exactly this scan — which IS the
    // software compare, so there is no second one to write.
    const CanIdEntry *e = nullptr;
    for (uint8_t i = 0; i < m.idCount; ++i) {
        if (m.id[i].canId == id) { e = &m.id[i]; gIdsSeenMask |= (uint16_t)(1u << i); break; }
        if (m.id[i].canId >  id) break;
    }
    if (e == nullptr) {
        evaluateHolds(v, nowMs);
        return 0u;
    }

    bool sawWheel = false;
    for (uint8_t k = 0; k < e->count; ++k) {
        const CanSigRow &r = m.row[e->first + k];
        // Per-row, not per-frame: a short frame still yields the fields that
        // fit. The old guard rejected the whole frame if any field would not.
        if (dlc < r.minDlc) continue;

        const uint32_t raw =
            (r.flags & CAN_ROW_SINGLEBIT) ? canExtractBit(d, r.startBit)          :
            (r.flags & CAN_ROW_ALIGNED)   ? canExtractAligned(d, r.startBit, r.len)
                                          : canExtractMotorola(d, r.startBit, r.len);
        applyRow(v, m, r, raw, nowMs);
        // Bounded on both sides on purpose. An open-ended `>=` was correct
        // only while the wheels were the last slots in the enum, and would
        // have silently counted every slot added after them as a wheel.
        if (r.slot >= CAN_SIG_WHEEL_FL && r.slot <= CAN_SIG_WHEEL_RR) sawWheel = true;
    }

    // Indicator exclusivity, decided on this frame alone. Both lamps present
    // and only one lit means the other is off NOW, whatever its hold says —
    // so drop it, or a stalk moved from left to right reads as hazards for
    // most of a second. Both lit leaves both holds standing.
    if (gFrameTurnL >= 0 && gFrameTurnR >= 0) {
        if (gFrameTurnL == 1 && gFrameTurnR == 0) gTurnRightLitMs = 0;
        if (gFrameTurnR == 1 && gFrameTurnL == 0) gTurnLeftLitMs  = 0;
    }
    gFrameTurnL = -1;
    gFrameTurnR = -1;

    // Yaw once per wheel frame, after every wheel row in it has landed.
    if (sawWheel && (m.statusFlags & CAN_MAP_F_YAW_OK)) {
        const int16_t yaw = yawRateFromWheels(y, v.wheelRaw[VEH_WHEEL_RL],
                                                 v.wheelRaw[VEH_WHEEL_RR]);
        v.yawRateCdps = yaw;
        v.yawSrc      = (yaw == INT16_MIN) ? VehSource::NONE : VehSource::CAN_SNIFF;
        if (yaw != INT16_MIN) v.yawMs = nowMs;
    }

    // Speed from the front-left wheel ONLY when the map defines no dedicated
    // speed source. Which of the two wins is now a property of the map, not
    // of an if-statement ordering.
    if (sawWheel && m.slotRow[CAN_SIG_SPEED] == 0xFFu &&
        v.wheelRaw[VEH_WHEEL_FL] != VEH_WHEEL_INVALID) {
        v.speedKmh = (float)v.wheelRaw[VEH_WHEEL_FL] * m.wheelKmhPerCount;
        v.speedMs  = nowMs;
        v.speedSrc = VehSource::CAN_SNIFF;
    }

    evaluateHolds(v, nowMs);
    return 1u;
}

/// Arrival time of the last frame decoded; see the clamp in tickCANSniff().
static uint32_t gLastDecodeMs = 0;

uint8_t tickCANSniff(VehicleSignals &v, YawEstimator &y)
{
    // SNIFF only. DISCOVER used to decode here too — it shared this drain —
    // and now decodes nothing at all: its frames go to the Orin raw, and the
    // telemetry's CAN fields stay at their sentinels.
    if (gMode != CanMode::SNIFF) return 0;

    // One reading of each clock per call. A frame's arrival is "now" minus its
    // age in the ring, which puts the controller's delivery time, not the time
    // this pass got round to it, into the millis() domain the freshness windows
    // run on. After a 750 ms pass the difference is the whole window.
    const uint32_t ms0 = millis();
    const uint32_t us0 = micros();

    uint8_t     taken = 0;
    CanRawFrame f;
    while (taken < CAN_SNIFF_MAX_PER_TICK && canRingPeek(gRing, gDecodeIdx, f)) {
        gDecodeIdx = (uint16_t)(gDecodeIdx + 1u);
        ++taken;
        ++gFrames;
        // Counted as a frame (it was on the bus and in the ring) but not
        // decoded, and deliberately not counted as a probe match either — a
        // remote frame is not evidence that this map fits this vehicle. See
        // canDecodeFrame() for why neither kind may reach the decoder.
        if (canRawRtr(f) || canRawExt(f)) continue;

        // Signed: a frame the ISR pushed after us0 was read is "younger than
        // now", which is now. Never backwards either — millis() and micros()
        // round independently by up to a millisecond, and a hold evaluated
        // against a time before the lamp's own stamp would read as long expired.
        const int32_t ageUs = (int32_t)(us0 - f.us);
        uint32_t nowMs = (ageUs > 0) ? ms0 - (uint32_t)ageUs / 1000u : ms0;
        if ((int32_t)(nowMs - gLastDecodeMs) < 0) nowMs = gLastDecodeMs;
        gLastDecodeMs = nowMs;

        if (canDecodeFrame(v, y, canRawId(f), canRawDlc(f), f.data, nowMs)) ++gMatches;
    }

    evaluateHolds(v, ms0);
    return taken;
}
