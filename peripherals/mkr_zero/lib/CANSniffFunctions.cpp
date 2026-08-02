#include <SPI.h>
#include <math.h>

#include "CANSniffFunctions.h"
#include "DataDictionary.h"

// ─── raw SPI, for the registers the library keeps private ─────────────────────
// Safe alongside the library: every one of its accessors brackets its own
// transaction and releases CS between calls, so interleaving is fine.

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

static const uint8_t INSTR_READ      = 0x03;
static const uint8_t INSTR_WRITE     = 0x02;
static const uint8_t INSTR_BITMOD    = 0x05;
static const uint8_t INSTR_READ_RXB0 = 0x90; ///< auto-clears RX0IF on CS rise
static const uint8_t INSTR_READ_RXB1 = 0x94;

static CanMode  gMode          = CanMode::OFF;
static uint32_t gLastModeMs    = 0;
static int      gCsPin         = MCP2515_DEFAULT_CS_PIN;

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

/**
 * Last millis() at which each indicator lamp was seen LIT.
 *
 * The lamp blinks at ~1.5 Hz and telemetry leaves at ~4 Hz, so the
 * instantaneous bit cannot survive the trip: the host would see the indicator
 * dark for roughly half the samples of a manoeuvre it was lit throughout. The
 * flash is therefore held here, where the frame arrives at 24 Hz — the same
 * argument that puts the yaw derivation on this board rather than the Jetson.
 *
 * Zero means never seen. Unsigned subtraction makes the comparison correct
 * across the millis() wrap, so no reset is needed on mode changes.
 */
static uint32_t gTurnLeftLitMs  = 0;
static uint32_t gTurnRightLitMs = 0;

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
        { CAN_SIG_GEAR,          0x191u, 47,  8, 1.0f  },
        { CAN_SIG_STEER_TORQUE,  0x1ABu,  0,  9, 1.0f  },
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
void     canSniffResetCounters() { gFrames = 0; gMatches = 0; }

// ─── boot-time source decision ────────────────────────────────────────────────

void canProbeArm(CanProbeState &p)
{
    p.stage        = CanProbeStage::Probing;
    p.startMs      = 0;
    p.clockStarted = false;
    canSniffResetCounters();
}

void canProbeSkip(CanProbeState &p)
{
    p.stage = CanProbeStage::Skipped;
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

    if (canSniffMatchCount() >= CAN_PROBE_MIN_MATCHES) {
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

/**
 * Programs the Honda filters. Six filters across two masks, which is the whole
 * reason this project vendored a fork that can express them: the stock library
 * could only write one (id, mask) pair to all six, forcing a permissive mask
 * plus a software compare on every frame.
 */
static bool programSniffFilters()
{
    const CanSignalMap &m = *canSniffGetMap();

    // Six slots across two masks, taken from the map's priority-chosen set.
    // Fewer IDs than slots: repeat the last, as the hardcoded version already
    // did — an unused slot left at 0 would accept ID 0, which is a real (and
    // very high priority) identifier.
    uint16_t f[CAN_MAP_FILTER_SLOTS];
    const uint8_t n = m.filterCount;
    for (uint8_t i = 0; i < CAN_MAP_FILTER_SLOTS; ++i) {
        f[i] = (i < n) ? m.filterId[i] : m.filterId[n ? n - 1u : 0u];
    }

    // More IDs than slots: the surplus are DROPPED, and the mask stays exact.
    //
    // Loosening it to admit them would fill both RX buffers with traffic that is
    // then discarded, and the frames that costs come out of the IDs we DO want —
    // the drain is already the bottleneck at ~267 frames/s of capacity against
    // ~1100 arriving. Losing the last few signals deterministically beats losing
    // all of them at random. WHICH ones go is by slot priority, not by ID order,
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

    return CAN.setFilterRegisters(
        /* mask0   */ 0x7FFu, f[0], f[1],
        /* mask1   */ 0x7FFu, f[2], f[3], f[4], f[5],
        /* allowRollover */ true,
        /* targetMode    */ MODE_LISTEN_ONLY);
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
    (void)rawSetMode(MODE_CONFIG);   // best effort; we are already failing
    gMode       = CanMode::OFF;
    gLastModeMs = millis();
    return why;
}

CanModeStatus canSetMode(CanMode mode, int csPin)
{
    gCsPin = csPin;

    if (mode == gMode) return CanModeStatus::UNCHANGED;

    // Idempotent above, rate-limited here: a repeated command costs nothing,
    // but an oscillating one would drag the controller through Configuration
    // mode repeatedly and drop frames the whole time.
    if (gLastModeMs != 0 && (millis() - gLastModeMs) < CAN_MODE_MIN_INTERVAL_MS) {
        return CanModeStatus::NOK_RATE_LIMIT;
    }

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
        // RXM=11 accepts everything. A census must not filter — that is the
        // point of it.
        rawWrite(REG_RXB0CTRL, 0x64);     // RXM=11 | BUKT
        rawWrite(REG_RXB0CTRL + 0x10, 0x60);
        if (!rawSetMode(MODE_LISTEN_ONLY)) return failToConfig(CanModeStatus::NOK_VERIFY);
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
        break;
    }

    case CanMode::OFF:
    default:
        // Left in Configuration: off the bus, and the only state from which any
        // other mode can be entered without a further reset.
        break;
    }

    gMode       = mode;
    gLastModeMs = millis();
    return CanModeStatus::OK;
}

// ─── receive path ─────────────────────────────────────────────────────────────

/**
 * Reads one RX buffer with a single READ RX BUFFER instruction.
 *
 * One CS pair and 13 bytes, against the ~15 separate register reads
 * parsePacket() issues for the same frame. At 500 kbps an 8-byte frame occupies
 * the bus for only ~222 us, so the cheaper read is what makes keeping up
 * possible. The instruction also auto-clears RXnIF on the CS rising edge.
 */
static bool readFrame(uint8_t instr, uint16_t &id, uint8_t &dlc, uint8_t *data)
{
    // RXBnCTRL first, because the READ RX BUFFER below clears RXnIF on its CS
    // rising edge and we want the control byte that belongs to THIS frame.
    // RXRTR (bit 3) is the only place a STANDARD remote frame is flagged: the
    // RTR bit in RXBnDLC is defined for extended frames only, so the bytes the
    // buffer read returns cannot answer the question on their own.
    const uint8_t ctrl = rawRead((instr == INSTR_READ_RXB0) ? REG_RXB0CTRL
                                                            : (uint8_t)(REG_RXB0CTRL + 0x10));

    uint8_t b[13];
    SPI.beginTransaction(kSniffSPI);
    digitalWrite(gCsPin, LOW);
    SPI.transfer(instr);
    for (uint8_t i = 0; i < 13; ++i) b[i] = SPI.transfer(0x00);
    digitalWrite(gCsPin, HIGH);
    SPI.endTransaction();

    // Rejected AFTER the buffer read, never before: the read is what clears
    // RXnIF, so returning early would leave the flag set and the drain would
    // spin on the same frame forever.
    //
    // A remote frame carries NO data bytes, and the MCP2515 leaves the data
    // registers holding whatever the previous frame put there. Decoding one
    // would publish a stale payload under a live ID — a reading that is not
    // merely wrong but plausible. An extended frame is rejected for the mirror
    // reason: only its low 11 bits are compared here, so a 29-bit ID could
    // masquerade as a mapped standard one.
    if (ctrl & 0x08u) return false;             // RXRTR: remote request
    if (b[1] & 0x08u) return false;             // IDE: extended identifier

    id  = (uint16_t)(((uint16_t)b[0] << 3) | (b[1] >> 5));
    dlc = b[4] & 0x0F;
    if (dlc > 8) dlc = 8;                       // DLC > 8 still means 8 bytes
    for (uint8_t i = 0; i < 8; ++i) data[i] = (i < dlc) ? b[5 + i] : 0x00;
    return true;
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
    // tickCANSniff() decides when it is genuinely off. The freshness stamp is
    // taken on every frame either way, because the MESSAGE is what went stale.
    case CAN_SIG_TURN_LEFT:
        if (raw != 0u) gTurnLeftLitMs = now;
        v.turnMs  = now;
        v.turnSrc = VehSource::CAN_SNIFF;
        break;
    case CAN_SIG_TURN_RIGHT:
        if (raw != 0u) gTurnRightLitMs = now;
        v.turnMs  = now;
        v.turnSrc = VehSource::CAN_SNIFF;
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

uint8_t tickCANSniff(VehicleSignals &v, YawEstimator &y)
{
    if (gMode != CanMode::SNIFF && gMode != CanMode::DISCOVER) return 0;

    const CanSignalMap &m = *canSniffGetMap();

    uint8_t decoded = 0;
    for (uint8_t n = 0; n < CAN_SNIFF_MAX_PER_TICK; ++n) {
        // Gated on CANINTF over SPI, NOT on the INT pin. INT is not wired on
        // this shield — measured 0 asserted against 181790 missed across a full
        // census — and a gate that is never satisfied silently reports an empty
        // bus rather than being merely slower.
        const uint8_t intf = rawRead(REG_CANINTF);
        if ((intf & 0x03u) == 0u) break;

        uint16_t id;
        uint8_t  dlc;
        uint8_t  d[8];
        const bool usable = readFrame((intf & 0x01u) ? INSTR_READ_RXB0 : INSTR_READ_RXB1,
                                      id, dlc, d);
        ++decoded;
        ++gFrames;
        // Counted as a frame (it occupied a buffer and cost a drain slot) but
        // not decoded, and deliberately not counted as a probe match either —
        // a remote frame is not evidence that this map fits this vehicle.
        if (!usable) continue;

        const uint32_t now = millis();

        // Directory scan. Ascending by ID, so it early-exits rather than always
        // walking the whole table. A frame that matches nothing was let through
        // by a loose hardware mask and costs exactly this scan — which IS the
        // software compare, so there is no second one to write.
        const CanIdEntry *e = nullptr;
        for (uint8_t i = 0; i < m.idCount; ++i) {
            if (m.id[i].canId == id) { e = &m.id[i]; break; }
            if (m.id[i].canId >  id) break;
        }
        if (e == nullptr) continue;
        ++gMatches;

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
            applyRow(v, m, r, raw, now);
            // Bounded on both sides on purpose. An open-ended `>=` was correct
            // only while the wheels were the last slots in the enum, and would
            // have silently counted every slot added after them as a wheel.
            if (r.slot >= CAN_SIG_WHEEL_FL && r.slot <= CAN_SIG_WHEEL_RR) sawWheel = true;
        }

        // Yaw once per wheel frame, after every wheel row in it has landed.
        if (sawWheel && (m.statusFlags & CAN_MAP_F_YAW_OK)) {
            const int16_t yaw = yawRateFromWheels(y, v.wheelRaw[VEH_WHEEL_RL],
                                                     v.wheelRaw[VEH_WHEEL_RR]);
            v.yawRateCdps = yaw;
            v.yawSrc      = (yaw == INT16_MIN) ? VehSource::NONE : VehSource::CAN_SNIFF;
            if (yaw != INT16_MIN) v.yawMs = now;
        }

        // Speed from the front-left wheel ONLY when the map defines no dedicated
        // speed source. Which of the two wins is now a property of the map, not
        // of an if-statement ordering.
        if (sawWheel && m.slotRow[CAN_SIG_SPEED] == 0xFFu &&
            v.wheelRaw[VEH_WHEEL_FL] != VEH_WHEEL_INVALID) {
            v.speedKmh = (float)v.wheelRaw[VEH_WHEEL_FL] * m.wheelKmhPerCount;
            v.speedMs  = now;
            v.speedSrc = VehSource::CAN_SNIFF;
        }
    }

    // Indicator hold, evaluated every pass rather than only on frames that
    // carried the message — otherwise a lamp that stopped flashing would stay
    // "active" until the next 0x294 arrived to disprove it, and on a quiet bus
    // that is exactly when it would not.
    if (v.turnSrc == VehSource::CAN_SNIFF) {
        const uint32_t t = millis();
        v.turnLeft  = (gTurnLeftLitMs  != 0u) && ((t - gTurnLeftLitMs)  <= CAN_TURN_HOLD_MS);
        v.turnRight = (gTurnRightLitMs != 0u) && ((t - gTurnRightLitMs) <= CAN_TURN_HOLD_MS);
    }

    // Overruns are cleared but not reported here: this is the production path,
    // and an overrun costs one frame of a 50-100 Hz signal that will repeat
    // within 20 ms. The census sketch is where the count matters.
    const uint8_t eflg = rawRead(REG_EFLG);
    if (eflg & 0xC0u) rawBitModify(REG_EFLG, 0xC0u, 0x00u);

    return decoded;
}
