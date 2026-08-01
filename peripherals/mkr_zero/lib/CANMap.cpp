#include <string.h>
#include <stdlib.h>

#include "CANMap.h"
#include "SDFunctions.h"

// ─── slot descriptors (flash, not configurable) ───────────────────────────────

static const char *const kSlotNames[CAN_SIG_COUNT] = {
    "speed", "rpm", "gear", "pedal",
    "brake_pressed", "brake_switch", "steer_torque",
    "turn_left", "turn_right",
    "wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"
};

static const CanSlotKind kSlotKinds[CAN_SIG_COUNT] = {
    CanSlotKind::FLOAT,  // speed
    CanSlotKind::FLOAT,  // rpm
    CanSlotKind::GEAR,   // gear
    CanSlotKind::RAW8,   // pedal
    CanSlotKind::BIT,    // brake_pressed
    CanSlotKind::BIT,    // brake_switch
    CanSlotKind::RAW16,  // steer_torque
    CanSlotKind::BIT,    // turn_left
    CanSlotKind::BIT,    // turn_right
    CanSlotKind::RAW16,  // wheel_fl
    CanSlotKind::RAW16,  // wheel_fr
    CanSlotKind::RAW16,  // wheel_rl
    CanSlotKind::RAW16   // wheel_rr
};

/**
 * Filter priority per slot. Higher keeps its hardware filter slot.
 *
 *  5  speed          — the map is refused outright without a speed source.
 *  4  wheels         — yaw, slip and ABS evidence; nothing reconstructs these.
 *  4  turn signals   — driver INTENT. Lane keeping cannot tell a deliberate
 *                      lane change from a departure without it, so losing it
 *                      does not degrade the feature, it inverts its verdict.
 *  3  brakes         — incident evidence, and cheap.
 *  2  rpm, pedal     — useful, reconstructible in outline from speed.
 *  1  gear, steer    — context. First to go.
 */
static const uint8_t kSlotFilterPriority[CAN_SIG_COUNT] = {
    5,  // speed
    2,  // rpm
    1,  // gear
    2,  // pedal
    3,  // brake_pressed
    3,  // brake_switch
    1,  // steer_torque
    4,  // turn_left
    4,  // turn_right
    4, 4, 4, 4  // wheel_fl, _fr, _rl, _rr
};

CanSlotKind canSlotKind(uint8_t slot)
{
    return (slot < CAN_SIG_COUNT) ? kSlotKinds[slot] : CanSlotKind::RAW16;
}

const char *canSlotName(uint8_t slot)
{
    return (slot < CAN_SIG_COUNT) ? kSlotNames[slot] : "?";
}

uint8_t canSlotFilterPriority(uint8_t slot)
{
    return (slot < CAN_SIG_COUNT) ? kSlotFilterPriority[slot] : 0u;
}

// ─── extraction ───────────────────────────────────────────────────────────────

uint32_t canExtractMotorola(const uint8_t *d, uint8_t s, uint8_t len)
{
    uint32_t v      = 0;
    uint8_t  byteIx = (uint8_t)(s >> 3);
    uint8_t  bitIx  = (uint8_t)(s & 7u);

    while (len--) {
        v = (v << 1) | (uint32_t)((d[byteIx] >> bitIx) & 1u);
        // Walk DOWN, then wrap to bit 7 of the next byte. This wrap is what a
        // byte-pair read gets wrong on an unaligned field.
        if (bitIx == 0u) { bitIx = 7u; ++byteIx; }
        else             { --bitIx; }
    }
    return v;
}

uint32_t canExtractAligned(const uint8_t *d, uint8_t s, uint8_t len)
{
    const uint8_t *p = d + (s >> 3);
    uint32_t v = 0;
    for (uint8_t n = (uint8_t)(len >> 3); n; --n) v = (v << 8) | *p++;
    return v;
}

uint32_t canExtractBit(const uint8_t *d, uint8_t s)
{
    return (uint32_t)((d[s >> 3] >> (s & 7u)) & 1u);
}

int32_t canSignExtend(uint32_t raw, uint8_t len)
{
    if (len >= 32u) return (int32_t)raw;
    const uint32_t signBit = 1UL << (len - 1u);
    return (raw & signBit) ? (int32_t)(raw | (0xFFFFFFFFUL << len)) : (int32_t)raw;
}

uint8_t canRowMinDlc(uint8_t s, uint8_t len)
{
    const uint8_t firstByte   = (uint8_t)(s >> 3);
    const uint8_t bitsInFirst = (uint8_t)((s & 7u) + 1u);
    const uint8_t lastByte    = (len <= bitsInFirst)
        ? firstByte
        : (uint8_t)(firstByte + 1u + (uint8_t)((len - bitsInFirst - 1u) >> 3));
    return (uint8_t)(lastByte + 1u);
}

// ─── scale parsing ────────────────────────────────────────────────────────────

/// 10^-n for n = 0..6. A table, not powf(): powf() is a software routine here
/// and the exponent is one of seven known values.
static const float kNegPow10[7] = { 1.0f, 0.1f, 0.01f, 1e-3f, 1e-4f, 1e-5f, 1e-6f };

/**
 * Parses a plain decimal into a float.
 *
 * NOT strtod(): that drags newlib's full double path in — hundreds of bytes of
 * flash and a software double multiply on a part with no FPU — to parse strings
 * that are always "0.01" or "1". "0.01" here yields exactly 1.0f * 0.01f, which
 * is bit-identical to the literal the hardcoded decoder used, so behaviour on
 * the car does not change.
 */
static bool parseScale(const char *s, float &out)
{
    uint32_t mant = 0;
    uint8_t  digits = 0, frac = 0;
    bool     seenDot = false, seenDigit = false;

    for (; *s; ++s) {
        if (*s == '.') { if (seenDot) return false; seenDot = true; continue; }
        if (*s < '0' || *s > '9') return false;
        if (++digits > 9u) return false;
        mant = mant * 10u + (uint32_t)(*s - '0');
        if (seenDot && ++frac > 6u) return false;
        seenDigit = true;
    }
    if (!seenDigit) return false;
    out = (float)mant * kNegPow10[frac];
    return true;
}

/** @brief Parses decimal or 0x-prefixed hex. */
static bool parseUInt(const char *s, uint32_t &out)
{
    if (s == nullptr || *s == '\0') return false;
    char *end = nullptr;
    const uint32_t v = (uint32_t)strtoul(s, &end, 0);
    if (end == s || (end != nullptr && *end != '\0')) return false;
    out = v;
    return true;
}

// ─── parsing ──────────────────────────────────────────────────────────────────

static uint8_t gErrors = 0;

void canMapInitDefaults(CanSignalMap &m)
{
    memset(&m, 0, sizeof(m));
    for (uint8_t i = 0; i < CAN_SIG_COUNT; ++i) m.slotRow[i] = 0xFFu;
    for (uint8_t i = 0; i < 7; ++i)             m.gearRaw[i] = 0xFFu;
    m.wheelKmhPerCount = CAN_MAP_DEFAULT_WHEEL_KMH_PER_COUNT;
    m.yawCdpsPerCount  = CAN_MAP_DEFAULT_YAW_CDPS_PER_COUNT;
    m.yawMinCounts     = CAN_MAP_DEFAULT_YAW_MIN_COUNTS;
    m.loaded           = false;
    gErrors            = 0;
}

/** @brief Splits @p s on @p sep in place; returns the field count found. */
static uint8_t split(char *s, char sep, char **out, uint8_t maxOut)
{
    uint8_t n = 0;
    out[n++] = s;
    for (char *p = s; *p && n < maxOut; ++p) {
        if (*p == sep) { *p = '\0'; out[n++] = p + 1; }
    }
    return n;
}

/** @brief Trims ASCII whitespace in place. */
static char *trimws(char *s)
{
    while (*s == ' ' || *s == '\t') ++s;
    size_t n = strlen(s);
    while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t')) s[--n] = '\0';
    return s;
}

static int8_t slotFromName(const char *n)
{
    for (uint8_t i = 0; i < CAN_SIG_COUNT; ++i) {
        if (strcmp(n, kSlotNames[i]) == 0) return (int8_t)i;
    }
    return -1;
}

static int8_t gearFromName(const char *n)
{
    if (strcmp(n, "P") == 0) return (int8_t)VehGear::PARK;
    if (strcmp(n, "R") == 0) return (int8_t)VehGear::REVERSE;
    if (strcmp(n, "N") == 0) return (int8_t)VehGear::NEUTRAL;
    if (strcmp(n, "D") == 0) return (int8_t)VehGear::DRIVE;
    if (strcmp(n, "L") == 0) return (int8_t)VehGear::LOW;
    if (strcmp(n, "S") == 0) return (int8_t)VehGear::SPORT;
    return -1;
}

bool canMapParseLine(CanSignalMap &m, char *line)
{
    char *p = trimws(line);
    if (*p == '\0' || *p == '#' || *p == ';') return true;   // comment or blank

    char *f[10];
    const uint8_t n = split(p, ',', f, 10);
    for (uint8_t i = 0; i < n; ++i) f[i] = trimws(f[i]);

    // gearmap,P=1,R=2,...
    if (strcmp(f[0], "gearmap") == 0) {
        bool any = false;
        for (uint8_t i = 1; i < n; ++i) {
            char *eq = strchr(f[i], '=');
            if (eq == nullptr) continue;
            *eq = '\0';
            const int8_t g = gearFromName(trimws(f[i]));
            uint32_t raw = 0;
            if (g < 0 || !parseUInt(trimws(eq + 1), raw) || raw > 255u) { ++gErrors; continue; }
            m.gearRaw[g] = (uint8_t)raw;
            any = true;
        }
        if (!any) ++gErrors;
        return any;
    }

    // yawscale,10.83
    if (strcmp(f[0], "yawscale") == 0) {
        float v = 0.0f;
        if (n < 2 || !parseScale(f[1], v) || v <= 0.0f) { ++gErrors; return false; }
        m.yawCdpsPerCount = v;
        return true;
    }

    // name,id,start,len,sign,scale
    const int8_t slot = slotFromName(f[0]);
    if (slot < 0 || n < 6) { ++gErrors; return false; }

    uint32_t id = 0, start = 0, len = 0;
    float    scale = 1.0f;
    if (!parseUInt(f[1], id)    || id    > 0x7FFu) { ++gErrors; return false; }
    if (!parseUInt(f[2], start) || start > 63u)    { ++gErrors; return false; }
    if (!parseUInt(f[3], len)   || len < 1u || len > 32u) { ++gErrors; return false; }
    if (f[4][0] != 'u' && f[4][0] != 's')          { ++gErrors; return false; }
    if (!parseScale(f[5], scale))                  { ++gErrors; return false; }

    // A field must fit inside the 8-byte payload.
    if (canRowMinDlc((uint8_t)start, (uint8_t)len) > 8u) { ++gErrors; return false; }

    // Duplicate slot: last wins, but it is still a mistake worth counting.
    uint8_t idx;
    if (m.slotRow[slot] != 0xFFu) { idx = m.slotRow[slot]; ++gErrors; }
    else {
        if (m.rowCount >= CAN_SIG_COUNT) { ++gErrors; return false; }
        idx = m.rowCount++;
        m.slotRow[slot] = idx;
    }

    CanSigRow &r = m.row[idx];
    r.scale    = scale;
    r.canId    = (uint16_t)id;
    r.startBit = (uint8_t)start;
    r.len      = (uint8_t)len;
    r.minDlc   = canRowMinDlc((uint8_t)start, (uint8_t)len);
    r.slot     = (uint8_t)slot;
    r.flags    = 0;
    if (f[4][0] == 's')                          r.flags |= CAN_ROW_SIGNED;
    if (len == 1u)                               r.flags |= CAN_ROW_SINGLEBIT;
    else if ((start & 7u) == 7u && (len % 8u) == 0u) r.flags |= CAN_ROW_ALIGNED;
    return true;
}

CanMapStatus canMapFinalise(CanSignalMap &m)
{
    if (gErrors > CAN_MAP_MAX_ERRORS) return CanMapStatus::NOK_TOO_MANY_ERRORS;
    if (m.rowCount == 0)              return CanMapStatus::NOK_NO_SIGNALS;

    // A road-speed source is the one hard requirement. Without it the map cannot
    // feed the overlay or the acceleration estimator, and OBD-II can.
    if (m.slotRow[CAN_SIG_SPEED] == 0xFFu && m.slotRow[CAN_SIG_WHEEL_FL] == 0xFFu) {
        return CanMapStatus::NOK_NO_SPEED;
    }

    // Sort rows by canId (insertion; 11 entries) and rebuild slotRow, so rows
    // sharing an ID end up contiguous and the directory can be a run table.
    for (uint8_t i = 1; i < m.rowCount; ++i) {
        CanSigRow key = m.row[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && m.row[j].canId > key.canId) { m.row[j + 1] = m.row[j]; --j; }
        m.row[j + 1] = key;
    }
    for (uint8_t i = 0; i < CAN_SIG_COUNT; ++i) m.slotRow[i] = 0xFFu;
    for (uint8_t i = 0; i < m.rowCount; ++i)    m.slotRow[m.row[i].slot] = i;

    // Directory: one entry per distinct ID, ascending, so the scan early-exits.
    m.idCount = 0;
    for (uint8_t i = 0; i < m.rowCount; ) {
        const uint16_t id = m.row[i].canId;
        uint8_t k = i;
        while (k < m.rowCount && m.row[k].canId == id) ++k;
        m.id[m.idCount].canId = id;
        m.id[m.idCount].first = i;
        m.id[m.idCount].count = (uint8_t)(k - i);
        ++m.idCount;
        i = k;
    }

    // Derived constants, rescaled ONCE. CAN_YAW_* are expressed per 0.01 km/h
    // count, so a map with a different wheel scale would silently mis-scale the
    // yaw rate if these were left alone.
    const uint8_t flRow = m.slotRow[CAN_SIG_WHEEL_FL];
    if (flRow != 0xFFu) m.wheelKmhPerCount = m.row[flRow].scale;

    const uint8_t rlRow = m.slotRow[CAN_SIG_WHEEL_RL];
    if (rlRow != 0xFFu && m.row[rlRow].scale > 0.0f) {
        const float ratio = m.row[rlRow].scale * 100.0f;   // vs the 0.01 baseline
        m.yawCdpsPerCount *= ratio;
        m.yawMinCounts = (uint16_t)(((float)CAN_MAP_DEFAULT_YAW_MIN_COUNTS * 0.01f) / m.row[rlRow].scale + 0.5f);
    }

    if (m.slotRow[CAN_SIG_WHEEL_RL] != 0xFFu && m.slotRow[CAN_SIG_WHEEL_RR] != 0xFFu) {
        m.statusFlags |= CAN_MAP_F_YAW_OK;
    }

    // ── which IDs get one of the six hardware filter slots ────────────────────
    //
    // By PRIORITY, not by ID. The directory above is sorted by ID so the
    // per-frame scan can early-exit, and reusing that order here would make the
    // casualty whichever ID sorts last rather than whichever matters least.
    // An ID inherits the highest priority among its rows, so a message carrying
    // one important signal is not dropped for the company it keeps.
    m.filterCount = 0;
    const uint8_t want = (m.idCount < CAN_MAP_FILTER_SLOTS) ? m.idCount : (uint8_t)CAN_MAP_FILTER_SLOTS;
    uint16_t taken = 0;   // one bit per directory entry; idCount <= CAN_SIG_COUNT
    for (uint8_t s = 0; s < want; ++s) {
        uint8_t best = 0xFFu, bestPri = 0;
        for (uint8_t i = 0; i < m.idCount; ++i) {
            if (taken & (uint16_t)(1u << i)) continue;
            uint8_t pri = 0;
            for (uint8_t k = 0; k < m.id[i].count; ++k) {
                const uint8_t p = canSlotFilterPriority(m.row[m.id[i].first + k].slot);
                if (p > pri) pri = p;
            }
            // Strictly greater, so a tie keeps the earlier entry — and the
            // directory is ascending by ID, which makes the choice reproducible
            // rather than dependent on row order in the file.
            if (best == 0xFFu || pri > bestPri) { best = i; bestPri = pri; }
        }
        taken |= (uint16_t)(1u << best);
        m.filterId[m.filterCount++] = m.id[best].canId;
    }
    // Back to ascending, so the register order and the boot log read the same
    // way as the directory does.
    for (uint8_t i = 1; i < m.filterCount; ++i) {
        const uint16_t key = m.filterId[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && m.filterId[j] > key) { m.filterId[j + 1] = m.filterId[j]; --j; }
        m.filterId[j + 1] = key;
    }

    if (m.idCount <= CAN_MAP_FILTER_SLOTS) m.statusFlags |= CAN_MAP_F_FILTER_EXACT;
    else                                   m.statusFlags |= CAN_MAP_F_IDS_DROPPED;

    // 8-bit checksum over the accepted rows: identifies WHICH map produced a
    // given telemetry frame, which the host otherwise has no way to know.
    uint8_t sum = 0;
    for (uint8_t i = 0; i < m.rowCount; ++i) {
        const CanSigRow &r = m.row[i];
        sum = (uint8_t)(sum + (uint8_t)r.canId + (uint8_t)(r.canId >> 8) +
                        r.startBit + r.len + r.slot);
    }
    m.checksum = (sum == 0u) ? 1u : sum;   // 0 is reserved for "no map"

    m.statusFlags |= CAN_MAP_F_LOADED;
    m.loaded = true;
    return CanMapStatus::OK;
}

CanMapStatus canMapLoad(CanSignalMap &m, const char *filename)
{
    canMapInitDefaults(m);

    if (!sdReady()) return CanMapStatus::NOK_NO_CARD;

    File32 f;
    if (!sdOpenRead(filename, f)) return CanMapStatus::NOK_NOT_FOUND;

    if (f.fileSize() > (uint32_t)CAN_MAP_MAX_BYTES) {
        f.close();
        return CanMapStatus::NOK_TOO_LARGE;
    }

    char line[CAN_MAP_MAX_LINE];
    while (sdReadLine(f, line, sizeof(line))) {
        (void)canMapParseLine(m, line);
        if (gErrors > CAN_MAP_MAX_ERRORS) break;
    }
    f.close();

    return canMapFinalise(m);
}

const char *canMapStatusName(CanMapStatus s)
{
    switch (s) {
        case CanMapStatus::OK:                  return "ok";
        case CanMapStatus::NOK_NO_CARD:         return "no card";
        case CanMapStatus::NOK_NOT_FOUND:       return "no map file";
        case CanMapStatus::NOK_TOO_LARGE:       return "map too large";
        case CanMapStatus::NOK_TOO_MANY_ERRORS: return "too many parse errors";
        case CanMapStatus::NOK_NO_SIGNALS:      return "map defined no signals";
        case CanMapStatus::NOK_NO_SPEED:        return "map has no speed source";
        default:                                return "?";
    }
}
