#ifndef CAN_MAP_H
#define CAN_MAP_H 1

/**
 * @file CANMap.h
 * @brief The vehicle signal map — which CAN bits mean what, loaded from SD.
 *
 * This is what makes the firmware stop knowing what car it is in.  The decoders
 * used to be a `switch` over five `#define`d Honda IDs used as case labels,
 * with Honda's bit layouts compiled in; moving the rig to another vehicle meant
 * editing and reflashing.  Now the map is data.
 *
 * ── BIT NUMBERING ────────────────────────────────────────────────────────────
 * `startBit` is DBC **Motorola** (`start|len@0`) as used by opendbc: the number
 * of the signal's MOST SIGNIFICANT bit, where bit j of byte i is at position
 * i*8+j and j is the ordinary LSB-first index within the byte (j=7 is that
 * byte's MSB, j=0 its LSB).  It is the SIGNAL that runs MSB-first: it starts at
 * `startBit`, walks DOWN through positions, and wraps to bit 7 of the NEXT byte.
 *
 * So position 7 is the top bit of byte 0 — which is why a plain big-endian
 * 16-bit field at byte 0 is written `7|16@0+` and not `0|16@0+`. A single bit
 * is easier: byte 0 bit 5 is simply position 5, mask 0x20.
 *
 * That wrap is the whole reason a byte-pair read gives the wrong answer for an
 * unaligned field.  Honda's four wheel speeds read as aligned 16-bit pairs give
 * 606/1212/2424/4953 — which no vehicle produces — against the correct
 * 303/303/303/309.  Numbers in this notation can be copied verbatim out of
 * opendbc or read off a CANDiscovery capture.
 *
 * @see peripherals/mkr_zero/config/canmap.brio.txt for a worked example.
 */

#include <Arduino.h>

#include "VehicleSignals.h"

// ─── where the map lives ──────────────────────────────────────────────────────
//
// Convention: `canmap.<vehicle>.txt` in the card root — `canmap.brio.txt`.
//
// Found by PATTERN rather than by a fixed name, and the vehicle is in the
// filename rather than a key inside the file, so that which map a card carries
// is legible from any host OS without opening anything, and one card can hold
// several without them overwriting each other. The reference map in
// config/ already follows it, so preparing a card is a copy with no rename —
// and a rename is exactly the step that gets skipped.
//
// Long file names are enabled in the vendored SdFat, so `<vehicle>` is not
// limited to the three characters an 8.3 name would leave.
#define CAN_MAP_PREFIX       "canmap."
#define CAN_MAP_SUFFIX       ".txt"
/// Longest full filename this will accept, including the NUL.
#define CAN_MAP_NAME_MAX     40u
/// Longest `<vehicle>` portion reported to the log, including the NUL.
#define CAN_MAP_VEHICLE_MAX  20u

/**
 * Refused, never truncated, above this. A truncated map is a WRONG map, and it
 * would be wrong silently — some signals present, others quietly missing.
 *
 * The gate is against a corrupt or binary file, not against a long one: parsing
 * is line-by-line into a CAN_MAP_MAX_LINE buffer, so the file size costs no RAM.
 * It was 2048 and the reference map outgrew it — these files carry the EVIDENCE
 * for each row, which is the point of them, and a map that documents why a bit
 * means what it means is worth more than the two kilobytes it saves.
 */
#define CAN_MAP_MAX_BYTES    8192u
/// Longer lines are skipped whole and counted as an error.
#define CAN_MAP_MAX_LINE     96u
/// Past this the file is rejected outright: nine typos is a half-edited file,
/// not a slightly imperfect one.
#define CAN_MAP_MAX_ERRORS   8u

// ─── baseline constants the map may override ──────────────────────────────────
//
// Defined here rather than pulled from CANSniffFunctions.h: the map OWNS these
// now, and the sniffer will consume them from a loaded map. Importing them the
// other way round would make the map depend on the decoder that depends on it.
//
// The values are the ones the hardcoded Honda decoder used, so a map that omits
// them reproduces today's behaviour exactly.

/// Wheel-speed units per count. Confirmed at 101.36 counts/km/h against GNSS.
#define CAN_MAP_DEFAULT_WHEEL_KMH_PER_COUNT  0.01f
/// Centi-deg/s per count of rear-pair difference; absorbs 2*pi*rear_track.
#define CAN_MAP_DEFAULT_YAW_CDPS_PER_COUNT   10.83f
/// Below this the reluctor sensors read exactly zero, so yaw is UNAVAILABLE.
#define CAN_MAP_DEFAULT_YAW_MIN_COUNTS       300u

/**
 * @brief One named signal the map can define.
 *
 * The four wheel slots MUST stay last and contiguous: the decoder identifies a
 * wheel row by the range and indexes @c VehicleSignals::wheelRaw by subtracting
 * @c CAN_SIG_WHEEL_FL.  Insert anything new BEFORE them.
 */
enum CanSigSlot : uint8_t {
    CAN_SIG_SPEED = 0,
    CAN_SIG_RPM,
    CAN_SIG_GEAR,
    CAN_SIG_PEDAL,
    CAN_SIG_BRAKE_PRESSED,
    CAN_SIG_BRAKE_SWITCH,
    CAN_SIG_STEER_TORQUE,
    CAN_SIG_TURN_LEFT,
    CAN_SIG_TURN_RIGHT,
    /**
     * Hazard lights, as their OWN signal — not "both indicators at once".
     *
     * That inference was tested on the vehicle and is wrong here: switching the
     * hazards on left both turn bits of 0x294 clear. The indicator message
     * reports the STALK, and the hazard switch bypasses the stalk entirely, so
     * on this platform the two are genuinely separate signals. Deriving one from
     * the other would report hazards as "not indicating" — the safest-looking
     * possible answer for a car stopped in a live lane.
     */
    CAN_SIG_HAZARD,
    CAN_SIG_WHEEL_FL,
    CAN_SIG_WHEEL_FR,
    CAN_SIG_WHEEL_RL,
    CAN_SIG_WHEEL_RR,
    CAN_SIG_COUNT
};

/**
 * @brief How a slot's raw value becomes a published signal.
 *
 * Lives in flash and is NOT configurable, which is what lets the file carry a
 * single uniform `scale` column safely: it is only APPLIED to FLOAT slots.
 * `wheelRaw[]` and `steerMotorTorque` are documented on the wire as raw counts,
 * and silently scaling them would break the contract CommProtocol.h states.
 */
enum class CanSlotKind : uint8_t { FLOAT, RAW16, RAW8, BIT, GEAR };

/** @brief The kind of a slot. */
CanSlotKind canSlotKind(uint8_t slot);

/** @brief Human name of a slot, for logs and the parser. */
const char *canSlotName(uint8_t slot);

/// Hardware filter slots the MCP2515 offers: six, across two masks.
#define CAN_MAP_FILTER_SLOTS  6u

/**
 * @brief How hard a slot fights to keep one of the six hardware filter slots.
 *
 * Higher wins.  Only consulted when a map names more distinct IDs than the
 * controller can filter, which is the one case where something must be given
 * up — and giving up the WRONG thing is how a signal quietly never arrives.
 *
 * Ordering by CAN ID, which is what the directory is sorted by, would make the
 * casualty whichever ID happens to sort last.  On the reference map that is
 * 0x294, the turn signals: the single worst choice, because a lane-departure
 * judgement without indicator state reads every deliberate lane change as an
 * unsignalled one.  Priority is therefore a property of what a signal is FOR.
 */
uint8_t canSlotFilterPriority(uint8_t slot);

// ─── row flags ────────────────────────────────────────────────────────────────

#define CAN_ROW_SIGNED     0x01u  ///< Two's complement; sign-extend from @c len.
#define CAN_ROW_ALIGNED    0x02u  ///< (start&7)==7 and len%8==0: whole-byte path.
#define CAN_ROW_SINGLEBIT  0x04u  ///< len==1: single-bit path.

/** @brief One configured signal. */
struct CanSigRow {
    float    scale;     ///< Units per count. Applied to FLOAT slots only.
    uint16_t canId;     ///< 11-bit standard identifier.
    uint8_t  startBit;  ///< DBC Motorola MSB position, 0..63.
    uint8_t  len;       ///< 1..32.
    /**
     * Payload bytes this field needs. DERIVED at load, never configured.
     *
     * Per-row rather than per-frame, which is strictly better than the old
     * `if (dlc < 8) break;`: a short frame still yields the fields that fit.
     */
    uint8_t  minDlc;
    uint8_t  flags;     ///< CAN_ROW_*.
    uint8_t  slot;      ///< CanSigSlot, so dispatch needs no second lookup.
};

/** @brief One distinct CAN ID and the contiguous run of rows that decode it. */
struct CanIdEntry {
    uint16_t canId;
    uint8_t  first;  ///< Index of its first row in @c CanSignalMap::row.
    uint8_t  count;
};

// ─── map status flags ─────────────────────────────────────────────────────────

#define CAN_MAP_F_LOADED        0x01u ///< A usable map was parsed.
#define CAN_MAP_F_YAW_OK        0x02u ///< Both rear wheel slots present.
#define CAN_MAP_F_FILTER_EXACT  0x04u ///< Every ID fits a hardware filter slot.
#define CAN_MAP_F_IDS_DROPPED   0x08u ///< Some IDs could not be filtered and were dropped.

/** @brief Outcome of @c canMapLoad(). */
enum class CanMapStatus : int8_t {
    OK            =  0,
    NOK_NO_CARD   = -1, ///< SD not mounted.
    NOK_NOT_FOUND = -2, ///< No map file on the card.
    NOK_TOO_LARGE = -3, ///< Past CAN_MAP_MAX_BYTES; refused rather than truncated.
    NOK_TOO_MANY_ERRORS = -4,
    NOK_NO_SIGNALS = -5, ///< Parsed, but defined nothing.
    NOK_NO_SPEED   = -6, ///< No road-speed source; see canMapLoad().
};

/** @brief The whole loaded map. */
struct CanSignalMap {
    CanSigRow  row[CAN_SIG_COUNT];      ///< Sorted by canId. @c rowCount valid.
    CanIdEntry id[CAN_SIG_COUNT];       ///< Ascending canId. @c idCount valid.
    uint8_t    slotRow[CAN_SIG_COUNT];  ///< slot -> row index; 0xFF = absent.
    uint8_t    gearRaw[7];              ///< VehGear -> raw code; 0xFF = unmapped.
    /**
     * IDs that get a hardware filter slot, ascending. @c filterCount valid.
     *
     * Equal to the first @c idCount entries of @c id when everything fits.
     * When it does not, this is the priority-chosen subset and the rest are
     * dropped — see @c canSlotFilterPriority().
     */
    uint16_t   filterId[CAN_MAP_FILTER_SLOTS];
    float      wheelKmhPerCount;        ///< Wheel rows' scale, for the FL fallback.
    float      yawCdpsPerCount;         ///< Rescaled to the rear rows' units.
    uint16_t   yawMinCounts;            ///< Rescaled likewise.
    uint8_t    rowCount;
    uint8_t    idCount;
    uint8_t    filterCount;             ///< Entries of @c filterId in use.
    uint8_t    checksum;                ///< Identifies the map on the wire.
    uint8_t    statusFlags;             ///< CAN_MAP_F_*.
    bool       loaded;                  ///< False -> the caller must use OBD2.
};

// ─── extraction ───────────────────────────────────────────────────────────────

/**
 * @brief Extracts @p len bits at DBC Motorola start bit @p s from 8 payload bytes.
 *
 * No bounds test in the loop: @c CanSigRow::minDlc is derived at load and checked
 * by the caller, and the frame reader zero-fills past DLC. Handles len 1..32.
 */
uint32_t canExtractMotorola(const uint8_t *d, uint8_t s, uint8_t len);

/** @brief Fast path for @c CAN_ROW_ALIGNED rows: whole bytes, big-endian. */
uint32_t canExtractAligned(const uint8_t *d, uint8_t s, uint8_t len);

/** @brief Fast path for @c CAN_ROW_SINGLEBIT rows. */
uint32_t canExtractBit(const uint8_t *d, uint8_t s);

/** @brief Sign-extends a @p len -bit two's-complement value. */
int32_t canSignExtend(uint32_t raw, uint8_t len);

/** @brief Payload bytes a field at @p s of @p len bits needs. */
uint8_t canRowMinDlc(uint8_t s, uint8_t len);

// ─── loading ──────────────────────────────────────────────────────────────────

/** @brief Clears @p m to "no map". */
void canMapInitDefaults(CanSignalMap &m);

/**
 * @brief Parses one line into @p m. Exposed so the self-test can drive it.
 * @return true if the line was understood (comments and blanks count as understood).
 */
bool canMapParseLine(CanSignalMap &m, char *line);

/** @brief Finalises a map after the last line: sort, directory, derived constants. */
CanMapStatus canMapFinalise(CanSignalMap &m);

/**
 * @brief Extracts the `<vehicle>` portion of a `canmap.<vehicle>.txt` name.
 *
 * Pure, so the self-test can exercise the convention without a card. Matching
 * is case-insensitive: a card written on a PC may present either case, and FAT
 * short names are upper-case regardless of what was typed.
 *
 * @param[in]  name        Filename with no directory part.
 * @param[out] vehicleOut  The `<vehicle>` portion, NUL-terminated. May be nullptr.
 * @return true if @p name follows the convention with a non-empty vehicle.
 */
bool canMapVehicleFromName(const char *name, char *vehicleOut, size_t vehicleLen);

/**
 * @brief Finds the vehicle map on the card.
 *
 * @param[out] pathOut     Filename to hand to @c canMapLoad().
 * @param[out] vehicleOut  The `<vehicle>` portion, for the log. May be nullptr.
 * @return How many files matched. 0 means none — the caller falls back to
 *         OBD-II. More than 1 is an operator error rather than a fault, so it
 *         still loads: @p pathOut holds the lexicographically FIRST match, which
 *         is reproducible, where "whichever the directory yields first" depends
 *         on the order the card happened to be written in.
 */
uint8_t canMapFindFile(char *pathOut, size_t pathLen,
                       char *vehicleOut, size_t vehicleLen);

/**
 * @brief Loads the map from the SD card.
 *
 * Requires a road-speed source — a @c speed row or a @c wheel_fl row. A map that
 * cannot produce a speed offers nothing OBD-II does not do better, and speed is
 * what the overlay and the acceleration estimator consume, so it returns
 * @c NOK_NO_SPEED and the caller falls back rather than sniffing a bus it can
 * only partially read.
 */
CanMapStatus canMapLoad(CanSignalMap &m, const char *filename);

/** @brief Log-friendly name for a status. */
const char *canMapStatusName(CanMapStatus s);

/** @brief Row index for @p slot, or 0xFF. */
inline uint8_t canMapSlotRow(const CanSignalMap &m, uint8_t slot)
{
    return (slot < CAN_SIG_COUNT) ? m.slotRow[slot] : 0xFFu;
}

/** @brief True if @p canId won one of the hardware filter slots. */
inline bool canMapIdIsFiltered(const CanSignalMap &m, uint16_t canId)
{
    for (uint8_t i = 0; i < m.filterCount; ++i) {
        if (m.filterId[i] == canId) return true;
    }
    return false;
}

#endif // CAN_MAP_H
