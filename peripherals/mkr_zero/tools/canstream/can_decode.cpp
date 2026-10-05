/**
 * @file can_decode.cpp
 * @brief Offline CAN decoder: a candump log through the MKR's OWN decoder.
 *
 *   can_decode [--map FILE|builtin] [--rate HZ | --every-frame] [-o OUT.csv]
 *              [--census CENSUS.csv] [--iface NAME] LOG|-
 *
 * The production firmware decodes nothing by default (CanMode::DISCOVER): every
 * frame goes to the Orin raw, and this is where it becomes signals. It is not a
 * re-implementation. It is compiled from the UNMODIFIED firmware sources —
 * lib/CANSniffFunctions.cpp (canDecodeFrame(), the indicator holds),
 * lib/CANMap.cpp (the map parser, the bit extraction, the checksum) and
 * lib/VehicleSignals.cpp (the freshness expiry) — against the host stand-ins in
 * ../../tests/host, exactly as the host test suites are. So what it prints is
 * what SNIFF would have computed live from the same frames, and a decoder fix
 * reaches the recordings already on disk by rebuilding this.
 *
 * ── WHAT IT REPLAYS, AND HOW ─────────────────────────────────────────────────
 *   - The map is read by canMapLoad() — the MKR's own SD loader — through a
 *     host file standing in for the card (sdOpenRead()/sdReadLine() below; the
 *     line reader mirrors lib/SDFunctions.cpp, CR swallowed, over-long lines
 *     consumed whole). A map the MKR would refuse is refused here, with the
 *     same status name, and its checksum is the 'id=0x..' of the MKR's boot log.
 *     `--map builtin` uses the compiled-in Honda map instead (canSniffSetMap(nullptr)).
 *   - Standard DATA frames go through canDecodeFrame() with the frame's own
 *     timestamp in milliseconds as nowMs. Remote and extended frames are
 *     counted, not decoded — as tickCANSniff() does, for the reasons given at
 *     canDecodeFrame().
 *   - Each output row is the state AT its instant, as a loop() pass at that
 *     instant would see it: every frame up to and including that time applied,
 *     the indicator holds re-evaluated at that time (canDecodeFrame() with an
 *     identifier above 0x7FF, which matches nothing and does only that — the
 *     offline twin of the evaluateHolds() that ends every tickCANSniff() pass),
 *     then expireVehicleSignals() with VEH_FRESH_SNIFF_MS, as loop() does next.
 *     Expiry is monotone in time, so applying it only at the sample instants
 *     yields exactly what per-pass expiry would have shown at those instants.
 *
 * ── TIME ─────────────────────────────────────────────────────────────────────
 * The decoder's clock is millis()-domain uint32: here, milliseconds since the
 * segment began, starting at 1000 (0 means "never" to both the freshness stamps
 * and the holds). A timestamp a little behind its predecessor (an offset update
 * in the logger, below a second) is clamped forward, as tickCANSniff() clamps
 * its own; one more than BACKSTEP_US behind (a wall-clock step) or more than
 * GAP_US ahead (a long gap: port lost, logger restarted, clock step) starts a
 * new SEGMENT: fresh signals and yaw state, and a decoder clock 60 s on, so
 * every freshness window and indicator hold lapses. The `segment` column
 * numbers them; within one, `time` only increases.
 *
 * ── OUTPUT ───────────────────────────────────────────────────────────────────
 * CSV, one row per grid instant (multiples of 1/HZ in log time, default 10 Hz)
 * or with --every-frame one row per frame the map decodes (its `id` column
 * added). The fields are the CAN-derived ones the MKR publishes, in the units it
 * publishes them (CommProtocol.h): speed km/h, rpm, gear selector, pedal raw
 * (x0.5 = %), brake pressed / switch, turn left / right (HELD, as on the wire),
 * hazard, EPS assist torque raw, yaw rate centi-deg/s, wheel speeds raw (0.01
 * km/h counts). An EMPTY cell is unavailable — never seen, expired, or not in
 * the map (hazard, until the map has its row) — never a zero. Yaw is
 * UNCALIBRATED here: the MKR learns the tyre-radius mismatch from GNSS heading
 * (yawObserveStraight()), and a recording has no GNSS, so the column is the raw
 * rear-pair difference times the map's scale, biased by that mismatch. The
 * firmware map has no odometer slot, so there is no odometer column.
 *
 * The census (--census, else stderr) has one row per identifier: count, remote
 * frames, mean rate, first/last time, the set of DLCs seen, and whether the map
 * decodes it. Map identifiers that never appeared are warned about on stderr.
 */

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <map>
#include <string>

// Firmware headers last: the host Arduino.h stand-in defines F(), HEX and DEC as
// macros, which must not reach into the standard library headers above.
#include "CANSniffFunctions.h"
#include "CANMap.h"
#include "VehicleSignals.h"
#include "SDFunctions.h"

// ─── the card, as a host file ─────────────────────────────────────────────────
//
// canMapLoad() reads the map through these, as the tests' stand-ins do; the
// host File32 has no storage, so the open file lives here. Nothing else on the
// SD side is reached by the decoder.

static FILE *g_mapFile = nullptr;

bool sdReady() { return true; }
bool sdOpenRoot(File32 &) { return false; }

bool sdOpenRead(const char *path, File32 &)
{
    if (g_mapFile != nullptr) fclose(g_mapFile);
    g_mapFile = fopen(path, "rb");
    return g_mapFile != nullptr;
}

/// lib/SDFunctions.cpp sdReadLine(), over a FILE*: EOF or '\n' ends a line, '\r'
/// is swallowed, an over-long line is consumed whole but only its head stored
/// (it then fails to parse, as on the card). Returns false only at EOF with
/// nothing read.
bool sdReadLine(File32 &, char *buf, size_t bufLen)
{
    if (buf == nullptr || bufLen == 0 || g_mapFile == nullptr) return false;
    buf[0] = '\0';
    size_t n   = 0;
    bool   any = false;
    for (;;) {
        const int c = fgetc(g_mapFile);
        if (c == EOF) break;
        any = true;
        if (c == '\n') break;
        if (c == '\r') continue;
        if (n + 1 < bufLen) buf[n++] = (char)c;
    }
    buf[n] = '\0';
    return any;
}

// ─── options ──────────────────────────────────────────────────────────────────

/// A step back further than this is a clock step, not an offset update.
static const int64_t BACKSTEP_US = 1000000;
/// A silence longer than this ends the segment instead of filling it with empty rows.
static const int64_t GAP_US      = 60000000;
/// Decoder-clock jump between segments: past every freshness window and hold.
static const uint32_t SEGMENT_GAP_MS = 60000u;
/// Identifier no map can hold (above 0x7FF): canDecodeFrame() then only runs the holds.
static const uint32_t NO_MATCH_ID = 0xFFFFFFFFu;

struct Options {
    const char *log         = nullptr;
    std::string map;
    double      rate        = 10.0;
    bool        everyFrame  = false;
    const char *out         = nullptr;
    const char *census      = nullptr;
    const char *iface       = nullptr;
};

static void usage(FILE *f)
{
    fprintf(f,
        "usage: can_decode [--map FILE|builtin] [--rate HZ | --every-frame] [-o OUT.csv]\n"
        "                  [--census CENSUS.csv] [--iface NAME] LOG|-\n"
        "  LOG          candump -l log (mkr_stream_log.py's can_raw.log, canrawlog2candump.py output)\n"
        "  --map        signal map, parsed by the firmware's canMapLoad(); default\n"
        "               ../../config/canmap.brio.txt beside this binary; 'builtin' = compiled-in map\n"
        "  --rate       rows per second of log time, on multiples of 1/HZ (default 10, max 1000)\n"
        "  --every-frame  one row per frame the map decodes instead\n"
        "  -o           CSV output (default stdout)\n"
        "  --census     per-identifier census CSV (default: printed to stderr)\n"
        "  --iface      decode only frames from this interface\n");
}

/// The repo's reference map, found relative to this binary rather than the cwd.
static std::string defaultMapPath()
{
    char exe[PATH_MAX];
    const ssize_t n = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
    if (n <= 0) return "../../config/canmap.brio.txt";
    exe[n] = '\0';
    char *slash = strrchr(exe, '/');
    if (slash != nullptr) *slash = '\0';
    return std::string(exe) + "/../../config/canmap.brio.txt";
}

static bool parseArgs(int argc, char **argv, Options &o)
{
    o.map = defaultMapPath();
    for (int i = 1; i < argc; ++i) {
        const char *a = argv[i];
        const bool more = (i + 1 < argc);
        if (!strcmp(a, "--map") && more)          o.map = argv[++i];
        else if (!strcmp(a, "--rate") && more)    o.rate = atof(argv[++i]);
        else if (!strcmp(a, "--every-frame"))     o.everyFrame = true;
        else if (!strcmp(a, "-o") && more)        o.out = argv[++i];
        else if (!strcmp(a, "--census") && more)  o.census = argv[++i];
        else if (!strcmp(a, "--iface") && more)   o.iface = argv[++i];
        else if (!strcmp(a, "-h") || !strcmp(a, "--help")) { usage(stdout); exit(0); }
        else if (a[0] == '-' && a[1] != '\0')     return false;
        else if (o.log == nullptr)                o.log = a;
        else                                      return false;
    }
    if (o.log == nullptr) return false;
    if (!(o.rate > 0.0 && o.rate <= 1000.0)) {
        fprintf(stderr, "can_decode: --rate must be in (0, 1000]\n");
        return false;
    }
    return true;
}

// ─── the map ──────────────────────────────────────────────────────────────────

static CanSignalMap g_map;

/// Loads @p path as the MKR would; false (with the reason printed) if the MKR would not.
static bool loadMap(const std::string &path)
{
    if (path == "builtin") {
        canSniffSetMap(nullptr);
        fprintf(stderr, "can_decode: map: compiled-in default (canSniffSetMap(nullptr))\n");
        return true;
    }
    struct stat st;
    if (stat(path.c_str(), &st) != 0) {
        fprintf(stderr, "can_decode: map %s: %s\n", path.c_str(), strerror(errno));
        return false;
    }
    // The host File32 reports no size, so canMapLoad()'s size gate is applied here, with its constant.
    if ((uint64_t)st.st_size > (uint64_t)CAN_MAP_MAX_BYTES) {
        fprintf(stderr, "can_decode: map %s: %s (%lld > %u bytes; the MKR refuses it too)\n", path.c_str(),
                canMapStatusName(CanMapStatus::NOK_TOO_LARGE), (long long)st.st_size, (unsigned)CAN_MAP_MAX_BYTES);
        return false;
    }
    const CanMapStatus ms = canMapLoad(g_map, path.c_str());
    if (g_mapFile != nullptr) { fclose(g_mapFile); g_mapFile = nullptr; }
    if (ms != CanMapStatus::OK) {
        fprintf(stderr, "can_decode: map %s: %s\n", path.c_str(), canMapStatusName(ms));
        return false;
    }

    // Lines the parser rejected, named — the MKR only counts them. A second pass into a scratch map,
    // so the installed one is exactly what canMapLoad() built.
    static CanSignalMap scratch;
    canMapInitDefaults(scratch);
    File32 f;
    if (sdOpenRead(path.c_str(), f)) {
        char line[CAN_MAP_MAX_LINE];
        unsigned no = 0;
        while (sdReadLine(f, line, sizeof(line))) {
            ++no;
            char copy[CAN_MAP_MAX_LINE];
            memcpy(copy, line, sizeof(copy));
            if (!canMapParseLine(scratch, line)) fprintf(stderr, "can_decode: map line %u rejected: %s\n", no, copy);
        }
        fclose(g_mapFile);
        g_mapFile = nullptr;
    }

    canSniffSetMap(&g_map);
    fprintf(stderr, "can_decode: map %s: ok, %u signals across %u ids, id=0x%02X%s\n", path.c_str(),
            (unsigned)g_map.rowCount, (unsigned)g_map.idCount, (unsigned)g_map.checksum,
            (g_map.statusFlags & CAN_MAP_F_YAW_OK) ? ", yaw ok" : ", no yaw (rear wheels unmapped)");
    return true;
}

static bool mapHasId(uint32_t id)
{
    const CanSignalMap &m = *canSniffGetMap();
    for (uint8_t i = 0; i < m.idCount; ++i) if (m.id[i].canId == id) return true;
    return false;
}

// ─── candump parsing ──────────────────────────────────────────────────────────

struct Frame {
    int64_t  tUs;
    uint32_t id;
    bool     ext;
    bool     rtr;
    uint8_t  dlc;
    uint8_t  data[8];
    char     iface[32];
};

enum class ParseResult { OK, SKIP, BAD, FD };

static int hexVal(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/// "(sec.frac) iface ID#DATA [flag]" — candump -l / -L. The time is kept as integer microseconds, so
/// nothing is lost to a double at epoch magnitudes. Blank and '#'-comment lines are SKIP.
static ParseResult parseCandump(const char *s, Frame &f)
{
    while (*s == ' ' || *s == '\t') ++s;
    if (*s == '\0' || *s == '\n' || *s == '#') return ParseResult::SKIP;
    if (*s++ != '(') return ParseResult::BAD;
    char *end = nullptr;
    errno = 0;
    const long long sec = strtoll(s, &end, 10);
    if (end == s || errno != 0 || sec < 0) return ParseResult::BAD;
    s = end;
    int64_t us = 0;
    if (*s == '.') {
        ++s;
        int digits = 0;
        while (*s >= '0' && *s <= '9') {
            if (digits < 6) { us = us * 10 + (*s - '0'); ++digits; }
            ++s;
        }
        while (digits < 6) { us *= 10; ++digits; }
    }
    if (*s++ != ')') return ParseResult::BAD;
    f.tUs = (int64_t)sec * 1000000 + us;

    while (*s == ' ') ++s;
    size_t n = 0;
    while (*s != '\0' && *s != ' ' && *s != '\n') {
        if (n + 1 >= sizeof(f.iface)) return ParseResult::BAD;
        f.iface[n++] = *s++;
    }
    f.iface[n] = '\0';
    if (n == 0) return ParseResult::BAD;
    while (*s == ' ') ++s;

    // Identifier: 3 hex digits standard, 8 extended (can-utils' own rule).
    uint32_t id = 0;
    int idDigits = 0;
    while (hexVal(*s) >= 0) { id = (id << 4) | (uint32_t)hexVal(*s); ++s; ++idDigits; if (idDigits > 8) return ParseResult::BAD; }
    if (*s++ != '#') return ParseResult::BAD;
    if (idDigits == 3)      { f.ext = false; if (id > 0x7FFu) return ParseResult::BAD; }
    else if (idDigits == 8) { f.ext = true;  if (id > 0x1FFFFFFFu) return ParseResult::BAD; }
    else return ParseResult::BAD;
    f.id = id;

    f.rtr = false;
    f.dlc = 0;
    memset(f.data, 0, sizeof(f.data));
    if (*s == '#') return ParseResult::FD;            // CAN FD: not on this bus, not decodable here
    if (*s == 'R' || *s == 'r') {
        f.rtr = true;
        ++s;
        if (*s >= '0' && *s <= '8') f.dlc = (uint8_t)(*s++ - '0');
    } else {
        while (hexVal(*s) >= 0) {
            if (hexVal(s[1]) < 0 || f.dlc >= 8u) return ParseResult::BAD;
            f.data[f.dlc++] = (uint8_t)((hexVal(s[0]) << 4) | hexVal(s[1]));
            s += 2;
            if (*s == '.') ++s;                       // can-utils allows dot-separated bytes
        }
    }
    // Whatever follows must be a separate token (candump -L adds a direction flag) or the end.
    if (*s != '\0' && *s != '\n' && *s != ' ' && *s != '\r') return ParseResult::BAD;
    return ParseResult::OK;
}

// ─── replay ───────────────────────────────────────────────────────────────────

struct Census {
    uint64_t count = 0;
    uint64_t rtr = 0;
    uint64_t decoded = 0;
    int64_t  firstUs = 0;
    int64_t  lastUs = 0;
    uint16_t dlcMask = 0;
};

struct Replay {
    Options        opt;
    FILE          *out = nullptr;
    VehicleSignals v;
    YawEstimator   y;
    bool           started = false;
    unsigned       segment = 0;
    int64_t        segT0Us = 0;
    uint32_t       segBaseMs = 1000u;
    int64_t        lastUs = 0;
    uint32_t       lastMs = 0;
    int64_t        periodUs = 100000;
    int64_t        nextGridUs = 0;
    uint64_t       rows = 0, clamped = 0;

    uint32_t msOf(int64_t tUs) const
    {
        return (uint32_t)(segBaseMs + (uint64_t)((tUs - segT0Us) / 1000));
    }

    void header()
    {
        fprintf(out, "time,segment,%sspeed_kmh,rpm,gear,pedal_raw,brake_pressed,brake_switch,turn_left,turn_right,"
                     "hazard,steer_torque_raw,yaw_rate_cdps_uncal,wheel_fl_raw,wheel_fr_raw,wheel_rl_raw,wheel_rr_raw\n",
                opt.everyFrame ? "id," : "");
    }

    static const char *gearName(VehGear g)
    {
        switch (g) {
            case VehGear::PARK:    return "P";
            case VehGear::REVERSE: return "R";
            case VehGear::NEUTRAL: return "N";
            case VehGear::DRIVE:   return "D";
            case VehGear::LOW:     return "L";
            case VehGear::SPORT:   return "S";
            default:               return "UNKNOWN";   // fresh, but a code the map's gearmap does not name
        }
    }

    void row(int64_t tUs, const Frame *f)
    {
        char tbuf[40];
        if (opt.everyFrame) snprintf(tbuf, sizeof(tbuf), "%lld.%06lld", (long long)(tUs / 1000000), (long long)(tUs % 1000000));
        else                snprintf(tbuf, sizeof(tbuf), "%lld.%03lld", (long long)(tUs / 1000000), (long long)((tUs % 1000000) / 1000));
        fprintf(out, "%s,%u,", tbuf, segment);
        if (f != nullptr) fprintf(out, f->ext ? "%08X," : "%03X,", (unsigned)f->id);

        // Empty = unavailable, exactly where the wire says so: a NONE source, NAN, or a sentinel.
        if (v.speedSrc != VehSource::NONE && !isnan(v.speedKmh)) fprintf(out, "%.2f", (double)v.speedKmh);
        fputc(',', out);
        if (v.rpmSrc != VehSource::NONE && !isnan(v.rpm)) fprintf(out, "%.0f", (double)v.rpm);
        fputc(',', out);
        if (v.gearSrc != VehSource::NONE) fputs(gearName(v.gear), out);
        fputc(',', out);
        if (v.pedalSrc != VehSource::NONE) fprintf(out, "%u", (unsigned)v.pedalGas);
        fputc(',', out);
        if (v.brakeSrc != VehSource::NONE) fprintf(out, "%d,%d", v.brakePressed ? 1 : 0, v.brakeSwitch ? 1 : 0);
        else                               fputc(',', out);
        fputc(',', out);
        if (v.turnSrc != VehSource::NONE) fprintf(out, "%d,%d", v.turnLeft ? 1 : 0, v.turnRight ? 1 : 0);
        else                              fputc(',', out);
        fputc(',', out);
        if (v.hazardSrc != VehSource::NONE) fprintf(out, "%d", v.hazard ? 1 : 0);
        fputc(',', out);
        if (v.steerSrc != VehSource::NONE && v.steerMotorTorque != VEH_TORQUE_INVALID) fprintf(out, "%u", (unsigned)v.steerMotorTorque);
        fputc(',', out);
        if (v.yawSrc != VehSource::NONE && v.yawRateCdps != INT16_MIN) fprintf(out, "%d", (int)v.yawRateCdps);
        for (uint8_t i = 0; i < VEH_WHEEL_COUNT; ++i) {
            fputc(',', out);
            if (v.wheelSrc != VehSource::NONE && v.wheelRaw[i] != VEH_WHEEL_INVALID) fprintf(out, "%u", (unsigned)v.wheelRaw[i]);
        }
        fputc('\n', out);
        ++rows;
    }

    /// The state a loop() pass at @p tUs would publish: holds at that instant, then expiry.
    void settleAt(int64_t tUs)
    {
        static const uint8_t none[8] = { 0 };
        const uint32_t ms = msOf(tUs);
        (void)canDecodeFrame(v, y, NO_MATCH_ID, 0u, none, ms);
        expireVehicleSignals(v, ms, VEH_FRESH_SNIFF_MS, VEH_FRESH_OBD2_MS);
    }

    void gridUpTo(int64_t tUs, bool inclusive)
    {
        if (opt.everyFrame) return;
        while (nextGridUs < tUs || (inclusive && nextGridUs == tUs)) {
            settleAt(nextGridUs);
            row(nextGridUs, nullptr);
            nextGridUs += periodUs;
        }
    }

    void startSegment(int64_t tUs)
    {
        ++segment;
        segBaseMs = started ? lastMs + SEGMENT_GAP_MS : 1000u;
        segT0Us   = tUs;
        // As applyCanMode() does on entering SNIFF: a fresh template and yaw state.
        initVehicleSignals(v);
        initYawEstimator(y, canSniffGetMap());
        // First grid instant at or after the segment's first frame, on multiples of the period.
        nextGridUs = ((tUs + periodUs - 1) / periodUs) * periodUs;
        started = true;
    }

    /// Places a frame on the decoder clock; returns its (possibly clamped) time.
    int64_t place(int64_t tUs)
    {
        if (!started) {
            startSegment(tUs);
        } else if (tUs < lastUs - BACKSTEP_US || tUs > lastUs + GAP_US) {
            gridUpTo(lastUs, true);              // the old segment's last instants
            startSegment(tUs);
        } else if (tUs < lastUs) {
            tUs = lastUs;                        // as tickCANSniff() clamps: never backwards
            ++clamped;
        }
        lastUs = tUs;
        lastMs = msOf(tUs);
        return tUs;
    }
};

int main(int argc, char **argv)
{
    Options opt;
    if (!parseArgs(argc, argv, opt)) { usage(stderr); return 2; }
    if (!loadMap(opt.map)) return 1;

    FILE *in = (!strcmp(opt.log, "-")) ? stdin : fopen(opt.log, "r");
    if (in == nullptr) { fprintf(stderr, "can_decode: %s: %s\n", opt.log, strerror(errno)); return 1; }

    Replay r;
    r.opt = opt;
    r.periodUs = (int64_t)llround(1e6 / opt.rate);
    if (r.periodUs < 1000) r.periodUs = 1000;
    r.out = opt.out ? fopen(opt.out, "w") : stdout;
    if (r.out == nullptr) { fprintf(stderr, "can_decode: %s: %s\n", opt.out, strerror(errno)); return 1; }
    r.header();

    std::map<uint64_t, Census> census;   // key: ext << 32 | id
    uint64_t lines = 0, frames = 0, stdData = 0, remote = 0, extended = 0, decoded = 0;
    uint64_t malformed = 0, fd = 0, otherIface = 0;
    char line[512];
    Frame f;
    while (fgets(line, sizeof(line), in) != nullptr) {
        ++lines;
        const ParseResult pr = parseCandump(line, f);
        if (pr == ParseResult::SKIP) continue;
        if (pr == ParseResult::FD)   { ++fd; continue; }
        if (pr == ParseResult::BAD) {
            if (++malformed <= 10) fprintf(stderr, "can_decode: line %llu malformed: %s", (unsigned long long)lines, line);
            continue;
        }
        if (opt.iface != nullptr && strcmp(opt.iface, f.iface) != 0) { ++otherIface; continue; }
        ++frames;

        const int64_t t = r.place(f.tUs);
        r.gridUpTo(t, false);

        Census &c = census[((uint64_t)(f.ext ? 1u : 0u) << 32) | f.id];
        if (c.count == 0) c.firstUs = t;
        ++c.count;
        c.lastUs = t;
        c.dlcMask = (uint16_t)(c.dlcMask | (1u << f.dlc));

        // As tickCANSniff(): remote and extended frames are taken and counted, never decoded.
        if (f.rtr)      { ++remote; ++c.rtr; continue; }
        if (f.ext)      { ++extended; continue; }
        ++stdData;
        if (canDecodeFrame(r.v, r.y, f.id, f.dlc, f.data, r.msOf(t))) {
            ++decoded;
            ++c.decoded;
            if (opt.everyFrame) {
                expireVehicleSignals(r.v, r.msOf(t), VEH_FRESH_SNIFF_MS, VEH_FRESH_OBD2_MS);
                r.row(t, &f);
            }
        }
    }
    if (r.started) r.gridUpTo(r.lastUs, true);
    if (in != stdin) fclose(in);
    if (r.out != stdout) fclose(r.out);

    // ── census ──
    FILE *cf = opt.census ? fopen(opt.census, "w") : stderr;
    if (cf == nullptr) { fprintf(stderr, "can_decode: %s: %s\n", opt.census, strerror(errno)); return 1; }
    fprintf(cf, "id,ext,count,remote,decoded,mean_rate_hz,first_time,last_time,dlcs,in_map\n");
    for (const auto &kv : census) {
        const bool ext = (kv.first >> 32) != 0u;
        const uint32_t id = (uint32_t)kv.first;
        const Census &c = kv.second;
        const double span = (double)(c.lastUs - c.firstUs) / 1e6;
        std::string dlcs;
        for (unsigned d = 0; d <= 8u; ++d) {
            if (c.dlcMask & (1u << d)) { if (!dlcs.empty()) dlcs += '|'; dlcs += (char)('0' + d); }
        }
        fprintf(cf, ext ? "%08X," : "%03X,", (unsigned)id);
        fprintf(cf, "%d,%llu,%llu,%llu,", ext ? 1 : 0, (unsigned long long)c.count, (unsigned long long)c.rtr,
                (unsigned long long)c.decoded);
        if (c.count > 1 && span > 0.0) fprintf(cf, "%.3f", (double)(c.count - 1) / span);
        fprintf(cf, ",%lld.%06lld,%lld.%06lld,%s,%d\n",
                (long long)(c.firstUs / 1000000), (long long)(c.firstUs % 1000000),
                (long long)(c.lastUs / 1000000), (long long)(c.lastUs % 1000000), dlcs.c_str(),
                (!ext && mapHasId(id)) ? 1 : 0);
    }
    if (cf != stderr) fclose(cf);

    const CanSignalMap &m = *canSniffGetMap();
    for (uint8_t i = 0; i < m.idCount; ++i) {
        if (census.find(m.id[i].canId) == census.end())
            fprintf(stderr, "can_decode: WARNING: map id %03X never appeared in the log\n", (unsigned)m.id[i].canId);
    }
    fprintf(stderr, "can_decode: %llu frames (%llu standard data, %llu remote, %llu extended), %llu decoded; "
                    "%llu malformed, %llu CAN FD skipped, %llu other-interface; %u segment(s), %llu clamped back-step(s); "
                    "%llu rows\n",
            (unsigned long long)frames, (unsigned long long)stdData, (unsigned long long)remote,
            (unsigned long long)extended, (unsigned long long)decoded, (unsigned long long)malformed,
            (unsigned long long)fd, (unsigned long long)otherIface, r.segment, (unsigned long long)r.clamped,
            (unsigned long long)r.rows);
    return malformed ? 3 : 0;
}
