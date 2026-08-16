#include <string.h>
#include <stdio.h>

#include "BNO055Calib.h"
#include "SDFunctions.h"

/// First offset register: ACC_OFFSET_X_LSB. The block runs to MAG_RADIUS_MSB.
#define BNO055_CALIB_FIRST_REG  0x55u

/// File format version. Bumped if the byte layout or the checksum ever changes,
/// so an old file is REFUSED rather than half-understood — the failure mode of
/// loading a mis-parsed profile is a silently biased sensor.
#define BNO055_CALIB_FILE_VER   1u

// ─── register access ──────────────────────────────────────────────────────────

bool bno055CalibRead(const BNO055InitState &state, uint8_t *out)
{
    if (out == nullptr || state.address == 0u) return false;
    // One burst: the 22 registers are contiguous, and a profile assembled from
    // several transactions could straddle a change and be internally
    // inconsistent. The part is in CONFIG so nothing should be moving, but the
    // burst costs nothing and removes the assumption.
    return bno055BusRead(state.address, BNO055_CALIB_FIRST_REG, out, BNO055_CALIB_BYTES) == 0;
}

bool bno055CalibWrite(const BNO055InitState &state, const uint8_t *in)
{
    if (in == nullptr || state.address == 0u) return false;

    // Byte at a time, and read back. The offsets decide what every subsequent
    // reading is measured against, so a write that silently did not land would
    // bias the sensor for the rest of the drive with nothing to show for it —
    // and this project has already been bitten once by exactly that, when a
    // whole configuration sequence was ignored because the part was on the wrong
    // register page.
    for (uint8_t i = 0; i < BNO055_CALIB_BYTES; ++i) {
        unsigned char v = in[i];
        if (bno055BusWrite(state.address, (uint8_t)(BNO055_CALIB_FIRST_REG + i), &v, 1u) != 0) {
            return false;
        }
    }

    uint8_t back[BNO055_CALIB_BYTES];
    if (!bno055CalibRead(state, back)) return false;
    return memcmp(back, in, BNO055_CALIB_BYTES) == 0;
}

/**
 * @brief Writes OPR_MODE, waits the switching time, and CONFIRMS it took.
 *
 * The confirmation is the point. A mode write can be acknowledged on the bus and
 * still not take — that is not hypothetical on this part, it is what thirty
 * bring-ups did while it sat on the wrong register page. Without the read-back,
 * a capture that failed to leave CONFIG returns success, the caller goes on
 * believing the sensor is running, and the part produces nothing for the rest of
 * the drive while every health flag says it is fine.
 */
static bool setModeVerified(const BNO055InitState &state, uint8_t mode)
{
    unsigned char v = mode;
    if (bno055BusWrite(state.address, BNO055_OPR_MODE_ADDR, &v, 1u) != 0) return false;
    // The datasheet's switching time, which the vendored driver does not honour
    // on its own. See vendor/BNO055/PATCHES.md.
    delay(BNO055_MODE_SWITCH_MS);

    unsigned char back = 0u;
    if (bno055BusRead(state.address, BNO055_OPR_MODE_ADDR, &back, 1u) != 0) return false;
    return (uint8_t)(back & 0x0Fu) == mode;
}

bool bno055CalibCapture(BNO055InitState &state, uint8_t *out)
{
    if (out == nullptr || state.address == 0u) return false;

    const uint8_t opMode = state.opMode;

    if (!setModeVerified(state, OPERATION_MODE_CONFIG)) {
        // Never entered CONFIG, so the offsets were never readable and the part
        // is still in its operating mode. Nothing to undo.
        return false;
    }

    const bool readOk = bno055CalibRead(state, out);

    // Restoration is attempted whether or not the read worked, because leaving
    // the part in CONFIG turns a failed save into a dead sensor — a far worse
    // outcome than not saving. Its result outranks the read's: a captured
    // profile is worthless if the sensor is no longer running.
    if (!setModeVerified(state, opMode)) return false;

    return readOk;
}

// ─── the file ─────────────────────────────────────────────────────────────────
//
// Text, not binary, and deliberately: it is inspectable on the card with any
// editor, it survives being copied between machines by tools that mangle line
// endings, and it is read by the same sdReadLine() every other parser here is
// built on. The whole profile is 22 bytes; the cost of hex is 22 bytes.

/**
 * @brief CRC-16/CCITT-FALSE over the profile and its install ID.
 *
 * An 8-bit additive sum stood here, copied from the CAN map's. It was the wrong
 * borrowing: the map is re-read and re-checked constantly and a bad row shows up
 * as a signal that never updates, whereas this is written straight into the
 * offset registers and then believed for the whole drive. A 1-in-256 chance of
 * accepting corruption buys a silently biased sensor, and an additive sum does
 * not notice transposed bytes at all — which is precisely what a partially
 * written file looks like.
 *
 * Same polynomial as the wire protocol, so there is one CRC in this codebase
 * rather than two.
 */
static uint16_t profileCrc(const uint8_t *p, uint16_t installId)
{
    uint16_t crc = 0xFFFFu;
    const uint8_t hdr[2] = { (uint8_t)(installId & 0xFFu), (uint8_t)(installId >> 8) };

    for (uint8_t i = 0; i < 2u + BNO055_CALIB_BYTES; ++i) {
        crc ^= (uint16_t)((i < 2u ? hdr[i] : p[i - 2u])) << 8;
        for (uint8_t b = 0; b < 8u; ++b) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/**
 * @brief Rejects offsets that no calibration could have produced.
 *
 * A CRC proves the file is intact, not that its contents are sane — a profile
 * captured from a sensor mid-failure is perfectly well-formed. The datasheet
 * bounds each field, so anything outside is refused rather than written into the
 * registers that every later reading is measured against.
 */
static bool profilePlausible(const uint8_t *p)
{
    // BOUNDS FROM THE DATASHEET, section 3.6.4 (docs/BST_BNO055_DS000-1509603.pdf).
    //
    // These were previously +/-2100 for accelerometer and gyroscope, described in
    // this comment as "taken generously". They were the opposite: TIGHTER than
    // the part can legitimately produce, so a valid profile could be refused and
    // the sensor would run uncalibrated with nothing to show for it.
    //
    //   Accelerometer (Table 3-16): the offset range follows the G-range in
    //   force when the profile was captured — +/-2000 mg at 2 g, rising to
    //   +/-16000 at 16 g, and 1 mg = 1 LSB (Table 3-17). Fusion locks the part
    //   at 4 g (so +/-4000) while this project's AMG mode selects 16 g. The
    //   check cannot know which mode captured the file, so it takes the widest.
    //
    //   Magnetometer (3.6.4.2): +/-6400 LSB, independent of range.
    //
    //   Gyroscope (Table 3-21): follows the dps range — +/-32000 at the 2000 dps
    //   the fusion modes run. This is a WEAK gate and worth saying so: against an
    //   int16 it rejects only 32001..32767. It still catches an all-ones block,
    //   which is the shape a garbage read takes, and there is nothing tighter
    //   that would not risk refusing a legitimate profile.
    for (uint8_t i = 0; i < 6u; i += 2u) {
        const int16_t a = (int16_t)(p[i]      | (p[i + 1]      << 8));   // accel
        const int16_t m = (int16_t)(p[6 + i]  | (p[7 + i]      << 8));   // mag
        const int16_t g = (int16_t)(p[12 + i] | (p[13 + i]     << 8));   // gyro
        if (a < -16000 || a > 16000) return false;
        if (m <  -6400 || m >  6400) return false;
        if (g < -32000 || g > 32000) return false;
    }
    // SIGNED, like every other field in the block. Read as uint16 they were
    // being compared against a positive bound, so a negative radius became a
    // number above 32000 and the profile was refused — a valid file rejected by
    // a type error rather than by anything about its contents.
    //
    // Table 3-24: accelerometer +/-1000 LSB, magnetometer +/-960. These are by
    // far the STRONGEST gates in this function — two orders of magnitude below
    // the encoding's range — which is the opposite of how they were treated
    // before, at a guessed +/-2048.
    const int16_t accRadius = (int16_t)(p[18] | (p[19] << 8));
    const int16_t magRadius = (int16_t)(p[20] | (p[21] << 8));
    if (accRadius < -1000 || accRadius > 1000) return false;
    if (magRadius <  -960 || magRadius >  960) return false;
    return true;
}

static int hexNibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

bool bno055CalibStore(const uint8_t *profile, uint16_t installId)
{
    if (profile == nullptr) return false;

    char text[224];
    int n = snprintf(text, sizeof(text),
                     "# BNO055 calibration offsets - regs 0x55..0x6A\n"
                     "# DELETE THIS FILE after replacing or remounting the sensor.\n"
                     "ver %u\n"
                     "install %04X\n"
                     "data ", (unsigned)BNO055_CALIB_FILE_VER, (unsigned)installId);
    if (n < 0 || (size_t)n >= sizeof(text)) return false;

    for (uint8_t i = 0; i < BNO055_CALIB_BYTES; ++i) {
        const int m = snprintf(text + n, sizeof(text) - (size_t)n, "%02X", (unsigned)profile[i]);
        if (m < 0 || (size_t)(n + m) >= sizeof(text)) return false;
        n += m;
    }
    const int m = snprintf(text + n, sizeof(text) - (size_t)n,
                           "\ncrc %04X\n", (unsigned)profileCrc(profile, installId));
    if (m < 0 || (size_t)(n + m) >= sizeof(text)) return false;

    return sdWriteTextAtomic(BNO055_CALIB_PATH, text) == SDReturnStatus::OK;
}

bool bno055CalibLoad(uint8_t *out, uint16_t *installId)
{
    if (out == nullptr) return false;

    File32 f;
    // A missing file is the ordinary state of a rig that has never been
    // calibrated. Reported as a plain false, not as an error, so the caller has
    // nothing to distinguish and nothing to log loudly.
    if (!sdOpenRead(BNO055_CALIB_PATH, f)) return false;

    // SIZE FIRST, BEFORE PARSING A BYTE OF IT.
    //
    // This runs with the watchdog already armed and does not feed it. The parse
    // below reads to EOF, and sdReadLine() drains an over-long line to its
    // newline rather than truncating — correct for its own job, and unbounded
    // in file size. A multi-megabyte file at this path, from a corrupted card or
    // simply dropped there, therefore spends longer inside this function than
    // the watchdog period, and the next boot parses exactly the same file: a
    // reboot loop with no way out but a card reader.
    //
    // bno055CalibStore() writes well under 224 bytes, so anything past a
    // generous multiple of that is not a profile this firmware produced,
    // whatever its contents turn out to be.
    if (f.fileSize() > BNO055_CALIB_MAX_FILE_BYTES) { f.close(); return false; }

    uint8_t  profile[BNO055_CALIB_BYTES];
    bool     haveData = false;
    bool     haveCrc  = false;
    unsigned ver      = 0;
    unsigned install  = 0;
    unsigned crc      = 0;

    char line[SD_MAX_LINE];
    while (sdReadLine(f, line, sizeof(line))) {
        if (line[0] == '#' || line[0] == '\0') continue;

        if (strncmp(line, "ver ", 4) == 0) {
            ver = (unsigned)strtoul(line + 4, nullptr, 10);
        } else if (strncmp(line, "install ", 8) == 0) {
            install = (unsigned)strtoul(line + 8, nullptr, 16);
        } else if (strncmp(line, "crc ", 4) == 0) {
            crc = (unsigned)strtoul(line + 4, nullptr, 16);
            haveCrc = true;
        } else if (strncmp(line, "data ", 5) == 0) {
            const char *p = line + 5;
            // Length checked BEFORE parsing, so a truncated line is rejected
            // rather than yielding a profile whose tail is whatever the stack
            // held. A short read here is exactly what a power cut mid-write
            // would produce if the write were not atomic.
            if (strlen(p) != (size_t)(BNO055_CALIB_BYTES * 2u)) { f.close(); return false; }
            for (uint8_t i = 0; i < BNO055_CALIB_BYTES; ++i) {
                const int hi = hexNibble(p[i * 2]);
                const int lo = hexNibble(p[i * 2 + 1]);
                if (hi < 0 || lo < 0) { f.close(); return false; }
                profile[i] = (uint8_t)((hi << 4) | lo);
            }
            haveData = true;
        }
    }
    f.close();

    if (!haveData || !haveCrc)        return false;
    if (ver != BNO055_CALIB_FILE_VER) return false;
    if (profileCrc(profile, (uint16_t)install) != (uint16_t)crc) return false;
    // Intact is not the same as sane. A profile captured from a sensor that was
    // already misbehaving CRCs perfectly.
    if (!profilePlausible(profile))   return false;

    memcpy(out, profile, BNO055_CALIB_BYTES);
    if (installId != nullptr) *installId = (uint16_t)install;
    return true;
}

bool bno055CalibEqual(const uint8_t *a, const uint8_t *b)
{
    if (a == nullptr || b == nullptr) return false;
    return memcmp(a, b, BNO055_CALIB_BYTES) == 0;
}

void bno055CalibDescribe(const uint8_t *profile, char *buf, size_t bufLen)
{
    if (buf == nullptr || bufLen == 0) return;
    buf[0] = '\0';
    if (profile == nullptr) return;

    // The three offset vectors as signed 16-bit, which is what they are and what
    // a technician comparing two profiles wants to see. The radii are omitted:
    // they are fusion internals with no bench interpretation.
    const int16_t ax = (int16_t)(profile[0]  | (profile[1]  << 8));
    const int16_t ay = (int16_t)(profile[2]  | (profile[3]  << 8));
    const int16_t az = (int16_t)(profile[4]  | (profile[5]  << 8));
    const int16_t gx = (int16_t)(profile[12] | (profile[13] << 8));
    const int16_t gy = (int16_t)(profile[14] | (profile[15] << 8));
    const int16_t gz = (int16_t)(profile[16] | (profile[17] << 8));

    snprintf(buf, bufLen, "accel %d,%d,%d  gyro %d,%d,%d",
             (int)ax, (int)ay, (int)az, (int)gx, (int)gy, (int)gz);
}
