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

bool bno055CalibCapture(BNO055InitState &state, uint8_t *out)
{
    if (out == nullptr || state.address == 0u) return false;

    const uint8_t opMode = state.opMode;
    unsigned char cfg    = OPERATION_MODE_CONFIG;

    if (bno055BusWrite(state.address, BNO055_OPR_MODE_ADDR, &cfg, 1u) != 0) return false;
    // The datasheet's operation->CONFIG figure, which the driver does not honour
    // on its own. See vendor/BNO055/PATCHES.md.
    delay(BNO055_MODE_SWITCH_MS);

    const bool ok = bno055CalibRead(state, out);

    // Restored whether or not the read worked. Leaving the part in CONFIG after
    // a failed capture would stop it producing data entirely — turning a failed
    // save into a dead sensor, which is a far worse outcome than not saving.
    unsigned char back = opMode;
    if (bno055BusWrite(state.address, BNO055_OPR_MODE_ADDR, &back, 1u) != 0) return false;
    delay(BNO055_MODE_SWITCH_MS);

    return ok;
}

// ─── the file ─────────────────────────────────────────────────────────────────
//
// Text, not binary, and deliberately: it is inspectable on the card with any
// editor, it survives being copied between machines by tools that mangle line
// endings, and it is read by the same sdReadLine() every other parser here is
// built on. The whole profile is 22 bytes; the cost of hex is 22 bytes.

/** @brief Sum-based checksum, matching the CAN map's. */
static uint8_t profileChecksum(const uint8_t *p, uint8_t chipId)
{
    uint16_t sum = chipId;
    for (uint8_t i = 0; i < BNO055_CALIB_BYTES; ++i) sum += p[i];
    return (uint8_t)(sum & 0xFFu);
}

static int hexNibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

bool bno055CalibStore(const uint8_t *profile, uint8_t chipId)
{
    if (profile == nullptr) return false;

    // Three short lines and the hex. Sized for the longest: 44 hex characters
    // plus its key and terminator.
    char text[160];
    int n = snprintf(text, sizeof(text),
                     "# BNO055 calibration offsets - regs 0x55..0x6A\n"
                     "ver %u\n"
                     "chip %02X\n"
                     "data ", (unsigned)BNO055_CALIB_FILE_VER, (unsigned)chipId);
    if (n < 0 || (size_t)n >= sizeof(text)) return false;

    for (uint8_t i = 0; i < BNO055_CALIB_BYTES; ++i) {
        const int m = snprintf(text + n, sizeof(text) - (size_t)n, "%02X", (unsigned)profile[i]);
        if (m < 0 || (size_t)(n + m) >= sizeof(text)) return false;
        n += m;
    }
    const int m = snprintf(text + n, sizeof(text) - (size_t)n,
                           "\nsum %02X\n", (unsigned)profileChecksum(profile, chipId));
    if (m < 0 || (size_t)(n + m) >= sizeof(text)) return false;

    return sdWriteTextAtomic(BNO055_CALIB_PATH, text) == SDReturnStatus::OK;
}

bool bno055CalibLoad(uint8_t *out, uint8_t *chipId)
{
    if (out == nullptr) return false;

    File32 f;
    // A missing file is the ordinary state of a rig that has never been
    // calibrated. Reported as a plain false, not as an error, so the caller has
    // nothing to distinguish and nothing to log loudly.
    if (!sdOpenRead(BNO055_CALIB_PATH, f)) return false;

    uint8_t  profile[BNO055_CALIB_BYTES];
    bool     haveData = false;
    bool     haveSum  = false;
    unsigned ver      = 0;
    unsigned chip     = 0;
    unsigned sum      = 0;

    char line[SD_MAX_LINE];
    while (sdReadLine(f, line, sizeof(line))) {
        if (line[0] == '#' || line[0] == '\0') continue;

        if (strncmp(line, "ver ", 4) == 0) {
            ver = (unsigned)strtoul(line + 4, nullptr, 10);
        } else if (strncmp(line, "chip ", 5) == 0) {
            chip = (unsigned)strtoul(line + 5, nullptr, 16);
        } else if (strncmp(line, "sum ", 4) == 0) {
            sum = (unsigned)strtoul(line + 4, nullptr, 16);
            haveSum = true;
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

    if (!haveData || !haveSum)              return false;
    if (ver != BNO055_CALIB_FILE_VER)       return false;
    if (profileChecksum(profile, (uint8_t)chip) != (uint8_t)sum) return false;

    memcpy(out, profile, BNO055_CALIB_BYTES);
    if (chipId != nullptr) *chipId = (uint8_t)chip;
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
