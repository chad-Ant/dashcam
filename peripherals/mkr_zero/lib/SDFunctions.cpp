#include <SPI.h>
#include <string.h>
#include <stdlib.h>

#include "SDFunctions.h"

// ─── the card ─────────────────────────────────────────────────────────────────

/**
 * The onboard slot, on SPI1, addressed explicitly.
 *
 * ⚠️ SdFat does NOT pick SPI1 by itself here. Its `#define SDCARD_SPI SPI1` in
 * SdFatConfig.h sits inside `#if defined(__MK64FX512__) || defined(__MK66FX1M0__)`
 * — a Teensy 3.5/3.6 guard that never compiles on SAMD21. A bare begin(csPin)
 * would put this card on the MAIN SPI bus, shared with the MCP2515 at CS 3.
 * Grepping for SDCARD_SPI finds the define and suggests the opposite, which is
 * exactly the kind of wrong-and-convincing that vendor/SdFat/PATCHES.md exists
 * to record.
 *
 * SDCARD_SS_PIN (28) comes from the MKR Zero variant, NOT from SdFat and NOT
 * from DataDictionary.h's SD_CS_PIN (4).
 *
 * DEDICATED_SPI because nothing else on this board uses SPI1: the driver may
 * keep the card selected between operations. 12 MHz is comfortably inside spec
 * for the short onboard traces and leaves margin on a marginal card.
 */
static const SdSpiConfig kSdCfg(SDCARD_SS_PIN, DEDICATED_SPI, SD_SCK_MHZ(12), &SPI1);

static SdFat32 gSd;
static bool    gMounted = false;

SDReturnStatus initializeSD()
{
    if (gMounted) return SDReturnStatus::OK;   // idempotent
    if (!gSd.begin(kSdCfg)) return SDReturnStatus::NOK_INIT_FAILED;
    gMounted = true;
    return SDReturnStatus::OK;
}

bool sdReady() { return gMounted; }

bool sdOpenRead(const char *path, File32 &f)
{
    if (!gMounted || path == nullptr) return false;
    if (!gSd.exists(path))            return false;
    return f.open(path, O_RDONLY);
}

bool sdReadLine(File32 &f, char *buf, size_t bufLen)
{
    if (buf == nullptr || bufLen == 0) return false;
    buf[0] = '\0';

    size_t n    = 0;
    bool   any  = false;
    bool   over = false;

    for (;;) {
        const int c = f.read();
        if (c < 0) break;              // EOF
        any = true;
        if (c == '\n') break;          // end of line
        if (c == '\r') continue;       // CRLF: swallow, the \n ends it

        if (n + 1 < bufLen) {
            buf[n++] = (char)c;
        } else {
            // Over-long line: keep consuming to the newline but stop storing.
            // Truncating without draining would make the remainder look like a
            // fresh line, so one 500-byte junk line would become five bogus
            // ones - and a file with no newline at all would never terminate.
            over = true;
        }
    }

    buf[n] = '\0';
    (void)over;   // the caller sees truncation as a line that fails to parse
    return any;
}

// ─── config.txt ───────────────────────────────────────────────────────────────

void initSDConfigDefaults(SDConfig &config)
{
    config.defaultLat  = GPS_DEFAULT_LAT;
    config.defaultLon  = GPS_DEFAULT_LON;
    config.defaultAlt  = GPS_DEFAULT_ALT;
    config.defaultPacc = GPS_DEFAULT_PACC;
    config.timezone    = LOCAL_TIMEZONE;
    config.servoXPin   = SERVO_XAXIS_PIN;
    config.servoYPin   = SERVO_YAXIS_PIN;
    config.canCSPin    = MCP2515_DEFAULT_CS_PIN;
    config.canIntPin   = MCP2515_DEFAULT_INT_PIN;
}

/** @brief Trims leading and trailing ASCII whitespace in place. @return start. */
static char *trim(char *s)
{
    while (*s == ' ' || *s == '\t') ++s;
    size_t n = strlen(s);
    while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t')) s[--n] = '\0';
    return s;
}

SDReturnStatus readConfig(const char *filename, SDConfig &config)
{
    if (!gMounted) return SDReturnStatus::NOK_INIT_FAILED;

    File32 f;
    if (!sdOpenRead(filename, f)) return SDReturnStatus::NOK_NOT_FOUND;

    char     line[SD_MAX_LINE];
    uint16_t matched = 0;

    while (sdReadLine(f, line, sizeof(line))) {
        char *p = trim(line);
        if (*p == '\0' || *p == '#' || *p == ';') continue;
        if (*p == '[') continue;   // section headers are accepted and ignored

        char *eq = strchr(p, '=');
        if (eq == nullptr) continue;
        *eq = '\0';
        char *key = trim(p);
        char *val = trim(eq + 1);
        if (*key == '\0' || *val == '\0') continue;

        // strtod, not atof, and only here: this runs once at boot on a handful
        // of lines, so the software-double cost is paid in setup() rather than
        // anywhere the vehicle signals are decoded.
        if      (strcmp(key, "lat")      == 0) { config.defaultLat  = (float)strtod(val, nullptr); ++matched; }
        else if (strcmp(key, "lon")      == 0) { config.defaultLon  = (float)strtod(val, nullptr); ++matched; }
        else if (strcmp(key, "alt")      == 0) { config.defaultAlt  = (float)strtod(val, nullptr); ++matched; }
        else if (strcmp(key, "pacc")     == 0) { config.defaultPacc = (float)strtod(val, nullptr); ++matched; }
        else if (strcmp(key, "timezone") == 0) { config.timezone    = (int8_t)atoi(val);           ++matched; }
        else if (strcmp(key, "servo_x")  == 0) { config.servoXPin   = (uint8_t)atoi(val);          ++matched; }
        else if (strcmp(key, "servo_y")  == 0) { config.servoYPin   = (uint8_t)atoi(val);          ++matched; }
        else if (strcmp(key, "can_cs")   == 0) { config.canCSPin    = (uint8_t)atoi(val);          ++matched; }
        else if (strcmp(key, "can_int")  == 0) { config.canIntPin   = (uint8_t)atoi(val);          ++matched; }
    }

    f.close();
    // A file that parsed to nothing is reported rather than passed off as a
    // success with defaults: "the card holds a config.txt I could not read" and
    // "there is no config.txt" need different repairs.
    return (matched > 0) ? SDReturnStatus::OK : SDReturnStatus::NOK_PARSE_ERROR;
}
