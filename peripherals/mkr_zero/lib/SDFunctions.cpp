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

bool sdOpenRoot(File32 &dir)
{
    if (!gMounted) return false;
    return dir.open("/", O_RDONLY);
}

/// Suffix of the staging file. Fixed rather than derived so the name is
/// predictable to anyone inspecting a card after a power cut.
static const char kTmpSuffix[] = ".tmp";

SDReturnStatus sdWriteTextAtomic(const char *path, const char *text)
{
    if (!gMounted)                       return SDReturnStatus::NOK_INIT_FAILED;
    if (path == nullptr || text == nullptr) return SDReturnStatus::NOK_WRITE_FAILED;

    const size_t pathLen = strlen(path);
    char tmp[SD_MAX_LINE];
    if ((pathLen + sizeof(kTmpSuffix)) > sizeof(tmp)) return SDReturnStatus::NOK_WRITE_FAILED;
    memcpy(tmp, path, pathLen);
    memcpy(tmp + pathLen, kTmpSuffix, sizeof(kTmpSuffix));   // includes the NUL

    // A leftover staging file means a previous attempt died mid-write. Its
    // contents are worthless — the point of staging is that nothing reads it —
    // so it is removed rather than appended to.
    if (gSd.exists(tmp)) (void)gSd.remove(tmp);

    {
        File32 f;
        if (!f.open(tmp, O_WRONLY | O_CREAT | O_TRUNC)) return SDReturnStatus::NOK_WRITE_FAILED;

        const size_t len     = strlen(text);
        const size_t written = f.write(text, len);
        // sync() BEFORE close(), and both checked. close() flushes too, but its
        // return says only that the descriptor was released — a card that failed
        // to commit reports it here or nowhere.
        const bool   ok      = (written == len) && f.sync();
        f.close();
        if (!ok) {
            (void)gSd.remove(tmp);
            return SDReturnStatus::NOK_WRITE_FAILED;
        }
    }

    // The swap. FAT has no atomic replace, so this is remove-then-rename and
    // there is a window in which neither name exists. That window is the reason
    // the caller must treat a missing profile as ordinary rather than as
    // corruption: losing power inside it costs the stored calibration, which is
    // a warm-up, where the alternative design — writing over the live file —
    // costs a corrupt profile, which is a biased sensor.
    if (gSd.exists(path) && !gSd.remove(path)) {
        (void)gSd.remove(tmp);
        return SDReturnStatus::NOK_WRITE_FAILED;
    }
    if (!gSd.rename(tmp, path)) {
        (void)gSd.remove(tmp);
        return SDReturnStatus::NOK_WRITE_FAILED;
    }
    return SDReturnStatus::OK;
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
