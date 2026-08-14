/**
 * CANDiscovery - passive CAN bus census for the MKR Zero + MCP2515.
 *
 * Answers the two questions that gate the whole CAN-sniffing effort:
 *   1. Does this tap point carry raw broadcast frames at all, or only
 *      diagnostic (0x7E8) responses?  On an OBD-II pins 6/14 tap the gateway
 *      may bridge nothing, in which case sniffing is impossible from there.
 *   2. Which CAN IDs does THIS car actually use, and which bytes inside them
 *      carry the signals we want?
 *
 * It also cross-checks the decoded wheel speed against GNSS, which answers a
 * question the CAN bus cannot answer about itself: a bus tells you counts, and
 * only an independent reference tells you what a count is worth.
 *
 * It MODIFIES no project source, but it does now READ lib\ - it reuses the
 * project's GNSS stack rather than growing a second one. Building it therefore
 * needs the same --library flags as the production firmware; see
 * BuildAndUpload.cmd.  Same role as I2CAddressScanner.ino next door.
 *
 * ── SAFETY ───────────────────────────────────────────────────────────────────
 * This sketch is STRICTLY READ-ONLY on the bus.  It never calls beginPacket() /
 * endPacket() and it runs the MCP2515 in Listen-Only mode, so it emits neither
 * ACK bits nor error frames.  Driving either onto a live vehicle bus can set a
 * VSA/ABS fault light.
 *
 * It does NOT use CAN.observe(): in the installed library that writes CANCTRL
 * 0x80, which is CONFIGURATION mode, not Listen-Only (the library's own source
 * carries a "TODO: These should probably be 0x60, not 0x80" beside it).  In
 * Configuration mode the controller is off the bus and receives nothing at all,
 * which would look exactly like "the gateway blocks broadcast traffic" and send
 * you chasing a hardware fault that isn't there.  Mode is set by raw SPI here
 * and verified by reading CANSTAT, not by trusting the CANCTRL readback.
 *
 * ── WIRING / HARDWARE ────────────────────────────────────────────────────────
 * MCP2515 on SPI, CS = D3, INT = D7, 16 MHz crystal, 500 kbps.
 * WARNING: the MKR Zero is NOT 5 V tolerant.  Generic "MCP2515 + TJA1050"
 * modules run at 5 V and drive MISO and INT at 5 V.  Confirm your module is a
 * 3.3 V variant or level-shifted before connecting anything.
 *
 * ── USE ──────────────────────────────────────────────────────────────────────
 * Build/flash with BuildAndUpload.cmd, open the serial monitor at 115200.
 * A table prints every 5 s.  Keys: r reset, d dump, h help, fNNN focus, m mark.
 *
 * Identification is DIFFERENTIAL - capture, change one thing, capture again:
 *   idle vs 2500 rpm ............ finds the rpm bytes
 *   stationary vs rolling ....... finds speed / wheel speeds
 *   lock-to-lock steering sweep . finds steering angle
 *   P -> R -> N -> D ............ finds gear position
 *
 * Two tools do that, for two different shapes of signal:
 *
 *   "changed (window)" - bits that moved during THIS report interval only.
 *   Hold one state for five seconds and only the bits belonging to it light up.
 *   Use the neighbouring "changed (all run)" column solely to ask whether a
 *   byte is ever alive; it saturates early and then answers nothing, which is
 *   how an ignition-on transition can make a byte look permanently active.
 *
 *   Focus min/max - for signals identified by MOTION rather than by state.
 *   A steering sweep has no "before and after" snapshot to compare, so arm
 *   focus on a candidate ID, press m, make one slow lock-to-lock pass, and read
 *   the span of each byte pair. The swept field's span dwarfs every other.
 */

#include <CAN.h>
#include <SPI.h>
#include <Wire.h>
// SdFat, not the Arduino SD library. The production firmware moved to SdFat
// when lib/SDFunctions.cpp was implemented, and lib/ is compiled into this
// sketch too — two filesystem stacks in one binary is waste at best. See
// vendor/SdFat/PATCHES.md for why the SPI port must be named explicitly.
#include <SdFat.h>

// The project's own GNSS stack, reused rather than reimplemented. It carries
// bring-up behaviour this sketch has no business duplicating - a non-blocking
// staged init with backoff, a quarantine path for a wedged bus, and the retry
// that initializeGPS_I2C() needs because it fails intermittently. A second,
// simpler GNSS path written for a helper sketch would be a second thing that
// can be wrong about the receiver, and it is the receiver we are trusting to
// calibrate the CAN decode.
#include "DataDictionary.h"
#include "I2CBus.h"
#include "GPSFunctions.h"

#include "CANTypes.h"   // RawFrame / CanIdStat — see the header for why they live there

// ─── configuration (mirrors DataDictionary.h; standalone so nothing is shared) ─

static const int  CS_PIN    = 3;
static const int  INT_PIN   = 7;
static const long CLOCK_HZ  = 16000000L;   // MUST match the module crystal
static const long BITRATE   = 500000L;     // Honda F-CAN
static const uint32_t REPORT_INTERVAL_MS = 5000;

/// Frames drained per loop() pass. Bounds the worst-case pass; leftovers are
/// picked up next time.
static const uint8_t MAX_FRAMES_PER_POLL = 16;

/// Distinct IDs tracked. 64 x 44 B = 2.8 KB of the MKR Zero's 32 KB.
static const uint8_t CENSUS_SLOTS = 64;

/// Bounded wait for a USB host, so the sketch still runs headless.
static const uint32_t SERIAL_WAIT_MS = 2000;

static const SPISettings SPICfg(10000000, MSBFIRST, SPI_MODE0);

// ─── MCP2515 registers and instructions ───────────────────────────────────────

static const uint8_t REG_CANSTAT  = 0x0E;
static const uint8_t REG_CANCTRL  = 0x0F;
static const uint8_t REG_TEC      = 0x1C;
static const uint8_t REG_REC      = 0x1D;
static const uint8_t REG_CANINTF  = 0x2C;
static const uint8_t REG_EFLG     = 0x2D;
static const uint8_t REG_RXB0CTRL = 0x60;
static const uint8_t REG_RXB1CTRL = 0x70;

static const uint8_t INSTR_WRITE       = 0x02;
static const uint8_t INSTR_READ        = 0x03;
static const uint8_t INSTR_BITMOD      = 0x05;
static const uint8_t INSTR_READ_RXB0   = 0x90; // from RXB0SIDH; auto-clears RX0IF on CS rise
static const uint8_t INSTR_READ_RXB1   = 0x94; // from RXB1SIDH; auto-clears RX1IF on CS rise

static const uint8_t MODE_CONFIG      = 0x80; // CANCTRL REQOP = 100
static const uint8_t MODE_LISTEN_ONLY = 0x60; // CANCTRL REQOP = 011
static const uint8_t OPMOD_MASK       = 0xE0; // CANSTAT bits 7:5

static const uint8_t EFLG_RX0OVR = 0x40;
static const uint8_t EFLG_RX1OVR = 0x80;

// ─── raw SPI helpers ──────────────────────────────────────────────────────────
// The library keeps its register accessors private, so mode/filter/diagnostic
// registers are reached directly. Safe alongside the library: every one of its
// accessors brackets its own transaction and releases CS between calls.

static uint8_t mcpRead(uint8_t reg)
{
    SPI.beginTransaction(SPICfg);
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(INSTR_READ);
    SPI.transfer(reg);
    const uint8_t v = SPI.transfer(0x00);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
    return v;
}

static void mcpWrite(uint8_t reg, uint8_t value)
{
    SPI.beginTransaction(SPICfg);
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(INSTR_WRITE);
    SPI.transfer(reg);
    SPI.transfer(value);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
}

static void mcpBitModify(uint8_t reg, uint8_t mask, uint8_t value)
{
    SPI.beginTransaction(SPICfg);
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(INSTR_BITMOD);
    SPI.transfer(reg);
    SPI.transfer(mask);
    SPI.transfer(value);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
}

/**
 * @brief Requests an operating mode and confirms it actually took effect.
 *
 * CANCTRL only echoes the REQUEST; the achieved mode lives in CANSTAT OPMOD,
 * and a mode change waits for any in-progress frame to finish. Polling CANSTAT
 * is the only way to know. Bounded so a wedged controller cannot hang setup().
 */
static bool mcpSetMode(uint8_t mode)
{
    mcpWrite(REG_CANCTRL, mode);
    for (uint8_t tries = 0; tries < 50; ++tries) {   // ~50 ms ceiling
        if ((mcpRead(REG_CANSTAT) & OPMOD_MASK) == mode) return true;
        delay(1);
    }
    return false;
}

// ─── received frame ───────────────────────────────────────────────────────────

/**
 * @brief Reads one RX buffer with a single READ RX BUFFER instruction.
 *
 * One CS pair and 14 bytes, versus the library's parsePacket() which issues
 * ~15 separate register reads for the same frame. At 500 kbps an 8-byte frame
 * occupies the bus for only ~222 us, so the cheaper read is what makes keeping
 * up with a busy bus possible. The instruction also auto-clears RXnIF on the
 * CS rising edge, so no follow-up write is needed.
 */
static void mcpReadFrame(uint8_t instr, RawFrame &f)
{
    uint8_t b[13];
    SPI.beginTransaction(SPICfg);
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(instr);
    for (uint8_t i = 0; i < 13; ++i) b[i] = SPI.transfer(0x00);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();

    const uint8_t sidh = b[0];
    const uint8_t sidl = b[1];

    f.extended = (sidl & 0x08) != 0;             // IDE
    f.id  = ((uint16_t)sidh << 3) | (sidl >> 5); // 11-bit standard identifier
    f.dlc = b[4] & 0x0F;
    if (f.dlc > 8) f.dlc = 8;                    // malformed DLC guard
    memcpy(f.data, &b[5], 8);

    // Zero everything past the DLC. The MCP2515 RX buffer is 8 bytes wide and
    // retains whatever a PREVIOUS frame left beyond the current frame's length,
    // so a DLC-3 message carries five bytes of stale rubbish. Folded into the
    // OR/AND masks that rubbish reads as "these bits keep changing", which is
    // exactly the signal the change mask exists to report - it made every short
    // message on the bus look like it was carrying live data.
    for (uint8_t i = f.dlc; i < 8; ++i) f.data[i] = 0;
}

// ─── census table ─────────────────────────────────────────────────────────────

static CanIdStat gStat[CENSUS_SLOTS];
static uint8_t   gUsed;              ///< slots occupied
static uint16_t  gSlotOverflow;      ///< frames whose ID found no free slot
static uint32_t  gTotalFrames;
static uint32_t  gExtendedFrames;    ///< 29-bit frames (unexpected on Honda F-CAN)
static uint32_t  gRxOverruns;        ///< EFLG RX0OVR|RX1OVR events
static uint32_t  gWindowStartMs;
static uint32_t  gRateBaseMs;        ///< start of the CURRENT report interval
static uint32_t  gIntAsserted;       ///< frames pending with INT pin LOW  (INT wired)
static uint32_t  gIntMissed;         ///< frames pending with INT pin HIGH (INT NOT wired)
// ─── ID-space sweep ───────────────────────────────────────────────────────────
//
// The census table is not the limit on what this tool can find: 39 of 64 slots
// used and slotOverflow 0 means no identifier was ever turned away. The limit is
// the DRAIN. About 1100 frames/s arrive, roughly 90 % of it a dozen 50-100 Hz
// powertrain messages in 0x100-0x1FF, and the RX buffers overflow ~3 times a
// second under that load. Every overrun discards a frame chosen by timing, not
// by importance — so a message that appears once a minute (a door, a reverse
// lamp, a fault) can be lost outright, and its absence looks exactly like it
// was never sent.
//
// Filtering fixes that by taking the load off — and MASK GROUPING means almost
// nothing has to be given up to do it.
//
// The two masks are independent and need not be the same WIDTH. MASK0 governs
// RXB0 (filters 0-1); MASK1 governs RXB1 (filters 2-5). Set MASK1 coarse
// (0x600, two bits) and each of its filters selects a PAIR of slices; set MASK0
// fine (0x700, three bits) and its filter selects a single slice. The complement
// of any one slice is exactly three pairs plus one single — which is precisely
// what these six filters can express:
//
//     MASK1 0x600 -> three slice PAIRS   (six slices)
//     MASK0 0x700 -> the excluded slice's PARTNER (one slice)
//                                        = seven of eight, in one configuration
//
// So instead of admitting one slice at a time and rotating, the census admits
// EVERYTHING EXCEPT the busiest slice, continuously. On this vehicle that one
// slice is 0x100-0x1FF and carries 883 of 1058 Hz: excluding it removes 83 % of
// the load, and the rest of the identifier space becomes quiet enough to receive
// losslessly. Nothing is time-shared, so a message that fires once a minute
// cannot fall in a gap between dwells.
static const uint16_t SWEEP_MASK_FINE   = 0x700;  ///< Selects one 256-id slice.
static const uint16_t SWEEP_MASK_COARSE = 0x600;  ///< Selects an aligned slice PAIR.
static const uint8_t  SWEEP_SLICES      = 8;

/** @brief What the hardware filter is currently doing. */
enum SweepMode : uint8_t {
    SWEEP_OFF = 0,  ///< RXM=11, accept everything. The default census.
    SWEEP_GROUPED,  ///< Seven slices admitted; the busiest excluded.
    SWEEP_PARK      ///< Exactly one slice admitted.
};

static SweepMode gSweepMode  = SWEEP_OFF;
static uint8_t   gSweepSlice = 0;   ///< Excluded slice (GROUPED) or admitted one (PARK).
static uint32_t  gSweepFrames[SWEEP_SLICES];   ///< Frames seen per slice while filtered.
/**
 * Overruns under the active filter configuration — ONE counter, not per slice.
 *
 * An overrun is a buffer event, not an identifier event: the frame that was
 * dropped is by definition the one never read, so its ID is unknowable. Under
 * grouping, seven slices share the two buffers, and charging the loss to any
 * one of them invents information. It was previously indexed by gSweepSlice,
 * which in grouped mode is the EXCLUDED slice — so every admitted row printed
 * "no loss" while the excluded one accumulated a count for traffic it never saw.
 */
static uint32_t  gSweepOverrunsCfg;

static uint16_t  gFocusId = 0xFFFF;  ///< stream every frame of this ID live; 0xFFFF = off

// ─── focus-mode extremes ──────────────────────────────────────────────────────
//
// A steering sweep is a MOTION, and neither the 5 s snapshot nor a change mask
// can express one: the snapshot shows a single arbitrary instant, and the mask
// says only "these bits moved", not how far. Extremes do express it. Sweeping
// the wheel lock to lock drives the real angle field across most of its range
// while every unrelated field stays put, so ranking fields by span identifies
// it in a single pass - no reading a 100 Hz scroll in real time, and no need to
// press a key at exactly the right moment.
//
// Tracked on EVERY matching frame; only the live printout is decimated.

static uint32_t gFocusFrames;
static uint8_t  gFocusDlc;
static uint8_t  gFocusByteMin[8];
static uint8_t  gFocusByteMax[8];
/**
 * Every ADJACENT big-endian byte pair, read as signed: b0:1, b1:2, ... b6:7.
 *
 * Seven, not the four aligned ones. Real 16-bit fields are not guaranteed to
 * start on an even byte, and only scanning even starts hides every odd-aligned
 * one. The cost is three extra lines of output for a single focused ID.
 *
 * The last pair is a trap and is labelled as one: on Honda the final byte is
 * COUNTER (bits 5:4) + CHECKSUM (bits 3:0), so any pair including it is a data
 * byte glued to a free-running nibble. It produces a large, entirely fictitious
 * span - 0x17C read "b6:7 span 8255" when byte 6 is a single brake bit.
 */
static int16_t  gFocusPairMin[7];
static int16_t  gFocusPairMax[7];
static uint32_t gFocusLastPrintMs;

/// Live focus lines are decimated to this, so a 100 Hz ID cannot swamp the link
/// or stall the drain. The min/max above still see every single frame.
static const uint32_t FOCUS_PRINT_INTERVAL_MS = 100;

static void focusReset()
{
    gFocusFrames = 0;
    gFocusDlc    = 0;
    for (uint8_t i = 0; i < 8; ++i) { gFocusByteMin[i] = 0xFF;  gFocusByteMax[i] = 0x00;   }
    for (uint8_t p = 0; p < 7; ++p) { gFocusPairMin[p] = 32767; gFocusPairMax[p] = -32768; }
}

static void focusTrack(const RawFrame &f)
{
    ++gFocusFrames;
    gFocusDlc = f.dlc;

    for (uint8_t i = 0; i < f.dlc; ++i) {
        if (f.data[i] < gFocusByteMin[i]) gFocusByteMin[i] = f.data[i];
        if (f.data[i] > gFocusByteMax[i]) gFocusByteMax[i] = f.data[i];
    }
    for (uint8_t p = 0; (uint8_t)(p + 1) < f.dlc; ++p) {   // adjacent, not aligned
        const int16_t v = (int16_t)(((uint16_t)f.data[p] << 8) | f.data[p + 1]);
        if (v < gFocusPairMin[p]) gFocusPairMin[p] = v;
        if (v > gFocusPairMax[p]) gFocusPairMax[p] = v;
    }
}

// ─── SD logging ───────────────────────────────────────────────────────────────
//
// The calibration run happens while DRIVING, where a tethered laptop is not an
// option, so a capture that only exists on a serial terminal is a capture that
// cannot be taken. That is the whole reason this exists.
//
// Not a second rendering of the report: the same bytes go to both sinks at once
// through TeePrint below. Rendering twice would mean the file and the terminal
// could disagree, and the file is the copy nobody can re-read at the time.
//
// No bus contention with the MCP2515. The MKR Zero's onboard card is on SPI1
// (variant.h: SDCARD_SPI = SPI1) while the CAN controller is on SPI, so an SD
// write and a CAN register read cannot interleave on the same peripheral. They
// still compete for CPU, which is why the write duration is measured.

/// How often the report is also written to the card.
static const uint32_t SD_DUMP_INTERVAL_MS = 60000UL;

static bool     gSdOk = false;
static SdFat32  gSd;
static File32   gSdFile;
static char     gSdName[13];        ///< 8.3, the only form the SD library accepts
static uint32_t gSdDumps;
static uint32_t gSdWriteMaxUs;      ///< worst drain stall caused by a card write
static uint32_t gLastSdDumpMs;

/**
 * @brief Writes to two sinks at once, so the file is the terminal's exact twin.
 *
 * Returns the first sink's count, not the second's: the console is the sink
 * whose success the caller can already see. A card that has been pulled makes
 * File::write() return 0, and propagating that would abort the console output
 * too - losing the copy that still works in order to report the loss of the one
 * that does not.
 */
class TeePrint : public Print {
public:
    TeePrint(Print &primary, Print &copy) : m_primary(primary), m_copy(copy) {}

    size_t write(uint8_t c) override
    {
        m_copy.write(c);
        return m_primary.write(c);
    }

    size_t write(const uint8_t *buf, size_t n) override
    {
        m_copy.write(buf, n);
        return m_primary.write(buf, n);
    }

private:
    Print &m_primary;
    Print &m_copy;
};

/**
 * @brief Opens the next unused CANLOGnn.TXT.
 *
 * Numbered rather than timestamped because the sketch has no trustworthy clock
 * at the moment the file is created: GNSS bring-up has not finished, and the
 * RTC is unset on a cold boot. A wrong timestamp on a forensic log is worse
 * than no timestamp, so the UTC time is written INSIDE the file once the
 * receiver supplies one, and the name is only ever used to tell runs apart.
 */
static bool sdBegin()
{
    // Explicit port: SdFat's own SDCARD_SPI define is Teensy-guarded and does
    // NOT apply on SAMD21, so a bare begin() would put the card on the main SPI
    // bus alongside the MCP2515 — exactly the contention this sketch's comments
    // claim it avoids.
    static const SdSpiConfig cfg(SDCARD_SS_PIN, DEDICATED_SPI, SD_SCK_MHZ(12), &SPI1);
    if (!gSd.begin(cfg)) return false;

    for (uint8_t n = 0; n < 100; ++n) {
        snprintf(gSdName, sizeof(gSdName), "CANLOG%02u.TXT", n);
        if (!gSd.exists(gSdName)) break;
    }

    gSdFile.open(gSdName, O_WRONLY | O_CREAT | O_APPEND);
    return (bool)gSdFile;
}

// ─── GNSS cross-check ─────────────────────────────────────────────────────────
//
// What this answers: the SCALE of the wheel-speed field. The bit LAYOUT is
// already settled without GNSS - opendbc's four unaligned 15-bit fields decode
// a real captured frame (02 5E 04 BC 09 78 13 59) to 303/303/303/309 counts,
// and four wheels on a car going straight must agree. Reading the same bytes as
// aligned 16-bit pairs gives 606/1212/2424/4953, which no vehicle produces. So
// the fields are right; what is unverified is whether a count is 0.01 km/h.
//
// The method is the one NIJ 311503 used against a VBOX: divide raw counts by a
// GPS speed and see what the constant comes out to. If a count is 0.01 km/h the
// answer is 100 counts per km/h. Any other number is the real scale.
//
// Cost, stated plainly: a GNSS poll is an I2C transaction on a shared 400 kHz
// bus and stalls the CAN drain for milliseconds. At ~1100 frames/s with an RX
// depth of 2, that overruns. Overruns do not bias the ratio - they drop frames,
// they do not corrupt the ones that arrive - but they do make a census taken
// during a calibration run incomplete, which is why the poll duration is
// measured and reported rather than left for someone to wonder about.

/** ID carrying the four wheel speeds - confirmed on this vehicle, see the plan. */
static const uint16_t WHEEL_SPEED_ID = 0x1D0;

/// Ignore samples below this: at a crawl the GPS speed error is a large
/// fraction of the reading, and the ratio it produces is mostly noise.
static const float CAL_MIN_KMH = 15.0f;

/// Ignore samples where the speed changed more than this since the previous
/// one. GNSS position fixes lag the wheel by ~0.1-0.2 s, so under acceleration
/// the two sides of the ratio are not the same instant. Steady state removes
/// the lag from the measurement entirely instead of trying to correct for it.
static const float CAL_MAX_DELTA_KMH = 1.0f;

static SFE_UBLOX_GNSS myGNSS;
static GPSInitState   gpsInit;
static GPSData        gpsData;
static bool           gpsReady    = false;
static bool           gpsEnabled  = true;   ///< 'g' toggles; off keeps the drain clean
static uint32_t       gGpsPolls;
static uint32_t       gGpsPollMaxUs;        ///< worst drain stall caused by a poll
static uint32_t       gLastGpsPollMs;
static uint32_t       gLastGpsFreshMs;

/// Latest decoded wheel speeds, in raw counts. Snapshotted in the drain rather
/// than read back out of the census table, so the calibrator always pairs the
/// GPS sample with a frame it knows the age of.
static uint16_t gWheelRaw[4];
static uint32_t gWheelMs;
static bool     gWheelSeen;

static uint32_t gCalSamples;
static uint32_t gCalSumRaw;      ///< sum of front-left raw counts
static float    gCalSumKmh;      ///< sum of the matching GPS speeds
static float    gCalRatioMin = 1e9f;
static float    gCalRatioMax = -1e9f;
static float    gCalPrevKmh  = NAN;

/**
 * @brief Decodes 0x1D0 into four raw 15-bit wheel-speed counts.
 *
 * opendbc `7|15@0+`, `8|15@0+`, `25|15@0+`, `42|15@0+` in DBC Motorola
 * numbering: the signal starts at its MSB and walks DOWN through bit positions,
 * wrapping to bit 7 of the next byte. None of the four is byte-aligned, which is
 * exactly why the census prints a per-BIT change mask - a byte-granular flag
 * could not have located these.
 *
 * Integer throughout. The `* 0.01f` that turns counts into km/h happens once, at
 * the point of display, and is never written as `/ 100.0` - that would promote
 * to software double on this FPU-less part.
 */
static void decodeWheelSpeeds(const uint8_t *d, uint16_t out[4])
{
    out[0] = (uint16_t)(((uint16_t)d[0] << 7) | (d[1] >> 1));                        // FL
    out[1] = (uint16_t)(((uint16_t)(d[1] & 0x01) << 14) | ((uint16_t)d[2] << 6) | (d[3] >> 2)); // FR
    out[2] = (uint16_t)(((uint16_t)(d[3] & 0x03) << 13) | ((uint16_t)d[4] << 5) | (d[5] >> 3)); // RL
    out[3] = (uint16_t)(((uint16_t)(d[5] & 0x07) << 12) | ((uint16_t)d[6] << 4) | (d[7] >> 4)); // RR
}

static void calibrationReset()
{
    gCalSamples  = 0;
    gCalSumRaw   = 0;
    gCalSumKmh   = 0.0f;
    gCalRatioMin = 1e9f;
    gCalRatioMax = -1e9f;
    gCalPrevKmh  = NAN;
}

// ─── watchlist ────────────────────────────────────────────────────────────────
//
// "Is ID X on this bus?" answered directly, instead of by scanning a 39-row
// table for something that is not there. Absence is a claim, and a claim needs
// its own evidence: these counters are incremented in the drain BEFORE the
// census slot lookup, so a watched ID is counted even if the 64-slot table is
// full - otherwise "not in the table" could mean "no slot was free", and the
// two would be indistinguishable at exactly the moment it mattered.

static const uint8_t WATCH_MAX = 8;

struct WatchEntry {
    uint16_t id;
    uint32_t count;
};

static WatchEntry gWatch[WATCH_MAX];
static uint8_t    gWatchUsed;

/** @brief Label for the IDs this project has an opinion about; "" otherwise. */
static const char *watchName(uint16_t id)
{
    switch (id) {
    case 0x156: return "STEERING_SENSORS angle+rate";
    case 0x18F: return "STEERING_STATUS";
    case 0x158: return "ENGINE_DATA speed+rpm";
    case 0x17C: return "POWERTRAIN_DATA rpm/pedal/brake";
    case 0x191: return "GEARBOX shifter";
    case 0x1AB: return "STEER_MOTOR_TORQUE";
    case 0x1D0: return "WHEEL_SPEEDS";
    default:    return "";
    }
}

static bool watchAdd(uint16_t id)
{
    for (uint8_t i = 0; i < gWatchUsed; ++i) {
        if (gWatch[i].id == id) return true;         // idempotent
    }
    if (gWatchUsed >= WATCH_MAX) return false;
    gWatch[gWatchUsed].id    = id;
    gWatch[gWatchUsed].count = 0;
    ++gWatchUsed;
    return true;
}

static void censusReset()
{
    memset(gStat, 0, sizeof(gStat));
    gUsed           = 0;
    gSlotOverflow   = 0;
    gTotalFrames    = 0;
    gExtendedFrames = 0;
    gRxOverruns     = 0;
    gIntAsserted    = 0;
    gIntMissed      = 0;
    gWindowStartMs  = millis();
    gRateBaseMs     = gWindowStartMs;
    // The filter statistics describe the same evidence and must restart with
    // it, or an 'r' leaves per-slice frame counts from a window whose census
    // has been thrown away.
    sweepStatsReset();
    // Entries survive a reset; only the evidence restarts.
    for (uint8_t i = 0; i < gWatchUsed; ++i) gWatch[i].count = 0;
}

static void censusAdd(const RawFrame &f, uint32_t nowMs)
{
    ++gTotalFrames;
    if (f.extended) { ++gExtendedFrames; return; }   // table is 11-bit only

    // Before the slot lookup, so a full table cannot turn "present" into "absent".
    for (uint8_t i = 0; i < gWatchUsed; ++i) {
        if (gWatch[i].id == f.id) { ++gWatch[i].count; break; }
    }

    uint8_t slot = 0xFF;
    for (uint8_t i = 0; i < gUsed; ++i) {
        if (gStat[i].id == f.id) { slot = i; break; }
    }

    if (slot == 0xFF) {
        if (gUsed >= CENSUS_SLOTS) { ++gSlotOverflow; return; }
        slot = gUsed++;
        CanIdStat &s = gStat[slot];
        s.id       = f.id;
        s.count    = 0;
        s.lastCount= 0;
        s.firstMs  = nowMs;
        s.minGapMs = 0xFFFF;
        s.maxGapMs = 0;
        memset(s.orMask,  0x00, 8);
        memset(s.andMask, 0xFF, 8);
        memset(s.winOr,   0x00, 8);
        memset(s.winAnd,  0xFF, 8);
    }

    CanIdStat &s = gStat[slot];
    if (s.count > 0) {
        const uint32_t gap = nowMs - s.lastMs;
        const uint16_t g   = (gap > 0xFFFF) ? 0xFFFF : (uint16_t)gap;
        if (g < s.minGapMs) s.minGapMs = g;
        if (g > s.maxGapMs) s.maxGapMs = g;
    }
    ++s.count;
    s.lastMs = nowMs;
    s.dlc    = f.dlc;

    for (uint8_t i = 0; i < 8; ++i) {
        s.orMask[i]  |= f.data[i];
        s.andMask[i] &= f.data[i];
        s.winOr[i]   |= f.data[i];
        s.winAnd[i]  &= f.data[i];
        s.last8[i]    = f.data[i];
    }
}

// ─── reporting ────────────────────────────────────────────────────────────────

static void printHex8(Print &out, uint8_t v)
{
    if (v < 0x10) out.print('0');
    out.print(v, HEX);
}

/** @brief Prints @p v right-aligned in @p width columns, so the table stays square. */
static void printPadded(Print &out, uint32_t v, uint8_t width)
{
    uint8_t digits = 1;
    for (uint32_t t = v; t >= 10UL; t /= 10UL) ++digits;
    for (uint8_t i = digits; i < width; ++i) out.print(' ');
    out.print(v);
}

/**
 * @brief Pairs one fresh GPS fix with the most recent wheel-speed frame.
 *
 * Called only on a fix the receiver marked valid. Rejects anything that would
 * make the ratio mean less than it appears to: too slow, still accelerating, or
 * paired with a wheel frame old enough that the vehicle moved between them.
 */
static void calibrationSample(float gpsKmh, uint32_t nowMs)
{
    const float prev = gCalPrevKmh;
    gCalPrevKmh = gpsKmh;

    if (!gWheelSeen)                    return;
    if (nowMs - gWheelMs > 100UL)       return;   // stale frame, 50 Hz ID
    if (gpsKmh < CAL_MIN_KMH)           return;
    if (isnan(prev))                    return;   // no previous sample to judge steadiness
    if (fabsf(gpsKmh - prev) > CAL_MAX_DELTA_KMH) return;
    if (gWheelRaw[0] == 0)              return;   // no divide, and no free sample

    ++gCalSamples;
    gCalSumRaw += gWheelRaw[0];
    gCalSumKmh += gpsKmh;

    const float r = (float)gWheelRaw[0] / gpsKmh;
    if (r < gCalRatioMin) gCalRatioMin = r;
    if (r > gCalRatioMax) gCalRatioMax = r;
}

/**
 * @brief The speed cross-check: GNSS truth against the decoded CAN wheels.
 *
 * The headline number is counts-per-km/h. 100 means a count is 0.01 km/h and
 * the assumed scale is right; anything else IS the scale, read directly.
 */
static void printSpeedCrossCheck(Print &out)
{
    out.println();
    out.println(F("---- GNSS cross-check ----------------------------------------"));

    if (!gpsEnabled) {
        out.println(F("GNSS disabled ('g' to enable). CAN drain runs clean without it."));
        return;
    }

    out.print(F("GNSS: "));
    if (!gpsReady) {
        out.print(F("bringing up - "));
        out.println(gpsInitStageName(gpsInit.stage));
        return;
    }

    out.print(gpsData.fixValid ? F("fix") : F("NO FIX"));
    out.print(F("  sats "));      out.print(gpsData.satellites);
    out.print(F("  speed "));     out.print(gpsData.velocityKmh, 2);
    out.print(F(" km/h  polls ")); out.print(gGpsPolls);
    out.print(F("  worst stall ")); out.print(gGpsPollMaxUs / 1000UL);
    out.println(F(" ms"));

    if (!gWheelSeen) {
        out.print(F("0x"));
        out.print(WHEEL_SPEED_ID, HEX);
        out.println(F(" not seen yet."));
        return;
    }

    out.print(F("wheels raw  FL "));  out.print(gWheelRaw[0]);
    out.print(F("  FR "));            out.print(gWheelRaw[1]);
    out.print(F("  RL "));            out.print(gWheelRaw[2]);
    out.print(F("  RR "));            out.print(gWheelRaw[3]);
    out.print(F("   (x0.01 = "));     out.print(gWheelRaw[0] * 0.01f, 2);
    out.println(F(" km/h FL)"));

    out.print(F("calibration: "));
    if (gCalSamples == 0) {
        out.print(F("0 usable samples - need a steady run above "));
        out.print(CAL_MIN_KMH, 0);
        out.println(F(" km/h"));
        return;
    }

    const float ratio = (float)gCalSumRaw / gCalSumKmh;
    out.print(gCalSamples);
    out.print(F(" samples   "));
    out.print(ratio, 2);
    out.print(F(" counts per km/h  (spread "));
    out.print(gCalRatioMin, 1);
    out.print(F(" .. "));
    out.print(gCalRatioMax, 1);
    out.println(')');

    // Stated as a range, not a point: tyre wear and pressure move a wheel-speed
    // reading a few percent away from true ground speed, so an exact 100.00 is
    // not the thing to wait for. The candidate scales differ by 2x, so a few
    // percent cannot confuse them.
    out.print(F("  -> "));
    if (ratio > 90.0f && ratio < 110.0f) {
        out.println(F("0.01 km/h per count CONFIRMED."));
    } else if (ratio > 180.0f && ratio < 220.0f) {
        out.println(F("0.005 km/h per count - HALVE the assumed scale."));
    } else if (ratio > 45.0f && ratio < 55.0f) {
        out.println(F("0.02 km/h per count - DOUBLE the assumed scale."));
    } else {
        out.print(F("scale is 1/"));
        out.print(ratio, 2);
        out.println(F(" km/h per count - none of the expected values."));
    }
    if (gCalSamples < 30) {
        out.println(F("     (few samples - keep driving steadily before trusting this)"));
    }
}

/**
 * @brief Reports each watched ID as present or absent, with the sample size.
 *
 * "NOT SEEN" is a negative result, so it is printed with the number of frames
 * it is based on. Absence over 200 000 frames is a conclusion; absence over 200
 * is a shrug, and the two must not look alike on the page.
 */
static void printWatchlist(Print &out)
{
    if (gWatchUsed == 0) return;

    out.println();
    out.println(F("---- watchlist (counted before the slot table, so a full table cannot hide one) ----"));
    for (uint8_t i = 0; i < gWatchUsed; ++i) {
        out.print(F("0x"));
        if (gWatch[i].id < 0x100) out.print('0');
        out.print(gWatch[i].id, HEX);
        out.print(' ');

        const char *nm = watchName(gWatch[i].id);
        out.print(nm);
        for (uint8_t p = (uint8_t)strlen(nm); p < 34; ++p) out.print(' ');

        if (gWatch[i].count == 0) {
            out.print(F("NOT SEEN in "));
            out.print(gTotalFrames);
            out.println(F(" frames"));
        } else {
            out.print(F("SEEN  "));
            out.print(gWatch[i].count);
            out.println(F(" frames - details in the table below"));
        }
    }
    if (gRxOverruns > 0) {
        out.println(F("  ^ overruns are non-zero, so a NOT SEEN is weaker than it looks."));
        out.println(F("    A 100 Hz message could not hide, but a rare one could. Re-run clean."));
    }
}

/**
 * @brief Prints the range each byte and each big-endian pair covered.
 *
 * @c span is the number to read: perform one deliberate motion, then whichever
 * field has a span far larger than the rest is the one that motion drives. A
 * field jittering on sensor noise spans tens of counts; a wheel taken lock to
 * lock spans thousands.
 */
static void focusPrintSummary(Print &out)
{
    if (gFocusId == 0xFFFF) return;

    out.print(F("\n[focus 0x"));
    out.print(gFocusId, HEX);
    out.print(F("]  frames "));
    out.print(gFocusFrames);
    out.println(F("   min..max since armed"));

    if (gFocusFrames == 0) {
        out.println(F("  (no frames with this ID - check it exists in the table)"));
        return;
    }

    out.print(F("  bytes "));
    for (uint8_t i = 0; i < gFocusDlc; ++i) {
        out.print(' ');
        printHex8(out, gFocusByteMin[i]);
        out.print(F(".."));
        printHex8(out, gFocusByteMax[i]);
    }
    out.println();

    for (uint8_t p = 0; (uint8_t)(p + 1) < gFocusDlc; ++p) {
        out.print(F("  b"));
        out.print(p);
        out.print(':');
        out.print(p + 1);
        out.print(F(" BE signed  "));
        out.print(gFocusPairMin[p]);
        out.print(F(" .. "));
        out.print(gFocusPairMax[p]);
        out.print(F("   span "));
        out.print((int32_t)gFocusPairMax[p] - (int32_t)gFocusPairMin[p]);
        // The final byte is COUNTER+CHECKSUM on Honda, so this pair is a data
        // byte glued to a free-running nibble. Its span is always large and
        // always fictitious - say so rather than let it be read as a signal.
        if ((uint8_t)(p + 2) == gFocusDlc)
            out.print(F("   <- spans the counter/checksum byte, ignore"));
        out.println();
    }
}

static void printReport(Print &out)
{
    const uint32_t nowMs   = millis();
    const uint32_t elapsed = nowMs - gWindowStartMs;   // whole run, for totals
    const uint32_t rateMs  = nowMs - gRateBaseMs;      // this interval, for Hz
    if (elapsed == 0) return;

    const uint8_t eflg = mcpRead(REG_EFLG);
    const uint8_t tec  = mcpRead(REG_TEC);
    const uint8_t rec  = mcpRead(REG_REC);

    out.println();
    out.println(F("================ CAN census ================"));
    out.print(F("window "));      out.print(elapsed / 1000);
    out.print(F(" s   frames ")); out.print(gTotalFrames);
    out.print(F("   ids "));      out.print(gUsed);
    out.print(F("/"));            out.println(CENSUS_SLOTS);

    // Wall-clock time, written into every report rather than into the filename.
    // millis() alone dates a log only relative to a boot nobody recorded; with
    // this line a card full of CANLOGnn.TXT can be ordered and correlated with
    // anything else that happened. Printed only when the receiver vouches for
    // it - an invalid UTC stamp is worse than an absent one, because it will be
    // believed.
    out.print(F("clock "));
    if (gpsData.utc.valid) {
        out.print(gpsData.utc.year);   out.print('-');
        if (gpsData.utc.month  < 10) out.print('0');
        out.print(gpsData.utc.month);  out.print('-');
        if (gpsData.utc.day    < 10) out.print('0');
        out.print(gpsData.utc.day);    out.print(' ');
        if (gpsData.utc.hour   < 10) out.print('0');
        out.print(gpsData.utc.hour);   out.print(':');
        if (gpsData.utc.minute < 10) out.print('0');
        out.print(gpsData.utc.minute); out.print(':');
        if (gpsData.utc.second < 10) out.print('0');
        out.print(gpsData.utc.second);
        out.print(F(" UTC (GNSS)"));
    } else {
        out.print(F("no valid GNSS time; uptime "));
        out.print(nowMs / 1000);
        out.print(F(" s"));
    }
    if (gSdOk) {
        out.print(F("   log "));
        out.print(gSdName);
        out.print(F(" dumps "));
        out.print(gSdDumps);
        out.print(F(" worst write "));
        out.print(gSdWriteMaxUs / 1000UL);
        out.print(F(" ms"));
    } else {
        out.print(F("   NO SD LOG"));
    }
    out.println();

    out.print(F("TEC "));           out.print(tec);
    out.print(F("  REC "));         out.print(rec);
    out.print(F("  EFLG 0x"));      printHex8(out, eflg);
    out.print(F("  overruns "));    out.print(gRxOverruns);
    out.print(F("  ext "));         out.print(gExtendedFrames);
    out.print(F("  slotOverflow ")); out.println(gSlotOverflow);

    // Whether the INT pin can be trusted as a "frame pending" gate on this
    // board. Production code wants this answered before relying on it.
    out.print(F("INT pin: asserted "));
    out.print(gIntAsserted);
    out.print(F("  missed "));
    out.print(gIntMissed);
    if (gIntMissed > 0 && gIntAsserted == 0)
        out.println(F("   -> INT NOT WIRED to this pin; poll CANINTF instead"));
    else if (gIntAsserted > 0 && gIntMissed == 0)
        out.println(F("   -> INT works; usable as a cheap poll gate"));
    else
        out.println();

    // BEFORE the no-frames bail-out, not after. A dead CAN bus says nothing
    // about the receiver, and the first minute of a real run is exactly when
    // both are unproven - ignition off, GNSS still acquiring. Reporting only
    // "NO FRAMES RECEIVED" for that minute hides whether the half of the system
    // that IS working is working, and it is the half you cannot re-check later.
    printSpeedCrossCheck(out);

    if (gSweepMode != SWEEP_OFF) {
        out.println(F("\n---- hardware ID filter --------------------------------------"));
        if (gSweepMode == SWEEP_GROUPED) {
            out.print(F("GROUPED: admitting 7 of 8 slices, EXCLUDING 0x"));
            out.print((uint16_t)(gSweepSlice << 8), HEX);
            out.print(F("-0x"));
            out.println((uint16_t)((gSweepSlice << 8) | 0x0FFu), HEX);
            out.println(F("  mask0 0x700 (one slice) + mask1 0x600 (three pairs)"));
        } else {
            out.print(F("PARKED on 0x"));
            out.print((uint16_t)(gSweepSlice << 8), HEX);
            out.print(F("-0x"));
            out.println((uint16_t)((gSweepSlice << 8) | 0x0FFu), HEX);
        }
        // Said plainly, because the rates in the table below are now wrong in a
        // specific way: an excluded ID stops accumulating entirely, so its Hz
        // decays toward zero and its maxGap grows without bound. A filtered run
        // answers PRESENCE; rates come from an accept-all run.
        out.println(F("Hz/maxGap below apply only to ADMITTED ids - an excluded"));
        out.println(F("one keeps its old totals and its gap grows forever."));
        // One number, stated once. A dropped frame is the one that was never
        // read, so which slice it belonged to cannot be known - a per-slice
        // loss column would be invention, and the version that had one printed
        // "no loss" against seven slices that were sharing the losses.
        uint32_t admitted = 0;
        for (uint8_t s = 0; s < SWEEP_SLICES; ++s) admitted += gSweepFrames[s];
        out.print(F("overruns under this configuration: "));
        out.print(gSweepOverrunsCfg);
        if (gSweepOverrunsCfg == 0) {
            out.println(F("  (admitted load fits)"));
        } else if (admitted < (uint32_t)elapsed * 300UL / 1000UL) {
            // Narrowing further would not help and the advice used to say it
            // would. At a low admitted rate the two RX buffers cannot overflow
            // from traffic alone - something STALLED the drain. On this board
            // that is the SD dump (measured at 252-292 ms) or a GNSS poll, both
            // of which are visible in the header above.
            out.println(F("  <- NOT bus load; the drain stalled"));
            out.println(F("     see 'worst write' and GNSS 'worst stall' above"));
        } else {
            out.println(F("  <- still lossy; park on a narrower range"));
        }
        out.println(F("  slice        range   frames"));
        for (uint8_t s = 0; s < SWEEP_SLICES; ++s) {
            const bool admitted = (gSweepMode == SWEEP_GROUPED) ? (s != gSweepSlice)
                                                                : (s == gSweepSlice);
            out.print(admitted ? F("    0x") : F("  - 0x"));
            out.print((uint16_t)(s << 8), HEX);
            out.print(F("  0x"));    out.print((uint16_t)(s << 8), HEX);
            out.print(F("-0x"));     out.print((uint16_t)((s << 8) | 0x0FFu), HEX);
            out.print(F("  "));      out.print(gSweepFrames[s]);
            if (!admitted) out.print(F("   excluded"));
            out.println();
        }
    }

    if (gTotalFrames == 0) {
        out.println();
        if (gRxOverruns > 0) {
            // Frames arrived and overflowed but none were decoded: that is a
            // fault in THIS sketch's drain path, not in the bus. Say so plainly
            // rather than let the bus take the blame.
            out.println(F("*** DRAIN BUG, NOT A BUS PROBLEM ***"));
            out.println(F("  Overruns are non-zero, so frames ARE arriving and"));
            out.println(F("  filling the RX buffers - but none were read out."));
            out.println(F("  The bus is fine; the receive path in this sketch is not."));
            return;
        }
        out.println(F("*** NO FRAMES RECEIVED ***"));
        out.println(F("  - This tap may not carry broadcast traffic (an OBD-II"));
        out.println(F("    gateway often bridges only diagnostic responses)."));
        out.println(F("  - Or the bit timing is wrong: check the module crystal"));
        out.println(F("    really is 16 MHz, and that the bus is 500 kbps."));
        out.println(F("  - Or CANH/CANL are swapped / not connected."));
        out.println(F("  TEC/REC stay 0 in listen-only even on a timing"));
        out.println(F("  mismatch, so they cannot rule this out on their own."));
        return;
    }

    printWatchlist(out);

    // Sorted by ID so two captures can be diffed line by line - which is how
    // signals actually get located (idle vs revving, parked vs rolling, ...).
    out.println();
    // maxGap flags intermittent / event-driven IDs: a message whose max gap far
    // exceeds its average period is not on a fixed schedule, which is a strong
    // hint on its own (and a reason a signal may look "missing" in a capture).
    // Two change masks, and the WINDOW one is the one to read. See CanIdStat:
    // the all-run mask saturates and then never says anything again, so it is
    // useful only as "was this byte ever alive", never as "did this motion move
    // it". Both are printed because they answer different questions.
    out.println(F("  ID   Hz  DLC    total maxGap  payload (last)           changed (window)         changed (all run)"));
    out.println(F("----- ---- --- -------- ------ ------------------------ ------------------------ ------------------------"));

    // Selection sort by ID, in place of building an index array (no heap, and
    // 64 entries makes the O(n^2) irrelevant). IDs in the table are unique, so
    // "smallest ID greater than the last one printed" is a total order.
    int32_t lastPrinted = -1;
    for (uint8_t n = 0; n < gUsed; ++n) {
        uint8_t  pick = 0xFF;
        uint32_t best = 0xFFFFFFFFUL;
        for (uint8_t i = 0; i < gUsed; ++i) {
            if ((int32_t)gStat[i].id <= lastPrinted) continue;   // already printed
            if (gStat[i].id < best) { best = gStat[i].id; pick = i; }
        }
        if (pick == 0xFF) break;
        lastPrinted = (int32_t)gStat[pick].id;

        const CanIdStat &s = gStat[pick];
        // Instantaneous: frames THIS interval, not since reset. See CanIdStat.
        const uint32_t hz = (rateMs > 0)
                          ? ((s.count - s.lastCount) * 1000UL) / rateMs
                          : 0UL;

        out.print(F("0x"));
        if (s.id < 0x100) out.print('0');
        out.print(s.id, HEX);
        printPadded(out, hz, 5);
        out.print(F("   "));
        out.print(s.dlc);
        printPadded(out, s.count, 9);
        printPadded(out, (s.count > 1) ? s.maxGapMs : 0, 7);
        out.print(' ');

        // Only DLC bytes exist; the rest are zeroed at capture. Print them as
        // ".." so a short frame is never mistaken for one carrying zeros.
        for (uint8_t i = 0; i < 8; ++i) {
            if (i < s.dlc) printHex8(out, s.last8[i]); else out.print(F(".."));
            out.print(' ');
        }
        out.print(' ');
        // or & ~and = bits that were not constant. Per-BIT, not per-byte: an
        // unaligned 15-bit wheel-speed field cannot be located from a
        // byte-granular "changed" flag.
        for (uint8_t i = 0; i < 8; ++i) {
            if (i < s.dlc) printHex8(out, (uint8_t)(s.winOr[i] & (uint8_t)~s.winAnd[i]));
            else           out.print(F(".."));
            out.print(' ');
        }
        out.print(' ');
        for (uint8_t i = 0; i < 8; ++i) {
            if (i < s.dlc) printHex8(out, (uint8_t)(s.orMask[i] & (uint8_t)~s.andMask[i]));
            else           out.print(F(".."));
            out.print(' ');
        }
        out.println();
    }

    if (gRxOverruns > 0) {
        out.println();
        out.println(F("NOTE: RX overruns occurred - this census is INCOMPLETE."));
        out.println(F("      Rates are undercounted and rare IDs may be missing."));
    }
    if (gSlotOverflow > 0) {
        out.println();
        out.println(F("NOTE: more distinct IDs than slots - raise CENSUS_SLOTS."));
    }

    focusPrintSummary(out);

    // Re-baseline so the next report's Hz and change mask cover only the next
    // interval. An ID that goes silent keeps winOr=0x00 / winAnd=0xFF, which
    // prints as "nothing changed" - correct, and distinct from a live-but-
    // constant field only by the Hz column, which is where it belongs.
    for (uint8_t i = 0; i < gUsed; ++i) {
        gStat[i].lastCount = gStat[i].count;
        memset(gStat[i].winOr,  0x00, 8);
        memset(gStat[i].winAnd, 0xFF, 8);
    }
    gRateBaseMs = millis();
}

/**
 * @brief Reads up to three hex digits following a command key ("f13C", "w156").
 *
 * @param digits out: digits consumed; 0 means the key was pressed bare.
 * Bounded by a 500 ms deadline so a torn paste or a half-typed command cannot
 * wedge loop() while the bus keeps filling the RX buffers.
 */
static uint16_t readHexId(uint8_t &digits)
{
    uint16_t id = 0;
    digits = 0;

    const uint32_t t0 = millis();
    while (digits < 3 && (millis() - t0) < 500) {
        // Keep draining while waiting. This used to spin on Serial alone, and
        // the bus does not pause for the operator: at ~1100 frames/s a 500 ms
        // wait overflows the two RX buffers many times over, so every command
        // typed punched a hole in the very census it was about to reconfigure.
        busDrain(millis());
        if (Serial.available() <= 0) continue;
        const int c = Serial.read();
        int v = -1;
        if      (c >= '0' && c <= '9') v = c - '0';
        else if (c >= 'a' && c <= 'f') v = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') v = c - 'A' + 10;
        if (v < 0) break;                       // non-hex ends the number
        id = (uint16_t)((id << 4) | (uint16_t)v);
        ++digits;
    }
    return id;
}

static void printHelp()
{
    Serial.println();
    Serial.println(F("keys:  r = reset stats   d = dump to serial   h = this help"));
    Serial.println(F("       s = dump to SD now (do this before switching off)"));
    Serial.println(F("       f13C = focus 0x13C   f = focus off   m = clear focus min/max"));
    Serial.println(F("       w156 = watch 0x156 for presence   w = clear watchlist"));
    Serial.println(F("       x = admit 7 of 8 ID slices, excluding the busiest"));
    Serial.println(F("           x300 = park on 0x300-0x3FF only   x again = off"));
    Serial.println(F("           (sheds the load that makes rare IDs get dropped)"));
    Serial.println(F("       g = GNSS cross-check on/off   k = clear calibration"));
    Serial.println(F("to calibrate wheel-speed scale: drive a steady 40-80 km/h for"));
    Serial.println(F("a minute, then read 'counts per km/h'. 100 = 0.01 km/h/count."));
    Serial.println(F("read the 'changed (window)' column, not 'all run': hold one"));
    Serial.println(F("state for a report interval and only its bits light up."));
    Serial.println(F("to identify a swept signal: arm focus, press m, do ONE slow"));
    Serial.println(F("motion, then read which pair has by far the largest span."));
}

// ─── bus setup ────────────────────────────────────────────────────────────────

/**
 * @brief Brings the controller up directly into Listen-Only.
 *
 * begin(bitrate, stayInConfigurationMode=true) programs the bit timing but
 * leaves the chip in Configuration mode, so it never passes through Normal mode
 * on the way to Listen-Only. That matters on a live bus: in Normal mode the
 * controller would ACK frames and could emit error frames if the bit timing is
 * wrong, which is exactly what a read-only tap must never do.
 */
/**
 * @brief Admits only identifiers whose top three bits are @p slice.
 *
 * All six filters go on the SAME slice deliberately. Spreading them across six
 * would admit six slices at once, which is the exact opposite of the point:
 * this exists to take traffic OFF the drain so the quiet parts of the
 * identifier space stop losing frames to overruns caused by the busy part.
 *
 * Ends in Listen-Only, never Normal — a census must not become bus-active.
 */
static bool sweepApplyPark(uint8_t slice)
{
    const uint16_t base = (uint16_t)((uint16_t)(slice & 0x07u) << 8);
    if (!CAN.setFilterRegisters(SWEEP_MASK_FINE, base, base,
                                SWEEP_MASK_FINE, base, base, base, base,
                                /* allowRollover = */ true,
                                /* targetMode    = */ MODE_LISTEN_ONLY)) {
        return false;
    }
    mcpWrite(REG_CANINTF, 0x00);
    mcpBitModify(REG_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0x00);
    return true;
}

/**
 * @brief Admits every slice EXCEPT @p exclude, using both mask widths at once.
 *
 * Three coarse filters take the three slice pairs that do not contain the
 * excluded slice; one fine filter picks up its pair-mate, which the coarse
 * filters had to leave behind. Seven of eight slices, one configuration, no
 * time-sharing — see the block comment on SWEEP_MASK_FINE.
 *
 * Works for any @p exclude: the complement of a single slice is always
 * decomposable this way, which is why the six filters are exactly enough.
 */
static bool sweepApplyGrouped(uint8_t exclude)
{
    exclude &= 0x07u;
    // The excluded slice's pair-mate: same pair, so the coarse filters cannot
    // admit it without also admitting the one being excluded.
    const uint16_t partner = (uint16_t)((uint16_t)(exclude ^ 1u) << 8);

    uint16_t pair[3];
    uint8_t  n = 0;
    for (uint8_t p = 0; p < 4u; ++p) {
        if (p == (uint8_t)(exclude >> 1)) continue;   // the pair holding it
        pair[n++] = (uint16_t)((uint16_t)p << 9);
    }

    if (!CAN.setFilterRegisters(SWEEP_MASK_FINE,   partner, partner,
                                SWEEP_MASK_COARSE, pair[0], pair[1], pair[2], pair[0],
                                /* allowRollover = */ true,
                                /* targetMode    = */ MODE_LISTEN_ONLY)) {
        return false;
    }
    mcpWrite(REG_CANINTF, 0x00);
    mcpBitModify(REG_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0x00);
    return true;
}

/** @brief Clears the per-configuration filter statistics. */
static void sweepStatsReset()
{
    for (uint8_t s = 0; s < SWEEP_SLICES; ++s) gSweepFrames[s] = 0;
    gSweepOverrunsCfg = 0;
}

/**
 * @brief The slice carrying the most traffic, from the census table itself.
 *
 * @param[out] framesSeen Total frames the decision rests on. ZERO means there is
 *        no evidence and the return value is meaningless - the caller must not
 *        treat slice 0 as a real answer, because with an empty table every slice
 *        ties at zero and the first one wins by position alone.
 *
 * Uses cumulative census totals, so a measurement taken while a filter was
 * already active is biased toward whatever that filter admitted. Reset the
 * census ('r') before choosing if the previous run was filtered.
 */
static uint8_t censusBusiestSlice(uint32_t &framesSeen)
{
    uint32_t load[SWEEP_SLICES];
    for (uint8_t s = 0; s < SWEEP_SLICES; ++s) load[s] = 0;
    framesSeen = 0;
    for (uint8_t i = 0; i < gUsed; ++i) {
        load[(gStat[i].id >> 8) & 0x07u] += gStat[i].count;
        framesSeen += gStat[i].count;
    }
    uint8_t best = 0;
    for (uint8_t s = 1; s < SWEEP_SLICES; ++s) if (load[s] > load[best]) best = s;
    return best;
}

/** @brief Back to RXM=11, accept-all — the normal census configuration. */
static bool sweepDisable()
{
    if (!mcpSetMode(MODE_CONFIG)) return false;
    mcpWrite(REG_RXB0CTRL, 0x64);   // RXM=11 | BUKT
    mcpWrite(REG_RXB1CTRL, 0x60);   // RXM=11
    mcpWrite(REG_CANINTF, 0x00);
    mcpBitModify(REG_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0x00);
    return mcpSetMode(MODE_LISTEN_ONLY);
}

static bool busBeginListenOnly()
{
    pinMode(INT_PIN, INPUT_PULLUP);

    CAN.setPins(CS_PIN, INT_PIN);
    CAN.setClockFrequency(CLOCK_HZ);
    if (!CAN.begin(BITRATE, /* stayInConfigurationMode = */ true)) {
        Serial.println(F("CAN.begin() failed - check wiring, CS pin, and that"));
        Serial.println(F("CLOCK_HZ matches the module crystal (8 vs 16 MHz)."));
        return false;
    }

    // Still in Configuration mode here, which is the only time RXBnCTRL is
    // writable. RXM=11 accepts every ID (no filtering - this is a census), and
    // BUKT lets RXB0 spill into RXB1, doubling the depth from 1 frame to 2.
    mcpWrite(REG_RXB0CTRL, 0x64);   // RXM=11 | BUKT
    mcpWrite(REG_RXB1CTRL, 0x60);   // RXM=11
    mcpWrite(REG_CANINTF, 0x00);
    mcpBitModify(REG_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0x00);

    if (!mcpSetMode(MODE_LISTEN_ONLY)) {
        Serial.print(F("Listen-only NOT confirmed. CANSTAT=0x"));
        printHex8(Serial, mcpRead(REG_CANSTAT));
        Serial.println();
        Serial.println(F("REFUSING to continue - the controller may be bus-active."));
        return false;
    }
    return true;
}

// ─── Arduino entry points ─────────────────────────────────────────────────────

static bool     gBusReady      = false;
static uint32_t gLastReportMs  = 0;

void setup()
{
    // Armed before anything touches I2C. The SAMD core's I2C driver waits on its
    // bus flags with no deadline, so a device that grabs SDA hangs the CPU
    // outright and only a watchdog ends that. Generous period: a full report is
    // several KB over USB CDC and the host decides how fast that drains.
    watchdogArm(8000UL);

    Serial.begin(115200);
    const uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < SERIAL_WAIT_MS) { /* bounded */ }

    if (watchdogCausedReset()) {
        Serial.println();
        Serial.println(F("*** last reset was the WATCHDOG - something hung, most likely I2C ***"));
    }

    Serial.println();
    Serial.println(F("CANDiscovery - passive CAN census (listen-only, read-only)"));
    Serial.print(F("500 kbps, 16 MHz crystal, CS=D"));
    Serial.print(CS_PIN);
    Serial.print(F(" INT=D"));
    Serial.println(INT_PIN);

    // The two IDs a Honda-style F-CAN would carry the steering angle on. Watched
    // by default because their ABSENCE is the finding: 0x156 b0:b1 is the signed
    // wheel angle at 100 Hz on every Honda that publishes one, and 0x18F is the
    // steering status message. Neither has appeared in any capture from this
    // car, which is why steering angle is not in the signal template - and a
    // claim that load-bearing should be re-checked on every run, not remembered.
    watchAdd(0x156);
    watchAdd(0x18F);

    // Non-fatal by design. A census on a bench with no card in the slot is a
    // perfectly good census, and refusing to run would trade a working tool for
    // a missing accessory. The absence is reported in every report header
    // instead, so a run cannot silently fail to be recorded.
    gSdOk = sdBegin();
    if (gSdOk) {
        Serial.print(F("SD: logging to "));
        Serial.print(gSdName);
        Serial.print(F(" every "));
        Serial.print(SD_DUMP_INTERVAL_MS / 1000UL);
        Serial.println(F(" s ('s' dumps now)"));
    } else {
        Serial.println(F("SD: no card - reports go to serial only."));
    }
    gLastSdDumpMs = millis();
    watchdogFeed();

    // GNSS bring-up is ARMED here, not performed: gpsInitTick() advances it one
    // stage per loop() pass. Doing the whole chain in setup() would block for
    // seconds, and a receiver that never answers would cost the census its first
    // seconds of capture for nothing.
    //
    // Preallocation is checked, not attempted-and-forgotten. On failure
    // setPacketCfgPayloadSize() leaves payloadCfg NULL, begin() retries it
    // without checking, and the configuration path then writes payloadCfg[0]
    // regardless - so continuing risks a null dereference rather than a clean
    // failure. Quarantine is terminal for the boot; the census carries on
    // without GNSS, which is still a useful sketch.
    if (i2cBusBegin() != I2CBusState::Ready) {
        Serial.println(F("GNSS: I2C bus not usable - cross-check disabled this boot."));
        gpsEnabled = false;
    } else if (preallocateGPS_I2C(myGNSS) != GPSReturnStatus::OK) {
        Serial.println(F("GNSS: preallocation FAILED (out of RAM) - cross-check disabled."));
        gpsEnabled = false;
    } else {
        gpsInitBegin(gpsInit);
    }
    initGPSData(gpsData);
    gLastGpsPollMs  = millis();
    gLastGpsFreshMs = gLastGpsPollMs;
    watchdogFeed();

    gBusReady = busBeginListenOnly();
    if (gBusReady) {
        Serial.print(F("Listen-only CONFIRMED (CANSTAT=0x"));
        printHex8(Serial, mcpRead(REG_CANSTAT));
        Serial.println(F(") - the bus is not being driven."));
        printHelp();
    }

    censusReset();
    gLastReportMs = millis();
}

/**
 * @brief Drains up to MAX_FRAMES_PER_POLL frames and folds them into the census.
 *
 * A FUNCTION rather than inline in loop() so it can also be pumped while the
 * command parser waits on serial input. readHexId() blocks up to 500 ms, and at
 * ~1100 frames/s that is roughly 550 frames the two RX buffers cannot hold - lost
 * on every keystroke, and then hidden, because the filter routines clear EFLG
 * immediately afterwards. A tool whose own user interface destroys the evidence
 * it exists to gather is worse than one that is merely slow.
 */
static void busDrain(uint32_t nowMs)
{
uint8_t serviced = 0;
while (serviced < MAX_FRAMES_PER_POLL) {
    const uint8_t intf = mcpRead(REG_CANINTF);
    if ((intf & 0x03) == 0) break;         // neither RX buffer holds a frame

    // Free diagnostic: with a frame provably pending, INT must be LOW if it
    // is connected. Tallies whether this board can use the optimisation.
    if (digitalRead(INT_PIN) == LOW) ++gIntAsserted; else ++gIntMissed;

    RawFrame f;
    if (intf & 0x01) mcpReadFrame(INSTR_READ_RXB0, f);
    else             mcpReadFrame(INSTR_READ_RXB1, f);
    censusAdd(f, nowMs);
    // Attributed to the slice that was admitted when it arrived, so the
    // report can show WHICH slices are lossy rather than one global count
    // that says only that something, somewhere, was dropped.
    if (gSweepMode != SWEEP_OFF) ++gSweepFrames[(f.id >> 8) & 0x07u];

    // Snapshot the wheel speeds here rather than reading them back out of
    // the census table later: the calibrator has to know how OLD its CAN
    // sample is before pairing it with a GPS fix, and the table keeps only
    // "the last payload", with no timestamp a consumer can check.
    if (f.id == WHEEL_SPEED_ID && f.dlc == 8) {
        decodeWheelSpeeds(f.data, gWheelRaw);
        gWheelMs   = nowMs;
        gWheelSeen = true;
    }

    // Focus mode: watch one ID closely. The 5 s table only ever shows the
    // LAST payload, which is useless for watching a signal move - you
    // cannot correlate a snapshot with a steering sweep.
    if (f.id == gFocusId) {
        focusTrack(f);                       // every frame, so extremes are exact
        if (nowMs - gFocusLastPrintMs >= FOCUS_PRINT_INTERVAL_MS) {
            gFocusLastPrintMs = nowMs;
            Serial.print(F("  "));
            Serial.print(nowMs);
            Serial.print(F("  0x"));
            Serial.print(f.id, HEX);
            Serial.print(F("  "));
            for (uint8_t i = 0; i < f.dlc; ++i) { printHex8(Serial, f.data[i]); Serial.print(' '); }
            // Every big-endian pair decoded, so a swing is readable
            // directly without hex arithmetic in your head.
            // Aligned pairs only on the live line - it has to stay narrow
            // enough to read while something is moving. The full adjacent
            // scan is in the min/max summary. The last byte is
            // counter+checksum, so it is never paired here.
            Serial.print('|');
            for (uint8_t p = 0; (uint8_t)(p * 2 + 2) < f.dlc; ++p) {
                const int16_t v =
                    (int16_t)(((uint16_t)f.data[p * 2] << 8) | f.data[p * 2 + 1]);
                Serial.print(F("  b"));
                Serial.print(p * 2);
                Serial.print(':');
                Serial.print(p * 2 + 1);
                Serial.print('=');
                Serial.print(v);
            }
            Serial.println();
        }
    }
    ++serviced;
}
}

void loop()
{
    if (!gBusReady) { delay(1000); return; }   // refuse to run half-configured

    const uint32_t nowMs = millis();

    // Drain, gated on CANINTF over SPI - NOT on the INT pin.
    //
    // An earlier version used `digitalRead(INT_PIN) == LOW` as a cheap 0.7 us
    // gate to skip the SPI read when idle. That is a fine optimisation only if
    // INT is actually wired to this pin. On this shield it is not, so the gate
    // was never satisfied, the drain never ran, and the census reported
    // "0 frames" on a bus that was in fact saturating the RX buffers - which
    // sent the whole investigation chasing gateways and transceivers.
    //
    // The lesson is the shape of the bug, not the pin: an optimisation whose
    // failure mode is silently reporting nothing, rather than being slower.
    // CANINTF is the authoritative source and costs ~5 us; that is affordable.
    busDrain(nowMs);

    // ── GNSS, AFTER the drain ────────────────────────────────────────────────
    //
    // Order matters. The drain is bounded and fast; a GNSS poll is neither, so
    // it goes last and the RX buffers are as empty as they will get before it
    // starts. Its duration is measured because it is the one thing in this loop
    // that can stall the drain long enough to lose frames, and an unmeasured
    // cost is one nobody weighs.
    if (gpsEnabled) {
        if (gpsReady) {
            if (millis() - gLastGpsPollMs >= GPS_POLL_MS) {
                const uint32_t t0 = micros();
                const GPSReturnStatus st = getGPSData(myGNSS, gpsData);
                const uint32_t dt = micros() - t0;
                if (dt > gGpsPollMaxUs) gGpsPollMaxUs = dt;
                gLastGpsPollMs = millis();
                ++gGpsPolls;

                // OK and NO_FIX both mean a PVT arrived, so both prove the
                // receiver is alive; only OK carries a position worth using.
                if (st == GPSReturnStatus::OK || st == GPSReturnStatus::NO_FIX) {
                    gLastGpsFreshMs = gLastGpsPollMs;
                    gpsInitConfirmStreaming(gpsInit);
                    if (st == GPSReturnStatus::OK && gpsData.fixValid) {
                        calibrationSample(gpsData.velocityKmh, gLastGpsPollMs);
                    }
                }
            }

            const uint32_t sinceFresh = millis() - gLastGpsFreshMs;
            if (sinceFresh > GPS_FIX_MAX_AGE_MS) invalidateGPSFix(gpsData);
            if (sinceFresh > GPS_MAX_SILENCE_MS) {
                Serial.println(F("\n[GNSS: silent past the window; re-initialising]"));
                gpsReady = false;
                gpsInitFail(gpsInit, GPSReturnStatus::NOK_INIT_FAILED);
                initGPSData(gpsData);
                // The calibration window is over: samples from before an outage
                // pair CAN frames with fixes the receiver has since disowned.
                calibrationReset();
            }
        } else {
            // One stage per pass, never the whole chain. A stage can still hold
            // the CPU for up to 750 ms, so bring-up WILL overrun the RX buffers
            // - bounded and visible, which is the most that can be said for it.
            const GPSInitStage stage = gpsInitTick(myGNSS, gpsInit);
            if (stage == GPSInitStage::Done) {
                gpsReady        = true;
                gLastGpsPollMs  = millis();
                gLastGpsFreshMs = gLastGpsPollMs;
                Serial.println(F("\n[GNSS: ready]"));
            }
        }
    }

    watchdogFeed();

    // Overruns must be visible: without this an undercount reads as truth.
    const uint8_t eflg = mcpRead(REG_EFLG);
    if (eflg & (EFLG_RX0OVR | EFLG_RX1OVR)) {
        ++gRxOverruns;
        // Charged to the ACTIVE CONFIGURATION, never to a slice - the dropped
        // frame's identifier is unknowable, so per-slice attribution would be
        // fabricated. What a non-zero count does say is that even the filtered
        // load is too much, which is the signal to park on a narrower range.
        if (gSweepMode != SWEEP_OFF) ++gSweepOverrunsCfg;
        mcpBitModify(REG_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0x00);
    }

    while (Serial.available() > 0) {
        const int cmd = Serial.read();
        switch (cmd) {
        case 'r': censusReset(); Serial.println(F("\n[stats reset]")); break;
        case 'd': printReport(Serial); gLastReportMs = millis();       break;
        case 's': {
            // Force a card dump now, without waiting out the minute. Useful
            // just before switching off, which is when a run's last and most
            // interesting minute would otherwise be the one that is missing.
            if (!gSdOk) { Serial.println(F("\n[no SD log]")); break; }
            TeePrint tee(Serial, gSdFile);
            printReport(tee);
            gSdFile.flush();
            gLastSdDumpMs = millis();
            gLastReportMs = gLastSdDumpMs;
            ++gSdDumps;
            break;
        }
        case 'h': printHelp();                                         break;
        case 'm':
            // Re-arm the extremes without disturbing the census: one motion per
            // marking, so spans are attributable to that motion alone.
            focusReset();
            Serial.println(F("\n[focus min/max cleared]"));
            break;
        case 'g':
            gpsEnabled = !gpsEnabled;
            Serial.print(F("\n[GNSS "));
            Serial.println(gpsEnabled ? F("on]") : F("off - drain runs clean]"));
            break;
        case 'k':
            calibrationReset();
            Serial.println(F("\n[calibration cleared]"));
            break;
        case 'f': {
            // "f13C" focuses 0x13C; a bare "f" turns focus off.
            uint8_t  n  = 0;
            const uint16_t id = readHexId(n);
            gFocusId = (n > 0) ? id : 0xFFFF;
            focusReset();          // extremes are per-arming, not per-boot
            Serial.print(F("\n[focus "));
            if (gFocusId == 0xFFFF) Serial.println(F("off]"));
            else { Serial.print(F("0x")); Serial.print(gFocusId, HEX); Serial.println(']'); }
            break;
        }
        case 'x': {
            // "x" toggles GROUPED: seven slices admitted, the busiest excluded.
            // The busiest is taken from the census table rather than configured,
            // so it is right for whatever vehicle this is plugged into.
            //
            // "x300" PARKS on the one slice holding 0x300. Grouped is the better
            // default - it covers almost everything at once - but parking is
            // what you want when even the filtered load still overruns, or to
            // get maximum headroom on the busy slice itself, which grouped mode
            // is precisely the one to exclude.
            uint8_t n = 0;
            const uint16_t id = readHexId(n);

            // ── hardware FIRST, state only on confirmed success ──────────────
            // Setting gSweepMode before programming meant a failed transition
            // left the sketch describing a configuration the controller was not
            // in - and on the disable path it reported "accept-all" while the
            // chip could still be sitting in Configuration mode receiving
            // nothing at all, which reads as a dead bus.
            uint8_t   wantSlice = gSweepSlice;
            SweepMode wantMode;

            if (n > 0) {
                if (id > 0x7FFu) {
                    // Masking it into range would silently park on some other
                    // slice and report success. An 11-bit bus has no 0x800.
                    Serial.println(F("\n[id above 0x7FF - standard frames only]"));
                    break;
                }
                wantSlice = (uint8_t)((id >> 8) & 0x07u);
                wantMode  = SWEEP_PARK;
            } else if (gSweepMode == SWEEP_OFF) {
                uint32_t seen = 0;
                wantSlice = censusBusiestSlice(seen);
                if (seen == 0) {
                    // Without traffic the "busiest" slice is just slice 0, and
                    // excluding it would be a coin toss dressed as a decision.
                    Serial.println(F("\n[no frames yet - let the census run first]"));
                    break;
                }
                wantMode = SWEEP_GROUPED;
            } else {
                if (!sweepDisable()) {
                    // Mode is left as it was: the filters may still be live, and
                    // claiming accept-all would be the more dangerous error.
                    Serial.println(F("\n[filter off: restore FAILED - state unchanged]"));
                } else {
                    gSweepMode = SWEEP_OFF;
                    sweepStatsReset();
                    Serial.println(F("\n[filter off - accept-all]"));
                }
                break;
            }

            if (!(wantMode == SWEEP_PARK ? sweepApplyPark(wantSlice)
                                         : sweepApplyGrouped(wantSlice))) {
                Serial.println(F("\n[filter programming FAILED]"));
                if (sweepDisable()) gSweepMode = SWEEP_OFF;
                else Serial.println(F("[and restore FAILED - controller state unknown]"));
                break;
            }
            gSweepMode  = wantMode;
            gSweepSlice = wantSlice;
            // Per-configuration, so a grouped run and a parked run are never
            // added together - they mean different things.
            sweepStatsReset();

            const uint16_t base = (uint16_t)((uint16_t)gSweepSlice << 8);
            if (gSweepMode == SWEEP_GROUPED) {
                Serial.print(F("\n[grouped: 7 of 8 slices, excluding 0x"));
                Serial.print(base, HEX);
                Serial.print(F("-0x"));
                Serial.print((uint16_t)(base | 0x0FFu), HEX);
                Serial.println(F(" (busiest)]"));
            } else {
                Serial.print(F("\n[parked on 0x"));
                Serial.print(base, HEX);
                Serial.print(F("-0x"));
                Serial.print((uint16_t)(base | 0x0FFu), HEX);
                Serial.println(']');
            }
            break;
        }
        case 'w': {
            // "w156" watches 0x156; a bare "w" clears the whole watchlist.
            uint8_t  n  = 0;
            const uint16_t id = readHexId(n);
            if (n == 0) {
                gWatchUsed = 0;
                Serial.println(F("\n[watchlist cleared]"));
            } else if (watchAdd(id)) {
                Serial.print(F("\n[watching 0x"));
                Serial.print(id, HEX);
                Serial.println(F(" - counts from now, press r to restart the window]"));
            } else {
                Serial.println(F("\n[watchlist full]"));
            }
            break;
        }
        default:  break;
        }
    }

    if (nowMs - gLastReportMs >= REPORT_INTERVAL_MS) {
        // One rendering, one or two sinks. printReport() re-baselines the
        // per-window Hz and change mask on the way out, so it must be called
        // exactly once per interval - rendering separately for the card would
        // either double-reset the window or leave the two copies describing
        // different intervals under the same heading.
        const bool sdDue = gSdOk && (nowMs - gLastSdDumpMs >= SD_DUMP_INTERVAL_MS);
        if (sdDue) {
            const uint32_t t0 = micros();
            TeePrint tee(Serial, gSdFile);
            printReport(tee);
            // flush(), not just write: without it the report sits in the
            // library's block buffer and the directory entry is never updated,
            // so pulling the ignition - which is how every one of these runs
            // ends - would lose the whole file, not just the last minute.
            gSdFile.flush();
            gSdWriteMaxUs = max(gSdWriteMaxUs, micros() - t0);
            gLastSdDumpMs = millis();
            ++gSdDumps;
            watchdogFeed();
        } else {
            printReport(Serial);
        }
        // Printing ~3 KB over USB CDC stalls this loop for as long as the host
        // takes to read it, and the bus keeps running at ~1100 frames/s the
        // whole time, so the RX buffers always overflow during a report. That
        // is an artefact of reporting, not of the drain, and counting it buries
        // any genuine steady-state overrun in constant noise. Clear the flags
        // without counting, and restart the clock AFTER the print so the next
        // window measures a full interval of actual capture.
        mcpBitModify(REG_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0x00);
        gLastReportMs = millis();
    }
}
