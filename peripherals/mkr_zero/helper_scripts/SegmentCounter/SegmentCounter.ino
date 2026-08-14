/**
 * @file SegmentCounter.ino
 * @brief Counts 0..255 on an HT16K33 7-segment backpack, forever.
 *
 * The display is daisy-chained off the IMU's STEMMA QT connector, so it sits on
 * the SAME I2C bus (SERCOM2, D11/D12) as the LSM6DSOX and the GNSS. That is the
 * point of it: a counter that is visibly stepping proves the bus is moving
 * without a serial console attached, and a counter that FREEZES is the bus
 * wedging, seen from across the workshop.
 *
 * That matters on this rig. The bus has been failing progressively rather than
 * cleanly - corrupt reads long before a hard clamp - and the failure has so far
 * only ever been visible in a log read afterwards. A frozen display timestamps
 * the moment it happens against whatever the vehicle was doing.
 *
 * ── NO DISPLAY LIBRARY ────────────────────────────────────────────────────────
 * The HT16K33 is driven directly. Adafruit_LEDBackpack would work, but it is a
 * global unpinned dependency, which this project has a standing rule against
 * (see vendor/CANBus/PATCHES.md for what that rule is protecting). The whole
 * chip is four commands and a 16-byte blit; a library buys nothing here.
 *
 * ── WIRING ────────────────────────────────────────────────────────────────────
 * STEMMA QT from the IMU breakout to the backpack. Nothing else needed - the
 * cable carries 3V3, GND, SDA, SCL. The address is auto-detected across the
 * HT16K33's 0x70..0x77 range, so the solder jumpers can be left alone.
 */

#include <Wire.h>

#ifndef DASHCAM_SAMD_WIRE_REQUESTFROM1_FIX
#error "Stock Wire detected. Build with --library peripherals/mkr_zero/vendor/Wire (see vendor/Wire/README.md); the stock TwoWire::requestFrom() reads an uninitialised busOwner on 1-byte transfers."
#endif

// ─── HT16K33 ──────────────────────────────────────────────────────────────────

static const uint8_t HT16K33_ADDR_FIRST = 0x70;   ///< A2..A0 all open.
static const uint8_t HT16K33_ADDR_LAST  = 0x77;

static const uint8_t HT16K33_CMD_OSC_ON     = 0x21; ///< System setup, oscillator running.
static const uint8_t HT16K33_CMD_DISPLAY_ON = 0x81; ///< Display on, no blink.
static const uint8_t HT16K33_CMD_DIM        = 0xE0; ///< | brightness 0..15.

/// Standard 7-segment bit order, dp-g-f-e-d-c-b-a. Hex digits so the same table
/// serves a decimal or a hex display without a second one.
static const uint8_t kFont[16] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07,
    0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71
};

/// Backpack RAM positions of the four digits. Index 2 is the colon, skipped.
static const uint8_t kDigitPos[4] = { 0, 1, 3, 4 };

static uint8_t  gAddr        = 0;      ///< 0 until the display is found.
static uint32_t gWrites      = 0;
static uint32_t gWriteErrors = 0;
static bool     gFaulted     = false;  ///< Latches the first failure notice.

/// Brightness 0..15, near maximum.
///
/// This is a diagnostic indicator meant to be read across a workshop, and the
/// STEMMA QT chain feeds the backpack 3V3 rather than 5V, which already costs
/// brightness. A dim-but-lit display and a dead one are the same observation
/// from two metres away, and that ambiguity is the one thing this sketch must
/// not introduce.
static const uint8_t BRIGHTNESS = 13;

/// Milliseconds per count. 40 ms gives a visibly smooth sweep and puts ~25
/// transactions/s on the bus - enough to trip a marginal bus, not enough to be
/// the reason it trips.
static const uint32_t STEP_MS = 40;

/** @brief One command byte. @return true when the device ACKed. */
static bool htCommand(uint8_t cmd)
{
    Wire.beginTransmission(gAddr);
    Wire.write(cmd);
    return Wire.endTransmission() == 0;
}

/**
 * @brief Blits all 16 display-RAM bytes.
 *
 * Always the full frame, never a partial update. A partial write that lands
 * during a bus glitch leaves the display showing a mix of two numbers, which
 * reads as a wrong count rather than as a fault - and the entire value of this
 * sketch is that what you see is either right or obviously stopped.
 */
static bool htBlit(const uint8_t *digits, uint8_t count)
{
    Wire.beginTransmission(gAddr);
    Wire.write(uint8_t(0x00));                  // RAM pointer
    for (uint8_t pos = 0; pos < 8u; ++pos) {
        uint8_t seg = 0u;
        for (uint8_t d = 0; d < count; ++d) {
            if (kDigitPos[d] == pos) { seg = digits[d]; break; }
        }
        Wire.write(seg);                        // low byte  = segments a..dp
        Wire.write(uint8_t(0x00));              // high byte = unused here
    }
    return Wire.endTransmission() == 0;
}

/** @brief Right-aligned decimal, blanks ahead of the number. */
static void renderDecimal(uint16_t value, uint8_t *out)
{
    for (uint8_t i = 0; i < 4u; ++i) out[i] = 0x00;
    uint8_t i = 4u;
    do {
        out[--i] = kFont[value % 10u];
        value /= 10u;
    } while (value != 0u && i != 0u);
}

/**
 * @brief Reports every device on the bus, and returns an HT16K33 if one is there.
 *
 * The whole bus is scanned, not just 0x70..0x77, because "the display is dark"
 * has three completely different causes and only a full scan separates them:
 * nothing at all responds (chain unplugged, or the bus is clamped again), the
 * IMU answers but the display does not (the display's own link or power), or
 * everything answers and the fault is in this sketch's driving of it. Printing
 * what IS present costs one pass and turns a guess into a reading.
 *
 * Known addresses on this rig: 0x1C LIS3MDL, 0x42 u-blox GNSS, 0x6A LSM6DSOX,
 * 0x70..0x77 HT16K33.
 */
static uint8_t scanBusAndFindDisplay()
{
    uint8_t found = 0;
    uint8_t display = 0;

    Serial.println(F("\nI2C scan:"));
    for (uint8_t a = 0x08u; a <= 0x77u; ++a) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() != 0) continue;
        ++found;
        Serial.print(F("  0x"));
        if (a < 0x10u) Serial.print('0');
        Serial.print(a, HEX);
        if      (a == 0x1Cu) Serial.print(F("  LIS3MDL (magnetometer)"));
        else if (a == 0x42u) Serial.print(F("  u-blox GNSS"));
        else if (a == 0x6Au || a == 0x6Bu) Serial.print(F("  LSM6DSOX (accel/gyro)"));
        else if (a >= HT16K33_ADDR_FIRST && a <= HT16K33_ADDR_LAST) {
            Serial.print(F("  HT16K33 backpack  <- the display"));
            if (display == 0u) display = a;
        }
        Serial.println();
    }
    if (found == 0u) {
        Serial.println(F("  (nothing responded at all)"));
    }
    return display;
}

/**
 * @brief Lights every segment for a moment.
 *
 * Run before counting so a dark display is never ambiguous. All segments lit
 * proves power, address, oscillator, brightness and the blit path in one look;
 * if the lamp test shows and the count does not, the fault is the counting
 * logic, and if neither shows it is not this sketch's arithmetic at fault.
 */
static bool lampTest()
{
    const uint8_t all[4] = { 0xFFu, 0xFFu, 0xFFu, 0xFFu };
    if (!htBlit(all, 4u)) return false;
    delay(1200);
    return true;
}

void setup()
{
    Serial.begin(115200);
    const uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 3000) { }

    Serial.println();
    Serial.println(F("=============== segment counter ==============="));
    Serial.println(F("Counts 0..255 forever on an HT16K33 backpack."));
    Serial.println(F("Shares the IMU's I2C bus, so a FROZEN display is"));
    Serial.println(F("the bus wedging - watch it, not the console."));

    Wire.begin();
    // 100 kHz, not the 400 kHz the production firmware uses. This sketch is a
    // bus-health indicator; running it at the slower, more forgiving rate means
    // a freeze points at the bus itself rather than at marginal timing this
    // sketch introduced.
    Wire.setClock(100000UL);

    gAddr = scanBusAndFindDisplay();
    if (gAddr == 0u) {
        Serial.println(F("\nNo HT16K33 in 0x70..0x77 - nothing to drive."));
        Serial.println(F("  If NOTHING was listed above, the chain is unplugged or the"));
        Serial.println(F("  bus is clamped again - check with the IMU sketch."));
        Serial.println(F("  If the IMU was listed but the display was not, the fault is"));
        Serial.println(F("  the display's own link: its QT connector, or the cable to it."));
        Serial.println(F("  If it is not an HT16K33 part at all (TM1637, MAX7219 and the"));
        Serial.println(F("  bare 4-digit modules are not), this sketch cannot drive it."));
        return;
    }

    Serial.print(F("\nDriving the display at 0x"));
    Serial.println(gAddr, HEX);

    // Oscillator FIRST: brightness and display-on are ignored while it is off,
    // and the part then sits there ACKing everything with the panel dark.
    if (!htCommand(HT16K33_CMD_OSC_ON) ||
        !htCommand(HT16K33_CMD_DIM | (BRIGHTNESS & 0x0Fu)) ||
        !htCommand(HT16K33_CMD_DISPLAY_ON)) {
        Serial.println(F("Display ACKed its address but refused its setup - bus unhealthy."));
        gAddr = 0u;
        return;
    }

    Serial.println(F("Lamp test: every segment on for 1.2 s."));
    if (!lampTest()) {
        Serial.println(F("  blit FAILED - it answers commands but not display RAM."));
        gAddr = 0u;
        return;
    }
    Serial.println(F("  If that stayed dark, the fault is power or the panel itself,"));
    Serial.println(F("  NOT the address or this sketch - it ACKed every write."));
    Serial.println(F("\nCounting.\n"));
}

void loop()
{
    if (gAddr == 0u) { delay(1000); return; }

    static uint16_t value      = 0;
    static uint32_t lastStepMs = 0;

    if ((millis() - lastStepMs) < STEP_MS) return;
    lastStepMs = millis();

    uint8_t digits[4];
    renderDecimal(value, digits);

    ++gWrites;
    if (!htBlit(digits, 4u)) {
        ++gWriteErrors;
        // Reported on the FIRST failure and then only on the round hundreds.
        // A wedged bus fails every single write, and a message per failure
        // would bury the one line that says when it started.
        if (!gFaulted) {
            gFaulted = true;
            Serial.print(F("WRITE FAILED at count "));
            Serial.print(value);
            Serial.print(F(", after "));
            Serial.print(gWrites);
            Serial.println(F(" good writes - bus is not ACKing."));
        } else if ((gWriteErrors % 100u) == 0u) {
            Serial.print(F("  still failing: "));
            Serial.print(gWriteErrors);
            Serial.println(F(" consecutive"));
        }
    } else if (gFaulted) {
        // Recovery is worth a line too: an intermittent bus that comes back is
        // a different fault from one that stays down, and only the log can
        // tell them apart after the fact.
        gFaulted = false;
        Serial.print(F("recovered after "));
        Serial.print(gWriteErrors);
        Serial.println(F(" failed writes"));
        gWriteErrors = 0;
    }

    value = (value + 1u) & 0xFFu;   // 0..255, wrapping
}
