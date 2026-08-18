/**
 * @file SwitchProbe.ino
 * @brief Bring-up for the 74HC165 switch chain. Verifies wiring, then positions.
 *
 * Standalone by design — it drives the three pins directly rather than linking
 * lib/SwitchFunctions.cpp, because the thing being commissioned here is the
 * WIRING, and a probe that shares its transport with the code under test cannot
 * distinguish "the chain is wrong" from "the driver is wrong". The bit loop
 * below is a deliberate duplicate of the one in SwitchFunctions.cpp for exactly
 * that reason; if they ever disagree, that disagreement is the finding.
 *
 * ── WHAT IT SHOWS ────────────────────────────────────────────────────────────
 *   SENTINELS   the two hard-wired inputs, checked on every read. Until this
 *               says OK nothing else on the line means anything.
 *   raw         the full 16-bit word exactly as shifted in, sentinels included
 *   SW00..      one column per switch, '1' = closed
 *
 * A line is printed on CHANGE, plus one per second regardless, so a wire that
 * is intermittent shows up as traffic rather than needing to be caught live.
 *
 * ── COMMISSIONING ORDER ──────────────────────────────────────────────────────
 *   1. Power up with NO switches connected. Sentinels must read OK and every
 *      SW column must read 0. If sentinels fail, stop — nothing downstream is
 *      meaningful. raw = 0xFFFF with sentinels failing is a chain that is
 *      absent, unpowered, or has SH/LD or CLK swapped.
 *   2. Short each switch input to ground in turn and confirm ONE column moves,
 *      and that it is the column you expect. This is the only step that
 *      actually proves the harness, and it is why the columns are numbered.
 *   3. Fit the switches and confirm each throw is clean — a column that flickers
 *      while you hold it steady is a contact or a connector, not software.
 *
 * ── WIRING ───────────────────────────────────────────────────────────────────
 *   D0 -> SH/LD  (pin 1, both packages)
 *   D1 -> CLK    (pin 2, both packages)
 *   D2 <- Q7     (pin 9 of the LAST package only)
 *   pin 15 (CLK INH) -> GND on BOTH packages. Easily missed; the symptom is a
 *   word that never changes no matter what you do to the inputs.
 *   pin 10 (DS): #A to GND, #B to #A's pin 9.
 *   Sentinels: #A pin 11 -> GND, #A pin 12 -> 3V3.
 *   Switch inputs: 10 kOhm to 3V3 each, switch to GND. ALL of them, including
 *   any you have not wired a switch to yet — a floating input reads as noise.
 *
 * 3.3 V ONLY. These are 74HC parts on the MKR's own rail; nothing here is 5 V
 * tolerant.
 */

#include <Arduino.h>

// Kept as literals rather than including DataDictionary.h: this sketch is meant
// to be readable on its own at a bench, and to still work if someone is
// checking a board whose pin assignment has been changed but not yet committed.
static const uint8_t  PIN_LOAD  = 0;   ///< SH/LD, active low.
static const uint8_t  PIN_CLK   = 1;
static const uint8_t  PIN_DATA  = 2;

static const uint8_t  REGISTERS = 2;
static const uint8_t  CHAIN_BITS = REGISTERS * 8;
static const uint8_t  SENTINEL_BITS = 2;
static const uint8_t  SWITCH_COUNT  = CHAIN_BITS - SENTINEL_BITS;

static const uint16_t SENTINEL_MASK     = 0x0003u;
static const uint16_t SENTINEL_EXPECTED = 0x0002u;  ///< bit1 high, bit0 low.

static const unsigned long POLL_MS      = 20;
static const unsigned long HEARTBEAT_MS = 1000;
static const unsigned long SERIAL_WAIT_MS = 2000;

static uint16_t      lastRaw     = 0xFFFFu;
static bool          havePrinted = false;
static unsigned long lastPollMs  = 0;
static unsigned long lastBeatMs  = 0;
static uint32_t      reads       = 0;
static uint32_t      rejected    = 0;

/** @brief One load-and-shift. Read before clocking — see the comment inside. */
static uint16_t readChain()
{
    digitalWrite(PIN_LOAD, LOW);
    delayMicroseconds(1);
    digitalWrite(PIN_LOAD, HIGH);
    delayMicroseconds(1);

    uint16_t raw = 0;
    for (uint8_t i = 0; i < CHAIN_BITS; ++i) {
        // Q7 already presents D7 after the load pulse, with no clock edge.
        // Clocking first silently drops that bit and shifts every other one,
        // which looks exactly like a miswired harness.
        raw = (uint16_t)(raw << 1);
        if (digitalRead(PIN_DATA) == HIGH) raw = (uint16_t)(raw | 1u);
        digitalWrite(PIN_CLK, HIGH);
        digitalWrite(PIN_CLK, LOW);
    }
    return raw;
}

static void printHeader()
{
    Serial.println();
    Serial.print("  sentinels  raw     ");
    for (uint8_t i = 0; i < SWITCH_COUNT; ++i) {
        Serial.print(" ");
        if (i < 10) Serial.print("0");
        Serial.print(i);
    }
    Serial.println();
}

static void printLine(uint16_t raw)
{
    const bool ok = ((raw & SENTINEL_MASK) == SENTINEL_EXPECTED);

    Serial.print(ok ? "  OK         " : "  FAIL       ");

    Serial.print("0x");
    for (int8_t nib = 3; nib >= 0; --nib) {
        Serial.print((raw >> (nib * 4)) & 0x0Fu, HEX);
    }

    if (!ok) {
        // No positions printed at all when the control fails. Printing them
        // anyway would invite reading switch states out of a word that has just
        // proved it is not describing the switches.
        Serial.print("   <- sentinels wrong: expect bit0=0 (pin 11 to GND), bit1=1 (pin 12 to 3V3)");
        Serial.println();
        return;
    }

    // Inverted here, not in readChain(): closed pulls the input low.
    const uint16_t positions = (uint16_t)((uint16_t)(~raw) >> SENTINEL_BITS);
    Serial.print("  ");
    for (uint8_t i = 0; i < SWITCH_COUNT; ++i) {
        Serial.print("  ");
        Serial.print((positions >> i) & 1u);
    }
    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    // Bounded. An unbounded wait here hangs the board whenever it is powered
    // from anything but a host that opens the port.
    const unsigned long t0 = millis();
    while (!Serial && (millis() - t0) < SERIAL_WAIT_MS) { /* up to 2 s */ }

    digitalWrite(PIN_LOAD, HIGH);
    digitalWrite(PIN_CLK, LOW);
    pinMode(PIN_LOAD, OUTPUT);
    pinMode(PIN_CLK, OUTPUT);
    digitalWrite(PIN_LOAD, HIGH);
    digitalWrite(PIN_CLK, LOW);

    // INPUT_PULLUP, matching lib/SwitchFunctions.cpp. A floating pin picks up
    // coupling from the clock beside it and produces random-looking words, and
    // random words satisfy a two-bit sentinel pattern about one read in four —
    // so an absent chain would appear to be present now and then. Pulled up it
    // reads 0xFFFF, which fails the pattern on bit 0 every single time.
    //
    // Q7 overrides it by two orders of magnitude: the datasheet gives an
    // absolute maximum output current of +/-25 mA against 55-165 uA from the
    // SAMD21 pull-up.
    pinMode(PIN_DATA, INPUT_PULLUP);

    pinMode(LED_BUILTIN, OUTPUT);

    Serial.println();
    Serial.println("=== 74HC165 switch probe ===");
    Serial.print("registers=");
    Serial.print(REGISTERS);
    Serial.print("  inputs=");
    Serial.print(SWITCH_COUNT);
    Serial.println("  pins: D0=SH/LD  D1=CLK  D2=Q7");
    Serial.println("Short an input to GND and exactly one column must move.");
    printHeader();

    lastPollMs = millis();
    lastBeatMs = lastPollMs;
}

void loop()
{
    const unsigned long now = millis();

    if ((now - lastPollMs) < POLL_MS) return;
    lastPollMs = now;

    const uint16_t raw = readChain();
    ++reads;
    if ((raw & SENTINEL_MASK) != SENTINEL_EXPECTED) ++rejected;

    // The LED follows the sentinel check, so the chain can be commissioned with
    // no console attached at all — solid means the control is passing.
    digitalWrite(LED_BUILTIN,
                 ((raw & SENTINEL_MASK) == SENTINEL_EXPECTED) ? HIGH : LOW);

    const bool changed  = (!havePrinted) || (raw != lastRaw);
    const bool beat     = (now - lastBeatMs) >= HEARTBEAT_MS;

    if (changed || beat) {
        printLine(raw);
        lastRaw     = raw;
        havePrinted = true;
        if (beat) lastBeatMs = now;
    }

    // Re-print the column header periodically so a long session stays readable
    // without scrolling back. Cheap, and it also marks elapsed time.
    if ((reads % 1000u) == 0u) {
        Serial.print("  (reads=");
        Serial.print(reads);
        Serial.print("  rejected=");
        Serial.print(rejected);
        Serial.println(")");
        printHeader();
    }
}
