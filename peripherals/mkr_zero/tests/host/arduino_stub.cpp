#include "Arduino.h"

#include "DataDictionary.h"

#include <string.h>

// ─── pin state ───────────────────────────────────────────────────────────────

static const uint32_t PIN_COUNT = 32u;

static uint8_t  g_mode[PIN_COUNT];
static uint8_t  g_level[PIN_COUNT];
static uint32_t g_millis;

// ─── 74HC165 chain model ─────────────────────────────────────────────────────
//
// Sixteen stages in one word, laid out so bit 15 is the stage feeding Q7 and
// bit 0 is the stage furthest from it (#A's D0 position, fed by #A's DS, which
// the design ties to ground).
//
// Datasheet behaviour, Nexperia 74HC/HCT165 rev 8 §1:
//   PL LOW   - D0..D7 load asynchronously and CONTINUOUSLY, overriding the
//              clock for as long as PL stays low.
//   PL HIGH  - data enters serially at DS.
//   CE LOW   - shift on the LOW-to-HIGH transition of CP. CE is tied to ground
//              on this board, so it is modelled as permanently enabled.
//   Q7       - the output of the final stage; valid with no clock at all
//              immediately after a load, which is what forces read-before-clock
//              on the driver side.

static uint16_t g_panel;
static uint16_t g_shift;
static bool     g_present;
static uint32_t g_loadPulses;
static uint32_t g_clocks;

void hostReset()
{
    memset(g_mode,  0xFF, sizeof(g_mode));
    memset(g_level, 0,    sizeof(g_level));
    g_millis     = 0u;
    g_panel      = 0xFFFFu;
    g_shift      = 0xFFFFu;
    g_present    = true;
    g_loadPulses = 0u;
    g_clocks     = 0u;

    // PL idles HIGH on the real board via its pull-up; start there so the first
    // digitalWrite(LOW) is a genuine falling edge.
    g_level[SWITCH_LOAD_PIN] = HIGH;
}

void fakeChainSetPanel(uint16_t panel) { g_panel = panel; }
void fakeChainSetPresent(bool present) { g_present = present; }
uint32_t fakeChainLoadPulses()         { return g_loadPulses; }
uint32_t fakeChainClocks()             { return g_clocks; }

void hostSetMillis(uint32_t ms) { g_millis = ms; }
uint32_t millis()               { return g_millis; }

uint32_t hostPinMode(uint32_t pin)
{
    return (pin < PIN_COUNT) ? g_mode[pin] : 0xFFu;
}

void pinMode(uint32_t pin, uint32_t mode)
{
    if (pin < PIN_COUNT) g_mode[pin] = static_cast<uint8_t>(mode);
}

void delayMicroseconds(uint32_t us)
{
    // Time is driven by the test, not by elapsed wall clock: a delay that
    // silently advanced millis() would make the chatter-window tests depend on
    // how many microseconds the transport happens to spend.
    (void)us;
}

void digitalWrite(uint32_t pin, uint32_t value)
{
    if (pin >= PIN_COUNT) return;

    const uint8_t prev = g_level[pin];
    const uint8_t now  = static_cast<uint8_t>(value ? HIGH : LOW);
    g_level[pin] = now;

    if (pin == SWITCH_LOAD_PIN) {
        if (prev == HIGH && now == LOW) ++g_loadPulses;
        // Asynchronous and level-sensitive, not edge-sensitive.
        if (now == LOW) g_shift = g_panel;
        return;
    }

    if (pin == SWITCH_CLK_PIN) {
        if (prev == LOW && now == HIGH) {
            ++g_clocks;
            // A load in progress wins over the clock, per the datasheet.
            if (g_level[SWITCH_LOAD_PIN] == LOW) {
                g_shift = g_panel;
            } else {
                // Toward the output. #A's DS is tied to GND, so zeros enter.
                g_shift = static_cast<uint16_t>(g_shift << 1);
            }
        }
    }
}

int digitalRead(uint32_t pin)
{
    if (pin >= PIN_COUNT) return LOW;

    if (pin == SWITCH_DATA_PIN) {
        if (!g_present) {
            // Nothing driving the line: it sits at whatever the MCU's own bias
            // puts it at. The production configuration is INPUT_PULLUP, and a
            // deterministic HIGH here is exactly the property that makes an
            // absent chain always fail the sentinel test rather than pass it a
            // quarter of the time.
            if (g_mode[pin] == INPUT_PULLUP)   return HIGH;
            if (g_mode[pin] == INPUT_PULLDOWN) return LOW;
            return LOW;   // bare INPUT: undefined on real hardware, see README
        }
        return ((g_shift & 0x8000u) != 0u) ? HIGH : LOW;
    }

    return g_level[pin];
}
