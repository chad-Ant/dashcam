#include "Arduino.h"

#include "DataDictionary.h"

#include <string.h>

HostSerial Serial;
HostUart   Serial1;

// ─── pin state ───────────────────────────────────────────────────────────────

static const uint32_t PIN_COUNT = 32u;

static uint8_t  g_mode[PIN_COUNT];
static uint8_t  g_level[PIN_COUNT];
static uint32_t g_millis;
static uint32_t g_micros;

// ─── USB console bank model ──────────────────────────────────────────────────

static const size_t USB_OUT_CAP = 1u << 20;   // plenty for any one test
static char     g_usbOut[USB_OUT_CAP + 1];
static size_t   g_usbOutLen;
static bool     g_usbDtr;
static bool     g_usbConfigured;
static bool     g_usbBusy;
static bool     g_usbWriteFails;
static int      g_usbRoom;
static uint32_t g_usbWrites;
static uint32_t g_usbBlocking;
static uint32_t g_usbLargest;

// One-shot hook on the next digitalRead() of one pin, run just before or just
// after the level is sampled: an interrupt landing inside the driver's window.
static uint32_t g_hookPin = 0xFFFFFFFFu;
static bool     g_hookBefore = false;
static void   (*g_hookFn)() = nullptr;

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
    g_micros     = 0u;
    g_usbOutLen     = 0u;
    g_usbOut[0]     = '\0';
    g_usbDtr        = false;
    g_usbConfigured = true;
    g_usbBusy       = false;
    g_usbWriteFails = false;
    g_usbRoom       = 63;
    g_usbWrites     = 0u;
    g_usbBlocking   = 0u;
    g_usbLargest    = 0u;
    g_panel      = 0xFFFFu;
    g_shift      = 0xFFFFu;
    g_present    = true;
    g_loadPulses = 0u;
    g_clocks     = 0u;
    g_hookFn     = nullptr;

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
void hostSetMicros(uint32_t us) { g_micros = us; }
uint32_t micros()               { return g_micros; }

// ─── USB console ─────────────────────────────────────────────────────────────

bool HostSerial::dtr()               { return g_usbDtr; }
int  HostSerial::availableForWrite() { return g_usbRoom; }

size_t HostSerial::write(const uint8_t *buf, size_t n)
{
    // Not enumerated: the core's send() returns -1 at once, and Serial_::write()
    // passes it on as (size_t)-1 — the "r > 0" test there is on a uint32_t.
    if (!g_usbConfigured || g_usbWriteFails) return (size_t)-1;
    // Armed and uncollected: the core would spin here for up to 70 ms.
    if (g_usbBusy) ++g_usbBlocking;
    if (n > g_usbLargest) g_usbLargest = (uint32_t)n;
    for (size_t i = 0; i < n && g_usbOutLen < USB_OUT_CAP; ++i) g_usbOut[g_usbOutLen++] = (char)buf[i];
    g_usbOut[g_usbOutLen] = '\0';
    g_usbBusy = true;
    ++g_usbWrites;
    return n;
}

void        hostUsbSetDtr(bool high)        { g_usbDtr = high; }
void        hostUsbSetConfigured(bool on)   { g_usbConfigured = on; }
void        hostUsbSetRoom(int bytes)       { g_usbRoom = bytes; }
void        hostUsbSetWriteFails(bool fail) { g_usbWriteFails = fail; }
void        hostUsbCollect()                { g_usbBusy = false; }
bool        hostUsbBankBusy()               { return g_usbBusy; }
bool        hostUsbConfigured()             { return g_usbConfigured; }
const char *hostUsbOutput()                 { return g_usbOut; }
void        hostUsbClearOutput()            { g_usbOutLen = 0u; g_usbOut[0] = '\0'; g_usbWrites = 0u; g_usbBlocking = 0u; g_usbLargest = 0u; }
uint32_t    hostUsbWrites()                 { return g_usbWrites; }
uint32_t    hostUsbBlockingWrites()         { return g_usbBlocking; }
uint32_t    hostUsbLargestWrite()           { return g_usbLargest; }
void delay(uint32_t ms)          { g_millis += ms; }

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

// One-shot hook on the next digitalRead() of one pin (declared above).

void hostOnNextRead(uint32_t pin, bool beforeSample, void (*fn)())
{
    g_hookPin = pin; g_hookBefore = beforeSample; g_hookFn = fn;
}

static int sampleLevel(uint32_t pin);

int digitalRead(uint32_t pin)
{
    if (pin != g_hookPin || g_hookFn == nullptr) return sampleLevel(pin);
    void (*fn)() = g_hookFn;
    g_hookFn = nullptr;
    if (g_hookBefore) fn();
    const int level = sampleLevel(pin);
    if (!g_hookBefore) fn();
    return level;
}

static int sampleLevel(uint32_t pin)
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
