#include "can_stream_hw_stub.h"

#include <Arduino.h>

#include "CANStreamHw.h"
#include "CANSniffFunctions.h"

static uint8_t  g_depth;
static bool     g_pending;
static bool     g_running;
static bool     g_inIsr;
static bool     g_preempt;
static uint32_t g_isrRuns;
static uint32_t g_violations;
static uint32_t g_unbalanced;
static uint32_t g_hostLatency;   ///< usbCdcTxIdle() polls of an armed bank before the host takes it.
static uint32_t g_usPerPoll;     ///< micros() advance per usbCdcTxIdle() poll.
static uint32_t g_idlePolls;
static uint32_t g_busyPolls;     ///< Polls that found the current packet still armed.

static void runIsr()
{
    g_inIsr = true;
    canDrainIsr();
    g_inIsr = false;
    ++g_isrRuns;
}

void hostDrainReset()
{
    g_depth      = 0u;
    g_pending    = false;
    g_running    = false;
    g_inIsr      = false;
    g_preempt    = false;
    g_isrRuns    = 0u;
    g_violations = 0u;
    g_unbalanced = 0u;
    g_hostLatency = 0u;
    g_usPerPoll   = 0u;
    g_idlePolls   = 0u;
    g_busyPolls   = 0u;
}

void hostDrainTick()
{
    if (!g_running) return;
    if (g_depth != 0u) { g_pending = true; return; }
    runIsr();
}

void     hostDrainSetPreempt(bool on) { g_preempt = on; }
uint8_t  hostDrainMaskDepth()         { return g_depth; }
bool     hostDrainPending()           { return g_pending; }
bool     hostDrainTimerRunning()      { return g_running; }
uint32_t hostDrainIsrRuns()           { return g_isrRuns; }
uint32_t hostDrainViolations()        { return g_violations; }
uint32_t hostDrainUnbalanced()        { return g_unbalanced; }
void     hostUsbSetHostLatency(uint32_t polls) { g_hostLatency = polls; g_busyPolls = 0u; }
void     hostUsbSetUsPerPoll(uint32_t us)      { g_usPerPoll = us; }
uint32_t hostUsbIdlePolls()                    { return g_idlePolls; }

void hostDrainOnSpiBegin()
{
    if (!g_inIsr && g_depth == 0u && canDrainArmed()) ++g_violations;
}

void hostDrainOnSpiByte()
{
    // A tick falls due at every byte of main-loop SPI, as it would at 10 kHz
    // during any transaction longer than a period. Unmasked, it runs right
    // there, in the middle of the transaction; masked, it waits for the unmask,
    // as the NVIC keeps it pending.
    if (!g_preempt || !g_running || g_inIsr) return;
    if (g_depth == 0u) runIsr();
    else               g_pending = true;
}

// ─── the four functions lib/CANStreamHw.cpp provides on the board ─────────────

void canDrainTimerBegin() { g_running = true; }

void canDrainIrqMask()
{
    if (g_depth < 0xFFu) ++g_depth;
}

void canDrainIrqUnmask()
{
    if (g_depth == 0u) { ++g_unbalanced; return; }
    --g_depth;
    if (g_depth == 0u && g_pending) {
        g_pending = false;
        runIsr();
    }
}

bool usbCdcTxIdle()
{
    // The poll costs time on the board (a register read and the caller's
    // micros()); the model charges it here so microsecond bounds can expire.
    ++g_idlePolls;
    if (g_usPerPoll != 0u) hostSetMicros(micros() + g_usPerPoll);
    // A reading host takes the armed packet on its n-th look, as an IN poll
    // that finally finds BK1RDY set would.
    if (g_hostLatency != 0u && hostUsbBankBusy()) {
        if (++g_busyPolls >= g_hostLatency) {
            hostUsbCollect();
            g_busyPolls = 0u;
        }
    } else {
        g_busyPolls = 0u;
    }
    return hostUsbConfigured() && !hostUsbBankBusy();
}
