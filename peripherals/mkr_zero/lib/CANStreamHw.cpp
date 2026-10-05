#include <Arduino.h>

#include "CANStreamHw.h"
#include "CANSniffFunctions.h"   // canDrainIsr()

// SAMD21 registers throughout, which is why tests/host replaces this file with a
// model (can_stream_hw_stub.cpp) rather than compiling it. Keep it thin: every
// decision belongs in CANSniffFunctions.cpp or CANRawStream.cpp, where it is
// tested; what is left here is register plumbing a bench run proves or breaks.

/// CC0 at 48 MHz with no prescaler; MFRQ restarts the count at CC0, so the
/// period is (CC0 + 1) ticks. 16 bits caps the period at ~1.36 ms.
static const uint32_t kDrainTopTicks = (F_CPU / 1000000UL) * CAN_DRAIN_PERIOD_US - 1UL;
static_assert(kDrainTopTicks <= 0xFFFFu, "CAN_DRAIN_PERIOD_US does not fit TC3's 16-bit CC0 at 48 MHz");

/// Nesting depth of canDrainIrqMask(). Main-loop only: the ISR never masks
/// itself, so there is exactly one writer and no need for anything stronger.
static uint8_t gMaskDepth = 0;
static bool    gTimerRunning = false;

static void tc3Sync()
{
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY) {
        // TC registers live in the GCLK domain; writes land a few cycles later
    }
}

void canDrainTimerBegin()
{
    if (gTimerRunning) return;
    NVIC_DisableIRQ(TC3_IRQn);

    // GCLK0 (48 MHz) to the channel TCC2 and TC3 share. The APB bus clock is
    // already on: init() (wiring.c) enables every TC/TCC at boot.
    GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 |
                                   GCLK_CLKCTRL_ID(GCM_TCC2_TC3));
    while (GCLK->STATUS.bit.SYNCBUSY) {}

    TC3->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
    while (TC3->COUNT16.CTRLA.bit.SWRST) {}

    // 16-bit counter in match-frequency mode: TOP is CC0, so CC0 alone is the
    // period and the MC0 flag is the tick.
    TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MFRQ | TC_CTRLA_PRESCALER_DIV1;
    tc3Sync();
    TC3->COUNT16.CC[0].reg = (uint16_t)kDrainTopTicks;
    tc3Sync();
    TC3->COUNT16.INTFLAG.reg  = TC_INTFLAG_MC0;
    TC3->COUNT16.INTENSET.reg = TC_INTENSET_MC0;

    // Priority set while the line is disabled. See CANStreamHw.h for the level.
    NVIC_SetPriority(TC3_IRQn, CAN_DRAIN_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(TC3_IRQn);

    TC3->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
    tc3Sync();
    gTimerRunning = true;

    // Honour a mask taken before the timer existed: setup() may already be
    // inside a CAN bring-up that holds one.
    if (gMaskDepth == 0u) NVIC_EnableIRQ(TC3_IRQn);
}

extern "C" void TC3_Handler(void)
{
    // Acknowledged FIRST. A tick that falls due while the drain is still
    // running then re-pends and runs straight after, instead of being cleared
    // unseen by a late acknowledge.
    TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
    canDrainIsr();
}

void canDrainIrqMask()
{
    NVIC_DisableIRQ(TC3_IRQn);
    // The disable must have taken effect before the caller's first SPI byte.
    // CMSIS 4.5's NVIC_DisableIRQ is a bare ICER store; ARM's guidance for
    // disabling an interrupt is to follow it with DSB + ISB, which later CMSIS
    // releases build in.
    __DSB();
    __ISB();
    if (gMaskDepth < 0xFFu) ++gMaskDepth;
}

void canDrainIrqUnmask()
{
    // Unbalanced calls are ignored rather than allowed to re-enable the drain
    // underneath an outer holder that still believes it is masked.
    if (gMaskDepth == 0u) return;
    --gMaskDepth;
    if (gMaskDepth == 0u && gTimerRunning) NVIC_EnableIRQ(TC3_IRQn);
}

bool usbCdcTxIdle()
{
    // Not enumerated: USBDeviceClass::send() returns at once without sending,
    // so there is nothing to wait for — and nothing to write to either.
    if (!USBDevice.configured()) return false;

    // The CDC data IN endpoint, found by its type rather than assumed. The core
    // numbers endpoints at plug time (PluggableUSB): CDC is the only module, so
    // ACM = 1, OUT = 2, IN = 3 — but a library that plugged first would shift
    // them, and a wrong guess here would read another endpoint's BK1RDY. It is
    // the only bulk IN endpoint this firmware has (initEP: EPTYPE1 3 = bulk IN).
    static uint8_t ep = 0;
    if (ep == 0u || USB->DEVICE.DeviceEndpoint[ep].EPCFG.bit.EPTYPE1 != 3u) {
        ep = 0u;
        for (uint8_t e = 1; e < USB_EPT_NUM; ++e) {
            if (USB->DEVICE.DeviceEndpoint[e].EPCFG.bit.EPTYPE1 == 3u) { ep = e; break; }
        }
        if (ep == 0u) return false;
    }

    // BK1RDY set = a packet is armed and the host has not collected it yet,
    // which is exactly the state in which send() would spin (USBCore.cpp).
    return USB->DEVICE.DeviceEndpoint[ep].EPSTATUS.bit.BK1RDY == 0u;
}
