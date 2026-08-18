/**
 * ESP_Sentinel — telemetry bridge.
 *
 * Sits between the MKR Zero (which owns OBD2 + GPS) and the Jetson Orin Nano
 * (which runs the dashcam application), speaking the same framed protocol on
 * both sides:
 *
 *   MKR Zero ──UART1 115200, CommProtocol.h──> [ESP32-C3] ──USB-C, HostProtocol.h──> Jetson
 *                 (C3 is initiator)                            (C3 is responder)
 *
 * Downward (commLink) it asks the MKR for a 10 Hz stream and re-asks if the
 * link goes quiet.  Upward (hostLink) it answers the Jetson's commands and
 * forwards each master snapshot as it lands — event-driven, so the Jetson never
 * receives the same sample twice and never receives a stale one.  A 1 Hz
 * MSG_STATUS heartbeat carries bridge health regardless of whether telemetry is
 * flowing, which is what lets the Jetson tell "USB gone" from "MKR gone".
 *
 * Wiring:
 *   MKR Zero Serial1 TX (pin 14) -> C3 RX (GPIO20)
 *   MKR Zero Serial1 RX (pin 13) <- C3 TX (GPIO21)
 *   GND <-> GND
 *   C3 USB-C -> any USB port on the Jetson Orin Nano  (enumerates as /dev/ttyACM*)
 *
 * Board settings: USB CDC On Boot = **Enabled** (Serial is the native USB link
 * to the Jetson; UART0's GPIO20/21 are then free for Serial1/commLink).
 *
 * NOTE: Serial carries binary frames — never print to it.  Use bridgeLog().
 */

#include <Arduino.h>
#include <esp_log.h>      // esp_log_level_set()
#include <esp_system.h>   // esp_reset_reason()
#include <math.h>
#include <stddef.h>
#include <string.h>

#include "commLink.h"
#include "hostLink.h"

// ─── build identity ───────────────────────────────────────────────────────────

static constexpr uint8_t  BRIDGE_FW_MAJOR = 1;
static constexpr uint8_t  BRIDGE_FW_MINOR = 0;

static constexpr uint32_t STATUS_INTERVAL_MS  = 1000; ///< MSG_STATUS heartbeat period.
static constexpr uint32_t MASTER_STALE_MS     = 1000; ///< No master telemetry for this long = link down.
static constexpr uint32_t MASTER_RETRY_MS     = 1000; ///< How often to re-request the master stream while down.
/**
 * Downstream heartbeat period (ms).
 *
 * Sent even while the link is healthy, which is the whole point.  This bridge
 * is silent by design once telemetry is flowing — it only speaks to re-request a
 * stopped stream — so from the MKR's side a working link and an unplugged cable
 * look identical: writing to a severed UART succeeds exactly like writing to a
 * live one.  A periodic PING is the only thing that lets the master distinguish
 * them, and it answers with PONG for free.
 *
 * Comfortably shorter than the master's COMM_LINK_SILENT_MS (8 s) so normal
 * jitter never reads as a disconnect.
 */
static constexpr uint32_t MASTER_HEARTBEAT_MS = 2000;
static constexpr uint32_t ONCE_TIMEOUT_MS     = 500;  ///< A latched CMD_GET_ONCE is abandoned after this.

// The two hops carry the same snapshot under different type names (one header is
// Arduino-only, the other portable).  Assert the layouts agree so a change to one
// that is not mirrored in the other fails the build instead of silently shifting
// every field on the wire.
//
// The size is deliberately NOT written as a literal here: it lives in exactly one
// place per header (their own static_asserts), and repeating it in a comment is
// how the previous "79-byte snapshot" note came to be wrong two versions running.
static_assert(sizeof(TelemetryPayload) == sizeof(hostproto::Telemetry),
              "TelemetryPayload and hostproto::Telemetry must stay the same size");
static_assert(offsetof(TelemetryPayload, masterMillis) == offsetof(hostproto::Telemetry, masterMillis),
              "TelemetryPayload/hostproto::Telemetry field order diverged (masterMillis)");
static_assert(offsetof(TelemetryPayload, accel) == offsetof(hostproto::Telemetry, accel),
              "TelemetryPayload/hostproto::Telemetry field order diverged (accel)");
static_assert(offsetof(TelemetryPayload, latitude) == offsetof(hostproto::Telemetry, latitude),
              "TelemetryPayload/hostproto::Telemetry field order diverged (latitude)");
// Both ends of the IMU block, so a field inserted or reordered inside it is
// caught rather than only a change to its overall length.
static_assert(offsetof(TelemetryPayload, imuAccelX) == offsetof(hostproto::Telemetry, imuAccelX),
              "TelemetryPayload/hostproto::Telemetry field order diverged (imuAccelX)");
static_assert(offsetof(TelemetryPayload, imuTempC) == offsetof(hostproto::Telemetry, imuTempC),
              "TelemetryPayload/hostproto::Telemetry field order diverged (imuTempC)");
static_assert(offsetof(TelemetryPayload, imuAccelPeak) == offsetof(hostproto::Telemetry, imuAccelPeak),
              "TelemetryPayload/hostproto::Telemetry field order diverged (imuAccelPeak)");
// The v0x06 fusion and CAN-map block, anchored at both ends on the same
// principle as the blocks around it. Without these, twelve bytes could be
// inserted in a different ORDER in the two headers and every assertion here
// would still pass — the sizes would match, and flags below would land at the
// same offset, while imuYawRelDeg and canMapChecksum quietly swapped places.
static_assert(offsetof(TelemetryPayload, imuLinAccelPeak) == offsetof(hostproto::Telemetry, imuLinAccelPeak),
              "TelemetryPayload/hostproto::Telemetry field order diverged (imuLinAccelPeak)");
static_assert(offsetof(TelemetryPayload, imuCalib) == offsetof(hostproto::Telemetry, imuCalib),
              "TelemetryPayload/hostproto::Telemetry field order diverged (imuCalib)");
static_assert(offsetof(TelemetryPayload, canMapChecksum) == offsetof(hostproto::Telemetry, canMapChecksum),
              "TelemetryPayload/hostproto::Telemetry field order diverged (canMapChecksum)");
static_assert(offsetof(TelemetryPayload, canMapFlags) == offsetof(hostproto::Telemetry, canMapFlags),
              "TelemetryPayload/hostproto::Telemetry field order diverged (canMapFlags)");
// The vehicle-bus tail. Checked at BOTH ends of the block, not just the start:
// the fields between are mixed 1/2-byte and the array is last, so a single
// anchor would pass while an interior field had drifted.
static_assert(offsetof(TelemetryPayload, canMode) == offsetof(hostproto::Telemetry, canMode),
              "canMode offset differs between the two protocol headers");
static_assert(offsetof(TelemetryPayload, steerMotorTorque) == offsetof(hostproto::Telemetry, steerMotorTorque),
              "steerMotorTorque offset differs between the two protocol headers");
static_assert(offsetof(TelemetryPayload, yawRateCdps) == offsetof(hostproto::Telemetry, yawRateCdps),
              "yawRateCdps offset differs between the two protocol headers");
static_assert(offsetof(TelemetryPayload, wheelRaw) == offsetof(hostproto::Telemetry, wheelRaw),
              "wheelRaw offset differs between the two protocol headers");
static_assert(offsetof(TelemetryPayload, flags) == offsetof(hostproto::Telemetry, flags),
              "TelemetryPayload/hostproto::Telemetry field order diverged (flags)");
// The v0x07 additions, anchored at both ends of each block on the same principle
// as everything above: the linear-accel vector is three consecutive floats and
// the switch pair two consecutive uint16s, so a single anchor per block would
// pass while an interior field had been transposed between the two headers.
static_assert(offsetof(TelemetryPayload, imuLinAccelX) == offsetof(hostproto::Telemetry, imuLinAccelX),
              "TelemetryPayload/hostproto::Telemetry field order diverged (imuLinAccelX)");
static_assert(offsetof(TelemetryPayload, imuLinAccelZ) == offsetof(hostproto::Telemetry, imuLinAccelZ),
              "TelemetryPayload/hostproto::Telemetry field order diverged (imuLinAccelZ)");
static_assert(offsetof(TelemetryPayload, imuHighGMs) == offsetof(hostproto::Telemetry, imuHighGMs),
              "TelemetryPayload/hostproto::Telemetry field order diverged (imuHighGMs)");
static_assert(offsetof(TelemetryPayload, imuHighGCount) == offsetof(hostproto::Telemetry, imuHighGCount),
              "TelemetryPayload/hostproto::Telemetry field order diverged (imuHighGCount)");
static_assert(offsetof(TelemetryPayload, switchState) == offsetof(hostproto::Telemetry, switchState),
              "TelemetryPayload/hostproto::Telemetry field order diverged (switchState)");
static_assert(offsetof(TelemetryPayload, switchChanged) == offsetof(hostproto::Telemetry, switchChanged),
              "TelemetryPayload/hostproto::Telemetry field order diverged (switchChanged)");

// ─── state ────────────────────────────────────────────────────────────────────

static CommLink gLink;  // not 'link' — collides with POSIX link() from <unistd.h>
static HostLink gHost;

// Boot counter, part of the forensic trail the Jetson receives in MSG_HELLO: a
// climbing count with a short bridgeMillis is a C3 stuck in a reset loop.
//
// RTC_NOINIT_ATTR — not RTC_DATA_ATTR — is what survives a watchdog or software
// reset: RTC_DATA_ATTR is re-initialised by the bootloader on every reset other
// than a deep-sleep wake, which would zero the counter in exactly the crash case
// it exists to diagnose.  NOINIT holds garbage on the first power-on, hence the
// magic-word guard.
static constexpr uint32_t BOOT_MAGIC = 0xB0071CEDu;
RTC_NOINIT_ATTR static uint32_t gBootMagic;
RTC_NOINIT_ATTR static uint16_t gBootCount;

static uint32_t gLastStatusMs   = 0;
static uint32_t gLastMasterMs   = 0;
static uint32_t gLastRetryMs    = 0;
static bool     gMasterSeen     = false; ///< False until the first master frame ever.
static bool     gMasterUp       = false; ///< Edge-detect for logging the master link state.
static bool     gForwardOnce    = false; ///< A CMD_GET_ONCE is waiting for the next master frame.
static uint32_t gForwardOnceMs  = 0;     ///< millis() when that request was latched (expiry clock).
static bool     gOncePendingTx  = false; ///< CMD_GET_ONCE not yet handed to the MKR (its TX was busy).
static uint8_t  gDecimCount     = 0;

// ─── helpers ──────────────────────────────────────────────────────────────────

/** @brief Sends one log line up to the Jetson. Silently dropped when no host is attached. */
static void bridgeLog(uint8_t level, const char *text)
{
    (void)gHost.sendLog(level, text); // best-effort diagnostics; never gate control flow on it
}

/** @brief Milliseconds since the last master telemetry, or UINT32_MAX if none ever arrived. */
static uint32_t masterAgeMs(uint32_t nowMs)
{
    return gMasterSeen ? (nowMs - gLastMasterMs) : UINT32_MAX;
}

/** @brief Copies the master snapshot into the portable host-hop struct. */
static void toHostTelemetry(const TelemetryPayload &src, hostproto::Telemetry &dst)
{
    // Layout equality is asserted at compile time above, so a flat copy is
    // correct and keeps the two contracts from having to know each other.
    memcpy(&dst, &src, sizeof(dst));
}

/** @brief Fills the 1 Hz health frame. */
static void buildStatus(uint32_t nowMs, hostproto::BridgeStatus &s)
{
    memset(&s, 0, sizeof(s));

    s.bridgeMillis    = nowMs;
    s.telemetryAgeMs  = masterAgeMs(nowMs);
    s.masterFrames    = gLink.framesRx();
    s.masterCrcErrors = gLink.crcErrors();
    s.hostFrames      = gHost.framesRx();
    s.hostTxDropped   = gHost.txDropped();

    // No PowerManager is instantiated by default: this sketch must not drive a
    // charge-enable GPIO on hardware whose wiring it cannot verify.  The fields
    // are reported as "unavailable" and BRIDGE_FLAG_PM_PRESENT stays clear.
    // To enable a fitted BMS, construct a PowerManager (see powerManager.h),
    // call update() in loop(), fill the four fields below from its getters, and
    // set BRIDGE_FLAG_PM_PRESENT plus the battery flags.
    s.batteryVolts   = NAN;
    s.batteryPercent = NAN;
    s.batteryStatus  = 0xFF;
    s.chargeState    = 0xFF;

    s.tempC = temperatureRead(); // C3 die temperature, not ambient

    const uint32_t freeKb = ESP.getFreeHeap() / 1024u;
    s.freeHeapKb = (freeKb > static_cast<uint32_t>(UINT16_MAX)) ? UINT16_MAX
                                                                : static_cast<uint16_t>(freeKb);

    uint8_t flags = 0;
    if (gMasterSeen && masterAgeMs(nowMs) < MASTER_STALE_MS) flags |= hostproto::BRIDGE_FLAG_MASTER_LINK;
    if (gHost.streaming())                                    flags |= hostproto::BRIDGE_FLAG_STREAMING;
    s.flags = flags;

    s.decimation = gHost.decimation();
}

/** @brief Answers a fresh host connection with its identity frame and an immediate status. */
static void greetHost(uint32_t nowMs)
{
    hostproto::Hello h;
    h.protoVersion   = hostproto::VERSION;
    h.fwMajor        = BRIDGE_FW_MAJOR;
    h.fwMinor        = BRIDGE_FW_MINOR;
    h.resetReason    = static_cast<uint8_t>(esp_reset_reason());
    h.telemetryBytes = static_cast<uint8_t>(sizeof(hostproto::Telemetry));
    h.statusBytes    = static_cast<uint8_t>(sizeof(hostproto::BridgeStatus));
    h.bootCount      = gBootCount;
    h.bridgeMillis   = nowMs;
    (void)gHost.sendHello(h);

    hostproto::BridgeStatus s;
    buildStatus(nowMs, s);
    (void)gHost.sendStatus(s);
    gLastStatusMs = nowMs;

    bridgeLog(hostproto::LOG_INFO, "bridge ready");
}

/**
 * @brief Abandons a latched one-shot that could not be answered in time.
 *
 * Called every loop() rather than from forwardTelemetry(), because the case
 * that needs the timeout most is the one where no master frame ever arrives —
 * exactly when forwardTelemetry() is never reached.  Past the deadline the
 * requester has moved on, and an unexpected late frame is worse than none.
 */
static void expireOneShot(uint32_t nowMs)
{
    if (!gForwardOnce && !gOncePendingTx) return;

    // Still inside the window: keep retrying the downstream hand-off.  The MKR's
    // TX buffer is only briefly full, so a request that lost one race is worth
    // re-offering rather than discarding — dropping it leaves the host waiting
    // on a reply that was never even asked for.
    if ((nowMs - gForwardOnceMs) < ONCE_TIMEOUT_MS) {
        if (gOncePendingTx && gLink.requestOnce()) gOncePendingTx = false;
        return;
    }

    if (gOncePendingTx)
        bridgeLog(hostproto::LOG_WARN, "CMD_GET_ONCE expired: master TX stayed busy");
    else if (gForwardOnce)
        bridgeLog(hostproto::LOG_WARN, "CMD_GET_ONCE expired: no master frame in time");

    gForwardOnce   = false;
    gOncePendingTx = false;
}

/** @brief Forwards one master snapshot upward, honouring streaming state and decimation. */
static void forwardTelemetry()
{
    const bool oneShot = gForwardOnce;

    if (!oneShot) {
        if (!gHost.streaming()) return;
        // Decimation gate: forward every Nth master frame.  decimation() is
        // guaranteed >= 1 by hostLink (CMD_SET_DECIM rejects 0).
        //
        // AN EVENT OVERRIDES IT. Decimation is a bandwidth preference about
        // routine sampling, and it was being applied to a High-G latch as though
        // an impact were routine — so at a decimation of 5 the frame carrying an
        // impact had a 4-in-5 chance of being dropped for arriving on the wrong
        // count. The flags are sticky, so the event would eventually ride out on
        // a later frame; sending it now removes up to half a second of delay from
        // the one signal that exists because delay loses data.
        if (!gLink.hasUrgent() && (++gDecimCount < gHost.decimation())) return;
    }
    gDecimCount = 0;

    // The snapshot is the newest master frame; the merge restores what the frames
    // coalesced behind it carried. Done on a COPY, before the hand-off, so a
    // failed send leaves both gLink's snapshot and its accumulator untouched.
    TelemetryPayload snap = gLink.latest();
    gLink.mergeCoalesced(snap);

    hostproto::Telemetry t;
    toHostTelemetry(snap, t);
    const bool sent = gHost.sendTelemetry(t); // a drop is counted in BridgeStatus::hostTxDropped

    // ONLY ON A CONFIRMED SEND. A full host TX ring, a disconnected Jetson or an
    // app that has stalled all end here, and clearing the accumulator on the
    // attempt discarded the event along with the frame that failed to carry it —
    // after which the next frame reported a quiet window over an interval that
    // contained an impact. Held instead until something actually leaves.
    if (sent) gLink.clearCoalesced();

    // Only retire the one-shot once it has actually gone out.  Clearing it on
    // the attempt would answer a full TX ring with silence, leaving the host
    // waiting for a reply that was never transmitted.
    if (oneShot && sent) { gForwardOnce = false; gOncePendingTx = false; }
}

/** @brief Keeps the master stream alive; logs the link's up/down edges upward. */
static void serviceMasterLink(uint32_t nowMs)
{
    const bool up = gMasterSeen && (masterAgeMs(nowMs) < MASTER_STALE_MS);

    if (up != gMasterUp) {
        gMasterUp = up;
        bridgeLog(up ? hostproto::LOG_INFO : hostproto::LOG_WARN,
                  up ? "master link up" : "master link lost");
    }

    // While the master is quiet, re-request the stream once per second: the MKR
    // may have reset and forgotten it was streaming.  Bounded retry with a
    // defined period — never a spin.
    if (!up && (nowMs - gLastRetryMs) >= MASTER_RETRY_MS) {
        gLastRetryMs = nowMs;
        (void)gLink.startStream(); // best-effort; this periodic path is itself the retry
        return;                    // a start-stream already proves we are alive
    }

    // Heartbeat while the link IS up, so the master can tell "bridge attached
    // and quiet" from "bridge unplugged".  Without it the master has no signal
    // at all during healthy streaming and cannot report a disconnect.
    static uint32_t lastBeatMs = 0;
    if (up && (nowMs - lastBeatMs) >= MASTER_HEARTBEAT_MS) {
        lastBeatMs = nowMs;
        (void)gLink.ping();   // best-effort; the MKR replies PONG
    }
}

// ─── Arduino entry points ─────────────────────────────────────────────────────

void setup()
{
    // MUST come first.  On a C3 the ESP-IDF console is routed to the same USB
    // Serial/JTAG peripheral this bridge uses for binary frames, so any
    // ESP_LOGx from an IDF component injects ASCII mid-frame.  CRC catches it
    // and the decoder resyncs, but the frame is lost — and a chatty component
    // would cost telemetry continuously.  Silencing the log at runtime covers
    // the IDF libraries that ship precompiled at INFO level, which the IDE's
    // "Core Debug Level" setting cannot reach.
    //
    // Bootloader and ROM output still precedes this (it runs before any user
    // code); the host decoder discards it as pre-SOF garbage.
    esp_log_level_set("*", ESP_LOG_NONE);

    // First power-on leaves RTC_NOINIT memory undefined; seed it before use.
    if (gBootMagic != BOOT_MAGIC) {
        gBootMagic = BOOT_MAGIC;
        gBootCount = 0;
    }
    ++gBootCount;

    gHost.begin();                 // native USB CDC -> Jetson
    gLink.begin(115200, COMMLINK_RX_PIN, COMMLINK_TX_PIN); // UART1 -> MKR Zero

    // Ask the master to stream straight away so telemetry is already flowing by
    // the time the Jetson attaches.  A failure here is not fatal: the stale-link
    // path in serviceMasterLink() retries every second.
    (void)gLink.startStream();

    const uint32_t now = millis();
    gLastStatusMs = now;
    gLastRetryMs  = now;
}

void loop()
{
    const uint32_t now = millis();

    // 1) Jetson commands.  Must run every pass: it is also what keeps the
    //    host-liveness timer fed.
    (void)gHost.poll(now);
    if (gHost.connectSeen()) {
        gDecimCount = 0;
        greetHost(now);
    }

    // 2) A one-shot request is forwarded down to the MKR; the answer is relayed
    //    when it lands, below.
    if (gHost.onceRequested()) {
        // Arm the relay first, then try to hand the request down.  A busy master
        // TX only defers the hand-off (expireOneShot() retries it within the
        // window); it must not discard the request, because the reply the host
        // is waiting for would then never be asked for at all.
        gForwardOnce   = true;
        gForwardOnceMs = now;
        gOncePendingTx = !gLink.requestOnce();
    }

    // 2b) CAN mode request, relayed straight down.
    //
    // Deliberately NOT latched-and-retried the way a one-shot is. A one-shot
    // has an answer the host is blocked on, so losing it strands the host; a
    // mode request has no reply, and a stale one delivered seconds later would
    // switch the bus mode at a moment the host has long since moved past. If
    // the MKR's TX is busy, say so and let the host ask again.
    if (const uint8_t m = gHost.takeCanModeRequest()) {
        if (!gLink.setCanMode(m)) {
            bridgeLog(hostproto::LOG_WARN, "CMD_SET_CAN_MODE dropped: master TX busy");
        }
    }

    // 2b-ii) IMU mode request, relayed on the same terms as the CAN mode: not
    //        latched, not retried.
    //
    // The reasoning is stronger here. Applying one costs the master a ~700 ms
    // blind window, so a request re-sent because the first was dropped would
    // blind the sensor twice for a change the host asked for once — and a stale
    // one delivered seconds later would switch the MEASUREMENT under a host that
    // has moved on, which is worse than not switching at all.
    if (const uint8_t m = gHost.takeImuModeRequest()) {
        if (!gLink.setImuMode(m)) {
            bridgeLog(hostproto::LOG_WARN, "CMD_SET_IMU_MODE dropped: master TX busy");
        }
    }

    // 2c) CAN filter set, relayed on the same terms as the mode above: not
    //     latched, not retried. There is no reply to strand a host on, and a
    //     stale filter change applied seconds later would narrow a capture the
    //     host has long since moved past.
    {
        uint16_t ids[hostproto::CAN_FILTER_SLOTS];
        uint8_t  count = 0;
        if (gHost.takeCanFilterRequest(ids, count)) {
            if (!gLink.setCanFilter(ids, count)) {
                bridgeLog(hostproto::LOG_WARN, "CMD_SET_CAN_FILTER dropped: master TX busy");
            }
        }
    }

    expireOneShot(now);

    // 3) Master telemetry.  Always polled, host attached or not, so the UART
    //    ring never overflows and the staleness view stays honest.
    if (gLink.poll()) {
        gMasterSeen   = true;
        gLastMasterMs = now;
        forwardTelemetry();
    }

    // 4) Health heartbeat + on-demand status.
    if (gHost.statusRequested() || (now - gLastStatusMs) >= STATUS_INTERVAL_MS) {
        gLastStatusMs = now;
        if (gHost.isConnected()) {
            hostproto::BridgeStatus s;
            buildStatus(now, s);
            (void)gHost.sendStatus(s);
        }
    }

    // 5) Master link supervision.
    serviceMasterLink(now);

    // Yield.  Every step above is non-blocking, so without this loop() spins as
    // fast as the core allows — on the C3's single RISC-V core that starves the
    // idle task and trips the task watchdog, and it burns power for nothing.
    // delay() calls vTaskDelay, which yields and feeds the watchdog.  1 ms is
    // invisible against a 100 ms telemetry period.
    delay(1);
}
