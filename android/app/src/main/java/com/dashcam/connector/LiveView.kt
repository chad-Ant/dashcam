package com.dashcam.connector

/** Live view turns itself off this long after the user taps Start. */
const val LIVE_VIEW_MAX_MS = 10 * 60 * 1000L

// Camera video streams only on explicit demand, within one window per tap.
// Times are elapsed-realtime milliseconds: expiry is decided by comparing the
// clock, so a late or throttled tick can never stretch the window.
class LiveViewSession {
    private var startedAtMs = 0L
    private var on = false
    var stoppedByTimer = false
        private set
    fun start(nowMs: Long) { startedAtMs = nowMs; on = true; stoppedByTimer = false }
    fun stop() { on = false; stoppedByTimer = false }
    fun isOn(nowMs: Long) = remainingMs(nowMs) > 0
    fun remainingMs(nowMs: Long): Long {
        if (!on) return 0
        val remaining = LIVE_VIEW_MAX_MS - (nowMs - startedAtMs)
        if (remaining <= 0) { on = false; stoppedByTimer = true; return 0 }
        return remaining
    }
}

// available: a Jetson is selected and has not reported no video or rejected the credential/certificate.
data class LiveViewState(val on: Boolean = false, val stoppedByTimer: Boolean = false,
                         val remainingMs: Long = 0, val available: Boolean = false)

/** Remaining time for the camera caption: whole minutes rounded up, "< 1 min" in the last one. */
fun liveViewRemaining(remainingMs: Long) =
    if (remainingMs < 60_000) "stops in < 1 min" else "stops in ${(remainingMs + 59_999) / 60_000} min"
