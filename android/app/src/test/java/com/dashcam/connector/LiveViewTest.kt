package com.dashcam.connector

import org.junit.Assert.*
import org.junit.Test

class LiveViewTest {
    @Test fun offByDefault() {
        val session = LiveViewSession()
        assertFalse(session.isOn(0))
        assertFalse(session.stoppedByTimer)
        assertEquals(0L, session.remainingMs(123_456))
        assertEquals(LiveViewState(), LiveViewState(on = false, stoppedByTimer = false, remainingMs = 0, available = false))
    }
    @Test fun startTurnsOnWithAFullWindow() {
        val session = LiveViewSession()
        session.start(5_000)
        assertTrue(session.isOn(5_000))
        assertEquals(LIVE_VIEW_MAX_MS, session.remainingMs(5_000))
        assertEquals(LIVE_VIEW_MAX_MS - 61_000, session.remainingMs(66_000))
    }
    @Test fun expiresAtExactlyTenMinutes() {
        val session = LiveViewSession()
        session.start(5_000)
        assertTrue(session.isOn(5_000 + LIVE_VIEW_MAX_MS - 1))
        assertFalse(session.stoppedByTimer)
        assertFalse(session.isOn(5_000 + LIVE_VIEW_MAX_MS))
        assertTrue(session.stoppedByTimer)
        assertEquals(0L, session.remainingMs(5_000 + LIVE_VIEW_MAX_MS))
    }
    @Test fun aLateCheckStillExpires() {
        // A throttled tick that first runs long after the window still ends it.
        val session = LiveViewSession()
        session.start(0)
        assertFalse(session.isOn(3 * LIVE_VIEW_MAX_MS))
        assertTrue(session.stoppedByTimer)
    }
    @Test fun restartAfterExpiryGivesAFreshWindow() {
        val session = LiveViewSession()
        session.start(0)
        assertFalse(session.isOn(LIVE_VIEW_MAX_MS))
        session.start(LIVE_VIEW_MAX_MS + 30_000)
        assertFalse(session.stoppedByTimer)
        assertTrue(session.isOn(2 * LIVE_VIEW_MAX_MS))
        assertEquals(LIVE_VIEW_MAX_MS, session.remainingMs(LIVE_VIEW_MAX_MS + 30_000))
        assertFalse(session.isOn(2 * LIVE_VIEW_MAX_MS + 30_000))
    }
    @Test fun stopClearsTheSessionAndTheTimerFlag() {
        val session = LiveViewSession()
        session.start(0)
        session.stop()
        assertFalse(session.isOn(1))
        assertFalse(session.stoppedByTimer)
        session.start(0)
        assertFalse(session.isOn(LIVE_VIEW_MAX_MS))
        session.stop()
        assertFalse(session.stoppedByTimer)
    }
    @Test fun remainingMinutesRoundUp() {
        assertEquals("stops in 10 min", liveViewRemaining(LIVE_VIEW_MAX_MS))
        assertEquals("stops in 10 min", liveViewRemaining(9 * 60_000L + 1))
        assertEquals("stops in 9 min", liveViewRemaining(9 * 60_000L))
        assertEquals("stops in 2 min", liveViewRemaining(60_001))
        assertEquals("stops in 1 min", liveViewRemaining(60_000))
        assertEquals("stops in < 1 min", liveViewRemaining(59_999))
        assertEquals("stops in < 1 min", liveViewRemaining(1))
    }
}
