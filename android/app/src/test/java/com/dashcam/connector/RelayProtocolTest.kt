package com.dashcam.connector

import org.junit.Assert.*
import org.junit.Test

class RelayProtocolTest {
    @Test fun requestIdSurvivesASnapshotInTheSamePollingInterval() {
        val inbox = RelayInbox { 42L }
        inbox.offer("""{"type":"connecting","requestId":"abc123"}""")
        inbox.offer("""{"schema":1,"type":"snapshot","seq":7}""")
        assertEquals("abc123", inbox.takeRequestId())
        val snap = inbox.takeSnapshot()
        assertNotNull(snap)
        assertEquals(7, snap!!.data.getInt("seq"))
        assertEquals(42L, snap.time)
        assertNull(inbox.takeRequestId())
        assertNull(inbox.takeSnapshot())
        assertFalse(inbox.invalid)
    }

    @Test fun onlyTheLatestSnapshotIsKept() {
        val inbox = RelayInbox { 0L }
        inbox.offer("""{"type":"snapshot","seq":1}""")
        inbox.offer("""{"type":"snapshot","seq":2}""")
        assertEquals(2, inbox.takeSnapshot()!!.data.getInt("seq"))
    }

    @Test fun nonJsonIsFlagged() {
        val inbox = RelayInbox { 0L }
        inbox.offer("not json")
        assertTrue(inbox.invalid)
        assertNull(inbox.takeSnapshot())
    }

    @Test fun capacityIsRetryableAndDistinctFromAccess() {
        assertEquals(RelayClose.FULL, relayClose(1013))
        for (code in listOf(401, 403, 1008)) assertEquals(RelayClose.REJECTED, relayClose(code))
        for (code in listOf(0, 1000, 1001, 1006, 1011)) assertEquals(RelayClose.OTHER, relayClose(code))
    }
}
