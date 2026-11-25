package com.dashcam.connector

import org.junit.Assert.*
import org.junit.Test

class ScanResultsTest {
    private val device = DiscoveredDevice("jetson-1", "192.168.1.78", 8443)

    @Test fun oldScanCallbacksDoNotTouchTheNewScan() {
        val results = ScanResults()
        val first = results.begin()
        val second = results.begin()
        assertTrue(results.found(second, device))
        // The first scan's failure, loss and status callbacks arrive after the second started.
        assertFalse(results.isCurrent(first))
        assertFalse(results.lost(first, device.id))
        assertFalse(results.say(first, "Discovery unavailable. Use a pairing link."))
        assertFalse(results.found(first, device.copy(host = "10.0.0.9")))
        assertEquals(listOf(device), results.devices.value)
        assertEquals("1 device(s) found. Pairing verifies device identity.", results.status.value)
        assertTrue(results.isCurrent(second))
    }

    @Test fun currentScanUpdatesAndLoses() {
        val results = ScanResults()
        val scan = results.begin()
        results.found(scan, device)
        results.found(scan, device.copy(host = "192.168.1.79"))
        assertEquals(listOf(device.copy(host = "192.168.1.79")), results.devices.value)
        assertTrue(results.lost(scan, device.id))
        assertTrue(results.devices.value.isEmpty())
    }

    @Test fun endedScanIgnoresLateCallbacks() {
        val results = ScanResults()
        val scan = results.begin()
        results.found(scan, device)
        results.end("Discovery stopped")
        assertFalse(results.lost(scan, device.id))
        assertFalse(results.say(scan, "A device could not be resolved. Manual pairing is available."))
        assertEquals(listOf(device), results.devices.value)
        assertEquals("Discovery stopped", results.status.value)
    }
}
