package com.dashcam.connector

import org.junit.Assert.*
import org.junit.Test

class ProtocolTest {
    private val id = "12345678-1234-1234-1234-123456789abc"
    private fun pair(extra: String = "") = "dashcam://pair?host=192.168.0.9&port=8443&pin=${"a".repeat(64)}&token=${"b".repeat(43)}&id=$id&name=My+Jetson$extra"
    @Test fun pairingValidatesAndDecodes() {
        val profile = DeviceProfile.parse(pair())
        assertEquals("My Jetson", profile.name)
        assertEquals(8443, profile.port)
        assertEquals(profile, DeviceProfile.fromJson(profile.toJson()))
    }
    @Test fun malformedCredentialsAreRejected() {
        for (link in listOf(pair().replace("8443", "0"), pair().replace("dashcam://", "http://"),
                            pair().replace("a".repeat(64), "abc"), pair().replace("192.168.0.9", "foo/path"))) {
            assertThrows(IllegalArgumentException::class.java) { DeviceProfile.parse(link) }
        }
    }
    private fun snapshot(fresh: Boolean, bridge: Boolean) = """{"schema":1,"type":"snapshot","device_id":"$id","seq":1,"session":"test","runtime_fresh":$fresh,"runtime":{"bridge_fresh":$bridge,"vehicle":{"speed_kmh":42}},"system":{}}"""
    @Test fun staleRuntimeAndBridgeHideVehicleReadings() {
        assertNull(Snapshot.parse(snapshot(false, true), id).vehicle)
        assertNull(Snapshot.parse(snapshot(true, false), id).vehicle)
        assertEquals(42.0, Snapshot.parse(snapshot(true, true), id).vehicle.finite("speed_kmh")!!, 0.0)
    }
    @Test fun identityAndSchemaAreChecked() {
        assertThrows(IllegalArgumentException::class.java) { Snapshot.parse(snapshot(true, true), "other") }
        assertThrows(IllegalArgumentException::class.java) { Snapshot.parse(snapshot(true, true).replace("\"schema\":1", "\"schema\":2"), id) }
    }
    @Test fun receiptFreshnessDoesNotDependOnWallClock() {
        val state = MonitorState(active = true, lastReceiveMs = 1000, nowMs = 2999,
            snapshot = Snapshot.parse(snapshot(true, true), id))
        assertTrue(state.runtimeFresh)
        assertFalse(state.copy(nowMs = 3001).runtimeFresh)
        assertFalse(state.copy(active = false).transportFresh)
    }
}
