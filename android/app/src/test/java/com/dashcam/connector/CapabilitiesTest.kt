package com.dashcam.connector

import org.json.JSONObject
import org.junit.Assert.assertEquals
import org.junit.Test

// Review finding (2026-11-26): HD must be offered only when the Jetson's identity says video_hd is true.
class CapabilitiesTest {
    private fun caps(json: String) = Capabilities.parse(JSONObject(json))

    @Test fun hdOnlyOnAnExplicitTrue() {
        // The 4f94de4 agent: no video_hd at all (it ignored quality=hd and sent 640x360).
        assertEquals(Capabilities(videoHd = false, recordings = false),
            caps("""{"capabilities": {"vehicle": true, "system": true, "video": true}}"""))
        assertEquals(false, caps("""{"capabilities": {"video_hd": false}}""").videoHd)
        assertEquals(true, caps("""{"capabilities": {"video_hd": true}}""").videoHd)
        // Anything but a JSON boolean true is no.
        for (odd in listOf("\"true\"", "1", "null", "{}")) assertEquals(odd, false, caps("""{"capabilities": {"video_hd": $odd}}""").videoHd)
        assertEquals(Capabilities(false, false), caps("{}"))
        assertEquals(Capabilities(false, false), Capabilities.parse(null))
    }

    @Test fun recordingsCapability() {
        assertEquals(Capabilities(videoHd = true, recordings = true),
            caps("""{"capabilities": {"video_hd": true, "recordings": true}}"""))
    }
}
