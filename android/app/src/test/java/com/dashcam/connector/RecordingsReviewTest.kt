package com.dashcam.connector

import org.json.JSONArray
import org.junit.Assert.*
import org.junit.Rule
import org.junit.Test
import org.junit.rules.TemporaryFolder
import java.io.File
import java.io.IOException
import java.net.SocketException
import java.security.MessageDigest

/** Review 2026-11-27: #1 (a failed download must not erase the saved copy), #6 (playable), #8 (Play after a drop). */
class RecordingsReviewTest {
    @get:Rule val temp = TemporaryFolder()

    private fun sha(f: File) = MessageDigest.getInstance("SHA-256").digest(f.readBytes()).joinToString("") { "%02x".format(it) }
    private val video = ByteArray(300_000) { (it * 7).toByte() }
    private val captions = "[Script Info]\nPlayResX: 384\n".toByteArray()

    private fun saveOnce(dir: File, failOn: String? = null, afterBytes: Int = 0) =
        saveStaged(DirectoryTarget(dir), listOf("seg.mkv", "seg.ass")) { name, out ->
            val data = if (name.endsWith(".mkv")) video else captions
            if (name == failOn) { out.write(data, 0, afterBytes); throw SocketException("Wi-Fi dropped") }
            out.write(data)
        }

    @Test fun interruptedVideoKeepsBothEarlierFiles() {
        val dir = temp.newFolder("Dashcam")
        saveOnce(dir)
        val before = sha(File(dir, "seg.mkv")) to sha(File(dir, "seg.ass"))
        assertThrows(SocketException::class.java) { saveOnce(dir, failOn = "seg.mkv", afterBytes = 1000) }
        assertEquals(before, sha(File(dir, "seg.mkv")) to sha(File(dir, "seg.ass")))
        assertEquals(setOf("seg.mkv", "seg.ass"), dir.list()!!.toSet())   // no partial file left behind
    }

    @Test fun failureDuringCaptionsKeepsTheEarlierVideoToo() {
        val dir = temp.newFolder("Dashcam")
        saveOnce(dir)
        val before = sha(File(dir, "seg.mkv")) to sha(File(dir, "seg.ass"))
        assertThrows(SocketException::class.java) { saveOnce(dir, failOn = "seg.ass", afterBytes = 5) }
        assertEquals(before, sha(File(dir, "seg.mkv")) to sha(File(dir, "seg.ass")))
        assertEquals(setOf("seg.mkv", "seg.ass"), dir.list()!!.toSet())
    }

    @Test fun cancellationDiscardsAndARetryLeavesOneNamedPair() {
        val dir = temp.newFolder("Dashcam")
        assertThrows(kotlinx.coroutines.CancellationException::class.java) {
            saveStaged(DirectoryTarget(dir), listOf("seg.mkv", "seg.ass")) { _, out ->
                out.write(video, 0, 10); throw kotlinx.coroutines.CancellationException("Download cancelled")
            }
        }
        assertEquals(0, dir.list()!!.size)
        assertEquals(dir.absolutePath, saveOnce(dir))
        assertEquals(setOf("seg.mkv", "seg.ass"), dir.list()!!.toSet())
        assertArrayEquals(video, File(dir, "seg.mkv").readBytes())
        assertArrayEquals(captions, File(dir, "seg.ass").readBytes())
    }

    @Test fun stagingNamesKeepTheExtension() {
        val name = stagingName("dashcam_000905_20261126_154012.ass")
        assertTrue(name, Regex("""dashcam_000905_20261126_154012\.partial-[0-9a-f]{6}\.ass""").matches(name))
        assertNotEquals(name, stagingName("dashcam_000905_20261126_154012.ass"))
    }

    @Test fun shortOrMismatchedTransfersAreErrors() {
        checkComplete(10, 10, 10)
        checkComplete(10, -1, null)   // no Content-Length and no listed size: nothing to compare
        assertThrows(IOException::class.java) { checkComplete(9, 10, 10) }
        assertThrows(IOException::class.java) { checkComplete(9, -1, 10) }
    }

    @Test fun playableComesFromTheListAndDefaultsToTrue() {
        val list = JSONArray("""[{"id":"a","seq":1,"start":"2026-11-26 13:41:21","duration_s":180.0,"bytes":5,"caption_bytes":null,"playable":false},
            {"id":"b","seq":2,"start":"2026-11-26 13:44:21","duration_s":null,"bytes":5,"caption_bytes":3}]""")
        assertEquals(listOf(false, true), parseSegments(list).map { it.playable })
    }

    @Test fun aLostConnectionKeepsPlayEnabled() {
        val drop = playbackFailure(SocketException("Software caused connection abort"), started = true)
        assertFalse(drop.terminal)
        assertEquals("Connection lost · tap Play to resume", drop.message)
        assertFalse(playbackFailure(java.net.SocketTimeoutException(), started = false).terminal)
        assertTrue(playbackFailure(MkvFormatException("No Cues at the indexed position"), started = true).terminal)
        assertTrue(playbackFailure(RecordingsException("This segment is still recording"), started = true).terminal)
    }
}
