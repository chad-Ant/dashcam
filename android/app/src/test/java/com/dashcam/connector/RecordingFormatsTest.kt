package com.dashcam.connector

import org.junit.Assert.*
import org.junit.Test

class RecordingFormatsTest {
    // 30 frames (1 s at 30 fps, 96x54 JPEG) written by GStreamer's matroskamux through splitmuxsink with
    // offset-to-zero, as v0.4 records: SeekHead, Info, Tracks (V_MS/VFW/FOURCC MJPG), two clusters, Cues.
    private val fixture = javaClass.getResourceAsStream("/recorder_fixture.mkv")!!.readBytes()

    @Test fun headerOfARecorderSegment() {
        val info = Mkv.parseHeader(fixture.copyOf(minOf(fixture.size, Mkv.HEADER_BYTES)))
        assertEquals(1, info.track)
        assertEquals(96, info.width)
        assertEquals(54, info.height)
        assertEquals(1000L, info.durationMs)
        assertEquals(1_000_000L, info.timecodeScaleNs)
        assertEquals(33.333, info.frameDurationMs, 0.001)
        assertEquals(44L, info.segmentDataStart)
        assertEquals(44L + 31398, info.cuesPosition)
        // Only the first bytes are needed: the Segment and its first Cluster run past them.
        assertEquals(info, Mkv.parseHeader(fixture.copyOf(600)))
    }

    @Test fun cuesGiveOneEntryPerCluster() {
        val info = Mkv.parseHeader(fixture)
        val clusters = Mkv.parseCues(fixture.copyOfRange(info.cuesPosition.toInt(), fixture.size), info)
        assertEquals(listOf(MkvCluster(484, 0), MkvCluster(15904, 500)), clusters)
        assertEquals(0, Mkv.clusterAt(clusters, 0))
        assertEquals(0, Mkv.clusterAt(clusters, 499))
        assertEquals(1, Mkv.clusterAt(clusters, 500))
        assertEquals(1, Mkv.clusterAt(clusters, 99_999))
    }

    @Test fun clustersHoldCompleteJpegsInOrder() {
        val info = Mkv.parseHeader(fixture)
        val clusters = Mkv.parseCues(fixture.copyOfRange(info.cuesPosition.toInt(), fixture.size), info)
        val ends = clusters.drop(1).map { it.offset } + info.cuesPosition
        val frames = clusters.zip(ends).flatMap { (c, end) -> Mkv.parseCluster(fixture.copyOfRange(c.offset.toInt(), end.toInt()), info) }
        assertEquals(30, frames.size)
        assertEquals(frames.map { it.ptsMs }.sorted(), frames.map { it.ptsMs })
        assertEquals(0L, frames.first().ptsMs)
        assertEquals(967L, frames.last().ptsMs)
        for (f in frames) {
            assertEquals(0xFF.toByte(), f.bytes[f.offset]); assertEquals(0xD8.toByte(), f.bytes[f.offset + 1])
            assertEquals(0xFF.toByte(), f.bytes[f.offset + f.length - 2]); assertEquals(0xD9.toByte(), f.bytes[f.offset + f.length - 1])
        }
    }

    @Test fun rejectsWhatIsNotARecorderSegment() {
        assertThrows(MkvFormatException::class.java) { Mkv.parseHeader(ByteArray(64)) }
        assertThrows(MkvFormatException::class.java) { Mkv.parseHeader(fixture.copyOf(5)) }
        // Without the SeekHead's Cues entry (a segment still recording) there is nothing to seek with.
        val noCues = fixture.copyOf(600)
        val cuesId = byteArrayOf(0x1C, 0x53, 0xBB.toByte(), 0x6B)
        val at = (0 until noCues.size - 3).first { i -> (0..3).all { noCues[i + it] == cuesId[it] } }
        noCues[at] = 0x1D   // another ID
        assertThrows(MkvFormatException::class.java) { Mkv.parseHeader(noCues) }
        val info = Mkv.parseHeader(fixture)
        assertThrows(MkvFormatException::class.java) { Mkv.parseCues(fixture.copyOfRange(484, 2000), info) }
        assertThrows(MkvFormatException::class.java) { Mkv.parseCluster(fixture.copyOfRange(info.cuesPosition.toInt(), fixture.size), info) }
        assertThrows(MkvFormatException::class.java) { Mkv.parseCluster(fixture.copyOfRange(484, 1000), info) }   // cut short
    }

    private val ass = """
        [Script Info]
        Title: dashcam telemetry
        ScriptType: v4.00+
        PlayResX: 1920
        PlayResY: 1080

        [V4+ Styles]
        Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding
        Style: TL,Monospace,21,&H00FFFFFF,&H00FFFFFF,&H26000000,&H26000000,-1,0,0,0,100,100,0,0,3,6,0,7,18,18,12,1
        Style: BR,Monospace,21,&H00FFFFFF,&H00FFFFFF,&H26000000,&H26000000,-1,0,0,0,100,100,0,0,3,6,0,3,18,18,12,1
        Style: DET,Monospace,13,&H00FFFFFF,&H00FFFFFF,&H26000000,&H26000000,-1,0,0,0,100,100,0,0,3,3,0,4,18,18,12,1

        [Events]
        Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text
        Dialogue: 0,0:00:00.19,0:00:00.39,TL,,0,0,0,,SPD 52.1 km/h\NACC 0.31 m/s2
        Dialogue: 0,0:00:00.19,0:00:00.39,BR,,0,0,0,,2026-11-26\N13:41:21 +07
        Dialogue: 0,0:00:00.39,0:00:00.59,TL,,0,0,0,,{\b1}SPD 53.0 km/h, rising\NACC 0.30 m/s2
        Dialogue: 0,0:00:00.39,0:00:00.59,DET,,0,0,0,,TLM ok
        Dialogue: 0,0:59:59.99,1:00:00.19,BR,,0,0,0,,late
        Dialogue: 0,garbage,0:00:01.00,TL,,0,0,0,,dropped
    """.trimIndent()

    @Test fun captionsOfTheRecorderFormat() {
        val script = AssScript.parse(ass)
        assertEquals(1920, script.playResX)
        assertEquals(1080, script.playResY)
        assertEquals(AssStyle(7, 21f, 18, 18, 12), script.styles["TL"])
        assertEquals(AssStyle(4, 13f, 18, 18, 12), script.styles["DET"])
        assertEquals(5, script.eventCount)   // the malformed line is skipped
        assertTrue(script.at(100).isEmpty())
        val first = script.at(200)
        assertEquals(setOf("SPD 52.1 km/h\nACC 0.31 m/s2", "2026-11-26\n13:41:21 +07"), first.map { it.text }.toSet())
        // The next line of a style replaces it; override tags are dropped; commas in the text survive.
        val second = script.at(390).associate { it.style.alignment to it.text }
        assertEquals("SPD 53.0 km/h, rising\nACC 0.30 m/s2", second[7])
        assertEquals("TLM ok", second[4])
        assertNull(second[3])   // the BR line ended at 0.39
        assertTrue(script.at(590).isEmpty())
        assertEquals(listOf("late"), script.at(3_600_000).map { it.text })
    }

    @Test fun assTimes() {
        assertEquals(190L, AssScript.time("0:00:00.19"))
        assertEquals(3_723_450L, AssScript.time("1:02:03.45"))
        assertEquals(-1L, AssScript.time("0:61:00.00"))
        assertEquals(-1L, AssScript.time("abc"))
    }
}
