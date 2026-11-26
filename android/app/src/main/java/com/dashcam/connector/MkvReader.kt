package com.dashcam.connector

// Plain Kotlin, no Android types: unit-tested on the JVM against a segment written by the recorder's muxer.
//
// Just enough Matroska for v0.4's segments (GStreamer matroskamux, connector/README.md "Recordings"):
// one video track of MJPEG (V_MS/VFW/FOURCC with an MJPG BITMAPINFOHEADER, or V_MJPEG), known-size
// clusters of unlaced SimpleBlocks (each a complete JPEG), and a Cues index written when the segment
// was finished. The player reads the header, then the Cues, then whole clusters by byte range.

class MkvFormatException(message: String) : Exception(message)

/** What the header (EBML, SeekHead, Info, Tracks) says. Positions are absolute file offsets. */
data class MkvInfo(
    val segmentDataStart: Long, val cuesPosition: Long, val timecodeScaleNs: Long, val durationMs: Long,
    val track: Int, val width: Int, val height: Int, val frameDurationMs: Double,
)

/** One cluster: its absolute offset and its start time. */
data class MkvCluster(val offset: Long, val timeMs: Long)

/** One frame inside a cluster's bytes: a complete JPEG at [offset], [length] bytes long. */
class MkvFrame(val ptsMs: Long, val bytes: ByteArray, val offset: Int, val length: Int)

object Mkv {
    private const val EBML = 0x1A45DFA3L
    private const val SEGMENT = 0x18538067L
    private const val SEEK_HEAD = 0x114D9B74L
    private const val SEEK = 0x4DBBL
    private const val SEEK_ID = 0x53ABL
    private const val SEEK_POSITION = 0x53ACL
    private const val INFO = 0x1549A966L
    private const val TIMECODE_SCALE = 0x2AD7B1L
    private const val DURATION = 0x4489L
    private const val TRACKS = 0x1654AE6BL
    private const val TRACK_ENTRY = 0xAEL
    private const val TRACK_NUMBER = 0xD7L
    private const val TRACK_TYPE = 0x83L
    private const val CODEC_ID = 0x86L
    private const val CODEC_PRIVATE = 0x63A2L
    private const val DEFAULT_DURATION = 0x23E383L
    private const val VIDEO = 0xE0L
    private const val PIXEL_WIDTH = 0xB0L
    private const val PIXEL_HEIGHT = 0xBAL
    private const val CLUSTER = 0x1F43B675L
    private const val TIMECODE = 0xE7L
    private const val SIMPLE_BLOCK = 0xA3L
    private const val BLOCK_GROUP = 0xA0L
    private const val BLOCK = 0xA1L
    private const val CUES = 0x1C53BB6BL
    private const val CUE_POINT = 0xBBL
    private const val CUE_TIME = 0xB3L
    private const val CUE_TRACK_POSITIONS = 0xB7L
    private const val CUE_CLUSTER_POSITION = 0xF1L

    /** Bytes the header needs: the recorder's ends well before 4 KiB. */
    const val HEADER_BYTES = 65536

    private class Element(val id: Long, val start: Int, val body: Int, val end: Int)

    private fun vint(b: ByteArray, at: Int, keepMarker: Boolean): Pair<Long, Int> {
        if (at >= b.size) throw MkvFormatException("Truncated element")
        val first = b[at].toInt() and 0xFF
        var length = 1
        while (length <= 8 && first and (0x80 ushr (length - 1)) == 0) length++
        if (length > 8 || at + length > b.size) throw MkvFormatException("Bad variable-length integer")
        var value = (if (keepMarker) first else first and (0xFF ushr length)).toLong()
        for (k in 1 until length) value = (value shl 8) or (b[at + k].toLong() and 0xFF)
        if (!keepMarker && value == (1L shl (7 * length)) - 1) value = -1   // unknown size
        return value to length
    }

    /** The element at [at]; one of unknown size runs to [limit]. [whole]: it must end within [limit]. */
    private fun element(b: ByteArray, at: Int, limit: Int, whole: Boolean = true): Element {
        val (id, idLength) = vint(b, at, true)
        val (size, sizeLength) = vint(b, at + idLength, false)
        val body = at + idLength + sizeLength
        val end = if (size < 0) limit.toLong() else body.toLong() + size
        if (whole && end > limit) throw MkvFormatException("Element past the end of the data")
        return Element(id, at, body, end.coerceAtMost(Int.MAX_VALUE.toLong()).toInt())
    }

    private fun children(b: ByteArray, from: Int, to: Int) = sequence {
        var at = from
        while (at < to) {
            val e = element(b, at, to)
            yield(e)
            at = e.end
        }
    }

    private fun uint(b: ByteArray, e: Element): Long {
        if (e.end - e.body !in 1..8) throw MkvFormatException("Bad unsigned integer")
        var v = 0L
        for (k in e.body until e.end) v = (v shl 8) or (b[k].toLong() and 0xFF)
        return v
    }

    private fun float(b: ByteArray, e: Element): Double {
        var bits = 0L
        for (k in e.body until e.end) bits = (bits shl 8) or (b[k].toLong() and 0xFF)
        return when (e.end - e.body) {
            4 -> java.lang.Float.intBitsToFloat(bits.toInt()).toDouble()
            8 -> java.lang.Double.longBitsToDouble(bits)
            else -> throw MkvFormatException("Bad float")
        }
    }

    /** Parses the file's first bytes ([HEADER_BYTES] or the whole file if shorter). */
    fun parseHeader(b: ByteArray): MkvInfo {
        val ebml = element(b, 0, b.size)
        if (ebml.id != EBML) throw MkvFormatException("Not a Matroska file")
        // Only the start of the Segment is here: it and its first Cluster run past these bytes.
        val segment = element(b, ebml.end, b.size, whole = false)
        if (segment.id != SEGMENT) throw MkvFormatException("No Segment")
        val dataStart = segment.body
        var cues = -1L
        var scale = 1_000_000L
        var duration = -1.0
        var track = -1; var width = 0; var height = 0; var frameNs = 0L
        var codec = ""; var fourcc = ""
        var at = segment.body
        while (at < b.size) {
            val e = element(b, at, b.size, whole = false)
            if (e.id == CLUSTER) break
            if (e.end > b.size) throw MkvFormatException("Header longer than $HEADER_BYTES bytes")
            at = e.end
            when (e.id) {
                SEEK_HEAD -> for (seek in children(b, e.body, e.end)) if (seek.id == SEEK) {
                    var target = 0L; var position = -1L
                    for (f in children(b, seek.body, seek.end)) when (f.id) {
                        SEEK_ID -> target = uint(b, f)
                        SEEK_POSITION -> position = uint(b, f)
                    }
                    if (target == CUES && position >= 0) cues = dataStart + position
                }
                INFO -> for (f in children(b, e.body, e.end)) when (f.id) {
                    TIMECODE_SCALE -> scale = uint(b, f)
                    DURATION -> duration = float(b, f)
                }
                TRACKS -> for (entry in children(b, e.body, e.end)) if (entry.id == TRACK_ENTRY && track < 0) {
                    var number = -1; var type = 0L; var id = ""; var codecPrivate: Element? = null
                    var w = 0; var h = 0; var fd = 0L
                    for (f in children(b, entry.body, entry.end)) when (f.id) {
                        TRACK_NUMBER -> number = uint(b, f).toInt()
                        TRACK_TYPE -> type = uint(b, f)
                        CODEC_ID -> id = String(b, f.body, f.end - f.body, Charsets.US_ASCII).trimEnd('\u0000', ' ')
                        CODEC_PRIVATE -> codecPrivate = f
                        DEFAULT_DURATION -> fd = uint(b, f)
                        VIDEO -> for (v in children(b, f.body, f.end)) when (v.id) {
                            PIXEL_WIDTH -> w = uint(b, v).toInt()
                            PIXEL_HEIGHT -> h = uint(b, v).toInt()
                        }
                    }
                    if (type != 1L) continue
                    track = number; width = w; height = h; frameNs = fd; codec = id
                    // BITMAPINFOHEADER: biCompression (the FourCC) at bytes 16..19.
                    codecPrivate?.let { p -> if (p.end - p.body >= 20) fourcc = String(b, p.body + 16, 4, Charsets.US_ASCII) }
                }
            }
        }
        if (track < 0) throw MkvFormatException("No video track")
        if (!(codec == "V_MJPEG" || (codec == "V_MS/VFW/FOURCC" && fourcc == "MJPG")))
            throw MkvFormatException("Not an MJPEG recording ($codec $fourcc)")
        if (cues < 0) throw MkvFormatException("Segment has no index (still recording, or cut short)")
        if (duration < 0) throw MkvFormatException("Segment has no duration")
        return MkvInfo(dataStart.toLong(), cues, scale, (duration * scale / 1e6).toLong(), track, width, height,
            if (frameNs > 0) frameNs / 1e6 else 1000.0 / 30)
    }

    /** Parses the Cues element at the start of [b] into clusters by time (one entry per cluster). */
    fun parseCues(b: ByteArray, info: MkvInfo): List<MkvCluster> {
        val cues = element(b, 0, b.size)
        if (cues.id != CUES) throw MkvFormatException("No Cues at the indexed position")
        val first = HashMap<Long, Long>()
        for (point in children(b, cues.body, cues.end)) if (point.id == CUE_POINT) {
            var time = -1L; var position = -1L
            for (f in children(b, point.body, point.end)) when (f.id) {
                CUE_TIME -> time = uint(b, f)
                CUE_TRACK_POSITIONS -> for (g in children(b, f.body, f.end))
                    if (g.id == CUE_CLUSTER_POSITION) position = uint(b, g)
            }
            if (time < 0 || position < 0) continue
            val ms = time * info.timecodeScaleNs / 1_000_000
            val offset = info.segmentDataStart + position
            first[offset] = minOf(first[offset] ?: Long.MAX_VALUE, ms)
        }
        if (first.isEmpty()) throw MkvFormatException("Empty index")
        return first.map { MkvCluster(it.key, it.value) }.sortedBy { it.offset }
    }

    /** The frames of one whole Cluster element ([b] starts at its ID), in order. */
    fun parseCluster(b: ByteArray, info: MkvInfo): List<MkvFrame> {
        val cluster = element(b, 0, b.size)
        if (cluster.id != CLUSTER) throw MkvFormatException("No Cluster at the indexed position")
        var base = 0L
        val frames = ArrayList<MkvFrame>(32)
        fun block(e: Element) {
            val (track, n) = vint(b, e.body, false)
            if (e.body + n + 3 > e.end) throw MkvFormatException("Truncated block")
            if (track.toInt() != info.track) return
            val relative = ((b[e.body + n].toInt() shl 8) or (b[e.body + n + 1].toInt() and 0xFF)).toShort().toLong()
            val flags = b[e.body + n + 2].toInt()
            if (flags and 0x06 != 0) throw MkvFormatException("Laced blocks are not supported")
            val data = e.body + n + 3
            frames += MkvFrame((base + relative) * info.timecodeScaleNs / 1_000_000, b, data, e.end - data)
        }
        for (e in children(b, cluster.body, cluster.end)) when (e.id) {
            TIMECODE -> base = uint(b, e)
            SIMPLE_BLOCK -> block(e)
            BLOCK_GROUP -> for (f in children(b, e.body, e.end)) if (f.id == BLOCK) block(f)
        }
        return frames
    }

    /** Index of the cluster that holds time [ms]: the last one starting at or before it. */
    fun clusterAt(clusters: List<MkvCluster>, ms: Long): Int {
        var lo = 0; var hi = clusters.size - 1; var found = 0
        while (lo <= hi) {
            val mid = (lo + hi) ushr 1
            if (clusters[mid].timeMs <= ms) { found = mid; lo = mid + 1 } else hi = mid - 1
        }
        return found
    }
}
