package com.dashcam.connector

// Plain Kotlin, no Android types: unit-tested on the JVM.
//
// The recorder's .ass captions (connector/README.md "Recordings"): a few styles that pin text to a
// corner, the top centre or the left middle, and short Dialogue lines (5 Hz) of plain text with \N
// line breaks. Only what those files use is read: style alignment, font size and margins, event
// times, style and text. Override tags ({...}) are dropped, never interpreted.

/** A style: numpad [alignment] (7 8 9 / 4 5 6 / 1 2 3), sizes in PlayRes pixels. */
data class AssStyle(val alignment: Int, val fontSize: Float, val marginL: Int, val marginR: Int, val marginV: Int)

class AssEvent(val startMs: Long, val endMs: Long, val style: String, val text: String)

/** One caption to draw now. */
data class AssCaption(val style: AssStyle, val text: String)

class AssScript(val playResX: Int, val playResY: Int, val styles: Map<String, AssStyle>, events: List<AssEvent>) {
    // Per style, sorted by start: a style's lines replace each other, so at most one is shown per style.
    private val byStyle: Map<String, List<AssEvent>> = events.groupBy { it.style }.mapValues { (_, list) -> list.sortedBy { it.startMs } }
    val eventCount = events.size

    /** The captions visible at [ms], one per style at most, in style order. */
    fun at(ms: Long): List<AssCaption> {
        val out = ArrayList<AssCaption>(byStyle.size)
        for ((name, list) in byStyle) {
            val style = styles[name] ?: continue
            var lo = 0; var hi = list.size - 1; var found = -1
            while (lo <= hi) {
                val mid = (lo + hi) ushr 1
                if (list[mid].startMs <= ms) { found = mid; lo = mid + 1 } else hi = mid - 1
            }
            if (found >= 0 && ms < list[found].endMs && list[found].text.isNotBlank()) out += AssCaption(style, list[found].text)
        }
        return out
    }

    companion object {
        // Both braces escaped: Android's ICU regex engine rejects a bare "}" that the desktop JVM accepts
        // (PatternSyntaxException at class load, which crashed playback; the JVM unit tests never saw it).
        private val OVERRIDE = Regex("\\{[^}]*\\}")
        private const val BYTE_ORDER_MARK = 0xFEFF.toChar()

        /** h:mm:ss.cc (centiseconds) to milliseconds; -1 when malformed. */
        fun time(text: String): Long {
            val parts = text.trim().split(':')
            if (parts.size != 3) return -1
            val h = parts[0].toLongOrNull() ?: return -1
            val m = parts[1].toLongOrNull() ?: return -1
            val sec = parts[2].toDoubleOrNull() ?: return -1
            if (h < 0 || m !in 0..59 || sec < 0 || sec >= 60) return -1
            return h * 3_600_000 + m * 60_000 + Math.round(sec * 1000)
        }

        fun parse(source: String): AssScript {
            var section = ""
            var playResX = 384; var playResY = 288   // the ASS defaults
            var styleFormat = emptyList<String>()
            var eventFormat = emptyList<String>()
            val styles = HashMap<String, AssStyle>()
            val events = ArrayList<AssEvent>()
            for (raw in source.lineSequence()) {
                val line = raw.trimEnd('\r').trimStart(BYTE_ORDER_MARK)
                if (line.startsWith("[")) { section = line.trim().lowercase(); continue }
                val colon = line.indexOf(':')
                if (colon < 0) continue
                val key = line.substring(0, colon).trim()
                val value = line.substring(colon + 1).trimStart()
                when (section) {
                    "[script info]" -> when (key) {
                        "PlayResX" -> playResX = value.trim().toIntOrNull() ?: playResX
                        "PlayResY" -> playResY = value.trim().toIntOrNull() ?: playResY
                    }
                    "[v4+ styles]", "[v4 styles]" -> when (key) {
                        "Format" -> styleFormat = value.split(',').map { it.trim().lowercase() }
                        "Style" -> {
                            val f = value.split(',').map { it.trim() }
                            fun field(name: String) = styleFormat.indexOf(name).takeIf { it in f.indices }?.let { f[it] }
                            val name = field("name") ?: continue
                            styles[name] = AssStyle(
                                alignment = field("alignment")?.toIntOrNull()?.takeIf { it in 1..9 } ?: 2,
                                fontSize = field("fontsize")?.toFloatOrNull()?.takeIf { it > 0 } ?: 20f,
                                marginL = field("marginl")?.toIntOrNull() ?: 0,
                                marginR = field("marginr")?.toIntOrNull() ?: 0,
                                marginV = field("marginv")?.toIntOrNull() ?: 0)
                        }
                    }
                    "[events]" -> when (key) {
                        "Format" -> eventFormat = value.split(',').map { it.trim().lowercase() }
                        "Dialogue" -> {
                            // Text is the last field and may itself contain commas.
                            val textIndex = eventFormat.indexOf("text")
                            if (textIndex < 0) continue
                            val f = value.split(',', limit = textIndex + 1)
                            if (f.size != textIndex + 1) continue
                            fun field(name: String) = eventFormat.indexOf(name).takeIf { it in f.indices }?.let { f[it].trim() }
                            val start = time(field("start") ?: continue)
                            val end = time(field("end") ?: continue)
                            if (start < 0 || end <= start) continue
                            val text = f[textIndex].replace(OVERRIDE, "").replace("\\N", "\n").replace("\\n", "\n").replace("\\h", " ")
                            events += AssEvent(start, end, field("style") ?: "Default", text)
                        }
                    }
                }
            }
            return AssScript(playResX, playResY, styles, events)
        }
    }
}
