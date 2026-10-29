package com.dashcam.connector

import androidx.compose.ui.graphics.ImageBitmap
import org.json.JSONObject
import java.net.URI
import java.net.URLDecoder
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.Locale

data class DeviceProfile(
    val id: String, val name: String, val host: String, val port: Int,
    val pin: String, val token: String
) {
    fun toJson() = JSONObject().put("id", id).put("name", name).put("host", host)
        .put("port", port).put("pin", pin).put("token", token)
    companion object {
        fun fromJson(j: JSONObject) = DeviceProfile(j.getString("id"), j.getString("name"),
            j.getString("host"), j.getInt("port"), j.getString("pin"), j.getString("token"))
        fun parse(link: String): DeviceProfile {
            val uri = URI(link.trim())
            require(uri.scheme == "dashcam" && uri.host == "pair") { "Paste a dashcam://pair link" }
            val args = (uri.rawQuery ?: "").split('&').associate { part ->
                val bits = part.split('=', limit = 2)
                require(bits.size == 2) { "Invalid pairing link" }
                URLDecoder.decode(bits[0], "UTF-8") to URLDecoder.decode(bits[1], "UTF-8")
            }
            val host = args["host"].orEmpty().trim()
            require(host.isNotEmpty() && host.length <= 253 && host.all { it.isLetterOrDigit() || it in ".-:" }) { "Invalid host" }
            val port = args["port"]?.toIntOrNull() ?: 0
            require(port in 1..65535) { "Invalid port" }
            val pin = args["pin"].orEmpty().lowercase(Locale.ROOT)
            require(pin.matches(Regex("[a-f0-9]{64}"))) { "Invalid certificate fingerprint" }
            val token = args["token"].orEmpty()
            require(token.matches(Regex("[A-Za-z0-9_-]{32,128}"))) { "Invalid device credential" }
            val id = args["id"].orEmpty()
            require(id.matches(Regex("[a-fA-F0-9-]{36}"))) { "Invalid device ID" }
            return DeviceProfile(id, args["name"].orEmpty().take(80).ifBlank { "Jetson dashcam" }, host, port, pin, token)
        }
    }
}

data class Snapshot(val json: JSONObject) {
    val runtimeFresh get() = json.optBoolean("runtime_fresh") && json.optJSONObject("runtime") != null
    val runtime get() = if (runtimeFresh) json.optJSONObject("runtime") else null
    val vehicle get() = runtime?.takeIf { it.optBoolean("bridge_fresh") }?.optJSONObject("vehicle")
    val system get() = json.optJSONObject("system")
    companion object {
        fun parse(text: String, deviceId: String): Snapshot {
            require(text.length <= 65536) { "Snapshot too large" }
            val json = JSONObject(text)
            require(json.getInt("schema") == 1 && json.getString("type") == "snapshot") { "Unsupported monitoring protocol" }
            require(json.getString("device_id") == deviceId) { "Device identity does not match pairing" }
            require(json.getLong("seq") >= 0 && json.getString("session").isNotBlank()) { "Invalid snapshot sequence" }
            return Snapshot(json)
        }
    }
}

fun JSONObject?.finite(key: String): Double? = this?.takeUnless { it.isNull(key) }
    ?.optDouble(key, Double.NaN)?.takeIf { it.isFinite() }

data class MonitorState(
    val status: String = "Disconnected", val snapshot: Snapshot? = null,
    val lastReceiveMs: Long = 0, val nowMs: Long = 0, val active: Boolean = false
) {
    val transportFresh get() = active && lastReceiveMs > 0 && nowMs - lastReceiveMs <= 2000
    val runtimeFresh get() = transportFresh && snapshot?.runtimeFresh == true
}

// /v1/video binary message header (24 bytes, little-endian): "DCV1", uint32
// header length (>= 24, JPEG starts there), uint64 agent sequence, uint32 age
// in ms at send time, uint16 width, uint16 height. Same checks as the web client.
data class VideoHeader(val headerLength: Int, val sequence: Long, val ageMs: Long, val width: Int, val height: Int) {
    companion object {
        const val MIN_LENGTH = 24
        fun parse(data: ByteArray): VideoHeader? {
            if (data.size < MIN_LENGTH) return null
            if (data[0] != 'D'.code.toByte() || data[1] != 'C'.code.toByte() ||
                data[2] != 'V'.code.toByte() || data[3] != '1'.code.toByte()) return null
            val buffer = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN)
            val headerLength = buffer.getInt(4).toLong() and 0xFFFFFFFFL
            if (headerLength < MIN_LENGTH || headerLength > data.size) return null
            val width = buffer.getShort(20).toInt() and 0xFFFF
            val height = buffer.getShort(22).toInt() and 0xFFFF
            if (width !in 1..4096 || height !in 1..4096) return null
            return VideoHeader(headerLength.toInt(), buffer.getLong(8),
                buffer.getInt(16).toLong() and 0xFFFFFFFFL, width, height)
        }
    }
}

// Not a data class: every decoded frame is a distinct value for StateFlow.
class VideoFrame(val image: ImageBitmap, val width: Int, val height: Int, val ageMs: Long,
                 val receivedAtMs: Long, val fps: Double?)

// connected: the /v1/video socket of this state is open; a closed or retrying
// connection never carries a frame, so it shows "No video".
data class VideoState(val status: String = "Video paused", val frame: VideoFrame? = null, val nowMs: Long = 0,
                      val connected: Boolean = false) {
    // Receipt time on the elapsed-realtime clock; an old picture is never shown.
    val stale get() = frame?.let { nowMs - it.receivedAtMs > 2000 } == true
}
