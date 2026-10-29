package com.dashcam.connector

import org.json.JSONObject
import java.net.URI
import java.net.URLDecoder
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
