package com.dashcam.connector

import org.json.JSONException
import org.json.JSONObject
import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.atomic.AtomicReference

/** A cold request may wait this long for the vehicle's first snapshot, across reconnects. */
const val CONNECT_DEADLINE_MS = 30_000L
/** The relay's close code for a vehicle whose viewer slots are all taken (1013 Try Again Later). */
const val RELAY_VEHICLE_FULL = 1013

internal enum class RelayClose { REJECTED, FULL, OTHER }

/** How the session treats a relay close code or a refused upgrade's HTTP status. */
internal fun relayClose(code: Int) = when (code) {
    401, 403, 1008 -> RelayClose.REJECTED   // access: retry once with a refreshed token
    RELAY_VEHICLE_FULL -> RelayClose.FULL   // capacity: retry later with the same token
    else -> RelayClose.OTHER
}

internal data class Received<T>(val data: T, val time: Long)

/** Relay answers to a recordings request (connector/README.md "Cloud recordings"), each kept for its request. */
internal val RECORDING_REPLIES = setOf("recordings", "clip", "clip_status", "error")

/**
 * Text messages from one relay socket, handed from OkHttp's thread to the session loop.
 * The connecting message has its own slot: in the slot it once shared with telemetry, a
 * first snapshot arriving within one polling interval overwrote the request ID. Answers to
 * recordings requests are queued: every one of them matters, not only the latest.
 */
internal class RelayInbox(private val clock: () -> Long) {
    private val requestId = AtomicReference<String?>()
    private val snapshot = AtomicReference<Received<JSONObject>?>()
    private val replies = ConcurrentLinkedQueue<JSONObject>()
    @Volatile var invalid = false
        private set

    fun offer(text: String) {
        val j = try { JSONObject(text) } catch (_: JSONException) { invalid = true; return }
        val type = j.optString("type")
        when {
            type == "connecting" -> requestId.set(j.optString("requestId"))
            type in RECORDING_REPLIES -> replies.add(j)
            else -> snapshot.set(Received(j, clock()))
        }
    }

    fun takeRequestId(): String? = requestId.getAndSet(null)
    fun takeSnapshot(): Received<JSONObject>? = snapshot.getAndSet(null)
    fun takeReply(): JSONObject? = replies.poll()
}
