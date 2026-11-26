package com.dashcam.connector

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.os.SystemClock
import com.google.firebase.auth.FirebaseAuth
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.tasks.await
import okhttp3.*
import okio.ByteString
import org.json.JSONObject
import java.io.IOException
import java.util.UUID
import java.util.concurrent.atomic.AtomicReference

internal data class CloudVehicle(val id: String, val label: String, val owner: Boolean)

/** The app may be in the background this long before the relay socket closes (covers activity switches and rotation). */
private const val BACKGROUND_GRACE_MS = 1000L

/**
 * The cloud viewing session, owned by the process rather than an activity, so that
 * leaving Cloud vehicles (Back or Open dashboard) keeps it and the main dashboard
 * shows it. The relay socket is open only while the app is in the foreground: it
 * closes in the background (no relay traffic) and reopens on return while the
 * user is still connected. Disconnect, sign-out or a rejected session end it.
 * All state is changed on the main thread; socket callbacks only hand data over.
 */
internal object CloudSession {
    // This client uses Android's default network and system trust store.
    private val http = OkHttpClient.Builder().followRedirects(false).build()
    private val scope = MainScope()

    private val vehicleMutable = MutableStateFlow<CloudVehicle?>(null)
    /** The vehicle the user connected to; null when not connected. */
    val vehicle = vehicleMutable.asStateFlow()
    private val stateMutable = MutableStateFlow(MonitorState())
    val state = stateMutable.asStateFlow()
    private val statusMutable = MutableStateFlow("")
    /** Latest session status or the reason it ended, for the Cloud vehicles screen. */
    val status = statusMutable.asStateFlow()
    private val pictureMutable = MutableStateFlow<Bitmap?>(null)
    val picture = pictureMutable.asStateFlow()
    private val cameraMutable = MutableStateFlow(false)
    val camera = cameraMutable.asStateFlow()
    private val requestIdMutable = MutableStateFlow("")
    val requestId = requestIdMutable.asStateFlow()
    private val frameLatencyMutable = MutableStateFlow<Long?>(null)
    val frameLatency = frameLatencyMutable.asStateFlow()

    private var origin = ""
    private var socket: WebSocket? = null
    private var task: Job? = null
    // Bumped by every stop: a cancelled stream finishing late never writes over its successor.
    private var generation = 0
    private var cameraStart = 0L
    private var started = 0
    private var foreground = false
    private var pause: Job? = null
    // Recordings requests sent on the open socket and waiting for their answer, by request id.
    private val pending = HashMap<String, (JSONObject) -> Unit>()

    fun connect(origin: String, vehicle: CloudVehicle) {
        if (vehicleMutable.value == vehicle && task != null) return
        stop()
        this.origin = origin
        vehicleMutable.value = vehicle
        CloudRecordings.reset(vehicle.id)
        restart()
    }

    fun disconnect(reason: String = "Disconnected") {
        vehicleMutable.value = null
        stop()
        CloudRecordings.reset(null)
        publishStatus(reason)
    }

    /**
     * Sends a recordings request ({type: "recordings"} or {type: "clip", ...}) on the open relay socket;
     * [onReply] gets each answer on the main thread: for a clip a "clip_status" first, then "clip" or
     * "error". A socket that closes first answers "error". False when no socket is open.
     */
    fun request(body: JSONObject, onReply: (JSONObject) -> Unit): Boolean {
        val ws = socket ?: return false
        val id = "a" + UUID.randomUUID().toString().replace("-", "").take(20)
        if (!ws.send(body.put("id", id).toString())) return false
        pending[id] = onReply
        return true
    }

    private fun dispatch(reply: JSONObject) {
        val id = reply.optString("id")
        val handler = pending[id] ?: return   // an answer nobody waits for any more
        if (reply.optString("type") != "clip_status") pending.remove(id)
        handler(reply)
    }

    private fun failPending(why: String) {
        if (pending.isEmpty()) return
        val waiting = pending.values.toList()
        pending.clear()
        val reply = JSONObject().put("type", "error").put("error", why)
        waiting.forEach { it(reply) }
    }

    /** Called from every activity's onStart and onStop: the session streams only while one is started. */
    fun activityStarted() {
        started++
        pause?.cancel(); pause = null
        if (!foreground) { foreground = true; restart() }
    }

    fun activityStopped() {
        started = (started - 1).coerceAtLeast(0)
        if (started > 0 || pause != null) return
        pause = scope.launch {
            delay(BACKGROUND_GRACE_MS)
            pause = null
            foreground = false
            stop()
            if (vehicleMutable.value != null) publishStatus("Paused in the background")
        }
    }

    /** The user's live view toggle (Start/Stop, or the Camera checkbox); ends after LIVE_VIEW_MAX_MS. */
    fun setCamera(enabled: Boolean) {
        if (cameraMutable.value == enabled) return
        cameraMutable.value = enabled
        cameraStart = SystemClock.elapsedRealtime()
        frameLatencyMutable.value = null
        videoChanged()
    }

    // Video additionally needs a camera view on screen: the dashboard's Live camera card
    // (it reports visibility, and not-visible while it is rebuilt on rotation) or the Cloud
    // vehicles screen. Leaving the view pauses video; it never turns live view off.
    fun setDashboardVideo(visible: Boolean) {
        if (dashboardVideo == visible) return
        dashboardVideo = visible
        videoChanged()
    }

    fun setScreenVideo(visible: Boolean) {
        if (screenVideo == visible) return
        screenVideo = visible
        videoChanged()
    }

    private var dashboardVideo = false
    private var screenVideo = false
    private fun videoOn() = cameraMutable.value && (dashboardVideo || screenVideo)

    private fun videoChanged() {
        socket?.send(demand())
        if (!videoOn()) pictureMutable.value = null
    }

    private fun demand() =
        if (videoOn()) """{"channels":["telemetry","video"]}""" else """{"channels":["telemetry"]}"""

    private fun restart() {
        val v = vehicleMutable.value ?: return
        if (!foreground || task != null) return
        val mine = generation
        task = scope.launch { run(v, mine) }
    }

    // Closes the socket and clears what it showed; live view never outlives a socket.
    private fun stop() {
        generation++
        task?.cancel(); task = null
        socket?.close(1000, null); socket = null
        failPending("The cloud session closed · try again once it is connected")
        cameraMutable.value = false
        pictureMutable.value = null
        requestIdMutable.value = ""
        frameLatencyMutable.value = null
        stateMutable.value = MonitorState()
    }

    private fun publishStatus(text: String) {
        statusMutable.value = text
        stateMutable.update { it.copy(status = text) }
    }

    private suspend fun run(vehicle: CloudVehicle, mine: Int) {
        fun current() = mine == generation
        val start = SystemClock.elapsedRealtime()
        var latency: Long? = null
        // The cached ID token (Firebase refreshes it near expiry) saves a round trip to Google
        // on every connect; a forced refresh costs 2-5 s on mobile data. A rejected session
        // retries once with a forced refresh before it ends.
        var forceRefresh = false
        // Until the vehicle first answers, the request (renewed every 10 s) has a deadline across
        // reconnects; a vehicle coming online later must not start streaming to a viewer who left.
        // A full vehicle holds no lease for this viewer, so time spent waiting for a slot is not counted.
        var waitStart = start
        var answered = false
        try {
            while (current()) {
                requestIdMutable.value = ""
                publishStatus("Connecting")
                val token = requireNotNull(FirebaseAuth.getInstance().currentUser?.getIdToken(forceRefresh)?.await()?.token) {
                    "Sign in again"
                }
                if (!current()) return
                val inbox = RelayInbox(SystemClock::elapsedRealtime)
                val frame = AtomicReference<Received<ByteString>?>()
                val closed = CompletableDeferred<Int>()
                var full = false
                val ws = http.newWebSocket(Request.Builder().url(origin + "/v1/view/" + vehicle.id)
                    .header("Authorization", "Bearer " + token).build(), object : WebSocketListener() {
                    override fun onMessage(webSocket: WebSocket, text: String) { inbox.offer(text) }
                    override fun onMessage(webSocket: WebSocket, bytes: ByteString) {
                        frame.set(Received(bytes, SystemClock.elapsedRealtime()))
                    }
                    override fun onClosing(webSocket: WebSocket, code: Int, reason: String) {
                        webSocket.close(1000, null)
                        closed.complete(code)
                    }
                    override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
                        closed.complete(response?.code ?: 0)
                    }
                })
                socket = ws
                val reconnectAt = SystemClock.elapsedRealtime() + 55 * 60000L
                var renew = 0L
                var lastSample = 0L
                var lastFrame = 0L
                var lastTick = 0L
                try {
                    while (current() && !closed.isCompleted && SystemClock.elapsedRealtime() < reconnectAt) {
                        val now = SystemClock.elapsedRealtime()
                        if (!answered && now - waitStart >= CONNECT_DEADLINE_MS)
                            throw IOException("The vehicle did not answer within " + CONNECT_DEADLINE_MS / 1000 + " s. Tap Connect to try again.")
                        if (inbox.invalid) throw IOException("Unsupported data from the relay")
                        if (now >= renew) { ws.send(demand()); renew = now + 10000 }
                        if (cameraMutable.value && now - cameraStart >= LIVE_VIEW_MAX_MS) setCamera(false)
                        inbox.takeRequestId()?.let { requestIdMutable.value = it }
                        while (true) dispatch(inbox.takeReply() ?: break)
                        inbox.takeSnapshot()?.let {
                            val s = it.data
                            if (s.optString("type") == "snapshot") {
                                lastSample = it.time
                                answered = true
                                forceRefresh = false
                                val fresh = s.optBoolean("runtime_fresh")
                                if (fresh && latency == null) latency = it.time - start
                                val text = if (fresh) "Live · first fresh sample " + latency + " ms" else "Online · recorder data unavailable"
                                statusMutable.value = text
                                stateMutable.value = MonitorState(text, Snapshot(s), it.time, now, true)
                            }
                        }
                        frame.getAndSet(null)?.takeIf { videoOn() }?.let {
                            val bytes = it.data.toByteArray()
                            val h = VideoHeader.parse(bytes)
                            if (h != null && h.ageMs <= VIDEO_STALE_MS) {
                                val image = withContext(Dispatchers.Default) {
                                    BitmapFactory.decodeByteArray(bytes, h.headerLength, bytes.size - h.headerLength)
                                }
                                if (current() && videoOn() && image != null) {
                                    pictureMutable.value = image
                                    if (frameLatencyMutable.value == null)
                                        frameLatencyMutable.value = SystemClock.elapsedRealtime() - cameraStart
                                }
                                lastFrame = it.time
                            }
                        }
                        if (lastSample > 0 && now - lastSample > 2000) {
                            stateMutable.value = MonitorState("Waiting for fresh telemetry")
                            statusMutable.value = "Waiting for fresh telemetry"
                            lastSample = 0
                        }
                        if (now - lastTick >= 500) { stateMutable.update { s -> s.copy(nowMs = now) }; lastTick = now }
                        if (!videoOn() || now - lastFrame > VIDEO_STALE_MS) pictureMutable.value = null
                        delay(100)
                    }
                    val code = if (closed.isCompleted) closed.await() else 0
                    when (relayClose(code)) {
                        RelayClose.REJECTED -> {
                            if (forceRefresh) throw IOException("Session ended. Check vehicle access and sign in again.")
                            forceRefresh = true   // perhaps a stale cached token: retry at once with a fresh one
                            continue
                        }
                        RelayClose.FULL -> full = true
                        RelayClose.OTHER -> {}
                    }
                } finally {
                    ws.close(1000, null)
                    if (socket === ws) { socket = null; failPending("The connection to the relay closed · try again") }
                    if (current()) { stateMutable.value = MonitorState(stateMutable.value.status); pictureMutable.value = null }
                }
                if (!current()) break
                if (full) {
                    publishStatus("All viewer slots for this vehicle are in use · retrying")
                    delay(5000)
                    waitStart = SystemClock.elapsedRealtime()
                } else { publishStatus("Reconnecting"); delay(2000) }
            }
        } catch (e: CancellationException) {
            throw e
        } catch (e: Exception) {
            if (current()) disconnect(e.localizedMessage ?: "Connection failed")
        }
    }
}
