package com.dashcam.connector

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.os.Bundle
import android.os.SystemClock
import androidx.activity.ComponentActivity
import androidx.activity.compose.BackHandler
import androidx.activity.compose.setContent
import androidx.compose.foundation.Image
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.unit.dp
import androidx.credentials.ClearCredentialStateRequest
import androidx.credentials.CredentialManager
import androidx.credentials.GetCredentialRequest
import com.google.android.libraries.identity.googleid.GetGoogleIdOption
import com.google.android.libraries.identity.googleid.GoogleIdTokenCredential
import com.google.firebase.auth.FirebaseAuth
import com.google.firebase.auth.GoogleAuthProvider
import kotlinx.coroutines.*
import kotlinx.coroutines.tasks.await
import okhttp3.*
import okhttp3.MediaType.Companion.toMediaType
import okhttp3.RequestBody.Companion.toRequestBody
import okio.ByteString
import org.json.JSONArray
import org.json.JSONObject
import java.io.IOException
import java.util.concurrent.atomic.AtomicReference

private data class Vehicle(val id: String, val label: String, val owner: Boolean)
private class RelayError(val status: Int) : IOException("Relay returned HTTP " + status)
private data class Received<T>(val data: T, val time: Long = SystemClock.elapsedRealtime())

class CloudActivity : ComponentActivity() {
    // This client uses Android's default network and system trust store.
    private val http = OkHttpClient.Builder().followRedirects(false).build()
    private val auth by lazy { FirebaseAuth.getInstance() }
    private val origin by lazy { getString(R.string.cloud_relay_origin) }
    private var watching by mutableStateOf(false)
    private var camera by mutableStateOf(false)
    private var socket: WebSocket? = null
    private var requestId by mutableStateOf("")
    private var cameraStart = 0L
    private var frameLatency by mutableStateOf<Long?>(null)

    override fun onStop() {
        watching = false
        camera = false
        socket?.close(1000, "Background")
        super.onStop()
    }

    private suspend fun request(path: String, refresh: Boolean = false): Request.Builder {
        val token = requireNotNull(auth.currentUser?.getIdToken(refresh)?.await()?.token)
        return Request.Builder().url(origin + path).header("Authorization", "Bearer " + token)
    }

    private suspend fun api(path: String, update: JSONObject? = null): String {
        val builder = request(path)
        if (update != null) builder.put(update.toString().toRequestBody("application/json".toMediaType()))
        return withContext(Dispatchers.IO) {
            http.newCall(builder.build()).execute().use {
                if (!it.isSuccessful) throw RelayError(it.code)
                requireNotNull(it.body).string()
            }
        }
    }

    private suspend fun login() {
        val option = GetGoogleIdOption.Builder().setServerClientId(getString(R.string.default_web_client_id))
            .setFilterByAuthorizedAccounts(false).build()
        val result = CredentialManager.create(this).getCredential(this,
            GetCredentialRequest.Builder().addCredentialOption(option).build())
        val token = GoogleIdTokenCredential.createFrom(result.credential.data).idToken
        auth.signInWithCredential(GoogleAuthProvider.getCredential(token, null)).await()
    }

    private fun demand() = if (camera) """{"channels":["telemetry","video"]}""" else """{"channels":["telemetry"]}"""

    private suspend fun stream(vehicle: Vehicle, status: (String) -> Unit, sample: (Snapshot?, Long) -> Unit,
                               picture: (Bitmap?) -> Unit) {
        val start = SystemClock.elapsedRealtime()
        var latency: Long? = null
        while (watching) {
            requestId = ""
            status("Connecting")
            val latest = AtomicReference<Received<String>?>()
            val frame = AtomicReference<Received<ByteString>?>()
            val closed = CompletableDeferred<Int>()
            val ws = http.newWebSocket(request("/v1/view/" + vehicle.id, refresh = true).build(), object : WebSocketListener() {
                override fun onMessage(webSocket: WebSocket, text: String) { latest.set(Received(text)) }
                override fun onMessage(webSocket: WebSocket, bytes: ByteString) { frame.set(Received(bytes)) }
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
            try {
                while (watching && !closed.isCompleted && SystemClock.elapsedRealtime() < reconnectAt) {
                    val now = SystemClock.elapsedRealtime()
                    if (now >= renew) { ws.send(demand()); renew = now + 10000 }
                    latest.getAndSet(null)?.let {
                        val s = JSONObject(it.data)
                        if (s.optString("type") == "connecting") requestId = s.getString("requestId")
                        if (s.optString("type") == "snapshot") {
                            lastSample = it.time
                            val fresh = s.optBoolean("runtime_fresh")
                            if (fresh && latency == null) latency = it.time - start
                            status(if (fresh) "Live · first fresh sample " + latency + " ms" else "Online · recorder data unavailable")
                            sample(Snapshot(s), it.time)
                        }
                    }
                    frame.getAndSet(null)?.takeIf { camera }?.let {
                        val bytes = it.data.toByteArray()
                        val h = VideoHeader.parse(bytes)
                        if (h != null && h.ageMs <= VIDEO_STALE_MS) {
                            val image = withContext(Dispatchers.Default) {
                                BitmapFactory.decodeByteArray(bytes, h.headerLength, bytes.size - h.headerLength)
                            }
                            if (camera && image != null) {
                                picture(image)
                                if (frameLatency == null) frameLatency = SystemClock.elapsedRealtime() - cameraStart
                            }
                            lastFrame = it.time
                        }
                    }
                    if (lastSample > 0 && now - lastSample > 2000) {
                        sample(null, 0); status("Waiting for fresh telemetry"); lastSample = 0
                    }
                    if (!camera || now - lastFrame > VIDEO_STALE_MS) picture(null)
                    delay(100)
                }
                if (closed.isCompleted && closed.await() in listOf(401, 403, 1008))
                    throw IOException("Session ended. Check vehicle access and sign in again.")
            } finally {
                ws.close(1000, null)
                if (socket === ws) socket = null
                sample(null, 0); picture(null)
            }
            if (watching) { status("Reconnecting"); delay(2000) }
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            MaterialTheme(colorScheme = DashcamColors) {
                Surface(modifier = Modifier.fillMaxSize(), color = MaterialTheme.colorScheme.background,
                    contentColor = MaterialTheme.colorScheme.onSurface) {
                    Screen()
                }
            }
        }
    }

    @Composable
    private fun Screen() {
        var uid by remember { mutableStateOf(auth.currentUser?.uid) }
        var vehicles by remember { mutableStateOf(emptyList<Vehicle>()) }
        var selected by remember { mutableStateOf<Vehicle?>(null) }
        var policy by remember { mutableStateOf<JSONObject?>(null) }
        var status by remember { mutableStateOf("") }
        val monitor = remember { mutableStateOf(MonitorState()) }
        var dashboard by remember { mutableStateOf(false) }
        var showDetails by remember { mutableStateOf(false) }
        var image by remember { mutableStateOf<Bitmap?>(null) }
        val scope = rememberCoroutineScope()
        fun setCamera(enabled: Boolean) {
            if (camera == enabled) return
            camera = enabled
            cameraStart = SystemClock.elapsedRealtime()
            frameLatency = null
            socket?.send(demand())
            if (!enabled) image = null
        }
        suspend fun attempt(block: suspend () -> Unit) {
            try { block() } catch (e: CancellationException) { throw e }
            catch (e: Exception) { status = e.localizedMessage ?: "Connection failed" }
        }
        LaunchedEffect(uid) {
            vehicles = emptyList(); selected = null
            if (uid != null && origin.isNotEmpty()) attempt {
                val list = JSONArray(api("/v1/vehicles"))
                vehicles = (0 until list.length()).map { i -> list.getJSONObject(i).let {
                    Vehicle(it.getString("id"), it.getString("label"), it.getString("role") == "owner")
                } }
                selected = vehicles.firstOrNull()
                status = if (vehicles.isEmpty()) "No vehicles assigned" else "Select a vehicle"
            }
        }
        LaunchedEffect(selected) {
            policy = null
            selected?.takeIf { it.owner }?.let { attempt { policy = JSONObject(api("/v1/vehicles/" + it.id + "/policy")) } }
        }
        LaunchedEffect(watching, selected) {
            if (watching) selected?.let {
                try {
                    attempt {
                        stream(it, { text ->
                            status = text
                            monitor.value = monitor.value.copy(status = text)
                        }, { snapshot, received ->
                            monitor.value = MonitorState(status, snapshot, received, SystemClock.elapsedRealtime(), snapshot != null)
                        }, { bitmap -> image = bitmap })
                    }
                } finally {
                    dashboard = false; watching = false; camera = false; image = null
                    monitor.value = MonitorState()
                }
            }
        }
        val vehicle = selected
        if (dashboard && watching && vehicle != null) {
            val map = remember(vehicle.id) { MapViewState() }
            val tiles = remember { TileLoader(applicationContext, scope) }
            DisposableEffect(tiles) { onDispose { tiles.close() } }
            LaunchedEffect(map) {
                snapshotFlow {
                    val state = monitor.value
                    val fix = state.snapshot?.vehicle?.takeIf { state.runtimeFresh }
                    TileMath.fix(fix.finite("latitude"), fix.finite("longitude"), fix.finite("heading_deg"))
                }.collect { map.update(it) }
            }
            LaunchedEffect(Unit) {
                while (true) {
                    delay(500)
                    monitor.value = monitor.value.copy(nowMs = SystemClock.elapsedRealtime())
                    if (camera && SystemClock.elapsedRealtime() - cameraStart >= LIVE_VIEW_MAX_MS) setCamera(false)
                }
            }
            BackHandler { dashboard = false }
            DashboardLayout(state = monitor, deviceName = vehicle.label, initialPage = 0,
                onKeepAwake = { window.decorView.keepScreenOn = it },
                onVideoWanted = { visible -> if (!visible) setCamera(false) },
                cameraCard = {
                    SurfaceCard("Camera") {
                        val frame = image
                        if (camera && frame != null) Image(frame.asImageBitmap(), "Camera preview",
                            Modifier.fillMaxWidth().aspectRatio(16f / 9f))
                        else Hint(if (camera) "Waiting for camera frames" else "Live view off")
                        OutlinedButton(onClick = { setCamera(!camera) }) {
                            Text(if (camera) "Stop live view" else "Start live view")
                        }
                    }
                }, mapCard = { MapCard(map, tiles) }) {
                item { Button(onClick = { dashboard = false }) { Text("Cloud vehicles") } }
            }
            return
        }
        Column(Modifier.fillMaxSize().safeDrawingPadding().verticalScroll(rememberScrollState()).padding(16.dp),
               verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("Cloud vehicles", style = MaterialTheme.typography.headlineMedium)
            if (origin.isEmpty()) Text("This build needs a staging relay URL.")
            else if (uid == null) Button(onClick = { scope.launch { attempt { login(); uid = auth.currentUser?.uid } } }) {
                Text("Sign in with Google")
            } else {
                vehicles.forEach { v -> OutlinedButton(enabled = !watching, onClick = { selected = v }) { Text(v.label) } }
                selected?.let { v ->
                    Button(onClick = { watching = !watching; if (!watching) status = "Disconnected" }) {
                        Text(if (watching) "Disconnect" else "Connect to " + v.label)
                    }
                    if (watching && monitor.value.snapshot != null) {
                        Button(onClick = { dashboard = true }) { Text("Open dashboard") }
                    }
                    if (watching) Row {
                        Checkbox(checked = camera, onCheckedChange = { setCamera(it) })
                        Text("Camera")
                    }
                    policy?.takeIf { v.owner }?.let { p ->
                        val ready = p.optString("mode") == "ready_while_powered"
                        OutlinedButton(onClick = { scope.launch { attempt {
                            val path = "/v1/vehicles/" + v.id + "/policy"
                            try { api(path, JSONObject().put("mode", if (ready) "on_demand" else "ready_while_powered")
                                .put("version", p.getInt("version"))) }
                            finally { val current = JSONObject(api(path)); if (selected == v) policy = current }
                        } } }) { Text(if (ready) "Switch to on demand" else "Switch to ready while powered") }
                    }
                }
                OutlinedButton(onClick = {
                    watching = false; camera = false; auth.signOut(); uid = null; status = "Signed out"
                    scope.launch { attempt { CredentialManager.create(this@CloudActivity)
                        .clearCredentialState(ClearCredentialStateRequest()) } }
                }) { Text("Sign out") }
            }
            Text(status)
            if (watching && requestId.isNotEmpty()) Text("Request " + requestId)
            if (watching && camera) frameLatency?.let { Text("First camera frame " + it + " ms") }
            if (camera && watching) image?.let { Image(it.asImageBitmap(), "Camera preview", Modifier.fillMaxWidth()) }
            if (watching) monitor.value.snapshot?.let { snapshot ->
                val json = snapshot.json
                val speed = snapshot.vehicle.finite("speed_kmh")
                Text(listOfNotNull("Sample " + json.optLong("seq"),
                    speed?.let { "%.0f km/h".format(it) },
                    snapshot.runtime?.let { if (it.optBoolean("recording")) "Recording" else "Recorder idle" }
                ).joinToString(" · "))
                TextButton(onClick = { showDetails = !showDetails }) {
                    Text(if (showDetails) "Hide sample details" else "Show sample details")
                }
                if (showDetails) Text(json.toString(2))
            }
        }
    }
}
