package com.dashcam.connector

import android.os.Bundle
import androidx.activity.ComponentActivity
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
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import com.google.android.libraries.identity.googleid.GetGoogleIdOption
import com.google.android.libraries.identity.googleid.GoogleIdTokenCredential
import com.google.firebase.auth.FirebaseAuth
import com.google.firebase.auth.GoogleAuthProvider
import kotlinx.coroutines.*
import kotlinx.coroutines.tasks.await
import okhttp3.*
import okhttp3.MediaType.Companion.toMediaType
import okhttp3.RequestBody.Companion.toRequestBody
import org.json.JSONArray
import org.json.JSONObject
import java.io.IOException

private class RelayError(val status: Int) : IOException("Relay returned HTTP " + status)

class CloudActivity : ComponentActivity() {
    // This client uses Android's default network and system trust store.
    private val http = OkHttpClient.Builder().followRedirects(false).build()
    private val auth by lazy { FirebaseAuth.getInstance() }
    private val origin by lazy { getString(R.string.cloud_relay_origin) }

    // The session belongs to the process (CloudSession): leaving this screen keeps it,
    // and the main dashboard shows it.
    // This screen shows the camera picture inline, so live view may stream while it is started.
    override fun onStart() { super.onStart(); CloudSession.activityStarted(); CloudSession.setScreenVideo(true) }
    override fun onStop() { CloudSession.setScreenVideo(false); CloudSession.activityStopped(); super.onStop() }

    private suspend fun request(path: String): Request.Builder {
        val token = requireNotNull(auth.currentUser?.getIdToken(false)?.await()?.token)
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
        var vehicles by remember { mutableStateOf(emptyList<CloudVehicle>()) }
        var selected by remember { mutableStateOf<CloudVehicle?>(null) }
        var policy by remember { mutableStateOf<JSONObject?>(null) }
        var status by remember { mutableStateOf("") }
        var showDetails by remember { mutableStateOf(false) }
        val connected by CloudSession.vehicle.collectAsStateWithLifecycle()
        val monitor by CloudSession.state.collectAsStateWithLifecycle()
        val sessionStatus by CloudSession.status.collectAsStateWithLifecycle()
        val camera by CloudSession.camera.collectAsStateWithLifecycle()
        val image by CloudSession.picture.collectAsStateWithLifecycle()
        val requestId by CloudSession.requestId.collectAsStateWithLifecycle()
        val frameLatency by CloudSession.frameLatency.collectAsStateWithLifecycle()
        val watching = connected != null
        val scope = rememberCoroutineScope()
        suspend fun attempt(block: suspend () -> Unit) {
            try { block() } catch (e: CancellationException) { throw e }
            catch (e: Exception) { status = e.localizedMessage ?: "Connection failed" }
        }
        LaunchedEffect(uid) {
            vehicles = emptyList(); selected = null
            if (uid != null && origin.isNotEmpty()) attempt {
                val list = JSONArray(api("/v1/vehicles"))
                vehicles = (0 until list.length()).map { i -> list.getJSONObject(i).let {
                    CloudVehicle(it.getString("id"), it.getString("label"), it.getString("role") == "owner")
                } }
                // Reopened during a session: show the connected vehicle's controls.
                selected = vehicles.find { it.id == CloudSession.vehicle.value?.id } ?: vehicles.firstOrNull()
                status = if (vehicles.isEmpty()) "No vehicles assigned" else "Select a vehicle"
            }
        }
        LaunchedEffect(selected) {
            policy = null
            selected?.takeIf { it.owner }?.let { attempt { policy = JSONObject(api("/v1/vehicles/" + it.id + "/policy")) } }
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
                    Button(onClick = { if (watching) CloudSession.disconnect() else CloudSession.connect(origin, v) }) {
                        Text(if (watching) "Disconnect" else "Connect to " + v.label)
                    }
                    // Back does the same: the main dashboard shows the connected session.
                    if (watching && monitor.snapshot != null) {
                        Button(onClick = { finish() }) { Text("Open dashboard") }
                    }
                    if (watching) Row {
                        Checkbox(checked = camera, onCheckedChange = { CloudSession.setCamera(it) })
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
                    CloudSession.disconnect("Signed out"); auth.signOut(); uid = null; status = "Signed out"
                    scope.launch { attempt { CredentialManager.create(this@CloudActivity)
                        .clearCredentialState(ClearCredentialStateRequest()) } }
                }) { Text("Sign out") }
            }
            Text(status)
            if (sessionStatus.isNotEmpty() && sessionStatus != status) Text(sessionStatus)
            if (watching && requestId.isNotEmpty()) Text("Request " + requestId)
            if (watching && camera) frameLatency?.let { Text("First camera frame " + it + " ms") }
            if (camera && watching) image?.let { Image(it.asImageBitmap(), "Camera preview", Modifier.fillMaxWidth()) }
            if (watching) monitor.snapshot?.let { snapshot ->
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
