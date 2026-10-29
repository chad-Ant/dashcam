package com.dashcam.connector

import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.activity.viewModels
import androidx.compose.foundation.background
import androidx.compose.foundation.ExperimentalFoundationApi
import androidx.compose.foundation.Image
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.text.input.PasswordVisualTransformation
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import org.json.JSONObject
import java.util.Locale

class MainActivity : ComponentActivity() {
    private val model: DashboardModel by viewModels()
    private var foreground = false
    private val permission = registerForActivityResult(ActivityResultContracts.RequestPermission()) {
        model.repository.setActive(it && foreground)
    }
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            MaterialTheme(colorScheme = darkColorScheme(
                primary = Color(0xFF5CE0C6), background = Color(0xFF0B121B), surface = Color(0xFF15212D),
                secondaryContainer = Color(0xFF23443F), onSecondaryContainer = Color(0xFF5CE0C6),
                onSurface = Color(0xFFE6EEF5), onSurfaceVariant = Color(0xFFA2B3C3)
            )) {
                Dashboard(model, onKeepAwake = { enabled ->
                    if (enabled) window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
                    else window.clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
                }, onPermission = { ensurePermission() })
            }
        }
    }
    private fun ensurePermission() {
        if (Build.VERSION.SDK_INT >= 37 && checkSelfPermission("android.permission.ACCESS_LOCAL_NETWORK") != PackageManager.PERMISSION_GRANTED)
            permission.launch("android.permission.ACCESS_LOCAL_NETWORK")
        else model.repository.setActive(foreground)
    }
    override fun onStart() { super.onStart(); foreground = true; ensurePermission() }
    override fun onStop() {
        foreground = false; model.repository.setActive(false); model.discovery.stop()
        window.clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        super.onStop()
    }
}

private val pages = listOf("Live", "Vehicle", "System", "Connection")
private val marks = listOf("◉", "↗", "▥", "⌁")

@OptIn(ExperimentalFoundationApi::class)
@Composable
private fun Dashboard(model: DashboardModel, onKeepAwake: (Boolean) -> Unit, onPermission: () -> Unit) {
    val state by model.repository.state.collectAsStateWithLifecycle()
    val profiles by model.profiles.collectAsStateWithLifecycle()
    val selected by model.selected.collectAsStateWithLifecycle()
    val video by model.repository.video.collectAsStateWithLifecycle()
    var page by remember { mutableIntStateOf(if (profiles.isEmpty()) 3 else 0) }
    var awake by remember { mutableStateOf(false) }
    LaunchedEffect(awake, state.active) { onKeepAwake(awake && state.active) }
    // Camera frames stream only while Live is shown; foreground and device
    // selection are gated by the repository (setActive from onStart/onStop).
    DisposableEffect(page) {
        model.repository.setVideoWanted(page == 0)
        onDispose { model.repository.setVideoWanted(false) }
    }
    BoxWithConstraints(Modifier.fillMaxSize().background(MaterialTheme.colorScheme.background).safeDrawingPadding()) {
        val wide = maxWidth >= 840.dp
        Scaffold(containerColor = MaterialTheme.colorScheme.background,
            bottomBar = { if (!wide) NavigationBar {
                pages.forEachIndexed { i, name -> NavigationBarItem(selected = page == i, onClick = { page = i },
                    icon = { Text(marks[i], fontSize = 22.sp) }, label = { Text(name) }) }
            } }) { padding ->
            Row(Modifier.fillMaxSize().padding(padding)) {
                if (wide) NavigationRail(containerColor = MaterialTheme.colorScheme.background) {
                    Spacer(Modifier.height(24.dp))
                    Text("DC", fontWeight = FontWeight.Bold, color = MaterialTheme.colorScheme.primary, fontSize = 22.sp)
                    Spacer(Modifier.height(32.dp))
                    pages.forEachIndexed { i, name -> NavigationRailItem(selected = page == i, onClick = { page = i },
                        icon = { Text(marks[i], fontSize = 24.sp) }, label = { Text(name) }) }
                }
                LazyColumn(Modifier.weight(1f).fillMaxHeight(), contentPadding = PaddingValues(20.dp),
                    verticalArrangement = Arrangement.spacedBy(16.dp)) {
                    item {
                        Text("DASHCAM CONNECT", letterSpacing = 2.sp, color = MaterialTheme.colorScheme.primary,
                            style = MaterialTheme.typography.labelMedium)
                        Spacer(Modifier.height(8.dp))
                        Text(pages[page], style = MaterialTheme.typography.headlineLarge, fontWeight = FontWeight.SemiBold)
                        Text(profiles.find { it.id == selected }?.name ?: "Your Jetson, at a glance",
                            color = MaterialTheme.colorScheme.onSurfaceVariant)
                    }
                    stickyHeader { Box(Modifier.background(MaterialTheme.colorScheme.background).padding(bottom = 4.dp)) { ConnectionBanner(state) } }
                    if (page == 3) {
                        item { ConnectionPanel(model, onPermission) }
                        item { SurfaceCard("Display") {
                            Row(verticalAlignment = Alignment.CenterVertically) {
                                Text("Keep screen on while connected", Modifier.weight(1f))
                                Switch(checked = awake, onCheckedChange = { awake = it })
                            }
                            Hint("Monitoring pauses when the app is in the background.")
                        } }
                    } else {
                        val runtime = state.snapshot?.runtime?.takeIf { state.runtimeFresh }
                        val vehicle = state.snapshot?.vehicle?.takeIf { state.runtimeFresh }
                        val system = state.snapshot?.system?.takeIf { state.transportFresh }
                        when (page) {
                            0 -> {
                                item { CameraCard(video) }
                                item { SpeedCard(vehicle, runtime) }
                                item { AdaptivePair(wide, {
                                    SurfaceCard("Recording") {
                                        BigValue(when { runtime == null -> "Unavailable"; runtime.optBoolean("recording") -> "Recording"; else -> "Waiting" })
                                        Metric("Actual frame rate", runtime.finite("fps"), "fps", 1)
                                        Metric("Frames in session", runtime.finite("frames"), "", 0)
                                    }
                                }, { AdasCard() }) }
                                item { AdaptivePair(wide, { PositionCard(vehicle) }, { HealthSummary(system) }) }
                            }
                            1 -> {
                                item { SpeedCard(vehicle, runtime) }
                                item { AdaptivePair(wide, { SurfaceCard("Powertrain") {
                                    Metric("Engine speed", vehicle.finite("rpm"), "rpm", 0)
                                    Metric("Coolant", vehicle.finite("coolant_c"), "°C", 1)
                                    Metric("Fuel level", vehicle.finite("fuel_percent"), "%", 1)
                                    Metric("Throttle", vehicle.finite("throttle_percent"), "%", 1)
                                    Metric("Engine load", vehicle.finite("engine_load_percent"), "%", 1)
                                } }, { PositionCard(vehicle) }) }
                                item { SurfaceCard("Telemetry link") {
                                    BigValue(when { runtime == null -> "Runtime unavailable"; runtime.optBoolean("bridge_fresh") -> "Fresh"; else -> "No fresh bridge data" })
                                    Metric("Bridge sample age", runtime.finite("bridge_age_ms"), "ms", 0)
                                    Hint("Missing or stale sensor readings display as —.")
                                } }
                            }
                            2 -> {
                                item { AdaptivePair(wide, { HealthSummary(system) }, { SurfaceCard("Memory & storage") {
                                    Metric("RAM used", system.finite("ram_used_bytes")?.div(1073741824), "GiB", 2)
                                    Metric("RAM total", system.finite("ram_total_bytes")?.div(1073741824), "GiB", 2)
                                    Metric("Swap used", system.finite("swap_used_bytes")?.div(1073741824), "GiB", 2)
                                    Metric("Footage disk free", system.finite("storage_free_bytes")?.div(1073741824), "GiB", 1)
                                    Metric("Footage disk total", system.finite("storage_total_bytes")?.div(1073741824), "GiB", 1)
                                } }) }
                                item { SurfaceCard("Temperatures") {
                                    val temperatures = system?.optJSONObject("temperatures_c")
                                    if (temperatures == null || temperatures.length() == 0) Hint("Temperature sensors unavailable")
                                    else temperatures.keys().asSequence().sorted().take(64).forEach { Metric(it, temperatures.finite(it), "°C", 1) }
                                } }
                                item { AdasCard() }
                            }
                        }
                    }
                    item { Spacer(Modifier.height(8.dp)) }
                }
            }
        }
    }
}

@Composable private fun ConnectionBanner(state: MonitorState) {
    val color = if (state.transportFresh) MaterialTheme.colorScheme.primary else Color(0xFFFFCA80)
    Surface(color = color.copy(alpha = .1f), shape = RoundedCornerShape(16.dp)) {
        Column(Modifier.fillMaxWidth().padding(16.dp)) {
            Text("●  " + if (state.active && state.lastReceiveMs > 0 && !state.transportFresh) "Data stale · waiting for Jetson" else state.status,
                color = color, fontWeight = FontWeight.SemiBold)
            if (state.transportFresh) Text(
                if (state.runtimeFresh) "Updated ${((state.nowMs - state.lastReceiveMs).coerceAtLeast(0) / 1000.0).let { "%.1f".format(Locale.US, it) }} s ago"
                else "System health available; recorder data has expired", style = MaterialTheme.typography.bodySmall)
        }
    }
}
@Composable private fun SurfaceCard(title: String, content: @Composable ColumnScope.() -> Unit) {
    Surface(color = MaterialTheme.colorScheme.surface, shape = RoundedCornerShape(22.dp)) {
        Column(Modifier.fillMaxWidth().padding(20.dp), verticalArrangement = Arrangement.spacedBy(10.dp)) {
            Text(title, style = MaterialTheme.typography.titleMedium, color = MaterialTheme.colorScheme.onSurfaceVariant)
            content()
        }
    }
}
@Composable private fun AdaptivePair(wide: Boolean, first: @Composable () -> Unit, second: @Composable () -> Unit) {
    if (wide) Row(horizontalArrangement = Arrangement.spacedBy(16.dp)) {
        Box(Modifier.weight(1f)) { first() }; Box(Modifier.weight(1f)) { second() }
    } else Column(verticalArrangement = Arrangement.spacedBy(16.dp)) { first(); second() }
}
@Composable private fun BigValue(value: String) { Text(value, style = MaterialTheme.typography.headlineSmall, fontWeight = FontWeight.SemiBold) }
@Composable private fun Hint(value: String) { Text(value, color = MaterialTheme.colorScheme.onSurfaceVariant, style = MaterialTheme.typography.bodySmall) }
@Composable private fun Metric(label: String, value: Double?, unit: String, decimals: Int = 1) {
    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(12.dp)) {
        Text(label, Modifier.weight(1f), color = MaterialTheme.colorScheme.onSurfaceVariant)
        Text(if (value == null) "—" else String.format(Locale.US, "%.${decimals}f", value) + if (unit.isBlank()) "" else " $unit", fontWeight = FontWeight.Medium)
    }
}
@Composable private fun SpeedCard(vehicle: JSONObject?, runtime: JSONObject?) {
    SurfaceCard("Vehicle speed") {
        Row(verticalAlignment = Alignment.Bottom, horizontalArrangement = Arrangement.spacedBy(12.dp)) {
            Text(vehicle.finite("speed_kmh")?.let { "%.1f".format(Locale.US, it) } ?: "—",
                fontSize = 64.sp, fontWeight = FontWeight.Light, color = MaterialTheme.colorScheme.primary)
            Text("km/h", Modifier.padding(bottom = 12.dp), color = MaterialTheme.colorScheme.onSurfaceVariant)
        }
        Hint(if (vehicle != null) "Source: ${vehicle.optString("speed_source", "Unavailable")}" else if (runtime == null) "Waiting for recorder telemetry" else "Waiting for a fresh vehicle source")
        Metric("Acceleration", vehicle.finite("acceleration_ms2"), "m/s²", 2)
    }
}
@Composable private fun CameraCard(video: VideoState) { SurfaceCard("Camera") {
    val frame = video.frame?.takeUnless { video.stale }
    Box(Modifier.align(Alignment.CenterHorizontally).widthIn(max = 720.dp).fillMaxWidth().aspectRatio(16f / 9f)
        .clip(RoundedCornerShape(14.dp)).background(Color.Black), contentAlignment = Alignment.Center) {
        if (frame != null) Image(bitmap = frame.image, contentDescription = "Dashcam camera",
            modifier = Modifier.fillMaxSize(), contentScale = ContentScale.Fit)
        else Text(if (video.frame != null) "Video stale" else "No video",
            color = Color(0xFFFFCA80), fontWeight = FontWeight.SemiBold)
    }
    val lastFrame = video.frame
    Hint(if (frame == null) {
        // Stale on an open socket: say so rather than repeating "Live".
        if (lastFrame != null) "No camera frames for ${(video.nowMs - lastFrame.receivedAtMs) / 1000} s" else video.status
    } else buildString {
        append("${frame.width}×${frame.height}")
        frame.fps?.let { append(String.format(Locale.US, " · %.1f fps", it)) }
        append(" · ${frame.ageMs} ms old")
    })
} }
@Composable private fun PositionCard(vehicle: JSONObject?) { SurfaceCard("GNSS position") {
    Metric("Latitude", vehicle.finite("latitude"), "°", 6)
    Metric("Longitude", vehicle.finite("longitude"), "°", 6)
    Metric("Heading", vehicle.finite("heading_deg"), "°", 1)
    Metric("Altitude", vehicle.finite("altitude_m"), "m", 1)
    Metric("Satellites", vehicle.finite("satellites"), "", 0)
} }
@Composable private fun HealthSummary(system: JSONObject?) { SurfaceCard("Jetson health") {
    Metric("CPU utilization", system.finite("cpu_percent"), "%", 1)
    Metric("GPU utilization", system.finite("gpu_percent"), "%", 1)
    Metric("Uptime", system.finite("uptime_s")?.div(3600), "h", 1)
    Hint("Host measurements · unsupported counters display as —")
} }
@Composable private fun AdasCard() { SurfaceCard("ADAS") {
    BigValue("Not available")
    Hint("Lane detection and driver monitoring are unavailable on this Jetson.")
} }

@Composable private fun ConnectionPanel(model: DashboardModel, onPermission: () -> Unit) {
    val profiles by model.profiles.collectAsStateWithLifecycle()
    val selected by model.selected.collectAsStateWithLifecycle()
    val discovered by model.discovery.devices.collectAsStateWithLifecycle()
    val discoveryStatus by model.discovery.status.collectAsStateWithLifecycle()
    val error by model.error.collectAsStateWithLifecycle()
    var link by remember { mutableStateOf("") }
    Column(verticalArrangement = Arrangement.spacedBy(16.dp)) {
        SurfaceCard("Paired Jetsons") {
            if (profiles.isEmpty()) Hint("No paired devices yet")
            profiles.forEach { profile ->
                Text(profile.name, fontWeight = FontWeight.SemiBold)
                Hint("${profile.host}:${profile.port}")
                Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    OutlinedButton(onClick = { model.select(profile.id) }) { Text(if (selected == profile.id) "Reconnect" else "Connect") }
                    TextButton(onClick = { model.forget(profile.id) }) { Text("Forget") }
                }
            }
        }
        SurfaceCard("Pair a Jetson") {
            Hint("Run the connector’s pair command on your Jetson. Paste its private pairing link here; it contains the device credential and certificate fingerprint.")
            OutlinedTextField(value = link, onValueChange = { link = it.take(4096) },
                modifier = Modifier.fillMaxWidth(), label = { Text("Private pairing link") },
                singleLine = true, visualTransformation = PasswordVisualTransformation(),
                keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Password))
            Button(enabled = link.isNotBlank(), onClick = { if (model.pair(link)) link = "" }) { Text("Save pairing & connect") }
            if (error.isNotEmpty()) Text(error, color = MaterialTheme.colorScheme.error)
        }
        SurfaceCard("Find devices on Wi-Fi") {
            Hint("Discovery never replaces certificate verification. Router client isolation or blocked multicast may require manual pairing.")
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                OutlinedButton(onClick = { model.discovery.start() }) { Text("Find Jetsons") }
                TextButton(onClick = onPermission) { Text("Network access") }
            }
            if (discoveryStatus.isNotBlank()) Hint(discoveryStatus)
            discovered.forEach { device ->
                val paired = profiles.find { it.id == device.id }
                Text(paired?.name ?: "Unpaired Jetson · ${device.id.take(8)}")
                Hint("${device.host}:${device.port}")
                if (paired != null) TextButton(onClick = { model.updateAddress(device) }) { Text("Use discovered address") }
                else Hint("Paste this Jetson’s pairing link to verify its identity.")
            }
        }
    }
}
