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
import androidx.compose.ui.graphics.graphicsLayer
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.semantics.stateDescription
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.text.input.PasswordVisualTransformation
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
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
        foreground = false; model.repository.setActive(false); model.discovery.stop(); model.tiles.pause()
        window.clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        super.onStop()
    }
}

private val pages = listOf("Live", "Vehicle", "System", "Connection")
private val marks = listOf("◉", "↗", "▥", "⌁")

@OptIn(ExperimentalFoundationApi::class)
@Composable
private fun Dashboard(model: DashboardModel, onKeepAwake: (Boolean) -> Unit, onPermission: () -> Unit) {
    // Handed to the cards unread: a snapshot (5 Hz) or clock tick (2 Hz) recomposes
    // only the cards whose values change, never the whole page.
    val state = model.repository.state.collectAsStateWithLifecycle()
    val profiles by model.profiles.collectAsStateWithLifecycle()
    val selected by model.selected.collectAsStateWithLifecycle()
    var page by remember { mutableIntStateOf(if (profiles.isEmpty()) 3 else 0) }
    var awake by remember { mutableStateOf(false) }
    val connected = state.derive { active }
    LaunchedEffect(awake, connected) { onKeepAwake(awake && connected) }
    // Camera frames stream only while Live is shown; foreground, device
    // selection and the live view toggle are gated by the repository.
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
                    when (page) {
                        0 -> {
                            item { CameraCard(model.repository) }
                            item { SpeedCard(state) }
                            item { AdaptivePair(wide, { RecordingCard(state) }, { AdasCard() }) }
                            item { AdaptivePair(wide, { PositionCard(state) }, { HealthSummary(state) }) }
                        }
                        1 -> {
                            item { SpeedCard(state) }
                            item { MapCard(model.map, model.tiles) }
                            item { AdaptivePair(wide, { PowertrainCard(state) }, { PositionCard(state) }) }
                            item { TelemetryLinkCard(state) }
                        }
                        2 -> {
                            item { AdaptivePair(wide, { HealthSummary(state) }, { MemoryCard(state) }) }
                            item { TemperaturesCard(state) }
                            item { AdasCard() }
                        }
                        3 -> {
                            item { ConnectionPanel(model, onPermission) }
                            item { SurfaceCard("Display") {
                                Row(verticalAlignment = Alignment.CenterVertically) {
                                    Text("Keep screen on while connected", Modifier.weight(1f))
                                    Switch(checked = awake, onCheckedChange = { awake = it })
                                }
                                Hint("Monitoring pauses when the app is in the background.")
                            } }
                        }
                    }
                    item { Spacer(Modifier.height(8.dp)) }
                }
            }
        }
    }
}

// derivedStateOf: the caller recomposes only when the computed value changes,
// not on every new snapshot or clock tick. calc must return a value with
// structural equality (strings, numbers, data classes, lists), never a JSONObject.
@Composable private fun <S, T> State<S>.derive(calc: S.() -> T): T =
    remember(this, calc) { derivedStateOf { value.calc() } }.value

// Freshness gating for the cards: recorder values only while the runtime is
// fresh, host health while the transport is.
private val MonitorState.freshRuntime get() = snapshot?.runtime?.takeIf { runtimeFresh }
private val MonitorState.freshVehicle get() = snapshot?.vehicle?.takeIf { runtimeFresh }
private val MonitorState.freshSystem get() = snapshot?.system?.takeIf { transportFresh }

// Holds the formatted text, not the raw Double: a row recomposes only when what it shows changes.
private data class MetricValue(val label: String, val text: String) {
    constructor(label: String, value: Double?, unit: String, decimals: Int) :
        this(label, if (value == null) "—" else String.format(Locale.US, "%.${decimals}f", value) + if (unit.isBlank()) "" else " $unit")
}
private data class StatusUi(val headline: String, val rows: List<MetricValue>)
private data class SpeedUi(val speed: String, val source: String, val acceleration: MetricValue)
private data class BannerUi(val fresh: Boolean, val title: String, val detail: String?)

@Composable private fun ConnectionBanner(state: State<MonitorState>) {
    val ui = state.derive {
        BannerUi(transportFresh,
            if (active && lastReceiveMs > 0 && !transportFresh) "Data stale · waiting for Jetson" else status,
            if (!transportFresh) null
            else if (runtimeFresh) "Updated ${((nowMs - lastReceiveMs).coerceAtLeast(0) / 1000.0).let { "%.1f".format(Locale.US, it) }} s ago"
            else "System health available; recorder data has expired")
    }
    val color = if (ui.fresh) MaterialTheme.colorScheme.primary else Color(0xFFFFCA80)
    Surface(color = color.copy(alpha = .1f), shape = RoundedCornerShape(16.dp)) {
        Column(Modifier.fillMaxWidth().padding(16.dp)) {
            Text("●  " + ui.title, color = color, fontWeight = FontWeight.SemiBold)
            ui.detail?.let { Text(it, style = MaterialTheme.typography.bodySmall) }
        }
    }
}
@Composable internal fun SurfaceCard(title: String, content: @Composable ColumnScope.() -> Unit) {
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
@Composable internal fun Hint(value: String) { Text(value, color = MaterialTheme.colorScheme.onSurfaceVariant, style = MaterialTheme.typography.bodySmall) }
@Composable private fun Metric(label: String, text: String) {
    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(12.dp)) {
        Text(label, Modifier.weight(1f), color = MaterialTheme.colorScheme.onSurfaceVariant)
        Text(text, fontWeight = FontWeight.Medium)
    }
}
// Each Metric skips unless its own row changed.
@Composable private fun Metrics(rows: List<MetricValue>) { rows.forEach { Metric(it.label, it.text) } }
@Composable private fun SpeedCard(state: State<MonitorState>) {
    val ui = state.derive {
        val vehicle = freshVehicle
        SpeedUi(vehicle.finite("speed_kmh")?.let { "%.1f".format(Locale.US, it) } ?: "—",
            if (vehicle != null) "Source: ${vehicle.optString("speed_source", "Unavailable")}" else if (freshRuntime == null) "Waiting for recorder telemetry" else "Waiting for a fresh vehicle source",
            MetricValue("Acceleration", vehicle.finite("acceleration_ms2"), "m/s²", 2))
    }
    SurfaceCard("Vehicle speed") {
        Row(verticalAlignment = Alignment.Bottom, horizontalArrangement = Arrangement.spacedBy(12.dp)) {
            Text(ui.speed, fontSize = 64.sp, fontWeight = FontWeight.Light, color = MaterialTheme.colorScheme.primary)
            Text("km/h", Modifier.padding(bottom = 12.dp), color = MaterialTheme.colorScheme.onSurfaceVariant)
        }
        Hint(ui.source)
        Metric(ui.acceleration.label, ui.acceleration.text)
    }
}
@Composable private fun RecordingCard(state: State<MonitorState>) {
    val ui = state.derive {
        val runtime = freshRuntime
        StatusUi(when { runtime == null -> "Unavailable"; runtime.optBoolean("recording") -> "Recording"; else -> "Waiting" },
            listOf(MetricValue("Actual frame rate", runtime.finite("fps"), "fps", 1),
                MetricValue("Frames in session", runtime.finite("frames"), "", 0)))
    }
    SurfaceCard("Recording") {
        BigValue(ui.headline)
        Metrics(ui.rows)
    }
}
@Composable private fun TelemetryLinkCard(state: State<MonitorState>) {
    val ui = state.derive {
        val runtime = freshRuntime
        StatusUi(when { runtime == null -> "Runtime unavailable"; runtime.optBoolean("bridge_fresh") -> "Fresh"; else -> "No fresh bridge data" },
            listOf(MetricValue("Bridge sample age", runtime.finite("bridge_age_ms"), "ms", 0)))
    }
    SurfaceCard("Telemetry link") {
        BigValue(ui.headline)
        Metrics(ui.rows)
        Hint("Missing or stale sensor readings display as —.")
    }
}
@Composable private fun PowertrainCard(state: State<MonitorState>) {
    val rows = state.derive {
        val vehicle = freshVehicle
        listOf(MetricValue("Engine speed", vehicle.finite("rpm"), "rpm", 0),
            MetricValue("Coolant", vehicle.finite("coolant_c"), "°C", 1),
            MetricValue("Fuel level", vehicle.finite("fuel_percent"), "%", 1),
            MetricValue("Throttle", vehicle.finite("throttle_percent"), "%", 1),
            MetricValue("Engine load", vehicle.finite("engine_load_percent"), "%", 1))
    }
    SurfaceCard("Powertrain") { Metrics(rows) }
}
@Composable private fun MemoryCard(state: State<MonitorState>) {
    val rows = state.derive {
        val system = freshSystem
        listOf(MetricValue("RAM used", system.finite("ram_used_bytes")?.div(1073741824), "GiB", 2),
            MetricValue("RAM total", system.finite("ram_total_bytes")?.div(1073741824), "GiB", 2),
            MetricValue("Swap used", system.finite("swap_used_bytes")?.div(1073741824), "GiB", 2),
            MetricValue("Footage disk free", system.finite("storage_free_bytes")?.div(1073741824), "GiB", 1),
            MetricValue("Footage disk total", system.finite("storage_total_bytes")?.div(1073741824), "GiB", 1))
    }
    SurfaceCard("Memory & storage") { Metrics(rows) }
}
@Composable private fun TemperaturesCard(state: State<MonitorState>) {
    val rows = state.derive {
        val temperatures = freshSystem?.optJSONObject("temperatures_c")
        if (temperatures == null || temperatures.length() == 0) emptyList<MetricValue>()
        else temperatures.keys().asSequence().sorted().take(64).map { MetricValue(it, temperatures.finite(it), "°C", 1) }.toList()
    }
    SurfaceCard("Temperatures") {
        if (rows.isEmpty()) Hint("Temperature sensors unavailable") else Metrics(rows)
    }
}
@Composable private fun CameraCard(repository: MonitorRepository) {
    // Collected here, not in Dashboard: frames (~10 Hz) and the countdown reach only this card.
    val video = repository.video.collectAsStateWithLifecycle()
    val live = repository.liveView.collectAsStateWithLifecycle()
    SurfaceCard("Camera") {
        CameraPicture(video, live, Modifier.align(Alignment.CenterHorizontally))
        CameraHint(video, live)
        LiveViewButton(live) { repository.setLiveView(it) }
    }
}
private fun shownFrame(video: VideoState, live: LiveViewState) = video.frame?.takeUnless { video.stale }?.takeIf { live.on }
@Composable private fun CameraPicture(video: State<VideoState>, live: State<LiveViewState>, modifier: Modifier) {
    val frame by remember(video, live) { derivedStateOf { shownFrame(video.value, live.value) } }
    val placeholder by remember(video, live) { derivedStateOf {
        val v = video.value
        val l = live.value
        when { !l.available -> "No video"; !l.on -> "Live view off"; v.frame != null -> "Video stale"; else -> "No video" }
    } }
    Box(modifier.widthIn(max = 720.dp).fillMaxWidth().aspectRatio(16f / 9f)
        .clip(RoundedCornerShape(14.dp)).background(Color.Black), contentAlignment = Alignment.Center) {
        val shown = frame
        // Image already paints in its own clipToBounds layer; this one makes that isolation explicit
        // (a new frame re-records only the picture) instead of relying on Image internals.
        if (shown != null) Image(bitmap = shown.image, contentDescription = "Dashcam camera",
            modifier = Modifier.fillMaxSize().graphicsLayer {}, contentScale = ContentScale.Fit)
        else Text(placeholder, color = Color(0xFFFFCA80), fontWeight = FontWeight.SemiBold)
    }
}
@Composable private fun CameraHint(video: State<VideoState>, live: State<LiveViewState>) {
    val text by remember(video, live) { derivedStateOf { cameraHint(video.value, live.value) } }
    Hint(text)
}
private fun cameraHint(video: VideoState, live: LiveViewState): String {
    val frame = shownFrame(video, live)
    val lastFrame = video.frame
    return when {
        !live.available -> video.status   // no Jetson selected, no video, or credential/certificate rejected
        !live.on -> if (live.stoppedByTimer) "Live view stopped after ${LIVE_VIEW_MAX_MS / 60_000} min · tap Start live view to resume"
            else "Live view off · tap Start live view to watch"
        // Stale on an open socket: say so rather than repeating "Live".
        frame == null -> (if (lastFrame != null) "No camera frames for ${(video.nowMs - lastFrame.receivedAtMs) / 1000} s" else video.status) +
            " · ${liveViewRemaining(live.remainingMs)}"
        else -> buildString {
            append("${frame.width}×${frame.height}")
            frame.fps?.let { append(String.format(Locale.US, " · %.1f fps", it)) }
            append(" · ${frame.ageMs} ms old")
            append(" · ${liveViewRemaining(live.remainingMs)}")
        }
    }
}
@Composable private fun LiveViewButton(live: State<LiveViewState>, onLiveView: (Boolean) -> Unit) {
    // The countdown ticks every second; the button changes only with on/available.
    val isOn = live.derive { on }
    val canStart = live.derive { available }
    OutlinedButton(onClick = { onLiveView(!isOn) }, enabled = canStart,
        modifier = Modifier.semantics { stateDescription = if (isOn) "Live view on" else "Live view off" }) {
        Text(if (isOn) "Stop live view" else "Start live view")
    }
}
@Composable private fun PositionCard(state: State<MonitorState>) {
    val rows = state.derive {
        val vehicle = freshVehicle
        listOf(MetricValue("Latitude", vehicle.finite("latitude"), "°", 6),
            MetricValue("Longitude", vehicle.finite("longitude"), "°", 6),
            MetricValue("Heading", vehicle.finite("heading_deg"), "°", 1),
            MetricValue("Altitude", vehicle.finite("altitude_m"), "m", 1),
            MetricValue("Satellites", vehicle.finite("satellites"), "", 0))
    }
    SurfaceCard("GNSS position") { Metrics(rows) }
}
@Composable private fun HealthSummary(state: State<MonitorState>) {
    val rows = state.derive {
        val system = freshSystem
        listOf(MetricValue("CPU utilization", system.finite("cpu_percent"), "%", 1),
            MetricValue("GPU utilization", system.finite("gpu_percent"), "%", 1),
            MetricValue("Uptime", system.finite("uptime_s")?.div(3600), "h", 1))
    }
    SurfaceCard("Jetson health") {
        Metrics(rows)
        Hint("Host measurements · unsupported counters display as —")
    }
}
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
