package com.dashcam.connector

import android.app.DownloadManager
import android.content.Context
import android.net.Uri
import android.os.Build
import android.os.Environment
import android.os.SystemClock
import android.view.Gravity
import android.widget.FrameLayout
import android.widget.VideoView
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyListScope
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.LifecycleEventObserver
import androidx.lifecycle.compose.LocalLifecycleOwner
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import org.json.JSONException
import org.json.JSONObject

// Recordings in a cloud session (connector/README.md "Cloud recordings"): the relay forwards the
// request to the vehicle, which lists its segments or makes part of one into an H.264 MP4 and uploads
// it to Cloud Storage; the relay answers with signed play and download links that last an hour.

/** Clip links are accepted from Cloud Storage only. */
internal const val CLIP_STORAGE = "https://storage.googleapis.com/"
private val CLIP_FILENAME = Regex("[A-Za-z0-9_.-]{1,120}\\.mp4")

internal data class ClipRequest(val startS: Int, val durationS: Int, val quality: String, val captions: Boolean)

internal sealed interface ClipState {
    data class Exporting(val status: String) : ClipState
    data class Ready(val url: String, val download: String, val bytes: Long, val captions: Boolean,
                     val filename: String, val expiresAt: Long, val tookS: Long) : ClipState
    data class Failed(val why: String) : ClipState
}

/** The cloud Recordings page's state, kept with the session (not the activity) so an export survives rotation. */
internal object CloudRecordings {
    private val mutable = MutableStateFlow(RecordingsState())
    val state = mutable.asStateFlow()
    private val clipsMutable = MutableStateFlow<Map<String, ClipState>>(emptyMap())
    /** By segment id: the latest clip asked for from that segment. */
    val clips = clipsMutable.asStateFlow()
    private var vehicle: String? = null

    /** A different vehicle (or none) starts from nothing; answers for the previous one are dropped. */
    fun reset(vehicleId: String?) {
        if (vehicleId == vehicle) return
        vehicle = vehicleId
        mutable.value = RecordingsState()
        clipsMutable.value = emptyMap()
    }

    fun refresh() {
        val v = vehicle ?: return
        if (mutable.value.loading) return
        mutable.update { it.copy(loading = true, status = "Asking the vehicle for its recordings") }
        val sent = CloudSession.request(JSONObject().put("type", "recordings")) { m ->
            if (vehicle != v) return@request
            mutable.value = try {
                if (m.optString("type") != "recordings") throw JSONException(m.optString("error"))
                val list = parseSegments(m.getJSONArray("segments"))
                RecordingsState(status = when (list.size) { 0 -> "No finished segments yet"; 1 -> "1 segment"; else -> "${list.size} segments" },
                    segments = list, loadedFor = v)
            } catch (e: JSONException) {
                mutable.value.copy(loading = false, loadedFor = v,
                    status = e.message?.takeIf { m.has("error") && it.isNotEmpty() } ?: "The vehicle did not answer · tap Refresh")
            }
        }
        if (!sent) mutable.update { it.copy(loading = false, status = "Not connected to the vehicle yet · tap Refresh in a moment") }
    }

    val exporting get() = clipsMutable.value.values.any { it is ClipState.Exporting }

    fun export(segment: RecordingSegment, request: ClipRequest) {
        val v = vehicle ?: return
        if (exporting) return
        fun set(state: ClipState) { if (vehicle == v) clipsMutable.update { it + (segment.id to state) } }
        val began = SystemClock.elapsedRealtime()
        val body = JSONObject().put("type", "clip").put("segment", segment.id).put("start_s", request.startS)
            .put("duration_s", request.durationS).put("quality", request.quality).put("captions", request.captions)
        set(ClipState.Exporting("Asking the vehicle"))
        val sent = CloudSession.request(body) { m ->
            when (m.optString("type")) {
                "clip_status" -> set(ClipState.Exporting("Making the clip on the Jetson and uploading it"))
                "clip" -> {
                    val url = m.optString("url"); val download = m.optString("download")
                    val name = m.optString("filename").takeIf { CLIP_FILENAME.matches(it) } ?: "${segment.id}_clip.mp4"
                    set(if (!url.startsWith(CLIP_STORAGE) || !download.startsWith(CLIP_STORAGE)) ClipState.Failed("The clip link was not valid")
                        else ClipState.Ready(url, download, m.optLong("bytes"), m.optBoolean("captions"), name,
                            SystemClock.elapsedRealtime() + m.optLong("expires_s", 3600) * 1000,
                            (SystemClock.elapsedRealtime() - began) / 1000))
                }
                else -> set(ClipState.Failed(m.optString("error").ifEmpty { "The clip failed" }))
            }
        }
        if (!sent) set(ClipState.Failed("Not connected to the vehicle · try again in a moment"))
    }

    /** Hands the signed download link to the system's download manager (it shows progress in notifications). */
    fun download(context: Context, clip: ClipState.Ready): String {
        val request = DownloadManager.Request(Uri.parse(clip.download)).setMimeType("video/mp4").setTitle(clip.filename)
            .setNotificationVisibility(DownloadManager.Request.VISIBILITY_VISIBLE_NOTIFY_COMPLETED)
        // Like LAN downloads: Download/Dashcam on Android 10+, the app's own folder on 9 (no storage permission).
        val where = if (Build.VERSION.SDK_INT >= 29) {
            request.setDestinationInExternalPublicDir(Environment.DIRECTORY_DOWNLOADS, "Dashcam/" + clip.filename)
            "Download/Dashcam"
        } else {
            request.setDestinationInExternalFilesDir(context, Environment.DIRECTORY_DOWNLOADS, "Dashcam/" + clip.filename)
            "the app's Download folder"
        }
        context.getSystemService(DownloadManager::class.java).enqueue(request)
        return "Downloading to $where · see notifications"
    }
}

/** The Recordings page in a cloud session: segments by day, each with a clip export form. */
internal fun LazyListScope.cloudRecordingsPage(onPlay: (ClipState.Ready) -> Unit) {
    item(key = "cloud-recordings-head") {
        val state by CloudRecordings.state.collectAsStateWithLifecycle()
        val session by CloudSession.state.collectAsStateWithLifecycle()
        // Asked for once the vehicle answers (the first snapshot), not before: the relay would only say it is offline.
        LaunchedEffect(session.active) { if (session.active && state.loadedFor == null) CloudRecordings.refresh() }
        SurfaceCard("Recordings") {
            Hint(state.status.ifEmpty { "Waiting for the vehicle" })
            Hint("In a cloud session a recording is shared as a clip: part of one 3-minute segment, made into an MP4 " +
                "on the Jetson and kept in Cloud Storage for a day. The links last an hour. Small clips (960×540, 10 fps) " +
                "are about 1 Mbit/s; 720p (30 fps) about 1.5 Mbit/s.")
            OutlinedButton(onClick = { CloudRecordings.refresh() }, enabled = !state.loading) { Text("Refresh") }
        }
    }
    item(key = "cloud-recordings-days") {
        val state by CloudRecordings.state.collectAsStateWithLifecycle()
        val clips by CloudRecordings.clips.collectAsStateWithLifecycle()
        val days = remember(state.segments) { state.segments.groupBy { it.day } }
        var open by remember { mutableStateOf<String?>(null) }
        val shown = open ?: days.keys.firstOrNull()
        Column(verticalArrangement = Arrangement.spacedBy(12.dp)) {
            for ((day, segments) in days) {
                SurfaceCard("$day · ${segments.size} " + if (segments.size == 1) "segment" else "segments") {
                    if (day != shown) TextButton(onClick = { open = day }) { Text("Show") }
                    else segments.forEach { segment -> CloudSegmentRow(segment, clips[segment.id], onPlay) }
                }
            }
        }
    }
}

private val CLIP_LENGTHS = listOf(10, 30, 60, 180)
private val CLIP_QUALITIES = listOf("small" to "Small · 960×540 · 10 fps", "720p" to "720p · 30 fps")

@OptIn(ExperimentalLayoutApi::class)
@Composable private fun CloudSegmentRow(segment: RecordingSegment, clip: ClipState?, onPlay: (ClipState.Ready) -> Unit) {
    val context = LocalContext.current
    val length = segment.durationS ?: 180.0
    var expanded by rememberSaveable(segment.id) { mutableStateOf(false) }
    var start by rememberSaveable(segment.id) { mutableStateOf("0") }
    var span by rememberSaveable(segment.id) { mutableIntStateOf(30) }
    var quality by rememberSaveable(segment.id) { mutableStateOf("small") }
    var captions by rememberSaveable(segment.id) { mutableStateOf(segment.captionBytes != null) }
    var downloadNote by remember(clip) { mutableStateOf<String?>(null) }
    Column(Modifier.fillMaxWidth().padding(vertical = 4.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
        Text(segment.time + " · " + formatClock((length * 1000).toLong()), fontWeight = FontWeight.SemiBold)
        Hint(formatBytes(segment.bytes) + if (segment.captionBytes != null) " · captions" else " · no captions")
        if (!expanded && clip == null) Button(onClick = { expanded = true }) { Text("Export clip") }
        if (expanded || clip != null) {
            OutlinedTextField(start, { v -> start = v.filter(Char::isDigit).take(4) }, singleLine = true,
                label = { Text("Start (s into the segment)") }, modifier = Modifier.width(220.dp))
            Hint("Length")
            FlowRow(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                CLIP_LENGTHS.forEach { v ->
                    FilterChip(span == v, { span = v }, { Text(if (v < 60) "$v s" else "${v / 60} min") })
                }
            }
            Hint("Quality")
            FlowRow(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                CLIP_QUALITIES.forEach { (v, label) -> FilterChip(quality == v, { quality = v }, { Text(label) }) }
            }
            Row(verticalAlignment = Alignment.CenterVertically) {
                Checkbox(captions, { captions = it }, enabled = segment.captionBytes != null)
                Text("Burn in captions")
            }
            val busy = clip is ClipState.Exporting
            Button(onClick = {
                val s = (start.toIntOrNull() ?: 0).coerceIn(0, (length - 1).toInt().coerceAtLeast(0))
                start = s.toString()
                CloudRecordings.export(segment, ClipRequest(s, span, quality, captions && segment.captionBytes != null))
            }, enabled = !busy && !CloudRecordings.exporting) { Text("Export") }
            when (clip) {
                is ClipState.Exporting -> Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    CircularProgressIndicator(Modifier.size(18.dp), strokeWidth = 2.dp)
                    Hint(clip.status + " · a 30 s clip takes about 10–25 s, 3 minutes up to about 2 minutes")
                }
                is ClipState.Failed -> Hint(clip.why)
                is ClipState.Ready -> {
                    val expired = SystemClock.elapsedRealtime() >= clip.expiresAt
                    Hint("Ready after ${clip.tookS} s · ${formatBytes(clip.bytes)}" + (if (clip.captions) " · captions burned in" else "") +
                        if (expired) " · links expired: export again" else " · links last an hour")
                    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        Button(onClick = { onPlay(clip) }, enabled = !expired) { Text("Play") }
                        OutlinedButton(onClick = { downloadNote = CloudRecordings.download(context, clip) }, enabled = !expired) {
                            Text("Download MP4")
                        }
                    }
                    downloadNote?.let { Hint(it) }
                }
                null -> {}
            }
        }
    }
}

/** A clip from its signed link, on the whole screen; the platform player streams the MP4 (faststart). */
@Composable
internal fun CloudClipPlayer(clip: ClipState.Ready, onDownload: () -> Unit, onClose: () -> Unit) {
    var video by remember { mutableStateOf<VideoView?>(null) }
    var playing by remember { mutableStateOf(false) }
    var position by remember { mutableLongStateOf(0L) }
    var duration by remember { mutableLongStateOf(0L) }
    var status by remember { mutableStateOf("Loading") }
    var dragging by remember { mutableStateOf<Float?>(null) }
    LaunchedEffect(video) {
        val v = video ?: return@LaunchedEffect
        while (true) {
            playing = v.isPlaying
            if (dragging == null && duration > 0) position = v.currentPosition.toLong()
            delay(250)
        }
    }
    val lifecycle = LocalLifecycleOwner.current.lifecycle
    DisposableEffect(lifecycle) {
        val observer = LifecycleEventObserver { _, event -> if (event == Lifecycle.Event.ON_STOP) video?.pause() }
        lifecycle.addObserver(observer)
        onDispose { lifecycle.removeObserver(observer) }
    }
    FullscreenCamera(onExit = onClose, controls = {
        FilledTonalButton(onClick = { video?.let { if (it.isPlaying) it.pause() else it.start() } }, enabled = duration > 0) {
            Text(if (playing) "Pause" else "Play")
        }
        OutlinedButton(onClick = onDownload) { Text("Download") }
    }, caption = {
        val shownAt = dragging?.toLong() ?: position
        Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(12.dp)) {
            Text(formatClock(shownAt), color = Color(0xFFE6EEF5), fontFamily = FontFamily.Monospace)
            Slider(value = shownAt.toFloat().coerceIn(0f, duration.coerceAtLeast(1).toFloat()),
                onValueChange = { dragging = it },
                onValueChangeFinished = { dragging?.let { video?.seekTo(it.toInt()); position = it.toLong() }; dragging = null },
                valueRange = 0f..duration.coerceAtLeast(1).toFloat(), enabled = duration > 0, modifier = Modifier.weight(1f))
            Text(formatClock(duration), color = Color(0xFFE6EEF5), fontFamily = FontFamily.Monospace)
        }
        Hint(clip.filename + " · " + status)
    }) {
        // VideoView keeps the picture's aspect ratio only when it may choose its own size: centred in a frame.
        AndroidView(factory = { context ->
            FrameLayout(context).apply {
                val view = VideoView(context)
                view.setOnPreparedListener { player -> duration = player.duration.toLong(); status = "Playing"; view.start() }
                view.setOnCompletionListener { status = "Finished" }
                view.setOnErrorListener { _, _, _ -> status = "Could not play the clip (the link may have expired)"; true }
                view.setVideoURI(Uri.parse(clip.url))
                addView(view, FrameLayout.LayoutParams(FrameLayout.LayoutParams.WRAP_CONTENT, FrameLayout.LayoutParams.WRAP_CONTENT, Gravity.CENTER))
                video = view
            }
        }, onRelease = { video?.stopPlayback() }, modifier = Modifier.fillMaxSize())
        if (duration == 0L) Text(status, Modifier.align(Alignment.Center), color = Color(0xFFFFCA80), fontWeight = FontWeight.SemiBold)
    }
}
