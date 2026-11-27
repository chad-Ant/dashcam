package com.dashcam.connector

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyListScope
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.LifecycleEventObserver
import androidx.lifecycle.compose.LocalLifecycleOwner
import androidx.lifecycle.compose.collectAsStateWithLifecycle

/** The Recordings page on a direct LAN connection: segments by day, newest first. */
internal fun LazyListScope.recordingsPage(browser: RecordingsBrowser, onPlay: (RecordingSegment) -> Unit) {
    item(key = "recordings-head") {
        val state by browser.state.collectAsStateWithLifecycle()
        LaunchedEffect(Unit) { if (state.loadedFor == null && !state.loading) browser.refresh() }
        SurfaceCard("Recordings") {
            Hint(if (state.loading) "Loading recordings" else state.status)
            Hint("3-minute segments as recorded (1080p MJPEG with telemetry captions). Playback and downloads " +
                "use the original files: about 15–75 Mbit/s of Wi-Fi while playing, 0.4–1.6 GB per download.")
            OutlinedButton(onClick = { browser.refresh() }, enabled = !state.loading) { Text("Refresh") }
        }
    }
    item(key = "recordings-days") {
        val state by browser.state.collectAsStateWithLifecycle()
        val downloads by browser.downloads.collectAsStateWithLifecycle()
        val days = remember(state.segments) { state.segments.groupBy { it.day } }
        var open by remember { mutableStateOf<String?>(null) }
        val shown = open ?: days.keys.firstOrNull()
        Column(verticalArrangement = Arrangement.spacedBy(12.dp)) {
            for ((day, segments) in days) {
                SurfaceCard("$day · ${segments.size} " + if (segments.size == 1) "segment" else "segments") {
                    if (day != shown) TextButton(onClick = { open = day }) { Text("Show") }
                    else segments.forEach { segment ->
                        SegmentRow(segment, downloads[segment.id], onPlay = { onPlay(segment) },
                            onDownload = { browser.download(segment) }, onCancel = { browser.cancel(segment.id) })
                    }
                }
            }
        }
    }
}

@Composable private fun SegmentRow(segment: RecordingSegment, download: DownloadState?, onPlay: () -> Unit,
                                   onDownload: () -> Unit, onCancel: () -> Unit) {
    Column(Modifier.fillMaxWidth().padding(vertical = 4.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
        Text(segment.time + (segment.durationS?.let { " · " + formatClock((it * 1000).toLong()) } ?: ""),
            fontWeight = FontWeight.SemiBold)
        Hint(formatBytes(segment.bytes) + (if (segment.captionBytes != null) " · captions" else " · no captions") +
            if (segment.playable) "" else " · cut short: download only")
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp), verticalAlignment = Alignment.CenterVertically) {
            Button(onClick = onPlay, enabled = segment.playable) { Text("Play") }
            when (download) {
                is DownloadState.Running -> {
                    val pct = if (download.total > 0) download.done * 100 / download.total else 0
                    Hint("${download.file.substringAfterLast('.')} $pct %")
                    TextButton(onClick = onCancel) { Text("Cancel") }
                }
                is DownloadState.Saved -> Hint("Saved to ${download.where}")
                is DownloadState.Failed -> { Hint(download.why); TextButton(onClick = onDownload) { Text("Retry") } }
                null -> OutlinedButton(onClick = onDownload) { Text("Download") }
            }
        }
    }
}

/** Fullscreen player for one segment; [onClose] also stops it. */
@Composable
internal fun RecordingPlayerDialog(player: RecordingPlayer, onDownload: () -> Unit, onClose: () -> Unit) {
    val state by player.state.collectAsStateWithLifecycle()
    val captions by player.captions.collectAsStateWithLifecycle()
    var showCaptions by remember { mutableStateOf(true) }
    var dragging by remember { mutableStateOf<Float?>(null) }
    // Backgrounding the app pauses playback (and its downloads of clusters).
    val lifecycle = LocalLifecycleOwner.current.lifecycle
    DisposableEffect(lifecycle, player) {
        val observer = LifecycleEventObserver { _, event -> if (event == Lifecycle.Event.ON_STOP) player.pause() }
        lifecycle.addObserver(observer)
        onDispose { lifecycle.removeObserver(observer) }
    }
    FullscreenCamera(onExit = onClose, controls = {
        FilledTonalButton(onClick = { if (state.playing) player.pause() else player.play() }, enabled = !state.failed) {
            Text(if (state.playing) "Pause" else "Play")
        }
        if (state.captionsAvailable) OutlinedButton(onClick = { showCaptions = !showCaptions }) {
            Text(if (showCaptions) "Captions on" else "Captions off")
        }
        OutlinedButton(onClick = onDownload) { Text("Download") }
    }, caption = {
        val position = dragging?.toLong() ?: state.positionMs
        Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(12.dp)) {
            Text(formatClock(position), color = Color(0xFFE6EEF5), fontFamily = FontFamily.Monospace)
            Slider(value = position.toFloat().coerceIn(0f, state.durationMs.coerceAtLeast(1).toFloat()),
                onValueChange = { dragging = it }, onValueChangeFinished = { dragging?.let { player.seek(it.toLong()) }; dragging = null },
                valueRange = 0f..state.durationMs.coerceAtLeast(1).toFloat(), enabled = state.durationMs > 0,
                modifier = Modifier.weight(1f))
            Text(formatClock(state.durationMs), color = Color(0xFFE6EEF5), fontFamily = FontFamily.Monospace)
        }
        Hint(player.segment.start + " · " + when {
            state.failed -> state.status
            state.buffering && state.playing -> "Buffering"
            else -> state.status
        })
    }) {
        CameraSurface(player.picture, Color.Black, 0.dp, Modifier.fillMaxSize())
        val script = player.script
        if (showCaptions && script != null && state.width > 0) CaptionOverlay(script, captions, state.width, state.height)
        if (state.failed) Text(state.status, Modifier.align(Alignment.Center), color = Color(0xFFFFCA80), fontWeight = FontWeight.SemiBold)
    }
}

/** The recorder's captions where its styles put them, scaled with the letterboxed picture. */
@Composable
private fun CaptionOverlay(script: AssScript, captions: List<AssCaption>, videoWidth: Int, videoHeight: Int) {
    BoxWithConstraints(Modifier.fillMaxSize()) {
        val density = LocalDensity.current
        val boxW = with(density) { maxWidth.toPx() }.toInt()
        val boxH = with(density) { maxHeight.toPx() }.toInt()
        val fit = letterbox(videoWidth, videoHeight, boxW, boxH) ?: return@BoxWithConstraints
        val scale = (fit.bottom - fit.top) / script.playResY
        for (caption in captions) {
            val s = caption.style
            val align = when (s.alignment) {
                7 -> Alignment.TopStart; 8 -> Alignment.TopCenter; 9 -> Alignment.TopEnd
                4 -> Alignment.CenterStart; 5 -> Alignment.Center; 6 -> Alignment.CenterEnd
                1 -> Alignment.BottomStart; 3 -> Alignment.BottomEnd; else -> Alignment.BottomCenter
            }
            with(density) {
                Box(Modifier.fillMaxSize().padding(
                    start = (fit.left + s.marginL * scale).toDp(), end = (boxW - fit.right + s.marginR * scale).toDp(),
                    top = (fit.top + s.marginV * scale).toDp(), bottom = (boxH - fit.bottom + s.marginV * scale).toDp()),
                    contentAlignment = align) {
                    // The recorder draws an opaque box (BorderStyle 3); never smaller than readable.
                    Text(caption.text, color = Color.White, fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold,
                        fontSize = (s.fontSize * scale).toSp().value.coerceAtLeast(9f).sp, lineHeight = (s.fontSize * scale * 1.15f).toSp(),
                        modifier = Modifier.background(Color.Black.copy(alpha = .78f), RoundedCornerShape(2.dp))
                            .padding(horizontal = 4.dp, vertical = 1.dp))
                }
            }
        }
    }
}
