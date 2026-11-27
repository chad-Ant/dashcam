package com.dashcam.connector

import android.content.Context
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.os.Build
import android.os.Environment
import android.os.SystemClock
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import okhttp3.HttpUrl
import okhttp3.OkHttpClient
import okhttp3.Request
import org.json.JSONArray
import org.json.JSONObject
import java.io.Closeable
import java.io.File
import java.io.IOException
import java.io.OutputStream
import java.util.Locale

// Recorded segments over the direct LAN (connector/README.md "Recordings"): the connector lists them
// and serves the .mkv and .ass files by byte range; this app reads the Matroska itself and decodes
// each JPEG frame, so the Jetson only moves bytes.

/** [playable] is false when the segment has no usable seek index (cut short by a power loss): it can still be
 *  downloaded for recovery. Connectors before 2026-11-27 do not send it; their segments count as playable. */
data class RecordingSegment(val id: String, val seq: Int, val start: String, val durationS: Double?,
                            val bytes: Long, val captionBytes: Long?, val playable: Boolean = true) {
    val day get() = start.take(10)
    val time get() = start.drop(11)
}

/** The connector's segment list (also what the cloud relay forwards from it). */
internal fun parseSegments(list: JSONArray): List<RecordingSegment> = (0 until list.length()).map { i ->
    val j = list.getJSONObject(i)
    RecordingSegment(j.getString("id"), j.getInt("seq"), j.getString("start"),
        if (j.isNull("duration_s")) null else j.getDouble("duration_s"), j.getLong("bytes"),
        if (j.isNull("caption_bytes")) null else j.getLong("caption_bytes"),
        j.opt("playable") != false)
}

/** Why recordings are unavailable, in words fit for the screen. */
class RecordingsException(message: String) : IOException(message)

/** Pinned HTTPS access to one Jetson's recordings; one client reused (keep-alive) for every range. */
class RecordingSource(private val client: OkHttpClient, private val base: HttpUrl, private val token: String,
                      val deviceId: String) : Closeable {
    private fun request(name: String?, range: String?): Request {
        val url = if (name == null) base else base.newBuilder().addPathSegment(name).build()
        return Request.Builder().url(url).header("Authorization", "Bearer $token")
            .apply { range?.let { header("Range", it) } }.build()
    }

    private fun failure(code: Int): RecordingsException = RecordingsException(when (code) {
        401 -> "Pairing rejected. Check device credential."
        404 -> "Not found: the segment was deleted by retention, or this connector has no recordings"
        409 -> "This segment is still recording"
        else -> "Jetson answered HTTP $code"
    })

    /** Blocking: call on Dispatchers.IO. */
    fun list(): List<RecordingSegment> {
        client.newCall(request(null, null)).execute().use { r ->
            if (r.code == 404) throw RecordingsException("This Jetson's connector does not offer recordings (update it)")
            if (!r.isSuccessful) throw failure(r.code)
            return parseSegments(JSONObject(r.body!!.string()).getJSONArray("segments"))
        }
    }

    /** Blocking: bytes [from, toInclusive] of a file, or from [from] to the end when [toInclusive] is null. */
    fun range(name: String, from: Long, toInclusive: Long?): ByteArray {
        val spec = "bytes=$from-" + (toInclusive?.toString() ?: "")
        client.newCall(request(name, spec)).execute().use { r ->
            if (r.code != 206 && !(r.code == 200 && from == 0L)) throw failure(r.code)
            return r.body!!.bytes()
        }
    }

    /**
     * Blocking: the whole file into [sink], reporting (done, total); stops when [cancelled] says so. Throws
     * unless exactly the announced length (and [expected], when known: the size the list gave) arrived, so a
     * cut-short transfer is never published as a finished file.
     */
    fun download(name: String, sink: OutputStream, expected: Long?, cancelled: () -> Boolean,
                 progress: (Long, Long) -> Unit) {
        client.newCall(request(name, null)).execute().use { r ->
            if (!r.isSuccessful) throw failure(r.code)
            val body = r.body!!
            val total = body.contentLength()
            val buffer = ByteArray(256 * 1024)
            var done = 0L
            body.byteStream().use { input ->
                while (true) {
                    if (cancelled()) throw CancellationException("Download cancelled")
                    val n = input.read(buffer)
                    if (n < 0) break
                    sink.write(buffer, 0, n)
                    done += n
                    progress(done, total)
                }
            }
            checkComplete(done, total, expected)
        }
    }

    // Closing an idle TLS connection writes close_notify: network I/O, which Android forbids on the main
    // thread (NetworkOnMainThreadException). Callers close from the main thread (a refresh's finally block,
    // the player's dispose), so the eviction always runs on a thread of its own.
    override fun close() {
        Thread({ client.connectionPool.evictAll(); client.dispatcher.executorService.shutdown() }, "recordings-close").start()
    }
}

data class RecordingsState(val status: String = "", val loading: Boolean = false,
                           val segments: List<RecordingSegment> = emptyList(), val loadedFor: String? = null)

sealed interface DownloadState {
    data class Running(val file: String, val done: Long, val total: Long) : DownloadState
    data class Saved(val where: String) : DownloadState
    data class Failed(val why: String) : DownloadState
}

/** The list of segments and the downloads, for the Recordings page. */
class RecordingsBrowser(private val context: Context, private val repository: MonitorRepository,
                        private val scope: CoroutineScope) {
    private val mutable = MutableStateFlow(RecordingsState())
    val state = mutable.asStateFlow()
    private val downloadsMutable = MutableStateFlow<Map<String, DownloadState>>(emptyMap())
    val downloads = downloadsMutable.asStateFlow()
    private val jobs = HashMap<String, Job>()
    private var refreshJob: Job? = null

    fun refresh() {
        refreshJob?.cancel()
        val source = repository.openRecordings()
        if (source == null) {
            mutable.value = RecordingsState(status = "Connect to a paired Jetson on Wi-Fi to see its recordings")
            return
        }
        mutable.update { it.copy(loading = true, status = "Loading recordings") }
        refreshJob = scope.launch {
            try {
                val list = withContext(Dispatchers.IO) { source.list() }
                mutable.value = RecordingsState(
                    status = when (list.size) { 0 -> "No finished segments yet"; 1 -> "1 segment"; else -> "${list.size} segments" },
                    segments = list, loadedFor = source.deviceId)
            } catch (e: CancellationException) { throw e
            } catch (e: RecordingsException) { mutable.value = RecordingsState(status = e.message ?: "Unavailable")
            } catch (_: Exception) { mutable.value = RecordingsState(status = "Could not reach the Jetson · tap Refresh")
            } finally { source.close() }
        }
    }

    fun cancel(id: String) { jobs.remove(id)?.cancel(); downloadsMutable.update { it - id } }

    /** Saves the segment and its captions side by side (same name: players pick the captions up). */
    fun download(segment: RecordingSegment) {
        if (jobs[segment.id]?.isActive == true) return
        val source = repository.openRecordings() ?: run {
            downloadsMutable.update { it + (segment.id to DownloadState.Failed("Not connected to the Jetson")) }
            return
        }
        jobs[segment.id] = scope.launch(Dispatchers.IO) {
            try {
                val sizes = linkedMapOf<String, Long?>("${segment.id}.mkv" to segment.bytes)
                segment.captionBytes?.let { sizes["${segment.id}.ass"] = it }
                // Both files are staged, checked and only then published together: an interrupted download
                // never replaces (or erases) the copy saved earlier. See DownloadStaging.kt.
                val place = saveStaged(target(), sizes.keys.toList()) { name, out ->
                    source.download(name, out, sizes[name], { !isActive }) { done, total ->
                        downloadsMutable.update { it + (segment.id to DownloadState.Running(name, done, total)) }
                    }
                }
                downloadsMutable.update { it + (segment.id to DownloadState.Saved(place)) }
            } catch (e: CancellationException) { throw e
            } catch (e: Exception) {
                downloadsMutable.update { it + (segment.id to DownloadState.Failed(
                    (e as? RecordingsException)?.message ?: "Download failed")) }
            } finally { source.close() }
        }
    }

    /** Download/Dashcam: MediaStore on Android 10+, the app's own folder on 9. */
    private fun target(): DownloadTarget =
        if (Build.VERSION.SDK_INT >= 29) MediaStoreTarget(context)
        else DirectoryTarget(File(context.getExternalFilesDir(Environment.DIRECTORY_DOWNLOADS), "Dashcam"))
}

/** A transfer that ended early, or did not match the size the list announced, is an error, never a file. */
internal fun checkComplete(done: Long, total: Long, expected: Long?) {
    if (total >= 0 && done != total) throw IOException("Download ended early ($done of $total bytes)")
    if (expected != null && done != expected) throw IOException("Download size $done does not match the listed $expected")
}

data class PlayerState(val status: String = "Loading", val positionMs: Long = 0, val durationMs: Long = 0,
                       val playing: Boolean = false, val buffering: Boolean = true, val failed: Boolean = false,
                       val width: Int = 0, val height: Int = 0, val captionsAvailable: Boolean = false)

/**
 * Plays one segment: the header and Cues first, then whole clusters by byte range, one cluster
 * fetched ahead. Frames are shown at their timestamps; when decoding falls behind, frames are
 * skipped rather than delayed. Pictures go to [picture] (a CameraSurface draws them off the main
 * thread); the captions for the frame on screen go to [captions].
 */
class RecordingPlayer(private val openSource: () -> RecordingSource?, val segment: RecordingSegment,
                      private val scope: CoroutineScope) {
    private val name = segment.id + ".mkv"
    private val stateMutable = MutableStateFlow(PlayerState())
    val state: StateFlow<PlayerState> = stateMutable.asStateFlow()
    private val pictureMutable = MutableStateFlow<Bitmap?>(null)
    val picture: StateFlow<Bitmap?> = pictureMutable.asStateFlow()
    private val captionsMutable = MutableStateFlow<List<AssCaption>>(emptyList())
    val captions: StateFlow<List<AssCaption>> = captionsMutable.asStateFlow()
    var script: AssScript? = null
        private set
    private var info: MkvInfo? = null
    private var clusters: List<MkvCluster> = emptyList()
    private var job: Job? = null
    private var closed = false

    fun start() {
        runPlayback { source ->
            withContext(Dispatchers.IO) {
                val header = source.range(name, 0, Mkv.HEADER_BYTES - 1L)
                val mkv = Mkv.parseHeader(header)
                clusters = Mkv.parseCues(source.range(name, mkv.cuesPosition, null), mkv)
                info = mkv
                if (segment.captionBytes != null) script = try {
                    AssScript.parse(String(source.range(segment.id + ".ass", 0, null), Charsets.UTF_8))
                } catch (e: CancellationException) { throw e } catch (_: Exception) { null }
                stateMutable.value = PlayerState("Playing", 0, mkv.durationMs, playing = true, buffering = true,
                    width = mkv.width, height = mkv.height, captionsAvailable = script != null)
            }
            withContext(Dispatchers.Default) { playback(source, 0, autoplay = true) }
        }
    }

    /** A terminal failure disables Play; a lost connection only pauses, and Play resumes (or reloads). */
    private fun stop(failure: PlaybackFailure) {
        stateMutable.update { it.copy(status = failure.message, failed = failure.terminal, playing = false, buffering = false) }
    }

    fun play() {
        val s = stateMutable.value
        if (s.failed || closed) return
        if (info == null) { if (job?.isActive != true) start(); return }   // the header never arrived: load again
        // From the last frame shown (positionMs follows it), through the Cues already in memory.
        playFrom(if (s.positionMs >= s.durationMs - 50) 0 else s.positionMs, autoplay = true)
    }

    fun pause() { job?.cancel(); stateMutable.update { it.copy(playing = false, buffering = false, status = "Paused") } }

    /** Jumps to [ms], keeping the playing/paused state; paused shows the frame there. */
    fun seek(ms: Long) {
        if (info == null) return
        playFrom(ms.coerceIn(0, stateMutable.value.durationMs), autoplay = stateMutable.value.playing)
    }

    fun close() { closed = true; job?.cancel(); pictureMutable.value = null }

    private fun playFrom(fromMs: Long, autoplay: Boolean) {
        job?.cancel()
        if (closed) return
        stateMutable.update { it.copy(positionMs = fromMs, playing = autoplay, buffering = true,
            status = if (autoplay) "Playing" else "Paused") }
        runPlayback { source -> withContext(Dispatchers.Default) { playback(source, fromMs, autoplay) } }
    }

    private fun runPlayback(action: suspend (RecordingSource) -> Unit) {
        job = scope.launch {
            try {
                // Android assigns a new Network handle when Wi-Fi reconnects. Keep the pin and device,
                // but open a source on the current Wi-Fi for every Play or seek action.
                val source = openSource() ?: throw IOException("Wi-Fi unavailable")
                source.use { action(it) }
            }
            catch (e: CancellationException) { throw e }
            catch (e: Exception) { stop(playbackFailure(e, started = info != null)) }
        }
    }

    private suspend fun playback(source: RecordingSource, fromMs: Long, autoplay: Boolean) = coroutineScope {
        val mkv = info ?: return@coroutineScope
        fun fetch(i: Int) = async(Dispatchers.IO) {
            val end = if (i + 1 < clusters.size) clusters[i + 1].offset else mkv.cuesPosition
            Mkv.parseCluster(source.range(name, clusters[i].offset, end - 1), mkv)
        }
        var index = Mkv.clusterAt(clusters, fromMs)
        var next: Deferred<List<MkvFrame>>? = fetch(index)
        var clockWall = -1L
        var clockPts = 0L
        val frameMs = mkv.frameDurationMs.toLong().coerceAtLeast(1)
        while (next != null) {
            val waitStart = SystemClock.elapsedRealtime()
            if (!next.isCompleted) stateMutable.update { it.copy(buffering = true) }
            val frames = next.await()
            if (SystemClock.elapsedRealtime() - waitStart > 50) clockWall = -1   // restart the clock after a stall
            stateMutable.update { it.copy(buffering = false) }
            next = if (index + 1 < clusters.size) fetch(index + 1) else null
            for ((k, frame) in frames.withIndex()) {
                if (frame.ptsMs + frameMs / 2 < fromMs) continue
                if (!autoplay) { show(frame, decode(frame)); return@coroutineScope }
                val now = SystemClock.elapsedRealtime()
                if (clockWall < 0) { clockWall = now; clockPts = frame.ptsMs }
                val due = clockWall + (frame.ptsMs - clockPts)
                if (now > due + 2 * frameMs && k < frames.size - 1) continue   // behind: skip, never queue
                // Decode first, then wait for the frame's time: the decode overlaps the wait.
                val bitmap = decode(frame)
                val ready = SystemClock.elapsedRealtime()
                if (due > ready) delay(due - ready)
                show(frame, bitmap)
            }
            index++
        }
        stateMutable.update { it.copy(playing = false, positionMs = it.durationMs, status = "Ended") }
    }

    private fun show(frame: MkvFrame, decoded: Bitmap?) {
        val bitmap = decoded ?: return
        bitmap.prepareToDraw()
        pictureMutable.value = bitmap
        captionsMutable.value = script?.at(frame.ptsMs) ?: emptyList()
        stateMutable.update { it.copy(positionMs = frame.ptsMs) }
    }

    private fun decode(frame: MkvFrame): Bitmap? = try {
        val bounds = BitmapFactory.Options().apply { inJustDecodeBounds = true }
        BitmapFactory.decodeByteArray(frame.bytes, frame.offset, frame.length, bounds)
        if (bounds.outWidth !in 1..4096 || bounds.outHeight !in 1..4096) null
        else BitmapFactory.decodeByteArray(frame.bytes, frame.offset, frame.length,
            BitmapFactory.Options().apply { inSampleSize = decodeSampleSize(bounds.outWidth, bounds.outHeight) })
    } catch (_: Exception) { null }
}

/** What the player says after an error, and whether Play stays enabled (review 2026-11-27 #8). */
data class PlaybackFailure(val message: String, val terminal: Boolean)

/**
 * Unreadable media and the Jetson's refusals (404 deleted, 409 still recording, 401) are terminal. Anything
 * else is the transport (Wi-Fi drop, timeout, reset) and is recoverable: playback pauses at the last frame
 * shown and Play resumes from there. [started]: the header and Cues had loaded.
 */
internal fun playbackFailure(e: Exception, started: Boolean): PlaybackFailure = when (e) {
    is MkvFormatException -> PlaybackFailure("Cannot play this segment: ${e.message}", terminal = true)
    is RecordingsException -> PlaybackFailure(e.message ?: "Unavailable", terminal = true)
    else -> PlaybackFailure(if (started) "Connection lost · tap Play to resume" else "Could not reach the Jetson · tap Play to retry",
        terminal = false)
}

/** "3:00", "1:02:03". */
fun formatClock(ms: Long): String {
    val s = (ms / 1000).coerceAtLeast(0)
    return if (s >= 3600) String.format(Locale.US, "%d:%02d:%02d", s / 3600, s / 60 % 60, s % 60)
    else String.format(Locale.US, "%d:%02d", s / 60, s % 60)
}

/** "362 MB", "1.6 GB". */
fun formatBytes(bytes: Long): String = when {
    bytes >= 1_000_000_000 -> String.format(Locale.US, "%.1f GB", bytes / 1e9)
    bytes >= 1_000_000 -> String.format(Locale.US, "%.0f MB", bytes / 1e6)
    else -> String.format(Locale.US, "%.0f KB", bytes / 1e3)
}
