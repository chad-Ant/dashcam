package com.dashcam.connector

import android.content.ContentValues
import android.content.Context
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.os.Build
import android.os.Environment
import android.os.SystemClock
import android.provider.MediaStore
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import okhttp3.HttpUrl
import okhttp3.OkHttpClient
import okhttp3.Request
import org.json.JSONObject
import java.io.Closeable
import java.io.File
import java.io.IOException
import java.io.OutputStream
import java.util.Locale

// Recorded segments over the direct LAN (connector/README.md "Recordings"): the connector lists them
// and serves the .mkv and .ass files by byte range; this app reads the Matroska itself and decodes
// each JPEG frame, so the Jetson only moves bytes.

data class RecordingSegment(val id: String, val seq: Int, val start: String, val durationS: Double?,
                            val bytes: Long, val captionBytes: Long?) {
    val day get() = start.take(10)
    val time get() = start.drop(11)
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
            val body = JSONObject(r.body!!.string())
            val list = body.getJSONArray("segments")
            return (0 until list.length()).map { i ->
                val j = list.getJSONObject(i)
                RecordingSegment(j.getString("id"), j.getInt("seq"), j.getString("start"),
                    if (j.isNull("duration_s")) null else j.getDouble("duration_s"), j.getLong("bytes"),
                    if (j.isNull("caption_bytes")) null else j.getLong("caption_bytes"))
            }
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

    /** Blocking: the whole file into [sink], reporting (done, total); stops when [cancelled] says so. */
    fun download(name: String, sink: OutputStream, cancelled: () -> Boolean, progress: (Long, Long) -> Unit) {
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
        }
    }

    override fun close() { client.connectionPool.evictAll(); client.dispatcher.executorService.shutdown() }
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
            var place = ""
            try {
                val files = listOfNotNull("${segment.id}.mkv", segment.captionBytes?.let { "${segment.id}.ass" })
                for (name in files) {
                    place = save(name) { out ->
                        source.download(name, out, { !isActive }) { done, total ->
                            downloadsMutable.update { it + (segment.id to DownloadState.Running(name, done, total)) }
                        }
                    }
                }
                downloadsMutable.update { it + (segment.id to DownloadState.Saved(place)) }
            } catch (e: CancellationException) { throw e
            } catch (e: Exception) {
                downloadsMutable.update { it + (segment.id to DownloadState.Failed(
                    (e as? RecordingsException)?.message ?: "Download failed · tap Download to retry")) }
            } finally { source.close() }
        }
    }

    /** Writes [name] to Download/Dashcam (MediaStore on Android 10+, the app's own folder on 9). */
    private fun save(name: String, write: (OutputStream) -> Unit): String {
        if (Build.VERSION.SDK_INT >= 29) {
            val resolver = context.contentResolver
            val values = ContentValues().apply {
                put(MediaStore.Downloads.DISPLAY_NAME, name)
                put(MediaStore.Downloads.MIME_TYPE, if (name.endsWith(".mkv")) "video/x-matroska" else "text/plain")
                put(MediaStore.Downloads.RELATIVE_PATH, Environment.DIRECTORY_DOWNLOADS + "/Dashcam")
                put(MediaStore.Downloads.IS_PENDING, 1)
            }
            val uri = resolver.insert(MediaStore.Downloads.EXTERNAL_CONTENT_URI, values)
                ?: throw IOException("Could not create the download")
            try {
                resolver.openOutputStream(uri)!!.use(write)
                values.clear(); values.put(MediaStore.Downloads.IS_PENDING, 0)
                resolver.update(uri, values, null, null)
            } catch (e: Throwable) { resolver.delete(uri, null, null); throw e }
            return "Download/Dashcam"
        }
        val dir = File(context.getExternalFilesDir(Environment.DIRECTORY_DOWNLOADS), "Dashcam").apply { mkdirs() }
        val file = File(dir, name)
        try { file.outputStream().use(write) } catch (e: Throwable) { file.delete(); throw e }
        return dir.absolutePath
    }
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
class RecordingPlayer(private val source: RecordingSource, val segment: RecordingSegment,
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
        job = scope.launch(Dispatchers.IO) {
            try {
                val header = source.range(name, 0, Mkv.HEADER_BYTES - 1L)
                val mkv = Mkv.parseHeader(header)
                clusters = Mkv.parseCues(source.range(name, mkv.cuesPosition, null), mkv)
                info = mkv
                if (segment.captionBytes != null) script = try {
                    AssScript.parse(String(source.range(segment.id + ".ass", 0, null), Charsets.UTF_8))
                } catch (e: CancellationException) { throw e } catch (_: Exception) { null }
                stateMutable.value = PlayerState("Playing", 0, mkv.durationMs, playing = true, buffering = true,
                    width = mkv.width, height = mkv.height, captionsAvailable = script != null)
                playFrom(0, autoplay = true)
            } catch (e: CancellationException) { throw e
            } catch (e: MkvFormatException) { fail("Cannot play this segment: ${e.message}")
            } catch (e: RecordingsException) { fail(e.message ?: "Unavailable")
            } catch (_: Exception) { fail("Could not reach the Jetson") }
        }
    }

    private fun fail(why: String) { stateMutable.update { it.copy(status = why, failed = true, playing = false, buffering = false) } }

    fun play() {
        val s = stateMutable.value
        if (info == null || s.failed) return
        playFrom(if (s.positionMs >= s.durationMs - 50) 0 else s.positionMs, autoplay = true)
    }

    fun pause() { job?.cancel(); stateMutable.update { it.copy(playing = false, buffering = false, status = "Paused") } }

    /** Jumps to [ms], keeping the playing/paused state; paused shows the frame there. */
    fun seek(ms: Long) {
        if (info == null) return
        playFrom(ms.coerceIn(0, stateMutable.value.durationMs), autoplay = stateMutable.value.playing)
    }

    fun close() { closed = true; job?.cancel(); pictureMutable.value = null; source.close() }

    private fun playFrom(fromMs: Long, autoplay: Boolean) {
        job?.cancel()
        if (closed) return
        stateMutable.update { it.copy(positionMs = fromMs, playing = autoplay, buffering = true,
            status = if (autoplay) "Playing" else "Paused") }
        job = scope.launch(Dispatchers.Default) {
            try { playback(fromMs, autoplay) }
            catch (e: CancellationException) { throw e }
            catch (e: MkvFormatException) { fail("Cannot play this segment: ${e.message}") }
            catch (e: RecordingsException) { fail(e.message ?: "Unavailable") }
            catch (_: Exception) { fail("Connection lost · tap Play to resume") }
        }
    }

    private suspend fun playback(fromMs: Long, autoplay: Boolean) = coroutineScope {
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
