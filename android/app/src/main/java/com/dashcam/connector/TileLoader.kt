package com.dashcam.connector

import android.content.Context
import android.content.pm.PackageInfo
import android.content.pm.PackageManager
import android.graphics.BitmapFactory
import android.os.Build
import android.os.SystemClock
import android.util.LruCache
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.compose.ui.graphics.ImageBitmap
import androidx.compose.ui.graphics.asImageBitmap
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.CoroutineStart
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.suspendCancellableCoroutine
import kotlinx.coroutines.withContext
import okhttp3.Cache
import okhttp3.Call
import okhttp3.Callback
import okhttp3.Dispatcher
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import java.io.File
import java.io.IOException
import java.util.concurrent.TimeUnit
import kotlin.coroutines.resumeWithException

// OSM tiles for the Vehicle map, fetched directly from tile.openstreetmap.org.
// Its own OkHttpClient with the platform's default trust: the Jetson client pins
// the Jetson certificate and carries its credential, and is never used here.
private object TileHttp {
    @Volatile private var client: OkHttpClient? = null
    fun get(context: Context): OkHttpClient = client ?: synchronized(this) {
        client ?: build(context.applicationContext).also { client = it }
    }
    private fun build(context: Context): OkHttpClient {
        // OSM tile policy: identify the application.
        val agent = "DashcamConnect/${versionName(context)} (Android; com.dashcam.connector)"
        val dispatcher = Dispatcher().apply { maxRequests = 4; maxRequestsPerHost = 4 }
        return OkHttpClient.Builder()
            .cache(Cache(File(context.cacheDir, "osm_tiles"), 50L * 1024 * 1024))
            .dispatcher(dispatcher)
            .addInterceptor { chain ->
                chain.proceed(chain.request().newBuilder().header("User-Agent", agent).build())
            }
            .retryOnConnectionFailure(false)
            .connectTimeout(10, TimeUnit.SECONDS).readTimeout(10, TimeUnit.SECONDS)
            .writeTimeout(10, TimeUnit.SECONDS).callTimeout(10, TimeUnit.SECONDS)
            .build()
    }
    private fun versionName(context: Context): String {
        val name = try { packageInfo(context).versionName } catch (_: Exception) { null }
        // Header values must be printable ASCII.
        return name?.filter { it in '!'..'~' }?.take(32)?.ifEmpty { null } ?: "unknown"
    }
    @Suppress("DEPRECATION")
    private fun packageInfo(context: Context): PackageInfo =
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU)
            context.packageManager.getPackageInfo(context.packageName, PackageManager.PackageInfoFlags.of(0))
        else context.packageManager.getPackageInfo(context.packageName, 0)
}

/**
 * Loads the tiles the map currently shows. Main thread only: [request] replaces the wanted
 * set (cancelling loads that left the view), [pause] stops everything. Network on OkHttp's
 * threads (at most 4 at once), decoding on Dispatchers.Default. A failed tile is not
 * retried for [RETRY_MS]; decoded tiles live in a small in-memory LRU, the HTTP responses
 * in OkHttp's disk cache (server cache headers).
 */
class TileLoader(private val context: Context, private val scope: CoroutineScope) {
    private val bitmaps = LruCache<TileKey, ImageBitmap>(64)
    private val failedAt = HashMap<TileKey, Long>()
    private val loading = HashMap<TileKey, Job>()
    private var wanted: Set<TileKey> = emptySet()
    private var retry: Job? = null
    // Bumped whenever a tile arrives or fails; bitmap() reads it so a draw that
    // looked a tile up is redrawn when tiles change.
    private var version by mutableIntStateOf(0)
    /** True while a wanted tile failed recently (offline, blocked DNS, HTTP error). */
    var failing by mutableStateOf(false)
        private set

    /** A decoded tile if it is in memory; never starts a download. */
    fun bitmap(key: TileKey): ImageBitmap? = if (version >= 0) bitmaps.get(key) else null

    fun request(keys: Collection<TileKey>) {
        wanted = keys.take(TileMath.MAX_TILES).toSet()
        val gone = loading.keys.filter { it !in wanted }
        gone.forEach { loading.remove(it)?.cancel() }
        pump()
    }

    fun pause() {
        wanted = emptySet()
        loading.values.forEach { it.cancel() }
        loading.clear()
        retry?.cancel(); retry = null
        failing = false
    }

    fun close() = pause()

    private fun pump() {
        val now = SystemClock.elapsedRealtime()
        failedAt.entries.removeAll { now - it.value >= RETRY_MS }
        for (key in wanted) {
            if (key !in loading && key !in failedAt && bitmaps.get(key) == null) start(key)
        }
        failing = wanted.any { it in failedAt }
        scheduleRetry()
    }

    private fun start(key: TileKey) {
        val job = scope.launch(start = CoroutineStart.LAZY) {
            val image = try {
                val bytes = fetch(key)
                withContext(Dispatchers.Default) { decode(bytes) }
            } catch (e: CancellationException) {
                throw e
            } catch (_: Exception) {
                null
            }
            loading.remove(key)
            if (image != null) bitmaps.put(key, image)
            else {
                failedAt[key] = SystemClock.elapsedRealtime()
                scheduleRetry()
            }
            failing = wanted.any { it in failedAt }
            version++
        }
        loading[key] = job
        job.start()
    }

    // One timer for all failures: when the oldest wanted failure is RETRY_MS old, pump again.
    private fun scheduleRetry() {
        if (retry?.isActive == true) return
        val oldest = wanted.mapNotNull { failedAt[it] }.minOrNull() ?: return
        val wait = (oldest + RETRY_MS - SystemClock.elapsedRealtime()).coerceAtLeast(1000)
        retry = scope.launch {
            delay(wait)
            retry = null
            pump()
        }
    }

    private suspend fun fetch(key: TileKey): ByteArray {
        val client = withContext(Dispatchers.IO) { TileHttp.get(context) }
        return suspendCancellableCoroutine { continuation ->
            val call = client.newCall(Request.Builder().url(key.url).build())
            continuation.invokeOnCancellation { call.cancel() }
            call.enqueue(object : Callback {
                override fun onFailure(call: Call, e: IOException) {
                    if (continuation.isActive) continuation.resumeWithException(e)
                }
                override fun onResponse(call: Call, response: Response) {
                    val result: Result<ByteArray> = try {
                        response.use {
                            if (!it.isSuccessful) throw IOException("Tile HTTP ${it.code}")
                            val source = it.body?.source() ?: throw IOException("Empty tile")
                            if (source.request(MAX_TILE_BYTES + 1L)) throw IOException("Tile too large")
                            Result.success(source.readByteArray())
                        }
                    } catch (e: Exception) {
                        Result.failure(e)
                    }
                    if (continuation.isActive) continuation.resumeWith(result)
                }
            })
        }
    }

    private fun decode(bytes: ByteArray): ImageBitmap {
        val bitmap = BitmapFactory.decodeByteArray(bytes, 0, bytes.size) ?: throw IOException("Not an image")
        if (bitmap.width !in 1..1024 || bitmap.height !in 1..1024) throw IOException("Unexpected tile size")
        bitmap.prepareToDraw()
        return bitmap.asImageBitmap()
    }

    private companion object {
        const val RETRY_MS = 30_000L
        const val MAX_TILE_BYTES = 1L shl 20
    }
}
