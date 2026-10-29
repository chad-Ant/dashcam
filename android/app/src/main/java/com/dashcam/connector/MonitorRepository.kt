package com.dashcam.connector

import android.content.Context
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.net.ConnectivityManager
import android.net.Network
import android.net.NetworkCapabilities
import android.net.NetworkRequest
import android.os.SystemClock
import androidx.compose.ui.graphics.asImageBitmap
import kotlinx.coroutines.*
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import okhttp3.*
import okio.ByteString
import java.io.IOException
import java.security.MessageDigest
import java.security.SecureRandom
import java.security.cert.X509Certificate
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicInteger
import javax.net.ssl.*
import kotlin.coroutines.resume

class MonitorRepository(context: Context, private val scope: CoroutineScope) {
    private val connectivity = context.getSystemService(ConnectivityManager::class.java)
    private val mutable = MutableStateFlow(MonitorState())
    val state = mutable.asStateFlow()
    private val videoMutable = MutableStateFlow(VideoState())
    val video = videoMutable.asStateFlow()
    private var task: Job? = null
    private var socket: WebSocket? = null
    private var videoTask: Job? = null
    private var videoSocket: WebSocket? = null
    private var desired: DeviceProfile? = null
    private var active = false
    private var videoWanted = false
    private var wifi: Network? = null
    private var registered = false
    private val generation = AtomicInteger()
    private val videoGeneration = AtomicInteger()
    private val callback = object : ConnectivityManager.NetworkCallback() {
        override fun onAvailable(network: Network) { scope.launch {
            if (wifi != network) { wifi = network; restart() }
        } }
        override fun onLost(network: Network) { scope.launch {
            if (wifi == network) { wifi = null; restart() }
        } }
    }
    init {
        scope.launch {
            while (isActive) {
                delay(500)
                val now = SystemClock.elapsedRealtime()
                mutable.update { it.copy(nowMs = now) }
                videoMutable.update { it.copy(nowMs = now) }
            }
        }
    }
    fun select(profile: DeviceProfile?) { desired = profile; restart() }
    fun setActive(value: Boolean) {
        active = value
        if (value && !registered) {
            try {
                connectivity.registerNetworkCallback(NetworkRequest.Builder().addTransportType(NetworkCapabilities.TRANSPORT_WIFI).build(), callback)
                registered = true
            } catch (_: SecurityException) {
                mutable.value = MonitorState(status = "Allow network access in Android settings")
            }
        }
        restart()
    }
    // The camera feed streams only while the Live page is shown; together with
    // setActive (foreground) and a selected device this gates /v1/video.
    fun setVideoWanted(value: Boolean) {
        if (videoWanted == value) return
        videoWanted = value
        restartVideo()
    }
    fun close() {
        active = false
        generation.incrementAndGet()
        videoGeneration.incrementAndGet()
        task?.cancel(); socket?.cancel()
        videoTask?.cancel(); videoSocket?.cancel()
        if (registered) connectivity.unregisterNetworkCallback(callback)
    }
    private fun restart() {
        val epoch = generation.incrementAndGet()
        task?.cancel(); socket?.cancel(); socket = null
        val device = desired
        val network = wifi
        mutable.value = MonitorState(status = when {
            !active -> "Disconnected"
            device == null -> "Choose or pair a Jetson"
            network == null -> "Waiting for Wi-Fi"
            else -> "Connecting"
        }, active = active)
        restartVideo()
        if (!active || device == null || network == null) return
        task = scope.launch {
            var backoff = 1L
            while (isActive) {
                try {
                    mutable.value = mutable.value.copy(status = "Connecting", lastReceiveMs = 0)
                    connect(device, network, epoch)
                    mutable.value = mutable.value.copy(status = "Connection lost · retrying", lastReceiveMs = 0)
                    backoff = 1
                } catch (e: CancellationException) { throw e
                } catch (_: SSLException) {
                    mutable.value = mutable.value.copy(status = "Certificate mismatch or expired certificate. Check pairing.", lastReceiveMs = 0)
                    return@launch
                } catch (_: SecurityException) {
                    mutable.value = mutable.value.copy(status = "Allow local network access in Android settings", lastReceiveMs = 0)
                    return@launch
                } catch (e: Exception) {
                    // Never show raw URLs/headers/exceptions containing credentials.
                    mutable.value = mutable.value.copy(status = if (e is AuthenticationFailure) "Pairing rejected. Check device credential." else "Connection lost · retrying", lastReceiveMs = 0)
                    if (e is AuthenticationFailure) return@launch
                }
                delay(backoff * 1000)
                backoff = (backoff * 2).coerceAtMost(15)
            }
        }
    }
    // One pinned client per connection: the exact leaf SHA-256 is the device
    // identity, independent of DHCP hostnames/IPs. Shared by both streams so
    // the pin and hostname checks exist exactly once.
    private fun pinnedClient(device: DeviceProfile, network: Network): OkHttpClient {
        val trust = object : X509TrustManager {
            override fun getAcceptedIssuers(): Array<X509Certificate> = emptyArray()
            override fun checkClientTrusted(chain: Array<X509Certificate>, authType: String) { throw java.security.cert.CertificateException("Client certificates unsupported") }
            override fun checkServerTrusted(chain: Array<X509Certificate>, authType: String) {
                val leaf = chain.firstOrNull() ?: throw java.security.cert.CertificateException("Missing certificate")
                leaf.checkValidity()
                val actual = MessageDigest.getInstance("SHA-256").digest(leaf.encoded)
                val expected = device.pin.chunked(2).map { it.toInt(16).toByte() }.toByteArray()
                if (!MessageDigest.isEqual(actual, expected)) throw java.security.cert.CertificateException("Certificate mismatch")
            }
        }
        val ssl = SSLContext.getInstance("TLS").apply { init(null, arrayOf(trust), SecureRandom()) }
        return OkHttpClient.Builder().socketFactory(network.socketFactory)
            .dns(object : Dns {
                override fun lookup(hostname: String) = network.getAllByName(hostname).toList()
            })
            .sslSocketFactory(ssl.socketFactory, trust)
            // Exact leaf pin above is the device identity, independent of DHCP
            // hostnames/IPs. Never allow an unpinned certificate.
            .hostnameVerifier { _, session ->
                val leaf = session.peerCertificates.firstOrNull() as? X509Certificate
                leaf != null && MessageDigest.getInstance("SHA-256").digest(leaf.encoded)
                    .joinToString("") { "%02x".format(it) } == device.pin
            }
            .followRedirects(false).followSslRedirects(false).retryOnConnectionFailure(false)
            .connectTimeout(5, TimeUnit.SECONDS).readTimeout(15, TimeUnit.SECONDS)
            .pingInterval(5, TimeUnit.SECONDS).build()
    }
    private suspend fun connect(device: DeviceProfile, network: Network, epoch: Int) = suspendCancellableCoroutine<Unit> { continuation ->
        val client = pinnedClient(device, network)
        fun cleanup() { client.connectionPool.evictAll(); client.dispatcher.executorService.shutdown() }
        val url = HttpUrl.Builder().scheme("https").host(device.host).port(device.port)
            .addPathSegments("v1/stream").build()
        var session = ""
        var sequence = -1L
        val ws = client.newWebSocket(Request.Builder().url(url).header("Authorization", "Bearer ${device.token}").build(),
            object : WebSocketListener() {
                override fun onMessage(webSocket: WebSocket, text: String) {
                    if (!continuation.isActive) return
                    try {
                        val data = Snapshot.parse(text, device.id)
                        val nextSession = data.json.getString("session")
                        val nextSeq = data.json.getLong("seq")
                        if (nextSession == session && nextSeq <= sequence) return
                        session = nextSession; sequence = nextSeq
                        val now = SystemClock.elapsedRealtime()
                        mutable.update { previous ->
                            if (generation.get() != epoch) previous else
                                MonitorState(if (data.runtimeFresh) "Live" else "Jetson online · runtime unavailable", data, now, now, true)
                        }
                    } catch (_: Exception) {
                        webSocket.close(1002, "Unsupported or invalid monitoring data")
                    }
                }
                override fun onClosing(webSocket: WebSocket, code: Int, reason: String) { webSocket.close(code, null) }
                override fun onClosed(webSocket: WebSocket, code: Int, reason: String) {
                    cleanup()
                    if (continuation.isActive) continuation.resume(Unit)
                }
                override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
                    cleanup()
                    if (continuation.isActive) continuation.resumeWith(Result.failure(
                        if (response?.code == 401) AuthenticationFailure() else t))
                }
            })
        socket = ws
        continuation.invokeOnCancellation { ws.cancel(); cleanup() }
    }
    // Every status change is a connection change (connecting, open, closed, failed):
    // the previous connection's frame is dropped so it is never shown as current.
    private fun videoStatus(epoch: Int, text: String, connected: Boolean = false) {
        videoMutable.update { previous ->
            if (videoGeneration.get() != epoch) previous else previous.copy(status = text, frame = null, connected = connected)
        }
    }
    private fun restartVideo() {
        val epoch = videoGeneration.incrementAndGet()
        videoTask?.cancel(); videoSocket?.cancel(); videoTask = null; videoSocket = null
        val device = desired
        val network = wifi
        // Dropping the frame here means a closed feed never shows an old picture.
        videoMutable.value = VideoState(status = when {
            !active || !videoWanted -> "Video paused"
            device == null -> "Choose or pair a Jetson"
            network == null -> "Waiting for Wi-Fi"
            else -> "Connecting"
        }, nowMs = SystemClock.elapsedRealtime())
        if (!active || !videoWanted || device == null || network == null) return
        videoTask = scope.launch {
            // Conflated: while a JPEG is decoding only the newest pending frame is kept.
            val frames = Channel<EncodedFrame>(Channel.CONFLATED)
            launch(Dispatchers.Default) { decodeFrames(frames, epoch) }
            try {
                var backoff = 1L
                while (isActive) {
                    try {
                        videoStatus(epoch, "Connecting")
                        connectVideo(device, network, epoch, frames)
                        videoStatus(epoch, "Video connection lost · retrying")
                        backoff = 1
                    } catch (e: CancellationException) { throw e
                    } catch (_: SSLException) {
                        videoStatus(epoch, "Certificate mismatch or expired certificate. Check pairing.")
                        return@launch
                    } catch (_: SecurityException) {
                        videoStatus(epoch, "Allow local network access in Android settings")
                        return@launch
                    } catch (e: Exception) {
                        // Never show raw URLs/headers/exceptions containing credentials.
                        val code = (e as? VideoHttpFailure)?.code
                        videoStatus(epoch, when (code) {
                            401 -> "Pairing rejected. Check device credential."
                            404 -> "This Jetson's connector has no video"
                            503 -> "Too many video viewers · retrying"
                            else -> "Video connection lost · retrying"
                        })
                        if (code == 401 || code == 404) return@launch
                    }
                    delay(backoff * 1000)
                    backoff = (backoff * 2).coerceAtMost(15)
                }
            } finally {
                frames.close()
            }
        }
    }
    private suspend fun connectVideo(device: DeviceProfile, network: Network, epoch: Int, frames: Channel<EncodedFrame>) = suspendCancellableCoroutine<Unit> { continuation ->
        val client = pinnedClient(device, network)
        fun cleanup() { client.connectionPool.evictAll(); client.dispatcher.executorService.shutdown() }
        val url = HttpUrl.Builder().scheme("https").host(device.host).port(device.port)
            .addPathSegments("v1/video").build()
        var sequence = 0L
        val ws = client.newWebSocket(Request.Builder().url(url).header("Authorization", "Bearer ${device.token}").build(),
            object : WebSocketListener() {
                override fun onOpen(webSocket: WebSocket, response: Response) {
                    videoStatus(epoch, "Waiting for camera frames", connected = true)
                }
                override fun onMessage(webSocket: WebSocket, bytes: ByteString) {
                    if (!continuation.isActive) return
                    val data = bytes.toByteArray()
                    val header = VideoHeader.parse(data) ?: return
                    // Agent sequence is an unsigned 64-bit counter starting at 1.
                    if (java.lang.Long.compareUnsigned(header.sequence, sequence) <= 0) return
                    sequence = header.sequence
                    frames.trySend(EncodedFrame(header, data))
                }
                override fun onClosing(webSocket: WebSocket, code: Int, reason: String) { webSocket.close(code, null) }
                override fun onClosed(webSocket: WebSocket, code: Int, reason: String) {
                    cleanup()
                    if (continuation.isActive) continuation.resume(Unit)
                }
                override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
                    cleanup()
                    val code = response?.code ?: 0
                    if (continuation.isActive) continuation.resumeWith(Result.failure(
                        if (code == 401 || code == 404 || code == 503) VideoHttpFailure(code) else t))
                }
            })
        videoSocket = ws
        continuation.invokeOnCancellation { ws.cancel(); cleanup() }
    }
    // Runs on Dispatchers.Default: BitmapFactory never touches the main thread.
    private suspend fun decodeFrames(frames: Channel<EncodedFrame>, epoch: Int) {
        var lastAt = 0L
        var interval = 0.0
        for (frame in frames) {
            val bitmap = decodeJpeg(frame.bytes, frame.header.headerLength) ?: continue
            bitmap.prepareToDraw()
            val image = bitmap.asImageBitmap()
            val now = SystemClock.elapsedRealtime()
            val gap = (now - lastAt).toDouble()
            interval = when {
                lastAt == 0L || gap > 2000 -> 0.0
                interval == 0.0 -> gap
                else -> interval * 0.8 + gap * 0.2
            }
            lastAt = now
            val fps = if (interval > 0) 1000.0 / interval else null
            val shown = VideoFrame(image, frame.header.width, frame.header.height, frame.header.ageMs, now, fps)
            videoMutable.update { previous ->
                // A decode that finishes after its socket closed is not shown.
                if (videoGeneration.get() != epoch || !previous.connected) previous
                else previous.copy(status = "Live", frame = shown, nowMs = now)
            }
        }
    }
    private fun decodeJpeg(data: ByteArray, offset: Int): Bitmap? {
        val length = data.size - offset
        if (length < 4 || data[offset] != 0xFF.toByte() || data[offset + 1] != 0xD8.toByte()) return null
        return try {
            val bounds = BitmapFactory.Options().apply { inJustDecodeBounds = true }
            BitmapFactory.decodeByteArray(data, offset, length, bounds)
            if (bounds.outWidth !in 1..4096 || bounds.outHeight !in 1..4096) return null
            var sample = 1
            while (maxOf(bounds.outWidth, bounds.outHeight) / sample > 1280) sample *= 2
            BitmapFactory.decodeByteArray(data, offset, length, BitmapFactory.Options().apply { inSampleSize = sample })
        } catch (_: Exception) { null }
    }
    private class EncodedFrame(val header: VideoHeader, val bytes: ByteArray)
    private class VideoHttpFailure(val code: Int) : IOException()
    private class AuthenticationFailure : IOException()
}
