package com.dashcam.connector

import android.content.Context
import android.net.ConnectivityManager
import android.net.Network
import android.net.NetworkCapabilities
import android.net.NetworkRequest
import android.os.SystemClock
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import okhttp3.*
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
    private var task: Job? = null
    private var socket: WebSocket? = null
    private var desired: DeviceProfile? = null
    private var active = false
    private var wifi: Network? = null
    private var registered = false
    private val generation = AtomicInteger()
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
                mutable.update { it.copy(nowMs = SystemClock.elapsedRealtime()) }
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
    fun close() {
        active = false
        generation.incrementAndGet()
        task?.cancel(); socket?.cancel()
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
    private suspend fun connect(device: DeviceProfile, network: Network, epoch: Int) = suspendCancellableCoroutine<Unit> { continuation ->
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
        val client = OkHttpClient.Builder().socketFactory(network.socketFactory)
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
    private class AuthenticationFailure : IOException()
}
